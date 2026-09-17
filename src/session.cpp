#include "session.h"
#include "graph.h"
#include "tree.h"
#include "intersection.h"
#include "tolerance.h"
#include "session.pb.h"
#include <algorithm>
#include <limits>
#include <map>

namespace session_cpp {

namespace {

/// Calls f on the Objects list of that name, the C++ spelling of getattr(objects, collection).
template <typename F> void with_collection(const Objects& objects, const std::string& collection, F&& f) {

    if (collection == "points")
        f(*objects.points);
    else if (collection == "lines")
        f(*objects.lines);
    else if (collection == "planes")
        f(*objects.planes);
    else if (collection == "bboxes")
        f(*objects.bboxes);
    else if (collection == "polylines")
        f(*objects.polylines);
    else if (collection == "pointclouds")
        f(*objects.pointclouds);
    else if (collection == "meshes")
        f(*objects.meshes);
    else if (collection == "nurbscurves")
        f(*objects.nurbscurves);
    else if (collection == "nurbssurfaces")
        f(*objects.nurbssurfaces);
    else if (collection == "breps")
        f(*objects.breps);
    else if (collection == "elements")
        f(*objects.elements);
    else if (collection == "components")
        f(*objects.components);
}

/// The guid of an Objects list element, a shared_ptr or a Component.
template <typename E> std::string guid_of(const E& element) {

    if constexpr (std::is_same_v<E, Component>)
        return element.guid();
    else
        return element->guid();
}

/// The guid of an item, geometry or component.
std::string item_guid(const Item& item) {

    if (const Geometry* geometry = std::get_if<Geometry>(&item))
        return std::visit(
            [](const auto& live) {
                return live->guid();
            },
            *geometry
        );

    return std::get<Component>(item).guid();
}

/// The name of an item, geometry or component.
std::string item_name(const Item& item) {

    if (const Geometry* geometry = std::get_if<Geometry>(&item))
        return std::visit(
            [](const auto& live) {
                return live->name;
            },
            *geometry
        );

    return std::get<Component>(item).name;
}

/// The element an Objects list of value type E stores for an item.
template <typename E> E element_of(const Item& item) {

    if constexpr (std::is_same_v<E, Component>)
        return std::get<Component>(item);
    else
        return std::get<E>(std::get<Geometry>(item));
}

/// Transforms every object of a list by its world placement, identity entries untouched.
template <typename T>
void bake(std::vector<std::shared_ptr<T>>& items, const std::unordered_map<std::string, Xform>& world) {

    for (std::shared_ptr<T>& item : items) {
        auto it = world.find(item->guid());

        if (it == world.end() || it->second.is_identity())
            continue;

        item->transform(it->second);
    }
}

/// Inflated box around the placed points, around the origin when there are none.
OBB placed_box(const std::vector<Point>& points, const Xform& xform, double inflate) {

    if (points.empty())
        return OBB::from_point(Point(0, 0, 0), inflate);

    std::vector<Point> placed;
    placed.reserve(points.size());

    for (const Point& point : points)
        placed.push_back(xform.transform_point(point));

    return OBB::from_points(placed, inflate);
}

/// The point on the ray closest to point when it lies ahead and within tolerance.
std::optional<Point> ray_point(const Line& ray, const Point& point, double tolerance) {

    const Vector ray_dir = ray.end() - ray.start();
    const Vector to_point = point - ray.start();
    const double t = to_point.dot(ray_dir) / ray_dir.dot(ray_dir);

    if (t < 0)
        return std::nullopt;

    const Point closest = ray.start() + ray_dir * t;

    if (point.distance(closest) > tolerance)
        return std::nullopt;

    return closest;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

Session::Session(std::string name)
    : name(std::move(name)), objects(), tree(this->name + "_tree"), graph(this->name + "_graph") {

    tree.add(std::make_shared<TreeNode>(this->name));
}

Session::Session(const Session& other)
    : name(other.name), objects(other.objects), tree(other.tree), graph(other.graph), xforms(other.xforms) {

    if (other.has_guid())
        guid() = other.guid();

    _index_objects();
    bvh_cache_dirty = true;
}

Session& Session::operator=(const Session& other) {

    if (this != &other) {
        Session copy(other);
        *this = std::move(copy);
    }

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════

std::shared_ptr<TreeNode> Session::find_group(const std::string& group_name) const {

    std::shared_ptr<TreeNode> root = tree.root();

    if (root)
        for (TreeNode* child : root->children())
            if (child && child->name == group_name)
                return child->shared_from_this();

    throw std::runtime_error("Group '" + group_name + "' not found");
}

std::vector<std::string> Session::order() const {

    std::vector<std::string> order;
    order.reserve(lookup.size());

    for (const std::shared_ptr<Point>& point : *objects.points)
        order.push_back(point->guid());

    for (const std::shared_ptr<Line>& line : *objects.lines)
        order.push_back(line->guid());

    for (const std::shared_ptr<Plane>& plane : *objects.planes)
        order.push_back(plane->guid());

    for (const std::shared_ptr<OBB>& bbox : *objects.bboxes)
        order.push_back(bbox->guid());

    for (const std::shared_ptr<Polyline>& polyline : *objects.polylines)
        order.push_back(polyline->guid());

    for (const std::shared_ptr<PointCloud>& pointcloud : *objects.pointclouds)
        order.push_back(pointcloud->guid());

    for (const std::shared_ptr<Mesh>& mesh : *objects.meshes)
        order.push_back(mesh->guid());

    for (const std::shared_ptr<NurbsCurve>& nurbscurve : *objects.nurbscurves)
        order.push_back(nurbscurve->guid());

    for (const std::shared_ptr<NurbsSurface>& nurbssurface : *objects.nurbssurfaces)
        order.push_back(nurbssurface->guid());

    for (const std::shared_ptr<BRep>& brep : *objects.breps)
        order.push_back(brep->guid());

    for (const std::shared_ptr<Element>& element : *objects.elements)
        order.push_back(element->guid());

    return order;
}

Xform Session::xform(const std::string& guid) const {
    auto it = xforms.find(guid);

    return it == xforms.end() ? Xform::identity() : it->second;
}

Xform Session::world_xform(const std::string& guid) const {

    Xform acc = xform(guid);
    std::shared_ptr<TreeNode> node = tree.get_node_by_name(guid);

    if (!node)
        return acc;

    for (TreeNode* ancestor : node->ancestors()) {
        auto it = xforms.find(ancestor->name);

        if (it != xforms.end())
            acc = it->second * acc;
    }

    return acc;
}

std::unordered_map<std::string, Xform> Session::world_xforms() const {

    std::unordered_map<std::string, Xform> out;

    if (xforms.empty())
        return out;

    std::vector<std::pair<TreeNode*, Xform>> stack;

    if (tree.root())
        stack.emplace_back(tree.root().get(), Xform::identity());

    while (!stack.empty()) {
        auto [node, parent_xform] = stack.back();
        stack.pop_back();
        auto it = xforms.find(node->name);
        const Xform current = it == xforms.end() ? parent_xform : parent_xform * it->second;
        out[node->name] = current;

        for (TreeNode* child : node->children())
            stack.emplace_back(child, current);
    }

    for (const auto& [obj_guid, obj_xform] : xforms)
        out.emplace(obj_guid, obj_xform);

    return out;
}

std::vector<std::string> Session::get_children(const std::string& obj_guid) const {
    return tree.get_children_guids(obj_guid);
}

std::vector<std::string> Session::get_neighbours(const std::string& obj_guid) {
    return graph.neighbors(obj_guid);
}

Objects Session::get_geometry() const {

    Objects out(objects);
    const std::unordered_map<std::string, Xform> world = world_xforms();
    bake(*out.points, world);
    bake(*out.lines, world);
    bake(*out.planes, world);
    bake(*out.bboxes, world);
    bake(*out.polylines, world);
    bake(*out.pointclouds, world);
    bake(*out.meshes, world);
    bake(*out.nurbscurves, world);
    bake(*out.nurbssurfaces, world);
    bake(*out.breps, world);

    for (std::shared_ptr<Element>& element : *out.elements) {
        auto it = world.find(element->guid());

        if (it == world.end() || it->second.is_identity())
            continue;

        element->place(it->second);
    }

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry management
// ═══════════════════════════════════════════════════════════════════════════

std::shared_ptr<TreeNode> Session::add_point(std::shared_ptr<Point> point, std::shared_ptr<TreeNode> parent) {
    if (!point)
        return nullptr;

    return _add_object("points", point, "point", parent);
}

std::shared_ptr<TreeNode> Session::add_line(std::shared_ptr<Line> line, std::shared_ptr<TreeNode> parent) {
    if (!line)
        return nullptr;

    return _add_object("lines", line, "line", parent);
}

std::shared_ptr<TreeNode> Session::add_plane(std::shared_ptr<Plane> plane, std::shared_ptr<TreeNode> parent) {
    if (!plane)
        return nullptr;

    return _add_object("planes", plane, "plane", parent);
}

std::shared_ptr<TreeNode> Session::add_obb(std::shared_ptr<OBB> bbox) {
    if (!bbox)
        return nullptr;

    return _add_object("bboxes", bbox, "bbox", nullptr);
}

std::shared_ptr<TreeNode> Session::add_polyline(std::shared_ptr<Polyline> polyline, std::shared_ptr<TreeNode> parent) {
    if (!polyline || polyline->point_count() < 2)
        return nullptr;

    return _add_object("polylines", polyline, "polyline", parent);
}

std::shared_ptr<TreeNode> Session::add_pointcloud(
    std::shared_ptr<PointCloud> pointcloud,
    std::shared_ptr<TreeNode> parent
) {
    if (!pointcloud || pointcloud->is_empty())
        return nullptr;

    return _add_object("pointclouds", pointcloud, "pointcloud", parent);
}

std::shared_ptr<TreeNode> Session::add_mesh(std::shared_ptr<Mesh> mesh, std::shared_ptr<TreeNode> parent) {
    if (!mesh || mesh->is_empty() || mesh->number_of_faces() == 0)
        return nullptr;

    return _add_object("meshes", mesh, "mesh", parent);
}

std::shared_ptr<TreeNode> Session::add_nurbscurve(
    std::shared_ptr<NurbsCurve> nurbscurve,
    std::shared_ptr<TreeNode> parent
) {
    if (!nurbscurve || nurbscurve->cv_count() < 2)
        return nullptr;

    return _add_object("nurbscurves", nurbscurve, "nurbscurve", parent);
}

std::shared_ptr<TreeNode> Session::add_nurbssurface(
    std::shared_ptr<NurbsSurface> nurbssurface,
    std::shared_ptr<TreeNode> parent
) {
    if (!nurbssurface || nurbssurface->cv_count() == 0)
        return nullptr;

    return _add_object("nurbssurfaces", nurbssurface, "nurbssurface", parent);
}

std::shared_ptr<TreeNode> Session::add_brep(std::shared_ptr<BRep> brep, std::shared_ptr<TreeNode> parent) {
    if (!brep || (brep->face_count() == 0 && brep->vertex_count() == 0))
        return nullptr;

    return _add_object("breps", brep, "brep", parent);
}

std::shared_ptr<TreeNode> Session::add_element(std::shared_ptr<Element> element, std::shared_ptr<TreeNode> parent) {
    if (!element)
        return nullptr;

    return _add_object("elements", element, "element", parent);
}

std::shared_ptr<TreeNode> Session::add_component(Component component, std::shared_ptr<TreeNode> parent) {
    return _add_object("components", component, "component", parent);
}

void Session::add(std::shared_ptr<TreeNode> node, std::shared_ptr<TreeNode> parent) {

    if (node == nullptr)
        return;

    if (parent == nullptr)
        tree.add(node, tree.root());
    else
        tree.add(node, parent);
}

std::shared_ptr<TreeNode> Session::add_group(const std::string& group_name) {
    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(group_name);
    add(node);

    return node;
}

void Session::add_edge(const std::string& guid1, const std::string& guid2, const std::string& attribute) {
    graph.add_edge(guid1, guid2, attribute);
}

bool Session::add_hierarchy(const std::string& parent_guid, const std::string& child_guid) {
    return tree.add_child_by_guid(parent_guid, child_guid);
}

void Session::add_relationship(
    const std::string& from_guid,
    const std::string& to_guid,
    const std::string& relationship_type
) {
    graph.add_edge(from_guid, to_guid, relationship_type);
}

bool Session::remove_object(const std::string& obj_guid) {

    std::optional<RemoveOp> op = _detach(obj_guid);

    if (!op)
        return false;

    history.record(*op);

    return true;
}

bool Session::replace(const std::string& guid, const Geometry& obj) {

    auto before = lookup.find(guid);

    if (before == lookup.end())
        return false;

    std::visit(
        [&](const auto& live) {
            live->guid() = guid;
        },
        obj
    );

    if (history.current)
        history.record(ReplaceOp(guid, clone(before->second), clone(obj)));

    _swap(guid, obj);

    return true;
}

void Session::set_xform(const std::string& guid, const Xform& xform) {

    if (history.current) {
        std::optional<Xform> before;
        auto it = xforms.find(guid);

        if (it != xforms.end())
            before = it->second;

        history.record(XformOp(guid, before, xform));
    }

    xforms[guid] = xform;
    bvh_cache_dirty = true;
}

bool Session::remove_xform(const std::string& guid) {

    auto before = xforms.find(guid);

    if (before == xforms.end())
        return false;

    if (history.current)
        history.record(XformOp(guid, before->second, std::nullopt));

    xforms.erase(before);
    bvh_cache_dirty = true;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════

void Session::begin(const std::string& label) {
    history.begin(label);
}

void Session::commit() {
    history.commit();
}

bool Session::undo() {
    return history.undo(*this);
}

bool Session::redo() {
    return history.redo(*this);
}

// ═══════════════════════════════════════════════════════════════════════════
// Collision detection and ray casting
// ═══════════════════════════════════════════════════════════════════════════

OBB Session::compute_bounding_box(const Geometry& geometry, const Xform& xform) {

    const double inflate = Tolerance::APPROXIMATION;

    if (const std::shared_ptr<Point>* point = std::get_if<std::shared_ptr<Point>>(&geometry))
        return OBB::from_point(xform.transform_point(**point), inflate);

    if (const std::shared_ptr<Plane>* plane = std::get_if<std::shared_ptr<Plane>>(&geometry))
        return OBB::from_point(xform.transform_point((*plane)->origin()), inflate * 10.0);

    if (const std::shared_ptr<OBB>* bbox = std::get_if<std::shared_ptr<OBB>>(&geometry)) {
        OBB inflated = **bbox;
        inflated.half_size = inflated.half_size + Vector(inflate, inflate, inflate);
        inflated.transform(xform);

        return inflated;
    }

    if (const std::shared_ptr<Element>* element = std::get_if<std::shared_ptr<Element>>(&geometry)) {
        Element copy = **element;
        OBB box = copy.aabb();
        box.transform(xform);

        return box;
    }

    std::vector<Point> points;

    if (const std::shared_ptr<Line>* line = std::get_if<std::shared_ptr<Line>>(&geometry)) {
        points.push_back((*line)->start());
        points.push_back((*line)->end());
    } else if (const std::shared_ptr<Polyline>* polyline = std::get_if<std::shared_ptr<Polyline>>(&geometry)) {
        points = (*polyline)->get_points();
    } else if (const std::shared_ptr<PointCloud>* pointcloud = std::get_if<std::shared_ptr<PointCloud>>(&geometry)) {
        points = (*pointcloud)->get_points();
    } else if (const std::shared_ptr<Mesh>* mesh = std::get_if<std::shared_ptr<Mesh>>(&geometry)) {
        for (const auto& [key, vertex] : (*mesh)->vertex)
            points.push_back(vertex.position());
    } else if (const std::shared_ptr<BRep>* brep = std::get_if<std::shared_ptr<BRep>>(&geometry)) {
        for (const BRepVertex& vertex : (*brep)->m_vertices)
            points.push_back(vertex.point);

        for (const NurbsSurface& surface : (*brep)->m_surfaces) {
            const auto [u0, u1] = surface.domain(0);
            const auto [v0, v1] = surface.domain(1);

            for (int i = 0; i <= 2; ++i)
                for (int j = 0; j <= 2; ++j)
                    points.push_back(surface.point_at(u0 + (u1 - u0) * i / 2.0, v0 + (v1 - v0) * j / 2.0));
        }
    } else if (const std::shared_ptr<NurbsCurve>* nurbscurve = std::get_if<std::shared_ptr<NurbsCurve>>(&geometry)) {
        for (int i = 0; i < (*nurbscurve)->cv_count(); ++i)
            points.push_back((*nurbscurve)->get_cv(i));
    } else if (
        const std::shared_ptr<NurbsSurface>* nurbssurface = std::get_if<std::shared_ptr<NurbsSurface>>(&geometry)
    ) {
        for (int i = 0; i < (*nurbssurface)->cv_count(0); ++i)
            for (int j = 0; j < (*nurbssurface)->cv_count(1); ++j)
                points.push_back((*nurbssurface)->get_cv(i, j));
    }

    return placed_box(points, xform, inflate);
}

std::vector<std::pair<std::string, std::string>> Session::get_collisions() {

    std::vector<std::string> guids;
    const std::vector<OBB> boxes = _compute_boxes(guids);

    if (boxes.empty())
        return {};

    bvh = SpatialBVH::from_boxes(boxes, SpatialBVH::compute_world_size(boxes));
    const auto [pairs, colliding, checks] = bvh.check_all_collisions(boxes);
    std::vector<std::pair<std::string, std::string>> guid_pairs;
    guid_pairs.reserve(pairs.size());

    for (const auto& [i, j] : pairs) {
        if (i < 0 || j < 0 || i >= static_cast<int>(guids.size()) || j >= static_cast<int>(guids.size()))
            continue;

        guid_pairs.emplace_back(guids[i], guids[j]);
        graph.add_edge(guids[i], guids[j], "bvh_collision");
    }

    return guid_pairs;
}

std::vector<Session::RayHit> Session::ray_cast(const Point& origin, const Vector& direction, double tolerance) {

    if (bvh_cache_dirty) {
        _rebuild_ray_bvh_cache();
        bvh_cache_dirty = false;
    }

    if (cached_guids.empty())
        return {};

    std::vector<int> candidates;
    cached_ray_bvh.ray_cast(origin, direction, candidates, true);
    const Line ray = Line::from_points(origin, origin + direction * 10000.0);
    const std::unordered_map<std::string, Xform> world = world_xforms();
    std::vector<RayHit> hits;
    double closest = std::numeric_limits<double>::infinity();

    for (int index : candidates) {
        const std::string& guid = cached_guids[index];
        auto it = lookup.find(guid);

        if (it == lookup.end())
            continue;

        auto wit = world.find(guid);
        const Xform placement = wit == world.end() ? Xform::identity() : wit->second;
        const std::optional<Point> hit = _ray_intersect_geometry(ray, it->second, tolerance, placement);

        if (!hit)
            continue;

        const double distance = origin.distance(*hit);

        if (distance >= closest)
            continue;

        if (distance < closest - tolerance)
            hits.clear();

        hits.push_back({guid, *hit, distance});
        closest = distance;
    }

    return hits;
}

// ═══════════════════════════════════════════════════════════════════════════
// Serialization
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Session::jsondump() const {

    nlohmann::ordered_json data;
    data["type"] = "Session";
    data["name"] = name;
    data["guid"] = guid();
    data["objects"] = objects.jsondump();
    data["tree"] = tree.jsondump();
    data["graph"] = graph.jsondump();
    nlohmann::ordered_json xforms_json = nlohmann::ordered_json::array();

    for (const auto& [obj_guid, obj_xform] : _xforms_ordered()) {
        nlohmann::ordered_json entry;
        entry["guid"] = obj_guid;
        entry["xform"] = obj_xform.jsondump();
        xforms_json.push_back(entry);
    }

    data["xforms"] = xforms_json;

    return data;
}

Session Session::jsonload(const nlohmann::json& data) {

    Session session(data.value("name", "my_session"));

    if (data.contains("guid"))
        session.guid() = data["guid"].get<std::string>();

    if (data.contains("objects"))
        session.objects = Objects::jsonload(data["objects"]);

    if (data.contains("tree"))
        session.tree = Tree::jsonload(data["tree"]);

    if (data.contains("graph"))
        session.graph = Graph::jsonload(data["graph"]);

    session._index_objects();

    if (data.contains("xforms"))
        for (const nlohmann::json& entry : data["xforms"])
            session.xforms[entry["guid"].get<std::string>()] = Xform::jsonload(entry["xform"]);

    return session;
}

std::string Session::file_json_dumps() const {
    history.clear();

    return jsondump().dump();
}

Session Session::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Session::file_json_dump(const std::string& filename) const {
    history.clear();
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Session Session::file_json_load(const std::string& filename) {
    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

std::string Session::pb_dumps() const {

    history.clear();
    session_proto::Session proto;
    proto.set_name(name);

    if (has_guid())
        proto.set_guid(guid());

    proto.mutable_objects()->ParseFromString(objects.pb_dumps());
    proto.mutable_tree()->ParseFromString(tree.pb_dumps());
    proto.mutable_graph()->ParseFromString(graph.pb_dumps());

    for (const auto& [obj_guid, obj_xform] : _xforms_ordered()) {
        session_proto::XformEntry* entry = proto.add_xforms();
        entry->set_guid(obj_guid);
        entry->mutable_xform()->ParseFromString(obj_xform.pb_dumps());
    }

    return proto.SerializeAsString();
}

Session Session::pb_loads(const std::string& data) {

    session_proto::Session proto;
    proto.ParseFromString(data);
    Session session(proto.name());

    if (!proto.guid().empty())
        session.guid() = proto.guid();

    if (proto.has_objects())
        session.objects = Objects::pb_loads(proto.objects().SerializeAsString());

    if (proto.has_tree())
        session.tree = Tree::pb_loads(proto.tree().SerializeAsString());

    if (proto.has_graph())
        session.graph = Graph::pb_loads(proto.graph().SerializeAsString());

    session._index_objects();

    for (const session_proto::XformEntry& entry : proto.xforms())
        session.xforms[entry.guid()] = Xform::pb_loads(entry.xform().SerializeAsString());

    return session;
}

void Session::pb_dump(const std::string& filename) const {

    history.clear();
    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Session Session::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

std::string Session::str() const {
    return fmt::format("Session(name={}, objects={}, tree={}, graph={})", name, objects.str(), tree.str(), graph.str());
}

// ═══════════════════════════════════════════════════════════════════════════
// Details
// ═══════════════════════════════════════════════════════════════════════════

std::shared_ptr<TreeNode> Session::_add_object(
    const std::string& collection,
    const Item& obj,
    const std::string& type_prefix,
    std::shared_ptr<TreeNode> parent
) {

    const std::string guid = item_guid(obj);
    int obj_index = 0;
    with_collection(objects, collection, [&](auto& items) {
        items.push_back(element_of<typename std::decay_t<decltype(items)>::value_type>(obj));
        obj_index = static_cast<int>(items.size()) - 1;
    });

    if (const Geometry* geometry = std::get_if<Geometry>(&obj))
        lookup[guid] = *geometry;
    else
        component_lookup[guid] = std::get<Component>(obj);

    const std::string attribute = type_prefix + "_" + item_name(obj);
    graph.add_node(guid, attribute);
    bvh_cache_dirty = true;
    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(guid);
    std::optional<std::string> parent_guid;
    int index = 0;

    if (parent) {
        add(node, parent);
        parent_guid = parent->name;
        index = static_cast<int>(parent->children().size()) - 1;
    }

    if (history.current)
        history.record(
            AddOp(guid, clone(obj), collection, obj_index, std::nullopt, parent_guid, index, nullptr, attribute, {})
        );

    return node;
}

std::pair<std::string, int> Session::_locate(const std::string& guid) const {

    for (const auto& [collection, prefix] : COLLECTIONS) {
        int found = -1;
        with_collection(objects, collection, [&](const auto& items) {
            for (size_t i = 0; i < items.size(); ++i)
                if (guid_of(items[i]) == guid)
                    found = static_cast<int>(i);
        });

        if (found >= 0)
            return {collection, found};
    }

    return {"", -1};
}

std::optional<RemoveOp> Session::_detach(const std::string& guid) {

    std::optional<Item> obj;

    if (auto it = lookup.find(guid); it != lookup.end())
        obj = it->second;
    else if (auto it = component_lookup.find(guid); it != component_lookup.end())
        obj = it->second;

    if (!obj)
        return std::nullopt;

    const auto [collection, obj_index] = _locate(guid);

    if (obj_index >= 0)
        with_collection(objects, collection, [&](auto& items) {
            items.erase(items.begin() + obj_index);
        });

    lookup.erase(guid);
    component_lookup.erase(guid);
    std::optional<Xform> xform;

    if (auto it = xforms.find(guid); it != xforms.end()) {
        xform = it->second;
        xforms.erase(it);
    }

    bvh_cache_dirty = true;
    std::optional<std::string> parent_guid;
    int index = 0;
    std::shared_ptr<TreeNode> node = tree.get_node_by_name(guid);

    if (node) {
        if (std::shared_ptr<TreeNode> parent = node->parent()) {
            parent_guid = parent->name;
            const std::vector<TreeNode*> children = parent->children();
            index = static_cast<int>(std::find(children.begin(), children.end(), node.get()) - children.begin());
        }

        node = tree.remove(node);
    }

    std::string attribute;
    std::vector<std::tuple<std::string, std::string, bool>> edges;

    if (graph.has_node(guid)) {
        attribute = graph.node_attribute(guid);
        edges = graph.edges_of(guid);
        graph.remove_node(guid);
    }

    return RemoveOp(guid, clone(*obj), collection, obj_index, xform, parent_guid, index, node, attribute, edges);
}

void Session::_attach(const Tombstone& op) {

    const Item obj = clone(op.obj);
    with_collection(objects, op.collection, [&](auto& items) {
        using E = typename std::decay_t<decltype(items)>::value_type;
        items.insert(items.begin() + std::min<size_t>(op.obj_index, items.size()), element_of<E>(obj));
    });

    if (const Geometry* geometry = std::get_if<Geometry>(&obj))
        lookup[op.guid] = *geometry;
    else
        component_lookup[op.guid] = std::get<Component>(obj);

    if (op.xform)
        xforms[op.guid] = *op.xform;

    bvh_cache_dirty = true;
    std::shared_ptr<TreeNode> node = op.node;

    if (!node)
        node = std::make_shared<TreeNode>(op.guid);

    if (op.parent_guid) {
        std::shared_ptr<TreeNode> parent = tree.get_node_by_name(*op.parent_guid);

        if (parent) {
            tree.add(node, parent);
            const std::vector<TreeNode*> children = parent->children();

            for (size_t i = std::min<size_t>(op.index, children.size() - 1); i + 1 < children.size(); ++i)
                parent->add(parent->remove(children[i]->shared_from_this()));
        }
    }

    graph.add_node(op.guid, op.attribute);

    for (const auto& [other, attribute, forward] : op.edges) {
        if (!graph.has_node(other))
            continue;

        if (forward)
            graph.add_edge(op.guid, other, attribute);
        else
            graph.add_edge(other, op.guid, attribute);
    }
}

void Session::_swap(const std::string& guid, const Item& obj) {

    const auto [collection, obj_index] = _locate(guid);

    if (obj_index < 0)
        return;

    with_collection(objects, collection, [&](auto& items) {
        items[obj_index] = element_of<typename std::decay_t<decltype(items)>::value_type>(obj);
    });

    if (const Geometry* geometry = std::get_if<Geometry>(&obj))
        lookup[guid] = *geometry;
    else
        component_lookup[guid] = std::get<Component>(obj);

    bvh_cache_dirty = true;
    std::string attribute;

    for (const auto& [name, prefix] : COLLECTIONS)
        if (name == collection)
            attribute = prefix + "_" + item_name(obj);

    if (graph.has_node(guid))
        graph.node_attribute(guid, attribute);
}

void Session::_index_objects() {

    lookup.clear();
    component_lookup.clear();

    for (const std::shared_ptr<Point>& point : *objects.points)
        lookup[point->guid()] = point;

    for (const std::shared_ptr<Line>& line : *objects.lines)
        lookup[line->guid()] = line;

    for (const std::shared_ptr<Plane>& plane : *objects.planes)
        lookup[plane->guid()] = plane;

    for (const std::shared_ptr<OBB>& bbox : *objects.bboxes)
        lookup[bbox->guid()] = bbox;

    for (const std::shared_ptr<Polyline>& polyline : *objects.polylines)
        lookup[polyline->guid()] = polyline;

    for (const std::shared_ptr<PointCloud>& pointcloud : *objects.pointclouds)
        lookup[pointcloud->guid()] = pointcloud;

    for (const std::shared_ptr<Mesh>& mesh : *objects.meshes)
        lookup[mesh->guid()] = mesh;

    for (const std::shared_ptr<NurbsCurve>& nurbscurve : *objects.nurbscurves)
        lookup[nurbscurve->guid()] = nurbscurve;

    for (const std::shared_ptr<NurbsSurface>& nurbssurface : *objects.nurbssurfaces)
        lookup[nurbssurface->guid()] = nurbssurface;

    for (const std::shared_ptr<BRep>& brep : *objects.breps)
        lookup[brep->guid()] = brep;

    for (const std::shared_ptr<Element>& element : *objects.elements)
        lookup[element->guid()] = element;

    for (const Component& component : *objects.components)
        component_lookup[component.guid()] = component;
}

void Session::_place(const std::string& guid, const std::optional<Xform>& xform) {

    if (xform)
        xforms[guid] = *xform;
    else
        xforms.erase(guid);

    bvh_cache_dirty = true;
}

std::vector<std::pair<std::string, Xform>> Session::_xforms_ordered() const {

    std::vector<std::pair<std::string, Xform>> ordered;
    std::map<std::string, Xform> rest;

    for (const auto& [obj_guid, obj_xform] : xforms)
        if (!obj_xform.is_identity())
            rest.emplace(obj_guid, obj_xform);

    for (const std::string& obj_guid : order()) {
        auto it = rest.find(obj_guid);

        if (it == rest.end())
            continue;

        ordered.emplace_back(obj_guid, it->second);
        rest.erase(it);
    }

    for (const auto& [obj_guid, obj_xform] : rest)
        ordered.emplace_back(obj_guid, obj_xform);

    return ordered;
}

std::vector<OBB> Session::_compute_boxes(std::vector<std::string>& guids) const {

    guids.clear();
    std::vector<OBB> boxes;
    boxes.reserve(lookup.size());
    const std::unordered_map<std::string, Xform> world = world_xforms();

    for (const std::string& guid : order()) {
        auto it = lookup.find(guid);

        if (it == lookup.end())
            continue;

        auto wit = world.find(guid);
        boxes.push_back(compute_bounding_box(it->second, wit == world.end() ? Xform::identity() : wit->second));
        guids.push_back(guid);
    }

    return boxes;
}

void Session::_rebuild_ray_bvh_cache() {

    cached_boxes = _compute_boxes(cached_guids);

    if (cached_boxes.empty())
        cached_ray_bvh = SpatialBVH();
    else
        cached_ray_bvh = SpatialBVH::from_boxes(cached_boxes, SpatialBVH::compute_world_size(cached_boxes));
}

std::optional<Point> Session::_ray_intersect_geometry(
    const Line& ray,
    const Geometry& geometry,
    double tolerance,
    const Xform& placement
) const {

    if (const std::shared_ptr<Point>* point = std::get_if<std::shared_ptr<Point>>(&geometry))
        return ray_point(ray, **point, tolerance);

    if (const std::shared_ptr<Line>* line = std::get_if<std::shared_ptr<Line>>(&geometry)) {
        Point hit;

        if (Intersection::line_line(ray, **line, hit, tolerance))
            return hit;

        return std::nullopt;
    }

    if (const std::shared_ptr<Plane>* plane = std::get_if<std::shared_ptr<Plane>>(&geometry)) {
        Point hit;

        if (Intersection::line_plane(ray, **plane, hit, true))
            return hit;

        return std::nullopt;
    }

    if (const std::shared_ptr<Polyline>* polyline = std::get_if<std::shared_ptr<Polyline>>(&geometry)) {
        std::optional<Point> closest;
        double min_dist = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < (*polyline)->segment_count(); ++i) {
            const Line segment = Line::from_points((*polyline)->get_point(i), (*polyline)->get_point(i + 1));
            Point hit;

            if (!Intersection::line_line(ray, segment, hit, tolerance))
                continue;

            const double dist = ray.start().distance(hit);

            if (dist < min_dist) {
                min_dist = dist;
                closest = hit;
            }
        }

        return closest;
    }

    if (const std::shared_ptr<PointCloud>* pointcloud = std::get_if<std::shared_ptr<PointCloud>>(&geometry)) {
        std::optional<Point> closest;
        double min_dist = std::numeric_limits<double>::infinity();

        for (const Point& point : (*pointcloud)->get_points()) {
            const std::optional<Point> hit = ray_point(ray, point, tolerance);

            if (!hit)
                continue;

            const double dist = point.distance(*hit);

            if (dist < min_dist) {
                min_dist = dist;
                closest = hit;
            }
        }

        return closest;
    }

    if (const std::shared_ptr<Mesh>* mesh = std::get_if<std::shared_ptr<Mesh>>(&geometry)) {
        const std::optional<Xform> inverse = placement.inverse();

        if (!inverse)
            return std::nullopt;

        const Line local_ray =
            Line::from_points(inverse->transform_point(ray.start()), inverse->transform_point(ray.end()));

        const std::vector<Point> hits = Intersection::ray_mesh_bvh(local_ray, **mesh, tolerance, true);

        if (hits.empty())
            return std::nullopt;

        return placement.transform_point(hits[0]);
    }

    if (const std::shared_ptr<OBB>* bbox = std::get_if<std::shared_ptr<OBB>>(&geometry)) {
        double tmin = 0.0;
        double tmax = 0.0;

        if (!Intersection::ray_box(ray.start(), ray.end() - ray.start(), **bbox, 0.0, 1.0, tmin, tmax))
            return std::nullopt;

        return ray.start() + (ray.end() - ray.start()) * tmin;
    }

    return std::nullopt;
}

std::ostream& operator<<(std::ostream& os, const Session& session) {
    return os << session.str();
}

} // namespace session_cpp
