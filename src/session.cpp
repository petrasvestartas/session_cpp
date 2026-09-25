#include "session.h"
#include "graph.h"
#include "tree.h"
#include "intersection.h"
#include "tolerance.h"
#include "session.pb.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <utility>

namespace session_cpp {

namespace {

constexpr uint64_t LENGTH_DELIMITED = 2; // Protobuf wire type of a message or string field.
constexpr size_t HEAD = 0;               // Checkpoint phase: the session name and guid.
constexpr size_t OBJECTS = 1;            // Checkpoint phases 1..=13: the objects lists.
constexpr size_t TREE = 14;              // Checkpoint phase: the tree, depth first.
constexpr size_t VERTICES = 15;          // Checkpoint phase: the graph vertices.
constexpr size_t EDGES = 16;             // Checkpoint phase: the graph edges.
constexpr size_t ORDERED = 17;           // Checkpoint phases 17..=27: xforms in order() sequence.
constexpr size_t REST = 28;              // Checkpoint phase: xforms outside order(), by guid.
constexpr size_t DEFINITIONS = 29;       // Checkpoint phases 29..=41: the definitions lists.
constexpr size_t INTERACTIONS = 42;      // Checkpoint phase: the interactions, by edge guid.
constexpr size_t ASSEMBLY = 43;          // Checkpoint phase: the sections joined into one message.

/// Field numbers the checkpoint writer frames by hand, from the generated messages.
struct Tags {
    std::array<int, 7> sections; // Session field per section, 0 for one written framed: head, objects, tree, graph, xforms, definitions, interactions.
    int root;                    // Tree.root
    int children;                // TreeNode.children
    std::array<int, 13> lists;   // Objects field per COLLECTIONS entry.
};

constexpr Tags TAGS = {
    {
        0,
        session_proto::Session::kObjectsFieldNumber,
        session_proto::Session::kTreeFieldNumber,
        session_proto::Session::kGraphFieldNumber,
        0,
        session_proto::Session::kDefinitionsFieldNumber,
        0,
    },
    session_proto::Tree::kRootFieldNumber,
    session_proto::TreeNode::kChildrenFieldNumber,
    {
        session_proto::Objects::kPointsFieldNumber,
        session_proto::Objects::kLinesFieldNumber,
        session_proto::Objects::kPlanesFieldNumber,
        session_proto::Objects::kBboxesFieldNumber,
        session_proto::Objects::kPolylinesFieldNumber,
        session_proto::Objects::kPointcloudsFieldNumber,
        session_proto::Objects::kMeshesFieldNumber,
        session_proto::Objects::kNurbscurvesFieldNumber,
        session_proto::Objects::kNurbssurfacesFieldNumber,
        session_proto::Objects::kBrepsFieldNumber,
        session_proto::Objects::kElementsFieldNumber,
        session_proto::Objects::kComponentsFieldNumber,
        session_proto::Objects::kInstancesFieldNumber,
    },
};

/// Append the key and length of a length-delimited field.
void frame(std::string& out, int tag, size_t length) {

    for (uint64_t value : {static_cast<uint64_t>(tag) << 3 | LENGTH_DELIMITED, static_cast<uint64_t>(length)}) {

        for (; value >= 0x80; value >>= 7)
            out.push_back(static_cast<char>(value | 0x80));

        out.push_back(static_cast<char>(value));
    }
}

/// Append a message, framed under tag unless it is 0, map entries sorted by key so equal content gives equal bytes.
void encode(std::string& out, const google::protobuf::MessageLite& message, int tag = 0) {

    const size_t size = message.ByteSizeLong();

    if (tag > 0)
        frame(out, tag, size);

    google::protobuf::io::StringOutputStream output(&out);
    google::protobuf::io::CodedOutputStream stream(&output);
    stream.SetSerializationDeterministic(true);
    message.SerializeWithCachedSizes(&stream);
}

/// The name and guid fields of a Graph message.
session_proto::Graph graph_head(const Graph& graph) {

    session_proto::Graph proto;
    proto.set_name(graph.name);

    if (graph.has_guid())
        proto.set_guid(graph.guid());

    return proto;
}

/// The fields of a Graph message after its edges: the counters and the default attributes.
session_proto::Graph graph_tail(const Graph& graph) {

    session_proto::Graph proto;
    proto.set_vertex_count(graph.vertex_count);
    proto.set_edge_count(graph.edge_count);

    for (const std::pair<const std::string, double>& attribute : graph.default_vertex_attributes)
        (*proto.mutable_default_vertex_attributes())[attribute.first] = attribute.second;

    for (const std::pair<const std::string, double>& attribute : graph.default_edge_attributes)
        (*proto.mutable_default_edge_attributes())[attribute.first] = attribute.second;

    return proto;
}

/// The name and guid fields of an Objects message.
session_proto::Objects objects_head(const Objects& objects) {

    session_proto::Objects proto;
    proto.set_name(objects.name);

    if (objects.has_guid())
        proto.set_guid(objects.guid());

    return proto;
}

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
    else if (collection == "instances")
        f(*objects.instances);
}

/// Whether an Objects list of value type E holds geometry, not components or instances.
template <typename E>
constexpr bool IS_GEOMETRY = !std::is_same_v<E, Component> && !std::is_same_v<E, std::shared_ptr<InstanceRef>>;

/// The guid of an item, geometry, component or instance.
std::string item_guid(const Item& item) {

    if (const Geometry* geometry = std::get_if<Geometry>(&item))
        return std::visit(
            [](const auto& live) {
                return live->guid();
            },
            *geometry
        );

    if (const Component* component = std::get_if<Component>(&item))
        return component->guid();

    return std::get<std::shared_ptr<InstanceRef>>(item)->guid();
}

/// The name of an item, geometry, component or instance.
std::string item_name(const Item& item) {

    if (const Geometry* geometry = std::get_if<Geometry>(&item))
        return std::visit(
            [](const auto& live) {
                return live->name;
            },
            *geometry
        );

    if (const Component* component = std::get_if<Component>(&item))
        return component->name;

    return std::get<std::shared_ptr<InstanceRef>>(item)->name;
}

/// The element an Objects list of value type E stores for an item.
template <typename E> E element_of(const Item& item) {

    if constexpr (IS_GEOMETRY<E>)
        return std::get<E>(std::get<Geometry>(item));
    else
        return std::get<E>(item);
}

/// The item an Objects list of value type E stores, as the stored pointer.
template <typename E> Item item_of(const E& stored) {

    if constexpr (IS_GEOMETRY<E>)
        return Geometry(stored);
    else
        return stored;
}

/// The COLLECTIONS entry whose list holds the type of geometry.
std::pair<std::string, std::string> collection_of(const Objects& objects, const Geometry& geometry) {

    for (const std::pair<std::string, std::string>& entry : COLLECTIONS) {

        bool match = false;

        with_collection(objects, entry.first, [&](const auto& items) {
            using E = typename std::decay_t<decltype(items)>::value_type;
            if constexpr (IS_GEOMETRY<E>)
                match = std::holds_alternative<E>(geometry);
        });

        if (match)
            return entry;
    }

    return {"", ""};
}

/// Point every live slot whose guid lookup holds with another value at the lookup value.
void repoint(const Objects& objects, const std::unordered_map<std::string, Geometry>& lookup) {

    for (const std::pair<const std::string, Geometry>& entry : lookup)
        with_collection(objects, collection_of(objects, entry.second).first, [&](auto& items) {
            using E = typename std::decay_t<decltype(items)>::value_type;
            if constexpr (IS_GEOMETRY<E>) {

                const std::optional<size_t> slot = items.get_slot(entry.first);

                if (slot && items.get_item(*slot) != std::get<E>(entry.second))
                    items.set_item(*slot, std::get<E>(entry.second));
            }
        });
}

/// Index every live slot lookup lacks, then push every geometry only lookup holds, in guid order.
void adopt(const Objects& objects, std::unordered_map<std::string, Geometry>& lookup) {

    for (const std::pair<std::string, std::string>& entry : COLLECTIONS)
        with_collection(objects, entry.first, [&](const auto& items) {
            using E = typename std::decay_t<decltype(items)>::value_type;
            if constexpr (IS_GEOMETRY<E>)
                for (const E& item : items)
                    lookup.emplace(item->guid(), item);
        });

    std::map<std::string, Geometry> orphans;

    for (const std::pair<const std::string, Geometry>& entry : lookup) {

        const std::string guid = item_guid(entry.second);
        bool held = false;

        with_collection(objects, collection_of(objects, entry.second).first, [&](const auto& items) {
            held = items.get_slot(guid).has_value();
        });

        if (!held)
            orphans.emplace(guid, entry.second);
    }

    for (const std::pair<const std::string, Geometry>& orphan : orphans)
        with_collection(objects, collection_of(objects, orphan.second).first, [&](auto& items) {
            items.push_back(element_of<typename std::decay_t<decltype(items)>::value_type>(orphan.second));
        });
}

/// The COLLECTIONS entry whose list holds an item.
std::pair<std::string, std::string> collection_for(const Objects& objects, const Item& item) {

    if (const Geometry* geometry = std::get_if<Geometry>(&item))
        return collection_of(objects, *geometry);

    if (std::holds_alternative<Component>(item))
        return {"components", "component"};

    return {"instances", "instance"};
}

/// The graph attribute prefix of the list of that name.
std::string prefix_of(const std::string& collection) {

    for (const std::pair<std::string, std::string>& entry : COLLECTIONS)
        if (entry.first == collection)
            return entry.second;

    return "";
}

/// The live slot of a guid in the list of that name.
std::optional<size_t> slot_of(const Objects& objects, const std::string& collection, const std::string& guid) {

    std::optional<size_t> slot;

    with_collection(objects, collection, [&](const auto& items) {
        slot = items.get_slot(guid);
    });

    return slot;
}

/// The item in a slot of the list of that name, dead or alive, as the stored pointer.
std::optional<Item> item_at(const Objects& objects, const std::string& collection, size_t slot) {

    std::optional<Item> item;

    with_collection(objects, collection, [&](const auto& items) {
        item = item_of(items.get_item(slot));
    });

    return item;
}

/// Append an item to the list of that name and return its slot.
size_t push(const Objects& objects, const std::string& collection, const Item& item) {

    size_t slot = 0;

    with_collection(objects, collection, [&](auto& items) {
        items.push_back(element_of<typename std::decay_t<decltype(items)>::value_type>(item));
        slot = items.number_of_slots() - 1;
    });

    return slot;
}

/// Put an item in a slot of the list of that name.
void store(const Objects& objects, const std::string& collection, size_t slot, const Item& item) {

    with_collection(objects, collection, [&](auto& items) {
        items.set_item(slot, element_of<typename std::decay_t<decltype(items)>::value_type>(item));
    });
}

/// Kill or revive a slot of the list of that name.
void flag(const Objects& objects, const std::string& collection, size_t slot, bool dead) {

    with_collection(objects, collection, [&](auto& items) {
        items.set_dead(slot, dead);
    });
}

/// The tomb pinning a slot of the list of that name, while a record still holds it.
std::shared_ptr<Tomb> tomb_at(const Objects& objects, const std::string& collection, size_t slot) {

    std::shared_ptr<Tomb> tomb;

    with_collection(objects, collection, [&](const auto& items) {
        tomb = items.get_tomb(slot);
    });

    return tomb;
}

/// Pin a slot of the list of that name to a tomb.
void pin(const Objects& objects, const std::string& collection, size_t slot, const std::shared_ptr<Tomb>& tomb) {

    with_collection(objects, collection, [&](auto& items) {
        items.set_tomb(slot, tomb);
    });
}

/// Whether two items are the same stored pointer; a component is never, so the map value is stored back.
bool same(const Item& a, const Item& b) {

    if (const Geometry* x = std::get_if<Geometry>(&a)) {

        const Geometry* y = std::get_if<Geometry>(&b);

        return y && *x == *y;
    }

    if (const std::shared_ptr<InstanceRef>* x = std::get_if<std::shared_ptr<InstanceRef>>(&a)) {

        const std::shared_ptr<InstanceRef>* y = std::get_if<std::shared_ptr<InstanceRef>>(&b);

        return y && *x == *y;
    }

    return false;
}

/// Move geometry in place: an element is placed, anything else transformed; identity leaves it untouched.
void place(const Geometry& geometry, const Xform& xform) {

    if (xform.is_identity())
        return;

    std::visit(
        [&](const auto& live) {
            using P = typename std::decay_t<decltype(live)>::element_type;
            if constexpr (std::is_same_v<P, Element>)
                live->place(xform);
            else
                live->transform(xform);
        },
        geometry
    );
}

/// The definition copied as the instance, its guid and name and on an element its features, then moved by xform.
Geometry resolve(const InstanceRef& instance, const Geometry& definition, const Xform& xform) {

    const Geometry copy = clone(definition);

    std::visit(
        [&](const auto& live) {
            using P = typename std::decay_t<decltype(live)>::element_type;
            live->guid() = instance.guid();
            live->name = instance.name;

            if constexpr (std::is_same_v<P, Element>)
                for (ElementFeature& feature : clone(instance.features))
                    live->add_feature(std::move(feature));
        },
        copy
    );

    place(copy, xform);

    return copy;
}

/// Transforms every object of a list by its world placement, identity entries untouched.
template <typename T>
void bake(Collection<std::shared_ptr<T>>& items, const std::unordered_map<std::string, Xform>& world) {

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

/// The segment hit closest to the ray start.
std::optional<Point> ray_polyline(const Line& ray, const Polyline& polyline, double tolerance) {

    std::optional<Point> closest;
    double min_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < polyline.segment_count(); ++i) {

        const Line segment = Line::from_points(polyline.get_point(i), polyline.get_point(i + 1));
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

/// The ray point closest to a cloud point within tolerance.
std::optional<Point> ray_pointcloud(const Line& ray, const PointCloud& pointcloud, double tolerance) {

    std::optional<Point> closest;
    double min_dist = std::numeric_limits<double>::infinity();

    for (const Point& point : pointcloud.get_points()) {

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

/// The first hit of the ray on the placed mesh, tested in the mesh frame.
std::optional<Point> ray_mesh(const Line& ray, const Mesh& mesh, double tolerance, const Xform& placement) {

    const std::optional<Xform> inverse = placement.inverse();

    if (!inverse)
        return std::nullopt;

    const Line local_ray = Line::from_points(
        inverse->transform_point(ray.start()),
        inverse->transform_point(ray.end())
    );
    const std::vector<Point> hits = Intersection::ray_mesh_bvh(local_ray, mesh, tolerance, true);

    if (hits.empty())
        return std::nullopt;

    return placement.transform_point(hits[0]);
}

/// The points whose box bounds a geometry: vertices, control points or surface samples.
std::vector<Point> box_points(const Geometry& geometry) {

    std::vector<Point> points;

    if (const std::shared_ptr<Line>* line = std::get_if<std::shared_ptr<Line>>(&geometry)) {
        points.push_back((*line)->start());
        points.push_back((*line)->end());
    } else if (const std::shared_ptr<Polyline>* polyline = std::get_if<std::shared_ptr<Polyline>>(&geometry)) {
        points = (*polyline)->get_points();
    } else if (const std::shared_ptr<PointCloud>* pointcloud = std::get_if<std::shared_ptr<PointCloud>>(&geometry)) {
        points = (*pointcloud)->get_points();
    } else if (const std::shared_ptr<Mesh>* mesh = std::get_if<std::shared_ptr<Mesh>>(&geometry)) {
        for (const std::pair<const size_t, VertexData>& vertex : (*mesh)->vertex)
            points.push_back(vertex.second.position());
    } else if (const std::shared_ptr<BRep>* brep = std::get_if<std::shared_ptr<BRep>>(&geometry)) {
        for (const BRepVertex& vertex : (*brep)->m_vertices)
            points.push_back(vertex.point);

        for (const NurbsSurface& surface : (*brep)->m_surfaces) {

            const std::pair<double, double> u = surface.domain(0);
            const std::pair<double, double> v = surface.domain(1);

            for (int i = 0; i <= 2; ++i)
                for (int j = 0; j <= 2; ++j)
                    points.push_back(surface.point_at(
                        u.first + (u.second - u.first) * i / 2.0,
                        v.first + (v.second - v.first) * j / 2.0
                    ));
        }
    } else if (const std::shared_ptr<NurbsCurve>* nurbscurve = std::get_if<std::shared_ptr<NurbsCurve>>(&geometry)) {
        for (int i = 0; i < (*nurbscurve)->cv_count(); ++i)
            points.push_back((*nurbscurve)->get_cv(i));
    } else if (const std::shared_ptr<NurbsSurface>* nurbssurface = std::get_if<std::shared_ptr<NurbsSurface>>(&geometry)) {
        for (int i = 0; i < (*nurbssurface)->cv_count(0); ++i)
            for (int j = 0; j < (*nurbssurface)->cv_count(1); ++j)
                points.push_back((*nurbssurface)->get_cv(i, j));
    }

    return points;
}

/// Whether guid is a graph node held by an object, instance or component.
bool registered(const Session& session, const std::string& guid) {

    const bool held = session.lookup.count(guid) || session.instance_lookup.count(guid) || session.component_lookup.count(guid);

    return session.graph.has_node(guid) && held;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Session::Session(std::string name)
    : name(std::move(name)), objects(), tree(this->name + "_tree"), graph(this->name + "_graph") {

    tree.add(std::make_shared<TreeNode>(this->name));
    _indexed = tree.root();
}

Session::Session(const Session& other)
    : name(other.name), objects(other.objects), tree(other.tree), graph(other.graph), xforms(other.xforms),
      definitions(other.definitions) {

    if (other.has_guid())
        guid() = other.guid();

    for (const std::pair<const std::string, std::vector<std::shared_ptr<Interaction>>>& entry : other.interactions)
        for (const std::shared_ptr<Interaction>& interaction : entry.second)
            interactions[entry.first].push_back(interaction->clone());

    graph.renumber();
    reindex();
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
std::shared_ptr<TreeNode> Session::get_node(const std::string& guid) const {

    if (!_is_live(guid))
        return nullptr;

    const std::shared_ptr<TreeNode> root = tree.root();
    auto it = node_lookup.find(guid);
    const bool fresh = root && _indexed.lock() == root;

    if (fresh && it != node_lookup.end() && it->second->name == guid && it->second->parent())
        return it->second;

    return tree.get_node_by_name(guid);
}

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
    std::shared_ptr<TreeNode> node = get_node(guid);

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

        TreeNode* node = stack.back().first;
        const Xform parent_xform = stack.back().second;
        stack.pop_back();
        auto it = xforms.find(node->name);
        const Xform current = it == xforms.end() ? parent_xform : parent_xform * it->second;
        out[node->name] = current;

        for (TreeNode* child : node->children())
            stack.emplace_back(child, current);
    }

    for (const std::pair<const std::string, Xform>& entry : xforms)
        out.emplace(entry.first, entry.second);

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

    for (const std::shared_ptr<InstanceRef>& instance : *objects.instances) {

        auto definition = definition_lookup.find(instance->definition_guid);

        if (definition == definition_lookup.end())
            continue;

        auto it = world.find(instance->guid());
        const Xform placement = it == world.end() ? Xform::identity() : it->second;
        const Geometry resolved = resolve(*instance, definition->second, placement);

        with_collection(out, collection_of(out, resolved).first, [&](auto& items) {
            items.push_back(element_of<typename std::decay_t<decltype(items)>::value_type>(resolved));
        });
    }

    out.instances->clear();

    return out;
}

std::optional<Geometry> Session::definition_of(const std::string& instance_guid) const {

    auto instance = instance_lookup.find(instance_guid);

    if (instance == instance_lookup.end())
        return std::nullopt;

    auto definition = definition_lookup.find(instance->second->definition_guid);

    if (definition == definition_lookup.end())
        return std::nullopt;

    return definition->second;
}

std::vector<std::string> Session::instances_of(const std::string& definition_guid) const {

    std::vector<std::string> guids;

    for (const std::shared_ptr<InstanceRef>& instance : *objects.instances)
        if (instance->definition_guid == definition_guid)
            guids.push_back(instance->guid());

    return guids;
}

std::optional<Geometry> Session::world_geometry(const std::string& guid) const {

    const Xform world = world_xform(guid);

    if (auto it = lookup.find(guid); it != lookup.end()) {

        const Geometry copy = clone(it->second);
        place(copy, world);

        return copy;
    }

    const std::optional<Geometry> definition = definition_of(guid);

    if (!definition)
        return std::nullopt;

    return resolve(*instance_lookup.at(guid), *definition, world);
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry management
// ═══════════════════════════════════════════════════════════════════════════
std::shared_ptr<TreeNode> Session::add_point(std::shared_ptr<Point> point, std::shared_ptr<TreeNode> parent) {

    if (!point)
        return nullptr;

    std::shared_ptr<TreeNode> node = _add_object("points", point, "point", parent);

    return node ? node : _node_of(point->guid());
}

std::shared_ptr<TreeNode> Session::add_line(std::shared_ptr<Line> line, std::shared_ptr<TreeNode> parent) {

    if (!line)
        return nullptr;

    std::shared_ptr<TreeNode> node = _add_object("lines", line, "line", parent);

    return node ? node : _node_of(line->guid());
}

std::shared_ptr<TreeNode> Session::add_plane(std::shared_ptr<Plane> plane, std::shared_ptr<TreeNode> parent) {

    if (!plane)
        return nullptr;

    std::shared_ptr<TreeNode> node = _add_object("planes", plane, "plane", parent);

    return node ? node : _node_of(plane->guid());
}

std::shared_ptr<TreeNode> Session::add_obb(std::shared_ptr<OBB> bbox) {

    if (!bbox)
        return nullptr;

    std::shared_ptr<TreeNode> node = _add_object("bboxes", bbox, "bbox", nullptr);

    return node ? node : _node_of(bbox->guid());
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

    std::shared_ptr<TreeNode> node = _add_object("elements", element, "element", parent);

    return node ? node : _node_of(element->guid());
}

std::shared_ptr<TreeNode> Session::add_component(Component component, std::shared_ptr<TreeNode> parent) {

    std::shared_ptr<TreeNode> node = _add_object("components", component, "component", parent);

    return node ? node : _node_of(component.guid());
}

std::string Session::add_definition(const Geometry& definition) {

    const bool empty = std::visit(
        [](const auto& live) {
            return live == nullptr;
        },
        definition
    );

    if (empty)
        return "";

    const std::string guid = item_guid(definition);

    if (definition_lookup.count(guid))
        return guid;

    if (_is_live(guid))
        return "";

    const std::string collection = collection_of(definitions, definition).first;
    const size_t slot = push(definitions, collection, definition);
    definition_lookup[guid] = definition;
    bvh_cache_dirty = true;
    revision++;

    if (history.current) {

        const std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>(collection, true, slot, nullptr);
        pin(definitions, collection, slot, tomb);
        history.record(AddOp(guid, "definitions", std::nullopt, 0, nullptr, tomb), RECORD);
    }

    return guid;
}

std::shared_ptr<TreeNode> Session::add_instance(
    std::shared_ptr<InstanceRef> instance,
    const Xform& xform,
    std::shared_ptr<TreeNode> parent
) {

    if (!instance || definition_lookup.count(instance->definition_guid) == 0)
        return nullptr;

    const Xform placement = xform * instance->xform;
    std::shared_ptr<TreeNode> node = _add_object("instances", instance, "instance", parent);

    if (!node)
        return nullptr;

    instance->xform = Xform::identity();

    if (!placement.is_identity())
        set_xform(instance->guid(), placement);

    return node;
}

void Session::add(std::shared_ptr<TreeNode> node, std::shared_ptr<TreeNode> parent) {

    const std::shared_ptr<TreeNode> host = parent ? parent : tree.root();

    if (!node || !host || node == host || node->parent() == host)
        return;

    const std::string name = node->name;
    const bool was_dead = node->is_dead();
    const std::shared_ptr<TreeNode> old = node->parent();
    const std::shared_ptr<TreeNode> ghost = host->add(node);

    if (!host->has_child(node))
        return;

    node->set_dead(false);
    revision++;

    if (old && ghost)
        _queue(old);

    const std::shared_ptr<Tomb> held = node->get_tomb();

    if (was_dead && held && held->collection.empty() && held->xform) {
        xforms[name] = *held->xform;
        held->xform.reset();
    }

    if (_is_live(name))
        node_lookup[name] = node;

    if (!history.current) {
        history.dropped += ghost ? 1 : 0;

        return;
    }

    const std::shared_ptr<Tomb> tomb = _node_tomb(ghost ? ghost : node);
    const bool dead_before = was_dead || !ghost;
    history.record(TreeOp(name, node, tomb, ghost, name, name, node->color, node->color, dead_before, false), RECORD);
}

std::shared_ptr<TreeNode> Session::add_group(const std::string& group_name) {

    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(group_name);
    add(node);

    return node;
}

bool Session::rename_node(std::shared_ptr<TreeNode> node, const std::string& name) {

    const std::string before = node->name;

    if (_is_live(before) || node->is_dead() || before == name)
        return false;

    node->name = name;
    revision++;

    if (history.current) {

        const std::shared_ptr<Tomb> tomb = _node_tomb(node);
        history.record(TreeOp(before, node, tomb, nullptr, before, name, node->color, node->color, false, false), RECORD);
    }

    return true;
}

bool Session::set_node_color(std::shared_ptr<TreeNode> node, std::optional<Color> color) {

    if (node->is_dead())
        return false;

    const std::optional<Color> before = node->color;
    node->color = color;
    revision++;

    if (history.current) {

        const std::string name = node->name;
        const std::shared_ptr<Tomb> tomb = _node_tomb(node);
        history.record(TreeOp(name, node, tomb, nullptr, name, name, before, color, false, false), RECORD);
    }

    return true;
}

bool Session::remove_group(std::shared_ptr<TreeNode> node) {

    const std::string name = node->name;
    const std::shared_ptr<TreeNode> parent = node->parent();

    if (!parent || _is_live(name) || node->is_dead())
        return false;

    const std::shared_ptr<Tomb> tomb = _node_tomb(node);
    node->set_dead(true);
    tomb->xform.reset();
    auto placed = xforms.find(name);

    if (placed != xforms.end()) {
        tomb->xform = placed->second;
        xforms.erase(placed);
    }

    _queue(parent);
    revision++;

    if (!history.current) {
        history.dropped += 1;

        return true;
    }

    history.record(TreeOp(name, node, tomb, nullptr, name, name, node->color, node->color, false, true), RECORD);

    return true;
}

void Session::add_edge(const std::string& guid1, const std::string& guid2, const std::string& attribute) {

    revision++;
    graph.add_edge(guid1, guid2, attribute);
}

bool Session::add_hierarchy(const std::string& parent_guid, const std::string& child_guid) {

    revision++;

    return tree.add_child_by_guid(parent_guid, child_guid);
}

void Session::add_relationship(
    const std::string& from_guid,
    const std::string& to_guid,
    const std::string& relationship_type
) {

    revision++;
    graph.add_edge(from_guid, to_guid, relationship_type);
}

bool Session::remove_object(const std::string& obj_guid) {

    const std::optional<Item> obj = _item(obj_guid);

    if (!obj)
        return false;

    const std::shared_ptr<Tomb> tomb = _tomb(obj_guid);
    auto incident = graph.edges.find(obj_guid);
    const size_t degree = incident == graph.edges.end() ? 0 : incident->second.size();
    const std::shared_ptr<TreeNode> node = tomb->node && tomb->node->parent() ? tomb->node : nullptr;
    const int index = node ? static_cast<int>(node->at()) : 0;
    std::optional<std::string> parent_guid;

    if (node)
        parent_guid = node->parent()->name;

    _kill(tomb);

    if (!history.current) {
        history.dropped += 1;

        return true;
    }

    const size_t bytes = RECORD + weight(*obj) + 128 * degree;
    history.record(RemoveOp(obj_guid, tomb->collection, parent_guid, index, node, tomb), bytes);

    return true;
}

bool Session::replace(const std::string& guid, const Geometry& obj) {

    auto found = lookup.find(guid);

    if (found == lookup.end())
        return false;

    const Geometry before = found->second;

    std::visit(
        [&](const auto& live) {
            live->guid() = guid;
        },
        obj
    );

    const std::string from = collection_of(objects, before).first;
    const std::pair<std::string, std::string> to = collection_of(objects, obj);
    const size_t bytes = RECORD + weight(before);

    if (from == to.first) {

        const Entry entry(false, get_node(guid), nullptr);

        if (history.current)
            history.record(ReplaceOp(guid, before, obj, entry), bytes);

        _swap(guid, obj, entry);

        return true;
    }

    const std::shared_ptr<TreeNode> node = get_node(guid);
    const std::shared_ptr<Tomb> removed = _half(false, from, guid);

    if (!removed)
        return false;

    _kill(removed);
    const size_t slot = push(objects, to.first, obj);
    const std::shared_ptr<Tomb> added = std::make_shared<Tomb>(to.first, false, slot, nullptr);
    pin(objects, to.first, slot, added);
    lookup[guid] = obj;
    _label(guid, to.second + "_" + item_name(obj));
    _pair(guid, from, to.first, node, removed, added, bytes);

    return true;
}

bool Session::replace_definition(const std::string& guid, const Geometry& definition) {

    auto found = definition_lookup.find(guid);

    if (found == definition_lookup.end())
        return false;

    const Geometry before = found->second;

    std::visit(
        [&](const auto& live) {
            live->guid() = guid;
        },
        definition
    );

    const std::string from = collection_of(definitions, before).first;
    const std::string to = collection_of(definitions, definition).first;
    const size_t bytes = RECORD + weight(before);

    if (from == to) {

        const std::shared_ptr<Tomb> tomb = _half(true, from, guid);

        if (!tomb)
            return false;

        const Entry entry(true, nullptr, tomb);

        if (history.current)
            history.record(ReplaceOp(guid, before, definition, entry), bytes);

        _swap(guid, definition, entry);

        return true;
    }

    const std::shared_ptr<Tomb> removed = _half(true, from, guid);

    if (!removed)
        return false;

    _kill(removed);
    const size_t slot = push(definitions, to, definition);
    const std::shared_ptr<Tomb> added = std::make_shared<Tomb>(to, true, slot, nullptr);
    pin(definitions, to, slot, added);
    definition_lookup[guid] = definition;
    _pair(guid, "definitions", "definitions", nullptr, removed, added, bytes);

    return true;
}

bool Session::remove_definition(const std::string& guid) {

    auto found = definition_lookup.find(guid);

    if (found == definition_lookup.end() || !instances_of(guid).empty())
        return false;

    const Geometry before = found->second;
    const std::string collection = collection_of(definitions, before).first;
    const std::shared_ptr<Tomb> tomb = _half(true, collection, guid);

    if (!tomb)
        return false;

    _kill(tomb);

    if (!history.current) {
        history.dropped += 1;

        return true;
    }

    history.record(RemoveOp(guid, "definitions", std::nullopt, 0, nullptr, tomb), RECORD + weight(before));

    return true;
}

bool Session::to_instance(const std::string& guid, const std::string& definition_guid, const Xform& frame) {

    auto found = lookup.find(guid);

    if (found == lookup.end() || definition_lookup.count(definition_guid) == 0)
        return false;

    const Geometry object = found->second;
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition_guid, Xform::identity());
    instance->guid() = guid;
    instance->name = item_name(object);
    const Xform placement = xform(guid) * frame;
    const std::string from = collection_of(objects, object).first;
    const std::shared_ptr<TreeNode> node = get_node(guid);
    const std::shared_ptr<Tomb> removed = _half(false, from, guid);

    if (!removed)
        return false;

    _kill(removed);
    const size_t slot = push(objects, "instances", instance);
    const std::shared_ptr<Tomb> added = std::make_shared<Tomb>("instances", false, slot, nullptr);
    pin(objects, "instances", slot, added);
    instance_lookup[guid] = instance;
    _label(guid, "instance_" + instance->name);
    _pair(guid, from, "instances", node, removed, added, RECORD + weight(object));

    if (placement.is_identity())
        remove_xform(guid);
    else
        set_xform(guid, placement);

    return true;
}

bool Session::explode(const std::string& instance_guid) {

    const std::optional<Geometry> definition = definition_of(instance_guid);

    if (!definition)
        return false;

    const std::shared_ptr<InstanceRef> instance = instance_lookup.at(instance_guid);
    const Geometry copy = resolve(*instance, *definition, Xform::identity());
    const std::pair<std::string, std::string> to = collection_of(objects, copy);
    const std::shared_ptr<TreeNode> node = get_node(instance_guid);
    const std::shared_ptr<Tomb> removed = _half(false, "instances", instance_guid);

    if (!removed)
        return false;

    _kill(removed);
    const size_t slot = push(objects, to.first, copy);
    const std::shared_ptr<Tomb> added = std::make_shared<Tomb>(to.first, false, slot, nullptr);
    pin(objects, to.first, slot, added);
    lookup[instance_guid] = copy;
    _label(instance_guid, to.second + "_" + instance->name);
    _pair(instance_guid, "instances", to.first, node, removed, added, RECORD + weight(instance));

    return true;
}

void Session::set_xform(const std::string& guid, const Xform& xform) {

    if (definition_lookup.count(guid) && !lookup.count(guid) && !instance_lookup.count(guid))
        return;

    if (history.current) {

        std::optional<Xform> before;
        auto it = xforms.find(guid);

        if (it != xforms.end())
            before = it->second;

        history.record(XformOp(guid, before, xform, get_node(guid)), RECORD);
    }

    xforms[guid] = xform;
    bvh_cache_dirty = true;
    revision++;
}

bool Session::remove_xform(const std::string& guid) {

    auto before = xforms.find(guid);

    if (before == xforms.end())
        return false;

    if (history.current)
        history.record(XformOp(guid, before->second, std::nullopt, get_node(guid)), RECORD);

    xforms.erase(before);
    bvh_cache_dirty = true;
    revision++;

    return true;
}

void Session::reindex() {

    repoint(objects, lookup);
    adopt(objects, lookup);
    repoint(definitions, definition_lookup);
    adopt(definitions, definition_lookup);
    Collection<Component>& components = *objects.components;
    Collection<std::shared_ptr<InstanceRef>>& instances = *objects.instances;

    for (size_t slot = 0; slot < components.number_of_slots(); ++slot) {

        if (components.is_dead(slot))
            continue;

        const std::string guid = components.get_item(slot).guid();
        auto held = component_lookup.find(guid);

        if (held == component_lookup.end())
            component_lookup[guid] = components.get_item(slot);
        else
            components.set_item(slot, held->second);
    }

    std::map<std::string, Component> loose_components;

    for (const std::pair<const std::string, Component>& entry : component_lookup)
        if (!components.get_slot(entry.first))
            loose_components.emplace(entry.second.guid(), entry.second);

    for (const std::pair<const std::string, Component>& entry : loose_components)
        components.push_back(entry.second);

    std::map<std::string, std::shared_ptr<InstanceRef>> loose_instances;

    for (const std::pair<const std::string, std::shared_ptr<InstanceRef>>& entry : instance_lookup)
        if (!instances.get_slot(entry.first))
            loose_instances.emplace(entry.second->guid(), entry.second);

    for (const std::pair<const std::string, std::shared_ptr<InstanceRef>>& entry : loose_instances)
        instances.push_back(entry.second);

    for (size_t slot = 0; slot < instances.number_of_slots(); ++slot) {

        if (instances.is_dead(slot))
            continue;

        const std::string guid = instances.get_item(slot)->guid();
        auto held = instance_lookup.find(guid);
        const std::shared_ptr<InstanceRef> instance = held == instance_lookup.end() ? instances.get_item(slot) : held->second;

        if (!instance->xform.is_identity()) {
            xforms[guid] = xform(guid) * instance->xform;
            instance->xform = Xform::identity();
        }

        instances.set_item(slot, instance);
        instance_lookup[guid] = instance;
    }

    node_lookup.clear();

    for (const std::shared_ptr<TreeNode>& node : tree.nodes())
        if (_is_live(node->name))
            node_lookup.emplace(node->name, node);

    _indexed = tree.root();
}

// ═══════════════════════════════════════════════════════════════════════════
// Session - Interactions
// ═══════════════════════════════════════════════════════════════════════════
std::shared_ptr<Interaction> Session::add_interaction(
    const std::shared_ptr<Element>& a,
    const std::shared_ptr<Element>& b,
    std::shared_ptr<Interaction> interaction
) {

    const std::string& first = a->guid();
    const std::string& second = b->guid();

    if (first == second || !registered(*this, first) || !registered(*this, second))
        throw std::invalid_argument("Session::add_interaction: add two distinct elements to the session first");

    if (!graph.has_edge({first, second}))
        graph.add_edge(first, second);

    revision++;
    const std::string& id = graph.edges.at(first).at(second).guid();
    interactions[id].push_back(interaction);

    return interaction;
}

std::vector<std::shared_ptr<Interaction>> Session::get_interaction(
    const std::shared_ptr<Element>& a,
    const std::shared_ptr<Element>& b
) const {

    if (!graph.has_edge({a->guid(), b->guid()}))
        return {};

    const std::string& id = graph.edges.at(a->guid()).at(b->guid()).guid();
    const std::map<std::string, std::vector<std::shared_ptr<Interaction>>>::const_iterator found = interactions.find(id);

    if (found == interactions.end())
        return {};

    return found->second;
}

bool Session::has_interaction(const std::shared_ptr<Element>& a, const std::shared_ptr<Element>& b) const {
    return graph.has_edge({a->guid(), b->guid()});
}

void Session::remove_interaction(const std::shared_ptr<Element>& a, const std::shared_ptr<Element>& b) {

    if (!graph.has_edge({a->guid(), b->guid()}))
        return;

    const std::string id = graph.edges.at(a->guid()).at(b->guid()).guid();
    revision++;
    interactions.erase(id);
    graph.remove_edge({a->guid(), b->guid()});
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

    revision++;

    return history.undo(*this);
}

bool Session::redo() {

    revision++;

    return history.redo(*this);
}

bool Session::abort() {

    revision++;

    return history.abort(*this);
}

// ═══════════════════════════════════════════════════════════════════════════
// Purge
// ═══════════════════════════════════════════════════════════════════════════
size_t Session::number_of_dead() const {

    size_t count = 0;

    for (const std::pair<std::string, std::string>& entry : COLLECTIONS) {
        with_collection(objects, entry.first, [&](const auto& items) {
            count += items.number_of_dead();
        });
        with_collection(definitions, entry.first, [&](const auto& items) {
            count += items.number_of_dead();
        });
    }

    return count;
}

bool Session::purge_due() const {
    return history.dropped > 0 && (number_of_dead() > 0 || !_sweep.empty());
}

bool Session::is_purging() const {
    return _purging.has_value();
}

bool Session::purge_step(size_t work) {

    const bool fresh = _writer && _writer->revision == revision;

    if (fresh || (!_purging && !purge_due()))
        return false;

    _purge(work);

    return _purging.has_value();
}

void Session::purge() {

    history.clear();
    _writer.reset();

    if (_purging)
        _purge(SIZE_MAX);

    _purge(SIZE_MAX);

    for (const std::shared_ptr<TreeNode>& node : tree.nodes())
        node->compact();

    graph.renumber();
    revision++;
}

std::optional<std::string> Session::checkpoint(size_t work) {

    if (_writer && _writer->revision != revision)
        _writer.reset();

    if (!_writer && (_purging || purge_due()))
        work = _purge(work);

    if (_purging || work == 0)
        return std::nullopt;

    if (!_writer) {
        _writer.emplace();
        _writer->revision = revision;
    }

    if (!_write(*_writer, work))
        return std::nullopt;

    std::string bytes = std::move(_writer->out);
    _writer.reset();

    return bytes;
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

        const std::shared_ptr<Element> copy = (*element)->clone();
        OBB box = copy->aabb();
        box.transform(xform);

        return box;
    }

    return placed_box(box_points(geometry), xform, inflate);
}

std::vector<std::pair<std::string, std::string>> Session::get_collisions() {

    std::vector<std::string> guids;
    const std::vector<OBB> boxes = _compute_boxes(guids);

    if (boxes.empty())
        return {};

    bvh = SpatialBVH::from_boxes(boxes, SpatialBVH::compute_world_size(boxes));
    const std::vector<std::pair<int, int>> pairs = std::get<0>(bvh.check_all_collisions(boxes));
    std::vector<std::pair<std::string, std::string>> guid_pairs;
    guid_pairs.reserve(pairs.size());

    for (const std::pair<int, int>& pair : pairs) {

        const int i = pair.first;
        const int j = pair.second;

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
        const std::optional<Geometry> geometry = it == lookup.end() ? definition_of(guid) : it->second;

        if (!geometry)
            continue;

        auto wit = world.find(guid);
        const Xform placement = wit == world.end() ? Xform::identity() : wit->second;
        const std::optional<Point> hit = _ray_intersect_geometry(ray, *geometry, tolerance, placement);

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
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Session::jsondump() const {

    nlohmann::ordered_json xforms_json = nlohmann::ordered_json::array();

    for (const std::pair<std::string, Xform>& entry : _xforms_ordered()) {

        nlohmann::ordered_json item;
        item["guid"] = entry.first;
        item["xform"] = entry.second.jsondump();
        xforms_json.push_back(item);
    }

    nlohmann::ordered_json interactions_json = nlohmann::ordered_json::array();

    for (const std::pair<const std::string, std::vector<std::shared_ptr<Interaction>>>& entry : interactions) {

        nlohmann::ordered_json items = nlohmann::ordered_json::array();

        for (const std::shared_ptr<Interaction>& interaction : entry.second)
            items.push_back(interaction->jsondump());

        interactions_json.push_back({{"guid", entry.first}, {"interactions", items}});
    }

    nlohmann::ordered_json data;

    if (!definition_lookup.empty())
        data["definitions"] = definitions.jsondump();

    data["graph"] = graph.jsondump();
    data["guid"] = guid();
    data["interactions"] = interactions_json;
    data["name"] = name;
    data["objects"] = objects.jsondump();
    data["tree"] = tree.jsondump();
    data["type"] = "Session";
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

    if (data.contains("definitions"))
        session.definitions = Objects::jsonload(data["definitions"]);

    if (data.contains("xforms"))
        for (const nlohmann::json& entry : data["xforms"])
            session.xforms[entry.at("guid").get<std::string>()] = Xform::jsonload(entry.at("xform"));

    if (data.contains("interactions"))
        for (const nlohmann::json& entry : data["interactions"])
            for (const nlohmann::json& item : entry.at("interactions"))
                session.interactions[entry.at("guid").get<std::string>()].push_back(Interaction::jsonload(item));

    session.reindex();

    return session;
}

std::string Session::file_json_dumps() {

    purge();

    return jsondump().dump();
}

Session Session::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Session::file_json_dump(const std::string& filename) {

    purge();
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Session Session::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Session Session::to_proto() const {

    session_proto::Session proto;
    proto.set_name(name);

    if (has_guid())
        proto.set_guid(guid());

    *proto.mutable_objects() = objects.to_proto();
    proto.mutable_tree()->ParseFromString(tree.pb_dumps());
    *proto.mutable_graph() = graph.to_proto();

    for (const std::pair<std::string, Xform>& entry : _xforms_ordered()) {

        session_proto::XformEntry* item = proto.add_xforms();
        item->set_guid(entry.first);
        *item->mutable_xform() = entry.second.to_proto();
    }

    if (!definition_lookup.empty())
        *proto.mutable_definitions() = definitions.to_proto();

    for (const std::pair<const std::string, std::vector<std::shared_ptr<Interaction>>>& entry : interactions) {

        session_proto::InteractionEntry* item = proto.add_interactions();
        item->set_guid(entry.first);

        for (const std::shared_ptr<Interaction>& interaction : entry.second)
            *item->add_interactions() = interaction->to_proto();
    }

    return proto;
}

Session Session::from_proto(const session_proto::Session& proto) {

    Session session(proto.name());

    if (!proto.guid().empty())
        session.guid() = proto.guid();

    if (proto.has_objects())
        session.objects = Objects::from_proto(proto.objects());

    if (proto.has_tree())
        session.tree = Tree::pb_loads(proto.tree().SerializeAsString());

    if (proto.has_graph())
        session.graph = Graph::from_proto(proto.graph());

    if (proto.has_definitions())
        session.definitions = Objects::from_proto(proto.definitions());

    for (const session_proto::XformEntry& entry : proto.xforms())
        session.xforms[entry.guid()] = Xform::from_proto(entry.xform());

    for (const session_proto::InteractionEntry& entry : proto.interactions())
        for (const session_proto::Interaction& item : entry.interactions())
            session.interactions[entry.guid()].push_back(Interaction::from_proto(item));

    session.reindex();

    return session;
}

std::string Session::pb_dumps() {

    purge();
    std::string bytes;
    encode(bytes, to_proto());

    return bytes;
}

Session Session::pb_loads(const std::string& data) {

    session_proto::Session proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Session protobuf data");

    return from_proto(proto);
}

void Session::pb_dump(const std::string& filename) {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Session Session::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Session::str() const {

    const std::string bar(80, '=');

    return fmt::format(
        "{0}\nSpatial Hierarchy\n{0}\n{1}{0}\nElement Interactions\n{0}\n{2}\n{0}\n",
        bar,
        tree.str(),
        graph.str()
    );
}

std::string Session::repr() const {
    return fmt::format("Session(name={}, objects={}, tree={}, graph={})", name, objects.str(), tree.repr(), graph.repr());
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

    if (_is_live(guid) || definition_lookup.count(guid))
        return nullptr;

    const size_t slot = push(objects, collection, obj);
    _hold(guid, obj);
    const std::string attribute = type_prefix + "_" + item_name(obj);
    graph.add_node(guid, attribute);
    bvh_cache_dirty = true;
    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(guid);
    node_lookup[guid] = node;
    revision++;
    const std::shared_ptr<TreeNode> host = parent ? parent : tree.root();
    std::optional<std::string> parent_guid;

    if (host) {
        tree.add(node, host);
        parent_guid = host->name;
    }

    if (history.current) {

        const std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>(collection, false, slot, node);
        pin(objects, collection, slot, tomb);
        node->set_tomb(tomb);
        history.record(AddOp(guid, collection, parent_guid, static_cast<int>(node->at()), node, tomb), RECORD);
    }

    return node;
}

std::shared_ptr<TreeNode> Session::_node_of(const std::string& guid) const {

    std::shared_ptr<TreeNode> node = get_node(guid);

    return node ? node : std::make_shared<TreeNode>(guid);
}

bool Session::_twin(bool definition, const std::string& collection, size_t slot, const std::string& guid) const {

    const Objects& list = definition ? definitions : objects;
    const bool other = definition ? _is_live(guid) : definition_lookup.count(guid) > 0;

    if (other)
        return true;

    std::optional<Item> held;

    if (!definition)
        held = _item(guid);
    else if (auto it = definition_lookup.find(guid); it != definition_lookup.end())
        held = it->second;

    if (!held)
        return false;

    return collection_for(list, *held).first != collection || slot_of(list, collection, guid) != slot;
}

bool Session::_owns(const std::string& guid, const std::shared_ptr<TreeNode>& node) const {
    return !node || get_node(guid) == node;
}

bool Session::_is_live(const std::string& guid) const {
    return lookup.count(guid) || component_lookup.count(guid) || instance_lookup.count(guid);
}

std::optional<Item> Session::_item(const std::string& guid) const {

    if (auto it = lookup.find(guid); it != lookup.end())
        return it->second;

    if (auto it = component_lookup.find(guid); it != component_lookup.end())
        return it->second;

    if (auto it = instance_lookup.find(guid); it != instance_lookup.end())
        return it->second;

    return std::nullopt;
}

void Session::_hold(const std::string& guid, const Item& item) {

    if (const Geometry* geometry = std::get_if<Geometry>(&item))
        lookup[guid] = *geometry;
    else if (const Component* component = std::get_if<Component>(&item))
        component_lookup[guid] = *component;
    else
        instance_lookup[guid] = std::get<std::shared_ptr<InstanceRef>>(item);
}

std::shared_ptr<Tomb> Session::_tomb(const std::string& guid) {

    const std::optional<Item> item = _item(guid);

    if (!item)
        return nullptr;

    const std::string collection = collection_for(objects, *item).first;
    const std::optional<size_t> found = slot_of(objects, collection, guid);
    const size_t slot = found ? *found : push(objects, collection, *item);
    const std::shared_ptr<Tomb> pinned = tomb_at(objects, collection, slot);

    if (pinned && pinned->node)
        return pinned;

    std::shared_ptr<TreeNode> node = get_node(guid);

    if (node)
        node_lookup[guid] = node;
    else
        node = std::make_shared<TreeNode>(guid);

    const std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>(collection, false, slot, node);
    pin(objects, collection, slot, tomb);
    node->set_tomb(tomb);

    return tomb;
}

std::shared_ptr<Tomb> Session::_node_tomb(const std::shared_ptr<TreeNode>& node) {

    const std::shared_ptr<Tomb> pinned = node->get_tomb();

    if (pinned && pinned->collection.empty())
        return pinned;

    const std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>("", false, 0, node);
    node->set_tomb(tomb);

    return tomb;
}

std::shared_ptr<Tomb> Session::_half(bool definition, const std::string& collection, const std::string& guid) {

    std::optional<Item> item;

    if (definition) {

        auto found = definition_lookup.find(guid);

        if (found != definition_lookup.end())
            item = found->second;
    } else {
        item = _item(guid);
    }

    if (!item)
        return nullptr;

    const Objects& lists = definition ? definitions : objects;
    const std::optional<size_t> found = slot_of(lists, collection, guid);
    const size_t slot = found ? *found : push(lists, collection, *item);
    const std::shared_ptr<Tomb> pinned = tomb_at(lists, collection, slot);

    if (pinned && !pinned->node && pinned->definition == definition)
        return pinned;

    const std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>(collection, definition, slot, nullptr);
    pin(lists, collection, slot, tomb);

    return tomb;
}

void Session::_pair(
    const std::string& guid,
    const std::string& from,
    const std::string& to,
    std::shared_ptr<TreeNode> node,
    std::shared_ptr<Tomb> removed,
    std::shared_ptr<Tomb> added,
    size_t bytes
) {

    revision++;
    bvh_cache_dirty = true;

    if (!history.current) {
        history.dropped += 1;

        return;
    }

    const int index = node ? static_cast<int>(node->at()) : 0;
    std::optional<std::string> parent_guid;

    if (node && node->parent())
        parent_guid = node->parent()->name;

    history.record(RemoveOp(guid, from, parent_guid, index, node, std::move(removed)), bytes);
    history.record(AddOp(guid, to, parent_guid, index, std::move(node), std::move(added)), RECORD);
}

void Session::_label(const std::string& guid, const std::string& label) {

    if (graph.has_node(guid))
        graph.node_label(guid, label);
}

void Session::_queue(const std::shared_ptr<TreeNode>& parent) {

    if (parent->is_queued())
        return;

    parent->set_queued(true);
    _sweep.push_back(parent);
}

void Session::_kill(const std::shared_ptr<Tomb>& tomb) {

    if (tomb->collection.empty())
        return;

    const size_t slot = tomb->slot;
    const std::string& collection = tomb->collection;
    revision++;
    bvh_cache_dirty = true;

    if (tomb->definition) {

        const std::optional<Item> stored = item_at(definitions, collection, slot);

        if (!stored)
            return;

        const std::string guid = item_guid(*stored);
        const bool owner = !_twin(true, collection, slot, guid);
        auto held = definition_lookup.find(guid);

        if (owner && held != definition_lookup.end() && !same(held->second, *stored))
            store(definitions, collection, slot, held->second);

        flag(definitions, collection, slot, true);

        if (owner)
            definition_lookup.erase(guid);

        return;
    }

    const std::optional<Item> stored = item_at(objects, collection, slot);

    if (!stored)
        return;

    const std::string guid = item_guid(*stored);
    const std::optional<Item> held = _item(guid);
    const bool owner = !_twin(false, collection, slot, guid) && (held || slot_of(objects, collection, guid) == slot);

    if (owner && held && !same(*held, *stored))
        store(objects, collection, slot, *held);

    flag(objects, collection, slot, true);

    if (owner) {
        lookup.erase(guid);
        component_lookup.erase(guid);
        instance_lookup.erase(guid);
    }

    const std::shared_ptr<TreeNode>& node = tomb->node;

    if (!node)
        return;

    const std::shared_ptr<TreeNode> parent = node->parent();
    node->set_dead(true);
    node->set_tomb(tomb);
    auto indexed = node_lookup.find(guid);

    if (indexed != node_lookup.end() && indexed->second == node)
        node_lookup.erase(indexed);

    if (parent)
        _queue(parent);

    if (!owner)
        return;

    tomb->xform.reset();
    auto placed = xforms.find(guid);

    if (placed != xforms.end()) {
        tomb->xform = placed->second;
        xforms.erase(placed);
    }

    std::optional<std::pair<Vertex, std::vector<Edge>>> taken = graph.take_node(guid);

    if (!taken)
        return;

    for (const Edge& edge : taken->second) {

        if (!edge.has_guid())
            continue;

        auto list = interactions.find(edge.guid());

        if (list == interactions.end())
            continue;

        tomb->interactions[edge.guid()] = std::move(list->second);
        interactions.erase(list);
    }

    tomb->vertex = std::move(taken->first);
    tomb->edges = std::move(taken->second);
}

void Session::_revive(const std::shared_ptr<Tomb>& tomb) {

    if (tomb->collection.empty())
        return;

    const size_t slot = tomb->slot;
    const std::string& collection = tomb->collection;
    revision++;
    bvh_cache_dirty = true;

    if (tomb->definition) {

        const std::optional<Item> item = item_at(definitions, collection, slot);

        if (!item)
            return;

        const std::string guid = item_guid(*item);

        if (!_twin(true, collection, slot, guid)) {
            flag(definitions, collection, slot, false);
            definition_lookup[guid] = std::get<Geometry>(*item);
        }

        return;
    }

    const std::optional<Item> item = item_at(objects, collection, slot);

    if (!item)
        return;

    const std::string guid = item_guid(*item);

    if (_twin(false, collection, slot, guid))
        return;

    flag(objects, collection, slot, false);
    _hold(guid, *item);
    const std::shared_ptr<TreeNode>& node = tomb->node;

    if (!node) {
        _label(guid, prefix_of(collection) + "_" + item_name(*item));

        return;
    }

    node->set_dead(false);

    if (node->parent())
        node_lookup[guid] = node;

    if (tomb->xform) {
        xforms[guid] = *tomb->xform;
        tomb->xform.reset();
    }

    if (!tomb->vertex)
        return;

    Vertex vertex = std::move(*tomb->vertex);
    std::vector<Edge> edges = std::move(tomb->edges);
    tomb->vertex.reset();
    tomb->edges.clear();
    std::vector<std::pair<std::string, std::string>> ids;

    for (const Edge& edge : edges)
        if (edge.has_guid())
            ids.emplace_back(edge.other_vertex(guid), edge.guid());

    graph.put_node(std::move(vertex), std::move(edges));

    for (const std::pair<std::string, std::string>& id : ids) {

        auto neighbours = graph.edges.find(guid);

        if (neighbours == graph.edges.end())
            continue;

        auto back = neighbours->second.find(id.first);

        if (back == neighbours->second.end() || !back->second.has_guid() || back->second.guid() != id.second)
            continue;

        auto parked = tomb->interactions.find(id.second);

        if (parked == tomb->interactions.end())
            continue;

        interactions[id.second] = std::move(parked->second);
        tomb->interactions.erase(parked);
    }
}

void Session::_swap(const std::string& guid, const Item& obj, const Entry& entry) {

    const std::pair<std::string, std::string> target = collection_for(objects, obj);

    if (entry.definition) {

        const Geometry* geometry = std::get_if<Geometry>(&obj);

        if (!geometry || _is_live(guid) || slot_of(definitions, target.first, guid) != entry.tomb->slot)
            return;

        store(definitions, target.first, entry.tomb->slot, obj);
        definition_lookup[guid] = *geometry;
    } else {

        if (definition_lookup.count(guid) || !_owns(guid, entry.node))
            return;

        const std::optional<size_t> slot = slot_of(objects, target.first, guid);

        if (!slot)
            return;

        store(objects, target.first, *slot, obj);
        _label(guid, target.second + "_" + item_name(obj));
        _hold(guid, obj);
    }

    revision++;
    bvh_cache_dirty = true;
}

void Session::_tree(const TreeOp& op, bool back) {

    const std::string& name = back ? op.name_before : op.name_after;
    const std::optional<Color>& color = back ? op.color_before : op.color_after;
    const bool dead = back ? op.dead_before : op.dead_after;
    const bool was = op.node->is_dead();
    const bool live = _is_live(name);

    if (op.ghost) {

        const std::shared_ptr<TreeNode> from = op.node->parent();
        TreeNode::swap(op.node, op.ghost);
        op.ghost->set_tomb(op.tomb);

        if (from)
            _queue(from);
    }

    const std::shared_ptr<TreeNode> parent = op.node->parent();
    op.node->name = name;
    op.node->color = color;
    op.node->set_dead(dead);
    revision++;

    if (dead && !was) {

        op.node->set_tomb(op.tomb);

        if (parent)
            _queue(parent);
    }

    if (dead && !was && !live) {

        op.tomb->xform.reset();
        auto placed = xforms.find(name);

        if (placed != xforms.end()) {
            op.tomb->xform = placed->second;
            xforms.erase(placed);
        }
    }

    if (was && !dead && !live && op.tomb->xform) {
        xforms[name] = *op.tomb->xform;
        op.tomb->xform.reset();
    }

    if (!live)
        return;

    if (!dead)
        node_lookup[name] = op.node;
    else if (auto held = node_lookup.find(name); held != node_lookup.end() && held->second == op.node)
        node_lookup.erase(held);
}

void Session::_place(const std::string& guid, const std::optional<Xform>& xform, const std::shared_ptr<TreeNode>& node) {

    if (!_owns(guid, node))
        return;

    if (xform)
        xforms[guid] = *xform;
    else
        xforms.erase(guid);

    bvh_cache_dirty = true;
    revision++;
}

std::vector<std::pair<std::string, Xform>> Session::_xforms_ordered() const {

    std::vector<std::pair<std::string, Xform>> ordered;
    std::map<std::string, Xform> rest;

    for (const std::pair<const std::string, Xform>& entry : xforms)
        if (!entry.second.is_identity())
            rest.emplace(entry.first, entry.second);

    for (const std::string& obj_guid : order()) {

        auto it = rest.find(obj_guid);

        if (it == rest.end())
            continue;

        ordered.emplace_back(obj_guid, it->second);
        rest.erase(it);
    }

    for (const std::pair<const std::string, Xform>& entry : rest)
        ordered.emplace_back(entry.first, entry.second);

    return ordered;
}

size_t Session::_purge(size_t work) {

    if (!_purging) {
        _purging = 0;
        history.dropped = 0;
    }

    while (work > 0 && _purging) {

        const size_t phase = *_purging;

        if (phase < 26) {

            size_t spent = 0;
            bool done = true;

            with_collection(phase < 13 ? objects : definitions, COLLECTIONS[phase % 13].first, [&](auto& items) {
                if (items.number_of_dead() > 0 || items.is_compacting()) {
                    spent = items.compact_step(work);
                    done = !items.is_compacting();
                }
            });
            work -= std::min(spent, work);

            if (done)
                _purging = phase + 1;

            continue;
        }

        if (_sweep.empty()) {
            _sweep.swap(_pinned);
            _purging.reset();
            break;
        }

        const std::shared_ptr<TreeNode> parent = _sweep.back().lock();

        if (!parent) {
            _sweep.pop_back();
            --work;
            continue;
        }

        work -= std::clamp<size_t>(parent->compact_step(work), 1, work);

        if (parent->is_compacting())
            continue;

        _sweep.pop_back();

        if (parent->has_dead())
            _pinned.push_back(parent);
        else
            parent->set_queued(false);
    }

    return work;
}

bool Session::_write(Checkpoint& writer, size_t work) const {

    while (work > 0) {

        size_t spent = 1;

        if (writer.phase == HEAD) {
            session_proto::Session head;
            head.set_name(name);

            if (has_guid())
                head.set_guid(guid());

            encode(writer.sections[0], head);
            encode(writer.sections[1], objects_head(objects));
            writer.phase = OBJECTS;
        } else if (writer.phase < TREE) {
            spent = _write_list(writer, false, work);
        } else if (writer.phase == TREE) {
            spent = _write_tree(writer, work);
        } else if (writer.phase < ORDERED) {
            spent = _write_graph(writer, work);
        } else if (writer.phase < REST) {
            spent = _write_ordered(writer, work);
        } else if (writer.phase == REST) {
            spent = _write_rest(writer, work);
        } else if (writer.phase < INTERACTIONS) {
            spent = _write_list(writer, true, work);
        } else if (writer.phase == INTERACTIONS) {
            spent = _write_interactions(writer, work);
        } else {
            return _assemble(writer, work);
        }

        work -= std::clamp<size_t>(spent, 1, work);
    }

    return false;
}

size_t Session::_write_list(Checkpoint& writer, bool definition, size_t work) const {

    const size_t list = writer.phase - (definition ? DEFINITIONS : OBJECTS);
    std::string& buffer = writer.sections[definition ? 5 : 1];
    const size_t start = writer.cursor;
    size_t end = start;
    size_t total = 0;

    with_collection(definition ? definitions : objects, COLLECTIONS[list].first, [&](const auto& items) {
        using E = typename std::decay_t<decltype(items)>::value_type;
        total = items.number_of_slots();
        end = std::min(total, start + std::min(work, total));

        for (size_t slot = start; slot < end; ++slot) {

            if (items.is_dead(slot))
                continue;

            if constexpr (std::is_same_v<E, Component>)
                encode(buffer, items.get_item(slot).to_proto(), TAGS.lists[list]);
            else
                encode(buffer, items.get_item(slot)->to_proto(), TAGS.lists[list]);
        }
    });
    writer.cursor = end;

    if (end >= total) {
        writer.cursor = 0;
        writer.phase++;
    }

    return end - start;
}

size_t Session::_write_tree(Checkpoint& writer, size_t work) const {

    if (writer.cursor == 0) {

        writer.cursor = 1;
        session_proto::Tree head;

        if (tree.has_guid())
            head.set_guid(tree.guid());

        head.set_name(tree.name);
        encode(writer.sections[2], head);

        if (const std::shared_ptr<TreeNode> root = tree.root())
            writer.stack.push_back(Frame{root, 0, root->_head()});
    }

    size_t spent = 0;

    while (spent < work && !writer.stack.empty()) {

        Frame& top = writer.stack.back();
        ++spent;

        if (top.next < top.node->_children.size()) {

            const std::shared_ptr<TreeNode> child = top.node->_children[top.next++];

            if (!child->_dead)
                writer.stack.push_back(Frame{child, 0, child->_head()});

            continue;
        }

        Frame done = std::move(top);
        writer.stack.pop_back();
        done.bytes += done.node->_tail();
        const bool root = writer.stack.empty();
        std::string& parent = root ? writer.sections[2] : writer.stack.back().bytes;
        frame(parent, root ? TAGS.root : TAGS.children, done.bytes.size());
        parent += done.bytes;
    }

    if (writer.stack.empty()) {
        writer.cursor = 0;
        writer.phase = VERTICES;
    }

    return spent;
}

size_t Session::_write_graph(Checkpoint& writer, size_t work) const {

    std::string& buffer = writer.sections[3];
    const bool resumed = writer.cursor > 0;
    size_t spent = 0;
    bool more = false;

    if (writer.phase == VERTICES) {

        if (!resumed)
            encode(buffer, graph_head(graph));

        auto it = resumed ? graph.vertices.upper_bound(writer.key) : graph.vertices.begin();

        for (; it != graph.vertices.end() && spent < work; ++it, ++spent) {
            session_proto::Graph entry;
            Graph::_to_proto(it->second, (*entry.mutable_vertices())[it->first]);
            encode(buffer, entry);
            writer.key = it->first;
        }

        more = it != graph.vertices.end();
    } else {

        auto it = resumed ? graph.edges.upper_bound(writer.key) : graph.edges.begin();

        for (; it != graph.edges.end() && spent < work; ++it) {

            for (const std::pair<const std::string, Edge>& neighbor : it->second) {

                if (it->first > neighbor.first)
                    continue;

                session_proto::Graph entry;
                Graph::_to_proto(neighbor.second, *entry.add_edges());
                encode(buffer, entry);
            }

            writer.key = it->first;
            spent += std::max<size_t>(it->second.size(), 1);
        }

        more = it != graph.edges.end();
    }

    if (more) {
        writer.cursor++;
        return spent;
    }

    if (writer.phase == EDGES)
        encode(buffer, graph_tail(graph));

    writer.key.clear();
    writer.cursor = 0;
    writer.phase++;

    return spent;
}

size_t Session::_write_ordered(Checkpoint& writer, size_t work) const {

    const size_t start = writer.cursor;
    size_t end = start;
    size_t total = 0;

    with_collection(objects, COLLECTIONS[writer.phase - ORDERED].first, [&](const auto& items) {
        total = items.number_of_slots();
        end = std::min(total, start + std::min(work, total));

        for (size_t slot = start; slot < end; ++slot) {

            if (items.is_dead(slot))
                continue;

            const std::string& guid = guid_of(items.get_item(slot));
            auto it = xforms.find(guid);

            if (it == xforms.end() || !writer.seen.insert(guid).second || it->second.is_identity())
                continue;

            session_proto::Session entry;
            session_proto::XformEntry* item = entry.add_xforms();
            item->set_guid(guid);
            *item->mutable_xform() = it->second.to_proto();
            encode(writer.sections[4], entry);
        }
    });
    writer.cursor = end;

    if (end >= total) {
        writer.cursor = 0;
        writer.phase++;
    }

    return end - start;
}

size_t Session::_write_rest(Checkpoint& writer, size_t work) const {

    size_t spent = 0;

    if (writer.cursor == 0 && writer.seen.size() < xforms.size()) {

        for (const std::pair<const std::string, Xform>& entry : xforms)
            if (!writer.seen.count(entry.first) && !entry.second.is_identity())
                writer.rest.push_back(entry.first);

        std::sort(writer.rest.begin(), writer.rest.end());
        spent = xforms.size();
    }

    const size_t start = writer.cursor;
    const size_t end = std::min(writer.rest.size(), start + std::min(work, writer.rest.size()));

    for (size_t i = start; i < end; ++i) {
        session_proto::Session entry;
        session_proto::XformEntry* item = entry.add_xforms();
        item->set_guid(writer.rest[i]);
        *item->mutable_xform() = xforms.at(writer.rest[i]).to_proto();
        encode(writer.sections[4], entry);
    }

    writer.cursor = end;

    if (end < writer.rest.size())
        return spent + end - start;

    writer.cursor = 0;
    writer.phase = INTERACTIONS;

    if (!definition_lookup.empty()) {
        encode(writer.sections[5], objects_head(definitions));
        writer.phase = DEFINITIONS;
    }

    return spent + end - start;
}

size_t Session::_write_interactions(Checkpoint& writer, size_t work) const {

    size_t spent = 0;
    auto it = writer.cursor > 0 ? interactions.upper_bound(writer.key) : interactions.begin();

    for (; it != interactions.end() && spent < work; ++it, ++spent) {

        session_proto::Session entry;
        session_proto::InteractionEntry* item = entry.add_interactions();
        item->set_guid(it->first);

        for (const std::shared_ptr<Interaction>& interaction : it->second)
            *item->add_interactions() = interaction->to_proto();

        encode(writer.sections[6], entry);
        writer.key = it->first;
    }

    if (it != interactions.end()) {
        writer.cursor++;
        return spent;
    }

    writer.cursor = 0;
    writer.phase = ASSEMBLY;

    return spent;
}

bool Session::_assemble(Checkpoint& writer, size_t work) const {

    if (writer.phase == ASSEMBLY) {

        std::vector<std::string> pieces;
        size_t total = 0;

        for (size_t i = 0; i < writer.sections.size(); ++i) {

            if (i == 5 && definition_lookup.empty())
                continue;

            if (TAGS.sections[i] > 0) {
                pieces.emplace_back();
                frame(pieces.back(), TAGS.sections[i], writer.sections[i].size());
            }

            pieces.push_back(std::move(writer.sections[i]));
        }

        for (const std::string& piece : pieces)
            total += piece.size();

        writer.out.reserve(total);
        writer.sections = std::move(pieces);
        writer.phase++;
    }

    size_t budget = work > SIZE_MAX / 1024 ? SIZE_MAX : work * 1024;
    size_t skip = writer.out.size();
    size_t total = 0;

    for (const std::string& piece : writer.sections) {

        total += piece.size();

        if (skip >= piece.size()) {
            skip -= piece.size();
            continue;
        }

        const size_t take = std::min(piece.size() - skip, budget);
        writer.out.append(piece, skip, take);
        budget -= take;
        skip = 0;
    }

    return writer.out.size() == total;
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

    std::unordered_map<std::string, OBB> local;

    for (const std::shared_ptr<InstanceRef>& instance : *objects.instances) {

        auto definition = definition_lookup.find(instance->definition_guid);

        if (definition == definition_lookup.end())
            continue;

        auto box = local.find(definition->first);

        if (box == local.end())
            box = local.emplace(definition->first, compute_bounding_box(definition->second, Xform::identity())).first;

        OBB placed = box->second;
        auto wit = world.find(instance->guid());

        if (wit != world.end())
            placed.transform(wit->second);

        boxes.push_back(placed);
        guids.push_back(instance->guid());
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

    if (const std::shared_ptr<Polyline>* polyline = std::get_if<std::shared_ptr<Polyline>>(&geometry))
        return ray_polyline(ray, **polyline, tolerance);

    if (const std::shared_ptr<PointCloud>* pointcloud = std::get_if<std::shared_ptr<PointCloud>>(&geometry))
        return ray_pointcloud(ray, **pointcloud, tolerance);

    if (const std::shared_ptr<Mesh>* mesh = std::get_if<std::shared_ptr<Mesh>>(&geometry))
        return ray_mesh(ray, **mesh, tolerance, placement);

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
