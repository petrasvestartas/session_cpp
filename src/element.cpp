#include "element.h"
#include "element.pb.h"
#include <fstream>
#include <cmath>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Element
///////////////////////////////////////////////////////////////////////////////////////////

Element::Element(const std::string& name) : name(name) {}

Element::Element(const Mesh& geometry, const std::string& name)
    : name(name), _geometry(geometry) {}

Element::Element(const BRep& geometry, const std::string& name)
    : name(name), _geometry(geometry) {}

Element::Element(const Element& other)
    : name(other.name),
      _geometry(other._geometry), _is_dirty(true),
      _geometry_ops(other._geometry_ops), _features(other._features),
      _insertion_vectors(other._insertion_vectors), _dimensions(other._dimensions) {}

Element& Element::operator=(const Element& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        _geometry = other._geometry;
        _geometry_ops = other._geometry_ops;
        _features = other._features;
        _insertion_vectors = other._insertion_vectors;
        _dimensions = other._dimensions;
        _is_dirty = true;
        _aabb.reset();
        _obb.reset();
        _collision_mesh.reset();
        _point.reset();
        _polylines.reset();
        _planes.reset();
        _edge_vectors.reset();
        _axis.reset();
    }
    return *this;
}

bool Element::has_geometry() const {
    return !std::holds_alternative<std::monostate>(_geometry);
}

std::string Element::geometry_type_name() const {
    if (std::holds_alternative<Mesh>(_geometry)) return "Mesh";
    if (std::holds_alternative<BRep>(_geometry)) return "BRep";
    return "None";
}

ElementGeometry Element::session_geometry(const Xform& xform) const {
    if (!has_geometry()) return std::monostate{};
    auto geo = _geometry;
    if (auto* mesh = std::get_if<Mesh>(&geo)) {
        *mesh = apply_geometry_ops(*mesh);
        if (!xform.is_identity()) {
            mesh->transform(xform);
        }
    } else if (auto* brep = std::get_if<BRep>(&geo)) {
        if (!xform.is_identity()) {
            brep->transform(xform);
        }
    }
    return geo;
}

OBB Element::aabb() {
    if (_is_dirty || !_aabb.has_value()) _aabb = compute_aabb();
    return _aabb.value();
}

OBB Element::obb() {
    if (_is_dirty || !_obb.has_value()) _obb = compute_obb();
    return _obb.value();
}

Mesh Element::collision_mesh() {
    if (_is_dirty || !_collision_mesh.has_value()) _collision_mesh = compute_collision_mesh();
    return _collision_mesh.value();
}

Point Element::point() {
    if (_is_dirty || !_point.has_value()) _point = compute_point();
    return _point.value();
}

std::vector<Polyline> Element::polylines() {
    if (_is_dirty || !_polylines.has_value()) _polylines = compute_polylines();
    return _polylines.value();
}

std::vector<Plane> Element::planes() {
    if (_is_dirty || !_planes.has_value()) _planes = compute_planes();
    return _planes.value();
}

std::vector<Vector> Element::edge_vectors() {
    if (_is_dirty || !_edge_vectors.has_value()) _edge_vectors = compute_edge_vectors();
    return _edge_vectors.value();
}

std::optional<Line> Element::axis() {
    if (_is_dirty || !_axis.has_value()) _axis = compute_axis();
    return _axis;
}

void Element::add_geometry_op(std::function<Mesh(Mesh)> f) {
    _geometry_ops.push_back(std::move(f));
    _is_dirty = true;
}

void Element::place(const Xform& xform) {
    _geometry = session_geometry(xform);
    _is_dirty = true;
}

void Element::set_geometry(const Mesh& geo) {
    _geometry = geo;
    _is_dirty = true;
}

void Element::set_geometry(const BRep& geo) {
    _geometry = geo;
    _is_dirty = true;
}

void Element::set_polylines(std::vector<Polyline> polys) {
    _polylines = std::move(polys);
}

void Element::set_planes(std::vector<Plane> plns) {
    _planes = std::move(plns);
}

void Element::reset() {
    _is_dirty = true;
    _aabb.reset();
    _obb.reset();
    _collision_mesh.reset();
    _point.reset();
    _polylines.reset();
    _planes.reset();
    _edge_vectors.reset();
    _axis.reset();
}

Element Element::duplicate() const {
    Element result(*this);

    return result;
}

bool Element::operator==(const Element& other) const {
    return name == other.name && geometry_type_name() == other.geometry_type_name();
}

bool Element::operator!=(const Element& other) const { return !(*this == other); }

std::string Element::str() const {
    return fmt::format("Element({}, {})", name, geometry_type_name());
}

std::string Element::repr() const {
    return fmt::format("Element({}, {}, {})", guid(), name, geometry_type_name());
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - Computation
///////////////////////////////////////////////////////////////////////////////////////////

OBB Element::compute_aabb() {
    auto geo = session_geometry(Xform::identity());
    if (std::holds_alternative<std::monostate>(geo))
        return OBB::from_point(Point(0, 0, 0), 0.0);
    return obb_from_geometry(geo);
}

OBB Element::compute_obb() {
    auto geo = session_geometry(Xform::identity());
    if (std::holds_alternative<std::monostate>(geo))
        return OBB::from_point(Point(0, 0, 0), 0.0);
    return obb_from_geometry(geo);
}

Mesh Element::compute_collision_mesh() {
    auto geo = session_geometry(Xform::identity());
    if (auto* mesh = std::get_if<Mesh>(&geo)) return *mesh;
    return Mesh();
}

Point Element::compute_point() {
    auto geo = session_geometry(Xform::identity());
    if (auto* mesh = std::get_if<Mesh>(&geo)) {
        if (mesh->vertex.empty()) return Point(0, 0, 0);
        double sx = 0, sy = 0, sz = 0;
        for (const auto& [k, v] : mesh->vertex) {
            sx += v.x; sy += v.y; sz += v.z;
        }
        double n = static_cast<double>(mesh->vertex.size());
        return Point(sx / n, sy / n, sz / n);
    }
    if (auto* brep = std::get_if<BRep>(&geo)) {
        if (brep->m_vertices.empty()) return Point(0, 0, 0);
        double sx = 0, sy = 0, sz = 0;
        for (const auto& p : brep->m_vertices) {
            sx += p[0]; sy += p[1]; sz += p[2];
        }
        double n = static_cast<double>(brep->m_vertices.size());
        return Point(sx / n, sy / n, sz / n);
    }
    return Point(0, 0, 0);
}

std::vector<Polyline> Element::compute_polylines() const { return {}; }
std::vector<Plane> Element::compute_planes() const { return {}; }
std::vector<Vector> Element::compute_edge_vectors() const { return {}; }
std::optional<Line> Element::compute_axis() const { return std::nullopt; }

Mesh Element::apply_geometry_ops(Mesh geo) const {
    for (const auto& f : _geometry_ops) geo = f(geo);
    return geo;
}

bool ElementFeature::operator==(const ElementFeature& other) const {
    return name == other.name && feature_type == other.feature_type
        && face_index == other.face_index && outlines == other.outlines;
}

std::string ElementFeature::str() const {
    return fmt::format("ElementFeature({}, face {}, {} outline(s))",
                       feature_type, face_index, outlines.size());
}

OBB Element::obb_from_geometry(const ElementGeometry& geo) {
    double inflate = 0.0;
    if (auto* mesh = std::get_if<Mesh>(&geo)) {
        std::vector<Point> points;
        for (const auto& [k, v] : mesh->vertex)
            points.push_back(v.position());
        if (points.empty()) return OBB::from_point(Point(0, 0, 0), inflate);
        return OBB::from_points(points, inflate);
    }
    if (auto* brep = std::get_if<BRep>(&geo)) {
        if (brep->m_vertices.empty()) return OBB::from_point(Point(0, 0, 0), inflate);
        return OBB::from_points(brep->m_vertices, inflate);
    }
    return OBB::from_point(Point(0, 0, 0), inflate);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - JSON
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Element::jsondump() const {
    nlohmann::ordered_json geo_data = nullptr;
    std::string geo_type = "None";
    if (auto* mesh = std::get_if<Mesh>(&_geometry)) {
        geo_type = "Mesh";
        geo_data = mesh->jsondump();
    } else if (auto* brep = std::get_if<BRep>(&_geometry)) {
        geo_type = "BRep";
        geo_data = brep->jsondump();
    }
    return nlohmann::ordered_json{
        {"geometry_data", geo_data},
        {"geometry_type", geo_type},
        {"guid", guid()},
        {"name", name},
        {"type", "Element"},
    };
}

Element Element::jsonload(const nlohmann::json& data) {
    std::string geo_type = data.value("geometry_type", "None");
    Element elem;
    if (geo_type == "Mesh" && data.contains("geometry_data") && !data["geometry_data"].is_null()) {
        elem._geometry = Mesh::jsonload(data["geometry_data"]);
    } else if (geo_type == "BRep" && data.contains("geometry_data") && !data["geometry_data"].is_null()) {
        elem._geometry = BRep::jsonload(data["geometry_data"]);
    }
    elem.guid() = data.value("guid", elem.guid());
    elem.name = data.value("name", elem.name);
    return elem;
}

std::string Element::file_json_dumps() const { return jsondump().dump(); }

Element Element::file_json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

void Element::file_json_dump(const std::string& path) const {
    std::ofstream file(path);
    file << jsondump().dump(2);
}

Element Element::file_json_load(const std::string& path) {
    std::ifstream file(path);
    return jsonload(nlohmann::json::parse(file));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

std::string Element::pb_dumps() const {
    session_proto::Element proto;
    proto.set_guid(guid());
    proto.set_name(name);
    if (auto* mesh = std::get_if<Mesh>(&_geometry)) {
        proto.set_geometry_type("Mesh");
        proto.set_geometry_data(mesh->pb_dumps());
    } else if (auto* brep = std::get_if<BRep>(&_geometry)) {
        proto.set_geometry_type("BRep");
        proto.set_geometry_data(brep->pb_dumps());
    } else {
        proto.set_geometry_type("None");
    }
    // Both empty for a plain Element, and proto3 does not emit empty scalars - so the bytes
    // of a base element are exactly what they were before the registry existed.
    proto.set_element_type(element_type_name());
    proto.set_element_data(element_data_dumps());

    for (const auto& v : _insertion_vectors) {
        proto.add_insertion_vectors()->ParseFromString(v.pb_dumps());
    }
    if (_dimensions.has_value()) {
        proto.mutable_dimensions()->ParseFromString(_dimensions->pb_dumps());
    }
    for (const auto& f : _features) {
        auto* pf = proto.add_features();
        pf->set_guid(f.guid());
        pf->set_name(f.name);
        pf->set_feature_type(f.feature_type);
        pf->set_face_index(f.face_index);
        for (const auto& o : f.outlines) {
            pf->add_outlines()->ParseFromString(o.pb_dumps());
        }
    }
    return proto.SerializeAsString();
}

Element Element::pb_loads(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);
    Element elem;
    elem.guid() = proto.guid();
    elem.name = proto.name();
    std::string geo_type = proto.geometry_type();
    if (geo_type == "Mesh" && !proto.geometry_data().empty()) {
        elem._geometry = Mesh::pb_loads(proto.geometry_data());
    } else if (geo_type == "BRep" && !proto.geometry_data().empty()) {
        elem._geometry = BRep::pb_loads(proto.geometry_data());
    }

    for (const auto& v : proto.insertion_vectors()) {
        elem._insertion_vectors.push_back(Vector::pb_loads(v.SerializeAsString()));
    }
    // has_dimensions, not an emptiness check: (0,0,0) is a legitimate authored value and
    // must not be confused with "never authored", which is why this member is optional.
    if (proto.has_dimensions()) {
        elem._dimensions = Vector::pb_loads(proto.dimensions().SerializeAsString());
    }
    for (const auto& f : proto.features()) {
        ElementFeature feature;
        // Assigned, not minted: a feature that comes back off the wire is the SAME feature the
        // package wrote, and anything holding its guid must still find it. An empty guid on the
        // wire (a file written before features had one) leaves the lazy mint to whoever asks.
        if (!f.guid().empty()) feature.guid() = f.guid();
        feature.name = f.name();
        feature.feature_type = f.feature_type();
        feature.face_index = f.face_index();
        for (const auto& o : f.outlines()) {
            feature.outlines.push_back(Polyline::pb_loads(o.SerializeAsString()));
        }
        elem._features.push_back(std::move(feature));
    }
    return elem;
}

void Element::pb_dump(const std::string& path) const {
    std::string data = pb_dumps();
    std::ofstream file(path, std::ios::binary);
    file.write(data.data(), data.size());
}

Element Element::pb_load(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - polymorphic registry
///////////////////////////////////////////////////////////////////////////////////////////

// Function-local static, not a namespace-scope one: a downstream package registers from a
// static initializer, and a namespace-scope map might not be constructed yet when it runs.
// This way the map is built on first use, whenever that is.
static std::map<std::string, Element::Factory>& element_registry() {
    static std::map<std::string, Element::Factory> registry;
    return registry;
}

void Element::register_type(const std::string& type_name, Element::Factory factory) {
    if (type_name.empty() || !factory) { return; }
    element_registry()[type_name] = std::move(factory);
}

bool Element::is_registered(const std::string& type_name) {
    return element_registry().count(type_name) > 0;
}

std::vector<std::string> Element::registered_types() {
    std::vector<std::string> names;
    names.reserve(element_registry().size());
    for (const auto& [name, _] : element_registry()) { names.push_back(name); }
    return names;
}

std::shared_ptr<Element> Element::pb_loads_shared(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);

    const std::string& type_name = proto.element_type();
    if (!type_name.empty()) {
        auto it = element_registry().find(type_name);
        if (it != element_registry().end()) {
            if (auto derived = it->second(data)) { return derived; }
            // A factory that returns null is a bug in that package, not a corrupt file -
            // fall through to the base so one bad type cannot take the Session with it.
        }
    }

    // No type, no factory, or a factory that declined: keep the geometry and the identity.
    auto base = std::make_shared<Element>(pb_loads(data));
    base->guid() = proto.guid();
    return base;
}

std::ostream& operator<<(std::ostream& os, const Element& e) { return os << e.str(); }

///////////////////////////////////////////////////////////////////////////////////////////

} // namespace session_cpp
