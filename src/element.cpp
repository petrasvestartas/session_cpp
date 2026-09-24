#include "element.h"
#include "element.pb.h"
#include <fstream>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Hex encoding
// ═══════════════════════════════════════════════════════════════════════════

/// Encode bytes as hex text, since element_data is opaque and JSON carries no bytes.
static std::string to_hex(const std::string& bytes) {

    static const std::string digits = "0123456789abcdef";
    std::string out;
    out.reserve(bytes.size() * 2);

    for (unsigned char c : bytes) {
        out += digits[c >> 4];
        out += digits[c & 15];
    }

    return out;
}

/// Decode hex text back to bytes.
static std::string from_hex(const std::string& hex) {

    std::string out;
    out.reserve(hex.size() / 2);

    for (size_t i = 0; i + 1 < hex.size(); i += 2)
        out += static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16));

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature
// ═══════════════════════════════════════════════════════════════════════════

ElementFeature& ElementFeature::operator=(const ElementFeature& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    name = other.name;
    feature_type = other.feature_type;
    face_index = other.face_index;
    outlines = other.outlines;
    visible = other.visible;

    return *this;
}

bool ElementFeature::operator==(const ElementFeature& other) const {
    return name == other.name && feature_type == other.feature_type && face_index == other.face_index &&
        outlines == other.outlines && visible == other.visible;
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature - JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json ElementFeature::jsondump() const {

    nlohmann::ordered_json outs = nlohmann::ordered_json::array();

    for (const Polyline& o : outlines)
        outs.push_back(o.jsondump());

    return nlohmann::ordered_json{
        {"face_index", face_index},
        {"feature_type", feature_type},
        {"guid", guid()},
        {"name", name},
        {"outlines", outs},
        {"type", "ElementFeature"},
        {"visible", visible},
    };
}

ElementFeature ElementFeature::jsonload(const nlohmann::json& data) {

    ElementFeature f;
    f.face_index = data.value("face_index", -1);
    f.feature_type = data.value("feature_type", std::string());

    const std::string g = data.value("guid", std::string());

    if (!g.empty())
        f.guid() = g;

    f.name = data.value("name", std::string());

    for (const nlohmann::json& o : data.value("outlines", nlohmann::json::array()))
        f.outlines.push_back(Polyline::jsonload(o));

    f.visible = data.value("visible", true);

    return f;
}

std::string ElementFeature::file_json_dumps() const {
    return jsondump().dump();
}

ElementFeature ElementFeature::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void ElementFeature::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(2);
}

ElementFeature ElementFeature::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature - Protobuf
// ═══════════════════════════════════════════════════════════════════════════

session_proto::ElementFeature ElementFeature::to_proto() const {

    session_proto::ElementFeature proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_feature_type(feature_type);
    proto.set_face_index(face_index);

    for (const Polyline& o : outlines)
        *proto.add_outlines() = o.to_proto();

    if (!visible)
        proto.set_visible(false);

    return proto;
}

ElementFeature ElementFeature::from_proto(const session_proto::ElementFeature& proto) {

    ElementFeature f;

    if (!proto.guid().empty())
        f.guid() = proto.guid();

    f.name = proto.name();
    f.feature_type = proto.feature_type();
    f.face_index = proto.face_index();

    for (const session_proto::Polyline& o : proto.outlines())
        f.outlines.push_back(Polyline::from_proto(o));

    f.visible = !proto.has_visible() || proto.visible();

    return f;
}

std::string ElementFeature::pb_dumps() const {
    return to_proto().SerializeAsString();
}

ElementFeature ElementFeature::pb_loads(const std::string& data) {

    session_proto::ElementFeature proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse ElementFeature protobuf data");

    return from_proto(proto);
}

void ElementFeature::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

ElementFeature ElementFeature::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature - String
// ═══════════════════════════════════════════════════════════════════════════

std::string ElementFeature::str() const {
    return fmt::format("ElementFeature({}, face {}, {} outline(s))", feature_type, face_index, outlines.size());
}

std::string ElementFeature::repr() const {
    return str();
}

std::ostream& operator<<(std::ostream& os, const ElementFeature& f) {
    return os << f.str();
}

// ═══════════════════════════════════════════════════════════════════════════
// Element
// ═══════════════════════════════════════════════════════════════════════════

Element::Element(const std::string& name) : name(name) {}

Element::Element(const Mesh& geometry, const std::string& name) : _geometry_mesh(geometry), name(name) {}

Element::Element(const BRep& geometry, const std::string& name) : _geometry_brep(geometry), name(name) {}

Element::Element(const Element& other)
    : _geometry_mesh(other._geometry_mesh), _geometry_brep(other._geometry_brep), _geometry_synced(other._geometry_synced), _geometry_ops(other._geometry_ops),
      _features(other._features), _insertion_vectors(other._insertion_vectors), _dimensions(other._dimensions),
      _element_type(other._element_type), _element_data(other._element_data), name(other.name) {}

Element& Element::operator=(const Element& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    name = other.name;
    _geometry_mesh = other._geometry_mesh;
    _geometry_brep = other._geometry_brep;
    _geometry_ops = other._geometry_ops;
    _features = other._features;
    _insertion_vectors = other._insertion_vectors;
    _dimensions = other._dimensions;
    _element_type = other._element_type;
    _element_data = other._element_data;
    _geometry_synced = other._geometry_synced;
    reset();

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Accessors
// ═══════════════════════════════════════════════════════════════════════════

bool Element::has_geometry() const {

    ensure_geometry();

    return _geometry_mesh.has_value() || _geometry_brep.has_value();
}

std::string Element::geometry_type_name() const {

    ensure_geometry();

    if (_geometry_mesh.has_value())
        return "Mesh";

    if (_geometry_brep.has_value())
        return "BRep";

    return "None";
}

const Mesh& Element::element_geometry_mesh() const {
    return geometry_mesh();
}

const BRep& Element::element_geometry_brep() const {
    return geometry_brep();
}

const Mesh& Element::model_geometry_mesh() const {

    if (!_model_mesh_cache)
        _model_mesh_cache = apply_geometry_ops(element_geometry_mesh());

    return *_model_mesh_cache;
}

const BRep& Element::model_geometry_brep() const {

    if (!_model_brep_cache)
        _model_brep_cache = element_geometry_brep();

    return *_model_brep_cache;
}

const Mesh& Element::geometry_mesh() const {

    const_cast<Element*>(this)->compute_geometry_mesh();
    static const Mesh empty;

    return _geometry_mesh ? *_geometry_mesh : empty;
}

void Element::compute_geometry_mesh() {

    if (_computing_geometry || (_geometry_synced && _geometry_mesh))
        return;

    _computing_geometry = true;

    try {
        compute_geometry_mesh_impl();
    } catch (...) {
        _computing_geometry = false;
        throw;
    }

    _computing_geometry = false;
    _geometry_synced = true;
}

Mesh Element::session_geometry_mesh(const Xform& xform) const {

    const Mesh& local = geometry_mesh();

    if (!_geometry_mesh)
        return Mesh();

    Mesh placed = apply_geometry_ops(local);

    if (!xform.is_identity())
        placed.transform(xform);

    return placed;
}

const BRep& Element::geometry_brep() const {

    const_cast<Element*>(this)->compute_geometry_brep();
    static const BRep empty;

    return _geometry_brep ? *_geometry_brep : empty;
}

void Element::compute_geometry_brep() {

    if (_computing_geometry || (_geometry_synced && _geometry_brep))
        return;

    _computing_geometry = true;

    try {
        compute_geometry_brep_impl();
    } catch (...) {
        _computing_geometry = false;
        throw;
    }

    _computing_geometry = false;
    _geometry_synced = true;
}

BRep Element::session_geometry_brep(const Xform& xform) const {

    const BRep& local = geometry_brep();
    BRep placed = local;

    if (!xform.is_identity())
        placed.transform(xform);

    return placed;
}

OBB Element::aabb() {

    if (_is_dirty || !_aabb.has_value()) {
        _aabb = compute_aabb();
        _is_dirty = false;
    }

    return _aabb.value();
}

OBB Element::obb() {

    if (_is_dirty || !_obb.has_value()) {
        _obb = compute_obb();
        _is_dirty = false;
    }

    return _obb.value();
}

Mesh Element::collision_mesh() {

    if (_is_dirty || !_collision_mesh.has_value()) {
        _collision_mesh = compute_collision_mesh();
        _is_dirty = false;
    }

    return _collision_mesh.value();
}

Point Element::point() {

    if (_is_dirty || !_point.has_value()) {
        _point = compute_point();
        _is_dirty = false;
    }

    return _point.value();
}

std::vector<Polyline> Element::polylines() {

    if (_is_dirty || !_polylines.has_value()) {
        _polylines = compute_polylines();
        _is_dirty = false;
    }

    return _polylines.value();
}

std::vector<Plane> Element::planes() {

    if (_is_dirty || !_planes.has_value()) {
        _planes = compute_planes();
        _is_dirty = false;
    }

    return _planes.value();
}

std::vector<Vector> Element::edge_vectors() {

    if (_is_dirty || !_edge_vectors.has_value()) {
        _edge_vectors = compute_edge_vectors();
        _is_dirty = false;
    }

    return _edge_vectors.value();
}

std::optional<Line> Element::axis() {

    if (_is_dirty || !_axis.has_value()) {
        _axis = compute_axis();
        _is_dirty = false;
    }

    return _axis;
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Mutators
// ═══════════════════════════════════════════════════════════════════════════

void Element::add_geometry_op(std::function<Mesh(Mesh)> f) {

    _geometry_ops.push_back(std::move(f));
    reset();
}

void Element::place(const Xform& xform) {

    ensure_geometry();

    if (_geometry_mesh)
        _geometry_mesh = session_geometry_mesh(xform);

    if (_geometry_brep)
        _geometry_brep = session_geometry_brep(xform);

    for (ElementFeature& feature : _features)
        for (Polyline& outline : feature.outlines)
            outline.transform(xform);

    for (Vector& direction : _insertion_vectors)
        direction.transform(xform);

    reset();
}

void Element::set_geometry(const Mesh& geo) {

    _geometry_mesh = geo;
    _geometry_brep.reset();
    reset();
}

void Element::set_geometry(const BRep& geo) {

    _geometry_brep = geo;
    _geometry_mesh.reset();
    reset();
}

void Element::set_polylines(std::vector<Polyline> polys) {

    _polylines = std::move(polys);
    _is_dirty = false;
}

void Element::set_planes(std::vector<Plane> plns) {

    _planes = std::move(plns);
    _is_dirty = false;
}

void Element::reset() {

    _is_dirty = true;
    _model_mesh_cache.reset();
    _model_brep_cache.reset();
    _aabb.reset();
    _obb.reset();
    _collision_mesh.reset();
    _point.reset();
    _polylines.reset();
    _planes.reset();
    _edge_vectors.reset();
    _axis.reset();
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Operators
// ═══════════════════════════════════════════════════════════════════════════

bool Element::operator==(const Element& other) const {
    return name == other.name && geometry_type_name() == other.geometry_type_name() &&
        element_type_name() == other.element_type_name() && element_data_dumps() == other.element_data_dumps() &&
        _insertion_vectors == other._insertion_vectors && _dimensions == other._dimensions &&
        _features == other._features;
}

bool Element::operator!=(const Element& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Utilities
// ═══════════════════════════════════════════════════════════════════════════

Element Element::duplicate() const {
    return Element(*this);
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Computation
// ═══════════════════════════════════════════════════════════════════════════

OBB Element::compute_aabb() {

    const std::vector<Point> points = geometry_points();

    return points.empty() ? OBB::from_point(Point(0, 0, 0), 0.0) : OBB::from_points(points, 0.0);
}

OBB Element::compute_obb() {
    return compute_aabb();
}

Mesh Element::compute_collision_mesh() {

    ensure_geometry();

    return _geometry_mesh ? session_geometry_mesh(Xform::identity()) : Mesh();
}

Point Element::compute_point() {
    return Point::centroid(geometry_points());
}

std::vector<Polyline> Element::compute_polylines() const {

    if (_geometry_mesh)
        return _geometry_mesh->face_outlines();

    return {};
}

std::vector<Plane> Element::compute_planes() const {

    std::vector<Plane> planes;

    for (const Polyline& outline : compute_polylines()) {
        std::vector<Point> points = outline.get_points();

        if (points.size() > 1 && points.front() == points.back())
            points.pop_back();

        if (points.size() < 3)
            continue;

        planes.push_back(Plane::from_point_normal(Point::centroid(points), Vector::average_normal(points)));
    }

    return planes;
}

std::vector<Vector> Element::compute_edge_vectors() const {
    return {};
}

std::optional<Line> Element::compute_axis() const {
    return std::nullopt;
}

Mesh Element::apply_geometry_ops(Mesh geo) const {

    for (const std::function<Mesh(Mesh)>& f : _geometry_ops)
        geo = f(geo);

    return geo;
}

std::vector<Point> Element::geometry_points() const {

    ensure_geometry();
    std::vector<Point> points;

    if (_geometry_mesh) {
        const Mesh mesh = session_geometry_mesh(Xform::identity());

        for (const auto& [key, vertex] : mesh.vertex)
            points.push_back(vertex.position());
    } else if (_geometry_brep) {
        points = _geometry_brep->vertex_points();
    }

    return points;
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Element::jsondump() const {

    ensure_geometry();

    nlohmann::ordered_json geo_data = nullptr;

    if (_geometry_mesh)
        geo_data = _geometry_mesh->jsondump();

    if (_geometry_brep)
        geo_data = _geometry_brep->jsondump();

    nlohmann::ordered_json dims = nullptr;

    if (_dimensions.has_value())
        dims = _dimensions->jsondump();

    nlohmann::ordered_json feats = nlohmann::ordered_json::array();

    for (const ElementFeature& f : _features)
        feats.push_back(f.jsondump());

    nlohmann::ordered_json ivs = nlohmann::ordered_json::array();

    for (const Vector& v : _insertion_vectors)
        ivs.push_back(v.jsondump());

    return nlohmann::ordered_json{
        {"dimensions", dims},
        {"element_data", to_hex(element_data_dumps())},
        {"element_type", element_type_name()},
        {"features", feats},
        {"geometry_data", geo_data},
        {"geometry_type", geometry_type_name()},
        {"guid", guid()},
        {"insertion_vectors", ivs},
        {"name", name},
        {"type", "Element"},
    };
}

Element Element::jsonload(const nlohmann::json& data) {

    Element elem;
    const std::string geo_type = data.value("geometry_type", "None");
    const bool has_data = data.contains("geometry_data") && !data["geometry_data"].is_null();

    if (geo_type == "Mesh" && has_data)
        elem._geometry_mesh = Mesh::jsonload(data["geometry_data"]);

    if (geo_type == "BRep" && has_data)
        elem._geometry_brep = BRep::jsonload(data["geometry_data"]);

    const std::string g = data.value("guid", std::string());

    if (!g.empty())
        elem.guid() = g;

    elem.name = data.value("name", elem.name);

    if (data.contains("dimensions") && !data["dimensions"].is_null())
        elem._dimensions = Vector::jsonload(data["dimensions"]);

    elem._element_type = data.value("element_type", std::string());
    elem._element_data = from_hex(data.value("element_data", std::string()));

    for (const nlohmann::json& f : data.value("features", nlohmann::json::array()))
        elem._features.push_back(ElementFeature::jsonload(f));

    for (const nlohmann::json& v : data.value("insertion_vectors", nlohmann::json::array()))
        elem._insertion_vectors.push_back(Vector::jsonload(v));

    return elem;
}

std::string Element::file_json_dumps() const {
    return jsondump().dump();
}

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

// ═══════════════════════════════════════════════════════════════════════════
// Element - Protobuf
// ═══════════════════════════════════════════════════════════════════════════

session_proto::Element Element::to_proto() const {

    ensure_geometry();

    session_proto::Element proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_geometry_type(geometry_type_name());

    if (_geometry_mesh)
        proto.set_geometry_data(_geometry_mesh->pb_dumps());

    if (_geometry_brep)
        proto.set_geometry_data(_geometry_brep->pb_dumps());

    proto.set_element_type(element_type_name());
    proto.set_element_data(element_data_dumps());

    for (const Vector& v : _insertion_vectors) {
        proto.add_insertion_vectors(v[0]);
        proto.add_insertion_vectors(v[1]);
        proto.add_insertion_vectors(v[2]);
    }

    if (_dimensions.has_value()) {
        proto.add_dimensions((*_dimensions)[0]);
        proto.add_dimensions((*_dimensions)[1]);
        proto.add_dimensions((*_dimensions)[2]);
    }

    for (const ElementFeature& f : _features)
        *proto.add_features() = f.to_proto();

    return proto;
}

Element Element::from_proto(const session_proto::Element& proto) {

    Element elem;

    if (!proto.guid().empty())
        elem.guid() = proto.guid();

    elem.name = proto.name();

    const bool has_data = !proto.geometry_data().empty();

    if (proto.geometry_type() == "Mesh" && has_data)
        elem._geometry_mesh = Mesh::pb_loads(proto.geometry_data());

    if (proto.geometry_type() == "BRep" && has_data)
        elem._geometry_brep = BRep::pb_loads(proto.geometry_data());

    elem._element_type = proto.element_type();
    elem._element_data = proto.element_data();

    for (int i = 0; i + 2 < proto.insertion_vectors_size(); i += 3)
        elem._insertion_vectors.push_back(
            Vector(proto.insertion_vectors(i), proto.insertion_vectors(i + 1), proto.insertion_vectors(i + 2))
        );

    if (proto.dimensions_size() == 3)
        elem._dimensions = Vector(proto.dimensions(0), proto.dimensions(1), proto.dimensions(2));

    for (const session_proto::ElementFeature& f : proto.features())
        elem._features.push_back(ElementFeature::from_proto(f));

    return elem;
}

std::string Element::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Element Element::pb_loads(const std::string& data) {

    session_proto::Element proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Element protobuf data");

    return from_proto(proto);
}

void Element::pb_dump(const std::string& path) const {

    const std::string data = pb_dumps();
    std::ofstream file(path, std::ios::binary);
    file.write(data.data(), data.size());
}

Element Element::pb_load(const std::string& path) {

    std::ifstream file(path, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Polymorphic registry
// ═══════════════════════════════════════════════════════════════════════════

/// Function-local so a package registering from a static initializer finds it built.
static std::map<std::string, Element::Factory>& element_registry() {

    static std::map<std::string, Element::Factory> registry;

    return registry;
}

/// A factory that throws or returns null degrades to the base exactly like an unregistered type.
static std::shared_ptr<Element> build_registered(const std::string& type_name, const std::string& data) {

    if (type_name.empty())
        return nullptr;

    const auto it = element_registry().find(type_name);

    if (it == element_registry().end())
        return nullptr;

    try {
        return it->second(data);
    } catch (const std::exception&) {
        return nullptr;
    }
}

void Element::register_type(const std::string& type_name, Element::Factory factory) {

    if (type_name.empty() || !factory)
        return;

    element_registry()[type_name] = std::move(factory);
}

bool Element::is_registered(const std::string& type_name) {
    return element_registry().count(type_name) > 0;
}

std::vector<std::string> Element::registered_types() {

    std::vector<std::string> names;

    for (const std::pair<const std::string, Element::Factory>& entry : element_registry())
        names.push_back(entry.first);

    return names;
}

std::shared_ptr<Element> Element::pb_loads_polymorphic(const std::string& data) {

    session_proto::Element proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Element protobuf data");

    if (std::shared_ptr<Element> derived = build_registered(proto.element_type(), data))
        return derived;

    return std::make_shared<Element>(from_proto(proto));
}

std::shared_ptr<Element> Element::file_json_loads_polymorphic(const std::string& s) {

    Element base = file_json_loads(s);

    if (std::shared_ptr<Element> derived = build_registered(base.element_type_name(), base.pb_dumps()))
        return derived;

    return std::make_shared<Element>(std::move(base));
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - String
// ═══════════════════════════════════════════════════════════════════════════

std::string Element::str() const {
    return fmt::format("Element({}, {})", name, geometry_type_name());
}

std::string Element::repr() const {
    return fmt::format("Element({}, {}, {})", guid(), name, geometry_type_name());
}

std::ostream& operator<<(std::ostream& os, const Element& e) {
    return os << e.str();
}

} // namespace session_cpp
