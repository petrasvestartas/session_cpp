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
      _insertion_vectors(other._insertion_vectors), _dimensions(other._dimensions),
      _element_type(other._element_type), _element_data(other._element_data) {}

Element& Element::operator=(const Element& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        _geometry = other._geometry;
        _geometry_ops = other._geometry_ops;
        _features = other._features;
        _insertion_vectors = other._insertion_vectors;
        _dimensions = other._dimensions;
        _element_type = other._element_type;
        _element_data = other._element_data;
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
    // Data equality, not identity - the guid is excluded, exactly as in Line. Every field that
    // survives a round trip is compared, so `pb_loads(e.pb_dumps()) == e` is a real test rather
    // than one that passes on two fields and ignores the other five.
    return name == other.name && geometry_type_name() == other.geometry_type_name() &&
           element_type_name() == other.element_type_name() &&
           element_data_dumps() == other.element_data_dumps() &&
           _insertion_vectors == other._insertion_vectors &&
           _dimensions == other._dimensions && _features == other._features;
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
        for (const auto& v : brep->m_vertices) {
            sx += v.point[0]; sy += v.point[1]; sz += v.point[2];
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
std::string ElementFeature::repr() const { return str(); }
std::ostream& operator<<(std::ostream& os, const ElementFeature& f) { return os << f.str(); }

nlohmann::ordered_json ElementFeature::jsondump() const {
    nlohmann::ordered_json outs = nlohmann::ordered_json::array();
    for (const auto& o : outlines) outs.push_back(o.jsondump());
    return nlohmann::ordered_json{
        {"face_index", face_index},
        {"feature_type", feature_type},
        {"guid", guid()},
        {"name", name},
        {"outlines", outs},
        {"type", "ElementFeature"},
    };
}
ElementFeature ElementFeature::jsonload(const nlohmann::json& data) {
    ElementFeature f;
    f.face_index = data.value("face_index", -1);
    f.feature_type = data.value("feature_type", std::string());
    // Assigned, not minted: a feature read back is the SAME feature, so anything holding its
    // guid still finds it. Absent means the file predates the field - leave the lazy mint.
    std::string g = data.value("guid", std::string());
    if (!g.empty()) f.guid() = g;
    f.name = data.value("name", std::string());
    if (data.contains("outlines")) {
        for (const auto& o : data["outlines"]) f.outlines.push_back(Polyline::jsonload(o));
    }
    return f;
}
std::string ElementFeature::file_json_dumps() const { return jsondump().dump(); }
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
std::string ElementFeature::pb_dumps() const {
    session_proto::ElementFeature proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_feature_type(feature_type);
    proto.set_face_index(face_index);
    for (const auto& o : outlines) proto.add_outlines()->ParseFromString(o.pb_dumps());
    return proto.SerializeAsString();
}
ElementFeature ElementFeature::pb_loads(const std::string& data) {
    session_proto::ElementFeature proto;
    proto.ParseFromString(data);
    ElementFeature f;
    if (!proto.guid().empty()) f.guid() = proto.guid();
    f.name = proto.name();
    f.feature_type = proto.feature_type();
    f.face_index = proto.face_index();
    for (const auto& o : proto.outlines()) f.outlines.push_back(Polyline::pb_loads(o.SerializeAsString()));
    return f;
}
void ElementFeature::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}
ElementFeature ElementFeature::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return pb_loads(data);
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
        return OBB::from_points(brep->vertex_points(), inflate);
    }
    return OBB::from_point(Point(0, 0, 0), inflate);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - JSON
///////////////////////////////////////////////////////////////////////////////////////////

/// `element_data` is opaque BYTES and JSON has no byte type, so it travels as hex. Hex rather
/// than base64 because it is a handful of lines in each of the three languages and needs no
/// dependency in any of them - and this has to encode identically in all three, or the JSON
/// stops being a cross-language format.
static std::string to_hex(const std::string& bytes) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(bytes.size() * 2);
    for (unsigned char c : bytes) { out += digits[c >> 4]; out += digits[c & 15]; }
    return out;
}
static std::string from_hex(const std::string& hex) {
    std::string out;
    out.reserve(hex.size() / 2);
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        out += static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16));
    }
    return out;
}

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
    // Everything the element carries, not just the two fields it had before the registry: a
    // format that silently drops five of them is not a serialization format, and `file_json_dump`
    // has to round-trip whatever `pb_dumps` does, or the two disagree about what an Element is.
    nlohmann::ordered_json dims = nullptr;
    if (_dimensions.has_value()) dims = _dimensions->jsondump();
    nlohmann::ordered_json feats = nlohmann::ordered_json::array();
    for (const auto& f : _features) feats.push_back(f.jsondump());
    nlohmann::ordered_json ivs = nlohmann::ordered_json::array();
    for (const auto& v : _insertion_vectors) ivs.push_back(v.jsondump());
    return nlohmann::ordered_json{
        {"dimensions", dims},
        {"element_data", to_hex(element_data_dumps())},
        {"element_type", element_type_name()},
        {"features", feats},
        {"geometry_data", geo_data},
        {"geometry_type", geo_type},
        {"guid", guid()},
        {"insertion_vectors", ivs},
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
    // null, not absent-or-empty: (0,0,0) is a legitimate authored dimension, so the two cases
    // stay distinguishable here exactly as `has_dimensions` keeps them apart on the wire.
    if (data.contains("dimensions") && !data["dimensions"].is_null()) {
        elem._dimensions = Vector::jsonload(data["dimensions"]);
    }
    elem._element_type = data.value("element_type", std::string());
    elem._element_data = from_hex(data.value("element_data", std::string()));
    if (data.contains("features")) {
        for (const auto& f : data["features"]) elem._features.push_back(ElementFeature::jsonload(f));
    }
    if (data.contains("insertion_vectors")) {
        for (const auto& v : data["insertion_vectors"]) {
            elem._insertion_vectors.push_back(Vector::jsonload(v));
        }
    }
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

    // Packed triples, not sub-messages. Not for the bytes - a unit axis is 2 B CHEAPER as a
    // sub-message - but for the shape: no per-entry `name` String allocated on decode, and no
    // serialize-then-reparse round trip. See element.proto.
    for (const auto& v : _insertion_vectors) {
        proto.add_insertion_vectors(v[0]);
        proto.add_insertion_vectors(v[1]);
        proto.add_insertion_vectors(v[2]);
    }
    if (_dimensions.has_value()) {
        proto.add_dimensions((*_dimensions)[0]);
        proto.add_dimensions((*_dimensions)[1]);
        proto.add_dimensions((*_dimensions)[2]);
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

    // Carried, not interpreted. A viewer with no wood package registered loads a wood element
    // as a base Element; if these two were dropped here, saving it again wrote empty strings and
    // destroyed the payload the file was written with. See `element_type_name` in element.h.
    elem._element_type = proto.element_type();
    elem._element_data = proto.element_data();

    for (int i = 0; i + 2 < proto.insertion_vectors_size(); i += 3) {
        elem._insertion_vectors.push_back(Vector(proto.insertion_vectors(i),
                                                 proto.insertion_vectors(i + 1),
                                                 proto.insertion_vectors(i + 2)));
    }
    // Length, not a zero check: (0,0,0) is a legitimate authored value and must not be confused
    // with "never authored", which is what an EMPTY field means here.
    if (proto.dimensions_size() == 3) {
        elem._dimensions = Vector(proto.dimensions(0), proto.dimensions(1), proto.dimensions(2));
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

std::shared_ptr<Element> Element::file_json_loads_polymorphic(const std::string& s) {
    // The base load carries element_type/element_data through, so re-encoding it gives the
    // factory exactly the proto bytes it expects - one registration, both formats.
    Element base = file_json_loads(s);
    if (!base.element_type_name().empty()) {
        auto it = element_registry().find(base.element_type_name());
        if (it != element_registry().end()) {
            try {
                if (auto derived = it->second(base.pb_dumps())) { return derived; }
            } catch (const std::exception&) {
            }
        }
    }
    return std::make_shared<Element>(std::move(base));
}

std::shared_ptr<Element> Element::pb_loads_polymorphic(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);

    const std::string& type_name = proto.element_type();
    if (!type_name.empty()) {
        auto it = element_registry().find(type_name);
        if (it != element_registry().end()) {
            // A THROWING factory is the same failure as a null-returning one - a bug in that
            // package - and it must not take the whole Session with it either. Without this
            // catch, one malformed wood element makes every other element in the file
            // unreachable, which is the opposite of the graceful degradation below.
            try {
                if (auto derived = it->second(data)) { return derived; }
            } catch (const std::exception&) {
            }
            // A factory that returns null or throws: fall through to the base.
        }
    }

    // No type, no factory, or a factory that declined: keep the geometry, the identity, AND the
    // derived payload - `pb_loads` carries `element_type`/`element_data` through, so re-saving
    // this element writes the bytes the package wrote rather than erasing them.
    return std::make_shared<Element>(pb_loads(data));
}

std::ostream& operator<<(std::ostream& os, const Element& e) { return os << e.str(); }

///////////////////////////////////////////////////////////////////////////////////////////

} // namespace session_cpp
