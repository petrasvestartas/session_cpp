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
    : guid(::guid()), name(other.name),
      session_transformation(other.session_transformation),
      _geometry(other._geometry), _is_dirty(true),
      _features(other._features) {}

Element& Element::operator=(const Element& other) {
    if (this != &other) {
        guid = ::guid();
        name = other.name;
        session_transformation = other.session_transformation;
        _geometry = other._geometry;
        _features = other._features;
        _is_dirty = true;
        _aabb.reset();
        _obb.reset();
        _collision_mesh.reset();
        _point.reset();
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

ElementGeometry Element::session_geometry() const {
    if (!has_geometry()) return std::monostate{};
    auto geo = _geometry;
    if (auto* mesh = std::get_if<Mesh>(&geo)) {
        *mesh = apply_features(*mesh);
        if (!session_transformation.is_identity()) {
            mesh->xform = session_transformation * mesh->xform;
            mesh->transform();
        }
    } else if (auto* brep = std::get_if<BRep>(&geo)) {
        if (!session_transformation.is_identity()) {
            brep->xform = session_transformation * brep->xform;
            brep->transform();
        }
    }
    return geo;
}

Obb Element::aabb() {
    if (_is_dirty || !_aabb.has_value()) _aabb = compute_aabb();
    return _aabb.value();
}

Obb Element::obb() {
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

void Element::add_feature(std::function<Mesh(Mesh)> f) {
    _features.push_back(std::move(f));
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

void Element::reset() {
    _is_dirty = true;
    _aabb.reset();
    _obb.reset();
    _collision_mesh.reset();
    _point.reset();
}

Element Element::duplicate() const {
    Element result(*this);
    result.guid = ::guid();
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
    return fmt::format("Element({}, {}, {})", guid, name, geometry_type_name());
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - Computation
///////////////////////////////////////////////////////////////////////////////////////////

Obb Element::compute_aabb() {
    auto geo = session_geometry();
    if (std::holds_alternative<std::monostate>(geo))
        return Obb::from_point(Point(0, 0, 0), 0.0);
    return obb_from_geometry(geo, true);
}

Obb Element::compute_obb() {
    auto geo = session_geometry();
    if (std::holds_alternative<std::monostate>(geo))
        return Obb::from_point(Point(0, 0, 0), 0.0);
    return obb_from_geometry(geo, false);
}

Mesh Element::compute_collision_mesh() {
    auto geo = session_geometry();
    if (auto* mesh = std::get_if<Mesh>(&geo)) return *mesh;
    return Mesh();
}

Point Element::compute_point() {
    auto geo = session_geometry();
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

Mesh Element::apply_features(Mesh geo) const {
    for (const auto& f : _features) geo = f(geo);
    return geo;
}

Obb Element::obb_from_geometry(const ElementGeometry& geo, bool as_aabb) {
    double inflate = 0.0;
    if (auto* mesh = std::get_if<Mesh>(&geo)) {
        std::vector<Point> points;
        for (const auto& [k, v] : mesh->vertex)
            points.push_back(v.position());
        if (points.empty()) return Obb::from_point(Point(0, 0, 0), inflate);
        return Obb::from_points(points, inflate);
    }
    if (auto* brep = std::get_if<BRep>(&geo)) {
        if (brep->m_vertices.empty()) return Obb::from_point(Point(0, 0, 0), inflate);
        return Obb::from_points(brep->m_vertices, inflate);
    }
    return Obb::from_point(Point(0, 0, 0), inflate);
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
        {"guid", guid},
        {"name", name},
        {"session_transformation", session_transformation.jsondump()},
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
    elem.guid = data.value("guid", elem.guid);
    elem.name = data.value("name", elem.name);
    if (data.contains("session_transformation"))
        elem.session_transformation = Xform::jsonload(data["session_transformation"]);
    return elem;
}

std::string Element::json_dumps() const { return jsondump().dump(); }

Element Element::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

void Element::json_dump(const std::string& path) const {
    std::ofstream file(path);
    file << jsondump().dump(2);
}

Element Element::json_load(const std::string& path) {
    std::ifstream file(path);
    return jsonload(nlohmann::json::parse(file));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

std::string Element::pb_dumps() const {
    session_proto::Element proto;
    proto.set_guid(guid);
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
    auto* xf = proto.mutable_session_transformation();
    xf->set_name(session_transformation.name);
    for (int i = 0; i < 16; ++i) xf->add_matrix(session_transformation.m[i]);
    return proto.SerializeAsString();
}

Element Element::pb_loads(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);
    Element elem;
    elem.guid = proto.guid();
    elem.name = proto.name();
    std::string geo_type = proto.geometry_type();
    if (geo_type == "Mesh" && !proto.geometry_data().empty()) {
        elem._geometry = Mesh::pb_loads(proto.geometry_data());
    } else if (geo_type == "BRep" && !proto.geometry_data().empty()) {
        elem._geometry = BRep::pb_loads(proto.geometry_data());
    }
    Xform xf;
    xf.name = proto.session_transformation().name();
    if (proto.session_transformation().matrix_size() == 16)
        for (int i = 0; i < 16; ++i) xf.m[i] = proto.session_transformation().matrix(i);
    elem.session_transformation = xf;
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

std::ostream& operator<<(std::ostream& os, const Element& e) { return os << e.str(); }

///////////////////////////////////////////////////////////////////////////////////////////
// ColumnElement
///////////////////////////////////////////////////////////////////////////////////////////

ColumnElement::ColumnElement(double width, double depth, double height, const std::string& name)
    : Element(name), _width(width), _depth(depth), _height(height) {
    _geometry = compute_element_geometry();
}

ColumnElement::ColumnElement(const ColumnElement& other)
    : Element(other), _width(other._width), _depth(other._depth), _height(other._height) {
    guid = ::guid();
}

ColumnElement& ColumnElement::operator=(const ColumnElement& other) {
    if (this != &other) {
        Element::operator=(other);
        _width = other._width;
        _depth = other._depth;
        _height = other._height;
    }
    return *this;
}

void ColumnElement::set_width(double v) {
    _width = v;
    _geometry = compute_element_geometry();
    reset();
}

void ColumnElement::set_depth(double v) {
    _depth = v;
    _geometry = compute_element_geometry();
    reset();
}

void ColumnElement::set_height(double v) {
    _height = v;
    _geometry = compute_element_geometry();
    reset();
}

Line ColumnElement::center_line() const {
    return Line(0, 0, 0, 0, 0, _height);
}

void ColumnElement::extend(double distance) {
    _height += distance * 2;
    _geometry = compute_element_geometry();
    reset();
}

Mesh ColumnElement::compute_element_geometry() const {
    double hx = _width * 0.5;
    double hy = _depth * 0.5;
    std::vector<Point> vertices = {
        Point(-hx, -hy, 0),       Point( hx, -hy, 0),
        Point( hx,  hy, 0),       Point(-hx,  hy, 0),
        Point(-hx, -hy, _height), Point( hx, -hy, _height),
        Point( hx,  hy, _height), Point(-hx,  hy, _height),
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1}, {4, 5, 6, 7},
        {0, 1, 5, 4}, {2, 3, 7, 6},
        {0, 4, 7, 3}, {1, 2, 6, 5},
    };
    return Mesh::from_vertices_and_faces(vertices, faces);
}

ColumnElement ColumnElement::duplicate() const {
    ColumnElement result(*this);
    result.guid = ::guid();
    return result;
}

bool ColumnElement::operator==(const Element& other) const {
    auto* o = dynamic_cast<const ColumnElement*>(&other);
    if (!o) return false;
    return name == o->name && _width == o->_width && _depth == o->_depth && _height == o->_height;
}

std::string ColumnElement::str() const {
    return fmt::format("ColumnElement({}, {}, {}, {})", name, _width, _depth, _height);
}

std::string ColumnElement::repr() const {
    return fmt::format("ColumnElement({}, {}, {}, {}, {})", guid, name, _width, _depth, _height);
}

nlohmann::ordered_json ColumnElement::jsondump() const {
    auto* mesh = std::get_if<Mesh>(&_geometry);
    return nlohmann::ordered_json{
        {"depth", _depth},
        {"geometry_data", mesh ? mesh->jsondump() : nullptr},
        {"geometry_type", mesh ? "Mesh" : "None"},
        {"guid", guid},
        {"height", _height},
        {"name", name},
        {"session_transformation", session_transformation.jsondump()},
        {"type", "ColumnElement"},
        {"width", _width},
    };
}

ColumnElement ColumnElement::jsonload(const nlohmann::json& data) {
    ColumnElement elem(
        data.value("width", 0.4),
        data.value("depth", 0.4),
        data.value("height", 3.0)
    );
    elem.guid = data.value("guid", elem.guid);
    elem.name = data.value("name", elem.name);
    if (data.contains("session_transformation"))
        elem.session_transformation = Xform::jsonload(data["session_transformation"]);
    return elem;
}

ColumnElement ColumnElement::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

ColumnElement ColumnElement::json_load(const std::string& path) {
    std::ifstream file(path);
    return jsonload(nlohmann::json::parse(file));
}

std::string ColumnElement::pb_dumps() const {
    session_proto::Element proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_geometry_type("ColumnElement");
    nlohmann::json params = {{"width", _width}, {"depth", _depth}, {"height", _height}};
    std::string params_str = params.dump();
    proto.set_geometry_data(params_str);
    auto* xf = proto.mutable_session_transformation();
    xf->set_name(session_transformation.name);
    for (int i = 0; i < 16; ++i) xf->add_matrix(session_transformation.m[i]);
    return proto.SerializeAsString();
}

ColumnElement ColumnElement::pb_loads(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);
    auto params = nlohmann::json::parse(proto.geometry_data());
    ColumnElement elem(params["width"], params["depth"], params["height"]);
    elem.guid = proto.guid();
    elem.name = proto.name();
    Xform xf;
    xf.name = proto.session_transformation().name();
    if (proto.session_transformation().matrix_size() == 16)
        for (int i = 0; i < 16; ++i) xf.m[i] = proto.session_transformation().matrix(i);
    elem.session_transformation = xf;
    return elem;
}

ColumnElement ColumnElement::pb_load(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// BeamElement
///////////////////////////////////////////////////////////////////////////////////////////

BeamElement::BeamElement(double width, double depth, double length, const std::string& name)
    : Element(name), _width(width), _depth(depth), _length(length) {
    _geometry = compute_element_geometry();
}

BeamElement::BeamElement(const BeamElement& other)
    : Element(other), _width(other._width), _depth(other._depth), _length(other._length) {
    guid = ::guid();
}

BeamElement& BeamElement::operator=(const BeamElement& other) {
    if (this != &other) {
        Element::operator=(other);
        _width = other._width;
        _depth = other._depth;
        _length = other._length;
    }
    return *this;
}

void BeamElement::set_width(double v) {
    _width = v;
    _geometry = compute_element_geometry();
    reset();
}

void BeamElement::set_depth(double v) {
    _depth = v;
    _geometry = compute_element_geometry();
    reset();
}

void BeamElement::set_length(double v) {
    _length = v;
    _geometry = compute_element_geometry();
    reset();
}

Line BeamElement::center_line() const {
    return Line(0, 0, 0, 0, 0, _length);
}

void BeamElement::extend(double distance) {
    _length += distance * 2;
    _geometry = compute_element_geometry();
    reset();
}

Mesh BeamElement::compute_element_geometry() const {
    double hx = _width * 0.5;
    double hy = _depth * 0.5;
    std::vector<Point> vertices = {
        Point(-hx, -hy, 0),       Point( hx, -hy, 0),
        Point( hx,  hy, 0),       Point(-hx,  hy, 0),
        Point(-hx, -hy, _length), Point( hx, -hy, _length),
        Point( hx,  hy, _length), Point(-hx,  hy, _length),
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1}, {4, 5, 6, 7},
        {0, 1, 5, 4}, {2, 3, 7, 6},
        {0, 4, 7, 3}, {1, 2, 6, 5},
    };
    return Mesh::from_vertices_and_faces(vertices, faces);
}

BeamElement BeamElement::duplicate() const {
    BeamElement result(*this);
    result.guid = ::guid();
    return result;
}

bool BeamElement::operator==(const Element& other) const {
    auto* o = dynamic_cast<const BeamElement*>(&other);
    if (!o) return false;
    return name == o->name && _width == o->_width && _depth == o->_depth && _length == o->_length;
}

std::string BeamElement::str() const {
    return fmt::format("BeamElement({}, {}, {}, {})", name, _width, _depth, _length);
}

std::string BeamElement::repr() const {
    return fmt::format("BeamElement({}, {}, {}, {}, {})", guid, name, _width, _depth, _length);
}

nlohmann::ordered_json BeamElement::jsondump() const {
    auto* mesh = std::get_if<Mesh>(&_geometry);
    return nlohmann::ordered_json{
        {"depth", _depth},
        {"geometry_data", mesh ? mesh->jsondump() : nullptr},
        {"geometry_type", mesh ? "Mesh" : "None"},
        {"guid", guid},
        {"length", _length},
        {"name", name},
        {"session_transformation", session_transformation.jsondump()},
        {"type", "BeamElement"},
        {"width", _width},
    };
}

BeamElement BeamElement::jsonload(const nlohmann::json& data) {
    BeamElement elem(
        data.value("width", 0.1),
        data.value("depth", 0.2),
        data.value("length", 3.0)
    );
    elem.guid = data.value("guid", elem.guid);
    elem.name = data.value("name", elem.name);
    if (data.contains("session_transformation"))
        elem.session_transformation = Xform::jsonload(data["session_transformation"]);
    return elem;
}

BeamElement BeamElement::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

BeamElement BeamElement::json_load(const std::string& path) {
    std::ifstream file(path);
    return jsonload(nlohmann::json::parse(file));
}

std::string BeamElement::pb_dumps() const {
    session_proto::Element proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_geometry_type("BeamElement");
    nlohmann::json params = {{"width", _width}, {"depth", _depth}, {"length", _length}};
    std::string params_str = params.dump();
    proto.set_geometry_data(params_str);
    auto* xf = proto.mutable_session_transformation();
    xf->set_name(session_transformation.name);
    for (int i = 0; i < 16; ++i) xf->add_matrix(session_transformation.m[i]);
    return proto.SerializeAsString();
}

BeamElement BeamElement::pb_loads(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);
    auto params = nlohmann::json::parse(proto.geometry_data());
    BeamElement elem(params["width"], params["depth"], params["length"]);
    elem.guid = proto.guid();
    elem.name = proto.name();
    Xform xf;
    xf.name = proto.session_transformation().name();
    if (proto.session_transformation().matrix_size() == 16)
        for (int i = 0; i < 16; ++i) xf.m[i] = proto.session_transformation().matrix(i);
    elem.session_transformation = xf;
    return elem;
}

BeamElement BeamElement::pb_load(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// PlateElement
///////////////////////////////////////////////////////////////////////////////////////////

PlateElement::PlateElement(const std::vector<Point>& polygon, double thickness,
                           const std::string& name)
    : Element(name), _thickness(thickness) {
    if (polygon.empty()) {
        _polygon = {
            Point(-0.5, -0.5, 0), Point( 0.5, -0.5, 0),
            Point( 0.5,  0.5, 0), Point(-0.5,  0.5, 0),
        };
    } else {
        _polygon.reserve(polygon.size());
        for (const auto& p : polygon)
            _polygon.emplace_back(p[0], p[1], p[2]);
    }
    _geometry = compute_element_geometry();
}

PlateElement::PlateElement(const PlateElement& other)
    : Element(other), _polygon(other._polygon), _thickness(other._thickness) {
    guid = ::guid();
}

PlateElement& PlateElement::operator=(const PlateElement& other) {
    if (this != &other) {
        Element::operator=(other);
        _polygon = other._polygon;
        _thickness = other._thickness;
    }
    return *this;
}

void PlateElement::set_polygon(const std::vector<Point>& pts) {
    _polygon.clear();
    _polygon.reserve(pts.size());
    for (const auto& p : pts) _polygon.emplace_back(p[0], p[1], p[2]);
    _geometry = compute_element_geometry();
    reset();
}

void PlateElement::set_thickness(double v) {
    _thickness = v;
    _geometry = compute_element_geometry();
    reset();
}

Vector PlateElement::polygon_normal(const std::vector<Point>& pts) {
    double nx = 0, ny = 0, nz = 0;
    size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& c = pts[i];
        const auto& nx_pt = pts[(i + 1) % n];
        nx += (c[1] - nx_pt[1]) * (c[2] + nx_pt[2]);
        ny += (c[2] - nx_pt[2]) * (c[0] + nx_pt[0]);
        nz += (c[0] - nx_pt[0]) * (c[1] + nx_pt[1]);
    }
    double mag = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (mag < 1e-12) return Vector(0, 0, 1);
    return Vector(nx / mag, ny / mag, nz / mag);
}

Mesh PlateElement::compute_element_geometry() const {
    Vector normal = polygon_normal(_polygon);
    size_t n = _polygon.size();
    std::vector<Point> vertices;
    vertices.reserve(n * 2);
    for (const auto& p : _polygon) {
        vertices.emplace_back(p[0], p[1], p[2]);
    }
    for (const auto& p : _polygon) {
        vertices.emplace_back(
            p[0] - normal[0] * _thickness,
            p[1] - normal[1] * _thickness,
            p[2] - normal[2] * _thickness
        );
    }
    std::vector<std::vector<size_t>> faces;
    std::vector<size_t> bottom_face;
    for (size_t i = n; i-- > 0;) bottom_face.push_back(i);
    std::vector<size_t> top_face;
    for (size_t i = n; i < 2 * n; ++i) top_face.push_back(i);
    faces.push_back(bottom_face);
    faces.push_back(top_face);
    for (size_t i = 0; i < n; ++i) {
        size_t a = i;
        size_t b = (i + 1) % n;
        size_t c = b + n;
        size_t d = a + n;
        faces.push_back({a, b, c, d});
    }
    return Mesh::from_vertices_and_faces(vertices, faces);
}

PlateElement PlateElement::duplicate() const {
    PlateElement result(*this);
    result.guid = ::guid();
    return result;
}

bool PlateElement::operator==(const Element& other) const {
    auto* o = dynamic_cast<const PlateElement*>(&other);
    if (!o) return false;
    if (name != o->name) return false;
    if (_thickness != o->_thickness) return false;
    if (_polygon.size() != o->_polygon.size()) return false;
    for (size_t i = 0; i < _polygon.size(); ++i) {
        if (_polygon[i][0] != o->_polygon[i][0] ||
            _polygon[i][1] != o->_polygon[i][1] ||
            _polygon[i][2] != o->_polygon[i][2])
            return false;
    }
    return true;
}

std::string PlateElement::str() const {
    return fmt::format("PlateElement({}, {} pts, {})", name, _polygon.size(), _thickness);
}

std::string PlateElement::repr() const {
    return fmt::format("PlateElement({}, {}, {} pts, {})", guid, name, _polygon.size(), _thickness);
}

nlohmann::ordered_json PlateElement::jsondump() const {
    auto* mesh = std::get_if<Mesh>(&_geometry);
    nlohmann::ordered_json poly_json = nlohmann::ordered_json::array();
    for (const auto& p : _polygon)
        poly_json.push_back({p[0], p[1], p[2]});
    return nlohmann::ordered_json{
        {"geometry_data", mesh ? mesh->jsondump() : nullptr},
        {"geometry_type", mesh ? "Mesh" : "None"},
        {"guid", guid},
        {"name", name},
        {"polygon", poly_json},
        {"session_transformation", session_transformation.jsondump()},
        {"thickness", _thickness},
        {"type", "PlateElement"},
    };
}

PlateElement PlateElement::jsonload(const nlohmann::json& data) {
    std::vector<Point> polygon;
    if (data.contains("polygon")) {
        for (const auto& p : data["polygon"])
            polygon.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    }
    PlateElement elem(polygon.empty() ? std::vector<Point>{} : polygon,
                      data.value("thickness", 0.1));
    elem.guid = data.value("guid", elem.guid);
    elem.name = data.value("name", elem.name);
    if (data.contains("session_transformation"))
        elem.session_transformation = Xform::jsonload(data["session_transformation"]);
    return elem;
}

PlateElement PlateElement::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

PlateElement PlateElement::json_load(const std::string& path) {
    std::ifstream file(path);
    return jsonload(nlohmann::json::parse(file));
}

std::string PlateElement::pb_dumps() const {
    session_proto::Element proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_geometry_type("PlateElement");
    nlohmann::json params;
    nlohmann::json poly_json = nlohmann::json::array();
    for (const auto& p : _polygon)
        poly_json.push_back({p[0], p[1], p[2]});
    params["polygon"] = poly_json;
    params["thickness"] = _thickness;
    std::string params_str = params.dump();
    proto.set_geometry_data(params_str);
    auto* xf = proto.mutable_session_transformation();
    xf->set_name(session_transformation.name);
    for (int i = 0; i < 16; ++i) xf->add_matrix(session_transformation.m[i]);
    return proto.SerializeAsString();
}

PlateElement PlateElement::pb_loads(const std::string& data) {
    session_proto::Element proto;
    proto.ParseFromString(data);
    auto params = nlohmann::json::parse(proto.geometry_data());
    std::vector<Point> polygon;
    for (const auto& p : params["polygon"])
        polygon.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    PlateElement elem(polygon, params["thickness"]);
    elem.guid = proto.guid();
    elem.name = proto.name();
    Xform xf;
    xf.name = proto.session_transformation().name();
    if (proto.session_transformation().matrix_size() == 16)
        for (int i = 0; i < 16; ++i) xf.m[i] = proto.session_transformation().matrix(i);
    elem.session_transformation = xf;
    return elem;
}

PlateElement PlateElement::pb_load(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

} // namespace session_cpp
