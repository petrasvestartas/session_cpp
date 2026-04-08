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
      session_transformation(other.session_transformation),
      _geometry(other._geometry), _is_dirty(true),
      _features(other._features) {}

Element& Element::operator=(const Element& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        session_transformation = other.session_transformation;
        _geometry = other._geometry;
        _features = other._features;
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
    auto geo = session_geometry();
    if (std::holds_alternative<std::monostate>(geo))
        return OBB::from_point(Point(0, 0, 0), 0.0);
    return obb_from_geometry(geo);
}

OBB Element::compute_obb() {
    auto geo = session_geometry();
    if (std::holds_alternative<std::monostate>(geo))
        return OBB::from_point(Point(0, 0, 0), 0.0);
    return obb_from_geometry(geo);
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

std::vector<Polyline> Element::compute_polylines() const { return {}; }
std::vector<Plane> Element::compute_planes() const { return {}; }
std::vector<Vector> Element::compute_edge_vectors() const { return {}; }
std::optional<Line> Element::compute_axis() const { return std::nullopt; }

Mesh Element::apply_features(Mesh geo) const {
    for (const auto& f : _features) geo = f(geo);
    return geo;
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
    elem.guid() = data.value("guid", elem.guid());
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
    auto* xf = proto.mutable_session_transformation();
    xf->set_name(session_transformation.name);
    for (int i = 0; i < 16; ++i) xf->add_matrix(session_transformation.m[i]);
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
    : Element(other), _width(other._width), _depth(other._depth), _height(other._height) {}

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
        Point(-hx, -hy, 0),
        Point( hx, -hy, 0),
        Point( hx,  hy, 0),
        Point(-hx,  hy, 0),
        Point(-hx, -hy, _height),
        Point( hx, -hy, _height),
        Point( hx,  hy, _height),
        Point(-hx,  hy, _height),
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1}, {4, 5, 6, 7},
        {0, 1, 5, 4}, {2, 3, 7, 6},
        {0, 4, 7, 3}, {1, 2, 6, 5},
    };
    return Mesh::from_vertices_and_faces(vertices, faces);
}

std::vector<Polyline> ColumnElement::compute_polylines() const {
    double hx = _width * 0.5;
    double hy = _depth * 0.5;
    Point b0(-hx, -hy, 0), b1(hx, -hy, 0), b2(hx, hy, 0), b3(-hx, hy, 0);
    Point t0(-hx, -hy, _height), t1(hx, -hy, _height), t2(hx, hy, _height), t3(-hx, hy, _height);
    return {
        Polyline({b0, b3, b2, b1, b0}),
        Polyline({t0, t1, t2, t3, t0}),
        Polyline({b0, b1, t1, t0, b0}),
        Polyline({b2, b3, t3, t2, b2}),
        Polyline({b0, t0, t3, b3, b0}),
        Polyline({b1, b2, t2, t1, b1}),
    };
}

std::vector<Plane> ColumnElement::compute_planes() const {
    double hx = _width * 0.5;
    double hy = _depth * 0.5;
    double hz = _height * 0.5;
    Point p0(0,0,0), p1(0,0,_height), p2(0,-hy,hz), p3(0,hy,hz), p4(-hx,0,hz), p5(hx,0,hz);
    Vector n0(0,0,-1), n1(0,0,1), n2(0,-1,0), n3(0,1,0), n4(-1,0,0), n5(1,0,0);
    return {
        Plane::from_point_normal(p0, n0), Plane::from_point_normal(p1, n1),
        Plane::from_point_normal(p2, n2), Plane::from_point_normal(p3, n3),
        Plane::from_point_normal(p4, n4), Plane::from_point_normal(p5, n5),
    };
}

std::vector<Vector> ColumnElement::compute_edge_vectors() const {
    return {
        Vector(1, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, -1, 0),
        Vector(1, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, -1, 0),
        Vector(0, 0, 1), Vector(0, 0, 1), Vector(0, 0, 1), Vector(0, 0, 1),
    };
}

std::optional<Line> ColumnElement::compute_axis() const {
    return Line(0, 0, 0, 0, 0, _height);
}

ColumnElement ColumnElement::duplicate() const {
    ColumnElement result(*this);

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
    return fmt::format("ColumnElement({}, {}, {}, {}, {})", guid(), name, _width, _depth, _height);
}

nlohmann::ordered_json ColumnElement::jsondump() const {
    auto* mesh = std::get_if<Mesh>(&_geometry);
    return nlohmann::ordered_json{
        {"depth", _depth},
        {"geometry_data", mesh ? mesh->jsondump() : nullptr},
        {"geometry_type", mesh ? "Mesh" : "None"},
        {"guid", guid()},
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
    elem.guid() = data.value("guid", elem.guid());
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
    proto.set_guid(guid());
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
    elem.guid() = proto.guid();
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
    : Element(other), _width(other._width), _depth(other._depth), _length(other._length) {}

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
        Point(-hx, -hy, 0),
        Point( hx, -hy, 0),
        Point( hx,  hy, 0),
        Point(-hx,  hy, 0),
        Point(-hx, -hy, _length),
        Point( hx, -hy, _length),
        Point( hx,  hy, _length),
        Point(-hx,  hy, _length),
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1}, {4, 5, 6, 7},
        {0, 1, 5, 4}, {2, 3, 7, 6},
        {0, 4, 7, 3}, {1, 2, 6, 5},
    };
    return Mesh::from_vertices_and_faces(vertices, faces);
}

std::vector<Polyline> BeamElement::compute_polylines() const {
    double hx = _width * 0.5;
    double hy = _depth * 0.5;
    Point b0(-hx, -hy, 0), b1(hx, -hy, 0), b2(hx, hy, 0), b3(-hx, hy, 0);
    Point t0(-hx, -hy, _length), t1(hx, -hy, _length), t2(hx, hy, _length), t3(-hx, hy, _length);
    return {
        Polyline({b0, b3, b2, b1, b0}),
        Polyline({t0, t1, t2, t3, t0}),
        Polyline({b0, b1, t1, t0, b0}),
        Polyline({b2, b3, t3, t2, b2}),
        Polyline({b0, t0, t3, b3, b0}),
        Polyline({b1, b2, t2, t1, b1}),
    };
}

std::vector<Plane> BeamElement::compute_planes() const {
    double hx = _width * 0.5;
    double hy = _depth * 0.5;
    double hz = _length * 0.5;
    Point p0(0,0,0), p1(0,0,_length), p2(0,-hy,hz), p3(0,hy,hz), p4(-hx,0,hz), p5(hx,0,hz);
    Vector n0(0,0,-1), n1(0,0,1), n2(0,-1,0), n3(0,1,0), n4(-1,0,0), n5(1,0,0);
    return {
        Plane::from_point_normal(p0, n0), Plane::from_point_normal(p1, n1),
        Plane::from_point_normal(p2, n2), Plane::from_point_normal(p3, n3),
        Plane::from_point_normal(p4, n4), Plane::from_point_normal(p5, n5),
    };
}

std::vector<Vector> BeamElement::compute_edge_vectors() const {
    return {
        Vector(1, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, -1, 0),
        Vector(1, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, -1, 0),
        Vector(0, 0, 1), Vector(0, 0, 1), Vector(0, 0, 1), Vector(0, 0, 1),
    };
}

std::optional<Line> BeamElement::compute_axis() const {
    return Line(0, 0, 0, 0, 0, _length);
}

BeamElement BeamElement::duplicate() const {
    BeamElement result(*this);

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
    return fmt::format("BeamElement({}, {}, {}, {}, {})", guid(), name, _width, _depth, _length);
}

nlohmann::ordered_json BeamElement::jsondump() const {
    auto* mesh = std::get_if<Mesh>(&_geometry);
    return nlohmann::ordered_json{
        {"depth", _depth},
        {"geometry_data", mesh ? mesh->jsondump() : nullptr},
        {"geometry_type", mesh ? "Mesh" : "None"},
        {"guid", guid()},
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
    elem.guid() = data.value("guid", elem.guid());
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
    proto.set_guid(guid());
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
    elem.guid() = proto.guid();
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
        _polygon = {Point(-0.5,-0.5,0), Point(0.5,-0.5,0), Point(0.5,0.5,0), Point(-0.5,0.5,0)};
    } else {
        _polygon.reserve(polygon.size());
        for (const auto& p : polygon) _polygon.emplace_back(p[0], p[1], p[2]);
    }
    // Compute top polygon from offset
    Vector normal = polygon_normal(_polygon);
    _polygon_top.reserve(_polygon.size());
    for (const auto& p : _polygon)
        _polygon_top.emplace_back(p[0]-normal[0]*_thickness, p[1]-normal[1]*_thickness, p[2]-normal[2]*_thickness);
    _geometry = compute_element_geometry();
}

// Strip closing duplicate if polygon is closed (last == first)
static std::vector<Point> strip_closing(const std::vector<Point>& pts) {
    if (pts.size() > 3) {
        auto& f = pts.front(); auto& l = pts.back();
        if (std::abs(f[0]-l[0])<1e-6 && std::abs(f[1]-l[1])<1e-6 && std::abs(f[2]-l[2])<1e-6)
            return std::vector<Point>(pts.begin(), pts.end()-1);
    }
    return pts;
}

PlateElement::PlateElement(const Polyline& bottom, const Polyline& top, const std::string& name)
    : PlateElement(bottom.get_points(), top.get_points(), name) {}

PlateElement::PlateElement(const std::vector<Point>& bottom, const std::vector<Point>& top,
                           const std::string& name)
    : Element(name) {
    auto bot = strip_closing(bottom);
    auto tp = strip_closing(top);
    _polygon.reserve(bot.size());
    for (const auto& p : bot) _polygon.emplace_back(p[0], p[1], p[2]);
    _polygon_top.reserve(tp.size());
    for (const auto& p : tp) _polygon_top.emplace_back(p[0], p[1], p[2]);
    // Ensure bottom normal points toward top
    Vector norm = polygon_normal(_polygon);
    size_t np = std::min(_polygon.size(), _polygon_top.size());
    double d = 0;
    for (size_t k = 0; k < np; k++)
        d += (_polygon_top[k][0]-_polygon[k][0])*norm[0] + (_polygon_top[k][1]-_polygon[k][1])*norm[1] + (_polygon_top[k][2]-_polygon[k][2])*norm[2];
    if (d < 0) std::swap(_polygon, _polygon_top);
    // Compute thickness from average distance
    _thickness = 0;
    for (size_t k = 0; k < np; k++) {
        double dx=_polygon_top[k][0]-_polygon[k][0], dy=_polygon_top[k][1]-_polygon[k][1], dz=_polygon_top[k][2]-_polygon[k][2];
        _thickness += std::sqrt(dx*dx+dy*dy+dz*dz);
    }
    _thickness /= np;
    _geometry = compute_element_geometry();
}

PlateElement::PlateElement(const PlateElement& other)
    : Element(other), _polygon(other._polygon), _polygon_top(other._polygon_top),
      _thickness(other._thickness), _joint_types(other._joint_types), _j_mf(other._j_mf),
      _key(other._key), _component_plane(other._component_plane) {}

PlateElement& PlateElement::operator=(const PlateElement& other) {
    if (this != &other) {
        Element::operator=(other);
        _polygon = other._polygon;
        _polygon_top = other._polygon_top;
        _thickness = other._thickness;
        _joint_types = other._joint_types;
        _j_mf = other._j_mf;
        _key = other._key;
        _component_plane = other._component_plane;
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

void PlateElement::set_polygon_top(const std::vector<Point>& pts) {
    _polygon_top.clear();
    _polygon_top.reserve(pts.size());
    for (const auto& p : pts) _polygon_top.emplace_back(p[0], p[1], p[2]);
    _geometry = compute_element_geometry();
    reset();
}

void PlateElement::set_thickness(double v) {
    _thickness = v;
    Vector normal = polygon_normal(_polygon);
    _polygon_top.clear();
    _polygon_top.reserve(_polygon.size());
    for (const auto& p : _polygon)
        _polygon_top.emplace_back(p[0]-normal[0]*v, p[1]-normal[1]*v, p[2]-normal[2]*v);
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

AABB PlateElement::compute_aabb_fast(double inflate) const {
    double minx=1e30,miny=1e30,minz=1e30,maxx=-1e30,maxy=-1e30,maxz=-1e30;
    for (auto* poly : {&_polygon, &_polygon_top}) {
        for (auto& p : *poly) {
            if(p[0]<minx)minx=p[0]; if(p[1]<miny)miny=p[1]; if(p[2]<minz)minz=p[2];
            if(p[0]>maxx)maxx=p[0]; if(p[1]>maxy)maxy=p[1]; if(p[2]>maxz)maxz=p[2];
        }
    }
    return AABB{(minx+maxx)*0.5, (miny+maxy)*0.5, (minz+maxz)*0.5,
                (maxx-minx)*0.5+inflate, (maxy-miny)*0.5+inflate, (maxz-minz)*0.5+inflate};
}

Mesh PlateElement::compute_element_geometry() const {
    size_t n = std::min(_polygon.size(), _polygon_top.size());
    std::vector<Point> vertices;
    vertices.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) vertices.emplace_back(_polygon[i][0], _polygon[i][1], _polygon[i][2]);
    for (size_t i = 0; i < n; ++i) vertices.emplace_back(_polygon_top[i][0], _polygon_top[i][1], _polygon_top[i][2]);
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
    Mesh mesh = Mesh::from_vertices_and_faces(vertices, faces);
    // Store triangulation for all faces (enables explode_mesh_faces in Rhino)
    for (auto& [fk, vks] : mesh.face) {
        if (vks.size() == 4) {
            mesh.set_face_triangulation(fk, {{vks[0], vks[1], vks[2]}, {vks[0], vks[2], vks[3]}});
        } else if (vks.size() > 4) {
            std::vector<std::array<size_t,3>> tris;
            for (size_t k = 1; k + 1 < vks.size(); k++)
                tris.push_back({vks[0], vks[k], vks[k+1]});
            mesh.set_face_triangulation(fk, tris);
        }
    }
    return mesh;
}

std::vector<Polyline> PlateElement::compute_polylines() const {
    size_t n = std::min(_polygon.size(), _polygon_top.size());
    // [0]=top polyline, [1]=bottom polyline, [2+]=side polylines (matching wood)
    std::vector<Point> top_closed;
    for (size_t i = 0; i < n; ++i) top_closed.push_back(_polygon_top[i]);
    top_closed.push_back(_polygon_top[0]);
    std::vector<Point> bot_closed;
    for (size_t i = 0; i < n; ++i) bot_closed.push_back(_polygon[i]);
    bot_closed.push_back(_polygon[0]);
    std::vector<Polyline> result;
    result.push_back(Polyline(top_closed));
    result.push_back(Polyline(bot_closed));
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        result.push_back(Polyline({_polygon_top[i], _polygon_top[j], _polygon[j], _polygon[i], _polygon_top[i]}));
    }
    return result;
}

std::vector<Plane> PlateElement::compute_planes() const {
    Vector normal = polygon_normal(_polygon);
    size_t n = std::min(_polygon.size(), _polygon_top.size());
    // [0]=top plane, [1]=bottom plane, [2+]=side planes (matching wood)
    double tcx=0,tcy=0,tcz=0, bcx=0,bcy=0,bcz=0;
    for (size_t i=0;i<n;i++){
        tcx+=_polygon_top[i][0]; tcy+=_polygon_top[i][1]; tcz+=_polygon_top[i][2];
        bcx+=_polygon[i][0]; bcy+=_polygon[i][1]; bcz+=_polygon[i][2];
    }
    tcx/=n;tcy/=n;tcz/=n; bcx/=n;bcy/=n;bcz/=n;
    std::vector<Plane> result;
    Point tp(tcx,tcy,tcz);
    result.push_back(Plane::from_point_normal(tp, normal));
    Point bp(bcx,bcy,bcz);
    Vector neg(-normal[0],-normal[1],-normal[2]);
    result.push_back(Plane::from_point_normal(bp, neg));
    // Side planes from 3 actual points: cross(top[k]-top[k+1], bot[k+1]-top[k+1])
    for (size_t i=0;i<n;i++){
        size_t j=(i+1)%n;
        double ax=_polygon_top[i][0]-_polygon_top[j][0], ay=_polygon_top[i][1]-_polygon_top[j][1], az=_polygon_top[i][2]-_polygon_top[j][2];
        double bx=_polygon[j][0]-_polygon_top[j][0], by=_polygon[j][1]-_polygon_top[j][1], bz=_polygon[j][2]-_polygon_top[j][2];
        double nx=ay*bz-az*by, ny=az*bx-ax*bz, nz=ax*by-ay*bx;
        double mag=std::sqrt(nx*nx+ny*ny+nz*nz);
        if(mag>1e-12){nx/=mag;ny/=mag;nz/=mag;}
        double cx=(_polygon_top[i][0]+_polygon_top[j][0]+_polygon[j][0]+_polygon[i][0])*0.25;
        double cy=(_polygon_top[i][1]+_polygon_top[j][1]+_polygon[j][1]+_polygon[i][1])*0.25;
        double cz=(_polygon_top[i][2]+_polygon_top[j][2]+_polygon[j][2]+_polygon[i][2])*0.25;
        Point sp(cx,cy,cz); Vector sn(nx,ny,nz);
        result.push_back(Plane::from_point_normal(sp, sn));
    }
    return result;
}

std::vector<Vector> PlateElement::compute_edge_vectors() const {
    size_t n = _polygon.size();
    std::vector<Vector> result;
    result.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double dx = _polygon[j][0] - _polygon[i][0];
        double dy = _polygon[j][1] - _polygon[i][1];
        double dz = _polygon[j][2] - _polygon[i][2];
        double mag = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (mag > 1e-12) result.emplace_back(dx / mag, dy / mag, dz / mag);
        else result.emplace_back(0, 0, 0);
    }
    return result;
}

std::optional<Line> PlateElement::compute_axis() const {
    size_t n = std::min(_polygon.size(), _polygon_top.size());
    double bcx=0,bcy=0,bcz=0,tcx=0,tcy=0,tcz=0;
    for (size_t i=0;i<n;i++){
        bcx+=_polygon[i][0];bcy+=_polygon[i][1];bcz+=_polygon[i][2];
        tcx+=_polygon_top[i][0];tcy+=_polygon_top[i][1];tcz+=_polygon_top[i][2];
    }
    return Line(bcx/n,bcy/n,bcz/n, tcx/n,tcy/n,tcz/n);
}

PlateElement PlateElement::duplicate() const {
    PlateElement result(*this);

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
    return fmt::format("PlateElement({}, {}, {} pts, {})", guid(), name, _polygon.size(), _thickness);
}

nlohmann::ordered_json PlateElement::jsondump() const {
    auto* mesh = std::get_if<Mesh>(&_geometry);
    nlohmann::ordered_json poly_json = nlohmann::ordered_json::array();
    for (const auto& p : _polygon)
        poly_json.push_back({p[0], p[1], p[2]});
    nlohmann::ordered_json j_mf_json = nlohmann::ordered_json::array();
    for (const auto& face : _j_mf) {
        nlohmann::ordered_json face_json = nlohmann::ordered_json::array();
        for (const auto& jc : face)
            face_json.push_back({jc.joint_id, jc.is_male, jc.parameter});
        j_mf_json.push_back(face_json);
    }
    nlohmann::ordered_json poly_top_json = nlohmann::ordered_json::array();
    for (const auto& p : _polygon_top)
        poly_top_json.push_back({p[0], p[1], p[2]});
    return nlohmann::ordered_json{
        {"component_plane", _component_plane.has_value() ? _component_plane->jsondump() : nullptr},
        {"geometry_data", mesh ? mesh->jsondump() : nullptr},
        {"geometry_type", mesh ? "Mesh" : "None"},
        {"guid", guid()},
        {"j_mf", j_mf_json},
        {"joint_types", _joint_types},
        {"key", _key},
        {"name", name},
        {"polygon", poly_json},
        {"polygon_top", poly_top_json},
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
    std::vector<Point> polygon_top;
    if (data.contains("polygon_top")) {
        for (const auto& p : data["polygon_top"])
            polygon_top.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    }
    PlateElement elem = polygon_top.empty()
        ? PlateElement(polygon.empty() ? std::vector<Point>{} : polygon, data.value("thickness", 0.1))
        : PlateElement(polygon, polygon_top);
    elem.guid() = data.value("guid", elem.guid());
    elem.name = data.value("name", elem.name);
    if (data.contains("session_transformation"))
        elem.session_transformation = Xform::jsonload(data["session_transformation"]);
    if (data.contains("joint_types"))
        elem._joint_types = data["joint_types"].get<std::vector<int>>();
    if (data.contains("j_mf")) {
        for (const auto& face : data["j_mf"]) {
            std::vector<JointConnection> fv;
            for (const auto& jc : face)
                fv.push_back({jc[0].get<int>(), jc[1].get<bool>(), jc[2].get<double>()});
            elem._j_mf.push_back(fv);
        }
    }
    elem._key = data.value("key", std::string(""));
    if (data.contains("component_plane") && !data["component_plane"].is_null())
        elem._component_plane = Plane::jsonload(data["component_plane"]);
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
    proto.set_guid(guid());
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
    for (int jt : _joint_types) proto.add_joint_types(jt);
    for (const auto& face : _j_mf) {
        auto* fj = proto.add_j_mf();
        for (const auto& jc : face) {
            auto* c = fj->add_connections();
            c->set_joint_id(jc.joint_id);
            c->set_is_male(jc.is_male);
            c->set_parameter(jc.parameter);
        }
    }
    proto.set_key(_key);
    if (_component_plane.has_value()) {
        auto* cp = proto.mutable_component_plane();
        cp->set_name(_component_plane->name);
        cp->add_frame(_component_plane->origin()[0]);
        cp->add_frame(_component_plane->origin()[1]);
        cp->add_frame(_component_plane->origin()[2]);
        cp->add_frame(_component_plane->x_axis()[0]);
        cp->add_frame(_component_plane->x_axis()[1]);
        cp->add_frame(_component_plane->x_axis()[2]);
        cp->add_frame(_component_plane->y_axis()[0]);
        cp->add_frame(_component_plane->y_axis()[1]);
        cp->add_frame(_component_plane->y_axis()[2]);
        cp->add_frame(_component_plane->z_axis()[0]);
        cp->add_frame(_component_plane->z_axis()[1]);
        cp->add_frame(_component_plane->z_axis()[2]);
    }
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
    elem.guid() = proto.guid();
    elem.name = proto.name();
    Xform xf;
    xf.name = proto.session_transformation().name();
    if (proto.session_transformation().matrix_size() == 16)
        for (int i = 0; i < 16; ++i) xf.m[i] = proto.session_transformation().matrix(i);
    elem.session_transformation = xf;
    elem._joint_types.assign(proto.joint_types().begin(), proto.joint_types().end());
    for (const auto& fj : proto.j_mf()) {
        std::vector<JointConnection> face;
        for (const auto& c : fj.connections())
            face.push_back({c.joint_id(), c.is_male(), c.parameter()});
        elem._j_mf.push_back(face);
    }
    elem._key = proto.key();
    if (proto.has_component_plane() && proto.component_plane().frame_size() == 12) {
        const auto& f = proto.component_plane().frame();
        elem._component_plane = Plane(
            Point(f[0], f[1], f[2]),
            Vector(f[3], f[4], f[5]),
            Vector(f[6], f[7], f[8])
        );
        elem._component_plane->name = proto.component_plane().name();
    }
    return elem;
}

PlateElement PlateElement::pb_load(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

} // namespace session_cpp
