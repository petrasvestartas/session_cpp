#include "plane.h"
#include "tolerance.h"
#include <algorithm>

#include "plane.pb.h"
#include "point.pb.h"
#include "vector.pb.h"
#include "xform.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

Plane::Plane() {
    xform = Xform::identity();
    _origin = Point(0.0, 0.0, 0.0);
    _x_axis = Vector::x_axis();
    _y_axis = Vector::y_axis();
    _z_axis = Vector::z_axis();
    _a = 0.0;
    _b = 0.0;
    _c = 1.0;
    _d = 0.0;
}

Plane::Plane(const Plane& other)
    : guid(::guid()),
      name(other.name),
      width(other.width),
      xform(other.xform),
      _origin(other._origin),
      _x_axis(other._x_axis),
      _y_axis(other._y_axis),
      _z_axis(other._z_axis),
      _a(other._a),
      _b(other._b),
      _c(other._c),
      _d(other._d) {
}

Plane& Plane::operator=(const Plane& other) {
    if (this != &other) {
        guid = ::guid();
        name = other.name;
        width = other.width;
        xform = other.xform;
        _origin = other._origin;
        _x_axis = other._x_axis;
        _y_axis = other._y_axis;
        _z_axis = other._z_axis;
        _a = other._a;
        _b = other._b;
        _c = other._c;
        _d = other._d;
    }
    return *this;
}

Plane::Plane(Point& point, Vector& x_axis, Vector& y_axis, std::string name) {
    xform = Xform::identity();
    this->name = name;
    _origin = point;
    _x_axis = x_axis;
    _x_axis.normalize_self();
    _y_axis = y_axis - x_axis * (y_axis.dot(_x_axis));
    _y_axis.normalize_self();
    _z_axis = _x_axis.cross(_y_axis);
    _z_axis.normalize_self();
    
    _a = _z_axis[0];
    _b = _z_axis[1];
    _c = _z_axis[2];
    _d = -(_a * _origin[0] + _b * _origin[1] + _c * _origin[2]);
}

Plane Plane::from_point_normal(Point& point, Vector& normal) {
    Plane plane;
    plane._origin = point;
    plane._z_axis = normal;
    plane._z_axis.normalize_self();
    plane._x_axis.perpendicular_to(plane._z_axis);
    plane._x_axis.normalize_self();
    plane._y_axis = plane._z_axis.cross(plane._x_axis);
    plane._y_axis.normalize_self();
    
    plane._a = plane._z_axis[0];
    plane._b = plane._z_axis[1];
    plane._c = plane._z_axis[2];
    plane._d = -(plane._a * plane._origin[0] + plane._b * plane._origin[1] + plane._c * plane._origin[2]);
    return plane;
}

Plane Plane::from_points(std::vector<Point>& points) {
    if (points.size() < 3) {
        return Plane();
    }
    
    Plane plane;
    plane._origin = points[0];
    
    Vector v1 = points[1] - points[0];
    Vector v2 = points[2] - points[0];
    plane._z_axis = v1.cross(v2);
    plane._z_axis.normalize_self();
    
    plane._x_axis = v1;
    plane._x_axis.normalize_self();
    plane._y_axis = plane._z_axis.cross(plane._x_axis);
    plane._y_axis.normalize_self();
    
    plane._a = plane._z_axis[0];
    plane._b = plane._z_axis[1];
    plane._c = plane._z_axis[2];
    plane._d = -(plane._a * plane._origin[0] + plane._b * plane._origin[1] + plane._c * plane._origin[2]);
    return plane;
}

Plane Plane::from_two_points(Point& point1, Point& point2) {
    Plane plane;
    plane._origin = point1;
    
    Vector direction = point2 - point1;
    direction.normalize_self();
    plane._z_axis.perpendicular_to(direction);
    plane._z_axis.normalize_self();
    
    plane._x_axis = direction;
    
    plane._y_axis = plane._z_axis.cross(plane._x_axis);
    plane._y_axis.normalize_self();
    
    plane._a = plane._z_axis[0];
    plane._b = plane._z_axis[1];
    plane._c = plane._z_axis[2];
    plane._d = -(plane._a * plane._origin[0] + plane._b * plane._origin[1] + plane._c * plane._origin[2]);
    return plane;
}

Plane Plane::xy_plane() {
    Plane plane;
    plane.name = "xy_plane";
    plane._origin = Point(0.0, 0.0, 0.0);
    plane._x_axis = Vector::x_axis();
    plane._y_axis = Vector::y_axis();
    plane._z_axis = Vector::z_axis();
    plane._a = 0.0;
    plane._b = 0.0;
    plane._c = 1.0;
    plane._d = 0.0;
    return plane;
}

Plane Plane::yz_plane() {
    Plane plane;
    plane.name = "yz_plane";
    plane._origin = Point(0.0, 0.0, 0.0);
    plane._x_axis = Vector::y_axis();
    plane._y_axis = Vector::z_axis();
    plane._z_axis = Vector::x_axis();
    plane._a = 1.0;
    plane._b = 0.0;
    plane._c = 0.0;
    plane._d = 0.0;
    return plane;
}

Plane Plane::xz_plane() {
    Plane plane;
    plane.name = "xz_plane";
    plane._origin = Point(0.0, 0.0, 0.0);
    plane._x_axis = Vector::x_axis();
    plane._y_axis = Vector(0.0, 0.0, -1.0);
    plane._z_axis = Vector(0.0, 1.0, 0.0);
    plane._a = 0.0;
    plane._b = 1.0;
    plane._c = 0.0;
    plane._d = 0.0;
    return plane;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////////////////

std::string Plane::str() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format("{}, {}, {}",
                       TOLERANCE.format_number(_origin[0], prec),
                       TOLERANCE.format_number(_origin[1], prec),
                       TOLERANCE.format_number(_origin[2], prec));
}

std::string Plane::repr() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format("Plane({}, {}, {}, {}, {}, {}, {})",
                       name,
                       TOLERANCE.format_number(_origin[0], prec),
                       TOLERANCE.format_number(_origin[1], prec),
                       TOLERANCE.format_number(_origin[2], prec),
                       TOLERANCE.format_number(_z_axis[0], prec),
                       TOLERANCE.format_number(_z_axis[1], prec),
                       TOLERANCE.format_number(_z_axis[2], prec));
}


///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void Plane::transform() {
  xform.transform_point(_origin);
  xform.transform_vector(_x_axis);
  xform.transform_vector(_y_axis);
  xform.transform_vector(_z_axis);
  xform = Xform::identity();
}

Plane Plane::transformed() const {
  Plane result = *this;
  result.transform();
  return result;
}

bool Plane::operator==(const Plane &other) const {
    return name == other.name &&
           _origin == other._origin &&
           _x_axis == other._x_axis &&
           _y_axis == other._y_axis &&
           _z_axis == other._z_axis;
}

bool Plane::operator!=(const Plane &other) const {
    return !(*this == other);
}

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

Vector &Plane::operator[](int index) {
    if (index == 0)
        return _x_axis;
    if (index == 1)
        return _y_axis;
    return _z_axis;
}

const Vector &Plane::operator[](int index) const {
    if (index == 0)
        return _x_axis;
    if (index == 1)
        return _y_axis;
    return _z_axis;
}

Plane &Plane::operator+=(const Vector &other) {
    _origin += other;
    _d = -(_a * _origin[0] + _b * _origin[1] + _c * _origin[2]);
    return *this;
}

Plane &Plane::operator-=(const Vector &other) {
    _origin -= other;
    _d = -(_a * _origin[0] + _b * _origin[1] + _c * _origin[2]);
    return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

Plane Plane::operator+(const Vector &other) const {
    Plane result = *this;
    result += other;
    return result;
}

Plane Plane::operator-(const Vector &other) const {
    Plane result = *this;
    result -= other;
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Plane::jsondump() const {
    auto clean_float = [](double val) -> double {
        return static_cast<double>(std::round(val * 100.0) / 100.0);
    };
    // Alphabetical order to match Rust's serde_json
    // Use single flat frame array of 12 numbers: [ox, oy, oz, xx, xy, xz, yx, yy, yz, zx, zy, zz]
    nlohmann::ordered_json data;
    data["frame"] = {
        clean_float(_origin[0]), clean_float(_origin[1]), clean_float(_origin[2]),
        clean_float(_x_axis[0]), clean_float(_x_axis[1]), clean_float(_x_axis[2]),
        clean_float(_y_axis[0]), clean_float(_y_axis[1]), clean_float(_y_axis[2]),
        clean_float(_z_axis[0]), clean_float(_z_axis[1]), clean_float(_z_axis[2])
    };
    data["guid"] = guid;
    data["name"] = name;
    data["type"] = "Plane";
    data["width"] = clean_float(width);
    data["xform"] = xform.jsondump();
    return data;
}

Plane Plane::jsonload(const nlohmann::json &data) {
    Plane plane;
    // Parse flat frame array of 12 numbers: [ox, oy, oz, xx, xy, xz, yx, yy, yz, zx, zy, zz]
    auto frame = data["frame"];

    plane._origin = Point(frame[0].get<double>(), frame[1].get<double>(), frame[2].get<double>());
    plane._x_axis = Vector(frame[3].get<double>(), frame[4].get<double>(), frame[5].get<double>());
    plane._y_axis = Vector(frame[6].get<double>(), frame[7].get<double>(), frame[8].get<double>());
    plane._z_axis = Vector(frame[9].get<double>(), frame[10].get<double>(), frame[11].get<double>());
    plane.guid = data["guid"];
    plane.name = data["name"];
    if (data.contains("width")) {
        plane.width = data["width"].get<double>();
    }
    if (data.contains("xform")) {
        plane.xform = Xform::jsonload(data["xform"]);
    }

    plane._a = plane._z_axis[0];
    plane._b = plane._z_axis[1];
    plane._c = plane._z_axis[2];
    plane._d = -(plane._a * plane._origin[0] + plane._b * plane._origin[1] + plane._c * plane._origin[2]);
    return plane;
}

std::string Plane::json_dumps() const {
    return jsondump().dump();
}

Plane Plane::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Plane::json_dump(const std::string& filename) const {
    std::ofstream ofs(filename);
    ofs << jsondump().dump(2);
    ofs.close();
}

Plane Plane::json_load(const std::string& filename) {
    std::ifstream ifs(filename);
    nlohmann::json data;
    ifs >> data;
    ifs.close();
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

std::string Plane::pb_dumps() const {
    session_proto::Plane proto;
    proto.set_guid(guid);
    proto.set_name(name);

    // Frame: origin(3) + x_axis(3) + y_axis(3) + z_axis(3) = 12 doubles
    proto.add_frame(_origin[0]);
    proto.add_frame(_origin[1]);
    proto.add_frame(_origin[2]);
    proto.add_frame(_x_axis[0]);
    proto.add_frame(_x_axis[1]);
    proto.add_frame(_x_axis[2]);
    proto.add_frame(_y_axis[0]);
    proto.add_frame(_y_axis[1]);
    proto.add_frame(_y_axis[2]);
    proto.add_frame(_z_axis[0]);
    proto.add_frame(_z_axis[1]);
    proto.add_frame(_z_axis[2]);

    // Serialize xform
    auto* proto_xform = proto.mutable_xform();
    proto_xform->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        proto_xform->add_matrix(xform.m[i]);
    }
    return proto.SerializeAsString();
}

Plane Plane::pb_loads(const std::string& data) {
    session_proto::Plane proto;
    proto.ParseFromString(data);

    Plane plane;
    plane.guid = proto.guid();
    plane.name = proto.name();

    // Parse frame: origin(3) + x_axis(3) + y_axis(3) + z_axis(3) = 12 doubles
    if (proto.frame_size() >= 12) {
        plane._origin = Point(proto.frame(0), proto.frame(1), proto.frame(2));
        plane._x_axis = Vector(proto.frame(3), proto.frame(4), proto.frame(5));
        plane._y_axis = Vector(proto.frame(6), proto.frame(7), proto.frame(8));
        plane._z_axis = Vector(proto.frame(9), proto.frame(10), proto.frame(11));
    }

    // Compute plane equation coefficients
    plane._a = plane._z_axis[0];
    plane._b = plane._z_axis[1];
    plane._c = plane._z_axis[2];
    plane._d = -(plane._a * plane._origin[0] + plane._b * plane._origin[1] + plane._c * plane._origin[2]);

    // Deserialize xform if present
    if (proto.has_xform()) {
        plane.xform.name = proto.xform().name();
        for (int i = 0; i < proto.xform().matrix_size() && i < 16; ++i) {
            plane.xform.m[i] = proto.xform().matrix(i);
        }
    }
    return plane;
}

void Plane::pb_dump(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << pb_dumps();
    ofs.close();
}

Plane Plane::pb_load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
    ifs.close();
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////

void Plane::reverse() {
    Vector temp = _x_axis;
    _x_axis = _y_axis;
    _y_axis = temp;
    _z_axis.reverse();
    
    _a = _z_axis[0];
    _b = _z_axis[1];
    _c = _z_axis[2];
    _d = -(_a * _origin[0] + _b * _origin[1] + _c * _origin[2]);
}

void Plane::rotate(double angles_in_radians) {
    double cos_angle = std::cos(angles_in_radians);
    double sin_angle = std::sin(angles_in_radians);
    
    Vector new_x = _x_axis * cos_angle + _y_axis * sin_angle;
    Vector new_y = _y_axis * cos_angle - _x_axis * sin_angle;
    
    _x_axis = new_x;
    _y_axis = new_y;
    
    _a = _z_axis[0];
    _b = _z_axis[1];
    _c = _z_axis[2];
    _d = -(_a * _origin[0] + _b * _origin[1] + _c * _origin[2]);
}

bool Plane::is_right_hand() const {
    Vector x_copy = _x_axis;
    Vector y_copy = _y_axis;
    Vector z_copy = _z_axis;
    Vector cross = x_copy.cross(y_copy);
    double dot_product = cross.dot(z_copy);
    return dot_product > 0.999;
}

bool Plane::is_same_direction(const Plane &plane0, const Plane &plane1, bool can_be_flipped) {
    Vector n0 = plane0._z_axis;
    Vector n1 = plane1._z_axis;
    
    int parallel = n0.is_parallel_to(n1);
    
    if (can_be_flipped) {
        return parallel != 0;
    } else {
        return parallel == 1;
    }
}

bool Plane::is_same_position(const Plane &plane0, const Plane &plane1) {
    double dist0 = std::abs(plane0._a * plane1._origin[0] + 
                           plane0._b * plane1._origin[1] + 
                           plane0._c * plane1._origin[2] + 
                           plane0._d);
    
    double dist1 = std::abs(plane1._a * plane0._origin[0] + 
                           plane1._b * plane0._origin[1] + 
                           plane1._c * plane0._origin[2] + 
                           plane1._d);
    
    double tolerance = static_cast<double>(session_cpp::Tolerance::ZERO_TOLERANCE);
    return dist0 < tolerance && dist1 < tolerance;
}

bool Plane::is_coplanar(const Plane &plane0, const Plane plane1, bool can_be_flipped) {
    return is_same_direction(plane0, plane1, can_be_flipped) && 
           is_same_position(plane0, plane1);
}

Plane Plane::translate_by_normal(double distance) const {
    // Get normalized normal vector (z_axis)
    Vector normal = _z_axis;
    normal.normalize_self();
    
    // Move origin along the normal
    Point new_origin = _origin + (normal * distance);
    
    // Create new plane with same orientation but new origin
    Vector x_copy = _x_axis;
    Vector y_copy = _y_axis;
    return Plane(new_origin, x_copy, y_copy, name);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &os, const Plane &plane) {
    os << plane.str();
    return os;
}

} // namespace session_cpp
