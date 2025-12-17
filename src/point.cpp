#include "point.h"
#include "tolerance.h"

#ifdef ENABLE_PROTOBUF
#include "point.pb.h"
#include "color.pb.h"
#include "xform.pb.h"
#endif

namespace session_cpp {

/// Copy constructor (creates a new guid while copying data)
Point::Point(const Point &other)
    : guid(::guid()),
      name(other.name),
      width(other.width),
      pointcolor(other.pointcolor),
      xform(other.xform),
      _x(other._x),
      _y(other._y),
      _z(other._z) {}

/// Copy assignment (creates a new guid while copying data)
Point &Point::operator=(const Point &other) {
  if (this != &other) {
    guid = ::guid();
    name = other.name;
    width = other.width;
    pointcolor = other.pointcolor;
    xform = other.xform;
    _x = other._x;
    _y = other._y;
    _z = other._z;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void Point::transform() {
  xform.transform_point(*this);
  xform = Xform::identity();
}

Point Point::transformed() const {
  Point result = *this;
  result.transform();
  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

/// Convert to JSON-serializable object
nlohmann::ordered_json Point::jsondump() const {
  auto clean_float = [](double val) -> double { return std::round(val * 100.0) / 100.0; };
  return nlohmann::ordered_json{
      {"type", "Point"}, {"guid", guid},
      {"name", name},    {"x", clean_float(_x)},
      {"y", clean_float(_y)}, {"z", clean_float(_z)},
      {"width", width},  {"pointcolor", pointcolor.jsondump()},
      {"xform", xform.jsondump()}};
}

/// Create point from JSON data
Point Point::jsonload(const nlohmann::json &data) {
  Point point(data["x"], data["y"], data["z"]);
  point.guid = data["guid"];
  point.name = data["name"];
  point.pointcolor = Color::jsonload(data["pointcolor"]);
  point.width = data["width"];
  if (data.contains("xform")) {
    point.xform = Xform::jsonload(data["xform"]);
  }
  return point;
}

/// Write JSON to file
void Point::json_dump(const std::string& filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

/// Read JSON from file
Point Point::json_load(const std::string& filename) {
  std::ifstream file(filename);
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_PROTOBUF
std::string Point::to_protobuf() const {
  session_proto::Point proto;
  proto.set_guid(guid);
  proto.set_name(name);
  proto.set_x(_x);
  proto.set_y(_y);
  proto.set_z(_z);
  proto.set_width(width);
  
  // Set color (no guid in proto schema)
  auto* color_proto = proto.mutable_pointcolor();
  color_proto->set_name(pointcolor.name);
  color_proto->set_r(pointcolor.r);
  color_proto->set_g(pointcolor.g);
  color_proto->set_b(pointcolor.b);
  color_proto->set_a(pointcolor.a);
  
  // Set xform
  auto* xform_proto = proto.mutable_xform();
  xform_proto->set_guid(xform.guid);
  xform_proto->set_name(xform.name);
  for (int i = 0; i < 16; ++i) {
    xform_proto->add_matrix(xform.m[i]);
  }
  
  return proto.SerializeAsString();
}

Point Point::from_protobuf(const std::string& data) {
  session_proto::Point proto;
  proto.ParseFromString(data);
  
  Point point(proto.x(), proto.y(), proto.z());
  point.guid = proto.guid();
  point.name = proto.name();
  point.width = proto.width();
  
  // Load color (no guid in proto schema)
  const auto& color_proto = proto.pointcolor();
  point.pointcolor.name = color_proto.name();
  point.pointcolor.r = color_proto.r();
  point.pointcolor.g = color_proto.g();
  point.pointcolor.b = color_proto.b();
  point.pointcolor.a = color_proto.a();
  
  // Load xform
  const auto& xform_proto = proto.xform();
  point.xform.guid = xform_proto.guid();
  point.xform.name = xform_proto.name();
  for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i) {
    point.xform.m[i] = xform_proto.matrix(i);
  }
  
  return point;
}

void Point::protobuf_dump(const std::string& filename) const {
  std::string data = to_protobuf();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Point Point::protobuf_load(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return from_protobuf(data);
}
#endif

/// Simple string form (like Python __str__): just coordinates
std::string Point::str() const {
  int prec = static_cast<int>(Tolerance::ROUNDING);
  return fmt::format(
      "{}, {}, {}",
      TOLERANCE.format_number(_x, prec),
      TOLERANCE.format_number(_y, prec),
      TOLERANCE.format_number(_z, prec));
}

/// Detailed representation (like Python __repr__)
std::string Point::repr() const {
  int prec = static_cast<int>(Tolerance::ROUNDING);
  return fmt::format(
      "Point({}, {}, {}, {}, Color({}, {}, {}, {}), {})",
      name,
      TOLERANCE.format_number(_x, prec),
      TOLERANCE.format_number(_y, prec),
      TOLERANCE.format_number(_z, prec),
      pointcolor.r, pointcolor.g, pointcolor.b, pointcolor.a,
      TOLERANCE.format_number(width, prec));
}

/// Equality operator
bool Point::operator==(const Point &other) const {
  return _x == other._x && _y == other._y && _z == other._z;
}

/// Inequality operator
bool Point::operator!=(const Point &other) const { return !(*this == other); }

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

double& Point::operator[](int index) {
  if (index == 0) {
    return _x;
  } else if (index == 1) {
    return _y;
  } else if (index == 2) {
    return _z;
  } else {
    throw std::out_of_range("Index out of range");
  }
}

const double &Point::operator[](int index) const{
  if (index == 0) {
    return _x;
  } else if (index == 1) {
    return _y;
  } else if (index == 2) {
    return _z;
  } else {
    throw std::out_of_range("Index out of range");
  }
}

Point &Point::operator*=(double factor) {
  _x *= factor;
  _y *= factor;
  _z *= factor;
  return *this;
}

Point &Point::operator/=(double factor) {
  _x /= factor;
  _y /= factor;
  _z /= factor;
  return *this;
}

Point& Point::operator+=(const Vector& other) {
  _x += other[0];
  _y += other[1];
  _z += other[2];
  return *this;
}

Point& Point::operator-=(const Vector& other) {
  _x -= other[0];
  _y -= other[1];
  _z -= other[2];
  return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

Point Point::operator*(double factor) const {
  Point result = *this;
  result *= factor;
  return result;
}

Point Point::operator/(double factor) const {
  Point result = *this;
  result /= factor;
  return result;
}

Point Point::operator+(const Vector& other) const {
  Point result(_x + other[0], _y + other[1], _z + other[2]);
  return result;
}

Point Point::operator-(const Vector& other) const {
  Point result(_x - other[0], _y - other[1], _z - other[2]);
  return result;
}

Vector Point::operator-(const Point& other) const {
  Vector result(_x - other._x, _y - other._y, _z - other._z);
  return result;
}

Point Point::sum(const Point& p0, const Point& p1) {
  return Point(p0._x + p1._x, p0._y + p1._y, p0._z + p1._z);
}

Point Point::sub(const Point& p0, const Point& p1) {
  return Point(p0._x - p1._x, p0._y - p1._y, p0._z - p1._z);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////

bool Point::ccw(const Point& a, const Point& b, const Point& c) {
    return (c._y - a._y) * (b._x - a._x) > (b._y - a._y) * (c._x - a._x);
}

bool Point::is_ccw(const Point& a, const Point& b, const Point& c) {
    return ccw(a, b, c);
}

Point Point::mid_point(const Point& p) const {
    return Point((_x + p._x) / 2, (_y + p._y) / 2, (_z + p._z) / 2);
}

Point Point::mid_point(const Point& a, const Point& b) {
    return a.mid_point(b);
}

double Point::distance(const Point& p, double float_min) const {
    double dx = std::abs((*this)[0] - p[0]);
    double dy = std::abs((*this)[1] - p[1]);
    double dz = std::abs((*this)[2] - p[2]);
    double length = 0.0;

    // Reorder coordinates to put largest in dx
    if (dy >= dx && dy >= dz) {
        std::swap(dx, dy);
    } else if (dz >= dx && dz >= dy) {
        std::swap(dx, dz);
    }

    if (dx > float_min) {
        dy /= dx;
        dz /= dx;
        length = dx * std::sqrt(1.0 + dy * dy + dz * dz);
    } else if (dx > 0.0 && std::isfinite(dx)) {
        length = dx;
    } else {
        length = 0.0;
    }

    return length;
}

double Point::squared_distance(const Point& p, double float_min) const {
    double dx = std::abs((*this)[0] - p[0]);
    double dy = std::abs((*this)[1] - p[1]);
    double dz = std::abs((*this)[2] - p[2]);
    double length2 = 0.0;

    if (dy >= dx && dy >= dz) {
        std::swap(dx, dy);
    } else if (dz >= dx && dz >= dy) {
        std::swap(dx, dz);
    }

    if (dx > float_min) {
        dy /= dx;
        dz /= dx;
        length2 = dx * dx * (1.0 + dy * dy + dz * dz);
    } else if (dx > 0.0 && std::isfinite(dx)) {
        length2 = dx * dx;
    } else {
        length2 = 0.0;
    }

    return length2;
}

double Point::distance(const Point& a, const Point& b, double float_min) {
    return a.distance(b, float_min);
}

double Point::squared_distance(const Point& a, const Point& b, double float_min) {
    return a.squared_distance(b, float_min);
}

double Point::area(const std::vector<Point>& points) {
    size_t n = points.size();
    double area = 0.0;
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += points[i][0] * points[j][1];
        area -= points[j][0] * points[i][1];
    }
    return std::abs(area) / 2.0;
}

Point Point::centroid_quad(const std::vector<Point>& vertices) {
    if (vertices.size() != 4) {
        throw std::invalid_argument("Polygon must have exactly 4 vertices.");
    }
    
    double total_area = 0.0;
    Vector centroid_sum(0, 0, 0);
    
    for (int i = 0; i < 4; ++i) {
        const Point& p0 = vertices[i];
        const Point& p1 = vertices[(i + 1) % 4];
        const Point& p2 = vertices[(i + 2) % 4];
        
        double tri_area = std::abs(p0[0] * (p1[1] - p2[1]) + 
                                  p1[0] * (p2[1] - p0[1]) + 
                                  p2[0] * (p0[1] - p1[1])) / 2.0;
        total_area += tri_area;
        
        Vector tri_centroid((p0[0] + p1[0] + p2[0]) / 3.0,
                          (p0[1] + p1[1] + p2[1]) / 3.0,
                          (p0[2] + p1[2] + p2[2]) / 3.0);
        centroid_sum += tri_centroid * tri_area;
    }
    
    Vector result = centroid_sum / total_area;
    return Point(result[0], result[1], result[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Not class methods
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &os, const Point &point) {
  // Delegate to to_str() for human-readable output
  return os << point.str();
}
} // namespace session_cpp