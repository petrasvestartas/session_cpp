#include "vector.h"
#include "point.h"
#include "vector.pb.h"
#include <algorithm>
#include <limits>

namespace session_cpp {

Vector::Vector(const Vector &other)
    : name(other.name), _x(other._x), _y(other._y), _z(other._z), _magnitude(other._magnitude), _has_magnitude(other._has_magnitude) {}

Vector &Vector::operator=(const Vector &other) {
  if (this == &other)
    return *this;
  _guid.clear();
  name = other.name;
  _x = other._x;
  _y = other._y;
  _z = other._z;
  _magnitude = other._magnitude;
  _has_magnitude = other._has_magnitude;
  return *this;
}

Vector Vector::zero() { return Vector(0.0, 0.0, 0.0); }

Vector Vector::x_axis() { return Vector(1.0, 0.0, 0.0); }

Vector Vector::y_axis() { return Vector(0.0, 1.0, 0.0); }

Vector Vector::z_axis() { return Vector(0.0, 0.0, 1.0); }

Vector Vector::from_points(const Point &p0, const Point &p1) { return Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]); }

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

double &Vector::operator[](int index) {
  _has_magnitude = false;
  if (index == 0)
    return _x;
  if (index == 1)
    return _y;
  if (index == 2)
    return _z;
  throw std::out_of_range("Index out of range");
}

const double &Vector::operator[](int index) const {
  if (index == 0)
    return _x;
  if (index == 1)
    return _y;
  if (index == 2)
    return _z;
  throw std::out_of_range("Index out of range");
}

bool Vector::operator==(const Vector &other) const {
  return name == other.name &&
         std::round(_x * 1000000.0) == std::round(other._x * 1000000.0) &&
         std::round(_y * 1000000.0) == std::round(other._y * 1000000.0) &&
         std::round(_z * 1000000.0) == std::round(other._z * 1000000.0);
}

bool Vector::operator!=(const Vector &other) const { return !(*this == other); }

Vector &Vector::operator*=(double factor) {
  _x *= factor;
  _y *= factor;
  _z *= factor;
  _has_magnitude = false;
  return *this;
}

Vector &Vector::operator/=(double factor) {
  _x /= factor;
  _y /= factor;
  _z /= factor;
  _has_magnitude = false;
  return *this;
}

Vector &Vector::operator+=(const Vector &other) {
  _x += other[0];
  _y += other[1];
  _z += other[2];
  _has_magnitude = false;
  return *this;
}

Vector &Vector::operator-=(const Vector &other) {
  _x -= other[0];
  _y -= other[1];
  _z -= other[2];
  _has_magnitude = false;
  return *this;
}

Vector Vector::operator*(double factor) const { return Vector(_x * factor, _y * factor, _z * factor); }

Vector Vector::operator/(double factor) const { return Vector(_x / factor, _y / factor, _z / factor); }

Vector Vector::operator+(const Vector &other) const { return Vector(_x + other[0], _y + other[1], _z + other[2]); }

Vector Vector::operator-(const Vector &other) const { return Vector(_x - other[0], _y - other[1], _z - other[2]); }

Vector Vector::operator-() const { return Vector(-_x, -_y, -_z); }

Vector operator*(double factor, const Vector &vector) { return vector * factor; }

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

void Vector::transform(const Xform &xform) {
  const double x = _x;
  const double y = _y;
  const double z = _z;
  const std::array<double, 16> &m = xform.m;
  _x = m[0] * x + m[4] * y + m[8] * z;
  _y = m[1] * x + m[5] * y + m[9] * z;
  _z = m[2] * x + m[6] * y + m[10] * z;
  _has_magnitude = false;
}

Vector Vector::transformed(const Xform &xform) const {
  Vector result = *this;
  result.transform(xform);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════

void Vector::reverse() {
  _x = -_x;
  _y = -_y;
  _z = -_z;
}

double Vector::compute_magnitude() const {
  double ax = std::abs(_x);
  double ay = std::abs(_y);
  double az = std::abs(_z);
  const bool x_zero = ax < Tolerance::ZERO_TOLERANCE;
  const bool y_zero = ay < Tolerance::ZERO_TOLERANCE;
  const bool z_zero = az < Tolerance::ZERO_TOLERANCE;
  if (x_zero && y_zero && z_zero)
    return 0.0;
  if (x_zero && y_zero)
    return az;
  if (x_zero && z_zero)
    return ay;
  if (y_zero && z_zero)
    return ax;
  if (ay >= ax && ay >= az)
    std::swap(ax, ay);
  else if (az >= ax && az >= ay)
    std::swap(ax, az);
  if (ax > std::numeric_limits<double>::min()) {
    ay /= ax;
    az /= ax;
    return ax * std::sqrt(1.0 + ay * ay + az * az);
  }
  if (ax > 0.0 && is_finite(ax))
    return ax;
  return 0.0;
}

double Vector::magnitude() const {
  if (!_has_magnitude) {
    _magnitude = compute_magnitude();
    _has_magnitude = true;
  }
  return _magnitude;
}

double Vector::magnitude_squared() const { return _x * _x + _y * _y + _z * _z; }

bool Vector::normalize_self() {
  const double d = compute_magnitude();
  if (d <= 0.0)
    return false;
  _x /= d;
  _y /= d;
  _z /= d;
  _magnitude = 1.0;
  _has_magnitude = true;
  return true;
}

Vector Vector::normalized() const {
  Vector result(_x, _y, _z);
  result.normalize_self();
  return result;
}

double Vector::dot(const Vector &other) const { return _x * other[0] + _y * other[1] + _z * other[2]; }

Vector Vector::cross(const Vector &other) const {
  return Vector(_y * other[2] - _z * other[1], _z * other[0] - _x * other[2], _x * other[1] - _y * other[0]);
}

double Vector::angle(const Vector &other, bool sign_by_cross_product, bool degrees, double tolerance) const {
  const double denominator = magnitude() * other.magnitude();
  if (denominator < tolerance)
    return 0.0;
  const double cos_angle = std::max(-1.0, std::min(1.0, dot(other) / denominator));
  double angle = std::acos(cos_angle);
  if (sign_by_cross_product && cross(other)[2] < 0.0)
    angle = -angle;
  return degrees ? angle * Tolerance::TO_DEGREES : angle;
}

std::tuple<Vector, double, Vector, double> Vector::projection(const Vector &projection_vector, double tolerance) const {
  const double projection_vector_length = projection_vector.magnitude();
  if (projection_vector_length < tolerance)
    return {
        Vector(0.0, 0.0, 0.0),
        0.0,
        Vector(0.0, 0.0, 0.0),
        0.0
    };
  const Vector projection_vector_unit = projection_vector / projection_vector_length;
  const double projected_length = dot(projection_vector_unit);
  const Vector projected = projection_vector_unit * projected_length;
  const Vector perpendicular = *this - projected;
  const double perpendicular_length = perpendicular.magnitude();
  return {projected, projected_length, perpendicular, perpendicular_length};
}

int Vector::is_parallel_to(const Vector &other) const {
  const double cos_tolerance = std::cos(Tolerance::ANGLE_TOLERANCE_DEGREES * Tolerance::TO_RADIANS);
  const double denominator = magnitude() * other.magnitude();
  if (denominator <= 0.0)
    return 0;
  const double cos_angle = dot(other) / denominator;
  if (cos_angle >= cos_tolerance)
    return 1;
  if (cos_angle <= -cos_tolerance)
    return -1;
  return 0;
}

bool Vector::is_perpendicular_to(const Vector &other) const { return std::abs(dot(other)) < Tolerance::ZERO_TOLERANCE; }

bool Vector::perpendicular_to(const Vector &v) {
  int i = 0;
  int j = 1;
  int k = 2;
  double a = v[0];
  double b = -v[1];
  if (std::abs(v[1]) > std::abs(v[0])) {
    if (std::abs(v[2]) > std::abs(v[1])) {
      i = 2;
      j = 1;
      k = 0;
      a = v[2];
      b = -v[1];
    } else if (std::abs(v[2]) >= std::abs(v[0])) {
      i = 1;
      j = 2;
      k = 0;
      a = v[1];
      b = -v[2];
    } else {
      i = 1;
      j = 0;
      k = 2;
      a = v[1];
      b = -v[0];
    }
  } else if (std::abs(v[2]) > std::abs(v[0])) {
    i = 2;
    j = 0;
    k = 1;
    a = v[2];
    b = -v[0];
  } else if (std::abs(v[2]) > std::abs(v[1])) {
    i = 0;
    j = 2;
    k = 1;
    a = v[0];
    b = -v[2];
  }
  double coords[3] = {0.0, 0.0, 0.0};
  coords[i] = b;
  coords[j] = a;
  coords[k] = 0.0;
  _x = coords[0];
  _y = coords[1];
  _z = coords[2];
  _has_magnitude = false;
  return a != 0.0;
}

bool Vector::is_zero() const { return compute_magnitude() < Tolerance::ZERO_TOLERANCE; }

Vector Vector::get_leveled_vector(double vertical_height) const {
  Vector copy(_x, _y, _z);
  if (copy.normalize_self()) {
    const double angle_rad = copy.angle(Vector::z_axis(), false) * Tolerance::TO_RADIANS;
    copy *= vertical_height / std::cos(angle_rad);
  }
  return copy;
}

std::array<double, 3> Vector::coordinate_direction_3angles(bool degrees) const {
  const double r = std::sqrt(_x * _x + _y * _y + _z * _z);
  if (r == 0.0)
    return {0.0, 0.0, 0.0};
  const double alpha = std::acos(_x / r);
  const double beta = std::acos(_y / r);
  const double gamma = std::acos(_z / r);
  if (degrees)
    return {alpha * Tolerance::TO_DEGREES, beta * Tolerance::TO_DEGREES, gamma * Tolerance::TO_DEGREES};
  return {alpha, beta, gamma};
}

std::array<double, 2> Vector::coordinate_direction_2angles(bool degrees) const {
  const double r = std::sqrt(_x * _x + _y * _y + _z * _z);
  if (r == 0.0)
    return {0.0, 0.0};
  const double phi = std::acos(_z / r);
  const double theta = std::atan2(_y, _x);
  if (degrees)
    return {phi * Tolerance::TO_DEGREES, theta * Tolerance::TO_DEGREES};
  return {phi, theta};
}

double Vector::angle_between_vector_xy_components(const Vector &vector) { return std::atan2(vector[1], vector[0]) * Tolerance::TO_DEGREES; }

Vector Vector::sum_of_vectors(const std::vector<Vector> &vectors) {
  Vector sum(0.0, 0.0, 0.0);
  for (const Vector &vector : vectors)
    sum += vector;
  return sum;
}

Vector Vector::average(const std::vector<Vector> &vectors) {
  if (vectors.empty())
    return Vector::zero();
  return sum_of_vectors(vectors) / static_cast<double>(vectors.size());
}

void Vector::scale(double factor) {
  _x *= factor;
  _y *= factor;
  _z *= factor;
  _has_magnitude = false;
}

void Vector::scale_up() { scale(SCALE); }

void Vector::scale_down() { scale(1.0 / SCALE); }

Vector Vector::reflect(const Vector &plane_normal) const {
  const double d = dot(plane_normal);
  return Vector(_x - 2.0 * d * plane_normal[0], _y - 2.0 * d * plane_normal[1], _z - 2.0 * d * plane_normal[2]);
}

Vector Vector::average_normal(const std::vector<Point> &points) {
  if (points.empty())
    return Vector::zero();
  const double dx = points.back()[0] - points.front()[0];
  const double dy = points.back()[1] - points.front()[1];
  const double dz = points.back()[2] - points.front()[2];
  const size_t n = dx * dx + dy * dy + dz * dz < 1e-10 ? points.size() - 1 : points.size();
  Vector normal(0.0, 0.0, 0.0);
  for (size_t i = 0; i < n; i++) {
    const size_t prev = (i + n - 1) % n;
    const size_t next = (i + 1) % n;
    const double ax = points[i][0] - points[prev][0];
    const double ay = points[i][1] - points[prev][1];
    const double az = points[i][2] - points[prev][2];
    const double bx = points[next][0] - points[i][0];
    const double by = points[next][1] - points[i][1];
    const double bz = points[next][2] - points[i][2];
    normal[0] += ay * bz - az * by;
    normal[1] += az * bx - ax * bz;
    normal[2] += ax * by - ay * bx;
  }
  normal.normalize_self();
  return normal;
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangle laws
// ═══════════════════════════════════════════════════════════════════════════

double Vector::cosine_law(double triangle_edge_length_a, double triangle_edge_length_b, double angle_in_between_edges, bool degrees) {
  const double to_radians = degrees ? Tolerance::TO_RADIANS : 1.0;
  return std::sqrt(
      triangle_edge_length_a * triangle_edge_length_a + triangle_edge_length_b * triangle_edge_length_b -
      2.0 * triangle_edge_length_a * triangle_edge_length_b * std::cos(angle_in_between_edges * to_radians)
  );
}

double Vector::sine_law_angle(double triangle_edge_length_a, double angle_in_front_of_a, double triangle_edge_length_b, bool degrees) {
  const double to_radians = degrees ? Tolerance::TO_RADIANS : 1.0;
  const double to_degrees = degrees ? Tolerance::TO_DEGREES : 1.0;
  return std::asin(triangle_edge_length_b * std::sin(angle_in_front_of_a * to_radians) / triangle_edge_length_a) * to_degrees;
}

double Vector::sine_law_length(double triangle_edge_length_a, double angle_in_front_of_a, double angle_in_front_of_b, bool degrees) {
  const double to_radians = degrees ? Tolerance::TO_RADIANS : 1.0;
  return triangle_edge_length_a * std::sin(angle_in_front_of_b * to_radians) / std::sin(angle_in_front_of_a * to_radians);
}

double Vector::angle_from_cosine_law(double triangle_edge_length_a, double triangle_edge_length_b, double triangle_edge_length_c, bool degrees) {
  const double cos_c =
      (triangle_edge_length_a * triangle_edge_length_a + triangle_edge_length_b * triangle_edge_length_b - triangle_edge_length_c * triangle_edge_length_c) /
      (2.0 * triangle_edge_length_a * triangle_edge_length_b);
  const double angle_rad = std::acos(cos_c);
  return degrees ? angle_rad * Tolerance::TO_DEGREES : angle_rad;
}

double Vector::side_from_sine_law(double angle_in_front_of_result_side, double angle_in_front_of_known_side, double known_side_length, bool degrees) {
  const double to_radians = degrees ? Tolerance::TO_RADIANS : 1.0;
  return known_side_length * std::sin(angle_in_front_of_result_side * to_radians) / std::sin(angle_in_front_of_known_side * to_radians);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Vector::jsondump() const {
  nlohmann::ordered_json data;
  data["guid"] = guid();
  data["name"] = name;
  data["type"] = "Vector";
  data["x"] = _x;
  data["y"] = _y;
  data["z"] = _z;
  return data;
}

Vector Vector::jsonload(const nlohmann::json &data) {
  Vector vector(data["x"], data["y"], data["z"]);
  vector.guid() = data["guid"];
  vector.name = data["name"];
  return vector;
}

std::string Vector::file_json_dumps() const { return jsondump().dump(); }

Vector Vector::file_json_loads(const std::string &json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void Vector::file_json_dump(const std::string &filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

Vector Vector::file_json_load(const std::string &filename) {
  std::ifstream file(filename);
  return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

std::string Vector::pb_dumps() const {
  session_proto::Vector proto;
  proto.set_name(name);
  proto.set_x(_x);
  proto.set_y(_y);
  proto.set_z(_z);
  return proto.SerializeAsString();
}

Vector Vector::pb_loads(const std::string &data) {
  session_proto::Vector proto;
  proto.ParseFromString(data);
  Vector vector(proto.x(), proto.y(), proto.z());
  vector.name = proto.name();
  return vector;
}

void Vector::pb_dump(const std::string &filename) const {
  const std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Vector Vector::pb_load(const std::string &filename) {
  std::ifstream file(filename, std::ios::binary);
  const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════

std::string Vector::str() const {
  const int prec = Tolerance::ROUNDING;
  return fmt::format("{}, {}, {}", TOLERANCE.format_number(_x, prec), TOLERANCE.format_number(_y, prec), TOLERANCE.format_number(_z, prec));
}

std::string Vector::repr() const {
  const int prec = Tolerance::ROUNDING;
  return fmt::format(
      "Vector({}, {}, {}, {}, {})",
      name,
      TOLERANCE.format_number(_x, prec),
      TOLERANCE.format_number(_y, prec),
      TOLERANCE.format_number(_z, prec),
      TOLERANCE.format_number(magnitude(), prec)
  );
}

std::ostream &operator<<(std::ostream &os, const Vector &vector) { return os << vector.str(); }

} // namespace session_cpp
