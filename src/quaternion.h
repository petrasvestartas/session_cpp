#pragma once
#include "guid.h"
#include "json.h"
#include "vector.h"
#include "fmt/core.h"
#include <ostream>
#include <string>
#include <utility>

namespace session_cpp {

class Plane;

/// A rotation as scalar plus vector part: q = s + xi + yj + zk
class Quaternion {
public:
  std::string name = "my_quaternion";
  double scalar = 1.0;
  Vector vector;

  Quaternion() {}

  /// Raw components; vector is (i, j, k), not a rotation axis
  Quaternion(double scalar, const Vector &vector) : scalar(scalar), vector(vector) {}

  /// Copy constructor (new guid, same data)
  Quaternion(const Quaternion &other);

  /// Copy assignment (new guid, same data)
  Quaternion &operator=(const Quaternion &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  Quaternion(Quaternion &&other) noexcept = default;
  Quaternion &operator=(Quaternion &&other) noexcept = default;

  /// Copy (new guid, same data)
  Quaternion duplicate() const;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// The rotation that does nothing: scalar 1, vector 0
  static Quaternion identity();

  /// Raw components; vector is (i, j, k), not a rotation axis
  static Quaternion from_components(double scalar, const Vector &vector);

  /// Unit quaternion rotating by angle radians around axis
  static Quaternion from_axis_angle(const Vector &axis, double angle);

  /// Shortest rotation taking direction src to direction dst
  static Quaternion from_arc(const Vector &src, const Vector &dst);

  /// Rotation from Euler angles in XYZ convention
  static Quaternion from_euler(double x, double y, double z);

  /// Rotation mapping the frame of plane_a onto the frame of plane_b
  static Quaternion from_rotation(const Plane &plane_a, const Plane &plane_b);

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Component by index (0=scalar, 1=x, 2=y, 3=z)
  double &operator[](int index);
  const double &operator[](int index) const;

  bool operator==(const Quaternion &other) const;
  bool operator!=(const Quaternion &other) const;

  /// Composition: (a * b) applies b first, then a
  Quaternion operator*(const Quaternion &other) const;
  Quaternion operator*(double amount) const;
  Quaternion operator+(const Quaternion &other) const;
  Quaternion operator-(const Quaternion &other) const;
  Quaternion operator-() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// Unit axis and angle in radians; (0, 0, 1) and 0 near identity
  std::pair<Vector, double> to_axis_angle() const;

  /// Rotated copy of vec: q * v * q^-1
  Vector rotate_vector(const Vector &vec) const;

  /// World XY plane rotated by this quaternion
  Plane get_rotation() const;

  /// 4D length
  double magnitude() const;

  /// Squared magnitude without the square root
  double magnitude_squared() const;

  /// Unit length copy; identity when the magnitude is zero
  Quaternion normalized() const;

  /// (s, -v); the inverse of a unit quaternion
  Quaternion conjugate() const;

  /// Multiplicative inverse: conjugate over squared magnitude
  Quaternion invert() const;

  /// 4D dot product
  double dot(const Quaternion &other) const;

  /// Spherical interpolation at constant angular velocity
  Quaternion slerp(const Quaternion &other, double amount) const;

  /// Normalized linear interpolation, cheaper than slerp
  Quaternion nlerp(const Quaternion &other, double amount) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Quaternion jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Quaternion file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static Quaternion file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static Quaternion pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static Quaternion pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "s, x, y, z"
  std::string str() const;

  /// "Quaternion(name, s, x, y, z)"
  std::string repr() const;

private:
  mutable std::string _guid;
};

std::ostream &operator<<(std::ostream &os, const Quaternion &q);

} // namespace session_cpp
