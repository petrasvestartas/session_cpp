#pragma once
#include "guid.h"
#include "json.h"
#include "vector.h"
#include "fmt/core.h"
#include <ostream>
#include <string>
#include <utility>

namespace session_proto {
class Quaternion;
}

namespace session_cpp {

class Plane;

/// A rotation as scalar plus vector part: q = s + xi + yj + zk.
class Quaternion {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_quaternion"; // Quaternion name.
    double scalar = 1.0; // Scalar part s.
    Vector vector; // Vector part (x, y, z).

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the identity rotation.
    Quaternion() {}

    /// Construct from raw components; vector is (i, j, k), not a rotation axis.
    Quaternion(double scalar, const Vector& vector) : scalar(scalar), vector(vector) {}

    /// Copy with a new guid and the same data.
    Quaternion(const Quaternion& other);

    /// Copy-assign with a new guid and the same data.
    Quaternion& operator=(const Quaternion& other);

    /// Move while preserving the guid.
    Quaternion(Quaternion&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Quaternion& operator=(Quaternion&& other) noexcept = default;

    /// Copy with a new guid and the same data.
    Quaternion duplicate() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const;

    /// Return the mutable guid, creating it on first access.
    std::string& guid();

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the rotation that does nothing: scalar 1, vector 0.
    static Quaternion identity();

    /// Construct from raw components; vector is (i, j, k), not a rotation axis.
    static Quaternion from_components(double scalar, const Vector& vector);

    /// Construct the unit quaternion rotating by angle radians around axis.
    static Quaternion from_axis_angle(const Vector& axis, double angle);

    /// Construct the shortest rotation taking direction src to direction dst.
    static Quaternion from_arc(const Vector& src, const Vector& dst);

    /// Construct the rotation from Euler angles in XYZ convention.
    static Quaternion from_euler(double x, double y, double z);

    /// Construct the rotation mapping the frame of plane_a onto the frame of plane_b.
    static Quaternion from_rotation(const Plane& plane_a, const Plane& plane_b);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the mutable component by index (0=scalar, 1=x, 2=y, 3=z).
    double& operator[](int index);

    /// Return the component by index (0=scalar, 1=x, 2=y, 3=z).
    const double& operator[](int index) const;

    /// Compare name and components to six decimals; guid ignored.
    bool operator==(const Quaternion& other) const;

    /// Compare name and components to six decimals; guid ignored.
    bool operator!=(const Quaternion& other) const;

    /// Return the composition: (a * b) applies b first, then a.
    Quaternion operator*(const Quaternion& other) const;

    /// Return a copy scaled by amount.
    Quaternion operator*(double amount) const;

    /// Return the component-wise sum.
    Quaternion operator+(const Quaternion& other) const;

    /// Return the component-wise difference.
    Quaternion operator-(const Quaternion& other) const;

    /// Return the negated copy.
    Quaternion operator-() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the unit axis and angle in radians; (0, 0, 1) and 0 near identity.
    std::pair<Vector, double> to_axis_angle() const;

    /// Return a rotated copy of vec: q * v * q^-1.
    Vector rotate_vector(const Vector& vec) const;

    /// Return the world XY plane rotated by this quaternion.
    Plane get_rotation() const;

    /// Return the 4D length.
    double magnitude() const;

    /// Return the squared magnitude without the square root.
    double magnitude_squared() const;

    /// Return a unit length copy; identity when the magnitude is zero.
    Quaternion normalized() const;

    /// Return (s, -v); the inverse of a unit quaternion.
    Quaternion conjugate() const;

    /// Return the multiplicative inverse: conjugate over squared magnitude.
    Quaternion invert() const;

    /// Return the 4D dot product.
    double dot(const Quaternion& other) const;

    /// Return the spherical interpolation at constant angular velocity.
    Quaternion slerp(const Quaternion& other, double amount) const;

    /// Return the normalized linear interpolation, cheaper than slerp.
    Quaternion nlerp(const Quaternion& other, double amount) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Quaternion jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Quaternion file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Quaternion file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Quaternion to_proto() const;

    /// Construct from the protobuf message.
    static Quaternion from_proto(const session_proto::Quaternion& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Quaternion pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Quaternion pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "s, x, y, z".
    std::string str() const;

    /// Return "Quaternion(name, s, x, y, z)".
    std::string repr() const;
};

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const Quaternion& quaternion);

} // namespace session_cpp
