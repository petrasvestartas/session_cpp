#pragma once

#include "vector.h"
#include "json.h"
#include <string>
#include <stdexcept>
#include <utility>

namespace session_cpp {

class Plane;

/**
 * @class Quaternion
 * @brief A quaternion for 3D rotations (scalar + vector).
 */
class Quaternion {
private:
    std::string typ;
    mutable std::string _guid; ///< Lazily generated unique identifier

    /// Internal constructor from scalar and vector — public API uses from_components
    Quaternion(double scalar, const Vector& vector);

public:
    /// Human-readable name
    std::string name;

    /// Lazy GUID accessor (const)
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Lazy GUID accessor (mutable)
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Scalar part
    double scalar;

    /// Vector part
    Vector vector;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor (identity quaternion)
    Quaternion();

    /// Copy constructor (creates a new guid while copying data)
    Quaternion(const Quaternion& other);

    /// Copy assignment (creates a new guid while copying data)
    Quaternion& operator=(const Quaternion& other);

    /// Type accessor
    const std::string& type() const { return typ; }

    /// Deep copy this quaternion with a new guid
    Quaternion duplicate() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Factories
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Identity quaternion (scalar=1, vector=0)
    static Quaternion identity();

    /**
     * @brief Create a quaternion from raw scalar (real) and vector (imaginary) components.
     *
     * WARNING: The vector argument is NOT a rotation axis. It is the (i, j, k)
     * coefficients of the quaternion. Most users want from_axis_angle() instead.
     *
     * A quaternion is canonically written as:  q = s + xi + yj + zk
     * where s is the scalar (real) part and (x, y, z) is the vector (imaginary) part.
     * Use this constructor only when you have raw quaternion components.
     *
     * To VISUALLY construct a plane from (s, v) values, you have three options:
     *
     *   1. If v should be the plane's NORMAL (the geometric meaning users
     *      usually expect), bypass the quaternion entirely:
     *          Plane p = Plane::from_point_normal(Point(0,0,0), v);
     *
     *   2. If you want the plane produced by the quaternion's rotation
     *      (i.e. the world XY plane rotated by q), normalize first:
     *          Plane p = Quaternion::from_components(s, v).normalized().get_rotation();
     *      The result's normal is the rotation of (0,0,1) by q, which equals v
     *      only in the trivial case where the rotation axis is already Z.
     *
     *   3. If you want a quaternion whose rotation produces a plane with
     *      normal v, use from_arc:
     *          Quaternion q = Quaternion::from_arc(Vector(0,0,1), v.normalized());
     *          Plane p = q.get_rotation();   // p.z_axis() == v.normalized()
     *
     * @param scalar Real part of the quaternion (the 's' in q = s + xi + yj + zk).
     * @param vector Imaginary parts (i, j, k coefficients) — NOT a rotation axis.
     * @return Quaternion with the given raw components (not normalized).
     */
    static Quaternion from_components(double scalar, const Vector& vector);

    /// Create from axis of rotation and angle
    static Quaternion from_axis_angle(const Vector& axis, double angle);

    /**
     * @brief Extract (axis, angle in radians) from this quaternion — the inverse of from_axis_angle.
     *
     * Geometric meaning of a quaternion (s, v):
     *   axis  = v / |v|
     *   angle = 2 * acos(s / |q|)
     *
     * Normalizes internally, so non-unit quaternions are handled correctly.
     *
     * Edge case: for the identity quaternion (or any near-identity) the axis
     * is undefined; this function returns (0, 0, 1) with angle 0.
     *
     * Example:
     *   Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 2.0, 3.0));
     *   auto [axis, angle] = q.to_axis_angle();
     *   // axis = (1,2,3)/sqrt(14), angle ≈ 2.1617 rad ≈ 123.85°
     *   // Reconstruct via geometric form:
     *   Quaternion q2 = Quaternion::from_axis_angle(axis, angle);  // == q.normalized()
     *
     * @return Pair of (unit axis, angle in radians).
     */
    std::pair<Vector, double> to_axis_angle() const;

    /// Create rotation from source vector to destination vector
    static Quaternion from_arc(const Vector& src, const Vector& dst);

    /// Create from Euler angles (XYZ convention)
    static Quaternion from_euler(double x, double y, double z);

    /// Create rotation that maps the basis of plane_a onto the basis of plane_b (Rhino: Quaternion.Rotation(plane, plane))
    static Quaternion from_rotation(const Plane& plane_a, const Plane& plane_b);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transforms
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Rotate a vector by this quaternion
    Vector rotate_vector(const Vector& vec) const;

    /// Apply this quaternion's rotation to the world XY plane and return the resulting plane (Rhino: Quaternion.GetRotation(out plane))
    Plane get_rotation() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Euclidean norm
    double magnitude() const;

    /// Squared magnitude
    double magnitude_squared() const;

    /// Unit quaternion with same direction
    Quaternion normalized() const;

    /// Conjugate (negates vector part)
    Quaternion conjugate() const;

    /// Multiplicative inverse
    Quaternion invert() const;

    /// Dot product with another quaternion
    double dot(const Quaternion& other) const;

    /// Spherical linear interpolation
    Quaternion slerp(const Quaternion& other, double amount) const;

    /// Normalized linear interpolation
    Quaternion nlerp(const Quaternion& other, double amount) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Indexed accessor (0=scalar, 1=vector.x, 2=vector.y, 3=vector.z)
    double& operator[](int index);
    const double& operator[](int index) const;

    /// Equality / inequality
    bool operator==(const Quaternion& other) const;
    bool operator!=(const Quaternion& other) const;

    /// Quaternion multiplication (composition)
    Quaternion operator*(const Quaternion& other) const;

    /// Scalar multiplication
    Quaternion operator*(double amount) const;

    /// Component-wise addition
    Quaternion operator+(const Quaternion& other) const;

    /// Component-wise subtraction
    Quaternion operator-(const Quaternion& other) const;

    /// Negation
    Quaternion operator-() const;

    /// Simple string form (like Python __str__): scalar + vector components
    std::string str() const;

    /// Detailed representation (like Python __repr__)
    std::string repr() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to ordered JSON object
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from JSON object
    static Quaternion jsonload(const nlohmann::json& data);

    /// Convert to JSON string
    std::string json_dumps() const;

    /// Load from JSON string
    static Quaternion json_loads(const std::string& json_string);

    /// Write JSON to file
    void json_dump(const std::string& filename) const;

    /// Read JSON from file
    static Quaternion json_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static Quaternion pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static Quaternion pb_load(const std::string& filename);
};

}  // namespace session_cpp
