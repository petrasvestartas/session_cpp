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

    /// Build a unit quaternion that rotates by `angle` radians around `axis`.
    /// THE everyday rotation builder. Use this whenever you can describe the
    /// rotation as "spin by N radians around this direction" - turning a wheel,
    /// opening a door, orbiting a camera. The result is always unit-length.
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

    /// Build the shortest rotation that maps direction `src` to direction `dst`.
    /// Use this for "look at" logic (point a camera at a target), aligning a
    /// model's forward axis with a desired direction, or snapping one face
    /// normal to another. Both arguments are normalized internally.
    static Quaternion from_arc(const Vector& src, const Vector& dst);

    /// Build a quaternion from three Euler angles (XYZ convention).
    /// Use only at I/O boundaries: importing rotations stored as pitch/yaw/roll
    /// or accepting user input. AVOID for composition - Euler angles suffer
    /// from gimbal lock. Store/compose as quaternions, convert to Euler only
    /// to display or save.
    static Quaternion from_euler(double x, double y, double z);

    /// Build the quaternion that maps the basis of `plane_a` onto the basis of
    /// `plane_b`. Use this to snap one local frame to another - aligning two
    /// CAD parts by their reference planes, transferring a frame between
    /// objects, or computing the relative rotation between two coordinate
    /// systems. (Rhino: Quaternion.Rotation(plane, plane))
    static Quaternion from_rotation(const Plane& plane_a, const Plane& plane_b);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transforms
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply this rotation to a 3D vector and return the rotated vector.
    /// Use this when you have a quaternion orientation and need to know where
    /// a specific direction points after the rotation - the camera's forward
    /// axis, a bone's tip, the normal of a rotated face. Math: q*v_pure*q^-1.
    Vector rotate_vector(const Vector& vec) const;

    /// Apply this rotation to the world XY plane and return the resulting Plane.
    /// Use this to visualize a quaternion as a frame in 3D, or to convert a
    /// stored quaternion into a Plane for frame-based APIs in the rest of the
    /// kernel. Inverse: from_rotation(xy_plane(), result).
    /// (Rhino: Quaternion.GetRotation(out plane))
    Plane get_rotation() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Euclidean norm
    double magnitude() const;

    /// Squared magnitude
    double magnitude_squared() const;

    /// Return a unit-length copy. Use periodically after composing many
    /// rotations - floating-point drift slowly makes a quaternion non-unit,
    /// and a non-unit quaternion no longer represents a valid rotation.
    Quaternion normalized() const;

    /// Flip the sign of the vector part: (s, v) -> (s, -v). For UNIT
    /// quaternions this equals the inverse - the opposite rotation. Use as
    /// the cheap inverse when you KNOW the quaternion is unit-length.
    Quaternion conjugate() const;

    /// True multiplicative inverse: conjugate / magnitude_squared. Works for
    /// non-unit quaternions too. Use as the safe inverse when the quaternion
    /// may not be unit-length. q * q.invert() always equals identity.
    Quaternion invert() const;

    /// Algebraic 4D dot product (NOT a geometric operation). Used inside
    /// slerp implementations and as a similarity measure between two unit
    /// quaternions (1 = same, 0 = 90 deg apart).
    double dot(const Quaternion& other) const;

    /// Spherical Linear intERPolation along the shortest great-circle path
    /// on S^3. Constant angular velocity. Use for high-quality animation
    /// between two orientations - camera transitions, character bones,
    /// anything where smoothness matters more than raw speed.
    Quaternion slerp(const Quaternion& other, double amount) const;

    /// Normalized Linear intERPolation. Cheaper than slerp but the angular
    /// velocity isn't perfectly uniform. Use in real-time loops where every
    /// microsecond matters and the visual difference from slerp is negligible.
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
