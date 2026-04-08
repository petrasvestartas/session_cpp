#pragma once

#include "vector.h"
#include "json.h"
#include <string>
#include <stdexcept>

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

    /// Internal constructor from scalar and vector — public API uses from_scalar_and_vector
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

    /// Create from scalar and vector components
    static Quaternion from_scalar_and_vector(double scalar, const Vector& vector);

    /// Create from axis of rotation and angle
    static Quaternion from_axis_angle(const Vector& axis, double angle);

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
