#pragma once

#include "vector.h"
#include "json.h"
#include <string>

namespace session_cpp {

/**
 * @class Quaternion
 * @brief A quaternion for 3D rotations (scalar + vector).
 */
class Quaternion {
private:
    std::string typ;
    mutable std::string _guid; ///< Lazily generated unique identifier

public:
    /// Human-readable name
    std::string name;

    /// Lazy GUID accessor (const)
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Lazy GUID accessor (mutable)
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Scalar part
    double s;

    /// Vector part
    Vector v;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor (identity quaternion)
    Quaternion();

    /// Construct from scalar and vector
    Quaternion(double s, const Vector& v);

    /// Type accessor
    const std::string& type() const { return typ; }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Factories
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Identity quaternion (s=1, v=0)
    static Quaternion identity();

    /// Create from scalar and vector components
    static Quaternion from_sv(double s, const Vector& v);

    /// Create from axis of rotation and angle
    static Quaternion from_axis_angle(const Vector& axis, double angle);

    /// Create rotation from source vector to destination vector
    static Quaternion from_arc(const Vector& src, const Vector& dst);

    /// Create from Euler angles (XYZ convention)
    static Quaternion from_euler(double x, double y, double z);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transforms
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Rotate a vector by this quaternion
    Vector rotate_vector(const Vector& vec) const;

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

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to ordered JSON object
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from JSON object
    static Quaternion jsonload(const nlohmann::json& data);
};

}  // namespace session_cpp
