#pragma once

#include "vector.h"
#include "json.h"
#include <string>

namespace session_cpp {

class Quaternion {
private:
    std::string typ;
    mutable std::string _guid; ///< Lazily generated unique identifier

public:
    std::string name;
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    double s;
    Vector v;

    Quaternion();
    Quaternion(double s, const Vector& v);

    const std::string& type() const { return typ; }

    static Quaternion identity();
    static Quaternion from_sv(double s, const Vector& v);
    static Quaternion from_axis_angle(const Vector& axis, double angle);
    static Quaternion from_arc(const Vector& src, const Vector& dst);
    static Quaternion from_euler(double x, double y, double z);

    Vector rotate_vector(const Vector& vec) const;
    double magnitude() const;
    double magnitude2() const;
    Quaternion normalize() const;
    Quaternion conjugate() const;
    Quaternion invert() const;
    double dot(const Quaternion& other) const;
    Quaternion slerp(const Quaternion& other, double amount) const;
    Quaternion nlerp(const Quaternion& other, double amount) const;

    Quaternion operator*(const Quaternion& other) const;
    Quaternion operator*(double amount) const;
    Quaternion operator+(const Quaternion& other) const;
    Quaternion operator-(const Quaternion& other) const;
    Quaternion operator-() const;

    nlohmann::ordered_json jsondump() const;
    static Quaternion jsonload(const nlohmann::json& data);
};

}  // namespace session_cpp
