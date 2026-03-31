#include "quaternion.h"
#include "guid.h"
#include <cmath>
#include <fstream>

namespace session_cpp {

static const double QUAT_PI = std::acos(-1.0);

Quaternion::Quaternion() : typ("Quaternion"), name("my_quaternion"), s(1.0), v(0.0, 0.0, 0.0) {}

Quaternion::Quaternion(double s, const Vector& v)
    : typ("Quaternion"), name("my_quaternion"), s(s), v(v) {}

Quaternion Quaternion::identity() {
    return Quaternion(1.0, Vector(0.0, 0.0, 0.0));
}

Quaternion Quaternion::from_sv(double s, const Vector& v) {
    return Quaternion(s, v);
}

Quaternion Quaternion::from_axis_angle(const Vector& axis, double angle) {
    Vector ax = axis.normalized();
    double half = angle * 0.5;
    return Quaternion(std::cos(half), ax * std::sin(half));
}

Quaternion Quaternion::from_arc(const Vector& src, const Vector& dst) {
    Vector s = src.normalized();
    Vector d = dst.normalized();
    Vector cross = s.cross(d);
    double dot_val = s.dot(d);
    if (cross.magnitude() < 1e-10) {
        if (dot_val < 0.0) {
            Vector perp = s.cross(Vector(0.0, 0.0, 1.0));
            if (perp.magnitude() < 1e-10) perp = s.cross(Vector(0.0, 1.0, 0.0));
            return from_axis_angle(perp.normalized(), QUAT_PI);
        }
        return identity();
    }
    return Quaternion(1.0 + dot_val, cross).normalized();
}

Quaternion Quaternion::from_euler(double x, double y, double z) {
    double s1 = std::sin(x * 0.5), c1 = std::cos(x * 0.5);
    double s2 = std::sin(y * 0.5), c2 = std::cos(y * 0.5);
    double s3 = std::sin(z * 0.5), c3 = std::cos(z * 0.5);
    return Quaternion(
        -s1 * s2 * s3 + c1 * c2 * c3,
        Vector(s1 * c2 * c3 + s2 * s3 * c1,
               -s1 * s3 * c2 + s2 * c1 * c3,
                s1 * s2 * c3 + s3 * c1 * c2));
}

Vector Quaternion::rotate_vector(const Vector& vec) const {
    Vector qv = v;
    Vector vec_copy = vec;
    Vector uv = qv.cross(vec_copy);
    Vector uuv = qv.cross(uv);
    return vec_copy + (uv * s + uuv) * 2.0;
}

double Quaternion::magnitude() const {
    return std::sqrt(s * s + v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double Quaternion::magnitude2() const {
    return s * s + v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

Quaternion Quaternion::normalized() const {
    double mag = magnitude();
    if (mag > 1e-10) {
        Quaternion q(s / mag, v / mag);
        q.typ = typ;
        q.guid() = guid();
        q.name = name;
        return q;
    }
    return identity();
}

Quaternion Quaternion::conjugate() const {
    Quaternion q(s, v * -1.0);
    q.typ = typ;
    q.guid() = guid();
    q.name = name;
    return q;
}

Quaternion Quaternion::invert() const {
    double mag2 = magnitude2();
    if (mag2 < 1e-20) return identity();
    Quaternion q(s / mag2, v * (-1.0 / mag2));
    q.typ = typ;
    q.guid() = guid();
    q.name = name;
    return q;
}

double Quaternion::dot(const Quaternion& other) const {
    return s * other.s + v.dot(other.v);
}

Quaternion Quaternion::slerp(const Quaternion& other, double amount) const {
    double dot_val = dot(other);
    if (dot_val > 0.9995) {
        return (*this + (other - *this) * amount).normalized();
    }
    double robust_dot = std::max(-1.0, std::min(1.0, dot_val));
    double theta = std::acos(robust_dot);
    double scale1 = std::sin(theta * (1.0 - amount));
    double scale2 = std::sin(theta * amount);
    double sin_theta = std::sin(theta);
    return (*this * scale1 + other * scale2) * (1.0 / sin_theta);
}

Quaternion Quaternion::nlerp(const Quaternion& other, double amount) const {
    return (*this * (1.0 - amount) + other * amount).normalized();
}

Quaternion Quaternion::operator*(const Quaternion& other) const {
    Vector v_copy = v;
    Vector other_v = other.v;
    double new_s = s * other.s - v_copy.dot(other_v);
    Vector new_v = other_v * s + v_copy * other.s + v_copy.cross(other_v);
    return Quaternion(new_s, new_v);
}

Quaternion Quaternion::operator*(double amount) const {
    return Quaternion(s * amount, v * amount);
}

Quaternion Quaternion::operator+(const Quaternion& other) const {
    return Quaternion(s + other.s, v + other.v);
}

Quaternion Quaternion::operator-(const Quaternion& other) const {
    return Quaternion(s - other.s, v - other.v);
}

Quaternion Quaternion::operator-() const {
    return Quaternion(-s, v * -1.0);
}

nlohmann::ordered_json Quaternion::jsondump() const {
    return nlohmann::ordered_json{
        {"type", typ},
        {"guid", guid()},
        {"name", name},
        {"s", static_cast<double>(s)},
        {"x", static_cast<double>(v[0])},
        {"y", static_cast<double>(v[1])},
        {"z", static_cast<double>(v[2])}
    };
}

Quaternion Quaternion::jsonload(const nlohmann::json& data) {
    Quaternion q(data["s"].get<double>(), Vector(data["x"].get<double>(), data["y"].get<double>(), data["z"].get<double>()));
    q.typ = data.value("type", "Quaternion");
    q.guid() = data["guid"].get<std::string>();
    q.name = data["name"].get<std::string>();
    return q;
}

}  // namespace session_cpp
