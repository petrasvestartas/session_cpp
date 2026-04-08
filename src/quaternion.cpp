#include "quaternion.h"
#include "guid.h"
#include "plane.h"
#include "point.h"
#include "tolerance.h"
#include "fmt/core.h"
#include "quaternion.pb.h"
#include <cmath>
#include <fstream>

namespace session_cpp {

Quaternion::Quaternion() : typ("Quaternion"), name("my_quaternion"), scalar(1.0), vector(0.0, 0.0, 0.0) {}

Quaternion::Quaternion(double scalar, const Vector& vector)
    : typ("Quaternion"), name("my_quaternion"), scalar(scalar), vector(vector) {}

Quaternion::Quaternion(const Quaternion& other)
    : typ(other.typ), name(other.name), scalar(other.scalar), vector(other.vector) {}

Quaternion& Quaternion::operator=(const Quaternion& other) {
    if (this != &other) {
        _guid.clear();
        typ = other.typ;
        name = other.name;
        scalar = other.scalar;
        vector = other.vector;
    }
    return *this;
}

Quaternion Quaternion::duplicate() const {
    Quaternion q(*this);
    q._guid = ::guid();
    return q;
}

Quaternion Quaternion::identity() {
    return Quaternion(1.0, Vector(0.0, 0.0, 0.0));
}

Quaternion Quaternion::from_scalar_and_vector(double scalar, const Vector& vector) {
    return Quaternion(scalar, vector);
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
            return from_axis_angle(perp.normalized(), Tolerance::PI);
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

Quaternion Quaternion::from_rotation(const Plane& plane_a, const Plane& plane_b) {
    const Vector& xa = plane_a.x_axis(); const Vector& ya = plane_a.y_axis(); const Vector& za = plane_a.z_axis();
    const Vector& xb = plane_b.x_axis(); const Vector& yb = plane_b.y_axis(); const Vector& zb = plane_b.z_axis();
    double m[3][3];
    m[0][0] = xb[0]*xa[0] + yb[0]*ya[0] + zb[0]*za[0];
    m[0][1] = xb[0]*xa[1] + yb[0]*ya[1] + zb[0]*za[1];
    m[0][2] = xb[0]*xa[2] + yb[0]*ya[2] + zb[0]*za[2];
    m[1][0] = xb[1]*xa[0] + yb[1]*ya[0] + zb[1]*za[0];
    m[1][1] = xb[1]*xa[1] + yb[1]*ya[1] + zb[1]*za[1];
    m[1][2] = xb[1]*xa[2] + yb[1]*ya[2] + zb[1]*za[2];
    m[2][0] = xb[2]*xa[0] + yb[2]*ya[0] + zb[2]*za[0];
    m[2][1] = xb[2]*xa[1] + yb[2]*ya[1] + zb[2]*za[1];
    m[2][2] = xb[2]*xa[2] + yb[2]*ya[2] + zb[2]*za[2];
    bool is_identity = true;
    const double eps = 1.490116119385e-8;
    for (int i = 0; i < 3 && is_identity; i++) {
        for (int j = 0; j < 3; j++) {
            double d = (i == j) ? std::fabs(m[i][i] - 1.0) : std::fabs(m[i][j]);
            if (d > eps) { is_identity = false; break; }
        }
    }
    if (is_identity) return Quaternion(1.0, Vector(0.0, 0.0, 0.0));
    int i = (m[0][0] >= m[1][1]) ? ((m[0][0] >= m[2][2]) ? 0 : 2) : ((m[1][1] >= m[2][2]) ? 1 : 2);
    int j = (i + 1) % 3;
    int k = (i + 2) % 3;
    double s = 1.0 + m[i][i] - m[j][j] - m[k][k];
    if (s <= 0.0) return Quaternion(1.0, Vector(0.0, 0.0, 0.0));
    double r = std::sqrt(s);
    s = 0.5 / r;
    double q[3];
    q[i] = 0.5 * r;
    q[j] = s * (m[i][j] + m[j][i]);
    q[k] = s * (m[k][i] + m[i][k]);
    return Quaternion(s * (m[k][j] - m[j][k]), Vector(q[0], q[1], q[2]));
}

Vector Quaternion::rotate_vector(const Vector& vec) const {
    Vector qv = vector;
    Vector vec_copy = vec;
    Vector uv = qv.cross(vec_copy);
    Vector uuv = qv.cross(uv);
    return vec_copy + (uv * scalar + uuv) * 2.0;
}

Plane Quaternion::get_rotation() const {
    double a = scalar, b = vector[0], c = vector[1], d = vector[2];
    Vector xaxis(a*a + b*b - c*c - d*d, 2.0*(a*d + b*c),       2.0*(b*d - a*c));
    Vector yaxis(2.0*(b*c - a*d),       a*a - b*b + c*c - d*d, 2.0*(a*b + c*d));
    Vector zaxis(2.0*(a*c + b*d),       2.0*(c*d - a*b),       a*a - b*b - c*c + d*d);
    return Plane(Point(0.0, 0.0, 0.0), xaxis, yaxis, zaxis);
}

double Quaternion::magnitude() const {
    return std::sqrt(scalar * scalar + vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

double Quaternion::magnitude_squared() const {
    return scalar * scalar + vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
}

Quaternion Quaternion::normalized() const {
    double mag = magnitude();
    if (mag > 1e-10) {
        Quaternion q(scalar / mag, vector / mag);
        q.typ = typ;
        q.guid() = guid();
        q.name = name;
        return q;
    }
    return identity();
}

Quaternion Quaternion::conjugate() const {
    Quaternion q(scalar, vector * -1.0);
    q.typ = typ;
    q.guid() = guid();
    q.name = name;
    return q;
}

Quaternion Quaternion::invert() const {
    double mag2 = magnitude_squared();
    if (mag2 < 1e-20) return identity();
    Quaternion q(scalar / mag2, vector * (-1.0 / mag2));
    q.typ = typ;
    q.guid() = guid();
    q.name = name;
    return q;
}

double Quaternion::dot(const Quaternion& other) const {
    return scalar * other.scalar + vector.dot(other.vector);
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

double& Quaternion::operator[](int index) {
    if (index == 0) return scalar;
    else if (index == 1) return vector[0];
    else if (index == 2) return vector[1];
    else if (index == 3) return vector[2];
    else throw std::out_of_range("Index out of range");
}

const double& Quaternion::operator[](int index) const {
    if (index == 0) return scalar;
    else if (index == 1) return vector[0];
    else if (index == 2) return vector[1];
    else if (index == 3) return vector[2];
    else throw std::out_of_range("Index out of range");
}

bool Quaternion::operator==(const Quaternion& other) const {
    return scalar == other.scalar && vector[0] == other.vector[0] && vector[1] == other.vector[1] && vector[2] == other.vector[2];
}

bool Quaternion::operator!=(const Quaternion& other) const { return !(*this == other); }

Quaternion Quaternion::operator*(const Quaternion& other) const {
    Vector v_copy = vector;
    Vector other_v = other.vector;
    double new_s = scalar * other.scalar - v_copy.dot(other_v);
    Vector new_v = other_v * scalar + v_copy * other.scalar + v_copy.cross(other_v);
    return Quaternion(new_s, new_v);
}

Quaternion Quaternion::operator*(double amount) const {
    return Quaternion(scalar * amount, vector * amount);
}

Quaternion Quaternion::operator+(const Quaternion& other) const {
    return Quaternion(scalar + other.scalar, vector + other.vector);
}

Quaternion Quaternion::operator-(const Quaternion& other) const {
    return Quaternion(scalar - other.scalar, vector - other.vector);
}

Quaternion Quaternion::operator-() const {
    return Quaternion(-scalar, vector * -1.0);
}

std::string Quaternion::str() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format(
        "{}, {}, {}, {}",
        TOLERANCE.format_number(scalar, prec),
        TOLERANCE.format_number(vector[0], prec),
        TOLERANCE.format_number(vector[1], prec),
        TOLERANCE.format_number(vector[2], prec));
}

std::string Quaternion::repr() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format(
        "Quaternion({}, {}, {}, {}, {})",
        name,
        TOLERANCE.format_number(scalar, prec),
        TOLERANCE.format_number(vector[0], prec),
        TOLERANCE.format_number(vector[1], prec),
        TOLERANCE.format_number(vector[2], prec));
}

nlohmann::ordered_json Quaternion::jsondump() const {
    return nlohmann::ordered_json{
        {"type", typ},
        {"guid", guid()},
        {"name", name},
        {"s", static_cast<double>(scalar)},
        {"x", static_cast<double>(vector[0])},
        {"y", static_cast<double>(vector[1])},
        {"z", static_cast<double>(vector[2])}
    };
}

Quaternion Quaternion::jsonload(const nlohmann::json& data) {
    Quaternion q(data["s"].get<double>(), Vector(data["x"].get<double>(), data["y"].get<double>(), data["z"].get<double>()));
    q.typ = data.value("type", "Quaternion");
    q.guid() = data["guid"].get<std::string>();
    q.name = data["name"].get<std::string>();
    return q;
}

std::string Quaternion::json_dumps() const {
    return jsondump().dump();
}

Quaternion Quaternion::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Quaternion::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Quaternion Quaternion::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data = nlohmann::json::parse(file);
    return jsonload(data);
}

std::string Quaternion::pb_dumps() const {
    session_proto::Quaternion proto;
    proto.set_a(scalar);
    proto.set_b(vector[0]);
    proto.set_c(vector[1]);
    proto.set_d(vector[2]);
    proto.set_name(name);
    return proto.SerializeAsString();
}

Quaternion Quaternion::pb_loads(const std::string& data) {
    session_proto::Quaternion proto;
    proto.ParseFromString(data);
    Quaternion q(proto.a(), Vector(proto.b(), proto.c(), proto.d()));
    q.name = proto.name();
    return q;
}

void Quaternion::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Quaternion Quaternion::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

}  // namespace session_cpp
