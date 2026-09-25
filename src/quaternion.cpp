#include "quaternion.h"
#include "plane.h"
#include "point.h"
#include "tolerance.h"
#include "quaternion.pb.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Quaternion::Quaternion(const Quaternion& other) : name(other.name), scalar(other.scalar), vector(other.vector) {}

Quaternion& Quaternion::operator=(const Quaternion& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    name = other.name;
    scalar = other.scalar;
    vector = other.vector;

    return *this;
}

Quaternion Quaternion::duplicate() const {
    return Quaternion(*this);
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& Quaternion::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& Quaternion::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

void Quaternion::refresh_guid() {
    _guid.clear();
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
Quaternion Quaternion::identity() {
    return Quaternion(1.0, Vector(0.0, 0.0, 0.0));
}

Quaternion Quaternion::from_components(double scalar, const Vector& vector) {
    return Quaternion(scalar, vector);
}

Quaternion Quaternion::from_axis_angle(const Vector& axis, double angle) {

    if (axis.magnitude() < 1e-10)
        return identity();

    const Vector ax = axis.normalized();
    const double half = angle * 0.5;

    return Quaternion(std::cos(half), ax * std::sin(half));
}

Quaternion Quaternion::from_arc(const Vector& src, const Vector& dst) {

    const Vector s = src.normalized();
    const Vector d = dst.normalized();
    const Vector cross = s.cross(d);
    const double dot_val = s.dot(d);

    if (cross.magnitude() < 1e-10) {
        if (dot_val < 0.0) {
            Vector perp = s.cross(Vector(0.0, 0.0, 1.0));

            if (perp.magnitude() < 1e-10)
                perp = s.cross(Vector(0.0, 1.0, 0.0));

            return from_axis_angle(perp.normalized(), Tolerance::PI);
        }

        return identity();
    }

    return Quaternion(1.0 + dot_val, cross).normalized();
}

Quaternion Quaternion::from_euler(double x, double y, double z) {

    const double s1 = std::sin(x * 0.5);
    const double c1 = std::cos(x * 0.5);
    const double s2 = std::sin(y * 0.5);
    const double c2 = std::cos(y * 0.5);
    const double s3 = std::sin(z * 0.5);
    const double c3 = std::cos(z * 0.5);

    return Quaternion(
        -s1 * s2 * s3 + c1 * c2 * c3,
        Vector(s1 * c2 * c3 + s2 * s3 * c1, -s1 * s3 * c2 + s2 * c1 * c3, s1 * s2 * c3 + s3 * c1 * c2)
    );
}

Quaternion Quaternion::from_rotation(const Plane& plane_a, const Plane& plane_b) {

    const Vector& xa = plane_a.x_axis();
    const Vector& ya = plane_a.y_axis();
    const Vector& za = plane_a.z_axis();
    const Vector& xb = plane_b.x_axis();
    const Vector& yb = plane_b.y_axis();
    const Vector& zb = plane_b.z_axis();
    double m[3][3];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m[i][j] = xb[i] * xa[j] + yb[i] * ya[j] + zb[i] * za[j];

    const double eps = 1.490116119385e-8;
    bool is_identity = true;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (std::fabs(m[i][j] - (i == j ? 1.0 : 0.0)) > eps)
                is_identity = false;

    if (is_identity)
        return identity();

    int i = 2;

    if (m[0][0] >= m[1][1] && m[0][0] >= m[2][2])
        i = 0;
    else if (m[1][1] >= m[0][0] && m[1][1] >= m[2][2])
        i = 1;

    const int j = (i + 1) % 3;
    const int k = (i + 2) % 3;
    double s = 1.0 + m[i][i] - m[j][j] - m[k][k];

    if (s <= 0.0)
        return identity();

    const double r = std::sqrt(s);
    s = 0.5 / r;

    double q[3];
    q[i] = 0.5 * r;
    q[j] = s * (m[i][j] + m[j][i]);
    q[k] = s * (m[k][i] + m[i][k]);

    return Quaternion(s * (m[k][j] - m[j][k]), Vector(q[0], q[1], q[2]));
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
double& Quaternion::operator[](int index) {

    if (index == 0)
        return scalar;

    if (index >= 1 && index <= 3)
        return vector[index - 1];

    throw std::out_of_range("Index out of range");
}

const double& Quaternion::operator[](int index) const {

    if (index == 0)
        return scalar;

    if (index >= 1 && index <= 3)
        return vector[index - 1];

    throw std::out_of_range("Index out of range");
}

bool Quaternion::operator==(const Quaternion& other) const {

    return name == other.name &&
           std::round(scalar * 1000000.0) == std::round(other.scalar * 1000000.0) &&
           std::round(vector[0] * 1000000.0) == std::round(other.vector[0] * 1000000.0) &&
           std::round(vector[1] * 1000000.0) == std::round(other.vector[1] * 1000000.0) &&
           std::round(vector[2] * 1000000.0) == std::round(other.vector[2] * 1000000.0);
}

bool Quaternion::operator!=(const Quaternion& other) const {
    return !(*this == other);
}

Quaternion Quaternion::operator*(const Quaternion& other) const {

    return Quaternion(
        scalar * other.scalar - vector.dot(other.vector),
        other.vector * scalar + vector * other.scalar + vector.cross(other.vector)
    );
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
    return Quaternion(-scalar, -vector);
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
std::pair<Vector, double> Quaternion::to_axis_angle() const {

    const Quaternion qn = normalized();
    const double s = std::clamp(qn.scalar, -1.0, 1.0);
    const double angle = 2.0 * std::acos(s);
    const double sin_half = std::sqrt(1.0 - s * s);

    if (sin_half < 1e-12)
        return {Vector(0.0, 0.0, 1.0), 0.0};

    return {qn.vector / sin_half, angle};
}

Vector Quaternion::rotate_vector(const Vector& vec) const {

    const Vector uv = vector.cross(vec);
    const Vector uuv = vector.cross(uv);

    return vec + (uv * scalar + uuv) * 2.0;
}

Plane Quaternion::get_rotation() const {

    const double a = scalar;
    const double b = vector[0];
    const double c = vector[1];
    const double d = vector[2];
    const Vector xaxis(a * a + b * b - c * c - d * d, 2.0 * (a * d + b * c), 2.0 * (b * d - a * c));
    const Vector yaxis(2.0 * (b * c - a * d), a * a - b * b + c * c - d * d, 2.0 * (a * b + c * d));
    const Vector zaxis(2.0 * (a * c + b * d), 2.0 * (c * d - a * b), a * a - b * b - c * c + d * d);

    return Plane::from_frame(Point(0.0, 0.0, 0.0), xaxis, yaxis, zaxis);
}

double Quaternion::magnitude() const {
    return std::sqrt(magnitude_squared());
}

double Quaternion::magnitude_squared() const {
    return scalar * scalar + vector.dot(vector);
}

Quaternion Quaternion::normalized() const {

    const double mag = magnitude();

    if (mag < 1e-10)
        return identity();

    Quaternion q(scalar / mag, vector / mag);
    q.name = name;

    return q;
}

Quaternion Quaternion::conjugate() const {

    Quaternion q(scalar, -vector);
    q.name = name;

    return q;
}

Quaternion Quaternion::invert() const {

    const double mag2 = magnitude_squared();

    if (mag2 < 1e-20)
        return identity();

    Quaternion q(scalar / mag2, vector * (-1.0 / mag2));
    q.name = name;

    return q;
}

double Quaternion::dot(const Quaternion& other) const {
    return scalar * other.scalar + vector.dot(other.vector);
}

Quaternion Quaternion::slerp(const Quaternion& other, double amount) const {

    Quaternion target = other;
    double dot_val = dot(target);

    if (dot_val < 0.0) {
        target = -target;
        dot_val = -dot_val;
    }

    if (dot_val > 0.9995)
        return (*this + (target - *this) * amount).normalized();

    const double theta = std::acos(std::clamp(dot_val, -1.0, 1.0));
    const double scale1 = std::sin(theta * (1.0 - amount));
    const double scale2 = std::sin(theta * amount);

    return (*this * scale1 + target * scale2) * (1.0 / std::sin(theta));
}

Quaternion Quaternion::nlerp(const Quaternion& other, double amount) const {

    return (*this * (1.0 - amount) + other * amount).normalized();
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Quaternion::jsondump() const {

    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["name"] = name;
    data["s"] = scalar;
    data["type"] = "Quaternion";
    data["x"] = vector[0];
    data["y"] = vector[1];
    data["z"] = vector[2];

    return data;
}

Quaternion Quaternion::jsonload(const nlohmann::json& data) {

    Quaternion q(data.at("s"), Vector(data.at("x"), data.at("y"), data.at("z")));
    q.guid() = data.at("guid");
    q.name = data.at("name");

    return q;
}

std::string Quaternion::file_json_dumps() const {
    return jsondump().dump();
}

Quaternion Quaternion::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Quaternion::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Quaternion Quaternion::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Quaternion Quaternion::to_proto() const {

    session_proto::Quaternion proto;
    proto.set_a(scalar);
    proto.set_b(vector[0]);
    proto.set_c(vector[1]);
    proto.set_d(vector[2]);
    proto.set_name(name);

    return proto;
}

Quaternion Quaternion::from_proto(const session_proto::Quaternion& proto) {

    Quaternion q(proto.a(), Vector(proto.b(), proto.c(), proto.d()));
    q.name = proto.name();

    return q;
}

std::string Quaternion::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Quaternion Quaternion::pb_loads(const std::string& data) {

    session_proto::Quaternion proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Quaternion protobuf data");

    return from_proto(proto);
}

void Quaternion::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Quaternion Quaternion::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Quaternion::str() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "{}, {}, {}, {}",
        TOLERANCE.format_number(scalar, prec),
        TOLERANCE.format_number(vector[0], prec),
        TOLERANCE.format_number(vector[1], prec),
        TOLERANCE.format_number(vector[2], prec)
    );
}

std::string Quaternion::repr() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "Quaternion({}, {}, {}, {}, {})",
        name,
        TOLERANCE.format_number(scalar, prec),
        TOLERANCE.format_number(vector[0], prec),
        TOLERANCE.format_number(vector[1], prec),
        TOLERANCE.format_number(vector[2], prec)
    );
}

std::ostream& operator<<(std::ostream& os, const Quaternion& quaternion) {
    return os << quaternion.str();
}

} // namespace session_cpp
