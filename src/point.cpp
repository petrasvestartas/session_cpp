#include "point.h"
#include "tolerance.h"
#include "point.pb.h"
#include "color.pb.h"
#include <algorithm>
#include <iterator>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Point::Point(const Point& other)
    : _x(other._x), _y(other._y), _z(other._z), name(other.name), width(other.width), pointcolor(other.pointcolor) {}

Point& Point::operator=(const Point& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    _x = other._x;
    _y = other._y;
    _z = other._z;
    name = other.name;
    width = other.width;
    pointcolor = other.pointcolor;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& Point::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& Point::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

void Point::refresh_guid() { _guid.clear(); }

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
double& Point::operator[](int index) {

    if (index == 0)
        return _x;

    if (index == 1)
        return _y;

    if (index == 2)
        return _z;

    throw std::out_of_range("Index out of range");
}

const double& Point::operator[](int index) const {

    if (index == 0)
        return _x;

    if (index == 1)
        return _y;

    if (index == 2)
        return _z;

    throw std::out_of_range("Index out of range");
}

bool Point::operator==(const Point& other) const {

    return name == other.name &&
           std::round(_x * 1000000.0) == std::round(other._x * 1000000.0) &&
           std::round(_y * 1000000.0) == std::round(other._y * 1000000.0) &&
           std::round(_z * 1000000.0) == std::round(other._z * 1000000.0) &&
           std::round(width * 1000000.0) == std::round(other.width * 1000000.0) &&
           pointcolor == other.pointcolor;
}

bool Point::operator!=(const Point& other) const { return !(*this == other); }

Point& Point::operator*=(double factor) {

    _x *= factor;
    _y *= factor;
    _z *= factor;

    return *this;
}

Point& Point::operator/=(double factor) {

    _x /= factor;
    _y /= factor;
    _z /= factor;

    return *this;
}

Point& Point::operator+=(const Vector& other) {

    _x += other[0];
    _y += other[1];
    _z += other[2];

    return *this;
}

Point& Point::operator-=(const Vector& other) {

    _x -= other[0];
    _y -= other[1];
    _z -= other[2];

    return *this;
}

Point Point::operator*(double factor) const { return Point(_x * factor, _y * factor, _z * factor); }

Point Point::operator/(double factor) const { return Point(_x / factor, _y / factor, _z / factor); }

Point Point::operator+(const Vector& other) const { return Point(_x + other[0], _y + other[1], _z + other[2]); }

Point Point::operator-(const Vector& other) const { return Point(_x - other[0], _y - other[1], _z - other[2]); }

Vector Point::operator-(const Point& other) const { return Vector(_x - other._x, _y - other._y, _z - other._z); }

Point Point::sum(const Point& p0, const Point& p1) { return Point(p0[0] + p1[0], p0[1] + p1[1], p0[2] + p1[2]); }

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void Point::transform(const Xform& xform) {

    const double x = _x;
    const double y = _y;
    const double z = _z;
    const std::array<double, 16>& m = xform.m;
    const double w = m[3] * x + m[7] * y + m[11] * z + m[15];
    const double w_inv = std::abs(w) > 1e-10 ? 1.0 / w : 1.0;

    _x = (m[0] * x + m[4] * y + m[8] * z + m[12]) * w_inv;
    _y = (m[1] * x + m[5] * y + m[9] * z + m[13]) * w_inv;
    _z = (m[2] * x + m[6] * y + m[10] * z + m[14]) * w_inv;
}

Point Point::transformed(const Xform& xform) const {

    Point result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
bool Point::is_ccw(const Point& a, const Point& b, const Point& c) {
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]);
}

Point Point::mid_point(const Point& p) const { return Point((_x + p[0]) / 2.0, (_y + p[1]) / 2.0, (_z + p[2]) / 2.0); }

Point Point::mid_point(const Point& a, const Point& b) { return a.mid_point(b); }

double Point::distance(const Point& p, double double_min) const {

    double dx = std::abs(_x - p[0]);
    double dy = std::abs(_y - p[1]);
    double dz = std::abs(_z - p[2]);

    if (dy >= dx && dy >= dz)
        std::swap(dx, dy);
    else if (dz >= dx && dz >= dy)
        std::swap(dx, dz);

    if (dx > double_min) {
        dy /= dx;
        dz /= dx;

        return dx * std::sqrt(1.0 + dy * dy + dz * dz);
    }

    if (dx > 0.0 && std::isfinite(dx))
        return dx;

    return 0.0;
}

double Point::distance(const Point& a, const Point& b, double double_min) { return a.distance(b, double_min); }

double Point::squared_distance(const Point& p, double double_min) const {

    double dx = std::abs(_x - p[0]);
    double dy = std::abs(_y - p[1]);
    double dz = std::abs(_z - p[2]);

    if (dy >= dx && dy >= dz)
        std::swap(dx, dy);
    else if (dz >= dx && dz >= dy)
        std::swap(dx, dz);

    if (dx > double_min) {
        dy /= dx;
        dz /= dx;

        return dx * dx * (1.0 + dy * dy + dz * dz);
    }

    if (dx > 0.0 && std::isfinite(dx))
        return dx * dx;

    return 0.0;
}

double Point::squared_distance(const Point& a, const Point& b, double double_min) {
    return a.squared_distance(b, double_min);
}

Point Point::lerp(const Point& a, const Point& b, double t) {
    return a + (b - a) * t;
}

std::vector<Point> Point::interpolate(const Point& from, const Point& to, int steps, int kind) {

    std::vector<Point> points;

    if (kind == 1 || kind == 2)
        points.push_back(from);

    for (int i = 1; i <= steps; ++i)
        points.push_back(lerp(from, to, static_cast<double>(i) / static_cast<double>(steps + 1)));

    if (kind == 1)
        points.push_back(to);

    return points;
}

double Point::area(const std::vector<Point>& points) {

    const size_t n = points.size();
    double area = 0.0;

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        area += points[i][0] * points[j][1];
        area -= points[j][0] * points[i][1];
    }

    return std::abs(area) / 2.0;
}

Point Point::centroid_quad(const std::vector<Point>& vertices) {

    if (vertices.size() != 4)
        throw std::invalid_argument("Polygon must have exactly 4 vertices.");

    double total_area = 0.0;
    Vector centroid_sum(0.0, 0.0, 0.0);

    for (int i = 0; i < 4; ++i) {
        const Point& p0 = vertices[i];
        const Point& p1 = vertices[(i + 1) % 4];
        const Point& p2 = vertices[(i + 2) % 4];
        const double tri_area = std::abs(p0[0] * (p1[1] - p2[1]) + p1[0] * (p2[1] - p0[1]) + p2[0] * (p0[1] - p1[1])) / 2.0;
        const Vector tri_centroid((p0[0] + p1[0] + p2[0]) / 3.0, (p0[1] + p1[1] + p2[1]) / 3.0, (p0[2] + p1[2] + p2[2]) / 3.0);

        total_area += tri_area;
        centroid_sum += tri_centroid * tri_area;
    }

    const Vector result = centroid_sum / total_area;

    return Point(result[0], result[1], result[2]);
}

Point Point::centroid(const std::vector<Point>& points) {

    if (points.empty())
        return Point(0.0, 0.0, 0.0);

    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;

    for (const Point& p : points) {
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }

    const double n = static_cast<double>(points.size());

    return Point(cx / n, cy / n, cz / n);
}

double Point::dihedral_angle_deg(const Point& p, const Point& q, const Point& r, const Point& s) {

    const Vector pq = q - p;
    const Vector pr = r - p;
    const Vector ps = s - p;
    const Vector n1 = pq.cross(pr);
    const Vector n2 = pq.cross(ps);
    const double m1 = n1.magnitude();
    const double m2 = n2.magnitude();

    if (m1 < Tolerance::ZERO_TOLERANCE || m2 < Tolerance::ZERO_TOLERANCE)
        return 0.0;

    const double cos_t = std::clamp(n1.dot(n2) / (m1 * m2), -1.0, 1.0);

    return std::acos(cos_t) * (180.0 / 3.141592653589793);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Point::jsondump() const {

    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["name"] = name;
    data["pointcolor"] = pointcolor.jsondump();
    data["type"] = "Point";
    data["width"] = width;
    data["x"] = _x;
    data["y"] = _y;
    data["z"] = _z;

    return data;
}

Point Point::jsonload(const nlohmann::json& data) {

    Point point(data["x"], data["y"], data["z"]);
    point.guid() = data["guid"];
    point.name = data["name"];
    point.pointcolor = Color::jsonload(data["pointcolor"]);
    point.width = data["width"];

    return point;
}

std::string Point::file_json_dumps() const { return jsondump().dump(); }

Point Point::file_json_loads(const std::string& json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void Point::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Point Point::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Point Point::to_proto() const {

    session_proto::Point proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_x(_x);
    proto.set_y(_y);
    proto.set_z(_z);
    proto.set_width(width);
    *proto.mutable_pointcolor() = pointcolor.to_proto();

    return proto;
}

Point Point::from_proto(const session_proto::Point& proto) {

    Point point(proto.x(), proto.y(), proto.z());

    if (!proto.guid().empty())
        point.guid() = proto.guid();

    point.name = proto.name();
    point.width = proto.width();
    point.pointcolor = Color::from_proto(proto.pointcolor());

    return point;
}

std::string Point::pb_dumps() const { return to_proto().SerializeAsString(); }

Point Point::pb_loads(const std::string& data) {

    session_proto::Point proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Point protobuf data");

    return from_proto(proto);
}

void Point::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Point Point::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Point::str() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format("{}, {}, {}", TOLERANCE.format_number(_x, prec), TOLERANCE.format_number(_y, prec), TOLERANCE.format_number(_z, prec));
}

std::string Point::repr() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "Point({}, {}, {}, {}, {}, {})",
        name,
        TOLERANCE.format_number(_x, prec),
        TOLERANCE.format_number(_y, prec),
        TOLERANCE.format_number(_z, prec),
        pointcolor.repr(),
        TOLERANCE.format_number(width, prec)
    );
}

std::ostream& operator<<(std::ostream& os, const Point& point) { return os << point.str(); }

} // namespace session_cpp
