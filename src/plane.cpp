#include "plane.h"
#include "polyline.h"
#include "tolerance.h"
#include "plane.pb.h"
#include "color.pb.h"
#include <iterator>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Plane::Plane() {}

Plane::Plane(const Point& point, const Vector& x_axis, const Vector& y_axis, std::string name)
    : _origin(point), _x_axis(x_axis), name(std::move(name)) {

    _x_axis.normalize_self();
    _y_axis = y_axis - _x_axis * y_axis.dot(_x_axis);
    _y_axis.normalize_self();
    _z_axis = _x_axis.cross(_y_axis);
    _z_axis.normalize_self();
    update_equation();
}

Plane::Plane(const Plane& other)
    : _origin(other._origin), _x_axis(other._x_axis), _y_axis(other._y_axis), _z_axis(other._z_axis), _a(other._a), _b(other._b), _c(other._c), _d(other._d), name(other.name), width(other.width), linecolor(other.linecolor) {}

Plane& Plane::operator=(const Plane& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    _origin = other._origin;
    _x_axis = other._x_axis;
    _y_axis = other._y_axis;
    _z_axis = other._z_axis;
    _a = other._a;
    _b = other._b;
    _c = other._c;
    _d = other._d;
    name = other.name;
    width = other.width;
    linecolor = other.linecolor;

    return *this;
}

void Plane::update_equation() {

    _a = _z_axis[0];
    _b = _z_axis[1];
    _c = _z_axis[2];
    _d = -(_a * _origin[0] + _b * _origin[1] + _c * _origin[2]);
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& Plane::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& Plane::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

void Plane::refresh_guid() {
    _guid.clear();
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
Plane Plane::from_frame(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis) {

    Plane plane;
    plane._origin = origin;
    plane._x_axis = x_axis;
    plane._y_axis = y_axis;
    plane._z_axis = z_axis;
    plane.update_equation();

    return plane;
}

Plane Plane::from_point_normal(const Point& point, const Vector& normal, bool normalize) {

    Vector z_axis = normal;

    if (normalize)
        z_axis.normalize_self();

    Vector x_axis;
    x_axis.perpendicular_to(z_axis);

    if (normalize)
        x_axis.normalize_self();

    Vector y_axis = z_axis.cross(x_axis);

    if (normalize)
        y_axis.normalize_self();

    return from_frame(point, x_axis, y_axis, z_axis);
}

Plane Plane::from_points(const std::vector<Point>& points) {

    if (points.size() < 3)
        return Plane();

    const Vector v1 = points[1] - points[0];
    const Vector v2 = points[2] - points[0];

    Vector z_axis = v1.cross(v2);
    z_axis.normalize_self();

    Vector x_axis = v1;
    x_axis.normalize_self();

    Vector y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();

    return from_frame(points[0], x_axis, y_axis, z_axis);
}

/// Mean of the points.
static Point pca_centroid(const std::vector<Point>& points) {

    const double n = static_cast<double>(points.size());
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;

    for (const Point& p : points) {
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }

    cx /= n;
    cy /= n;
    cz /= n;

    return Point(cx, cy, cz);
}

/// Covariance matrix of the points about their centroid.
static std::array<std::array<double, 3>, 3> pca_covariance(const std::vector<Point>& points, const Point& centroid) {

    double cxx = 0.0;
    double cyy = 0.0;
    double czz = 0.0;
    double cxy = 0.0;
    double cxz = 0.0;
    double cyz = 0.0;

    for (const Point& p : points) {
        const double dx = p[0] - centroid[0];
        const double dy = p[1] - centroid[1];
        const double dz = p[2] - centroid[2];

        cxx += dx * dx;
        cyy += dy * dy;
        czz += dz * dz;
        cxy += dx * dy;
        cxz += dx * dz;
        cyz += dy * dz;
    }

    return {{{cxx, cxy, cxz}, {cxy, cyy, cyz}, {cxz, cyz, czz}}};
}

/// Eigenvectors of a covariance matrix by power iteration with deflation, largest eigenvalue first.
static std::array<std::array<double, 3>, 3> pca_eigenvectors(std::array<std::array<double, 3>, 3> cov) {

    std::array<std::array<double, 3>, 3> eigvec{};
    std::array<double, 3> eigval{};

    for (int e = 0; e < 3; e++) {
        double vx = e == 0 ? 1.0 : 0.0;
        double vy = e == 1 ? 1.0 : 0.0;
        double vz = e == 2 ? 1.0 : 0.0;

        for (int iter = 0; iter < 100; iter++) {
            const double nx = cov[0][0] * vx + cov[0][1] * vy + cov[0][2] * vz;
            const double ny = cov[1][0] * vx + cov[1][1] * vy + cov[1][2] * vz;
            const double nz = cov[2][0] * vx + cov[2][1] * vy + cov[2][2] * vz;
            const double mag = std::sqrt(nx * nx + ny * ny + nz * nz);

            if (mag < 1e-15)
                break;

            vx = nx / mag;
            vy = ny / mag;
            vz = nz / mag;
        }

        eigvec[e][0] = vx;
        eigvec[e][1] = vy;
        eigvec[e][2] = vz;
        eigval[e] = cov[0][0] * vx * vx + cov[1][1] * vy * vy + cov[2][2] * vz * vz + 2.0 * cov[0][1] * vx * vy + 2.0 * cov[0][2] * vx * vz + 2.0 * cov[1][2] * vy * vz;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov[i][j] -= eigval[e] * eigvec[e][i] * eigvec[e][j];
    }

    return eigvec;
}

Plane Plane::from_points_pca(const std::vector<Point>& points) {

    if (points.size() < 3)
        return Plane();

    const Point centroid = pca_centroid(points);
    const std::array<std::array<double, 3>, 3> eigvec = pca_eigenvectors(pca_covariance(points, centroid));

    Vector x_axis(eigvec[0][0], eigvec[0][1], eigvec[0][2]);
    Vector y_axis(eigvec[1][0], eigvec[1][1], eigvec[1][2]);

    Vector z_axis = x_axis.cross(y_axis);
    z_axis.normalize_self();

    y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();
    x_axis.normalize_self();

    return from_frame(centroid, x_axis, y_axis, z_axis);
}

Plane Plane::from_two_points(const Point& point1, const Point& point2) {

    Vector x_axis = point2 - point1;
    x_axis.normalize_self();

    Vector z_axis;
    z_axis.perpendicular_to(x_axis);
    z_axis.normalize_self();

    Vector y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();

    return from_frame(point1, x_axis, y_axis, z_axis);
}

Plane Plane::invalid() {
    return from_frame(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
}

Plane Plane::xy_plane() {

    Plane plane = from_frame(Point(0.0, 0.0, 0.0), Vector::x_axis(), Vector::y_axis(), Vector::z_axis());
    plane.name = "xy_plane";

    return plane;
}

Plane Plane::yz_plane() {

    Plane plane = from_frame(Point(0.0, 0.0, 0.0), Vector::y_axis(), Vector::z_axis(), Vector::x_axis());
    plane.name = "yz_plane";

    return plane;
}

Plane Plane::xz_plane() {

    Plane plane = from_frame(Point(0.0, 0.0, 0.0), Vector::x_axis(), Vector(0.0, 0.0, -1.0), Vector(0.0, 1.0, 0.0));
    plane.name = "xz_plane";

    return plane;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
Vector& Plane::operator[](int index) {

    if (index == 0)
        return _x_axis;

    if (index == 1)
        return _y_axis;

    if (index == 2)
        return _z_axis;

    throw std::out_of_range("Index out of range");
}

const Vector& Plane::operator[](int index) const {

    if (index == 0)
        return _x_axis;

    if (index == 1)
        return _y_axis;

    if (index == 2)
        return _z_axis;

    throw std::out_of_range("Index out of range");
}

bool Plane::operator==(const Plane& other) const {
    return name == other.name && _origin == other._origin && _x_axis == other._x_axis && _y_axis == other._y_axis && _z_axis == other._z_axis && linecolor == other.linecolor;
}

bool Plane::operator!=(const Plane& other) const {
    return !(*this == other);
}

Plane& Plane::operator+=(const Vector& other) {

    _origin += other;
    update_equation();

    return *this;
}

Plane& Plane::operator-=(const Vector& other) {

    _origin -= other;
    update_equation();

    return *this;
}

Plane Plane::operator+(const Vector& other) const {

    Plane result = *this;
    result += other;

    return result;
}

Plane Plane::operator-(const Vector& other) const {

    Plane result = *this;
    result -= other;

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void Plane::transform(const Xform& xform) {

    _origin.transform(xform);
    _x_axis.transform(xform);
    _y_axis.transform(xform);
    _z_axis.transform(xform);
    update_equation();
}

Plane Plane::transformed(const Xform& xform) const {

    Plane result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
bool Plane::is_valid() const {
    return _x_axis.magnitude() > 1e-14 && _y_axis.magnitude() > 1e-14 && _z_axis.magnitude() > 1e-14;
}

void Plane::reverse() {

    std::swap(_x_axis, _y_axis);
    _z_axis.reverse();
    update_equation();
}

void Plane::rotate(double angles_in_radians) {

    const double cos_angle = std::cos(angles_in_radians);
    const double sin_angle = std::sin(angles_in_radians);
    const Vector new_x = _x_axis * cos_angle + _y_axis * sin_angle;
    const Vector new_y = _y_axis * cos_angle - _x_axis * sin_angle;

    _x_axis = new_x;
    _y_axis = new_y;
}

bool Plane::is_right_hand() const {
    return _x_axis.cross(_y_axis).dot(_z_axis) > 0.999;
}

bool Plane::is_same_direction(const Plane& plane0, const Plane& plane1, bool can_be_flipped) {

    const int parallel = plane0._z_axis.is_parallel_to(plane1._z_axis);

    if (can_be_flipped)
        return parallel != 0;

    return parallel == -1;
}

bool Plane::is_same_position(const Plane& plane0, const Plane& plane1) {

    const double dist0 = std::abs(plane0._a * plane1._origin[0] + plane0._b * plane1._origin[1] + plane0._c * plane1._origin[2] + plane0._d);
    const double dist1 = std::abs(plane1._a * plane0._origin[0] + plane1._b * plane0._origin[1] + plane1._c * plane0._origin[2] + plane1._d);
    const double tolerance = static_cast<double>(Tolerance::APPROXIMATION);

    return dist0 < tolerance && dist1 < tolerance;
}

bool Plane::is_coplanar(const Plane& plane0, const Plane& plane1, bool can_be_flipped) {
    return is_same_direction(plane0, plane1, can_be_flipped) && is_same_position(plane0, plane1);
}

bool Plane::is_coplanar_from_normals(const Point& origin0, const Vector& normal0, const Point& origin1, const Vector& normal1, bool can_be_flipped, double tolerance) {

    const int parallel = normal0.is_parallel_to(normal1);

    if (can_be_flipped ? parallel == 0 : parallel != -1)
        return false;

    const double d0 = -(normal0[0] * origin0[0] + normal0[1] * origin0[1] + normal0[2] * origin0[2]);
    const double d1 = -(normal1[0] * origin1[0] + normal1[1] * origin1[1] + normal1[2] * origin1[2]);
    const double dist0 = std::abs(normal0[0] * origin1[0] + normal0[1] * origin1[1] + normal0[2] * origin1[2] + d0);
    const double dist1 = std::abs(normal1[0] * origin0[0] + normal1[1] * origin0[1] + normal1[2] * origin0[2] + d1);
    const double tol = tolerance < 0.0 ? static_cast<double>(Tolerance::APPROXIMATION) : tolerance;

    return dist0 < tol && dist1 < tol;
}

Plane Plane::translate_by_normal(double distance) const {

    Vector normal = _z_axis;
    normal.normalize_self();

    return Plane(_origin + normal * distance, _x_axis, _y_axis, name);
}

Point Plane::project(const Point& p) const {

    const double dist = _a * p[0] + _b * p[1] + _c * p[2] + _d;

    return Point(p[0] - dist * _a, p[1] - dist * _b, p[2] - dist * _c);
}

Point Plane::axis_point() const {

    const Vector& n = _z_axis;
    const double d = -n.dot(Vector(_origin[0], _origin[1], _origin[2]));
    const double fa = std::abs(n[0]);
    const double fb = std::abs(n[1]);
    const double fc = std::abs(n[2]);

    if (fa > fb && fa > fc)
        return Point(-d / n[0], 0.0, 0.0);

    if (fb > fc)
        return Point(0.0, -d / n[1], 0.0);

    return Point(0.0, 0.0, -d / n[2]);
}

bool Plane::has_on_negative_side(const Point& p) const {
    return _a * p[0] + _b * p[1] + _c * p[2] + _d < 0.0;
}

double Plane::squared_distance(const Point& p) const {

    const double value = _a * p[0] + _b * p[1] + _c * p[2] + _d;
    const double normal_sq = _a * _a + _b * _b + _c * _c;

    return normal_sq > 1e-20 ? value * value / normal_sq : value * value;
}

Vector Plane::base1() const {

    const double nx = _z_axis[0];
    const double ny = _z_axis[1];
    const double nz = _z_axis[2];
    const double ax = std::abs(nx);
    const double ay = std::abs(ny);
    const double az = std::abs(nz);
    Vector b;

    if (ax <= ay && ax <= az)
        b = Vector(0.0, -nz, ny);
    else if (ay <= ax && ay <= az)
        b = Vector(-nz, 0.0, nx);
    else
        b = Vector(-ny, nx, 0.0);

    b.normalize_self();

    return b;
}

Vector Plane::base2() const {

    Vector b2 = _z_axis.cross(base1());
    b2.normalize_self();

    return b2;
}

std::vector<Polyline> Plane::to_polylines(double scale) const {

    const double s = scale * 0.5;
    const Point& o = _origin;
    const Vector& x = _x_axis;
    const Vector& y = _y_axis;
    const Vector& z = _z_axis;
    const Point c0 = o - x * s - y * s;
    const Point c1 = o + x * s - y * s;
    const Point c2 = o + x * s + y * s;
    const Point c3 = o - x * s + y * s;
    const Point origin_pt(o[0], o[1], o[2]);

    Polyline rect({c0, c1, c2, c3, c0});
    rect.linecolor = linecolor;

    Polyline x_line({origin_pt, o + x * s});
    x_line.linecolor = Color::red();

    Polyline y_line({origin_pt, o + y * s});
    y_line.linecolor = Color::green();

    Polyline z_line({origin_pt, o + z * s});
    z_line.linecolor = Color::blue();

    return {rect, x_line, y_line, z_line};
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Plane::jsondump() const {

    nlohmann::ordered_json data;
    data["frame"] = {_origin[0], _origin[1], _origin[2], _x_axis[0], _x_axis[1], _x_axis[2], _y_axis[0], _y_axis[1], _y_axis[2], _z_axis[0], _z_axis[1], _z_axis[2]};
    data["guid"] = guid();
    data["linecolor"] = linecolor.jsondump();
    data["name"] = name;
    data["type"] = "Plane";
    data["width"] = width;

    return data;
}

Plane Plane::jsonload(const nlohmann::json& data) {

    const nlohmann::json& frame = data["frame"];
    Plane plane = from_frame(
        Point(frame[0].get<double>(), frame[1].get<double>(), frame[2].get<double>()),
        Vector(frame[3].get<double>(), frame[4].get<double>(), frame[5].get<double>()),
        Vector(frame[6].get<double>(), frame[7].get<double>(), frame[8].get<double>()),
        Vector(frame[9].get<double>(), frame[10].get<double>(), frame[11].get<double>())
    );

    plane.guid() = data["guid"];
    plane.name = data["name"];

    if (data.contains("linecolor"))
        plane.linecolor = Color::jsonload(data["linecolor"]);

    if (data.contains("width"))
        plane.width = data["width"].get<double>();

    return plane;
}

std::string Plane::file_json_dumps() const {
    return jsondump().dump();
}

Plane Plane::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Plane::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(2);
}

Plane Plane::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Plane Plane::to_proto() const {

    session_proto::Plane proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);

    for (int i = 0; i < 3; i++)
        proto.add_frame(_origin[i]);

    for (int i = 0; i < 3; i++)
        proto.add_frame(_x_axis[i]);

    for (int i = 0; i < 3; i++)
        proto.add_frame(_y_axis[i]);

    for (int i = 0; i < 3; i++)
        proto.add_frame(_z_axis[i]);

    proto.set_width(width);
    *proto.mutable_linecolor() = linecolor.to_proto();

    return proto;
}

Plane Plane::from_proto(const session_proto::Plane& proto) {

    Plane plane;

    if (proto.frame_size() >= 12)
        plane = from_frame(
            Point(proto.frame(0), proto.frame(1), proto.frame(2)),
            Vector(proto.frame(3), proto.frame(4), proto.frame(5)),
            Vector(proto.frame(6), proto.frame(7), proto.frame(8)),
            Vector(proto.frame(9), proto.frame(10), proto.frame(11))
        );

    if (!proto.guid().empty())
        plane.guid() = proto.guid();

    plane.name = proto.name();

    if (proto.width() > 0.0)
        plane.width = proto.width();

    plane.linecolor = Color::from_proto(proto.linecolor());

    return plane;
}

std::string Plane::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Plane Plane::pb_loads(const std::string& data) {

    session_proto::Plane proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Plane protobuf data");

    return from_proto(proto);
}

void Plane::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Plane Plane::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Plane::str() const {
    return fmt::format("{}\n{}\n{}\n{}", _origin.str(), _x_axis.str(), _y_axis.str(), _z_axis.str());
}

std::string Plane::repr() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "Plane({}, {}, {}, {}, {}, {}, {}, {})",
        name,
        TOLERANCE.format_number(_origin[0], prec),
        TOLERANCE.format_number(_origin[1], prec),
        TOLERANCE.format_number(_origin[2], prec),
        TOLERANCE.format_number(_z_axis[0], prec),
        TOLERANCE.format_number(_z_axis[1], prec),
        TOLERANCE.format_number(_z_axis[2], prec),
        linecolor.repr()
    );
}

std::ostream& operator<<(std::ostream& os, const Plane& plane) {
    return os << plane.str();
}

} // namespace session_cpp
