#include "line.h"
#include "polyline.h"
#include "tolerance.h"
#include "line.pb.h"
#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Line::Line(double x0, double y0, double z0, double x1, double y1, double z1)
    : _x0(x0), _y0(y0), _z0(z0), _x1(x1), _y1(y1), _z1(z1) {}

Line::Line(const Line& other)
    : _x0(other._x0), _y0(other._y0), _z0(other._z0), _x1(other._x1), _y1(other._y1), _z1(other._z1),
      name(other.name), width(other.width), dash(other.dash), linecolor(other.linecolor) {}

Line& Line::operator=(const Line& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    _x0 = other._x0;
    _y0 = other._y0;
    _z0 = other._z0;
    _x1 = other._x1;
    _y1 = other._y1;
    _z1 = other._z1;
    name = other.name;
    width = other.width;
    dash = other.dash;
    linecolor = other.linecolor;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& Line::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& Line::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

void Line::refresh_guid() {
    _guid.clear();
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
Line Line::from_points(const Point& p1, const Point& p2) {
    return Line(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
}

Line Line::from_point_and_vector(const Point& point, const Vector& vector) {
    return from_points(point, point + vector);
}

Line Line::from_point_direction_length(const Point& point, const Vector& direction, double length) {
    return from_points(point, point + direction.normalized() * length);
}

/// Power iteration on the covariance rows from seed: the unit axis and its eigenvalue estimate.
static std::pair<Vector, double> fit_points_power(const Vector& row0, const Vector& row1, const Vector& row2, const Vector& seed) {

    Vector axis = seed;
    double eigen = 0.0;

    for (int i = 0; i < 100; i++) {
        const Vector next(row0.dot(axis), row1.dot(axis), row2.dot(axis));
        eigen = std::sqrt(next.magnitude_squared());

        if (eigen < 1e-15)
            break;

        axis = next / eigen;
    }

    return {axis, eigen};
}

/// Principal direction of the points about center: power iteration from each of X, Y and Z, largest eigenvalue kept.
static Vector fit_points_axis(const std::vector<Point>& points, const Point& center) {

    double cxx = 0.0;
    double cyy = 0.0;
    double czz = 0.0;
    double cxy = 0.0;
    double cxz = 0.0;
    double cyz = 0.0;

    for (const Point& p : points) {
        const Vector d = p - center;
        cxx += d[0] * d[0];
        cyy += d[1] * d[1];
        czz += d[2] * d[2];
        cxy += d[0] * d[1];
        cxz += d[0] * d[2];
        cyz += d[1] * d[2];
    }

    const Vector row0(cxx, cxy, cxz);
    const Vector row1(cxy, cyy, cyz);
    const Vector row2(cxz, cyz, czz);
    int first = 0;

    if (cyy > cxx && cyy >= czz)
        first = 1;
    else if (czz > cxx && czz > cyy)
        first = 2;

    Vector axis(1.0, 0.0, 0.0);
    double best = -1.0;

    for (int k = 0; k < 3; k++) {
        Vector seed(0.0, 0.0, 0.0);
        seed[(first + k) % 3] = 1.0;
        const std::pair<Vector, double> power = fit_points_power(row0, row1, row2, seed);

        if (power.second > best * (1.0 + Tolerance::RELATIVE)) {
            axis = power.first;
            best = power.second;
        }
    }

    return axis;
}

/// Parameter range of the fitted line along axis: +-length / 2, or the projected extent when length <= 0.
static std::pair<double, double> fit_points_extent(const std::vector<Point>& points, const Point& center, const Vector& axis, double length) {

    if (length > 0.0)
        return {-length / 2.0, length / 2.0};

    double t_min = 0.0;
    double t_max = 0.0;

    for (const Point& p : points) {
        const double t = (p - center).dot(axis);
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
    }

    if (t_max - t_min < 1e-10)
        return {-0.5, 0.5};

    return {t_min, t_max};
}

Line Line::fit_points(const std::vector<Point>& points, double length) {

    if (points.size() < 2)
        throw std::invalid_argument("At least 2 points are required for line fitting");

    const Point center = Point::centroid(points);
    const Vector axis = fit_points_axis(points, center);
    const std::pair<double, double> extent = fit_points_extent(points, center, axis, length);

    return from_points(center + axis * extent.first, center + axis * extent.second);
}

Line Line::with_name(const std::string& name, double x0, double y0, double z0, double x1, double y1, double z1) {

    Line line(x0, y0, z0, x1, y1, z1);
    line.name = name;

    return line;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
double& Line::operator[](int index) {

    if (index == 0)
        return _x0;

    if (index == 1)
        return _y0;

    if (index == 2)
        return _z0;

    if (index == 3)
        return _x1;

    if (index == 4)
        return _y1;

    if (index == 5)
        return _z1;

    throw std::out_of_range("Index out of bounds");
}

const double& Line::operator[](int index) const {

    if (index == 0)
        return _x0;

    if (index == 1)
        return _y0;

    if (index == 2)
        return _z0;

    if (index == 3)
        return _x1;

    if (index == 4)
        return _y1;

    if (index == 5)
        return _z1;

    throw std::out_of_range("Index out of bounds");
}

bool Line::operator==(const Line& other) const {

    return name == other.name &&
           std::round(_x0 * 1000000.0) == std::round(other._x0 * 1000000.0) &&
           std::round(_y0 * 1000000.0) == std::round(other._y0 * 1000000.0) &&
           std::round(_z0 * 1000000.0) == std::round(other._z0 * 1000000.0) &&
           std::round(_x1 * 1000000.0) == std::round(other._x1 * 1000000.0) &&
           std::round(_y1 * 1000000.0) == std::round(other._y1 * 1000000.0) &&
           std::round(_z1 * 1000000.0) == std::round(other._z1 * 1000000.0) &&
           std::round(width * 1000000.0) == std::round(other.width * 1000000.0) &&
           linecolor == other.linecolor;
}

bool Line::operator!=(const Line& other) const {
    return !(*this == other);
}

Line& Line::operator+=(const Vector& other) {

    _x0 += other[0];
    _y0 += other[1];
    _z0 += other[2];
    _x1 += other[0];
    _y1 += other[1];
    _z1 += other[2];

    return *this;
}

Line& Line::operator-=(const Vector& other) {

    _x0 -= other[0];
    _y0 -= other[1];
    _z0 -= other[2];
    _x1 -= other[0];
    _y1 -= other[1];
    _z1 -= other[2];

    return *this;
}

Line& Line::operator*=(double factor) {

    _x0 *= factor;
    _y0 *= factor;
    _z0 *= factor;
    _x1 *= factor;
    _y1 *= factor;
    _z1 *= factor;

    return *this;
}

Line& Line::operator/=(double factor) {

    _x0 /= factor;
    _y0 /= factor;
    _z0 /= factor;
    _x1 /= factor;
    _y1 /= factor;
    _z1 /= factor;

    return *this;
}

Line Line::operator+(const Vector& other) const {

    Line result = *this;
    result += other;

    return result;
}

Line Line::operator-(const Vector& other) const {

    Line result = *this;
    result -= other;

    return result;
}

Line Line::operator*(double factor) const {

    Line result = *this;
    result *= factor;

    return result;
}

Line Line::operator/(double factor) const {

    Line result = *this;
    result /= factor;

    return result;
}

Line Line::operator-() const {
    return Line(_x1, _y1, _z1, _x0, _y0, _z0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void Line::transform(const Xform& xform) {

    const Point s = start().transformed(xform);
    const Point e = end().transformed(xform);

    _x0 = s[0];
    _y0 = s[1];
    _z0 = s[2];
    _x1 = e[0];
    _y1 = e[1];
    _z1 = e[2];
}

Line Line::transformed(const Xform& xform) const {

    Line result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
double Line::length() const {
    return std::sqrt(squared_length());
}

double Line::squared_length() const {
    return to_vector().magnitude_squared();
}

Vector Line::to_vector() const {
    return Vector(_x1 - _x0, _y1 - _y0, _z1 - _z0);
}

Vector Line::to_direction() const {
    return to_vector().normalized();
}

Point Line::start() const {
    return Point(_x0, _y0, _z0);
}

Point Line::end() const {
    return Point(_x1, _y1, _z1);
}

Point Line::center() const {
    return Point((_x0 + _x1) * 0.5, (_y0 + _y1) * 0.5, (_z0 + _z1) * 0.5);
}

Point Line::point_at(double t) const {

    const double s = 1.0 - t;

    return Point(s * _x0 + t * _x1, s * _y0 + t * _y1, s * _z0 + t * _z1);
}

std::vector<Point> Line::subdivide(int n) const {

    if (n < 2)
        throw std::invalid_argument("n must be at least 2");

    std::vector<Point> points;
    points.reserve(n);

    for (int i = 0; i < n; i++)
        points.push_back(point_at(static_cast<double>(i) / (n - 1)));

    return points;
}

std::vector<Point> Line::subdivide_by_distance(double distance) const {

    if (distance <= 0.0)
        throw std::invalid_argument("distance must be positive");

    const double total = length();

    if (total < 1e-10)
        return {start(), end()};

    const int n = std::max(2, static_cast<int>(total / distance + 0.5) + 1);

    return subdivide(n);
}

std::pair<double, Point> Line::closest_point(const Point& point, bool limited) const {

    const Vector d = to_vector();
    const double len_sq = d.magnitude_squared();

    if (len_sq < 1e-20)
        return {0.0, start()};

    double t = (point - start()).dot(d) / len_sq;

    if (limited)
        t = std::clamp(t, 0.0, 1.0);

    return {t, point_at(t)};
}

void Line::get_middle_line(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& output_start, Point& output_end) {

    output_start = Point(
        (line0_start[0] + line1_start[0]) * 0.5,
        (line0_start[1] + line1_start[1]) * 0.5,
        (line0_start[2] + line1_start[2]) * 0.5
    );

    output_end = Point(
        (line0_end[0] + line1_end[0]) * 0.5,
        (line0_end[1] + line1_end[1]) * 0.5,
        (line0_end[2] + line1_end[2]) * 0.5
    );
}

void Line::get_middle_line(const Line& l0, const Line& l1, Line& out) {

    Point output_start;
    Point output_end;
    get_middle_line(l0.start(), l0.end(), l1.start(), l1.end(), output_start, output_end);

    out = from_points(output_start, output_end);
}

bool Line::from_projected_points(const Line& line, const std::vector<Point>& points, Line& out) {

    Point output_start;
    Point output_end;
    const bool ok = Polyline::line_from_projected_points(line.start(), line.end(), points, output_start, output_end);

    out = from_points(output_start, output_end);

    return ok;
}

bool Line::overlap(const Line& other, Line& out) const {

    Point output_start;
    Point output_end;
    const bool ok = Polyline::line_line_overlap(start(), end(), other.start(), other.end(), output_start, output_end);

    out = from_points(output_start, output_end);

    return ok;
}

bool Line::overlap_average(const Line& other, Line& out) const {

    Point output_start;
    Point output_end;
    Polyline::line_line_overlap_average(start(), end(), other.start(), other.end(), output_start, output_end);

    out = from_points(output_start, output_end);

    return out.squared_length() > 0.0;
}

void Line::extend(double ext_start, double ext_end) {

    Point s = start();
    Point e = end();
    Polyline::extend_line_segment(s, e, ext_start, ext_end);

    _x0 = s[0];
    _y0 = s[1];
    _z0 = s[2];
    _x1 = e[0];
    _y1 = e[1];
    _z1 = e[2];
}

void Line::extend_equally(double dist, double proportion) {

    if (dist == 0.0 && proportion == 0.0)
        return;

    Point s = start();
    Point e = end();
    Polyline::extend_segment_equally(s, e, dist, proportion);

    _x0 = s[0];
    _y0 = s[1];
    _z0 = s[2];
    _x1 = e[0];
    _y1 = e[1];
    _z1 = e[2];
}

void Line::scale(double dist) {

    Point s = start();
    Point e = end();
    Polyline::shrink_line_segment(s, e, dist);

    _x0 = s[0];
    _y0 = s[1];
    _z0 = s[2];
    _x1 = e[0];
    _y1 = e[1];
    _z1 = e[2];
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Line::jsondump() const {

    nlohmann::ordered_json data;
    data["dash"] = dash;
    data["guid"] = guid();
    data["linecolor"] = linecolor.jsondump();
    data["name"] = name;
    data["type"] = "Line";
    data["width"] = width;
    data["x0"] = _x0;
    data["x1"] = _x1;
    data["y0"] = _y0;
    data["y1"] = _y1;
    data["z0"] = _z0;
    data["z1"] = _z1;

    return data;
}

Line Line::jsonload(const nlohmann::json& data) {

    Line line(data.at("x0"), data.at("y0"), data.at("z0"), data.at("x1"), data.at("y1"), data.at("z1"));
    line.guid() = data.at("guid");
    line.name = data.at("name");

    if (data.contains("dash"))
        line.dash = data["dash"].get<std::vector<double>>();

    if (data.contains("linecolor"))
        line.linecolor = Color::jsonload(data["linecolor"]);

    if (data.contains("width"))
        line.width = data["width"].get<double>();

    return line;
}

std::string Line::file_json_dumps() const {
    return jsondump().dump();
}

Line Line::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Line::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(2);
}

Line Line::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Line Line::to_proto() const {

    session_proto::Line proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_width(width);

    for (int i = 0; i < 6; i++)
        proto.add_coords((*this)[i]);

    for (double d : dash)
        proto.add_dash(d);

    proto.add_linecolor_rgba(linecolor.r);
    proto.add_linecolor_rgba(linecolor.g);
    proto.add_linecolor_rgba(linecolor.b);
    proto.add_linecolor_rgba(linecolor.a);
    proto.set_linecolor_name(linecolor.name);

    return proto;
}

Line Line::from_proto(const session_proto::Line& proto) {

    Line line;

    if (proto.coords_size() == 6)
        line = Line(proto.coords(0), proto.coords(1), proto.coords(2), proto.coords(3), proto.coords(4), proto.coords(5));

    if (!proto.guid().empty())
        line.guid() = proto.guid();

    line.name = proto.name();

    if (proto.width() > 0.0)
        line.width = proto.width();

    line.dash.assign(proto.dash().begin(), proto.dash().end());

    if (proto.linecolor_rgba_size() == 4) {
        line.linecolor.r = proto.linecolor_rgba(0);
        line.linecolor.g = proto.linecolor_rgba(1);
        line.linecolor.b = proto.linecolor_rgba(2);
        line.linecolor.a = proto.linecolor_rgba(3);

        if (!proto.linecolor_name().empty())
            line.linecolor.name = proto.linecolor_name();
    }

    return line;
}

std::string Line::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Line Line::pb_loads(const std::string& data) {

    session_proto::Line proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Line protobuf data");

    return from_proto(proto);
}

void Line::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Line Line::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Line::str() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "{}, {}, {}, {}, {}, {}",
        TOLERANCE.format_number(_x0, prec),
        TOLERANCE.format_number(_y0, prec),
        TOLERANCE.format_number(_z0, prec),
        TOLERANCE.format_number(_x1, prec),
        TOLERANCE.format_number(_y1, prec),
        TOLERANCE.format_number(_z1, prec)
    );
}

std::string Line::repr() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "Line({}, {}, {}, {}, {}, {}, {}, {}, {})",
        name,
        TOLERANCE.format_number(_x0, prec),
        TOLERANCE.format_number(_y0, prec),
        TOLERANCE.format_number(_z0, prec),
        TOLERANCE.format_number(_x1, prec),
        TOLERANCE.format_number(_y1, prec),
        TOLERANCE.format_number(_z1, prec),
        linecolor.repr(),
        TOLERANCE.format_number(width, prec)
    );
}

std::ostream& operator<<(std::ostream& os, const Line& line) {
    return os << line.str();
}

} // namespace session_cpp
