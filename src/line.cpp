#include "line.h"
#include "polyline.h"  // Polyline static helpers (line_line_overlap, extend_line_segment, ...)
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "line.pb.h"
#include "point.pb.h"

namespace session_cpp {

Line::Line() : _x0(0.0), _y0(0.0), _z0(0.0), _x1(0.0), _y1(0.0), _z1(1.0) {}

Line::Line(double x0, double y0, double z0, double x1, double y1, double z1) 
    : _x0(x0), _y0(y0), _z0(z0), _x1(x1), _y1(y1), _z1(z1) {}

Line Line::from_points(const Point& p1, const Point& p2) {
    return Line(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
}

Line Line::fit_points(const std::vector<Point>& points, double length) {
    if (points.size() < 2) {
        throw std::invalid_argument("At least 2 points are required for line fitting");
    }

    size_t n = points.size();

    // Compute centroid
    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (const auto& p : points) {
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }
    cx /= n;
    cy /= n;
    cz /= n;

    // Compute covariance matrix elements
    double cxx = 0.0, cyy = 0.0, czz = 0.0;
    double cxy = 0.0, cxz = 0.0, cyz = 0.0;
    for (const auto& p : points) {
        double dx = p[0] - cx;
        double dy = p[1] - cy;
        double dz = p[2] - cz;
        cxx += dx * dx;
        cyy += dy * dy;
        czz += dz * dz;
        cxy += dx * dy;
        cxz += dx * dz;
        cyz += dy * dz;
    }

    // Power iteration to find dominant eigenvector
    double vx = 1.0, vy = 0.0, vz = 0.0;
    for (int iter = 0; iter < 100; ++iter) {
        double nx = cxx * vx + cxy * vy + cxz * vz;
        double ny = cxy * vx + cyy * vy + cyz * vz;
        double nz = cxz * vx + cyz * vy + czz * vz;
        double mag = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (mag < 1e-15) break;
        vx = nx / mag;
        vy = ny / mag;
        vz = nz / mag;
    }

    // Determine line extent from projected points
    double half_len;
    if (length <= 0.0) {
        double t_min = 0.0, t_max = 0.0;
        for (const auto& p : points) {
            double dx = p[0] - cx;
            double dy = p[1] - cy;
            double dz = p[2] - cz;
            double t = dx * vx + dy * vy + dz * vz;
            t_min = std::min(t_min, t);
            t_max = std::max(t_max, t);
        }
        half_len = std::max(std::abs(t_min), std::abs(t_max));
        if (half_len < 1e-10) half_len = 0.5;
    } else {
        half_len = length / 2.0;
    }

    // Create line from centroid +/- direction * half_len
    return Line(
        cx - vx * half_len, cy - vy * half_len, cz - vz * half_len,
        cx + vx * half_len, cy + vy * half_len, cz + vz * half_len
    );
}

Line Line::with_name(const std::string& name, double x0, double y0, double z0, double x1, double y1, double z1) {
    Line line(x0, y0, z0, x1, y1, z1);
    line.name = name;
    return line;
}

/// Copy constructor (creates a new guid() while copying data)
Line::Line(const Line& other)
    :
      name(other.name),
      width(other.width),
      linecolor(other.linecolor),
      xform(other.xform),
      _x0(other._x0),
      _y0(other._y0),
      _z0(other._z0),
      _x1(other._x1),
      _y1(other._y1),
      _z1(other._z1) {}

/// Copy assignment (creates a new guid() while copying data)
Line& Line::operator=(const Line& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        width = other.width;
        linecolor = other.linecolor;
        xform = other.xform;
        _x0 = other._x0;
        _y0 = other._y0;
        _z0 = other._z0;
        _x1 = other._x1;
        _y1 = other._y1;
        _z1 = other._z1;
    }
    return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void Line::transform() {
  Point start(_x0, _y0, _z0);
  Point end(_x1, _y1, _z1);
  
  start.xform = xform; start.transform();
  end.xform = xform; end.transform();
  
  _x0 = start[0];
  _y0 = start[1];
  _z0 = start[2];
  _x1 = end[0];
  _y1 = end[1];
  _z1 = end[2];
  xform = Xform::identity();
}

Line Line::transformed() const {
  Line result = *this;
  result.transform();
  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Line::jsondump() const {
    // Alphabetical order to match Rust's serde_json
    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["linecolor"] = linecolor.jsondump();
    data["name"] = name;
    data["type"] = "Line";
    data["width"] = width;
    data["x0"] = _x0;
    data["x1"] = _x1;
    data["xform"] = xform.jsondump();
    data["y0"] = _y0;
    data["y1"] = _y1;
    data["z0"] = _z0;
    data["z1"] = _z1;
    return data;
}

Line Line::jsonload(const nlohmann::json& data) {
    Line line(data["x0"], data["y0"], data["z0"], data["x1"], data["y1"], data["z1"]);
    line.guid() = data["guid"];
    line.name = data["name"];
    line.linecolor = Color::jsonload(data["linecolor"]);
    line.width = data["width"];
    if (data.contains("xform")) {
        line.xform = Xform::jsonload(data["xform"]);
    }
    return line;
}

std::string Line::file_json_dumps() const {
    return jsondump().dump();
}

Line Line::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Line::file_json_dump(const std::string& filename) const {
    std::ofstream ofs(filename);
    ofs << jsondump().dump(4);
    ofs.close();
}

Line Line::file_json_load(const std::string& filename) {
    std::ifstream ifs(filename);
    nlohmann::json data = nlohmann::json::parse(ifs);
    ifs.close();
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

std::string Line::pb_dumps() const {
    session_proto::Line proto;
    auto* start = proto.mutable_start();
    start->set_x(_x0);
    start->set_y(_y0);
    start->set_z(_z0);
    auto* end = proto.mutable_end();
    end->set_x(_x1);
    end->set_y(_y1);
    end->set_z(_z1);
    proto.set_guid(guid());
    proto.set_name(name);
    // Serialize xform
    auto* proto_xform = proto.mutable_xform();
    proto_xform->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        proto_xform->add_matrix(xform.m[i]);
    }
    return proto.SerializeAsString();
}

Line Line::pb_loads(const std::string& data) {
    session_proto::Line proto;
    proto.ParseFromString(data);
    Line line(proto.start().x(), proto.start().y(), proto.start().z(),
              proto.end().x(), proto.end().y(), proto.end().z());
    line.guid() = proto.guid();
    line.name = proto.name();
    // Deserialize xform if present
    if (proto.has_xform()) {
        line.xform.name = proto.xform().name();
        for (int i = 0; i < proto.xform().matrix_size() && i < 16; ++i) {
            line.xform.m[i] = proto.xform().matrix(i);
        }
    }
    return line;
}

void Line::pb_dump(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << pb_dumps();
    ofs.close();
}

Line Line::pb_load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
    ifs.close();
    return pb_loads(data);
}

/// Simple string form (like Python __str__): just coordinates
std::string Line::str() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format(
        "{}, {}, {}, {}, {}, {}",
        TOLERANCE.format_number(_x0, prec),
        TOLERANCE.format_number(_y0, prec),
        TOLERANCE.format_number(_z0, prec),
        TOLERANCE.format_number(_x1, prec),
        TOLERANCE.format_number(_y1, prec),
        TOLERANCE.format_number(_z1, prec));
}

/// Detailed representation (like Python __repr__)
std::string Line::repr() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format(
        "Line({}, {}, {}, {}, {}, {}, {}, Color({}, {}, {}, {}), {})",
        name,
        TOLERANCE.format_number(_x0, prec),
        TOLERANCE.format_number(_y0, prec),
        TOLERANCE.format_number(_z0, prec),
        TOLERANCE.format_number(_x1, prec),
        TOLERANCE.format_number(_y1, prec),
        TOLERANCE.format_number(_z1, prec),
        linecolor.r, linecolor.g, linecolor.b, linecolor.a,
        TOLERANCE.format_number(width, prec));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////////////////

double& Line::operator[](int index) {
    switch(index) {
        case 0: return _x0;
        case 1: return _y0;
        case 2: return _z0;
        case 3: return _x1;
        case 4: return _y1;
        case 5: return _z1;
        default: throw std::out_of_range("Index out of bounds");
    }
}

const double& Line::operator[](int index) const {
    switch(index) {
        case 0: return _x0;
        case 1: return _y0;
        case 2: return _z0;
        case 3: return _x1;
        case 4: return _y1;
        case 5: return _z1;
        default: throw std::out_of_range("Index out of bounds");
    }
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

Vector Line::to_vector() const {
    return Vector(_x1 - _x0, _y1 - _y0, _z1 - _z0);
}

Vector Line::to_direction() const {
    return to_vector().normalized();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Geometry Methods
///////////////////////////////////////////////////////////////////////////////////////////

double Line::length() const {
    double dx = _x1 - _x0;
    double dy = _y1 - _y0;
    double dz = _z1 - _z0;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Line::squared_length() const {
    double dx = _x1 - _x0;
    double dy = _y1 - _y0;
    double dz = _z1 - _z0;
    return dx * dx + dy * dy + dz * dz;
}

Point Line::point_at(double t) const {
    double s = 1.0 - t;
    return Point(s * _x0 + t * _x1, s * _y0 + t * _y1, s * _z0 + t * _z1);
}

std::vector<Point> Line::subdivide(int n) const {
    if (n < 2) {
        throw std::invalid_argument("n must be at least 2");
    }
    std::vector<Point> points;
    points.reserve(n);
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1);
        points.push_back(point_at(t));
    }
    return points;
}

std::vector<Point> Line::subdivide_by_distance(double distance) const {
    if (distance <= 0) {
        throw std::invalid_argument("distance must be positive");
    }
    double len = length();
    if (len < 1e-10) {
        return {start(), end()};
    }
    int n = std::max(2, static_cast<int>(len / distance + 0.5) + 1);
    return subdivide(n);
}

Point Line::start() const {
    return Point(_x0, _y0, _z0);
}

Point Line::end() const {
    return Point(_x1, _y1, _z1);
}

Point Line::center() const {
    return Point(
        (_x0 + _x1) * 0.5,
        (_y0 + _y1) * 0.5,
        (_z0 + _z1) * 0.5
    );
}

std::pair<double, Point> Line::closest_point(const Point& point, bool limited) const {
    double dx = _x1 - _x0;
    double dy = _y1 - _y0;
    double dz = _z1 - _z0;
    double len_sq = dx * dx + dy * dy + dz * dz;
    if (len_sq < 1e-20) {
        return {0.0, start()};
    }
    double t = ((point[0] - _x0) * dx + (point[1] - _y0) * dy + (point[2] - _z0) * dz) / len_sq;
    if (limited) {
        t = std::max(0.0, std::min(1.0, t));
    }
    return {t, point_at(t)};
}

Line Line::from_point_and_vector(const Point& point, const Vector& vector) {
    return Line(
        point[0], point[1], point[2],
        point[0] + vector[0], point[1] + vector[1], point[2] + vector[2]
    );
}

Line Line::from_point_direction_length(const Point& point, const Vector& direction, double length) {
    Vector d = direction.normalized();
    return Line(
        point[0], point[1], point[2],
        point[0] + d[0] * length, point[1] + d[1] * length, point[2] + d[2] * length
    );
}

void Line::get_middle_line(const Point& line0_start, const Point& line0_end,
                          const Point& line1_start, const Point& line1_end,
                          Point& output_start, Point& output_end) {
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

bool Line::overlap(const Line& other, Line& out) const {
    return ::session_cpp::line_line_overlap(*this, other, out);
}

bool Line::overlap_average(const Line& other, Line& out) const {
    ::session_cpp::line_line_overlap_average(*this, other, out);
    return out.squared_length() > 0.0;
}

void Line::extend(double ext_start, double ext_end) {
    ::session_cpp::extend_line(*this, ext_start, ext_end);
}

std::ostream& operator<<(std::ostream& os, const Line& line) {
    return os << line.str();
}

// ── Free functions on Line (moved from polyline.cpp; CGAL_Polyline compat) ──

void line_line_average(const Line& l0, const Line& l1, Line& out) {
    Point s0=l0.start(), e0=l0.end(), s1=l1.start(), e1=l1.end();
    out = Line::from_points(Point((s0[0]+s1[0])*0.5, (s0[1]+s1[1])*0.5, (s0[2]+s1[2])*0.5),
                            Point((e0[0]+e1[0])*0.5, (e0[1]+e1[1])*0.5, (e0[2]+e1[2])*0.5));
}

bool line_line_overlap(const Line& l0, const Line& l1, Line& out) {
    Point os, oe;
    bool r = Polyline::line_line_overlap(l0.start(), l0.end(), l1.start(), l1.end(), os, oe);
    out = Line::from_points(os, oe);
    return r;
}

void line_line_overlap_average(const Line& l0, const Line& l1, Line& out) {
    Line lineA, lineB;
    line_line_overlap(l0, l1, lineA);
    line_line_overlap(l1, l0, lineB);
    Point a0=lineA.start(), a1=lineA.end(), b0=lineB.start(), b1=lineB.end();
    Point m0s((a0[0]+b0[0])*0.5, (a0[1]+b0[1])*0.5, (a0[2]+b0[2])*0.5);
    Point m0e((a1[0]+b1[0])*0.5, (a1[1]+b1[1])*0.5, (a1[2]+b1[2])*0.5);
    Point m1s((a0[0]+b1[0])*0.5, (a0[1]+b1[1])*0.5, (a0[2]+b1[2])*0.5);
    Point m1e((a1[0]+b0[0])*0.5, (a1[1]+b0[1])*0.5, (a1[2]+b0[2])*0.5);
    double dx0=m0e[0]-m0s[0], dy0=m0e[1]-m0s[1], dz0=m0e[2]-m0s[2];
    double dx1=m1e[0]-m1s[0], dy1=m1e[1]-m1s[1], dz1=m1e[2]-m1s[2];
    out = (dx0*dx0+dy0*dy0+dz0*dz0 >= dx1*dx1+dy1*dy1+dz1*dz1)
          ? Line::from_points(m0s, m0e) : Line::from_points(m1s, m1e);
}

bool line_from_projected_points(const Line& line, const std::vector<Point>& pts, Line& out) {
    Point os, oe;
    bool r = Polyline::line_from_projected_points(line.start(), line.end(), pts, os, oe);
    out = Line::from_points(os, oe);
    return r;
}

void extend_line(Line& line, double d0, double d1) {
    Point s = line.start(), e = line.end();
    Polyline::extend_line_segment(s, e, d0, d1);
    line = Line::from_points(s, e);
}

void extend_equally(Line& line, double dist, double proportion) {
    if (dist == 0 && proportion == 0) return;
    Point s = line.start(), e = line.end();
    Polyline::extend_segment_equally(s, e, dist, proportion);
    line = Line::from_points(s, e);
}

void scale_line(Line& line, double dist) {
    Point s = line.start(), e = line.end();
    Vector v(e[0]-s[0], e[1]-s[1], e[2]-s[2]);
    s[0]+=v[0]*dist; s[1]+=v[1]*dist; s[2]+=v[2]*dist;
    e[0]-=v[0]*dist; e[1]-=v[1]*dist; e[2]-=v[2]*dist;
    line = Line::from_points(s, e);
}

void get_middle_line(const Line& l0, const Line& l1, Line& out) {
    Point s0=l0.start(), e0=l0.end(), s1=l1.start(), e1=l1.end();
    Point os, oe;
    Line::get_middle_line(s0, e0, s1, e1, os, oe);
    out = Line::from_points(os, oe);
}

}  // namespace session_cpp