#include "line.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#ifdef ENABLE_PROTOBUF
#include "line.pb.h"
#include "point.pb.h"
#endif

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

/// Copy constructor (creates a new guid while copying data)
Line::Line(const Line& other)
    : guid(::guid()),
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

/// Copy assignment (creates a new guid while copying data)
Line& Line::operator=(const Line& other) {
    if (this != &other) {
        guid = ::guid();
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
  
  xform.transform_point(start);
  xform.transform_point(end);
  
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
    return nlohmann::ordered_json{
        {"type", "Line"},
        {"guid", guid},
        {"name", name},
        {"x0", _x0},
        {"y0", _y0},
        {"z0", _z0},
        {"x1", _x1},
        {"y1", _y1},
        {"z1", _z1},
        {"width", width},
        {"linecolor", linecolor.jsondump()},
        {"xform", xform.jsondump()}};
}

Line Line::jsonload(const nlohmann::json& data) {
    Line line(data["x0"], data["y0"], data["z0"], data["x1"], data["y1"], data["z1"]);
    line.guid = data["guid"];
    line.name = data["name"];
    line.linecolor = Color::jsonload(data["linecolor"]);
    line.width = data["width"];
    if (data.contains("xform")) {
        line.xform = Xform::jsonload(data["xform"]);
    }
    return line;
}

void Line::json_dump(const std::string& filename) const {
    std::ofstream ofs(filename);
    ofs << jsondump().dump(4);
    ofs.close();
}

Line Line::json_load(const std::string& filename) {
    std::ifstream ifs(filename);
    nlohmann::json data = nlohmann::json::parse(ifs);
    ifs.close();
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_PROTOBUF
std::string Line::to_protobuf() const {
    session_proto::Line proto;
    auto* start = proto.mutable_start();
    start->set_x(_x0);
    start->set_y(_y0);
    start->set_z(_z0);
    auto* end = proto.mutable_end();
    end->set_x(_x1);
    end->set_y(_y1);
    end->set_z(_z1);
    proto.set_guid(guid);
    proto.set_name(name);
    // Serialize xform
    auto* proto_xform = proto.mutable_xform();
    proto_xform->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        proto_xform->add_matrix(xform.m[i]);
    }
    return proto.SerializeAsString();
}

Line Line::from_protobuf(const std::string& data) {
    session_proto::Line proto;
    proto.ParseFromString(data);
    Line line(proto.start().x(), proto.start().y(), proto.start().z(),
              proto.end().x(), proto.end().y(), proto.end().z());
    line.guid = proto.guid();
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

void Line::protobuf_dump(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << to_protobuf();
    ofs.close();
}

Line Line::protobuf_load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
    ifs.close();
    return from_protobuf(data);
}
#else
std::string Line::to_protobuf() const {
    throw std::runtime_error("Protobuf support not enabled");
}

Line Line::from_protobuf(const std::string& data) {
    throw std::runtime_error("Protobuf support not enabled");
}

void Line::protobuf_dump(const std::string& filename) const {
    throw std::runtime_error("Protobuf support not enabled");
}

Line Line::protobuf_load(const std::string& filename) {
    throw std::runtime_error("Protobuf support not enabled");
}
#endif

std::string Line::to_string() const {
    return fmt::format("Line({}, {}, {}, {}, {}, {})", _x0, _y0, _z0, _x1, _y1, _z1);
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
    return to_vector().normalize();
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

Point Line::closest_point(const Point& point) const {
    double dx = _x1 - _x0;
    double dy = _y1 - _y0;
    double dz = _z1 - _z0;
    double len_sq = dx * dx + dy * dy + dz * dz;
    if (len_sq < 1e-20) {
        return start();
    }
    double t = ((point[0] - _x0) * dx + (point[1] - _y0) * dy + (point[2] - _z0) * dz) / len_sq;
    t = std::max(0.0, std::min(1.0, t));
    return point_at(t);
}

Line Line::from_point_and_vector(const Point& point, const Vector& vector) {
    return Line(
        point[0], point[1], point[2],
        point[0] + vector[0], point[1] + vector[1], point[2] + vector[2]
    );
}

Line Line::from_point_direction_length(const Point& point, const Vector& direction, double length) {
    Vector d = direction.normalize();
    return Line(
        point[0], point[1], point[2],
        point[0] + d[0] * length, point[1] + d[1] * length, point[2] + d[2] * length
    );
}

std::ostream& operator<<(std::ostream& os, const Line& line) {
    return os << line.to_string();
}

}  // namespace session_cpp