#include "polyline.h"
#include "boolean_polyline.h"
#include "tolerance.h"
#include <cmath>
#include <sstream>
#include <array>
#include <functional>
#include <limits>
#include <queue>
#include <algorithm>
#include <numeric>

#include "polyline.pb.h"
#include "point.pb.h"
#include "xform.pb.h"

namespace session_cpp {

Polyline::Polyline() : name("my_polyline"), plane(Plane()) {}

/// Copy constructor (creates a new guid() while copying data)
Polyline::Polyline(const Polyline& other)
    :
      name(other.name),
      _coords(other._coords),
      plane(other.plane),
      width(other.width),
      linecolor(other.linecolor),
      xform(other.xform) {}

/// Copy assignment (creates a new guid() while copying data)
Polyline& Polyline::operator=(const Polyline& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        _coords = other._coords;
        plane = other.plane;
        width = other.width;
        linecolor = other.linecolor;
        xform = other.xform;
    }
    return *this;
}

Polyline::Polyline(const std::vector<Point>& pts)
    : name("my_polyline") {
    // Convert points to flat coords
    _coords.reserve(pts.size() * 3);
    for (const auto& p : pts) {
        _coords.push_back(p[0]);
        _coords.push_back(p[1]);
        _coords.push_back(p[2]);
    }
    recompute_plane_if_needed();
}

Polyline Polyline::from_sides(int sides, double radius, bool close) {
    std::vector<Point> pts;
    pts.reserve(close ? sides + 1 : sides);
    for (int i = 0; i < sides; ++i) {
        double angle = 2.0 * Tolerance::PI * i / sides;
        pts.push_back({radius * std::cos(angle), radius * std::sin(angle), 0.0});
    }
    if (close) pts.push_back(pts.front());
    return Polyline(pts);
}

Polyline Polyline::from_coords(const std::vector<double>& coords) {
    Polyline pl;
    pl._coords = coords;
    pl.recompute_plane_if_needed();
    return pl;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Point Access
///////////////////////////////////////////////////////////////////////////////////////////

size_t Polyline::point_count() const {
    return _coords.size() / 3;
}

size_t Polyline::len() const {
    return point_count();
}

std::vector<Point> Polyline::get_points() const {
    std::vector<Point> points;
    points.reserve(point_count());
    for (size_t i = 0; i < point_count(); i++) {
        size_t idx = i * 3;
        points.emplace_back(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
    }
    return points;
}

bool Polyline::is_empty() const {
    return _coords.empty();
}

size_t Polyline::segment_count() const {
    size_t n = point_count();
    return n > 1 ? n - 1 : 0;
}

std::vector<Line> Polyline::get_lines() const {
    std::vector<Line> result;
    result.reserve(segment_count());
    for (size_t i = 0; i < segment_count(); i++) {
        size_t idx0 = i * 3;
        size_t idx1 = (i + 1) * 3;
        result.emplace_back(_coords[idx0], _coords[idx0 + 1], _coords[idx0 + 2],
                           _coords[idx1], _coords[idx1 + 1], _coords[idx1 + 2]);
    }
    return result;
}

double Polyline::length() const {
    double total_length = 0.0;
    for (size_t i = 0; i < segment_count(); i++) {
        size_t idx0 = i * 3;
        size_t idx1 = (i + 1) * 3;
        double dx = _coords[idx1] - _coords[idx0];
        double dy = _coords[idx1 + 1] - _coords[idx0 + 1];
        double dz = _coords[idx1 + 2] - _coords[idx0 + 2];
        total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return total_length;
}

Point Polyline::get_point(size_t index) const {
    if (index < point_count()) {
        size_t idx = index * 3;
        return Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
    }
    return Point(0, 0, 0);
}

void Polyline::set_point(size_t index, const Point& point) {
    if (index < point_count()) {
        size_t idx = index * 3;
        _coords[idx] = point[0];
        _coords[idx + 1] = point[1];
        _coords[idx + 2] = point[2];
    }
}

void Polyline::add_point(const Point& point) {
    _coords.push_back(point[0]);
    _coords.push_back(point[1]);
    _coords.push_back(point[2]);
    if (point_count() == 3) {
        recompute_plane_if_needed();
    }
}

void Polyline::insert_point(size_t index, const Point& point) {
    if (index <= point_count()) {
        size_t idx = index * 3;
        _coords.insert(_coords.begin() + idx, {point[0], point[1], point[2]});
        if (point_count() == 3) {
            recompute_plane_if_needed();
        }
    }
}

bool Polyline::remove_point(size_t index, Point& out_point) {
    if (index < point_count()) {
        size_t idx = index * 3;
        out_point = Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
        _coords.erase(_coords.begin() + idx, _coords.begin() + idx + 3);
        if (point_count() == 3) {
            recompute_plane_if_needed();
        }
        return true;
    }
    return false;
}

void Polyline::reverse() {
    // Reverse coords in groups of 3
    size_t n = point_count();
    std::vector<double> new_coords;
    new_coords.reserve(_coords.size());
    for (size_t i = n; i > 0; i--) {
        size_t idx = (i - 1) * 3;
        new_coords.push_back(_coords[idx]);
        new_coords.push_back(_coords[idx + 1]);
        new_coords.push_back(_coords[idx + 2]);
    }
    _coords = std::move(new_coords);
    plane.reverse();
}

Polyline Polyline::reversed() const {
    Polyline result = *this;
    result.reverse();
    return result;
}

void Polyline::recompute_plane_if_needed() {
    _plane_dirty = true; // defer computation to get_plane()
}

const Plane& Polyline::get_plane() const {
    if (_plane_dirty && point_count() >= 3) {
        size_t n = point_count();
        Point p0(_coords[0], _coords[1], _coords[2]);
        bool found = false;
        for (size_t i = 1; i < n && !found; i++) {
            Vector v1(_coords[i*3]-p0[0], _coords[i*3+1]-p0[1], _coords[i*3+2]-p0[2]);
            if (v1.magnitude_squared() < 1e-20) continue;
            for (size_t j = i + 1; j < n; j++) {
                Vector v2(_coords[j*3]-p0[0], _coords[j*3+1]-p0[1], _coords[j*3+2]-p0[2]);
                Vector normal = v1.cross(v2);
                if (normal.magnitude_squared() < 1e-20) continue;
                normal.normalize_self();
                v1.normalize_self();
                Vector yax = normal.cross(v1);
                yax.normalize_self();
                plane = Plane(p0, v1, yax, normal);
                found = true;
                break;
            }
        }
        if (!found) plane = Plane();
        _plane_dirty = false;
    }
    return plane;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////////////////

Polyline& Polyline::operator+=(const Vector& v) {
    for (size_t i = 0; i < point_count(); i++) {
        size_t idx = i * 3;
        _coords[idx] += v[0];
        _coords[idx + 1] += v[1];
        _coords[idx + 2] += v[2];
    }
    // Update plane origin
    Point new_origin = plane.origin() + v;
    Vector x = plane.x_axis();
    Vector y = plane.y_axis();
    plane = Plane(new_origin, x, y);
    return *this;
}

Polyline Polyline::operator+(const Vector& v) const {
    Polyline result = *this;
    result += v;
    return result;
}

Polyline& Polyline::operator-=(const Vector& v) {
    for (size_t i = 0; i < point_count(); i++) {
        size_t idx = i * 3;
        _coords[idx] -= v[0];
        _coords[idx + 1] -= v[1];
        _coords[idx + 2] -= v[2];
    }
    // Update plane origin
    Point new_origin = plane.origin() - v;
    Vector x = plane.x_axis();
    Vector y = plane.y_axis();
    plane = Plane(new_origin, x, y);
    return *this;
}

Polyline Polyline::operator-(const Vector& v) const {
    Polyline result = *this;
    result -= v;
    return result;
}

Polyline& Polyline::operator*=(double factor) {
    for (size_t i = 0; i < _coords.size(); i++) {
        _coords[i] *= factor;
    }
    return *this;
}

Polyline Polyline::operator*(double factor) const {
    Polyline result = *this;
    result *= factor;
    return result;
}

Polyline& Polyline::operator/=(double factor) {
    for (size_t i = 0; i < _coords.size(); i++) {
        _coords[i] /= factor;
    }
    return *this;
}

Polyline Polyline::operator/(double factor) const {
    Polyline result = *this;
    result /= factor;
    return result;
}

Polyline Polyline::operator-() const {
    return reversed();
}

Point Polyline::operator[](size_t index) const {
    if (index >= point_count()) {
        throw std::out_of_range("Index out of range");
    }
    size_t idx = index * 3;
    return Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void Polyline::transform() {
    for (size_t i = 0; i < point_count(); i++) {
        size_t idx = i * 3;
        Point pt(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
        pt.xform = xform; pt.transform();
        _coords[idx] = pt[0];
        _coords[idx + 1] = pt[1];
        _coords[idx + 2] = pt[2];
    }
    xform = Xform::identity();
}

Polyline Polyline::transformed() const {
    Polyline result = *this;
    result.transform();
    return result;
}

Polyline Polyline::transformed_xform(const Xform& xf) const {
    // Verbatim port of `xform_polyline()` from main_5.cpp. Applies a
    // column-major affine transformation matrix.
    const auto& M = xf.m;
    std::vector<Point> pts;
    pts.reserve(point_count());
    for (size_t i = 0; i < point_count(); i++) {
        Point p = get_point(i);
        double x = M[0]*p[0] + M[4]*p[1] + M[8]*p[2]  + M[12];
        double y = M[1]*p[0] + M[5]*p[1] + M[9]*p[2]  + M[13];
        double z = M[2]*p[0] + M[6]*p[1] + M[10]*p[2] + M[14];
        pts.emplace_back(x, y, z);
    }
    return Polyline(pts);
}

void Polyline::translate(const Vector& v) {
    // Verbatim of `move_polyline()` from main_5.cpp.
    std::vector<Point> pts;
    pts.reserve(point_count());
    for (size_t i = 0; i < point_count(); i++) {
        Point p = get_point(i);
        pts.emplace_back(p[0]+v[0], p[1]+v[1], p[2]+v[2]);
    }
    *this = Polyline(pts);
}

void Polyline::extend_edge_equally(size_t edge_idx, double distance) {
    // Verbatim of `extend_polyline_edge_equally()` from main_5.cpp:803.
    size_t n = point_count();
    if (n < 2 || edge_idx + 1 >= n) return;
    size_t i = edge_idx;
    size_t j = edge_idx + 1;
    Point pi = get_point(i);
    Point pj = get_point(j);
    double dx = pj[0] - pi[0];
    double dy = pj[1] - pi[1];
    double dz = pj[2] - pi[2];
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len < 1e-12) return;
    double inv = 1.0 / len;
    double ux = dx * inv * distance;
    double uy = dy * inv * distance;
    double uz = dz * inv * distance;
    Point new_pi(pi[0]-ux, pi[1]-uy, pi[2]-uz);
    Point new_pj(pj[0]+ux, pj[1]+uy, pj[2]+uz);
    set_point(i, new_pi);
    set_point(j, new_pj);
    if (i == 0)        set_point(n - 1, new_pi);
    if (j == n - 1)    set_point(0,     new_pj);
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Polyline::jsondump() const {
    // Alphabetical order to match Rust's serde_json
    nlohmann::ordered_json j;
    j["coords"] = _coords;
    j["guid"] = guid();
    j["linecolor"] = linecolor.jsondump();
    j["name"] = name;
    j["type"] = "Polyline";
    j["width"] = width;
    j["xform"] = xform.jsondump();
    return j;
}

Polyline Polyline::jsonload(const nlohmann::json& data) {
    Polyline polyline;
    polyline.guid() = data["guid"];
    polyline.name = data["name"];

    // Support both new coords format and legacy points format
    if (data.contains("coords")) {
        polyline._coords = data["coords"].get<std::vector<double>>();
    } else if (data.contains("points")) {
        // Legacy format with full Point objects
        for (const auto& pt_json : data["points"]) {
            Point pt = Point::jsonload(pt_json);
            polyline._coords.push_back(pt[0]);
            polyline._coords.push_back(pt[1]);
            polyline._coords.push_back(pt[2]);
        }
    }

    if (data.contains("width")) {
        polyline.width = data["width"];
    }
    if (data.contains("linecolor")) {
        polyline.linecolor = Color::jsonload(data["linecolor"]);
    }
    if (data.contains("xform")) {
        polyline.xform = Xform::jsonload(data["xform"]);
    }

    polyline.recompute_plane_if_needed();
    return polyline;
}

std::string Polyline::json_dumps() const {
    return jsondump().dump();
}

Polyline Polyline::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Polyline::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(2);
}

Polyline Polyline::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

std::string Polyline::pb_dumps() const {
    session_proto::Polyline proto;
    proto.set_guid(this->guid());
    proto.set_name(this->name);
    proto.set_width(this->width);

    // Add coords as flat array [x0, y0, z0, x1, y1, z1, ...]
    for (double c : _coords) {
        proto.add_coords(c);
    }

    // Set linecolor
    auto* color_proto = proto.mutable_linecolor();
    color_proto->set_r(linecolor.r);
    color_proto->set_g(linecolor.g);
    color_proto->set_b(linecolor.b);
    color_proto->set_a(linecolor.a);
    color_proto->set_name(linecolor.name);

    // Set xform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }

    return proto.SerializeAsString();
}

Polyline Polyline::pb_loads(const std::string& data) {
    session_proto::Polyline proto;
    proto.ParseFromString(data);

    // Read coords as flat array [x0, y0, z0, x1, y1, z1, ...]
    std::vector<double> coords(proto.coords().begin(), proto.coords().end());

    Polyline pl = Polyline::from_coords(coords);
    pl.guid() = proto.guid();
    pl.name = proto.name();
    pl.width = proto.width();

    // Load linecolor
    if (proto.has_linecolor()) {
        const auto& c = proto.linecolor();
        pl.linecolor = Color(c.r(), c.g(), c.b(), c.a(), c.name());
    }

    // Load xform
    if (proto.has_xform()) {
        const auto& xform_proto = proto.xform();
        pl.xform.name = xform_proto.name();
        for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i) {
            pl.xform.m[i] = xform_proto.matrix(i);
        }
    }

    return pl;
}

void Polyline::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Polyline Polyline::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

std::string Polyline::str() const {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < point_count(); i++) {
        if (i > 0) oss << ", ";
        size_t idx = i * 3;
        oss << "(" << _coords[idx] << ", " << _coords[idx + 1] << ", " << _coords[idx + 2] << ")";
    }
    oss << "]";
    return oss.str();
}

std::string Polyline::repr() const {
    return "Polyline(" + name + ", " + std::to_string(point_count()) + " points)";
}

bool Polyline::operator==(const Polyline& other) const {
    if (name != other.name) return false;
    if (point_count() != other.point_count()) return false;
    for (size_t i = 0; i < _coords.size(); i++) {
        if (std::round(_coords[i] * 1000000.0) != std::round(other._coords[i] * 1000000.0)) {
            return false;
        }
    }
    if (std::round(width * 1000000.0) != std::round(other.width * 1000000.0)) return false;
    if (!(linecolor == other.linecolor)) return false;
    return true;
}

bool Polyline::operator!=(const Polyline& other) const {
    return !(*this == other);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Geometric Utilities
///////////////////////////////////////////////////////////////////////////////////////////

void Polyline::shift(int times) {
    if (_coords.empty()) return;

    // Remove last point if closed
    bool was_closed = is_closed();
    if (was_closed && point_count() > 0) {
        _coords.resize(_coords.size() - 3);
    }

    // Perform rotation
    size_t n = point_count();
    if (n > 0 && times != 0) {
        times = times % static_cast<int>(n);
        if (times < 0) times += static_cast<int>(n);

        // Rotate coords in groups of 3
        std::vector<double> new_coords;
        new_coords.reserve(_coords.size());
        for (size_t i = 0; i < n; i++) {
            size_t src_idx = ((i + times) % n) * 3;
            new_coords.push_back(_coords[src_idx]);
            new_coords.push_back(_coords[src_idx + 1]);
            new_coords.push_back(_coords[src_idx + 2]);
        }
        _coords = std::move(new_coords);
    }

    // Restore closure if it was closed
    if (was_closed && point_count() > 0) {
        _coords.push_back(_coords[0]);
        _coords.push_back(_coords[1]);
        _coords.push_back(_coords[2]);
    }
}

double Polyline::length_squared() const {
    double len = 0.0;
    for (size_t i = 0; i < segment_count(); i++) {
        size_t idx0 = i * 3;
        size_t idx1 = (i + 1) * 3;
        double dx = _coords[idx1] - _coords[idx0];
        double dy = _coords[idx1 + 1] - _coords[idx0 + 1];
        double dz = _coords[idx1 + 2] - _coords[idx0 + 2];
        len += dx * dx + dy * dy + dz * dz;
    }
    return len;
}

Point Polyline::point_at(const Point& start, const Point& end, double t) {
    const double s = 1.0 - t;
    return Point(
        (start[0] == end[0]) ? start[0] : static_cast<double>(s * start[0] + t * end[0]),
        (start[1] == end[1]) ? start[1] : static_cast<double>(s * start[1] + t * end[1]),
        (start[2] == end[2]) ? start[2] : static_cast<double>(s * start[2] + t * end[2])
    );
}

void Polyline::closest_point_to_line(const Point& point, const Point& line_start, 
                                    const Point& line_end, double& t) {
    Vector D = line_end - line_start;
    double DoD = D.magnitude_squared();

    if (DoD > 0.0) {
        Vector to_point_start = point - line_start;
        Vector to_point_end = point - line_end;
        
        if (to_point_start.magnitude_squared() <= to_point_end.magnitude_squared()) {
            t = to_point_start.dot(D) / DoD;
        } else {
            t = 1.0 + to_point_end.dot(D) / DoD;
        }
    } else {
        t = 0.0;
    }
}

bool Polyline::line_line_overlap(const Point& line0_start, const Point& line0_end,
                                const Point& line1_start, const Point& line1_end,
                                Point& overlap_start, Point& overlap_end) {
    double t[4];
    t[0] = 0.0;
    t[1] = 1.0;

    closest_point_to_line(line1_start, line0_start, line0_end, t[2]);
    closest_point_to_line(line1_end, line0_start, line0_end, t[3]);

    // Check if there is an overlap
    bool do_overlap = !((t[2] < 0 && t[3] < 0) || (t[2] > 1 && t[3] > 1));

    // Sort parameters
    std::sort(t, t + 4);

    // Check if the overlap is not just a point
    do_overlap = do_overlap && (std::abs(t[2] - t[1]) > Tolerance::ZERO_TOLERANCE);

    // Get overlap points
    overlap_start = point_at(line0_start, line0_end, t[1]);
    overlap_end = point_at(line0_start, line0_end, t[2]);

    return do_overlap;
}

void Polyline::line_line_average(const Point& line0_start, const Point& line0_end,
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

void Polyline::line_line_overlap_average(const Point& line0_start, const Point& line0_end,
                                        const Point& line1_start, const Point& line1_end,
                                        Point& output_start, Point& output_end) {
    // Get two overlaps
    Point lineA_start, lineA_end;
    line_line_overlap(line0_start, line0_end, line1_start, line1_end, lineA_start, lineA_end);
    
    Point lineB_start, lineB_end;
    line_line_overlap(line1_start, line1_end, line0_start, line0_end, lineB_start, lineB_end);

    // Construct middle lines, in case the first one is flipped
    Point mid_line0_start(
        (lineA_start[0] + lineB_start[0]) * 0.5,
        (lineA_start[1] + lineB_start[1]) * 0.5,
        (lineA_start[2] + lineB_start[2]) * 0.5
    );
    Point mid_line0_end(
        (lineA_end[0] + lineB_end[0]) * 0.5,
        (lineA_end[1] + lineB_end[1]) * 0.5,
        (lineA_end[2] + lineB_end[2]) * 0.5
    );
    
    Point mid_line1_start(
        (lineA_start[0] + lineB_end[0]) * 0.5,
        (lineA_start[1] + lineB_end[1]) * 0.5,
        (lineA_start[2] + lineB_end[2]) * 0.5
    );
    Point mid_line1_end(
        (lineA_end[0] + lineB_start[0]) * 0.5,
        (lineA_end[1] + lineB_start[1]) * 0.5,
        (lineA_end[2] + lineB_start[2]) * 0.5
    );

    // The diagonal is always longer, so return the longer
    Vector mid0_vec = mid_line0_end - mid_line0_start;
    Vector mid1_vec = mid_line1_end - mid_line1_start;
    
    if (mid0_vec.magnitude_squared() > mid1_vec.magnitude_squared()) {
        output_start = mid_line0_start;
        output_end = mid_line0_end;
    } else {
        output_start = mid_line1_start;
        output_end = mid_line1_end;
    }
}

bool Polyline::line_from_projected_points(const Point& line_start, const Point& line_end,
                                         const std::vector<Point>& points,
                                         Point& output_start, Point& output_end) {
    if (points.empty()) return false;
    
    std::vector<double> t_values;
    t_values.reserve(points.size());

    // Project all points to the line
    for (const auto& point : points) {
        double t;
        closest_point_to_line(point, line_start, line_end, t);
        t_values.push_back(t);
    }

    // Sort parameters
    std::sort(t_values.begin(), t_values.end());

    // Output first and last points
    output_start = point_at(line_start, line_end, t_values.front());
    output_end = point_at(line_start, line_end, t_values.back());

    // Check if not just a point
    return std::abs(t_values.front() - t_values.back()) > Tolerance::ZERO_TOLERANCE;
}

double Polyline::closest_distance_and_point(const Point& point, size_t& edge_id, Point& closest_point) const {
    edge_id = 0;
    double closest_distance = std::numeric_limits<double>::max();
    double best_t = 0.0;

    for (size_t i = 0; i < segment_count(); i++) {
        double t;
        Point pi = get_point(i);
        Point pi1 = get_point(i + 1);
        closest_point_to_line(point, pi, pi1, t);

        Point point_on_segment = point_at(pi, pi1, t);
        double distance = point.distance(point_on_segment);

        if (distance < closest_distance) {
            closest_distance = distance;
            edge_id = i;
            best_t = t;
        }

        if (closest_distance < Tolerance::ZERO_TOLERANCE) {
            break;
        }
    }

    closest_point = point_at(get_point(edge_id), get_point(edge_id + 1), best_t);
    return closest_distance;
}

bool Polyline::is_closed() const {
    if (point_count() < 2) return false;
    return get_point(0).distance(get_point(point_count() - 1)) < static_cast<double>(Tolerance::ZERO_TOLERANCE);
}

Polyline Polyline::closed() const {
    if (is_closed()) return Polyline::from_coords(_coords);
    std::vector<double> new_coords(_coords);
    new_coords.push_back(_coords[0]);
    new_coords.push_back(_coords[1]);
    new_coords.push_back(_coords[2]);
    return Polyline::from_coords(new_coords);
}

void Polyline::merge_collinear(double tol) {
    bool closed = is_closed();
    std::vector<Point> pts = get_points();
    if (closed && pts.size() > 1) pts.pop_back();
    const double zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
    bool changed = true;
    while (changed) {
        changed = false;
        int m = (int)pts.size();
        if (m < 3) break;
        std::vector<Point> out;
        for (int i = 0; i < m; i++) {
            int p = (i - 1 + m) % m, nx = (i + 1) % m;
            if (!closed && (i == 0 || i == m - 1)) { out.push_back(pts[i]); continue; }
            double ax=pts[i][0]-pts[p][0], ay=pts[i][1]-pts[p][1], az=pts[i][2]-pts[p][2];
            double bx=pts[nx][0]-pts[i][0], by=pts[nx][1]-pts[i][1], bz=pts[nx][2]-pts[i][2];
            double cx=ay*bz-az*by, cy=az*bx-ax*bz, cz=ax*by-ay*bx;
            double a2=ax*ax+ay*ay+az*az, b2=bx*bx+by*by+bz*bz;
            if (a2<zt2||b2<zt2||cx*cx+cy*cy+cz*cz<tol*tol*a2*b2) changed=true;
            else out.push_back(pts[i]);
        }
        pts = out;
    }
    _coords.clear();
    for (const auto& p : pts) { _coords.push_back(p[0]); _coords.push_back(p[1]); _coords.push_back(p[2]); }
    if (closed && !pts.empty()) { _coords.push_back(pts[0][0]); _coords.push_back(pts[0][1]); _coords.push_back(pts[0][2]); }
    recompute_plane_if_needed();
}

Point Polyline::center() const {
    if (_coords.empty()) return Point(0, 0, 0);

    double x = 0, y = 0, z = 0;
    size_t n = is_closed() ? point_count() - 1 : point_count();

    for (size_t i = 0; i < n; i++) {
        size_t idx = i * 3;
        x += _coords[idx];
        y += _coords[idx + 1];
        z += _coords[idx + 2];
    }

    x /= n;
    y /= n;
    z /= n;

    return Point(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
}

void Polyline::get_average_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) const {
    // Origin
    origin = center();

    // X-axis (first segment direction)
    if (point_count() >= 2) {
        x_axis = get_point(1) - get_point(0);
        x_axis.normalize_self();
    } else {
        x_axis = Vector(1, 0, 0);
    }

    // Z-axis (average normal)
    average_normal(z_axis);

    // Y-axis (cross product)
    y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();
}

bool Polyline::point_in_polygon_2d(const Point& p) const {
    double px = p[0], py = p[1];
    int winding = 0;
    size_t n = _coords.size() / 3;
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double y0 = _coords[i*3+1], y1 = _coords[j*3+1];
        if (y0 <= py) {
            if (y1 > py) {
                double x0 = _coords[i*3], x1 = _coords[j*3];
                if ((x1-x0)*(py-y0) - (px-x0)*(y1-y0) > 0.0) ++winding;
            }
        } else if (y1 <= py) {
            double x0 = _coords[i*3], x1 = _coords[j*3];
            if ((x1-x0)*(py-y0) - (px-x0)*(y1-y0) < 0.0) --winding;
        }
    }
    return winding != 0;
}

void Polyline::get_fast_plane(Point& origin, Plane& pln) const {
    if (_coords.empty()) {
        origin = Point(0, 0, 0);
        pln = Plane();
        return;
    }

    origin = get_point(0);

    Vector normal;
    average_normal(normal);

    // Create plane from point and normal
    pln = Plane::from_point_normal(origin, normal);
}

void Polyline::extend_segment(int segment_id, double dist0, double dist1,
                             double proportion0, double proportion1) {
    if (segment_id < 0 || segment_id >= static_cast<int>(segment_count())) return;
    if (dist0 == 0 && dist1 == 0 && proportion0 == 0 && proportion1 == 0) return;

    // Cache closed-ness BEFORE the mutations below. The two `set_point`
    // calls move either pts[0] (when segment_id==0) or pts[last] (when
    // segment_id+1==last) in isolation; the other endpoint is still at
    // its old value, so a post-mutation `is_closed()` would falsely
    // report "open" (distance ≥ extension amount ≫ ZERO_TOLERANCE) and
    // the closing-duplicate sync below would be skipped — leaving the
    // polyline's closing vertex stale. Wood's
    // `cgal::polyline_util::extend` (`cgal_polyline_util.cpp:405-435`)
    // does the sync unconditionally based on `sID`; caching here gives
    // the same semantics without losing the closed-polyline precondition.
    const bool was_closed = is_closed();

    Point p0 = get_point(segment_id);
    Point p1 = get_point(segment_id + 1);
    Vector v = p1 - p0;

    if (proportion0 != 0 || proportion1 != 0) {
        p0 = p0 - v * static_cast<double>(proportion0);
        p1 = p1 + v * static_cast<double>(proportion1);
    } else {
        v.normalize_self();
        p0 = p0 - v * static_cast<double>(dist0);
        p1 = p1 + v * static_cast<double>(dist1);
    }

    set_point(segment_id, p0);
    set_point(segment_id + 1, p1);

    if (was_closed) {
        if (segment_id == 0) {
            set_point(point_count() - 1, get_point(0));
        } else if ((segment_id + 1) == static_cast<int>(point_count() - 1)) {
            set_point(0, get_point(point_count() - 1));
        }
    }
}

void Polyline::extend_segment_equally(Point& segment_start, Point& segment_end, double dist, double proportion) {
    if (dist == 0 && proportion == 0) return;

    Vector v = segment_end - segment_start;

    if (proportion != 0) {
        segment_start = segment_start - v * static_cast<double>(proportion);
        segment_end = segment_end + v * static_cast<double>(proportion);
    } else {
        v.normalize_self();
        segment_start = segment_start - v * static_cast<double>(dist);
        segment_end = segment_end + v * static_cast<double>(dist);
    }
}

void Polyline::extend_segment_equally(int segment_id, double dist, double proportion) {
    if (segment_id < 0 || segment_id >= static_cast<int>(segment_count())) return;

    Point p0 = get_point(segment_id);
    Point p1 = get_point(segment_id + 1);
    extend_segment_equally(p0, p1, dist, proportion);
    set_point(segment_id, p0);
    set_point(segment_id + 1, p1);

    // Handle closed polylines
    if (point_count() > 2 && is_closed()) {
        if (segment_id == 0) {
            set_point(point_count() - 1, get_point(0));
        } else if ((segment_id + 1) == static_cast<int>(point_count() - 1)) {
            set_point(0, get_point(point_count() - 1));
        }
    }
}

void Polyline::extend_line_segment(Point& start, Point& end, double d0, double d1) {
    Vector v = end - start;
    v.normalize_self();
    start = start - v * d0;
    end = end + v * d1;
}

void Polyline::shrink_line_segment(Point& start, Point& end, double dist) {
    Vector v = end - start;
    start = start + v * dist;
    end = end - v * dist;
}

bool Polyline::is_clockwise(const Plane& pln) const {
    size_t n = point_count();
    if (n < 3) return false;

    // Project onto plane's local XY axes and compute shoelace signed area.
    // Matches wood's cgal_polyline_util::is_clockwise which uses plane_to_xy.
    const Vector& xv = pln.x_axis();
    const Vector& yv = pln.y_axis();
    const Point& orig = pln.origin();

    // For closed polylines the last point duplicates the first; iterate n-1
    // unique edges. For open polylines iterate n edges (last→first closes it).
    size_t lim = is_closed() ? n - 1 : n;

    double area = 0.0;
    for (size_t i = 0; i < lim; i++) {
        Point pi  = get_point(i);
        Point pi1 = get_point((i + 1) % lim);
        double u0 = (pi[0]-orig[0])*xv[0]  + (pi[1]-orig[1])*xv[1]  + (pi[2]-orig[2])*xv[2];
        double v0 = (pi[0]-orig[0])*yv[0]  + (pi[1]-orig[1])*yv[1]  + (pi[2]-orig[2])*yv[2];
        double u1 = (pi1[0]-orig[0])*xv[0] + (pi1[1]-orig[1])*xv[1] + (pi1[2]-orig[2])*xv[2];
        double v1 = (pi1[0]-orig[0])*yv[0] + (pi1[1]-orig[1])*yv[1] + (pi1[2]-orig[2])*yv[2];
        area += (u1 - u0) * (v1 + v0);
    }
    return area > 0;
}

void Polyline::get_convex_corners(std::vector<bool>& convex_or_concave) const {
    if (point_count() < 3) return;

    bool closed = is_closed();
    size_t n = closed ? point_count() - 1 : point_count();

    Vector normal;
    average_normal(normal);

    convex_or_concave.clear();
    convex_or_concave.reserve(n);

    for (size_t current = 0; current < n; current++) {
        size_t prev = (current == 0) ? n - 1 : current - 1;
        size_t next = (current == n - 1) ? 0 : current + 1;

        Vector dir0 = get_point(current) - get_point(prev);
        dir0.normalize_self();

        Vector dir1 = get_point(next) - get_point(current);
        dir1.normalize_self();

        Vector cross = dir0.cross(dir1);
        cross.normalize_self();

        double dot = cross.dot(normal);
        bool is_convex = !(dot < 0.0);
        convex_or_concave.push_back(is_convex);
    }
}

Polyline Polyline::tween_two_polylines(const Polyline& polyline0, const Polyline& polyline1, double weight) {
    if (polyline0.point_count() != polyline1.point_count()) {
        // Return first polyline if sizes don't match
        return polyline0;
    }

    Polyline result;
    result._coords.reserve(polyline0._coords.size());

    for (size_t i = 0; i < polyline0.point_count(); i++) {
        Point p0 = polyline0.get_point(i);
        Point p1 = polyline1.get_point(i);
        Vector diff = p1 - p0;
        Point interpolated = p0 + diff * static_cast<double>(weight);
        result.add_point(interpolated);
    }

    return result;
}

void Polyline::average_normal(Vector& avg_normal) const {
    size_t n = point_count();
    if (n < 3) {
        avg_normal = Vector(0, 0, 1);
        return;
    }

    // Check if closed
    bool closed = (get_point(0).distance(get_point(n - 1)) < static_cast<double>(Tolerance::ZERO_TOLERANCE));
    if (closed && n > 1) n = n - 1;

    avg_normal = Vector(0, 0, 0);

    for (size_t i = 0; i < n; i++) {
        size_t prev = (i == 0) ? n - 1 : i - 1;
        size_t next = (i + 1) % n;

        Vector v1 = get_point(i) - get_point(prev);
        Vector v2 = get_point(next) - get_point(i);
        Vector cross = v1.cross(v2);
        avg_normal += cross;
    }
    avg_normal.normalize_self();
}

std::vector<Point> Polyline::interpolate_points(const Point& from, const Point& to, int steps, int kind) {
    std::vector<Point> pts;
    if (kind == 1 || kind == 2) {
        pts.push_back(from);
    }
    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(steps + 1);
        pts.push_back(Point(
            from[0] + t * (to[0] - from[0]),
            from[1] + t * (to[1] - from[1]),
            from[2] + t * (to[2] - from[2])
        ));
    }
    if (kind == 1) {
        pts.push_back(to);
    }
    return pts;
}

Polyline Polyline::quick_hull(const Polyline& polygon) {
    Point origin;
    Vector xa, ya, za;
    polygon.get_average_plane(origin, xa, ya, za);
    std::vector<Point> pts = polygon.get_points();

    std::vector<std::array<double,2>> pts2d;
    pts2d.reserve(pts.size());
    for (const auto& p : pts) {
        double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
        pts2d.push_back({dx*xa[0]+dy*xa[1]+dz*xa[2], dx*ya[0]+dy*ya[1]+dz*ya[2]});
    }

    auto ccw2d = [](double ax, double ay, double bx, double by, double px, double py) -> double {
        return (bx-ax)*(py-ay) - (by-ay)*(px-ax);
    };

    std::function<void(const std::vector<std::array<double,2>>&, double, double, double, double,
                       std::vector<std::array<double,2>>&)> qh_recurse;
    qh_recurse = [&](const std::vector<std::array<double,2>>& v,
                     double ax, double ay, double bx, double by,
                     std::vector<std::array<double,2>>& hull) {
        if (v.empty()) return;
        size_t fi = 0;
        double best = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < v.size(); i++) {
            double val = ccw2d(ax, ay, bx, by, v[i][0], v[i][1]);
            if (val >= best) { best = val; fi = i; }
        }
        double fx = v[fi][0], fy = v[fi][1];
        std::vector<std::array<double,2>> left, right;
        for (const auto& p : v) {
            if (ccw2d(ax, ay, fx, fy, p[0], p[1]) > 0.0) left.push_back(p);
        }
        qh_recurse(left, ax, ay, fx, fy, hull);
        hull.push_back({fx, fy});
        for (const auto& p : v) {
            if (ccw2d(fx, fy, bx, by, p[0], p[1]) > 0.0) right.push_back(p);
        }
        qh_recurse(right, fx, fy, bx, by, hull);
    };

    size_t ai = 0, bi = 0;
    for (size_t i = 1; i < pts2d.size(); i++) {
        if (pts2d[i][0] < pts2d[ai][0]) ai = i;
        if (pts2d[i][0] >= pts2d[bi][0]) bi = i;
    }
    double ax = pts2d[ai][0], ay = pts2d[ai][1];
    double bx = pts2d[bi][0], by = pts2d[bi][1];

    std::vector<std::array<double,2>> left_pts, right_pts;
    for (const auto& p : pts2d) {
        if (ccw2d(ax, ay, bx, by, p[0], p[1]) > 0.0) left_pts.push_back(p);
        else right_pts.push_back(p);
    }
    std::vector<std::array<double,2>> hull;
    hull.push_back({ax, ay});
    qh_recurse(left_pts, ax, ay, bx, by, hull);
    hull.push_back({bx, by});
    qh_recurse(right_pts, bx, by, ax, ay, hull);

    std::vector<Point> pts3d;
    pts3d.reserve(hull.size());
    for (const auto& h : hull) {
        pts3d.emplace_back(
            origin[0] + h[0]*xa[0] + h[1]*ya[0],
            origin[1] + h[0]*xa[1] + h[1]*ya[1],
            origin[2] + h[0]*xa[2] + h[1]*ya[2]
        );
    }
    return Polyline(pts3d);
}

std::optional<Polyline> Polyline::bounding_rectangle(const Polyline& polygon) {
    Polyline hull = quick_hull(polygon);
    if (hull.point_count() <= 2) return std::nullopt;
    Point origin;
    Vector xa, ya, za;
    polygon.get_average_plane(origin, xa, ya, za);

    std::vector<Point> hull_pts = hull.get_points();
    std::vector<std::array<double,2>> hull2d;
    hull2d.reserve(hull_pts.size());
    for (const auto& p : hull_pts) {
        double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
        hull2d.push_back({dx*xa[0]+dy*xa[1]+dz*xa[2], dx*ya[0]+dy*ya[1]+dz*ya[2]});
    }

    double best_area = std::numeric_limits<double>::max();
    double best_min_u=0, best_max_u=0, best_min_v=0, best_max_v=0, best_angle=0;
    size_t hn = hull2d.size();
    for (size_t i = 0; i < hn; i++) {
        size_t j = (i + 1) % hn;
        double ex = hull2d[j][0]-hull2d[i][0], ey = hull2d[j][1]-hull2d[i][1];
        double len = std::sqrt(ex*ex + ey*ey);
        if (len < 1e-12) continue;
        double ca = ex/len, sa = ey/len;
        double min_u = std::numeric_limits<double>::max(), max_u = -std::numeric_limits<double>::max();
        double min_v = std::numeric_limits<double>::max(), max_v = -std::numeric_limits<double>::max();
        for (const auto& h : hull2d) {
            double u =  h[0]*ca + h[1]*sa;
            double v = -h[0]*sa + h[1]*ca;
            min_u = std::min(min_u, u); max_u = std::max(max_u, u);
            min_v = std::min(min_v, v); max_v = std::max(max_v, v);
        }
        double area = (max_u-min_u)*(max_v-min_v);
        if (area < best_area) {
            best_area = area;
            best_min_u = min_u; best_max_u = max_u;
            best_min_v = min_v; best_max_v = max_v;
            best_angle = std::atan2(ey, ex);
        }
    }

    double ca = std::cos(best_angle), sa = std::sin(best_angle);
    auto rot_back = [&](double u, double v) -> std::array<double,2> {
        return {u*ca - v*sa, u*sa + v*ca};
    };
    auto to3d = [&](double u2, double v2) -> Point {
        return Point(
            origin[0] + u2*xa[0] + v2*ya[0],
            origin[1] + u2*xa[1] + v2*ya[1],
            origin[2] + u2*xa[2] + v2*ya[2]
        );
    };
    auto c0 = rot_back(best_min_u, best_min_v);
    auto c1 = rot_back(best_min_u, best_max_v);
    auto c2 = rot_back(best_max_u, best_max_v);
    auto c3 = rot_back(best_max_u, best_min_v);
    std::vector<Point> pts3d = {
        to3d(c0[0], c0[1]),
        to3d(c1[0], c1[1]),
        to3d(c2[0], c2[1]),
        to3d(c3[0], c3[1]),
        to3d(c0[0], c0[1]),
    };
    return Polyline(pts3d);
}

std::vector<Point> Polyline::grid_of_points_in_polygon(const Polyline& polygon,
                                                       double offset_dist, double div_dist,
                                                       size_t max_pts) {
    if (div_dist < 1e-12) return {};
    Point origin;
    Vector xa, ya, za;
    polygon.get_average_plane(origin, xa, ya, za);
    std::vector<Point> pts = polygon.get_points();

    size_t last = pts.size();
    if (last > 1) {
        const Point& a = pts[0];
        const Point& b = pts.back();
        if (std::abs(a[0]-b[0])<1e-10 && std::abs(a[1]-b[1])<1e-10 && std::abs(a[2]-b[2])<1e-10)
            last--;
    }

    std::vector<std::array<double,2>> poly2d;
    poly2d.reserve(last);
    for (size_t i = 0; i < last; i++) {
        const Point& p = pts[i];
        double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
        poly2d.push_back({dx*xa[0]+dy*xa[1]+dz*xa[2], dx*ya[0]+dy*ya[1]+dz*ya[2]});
    }
    if (poly2d.empty()) return {};

    // Miter offset in 2D (negative = inward, positive = outward). Reuses the
    // same algorithm as Intersection::offset_in_3d. Falls back to the
    // un-offset polygon if the result degenerates.
    if (offset_dist != 0.0 && poly2d.size() >= 3) {
        size_t n = poly2d.size();
        double signed_area = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const auto& a = poly2d[i];
            const auto& b = poly2d[(i+1) % n];
            signed_area += a[0]*b[1] - b[0]*a[1];
        }
        double delta = offset_dist;
        if (signed_area < 0.0) delta = -delta;

        std::vector<std::array<double,2>> normals;
        normals.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const auto& a = poly2d[i];
            const auto& b = poly2d[(i+1) % n];
            double ex = b[0]-a[0], ey = b[1]-a[1];
            double len = std::sqrt(ex*ex + ey*ey);
            if (len < 1e-12) normals.push_back({0.0, 0.0});
            else normals.push_back({ey/len, -ex/len});
        }

        std::vector<std::array<double,2>> out;
        out.reserve(n * 3);
        for (size_t i = 0; i < n; ++i) {
            const auto& np = normals[(i + n - 1) % n];
            const auto& nn = normals[i];
            double cos_a = np[0]*nn[0] + np[1]*nn[1];
            double sin_a = np[0]*nn[1] - np[1]*nn[0];
            double denom = 1.0 + cos_a;
            bool concave = (cos_a > -0.999) && (sin_a * delta < 0.0) && (offset_dist > 0.0);
            if (concave) {
                out.push_back({poly2d[i][0] + np[0]*delta, poly2d[i][1] + np[1]*delta});
                out.push_back({poly2d[i][0], poly2d[i][1]});
                out.push_back({poly2d[i][0] + nn[0]*delta, poly2d[i][1] + nn[1]*delta});
            } else if (std::abs(denom) < 1e-9) {
                double mx = (np[0]+nn[0])*0.5, my = (np[1]+nn[1])*0.5;
                out.push_back({poly2d[i][0] + mx*delta, poly2d[i][1] + my*delta});
            } else {
                double bx = (np[0]+nn[0])/denom, by = (np[1]+nn[1])/denom;
                out.push_back({poly2d[i][0] + bx*delta, poly2d[i][1] + by*delta});
            }
        }

        // Reject if offset collapsed the polygon (area near zero).
        double out_area = 0.0;
        for (size_t i = 0; i < out.size(); ++i) {
            const auto& a = out[i];
            const auto& b = out[(i+1) % out.size()];
            out_area += a[0]*b[1] - b[0]*a[1];
        }
        if (out.size() >= 3 && std::abs(out_area) > 1e-4) poly2d = std::move(out);
    }

    double x_min = std::numeric_limits<double>::max(), x_max = -std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max(), y_max = -std::numeric_limits<double>::max();
    for (const auto& p : poly2d) {
        x_min = std::min(x_min, p[0]); x_max = std::max(x_max, p[0]);
        y_min = std::min(y_min, p[1]); y_max = std::max(y_max, p[1]);
    }

    auto pt_in_poly = [&](double px, double py) -> bool {
        size_t n = poly2d.size();
        bool inside = false;
        size_t j = n - 1;
        for (size_t i = 0; i < n; i++) {
            double xi = poly2d[i][0], yi = poly2d[i][1];
            double xj = poly2d[j][0], yj = poly2d[j][1];
            if (((yi > py) != (yj > py)) && (px < (xj-xi)*(py-yi)/(yj-yi)+xi))
                inside = !inside;
            j = i;
        }
        return inside;
    };

    std::vector<Point> result;
    for (double u = x_min; u <= x_max + 1e-10 && result.size() < max_pts; u += div_dist) {
        for (double v = y_min; v <= y_max + 1e-10 && result.size() < max_pts; v += div_dist) {
            if (pt_in_poly(u, v)) {
                result.emplace_back(
                    origin[0] + u*xa[0] + v*ya[0],
                    origin[1] + u*xa[1] + v*ya[1],
                    origin[2] + u*xa[2] + v*ya[2]
                );
            }
        }
    }
    return result;
}

namespace {
// Mapbox polylabel (quadtree+PQ). Polygon = rings of (u,v); ring 0 outer, rest holes.
using Ring2 = std::vector<std::array<double,2>>;

static double seg_dist_sq(double px, double py, double ax, double ay, double bx, double by) {
    double x = ax, y = ay;
    double dx = bx - x, dy = by - y;
    if (dx != 0.0 || dy != 0.0) {
        double t = ((px - x) * dx + (py - y) * dy) / (dx*dx + dy*dy);
        if (t > 1.0) { x = bx; y = by; }
        else if (t > 0.0) { x += dx * t; y += dy * t; }
    }
    dx = px - x; dy = py - y;
    return dx*dx + dy*dy;
}

static double point_to_polygon_dist(double px, double py, const std::vector<Ring2>& polygon) {
    bool inside = false;
    double min_dist_sq = std::numeric_limits<double>::infinity();
    for (const auto& ring : polygon) {
        size_t len = ring.size();
        for (size_t i = 0, j = len - 1; i < len; j = i++) {
            double ax = ring[i][0], ay = ring[i][1];
            double bx = ring[j][0], by = ring[j][1];
            if ((ay > py) != (by > py) && (px < (bx - ax) * (py - ay) / (by - ay) + ax))
                inside = !inside;
            min_dist_sq = std::min(min_dist_sq, seg_dist_sq(px, py, ax, ay, bx, by));
        }
    }
    return (inside ? 1.0 : -1.0) * std::sqrt(min_dist_sq);
}

struct PCell {
    double cx;
    double cy;
    double h;
    double d;
    double mx;
    PCell(double cx_, double cy_, double h_, const std::vector<Ring2>& polygon)
        : cx(cx_), cy(cy_), h(h_),
          d(point_to_polygon_dist(cx_, cy_, polygon)),
          mx(d + h_ * std::sqrt(2.0)) {}
};

static PCell centroid_cell(const std::vector<Ring2>& polygon) {
    double area = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    const auto& ring = polygon.at(0);
    size_t len = ring.size();
    for (size_t i = 0, j = len - 1; i < len; j = i++) {
        double ax = ring[i][0], ay = ring[i][1];
        double bx = ring[j][0], by = ring[j][1];
        double f = ax * by - bx * ay;
        cx += (ax + bx) * f;
        cy += (ay + by) * f;
        area += f * 3.0;
    }
    if (area == 0.0) {
        return PCell(ring.at(0)[0], ring.at(0)[1], 0.0, polygon);
    }
    return PCell(cx / area, cy / area, 0.0, polygon);
}

static std::array<double,3> mapbox_polylabel(const std::vector<Ring2>& polygon, double precision) {
    // envelope of outer ring
    const auto& outer = polygon.at(0);
    double min_x =  std::numeric_limits<double>::infinity();
    double min_y =  std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& p : outer) {
        min_x = std::min(min_x, p[0]); max_x = std::max(max_x, p[0]);
        min_y = std::min(min_y, p[1]); max_y = std::max(max_y, p[1]);
    }
    double size_x = max_x - min_x;
    double size_y = max_y - min_y;
    double cell_size = std::min(size_x, size_y);
    double h = cell_size / 2.0;
    if (cell_size == 0.0) return { min_x, min_y, 0.0 };

    auto cmp = [](const PCell& a, const PCell& b) { return a.mx < b.mx; };
    std::priority_queue<PCell, std::vector<PCell>, decltype(cmp)> queue(cmp);

    for (double x = min_x; x < max_x; x += cell_size) {
        for (double y = min_y; y < max_y; y += cell_size) {
            queue.emplace(x + h, y + h, h, polygon);
        }
    }

    PCell best = centroid_cell(polygon);
    PCell bbox_c(min_x + size_x / 2.0, min_y + size_y / 2.0, 0.0, polygon);
    if (bbox_c.d > best.d) best = bbox_c;

    while (!queue.empty()) {
        PCell cell = queue.top();
        queue.pop();
        if (cell.d > best.d) best = cell;
        if (cell.mx - best.d <= precision) continue;
        double nh = cell.h / 2.0;
        queue.emplace(cell.cx - nh, cell.cy - nh, nh, polygon);
        queue.emplace(cell.cx + nh, cell.cy - nh, nh, polygon);
        queue.emplace(cell.cx - nh, cell.cy + nh, nh, polygon);
        queue.emplace(cell.cx + nh, cell.cy + nh, nh, polygon);
    }
    return { best.cx, best.cy, best.d };
}
} // anonymous namespace

std::tuple<Point, Plane, double> Polyline::polylabel(const std::vector<Polyline>& polylines,
                                                     double precision) {
    if (polylines.empty()) return { Point(0,0,0), Plane(), 0.0 };

    // Local frame from outer polyline
    Point origin;
    Vector xa, ya, za;
    polylines[0].get_average_plane(origin, xa, ya, za);

    auto to2d = [&](const Point& p) -> std::array<double,2> {
        double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
        return { dx*xa[0]+dy*xa[1]+dz*xa[2], dx*ya[0]+dy*ya[1]+dz*ya[2] };
    };

    // Sort rings by bbox diagonal (outer first), wood behavior
    std::vector<std::vector<std::array<double,2>>> rings2d(polylines.size());
    std::vector<double> sizes(polylines.size(), 0.0);
    for (size_t i = 0; i < polylines.size(); i++) {
        const auto& pts = polylines[i].get_points();
        size_t last = pts.size();
        if (last > 1) {
            const Point& a = pts.front();
            const Point& b = pts.back();
            if (std::abs(a[0]-b[0])<1e-10 && std::abs(a[1]-b[1])<1e-10 && std::abs(a[2]-b[2])<1e-10)
                last--;
        }
        rings2d[i].reserve(last);
        double mnx = std::numeric_limits<double>::infinity(), mny = mnx;
        double mxx = -mnx, mxy = mxx;
        for (size_t j = 0; j < last; j++) {
            auto uv = to2d(pts[j]);
            rings2d[i].push_back(uv);
            mnx = std::min(mnx, uv[0]); mxx = std::max(mxx, uv[0]);
            mny = std::min(mny, uv[1]); mxy = std::max(mxy, uv[1]);
        }
        double dx = mxx - mnx, dy = mxy - mny;
        sizes[i] = dx*dx + dy*dy;
    }
    std::vector<int> ids(rings2d.size());
    std::iota(ids.begin(), ids.end(), 0);
    std::sort(ids.begin(), ids.end(), [&](int a, int b) { return sizes[a] > sizes[b]; });

    std::vector<Ring2> polygon;
    polygon.reserve(rings2d.size());
    for (int id : ids) polygon.push_back(std::move(rings2d[id]));

    std::array<double,3> cr = mapbox_polylabel(polygon, precision);

    // Back to 3D
    Point center(origin[0] + cr[0]*xa[0] + cr[1]*ya[0],
                 origin[1] + cr[0]*xa[1] + cr[1]*ya[1],
                 origin[2] + cr[0]*xa[2] + cr[1]*ya[2]);
    Plane plane(origin, xa, ya, za);
    return std::make_tuple(center, plane, cr[2]);
}

std::vector<Point> Polyline::polylabel_circle_division_points(
    const Vector& division_direction_in_3d,
    const std::vector<Polyline>& polylines,
    int division, double scale, double precision, bool orient_to_closest_edge) {

    std::tuple<Point, Plane, double> circle = polylabel(polylines, precision);
    const Point& center = std::get<0>(circle);
    const Plane& plane  = std::get<1>(circle);
    double radius       = std::get<2>(circle) * scale;

    bool is_direction_valid =
        division_direction_in_3d[0] != 0.0 ||
        division_direction_in_3d[1] != 0.0 ||
        division_direction_in_3d[2] != 0.0;

    // closest edge search
    size_t edge_i = 0;
    size_t edge_j = 0;
    double best_sq = std::numeric_limits<double>::infinity();
    if (orient_to_closest_edge) {
        for (size_t i = 0; i < polylines.size(); i++) {
            const auto& pts = polylines[i].get_points();
            if (pts.size() < 2) continue;
            for (size_t j = 0; j + 1 < pts.size(); j++) {
                const Point& a = pts[j];
                const Point& b = pts[j+1];
                double ex = b[0]-a[0], ey = b[1]-a[1], ez = b[2]-a[2];
                double len2 = ex*ex + ey*ey + ez*ez;
                if (len2 <= 0.0) continue;
                double px = center[0]-a[0], py = center[1]-a[1], pz = center[2]-a[2];
                double t = (px*ex + py*ey + pz*ez) / len2;
                if (t < 0.0 || t > 1.0) continue;
                double cx = a[0]+t*ex, cy = a[1]+t*ey, cz = a[2]+t*ez;
                double dx = center[0]-cx, dy = center[1]-cy, dz = center[2]-cz;
                double d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < best_sq) { best_sq = d2; edge_i = i; edge_j = j; }
            }
        }
    }

    Vector x_axis;
    Vector y_axis;
    Vector z_axis = plane.z_axis();
    if (is_direction_valid || orient_to_closest_edge) {
        Vector dir;
        if (orient_to_closest_edge && best_sq < std::numeric_limits<double>::infinity()) {
            const auto& pts = polylines[edge_i].get_points();
            dir = Vector(pts[edge_j+1][0]-pts[edge_j][0],
                         pts[edge_j+1][1]-pts[edge_j][1],
                         pts[edge_j+1][2]-pts[edge_j][2]);
        } else {
            dir = division_direction_in_3d;
        }
        x_axis = dir;
        y_axis = Vector(dir[1]*z_axis[2] - dir[2]*z_axis[1],
                        dir[2]*z_axis[0] - dir[0]*z_axis[2],
                        dir[0]*z_axis[1] - dir[1]*z_axis[0]);
    } else {
        x_axis = plane.x_axis();
        y_axis = plane.y_axis();
    }

    // unitize
    auto unit = [](Vector& v) {
        double L = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
        if (L > 0.0) v = Vector(v[0]/L, v[1]/L, v[2]/L);
    };
    unit(x_axis); unit(y_axis); unit(z_axis);

    std::vector<Point> points;
    points.reserve(division);
    const double pi_rad = 3.14159265358979323846 / 180.0;
    double chunk = 360.0 / division;
    for (int i = 0; i < division; i++) {
        double deg = i * chunk;
        double r = (45.0 + deg) * pi_rad;
        double u = radius * std::cos(r);
        double v = radius * std::sin(r);
        points.emplace_back(center[0] + u*x_axis[0] + v*y_axis[0],
                            center[1] + u*x_axis[1] + v*y_axis[1],
                            center[2] + u*x_axis[2] + v*y_axis[2]);
    }
    return points;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const Polyline& polyline) {
    return os << polyline.repr();
}

std::vector<Polyline> Polyline::boolean_op(const Polyline& a, const Polyline& b, int clip_type) {
    return BooleanPolyline::compute(a, b, clip_type);
}

std::vector<Polyline> Polyline::boolean_op(const Polyline& a, const Polyline& b, const Plane& plane, int clip_type) {
    double ox = plane.origin()[0], oy = plane.origin()[1], oz = plane.origin()[2];
    double xx = plane.x_axis()[0], xy = plane.x_axis()[1], xz = plane.x_axis()[2];
    double yx = plane.y_axis()[0], yy = plane.y_axis()[1], yz = plane.y_axis()[2];

    // Project to 2D as raw stride-3 coords (z=0) — no Polyline construction
    auto project = [&](const Polyline& pl) -> Polyline {
        Polyline p2d;
        int n = pl.point_count();
        p2d._coords.resize(n * 3);
        for (int i = 0; i < n; i++) {
            double dx = pl._coords[i*3] - ox, dy = pl._coords[i*3+1] - oy, dz = pl._coords[i*3+2] - oz;
            p2d._coords[i*3]   = dx*xx + dy*xy + dz*xz;
            p2d._coords[i*3+1] = dx*yx + dy*yy + dz*yz;
            p2d._coords[i*3+2] = 0.0;
        }
        // Ensure closing point matches first point exactly (avoid float drift)
        if (n >= 4) {
            double dx = p2d._coords[(n-1)*3] - p2d._coords[0];
            double dy = p2d._coords[(n-1)*3+1] - p2d._coords[1];
            if (dx*dx + dy*dy < 1.0) { // within 1mm = was meant to be closed
                p2d._coords[(n-1)*3] = p2d._coords[0];
                p2d._coords[(n-1)*3+1] = p2d._coords[1];
            }
        }
        return p2d;
    };

    auto pa2d = project(a);
    auto pb2d = project(b);

    // Ensure CCW winding in 2D (Vatti containment check requires CCW)
    auto ensure_ccw = [](Polyline& p2d) {
        int n = p2d.point_count();
        // Strip closing point for area calc
        int m = n;
        if (m >= 4) {
            double dx = p2d._coords[(m-1)*3] - p2d._coords[0];
            double dy = p2d._coords[(m-1)*3+1] - p2d._coords[1];
            if (dx*dx + dy*dy < 1e-10) m--;
        }
        if (m < 3) return;
        double area = 0;
        for (int i = 0; i < m; i++) {
            int j = (i + 1) % m;
            area += p2d._coords[i*3] * p2d._coords[j*3+1] - p2d._coords[j*3] * p2d._coords[i*3+1];
        }
        if (area < 0) { // CW → reverse to CCW
            for (int i = 0; i < n / 2; i++) {
                int j = n - 1 - i;
                std::swap(p2d._coords[i*3],   p2d._coords[j*3]);
                std::swap(p2d._coords[i*3+1], p2d._coords[j*3+1]);
                std::swap(p2d._coords[i*3+2], p2d._coords[j*3+2]);
            }
        }
    };
    ensure_ccw(pa2d);
    ensure_ccw(pb2d);

    auto results = BooleanPolyline::compute(pa2d, pb2d, clip_type);

    // Inverse-transform results back to 3D (in-place on _coords)
    for (auto& r : results) {
        int n = (int)(r._coords.size() / 3);
        for (int i = 0; i < n; i++) {
            double u = r._coords[i*3], v = r._coords[i*3+1];
            r._coords[i*3]   = ox + u*xx + v*yx;
            r._coords[i*3+1] = oy + u*xy + v*yy;
            r._coords[i*3+2] = oz + u*xz + v*yz;
        }
    }
    return results;
}

double Polyline::simplify_perp_dist(const Point& pt, const Point& line_start, const Point& line_end) {
    double dx = line_end[0] - line_start[0];
    double dy = line_end[1] - line_start[1];
    double dz = line_end[2] - line_start[2];
    double len_sq = dx * dx + dy * dy + dz * dz;
    if (len_sq == 0.0) {
        double ex = pt[0] - line_start[0];
        double ey = pt[1] - line_start[1];
        double ez = pt[2] - line_start[2];
        return std::sqrt(ex * ex + ey * ey + ez * ez);
    }
    double t = ((pt[0] - line_start[0]) * dx + (pt[1] - line_start[1]) * dy + (pt[2] - line_start[2]) * dz) / len_sq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double cx = line_start[0] + t * dx;
    double cy = line_start[1] + t * dy;
    double cz = line_start[2] + t * dz;
    double ex = pt[0] - cx;
    double ey = pt[1] - cy;
    double ez = pt[2] - cz;
    return std::sqrt(ex * ex + ey * ey + ez * ez);
}

void Polyline::simplify_rdp(const std::vector<Point>& points, int start, int end, double tolerance, std::vector<bool>& keep) {
    if (end <= start + 1) return;
    double max_dist = 0.0;
    int max_idx = start;
    for (int i = start + 1; i < end; ++i) {
        double d = simplify_perp_dist(points[i], points[start], points[end]);
        if (d > max_dist) {
            max_dist = d;
            max_idx = i;
        }
    }
    if (max_dist > tolerance) {
        keep[max_idx] = true;
        simplify_rdp(points, start, max_idx, tolerance, keep);
        simplify_rdp(points, max_idx, end, tolerance, keep);
    }
}

std::vector<Point> Polyline::simplify_points(const std::vector<Point>& points, double tolerance) {
    int n = static_cast<int>(points.size());
    if (n < 3) return points;
    std::vector<bool> keep(n, false);
    keep[0] = true;
    keep[n - 1] = true;
    simplify_rdp(points, 0, n - 1, tolerance, keep);
    std::vector<Point> result;
    for (int i = 0; i < n; ++i) {
        if (keep[i]) result.push_back(points[i]);
    }
    return result;
}

Polyline Polyline::simplify(double tolerance) const {
    std::vector<Point> pts = get_points();
    std::vector<Point> simplified = simplify_points(pts, tolerance);
    return Polyline(simplified);
}

} // namespace session_cpp
