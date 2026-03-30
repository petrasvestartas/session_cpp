#include "polyline.h"
#include "tolerance.h"
#include <cmath>
#include <sstream>
#include <array>
#include <functional>
#include <limits>
#include <queue>
#include <algorithm>

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
        double angle = 2.0 * TOLERANCE.PI * i / sides;
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
    if (point_count() >= 3) {
        std::vector<Point> pts = get_points();
        plane = Plane::from_points(pts);
    }
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
        xform.transform_point(pt);
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

    // Handle closed polylines
    if (is_closed()) {
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
    (void)pln;  // Reserved for future use - may project to plane
    if (point_count() < 3) return false;

    // Create a copy for transformation
    Polyline cp = *this;

    // Ensure closed for winding calculation
    if (!cp.is_closed()) {
        cp.add_point(cp.get_point(0));
    }

    // Calculate signed area (shoelace formula)
    double signed_area = 0.0;
    for (size_t i = 0; i < cp.point_count() - 1; i++) {
        Point pi = cp.get_point(i);
        Point pi1 = cp.get_point(i + 1);
        signed_area += (pi1[0] - pi[0]) * (pi1[1] + pi[1]);
    }

    return signed_area > 0;
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
    (void)offset_dist;
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

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const Polyline& polyline) {
    return os << polyline.repr();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Free functions on raw std::vector<Point> polylines (CGAL_Polyline compat)
///////////////////////////////////////////////////////////////////////////////////////////

void shift(std::vector<Point>& pline, int times) {
    Polyline tmp(pline);
    tmp.shift(times);
    pline = tmp.get_points();
}

double polyline_length(const std::vector<Point>& pline) {
    double len = 0;
    for (size_t i = 0; i + 1 < pline.size(); i++) {
        double dx = pline[i+1][0]-pline[i][0], dy = pline[i+1][1]-pline[i][1], dz = pline[i+1][2]-pline[i][2];
        len += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return len;
}

double polyline_length_squared(const std::vector<Point>& pline) {
    double len = 0;
    for (size_t i = 0; i + 1 < pline.size(); i++) {
        double dx = pline[i+1][0]-pline[i][0], dy = pline[i+1][1]-pline[i][1], dz = pline[i+1][2]-pline[i][2];
        len += dx*dx + dy*dy + dz*dz;
    }
    return len;
}

bool is_closed(const std::vector<Point>& pline) {
    if (pline.size() < 2) return false;
    return pline.front().distance(pline.back()) < static_cast<double>(Tolerance::ZERO_TOLERANCE);
}

Point center(const std::vector<Point>& pline) {
    size_t n = (is_closed(pline) && pline.size() > 1) ? pline.size()-1 : pline.size();
    double x=0, y=0, z=0;
    for (size_t i=0; i<n; i++) { x+=pline[i][0]; y+=pline[i][1]; z+=pline[i][2]; }
    return Point(x/n, y/n, z/n);
}

Vector center_vec(const std::vector<Point>& pline) {
    Point c = center(pline);
    return Vector(c[0], c[1], c[2]);
}

void get_average_plane(const std::vector<Point>& pline, Vector (&axes)[4]) {
    Point o = center(pline);
    axes[0] = Vector(o[0], o[1], o[2]);

    size_t len = (is_closed(pline) && pline.size() > 1) ? pline.size()-1 : pline.size();
    Vector normal(0, 0, 0);
    for (size_t i = 0; i < len; i++) {
        size_t prev = (i == 0) ? len-1 : i-1;
        size_t next = (i+1 == len) ? 0 : i+1;
        double ax = pline[i][0]-pline[prev][0], ay = pline[i][1]-pline[prev][1], az = pline[i][2]-pline[prev][2];
        double bx = pline[next][0]-pline[i][0],  by = pline[next][1]-pline[i][1],  bz = pline[next][2]-pline[i][2];
        normal[0] += ay*bz - az*by;
        normal[1] += az*bx - ax*bz;
        normal[2] += ax*by - ay*bx;
    }
    normal.normalize_self();

    double xdx = pline[1][0]-pline[0][0], xdy = pline[1][1]-pline[0][1], xdz = pline[1][2]-pline[0][2];
    Vector x_axis(xdx, xdy, xdz);
    x_axis.normalize_self();
    Vector y_axis = normal.cross(x_axis);
    axes[1] = x_axis;
    axes[2] = y_axis;
    axes[3] = normal;
}

void get_fast_plane(const std::vector<Point>& pline, Point& origin, Plane& pln) {
    Polyline(pline).get_fast_plane(origin, pln);
}

void transform(std::vector<Point>& pline, const Xform& xf) {
    for (auto& p : pline) xf.transform_point(p);
}

void move(std::vector<Point>& pline, const Vector& dir) {
    for (auto& p : pline) { p[0]+=dir[0]; p[1]+=dir[1]; p[2]+=dir[2]; }
}

bool is_clockwise(std::vector<Point>& pline, const Plane& pln) {
    return Polyline(pline).is_clockwise(pln);
}

void flip(std::vector<Point>& pline) {
    std::reverse(pline.begin(), pline.end());
}

void get_convex_corners(const std::vector<Point>& pline, std::vector<bool>& flags) {
    Polyline(pline).get_convex_corners(flags);
}

std::vector<Point> tween_two_polylines(const std::vector<Point>& p0, const std::vector<Point>& p1, double w) {
    std::vector<Point> result;
    result.reserve(p0.size());
    for (size_t i = 0; i < p0.size(); i++)
        result.push_back(Point(p0[i][0]+(p1[i][0]-p0[i][0])*w, p0[i][1]+(p1[i][1]-p0[i][1])*w, p0[i][2]+(p1[i][2]-p0[i][2])*w));
    return result;
}

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

void line_line_overlap_average(const std::vector<Point>& l0, const std::vector<Point>& l1, Line& out) {
    line_line_overlap_average(Line::from_points(l0[0], l0[1]), Line::from_points(l1[0], l1[1]), out);
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

void extend(std::vector<Point>& pline, int sID, double d0, double d1, double proportion0, double proportion1) {
    if (d0==0 && d1==0 && proportion0==0 && proportion1==0) return;
    Polyline tmp(pline);
    tmp.extend_segment(sID, d0, d1, proportion0, proportion1);
    pline = tmp.get_points();
}

double closest_distance_and_point(const Point& pt, const std::vector<Point>& poly, size_t& edge_id, Point& closest_point) {
    return Polyline(poly).closest_distance_and_point(pt, edge_id, closest_point);
}

void get_middle_line(const Line& l0, const Line& l1, Line& out) {
    Point s0=l0.start(), e0=l0.end(), s1=l1.start(), e1=l1.end();
    Point os, oe;
    Line::get_middle_line(s0, e0, s1, e1, os, oe);
    out = Line::from_points(os, oe);
}

// ═══════════════════════════════════════════════════════════════════════════
// Vatti scanline polygon clipping — ported from Clipper2 (Angus Johnson,
// Boost licence) into session_cpp types. Closed-path NonZero fill only.
// ═══════════════════════════════════════════════════════════════════════════
namespace {

struct BIVec2 { int64_t x, y; };
inline bool operator==(BIVec2 a, BIVec2 b) { return a.x==b.x && a.y==b.y; }
inline bool operator!=(BIVec2 a, BIVec2 b) { return !(a==b); }

// SIMD-optimized rounding on MSVC x64 — same trick Clipper2 uses.
#if defined(_MSC_VER) && (defined(_M_AMD64) || defined(_M_X64))
#include <xmmintrin.h>
#include <emmintrin.h>
#define VATTI_NEARBYINT(a) _mm_cvtsd_si64(_mm_set_sd(a))
#else
#define VATTI_NEARBYINT(a) static_cast<int64_t>(std::nearbyint(a))
#endif

enum : uint32_t { VF_None=0, VF_LocalMax=4, VF_LocalMin=8 };

struct VVertex {
    BIVec2 pt;
    VVertex* next = nullptr;
    VVertex* prev = nullptr;
    uint32_t flags = VF_None;
};

struct VLocalMinima { VVertex* vertex; int8_t polytype; };

struct VHorzSeg;
struct VOutPt {
    BIVec2 pt;
    VOutPt* next = nullptr;
    VOutPt* prev = nullptr;
    struct VOutRec* outrec;
    VHorzSeg* horz = nullptr;
};

struct VOutRec {
    size_t idx = 0;
    struct VActive* front_edge = nullptr;
    struct VActive* back_edge = nullptr;
    VOutPt* pts = nullptr;
    VOutRec* owner = nullptr;
};

struct VActive {
    BIVec2 bot, top;
    int64_t curr_x = 0;
    double dx = 0.0;
    int wind_dx = 1;
    int wind_cnt = 0;
    int wind_cnt2 = 0;
    VOutRec* outrec = nullptr;
    VActive* prev_in_ael = nullptr;
    VActive* next_in_ael = nullptr;
    VActive* prev_in_sel = nullptr;
    VActive* next_in_sel = nullptr;
    VActive* jump = nullptr;
    VVertex* vertex_top = nullptr;
    VLocalMinima* local_min = nullptr;
    bool is_left_bound = false;
    int8_t join_with = 0; // 0=None, 1=Left, 2=Right
};

struct VIntersectNode { BIVec2 pt; VActive* edge1; VActive* edge2; };
struct VHorzSeg { VOutPt* left_op; VOutPt* right_op = nullptr; bool left_to_right = true; };
struct VHorzJoin { VOutPt* op1; VOutPt* op2; };

// Pool allocator — items are never freed mid-run; reset() recycles all memory.
// Uses a single flat vector; pointers stay valid because we never shrink.
template<typename T> struct Pool {
    std::vector<T> buf;
    size_t count = 0;
    void ensure(size_t n) { if (buf.size() < n) buf.resize(n); }
    T* alloc() {
        if (count >= buf.size()) buf.resize(std::max<size_t>(buf.size()*2, 256));
        return &buf[count++];
    }
    void reset() { count = 0; }
};

// Max-heap using sorted vector — avoids std::priority_queue realloc on reset.
struct ScanlineHeap {
    std::vector<int64_t> buf;
    size_t sz = 0;
    void clear() { sz = 0; }
    bool empty() const { return sz == 0; }
    void push(int64_t y) {
        if (sz >= buf.size()) buf.resize(std::max<size_t>(buf.size()*2, 64));
        buf[sz] = y;
        // sift up
        size_t i = sz++;
        while (i > 0) { size_t p=(i-1)/2; if (buf[p]>=buf[i]) break; std::swap(buf[p],buf[i]); i=p; }
    }
    int64_t top() const { return buf[0]; }
    void pop() {
        buf[0] = buf[--sz];
        // sift down
        size_t i=0;
        for(;;) { size_t l=2*i+1, r=l+1, m=i;
            if(l<sz&&buf[l]>buf[m]) m=l; if(r<sz&&buf[r]>buf[m]) m=r;
            if(m==i) break; std::swap(buf[i],buf[m]); i=m; }
    }
};

struct VattiScratch {
    Pool<VVertex> vtx_pool;
    Pool<VActive> act_pool;
    Pool<VOutPt>  opt_pool;
    Pool<VOutRec> orc_pool;
    std::vector<VLocalMinima> locmin_list;
    std::vector<VIntersectNode> intersect_nodes;
    std::vector<VHorzSeg> horz_seg_list;
    std::vector<VHorzJoin> horz_join_list;
    std::vector<VOutRec*> outrec_list;
    ScanlineHeap scanline_list;
    std::vector<BIVec2> va, vb;
    // Engine state
    VActive* actives = nullptr;
    VActive* sel = nullptr;
    int64_t bot_y = 0;
    size_t locmin_idx = 0;
    bool succeeded = true;
    void reset() {
        vtx_pool.reset(); act_pool.reset(); opt_pool.reset(); orc_pool.reset();
        locmin_list.clear(); intersect_nodes.clear(); horz_seg_list.clear();
        horz_join_list.clear(); outrec_list.clear(); scanline_list.clear();
        actives = nullptr; sel = nullptr; bot_y = 0; locmin_idx = 0; succeeded = true;
    }
    VVertex* new_vertex() { auto* v = vtx_pool.alloc(); *v = VVertex{}; return v; }
    VActive* new_active() { auto* a = act_pool.alloc(); *a = VActive{}; return a; }
    VOutPt* new_outpt(BIVec2 pt, VOutRec* rec) { auto* o = opt_pool.alloc(); *o = VOutPt{}; o->pt=pt; o->outrec=rec; o->next=o; o->prev=o; return o; }
    VOutRec* new_outrec() { auto* r = orc_pool.alloc(); *r = VOutRec{}; r->idx = outrec_list.size(); outrec_list.push_back(r); return r; }
};
static thread_local VattiScratch vtls;

// ── Geometry helpers ─────────────────────────────────────────────────────

inline double v_get_dx(BIVec2 p1, BIVec2 p2) {
    double dy = double(p2.y - p1.y);
    if (dy != 0) return double(p2.x - p1.x) / dy;
    return (p2.x > p1.x) ? -std::numeric_limits<double>::max() : std::numeric_limits<double>::max();
}

inline int64_t v_top_x(const VActive& ae, int64_t y) {
    if (y == ae.top.y || ae.top.x == ae.bot.x) return ae.top.x;
    if (y == ae.bot.y) return ae.bot.x;
    return ae.bot.x + VATTI_NEARBYINT(ae.dx * double(y - ae.bot.y));
}

inline bool v_is_horizontal(const VActive& e) { return e.top.y == e.bot.y; }
inline bool v_is_hot(const VActive& e) { return e.outrec != nullptr; }
inline bool v_is_maxima(const VVertex& v) { return (v.flags & VF_LocalMax) != 0; }
inline bool v_is_maxima(const VActive& e) { return v_is_maxima(*e.vertex_top); }
inline bool v_is_front(const VActive& e) { return &e == e.outrec->front_edge; }
inline bool v_is_joined(const VActive& e) { return e.join_with != 0; }
inline bool v_same_polytype(const VActive& a, const VActive& b) { return a.local_min->polytype == b.local_min->polytype; }
inline int8_t v_polytype(const VActive& e) { return e.local_min->polytype; }
inline void v_set_dx(VActive& e) { e.dx = v_get_dx(e.bot, e.top); }

inline VVertex* v_next_vertex(const VActive& e) { return (e.wind_dx > 0) ? e.vertex_top->next : e.vertex_top->prev; }
inline VVertex* v_prev_prev_vertex(const VActive& ae) { return (ae.wind_dx > 0) ? ae.vertex_top->prev->prev : ae.vertex_top->next->next; }

inline double v_cross_product(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p2.x-p1.x)*double(p3.y-p2.y) - double(p2.y-p1.y)*double(p3.x-p2.x);
}
inline double v_dot_product(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p2.x-p1.x)*double(p3.x-p2.x) + double(p2.y-p1.y)*double(p3.y-p2.y);
}

inline bool v_products_equal(int64_t a, int64_t b, int64_t c, int64_t d) {
#if (defined(__clang__) || defined(__GNUC__)) && UINTPTR_MAX >= UINT64_MAX
    return static_cast<__int128_t>(a)*static_cast<__int128_t>(b) == static_cast<__int128_t>(c)*static_cast<__int128_t>(d);
#else
    auto lo = [](uint64_t x){ return x & 0xFFFFFFFF; };
    auto hi = [](uint64_t x){ return x >> 32; };
    auto mul = [&](uint64_t a, uint64_t b) -> std::pair<uint64_t,uint64_t> {
        uint64_t x1=lo(a)*lo(b); uint64_t x2=hi(a)*lo(b)+hi(x1); uint64_t x3=lo(a)*hi(b)+lo(x2);
        return {lo(x3)<<32|lo(x1), hi(a)*hi(b)+hi(x2)+hi(x3)};
    };
    auto sign = [](int64_t x){ return (x>0)-(x<0); };
    uint64_t ua=std::abs(a), ub=std::abs(b), uc=std::abs(c), ud=std::abs(d);
    auto [r1,c1]=mul(ua,ub); auto [r2,c2]=mul(uc,ud);
    return r1==r2 && c1==c2 && sign(a)*sign(b)==sign(c)*sign(d);
#endif
}

inline bool v_is_collinear(BIVec2 p1, BIVec2 shared, BIVec2 p2) {
    return v_products_equal(shared.x-p1.x, p2.y-shared.y, shared.y-p1.y, p2.x-shared.x);
}

inline double v_perpendic_dist_sq(BIVec2 pt, BIVec2 l1, BIVec2 l2) {
    double a=double(pt.x-l1.x), b=double(pt.y-l1.y), c=double(l2.x-l1.x), d=double(l2.y-l1.y);
    if (c==0 && d==0) return 0;
    double e = a*d - c*b;
    return (e*e) / (c*c + d*d);
}

inline bool v_get_seg_isect_pt(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d, BIVec2& ip) {
    double dx1=double(b.x-a.x), dy1=double(b.y-a.y), dx2=double(d.x-c.x), dy2=double(d.y-c.y);
    double det = dy1*dx2 - dy2*dx1;
    if (det == 0.0) return false;
    double t = (double(a.x-c.x)*dy2 - double(a.y-c.y)*dx2) / det;
    if (t <= 0.0) ip = a;
    else if (t >= 1.0) ip = b;
    else { ip.x = a.x + VATTI_NEARBYINT(t*dx1); ip.y = a.y + VATTI_NEARBYINT(t*dy1); }
    return true;
}

inline BIVec2 v_closest_pt_on_seg(BIVec2 pt, BIVec2 s1, BIVec2 s2) {
    if (s1==s2) return s1;
    double dx=double(s2.x-s1.x), dy=double(s2.y-s1.y);
    double q = (double(pt.x-s1.x)*dx + double(pt.y-s1.y)*dy) / (dx*dx + dy*dy);
    if (q<0) q=0; else if (q>1) q=1;
    return {s1.x + VATTI_NEARBYINT(q*dx), s1.y + VATTI_NEARBYINT(q*dy)};
}

inline bool v_segs_intersect(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d) {
    auto sign = [](double v) -> int { if (!v) return 0; return v>0?1:-1; };
    return (sign(v_cross_product(a,c,d)) * sign(v_cross_product(b,c,d)) < 0) &&
           (sign(v_cross_product(c,a,b)) * sign(v_cross_product(d,a,b)) < 0);
}

inline double v_area_outpt(VOutPt* op) {
    double r = 0.0; VOutPt* o = op;
    do { r += double(o->prev->pt.y + o->pt.y) * double(o->prev->pt.x - o->pt.x); o = o->next; } while (o != op);
    return r * 0.5;
}

inline double v_area_tri(BIVec2 p1, BIVec2 p2, BIVec2 p3) {
    return double(p3.y+p1.y)*double(p3.x-p1.x) + double(p1.y+p2.y)*double(p1.x-p2.x) + double(p2.y+p3.y)*double(p2.x-p3.x);
}

inline bool v_pts_close(BIVec2 a, BIVec2 b) { return std::llabs(a.x-b.x)<2 && std::llabs(a.y-b.y)<2; }

inline bool v_very_small_tri(VOutPt& op) {
    return op.next->next==op.prev && (v_pts_close(op.prev->pt,op.next->pt) || v_pts_close(op.pt,op.next->pt) || v_pts_close(op.pt,op.prev->pt));
}

inline bool v_valid_closed(VOutPt* op) { return op && op->next!=op && op->next!=op->prev && !v_very_small_tri(*op); }

static bool pip_i(BIVec2 pt, const std::vector<BIVec2>& poly) {
    int winding = 0, n = (int)poly.size();
    for (int i = 0; i < n; i++) {
        BIVec2 a = poly[i], b = poly[(i+1)%n];
        if (a.y <= pt.y) { if (b.y > pt.y && (int64_t(b.x-a.x)*int64_t(pt.y-a.y) - int64_t(b.y-a.y)*int64_t(pt.x-a.x)) > 0) ++winding; }
        else { if (b.y <= pt.y && (int64_t(b.x-a.x)*int64_t(pt.y-a.y) - int64_t(b.y-a.y)*int64_t(pt.x-a.x)) < 0) --winding; }
    }
    return winding != 0;
}

// ── Vertex building + local minima detection ─────────────────────────────

static void v_add_path(const std::vector<BIVec2>& pts, int n, int8_t polytype, VattiScratch& sc) {
    if (n < 3) return;
    // Bulk-allocate all vertices at once — one bounds check, contiguous in memory.
    auto& pool = sc.vtx_pool;
    pool.ensure(pool.count + n);
    VVertex* base = &pool.buf[pool.count];
    // Fill vertices, dedup, build linked list in single pass
    base[0] = VVertex{}; base[0].pt = pts[0];
    VVertex* prev_v = &base[0];
    int cnt = 1;
    for (int i = 1; i < n; i++) {
        if (pts[i] == prev_v->pt) continue;
        VVertex* cv = &base[cnt]; *cv = VVertex{}; cv->pt = pts[i];
        cv->prev = prev_v; prev_v->next = cv;
        prev_v = cv; cnt++;
    }
    if (cnt < 3) return;
    if (prev_v->pt == base[0].pt) { prev_v = prev_v->prev; cnt--; }
    if (cnt < 3) return;
    pool.count += cnt; // commit allocation
    prev_v->next = &base[0]; base[0].prev = prev_v;

    // Find local minima in the closed path (single pass)
    VVertex* pv = base[0].prev;
    while (pv != &base[0] && pv->pt.y == base[0].pt.y) pv = pv->prev;
    if (pv == &base[0]) return;
    bool going_up = pv->pt.y > base[0].pt.y, going_up0 = going_up;
    pv = &base[0];
    VVertex* cv = base[0].next;
    while (cv != &base[0]) {
        if (cv->pt.y > pv->pt.y && going_up) { pv->flags |= VF_LocalMax; going_up = false; }
        else if (cv->pt.y < pv->pt.y && !going_up) { going_up = true; pv->flags |= VF_LocalMin; sc.locmin_list.push_back({pv, polytype}); }
        pv = cv; cv = cv->next;
    }
    if (going_up != going_up0) {
        if (going_up0) { pv->flags |= VF_LocalMin; sc.locmin_list.push_back({pv, polytype}); }
        else pv->flags |= VF_LocalMax;
    }
}

// ── AEL operations (doubly-linked list — all O(1)) ──────────────────────

inline VActive* v_get_maxima_pair(const VActive& e) {
    VActive* e2 = e.next_in_ael;
    while (e2) { if (e2->vertex_top == e.vertex_top) return e2; e2 = e2->next_in_ael; }
    return nullptr;
}

inline VVertex* v_get_curr_y_maxima(const VActive& e) {
    VVertex* r = e.vertex_top;
    if (e.wind_dx > 0) while (r->next->pt.y == r->pt.y) r = r->next;
    else while (r->prev->pt.y == r->pt.y) r = r->prev;
    return v_is_maxima(*r) ? r : nullptr;
}

inline VActive* v_get_prev_hot(const VActive& e) {
    VActive* p = e.prev_in_ael;
    while (p && !v_is_hot(*p)) p = p->prev_in_ael;
    return p;
}

static bool v_is_valid_ael_order(const VActive& resident, const VActive& newcomer) {
    if (newcomer.curr_x != resident.curr_x) return newcomer.curr_x > resident.curr_x;
    double d = v_cross_product(resident.top, newcomer.bot, newcomer.top);
    if (d != 0) return d < 0;
    if (!v_is_maxima(resident) && resident.top.y > newcomer.top.y)
        return v_cross_product(newcomer.bot, resident.top, v_next_vertex(resident)->pt) <= 0;
    if (!v_is_maxima(newcomer) && newcomer.top.y > resident.top.y)
        return v_cross_product(newcomer.bot, newcomer.top, v_next_vertex(newcomer)->pt) >= 0;
    int64_t y = newcomer.bot.y;
    if (resident.bot.y != y || resident.local_min->vertex->pt.y != y) return newcomer.is_left_bound;
    if (resident.is_left_bound != newcomer.is_left_bound) return newcomer.is_left_bound;
    if (v_is_collinear(v_prev_prev_vertex(resident)->pt, resident.bot, resident.top)) return true;
    return (v_cross_product(v_prev_prev_vertex(resident)->pt, newcomer.bot, v_prev_prev_vertex(newcomer)->pt) > 0) == newcomer.is_left_bound;
}

static void v_insert_left_edge(VattiScratch& sc, VActive& e) {
    if (!sc.actives) { e.prev_in_ael=nullptr; e.next_in_ael=nullptr; sc.actives=&e; }
    else if (!v_is_valid_ael_order(*sc.actives, e)) {
        e.prev_in_ael=nullptr; e.next_in_ael=sc.actives; sc.actives->prev_in_ael=&e; sc.actives=&e;
    } else {
        VActive* e2 = sc.actives;
        while (e2->next_in_ael && v_is_valid_ael_order(*e2->next_in_ael, e)) e2 = e2->next_in_ael;
        if (e2->join_with == 2/*Right*/) e2 = e2->next_in_ael;
        if (!e2) return;
        e.next_in_ael = e2->next_in_ael; if (e2->next_in_ael) e2->next_in_ael->prev_in_ael = &e;
        e.prev_in_ael = e2; e2->next_in_ael = &e;
    }
}

inline void v_insert_right_edge(VActive& e, VActive& e2) {
    e2.next_in_ael = e.next_in_ael; if (e.next_in_ael) e.next_in_ael->prev_in_ael = &e2;
    e2.prev_in_ael = &e; e.next_in_ael = &e2;
}

inline void v_swap_positions_in_ael(VattiScratch& sc, VActive& e1, VActive& e2) {
    VActive* next = e2.next_in_ael; if (next) next->prev_in_ael = &e1;
    VActive* prev = e1.prev_in_ael; if (prev) prev->next_in_ael = &e2;
    e2.prev_in_ael = prev; e2.next_in_ael = &e1; e1.prev_in_ael = &e2; e1.next_in_ael = next;
    if (!e2.prev_in_ael) sc.actives = &e2;
}

inline void v_delete_from_ael(VattiScratch& sc, VActive& e) {
    VActive* prev = e.prev_in_ael; VActive* next = e.next_in_ael;
    if (!prev && !next && &e != sc.actives) return;
    if (prev) prev->next_in_ael = next; else sc.actives = next;
    if (next) next->prev_in_ael = prev;
}

// ── Scanline ─────────────────────────────────────────────────────────────

inline void v_insert_scanline(VattiScratch& sc, int64_t y) { sc.scanline_list.push(y); }
inline bool v_pop_scanline(VattiScratch& sc, int64_t& y) {
    auto& sl = sc.scanline_list;
    if (sl.empty()) return false;
    y = sl.top(); sl.pop();
    while (!sl.empty() && y == sl.top()) sl.pop();
    return true;
}
inline bool v_pop_locmin(VattiScratch& sc, int64_t y, VLocalMinima*& lm) {
    if (sc.locmin_idx >= sc.locmin_list.size() || sc.locmin_list[sc.locmin_idx].vertex->pt.y != y) return false;
    lm = &sc.locmin_list[sc.locmin_idx++];
    return true;
}
inline void v_push_horz(VattiScratch& sc, VActive& e) { e.next_in_sel = sc.sel; sc.sel = &e; }
inline bool v_pop_horz(VattiScratch& sc, VActive*& e) { e = sc.sel; if (!e) return false; sc.sel = sc.sel->next_in_sel; return true; }

// ── Winding + contribution ───────────────────────────────────────────────

static void v_set_wind_count(VattiScratch& sc, VActive& e) {
    VActive* e2 = e.prev_in_ael;
    int8_t pt = v_polytype(e);
    while (e2 && v_polytype(*e2) != pt) e2 = e2->prev_in_ael;
    if (!e2) { e.wind_cnt = e.wind_dx; e2 = sc.actives; }
    else {
        if (e2->wind_cnt * e2->wind_dx < 0) {
            if (std::abs(e2->wind_cnt) > 1)
                e.wind_cnt = (e2->wind_dx * e.wind_dx < 0) ? e2->wind_cnt : e2->wind_cnt + e.wind_dx;
            else e.wind_cnt = e.wind_dx;
        } else {
            e.wind_cnt = (e2->wind_dx * e.wind_dx < 0) ? e2->wind_cnt : e2->wind_cnt + e.wind_dx;
        }
        e.wind_cnt2 = e2->wind_cnt2; e2 = e2->next_in_ael;
    }
    while (e2 != &e) {
        if (v_polytype(*e2) != pt) e.wind_cnt2 += e2->wind_dx;
        e2 = e2->next_in_ael;
    }
}

static bool v_is_contributing(const VActive& e, int cliptype) {
    // NonZero fill rule only
    if (std::abs(e.wind_cnt) != 1) return false;
    int wc2 = std::abs(e.wind_cnt2);
    if (cliptype == 0) return wc2 != 0;              // intersection
    if (cliptype == 1) return wc2 == 0;              // union
    // difference
    bool r = (wc2 == 0);
    return (v_polytype(e) == 0) ? r : !r; // subject vs clip
}

// ── Output operations ────────────────────────────────────────────────────

inline void v_set_sides(VOutRec& or_, VActive& f, VActive& b) { or_.front_edge = &f; or_.back_edge = &b; }

static void v_swap_outrecs(VActive& e1, VActive& e2) {
    VOutRec* or1=e1.outrec, *or2=e2.outrec;
    if (or1==or2) { VActive* t=or1->front_edge; or1->front_edge=or1->back_edge; or1->back_edge=t; return; }
    if (or1) { if (&e1==or1->front_edge) or1->front_edge=&e2; else or1->back_edge=&e2; }
    if (or2) { if (&e2==or2->front_edge) or2->front_edge=&e1; else or2->back_edge=&e1; }
    e1.outrec=or2; e2.outrec=or1;
}

static VOutPt* v_add_outpt(const VActive& e, BIVec2 pt, VattiScratch& sc) {
    VOutRec* outrec = e.outrec;
    bool to_front = v_is_front(e);
    VOutPt* op_front = outrec->pts, *op_back = op_front->next;
    if (to_front && pt == op_front->pt) return op_front;
    if (!to_front && pt == op_back->pt) return op_back;
    VOutPt* nop = sc.new_outpt(pt, outrec);
    op_back->prev = nop; nop->prev = op_front; nop->next = op_back; op_front->next = nop;
    if (to_front) outrec->pts = nop;
    return nop;
}

static VOutPt* v_add_local_min_poly(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc, bool is_new) {
    VOutRec* outrec = sc.new_outrec();
    e1.outrec = outrec; e2.outrec = outrec;
    VActive* prev_hot = v_get_prev_hot(e1);
    if (prev_hot) {
        if ((prev_hot == prev_hot->outrec->front_edge) == is_new) v_set_sides(*outrec, e2, e1);
        else v_set_sides(*outrec, e1, e2);
    } else {
        outrec->owner = nullptr;
        if (is_new) v_set_sides(*outrec, e1, e2); else v_set_sides(*outrec, e2, e1);
    }
    VOutPt* op = sc.new_outpt(pt, outrec);
    outrec->pts = op;
    return op;
}

static void v_uncouple(VActive& ae) {
    VOutRec* or_ = ae.outrec; if (!or_) return;
    or_->front_edge->outrec = nullptr; or_->back_edge->outrec = nullptr;
    or_->front_edge = nullptr; or_->back_edge = nullptr;
}

static void v_join_outrec_paths(VActive& e1, VActive& e2) {
    VOutPt* p1_st=e1.outrec->pts, *p2_st=e2.outrec->pts;
    VOutPt* p1_end=p1_st->next, *p2_end=p2_st->next;
    if (v_is_front(e1)) {
        p2_end->prev=p1_st; p1_st->next=p2_end; p2_st->next=p1_end; p1_end->prev=p2_st;
        e1.outrec->pts=p2_st; e1.outrec->front_edge=e2.outrec->front_edge;
        if (e1.outrec->front_edge) e1.outrec->front_edge->outrec=e1.outrec;
    } else {
        p1_end->prev=p2_st; p2_st->next=p1_end; p1_st->next=p2_end; p2_end->prev=p1_st;
        e1.outrec->back_edge=e2.outrec->back_edge;
        if (e1.outrec->back_edge) e1.outrec->back_edge->outrec=e1.outrec;
    }
    e2.outrec->front_edge=nullptr; e2.outrec->back_edge=nullptr; e2.outrec->pts=nullptr;
    e2.outrec->owner = e1.outrec;
    e1.outrec=nullptr; e2.outrec=nullptr;
}

static void v_split(VActive& e, BIVec2 pt, VattiScratch& sc); // forward decl

static VOutPt* v_add_local_max_poly(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc) {
    if (v_is_joined(e1)) v_split(e1, pt, sc);
    if (v_is_joined(e2)) v_split(e2, pt, sc);

    // Reset joins more carefully
    // (simplified: no open path handling)
    if (v_is_front(e1) == v_is_front(e2)) { sc.succeeded = false; return nullptr; }

    VOutPt* result = v_add_outpt(e1, pt, sc);
    if (e1.outrec == e2.outrec) {
        VOutRec& outrec = *e1.outrec; outrec.pts = result;
        v_uncouple(e1); result = outrec.pts;
    } else if (e1.outrec->idx < e2.outrec->idx) v_join_outrec_paths(e1, e2);
    else v_join_outrec_paths(e2, e1);
    return result;
}

// ── Split + CheckJoin ────────────────────────────────────────────────────

static void v_split(VActive& e, BIVec2 pt, VattiScratch& sc) {
    if (e.join_with == 2/*Right*/) {
        e.join_with = 0; e.next_in_ael->join_with = 0;
        v_add_local_min_poly(e, *e.next_in_ael, pt, sc, true);
    } else {
        e.join_with = 0; e.prev_in_ael->join_with = 0;
        v_add_local_min_poly(*e.prev_in_ael, e, pt, sc, true);
    }
}

static void v_check_join_left(VActive& e, BIVec2 pt, VattiScratch& sc, bool check_curr_x = false) {
    VActive* prev = e.prev_in_ael;
    if (!prev || !v_is_hot(e) || !v_is_hot(*prev) || v_is_horizontal(e) || v_is_horizontal(*prev)) return;
    if ((pt.y < e.top.y+2 || pt.y < prev->top.y+2) && (e.bot.y > pt.y || prev->bot.y > pt.y)) return;
    if (check_curr_x) { if (v_perpendic_dist_sq(pt, prev->bot, prev->top) > 0.25) return; }
    else if (e.curr_x != prev->curr_x) return;
    if (!v_is_collinear(e.top, pt, prev->top)) return;
    if (e.outrec->idx == prev->outrec->idx) v_add_local_max_poly(*prev, e, pt, sc);
    else if (e.outrec->idx < prev->outrec->idx) v_join_outrec_paths(e, *prev);
    else v_join_outrec_paths(*prev, e);
    prev->join_with = 2; e.join_with = 1;
}

static void v_check_join_right(VActive& e, BIVec2 pt, VattiScratch& sc, bool check_curr_x = false) {
    VActive* next = e.next_in_ael;
    if (!next || !v_is_hot(e) || !v_is_hot(*next) || v_is_horizontal(e) || v_is_horizontal(*next)) return;
    if ((pt.y < e.top.y+2 || pt.y < next->top.y+2) && (e.bot.y > pt.y || next->bot.y > pt.y)) return;
    if (check_curr_x) { if (v_perpendic_dist_sq(pt, next->bot, next->top) > 0.35) return; }
    else if (e.curr_x != next->curr_x) return;
    if (!v_is_collinear(e.top, pt, next->top)) return;
    if (e.outrec->idx == next->outrec->idx) v_add_local_max_poly(e, *next, pt, sc);
    else if (e.outrec->idx < next->outrec->idx) v_join_outrec_paths(e, *next);
    else v_join_outrec_paths(*next, e);
    e.join_with = 2; next->join_with = 1;
}

// ── IntersectEdges ───────────────────────────────────────────────────────

static void v_intersect_edges(VActive& e1, VActive& e2, BIVec2 pt, VattiScratch& sc, int cliptype) {
    // Closed paths only
    if (v_is_joined(e1)) v_split(e1, pt, sc);
    if (v_is_joined(e2)) v_split(e2, pt, sc);

    int old_e1_wc, old_e2_wc;
    if (v_polytype(e1) == v_polytype(e2)) {
        // NonZero fill
        if (e1.wind_cnt + e2.wind_dx == 0) e1.wind_cnt = -e1.wind_cnt; else e1.wind_cnt += e2.wind_dx;
        if (e2.wind_cnt - e1.wind_dx == 0) e2.wind_cnt = -e2.wind_cnt; else e2.wind_cnt -= e1.wind_dx;
    } else {
        e1.wind_cnt2 += e2.wind_dx;
        e2.wind_cnt2 -= e1.wind_dx;
    }
    old_e1_wc = std::abs(e1.wind_cnt);
    old_e2_wc = std::abs(e2.wind_cnt);

    bool e1_in01 = old_e1_wc==0||old_e1_wc==1;
    bool e2_in01 = old_e2_wc==0||old_e2_wc==1;
    if ((!v_is_hot(e1) && !e1_in01) || (!v_is_hot(e2) && !e2_in01)) return;

    if (v_is_hot(e1) && v_is_hot(e2)) {
        if ((old_e1_wc!=0&&old_e1_wc!=1) || (old_e2_wc!=0&&old_e2_wc!=1) || (v_polytype(e1)!=v_polytype(e2))) {
            v_add_local_max_poly(e1, e2, pt, sc);
        } else if (v_is_front(e1) || e1.outrec==e2.outrec) {
            v_add_local_max_poly(e1, e2, pt, sc);
            v_add_local_min_poly(e1, e2, pt, sc, false);
        } else {
            v_add_outpt(e1, pt, sc); v_add_outpt(e2, pt, sc);
            v_swap_outrecs(e1, e2);
        }
    } else if (v_is_hot(e1)) { v_add_outpt(e1, pt, sc); v_swap_outrecs(e1, e2); }
    else if (v_is_hot(e2)) { v_add_outpt(e2, pt, sc); v_swap_outrecs(e1, e2); }
    else {
        int64_t e1Wc2 = std::abs(e1.wind_cnt2), e2Wc2 = std::abs(e2.wind_cnt2);
        if (!v_same_polytype(e1, e2)) { v_add_local_min_poly(e1, e2, pt, sc, false); }
        else if (old_e1_wc==1 && old_e2_wc==1) {
            if (cliptype==0) { if (e1Wc2>0 && e2Wc2>0) v_add_local_min_poly(e1, e2, pt, sc, false); }
            else if (cliptype==1) { if (e1Wc2<=0 && e2Wc2<=0) v_add_local_min_poly(e1, e2, pt, sc, false); }
            else { // difference
                if ((v_polytype(e1)==1/*clip*/ && e1Wc2>0 && e2Wc2>0) ||
                    (v_polytype(e1)==0/*subj*/ && e1Wc2<=0 && e2Wc2<=0))
                    v_add_local_min_poly(e1, e2, pt, sc, false);
            }
        }
    }
}

// ── Horizontal edge processing ───────────────────────────────────────────

static void v_add_trial_horz_join(VattiScratch& sc, VOutPt* op) { sc.horz_seg_list.push_back({op}); }

inline VOutPt* v_get_last_op(const VActive& e) {
    VOutPt* r = e.outrec->pts;
    if (&e != e.outrec->front_edge) r = r->next;
    return r;
}

inline void v_update_edge_into_ael(VattiScratch& sc, VActive* e, int cliptype) {
    e->bot = e->top; e->vertex_top = v_next_vertex(*e); e->top = e->vertex_top->pt;
    e->curr_x = e->bot.x; v_set_dx(*e);
    if (v_is_joined(*e)) v_split(*e, e->bot, sc);
    if (v_is_horizontal(*e)) {
        // Trim 180-deg spikes
        BIVec2 pt = v_next_vertex(*e)->pt;
        while (pt.y == e->top.y) {
            if ((pt.x < e->top.x) != (e->bot.x < e->top.x)) break;
            e->vertex_top = v_next_vertex(*e); e->top = pt;
            if (v_is_maxima(*e)) break;
            pt = v_next_vertex(*e)->pt;
        }
        v_set_dx(*e);
        return;
    }
    v_insert_scanline(sc, e->top.y);
    v_check_join_left(*e, e->bot, sc);
    v_check_join_right(*e, e->bot, sc, true);
}

static bool v_reset_horz_dir(const VActive& horz, const VVertex* max_v, int64_t& left, int64_t& right) {
    if (horz.bot.x == horz.top.x) { left=horz.curr_x; right=horz.curr_x;
        VActive* e=horz.next_in_ael; while(e && e->vertex_top!=max_v) e=e->next_in_ael; return e!=nullptr; }
    if (horz.curr_x < horz.top.x) { left=horz.curr_x; right=horz.top.x; return true; }
    left=horz.top.x; right=horz.curr_x; return false;
}

static void v_do_horizontal(VActive& horz, VattiScratch& sc, int cliptype) {
    int64_t y = horz.bot.y;
    VVertex* vertex_max = v_get_curr_y_maxima(horz);
    int64_t horz_left, horz_right;
    bool is_ltr = v_reset_horz_dir(horz, vertex_max, horz_left, horz_right);
    if (v_is_hot(horz)) { VOutPt* op = v_add_outpt(horz, {horz.curr_x, y}, sc); v_add_trial_horz_join(sc, op); }

    while (true) {
        VActive* e = is_ltr ? horz.next_in_ael : horz.prev_in_ael;
        while (e) {
            if (e->vertex_top == vertex_max) {
                if (v_is_hot(horz) && v_is_joined(*e)) v_split(*e, e->top, sc);
                if (v_is_hot(horz)) {
                    while (horz.vertex_top != vertex_max) { v_add_outpt(horz, horz.top, sc); v_update_edge_into_ael(sc, &horz, cliptype); }
                    if (is_ltr) v_add_local_max_poly(horz, *e, horz.top, sc);
                    else v_add_local_max_poly(*e, horz, horz.top, sc);
                }
                v_delete_from_ael(sc, *e); v_delete_from_ael(sc, horz); return;
            }
            if (vertex_max != horz.vertex_top) {
                if ((is_ltr && e->curr_x > horz_right) || (!is_ltr && e->curr_x < horz_left)) break;
                if (e->curr_x == horz.top.x && !v_is_horizontal(*e)) {
                    BIVec2 pt2 = v_next_vertex(horz)->pt;
                    if (is_ltr) { if (v_top_x(*e, pt2.y) >= pt2.x) break; }
                    else { if (v_top_x(*e, pt2.y) <= pt2.x) break; }
                }
            }
            BIVec2 pt = {e->curr_x, horz.bot.y};
            if (is_ltr) {
                v_intersect_edges(horz, *e, pt, sc, cliptype);
                v_swap_positions_in_ael(sc, horz, *e);
                v_check_join_left(*e, pt, sc);
                horz.curr_x = e->curr_x; e = horz.next_in_ael;
            } else {
                v_intersect_edges(*e, horz, pt, sc, cliptype);
                v_swap_positions_in_ael(sc, *e, horz);
                v_check_join_right(*e, pt, sc);
                horz.curr_x = e->curr_x; e = horz.prev_in_ael;
            }
            if (horz.outrec) v_add_trial_horz_join(sc, v_get_last_op(horz));
        }
        if (v_next_vertex(horz)->pt.y != horz.top.y) break;
        if (v_is_hot(horz)) v_add_outpt(horz, horz.top, sc);
        v_update_edge_into_ael(sc, &horz, cliptype);
        is_ltr = v_reset_horz_dir(horz, vertex_max, horz_left, horz_right);
    }
    if (v_is_hot(horz)) { VOutPt* op = v_add_outpt(horz, horz.top, sc); v_add_trial_horz_join(sc, op); }
    v_update_edge_into_ael(sc, &horz, cliptype);
}

// ── HorzSeg/HorzJoin handling ────────────────────────────────────────────

static VOutPt* v_dup_outpt(VOutPt* op, bool after, VattiScratch& sc) {
    VOutPt* r = sc.new_outpt(op->pt, op->outrec);
    if (after) { r->next=op->next; r->next->prev=r; r->prev=op; op->next=r; }
    else { r->prev=op->prev; r->prev->next=r; r->next=op; op->prev=r; }
    return r;
}

static void v_convert_horz_segs_to_joins(VattiScratch& sc) {
    // Update horz segments to find proper left/right ops
    int valid = 0;
    for (auto& hs : sc.horz_seg_list) {
        VOutPt* op = hs.left_op;
        VOutRec* outrec = op->outrec; while (outrec && !outrec->pts) outrec = outrec->owner;
        if (!outrec) { hs.right_op = nullptr; continue; }
        bool has_edges = outrec->front_edge != nullptr;
        int64_t cy = op->pt.y;
        VOutPt* opP = op, *opN = op;
        if (has_edges) {
            VOutPt* opA=outrec->pts, *opZ=opA->next;
            while (opP!=opZ && opP->prev->pt.y==cy) opP=opP->prev;
            while (opN!=opA && opN->next->pt.y==cy) opN=opN->next;
        } else {
            while (opP->prev!=opN && opP->prev->pt.y==cy) opP=opP->prev;
            while (opN->next!=opP && opN->next->pt.y==cy) opN=opN->next;
        }
        if (opP->pt.x == opN->pt.x) { hs.right_op=nullptr; continue; }
        if (opP->pt.x < opN->pt.x) { hs.left_op=opP; hs.right_op=opN; hs.left_to_right=true; }
        else { hs.left_op=opN; hs.right_op=opP; hs.left_to_right=false; }
        if (hs.left_op->horz) { hs.right_op=nullptr; continue; }
        hs.left_op->horz = reinterpret_cast<VHorzSeg*>(1); // mark used
        valid++;
    }
    if (valid < 2) return;
    std::stable_sort(sc.horz_seg_list.begin(), sc.horz_seg_list.end(),
        [](const VHorzSeg& a, const VHorzSeg& b) { if (!a.right_op||!b.right_op) return (a.right_op!=nullptr); return b.left_op->pt.x > a.left_op->pt.x; });
    int j = valid;
    for (int i=0; i<j-1; i++) {
        auto& hs1 = sc.horz_seg_list[i];
        for (int k=i+1; k<j; k++) {
            auto& hs2 = sc.horz_seg_list[k];
            if (hs2.left_op->pt.x >= hs1.right_op->pt.x || hs2.left_to_right==hs1.left_to_right || hs2.right_op->pt.x <= hs1.left_op->pt.x) continue;
            int64_t cy = hs1.left_op->pt.y;
            if (hs1.left_to_right) {
                while (hs1.left_op->next->pt.y==cy && hs1.left_op->next->pt.x<=hs2.left_op->pt.x) hs1.left_op=hs1.left_op->next;
                while (hs2.left_op->prev->pt.y==cy && hs2.left_op->prev->pt.x<=hs1.left_op->pt.x) hs2.left_op=hs2.left_op->prev;
                sc.horz_join_list.push_back({v_dup_outpt(hs1.left_op, true, sc), v_dup_outpt(hs2.left_op, false, sc)});
            } else {
                while (hs1.left_op->prev->pt.y==cy && hs1.left_op->prev->pt.x<=hs2.left_op->pt.x) hs1.left_op=hs1.left_op->prev;
                while (hs2.left_op->next->pt.y==cy && hs2.left_op->next->pt.x<=hs1.left_op->pt.x) hs2.left_op=hs2.left_op->next;
                sc.horz_join_list.push_back({v_dup_outpt(hs2.left_op, true, sc), v_dup_outpt(hs1.left_op, false, sc)});
            }
        }
    }
}

static void v_fix_outrec_pts(VOutRec* outrec) { VOutPt* op=outrec->pts; do { op->outrec=outrec; op=op->next; } while(op!=outrec->pts); }

static void v_process_horz_joins(VattiScratch& sc) {
    for (auto& j : sc.horz_join_list) {
        VOutRec* or1 = j.op1->outrec; while(or1 && !or1->pts) or1=or1->owner;
        VOutRec* or2 = j.op2->outrec; while(or2 && !or2->pts) or2=or2->owner;
        VOutPt* op1b=j.op1->next, *op2b=j.op2->prev;
        j.op1->next=j.op2; j.op2->prev=j.op1; op1b->prev=op2b; op2b->next=op1b;
        if (or1==or2) {
            or2 = sc.new_outrec(); or2->pts = op1b; v_fix_outrec_pts(or2);
            if (or1->pts->outrec==or2) { or1->pts=j.op1; or1->pts->outrec=or1; }
            or2->owner = or1;
        } else { or2->pts=nullptr; or2->owner=or1; }
    }
}

// ── Intersection detection (merge sort) ──────────────────────────────────

inline void v_adjust_curr_x_copy_to_sel(VattiScratch& sc, int64_t top_y) {
    VActive* e = sc.actives; sc.sel = e;
    while (e) {
        e->prev_in_sel=e->prev_in_ael; e->next_in_sel=e->next_in_ael; e->jump=e->next_in_sel;
        if (e->join_with==1) e->curr_x = e->prev_in_ael->curr_x;
        else e->curr_x = v_top_x(*e, top_y);
        e = e->next_in_ael;
    }
}

inline VActive* v_extract_from_sel(VActive* ae) {
    VActive* res = ae->next_in_sel; if (res) res->prev_in_sel = ae->prev_in_sel;
    ae->prev_in_sel->next_in_sel = res; return res;
}
inline void v_insert1_before2_in_sel(VActive* a1, VActive* a2) {
    a1->prev_in_sel=a2->prev_in_sel; if(a1->prev_in_sel) a1->prev_in_sel->next_in_sel=a1;
    a1->next_in_sel=a2; a2->prev_in_sel=a1;
}

static void v_add_new_isect_node(VattiScratch& sc, VActive& e1, VActive& e2, int64_t top_y) {
    BIVec2 ip;
    if (!v_get_seg_isect_pt(e1.bot, e1.top, e2.bot, e2.top, ip)) ip = {e1.curr_x, top_y};
    if (ip.y > sc.bot_y || ip.y < top_y) {
        double ad1=std::fabs(e1.dx), ad2=std::fabs(e2.dx);
        if (ad1>100 && ad2>100) ip = (ad1>ad2) ? v_closest_pt_on_seg(ip,e1.bot,e1.top) : v_closest_pt_on_seg(ip,e2.bot,e2.top);
        else if (ad1>100) ip = v_closest_pt_on_seg(ip,e1.bot,e1.top);
        else if (ad2>100) ip = v_closest_pt_on_seg(ip,e2.bot,e2.top);
        else { if (ip.y<top_y) ip.y=top_y; else ip.y=sc.bot_y; ip.x = (ad1<ad2) ? v_top_x(e1,ip.y) : v_top_x(e2,ip.y); }
    }
    sc.intersect_nodes.push_back({ip, &e1, &e2});
}

static bool v_build_intersect_list(VattiScratch& sc, int64_t top_y) {
    if (!sc.actives || !sc.actives->next_in_ael) return false;
    v_adjust_curr_x_copy_to_sel(sc, top_y);
    VActive* left = sc.sel;
    while (left && left->jump) {
        VActive* prev_base = nullptr;
        while (left && left->jump) {
            VActive* curr_base = left;
            VActive* right = left->jump;
            VActive* l_end = right;
            VActive* r_end = right->jump;
            left->jump = r_end;
            while (left != l_end && right != r_end) {
                if (right->curr_x < left->curr_x) {
                    VActive* tmp = right->prev_in_sel;
                    for (;;) { v_add_new_isect_node(sc, *tmp, *right, top_y); if (tmp==left) break; tmp=tmp->prev_in_sel; }
                    tmp = right; right = v_extract_from_sel(tmp); l_end = right;
                    v_insert1_before2_in_sel(tmp, left);
                    if (left==curr_base) { curr_base=tmp; curr_base->jump=r_end; if(!prev_base) sc.sel=curr_base; else prev_base->jump=curr_base; }
                } else left = left->next_in_sel;
            }
            prev_base = curr_base; left = r_end;
        }
        left = sc.sel;
    }
    return !sc.intersect_nodes.empty();
}

static void v_process_intersect_list(VattiScratch& sc, int cliptype) {
    std::sort(sc.intersect_nodes.begin(), sc.intersect_nodes.end(),
        [](const VIntersectNode& a, const VIntersectNode& b) { return (a.pt.y==b.pt.y) ? a.pt.x<b.pt.x : a.pt.y>b.pt.y; });
    for (size_t i = 0; i < sc.intersect_nodes.size(); i++) {
        auto& node = sc.intersect_nodes[i];
        if (!(node.edge1->next_in_ael==node.edge2 || node.edge1->prev_in_ael==node.edge2)) {
            for (size_t j=i+1; j<sc.intersect_nodes.size(); j++) {
                if (sc.intersect_nodes[j].edge1->next_in_ael==sc.intersect_nodes[j].edge2 || sc.intersect_nodes[j].edge1->prev_in_ael==sc.intersect_nodes[j].edge2)
                    { std::swap(sc.intersect_nodes[i], sc.intersect_nodes[j]); node = sc.intersect_nodes[i]; break; }
            }
        }
        v_intersect_edges(*node.edge1, *node.edge2, node.pt, sc, cliptype);
        v_swap_positions_in_ael(sc, *node.edge1, *node.edge2);
        node.edge1->curr_x = node.pt.x; node.edge2->curr_x = node.pt.x;
        v_check_join_left(*node.edge2, node.pt, sc, true);
        v_check_join_right(*node.edge1, node.pt, sc, true);
    }
}

// ── InsertLocalMinimaIntoAEL ─────────────────────────────────────────────

static void v_insert_local_minima_into_ael(VattiScratch& sc, int64_t bot_y, int cliptype) {
    VLocalMinima* lm;
    while (v_pop_locmin(sc, bot_y, lm)) {
        VActive* lb = sc.new_active();
        lb->bot = lm->vertex->pt; lb->curr_x = lb->bot.x; lb->wind_dx = -1;
        lb->vertex_top = lm->vertex->prev; lb->top = lb->vertex_top->pt;
        lb->local_min = lm; v_set_dx(*lb);

        VActive* rb = sc.new_active();
        rb->bot = lm->vertex->pt; rb->curr_x = rb->bot.x; rb->wind_dx = 1;
        rb->vertex_top = lm->vertex->next; rb->top = rb->vertex_top->pt;
        rb->local_min = lm; v_set_dx(*rb);

        if (v_is_horizontal(*lb)) {
            if (lb->dx == -std::numeric_limits<double>::max()) std::swap(lb, rb);
        } else if (v_is_horizontal(*rb)) {
            if (rb->dx == std::numeric_limits<double>::max()) std::swap(lb, rb);
        } else if (lb->dx < rb->dx) std::swap(lb, rb);

        lb->is_left_bound = true;
        v_insert_left_edge(sc, *lb);
        v_set_wind_count(sc, *lb);
        bool contributing = v_is_contributing(*lb, cliptype);

        rb->is_left_bound = false;
        rb->wind_cnt = lb->wind_cnt; rb->wind_cnt2 = lb->wind_cnt2;
        v_insert_right_edge(*lb, *rb);
        if (contributing) {
            v_add_local_min_poly(*lb, *rb, lb->bot, sc, true);
            if (!v_is_horizontal(*lb)) v_check_join_left(*lb, lb->bot, sc);
        }
        while (rb->next_in_ael && v_is_valid_ael_order(*rb->next_in_ael, *rb)) {
            v_intersect_edges(*rb, *rb->next_in_ael, rb->bot, sc, cliptype);
            v_swap_positions_in_ael(sc, *rb, *rb->next_in_ael);
        }
        if (v_is_horizontal(*rb)) v_push_horz(sc, *rb);
        else { v_check_join_right(*rb, rb->bot, sc); v_insert_scanline(sc, rb->top.y); }
        if (v_is_horizontal(*lb)) v_push_horz(sc, *lb);
        else v_insert_scanline(sc, lb->top.y);
    }
}

// ── DoMaxima ─────────────────────────────────────────────────────────────

static VActive* v_do_maxima(VActive& e, VattiScratch& sc, int cliptype) {
    VActive* prev_e = e.prev_in_ael;
    VActive* next_e = e.next_in_ael;
    VActive* max_pair = v_get_maxima_pair(e);
    if (!max_pair) return next_e;
    if (v_is_joined(e)) v_split(e, e.top, sc);
    if (v_is_joined(*max_pair)) v_split(*max_pair, max_pair->top, sc);
    while (next_e != max_pair) {
        v_intersect_edges(e, *next_e, e.top, sc, cliptype);
        v_swap_positions_in_ael(sc, e, *next_e);
        next_e = e.next_in_ael;
    }
    if (v_is_hot(e)) v_add_local_max_poly(e, *max_pair, e.top, sc);
    v_delete_from_ael(sc, *max_pair); v_delete_from_ael(sc, e);
    return prev_e ? prev_e->next_in_ael : sc.actives;
}

// ── DoTopOfScanbeam ──────────────────────────────────────────────────────

static void v_do_top_of_scanbeam(VattiScratch& sc, int64_t y, int cliptype) {
    sc.sel = nullptr;
    VActive* e = sc.actives;
    while (e) {
        if (e->top.y == y) {
            e->curr_x = e->top.x;
            if (v_is_maxima(*e)) { e = v_do_maxima(*e, sc, cliptype); continue; }
            if (v_is_hot(*e)) v_add_outpt(*e, e->top, sc);
            v_update_edge_into_ael(sc, e, cliptype);
            if (v_is_horizontal(*e)) v_push_horz(sc, *e);
        } else e->curr_x = v_top_x(*e, y);
        e = e->next_in_ael;
    }
}

// ── CleanCollinear + FixSelfIntersects ───────────────────────────────────

static VOutPt* v_dispose_outpt(VOutPt* op) {
    VOutPt* r = op->next; op->prev->next=op->next; op->next->prev=op->prev; return r;
}

static void v_do_split_op(VattiScratch& sc, VOutRec* outrec, VOutPt* splitOp) {
    VOutPt* prevOp = splitOp->prev; VOutPt* nnOp = splitOp->next->next;
    outrec->pts = prevOp;
    BIVec2 ip;
    v_get_seg_isect_pt(prevOp->pt, splitOp->pt, splitOp->next->pt, nnOp->pt, ip);
    double area1 = v_area_outpt(outrec->pts);
    if (std::fabs(area1) < 2) { outrec->pts=nullptr; return; }
    double area2 = v_area_tri(ip, splitOp->pt, splitOp->next->pt);
    double absA2 = std::fabs(area2);
    if (ip==prevOp->pt || ip==nnOp->pt) { nnOp->prev=prevOp; prevOp->next=nnOp; }
    else {
        VOutPt* nop = sc.new_outpt(ip, prevOp->outrec);
        nop->prev=prevOp; nop->next=nnOp; nnOp->prev=nop; prevOp->next=nop;
    }
    if (absA2>=1 && (absA2>std::fabs(area1) || (area2>0)==(area1>0))) {
        VOutRec* nr = sc.new_outrec(); nr->owner=outrec->owner;
        splitOp->outrec=nr; splitOp->next->outrec=nr;
        VOutPt* nop = sc.new_outpt(ip, nr);
        nop->prev=splitOp->next; nop->next=splitOp; nr->pts=nop;
        splitOp->prev=nop; splitOp->next->next=nop;
    }
}

static void v_fix_self_intersects(VattiScratch& sc, VOutRec* outrec) {
    VOutPt* op2 = outrec->pts;
    for (;;) {
        if (op2->prev == op2->next->next) break;
        if (v_segs_intersect(op2->prev->pt, op2->pt, op2->next->pt, op2->next->next->pt)) {
            if (op2==outrec->pts || op2->next==outrec->pts) outrec->pts=outrec->pts->prev;
            v_do_split_op(sc, outrec, op2);
            if (!outrec->pts) break;
            op2 = outrec->pts; continue;
        }
        op2 = op2->next;
        if (op2 == outrec->pts) break;
    }
}

static void v_clean_collinear(VattiScratch& sc, VOutRec* outrec) {
    while (outrec && !outrec->pts) outrec=outrec->owner;
    if (!outrec) return;
    if (!v_valid_closed(outrec->pts)) { outrec->pts=nullptr; return; }
    VOutPt* startOp = outrec->pts, *op2 = startOp;
    for (;;) {
        if (v_is_collinear(op2->prev->pt, op2->pt, op2->next->pt) &&
            (op2->pt==op2->prev->pt || op2->pt==op2->next->pt || v_dot_product(op2->prev->pt, op2->pt, op2->next->pt)<0)) {
            if (op2==outrec->pts) outrec->pts=op2->prev;
            op2 = v_dispose_outpt(op2);
            if (!v_valid_closed(op2)) { outrec->pts=nullptr; return; }
            startOp = op2; continue;
        }
        op2 = op2->next;
        if (op2==startOp) break;
    }
    v_fix_self_intersects(sc, outrec);
}

// ── ExecuteInternal ──────────────────────────────────────────────────────

static bool v_execute_internal(VattiScratch& sc, int cliptype) {
    std::stable_sort(sc.locmin_list.begin(), sc.locmin_list.end(),
        [](const VLocalMinima& a, const VLocalMinima& b) {
            if (b.vertex->pt.y != a.vertex->pt.y) return b.vertex->pt.y < a.vertex->pt.y;
            return b.vertex->pt.x > a.vertex->pt.x;
        });
    for (auto& lm : sc.locmin_list) v_insert_scanline(sc, lm.vertex->pt.y);
    sc.locmin_idx = 0;

    int64_t y;
    if (!v_pop_scanline(sc, y)) return true;
    while (sc.succeeded) {
        v_insert_local_minima_into_ael(sc, y, cliptype);
        VActive* e;
        while (v_pop_horz(sc, e)) v_do_horizontal(*e, sc, cliptype);
        if (!sc.horz_seg_list.empty()) { v_convert_horz_segs_to_joins(sc); sc.horz_seg_list.clear(); }
        sc.bot_y = y;
        if (!v_pop_scanline(sc, y)) break;
        if (sc.succeeded && v_build_intersect_list(sc, y)) { v_process_intersect_list(sc, cliptype); sc.intersect_nodes.clear(); }
        v_do_top_of_scanbeam(sc, y, cliptype);
        while (v_pop_horz(sc, e)) v_do_horizontal(*e, sc, cliptype);
    }
    if (sc.succeeded) v_process_horz_joins(sc);
    return sc.succeeded;
}

// ── Build output paths from OutRec list ──────────────────────────────────

static bool v_build_path(VOutPt* op, std::vector<BIVec2>& path) {
    if (!op || op->next==op || op->next==op->prev) return false;
    path.clear();
    BIVec2 last = op->next->pt;
    VOutPt* op2 = op->next->next;
    path.push_back(last);
    while (op2 != op->next) { if (op2->pt != last) { last=op2->pt; path.push_back(last); } op2=op2->next; }
    return path.size() >= 3 && !v_very_small_tri(*op);
}

} // anonymous namespace

std::vector<Polyline> Polyline::boolean_op(const Polyline& a, const Polyline& b, int clip_type) {
    const double* ca = a._coords.data();
    const double* cb = b._coords.data();
    int na = (int)(a._coords.size() / 3);
    int nb = (int)(b._coords.size() / 3);

    // Strip closing duplicate
    if (na>=2) { double dx=ca[(na-1)*3]-ca[0],dy=ca[(na-1)*3+1]-ca[1]; if(dx*dx+dy*dy<1e-20) --na; }
    if (nb>=2) { double dx=cb[(nb-1)*3]-cb[0],dy=cb[(nb-1)*3+1]-cb[1]; if(dx*dx+dy*dy<1e-20) --nb; }
    if (na < 3 || nb < 3) return {};

    // Fixed-point scaling: coords assumed < 2.3e9 (covers all practical geometry).
    // Scale = 1e9 → 9 decimal digits of sub-unit precision, max safe coord ~2.3e9.
    // This eliminates the max-abs scan entirely.
    constexpr double BOOL_SCALE = 1e9;
    constexpr double BOOL_INV_SCALE = 1e-9;
    VattiScratch& sc = vtls; sc.reset();
    auto& va = sc.va; va.resize(na);
    auto& vb = sc.vb; vb.resize(nb);
    double scale = BOOL_SCALE;

    // Scale + AABB in one pass. NEARBYINT with constant scale → compiler can
    // use a single fused multiply-convert instruction per coordinate.
    int64_t aMinX,aMaxX,aMinY,aMaxY,bMinX,bMaxX,bMinY,bMaxY;
    {
        const double S = BOOL_SCALE;
        int64_t x=VATTI_NEARBYINT(ca[0]*S), y=VATTI_NEARBYINT(ca[1]*S);
        va[0]={x,y}; aMinX=aMaxX=x; aMinY=aMaxY=y;
        for (int i=1;i<na;i++) {
            x=VATTI_NEARBYINT(ca[i*3]*S); y=VATTI_NEARBYINT(ca[i*3+1]*S);
            va[i]={x,y};
            if(x<aMinX) aMinX=x; else if(x>aMaxX) aMaxX=x;
            if(y<aMinY) aMinY=y; else if(y>aMaxY) aMaxY=y;
        }
        x=VATTI_NEARBYINT(cb[0]*S); y=VATTI_NEARBYINT(cb[1]*S);
        vb[0]={x,y}; bMinX=bMaxX=x; bMinY=bMaxY=y;
        for (int i=1;i<nb;i++) {
            x=VATTI_NEARBYINT(cb[i*3]*S); y=VATTI_NEARBYINT(cb[i*3+1]*S);
            vb[i]={x,y};
            if(x<bMinX) bMinX=x; else if(x>bMaxX) bMaxX=x;
            if(y<bMinY) bMinY=y; else if(y>bMaxY) bMaxY=y;
        }
    }

    // AABB disjoint → containment fast-path
    if (aMaxX < bMinX || bMaxX < aMinX || aMaxY < bMinY || bMaxY < aMinY) {
        bool a_in_b = pip_i(va[0], vb), b_in_a = pip_i(vb[0], va);
        if (clip_type == 0) { if (a_in_b) return {a}; if (b_in_a) return {b}; return {}; }
        if (clip_type == 1) { if (a_in_b) return {b}; if (b_in_a) return {a}; return {a, b}; }
        if (a_in_b) return {}; if (b_in_a) return {a}; return {a};
    }

    // Small polygon containment test (no edge crossings → pure containment)
    if ((int64_t)na * nb <= 10000) {
        bool any_cross = false;
        for (int i = 0; i < na && !any_cross; i++) {
            BIVec2 a1=va[i], a2=va[(i+1)%na];
            int64_t axmin=std::min(a1.x,a2.x), axmax=std::max(a1.x,a2.x);
            int64_t aymin=std::min(a1.y,a2.y), aymax=std::max(a1.y,a2.y);
            for (int j = 0; j < nb && !any_cross; j++) {
                BIVec2 b1=vb[j], b2=vb[(j+1)%nb];
                if (std::max(b1.x,b2.x)<axmin||std::min(b1.x,b2.x)>axmax||
                    std::max(b1.y,b2.y)<aymin||std::min(b1.y,b2.y)>aymax) continue;
                any_cross = v_segs_intersect(a1,a2,b1,b2);
            }
        }
        if (!any_cross) {
            bool a_in_b=pip_i(va[0],vb), b_in_a=pip_i(vb[0],va);
            if (clip_type==0){if(a_in_b)return{a};if(b_in_a)return{b};return{};}
            if (clip_type==1){if(a_in_b)return{b};if(b_in_a)return{a};return{a,b};}
            if (a_in_b) return {}; if (b_in_a) return {a}; return {a};
        }
    }

    // Pre-reserve pools + run Vatti
    int total = na + nb;
    sc.vtx_pool.ensure(total + 4);
    sc.act_pool.ensure(total * 2 + 4);
    sc.opt_pool.ensure(total * 4);
    sc.orc_pool.ensure(total);
    sc.locmin_list.reserve(total);
    sc.outrec_list.reserve(total);
    sc.scanline_list.buf.reserve(total * 2);

    v_add_path(va, na, 0, sc);
    v_add_path(vb, nb, 1, sc);
    if (!v_execute_internal(sc, clip_type)) return {};

    // Extract: OutPt → Polyline._coords (single pass, z=0)
    constexpr double inv_scale = BOOL_INV_SCALE;
    std::vector<Polyline> out;
    for (size_t i = 0; i < sc.outrec_list.size(); i++) {
        VOutRec* outrec = sc.outrec_list[i];
        if (!outrec->pts) continue;
        v_clean_collinear(sc, outrec);
        if (!outrec->pts) continue;
        VOutPt* op = outrec->pts;
        if (!op || op->next==op || op->next==op->prev || v_very_small_tri(*op)) continue;
        int cnt = 0;
        { VOutPt* o = op->next; BIVec2 last = o->pt; cnt = 1;
          for (o = o->next; o != op->next; o = o->next) if (o->pt != last) { last = o->pt; cnt++; } }
        if (cnt < 3) continue;
        Polyline result;
        result._coords.resize(cnt * 3);
        double* dst = result._coords.data();
        VOutPt* o = op->next; BIVec2 last = o->pt;
        dst[0]=last.x*inv_scale; dst[1]=last.y*inv_scale; dst[2]=0.0; dst+=3;
        for (o=o->next; o!=op->next; o=o->next) {
            if (o->pt==last) continue; last=o->pt;
            dst[0]=last.x*inv_scale; dst[1]=last.y*inv_scale; dst[2]=0.0; dst+=3;
        }
        out.push_back(std::move(result));
    }
    return out;
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
