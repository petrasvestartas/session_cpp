#include "polyline.h"
#include "tolerance.h"
#include <cmath>
#include <sstream>
#include <array>
#include <functional>
#include <limits>
#include <queue>
#include <set>

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

namespace {

struct BIVec2 { int64_t x, y; };

struct BAVertex {
    BIVec2 pt;
    bool   is_alpha  = false;
    bool   entering  = false;
    int    isect_id  = -1;
    int    neighbor  = -1;
};

struct BIsect { BIVec2 pt; int ea, eb, id; double s, t; };

struct BOScratch {
    std::vector<BIVec2>   va, vb;
    std::vector<BIsect>   isects;
    std::vector<BAVertex> listA, listB;
    std::vector<int>      posA, posB;
    std::vector<bool>     usedA;
    std::vector<double>   poly_coords;
};
static thread_local BOScratch tls;

static int64_t cross2i(BIVec2 a, BIVec2 b) { return a.x * b.y - a.y * b.x; }

static bool pip_i(BIVec2 pt, const std::vector<BIVec2>& poly) {
    int winding = 0, n = (int)poly.size();
    for (int i = 0; i < n; i++) {
        BIVec2 a = poly[i], b = poly[(i+1)%n];
        if (a.y <= pt.y) {
            if (b.y > pt.y && cross2i({b.x-a.x, b.y-a.y}, {pt.x-a.x, pt.y-a.y}) > 0) ++winding;
        } else {
            if (b.y <= pt.y && cross2i({b.x-a.x, b.y-a.y}, {pt.x-a.x, pt.y-a.y}) < 0) --winding;
        }
    }
    return winding != 0;
}

static bool on_seg_i(BIVec2 p, BIVec2 a, BIVec2 b) {
    BIVec2 d = {b.x-a.x, b.y-a.y}, e = {p.x-a.x, p.y-a.y};
    if (cross2i(d, e) != 0) return false;
    int64_t dd = d.x*d.x + d.y*d.y;
    if (!dd) return false;
    int64_t de = d.x*e.x + d.y*e.y;
    return de > 0 && de < dd;
}

static BIVec2 perturb_off_seg(BIVec2 p, BIVec2 a, BIVec2 b) {
    int64_t dx = b.x-a.x, dy = b.y-a.y;
    if (std::abs(dy) >= std::abs(dx)) p.x += (dy > 0) ? 1 : -1;
    else                              p.y += (dx > 0) ? 1 : -1;
    return p;
}

static bool seg_isect_i(BIVec2 a, BIVec2 b, BIVec2 c, BIVec2 d,
                        double& s, double& t, BIVec2& pt) {
    BIVec2 ab = {b.x-a.x, b.y-a.y}, cd = {d.x-c.x, d.y-c.y};
    int64_t denom = cross2i(ab, cd);
    if (!denom) return false;
    BIVec2 ac = {c.x-a.x, c.y-a.y};
    int64_t sn = cross2i(ac, cd), tn = cross2i(ac, ab);
    auto in01 = [](int64_t num, int64_t den) {
        return den > 0 ? (num > 0 && num < den) : (num < 0 && num > den);
    };
    if (!in01(sn, denom) || !in01(tn, denom)) return false;
    s = (double)sn / (double)denom;
    t = (double)tn / (double)denom;
    pt = {a.x + (int64_t)std::round(s*ab.x), a.y + (int64_t)std::round(s*ab.y)};
    return true;
}

// Sweep-line O((n+k) log n) A-B intersection finder.
// Events: kind 0=RIGHT(remove) 1=ISECT(swap) 2=LEFT(insert)
// At same (x,y): RIGHT first, then ISECT, then LEFT — keeps AET consistent.
static void find_isects_sweep(
    const std::vector<BIVec2>& va, int na,
    const std::vector<BIVec2>& vb, int nb,
    std::vector<BIsect>& isects)
{
    struct Ev {
        int64_t x, y;
        int8_t  kind;  // 0=right, 1=isect, 2=left
        int8_t  poly;
        int     ia, ib;
        double  s, t;
        BIVec2  pt;
        bool operator>(const Ev& o) const {
            if (x != o.x) return x > o.x;
            if (y != o.y) return y > o.y;
            return (int)kind > (int)o.kind;
        }
    };
    std::priority_queue<Ev, std::vector<Ev>, std::greater<Ev>> Q;

    for (int i = 0; i < na; i++) {
        BIVec2 p=va[i], q=va[(i+1)%na];
        bool pl=p.x<q.x||(p.x==q.x&&p.y<=q.y);
        BIVec2 L=pl?p:q, R=pl?q:p;
        Q.push({L.x,L.y,2,0,i,-1,0,0,L}); Q.push({R.x,R.y,0,0,i,-1,0,0,R});
    }
    for (int j = 0; j < nb; j++) {
        BIVec2 p=vb[j], q=vb[(j+1)%nb];
        bool pl=p.x<q.x||(p.x==q.x&&p.y<=q.y);
        BIVec2 L=pl?p:q, R=pl?q:p;
        Q.push({L.x,L.y,2,1,j,-1,0,0,L}); Q.push({R.x,R.y,0,1,j,-1,0,0,R});
    }

    int64_t cur_x = std::numeric_limits<int64_t>::min();
    struct AE { int8_t poly; int idx; };
    std::vector<AE> aet;
    aet.reserve(16);

    auto yat = [&](const AE& e, int64_t x) -> double {
        const auto& v=(e.poly==0)?va:vb; int n=(e.poly==0)?na:nb;
        BIVec2 p=v[e.idx],q=v[(e.idx+1)%n];
        if(p.x==q.x) return (double)std::min(p.y,q.y);
        return p.y+(double)(q.y-p.y)*(double)(x-p.x)/(double)(q.x-p.x);
    };

    std::set<std::pair<int,int>> queued;

    auto maybe_q = [&](int i) {
        if(i<0||i+1>=(int)aet.size()) return;
        if(aet[i].poly==aet[i+1].poly) return;
        int ea=aet[i].poly==0?aet[i].idx:aet[i+1].idx;
        int eb=aet[i].poly==0?aet[i+1].idx:aet[i].idx;
        if(!queued.insert({ea,eb}).second) return;
        double s,t; BIVec2 pt;
        if(seg_isect_i(va[ea],va[(ea+1)%na],vb[eb],vb[(eb+1)%nb],s,t,pt)&&pt.x>=cur_x)
            Q.push({pt.x,pt.y,1,-1,ea,eb,s,t,pt});
        else queued.erase({ea,eb});
    };

    while(!Q.empty()) {
        auto ev=Q.top(); Q.pop();
        cur_x=ev.x;
        if(ev.kind==2) { // LEFT: insert
            double yi=yat({ev.poly,ev.ia},cur_x);
            int pos=0;
            while(pos<(int)aet.size()&&yat(aet[pos],cur_x)<yi) pos++;
            aet.insert(aet.begin()+pos,{ev.poly,ev.ia});
            maybe_q(pos-1); maybe_q(pos);
        } else if(ev.kind==0) { // RIGHT: remove
            for(int i=0;i<(int)aet.size();i++) {
                if(aet[i].poly==ev.poly&&aet[i].idx==ev.ia) {
                    aet.erase(aet.begin()+i);
                    if(i>0&&i<(int)aet.size()) maybe_q(i-1);
                    break;
                }
            }
        } else { // ISECT: record + swap
            isects.push_back({ev.pt,ev.ia,ev.ib,(int)isects.size(),ev.s,ev.t});
            queued.erase({ev.ia,ev.ib});
            int pa=-1,pb=-1;
            for(int i=0;i<(int)aet.size();i++) {
                if(aet[i].poly==0&&aet[i].idx==ev.ia) pa=i;
                if(aet[i].poly==1&&aet[i].idx==ev.ib) pb=i;
            }
            if(pa>=0&&pb>=0&&std::abs(pa-pb)==1) {
                int lo=std::min(pa,pb);
                std::swap(aet[lo],aet[lo+1]);
                maybe_q(lo-1); maybe_q(lo+1);
            }
        }
    }
}

} // anonymous namespace

std::vector<Polyline> Polyline::boolean_op(const Polyline& a, const Polyline& b, int clip_type) {
    // Work directly on _coords — never construct Point/Vector/Polyline objects in the hot path.
    const double* ca = a._coords.data();
    const double* cb = b._coords.data();
    int na = (int)(a._coords.size() / 3);
    int nb = (int)(b._coords.size() / 3);

    // Strip closing duplicate if present
    auto dup_last = [](const double* c, int n) {
        if (n < 2) return false;
        double dx=c[(n-1)*3]-c[0], dy=c[(n-1)*3+1]-c[1], dz=c[(n-1)*3+2]-c[2];
        return dx*dx+dy*dy+dz*dz < 1e-20;
    };
    if (dup_last(ca, na)) --na;
    if (dup_last(cb, nb)) --nb;
    if (na < 3 || nb < 3) return {};

    // 2D fast path: if all z≈0, skip Newell normal and projection entirely.
    // Just scale x,y directly — eliminates ~3 full vertex passes + 2 sqrts.
    bool flat=true;
    for(int i=0;i<na&&flat;i++) flat=std::abs(ca[i*3+2])<1e-10;
    for(int i=0;i<nb&&flat;i++) flat=std::abs(cb[i*3+2])<1e-10;

    double ox=0,oy=0,oz=0;
    double xax,xay,xaz,yax,yay,yaz,scale;

    constexpr int64_t INT_SAFE_MAX=2305843009213693952LL;
    BOScratch* sc = &tls;
    auto& va = sc->va; va.resize(na);
    auto& vb = sc->vb; vb.resize(nb);

    if(flat) {
        xax=1;xay=0;xaz=0; yax=0;yay=1;yaz=0;
        double max_abs=1.0;
        for(int i=0;i<na;i++) max_abs=std::max(max_abs,std::max(std::abs(ca[i*3]),std::abs(ca[i*3+1])));
        for(int i=0;i<nb;i++) max_abs=std::max(max_abs,std::max(std::abs(cb[i*3]),std::abs(cb[i*3+1])));
        int prec=std::clamp((int)std::floor(std::log10((double)INT_SAFE_MAX/max_abs)),0,9);
        scale=std::pow(10.0,prec);
        for(int i=0;i<na;i++) va[i]={(int64_t)std::round(ca[i*3]*scale),(int64_t)std::round(ca[i*3+1]*scale)};
        for(int i=0;i<nb;i++) vb[i]={(int64_t)std::round(cb[i*3]*scale),(int64_t)std::round(cb[i*3+1]*scale)};
    } else {
        // Full 3D projection: centroid → Newell normal → axis setup → per-vertex dot product
        for(int i=0;i<na;i++){ox+=ca[i*3];oy+=ca[i*3+1];oz+=ca[i*3+2];}
        for(int i=0;i<nb;i++){ox+=cb[i*3];oy+=cb[i*3+1];oz+=cb[i*3+2];}
        double inv_tot=1.0/(na+nb); ox*=inv_tot; oy*=inv_tot; oz*=inv_tot;
        xax=ca[3]-ca[0];xay=ca[4]-ca[1];xaz=ca[5]-ca[2];
        double xl=std::sqrt(xax*xax+xay*xay+xaz*xaz);
        if(xl>1e-12){xax/=xl;xay/=xl;xaz/=xl;}else{xax=1;xay=0;xaz=0;}
        double znx=0,zny=0,znz=0;
        for(int i=0;i<na;i++){
            int j=(i+1)%na;
            double ax=ca[i*3]-ca[0],ay=ca[i*3+1]-ca[1],az=ca[i*3+2]-ca[2];
            double bx=ca[j*3]-ca[0],by=ca[j*3+1]-ca[1],bz=ca[j*3+2]-ca[2];
            double v2x=bx-ax,v2y=by-ay,v2z=bz-az;
            znx+=ay*v2z-az*v2y; zny+=az*v2x-ax*v2z; znz+=ax*v2y-ay*v2x;
        }
        double zl=std::sqrt(znx*znx+zny*zny+znz*znz);
        if(zl>1e-12){znx/=zl;zny/=zl;znz/=zl;}else{znx=0;zny=0;znz=1;}
        yax=zny*xaz-znz*xay; yay=znz*xax-znx*xaz; yaz=znx*xay-zny*xax;
        double yl=std::sqrt(yax*yax+yay*yay+yaz*yaz);
        if(yl>1e-12){yax/=yl;yay/=yl;yaz/=yl;}else{yax=0;yay=1;yaz=0;}
        double max_abs=1.0;
        auto scan_max=[&](const double* c,int n){
            for(int i=0;i<n;i++){
                double dx=c[i*3]-ox,dy=c[i*3+1]-oy,dz=c[i*3+2]-oz;
                double u=dx*xax+dy*xay+dz*xaz, v=dx*yax+dy*yay+dz*yaz;
                double m=std::max(std::abs(u),std::abs(v));
                if(m>max_abs) max_abs=m;
            }
        };
        scan_max(ca,na); scan_max(cb,nb);
        int prec=std::clamp((int)std::floor(std::log10((double)INT_SAFE_MAX/max_abs)),0,9);
        scale=std::pow(10.0,prec);
        auto proj_c=[&](const double* c,int i)->BIVec2{
            double dx=c[i*3]-ox,dy=c[i*3+1]-oy,dz=c[i*3+2]-oz;
            double u=dx*xax+dy*xay+dz*xaz, v=dx*yax+dy*yay+dz*yaz;
            return{(int64_t)std::round(u*scale),(int64_t)std::round(v*scale)};
        };
        for(int i=0;i<na;i++) va[i]=proj_c(ca,i);
        for(int i=0;i<nb;i++) vb[i]=proj_c(cb,i);
    }

    // AABB pre-check — reject disjoint pairs before any O(na*nb) work
    {
        int64_t aMinX=va[0].x,aMaxX=va[0].x,aMinY=va[0].y,aMaxY=va[0].y;
        for (auto& p:va){aMinX=std::min(aMinX,p.x);aMaxX=std::max(aMaxX,p.x);aMinY=std::min(aMinY,p.y);aMaxY=std::max(aMaxY,p.y);}
        int64_t bMinX=vb[0].x,bMaxX=vb[0].x,bMinY=vb[0].y,bMaxY=vb[0].y;
        for (auto& p:vb){bMinX=std::min(bMinX,p.x);bMaxX=std::max(bMaxX,p.x);bMinY=std::min(bMinY,p.y);bMaxY=std::max(bMaxY,p.y);}
        if (aMaxX < bMinX || bMaxX < aMinX || aMaxY < bMinY || bMaxY < aMinY) {
            // Disjoint bounding boxes — skip all intersection work
            bool a_in_b = pip_i(va[0], vb), b_in_a = pip_i(vb[0], va);
            if (clip_type == 0) { if (a_in_b) return {a}; if (b_in_a) return {b}; return {}; }
            if (clip_type == 1) { if (a_in_b) return {b}; if (b_in_a) return {a}; return {a, b}; }
            if (a_in_b) return {}; if (b_in_a) return {a}; return {a};
        }
    }

    // Degenerate handling: perturb vertices coincident with or lying on the other polygon's edges
    for (int i = 0; i < na; i++) {
        for (int j = 0; j < nb; j++) {
            if (va[i].x == vb[j].x && va[i].y == vb[j].y) {
                BIVec2 prev_b=vb[(j-1+nb)%nb], next_b=vb[(j+1)%nb];
                va[i]=perturb_off_seg(va[i],prev_b,vb[j]);
                if(va[i].x==vb[j].x&&va[i].y==vb[j].y) va[i]=perturb_off_seg(va[i],vb[j],next_b);
                goto next_va;
            }
        }
        for (int j = 0; j < nb; j++) {
            if (on_seg_i(va[i],vb[j],vb[(j+1)%nb])) {
                va[i]=perturb_off_seg(va[i],vb[j],vb[(j+1)%nb]); goto next_va;
            }
        }
        next_va:;
    }
    for (int j = 0; j < nb; j++) {
        for (int i = 0; i < na; i++) {
            if (vb[j].x == va[i].x && vb[j].y == va[i].y) {
                BIVec2 prev_a=va[(i-1+na)%na], next_a=va[(i+1)%na];
                vb[j]=perturb_off_seg(vb[j],prev_a,va[i]);
                if(vb[j].x==va[i].x&&vb[j].y==va[i].y) vb[j]=perturb_off_seg(vb[j],va[i],next_a);
                goto next_vb;
            }
        }
        for (int i = 0; i < na; i++) {
            if (on_seg_i(vb[j],va[i],va[(i+1)%na])) {
                vb[j]=perturb_off_seg(vb[j],va[i],va[(i+1)%na]); goto next_vb;
            }
        }
        next_vb:;
    }

    // Find all proper A-B intersections using sweep-line O((n+k) log n)
    auto& isects = sc->isects; isects.clear();
    find_isects_sweep(va, na, vb, nb, isects);
    int ni = (int)isects.size();

    if (ni == 0) {
        bool a_in_b=pip_i(va[0],vb), b_in_a=pip_i(vb[0],va);
        if (clip_type==0){if(a_in_b)return{a};if(b_in_a)return{b};return{};}
        if (clip_type==1){if(a_in_b)return{b};if(b_in_a)return{a};return{a,b};}
        if (a_in_b) return {}; if (b_in_a) return {a}; return {a};
    }

    auto& listA=sc->listA; listA.clear(); listA.reserve(na+ni);
    auto& listB=sc->listB; listB.clear(); listB.reserve(nb+ni);

    std::sort(isects.begin(),isects.end(),[](const BIsect& x,const BIsect& y){
        return x.ea<y.ea||(x.ea==y.ea&&x.s<y.s);
    });
    for(int i=0,k=0;i<na;i++){
        listA.push_back({va[i]});
        while(k<ni&&isects[k].ea==i){BAVertex av;av.pt=isects[k].pt;av.is_alpha=true;av.isect_id=isects[k].id;listA.push_back(av);k++;}
    }
    std::sort(isects.begin(),isects.end(),[](const BIsect& x,const BIsect& y){
        return x.eb<y.eb||(x.eb==y.eb&&x.t<y.t);
    });
    for(int j=0,k=0;j<nb;j++){
        listB.push_back({vb[j]});
        while(k<ni&&isects[k].eb==j){BAVertex av;av.pt=isects[k].pt;av.is_alpha=true;av.isect_id=isects[k].id;listB.push_back(av);k++;}
    }

    int lenA=(int)listA.size(), lenB=(int)listB.size();
    auto& posA=sc->posA; posA.assign(ni,-1);
    auto& posB=sc->posB; posB.assign(ni,-1);
    for(int i=0;i<lenA;i++) if(listA[i].is_alpha) posA[listA[i].isect_id]=i;
    for(int j=0;j<lenB;j++) if(listB[j].is_alpha) posB[listB[j].isect_id]=j;
    for(int k=0;k<ni;k++) if(posA[k]>=0&&posB[k]>=0){listA[posA[k]].neighbor=posB[k];listB[posB[k]].neighbor=posA[k];}

    {
        int first=-1;
        for(int i=0;i<lenA&&first<0;i++) if(!listA[i].is_alpha) first=i;
        if(first>=0){
            bool cur_inside=pip_i(listA[first].pt,vb);
            for(int k=1;k<=lenA;k++){
                int i=(first+k)%lenA;
                if(!listA[i].is_alpha) continue;
                listA[i].entering=!cur_inside; cur_inside=!cur_inside;
                if(listA[i].neighbor>=0) listB[listA[i].neighbor].entering=listA[i].entering;
            }
        }
    }

    auto& usedA=sc->usedA; usedA.assign(lenA,false);
    int find_cursor=0;
    auto find_start=[&]()->int{
        for(int k=0;k<lenA;k++){
            int i=(find_cursor+k)%lenA;
            if(!listA[i].is_alpha||usedA[i]) continue;
            if(clip_type==0&&!listA[i].entering) continue;
            if(clip_type!=0&& listA[i].entering) continue;
            find_cursor=i; return i;
        }
        return -1;
    };

    auto push_coord=[&](BIVec2 iv){
        double u=iv.x/scale, v=iv.y/scale;
        if(flat){
            sc->poly_coords.push_back(u);
            sc->poly_coords.push_back(v);
            sc->poly_coords.push_back(0.0);
        } else {
            sc->poly_coords.push_back(ox+u*xax+v*yax);
            sc->poly_coords.push_back(oy+u*xay+v*yay);
            sc->poly_coords.push_back(oz+u*xaz+v*yaz);
        }
    };

    std::vector<Polyline> out;
    int start;
    while((start=find_start())>=0){
        sc->poly_coords.clear();
        push_coord(listA[start].pt); usedA[start]=true;
        bool on_a=true; int cur=start;
        for(int iter=0;iter<lenA+lenB+10;iter++){
            int next;
            if(on_a) next=(cur+1)%lenA;
            else if(clip_type==2) next=(cur-1+lenB)%lenB;
            else next=(cur+1)%lenB;
            cur=next;
            BIVec2 cpt=on_a?listA[cur].pt:listB[cur].pt;
            int64_t dx=cpt.x-listA[start].pt.x, dy=cpt.y-listA[start].pt.y;
            if(dx==0&&dy==0) break;
            push_coord(cpt);
            bool is_alpha=on_a?listA[cur].is_alpha:listB[cur].is_alpha;
            if(!is_alpha) continue;
            bool ent=on_a?listA[cur].entering:listB[cur].entering;
            bool sw;
            if(on_a)             sw=(clip_type==0)?!ent:ent;
            else if(clip_type==0) sw=ent;
            else if(clip_type==1) sw=!ent;
            else                  sw=ent;
            if(!sw) continue;
            int nb_idx=on_a?listA[cur].neighbor:listB[cur].neighbor;
            if(nb_idx<0) break;
            if(on_a) usedA[cur]=true;
            on_a=!on_a; cur=nb_idx;
        }
        if((int)sc->poly_coords.size()>=9){
            Polyline result;
            result._coords=sc->poly_coords;
            out.push_back(std::move(result));
        }
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
