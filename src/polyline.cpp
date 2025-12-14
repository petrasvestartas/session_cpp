#include "polyline.h"
#include "tolerance.h"
#include <cmath>
#include <sstream>

#ifdef ENABLE_PROTOBUF
#include "polyline.pb.h"
#include "point.pb.h"
#include "xform.pb.h"
#endif

namespace session_cpp {

Polyline::Polyline() : guid(::guid()), name("my_polyline"), plane(Plane()) {}

/// Copy constructor (creates a new guid while copying data)
Polyline::Polyline(const Polyline& other)
    : guid(::guid()),
      name(other.name),
      _coords(other._coords),
      plane(other.plane),
      width(other.width),
      linecolor(other.linecolor),
      xform(other.xform) {}

/// Copy assignment (creates a new guid while copying data)
Polyline& Polyline::operator=(const Polyline& other) {
    if (this != &other) {
        guid = ::guid();
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
    : guid(::guid()), name("my_polyline") {
    // Convert points to flat coords
    _coords.reserve(pts.size() * 3);
    for (const auto& p : pts) {
        _coords.push_back(p[0]);
        _coords.push_back(p[1]);
        _coords.push_back(p[2]);
    }
    recompute_plane_if_needed();
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
    nlohmann::ordered_json j;
    j["type"] = "Polyline";
    j["guid"] = guid;
    j["name"] = name;
    j["coords"] = _coords;
    j["width"] = width;
    j["linecolor"] = linecolor.jsondump();
    j["xform"] = xform.jsondump();
    return j;
}

Polyline Polyline::jsonload(const nlohmann::json& data) {
    Polyline polyline;
    polyline.guid = data["guid"];
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

#ifdef ENABLE_PROTOBUF
std::string Polyline::to_protobuf() const {
    session_proto::Polyline proto;
    proto.set_guid(this->guid);
    proto.set_name(this->name);
    
    // Add points
    for (const auto& pt : get_points()) {
        auto* point_proto = proto.add_points();
        point_proto->set_guid(pt.guid);
        point_proto->set_name(pt.name);
        point_proto->set_x(pt[0]);
        point_proto->set_y(pt[1]);
        point_proto->set_z(pt[2]);
    }
    
    // Set xform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }
    
    return proto.SerializeAsString();
}

Polyline Polyline::from_protobuf(const std::string& data) {
    session_proto::Polyline proto;
    proto.ParseFromString(data);
    
    std::vector<Point> pts;
    pts.reserve(proto.points_size());
    for (int i = 0; i < proto.points_size(); ++i) {
        const auto& pt = proto.points(i);
        Point p(pt.x(), pt.y(), pt.z());
        p.guid = pt.guid();
        p.name = pt.name();
        pts.push_back(p);
    }
    
    Polyline pl(pts);
    pl.guid = proto.guid();
    pl.name = proto.name();
    
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

void Polyline::protobuf_dump(const std::string& filename) const {
    std::string data = to_protobuf();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Polyline Polyline::protobuf_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return from_protobuf(data);
}
#endif

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

Point Polyline::point_at_parameter(const Point& start, const Point& end, double t) {
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
    overlap_start = point_at_parameter(line0_start, line0_end, t[1]);
    overlap_end = point_at_parameter(line0_start, line0_end, t[2]);

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
    output_start = point_at_parameter(line_start, line_end, t_values.front());
    output_end = point_at_parameter(line_start, line_end, t_values.back());

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

        Point point_on_segment = point_at_parameter(pi, pi1, t);
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

    closest_point = point_at_parameter(get_point(edge_id), get_point(edge_id + 1), best_t);
    return closest_distance;
}

bool Polyline::is_closed() const {
    if (point_count() < 2) return false;
    return get_point(0).distance(get_point(point_count() - 1)) < static_cast<double>(Tolerance::ZERO_TOLERANCE);
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

Vector Polyline::center_vec() const {
    Point c = center();
    return Vector(c[0], c[1], c[2]);
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

void Polyline::get_middle_line(const Point& line0_start, const Point& line0_end,
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

void Polyline::extend_line(Point& line_start, Point& line_end, double distance0, double distance1) {
    Vector v = line_end - line_start;
    v.normalize_self();

    line_start = line_start - v * static_cast<double>(distance0);
    line_end = line_end + v * static_cast<double>(distance1);
}

void Polyline::scale_line(Point& line_start, Point& line_end, double distance) {
    Vector v = line_end - line_start;
    Point p0 = line_start + v * static_cast<double>(distance);
    Point p1 = line_end - v * static_cast<double>(distance);
    line_start = p0;
    line_end = p1;
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

void Polyline::move(const Vector& direction) {
    for (size_t i = 0; i < point_count(); i++) {
        size_t idx = i * 3;
        _coords[idx] += direction[0];
        _coords[idx + 1] += direction[1];
        _coords[idx + 2] += direction[2];
    }
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

void Polyline::flip() {
    reverse();
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

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const Polyline& polyline) {
    return os << polyline.repr();
}

} // namespace session_cpp
