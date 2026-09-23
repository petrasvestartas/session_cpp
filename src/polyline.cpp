#include "polyline.h"
#include "boolean_polyline.h"
#include "intersection.h"
#include "polyline.pb.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// 2D helpers
// ═══════════════════════════════════════════════════════════════════════════
namespace {

/// Return the cross product sign of (b - a) x (p - a).
double ccw_2d(double ax, double ay, double bx, double by, double px, double py) { return (bx - ax) * (py - ay) - (by - ay) * (px - ax); }

/// Return the squared distance from (px, py) to the segment (a, b).
double seg_dist_sq(double px, double py, double ax, double ay, double bx, double by) {

    double x = ax;
    double y = ay;
    double dx = bx - x;
    double dy = by - y;

    if (dx != 0.0 || dy != 0.0) {
        const double t = ((px - x) * dx + (py - y) * dy) / (dx * dx + dy * dy);

        if (t > 1.0) {
            x = bx;
            y = by;
        } else if (t > 0.0) {
            x += dx * t;
            y += dy * t;
        }
    }

    dx = px - x;
    dy = py - y;

    return dx * dx + dy * dy;
}

/// Return the signed distance to the polygon rings, positive inside.
double point_to_polygon_dist(double px, double py, const std::vector<std::vector<std::array<double, 2>>>& polygon) {

    bool inside = false;
    double min_dist_sq = std::numeric_limits<double>::infinity();

    for (const std::vector<std::array<double, 2>>& ring : polygon) {
        const size_t len = ring.size();

        for (size_t i = 0, j = len - 1; i < len; j = i++) {
            const double ax = ring[i][0];
            const double ay = ring[i][1];
            const double bx = ring[j][0];
            const double by = ring[j][1];

            if ((ay > py) != (by > py) && (px < (bx - ax) * (py - ay) / (by - ay) + ax))
                inside = !inside;

            min_dist_sq = std::min(min_dist_sq, seg_dist_sq(px, py, ax, ay, bx, by));
        }
    }

    return (inside ? 1.0 : -1.0) * std::sqrt(min_dist_sq);
}

/// Quadtree cell of the polylabel search: center, half size, distance and its upper bound.
struct PCell {
    double cx; // Center x.
    double cy; // Center y.
    double h; // Half size.
    double d; // Signed distance from the center to the polygon.
    double mx; // Upper bound of the distance inside the cell.

    /// Construct the cell at (cx_, cy_) with half size h_ against polygon.
    PCell(double cx_, double cy_, double h_, const std::vector<std::vector<std::array<double, 2>>>& polygon)
        : cx(cx_), cy(cy_), h(h_), d(point_to_polygon_dist(cx_, cy_, polygon)), mx(d + h_ * std::sqrt(2.0)) {}

    /// Order by the upper bound so the priority queue pops the most promising cell.
    bool operator<(const PCell& o) const { return mx < o.mx; }
};

/// Return the cell at the centroid of the outer ring.
PCell centroid_cell(const std::vector<std::vector<std::array<double, 2>>>& polygon) {

    double area = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    const std::vector<std::array<double, 2>>& ring = polygon.at(0);
    const size_t len = ring.size();

    for (size_t i = 0, j = len - 1; i < len; j = i++) {
        const double ax = ring[i][0];
        const double ay = ring[i][1];
        const double bx = ring[j][0];
        const double by = ring[j][1];
        const double f = ax * by - bx * ay;
        cx += (ax + bx) * f;
        cy += (ay + by) * f;
        area += f * 3.0;
    }

    if (area == 0.0)
        return PCell(ring.at(0)[0], ring.at(0)[1], 0.0, polygon);

    return PCell(cx / area, cy / area, 0.0, polygon);
}

/// Return the center and radius of the largest inscribed circle in 2D (Mapbox polylabel).
std::array<double, 3> mapbox_polylabel(const std::vector<std::vector<std::array<double, 2>>>& polygon, double precision) {

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const std::array<double, 2>& p : polygon.at(0)) {
        min_x = std::min(min_x, p[0]);
        max_x = std::max(max_x, p[0]);
        min_y = std::min(min_y, p[1]);
        max_y = std::max(max_y, p[1]);
    }

    const double size_x = max_x - min_x;
    const double size_y = max_y - min_y;
    const double cell_size = std::min(size_x, size_y);
    const double h = cell_size / 2.0;

    if (cell_size == 0.0)
        return {min_x, min_y, 0.0};

    std::priority_queue<PCell> queue;

    for (double x = min_x; x < max_x; x += cell_size) {
        for (double y = min_y; y < max_y; y += cell_size)
            queue.emplace(x + h, y + h, h, polygon);
    }

    PCell best = centroid_cell(polygon);
    const PCell bbox_cell(min_x + size_x / 2.0, min_y + size_y / 2.0, 0.0, polygon);

    if (bbox_cell.d > best.d)
        best = bbox_cell;

    const size_t max_iter = 1000000;

    for (size_t iter = 0; iter < max_iter && !queue.empty(); iter++) {
        const PCell cell = queue.top();
        queue.pop();

        if (cell.d > best.d)
            best = cell;

        if (cell.mx - best.d <= precision)
            continue;

        const double nh = cell.h / 2.0;
        queue.emplace(cell.cx - nh, cell.cy - nh, nh, polygon);
        queue.emplace(cell.cx + nh, cell.cy - nh, nh, polygon);
        queue.emplace(cell.cx - nh, cell.cy + nh, nh, polygon);
        queue.emplace(cell.cx + nh, cell.cy + nh, nh, polygon);
    }

    return {best.cx, best.cy, best.d};
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Polyline::Polyline(const std::vector<Point>& points) {

    _coords.reserve(points.size() * 3);

    for (const Point& p : points) {
        _coords.push_back(p[0]);
        _coords.push_back(p[1]);
        _coords.push_back(p[2]);
    }

    recompute_plane_if_needed();
}

Polyline::Polyline(const Polyline& other)
    : _plane_dirty(other._plane_dirty), name(other.name), _coords(other._coords), plane(other.plane),
      width(other.width), dash(other.dash), linecolor(other.linecolor) {}

Polyline& Polyline::operator=(const Polyline& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    _plane_dirty = other._plane_dirty;
    name = other.name;
    _coords = other._coords;
    plane = other.plane;
    width = other.width;
    dash = other.dash;
    linecolor = other.linecolor;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
Polyline Polyline::from_coords(const std::vector<double>& coords) {

    Polyline polyline;
    polyline._coords = coords;
    polyline.recompute_plane_if_needed();

    return polyline;
}

Polyline Polyline::from_sides(int sides, double radius, bool close) {

    std::vector<Point> points;
    points.reserve(close ? sides + 1 : sides);

    for (int i = 0; i < sides; i++) {
        const double angle = 2.0 * Tolerance::PI * i / sides;
        points.emplace_back(radius * std::cos(angle), radius * std::sin(angle), 0.0);
    }

    if (close)
        points.push_back(points.front());

    return Polyline(points);
}

Polyline Polyline::rectangle(const Point& origin, const Vector& x_axis, const Vector& y_axis, double width, double height, bool close) {

    const Plane plane(origin, x_axis, y_axis);
    const Point o = plane.origin();
    const Vector x = plane.x_axis() * width;
    const Vector y = plane.y_axis() * height;
    std::vector<Point> points{o, o + x, o + x + y, o + y};

    if (close)
        points.push_back(points.front());

    return Polyline(points);
}

Polyline Polyline::quadratic_points(const Point& p0, const Point& p1, const Point& p2, int divisions) {

    const int n = std::max(divisions, 2);
    const double d = static_cast<double>(n - 1);
    std::vector<Point> points;
    points.reserve(n);

    for (int k = 0; k < n; k++) {
        const double t = k / d;
        const double s = 1.0 - t;
        const double s2 = s * s;
        const double ts = 2.0 * s * t;
        const double t2 = t * t;
        points.emplace_back(s2 * p0[0] + ts * p1[0] + t2 * p2[0], s2 * p0[1] + ts * p1[1] + t2 * p2[1], s2 * p0[2] + ts * p1[2] + t2 * p2[2]);
    }

    return Polyline(points);
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& Polyline::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& Polyline::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

void Polyline::refresh_guid() { _guid.clear(); }

size_t Polyline::point_count() const { return _coords.size() / 3; }

size_t Polyline::len() const { return point_count(); }

bool Polyline::is_empty() const { return _coords.empty(); }

size_t Polyline::segment_count() const {

    const size_t n = point_count();

    return n > 1 ? n - 1 : 0;
}

Point Polyline::get_point(size_t index) const {

    if (index >= point_count())
        return Point(0, 0, 0);

    const size_t idx = index * 3;

    return Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
}

std::vector<Point> Polyline::get_points() const {

    std::vector<Point> points;
    points.reserve(point_count());

    for (size_t i = 0; i < point_count(); i++)
        points.emplace_back(_coords[i * 3], _coords[i * 3 + 1], _coords[i * 3 + 2]);

    return points;
}

std::vector<Line> Polyline::get_lines() const {

    std::vector<Line> lines;
    lines.reserve(segment_count());

    for (size_t i = 0; i < segment_count(); i++) {
        const size_t idx0 = i * 3;
        const size_t idx1 = (i + 1) * 3;
        lines.emplace_back(_coords[idx0], _coords[idx0 + 1], _coords[idx0 + 2], _coords[idx1], _coords[idx1 + 1], _coords[idx1 + 2]);
    }

    return lines;
}

const Plane& Polyline::get_plane() const {

    if (!_plane_dirty || point_count() < 3)
        return plane;

    const size_t n = point_count();
    const Point p0 = get_point(0);
    bool found = false;

    for (size_t i = 1; i < n && !found; i++) {
        Vector v1 = get_point(i) - p0;

        if (v1.magnitude_squared() < 1e-20)
            continue;

        for (size_t j = i + 1; j < n && !found; j++) {
            Vector normal = v1.cross(get_point(j) - p0);

            if (normal.magnitude_squared() < 1e-20)
                continue;

            normal.normalize_self();
            v1.normalize_self();
            Vector yax = normal.cross(v1);
            yax.normalize_self();
            plane = Plane::from_frame(p0, v1, yax, normal);
            found = true;
        }
    }

    if (!found)
        plane = Plane();

    _plane_dirty = false;

    return plane;
}

double Polyline::length() const {

    double total = 0.0;

    for (size_t i = 0; i < segment_count(); i++)
        total += std::sqrt((get_point(i + 1) - get_point(i)).magnitude_squared());

    return total;
}

double Polyline::length_squared() const {

    double total = 0.0;

    for (size_t i = 0; i < segment_count(); i++)
        total += (get_point(i + 1) - get_point(i)).magnitude_squared();

    return total;
}

bool Polyline::is_closed() const {

    if (point_count() < 2)
        return false;

    return get_point(0).distance(get_point(point_count() - 1)) < Tolerance::ZERO_TOLERANCE;
}

Polyline Polyline::closed() const {

    if (is_closed())
        return Polyline::from_coords(_coords);

    std::vector<double> coords(_coords);
    coords.push_back(_coords[0]);
    coords.push_back(_coords[1]);
    coords.push_back(_coords[2]);

    return Polyline::from_coords(coords);
}

Point Polyline::center() const {

    if (_coords.empty())
        return Point(0, 0, 0);

    const size_t n = is_closed() ? point_count() - 1 : point_count();
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    for (size_t i = 0; i < n; i++) {
        x += _coords[i * 3];
        y += _coords[i * 3 + 1];
        z += _coords[i * 3 + 2];
    }

    return Point(x / n, y / n, z / n);
}

void Polyline::get_average_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) const {

    origin = center();
    x_axis = point_count() >= 2 ? get_point(1) - get_point(0) : Vector(1, 0, 0);
    x_axis.normalize_self();
    average_normal(z_axis);
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
    pln = Plane::from_point_normal(origin, normal);
}

void Polyline::get_convex_corners(std::vector<bool>& convex_or_concave) const {

    if (point_count() < 3)
        return;

    const size_t n = is_closed() ? point_count() - 1 : point_count();
    Vector normal;
    average_normal(normal);
    convex_or_concave.clear();
    convex_or_concave.reserve(n);

    for (size_t current = 0; current < n; current++) {
        const size_t prev = current == 0 ? n - 1 : current - 1;
        const size_t next = current == n - 1 ? 0 : current + 1;
        Vector dir0 = get_point(current) - get_point(prev);
        dir0.normalize_self();
        Vector dir1 = get_point(next) - get_point(current);
        dir1.normalize_self();
        Vector cross = dir0.cross(dir1);
        cross.normalize_self();
        convex_or_concave.push_back(cross.dot(normal) >= 0.0);
    }
}

bool Polyline::is_clockwise(const Plane& pln) const {

    const size_t n = point_count();

    if (n < 3)
        return false;

    const Vector& xv = pln.x_axis();
    const Vector& yv = pln.y_axis();
    const Point& orig = pln.origin();
    const size_t lim = is_closed() ? n - 1 : n;
    double area = 0.0;

    for (size_t i = 0; i < lim; i++) {
        const Vector d0 = get_point(i) - orig;
        const Vector d1 = get_point((i + 1) % lim) - orig;
        const double u0 = d0.dot(xv);
        const double v0 = d0.dot(yv);
        const double u1 = d1.dot(xv);
        const double v1 = d1.dot(yv);
        area += (u1 - u0) * (v1 + v0);
    }

    return area > 0;
}

bool Polyline::point_in_polygon_2d(const Point& p) const {

    const double px = p[0];
    const double py = p[1];
    const size_t n = point_count();
    int winding = 0;

    for (size_t i = 0; i < n; i++) {
        const size_t j = (i + 1) % n;
        const double x0 = _coords[i * 3];
        const double y0 = _coords[i * 3 + 1];
        const double x1 = _coords[j * 3];
        const double y1 = _coords[j * 3 + 1];
        const double side = (x1 - x0) * (py - y0) - (px - x0) * (y1 - y0);

        if (y0 <= py && y1 > py && side > 0.0)
            winding++;
        else if (y0 > py && y1 <= py && side < 0.0)
            winding--;
    }

    return winding != 0;
}

double Polyline::closest_distance_and_point(const Point& point, size_t& edge_id, Point& closest_point) const {

    edge_id = 0;
    double closest_distance = std::numeric_limits<double>::max();
    double best_t = 0.0;

    for (size_t i = 0; i < segment_count(); i++) {
        double t = 0.0;
        closest_point_to_line(point, get_point(i), get_point(i + 1), t);
        const double distance = point.distance(point_at(get_point(i), get_point(i + 1), t));

        if (distance < closest_distance) {
            closest_distance = distance;
            edge_id = i;
            best_t = t;
        }

        if (closest_distance < Tolerance::ZERO_TOLERANCE)
            break;
    }

    closest_point = point_at(get_point(edge_id), get_point(edge_id + 1), best_t);

    return closest_distance;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mutators
// ═══════════════════════════════════════════════════════════════════════════
void Polyline::set_point(size_t index, const Point& point) {

    if (index >= point_count())
        return;

    const size_t idx = index * 3;
    _coords[idx] = point[0];
    _coords[idx + 1] = point[1];
    _coords[idx + 2] = point[2];
}

void Polyline::add_point(const Point& point) {

    _coords.push_back(point[0]);
    _coords.push_back(point[1]);
    _coords.push_back(point[2]);

    if (point_count() == 3)
        recompute_plane_if_needed();
}

void Polyline::insert_point(size_t index, const Point& point) {

    if (index > point_count())
        return;

    const size_t idx = index * 3;
    _coords.insert(_coords.begin() + idx, {point[0], point[1], point[2]});

    if (point_count() == 3)
        recompute_plane_if_needed();
}

bool Polyline::remove_point(size_t index, Point& out_point) {

    if (index >= point_count())
        return false;

    const size_t idx = index * 3;
    out_point = Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
    _coords.erase(_coords.begin() + idx, _coords.begin() + idx + 3);

    if (point_count() == 3)
        recompute_plane_if_needed();

    return true;
}

void Polyline::reverse() {

    const size_t n = point_count();
    std::vector<double> coords;
    coords.reserve(_coords.size());

    for (size_t i = n; i > 0; i--) {
        const size_t idx = (i - 1) * 3;
        coords.push_back(_coords[idx]);
        coords.push_back(_coords[idx + 1]);
        coords.push_back(_coords[idx + 2]);
    }

    _coords = std::move(coords);
    plane.reverse();
}

Polyline Polyline::reversed() const {

    Polyline result = *this;
    result.reverse();

    return result;
}

void Polyline::shift(int times) {

    if (_coords.empty())
        return;

    const bool was_closed = is_closed();

    if (was_closed)
        _coords.resize(_coords.size() - 3);

    const size_t n = point_count();

    if (n > 0 && times != 0) {
        int offset = times % static_cast<int>(n);

        if (offset < 0)
            offset += static_cast<int>(n);

        std::vector<double> coords;
        coords.reserve(_coords.size());

        for (size_t i = 0; i < n; i++) {
            const size_t src = ((i + offset) % n) * 3;
            coords.push_back(_coords[src]);
            coords.push_back(_coords[src + 1]);
            coords.push_back(_coords[src + 2]);
        }

        _coords = std::move(coords);
    }

    if (was_closed && n > 0) {
        _coords.push_back(_coords[0]);
        _coords.push_back(_coords[1]);
        _coords.push_back(_coords[2]);
    }
}

void Polyline::translate(const Vector& v) { *this += v; }

Polyline Polyline::translated(const Vector& v) const {

    Polyline result = *this;
    result.translate(v);

    return result;
}

void Polyline::extend_segment(int segment_id, double dist0, double dist1, double proportion0, double proportion1) {

    if (segment_id < 0 || segment_id >= static_cast<int>(segment_count()))
        return;

    if (dist0 == 0 && dist1 == 0 && proportion0 == 0 && proportion1 == 0)
        return;

    const bool was_closed = is_closed();
    Point p0 = get_point(segment_id);
    Point p1 = get_point(segment_id + 1);
    Vector v = p1 - p0;

    if (proportion0 != 0 || proportion1 != 0) {
        p0 = p0 - v * proportion0;
        p1 = p1 + v * proportion1;
    } else {
        v.normalize_self();
        p0 = p0 - v * dist0;
        p1 = p1 + v * dist1;
    }

    set_point(segment_id, p0);
    set_point(segment_id + 1, p1);

    if (!was_closed)
        return;

    if (segment_id == 0)
        set_point(point_count() - 1, get_point(0));
    else if (segment_id + 1 == static_cast<int>(point_count() - 1))
        set_point(0, get_point(point_count() - 1));
}

void Polyline::extend_segment_equally(int segment_id, double dist, double proportion) {

    if (segment_id < 0 || segment_id >= static_cast<int>(segment_count()))
        return;

    Point p0 = get_point(segment_id);
    Point p1 = get_point(segment_id + 1);
    extend_segment_equally(p0, p1, dist, proportion);
    set_point(segment_id, p0);
    set_point(segment_id + 1, p1);

    if (point_count() <= 2 || !is_closed())
        return;

    if (segment_id == 0)
        set_point(point_count() - 1, get_point(0));
    else if (segment_id + 1 == static_cast<int>(point_count() - 1))
        set_point(0, get_point(point_count() - 1));
}

void Polyline::extend_edge_equally(size_t edge_idx, double distance) {

    const size_t n = point_count();

    if (n < 2 || edge_idx + 1 >= n)
        return;

    const size_t i = edge_idx;
    const size_t j = edge_idx + 1;
    const Point pi = get_point(i);
    const Point pj = get_point(j);
    Vector dir = pj - pi;
    const double len = std::sqrt(dir.magnitude_squared());

    if (len < 1e-12)
        return;

    dir = dir * (distance / len);
    const Point new_pi = pi - dir;
    const Point new_pj = pj + dir;
    set_point(i, new_pi);
    set_point(j, new_pj);

    if (i == 0)
        set_point(n - 1, new_pi);

    if (j == n - 1)
        set_point(0, new_pj);
}

void Polyline::merge_collinear(double tol) {

    const bool closed = is_closed();
    std::vector<Point> points = get_points();

    if (closed && points.size() > 1)
        points.pop_back();

    const double zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
    const size_t max_pass = points.size();
    bool changed = true;

    for (size_t pass = 0; pass < max_pass && changed && points.size() >= 3; pass++) {
        changed = false;
        const size_t m = points.size();
        std::vector<Point> out;

        for (size_t i = 0; i < m; i++) {
            const size_t p = (i + m - 1) % m;
            const size_t nx = (i + 1) % m;

            if (!closed && (i == 0 || i == m - 1)) {
                out.push_back(points[i]);
                continue;
            }

            const Vector a = points[i] - points[p];
            const Vector b = points[nx] - points[i];
            const double a2 = a.magnitude_squared();
            const double b2 = b.magnitude_squared();

            if (a2 < zt2 || b2 < zt2 || a.cross(b).magnitude_squared() < tol * tol * a2 * b2)
                changed = true;
            else
                out.push_back(points[i]);
        }

        points = out;
    }

    if (closed && !points.empty())
        points.push_back(points[0]);

    _coords = Polyline(points)._coords;
    recompute_plane_if_needed();
}

void Polyline::remove_consecutive_duplicates(double tol) {

    const double tol_sq = tol * tol;
    std::vector<Point> cleaned;
    cleaned.reserve(point_count());

    for (const Point& p : get_points()) {
        if (cleaned.empty() || (p - cleaned.back()).magnitude_squared() >= tol_sq)
            cleaned.push_back(p);
    }

    _coords = Polyline(cleaned)._coords;
    recompute_plane_if_needed();
}

Polyline Polyline::simplify(double tolerance) const { return Polyline(simplify_points(get_points(), tolerance)); }

Polyline Polyline::cut_by_plane(const Plane& plane, std::optional<bool> flip) const {

    const size_t n = point_count();

    if (n < 2)
        return *this;

    const Vector& normal = plane.z_axis();
    const Point& origin = plane.origin();
    double keep_sign = 1.0;

    if (flip.has_value())
        keep_sign = flip.value() ? 1.0 : -1.0;
    else
        keep_sign = normal.dot(point_at_length(length() * 0.5) - origin) >= 0.0 ? 1.0 : -1.0;

    std::vector<Point> result;

    for (size_t i = 0; i + 1 < n; i++) {
        const Point a = get_point(i);
        const Point b = get_point(i + 1);
        const double da = normal.dot(a - origin);
        const double db = normal.dot(b - origin);

        if (da * keep_sign >= 0.0)
            result.push_back(a);

        if ((da > 0.0) != (db > 0.0))
            result.push_back(a + (b - a) * (da / (da - db)));
    }

    const Point last = get_point(n - 1);

    if (normal.dot(last - origin) * keep_sign >= 0.0)
        result.push_back(last);

    Polyline cut(result);
    cut.remove_consecutive_duplicates(1e-6);

    return cut;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool Polyline::operator==(const Polyline& other) const {

    if (name != other.name)
        return false;

    if (point_count() != other.point_count())
        return false;

    for (size_t i = 0; i < _coords.size(); i++) {
        if (std::round(_coords[i] * 1000000.0) != std::round(other._coords[i] * 1000000.0))
            return false;
    }

    if (std::round(width * 1000000.0) != std::round(other.width * 1000000.0))
        return false;

    return linecolor == other.linecolor;
}

bool Polyline::operator!=(const Polyline& other) const { return !(*this == other); }

Point Polyline::operator[](size_t index) const {

    if (index >= point_count())
        throw std::out_of_range("Index out of range");

    return get_point(index);
}

Polyline& Polyline::operator+=(const Vector& v) {

    for (size_t i = 0; i < point_count(); i++) {
        _coords[i * 3] += v[0];
        _coords[i * 3 + 1] += v[1];
        _coords[i * 3 + 2] += v[2];
    }

    plane = Plane(plane.origin() + v, plane.x_axis(), plane.y_axis());

    return *this;
}

Polyline& Polyline::operator-=(const Vector& v) {

    for (size_t i = 0; i < point_count(); i++) {
        _coords[i * 3] -= v[0];
        _coords[i * 3 + 1] -= v[1];
        _coords[i * 3 + 2] -= v[2];
    }

    plane = Plane(plane.origin() - v, plane.x_axis(), plane.y_axis());

    return *this;
}

Polyline& Polyline::operator*=(double factor) {

    for (size_t i = 0; i < _coords.size(); i++)
        _coords[i] *= factor;

    return *this;
}

Polyline& Polyline::operator/=(double factor) {

    for (size_t i = 0; i < _coords.size(); i++)
        _coords[i] /= factor;

    return *this;
}

Polyline Polyline::operator+(const Vector& v) const {

    Polyline result = *this;
    result += v;

    return result;
}

Polyline Polyline::operator-(const Vector& v) const {

    Polyline result = *this;
    result -= v;

    return result;
}

Polyline Polyline::operator*(double factor) const {

    Polyline result = *this;
    result *= factor;

    return result;
}

Polyline Polyline::operator/(double factor) const {

    Polyline result = *this;
    result /= factor;

    return result;
}

Polyline Polyline::operator-() const { return reversed(); }

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void Polyline::transform(const Xform& xform) {

    for (size_t i = 0; i < point_count(); i++) {
        Point point = get_point(i);
        point.transform(xform);
        set_point(i, point);
    }
}

Polyline Polyline::transformed(const Xform& xform) const {

    Polyline result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Segment utilities
// ═══════════════════════════════════════════════════════════════════════════
Point Polyline::point_at(const Point& start, const Point& end, double t) {

    const double s = 1.0 - t;

    return Point(
        start[0] == end[0] ? start[0] : s * start[0] + t * end[0],
        start[1] == end[1] ? start[1] : s * start[1] + t * end[1],
        start[2] == end[2] ? start[2] : s * start[2] + t * end[2]
    );
}

void Polyline::closest_point_to_line(const Point& point, const Point& line_start, const Point& line_end, double& t) {

    const Vector d = line_end - line_start;
    const double dod = d.magnitude_squared();

    if (dod <= 0.0) {
        t = 0.0;

        return;
    }

    const Vector to_start = point - line_start;
    const Vector to_end = point - line_end;

    if (to_start.magnitude_squared() <= to_end.magnitude_squared())
        t = to_start.dot(d) / dod;
    else
        t = 1.0 + to_end.dot(d) / dod;
}

bool Polyline::line_line_overlap(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& overlap_start, Point& overlap_end) {

    double t[4] = {0.0, 1.0, 0.0, 0.0};
    closest_point_to_line(line1_start, line0_start, line0_end, t[2]);
    closest_point_to_line(line1_end, line0_start, line0_end, t[3]);
    bool do_overlap = !((t[2] < 0 && t[3] < 0) || (t[2] > 1 && t[3] > 1));
    std::sort(t, t + 4);
    do_overlap = do_overlap && std::abs(t[2] - t[1]) > Tolerance::ZERO_TOLERANCE;
    overlap_start = point_at(line0_start, line0_end, t[1]);
    overlap_end = point_at(line0_start, line0_end, t[2]);

    return do_overlap;
}

void Polyline::line_line_average(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& output_start, Point& output_end) {

    output_start = Point::mid_point(line0_start, line1_start);
    output_end = Point::mid_point(line0_end, line1_end);
}

void Polyline::line_line_overlap_average(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& output_start, Point& output_end) {

    Point line_a_start;
    Point line_a_end;
    line_line_overlap(line0_start, line0_end, line1_start, line1_end, line_a_start, line_a_end);

    Point line_b_start;
    Point line_b_end;
    line_line_overlap(line1_start, line1_end, line0_start, line0_end, line_b_start, line_b_end);

    Point mid_line0_start;
    Point mid_line0_end;
    line_line_average(line_a_start, line_a_end, line_b_start, line_b_end, mid_line0_start, mid_line0_end);

    Point mid_line1_start;
    Point mid_line1_end;
    line_line_average(line_a_start, line_a_end, line_b_end, line_b_start, mid_line1_start, mid_line1_end);

    if ((mid_line0_end - mid_line0_start).magnitude_squared() > (mid_line1_end - mid_line1_start).magnitude_squared()) {
        output_start = mid_line0_start;
        output_end = mid_line0_end;
    } else {
        output_start = mid_line1_start;
        output_end = mid_line1_end;
    }
}

bool Polyline::line_from_projected_points(const Point& line_start, const Point& line_end, const std::vector<Point>& points, Point& output_start, Point& output_end) {

    if (points.empty())
        return false;

    std::vector<double> t_values;
    t_values.reserve(points.size());

    for (const Point& point : points) {
        double t = 0.0;
        closest_point_to_line(point, line_start, line_end, t);
        t_values.push_back(t);
    }

    std::sort(t_values.begin(), t_values.end());
    output_start = point_at(line_start, line_end, t_values.front());
    output_end = point_at(line_start, line_end, t_values.back());

    return std::abs(t_values.front() - t_values.back()) > Tolerance::ZERO_TOLERANCE;
}

void Polyline::extend_segment_equally(Point& segment_start, Point& segment_end, double dist, double proportion) {

    if (dist == 0 && proportion == 0)
        return;

    Vector v = segment_end - segment_start;

    if (proportion != 0) {
        segment_start = segment_start - v * proportion;
        segment_end = segment_end + v * proportion;
    } else {
        v.normalize_self();
        segment_start = segment_start - v * dist;
        segment_end = segment_end + v * dist;
    }
}

void Polyline::extend_line_segment(Point& start, Point& end, double d0, double d1) {

    Vector v = end - start;
    v.normalize_self();
    start = start - v * d0;
    end = end + v * d1;
}

void Polyline::shrink_line_segment(Point& start, Point& end, double dist) {

    const Vector v = end - start;
    start = start + v * dist;
    end = end - v * dist;
}

// ═══════════════════════════════════════════════════════════════════════════
// Polygon utilities
// ═══════════════════════════════════════════════════════════════════════════
Polyline Polyline::tween_two_polylines(const Polyline& polyline0, const Polyline& polyline1, double weight) {

    if (polyline0.point_count() != polyline1.point_count())
        return polyline0;

    Polyline result;
    result._coords.reserve(polyline0._coords.size());

    for (size_t i = 0; i < polyline0.point_count(); i++) {
        const Point p0 = polyline0.get_point(i);
        const Point p1 = polyline1.get_point(i);
        result.add_point(p0 + (p1 - p0) * weight);
    }

    return result;
}

std::vector<Point> Polyline::interpolate_points(const Point& from, const Point& to, int steps, int kind) { return Point::interpolate(from, to, steps, kind); }

Polyline Polyline::quick_hull(const Polyline& polygon) {

    Point origin;
    Vector xa;
    Vector ya;
    Vector za;
    polygon.get_average_plane(origin, xa, ya, za);

    std::vector<std::array<double, 2>> pts2d;
    polygon.project_to_plane(origin, xa, ya, pts2d);
    size_t ai = 0;
    size_t bi = 0;

    for (size_t i = 1; i < pts2d.size(); i++) {
        if (pts2d[i][0] < pts2d[ai][0])
            ai = i;

        if (pts2d[i][0] >= pts2d[bi][0])
            bi = i;
    }

    const double ax = pts2d[ai][0];
    const double ay = pts2d[ai][1];
    const double bx = pts2d[bi][0];
    const double by = pts2d[bi][1];
    std::vector<std::array<double, 2>> left;
    std::vector<std::array<double, 2>> right;

    for (const std::array<double, 2>& p : pts2d) {
        if (ccw_2d(ax, ay, bx, by, p[0], p[1]) > 0.0)
            left.push_back(p);
        else
            right.push_back(p);
    }

    std::vector<std::array<double, 2>> hull;
    hull.push_back({ax, ay});
    quick_hull_recurse(left, ax, ay, bx, by, hull);
    hull.push_back({bx, by});
    quick_hull_recurse(right, bx, by, ax, ay, hull);

    std::vector<Point> pts3d;
    pts3d.reserve(hull.size());

    for (const std::array<double, 2>& h : hull)
        pts3d.push_back(unproject(origin, xa, ya, h[0], h[1]));

    return Polyline(pts3d);
}

std::optional<Polyline> Polyline::bounding_rectangle(const Polyline& polygon) {

    const Polyline hull = quick_hull(polygon);

    if (hull.point_count() <= 2)
        return std::nullopt;

    Point origin;
    Vector xa;
    Vector ya;
    Vector za;
    polygon.get_average_plane(origin, xa, ya, za);

    std::vector<std::array<double, 2>> hull2d;
    hull.project_to_plane(origin, xa, ya, hull2d);
    double best_area = std::numeric_limits<double>::max();
    double best_min_u = 0.0;
    double best_max_u = 0.0;
    double best_min_v = 0.0;
    double best_max_v = 0.0;
    double best_angle = 0.0;
    const size_t hn = hull2d.size();

    for (size_t i = 0; i < hn; i++) {
        const size_t j = (i + 1) % hn;
        const double ex = hull2d[j][0] - hull2d[i][0];
        const double ey = hull2d[j][1] - hull2d[i][1];
        const double len = std::sqrt(ex * ex + ey * ey);

        if (len < 1e-12)
            continue;

        const double ca = ex / len;
        const double sa = ey / len;
        double min_u = std::numeric_limits<double>::max();
        double max_u = -std::numeric_limits<double>::max();
        double min_v = std::numeric_limits<double>::max();
        double max_v = -std::numeric_limits<double>::max();

        for (const std::array<double, 2>& h : hull2d) {
            const double u = h[0] * ca + h[1] * sa;
            const double v = -h[0] * sa + h[1] * ca;
            min_u = std::min(min_u, u);
            max_u = std::max(max_u, u);
            min_v = std::min(min_v, v);
            max_v = std::max(max_v, v);
        }

        const double area = (max_u - min_u) * (max_v - min_v);

        if (area < best_area) {
            best_area = area;
            best_min_u = min_u;
            best_max_u = max_u;
            best_min_v = min_v;
            best_max_v = max_v;
            best_angle = std::atan2(ey, ex);
        }
    }

    const double ca = std::cos(best_angle);
    const double sa = std::sin(best_angle);
    const std::array<std::array<double, 2>, 4> uv = {{{best_min_u, best_min_v}, {best_min_u, best_max_v}, {best_max_u, best_max_v}, {best_max_u, best_min_v}}};
    std::vector<Point> pts3d;
    pts3d.reserve(5);

    for (const std::array<double, 2>& c : uv)
        pts3d.push_back(unproject(origin, xa, ya, c[0] * ca - c[1] * sa, c[0] * sa + c[1] * ca));

    pts3d.push_back(pts3d[0]);

    return Polyline(pts3d);
}

std::vector<Point> Polyline::grid_of_points_in_polygon(const Polyline& polygon, double offset_dist, double div_dist, size_t max_pts) {

    if (div_dist < 1e-12)
        return {};

    Point origin;
    Vector xa;
    Vector ya;
    Vector za;
    polygon.get_average_plane(origin, xa, ya, za);

    std::vector<std::array<double, 2>> poly2d;
    polygon.project_to_plane(origin, xa, ya, poly2d);

    if (poly2d.size() > 1 && polygon.get_point(0).distance(polygon.get_point(polygon.point_count() - 1)) < 1e-10)
        poly2d.pop_back();

    if (poly2d.empty())
        return {};

    offset_polygon_2d(poly2d, offset_dist);
    double x_min = std::numeric_limits<double>::max();
    double x_max = -std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double y_max = -std::numeric_limits<double>::max();

    for (const std::array<double, 2>& p : poly2d) {
        x_min = std::min(x_min, p[0]);
        x_max = std::max(x_max, p[0]);
        y_min = std::min(y_min, p[1]);
        y_max = std::max(y_max, p[1]);
    }

    std::vector<Point> result;

    for (double u = x_min; u <= x_max + 1e-10 && result.size() < max_pts; u += div_dist) {
        for (double v = y_min; v <= y_max + 1e-10 && result.size() < max_pts; v += div_dist) {
            if (point_in_polygon(poly2d, u, v))
                result.push_back(unproject(origin, xa, ya, u, v));
        }
    }

    return result;
}

std::tuple<Point, Plane, double> Polyline::polylabel(const std::vector<Polyline>& polylines, double precision) {

    if (polylines.empty())
        return {Point(0, 0, 0), Plane(), 0.0};

    Point origin;
    Vector xa;
    Vector ya;
    Vector za;
    polylines[0].get_average_plane(origin, xa, ya, za);

    std::vector<std::vector<std::array<double, 2>>> rings2d(polylines.size());
    std::vector<double> sizes(polylines.size(), 0.0);

    for (size_t i = 0; i < polylines.size(); i++) {
        const Polyline& pl = polylines[i];
        pl.project_to_plane(origin, xa, ya, rings2d[i]);

        if (rings2d[i].size() > 1 && pl.get_point(0).distance(pl.get_point(pl.point_count() - 1)) < 1e-10)
            rings2d[i].pop_back();

        double mnx = std::numeric_limits<double>::infinity();
        double mny = std::numeric_limits<double>::infinity();
        double mxx = -std::numeric_limits<double>::infinity();
        double mxy = -std::numeric_limits<double>::infinity();

        for (const std::array<double, 2>& uv : rings2d[i]) {
            mnx = std::min(mnx, uv[0]);
            mxx = std::max(mxx, uv[0]);
            mny = std::min(mny, uv[1]);
            mxy = std::max(mxy, uv[1]);
        }

        sizes[i] = (mxx - mnx) * (mxx - mnx) + (mxy - mny) * (mxy - mny);
    }

    std::vector<std::pair<double, size_t>> order;
    order.reserve(rings2d.size());

    for (size_t i = 0; i < rings2d.size(); i++)
        order.emplace_back(-sizes[i], i);

    std::sort(order.begin(), order.end());
    std::vector<std::vector<std::array<double, 2>>> polygon;
    polygon.reserve(rings2d.size());

    for (const std::pair<double, size_t>& item : order)
        polygon.push_back(std::move(rings2d[item.second]));

    const std::array<double, 3> cr = mapbox_polylabel(polygon, precision);
    const Point center = unproject(origin, xa, ya, cr[0], cr[1]);

    return std::make_tuple(center, Plane::from_frame(origin, xa, ya, za), cr[2]);
}

std::vector<Point> Polyline::polylabel_circle_division_points(const Vector& division_direction_in_3d, const std::vector<Polyline>& polylines, int division, double scale, double precision, bool orient_to_closest_edge) {

    const std::tuple<Point, Plane, double> circle = polylabel(polylines, precision);
    const Point& center = std::get<0>(circle);
    const Plane& plane = std::get<1>(circle);
    const double radius = std::get<2>(circle) * scale;
    const bool is_direction_valid = division_direction_in_3d[0] != 0.0 || division_direction_in_3d[1] != 0.0 || division_direction_in_3d[2] != 0.0;
    size_t edge_i = 0;
    size_t edge_j = 0;
    const bool found = orient_to_closest_edge && closest_edge(center, polylines, edge_i, edge_j);
    Vector x_axis = plane.x_axis();
    Vector y_axis = plane.y_axis();
    Vector z_axis = plane.z_axis();

    if (is_direction_valid || orient_to_closest_edge) {
        const Vector dir = found ? polylines[edge_i].get_point(edge_j + 1) - polylines[edge_i].get_point(edge_j) : division_direction_in_3d;
        x_axis = dir;
        y_axis = dir.cross(z_axis);
    }

    x_axis.normalize_self();
    y_axis.normalize_self();
    z_axis.normalize_self();

    std::vector<Point> points;
    points.reserve(division);
    const double chunk = 360.0 / division;

    for (int i = 0; i < division; i++) {
        const double rad = (45.0 + i * chunk) * Tolerance::PI / 180.0;
        points.push_back(unproject(center, x_axis, y_axis, radius * std::cos(rad), radius * std::sin(rad)));
    }

    return points;
}

std::vector<Polyline> Polyline::boolean_op(const Polyline& a, const Polyline& b, int clip_type) { return BooleanPolyline::compute(a, b, clip_type); }

std::vector<Polyline> Polyline::boolean_op(const Polyline& a, const Polyline& b, const Plane& plane, int clip_type) {

    Polyline pa2d = boolean_project(a, plane);
    Polyline pb2d = boolean_project(b, plane);
    ensure_ccw(pa2d);
    ensure_ccw(pb2d);

    std::vector<Polyline> results = BooleanPolyline::compute(pa2d, pb2d, clip_type);
    const Point& o = plane.origin();
    const Vector& x = plane.x_axis();
    const Vector& y = plane.y_axis();

    for (Polyline& r : results) {
        for (size_t i = 0; i < r.point_count(); i++)
            r.set_point(i, unproject(o, x, y, r._coords[i * 3], r._coords[i * 3 + 1]));
    }

    return results;
}

std::vector<Point> Polyline::simplify_points(const std::vector<Point>& points, double tolerance) {

    const int n = static_cast<int>(points.size());

    if (n < 3)
        return points;

    std::vector<bool> keep(n, false);
    keep[0] = true;
    keep[n - 1] = true;
    simplify_rdp(points, 0, n - 1, tolerance, keep);

    std::vector<Point> result;

    for (int i = 0; i < n; i++) {
        if (keep[i])
            result.push_back(points[i]);
    }

    return result;
}

void Polyline::two_rects_from_frame(const Point& p, const Vector& segment_vector, const Vector& zaxis, bool middle, double radius, double length, int flip_male, Polyline& rect0, Polyline& rect1) {

    Vector y_axis = zaxis.cross(segment_vector);
    Vector x_axis = y_axis.cross(segment_vector);
    x_axis.normalize_self();
    y_axis.normalize_self();
    x_axis = x_axis * radius;
    y_axis = y_axis * radius;
    const Vector sv0 = segment_vector * (length * -0.5);
    const Vector sv1 = segment_vector * (length * 0.5);
    std::array<Vector, 4> v = {-x_axis - y_axis, x_axis - y_axis, x_axis + y_axis, -x_axis + y_axis};

    if (!middle && flip_male == 1)
        std::rotate(v.begin(), v.begin() + 1, v.end());
    else if (!middle && flip_male == -1)
        std::rotate(v.rbegin(), v.rbegin() + 1, v.rend());

    rect0 = Polyline({p + sv0 + v[1], p + sv1 + v[1], p + sv1 + v[0], p + sv0 + v[0], p + sv0 + v[1]});
    rect1 = Polyline({p + sv0 + v[2], p + sv1 + v[2], p + sv1 + v[3], p + sv0 + v[3], p + sv0 + v[2]});
}

bool Polyline::trim_rectangles_by_plane(Polyline& first, Polyline& second, const Plane& plane) {

    if (first.point_count() != 5 || second.point_count() != 5)
        return false;

    std::array<Point, 4> points;
    const bool hit0 = Intersection::line_plane(Line::from_points(first[0], first[1]), plane, points[0], false);
    const bool hit1 = Intersection::line_plane(Line::from_points(first[3], first[2]), plane, points[1], false);
    const bool hit2 = Intersection::line_plane(Line::from_points(second[0], second[1]), plane, points[2], false);
    const bool hit3 = Intersection::line_plane(Line::from_points(second[3], second[2]), plane, points[3], false);

    if (!hit0 || !hit1 || !hit2 || !hit3)
        return false;

    for (const Point& point : points) {
        for (size_t i = 0; i < 3; i++) {
            if (!std::isfinite(point[i]))
                return false;
        }
    }

    if (plane.has_on_negative_side(first[0])) {
        first.set_point(0, points[0]);
        first.set_point(3, points[1]);
        first.set_point(4, points[0]);
        second.set_point(0, points[2]);
        second.set_point(3, points[3]);
        second.set_point(4, points[2]);
    } else {
        first.set_point(1, points[0]);
        first.set_point(2, points[1]);
        second.set_point(1, points[2]);
        second.set_point(2, points[3]);
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Polyline::jsondump() const {

    nlohmann::ordered_json data;
    data["coords"] = _coords;
    data["dash"] = dash;
    data["guid"] = guid();
    data["linecolor"] = linecolor.jsondump();
    data["name"] = name;
    data["type"] = "Polyline";
    data["width"] = width;

    return data;
}

Polyline Polyline::jsonload(const nlohmann::json& data) {

    Polyline polyline;
    polyline.guid() = data["guid"];
    polyline.name = data["name"];

    if (data.contains("coords")) {
        polyline._coords = data["coords"].get<std::vector<double>>();
    } else if (data.contains("points")) {
        for (const nlohmann::json& point : data["points"])
            polyline.add_point(Point::jsonload(point));
    }

    if (data.contains("width"))
        polyline.width = data["width"];

    if (data.contains("dash"))
        polyline.dash = data["dash"].get<std::vector<double>>();

    if (data.contains("linecolor"))
        polyline.linecolor = Color::jsonload(data["linecolor"]);

    polyline.recompute_plane_if_needed();

    return polyline;
}

std::string Polyline::file_json_dumps() const { return jsondump().dump(); }

Polyline Polyline::file_json_loads(const std::string& json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void Polyline::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(2);
}

Polyline Polyline::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Polyline Polyline::to_proto() const {

    session_proto::Polyline proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_width(width);

    for (double d : dash)
        proto.add_dash(d);

    for (double c : _coords)
        proto.add_coords(c);

    *proto.mutable_linecolor() = linecolor.to_proto();

    return proto;
}

Polyline Polyline::from_proto(const session_proto::Polyline& proto) {

    Polyline polyline = Polyline::from_coords(std::vector<double>(proto.coords().begin(), proto.coords().end()));

    if (!proto.guid().empty())
        polyline.guid() = proto.guid();

    polyline.name = proto.name();
    polyline.width = proto.width();
    polyline.dash.assign(proto.dash().begin(), proto.dash().end());

    if (proto.has_linecolor())
        polyline.linecolor = Color::from_proto(proto.linecolor());

    return polyline;
}

std::string Polyline::pb_dumps() const { return to_proto().SerializeAsString(); }

Polyline Polyline::pb_loads(const std::string& data) {

    session_proto::Polyline proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Polyline protobuf data");

    return from_proto(proto);
}

void Polyline::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Polyline Polyline::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Polyline::str() const {

    std::ostringstream oss;
    oss << "[";

    for (size_t i = 0; i < point_count(); i++) {
        if (i > 0)
            oss << ", ";

        oss << "(" << _coords[i * 3] << ", " << _coords[i * 3 + 1] << ", " << _coords[i * 3 + 2] << ")";
    }

    oss << "]";

    return oss.str();
}

std::string Polyline::repr() const { return "Polyline(" + name + ", " + std::to_string(point_count()) + " points)"; }

std::ostream& operator<<(std::ostream& os, const Polyline& polyline) { return os << polyline.repr(); }

// ═══════════════════════════════════════════════════════════════════════════
// Private helpers
// ═══════════════════════════════════════════════════════════════════════════
void Polyline::recompute_plane_if_needed() { _plane_dirty = true; }

void Polyline::average_normal(Vector& avg_normal) const {

    if (point_count() < 3) {
        avg_normal = Vector(0, 0, 1);

        return;
    }

    const size_t n = is_closed() ? point_count() - 1 : point_count();
    avg_normal = Vector(0, 0, 0);

    for (size_t i = 0; i < n; i++) {
        const size_t prev = i == 0 ? n - 1 : i - 1;
        const size_t next = (i + 1) % n;
        const Vector v1 = get_point(i) - get_point(prev);
        const Vector v2 = get_point(next) - get_point(i);
        avg_normal += v1.cross(v2);
    }

    avg_normal.normalize_self();
}

Point Polyline::point_at_length(double distance) const {

    double acc = 0.0;

    for (size_t i = 0; i + 1 < point_count(); i++) {
        const Point a = get_point(i);
        const Point b = get_point(i + 1);
        const double seg_len = (b - a).magnitude();

        if (acc + seg_len >= distance) {
            const double t = seg_len > 1e-14 ? (distance - acc) / seg_len : 0.0;

            return a + (b - a) * t;
        }

        acc += seg_len;
    }

    return get_point(0);
}

void Polyline::project_to_plane(const Point& origin, const Vector& x_axis, const Vector& y_axis, std::vector<std::array<double, 2>>& pts2d) const {

    pts2d.clear();
    pts2d.reserve(point_count());

    for (size_t i = 0; i < point_count(); i++) {
        const Vector d = get_point(i) - origin;
        pts2d.push_back({d.dot(x_axis), d.dot(y_axis)});
    }
}

Point Polyline::unproject(const Point& origin, const Vector& x_axis, const Vector& y_axis, double u, double v) { return origin + x_axis * u + y_axis * v; }

void Polyline::quick_hull_recurse(const std::vector<std::array<double, 2>>& pts, double ax, double ay, double bx, double by, std::vector<std::array<double, 2>>& hull) {

    if (pts.empty())
        return;

    size_t fi = 0;
    double best = -std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < pts.size(); i++) {
        const double val = ccw_2d(ax, ay, bx, by, pts[i][0], pts[i][1]);

        if (val >= best) {
            best = val;
            fi = i;
        }
    }

    const double fx = pts[fi][0];
    const double fy = pts[fi][1];
    std::vector<std::array<double, 2>> left;
    std::vector<std::array<double, 2>> right;

    for (const std::array<double, 2>& p : pts) {
        if (ccw_2d(ax, ay, fx, fy, p[0], p[1]) > 0.0)
            left.push_back(p);

        if (ccw_2d(fx, fy, bx, by, p[0], p[1]) > 0.0)
            right.push_back(p);
    }

    quick_hull_recurse(left, ax, ay, fx, fy, hull);
    hull.push_back({fx, fy});
    quick_hull_recurse(right, fx, fy, bx, by, hull);
}

void Polyline::offset_polygon_2d(std::vector<std::array<double, 2>>& poly2d, double offset_dist) {

    const size_t n = poly2d.size();

    if (offset_dist == 0.0 || n < 3)
        return;

    double signed_area = 0.0;

    for (size_t i = 0; i < n; i++) {
        const std::array<double, 2>& a = poly2d[i];
        const std::array<double, 2>& b = poly2d[(i + 1) % n];
        signed_area += a[0] * b[1] - b[0] * a[1];
    }

    const double delta = signed_area < 0.0 ? -offset_dist : offset_dist;
    std::vector<std::array<double, 2>> normals;
    normals.reserve(n);

    for (size_t i = 0; i < n; i++) {
        const std::array<double, 2>& a = poly2d[i];
        const std::array<double, 2>& b = poly2d[(i + 1) % n];
        const double ex = b[0] - a[0];
        const double ey = b[1] - a[1];
        const double len = std::sqrt(ex * ex + ey * ey);

        if (len < 1e-12)
            normals.push_back({0.0, 0.0});
        else
            normals.push_back({ey / len, -ex / len});
    }

    std::vector<std::array<double, 2>> out;
    out.reserve(n * 3);

    for (size_t i = 0; i < n; i++) {
        const std::array<double, 2>& np = normals[(i + n - 1) % n];
        const std::array<double, 2>& nn = normals[i];
        const double cos_a = np[0] * nn[0] + np[1] * nn[1];
        const double sin_a = np[0] * nn[1] - np[1] * nn[0];
        const double denom = 1.0 + cos_a;
        const bool concave = cos_a > -0.999 && sin_a * delta < 0.0 && offset_dist > 0.0;

        if (concave) {
            out.push_back({poly2d[i][0] + np[0] * delta, poly2d[i][1] + np[1] * delta});
            out.push_back({poly2d[i][0], poly2d[i][1]});
            out.push_back({poly2d[i][0] + nn[0] * delta, poly2d[i][1] + nn[1] * delta});
        } else if (std::abs(denom) < 1e-9) {
            out.push_back({poly2d[i][0] + (np[0] + nn[0]) * 0.5 * delta, poly2d[i][1] + (np[1] + nn[1]) * 0.5 * delta});
        } else {
            out.push_back({poly2d[i][0] + (np[0] + nn[0]) / denom * delta, poly2d[i][1] + (np[1] + nn[1]) / denom * delta});
        }
    }

    double out_area = 0.0;

    for (size_t i = 0; i < out.size(); i++) {
        const std::array<double, 2>& a = out[i];
        const std::array<double, 2>& b = out[(i + 1) % out.size()];
        out_area += a[0] * b[1] - b[0] * a[1];
    }

    if (out.size() >= 3 && std::abs(out_area) > 1e-4)
        poly2d = std::move(out);
}

bool Polyline::point_in_polygon(const std::vector<std::array<double, 2>>& poly2d, double px, double py) {

    const size_t n = poly2d.size();
    bool inside = false;
    size_t j = n - 1;

    for (size_t i = 0; i < n; i++) {
        const double xi = poly2d[i][0];
        const double yi = poly2d[i][1];
        const double xj = poly2d[j][0];
        const double yj = poly2d[j][1];

        if ((yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi)
            inside = !inside;

        j = i;
    }

    return inside;
}

bool Polyline::closest_edge(const Point& center, const std::vector<Polyline>& polylines, size_t& edge_i, size_t& edge_j) {

    double best_sq = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < polylines.size(); i++) {
        for (size_t j = 0; j + 1 < polylines[i].point_count(); j++) {
            const Point a = polylines[i].get_point(j);
            const Vector e = polylines[i].get_point(j + 1) - a;
            const double len2 = e.magnitude_squared();

            if (len2 <= 0.0)
                continue;

            const double t = (center - a).dot(e) / len2;

            if (t < 0.0 || t > 1.0)
                continue;

            const double d2 = (center - (a + e * t)).magnitude_squared();

            if (d2 < best_sq) {
                best_sq = d2;
                edge_i = i;
                edge_j = j;
            }
        }
    }

    return best_sq < std::numeric_limits<double>::infinity();
}

Polyline Polyline::boolean_project(const Polyline& pl, const Plane& plane) {

    const Point& o = plane.origin();
    const Vector& x = plane.x_axis();
    const Vector& y = plane.y_axis();
    const size_t n = pl.point_count();
    Polyline p2d;
    p2d._coords.resize(n * 3);

    for (size_t i = 0; i < n; i++) {
        const Vector d = pl.get_point(i) - o;
        p2d._coords[i * 3] = d.dot(x);
        p2d._coords[i * 3 + 1] = d.dot(y);
        p2d._coords[i * 3 + 2] = 0.0;
    }

    if (n >= 4) {
        const double dx = p2d._coords[(n - 1) * 3] - p2d._coords[0];
        const double dy = p2d._coords[(n - 1) * 3 + 1] - p2d._coords[1];

        if (dx * dx + dy * dy < 1.0) {
            p2d._coords[(n - 1) * 3] = p2d._coords[0];
            p2d._coords[(n - 1) * 3 + 1] = p2d._coords[1];
        }
    }

    return p2d;
}

void Polyline::ensure_ccw(Polyline& p2d) {

    const size_t n = p2d.point_count();
    size_t m = n;

    if (m >= 4) {
        const double dx = p2d._coords[(m - 1) * 3] - p2d._coords[0];
        const double dy = p2d._coords[(m - 1) * 3 + 1] - p2d._coords[1];

        if (dx * dx + dy * dy < 1e-10)
            m--;
    }

    if (m < 3)
        return;

    double area = 0.0;

    for (size_t i = 0; i < m; i++) {
        const size_t j = (i + 1) % m;
        area += p2d._coords[i * 3] * p2d._coords[j * 3 + 1] - p2d._coords[j * 3] * p2d._coords[i * 3 + 1];
    }

    if (area < 0.0)
        p2d.reverse();
}

double Polyline::simplify_perp_dist(const Point& pt, const Point& line_start, const Point& line_end) {

    const Vector d = line_end - line_start;
    const double len_sq = d.magnitude_squared();

    if (len_sq == 0.0)
        return std::sqrt((pt - line_start).magnitude_squared());

    double t = (pt - line_start).dot(d) / len_sq;
    t = std::max(0.0, std::min(1.0, t));

    return std::sqrt((pt - (line_start + d * t)).magnitude_squared());
}

void Polyline::simplify_rdp(const std::vector<Point>& points, int start, int end, double tolerance, std::vector<bool>& keep) {

    if (end <= start + 1)
        return;

    double max_dist = 0.0;
    int max_idx = start;

    for (int i = start + 1; i < end; i++) {
        const double d = simplify_perp_dist(points[i], points[start], points[end]);

        if (d > max_dist) {
            max_dist = d;
            max_idx = i;
        }
    }

    if (max_dist <= tolerance)
        return;

    keep[max_idx] = true;
    simplify_rdp(points, start, max_idx, tolerance, keep);
    simplify_rdp(points, max_idx, end, tolerance, keep);
}

} // namespace session_cpp
