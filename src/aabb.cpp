#include "aabb.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "pointcloud.h"
#include "polyline.h"
#include "xform.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
AABB::AABB(double cx, double cy, double cz, double hx, double hy, double hz)
    : cx(cx), cy(cy), cz(cz), hx(hx), hy(hy), hz(hz) {}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
AABB AABB::from_point(const Point& point, double inflate) {
    return AABB(point[0], point[1], point[2], inflate, inflate, inflate);
}

AABB AABB::from_points(const std::vector<Point>& points, double inflate) {

    if (points.empty())
        return AABB();

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();

    for (const Point& pt : points) {
        min_x = std::min(min_x, pt[0]);
        min_y = std::min(min_y, pt[1]);
        min_z = std::min(min_z, pt[2]);
        max_x = std::max(max_x, pt[0]);
        max_y = std::max(max_y, pt[1]);
        max_z = std::max(max_z, pt[2]);
    }

    return AABB(
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
        (max_x - min_x) * 0.5 + inflate,
        (max_y - min_y) * 0.5 + inflate,
        (max_z - min_z) * 0.5 + inflate
    );
}

AABB AABB::from_line(const Line& line, double inflate) {
    return from_points({line.start(), line.end()}, inflate);
}

AABB AABB::from_polyline(const Polyline& polyline, double inflate) {
    return from_points(polyline.get_points(), inflate);
}

AABB AABB::from_mesh(const Mesh& mesh, double inflate) {
    return from_points(mesh.to_vertices_and_faces().first, inflate);
}

AABB AABB::from_pointcloud(const PointCloud& pointcloud, double inflate) {
    return from_points(pointcloud.get_points(), inflate);
}

AABB AABB::from_nurbscurve(const NurbsCurve& curve, double inflate, bool tight) {

    if (!curve.is_valid() || curve.cv_count() == 0)
        return AABB();

    std::vector<Point> points;

    if (!tight) {
        for (int i = 0; i < curve.cv_count(); i++)
            points.push_back(curve.get_cv(i));

        return from_points(points, inflate);
    }

    const double t0 = curve.domain_start();
    const double t1 = curve.domain_end();

    points.push_back(curve.point_at(t0));
    points.push_back(curve.point_at(t1));

    for (const double t : curve.get_span_vector())
        if (t > t0 && t < t1)
            points.push_back(curve.point_at(t));

    const double dt = (t1 - t0) / NUM_SAMPLES;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        const double t_start = t0 + i * dt;
        const double t_end = t_start + dt;
        const std::vector<Vector> deriv_start = curve.evaluate(t_start, 1);
        const std::vector<Vector> deriv_end = curve.evaluate(t_end, 1);

        if (deriv_start.size() < 2 || deriv_end.size() < 2)
            continue;

        for (int axis = 0; axis < 3; axis++) {
            const double d_start = deriv_start[1][axis];
            const double d_end = deriv_end[1][axis];

            if (d_start * d_end < 0) {
                const double t_root = compute_extremum(curve, axis, t_start, t_end, d_start);
                points.push_back(curve.point_at(t_root));
            } else if (d_start == 0.0 && d_end != 0.0)
                points.push_back(curve.point_at(t_start));
            else if (d_end == 0.0 && d_start != 0.0)
                points.push_back(curve.point_at(t_end));
        }
    }

    return from_points(points, inflate);
}

AABB AABB::from_nurbssurface(const NurbsSurface& surface, double inflate) {

    if (!surface.is_valid() || surface.cv_count(0) == 0 || surface.cv_count(1) == 0)
        return AABB();

    std::vector<Point> points;

    for (int i = 0; i < surface.cv_count(0); i++)
        for (int j = 0; j < surface.cv_count(1); j++)
            points.push_back(surface.get_cv(i, j));

    return from_points(points, inflate);
}

AABB AABB::merge(const AABB& a, const AABB& b) {

    if (!a.is_valid())
        return b;

    if (!b.is_valid())
        return a;

    const double min_x = std::min(a.cx - a.hx, b.cx - b.hx);
    const double min_y = std::min(a.cy - a.hy, b.cy - b.hy);
    const double min_z = std::min(a.cz - a.hz, b.cz - b.hz);
    const double max_x = std::max(a.cx + a.hx, b.cx + b.hx);
    const double max_y = std::max(a.cy + a.hy, b.cy + b.hy);
    const double max_z = std::max(a.cz + a.hz, b.cz + b.hz);

    return AABB(
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
        (max_x - min_x) * 0.5,
        (max_y - min_y) * 0.5,
        (max_z - min_z) * 0.5
    );
}

AABB AABB::empty() {
    return AABB(0.0, 0.0, 0.0, -1.0, -1.0, -1.0);
}

double AABB::compute_extremum(const NurbsCurve& curve, int axis, double t_lo, double t_hi, double d_start) {

    double t_root = (t_lo + t_hi) * 0.5;

    for (int it = 0; it < MAX_ITER; it++) {
        const std::vector<Vector> deriv = curve.evaluate(t_root, 2);

        if (deriv.size() < 3)
            break;

        const double d1 = deriv[1][axis];
        const double d2 = deriv[2][axis];

        if (std::abs(d1) < 1e-12)
            break;

        if (std::abs(d2) > 1e-14) {
            const double t_new = t_root - d1 / d2;

            if (t_new >= t_lo && t_new <= t_hi) {
                t_root = t_new;
            } else {
                if (d1 * d_start < 0)
                    t_hi = t_root;
                else
                    t_lo = t_root;

                t_root = (t_lo + t_hi) * 0.5;
            }
        } else {
            t_root = (t_lo + t_hi) * 0.5;
        }

        const std::vector<Vector> deriv_check = curve.evaluate(t_root, 1);

        if (deriv_check.size() < 2)
            continue;

        const double d_check = deriv_check[1][axis];

        if (d_check * d_start < 0) {
            t_hi = t_root;
        } else {
            t_lo = t_root;
            d_start = d_check;
        }
    }

    return t_root;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool AABB::operator==(const AABB& other) const {

    return std::round(cx * 1000000.0) == std::round(other.cx * 1000000.0) &&
           std::round(cy * 1000000.0) == std::round(other.cy * 1000000.0) &&
           std::round(cz * 1000000.0) == std::round(other.cz * 1000000.0) &&
           std::round(hx * 1000000.0) == std::round(other.hx * 1000000.0) &&
           std::round(hy * 1000000.0) == std::round(other.hy * 1000000.0) &&
           std::round(hz * 1000000.0) == std::round(other.hz * 1000000.0);
}

bool AABB::operator!=(const AABB& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
Point AABB::min_point() const {
    return Point(cx - hx, cy - hy, cz - hz);
}

Point AABB::max_point() const {
    return Point(cx + hx, cy + hy, cz + hz);
}

Point AABB::center() const {
    return Point(cx, cy, cz);
}

double AABB::area() const {
    return 8.0 * (hx * hy + hy * hz + hz * hx);
}

double AABB::diagonal() const {

    if (!is_valid())
        return 0.0;

    return 2.0 * std::sqrt(hx * hx + hy * hy + hz * hz);
}

double AABB::volume() const {
    return 8.0 * hx * hy * hz;
}

bool AABB::is_valid() const {
    return hx >= 0.0 && hy >= 0.0 && hz >= 0.0;
}

Point AABB::closest_point(const Point& pt) const {

    return Point(
        std::max(cx - hx, std::min(cx + hx, pt[0])),
        std::max(cy - hy, std::min(cy + hy, pt[1])),
        std::max(cz - hz, std::min(cz + hz, pt[2]))
    );
}

bool AABB::contains(const Point& pt) const {

    return pt[0] >= cx - hx && pt[0] <= cx + hx &&
           pt[1] >= cy - hy && pt[1] <= cy + hy &&
           pt[2] >= cz - hz && pt[2] <= cz + hz;
}

bool AABB::intersects(const AABB& other) const {

    return cx - hx <= other.cx + other.hx &&
           cx + hx >= other.cx - other.hx &&
           cy - hy <= other.cy + other.hy &&
           cy + hy >= other.cy - other.hy &&
           cz - hz <= other.cz + other.hz &&
           cz + hz >= other.cz - other.hz;
}

Point AABB::corner(bool x_max, bool y_max, bool z_max) const {

    return Point(
        cx + (x_max ? hx : -hx),
        cy + (y_max ? hy : -hy),
        cz + (z_max ? hz : -hz)
    );
}

std::array<Point, 8> AABB::corners() const {

    return {
        Point(cx + hx, cy + hy, cz - hz),
        Point(cx - hx, cy + hy, cz - hz),
        Point(cx - hx, cy - hy, cz - hz),
        Point(cx + hx, cy - hy, cz - hz),
        Point(cx + hx, cy + hy, cz + hz),
        Point(cx - hx, cy + hy, cz + hz),
        Point(cx - hx, cy - hy, cz + hz),
        Point(cx + hx, cy - hy, cz + hz),
    };
}

std::array<Point, 8> AABB::get_corners() const {
    return corners();
}

std::vector<Line> AABB::get_edges() const {

    const std::array<Point, 8> points = corners();

    return {
        Line::from_points(points[0], points[1]),
        Line::from_points(points[1], points[2]),
        Line::from_points(points[2], points[3]),
        Line::from_points(points[3], points[0]),
        Line::from_points(points[4], points[5]),
        Line::from_points(points[5], points[6]),
        Line::from_points(points[6], points[7]),
        Line::from_points(points[7], points[4]),
        Line::from_points(points[0], points[4]),
        Line::from_points(points[1], points[5]),
        Line::from_points(points[2], points[6]),
        Line::from_points(points[3], points[7]),
    };
}

Point AABB::point_at(double x, double y, double z) const {
    return Point(cx + x, cy + y, cz + z);
}

void AABB::inflate(double amount) {

    hx += amount;
    hy += amount;
    hz += amount;
}

void AABB::union_with(const AABB& other) {
    *this = merge(*this, other);
}

void AABB::union_with_point(double x, double y, double z) {
    *this = merge(*this, AABB(x, y, z, 0.0, 0.0, 0.0));
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void AABB::transform(const Xform& xform) {
    *this = transformed(xform);
}

AABB AABB::transformed(const Xform& xform) const {

    if (!is_valid())
        return *this;

    AABB out = AABB::empty();

    for (const Point& point : corners()) {
        const Point moved = xform.transform_point(point);
        out.union_with_point(moved[0], moved[1], moved[2]);
    }

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string AABB::str() const {

    const int prec = static_cast<int>(Tolerance::ROUNDING);

    return fmt::format(
        "{}, {}, {}, {}, {}, {}",
        TOLERANCE.format_number(cx, prec),
        TOLERANCE.format_number(cy, prec),
        TOLERANCE.format_number(cz, prec),
        TOLERANCE.format_number(hx, prec),
        TOLERANCE.format_number(hy, prec),
        TOLERANCE.format_number(hz, prec)
    );
}

std::string AABB::repr() const {
    return fmt::format("AABB({})", str());
}

std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
    return os << aabb.str();
}

} // namespace session_cpp
