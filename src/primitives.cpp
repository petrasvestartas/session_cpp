#include "primitives.h"
#include "nurbssurface.h"
#include "tolerance.h"
#include "knot.h"
#include <cmath>

namespace session_cpp {

NurbsCurve Primitives::circle(double cx, double cy, double cz, double radius) {
    // Circle as rational quadratic NURBS with 9 control points
    // Uses the standard 9-point representation for a full circle
    const double w = std::sqrt(2.0) / 2.0;  // Weight for corner points

    NurbsCurve curve(3, true, 3, 9);  // 3D, rational, order 3 (quadratic), 9 CVs

    // Control points around circle (every 45 degrees)
    // Homogeneous coordinates: [x*w, y*w, z*w, w] for weighted points
    double angles[] = {0, Tolerance::PI/4, Tolerance::PI/2, 3*Tolerance::PI/4,
                       Tolerance::PI, 5*Tolerance::PI/4, 3*Tolerance::PI/2, 7*Tolerance::PI/4, 2*Tolerance::PI};
    double weights[] = {1, w, 1, w, 1, w, 1, w, 1};

    for (int i = 0; i < 9; i++) {
        double x = cx + radius * std::cos(angles[i]);
        double y = cy + radius * std::sin(angles[i]);
        double z = cz;
        curve.set_cv_4d(i, x * weights[i], y * weights[i], z * weights[i], weights[i]);
    }

    // Knot vector for closed curve: 0,0,0, 1,1, 2,2, 3,3, 4,4,4 (12 knots = 9 CVs + order 3)
    double knots[] = {0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4};
    for (int i = 0; i < 12; i++) {
        curve.set_knot(i, knots[i]);
    }

    return curve;
}

NurbsCurve Primitives::ellipse(double cx, double cy, double cz, double major_radius, double minor_radius) {
    // Ellipse as rational quadratic NURBS with 9 control points
    const double w = std::sqrt(2.0) / 2.0;

    NurbsCurve curve(3, true, 3, 9);

    double angles[] = {0, Tolerance::PI/4, Tolerance::PI/2, 3*Tolerance::PI/4,
                       Tolerance::PI, 5*Tolerance::PI/4, 3*Tolerance::PI/2, 7*Tolerance::PI/4, 2*Tolerance::PI};
    double weights[] = {1, w, 1, w, 1, w, 1, w, 1};

    for (int i = 0; i < 9; i++) {
        double x = cx + major_radius * std::cos(angles[i]);
        double y = cy + minor_radius * std::sin(angles[i]);
        double z = cz;
        curve.set_cv_4d(i, x * weights[i], y * weights[i], z * weights[i], weights[i]);
    }

    double knots[] = {0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4};
    for (int i = 0; i < 12; i++) {
        curve.set_knot(i, knots[i]);
    }

    return curve;
}

NurbsCurve Primitives::arc(const Point& start, const Point& mid, const Point& end) {
    // Arc through 3 points as a rational quadratic NURBS
    // Uses circle fitting algorithm

    // Find circle center through 3 points
    Vector d1(mid[0] - start[0], mid[1] - start[1], mid[2] - start[2]);
    Vector d2(end[0] - mid[0], end[1] - mid[1], end[2] - mid[2]);

    Point m1((start[0] + mid[0]) / 2, (start[1] + mid[1]) / 2, (start[2] + mid[2]) / 2);
    Point m2((mid[0] + end[0]) / 2, (mid[1] + end[1]) / 2, (mid[2] + end[2]) / 2);

    // Normal to the plane
    Vector normal = d1.cross(d2);
    double normal_len = normal.magnitude();

    if (normal_len < Tolerance::ZERO_TOLERANCE) {
        // Points are collinear, create a line instead
        std::vector<Point> pts = {start, end};
        return NurbsCurve::create(false, 1, pts);
    }

    normal = normal.normalize();

    // Perpendicular bisectors
    Vector perp1 = d1.cross(normal).normalize();
    Vector perp2 = d2.cross(normal).normalize();

    // Find intersection of perpendicular bisectors (circle center)
    // Solve: m1 + t1*perp1 = m2 + t2*perp2
    double denom = perp1[0] * perp2[1] - perp1[1] * perp2[0];
    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE) {
        denom = perp1[0] * perp2[2] - perp1[2] * perp2[0];
    }

    double t1 = 0;
    if (std::abs(denom) > Tolerance::ZERO_TOLERANCE) {
        double dx = m2[0] - m1[0];
        double dy = m2[1] - m1[1];
        t1 = (dx * perp2[1] - dy * perp2[0]) / denom;
    }

    Point center(m1[0] + t1 * perp1[0], m1[1] + t1 * perp1[1], m1[2] + t1 * perp1[2]);
    double radius = center.distance(start);

    // Calculate angle subtended
    Vector v1(start[0] - center[0], start[1] - center[1], start[2] - center[2]);
    Vector v2(end[0] - center[0], end[1] - center[1], end[2] - center[2]);
    double dot = v1.dot(v2) / (v1.magnitude() * v2.magnitude());
    dot = std::max(-1.0, std::min(1.0, dot));
    double angle = std::acos(dot);

    // Check direction using mid point
    Vector vm(mid[0] - center[0], mid[1] - center[1], mid[2] - center[2]);
    Vector cross = v1.cross(vm);
    if (cross.dot(normal) < 0) {
        angle = 2 * Tolerance::PI - angle;
    }

    // Create rational arc
    double half_angle = angle / 2;
    double w = std::cos(half_angle);

    NurbsCurve curve(3, true, 3, 3);

    // Control points: start, shoulder, end
    Point shoulder(center[0] + radius * std::cos(half_angle) * (v1[0]/radius + std::tan(half_angle) * perp1[0]),
                   center[1] + radius * std::cos(half_angle) * (v1[1]/radius + std::tan(half_angle) * perp1[1]),
                   center[2] + radius * std::cos(half_angle) * (v1[2]/radius + std::tan(half_angle) * perp1[2]));

    // Simpler shoulder calculation
    Vector tangent1 = normal.cross(v1).normalize();
    double shoulder_dist = radius / std::cos(half_angle);
    shoulder = Point(center[0] + shoulder_dist * (v1[0] + v2[0]) / (v1.magnitude() + v2.magnitude()) * v1.magnitude(),
                     center[1] + shoulder_dist * (v1[1] + v2[1]) / (v1.magnitude() + v2.magnitude()) * v1.magnitude(),
                     center[2] + shoulder_dist * (v1[2] + v2[2]) / (v1.magnitude() + v2.magnitude()) * v1.magnitude());

    // Use mid point projected calculation
    shoulder = Point((start[0] + end[0]) / 2 + (mid[0] - (start[0] + end[0]) / 2) / w,
                     (start[1] + end[1]) / 2 + (mid[1] - (start[1] + end[1]) / 2) / w,
                     (start[2] + end[2]) / 2 + (mid[2] - (start[2] + end[2]) / 2) / w);

    curve.set_cv_4d(0, start[0], start[1], start[2], 1.0);
    curve.set_cv_4d(1, shoulder[0] * w, shoulder[1] * w, shoulder[2] * w, w);
    curve.set_cv_4d(2, end[0], end[1], end[2], 1.0);

    curve.set_knot(0, 0);
    curve.set_knot(1, 0);
    curve.set_knot(2, 1);
    curve.set_knot(3, 1);

    return curve;
}

NurbsCurve Primitives::parabola(const Point& p0, const Point& p1, const Point& p2) {
    // Parabola through 3 points as a non-rational quadratic NURBS
    // For a parabola, the middle CV is the vertex of the parabola

    NurbsCurve curve(3, false, 3, 3);

    // Control points: endpoints and computed middle point
    // For parabola: CV1 = 2*P1 - (P0+P2)/2 (where P1 is the apex)
    Point cv1(2 * p1[0] - (p0[0] + p2[0]) / 2,
              2 * p1[1] - (p0[1] + p2[1]) / 2,
              2 * p1[2] - (p0[2] + p2[2]) / 2);

    curve.set_cv(0, p0);
    curve.set_cv(1, cv1);
    curve.set_cv(2, p2);

    curve.set_knot(0, 0);
    curve.set_knot(1, 0);
    curve.set_knot(2, 1);
    curve.set_knot(3, 1);

    return curve;
}

NurbsCurve Primitives::hyperbola(const Point& center, double a, double b, double extent) {
    // Hyperbola as a rational NURBS curve
    // x = a * cosh(t), y = b * sinh(t)
    // Approximate with multiple segments

    int num_segments = 8;
    int cv_count = num_segments + 1;
    NurbsCurve curve(3, false, 4, cv_count);  // Cubic non-rational

    for (int i = 0; i <= num_segments; i++) {
        double t = -extent + 2 * extent * i / num_segments;
        double x = center[0] + a * std::cosh(t);
        double y = center[1] + b * std::sinh(t);
        double z = center[2];
        curve.set_cv(i, Point(x, y, z));
    }

    // Create clamped uniform knot vector
    int knot_count = cv_count + 4 - 2;
    double k = 0;
    for (int i = 0; i < knot_count; i++) {
        if (i < 3) {
            curve.set_knot(i, 0);
        } else if (i >= knot_count - 3) {
            curve.set_knot(i, num_segments - 2);
        } else {
            curve.set_knot(i, i - 2);
        }
    }

    return curve;
}

NurbsCurve Primitives::spiral(double start_radius, double end_radius, double pitch, double turns) {
    // Spiral (helix with varying radius)
    // Parametric: x = r(t)*cos(t), y = r(t)*sin(t), z = pitch*t/(2*pi)
    // where r(t) varies linearly from start_radius to end_radius

    int segments_per_turn = 8;
    int total_segments = static_cast<int>(turns * segments_per_turn);
    if (total_segments < 4) total_segments = 4;

    int cv_count = total_segments + 1;
    NurbsCurve curve(3, false, 4, cv_count);  // Cubic

    double total_angle = turns * 2 * Tolerance::PI;

    for (int i = 0; i <= total_segments; i++) {
        double t = static_cast<double>(i) / total_segments;
        double angle = t * total_angle;
        double r = start_radius + t * (end_radius - start_radius);
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);
        double z = t * turns * pitch;
        curve.set_cv(i, Point(x, y, z));
    }

    curve.m_knot = knot::make_clamped_uniform(curve.m_order, curve.m_cv_count, 1.0);

    return curve;
}

NurbsSurface Primitives::cylinder_surface(double cx, double cy, double cz, double radius, double height) {
    const double w = std::sqrt(2.0) / 2.0;
    double circle_weights[] = {1, w, 1, w, 1, w, 1, w, 1};
    double circle_x[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double circle_y[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double u_knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    double v_knots[] = {0, 1};

    NurbsSurface srf(3, true, 3, 2, 9, 2);

    for (int i = 0; i < 10; i++) srf.set_knot(0, i, u_knots[i]);
    for (int i = 0; i < 2; i++) srf.set_knot(1, i, v_knots[i]);

    for (int i = 0; i < 9; i++) {
        double wi = circle_weights[i];
        double px = cx + radius * circle_x[i];
        double py = cy + radius * circle_y[i];
        srf.set_cv_4d(i, 0, px * wi, py * wi, cz * wi, wi);
        srf.set_cv_4d(i, 1, px * wi, py * wi, (cz + height) * wi, wi);
    }

    return srf;
}

NurbsSurface Primitives::cone_surface(double cx, double cy, double cz, double radius, double height) {
    const double w = std::sqrt(2.0) / 2.0;
    double circle_weights[] = {1, w, 1, w, 1, w, 1, w, 1};
    double circle_x[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double circle_y[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double u_knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    double v_knots[] = {0, 1};

    NurbsSurface srf(3, true, 3, 2, 9, 2);

    for (int i = 0; i < 10; i++) srf.set_knot(0, i, u_knots[i]);
    for (int i = 0; i < 2; i++) srf.set_knot(1, i, v_knots[i]);

    double apex_z = cz + height;
    for (int i = 0; i < 9; i++) {
        double wi = circle_weights[i];
        double px = cx + radius * circle_x[i];
        double py = cy + radius * circle_y[i];
        srf.set_cv_4d(i, 0, px * wi, py * wi, cz * wi, wi);
        srf.set_cv_4d(i, 1, cx * wi, cy * wi, apex_z * wi, wi);
    }

    return srf;
}

NurbsSurface Primitives::torus_surface(double cx, double cy, double cz, double major_radius, double minor_radius) {
    const double w = std::sqrt(2.0) / 2.0;
    double cw[] = {1, w, 1, w, 1, w, 1, w, 1};
    double cos_a[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double sin_a[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double u_knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};

    NurbsSurface srf(3, true, 3, 3, 9, 9);

    for (int d = 0; d < 2; d++)
        for (int i = 0; i < 10; i++)
            srf.set_knot(d, i, u_knots[i]);

    for (int i = 0; i < 9; i++) {
        double ca = cos_a[i];
        double sa = sin_a[i];
        for (int j = 0; j < 9; j++) {
            double cb = cos_a[j];
            double sb = sin_a[j];
            double r = major_radius + minor_radius * cb;
            double px = cx + r * ca;
            double py = cy + r * sa;
            double pz = cz + minor_radius * sb;
            double wij = cw[i] * cw[j];
            srf.set_cv_4d(i, j, px * wij, py * wij, pz * wij, wij);
        }
    }

    return srf;
}

} // namespace session_cpp
