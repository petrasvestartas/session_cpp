#include "primitives.h"
#include "nurbssurface.h"
#include "tolerance.h"
#include "knot.h"
#include <cmath>

namespace session_cpp {

NurbsCurve Primitives::circle(double cx, double cy, double cz, double radius) {
    const double w = std::sqrt(2.0) / 2.0;
    // Tangent intersection pattern: on-circle CVs at cardinal points,
    // weighted CVs at tangent intersections (NOT on circle)
    double circle_x[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double circle_y[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double weights[] = {1, w, 1, w, 1, w, 1, w, 1};

    NurbsCurve curve(3, true, 3, 9);
    double knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    for (int i = 0; i < 10; i++) curve.set_knot(i, knots[i]);

    for (int i = 0; i < 9; i++) {
        double px = cx + radius * circle_x[i];
        double py = cy + radius * circle_y[i];
        curve.set_cv_4d(i, px * weights[i], py * weights[i], cz * weights[i], weights[i]);
    }
    return curve;
}

NurbsCurve Primitives::ellipse(double cx, double cy, double cz, double major_radius, double minor_radius) {
    const double w = std::sqrt(2.0) / 2.0;
    double ex[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double ey[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double weights[] = {1, w, 1, w, 1, w, 1, w, 1};

    NurbsCurve curve(3, true, 3, 9);
    double knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    for (int i = 0; i < 10; i++) curve.set_knot(i, knots[i]);

    for (int i = 0; i < 9; i++) {
        double px = cx + major_radius * ex[i];
        double py = cy + minor_radius * ey[i];
        curve.set_cv_4d(i, px * weights[i], py * weights[i], cz * weights[i], weights[i]);
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

NurbsSurface Primitives::sphere_surface(double cx, double cy, double cz, double radius) {
    const double w = std::sqrt(2.0) / 2.0;
    double cw[] = {1, w, 1, w, 1, w, 1, w, 1};
    double cos_a[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double sin_a[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double u_knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    double v_knots[] = {0, 0, 1, 1, 2, 2};
    double lat_r[] = {0, 1, 1, 1, 0};
    double lat_z[] = {-1, -1, 0, 1, 1};
    double lat_w[] = {1, w, 1, w, 1};

    NurbsSurface srf(3, true, 3, 3, 9, 5);

    for (int i = 0; i < 10; i++) srf.set_knot(0, i, u_knots[i]);
    for (int i = 0; i < 6; i++) srf.set_knot(1, i, v_knots[i]);

    for (int j = 0; j < 5; j++) {
        double r = radius * lat_r[j];
        double pz = cz + radius * lat_z[j];
        double wj = lat_w[j];
        for (int i = 0; i < 9; i++) {
            double px = cx + r * cos_a[i];
            double py = cy + r * sin_a[i];
            double wij = cw[i] * wj;
            srf.set_cv_4d(i, j, px * wij, py * wij, pz * wij, wij);
        }
    }

    return srf;
}

std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>> Primitives::unit_cylinder_geometry() {
    std::vector<Point> vertices = {
        Point(0.5, 0.0, -0.5),
        Point(0.404508, 0.293893, -0.5),
        Point(0.154508, 0.475528, -0.5),
        Point(-0.154508, 0.475528, -0.5),
        Point(-0.404508, 0.293893, -0.5),
        Point(-0.5, 0.0, -0.5),
        Point(-0.404508, -0.293893, -0.5),
        Point(-0.154508, -0.475528, -0.5),
        Point(0.154508, -0.475528, -0.5),
        Point(0.404508, -0.293893, -0.5),
        Point(0.5, 0.0, 0.5),
        Point(0.404508, 0.293893, 0.5),
        Point(0.154508, 0.475528, 0.5),
        Point(-0.154508, 0.475528, 0.5),
        Point(-0.404508, 0.293893, 0.5),
        Point(-0.5, 0.0, 0.5),
        Point(-0.404508, -0.293893, 0.5),
        Point(-0.154508, -0.475528, 0.5),
        Point(0.154508, -0.475528, 0.5),
        Point(0.404508, -0.293893, 0.5)
    };
    std::vector<std::array<size_t, 3>> triangles = {
        {0, 1, 11}, {0, 11, 10},
        {1, 2, 12}, {1, 12, 11},
        {2, 3, 13}, {2, 13, 12},
        {3, 4, 14}, {3, 14, 13},
        {4, 5, 15}, {4, 15, 14},
        {5, 6, 16}, {5, 16, 15},
        {6, 7, 17}, {6, 17, 16},
        {7, 8, 18}, {7, 18, 17},
        {8, 9, 19}, {8, 19, 18},
        {9, 0, 10}, {9, 10, 19}
    };
    return {vertices, triangles};
}

std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>> Primitives::unit_cone_geometry() {
    std::vector<Point> vertices = {
        Point(0.0, 0.0, 0.5),
        Point(0.5, 0.0, -0.5),
        Point(0.353553, -0.353553, -0.5),
        Point(0.0, -0.5, -0.5),
        Point(-0.353553, -0.353553, -0.5),
        Point(-0.5, 0.0, -0.5),
        Point(-0.353553, 0.353553, -0.5),
        Point(0.0, 0.5, -0.5),
        Point(0.353553, 0.353553, -0.5)
    };
    std::vector<std::array<size_t, 3>> triangles = {
        {0, 2, 1}, {0, 3, 2}, {0, 4, 3}, {0, 5, 4},
        {0, 6, 5}, {0, 7, 6}, {0, 8, 7}, {0, 1, 8}
    };
    return {vertices, triangles};
}

Xform Primitives::line_to_cylinder_transform(const Line& line, double radius) {
    Point start = line.start();
    Point end = line.end();
    Vector line_vec = line.to_vector();
    double length = line.length();

    Vector z_axis = line_vec;
    z_axis.normalize_self();
    Vector x_axis;
    if (std::abs(z_axis[2]) < 0.9) {
        x_axis = Vector(0.0, 0.0, 1.0).cross(z_axis);
        x_axis.normalize_self();
    } else {
        x_axis = Vector(1.0, 0.0, 0.0).cross(z_axis);
        x_axis.normalize_self();
    }
    Vector y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();

    Xform scale = Xform::scale_xyz(radius * 2.0, radius * 2.0, length);
    Xform rotation;
    rotation.m[0] = x_axis[0]; rotation.m[1] = x_axis[1]; rotation.m[2] = x_axis[2];
    rotation.m[4] = y_axis[0]; rotation.m[5] = y_axis[1]; rotation.m[6] = y_axis[2];
    rotation.m[8] = z_axis[0]; rotation.m[9] = z_axis[1]; rotation.m[10] = z_axis[2];

    Point center((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5);
    Xform translation = Xform::translation(center[0], center[1], center[2]);
    return translation * rotation * scale;
}

Mesh Primitives::transform_geometry(
    const std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>>& geometry,
    const Xform& xform
) {
    const auto& [vertices, triangles] = geometry;
    Mesh mesh;
    std::vector<size_t> vertex_keys;
    vertex_keys.reserve(vertices.size());
    for (const auto& v : vertices) {
        Point transformed = xform.transformed_point(v);
        vertex_keys.push_back(mesh.add_vertex(transformed));
    }
    for (const auto& tri : triangles) {
        std::vector<size_t> face_vertices = {vertex_keys[tri[0]], vertex_keys[tri[1]], vertex_keys[tri[2]]};
        mesh.add_face(face_vertices);
    }
    return mesh;
}

Mesh Primitives::cylinder_mesh(const Line& line, double radius) {
    auto unit_cyl = unit_cylinder_geometry();
    Xform xform = line_to_cylinder_transform(line, radius);
    return transform_geometry(unit_cyl, xform);
}

Mesh Primitives::tetrahedron(double edge) {
    double a = edge / 2.0;
    double h = edge * std::sqrt(2.0 / 3.0);
    double r = edge / std::sqrt(3.0);
    double z0 = -h / 4.0;
    double z1 = 3.0 * h / 4.0;
    std::vector<std::vector<Point>> faces = {
        {Point(a, -r/2.0, z0), Point(-a, -r/2.0, z0), Point(0, r, z0)},
        {Point(0, 0, z1), Point(-a, -r/2.0, z0), Point(a, -r/2.0, z0)},
        {Point(0, 0, z1), Point(0, r, z0), Point(-a, -r/2.0, z0)},
        {Point(0, 0, z1), Point(a, -r/2.0, z0), Point(0, r, z0)},
    };
    return Mesh::from_polygons(faces, 1e-10);
}

Mesh Primitives::cube(double edge) {
    double a = edge / 2.0;
    Point v0(-a,-a,-a), v1(a,-a,-a), v2(a,a,-a), v3(-a,a,-a);
    Point v4(-a,-a,a), v5(a,-a,a), v6(a,a,a), v7(-a,a,a);
    std::vector<std::vector<Point>> faces = {
        {v3, v2, v1, v0},
        {v4, v5, v6, v7},
        {v0, v1, v5, v4},
        {v2, v3, v7, v6},
        {v0, v4, v7, v3},
        {v1, v2, v6, v5},
    };
    return Mesh::from_polygons(faces, 1e-10);
}

Mesh Primitives::octahedron(double edge) {
    double a = edge / std::sqrt(2.0);
    Point px(a,0,0), nx(-a,0,0);
    Point py(0,a,0), ny(0,-a,0);
    Point pz(0,0,a), nz(0,0,-a);
    std::vector<std::vector<Point>> faces = {
        {pz, px, py}, {pz, py, nx}, {pz, nx, ny}, {pz, ny, px},
        {nz, py, px}, {nz, nx, py}, {nz, ny, nx}, {nz, px, ny},
    };
    return Mesh::from_polygons(faces, 1e-10);
}

Mesh Primitives::icosahedron(double edge) {
    double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    double s = edge / 2.0;
    double sp = s * phi;
    Point verts[12] = {
        Point(-s, sp, 0), Point(s, sp, 0), Point(-s,-sp, 0), Point(s,-sp, 0),
        Point(0,-s, sp), Point(0, s, sp), Point(0,-s,-sp), Point(0, s,-sp),
        Point(sp, 0,-s), Point(sp, 0, s), Point(-sp, 0,-s), Point(-sp, 0, s),
    };
    int idx[20][3] = {
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1},
    };
    std::vector<std::vector<Point>> faces;
    for (auto& f : idx)
        faces.push_back({verts[f[0]], verts[f[1]], verts[f[2]]});
    return Mesh::from_polygons(faces, 1e-10);
}

Mesh Primitives::dodecahedron(double edge) {
    double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    double ip = 1.0 / phi;
    double s = edge / (2.0 * ip); // scale so actual edge length = edge
    Point verts[20] = {
        Point( s,  s,  s), Point( s,  s, -s), Point( s, -s,  s), Point( s, -s, -s),
        Point(-s,  s,  s), Point(-s,  s, -s), Point(-s, -s,  s), Point(-s, -s, -s),
        Point(0,  s*ip,  s*phi), Point(0,  s*ip, -s*phi),
        Point(0, -s*ip,  s*phi), Point(0, -s*ip, -s*phi),
        Point( s*ip,  s*phi, 0), Point( s*ip, -s*phi, 0),
        Point(-s*ip,  s*phi, 0), Point(-s*ip, -s*phi, 0),
        Point( s*phi, 0,  s*ip), Point( s*phi, 0, -s*ip),
        Point(-s*phi, 0,  s*ip), Point(-s*phi, 0, -s*ip),
    };
    int idx[12][5] = {
        {0, 8,10, 2,16}, {0,16,17, 1,12}, {0,12,14, 4, 8},
        {1,17, 3,11, 9}, {1, 9, 5,14,12}, {2,10, 6,15,13},
        {2,13, 3,17,16}, {3,13,15, 7,11}, {4,14, 5,19,18},
        {4,18, 6,10, 8}, {5, 9,11, 7,19}, {6,18,19, 7,15},
    };
    std::vector<std::vector<Point>> faces;
    for (auto& f : idx)
        faces.push_back({verts[f[0]], verts[f[1]], verts[f[2]], verts[f[3]], verts[f[4]]});
    return Mesh::from_polygons(faces, 1e-10);
}

Mesh Primitives::arrow_mesh(const Line& line, double radius) {
    Point start = line.start();
    Vector line_vec = line.to_vector();
    double length = line.length();

    Vector z_axis = line_vec;
    z_axis.normalize_self();
    Vector x_axis;
    if (std::abs(z_axis[2]) < 0.9) {
        x_axis = Vector(0.0, 0.0, 1.0).cross(z_axis);
        x_axis.normalize_self();
    } else {
        x_axis = Vector(1.0, 0.0, 0.0).cross(z_axis);
        x_axis.normalize_self();
    }
    Vector y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();

    double cone_length = length * 0.2;
    double body_length = length * 0.8;

    Point body_center(
        start[0] + line_vec[0] * 0.4,
        start[1] + line_vec[1] * 0.4,
        start[2] + line_vec[2] * 0.4
    );
    Point cone_base_center(
        start[0] + line_vec[0] * 0.9,
        start[1] + line_vec[1] * 0.9,
        start[2] + line_vec[2] * 0.9
    );

    Xform body_scale = Xform::scale_xyz(radius * 2.0, radius * 2.0, body_length);
    Point origin(0.0, 0.0, 0.0);
    Xform rotation = Xform::xy_to_plane(origin, x_axis, y_axis, z_axis);
    Xform body_translation = Xform::translation(body_center[0], body_center[1], body_center[2]);
    Xform body_xform = body_translation * rotation * body_scale;

    Xform cone_scale = Xform::scale_xyz(radius * 3.0, radius * 3.0, cone_length);
    Xform cone_translation = Xform::translation(cone_base_center[0], cone_base_center[1], cone_base_center[2]);
    Xform cone_xform = cone_translation * rotation * cone_scale;

    auto body_geometry = unit_cylinder_geometry();
    auto cone_geometry = unit_cone_geometry();

    Mesh mesh;

    std::vector<size_t> body_vertex_map;
    for (const auto& v : body_geometry.first) {
        Point transformed = body_xform.transformed_point(v);
        body_vertex_map.push_back(mesh.add_vertex(transformed));
    }
    for (const auto& tri : body_geometry.second) {
        std::vector<size_t> face_vertices = {body_vertex_map[tri[0]], body_vertex_map[tri[1]], body_vertex_map[tri[2]]};
        mesh.add_face(face_vertices);
    }

    std::vector<size_t> cone_vertex_map;
    for (const auto& v : cone_geometry.first) {
        Point transformed = cone_xform.transformed_point(v);
        cone_vertex_map.push_back(mesh.add_vertex(transformed));
    }
    for (const auto& tri : cone_geometry.second) {
        std::vector<size_t> face_vertices = {cone_vertex_map[tri[0]], cone_vertex_map[tri[1]], cone_vertex_map[tri[2]]};
        mesh.add_face(face_vertices);
    }

    return mesh;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Surface Factory Methods (moved from NurbsSurface)
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

std::vector<double> merge_knot_vectors(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-10) {
    std::vector<double> merged;
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        if (std::abs(a[i] - b[j]) < tol) {
            merged.push_back(a[i]);
            i++; j++;
        } else if (a[i] < b[j]) {
            merged.push_back(a[i]);
            i++;
        } else {
            merged.push_back(b[j]);
            j++;
        }
    }
    while (i < a.size()) { merged.push_back(a[i]); i++; }
    while (j < b.size()) { merged.push_back(b[j]); j++; }
    return merged;
}

bool knot_vectors_equal(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-10) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) {
        if (std::abs(a[i] - b[i]) > tol) return false;
    }
    return true;
}

void make_curves_compatible(std::vector<NurbsCurve>& curves) {
    if (curves.size() < 2) return;

    int max_deg = 0;
    for (auto& c : curves) {
        if (c.degree() > max_deg) max_deg = c.degree();
    }

    for (auto& c : curves) {
        if (c.degree() < max_deg) c.increase_degree(max_deg);
    }

    bool any_rational = false;
    for (auto& c : curves) {
        if (c.is_rational()) { any_rational = true; break; }
    }
    if (any_rational) {
        for (auto& c : curves) c.make_rational();
    }

    bool already_compatible = true;
    for (size_t i = 1; i < curves.size(); i++) {
        if (curves[i].cv_count() != curves[0].cv_count() ||
            !knot_vectors_equal(curves[i].get_knots(), curves[0].get_knots())) {
            already_compatible = false;
            break;
        }
    }
    if (already_compatible) return;

    for (auto& c : curves) {
        c.set_domain(0.0, 1.0);
    }

    std::vector<double> unified = curves[0].get_knots();
    for (size_t i = 1; i < curves.size(); i++) {
        unified = merge_knot_vectors(unified, curves[i].get_knots());
    }

    const double tol = 1e-10;
    for (auto& c : curves) {
        std::vector<double> cur_knots = c.get_knots();
        size_t ci = 0;
        for (size_t ui = 0; ui < unified.size(); ui++) {
            if (ci < cur_knots.size() && std::abs(cur_knots[ci] - unified[ui]) < tol) {
                ci++;
            } else {
                c.insert_knot(unified[ui], 1);
            }
        }
    }
}

bool find_curve_intersection(const NurbsCurve& crv1, const NurbsCurve& crv2,
                             double& t1, double& t2, Point& pt) {
    auto [a1, b1] = crv1.domain();
    auto [a2, b2] = crv2.domain();
    int n = 80;
    double best_d2 = 1e30;
    t1 = a1; t2 = a2;

    for (int i = 0; i <= n; i++) {
        double s1 = a1 + (b1 - a1) * i / n;
        Point p1 = crv1.point_at(s1);
        for (int j = 0; j <= n; j++) {
            double s2 = a2 + (b2 - a2) * j / n;
            Point p2 = crv2.point_at(s2);
            double dx = p1[0]-p2[0], dy = p1[1]-p2[1], dz = p1[2]-p2[2];
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < best_d2) { best_d2 = d2; t1 = s1; t2 = s2; }
        }
    }

    double dt1 = (b1 - a1) / n, dt2 = (b2 - a2) / n;
    for (int refine = 0; refine < 4; refine++) {
        double lo1 = std::max(a1, t1 - dt1), hi1 = std::min(b1, t1 + dt1);
        double lo2 = std::max(a2, t2 - dt2), hi2 = std::min(b2, t2 + dt2);
        for (int i = 0; i <= 20; i++) {
            double s1 = lo1 + (hi1 - lo1) * i / 20;
            Point p1 = crv1.point_at(s1);
            for (int j = 0; j <= 20; j++) {
                double s2 = lo2 + (hi2 - lo2) * j / 20;
                Point p2 = crv2.point_at(s2);
                double dx = p1[0]-p2[0], dy = p1[1]-p2[1], dz = p1[2]-p2[2];
                double d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < best_d2) { best_d2 = d2; t1 = s1; t2 = s2; }
            }
        }
        dt1 /= 20; dt2 /= 20;
    }

    Point p1 = crv1.point_at(t1), p2 = crv2.point_at(t2);
    pt = Point((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2);
    return best_d2 < 1.0;
}

double arc_length_between(const NurbsCurve& crv, double t0, double t1) {
    const int n = 10;
    double length = 0.0;
    Point prev = crv.point_at(t0);
    for (int i = 1; i <= n; i++) {
        double t = t0 + (t1 - t0) * i / n;
        Point curr = crv.point_at(t);
        double dx = curr[0]-prev[0], dy = curr[1]-prev[1], dz = curr[2]-prev[2];
        length += std::sqrt(dx*dx + dy*dy + dz*dz);
        prev = curr;
    }
    return length;
}

// TL-style cubic interpolation: cv_count = n+2, Bessel tangents, tridiagonal solve
NurbsCurve cubic_interpolation_curve(const std::vector<Point>& points, const std::vector<double>& params) {
    int n = (int)points.size();
    if (n < 2) return NurbsCurve();
    int dim = 3;
    int degree = std::min(3, n - 1);
    int order = degree + 1;

    if (degree < 3) {
        int cv_count = n;
        if (degree == 1) {
            // Degree 1: knots = params directly (knot_count = n)
            NurbsCurve result(dim, false, order, cv_count);
            for (int i = 0; i < n; i++) result.set_knot(i, params[i]);
            for (int i = 0; i < n; i++) result.set_cv(i, points[i]);
            return result;
        }
        // degree 2: Piegl approach with averaged knots
        cv_count = n;
        int kn_count = cv_count + order - 2;
        std::vector<double> kn(kn_count);
        for (int i = 0; i < order - 1; i++) kn[i] = params[0];
        for (int i = kn_count - order + 1; i < kn_count; i++) kn[i] = params[n - 1];
        for (int j = 1; j <= n - order; j++) {
            double sum = 0.0;
            for (int i = j; i < j + degree; i++) sum += params[i];
            kn[order - 2 + j] = sum / degree;
        }
        std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));
        for (int k = 0; k < n; k++) {
            int span = knot::find_span(order, cv_count, kn, params[k]);
            auto basis = knot::eval_basis(order, kn, span, params[k]);
            for (int j = 0; j < order && span + j < n; j++)
                A[k][span + j] = basis[j];
        }
        std::vector<double> cv(n * dim);
        for (int i = 0; i < n; i++)
            for (int d = 0; d < dim; d++) cv[i * dim + d] = points[i][d];
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++)
                if (std::fabs(A[row][col]) > std::fabs(A[pivot][col])) pivot = row;
            if (pivot != col) {
                std::swap(A[col], A[pivot]);
                for (int d = 0; d < dim; d++) std::swap(cv[col*dim+d], cv[pivot*dim+d]);
            }
            for (int row = col + 1; row < n; row++) {
                double factor = A[row][col] / A[col][col];
                for (int j = col; j < n; j++) A[row][j] -= factor * A[col][j];
                for (int d = 0; d < dim; d++) cv[row*dim+d] -= factor * cv[col*dim+d];
            }
        }
        for (int i = n - 1; i >= 0; i--)
            for (int d = 0; d < dim; d++) {
                double sum = cv[i*dim+d];
                for (int j = i + 1; j < n; j++) sum -= A[i][j] * cv[j*dim+d];
                cv[i*dim+d] = sum / A[i][i];
            }
        NurbsCurve result(dim, false, order, cv_count);
        for (int i = 0; i < kn_count; i++) result.set_knot(i, kn[i]);
        for (int i = 0; i < cv_count; i++)
            result.set_cv(i, Point(cv[i*3], cv[i*3+1], cv[i*3+2]));
        return result;
    }

    // Degree 3: TL approach (cv_count = n+2, Bessel tangents, tridiagonal)
    int cv_count = n + 2;
    auto knots = knot::build_interp_knots(params, degree);
    int kc = (int)knots.size();

    auto pdist = [](const Point& a, const Point& b) {
        double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    auto estimate_tangent = [&](int i0, int i1, int i2) -> Vector {
        double d01 = pdist(points[i0], points[i1]);
        double d21 = pdist(points[i2], points[i1]);
        if (d01 + d21 < 1e-300) return Vector(0,0,0);
        double s = d01 / (d01 + d21);
        double t = 1.0 - s;
        double denom = 2.0 * s * t;
        if (denom < 1e-16) {
            double dx = points[i1][0]-points[i0][0];
            double dy = points[i1][1]-points[i0][1];
            double dz = points[i1][2]-points[i0][2];
            double len = std::sqrt(dx*dx+dy*dy+dz*dz);
            return len > 0 ? Vector(dx/len, dy/len, dz/len) : Vector(0,0,0);
        }
        double cvx = (-t*t*points[i0][0] + points[i1][0] - s*s*points[i2][0]) / denom;
        double cvy = (-t*t*points[i0][1] + points[i1][1] - s*s*points[i2][1]) / denom;
        double cvz = (-t*t*points[i0][2] + points[i1][2] - s*s*points[i2][2]) / denom;
        double dx = cvx - points[i0][0], dy = cvy - points[i0][1], dz = cvz - points[i0][2];
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        return len > 0 ? Vector(dx/len, dy/len, dz/len) : Vector(0,0,0);
    };

    Vector tan_start, tan_end;
    if (n >= 3) {
        tan_start = estimate_tangent(0, 1, 2);
        Vector end_raw = estimate_tangent(n-1, n-2, n-3);
        tan_end = Vector(-end_raw[0], -end_raw[1], -end_raw[2]);
    } else {
        double dx = points[1][0]-points[0][0], dy = points[1][1]-points[0][1], dz = points[1][2]-points[0][2];
        double len = std::sqrt(dx*dx+dy*dy+dz*dz);
        if (len > 0) { tan_start = Vector(dx/len, dy/len, dz/len); tan_end = tan_start; }
    }

    double d_start = pdist(points[0], points[1]);
    double d_end = pdist(points[n-1], points[n-2]);

    std::vector<double> cv(cv_count * dim);
    for (int d = 0; d < dim; d++) cv[d] = points[0][d];
    double s0 = d_start / 3.0;
    for (int d = 0; d < dim; d++) cv[dim + d] = points[0][d] + s0 * tan_start[d];
    for (int i = 1; i <= n-2; i++)
        for (int d = 0; d < dim; d++) cv[(i+1) * dim + d] = points[i][d];
    double s1 = -d_end / 3.0;
    for (int d = 0; d < dim; d++) cv[n * dim + d] = points[n-1][d] + s1 * tan_end[d];
    for (int d = 0; d < dim; d++) cv[(n+1) * dim + d] = points[n-1][d];

    int sys_n = n;
    std::vector<double> lower(sys_n, 0.0), diag(sys_n, 0.0), upper(sys_n, 0.0);
    std::vector<double> rhs(sys_n * dim);

    diag[0] = 1.0;
    for (int d = 0; d < dim; d++) rhs[d] = cv[dim + d];

    for (int i = 1; i <= n-2; i++) {
        auto basis = knot::eval_basis(order, knots, i, params[i]);
        lower[i] = basis[0];
        diag[i] = basis[1];
        upper[i] = basis[2];
        for (int d = 0; d < dim; d++) rhs[i * dim + d] = points[i][d];
    }

    diag[n-1] = 1.0;
    for (int d = 0; d < dim; d++) rhs[(n-1) * dim + d] = cv[n * dim + d];

    std::vector<double> solution;
    knot::solve_tridiagonal(dim, sys_n, lower, diag, upper, rhs, solution);

    for (int i = 0; i < sys_n; i++)
        for (int d = 0; d < dim; d++) cv[(i+1) * dim + d] = solution[i * dim + d];

    NurbsCurve result(dim, false, order, cv_count);
    for (int i = 0; i < kc; i++) result.set_knot(i, knots[i]);
    for (int i = 0; i < cv_count; i++)
        result.set_cv(i, Point(cv[i*3], cv[i*3+1], cv[i*3+2]));
    return result;
}

// Build a loft surface through section curves using explicit cross-section params
NurbsSurface create_loft_with_params(
    const std::vector<NurbsCurve>& input_curves,
    const std::vector<double>& cross_params)
{
    std::vector<NurbsCurve> curves = input_curves;
    make_curves_compatible(curves);

    int n_sections = (int)curves.size();
    int cv_u = curves[0].cv_count();
    bool is_rat = curves[0].is_rational();

    // For each CV column, interpolate through sections using cross_params
    std::vector<NurbsCurve> col_curves;
    for (int col = 0; col < cv_u; col++) {
        std::vector<Point> col_pts;
        for (int k = 0; k < n_sections; k++) {
            if (is_rat) {
                double x, y, z, w;
                curves[k].get_cv_4d(col, x, y, z, w);
                col_pts.push_back(Point(x/w, y/w, z/w));
            } else {
                col_pts.push_back(curves[k].get_cv(col));
            }
        }
        col_curves.push_back(cubic_interpolation_curve(col_pts, cross_params));
    }

    int cv_v = col_curves[0].cv_count();
    int order_v = col_curves[0].order();

    NurbsSurface srf;
    srf.create_raw(3, is_rat, curves[0].order(), order_v, cv_u, cv_v);
    for (int i = 0; i < srf.knot_count(0); i++)
        srf.set_knot(0, i, curves[0].knot(i));
    for (int i = 0; i < srf.knot_count(1); i++)
        srf.set_knot(1, i, col_curves[0].knot(i));
    for (int i = 0; i < cv_u; i++)
        for (int j = 0; j < cv_v; j++) {
            if (is_rat) {
                // TODO: handle rational CVs properly
                Point p = col_curves[i].get_cv(j);
                srf.set_cv(i, j, p);
            } else {
                srf.set_cv(i, j, col_curves[i].get_cv(j));
            }
        }
    return srf;
}

NurbsSurface create_bicubic_grid_surface(
    const std::vector<std::vector<Point>>& grid,
    const std::vector<double>& u_params,
    const std::vector<double>& v_params)
{
    int n_rows = (int)grid.size();

    std::vector<NurbsCurve> row_curves;
    for (int i = 0; i < n_rows; i++)
        row_curves.push_back(cubic_interpolation_curve(grid[i], u_params));
    make_curves_compatible(row_curves);

    int cv_u = row_curves[0].cv_count();
    int order_u = row_curves[0].order();

    std::vector<NurbsCurve> col_curves;
    for (int col = 0; col < cv_u; col++) {
        std::vector<Point> col_pts;
        for (int row = 0; row < n_rows; row++)
            col_pts.push_back(row_curves[row].get_cv(col));
        col_curves.push_back(cubic_interpolation_curve(col_pts, v_params));
    }

    int cv_v = col_curves[0].cv_count();
    int order_v = col_curves[0].order();

    NurbsSurface srf;
    srf.create_raw(3, false, order_u, order_v, cv_u, cv_v);
    for (int i = 0; i < srf.knot_count(0); i++)
        srf.set_knot(0, i, row_curves[0].knot(i));
    for (int i = 0; i < srf.knot_count(1); i++)
        srf.set_knot(1, i, col_curves[0].knot(i));
    for (int i = 0; i < cv_u; i++)
        for (int j = 0; j < cv_v; j++)
            srf.set_cv(i, j, col_curves[i].get_cv(j));
    return srf;
}

void make_surfaces_knot_compatible_1d(NurbsSurface& a, NurbsSurface& b, NurbsSurface& c) {
    int dir = 0;
    int max_deg = std::max({a.degree(dir), b.degree(dir), c.degree(dir)});
    if (a.degree(dir) < max_deg) a.increase_degree(dir, max_deg);
    if (b.degree(dir) < max_deg) b.increase_degree(dir, max_deg);
    if (c.degree(dir) < max_deg) c.increase_degree(dir, max_deg);

    a.set_domain(dir, 0.0, 1.0);
    b.set_domain(dir, 0.0, 1.0);
    c.set_domain(dir, 0.0, 1.0);

    auto ka = a.get_knots(dir), kb = b.get_knots(dir), kc = c.get_knots(dir);
    auto unified = merge_knot_vectors(merge_knot_vectors(ka, kb), kc);

    auto insert_missing = [](NurbsSurface& srf, int d, const std::vector<double>& uni) {
        std::vector<double> orig = srf.get_knots(d);
        size_t ci = 0;
        for (size_t ui = 0; ui < uni.size(); ui++) {
            if (ci < orig.size() && std::abs(orig[ci] - uni[ui]) < 1e-10) ci++;
            else srf.insert_knot(d, uni[ui], 1);
        }
    };
    insert_missing(a, dir, unified);
    insert_missing(b, dir, unified);
    insert_missing(c, dir, unified);
}

void make_surfaces_knot_compatible(NurbsSurface& a, NurbsSurface& b, NurbsSurface& c) {
    for (int dir = 0; dir < 2; dir++) {
        int max_deg = std::max({a.degree(dir), b.degree(dir), c.degree(dir)});
        if (a.degree(dir) < max_deg) a.increase_degree(dir, max_deg);
        if (b.degree(dir) < max_deg) b.increase_degree(dir, max_deg);
        if (c.degree(dir) < max_deg) c.increase_degree(dir, max_deg);

        a.set_domain(dir, 0.0, 1.0);
        b.set_domain(dir, 0.0, 1.0);
        c.set_domain(dir, 0.0, 1.0);

        auto ka = a.get_knots(dir);
        auto kb = b.get_knots(dir);
        auto kc = c.get_knots(dir);
        auto unified = merge_knot_vectors(merge_knot_vectors(ka, kb), kc);

        auto insert_missing = [](NurbsSurface& srf, int d, const std::vector<double>& uni) {
            std::vector<double> orig = srf.get_knots(d);
            size_t ci = 0;
            for (size_t ui = 0; ui < uni.size(); ui++) {
                if (ci < orig.size() && std::abs(orig[ci] - uni[ui]) < 1e-10) {
                    ci++;
                } else {
                    srf.insert_knot(d, uni[ui], 1);
                }
            }
        };
        insert_missing(a, dir, unified);
        insert_missing(b, dir, unified);
        insert_missing(c, dir, unified);
    }

    bool any_rat = a.is_rational() || b.is_rational() || c.is_rational();
    if (any_rat) {
        a.make_rational();
        b.make_rational();
        c.make_rational();
    }
}

} // anonymous namespace

NurbsSurface Primitives::create_extrusion(const NurbsCurve& curve, const Vector& direction) {
    if (!curve.is_valid()) return NurbsSurface();
    NurbsCurve translated = curve;
    Xform t = Xform::translation(direction[0], direction[1], direction[2]);
    translated.transform(t);
    return create_ruled(curve, translated);
}

NurbsSurface Primitives::create_ruled(const NurbsCurve& curveA, const NurbsCurve& curveB) {
    NurbsSurface surface;
    if (!curveA.is_valid() || !curveB.is_valid()) return surface;

    NurbsCurve cA = curveA;
    NurbsCurve cB = curveB;

    cA.set_domain(0.0, 1.0);
    cB.set_domain(0.0, 1.0);

    if (cA.degree() < cB.degree()) cA.increase_degree(cB.degree());
    else if (cB.degree() < cA.degree()) cB.increase_degree(cA.degree());

    if (cA.is_rational() || cB.is_rational()) {
        cA.make_rational();
        cB.make_rational();
    }

    {
        std::vector<double> knotsA = cA.get_knots();
        std::vector<double> knotsB = cB.get_knots();
        const double tol = 1e-10;

        for (double k : knotsB) {
            bool found = false;
            for (double ka : knotsA) {
                if (std::abs(ka - k) < tol) { found = true; break; }
            }
            if (!found) cA.insert_knot(k, 1);
        }

        knotsA = cA.get_knots();
        for (double k : knotsA) {
            bool found = false;
            for (double kb : knotsB) {
                if (std::abs(kb - k) < tol) { found = true; break; }
            }
            if (!found) cB.insert_knot(k, 1);
        }
    }

    int order_u = cA.order();
    int cv_count_u = cA.cv_count();
    int order_v = 2;
    int cv_count_v = 2;
    bool is_rat = cA.is_rational();

    surface.create_raw(3, is_rat, order_u, order_v, cv_count_u, cv_count_v);

    for (int i = 0; i < cA.knot_count(); i++) {
        surface.set_knot(0, i, cA.knot(i));
    }

    surface.set_knot(1, 0, 0.0);
    surface.set_knot(1, 1, 1.0);

    if (is_rat) {
        for (int i = 0; i < cv_count_u; i++) {
            double ax, ay, az, aw;
            cA.get_cv_4d(i, ax, ay, az, aw);
            surface.set_cv_4d(i, 0, ax, ay, az, aw);

            double bx, by, bz, bw;
            cB.get_cv_4d(i, bx, by, bz, bw);
            surface.set_cv_4d(i, 1, bx, by, bz, bw);
        }
    } else {
        for (int i = 0; i < cv_count_u; i++) {
            surface.set_cv(i, 0, cA.get_cv(i));
            surface.set_cv(i, 1, cB.get_cv(i));
        }
    }

    return surface;
}

NurbsSurface Primitives::create_planar(const NurbsCurve& boundary) {
    NurbsSurface surface;

    std::vector<Point> all_pts;
    for (int i = 0; i < boundary.cv_count(); i++)
        all_pts.push_back(boundary.get_cv(i));

    std::vector<Point> unique_pts = all_pts;
    if (unique_pts.size() >= 2) {
        auto& f = unique_pts.front();
        auto& l = unique_pts.back();
        double d2 = (f[0]-l[0])*(f[0]-l[0]) + (f[1]-l[1])*(f[1]-l[1]) + (f[2]-l[2])*(f[2]-l[2]);
        if (d2 < 1e-20) unique_pts.pop_back();
    }
    if (unique_pts.size() < 3) return surface;

    auto make_bilinear = [&](const Point& orig, const Vector& xax, const Vector& yax,
                             double min_u, double max_u, double min_v, double max_v) {
        surface.create_raw(3, false, 2, 2, 2, 2);
        surface.set_knot(0, 0, 0.0); surface.set_knot(0, 1, 1.0);
        surface.set_knot(1, 0, 0.0); surface.set_knot(1, 1, 1.0);
        auto pt = [&](double u, double v) -> Point {
            return Point(orig[0] + u*xax[0] + v*yax[0],
                         orig[1] + u*xax[1] + v*yax[1],
                         orig[2] + u*xax[2] + v*yax[2]);
        };
        surface.set_cv(0, 0, pt(min_u, min_v));
        surface.set_cv(1, 0, pt(max_u, min_v));
        surface.set_cv(1, 1, pt(max_u, max_v));
        surface.set_cv(0, 1, pt(min_u, max_v));
    };

    auto longest_edge_dir = [&](const std::vector<Point>& pts) -> Vector {
        double best_d2 = 0.0;
        int best_i = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            size_t j = (i + 1) % pts.size();
            double dx = pts[j][0]-pts[i][0], dy = pts[j][1]-pts[i][1], dz = pts[j][2]-pts[i][2];
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 > best_d2) { best_d2 = d2; best_i = (int)i; }
        }
        int j = (best_i + 1) % (int)pts.size();
        double dx = pts[j][0]-pts[best_i][0];
        double dy = pts[j][1]-pts[best_i][1];
        double dz = pts[j][2]-pts[best_i][2];
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        return Vector(dx/len, dy/len, dz/len);
    };

    if (unique_pts.size() == 3 && boundary.degree() <= 1) {
        surface.create_raw(3, false, 2, 2, 2, 2);
        surface.set_knot(0, 0, 0.0); surface.set_knot(0, 1, 1.0);
        surface.set_knot(1, 0, 0.0); surface.set_knot(1, 1, 1.0);
        surface.set_cv(0, 0, unique_pts[0]);
        surface.set_cv(1, 0, unique_pts[1]);
        surface.set_cv(1, 1, unique_pts[2]);
        surface.set_cv(0, 1, unique_pts[0]);
        return surface;
    }

    if (unique_pts.size() == 4 && boundary.degree() <= 1) {
        surface.create_raw(3, false, 2, 2, 2, 2);
        surface.set_knot(0, 0, 0.0); surface.set_knot(0, 1, 1.0);
        surface.set_knot(1, 0, 0.0); surface.set_knot(1, 1, 1.0);
        surface.set_cv(0, 0, unique_pts[0]);
        surface.set_cv(1, 0, unique_pts[1]);
        surface.set_cv(1, 1, unique_pts[2]);
        surface.set_cv(0, 1, unique_pts[3]);
        return surface;
    }

    if (boundary.degree() <= 1) {
        Vector e1(unique_pts[1][0]-unique_pts[0][0], unique_pts[1][1]-unique_pts[0][1], unique_pts[1][2]-unique_pts[0][2]);
        Vector e2(unique_pts[2][0]-unique_pts[0][0], unique_pts[2][1]-unique_pts[0][1], unique_pts[2][2]-unique_pts[0][2]);
        Vector normal = e1.cross(e2);
        double nlen = normal.magnitude();
        if (nlen < 1e-14) return surface;
        normal = normal * (1.0 / nlen);

        Vector xax = longest_edge_dir(unique_pts);
        Vector yax = normal.cross(xax);
        double ylen = yax.magnitude();
        if (ylen < 1e-14) return surface;
        yax = yax * (1.0 / ylen);

        Point orig = unique_pts[0];
        double min_u = 0, max_u = 0, min_v = 0, max_v = 0;
        for (const auto& pt : unique_pts) {
            double dx = pt[0]-orig[0], dy = pt[1]-orig[1], dz = pt[2]-orig[2];
            double u = dx*xax[0] + dy*xax[1] + dz*xax[2];
            double v = dx*yax[0] + dy*yax[1] + dz*yax[2];
            if (u < min_u) min_u = u; if (u > max_u) max_u = u;
            if (v < min_v) min_v = v; if (v > max_v) max_v = v;
        }

        double pad = std::max(max_u - min_u, max_v - min_v) * 0.05;
        if (pad < 1e-6) pad = 1.0;
        min_u -= pad; max_u += pad;
        min_v -= pad; max_v += pad;

        make_bilinear(orig, xax, yax, min_u, max_u, min_v, max_v);
        return surface;
    }

    int n_samples = std::max(20, boundary.cv_count() * 4);
    auto [sample_pts, sample_params] = boundary.divide_by_count(n_samples);
    Plane plane = Plane::from_points_pca(sample_pts);
    if (plane.z_axis().magnitude() < 1e-10) return surface;

    Vector xax = plane.x_axis();
    Vector yax = plane.y_axis();
    Point orig = plane.origin();

    double min_u = 1e30, max_u = -1e30, min_v = 1e30, max_v = -1e30;
    for (const auto& pt : sample_pts) {
        double dx = pt[0]-orig[0], dy = pt[1]-orig[1], dz = pt[2]-orig[2];
        double u = dx*xax[0] + dy*xax[1] + dz*xax[2];
        double v = dx*yax[0] + dy*yax[1] + dz*yax[2];
        if (u < min_u) min_u = u; if (u > max_u) max_u = u;
        if (v < min_v) min_v = v; if (v > max_v) max_v = v;
    }

    double pad = std::max(max_u - min_u, max_v - min_v) * 0.05;
    if (pad < 1e-6) pad = 1.0;
    min_u -= pad; max_u += pad;
    min_v -= pad; max_v += pad;

    make_bilinear(orig, xax, yax, min_u, max_u, min_v, max_v);
    return surface;
}

NurbsSurface Primitives::create_loft(const std::vector<NurbsCurve>& input_curves, int degree_v) {
    NurbsSurface surface;
    if (input_curves.size() < 2) return surface;

    for (auto& c : input_curves) {
        if (!c.is_valid()) return surface;
    }

    std::vector<NurbsCurve> curves = input_curves;
    make_curves_compatible(curves);
    make_curves_compatible(curves);

    int n_sections = static_cast<int>(curves.size());
    int cv_count_u = curves[0].cv_count();
    int order_u = curves[0].order();
    bool is_rat = curves[0].is_rational();

    if (degree_v >= n_sections) degree_v = n_sections - 1;
    if (degree_v < 1) degree_v = 1;
    int order_v = degree_v + 1;

    std::vector<double> v_params(n_sections, 0.0);
    for (int k = 1; k < n_sections; k++) {
        Point pk_prev = curves[k - 1].point_at_middle();
        Point pk_curr = curves[k].point_at_middle();
        double dx = pk_curr[0] - pk_prev[0];
        double dy = pk_curr[1] - pk_prev[1];
        double dz = pk_curr[2] - pk_prev[2];
        v_params[k] = v_params[k - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    double total_len = v_params.back();
    if (total_len > 1e-14) {
        for (int k = 0; k < n_sections; k++) v_params[k] /= total_len;
    } else {
        for (int k = 0; k < n_sections; k++) v_params[k] = static_cast<double>(k) / (n_sections - 1);
    }

    int cv_count_v = n_sections;
    int knot_count_v = order_v + cv_count_v - 2;
    std::vector<double> knots_v(knot_count_v);

    if (degree_v >= n_sections - 1) {
        int d = degree_v;
        for (int i = 0; i < d; i++) knots_v[i] = 0.0;
        for (int i = d; i < knot_count_v; i++) knots_v[i] = 1.0;
    } else {
        for (int i = 0; i < order_v - 1; i++) knots_v[i] = v_params[0];
        for (int j = 1; j <= n_sections - order_v; j++) {
            double sum = 0.0;
            for (int i = j; i < j + degree_v; i++) sum += v_params[i];
            knots_v[order_v - 2 + j] = sum / degree_v;
        }
        for (int i = knot_count_v - order_v + 1; i < knot_count_v; i++) knots_v[i] = v_params[n_sections - 1];
    }

    surface.create_raw(3, is_rat, order_u, order_v, cv_count_u, cv_count_v);

    for (int i = 0; i < surface.knot_count(0); i++) {
        surface.set_knot(0, i, curves[0].knot(i));
    }

    for (int i = 0; i < static_cast<int>(knots_v.size()) && i < surface.knot_count(1); i++) {
        surface.set_knot(1, i, knots_v[i]);
    }

    {
        int n = n_sections;
        std::vector<std::vector<double>> N_matrix(n, std::vector<double>(n, 0.0));

        NurbsCurve temp_crv(1, false, order_v, cv_count_v);
        for (int i = 0; i < static_cast<int>(knots_v.size()); i++) {
            temp_crv.set_knot(i, knots_v[i]);
        }

        for (int k = 0; k < n; k++) {
            double t = v_params[k];
            auto [t0, t1] = temp_crv.domain();
            if (t < t0) t = t0;
            if (t > t1) t = t1;

            int span = knot::find_span(order_v, cv_count_v, knots_v, t);
            int d = order_v - 1;
            int knot_base = span + d;

            if (knots_v[knot_base - 1] == knots_v[knot_base]) {
                if (t <= knots_v[knot_base]) {
                    N_matrix[k][span] = 1.0;
                } else {
                    N_matrix[k][span + order_v - 1] = 1.0;
                }
                continue;
            }

            std::vector<double> Nvals(order_v * order_v, 0.0);
            Nvals[order_v * order_v - 1] = 1.0;
            std::vector<double> left(d), right(d);
            int N_idx = order_v * order_v - 1;
            int k_right = knot_base;
            int k_left = knot_base - 1;

            for (int j = 0; j < d; j++) {
                int N0_idx = N_idx;
                N_idx -= (order_v + 1);
                left[j] = t - knots_v[k_left];
                right[j] = knots_v[k_right] - t;
                k_left--;
                k_right++;

                double x = 0.0;
                for (int r = 0; r <= j; r++) {
                    double a0 = left[j - r];
                    double a1 = right[r];
                    double denom = a0 + a1;
                    double y = (denom != 0.0) ? Nvals[N0_idx + r] / denom : 0.0;
                    Nvals[N_idx + r] = x + a1 * y;
                    x = a0 * y;
                }
                Nvals[N_idx + j + 1] = x;
            }

            for (int j = 0; j < order_v; j++) {
                int col = span + j;
                if (col >= 0 && col < n) {
                    N_matrix[k][col] = Nvals[j];
                }
            }
        }

        int dim = is_rat ? 4 : 3;
        for (int i = 0; i < cv_count_u; i++) {
            std::vector<std::vector<double>> rhs(n, std::vector<double>(dim, 0.0));
            for (int k = 0; k < n; k++) {
                if (is_rat) {
                    double x, y, z, w;
                    curves[k].get_cv_4d(i, x, y, z, w);
                    rhs[k] = {x, y, z, w};
                } else {
                    Point p = curves[k].get_cv(i);
                    rhs[k] = {p[0], p[1], p[2]};
                }
            }

            std::vector<std::vector<double>> A = N_matrix;
            std::vector<std::vector<double>> b = rhs;

            for (int col = 0; col < n; col++) {
                int max_row = col;
                double max_val = std::abs(A[col][col]);
                for (int row = col + 1; row < n; row++) {
                    if (std::abs(A[row][col]) > max_val) {
                        max_val = std::abs(A[row][col]);
                        max_row = row;
                    }
                }
                if (max_val < 1e-14) continue;

                std::swap(A[col], A[max_row]);
                std::swap(b[col], b[max_row]);

                for (int row = col + 1; row < n; row++) {
                    double factor = A[row][col] / A[col][col];
                    for (int c = col; c < n; c++) {
                        A[row][c] -= factor * A[col][c];
                    }
                    for (int d2 = 0; d2 < dim; d2++) {
                        b[row][d2] -= factor * b[col][d2];
                    }
                }
            }

            std::vector<std::vector<double>> Q(n, std::vector<double>(dim, 0.0));
            for (int row = n - 1; row >= 0; row--) {
                for (int d2 = 0; d2 < dim; d2++) {
                    Q[row][d2] = b[row][d2];
                    for (int c = row + 1; c < n; c++) {
                        Q[row][d2] -= A[row][c] * Q[c][d2];
                    }
                    if (std::abs(A[row][row]) > 1e-14) {
                        Q[row][d2] /= A[row][row];
                    }
                }
            }

            for (int j = 0; j < n; j++) {
                if (is_rat) {
                    surface.set_cv_4d(i, j, Q[j][0], Q[j][1], Q[j][2], Q[j][3]);
                } else {
                    surface.set_cv(i, j, Point(Q[j][0], Q[j][1], Q[j][2]));
                }
            }
        }
    }

    return surface;
}

NurbsSurface Primitives::create_revolve(const NurbsCurve& profile, const Point& axis_origin,
                                            const Vector& axis_direction, double angle) {
    NurbsSurface surface;
    if (!profile.is_valid()) return surface;

    double ax_len = axis_direction.magnitude();
    if (ax_len < 1e-14) return surface;
    Vector axis_dir = axis_direction / ax_len;

    double PI = Tolerance::PI;
    if (angle < 0) angle = -angle;
    if (angle > 2.0 * PI) angle = 2.0 * PI;
    if (angle < 1e-14) return surface;

    int n_arcs;
    if (angle <= PI / 2.0 + 1e-10) n_arcs = 1;
    else if (angle <= PI + 1e-10) n_arcs = 2;
    else if (angle <= 3.0 * PI / 2.0 + 1e-10) n_arcs = 3;
    else n_arcs = 4;

    double d_theta = angle / n_arcs;
    double w_mid = std::cos(d_theta / 2.0);
    int n_u = 2 * n_arcs + 1;

    std::vector<double> knots_u;
    knots_u.clear();
    int knot_count_u = n_u + 1;
    knots_u.resize(knot_count_u);
    knots_u[0] = 0.0;
    knots_u[1] = 0.0;
    for (int i = 1; i <= n_arcs; i++) {
        double kv = i * d_theta;
        knots_u[2 * i] = kv;
        knots_u[2 * i + 1] = kv;
    }
    knots_u[knot_count_u - 1] = angle;
    knots_u[knot_count_u - 2] = angle;

    int cv_count_v = profile.cv_count();
    int order_v = profile.order();
    bool profile_rational = profile.is_rational();

    surface.create_raw(3, true, 3, order_v, n_u, cv_count_v);

    for (int i = 0; i < knot_count_u && i < surface.knot_count(0); i++) {
        surface.set_knot(0, i, knots_u[i]);
    }

    for (int i = 0; i < profile.knot_count() && i < surface.knot_count(1); i++) {
        surface.set_knot(1, i, profile.knot(i));
    }

    std::vector<double> u_angles(n_u);
    std::vector<double> u_weights(n_u);
    for (int i = 0; i < n_u; i++) {
        if (i % 2 == 0) {
            u_angles[i] = (i / 2) * d_theta;
            u_weights[i] = 1.0;
        } else {
            u_angles[i] = (i / 2) * d_theta + d_theta / 2.0;
            u_weights[i] = w_mid;
        }
    }

    for (int j = 0; j < cv_count_v; j++) {
        Point P_j = profile.get_cv(j);
        double profile_w = profile_rational ? profile.weight(j) : 1.0;

        double dx = P_j[0] - axis_origin[0];
        double dy = P_j[1] - axis_origin[1];
        double dz = P_j[2] - axis_origin[2];
        double proj = dx * axis_dir[0] + dy * axis_dir[1] + dz * axis_dir[2];
        Point O_j(axis_origin[0] + proj * axis_dir[0],
                  axis_origin[1] + proj * axis_dir[1],
                  axis_origin[2] + proj * axis_dir[2]);

        double rx = P_j[0] - O_j[0];
        double ry = P_j[1] - O_j[1];
        double rz = P_j[2] - O_j[2];
        double r_j = std::sqrt(rx * rx + ry * ry + rz * rz);

        if (r_j < 1e-14) {
            for (int i = 0; i < n_u; i++) {
                double combined_w = u_weights[i] * profile_w;
                surface.set_cv(i, j, O_j);
                surface.set_weight(i, j, combined_w);
            }
        } else {
            Vector x_local(rx / r_j, ry / r_j, rz / r_j);
            Vector y_local = axis_dir.cross(x_local);
            double y_len = y_local.magnitude();
            if (y_len > 1e-14) {
                y_local = y_local / y_len;
            }

            for (int i = 0; i < n_u; i++) {
                double theta = u_angles[i];
                double cos_t = std::cos(theta);
                double sin_t = std::sin(theta);

                double effective_r = r_j;
                if (i % 2 == 1) {
                    effective_r = r_j / w_mid;
                }

                double px = O_j[0] + effective_r * (cos_t * x_local[0] + sin_t * y_local[0]);
                double py = O_j[1] + effective_r * (cos_t * x_local[1] + sin_t * y_local[1]);
                double pz = O_j[2] + effective_r * (cos_t * x_local[2] + sin_t * y_local[2]);

                double combined_w = u_weights[i] * profile_w;
                surface.set_cv_4d(i, j, px * combined_w, py * combined_w, pz * combined_w, combined_w);
            }
        }
    }

    return surface;
}

NurbsSurface Primitives::create_sweep1(const NurbsCurve& rail, const NurbsCurve& profile) {
    NurbsSurface surface;
    if (!rail.is_valid() || !profile.is_valid()) return surface;

    NurbsCurve working_profile = profile;

    int N = std::min(std::max(rail.span_count() * 2 + 1, 5), 20);
    std::vector<Plane> frames = rail.get_perpendicular_planes(N);
    if (frames.empty()) return surface;

    double cx = 0, cy = 0, cz = 0;
    int nc = working_profile.cv_count();
    for (int k = 0; k < nc; k++) {
        Point cv = working_profile.get_cv(k);
        cx += cv[0]; cy += cv[1]; cz += cv[2];
    }
    cx /= nc; cy /= nc; cz /= nc;

    auto dom = working_profile.domain();
    double t0 = dom.first, t1 = dom.second;
    Point pa = working_profile.point_at(t0);
    Point pb = working_profile.point_at(t0 + (t1 - t0) / 3.0);
    Point pc = working_profile.point_at(t0 + 2.0 * (t1 - t0) / 3.0);
    Vector v1(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
    Vector v2(pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]);
    Vector prof_normal = v1.cross(v2);
    double nlen = prof_normal.magnitude();
    if (nlen < 1e-14) prof_normal = Vector(1, 0, 0);
    else prof_normal = prof_normal / nlen;

    Vector prof_x(pa[0] - cx, pa[1] - cy, pa[2] - cz);
    double pxlen = prof_x.magnitude();
    if (pxlen < 1e-14) prof_x = Vector(0, 1, 0);
    else prof_x = prof_x / pxlen;
    double dot = prof_x[0] * prof_normal[0] + prof_x[1] * prof_normal[1] + prof_x[2] * prof_normal[2];
    prof_x = Vector(prof_x[0] - dot * prof_normal[0], prof_x[1] - dot * prof_normal[1], prof_x[2] - dot * prof_normal[2]);
    pxlen = prof_x.magnitude();
    if (pxlen < 1e-14) prof_x = Vector(0, 1, 0);
    else prof_x = prof_x / pxlen;
    Vector prof_y = prof_normal.cross(prof_x);
    double pylen = prof_y.magnitude();
    if (pylen > 1e-14) prof_y = prof_y / pylen;

    std::vector<NurbsCurve> positioned_profiles;
    positioned_profiles.reserve(frames.size());

    for (size_t i = 0; i < frames.size(); i++) {
        NurbsCurve prof_copy = working_profile;
        Plane& frame = frames[i];
        Point fo = frame.origin();
        Vector fx = frame.x_axis();
        Vector fy = frame.y_axis();
        Vector fz = frame.z_axis();

        Xform t1x = Xform::translation(-cx, -cy, -cz);

        Xform rot = Xform::identity();
        rot.m[0]  = fx[0]*prof_x[0] + fy[0]*prof_y[0] + fz[0]*prof_normal[0];
        rot.m[1]  = fx[1]*prof_x[0] + fy[1]*prof_y[0] + fz[1]*prof_normal[0];
        rot.m[2]  = fx[2]*prof_x[0] + fy[2]*prof_y[0] + fz[2]*prof_normal[0];
        rot.m[4]  = fx[0]*prof_x[1] + fy[0]*prof_y[1] + fz[0]*prof_normal[1];
        rot.m[5]  = fx[1]*prof_x[1] + fy[1]*prof_y[1] + fz[1]*prof_normal[1];
        rot.m[6]  = fx[2]*prof_x[1] + fy[2]*prof_y[1] + fz[2]*prof_normal[1];
        rot.m[8]  = fx[0]*prof_x[2] + fy[0]*prof_y[2] + fz[0]*prof_normal[2];
        rot.m[9]  = fx[1]*prof_x[2] + fy[1]*prof_y[2] + fz[1]*prof_normal[2];
        rot.m[10] = fx[2]*prof_x[2] + fy[2]*prof_y[2] + fz[2]*prof_normal[2];
        rot.m[12] = fo[0]; rot.m[13] = fo[1]; rot.m[14] = fo[2];

        prof_copy.transform(t1x);
        prof_copy.transform(rot);
        positioned_profiles.push_back(prof_copy);
    }

    int loft_degree = std::min(3, static_cast<int>(positioned_profiles.size()) - 1);
    surface = create_loft(positioned_profiles, loft_degree);
    return surface;
}

NurbsSurface Primitives::create_sweep2(const NurbsCurve& rail1, const NurbsCurve& rail2,
                                           const std::vector<NurbsCurve>& shapes) {
    NurbsSurface surface;
    if (!rail1.is_valid() || !rail2.is_valid() || shapes.empty()) return surface;
    for (auto& s : shapes) { if (!s.is_valid()) return surface; }

    std::vector<NurbsCurve> compat_shapes = shapes;
    if (compat_shapes.size() >= 2) make_curves_compatible(compat_shapes);

    int n_shapes = static_cast<int>(compat_shapes.size());

    std::vector<double> shape_params(n_shapes);
    for (int k = 0; k < n_shapes; k++)
        shape_params[k] = (n_shapes == 1) ? 0.0 : (double)k / (n_shapes - 1);

    int N = std::min(std::max(std::max(rail1.span_count(), rail2.span_count()) * 2 + 1, 5), 20);

    std::vector<Point> pts1, pts2;
    std::vector<double> params1, params2;
    rail1.divide_by_count(N + 1, pts1, &params1, true);
    rail2.divide_by_count(N + 1, pts2, &params2, true);

    std::vector<Plane> frames1 = rail1.get_perpendicular_planes(N);
    if (frames1.empty()) return surface;

    struct ShapeInfo {
        Point start, end;
        double width;
        Vector dir, side, up;
    };
    std::vector<ShapeInfo> sinfo(n_shapes);
    for (int k = 0; k < n_shapes; k++) {
        auto& si = sinfo[k];
        si.start = compat_shapes[k].point_at_start();
        si.end = compat_shapes[k].point_at_end();
        Vector span(si.end[0]-si.start[0], si.end[1]-si.start[1], si.end[2]-si.start[2]);
        si.width = span.magnitude();
        if (si.width < 1e-14) si.width = 1.0;
        si.dir = span / si.width;
        Vector up_try(0, 0, 1);
        si.side = si.dir.cross(up_try);
        if (si.side.magnitude() < 1e-10) {
            up_try = Vector(0, 1, 0);
            si.side = si.dir.cross(up_try);
        }
        si.side = si.side / si.side.magnitude();
        si.up = si.side.cross(si.dir);
        double ulen = si.up.magnitude();
        if (ulen > 1e-14) si.up = si.up / ulen;
    }

    std::vector<NurbsCurve> positioned_profiles;
    positioned_profiles.reserve(frames1.size());

    for (size_t i = 0; i < frames1.size() && i < pts1.size() && i < pts2.size(); i++) {
        double t = (frames1.size() <= 1) ? 0.0 : (double)i / (frames1.size() - 1);

        int j = 0;
        double s = 0.0;
        if (n_shapes == 1) {
            j = 0; s = 0.0;
        } else {
            for (int k = 0; k < n_shapes - 1; k++) {
                if (t <= shape_params[k + 1] + 1e-14) { j = k; break; }
                j = k;
            }
            double denom = shape_params[j + 1] - shape_params[j];
            s = (denom > 1e-14) ? (t - shape_params[j]) / denom : 0.0;
            s = std::max(0.0, std::min(1.0, s));
        }

        NurbsCurve interp_shape = compat_shapes[j];
        if (n_shapes > 1 && j + 1 < n_shapes) {
            int nc = compat_shapes[j].cv_count();
            for (int c = 0; c < nc; c++) {
                Point cv0 = compat_shapes[j].get_cv(c);
                Point cv1 = compat_shapes[j + 1].get_cv(c);
                Point lerped(cv0[0]*(1-s) + cv1[0]*s, cv0[1]*(1-s) + cv1[1]*s, cv0[2]*(1-s) + cv1[2]*s);
                interp_shape.set_cv(c, lerped);
            }
        }

        double shape_width = sinfo[j].width * (1-s) + ((n_shapes > 1 && j+1 < n_shapes) ? sinfo[j+1].width * s : 0.0);
        if (n_shapes == 1) shape_width = sinfo[0].width;
        Vector prof_dir(
            sinfo[j].dir[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].dir[0]*s : 0.0),
            sinfo[j].dir[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].dir[1]*s : 0.0),
            sinfo[j].dir[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].dir[2]*s : 0.0));
        double pdlen = prof_dir.magnitude();
        if (pdlen > 1e-14) prof_dir = prof_dir / pdlen;
        Vector prof_side(
            sinfo[j].side[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].side[0]*s : 0.0),
            sinfo[j].side[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].side[1]*s : 0.0),
            sinfo[j].side[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].side[2]*s : 0.0));
        double pslen = prof_side.magnitude();
        if (pslen > 1e-14) prof_side = prof_side / pslen;
        Vector prof_up(
            sinfo[j].up[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].up[0]*s : 0.0),
            sinfo[j].up[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].up[1]*s : 0.0),
            sinfo[j].up[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].up[2]*s : 0.0));
        double pulen = prof_up.magnitude();
        if (pulen > 1e-14) prof_up = prof_up / pulen;

        Point interp_start(
            sinfo[j].start[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].start[0]*s : 0.0),
            sinfo[j].start[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].start[1]*s : 0.0),
            sinfo[j].start[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].start[2]*s : 0.0));
        if (n_shapes == 1) interp_start = sinfo[0].start;

        Point p1 = pts1[i];
        Point p2 = pts2[i];
        double dx = p2[0] - p1[0], dy = p2[1] - p1[1], dz = p2[2] - p1[2];
        double rail_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        double scale_factor = (rail_dist > 1e-14 && shape_width > 1e-14) ? rail_dist / shape_width : 1.0;

        NurbsCurve prof_copy = interp_shape;
        Xform t1 = Xform::translation(-interp_start[0], -interp_start[1], -interp_start[2]);
        prof_copy.transform(t1);

        Xform sc = Xform::scale_xyz(scale_factor, scale_factor, scale_factor);
        prof_copy.transform(sc);

        Plane& frame = frames1[i];
        Vector tangent = frame.z_axis();
        Vector x_dir(dx, dy, dz);
        double x_len = x_dir.magnitude();
        if (x_len > 1e-14) x_dir = x_dir / x_len;
        else x_dir = frame.x_axis();
        Vector y_dir = tangent.cross(x_dir);
        double y_len = y_dir.magnitude();
        if (y_len > 1e-14) y_dir = y_dir / y_len;
        else y_dir = frame.y_axis();
        double dot_up = y_dir[0]*prof_up[0] + y_dir[1]*prof_up[1] + y_dir[2]*prof_up[2];
        if (dot_up < 0) y_dir = Vector(-y_dir[0], -y_dir[1], -y_dir[2]);
        tangent = x_dir.cross(y_dir);
        double tz = tangent.magnitude();
        if (tz > 1e-14) tangent = tangent / tz;

        Xform rot = Xform::identity();
        rot.m[0]  = tangent[0]*prof_side[0] + x_dir[0]*prof_dir[0] + y_dir[0]*prof_up[0];
        rot.m[1]  = tangent[1]*prof_side[0] + x_dir[1]*prof_dir[0] + y_dir[1]*prof_up[0];
        rot.m[2]  = tangent[2]*prof_side[0] + x_dir[2]*prof_dir[0] + y_dir[2]*prof_up[0];
        rot.m[4]  = tangent[0]*prof_side[1] + x_dir[0]*prof_dir[1] + y_dir[0]*prof_up[1];
        rot.m[5]  = tangent[1]*prof_side[1] + x_dir[1]*prof_dir[1] + y_dir[1]*prof_up[1];
        rot.m[6]  = tangent[2]*prof_side[1] + x_dir[2]*prof_dir[1] + y_dir[2]*prof_up[1];
        rot.m[8]  = tangent[0]*prof_side[2] + x_dir[0]*prof_dir[2] + y_dir[0]*prof_up[2];
        rot.m[9]  = tangent[1]*prof_side[2] + x_dir[1]*prof_dir[2] + y_dir[1]*prof_up[2];
        rot.m[10] = tangent[2]*prof_side[2] + x_dir[2]*prof_dir[2] + y_dir[2]*prof_up[2];
        rot.m[12] = p1[0]; rot.m[13] = p1[1]; rot.m[14] = p1[2];

        prof_copy.transform(rot);
        positioned_profiles.push_back(prof_copy);
    }

    int loft_degree = std::min(3, static_cast<int>(positioned_profiles.size()) - 1);
    surface = create_loft(positioned_profiles, loft_degree);
    return surface;
}

NurbsSurface Primitives::create_edge(
    const NurbsCurve& c0, const NurbsCurve& c1,
    const NurbsCurve& c2, const NurbsCurve& c3)
{
    NurbsSurface surface;

    if (!c0.is_valid() || !c1.is_valid() || !c2.is_valid() || !c3.is_valid())
        return surface;

    std::vector<NurbsCurve> input = {c0, c1, c2, c3};
    std::vector<NurbsCurve> loop;
    std::vector<bool> used(4, false);

    loop.push_back(input[0]);
    used[0] = true;
    const double tol = 1e-6;

    for (int step = 0; step < 3; step++) {
        Point tail = loop.back().point_at_end();
        bool found = false;
        for (int i = 0; i < 4; i++) {
            if (used[i]) continue;
            Point s = input[i].point_at_start();
            Point e = input[i].point_at_end();
            if (s.distance(tail) < tol) {
                loop.push_back(input[i]);
                used[i] = true;
                found = true;
                break;
            }
            if (e.distance(tail) < tol) {
                NurbsCurve rev = input[i];
                rev.reverse();
                loop.push_back(rev);
                used[i] = true;
                found = true;
                break;
            }
        }
        if (!found) return surface;
    }

    if (loop[3].point_at_end().distance(loop[0].point_at_start()) > tol)
        return surface;

    NurbsCurve south = loop[0];
    NurbsCurve east  = loop[1];
    NurbsCurve north = loop[2]; north.reverse();
    NurbsCurve west  = loop[3]; west.reverse();

    std::vector<NurbsCurve> v_pair = {south, north};
    make_curves_compatible(v_pair);
    south = v_pair[0];
    north = v_pair[1];

    std::vector<NurbsCurve> u_pair = {west, east};
    make_curves_compatible(u_pair);
    west = u_pair[0];
    east = u_pair[1];

    int order_v = south.order();
    int cv_count_v = south.cv_count();
    int order_u = west.order();
    int cv_count_u = west.cv_count();
    bool is_rat = south.is_rational() || west.is_rational();

    surface.create_raw(3, is_rat, order_u, order_v, cv_count_u, cv_count_v);

    for (int i = 0; i < surface.knot_count(0); i++)
        surface.set_knot(0, i, west.knot(i));

    for (int i = 0; i < surface.knot_count(1); i++)
        surface.set_knot(1, i, south.knot(i));

    std::vector<double> u_grev = west.get_greville_abcissae();
    std::vector<double> v_grev = south.get_greville_abcissae();

    auto [u0, u1] = west.domain();
    auto [v0, v1] = south.domain();
    for (auto& g : u_grev) g = (u1 > u0) ? (g - u0) / (u1 - u0) : 0.0;
    for (auto& g : v_grev) g = (v1 > v0) ? (g - v0) / (v1 - v0) : 0.0;

    Point C00 = south.get_cv(0);
    Point C01 = south.get_cv(cv_count_v - 1);
    Point C10 = north.get_cv(0);
    Point C11 = north.get_cv(cv_count_v - 1);

    for (int i = 0; i < cv_count_u; i++) {
        double ui = u_grev[i];
        Point wi = west.get_cv(i);
        Point ei = east.get_cv(i);
        for (int j = 0; j < cv_count_v; j++) {
            double vj = v_grev[j];
            Point sj = south.get_cv(j);
            Point nj = north.get_cv(j);

            double x = (1-ui)*sj[0] + ui*nj[0] + (1-vj)*wi[0] + vj*ei[0]
                      - (1-ui)*(1-vj)*C00[0] - (1-ui)*vj*C01[0]
                      - ui*(1-vj)*C10[0] - ui*vj*C11[0];
            double y = (1-ui)*sj[1] + ui*nj[1] + (1-vj)*wi[1] + vj*ei[1]
                      - (1-ui)*(1-vj)*C00[1] - (1-ui)*vj*C01[1]
                      - ui*(1-vj)*C10[1] - ui*vj*C11[1];
            double z = (1-ui)*sj[2] + ui*nj[2] + (1-vj)*wi[2] + vj*ei[2]
                      - (1-ui)*(1-vj)*C00[2] - (1-ui)*vj*C01[2]
                      - ui*(1-vj)*C10[2] - ui*vj*C11[2];

            surface.set_cv(i, j, Point(x, y, z));
        }
    }

    return surface;
}

NurbsSurface Primitives::create_network(
    const std::vector<NurbsCurve>& u_curves,
    const std::vector<NurbsCurve>& v_curves)
{
    NurbsSurface surface;
    int n_u = (int)u_curves.size();
    int n_v = (int)v_curves.size();
    if (n_u < 2 || n_v < 2) return surface;

    for (auto& c : u_curves) if (!c.is_valid()) return surface;
    for (auto& c : v_curves) if (!c.is_valid()) return surface;

    // Step 1: Find intersection parameters
    std::vector<std::vector<double>> u_par(n_u, std::vector<double>(n_v));
    std::vector<std::vector<double>> v_par(n_u, std::vector<double>(n_v));

    for (int i = 0; i < n_u; i++) {
        for (int j = 0; j < n_v; j++) {
            Point dummy;
            find_curve_intersection(u_curves[i], v_curves[j],
                                    u_par[i][j], v_par[i][j], dummy);
        }
    }

    // Step 2: Build point grid with averaging (decompiled GetGrid)
    std::vector<std::vector<Point>> grid(n_u, std::vector<Point>(n_v));
    for (int i = 0; i < n_u; i++) {
        for (int j = 0; j < n_v; j++) {
            Point pu = u_curves[i].point_at(u_par[i][j]);
            Point pv = v_curves[j].point_at(v_par[i][j]);
            grid[i][j] = Point(
                0.5 * (pu[0] + pv[0]),
                0.5 * (pu[1] + pv[1]),
                0.5 * (pu[2] + pv[2])
            );
        }
    }

    // Step 3: Compute parameterization using centripetal (sqrt chord-length) formula
    // from decompiled GetGrid: inc = ((sqrt(max_len) + sqrt(min_len)) / 2)^2
    auto pdist = [](const Point& a, const Point& b) {
        double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    // v_arc_params: positions of u-curves across the surface
    std::vector<double> v_arc_params(n_u, 0.0);
    for (int i = 1; i < n_u; i++) {
        double mx = 0, mn = 1e30;
        for (int j = 0; j < n_v; j++) {
            double d = pdist(grid[i-1][j], grid[i][j]);
            mx = std::max(mx, d); mn = std::min(mn, d);
        }
        double inc = (std::sqrt(mx) + std::sqrt(mn)) * 0.5;
        inc = inc * inc;
        if (inc < 0.0001) inc = 0.0001;
        v_arc_params[i] = v_arc_params[i-1] + inc;
    }
    if (v_arc_params.back() > 1e-14) for (auto& x : v_arc_params) x /= v_arc_params.back();

    // u_arc_params: positions of v-curves across the surface
    std::vector<double> u_arc_params(n_v, 0.0);
    for (int j = 1; j < n_v; j++) {
        double mx = 0, mn = 1e30;
        for (int i = 0; i < n_u; i++) {
            double d = pdist(grid[i][j-1], grid[i][j]);
            mx = std::max(mx, d); mn = std::min(mn, d);
        }
        double inc = (std::sqrt(mx) + std::sqrt(mn)) * 0.5;
        inc = inc * inc;
        if (inc < 0.0001) inc = 0.0001;
        u_arc_params[j] = u_arc_params[j-1] + inc;
    }
    if (u_arc_params.back() > 1e-14) for (auto& x : u_arc_params) x /= u_arc_params.back();

    // Step 4: Build grid surface T
    NurbsSurface T = create_bicubic_grid_surface(grid, u_arc_params, v_arc_params);

    // Step 5: Build L_u (loft through u-curves)
    std::vector<NurbsCurve> u_crvs(u_curves.begin(), u_curves.end());
    NurbsSurface L_u = create_loft_with_params(u_crvs, v_arc_params);

    // Step 6: Build L_v — reparametrize v-curves so their intersection params
    // map to v_arc_params, then loft and transpose
    auto reparametrize_curve = [](const NurbsCurve& curve,
                                   const std::vector<double>& old_params,
                                   const std::vector<double>& new_params,
                                   int samples_per_span = 10) -> NurbsCurve {
        int n_pts = (int)old_params.size();
        std::vector<Point> pts;
        std::vector<double> params;
        for (int i = 0; i < n_pts - 1; i++) {
            double t0 = old_params[i], t1 = old_params[i + 1];
            double a0 = new_params[i], a1 = new_params[i + 1];
            int nsamp = (i == n_pts - 2) ? samples_per_span + 1 : samples_per_span;
            for (int s = 0; s < nsamp; s++) {
                double frac = (double)s / samples_per_span;
                pts.push_back(curve.point_at(t0 + (t1 - t0) * frac));
                params.push_back(a0 + (a1 - a0) * frac);
            }
        }
        return cubic_interpolation_curve(pts, params);
    };
    std::vector<NurbsCurve> v_repar;
    for (int j = 0; j < n_v; j++) {
        std::vector<double> old_p, new_p;
        for (int i = 0; i < n_u; i++) {
            old_p.push_back(v_par[i][j]);
            new_p.push_back(v_arc_params[i]);
        }
        v_repar.push_back(reparametrize_curve(v_curves[j], old_p, new_p));
    }
    NurbsSurface L_v = create_loft_with_params(v_repar, u_arc_params);
    L_v.transpose();

    if (!T.is_valid() || !L_u.is_valid() || !L_v.is_valid()) return surface;

    // Step 7: Knot compatibility via transpose trick (decompiled CreateSurface)
    make_surfaces_knot_compatible_1d(T, L_u, L_v);
    T.transpose(); L_u.transpose(); L_v.transpose();
    make_surfaces_knot_compatible_1d(T, L_u, L_v);
    T.transpose(); L_u.transpose(); L_v.transpose();

    bool any_rat = T.is_rational() || L_u.is_rational() || L_v.is_rational();
    if (any_rat) {
        T.make_rational();
        L_u.make_rational();
        L_v.make_rational();
    }

    // Step 8: Gordon formula: S_cv = L_u_cv + L_v_cv - T_cv
    surface = T;
    for (int i = 0; i < surface.cv_count(0); i++) {
        for (int j = 0; j < surface.cv_count(1); j++) {
            if (any_rat) {
                double lux, luy, luz, luw, lvx, lvy, lvz, lvw, tx, ty, tz, tw;
                L_u.get_cv_4d(i, j, lux, luy, luz, luw);
                L_v.get_cv_4d(i, j, lvx, lvy, lvz, lvw);
                T.get_cv_4d(i, j, tx, ty, tz, tw);
                surface.set_cv_4d(i, j, lux+lvx-tx, luy+lvy-ty, luz+lvz-tz, luw+lvw-tw);
            } else {
                Point lu = L_u.get_cv(i, j);
                Point lv = L_v.get_cv(i, j);
                Point t = T.get_cv(i, j);
                surface.set_cv(i, j, Point(lu[0]+lv[0]-t[0], lu[1]+lv[1]-t[1], lu[2]+lv[2]-t[2]));
            }
        }
    }

    return surface;
}

NurbsCurve Primitives::create_interpolated(const std::vector<Point>& points,
                                            CurveKnotStyle parameterization) {
    return NurbsCurve::create_interpolated(points, parameterization);
}

} // namespace session_cpp
