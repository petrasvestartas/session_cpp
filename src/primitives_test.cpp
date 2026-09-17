#include "mini_test.h"
#include "primitives.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "mesh.h"
#include "line.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Mesh primitives
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Primitives", "Mesh Arrow") {

    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Mesh m = Primitives::arrow_mesh(line, 1.0);

    MINI_CHECK(m.number_of_vertices() == 29);
    MINI_CHECK(m.number_of_faces() == 28);
}

MINI_TEST("Primitives", "Mesh Cylinder") {

    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Mesh m = Primitives::cylinder_mesh(line, 1.0);

    MINI_CHECK(m.number_of_vertices() == 20);
    MINI_CHECK(m.number_of_faces() == 20);
}

MINI_TEST("Primitives", "Mesh Edge Pipes") {

    Mesh mesh;
    size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0));
    size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0));
    size_t v2 = mesh.add_vertex(Point(1.0, 1.0, 0.0));
    size_t v3 = mesh.add_vertex(Point(0.0, 1.0, 0.0));
    mesh.add_face({v0, v1, v2, v3});
    mesh.set_linecolors({Color::red(), Color::red(), Color::red(), Color::red()});

    std::vector<Mesh> pipes = Primitives::edge_pipes(mesh, 0.1);

    MINI_CHECK(pipes.size() == 4);
    MINI_CHECK(pipes[0].number_of_faces() > 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsCurve primitives
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Primitives", "Nurbscurve Polyline") {

    NurbsCurve c =
        NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(1, 2, 0), Point(2, 0, 0), Point(3, 2, 0), Point(4, 0, 0)});

    MINI_CHECK(c.cv_count() == 5);
    MINI_CHECK(c.order() == 2);
    MINI_CHECK(c.degree() == 1);
    MINI_CHECK(c.is_rational() == false);
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_start()), Point(0, 0, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_end()), Point(4, 0, 0)));
}

MINI_TEST("Primitives", "Nurbscurve Circle") {

    NurbsCurve c = Primitives::circle(0.0, 0.0, 0.0, 1.0);

    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "Nurbscurve Ellipse") {

    NurbsCurve c = Primitives::ellipse(0.0, 0.0, 0.0, 2.0, 1.0);

    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "Nurbscurve Arc") {

    Point start(0.0, 0.0, 0.0);
    Point mid(1.0, 1.0, 0.0);
    Point end(2.0, 0.0, 0.0);
    NurbsCurve c = Primitives::arc(start, mid, end);

    MINI_CHECK(c.cv_count() == 3);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "Nurbscurve Parabola") {

    Point p0(-1.0, 1.0, 0.0);
    Point p1(0.0, 0.0, 0.0);
    Point p2(1.0, 1.0, 0.0);
    NurbsCurve c = Primitives::parabola(p0, p1, p2);

    MINI_CHECK(c.cv_count() == 3);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == false);
}

MINI_TEST("Primitives", "Nurbscurve Hyperbola") {

    Point center(0.0, 0.0, 0.0);
    NurbsCurve c = Primitives::hyperbola(center, 1.0, 1.0, 1.0);

    MINI_CHECK(c.cv_count() >= 4);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.is_rational() == false);
}

MINI_TEST("Primitives", "Nurbscurve Spiral") {

    NurbsCurve c = Primitives::spiral(1.0, 2.0, 1.0, 5.0);

    MINI_CHECK(c.cv_count() >= 4);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.is_rational() == false);
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsSurface primitives
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Primitives", "Nurbssurface Cylinder") {

    NurbsSurface s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 2);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 2);

    Point p00 = s.point_at(0.0, 0.0);

    MINI_CHECK(std::abs(p00[0] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(p00[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p00[2] - 0.0) < 1e-10);

    Point p01 = s.point_at(0.0, 1.0);

    MINI_CHECK(std::abs(p01[0] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(p01[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p01[2] - 5.0) < 1e-10);

    Point pmid = s.point_at(1.0, 0.5);

    MINI_CHECK(std::abs(pmid[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(pmid[1] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(pmid[2] - 2.5) < 1e-10);
}

MINI_TEST("Primitives", "Nurbssurface Cone") {

    NurbsSurface s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 2);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 2);

    Point pbase = s.point_at(0.0, 0.0);

    MINI_CHECK(std::abs(pbase[0] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(pbase[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(pbase[2] - 0.0) < 1e-10);

    Point papex = s.point_at(0.0, 1.0);

    MINI_CHECK(std::abs(papex[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(papex[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(papex[2] - 5.0) < 1e-10);

    Point pmid = s.point_at(0.0, 0.5);

    MINI_CHECK(std::abs(pmid[0] - 0.5) < 1e-10);
    MINI_CHECK(std::abs(pmid[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(pmid[2] - 2.5) < 1e-10);
}

MINI_TEST("Primitives", "Nurbssurface Torus") {

    NurbsSurface s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 9);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 3);

    Point p00 = s.point_at(0.0, 0.0);

    MINI_CHECK(std::abs(p00[0] - 4.0) < 1e-10);
    MINI_CHECK(std::abs(p00[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p00[2] - 0.0) < 1e-10);

    Point p10 = s.point_at(1.0, 0.0);

    MINI_CHECK(std::abs(p10[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p10[1] - 4.0) < 1e-10);
    MINI_CHECK(std::abs(p10[2] - 0.0) < 1e-10);

    Point p_top = s.point_at(0.0, 1.0);

    MINI_CHECK(std::abs(p_top[0] - 3.0) < 1e-10);
    MINI_CHECK(std::abs(p_top[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p_top[2] - 1.0) < 1e-10);
}

MINI_TEST("Primitives", "Nurbssurface Sphere") {

    NurbsSurface s = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 5);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 3);

    Point p00 = s.point_at(0.0, 0.0);

    MINI_CHECK(std::abs(p00[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p00[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p00[2] - (-2.0)) < 1e-10);

    Point p_top = s.point_at(0.0, 2.0);

    MINI_CHECK(std::abs(p_top[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p_top[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p_top[2] - 2.0) < 1e-10);

    Point p_eq = s.point_at(0.0, 1.0);

    MINI_CHECK(std::abs(p_eq[0] - 2.0) < 1e-10);
    MINI_CHECK(std::abs(p_eq[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p_eq[2] - 0.0) < 1e-10);

    Point p_eq2 = s.point_at(1.0, 1.0);

    MINI_CHECK(std::abs(p_eq2[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p_eq2[1] - 2.0) < 1e-10);
    MINI_CHECK(std::abs(p_eq2[2] - 0.0) < 1e-10);
}

MINI_TEST("Primitives", "Nurbssurface Quad Sphere") {

    const double radius = 5.0;
    std::vector<NurbsSurface> faces = Primitives::quad_sphere(0.0, 0.0, 0.0, radius);

    MINI_CHECK(faces.size() == 6);

    for (int f = 0; f < 6; f++) {
        MINI_CHECK(faces[f].is_valid());
        MINI_CHECK(faces[f].is_rational());
        MINI_CHECK(faces[f].order(0) == 3);
        MINI_CHECK(faces[f].order(1) == 3);
        MINI_CHECK(faces[f].cv_count(0) == 3);
        MINI_CHECK(faces[f].cv_count(1) == 3);
    }

    double max_err = 0.0;

    for (int f = 0; f < 6; f++) {
        for (int i = 0; i <= 4; i++) {
            double u = i / 4.0;

            for (int j = 0; j <= 4; j++) {
                double v = j / 4.0;
                Point p = faces[f].point_at(u, v);
                double dist = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
                const double err = std::abs(dist - radius);

                if (err > max_err)
                    max_err = err;
            }
        }
    }

    MINI_CHECK(max_err < 0.02 * radius);

    Point top = faces[0].point_at(0.5, 0.5);

    MINI_CHECK(std::abs(top[2] - radius) < 1e-10);
    MINI_CHECK(std::abs(top[0]) < 1e-10);
    MINI_CHECK(std::abs(top[1]) < 1e-10);

    Point bottom = faces[1].point_at(0.5, 0.5);

    MINI_CHECK(std::abs(bottom[2] + radius) < 1e-10);

    Point right = faces[2].point_at(0.5, 0.5);

    MINI_CHECK(std::abs(right[0] - radius) < 1e-10);

    Point left = faces[3].point_at(0.5, 0.5);

    MINI_CHECK(std::abs(left[0] + radius) < 1e-10);

    Point front = faces[4].point_at(0.5, 0.5);

    MINI_CHECK(std::abs(front[1] - radius) < 1e-10);

    Point back = faces[5].point_at(0.5, 0.5);

    MINI_CHECK(std::abs(back[1] + radius) < 1e-10);
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsSurface factory methods
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Primitives", "Nurbssurface Ruled") {

    std::vector<Point> pts_a = {
        Point(3.0, 0.0, 0.0),
        Point(-2.0, 0.0, 5.0),
    };
    std::vector<Point> pts_b = {
        Point(3.0, 5.0, 5.0),
        Point(-2.0, 5.0, 0.0),
    };
    NurbsCurve crv_a = NurbsCurve::create(false, 1, pts_a);
    NurbsCurve crv_b = NurbsCurve::create(false, 1, pts_b);
    NurbsSurface srf = Primitives::create_ruled(crv_a, crv_b);

    Mesh m = srf.mesh();

    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.degree(0) == 1);
    MINI_CHECK(srf.degree(1) == 1);
    MINI_CHECK(srf.cv_count(0) == 2);
    MINI_CHECK(srf.cv_count(1) == 2);

    auto [rd, rv, ruv] = srf.divide_by_count_points(4, 4);

    MINI_CHECK(rd.size() == 5);
    MINI_CHECK(rd[0].size() == 5);

    std::vector<Point> pts;

    for (int i = 0; i < (int)rd.size(); i++)
        for (int j = 0; j < (int)rd[i].size(); j++)
            pts.push_back(rd[i][j]);

    std::vector<Vector> normals;

    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++)
            normals.push_back(srf.normal_at(ruv[i][j].first, ruv[i][j].second));

    std::vector<std::pair<double, double>> uvs;

    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++)
            uvs.push_back(ruv[i][j]);

    MINI_CHECK(TOLERANCE.is_point_close(pts[0], Point(3.00, 0.00, 0.00)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[1], Point(3.00, 1.25, 1.25)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[2], Point(3.00, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[3], Point(3.00, 3.75, 3.75)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[4], Point(3.00, 5.00, 5.00)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[5], Point(1.75, 0.00, 1.25)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[6], Point(1.75, 1.25, 1.875)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[7], Point(1.75, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[8], Point(1.75, 3.75, 3.125)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[9], Point(1.75, 5.00, 3.75)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[10], Point(0.50, 0.00, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[11], Point(0.50, 1.25, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[12], Point(0.50, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[13], Point(0.50, 3.75, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[14], Point(0.50, 5.00, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[15], Point(-0.75, 0.00, 3.75)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[16], Point(-0.75, 1.25, 3.125)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[17], Point(-0.75, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[18], Point(-0.75, 3.75, 1.875)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[19], Point(-0.75, 5.00, 1.25)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[20], Point(-2.00, 0.00, 5.00)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[21], Point(-2.00, 1.25, 3.75)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[22], Point(-2.00, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[23], Point(-2.00, 3.75, 1.25)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[24], Point(-2.00, 5.00, 0.00)));

    MINI_CHECK(
        TOLERANCE.is_vector_close(normals[0], Vector(-0.577350269189626, 0.577350269189626, -0.577350269189626))
    );
    MINI_CHECK(TOLERANCE.is_vector_close(normals[1], Vector(-1.0 / 3.0, 2.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[2], Vector(0.0, 0.707106781186547, -0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[3], Vector(1.0 / 3.0, 2.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[4], Vector(0.577350269189626, 0.577350269189626, -0.577350269189626)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[5], Vector(-2.0 / 3.0, 1.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(
        TOLERANCE.is_vector_close(normals[6], Vector(-0.408248290463863, 0.408248290463863, -0.816496580927726))
    );
    MINI_CHECK(TOLERANCE.is_vector_close(normals[7], Vector(0.0, 0.447213595499958, -0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[8], Vector(0.408248290463863, 0.408248290463863, -0.816496580927726)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[9], Vector(2.0 / 3.0, 1.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[10], Vector(-0.707106781186547, 0.0, -0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[11], Vector(-0.447213595499958, 0.0, -0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[12], Vector(0.0, 0.0, -1.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[13], Vector(0.447213595499958, 0.0, -0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[14], Vector(0.707106781186547, 0.0, -0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[15], Vector(-2.0 / 3.0, -1.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(
        TOLERANCE.is_vector_close(normals[16], Vector(-0.408248290463863, -0.408248290463863, -0.816496580927726))
    );
    MINI_CHECK(TOLERANCE.is_vector_close(normals[17], Vector(0.0, -0.447213595499958, -0.894427190999916)));
    MINI_CHECK(
        TOLERANCE.is_vector_close(normals[18], Vector(0.408248290463863, -0.408248290463863, -0.816496580927726))
    );
    MINI_CHECK(TOLERANCE.is_vector_close(normals[19], Vector(2.0 / 3.0, -1.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(
        TOLERANCE.is_vector_close(normals[20], Vector(-0.577350269189626, -0.577350269189626, -0.577350269189626))
    );
    MINI_CHECK(TOLERANCE.is_vector_close(normals[21], Vector(-1.0 / 3.0, -2.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[22], Vector(0.0, -0.707106781186547, -0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[23], Vector(1.0 / 3.0, -2.0 / 3.0, -2.0 / 3.0)));
    MINI_CHECK(
        TOLERANCE.is_vector_close(normals[24], Vector(0.577350269189626, -0.577350269189626, -0.577350269189626))
    );
    MINI_CHECK(TOLERANCE.is_close(uvs[0].first, 0.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[0].second, 0.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[1].first, 0.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[1].second, 0.25));
    MINI_CHECK(TOLERANCE.is_close(uvs[4].first, 0.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[4].second, 1.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[6].first, 0.25));
    MINI_CHECK(TOLERANCE.is_close(uvs[6].second, 0.25));
    MINI_CHECK(TOLERANCE.is_close(uvs[12].first, 0.50));
    MINI_CHECK(TOLERANCE.is_close(uvs[12].second, 0.50));
    MINI_CHECK(TOLERANCE.is_close(uvs[24].first, 1.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[24].second, 1.00));
}

MINI_TEST("Primitives", "Nurbssurface Planar") {

    TOLERANCE.set_absolute(1e-6);
    const double c1 = std::cos(0.7);
    const double s1 = std::sin(0.7);
    const double c2 = std::cos(0.96);
    const double s2 = std::sin(0.96);
    const double c3 = std::cos(0.52);
    const double s3 = std::sin(0.52);
    const double c4 = std::cos(1.13);
    const double s4 = std::sin(1.13);

    const NurbsCurve ca = NurbsCurve::create(
        false,
        1,
        {Point(0, 0, 0), Point(4, 0, 0), Point(4, 3 * c1, 3 * s1), Point(0, 3 * c1, 3 * s1), Point(0, 0, 0)}
    );
    const NurbsSurface s_quad = Primitives::create_planar(ca);
    const Mesh m_quad = s_quad.mesh();

    const NurbsCurve cb1 = NurbsCurve::create(
        false,
        1,
        {Point(8, 0, 0), Point(8 + 5 * c2, 0, 5 * s2), Point(8 + 2 * c2, 3, 2 * s2), Point(8, 0, 0)}
    );
    const NurbsSurface s_triangle = Primitives::create_planar(cb1);
    const Mesh m_triangle = s_triangle.mesh();

    const double ox = 18.0;
    const NurbsCurve cb2 = NurbsCurve::create(
        false,
        1,
        {Point(ox + 0 * c3, 0 * s3, 0),
         Point(ox + 4 * c3, 4 * s3, 0),
         Point(ox + 5 * c3 - 2 * s3, 5 * s3 + 2 * c3, 0),
         Point(ox + 3 * c3 - 4 * s3, 3 * s3 + 4 * c3, 0),
         Point(ox - 1 * c3 - 3 * s3, -1 * s3 + 3 * c3, 0),
         Point(ox + 0 * c3, 0 * s3, 0)}
    );
    const NurbsSurface s_polygon = Primitives::create_planar(cb2);
    const Mesh m_polygon = s_polygon.mesh();

    const NurbsCurve cc = NurbsCurve::create(
        false,
        3,
        {Point(26, 0, 0),
         Point(29, 1 * c4, 1 * s4),
         Point(31, 0.5 * c4, 0.5 * s4),
         Point(32, 3 * c4, 3 * s4),
         Point(30, 5 * c4, 5 * s4),
         Point(27, 4 * c4, 4 * s4),
         Point(26, 0, 0)}
    );
    const NurbsSurface s_nurbs = Primitives::create_planar(cc);
    const Mesh m_nurbs = s_nurbs.mesh();

    MINI_CHECK(s_quad.is_valid());
    MINI_CHECK(s_quad.is_planar());
    MINI_CHECK(s_quad.cv_count(0) == 2);
    MINI_CHECK(s_quad.cv_count(1) == 2);
    MINI_CHECK(m_quad.number_of_vertices() == 4);
    MINI_CHECK(m_quad.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(0, 0), Point(0.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(0, 1), Point(0.0, 2.294526561853465, 1.932653061713073)));
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(1, 0), Point(4.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(1, 1), Point(4.0, 2.294526561853465, 1.932653061713073)));

    MINI_CHECK(s_triangle.is_valid());
    MINI_CHECK(s_triangle.is_planar());
    MINI_CHECK(s_triangle.cv_count(0) == 2);
    MINI_CHECK(s_triangle.cv_count(1) == 2);
    MINI_CHECK(m_triangle.number_of_vertices() == 3);
    MINI_CHECK(m_triangle.number_of_faces() == 1);
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(0, 0), Point(8.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(0, 1), Point(8.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(1, 0), Point(10.867599930362283, 0.0, 4.095957841504991)));
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(1, 1), Point(9.147039972144913, 3.0, 1.638383136601997)));

    MINI_CHECK(s_polygon.is_valid());
    MINI_CHECK(s_polygon.is_planar());
    MINI_CHECK(s_polygon.cv_count(0) == 2);
    MINI_CHECK(s_polygon.cv_count(1) == 2);
    MINI_CHECK(m_polygon.number_of_vertices() == 4);
    MINI_CHECK(m_polygon.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(0, 0), Point(19.673777861921977, 6.364048611360808, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(0, 1), Point(22.915428262469927, 2.987233669553135, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(1, 0), Point(15.247175891573059, 2.114631246911942, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(1, 1), Point(18.488826292121008, -1.262183694895731, 0.0)));

    MINI_CHECK(s_nurbs.is_valid());
    MINI_CHECK(s_nurbs.is_planar());
    MINI_CHECK(s_nurbs.cv_count(0) == 2);
    MINI_CHECK(s_nurbs.cv_count(1) == 2);
    MINI_CHECK(m_nurbs.number_of_vertices() == 4);
    MINI_CHECK(m_nurbs.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(
        s_nurbs.get_cv(0, 0),
        Point(26.652846559932474, -0.727774577493594, -1.542700265577809)
    ));

    MINI_CHECK(
        TOLERANCE.is_point_close(s_nurbs.get_cv(0, 1), Point(24.347485651711366, 0.916607409071279, 1.942978687541882))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_nurbs.get_cv(1, 0), Point(32.606791655643732, 0.791738725121784, 1.678288276735475))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_nurbs.get_cv(1, 1), Point(30.301430747422629, 2.436120711686657, 5.163967229855166))
    );

    TOLERANCE.reset();
}

MINI_TEST("Primitives", "Nurbssurface Extrusion") {

    Vector dir(0, 1, 5);

    const NurbsCurve c1 = NurbsCurve::create(false, 1, {Point(13, 0, 0), Point(18, 0, 0)});
    const NurbsSurface s_line = Primitives::create_extrusion(c1, dir);
    const Mesh m_line = s_line.mesh();

    const NurbsCurve c2 = Primitives::circle(24, 0, 0, 3.0);
    const NurbsSurface s_circle = Primitives::create_extrusion(c2, dir);
    const Mesh m_circle = s_circle.mesh();

    const NurbsCurve c3 = NurbsCurve::create(false, 2, {Point(30, 0, 0), Point(33, 5, 0), Point(37, 0, 0)});
    const NurbsSurface s_arc = Primitives::create_extrusion(c3, dir);
    const Mesh m_arc = s_arc.mesh();

    const NurbsCurve c4 = NurbsCurve::create(false, 1, {Point(40, 3, 0), Point(45, 0, 0), Point(50, 3, 0), Point(55, 0, 0)});
    const NurbsSurface s_wavy = Primitives::create_extrusion(c4, dir);
    const Mesh m_wavy = s_wavy.mesh();

    MINI_CHECK(s_line.is_valid());
    MINI_CHECK(s_line.degree(0) == 1 && s_line.degree(1) == 1);
    MINI_CHECK(s_line.cv_count(0) == 2 && s_line.cv_count(1) == 2);
    MINI_CHECK(m_line.number_of_vertices() == 4);
    MINI_CHECK(m_line.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(0, 0), Point(13.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(0, 1), Point(13.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(1, 0), Point(18.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(1, 1), Point(18.0, 1.0, 5.0)));

    MINI_CHECK(s_circle.is_valid());
    MINI_CHECK(s_circle.degree(0) == 2 && s_circle.degree(1) == 1);
    MINI_CHECK(s_circle.is_rational());
    MINI_CHECK(s_circle.is_closed(0) == true && s_circle.is_closed(1) == false);
    MINI_CHECK(s_circle.cv_count(0) == 9 && s_circle.cv_count(1) == 2);
    MINI_CHECK(m_circle.number_of_vertices() == 42);
    MINI_CHECK(m_circle.number_of_faces() == 42);
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(0, 0), Point(27.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(0, 1), Point(27.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(4, 0), Point(21.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(4, 1), Point(21.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(8, 0), Point(27.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(8, 1), Point(27.0, 1.0, 5.0)));

    MINI_CHECK(s_arc.is_valid());
    MINI_CHECK(s_arc.degree(0) == 2 && s_arc.degree(1) == 1);
    MINI_CHECK(s_arc.cv_count(0) == 3 && s_arc.cv_count(1) == 2);
    MINI_CHECK(m_arc.number_of_vertices() == 16);
    MINI_CHECK(m_arc.number_of_faces() == 14);
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(0, 0), Point(30.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(0, 1), Point(30.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(1, 0), Point(33.0, 5.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(1, 1), Point(33.0, 6.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(2, 0), Point(37.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(2, 1), Point(37.0, 1.0, 5.0)));

    MINI_CHECK(s_wavy.is_valid());
    MINI_CHECK(s_wavy.degree(0) == 1 && s_wavy.degree(1) == 1);
    MINI_CHECK(s_wavy.cv_count(0) == 4 && s_wavy.cv_count(1) == 2);
    MINI_CHECK(m_wavy.number_of_vertices() == 12);
    MINI_CHECK(m_wavy.number_of_faces() == 6);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            const Point position = s_wavy.get_cv(i, j);
            int copies = 0;

            for (const auto& [key, vertex] : m_wavy.vertex)
                if (TOLERANCE.is_point_close(vertex.position(), position))
                    copies++;

            MINI_CHECK(copies == (i == 0 || i == 3 ? 1 : 2));
        }
    }

    for (const auto& [key, corners] : m_wavy.face) {
        const Vector normal = m_wavy.face_normal(key).value();

        for (const size_t corner : corners) {
            const std::array<double, 3> shading = m_wavy.vertex.at(corner).normal().value();

            for (int axis = 0; axis < 3; axis++)
                MINI_CHECK(std::abs(shading[axis] - normal[axis]) < 1e-9);
        }
    }

    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(0, 0), Point(40.0, 3.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(0, 1), Point(40.0, 4.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(1, 0), Point(45.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(1, 1), Point(45.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(3, 0), Point(55.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(3, 1), Point(55.0, 1.0, 5.0)));
}

MINI_TEST("Primitives", "Nurbssurface Loft") {

    NurbsCurve c1 = Primitives::circle(0, 0, 0.0, 2.0);
    NurbsCurve c2 = Primitives::circle(0, 0, 2.0, 1.0);
    NurbsCurve c3 = Primitives::circle(0, 0, 4.0, 1.5);
    NurbsCurve c4 = Primitives::circle(0, 0, 6.0, 0.8);

    NurbsSurface srf = Primitives::create_loft({c1, c2, c3, c4}, 3);

    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.cv_count(0) == 9);
    MINI_CHECK(srf.cv_count(1) == 4);
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(0, 0), Point(2.000000000000000, 0.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(0, 1), Point(-0.689223125118461, 0.000000000000000, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(0, 2), Point(3.009774760647534, 0.000000000000000, 4.110399016539784))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(0, 3), Point(0.800000000000000, 0.000000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(1, 0), Point(2.000000000000000, 2.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(1, 1), Point(-0.689223125118461, -0.689223125118461, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(1, 2), Point(3.009774760647534, 3.009774760647534, 4.110399016539783))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(1, 3), Point(0.800000000000000, 0.800000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(2, 0), Point(0.000000000000000, 2.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(2, 1), Point(0.000000000000000, -0.689223125118461, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(2, 2), Point(0.000000000000000, 3.009774760647534, 4.110399016539784))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(2, 3), Point(0.000000000000000, 0.800000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(3, 0), Point(-2.000000000000000, 2.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(3, 1), Point(0.689223125118461, -0.689223125118461, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(3, 2), Point(-3.009774760647534, 3.009774760647534, 4.110399016539783))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(3, 3), Point(-0.800000000000000, 0.800000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(4, 0), Point(-2.000000000000000, 0.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(4, 1), Point(0.689223125118461, 0.000000000000000, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(4, 2), Point(-3.009774760647534, 0.000000000000000, 4.110399016539784))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(4, 3), Point(-0.800000000000000, 0.000000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(5, 0), Point(-2.000000000000000, -2.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(5, 1), Point(0.689223125118461, 0.689223125118461, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(5, 2), Point(-3.009774760647534, -3.009774760647534, 4.110399016539783))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(5, 3), Point(-0.800000000000000, -0.800000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(6, 0), Point(0.000000000000000, -2.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(6, 1), Point(0.000000000000000, 0.689223125118461, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(6, 2), Point(0.000000000000000, -3.009774760647534, 4.110399016539784))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(6, 3), Point(0.000000000000000, -0.800000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(7, 0), Point(2.000000000000000, -2.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(7, 1), Point(-0.689223125118461, 0.689223125118461, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(7, 2), Point(3.009774760647534, -3.009774760647534, 4.110399016539783))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(7, 3), Point(0.800000000000000, -0.800000000000000, 6.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(8, 0), Point(2.000000000000000, 0.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(8, 1), Point(-0.689223125118461, 0.000000000000000, 1.662346559763863))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(8, 2), Point(3.009774760647534, 0.000000000000000, 4.110399016539784))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(srf.get_cv(8, 3), Point(0.800000000000000, 0.000000000000000, 6.000000000000000))
    );

    std::vector<std::vector<Point>> open_pts = {
        {
            Point(10, -12, 0),
            Point(10, -10, 3),
            Point(10, -7, 3),
            Point(10, -5, 0),
        },
        {
            Point(5.5, -12, 3.5),
            Point(5.5, -10.0, 1.5),
            Point(5.5, -7.0, 1.5),
            Point(5.5, -5, 3.5),
        },
        {
            Point(1, -12, 0),
            Point(1, -10, 3.0),
            Point(1, -7, 3.0),
            Point(1, -5, 0),
        },
    };
    std::vector<NurbsCurve> open_curves = {
        NurbsCurve::create(false, 3, open_pts[0]),
        NurbsCurve::create(false, 3, open_pts[1]),
        NurbsCurve::create(false, 3, open_pts[2]),
    };
    NurbsSurface open_srf = Primitives::create_loft(open_curves, 3);

    MINI_CHECK(open_srf.is_valid());
    MINI_CHECK(open_srf.cv_count(0) == 4);
    MINI_CHECK(open_srf.cv_count(1) == 3);

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(0, 0),
        Point(10.000000000000000, -12.000000000000000, 0.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(0, 1),
        Point(5.500000000000000, -12.000000000000000, 7.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(0, 2),
        Point(1.000000000000000, -12.000000000000000, 0.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(1, 0),
        Point(10.000000000000000, -10.000000000000000, 3.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(1, 1),
        Point(5.500000000000000, -10.000000000000000, 0.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(1, 2),
        Point(1.000000000000000, -10.000000000000000, 3.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(2, 0),
        Point(10.000000000000000, -7.000000000000000, 3.000000000000000)
    ));

    MINI_CHECK(
        TOLERANCE.is_point_close(open_srf.get_cv(2, 1), Point(5.500000000000000, -7.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(open_srf.get_cv(2, 2), Point(1.000000000000000, -7.000000000000000, 3.000000000000000))
    );
    MINI_CHECK(TOLERANCE.is_point_close(
        open_srf.get_cv(3, 0),
        Point(10.000000000000000, -5.000000000000000, 0.000000000000000)
    ));

    MINI_CHECK(
        TOLERANCE.is_point_close(open_srf.get_cv(3, 1), Point(5.500000000000000, -5.000000000000000, 7.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(open_srf.get_cv(3, 2), Point(1.000000000000000, -5.000000000000000, 0.000000000000000))
    );
}

MINI_TEST("Primitives", "Nurbssurface Revolve") {

    const NurbsCurve pa = NurbsCurve::create(
        false,
        3,
        {
            Point(1.5, 0, 0),
            Point(1.5, 0, 0.3),
            Point(0.3, 0, 0.5),
            Point(0.3, 0, 2.5),
            Point(0.2, 0, 3.0),
            Point(2.0, 0, 4.5),
            Point(1.8, 0, 5.0),
        }
    );
    const NurbsSurface s_vase = Primitives::create_revolve(pa, Point(0, 0, 0), Vector(0, 0, 1));
    const Mesh m_vase = s_vase.mesh();

    NurbsCurve pb(3, true, 3, 9);
    const double w = std::sqrt(2.0) / 2.0;
    const double cw[] = {1, w, 1, w, 1, w, 1, w, 1};
    const double ca[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double sa[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    const double ck[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    const double rr = 5.0;
    const double r = 1.5;
    const double tcx = 14.0;

    for (int i = 0; i < 10; i++)
        pb.set_nurbsknot(i, ck[i]);

    for (int i = 0; i < 9; i++)
        pb.set_cv_4d(i, (tcx + rr + r * ca[i]) * cw[i], 0.0, r * sa[i] * cw[i], cw[i]);

    const NurbsSurface s_torus = Primitives::create_revolve(pb, Point(tcx, 0, 0), Vector(0, 0, 1));
    const Mesh m_torus = s_torus.mesh();

    const NurbsCurve pc = NurbsCurve::create(false, 1, {Point(29, 0, -0.5), Point(29, 0, 0.5)});
    const NurbsSurface s_elbow = Primitives::create_revolve(pc, Point(26, 0, 0), Vector(0, 0, 1), Tolerance::PI / 2.0);
    const Mesh m_elbow = s_elbow.mesh();

    const double sr = 2.0;
    const double scx = 36.0;
    NurbsCurve pd(3, true, 3, 5);
    const double sk[] = {0, 0, 1, 1, 2, 2};

    for (int i = 0; i < 6; i++)
        pd.set_nurbsknot(i, sk[i]);

    const double spx[] = {0, sr, sr, sr, 0};
    const double spz[] = {-sr, -sr, 0, sr, sr};
    const double spw[] = {1, w, 1, w, 1};

    for (int i = 0; i < 5; i++)
        pd.set_cv_4d(i, (scx + spx[i]) * spw[i], 0.0, spz[i] * spw[i], spw[i]);

    const NurbsSurface s_sphere = Primitives::create_revolve(pd, Point(scx, 0, 0), Vector(0, 0, 1));
    const Mesh m_sphere = s_sphere.mesh();

    const NurbsCurve pe = NurbsCurve::create(false, 1, {Point(44, 0, 3), Point(46, 0, 0)});
    const NurbsSurface s_cone = Primitives::create_revolve(pe, Point(44, 0, 0), Vector(0, 0, 1));
    const Mesh m_cone = s_cone.mesh();

    MINI_CHECK(s_vase.is_valid());
    MINI_CHECK(s_vase.is_closed(0) == true);
    MINI_CHECK(s_vase.is_closed(1) == false);
    MINI_CHECK(s_vase.cv_count(0) == 9);
    MINI_CHECK(s_vase.cv_count(1) == 7);
    MINI_CHECK(m_vase.number_of_vertices() == 609);
    MINI_CHECK(m_vase.number_of_faces() == 1176);
    MINI_CHECK(TOLERANCE.is_point_close(s_vase.get_cv(0, 0), Point(1.5, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_vase.get_cv(0, 6), Point(1.8, 0.0, 5.0)));

    MINI_CHECK(s_torus.is_valid());
    MINI_CHECK(s_torus.is_closed(0) == true);
    MINI_CHECK(s_torus.is_closed(1) == true);
    MINI_CHECK(s_torus.cv_count(0) == 9);
    MINI_CHECK(s_torus.cv_count(1) == 9);
    MINI_CHECK(m_torus.number_of_vertices() == 693);
    MINI_CHECK(m_torus.number_of_faces() == 1386);
    MINI_CHECK(TOLERANCE.is_point_close(s_torus.get_cv(0, 0), Point(20.5, 0.0, 0.0)));

    MINI_CHECK(s_elbow.is_valid());
    MINI_CHECK(s_elbow.is_closed(0) == false);
    MINI_CHECK(s_elbow.is_closed(1) == false);
    MINI_CHECK(s_elbow.cv_count(0) == 3);
    MINI_CHECK(s_elbow.cv_count(1) == 2);
    MINI_CHECK(m_elbow.number_of_vertices() == 16);
    MINI_CHECK(m_elbow.number_of_faces() == 14);
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(0, 0), Point(29.0, 0.0, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(0, 1), Point(29.0, 0.0, 0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(2, 0), Point(26.0, 3.0, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(2, 1), Point(26.0, 3.0, 0.5)));

    MINI_CHECK(s_sphere.is_valid());
    MINI_CHECK(s_sphere.is_closed(0) == true);
    MINI_CHECK(s_sphere.is_closed(1) == false);
    MINI_CHECK(s_sphere.is_singular(0) == true);
    MINI_CHECK(s_sphere.is_singular(2) == true);
    MINI_CHECK(s_sphere.cv_count(0) == 9);
    MINI_CHECK(s_sphere.cv_count(1) == 5);
    MINI_CHECK(m_sphere.number_of_vertices() == 191);
    MINI_CHECK(m_sphere.number_of_faces() == 378);
    MINI_CHECK(TOLERANCE.is_point_close(s_sphere.get_cv(0, 0), Point(36.0, 0.0, -2.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sphere.get_cv(0, 4), Point(36.0, 0.0, 2.0)));

    MINI_CHECK(s_cone.is_valid());
    MINI_CHECK(s_cone.is_closed(0) == true);
    MINI_CHECK(s_cone.is_closed(1) == false);
    MINI_CHECK(s_cone.is_singular(0) == true);
    MINI_CHECK(s_cone.is_singular(2) == false);
    MINI_CHECK(s_cone.cv_count(0) == 9);
    MINI_CHECK(s_cone.cv_count(1) == 2);
    MINI_CHECK(m_cone.number_of_vertices() == 22);
    MINI_CHECK(m_cone.number_of_faces() == 21);
    MINI_CHECK(TOLERANCE.is_point_close(s_cone.get_cv(0, 0), Point(44.0, 0.0, 3.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_cone.get_cv(0, 1), Point(46.0, 0.0, 0.0)));
}

MINI_TEST("Primitives", "Nurbssurface Sweep") {

    NurbsCurve rail = NurbsCurve::create(false, 2, {Point(0, 0, 0), Point(0, 5, 0), Point(2, 9, 0)});
    NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
    NurbsSurface s_sweep1 = Primitives::create_sweep1(rail, profile);
    const Mesh m_sweep1 = s_sweep1.mesh();

    NurbsCurve rail1 = NurbsCurve::create(false, 2, {Point(6, -1, 0), Point(7, 3, 0), Point(8, 4, 0)});
    NurbsCurve rail2 = NurbsCurve::create(false, 2, {Point(10, -1, 0), Point(10, 3, 0), Point(9, 4, 0)});
    NurbsCurve shape1 = NurbsCurve::create(false, 2, {Point(6, -1, 0), Point(8, -1, 2), Point(10, -1, 0)});
    NurbsCurve shape2 = NurbsCurve::create(false, 2, {Point(8, 4, 0), Point(8.5, 4, 1.5), Point(9, 4, 0)});
    NurbsSurface s_sweep2 = Primitives::create_sweep2(rail1, rail2, {shape1, shape2});
    const Mesh m_sweep2 = s_sweep2.mesh();

    MINI_CHECK(s_sweep1.is_valid());
    MINI_CHECK(s_sweep1.is_rational());
    MINI_CHECK(s_sweep1.cv_count(0) == 9);
    MINI_CHECK(s_sweep1.cv_count(1) == 6);
    MINI_CHECK(m_sweep1.number_of_vertices() > 0);
    MINI_CHECK(m_sweep1.number_of_faces() > 0);
    TOLERANCE.set_absolute(1e-6);

    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(0, 0), Point(0.888888888888889, 0.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(0, 1), Point(0.888650781842197, 1.196033690639573, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(0, 2), Point(1.023137542521078, 2.984678629452259, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(0, 3), Point(1.644124175132323, 5.883369976716751, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(0, 4), Point(2.267033741447567, 7.548154043673421, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(0, 5), Point(2.795046402150731, 8.602476824301650, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(1, 0), Point(0.888888888888889, 0.000000000000000, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(1, 1), Point(0.888650781842196, 1.196033690639572, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(1, 2), Point(1.023137542521079, 2.984678629452261, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(1, 3), Point(1.644124175132322, 5.883369976716749, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(1, 4), Point(2.267033741447568, 7.548154043673421, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(1, 5), Point(2.795046402150731, 8.602476824301650, -1.000000000000000))
    );
    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep1.get_cv(2, 0),
        Point(-0.111111111111111, 0.000000000000000, -1.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep1.get_cv(2, 1),
        Point(-0.111355426965362, 1.245520819229018, -1.000000000000000)
    ));

    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(2, 2), Point(0.028671366170157, 3.117459574526332, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(2, 3), Point(0.682455101244336, 6.170731523133928, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(2, 4), Point(1.341409898439919, 7.933301269620500, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(2, 5), Point(1.900619199731158, 9.049690396962294, -1.000000000000000))
    );
    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep1.get_cv(3, 0),
        Point(-1.111111111111111, 0.000000000000000, -1.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep1.get_cv(3, 1),
        Point(-1.111361635772921, 1.295007947818465, -1.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep1.get_cv(3, 2),
        Point(-0.965794810180765, 3.250240519600404, -1.000000000000000)
    ));

    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep1.get_cv(3, 3),
        Point(-0.279213972643651, 6.458093069551111, -1.000000000000000)
    ));

    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(3, 4), Point(0.415786055432270, 8.318448495567573, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(3, 5), Point(1.006191997311586, 9.496903969622938, -1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(4, 0), Point(-1.111111111111111, 0.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(4, 1), Point(-1.111361635772921, 1.295007947818464, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(4, 2), Point(-0.965794810180765, 3.250240519600406, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(4, 3), Point(-0.279213972643651, 6.458093069551108, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(4, 4), Point(0.415786055432269, 8.318448495567575, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(4, 5), Point(1.006191997311586, 9.496903969622938, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(5, 0), Point(-1.111111111111111, 0.000000000000000, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(5, 1), Point(-1.111361635772921, 1.295007947818465, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(5, 2), Point(-0.965794810180765, 3.250240519600404, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(5, 3), Point(-0.279213972643651, 6.458093069551111, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(5, 4), Point(0.415786055432270, 8.318448495567573, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(5, 5), Point(1.006191997311586, 9.496903969622938, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(6, 0), Point(-0.111111111111111, 0.000000000000000, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(6, 1), Point(-0.111355426965362, 1.245520819229018, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(6, 2), Point(0.028671366170157, 3.117459574526332, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(6, 3), Point(0.682455101244336, 6.170731523133928, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(6, 4), Point(1.341409898439919, 7.933301269620500, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(6, 5), Point(1.900619199731158, 9.049690396962294, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(7, 0), Point(0.888888888888889, 0.000000000000000, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(7, 1), Point(0.888650781842196, 1.196033690639572, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(7, 2), Point(1.023137542521079, 2.984678629452261, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(7, 3), Point(1.644124175132322, 5.883369976716749, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(7, 4), Point(2.267033741447568, 7.548154043673421, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(7, 5), Point(2.795046402150731, 8.602476824301650, 1.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(8, 0), Point(0.888888888888889, 0.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(8, 1), Point(0.888650781842197, 1.196033690639573, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(8, 2), Point(1.023137542521078, 2.984678629452259, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(8, 3), Point(1.644124175132323, 5.883369976716751, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(8, 4), Point(2.267033741447567, 7.548154043673421, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep1.get_cv(8, 5), Point(2.795046402150731, 8.602476824301650, 0.000000000000000))
    );
    MINI_CHECK(s_sweep2.is_valid());
    MINI_CHECK(s_sweep2.cv_count(0) == 3);
    MINI_CHECK(s_sweep2.cv_count(1) == 6);
    MINI_CHECK(m_sweep2.number_of_vertices() > 0);
    MINI_CHECK(m_sweep2.number_of_faces() > 0);
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(0, 0), Point(6.000000000000000, -1.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(0, 1), Point(6.175969120718316, -0.300506740098127, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(0, 2), Point(6.459569103687756, 0.747208154997334, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(0, 3), Point(7.052015306099445, 2.456377031677760, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(0, 4), Point(7.525387263758168, 3.480360762535406, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(0, 5), Point(8.000000000000000, 4.000000000000000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(1, 0), Point(8.000000000000000, -1.000000000000000, 2.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(1, 1), Point(8.087302079238063, -0.305563785913389, 2.040878660089621))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(1, 2), Point(8.215030901365994, 0.738012519337792, 2.128261757662849))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(1, 3), Point(8.402488235814772, 2.450241721213838, 2.205206310224664))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(1, 4), Point(8.490843623722080, 3.486294461007204, 2.229418074862260))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(1, 5), Point(8.500000000000000, 4.000000000000000, 1.500000000000000))
    );
    MINI_CHECK(TOLERANCE.is_point_close(
        s_sweep2.get_cv(2, 0),
        Point(10.000000000000000, -1.000000000000000, 0.000000000000000)
    ));

    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(2, 1), Point(9.998635037757797, -0.310620831728651, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(2, 2), Point(9.970492699044241, 0.728816883678250, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(2, 3), Point(9.752961165530088, 2.444106410749916, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(2, 4), Point(9.456299983685991, 3.492228159479000, 0.000000000000000))
    );
    MINI_CHECK(
        TOLERANCE.is_point_close(s_sweep2.get_cv(2, 5), Point(9.000000000000000, 4.000000000000000, 0.000000000000000))
    );
    TOLERANCE.reset();
}

MINI_TEST("Primitives", "Nurbssurface Edge") {

    std::vector<Point> pts_south = {
        Point(1, 20.569076, 0),
        Point(1, 22.569076, 3.0),
        Point(1, 25.569076, 3.0),
        Point(1, 27.569076, 0),
    };
    std::vector<Point> pts_west = {
        Point(10, 20.569076, 0),
        Point(5.5, 20.569076, 3.5),
        Point(1, 20.569076, 0),
    };
    std::vector<Point> pts_north = {
        Point(10, 20.569076, 0),
        Point(10, 22.569076, 3),
        Point(10, 25.569076, 3),
        Point(10, 27.569076, 0),
    };
    std::vector<Point> pts_east = {
        Point(10, 27.569076, 0),
        Point(5.5, 27.569076, 3.5),
        Point(1, 27.569076, 0),
    };

    NurbsCurve south = NurbsCurve::create(false, 3, pts_south);
    NurbsCurve west = NurbsCurve::create(false, 2, pts_west);
    NurbsCurve north = NurbsCurve::create(false, 3, pts_north);
    NurbsCurve east = NurbsCurve::create(false, 2, pts_east);

    NurbsSurface surf = Primitives::create_edge(south, west, north, east);
    Mesh m = surf.mesh();

    MINI_CHECK(surf.is_valid());
    MINI_CHECK(m.is_valid());
    MINI_CHECK(surf.degree(0) == 2);
    MINI_CHECK(surf.degree(1) == 3);
    MINI_CHECK(surf.cv_count(0) == 3);
    MINI_CHECK(surf.cv_count(1) == 4);

    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(0, 0), Point(1, 20.569076, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(0, 1), Point(1, 22.569076, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(0, 2), Point(1, 25.569076, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(0, 3), Point(1, 27.569076, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(1, 0), Point(5.5, 20.569076, 3.5)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(1, 1), Point(5.5, 22.569076, 6.5)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(1, 2), Point(5.5, 25.569076, 6.5)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(1, 3), Point(5.5, 27.569076, 3.5)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(2, 0), Point(10, 20.569076, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(2, 1), Point(10, 22.569076, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(2, 2), Point(10, 25.569076, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(surf.get_cv(2, 3), Point(10, 27.569076, 0)));
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface-to-mesh subdivision
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Primitives", "Mesh Quad Mesh") {

    NurbsSurface cyl = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::quad_mesh(cyl, 8, 4);

    MINI_CHECK(m.number_of_vertices() == 40);
    MINI_CHECK(m.number_of_faces() == 32);
    MINI_CHECK(m.is_valid());

    NurbsSurface sph = Primitives::sphere_surface(0, 0, 0, 3.0);
    Mesh m2 = Primitives::quad_mesh(sph, 8, 4);

    MINI_CHECK(m2.number_of_vertices() == 26);
    MINI_CHECK(m2.number_of_faces() == 32);
    MINI_CHECK(m2.is_valid());
}

MINI_TEST("Primitives", "Mesh Diamond Mesh") {

    NurbsSurface cyl = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::diamond_mesh(cyl, 8, 4);

    MINI_CHECK(m.number_of_vertices() == 40);
    MINI_CHECK(m.number_of_faces() == 20);
    MINI_CHECK(m.is_valid());

    NurbsSurface sph = Primitives::sphere_surface(0, 0, 0, 3.0);
    Mesh m2 = Primitives::diamond_mesh(sph, 8, 4);

    MINI_CHECK(m2.number_of_vertices() == 26);
    MINI_CHECK(m2.number_of_faces() == 12);
    MINI_CHECK(m2.is_valid());
}

MINI_TEST("Primitives", "Mesh Hex Mesh") {

    NurbsSurface cyl = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::hex_mesh(cyl, 6, 4, 1.0 / 3.0);

    MINI_CHECK(m.number_of_vertices() == 78);
    MINI_CHECK(m.number_of_faces() == 15);
    MINI_CHECK(m.is_valid());

    NurbsSurface sph = Primitives::sphere_surface(0, 0, 0, 3.0);
    Mesh m2 = Primitives::hex_mesh(sph, 6, 4, 1.0 / 3.0);

    MINI_CHECK(m2.number_of_vertices() == 68);
    MINI_CHECK(m2.number_of_faces() == 15);
    MINI_CHECK(m2.is_valid());
}

MINI_TEST("Primitives", "Mesh Cone Subdivisions") {

    NurbsSurface cone = Primitives::cone_surface(0, 0, 0, 3.0, 5.0);

    Mesh m1 = Primitives::quad_mesh(cone, 8, 4);

    MINI_CHECK(m1.number_of_vertices() == 33);
    MINI_CHECK(m1.number_of_faces() == 32);
    MINI_CHECK(m1.is_valid());

    Mesh m2 = Primitives::diamond_mesh(cone, 8, 4);

    MINI_CHECK(m2.number_of_vertices() == 33);
    MINI_CHECK(m2.number_of_faces() == 16);
    MINI_CHECK(m2.is_valid());

    Mesh m3 = Primitives::hex_mesh(cone, 6, 4, 1.0 / 3.0);

    MINI_CHECK(m3.number_of_vertices() == 73);
    MINI_CHECK(m3.number_of_faces() == 15);
    MINI_CHECK(m3.is_valid());
}

MINI_TEST("Primitives", "Nurbscurve Interpolated") {

    std::vector<Point> points = {
        Point(14, 9, 0),
        Point(15.342777, 13.734889, 0),
        Point(21.897914, 32.239195, 0),
        Point(24.678472, 0.354555, 0),
        Point(33.813678, 24.76858, 0),
        Point(39.626394, 15.47249, 0),
        Point(41, 13, 0),
    };

    NurbsCurve c = Primitives::create_interpolated(points, CurveNurbsKnotStyle::Chord);

    MINI_CHECK(c.is_valid());
    MINI_CHECK(c.degree() == 3);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.is_rational() == false);

    auto [d0, d1] = c.domain();
    std::vector<double> nurbsknots = c.get_nurbsknots();

    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d0), points[0]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(nurbsknots[3]), points[1]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(nurbsknots[4]), points[2]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(nurbsknots[5]), points[3]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(nurbsknots[6]), points[4]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(nurbsknots[7]), points[5]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d1), points[6]));

    MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(0), points[0]));
    MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(8), points[6]));

    std::vector<Point> pts4 = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 2.0, 0.0),
        Point(3.0, 1.0, 0.0),
        Point(5.0, 3.0, 0.0),
    };
    NurbsCurve c4 = Primitives::create_interpolated(pts4, CurveNurbsKnotStyle::Chord);

    MINI_CHECK(c4.is_valid());
    MINI_CHECK(c4.degree() == 3);
    MINI_CHECK(c4.cv_count() == 6);
    auto [d4_0, d4_1] = c4.domain();

    MINI_CHECK(TOLERANCE.is_point_close(c4.point_at(d4_0), pts4[0]));
    MINI_CHECK(TOLERANCE.is_point_close(c4.point_at(d4_1), pts4[3]));
}

MINI_TEST("Primitives", "Mesh Tetrahedron") {

    Mesh m = Primitives::tetrahedron(2.0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 4);
    MINI_CHECK(m.number_of_faces() == 4);
}

MINI_TEST("Primitives", "Mesh Cube") {

    Mesh m = Primitives::cube(2.0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 8);
    MINI_CHECK(m.number_of_faces() == 6);
}

MINI_TEST("Primitives", "Mesh Octahedron") {

    Mesh m = Primitives::octahedron(2.0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 6);
    MINI_CHECK(m.number_of_faces() == 8);
}

MINI_TEST("Primitives", "Mesh Icosahedron") {

    Mesh m = Primitives::icosahedron(2.0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 12);
    MINI_CHECK(m.number_of_faces() == 20);
}

MINI_TEST("Primitives", "Nurbssurface Wave") {

    NurbsSurface srf = Primitives::wave_surface(10.0, 2.0);

    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.degree(0) == 3);
    MINI_CHECK(srf.degree(1) == 3);
    MINI_CHECK(srf.cv_count(0) == 13);
    MINI_CHECK(srf.cv_count(1) == 13);
    Point corner = srf.point_at(0.0, 0.0);

    MINI_CHECK(std::abs(corner[2]) < 0.1);
}

} // namespace session_cpp
