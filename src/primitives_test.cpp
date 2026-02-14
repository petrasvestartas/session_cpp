#include "mini_test.h"
#include "primitives.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "mesh.h"
#include "line.h"
#include "point.h"
#include "vector.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Mesh primitives
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Primitives", "Mesh_arrow") {
    // uncomment #include "mesh.h"
    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Mesh m = Primitives::arrow_mesh(line, 1.0);

    MINI_CHECK(m.number_of_vertices() == 29);
    MINI_CHECK(m.number_of_faces() == 28);
}

MINI_TEST("Primitives", "Mesh_cylinder") {
    // uncomment #include "mesh.h"
    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Mesh m = Primitives::cylinder_mesh(line, 1.0);

    MINI_CHECK(m.number_of_vertices() == 20);
    MINI_CHECK(m.number_of_faces() == 20);
}

///////////////////////////////////////////////////////////////////////////////////////////
// NurbsCurve primitives
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Primitives", "Nurbscurve_polyline") {
    // uncomment #include "nurbscurve.h"
    NurbsCurve c = NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)});

    MINI_CHECK(c.cv_count() == 5);
    MINI_CHECK(c.order() == 2);
    MINI_CHECK(c.degree() == 1);
    MINI_CHECK(c.is_rational() == false);
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_start()), Point(0,0,0)));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_end()), Point(4,0,0)));
}

MINI_TEST("Primitives", "Nurbscurve_circle") {
    // uncomment #include "nurbscurve.h"
    NurbsCurve c = Primitives::circle(0.0, 0.0, 0.0, 1.0);

    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "Nurbscurve_ellipse") {
    // uncomment #include "nurbscurve.h"
    NurbsCurve c = Primitives::ellipse(0.0, 0.0, 0.0, 2.0, 1.0);

    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "Nurbscurve_arc") {
    // uncomment #include "nurbscurve.h"
    Point start(0.0, 0.0, 0.0);
    Point mid(1.0, 1.0, 0.0);
    Point end(2.0, 0.0, 0.0);
    NurbsCurve c = Primitives::arc(start, mid, end);

    MINI_CHECK(c.cv_count() == 3);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "Nurbscurve_parabola") {
    // uncomment #include "nurbscurve.h"
    Point p0(-1.0, 1.0, 0.0);
    Point p1(0.0, 0.0, 0.0);
    Point p2(1.0, 1.0, 0.0);
    NurbsCurve c = Primitives::parabola(p0, p1, p2);

    MINI_CHECK(c.cv_count() == 3);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == false);
}

MINI_TEST("Primitives", "Nurbscurve_hyperbola") {
    // uncomment #include "nurbscurve.h"
    Point center(0.0, 0.0, 0.0);
    NurbsCurve c = Primitives::hyperbola(center, 1.0, 1.0, 1.0);

    MINI_CHECK(c.cv_count() >= 4);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.is_rational() == false);
}

MINI_TEST("Primitives", "Nurbscurve_spiral") {
    // uncomment #include "nurbscurve.h"
    NurbsCurve c = Primitives::spiral(1.0, 2.0, 1.0, 5.0);

    MINI_CHECK(c.cv_count() >= 4);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.is_rational() == false);
}

///////////////////////////////////////////////////////////////////////////////////////////
// NurbsSurface primitives
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Primitives", "Nurbssurface_cylinder") {
    // uncomment #include "nurbssurface.h"
    NurbsSurface s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
    s.name = "cylinder";

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

MINI_TEST("Primitives", "Nurbssurface_cone") {
    // uncomment #include "nurbssurface.h"
    NurbsSurface s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);
    s.name = "cone";

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

MINI_TEST("Primitives", "Nurbssurface_torus") {
    // uncomment #include "nurbssurface.h"
    NurbsSurface s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);
    s.name = "torus";

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

MINI_TEST("Primitives", "Nurbssurface_sphere") {
    // uncomment #include "nurbssurface.h"
    NurbsSurface s = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    s.name = "sphere";

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

MINI_TEST("Primitives", "Nurbssurface_quad_sphere") {
    // uncomment #include "nurbssurface.h"
    double R = 5.0;
    auto faces = Primitives::quad_sphere(0.0, 0.0, 0.0, R);

    MINI_CHECK(faces.size() == 6);
    for (int f = 0; f < 6; f++) {
        MINI_CHECK(faces[f].is_valid());
        MINI_CHECK(faces[f].is_rational());
        MINI_CHECK(faces[f].order(0) == 3);
        MINI_CHECK(faces[f].order(1) == 3);
        MINI_CHECK(faces[f].cv_count(0) == 3);
        MINI_CHECK(faces[f].cv_count(1) == 3);
    }

    // All surface points should be close to sphere radius
    double max_err = 0.0;
    for (int f = 0; f < 6; f++) {
        for (int i = 0; i <= 4; i++) {
            double u = i / 4.0;
            for (int j = 0; j <= 4; j++) {
                double v = j / 4.0;
                Point p = faces[f].point_at(u, v);
                double dist = std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
                double err = std::abs(dist - R);
                if (err > max_err) max_err = err;
            }
        }
    }
    MINI_CHECK(max_err < 0.02 * R);

    // Face centers should be exactly at radius (by construction)
    Point top = faces[0].point_at(0.5, 0.5);
    MINI_CHECK(std::abs(top[2] - R) < 1e-10);
    MINI_CHECK(std::abs(top[0]) < 1e-10);
    MINI_CHECK(std::abs(top[1]) < 1e-10);

    Point bottom = faces[1].point_at(0.5, 0.5);
    MINI_CHECK(std::abs(bottom[2] + R) < 1e-10);

    Point right = faces[2].point_at(0.5, 0.5);
    MINI_CHECK(std::abs(right[0] - R) < 1e-10);

    Point left = faces[3].point_at(0.5, 0.5);
    MINI_CHECK(std::abs(left[0] + R) < 1e-10);

    Point front = faces[4].point_at(0.5, 0.5);
    MINI_CHECK(std::abs(front[1] - R) < 1e-10);

    Point back = faces[5].point_at(0.5, 0.5);
    MINI_CHECK(std::abs(back[1] + R) < 1e-10);
}

MINI_TEST("Primitives", "Nurbssurface_schwarz_p") {
    double S = 10.0;
    auto patches = Primitives::schwarz_p(0.0, 0.0, 0.0, S);

    MINI_CHECK(patches.size() == 48);
    for (size_t f = 0; f < 48; f++) {
        MINI_CHECK(patches[f].is_valid());
        MINI_CHECK(patches[f].is_rational() == false);
        MINI_CHECK(patches[f].degree(0) == 2);
        MINI_CHECK(patches[f].degree(1) == 2);
        MINI_CHECK(patches[f].cv_count(0) == 3);
        MINI_CHECK(patches[f].cv_count(1) == 3);
    }

    const double PI2 = 2.0 * 3.14159265358979323846;
    double max_err = 0.0;
    for (size_t f = 0; f < 48; f++) {
        for (int i = 0; i <= 4; i++) {
            double u = i / 4.0;
            for (int j = 0; j <= 4; j++) {
                double v = j / 4.0;
                Point p = patches[f].point_at(u, v);
                double val = std::cos(PI2 * p[0] / S) + std::cos(PI2 * p[1] / S) + std::cos(PI2 * p[2] / S);
                double err = std::abs(val);
                if (err > max_err) max_err = err;
            }
        }
    }
    MINI_CHECK(max_err < 0.15);

    Point mid = patches[0].point_at(0.5, 0.5);
    double mid_val = std::cos(PI2 * mid[0] / S) + std::cos(PI2 * mid[1] / S) + std::cos(PI2 * mid[2] / S);
    MINI_CHECK(std::abs(mid_val) < 0.15);
}

///////////////////////////////////////////////////////////////////////////////////////////
// NurbsSurface factory methods
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Primitives", "Nurbssurface_ruled") {
    // uncomment #include "nurbssurface.h"
    std::vector<Point> pts_a = {Point(3,0,0), Point(-2,0,5)};
    std::vector<Point> pts_b = {Point(3,5,5), Point(-2,5,0)};
    NurbsCurve crvA = NurbsCurve::create(false, 1, pts_a);
    NurbsCurve crvB = NurbsCurve::create(false, 1, pts_b);
    NurbsSurface srf = Primitives::create_ruled(crvA, crvB);
    srf.name = "ruled";

    Mesh m = srf.mesh(45);

    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.degree(0) == 1);
    MINI_CHECK(srf.degree(1) == 1);
    MINI_CHECK(srf.cv_count(0) == 2);
    MINI_CHECK(srf.cv_count(1) == 2);

    auto [rd, ruv] = srf.divide_by_count(4, 4);
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

    std::vector<std::pair<double,double>> uvs;
    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++)
            uvs.push_back(ruv[i][j]);

    MINI_CHECK(TOLERANCE.is_point_close(pts[0],  Point( 3.00, 0.00, 0.00)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[1],  Point( 3.00, 1.25, 1.25)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[2],  Point( 3.00, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[3],  Point( 3.00, 3.75, 3.75)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[4],  Point( 3.00, 5.00, 5.00)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[5],  Point( 1.75, 0.00, 1.25)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[6],  Point( 1.75, 1.25, 1.875)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[7],  Point( 1.75, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[8],  Point( 1.75, 3.75, 3.125)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[9],  Point( 1.75, 5.00, 3.75)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[10], Point( 0.50, 0.00, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[11], Point( 0.50, 1.25, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[12], Point( 0.50, 2.50, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[13], Point( 0.50, 3.75, 2.50)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[14], Point( 0.50, 5.00, 2.50)));
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

    MINI_CHECK(TOLERANCE.is_vector_close(normals[0],  Vector( 0.577350269189626, -0.577350269189626,  0.577350269189626)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[1],  Vector( 1.0/3.0, -2.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[2],  Vector( 0.0, -0.707106781186547,  0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[3],  Vector(-1.0/3.0, -2.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[4],  Vector(-0.577350269189626, -0.577350269189626,  0.577350269189626)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[5],  Vector( 2.0/3.0, -1.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[6],  Vector( 0.408248290463863, -0.408248290463863,  0.816496580927726)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[7],  Vector( 0.0, -0.447213595499958,  0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[8],  Vector(-0.408248290463863, -0.408248290463863,  0.816496580927726)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[9],  Vector(-2.0/3.0, -1.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[10], Vector( 0.707106781186547,  0.0,  0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[11], Vector( 0.447213595499958,  0.0,  0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[12], Vector( 0.0, 0.0, 1.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[13], Vector(-0.447213595499958,  0.0,  0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[14], Vector(-0.707106781186547,  0.0,  0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[15], Vector( 2.0/3.0, 1.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[16], Vector( 0.408248290463863,  0.408248290463863,  0.816496580927726)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[17], Vector( 0.0, 0.447213595499958,  0.894427190999916)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[18], Vector(-0.408248290463863,  0.408248290463863,  0.816496580927726)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[19], Vector(-2.0/3.0, 1.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[20], Vector( 0.577350269189626,  0.577350269189626,  0.577350269189626)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[21], Vector( 1.0/3.0, 2.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[22], Vector( 0.0, 0.707106781186547,  0.707106781186547)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[23], Vector(-1.0/3.0, 2.0/3.0, 2.0/3.0)));
    MINI_CHECK(TOLERANCE.is_vector_close(normals[24], Vector(-0.577350269189626,  0.577350269189626,  0.577350269189626)));

    MINI_CHECK(TOLERANCE.is_close(uvs[0].first,  0.00) && TOLERANCE.is_close(uvs[0].second,  0.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[1].first,  0.00) && TOLERANCE.is_close(uvs[1].second,  0.25));
    MINI_CHECK(TOLERANCE.is_close(uvs[4].first,  0.00) && TOLERANCE.is_close(uvs[4].second,  1.00));
    MINI_CHECK(TOLERANCE.is_close(uvs[6].first,  0.25) && TOLERANCE.is_close(uvs[6].second,  0.25));
    MINI_CHECK(TOLERANCE.is_close(uvs[12].first, 0.50) && TOLERANCE.is_close(uvs[12].second, 0.50));
    MINI_CHECK(TOLERANCE.is_close(uvs[24].first, 1.00) && TOLERANCE.is_close(uvs[24].second, 1.00));
}

MINI_TEST("Primitives", "Nurbssurface_planar"){
    // uncomment #include "nurbssurface.h"
    double c1=std::cos(0.7), s1=std::sin(0.7);
    double c2=std::cos(0.96), s2=std::sin(0.96);
    double c3=std::cos(0.52), s3=std::sin(0.52);
    double c4=std::cos(1.13), s4=std::sin(1.13);

    auto ca = NurbsCurve::create(false, 1, {
        Point(0,0,0),
        Point(4,0,0),
        Point(4,3*c1,3*s1),
        Point(0,3*c1,3*s1),
        Point(0,0,0)});
    auto s_quad = Primitives::create_planar(ca);
    s_quad.name = "quad";
    auto m_quad = s_quad.mesh();

    auto cb1 = NurbsCurve::create(false, 1, {
        Point(8,0,0),
        Point(8+5*c2,0,5*s2),
        Point(8+2*c2,3,2*s2),
        Point(8,0,0)});
    auto s_triangle = Primitives::create_planar(cb1);
    s_triangle.name = "triangle";
    auto m_triangle = s_triangle.mesh();

    double ox=18;
    auto cb2 = NurbsCurve::create(false, 1, {
        Point(ox+0*c3,0*s3,0),
        Point(ox+4*c3,4*s3,0),
        Point(ox+5*c3-2*s3,5*s3+2*c3,0),
        Point(ox+3*c3-4*s3,3*s3+4*c3,0),
        Point(ox-1*c3-3*s3,-1*s3+3*c3,0),
        Point(ox+0*c3,0*s3,0)});
    auto s_polygon = Primitives::create_planar(cb2);
    s_polygon.name = "polygon";
    auto m_polygon = s_polygon.mesh();

    auto cc = NurbsCurve::create(false, 3, {
        Point(26,0,0),
        Point(29,1*c4,1*s4),
        Point(31,0.5*c4,0.5*s4),
        Point(32,3*c4,3*s4),
        Point(30,5*c4,5*s4),
        Point(27,4*c4,4*s4),
        Point(26,0,0)});
    auto s_nurbs = Primitives::create_planar(cc);
    s_nurbs.name = "nurbs";
    auto m_nurbs = s_nurbs.mesh();

    MINI_CHECK(s_quad.is_valid());
    MINI_CHECK(s_quad.is_planar());
    MINI_CHECK(s_quad.cv_count(0) == 2);
    MINI_CHECK(s_quad.cv_count(1) == 2);
    MINI_CHECK(m_quad.number_of_vertices() == 4);
    MINI_CHECK(m_quad.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(0,0), Point(0.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(0,1), Point(0.0, 2.294526561853465, 1.932653061713073)));
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(1,0), Point(4.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_quad.get_cv(1,1), Point(4.0, 2.294526561853465, 1.932653061713073)));

    MINI_CHECK(s_triangle.is_valid());
    MINI_CHECK(s_triangle.is_planar());
    MINI_CHECK(s_triangle.cv_count(0) == 2);
    MINI_CHECK(s_triangle.cv_count(1) == 2);
    MINI_CHECK(m_triangle.number_of_vertices() == 3);
    MINI_CHECK(m_triangle.number_of_faces() == 1);
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(0,0), Point(8.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(0,1), Point(8.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(1,0), Point(10.867599930362283, 0.0, 4.095957841504991)));
    MINI_CHECK(TOLERANCE.is_point_close(s_triangle.get_cv(1,1), Point(9.147039972144913, 3.0, 1.638383136601997)));

    MINI_CHECK(s_polygon.is_valid());
    MINI_CHECK(s_polygon.is_planar());
    MINI_CHECK(s_polygon.cv_count(0) == 2);
    MINI_CHECK(s_polygon.cv_count(1) == 2);
    MINI_CHECK(m_polygon.number_of_vertices() == 4);
    MINI_CHECK(m_polygon.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(0,0), Point(19.673777861921977, 6.364048611360808, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(0,1), Point(22.915428262469927, 2.987233669553135, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(1,0), Point(15.247175891573059, 2.114631246911942, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_polygon.get_cv(1,1), Point(18.488826292121008, -1.262183694895731, 0.0)));

    MINI_CHECK(s_nurbs.is_valid());
    MINI_CHECK(s_nurbs.is_planar());
    MINI_CHECK(s_nurbs.cv_count(0) == 2);
    MINI_CHECK(s_nurbs.cv_count(1) == 2);
    MINI_CHECK(m_nurbs.number_of_vertices() == 4);
    MINI_CHECK(m_nurbs.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_nurbs.get_cv(0,0), Point(26.652846559932474, -0.727774577493594, -1.542700265577809)));
    MINI_CHECK(TOLERANCE.is_point_close(s_nurbs.get_cv(0,1), Point(24.347485651711366, 0.916607409071279, 1.942978687541882)));
    MINI_CHECK(TOLERANCE.is_point_close(s_nurbs.get_cv(1,0), Point(32.606791655643732, 0.791738725121784, 1.678288276735475)));
    MINI_CHECK(TOLERANCE.is_point_close(s_nurbs.get_cv(1,1), Point(30.301430747422629, 2.436120711686657, 5.163967229855166)));
}

MINI_TEST("Primitives", "Nurbssurface_extrusion") {
    // uncomment #include "nurbssurface.h"
    Vector dir(0, 1, 5);

    auto c1 = NurbsCurve::create(false, 1, {Point(13,0,0), Point(18,0,0)});
    auto s_line = Primitives::create_extrusion(c1, dir);
    s_line.name = "line";
    auto m_line = s_line.mesh();

    auto c2 = Primitives::circle(24, 0, 0, 3.0);
    auto s_circle = Primitives::create_extrusion(c2, dir);
    s_circle.name = "circle";
    auto m_circle = s_circle.mesh();

    auto c3 = NurbsCurve::create(false, 2, {Point(30,0,0), Point(33,5,0), Point(37,0,0)});
    auto s_arc = Primitives::create_extrusion(c3, dir);
    s_arc.name = "arc";
    auto m_arc = s_arc.mesh();

    auto c4 = NurbsCurve::create(false, 1, {Point(40,3,0), Point(45,0,0), Point(50,3,0), Point(55,0,0)});
    auto s_wavy = Primitives::create_extrusion(c4, dir);
    s_wavy.name = "wavy";
    auto m_wavy = s_wavy.mesh();

    MINI_CHECK(s_line.is_valid());
    MINI_CHECK(s_line.degree(0) == 1 && s_line.degree(1) == 1);
    MINI_CHECK(s_line.cv_count(0) == 2 && s_line.cv_count(1) == 2);
    MINI_CHECK(m_line.number_of_vertices() == 4);
    MINI_CHECK(m_line.number_of_faces() == 2);
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(0,0), Point(13.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(0,1), Point(13.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(1,0), Point(18.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_line.get_cv(1,1), Point(18.0, 1.0, 5.0)));

    MINI_CHECK(s_circle.is_valid());
    MINI_CHECK(s_circle.degree(0) == 2 && s_circle.degree(1) == 1);
    MINI_CHECK(s_circle.is_rational());
    MINI_CHECK(s_circle.is_closed(0) == true && s_circle.is_closed(1) == false);
    MINI_CHECK(s_circle.cv_count(0) == 9 && s_circle.cv_count(1) == 2);
    MINI_CHECK(m_circle.number_of_vertices() == 40);
    MINI_CHECK(m_circle.number_of_faces() == 40);
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(0,0), Point(27.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(0,1), Point(27.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(4,0), Point(21.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(4,1), Point(21.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(8,0), Point(27.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_circle.get_cv(8,1), Point(27.0, 1.0, 5.0)));

    MINI_CHECK(s_arc.is_valid());
    MINI_CHECK(s_arc.degree(0) == 2 && s_arc.degree(1) == 1);
    MINI_CHECK(s_arc.cv_count(0) == 3 && s_arc.cv_count(1) == 2);
    MINI_CHECK(m_arc.number_of_vertices() == 16);
    MINI_CHECK(m_arc.number_of_faces() == 14);
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(0,0), Point(30.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(0,1), Point(30.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(1,0), Point(33.0, 5.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(1,1), Point(33.0, 6.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(2,0), Point(37.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_arc.get_cv(2,1), Point(37.0, 1.0, 5.0)));

    MINI_CHECK(s_wavy.is_valid());
    MINI_CHECK(s_wavy.degree(0) == 1 && s_wavy.degree(1) == 1);
    MINI_CHECK(s_wavy.cv_count(0) == 4 && s_wavy.cv_count(1) == 2);
    MINI_CHECK(m_wavy.number_of_vertices() == 8);
    MINI_CHECK(m_wavy.number_of_faces() == 6);
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(0,0), Point(40.0, 3.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(0,1), Point(40.0, 4.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(1,0), Point(45.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(1,1), Point(45.0, 1.0, 5.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(3,0), Point(55.0, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_wavy.get_cv(3,1), Point(55.0, 1.0, 5.0)));
}

MINI_TEST("Primitives", "Nurbssurface_loft") {
    // uncomment #include "nurbssurface.h"
    NurbsCurve c1 = Primitives::circle(0, 0, 0.0, 2.0);
    NurbsCurve c2 = Primitives::circle(0, 0, 2.0, 1.0);
    NurbsCurve c3 = Primitives::circle(0, 0, 4.0, 1.5);
    NurbsCurve c4 = Primitives::circle(0, 0, 6.0, 0.8);

    NurbsSurface srf = Primitives::create_loft({c1, c2, c3, c4}, 3);
    srf.name = "loft";

    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.cv_count(0) == 9);
    MINI_CHECK(srf.cv_count(1) == 4);
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(0, 0), Point(2, 0, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(0, 1), Point(-0.677194251158421, 0, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(0, 2), Point(3.00619893067415, 0, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(0, 3), Point(0.8, 0, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(1, 0), Point(2, 2, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(1, 1), Point(-0.677194251158421, -0.677194251158421, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(1, 2), Point(3.00619893067414, 3.00619893067414, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(1, 3), Point(0.8, 0.8, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(2, 0), Point(0, 2, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(2, 1), Point(0, -0.677194251158421, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(2, 2), Point(0, 3.00619893067415, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(2, 3), Point(0, 0.8, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(3, 0), Point(-2, 2, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(3, 1), Point(0.677194251158421, -0.677194251158421, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(3, 2), Point(-3.00619893067414, 3.00619893067414, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(3, 3), Point(-0.8, 0.8, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(4, 0), Point(-2, 0, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(4, 1), Point(0.677194251158421, 0, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(4, 2), Point(-3.00619893067415, 0, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(4, 3), Point(-0.8, 0, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(5, 0), Point(-2, -2, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(5, 1), Point(0.677194251158421, 0.677194251158421, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(5, 2), Point(-3.00619893067414, -3.00619893067414, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(5, 3), Point(-0.8, -0.8, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(6, 0), Point(0, -2, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(6, 1), Point(0, 0.677194251158421, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(6, 2), Point(0, -3.00619893067415, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(6, 3), Point(0, -0.8, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(7, 0), Point(2, -2, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(7, 1), Point(-0.677194251158421, 0.677194251158421, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(7, 2), Point(3.00619893067414, -3.00619893067414, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(7, 3), Point(0.8, -0.8, 6)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(8, 0), Point(2, 0, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(8, 1), Point(-0.677194251158421, 0, 1.75222035185728)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(8, 2), Point(3.00619893067415, 0, 4.08030037218547)));
    MINI_CHECK(TOLERANCE.is_point_close(srf.get_cv(8, 3), Point(0.8, 0, 6)));

    std::vector<std::vector<Point>> open_pts = {
        {Point(10, -12, 0), Point(10, -10, 3), Point(10, -7, 3), Point(10, -5, 0)},
        {Point(5.5, -12, 3.5), Point(5.5, -10.0, 1.5), Point(5.5, -7.0, 1.5), Point(5.5, -5, 3.5)},
        {Point(1, -12, 0), Point(1, -10, 3.0), Point(1, -7, 3.0), Point(1, -5, 0)},
    };
    std::vector<NurbsCurve> open_curves = {
        NurbsCurve::create(false, 3, open_pts[0]),
        NurbsCurve::create(false, 3, open_pts[1]),
        NurbsCurve::create(false, 3, open_pts[2]),
    };
    NurbsSurface open_srf = Primitives::create_loft(open_curves, 3);
    open_srf.name = "open_loft";

    MINI_CHECK(open_srf.is_valid());
    MINI_CHECK(open_srf.cv_count(0) == 4);
    MINI_CHECK(open_srf.cv_count(1) == 3);

    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(0, 0), Point(10, -12, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(0, 1), Point(5.5, -12, 7)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(0, 2), Point(1, -12, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(1, 0), Point(10, -10, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(1, 1), Point(5.5, -10, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(1, 2), Point(1, -10, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(2, 0), Point(10, -7, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(2, 1), Point(5.5, -7, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(2, 2), Point(1, -7, 3)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(3, 0), Point(10, -5, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(3, 1), Point(5.5, -5, 7)));
    MINI_CHECK(TOLERANCE.is_point_close(open_srf.get_cv(3, 2), Point(1, -5, 0)));
}

MINI_TEST("Primitives", "Nurbssurface_revolve"){
    // uncomment #include "nurbssurface.h"
    auto pa = NurbsCurve::create(false, 3, {
        Point(1.5, 0, 0), Point(1.5, 0, 0.3), Point(0.3, 0, 0.5),
        Point(0.3, 0, 2.5), Point(0.2, 0, 3.0), Point(2.0, 0, 4.5), Point(1.8, 0, 5.0)});
    auto s_vase = Primitives::create_revolve(pa, Point(0,0,0), Vector(0,0,1));
    s_vase.name = "vase";
    auto m_vase = s_vase.mesh();

    NurbsCurve pb(3, true, 3, 9);
    const double w = std::sqrt(2.0) / 2.0;
    double cw[] = {1, w, 1, w, 1, w, 1, w, 1};
    double ca[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double sa[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double ck[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    double R = 5.0, r = 1.5, tcx = 14;
    for (int i = 0; i < 10; i++) pb.set_knot(i, ck[i]);
    for (int i = 0; i < 9; i++)
        pb.set_cv_4d(i, (tcx + R + r * ca[i]) * cw[i], 0, r * sa[i] * cw[i], cw[i]);
    auto s_torus = Primitives::create_revolve(pb, Point(tcx,0,0), Vector(0,0,1));
    s_torus.name = "torus";
    auto m_torus = s_torus.mesh();

    auto pc = NurbsCurve::create(false, 1, {Point(29, 0, -0.5), Point(29, 0, 0.5)});
    auto s_elbow = Primitives::create_revolve(pc, Point(26,0,0), Vector(0,0,1), Tolerance::PI / 2.0);
    s_elbow.name = "elbow";
    auto m_elbow = s_elbow.mesh();

    double sr = 2.0, scx = 36;
    NurbsCurve pd(3, true, 3, 5);
    double sk[] = {0, 0, 1, 1, 2, 2};
    for (int i = 0; i < 6; i++) pd.set_knot(i, sk[i]);
    double spx[] = {0, sr, sr, sr, 0}, spz[] = {-sr, -sr, 0, sr, sr}, spw[] = {1, w, 1, w, 1};
    for (int i = 0; i < 5; i++)
        pd.set_cv_4d(i, (scx + spx[i]) * spw[i], 0, spz[i] * spw[i], spw[i]);
    auto s_sphere = Primitives::create_revolve(pd, Point(scx,0,0), Vector(0,0,1));
    s_sphere.name = "sphere";
    auto m_sphere = s_sphere.mesh();

    auto pe = NurbsCurve::create(false, 1, {Point(44, 0, 3), Point(46, 0, 0)});
    auto s_cone = Primitives::create_revolve(pe, Point(44,0,0), Vector(0,0,1));
    s_cone.name = "cone";
    auto m_cone = s_cone.mesh();

    MINI_CHECK(s_vase.is_valid());
    MINI_CHECK(s_vase.is_closed(0) == true);
    MINI_CHECK(s_vase.is_closed(1) == false);
    MINI_CHECK(s_vase.cv_count(0) == 9);
    MINI_CHECK(s_vase.cv_count(1) == 7);
    MINI_CHECK(m_vase.number_of_vertices() == 660);
    MINI_CHECK(m_vase.number_of_faces() == 1280);
    MINI_CHECK(TOLERANCE.is_point_close(s_vase.get_cv(0,0), Point(1.5, 0.0, 0.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_vase.get_cv(0,6), Point(1.8, 0.0, 5.0)));

    MINI_CHECK(s_torus.is_valid());
    MINI_CHECK(s_torus.is_closed(0) == true);
    MINI_CHECK(s_torus.is_closed(1) == true);
    MINI_CHECK(s_torus.cv_count(0) == 9);
    MINI_CHECK(s_torus.cv_count(1) == 9);
    MINI_CHECK(m_torus.number_of_vertices() == 640);
    MINI_CHECK(m_torus.number_of_faces() == 1280);
    MINI_CHECK(TOLERANCE.is_point_close(s_torus.get_cv(0,0), Point(20.5, 0.0, 0.0)));

    MINI_CHECK(s_elbow.is_valid());
    MINI_CHECK(s_elbow.is_closed(0) == false);
    MINI_CHECK(s_elbow.is_closed(1) == false);
    MINI_CHECK(s_elbow.cv_count(0) == 3);
    MINI_CHECK(s_elbow.cv_count(1) == 2);
    MINI_CHECK(m_elbow.number_of_vertices() == 16);
    MINI_CHECK(m_elbow.number_of_faces() == 14);
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(0,0), Point(29.0, 0.0, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(0,1), Point(29.0, 0.0, 0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(2,0), Point(26.0, 3.0, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(s_elbow.get_cv(2,1), Point(26.0, 3.0, 0.5)));

    MINI_CHECK(s_sphere.is_valid());
    MINI_CHECK(s_sphere.is_closed(0) == true);
    MINI_CHECK(s_sphere.is_closed(1) == false);
    MINI_CHECK(s_sphere.is_singular(0) == true);
    MINI_CHECK(s_sphere.is_singular(2) == true);
    MINI_CHECK(s_sphere.cv_count(0) == 9);
    MINI_CHECK(s_sphere.cv_count(1) == 5);
    MINI_CHECK(m_sphere.number_of_vertices() == 322);
    MINI_CHECK(m_sphere.number_of_faces() == 640);
    MINI_CHECK(TOLERANCE.is_point_close(s_sphere.get_cv(0,0), Point(36.0, 0.0, -2.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sphere.get_cv(0,4), Point(36.0, 0.0, 2.0)));

    MINI_CHECK(s_cone.is_valid());
    MINI_CHECK(s_cone.is_closed(0) == true);
    MINI_CHECK(s_cone.is_closed(1) == false);
    MINI_CHECK(s_cone.is_singular(0) == true);
    MINI_CHECK(s_cone.is_singular(2) == false);
    MINI_CHECK(s_cone.cv_count(0) == 9);
    MINI_CHECK(s_cone.cv_count(1) == 2);
    MINI_CHECK(m_cone.number_of_vertices() == 21);
    MINI_CHECK(m_cone.number_of_faces() == 20);
    MINI_CHECK(TOLERANCE.is_point_close(s_cone.get_cv(0,0), Point(44.0, 0.0, 3.0)));
    MINI_CHECK(TOLERANCE.is_point_close(s_cone.get_cv(0,1), Point(46.0, 0.0, 0.0)));
}

MINI_TEST("Primitives", "Nurbssurface_sweep") {
    // uncomment #include "nurbssurface.h"
    NurbsCurve rail = NurbsCurve::create(false, 2, {Point(0,0,0), Point(0,5,0), Point(2,9,0)});
    NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
    NurbsSurface s_sweep1 = Primitives::create_sweep1(rail, profile);
    s_sweep1.name = "sweep1";
    auto m_sweep1 = s_sweep1.mesh();

    NurbsCurve rail1 = NurbsCurve::create(false, 2, {Point(6,-1,0), Point(7,3,0), Point(8,4,0)});
    NurbsCurve rail2 = NurbsCurve::create(false, 2, {Point(10,-1,0), Point(10,3,0), Point(9,4,0)});
    NurbsCurve shape1 = NurbsCurve::create(false, 2, {Point(6,-1,0), Point(8,-1,2), Point(10,-1,0)});
    NurbsCurve shape2 = NurbsCurve::create(false, 2, {Point(8,4,0), Point(8.5,4,1.5), Point(9,4,0)});
    NurbsSurface s_sweep2 = Primitives::create_sweep2(rail1, rail2, {shape1, shape2});
    s_sweep2.name = "sweep2";
    auto m_sweep2 = s_sweep2.mesh();

    MINI_CHECK(s_sweep1.is_valid());
    MINI_CHECK(s_sweep1.is_rational());
    MINI_CHECK(s_sweep1.cv_count(0) == 9);
    MINI_CHECK(s_sweep1.cv_count(1) == 6);
    MINI_CHECK(m_sweep1.number_of_vertices() > 0);
    MINI_CHECK(m_sweep1.number_of_faces() > 0);
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(0,0), Point(0.888888888888889, 0.000000000000000, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(0,1), Point(0.888635792881381, 1.202714517481950, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(0,2), Point(1.024939251342349, 2.995270183326687, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(0,3), Point(1.646130147308625, 5.890456645823520, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(0,4), Point(2.268126490080245, 7.550043751484516, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(0,5), Point(2.795046402150731, 8.602476824301650, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(1,0), Point(0.888888888888889, 0.000000000000000, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(1,1), Point(0.888635792881381, 1.202714517481950, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(1,2), Point(1.024939251342349, 2.995270183326688, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(1,3), Point(1.646130147308625, 5.890456645823518, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(1,4), Point(2.268126490080246, 7.550043751484516, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(1,5), Point(2.795046402150731, 8.602476824301650, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(2,0), Point(-0.111111111111111, 0.000000000000000, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(2,1), Point(-0.111375650381527, 1.252481986115285, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(2,2), Point(0.030563435078404, 3.128596097925310, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(2,3), Point(0.684572492801853, 6.178215457853874, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(2,4), Point(1.342568351103634, 7.935308821507740, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(2,5), Point(1.900619199731159, 9.049690396962294, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(3,0), Point(-1.111111111111111, 0.000000000000000, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(3,1), Point(-1.111387093644435, 1.302249454748620, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(3,2), Point(-0.963812381185542, 3.261922012523934, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(3,3), Point(-0.276985161704920, 6.465974269884224, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(3,4), Point(0.417010212127023, 8.320573891530971, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(3,5), Point(1.006191997311586, 9.496903969622938, -1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(4,0), Point(-1.111111111111111, 0.000000000000000, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(4,1), Point(-1.111387093644435, 1.302249454748620, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(4,2), Point(-0.963812381185542, 3.261922012523933, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(4,3), Point(-0.276985161704920, 6.465974269884226, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(4,4), Point(0.417010212127023, 8.320573891530966, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(4,5), Point(1.006191997311586, 9.496903969622938, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(5,0), Point(-1.111111111111111, 0.000000000000000, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(5,1), Point(-1.111387093644435, 1.302249454748620, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(5,2), Point(-0.963812381185542, 3.261922012523934, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(5,3), Point(-0.276985161704920, 6.465974269884224, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(5,4), Point(0.417010212127023, 8.320573891530971, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(5,5), Point(1.006191997311586, 9.496903969622938, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(6,0), Point(-0.111111111111111, 0.000000000000000, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(6,1), Point(-0.111375650381527, 1.252481986115285, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(6,2), Point(0.030563435078404, 3.128596097925310, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(6,3), Point(0.684572492801853, 6.178215457853874, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(6,4), Point(1.342568351103634, 7.935308821507740, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(6,5), Point(1.900619199731159, 9.049690396962294, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(7,0), Point(0.888888888888889, 0.000000000000000, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(7,1), Point(0.888635792881381, 1.202714517481950, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(7,2), Point(1.024939251342349, 2.995270183326688, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(7,3), Point(1.646130147308625, 5.890456645823518, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(7,4), Point(2.268126490080246, 7.550043751484516, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(7,5), Point(2.795046402150731, 8.602476824301650, 1.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(8,0), Point(0.888888888888889, 0.000000000000000, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(8,1), Point(0.888635792881381, 1.202714517481950, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(8,2), Point(1.024939251342349, 2.995270183326687, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(8,3), Point(1.646130147308625, 5.890456645823520, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(8,4), Point(2.268126490080245, 7.550043751484516, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep1.get_cv(8,5), Point(2.795046402150731, 8.602476824301650, 0.000000000000000)));

    MINI_CHECK(s_sweep2.is_valid());
    MINI_CHECK(s_sweep2.cv_count(0) == 3);
    MINI_CHECK(s_sweep2.cv_count(1) == 6);
    MINI_CHECK(m_sweep2.number_of_vertices() > 0);
    MINI_CHECK(m_sweep2.number_of_faces() > 0);
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(0,0), Point(6.000000000000000, -1.000000000000000, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(0,1), Point(6.175891405559444, -0.303365249618322, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(0,2), Point(6.455069225035283, 0.735777178606404, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(0,3), Point(7.037890496541333, 2.410962615700823, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(0,4), Point(7.483820048623357, 3.410830407841271, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(0,5), Point(8.000000000000000, 4.000000000000000, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(1,0), Point(7.999999999999999, -1.000000000000000, 1.999999999999999)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(1,1), Point(8.086850261873641, -0.308423766638977, 2.038904142544954)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(1,2), Point(8.213847160672065, 0.726655146723519, 2.130856460476301)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(1,3), Point(8.397280127738487, 2.404595750448798, 2.196607499777196)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(1,4), Point(8.486313675857327, 3.416340095667512, 2.255660179912791)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(1,5), Point(8.500000000000000, 4.000000000000000, 1.499999999999997)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(2,0), Point(9.999999999999998, -1.000000000000000, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(2,1), Point(9.997809118187826, -0.313482283659631, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(2,2), Point(9.972625096308857, 0.717533114840633, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(2,3), Point(9.756669758935631, 2.398228885196774, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(2,4), Point(9.488807303091296, 3.421849783493755, 0.000000000000000)));
    MINI_CHECK(TOLERANCE.is_point_close(s_sweep2.get_cv(2,5), Point(8.999999999999998, 4.000000000000000, 0.000000000000000)));
}

MINI_TEST("Primitives", "Nurbssurface_edge") {
    // uncomment #include "nurbssurface.h"
    std::vector<Point> pts_south = {Point(1, 20.569076, 0), Point(1, 22.569076, 3.0), Point(1, 25.569076, 3.0), Point(1, 27.569076, 0)};
    std::vector<Point> pts_west  = {Point(10, 20.569076, 0), Point(5.5, 20.569076, 3.5), Point(1, 20.569076, 0)};
    std::vector<Point> pts_north = {Point(10, 20.569076, 0), Point(10, 22.569076, 3), Point(10, 25.569076, 3), Point(10, 27.569076, 0)};
    std::vector<Point> pts_east  = {Point(10, 27.569076, 0), Point(5.5, 27.569076, 3.5), Point(1, 27.569076, 0)};

    NurbsCurve south = NurbsCurve::create(false, 3, pts_south);
    NurbsCurve west  = NurbsCurve::create(false, 2, pts_west);
    NurbsCurve north = NurbsCurve::create(false, 3, pts_north);
    NurbsCurve east  = NurbsCurve::create(false, 2, pts_east);

    NurbsSurface surf = Primitives::create_edge(south, west, north, east);
    surf.name = "edge";
    Mesh m = surf.mesh(15);

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

///////////////////////////////////////////////////////////////////////////////////////////
// Surface-to-mesh subdivision
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Primitives", "Mesh_quad_mesh") {
    NurbsSurface srf = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::quad_mesh(srf, 8, 4);

    MINI_CHECK(m.number_of_vertices() == 45);
    MINI_CHECK(m.number_of_faces() == 32);
    MINI_CHECK(m.is_valid());
}

MINI_TEST("Primitives", "Mesh_diamond_mesh") {
    NurbsSurface srf = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::diamond_mesh(srf, 8, 4);

    MINI_CHECK(m.number_of_vertices() == 45);
    MINI_CHECK(m.number_of_faces() == 23);
    MINI_CHECK(m.is_valid());
}

MINI_TEST("Primitives", "Mesh_hex_mesh") {
    NurbsSurface srf = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::hex_mesh(srf, 6, 4, 1.0/3.0);

    MINI_CHECK(m.number_of_vertices() == 91);
    MINI_CHECK(m.number_of_faces() == 32);
    MINI_CHECK(m.is_valid());
}

MINI_TEST("Primitives", "Mesh_hex_mesh2") {
    NurbsSurface srf = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = Primitives::hex_mesh2(srf, 6, 4, 2.0/3.0);

    MINI_CHECK(m.number_of_vertices() == 91);
    MINI_CHECK(m.number_of_faces() == 54);
    MINI_CHECK(m.is_valid());
}

MINI_TEST("Primitives", "Nurbscurve_interpolated") {
    // uncomment #include "nurbscurve.h"
    std::vector<Point> points = {
        Point(14, 9, 0), Point(15.342777, 13.734889, 0), Point(21.897914, 32.239195, 0),
        Point(24.678472, 0.354555, 0), Point(33.813678, 24.76858, 0),
        Point(39.626394, 15.47249, 0), Point(41, 13, 0)
    };

    NurbsCurve c = Primitives::create_interpolated(points, CurveKnotStyle::Chord);

    MINI_CHECK(c.is_valid());
    MINI_CHECK(c.degree() == 3);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.is_rational() == false);

    // Verify curve passes through all input points
    auto [d0, d1] = c.domain();
    std::vector<double> knots = c.get_knots();
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d0), points[0]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(knots[3]), points[1]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(knots[4]), points[2]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(knots[5]), points[3]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(knots[6]), points[4]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(knots[7]), points[5]));
    MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d1), points[6]));

    // Verify endpoints are exact
    MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(0), points[0]));
    MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(8), points[6]));

    // Test with 4 points
    std::vector<Point> pts4 = {Point(0,0,0), Point(1,2,0), Point(3,1,0), Point(5,3,0)};
    NurbsCurve c4 = Primitives::create_interpolated(pts4, CurveKnotStyle::Chord);
    MINI_CHECK(c4.is_valid());
    MINI_CHECK(c4.degree() == 3);
    MINI_CHECK(c4.cv_count() == 6);
    auto [d4_0, d4_1] = c4.domain();
    MINI_CHECK(TOLERANCE.is_point_close(c4.point_at(d4_0), pts4[0]));
    MINI_CHECK(TOLERANCE.is_point_close(c4.point_at(d4_1), pts4[3]));
}

} // namespace session_cpp
