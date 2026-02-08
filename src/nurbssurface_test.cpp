#include "mini_test.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "trimesh_grid.h"
#include "trimesh_delaunay.h"
#include "mesh.h"
#include "color.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "primitives.h"

#include <cmath>
#include <filesystem>
 #include <sstream>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("NurbsSurface", "constructor") {
        // uncomment #include "nurbssurface.h"

        std::vector<Point> points = {
            // i=0
            Point(0.0, 0.0, 0.0),
            Point(-1.0, 0.75, 2.0),
            Point(-1.0, 4.25, 2.0),
            Point(0.0, 5.0, 0.0),
            // i=1
            Point(0.75, -1.0, 2.0),
            Point(1.25, 1.25, 4.0),
            Point(1.25, 3.75, 4.0),
            Point(0.75, 6.0, 2.0),
            // i=2
            Point(4.25, -1.0, 2.0),
            Point(3.75, 1.25, 4.0),
            Point(3.75, 3.75, 4.0),
            Point(4.25, 6.0, 2.0),
            // i=3
            Point(5.0, 0.0, 0.0),
            Point(6.0, 0.75, 2.0),
            Point(6.0, 4.25, 2.0),
            Point(5.0, 5.0, 0.0),
        };

        NurbsSurface s = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

        // Get mesh
        Mesh m = s.mesh();

        // Point division matching Rhino's 4x6 grid
        auto [v, uv] = s.divide_by_count(4, 6);

        // Minimal and Full String Representation
        std::string sstr = s.str();
        std::string srepr = s.repr();

        // Copy (duplicates everything except guid)
        NurbsSurface scopy = s;
        NurbsSurface sother = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

        MINI_CHECK(s.is_valid() == true);
        MINI_CHECK(s.cv_count(0) == 4);
        MINI_CHECK(s.cv_count(1) == 4);
        MINI_CHECK(s.cv_count() == 16);
        MINI_CHECK(s.degree(0) == 3);
        MINI_CHECK(s.degree(1) == 3);
        MINI_CHECK(s.order(0) == 4);
        MINI_CHECK(s.order(1) == 4);
        MINI_CHECK(s.dimension() == 3);
        MINI_CHECK(!s.is_rational());
        MINI_CHECK(s.knot_count(0) == 6);
        MINI_CHECK(s.knot_count(1) == 6);
        MINI_CHECK(s.name == "my_nurbssurface");
        MINI_CHECK(!s.guid.empty());
        MINI_CHECK(sstr == "NurbsSurface(name=my_nurbssurface, degree=(3,3), cvs=(4,4))");
        MINI_CHECK(srepr == "NurbsSurface(\n  name=my_nurbssurface,\n  degree=(3,3),\n  cvs=(4,4),\n  rational=false,\n  control_points=[\n    0, 0, 0\n    -1, 0.75, 2\n    -1, 4.25, 2\n    0, 5, 0\n    0.75, -1, 2\n    1.25, 1.25, 4\n    1.25, 3.75, 4\n    0.75, 6, 2\n    4.25, -1, 2\n    3.75, 1.25, 4\n    3.75, 3.75, 4\n    4.25, 6, 2\n    5, 0, 0\n    6, 0.75, 2\n    6, 4.25, 2\n    5, 5, 0\n  ]\n)");
        MINI_CHECK(scopy.cv_count() == s.cv_count());
        MINI_CHECK(scopy.guid != s.guid);

        MINI_CHECK(TOLERANCE.is_point_close(v[0][0], Point(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[0][1], Point(-0.416666666666667, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[0][2], Point(-0.666666666666667, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[0][3], Point(-0.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[0][4], Point(-0.666666666666667, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[0][5], Point(-0.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[0][6], Point(0.000000000000000, 5.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][0], Point(0.992187500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][1], Point(0.881510416666667, 0.333912037037037, 1.958333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][2], Point(0.815104166666667, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][3], Point(0.792968750000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][4], Point(0.815104166666667, 3.620370370370370, 2.458333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][5], Point(0.881510416666667, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[1][6], Point(0.992187500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][0], Point(2.500000000000000, -0.750000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][1], Point(2.500000000000000, 0.252314814814815, 2.333333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][2], Point(2.500000000000000, 1.351851851851852, 2.833333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][3], Point(2.500000000000000, 2.500000000000000, 3.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][4], Point(2.500000000000000, 3.648148148148148, 2.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][5], Point(2.500000000000000, 4.747685185185186, 2.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[2][6], Point(2.500000000000000, 5.750000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][0], Point(4.007812500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][1], Point(4.118489583333334, 0.333912037037037, 1.958333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][2], Point(4.184895833333334, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][3], Point(4.207031250000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][4], Point(4.184895833333333, 3.620370370370370, 2.458333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][5], Point(4.118489583333333, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[3][6], Point(4.007812500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][0], Point(5.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][1], Point(5.416666666666668, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][2], Point(5.666666666666668, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][3], Point(5.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][4], Point(5.666666666666666, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][5], Point(5.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(v[4][6], Point(5.000000000000000, 5.000000000000000, 0.000000000000000)));
    }

    MINI_TEST("NurbsSurface", "constructor_ruled") {
        // uncomment #include "nurbssurface.h"

        std::vector<Point> pts_a = {Point(3,0,0), Point(-2,0,5)};
        std::vector<Point> pts_b = {Point(3,5,5), Point(-2,5,0)};
        NurbsCurve crvA = NurbsCurve::create(false, 1, pts_a);
        NurbsCurve crvB = NurbsCurve::create(false, 1, pts_b);
        NurbsSurface srf = NurbsSurface::create_ruled(crvA, crvB);

        // Mesh 180 - lowest quality, 5 - highest quality
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

    MINI_TEST("NurbsSurface", "constructor_loft") {
        // uncomment #include "nurbssurface.h"

        // Closed circles with varying radius and same center
        NurbsCurve c1 = Primitives::circle(0, 0, 0.0, 2.0);
        NurbsCurve c2 = Primitives::circle(0, 0, 2.0, 1.0);
        NurbsCurve c3 = Primitives::circle(0, 0, 4.0, 1.5);
        NurbsCurve c4 = Primitives::circle(0, 0, 6.0, 0.8);

        NurbsSurface srf = NurbsSurface::create_loft({c1, c2, c3, c4}, 3);

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

        // Open loft
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
        NurbsSurface open_srf = NurbsSurface::create_loft(open_curves, 3);

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

    MINI_TEST("NurbsSurface", "constructor_revolve") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsCurve profile = NurbsCurve::create(false, 3, {
            Point(1.5, 0, 0), Point(1.5, 0, 0.3), Point(0.3, 0, 0.5),
            Point(0.3, 0, 2.5), Point(0.2, 0, 3.0), Point(2.0, 0, 4.5), Point(1.8, 0, 5.0),
        });
        NurbsSurface srf = NurbsSurface::create_revolve(profile, Point(0,0,0), Vector(0,0,1));
    }

    MINI_TEST("NurbsSurface", "constructor_sweep") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include "primitives.h"

        // Sweep1
        NurbsCurve rail = NurbsCurve::create(false, 2, {
            Point(0,0,0), Point(0,5,0), Point(2,9,0)
        });
        NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
        NurbsSurface srf = NurbsSurface::create_sweep1(rail, profile);

        // Sweep2
        NurbsCurve rail1 = NurbsCurve::create(false, 2, {Point(-2,-1,0), Point(-1,3,0), Point(0,4,0)});
        NurbsCurve rail2 = NurbsCurve::create(false, 2, {Point(2,-1,0), Point(2,3,0), Point(1,4,0)});
        NurbsCurve shape1 = NurbsCurve::create(false, 2, {Point(-2,-1,0), Point(0,-1,2), Point(2,-1,0)});       
        NurbsCurve shape2 = NurbsCurve::create(false, 2, {Point(0,4,0), Point(0.5,4,1.5), Point(1,4,0)});
        srf = NurbsSurface::create_sweep2(rail1, rail2, {shape1, shape2});
    
    }

    MINI_TEST("NurbsSurface", "create_operations") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create a simple 2x2 bilinear surface
        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set corner control points
        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, Point(1.0, 1.0, 0.0));

        // Check knot vectors
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(dom_u.first == 0.0);
        MINI_CHECK(dom_v.first == 0.0);
    }

    MINI_TEST("NurbsSurface", "accessors") {
        // uncomment #include "nurbssurface.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 4, 3, 5, 4, false, false, 1.0, 1.0);

        // Test knot access
        double knot_val = surf.knot(0, 2);

        // Test set knot
        surf.set_knot(0, 2, 5.0);
        double new_val = surf.knot(0, 2);

        MINI_CHECK(surf.dimension() == 3);
        MINI_CHECK(!surf.is_rational());
        MINI_CHECK(surf.order(0) == 4);
        MINI_CHECK(surf.order(1) == 3);
        MINI_CHECK(surf.degree(0) == 3);
        MINI_CHECK(surf.degree(1) == 2);
        MINI_CHECK(surf.cv_count(0) == 5);
        MINI_CHECK(surf.cv_count(1) == 4);
        MINI_CHECK(surf.cv_count() == 20);
        MINI_CHECK(surf.cv_size() == 3);
        MINI_CHECK(surf.knot_count(0) == 7);
        MINI_CHECK(surf.knot_count(1) == 5);
        MINI_CHECK(surf.span_count(0) == 2);
        MINI_CHECK(surf.span_count(1) == 2);
        MINI_CHECK(new_val == 5.0);
    }

    MINI_TEST("NurbsSurface", "knot_operations") {
        // uncomment #include "nurbssurface.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);

        // Verify domain
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);

        MINI_CHECK(dom_u.first == 0.0);
        MINI_CHECK(dom_u.second > dom_u.first);
        MINI_CHECK(dom_v.first == 0.0);
        MINI_CHECK(dom_v.second > dom_v.first);
        MINI_CHECK(surf.is_clamped(0, 0));
        MINI_CHECK(surf.is_clamped(1, 0));
    }

    MINI_TEST("NurbsSurface", "rational_operations") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create non-rational surface
        NurbsSurface surf;
        surf.create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);

        // Make it rational
        surf.make_rational();

        // Set a control point and weight
        surf.set_cv(1, 1, Point(1.0, 2.0, 3.0));
        surf.set_weight(1, 1, 2.0);

        // Verify weight
        double w = surf.weight(1, 1);

        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.cv_size() == 4);
        MINI_CHECK(w == 2.0);
    }

    MINI_TEST("NurbsSurface", "evaluation") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

        // Create simple bilinear surface
        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set corner control points
        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, Point(1.0, 1.0, 0.0));

        // Evaluate at corner
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);

        Point pt_corner = surf.point_at(dom_u.first, dom_v.first);

        // Evaluate at center (should be center of unit square)
        double u_mid = (dom_u.first + dom_u.second) / 2.0;
        double v_mid = (dom_v.first + dom_v.second) / 2.0;
        Point pt_mid = surf.point_at(u_mid, v_mid);

        // Test derivatives
        auto derivs = surf.evaluate(u_mid, v_mid, 1);

        // Test normal (for flat plane in XY, normal should point in +Z)
        Vector normal = surf.normal_at(u_mid, v_mid);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(TOLERANCE.is_close(pt_corner[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_corner[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_corner[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_mid[0], 0.5));
        MINI_CHECK(TOLERANCE.is_close(pt_mid[1], 0.5));
        MINI_CHECK(derivs.size() == 3);
        MINI_CHECK(TOLERANCE.is_close(std::abs(normal[2]), 1.0));
    }

    MINI_TEST("NurbsSurface", "geometric_queries") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create and setup surface
        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, Point(1.0, 1.0, 0.0));

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(!surf.is_closed(0));
        MINI_CHECK(!surf.is_closed(1));
        MINI_CHECK(!surf.is_periodic(0));
        MINI_CHECK(!surf.is_periodic(1));
        MINI_CHECK(surf.is_clamped(0, 0));
        MINI_CHECK(surf.is_clamped(1, 0));
        MINI_CHECK(surf.is_planar(nullptr, 1e-6));
    }

    MINI_TEST("NurbsSurface", "modification") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 3, 2, false, false, 1.0, 1.0);

        // Set some CVs
        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(2, 0, Point(2.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(2, 1, Point(2.0, 1.0, 0.0));

        Point cv_before = surf.get_cv(0, 0);

        // Test reverse in u direction
        surf.reverse(0);
        Point cv_after = surf.get_cv(2, 0);

        // Reverse back
        surf.reverse(0);

        // Test transpose
        int order_u_before = surf.order(0);
        int order_v_before = surf.order(1);
        surf.transpose();

        MINI_CHECK(cv_after[0] == cv_before[0]);
        MINI_CHECK(surf.order(0) == order_v_before);
        MINI_CHECK(surf.order(1) == order_u_before);
    }

    MINI_TEST("NurbsSurface", "isocurve") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create surface
        NurbsSurface surf;
        surf.create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);

        // Set up a grid of control points
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        // Extract iso-u curve (v varies)
        auto dom_u = surf.domain(0);
        double u_mid = (dom_u.first + dom_u.second) / 2.0;
        NurbsCurve* iso_u = surf.iso_curve(0, u_mid);

        // Extract iso-v curve (u varies)
        auto dom_v = surf.domain(1);
        double v_mid = (dom_v.first + dom_v.second) / 2.0;
        NurbsCurve* iso_v = surf.iso_curve(1, v_mid);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(iso_u != nullptr);
        MINI_CHECK(iso_u->is_valid());
        MINI_CHECK(iso_v != nullptr);
        MINI_CHECK(iso_v->is_valid());

        if (iso_u != nullptr) delete iso_u;
        if (iso_v != nullptr) delete iso_v;
    }

    MINI_TEST("NurbsSurface", "transformation") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"

        // Create simple surface
        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, Point(1.0, 1.0, 0.0));

        // Apply translation
        Xform xf = Xform::translation(1.0, 2.0, 3.0);
        surf.transform(xf);

        // Check transformed CV
        Point pt = surf.get_cv(0, 0);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 3.0));
    }

    MINI_TEST("NurbsSurface", "json_roundtrip") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        NurbsSurface surf;
        surf.create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.surfacecolor = Color(255, 128, 64, 255);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        //   jsondump()      │ ordered_json │ to JSON object (internal use)
        //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
        //   json_dumps()    │ std::string  │ to JSON string
        //   json_loads(s)   │ std::string  │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        // JSON object
        nlohmann::ordered_json json = surf.jsondump();
        NurbsSurface loaded_json = NurbsSurface::jsonload(json);

        // String
        std::string json_string = surf.json_dumps();
        NurbsSurface loaded_json_string = NurbsSurface::json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbssurface.json").string();
        surf.json_dump(filename);
        NurbsSurface loaded_from_file = NurbsSurface::json_load(filename);

        MINI_CHECK(loaded_json == surf);
        MINI_CHECK(loaded_json_string == surf);
        MINI_CHECK(loaded_from_file == surf);
    }

    MINI_TEST("NurbsSurface", "protobuf_roundtrip") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        NurbsSurface surf;
        surf.create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.surfacecolor = Color(255, 128, 64, 255);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        //   pb_dumps()      │ string/bytes │ to protobuf bytes
        //   pb_loads(b)     │ string/bytes │ from protobuf bytes
        //   pb_dump(path)   │ file         │ write to file
        //   pb_load(path)   │ file         │ read from file

        // String
        std::string proto_string = surf.pb_dumps();
        NurbsSurface loaded_proto_string = NurbsSurface::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbssurface.bin").string();
        surf.pb_dump(filename);
        NurbsSurface loaded = NurbsSurface::pb_load(filename);

        MINI_CHECK(loaded_proto_string == surf);
        MINI_CHECK(loaded == surf);
    }

    MINI_TEST("NurbsSurface", "advanced_accessors") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create rational surface for testing get_cv_4d/set_cv_4d
        NurbsSurface surf;
        surf.create_raw(3, true, 3, 3, 3, 3, false, false, 1.0, 1.0);

        // Test set_cv_4d with homogeneous coordinates
        double x = 2.0, y = 3.0, z = 4.0, w = 2.0;

        // Set CV using set_cv_4d
        surf.set_cv_4d(1, 1, x, y, z, w);

        // Get CV and verify using get_cv_4d
        double rx, ry, rz, rw;
        surf.get_cv_4d(1, 1, rx, ry, rz, rw);

        // Also test get_cv
        Point pt = surf.get_cv(1, 1);
        double retrieved_w = surf.weight(1, 1);

        // Test knot_multiplicity
        int mult = surf.knot_count(0);
        int first_knot_mult = 0;
        if (mult > 0) {
            double first_val = surf.knot(0, 0);
            int count = 1;
            for (int i = 1; i < mult; ++i) {
                double val = surf.knot(0, i);
                if (std::abs(val - first_val) < 1e-10) {
                    count++;
                } else {
                    break;
                }
            }
            first_knot_mult = count;
        }

        MINI_CHECK(surf.is_rational());
        MINI_CHECK(TOLERANCE.is_close(rx, x));
        MINI_CHECK(TOLERANCE.is_close(ry, y));
        MINI_CHECK(TOLERANCE.is_close(rz, z));
        MINI_CHECK(TOLERANCE.is_close(rw, w));
        // get_cv returns Euclidean coordinates, so it divides homogeneous coords by w
        MINI_CHECK(TOLERANCE.is_close(pt[0], x/w));
        MINI_CHECK(TOLERANCE.is_close(pt[1], y/w));
        MINI_CHECK(TOLERANCE.is_close(pt[2], z/w));
        MINI_CHECK(TOLERANCE.is_close(retrieved_w, w));
        MINI_CHECK(first_knot_mult > 0);
    }

    MINI_TEST("NurbsSurface", "clamp_operations") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);

        // Set up control points
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        // Test clamp_end
        bool was_clamped_before = surf.is_clamped(0, 2);
        surf.clamp_end(0, 2);
        bool is_clamped_after = surf.is_clamped(0, 2);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(is_clamped_after);
    }

    MINI_TEST("NurbsSurface", "singularity") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create a simple surface
        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set all CVs to different points (non-singular)
        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, Point(1.0, 1.0, 0.0));

        // Test is_singular for each side
        bool is_singular_south = surf.is_singular(0);
        bool is_singular_east = surf.is_singular(1);
        bool is_singular_north = surf.is_singular(2);
        bool is_singular_west = surf.is_singular(3);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(!is_singular_south);
        MINI_CHECK(!is_singular_east);
        MINI_CHECK(!is_singular_north);
        MINI_CHECK(!is_singular_west);
    }

    MINI_TEST("NurbsSurface", "bounding_box") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 3, 3, false, false, 1.0, 1.0);

        // Set CVs in a known range
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        // Get bounding box
        BoundingBox bbox = surf.get_bounding_box();

        MINI_CHECK(surf.is_valid());
    }

    MINI_TEST("NurbsSurface", "domain_operations") {
        // uncomment #include "nurbssurface.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);

        // Get initial domain
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);

        // Set new domain
        surf.set_domain(0, 0.0, 10.0);
        surf.set_domain(1, 5.0, 15.0);

        auto new_dom_u = surf.domain(0);
        auto new_dom_v = surf.domain(1);

        // Get span vectors
        auto span_u = surf.get_span_vector(0);
        auto span_v = surf.get_span_vector(1);

        MINI_CHECK(dom_u.first == 0.0 && dom_u.second > 0.0);
        MINI_CHECK(dom_v.first == 0.0 && dom_v.second > 0.0);
        MINI_CHECK(TOLERANCE.is_close(new_dom_u.first, 0.0));
        MINI_CHECK(TOLERANCE.is_close(new_dom_u.second, 10.0));
        MINI_CHECK(TOLERANCE.is_close(new_dom_v.first, 5.0));
        MINI_CHECK(TOLERANCE.is_close(new_dom_v.second, 15.0));
        MINI_CHECK(span_u.size() > 0);
        MINI_CHECK(span_v.size() > 0);
    }

    MINI_TEST("NurbsSurface", "corner_points") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set corner control points
        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(10.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 10.0, 0.0));
        surf.set_cv(1, 1, Point(10.0, 10.0, 0.0));

        // Get corner points
        Point p00 = surf.point_at_corner(0, 0);
        Point p10 = surf.point_at_corner(1, 0);
        Point p01 = surf.point_at_corner(0, 1);
        Point p11 = surf.point_at_corner(1, 1);

        MINI_CHECK(TOLERANCE.is_close(p00[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(p10[0], 10.0));
        MINI_CHECK(TOLERANCE.is_close(p01[1], 10.0));
        MINI_CHECK(TOLERANCE.is_close(p11[0], 10.0) && TOLERANCE.is_close(p11[1], 10.0));
    }

    MINI_TEST("NurbsSurface", "swap_coordinates") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set a control point with distinct coordinates
        surf.set_cv(0, 0, Point(1.0, 2.0, 3.0));

        // Swap X and Y
        surf.swap_coordinates(0, 1);

        Point pt = surf.get_cv(0, 0);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 3.0));
    }

    MINI_TEST("NurbsSurface", "zero_cvs") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set non-zero control points
        surf.set_cv(0, 0, Point(1.0, 2.0, 3.0));
        surf.set_cv(1, 1, Point(4.0, 5.0, 6.0));

        // Zero all CVs
        surf.zero_cvs();

        Point pt0 = surf.get_cv(0, 0);
        Point pt1 = surf.get_cv(1, 1);

        MINI_CHECK(TOLERANCE.is_close(pt0[0], 0.0) &&
                   TOLERANCE.is_close(pt0[1], 0.0) &&
                   TOLERANCE.is_close(pt0[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt1[0], 0.0) &&
                   TOLERANCE.is_close(pt1[1], 0.0) &&
                   TOLERANCE.is_close(pt1[2], 0.0));
    }

    MINI_TEST("NurbsSurface", "get_knots") {
        // uncomment #include "nurbssurface.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 4, 3, 4, 3, false, false, 1.0, 2.0);

        auto knots_u = surf.get_knots(0);
        auto knots_v = surf.get_knots(1);

        MINI_CHECK(knots_u.size() == static_cast<size_t>(surf.knot_count(0)));
        MINI_CHECK(knots_v.size() == static_cast<size_t>(surf.knot_count(1)));
        MINI_CHECK(knots_u.size() > 0);
        MINI_CHECK(knots_v.size() > 0);
    }

    MINI_TEST("NurbsSurface", "make_non_rational") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create rational surface with all weights = 1
        NurbsSurface surf;
        surf.create_raw(3, true, 3, 3, 3, 3, false, false, 1.0, 1.0);

        // Set all weights to 1.0
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
                surf.set_weight(i, j, 1.0);
            }
        }

        bool was_rational = surf.is_rational();
        surf.make_non_rational();
        bool is_rational_after = surf.is_rational();

        MINI_CHECK(was_rational);
        MINI_CHECK(!is_rational_after);
    }

    MINI_TEST("NurbsSurface", "create_clamped_uniform") {
        // uncomment #include "nurbssurface.h"

        NurbsSurface surf;
        surf.create_clamped_uniform(3, 4, 3, 4, 4, 1.0, 2.0);

        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.dimension() == 3);
        MINI_CHECK(surf.order(0) == 4);
        MINI_CHECK(surf.order(1) == 3);
        MINI_CHECK(surf.cv_count(0) == 4);
        MINI_CHECK(surf.cv_count(1) == 4);
        MINI_CHECK(surf.is_clamped(0, 0) && surf.is_clamped(0, 1));
        MINI_CHECK(surf.is_clamped(1, 0) && surf.is_clamped(1, 1));
    }

    MINI_TEST("NurbsSurface", "knot_multiplicity") {
        // uncomment #include "nurbssurface.h"

        NurbsSurface surf;
        surf.create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);

        // Check first knot multiplicity (should be equal to degree for clamped)
        int mult_u_start = surf.knot_multiplicity(0, 0);
        int mult_v_start = surf.knot_multiplicity(1, 0);

        // Check last knot multiplicity
        int last_u = surf.knot_count(0) - 1;
        int last_v = surf.knot_count(1) - 1;
        int mult_u_end = surf.knot_multiplicity(0, last_u);
        int mult_v_end = surf.knot_multiplicity(1, last_v);

        MINI_CHECK(mult_u_start >= surf.degree(0));
        MINI_CHECK(mult_v_start >= surf.degree(1));
        MINI_CHECK(mult_u_end >= surf.degree(0));
        MINI_CHECK(mult_v_end >= surf.degree(1));
    }

    MINI_TEST("NurbsSurface", "sphere") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create a sphere as a rational NURBS surface
        // Sphere: 9x5 control points, degree 2 in both directions
        // Based on OpenNURBS sphere representation
        double radius = 2.0;
        double w = std::sqrt(2.0) / 2.0;  // 0.707107

        NurbsSurface surf;
        surf.create_raw(3, true, 3, 3, 9, 5, false, false, 1.0, 1.0);
        surf.name = "unit_sphere";

        // U-knots: periodic around equator with multiplicity 2
        double u_knots[] = {0, 0, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5,
                            TOLERANCE.PI, TOLERANCE.PI, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.5,
                            TOLERANCE.PI * 2.0, TOLERANCE.PI * 2.0};
        for (int i = 0; i < 10; ++i) surf.set_knot(0, i, u_knots[i]);

        // V-knots: from south pole to north pole
        double v_knots[] = {-TOLERANCE.PI * 0.5, -TOLERANCE.PI * 0.5, 0, 0,
                            TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5};
        for (int i = 0; i < 6; ++i) surf.set_knot(1, i, v_knots[i]);

        // Set up control points for sphere (9 around, 5 latitude levels)
        // Latitude levels: south pole, -45deg, equator, +45deg, north pole
        double lat_weights[] = {w, 0.5, w, 0.5, w};  // alternating for cardinal/diagonal
        double lat_z[] = {-radius, -radius * w, 0.0, radius * w, radius};
        double lat_r[] = {0.0, radius * w, radius, radius * w, 0.0};  // radius at each latitude

        for (int j = 0; j < 5; ++j) {
            double r = lat_r[j];
            double z = lat_z[j];
            // 9 points around (0, 45, 90, 135, 180, 225, 270, 315, 360=0)
            double angles[] = {0, TOLERANCE.PI * 0.25, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.75,
                               TOLERANCE.PI, TOLERANCE.PI * 1.25, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.75,
                               TOLERANCE.PI * 2.0};
            for (int i = 0; i < 9; ++i) {
                double x = r * std::cos(angles[i]);
                double y = r * std::sin(angles[i]);
                surf.set_cv(i, j, Point(x, y, z));
                // Weight: w for cardinal directions (0, 90, 180, 270), 0.5 for diagonals at non-pole latitudes
                double weight = (i % 2 == 0) ? w : lat_weights[j];
                if (j == 0 || j == 4) weight = w;  // poles
                surf.set_weight(i, j, weight);
            }
        }

        // Verify sphere properties
        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.degree(0) == 2);
        MINI_CHECK(surf.degree(1) == 2);
        MINI_CHECK(surf.cv_count(0) == 9);
        MINI_CHECK(surf.cv_count(1) == 5);

        // Check point on equator at angle 0 (should be at (radius, 0, 0))
        Point pt = surf.point_at(0.0, 0.0);
        MINI_CHECK(TOLERANCE.is_close(pt[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));

        // Check north pole
        Point north = surf.point_at(0.0, TOLERANCE.PI * 0.5);
        MINI_CHECK(TOLERANCE.is_close(north[2], radius));
    }

    MINI_TEST("NurbsSurface", "cylinder") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create an uncapped cylinder as a rational NURBS surface
        // Cylinder: 9x2 control points (circle at bottom, circle at top)
        // Degree 2 in U (angular), degree 1 in V (height)
        double radius = 1.5;
        double height = 3.0;
        double w = std::sqrt(2.0) / 2.0;

        NurbsSurface surf;
        surf.create_raw(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0);
        surf.name = "unit_cylinder";

        // U-knots: periodic for circle
        double u_knots[] = {0, 0, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5,
                            TOLERANCE.PI, TOLERANCE.PI, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.5,
                            TOLERANCE.PI * 2.0, TOLERANCE.PI * 2.0};
        for (int i = 0; i < 10; ++i) surf.set_knot(0, i, u_knots[i]);

        // V-knots: linear from bottom to top
        surf.set_knot(1, 0, 0.0);
        surf.set_knot(1, 1, height);

        // Set up control points for cylinder (9 around, 2 heights)
        double angles[] = {0, TOLERANCE.PI * 0.25, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.75,
                           TOLERANCE.PI, TOLERANCE.PI * 1.25, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.75,
                           TOLERANCE.PI * 2.0};

        for (int j = 0; j < 2; ++j) {
            double z = (j == 0) ? 0.0 : height;
            for (int i = 0; i < 9; ++i) {
                double x = radius * std::cos(angles[i]);
                double y = radius * std::sin(angles[i]);
                // For diagonal points (45, 135, etc), radius needs to be scaled
                if (i % 2 == 1) {
                    x = radius * std::sqrt(2.0) * std::cos(angles[i]);
                    y = radius * std::sqrt(2.0) * std::sin(angles[i]);
                }
                surf.set_cv(i, j, Point(x, y, z));
                double weight = (i % 2 == 0) ? 1.0 : w;
                surf.set_weight(i, j, weight);
            }
        }

        // Verify cylinder properties
        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.degree(0) == 2);
        MINI_CHECK(surf.degree(1) == 1);
        MINI_CHECK(surf.cv_count(0) == 9);
        MINI_CHECK(surf.cv_count(1) == 2);

        // Check point on bottom circle at angle 0
        Point pt_bottom = surf.point_at(0.0, 0.0);
        MINI_CHECK(TOLERANCE.is_close(pt_bottom[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_bottom[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_bottom[2], 0.0));

        // Check point on top circle at angle 0
        Point pt_top = surf.point_at(0.0, height);
        MINI_CHECK(TOLERANCE.is_close(pt_top[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_top[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_top[2], height));

        // Check midpoint at angle PI/2
        Point pt_mid = surf.point_at(TOLERANCE.PI * 0.5, height * 0.5);
        MINI_CHECK(TOLERANCE.is_close(pt_mid[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_mid[1], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_mid[2], height * 0.5));
    }

    MINI_TEST("NurbsSurface", "torus") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create a torus as a rational NURBS surface
        // Torus: 9x9 control points (minor circle revolved around major axis)
        // Degree 2 in both directions
        double major_radius = 3.0;  // distance from center to tube center
        double minor_radius = 1.0;  // tube radius
        double w = std::sqrt(2.0) / 2.0;

        NurbsSurface surf;
        surf.create_raw(3, true, 3, 3, 9, 9, false, false, 1.0, 1.0);
        surf.name = "unit_torus";

        // Both U and V knots: periodic for circles
        double knots[] = {0, 0, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5,
                          TOLERANCE.PI, TOLERANCE.PI, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.5,
                          TOLERANCE.PI * 2.0, TOLERANCE.PI * 2.0};
        for (int i = 0; i < 10; ++i) {
            surf.set_knot(0, i, knots[i]);
            surf.set_knot(1, i, knots[i]);
        }

        // Set up control points for torus
        // U: major angle (around torus), V: minor angle (around tube)
        double angles[] = {0, TOLERANCE.PI * 0.25, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.75,
                           TOLERANCE.PI, TOLERANCE.PI * 1.25, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.75,
                           TOLERANCE.PI * 2.0};

        for (int i = 0; i < 9; ++i) {
            double major_angle = angles[i];
            double cos_ma = std::cos(major_angle);
            double sin_ma = std::sin(major_angle);
            double major_scale = (i % 2 == 0) ? 1.0 : std::sqrt(2.0);

            for (int j = 0; j < 9; ++j) {
                double minor_angle = angles[j];
                double cos_mi = std::cos(minor_angle);
                double sin_mi = std::sin(minor_angle);
                double minor_scale = (j % 2 == 0) ? 1.0 : std::sqrt(2.0);

                double r = major_radius + minor_radius * minor_scale * cos_mi;
                double x = r * major_scale * cos_ma;
                double y = r * major_scale * sin_ma;
                double z = minor_radius * minor_scale * sin_mi;

                surf.set_cv(i, j, Point(x, y, z));

                // Weight is product of major and minor weights
                double w_major = (i % 2 == 0) ? 1.0 : w;
                double w_minor = (j % 2 == 0) ? 1.0 : w;
                surf.set_weight(i, j, w_major * w_minor);
            }
        }

        // Verify torus properties
        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.degree(0) == 2);
        MINI_CHECK(surf.degree(1) == 2);
        MINI_CHECK(surf.cv_count(0) == 9);
        MINI_CHECK(surf.cv_count(1) == 9);

        // Check point at (0, 0) - should be on outer edge of torus
        Point pt = surf.point_at(0.0, 0.0);
        MINI_CHECK(TOLERANCE.is_close(pt[0], major_radius + minor_radius));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));

        // Check point at (PI, 0) - should be on opposite side of torus
        Point pt_opp = surf.point_at(TOLERANCE.PI, 0.0);
        MINI_CHECK(TOLERANCE.is_close(pt_opp[0], -(major_radius + minor_radius)));
        MINI_CHECK(TOLERANCE.is_close(pt_opp[1], 0.0));
    }

    MINI_TEST("NurbsSurface", "cone") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create an uncapped cone as a rational NURBS surface
        // Cone: 9x2 control points (apex at top, circle at base)
        // Degree 2 in U (angular), degree 1 in V (height)
        double radius = 2.0;
        double height = 4.0;
        double w = std::sqrt(2.0) / 2.0;

        NurbsSurface surf;
        surf.create_raw(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0);
        surf.name = "unit_cone";

        // U-knots: periodic for circle
        double u_knots[] = {0, 0, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5,
                            TOLERANCE.PI, TOLERANCE.PI, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.5,
                            TOLERANCE.PI * 2.0, TOLERANCE.PI * 2.0};
        for (int i = 0; i < 10; ++i) surf.set_knot(0, i, u_knots[i]);

        // V-knots: linear from apex to base
        surf.set_knot(1, 0, 0.0);
        surf.set_knot(1, 1, height);

        // Set up control points for cone (9 around, 2 heights: apex and base)
        double angles[] = {0, TOLERANCE.PI * 0.25, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.75,
                           TOLERANCE.PI, TOLERANCE.PI * 1.25, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.75,
                           TOLERANCE.PI * 2.0};

        // Apex (j=0) - all points collapse to the apex
        for (int i = 0; i < 9; ++i) {
            surf.set_cv(i, 0, Point(0.0, 0.0, height));
            double weight = (i % 2 == 0) ? 1.0 : w;
            surf.set_weight(i, 0, weight);
        }

        // Base (j=1) - circle at z=0
        for (int i = 0; i < 9; ++i) {
            double x = radius * std::cos(angles[i]);
            double y = radius * std::sin(angles[i]);
            if (i % 2 == 1) {
                x = radius * std::sqrt(2.0) * std::cos(angles[i]);
                y = radius * std::sqrt(2.0) * std::sin(angles[i]);
            }
            surf.set_cv(i, 1, Point(x, y, 0.0));
            double weight = (i % 2 == 0) ? 1.0 : w;
            surf.set_weight(i, 1, weight);
        }

        // Verify cone properties
        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.degree(0) == 2);
        MINI_CHECK(surf.degree(1) == 1);
        MINI_CHECK(surf.cv_count(0) == 9);
        MINI_CHECK(surf.cv_count(1) == 2);

        // Check apex
        Point apex = surf.point_at(0.0, 0.0);
        MINI_CHECK(TOLERANCE.is_close(apex[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(apex[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(apex[2], height));

        // Check point on base circle at angle 0
        Point base = surf.point_at(0.0, height);
        MINI_CHECK(TOLERANCE.is_close(base[0], radius));
        MINI_CHECK(TOLERANCE.is_close(base[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(base[2], 0.0));

        // Check midpoint - should be at half radius, half height
        Point mid = surf.point_at(0.0, height * 0.5);
        MINI_CHECK(TOLERANCE.is_close(mid[0], radius * 0.5));
        MINI_CHECK(TOLERANCE.is_close(mid[2], height * 0.5));
    }

    MINI_TEST("NurbsSurface", "create_loft") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Loft 3 parallel circles at different Z heights
        double radius = 2.0;
        double w = std::sqrt(2.0) / 2.0;

        auto make_circle = [&](double z) -> NurbsCurve {
            NurbsCurve c(3, true, 3, 9);
            double knots[] = {0, 0, Tolerance::PI * 0.5, Tolerance::PI * 0.5,
                              Tolerance::PI, Tolerance::PI, Tolerance::PI * 1.5, Tolerance::PI * 1.5,
                              Tolerance::PI * 2.0, Tolerance::PI * 2.0};
            for (int i = 0; i < 10; ++i) c.set_knot(i, knots[i]);

            double angles[] = {0, Tolerance::PI * 0.25, Tolerance::PI * 0.5, Tolerance::PI * 0.75,
                               Tolerance::PI, Tolerance::PI * 1.25, Tolerance::PI * 1.5, Tolerance::PI * 1.75,
                               Tolerance::PI * 2.0};
            for (int i = 0; i < 9; ++i) {
                double scale = (i % 2 == 0) ? 1.0 : std::sqrt(2.0);
                double x = radius * scale * std::cos(angles[i]);
                double y = radius * scale * std::sin(angles[i]);
                c.set_cv(i, Point(x, y, z));
                c.set_weight(i, (i % 2 == 0) ? 1.0 : w);
            }
            return c;
        };

        NurbsCurve c0 = make_circle(0.0);
        NurbsCurve c1 = make_circle(3.0);
        NurbsCurve c2 = make_circle(6.0);

        NurbsSurface surf = NurbsSurface::create_loft({c0, c1, c2}, 2);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.cv_count(0) >= 9);
        MINI_CHECK(surf.cv_count(1) == 3);

        // Check point on bottom circle at angle 0
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);
        Point pt_bottom = surf.point_at(dom_u.first, dom_v.first);
        MINI_CHECK(TOLERANCE.is_close(pt_bottom[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_bottom[2], 0.0));

        // Check point on top circle at angle 0
        Point pt_top = surf.point_at(dom_u.first, dom_v.second);
        MINI_CHECK(TOLERANCE.is_close(pt_top[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_top[2], 6.0));
    }

    MINI_TEST("NurbsSurface", "create_revolve_full") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Revolve a line segment 360 degrees around Z axis -> cylinder
        double radius = 2.0;
        double height = 5.0;

        // Profile: vertical line at x=radius
        NurbsCurve profile = NurbsCurve::create(false, 1, {Point(radius, 0.0, 0.0), Point(radius, 0.0, height)});

        NurbsSurface surf = NurbsSurface::create_revolve(profile, Point(0, 0, 0), Vector(0, 0, 1));

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.degree(0) == 2);

        // Check point at angle 0, bottom
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);
        Point pt0 = surf.point_at(dom_u.first, dom_v.first);
        MINI_CHECK(TOLERANCE.is_close(pt0[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt0[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt0[2], 0.0));

        // Check point at angle PI/2, bottom
        Point pt90 = surf.point_at(Tolerance::PI * 0.5, dom_v.first);
        MINI_CHECK(TOLERANCE.is_close(pt90[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt90[1], radius));
        MINI_CHECK(TOLERANCE.is_close(pt90[2], 0.0));

        // Check point at angle 0, top
        Point pt_top = surf.point_at(dom_u.first, dom_v.second);
        MINI_CHECK(TOLERANCE.is_close(pt_top[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_top[2], height));
    }

    MINI_TEST("NurbsSurface", "create_revolve_partial") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Revolve a line segment 90 degrees -> quarter cylinder
        double radius = 3.0;
        double height = 4.0;

        NurbsCurve profile = NurbsCurve::create(false, 1, {Point(radius, 0.0, 0.0), Point(radius, 0.0, height)});

        double angle = Tolerance::PI * 0.5;
        NurbsSurface surf = NurbsSurface::create_revolve(profile, Point(0, 0, 0), Vector(0, 0, 1), angle);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_rational());
        MINI_CHECK(surf.degree(0) == 2);

        // Check start point
        auto dom_u = surf.domain(0);
        auto dom_v = surf.domain(1);
        Point pt_start = surf.point_at(dom_u.first, dom_v.first);
        MINI_CHECK(TOLERANCE.is_close(pt_start[0], radius));
        MINI_CHECK(TOLERANCE.is_close(pt_start[1], 0.0));

        // Check end point at 90 degrees
        Point pt_end = surf.point_at(dom_u.second, dom_v.first);
        MINI_CHECK(TOLERANCE.is_close(pt_end[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pt_end[1], radius));
    }

    MINI_TEST("NurbsSurface", "create_sweep1") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Sweep a small circle profile along a straight rail
        // Rail: line along Z axis
        NurbsCurve rail = NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(0, 0, 10)});

        // Profile: small circle in XY plane
        double r = 1.0;
        double w = std::sqrt(2.0) / 2.0;
        NurbsCurve profile(3, true, 3, 9);
        double knots[] = {0, 0, Tolerance::PI * 0.5, Tolerance::PI * 0.5,
                          Tolerance::PI, Tolerance::PI, Tolerance::PI * 1.5, Tolerance::PI * 1.5,
                          Tolerance::PI * 2.0, Tolerance::PI * 2.0};
        for (int i = 0; i < 10; ++i) profile.set_knot(i, knots[i]);

        double angles[] = {0, Tolerance::PI * 0.25, Tolerance::PI * 0.5, Tolerance::PI * 0.75,
                           Tolerance::PI, Tolerance::PI * 1.25, Tolerance::PI * 1.5, Tolerance::PI * 1.75,
                           Tolerance::PI * 2.0};
        for (int i = 0; i < 9; ++i) {
            double scale = (i % 2 == 0) ? 1.0 : std::sqrt(2.0);
            double x = r * scale * std::cos(angles[i]);
            double y = r * scale * std::sin(angles[i]);
            profile.set_cv(i, Point(x, y, 0.0));
            profile.set_weight(i, (i % 2 == 0) ? 1.0 : w);
        }

        NurbsSurface surf = NurbsSurface::create_sweep1(rail, profile);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.cv_count(0) >= 9);
        MINI_CHECK(surf.cv_count(1) >= 2);
    }

    MINI_TEST("NurbsSurface", "create_sweep2") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Sweep a line profile along two diverging rails
        // Rail1: line along Z at x=0
        // Rail2: line along Z at x=5
        NurbsCurve rail1 = NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(0, 0, 10)});
        NurbsCurve rail2 = NurbsCurve::create(false, 1, {Point(5, 0, 0), Point(5, 0, 10)});

        // Profile: horizontal line from (0,0,0) to (5,0,0)
        NurbsCurve profile = NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(5, 0, 0)});

        NurbsSurface surf = NurbsSurface::create_sweep2(rail1, rail2, {profile});

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.cv_count(0) >= 2);
        MINI_CHECK(surf.cv_count(1) >= 2);
    }

    MINI_TEST("NurbsSurface", "create_planar") {
        // uncomment #include "mesh.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include "trimesh_delaunay.h"

        std::vector<Point> pts = {Point(0,0,0), Point(3,1,0), Point(5,0.5,0), Point(6,3,0), Point(4,5,0), Point(1,4,0), Point(0,0,0)};
        NurbsCurve boundary = NurbsCurve::create(false, 3, pts);
        NurbsSurface surf = NurbsSurface::create_planar(boundary);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_planar());
        MINI_CHECK(surf.degree(0) == 1);
        MINI_CHECK(surf.degree(1) == 1);
        MINI_CHECK(surf.cv_count(0) == 2);
        MINI_CHECK(surf.cv_count(1) == 2);
        MINI_CHECK(surf.is_trimmed());
        MINI_CHECK(surf.get_outer_loop().is_valid());

        auto [pd, puv] = surf.divide_by_count(4, 4);
        MINI_CHECK(pd.size() == 5);
        MINI_CHECK(pd[0].size() == 5);

        TrimeshDelaunay tri(surf);
        Mesh mesh = tri.mesh();
        MINI_CHECK(mesh.number_of_faces() > 0);
        MINI_CHECK(mesh.number_of_vertices() > 0);
    }

}
