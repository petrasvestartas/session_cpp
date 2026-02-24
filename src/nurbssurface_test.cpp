#include "mini_test.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
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

    MINI_TEST("NurbsSurface", "Constructor") {
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
        auto [p, v, uv] = s.divide_by_count_points(4, 6);

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
        MINI_CHECK(TOLERANCE.is_point_close(p[0][0], Point(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[0][1], Point(-0.416666666666667, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[0][2], Point(-0.666666666666667, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[0][3], Point(-0.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[0][4], Point(-0.666666666666667, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[0][5], Point(-0.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[0][6], Point(0.000000000000000, 5.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][0], Point(0.992187500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][1], Point(0.881510416666667, 0.333912037037037, 1.958333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][2], Point(0.815104166666667, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][3], Point(0.792968750000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][4], Point(0.815104166666667, 3.620370370370370, 2.458333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][5], Point(0.881510416666667, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[1][6], Point(0.992187500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][0], Point(2.500000000000000, -0.750000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][1], Point(2.500000000000000, 0.252314814814815, 2.333333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][2], Point(2.500000000000000, 1.351851851851852, 2.833333333333334)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][3], Point(2.500000000000000, 2.500000000000000, 3.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][4], Point(2.500000000000000, 3.648148148148148, 2.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][5], Point(2.500000000000000, 4.747685185185186, 2.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[2][6], Point(2.500000000000000, 5.750000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][0], Point(4.007812500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][1], Point(4.118489583333334, 0.333912037037037, 1.958333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][2], Point(4.184895833333334, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][3], Point(4.207031250000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][4], Point(4.184895833333333, 3.620370370370370, 2.458333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][5], Point(4.118489583333333, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[3][6], Point(4.007812500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][0], Point(5.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][1], Point(5.416666666666668, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][2], Point(5.666666666666668, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][3], Point(5.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][4], Point(5.666666666666666, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][5], Point(5.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(p[4][6], Point(5.000000000000000, 5.000000000000000, 0.000000000000000)));
    }

    MINI_TEST("NurbsSurface", "Booleans Queries"){
        // uncomment #include "nurbssurface.h"

        NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 5.0);

        // Validity surface and knots
        bool is_valid = s.is_valid();
        bool are_knots_valid = s.is_valid_knot_vector(0) && s.is_valid_knot_vector(1);

        // Are control points weights enabled?
        bool is_rational = s.is_rational();

        // Sphere has one seam that is closed, but two poles
        bool is_closed = s.is_closed(0) == true && s.is_closed(1) == false;

        // sphere cannot be truly periodic because it has poles
        bool is_periodic = s.is_periodic(0) && s.is_periodic(1);

        // Planarity
        Plane plane = Plane::xy_plane();
        bool is_planar = s.is_planar(&plane);

        // Surface is collapsed to a point
        bool is_point = s.is_singular(0) && s.is_singular(1) && s.is_singular(2) && s.is_singular(3);

        // Most surfaces are clamped except periodic surfaces
        bool is_clamped = s.is_clamped(0, 2) && s.is_clamped(1, 2);

        MINI_CHECK(is_valid);
        MINI_CHECK(are_knots_valid);
        MINI_CHECK(is_rational);
        MINI_CHECK(is_closed);
        MINI_CHECK(!is_periodic);
        MINI_CHECK(!is_planar);
        MINI_CHECK(!is_point);
        MINI_CHECK(is_clamped);
    }


    MINI_TEST("NurbsSurface", "Attributes") {
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

        // Check the dimentions of a surface
        // Mostly 3d
        // But 2d can be used for: scalar field over parameter space e.g. czrvatzre map, distance field
        // Planar geometry: texture coordinates
        int dimensions = s.dimension();

        // Degree types 1 - linear, 2 - quadratic, 3 - cubic
        int order_u = s.order(0);
        int order_v = s.order(1);

        // Control vertex count
        int cv_count_u = s.cv_count(0);
        int cv_count_v = s.cv_count(1);
        int cv_count = s.cv_count();
        int cv_size = s.cv_size();

        // Number of knots
        int k_count_0 = s.knot_count(0);
        int k_count_1 = s.knot_count(1);

        // Span count
        int s_count_0 = s.span_count(0);
        int s_count_1 = s.span_count(1);

        MINI_CHECK(dimensions==3);
        MINI_CHECK(order_u==4);
        MINI_CHECK(order_v==4);
        MINI_CHECK(cv_count_u);
        MINI_CHECK(cv_count_v);
        MINI_CHECK(cv_count);
        MINI_CHECK(cv_size);
        MINI_CHECK(k_count_0);
        MINI_CHECK(k_count_1);
        MINI_CHECK(s_count_0);
        MINI_CHECK(s_count_1);
    }

    MINI_TEST("NurbsSurface", "Control Vertices Access") {
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
        s.make_rational(); // to change weights

        // const - to read, non-const point to write
        const double* const_pointer_cv = s.cv(0,0);
        MINI_CHECK( const_pointer_cv[2] == 0);
        double* pointer_cv = s.cv(0,0);
        pointer_cv[2] = 10.0; // modifies surface because it is a pointer
        MINI_CHECK( pointer_cv[2] == 10);

        // Point and Weight
        // NOTE
        // point is (Xw, Yw, Zw, w)
        // cv pointer is (X, Y, Z)
        Point cv = s.get_cv(0,0);
        MINI_CHECK( cv == Point(0,0,10));
        double x, y, z, w;
        s.get_cv_4d(0,0, x, y, z, w);
        MINI_CHECK( x == 0 && y == 0 && z == 10 && w == 1);

        s.set_cv(0,0, Point(0, 0, 5));
        MINI_CHECK(  s.get_cv(0,0) == Point(0,0,5) );
        s.set_cv_4d(0,0, 0, 0, 4, 0.5);
        MINI_CHECK( s.get_cv(0,0) == Point(0, 0, 8) && s.cv(0,0)[2] == 4 && s.weight(0,0) == 0.5 );

        w = s.weight(0,0);
        s.set_weight(0,0,1);
        MINI_CHECK( s.weight(0,0) == 1); 
    }

    MINI_TEST("NurbsSurface", "Knot Access") {
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

        // Get knot vectors and individual knot
        std::vector<double> knots_u = s.get_knots(0);
        for (int i = 0; i < s.knot_count(0); i++){
            double knot = s.knot(0, i);
            MINI_CHECK(knot == knots_u[i]);
        }

        std::vector<double> knots_v = s.get_knots(1);
        for (int i = 0; i < s.knot_count(1); i++){
            double knot = s.knot(1, i);
            MINI_CHECK(knot == knots_v[i]);
        }

        // Set knots
        bool is_set = s.set_knot(0, 2, 0.5);
        MINI_CHECK(s.knot(0, 2) == 0.5);
        is_set = s.set_knot(0, 2, 0.0); // reset

        // Verify start multiplicity
        int mult_u_start = s.knot_multiplicity(0, 0);
        int mult_v_start = s.knot_multiplicity(1, 0);
        MINI_CHECK(mult_u_start == 3);
        MINI_CHECK(mult_v_start == 3);

        s.insert_knot(0, 0.1, 2);
        MINI_CHECK(s.knot_count(0) == 8);
        MINI_CHECK(s.knot(0, 3) == 0.1);
        MINI_CHECK(s.knot_multiplicity(0, 3) == 2);
    }

    MINI_TEST("NurbsSurface", "Domain") {
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

        // Get domain 0 - 1
        std::pair<double, double> domain_u = s.domain(0);
        std::pair<double, double> domain_v = s.domain(1);
        MINI_CHECK(TOLERANCE.is_close(domain_u.first, 0));
        MINI_CHECK(TOLERANCE.is_close(domain_u.second, 1));

        // Set Domain
        bool is_set_u = s.set_domain(0, -1.1, 2.3);
        bool is_set_v = s.set_domain(1, -5.1, 1.3);
        MINI_CHECK(is_set_u && TOLERANCE.is_close(s.domain(1).first, -5.1));
        MINI_CHECK(is_set_v && TOLERANCE.is_close(s.domain(1).second, 1.3));

        // Get sorted list of distinct knot values
        std::vector<double> span_vector = s.get_span_vector(0);
        double first_item = span_vector.front();
        double last_item = span_vector.back();
        MINI_CHECK(TOLERANCE.is_close(first_item, -1.1));
        MINI_CHECK(TOLERANCE.is_close(last_item, 2.3));
    }

    MINI_TEST("NurbsSurface", "Division") {

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

        // points, normals, uv
        auto [division_points, vectors, uvs0] = s.divide_by_count_points(3, 3);

        // planes, normals, uv
        auto [planes, uvs1] = s.divide_by_count_planes(3, 3);

        MINI_CHECK(TOLERANCE.is_point_close(division_points[0][0], Point(0, 0, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[0][1], Point(-0.666666666666667, 1.46296296296296, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[0][2], Point(-0.666666666666667, 3.53703703703704, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[0][3], Point(0, 5, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[1][0], Point(1.46296296296296, -0.666666666666667, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[1][1], Point(1.3641975308642, 1.3641975308642, 2.66666666666667)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[1][2], Point(1.3641975308642, 3.6358024691358, 2.66666666666667)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[1][3], Point(1.46296296296296, 5.66666666666667, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[2][0], Point(3.53703703703704, -0.666666666666667, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[2][1], Point(3.6358024691358, 1.3641975308642, 2.66666666666667)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[2][2], Point(3.6358024691358, 3.6358024691358, 2.66666666666667)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[2][3], Point(3.53703703703704, 5.66666666666667, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[3][0], Point(5, 0, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[3][1], Point(5.66666666666667, 1.46296296296296, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[3][2], Point(5.66666666666667, 3.53703703703704, 1.33333333333333)));
        MINI_CHECK(TOLERANCE.is_point_close(division_points[3][3], Point(5, 5, 0)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[0][0], Vector(-0.704360725060499, -0.704360725060499, -0.0880450906325624)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[0][1], Vector(-0.722897836195991, -0.327787263130091, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[0][2], Vector(-0.722897836195991, 0.327787263130091, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[0][3], Vector(-0.704360725060499, 0.704360725060499, -0.0880450906325624)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[1][0], Vector(-0.327787263130091, -0.722897836195991, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[1][1], Vector(-0.280457757277237, -0.280457757277237, 0.917979788865771)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[1][2], Vector(-0.280457757277237, 0.280457757277237, 0.917979788865771)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[1][3], Vector(-0.327787263130091, 0.722897836195991, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[2][0], Vector(0.327787263130091, -0.722897836195991, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[2][1], Vector(0.280457757277237, -0.280457757277237, 0.917979788865771)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[2][2], Vector(0.280457757277237, 0.280457757277237, 0.917979788865771)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[2][3], Vector(0.327787263130091, 0.722897836195991, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[3][0], Vector(0.704360725060499, -0.704360725060499, -0.0880450906325624)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[3][1], Vector(0.722897836195991, -0.327787263130091, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[3][2], Vector(0.722897836195991, 0.327787263130091, 0.608255068661856)));
        MINI_CHECK(TOLERANCE.is_vector_close(vectors[3][3], Vector(0.704360725060499, 0.704360725060499, -0.0880450906325624)));
        MINI_CHECK(TOLERANCE.is_close(uvs0[0][0].first, 0.0) && TOLERANCE.is_close(uvs0[0][0].second, 0.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[0][1].first, 0.0) && TOLERANCE.is_close(uvs0[0][1].second, 0.333333333333333));
        MINI_CHECK(TOLERANCE.is_close(uvs0[0][2].first, 0.0) && TOLERANCE.is_close(uvs0[0][2].second, 0.666666666666667));
        MINI_CHECK(TOLERANCE.is_close(uvs0[0][3].first, 0.0) && TOLERANCE.is_close(uvs0[0][3].second, 1.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[1][0].first, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][0].second, 0.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[1][1].first, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][1].second, 0.333333333333333));
        MINI_CHECK(TOLERANCE.is_close(uvs0[1][2].first, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][2].second, 0.666666666666667));
        MINI_CHECK(TOLERANCE.is_close(uvs0[1][3].first, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][3].second, 1.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[2][0].first, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][0].second, 0.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[2][1].first, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][1].second, 0.333333333333333));
        MINI_CHECK(TOLERANCE.is_close(uvs0[2][2].first, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][2].second, 0.666666666666667));
        MINI_CHECK(TOLERANCE.is_close(uvs0[2][3].first, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][3].second, 1.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[3][0].first, 1.0) && TOLERANCE.is_close(uvs0[3][0].second, 0.0));
        MINI_CHECK(TOLERANCE.is_close(uvs0[3][1].first, 1.0) && TOLERANCE.is_close(uvs0[3][1].second, 0.333333333333333));
        MINI_CHECK(TOLERANCE.is_close(uvs0[3][2].first, 1.0) && TOLERANCE.is_close(uvs0[3][2].second, 0.666666666666667));
        MINI_CHECK(TOLERANCE.is_close(uvs0[3][3].first, 1.0) && TOLERANCE.is_close(uvs0[3][3].second, 1.0));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][0].x_axis(), Vector(0.317999364001908, -0.423999152002544, 0.847998304005088)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][1].x_axis(), Vector(0.657483781160109, -0.0556600026378928, 0.751410035611553)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][2].x_axis(), Vector(0.657483781160109, 0.055660002637893, 0.751410035611553)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][3].x_axis(), Vector(0.317999364001908, 0.423999152002544, 0.847998304005088)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][0].x_axis(), Vector(0.93542594448836, -0.158100159631836, 0.316200319263671)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][1].x_axis(), Vector(0.957938608304167, -0.0211991946512679, 0.286189127792116)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][2].x_axis(), Vector(0.957938608304167, 0.0211991946512677, 0.286189127792116)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][3].x_axis(), Vector(0.93542594448836, 0.158100159631835, 0.316200319263671)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][0].x_axis(), Vector(0.93542594448836, 0.158100159631835, -0.316200319263671)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][1].x_axis(), Vector(0.957938608304167, 0.0211991946512679, -0.286189127792116)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][2].x_axis(), Vector(0.957938608304167, -0.021199194651268, -0.286189127792116)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][3].x_axis(), Vector(0.93542594448836, -0.158100159631836, -0.316200319263671)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][0].x_axis(), Vector(0.317999364001908, 0.423999152002544, -0.847998304005088)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][1].x_axis(), Vector(0.657483781160109, 0.0556600026378928, -0.751410035611553)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][2].x_axis(), Vector(0.657483781160109, -0.0556600026378928, -0.751410035611553)));      
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][3].x_axis(), Vector(0.317999364001908, -0.423999152002544, -0.847998304005088)));  
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][0].y_axis(), Vector(-0.423999152002544, 0.317999364001908, 0.847998304005088)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][1].y_axis(), Vector(-0.158100159631836, 0.93542594448836, 0.316200319263671)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][2].y_axis(), Vector(0.158100159631835, 0.93542594448836, -0.316200319263671)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[0][3].y_axis(), Vector(0.423999152002544, 0.317999364001908, -0.847998304005088)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][0].y_axis(), Vector(-0.0556600026378928, 0.657483781160109, 0.751410035611553)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][1].y_axis(), Vector(-0.0211991946512679, 0.957938608304167, 0.286189127792116)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][2].y_axis(), Vector(0.0211991946512679, 0.957938608304167, -0.286189127792116)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[1][3].y_axis(), Vector(0.0556600026378928, 0.657483781160109, -0.751410035611553)));       
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][0].y_axis(), Vector(0.0556600026378928, 0.657483781160109, 0.751410035611553)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][1].y_axis(), Vector(0.0211991946512678, 0.957938608304167, 0.286189127792116)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][2].y_axis(), Vector(-0.0211991946512678, 0.957938608304167, -0.286189127792116)));      
        MINI_CHECK(TOLERANCE.is_vector_close(planes[2][3].y_axis(), Vector(-0.0556600026378928, 0.657483781160109, -0.751410035611553)));      
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][0].y_axis(), Vector(0.423999152002544, 0.317999364001908, 0.847998304005088)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][1].y_axis(), Vector(0.158100159631835, 0.93542594448836, 0.316200319263671)));
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][2].y_axis(), Vector(-0.158100159631836, 0.93542594448836, -0.316200319263671)));        
        MINI_CHECK(TOLERANCE.is_vector_close(planes[3][3].y_axis(), Vector(-0.423999152002544, 0.317999364001908, -0.847998304005088)));  
    }



    MINI_TEST("NurbsSurface", "Evaluation") {
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

        double u = 0.5, v = 0.5;

        // point_at(u, v) - returns Point
        Point p1 = s.point_at(u, v);
        MINI_CHECK(TOLERANCE.is_point_close(p1, Point(2.5, 2.5, 3.0)));

        // normal_at(u, v) - returns Vector
        Vector n1 = s.normal_at(u, v);
        MINI_CHECK(TOLERANCE.is_vector_close(n1, Vector(0, 0, 1)));

        // evaluate(u, v, num_derivs) - returns vector of derivatives
        auto derivs = s.evaluate(u, v, 1);
        MINI_CHECK(TOLERANCE.is_vector_close(derivs[0], Vector(2.5, 2.5, 3.0)));
        MINI_CHECK(TOLERANCE.is_vector_close(derivs[1], Vector(0.0, 6.9375, 0.0)));
        MINI_CHECK(TOLERANCE.is_vector_close(derivs[2], Vector(6.9375, 0.0, 0.0)));

        // point_at_corner(u_end, v_end) - corner point
        Point p_corner = s.point_at_corner(1, 1);
        MINI_CHECK(TOLERANCE.is_point_close(p_corner, Point(5.0, 5.0, 0.0)));
   
        // get isocurve - returns NurbsCurve
        NurbsCurve iso_u = s.iso_curve(0, v);
        NurbsCurve iso_v = s.iso_curve(1, u);
        MINI_CHECK(TOLERANCE.is_point_close(iso_u.point_at(0.5), Point(2.5, 2.5, 3.0)));
        MINI_CHECK(TOLERANCE.is_point_close(iso_v.point_at(0.5), Point(2.5, 2.5, 3.0)));
    }

    MINI_TEST("NurbsSurface", "Modification") {
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


        // Reverse one direction
        NurbsSurface s_rev = s;
        s_rev.reverse(0);
        MINI_CHECK(s_rev.point_at_corner(0, 0) == s.point_at_corner(1, 0));
        MINI_CHECK(s_rev.normal_at(0.5, 0.5) == s.normal_at(0.5, 0.5)*-1);

        // Swap u and v direction
        NurbsSurface s_tr = s;
        s_tr.transpose();
        MINI_CHECK(s.point_at(0, 0.5) == s_tr.point_at(0.5, 0));

        // Swap coordinates - swap x and z
        NurbsSurface s_swap = s;
        s_swap.swap_coordinates(0, 2);
        MINI_CHECK(s.point_at(0.5, 0.5)[0] == s_swap.point_at(0.5, 0.5)[2]);
        MINI_CHECK(s.point_at(0.5, 0.5)[2] == s_swap.point_at(0.5, 0.5)[0]);

        // Trim surface, domain changed but parametrization preserved
        NurbsSurface s_trim = s;
        s_trim.trim(0, {0.25, 0.75});
        MINI_CHECK(TOLERANCE.is_close(s_trim.domain(0).first, 0.25) && TOLERANCE.is_close(s_trim.domain(0).second, 0.75));
        MINI_CHECK(TOLERANCE.is_point_close(s.point_at(0.25, 0.5), s_trim.point_at(0.25, 0.5)));
       
        // Split surface into 4 quadrants, check shared corner point is the same
        auto [west, east] = s.split(0, 0.5);
        auto [ww, we] = west.split(1, (west.domain(1).first + west.domain(1).second) / 2.0);
        auto [ew, ee] = east.split(1, (east.domain(1).first + east.domain(1).second) / 2.0);
        Point center = s.point_at(0.5, 0.5);
        MINI_CHECK(TOLERANCE.is_point_close(ww.point_at_corner(1, 1), center));  
        MINI_CHECK(TOLERANCE.is_point_close(we.point_at_corner(1, 0), center));  
        MINI_CHECK(TOLERANCE.is_point_close(ew.point_at_corner(0, 1), center));  
        MINI_CHECK(TOLERANCE.is_point_close(ee.point_at_corner(0, 0), center));

        // Make rational and change weight    
        NurbsSurface s_rat = s;
        s_rat.make_rational();
        s_rat.set_weight(2, 2, 3.0); // pull center CV toward the surface
        MINI_CHECK(s.point_at(0.5, 0.5) != s_rat.point_at(0.5, 0.5));  
        s_rat.make_non_rational();
        MINI_CHECK(s.point_at(0.5, 0.5) == s_rat.point_at(0.5, 0.5));  

        // Increase degree
        NurbsSurface s_deg = s;
        s_deg.increase_degree(0, 6);
        s_deg.increase_degree(1, 6);
        MINI_CHECK(s.cv_count(0) == 4 && s.cv_count(1) == 4); // same CV count
        MINI_CHECK(s_deg.cv_count(0) == 7 && s_deg.cv_count(1) == 7); // same CV count
    
    }
 


    MINI_TEST("NurbsSurface", "Transformations"){
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


        // transform() - Apply stored xform (in-place)                                                                                                                                                         
        NurbsSurface surface1 = NurbsSurface::create(false, false, 3, 3, 4, 4, points);                                                         
        surface1.xform = Xform::translation(0.0, 0.0, 1.0);  
        surface1.transform();  // applies stored xform, modifies surface1
        MINI_CHECK(surface1.xform.is_identity() == false);
        MINI_CHECK(surface1.cv(0, 0)[2] == 1.0);

        // transform(const Xform&) - Apply custom xform (in-place)
        NurbsSurface surface2 = NurbsSurface::create(false, false, 3, 3, 4, 4, points);
        Xform x = Xform::translation(0.0, 0.0, 1.0);
        surface2.transform(x);  // modifies surface2 directly
        MINI_CHECK(surface2.xform.is_identity() == true);
        MINI_CHECK(surface2.cv(0, 0)[2] == 1.0);

        // transformed() - Get copy with stored xform applied
        NurbsSurface surface3 = NurbsSurface::create(false, false, 3, 3, 4, 4, points);
        surface3.xform = Xform::translation(0.0, 0.0, 10.0);
        NurbsSurface surface3_transformed = surface3.transformed();
        MINI_CHECK(surface3_transformed.xform.is_identity() == false);
        MINI_CHECK(surface3_transformed.cv(0, 0)[2] == 10.0);
        
        // transformed(const Xform&) - Get copy with custom xform
        NurbsSurface surface4 = NurbsSurface::create(false, false, 3, 3, 4, 4, points);
        x = Xform::translation(0.0, 0.0, 10.0);
        NurbsSurface surface4_transformed = surface4.transformed(x); 
        MINI_CHECK(surface4_transformed.xform.is_identity() == true);
        MINI_CHECK(surface4_transformed.cv(0, 0)[2] == 10.0);

    }

    MINI_TEST("NurbsSurface", "Meshing") {
        // uncomment #include "primitives.h"

        // 1. Sphere — two poles, closed U, rational
        NurbsSurface sphere = Primitives::sphere_surface(0, 0, 0, 3.0);
        Mesh mesh_sphere = sphere.mesh();
        Mesh mesh_sphere_delaunay = sphere.mesh_delaunay(45.0);
        MINI_CHECK(mesh_sphere.is_valid());
        MINI_CHECK(mesh_sphere_delaunay.is_valid());

        // 2. Cone — singular apex (pole), closed U
        NurbsSurface cone = Primitives::cone_surface(0, 12, 0, 2.0, 6.0);
        Mesh mesh_cone = cone.mesh();
        Mesh mesh_cone_delaunay = cone.mesh_delaunay(45.0);
        MINI_CHECK(mesh_cone.is_valid());
        MINI_CHECK(mesh_cone_delaunay.is_valid());

        // 3. Torus — doubly closed (U and V), rational
        NurbsSurface torus = Primitives::torus_surface(0, 24, 0, 4.0, 1.5);
        Mesh mesh_torus = torus.mesh();
        Mesh mesh_torus_delaunay = torus.mesh_delaunay(45.0);
        MINI_CHECK(mesh_torus.is_valid());
        MINI_CHECK(mesh_torus_delaunay.is_valid());

        // 4. Loft — varying radius circles, closed U, multi-span V
        NurbsSurface loft = Primitives::create_loft({
            Primitives::circle(0, 38, 0, 2.0), 
            Primitives::circle(0, 38, 2, 1.0),
            Primitives::circle(0, 38, 4, 1.5), 
            Primitives::circle(0, 38, 6, 0.8)}, 3);
        Mesh mesh_loft = loft.mesh();
        Mesh mesh_loft_delaunay = loft.mesh_delaunay(45.0);
        MINI_CHECK(mesh_loft.is_valid());
        MINI_CHECK(mesh_loft_delaunay.is_valid());

        // 5. Extrusion (circle) — closed U, linear V, rational
        Vector ext_dir(0, 0, 5);
        NurbsSurface cylinder = Primitives::create_extrusion(
            Primitives::circle(0, 52, 0, 3.0), 
            ext_dir);
        Mesh mesh_cylinder = cylinder.mesh();
        Mesh mesh_cylinder_delaunay = cylinder.mesh_delaunay(45.0);
        MINI_CHECK(mesh_cylinder.is_valid());
        MINI_CHECK(mesh_cylinder_delaunay.is_valid());

        // 6. Ruled — bilinear (degree 1×1), tests twist subdivision
        auto ra = NurbsCurve::create(false, 1, {
            Point(0, 64, 0), 
            Point(5, 64, 5)});
        auto rb = NurbsCurve::create(false, 1, {
            Point(0, 64 + 5, 5), Point(5, 64 + 5, 0)});
        NurbsSurface hypar = Primitives::create_ruled(ra, rb);
        Mesh mesh_hypar = hypar.mesh();
        Mesh mesh_hypar_delaunay = hypar.mesh_delaunay(45.0);
        MINI_CHECK(mesh_hypar.is_valid());
        MINI_CHECK(mesh_hypar_delaunay.is_valid());

        // 7. Sweep1 — circle along curved rail
        NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
        NurbsCurve rail = NurbsCurve::create(false, 2, {
            Point(0, 76, 0), 
            Point(0, 81, 0), 
            Point(2, 85, 0)});
        NurbsSurface sweep1 = Primitives::create_sweep1(rail, profile);
        Mesh mesh_sweep1 = sweep1.mesh();
        Mesh mesh_sweep1_delaunay = sweep1.mesh_delaunay(45.0);
        MINI_CHECK(mesh_sweep1.is_valid());
        MINI_CHECK(mesh_sweep1_delaunay.is_valid());

        // 8. Sweep2 — two rails + cross sections
        NurbsCurve r1 = NurbsCurve::create(false, 2, {
            Point(0, 90 - 1, 0), 
            Point(1, 90 + 3, 0), 
            Point(2, 90 + 4, 0)});
        NurbsCurve r2 = NurbsCurve::create(false, 2, {
            Point(4, 90 - 1, 0), 
            Point(4, 90 + 3, 0), 
            Point(3, 90 + 4, 0)});
        NurbsCurve sh1 = NurbsCurve::create(false, 2, {
            Point(0, 90 - 1, 0), 
            Point(2, 90 - 1, 2), 
            Point(4, 90 - 1, 0)});
        NurbsCurve sh2 = NurbsCurve::create(false, 2, {
            Point(2, 90 + 4, 0), 
            Point(2.5, 90 + 4, 1.5), 
            Point(3, 90 + 4, 0)});
        NurbsSurface sweep2 = Primitives::create_sweep2(r1, r2, {sh1, sh2});
        Mesh mesh_sweep2 = sweep2.mesh();
        Mesh mesh_sweep2_delaunay = sweep2.mesh_delaunay(45.0);
        MINI_CHECK(mesh_sweep2.is_valid());
        MINI_CHECK(mesh_sweep2_delaunay.is_valid());

        // 9. Edge surface (Coons patch) — 4 boundary curves
        auto south = NurbsCurve::create(false, 3, {
            Point(1, 104, 0), 
            Point(1, 104 + 2, 3), 
            Point(1, 104 + 5, 3), 
            Point(1, 104 + 7, 0)});
        auto west  = NurbsCurve::create(false, 2, {
            Point(10, 104, 0), 
            Point(5.5, 104, 3.5), 
            Point(1, 104, 0)});
        auto north = NurbsCurve::create(false, 3, {
            Point(10, 104, 0), 
            Point(10, 104 + 2, 3), 
            Point(10, 104 + 5, 3), 
            Point(10, 104 + 7, 0)});
        auto east  = NurbsCurve::create(false, 2, {
            Point(10, 104 + 7, 0), 
            Point(5.5, 104 + 7, 3.5), 
            Point(1, 104 + 7, 0)});
        NurbsSurface arched = Primitives::create_edge(south, west, north, east);
        Mesh mesh_arched = arched.mesh();
        Mesh mesh_arched_delaunay = arched.mesh_delaunay(45.0);
        MINI_CHECK(mesh_arched.is_valid());
        MINI_CHECK(mesh_arched_delaunay.is_valid());

        // 10. Wave — multi-span freeform (13×13 CVs, 10 spans)
        NurbsSurface wave = Primitives::wave_surface(5.0, 1.5);
        Mesh mesh_wave = wave.mesh();
        Mesh mesh_wave_delaunay = wave.mesh_delaunay(45.0);
        MINI_CHECK(mesh_wave.is_valid());
        MINI_CHECK(mesh_wave_delaunay.is_valid());

        // 11. Planar — mesh() early exit: 2 triangles
        auto planar = NurbsCurve::create(false, 1, {
            Point(0, 132, 0), 
            Point(6, 132, 0), 
            Point(6, 136, 0), 
            Point(0, 136, 0), 
            Point(0, 132, 0)});
        NurbsSurface pln = Primitives::create_planar(planar);
        Mesh mesh_planar = pln.mesh();
        Mesh mesh_planar_delaunay = pln.mesh_delaunay(45.0);
        MINI_CHECK(mesh_planar.is_valid());
        MINI_CHECK(mesh_planar_delaunay.is_valid());

    }
   
    MINI_TEST("NurbsSurface", "Json_roundtrip") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(-1.0, 0.75, 2.0), Point(-1.0, 4.25, 2.0), Point(0.0, 5.0, 0.0),
            Point(0.75, -1.0, 2.0), Point(1.25, 1.25, 4.0), Point(1.25, 3.75, 4.0), Point(0.75, 6.0, 2.0),
            Point(4.25, -1.0, 2.0), Point(3.75, 1.25, 4.0), Point(3.75, 3.75, 4.0), Point(4.25, 6.0, 2.0),
            Point(5.0, 0.0, 0.0), Point(6.0, 0.75, 2.0), Point(6.0, 4.25, 2.0), Point(5.0, 5.0, 0.0),
        };
        NurbsSurface surface = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

        // JSON object
        nlohmann::ordered_json json = surface.jsondump();
        NurbsSurface loaded_json = NurbsSurface::jsonload(json);

        // String
        std::string json_string = surface.json_dumps();
        NurbsSurface loaded_json_string = NurbsSurface::json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbssurface.json").string();
        surface.json_dump(filename);
        NurbsSurface loaded_from_file = NurbsSurface::json_load(filename);

        MINI_CHECK(loaded_json == surface);
        MINI_CHECK(loaded_json_string == surface);
        MINI_CHECK(loaded_from_file == surface);
    }

    MINI_TEST("NurbsSurface", "Protobuf_roundtrip") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(-1.0, 0.75, 2.0), Point(-1.0, 4.25, 2.0), Point(0.0, 5.0, 0.0),
            Point(0.75, -1.0, 2.0), Point(1.25, 1.25, 4.0), Point(1.25, 3.75, 4.0), Point(0.75, 6.0, 2.0),
            Point(4.25, -1.0, 2.0), Point(3.75, 1.25, 4.0), Point(3.75, 3.75, 4.0), Point(4.25, 6.0, 2.0),
            Point(5.0, 0.0, 0.0), Point(6.0, 0.75, 2.0), Point(6.0, 4.25, 2.0), Point(5.0, 5.0, 0.0),
        };
        NurbsSurface surface = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

        // String
        std::string proto_string = surface.pb_dumps();
        NurbsSurface loaded_proto_string = NurbsSurface::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbssurface.bin").string();
        surface.pb_dump(filename);
        NurbsSurface loaded = NurbsSurface::pb_load(filename);

        MINI_CHECK(loaded_proto_string == surface);
        MINI_CHECK(loaded == surface);
    }

}
