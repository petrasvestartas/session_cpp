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

        // Test Surace validity
        bool is_valid = s.is_valid();

        // Check the dimentions of a surface
        // Mostly 3d
        // But 2d can be used for: scalar field over parameter space e.g. czrvatzre map, distance field
        // Planar geometry: texture coordinates
        int dimensions = s.dimension();


        

        // // Test knot access and set knot
        // surf.set_knot(0, 2, 5.0);
        // double new_val = surf.knot(0, 2);

        // MINI_CHECK(surf.dimension() == 3);
        // MINI_CHECK(!surf.is_rational());
        // MINI_CHECK(surf.order(0) == 4);
        // MINI_CHECK(surf.order(1) == 3);
        // MINI_CHECK(surf.degree(0) == 3);
        // MINI_CHECK(surf.degree(1) == 2);
        // MINI_CHECK(surf.cv_count(0) == 5);
        // MINI_CHECK(surf.cv_count(1) == 4);
        // MINI_CHECK(surf.cv_count() == 20);
        // MINI_CHECK(surf.cv_size() == 3);
        // MINI_CHECK(surf.knot_count(0) == 7);
        // MINI_CHECK(surf.knot_count(1) == 5);
        // MINI_CHECK(surf.span_count(0) == 2);
        // MINI_CHECK(surf.span_count(1) == 2);
        // MINI_CHECK(new_val == 5.0);
    }

    MINI_TEST("NurbsSurface", "Knot_operations") {
        // uncomment #include "nurbssurface.h"

        std::vector<Point> points;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

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

    MINI_TEST("NurbsSurface", "Rational_operations") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create non-rational surface, then make rational
        std::vector<Point> points(9, Point(0.0, 0.0, 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);

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

    MINI_TEST("NurbsSurface", "Evaluation") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

        // Create simple bilinear surface
        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0),
            Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

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

    MINI_TEST("NurbsSurface", "Geometric_queries") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create and setup surface
        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0),
            Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(!surf.is_closed(0));
        MINI_CHECK(!surf.is_closed(1));
        MINI_CHECK(!surf.is_periodic(0));
        MINI_CHECK(!surf.is_periodic(1));
        MINI_CHECK(surf.is_clamped(0, 0));
        MINI_CHECK(surf.is_clamped(1, 0));
        MINI_CHECK(surf.is_planar(nullptr, 1e-6));
    }

    MINI_TEST("NurbsSurface", "Modification") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0), Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0), Point(2.0, 1.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 3, 2, points);

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

    MINI_TEST("NurbsSurface", "Isocurve") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create surface
        std::vector<Point> points;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);

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

    MINI_TEST("NurbsSurface", "Transformation") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"

        // Create simple surface
        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0),
            Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

        // Apply translation
        Xform xf = Xform::translation(1.0, 2.0, 3.0);
        surf.transform(xf);

        // Check transformed CV
        Point pt = surf.get_cv(0, 0);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 3.0));
    }

    MINI_TEST("NurbsSurface", "Json_roundtrip") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        std::vector<Point> points;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.facecolors.push_back(Color(255, 128, 64, 255));
        surf.pointcolors.push_back(Color(0, 255, 0, 255));
        surf.linecolors.push_back(Color(0, 0, 255, 255));

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

    MINI_TEST("NurbsSurface", "Protobuf_roundtrip") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        std::vector<Point> points;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.facecolors.push_back(Color(255, 128, 64, 255));
        surf.pointcolors.push_back(Color(0, 255, 0, 255));
        surf.linecolors.push_back(Color(0, 0, 255, 255));

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

    MINI_TEST("NurbsSurface", "Advanced_accessors") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create rational surface for testing get_cv_4d/set_cv_4d
        std::vector<Point> points(9, Point(0.0, 0.0, 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
        surf.make_rational();

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

    MINI_TEST("NurbsSurface", "Clamp_operations") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        std::vector<Point> points;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

        surf.clamp_end(0, 2);

        MINI_CHECK(surf.is_valid());
        MINI_CHECK(surf.is_clamped(0, 2));
    }

    MINI_TEST("NurbsSurface", "Singularity") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create a simple surface with all CVs at different points (non-singular)
        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0),
            Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

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

    MINI_TEST("NurbsSurface", "Bounding_box") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        std::vector<Point> points;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 3, 3, points);

        // Get bounding box
        BoundingBox bbox = surf.get_bounding_box();

        MINI_CHECK(surf.is_valid());
    }

    MINI_TEST("NurbsSurface", "Domain_operations") {
        // uncomment #include "nurbssurface.h"

        std::vector<Point> points(9, Point(0.0, 0.0, 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);

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

    MINI_TEST("NurbsSurface", "Corner_points") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0), Point(0.0, 10.0, 0.0),
            Point(10.0, 0.0, 0.0), Point(10.0, 10.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

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

    MINI_TEST("NurbsSurface", "Swap_coordinates") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(1.0, 2.0, 3.0), Point(0.0, 0.0, 0.0),
            Point(0.0, 0.0, 0.0), Point(0.0, 0.0, 0.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

        // Swap X and Y
        surf.swap_coordinates(0, 1);

        Point pt = surf.get_cv(0, 0);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 3.0));
    }

    MINI_TEST("NurbsSurface", "Zero_cvs") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(1.0, 2.0, 3.0), Point(0.0, 0.0, 0.0),
            Point(0.0, 0.0, 0.0), Point(4.0, 5.0, 6.0),
        };
        NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, points);

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

    MINI_TEST("NurbsSurface", "Get_knots") {
        // uncomment #include "nurbssurface.h"

        std::vector<Point> points;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 3; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 3, 2, 4, 3, points);

        auto knots_u = surf.get_knots(0);
        auto knots_v = surf.get_knots(1);

        MINI_CHECK(knots_u.size() == static_cast<size_t>(surf.knot_count(0)));
        MINI_CHECK(knots_v.size() == static_cast<size_t>(surf.knot_count(1)));
        MINI_CHECK(knots_u.size() > 0);
        MINI_CHECK(knots_v.size() > 0);
    }

    MINI_TEST("NurbsSurface", "Make_non_rational") {
        // uncomment #include "nurbssurface.h"
        // uncomment #include "point.h"

        // Create surface, then make rational with all weights = 1
        std::vector<Point> points;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
        surf.make_rational();

        // Set all weights to 1.0
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_weight(i, j, 1.0);
            }
        }

        bool was_rational = surf.is_rational();
        surf.make_non_rational();
        bool is_rational_after = surf.is_rational();

        MINI_CHECK(was_rational);
        MINI_CHECK(!is_rational_after);
    }

    MINI_TEST("NurbsSurface", "Create_clamped_uniform") {
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

    MINI_TEST("NurbsSurface", "Knot_multiplicity") {
        // uncomment #include "nurbssurface.h"

        std::vector<Point> points;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                points.push_back(Point(static_cast<double>(i), static_cast<double>(j), 0.0));
        NurbsSurface surf = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

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

}
