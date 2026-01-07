#include "mini_test.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "color.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"

#include <cmath>
#include <filesystem>
 #include <sstream>

using namespace session_cpp::mini_test;

namespace session_cpp {

    static NurbsSurface make_test_nurbssurface() {
        NurbsSurface surf;
        surf.create(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.surfacecolor = Color(255, 128, 64, 255);

        for (int i = 0; i < surf.cv_count(0); ++i) {
            for (int j = 0; j < surf.cv_count(1); ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), static_cast<double>(i + j)));
            }
        }

        return surf;
    }

    MINI_TEST("NurbsSurface", "constructor") {
        // Create surface with parameters (4x4 quadratic surface, order 3)
        NurbsSurface s;
        s.create(3, false, 3, 3, 4, 4, false, false, 2.5, 2.5);

        // Set hardcoded control points
        std::vector<Point> cvs = {
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

        // Setters
        int idx = 0;
        for (int i = 0; i < s.cv_count(0); ++i) {
            for (int j = 0; j < s.cv_count(1); ++j) {
                s.set_cv(i, j, cvs[idx]);
                idx++;
            }
        }

        // Getters
        Point control_point = s.get_cv(2, 1);  // 3.75, 1.25, 4.0
        Point point = s.point_at(2.5, 2.5);    // 2.5, 2.5, 4.0

        // String representation
        std::string str_repr = s.to_string();

        // Duplicate for comparison
        NurbsSurface s_copy = s;

        // Subdivision test
        auto v = s.subdivide(5, 5);
        MINI_CHECK(s.name == "my_nurbssurface");
        MINI_CHECK(s.width == 1.0);
        MINI_CHECK(s.surfacecolor == Color::white());
        MINI_CHECK(!s.guid.empty());
        MINI_CHECK(s.m_dim == 3);
        MINI_CHECK(!s.m_is_rat);
        MINI_CHECK(s.dimension() == 3);
        MINI_CHECK(!s.is_rational());
        MINI_CHECK(s.order(0) == 3);
        MINI_CHECK(s.order(1) == 3);
        MINI_CHECK(s.degree(0) == 2);
        MINI_CHECK(s.degree(1) == 2);
        MINI_CHECK(s.cv_count(0) == 4);
        MINI_CHECK(s.cv_count(1) == 4);
        MINI_CHECK(s.cv_count() == 16);
        MINI_CHECK(s.knot_count(0) == 5);
        MINI_CHECK(s.knot_count(1) == 5);
        MINI_CHECK(control_point[0] == 3.75 && control_point[1] == 1.25 && control_point[2] == 4.0);
        MINI_CHECK(point[0] == 2.5 && point[1] == 2.5 && point[2] == 4.0);
        MINI_CHECK(str_repr == "NurbsSurface(dim=3, order=(3,3), cv_count=(4,4))");
        MINI_CHECK(s_copy == s);
        MINI_CHECK(s_copy.name == s.name);
        MINI_CHECK(s_copy.width == s.width);
        MINI_CHECK(s_copy.surfacecolor == s.surfacecolor);
        MINI_CHECK(s_copy.guid != s.guid);
        // Helper lambda for tolerance-based point comparison
        auto close_pt = [](const Point& a, double x, double y, double z) {
            return TOLERANCE.is_close(a[0], x) && TOLERANCE.is_close(a[1], y) && TOLERANCE.is_close(a[2], z);
        };
        MINI_CHECK(close_pt(v[0][0], 0.0, 0.0, 0.0));
        MINI_CHECK(close_pt(v[0][1], -0.64, 0.76, 1.28));
        MINI_CHECK(close_pt(v[0][2], -0.96, 1.84, 1.92));
        MINI_CHECK(close_pt(v[0][3], -0.96, 3.16, 1.92));
        MINI_CHECK(close_pt(v[0][4], -0.64, 4.24, 1.28));
        MINI_CHECK(close_pt(v[0][5], 0.0, 5.0, 0.0));
        MINI_CHECK(close_pt(v[1][0], 0.76, -0.64, 1.28));
        MINI_CHECK(close_pt(v[1][1], 0.6832, 0.6832, 2.56));
        MINI_CHECK(close_pt(v[1][2], 0.6448, 1.9168, 3.2));
        MINI_CHECK(close_pt(v[1][3], 0.6448, 3.0832, 3.2));
        MINI_CHECK(close_pt(v[1][4], 0.6832, 4.3168, 2.56));
        MINI_CHECK(close_pt(v[1][5], 0.76, 5.64, 1.28));
        MINI_CHECK(close_pt(v[2][0], 1.84, -0.96, 1.92));
        MINI_CHECK(close_pt(v[2][1], 1.9168, 0.6448, 3.2));
        MINI_CHECK(close_pt(v[2][2], 1.9552, 1.9552, 3.84));
        MINI_CHECK(close_pt(v[2][3], 1.9552, 3.0448, 3.84));
        MINI_CHECK(close_pt(v[2][4], 1.9168, 4.3552, 3.2));
        MINI_CHECK(close_pt(v[2][5], 1.84, 5.96, 1.92));
        MINI_CHECK(close_pt(v[3][0], 3.16, -0.96, 1.92));
        MINI_CHECK(close_pt(v[3][1], 3.0832, 0.6448, 3.2));
        MINI_CHECK(close_pt(v[3][2], 3.0448, 1.9552, 3.84));
        MINI_CHECK(close_pt(v[3][3], 3.0448, 3.0448, 3.84));
        MINI_CHECK(close_pt(v[3][4], 3.0832, 4.3552, 3.2));
        MINI_CHECK(close_pt(v[3][5], 3.16, 5.96, 1.92));
        MINI_CHECK(close_pt(v[4][0], 4.24, -0.64, 1.28));
        MINI_CHECK(close_pt(v[4][1], 4.3168, 0.6832, 2.56));
        MINI_CHECK(close_pt(v[4][2], 4.3552, 1.9168, 3.2));
        MINI_CHECK(close_pt(v[4][3], 4.3552, 3.0832, 3.2));
        MINI_CHECK(close_pt(v[4][4], 4.3168, 4.3168, 2.56));
        MINI_CHECK(close_pt(v[4][5], 4.24, 5.64, 1.28));
        MINI_CHECK(close_pt(v[5][0], 5.0, 0.0, 0.0));
        MINI_CHECK(close_pt(v[5][1], 5.64, 0.76, 1.28));
        MINI_CHECK(close_pt(v[5][2], 5.96, 1.84, 1.92));
        MINI_CHECK(close_pt(v[5][3], 5.96, 3.16, 1.92));
        MINI_CHECK(close_pt(v[5][4], 5.64, 4.24, 1.28));
        MINI_CHECK(close_pt(v[5][5], 5.0, 5.0, 0.0));
    }

    MINI_TEST("NurbsSurface", "create_operations") {
        // Create a simple 2x2 bilinear surface
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 4, 3, 5, 4, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);

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
        // Create non-rational surface
        NurbsSurface surf;
        surf.create(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);

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
        // Create simple bilinear surface
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        // Create and setup surface
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 3, 2, false, false, 1.0, 1.0);

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
        // Create surface
        NurbsSurface surf;
        surf.create(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);

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
        // Create simple surface
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        // Create and setup surface
        NurbsSurface surf;
        surf.create(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.surfacecolor = Color(255, 128, 64, 255);

        // Set some CVs
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        // Serialize to JSON
        std::string fname = "serialization/test_nurbssurface.json";
        surf.json_dump(fname);
        NurbsSurface loaded = NurbsSurface::json_load(fname);

        MINI_CHECK(loaded.name == surf.name);
        MINI_CHECK(TOLERANCE.is_close(loaded.width, surf.width));
        MINI_CHECK(loaded.surfacecolor[0] == surf.surfacecolor[0]);
        MINI_CHECK(loaded.surfacecolor[1] == surf.surfacecolor[1]);
        MINI_CHECK(loaded.surfacecolor[2] == surf.surfacecolor[2]);
        MINI_CHECK(loaded.dimension() == surf.dimension());
        MINI_CHECK(loaded.is_rational() == surf.is_rational());
        MINI_CHECK(loaded.order(0) == surf.order(0));
        MINI_CHECK(loaded.order(1) == surf.order(1));
        MINI_CHECK(loaded.cv_count(0) == surf.cv_count(0));
        MINI_CHECK(loaded.cv_count(1) == surf.cv_count(1));
    }

    MINI_TEST("NurbsSurface", "protobuf_roundtrip") {
        // Create and setup surface
        NurbsSurface surf;
        surf.create(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);
        surf.name = "test_nurbssurface";
        surf.width = 2.0;
        surf.surfacecolor = Color(255, 128, 64, 255);

        // Set some CVs
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                surf.set_cv(i, j, Point(static_cast<double>(i), static_cast<double>(j), 0.0));
            }
        }

        // Serialize to protobuf
        std::string path = "serialization/test_nurbssurface.bin";
        surf.protobuf_dump(path);
        NurbsSurface loaded = NurbsSurface::protobuf_load(path);

        MINI_CHECK(loaded.name == surf.name);
        MINI_CHECK(TOLERANCE.is_close(loaded.width, surf.width));
        MINI_CHECK(loaded.surfacecolor[0] == surf.surfacecolor[0]);
        MINI_CHECK(loaded.surfacecolor[1] == surf.surfacecolor[1]);
        MINI_CHECK(loaded.surfacecolor[2] == surf.surfacecolor[2]);
        MINI_CHECK(loaded.dimension() == surf.dimension());
        MINI_CHECK(loaded.is_rational() == surf.is_rational());
        MINI_CHECK(loaded.order(0) == surf.order(0));
        MINI_CHECK(loaded.order(1) == surf.order(1));
        MINI_CHECK(loaded.cv_count(0) == surf.cv_count(0));
        MINI_CHECK(loaded.cv_count(1) == surf.cv_count(1));
    }

    MINI_TEST("NurbsSurface", "advanced_accessors") {
        // Create rational surface for testing get_cv_4d/set_cv_4d
        NurbsSurface surf;
        surf.create(3, true, 3, 3, 3, 3, false, false, 1.0, 1.0);

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

        // Test cv_capacity and knot_capacity
        int cv_cap = surf.cv_capacity();
        int knot_cap_u = surf.knot_capacity(0);
        int knot_cap_v = surf.knot_capacity(1);

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
        MINI_CHECK(cv_cap > 0);
        MINI_CHECK(knot_cap_u > 0);
        MINI_CHECK(knot_cap_v > 0);
    }

    MINI_TEST("NurbsSurface", "clamp_operations") {
        NurbsSurface surf;
        surf.create(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);

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
        // Create a simple surface
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 3, 3, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        // Set a control point with distinct coordinates
        surf.set_cv(0, 0, Point(1.0, 2.0, 3.0));

        // Swap X and Y
        surf.swap_coordinates(0, 1);

        Point pt = surf.get_cv(0, 0);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 3.0));
    }

    MINI_TEST("NurbsSurface", "change_dimension") {
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

        surf.set_cv(0, 0, Point(1.0, 2.0, 3.0));

        int old_dim = surf.dimension();
        surf.change_dimension(2);
        int new_dim = surf.dimension();

        Point pt = surf.get_cv(0, 0);

        MINI_CHECK(old_dim == 3);
        MINI_CHECK(new_dim == 2);
        MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 2.0));
    }

    MINI_TEST("NurbsSurface", "zero_cvs") {
        NurbsSurface surf;
        surf.create(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 4, 3, 4, 3, false, false, 1.0, 2.0);

        auto knots_u = surf.get_knots(0);
        auto knots_v = surf.get_knots(1);

        MINI_CHECK(knots_u.size() == static_cast<size_t>(surf.knot_count(0)));
        MINI_CHECK(knots_v.size() == static_cast<size_t>(surf.knot_count(1)));
        MINI_CHECK(knots_u.size() > 0);
        MINI_CHECK(knots_v.size() > 0);
    }

    MINI_TEST("NurbsSurface", "make_non_rational") {
        // Create rational surface with all weights = 1
        NurbsSurface surf;
        surf.create(3, true, 3, 3, 3, 3, false, false, 1.0, 1.0);

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
        NurbsSurface surf;
        surf.create(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);

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
        // Create a sphere as a rational NURBS surface
        // Sphere: 9x5 control points, degree 2 in both directions
        // Based on OpenNURBS sphere representation
        double radius = 2.0;
        double w = std::sqrt(2.0) / 2.0;  // 0.707107

        NurbsSurface surf;
        surf.create(3, true, 3, 3, 9, 5, false, false, 1.0, 1.0);
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
        // Create an uncapped cylinder as a rational NURBS surface
        // Cylinder: 9x2 control points (circle at bottom, circle at top)
        // Degree 2 in U (angular), degree 1 in V (height)
        double radius = 1.5;
        double height = 3.0;
        double w = std::sqrt(2.0) / 2.0;

        NurbsSurface surf;
        surf.create(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0);
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
        // Create a torus as a rational NURBS surface
        // Torus: 9x9 control points (minor circle revolved around major axis)
        // Degree 2 in both directions
        double major_radius = 3.0;  // distance from center to tube center
        double minor_radius = 1.0;  // tube radius
        double w = std::sqrt(2.0) / 2.0;

        NurbsSurface surf;
        surf.create(3, true, 3, 3, 9, 9, false, false, 1.0, 1.0);
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
        // Create an uncapped cone as a rational NURBS surface
        // Cone: 9x2 control points (apex at top, circle at base)
        // Degree 2 in U (angular), degree 1 in V (height)
        double radius = 2.0;
        double height = 4.0;
        double w = std::sqrt(2.0) / 2.0;

        NurbsSurface surf;
        surf.create(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0);
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

}
