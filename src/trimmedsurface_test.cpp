#include "mini_test.h"
#include "trimmedsurface.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "primitives.h"
#include "mesh.h"
#include "color.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("TrimmedSurface", "constructor") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        // Create a bilinear surface
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, Point(6.0, 0.0, 0.0));
        srf.set_cv(0, 1, Point(0.0, 6.0, 0.0));
        srf.set_cv(1, 1, Point(6.0, 6.0, 0.0));

        // Outer trim loop (rectangle in UV space)
        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0.0), Point(0.9, 0.1, 0.0),
            Point(0.9, 0.9, 0.0), Point(0.1, 0.9, 0.0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);

        // String representations
        std::string sstr = ts.str();
        std::string srepr = ts.repr();

        // Copy (new guid)
        TrimmedSurface tscopy = ts;

        MINI_CHECK(ts.is_valid());
        MINI_CHECK(ts.is_trimmed());
        MINI_CHECK(ts.name == "my_trimmedsurface");
        MINI_CHECK(!ts.guid.empty());
        MINI_CHECK(sstr.find("TrimmedSurface") != std::string::npos);
        MINI_CHECK(srepr.find("name=my_trimmedsurface") != std::string::npos);
        MINI_CHECK(tscopy.is_valid());
        MINI_CHECK(tscopy.guid != ts.guid);
        MINI_CHECK(tscopy == ts);
    }

    MINI_TEST("TrimmedSurface", "constructor_planar") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        // Planar curve boundary
        std::vector<Point> pts = {Point(0,0,0), Point(3,1,0), Point(5,0.5,0), Point(6,3,0), Point(4,5,0), Point(1,4,0)};
        NurbsCurve bnd = NurbsCurve::create(true, 3, pts);
        TrimmedSurface ts = TrimmedSurface::create_planar(bnd);

        // Rotated planar
        pts = {Point(0,0,0), Point(3,1,-2), Point(5,2,-3), Point(4,4,0), Point(1,3,2)};
        bnd = NurbsCurve::create(true, 3, pts);
        ts = TrimmedSurface::create_planar(bnd);

        // Triangle
        bnd = NurbsCurve::create(true, 1, {Point(0,0,0), Point(6,3,3), Point(2,5,1)});
        ts = TrimmedSurface::create_planar(bnd);

        // Trapezoid
        bnd = NurbsCurve::create(true, 1, {Point(0,0,6), Point(5,0,6), Point(4,4,2), Point(1,4,2)});
        ts = TrimmedSurface::create_planar(bnd);

        // Rectangle with a hole
        bnd = NurbsCurve::create(true, 1, {Point(0,0,0), Point(6,0,0), Point(6,6,0), Point(0,6,0)});
        ts = TrimmedSurface::create_planar(bnd);
        ts.add_hole(NurbsCurve::create(true, 1, {Point(2,2,0), Point(4,2,0), Point(4,4,0), Point(2,4,0)}));

        // Hexagon with 2 holes
        double R = 4.0;
        pts = {};
        for (int k = 0; k < 6; ++k) {
            double a = k * TOLERANCE.PI / 3.0;
            pts.push_back(Point(R*cos(a), R*sin(a), R*cos(a)*0.5));
        }
        bnd = NurbsCurve::create(true, 1, pts);
        ts = TrimmedSurface::create_planar(bnd);
        ts.add_holes({
            NurbsCurve::create(true, 1, {Point(1.5,0.5,0.75), Point(2.5,0.5,1.25), Point(2.0,1.5,1.0)}),
            NurbsCurve::create(true, 1, {Point(-2,-0.5,-1), Point(-1,-0.5,-0.5), Point(-1,-1.5,-0.5), Point(-2,-1.5,-1)})
        });
    }

    MINI_TEST("TrimmedSurface", "constructor_hole") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "primitives.h"

        // Create surface with bump
        int n = 8;
        std::vector<Point> pts;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                double x = i, y = j;
                double r2 = (x - 1.5) * (x - 1.5) + (y - 1.5) * (y - 1.5);
                double z = 5.0 * exp(-r2 / 1.0) + 0.3 * sin(Tolerance::PI * x / 7) * sin(Tolerance::PI * y / 7);
                pts.push_back(Point(x, y, z));
            }

        NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, n, n, pts);

        // Create outer loop (full boundary in UV)
        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);

        // Add hole
        NurbsCurve hole = Primitives::circle(3.5, 3.5, 0, 1.0);
        ts.add_hole(hole);

        MINI_CHECK(ts.is_valid());
        MINI_CHECK(ts.is_trimmed());
        MINI_CHECK(ts.inner_loop_count() == 1);
    }

    MINI_TEST("TrimmedSurface", "accessors") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(5, 0, 0));
        srf.set_cv(0, 1, Point(0, 5, 0));
        srf.set_cv(1, 1, Point(5, 5, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0), Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0), Point(0.1, 0.9, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);
        ts.name = "test_accessors";
        ts.width = 2.5;

        NurbsSurface got_srf = ts.surface();
        NurbsCurve got_loop = ts.get_outer_loop();

        MINI_CHECK(ts.is_valid());
        MINI_CHECK(ts.is_trimmed());
        MINI_CHECK(ts.name == "test_accessors");
        MINI_CHECK(ts.width == 2.5);
        MINI_CHECK(got_srf.is_valid());
        MINI_CHECK(got_loop.is_valid());
        MINI_CHECK(ts.inner_loop_count() == 0);
    }

    MINI_TEST("TrimmedSurface", "add_inner_loop") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(10, 0, 0));
        srf.set_cv(0, 1, Point(0, 10, 0));
        srf.set_cv(1, 1, Point(10, 10, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);

        // Add inner loop (hole in UV)
        NurbsCurve hole1 = NurbsCurve::create(true, 1, {
            Point(0.2, 0.2, 0), Point(0.4, 0.2, 0),
            Point(0.4, 0.4, 0), Point(0.2, 0.4, 0)
        });
        NurbsCurve hole2 = NurbsCurve::create(true, 1, {
            Point(0.6, 0.6, 0), Point(0.8, 0.6, 0),
            Point(0.8, 0.8, 0), Point(0.6, 0.8, 0)
        });

        ts.add_inner_loop(hole1);
        ts.add_inner_loop(hole2);

        NurbsCurve got = ts.get_inner_loop(0);

        MINI_CHECK(ts.inner_loop_count() == 2);
        MINI_CHECK(got.is_valid());

        ts.clear_inner_loops();
        MINI_CHECK(ts.inner_loop_count() == 0);
    }

    MINI_TEST("TrimmedSurface", "point_at") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(4, 0, 0));
        srf.set_cv(0, 1, Point(0, 4, 0));
        srf.set_cv(1, 1, Point(4, 4, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);

        auto dom_u = ts.surface().domain(0);
        auto dom_v = ts.surface().domain(1);
        double u_mid = (dom_u.first + dom_u.second) / 2.0;
        double v_mid = (dom_v.first + dom_v.second) / 2.0;

        Point pt = ts.point_at(u_mid, v_mid);
        Vector nm = ts.normal_at(u_mid, v_mid);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(std::abs(nm[2]), 1.0));
    }

    MINI_TEST("TrimmedSurface", "mesh") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "mesh.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(6, 0, 0));
        srf.set_cv(0, 1, Point(0, 6, 0));
        srf.set_cv(1, 1, Point(6, 6, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.05, 0.05, 0), Point(0.95, 0.05, 0),
            Point(0.95, 0.95, 0), Point(0.05, 0.95, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);
        Mesh m = ts.mesh();

        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("TrimmedSurface", "transformation") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0));
        srf.set_cv(1, 1, Point(1, 1, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);
        ts.xform = Xform::translation(10.0, 20.0, 30.0);
        TrimmedSurface ts2 = ts.transformed();

        auto dom_u = ts2.surface().domain(0);
        auto dom_v = ts2.surface().domain(1);
        Point pt = ts2.point_at(dom_u.first, dom_v.first);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 10.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 20.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 30.0));
    }

    MINI_TEST("TrimmedSurface", "json_roundtrip") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(5, 0, 0));
        srf.set_cv(0, 1, Point(0, 5, 0));
        srf.set_cv(1, 1, Point(5, 5, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0), Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0), Point(0.1, 0.9, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);
        ts.name = "test_trimmedsurface";
        ts.width = 2.0;
        ts.surfacecolor = Color(255, 128, 64, 255);

        // JSON object
        nlohmann::ordered_json json = ts.jsondump();
        TrimmedSurface loaded_json = TrimmedSurface::jsonload(json);

        // String
        std::string json_string = ts.json_dumps();
        TrimmedSurface loaded_json_string = TrimmedSurface::json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_trimmedsurface.json").string();
        ts.json_dump(filename);
        TrimmedSurface loaded_from_file = TrimmedSurface::json_load(filename);

        MINI_CHECK(loaded_json == ts);
        MINI_CHECK(loaded_json_string == ts);
        MINI_CHECK(loaded_from_file == ts);
    }

    MINI_TEST("TrimmedSurface", "protobuf_roundtrip") {
        // uncomment #include "trimmedsurface.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(5, 0, 0));
        srf.set_cv(0, 1, Point(0, 5, 0));
        srf.set_cv(1, 1, Point(5, 5, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0), Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0), Point(0.1, 0.9, 0)
        });

        TrimmedSurface ts = TrimmedSurface::create(srf, outer);
        ts.name = "test_trimmedsurface";
        ts.width = 2.0;
        ts.surfacecolor = Color(255, 128, 64, 255);

        // String
        std::string proto_string = ts.pb_dumps();
        TrimmedSurface loaded_proto_string = TrimmedSurface::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_trimmedsurface.bin").string();
        ts.pb_dump(filename);
        TrimmedSurface loaded = TrimmedSurface::pb_load(filename);

        MINI_CHECK(loaded_proto_string == ts);
        MINI_CHECK(loaded == ts);
    }

} // namespace session_cpp
