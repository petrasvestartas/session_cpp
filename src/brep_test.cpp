#include "mini_test.h"
#include "brep.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "mesh.h"
#include "color.h"
#include "primitives.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("BRep", "Constructor") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"

        BRep b;

        // String representations
        std::string sstr = b.str();
        std::string srepr = b.repr();

        // Copy (new guid)
        BRep bcopy = b;

        MINI_CHECK(!b.is_valid());
        MINI_CHECK(b.face_count() == 0);
        MINI_CHECK(b.name == "my_brep");
        MINI_CHECK(!b.guid.empty());
        MINI_CHECK(sstr.find("BRep") != std::string::npos);
        MINI_CHECK(srepr.find("name=my_brep") != std::string::npos);
        MINI_CHECK(bcopy.guid != b.guid);
        MINI_CHECK(bcopy == b);
        MINI_CHECK(!(bcopy != b));
    }

    MINI_TEST("BRep", "Create Box") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);

        MINI_CHECK(box.is_valid());
        MINI_CHECK(box.face_count() == 6);
        MINI_CHECK(box.edge_count() == 12);
        MINI_CHECK(box.vertex_count() == 8);
        MINI_CHECK(box.is_solid());
        MINI_CHECK(box.name == "box");
    }

    MINI_TEST("BRep", "Accessors") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"
        // uncomment #include "nurbssurface.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);

        int fc = box.face_count();
        int ec = box.edge_count();
        int vc = box.vertex_count();

        MINI_CHECK(fc == 6);
        MINI_CHECK(ec == 12);
        MINI_CHECK(vc == 8);
        MINI_CHECK((int)box.m_surfaces.size() == 6);
        MINI_CHECK((int)box.m_loops.size() == 6);
        MINI_CHECK((int)box.m_trims.size() == 24);
    }

    MINI_TEST("BRep", "Add Face") {
        // uncomment #include "brep.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        BRep b;
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0)); srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0)); srf.set_cv(1, 1, Point(1, 1, 0));

        int si = b.add_surface(srf);
        int fi = b.add_face(si, false);
        int li = b.add_loop(fi, BRepLoopType::Outer);

        NurbsCurve trim = NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(1, 0, 0)});
        int ci = b.add_curve_2d(trim);
        b.add_trim(ci, -1, li, false, BRepTrimType::Boundary);

        MINI_CHECK(b.face_count() == 1);
        MINI_CHECK((int)b.m_surfaces.size() == 1);
        MINI_CHECK((int)b.m_loops.size() == 1);
        MINI_CHECK((int)b.m_trims.size() == 1);
    }

    MINI_TEST("BRep", "Mesh") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        Mesh m = box.mesh();

        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "Point At") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"
        // uncomment #include "tolerance.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        Point pt = box.point_at(0, 0.5, 0.5);

        MINI_CHECK(std::abs(pt[2] + 2.0) < 0.01 || std::abs(pt[2] - 2.0) < 0.01 ||
                   std::abs(pt[1] + 1.5) < 0.01 || std::abs(pt[1] - 1.5) < 0.01 ||
                   std::abs(pt[0] + 1.0) < 0.01 || std::abs(pt[0] - 1.0) < 0.01);
    }

    MINI_TEST("BRep", "Is Solid") {
        // uncomment #include "brep.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        // Box is solid
        BRep box = BRep::create_box(2.0, 3.0, 4.0);

        // Single face is not solid
        BRep single;
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0)); srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0)); srf.set_cv(1, 1, Point(1, 1, 0));
        int si = single.add_surface(srf);
        single.add_face(si, false);
        single.add_vertex(Point(0, 0, 0));

        MINI_CHECK(box.is_solid());
        MINI_CHECK(!single.is_solid());
    }

    MINI_TEST("BRep", "Transformation") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"
        // uncomment #include "tolerance.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        box.xform = Xform::translation(10.0, 20.0, 30.0);
        BRep moved = box.transformed();

        Point pt = moved.point_at(0, 0.0, 0.0);
        Point pt_orig = box.point_at(0, 0.0, 0.0);

        MINI_CHECK(std::abs(pt[0] - pt_orig[0] - 10.0) < 0.01);
        MINI_CHECK(std::abs(pt[1] - pt_orig[1] - 20.0) < 0.01);
        MINI_CHECK(std::abs(pt[2] - pt_orig[2] - 30.0) < 0.01);
    }

    MINI_TEST("BRep", "Json Roundtrip") {
        // uncomment #include "brep.h"
        // uncomment #include "color.h"
        // uncomment #include <filesystem>

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        box.name = "test_brep";
        box.width = 2.0;
        box.surfacecolor = Color(255, 128, 64, 255);

        // JSON object
        nlohmann::ordered_json json = box.jsondump();
        BRep loaded_json = BRep::jsonload(json);

        // String
        std::string json_string = box.json_dumps();
        BRep loaded_json_string = BRep::json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_brep.json").string();
        box.json_dump(filename);
        BRep loaded_from_file = BRep::json_load(filename);

        MINI_CHECK(loaded_json == box);
        MINI_CHECK(loaded_json_string == box);
        MINI_CHECK(loaded_from_file == box);
    }

    MINI_TEST("BRep", "Create Cylinder") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep cyl = BRep::create_cylinder(1.0, 2.0);
        Mesh m = cyl.mesh();

        MINI_CHECK(cyl.is_valid());
        MINI_CHECK(cyl.face_count() == 3);
        MINI_CHECK(cyl.is_solid());
        MINI_CHECK(cyl.name == "cylinder");
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
    }

    MINI_TEST("BRep", "Create Sphere") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep sph = BRep::create_sphere(2.0);
        Mesh m = sph.mesh();

        MINI_CHECK(sph.is_valid());
        MINI_CHECK(sph.face_count() == 1);
        MINI_CHECK(sph.is_solid());
        MINI_CHECK(sph.name == "sphere");
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
    }

    MINI_TEST("BRep", "From Polylines") {
        // uncomment #include "brep.h"
        // uncomment #include "polyline.h"
        // uncomment #include "mesh.h"

        double hx = 1.0, hy = 1.5, hz = 2.0;
        Point c[8] = {
            Point(-hx, -hy, -hz), Point( hx, -hy, -hz),
            Point( hx, hy, -hz), Point(-hx, hy, -hz),
            Point(-hx, -hy, hz), Point( hx, -hy, hz),
            Point( hx, hy, hz), Point(-hx, hy, hz)
        };

        Polyline bottom(std::vector<Point>{c[0],c[3],c[2],c[1],c[0]});
        Polyline top(std::vector<Point>{c[4],c[5],c[6],c[7],c[4]});
        Polyline front(std::vector<Point>{c[0],c[1],c[5],c[4],c[0]});
        Polyline right(std::vector<Point>{c[1],c[2],c[6],c[5],c[1]});
        Polyline back(std::vector<Point>{c[2],c[3],c[7],c[6],c[2]});
        Polyline left(std::vector<Point>{c[3],c[0],c[4],c[7],c[3]});

        BRep b = BRep::from_polylines({bottom, top, front, right, back, left});
        Mesh m = b.mesh();

        MINI_CHECK(b.is_valid());
        MINI_CHECK(b.face_count() == 6);
        MINI_CHECK(b.edge_count() == 12);
        MINI_CHECK(b.vertex_count() == 8);
        MINI_CHECK(b.is_solid());
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "From Nurbscurves") {
        // uncomment #include "brep.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "mesh.h"

        double hx = 1.0, hy = 1.5, hz = 2.0;
        Point c[8] = {
            Point(-hx, -hy, -hz), Point( hx, -hy, -hz),
            Point( hx, hy, -hz), Point(-hx, hy, -hz),
            Point(-hx, -hy, hz), Point( hx, -hy, hz),
            Point( hx, hy, hz), Point(-hx, hy, hz)
        };

        auto bottom = NurbsCurve::create(false, 1, {c[0],c[3],c[2],c[1],c[0]});
        auto top = NurbsCurve::create(false, 1, {c[4],c[5],c[6],c[7],c[4]});
        auto front = NurbsCurve::create(false, 1, {c[0],c[1],c[5],c[4],c[0]});
        auto right = NurbsCurve::create(false, 1, {c[1],c[2],c[6],c[5],c[1]});
        auto back = NurbsCurve::create(false, 1, {c[2],c[3],c[7],c[6],c[2]});
        auto left = NurbsCurve::create(false, 1, {c[3],c[0],c[4],c[7],c[3]});

        BRep b = BRep::from_nurbscurves({bottom, top, front, right, back, left});
        Mesh m = b.mesh();

        MINI_CHECK(b.is_valid());
        MINI_CHECK(b.face_count() == 6);
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "From Nurbscurves Holes") {
        // uncomment #include "brep.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "mesh.h"
        // uncomment #include "primitives.h"

        auto outer = NurbsCurve::create(false, 1, {
            Point(-5, -5, 0), Point(5, -5, 0), Point(5, 5, 0), Point(-5, 5, 0), Point(-5, -5, 0)});
        auto hole = Primitives::circle(0.0, 0.0, 0.0, 2.0);

        BRep b = BRep::from_nurbscurves({outer}, {{hole}});
        Mesh m = b.mesh();

        MINI_CHECK(b.is_valid());
        MINI_CHECK(b.face_count() == 1);
        MINI_CHECK(b.m_loops.size() == 2);
        MINI_CHECK(b.m_loops[0].type == BRepLoopType::Outer);
        MINI_CHECK(b.m_loops[1].type == BRepLoopType::Inner);
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "Create Block With Hole") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        Mesh m = bh.mesh();

        MINI_CHECK(bh.is_valid());
        MINI_CHECK(bh.face_count() == 7);
        MINI_CHECK(bh.name == "block_with_hole");
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "Protobuf Roundtrip") {
        // uncomment #include "brep.h"
        // uncomment #include "color.h"
        // uncomment #include <filesystem>

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        box.name = "test_brep";
        box.width = 2.0;
        box.surfacecolor = Color(255, 128, 64, 255);

        // String
        std::string proto_string = box.pb_dumps();
        BRep loaded_proto_string = BRep::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_brep.bin").string();
        box.pb_dump(filename);
        BRep loaded = BRep::pb_load(filename);

        MINI_CHECK(loaded_proto_string == box);
        MINI_CHECK(loaded == box);
    }

} // namespace session_cpp
