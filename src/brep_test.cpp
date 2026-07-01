#include "mini_test.h"
#include "brep.h"
#include "intersection.h"
#include "closest.h"
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
#include "plane.h"
#include "line.h"

#include <cmath>
#include <filesystem>
#include <chrono>
#include <iostream>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("BRep", "Constructor") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"

        BRep b;

        // String representations
        std::string sstr = b.str();
        std::string srepr = b.repr();

        // Copy (new guid())
        BRep bcopy = b;

        MINI_CHECK(!b.is_valid());
        MINI_CHECK(b.face_count() == 0);
        MINI_CHECK(b.name == "my_brep");
        MINI_CHECK(!b.guid().empty());
        MINI_CHECK(sstr.find("BRep") != std::string::npos);
        MINI_CHECK(srepr.find("name=my_brep") != std::string::npos);
        MINI_CHECK(bcopy.guid() != b.guid());
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

        NurbsCurve trim = NurbsCurve::create(false, 1, {
            Point(0, 0, 0),
            Point(1, 0, 0),
        });
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
        std::string json_string = box.file_json_dumps();
        BRep loaded_json_string = BRep::file_json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_brep.json").string();
        box.file_json_dump(filename);
        BRep loaded_from_file = BRep::file_json_load(filename);

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

    MINI_TEST("BRep", "Create Cone") {
        const double PI = 3.14159265358979323846;
        BRep cone = BRep::create_cone(1.0, 2.0);   // base r=1 at z=0, apex z=2
        Mesh m = cone.mesh();
        MINI_CHECK(cone.is_valid());
        MINI_CHECK(cone.face_count() == 2);           // side + base cap
        MINI_CHECK(cone.is_solid());
        MINI_CHECK(cone.name == "cone");
        MINI_CHECK(!m.is_empty());
        // V = (1/3) pi r^2 h
        MINI_CHECK(std::abs(cone.volume() - (PI * 1.0 * 2.0 / 3.0)) / (PI * 2.0 / 3.0) < 1e-4);
    }

    MINI_TEST("BRep", "Create Torus") {
        const double PI = 3.14159265358979323846;
        BRep tor = BRep::create_torus(2.0, 0.5);   // major R=2, minor r=0.5
        Mesh m = tor.mesh();
        MINI_CHECK(tor.is_valid());
        MINI_CHECK(tor.face_count() == 1);
        MINI_CHECK(tor.is_solid());
        MINI_CHECK(tor.name == "torus");
        MINI_CHECK(!m.is_empty());
        // V = 2 pi^2 R r^2
        MINI_CHECK(std::abs(tor.volume() - (2.0 * PI * PI * 2.0 * 0.25)) / (2.0 * PI * PI * 2.0 * 0.25) < 1e-3);
    }

    MINI_TEST("BRep", "From Polylines") {
        // uncomment #include "brep.h"
        // uncomment #include "polyline.h"
        // uncomment #include "mesh.h"

        double hx = 1.0, hy = 1.5, hz = 2.0;
        Point c[8] = {
            Point(-hx, -hy, -hz),
            Point( hx, -hy, -hz),
            Point( hx,  hy, -hz),
            Point(-hx,  hy, -hz),
            Point(-hx, -hy,  hz),
            Point( hx, -hy,  hz),
            Point( hx,  hy,  hz),
            Point(-hx,  hy,  hz),
        };

        Polyline bottom(std::vector<Point>{
            c[0],
            c[3],
            c[2],
            c[1],
            c[0],
        });
        Polyline top(std::vector<Point>{
            c[4],
            c[5],
            c[6],
            c[7],
            c[4],
        });
        Polyline front(std::vector<Point>{
            c[0],
            c[1],
            c[5],
            c[4],
            c[0],
        });
        Polyline right(std::vector<Point>{
            c[1],
            c[2],
            c[6],
            c[5],
            c[1],
        });
        Polyline back(std::vector<Point>{
            c[2],
            c[3],
            c[7],
            c[6],
            c[2],
        });
        Polyline left(std::vector<Point>{
            c[3],
            c[0],
            c[4],
            c[7],
            c[3],
        });

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
            Point(-hx, -hy, -hz),
            Point( hx, -hy, -hz),
            Point( hx,  hy, -hz),
            Point(-hx,  hy, -hz),
            Point(-hx, -hy,  hz),
            Point( hx, -hy,  hz),
            Point( hx,  hy,  hz),
            Point(-hx,  hy,  hz),
        };

        auto bottom = NurbsCurve::create(false, 1, {
            c[0],
            c[3],
            c[2],
            c[1],
            c[0],
        });
        auto top = NurbsCurve::create(false, 1, {
            c[4],
            c[5],
            c[6],
            c[7],
            c[4],
        });
        auto front = NurbsCurve::create(false, 1, {
            c[0],
            c[1],
            c[5],
            c[4],
            c[0],
        });
        auto right = NurbsCurve::create(false, 1, {
            c[1],
            c[2],
            c[6],
            c[5],
            c[1],
        });
        auto back = NurbsCurve::create(false, 1, {
            c[2],
            c[3],
            c[7],
            c[6],
            c[2],
        });
        auto left = NurbsCurve::create(false, 1, {
            c[3],
            c[0],
            c[4],
            c[7],
            c[3],
        });

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
            Point(-5, -5, 0),
            Point(5, -5, 0),
            Point(5, 5, 0),
            Point(-5, 5, 0),
            Point(-5, -5, 0),
        });
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
        // Analytic volume: the annular top/bottom faces' interior sample must land on the
        // MATERIAL (not in the hole) for the outward-sign probe. OCCT: sx*sy*sz - pi*r^2*sz.
        {
            const double PI = 3.14159265358979323846;
            double ref = 8.0*6.0*4.0 - PI*1.5*1.5*4.0;
            MINI_CHECK(std::abs(bh.volume() - ref) / ref < 1e-6);
            MINI_CHECK(bh.is_solid());
        }
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "Mesh Orientation") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        // Reversed faces must flip winding; the bug inflated volume() past the solid box.
        BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        double vol = bh.mesh().volume();

        MINI_CHECK(vol > 60.0);
        MINI_CHECK(vol < 175.0);
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

    MINI_TEST("BRep", "Split By Plane") {
        // uncomment #include "brep.h"
        // uncomment #include "plane.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

        BRep box = BRep::create_box(2.0, 2.0, 2.0);
        Point origin(0.0, 0.0, 0.0);
        Vector normal(0.0, 0.0, 1.0);
        Plane plane = Plane::from_point_normal(origin, normal);
        BRep split = box.split_by_plane(plane);
        double box_area = box.mesh().area();
        double split_area = split.mesh().area();
        int inner = 0;
        for (const auto& face : split.m_faces)
            for (int li : face.loop_indices)
                if (split.m_loops[li].type == BRepLoopType::Inner)
                    inner += 1;

        MINI_CHECK(split.face_count() == 10);
        MINI_CHECK(std::abs(split_area - box_area) < box_area * 0.01);
        MINI_CHECK(!split.mesh().is_empty());
        MINI_CHECK(inner == 0);

        BRep cylinder = BRep::create_cylinder(1.0, 4.0);
        Point mid_origin(0.0, 0.0, 1.0);
        Vector mid_normal(0.0, 0.0, 1.0);
        Plane mid = Plane::from_point_normal(mid_origin, mid_normal);
        BRep cut = cylinder.split_by_plane(mid);

        MINI_CHECK(cut.face_count() == 4);
        MINI_CHECK(std::abs(cut.mesh().area() - cylinder.mesh().area()) < cylinder.mesh().area() * 0.02);
    }

    MINI_TEST("BRep", "Split By Plane Pieces") {
        // uncomment #include "brep.h"
        // uncomment #include "plane.h"

        BRep box = BRep::create_box(2.0, 2.0, 2.0);
        Point origin(0.0, 0.0, 0.0);
        Vector normal(0.0, 0.0, 1.0);
        Plane plane = Plane::from_point_normal(origin, normal);
        std::vector<BRep> pieces = box.split_by_plane_pieces(plane);
        double total = 0.0;
        for (const auto& piece : pieces) total += piece.mesh().area();

        MINI_CHECK(pieces.size() == 2);
        MINI_CHECK(pieces[0].face_count() == 5);
        MINI_CHECK(pieces[1].face_count() == 5);
        MINI_CHECK(std::abs(total - box.mesh().area()) < box.mesh().area() * 0.01);

        Point far_o(0.0, 0.0, 5.0);
        Vector far_n(0.0, 0.0, 1.0);
        Plane far = Plane::from_point_normal(far_o, far_n);
        std::vector<BRep> whole = box.split_by_plane_pieces(far);

        MINI_CHECK(whole.size() == 1);
        MINI_CHECK(whole[0].face_count() == 6);
    }

    MINI_TEST("BRep", "Split By Line") {
        // uncomment #include "brep.h"
        // uncomment #include "line.h"
        // uncomment #include "point.h"

        BRep box = BRep::create_box(2.0, 2.0, 2.0);
        Line line = Line::from_points(Point(0.0, -2.0, 1.0), Point(0.0, 2.0, 1.0));
        BRep split = box.split_by_line(line);
        double box_area = box.mesh().area();
        double split_area = split.mesh().area();

        MINI_CHECK(split.face_count() == 7);
        MINI_CHECK(std::abs(split_area - box_area) < box_area * 0.01);
        MINI_CHECK(!split.mesh().is_empty());
    }

    MINI_TEST("BRep", "Split By Brep") {
        // uncomment #include "brep.h"

        BRep target = BRep::create_box(4.0, 4.0, 2.0);
        BRep cutter = BRep::create_box(2.0, 2.0, 6.0);
        BRep split = target.split_by_brep(cutter);
        double target_area = target.mesh().area();
        double split_area = split.mesh().area();

        MINI_CHECK(split.face_count() == 8);
        MINI_CHECK(std::abs(split_area - target_area) < target_area * 0.01);
        MINI_CHECK(!split.mesh().is_empty());
    }

    MINI_TEST("BRep", "Boolean") {
        const double PI = 3.14159265358979323846;
        // Box 4x4x4 centered (z in [-2,2]) with a cylinder r=1 running through it (z in [-3,3]).
        BRep box = BRep::create_box(4, 4, 4);
        BRep cyl = BRep::create_cylinder(1.0, 6.0);
        cyl.xform = Xform::translation(0, 0, -3);
        cyl = cyl.transformed();

        // imprint -> classify -> select -> sew. Validated vs OCCT BRepAlgoAPI_* (oracle):
        // face counts match exactly (cut=7, common=3, fuse=10) and -- with the exact rational
        // intersection circles preserved through the imprint and sewn watertight -- the volumes
        // match OCCT to machine precision and each result is a closed solid.
        BRep cut = box.boolean_difference(cyl);   // 64 - 4pi = 51.4336, 7 faces
        BRep com = box.boolean_intersection(cyl); // 4pi = 12.566, 3 faces
        BRep fus = box.boolean_union(cyl);         // 64 + 2pi = 70.283, 10 faces

        MINI_CHECK(cut.face_count() == 7);
        MINI_CHECK(com.face_count() == 3);
        MINI_CHECK(fus.face_count() == 10);
        // Watertight closed solids (every edge carries exactly two trims).
        MINI_CHECK(cut.is_solid());
        MINI_CHECK(com.is_solid());
        MINI_CHECK(fus.is_solid());
        // Volumes exact to OCCT (tolerance 1e-6 relative; the analytic circles are exact).
        MINI_CHECK(std::abs(cut.volume() - (64 - 4*PI)) / (64 - 4*PI) < 1e-6);
        MINI_CHECK(std::abs(com.volume() - (4*PI)) / (4*PI) < 1e-6);
        MINI_CHECK(std::abs(fus.volume() - (64 + 2*PI)) / (64 + 2*PI) < 1e-6);

        // Box(4) - Sphere(2.5): sphere pokes through every box face. The exact rational
        // intersection circles ARE preserved, but full exactness/watertightness for sphere
        // pieces still needs (a) volume integration over circular (non-rectangular) trims and
        // (b) pole/seam-aware sewing -- a follow-up iteration. Kept here as a timing case only.
        // OCCT reference (oracle): cut V=9.545724580842144 7f, common V=54.45427562996632 7f,
        // fuse V=74.99557383546609 13f.
        BRep sph2 = BRep::create_sphere(2.5);
        auto bench = [](const char* tag, const BRep& A, const BRep& B, double occt_us) {
            A.boolean_difference(B);  // warm up
            const int NB = 20;
            auto t0 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < NB; i++) { auto r = A.boolean_difference(B); (void)r.face_count(); }
            auto t1 = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / NB;
            std::cout << "[bool-time] " << tag << " session=" << us << " us  OCCT=" << occt_us
                      << " us  (" << us / occt_us << "x)\n";
        };
        // OCCT BRepAlgoAPI_Cut reference times, re-measured via the oracle's boolean_bench
        // (validation/occt_oracle, N=100): box-cyl ~3795us, box-sphere ~20652us. (The old 4000us
        // box-sphere figure was a stale placeholder -- OCCT's sphere boolean is ~5x costlier than
        // that, so our kernel is far closer to OCCT on the sphere than it appeared.)
        bench("box-cyl  ", box, cyl, 3795.0);
        bench("box-sphere", box, sph2, 20652.0);
    }

    MINI_TEST("BRep", "Boolean Sphere Split") {
        // The sphere (u-periodic seam + v-poles) is correctly arranged by split_by_brep into the
        // 7 expected regions (1 central band + 2 polar caps + 4 side caps) -- the gateway to
        // curved-periodic booleans. Full box-sphere watertight boolean still needs seam-aware
        // inner-loop closing for the seam-straddling cap (volume + sew); tracked separately.
        BRep sph = BRep::create_sphere(2.5);
        BRep box = BRep::create_box(4, 4, 4);
        BRep B2 = sph.split_by_brep(box);
        // The +x cap straddles the periodic u-seam. The analytic sphere pull-back (replicating
        // OCCT ProjLib_Sphere's per-point inverse U=atan2 -> EXACT seam crossings) now cuts it on
        // BOTH sides, so it splits into two half-caps -> 8 sphere regions (OCCT keeps the cap as a
        // single seam-spanning face = 7; joining the halves needs full seam identification, tracked).
        MINI_CHECK(B2.face_count() == 8);
        // Box-sphere boolean matches OCCT face counts (cut 7, common 7) and volumes to <0.3% (cut
        // 9.5457 / common 54.4543), vs ~40% before. Two fixes got here: (1) analytic_sphere_pullback
        // now maps longitude->u through the TRUE rational-NURBS parametrization (was a linear approx
        // that distorted the cut circle ~2% in flux); (2) volume() integrates sphere cap-cut faces by
        // the analytic boundary integral flux = C.A - R^2*closed_integral(h dtheta) instead of a
        // masked Gauss. Not yet watertight (the box disc's full circle must be co-split at the 2
        // seam-crossings to sew 1:1 to the 2 arcs); B2==8 (cap as 2 half-faces) vs OCCT's seam-span 7.
        BRep bcut = box.boolean_difference(sph), bcom = box.boolean_intersection(sph);
        MINI_CHECK(bcut.is_solid());   // NOW WATERTIGHT via shared-section-edge co-refinement
        MINI_CHECK(bcom.is_solid());
        MINI_CHECK(bcut.face_count() == 7);
        MINI_CHECK(bcom.face_count() == 7);
        MINI_CHECK(std::abs(bcut.volume() - 9.545724580842144) / 9.545724580842144 < 0.01);
        MINI_CHECK(std::abs(bcom.volume() - 54.45427562996632) / 54.45427562996632 < 0.002);
        MINI_CHECK(std::abs(bcut.volume() + bcom.volume() - 64.0) < 1e-4);  // partition box exactly
    }

    MINI_TEST("BRep", "Boolean Contained Sphere") {
        // Sphere (r=1.5) fully inside box(4) -- no surface intersection, so no seam-straddling
        // arrangement; exercises the robust volume() over a full periodic sphere + the
        // degenerate-pole-edge handling in is_solid(). OCCT: cut 64-(4/3)pi r^3 / 7, common
        // (4/3)pi r^3 / 1, fuse 64 / 6, all watertight.
        const double PI = 3.14159265358979323846;
        const double sv = (4.0/3.0) * PI * 1.5 * 1.5 * 1.5;
        BRep box = BRep::create_box(4, 4, 4);
        BRep sph = BRep::create_sphere(1.5);
        BRep cut = box.boolean_difference(sph);
        BRep com = box.boolean_intersection(sph);
        BRep fus = box.boolean_union(sph);
        MINI_CHECK(std::abs(cut.volume() - (64.0 - sv)) / (64.0 - sv) < 1e-6);
        MINI_CHECK(std::abs(com.volume() - sv) / sv < 1e-6);
        MINI_CHECK(std::abs(fus.volume() - 64.0) < 1e-6);
        MINI_CHECK(cut.face_count() == 7);
        MINI_CHECK(com.face_count() == 1);
        MINI_CHECK(fus.face_count() == 6);
        MINI_CHECK(cut.is_solid());
        MINI_CHECK(com.is_solid());
        MINI_CHECK(fus.is_solid());
    }

    MINI_TEST("BRep", "Boolean Example brep_booleans") {
        // Reproduces docs/examples/breps/brep_booleans.py: Box(2) + Cylinder(r=0.7, h=3, centred).
        // OCCT (oracle): fuse 9.539380400258997/10, cut 4.921239199482002/7, common 3.078760800517997/3.
        BRep box = BRep::create_box(2, 2, 2);
        BRep cyl = BRep::create_cylinder(0.7, 3.0); cyl.xform = Xform::translation(0, 0, -1.5); cyl = cyl.transformed();
        BRep fus = box.boolean_union(cyl);
        BRep cut = box.boolean_difference(cyl);
        BRep com = box.boolean_intersection(cyl);
        MINI_CHECK(fus.face_count() == 10);
        MINI_CHECK(cut.face_count() == 7);
        MINI_CHECK(com.face_count() == 3);
        MINI_CHECK(std::abs(fus.volume() - 9.539380400258997) / 9.539380400258997 < 1e-6);
        MINI_CHECK(std::abs(cut.volume() - 4.921239199482002) / 4.921239199482002 < 1e-6);
        MINI_CHECK(std::abs(com.volume() - 3.078760800517997) / 3.078760800517997 < 1e-6);
        MINI_CHECK(fus.is_solid());
        MINI_CHECK(cut.is_solid());
        MINI_CHECK(com.is_solid());
    }

    MINI_TEST("BRep", "Boolean Off-Center Cyl") {
        // Cylinder hole through the box, shifted off-centre by 0.5 in x. Topology unchanged from
        // the centred case. OCCT (oracle): cut V=51.43362938564082 7f, common V=12.566 3f,
        // fuse V=70.283 10f.
        const double PI = 3.14159265358979323846;
        BRep box = BRep::create_box(4, 4, 4);
        BRep cyl = BRep::create_cylinder(1.0, 6.0); cyl.xform = Xform::translation(0.5, 0, -3); cyl = cyl.transformed();
        BRep cut = box.boolean_difference(cyl);
        BRep com = box.boolean_intersection(cyl);
        BRep fus = box.boolean_union(cyl);
        MINI_CHECK(cut.face_count() == 7);
        MINI_CHECK(com.face_count() == 3);
        MINI_CHECK(fus.face_count() == 10);
        MINI_CHECK(std::abs(cut.volume() - (64 - 4*PI)) / (64 - 4*PI) < 1e-6);
        MINI_CHECK(std::abs(com.volume() - (4*PI)) / (4*PI) < 1e-6);
        MINI_CHECK(std::abs(fus.volume() - (64 + 2*PI)) / (64 + 2*PI) < 1e-6);
        MINI_CHECK(cut.is_solid());
        MINI_CHECK(com.is_solid());
        MINI_CHECK(fus.is_solid());
    }

    MINI_TEST("BRep", "Boolean Contained Box") {
        // B (vol 8) fully inside A (vol 64). cut = A with a cubic void (12 faces, watertight),
        // common = B (6 faces), fuse = A (6 faces). OCCT (oracle): 56/12, 8/6, 64/6.
        BRep ba = BRep::create_box(4, 4, 4);
        BRep bb = BRep::create_box(2, 2, 2);
        BRep cut = ba.boolean_difference(bb);
        BRep com = ba.boolean_intersection(bb);
        BRep fus = ba.boolean_union(bb);
        MINI_CHECK(std::abs(cut.volume() - 56.0) < 1e-6);
        MINI_CHECK(std::abs(com.volume() - 8.0) < 1e-6);
        MINI_CHECK(std::abs(fus.volume() - 64.0) < 1e-6);
        MINI_CHECK(cut.face_count() == 12);
        MINI_CHECK(com.face_count() == 6);
        MINI_CHECK(fus.face_count() == 6);
        MINI_CHECK(cut.is_solid());
        MINI_CHECK(com.is_solid());
        MINI_CHECK(fus.is_solid());
    }

    MINI_TEST("BRep", "Boolean Box-Box") {
        // Pure-planar booleans. A=[-2,2]^3 (vol 64); B=[-1,1]^3 translated (2,0,0) =
        // [1,3]x[-1,1]x[-1,1] (vol 8); overlap [1,2]x[-1,1]^2 = 4.  Exact analytic plane-plane
        // intersection + edge-imprint (T-junction split) + sew -> watertight solids matching
        // OCCT BRepAlgoAPI (oracle): cut V=60 11f, common V=4 6f, fuse V=68 11f.
        BRep ba = BRep::create_box(4, 4, 4);
        BRep bb = BRep::create_box(2, 2, 2); bb.xform = Xform::translation(2, 0, 0); bb = bb.transformed();
        BRep bcut = ba.boolean_difference(bb);
        BRep bcom = ba.boolean_intersection(bb);
        BRep bfus = ba.boolean_union(bb);
        MINI_CHECK(bcut.face_count() == 11);
        MINI_CHECK(bcom.face_count() == 6);
        MINI_CHECK(bfus.face_count() == 11);
        MINI_CHECK(std::abs(bcut.volume() - 60.0) < 1e-6);
        MINI_CHECK(std::abs(bcom.volume() - 4.0) < 1e-6);
        MINI_CHECK(std::abs(bfus.volume() - 68.0) < 1e-6);
        MINI_CHECK(bcut.is_solid());
        MINI_CHECK(bcom.is_solid());
        MINI_CHECK(bfus.is_solid());
    }

    MINI_TEST("BRep", "ZZ NxN Boolean Matrix") {
        // Diagnostic scorecard: every primitive pair x {cut,common,fuse} over
        // {box,sphere,cone,cylinder,torus}. Prints faces/volume/is_solid -> the boolean frontier.
        // Gated behind SESSION_MATRIX (45 boolean ops, slow) so normal runs stay fast.
        if (!std::getenv("SESSION_MATRIX")) { MINI_CHECK(true); return; }
        auto cell = [](const char* a, const char* b, BRep A, BRep B) {
            const char* ops[3] = {"cut", "com", "fus"};
            std::fprintf(stderr, "%-7s x %-7s |", a, b);
            for (int o = 0; o < 3; ++o) {
                try {
                    BRep r = (o==0) ? A.boolean_difference(B) : (o==1) ? A.boolean_intersection(B)
                                                                       : A.boolean_union(B);
                    int nf = r.face_count(); double v = r.volume(); int s = (int)r.is_solid();
                    std::fprintf(stderr, " %s f=%-2d v=%8.3f s=%d |", ops[o], nf, v, s);
                } catch (...) { std::fprintf(stderr, " %s  ---THREW---       |", ops[o]); }
            }
            std::fprintf(stderr, "\n");
        };
        auto box  = []{ return BRep::create_box(4,4,4); };
        auto sph  = []{ return BRep::create_sphere(2.5); };
        auto cyl  = []{ BRep c=BRep::create_cylinder(1.5,6); c.xform=Xform::translation(0,0,-3); return c.transformed(); };
        auto cone = []{ BRep c=BRep::create_cone(2.0,4.0);   c.xform=Xform::translation(0,0,-2); return c.transformed(); };
        auto tor  = []{ return BRep::create_torus(2.0,0.8); };
        auto box2 = []{ BRep c=BRep::create_box(2,2,2);      c.xform=Xform::translation(2,0,0); return c.transformed(); };
        auto sph2 = []{ BRep c=BRep::create_sphere(2.0);     c.xform=Xform::translation(2,0,0); return c.transformed(); };
        auto cyl2 = []{ BRep c=BRep::create_cylinder(1.5,6); c.xform=Xform::translation(-3,0,0)*Xform::rotation_y(90,true); return c.transformed(); };
        auto cone2= []{ BRep c=BRep::create_cone(2.0,4.0);   c.xform=Xform::translation(0,0,2)*Xform::rotation_x(180,true); return c.transformed(); };
        auto tor2 = []{ BRep c=BRep::create_torus(2.0,0.8);  c.xform=Xform::translation(2,0,0); return c.transformed(); };
        std::fprintf(stderr, "\n=== NxN BOOLEAN MATRIX (session, overlapping configs) ===\n");
        cell("box","box",    box(), box2());
        cell("box","sphere", box(), sph());
        cell("box","cone",   box(), cone());
        cell("box","cyl",    box(), cyl());
        cell("box","torus",  box(), tor());
        cell("sphere","sphere", sph(), sph2());
        cell("sphere","cone",   sph(), cone());
        cell("sphere","cyl",    sph(), cyl());
        cell("sphere","torus",  sph(), tor());
        cell("cone","cone",   cone(), cone2());
        cell("cone","cyl",    cone(), cyl());
        cell("cone","torus",  cone(), tor());
        cell("cyl","cyl",     cyl(),  cyl2());
        cell("cyl","torus",   cyl(),  tor());
        cell("torus","torus", tor(),  tor2());
        std::fprintf(stderr, "=== end matrix ===\n");
        MINI_CHECK(true);
    }

    MINI_TEST("BRep", "Contains Point") {
        // Ray-cast parity classification; IN/OUT validated vs OCCT
        // BRepClass3d_SolidClassifier (validation/compare_classify.py).
        BRep box = BRep::create_box(2, 3, 4);   // [-1,1]x[-1.5,1.5]x[-2,2]
        MINI_CHECK(box.contains_point(Point(0, 0, 0)));
        MINI_CHECK(box.contains_point(Point(0.9, 1.4, 1.9)));
        MINI_CHECK(!box.contains_point(Point(1.1, 0, 0)));
        MINI_CHECK(!box.contains_point(Point(5, 0, 0)));

        BRep cyl = BRep::create_cylinder(1.0, 4.0);  // r=1, z in [0,4]
        MINI_CHECK(cyl.contains_point(Point(0, 0, 2)));
        MINI_CHECK(cyl.contains_point(Point(0.9, 0, 2)));
        MINI_CHECK(!cyl.contains_point(Point(0, 0, -1)));
        MINI_CHECK(!cyl.contains_point(Point(1.1, 0, 2)));

        BRep sph = BRep::create_sphere(2.0);
        MINI_CHECK(sph.contains_point(Point(0, 0, 0)));
        MINI_CHECK(sph.contains_point(Point(1.5, 0, 0)));
        MINI_CHECK(!sph.contains_point(Point(3, 0, 0)));
    }

    MINI_TEST("BRep", "Volume") {
        // Compare BRep::volume() to the analytic value (= OCCT BRepGProp::VolumeProperties).
        const double PI = 3.14159265358979323846;
        BRep box = BRep::create_box(2, 3, 4);          // 2x3x4 -> 24
        BRep cyl = BRep::create_cylinder(1.0, 4.0);    // pi r^2 h = 4 pi
        BRep sph = BRep::create_sphere(2.0);           // 4/3 pi r^3
        double vbox = box.volume(), vcyl = cyl.volume(), vsph = sph.volume();
        // Exact divergence-theorem volume: matches OCCT BRepGProp to machine precision.
        MINI_CHECK(std::abs(vbox - 24.0) < 1e-9);
        MINI_CHECK(std::abs(vcyl - 4*PI) < 4*PI * 1e-9);
        MINI_CHECK(std::abs(vsph - (4.0/3.0)*PI*8) < (4.0/3.0)*PI*8 * 1e-9);
        // Face counts match OCCT (box=6, cylinder=3, sphere=1).
        MINI_CHECK(box.face_count() == 6);
        MINI_CHECK(cyl.face_count() == 3);
        MINI_CHECK(sph.face_count() == 1);
    }

} // namespace session_cpp
