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

    // Every non-degenerated edge of a solid is used by exactly two faces with opposite
    // composed orientations (the manifold contract BRepCheck enforces).
    static bool edges_manifold(const BRep& b) {
        for (int ei = 0; ei < b.edge_count(); ++ei) {
            if (b.m_edges[ei].degenerated) continue;
            std::vector<BRepRef> uses = b.edge_faces(ei);
            if (uses.size() != 2) return false;
            if (uses[0].orientation == uses[1].orientation) return false;
        }
        return true;
    }

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

        BRep box = BRep::create_box(2.0, 3.0, 4.0);

        int vc = box.vertex_count();
        int ec = box.edge_count();
        int wc = box.wire_count();
        int fc = box.face_count();
        int sc = box.shell_count();
        int oc = box.solid_count();
        std::vector<Point> pts = box.vertex_points();

        MINI_CHECK(vc == 8);
        MINI_CHECK(ec == 12);
        MINI_CHECK(wc == 6);
        MINI_CHECK(fc == 6);
        MINI_CHECK(sc == 1);
        MINI_CHECK(oc == 1);
        MINI_CHECK(pts.size() == 8);
        MINI_CHECK(std::abs(pts[0][0] + 1.0) < 1e-9);
        MINI_CHECK(box.m_surfaces.size() == 6);
        MINI_CHECK(box.m_curves_3d.size() == 12);
        MINI_CHECK(box.m_curves_2d.size() == 24);
    }

    MINI_TEST("BRep", "Add Face") {
        // uncomment #include "brep.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "mesh.h"

        BRep b;
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0)); srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0)); srf.set_cv(1, 1, Point(1, 1, 0));
        int si = b.add_surface(srf);

        Point corners[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        std::vector<BRepRef> refs;
        for (int i = 0; i < 4; ++i) b.add_vertex(corners[i]);
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            int ci = b.add_curve_3d(NurbsCurve::create(false, 1, {corners[i], corners[j]}));
            int ei = b.add_edge(ci, i, j);
            int c2 = b.add_curve_2d(NurbsCurve::create(false, 1, {corners[i], corners[j]}));
            b.add_pcurve(ei, si, c2);
            refs.push_back({ei, BRepOrientation::Forward});
        }
        int wi = b.add_wire(refs);
        int fi = b.add_face(si, {{wi, BRepOrientation::Forward}});
        Mesh m = b.mesh();

        MINI_CHECK(b.is_valid());
        MINI_CHECK(fi == 0);
        MINI_CHECK(b.face_count() == 1);
        MINI_CHECK(b.wire_count() == 1);
        MINI_CHECK(b.edge_count() == 4);
        MINI_CHECK(b.vertex_count() == 4);
        MINI_CHECK(b.m_edges[0].pcurves.size() == 1);
        MINI_CHECK(b.pcurve_index(0, 0, BRepOrientation::Forward) == 0);
        MINI_CHECK(!b.is_solid());
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Mesh") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        Mesh m = box.mesh();
        std::vector<Mesh> fm = box.face_meshes();

        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
        MINI_CHECK(fm.size() == 6);
        MINI_CHECK(!fm[0].is_empty());
    }

    MINI_TEST("BRep", "Point At") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include "tolerance.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        Point pt = box.point_at(0, 0.5, 0.5);
        Vector n = box.normal_at(0, 0.5, 0.5);
        Vector n_top = box.normal_at(1, 0.5, 0.5);

        MINI_CHECK(std::abs(pt[2] + 2.0) < 1e-9);
        MINI_CHECK(std::abs(pt[0]) < 1e-9);
        MINI_CHECK(std::abs(pt[1]) < 1e-9);
        MINI_CHECK(n[2] < -0.99);
        MINI_CHECK(n_top[2] > 0.99);
    }

    MINI_TEST("BRep", "Is Solid") {
        // uncomment #include "brep.h"
        // uncomment #include "polyline.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRep cyl = BRep::create_cylinder(1.0, 2.0);
        BRep sph = BRep::create_sphere(1.0);
        BRep cone = BRep::create_cone(1.0, 2.0);
        BRep pyr = BRep::create_pyramid(2.0, 1.0);
        BRep tor = BRep::create_torus(2.0, 0.5);
        BRep blk = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);

        Polyline quad(std::vector<Point>{
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
            Point(0, 0, 0),
        });
        BRep sheet = BRep::from_polylines({quad});

        MINI_CHECK(box.is_solid() && edges_manifold(box));
        MINI_CHECK(cyl.is_solid() && edges_manifold(cyl));
        MINI_CHECK(sph.is_solid() && edges_manifold(sph));
        MINI_CHECK(cone.is_solid() && edges_manifold(cone));
        MINI_CHECK(pyr.is_solid() && edges_manifold(pyr));
        MINI_CHECK(tor.is_solid() && edges_manifold(tor));
        MINI_CHECK(blk.is_solid() && edges_manifold(blk));
        MINI_CHECK(!sheet.is_solid());
        MINI_CHECK(sheet.solid_count() == 0);
    }

    MINI_TEST("BRep", "Is Closed") {
        // uncomment #include "brep.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRep open = box;
        open.m_shells[0].faces.pop_back();

        MINI_CHECK(box.is_closed(0));
        MINI_CHECK(!box.is_closed(1));
        MINI_CHECK(!open.is_closed(0));
        MINI_CHECK(!open.is_solid());
    }

    MINI_TEST("BRep", "Wire Edges") {
        // uncomment #include "brep.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRepRef fwd{0, BRepOrientation::Forward};
        BRepRef rev{0, BRepOrientation::Reversed};
        std::vector<BRepRef> a = box.wire_edges(fwd);
        std::vector<BRepRef> c = box.wire_edges(rev);

        MINI_CHECK(a.size() == 4);
        MINI_CHECK(c.size() == 4);
        MINI_CHECK(a[0].index == c[3].index);
        MINI_CHECK(a[0].orientation == brep_reverse(c[3].orientation));
        MINI_CHECK(brep_compose(BRepOrientation::Reversed, BRepOrientation::Reversed) == BRepOrientation::Forward);
        MINI_CHECK(brep_compose(BRepOrientation::Forward, BRepOrientation::Reversed) == BRepOrientation::Reversed);
        MINI_CHECK(brep_compose(BRepOrientation::Internal, BRepOrientation::Reversed) == BRepOrientation::Internal);
    }

    MINI_TEST("BRep", "Edge Faces") {
        // uncomment #include "brep.h"

        BRep cyl = BRep::create_cylinder(1.0, 2.0);
        std::vector<BRepRef> bot = cyl.edge_faces(0);
        std::vector<BRepRef> seam = cyl.edge_faces(2);
        int pc_f = cyl.pcurve_index(2, 0, BRepOrientation::Forward);
        int pc_r = cyl.pcurve_index(2, 0, BRepOrientation::Reversed);

        MINI_CHECK(bot.size() == 2);
        MINI_CHECK(bot[0].index == 0 && bot[1].index == 1);
        MINI_CHECK(bot[0].orientation != bot[1].orientation);
        MINI_CHECK(seam.size() == 2);
        MINI_CHECK(seam[0].index == 0 && seam[1].index == 0);
        MINI_CHECK(pc_f >= 0 && pc_r >= 0 && pc_f != pc_r);
        MINI_CHECK(cyl.pcurve_index(2, 1, BRepOrientation::Forward) == -1);
        MINI_CHECK(cyl.face_orientation(0) == BRepOrientation::Forward);
    }

    MINI_TEST("BRep", "Update Tolerances") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        double worst = box.update_tolerances();
        BRep bent = box;
        bent.m_vertices[0].point = Point(-1.0, -1.5, -2.01);
        double worst_bent = bent.update_tolerances();
        double worst_prims = 0.0;
        for (BRep p : {BRep::create_cylinder(1.0, 2.0), BRep::create_sphere(1.0), BRep::create_cone(1.0, 2.0),
                       BRep::create_pyramid(2.0, 1.0), BRep::create_torus(2.0, 0.5), BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0)})
            worst_prims = std::max(worst_prims, p.update_tolerances());

        MINI_CHECK(worst < 1e-9);
        MINI_CHECK(box.m_edges[0].tolerance < 1e-9);
        MINI_CHECK(std::abs(worst_bent - 0.01) < 1e-9);
        MINI_CHECK(std::abs(bent.m_vertices[0].tolerance - 0.01) < 1e-9);
        MINI_CHECK(bent.m_vertices[6].tolerance < 1e-9);
        MINI_CHECK(worst_prims < 1e-6);
    }

    MINI_TEST("BRep", "Transformation") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"
        // uncomment #include "tolerance.h"

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        Xform box_xf = Xform::translation(10.0, 20.0, 30.0);
        BRep moved = box.transformed(box_xf);

        Point pt = moved.point_at(0, 0.0, 0.0);
        Point pt_orig = box.point_at(0, 0.0, 0.0);

        MINI_CHECK(std::abs(pt[0] - pt_orig[0] - 10.0) < 0.01);
        MINI_CHECK(std::abs(pt[1] - pt_orig[1] - 20.0) < 0.01);
        MINI_CHECK(std::abs(pt[2] - pt_orig[2] - 30.0) < 0.01);
        MINI_CHECK(std::abs(moved.m_vertices[0].point[0] - box.m_vertices[0].point[0] - 10.0) < 0.01);
    }

    MINI_TEST("BRep", "Transform Roundtrip") {
        // uncomment #include "brep.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include "xform.h"
        // uncomment #include "tolerance.h"

        Vector axis(0.3, 0.5, 0.81);
        Xform rot = Xform::rotation(axis, 37.0, true);
        Xform tr = Xform::translation(10.0, -5.0, 3.0);
        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRep moved = box.transformed(rot).transformed(tr);

        bool match = true;
        for (size_t i = 0; i < box.m_vertices.size(); ++i) {
            Point expect = tr.transform_point(rot.transform_point(box.m_vertices[i].point));
            if (moved.m_vertices[i].point.distance(expect) > 1e-9) match = false;
        }

        BRep back = moved.transformed(tr.inverse().value()).transformed(rot.inverse().value());

        bool restored = true;
        for (size_t i = 0; i < box.m_vertices.size(); ++i)
            if (back.m_vertices[i].point.distance(box.m_vertices[i].point) > 1e-9) restored = false;

        MINI_CHECK(match);
        MINI_CHECK(restored);
        MINI_CHECK(back.is_solid());
        MINI_CHECK(back.update_tolerances() < 1e-9);
    }

    MINI_TEST("BRep", "Json Roundtrip") {
        // uncomment #include "brep.h"
        // uncomment #include "color.h"
        // uncomment #include <filesystem>

        BRep box = BRep::create_cylinder(1.0, 2.0);
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
        MINI_CHECK(loaded_from_file.is_solid());
        MINI_CHECK(loaded_from_file.m_edges[2].pcurves[0].curve_2d_index_2 >= 0);
        MINI_CHECK(loaded_from_file.m_wires[0].edges[2].orientation == BRepOrientation::Reversed);
    }

    MINI_TEST("BRep", "Create Cylinder") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep cyl = BRep::create_cylinder(1.0, 2.0);
        Mesh m = cyl.mesh();

        MINI_CHECK(cyl.is_valid());
        MINI_CHECK(cyl.face_count() == 3);
        MINI_CHECK(cyl.edge_count() == 3);
        MINI_CHECK(cyl.vertex_count() == 2);
        MINI_CHECK(cyl.is_solid());
        MINI_CHECK(cyl.name == "cylinder");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Sphere") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep sph = BRep::create_sphere(1.0);
        Mesh m = sph.mesh();

        MINI_CHECK(sph.is_valid());
        MINI_CHECK(sph.face_count() == 1);
        MINI_CHECK(sph.edge_count() == 3);
        MINI_CHECK(sph.vertex_count() == 2);
        MINI_CHECK(sph.m_edges[1].degenerated && sph.m_edges[2].degenerated);
        MINI_CHECK(sph.is_solid());
        MINI_CHECK(sph.name == "sphere");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Cone") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep cone = BRep::create_cone(1.0, 2.0);
        Mesh m = cone.mesh();

        MINI_CHECK(cone.is_valid());
        MINI_CHECK(cone.face_count() == 2);
        MINI_CHECK(cone.edge_count() == 3);
        MINI_CHECK(cone.vertex_count() == 2);
        MINI_CHECK(cone.is_solid());
        MINI_CHECK(cone.name == "cone");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Pyramid") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep pyr = BRep::create_pyramid(2.0, 1.0);
        Mesh m = pyr.mesh();

        MINI_CHECK(pyr.is_valid());
        MINI_CHECK(pyr.face_count() == 5);
        MINI_CHECK(pyr.edge_count() == 12);
        MINI_CHECK(pyr.vertex_count() == 5);
        MINI_CHECK(pyr.is_solid());
        MINI_CHECK(pyr.name == "pyramid");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Torus") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep tor = BRep::create_torus(2.0, 0.5);
        Mesh m = tor.mesh();

        MINI_CHECK(tor.is_valid());
        MINI_CHECK(tor.face_count() == 1);
        MINI_CHECK(tor.edge_count() == 2);
        MINI_CHECK(tor.vertex_count() == 1);
        MINI_CHECK(tor.is_solid());
        MINI_CHECK(tor.name == "torus");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Block With Hole") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        Mesh m = bh.mesh();

        MINI_CHECK(bh.is_valid());
        MINI_CHECK(bh.face_count() == 7);
        MINI_CHECK(bh.edge_count() == 15);
        MINI_CHECK(bh.vertex_count() == 10);
        MINI_CHECK(bh.m_faces[6].wires.size() == 2);
        MINI_CHECK(bh.face_orientation(4) == BRepOrientation::Reversed);
        MINI_CHECK(bh.is_solid());
        MINI_CHECK(bh.name == "block_with_hole");
        MINI_CHECK(!m.is_empty());
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
        MINI_CHECK(b.shell_count() == 1);
        MINI_CHECK(b.is_solid() && edges_manifold(b));
        MINI_CHECK(std::abs(b.volume() - 24.0) < 1e-6);
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
        MINI_CHECK(b.edge_count() == 6);
        MINI_CHECK(b.vertex_count() == 5);
        MINI_CHECK(!b.is_solid());
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
        MINI_CHECK(b.wire_count() == 2);
        MINI_CHECK(b.m_faces[0].wires.size() == 2);
        MINI_CHECK(b.m_faces[0].wires[1].index == 1);
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(std::abs(m.area() - (100.0 - Tolerance::PI * 4.0)) < 0.5);
    }

    MINI_TEST("BRep", "Mesh Orientation") {
        // uncomment #include "brep.h"
        // uncomment #include "mesh.h"

        // Reversed faces must flip winding; an unflipped bore inflates the volume.
        BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        double vol = bh.mesh().volume();
        double ref = 8.0 * 6.0 * 4.0 - Tolerance::PI * 1.5 * 1.5 * 4.0;

        MINI_CHECK(std::abs(vol - ref) / ref < 0.02);
    }

    MINI_TEST("BRep", "Protobuf Roundtrip") {
        // uncomment #include "brep.h"
        // uncomment #include "color.h"
        // uncomment #include <filesystem>

        BRep box = BRep::create_cylinder(1.0, 2.0);
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
        MINI_CHECK(loaded.is_solid());
        MINI_CHECK(loaded.m_edges[2].pcurves[0].curve_2d_index_2 >= 0);
        MINI_CHECK(loaded.m_wires[0].edges[2].orientation == BRepOrientation::Reversed);
    }

    MINI_TEST("BRep", "Volume") {
        // uncomment #include "brep.h"
        // uncomment #include "tolerance.h"

        BRep box = BRep::create_box(2, 3, 4);          // 2x3x4 -> 24
        BRep cyl = BRep::create_cylinder(1.0, 4.0);    // pi r^2 h = 4 pi
        BRep sph = BRep::create_sphere(2.0);           // 4/3 pi r^3
        double vbox = box.volume(), vcyl = cyl.volume(), vsph = sph.volume();

        // Tessellated volume: the default grid density is 2-4% under the analytic value.
        MINI_CHECK(std::abs(vbox - 24.0) < 1e-9);
        MINI_CHECK(std::abs(vcyl - 4 * Tolerance::PI) / (4 * Tolerance::PI) < 0.05);
        MINI_CHECK(std::abs(vsph - (4.0 / 3.0) * Tolerance::PI * 8) / ((4.0 / 3.0) * Tolerance::PI * 8) < 0.05);
    }

} // namespace session_cpp
