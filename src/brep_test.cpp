#include "mini_test.h"
#include "brep.h"
#include "brep.pb.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "mesh.h"
#include "color.h"
#include "primitives.h"
#include "remesh_nurbssurface_grid.h"
#include "tolerance.h"

#include <cmath>
#include <array>
#include <limits>
#include <algorithm>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    /// Every non-degenerated edge of a solid is used by exactly two faces with opposite composed orientations
    static bool edges_manifold(const BRep& b) {

        for (int ei = 0; ei < b.edge_count(); ++ei) {
            if (b.m_edges[ei].degenerated)
                continue;

            const std::vector<BRepRef> uses = b.edge_faces(ei);

            if (uses.size() != 2)
                return false;

            if (uses[0].orientation == uses[1].orientation)
                return false;
        }

        return true;
    }

    /// Sorted positions of the mesh vertices on the v = 0 side
    static std::vector<std::array<double, 3>> boundary_points(const Mesh& mesh) {

        std::vector<std::array<double, 3>> points;

        for (const std::pair<const size_t, VertexData>& entry : mesh.vertex)
            if (entry.second.attributes.at("v") == 0.0)
                points.push_back({entry.second.position()[0], entry.second.position()[1], entry.second.position()[2]});

        std::sort(points.begin(), points.end());

        return points;
    }

    /// Unit planar quad face with straight edges and pcurves; returns the face index
    static int build_quad_face(BRep& b) {

        NurbsSurface srf(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0));
        srf.set_cv(1, 1, Point(1, 1, 0));
        const int si = b.add_surface(srf);
        const Point corners[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };

        for (int i = 0; i < 4; ++i)
            b.add_vertex(corners[i]);

        std::vector<BRepRef> refs;

        for (int i = 0; i < 4; ++i) {
            const int j = (i + 1) % 4;
            const int ci = b.add_curve_3d(NurbsCurve::create(false, 1, {corners[i], corners[j]}));
            const int ei = b.add_edge(ci, i, j);
            const int c2 = b.add_curve_2d(NurbsCurve::create(false, 1, {corners[i], corners[j]}));
            b.add_pcurve(ei, si, c2);
            refs.push_back({ei, BRepOrientation::Forward});
        }

        const int wi = b.add_wire(refs);

        return b.add_face(si, {{wi, BRepOrientation::Forward}});
    }

    MINI_TEST("BRep", "Shared Grid Boundary") {

        BRep b;
        std::vector<NurbsSurface> surfaces;

        for (int face = 0; face < 2; ++face) {
            std::vector<Point> points;

            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 2; ++j) {
                    const double z = i != 1 ? 0.0 : (j == 0 || face == 0 ? 0.5 : 4.0);
                    points.push_back(Point(i * 0.5, j * (face == 0 ? 1.0 : -1.0), z));
                }

            const NurbsSurface surface = NurbsSurface::create(false, false, 2, 1, 3, 2, points);
            const int si = b.add_surface(surface);
            const std::array<double, 2> corners[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
            std::vector<int> vertices;

            for (int side = 0; side < 4; ++side)
                vertices.push_back(b.add_vertex(surface.point_at(corners[side][0], corners[side][1])));

            std::vector<BRepRef> edges;

            for (int side = 0; side < 4; ++side) {
                const std::array<double, 2> a = corners[side];
                const std::array<double, 2> z = corners[(side + 1) % 4];
                int edge = 0;

                if (side != 0 || face != 1) {
                    const int dir = a[0] != z[0] ? 0 : 1;
                    NurbsCurve curve = surface.iso_curve(dir, a[1 - dir]);

                    if (a[dir] > z[dir])
                        curve.reverse();

                    edge = b.add_edge(b.add_curve_3d(curve), vertices[side], vertices[(side + 1) % 4]);
                }

                const NurbsCurve pc = NurbsCurve::create(false, 1, {Point(a[0], a[1], 0), Point(z[0], z[1], 0)});
                b.add_pcurve(edge, si, b.add_curve_2d(pc));
                edges.push_back({edge, BRepOrientation::Forward});
            }

            b.add_face(si, {{b.add_wire(edges), BRepOrientation::Forward}}, 1e-8);
            surfaces.push_back(surface);
        }

        std::vector<Mesh> original;

        for (const NurbsSurface& s : surfaces)
            original.push_back(RemeshNurbsSurfaceGrid::from_u_v_q(s, 0, 0, 20.0, 0.005));

        MINI_CHECK(boundary_points(original[0]).size() == 7 && boundary_points(original[1]).size() == 11);

        std::vector<Mesh> meshes = b.face_meshes_q(true, 20.0, 0.005);
        std::vector<std::array<double, 3>> first = boundary_points(meshes[0]);
        const std::vector<std::array<double, 3>> second = boundary_points(meshes[1]);

        MINI_CHECK(first == second && first.size() == 7);
        MINI_CHECK(meshes[0].face.size() == original[0].face.size() && !meshes[1].face.empty());

        double maximum = 0.0;

        for (size_t i = 0; i + 1 < first.size(); ++i) {
            const std::array<double, 3> a = first[i];
            const std::array<double, 3> z = first[i + 1];
            const Point actual = surfaces[0].point_at((a[0] + z[0]) * 0.5, 0.0);
            double sag = 0.0;

            for (int d = 0; d < 3; ++d)
                sag += std::pow(actual[d] - (a[d] + z[d]) * 0.5, 2);

            maximum = std::max(maximum, std::sqrt(sag));
        }

        MINI_CHECK(maximum <= 0.005 * 1.5);

        const Mesh refined = RemeshNurbsSurfaceGrid::from_u_v_q(surfaces[0], 0, 0, 5.0, 0.001);
        meshes = b.face_meshes_q(true, 5.0, 0.001);
        first = boundary_points(meshes[0]);

        MINI_CHECK(first == boundary_points(meshes[1]) && !meshes[0].face.empty() && !meshes[1].face.empty());

        for (const std::array<double, 3>& point : boundary_points(refined))
            MINI_CHECK(std::find(first.begin(), first.end(), point) != first.end());

        const double cosine = std::cos(5.0 * Tolerance::PI / 180.0);

        for (size_t i = 0; i + 1 < first.size(); ++i) {
            const Vector a = surfaces[0].normal_at(first[i][0], 0.0);
            const Vector z = surfaces[0].normal_at(first[i + 1][0], 0.0);
            MINI_CHECK(a.dot(z) >= cosine - 64.0 * std::numeric_limits<double>::epsilon());
        }
    }

    MINI_TEST("BRep", "Constructor") {

        BRep b;

        const std::string sstr = b.str();
        const std::string srepr = b.repr();

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

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);

        MINI_CHECK(box.is_valid());
        MINI_CHECK(box.face_count() == 6);
        MINI_CHECK(box.edge_count() == 12);
        MINI_CHECK(box.vertex_count() == 8);
        MINI_CHECK(box.is_solid());
        MINI_CHECK(box.name == "box");
    }

    MINI_TEST("BRep", "Accessors") {

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);

        const int vc = box.vertex_count();
        const int ec = box.edge_count();
        const int wc = box.wire_count();
        const int fc = box.face_count();
        const int sc = box.shell_count();
        const int oc = box.solid_count();
        const std::vector<Point> pts = box.vertex_points();

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

        BRep b;
        NurbsSurface srf(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0));
        srf.set_cv(1, 1, Point(1, 1, 0));
        const int si = b.add_surface(srf);

        const Point corners[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        std::vector<BRepRef> refs;

        for (int i = 0; i < 4; ++i)
            b.add_vertex(corners[i]);

        for (int i = 0; i < 4; ++i) {
            const int j = (i + 1) % 4;
            const int ci = b.add_curve_3d(NurbsCurve::create(false, 1, {corners[i], corners[j]}));
            const int ei = b.add_edge(ci, i, j);
            const int c2 = b.add_curve_2d(NurbsCurve::create(false, 1, {corners[i], corners[j]}));
            b.add_pcurve(ei, si, c2);
            refs.push_back({ei, BRepOrientation::Forward});
        }

        const int wi = b.add_wire(refs);
        const int fi = b.add_face(si, {{wi, BRepOrientation::Forward}});
        const Mesh m = b.mesh();

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

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const Mesh m = box.mesh();
        const std::vector<Mesh> fm = box.face_meshes();

        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
        MINI_CHECK(fm.size() == 6);
        MINI_CHECK(!fm[0].is_empty());
    }

    MINI_TEST("BRep", "Point At") {

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const Point pt = box.point_at(0, 0.5, 0.5);
        const Vector n = box.normal_at(0, 0.5, 0.5);
        const Vector n_top = box.normal_at(1, 0.5, 0.5);

        MINI_CHECK(std::abs(pt[2] + 2.0) < 1e-9);
        MINI_CHECK(std::abs(pt[0]) < 1e-9);
        MINI_CHECK(std::abs(pt[1]) < 1e-9);
        MINI_CHECK(n[2] < -0.99);
        MINI_CHECK(n_top[2] > 0.99);
    }

    MINI_TEST("BRep", "Is Solid") {

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const BRep cyl = BRep::create_cylinder(1.0, 2.0);
        const BRep sph = BRep::create_sphere(1.0);
        const BRep cone = BRep::create_cone(1.0, 2.0);
        const BRep pyr = BRep::create_pyramid(2.0, 1.0);
        const BRep tor = BRep::create_torus(2.0, 0.5);
        const BRep blk = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);

        const Polyline quad(std::vector<Point>{
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
            Point(0, 0, 0),
        });
        const BRep sheet = BRep::from_polylines({quad});

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

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRep open = box;
        open.m_shells[0].faces.pop_back();

        MINI_CHECK(box.is_closed(0));
        MINI_CHECK(!box.is_closed(1));
        MINI_CHECK(!open.is_closed(0));
        MINI_CHECK(!open.is_solid());
    }

    MINI_TEST("BRep", "Wire Edges") {

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const BRepRef fwd{0, BRepOrientation::Forward};
        const BRepRef rev{0, BRepOrientation::Reversed};
        const std::vector<BRepRef> a = box.wire_edges(fwd);
        const std::vector<BRepRef> c = box.wire_edges(rev);

        MINI_CHECK(a.size() == 4);
        MINI_CHECK(c.size() == 4);
        MINI_CHECK(a[0].index == c[3].index);
        MINI_CHECK(a[0].orientation == brep_reverse(c[3].orientation));
        MINI_CHECK(brep_compose(BRepOrientation::Reversed, BRepOrientation::Reversed) == BRepOrientation::Forward);
        MINI_CHECK(brep_compose(BRepOrientation::Forward, BRepOrientation::Reversed) == BRepOrientation::Reversed);
        MINI_CHECK(brep_compose(BRepOrientation::Internal, BRepOrientation::Reversed) == BRepOrientation::Internal);
    }

    MINI_TEST("BRep", "Edge Faces") {

        const BRep cyl = BRep::create_cylinder(1.0, 2.0);
        const std::vector<BRepRef> bot = cyl.edge_faces(0);
        const std::vector<BRepRef> seam = cyl.edge_faces(2);
        const int pc_f = cyl.pcurve_index(2, 0, BRepOrientation::Forward);
        const int pc_r = cyl.pcurve_index(2, 0, BRepOrientation::Reversed);

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

        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const double worst = box.update_tolerances();
        BRep bent = box;
        bent.m_vertices[0].point = Point(-1.0, -1.5, -2.01);
        const double worst_bent = bent.update_tolerances();
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

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const Xform box_xf = Xform::translation(10.0, 20.0, 30.0);
        const BRep moved = box.transformed(box_xf);

        const Point pt = moved.point_at(0, 0.0, 0.0);
        const Point pt_orig = box.point_at(0, 0.0, 0.0);

        MINI_CHECK(std::abs(pt[0] - pt_orig[0] - 10.0) < 0.01);
        MINI_CHECK(std::abs(pt[1] - pt_orig[1] - 20.0) < 0.01);
        MINI_CHECK(std::abs(pt[2] - pt_orig[2] - 30.0) < 0.01);
        MINI_CHECK(std::abs(moved.m_vertices[0].point[0] - box.m_vertices[0].point[0] - 10.0) < 0.01);
    }

    MINI_TEST("BRep", "Transform Roundtrip") {

        const Vector axis(0.3, 0.5, 0.81);
        const Xform rot = Xform::rotation(axis, 37.0, true);
        const Xform tr = Xform::translation(10.0, -5.0, 3.0);
        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const BRep moved = box.transformed(rot).transformed(tr);

        bool match = true;

        for (size_t i = 0; i < box.m_vertices.size(); ++i) {
            const Point expect = tr.transform_point(rot.transform_point(box.m_vertices[i].point));

            if (moved.m_vertices[i].point.distance(expect) > 1e-9)
                match = false;
        }

        BRep back = moved.transformed(tr.inverse().value()).transformed(rot.inverse().value());

        bool restored = true;

        for (size_t i = 0; i < box.m_vertices.size(); ++i)
            if (back.m_vertices[i].point.distance(box.m_vertices[i].point) > 1e-9)
                restored = false;

        MINI_CHECK(match);
        MINI_CHECK(restored);
        MINI_CHECK(back.is_solid());
        MINI_CHECK(back.update_tolerances() < 1e-9);
    }

    MINI_TEST("BRep", "Cut By Plane") {

        const BRep box = BRep::create_box(2.0, 2.0, 2.0);
        const BRep half = box.cut_by_plane(Plane::from_point_normal(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0)));

        const Xform far = Xform::translation(100000.0, 200000.0, 30000.0) * Xform::rotation(Vector(1.0, 2.0, 3.0), 40.0, true);
        const BRep beam = BRep::create_box(200.0, 100.0, 600.0).transformed(far);
        const BRep piece = beam.cut_by_plane(Plane::from_point_normal(far.transform_point(Point(0.0, 0.0, 0.0)), far.transform_vector(Vector(0.0, 0.0, 1.0))));

        MINI_CHECK(half.is_solid());
        MINI_CHECK(half.vertex_count() == 8);
        MINI_CHECK(half.edge_count() == 12);
        MINI_CHECK(half.face_count() == 6);
        MINI_CHECK(std::abs(half.volume() - 4.0) < 1e-9);
        MINI_CHECK(piece.is_solid());
        MINI_CHECK(piece.face_count() == 6);
        MINI_CHECK(std::abs(piece.volume() / 6000000.0 - 1.0) < 1e-6);
    }

    MINI_TEST("BRep", "Json Roundtrip") {

        BRep box = BRep::create_cylinder(1.0, 2.0);
        box.name = "test_brep";
        box.width = 2.0;
        box.surfacecolor = Color(255, 128, 64, 255);

        const nlohmann::ordered_json json = box.jsondump();
        const BRep loaded_json = BRep::jsonload(json);

        const std::string json_string = box.file_json_dumps();
        const BRep loaded_json_string = BRep::file_json_loads(json_string);

        const std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_brep.json").string();
        box.file_json_dump(filename);
        const BRep loaded_from_file = BRep::file_json_load(filename);

        MINI_CHECK(loaded_json == box);
        MINI_CHECK(loaded_json_string == box);
        MINI_CHECK(loaded_from_file == box);
        MINI_CHECK(loaded_from_file.is_solid());
        MINI_CHECK(loaded_from_file.m_edges[2].pcurves[0].curve_2d_index_2 >= 0);
        MINI_CHECK(loaded_from_file.m_wires[0].edges[2].orientation == BRepOrientation::Reversed);
    }

    MINI_TEST("BRep", "Create Cylinder") {

        const BRep cyl = BRep::create_cylinder(1.0, 2.0);
        const Mesh m = cyl.mesh();

        MINI_CHECK(cyl.is_valid());
        MINI_CHECK(cyl.face_count() == 3);
        MINI_CHECK(cyl.edge_count() == 3);
        MINI_CHECK(cyl.vertex_count() == 2);
        MINI_CHECK(cyl.is_solid());
        MINI_CHECK(cyl.name == "cylinder");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Sphere") {

        const BRep sph = BRep::create_sphere(1.0);
        const Mesh m = sph.mesh();

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

        const BRep cone = BRep::create_cone(1.0, 2.0);
        const Mesh m = cone.mesh();

        MINI_CHECK(cone.is_valid());
        MINI_CHECK(cone.face_count() == 2);
        MINI_CHECK(cone.edge_count() == 3);
        MINI_CHECK(cone.vertex_count() == 2);
        MINI_CHECK(cone.is_solid());
        MINI_CHECK(cone.name == "cone");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Pyramid") {

        const BRep pyr = BRep::create_pyramid(2.0, 1.0);
        const Mesh m = pyr.mesh();

        MINI_CHECK(pyr.is_valid());
        MINI_CHECK(pyr.face_count() == 5);
        MINI_CHECK(pyr.edge_count() == 12);
        MINI_CHECK(pyr.vertex_count() == 5);
        MINI_CHECK(pyr.is_solid());
        MINI_CHECK(pyr.name == "pyramid");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Torus") {

        const BRep tor = BRep::create_torus(2.0, 0.5);
        const Mesh m = tor.mesh();

        MINI_CHECK(tor.is_valid());
        MINI_CHECK(tor.face_count() == 1);
        MINI_CHECK(tor.edge_count() == 2);
        MINI_CHECK(tor.vertex_count() == 1);
        MINI_CHECK(tor.is_solid());
        MINI_CHECK(tor.name == "torus");
        MINI_CHECK(!m.is_empty());
    }

    MINI_TEST("BRep", "Create Block With Hole") {

        const BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        const Mesh m = bh.mesh();

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

        const double hx = 1.0;
        const double hy = 1.5;
        const double hz = 2.0;
        const Point c[8] = {
            Point(-hx, -hy, -hz),
            Point( hx, -hy, -hz),
            Point( hx,  hy, -hz),
            Point(-hx,  hy, -hz),
            Point(-hx, -hy,  hz),
            Point( hx, -hy,  hz),
            Point( hx,  hy,  hz),
            Point(-hx,  hy,  hz),
        };

        const Polyline bottom(std::vector<Point>{
            c[0],
            c[3],
            c[2],
            c[1],
            c[0],
        });
        const Polyline top(std::vector<Point>{
            c[4],
            c[5],
            c[6],
            c[7],
            c[4],
        });
        const Polyline front(std::vector<Point>{
            c[0],
            c[1],
            c[5],
            c[4],
            c[0],
        });
        const Polyline right(std::vector<Point>{
            c[1],
            c[2],
            c[6],
            c[5],
            c[1],
        });
        const Polyline back(std::vector<Point>{
            c[2],
            c[3],
            c[7],
            c[6],
            c[2],
        });
        const Polyline left(std::vector<Point>{
            c[3],
            c[0],
            c[4],
            c[7],
            c[3],
        });

        const BRep b = BRep::from_polylines({bottom, top, front, right, back, left});
        const Mesh m = b.mesh();

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

        const double hx = 1.0;
        const double hy = 1.5;
        const double hz = 2.0;
        const Point c[8] = {
            Point(-hx, -hy, -hz),
            Point( hx, -hy, -hz),
            Point( hx,  hy, -hz),
            Point(-hx,  hy, -hz),
            Point(-hx, -hy,  hz),
            Point( hx, -hy,  hz),
            Point( hx,  hy,  hz),
            Point(-hx,  hy,  hz),
        };

        const NurbsCurve bottom = NurbsCurve::create(false, 1, {
            c[0],
            c[3],
            c[2],
            c[1],
            c[0],
        });
        const NurbsCurve top = NurbsCurve::create(false, 1, {
            c[4],
            c[5],
            c[6],
            c[7],
            c[4],
        });
        const NurbsCurve front = NurbsCurve::create(false, 1, {
            c[0],
            c[1],
            c[5],
            c[4],
            c[0],
        });
        const NurbsCurve right = NurbsCurve::create(false, 1, {
            c[1],
            c[2],
            c[6],
            c[5],
            c[1],
        });
        const NurbsCurve back = NurbsCurve::create(false, 1, {
            c[2],
            c[3],
            c[7],
            c[6],
            c[2],
        });
        const NurbsCurve left = NurbsCurve::create(false, 1, {
            c[3],
            c[0],
            c[4],
            c[7],
            c[3],
        });

        const BRep b = BRep::from_nurbscurves({bottom, top, front, right, back, left});
        const Mesh m = b.mesh();

        MINI_CHECK(b.is_valid());
        MINI_CHECK(b.face_count() == 6);
        MINI_CHECK(b.edge_count() == 6);
        MINI_CHECK(b.vertex_count() == 5);
        MINI_CHECK(!b.is_solid());
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_faces() > 0);
    }

    MINI_TEST("BRep", "From Nurbscurves Holes") {

        const NurbsCurve outer = NurbsCurve::create(false, 1, {
            Point(-5, -5, 0),
            Point(5, -5, 0),
            Point(5, 5, 0),
            Point(-5, 5, 0),
            Point(-5, -5, 0),
        });
        const NurbsCurve hole = Primitives::circle(0.0, 0.0, 0.0, 2.0);

        const BRep b = BRep::from_nurbscurves({outer}, {{hole}});
        const Mesh m = b.mesh();

        MINI_CHECK(b.is_valid());
        MINI_CHECK(b.face_count() == 1);
        MINI_CHECK(b.wire_count() == 2);
        MINI_CHECK(b.m_faces[0].wires.size() == 2);
        MINI_CHECK(b.m_faces[0].wires[1].index == 1);
        MINI_CHECK(!m.is_empty());
        MINI_CHECK(std::abs(m.area() - (100.0 - Tolerance::PI * 4.0)) < 0.5);
    }

    /// Closed square ring at height z through the (x, y) corners
    static Polyline ring(const std::vector<std::pair<double, double>>& pts, double z) {

        std::vector<Point> v;

        for (const std::pair<double, double>& p : pts)
            v.emplace_back(p.first, p.second, z);

        v.emplace_back(pts[0].first, pts[0].second, z);

        return Polyline(v);
    }

    /// 4 x 4 x 2 box with a 1 x 1 through-hole along z: bottom and top with a hole, four outer and four inner side quads
    static void box_with_square_hole(std::vector<Polyline>& faces, std::vector<std::vector<Polyline>>& holes) {

        const std::vector<std::pair<double, double>> outer{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}};
        const std::vector<std::pair<double, double>> inner{{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
        faces = {ring(outer, 0.0), ring(outer, 2.0)};
        holes = {{ring(inner, 0.0)}, {ring(inner, 2.0)}};

        for (const std::vector<std::pair<double, double>>& pts : {outer, inner})
            for (size_t i = 0; i < 4; ++i) {
                const std::pair<double, double>& a = pts[i];
                const std::pair<double, double>& b = pts[(i + 1) % 4];
                faces.push_back(Polyline(std::vector<Point>{
                    Point(a.first, a.second, 0.0),
                    Point(b.first, b.second, 0.0),
                    Point(b.first, b.second, 2.0),
                    Point(a.first, a.second, 2.0),
                    Point(a.first, a.second, 0.0),
                }));
                holes.push_back({});
            }
    }

    MINI_TEST("BRep", "From Polylines Holes") {

        std::vector<Polyline> faces;
        std::vector<std::vector<Polyline>> holes;
        box_with_square_hole(faces, holes);
        const BRep b = BRep::from_polylines(faces, holes);

        MINI_CHECK(b.face_count() == 10);
        MINI_CHECK(b.m_faces[0].wires.size() == 2);
        MINI_CHECK(b.is_solid());
        MINI_CHECK(std::abs(b.mesh().volume() - 30.0) < 1e-6);
    }

    MINI_TEST("BRep", "Planar Fast Path") {

        std::vector<Polyline> faces;
        std::vector<std::vector<Polyline>> holes;
        box_with_square_hole(faces, holes);
        const BRep b = BRep::from_polylines(faces, holes);
        const std::vector<Mesh> fm = b.face_meshes();
        double total = 0.0;

        for (const Mesh& m : fm)
            total += m.area();

        int tagged = 0;

        for (const std::pair<const size_t, VertexData>& entry : fm[0].vertex)
            for (const std::pair<const std::string, double>& attribute : entry.second.attributes)
                if (attribute.first.rfind("brep_edge/", 0) == 0) {
                    tagged++;
                    break;
                }

        MINI_CHECK(fm.size() == 10);
        MINI_CHECK(fm[0].vertex.size() == 8 && fm[0].face.size() == 8);
        MINI_CHECK(fm[2].vertex.size() == 4 && fm[2].face.size() == 2);
        MINI_CHECK(std::abs(total - 70.0) < 1e-6);
        MINI_CHECK(tagged == 8);
    }

    MINI_TEST("BRep", "Mesh Orientation") {

        const BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        const double vol = bh.mesh().volume();
        const double ref = 8.0 * 6.0 * 4.0 - Tolerance::PI * 1.5 * 1.5 * 4.0;

        MINI_CHECK(std::abs(vol - ref) / ref < 0.02);
    }

    MINI_TEST("BRep", "Protobuf Roundtrip") {

        BRep box = BRep::create_cylinder(1.0, 2.0);
        box.name = "test_brep";
        box.width = 2.0;
        box.surfacecolor = Color(255, 128, 64, 255);

        const BRep loaded_proto = BRep::from_proto(box.to_proto());

        const std::string proto_string = box.pb_dumps();
        const BRep loaded_proto_string = BRep::pb_loads(proto_string);

        const std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_brep.bin").string();
        box.pb_dump(filename);
        const BRep loaded = BRep::pb_load(filename);

        MINI_CHECK(loaded_proto == box);
        MINI_CHECK(loaded_proto_string == box);
        MINI_CHECK(loaded == box);
        MINI_CHECK(loaded.is_solid());
        MINI_CHECK(loaded.m_edges[2].pcurves[0].curve_2d_index_2 >= 0);
        MINI_CHECK(loaded.m_wires[0].edges[2].orientation == BRepOrientation::Reversed);
    }

    MINI_TEST("BRep", "Volume") {

        const BRep box = BRep::create_box(2, 3, 4);
        const BRep cyl = BRep::create_cylinder(1.0, 4.0);
        const BRep sph = BRep::create_sphere(2.0);
        const double vbox = box.volume();
        const double vcyl = cyl.volume();
        const double vsph = sph.volume();

        MINI_CHECK(std::abs(vbox - 24.0) < 1e-9);
        MINI_CHECK(std::abs(vcyl - 4 * Tolerance::PI) / (4 * Tolerance::PI) < 0.05);
        MINI_CHECK(std::abs(vsph - (4.0 / 3.0) * Tolerance::PI * 8) / ((4.0 / 3.0) * Tolerance::PI * 8) < 0.05);
    }

    MINI_TEST("BRep", "Face Polylines Box") {

        const BRep b = BRep::create_box(2.0, 2.0, 2.0);
        const std::vector<Polyline> pls = b.face_polylines();
        const std::vector<Plane> pls_planes = b.face_planes();

        MINI_CHECK(pls.size() == 6);
        MINI_CHECK(pls_planes.size() == pls.size());

        bool seen[3][2] = {{false, false}, {false, false}, {false, false}};

        for (size_t fi = 0; fi < pls.size(); ++fi) {
            const Polyline& p = pls[fi];
            const Plane& pl = pls_planes[fi];
            MINI_CHECK(p.point_count() == 5);
            MINI_CHECK(p.get_point(0) == p.get_point(4));

            int axis = -1;
            double sign = 0.0;

            for (int a = 0; a < 3; ++a) {
                bool constant = true;

                for (size_t i = 1; i < p.point_count(); ++i)
                    if (std::fabs(p.get_point(i)[a] - p.get_point(0)[a]) > 1e-9) {
                        constant = false;
                        break;
                    }

                if (constant && std::fabs(std::fabs(p.get_point(0)[a]) - 1.0) < 1e-9) {
                    axis = a;
                    sign = p.get_point(0)[a] > 0 ? 1.0 : -1.0;
                    break;
                }
            }

            MINI_CHECK(axis >= 0);
            MINI_CHECK(std::fabs(pl.origin()[axis] - sign) < 1e-9);

            const Vector& n = pl.z_axis();
            MINI_CHECK(std::fabs(std::fabs(n[axis]) - 1.0) < 1e-6);

            for (int a2 = 0; a2 < 3; ++a2)
                if (a2 != axis)
                    MINI_CHECK(std::fabs(n[a2]) < 1e-6);

            const int normal_sign = n[axis] > 0.0 ? 1 : 0;
            MINI_CHECK(!seen[axis][normal_sign]);

            seen[axis][normal_sign] = true;
        }

        for (int a = 0; a < 3; ++a)
            for (int s = 0; s < 2; ++s)
                MINI_CHECK(seen[a][s]);
    }

    MINI_TEST("BRep", "Face Polylines Cylinder Caps Only") {

        const BRep b = BRep::create_cylinder(1.0, 4.0);

        MINI_CHECK(b.face_count() == 3);
        MINI_CHECK(b.face_polylines().size() == 2);
        MINI_CHECK(b.face_planes().size() == 2);
    }

    MINI_TEST("BRep", "Face Polylines Ignores Holes") {

        const BRep b = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);
        const std::vector<Polyline> pls = b.face_polylines();

        MINI_CHECK(pls.size() == 6);
        MINI_CHECK(pls.size() == b.face_planes().size());

        for (const Polyline& p : pls) {
            MINI_CHECK(p.point_count() == 5);

            for (size_t i = 0; i < p.point_count(); ++i) {
                const Point pt = p.get_point(i);
                const bool on_bounds = std::fabs(std::fabs(pt[0]) - 2.0) < 1e-6
                                    || std::fabs(std::fabs(pt[1]) - 2.0) < 1e-6
                                    || std::fabs(std::fabs(pt[2]) - 1.0) < 1e-6;
                MINI_CHECK(on_bounds);

                const double radius_to_z_axis = std::sqrt(pt[0] * pt[0] + pt[1] * pt[1]);
                MINI_CHECK(radius_to_z_axis > 1.0 + 1e-6);
            }
        }
    }

    MINI_TEST("BRep", "Face Polylines No Planar Faces") {

        const BRep b = BRep::create_sphere(1.0);

        MINI_CHECK(b.face_polylines().empty());
        MINI_CHECK(b.face_planes().empty());
    }

    MINI_TEST("BRep", "Face Planes Reversed Flip") {

        BRep b;
        const int fi_forward = build_quad_face(b);
        b.add_shell({{fi_forward, BRepOrientation::Forward}});
        const int fi_reversed = build_quad_face(b);
        b.add_shell({{fi_reversed, BRepOrientation::Reversed}});

        MINI_CHECK(b.face_orientation(fi_forward) == BRepOrientation::Forward);
        MINI_CHECK(b.face_orientation(fi_reversed) == BRepOrientation::Reversed);

        const std::vector<Plane> planes = b.face_planes();
        MINI_CHECK(planes.size() == 2);

        const Vector& n_forward = planes[0].z_axis();
        const Vector& n_reversed = planes[1].z_axis();
        MINI_CHECK(std::fabs(n_forward[0] + n_reversed[0]) < 1e-9);
        MINI_CHECK(std::fabs(n_forward[1] + n_reversed[1]) < 1e-9);
        MINI_CHECK(std::fabs(n_forward[2] + n_reversed[2]) < 1e-9);
    }

    MINI_TEST("BRep", "Face Planes Point Outward") {

        const BRep box = BRep::create_box(2.0, 2.0, 2.0);
        const std::vector<Plane> box_planes = box.face_planes();
        MINI_CHECK(box_planes.size() == 6);

        for (const Plane& pl : box_planes) {
            const Point& o = pl.origin();
            const Vector& n = pl.z_axis();
            const double d = o[0] * n[0] + o[1] * n[1] + o[2] * n[2];
            MINI_CHECK(d > 0.0);
        }

        const BRep cyl = BRep::create_cylinder(1.0, 4.0);
        const std::vector<Plane> cyl_planes = cyl.face_planes();
        MINI_CHECK(cyl_planes.size() == 2);

        for (const Plane& pl : cyl_planes) {
            const Point& o = pl.origin();
            const Vector& n = pl.z_axis();
            const double mid_z = 2.0;
            MINI_CHECK((o[2] - mid_z) * n[2] > 0.0);
        }
    }

    MINI_TEST("BRep", "Face Planes Outward Block With Hole") {

        const BRep b = BRep::create_block_with_hole(4.0, 4.0, 2.0, 1.0);
        MINI_CHECK(b.is_solid());

        const Point solid_centroid = Point::centroid(b.vertex_points());
        const std::vector<Plane> planes = b.face_planes();
        MINI_CHECK(planes.size() == 6);

        for (const Plane& pl : planes) {
            const Point& o = pl.origin();
            const Vector& n = pl.z_axis();
            const double d = (o[0] - solid_centroid[0]) * n[0]
                           + (o[1] - solid_centroid[1]) * n[1]
                           + (o[2] - solid_centroid[2]) * n[2];
            MINI_CHECK(d > 0.0);
        }
    }

    MINI_TEST("BRep", "Face Planes Outward Under Mirrored Winding") {

        const Xform mirror = Xform::scale_xyz(-1.0, 1.0, 1.0);
        const BRep b = BRep::create_box(2.0, 2.0, 2.0).transformed(mirror);
        MINI_CHECK(b.is_solid());

        const std::vector<Polyline> pls = b.face_polylines();
        const std::vector<Plane> planes = b.face_planes();
        MINI_CHECK(pls.size() == 6);
        MINI_CHECK(planes.size() == 6);

        const Point solid_centroid = Point::centroid(b.vertex_points());

        for (size_t fi = 0; fi < pls.size(); ++fi) {
            const Point& o = planes[fi].origin();
            const Vector& n = planes[fi].z_axis();
            const double d = (o[0] - solid_centroid[0]) * n[0]
                           + (o[1] - solid_centroid[1]) * n[1]
                           + (o[2] - solid_centroid[2]) * n[2];
            MINI_CHECK(d > 0.0);

            std::vector<Point> expected;

            for (const BRepRef& er : b.wire_edges(b.m_faces[(int)fi].wires[0])) {
                const BRepEdge& edge = b.m_edges[er.index];
                const bool reversed = (er.orientation == BRepOrientation::Reversed);
                const int start = reversed ? edge.end_vertex : edge.start_vertex;
                expected.push_back(b.m_vertices[start].point);
            }

            expected.push_back(expected.front());

            const std::vector<Point> actual = pls[fi].get_points();
            MINI_CHECK(actual.size() == expected.size());

            for (size_t k = 0; k < actual.size(); ++k)
                MINI_CHECK(actual[k] == expected[k]);
        }
    }

} // namespace session_cpp
