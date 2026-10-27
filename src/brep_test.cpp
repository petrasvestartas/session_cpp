#include "mini_test.h"
#include "brep.h"
#include "brep.pb.h"
#include "file_step.h"
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
#include <map>
#include <limits>
#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace session_cpp::mini_test;

namespace session_cpp {

    /// Save the tested BRep and its rendering mesh.
    static void save_brep_geometry(const std::string& name, const BRep& brep) {

        const std::filesystem::path folder = std::filesystem::path("build") / "geometry-tests";
        std::filesystem::create_directories(folder);
        brep.file_json_dump((folder / (name + ".brep.json")).string());
        const Mesh mesh = brep.mesh();
        mesh.file_json_dump((folder / (name + ".mesh.json")).string());
#ifdef SESSION_WITH_OCCT
        if (name.starts_with("rhino_") || name.starts_with("boolean_")) {
            const Mesh controlled = brep.tessellate(0.005, 0.15);
            controlled.file_json_dump((folder / (name + ".controlled.mesh.json")).string());
            MINI_CHECK(std::abs(std::abs(controlled.volume()) - brep.volume_exact()) < brep.volume_exact() * 0.01);
        }
#endif
    }

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

    /// Quarter arc of the given radius in the xz plane at y, from (r, y, 0) to (0, y, r)
    static NurbsCurve quarter_arc(double radius, double y) {

        const double c = radius * std::sqrt(0.5);

        return Primitives::arc(Point(radius, y, 0.0), Point(c, y, c), Point(0.0, y, radius));
    }

    /// Orientation of face fi in a shell where face fj already has orientation oj: the shell runs their shared edge the opposite way on each; -1 when they share no edge
    static int shell_orientation(const BRep& b, int fi, int fj, BRepOrientation oj) {

        for (const BRepRef& er : b.wire_edges(b.m_faces[fi].wires[0]))
            for (const BRepRef& es : b.wire_edges(b.m_faces[fj].wires[0])) {
                if (er.index != es.index)
                    continue;

                const bool direction_j = (es.orientation == BRepOrientation::Forward) != (oj == BRepOrientation::Reversed);
                const bool reversed_i = (er.orientation == BRepOrientation::Forward) == direction_j;

                return reversed_i ? 1 : 0;
            }

        return -1;
    }

    /// Closed solid of six ruled patches between four arcs (outer a, outer b, inner a, inner b) running the same way: the outer and inner faces, the two sides and the two caps, every face wound outwards, the outer face by its distance from the inner and every other by the shell's consistency with an oriented neighbour
    static BRep build_six_patch(const std::array<NurbsCurve, 4>& arcs) {

        BRep b;
        std::map<std::pair<int, int>, int> edges;

        for (int k = 0; k < 4; ++k) {
            b.add_vertex(arcs[k].point_at(arcs[k].domain_start()));
            b.add_vertex(arcs[k].point_at(arcs[k].domain_end()));
            edges[{2 * k, 2 * k + 1}] = b.add_edge(b.add_curve_3d(arcs[k]), 2 * k, 2 * k + 1);
        }

        const std::array<std::pair<int, int>, 4> rulings = {{{0, 2}, {4, 6}, {0, 4}, {2, 6}}};

        for (int end = 0; end < 2; ++end)
            for (const std::pair<int, int>& ruling : rulings) {
                const int start = ruling.first + end;
                const int stop = ruling.second + end;
                const std::vector<Point> ends = {b.m_vertices[start].point, b.m_vertices[stop].point};
                edges[{start, stop}] = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, ends)), start, stop);
            }

        const std::array<NurbsSurface, 6> patches = {
            Primitives::create_ruled(arcs[0], arcs[1]),
            Primitives::create_ruled(arcs[2], arcs[3]),
            Primitives::create_ruled(arcs[0], arcs[2]),
            Primitives::create_ruled(arcs[1], arcs[3]),
            Primitives::create_ruled(b.m_curves_3d[edges.at({0, 2})], b.m_curves_3d[edges.at({4, 6})]),
            Primitives::create_ruled(b.m_curves_3d[edges.at({1, 3})], b.m_curves_3d[edges.at({5, 7})]),
        };
        const std::array<std::array<int, 4>, 6> corners = {{
            {0, 1, 3, 2},
            {4, 5, 7, 6},
            {0, 1, 5, 4},
            {2, 3, 7, 6},
            {0, 2, 6, 4},
            {1, 3, 7, 5},
        }};
        const std::array<Point, 4> uv = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        };

        for (int i = 0; i < 6; ++i) {
            const int si = b.add_surface(patches[i]);
            std::vector<BRepRef> wire;

            for (int k = 0; k < 4; ++k) {
                const int next = (k + 1) % 4;
                const int a = std::min(corners[i][k], corners[i][next]);
                const int z = std::max(corners[i][k], corners[i][next]);
                const int edge = edges.at({a, z});
                const bool forward = b.m_edges[edge].start_vertex == corners[i][k];
                const std::vector<Point> pcurve = forward ? std::vector<Point>{uv[k], uv[next]} : std::vector<Point>{uv[next], uv[k]};
                b.add_pcurve(edge, si, b.add_curve_2d(NurbsCurve::create(false, 1, pcurve)));
                wire.push_back({edge, forward ? BRepOrientation::Forward : BRepOrientation::Reversed});
            }

            b.add_face(si, {{b.add_wire(wire), BRepOrientation::Forward}});
        }

        const Vector away = patches[0].point_at(0.5, 0.5) - patches[1].point_at(0.5, 0.5);
        std::array<int, 6> oriented = {patches[0].normal_at(0.5, 0.5).dot(away) > 0.0 ? 0 : 1, -1, -1, -1, -1, -1};

        for (int pass = 0; pass < 6; ++pass)
            for (int i = 1; i < 6; ++i)
                for (int j = 0; j < 6 && oriented[i] < 0; ++j)
                    if (oriented[j] >= 0)
                        oriented[i] = shell_orientation(b, i, j, oriented[j] == 1 ? BRepOrientation::Reversed : BRepOrientation::Forward);

        std::vector<BRepRef> shell;

        for (int i = 0; i < 6; ++i)
            shell.push_back({i, oriented[i] == 1 ? BRepOrientation::Reversed : BRepOrientation::Forward});

        b.add_solid({{b.add_shell(shell), BRepOrientation::Forward}});

        return b;
    }

    /// Closed solid of six ruled patches between four quarter arcs of radii ro and ri at y = 0 and y = width
    static BRep build_ring_sector(double ro, double ri, double width) {

        return build_six_patch({quarter_arc(ro, 0.0), quarter_arc(ro, width), quarter_arc(ri, 0.0), quarter_arc(ri, width)});
    }

    MINI_TEST("BRep", "Shared Grid Boundary") {
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::BRepRef;
        // using session_cpp::NurbsSurface;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::RemeshNurbsSurfaceGrid;
        // using session_cpp::Tolerance::PI;
        // using session_cpp::Mesh;
        // using session_cpp::Vector;

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

        MINI_CHECK(first == second && first.size() >= 7);
        MINI_CHECK(meshes[0].face.size() == 2 * (first.size() - 1) && meshes[1].face.size() == meshes[0].face.size());

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

        meshes = b.face_meshes_q(true, 5.0, 0.001);
        first = boundary_points(meshes[0]);

        MINI_CHECK(first == boundary_points(meshes[1]) && meshes[0].face.size() == 2 * (first.size() - 1));

        const double cosine = std::cos(5.0 * Tolerance::PI / 180.0);

        for (size_t i = 0; i + 1 < first.size(); ++i) {
            const Vector a = surfaces[0].normal_at(first[i][0], 0.0);
            const Vector z = surfaces[0].normal_at(first[i + 1][0], 0.0);
            MINI_CHECK(a.dot(z) >= cosine - 64.0 * std::numeric_limits<double>::epsilon());
        }
    }

    MINI_TEST("BRep", "Constructor") {
        // using session_cpp::BRep;

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
        // using session_cpp::BRep;

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);

        MINI_CHECK(box.is_valid());
        MINI_CHECK(box.face_count() == 6);
        MINI_CHECK(box.edge_count() == 12);
        MINI_CHECK(box.vertex_count() == 8);
        MINI_CHECK(box.is_solid());
        MINI_CHECK(box.name == "box");
    }

    MINI_TEST("BRep", "Accessors") {
        // using session_cpp::BRep;
        // using session_cpp::Point;

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
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::BRepRef;
        // using session_cpp::NurbsSurface;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;
        // using session_cpp::Vector;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;
        // using session_cpp::Polyline;

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
        // using session_cpp::BRep;

        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRep open = box;
        open.m_shells[0].faces.pop_back();

        MINI_CHECK(box.is_closed(0));
        MINI_CHECK(!box.is_closed(1));
        MINI_CHECK(!open.is_closed(0));
        MINI_CHECK(!open.is_solid());
    }

    MINI_TEST("BRep", "Invalid Topology") {
        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        BRep empty = box;
        empty.m_solids[0].shells.clear();

        MINI_CHECK(!empty.is_valid());
        MINI_CHECK(!empty.is_solid());

        BRep broken = box;
        broken.m_shells[0].faces[0].index = broken.face_count();

        MINI_CHECK(!broken.is_valid());
        MINI_CHECK(!broken.is_closed(0));
        MINI_CHECK(!broken.is_solid());

        broken = box;
        broken.m_faces[0].wires[0].index = broken.wire_count();

        MINI_CHECK(!broken.is_closed(0));

        broken = box;
        broken.m_wires[0].edges[0].index = broken.edge_count();

        MINI_CHECK(!broken.is_closed(0));

        broken = box;
        broken.m_edges[0].degenerated = true;

        MINI_CHECK(!broken.is_valid());

        broken = box;
        broken.m_vertices[0].point[0] = std::numeric_limits<double>::quiet_NaN();

        MINI_CHECK(!broken.is_valid());

        broken = box;
        broken.m_edges[0].tolerance = -1.0;

        MINI_CHECK(!broken.is_valid());

        broken = box;
        broken.m_curves_3d[0].m_nurbsknot[0] = std::numeric_limits<double>::quiet_NaN();

        MINI_CHECK(!broken.is_valid());

        broken = box;
        broken.m_surfaces[0].m_cv[0] = std::numeric_limits<double>::infinity();

        MINI_CHECK(!broken.is_valid());

        broken = box;
        for (double& knot : broken.m_surfaces[0].m_nurbsknot[0])
            knot = 0.0;

        MINI_CHECK(!broken.is_valid());
    }

    MINI_TEST("BRep", "Disconnected Boundary Wire") {
        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        std::swap(box.m_wires[0].edges[0], box.m_wires[0].edges[1]);

        MINI_CHECK(!box.is_valid());
        MINI_CHECK(!box.is_solid());
    }

    MINI_TEST("BRep", "Inconsistent Shell Orientation") {
        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        box.m_shells[0].faces[0].orientation = BRepOrientation::Reversed;

        MINI_CHECK(box.is_closed(0));
        MINI_CHECK(!box.is_solid());
    }

    MINI_TEST("BRep", "Shell Boundary Orientations") {
        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        box.m_shells[0].faces.push_back({0, BRepOrientation::Internal});

        MINI_CHECK(box.is_closed(0));

        box.m_shells[0].faces.back().orientation = BRepOrientation::External;

        MINI_CHECK(box.is_closed(0));

        for (BRepRef& face : box.m_shells[0].faces)
            face.orientation = BRepOrientation::Internal;

        MINI_CHECK(!box.is_closed(0));
    }

    MINI_TEST("BRep", "Reversed Solid Shell") {
        BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const Vector normal = box.normal_at(0, 0.5, 0.5);
        box.m_solids[0].shells[0].orientation = BRepOrientation::Reversed;
        const Vector reversed = box.normal_at(0, 0.5, 0.5);

        MINI_CHECK(box.face_orientation(0) == BRepOrientation::Reversed);
        MINI_CHECK(normal.dot(reversed) < -0.99);
        MINI_CHECK(box.is_solid());
    }

    MINI_TEST("BRep", "Offset Convex Box") {
        const BRep box = BRep::create_box(2.0, 3.0, 4.0);
        const BRep outward = box.offset_convex(0.25);
        const BRep inward = box.offset_convex(-0.25);

        MINI_CHECK(outward.is_valid() && outward.is_solid() && edges_manifold(outward));
        MINI_CHECK(inward.is_valid() && inward.is_solid() && edges_manifold(inward));
        MINI_CHECK(std::abs(outward.volume() - 2.5 * 3.5 * 4.5) < 1e-8);
        MINI_CHECK(std::abs(inward.volume() - 1.5 * 2.5 * 3.5) < 1e-8);
        MINI_CHECK(box.offset_convex(-1.0).face_count() == 0);
        MINI_CHECK(box.offset_convex(-2.0).face_count() == 0);
        MINI_CHECK(box.offset_convex(0.0) == box);

        save_brep_geometry("offset_box_original", box);
        save_brep_geometry("offset_box_outward", outward);
        save_brep_geometry("offset_box_inward", inward);
    }

    MINI_TEST("BRep", "Offset Convex Platonic Solids") {
        const std::array<Mesh, 5> meshes = {
            Primitives::tetrahedron(), Primitives::cube(), Primitives::octahedron(),
            Mesh::create_dodecahedron(), Primitives::icosahedron()
        };
        const std::array<std::string, 5> names = {"tetrahedron", "cube", "octahedron", "dodecahedron", "icosahedron"};

        for (size_t i = 0; i < meshes.size(); ++i) {
            std::vector<Polyline> polygons;
            for (const std::pair<const size_t, std::vector<size_t>>& face : meshes[i].face)
                polygons.push_back(meshes[i].face_polygon(face.first).value());

            const BRep solid = BRep::from_polylines(polygons);
            const std::vector<Plane> planes = solid.face_planes();
            const double radius = planes[0].z_axis().dot(planes[0].origin() - Point());
            const double volume = solid.volume();
            const BRep outward = solid.offset_convex(radius * 0.2);
            const BRep inward = solid.offset_convex(-radius * 0.2);

            MINI_CHECK(solid.is_solid());
            MINI_CHECK(outward.is_solid() && edges_manifold(outward));
            MINI_CHECK(inward.is_solid() && edges_manifold(inward));
            MINI_CHECK(outward.face_count() == solid.face_count());
            MINI_CHECK(inward.face_count() == solid.face_count());
            MINI_CHECK(std::abs(outward.volume() / volume - 1.728) < 1e-8);
            MINI_CHECK(std::abs(inward.volume() / volume - 0.512) < 1e-8);
            MINI_CHECK(solid.offset_convex(-radius * 1.1).face_count() == 0);

            save_brep_geometry("offset_" + names[i] + "_original", solid);
            save_brep_geometry("offset_" + names[i] + "_outward", outward);
            save_brep_geometry("offset_" + names[i] + "_inward", inward);
        }
    }

    MINI_TEST("BRep", "Offset Convex Face Disappears") {
        const BRep box = BRep::create_box(2.0, 2.0, 2.0);
        const Plane bevel = Plane::from_point_normal({2.8 / 3.0, 2.8 / 3.0, 2.8 / 3.0}, {-1, -1, -1});
        const BRep clipped = box.cut_by_plane(bevel);
        const BRep offset = clipped.offset_convex(-0.5);

        MINI_CHECK(clipped.face_count() == 7);
        MINI_CHECK(offset.face_count() == 6);
        MINI_CHECK(offset.is_solid() && edges_manifold(offset));
        MINI_CHECK(std::abs(offset.volume() - 1.0) < 1e-8);

        save_brep_geometry("offset_bevel_original", clipped);
        save_brep_geometry("offset_bevel_inward", offset);
    }

    MINI_TEST("BRep", "Offset Convex Unsupported Surface") {
        bool rejected = false;
        try {
            BRep::create_sphere(1.0).offset_convex(0.1);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }

        MINI_CHECK(rejected);
    }

    MINI_TEST("BRep", "Singular And Periodic Solid Boundaries") {
        const std::array<BRep, 5> solids = {
            BRep::create_sphere(1.0), BRep::create_cone(1.0, 2.0),
            BRep::create_torus(2.0, 0.25), BRep::create_block_with_hole(4.0, 4.0, 1.0, 1.8),
            build_ring_sector(2.0, 1.98, 0.3)
        };
        const std::array<std::string, 5> names = {
            "sphere_poles", "cone_apex", "torus_seams", "thin_bore_walls", "thin_rational_shell"
        };

        for (size_t i = 0; i < solids.size(); ++i) {
            const BRep& solid = solids[i];
            MINI_CHECK(solid.is_valid());
            MINI_CHECK(solid.is_solid());
            MINI_CHECK(edges_manifold(solid));
            MINI_CHECK(solid.volume() > 0.0);

            const BRep restored = BRep::pb_loads(solid.pb_dumps());
            MINI_CHECK(restored.is_valid() && restored.is_solid());
            MINI_CHECK(std::abs(restored.volume() - solid.volume()) < 1e-9);

            save_brep_geometry(names[i], solid);
        }
    }

    MINI_TEST("BRep", "Rhino Freeform Chairs Import") {
        const std::filesystem::path source = std::filesystem::path(__FILE__).parent_path().parent_path();
        const std::vector<BRep> chairs = file_step::read_file_step_breps((source / "tests/fixtures/solidbrep.stp").string());

        MINI_CHECK(chairs.size() == 2);
        for (size_t i = 0; i < chairs.size(); ++i) {
            const BRep& chair = chairs[i];
            MINI_CHECK(chair.face_count() == 20);
            MINI_CHECK(chair.is_valid());
            MINI_CHECK(chair.is_solid());
            MINI_CHECK(edges_manifold(chair));
            int curved = 0;
            for (const NurbsSurface& surface : chair.m_surfaces)
                if (surface.degree(0) > 1 && surface.degree(1) > 1 && !surface.is_planar())
                    ++curved;
            MINI_CHECK(curved > 0);

            save_brep_geometry("rhino_chair_" + std::to_string(i), chair);
        }
    }

#ifdef SESSION_WITH_OCCT
    MINI_TEST("BRep", "Boolean OCCT Box Configurations") {
        const std::filesystem::path source = std::filesystem::path(__FILE__).parent_path().parent_path();
        std::ifstream input(source / "tests/fixtures/occt_boolean_boxes.json");
        const nlohmann::json fixtures = nlohmann::json::parse(input);
        nlohmann::ordered_json report = nlohmann::ordered_json::array();
        MINI_CHECK(fixtures["cases"].size() == 38);
        for (const nlohmann::json& fixture : fixtures["cases"]) {
            std::array<BRep, 2> boxes;
            std::array<double, 2> volumes;
            for (int i = 0; i < 2; ++i) {
                const std::vector<double> values = fixture["boxes"][i].get<std::vector<double>>();
                boxes[i] = BRep::create_box(values[3], values[4], values[5]).transformed(
                    Xform::translation(values[0] + values[3] / 2, values[1] + values[4] / 2, values[2] + values[5] / 2)
                );
                volumes[i] = values[3] * values[4] * values[5];
            }
            const BRep& a = boxes[0];
            const BRep& b = boxes[1];
            const double overlap = fixture["intersection_volume"];
            const std::array<BRep, 5> results = {
                a.boolean_intersection(b), a.boolean_union(b), a.boolean_difference(b),
                b.boolean_difference(a), a.boolean_xor(b)
            };
            const std::array<double, 5> expected = {
                overlap, volumes[0] + volumes[1] - overlap, volumes[0] - overlap,
                volumes[1] - overlap, volumes[0] + volumes[1] - 2 * overlap
            };
            std::vector<double> measured;
            for (size_t i = 0; i < results.size(); ++i) {
                MINI_CHECK(results[i].face_count() == 0 || results[i].is_solid());
                measured.push_back(results[i].volume_exact());
                MINI_CHECK(std::abs(measured.back() - expected[i]) < std::max(1.0, expected[i]) * 1e-6);
            }
            report.push_back({{"sources", fixture["sources"]}, {"expected", expected}, {"volumes", measured}});
        }
        std::ofstream("build/geometry-tests/occt_boolean_boxes_report.json") << report.dump(2);
    }

    MINI_TEST("BRep", "Boolean Platonic Solids") {
        const std::array<Mesh, 5> meshes = {
            Primitives::tetrahedron(), Primitives::cube(), Primitives::octahedron(),
            Mesh::create_dodecahedron(), Primitives::icosahedron()
        };
        const std::array<std::string, 5> names = {"tetrahedron", "cube", "octahedron", "dodecahedron", "icosahedron"};
        for (size_t i = 0; i < meshes.size(); ++i) {
            std::vector<Polyline> polygons;
            for (const std::pair<const size_t, std::vector<size_t>>& face : meshes[i].face)
                polygons.push_back(meshes[i].face_polygon(face.first).value());
            const BRep a = BRep::from_polylines(polygons);
            const BRep b = a.transformed(Xform::rotation({1, 2, 3}, 23.0, true));
            const BRep common = a.boolean_intersection(b);
            const BRep united = a.boolean_union(b);
            const BRep cut = a.boolean_difference(b);
            const BRep exclusive = a.boolean_xor(b);
            const double volume = a.volume_exact();
            const double overlap = common.volume_exact();
            MINI_CHECK(common.is_solid());
            MINI_CHECK(united.is_solid());
            MINI_CHECK(cut.is_solid());
            MINI_CHECK(exclusive.is_solid());
            MINI_CHECK(overlap > 0 && overlap < volume);
            MINI_CHECK(std::abs(united.volume_exact() + overlap - 2 * volume) < volume * 1e-6);
            MINI_CHECK(std::abs(cut.volume_exact() + overlap - volume) < volume * 1e-6);
            MINI_CHECK(std::abs(exclusive.volume_exact() + 2 * overlap - 2 * volume) < volume * 1e-6);
            save_brep_geometry("boolean_platonic_" + names[i] + "_intersection", common);
            save_brep_geometry("boolean_platonic_" + names[i] + "_difference", cut);
        }
    }

    MINI_TEST("BRep", "Boolean Curved Seams And Poles") {
        const BRep sphere = BRep::create_sphere(1.0);
        const double sphere_volume = 4.0 * Tolerance::PI / 3.0;
        MINI_CHECK(std::abs(sphere.volume_exact() - sphere_volume) < 1e-6);
        for (double distance : {0.0, 1.0, 1.999, 2.0, 2.001}) {
            const BRep other = sphere.transformed(Xform::translation(distance, 0.0, 0.0));
            const double overlap = distance < 2.0
                ? Tolerance::PI * (4.0 + distance) * std::pow(2.0 - distance, 2) / 12.0 : 0.0;
            const BRep common = sphere.boolean_intersection(other);
            const BRep united = sphere.boolean_union(other);
            const BRep cut = sphere.boolean_difference(other);
            const BRep exclusive = sphere.boolean_xor(other);
            MINI_CHECK(distance >= 2.0 || common.is_solid());
            MINI_CHECK(std::abs(common.volume_exact() - overlap) < 1e-5);
            MINI_CHECK(std::abs(united.volume_exact() - 2.0 * sphere_volume + overlap) < 1e-5);
            MINI_CHECK(std::abs(cut.volume_exact() - sphere_volume + overlap) < 1e-5);
            MINI_CHECK(std::abs(exclusive.volume_exact() - 2.0 * sphere_volume + 2.0 * overlap) < 1e-5);
            if (distance == 1.0) {
                save_brep_geometry("boolean_spheres_intersection", common);
                save_brep_geometry("boolean_spheres_difference", cut);
                save_brep_geometry("boolean_spheres_xor", exclusive);
            }
        }

        const BRep torus = BRep::create_torus(2.0, 0.1);
        const BRep cutter = BRep::create_box(6.0, 6.0, 1.0).transformed(Xform::translation(0.0, 0.0, 0.5));
        const BRep half = torus.boolean_intersection(cutter);
        const double expected = Tolerance::PI * Tolerance::PI * 2.0 * 0.1 * 0.1;
        MINI_CHECK(half.is_solid());
        MINI_CHECK(std::abs(half.volume_exact() - expected) < 1e-6);
        save_brep_geometry("boolean_torus_half", half);
    }

    MINI_TEST("BRep", "Offset Curved Solids") {
        for (double distance : {-0.1, 0.1}) {
            const BRep sphere = BRep::create_sphere(1.0).offset(distance);
            const double radius = 1.0 + distance;
            MINI_CHECK(sphere.is_solid());
            MINI_CHECK(std::abs(sphere.volume_exact() - 4.0 * Tolerance::PI * radius * radius * radius / 3.0) < 1e-5);
            save_brep_geometry(distance < 0 ? "offset_sphere_inward" : "offset_sphere_outward", sphere);
        }
    }

    MINI_TEST("BRep", "Hollow Solid With Opening") {
        const BRep box = BRep::create_box(2.0, 2.0, 2.0);
        const BRep hollow = box.hollow({0}, -0.2);
        MINI_CHECK(hollow.is_valid());
        MINI_CHECK(hollow.is_solid());
        MINI_CHECK(std::abs(hollow.volume_exact() - (8.0 - 1.6 * 1.6 * 1.8)) < 1e-5);
        save_brep_geometry("boolean_hollow_box", hollow);
    }

    MINI_TEST("BRep", "Boolean Scale And Translation") {
        for (double scale : {0.001, 1.0, 1000.0}) {
            const BRep first = BRep::create_box(2 * scale, 2 * scale, 2 * scale);
            const BRep second = first.transformed(Xform::translation(scale, scale, scale));
            const BRep common = first.boolean_intersection(second);
            MINI_CHECK(common.is_solid());
            MINI_CHECK(std::abs(common.volume_exact() / (scale * scale * scale) - 1.0) < 1e-5);
        }
        const BRep first = BRep::create_box(2, 2, 2).transformed(Xform::translation(1e6, -1e6, 1e6));
        const BRep second = first.transformed(Xform::translation(1, 1, 1));
        const BRep common = first.boolean_intersection(second);
        MINI_CHECK(common.is_solid());
        MINI_CHECK(std::abs(common.volume_exact() - 1.0) < 1e-5);
    }

    MINI_TEST("BRep", "Boolean Solid Configurations") {
        const BRep box = BRep::create_box(2.0, 2.0, 2.0);
        const std::array<double, 5> translations = {0.0, 0.75, 2.0, 2.5, -0.75};
        for (double shift : translations) {
            const BRep other = box.transformed(Xform::translation(shift, 0.0, 0.0));
            const double common = std::max(0.0, 2.0 - std::abs(shift)) * 4.0;
            const BRep united = box.boolean_union(other);
            const BRep intersected = box.boolean_intersection(other);
            const BRep difference = box.boolean_difference(other);
            const BRep exclusive = box.boolean_xor(other);

            MINI_CHECK(std::abs(united.volume_exact() - (16.0 - common)) < 1e-6);
            MINI_CHECK(std::abs(intersected.volume_exact() - common) < 1e-6);
            MINI_CHECK(std::abs(difference.volume_exact() - (8.0 - common)) < 1e-6);
            MINI_CHECK(std::abs(exclusive.volume_exact() - (16.0 - 2.0 * common)) < 1e-6);
            for (const BRep* result : {&united, &intersected, &difference, &exclusive})
                MINI_CHECK(result->face_count() == 0 || (result->is_valid() && result->is_solid()));
        }

        const BRep inner = BRep::create_box(1.0, 1.0, 1.0);
        const BRep cavity = box.boolean_difference(inner);
        MINI_CHECK(cavity.solid_count() == 1);
        MINI_CHECK(cavity.shell_count() == 2);
        MINI_CHECK(std::abs(cavity.volume_exact() - 7.0) < 1e-6);
        save_brep_geometry("boolean_box_cavity", cavity);
    }

    MINI_TEST("BRep", "Rhino Freeform Chairs Booleans") {
        const std::filesystem::path source = std::filesystem::path(__FILE__).parent_path().parent_path();
        const std::vector<BRep> chairs = file_step::read_file_step_breps((source / "tests/fixtures/solidbrep.stp").string());
        MINI_CHECK(chairs.size() == 2);
        const BRep& a = chairs[0];
        const BRep& b = chairs[1];
        const double va = a.volume_exact();
        const double vb = b.volume_exact();
        const double tolerance = (va + vb) * 1e-6;
        const std::array<BRep, 5> results = {
            a.boolean_intersection(b), a.boolean_union(b), a.boolean_difference(b),
            b.boolean_difference(a), a.boolean_xor(b)
        };
        const std::array<std::string, 5> names = {"intersection", "union", "difference_ab", "difference_ba", "xor"};
        std::array<double, 5> volumes;
        nlohmann::ordered_json report = {{"input_volumes", {va, vb}}, {"results", nlohmann::ordered_json::array()}};
        for (size_t i = 0; i < results.size(); ++i) {
            const BRep& result = results[i];
            MINI_CHECK(result.is_valid());
            MINI_CHECK(result.is_solid());
            int curved = 0;
            for (const NurbsSurface& surface : result.m_surfaces)
                if (surface.degree(0) > 1 && surface.degree(1) > 1 && !surface.is_planar())
                    ++curved;
            MINI_CHECK(curved > 0);
            double maximum_gap = 0.0;
            double maximum_tolerance = 0.0;
            for (const BRepEdge& edge : result.m_edges) {
                maximum_tolerance = std::max(maximum_tolerance, edge.tolerance);
                if (edge.degenerated)
                    continue;
                const NurbsCurve& curve = result.m_curves_3d[edge.curve_3d_index];
                for (const BRepCurveOnSurface& representation : edge.pcurves) {
                    const NurbsSurface& surface = result.m_surfaces[representation.surface_index];
                    for (int index : {representation.curve_2d_index, representation.curve_2d_index_2}) {
                        if (index < 0)
                            continue;
                        const NurbsCurve& pcurve = result.m_curves_2d[index];
                        for (int j = 0; j <= 32; ++j) {
                            const double fraction = j / 32.0;
                            const Point point = curve.point_at(curve.domain_start() + (curve.domain_end() - curve.domain_start()) * fraction);
                            const Point uv = pcurve.point_at(pcurve.domain_start() + (pcurve.domain_end() - pcurve.domain_start()) * fraction);
                            maximum_gap = std::max(maximum_gap, point.distance(surface.point_at(uv[0], uv[1])));
                        }
                    }
                }
            }
            MINI_CHECK(maximum_tolerance < 2e-5);
            MINI_CHECK(maximum_gap < 2e-5);
            volumes[i] = result.volume_exact();
            MINI_CHECK(volumes[i] > 0.0);
            const BRep restored = BRep::pb_loads(result.pb_dumps());
            MINI_CHECK(restored.is_solid());
            MINI_CHECK(std::abs(restored.volume_exact() - volumes[i]) < tolerance);
            save_brep_geometry("rhino_chairs_" + names[i], result);
            report["results"].push_back({
                {"operation", names[i]}, {"volume", volumes[i]}, {"faces", result.face_count()},
                {"solids", result.solid_count()}, {"doubly_curved_faces", curved},
                {"maximum_edge_tolerance", maximum_tolerance}, {"maximum_edge_surface_gap", maximum_gap}
            });
        }

        MINI_CHECK(volumes[0] < std::min(va, vb));
        MINI_CHECK(std::abs(volumes[1] + volumes[0] - va - vb) < tolerance);
        MINI_CHECK(std::abs(volumes[2] + volumes[0] - va) < tolerance);
        MINI_CHECK(std::abs(volumes[3] + volumes[0] - vb) < tolerance);
        MINI_CHECK(std::abs(volumes[4] - volumes[2] - volumes[3]) < tolerance);
        MINI_CHECK(std::abs(volumes[4] - volumes[1] + volumes[0]) < tolerance);
        std::ofstream("build/geometry-tests/rhino_chairs_report.json") << report.dump(2);
    }

    MINI_TEST("BRep", "Rhino Freeform Chairs Inward Offsets") {
        const std::filesystem::path source = std::filesystem::path(__FILE__).parent_path().parent_path();
        const std::vector<BRep> chairs = file_step::read_file_step_breps((source / "tests/fixtures/solidbrep.stp").string());
        const std::array<double, 2> reference = {2913.7708513751622, 2913.7708514508217};
        MINI_CHECK(chairs.size() == 2);
        for (size_t i = 0; i < chairs.size(); ++i) {
            const BRep offset = chairs[i].offset(-0.01);
            MINI_CHECK(offset.is_valid());
            MINI_CHECK(offset.is_solid());
            MINI_CHECK(offset.volume_exact() < chairs[i].volume_exact());
            MINI_CHECK(std::abs(offset.volume_exact() - reference[i]) < 0.001);
            save_brep_geometry("rhino_offset_inward_" + std::to_string(i), offset);
        }
    }
#endif

    MINI_TEST("BRep", "Wire Edges") {
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::BRepRef;
        // using session_cpp::brep_compose;
        // using session_cpp::brep_reverse;

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
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::BRepRef;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;

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
        // using session_cpp::BRep;
        // using session_cpp::Xform;
        // using session_cpp::Point;

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
        // using session_cpp::BRep;
        // using session_cpp::Vector;
        // using session_cpp::Xform;
        // using session_cpp::Point;

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
        // using session_cpp::BRep;
        // using session_cpp::Plane;
        // using session_cpp::Point;
        // using session_cpp::Vector;
        // using session_cpp::Xform;

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
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::Color;

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
        // using session_cpp::BRep;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Primitives;
        // using session_cpp::Tolerance::PI;
        // using session_cpp::Mesh;

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
        // using session_cpp::BRep;
        // using session_cpp::Polyline;

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
        // using session_cpp::BRep;
        // using session_cpp::Polyline;
        // using session_cpp::Mesh;
        // using session_cpp::VertexData;

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

    MINI_TEST("BRep", "Strip Fast Path") {
        // using session_cpp::BRep;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::VertexData;
        // using session_cpp::Tolerance::PI;

        const BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        const std::vector<Mesh> fm = bh.face_meshes_q(true, 10.0, 0.005);
        const Mesh& bore = fm[4];
        int round = 0;
        int rim = 0;
        int seam = 0;
        int shared = 0;

        for (const std::pair<const size_t, VertexData>& entry : bore.vertex) {
            const Point p = entry.second.position();

            if (std::abs(std::sqrt(p[0] * p[0] + p[1] * p[1]) - 1.5) < 1e-9 && entry.second.attributes.count("u") && entry.second.attributes.count("v"))
                round++;

            for (const std::pair<const std::string, double>& attribute : entry.second.attributes) {
                if (attribute.first.rfind("brep_edge/12/0/", 0) == 0)
                    rim++;

                if (attribute.first.rfind("brep_edge/14/", 0) == 0)
                    seam++;
            }

            bool on_cap = false;

            for (const int cap : {5, 6})
                for (const std::pair<const size_t, VertexData>& other : fm[cap].vertex)
                    on_cap = on_cap || other.second.position().distance(p) == 0.0;

            if (on_cap)
                shared++;
        }

        const std::vector<Mesh> body = BRep::create_cylinder(1.0, 2.0).face_meshes();
        const double volume = bh.mesh().volume();
        const double ref = 8.0 * 6.0 * 4.0 - Tolerance::PI * 1.5 * 1.5 * 4.0;

        MINI_CHECK(bore.face.size() == 64 && bore.vertex.size() == 66);
        MINI_CHECK(round == 66 && rim == 33 && seam == 4 && shared == 66);
        MINI_CHECK(body[0].face.size() == 72 && body[0].vertex.size() == 74);
        MINI_CHECK(std::abs(volume - ref) / ref < 0.005);
    }

    MINI_TEST("BRep", "Strip Step Share") {
        const std::vector<Mesh> body = BRep::create_cylinder(150.0, 400.0).face_meshes_q(true, 5.0, 0.001);
        const std::vector<Mesh> bore = BRep::create_block_with_hole(100.0, 100.0, 50.0, 10.0).face_meshes_q(true, 5.0, 0.001);
        const std::vector<Mesh> rib = BRep::create_block_with_hole(4000.0, 200.0, 100.0, 10.0).face_meshes_q(true, 5.0, 0.001);

        MINI_CHECK(body[0].face.size() == 144 && body[0].vertex.size() == 146);
        MINI_CHECK(bore[4].face.size() == 64 && bore[4].vertex.size() == 66);
        MINI_CHECK(rib[4].face.size() == 64 && rib[4].vertex.size() == 66);
        MINI_CHECK(rib[5].vertex.size() == 36 && rib[6].vertex.size() == 36);
    }

    MINI_TEST("BRep", "Mesh Watertight") {
        const std::vector<BRep> bodies = {
            BRep::create_box(8.0, 6.0, 4.0),
            BRep::create_cylinder(2.0, 5.0),
            BRep::create_sphere(2.0),
            BRep::create_cone(2.0, 5.0),
            BRep::create_torus(4.0, 1.0),
            BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5),
        };
        const auto key = [](const Point& p) { return std::array<double, 3>{p[0] + 0.0, p[1] + 0.0, p[2] + 0.0}; };
        int empty = 0;
        int open = 0;
        int missing = 0;

        for (const BRep& b : bodies) {
            const std::vector<Mesh> fms = b.face_meshes_q(true, 10.0, 0.005);
            std::map<std::pair<std::array<double, 3>, std::array<double, 3>>, int> edges;

            for (const Mesh& fm : fms) {
                if (fm.face.empty())
                    ++empty;

                std::map<std::pair<std::array<double, 3>, std::array<double, 3>>, int> local;

                for (const std::pair<const size_t, std::vector<size_t>>& face : fm.face) {
                    const size_t n = face.second.size();

                    for (size_t i = 0; i < n; ++i) {
                        const std::array<double, 3> a = key(fm.vertex.at(face.second[i]).position());
                        const std::array<double, 3> c = key(fm.vertex.at(face.second[(i + 1) % n]).position());
                        local[{a, c}] += 1;
                        local[{c, a}] -= 1;
                    }
                }

                for (const auto& entry : local)
                    if (entry.second > 0)
                        edges[entry.first] += entry.second;
            }

            for (const auto& entry : edges) {
                const auto reverse = edges.find({entry.first.second, entry.first.first});

                if ((reverse == edges.end() ? 0 : reverse->second) != entry.second)
                    ++open;
            }

            for (int ei = 0; ei < b.edge_count(); ++ei) {
                if (b.m_edges[ei].degenerated)
                    continue;

                for (const BRepRef& fr : b.edge_faces(ei)) {
                    for (const int vi : {b.m_edges[ei].start_vertex, b.m_edges[ei].end_vertex}) {
                        const std::array<double, 3> p = key(b.m_vertices[vi].point);
                        bool found = false;

                        for (const std::pair<const size_t, VertexData>& vd : fms[fr.index].vertex)
                            found = found || key(vd.second.position()) == p;

                        if (!found)
                            ++missing;
                    }
                }
            }
        }

        MINI_CHECK(empty == 0);
        MINI_CHECK(open == 0);
        MINI_CHECK(missing == 0);
    }

    MINI_TEST("BRep", "Ruled Watertight") {
        const auto at = [](double r, double y, double degrees) { return Point(r * std::cos(degrees * Tolerance::PI / 180.0), y, r * std::sin(degrees * Tolerance::PI / 180.0)); };
        const std::array<NurbsCurve, 4> meridians = {
            Primitives::arc(at(1000.0, 0.0, 165.0), at(1000.0, 0.0, 90.0), at(1000.0, 0.0, 15.0)),
            Primitives::arc(at(1300.0, 300.0, 165.0), at(1300.0, 300.0, 90.0), at(1300.0, 300.0, 15.0)),
            Primitives::arc(at(880.0, 0.0, 165.0), at(880.0, 0.0, 90.0), at(880.0, 0.0, 15.0)),
            Primitives::arc(at(1180.0, 300.0, 165.0), at(1180.0, 300.0, 90.0), at(1180.0, 300.0, 15.0)),
        };
        const BRep b = build_six_patch(meridians);
        const std::vector<Mesh> fms = b.face_meshes();
        const auto key = [](const Point& p) { return std::array<double, 3>{p[0] + 0.0, p[1] + 0.0, p[2] + 0.0}; };
        std::map<std::pair<std::array<double, 3>, std::array<double, 3>>, int> directed;
        int empty = 0;
        int inward = 0;
        int spanning = 0;
        int open = 0;

        for (size_t fi = 0; fi < fms.size(); ++fi) {
            const Mesh& fm = fms[fi];

            if (fm.face.empty())
                ++empty;

            for (const std::pair<const size_t, std::vector<size_t>>& face : fm.face) {
                const Point a = fm.vertex.at(face.second[0]).position();
                const Point c = fm.vertex.at(face.second[1]).position();
                const Point d = fm.vertex.at(face.second[2]).position();
                double u = 0.0;
                double v = 0.0;
                std::vector<double> stations;

                for (const size_t vk : face.second) {
                    const Attributes& attributes = fm.vertex.at(vk).attributes;
                    u += (attributes.count("u") ? attributes.at("u") : 0.5) / 3.0;
                    v += (attributes.count("v") ? attributes.at("v") : 0.5) / 3.0;
                    stations.push_back(attributes.count("u") ? attributes.at("u") : 0.5);
                }

                std::sort(stations.begin(), stations.end());

                if (stations[1] - stations[0] > 1e-6 && stations[2] - stations[1] > 1e-6 && fi < 4)
                    ++spanning;

                if ((c - a).cross(d - a).dot(b.normal_at((int)fi, u, v)) <= 0.0)
                    ++inward;

                for (size_t i = 0; i < 3; ++i) {
                    const std::array<double, 3> from = key(fm.vertex.at(face.second[i]).position());
                    const std::array<double, 3> to = key(fm.vertex.at(face.second[(i + 1) % 3]).position());
                    directed[{from, to}] += 1;
                }
            }
        }

        for (const auto& entry : directed) {
            const auto reverse = directed.find({entry.first.second, entry.first.first});

            if (entry.second != 1 || reverse == directed.end() || reverse->second != 1)
                ++open;
        }

        std::vector<std::vector<Point>> polygons;

        for (const Mesh& fm : b.face_meshes_q(true, 2.0, 0.0005))
            for (const std::pair<const size_t, std::vector<size_t>>& face : fm.face)
                polygons.push_back({fm.vertex.at(face.second[0]).position(), fm.vertex.at(face.second[1]).position(), fm.vertex.at(face.second[2]).position()});

        const double volume = b.volume();
        const double fine = Mesh::from_polylines(polygons, 1e-6).volume();
        const double outer = (double)fms[0].face.size();
        const double inner = (double)fms[1].face.size();

        MINI_CHECK(b.is_solid() && empty == 0);
        MINI_CHECK(open == 0);
        MINI_CHECK(inward == 0 && spanning == 0);
        MINI_CHECK(std::abs(fine - 1.0273e8) / 1.0273e8 < 0.01 && std::abs(volume - fine) / fine < 0.05);
        MINI_CHECK(outer / inner > 0.5 && outer / inner < 2.0);
    }

    MINI_TEST("BRep", "Mesh Orientation") {
        // using session_cpp::BRep;
        // using session_cpp::Tolerance::PI;

        const BRep bh = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
        const double vol = bh.mesh().volume();
        const double ref = 8.0 * 6.0 * 4.0 - Tolerance::PI * 1.5 * 1.5 * 4.0;

        MINI_CHECK(std::abs(vol - ref) / ref < 0.02);
    }

    MINI_TEST("BRep", "Protobuf Roundtrip") {
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::Color;

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
        // using session_cpp::BRep;
        // using session_cpp::Tolerance::PI;

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

    MINI_TEST("BRep", "Volume Far From Origin") {
        // using session_cpp::BRep;
        // using session_cpp::Xform;

        const Xform far = Xform::translation(1000000.1, 1000000.2, 1000000.3);
        const BRep box = BRep::create_box(2.0, 3.0, 4.0).transformed(far);

        MINI_CHECK(std::abs(box.volume() / 24.0 - 1.0) < 1e-9);
    }

    MINI_TEST("BRep", "Volume Planar Curved Boundary") {
        // using session_cpp::BRep;
        // using session_cpp::Mesh;
        // using session_cpp::Tolerance::PI;

        const BRep sector = build_ring_sector(2000.0, 1880.0, 600.0);
        const std::vector<Mesh> fm = sector.face_meshes();
        const Mesh m = sector.mesh();
        const double ref = Tolerance::PI / 4.0 * (2000.0 * 2000.0 - 1880.0 * 1880.0) * 600.0;

        MINI_CHECK(sector.is_solid() && fm.size() == 6);
        MINI_CHECK(fm[2].face.size() > 2 && fm[3].face.size() > 2);
        MINI_CHECK(m.is_closed());
        MINI_CHECK(std::abs(sector.volume() - ref) / ref < 0.01);
    }

    MINI_TEST("BRep", "Face Polylines Box") {
        // using session_cpp::BRep;
        // using session_cpp::Polyline;
        // using session_cpp::Plane;
        // using session_cpp::Vector;

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
        // using session_cpp::BRep;

        const BRep b = BRep::create_cylinder(1.0, 4.0);

        MINI_CHECK(b.face_count() == 3);
        MINI_CHECK(b.face_polylines().size() == 2);
        MINI_CHECK(b.face_planes().size() == 2);
    }

    MINI_TEST("BRep", "Face Polylines Ignores Holes") {
        // using session_cpp::BRep;
        // using session_cpp::Polyline;
        // using session_cpp::Point;

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
        // using session_cpp::BRep;

        const BRep b = BRep::create_sphere(1.0);

        MINI_CHECK(b.face_polylines().empty());
        MINI_CHECK(b.face_planes().empty());
    }

    MINI_TEST("BRep", "Face Planes Reversed Flip") {
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::Plane;
        // using session_cpp::Vector;

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
        // using session_cpp::BRep;
        // using session_cpp::Plane;
        // using session_cpp::Point;
        // using session_cpp::Vector;

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
        // using session_cpp::BRep;
        // using session_cpp::Point;
        // using session_cpp::Plane;
        // using session_cpp::Vector;

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
        // using session_cpp::BRep;
        // using session_cpp::BRepOrientation;
        // using session_cpp::Point;
        // using session_cpp::Xform;
        // using session_cpp::Polyline;
        // using session_cpp::Plane;
        // using session_cpp::Vector;
        // using session_cpp::BRepRef;
        // using session_cpp::BRepEdge;

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

    MINI_TEST("BRep", "Is Visible Round Trip") {
        // using session_cpp::BRep;

        BRep hidden = BRep::create_box(1.0, 1.0, 1.0);

        MINI_CHECK(hidden.is_visible);

        hidden.is_visible = false;
        const BRep json = BRep::file_json_loads(hidden.file_json_dumps());
        const BRep proto = BRep::pb_loads(hidden.pb_dumps());

        MINI_CHECK(!json.is_visible);
        MINI_CHECK(!proto.is_visible);
    }

    MINI_TEST("BRep", "Is Locked Round Trip") {
        // using session_cpp::BRep;

        BRep locked = BRep::create_box(1.0, 1.0, 1.0);

        MINI_CHECK(!locked.is_locked);

        locked.is_locked = true;
        const BRep json = BRep::file_json_loads(locked.file_json_dumps());
        const BRep proto = BRep::pb_loads(locked.pb_dumps());

        MINI_CHECK(json.is_locked);
        MINI_CHECK(proto.is_locked);
    }

} // namespace session_cpp
