#include "bop.h"
#include "bop_builder_face.h"
#include "mini_test.h"
#include "xform.h"
#include <cmath>
#include <numbers>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

BRep moved(BRep b, double x, double y, double z) {
    b.transform(Xform::translation(x, y, z));
    return b;
}

}

MINI_TEST("BRep BOP", "Offset Boxes") {
    const BRep a = BRep::create_box(2, 2, 2), b = moved(BRep::create_box(2, 2, 2), 1, 1, 1);
    const std::array<bop::Operation, 5> operations = {bop::Operation::Intersection, bop::Operation::Union,
        bop::Operation::Difference, bop::Operation::ReverseDifference, bop::Operation::Xor};
    const std::array<double, 5> volumes = {1, 15, 7, 7, 14};
    const std::array<int, 5> faces = {6, 12, 9, 9, 18};
    for (int i = 0; i < 5; ++i) {
        bop::Report report;
        const BRep result = bop::apply(a, b, operations[i], 0.0, &report);
        MINI_CHECK(report.solid);
        MINI_CHECK(result.face_count() == faces[i]);
        MINI_CHECK(std::abs(result.volume_exact() - volumes[i]) < 1e-9);
    }
}

MINI_TEST("BRep BOP", "Cylinder Through Box") {
    const BRep box = BRep::create_box(2, 2, 2), drill = moved(BRep::create_cylinder(0.5, 4), 0.2, 0, -2);
    const BRep result = bop::apply(box, drill, bop::Operation::Difference);
    MINI_CHECK(result.is_solid());
    MINI_CHECK(result.face_count() == 7);
    MINI_CHECK(std::abs(result.volume_exact() - (8 - 2 * std::numbers::pi * 0.25)) < 1e-7);
}

MINI_TEST("BRep BOP", "Same Domain Faces") {
    const BRep a = BRep::create_box(2, 2, 2), b = moved(BRep::create_box(2, 2, 2), 1, 0, 0);
    const BRep sum = bop::apply(a, b, bop::Operation::Union), common = bop::apply(a, b, bop::Operation::Intersection);
    MINI_CHECK(sum.is_solid() && common.is_solid());
    MINI_CHECK(std::abs(sum.volume_exact() - 12) < 1e-9);
    MINI_CHECK(std::abs(common.volume_exact() - 4) < 1e-9);
    MINI_CHECK(common.face_count() == 6);
}

MINI_TEST("BRep BOP", "Loose Contact") {
    BRep a = BRep::create_box(2, 2, 2), b = moved(BRep::create_box(2, 2, 2), 2 + 4e-6, 0.5, 0.5);
    for (BRepVertex& v : b.m_vertices)
        v.tolerance = 1e-5;
    for (BRepEdge& e : b.m_edges)
        e.tolerance = 1e-5;
    for (BRepFace& f : b.m_faces)
        f.tolerance = 1e-5;
    bop::Report report;
    const BRep sum = bop::apply(a, b, bop::Operation::Union, 0.0, &report);
    MINI_CHECK(report.solid);
    MINI_CHECK(sum.solid_count() == 1);
    MINI_CHECK(std::abs(sum.volume_exact() - 16) < 1e-4);
}

MINI_TEST("BRep BOP", "Rotated Copies") {
    for (const BRep& a : {BRep::create_cylinder(1, 2), BRep::create_block_with_hole(2, 2, 1, 0.4)}) {
        const BRep b = a.transformed(Xform::rotation(Vector(0, 0, 1), 5, true));
        bop::Report first, second;
        const BRep ab = bop::apply(a, b, bop::Operation::Difference, 0.0, &first), ba = bop::apply(a, b, bop::Operation::ReverseDifference, 0.0, &second);
        const BRep sum = bop::apply(a, b, bop::Operation::Union);
        MINI_CHECK(sum.is_solid());
        MINI_CHECK(ab.face_count() == 0 || (first.solid && second.solid));
        MINI_CHECK(ab.face_count() == 0 || std::abs(ab.volume_exact() - ba.volume_exact()) < 1e-6);
    }
}

MINI_TEST("BRep BOP", "Coaxial Caps") {
    const BRep a = BRep::create_box(40, 40, 20), b = moved(BRep::create_cylinder(5, 20), 0, 0, -10);
    bop::Report report;
    const BRep rest = bop::apply(a, b, bop::Operation::Xor, 1e-7, &report);
    MINI_CHECK(report.solid && report.failure.empty());
    MINI_CHECK(rest.face_count() == 7 && rest.edge_count() == 15 && rest.vertex_count() == 10);
    MINI_CHECK(std::abs(rest.volume_exact() - (32000 - 500 * 3.14159265358979323846)) < 1e-6);
}

MINI_TEST("BRep BOP", "Band Without Seam") {
    // A face around a periodic surface with no seam edge, cut by a closed section that has no seam start either.
    const auto line = [](double u0, double v0, double u1, double v1) { return bop::owned(NurbsCurve::create(false, 1, {Point(u0, v0, 0), Point(u1, v1, 0)})); };
    bop::Region region;
    region.period = {4.0, 0.0};
    region.add_loop({Point(0, 0, 0), Point(2, 0, 0), Point(4, 0, 0)}, 0);
    region.add_loop({Point(4, 1, 0), Point(2, 1, 0), Point(0, 1, 0)}, 0);
    MINI_CHECK(region.state(Point(1, 0.5, 0)) == 1 && region.state(Point(5, 0.5, 0)) == 1 && region.state(Point(1, 2, 0)) == -1 && region.state(Point(3, 0, 0)) == 0);
    Point inside;
    MINI_CHECK(region.interior(inside) && region.state(inside) == 1);
    std::vector<bop::EdgeUse> uses{{0, 0, 0, line(0, 0, 4, 0), false, false, 0.0}, {1, 1, 1, line(4, 1, 0, 1), false, false, 0.0},
        {2, 2, 2, line(1, 0.5, 5, 0.5), false, true, 0.0}, {2, 2, 2, line(5, 0.5, 1, 0.5), true, true, 0.0}};
    std::vector<std::string> log;
    const auto faces = bop::build_faces(uses, region, log);
    MINI_CHECK(log.empty() && faces.size() == 2 && faces[0].size() == 2 && faces[1].size() == 2);
    MINI_CHECK(uses[2].pcurve.shift[0] == 0.0 && uses[2].pcurve.at(1.0).distance(Point(5, 0.5, 0)) < 1e-12);
}

MINI_TEST("BRep BOP", "Shared Curves") {
    // A split edge is its operand's curve with a range, and a result with ranges is an operand like any other.
    const BRep a = BRep::create_box(2, 2, 2), b = moved(BRep::create_box(2, 2, 2), 1, 1, 1);
    const BRep cut = bop::apply(a, b, bop::Operation::Difference);
    MINI_CHECK(cut.has_ranges() && cut.is_valid() && cut.is_solid());
    MINI_CHECK(std::abs(cut.volume_exact() - 7) < 1e-9);
    bop::Report report;
    const BRep back = bop::apply(cut, b, bop::Operation::Union, 0.0, &report);
    MINI_CHECK(report.solid && report.failure.empty());
    MINI_CHECK(std::abs(back.volume_exact() - 15) < 1e-9);
    const BRep twice = bop::apply(cut, moved(BRep::create_box(2, 2, 2), -1, -1, -1), bop::Operation::Difference, 0.0, &report);
    MINI_CHECK(report.solid && std::abs(twice.volume_exact() - 6) < 1e-9);
}

MINI_TEST("BRep BOP", "Analytic Result") {
    // Analytic operands are cut in their true parameters and the result stays analytic: its surfaces as they were, its sections the lines and circles they are, read as NURBS by the consumers still on them.
    const BRep box = BRep::create_box(2, 2, 2), drill = moved(BRep::create_cylinder(0.5, 4), 0.2, 0, -2);
    bop::Report report;
    const BRep sum = bop::apply(box, drill, bop::Operation::Union, 0.0, &report);
    bool analytic = report.solid && report.failure.empty();
    for (const Surface& s : sum.m_surfaces)
        analytic = analytic && s.kind() != Surface::Kind::Nurbs;
    for (const Curve& c : sum.m_curves_3d)
        analytic = analytic && c.kind() != Curve::Kind::Nurbs;
    for (const Curve2d& c : sum.m_curves_2d)
        analytic = analytic && c.kind() != Curve::Kind::Nurbs;
    MINI_CHECK(analytic && sum.is_valid() && sum.as_nurbs().is_solid());
    MINI_CHECK(sum.face_count() == 10 && sum.edge_count() == 18 && sum.vertex_count() == 12);
    MINI_CHECK(std::abs(sum.volume_exact() - (8 + 0.5 * std::numbers::pi)) < 1e-9);
}

MINI_TEST("BRep BOP", "Cylinder Without Seam") {
    // A cylinder whose side face is bounded by its two circles alone: a cut across the angle origin and one around the axis come out as with the seam edge.
    const BRep seamed = BRep::create_cylinder(0.5, 2);
    BRep bare = seamed;
    for (BRepFace& f : bare.m_faces)
        if (bare.m_surfaces[f.surface_index].kind() == Surface::Kind::Cylinder) {
            std::vector<BRepRef> wires;
            for (const BRepRef& use : std::vector<BRepRef>(bare.m_wires[f.wires[0].index].edges))
                if (bare.m_edges[use.index].start_vertex == bare.m_edges[use.index].end_vertex)
                    wires.push_back({bare.add_wire({use}), BRepOrientation::Forward});
            f.wires = wires;
        }
    for (const BRep& cutter : {moved(BRep::create_box(0.6, 0.6, 0.6), 0.5, 0.1, 1), moved(BRep::create_box(3, 3, 0.5), 0.1, 0.1, 1), moved(BRep::create_sphere(0.4), 0.45, 0.1, 1)})
        for (bop::Operation operation : {bop::Operation::Intersection, bop::Operation::Union, bop::Operation::Difference, bop::Operation::ReverseDifference}) {
            bop::Report report;
            const BRep result = bop::apply(bare, cutter, operation, 0.0, &report);
            MINI_CHECK(report.solid && report.log.empty() && result.is_valid());
            MINI_CHECK(std::abs(result.volume_exact() - bop::apply(seamed, cutter, operation).volume_exact()) < 1e-9);
        }
}

MINI_TEST("BRep BOP", "Torus Sphere") {
    // A walked section crosses the seams of both faces a short way apart: the piece between them is shorter than a step of the walk.
    const BRep torus = BRep::create_torus(1.1646083967015148, 0.27873076502000915);
    const BRep sphere = BRep::create_sphere(0.82153776235878473).transformed(Xform::translation(0.10451730906497687, 0.19312218977138401, 0.16540525769814848)
        * Xform::rotation(Vector(-0.66029139049351215, -0.23508306173607707, 0.71192960673943162), -0.3838125541806221));
    bop::Report report;
    const BRep common = bop::apply(torus, sphere, bop::Operation::Intersection, 0.0, &report);
    MINI_CHECK(report.solid && report.failure.empty() && common.is_valid());
    MINI_CHECK(common.face_count() == 4 && common.edge_count() == 6 && common.vertex_count() == 4);
    MINI_CHECK(std::abs(common.volume_exact() - 0.05150034080275) < 1e-7);
}

MINI_TEST("BRep BOP", "Boxes Sharing An Edge") {
    // BRep_Tool::IsClosed: the shell of two boxes that touch along an edge runs it four times.
    const BRep a = BRep::create_box(1, 1, 1), b = moved(BRep::create_box(1, 1, 1), 1, 1, 0);
    bop::Report report;
    const BRep both = bop::apply(a, b, bop::Operation::Union, 0.0, &report);
    MINI_CHECK(report.solid && report.failure.empty() && both.is_solid());
    MINI_CHECK(both.face_count() == 12 && both.edge_count() == 23 && both.vertex_count() == 14);
    MINI_CHECK(std::abs(both.volume_exact() - 2.0) < 1e-9);
}

MINI_TEST("BRep BOP", "Fused Vertices") {
    // BRepLib::BoundingVertex of two: the larger ball where it holds the other, else the smallest ball around both.
    bop::DS ds;
    const int inside = ds.merge({ds.add_vertex(Point(0, 0, 0), 1.0), ds.add_vertex(Point(0.5, 0, 0), 0.1)});
    MINI_CHECK(ds.vertices[inside].point.distance(Point(0, 0, 0)) < 1e-12 && std::abs(ds.vertices[inside].tolerance - 1.0) < 1e-12);
    const int across = ds.merge({ds.add_vertex(Point(0, 0, 0), 0.3), ds.add_vertex(Point(1, 0, 0), 0.1)});
    MINI_CHECK(ds.vertices[across].point.distance(Point(0.4, 0, 0)) < 1e-12 && std::abs(ds.vertices[across].tolerance - 0.7) < 1e-12);
}

MINI_TEST("BRep BOP", "Arc Through The End Of An Edge It Touches") {
    // A stadium prism against its copy turned 30 degrees about an axis 5e-8 off its own: the copy's arc passes the vertex where an arc of the first meets its straight edge and crosses that edge 3e-4 on, inside the common part that holds the vertex.
    const BRep a = bop::apply(moved(BRep::create_box(2, 2, 1), 1, 0, 0.5), BRep::create_cylinder(1, 1), bop::Operation::Union);
    const BRep b = a.transformed(Xform::translation(5e-8, 0, 0) * Xform::rotation_z(30, true) * Xform::translation(-5e-8, 0, 0));
    bop::Report joined, shared;
    const BRep both = bop::apply(a, b, bop::Operation::Union, 0.0, &joined), common = bop::apply(a, b, bop::Operation::Intersection, 0.0, &shared);
    MINI_CHECK(joined.solid && joined.failure.empty() && shared.solid && shared.failure.empty());
    MINI_CHECK(both.face_count() == 27 && common.face_count() == 17);
    MINI_CHECK(std::abs(both.volume_exact() + common.volume_exact() - 2.0 * a.volume_exact()) < 1e-6);
}
