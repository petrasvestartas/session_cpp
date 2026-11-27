#include "bop.h"
#include "brep_check.h"
#include "mini_test.h"
#include "xform.h"
#include <numbers>

using namespace session_cpp;
using namespace session_cpp::mini_test;
using brep_check::Status;

namespace {

BRep moved(BRep b, double x, double y, double z) {
    b.transform(Xform::translation(x, y, z));
    return b;
}

bool flags(const BRep& b, Status status) {
    return brep_check::check(b).has(status);
}

int degenerated(const BRep& b) {
    for (int i = 0; i < b.edge_count(); ++i)
        if (b.m_edges[i].degenerated)
            return i;
    return -1;
}

int seam(const BRep& b) {
    for (int i = 0; i < b.edge_count(); ++i)
        if (!b.m_edges[i].pcurves.empty() && b.m_edges[i].pcurves[0].curve_2d_index_2 >= 0)
            return i;
    return -1;
}

BRep square(const std::vector<Point>& outer, const std::vector<Point>& hole = {}) {
    std::vector<Point> loop = outer;
    loop.push_back(outer[0]);
    std::vector<std::vector<Polyline>> holes(1);
    if (!hole.empty()) {
        std::vector<Point> inner = hole;
        inner.push_back(hole[0]);
        holes[0].push_back(Polyline(inner));
    }
    return BRep::from_polylines({Polyline(loop)}, holes);
}

}

MINI_TEST("BRep Check", "Primitives Valid") {
    const std::vector<BRep> primitives = {BRep::create_box(2, 3, 4), BRep::create_cylinder(1, 2), BRep::create_sphere(1.5), BRep::create_cone(1, 2),
        BRep::create_frustum(2, 1, 3), BRep::create_pyramid(2, 1), BRep::create_torus(3, 1), BRep::create_block_with_hole(4, 4, 2, 1)};
    for (const BRep& b : primitives) {
        const brep_check::Report report = brep_check::check(b);
        MINI_CHECK(report.valid());
        MINI_CHECK(report.summary() == "valid");
        MINI_CHECK(b.is_valid() && b.is_solid());
    }
}

MINI_TEST("BRep Check", "Booleans Valid") {
    const BRep box = BRep::create_box(2, 2, 2);
    const std::vector<BRep> results = {bop::apply(box, moved(box, 1, 1, 1), bop::Operation::Union), bop::apply(box, moved(BRep::create_cylinder(0.5, 4), 0.2, 0, -2), bop::Operation::Difference),
        bop::apply(box, BRep::create_sphere(1.2), bop::Operation::Intersection)};
    for (const BRep& b : results)
        MINI_CHECK(brep_check::check(b).valid());
}

MINI_TEST("BRep Check", "Bad Reference") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_edges[0].start_vertex = 99;
    MINI_CHECK(flags(b, Status::BadReference) && !b.is_valid());
}

MINI_TEST("BRep Check", "Vertex Off Edge") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_vertices[0].point = b.m_vertices[0].point + Vector(0.1, 0, 0);
    const brep_check::Report report = brep_check::check(b);
    MINI_CHECK(report.has(Status::VertexOffEdge) && !report.has(Status::EdgeOffPCurve) && b.is_valid());
}

MINI_TEST("BRep Check", "Edge Off PCurve") {
    BRep b = BRep::create_cylinder(1, 2);
    const int c = b.m_edges[seam(b)].curve_3d_index;
    b.m_curves_3d[c] = b.m_curves_3d[c].transformed(Xform::translation(0.01, 0, 0));
    MINI_CHECK(flags(b, Status::EdgeOffPCurve));
}

MINI_TEST("BRep Check", "Same Parameter") {
    BRep b = BRep::create_box(2, 2, 2);
    const Point a = b.m_vertices[b.m_edges[0].start_vertex].point, z = b.m_vertices[b.m_edges[0].end_vertex].point;
    b.m_edges[0].curve_3d_index = b.add_curve_3d(Curve(NurbsCurve::create(false, 2, {a, a + (z - a) * 0.9, z})));
    b.m_edges[0].range = {0.0, 0.0};
    const brep_check::Report report = brep_check::check(b);
    MINI_CHECK(report.has(Status::SameParameter) && !report.has(Status::EdgeOffPCurve) && b.is_valid());
}

MINI_TEST("BRep Check", "Bad Range") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_edges[0].range = {1.0, 0.5};
    MINI_CHECK(flags(b, Status::BadRange));
}

MINI_TEST("BRep Check", "PCurve Self Intersect") {
    BRep b = square({Point(0, 0, 0), Point(4, 0, 0), Point(4, 4, 0), Point(0, 4, 0)});
    BRepCurveOnSurface& use = b.m_edges[0].pcurves[0];
    const Point a = b.m_curves_2d[use.curve_2d_index].point_at(b.pcurve_range(use).first), z = b.m_curves_2d[use.curve_2d_index].point_at(b.pcurve_range(use).second);
    const Vector d = z - a;
    const std::vector<Point> loop = {a, a + d * 0.7 + Vector(-d[1], d[0], 0) * 0.2, a + d * 0.7 + Vector(d[1], -d[0], 0) * 0.2, a + d * 0.3 + Vector(d[1], -d[0], 0) * 0.2, a + d * 0.3 + Vector(-d[1], d[0], 0) * 0.2, z};
    use.curve_2d_index = b.add_curve_2d(Curve2d(NurbsCurve::create(false, 1, loop)));
    use.range = {0.0, 0.0};
    MINI_CHECK(flags(b, Status::PCurveSelfIntersect));
}

MINI_TEST("BRep Check", "Loop Self Intersect") {
    const BRep b = square({Point(0, 0, 0), Point(4, 4, 0), Point(4, 0, 0), Point(0, 2, 0)});
    MINI_CHECK(flags(b, Status::LoopSelfIntersect));
}

MINI_TEST("BRep Check", "Loops Intersect") {
    const BRep b = square({Point(0, 0, 0), Point(4, 0, 0), Point(4, 4, 0), Point(0, 4, 0)}, {Point(3, 1, 0), Point(5, 1, 0), Point(5, 3, 0), Point(3, 3, 0)});
    MINI_CHECK(flags(b, Status::LoopsIntersect));
}

MINI_TEST("BRep Check", "Wire Open") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_wires[0].edges.pop_back();
    MINI_CHECK(flags(b, Status::WireOpen) && !b.is_valid());
}

MINI_TEST("BRep Check", "Wire Open UV") {
    BRep b = BRep::create_cylinder(1, 2);
    const int s = seam(b), side = b.m_edges[s].pcurves[0].surface_index;
    for (BRepEdge& e : b.m_edges)
        if (&e != &b.m_edges[s] && !e.degenerated)
            for (BRepCurveOnSurface& use : e.pcurves)
                if (use.surface_index == side) {
                    use.curve_2d_index = b.add_curve_2d(b.m_curves_2d[use.curve_2d_index].shifted(2.0 * std::numbers::pi, 0.0));
                    const brep_check::Report report = brep_check::check(b);
                    MINI_CHECK(report.has(Status::WireOpenUV) && !report.has(Status::EdgeOffPCurve));
                    MINI_CHECK(report.summary().find("only modulo a period") != std::string::npos);
                    return;
                }
}

MINI_TEST("BRep Check", "Loop Orientation") {
    BRep b = square({Point(0, 0, 0), Point(4, 0, 0), Point(4, 4, 0), Point(0, 4, 0)});
    b.m_faces[0].wires[0].orientation = brep_reverse(b.m_faces[0].wires[0].orientation);
    MINI_CHECK(flags(b, Status::LoopOrientation) && b.is_valid());
}

MINI_TEST("BRep Check", "Seam Boundary") {
    BRep b = BRep::create_cylinder(1, 2);
    b.m_edges[seam(b)].pcurves[0].curve_2d_index_2 = -1;
    MINI_CHECK(flags(b, Status::SeamBoundary));
}

MINI_TEST("BRep Check", "Tolerance Out Of Proportion") {
    BRep b = BRep::create_sphere(10);
    b.m_edges[degenerated(b)].tolerance = 1.37;
    MINI_CHECK(flags(b, Status::ToleranceOutOfProportion));
    BRep box = BRep::create_box(2, 2, 2);
    box.m_edges[0].tolerance = 1e-4;
    brep_check::Options options;
    options.reference_tolerance = 1e-7;
    MINI_CHECK(!flags(box, Status::ToleranceOutOfProportion) && brep_check::check(box, options).has(Status::ToleranceOutOfProportion));
    // BRepCheck passes a vertex whose ball covers its edges: OCCT's own results carry them.
    box.m_vertices[0].tolerance = 1.5;
    MINI_CHECK(!flags(box, Status::ToleranceOutOfProportion));
}

MINI_TEST("BRep Check", "Edge Uses") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_shells[0].faces.push_back(b.m_shells[0].faces[0]);
    MINI_CHECK(flags(b, Status::EdgeUses) && !b.is_solid());
}

MINI_TEST("BRep Check", "Shell Open") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_shells[0].faces.pop_back();
    MINI_CHECK(flags(b, Status::ShellOpen) && flags(b, Status::EdgeUses) && b.is_valid() && !b.is_solid());
}

MINI_TEST("BRep Check", "Shell Not Orientable") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_shells[0].faces[0].orientation = brep_reverse(b.m_shells[0].faces[0].orientation);
    MINI_CHECK(flags(b, Status::ShellNotOrientable) && !b.is_solid());
}

MINI_TEST("BRep Check", "Solid Inside Out") {
    BRep b = BRep::create_cylinder(1, 2);
    b.m_solids[0].shells[0].orientation = brep_reverse(b.m_solids[0].shells[0].orientation);
    const brep_check::Report report = brep_check::check(b);
    MINI_CHECK(report.has(Status::SolidInsideOut) && report.issues.size() == 1 && b.is_solid());
}

MINI_TEST("BRep Check", "Degenerated Without PCurve") {
    BRep b = BRep::create_sphere(1);
    b.m_edges[degenerated(b)].pcurves.clear();
    MINI_CHECK(flags(b, Status::DegeneratedWithoutPCurve));
}

MINI_TEST("BRep Check", "Edge Without PCurve") {
    BRep b = BRep::create_box(2, 2, 2);
    b.m_edges[0].pcurves.resize(1);
    MINI_CHECK(flags(b, Status::EdgeWithoutPCurve));
}
