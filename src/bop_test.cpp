#include "bop.h"
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
