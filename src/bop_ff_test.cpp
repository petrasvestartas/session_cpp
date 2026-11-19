#include "bop_ff.h"
#include "closest.h"
#include "mini_test.h"
#include "xform.h"
#include <cmath>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

int face_near(const BRep& brep, const Point& p) {
    int best = -1;
    double nearest = 1e300;
    for (int f = 0; f < brep.face_count(); ++f) {
        const double d = std::get<2>(Closest::surface_point(brep.m_surfaces[brep.m_faces[f].surface_index].nurbs(), p));
        if (d < nearest)
            nearest = d, best = f;
    }
    return best;
}

// The largest distance from each curve to both faces' surfaces along its pcurves, over its own tolerance.
double worst_fit(const BRep& a, int fa, const BRep& b, int fb, const bop_ff::Result& r) {
    const NurbsSurface& sa = a.m_surfaces[a.m_faces[fa].surface_index].nurbs();
    const NurbsSurface& sb = b.m_surfaces[b.m_faces[fb].surface_index].nurbs();
    double worst = 0.0;
    for (const bop_ff::Curve& c : r.curves)
        for (int i = 0; i <= 50; ++i) {
            const double t = c.curve.domain_start() + (c.curve.domain_end() - c.curve.domain_start()) * i / 50;
            const Point p = c.curve.point_at(t), ua = c.pcurve_a.point_at(t), ub = c.pcurve_b.point_at(t);
            worst = std::max({worst, p.distance(sa.point_at(ua[0], ua[1])) / c.tolerance, p.distance(sb.point_at(ub[0], ub[1])) / c.tolerance});
        }
    return worst;
}

double nearest_on_curves(const bop_ff::Result& r, const Point& p) {
    double best = 1e300;
    for (const bop_ff::Curve& c : r.curves)
        best = std::min(best, Closest::curve_point(c.curve, p).second);
    return best;
}

double length(const bop_ff::Result& r) {
    double total = 0.0;
    for (const bop_ff::Curve& c : r.curves)
        for (int i = 1; i <= 2000; ++i) {
            const double t0 = c.curve.domain_start(), t1 = c.curve.domain_end();
            total += c.curve.point_at(t0 + (t1 - t0) * (i - 1) / 2000).distance(c.curve.point_at(t0 + (t1 - t0) * i / 2000));
        }
    return total;
}

BRep patch(const NurbsSurface& s) {
    BRep brep;
    const int surface = brep.add_surface(s);
    const auto [u0, u1] = s.domain(0);
    const auto [v0, v1] = s.domain(1);
    const std::array<std::array<double, 2>, 4> corners = {{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}};
    std::vector<int> vertices;
    for (const auto& c : corners)
        vertices.push_back(brep.add_vertex(s.point_at(c[0], c[1])));
    std::vector<BRepRef> edges;
    for (int i = 0; i < 4; ++i) {
        const auto& p = corners[i];
        const auto& q = corners[(i + 1) % 4];
        const int dir = p[1] == q[1] ? 0 : 1;
        NurbsCurve world = s.iso_curve(dir, dir ? p[0] : p[1]);
        const bool backward = (dir ? q[1] < p[1] : q[0] < p[0]);
        if (backward)
            world.reverse();
        const int edge = brep.add_edge(brep.add_curve_3d(world), vertices[i], vertices[(i + 1) % 4]);
        NurbsCurve line = NurbsCurve::create(false, 1, {Point(p[0], p[1], 0), Point(q[0], q[1], 0)});
        line.set_domain(world.domain_start(), world.domain_end());
        brep.add_pcurve(edge, surface, brep.add_curve_2d(line));
        edges.push_back({edge, BRepOrientation::Forward});
    }
    brep.add_face(surface, {{brep.add_wire(edges), BRepOrientation::Forward}});
    return brep;
}

// A face over the whole of a surface given by its degrees, nurbsknots and control points x, y, z, w.
BRep face(int degree_u, int degree_v, const std::vector<double>& knots_u, const std::vector<double>& knots_v, const std::vector<std::array<double, 4>>& cvs) {
    NurbsSurface s;
    const int count_u = int(knots_u.size()) - degree_u + 1, count_v = int(knots_v.size()) - degree_v + 1;
    s.create_raw(3, true, degree_u + 1, degree_v + 1, count_u, count_v);
    for (size_t i = 0; i < knots_u.size(); ++i)
        s.set_nurbsknot(0, int(i), knots_u[i]);
    for (size_t i = 0; i < knots_v.size(); ++i)
        s.set_nurbsknot(1, int(i), knots_v[i]);
    for (int i = 0; i < count_u; ++i)
        for (int j = 0; j < count_v; ++j) {
            const std::array<double, 4>& q = cvs[i * count_v + j];
            s.set_cv_4d(i, j, q[0] * q[3], q[1] * q[3], q[2] * q[3], q[3]);
        }
    return patch(s);
}

NurbsSurface bump() {
    std::vector<Point> points;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            points.push_back(Point(i, j, 0.3 * std::sin(i + 0.7 * j)));
    return NurbsSurface::create(false, false, 3, 3, 5, 5, points);
}

}

MINI_TEST("Bop FF", "Plane Plane Coincident Offset") {
    const BRep a = BRep::create_box(2, 2, 2);
    const BRep b = BRep::create_box(2, 2, 2).transformed(Xform::translation(1, 0.5, 2 + 1e-6));
    const int fa = face_near(a, Point(0, 0, 1)), fb = face_near(b, Point(1, 0.5, 1 + 1e-6));
    const bop_ff::Result loose = bop_ff::intersect(a, fa, b, fb, 2e-6);
    const bop_ff::Result tight = bop_ff::intersect(a, fa, b, fb, 0.0);
    MINI_CHECK(loose.same_domain && loose.curves.empty() && loose.points.empty());
    MINI_CHECK(!tight.same_domain && tight.curves.empty() && tight.points.empty());
}

MINI_TEST("Bop FF", "Cylinder Cylinder Equal Radius 5 Degrees") {
    const BRep a = BRep::create_cylinder(1, 4).transformed(Xform::translation(0, 0, -2));
    const BRep b = a.transformed(Xform::rotation_x(5, true));
    const int fa = face_near(a, Point(0, 1, 0.5)), fb = face_near(b, Point(0, 1, 0.5));
    const bop_ff::Result r = bop_ff::intersect(a, fa, b, fb, 0.0);
    MINI_CHECK(!r.same_domain && r.curves.size() >= 3);
    MINI_CHECK(worst_fit(a, fa, b, fb, r) <= 1.0);
    MINI_CHECK(nearest_on_curves(r, Point(1, 0, 0)) < 1e-6 && nearest_on_curves(r, Point(-1, 0, 0)) < 1e-6);
    MINI_CHECK(nearest_on_curves(r, Point(0, 1, std::tan(2.5 * M_PI / 180))) < 1e-6);
}

MINI_TEST("Bop FF", "Sphere Sphere Touching") {
    const BRep a = BRep::create_sphere(1);
    const BRep b = a.transformed(Xform::translation(2, 0, 0));
    const bop_ff::Result r = bop_ff::intersect(a, 0, b, 0, 0.0);
    MINI_CHECK(!r.same_domain && r.curves.empty() && r.points.size() == 1);
    MINI_CHECK(r.points.size() == 1 && r.points[0].point.distance(Point(1, 0, 0)) < 1e-6 && r.points[0].tolerance < 1e-6);
}

MINI_TEST("Bop FF", "Cone Plane Through Apex") {
    const BRep cone = BRep::create_cone(1, 2);
    const BRep box = BRep::create_box(4, 4, 4).transformed(Xform::translation(2, 0, 1));
    const int fa = face_near(cone, Point(0.5, 0, 1)), fb = face_near(box, Point(0, 0, 1));
    const bop_ff::Result r = bop_ff::intersect(cone, fa, box, fb, 0.0);
    MINI_CHECK(!r.same_domain && !r.curves.empty());
    MINI_CHECK(worst_fit(cone, fa, box, fb, r) <= 1.0);
    MINI_CHECK(std::abs(length(r) - 2 * std::sqrt(5.0)) < 1e-3);
    MINI_CHECK(nearest_on_curves(r, Point(0, 0, 2)) < 1e-6 && nearest_on_curves(r, Point(0, 1, 0)) < 1e-6);
}

MINI_TEST("Bop FF", "Freeform Patches Coincident") {
    const BRep a = patch(bump());
    NurbsSurface flipped = bump();
    flipped.reverse(0);
    NurbsSurface part = bump();
    part.trim(0, {0.3, 1.4});
    part.trim(1, {0.2, 1.1});
    NurbsSurface lifted = bump();
    lifted.transform(Xform::translation(0, 0, 1e-3));
    MINI_CHECK(bop_ff::intersect(a, 0, patch(flipped), 0, 0.0).same_domain);
    MINI_CHECK(bop_ff::intersect(a, 0, patch(part), 0, 0.0).same_domain);
    const bop_ff::Result apart = bop_ff::intersect(a, 0, patch(lifted), 0, 0.0);
    MINI_CHECK(!apart.same_domain && apart.curves.empty() && apart.points.empty());
    NurbsSurface left = bump();
    NurbsSurface right = bump();
    left.trim(0, {0.0, 1.05});
    right.trim(0, {1.0, 2.0});
    MINI_CHECK(bop_ff::intersect(patch(left), 0, patch(right), 0, 0.0).same_domain);
    right.transform(Xform::translation(0, 0, 1e-5));
    const bop_ff::Result strip = bop_ff::intersect(patch(left), 0, patch(right), 0, 0.0);
    MINI_CHECK(!strip.same_domain && strip.curves.empty() && strip.points.empty());
    MINI_CHECK(bop_ff::intersect(patch(left), 0, patch(right), 0, 4e-5).same_domain);
}

MINI_TEST("Bop FF", "Face Against Rotated Copy") {
    const BRep box = BRep::create_box(2, 2, 2).as_nurbs();
    const BRep turned = box.transformed(Xform::rotation_z(1e-3));
    MINI_CHECK(bop_ff::intersect(box, face_near(box, Point(0, 0, 1)), turned, face_near(turned, Point(0, 0, 1)), 0.0).same_domain);
    const int fa = face_near(box, Point(1, 0, 0)), fb = face_near(turned, Point(1, 0, 0));
    const bop_ff::Result side = bop_ff::intersect(box, fa, turned, fb, 0.0);
    MINI_CHECK(!side.same_domain && side.curves.size() == 1 && worst_fit(box, fa, turned, fb, side) <= 1.0);
    MINI_CHECK(std::abs(length(side) - 2.0) < 1e-6 && nearest_on_curves(side, Point(1, std::tan(5e-4), 0)) < 1e-9);
    const BRep cylinder = BRep::create_cylinder(1, 4).as_nurbs().transformed(Xform::translation(0, 0, -2));
    const int fc = face_near(cylinder, Point(0, 1, 0.5));
    MINI_CHECK(bop_ff::intersect(cylinder, fc, cylinder.transformed(Xform::rotation_z(1e-3)), fc, 0.0).same_domain);
    const BRep tilted = cylinder.transformed(Xform::rotation_x(1e-3));
    const bop_ff::Result near = bop_ff::intersect(cylinder, fc, tilted, fc, 0.0);
    MINI_CHECK(!near.same_domain && !near.curves.empty() && worst_fit(cylinder, fc, tilted, fc, near) <= 1.0);
    MINI_CHECK(nearest_on_curves(near, Point(0, 1, 5e-4)) < 1e-6);
}

// Real face pairs of the STEP corpus that the certified pipeline refused, each face as its surface over the box of its pcurves.

MINI_TEST("Bop FF", "Corpus Hull Pair Tangent Cylinders") {
    // zip_ea0000006997revit against its copy turned 5 degrees, faces 49 and 48: equal cylinders touching at two points, "could not resolve an overlapping rational hull pair".
    const BRep a = face(2, 1, {0, 0, 1, 1, 2, 2}, {-19.94870090963, 90.66829909034}, {{19.94870090963, 198.12, 85.9155, 1}, {-90.66829909034, 198.12, 85.9155, 1}, {19.94870090963, 284.0355, 85.9155, 0.707106781186548}, {-90.66829909034, 284.0355, 85.9155, 0.707106781186548}, {19.94870090963, 284.0355, 5.26080710360722e-15, 1}, {-90.66829909034, 284.0355, 5.26080710360722e-15, 1}, {19.94870090963, 284.0355, -85.9155, 0.707106781186548}, {-90.66829909034, 284.0355, -85.9155, 0.707106781186548}, {19.94870090963, 198.12, -85.9155, 1}, {-90.66829909034, 198.12, -85.9155, 1}});
    const BRep b = face(2, 1, {0, 0, 1, 1, 2, 2}, {-19.94870090963, 90.66829909034}, {{18.7777432450378, 198.269339816959, -85.9155, 1}, {-91.418325673747, 188.628433021444, -85.9155, 1}, {26.2657724610742, 112.680774233057, -85.9155, 0.707106781186548}, {-83.9302964577105, 103.039867437542, -85.9155, 0.707106781186548}, {26.2657724610742, 112.680774233057, 5.26080710360722e-15, 1}, {-83.9302964577105, 103.039867437542, 5.26080710360722e-15, 1}, {26.2657724610742, 112.680774233057, 85.9155, 0.707106781186548}, {-83.9302964577105, 103.039867437542, 85.9155, 0.707106781186548}, {18.7777432450378, 198.269339816959, 85.9155, 1}, {-91.418325673747, 188.628433021444, 85.9155, 1}});
    const bop_ff::Result r = bop_ff::intersect(a, 0, b, 0, 0.0);
    MINI_CHECK(!r.same_domain && r.curves.size() == 2 && r.points.empty());
    MINI_CHECK(worst_fit(a, 0, b, 0, r) <= 1.0 && std::abs(length(r) - 3.42) < 0.02);
    MINI_CHECK(nearest_on_curves(r, Point(17.0709946, 198.12, 85.9155)) < 1e-4 && nearest_on_curves(r, Point(17.0709946, 198.12, -85.9155)) < 1e-4);
}

MINI_TEST("Bop FF", "Corpus Fitted Section Apart") {
    // The same part, faces 11 and 20: cylinders that pass each other, "fitted section does not cover its certified starting seed".
    const BRep a = face(2, 1, {0, 0, 1, 1}, {485.9281818448, 514.7571818448}, {{-87.757, 349.885, 195.58, 1}, {-87.757, 349.885, 166.751, 1}, {-87.757, 368.935, 195.58, 0.707106781186548}, {-87.757, 368.935, 166.751, 0.707106781186548}, {-68.707, 368.935, 195.58, 1}, {-68.707, 368.935, 166.751, 1}});
    const BRep b = face(2, 1, {0, 0, 1, 1}, {19.92075494779, 91.04075494352}, {{-13.3020177426756, 363.743081967679, 198.755, 1}, {-84.1513846667068, 357.544565543838, 198.755, 1}, {-13.5787372258995, 366.906000134121, 198.755, 0.707106781186548}, {-84.4281041499306, 360.707483710279, 198.755, 0.707106781186548}, {-13.5787372258995, 366.906000134121, 195.58, 1}, {-84.4281041499306, 360.707483710279, 195.58, 1}});
    const bop_ff::Result r = bop_ff::intersect(a, 0, b, 0, 0.0);
    MINI_CHECK(!r.same_domain && r.curves.empty() && r.points.empty());
}

MINI_TEST("Bop FF", "Corpus Plane Section Refinement") {
    // 5c6512ae against its copy turned 5 degrees, faces 19 and 21: a cone and a plane, "plane section refinement did not converge".
    const BRep a = face(2, 1, {0.124902495638994, 0.124902495638994, 1, 1, 2, 2, 2.87509750436092, 2.87509750436092}, {-2.98945604133257e-13, 0.707106781187944}, {{-27.78750262642, -0.409090909090579, 0.500000000000211, 0.935972451502903}, {-27.2958365434012, -0.499999999999782, -1.21136434216851e-12, 0.935972451502903}, {-28.1278868377398, -2.24999999999979, 0.500000000000211, 0.743689875172086}, {-27.71186169057, -2.75000000000076, -1.21136434216851e-12, 0.743689875172086}, {-30, -2.24999999999979, 0.500000000000211, 1}, {-30, -2.75000000000076, -1.21136434216851e-12, 1}, {-32.2499999999998, -2.24999999999979, 0.500000000000211, 0.707106781186548}, {-32.7500000000008, -2.75000000000076, -1.21136434216851e-12, 0.707106781186548}, {-32.2499999999998, 1.37772764904064e-16, 0.500000000000211, 1}, {-32.7500000000008, 1.68388934882808e-16, -1.21136434216851e-12, 1}, {-32.2499999999998, 1.87211316225997, 0.500000000000211, 0.743689875172111}, {-32.7500000000008, 2.28813830942971, -1.21136434216851e-12, 0.743689875172111}, {-30.4090909090909, 2.21249737357997, 0.500000000000211, 0.935972451502864}, {-30.5000000000001, 2.70416345659874, -1.21136434216851e-12, 0.935972451502864}});
    const BRep b = face(1, 1, {-9.295836543402, -0.5}, {-0.353553390593274, 0.353553390593274}, {{-30.6196216663691, 0.0356231444328011, 1.43114686768087e-16, 1}, {-30.1215243173232, 0.0792010158066303, 0.5, 1}, {-31.3862293333963, 8.7979888742515, 1.43114686768087e-16, 1}, {-30.8881319843504, 8.84156674562533, 0.5, 1}});
    const bop_ff::Result r = bop_ff::intersect(a, 0, b, 0, 0.0);
    MINI_CHECK(!r.same_domain && r.curves.size() == 1 && r.points.empty());
    MINI_CHECK(worst_fit(a, 0, b, 0, r) <= 1.0 && r.curves[0].tolerance < 1e-6 && std::abs(length(r) - 0.641514) < 1e-5);
}

MINI_TEST("Bop FF", "Corpus Surface Interpolation") {
    // 00171214_fillet_11 against its copy turned 30 degrees, faces 1 and 9: a plane and a cone, "surface intersection could not interpolate".
    const BRep a = face(1, 1, {0, 56.5}, {-97.05, 16.82}, {{-28.25, 97.05, -28.91, 1}, {-28.25, -16.82, -28.91, 1}, {28.25, 97.05, -28.91, 1}, {28.25, -16.82, -28.91, 1}});
    const BRep b = face(2, 1, {0.34979452972231, 0.34979452972231, 1, 1, 2, 2, 2.195065535095, 2.195065535095}, {1.68170353804285, 11.9783888733001}, {{-28.2153995102973, 3.23782158665552, -31.2594095247211, 0.866769718681234}, {-35.4406745499118, 9.09426655026443, -26.8413577048071, 0.866769718681234}, {-26.2240030485669, -4.77422013975465, -17.3821461822168, 0.809559226920253}, {-32.9393300669362, -0.969463477814271, -9.4104659825182, 0.809559226920253}, {-11.4418423442201, -8.02324442719889, -11.7546710413381, 1}, {-14.3718188581705, -5.05048357232142, -2.34193183212227, 1}, {14.5867646039764, -13.744165599309, -1.84574490514698, 0.707106781186548}, {18.3220789369661, -12.2363930014067, 10.1044283976417, 0.707106781186548}, {26.0286069481966, -0.729862125210675, -24.3871797474054, 1}, {32.6938977951366, 4.11055589616162, -18.2093176416787, 1}, {27.7024365366376, 1.17400314163599, -27.6847711203495, 0.942866627546458}, {34.7963542807979, 6.50195458199518, -22.3513416666959, 0.942866627546458}, {28.2153995102974, 3.23782158665576, -31.2594095247215, 0.90802275883178}, {35.4406745499119, 9.09426655026473, -26.8413577048076, 0.90802275883178}});
    const bop_ff::Result r = bop_ff::intersect(a, 0, b, 0, 0.0);
    MINI_CHECK(!r.same_domain && r.curves.size() == 2 && r.points.empty());
    MINI_CHECK(worst_fit(a, 0, b, 0, r) <= 1.0 && std::abs(length(r) - 1.435) < 1e-3);
}

MINI_TEST("Bop FF", "Corpus Constant Input Sphere Plane") {
    // as1-ac-214 against a sphere, face 17: "curve intersection has a constant input", the pole pcurves of the sphere.
    const BRep a = face(1, 1, {-90, 90}, {-75, 75}, {{0, 0, 0, 1}, {0, 150, 0, 1}, {180, 0, 0, 1}, {180, 150, 0, 1}});
    const BRep b = BRep::create_sphere(6).transformed(Xform::translation(36, 30, 4));
    const bop_ff::Result r = bop_ff::intersect(a, 0, b, 0, 0.0);
    MINI_CHECK(!r.same_domain && !r.curves.empty() && r.points.empty());
    MINI_CHECK(worst_fit(a, 0, b, 0, r) <= 1.0 && std::abs(length(r) - 2 * M_PI * std::sqrt(20.0)) < 1e-4);
}

MINI_TEST("Bop FF", "Cylinder Plane Tangent") {
    const BRep cylinder = BRep::create_cylinder(1, 4);
    const int fc = face_near(cylinder, Point(1, 0, 2));
    const BRep plane = face(1, 1, {-3, 3}, {-1, 5}, {{1, -3, -1, 1}, {1, -3, 5, 1}, {1, 3, -1, 1}, {1, 3, 5, 1}});
    const bop_ff::Result touching = bop_ff::intersect(cylinder, fc, plane, 0, 0.0);
    MINI_CHECK(!touching.same_domain && touching.curves.size() == 1 && touching.points.empty());
    MINI_CHECK(worst_fit(cylinder, fc, plane, 0, touching) <= 1.0 && touching.curves[0].tolerance < 1e-6 && std::abs(length(touching) - 4.0) < 1e-3);
    const BRep lifted = plane.transformed(Xform::translation(1e-6, 0, 0));
    MINI_CHECK(bop_ff::intersect(cylinder, fc, lifted, 0, 0.0).curves.empty());
    const bop_ff::Result fuzzy = bop_ff::intersect(cylinder, fc, lifted, 0, 4e-6);
    MINI_CHECK(fuzzy.curves.size() == 1 && worst_fit(cylinder, fc, lifted, 0, fuzzy) <= 1.0 && fuzzy.curves[0].tolerance >= 0.5e-6 && fuzzy.curves[0].tolerance < 5e-6);
}
