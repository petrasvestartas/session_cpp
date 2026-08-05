// main_25 — v3 boolean rewrite gate driver (kb/BOOL_V3_MEMORY.md).
// G1: geometry kernel — analytic surface/curve eval, inverse, implicit, and
// residual-gated recognition from the project's NURBS primitives, including
// random rigid motions (recognition must be rotation-invariant).
#include "v3_geom.h"
#include "v3_topo.h"
#include "v3_classify.h"
#include "v3_bool.h"
#include "brep.h"
#include "file_step.h"
#include "xform.h"
#include <cstdio>
#include <cmath>
#include <random>

using namespace session_cpp;
using namespace v3;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::printf("FAIL [%s:%d] ", __FILE__, __LINE__); \
           std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static std::mt19937 rng(12345);

static Xform random_rotation() {
    std::uniform_real_distribution<double> U(0, 1);
    V3 axis{U(rng) - 0.5, U(rng) - 0.5, U(rng) - 0.5};
    axis = axis.normalized(); // axis_rotation does NOT normalize (non-rigid otherwise!)
    Vector ax(axis.x, axis.y, axis.z);
    double ang = U(rng) * 2.0 * PI;
    return Xform::axis_rotation(ang, ax, false);
}

// ---- analytic self-consistency ----------------------------------------------
static void test_surface_roundtrip(const Srf& s, const char* name,
                                   double u0, double u1, double v0, double v1) {
    std::uniform_real_distribution<double> Uu(u0, u1), Uv(v0, v1);
    double worst_rt = 0, worst_F = 0, worst_nn = 0;
    for (int i = 0; i < 200; i++) {
        double u = Uu(rng), v = Uv(rng);
        V3 p = s.eval(u, v);
        double uu, vv;
        CHECK(s.uv_of(p, uu, vv), "%s: uv_of failed", name);
        V3 q = s.eval(uu, vv);
        worst_rt = std::max(worst_rt, p.dist(q));
        if (s.has_implicit())
            worst_F = std::max(worst_F, std::abs(s.F(p)));
        // normal perpendicular to both partials
        V3 pp, du, dv;
        s.d0d1(u, v, pp, du, dv);
        V3 n = s.normal(u, v);
        double scale = du.norm() * dv.norm() + 1e-9;
        if (du.cross(dv).norm() > 1e-9 * scale)
            worst_nn = std::max(worst_nn,
                std::abs(n.dot(du.normalized())) + std::abs(n.dot(dv.normalized())));
    }
    double scale = s.r > 0 ? s.r : 1.0;
    CHECK(worst_rt < 1e-7 * scale, "%s: roundtrip %.3g", name, worst_rt);
    CHECK(worst_F < 1e-7 * scale * scale, "%s: implicit %.3g", name, worst_F);
    CHECK(worst_nn < 1e-7, "%s: normal not perpendicular %.3g", name, worst_nn);
    std::printf("  %-9s roundtrip %.2e  F %.2e  n-perp %.2e\n", name, worst_rt, worst_F, worst_nn);
}

static void test_analytic_basics() {
    std::printf("[G1a] analytic surface self-consistency\n");
    test_surface_roundtrip(srf_plane(V3{1, 2, 3}, V3{0.3, -0.5, 1}.normalized()), "plane", -5, 5, -5, 5);
    test_surface_roundtrip(srf_cylinder({0.5, -1, 2}, V3{0.2, 0.3, 1}.normalized(), 1.7),
                           "cylinder", 0, TWO_PI, -3, 3);
    test_surface_roundtrip(srf_cone({1, 1, 0}, V3{-0.2, 0.1, 1}.normalized(), 0.35, 0.0),
                           "cone", 0, TWO_PI, 0.3, 4);
    test_surface_roundtrip(srf_sphere({2, -1, 0.5}, 2.3), "sphere",
                           0, TWO_PI, -PI / 2 + 0.05, PI / 2 - 0.05);
    test_surface_roundtrip(srf_torus({0, 0, 0}, V3{0.1, 0.2, 1}.normalized(), 3.0, 1.0),
                           "torus", 0, TWO_PI, 0, TWO_PI);

    std::printf("[G1b] analytic curves\n");
    {
        Cur c = cur_circle(V3{1, 2, 3}, V3{0, 0, 1}, 2.0);
        double worst = 0;
        for (int i = 0; i < 100; i++) {
            double t = i * 0.0628;
            worst = std::max(worst, std::abs(c.eval(t).dist(V3{1, 2, 3}) - 2.0));
        }
        CHECK(worst < 1e-12, "circle radius %.3g", worst);
    }
    {
        Frame ef;
        ef.o = {0, 0, 0}; ef.x = {1, 0, 0}; ef.y = {0, 1, 0}; ef.z = {0, 0, 1};
        Cur e = cur_ellipse(ef, 3.0, 1.0);
        double t;
        CHECK(e.project(V3{3, 0, 0}, t), "ellipse project");
        CHECK(std::abs(t) < 1e-9 || std::abs(std::abs(t) - TWO_PI) < 1e-9, "ellipse t=%g", t);
        CHECK(e.project(V3{0, 1, 0}, t), "ellipse project 2");
        CHECK(std::abs(t - PI / 2) < 1e-6, "ellipse t2=%g", t);
    }
    {
        Cur l = cur_line({1, 2, 3}, {1, 0, 0});
        double t;
        CHECK(l.project(V3{5, 9, 9}, t) && std::abs(t - 4) < 1e-12, "line project t=%g", t);
    }
}

// ---- recognition -------------------------------------------------------------
static const char* kind_name(Srf::K k) {
    switch (k) {
    case Srf::PLANE: return "plane"; case Srf::CYLINDER: return "cylinder";
    case Srf::CONE: return "cone"; case Srf::SPHERE: return "sphere";
    case Srf::TORUS: return "torus"; case Srf::NURBS: return "nurbs";
    }
    return "?";
}

static void test_recognize_brep(const BRep& b, const char* name,
                                const std::vector<Srf::K>& expect) {
    std::printf("[G1c] recognize %s (%zu surfaces)\n", name, b.m_surfaces.size());
    std::vector<int> hist(6, 0);
    double worst = 0, worst_n = 0;
    for (size_t i = 0; i < b.m_surfaces.size(); i++) {
        Srf s = recognize(b.m_surfaces[i]);
        hist[(int)s.k]++;
        // roundtrip residual against source surface
        auto [u0, u1] = b.m_surfaces[i].domain(0);
        auto [v0, v1] = b.m_surfaces[i].domain(1);
        for (int a = 0; a <= 4; a++)
            for (int c = 0; c <= 4; c++) {
                double u = u0 + (u1 - u0) * a / 4.0, v = v0 + (v1 - v0) * c / 4.0;
                V3 p = to_v3(b.m_surfaces[i].point_at(u, v));
                double uu, vv;
                if (!s.uv_of(p, uu, vv)) { worst = 1e9; continue; }
                worst = std::max(worst, s.eval(uu, vv).dist(p));
                if (s.k != Srf::NURBS) {
                    // skip degenerate samples (poles/apex): normal_at reports
                    // unit garbage there; detect via derivative cross product
                    auto d = b.m_surfaces[i].evaluate(u, v, 1);
                    bool degen = d.size() < 3 ||
                        to_v3(d[1]).cross(to_v3(d[2])).norm() < 1e-8;
                    if (degen) continue;
                    V3 n_src = to_v3(b.m_surfaces[i].normal_at(u, v));
                    V3 n_v3 = s.normal(uu, vv);
                    worst_n = std::max(worst_n, (n_src - n_v3).norm());
                }
            }
    }
    std::printf("  kinds: plane=%d cyl=%d cone=%d sphere=%d torus=%d nurbs=%d  resid %.2e  ndiff %.2e\n",
                hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], worst, worst_n);
    for (Srf::K k : expect)
        CHECK(hist[(int)k] > 0, "%s: expected a %s surface", name, kind_name(k));
    if (!expect.empty() && expect[0] != Srf::NURBS)
        CHECK(hist[(int)Srf::NURBS] == 0, "%s: %d surfaces unrecognized", name, hist[(int)Srf::NURBS]);
    CHECK(worst < 1e-5, "%s: recognition residual %.3g", name, worst);
    CHECK(worst_n < 1e-4, "%s: normal mismatch %.3g (nflip wrong?)", name, worst_n);
}

static void test_recognition() {
    test_recognize_brep(BRep::create_box(2, 3, 4), "box", {Srf::PLANE});
    test_recognize_brep(BRep::create_cylinder(1.5, 3.0), "cylinder", {Srf::PLANE, Srf::CYLINDER});
    test_recognize_brep(BRep::create_sphere(2.0), "sphere", {Srf::SPHERE});
    test_recognize_brep(BRep::create_cone(2.0, 3.0), "cone", {Srf::CONE, Srf::PLANE});
    test_recognize_brep(BRep::create_torus(3.0, 1.0), "torus", {Srf::TORUS});
    test_recognize_brep(BRep::create_pyramid(2.0, 3.0), "pyramid", {Srf::PLANE});

    std::printf("[G1d] recognition under random rigid motions\n");
    for (int trial = 0; trial < 8; trial++) {
        Xform xf = random_rotation();
        BRep cyl = BRep::create_cylinder(1.5, 3.0);
        cyl.xform = xf;
        cyl.transform();
        int cyl_count = 0;
        for (auto& s : cyl.m_surfaces)
            if (recognize(s).k == Srf::CYLINDER) cyl_count++;
        CHECK(cyl_count == 1, "rotated cylinder: recognized %d cyl surfaces", cyl_count);

        BRep sph = BRep::create_sphere(2.0);
        sph.xform = xf;
        sph.transform();
        CHECK(recognize(sph.m_surfaces[0]).k == Srf::SPHERE, "rotated sphere not recognized");
    }
}

// ---- topology roundtrip (G2) -------------------------------------------------
static void test_roundtrip(const BRep& b, const char* name, double vol_truth,
                           const Point& inside, const Point& outside) {
    v3::Solid s = v3::from_brep(b);
    int naked = -1, nm = -1;
    bool closed = s.is_closed(&naked, &nm);
    CHECK(closed, "%s: v3 solid not closed (naked=%d nm=%d)", name, naked, nm);
    double vv = s.signed_volume(40);
    CHECK(std::abs(vv - vol_truth) < 0.02 * vol_truth,
          "%s: v3 volume %.4f vs truth %.4f", name, vv, vol_truth);
    BRep r = v3::to_brep(s);
    if (std::getenv("V3DBG2")) {
        auto om = b.face_meshes();
        auto rm = r.face_meshes();
        std::printf("    orig mesh verts: ");
        for (auto& m : om) std::printf("%zu ", m.vertices().size());
        std::printf("\n    rt mesh verts:   ");
        for (auto& m : rm) std::printf("%zu ", m.vertices().size());
        std::printf("\n");
        auto signs = r.face_outward_signs();
        for (size_t fi = 0; fi < r.m_faces.size(); fi++) {
            auto& f = r.m_faces[fi];
            std::printf("    face %zu srf=%d rev=%d loops=%zu osign=%.0f\n", fi,
                        f.surface_index, (int)f.reversed, f.loop_indices.size(),
                        signs[fi]);
            for (int li : f.loop_indices) {
                auto& l = r.m_loops[li];
                std::printf("      loop type=%d trims=%zu area-ish:", (int)l.type,
                            l.trim_indices.size());
                for (int ti : l.trim_indices) {
                    auto& tr = r.m_trims[ti];
                    auto& c2 = r.m_curves_2d[tr.curve_2d_index];
                    auto dom = c2.domain();
                    Point p0 = c2.point_at(dom.first), p1 = c2.point_at(dom.second);
                    std::printf(" [rev=%d (%.2f,%.2f)->(%.2f,%.2f)]", (int)tr.reversed,
                                p0[0], p0[1], p1[0], p1[1]);
                }
                std::printf("\n");
            }
        }
    }
    std::printf("  %-9s faces %d->%d  vol %.4f->%.4f  solid %d->%d\n", name,
                (int)b.m_faces.size(), r.face_count(), vol_truth, r.volume(),
                (int)b.is_solid(), (int)r.is_solid());
    CHECK(r.face_count() == (int)b.m_faces.size(), "%s: face count %d vs %zu",
          name, r.face_count(), b.m_faces.size());
    CHECK(r.is_solid(), "%s: roundtrip not solid", name);
    CHECK(std::abs(r.volume() - vol_truth) < 0.02 * vol_truth,
          "%s: roundtrip volume %.4f vs %.4f", name, r.volume(), vol_truth);
    CHECK(r.contains_point(inside), "%s: inside point rejected", name);
    CHECK(!r.contains_point(outside), "%s: outside point accepted", name);
}

static void test_topology_roundtrip() {
    std::printf("[G2] BRep <-> v3 roundtrip\n");
    test_roundtrip(BRep::create_box(2, 3, 4), "box", 24.0,
                   Point(0, 0, 0), Point(5, 0, 0));
    test_roundtrip(BRep::create_cylinder(1.5, 3.0), "cylinder",
                   PI * 1.5 * 1.5 * 3.0, Point(0, 0, 1.5), Point(3, 0, 1.5));
    test_roundtrip(BRep::create_sphere(2.0), "sphere",
                   4.0 / 3.0 * PI * 8.0, Point(0, 0, 0), Point(5, 0, 0));
    test_roundtrip(BRep::create_cone(2.0, 3.0), "cone",
                   PI * 4.0 * 3.0 / 3.0, Point(0, 0, 1.0), Point(5, 0, 0));
    test_roundtrip(BRep::create_torus(3.0, 1.0), "torus",
                   2 * PI * PI * 3.0, Point(3, 0, 0), Point(0, 0, 0));
    test_roundtrip(BRep::create_pyramid(2.0, 3.0), "pyramid",
                   4.0 * 3.0 / 3.0, Point(0, 0, 1.0), Point(5, 0, 0));
    // and under random rotations
    for (int trial = 0; trial < 4; trial++) {
        Xform xf = random_rotation();
        BRep b = BRep::create_cylinder(1.5, 3.0);
        b.xform = xf;
        b.transform();
        v3::Solid s = v3::from_brep(b);
        CHECK(s.is_closed(), "rot cyl: v3 not closed");
        BRep r = v3::to_brep(s);
        CHECK(r.is_solid(), "rot cyl: roundtrip not solid");
        CHECK(std::abs(r.volume() - PI * 2.25 * 3.0) < 0.02 * PI * 2.25 * 3.0,
              "rot cyl: volume %.4f", r.volume());
    }
}

static void test_classifier() {
    std::printf("[G2b] solid classifier\n");
    auto check = [](const BRep& b, const char* name, const Point& in,
                    const Point& out) {
        v3::Solid s = v3::from_brep(b);
        v3::orient_solid(s);
        v3::PtCls ci = v3::classify_point(s, v3::to_v3(in));
        v3::PtCls co = v3::classify_point(s, v3::to_v3(out));
        CHECK(ci == v3::PtCls::IN, "%s: inside misclassified %d", name, (int)ci);
        CHECK(co == v3::PtCls::OUT, "%s: outside misclassified %d", name, (int)co);
        // boundary point -> ON
        v3::ClosestInfo cl = v3::closest_on_solid(s, v3::to_v3(in));
        (void)cl;
    };
    check(BRep::create_box(2, 3, 4), "box", Point(0, 0, 0), Point(5, 0, 0));
    check(BRep::create_cylinder(1.5, 3.0), "cylinder", Point(0, 0, 1.5), Point(3, 0, 1.5));
    check(BRep::create_sphere(2.0), "sphere", Point(0, 0, 0), Point(5, 0, 0));
    check(BRep::create_cone(2.0, 3.0), "cone", Point(0, 0, 1.0), Point(5, 0, 0));
    check(BRep::create_torus(3.0, 1.0), "torus", Point(3, 0, 0), Point(0, 0, 0));
    check(BRep::create_pyramid(2.0, 3.0), "pyramid", Point(0, 0, 1.0), Point(5, 0, 0));
    // orientation repair: flip every face's rev, orient must restore classifiability
    {
        v3::Solid s = v3::from_brep(BRep::create_cylinder(1.5, 3.0));
        for (auto& f : s.faces) f.rev = !f.rev;
        v3::orient_solid(s);
        CHECK(v3::classify_point(s, v3::V3{0, 0, 1.5}) == v3::PtCls::IN,
              "flipped cyl: inside after orient");
        CHECK(v3::classify_point(s, v3::V3{5, 0, 1.5}) == v3::PtCls::OUT,
              "flipped cyl: outside after orient");
    }
    // rotated
    for (int trial = 0; trial < 3; trial++) {
        Xform xf = random_rotation();
        BRep b = BRep::create_sphere(2.0);
        b.xform = xf;
        b.transform();
        v3::Solid s = v3::from_brep(b);
        v3::orient_solid(s);
        CHECK(v3::classify_point(s, v3::V3{0, 0, 0}) == v3::PtCls::IN,
              "rot sphere: inside");
        CHECK(v3::classify_point(s, v3::V3{9, 9, 9}) == v3::PtCls::OUT,
              "rot sphere: outside");
    }
}

// ---- boolean gate (G3) --------------------------------------------------------
static const char* g_only = nullptr;
static void test_boolean() {
    std::printf("[G3] boolean primitives\n");
    auto run = [&](const BRep& a, const BRep& b, const char* name) {
        if (g_only && std::strcmp(g_only, name) != 0) return;
        v3::Solid sa = v3::from_brep(a), sb = v3::from_brep(b);
        bool ixc = false, ixm = false, ixf = false;
        v3::Solid cut = v3::boolean(sa, sb, v3::BoolOp::CUT, 1e-6, &ixc);
        v3::Solid com = v3::boolean(sa, sb, v3::BoolOp::COMMON, 1e-6, &ixm);
        v3::Solid fus = v3::boolean(sa, sb, v3::BoolOp::FUSE, 1e-6, &ixf);
        CHECK(ixc && ixm && ixf, "%s: operands not intersecting", name);
        BRep bc = v3::to_brep(cut), bm = v3::to_brep(com), bf = v3::to_brep(fus);
        if (std::getenv("V3DUMPB")) {
            auto dump = [](const BRep& b, const char* nm) {
                std::printf("=== %s: %zu faces %zu edges\n", nm, b.m_faces.size(),
                            b.m_topology_edges.size());
                for (size_t e = 0; e < b.m_topology_edges.size(); e++) {
                    auto& te = b.m_topology_edges[e];
                    std::printf("  edge %zu: c3d=%d v=%d->%d trims=%zu deg=%d\n", e,
                                te.curve_3d_index, te.start_vertex, te.end_vertex,
                                te.trim_indices.size(),
                                te.curve_3d_index >= 0
                                    ? b.m_curves_3d[te.curve_3d_index].degree()
                                    : -1);
                }
                for (size_t f = 0; f < b.m_faces.size(); f++) {
                    std::printf("  face %zu srf=%d rev=%d loops=%zu\n", f,
                                b.m_faces[f].surface_index,
                                (int)b.m_faces[f].reversed,
                                b.m_faces[f].loop_indices.size());
                    for (int li : b.m_faces[f].loop_indices) {
                        std::printf("    loop %d type=%d trims:", li,
                                    (int)b.m_loops[li].type);
                        for (int ti : b.m_loops[li].trim_indices) {
                            auto& tr = b.m_trims[ti];
                            std::printf(" (e%d rev=%d ty=%d)", tr.edge_index,
                                        (int)tr.reversed, (int)tr.type);
                        }
                        std::printf("\n");
                    }
                }
            };
            dump(bc, "cut");
            dump(bm, "common");
            dump(bf, "fuse");
        }
        double va = a.volume(), vb = b.volume();
        double vc = bc.volume(), vm = bm.volume(), vf = bf.volume();
        if (std::getenv("V3MESHVOL")) {
            std::printf("  [meshvol] %s cut %.4f com %.4f fus %.4f\n", name,
                        bc.mesh().volume(), bm.mesh().volume(),
                        bf.mesh().volume());
        }
        bool cc = bc.is_solid(), cm = bm.is_solid(), cf = bf.is_solid();
        std::printf("  %-16s cut %8.4f(%d) com %8.4f(%d) fus %8.4f(%d)\n", name, vc,
                    (int)cc, vm, (int)cm, vf, (int)cf);
        CHECK(cc, "%s: cut not solid", name);
        CHECK(cm, "%s: common not solid", name);
        CHECK(cf, "%s: fuse not solid", name);
        double part = vc + vm - va;
        CHECK(std::abs(part) < 0.02 * va,
              "%s: partition cut(%.4f)+common(%.4f)-A(%.4f) = %.4f", name, vc, vm, va,
              part);
        double fid = vf - (va + vb - vm);
        CHECK(std::abs(fid) < 0.02 * (va + vb), "%s: fuse identity %.4f", name, fid);
    };
    run(BRep::create_box(2, 2, 2), BRep::create_cylinder(0.8, 3.0), "box_cyl");
    {
        BRep a = BRep::create_box(2, 2, 2);
        BRep b = BRep::create_box(1.5, 1.5, 1.5);
        Vector ax(1, 1, 1);
        ax = ax / ax.magnitude();
        b.xform = Xform::axis_rotation(0.6, ax, false);
        b.transform();
        run(a, b, "box_box_rot");
    }
    run(BRep::create_sphere(1.5), BRep::create_cylinder(0.8, 3.0), "sph_cyl");
    run(BRep::create_box(2, 2, 2), BRep::create_sphere(1.5), "box_sph");
    run(BRep::create_cone(1.5, 3.0), BRep::create_cylinder(0.8, 3.2), "cone_cyl");
    run(BRep::create_box(4, 4, 1), BRep::create_torus(1.5, 0.5), "box_torus");
    run(BRep::create_pyramid(2, 3), BRep::create_box(3, 3, 1.5), "pyr_box");
    // walker cases (no closed-form SSI -> marched sections -> Plug B fits):
    // general plane-torus (box side walls parallel to the torus axis),
    // non-coaxial torus-cylinder, off-center sphere-torus, non-coaxial
    // cone-sphere. Operands are spun about their own symmetry axes so the
    // marched loops stay clear of face seam corners (corner-straddling zone
    // extraction in the UV arrangement is a known gap).
    auto rot_z = [](BRep b, double ang) {
        Vector z(0, 0, 1);
        b.xform = Xform::axis_rotation(ang, z, false);
        b.transform();
        return b;
    };
    {
        // oblique box face -> generic plane-torus walker loop (compact,
        // seam-interior on the tube's upper half)
        BRep b = BRep::create_box(1.0, 1.0, 1.0);
        Vector ax(0, 1, 0);
        b.xform = Xform::translation(1.5, 0, 0.95) *
                  Xform::axis_rotation(0.35, ax, false);
        b.transform();
        run(b, rot_z(BRep::create_torus(1.5, 0.5), 1.1), "tor_box");
    }
    {
        BRep b = BRep::create_cylinder(0.35, 5.0);
        Vector ax(0, 1, 0);
        b.xform = Xform::translation(0, 0, -0.45) *
                  Xform::axis_rotation(PI / 2, ax, false); // axis +z -> +x
        b.transform();
        run(rot_z(BRep::create_torus(1.5, 0.5), 1.1), b, "tor_cyl");
    }
    {
        BRep b = BRep::create_sphere(0.6);
        b.xform = Xform::translation(1.5, 0, 0.55);
        b.transform();
        run(rot_z(BRep::create_torus(1.5, 0.5), 1.1), b, "sph_tor");
    }
    {
        // regression: walked loop crossing the torus v-seam (z straddles 0)
        BRep b = BRep::create_sphere(0.8);
        b.xform = Xform::translation(1.5, 0, 0.35);
        b.transform();
        run(rot_z(BRep::create_torus(1.5, 0.5), 1.1), b, "sph_tor_seam");
    }
    {
        // regression: axis-parallel walls x=+-1.3 cut full ovals that cross
        // the torus v-seam (general plane-torus, wall perpendicular to axis)
        run(BRep::create_box(2.6, 4.4, 3.0),
            rot_z(BRep::create_torus(1.5, 0.5), 2.2), "tor_wall");
    }
    {
        BRep b = BRep::create_sphere(0.7);
        b.xform = Xform::translation(0.9, 0, 1.2);
        b.transform();
        run(rot_z(BRep::create_cone(1.5, 3.0), 0.9), b, "cone_sph_nc");
    }
    // randomly rotated primitive pairs
    for (int trial = 0; trial < 4; trial++) {
        Xform xf = random_rotation();
        BRep a = BRep::create_box(2, 2, 2);
        BRep b = BRep::create_cylinder(0.9, 3.0);
        b.xform = xf;
        b.transform();
        char nm[32];
        std::snprintf(nm, 32, "box_cyl_rot%d", trial);
        run(a, b, nm);
    }
}

int main(int argc, char** argv) {
    if (argc > 1) g_only = argv[1];
    if (std::getenv("V3MC")) {
        // Monte Carlo oracle for a main_26 case (same per-case pose seeding):
        // V3MC=box_x_box_p1 -> v3 volumes vs MC estimates from contains_point.
        const char* nm = std::getenv("V3MC");
        char f1[64], f2[64];
        int pose_idx = 0;
        if (std::sscanf(nm, "%59[a-z]_x_%59[a-z]_p%d", f1, f2, &pose_idx) != 3) {
            std::printf("V3MC: bad case name %s\n", nm);
            return 1;
        }
        auto load = [](const char* shortname, BRep& out) {
            std::string dir = "../serialization/step_import";
            std::string fn = std::string("occt_prim_") +
                (std::string(shortname) == "cyl" ? "cylinder" :
                 std::string(shortname) == "sph" ? "sphere" : shortname) + ".step";
            auto bs = file_step::read_file_step_breps(dir + "/" + fn);
            out = bs[0];
        };
        BRep A, B0;
        load(f1, A);
        load(f2, B0);
        std::printf("V3MC %s: loaded volA=%.4f volB=%.4f\n", nm, A.volume(),
                    B0.volume());
        std::fflush(stdout);
        std::mt19937 mrng;
        // replicate main_26 pose_of (mesh-bbox center, random rot + translate)
        std::uniform_real_distribution<double> U(0, 1);
        auto mc_pose = [&](const BRep& b, double vol_b, int skip) {
            auto mesh = b.mesh();
            double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
            for (const auto& kv : mesh.vertex) {
                double q[3] = {kv.second.x, kv.second.y, kv.second.z};
                for (int k = 0; k < 3; k++) {
                    lo[k] = std::min(lo[k], q[k]);
                    hi[k] = std::max(hi[k], q[k]);
                }
            }
            double c[3] = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                           (lo[2] + hi[2]) / 2};
            Xform xf;
            for (int a = 0; a <= skip; a++) {
                double d[3] = {U(mrng) - 0.5, U(mrng) - 0.5, U(mrng) - 0.5};
                double dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                double mag = (dn > 1e-12 ? U(mrng) * 0.5 * std::cbrt(vol_b) / dn : 0.0);
                for (int k = 0; k < 3; k++) d[k] *= mag;
                V3 axv{U(mrng) - 0.5, U(mrng) - 0.5, U(mrng) - 0.5};
                axv = axv.normalized();
                Vector ax(axv.x, axv.y, axv.z);
                Xform rot = Xform::axis_rotation(U(mrng) * 2.0 * PI, ax, false);
                xf = Xform::translation(c[0] + d[0], c[1] + d[1], c[2] + d[2]) * rot *
                     Xform::translation(-c[0], -c[1], -c[2]);
            }
            return xf;
        };
        // replicate main_26's run_case: per-case RNG, up to 4 pose attempts,
        // first intersecting one wins
        mrng.seed(20260803u ^ (unsigned)std::hash<std::string>{}(nm));
        BRep B;
        double vc = 0, vm = 0, vf = 0;
        bool got = false;
        for (int attempt = 0; attempt < 4 && !got; attempt++) {
            B = B0;
            B.xform = mc_pose(B0, B0.volume(), 0);
            B.transform();
            v3::Solid sa = v3::from_brep(A), sb = v3::from_brep(B);
            bool ixc = false, ixm = false, ixf = false;
            v3::Solid cut = v3::boolean(sa, sb, v3::BoolOp::CUT, 1e-6, &ixc);
            v3::Solid com = v3::boolean(sa, sb, v3::BoolOp::COMMON, 1e-6, &ixm);
            v3::Solid fus = v3::boolean(sa, sb, v3::BoolOp::FUSE, 1e-6, &ixf);
            if (!(ixc && ixm && ixf)) continue;
            vc = v3::to_brep(cut).volume();
            vm = v3::to_brep(com).volume();
            vf = v3::to_brep(fus).volume();
            if (std::getenv("V3MCMESH")) {
                std::printf("  [meshvol] cut %.4f com %.4f fus %.4f\n",
                            v3::to_brep(cut).mesh().volume(),
                            v3::to_brep(com).mesh().volume(),
                            v3::to_brep(fus).mesh().volume());
                std::fflush(stdout);
            }
            got = true;
        }
        if (!got) {
            std::printf("V3MC %s: no intersecting pose\n", nm);
            return 1;
        }
        if (std::getenv("V3MCPARTS")) {
            // per-part classification audit: v3's classify vs the TRUE
            // contains_point of the other operand at the probe point
            v3::Solid sa = v3::from_brep(A), sb = v3::from_brep(B);
            v3::orient_solid(sa);
            v3::orient_solid(sb);
            v3::Solid cut = v3::boolean(sa, sb, v3::BoolOp::CUT, 1e-6,
                                          nullptr);
            (void)cut;
        }
        // MC oracle on the original breps
        V3 lo, hi, lo2, hi2;
        auto bb = [](const BRep& b, V3& l, V3& h) {
            auto m = b.mesh();
            l = {1e300, 1e300, 1e300};
            h = {-1e300, -1e300, -1e300};
            for (const auto& kv : m.vertex) {
                l.x = std::min(l.x, kv.second.x); h.x = std::max(h.x, kv.second.x);
                l.y = std::min(l.y, kv.second.y); h.y = std::max(h.y, kv.second.y);
                l.z = std::min(l.z, kv.second.z); h.z = std::max(h.z, kv.second.z);
            }
        };
        bb(A, lo, hi);
        bb(B, lo2, hi2);
        lo.x = std::min(lo.x, lo2.x); hi.x = std::max(hi.x, hi2.x);
        lo.y = std::min(lo.y, lo2.y); hi.y = std::max(hi.y, hi2.y);
        lo.z = std::min(lo.z, lo2.z); hi.z = std::max(hi.z, hi2.z);
        double vbox = (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z);
        std::mt19937 mc(7);
        std::uniform_real_distribution<double> Ux(lo.x, hi.x), Uy(lo.y, hi.y),
            Uz(lo.z, hi.z);
        long n_in_a = 0, n_com = 0, n_fus = 0;
        const long N = 300000;
        for (long i = 0; i < N; i++) {
            Point p(Ux(mc), Uy(mc), Uz(mc));
            bool ia = A.contains_point(p), ib = B.contains_point(p);
            if (ia) n_in_a++;
            if (ia && ib) n_com++;
            if (ia || ib) n_fus++;
        }
        double mc_com = vbox * n_com / N, mc_fus = vbox * n_fus / N;
        double mc_a = vbox * n_in_a / N;
        double mc_cut = mc_a - mc_com;
        std::printf("V3MC %s:\n", nm);
        std::printf("  v3:  cut %.4f com %.4f fus %.4f\n", vc, vm, vf);
        std::printf("  mc:  cut %.4f com %.4f fus %.4f  (volA_mc=%.4f)\n", mc_cut,
                    mc_com, mc_fus, mc_a);
        std::printf("  diff: cut %+.4f com %+.4f fus %+.4f\n", vc - mc_cut,
                    vm - mc_com, vf - mc_fus);
        return 0;
    }
    if (std::getenv("V3DUMP")) {
        auto dump = [](const BRep& b, const char* name) {
            std::printf("=== %s: %zu faces %zu edges %zu verts\n", name,
                        b.m_faces.size(), b.m_topology_edges.size(),
                        b.m_topology_vertices.size());
            for (size_t e = 0; e < b.m_topology_edges.size(); e++) {
                auto& te = b.m_topology_edges[e];
                std::printf("  edge %zu: c3d=%d v=%d->%d trims=%zu\n", e,
                            te.curve_3d_index, te.start_vertex, te.end_vertex,
                            te.trim_indices.size());
            }
            for (size_t f = 0; f < b.m_faces.size(); f++) {
                std::printf("  face %zu srf=%d rev=%d loops=%zu\n", f,
                            b.m_faces[f].surface_index, (int)b.m_faces[f].reversed,
                            b.m_faces[f].loop_indices.size());
                for (int li : b.m_faces[f].loop_indices) {
                    std::printf("    loop %d type=%d trims:", li,
                                (int)b.m_loops[li].type);
                    for (int ti : b.m_loops[li].trim_indices) {
                        auto& tr = b.m_trims[ti];
                        std::printf(" (e%d rev=%d ty=%d)", tr.edge_index,
                                    (int)tr.reversed, (int)tr.type);
                    }
                    std::printf("\n");
                }
            }
        };
        dump(BRep::create_sphere(2.0), "sphere");
        dump(BRep::create_cone(2.0, 3.0), "cone");
        dump(BRep::create_torus(3.0, 1.0), "torus");
        dump(BRep::create_pyramid(2.0, 3.0), "pyramid");
        return 0;
    }
    if (std::getenv("V3PROBE")) {
        const char* what = std::getenv("V3PROBE");
        BRep b = std::string(what) == "cone" ? BRep::create_cone(2.0, 3.0)
               : std::string(what) == "pyramid" ? BRep::create_pyramid(2.0, 3.0)
               : std::string(what) == "torus" ? BRep::create_torus(3.0, 1.0)
               : std::string(what) == "cylinder" ? BRep::create_cylinder(0.8, 3.0)
               : BRep::create_sphere(1.5);
        v3::Solid s = v3::from_brep(b);
        for (size_t f = 0; f < s.faces.size(); f++) {
            std::printf("face %zu srf=%d rev=%d\n", f, s.faces[f].srf, (int)s.faces[f].rev);
            for (auto& l : s.faces[f].loops) {
                for (int cei : l.ces) {
                    auto& ce = s.coedges[cei];
                    auto& e = s.edges[ce.edge];
                    std::printf("  ce%d e=%d fwd=%d degen=%d t=[%.4f,%.4f] ckind=%d frame o=(%.2f,%.2f,%.2f) x=(%.2f,%.2f,%.2f) z=(%.2f,%.2f,%.2f) r=%.3f\n",
                                cei, ce.edge, (int)ce.fwd, (int)e.degenerate, e.t0,
                                e.t1, (int)e.c.k, e.c.f.o.x, e.c.f.o.y, e.c.f.o.z,
                                e.c.f.x.x, e.c.f.x.y, e.c.f.x.z, e.c.f.z.x,
                                e.c.f.z.y, e.c.f.z.z, e.c.r);
                }
            }
        }
        return 0;
    }
    if (std::getenv("V3PROBE4")) {
        // tor_cyl's rotated cylinder: dump aligned loop charts
        BRep b = BRep::create_cylinder(0.35, 5.0);
        Vector ax(0, 1, 0);
        b.xform = Xform::axis_rotation(PI / 2, ax, false);
        b.transform();
        v3::Solid s = v3::from_brep(b);
        for (size_t f = 0; f < s.faces.size(); f++) {
            const v3::Srf& sr = s.srfs[s.faces[f].srf];
            std::printf("face %zu kind=%d rev=%d frame z=(%.2f,%.2f,%.2f) x=(%.2f,%.2f,%.2f) o=(%.2f,%.2f,%.2f)\n",
                        f, (int)sr.k, (int)s.faces[f].rev, sr.f.z.x, sr.f.z.y,
                        sr.f.z.z, sr.f.x.x, sr.f.x.y, sr.f.x.z, sr.f.o.x,
                        sr.f.o.y, sr.f.o.z);
            for (auto& l : s.faces[f].loops) {
                for (int cei : l.ces) {
                    auto& ce = s.coedges[cei];
                    std::printf("  ce%d e=%d fwd=%d degen=%d n=%zu first=(%.4f,%.4f) last=(%.4f,%.4f)\n",
                                cei, ce.edge, (int)ce.fwd,
                                (int)s.edges[ce.edge].degenerate, ce.pc.size(),
                                ce.pc.front().u, ce.pc.front().v, ce.pc.back().u,
                                ce.pc.back().v);
                }
            }
        }
        return 0;
    }
    if (std::getenv("V3PROBE3")) {
        v3::Solid a = v3::from_brep(BRep::create_cone(1.5, 3.0));
        v3::Solid c = v3::from_brep(BRep::create_cylinder(0.8, 3.2));
        v3::orient_solid(a);
        v3::orient_solid(c);
        v3::V3 pts[4] = {{0.0647, -0.0470, 2.8400}, {0.0, 0.0, 1.6},
                         {-1.2, -0.6, 0.0001}, {0.0, 0.0, 3.3}};
        for (auto& p : pts) {
            v3::PtCls r = v3::classify_point(c, p, 1e-5);
            v3::ClosestInfo ci = v3::closest_on_solid(c, p);
            std::printf("classify (%.4f,%.4f,%.4f) -> %d  face=%d bnd=%d edge=%d d=%.5f uv=(%.4f,%.4f)\n",
                        p.x, p.y, p.z, (int)r, ci.face, (int)ci.on_boundary,
                        ci.edge, std::sqrt(ci.dist2), ci.u, ci.v);
            if (ci.face >= 0) {
                const v3::Face& f = c.faces[ci.face];
                v3::V3 q = c.srfs[f.srf].eval(ci.u, ci.v);
                v3::V3 n = v3::face_outward_normal(c, f, ci.u, ci.v);
                std::printf("   q=(%.4f,%.4f,%.4f) n=(%.4f,%.4f,%.4f) rev=%d dot=%.5f\n",
                            q.x, q.y, q.z, n.x, n.y, n.z, (int)f.rev,
                            (p - q).dot(n));
            }
        }
        return 0;
    }
    if (std::getenv("V3PROBE2")) {
        v3::Srf pl = v3::srf_plane(v3::V3{0, 0, 0.5}, v3::V3{0, 0, 1});
        v3::Srf to = v3::srf_torus(v3::V3{0, 0, 0}, v3::V3{0, 0, 1}, 1.5, 0.5);
        v3::SSIResult r = v3::ssi(pl, to, 1e-6);
        std::printf("plane(tangent) x torus: curves=%zu same_domain=%d\n",
                    r.curves.size(), (int)r.same_domain);
        for (auto& sc : r.curves)
            std::printf("  exact=%d closed=%d pts=%zu\n", (int)sc.has_exact,
                        (int)sc.closed, sc.pts.size());
        // and the walker for a parallel plane
        v3::Srf pl2 = v3::srf_plane(v3::V3{2, 0, 0}, v3::V3{1, 0, 0});
        v3::SSIResult r2 = v3::ssi(pl2, to, 1e-6);
        std::printf("plane(parallel) x torus: curves=%zu\n", r2.curves.size());

        // replicate the boolean face-pair loop for box(4,4,1) x torus(1.5,0.5)
        v3::Solid A = v3::from_brep(BRep::create_box(4, 4, 1));
        v3::Solid B = v3::from_brep(BRep::create_torus(1.5, 0.5));
        std::printf("box_torus: A faces=%zu B faces=%zu volA=%.4f volB=%.4f\n",
                    A.faces.size(), B.faces.size(), A.signed_volume(10),
                    B.signed_volume(10));
        for (size_t f = 0; f < B.faces.size(); f++)
            std::printf("  B face %zu kind=%d rev=%d\n", f,
                        (int)B.srfs[B.faces[f].srf].k, (int)B.faces[f].rev);
        for (size_t ai = 0; ai < A.faces.size(); ai++) {
            for (size_t bi = 0; bi < B.faces.size(); bi++) {
                v3::V3 lo, hi;
                // crude overlap print via ssi directly
                v3::SSIResult rr = v3::ssi(A.srfs[A.faces[ai].srf],
                                           B.srfs[B.faces[bi].srf], 1e-6);
                std::printf("  A%zu(plane) x B%zu: curves=%zu sd=%d\n", ai, bi,
                            rr.curves.size(), (int)rr.same_domain);
                (void)lo; (void)hi;
            }
        }
        return 0;
    }
    test_analytic_basics();
    test_recognition();
    test_topology_roundtrip();
    test_classifier();
    test_boolean();
    std::printf("\nG1+G2 totals: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
