// main_16 — V2 SOLID ASSEMBLY + END-TO-END DRIVER acceptance driver.
//
// Part 1  METRIC VALIDATION. The verdict metric is validated against is_solid() on a sphere
//         (poles), a cone (apex) and a cylinder (seam + degenerate caps) BEFORE any other
//         number is printed. Five measurement errors this campaign came from metrics that
//         counted zero-length degenerate edges as naked; this section is the tripwire.
// Part 2  ASSEMBLY UNIT CELLS (kb/port_10_builder_solid.md §5 T1..T9): shared-edge connexity,
//         coordinate independence, cavities, torus genus, disjoint lumps, the op-table.
// Part 3  THE ACCEPTANCE LADDER, in memory, no STEP. Every pair under N rigid motions applied
//         to BOTH operands (all volumes are invariant), judged by
//            closed  = degeneracy-aware naked 0 AND non-manifold 0 AND closure_residual < 1e-9
//            volume  = matches the ANALYTIC value
//            partition identity  cut + common = vol(A)
//         v2 is reported SIDE BY SIDE with the current kernel on the same cells.
// Part 4  A-op-A idempotence under rotation: cut EMPTY, common == A, fuse == A.
//
// Env: SESSION_V2_N   number of motions per cell (default 20)
//      SESSION_V2_ONLY  substring filter on cell names
//      SESSION_V2_DBG   per-run driver census

#include "src/brep.h"
#include "src/brep_massprops.h"
#include "src/v2/brep_v2_boolean.h"
#include "src/v2/brep_v2_solid.h"
#include "src/nurbscurve.h"
#include "src/xform.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;
using namespace session_cpp::v2sol;

static int g_pass = 0, g_fail = 0;

static void cell(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("[V2] %-56s %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL",
                detail.empty() ? "" : "  ", detail.c_str());
    ok ? ++g_pass : ++g_fail;
}

static std::string sfmt(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[512];
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return std::string(buf);
}

///////////////////////////////////////////////////////////////////////////////////////////
// judging
///////////////////////////////////////////////////////////////////////////////////////////

struct Judge {
    int faces = 0, naked = 0, nonmanifold = 0, degenerate = 0, shells = 0;
    double volume = 0.0, area = 0.0, residual = 0.0;
    bool closed = false;
    bool empty = false;
};

static MassPropsOptions mp_opt() {
    MassPropsOptions o;
    o.rel_tolerance = 1e-10;
    o.max_surface_evals = 4000000;
    o.min_face_evals = 40000;
    return o;
}

static Judge judge(const BRep& b) {
    Judge j;
    const V2Verdict v = v2_verdict(b);
    j.faces = v.faces;
    j.naked = v.naked;
    j.nonmanifold = v.nonmanifold;
    j.degenerate = v.degenerate;
    j.shells = v.shells;
    if (v.faces == 0) {
        j.empty = true;
        j.closed = false;
        return j;
    }
    const MassProps mp = brep_massprops(b, mp_opt());
    j.volume = mp.volume;
    j.area = mp.area;
    j.residual = mp.closure_residual;
    j.closed = (v.naked == 0 && v.nonmanifold == 0 && mp.closure_residual < 1e-9);
    return j;
}

static bool vol_ok(double got, double want, double relerr = 1e-6) {
    if (std::fabs(want) < 1e-12) return std::fabs(got) < 1e-9;
    return std::fabs(got - want) <= relerr * std::fabs(want);
}

///////////////////////////////////////////////////////////////////////////////////////////
// rigid motions applied to BOTH operands (every volume in the ladder is invariant)
///////////////////////////////////////////////////////////////////////////////////////////

static Xform motion(int k) {
    if (k == 0) return Xform::identity();
    // deterministic pseudo-random axis/angle/translation
    const double a = 0.7548776662 * k, b = 0.5698402909 * k, c = 0.3141592653 * k;
    Vector axis(std::sin(a * 3.1) + 0.31, std::cos(b * 2.7) - 0.17, std::sin(c * 1.9) + 0.53);
    const double L = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis = Vector(axis[0] / L, axis[1] / L, axis[2] / L);
    const double ang = 0.13 + 3.0 * std::fabs(std::sin(1.7 * k));
    Xform r = Xform::rotation(axis, ang, false);
    Xform t = Xform::translation(0.37 * std::sin(2.1 * k), -0.29 * std::cos(1.3 * k),
                                 0.44 * std::sin(0.9 * k));
    return t * r;
}

static BRep moved(const BRep& b, const Xform& x) {
    BRep c = b;
    c.xform = x;
    return c.transformed();
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 1 — metric validation against is_solid()
///////////////////////////////////////////////////////////////////////////////////////////

static void part1_metric_validation() {
    std::printf("\n=== PART 1  VERDICT METRIC vs is_solid()  (degeneracy awareness) ===\n");
    struct Case {
        const char* name;
        BRep b;
    };
    std::vector<Case> cs;
    cs.push_back({"sphere r=1 (poles+seam)", BRep::create_sphere(1.0)});
    cs.push_back({"cone r=1 h=2 (apex+seam)", BRep::create_cone(1.0, 2.0)});
    cs.push_back({"cylinder r=1 h=2 (seam)", BRep::create_cylinder(1.0, 2.0)});
    cs.push_back({"box 2x2x2", BRep::create_box(1, 1, 1)});
    cs.push_back({"torus R=3 r=1", BRep::create_torus(3.0, 1.0)});

    bool all = true;
    for (auto& c : cs) {
        const V2Verdict v = v2_verdict(c.b);
        const bool is_sol = c.b.is_solid();
        const MassProps mp = brep_massprops(c.b, mp_opt());
        const bool agree = (v.closed == is_sol) && v.naked == 0 && v.nonmanifold == 0 &&
                           mp.closure_residual < 1e-9;
        std::printf("[V2]   %-26s v2:{%s} is_solid=%d resid=%.2e  %s\n", c.name,
                    v.str().c_str(), (int)is_sol, mp.closure_residual, agree ? "AGREE" : "DISAGREE");
        if (!agree) all = false;
    }
    cell("metric_validation_matches_is_solid", all,
         "sphere/cone/cylinder seam edges count 2 uses; pole singularities carry no edge here");

    // the degeneracy branch itself: give the sphere's seam a zero-length 3D curve. It must be
    // classified DEGENERATE and excluded from `naked` -- the exact failure mode that produced
    // five bad measurements this campaign.
    {
        BRep s = BRep::create_sphere(1.0);
        const V2Verdict before = v2_verdict(s);
        for (auto& c : s.m_curves_3d) c = NurbsCurve::create(false, 1, {Point(1, 2, 3), Point(1, 2, 3)});
        const V2Verdict after = v2_verdict(s);
        cell("degenerate_edges_are_not_counted_naked",
             before.degenerate == 0 && after.degenerate == after.edges && after.naked == 0,
             sfmt("before{degen=%d naked=%d} after{degen=%d/%d naked=%d}", before.degenerate,
                  before.naked, after.degenerate, after.edges, after.naked));
    }

    // a genuinely naked edge must be seen: drop one face of a box
    {
        BRep b = BRep::create_box(1, 1, 1);
        std::vector<int> keep{0, 1, 2, 3, 4};
        const V2Verdict v = v2_verdict(b.subset(keep));
        cell("open_box_reports_naked_edges", v.naked == 4 && !v.closed,
             sfmt("naked=%d closed=%d", v.naked, (int)v.closed));
    }

    // rotate each primitive: degeneracy classification must not depend on the frame
    bool rot_ok = true;
    for (auto& c : cs)
        for (int k = 1; k <= 5; ++k) {
            const V2Verdict v0 = v2_verdict(c.b);
            const V2Verdict v1 = v2_verdict(moved(c.b, motion(k)));
            if (v0.naked != v1.naked || v0.nonmanifold != v1.nonmanifold ||
                v0.degenerate != v1.degenerate || v0.shells != v1.shells)
                rot_ok = false;
        }
    cell("metric_invariant_under_rigid_motion", rot_ok);
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 2 — assembly unit cells
///////////////////////////////////////////////////////////////////////////////////////////

static std::vector<V2OrientedFace> all_faces(const V2Topo& t) {
    std::vector<V2OrientedFace> v;
    for (int f = 0; f < t.nb_faces(); ++f) v.push_back(V2OrientedFace{f, false});
    return v;
}

static void part2_assembly() {
    std::printf("\n=== PART 2  ASSEMBLY UNIT CELLS ===\n");

    // T1 — single box, identity assembly
    {
        BRep b = BRep::create_box(1, 1, 1);
        V2Topo t;
        t.build(b);
        V2BuilderSolid bs;
        bs.set_topo(&t);
        bs.set_faces(all_faces(t));
        bs.set_avoid_internal_shapes(true);
        bs.perform();
        const bool ok = bs.shells().size() == 1 && bs.areas().size() == 1 &&
                        bs.shells()[0].closed && bs.shells()[0].manifold &&
                        bs.shells()[0].faces.size() == 6 && !bs.shells()[0].is_hole &&
                        bs.areas()[0].hole_shells.empty() && bs.unused().empty();
        cell("T1 box_one_closed_manifold_shell_one_solid", ok,
             sfmt("shells=%zu solids=%zu faces=%zu closed=%d hole=%d", bs.shells().size(),
                  bs.areas().size(), bs.shells().empty() ? 0 : bs.shells()[0].faces.size(),
                  bs.shells().empty() ? -1 : (int)bs.shells()[0].closed,
                  bs.shells().empty() ? -1 : (int)bs.shells()[0].is_hole));
    }

    // T2 — coordinate independence (I-1): nonsense 3D curves, identical decomposition
    {
        BRep b = BRep::create_box(1, 1, 1);
        V2Topo t0;
        t0.build(b);
        auto blocks0 = v2_make_connexity_blocks(t0, all_faces(t0));

        BRep n = b;
        for (size_t i = 0; i < n.m_curves_3d.size(); ++i) {
            const double s = 1.0 + 0.37 * (double)i;
            n.m_curves_3d[i] = NurbsCurve::create(false, 1,
                                                  {Point(s, -s, 2 * s), Point(-3 * s, s, s)});
        }
        V2Topo t1;
        t1.build(n);
        auto blocks1 = v2_make_connexity_blocks(t1, all_faces(t1));

        bool same = blocks0.size() == blocks1.size();
        for (size_t i = 0; same && i < blocks0.size(); ++i)
            same = blocks0[i].faces == blocks1[i].faces && blocks0[i].regular == blocks1[i].regular;
        cell("T2 shell_decomposition_is_coordinate_independent", same,
             sfmt("blocks %zu vs %zu", blocks0.size(), blocks1.size()));
    }

    // T3 — box with a concentric box cavity (the inner shell is a hole)
    {
        BRep outer = BRep::create_box(4, 4, 4);   // 64
        BRep inner = BRep::create_box(2, 2, 2);   // 8
        BRep b = outer;
        b.append_brep(inner);
        V2Topo t;
        t.build(b);
        std::vector<V2OrientedFace> in;
        for (int f = 0; f < t.nb_faces(); ++f) in.push_back(V2OrientedFace{f, f >= 6});
        V2BuilderSolid bs;
        bs.set_topo(&t);
        bs.set_faces(in);
        bs.set_avoid_internal_shapes(true);
        bs.perform();
        int holes = 0;
        for (const auto& s : bs.shells()) holes += (int)s.is_hole;
        const bool ok = bs.shells().size() == 2 && holes == 1 && bs.areas().size() == 1 &&
                        bs.areas()[0].hole_shells.size() == 1;
        cell("T3 box_with_box_cavity_nests", ok,
             sfmt("shells=%zu holes=%d solids=%zu cavities=%zu", bs.shells().size(), holes,
                  bs.areas().size(),
                  bs.areas().empty() ? 0 : bs.areas()[0].hole_shells.size()));
    }

    // T4 — box with a SPHERICAL cavity (the curved-shell version of T3)
    {
        BRep outer = BRep::create_box(4, 4, 4);
        BRep inner = BRep::create_sphere(1.0);
        BRep b = outer;
        b.append_brep(inner);
        V2Topo t;
        t.build(b);
        std::vector<V2OrientedFace> in;
        for (int f = 0; f < t.nb_faces(); ++f) in.push_back(V2OrientedFace{f, f >= 6});
        V2BuilderSolid bs;
        bs.set_topo(&t);
        bs.set_faces(in);
        bs.set_avoid_internal_shapes(true);
        bs.perform();
        int holes = 0;
        for (const auto& s : bs.shells()) holes += (int)s.is_hole;
        const bool ok = bs.shells().size() == 2 && holes == 1 && bs.areas().size() == 1 &&
                        bs.areas()[0].hole_shells.size() == 1;
        cell("T4 box_with_sphere_cavity_nests", ok,
             sfmt("shells=%zu holes=%d solids=%zu", bs.shells().size(), holes,
                  bs.areas().size()));
    }

    // T7 — torus: closed and manifold with Euler characteristic 0 (closure != Euler 2)
    {
        BRep b = BRep::create_torus(3.0, 1.0);
        V2Topo t;
        t.build(b);
        V2BuilderSolid bs;
        bs.set_topo(&t);
        bs.set_faces(all_faces(t));
        bs.set_avoid_internal_shapes(true);
        bs.perform();
        const bool ok = bs.shells().size() == 1 && bs.shells()[0].closed &&
                        bs.shells()[0].manifold && !bs.shells()[0].is_hole;
        const MassProps mp = brep_massprops(b, mp_opt());
        cell("T7 torus_closed_manifold_not_hole", ok && vol_ok(mp.volume, 2 * M_PI * M_PI * 3.0),
             sfmt("shells=%zu vol=%.9f want=%.9f", bs.shells().size(), mp.volume,
                  2 * M_PI * M_PI * 3.0));
    }

    // T8 — two disjoint boxes: two solids, never merged toward one (I-8)
    {
        BRep b = BRep::create_box(1, 1, 1);
        BRep c = moved(BRep::create_box(1, 1, 1), Xform::translation(10, 0, 0));
        b.append_brep(c);
        V2Topo t;
        t.build(b);
        V2BuilderSolid bs;
        bs.set_topo(&t);
        bs.set_faces(all_faces(t));
        bs.set_avoid_internal_shapes(true);
        bs.perform();
        int holes = 0;
        for (const auto& s : bs.shells()) holes += (int)s.is_hole;
        cell("T8 two_disjoint_boxes_are_two_solids",
             bs.shells().size() == 2 && bs.areas().size() == 2 && holes == 0,
             sfmt("shells=%zu solids=%zu holes=%d", bs.shells().size(), bs.areas().size(), holes));
    }

    // T9 — the op table on a shared same-domain wall (one arena face, both orientations)
    {
        // arena faces 0..4 = A's non-wall faces, 5 = the shared wall, 6..10 = B's non-wall
        V2SelectionInput in;
        for (int f = 0; f < 5; ++f) in.object_faces.push_back(V2OrientedFace{f, false});
        in.object_faces.push_back(V2OrientedFace{5, false});          // A's sighting of the wall
        for (int f = 6; f < 11; ++f) in.tool_faces.push_back(V2OrientedFace{f, false});
        in.tool_faces.push_back(V2OrientedFace{5, true});             // B's sighting, opposed
        V2InParts none;
        in.in_objects = &none;
        in.in_tools = &none;

        std::vector<V2AlertRec> al;
        auto count_wall = [](const std::vector<V2OrientedFace>& v, bool* rev) {
            int n = 0;
            for (const auto& f : v)
                if (f.face == 5) { ++n; if (rev) *rev = f.reversed; }
            return n;
        };
        bool revw = false;
        auto fuse = v2_select_faces(in, v2_op_states(V2Op::Fuse), al);
        const int nf = count_wall(fuse, nullptr);
        auto comm = v2_select_faces(in, v2_op_states(V2Op::Common), al);
        const int nc = count_wall(comm, nullptr);
        auto cut = v2_select_faces(in, v2_op_states(V2Op::Cut), al);
        const int ncut = count_wall(cut, &revw);
        const bool cut_ori_is_a = !revw;
        bool revw21 = false;
        auto cut21 = v2_select_faces(in, v2_op_states(V2Op::Cut21), al);
        const int ncut21 = count_wall(cut21, &revw21);

        // opposed normals: FUSE/COMMON drop the wall; CUT/CUT21 keep it once
        const bool ok = (nf == 0) && (nc == 0) && (ncut == 1) && (ncut21 == 1) &&
                        cut_ori_is_a && revw21;
        cell("T9 sd_wall_op_table_opposed_normals", ok,
             sfmt("fuse=%d common=%d cut=%d(revA=%d) cut21=%d(revB=%d)", nf, nc, ncut,
                  (int)revw, ncut21, (int)revw21));
    }

    // T9b — the same wall with AGREEING normals: FUSE/COMMON keep once, CUT/CUT21 drop
    {
        V2SelectionInput in;
        for (int f = 0; f < 5; ++f) in.object_faces.push_back(V2OrientedFace{f, false});
        in.object_faces.push_back(V2OrientedFace{5, false});
        for (int f = 6; f < 11; ++f) in.tool_faces.push_back(V2OrientedFace{f, false});
        in.tool_faces.push_back(V2OrientedFace{5, false});   // same orientation
        V2InParts none;
        in.in_objects = &none;
        in.in_tools = &none;
        std::vector<V2AlertRec> al;
        auto n_wall = [](const std::vector<V2OrientedFace>& v) {
            int n = 0;
            for (const auto& f : v) n += (f.face == 5);
            return n;
        };
        const int nf = n_wall(v2_select_faces(in, v2_op_states(V2Op::Fuse), al));
        const int nc = n_wall(v2_select_faces(in, v2_op_states(V2Op::Common), al));
        const int ncut = n_wall(v2_select_faces(in, v2_op_states(V2Op::Cut), al));
        const int nc21 = n_wall(v2_select_faces(in, v2_op_states(V2Op::Cut21), al));
        cell("T9b sd_wall_op_table_agreeing_normals",
             nf == 1 && nc == 1 && ncut == 0 && nc21 == 0,
             sfmt("fuse=%d common=%d cut=%d cut21=%d", nf, nc, ncut, nc21));
    }

    // T10 — CUT tool-face reversal and the ordinary IN/OUT rows
    {
        V2SelectionInput in;
        in.object_faces.push_back(V2OrientedFace{0, false});   // A face OUT of B  -> kept as-is
        in.object_faces.push_back(V2OrientedFace{1, false});   // A face IN B      -> dropped
        in.tool_faces.push_back(V2OrientedFace{2, false});     // B face OUT of A  -> dropped
        in.tool_faces.push_back(V2OrientedFace{3, false});     // B face IN A      -> kept REVERSED
        V2InParts in_objects, in_tools;
        in_tools.add(1);      // face 1 (of A) lies IN a tool solid
        in_objects.add(3);    // face 3 (of B) lies IN an object solid
        in.in_objects = &in_objects;
        in.in_tools = &in_tools;
        std::vector<V2AlertRec> al;
        auto cut = v2_select_faces(in, v2_op_states(V2Op::Cut), al);
        bool ok = cut.size() == 2;
        if (ok) {
            bool got0 = false, got3rev = false;
            for (const auto& f : cut) {
                if (f.face == 0 && !f.reversed) got0 = true;
                if (f.face == 3 && f.reversed) got3rev = true;
            }
            ok = got0 && got3rev;
        }
        cell("T10 cut_keeps_A_out_and_B_in_reversed", ok, sfmt("selected=%zu", cut.size()));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 3 — the acceptance ladder
///////////////////////////////////////////////////////////////////////////////////////////

struct Pair {
    std::string name;
    BRep A, B;
    double volA = 0, volB = 0;
    double common = 0, cut = 0;   // analytic
};

struct Score {
    int closed = 0, vol = 0, partition = 0, n = 0;
};

static void run_ladder_pair(const Pair& p, int nmotions, Score& cur, Score& v2s) {
    for (int k = 0; k < nmotions; ++k) {
        const Xform M = motion(k);
        const BRep A = moved(p.A, M);
        const BRep B = moved(p.B, M);

        // --- current kernel -------------------------------------------------------------
        {
            Judge jc, ji;
            bool crashed = false;
            try {
                jc = judge(A.boolean(B, BRep::BooleanOp::Difference));
                ji = judge(A.boolean(B, BRep::BooleanOp::Intersection));
            } catch (...) {
                crashed = true;
            }
            ++cur.n;
            if (!crashed) {
                const bool ccl = jc.closed && ji.closed;
                const bool cvl = vol_ok(jc.volume, p.cut) && vol_ok(ji.volume, p.common);
                const bool cpt = vol_ok(jc.volume + ji.volume, p.volA);
                cur.closed += ccl;
                cur.vol += cvl;
                cur.partition += cpt;
            }
        }

        // --- v2 -------------------------------------------------------------------------
        {
            Judge jc, ji;
            bool crashed = false;
            try {
                jc = judge(v2_cut(A, B));
                ji = judge(v2_common(A, B));
            } catch (...) {
                crashed = true;
            }
            ++v2s.n;
            if (!crashed) {
                const bool ccl = jc.closed && ji.closed;
                const bool cvl = vol_ok(jc.volume, p.cut) && vol_ok(ji.volume, p.common);
                const bool cpt = vol_ok(jc.volume + ji.volume, p.volA);
                v2s.closed += ccl;
                v2s.vol += cvl;
                v2s.partition += cpt;
            }
        }
    }
}

static std::vector<Pair> build_ladder() {
    std::vector<Pair> v;

    {   // 1. box x box, offset in all three axes so no wall is coplanar
        Pair p;
        p.name = "box x box";
        p.A = BRep::create_box(2, 2, 2);
        p.B = moved(BRep::create_box(2, 2, 2), Xform::translation(1.0, 0.5, 0.25));
        p.volA = 8.0;
        p.common = 1.0 * 1.5 * 1.75;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 2. sphere x sphere, lens
        Pair p;
        p.name = "sphere x sphere";
        p.A = BRep::create_sphere(1.0);
        p.B = moved(BRep::create_sphere(1.0), Xform::translation(1.0, 0, 0));
        p.volA = 4.0 * M_PI / 3.0;
        p.common = 5.0 * M_PI / 12.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 3. sphere x cylinder THROUGH THE CENTRE (tilt-invariant closed form)
        Pair p;
        p.name = "sphere x cylinder";
        p.A = BRep::create_sphere(2.5);
        p.B = moved(BRep::create_cylinder(1.0, 8.0), Xform::translation(0, 0, -4.0));
        p.volA = 4.0 * M_PI / 3.0 * 15.625;
        p.common = 4.0 * M_PI / 3.0 * (15.625 - std::pow(6.25 - 1.0, 1.5));
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 4. box x sphere: hemisphere cut off by a box face
        Pair p;
        p.name = "box x sphere";
        p.A = BRep::create_box(3, 3, 3);
        p.B = moved(BRep::create_sphere(1.0), Xform::translation(1.5, 0, 0));
        p.volA = 27.0;
        p.common = 2.0 * M_PI / 3.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 5. cylinder x cylinder: the Steinmetz solid, 16 r^3 / 3
        Pair p;
        p.name = "cylinder x cylinder";
        p.A = moved(BRep::create_cylinder(1.0, 4.0), Xform::translation(0, 0, -2.0));
        Vector ax(1, 0, 0);
        p.B = moved(moved(BRep::create_cylinder(1.0, 4.0), Xform::translation(0, 0, -2.0)),
                    Xform::rotation(ax, M_PI / 2.0, false));
        p.volA = M_PI * 4.0;
        p.common = 16.0 / 3.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 6. box x cone: a horizontal slab takes the cone's top off
        Pair p;
        p.name = "box x cone(A=cone)";
        p.A = BRep::create_cone(1.0, 2.0);
        p.B = moved(BRep::create_box(4, 4, 4), Xform::translation(0, 0, 3.0));
        p.volA = M_PI * 2.0 / 3.0;
        p.common = M_PI / 12.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 7. cone x cone: opposed cones with a genuine lateral-lateral section circle
        Pair p;
        p.name = "cone x cone";
        p.A = BRep::create_cone(1.0, 2.0);
        Vector ax(1, 0, 0);
        p.B = moved(moved(BRep::create_cone(1.5, 2.0), Xform::rotation(ax, M_PI, false)),
                    Xform::translation(0, 0, 1.6));
        p.volA = M_PI * 2.0 / 3.0;
        // r_A(z) = 1 - z/2 on [0,2]; r_B(z) = 0.75(z+0.4) on [-0.4,1.6]; they cross at z=0.56
        p.common = M_PI * (0.5625 * (std::pow(0.96, 3) - std::pow(0.4, 3)) / 3.0 +
                           2.0 * (std::pow(0.72, 3) - std::pow(0.2, 3)) / 3.0);
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {   // 8. torus x torus: coaxial, offset along the axis; Pappus on the lens cross-section
        Pair p;
        p.name = "torus x torus";
        p.A = BRep::create_torus(3.0, 1.0);
        p.B = moved(BRep::create_torus(3.0, 1.0), Xform::translation(0, 0, 1.5));
        p.volA = 2.0 * M_PI * M_PI * 3.0;
        const double lens = 2.0 * std::acos(0.75) - 0.75 * std::sqrt(4.0 - 2.25);
        p.common = 2.0 * M_PI * 3.0 * lens;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    return v;
}

static void diag(const char* only, int kmot) {
    std::printf("\n=== DIAG  motion %d ===\n", kmot);
    for (const Pair& p : build_ladder()) {
        if (only && p.name.find(only) == std::string::npos) continue;
        const Xform M = motion(kmot);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        std::printf("[V2] %-22s analytic volA=%.9f cut=%.9f common=%.9f\n", p.name.c_str(),
                    p.volA, p.cut, p.common);
        Judge ja = judge(A);
        std::printf("[V2]   operandA  F=%d nk=%d nm=%d vol=%.9f resid=%.2e\n", ja.faces, ja.naked,
                    ja.nonmanifold, ja.volume, ja.residual);
        struct R { const char* nm; BRep b; };
        std::vector<R> rs;
        rs.push_back({"cur cut   ", A.boolean(B, BRep::BooleanOp::Difference)});
        rs.push_back({"cur common", A.boolean(B, BRep::BooleanOp::Intersection)});
        V2BooleanReport r1, r2;
        rs.push_back({"v2  cut   ", v2_cut(A, B, 0.0, &r1)});
        rs.push_back({"v2  common", v2_common(A, B, 0.0, &r2)});
        for (auto& r : rs) {
            Judge j = judge(r.b);
            std::printf("[V2]   %s F=%d nk=%d nm=%d sh=%d vol=%.9f resid=%.2e closed=%d\n", r.nm,
                        j.faces, j.naked, j.nonmanifold, j.shells, j.volume, j.residual,
                        (int)j.closed);
        }
        std::printf("[V2]   v2cut   %s\n", r1.str().c_str());
        std::printf("[V2]   v2common %s\n", r2.str().c_str());
        std::fflush(stdout);
    }
}

static void part3_ladder(int nmotions, const char* only) {
    std::printf("\n=== PART 3  ACCEPTANCE LADDER  (%d rigid motions per pair, in memory) ===\n",
                nmotions);
    std::printf("[V2] %-22s | %-23s | %-23s\n", "pair", "CURRENT closed/vol/part",
                "V2      closed/vol/part");
    std::printf("[V2] %s\n", std::string(74, '-').c_str());

    Score tot_cur, tot_v2;
    for (const Pair& p : build_ladder()) {
        if (only && p.name.find(only) == std::string::npos) continue;
        Score cur, v2s;
        run_ladder_pair(p, nmotions, cur, v2s);
        std::printf("[V2] %-22s |   %2d/%-2d   %2d/%-2d   %2d/%-2d |   %2d/%-2d   %2d/%-2d   %2d/%-2d\n",
                    p.name.c_str(), cur.closed, cur.n, cur.vol, cur.n, cur.partition, cur.n,
                    v2s.closed, v2s.n, v2s.vol, v2s.n, v2s.partition, v2s.n);
        std::fflush(stdout);
        tot_cur.closed += cur.closed; tot_cur.vol += cur.vol;
        tot_cur.partition += cur.partition; tot_cur.n += cur.n;
        tot_v2.closed += v2s.closed; tot_v2.vol += v2s.vol;
        tot_v2.partition += v2s.partition; tot_v2.n += v2s.n;
    }
    std::printf("[V2] %s\n", std::string(74, '-').c_str());
    std::printf("[V2] %-22s |   %2d/%-2d   %2d/%-2d   %2d/%-2d |   %2d/%-2d   %2d/%-2d   %2d/%-2d\n",
                "TOTAL", tot_cur.closed, tot_cur.n, tot_cur.vol, tot_cur.n, tot_cur.partition,
                tot_cur.n, tot_v2.closed, tot_v2.n, tot_v2.vol, tot_v2.n, tot_v2.partition,
                tot_v2.n);
    cell("ladder_v2_not_worse_than_current_on_closure", tot_v2.closed >= tot_cur.closed,
         sfmt("v2=%d current=%d", tot_v2.closed, tot_cur.closed));
    cell("ladder_v2_not_worse_than_current_on_volume", tot_v2.vol >= tot_cur.vol,
         sfmt("v2=%d current=%d", tot_v2.vol, tot_cur.vol));
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 3b — the sphere x cylinder TILT SWEEP (relative tilt, exact oracle at every angle)
///////////////////////////////////////////////////////////////////////////////////////////

static void part3b_tilt_sweep(int nsteps) {
    std::printf("\n=== PART 3b  SPHERE r=2.5  x  CYLINDER r=1 THROUGH CENTRE, tilt 0..45 deg ===\n");
    const double want_common = 4.0 * M_PI / 3.0 * (15.625 - std::pow(5.25, 1.5));
    const double want_cut = 4.0 * M_PI / 3.0 * 15.625 - want_common;
    std::printf("[V2] analytic: common=%.6f cut=%.6f (tilt invariant)\n", want_common, want_cut);

    const BRep S = BRep::create_sphere(2.5);
    const BRep C0 = moved(BRep::create_cylinder(1.0, 8.0), Xform::translation(0, 0, -4.0));

    int cur_ok = 0, v2_ok = 0, n = 0;
    for (int i = 0; i < nsteps; ++i) {
        const double deg = 45.0 * i / (double)(nsteps - 1 > 0 ? nsteps - 1 : 1);
        Vector ay(0, 1, 0);
        const BRep C = moved(C0, Xform::rotation(ay, deg * M_PI / 180.0, false));
        static const char* side = std::getenv("SESSION_V2_TILT_SIDE");
        static const double t_from = std::getenv("SESSION_V2_TILT_FROM")
                                         ? std::atof(std::getenv("SESSION_V2_TILT_FROM")) : -1.0;
        if (deg < t_from) continue;
        ++n;
        Judge c_cut, c_com, v_cut, v_com;
        if (!side || std::string(side) != "v2") {
            try {
                c_cut = judge(S.boolean(C, BRep::BooleanOp::Difference));
                c_com = judge(S.boolean(C, BRep::BooleanOp::Intersection));
            } catch (...) {}
        }
        if (!side || std::string(side) != "cur") {
            try {
                v_cut = judge(v2_cut(S, C));
                v_com = judge(v2_common(S, C));
            } catch (...) {}
        }
        const bool cok = c_cut.closed && c_com.closed && vol_ok(c_cut.volume, want_cut, 1e-9) &&
                         vol_ok(c_com.volume, want_common, 1e-9);
        const bool vok = v_cut.closed && v_com.closed && vol_ok(v_cut.volume, want_cut, 1e-9) &&
                         vol_ok(v_com.volume, want_common, 1e-9);
        cur_ok += cok;
        v2_ok += vok;
        std::printf("[V2]  tilt %5.1f  CURRENT cut{cl=%d nk=%d v=%.6f} com{cl=%d v=%.6f}  "
                    "V2 cut{cl=%d nk=%d v=%.6f} com{cl=%d v=%.6f}\n",
                    deg, (int)c_cut.closed, c_cut.naked, c_cut.volume, (int)c_com.closed,
                    c_com.volume, (int)v_cut.closed, v_cut.naked, v_cut.volume,
                    (int)v_com.closed, v_com.volume);
        std::fflush(stdout);
    }
    std::printf("[V2] tilt sweep exact-at-1e-9: CURRENT %d/%d   V2 %d/%d\n", cur_ok, n, v2_ok, n);
    cell("tilt_sweep_v2_not_worse", v2_ok >= cur_ok, sfmt("v2=%d current=%d", v2_ok, cur_ok));
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 4 — A-op-A idempotence under rotation
///////////////////////////////////////////////////////////////////////////////////////////

static void part4_idempotence(int nmotions) {
    std::printf("\n=== PART 4  A-op-A IDEMPOTENCE UNDER ROTATION ===\n");
    struct C { const char* name; BRep b; double vol; };
    std::vector<C> cs;
    cs.push_back({"box", BRep::create_box(2, 2, 2), 8.0});
    cs.push_back({"sphere", BRep::create_sphere(1.0), 4.0 * M_PI / 3.0});
    cs.push_back({"cylinder", BRep::create_cylinder(1.0, 2.0), M_PI * 2.0});
    cs.push_back({"cone", BRep::create_cone(1.0, 2.0), M_PI * 2.0 / 3.0});

    for (auto& c : cs) {
        int cut_ok = 0, com_ok = 0, fus_ok = 0, n = 0;
        int ccut = 0, ccom = 0, cfus = 0;
        for (int k = 0; k < nmotions; ++k) {
            const BRep A = moved(c.b, motion(k));
            ++n;
            {   // v2
                Judge jc = judge(v2_cut(A, A));
                Judge ji = judge(v2_common(A, A));
                Judge jf = judge(v2_fuse(A, A));
                cut_ok += (jc.empty || vol_ok(jc.volume, 0.0));
                com_ok += (ji.closed && vol_ok(ji.volume, c.vol));
                fus_ok += (jf.closed && vol_ok(jf.volume, c.vol));
            }
            {   // current kernel
                Judge jc = judge(A.boolean(A, BRep::BooleanOp::Difference));
                Judge ji = judge(A.boolean(A, BRep::BooleanOp::Intersection));
                Judge jf = judge(A.boolean(A, BRep::BooleanOp::Union));
                ccut += (jc.empty || vol_ok(jc.volume, 0.0));
                ccom += (ji.closed && vol_ok(ji.volume, c.vol));
                cfus += (jf.closed && vol_ok(jf.volume, c.vol));
            }
        }
        std::printf("[V2] %-10s CURRENT cut=%d/%d common=%d/%d fuse=%d/%d   "
                    "V2 cut=%d/%d common=%d/%d fuse=%d/%d\n",
                    c.name, ccut, n, ccom, n, cfus, n, cut_ok, n, com_ok, n, fus_ok, n);
        std::fflush(stdout);
        cell(sfmt("A-op-A %s v2 idempotent", c.name),
             cut_ok == n && com_ok == n && fus_ok == n,
             sfmt("cut=%d common=%d fuse=%d of %d", cut_ok, com_ok, fus_ok, n));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const char* ns = std::getenv("SESSION_V2_N");
    const int n = ns ? std::max(1, std::atoi(ns)) : 20;
    const char* only = std::getenv("SESSION_V2_ONLY");

    if (std::getenv("SESSION_V2_DIAG")) {
        diag(only, std::atoi(std::getenv("SESSION_V2_DIAG")));
        return 0;
    }
    part1_metric_validation();
    part2_assembly();
    part3_ladder(n, only);
    const char* nt = std::getenv("SESSION_V2_NT");
    const char* n4 = std::getenv("SESSION_V2_N4");
    part3b_tilt_sweep(nt ? std::max(2, std::atoi(nt)) : std::min(n, 20));
    part4_idempotence(n4 ? std::max(1, std::atoi(n4)) : std::min(n, 10));

    std::printf("\n[V2] TOTAL %d/%d\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
