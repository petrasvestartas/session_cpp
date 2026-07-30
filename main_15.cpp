// main_15 — V2 INTERFERENCE STAGES test driver (src/brep_v2_interf.h/.cpp).
//
// Cell 0 is the VERDICT METRIC VALIDATION and it runs FIRST, before any other number is printed.
// Five measurement errors this session came from metrics that counted a zero-length degenerate
// edge (a sphere pole, a cone apex) as naked. v2_naked_edges() is therefore validated against
// BRep::is_solid() on a sphere, a cone and a cylinder, plus a deliberately planted degenerate
// 1-trim edge that a naive metric WOULD count. Nothing else is trusted until that passes.
//
// Then: vertex-fusion idempotence/order-independence, EF on box x sphere under 20 rotations
// against an ANALYTIC line/sphere oracle, EF on cone pairs against an IMPLICIT-SURFACE oracle,
// tangential typing, and rigid-motion invariance of the interference count.
//
// Prints one line per cell: [V2INT] <name> PASS|FAIL <detail>; exit 0 iff all pass.

#include "src/brep.h"
#include "src/brep_bds.h"
#include "src/v2/brep_v2_interf.h"
#include "src/nurbscurve.h"
#include "src/nurbssurface.h"
#include "src/xform.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;
using namespace session_cpp::v2int;

static int g_pass = 0, g_fail = 0;

static void cell(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("[V2INT] %-46s %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL",
                detail.empty() ? "" : " ", detail.c_str());
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

static bool bits_eq(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
static double d3(const Point& a, const Point& b) { return a.distance(b); }

/// Deterministic xorshift so every run of this driver is byte-identical.
struct Rng {
    unsigned long long s = 88172645463325252ULL;
    double next() {
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        return (double)((s >> 11) & ((1ULL << 53) - 1)) / (double)(1ULL << 53);
    }
};

/// A rigid motion recovered EMPIRICALLY from an Xform, so the test never depends on Xform's
/// internal storage convention: t = X(0), R.col_k = X(e_k) - t.
struct Rigid {
    double R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double t[3] = {0, 0, 0};
    Point fwd(const Point& p) const {
        return Point(R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + t[0],
                     R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + t[1],
                     R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + t[2]);
    }
    Point inv(const Point& p) const {
        const double a = p[0] - t[0], b = p[1] - t[1], c = p[2] - t[2];
        return Point(R[0] * a + R[3] * b + R[6] * c, R[1] * a + R[4] * b + R[7] * c,
                     R[2] * a + R[5] * b + R[8] * c);
    }
};

static Rigid recover_rigid(Xform x) {
    Rigid g;
    const Point o = x.transform_point(Point(0, 0, 0));
    const Point ex = x.transform_point(Point(1, 0, 0));
    const Point ey = x.transform_point(Point(0, 1, 0));
    const Point ez = x.transform_point(Point(0, 0, 1));
    g.t[0] = o[0]; g.t[1] = o[1]; g.t[2] = o[2];
    for (int r = 0; r < 3; ++r) {
        g.R[r * 3 + 0] = ex[r] - o[r];
        g.R[r * 3 + 1] = ey[r] - o[r];
        g.R[r * 3 + 2] = ez[r] - o[r];
    }
    return g;
}

static Rigid compose(const Rigid& second, const Rigid& first) {
    Rigid g;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += second.R[r * 3 + k] * first.R[k * 3 + c];
            g.R[r * 3 + c] = s;
        }
    const Point o = second.fwd(first.fwd(Point(0, 0, 0)));
    g.t[0] = o[0]; g.t[1] = o[1]; g.t[2] = o[2];
    return g;
}

static void apply_motion(BRep& b, const Xform& xr, const Xform& xt) {
    b.xform = xr;
    b.transform();
    b.xform = xt;
    b.transform();
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 0 — VERDICT METRIC VALIDATION.  RUNS FIRST.  NOTHING ELSE IS TRUSTED UNTIL IT PASSES.
///////////////////////////////////////////////////////////////////////////////////////////

/// The NAIVE metric this session got wrong five times: "an edge with != 2 trims is naked".
static int naive_naked(const BRep& b) {
    int n = 0;
    for (const BRepEdge& e : b.m_topology_edges)
        if ((int)e.trim_indices.size() != 2) ++n;
    return n;
}

static bool verdict_metric_ok = false;

static void run_verdict_metric() {
    std::printf("--- verdict metric validation (runs before every other measurement) ---\n");
    bool all = true;
    struct Case { std::string name; BRep b; };
    std::vector<Case> cases;
    cases.push_back({"sphere_poles", BRep::create_sphere(1.0)});
    cases.push_back({"cone_apex", BRep::create_cone(1.0, 2.0)});
    cases.push_back({"cylinder_seam", BRep::create_cylinder(1.0, 2.0)});
    for (Case& c : cases) {
        int nm = 0, deg = 0;
        const int naked = v2_naked_edges(c.b, &nm, &deg);
        const bool solid = c.b.is_solid();
        const bool ok = solid && naked == 0 && nm == 0;
        all = all && ok;
        cell(std::string("verdict_metric ") + c.name, ok,
             sfmt("naked=%d nonmanifold=%d degenerate=%d is_solid=%d naive=%d edges=%d", naked,
                  nm, deg, (int)solid, naive_naked(c.b), (int)c.b.m_topology_edges.size()));
    }
    // The trap, planted explicitly: a ZERO-LENGTH edge carrying ONE trim. is_solid() ignores it
    // (brep.cpp:1085-1102); the naive metric calls it naked. v2_naked_edges must agree with
    // is_solid(), i.e. report 0 naked and 1 degenerate.
    {
        BRep b = BRep::create_sphere(1.0);
        const Point pole = b.m_vertices[1];
        const int ci = b.add_curve_3d(NurbsCurve::create(false, 1, {pole, pole}));
        BRepEdge e;
        e.curve_3d_index = ci;
        e.start_vertex = 1;
        e.end_vertex = 1;
        e.trim_indices.push_back(0);
        b.m_topology_edges.push_back(e);
        int nm = 0, deg = 0;
        const int naked = v2_naked_edges(b, &nm, &deg);
        const bool solid = b.is_solid();
        const int naive = naive_naked(b);
        const bool ok = (naked == 0) && solid && (naive == 1) && (deg == 1);
        all = all && ok;
        cell("verdict_metric degenerate_1trim_edge_not_naked", ok,
             sfmt("naked=%d degenerate=%d is_solid=%d naive_would_say=%d", naked, deg,
                  (int)solid, naive));
    }
    // A genuine hole must still be caught: drop one trim of a real edge.
    {
        BRep b = BRep::create_box(2, 2, 2);
        b.m_topology_edges[0].trim_indices.pop_back();
        int nm = 0, deg = 0;
        const int naked = v2_naked_edges(b, &nm, &deg);
        const bool solid = b.is_solid();
        const bool ok = (naked == 1) && !solid;
        all = all && ok;
        cell("verdict_metric real_hole_is_caught", ok,
             sfmt("naked=%d is_solid=%d", naked, (int)solid));
    }
    verdict_metric_ok = all;
    std::printf("--- verdict metric %s: v2_naked_edges() agrees with is_solid() on sphere "
                "(poles), cone (apex) and cylinder (seam), excludes zero-length degenerate edges "
                "the naive metric counts, and still catches a real hole ---\n",
                all ? "VALIDATED" : "NOT VALIDATED");
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 1 — vertex fusion: idempotence and order independence under adversarial input
///////////////////////////////////////////////////////////////////////////////////////////

static void run_fusion() {
    {
        // Adversarial: coordinates of wildly mixed magnitude, so a naive unsorted sum IS
        // order-dependent. This is what the lexicographic sort at BRepLib.cxx:3090 buys.
        std::vector<Point> pts = {Point(1e8, 1e-8, 0), Point(1e8 + 3e-7, 0, 1e-8),
                                  Point(1e8 - 2e-7, 1e-8, 1e-8), Point(1e8 + 1e-7, 0, 0),
                                  Point(1e8, 2e-8, 3e-8)};
        std::vector<double> tols = {1e-7, 2e-7, 3e-7, 1.5e-7, 1e-7};
        Point c0(0, 0, 0);
        double t0 = 0;
        bds_bounding_vertex(pts, tols, c0, t0);
        bool same = true;
        std::vector<int> idx = {0, 1, 2, 3, 4};
        do {
            std::vector<Point> p2;
            std::vector<double> t2;
            for (int i : idx) { p2.push_back(pts[i]); t2.push_back(tols[i]); }
            Point c(0, 0, 0);
            double t = 0;
            bds_bounding_vertex(p2, t2, c, t);
            same = same && bits_eq(c[0], c0[0]) && bits_eq(c[1], c0[1]) && bits_eq(c[2], c0[2]) &&
                   bits_eq(t, t0);
        } while (std::next_permutation(idx.begin(), idx.end()));
        bool contains = true;
        for (size_t i = 0; i < pts.size(); ++i)
            contains = contains && (d3(c0, pts[i]) + tols[i] <= t0 * (1 + 1e-12) + 1e-20);
        cell("P1 bounding_vertex_order_independent_120perm", same && contains,
             sfmt("tol=%.17g contains=%d", t0, (int)contains));
    }
    {
        // Exact 2-ball smallest enclosing sphere: both balls internally tangent (kb/port_02 T1b).
        std::vector<Point> pts = {Point(0, 0, 0), Point(1, 0, 0)};
        std::vector<double> tols = {1e-3, 2e-3};
        Point c(0, 0, 0);
        double t = 0;
        bds_bounding_vertex(pts, tols, c, t);
        const double e1 = std::fabs(d3(c, pts[0]) + tols[0] - t);
        const double e2 = std::fabs(d3(c, pts[1]) + tols[1] - t);
        cell("P1 two_ball_exact_smallest_enclosing_sphere",
             e1 < 1e-15 && e2 < 1e-15 && std::fabs(t - 0.5015) < 1e-15,
             sfmt("T=%.17g e1=%.3g e2=%.3g", t, e1, e2));
    }
    {
        // Arena level: fusing twice, in any order, with a duplicate member, returns the SAME
        // index and creates no second vertex.
        BdsArena ds;
        const int a = ds.append_vertex(Point(0, 0, 0), 1e-7);
        const int b = ds.append_vertex(Point(1.5e-7, 0, 0), 1e-7);
        const int c = ds.append_vertex(Point(3.0e-7, 0, 0), 1e-7);
        const int n0 = ds.nb_shapes();
        const int f1 = ds.fuse_vertices({a, b, c});
        const int n1 = ds.nb_shapes();
        const int f2 = ds.fuse_vertices({c, b, a, b});
        const int f3 = ds.fuse_vertices({b, c, a});
        cell("P1 fuse_idempotent_and_order_independent",
             f1 == f2 && f2 == f3 && n1 == n0 + 1 && ds.nb_shapes() == n1 &&
                 ds.resolve_sd(a) == f1 && ds.resolve_sd(b) == f1 && ds.resolve_sd(c) == f1,
             sfmt("f=%d shapes %d->%d", f1, n0, ds.nb_shapes()));
    }
    {
        // VV run twice on a real pair: the second run must change nothing (G4).
        BRep A = BRep::create_box(2, 2, 2);
        BRep B = BRep::create_box(2, 2, 2);
        BdsArena ds;
        ds.init({&A, &B});
        V2Interf ia(ds, {&A, &B});
        ia.perform_vv();
        const int n1 = ds.nb_shapes();
        const int fused1 = ia.stats().vv_fused;
        const std::string sig1 = ds.signature();
        V2Interf ib(ds, {&A, &B});
        ib.perform_vv();
        cell("P1 vv_stage_is_idempotent",
             ds.nb_shapes() == n1 && ib.stats().vv_fused == 0 && ds.signature() == sig1,
             sfmt("fused=%d shapes=%d", fused1, n1));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 2 — stage-order coverage
///////////////////////////////////////////////////////////////////////////////////////////

static void run_stage_coverage() {
    {
        // P5: two boxes overlapping in a corner. A = [-1,1]^3, B = [0,2]^3. Every edge of A that
        // enters B pierces three of B's faces (and vice versa); NO edge pair crosses and NO
        // vertex lies on a face, so this cell measures EF alone.
        BRep A = BRep::create_box(2, 2, 2);
        BRep B = BRep::create_box(2, 2, 2);
        B.xform = Xform::translation(1.0, 1.0, 1.0);
        B.transform();
        BdsArena ds;
        ds.init({&A, &B});
        V2Interf iv(ds, {&A, &B});
        iv.perform_all();
        const V2InterfStats& st = iv.stats();
        std::string why;
        const bool inv = iv.check_invariants(&why);
        cell("P5 corner_overlap_EF_pierces_and_splits",
             st.ef_new_vertices == 6 && st.ef_paves == 6 && inv,
             sfmt("ee_v=%d vf=%d ef_t=%d ef_v=%d ef_pav=%d %s", st.ee_new_vertices, st.vf_in,
                  st.ef_transversal, st.ef_new_vertices, st.ef_paves, inv ? "" : why.c_str()));
    }
    {
        // P3: two boxes, one rotated 45 degrees about z. Their top faces (z = 1) and bottom
        // faces (z = -1) are COPLANAR, so 8 edge pairs cross in each plane: 16 EE crossings,
        // each of which must become ONE new arena vertex with a pave on BOTH edges.
        BRep A = BRep::create_box(2, 2, 2);
        BRep B = BRep::create_box(2, 2, 2);
        Vector zax(0, 0, 1);
        B.xform = Xform::rotation(zax, 0.7853981633974483);
        B.transform();
        BdsArena ds;
        ds.init({&A, &B});
        V2Interf iv(ds, {&A, &B});
        iv.perform_all();
        const V2InterfStats& st = iv.stats();
        std::string why;
        const bool inv = iv.check_invariants(&why);
        cell("P3 edge_crossings_become_vertices_and_paves",
             st.ee_new_vertices == 16 && st.ee_paves == 32 && inv,
             sfmt("ee_vparts=%d ee_v=%d ee_paves=%d %s", st.ee_vertex_parts, st.ee_new_vertices,
                  st.ee_paves, inv ? "" : why.c_str()));
    }
    {
        // P4: B's four vertices at x = 1 lie exactly ON A's face x = 1, strictly inside its
        // trim (kb/port_03 acceptance test T5). They must land in that face's IN set.
        BRep A = BRep::create_box(2, 2, 2);
        BRep B = BRep::create_box(1, 1, 1);
        B.xform = Xform::translation(1.5, 0.0, 0.0);
        B.transform();
        BdsArena ds;
        ds.init({&A, &B});
        V2Interf iv(ds, {&A, &B});
        iv.perform_all();
        int in_total = 0;
        for (int i = 0; i < ds.nb_shapes(); ++i)
            if (ds.shape(i).type == BdsType::Face && ds.has_face_info(i))
                in_total += (int)ds.face_info(i).v_in.size();
        cell("P4 vertex_on_face_lands_in_the_face_IN_set",
             iv.stats().vf_in >= 4 && in_total >= 4,
             sfmt("vf_in=%d face_in_entries=%d rejected_on=%d", iv.stats().vf_in, in_total,
                  iv.stats().vf_rejected_on));
    }
    {
        // Face-to-face contact: coincident edges must become COMMON BLOCKS (P3 / G9), not a
        // cloud of vertex parts.
        BRep C = BRep::create_box(2, 2, 2);
        BRep D = BRep::create_box(2, 2, 2);
        D.xform = Xform::translation(2.0, 0.0, 0.0);
        D.transform();
        BdsArena ds2;
        ds2.init({&C, &D});
        V2Interf iv2(ds2, {&C, &D});
        iv2.perform_all();
        const V2InterfStats& s2 = iv2.stats();
        cell("P3 face_contact_makes_common_blocks",
             s2.ee_edge_parts > 0 && s2.ee_common_blocks > 0,
             sfmt("vv=%d ve=%d ee_edge=%d cb=%d ef_coin=%d", s2.vv_fused, s2.ve_paves,
                  s2.ee_edge_parts, s2.ee_common_blocks, s2.ef_coincident));
    }
    {
        // P2: a vertex of B sitting on an edge of A must become a PAVE on that edge, referring
        // to the vertex BY ARENA INDEX (not rediscovered later by distance).
        // B's two corners (+-0.5, 1, 1) sit exactly on A's edge {y=1, z=1, x in [-1,1]}.
        BRep A = BRep::create_box(2, 2, 2);
        BRep B = BRep::create_box(1, 1, 1);
        B.xform = Xform::translation(0.0, 1.5, 1.5);
        B.transform();
        BdsArena ds;
        ds.init({&A, &B});
        V2Interf iv(ds, {&A, &B});
        iv.perform_vv();
        ds.update_pave_blocks();
        iv.perform_ve();
        int shared = 0;
        for (int i = 0; i < ds.nb_shapes(); ++i) {
            if (ds.shape(i).type != BdsType::Edge || ds.rank(i) != 0) continue;
            std::vector<BdsPave> pv;
            if (!ds.has_pave_blocks(i) || !ds.paves(i, pv)) continue;
            for (const BdsPave& p : pv)
                if (ds.rank(ds.resolve_sd(p.vertex)) == 1) ++shared;
        }
        cell("P2 vertex_on_edge_becomes_a_pave_by_index",
             iv.stats().ve_paves > 0 && shared > 0,
             sfmt("ve_paves=%d cross_operand_paves=%d", iv.stats().ve_paves, shared));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 3 — tangential edge/face contact must be TYPED, not emitted as a piercing
///////////////////////////////////////////////////////////////////////////////////////////

static void run_tangential() {
    BRep cyl = BRep::create_cylinder(1.0, 2.0);
    const V2FaceView fv = v2_make_face_view(cyl, 0, 0);   // face 0 is the lateral surface
    if (std::getenv("V2DBG")) {
        std::fprintf(stderr, "[FV] ok=%d faces=%d loops=%d u[%g,%g] v[%g,%g] grid=%dx%d\n",
                     (int)fv.ok(), (int)cyl.m_faces.size(), (int)fv.loops.size(), fv.umin,
                     fv.umax, fv.vmin, fv.vmax, fv.nu, fv.nv);
        const NurbsCurve dbg = NurbsCurve::create(false, 1, {Point(-2, 0.5, 1), Point(2, 0.5, 1)});
        const std::pair<double, double> dd = dbg.domain();
        double su = -1e300, sv = -1e300;
        for (int i = 0; i <= 16; ++i) {
            const double t = dd.first + (dd.second - dd.first) * i / 16.0;
            const Point p = dbg.point_at(t);
            double u = 0, v = 0, dist = 0;
            const bool okp = v2_proj_ps(fv, p, su, sv, u, v, dist);
            su = u; sv = v;
            double nx = 0, ny = 0, nz = 0;
            const bool okn = v2_surface_normal(*fv.surface, u, v, nx, ny, nz);
            const Point q = fv.surface->point_at(u, v);
            const double s = (p[0] - q[0]) * nx + (p[1] - q[1]) * ny + (p[2] - q[2]) * nz;
            std::fprintf(stderr, "[SEC] t=%.4f P=(%.3f,%.3f,%.3f) ok=%d,%d uv=(%.4f,%.4f) "
                         "S=(%.3f,%.3f,%.3f) d=%.4f s=%.4f\n", t, p[0], p[1], p[2], (int)okp,
                         (int)okn, u, v, q[0], q[1], q[2], dist, s);
        }
    }
    const NurbsCurve tangent = NurbsCurve::create(false, 1, {Point(-2, 1, 1), Point(2, 1, 1)});
    const NurbsCurve secant = NurbsCurve::create(false, 1, {Point(-2, 0.5, 1), Point(2, 0.5, 1)});
    const NurbsCurve missing = NurbsCurve::create(false, 1, {Point(-2, 1.2, 1), Point(2, 1.2, 1)});
    const std::pair<double, double> dm = tangent.domain();

    std::vector<V2EFPart> pt, ps, pm;
    v2_edge_face(tangent, dm.first, dm.second, V2_CONFUSION, fv, V2_CONFUSION, V2_CONFUSION, pt);
    v2_edge_face(secant, dm.first, dm.second, V2_CONFUSION, fv, V2_CONFUSION, V2_CONFUSION, ps);
    v2_edge_face(missing, dm.first, dm.second, V2_CONFUSION, fv, V2_CONFUSION, V2_CONFUSION, pm);

    const bool tang_ok = pt.size() == 1 && pt[0].type == V2PartType::Vertex &&
                         pt[0].kind == V2EFKind::Tangential;
    double terr = 1e300;
    if (!pt.empty()) terr = d3(tangent.point_at(pt[0].tv), Point(0, 1, 1));
    cell("P5 tangential_contact_typed_Tangential", tang_ok && terr < 1e-6,
         sfmt("parts=%d kind=%d err=%.3g", (int)pt.size(), pt.empty() ? -1 : (int)pt[0].kind,
              terr));

    bool sec_ok = ps.size() == 2;
    for (const V2EFPart& p : ps) sec_ok = sec_ok && p.kind == V2EFKind::Transversal;
    double serr = 1e300;
    if (ps.size() == 2) {
        const double xr = std::sqrt(1.0 - 0.25);
        serr = std::max(std::fabs(std::fabs(secant.point_at(ps[0].tv)[0]) - xr),
                        std::fabs(std::fabs(secant.point_at(ps[1].tv)[0]) - xr));
    }
    cell("P5 secant_contact_typed_Transversal_2pts", sec_ok && serr < 1e-7,
         sfmt("parts=%d err=%.3g", (int)ps.size(), serr));

    cell("P5 clear_miss_yields_no_part", pm.empty(), sfmt("parts=%d", (int)pm.size()));

    // kb/port_03 I3d: raising the fuzzy value must not turn the tangency into an EDGE part.
    std::vector<V2EFPart> pf;
    v2_edge_face(tangent, dm.first, dm.second, V2_CONFUSION, fv, V2_CONFUSION, 1e-3, pf);
    bool still = !pf.empty();
    for (const V2EFPart& p : pf) still = still && p.type == V2PartType::Vertex;
    cell("P5 tangency_survives_fuzz_1e-3_without_becoming_Edge", still,
         sfmt("parts=%d type0=%d", (int)pf.size(), pf.empty() ? -1 : (int)pf[0].type));
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 4 — EF on box x sphere, 20 rotations, against an ANALYTIC oracle
///////////////////////////////////////////////////////////////////////////////////////////

/// Analytic line/sphere: the box's straight edges against |P| = R centred at the origin.
static void oracle_box_edges_vs_sphere(const BRep& box, double R, std::vector<int>& edge_of,
                                       std::vector<Point>& pts) {
    for (size_t i = 0; i < box.m_topology_edges.size(); ++i) {
        const BRepEdge& e = box.m_topology_edges[i];
        if (e.curve_3d_index < 0) continue;
        const NurbsCurve& c = box.m_curves_3d[e.curve_3d_index];
        const std::pair<double, double> dm = c.domain();
        const Point P0 = c.point_at(dm.first), P1 = c.point_at(dm.second);
        const double dx = P1[0] - P0[0], dy = P1[1] - P0[1], dz = P1[2] - P0[2];
        const double a = dx * dx + dy * dy + dz * dz;
        const double b = 2.0 * (P0[0] * dx + P0[1] * dy + P0[2] * dz);
        const double cc = P0[0] * P0[0] + P0[1] * P0[1] + P0[2] * P0[2] - R * R;
        const double disc = b * b - 4 * a * cc;
        if (disc <= 0 || a <= 0) continue;
        const double sq = std::sqrt(disc);
        for (int k = 0; k < 2; ++k) {
            const double s = (k ? (-b + sq) : (-b - sq)) / (2 * a);
            if (s <= 1e-7 || s >= 1 - 1e-7) continue;
            edge_of.push_back((int)i);
            pts.push_back(Point(P0[0] + s * dx, P0[1] + s * dy, P0[2] + s * dz));
        }
    }
}

/// The sphere's seam edge against the box's six planar faces, evaluated in the BOX's own frame.
/// Uses only plane equations — independent of the production projector.
static void oracle_seam_vs_box(const BRep& sph, const Rigid& g, double h,
                               std::vector<Point>& pts) {
    for (size_t i = 0; i < sph.m_topology_edges.size(); ++i) {
        const BRepEdge& e = sph.m_topology_edges[i];
        if (e.curve_3d_index < 0) continue;
        const NurbsCurve& c = sph.m_curves_3d[e.curve_3d_index];
        const std::pair<double, double> dm = c.domain();
        const int N = 20000;
        for (int ax = 0; ax < 3; ++ax) {
            for (int sgn = 0; sgn < 2; ++sgn) {
                const double pl = sgn ? h : -h;
                double prev = 0;
                for (int k = 0; k <= N; ++k) {
                    const double t = dm.first + (dm.second - dm.first) * (double)k / N;
                    const double f = g.inv(c.point_at(t))[ax] - pl;
                    if (k > 0 && prev * f < 0) {
                        double ta = dm.first + (dm.second - dm.first) * (double)(k - 1) / N;
                        double tb = dm.first + (dm.second - dm.first) * (double)k / N;
                        double fa = prev;
                        for (int it = 0; it < 80; ++it) {
                            const double tm = 0.5 * (ta + tb);
                            const double fm = g.inv(c.point_at(tm))[ax] - pl;
                            if ((fa < 0) == (fm < 0)) { ta = tm; fa = fm; } else { tb = tm; }
                        }
                        const Point w2 = c.point_at(0.5 * (ta + tb));
                        const Point l2 = g.inv(w2);
                        bool inside = true;
                        for (int q = 0; q < 3; ++q)
                            if (q != ax && std::fabs(l2[q]) > h - 1e-6) inside = false;
                        if (inside) pts.push_back(w2);
                    }
                    prev = f;
                }
            }
        }
    }
}

static void run_ef_box_sphere() {
    Rng rng;
    const double h = 1.0;        // box half-extent
    const double R = 1.55 * h;   // between the edge distance sqrt(2)h and the corner sqrt(3)h
    int tot_exp = 0, tot_found = 0, tot_split = 0, tot_seam_exp = 0, tot_seam_found = 0;
    int worst_missing = 0, poses = 0, inv_ok = 0, dup = 0;
    for (int rot = 0; rot < 20; ++rot) {
        BRep box = BRep::create_box(2 * h, 2 * h, 2 * h);
        BRep sph = BRep::create_sphere(R);
        Vector axis(rng.next() * 2 - 1, rng.next() * 2 - 1, rng.next() * 2 - 1);
        if (axis.magnitude() < 1e-6) axis = Vector(0, 0, 1);
        const Xform xr = Xform::rotation(axis, rng.next() * 6.283185307179586);
        const Xform xt = Xform::translation((rng.next() * 2 - 1) * 0.3 * h,
                                            (rng.next() * 2 - 1) * 0.3 * h,
                                            (rng.next() * 2 - 1) * 0.3 * h);
        const Rigid g = compose(recover_rigid(xt), recover_rigid(xr));
        apply_motion(box, xr, xt);

        std::vector<int> oe;
        std::vector<Point> op;
        oracle_box_edges_vs_sphere(box, R, oe, op);
        std::vector<Point> os;
        oracle_seam_vs_box(sph, g, h, os);

        BdsArena ds;
        ds.init({&box, &sph});
        V2Interf iv(ds, {&box, &sph});
        iv.perform_all();

        int found = 0, split = 0;
        for (size_t k = 0; k < op.size(); ++k) {
            const int ae = ds.index_of_edge(0, oe[k]);
            bool hit = false;
            for (const V2EFRecord& r : iv.ef_records()) {
                if (r.edge != ae || r.type != V2PartType::Vertex) continue;
                const NurbsCurve* c = ds.edge_curve(r.edge);
                if (c && d3(c->point_at(r.t), op[k]) < 1e-6) { hit = true; ++found; break; }
            }
            if (!hit && std::getenv("V2DBG")) {
                const NurbsCurve* c = ds.edge_curve(ae);
                double bt = 0, bd = 0;
                const std::pair<double, double> rr = ds.edge_range(ae);
                v2_proj_pc(*c, rr.first, rr.second, op[k], bt, bd);
                int nrec = 0;
                for (const V2EFRecord& r : iv.ef_records()) if (r.edge == ae) ++nrec;
                std::fprintf(stderr,
                             "[MISS] pose=%d edge=%d ae=%d p=(%.6f,%.6f,%.6f) |p|=%.6f t=%.6f "
                             "projdist=%.3g recs=%d\n",
                             rot, oe[k], ae, op[k][0], op[k][1], op[k][2],
                             std::sqrt(op[k][0] * op[k][0] + op[k][1] * op[k][1] +
                                       op[k][2] * op[k][2]),
                             bt, bd, nrec);
                // Re-run the EF core on that (edge, sphere face) pair and dump every part.
                const int nf = ds.index_of_face(1, 0);
                const V2FaceView& fvv = iv.face_view(nf);
                std::vector<V2EFPart> pp;
                v2_edge_face(*c, rr.first, rr.second, ds.tolerance(ae), fvv, ds.tolerance(nf),
                             V2_CONFUSION, pp);
                for (const V2EFPart& q : pp)
                    std::fprintf(stderr, "   part kind=%d type=%d tv=%.6f uv=(%.5f,%.5f) "
                                 "d=%.3g state=%d\n", (int)q.kind, (int)q.type, q.tv, q.u, q.v,
                                 q.dist, (int)q.state);
            }
            std::vector<BdsPave> pv;
            if (ds.has_pave_blocks(ae) && ds.paves(ae, pv))
                for (const BdsPave& p : pv)
                    if (d3(ds.vertex_point(p.vertex), op[k]) < 1e-6) { ++split; break; }
            // G2 / kb/port_03 I1d: EXACTLY ONE arena vertex at the piercing, never two a hair
            // apart. The A-side and B-side of the same piercing must have fused.
            int near = 0;
            for (int q = 0; q < ds.nb_shapes(); ++q)
                if (ds.shape(q).type == BdsType::Vertex && ds.resolve_sd(q) == q &&
                    d3(ds.vertex_point(q), op[k]) < 1e-6)
                    ++near;
            if (near != 1) ++dup;
        }
        int sfound = 0;
        for (const Point& p : os)
            for (const V2EFRecord& r : iv.ef_records()) {
                if (r.type != V2PartType::Vertex) continue;
                const NurbsCurve* c = ds.edge_curve(r.edge);
                if (c && d3(c->point_at(r.t), p) < 1e-5) { ++sfound; break; }
            }
        std::string why;
        if (iv.check_invariants(&why)) ++inv_ok;
        tot_exp += (int)op.size();
        tot_found += found;
        tot_split += split;
        tot_seam_exp += (int)os.size();
        tot_seam_found += sfound;
        worst_missing = std::max(worst_missing, (int)op.size() - found);
        ++poses;
    }
    cell("P5 EF box_x_sphere 20rot box_edge_pierces_sphere",
         tot_found == tot_exp && tot_split == tot_exp && tot_exp > 0,
         sfmt("poses=%d expected=%d found=%d split=%d worst_missing=%d", poses, tot_exp,
              tot_found, tot_split, worst_missing));
    cell("P5 EF box_x_sphere 20rot seam_pierces_box_face",
         tot_seam_found == tot_seam_exp,
         sfmt("expected=%d found=%d", tot_seam_exp, tot_seam_found));
    cell("P5 EF box_x_sphere 20rot arena_invariants", inv_ok == poses,
         sfmt("%d/%d", inv_ok, poses));
    cell("P5 EF box_x_sphere 20rot exactly_one_vertex_per_piercing", dup == 0,
         sfmt("piercings=%d with_wrong_vertex_count=%d", tot_exp, dup));
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 5 — EF on cone pairs, against an IMPLICIT-SURFACE oracle
///////////////////////////////////////////////////////////////////////////////////////////

/// Signed implicit function of a cone's LATERAL face in its own frame: distance-to-axis minus
/// the local cone radius, valid for z in (0,h). Independent of everything the production code
/// computes — it is the algebraic surface, not a projection.
static double cone_lat(const Point& p, double r, double h) {
    return std::sqrt(p[0] * p[0] + p[1] * p[1]) - r * (1.0 - p[2] / h);
}

static void run_ef_cone_cone() {
    Rng rng;
    rng.s = 0x1234567890abcdefULL;
    const double r = 1.0, h = 2.0;
    int tot_exp = 0, tot_found = 0, tot_split = 0, poses = 0, inv_ok = 0;
    for (int rot = 0; rot < 20; ++rot) {
        BRep A = BRep::create_cone(r, h);
        BRep B = BRep::create_cone(r, h);
        Vector axis(rng.next() * 2 - 1, rng.next() * 2 - 1, rng.next() * 2 - 1);
        if (axis.magnitude() < 1e-6) axis = Vector(0, 0, 1);
        const Xform xr = Xform::rotation(axis, rng.next() * 6.283185307179586);
        // Rotate B arbitrarily, then translate it so its centroid lands on A's centroid plus a
        // small jitter. Rotating about the origin alone leaves most poses barely touching; this
        // guarantees deep interpenetration, which is what makes the cell a cone test rather than
        // a near-miss test.
        const Rigid gr = recover_rigid(xr);
        double cx = 0, cy = 0, cz = 0;
        for (const Point& p : B.m_vertices) {
            const Point q = gr.fwd(p);
            cx += q[0]; cy += q[1]; cz += q[2];
        }
        cx /= (double)B.m_vertices.size();
        cy /= (double)B.m_vertices.size();
        cz /= (double)B.m_vertices.size();
        double ax = 0, ay = 0, az = 0;
        for (const Point& p : A.m_vertices) { ax += p[0]; ay += p[1]; az += p[2]; }
        ax /= (double)A.m_vertices.size();
        ay /= (double)A.m_vertices.size();
        az /= (double)A.m_vertices.size();
        const Xform xt = Xform::translation(ax - cx + 0.25 * r * (rng.next() * 2 - 1),
                                            ay - cy + 0.25 * r * (rng.next() * 2 - 1),
                                            az - cz + 0.25 * h * (rng.next() * 2 - 1));
        const Rigid g = compose(recover_rigid(xt), gr);
        apply_motion(B, xr, xt);

        struct Hit { int operand, edge; Point p; };
        std::vector<Hit> hits;
        auto sweep = [&](const BRep& src, int op_src, bool to_b) {
            for (size_t i = 0; i < src.m_topology_edges.size(); ++i) {
                const BRepEdge& e = src.m_topology_edges[i];
                if (e.curve_3d_index < 0) continue;
                const NurbsCurve& c = src.m_curves_3d[e.curve_3d_index];
                const std::pair<double, double> dm = c.domain();
                const int N = 8000;
                double prev = 0;
                bool prev_ok = false;
                for (int k = 0; k <= N; ++k) {
                    const double t = dm.first + (dm.second - dm.first) * (double)k / N;
                    const Point w = c.point_at(t);
                    const Point l = to_b ? g.inv(w) : w;
                    const bool ok = (l[2] > 1e-6 && l[2] < h - 1e-6);
                    const double f = cone_lat(l, r, h);
                    if (k > 0 && prev_ok && ok && prev * f < 0) {
                        double ta = dm.first + (dm.second - dm.first) * (double)(k - 1) / N;
                        double tb = dm.first + (dm.second - dm.first) * (double)k / N;
                        double fa = prev;
                        for (int it = 0; it < 80; ++it) {
                            const double tm = 0.5 * (ta + tb);
                            const Point wm = c.point_at(tm);
                            const double fm = cone_lat(to_b ? g.inv(wm) : wm, r, h);
                            if ((fa < 0) == (fm < 0)) { ta = tm; fa = fm; } else { tb = tm; }
                        }
                        Hit hh;
                        hh.operand = op_src;
                        hh.edge = (int)i;
                        hh.p = c.point_at(0.5 * (ta + tb));
                        hits.push_back(hh);
                    }
                    prev = f;
                    prev_ok = ok;
                }
            }
        };
        sweep(A, 0, true);    // A's edges against B's lateral cone
        sweep(B, 1, false);   // B's edges against A's lateral cone (A sits at the origin)

        BdsArena ds;
        ds.init({&A, &B});
        V2Interf iv(ds, {&A, &B});
        iv.perform_all();

        int found = 0, split = 0;
        for (const Hit& hh : hits) {
            const int ae = ds.index_of_edge(hh.operand, hh.edge);
            for (const V2EFRecord& rec : iv.ef_records()) {
                if (rec.edge != ae || rec.type != V2PartType::Vertex) continue;
                const NurbsCurve* c = ds.edge_curve(rec.edge);
                if (c && d3(c->point_at(rec.t), hh.p) < 1e-5) { ++found; break; }
            }
            std::vector<BdsPave> pv;
            if (ds.has_pave_blocks(ae) && ds.paves(ae, pv))
                for (const BdsPave& p : pv)
                    if (d3(ds.vertex_point(p.vertex), hh.p) < 1e-5) { ++split; break; }
        }
        std::string why;
        if (iv.check_invariants(&why)) ++inv_ok;
        tot_exp += (int)hits.size();
        tot_found += found;
        tot_split += split;
        ++poses;
    }
    cell("P5 EF cone_x_cone 20rot lateral_piercings",
         tot_found == tot_exp && tot_split == tot_exp && tot_exp > 0,
         sfmt("poses=%d expected=%d found=%d split=%d", poses, tot_exp, tot_found, tot_split));
    cell("P5 EF cone_x_cone 20rot arena_invariants", inv_ok == poses,
         sfmt("%d/%d", inv_ok, poses));
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 5b — EF on cylinder pairs, against an IMPLICIT-SURFACE oracle
///////////////////////////////////////////////////////////////////////////////////////////

/// Signed implicit function of a cylinder's LATERAL face in its own frame, valid for z in (0,h).
static double cyl_lat(const Point& p, double r) {
    return std::sqrt(p[0] * p[0] + p[1] * p[1]) - r;
}

static void run_ef_cyl_cyl() {
    Rng rng;
    rng.s = 0x0badc0ffee123456ULL;
    const double r = 1.0, h = 2.0;
    int tot_exp = 0, tot_found = 0, tot_split = 0, poses = 0, inv_ok = 0;
    for (int rot = 0; rot < 20; ++rot) {
        BRep A = BRep::create_cylinder(r, h);
        BRep B = BRep::create_cylinder(r, h);
        Vector axis(rng.next() * 2 - 1, rng.next() * 2 - 1, rng.next() * 2 - 1);
        if (axis.magnitude() < 1e-6) axis = Vector(0, 0, 1);
        const Xform xr = Xform::rotation(axis, rng.next() * 6.283185307179586);
        const Rigid gr = recover_rigid(xr);
        double cx = 0, cy = 0, cz = 0;
        for (const Point& p : B.m_vertices) {
            const Point q = gr.fwd(p);
            cx += q[0]; cy += q[1]; cz += q[2];
        }
        cx /= (double)B.m_vertices.size();
        cy /= (double)B.m_vertices.size();
        cz /= (double)B.m_vertices.size();
        double ax = 0, ay = 0, az = 0;
        for (const Point& p : A.m_vertices) { ax += p[0]; ay += p[1]; az += p[2]; }
        ax /= (double)A.m_vertices.size();
        ay /= (double)A.m_vertices.size();
        az /= (double)A.m_vertices.size();
        const Xform xt = Xform::translation(ax - cx + 0.3 * r * (rng.next() * 2 - 1),
                                            ay - cy + 0.3 * r * (rng.next() * 2 - 1),
                                            az - cz + 0.3 * h * (rng.next() * 2 - 1));
        const Rigid g = compose(recover_rigid(xt), gr);
        apply_motion(B, xr, xt);

        struct Hit { int operand, edge; Point p; };
        std::vector<Hit> hits;
        auto sweep = [&](const BRep& src, int op_src, bool to_b) {
            for (size_t i = 0; i < src.m_topology_edges.size(); ++i) {
                const BRepEdge& e = src.m_topology_edges[i];
                if (e.curve_3d_index < 0) continue;
                const NurbsCurve& c = src.m_curves_3d[e.curve_3d_index];
                const std::pair<double, double> dm = c.domain();
                const int N = 8000;
                double prev = 0;
                bool prev_ok = false;
                for (int k = 0; k <= N; ++k) {
                    const double t = dm.first + (dm.second - dm.first) * (double)k / N;
                    const Point w = c.point_at(t);
                    const Point l = to_b ? g.inv(w) : w;
                    const bool ok = (l[2] > 1e-6 && l[2] < h - 1e-6);
                    const double fv = cyl_lat(l, r);
                    if (k > 0 && prev_ok && ok && prev * fv < 0) {
                        double ta = dm.first + (dm.second - dm.first) * (double)(k - 1) / N;
                        double tb = dm.first + (dm.second - dm.first) * (double)k / N;
                        double fa = prev;
                        for (int it = 0; it < 80; ++it) {
                            const double tm = 0.5 * (ta + tb);
                            const Point wm = c.point_at(tm);
                            const double fm = cyl_lat(to_b ? g.inv(wm) : wm, r);
                            if ((fa < 0) == (fm < 0)) { ta = tm; fa = fm; } else { tb = tm; }
                        }
                        Hit hh;
                        hh.operand = op_src;
                        hh.edge = (int)i;
                        hh.p = c.point_at(0.5 * (ta + tb));
                        hits.push_back(hh);
                    }
                    prev = fv;
                    prev_ok = ok;
                }
            }
        };
        sweep(A, 0, true);
        sweep(B, 1, false);

        BdsArena ds;
        ds.init({&A, &B});
        V2Interf iv(ds, {&A, &B});
        iv.perform_all();

        int found = 0, split = 0;
        for (const Hit& hh : hits) {
            const int ae = ds.index_of_edge(hh.operand, hh.edge);
            for (const V2EFRecord& rec : iv.ef_records()) {
                if (rec.edge != ae || rec.type != V2PartType::Vertex) continue;
                const NurbsCurve* c = ds.edge_curve(rec.edge);
                if (c && d3(c->point_at(rec.t), hh.p) < 1e-5) { ++found; break; }
            }
            std::vector<BdsPave> pv;
            if (ds.has_pave_blocks(ae) && ds.paves(ae, pv))
                for (const BdsPave& p : pv)
                    if (d3(ds.vertex_point(p.vertex), hh.p) < 1e-5) { ++split; break; }
        }
        std::string why;
        if (iv.check_invariants(&why)) ++inv_ok;
        tot_exp += (int)hits.size();
        tot_found += found;
        tot_split += split;
        ++poses;
    }
    cell("P5 EF cylinder_x_cylinder 20rot lateral_piercings",
         tot_found == tot_exp && tot_split == tot_exp && tot_exp > 0,
         sfmt("poses=%d expected=%d found=%d split=%d", poses, tot_exp, tot_found, tot_split));
    cell("P5 EF cylinder_x_cylinder 20rot arena_invariants", inv_ok == poses,
         sfmt("%d/%d", inv_ok, poses));
}

///////////////////////////////////////////////////////////////////////////////////////////
// CELL 6 — rigid-motion invariance of the interference COUNT
///////////////////////////////////////////////////////////////////////////////////////////

static void run_rotation_invariance() {
    Rng rng;
    rng.s = 0xfeedfacecafebeefULL;
    struct Setup { std::string name; BRep A, B; };
    std::vector<Setup> setups;
    {
        BRep a = BRep::create_box(2, 2, 2);
        BRep b = BRep::create_sphere(1.55);
        setups.push_back({"box_x_sphere", a, b});
    }
    {
        // Tip-to-base cones: B is flipped and pushed through A, so the pair really
        // interpenetrates instead of grazing.
        BRep a = BRep::create_cone(1.0, 2.0);
        BRep b = BRep::create_cone(1.0, 2.0);
        Vector xax(1, 0, 0);
        b.xform = Xform::rotation(xax, 3.141592653589793);
        b.transform();
        b.xform = Xform::translation(0.3, 0.0, 1.5);
        b.transform();
        setups.push_back({"cone_x_cone", a, b});
    }
    {
        BRep a = BRep::create_cylinder(1.0, 2.0);
        BRep b = BRep::create_cylinder(1.0, 2.0);
        Vector yax(0, 1, 0);
        b.xform = Xform::rotation(yax, 1.5707963267948966);
        b.transform();
        b.xform = Xform::translation(0.0, 0.0, 1.0);
        b.transform();
        setups.push_back({"cylinder_x_cylinder", a, b});
    }
    for (Setup& s : setups) {
        BdsArena ds0;
        ds0.init({&s.A, &s.B});
        V2Interf i0(ds0, {&s.A, &s.B});
        i0.perform_all();
        const int base = i0.interf_count();
        const int base_ef = i0.stats().ef_transversal + i0.stats().ef_tangential +
                            i0.stats().ef_coincident;
        if (std::getenv("V2DBG")) {
            const V2InterfStats& q = i0.stats();
            std::fprintf(stderr, "[RI] %s BASE n=%d vv=%d ve=%d eev=%d eee=%d vf=%d eft=%d "
                         "efg=%d efc=%d efv=%d onp=%d out=%d\n", s.name.c_str(),
                         i0.interf_count(), q.vv_fused, q.ve_paves, q.ee_vertex_parts,
                         q.ee_edge_parts, q.vf_in, q.ef_transversal, q.ef_tangential,
                         q.ef_coincident, q.ef_new_vertices, q.ef_on_pave, q.ef_rejected_out);
        }
        int ok = 0, worst = 0;
        const int N = 20;
        for (int k = 0; k < N; ++k) {
            BRep A = s.A, B = s.B;
            Vector axis(rng.next() * 2 - 1, rng.next() * 2 - 1, rng.next() * 2 - 1);
            if (axis.magnitude() < 1e-6) axis = Vector(0, 0, 1);
            const Xform xr = Xform::rotation(axis, rng.next() * 6.283185307179586);
            const Xform xt = Xform::translation((rng.next() * 2 - 1) * 3.0,
                                                (rng.next() * 2 - 1) * 3.0,
                                                (rng.next() * 2 - 1) * 3.0);
            apply_motion(A, xr, xt);
            apply_motion(B, xr, xt);
            BdsArena ds;
            ds.init({&A, &B});
            V2Interf iv(ds, {&A, &B});
            iv.perform_all();
            const int ef = iv.stats().ef_transversal + iv.stats().ef_tangential +
                           iv.stats().ef_coincident;
            if (iv.interf_count() == base && ef == base_ef) ++ok;
            worst = std::max(worst, std::abs(iv.interf_count() - base));
            if (std::getenv("V2DBG")) {
                const V2InterfStats& q = iv.stats();
                std::fprintf(stderr, "[RI] %s k=%d n=%d(base %d) vv=%d ve=%d eev=%d eee=%d "
                             "vf=%d eft=%d efg=%d efc=%d efv=%d onp=%d out=%d\n",
                             s.name.c_str(), k, iv.interf_count(), base, q.vv_fused, q.ve_paves,
                             q.ee_vertex_parts, q.ee_edge_parts, q.vf_in, q.ef_transversal,
                             q.ef_tangential, q.ef_coincident, q.ef_new_vertices, q.ef_on_pave,
                             q.ef_rejected_out);
            }
        }
        cell(std::string("P5 rigid_motion_invariant_count ") + s.name, ok == N,
             sfmt("%d/%d base=%d ef=%d worst_delta=%d", ok, N, base, base_ef, worst));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

int main() {
    run_verdict_metric();
    if (!verdict_metric_ok) {
        std::printf("[V2INT] verdict metric NOT validated - refusing to report further numbers\n");
        return 1;
    }
    std::printf("--- stages ---\n");
    run_fusion();
    run_stage_coverage();
    run_tangential();
    run_ef_box_sphere();
    run_ef_cone_cone();
    run_ef_cyl_cyl();
    run_rotation_invariance();
    std::printf("[V2INT] %d/%d PASS\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
