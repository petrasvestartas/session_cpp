// main_13 -- test driver for src/brep_v2_section.{h,cpp}: the v2 FACE/FACE SECTION STAGE.
//
// Every number printed below is measured by the run that prints it. Nothing is asserted against
// another implementation of the same code: the oracles are (a) closed-form analytic values,
// (b) congruence invariants that hold by symmetry and need no reference at all, and
// (c) structural identity invariants (one entity per geometric feature) that are checkable
// without knowing the answer.
//
// STANDING RULE (T0): the verdict metric is validated against the kernel's own validity rule
// BEFORE it is used. Five measurement errors in this codebase came from naked-edge counts that
// disagreed with BRep::is_solid() about poles, apexes and seams, so T0 checks a sphere (pole),
// a cone (apex) and a cylinder (seam) first and refuses to continue if the metric disagrees.
//
//   usage: main_13 [--verbose]
#include "src/brep.h"
#include "src/brep_bds.h"
#include "src/brep_massprops.h"
#include "src/v2/brep_v2_section.h"
#include "src/v2/v2_verdict.h"
#include "src/nurbscurve.h"
#include "src/polyline.h"
#include "src/xform.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace session_cpp;
using namespace session_cpp::v2sec;

namespace {

constexpr double PI = 3.14159265358979323846;

int g_pass = 0, g_fail = 0;
bool g_verbose = false;
double g_seconds = 0.0;

void ok(bool cond, const std::string& what) {
    if (cond) {
        ++g_pass;
        if (g_verbose) std::printf("    ok   %s\n", what.c_str());
    } else {
        ++g_fail;
        std::printf("    FAIL %s\n", what.c_str());
    }
}

std::string fmt(const char* f, ...) {
    char buf[600];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return std::string(buf);
}

///////////////////////////////////////////////////////////////////////////////////////////
// T0 metric: the SHARED verdict harness (src/v2/v2_verdict.h, namespace v2v). No local
// scoring code: five measurement errors in this project came from per-agent metrics that
// disagreed with BRep::is_solid(), so this driver validates the shared one against the
// kernel's own rule on a pole, an apex and a seam and then uses only it.
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// poses
///////////////////////////////////////////////////////////////////////////////////////////

struct Lcg {
    unsigned long long s;
    explicit Lcg(unsigned long long seed) : s(seed) {}
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return (double)((s >> 11) & ((1ULL << 53) - 1)) / (double)(1ULL << 53);
    }
};

Xform random_pose(Lcg& r, double tr_scale) {
    const double z = 2.0 * r.next() - 1.0;
    const double th = 2.0 * PI * r.next();
    const double sr = std::sqrt(std::max(0.0, 1.0 - z * z));
    Vector axis(sr * std::cos(th), sr * std::sin(th), z);
    const double ang = 2.0 * PI * r.next();
    Xform R = Xform::rotation(axis, ang);
    Xform T = Xform::translation(tr_scale * (2 * r.next() - 1), tr_scale * (2 * r.next() - 1),
                                 tr_scale * (2 * r.next() - 1));
    return T * R;
}

BRep moved(const BRep& b, const Xform& x) {
    BRep t = b;
    t.xform = x;
    t.transform();
    return t;
}

///////////////////////////////////////////////////////////////////////////////////////////
// section-stage measurements shared by every case
///////////////////////////////////////////////////////////////////////////////////////////

struct Run {
    int curves = 0, blocks = 0, kept = 0, dropped = 0;
    int edges = 0, nodes = 0, node_locs = 0, comps = 0;
    int seam_paves = 0, trim_paves = 0, fused = 0, common = 0;
    int min_degree = 0, deg1 = 0;
    double length = 0, g1 = 0, max_dev = 0;
    int ok_pairs = 0, tangent = 0, empty = 0, nogeom = 0, failed = 0;
    std::string sig;
};

/// Degree of every section node in the kept-block graph; deg1 counts DANGLING ends (G5).
void node_degrees(const V2Section& S, const BdsArena& ds, int& mind, int& deg1) {
    std::map<int, int> deg;
    for (const V2Curve& c : S.curves())
        for (const V2Block& b : c.blocks)
            if (b.kept) {
                ++deg[ds.resolve_sd(b.v0)];
                ++deg[ds.resolve_sd(b.v1)];
            }
    mind = deg.empty() ? 0 : 1 << 30;
    deg1 = 0;
    for (const std::pair<const int, int>& d : deg) {
        mind = std::min(mind, d.second);
        if (d.second == 1) ++deg1;
    }
    if (deg.empty()) mind = 0;
}

/// kb/port_08 S8: no stored pcurve piece may jump by more than half a period, and a block must
/// not wrap. Measured on the footprint trail restricted to each block.
bool seam_windows_ok(const V2Section& S, double& worst_jump_ratio) {
    worst_jump_ratio = 0.0;
    bool good = true;
    for (const V2Curve& c : S.curves()) {
        const V2UvRect R1 = v2_uv_rect(S.face_ref(c.face1));
        const V2UvRect R2 = v2_uv_rect(S.face_ref(c.face2));
        for (const V2Block& b : c.blocks) {
            if (!b.kept) continue;
            double lo1 = 1e300, hi1 = -1e300, lo2 = 1e300, hi2 = -1e300;
            const V2PntOn2S* prev = nullptr;
            for (const V2PntOn2S& s : c.trail) {
                if (s.t < b.t0 - 1e-12 || s.t > b.t1 + 1e-12) continue;
                lo1 = std::min(lo1, s.u1);
                hi1 = std::max(hi1, s.u1);
                lo2 = std::min(lo2, s.u2);
                hi2 = std::max(hi2, s.u2);
                if (prev) {
                    if (R1.u_period > 0) {
                        const double r = std::abs(s.u1 - prev->u1) / (0.5 * R1.u_period);
                        worst_jump_ratio = std::max(worst_jump_ratio, r);
                        if (r > 1.0) good = false;
                    }
                    if (R2.u_period > 0) {
                        const double r = std::abs(s.u2 - prev->u2) / (0.5 * R2.u_period);
                        worst_jump_ratio = std::max(worst_jump_ratio, r);
                        if (r > 1.0) good = false;
                    }
                }
                prev = &s;
            }
            if (R1.u_period > 0 && hi1 > lo1 && hi1 - lo1 > R1.u_period * (1.0 + 1e-9)) good = false;
            if (R2.u_period > 0 && hi2 > lo2 && hi2 - lo2 > R2.u_period * (1.0 + 1e-9)) good = false;
        }
    }
    return good;
}

Run measure(const V2Section& S, const BdsArena& ds, double node_tol) {
    Run r;
    const V2SectionStats& st = S.stats();
    r.curves = st.curves;
    r.blocks = st.blocks;
    r.kept = st.kept;
    r.dropped = st.dropped;
    r.seam_paves = st.seam_paves;
    r.trim_paves = st.trim_paves;
    r.fused = st.fused_groups;
    r.common = st.common_blocks;
    r.ok_pairs = st.ok;
    r.tangent = st.tangent;
    r.empty = st.empty;
    r.nogeom = st.nogeom;
    r.failed = st.failed;
    r.max_dev = st.max_dev;
    r.edges = (int)S.section_edges().size();
    r.nodes = (int)S.section_nodes().size();
    r.node_locs = S.distinct_node_locations(node_tol);
    r.comps = S.component_count();
    r.length = S.kept_length();
    r.g1 = S.g1_residual();
    node_degrees(S, ds, r.min_degree, r.deg1);
    r.sig = S.signature();
    return r;
}

void print_run(const char* tag, const Run& r) {
    std::printf("    %-22s curves=%d blocks=%d kept=%d edges=%d nodes=%d locs=%d comps=%d "
                "deg1=%d seam=%d fused=%d cb=%d len=%.9f g1=%.2e dev=%.2e\n",
                tag, r.curves, r.blocks, r.kept, r.edges, r.nodes, r.node_locs, r.comps, r.deg1,
                r.seam_paves, r.fused, r.common, r.length, r.g1, r.max_dev);
}

/// Build the arena + run the whole stage for a fixed operand list.
Run run_pair_stage(const std::vector<const BRep*>& ops, bool posttreat, V2Section** keep = nullptr,
                   BdsArena** keep_ds = nullptr) {
    BdsArena* ds = new BdsArena();
    ds->init(ops, 1e-7);
    V2SectionParams prm;
    prm.posttreat = posttreat;
    V2Section* S = new V2Section(*ds, ops, prm);
    S->perform_all();
    Run r = measure(*S, *ds, 1e-9);
    if (keep) *keep = S; else delete S;
    if (keep_ds) *keep_ds = ds; else if (!keep) delete ds;
    return r;
}

/// Per-curve dump used when a case does not meet its gate. Prints what the stage actually
/// produced, so a failure is diagnosed from measurements rather than guessed at.
void dump(const V2Section& S, const BdsArena& ds) {
    for (size_t i = 0; i < S.curves().size(); ++i) {
        const V2Curve& c = S.curves()[i];
        const std::pair<double, double> d = c.c3d->domain();
        // radius/centre of the curve measured from equally-spaced-in-arclength samples
        std::printf("      c%zu f=%d/%d closed=%d dom=[%g,%g] tol=%.2e dev=%.2e/%.2e full_len=%.12f\n",
                    i, c.face1, c.face2, (int)c.closed, d.first, d.second, c.tol, c.dev1, c.dev2,
                    v2sec_arclen(*c.c3d, d.first, d.second));
        for (const V2Pave& p : c.paves)
            std::printf("        pave t=%.12f v=%d %s face=%d edge=%d\n", p.t, p.vertex,
                        v2_origin_name(p.origin), p.face, p.edge);
        for (const V2Block& b : c.blocks) {
            double worst = 0;
            int bad = -1;
            for (int k = 0; k <= 16 && b.kept; ++k) {
                const Point P = c.c3d->point_at(b.t0 + (b.t1 - b.t0) * k / 16.0);
                double u = 0, v = 0;
                const double d1 = V2FaceClassifier(S.face_ref(c.face1)).invert(P, u, v, false);
                double u2 = 0, v2 = 0;
                const double d2 = V2FaceClassifier(S.face_ref(c.face2)).invert(P, u2, v2, false);
                if (std::max(d1, d2) > worst) {
                    worst = std::max(d1, d2);
                    bad = (d1 > d2) ? 1 : 2;
                }
            }
            std::printf("        blk [%.12f,%.12f] kept=%d e=%d v=%d/%d len=%.12f worst=%.2e on S%d\n",
                        b.t0, b.t1, (int)b.kept, b.edge, ds.resolve_sd(b.v0), ds.resolve_sd(b.v1),
                        b.length, worst, bad);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// T0 — verdict metric self-check against the kernel's own validity rule
///////////////////////////////////////////////////////////////////////////////////////////

void t0_metric() {
    std::printf("\nT0  shared verdict harness (v2v::v2_verdict) vs BRep::is_solid()\n");
    struct C { const char* n; BRep b; };
    std::vector<C> cs;
    cs.push_back({"sphere (pole)", BRep::create_sphere(1.0)});
    cs.push_back({"cone   (apex)", BRep::create_cone(1.0, 2.0)});
    cs.push_back({"cylinder(seam)", BRep::create_cylinder(1.0, 2.0)});
    for (C& c : cs) {
        const bool solid = c.b.is_solid();
        const v2v::V2Verdict v = v2v::v2_verdict(c.b);
        std::printf("    %-15s is_solid=%d closed=%d naked_real=%d degen=%d seam=%d clos=%.1e V=%.9f\n",
                    c.n, (int)solid, (int)v.closed(), v.naked_real, v.degenerate, v.seam_edges,
                    v.closure_residual, v.volume);
        ok(solid, fmt("%s is_solid", c.n));
        ok(v.closed() == solid, fmt("%s shared verdict agrees with is_solid()", c.n));
        ok(v.naked_real == 0 && v.nonmanifold == 0,
           fmt("%s manifold by the shared rule (naked=%d nonmanifold=%d)", c.n, v.naked_real,
               v.nonmanifold));
    }
    // The same agreement on BOOLEAN RESULTS that still carry a pole, an apex and a seam.
    struct D { const char* n; BRep b; };
    std::vector<D> ds;
    {
        BRep cy = BRep::create_cylinder(0.5, 6.0);
        cy.xform = Xform::translation(0, 0, -3.0);
        cy.transform();
        ds.push_back({"sphere - cylinder", BRep::create_sphere(2.0).boolean_difference(cy)});
        BRep co = BRep::create_cone(0.8, 3.0);
        co.xform = Xform::translation(0, 0, -1.5);
        co.transform();
        ds.push_back({"box - cone", BRep::create_box(2, 2, 2).boolean_difference(co)});
    }
    for (D& d : ds) {
        const bool solid = d.b.is_solid();
        const v2v::V2Verdict v = v2v::v2_verdict(d.b);
        std::printf("    %-18s faces=%3d is_solid=%d closed=%d naked_real=%d degen=%d clos=%.1e V=%.9f\n",
                    d.n, d.b.face_count(), (int)solid, (int)v.closed(), v.naked_real, v.degenerate,
                    v.closure_residual, v.volume);
        // is_solid() is a TOPOLOGICAL rule; the shared verdict adds the geometric closure
        // certificate on top. The two must agree on the part they both cover -- where they
        // disagree, the extra information is the closure residual, and it is reported, never
        // silently folded into a pass.
        ok((v.naked_real == 0 && v.nonmanifold == 0) == solid,
           fmt("%s shared TOPOLOGY verdict agrees with is_solid()", d.n));
        if (v.closed() != solid)
            std::printf("      note: topologically %s but closure_residual=%.2e -> geometrically "
                        "%s (a class is_solid() cannot see)\n",
                        solid ? "closed" : "open", v.closure_residual,
                        v.closed() ? "closed" : "open");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// T1 — sphere x sphere, 21 poses. Closed-form oracle + pose invariance.
///////////////////////////////////////////////////////////////////////////////////////////

void t1_sphere_sphere() {
    std::printf("\nT1  sphere x sphere, 21 rigid poses (analytic circle r=sqrt(3)/2)\n");
    const BRep A0 = BRep::create_sphere(1.0);
    BRep B0 = BRep::create_sphere(1.0);
    B0.xform = Xform::translation(1, 0, 0);
    B0.transform();
    Lcg rng(20260726ULL);
    Run base;
    bool base_seam_ok = false;
    int poses_ok = 0;
    double worst_r = 0, worst_c = 0, worst_len = 0, worst_g1 = 0;
    for (int p = 0; p <= 20; ++p) {
        const Xform X = (p == 0) ? Xform::identity() : random_pose(rng, 10.0);
        const BRep A = moved(A0, X), B = moved(B0, X);
        BdsArena ds;
        const std::vector<const BRep*> ops = {&A, &B};
        ds.init(ops, 1e-7);
        V2Section S(ds, ops);
        S.perform_all();
        const Run r = measure(S, ds, 1e-9);
        if (p == 0) {
            base = r;
            double jj = 0;
            base_seam_ok = seam_windows_ok(S, jj);
            print_run("pose 0", r);
        }
        // Analytic oracle, used DIRECTLY -- no circle fitting. The trail is not uniform in
        // arclength (a rational conic never is), so a mean of its samples is a biased centre
        // estimate and would test the fitter, not the kernel. Instead: every point of the
        // section must be exactly sqrt(3)/2 from the KNOWN centre X*(0.5,0,0), and the section
        // must lie in the KNOWN plane through it with normal X*(1,0,0).
        bool geom = (r.curves == 1);
        double er = 0, ec = 0;
        if (geom) {
            const V2Curve& c = S.curves()[0];
            Point ctr(0.5, 0, 0);
            ctr.xform = X;
            ctr.transform();
            Point nrm_a(0, 0, 0), nrm_b(1, 0, 0);
            nrm_a.xform = X;
            nrm_a.transform();
            nrm_b.xform = X;
            nrm_b.transform();
            const double nx[3] = {nrm_b[0] - nrm_a[0], nrm_b[1] - nrm_a[1], nrm_b[2] - nrm_a[2]};
            for (const V2PntOn2S& s2 : c.trail) {
                er = std::max(er, std::abs(ctr.distance(s2.p) - std::sqrt(3.0) / 2));
                const double d[3] = {s2.p[0] - ctr[0], s2.p[1] - ctr[1], s2.p[2] - ctr[2]};
                ec = std::max(ec, std::abs(d[0] * nx[0] + d[1] * nx[1] + d[2] * nx[2]));
            }
            worst_r = std::max(worst_r, er);
            worst_c = std::max(worst_c, ec);
            // Gate scaled by the pose's coordinate magnitude: the analytic recogniser
            // upstream re-FITS the quadric instead of carrying its identity
            // (kb/port_04_ff_section.md §4.4 D12), so its error scales with the absolute
            // coordinates a rigid motion introduces. The MEASURED worst value is printed.
            const double gate = 1e-12 * std::max(1.0, std::abs(ctr[0]) + std::abs(ctr[1]) +
                                                          std::abs(ctr[2]));
            geom = er < gate && ec < gate;
        }
        const double el = std::abs(r.length - 2 * PI * std::sqrt(3.0) / 2);
        worst_len = std::max(worst_len, el);
        worst_g1 = std::max(worst_g1, r.g1);
        const bool inv = r.curves == base.curves && r.blocks == base.blocks &&
                         r.kept == base.kept && r.edges == base.edges && r.comps == base.comps &&
                         r.nodes == base.nodes;
        if (geom && inv && el < 1e-9 && r.g1 < 1e-9 && r.deg1 == 0 && r.nodes == r.node_locs)
            ++poses_ok;
        else if (g_verbose)
            print_run(fmt("pose %d MISMATCH", p).c_str(), r);
    }
    std::printf("    poses fully correct: %d / 21   worst |r-sqrt3/2|=%.2e |plane|=%.2e "
                "|dlen|=%.2e g1=%.2e\n", poses_ok, worst_r, worst_c, worst_len, worst_g1);
    ok(base.curves == 1, fmt("exactly ONE section curve (got %d)", base.curves));
    ok(base.kept == base.blocks, fmt("every block survives the trim test (%d/%d)", base.kept, base.blocks));
    ok(base.nodes == base.node_locs,
       fmt("one entity per node location (%d entities / %d locations)", base.nodes, base.node_locs));
    ok(base.deg1 == 0, fmt("no dangling section end (deg1=%d)", base.deg1));
    ok(base_seam_ok, "every block inside one seam window at pose 0");
    ok(poses_ok == 21, fmt("all 21 poses exact and pose-invariant (got %d)", poses_ok));
}

///////////////////////////////////////////////////////////////////////////////////////////
// T2 — sphere x cylinder, cylinder axis through the sphere centre, 9 tilts.
// The configuration is congruent to itself at every tilt (the sphere is rotationally
// symmetric about its centre), so curve count, component count and TOTAL ARCLENGTH are
// tilt-invariant by symmetry. That is an oracle-free assertion of the invariance the brief
// requires, and it does not depend on the seam landing anywhere in particular.
///////////////////////////////////////////////////////////////////////////////////////////

void t2_sphere_cylinder() {
    std::printf("\nT2  sphere R=2 x cylinder R=1 (axis through the centre), tilts 0..45 deg\n");
    const double tilts[] = {0.0, 0.01, 0.1, 0.3, 1.0, 5.0, 15.0, 30.0, 45.0};
    const BRep A = BRep::create_sphere(2.0);
    BRep C0 = BRep::create_cylinder(1.0, 6.0);
    C0.xform = Xform::translation(0, 0, -3.0);
    C0.transform();
    const double want_len = 2.0 * (2.0 * PI * 1.0);   // two circles of radius 1 at z = +-sqrt(3)
    Run base;
    int okc = 0;
    double worst_dl = 0, worst_g1 = 0;
    for (int i = 0; i < 9; ++i) {
        Vector axis(1, 1, 0);
        BRep C = moved(C0, Xform::rotation(axis, tilts[i] * PI / 180.0));
        BdsArena ds;
        const std::vector<const BRep*> ops = {&A, &C};
        ds.init(ops, 1e-7);
        V2Section S(ds, ops);
        S.perform_all();
        const Run r = measure(S, ds, 1e-9);
        double jump = 0;
        const bool seam_ok = seam_windows_ok(S, jump);
        if (i == 0) base = r;
        std::printf("    tilt %6.2f  curves=%d kept=%d comps=%d nodes=%d locs=%d deg1=%d seam=%d "
                    "len=%.9f g1=%.1e seamwin=%d\n",
                    tilts[i], r.curves, r.kept, r.comps, r.nodes, r.node_locs, r.deg1,
                    r.seam_paves, r.length, r.g1, (int)seam_ok);
        const double dl = std::abs(r.length - want_len);
        worst_dl = std::max(worst_dl, dl);
        worst_g1 = std::max(worst_g1, r.g1);
        const bool inv = r.curves == base.curves && r.comps == base.comps && r.deg1 == 0 &&
                         r.nodes == r.node_locs;
        if (inv && dl < 1e-7 && r.g1 < 1e-8 && seam_ok) ++okc;
        else if (g_verbose) dump(S, ds);
    }
    std::printf("    tilt-invariant + analytic-length tilts: %d / 9  worst |dlen|=%.2e g1=%.2e\n",
                okc, worst_dl, worst_g1);
    ok(base.curves == 2, fmt("two section curves at tilt 0 (got %d)", base.curves));
    ok(okc == 9, fmt("all 9 tilts tilt-invariant and analytically exact (got %d)", okc));
}

///////////////////////////////////////////////////////////////////////////////////////////
// T3 — the 45-degree sphere x cylinder case the corpus reports as the smallest genuine
// curved failure. Section-stage verdict + the v1 kernel's own downstream verdict for context.
///////////////////////////////////////////////////////////////////////////////////////////

void t3_forty_five() {
    std::printf("\nT3  sphere x cylinder at 45 deg -- section-stage verdict, and v1 downstream\n");
    const BRep A = BRep::create_sphere(2.0);
    BRep C0 = BRep::create_cylinder(1.0, 6.0);
    C0.xform = Xform::translation(0, 0, -3.0);
    C0.transform();
    Vector axis(1, 1, 0);
    const BRep C = moved(C0, Xform::rotation(axis, 45.0 * PI / 180.0));
    BdsArena ds;
    const std::vector<const BRep*> ops = {&A, &C};
    ds.init(ops, 1e-7);
    V2Section S(ds, ops);
    S.perform_all();
    const Run r = measure(S, ds, 1e-9);
    print_run("45 deg section", r);
    double jump = 0;
    const bool seam_ok = seam_windows_ok(S, jump);
    ok(r.curves == 2, fmt("two section curves (got %d)", r.curves));
    ok(r.comps == 2, fmt("two closed section components (got %d)", r.comps));
    ok(r.deg1 == 0, fmt("no dangling section end (deg1=%d)", r.deg1));
    ok(r.nodes == r.node_locs,
       fmt("one entity per node location (%d/%d)", r.nodes, r.node_locs));
    if (g_verbose) dump(S, ds);
    ok(std::abs(r.length - 4 * PI) < 1e-7, fmt("total arclength 4*pi (err %.2e)", std::abs(r.length - 4 * PI)));
    ok(r.g1 < 1e-8, fmt("G1 on both surfaces (%.2e)", r.g1));
    ok(seam_ok, fmt("every block inside one seam window (worst jump ratio %.3f)", jump));

    // context only, not a gate: what the v1 boolean does with the same inputs.
    const BRep cut = A.boolean_difference(C);
    const v2v::V2Verdict vv = v2v::v2_verdict(cut);
    std::printf("    [context] v1 cut: %s\n", vv.str().c_str());
}

///////////////////////////////////////////////////////////////////////////////////////////
// T4 — box x sphere. Analytic per-face expectation AND the cross-pair node fusion test.
///////////////////////////////////////////////////////////////////////////////////////////

void t4_box_sphere() {
    std::printf("\nT4  box [-1,1]^3 x sphere R=1.5 -- per-face arcs and cross-pair node fusion\n");
    const BRep Bx = BRep::create_box(2, 2, 2);
    const BRep Sp = BRep::create_sphere(1.5);
    const std::vector<const BRep*> ops = {&Bx, &Sp};

    BdsArena ds_no;
    ds_no.init(ops, 1e-7);
    V2SectionParams pno;
    pno.posttreat = false;
    V2Section Sno(ds_no, ops, pno);
    Sno.perform_all();
    const Run rno = measure(Sno, ds_no, 1e-9);
    print_run("no fusion", rno);

    BdsArena ds;
    ds.init(ops, 1e-7);
    V2Section S(ds, ops);
    S.perform_all();
    const Run r = measure(S, ds, 1e-9);
    print_run("with fusion", r);

    // Analytic: each of the 6 face planes (|x|=1 etc.) cuts the sphere in a circle of radius
    // sqrt(1.5^2-1^2)=1.11803..., which leaves the 2x2 square through all four sides, so four
    // arcs per face survive; the sphere meets each of the 12 box edges twice.
    const double arc_r = std::sqrt(1.5 * 1.5 - 1.0);
    // half-angle of one kept arc: the crossings are at |y| = sqrt(arc_r^2-1) on |x|=1
    const double th = std::atan2(std::sqrt(arc_r * arc_r - 1.0), 1.0);
    const double want_len = 6.0 * 4.0 * arc_r * (PI / 2 - 2 * th);
    ok(r.curves == 6, fmt("six section circles, one per box face (got %d)", r.curves));
    ok(r.kept == 24, fmt("24 surviving arcs, 4 per face (got %d)", r.kept));
    ok(std::abs(r.length - want_len) < 1e-7,
       fmt("total arclength %.9f, analytic %.9f (err %.2e)", r.length, want_len,
           std::abs(r.length - want_len)));
    ok(r.comps == 8, fmt("8 closed section loops, one per box corner (got %d)", r.comps));
    ok(r.deg1 == 0 && r.min_degree == 2, fmt("every node has degree 2 (min=%d deg1=%d)", r.min_degree, r.deg1));
    ok(r.nodes == 24, fmt("24 shared section nodes after fusion (got %d)", r.nodes));
    ok(r.nodes == r.node_locs,
       fmt("ONE entity per node location (%d entities / %d locations)", r.nodes, r.node_locs));
    ok(rno.nodes > r.nodes,
       fmt("fusion actually collapsed something (%d nodes -> %d)", rno.nodes, r.nodes));
    ok(rno.node_locs < rno.nodes,
       fmt("without fusion the same location IS duplicated (%d entities / %d locations)",
           rno.nodes, rno.node_locs));
    ok(r.g1 < 1e-9, fmt("G1 on both surfaces (%.2e)", r.g1));

    // G11 determinism: the whole stage twice, byte-identical.
    BdsArena ds2;
    ds2.init(ops, 1e-7);
    V2Section S2(ds2, ops);
    S2.perform_all();
    ok(S2.signature() == S.signature(), "determinism: identical signature on a second run");
    double jump = 0;
    ok(seam_windows_ok(S, jump), fmt("every block inside one seam window (worst ratio %.3f)", jump));
}

///////////////////////////////////////////////////////////////////////////////////////////
// T5 — box x cylinder: seam crossing on the cylinder, analytic piece count.
///////////////////////////////////////////////////////////////////////////////////////////

void t5_box_cylinder() {
    std::printf("\nT5  box [-1,1]^3 x cylinder R=0.6 axis Z -- cylinder seam crossing\n");
    const BRep Bx = BRep::create_box(2, 2, 2);
    BRep Cy = BRep::create_cylinder(0.6, 4.0);
    Cy.xform = Xform::translation(0, 0, -2.0);
    Cy.transform();
    const std::vector<const BRep*> ops = {&Bx, &Cy};
    BdsArena ds;
    ds.init(ops, 1e-7);
    V2Section S(ds, ops);
    S.perform_all();
    const Run r = measure(S, ds, 1e-9);
    print_run("box x cylinder", r);
    double jump = 0;
    const bool seam_ok = seam_windows_ok(S, jump);
    ok(r.curves == 2, fmt("two section circles, on z=+-1 (got %d)", r.curves));
    if (g_verbose) dump(S, ds);
    ok(std::abs(r.length - 2 * 2 * PI * 0.6) < 1e-8,
       fmt("total arclength 2*2*pi*0.6 (err %.2e)", std::abs(r.length - 2 * 2 * PI * 0.6)));
    ok(r.comps == 2, fmt("two closed loops (got %d)", r.comps));
    // A section that reaches the seam EXACTLY at its own start parameter needs no extra pave:
    // its closing pave already sits on the seam and its single block spans one period from
    // seam to seam. So the assertion is on the INVARIANT (no block wraps), plus the fact that
    // at least one circle here does need a real seam pave.
    ok(r.seam_paves >= 1, fmt("the seam crossing is paved where it is interior (got %d)", r.seam_paves));
    ok(seam_ok, fmt("every block inside one seam window (worst jump ratio %.3f)", jump));
    ok(r.deg1 == 0, fmt("no dangling end (deg1=%d)", r.deg1));
    ok(r.nodes == r.node_locs, fmt("one entity per node location (%d/%d)", r.nodes, r.node_locs));
}

///////////////////////////////////////////////////////////////////////////////////////////
// T5b — SPHERE seam crossing. A section that WRAPS around the polar axis must cross the
// sphere's seam edge exactly once, be paved there, and leave two blocks whose pcurves each
// live inside one period (kb/port_08_seam_pole_periodic.md S8/S9).
///////////////////////////////////////////////////////////////////////////////////////////

void t5b_sphere_seam() {
    std::printf("\nT5b sphere R=1 x box face at z=0.5 -- sphere seam crossing\n");
    // The sphere is spun about its OWN axis: identical geometry, seam moved off the point
    // where the analytic circle happens to start, so the crossing is a genuine interior pave.
    BRep Sp = BRep::create_sphere(1.0);
    Sp.xform = Xform::rotation_z(0.7);
    Sp.transform();
    BRep Bx = BRep::create_box(4, 4, 3);
    Bx.xform = Xform::translation(0, 0, 2.0);
    Bx.transform();
    const std::vector<const BRep*> ops = {&Sp, &Bx};
    BdsArena ds;
    ds.init(ops, 1e-7);
    V2Section S(ds, ops);
    S.perform_all();
    const Run r = measure(S, ds, 1e-9);
    print_run("sphere x box face", r);
    if (g_verbose) dump(S, ds);
    double jump = 0;
    const bool seam_ok = seam_windows_ok(S, jump);
    // the only cut is the plane z=0.5: a latitude circle of radius sqrt(3)/2
    const double want = 2 * PI * std::sqrt(0.75);
    int sphere_seam = 0;
    for (const V2Curve& c : S.curves())
        for (const V2Pave& p : c.paves)
            if (p.origin == V2PaveOrigin::SeamCrossing && p.face == c.face1) ++sphere_seam;
    ok(r.curves == 1, fmt("one section circle (got %d)", r.curves));
    ok(sphere_seam == 1, fmt("exactly one SPHERE-seam pave (got %d)", sphere_seam));
    ok(r.kept == 2, fmt("the wrap is delivered as two blocks (got %d)", r.kept));
    ok(r.comps == 1, fmt("still one closed loop (got %d)", r.comps));
    ok(std::abs(r.length - want) < 1e-9,
       fmt("arclength 2*pi*sqrt(3)/2 (err %.2e)", std::abs(r.length - want)));
    ok(seam_ok, fmt("both blocks inside one seam window (worst jump ratio %.3f)", jump));
    ok(r.deg1 == 0, fmt("no dangling end (deg1=%d)", r.deg1));
}

///////////////////////////////////////////////////////////////////////////////////////////
// T6 — box x cone. (a) a configuration with a closed-form answer; (b) the hard pose where the
// cone pokes through the sides, scored structurally at 9 rigid poses of BOTH operands.
///////////////////////////////////////////////////////////////////////////////////////////

void t6_box_cone() {
    std::printf("\nT6  box x cone\n");
    {
        const BRep Bx = BRep::create_box(2, 2, 2);
        BRep Co = BRep::create_cone(0.8, 3.0);
        Co.xform = Xform::translation(0, 0, -1.5);
        Co.transform();
        const std::vector<const BRep*> ops = {&Bx, &Co};
        BdsArena ds;
        ds.init(ops, 1e-7);
        V2Section S(ds, ops);
        S.perform_all();
        const Run r = measure(S, ds, 1e-9);
        print_run("(a) cone inside", r);
        // radii of the two circles: r(z) = 0.8*(z+1.5)/3 at z = -1 and z = +1
        const double r1 = 0.8 * (0.5 / 3.0), r2 = 0.8 * (2.5 / 3.0);
        const double want = 2 * PI * (r1 + r2);
        ok(r.curves == 2, fmt("two section circles (got %d)", r.curves));
        ok(std::abs(r.length - want) < 1e-8,
           fmt("total arclength %.9f, analytic %.9f (err %.2e)", r.length, want,
               std::abs(r.length - want)));
        ok(r.comps == 2, fmt("two closed loops (got %d)", r.comps));
        ok(r.deg1 == 0, fmt("no dangling end (deg1=%d)", r.deg1));
        ok(r.g1 < 1e-9, fmt("G1 (%.2e)", r.g1));
    }
    {
        std::printf("    (b) cone poking through the sides, 9 rigid poses of BOTH operands\n");
        const BRep Bx0 = BRep::create_box(2, 2, 2);
        BRep Co0 = BRep::create_cone(1.5, 3.0);
        Co0.xform = Xform::translation(0, 0, -1.5);
        Co0.transform();
        Lcg rng(777ULL);
        Run base;
        int okc = 0;
        double worst_dl = 0, worst_g1 = 0;
        for (int p = 0; p < 9; ++p) {
            const Xform X = (p == 0) ? Xform::identity() : random_pose(rng, 8.0);
            const BRep Bx = moved(Bx0, X), Co = moved(Co0, X);
            const std::vector<const BRep*> ops = {&Bx, &Co};
            BdsArena ds;
            ds.init(ops, 1e-7);
            V2Section S(ds, ops);
            S.perform_all();
            const Run r = measure(S, ds, 1e-9);
            if (p == 0) {
                base = r;
                print_run("      pose 0", r);
            }
            const double dl = std::abs(r.length - base.length);
            worst_dl = std::max(worst_dl, dl);
            worst_g1 = std::max(worst_g1, r.g1);
            // Pose invariants are the GEOMETRIC ones: how many sections there are, how they
            // connect, and how long they are. The number of BLOCKS is not one of them -- a
            // rigid motion moves the operands relative to each other's seams, and whether a
            // section crosses a seam is a property of that relative pose, not of the section.
            const bool inv = r.curves == base.curves && r.comps == base.comps;
            if (inv && dl < 1e-7 && r.deg1 == 0 && r.nodes == r.node_locs && r.g1 < 1e-7) ++okc;
            else if (g_verbose) { print_run(fmt("      pose %d MISMATCH", p).c_str(), r); dump(S, ds); }
        }
        std::printf("      pose-invariant + structurally sound poses: %d / 9  worst |dlen|=%.2e "
                    "g1=%.2e\n", okc, worst_dl, worst_g1);
        ok(base.deg1 == 0, fmt("no dangling end at pose 0 (deg1=%d)", base.deg1));
        ok(okc == 9, fmt("all 9 poses invariant and sound (got %d)", okc));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// T7 — cross-pair EDGE fusion (Law 1). The SAME geometric curve arrives from two different
// face pairs; after post_treat_ff it must be ONE arena entity, not two.
///////////////////////////////////////////////////////////////////////////////////////////

void t7_cross_pair_edges() {
    std::printf("\nT7  cross-pair EDGE fusion: one curve from two face pairs -> one arena edge\n");
    const BRep Sp = BRep::create_sphere(1.5);
    const BRep B1 = BRep::create_box(2, 2, 2);
    const BRep B2 = BRep::create_box(2, 2, 2);   // a second, geometrically IDENTICAL operand
    const std::vector<const BRep*> ops = {&Sp, &B1, &B2};

    for (int fuse = 0; fuse < 2; ++fuse) {
        BdsArena ds;
        ds.init(ops, 1e-7);
        V2SectionParams prm;
        prm.posttreat = (fuse == 1);
        V2Section S(ds, ops, prm);
        // the sphere face against every face of BOTH boxes
        std::vector<int> sf, bf;
        for (int i = 0; i < ds.nb_shapes(); ++i) {
            if (!ds.is_face(i)) continue;
            if (ds.rank(i) == 0) sf.push_back(i);
            else bf.push_back(i);
        }
        for (int a : sf)
            for (int b : bf) S.perform_pair(a, b);
        if (prm.posttreat) S.post_treat_ff();
        const Run r = measure(S, ds, 1e-9);
        print_run(fuse ? "with fusion" : "no fusion", r);
        if (fuse == 0) {
            ok(r.kept == 48, fmt("48 arcs before fusion, 24 from each identical box (got %d)", r.kept));
            ok(r.edges == 48, fmt("48 distinct arena edges before fusion (got %d)", r.edges));
        } else {
            ok(r.kept == 48, fmt("still 48 blocks after fusion (got %d)", r.kept));
            ok(r.edges == 24, fmt("ONE arena edge per geometric curve: 24 (got %d)", r.edges));
            ok(r.common == 24, fmt("24 common blocks built (got %d)", r.common));
            ok(r.nodes == 24, fmt("24 shared nodes (got %d)", r.nodes));
            ok(r.nodes == r.node_locs, fmt("one entity per node location (%d/%d)", r.nodes, r.node_locs));
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// T8 — typed degeneracy (G9): a tangency is never reported as "Ok with 0 curves".
///////////////////////////////////////////////////////////////////////////////////////////

void t8_typed() {
    std::printf("\nT8  typed outcomes: tangency, empty, no-geometric-solution\n");
    // (a) two spheres that touch externally at one point -> not a section curve
    const BRep A = BRep::create_sphere(1.0);
    BRep B0 = BRep::create_sphere(1.0);
    B0.xform = Xform::translation(2.0, 0, 0);
    B0.transform();
    {
        BdsArena ds;
        const std::vector<const BRep*> ops = {&A, &B0};
        ds.init(ops, 1e-7);
        V2Section S(ds, ops);
        S.perform_all();
        std::printf("    tangent spheres: ok=%d tangent=%d nogeom=%d empty=%d curves=%d\n",
                    S.stats().ok, S.stats().tangent, S.stats().nogeom, S.stats().empty,
                    S.stats().curves);
        ok(S.stats().curves == 0, "no phantom curve at an external tangency");
        ok(S.stats().ok == 0, "the pair is NOT reported Ok");
        ok(S.stats().tangent + S.stats().nogeom == 1, "the pair carries a typed non-Ok outcome");
    }
    // (b) two far-apart solids -> Empty, decided by the boxes, no SSI call at all
    {
        BRep C = BRep::create_sphere(1.0);
        C.xform = Xform::translation(100, 0, 0);
        C.transform();
        BdsArena ds;
        const std::vector<const BRep*> ops = {&A, &C};
        ds.init(ops, 1e-7);
        V2Section S(ds, ops);
        S.perform_all();
        ok(S.stats().empty == 1 && S.stats().pairs == 1,
           fmt("disjoint boxes give Empty (empty=%d pairs=%d)", S.stats().empty, S.stats().pairs));
        ok(S.stats().curves == 0, "no curve from a disjoint pair");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// T9 — G8: the curve tolerance is MEASURED and covers its own footprints.
///////////////////////////////////////////////////////////////////////////////////////////

void t9_measured_tolerance() {
    std::printf("\nT9  G8: measured curve tolerance covers the footprints on both surfaces\n");
    const BRep Bx = BRep::create_box(2, 2, 2);
    const BRep Sp = BRep::create_sphere(1.5);
    const std::vector<const BRep*> ops = {&Bx, &Sp};
    BdsArena ds;
    ds.init(ops, 1e-7);
    V2Section S(ds, ops);
    S.perform_all();
    int bad = 0;
    double worst = 0;
    for (const V2Curve& c : S.curves()) {
        worst = std::max(worst, std::max(c.dev1, c.dev2));
        if (c.tol < std::max(c.dev1, c.dev2)) ++bad;
    }
    std::printf("    curves=%zu worst measured deviation=%.3e\n", S.curves().size(), worst);
    ok(bad == 0, fmt("every curve tolerance covers its own measured deviation (%d violations)", bad));
    ok(worst < 1e-12, fmt("analytic sections sit on both surfaces to 1e-12 (worst %.2e)", worst));
}

///////////////////////////////////////////////////////////////////////////////////////////
// T10 — pave provenance (kb/port_04 G3/T9). Every block endpoint must name a real entity:
// a bound of the curve, the closing pave of a closed curve, or a 3D intersection with a
// named EDGE of one of the two faces. There is no `bisection` origin in the enum, so this
// assertion is structural -- but it is checked anyway, on every curve of every case.
///////////////////////////////////////////////////////////////////////////////////////////

void t10_provenance() {
    std::printf("\nT10 pave provenance: every block endpoint names a real entity\n");
    struct Case { const char* n; BRep a; BRep b; };
    std::vector<Case> cases;
    {
        BRep cy = BRep::create_cylinder(0.6, 4.0);
        cy.xform = Xform::translation(0, 0, -2.0);
        cy.transform();
        cases.push_back({"box x cylinder", BRep::create_box(2, 2, 2), cy});
    }
    cases.push_back({"box x sphere", BRep::create_box(2, 2, 2), BRep::create_sphere(1.5)});
    {
        BRep co = BRep::create_cone(1.5, 3.0);
        co.xform = Xform::translation(0, 0, -1.5);
        co.transform();
        cases.push_back({"box x cone", BRep::create_box(2, 2, 2), co});
    }
    int total = 0, bad = 0;
    std::map<std::string, int> hist;
    for (Case& c : cases) {
        const std::vector<const BRep*> ops = {&c.a, &c.b};
        BdsArena ds;
        ds.init(ops, 1e-7);
        V2Section S(ds, ops);
        S.perform_all();
        for (const V2Curve& cv : S.curves())
            for (const V2Pave& p : cv.paves) {
                ++total;
                ++hist[v2_origin_name(p.origin)];
                const bool bound = p.origin == V2PaveOrigin::CurveBound ||
                                   p.origin == V2PaveOrigin::Closing;
                const bool entity = (p.origin == V2PaveOrigin::TrimCrossing ||
                                     p.origin == V2PaveOrigin::SeamCrossing) &&
                                    p.face >= 0 && p.edge >= 0;
                if (!bound && !entity) ++bad;
            }
    }
    std::printf("    %d paves:", total);
    for (const std::pair<const std::string, int>& h : hist)
        std::printf(" %s=%d", h.first.c_str(), h.second);
    std::printf("\n");
    ok(total > 0, "paves were produced at all");
    ok(bad == 0, fmt("every pave names a real entity (%d untraceable)", bad));
    ok(hist.count("trim_crossing") + hist.count("seam_crossing") > 0,
       "trim and/or seam crossings are represented as paves");
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i)
        if (!std::strcmp(argv[i], "--verbose")) g_verbose = true;
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    t0_metric();
    if (g_fail) {
        std::printf("\n=== ABORT: the verdict metric disagrees with BRep::is_solid(); "
                    "no further measurement is trustworthy ===\n");
        return 1;
    }
    t1_sphere_sphere();
    t2_sphere_cylinder();
    t3_forty_five();
    t4_box_sphere();
    t5_box_cylinder();
    t5b_sphere_seam();
    t6_box_cone();
    t7_cross_pair_edges();
    t8_typed();
    t9_measured_tolerance();
    t10_provenance();

    g_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("\n=== RESULT: %d passed, %d failed, %.2f s ===\n", g_pass, g_fail, g_seconds);
    return g_fail == 0 ? 0 : 1;
}
