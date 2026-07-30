// main_17 — THE VALIDATION GATE for the shared verdict harness (src/v2/v2_verdict.h).
//
// The harness may not score anything until this driver passes. Five measurement errors in one
// session, three of which inverted a headline conclusion, came from verdict metrics that
// disagreed with BRep::is_solid() on un-split primitives -- a metric that calls an intact
// sphere "open" turns a working boolean into a reported failure.
//
// PART 1  AGREEMENT. Five un-split solids: box (no degeneracy), sphere (poles + seam), cone
//         (apex + seam), cylinder (seam), torus (two seams). For each:
//         v2_verdict(b).closed() == b.is_solid(), naked_real == 0, nonmanifold == 0.
// PART 2  VOLUME. The same five against their closed-form volumes (the harness reports
//         brep_massprops' number; if the plumbing is wrong this is where it shows).
// PART 3  INVARIANCE. Each shape under 10 arbitrary rigid motions: every count identical,
//         volume identical to 1e-9 relative, still closed. A verdict that moves with pose is
//         not a verdict.
// PART 4  THE NEGATIVE CONTROL. A shell with one face removed must report naked_real > 0 and
//         closed() == false. A metric that cannot fail proves nothing.

#include "src/brep.h"
#include "src/nurbscurve.h"
#include "src/v2/v2_verdict.h"
#include "src/vector.h"
#include "src/xform.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

using namespace session_cpp;
using session_cpp::v2v::V2Verdict;
using session_cpp::v2v::v2_verdict;

static int g_pass = 0, g_fail = 0;

static void cell(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("[V17] %-52s %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL",
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
// the five shapes and their closed-form volumes
///////////////////////////////////////////////////////////////////////////////////////////

struct Shape {
    const char* name;
    BRep b;
    double volume;
};

static const double kPi = 3.14159265358979323846;

static std::vector<Shape> shapes() {
    std::vector<Shape> s;
    // create_box takes FULL extents (it halves them internally).
    s.push_back({"box 2x2x2", BRep::create_box(2.0, 2.0, 2.0), 8.0});
    s.push_back({"sphere r=1 (2 poles + seam)", BRep::create_sphere(1.0), 4.0 / 3.0 * kPi});
    s.push_back({"cone r=1 h=2 (apex + seam)", BRep::create_cone(1.0, 2.0), kPi * 2.0 / 3.0});
    s.push_back({"cylinder r=1 h=2 (seam)", BRep::create_cylinder(1.0, 2.0), kPi * 2.0});
    s.push_back({"torus R=3 r=1 (two seams)", BRep::create_torus(3.0, 1.0),
                 2.0 * kPi * kPi * 3.0 * 1.0});
    return s;
}

/// Deterministic rigid motion k: axis/angle rotation followed by a translation.
static Xform motion(int k) {
    const double a = 0.7548776662 * k, b = 0.5698402909 * k, c = 0.3141592653 * k;
    Vector axis(std::sin(a * 3.1) + 0.31, std::cos(b * 2.7) - 0.17, std::sin(c * 1.9) + 0.53);
    const double L = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis = Vector(axis[0] / L, axis[1] / L, axis[2] / L);
    const double ang = 0.13 + 3.0 * std::fabs(std::sin(1.7 * k));
    const Xform r = Xform::rotation(axis, ang, false);
    const Xform t = Xform::translation(0.37 * std::sin(2.1 * k), -0.29 * std::cos(1.3 * k),
                                       0.44 * std::sin(0.9 * k));
    return t * r;
}

static BRep moved(const BRep& b, const Xform& x) {
    BRep c = b;
    c.xform = x;
    return c.transformed();
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 1 — agreement with is_solid()
///////////////////////////////////////////////////////////////////////////////////////////

static void part1_agreement() {
    std::printf("\n=== PART 1  v2_verdict().closed() vs BRep::is_solid() ===\n");
    bool all = true;
    for (Shape& s : shapes()) {
        const V2Verdict v = v2_verdict(s.b);
        const bool is_sol = s.b.is_solid();
        const bool ok = (v.closed() == is_sol) && is_sol && v.naked_real == 0 && v.nonmanifold == 0;
        std::printf("[V17]   %-28s is_solid=%d  %s  %s\n", s.name, (int)is_sol, v.str().c_str(),
                    ok ? "AGREE" : "DISAGREE");
        if (!ok) all = false;
    }
    cell("agreement_with_is_solid_on_five_solids", all,
         "degenerate poles/apexes excluded; seam edges have two uses, not naked");

    // The five primitives above carry NO zero-length edge (their poles and apexes are pcurve
    // singularities with no 3D edge record), so they do not exercise the degeneracy branch --
    // the branch whose absence produced this campaign's errors. Force it: take the open box
    // from PART 4 and collapse its four naked edges to points. is_solid() then skips them and
    // reports solid; naked_real must do exactly the same. closed() stays FALSE because the
    // closure certificate still sees the hole -- the geometric gate is strictly stronger than
    // is_solid(), which is the point of carrying it.
    {
        BRep open = BRep::create_box(2.0, 2.0, 2.0).subset({0, 1, 2, 3, 4});
        const V2Verdict before = v2_verdict(open);
        int collapsed = 0;
        for (const BRepEdge& e : open.m_topology_edges) {
            if (e.trim_indices.size() != 1) continue;
            const int ci = e.curve_3d_index;
            if (ci < 0 || ci >= (int)open.m_curves_3d.size()) continue;
            const Point p = open.m_curves_3d[(size_t)ci].point_at(
                open.m_curves_3d[(size_t)ci].domain().first);
            open.m_curves_3d[(size_t)ci] = NurbsCurve::create(false, 1, {p, p});
            ++collapsed;
        }
        const V2Verdict after = v2_verdict(open);
        const bool ok = before.naked_real == 4 && before.degenerate == 0 && collapsed == 4 &&
                        after.naked_real == 0 && after.degenerate == 4 && open.is_solid() &&
                        !after.closed();
        cell("degenerate_edges_excluded_exactly_as_is_solid", ok,
             sfmt("before{naked=%d degen=%d} after{naked=%d degen=%d} is_solid=%d closed=%d "
                  "resid=%.2e",
                  before.naked_real, before.degenerate, after.naked_real, after.degenerate,
                  (int)open.is_solid(), (int)after.closed(), after.closure_residual));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 2 — volume against the closed form
///////////////////////////////////////////////////////////////////////////////////////////

static void part2_volume() {
    std::printf("\n=== PART 2  volume vs closed form ===\n");
    bool all = true;
    for (Shape& s : shapes()) {
        const V2Verdict v = v2_verdict(s.b);
        const double rel = std::fabs(v.volume - s.volume) / std::fabs(s.volume);
        const bool ok = v.volume_valid && rel < 1e-9;
        std::printf("[V17]   %-28s vol=%.12f exact=%.12f rel=%.2e valid=%d %s\n", s.name, v.volume,
                    s.volume, rel, (int)v.volume_valid, ok ? "OK" : "BAD");
        if (!ok) all = false;
    }
    cell("volume_matches_closed_form", all, "brep_massprops through the harness");
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 3 — invariance under 10 rigid motions
///////////////////////////////////////////////////////////////////////////////////////////

static void part3_invariance() {
    std::printf("\n=== PART 3  invariance under 10 rigid motions ===\n");
    const int N = 10;
    bool all = true;
    for (Shape& s : shapes()) {
        const V2Verdict v0 = v2_verdict(s.b);
        int bad = 0;
        double worst_rel = 0.0, worst_resid = v0.closure_residual;
        for (int k = 1; k <= N; ++k) {
            const V2Verdict v = v2_verdict(moved(s.b, motion(k)));
            const double rel = std::fabs(v.volume - v0.volume) / std::max(1e-300, std::fabs(v0.volume));
            worst_rel = std::max(worst_rel, rel);
            worst_resid = std::max(worst_resid, v.closure_residual);
            const bool same = v.faces == v0.faces && v.shells == v0.shells &&
                              v.solids == v0.solids && v.naked_real == v0.naked_real &&
                              v.nonmanifold == v0.nonmanifold && v.seam_edges == v0.seam_edges &&
                              v.degenerate == v0.degenerate && v.closed() == v0.closed() &&
                              v.closed() && rel < 1e-9;
            if (!same) {
                ++bad;
                if (bad == 1)
                    std::printf("[V17]     %s motion %d: %s (rel=%.2e)\n", s.name, k, v.str().c_str(),
                                rel);
            }
        }
        std::printf("[V17]   %-28s %2d/%2d invariant  worst_rel=%.2e worst_resid=%.2e\n", s.name,
                    N - bad, N, worst_rel, worst_resid);
        if (bad) all = false;
    }
    cell("verdict_invariant_under_rigid_motion", all, sfmt("5 shapes x %d motions", N));
}

///////////////////////////////////////////////////////////////////////////////////////////
// PART 4 — the negative control: a shell with one face removed
///////////////////////////////////////////////////////////////////////////////////////////

static void part4_broken() {
    std::printf("\n=== PART 4  negative control: one face removed ===\n");
    bool all = true;
    int tested = 0;
    for (Shape& s : shapes()) {
        const int nf = (int)s.b.m_faces.size();
        if (nf < 2) {
            std::printf("[V17]   %-28s single face, cannot remove one -- skipped\n", s.name);
            continue;
        }
        std::vector<int> keep;
        for (int i = 0; i + 1 < nf; ++i) keep.push_back(i);
        const BRep open = s.b.subset(keep);
        const V2Verdict v = v2_verdict(open);
        const bool ok = v.naked_real > 0 && !v.closed() && !v.volume_valid;
        std::printf("[V17]   %-28s drop 1 of %d faces: %s  %s\n", s.name, nf, v.str().c_str(),
                    ok ? "DETECTED" : "MISSED");
        ++tested;
        if (!ok) all = false;
    }
    cell("open_shell_reports_naked_and_not_closed", all && tested > 0,
         sfmt("%d shapes with a face removed", tested));
}

///////////////////////////////////////////////////////////////////////////////////////////

int main() {
    std::printf("=== main_17  VERDICT HARNESS VALIDATION GATE (src/v2/v2_verdict) ===\n");
    part1_agreement();
    part2_volume();
    part3_invariance();
    part4_broken();
    std::printf("\n[V17] TOTAL pass=%d fail=%d  --  harness %s\n", g_pass, g_fail,
                g_fail == 0 ? "VALIDATED, may be used to score" : "NOT VALIDATED, do not use");
    return g_fail == 0 ? 0 : 1;
}
