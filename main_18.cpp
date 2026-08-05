// main_18 — CONE DIAGNOSTIC + INDEPENDENT LADDER driver (v2).
//
// Scored ONLY with the shared harness src/v2/v2_verdict.h (namespace v2v). Nothing here
// invents a verdict metric.
//
// MODES (argv[1]):
//   ladder [N] [filter]   8-pair ladder, PER-MOTION rows, v2 vs current kernel
//   signs  [N]            outward-sign truth table for the two cone pairs' operands:
//                         v2sol::v2_outward_signs vs BRep::face_outward_signs vs the ANALYTIC
//                         outward direction of the primitive in its own local frame
//   probe  [N]            for every operand face, the point-in-other-solid verdict that
//                         stage 6 would use (nearest_on_solid's sign), against the analytic
//                         answer for the same point
//
// The analytic truth used by `signs`/`probe` is the primitive's own implicit description in
// LOCAL coordinates (cone: r <= r0*(1 - z/h), 0 <= z <= h), with the operand's local->world
// transform composed exactly as main_16's build_ladder() composes it and inverted.

#include "src/brep.h"
#include "src/tolerance.h"
#include "src/brep_massprops.h"
#include "src/v2/brep_v2_boolean.h"
#include "src/v2/brep_v2_solid.h"
#include "src/v2/v2_verdict.h"
#include "src/xform.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;

static const char* BANNER = "main_18 cone diagnostic v1";

///////////////////////////////////////////////////////////////////////////////////////////
// poses and operands — byte-for-byte main_16::motion / build_ladder
///////////////////////////////////////////////////////////////////////////////////////////

static Xform motion(int k) {
    if (k == 0) return Xform::identity();
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

static Point xf_pt(const Xform& m, const Point& p) {
    return Point(m(0, 0) * p[0] + m(0, 1) * p[1] + m(0, 2) * p[2] + m(0, 3),
                 m(1, 0) * p[0] + m(1, 1) * p[1] + m(1, 2) * p[2] + m(1, 3),
                 m(2, 0) * p[0] + m(2, 1) * p[1] + m(2, 2) * p[2] + m(2, 3));
}

static Vector xf_vec(const Xform& m, const Vector& v) {
    return Vector(m(0, 0) * v[0] + m(0, 1) * v[1] + m(0, 2) * v[2],
                  m(1, 0) * v[0] + m(1, 1) * v[1] + m(1, 2) * v[2],
                  m(2, 0) * v[0] + m(2, 1) * v[1] + m(2, 2) * v[2]);
}

struct Pair {
    std::string name;
    BRep A, B;
    double volA = 0, common = 0, cut = 0;
};

static std::vector<Pair> build_ladder() {
    std::vector<Pair> v;
    {
        Pair p;
        p.name = "box x box";
        p.A = BRep::create_box(2, 2, 2);
        p.B = moved(BRep::create_box(2, 2, 2), Xform::translation(1.0, 0.5, 0.25));
        p.volA = 8.0;
        p.common = 1.0 * 1.5 * 1.75;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "sphere x sphere";
        p.A = BRep::create_sphere(1.0);
        p.B = moved(BRep::create_sphere(1.0), Xform::translation(1.0, 0, 0));
        p.volA = 4.0 * Tolerance::PI / 3.0;
        p.common = 5.0 * Tolerance::PI / 12.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "sphere x cylinder";
        p.A = BRep::create_sphere(2.5);
        p.B = moved(BRep::create_cylinder(1.0, 8.0), Xform::translation(0, 0, -4.0));
        p.volA = 4.0 * Tolerance::PI / 3.0 * 15.625;
        p.common = 4.0 * Tolerance::PI / 3.0 * (15.625 - std::pow(6.25 - 1.0, 1.5));
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "box x sphere";
        p.A = BRep::create_box(3, 3, 3);
        p.B = moved(BRep::create_sphere(1.0), Xform::translation(1.5, 0, 0));
        p.volA = 27.0;
        p.common = 2.0 * Tolerance::PI / 3.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "cylinder x cylinder";
        p.A = moved(BRep::create_cylinder(1.0, 4.0), Xform::translation(0, 0, -2.0));
        Vector ax(1, 0, 0);
        p.B = moved(moved(BRep::create_cylinder(1.0, 4.0), Xform::translation(0, 0, -2.0)),
                    Xform::rotation(ax, Tolerance::PI / 2.0, false));
        p.volA = Tolerance::PI * 4.0;
        p.common = 16.0 / 3.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "box x cone(A=cone)";
        p.A = BRep::create_cone(1.0, 2.0);
        p.B = moved(BRep::create_box(4, 4, 4), Xform::translation(0, 0, 3.0));
        p.volA = Tolerance::PI * 2.0 / 3.0;
        p.common = Tolerance::PI / 12.0;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "cone x cone";
        p.A = BRep::create_cone(1.0, 2.0);
        Vector ax(1, 0, 0);
        p.B = moved(moved(BRep::create_cone(1.5, 2.0), Xform::rotation(ax, Tolerance::PI, false)),
                    Xform::translation(0, 0, 1.6));
        p.volA = Tolerance::PI * 2.0 / 3.0;
        p.common = Tolerance::PI * (0.5625 * (std::pow(0.96, 3) - std::pow(0.4, 3)) / 3.0 +
                           2.0 * (std::pow(0.72, 3) - std::pow(0.2, 3)) / 3.0);
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "torus x torus";
        p.A = BRep::create_torus(3.0, 1.0);
        p.B = moved(BRep::create_torus(3.0, 1.0), Xform::translation(0, 0, 1.5));
        p.volA = 2.0 * Tolerance::PI * Tolerance::PI * 3.0;
        const double lens = 2.0 * std::acos(0.75) - 0.75 * std::sqrt(4.0 - 2.25);
        p.common = 2.0 * Tolerance::PI * 3.0 * lens;
        p.cut = p.volA - p.common;
        v.push_back(p);
    }
    return v;
}

///////////////////////////////////////////////////////////////////////////////////////////
// scoring — the shared harness only
///////////////////////////////////////////////////////////////////////////////////////////

static MassPropsOptions mp_opt() {
    MassPropsOptions o;
    o.rel_tolerance = 1e-10;
    o.max_surface_evals = 4000000;
    o.min_face_evals = 40000;
    return o;
}

struct J {
    bool closed = false, empty = false;
    double vol = 0;
    int faces = 0, naked = 0, nonman = 0;
    double resid = 1.0;
};

static J judge(const BRep& b) {
    J j;
    const v2v::V2Verdict v = v2v::v2_verdict(b, mp_opt());
    j.faces = v.faces;
    j.naked = v.naked_real;
    j.nonman = v.nonmanifold;
    j.resid = v.closure_residual;
    j.vol = v.volume;
    j.closed = v.closed();
    j.empty = (v.faces == 0);
    return j;
}

static bool vol_ok(double got, double want, double rel = 1e-6) {
    if (std::fabs(want) < 1e-12) return std::fabs(got) < 1e-9;
    return std::fabs(got - want) <= rel * std::fabs(want);
}

///////////////////////////////////////////////////////////////////////////////////////////
// mode: ladder
///////////////////////////////////////////////////////////////////////////////////////////

struct Score {
    int closed = 0, vol = 0, part = 0, n = 0;
};

static int mode_ladder(int n, const char* filter, bool per_motion) {
    std::printf("# %s  ladder n=%d filter=%s\n", BANNER, n, filter ? filter : "-");
    std::printf("pair,motion,kernel,cut_closed,cut_vol,com_closed,com_vol,ok_closed,ok_vol,ok_part\n");
    Score tc, tv;
    for (const Pair& p : build_ladder()) {
        if (filter && p.name.find(filter) == std::string::npos) continue;
        Score cur, v2s;
        for (int k = 0; k < n; ++k) {
            const Xform M = motion(k);
            const BRep A = moved(p.A, M), B = moved(p.B, M);
            for (int side = 0; side < 2; ++side) {
                J jc, ji;
                bool crashed = false;
                try {
                    if (side == 0) {
                        jc = judge(A.boolean(B, BRep::BooleanOp::Difference));
                        ji = judge(A.boolean(B, BRep::BooleanOp::Intersection));
                    } else {
                        jc = judge(v2sol::v2_cut(A, B));
                        ji = judge(v2sol::v2_common(A, B));
                    }
                } catch (...) {
                    crashed = true;
                }
                Score& s = side == 0 ? cur : v2s;
                ++s.n;
                bool cl = false, vv = false, pt = false;
                if (!crashed) {
                    cl = jc.closed && ji.closed;
                    vv = vol_ok(jc.vol, p.cut) && vol_ok(ji.vol, p.common);
                    pt = vol_ok(jc.vol + ji.vol, p.volA);
                    s.closed += cl;
                    s.vol += vv;
                    s.part += pt;
                }
                if (per_motion)
                    std::printf("%s,%d,%s,%d,%.9f,%d,%.9f,%d,%d,%d\n", p.name.c_str(), k,
                                side == 0 ? "cur" : "v2", (int)jc.closed, jc.vol, (int)ji.closed,
                                ji.vol, (int)cl, (int)vv, (int)pt);
            }
        }
        std::printf("SUM %-22s CURRENT %2d/%2d %2d/%2d %2d/%2d   V2 %2d/%2d %2d/%2d %2d/%2d\n",
                    p.name.c_str(), cur.closed, cur.n, cur.vol, cur.n, cur.part, cur.n, v2s.closed,
                    v2s.n, v2s.vol, v2s.n, v2s.part, v2s.n);
        std::fflush(stdout);
        tc.closed += cur.closed; tc.vol += cur.vol; tc.part += cur.part; tc.n += cur.n;
        tv.closed += v2s.closed; tv.vol += v2s.vol; tv.part += v2s.part; tv.n += v2s.n;
    }
    std::printf("TOTAL                      CURRENT %2d/%2d %2d/%2d %2d/%2d   V2 %2d/%2d %2d/%2d %2d/%2d\n",
                tc.closed, tc.n, tc.vol, tc.n, tc.part, tc.n, tv.closed, tv.n, tv.vol, tv.n,
                tv.part, tv.n);
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
// mode: signs / probe — the analytic truth for the two cone operands
///////////////////////////////////////////////////////////////////////////////////////////

/// A cone in its OWN local frame: base radius r at z=0, apex at z=h.
struct ConeSpec {
    double r = 1, h = 2;
    Xform to_world;   ///< local -> world
    Xform to_local;   ///< world -> local

    bool inside(const Point& pw, double eps) const {
        const Point p = xf_pt(to_local, pw);
        const double rr = std::hypot(p[0], p[1]);
        if (p[2] < -eps || p[2] > h + eps) return false;
        return rr <= r * (1.0 - p[2] / h) + eps;
    }
};

static void report_signs(const char* label, const BRep& b, const ConeSpec& cs) {
    const std::vector<double> sv2 = v2sol::v2_outward_signs(b);
    const std::vector<double> sk = b.face_outward_signs();
    for (int f = 0; f < b.face_count(); ++f) {
        Point p(0, 0, 0);
        Vector nn;
        if (!v2sol::v2_face_probe(b, f, p, nn)) {
            std::printf("%-14s f=%d PROBE_FAILED\n", label, f);
            continue;
        }
        const double L = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
        if (L < 1e-12) {
            std::printf("%-14s f=%d ZERO_NORMAL\n", label, f);
            continue;
        }
        const Vector u(nn[0] / L, nn[1] / L, nn[2] / L);
        const double e = 1e-4;
        const Point pp(p[0] + e * u[0], p[1] + e * u[1], p[2] + e * u[2]);
        const Point pm(p[0] - e * u[0], p[1] - e * u[1], p[2] - e * u[2]);
        const bool ip = cs.inside(pp, 0.0), im = cs.inside(pm, 0.0);
        int truth = 0;
        if (!ip && im) truth = +1;        // natural normal already points OUT
        else if (ip && !im) truth = -1;   // natural normal points IN
        const double s2 = f < (int)sv2.size() ? sv2[f] : 0.0;
        const double s1 = f < (int)sk.size() ? sk[f] : 0.0;
        std::printf("%-14s f=%d rev=%d probe=(%.5f,%.5f,%.5f) n=(%+.3f,%+.3f,%+.3f) truth=%+d "
                    "v2_outward=%+.0f kernel_outward=%+.0f  %s%s\n",
                    label, f, (int)b.m_faces[f].reversed, p[0], p[1], p[2], u[0], u[1], u[2],
                    truth, s2, s1,
                    (truth != 0 && s2 != truth) ? "V2_SIGN_WRONG " : "",
                    (truth != 0 && s1 != truth) ? "KERNEL_SIGN_WRONG" : "");
    }
}

static int mode_signs(int n) {
    std::printf("# %s  signs n=%d\n", BANNER, n);
    Vector ax(1, 0, 0);
    for (int k = 0; k < n; ++k) {
        const Xform M = motion(k);
        // cone x cone: A = cone(1,2) at identity, B = cone(1.5,2) rot pi about X then +1.6 z
        {
            ConeSpec ca;
            ca.r = 1.0; ca.h = 2.0;
            ca.to_world = M;
            ca.to_local = M.inverse().value_or(Xform::identity());
            const BRep A = moved(BRep::create_cone(1.0, 2.0), M);
            std::printf("--- motion %d ---\n", k);
            report_signs("coneA", A, ca);

            Xform rb = Xform::rotation(ax, Tolerance::PI, false);
            Xform bw = M * (Xform::translation(0, 0, 1.6) * rb);
            ConeSpec cb;
            cb.r = 1.5; cb.h = 2.0;
            cb.to_world = bw;
            cb.to_local = bw.inverse().value_or(Xform::identity());
            const BRep B = moved(moved(moved(BRep::create_cone(1.5, 2.0), rb),
                                       Xform::translation(0, 0, 1.6)), M);
            report_signs("coneB", B, cb);
        }
    }
    return 0;
}

/// Every ladder operand, every motion: v2sol::v2_outward_signs vs BRep::face_outward_signs.
/// The kernel's version is the one `nearest_on_solid` (brep_samedomain.cpp, the classifier that
/// stage 6 calls) was written against: it multiplies the SURFACE natural normal by the sign.
static int mode_signsall(int n) {
    std::printf("# %s  signsall n=%d\n", BANNER, n);
    std::printf("pair,operand,motion,faces,mismatch,shells,holes,v2_signs,kernel_signs\n");
    for (const Pair& p : build_ladder()) {
        for (int side = 0; side < 2; ++side) {
            int worst = 0;
            for (int k = 0; k < n; ++k) {
                const BRep b = moved(side ? p.B : p.A, motion(k));
                int shells = 0, holes = 0;
                const std::vector<double> s2 = v2sol::v2_outward_signs(b, &shells, &holes);
                const std::vector<double> s1 = b.face_outward_signs();
                int mism = 0;
                std::string a2, a1;
                for (size_t f = 0; f < s1.size(); ++f) {
                    if (f < s2.size() && s2[f] != s1[f]) ++mism;
                    a2 += (f < s2.size() && s2[f] < 0) ? '-' : '+';
                    a1 += (s1[f] < 0) ? '-' : '+';
                }
                worst = std::max(worst, mism);
                if (mism || k == 0)
                    std::printf("%s,%c,%d,%d,%d,%d,%d,%s,%s\n", p.name.c_str(), side ? 'B' : 'A',
                                k, (int)s1.size(), mism, shells, holes, a2.c_str(), a1.c_str());
            }
            std::printf("SUM %-22s %c worst_mismatch=%d\n", p.name.c_str(), side ? 'B' : 'A',
                        worst);
            std::fflush(stdout);
        }
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const std::string m = argc > 1 ? argv[1] : "ladder";
    if (m == "ladder")
        return mode_ladder(argc > 2 ? std::atoi(argv[2]) : 20, argc > 3 ? argv[3] : nullptr,
                           std::getenv("M18_PER_MOTION") != nullptr);
    if (m == "signs") return mode_signs(argc > 2 ? std::atoi(argv[2]) : 3);
    if (m == "signsall") return mode_signsall(argc > 2 ? std::atoi(argv[2]) : 20);
    std::printf("usage: main_18 ladder [N] [filter] | signs [N] | signsall [N]\n");
    return 2;
}
