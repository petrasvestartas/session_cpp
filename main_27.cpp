// main_27 — v3 G4 gate: chairs (kb/BOOL_V3_MEMORY.md, remaining work item 3).
// Freeform B-spline STEP solids (20 faces each, no pcurves) — exercises the
// marching/walker SSI path. Base pair chair0 x chair1 is checked against OCCT
// ground truth (cut 46.7943/35f, common 33.5025/25f, fuse 127.0913/50f, <=1%
// on volumes); the 10 rotated copies of chair1 in rot/ are gated on closedness
// plus the partition/fuse identities (no volume truth for those).
// Usage: main_27 [case-substring] [op: cut|common|fuse]   (filters, for repro)
#include "v3_geom.h"
#include "v3_topo.h"
#include "v3_classify.h"
#include "v3_bool.h"
#include "brep.h"
#include "file_step.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace session_cpp;
using namespace v3;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::printf("FAIL [%s:%d] ", __FILE__, __LINE__); \
           std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static const char* g_only_case = nullptr;
static const char* g_only_op = nullptr;

static std::string chairs_dir() {
    // driver runs from build_v3; fall back to the source root
    if (std::filesystem::exists("../serialization/boolean_steps/chairs/chair0.stp"))
        return "../serialization/boolean_steps/chairs/";
    return "serialization/boolean_steps/chairs/";
}

static BRep load_chair(const std::string& path) {
    auto breps = file_step::read_file_step_breps(path);
    CHECK(breps.size() == 1, "%s: expected 1 body, got %zu", path.c_str(), breps.size());
    if (breps.empty()) return BRep();
    return breps[0];
}

struct OpOut {
    BRep b;
    bool intersected = false;
    double seconds = 0.0;
};

static OpOut run_op(const v3::Solid& sa, const v3::Solid& sb, v3::BoolOp op) {
    OpOut r;
    auto t0 = std::chrono::steady_clock::now();
    v3::Solid s = v3::boolean(sa, sb, op, 1e-6, &r.intersected);
    r.b = v3::to_brep(s);
    r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

// One boolean op row: prints result, applies the gates that have truth.
static void check_op(const char* name, const char* opname, const OpOut& r,
                     bool gated, double vol_truth, int faces_truth) {
    double vol = r.b.volume();
    int faces = r.b.face_count();
    bool solid = r.b.is_solid();
    std::printf("  %-14s %-6s vol %9.4f  faces %3d  solid %d  ix %d  %.1fs",
                name, opname, vol, faces, (int)solid, (int)r.intersected, r.seconds);
    if (gated) {
        double rel = std::abs(vol - vol_truth) / vol_truth;
        std::printf("  truth %9.4f/%df  dv %.2e", vol_truth, faces_truth, rel);
        CHECK(r.intersected, "%s %s: operands not intersecting", name, opname);
        CHECK(solid, "%s %s: result not solid", name, opname);
        CHECK(rel <= 0.01, "%s %s: volume %.4f vs truth %.4f (%.2e)",
              name, opname, vol, vol_truth, rel);
        CHECK(faces == faces_truth, "%s %s: faces %d vs truth %d",
              name, opname, faces, faces_truth);
    } else {
        CHECK(r.intersected, "%s %s: operands not intersecting", name, opname);
        CHECK(solid, "%s %s: result not solid", name, opname);
    }
    std::printf("\n");
    std::fflush(stdout);
}

struct Case {
    std::string name;
    std::string bfile;
    // OCCT truth (base pair only)
    double cut_vol, common_vol, fuse_vol;
    int cut_faces, common_faces, fuse_faces;
};

int main(int argc, char** argv) {
    if (argc > 1) g_only_case = argv[1];
    if (argc > 2) g_only_op = argv[2];

    std::string dir = chairs_dir();

    // V3UVX=1 — section endpoint UV consistency: walker's on-surface UV vs
    // uv_of(3D point) vs distance to the face boundary (the B2 anomaly:
    // endpoint 0.366 UV off the boundary loop while ~0 off the 3D edge)
    if (std::getenv("V3UVX")) {
        BRep a = load_chair(dir + "chair0.stp");
        BRep b = load_chair(dir + "chair1.stp");
        v3::Solid sa = v3::from_brep(a), sb = v3::from_brep(b);
        const v3::Srf& A18 = sa.srfs[sa.faces[18].srf];
        const v3::Srf& B2 = sb.srfs[sb.faces[2].srf];
        // V3UVX=2: just the failing continuation pair, walker debug visible
        if (std::string(std::getenv("V3UVX")) == "2") {
            const v3::Srf& A16 = sa.srfs[sa.faces[16].srf];
            v3::SSIResult rr = v3::ssi(A16, B2, 1e-6);
            std::printf("A16 x B2: %zu curves\n", rr.curves.size());
            return 0;
        }
        // V3UVX=7: fine strict contact scan A16 x B2 near the anchor
        if (std::string(std::getenv("V3UVX")) == "7") {
            const v3::Srf& A16 = sa.srfs[sa.faces[16].srf];
            double bu0, bu1, bv0, bv1;
            B2.canonical_domain(bu0, bu1, bv0, bv1);
            v3::V3 anchor{7.6672, 4.5017, -1.4242};
            double au0, au1, av0, av1;
            A16.canonical_domain(au0, au1, av0, av1);
            double best = 1e300;
            int sign_pos = 0, sign_neg = 0;
            for (int i = 0; i <= 400; i++)
                for (int j = 0; j <= 400; j++) {
                    double u = au0 + (au1 - au0) * i / 400.0;
                    double v = av0 + (av1 - av0) * j / 400.0;
                    v3::V3 p = A16.eval(u, v);
                    if (p.dist(anchor) > 0.8) continue;
                    double uu, vv;
                    if (!B2.uv_of(p, uu, vv)) continue;
                    if (uu < bu0 || uu > bu1 || vv < bv0 || vv > bv1) continue;
                    v3::V3 q = B2.eval(uu, vv);
                    double d = q.dist(p);
                    if (d < 0.02) {
                        v3::V3 n = B2.normal(uu, vv);
                        if ((p - q).dot(n) > 0) sign_pos++; else sign_neg++;
                    }
                    if (d < best) best = d;
                }
            std::printf("A16 x B2 near anchor: min d %.6g, near pts +:%d -:%d\n",
                        best, sign_pos, sign_neg);
            // extent of the near-contact zone in A16's UV (is it inside the
            // walker's rect, or hugging the domain edge?)
            double cu0 = 1e300, cu1 = -1e300, cv0 = 1e300, cv1 = -1e300;
            int cnt = 0;
            for (int i = 0; i <= 400; i++)
                for (int j = 0; j <= 400; j++) {
                    double u = au0 + (au1 - au0) * i / 400.0;
                    double v = av0 + (av1 - av0) * j / 400.0;
                    v3::V3 p = A16.eval(u, v);
                    if (p.dist(anchor) > 0.8) continue;
                    double uu, vv;
                    if (!B2.uv_of(p, uu, vv)) continue;
                    if (uu < bu0 || uu > bu1 || vv < bv0 || vv > bv1) continue;
                    if (B2.eval(uu, vv).dist(p) < 3e-3) {
                        cu0 = std::min(cu0, u); cu1 = std::max(cu1, u);
                        cv0 = std::min(cv0, v); cv1 = std::max(cv1, v);
                        cnt++;
                    }
                }
            std::printf("  contact zone (%d pts): u[%.4f,%.4f] v[%.4f,%.4f] "
                        "(domain u[%.3f,%.3f] v[%.3f,%.3f])\n", cnt, cu0, cu1,
                        cv0, cv1, au0, au1, av0, av1);
            return 0;
        }
        // projections only (the earlier scan accepted out-of-domain phantom UVs)
        if (std::string(std::getenv("V3UVX")) == "6") {
            const v3::Srf& A16 = sa.srfs[sa.faces[16].srf];
            double bu0, bu1, bv0, bv1;
            B2.canonical_domain(bu0, bu1, bv0, bv1);
            double au0, au1, av0, av1;
            A16.canonical_domain(au0, au1, av0, av1);
            double best = 1e300;
            v3::V3 bp{0, 0, 0};
            double bu = 0, bv = 0;
            for (int i = 0; i <= 96; i++)
                for (int j = 0; j <= 96; j++) {
                    double u = au0 + (au1 - au0) * i / 96.0;
                    double v = av0 + (av1 - av0) * j / 96.0;
                    if (!v3::uv_in_face(sa, sa.faces[16], u, v)) continue;
                    v3::V3 p = A16.eval(u, v);
                    double uu, vv;
                    if (!B2.uv_of(p, uu, vv)) continue;
                    if (uu < bu0 - 1e-9 || uu > bu1 + 1e-9 || vv < bv0 - 1e-9 ||
                        vv > bv1 + 1e-9)
                        continue; // out-of-domain phantom
                    double d = B2.eval(uu, vv).dist(p);
                    if (d < best) { best = d; bp = p; bu = uu; bv = vv; }
                }
            std::printf("A16(trim) x B2(in-domain): min d %.6g at (%.4f,%.4f,%.4f) "
                        "uvB=(%.4f,%.4f)\n", best, bp.x, bp.y, bp.z, bu, bv);
            return 0;
        }
        // uv_in_face, and vs a raw polygon winding test on the face UV loop?
        if (std::string(std::getenv("V3UVX")) == "5") {
            const v3::Srf& A16 = sa.srfs[sa.faces[16].srf];
            const v3::Srf& B17 = sb.srfs[sb.faces[17].srf];
            v3::SSIResult rr = v3::ssi(A16, B17, 1e-6);
            std::printf("A16 x B17: %zu curves\n", rr.curves.size());
            for (auto& sc : rr.curves) {
                int in = 0, out = 0;
                double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
                for (auto& q : sc.pts) {
                    bool f = v3::uv_in_face(sa, sa.faces[16], q.u1, q.v1);
                    if (f) in++; else out++;
                    u0 = std::min(u0, q.u1); u1 = std::max(u1, q.u1);
                    v0 = std::min(v0, q.v1); v1 = std::max(v1, q.v1);
                }
                std::printf("  curve pts=%zu inA=%d outA=%d  u[%.4f,%.4f] v[%.4f,%.4f]\n",
                            sc.pts.size(), in, out, u0, u1, v0, v1);
            }
            // the face's UV loop bbox for reference
            double lu0 = 1e300, lu1 = -1e300, lv0 = 1e300, lv1 = -1e300;
            for (auto& l : sa.faces[16].loops)
                for (int cei : l.ces)
                    for (auto& q : sa.coedges[cei].pc) {
                        lu0 = std::min(lu0, q.u); lu1 = std::max(lu1, q.u);
                        lv0 = std::min(lv0, q.v); lv1 = std::max(lv1, q.v);
                    }
            std::printf("  A16 loop bbox u[%.4f,%.4f] v[%.4f,%.4f]\n", lu0, lu1,
                        lv0, lv1);
            // independent winding test on the face's UV loops (outer - holes)
            auto wind = [&](double u, double v) {
                int w = 0;
                for (auto& l : sa.faces[16].loops) {
                    int wl = 0;
                    for (int cei : l.ces) {
                        auto& pc = sa.coedges[cei].pc;
                        for (size_t i = 0; i + 1 < pc.size(); i++) {
                            double u0 = pc[i].u, v0 = pc[i].v, u1 = pc[i + 1].u,
                                   v1 = pc[i + 1].v;
                            if (v0 <= v) {
                                if (v1 > v && (u1 - u0) * (v - v0) - (u - u0) * (v1 - v0) > 0) wl++;
                            } else {
                                if (v1 <= v && (u1 - u0) * (v - v0) - (u - u0) * (v1 - v0) < 0) wl--;
                            }
                        }
                    }
                    w += l.outer ? wl : -wl;
                }
                return w;
            };
            for (auto& sc : rr.curves) {
                for (size_t k : {sc.pts.size() / 4, sc.pts.size() / 2,
                                 3 * sc.pts.size() / 4}) {
                    auto& q = sc.pts[k];
                    std::printf("  sample (%.4f,%.4f): uv_in_face=%d winding=%d\n",
                                q.u1, q.v1,
                                (int)v3::uv_in_face(sa, sa.faces[16], q.u1, q.v1),
                                wind(q.u1, q.v1));
                }
            }
            return 0;
        }
        if (std::string(std::getenv("V3UVX")) == "4") {
            for (int side = 0; side < 2; side++) {
                BRep& bb = side ? b : a;
                int pu = 0, pv = 0;
                for (auto& s : bb.m_surfaces) {
                    if (s.is_periodic(0)) pu++;
                    if (s.is_periodic(1)) pv++;
                }
                std::printf("%s: %zu surfaces, periodic_u %d, periodic_v %d\n",
                            side ? "chair1" : "chair0", bb.m_surfaces.size(), pu, pv);
            }
            // B2's eval continuity across the v domain edges
            const v3::Srf& B2s = sb.srfs[sb.faces[2].srf];
            double u0, u1, v0, v1;
            B2s.canonical_domain(u0, u1, v0, v1);
            v3::V3 p0 = B2s.eval(1.2, v0), p1 = B2s.eval(1.2, v1);
            std::printf("B2 v-seam gap at u=1.2: %.6g (domain v %.4f..%.4f)\n",
                        p0.dist(p1), v0, v1);
            return 0;
        }
        if (std::string(std::getenv("V3UVX")) == "3") {
            const v3::Srf& A16 = sa.srfs[sa.faces[16].srf];
            double au0, au1, av0, av1, bu0, bu1, bv0, bv1;
            A16.canonical_domain(au0, au1, av0, av1);
            B2.canonical_domain(bu0, bu1, bv0, bv1);
            double best = 1e300;
            v3::V3 bp, bq;
            int neg = 0, pos = 0;
            for (int i = 0; i <= 64; i++)
                for (int j = 0; j <= 64; j++) {
                    v3::V3 p = A16.eval(au0 + (au1 - au0) * i / 64.0,
                                        av0 + (av1 - av0) * j / 64.0);
                    double uu, vv;
                    if (!B2.uv_of(p, uu, vv)) continue;
                    v3::V3 q = B2.eval(uu, vv);
                    v3::V3 n = B2.normal(uu, vv);
                    double d = q.dist(p);
                    double sgn = (p - q).dot(n);
                    if (d < 0.02) {
                        if (sgn < 0) neg++; else pos++;
                        if (d < best) { best = d; bp = p; bq = q; }
                    }
                }
            std::printf("A16 vs B2: min d %.6g at (%.4f,%.4f,%.4f), near pts neg=%d pos=%d\n",
                        best, bp.x, bp.y, bp.z, neg, pos);
            double ua, va, ub2, vb2;
            bool oka = A16.uv_of(bp, ua, va), okb = B2.uv_of(bp, ub2, vb2);
            std::printf("  uv on A16 (%d): (%.4f,%.4f) of u[%.3f,%.3f] v[%.3f,%.3f]\n",
                        (int)oka, ua, va, au0, au1, av0, av1);
            std::printf("  uv on B2  (%d): (%.4f,%.4f) of u[%.3f,%.3f] v[%.3f,%.3f]\n",
                        (int)okb, ub2, vb2, bu0, bu1, bv0, bv1);
            return 0;
        }
        v3::SSIResult r = v3::ssi(A18, B2, 1e-6);
        // continuation census: the section ends at A18's surface domain edge,
        // interior to B2's face — the curve must continue on a neighboring A
        // face. Test every A face x B2 and measure endpoint proximity.
        v3::V3 anchor = r.curves.empty() || r.curves[0].pts.empty()
                            ? v3::V3{0, 0, 0}
                            : r.curves[0].pts.front().p;
        for (size_t fi = 0; fi < sa.faces.size(); fi++) {
            if ((int)fi == 18) continue;
            v3::SSIResult rr = v3::ssi(sa.srfs[sa.faces[fi].srf], B2, 1e-6);
            double best = 1e300;
            for (auto& sc : rr.curves) {
                if (sc.pts.empty()) continue;
                best = std::min(best, sc.pts.front().p.dist(anchor));
                best = std::min(best, sc.pts.back().p.dist(anchor));
            }
            std::printf("continuation A%zu x B2: %zu curves, nearest end %.4g\n",
                        fi, rr.curves.size(), best);
        }
        std::printf("anchor (A18 x B2 sec start): (%.4f,%.4f,%.4f)\n", anchor.x,
                    anchor.y, anchor.z);
        // iso-scan replication for A16 x B2 (the suspected continuation pair):
        // does the walker's seeding see the crossing at all?
        for (int swap = 0; swap < 2; swap++) {
            const v3::Srf& SA = swap ? B2 : sa.srfs[sa.faces[16].srf];
            const v3::Srf& SB = swap ? sa.srfs[sa.faces[16].srf] : B2;
            double au0, au1, av0, av1;
            SA.canonical_domain(au0, au1, av0, av1);
            double best_min = 1e300, gmin = 1e300;
            int n_min = 0;
            const int NG = 16, NS = 128;
            for (int dir = 0; dir < 2; dir++)
                for (int i = 0; i <= NG; i++) {
                    double iso = (dir == 0 ? au0 + (au1 - au0) * i / NG
                                           : av0 + (av1 - av0) * i / NG);
                    double pd = 1e300, p2d = 1e300, sp3 = 1e300;
                    v3::V3 pp{0, 0, 0};
                    double su = 0, sv = 0;
                    bool seeded = false;
                    for (int j = 0; j <= NS; j++) {
                        double par = (dir == 0 ? av0 + (av1 - av0) * j / NS
                                               : au0 + (au1 - au0) * j / NS);
                        v3::V3 p = SA.eval(dir == 0 ? iso : par, dir == 0 ? par : iso);
                        double uu = seeded ? su : 0, vv = seeded ? sv : 0;
                        if (!seeded) {
                            if (!SB.uv_of(p, uu, vv)) { pp = p; continue; }
                        } else if (!SB.uv_of(p, uu, vv)) { pp = p; continue; }
                        seeded = true; su = uu; sv = vv;
                        double d = SB.eval(uu, vv).dist(p);
                        gmin = std::min(gmin, d);
                        if (j >= 2 && pd <= p2d && pd <= d && pd < 4.0 * sp3) {
                            n_min++;
                            best_min = std::min(best_min, pd);
                        }
                        p2d = pd; pd = d;
                        if (j >= 1) sp3 = pp.dist(p);
                        pp = p;
                    }
                }
            std::printf("iso-scan %s x %s: minima %d (best %.4g), global min d %.4g\n",
                        swap ? "B2" : "A16", swap ? "A16" : "B2", n_min, best_min,
                        gmin);
        }
        for (auto& sc : r.curves) {
            if (sc.pts.size() < 2) continue;
            for (int e = 0; e < 2; e++) {
                auto& q = e ? sc.pts.back() : sc.pts.front();
                double uu = 0, vv = 0;
                bool ok = B2.uv_of(q.p, uu, vv);
                double best = 1e300;
                for (auto& l : sb.faces[2].loops)
                    for (int cei : l.ces) {
                        const v3::Edge& ed = sb.edges[sb.coedges[cei].edge];
                        if (ed.degenerate) continue;
                        double t;
                        if (!ed.c.project(q.p, t)) continue;
                        t = std::max(ed.t0, std::min(ed.t1, t));
                        best = std::min(best, ed.c.eval(t).dist(q.p));
                    }
                std::printf("sec end %d: walk uvB=(%.6f,%.6f) uv_of=(%.6f,%.6f) ok=%d "
                            "res=%.3g dist-to-B2-bnd=%.3g\n", e, q.u2, q.v2, uu, vv,
                            (int)ok, ok ? B2.eval(uu, vv).dist(q.p) : -1.0, best);
            }
        }
        return 0;
    }
    // V3SPC=1 — SameParameter check of imported boundary loops: residual
    // |S.eval(pc) - edge(t)| per coedge sample (uv_of failures leave garbage).
    if (std::getenv("V3SPC")) {
        BRep a = load_chair(dir + "chair0.stp");
        BRep b = load_chair(dir + "chair1.stp");
        v3::Solid sa = v3::from_brep(a), sb = v3::from_brep(b);
        for (int side = 0; side < 2; side++) {
            v3::Solid& s = side ? sb : sa;
            double worst = 0;
            int wf = -1, we = -1;
            for (size_t fi = 0; fi < s.faces.size(); fi++)
                for (auto& l : s.faces[fi].loops)
                    for (int cei : l.ces) {
                        const v3::CoEdge& ce = s.coedges[cei];
                        const v3::Edge& e = s.edges[ce.edge];
                        if (e.degenerate || ce.pc.empty()) continue;
                        const v3::Srf& srf = s.srfs[s.faces[fi].srf];
                        for (size_t i = 0; i < ce.pc.size(); i++) {
                            double r = srf.eval(ce.pc[i].u, ce.pc[i].v)
                                           .dist(e.c.eval(ce.pt[i]));
                            if (r > worst) { worst = r; wf = (int)fi; we = ce.edge; }
                        }
                    }
            std::printf("sameparam %s: worst %.6g (face %d edge %d)\n",
                        side ? "chair1" : "chair0", worst, wf, we);
        }
        return 0;
    }
    // V3CHAIRPROBE=1 — walker SSI diagnosis (no booleans): recognition census of
    // the chair surfaces, ssi() curve count over all chair0 x chair1 face pairs,
    // and a minimal analytic walker repro (non-coaxial cylinder x torus).
    // V3XC=1 — classifier cross-check only (v3 classify_point vs v1
    // contains_point oracle at random bbox points).
    if (std::getenv("V3XC")) {
        BRep a = load_chair(dir + "chair0.stp");
        BRep b = load_chair(dir + "chair1.stp");
        v3::Solid sa = v3::from_brep(a), sb = v3::from_brep(b);
        v3::orient_solid(sa);
        v3::orient_solid(sb);
        V3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
        for (auto& vtx : sa.verts) {
            lo.x = std::min(lo.x, vtx.p.x); hi.x = std::max(hi.x, vtx.p.x);
            lo.y = std::min(lo.y, vtx.p.y); hi.y = std::max(hi.y, vtx.p.y);
            lo.z = std::min(lo.z, vtx.p.z); hi.z = std::max(hi.z, vtx.p.z);
        }
        std::mt19937 rng(777);
        std::uniform_real_distribution<double> U(0, 1);
        int bad_a = 0, bad_b = 0, n = 300, ta = 0, tb = 0;
        for (int i = 0; i < n; i++) {
            v3::V3 p{lo.x + (hi.x - lo.x) * U(rng), lo.y + (hi.y - lo.y) * U(rng),
                     lo.z + (hi.z - lo.z) * U(rng)};
            bool in1 = a.contains_point(Point(p.x, p.y, p.z));
            v3::PtCls c3 = v3::classify_point(sa, p, 1e-5);
            if (c3 != v3::PtCls::ON) { ta++; if ((c3 == v3::PtCls::IN) != in1) bad_a++; }
            bool in1b = b.contains_point(Point(p.x, p.y, p.z));
            v3::PtCls c3b = v3::classify_point(sb, p, 1e-5);
            if (c3b != v3::PtCls::ON) { tb++; if ((c3b == v3::PtCls::IN) != in1b) bad_b++; }
            if (i % 50 == 0) std::fflush(stdout);
        }
        std::printf("classifier xcheck: chair0 bad %d/%d, chair1 bad %d/%d\n",
                    bad_a, ta, bad_b, tb);
        return 0;
    }
    if (std::getenv("V3CHAIRPROBE")) {
        BRep a = load_chair(dir + "chair0.stp");
        BRep b = load_chair(dir + "chair1.stp");
        v3::Solid sa = v3::from_brep(a), sb = v3::from_brep(b);
        auto kinds = [](v3::Solid& s, const char* nm) {
            int hist[8] = {0};
            for (auto& f : s.faces) hist[(int)s.srfs[f.srf].k]++;
            std::printf("%s kinds: plane=%d cyl=%d cone=%d sph=%d torus=%d nurbs=%d\n",
                        nm, hist[0], hist[1], hist[2], hist[3], hist[4], hist[5]);
        };
        kinds(sa, "chair0");
        kinds(sb, "chair1");
        int pairs_with = 0, total_curves = 0;
        for (size_t ai = 0; ai < sa.faces.size(); ai++)
            for (size_t bi = 0; bi < sb.faces.size(); bi++) {
                v3::SSIResult r = v3::ssi(sa.srfs[sa.faces[ai].srf],
                                          sb.srfs[sb.faces[bi].srf], 1e-6);
                if (!r.curves.empty()) {
                    pairs_with++;
                    total_curves += (int)r.curves.size();
                    std::printf("  ssi A%zu x B%zu: %zu curves\n", ai, bi, r.curves.size());
                }
            }
        std::printf("chair0 x chair1: pairs with sections %d/400, total curves %d\n",
                    pairs_with, total_curves);
        // surface-surface min-distance census (independent of the walker):
        // sample A on a grid, project onto B via uv_of, track the min.
        double gmin = 1e300;
        size_t gai = 0, gbi = 0;
        for (size_t ai = 0; ai < sa.faces.size(); ai++)
            for (size_t bi = 0; bi < sb.faces.size(); bi++) {
                const v3::Srf& A = sa.srfs[sa.faces[ai].srf];
                const v3::Srf& B = sb.srfs[sb.faces[bi].srf];
                double au0, au1, av0, av1;
                A.canonical_domain(au0, au1, av0, av1);
                for (int i = 0; i <= 24; i++)
                    for (int j = 0; j <= 24; j++) {
                        V3 p = A.eval(au0 + (au1 - au0) * i / 24.0,
                                      av0 + (av1 - av0) * j / 24.0);
                        double uu, vv;
                        if (!B.uv_of(p, uu, vv)) continue;
                        double d = B.eval(uu, vv).dist(p);
                        if (d < gmin) { gmin = d; gai = ai; gbi = bi; }
                    }
            }
        std::printf("min surface-surface distance over all pairs: %.6g (A%zu x B%zu)\n",
                    gmin, gai, gbi);
        // replicate the walker's iso-scan seeding on the closest pair to find
        // which stage loses the section (candidate minima vs corrector)
        {
            const v3::Srf& A = sa.srfs[sa.faces[gai].srf];
            const v3::Srf& B = sb.srfs[sb.faces[gbi].srf];
            double au0, au1, av0, av1, bu0, bu1, bv0, bv1;
            A.canonical_domain(au0, au1, av0, av1);
            B.canonical_domain(bu0, bu1, bv0, bv1);
            int n_minima = 0, n_proj_fail = 0;
            double best_min_d = 1e300;
            const int NG = 16, NS = 128;
            for (int dir = 0; dir < 2; dir++)
                for (int i = 0; i <= NG; i++) {
                    double iso = (dir == 0 ? au0 + (au1 - au0) * i / NG
                                           : av0 + (av1 - av0) * i / NG);
                    double prev_d = 1e300, prev2_d = 1e300, sp3 = 1e300;
                    v3::V3 prev_p{0, 0, 0};
                    double su = 0.5 * (bu0 + bu1), sv = 0.5 * (bv0 + bv1);
                    for (int j = 0; j <= NS; j++) {
                        double par = (dir == 0 ? av0 + (av1 - av0) * j / NS
                                               : au0 + (au1 - au0) * j / NS);
                        v3::V3 p = A.eval(dir == 0 ? iso : par, dir == 0 ? par : iso);
                        double uu = su, vv = sv;
                        if (!B.uv_of(p, uu, vv)) { n_proj_fail++; continue; }
                        double d = B.eval(uu, vv).dist(p);
                        su = uu; sv = vv;
                        if (j >= 2 && prev_d <= prev2_d && prev_d <= d &&
                            prev_d < 4.0 * sp3) {
                            n_minima++;
                            best_min_d = std::min(best_min_d, prev_d);
                        }
                        prev2_d = prev_d; prev_d = d;
                        if (j >= 1) sp3 = prev_p.dist(p);
                        prev_p = p;
                    }
                }
            std::printf("iso-scan A%zu x B%zu: candidate minima %d (best d %.3g), "
                        "projection failures %d\n", gai, gbi, n_minima, best_min_d,
                        n_proj_fail);
        }
        // minimal walker repro: cylinder(z, r=0.8) x torus(x-axis, R=1.0, r=0.5)
        // interpenetrate (torus rho band [0.5,1.5] crosses 0.8); no exact case
        // exists for non-coaxial cyl x torus, so this must go through the walker.
        v3::Srf cyl = v3::srf_cylinder({0, 0, 0}, {0, 0, 1}, 0.8);
        v3::Srf tor = v3::srf_torus({0, 0, 0}, {1, 0, 0}, 1.0, 0.5);
        v3::SSIResult rr = v3::ssi(cyl, tor, 1e-6);
        std::printf("walker repro cyl(z,0.8) x torus(x,1.0,0.5): %zu curves\n",
                    rr.curves.size());
        // corner chaining: UV + 3D gap between consecutive coedges in each loop
        // (chair STEP has no pcurves; kb v2 measured max_chain_gap 1.3e-4)
        for (v3::Solid* s : {&sa, &sb}) {
            double max_uv = 0, max_3d = 0;
            int nf = 0;
            for (auto& f : s->faces)
                for (auto& l : f.loops)
                    for (size_t k = 0; k < l.ces.size(); k++) {
                        const v3::CoEdge& c0 = s->coedges[l.ces[k]];
                        const v3::CoEdge& c1 = s->coedges[l.ces[(k + 1) % l.ces.size()]];
                        if (c0.pc.empty() || c1.pc.empty()) continue;
                        nf++;
                        double du = c0.pc.back().u - c1.pc.front().u;
                        double dv = c0.pc.back().v - c1.pc.front().v;
                        max_uv = std::max(max_uv, std::sqrt(du * du + dv * dv));
                        const v3::Edge& e0 = s->edges[c0.edge];
                        const v3::Edge& e1 = s->edges[c1.edge];
                        v3::V3 p0 = e0.c.eval(c0.fwd ? e0.t1 : e0.t0);
                        v3::V3 p1 = e1.c.eval(c1.fwd ? e1.t0 : e1.t1);
                        max_3d = std::max(max_3d, p0.dist(p1));
                    }
            std::printf("corner gaps: junctions %d  max UV %.3g  max 3D %.3g\n",
                        nf, max_uv, max_3d);
        }
        // EF pierce census (replicates v3_bool's pierce()): every edge curve
        // against the other solid's face surfaces via csi()
        for (int side = 0; side < 2; side++) {
            v3::Solid& X = side ? sb : sa;
            v3::Solid& Y = side ? sa : sb;
            int hits = 0, tested = 0;
            double best_miss = 1e300;
            for (auto& e : X.edges) {
                if (e.degenerate) continue;
                for (auto& f : Y.faces) {
                    tested++;
                    auto h = v3::csi(e.c, e.t0, e.t1, Y.srfs[f.srf], 1e-6);
                    hits += (int)h.size();
                    // track nearest approach for the no-hit diagnosis
                    if (h.empty()) {
                        for (int i = 0; i <= 32; i++) {
                            v3::V3 p = e.c.eval(e.t0 + (e.t1 - e.t0) * i / 32.0);
                            double uu, vv;
                            if (!Y.srfs[f.srf].uv_of(p, uu, vv)) continue;
                            best_miss = std::min(best_miss,
                                Y.srfs[f.srf].eval(uu, vv).dist(p));
                        }
                    }
                }
            }
            std::printf("csi census %s: pairs %d, hits %d, nearest miss %.3g\n",
                        side ? "B-edges x A-faces" : "A-edges x B-faces",
                        tested, hits, best_miss);
        }
        // classifier cross-check moved to V3XC (see above)
        return 0;
    }

    BRep a = load_chair(dir + "chair0.stp");
    std::printf("[G4] chairs — chair0: %d faces, vol %.6f\n", a.face_count(), a.volume());

    Case base{"base", "chair1.stp", 46.7943, 33.5025, 127.0913, 35, 25, 50};
    std::vector<Case> rots;
    for (const char* cfg : {"z15", "z30", "z45", "z90", "x20", "y30",
                            "z30x20", "z37", "x13y29", "z63"}) {
        Case c;
        c.name = std::string("rot_") + cfg;
        c.bfile = std::string("rot/B_") + cfg + ".step";
        rots.push_back(c);
    }

    v3::Solid sa = v3::from_brep(a);
    double va = a.volume();

    auto run_case = [&](const Case& c) {
        if (g_only_case && strcmp(c.name.c_str(), g_only_case) != 0) return;
        bool gated = !c.bfile.empty() && c.bfile == "chair1.stp";
        std::printf("[case] %s  B=%s\n", c.name.c_str(), c.bfile.c_str());
        std::fflush(stdout); // a crash in v3 still leaves the case name in the log
        BRep b = load_chair(dir + c.bfile);
        double vb = b.volume();
        v3::Solid sb = v3::from_brep(b);
        OpOut cut = run_op(sa, sb, v3::BoolOp::CUT);
        OpOut com = run_op(sa, sb, v3::BoolOp::COMMON);
        OpOut fus = run_op(sa, sb, v3::BoolOp::FUSE);
        if (!g_only_op || !strcmp(g_only_op, "cut"))
            check_op(c.name.c_str(), "cut", cut, gated, c.cut_vol, c.cut_faces);
        if (!g_only_op || !strcmp(g_only_op, "common"))
            check_op(c.name.c_str(), "common", com, gated, c.common_vol, c.common_faces);
        if (!g_only_op || !strcmp(g_only_op, "fuse"))
            check_op(c.name.c_str(), "fuse", fus, gated, c.fuse_vol, c.fuse_faces);
        // oracle-free identities (all cases): partition + fuse
        double vc = cut.b.volume(), vm = com.b.volume(), vf = fus.b.volume();
        double part = vc + vm - va;
        double fid = vf - (va + vb - vm);
        std::printf("  %-14s volA %.4f volB %.4f  partition %+.4f (%.2e)  fuse-id %+.4f (%.2e)\n",
                    c.name.c_str(), va, vb, part, std::abs(part) / va,
                    fid, std::abs(fid) / (va + vb));
        std::fflush(stdout);
        CHECK(cut.intersected && com.intersected && fus.intersected,
              "%s: not intersecting on some op", c.name.c_str());
        CHECK(std::abs(part) <= 0.01 * va,
              "%s: partition cut(%.4f)+common(%.4f)-A(%.4f) = %.4f",
              c.name.c_str(), vc, vm, va, part);
        CHECK(std::abs(fid) <= 0.01 * (va + vb),
              "%s: fuse identity %.4f", c.name.c_str(), fid);
    };

    run_case(base);
    for (const Case& c : rots) run_case(c);

    std::printf("\nG4 totals: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
