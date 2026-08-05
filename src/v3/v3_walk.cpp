// v3 marching surface-surface intersection (IntWalk equivalent) + curve-surface
// intersection. Used when no closed form exists (general quadric pairs, torus
// non-coaxial, anything involving NURBS). Architecture per kb/BOOL_V3_MEMORY.md:
// iso-curve seeding -> Newton polish -> predictor/corrector walk with
// deflection control -> branch dedupe. Point budgets are hard-capped.

#include "v3_ssi.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>

namespace v3 {

namespace {

struct WalkOpt {
    double tol3d;
    // UV seeding rects (natural domains by default; boolean passes face rects)
    double au0, au1, av0, av1;
    double bu0, bu1, bv0, bv1;
    double seed_skip = 0;    // seed-on-walked-curve skip radius (set by ssi)
    int max_branch_pts = 20000;
    long max_total_pts = 200000; // hard memory guard (kb lesson: cap early)
};

// fast Newton projection onto NURBS from a seed (no grid search). Returns
// true when Newton CONVERGED to a stationary point — NOT a proximity test:
// callers measure the residual themselves (the iso-scan minima finders need
// the distance even when the point is far from the surface). Each step is
// CLAMPED to the canonical domain: unconstrained Newton happily walks off
// the knot span into polynomial extrapolation, producing phantom closest
// points (and phantom intersection seeds) outside the surface entirely.
bool project_nurbs_seeded(const Srf& s, const V3& p, double& u, double& v, int iters = 25) {
    double u0, u1, v0, v1;
    s.canonical_domain(u0, u1, v0, v1);
    u = std::max(u0, std::min(u1, u));
    v = std::max(v0, std::min(v1, v));
    for (int it = 0; it < iters; it++) {
        V3 sp, du, dv;
        s.d0d1(u, v, sp, du, dv);
        V3 r = sp - p;
        double a = du.norm2(), b = du.dot(dv), c = dv.norm2();
        double det = a * c - b * b;
        if (det < 1e-300) return false;
        double ru = r.dot(du), rv = r.dot(dv);
        double ddu = (-ru * c + rv * b) / det;
        double ddv = (-rv * a + ru * b) / det;
        u = std::max(u0, std::min(u1, u + ddu));
        v = std::max(v0, std::min(v1, v + ddv));
        if ((du * ddu + dv * ddv).norm() < 1e-12) return true;
    }
    return false; // no stationary point reached within the iteration budget
}

// 4-var Newton corrector: bring (u,v,s,t) onto both surfaces.
// Min-norm steps keep the point near the predictor estimate.
bool correct(const Srf& A, const Srf& B, double& u, double& v, double& s, double& t,
             double tol, V3* out_p = nullptr) {
    for (int it = 0; it < 25; it++) {
        V3 pa, du, dv, pb, ds, dt;
        A.d0d1(u, v, pa, du, dv);
        B.d0d1(s, t, pb, ds, dt);
        V3 r = pa - pb;
        double err = r.norm();
        if (err < tol) {
            if (out_p) *out_p = (pa + pb) * 0.5;
            return true;
        }
        // J = [du dv -ds -dt] (3x4); solve J d = -r min-norm via J^T (J J^T)^-1
        double jj[3][3] = {};
        V3 J[4] = {du, dv, ds * -1.0, dt * -1.0};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                jj[i][j] = J[0][i] * J[0][j] + J[1][i] * J[1][j] +
                           J[2][i] * J[2][j] + J[3][i] * J[3][j];
            }
        // damping for near-singular (tangent) configurations
        double tr = (jj[0][0] + jj[1][1] + jj[2][2]) / 3.0;
        double lam = tr * 1e-14 + 1e-300;
        jj[0][0] += lam; jj[1][1] += lam; jj[2][2] += lam;
        // solve jj * y = -r  (3x3, gaussian)
        double M[3][4] = {{jj[0][0], jj[0][1], jj[0][2], -r.x},
                          {jj[1][0], jj[1][1], jj[1][2], -r.y},
                          {jj[2][0], jj[2][1], jj[2][2], -r.z}};
        bool ok = true;
        for (int col = 0; col < 3 && ok; col++) {
            int piv = col;
            for (int r2 = col + 1; r2 < 3; r2++)
                if (std::abs(M[r2][col]) > std::abs(M[piv][col])) piv = r2;
            if (std::abs(M[piv][col]) < 1e-300) { ok = false; break; }
            for (int c2 = col; c2 < 4; c2++) std::swap(M[piv][c2], M[col][c2]);
            for (int r2 = col + 1; r2 < 3; r2++) {
                double f = M[r2][col] / M[col][col];
                for (int c2 = col; c2 < 4; c2++) M[r2][c2] -= f * M[col][c2];
            }
        }
        if (!ok) return false;
        double y[3];
        for (int i = 2; i >= 0; i--) {
            double sum = M[i][3];
            for (int j = i + 1; j < 3; j++) sum -= M[i][j] * y[j];
            y[i] = sum / M[i][i];
        }
        // d = J^T y
        double d[4] = {};
        for (int k = 0; k < 4; k++) d[k] = J[k].x * y[0] + J[k].y * y[1] + J[k].z * y[2];
        u += d[0]; v += d[1]; s += d[2]; t += d[3];
        double step = std::abs(d[0]) + std::abs(d[1]) + std::abs(d[2]) + std::abs(d[3]);
        if (step < 1e-14) {
            V3 qa = A.eval(u, v), qb = B.eval(s, t);
            if ((qa - qb).norm() < tol) {
                if (out_p) *out_p = (qa + qb) * 0.5;
                return true;
            }
            return false;
        }
    }
    V3 qa = A.eval(u, v), qb = B.eval(s, t);
    if ((qa - qb).norm() < tol) {
        if (out_p) *out_p = (qa + qb) * 0.5;
        return true;
    }
    return false;
}

// tangent of the intersection curve at a point: nA x nB
V3 walk_tangent(const Srf& A, const Srf& B, double u, double v, double s, double t) {
    V3 pa, du, dv, pb, ds, dt;
    A.d0d1(u, v, pa, du, dv);
    B.d0d1(s, t, pb, ds, dt);
    V3 na = du.cross(dv), nb = ds.cross(dt);
    if (na.norm2() < 1e-24 && A.has_implicit()) na = A.gradF(pa);
    if (nb.norm2() < 1e-24 && B.has_implicit()) nb = B.gradF(pb);
    return na.cross(nb);
}

struct Seed { double u, v, s, t; V3 p; };

// spatial hash for branch dedupe
struct PtHash {
    double cell;
    std::unordered_map<int64_t, std::vector<int>> map;
    std::vector<V3> pts;
    static int64_t key(int x, int y, int z) {
        return ((int64_t)(x) & 0x1FFFFF) << 42 | ((int64_t)(y) & 0x1FFFFF) << 21 |
               ((int64_t)(z) & 0x1FFFFF);
    }
    void add(const V3& p) {
        int idx = (int)pts.size();
        pts.push_back(p);
        int x = (int)std::llround(p.x / cell), y = (int)std::llround(p.y / cell),
            z = (int)std::llround(p.z / cell);
        map[key(x, y, z)].push_back(idx);
    }
    bool near(const V3& p, double r) const {
        int x = (int)std::llround(p.x / cell), y = (int)std::llround(p.y / cell),
            z = (int)std::llround(p.z / cell);
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++) {
                    auto it = map.find(key(x + dx, y + dy, z + dz));
                    if (it == map.end()) continue;
                    for (int i : it->second)
                        if (pts[i].dist(p) < r) return true;
                }
        return false;
    }
};

struct Walker {
    const Srf& A;
    const Srf& B;
    WalkOpt o;
    PtHash walked;
    long total_pts = 0;

    Walker(const Srf& a, const Srf& b, const WalkOpt& opt)
        : A(a), B(b), o(opt) {
        // hash cell must cover the seed-skip radius for near() to be correct
        walked.cell = std::max(o.tol3d * 4, o.seed_skip) + 1e-9;
    }

    bool in_rect(double u, double v, double s, double t) const {
        // a direction whose rect spans the full period is not a boundary:
        // the walk must cross the seam freely (UVs are re-wrapped downstream)
        bool au_per = A.periodic_u() && (o.au1 - o.au0) >= TWO_PI - 1e-9;
        bool av_per = A.periodic_v() && (o.av1 - o.av0) >= TWO_PI - 1e-9;
        bool bu_per = B.periodic_u() && (o.bu1 - o.bu0) >= TWO_PI - 1e-9;
        bool bv_per = B.periodic_v() && (o.bv1 - o.bv0) >= TWO_PI - 1e-9;
        return (au_per || (u >= o.au0 - 1e-9 && u <= o.au1 + 1e-9)) &&
               (av_per || (v >= o.av0 - 1e-9 && v <= o.av1 + 1e-9)) &&
               (bu_per || (s >= o.bu0 - 1e-9 && s <= o.bu1 + 1e-9)) &&
               (bv_per || (t >= o.bv0 - 1e-9 && t <= o.bv1 + 1e-9));
    }

    // walk one direction from a seed; appends points to branch (excluding seed
    // itself for dir<0 ordering handled by caller). Sets *closed_out when the
    // walk returns to the seed (closed loop) — the caller must then NOT walk
    // the opposite direction (it would double-cover the loop).
    void walk_dir(const Seed& seed, int dir, std::vector<SecPoint>& branch,
                  bool* closed_out = nullptr) {
        double u = seed.u, v = seed.v, s = seed.s, t = seed.t;
        V3 p = seed.p;
        double bbox_diag = 1.0;
        {
            // estimate scale from seed rects
            V3 pa = A.eval(0.5 * (o.au0 + o.au1), 0.5 * (o.av0 + o.av1));
            V3 pb = B.eval(0.5 * (o.bu0 + o.bu1), 0.5 * (o.bv0 + o.bv1));
            bbox_diag = pa.dist(pb) + A.r + B.r + 1.0;
        }
        double h = bbox_diag * 0.02;
        double hmin = o.tol3d * 0.5, hmax = bbox_diag * 0.1;
        V3 tan_prev = walk_tangent(A, B, u, v, s, t);
        if (std::getenv("V3WALKDBG"))
            std::fprintf(stderr,
                         "[seed] u=%.4f v=%.4f (rect %.3f..%.3f x %.3f..%.3f) "
                         "s=%.4f t=%.4f (rect %.3f..%.3f x %.3f..%.3f) dir=%d\n",
                         u, v, o.au0, o.au1, o.av0, o.av1, s, t, o.bu0, o.bu1,
                         o.bv0, o.bv1, dir);
        if (tan_prev.norm2() < 1e-20) return;
        tan_prev = tan_prev.normalized() * dir;
        int stall = 0;
        int accepted = 0;
        int n_corr_fail = 0, n_defl_rej = 0, n_ang_rej = 0;
        const char* why = "max_pts";
        for (int i = 0; i < o.max_branch_pts; i++) {
            if (total_pts > o.max_total_pts) { why = "total_guard"; break; }
            // predictor: advance h along the curve tangent on BOTH surfaces.
            // The tangent T lies in both tangent planes (T = nA x nB), so the
            // param step is the 2-var least-squares of each surface's
            // Jacobian against T*h — the null-space direction of the 3x4
            // J = [du dv -ds -dt]. (Solving J d = T*h instead splits the step
            // into OPPOSITE motions of the two surface points and the walk
            // crawls in place.)
            V3 pa, du, dv, pb, ds, dt;
            A.d0d1(u, v, pa, du, dv);
            B.d0d1(s, t, pb, ds, dt);
            auto lstsq2 = [](const V3& c0, const V3& c1, const V3& r,
                             double& x0, double& x1) {
                double a = c0.norm2(), b = c0.dot(c1), c = c1.norm2();
                double det = a * c - b * b;
                if (det < 1e-300) { x0 = x1 = 0; return; }
                double r0 = c0.dot(r), r1 = c1.dot(r);
                x0 = (r0 * c - r1 * b) / det;
                x1 = (r1 * a - r0 * b) / det;
            };
            V3 T = tan_prev * h;
            double du_p, dv_p, ds_p, dt_p;
            lstsq2(du, dv, T, du_p, dv_p);
            lstsq2(ds, dt, T, ds_p, dt_p);
            double pu = u, pv = v, ps = s, pt = t;
            u += du_p; v += dv_p; s += ds_p; t += dt_p;
            // corrector
            V3 pc;
            if (!correct(A, B, u, v, s, t, o.tol3d * 0.1, &pc)) {
                h *= 0.5;
                u = pu; v = pv; s = ps; t = pt;
                n_corr_fail++;
                if (h < hmin || ++stall > 8) { why = "corr_stall"; break; }
                continue;
            }
            stall = 0;
            // deflection: chord midpoint corrected
            V3 mid = (p + pc) * 0.5;
            double mu = 0.5 * (pu + u), mv = 0.5 * (pv + v), ms = 0.5 * (ps + s),
                   mt = 0.5 * (pt + t);
            double defl = 0;
            {
                V3 mc = mid;
                double u2 = mu, v2 = mv, s2 = ms, t2 = mt;
                if (correct(A, B, u2, v2, s2, t2, o.tol3d * 0.1, &mc))
                    defl = (mc - mid).norm();
                else
                    defl = o.tol3d * 1000; // correction failed: overshot step,
                                           // force h down (0 would accept it)
            }
            V3 tan_new = walk_tangent(A, B, u, v, s, t);
            double ang = 0;
            if (tan_new.norm2() > 1e-20)
                // tan_prev carries the walk direction; orient tan_new the same
                // way or the angle reads pi for every backward-walk step
                ang = std::acos(std::max(-1.0, std::min(1.0,
                          (tan_new.normalized() * dir).dot(tan_prev))));
            if (defl > o.tol3d || ang > 0.5) {
                h *= 0.5;
                u = pu; v = pv; s = ps; t = pt;
                if (defl > o.tol3d) n_defl_rej++;
                else n_ang_rej++;
                if (h < hmin) { /* accept anyway */ }
                else continue;
            } else if (defl < o.tol3d * 0.15 && ang < 0.1 && h < hmax) {
                h *= 1.5;
            }
            // closure: the accepted step chord passes the branch start — the
            // loop has closed. The chord deviates from the true curve by at
            // most the deflection tolerance, so a closed loop always triggers
            // this (without it a loop is re-walked until max_branch_pts).
            // The radius must cover per-lap integration drift (~1e-5 at
            // tol3d=1e-6), hence the h-relative term.
            if (accepted > 8) {
                V3 ab = pc - p;
                double L2 = ab.norm2();
                double fr = L2 > 1e-300
                                ? std::max(0.0, std::min(1.0, (seed.p - p).dot(ab) / L2))
                                : 0.0;
                // The radius must cover per-lap integration drift (~1e-5 at
                // tol3d=1e-6), hence the h-relative term — capped, or a grown
                // step size would close loops spuriously.
                double r_close =
                    std::max(o.tol3d * 20, std::min(0.25 * h, o.tol3d * 1000));
                if ((p + ab * fr - seed.p).norm() < r_close) {
                    if (closed_out) *closed_out = true;
                    why = "closed";
                    break;
                }
            }
            // bounds: stop at rect edge
            if (!in_rect(u, v, s, t)) {
                // bisect to boundary on h
                double lo = 0, hi = 1;
                double bu = pu, bv = pv, bs = ps, bt = pt;
                for (int it = 0; it < 40; it++) {
                    double m = 0.5 * (lo + hi);
                    double u3 = pu + (u - pu) * m, v3 = pv + (v - pv) * m;
                    double s3 = ps + (s - ps) * m, t3 = pt + (t - pt) * m;
                    if (in_rect(u3, v3, s3, t3)) { lo = m; bu = u3; bv = v3; bs = s3; bt = t3; }
                    else hi = m;
                }
                V3 pb2;
                if (correct(A, B, bu, bv, bs, bt, o.tol3d * 0.1, &pb2)) {
                    SecPoint sp;
                    sp.p = pb2; sp.u1 = bu; sp.v1 = bv; sp.u2 = bs; sp.v2 = bt;
                    branch.push_back(sp);
                    walked.add(pb2);
                    total_pts++;
                }
                why = "bounds";
                break;
            }
            // closure: back near start of the whole branch?
            SecPoint sp;
            sp.p = pc; sp.u1 = u; sp.v1 = v; sp.u2 = s; sp.v2 = t;
            branch.push_back(sp);
            walked.add(pc);
            total_pts++;
            accepted++;
            if (std::getenv("V3WALKDBG2") && accepted <= 40)
                std::fprintf(stderr,
                             "  [step] i=%d p=(%.6f,%.6f,%.6f) h=%.4g defl=%.3g ang=%.3g tan=(%.3f,%.3f,%.3f)\n",
                             i, pc.x, pc.y, pc.z, h, defl, ang, tan_new.x,
                             tan_new.y, tan_new.z);
            p = pc;
            tan_prev = tan_new.norm2() > 1e-20 ? tan_new.normalized() * dir : tan_prev;
        }
        if (std::getenv("V3WALKDBG"))
            std::fprintf(stderr,
                         "[walkdir] why=%s acc=%d corr_fail=%d defl_rej=%d "
                         "ang_rej=%d h=%.2e\n",
                         why, accepted, n_corr_fail, n_defl_rej, n_ang_rej, h);
    }
};

// unwrap a branch's UVs for continuity (in place)
void unwrap_branch(std::vector<SecPoint>& br, const Srf& A, const Srf& B) {
    for (size_t i = 1; i < br.size(); i++) {
        if (A.periodic_u()) br[i].u1 = unwrap_near(br[i].u1, br[i - 1].u1);
        if (A.periodic_v()) br[i].v1 = unwrap_near(br[i].v1, br[i - 1].v1);
        if (B.periodic_u()) br[i].u2 = unwrap_near(br[i].u2, br[i - 1].u2);
        if (B.periodic_v()) br[i].v2 = unwrap_near(br[i].v2, br[i - 1].v2);
    }
}

SSIResult ssi_walk(const Srf& A, const Srf& B, const WalkOpt& o) {
    SSIResult out;
    Walker W(A, B, o);
    std::vector<Seed> seeds;
    PtHash seed_hash;
    seed_hash.cell = o.tol3d * 8 + 1e-9;

    auto add_seed = [&](double u, double v, double s, double t, const V3& p) {
        if (seed_hash.near(p, o.tol3d * 8)) return;
        seed_hash.add(p);
        seeds.push_back({u, v, s, t, p});
    };

    // --- seeding: iso-curves of A against B ---
    const int NG = 16;
    for (int dir = 0; dir < 2; dir++) {
        for (int i = 0; i <= NG; i++) {
            double iso = (dir == 0 ? o.au0 + (o.au1 - o.au0) * i / NG
                                   : o.av0 + (o.av1 - o.av0) * i / NG);
            // sample the iso-curve
            const int NS = 128;
            double prev_par = 0;
            V3 prev_p;
            double prev_F = 0, prev_d = 1e300, prev2_d = 1e300;
            double pu = 0, pv = 0, pbu = 0, pbv = 0; // sample j-1's params
            bool have_prev = false, have_prev2 = false;
            double seed_u = 0.5 * (o.bu0 + o.bu1), seed_v = 0.5 * (o.bv0 + o.bv1);
            for (int j = 0; j <= NS; j++) {
                double par = (dir == 0 ? o.av0 + (o.av1 - o.av0) * j / NS
                                       : o.au0 + (o.au1 - o.au0) * j / NS);
                double u = dir == 0 ? iso : par;
                double v = dir == 0 ? par : iso;
                V3 p = A.eval(u, v);
                if (B.has_implicit()) {
                    double F = B.F(p);
                    if (have_prev && F * prev_F < 0) {
                        // bisection on par
                        double lo = prev_par, hi = par, Flo = prev_F;
                        double u2 = u, v2 = v;
                        for (int it = 0; it < 50; it++) {
                            double m = 0.5 * (lo + hi);
                            u2 = dir == 0 ? iso : m;
                            v2 = dir == 0 ? m : iso;
                            double Fm = B.F(A.eval(u2, v2));
                            if (Fm * Flo < 0) hi = m;
                            else { lo = m; Flo = Fm; }
                        }
                        double s2 = seed_u, t2 = seed_v;
                        V3 pc;
                        if (correct(A, B, u2, v2, s2, t2, o.tol3d * 0.1, &pc))
                            add_seed(u2, v2, s2, t2, pc);
                    }
                    prev_F = F;
                } else {
                    // NURBS B: distance-to-B along the iso-curve; at a local
                    // minimum polish with the corrector. Raw iso samples sit
                    // ~spacing/2 from a crossing — far above any direct-accept
                    // gate — so minima must be refined or NURBS pairs never
                    // seed. The gate is spacing-relative: any genuine crossing
                    // has its nearest sample within half a spacing.
                    double su = seed_u, sv = seed_v;
                    if (project_nurbs_seeded(B, p, su, sv)) {
                        double d = (B.eval(su, sv) - p).norm();
                        double sp3 = have_prev ? prev_p.dist(p) : 1e300;
                        if (have_prev2 && prev_d <= prev2_d && prev_d <= d &&
                            prev_d < 4.0 * sp3) {
                            double u2 = pu, v2 = pv, s2 = pbu, t2 = pbv;
                            V3 pc;
                            // NURBS extrapolation guard: a polished seed must
                            // live inside both surface domains
                            if (correct(A, B, u2, v2, s2, t2, o.tol3d * 0.1, &pc) &&
                                W.in_rect(u2, v2, s2, t2))
                                add_seed(u2, v2, s2, t2, pc);
                        }
                        prev2_d = prev_d;
                        prev_d = d;
                        pu = u; pv = v; pbu = su; pbv = sv;
                        have_prev2 = have_prev;
                        seed_u = su; seed_v = sv; // chain seed
                    }
                }
                prev_par = par;
                prev_p = p;
                have_prev = true;
            }
        }
    }
    // --- seeding: iso-curves of B against A (catches branches missed above) ---
    if (A.has_implicit()) {
        for (int dir = 0; dir < 2; dir++) {
            for (int i = 0; i <= NG; i++) {
                double iso = (dir == 0 ? o.bu0 + (o.bu1 - o.bu0) * i / NG
                                       : o.bv0 + (o.bv1 - o.bv0) * i / NG);
                const int NS = 128;
                double prev_F = 0, prev_par = 0;
                bool have_prev = false;
                double seed_u = 0.5 * (o.au0 + o.au1), seed_v = 0.5 * (o.av0 + o.av1);
                for (int j = 0; j <= NS; j++) {
                    double par = (dir == 0 ? o.bv0 + (o.bv1 - o.bv0) * j / NS
                                           : o.bu0 + (o.bu1 - o.bu0) * j / NS);
                    double s = dir == 0 ? iso : par;
                    double t = dir == 0 ? par : iso;
                    V3 p = B.eval(s, t);
                    double F = A.F(p);
                    if (have_prev && F * prev_F < 0) {
                        double lo = prev_par, hi = par, Flo = prev_F;
                        double s2 = s, t2 = t;
                        for (int it = 0; it < 50; it++) {
                            double m = 0.5 * (lo + hi);
                            s2 = dir == 0 ? iso : m;
                            t2 = dir == 0 ? m : iso;
                            double Fm = A.F(B.eval(s2, t2));
                            if (Fm * Flo < 0) hi = m;
                            else { lo = m; Flo = Fm; }
                        }
                        double u2 = seed_u, v2 = seed_v;
                        V3 pc;
                        if (correct(A, B, u2, v2, s2, t2, o.tol3d * 0.1, &pc))
                            add_seed(u2, v2, s2, t2, pc);
                    }
                    prev_F = F;
                    prev_par = par;
                    have_prev = true;
                }
            }
        }
    }

    // --- walk each seed, both directions ---
    if (std::getenv("V3WALKDBG"))
        std::fprintf(stderr, "[walk] seeds=%zu\n", seeds.size());
    for (auto& sd : seeds) {
        // seeds on an already-walked curve are skipped; the radius must cover
        // the walk's sample spacing (~1e-3 at tol3d=1e-6 on unit geometry) or
        // every seed re-walks the same loop
        if (W.walked.near(sd.p, o.seed_skip)) continue;
        std::vector<SecPoint> fwd, bwd;
        bool closed = false;
        W.walk_dir(sd, +1, fwd, &closed);
        if (!closed) W.walk_dir(sd, -1, bwd); // closed loop: no second sweep
        if (fwd.size() + bwd.size() + 1 < 3) continue;
        SecCurve sc;
        sc.has_exact = false;
        // order: reverse(bwd), seed, fwd
        for (int i = (int)bwd.size() - 1; i >= 0; i--) sc.pts.push_back(bwd[i]);
        SecPoint sp0;
        sp0.p = sd.p; sp0.u1 = sd.u; sp0.v1 = sd.v; sp0.u2 = sd.s; sp0.v2 = sd.t;
        sc.pts.push_back(sp0);
        for (auto& q : fwd) sc.pts.push_back(q);
        // closure?
        if (closed) {
            sc.closed = true;
        } else if (sc.pts.size() > 8 &&
                   sc.pts.front().p.dist(sc.pts.back().p) < o.tol3d * 8) {
            sc.closed = true;
            sc.pts.pop_back();
        }
        // drop zero-length spurs (tangent-grazing seeds): they only produce
        // degenerate segments downstream
        double len = 0;
        for (size_t i = 1; i < sc.pts.size(); i++)
            len += sc.pts[i].p.dist(sc.pts[i - 1].p);
        if (len < o.tol3d * 20) continue;
        if (std::getenv("V3WALKDBG")) {
            V3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
            for (auto& q : sc.pts) {
                lo.x = std::min(lo.x, q.p.x); hi.x = std::max(hi.x, q.p.x);
                lo.y = std::min(lo.y, q.p.y); hi.y = std::max(hi.y, q.p.y);
                lo.z = std::min(lo.z, q.p.z); hi.z = std::max(hi.z, q.p.z);
            }
            std::fprintf(stderr,
                         "[walk] pts=%zu closed=%d len=%.4f bbox=(%.3f..%.3f, "
                         "%.3f..%.3f, %.3f..%.3f)\n",
                         sc.pts.size(), (int)sc.closed, len, lo.x, hi.x, lo.y,
                         hi.y, lo.z, hi.z);
        }
        unwrap_branch(sc.pts, A, B);
        out.curves.push_back(std::move(sc));
        if (W.total_pts > o.max_total_pts) break;
    }
    return out;
}

void natural_rect(const Srf& s, double other_bbox[6], double& u0, double& u1,
                  double& v0, double& v1) {
    s.canonical_domain(u0, u1, v0, v1);
    if (s.k == Srf::NURBS) return;
    // bound unbounded directions by the other surface's bbox (expanded)
    double cx = 0.5 * (other_bbox[0] + other_bbox[3]);
    double cy = 0.5 * (other_bbox[1] + other_bbox[4]);
    double cz = 0.5 * (other_bbox[2] + other_bbox[5]);
    double dx = 0.5 * (other_bbox[3] - other_bbox[0]);
    double dy = 0.5 * (other_bbox[4] - other_bbox[1]);
    double dz = 0.5 * (other_bbox[5] - other_bbox[2]);
    double reach = std::sqrt(dx * dx + dy * dy + dz * dz) + s.r + s.r2 + 1.0;
    V3 c{cx, cy, cz};
    if (s.k == Srf::PLANE) {
        // project bbox center + reach onto plane axes
        V3 w = c - s.f.o;
        double uc = w.dot(s.f.x), vc = w.dot(s.f.y);
        u0 = uc - reach; u1 = uc + reach; v0 = vc - reach; v1 = vc + reach;
    } else if (s.k == Srf::CYLINDER) {
        double vc = (c - s.f.o).dot(s.f.z);
        v0 = vc - reach; v1 = vc + reach;
    } else if (s.k == Srf::CONE) {
        double vc = (c - s.f.o).dot(s.f.z);
        v0 = std::max(0.0, vc - reach * 2);
        v1 = vc + reach * 2;
    }
}

void bbox_of(const Srf& s, double bb[6]) {
    double u0, u1, v0, v1;
    s.canonical_domain(u0, u1, v0, v1);
    if (s.k != Srf::NURBS) {
        // analytic unbounded: produce a finite box from characteristic sizes
        double R = s.r + s.r2 + 5.0;
        V3 c = s.f.o;
        bb[0] = c.x - R; bb[3] = c.x + R;
        bb[1] = c.y - R; bb[4] = c.y + R;
        bb[2] = c.z - R; bb[5] = c.z + R;
        if (s.k == Srf::SPHERE || s.k == Srf::TORUS) return;
        // cylinder/cone/plane: extend along axis generously
        bb[0] -= 10 * std::abs(s.f.z.x); bb[3] += 10 * std::abs(s.f.z.x);
        bb[1] -= 10 * std::abs(s.f.z.y); bb[4] += 10 * std::abs(s.f.z.y);
        bb[2] -= 10 * std::abs(s.f.z.z); bb[5] += 10 * std::abs(s.f.z.z);
        return;
    }
    bb[0] = bb[1] = bb[2] = 1e300;
    bb[3] = bb[4] = bb[5] = -1e300;
    for (int i = 0; i <= 16; i++)
        for (int j = 0; j <= 16; j++) {
            V3 p = s.eval(u0 + (u1 - u0) * i / 16, v0 + (v1 - v0) * j / 16);
            bb[0] = std::min(bb[0], p.x); bb[3] = std::max(bb[3], p.x);
            bb[1] = std::min(bb[1], p.y); bb[4] = std::max(bb[4], p.y);
            bb[2] = std::min(bb[2], p.z); bb[5] = std::max(bb[5], p.z);
        }
}

} // namespace

SSIResult ssi(const Srf& a, const Srf& b, double tol3d) {
    SSIResult out;
    if (ssi_exact(a, b, out, tol3d)) return out;
    WalkOpt o;
    o.tol3d = tol3d;
    double bba[6], bbb[6];
    bbox_of(a, bba);
    bbox_of(b, bbb);
    // overlap box
    double ov[6] = {std::max(bba[0], bbb[0]), std::max(bba[1], bbb[1]),
                    std::max(bba[2], bbb[2]), std::min(bba[3], bbb[3]),
                    std::min(bba[4], bbb[4]), std::min(bba[5], bbb[5])};
    if (ov[3] < ov[0] || ov[4] < ov[1] || ov[5] < ov[2]) return out; // disjoint
    natural_rect(a, ov, o.au0, o.au1, o.av0, o.av1);
    natural_rect(b, ov, o.bu0, o.bu1, o.bv0, o.bv1);
    {
        // seed-skip radius: a seed sitting on an already-walked curve can be
        // up to ~half the walk's sample spacing from the nearest walked point
        V3 pa = a.eval(0.5 * (o.au0 + o.au1), 0.5 * (o.av0 + o.av1));
        V3 pb = b.eval(0.5 * (o.bu0 + o.bu1), 0.5 * (o.bv0 + o.bv1));
        double diag = pa.dist(pb) + a.r + b.r + 1.0;
        o.seed_skip = std::max(tol3d * 8, 0.005 * diag);
    }
    return ssi_walk(a, b, o);
}

// ============================================================================
// curve-surface intersection
// ============================================================================

std::vector<CSIPoint> csi(const Cur& c, double t0, double t1, const Srf& s,
                          double tol3d) {
    std::vector<CSIPoint> out;
    const int NS = 256;
    double prev_t = t0;
    V3 prev_p = c.eval(t0);
    double prev_F = s.has_implicit() ? s.F(prev_p) : 0;
    double prev_d = 1e300, prev2_d = 1e300;
    double prev_su = 0, prev_sv = 0;
    double track_min = 1e300;
    int cands = 0;
    double du0, du1, dv0, dv1;
    s.canonical_domain(du0, du1, dv0, dv1);
    double seed_u = 0.5 * (du0 + du1), seed_v = 0.5 * (dv0 + dv1);
    for (int i = 1; i <= NS; i++) {
        double t = t0 + (t1 - t0) * i / NS;
        V3 p = c.eval(t);
        if (s.has_implicit()) {
            double F = s.F(p);
            double feps = 1e-11 * (1.0 + std::abs(s.r));
            // sign change, or an exact zero on the sample (product would be 0)
            if (F * prev_F < 0 || (std::abs(F) < feps && std::abs(prev_F) > feps)) {
                double lo = prev_t, hi = t, Flo = prev_F;
                for (int it = 0; it < 60; it++) {
                    double m = 0.5 * (lo + hi);
                    double Fm = s.F(c.eval(m));
                    if (Fm * Flo < 0) hi = m;
                    else { lo = m; Flo = Fm; }
                }
                double tm = 0.5 * (lo + hi);
                V3 pm = c.eval(tm);
                CSIPoint cp;
                cp.t = tm; cp.p = pm;
                if (s.uv_of(pm, cp.u, cp.v)) out.push_back(cp);
            }
            prev_F = F;
        } else {
            // grid-reseeded projection every sample: the chained-seed Newton
            // loses the point whenever the previous sample projected far away
            // (domain-center seed on the first sample, curved freeform
            // surfaces), leaving holes in the distance track so no minimum is
            // ever found (chairs: 0 EF pierces). 9x9 reseed + polish is what
            // Srf::uv_of does, but uv_of's <1e-7 proximity return rejects
            // off-surface samples — csi needs best-effort distances.
            double uu = 0, vv = 0, bd = 1e300;
            for (int gi = 0; gi <= 8; gi++)
                for (int gj = 0; gj <= 8; gj++) {
                    double gu = du0 + (du1 - du0) * gi / 8.0;
                    double gv = dv0 + (dv1 - dv0) * gj / 8.0;
                    double d2 = (s.eval(gu, gv) - p).norm2();
                    if (d2 < bd) { bd = d2; uu = gu; vv = gv; }
                }
            if (project_nurbs_seeded(s, p, uu, vv, 40)) {
                double d = (s.eval(uu, vv) - p).norm();
                track_min = std::min(track_min, d);
                seed_u = uu; seed_v = vv;
                // local minimum below 4x the local sample spacing: raw samples
                // sit ~spacing/2 from a crossing — far above any direct-accept
                // gate — so minima must be polished (same fix as the walker's
                // NURBS iso seeding; without it EF pierces never fire on
                // freeform surfaces and section endpoints stay unanchored).
                double sp = prev_p.dist(p);
                if (prev_d <= prev2_d && prev_d <= d && prev_d < 4.0 * sp) {
                    cands++;
                    // local minimum behind us: polish at i-1
                    double tm = t - (t1 - t0) / NS;
                    V3 pm = c.eval(tm);
                    double u2 = prev_su, v2 = prev_sv;
                    // 3-var Newton on |C(t) - S(u,v)|^2
                    for (int it = 0; it < 30; it++) {
                        V3 sp, du, dv;
                        s.d0d1(u2, v2, sp, du, dv);
                        V3 ct = c.d1(tm);
                        V3 r = c.eval(tm) - sp;
                        // J: [ct, -du, -dv] (3x3)
                        double A[3][3] = {{ct.x, -du.x, -dv.x},
                                          {ct.y, -du.y, -dv.y},
                                          {ct.z, -du.z, -dv.z}};
                        // solve A d = r via normal equations
                        double ata[3][3] = {}, atb[3] = {};
                        for (int a2 = 0; a2 < 3; a2++)
                            for (int b2 = 0; b2 < 3; b2++)
                                for (int k = 0; k < 3; k++)
                                    ata[a2][b2] += A[k][a2] * A[k][b2];
                        double rv[3] = {r.x, r.y, r.z};
                        for (int a2 = 0; a2 < 3; a2++)
                            for (int k = 0; k < 3; k++) atb[a2] -= A[k][a2] * rv[k];
                        ata[0][0] += 1e-14; ata[1][1] += 1e-14; ata[2][2] += 1e-14;
                        // solve 3x3
                        bool ok = true;
                        for (int col = 0; col < 3 && ok; col++) {
                            int piv = col;
                            for (int r2 = col + 1; r2 < 3; r2++)
                                if (std::abs(ata[r2][col]) > std::abs(ata[piv][col]))
                                    piv = r2;
                            if (std::abs(ata[piv][col]) < 1e-300) { ok = false; break; }
                            for (int c2 = col; c2 < 3; c2++) std::swap(ata[piv][c2], ata[col][c2]);
                            std::swap(atb[piv], atb[col]);
                            for (int r2 = col + 1; r2 < 3; r2++) {
                                double f = ata[r2][col] / ata[col][col];
                                for (int c2 = col; c2 < 3; c2++) ata[r2][c2] -= f * ata[col][c2];
                                atb[r2] -= f * atb[col];
                            }
                        }
                        if (!ok) break;
                        double d_[3];
                        for (int a2 = 2; a2 >= 0; a2--) {
                            double sum = atb[a2];
                            for (int b2 = a2 + 1; b2 < 3; b2++) sum -= ata[a2][b2] * d_[b2];
                            d_[a2] = sum / ata[a2][a2];
                        }
                        tm += d_[0]; u2 += d_[1]; v2 += d_[2];
                        if (std::abs(d_[0]) + std::abs(d_[1]) + std::abs(d_[2]) < 1e-13) break;
                    }
                    V3 pm2 = c.eval(tm);
                    double pol = (pm2 - s.eval(u2, v2)).norm();
                    if (std::getenv("V3CSIDBG"))
                        std::fprintf(stderr, "[csi]   polish raw=%.3g -> %.3g\n",
                                     prev_d, pol);
                    if (pol < tol3d * 10) {
                        CSIPoint cp;
                        cp.t = tm; cp.p = pm2; cp.u = u2; cp.v = v2;
                        out.push_back(cp);
                    }
                }
                prev2_d = prev_d;
                prev_d = d;
                prev_su = uu;
                prev_sv = vv;
            }
        }
        prev_t = t;
        prev_p = p;
    }
    // sort by t and dedupe
    if (std::getenv("V3CSIDBG")) {
        std::fprintf(stderr, "[csi] ckind=%d skind=%d out=%zu trackmin=%.3g cands=%d\n",
                     (int)c.k, (int)s.k, out.size(), track_min, cands);
    }
    std::sort(out.begin(), out.end(), [](const CSIPoint& a, const CSIPoint& b) {
        return a.t < b.t;
    });
    std::vector<CSIPoint> ded;
    for (auto& p : out)
        if (ded.empty() || std::abs(p.t - ded.back().t) > 1e-9) ded.push_back(p);
    return ded;
}

} // namespace v3
