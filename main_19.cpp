// main_19 — V2 CLOSURE-RESIDUAL diagnostic driver.
//
// closure_residual = |sum_f outward_f * II (Su x Sv) du dv| / area  (brep_massprops).
//
// KEY IDENTITY THIS DRIVER EXPLOITS. For ANY surface patch, II (Su x Sv) du dv over the trimmed
// region equals 1/2 * oint r x dr taken over the patch's 3D BOUNDARY IMAGE (the pcurve pushed
// through the surface). So the sum over a topologically closed shell telescopes edge by edge and
// is EXACTLY zero iff, for every edge, the two faces' boundary IMAGES coincide geometrically.
// A nonzero residual therefore localises to EDGES whose two trim images disagree — not to a face.
//
// Parts:
//   A  controls: hand-built primitives (residual must be ~1e-16)
//   B  tilt sweep, official verdict (v2v::v2_verdict) + per-face vec_area/path/chain_gap table
//   C  per-EDGE trim-image mismatch: the two trims of every edge sampled through their own
//      surfaces, compared in 3D, and each edge's vector-area defect  |A1 + A2|  computed. The
//      sum of those defects reproduces the residual numerator, which proves the localisation.
//
// Env: SESSION_V19_NT   tilt steps (default 20 over 0..45 deg)
//      SESSION_V19_TILT run ONE tilt (degrees), full per-face + per-edge dump
//      SESSION_V19_OP   cut|common|both (default both)
//      SESSION_V19_TOPN how many worst edges to print (default 8)

#include "src/brep.h"
#include "src/tolerance.h"
#include "src/brep_massprops.h"
#include "src/nurbscurve.h"
#include "src/nurbssurface.h"
#include "src/v2/brep_v2_boolean.h"
#include "src/v2/brep_v2_solid.h"
#include "src/v2/v2_verdict.h"
#include "src/xform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;
using session_cpp::v2v::v2_verdict;
using session_cpp::v2v::v2_verdict_options;
using session_cpp::v2sol::v2_common;
using session_cpp::v2sol::v2_cut;
using session_cpp::v2sol::V2BooleanReport;

static int g_pass = 0, g_fail = 0;
static void cell(const std::string& n, bool ok, const std::string& d = "") {
    std::printf("[V19] %-54s %s%s%s\n", n.c_str(), ok ? "PASS" : "FAIL", d.empty() ? "" : "  ",
                d.c_str());
    ok ? ++g_pass : ++g_fail;
}

static BRep moved(const BRep& b, const Xform& x) {
    BRep c = b;
    c.xform = x;
    return c.transformed();
}

///////////////////////////////////////////////////////////////////////////////////////////
// per-edge vector-area defect
///////////////////////////////////////////////////////////////////////////////////////////

struct EdgeDefect {
    int edge = -1;
    int trims = 0;
    double len = 0.0;
    double defect = 0.0;       ///< |sum over trims of oriented 1/2 oint r x dr|
    double max_dev = 0.0;      ///< Hausdorff gap between the two faces' boundary images
    double max_dev_3d = 0.0;   ///< Hausdorff gap trim image vs stored edge 3D curve
    double esph = 0.0;         ///< analytic oracle: max | |p| - R |
    double ecyl = 0.0;         ///< analytic oracle: max | dist(p,axis) - r |
    int f0 = -1, f1 = -1;
    bool degenerate = false;
};

/// Sample one trim's 3D image through ITS OWN face surface. Returns the polyline in the
/// direction the LOOP traverses it, and the oriented half-cross-product vector area.
static void trim_image(const BRep& b, int trim, int nsamp, std::vector<Point>& out) {
    out.clear();
    const BRepTrim& t = b.m_trims[trim];
    if (t.curve_2d_index < 0 || t.curve_2d_index >= (int)b.m_curves_2d.size()) return;
    const BRepLoop& lp = b.m_loops[t.loop_index];
    if (lp.face_index < 0 || lp.face_index >= (int)b.m_faces.size()) return;
    const BRepFace& f = b.m_faces[lp.face_index];
    if (f.surface_index < 0 || f.surface_index >= (int)b.m_surfaces.size()) return;
    const NurbsSurface& S = b.m_surfaces[f.surface_index];
    const NurbsCurve& c2 = b.m_curves_2d[t.curve_2d_index];
    const double t0 = c2.domain_start(), t1 = c2.domain_end();
    out.reserve(nsamp + 1);
    for (int i = 0; i <= nsamp; ++i) {
        const double s = (double)i / (double)nsamp;
        const double tt = t.reversed ? t1 + (t0 - t1) * s : t0 + (t1 - t0) * s;
        const Point uv = c2.point_at(tt);
        out.push_back(S.point_at(uv[0], uv[1]));
    }
}

static void vec_area_of(const std::vector<Point>& p, double a[3]) {
    a[0] = a[1] = a[2] = 0.0;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        const Point& u = p[i];
        const Point& v = p[i + 1];
        a[0] += 0.5 * (u[1] * v[2] - u[2] * v[1]);
        a[1] += 0.5 * (u[2] * v[0] - u[0] * v[2]);
        a[2] += 0.5 * (u[0] * v[1] - u[1] * v[0]);
    }
}

static double plen(const std::vector<Point>& p) {
    double L = 0;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        const double dx = p[i + 1][0] - p[i][0], dy = p[i + 1][1] - p[i][1],
                     dz = p[i + 1][2] - p[i][2];
        L += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return L;
}

/// Parameterisation-free deviation: max over samples of a of the distance to the POLYLINE b.
static double hausdorff_1way(const std::vector<Point>& a, const std::vector<Point>& b) {
    double worst = 0.0;
    for (const Point& p : a) {
        double best = 1e300;
        for (size_t i = 0; i + 1 < b.size(); ++i) {
            const double ex = b[i + 1][0] - b[i][0], ey = b[i + 1][1] - b[i][1],
                         ez = b[i + 1][2] - b[i][2];
            const double L2 = ex * ex + ey * ey + ez * ez;
            double t = 0.0;
            if (L2 > 1e-300)
                t = ((p[0] - b[i][0]) * ex + (p[1] - b[i][1]) * ey + (p[2] - b[i][2]) * ez) / L2;
            t = std::max(0.0, std::min(1.0, t));
            const double dx = p[0] - (b[i][0] + t * ex), dy = p[1] - (b[i][1] + t * ey),
                         dz = p[2] - (b[i][2] + t * ez);
            best = std::min(best, dx * dx + dy * dy + dz * dz);
        }
        worst = std::max(worst, std::sqrt(best));
    }
    return worst;
}

/// ANALYTIC ORACLE for this sweep: sphere |p| = R, cylinder dist(p, axis) = r. A section point
/// must satisfy BOTH. Set by main() before the dumps.
static double g_R = 2.5, g_r = 1.0;
static Vector g_axis(0, 0, 1);

static void oracle_err(const std::vector<Point>& p, double& esph, double& ecyl) {
    esph = ecyl = 0.0;
    for (const Point& q : p) {
        esph = std::max(esph, std::fabs(std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]) - g_R));
        const double d = q[0] * g_axis[0] + q[1] * g_axis[1] + q[2] * g_axis[2];
        const double px = q[0] - d * g_axis[0], py = q[1] - d * g_axis[1], pz = q[2] - d * g_axis[2];
        ecyl = std::max(ecyl, std::fabs(std::sqrt(px * px + py * py + pz * pz) - g_r));
    }
}

/// Per-edge defect table. `outward` is the per-face outward sign from brep_massprops (the same
/// sign it uses to sum vector areas), so the defects sum to the residual numerator.
static std::vector<EdgeDefect> edge_defects(const BRep& b, const std::vector<double>& outward,
                                            int nsamp, double tot[3]) {
    std::vector<EdgeDefect> out;
    tot[0] = tot[1] = tot[2] = 0.0;
    for (int e = 0; e < (int)b.m_topology_edges.size(); ++e) {
        const BRepEdge& E = b.m_topology_edges[e];
        if (E.trim_indices.empty()) continue;
        EdgeDefect d;
        d.edge = e;
        d.trims = (int)E.trim_indices.size();
        double acc[3] = {0, 0, 0};
        std::vector<std::vector<Point>> imgs;
        for (int ti : E.trim_indices) {
            const BRepTrim& T = b.m_trims[ti];
            const int face = (T.loop_index >= 0) ? b.m_loops[T.loop_index].face_index : -1;
            std::vector<Point> img;
            trim_image(b, ti, nsamp, img);
            if (img.empty()) continue;
            double a[3];
            vec_area_of(img, a);
            const double sgn = (face >= 0 && face < (int)outward.size()) ? outward[face] : 1.0;
            acc[0] += sgn * a[0];
            acc[1] += sgn * a[1];
            acc[2] += sgn * a[2];
            if (d.f0 < 0) d.f0 = face; else if (d.f1 < 0) d.f1 = face;
            d.len = std::max(d.len, plen(img));
            imgs.push_back(std::move(img));
        }
        d.defect = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        tot[0] += acc[0];
        tot[1] += acc[1];
        tot[2] += acc[2];
        // GEOMETRIC gap between the two faces' boundary images, parameterisation-free.
        if (imgs.size() == 2)
            d.max_dev = std::max(hausdorff_1way(imgs[0], imgs[1]), hausdorff_1way(imgs[1], imgs[0]));
        // same-parameter: trim image vs the stored 3D curve, parameterisation-free
        if (E.curve_3d_index >= 0 && E.curve_3d_index < (int)b.m_curves_3d.size() &&
            !imgs.empty()) {
            const NurbsCurve& C = b.m_curves_3d[E.curve_3d_index];
            const double c0 = C.domain_start(), c1 = C.domain_end();
            std::vector<Point> cp;
            for (int i = 0; i <= nsamp; ++i)
                cp.push_back(C.point_at(c0 + (c1 - c0) * (double)i / (double)nsamp));
            double best = 1e300;
            for (const auto& img : imgs)
                best = std::min(best, std::max(hausdorff_1way(img, cp), hausdorff_1way(cp, img)));
            d.max_dev_3d = best;
        }
        if (!imgs.empty()) oracle_err(imgs[0], d.esph, d.ecyl);
        if (d.len < 1e-12) d.degenerate = true;
        out.push_back(d);
    }
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////

static const char* path_name(MassPropsPath p) {
    switch (p) {
        case MassPropsPath::Skipped: return "skip";
        case MassPropsPath::PlanarExact: return "plan";
        case MassPropsPath::GreenUV: return "green";
        case MassPropsPath::FullDomain: return "FULL";
    }
    return "?";
}

static void dump_faces(const BRep& b, const MassProps& mp) {
    double sum[3] = {0, 0, 0};
    for (const auto& f : mp.faces) {
        sum[0] += f.outward * f.vec_area[0];
        sum[1] += f.outward * f.vec_area[1];
        sum[2] += f.outward * f.vec_area[2];
    }
    std::printf("[V19]   residual vec = (%.3e %.3e %.3e)  |.|=%.3e  area=%.6f  resid=%.3e\n",
                sum[0], sum[1], sum[2],
                std::sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]), mp.area,
                mp.closure_residual);
    std::printf("[V19]   %-4s %-6s %-10s %-11s %-11s %-11s %-9s %-6s %-4s %s\n", "f", "path",
                "area", "vecA.x", "vecA.y", "vecA.z", "chaingap", "outw", "surf", "|bnd-vecA|");
    for (const auto& f : mp.faces) {
        const int si = (f.face_index >= 0 && f.face_index < (int)b.m_faces.size())
                           ? b.m_faces[f.face_index].surface_index : -1;
        // Independent check of the SAME quantity from the face's own boundary images
        // (Stokes: II (Su x Sv) = 1/2 oint r x dr). A disagreement means the sampling
        // convention below is wrong, not the geometry.
        double bnd[3] = {0, 0, 0};
        if (f.face_index >= 0 && f.face_index < (int)b.m_faces.size()) {
            for (int li : b.m_faces[f.face_index].loop_indices) {
                if (li < 0 || li >= (int)b.m_loops.size()) continue;
                for (int ti : b.m_loops[li].trim_indices) {
                    std::vector<Point> img;
                    trim_image(b, ti, 256, img);
                    if (img.empty()) continue;
                    double a[3];
                    vec_area_of(img, a);
                    bnd[0] += a[0]; bnd[1] += a[1]; bnd[2] += a[2];
                }
            }
        }
        const double dif = std::sqrt((bnd[0] - f.vec_area[0]) * (bnd[0] - f.vec_area[0]) +
                                     (bnd[1] - f.vec_area[1]) * (bnd[1] - f.vec_area[1]) +
                                     (bnd[2] - f.vec_area[2]) * (bnd[2] - f.vec_area[2]));
        std::printf("[V19]   %-4d %-6s %-10.6f %-11.3e %-11.3e %-11.3e %-9.2e %-6.0f %-4d %.3e\n",
                    f.face_index, path_name(f.path), f.area, f.outward * f.vec_area[0],
                    f.outward * f.vec_area[1], f.outward * f.vec_area[2], f.chain_gap, f.outward,
                    si, dif);
    }
}

static void dump_edges(const BRep& b, const MassProps& mp, int nsamp, int topn) {
    std::vector<double> outward(b.m_faces.size(), 1.0);
    for (const auto& f : mp.faces)
        if (f.face_index >= 0 && f.face_index < (int)outward.size()) outward[f.face_index] = f.outward;
    double tot[3];
    std::vector<EdgeDefect> d = edge_defects(b, outward, nsamp, tot);
    std::printf("[V19]   edge-defect SUM = (%.3e %.3e %.3e) |.|=%.3e  (chordal n=%d)\n", tot[0],
                tot[1], tot[2], std::sqrt(tot[0] * tot[0] + tot[1] * tot[1] + tot[2] * tot[2]),
                nsamp);
    std::sort(d.begin(), d.end(),
              [](const EdgeDefect& a, const EdgeDefect& b2) { return a.defect > b2.defect; });
    std::printf("[V19]   %-5s %-4s %-5s %-10s %-11s %-11s %-11s %-11s %-11s %s\n", "edge", "ntr",
                "deg", "len", "defect", "gap(pair)", "gap(vs c3d)", "err|p|-R", "err d-r", "faces");
    for (int i = 0; i < (int)d.size() && i < topn; ++i) {
        const EdgeDefect& e = d[i];
        std::printf("[V19]   %-5d %-4d %-5d %-10.6f %-11.3e %-11.3e %-11.3e %-11.3e %-11.3e "
                    "%d,%d\n",
                    e.edge, e.trims, (int)e.degenerate, e.len, e.defect, e.max_dev, e.max_dev_3d,
                    e.esph, e.ecyl, e.f0, e.f1);
    }
    // PER-TRIM: which chart, what UV window, is the image ON both analytic surfaces, and does the
    // pcurve leave its surface's DOMAIN (a clamped NURBS extrapolates there -- off-surface).
    std::printf("[V19]   %-5s %-5s %-4s %-4s %-21s %-21s %-11s %-11s %-9s %s\n", "edge", "trim",
                "face", "srf", "u[lo,hi]", "v[lo,hi]", "err|p|-R", "err d-r", "outside", "deg1?");
    for (int i = 0; i < (int)d.size() && i < topn; ++i) {
        const int e = d[i].edge;
        for (int ti : b.m_topology_edges[e].trim_indices) {
            const BRepTrim& T = b.m_trims[ti];
            const int face = (T.loop_index >= 0) ? b.m_loops[T.loop_index].face_index : -1;
            const int si = (face >= 0) ? b.m_faces[face].surface_index : -1;
            std::vector<Point> img;
            trim_image(b, ti, nsamp, img);
            if (img.empty() || si < 0) continue;
            const NurbsCurve& c2 = b.m_curves_2d[T.curve_2d_index];
            const auto cd = c2.domain();
            double ulo = 1e300, uhi = -1e300, vlo = 1e300, vhi = -1e300;
            for (int k = 0; k <= 128; ++k) {
                const Point uv = c2.point_at(cd.first + (cd.second - cd.first) * (k / 128.0));
                ulo = std::min(ulo, uv[0]); uhi = std::max(uhi, uv[0]);
                vlo = std::min(vlo, uv[1]); vhi = std::max(vhi, uv[1]);
            }
            const NurbsSurface& S = b.m_surfaces[si];
            const auto du = S.domain(0), dv = S.domain(1);
            const double outside =
                std::max(std::max(du.first - ulo, uhi - du.second),
                         std::max(dv.first - vlo, vhi - dv.second));
            double es, ec;
            oracle_err(img, es, ec);
            std::printf("[V19]   %-5d %-5d %-4d %-4d [%8.5f,%8.5f] [%8.5f,%8.5f] %-11.3e %-11.3e "
                        "%-9.2e %d\n",
                        e, ti, face, si, ulo, uhi, vlo, vhi, es, ec, outside, c2.degree());
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

struct Row {
    double deg = 0;
    double cut_v = 0, com_v = 0, cut_r = 1, com_r = 1;
    int cut_nk = 0, com_nk = 0, cut_f = 0, com_f = 0;
    bool cut_cl = false, com_cl = false;
};

static void one(const BRep& S, const BRep& C, double deg, bool full, int topn) {
    const MassPropsOptions o = v2_verdict_options();
    g_axis = Vector(std::sin(deg * Tolerance::PI / 180.0), 0.0, std::cos(deg * Tolerance::PI / 180.0));
    const char* opsel = std::getenv("SESSION_V19_OP");
    const bool do_cut = !opsel || std::strcmp(opsel, "common") != 0;
    const bool do_com = !opsel || std::strcmp(opsel, "cut") != 0;
    const int ns = std::getenv("SESSION_V19_NS") ? std::atoi(std::getenv("SESSION_V19_NS")) : 256;

    struct Item { const char* nm; BRep b; V2BooleanReport rep; };
    std::vector<Item> items;
    if (do_cut) { items.push_back({"cut   ", BRep(), {}}); items.back().b = v2_cut(S, C, 0.0, &items.back().rep); }
    if (do_com) { items.push_back({"common", BRep(), {}}); items.back().b = v2_common(S, C, 0.0, &items.back().rep); }
    for (auto& it : items) {
        if (full) std::printf("[V19]   report %s: %s\n", it.nm, it.rep.str().c_str());
        const auto v = v2_verdict(it.b);
        const MassProps mp = brep_massprops(it.b, o);
        std::printf("[V19] tilt %6.2f %s F=%d E=%d nk=%d nm=%d seam=%d deg=%d sh=%d "
                    "vol=%.9f resid=%.3e chaingap=%.2e closed=%d conv=%d\n",
                    deg, it.nm, v.faces, v.edges, v.naked_real, v.nonmanifold, v.seam_edges,
                    v.degenerate, v.shells, mp.volume, mp.closure_residual, mp.max_chain_gap,
                    (int)v.closed(), (int)mp.converged);
        if (full) {
            dump_faces(it.b, mp);
            dump_edges(it.b, mp, ns, topn);
        }
        std::fflush(stdout);
    }
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const MassPropsOptions o = v2_verdict_options();
    const int topn = std::getenv("SESSION_V19_TOPN") ? std::atoi(std::getenv("SESSION_V19_TOPN")) : 8;

    ///////////////////////////////////////////////////////////////////////////////////////
    std::printf("\n=== PART A  CONTROLS (hand-built primitives; residual must be ~1e-16) ===\n");
    {
        struct C { const char* n; BRep b; };
        std::vector<C> cs;
        cs.push_back({"sphere r=2.5", BRep::create_sphere(2.5)});
        cs.push_back({"cylinder r=1 h=8", BRep::create_cylinder(1.0, 8.0)});
        cs.push_back({"box 2x2x2", BRep::create_box(1, 1, 1)});
        cs.push_back({"cone r=1 h=2", BRep::create_cone(1.0, 2.0)});
        bool all = true;
        for (auto& c : cs) {
            const auto v = v2_verdict(c.b);
            const MassProps mp = brep_massprops(c.b, o);
            const bool ok = v.closed();
            std::printf("[V19]   %-20s F=%d nk=%d resid=%.3e vol=%.9f is_solid=%d closed=%d\n",
                        c.n, v.faces, v.naked_real, mp.closure_residual, mp.volume,
                        (int)c.b.is_solid(), (int)ok);
            if (!ok) all = false;
        }
        cell("controls_hand_built_primitives_close", all);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    const BRep S = BRep::create_sphere(2.5);
    const BRep C0 = moved(BRep::create_cylinder(1.0, 8.0), Xform::translation(0, 0, -4.0));
    const double want_common = 4.0 * Tolerance::PI / 3.0 * (15.625 - std::pow(5.25, 1.5));
    const double want_cut = 4.0 * Tolerance::PI / 3.0 * 15.625 - want_common;
    std::printf("\n=== PART B  TILT SWEEP  analytic common=%.9f cut=%.9f ===\n", want_common,
                want_cut);

    const char* one_tilt = std::getenv("SESSION_V19_TILT");
    if (one_tilt) {
        const double deg = std::atof(one_tilt);
        Vector ay(0, 1, 0);
        const BRep C = moved(C0, Xform::rotation(ay, deg * Tolerance::PI / 180.0, false));
        one(S, C, deg, true, topn);
        std::printf("\n[V19] TOTAL %d/%d\n", g_pass, g_pass + g_fail);
        return g_fail == 0 ? 0 : 1;
    }

    const int nt = std::getenv("SESSION_V19_NT") ? std::max(2, std::atoi(std::getenv("SESSION_V19_NT")))
                                                 : 20;
    for (int i = 0; i < nt; ++i) {
        const double deg = 45.0 * i / (double)(nt - 1);
        Vector ay(0, 1, 0);
        const BRep C = moved(C0, Xform::rotation(ay, deg * Tolerance::PI / 180.0, false));
        one(S, C, deg, false, topn);
    }

    std::printf("\n[V19] TOTAL %d/%d\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
