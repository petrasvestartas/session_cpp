#include "brep_v2_boolean.h"

#include "brep_samedomain.h"
#include "brep_section.h"
#include "brep_bds.h"
#include "brep_v2_section.h"
#include "brep_v2_splitface.h"
#include "v2_verdict.h"
#include "intersection.h"
#include "nurbscurve.h"
#include "nurbssurface.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <memory>
#include <map>
#include <set>

// Stages 1/2/3/5 are owned by other agents (v2int / v2sec / v2sf). Their headers are DETECTED
// here but deliberately not included yet: brep_v2_interf.h and brep_v2_section.h each declare a
// different `V2State` in the same namespace, so a translation unit that pulls in both does not
// compile. The switchover is a one-line include per stage once those two agree.
#if defined(__has_include)
#if __has_include("brep_v2_interf.h")
#define V2SOL_HAVE_INTERF 1
#endif
#if __has_include("brep_v2_section.h")
#define V2SOL_HAVE_SECTION 1
#endif
#if __has_include("brep_v2_splitface.h")
#define V2SOL_HAVE_SPLITFACE 1
#endif
#endif

namespace session_cpp {
namespace v2sol {

bool v2_enabled() {
    static const bool on = (std::getenv("SESSION_V2") != nullptr);
    return on;
}
static bool v2sol_dbg() {
    static const bool on = (std::getenv("SESSION_V2_DBG") != nullptr);
    return on;
}

std::string V2BooleanReport::str() const {
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "pairs=%d sdsurf=%d sec=%d fA=%d fB=%d inA=%d inB=%d on=%d sd=%d sel=%d "
                  "shells=%d solids=%d holes=%d open=%d unused=%d alerts=%d [%s]%s",
                  n_surface_pairs, n_sd_surface_pairs, n_section_curves, n_faces_a, n_faces_b,
                  n_in_a, n_in_b, n_on, n_sd_pairs, n_selected, n_shells, n_solids, n_holes,
                  n_open_shells, n_unused, (int)alerts.size(), verdict.str().c_str(),
                  stage_fail.empty() ? "" : (" FAIL:" + stage_fail).c_str());
    return std::string(buf);
}

///////////////////////////////////////////////////////////////////////////////////////////
// helpers
///////////////////////////////////////////////////////////////////////////////////////////

static V2Box v2sol_brep_box(const BRep& b) {
    V2Box bx;
    for (const Point& p : b.m_vertices) bx.add(p);
    for (const auto& s : b.m_surfaces) {
        auto du = s.domain(0), dv = s.domain(1);
        for (int i = 0; i <= 4; ++i)
            for (int j = 0; j <= 4; ++j)
                bx.add(s.point_at(du.first + (du.second - du.first) * (i / 4.0),
                                  dv.first + (dv.second - dv.first) * (j / 4.0)));
    }
    return bx;
}

static V2Box v2sol_surface_box(const NurbsSurface& s) {
    V2Box bx;
    auto du = s.domain(0), dv = s.domain(1);
    for (int i = 0; i <= 6; ++i)
        for (int j = 0; j <= 6; ++j)
            bx.add(s.point_at(du.first + (du.second - du.first) * (i / 6.0),
                              dv.first + (dv.second - dv.first) * (j / 6.0)));
    return bx;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Outward orientation, the OCCT way (no weighted vote)
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<double> v2_outward_signs(const BRep& b, int* shells_out, int* holes_out) {
    std::vector<double> sign((size_t)b.m_faces.size(), 1.0);
    V2Topo topo;
    topo.build(b);
    std::vector<V2OrientedFace> all;
    for (int f = 0; f < topo.nb_faces(); ++f) all.push_back(V2OrientedFace{f, false});

    V2BuilderSolid bs;
    bs.set_topo(&topo);
    bs.set_faces(all);
    bs.set_avoid_internal_shapes(true);
    bs.set_accept_open_shells(true);
    bs.perform();

    int nh = 0;
    for (const V2Shell& sh : bs.shells()) {
        bool undec = false;
        const bool hole = v2_shell_is_hole(topo, sh, &undec);
        if (hole) ++nh;
        for (const auto& of : sh.faces) {
            // outward = the shell's relative orientation, globally flipped iff the shell encloses
            // negative volume. OCCT never flips a shell; this is only how we EXPRESS a face's
            // outward sign, not a change to the shell.
            const bool rev = hole ? !of.reversed : of.reversed;
            sign[(size_t)of.face] = rev ? -1.0 : 1.0;
        }
    }
    if (shells_out) *shells_out = (int)bs.shells().size();
    if (holes_out) *holes_out = nh;
    return sign;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Stage 3+4 — FF section with the same-domain fence
///////////////////////////////////////////////////////////////////////////////////////////

struct V2SolSectionSet {
    std::vector<std::vector<NurbsCurve>> cuts_a;   ///< per A-surface index
    std::vector<std::vector<NurbsCurve>> cuts_b;   ///< per B-surface index
    std::vector<NurbsCurve> c3d;                   ///< the ONE 3D section curve per SSI branch
    int n_curves = 0;
    int n_pairs = 0;
    int n_sd_pairs = 0;
};

/// Are these two surfaces the SAME surface? A named predicate over two NAMED surfaces (degree,
/// CV count, then CV coordinates), never a lookup. A surface cannot be intersected with itself:
/// feeding such a pair to the marcher is what makes A-op-A hang, and OCCT never reaches the
/// marcher for a same-domain FF interference at all.
static bool v2sol_same_surface(const NurbsSurface& a, const NurbsSurface& b, double tol) {
    if (a.degree(0) != b.degree(0) || a.degree(1) != b.degree(1)) return false;
    auto da = a.domain(0), db = b.domain(0);
    auto ea = a.domain(1), eb = b.domain(1);
    if (std::fabs(da.first - db.first) > 1e-9 || std::fabs(da.second - db.second) > 1e-9) return false;
    if (std::fabs(ea.first - eb.first) > 1e-9 || std::fabs(ea.second - eb.second) > 1e-9) return false;
    for (int i = 0; i <= 5; ++i)
        for (int j = 0; j <= 5; ++j) {
            const double u = da.first + (da.second - da.first) * (i / 5.0);
            const double v = ea.first + (ea.second - ea.first) * (j / 5.0);
            if (v2_vlen(a.point_at(u, v) - b.point_at(u, v)) > tol) return false;
        }
    return true;
}

/// ONE SSI per surface PAIR, its pcurves fed to BOTH operands' splits (symmetric imprint by
/// construction). Coincident (same-domain) pairs are FENCED OUT: a surface cannot be
/// intersected with itself, and routing such a pair to the marcher is what shatters A-op-A.
static V2SolSectionSet v2sol_build_sections(const BRep& A, const BRep& B, double tol,
                                            const std::set<std::pair<int, int>>& sd_pairs) {
    V2SolSectionSet out;
    out.cuts_a.assign(A.m_surfaces.size(), {});
    out.cuts_b.assign(B.m_surfaces.size(), {});

    std::vector<V2Box> ba, bb;
    for (const auto& s : A.m_surfaces) ba.push_back(v2sol_surface_box(s));
    for (const auto& s : B.m_surfaces) bb.push_back(v2sol_surface_box(s));

    for (size_t ai = 0; ai < A.m_surfaces.size(); ++ai) {
        const double gap = std::max(1e-9, ba[ai].diagonal() * 1e-3);
        for (size_t bi = 0; bi < B.m_surfaces.size(); ++bi) {
            if (ba[ai].is_out(bb[bi], gap)) continue;
            ++out.n_pairs;
            if (sd_pairs.count({(int)ai, (int)bi}) ||
                v2sol_same_surface(A.m_surfaces[ai], B.m_surfaces[bi], std::max(tol, 1e-9))) {
                ++out.n_sd_pairs;
                continue;
            }
            auto trs = Intersection::surface_surface(A.m_surfaces[ai], B.m_surfaces[bi], tol);
            if (trs.empty()) {
                // the marcher is order-sensitive: retry swapped and exchange the pcurve roles
                for (auto& tr :
                     Intersection::surface_surface(B.m_surfaces[bi], A.m_surfaces[ai], tol)) {
                    if (std::get<2>(tr).is_valid()) out.cuts_a[ai].push_back(std::get<2>(tr));
                    if (std::get<1>(tr).is_valid()) out.cuts_b[bi].push_back(std::get<1>(tr));
                    if (std::get<0>(tr).is_valid()) {
                        out.c3d.push_back(std::get<0>(tr));
                        ++out.n_curves;
                    }
                }
                continue;
            }
            for (auto& tr : trs) {
                if (std::get<1>(tr).is_valid()) out.cuts_a[ai].push_back(std::get<1>(tr));
                if (std::get<2>(tr).is_valid()) out.cuts_b[bi].push_back(std::get<2>(tr));
                if (std::get<0>(tr).is_valid()) {
                    out.c3d.push_back(std::get<0>(tr));
                    ++out.n_curves;
                }
            }
        }
    }
    return out;
}

/// Coincident surface pairs, detected on the ORIGINAL faces before any split (OCCT: an FF
/// interference between same-domain faces yields an SD pair, never section curves).
static std::set<std::pair<int, int>> v2sol_sd_surface_pairs(const BRep& A, const BRep& B,
                                                            double tol) {
    std::set<std::pair<int, int>> out;
    SameDomain sd(tol);
    for (int f = 0; f < (int)A.m_faces.size(); ++f) sd.add_face(A, f, 0, 0);
    for (int f = 0; f < (int)B.m_faces.size(); ++f) sd.add_face(B, f, 1, 1);
    sd.detect();
    for (const auto& p : sd.pairs()) {
        const SDFaceRec& fi = sd.face(p.i);
        const SDFaceRec& fj = sd.face(p.j);
        if (fi.operand == fj.operand) continue;
        const int sa = (fi.operand == 0 ? A.m_faces[fi.face].surface_index
                                        : A.m_faces[fj.face].surface_index);
        const int sb = (fi.operand == 1 ? B.m_faces[fi.face].surface_index
                                        : B.m_faces[fj.face].surface_index);
        out.insert({sa, sb});
    }
    return out;
}

struct V2SolHarvest {
    BRep split;
    std::vector<double> osign;   ///< per split face: +1 when the natural normal points OUT
    int shells = 0;
    int holes = 0;
};

///////////////////////////////////////////////////////////////////////////////////////////
// STAGES 3-5, v2 — arena -> v2sec section -> v2sf face split, into ONE arena BRep.
//
// The whole reason v2 lost on closure was that the two operands' copies of a section edge were
// two entities. Here they cannot be: v2sec fuses every geometric feature to ONE arena edge
// (post_treat_ff), v2sf's identity IS the pave-block pointer, and ONE SfEmitter serves BOTH
// operands -- so a section pave block shared by a face of A and a face of B is emitted as the
// SAME BRep edge index. No sew, no snap, no Hausdorff, no span key: the shell walk sees the two
// operands' face images as adjacent BY CONSTRUCTION.
///////////////////////////////////////////////////////////////////////////////////////////

/// Sub-pcurve of a stored trim curve over the fraction range [f0,f1] of its domain, sampled to a
/// degree-1 curve (what a real imprint produces). Used only when an operand edge carries more
/// than one pave block; an unsplit edge hands over its stored pcurve verbatim.
static NurbsCurve v2sol_sub_pcurve(const NurbsCurve& pc, double f0, double f1, int n) {
    auto d = pc.domain();
    std::vector<Point> pts;
    pts.reserve((size_t)n + 1);
    for (int k = 0; k <= n; ++k) {
        const double f = f0 + (f1 - f0) * ((double)k / (double)n);
        pts.push_back(pc.point_at(d.first + (d.second - d.first) * f));
    }
    return NurbsCurve::create(false, 1, pts);
}

/// Parameter of the trim curve `pc` whose SURFACE image is nearest `p3`.
///
/// THE PCURVE AND THE 3D CURVE ARE NOT SAME-PARAMETER. A pave block is an interval of the EDGE's
/// parameter; mapping that interval onto the pcurve by the FRACTION of the range assumes an
/// affine relation between the two parameterisations. It holds for a box (both linear) and fails
/// for every quadric: measured on a sphere, a seam pave at 3D parameter t landed at v = 0.319790
/// where the section that created it sits at v = 0.341081, so the split node and the section node
/// were 0.02 apart in UV and the face never split. The window must be located by the block's own
/// NAMED endpoint VERTICES, projected into this face's parameterisation -- a disambiguation
/// inside an entity that is already named, never a way to decide identity.
static double v2sol_pc_param_at(const NurbsSurface& srf, const NurbsCurve& pc, const Point& p3) {
    const std::pair<double, double> d = pc.domain();
    double bt = d.first, bd = 1e300;
    for (int k = 0; k <= 256; ++k) {
        const double s = d.first + (d.second - d.first) * (k / 256.0);
        const Point uv = pc.point_at(s);
        const double dd = v2_vlen(srf.point_at(uv[0], uv[1]) - p3);
        if (dd < bd) { bd = dd; bt = s; }
    }
    for (int it = 0; it < 60; ++it) {
        const double h = (d.second - d.first) * std::pow(0.5, it + 1) * 0.5;
        const double sm = std::max(d.first, bt - h), sp = std::min(d.second, bt + h);
        const Point um = pc.point_at(sm), up = pc.point_at(sp);
        const double dm = v2_vlen(srf.point_at(um[0], um[1]) - p3);
        const double dp = v2_vlen(srf.point_at(up[0], up[1]) - p3);
        if (dm < bd) { bd = dm; bt = sm; }
        if (dp < bd) { bd = dp; bt = sp; }
    }
    return bt;
}

/// Pcurve of a section block on one of its two faces, read from the parameter-synchronised
/// footprint trail (v2sec's V2PntOn2S). `first` selects face1's footprint (u1,v1) or face2's
/// (u2,v2). The trail is UNWRAPPED, so a block that crosses a seam stays continuous in UV.
static NurbsCurve v2sol_trail_pcurve(const std::vector<v2sec::V2PntOn2S>& tr, double t0, double t1,
                                     bool first, int n) {
    // The trail's OWN points are exact footprints of the section (v2sec put them on both
    // surfaces); anything between them is a linear guess. So take the trail points inside
    // (t0, t1) verbatim, subsampled by stride to about `n` of them, and interpolate only the two
    // ends -- which are the block's named nodes anyway.
    auto at = [&](double t) {
        size_t i = 0;
        while (i + 2 < tr.size() && tr[i + 1].t < t) ++i;
        const v2sec::V2PntOn2S& a = tr[i];
        const v2sec::V2PntOn2S& b = tr[std::min(i + 1, tr.size() - 1)];
        const double dt = b.t - a.t;
        const double w = (std::fabs(dt) > 1e-300) ? (t - a.t) / dt : 0.0;
        const double ua = first ? a.u1 : a.u2, va = first ? a.v1 : a.v2;
        const double ub = first ? b.u1 : b.u2, vb = first ? b.v1 : b.v2;
        return Point(ua + (ub - ua) * w, va + (vb - va) * w, 0.0);
    };
    std::vector<Point> pts;
    {
        std::vector<size_t> idx;
        for (size_t i = 0; i < tr.size(); ++i)
            if (tr[i].t > t0 + 1e-12 && tr[i].t < t1 - 1e-12) idx.push_back(i);
        const size_t stride =
            idx.empty() ? 1 : std::max<size_t>(1, (idx.size() + (size_t)n - 1) / (size_t)n);
        pts.push_back(at(t0));
        for (size_t q = 0; q < idx.size(); q += stride) {
            const v2sec::V2PntOn2S& a = tr[idx[q]];
            pts.push_back(Point(first ? a.u1 : a.u2, first ? a.v1 : a.v2, 0.0));
        }
        pts.push_back(at(t1));
    }
    if (pts.size() < 3) {
        pts.clear();
        for (int k = 0; k <= n; ++k)
            pts.push_back(at(t0 + (t1 - t0) * ((double)k / (double)n)));
    }
    // DEGREE MATTERS, and it is measurable. A degree-1 chord polygon leaves the two faces that
    // share this section disagreeing about their common boundary by the chord sagitta, and the
    // closure residual is exactly that: MEASURED on sphere x sphere, resid = 1.19e-04 at n=32,
    // 6.59e-06 at 128, 4.12e-07 at 512 -- a clean O(h^2). A cubic interpolant through the SAME
    // trail points is O(h^4), so the two faces agree far better for the same sample count.
    static const bool s_lin = []() {
        const char* p = std::getenv("SESSION_V2_SEC_LINEAR");
        return p && p[0];
    }();
    if (!s_lin && pts.size() >= 4) {
        const NurbsCurve c = NurbsCurve::create_interpolated(pts);
        if (c.is_valid()) return c;
    }
    return NurbsCurve::create(false, 1, pts);
}

///////////////////////////////////////////////////////////////////////////////////////////
// LOOP WINDING, DERIVED FROM THE DATA.
//
// The stored trim conventions are not uniform across this kernel's construction paths
// (brep.h:348-352 says so), and MEASURED on box x box every single boundary trim satisfies
//     natural_dir == !BRepTrim::reversed
// i.e. `reversed` records the PCURVE-vs-3D-CURVE relation, not a traversal winding: the 2D
// curve of a trim is already stored in the loop's traversal order. Feeding
// BRep::loop_material_left the traversal `!reversed` therefore measures the wrong walk on any
// face whose trims all carry reversed=1 (it returned material_left=0 for the identical box
// face that its neighbour reported 1) and the two flips cancel on half the faces and add on
// the other half -- which is exactly the "same shape, one works one dead-ends" signature.
//
// So nothing below reads `reversed` or `loop_material_left`. Each loop's pcurves are CHAINED
// head-to-tail in UV (nearest-endpoint, a pure data operation), and the resulting closed
// polygon is oriented by its OWN SIGNED AREA: the face's largest loop is forced CCW and every
// other loop CW. That is precisely the winding v2sf's area stage classifies by (outer =
// positive UV area, hole = negative), so the caller and the splitter cannot disagree.
///////////////////////////////////////////////////////////////////////////////////////////

/// Signed UV area of one chained loop, sampled at `n` points per trim.
static double v2sol_loop_area(const BRep& b, const std::vector<int>& trims,
                              const std::vector<char>& ff, int n) {
    double A = 0.0, pu = 0.0, pv = 0.0, u0 = 0.0, v0 = 0.0;
    bool first = true;
    for (size_t k = 0; k < trims.size(); ++k) {
        const NurbsCurve& pc = b.m_curves_2d[b.m_trims[trims[k]].curve_2d_index];
        const auto d = pc.domain();
        for (int i = 0; i <= n; ++i) {
            double f = (double)i / (double)n;
            if (!ff[k]) f = 1.0 - f;
            const Point p = pc.point_at(d.first + (d.second - d.first) * f);
            if (first) { u0 = pu = p[0]; v0 = pv = p[1]; first = false; continue; }
            A += pu * p[1] - p[0] * pv;
            pu = p[0];
            pv = p[1];
        }
    }
    if (first) return 0.0;
    A += pu * v0 - u0 * pv;
    return 0.5 * A;
}

/// Traversal direction of every trim of face `fi`, keyed by trim index. `true` = traverse the
/// stored pcurve from its first parameter to its last. `prev`/`next` give the CYCLIC chain
/// neighbour of every trim inside its own loop, which is how a POLE ROW (a trim with no edge
/// record, so no named vertex of its own) learns which arena vertex it sits on: the vertex its
/// neighbours already name.
struct V2FaceDirs {
    std::map<int, char> ff;
    std::map<int, int> prev, next;
};
static V2FaceDirs v2sol_face_trim_dirs(const BRep& b, int fi) {
    V2FaceDirs out;
    if (fi < 0 || fi >= (int)b.m_faces.size()) return out;
    struct L {
        std::vector<int> trims;
        std::vector<char> ff;
        double area = 0.0;
    };
    std::vector<L> ls;
    for (int li : b.m_faces[fi].loop_indices) {
        if (li < 0 || li >= (int)b.m_loops.size()) continue;
        std::vector<int> ts;
        for (int ti : b.m_loops[li].trim_indices) {
            if (ti < 0 || ti >= (int)b.m_trims.size()) continue;
            const int c2 = b.m_trims[ti].curve_2d_index;
            if (c2 < 0 || c2 >= (int)b.m_curves_2d.size()) continue;
            if (!b.m_curves_2d[c2].is_valid()) continue;
            ts.push_back(ti);
        }
        if (ts.empty()) continue;
        // endpoints of each candidate trim's stored pcurve
        std::vector<Point> pa(ts.size()), pb(ts.size());
        for (size_t k = 0; k < ts.size(); ++k) {
            const NurbsCurve& pc = b.m_curves_2d[b.m_trims[ts[k]].curve_2d_index];
            const auto d = pc.domain();
            pa[k] = pc.point_at(d.first);
            pb[k] = pc.point_at(d.second);
        }
        // greedy head-to-tail chain; the seed direction is arbitrary because a closed loop is
        // fixed up to ONE global flip, and the area sign below decides that flip.
        L L0;
        std::vector<char> used(ts.size(), 0);
        L0.trims.push_back(ts[0]);
        L0.ff.push_back(1);
        used[0] = 1;
        Point end = pb[0];
        for (size_t step = 1; step < ts.size(); ++step) {
            int best = -1;
            char best_ff = 1;
            double bd = 1e300;
            for (size_t k = 0; k < ts.size(); ++k) {
                if (used[k]) continue;
                const double d1 = std::hypot(pa[k][0] - end[0], pa[k][1] - end[1]);
                const double d2 = std::hypot(pb[k][0] - end[0], pb[k][1] - end[1]);
                if (d1 < bd) { bd = d1; best = (int)k; best_ff = 1; }
                if (d2 < bd) { bd = d2; best = (int)k; best_ff = 0; }
            }
            if (best < 0) break;
            used[best] = 1;
            L0.trims.push_back(ts[best]);
            L0.ff.push_back(best_ff);
            end = best_ff ? pb[best] : pa[best];
        }
        L0.area = v2sol_loop_area(b, L0.trims, L0.ff, 16);
        ls.push_back(L0);
    }
    if (ls.empty()) return out;
    // The face's OUTER loop is its largest; force it CCW and every hole CW.
    size_t big = 0;
    for (size_t k = 1; k < ls.size(); ++k)
        if (std::fabs(ls[k].area) > std::fabs(ls[big].area)) big = k;
    for (size_t k = 0; k < ls.size(); ++k) {
        const bool want_pos = (k == big);
        const bool flip = (want_pos ? (ls[k].area < 0.0) : (ls[k].area > 0.0));
        const size_t n = ls[k].trims.size();
        for (size_t q = 0; q < n; ++q) {
            const int ti = ls[k].trims[q];
            out.ff[ti] = flip ? (char)!ls[k].ff[q] : ls[k].ff[q];
            const int fwd_next = ls[k].trims[(q + 1) % n];
            const int fwd_prev = ls[k].trims[(q + n - 1) % n];
            out.next[ti] = flip ? fwd_prev : fwd_next;
            out.prev[ti] = flip ? fwd_next : fwd_prev;
        }
    }
    return out;
}

/// Is the chart CLOSED in direction `dir` (u=0), i.e. do its two boundary rows coincide in 3D?
/// Then its domain span IS the period. Measured on the surface, never assumed from a flag.
static bool v2sol_closed_dir(const NurbsSurface& s, int dir, double tol, double& period) {
    const std::pair<double, double> d0 = s.domain(0), d1 = s.domain(1);
    period = (dir == 0) ? (d0.second - d0.first) : (d1.second - d1.first);
    if (!(period > 0.0)) return false;
    const std::pair<double, double> o = (dir == 0) ? d1 : d0;
    for (int k = 0; k <= 8; ++k) {
        const double w = o.first + (o.second - o.first) * (k / 8.0);
        const Point a = (dir == 0) ? s.point_at(d0.first, w) : s.point_at(w, d1.first);
        const Point b = (dir == 0) ? s.point_at(d0.second, w) : s.point_at(w, d1.second);
        if (v2_vlen(a - b) > std::max(tol, 1e-9) * 100.0) return false;
    }
    return true;
}

/// Translate a section pcurve by WHOLE PERIODS until it lies inside the chart. v2sec's footprint
/// trail is UNWRAPPED so that a curve crossing a seam stays continuous, which means a block on
/// the far side of the seam is expressed at u < u0 (or u > u1). Our charts are CLAMPED NURBS:
/// evaluating one there is NOT the periodic continuation, so the face would be built on garbage
/// geometry -- measured on sphere x sphere, the cap came out at HALF its area (pi/2 against pi)
/// while still closing topologically. The block itself never straddles the seam (v2sec puts a
/// pave at every crossing), so a whole-period shift of the whole curve is exact.
static void v2sol_wrap_pcurve(const NurbsSurface& s, double tol, NurbsCurve& pc) {
    const std::pair<double, double> dm[2] = {s.domain(0), s.domain(1)};
    for (int dir = 0; dir < 2; ++dir) {
        double period = 0.0;
        if (!v2sol_closed_dir(s, dir, tol, period)) continue;
        const std::pair<double, double> d = pc.domain();
        const Point mid = pc.point_at(0.5 * (d.first + d.second));
        const double m = mid[dir];
        const double k = std::floor((m - dm[dir].first) / period);
        if (k == 0.0) continue;
        const double sh = -k * period;
        for (int c = 0; c < pc.cv_count(); ++c) {
            const Point p = pc.get_cv(c);
            pc.set_cv(c, dir == 0 ? Point(p[0] + sh, p[1], p[2]) : Point(p[0], p[1] + sh, p[2]));
        }
    }
}

struct V2Front {
    BdsArena ds;
    BRep arena_brep;                 ///< BOTH operands' split faces, ONE edge per pave block
    std::vector<int> face_operand;   ///< per arena_brep face: 0 = A, 1 = B
    std::vector<int> face_src;       ///< per arena_brep face: its source arena face index
    int n_curves = 0, n_section_edges = 0, n_section_nodes = 0, n_faces_split = 0;
    int n_alerts = 0, n_unused = 0, n_extended = 0, n_inextensible = 0;
    int n_faces_lost = 0;   ///< source faces the splitter could not turn into at least one face
    bool split_faithful = false;   ///< each operand's face images ARE that operand (closed, exact volume)
    bool ok = false;
};

/// Build the whole v2 front end for one operand pair. Returns false when a stage refuses; the
/// caller then falls back to the v1 route and SAYS SO in the report (guarantee I-13).
static bool v2sol_run_front(const BRep& A, const BRep& B, double tol, V2Front& F) {
    F.ds.init({&A, &B}, std::max(tol, 1e-7));

    // The arena builds pave-block pools LAZILY (kb/v2_splitface_notes.md §2 rule 2): materialise
    // every operand edge once, or faces silently receive no boundary at all.
    for (int i = 0; i < F.ds.nb_shapes(); ++i)
        if (F.ds.is_edge(i)) F.ds.change_pave_blocks(i);

    const std::vector<const BRep*> ops = {&A, &B};
    v2sec::V2SectionParams prm;
    // The trail's chord sets the accuracy of every crossing node v2sec names (the crossing is
    // interpolated along the trail), and the node's position IS the output vertex. The default
    // 192 leaves a sphere's seam crossing 4.3e-4 off the meridian it crosses; the error falls
    // quadratically with the sample count, so this is the one dial that buys geometric accuracy
    // on curved pairs. Tunable to make that scaling measurable rather than asserted.
    {
        const char* p = std::getenv("SESSION_V2_TRAIL_N");
        if (p && p[0]) prm.trail_samples = std::max(16, std::atoi(p));
        else prm.trail_samples = 768;
    }
    v2sec::V2Section S(F.ds, ops, prm);
    if (S.perform_all() <= 0) return false;
    F.n_curves = (int)S.curves().size();
    F.n_section_edges = (int)S.section_edges().size();
    F.n_section_nodes = (int)S.section_nodes().size();

    // HOW FAR OFF ITS OWN SURFACE IS EACH OPERAND EDGE? Measured, per edge, once.
    //
    // A section node provably lies on BOTH surfaces (that is what a section is). An operand edge
    // is supposed to lie on its face's surface too -- but in this kernel's own primitives it does
    // not: create_sphere's seam edge is 4.31e-04 off its sphere and create_torus's is 1.78e-03
    // off its torus (both measured below, both reproduced in the SFDBG=3 dump). So a node that
    // genuinely lies on an edge can be that far from the edge's CURVE, and a pave-attachment gate
    // set from the model tolerance alone throws it away -- which is exactly what left every
    // sphere and torus seam unsplit and the face unsplittable.
    std::vector<double> egap((size_t)F.ds.nb_shapes(), 0.0);
    for (int q = 0; q < F.ds.nb_shapes(); ++q) {
        if (!F.ds.is_edge(q)) continue;
        const int r = F.ds.rank(q);
        if (r != 0 && r != 1) continue;
        const BRep& b = *ops[r];
        const int le = F.ds.shape(q).source;
        const NurbsSurface* srf = nullptr;
        for (size_t ti = 0; ti < b.m_trims.size() && !srf; ++ti) {
            if (b.m_trims[ti].edge_index != le) continue;
            const int li = b.m_trims[ti].loop_index;
            if (li < 0 || li >= (int)b.m_loops.size()) continue;
            const int fi = b.m_loops[li].face_index;
            if (fi < 0 || fi >= (int)b.m_faces.size()) continue;
            const int si = b.m_faces[fi].surface_index;
            if (si >= 0 && si < (int)b.m_surfaces.size()) srf = &b.m_surfaces[si];
        }
        const NurbsCurve* ec = F.ds.edge_curve(q);
        if (!srf || !ec) continue;
        const std::pair<double, double> er = F.ds.edge_range(q);
        double g = 0.0;
        for (int k = 0; k <= 16; ++k) {
            const Point p = ec->point_at(er.first + (er.second - er.first) * (k / 16.0));
            const std::pair<double, double> uv = srf->closest_parameters(p);
            g = std::max(g, v2_vlen(srf->point_at(uv.first, uv.second) - p));
        }
        egap[(size_t)q] = g;
    }

    // A section pave that was born ON an operand edge (a trim crossing) must also become a pave
    // OF that edge, or the boundary carries no node where the section lands and the wire walk
    // prunes the whole face. This is the EF interference in its only load-bearing form; the
    // parameter is computed on a NAMED edge and the VERTEX comes from the pave, so identity is
    // still the arena index.
    for (const auto& c : S.curves()) {
        const char* dbgc = std::getenv("SESSION_V2_SFDBG");
        if (dbgc && dbgc[0] == '3') {
            std::printf("[SFDBG] curve f1=%d f2=%d paves=%zu blocks=%zu closed=%d\n", c.face1,
                        c.face2, c.paves.size(), c.blocks.size(), (int)c.closed);
            for (const auto& pv : c.paves)
                std::printf("[SFDBG]    pave t=%.6f v=%d org=%s edge=%d face=%d fused=%d\n", pv.t,
                            pv.vertex, v2sec::v2_origin_name(pv.origin), pv.edge, pv.face,
                            (int)pv.fused);
        }
        // Project `p` onto arena edge `q`: coarse scan, then a bracketed bisection. The COARSE
        // distance is never a verdict -- gating on it rejected every seam crossing of a sphere
        // (coarse 1.15e-2 against a limit of 4.1e-3 where the refined distance is 1e-12).
        const double lim = std::max(tol, 1e-7) * 100.0;
        // The section stage's OWN resolution on this curve. v2sec locates a trim/seam crossing on
        // a SAMPLED footprint trail, so the node it names sits up to the trail's chord sagitta
        // off the edge it provably crosses -- MEASURED here as the largest chord of this curve's
        // own trail, not guessed.
        double chord = 0.0;
        for (size_t q = 1; q < c.trail.size(); ++q)
            chord = std::max(chord, v2_vlen(c.trail[q].p - c.trail[q - 1].p));
        const double sec_lim = std::max(c.tol, std::max(c.dev1, c.dev2)) * 8.0 + chord + lim;
        auto project = [&](int q, const Point& p, double& t_out) -> double {
            const NurbsCurve* qc = F.ds.edge_curve(q);
            if (!qc) return 1e300;
            const auto qr = F.ds.edge_range(q);
            double bt = qr.first, bd = 1e300;
            for (int k = 0; k <= 256; ++k) {
                const double t = qr.first + (qr.second - qr.first) * (k / 256.0);
                const double d = v2_vlen(qc->point_at(t) - p);
                if (d < bd) { bd = d; bt = t; }
            }
            for (int it = 0; it < 60; ++it) {
                const double h = (qr.second - qr.first) * std::pow(0.5, it + 1) * 0.5;
                const double tm = std::max(qr.first, bt - h), tp = std::min(qr.second, bt + h);
                const double dm = v2_vlen(qc->point_at(tm) - p);
                const double dp = v2_vlen(qc->point_at(tp) - p);
                if (dm < bd) { bd = dm; bt = tm; }
                if (dp < bd) { bd = dp; bt = tp; }
            }
            t_out = bt;
            return bd;
        };
        for (const auto& pv : c.paves) {
            if (pv.vertex < 0) continue;
            const Point p = F.ds.vertex_point(pv.vertex);
            // A node lands on EVERY edge it lies on, not on the nearest one. Two coaxial tori put
            // their section circle through BOTH operands' seam meridians at the same point;
            // attaching that node to one seam only left the other unsplit and its face unsplit
            // with it. Each candidate is gated by its OWN measured limit, so this cannot bind a
            // node to an edge it does not touch.
            //
            // The named route (V2Pave::edge is the index in the OPERAND's own BRep and
            // V2Pave::face names the arena face) gets the SECTION's accuracy as its gate, not the
            // model tolerance: gating a sphere's seam crossing at tol*100 (measured 4.31e-4
            // against a 4.12e-4 limit) threw away every one of them.
            const double named_lim = std::max(lim, sec_lim);
            int named_edge = -1;
            if (pv.edge >= 0 && pv.face >= 0 && F.ds.is_face(pv.face)) {
                const int fop = F.ds.rank(pv.face);
                if (fop >= 0) named_edge = F.ds.index_of_edge(fop, pv.edge);
            }
            const char* dbgp = std::getenv("SESSION_V2_SFDBG");
            int hits = 0;
            double show_d = 1e300;
            int show_e = -1;
            for (int q = 0; q < F.ds.nb_shapes(); ++q) {
                if (!F.ds.is_edge(q) || F.ds.rank(q) < 0) continue;
                double t = 0.0;
                const double d = project(q, p, t);
                if (d < show_d) { show_d = d; show_e = q; }
                const double gate = (q == named_edge ? named_lim : lim) + egap[(size_t)q];
                if (d > gate) continue;
                const BdsArena::PaveStatus ps = F.ds.add_pave(q, t, pv.vertex);
                ++hits;
                if (dbgp && dbgp[0] == '3')
                    std::printf("[SFDBG] pave v=%d org=%s edge=%d -> ae=%d t=%.6f d=%.2e "
                                "gate=%.2e st=%d\n",
                                pv.vertex, v2sec::v2_origin_name(pv.origin), pv.edge, q, t, d,
                                gate, (int)ps);
            }
            if (hits == 0 && dbgp && dbgp[0] == '3') {
                const NurbsCurve* qc = (show_e >= 0) ? F.ds.edge_curve(show_e) : nullptr;
                const Point q3 = qc ? qc->point_at(0.0) : Point(0, 0, 0);
                (void)q3;
                std::printf("[SFDBG] pave v=%d org=%s edge=%d NOEDGE nearest=%d d=%.3e "
                            "lim=%.3e sec=%.3e egap=%.3e\n",
                            pv.vertex, v2sec::v2_origin_name(pv.origin), pv.edge, show_e, show_d,
                            lim, sec_lim, show_e >= 0 ? egap[(size_t)show_e] : 0.0);
            }
        }
    }
    F.ds.update_pave_blocks();

    // ---- per-face input assembly (kb/v2_splitface_notes.md §9 gap 1) ----------------------
    v2sf::SfEmitter em(F.arena_brep);
    em.set_pole_edges(false);   // this kernel's own primitives' convention; stated, not accidental

    for (int af = 0; af < F.ds.nb_shapes(); ++af) {
        if (!F.ds.is_face(af)) continue;
        const int op = F.ds.rank(af);
        if (op != 0 && op != 1) continue;
        const BRep& src = *ops[op];
        const int lf = F.ds.shape(af).source;
        if (lf < 0 || lf >= (int)src.m_faces.size()) continue;
        const NurbsSurface* srf = F.ds.face_surface(af);
        if (!srf) continue;

        std::vector<v2sf::SfInputEdge> in;
        std::vector<std::unique_ptr<NurbsCurve>> pool;   // must outlive sf_split_face + emit
        struct DbgRec { int ti, ae, blk; char nat, rev, matl; };
        std::vector<DbgRec> dbg;

        // (a) the face's own boundary. A seam edge already carries TWO trims with TWO different
        // 2D curves in this BRep, so it produces two entries with opposite senses for free.
        // Traversal direction of every trim of this face, chained + area-oriented from the DATA.
        // Nothing here reads BRepTrim::reversed or loop_material_left; see the note above
        // v2sol_face_trim_dirs for the measurement that rules both of them out.
        const V2FaceDirs tdir = v2sol_face_trim_dirs(src, lf);

        // PRE-PASS. A POLE ROW carries no edge record, so it has no named vertex of its own; it
        // must be given the arena vertex its chain neighbours already name, or the meridians are
        // valence-1, the fixpoint prune eats the face, and a sphere never splits. So resolve
        // every REAL trim's arena edge and traversal endpoints first.
        struct TrimRec {
            int ae = -1;
            bool nat = true;
            int v_start = -1, v_end = -1;   ///< traversal endpoints, arena vertex indices
        };
        std::map<int, TrimRec> trec;
        for (int li : src.m_faces[lf].loop_indices) {
            if (li < 0 || li >= (int)src.m_loops.size()) continue;
            for (int ti : src.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)src.m_trims.size()) continue;
                const BRepTrim& T = src.m_trims[ti];
                if (T.edge_index < 0) continue;
                if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)src.m_curves_2d.size())
                    continue;
                const std::map<int, char>::const_iterator d0 = tdir.ff.find(ti);
                if (d0 == tdir.ff.end()) continue;
                const int ae = F.ds.index_of_edge(op, T.edge_index);
                if (ae < 0 || !F.ds.has_pave_blocks(ae)) continue;
                const auto& pbs = F.ds.pave_blocks(ae);
                if (pbs.empty()) continue;
                const NurbsCurve& pc0 = src.m_curves_2d[T.curve_2d_index];
                // Which way does the STORED pcurve run relative to the edge's natural direction?
                // v1 physically reverses a pcurve and still writes reversed=false
                // (kb/v2_splitface_notes.md D14), so the flag alone cannot answer it. Measure the
                // two NAMED endpoint vertices; v2sf requires the pcurve in the block's NATURAL
                // direction and the traversal in `sense`.
                TrimRec r;
                r.ae = ae;
                {
                    // Does the stored pcurve run WITH the edge's parameter or against it?
                    // Measured on two INTERIOR points of the edge, never on its endpoints: a
                    // CLOSED edge (a torus's seam meridian, a cylinder's cap circle) has the same
                    // vertex at both ends, so the endpoint comparison is a coin toss -- and when
                    // it lands wrong the block windows come out non-monotone and the face dies.
                    const NurbsCurve* ec0 = F.ds.edge_curve(ae);
                    const std::pair<double, double> er0 = F.ds.edge_range(ae);
                    bool decided = false;
                    if (ec0) {
                        const double ta = er0.first + (er0.second - er0.first) * 0.25;
                        const double tb = er0.first + (er0.second - er0.first) * 0.75;
                        const double sa = v2sol_pc_param_at(*srf, pc0, ec0->point_at(ta));
                        const double sb0 = v2sol_pc_param_at(*srf, pc0, ec0->point_at(tb));
                        if (std::fabs(sb0 - sa) > 1e-12) { r.nat = (sb0 > sa); decided = true; }
                    }
                    if (!decided) {
                        const Point uv0 = pc0.point_at(pc0.domain().first);
                        const Point p0 = srf->point_at(uv0[0], uv0[1]);
                        const BdsPB& fb = pbs.front();
                        const BdsPB& lb = pbs.back();
                        if (fb->pave1.vertex >= 0 && lb->pave2.vertex >= 0) {
                            const double d1 = v2_vlen(F.ds.vertex_point(fb->pave1.vertex) - p0);
                            const double d2 = v2_vlen(F.ds.vertex_point(lb->pave2.vertex) - p0);
                            r.nat = (d1 <= d2);
                        }
                    }
                }
                const int v1 = F.ds.resolve_sd(pbs.front()->pave1.vertex);
                const int v2 = F.ds.resolve_sd(pbs.back()->pave2.vertex);
                const int a = r.nat ? v1 : v2, b2 = r.nat ? v2 : v1;   // stored-curve ends
                r.v_start = (d0->second != 0) ? a : b2;
                r.v_end = (d0->second != 0) ? b2 : a;
                trec[ti] = r;
            }
        }

        for (int li : src.m_faces[lf].loop_indices) {
            if (li < 0 || li >= (int)src.m_loops.size()) continue;
            for (int ti : src.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)src.m_trims.size()) continue;
                const BRepTrim& T = src.m_trims[ti];
                if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)src.m_curves_2d.size())
                    continue;
                const NurbsCurve& pc = src.m_curves_2d[T.curve_2d_index];
                const std::map<int, char>::const_iterator dit = tdir.ff.find(ti);
                if (dit == tdir.ff.end()) continue;
                const bool from_first = (dit->second != 0);

                if (T.edge_index < 0) {
                    // A POLE ROW. It must sit on the arena vertex its chain neighbours name --
                    // minting a fresh one leaves the pole disconnected from the seam that ends
                    // there and the whole face dead-ends.
                    int pv = -1;
                    const std::map<int, int>::const_iterator nx = tdir.next.find(ti);
                    const std::map<int, int>::const_iterator pv0 = tdir.prev.find(ti);
                    if (nx != tdir.next.end()) {
                        const std::map<int, TrimRec>::const_iterator q = trec.find(nx->second);
                        if (q != trec.end()) pv = q->second.v_start;
                    }
                    if (pv < 0 && pv0 != tdir.prev.end()) {
                        const std::map<int, TrimRec>::const_iterator q = trec.find(pv0->second);
                        if (q != trec.end()) pv = q->second.v_end;
                    }
                    const Point p0 = pc.point_at(pc.domain().first);
                    const Point p3 = srf->point_at(p0[0], p0[1]);
                    if (pv < 0) pv = F.ds.append_vertex(p3, std::max(tol, 1e-7));
                    const Point pk = F.ds.vertex_point(pv);
                    const NurbsCurve dc = NurbsCurve::create(false, 1, {pk, pk});
                    const int de = F.ds.append_edge(dc, pv, pv, std::max(tol, 1e-7));
                    F.ds.change_pave_blocks(de);
                    if (F.ds.pave_blocks(de).empty()) continue;
                    pool.push_back(std::unique_ptr<NurbsCurve>(new NurbsCurve(pc)));
                    v2sf::SfInputEdge e;
                    e.pb = F.ds.pave_blocks(de).front();
                    e.sense = from_first ? v2sf::SfSense::Forward : v2sf::SfSense::Reversed;
                    e.pcurve = pool.back().get();
                    e.degenerate = true;
                    e.tag = ti;
                    in.push_back(e);
                    dbg.push_back({ti, de, 0, 1, (char)T.reversed, (char)from_first});
                    continue;
                }
                const std::map<int, TrimRec>::const_iterator tr = trec.find(ti);
                if (tr == trec.end()) continue;
                const int ae = tr->second.ae;
                const bool natural_dir = tr->second.nat;
                const auto& pbs = F.ds.pave_blocks(ae);
                const auto er = F.ds.edge_range(ae);
                const double span = er.second - er.first;
                const std::pair<double, double> pcd = pc.domain();
                const double pcspan = pcd.second - pcd.first;
                (void)span;
                (void)er;
                // THE BLOCK WINDOWS, MONOTONE BY CONSTRUCTION. The pcurve is a monotone
                // reparameterisation of the edge, so the pave blocks tile its domain in order:
                // only the INTERNAL split parameters are located by projecting the shared vertex,
                // and the two outer ends are the pcurve's own domain ends. Projecting the OUTER
                // vertices too is wrong on a CLOSED edge, where one vertex has two parameter
                // images -- measured on a torus: the last block of the split u-seam came back as
                // [1.462 -> 0.000] instead of [1.462 -> 4.000] and the face never closed.
                std::vector<double> sb(pbs.size() + 1);
                sb[0] = natural_dir ? pcd.first : pcd.second;
                sb[pbs.size()] = natural_dir ? pcd.second : pcd.first;
                for (size_t k = 1; k < pbs.size(); ++k) {
                    const int iv = pbs[k]->pave1.vertex;
                    sb[k] = (iv >= 0) ? v2sol_pc_param_at(*srf, pc, F.ds.vertex_point(iv))
                                      : (sb[0] + (sb[pbs.size()] - sb[0]) * (double)k /
                                                     (double)pbs.size());
                }
                for (size_t k = 0; k < pbs.size(); ++k) {
                    // "whole" means the STORED curve can be handed over verbatim -- which needs it
                    // to already run in the block's natural direction, not merely to be the only
                    // block.
                    const bool whole = (pbs.size() == 1 && natural_dir);
                    const double s1 = sb[k], s2 = sb[k + 1];
                    const bool inc = (s2 >= s1);
                    if (whole) {
                        pool.push_back(std::unique_ptr<NurbsCurve>(new NurbsCurve(pc)));
                    } else {
                        // EXACT sub-pcurve: trim the stored curve, never re-sample it. A degree-1
                        // resampling of a CURVED boundary trim (a cylinder cap circle, a sphere
                        // meridian) puts a chord error into the face boundary that no downstream
                        // stage can remove.
                        NurbsCurve sub = pc;
                        const double a = std::min(s1, s2), b3 = std::max(s1, s2);
                        if (!sub.trim(a, b3))
                            sub = v2sol_sub_pcurve(pc, (a - pcd.first) / pcspan,
                                                   (b3 - pcd.first) / pcspan, 24);
                        if (s1 > s2) sub.reverse();
                        pool.push_back(std::unique_ptr<NurbsCurve>(new NurbsCurve(sub)));
                    }
                    v2sf::SfInputEdge e;
                    e.pb = pbs[k];
                    // The handed-over pcurve is ALWAYS the block's natural direction (pave1 ->
                    // pave2), so the traversal sense is exactly "does the loop walk that way".
                    const bool fwd = (from_first == (whole ? natural_dir : inc));
                    e.sense = fwd ? v2sf::SfSense::Forward : v2sf::SfSense::Reversed;
                    e.pcurve = pool.back().get();
                    e.degenerate = (T.type == BRepTrimType::Singular);
                    e.tag = ti;
                    in.push_back(e);
                    dbg.push_back({ti, ae, (int)k, (char)natural_dir, (char)T.reversed,
                                   (char)from_first});
                }
            }
        }

        // (b) the section blocks lying on THIS face: two entries each, Forward + Reversed, the
        // SAME pcurve object (kb/v2_splitface_notes.md §2).
        for (const auto& c : S.curves()) {
            const bool is1 = (c.face1 == af), is2 = (c.face2 == af);
            if (!is1 && !is2) continue;
            for (const auto& blk : c.blocks) {
                if (!blk.kept || !blk.pb) continue;
                // Sample density of the imprinted pcurve. It is a degree-1 chord polygon, so the
                // face's boundary carries a sagitta error of O((span/n)^2) on a CURVED section --
                // that, not the topology, is what caps the closure residual on quadric pairs.
                static const int s_secn = []() {
                    const char* p = std::getenv("SESSION_V2_SEC_N");
                    return (p && p[0]) ? std::max(4, std::atoi(p)) : 192;
                }();
                NurbsCurve spc = v2sol_trail_pcurve(c.trail, blk.t0, blk.t1, is1, s_secn);
                v2sol_wrap_pcurve(*srf, tol, spc);
                pool.push_back(std::unique_ptr<NurbsCurve>(new NurbsCurve(spc)));
                const NurbsCurve* pc = pool.back().get();
                v2sf::SfInputEdge a;
                a.pb = blk.pb;
                a.sense = v2sf::SfSense::Forward;
                a.pcurve = pc;
                a.tag = -2;
                in.push_back(a);
                dbg.push_back({-2, -1, 0, 1, 0, 0});
                v2sf::SfInputEdge r = a;
                r.sense = v2sf::SfSense::Reversed;
                in.push_back(r);
                dbg.push_back({-2, -1, 0, 1, 0, 0});
            }
        }
        if (in.empty()) continue;

        v2sf::SfOptions so;
        so.tol3d = std::max(tol, 1e-7);
        so.avoid_internal = true;
        const v2sf::SfResult res = v2sf::sf_split_face(F.ds, *srf, in, so);
        if (std::getenv("SESSION_V2_SFDBG")) {
            int nb = 0, nsec = 0, ndeg = 0;
            for (const auto& e : in) {
                if (e.tag == -2) ++nsec;
                else ++nb;
                if (e.degenerate) ++ndeg;
            }
            std::printf("[SFDBG] af=%d op=%d lf=%d in=%zu(bnd=%d sec=%d deg=%d) faces=%zu "
                        "unused=%zu alerts=%zu\n",
                        af, op, lf, in.size(), nb, nsec, ndeg, res.faces.size(),
                        res.unused.size(), res.report.events.size());
            for (const auto& ev : res.report.events)
                std::printf("[SFDBG]    alert %s @%d\n", v2sf::sf_alert_name(ev.first), ev.second);
            const char* lv = std::getenv("SESSION_V2_SFDBG");
            if (lv && lv[0] == '2') {
                for (size_t q = 0; q < in.size(); ++q) {
                    const auto& e = in[q];
                    const auto d = e.pcurve->domain();
                    const Point a0 = e.pcurve->point_at(d.first);
                    const Point a1 = e.pcurve->point_at(d.second);
                    const v2sf::SfPaveBlockId id = v2sf::sf_pave_block_id(F.ds, e.pb.get());
                    std::printf("[SFDBG]  #%zu ti=%d ae=%d blk=%d pbid=(%d,%d) v=(%d->%d) "
                                "sense=%c nat=%d rev=%d ff=%d uv0=(%.6f,%.6f) uv1=(%.6f,%.6f)\n",
                                q, dbg[q].ti, dbg[q].ae, dbg[q].blk, id.edge, id.block,
                                F.ds.resolve_sd(e.pb->pave1.vertex),
                                F.ds.resolve_sd(e.pb->pave2.vertex),
                                e.sense == v2sf::SfSense::Forward ? 'F' : 'R', (int)dbg[q].nat,
                                (int)dbg[q].rev, (int)dbg[q].matl, a0[0], a0[1], a1[0], a1[1]);
                }
            }
        }
        F.n_alerts += (int)res.report.events.size();
        F.n_unused += (int)res.unused.size();
        const int before = (int)F.arena_brep.m_faces.size();
        em.emit(F.ds, *srf, in, res, src.m_faces[lf].reversed);
        const int after = (int)F.arena_brep.m_faces.size();
        for (int k = before; k < after; ++k) {
            F.face_operand.push_back(op);
            F.face_src.push_back(af);
        }
        if (after > before) ++F.n_faces_split;
        else ++F.n_faces_lost;   // a source face that produced nothing: the stage REFUSES
    }
    F.n_extended = em.extended_surfaces();
    F.n_inextensible = em.inextensible_faces();
    // THE FRONT-END SELF-CHECK, AND THE THIRD ACCEPTANCE CONDITION.
    //
    // Splitting a face may not change the solid it belongs to: each operand's face IMAGES must
    // still BE that operand -- closed, 2-manifold, and of exactly the operand's volume. This is
    // the only oracle-free statement available before selection runs (no analytic answer, no
    // reference kernel), it separates "the split is wrong" from "the selection is wrong", and it
    // is the condition that "the face split succeeded" actually means.
    //
    // It is not redundant with the two conditions above. MEASURED on sphere x sphere under the
    // 20 rigid motions: at several of them every face splits and every input is consumed, yet the
    // split solid integrates to 4.18878188 against the sphere's own 4.18879020 -- 2e-6 out,
    // because the imprinted section pcurve is an approximation and the two pieces then do not
    // tile the chart exactly. Those are precisely the motions whose boolean came back not-closed.
    {
        std::vector<int> ia, ib;
        for (int k = 0; k < (int)F.face_operand.size(); ++k)
            (F.face_operand[k] == 0 ? ia : ib).push_back(k);
        const v2v::V2Verdict va = v2v::v2_verdict(F.arena_brep.subset(ia));
        const v2v::V2Verdict vb = v2v::v2_verdict(F.arena_brep.subset(ib));
        const v2v::V2Verdict oa = v2v::v2_verdict(A);
        const v2v::V2Verdict ob = v2v::v2_verdict(B);
        const double rel = 1e-9;
        F.split_faithful =
            va.closed() && vb.closed() &&
            std::fabs(va.volume - oa.volume) <= rel * std::max(1e-30, std::fabs(oa.volume)) &&
            std::fabs(vb.volume - ob.volume) <= rel * std::max(1e-30, std::fabs(ob.volume));
        const char* p = std::getenv("SESSION_V2_SFDBG");
        if (p && p[0])
            std::printf("[SFDBG] SPLIT A{%s} want_vol=%.10f\n[SFDBG] SPLIT B{%s} want_vol=%.10f\n"
                        "[SFDBG] SPLIT faithful=%d\n",
                        va.str().c_str(), oa.volume, vb.str().c_str(), ob.volume,
                        (int)F.split_faithful);
    }
    // ACCEPTANCE GATE. The front end is used only when it split EVERY source face. Losing even
    // one face silently deletes material, which is exactly the failure mode this pipeline exists
    // to remove -- so a partial split is a REFUSAL, reported, and the v1 route runs instead
    // (guarantee I-13). Measured today: the per-face input assembly (the caller's job, and the
    // one piece v2sf deliberately does not do -- kb/v2_splitface_notes.md §9 gap 1) still loses
    // roughly half the faces of a box, so this gate fires on most pairs.
    //
    // AND EVERY INPUT EDGE CONSUMED. An UNUSED input is a section block the wire walk could not
    // place -- always because the section itself is incomplete, never because the walk is shy:
    // a half-imprinted section leaves the face watertight (a cut that does not close simply does
    // not cut) while classifying and selecting as though it had cut, which is the one failure
    // mode that produces a CLOSED AND WRONG answer. Measured over the ladder, the count separates
    // the pairs cleanly -- box/sphere/torus 0, cylinder x cylinder 4, cone x cone 2,
    // box x sphere 4, box x cone 2, sphere x cylinder 8 -- and every non-zero one is a pair whose
    // v2 answer was wrong. So it is a REFUSAL, reported, and the v1 route runs instead.
    F.ok = (F.arena_brep.face_count() > 0 && F.n_faces_lost == 0 && F.n_unused == 0 &&
            F.split_faithful);
    return F.ok;
}

///////////////////////////////////////////////////////////////////////////////////////////
// THE DRIVER
///////////////////////////////////////////////////////////////////////////////////////////

BRep v2_boolean(const BRep& A, const BRep& B, V2Op op, const V2BooleanOptions& opt,
                V2BooleanReport* report) {
    V2BooleanReport rep;
    const bool dbg = opt.verbose || v2sol_dbg();

    const V2Box boxA = v2sol_brep_box(A), boxB = v2sol_brep_box(B);
    V2Box both = boxA;
    both.add(boxB);
    const double scale = std::max(1e-9, both.diagonal());
    const double tol = opt.tolerance > 0 ? opt.tolerance : std::max(1e-9, scale * 1e-6);

    // ---- stage 0/1/2: arena + VV/VE/EE + VF/EF ------------------------------------------
    // The interference stages belong to v2int. Until the shared declarations settle, the split
    // stage below does the equivalent work implicitly (an EF interference is exactly a section
    // curve crossing a face boundary); the report records that rather than pretending otherwise.


    // ---- stages 3-5, v2: arena -> v2sec -> v2sf --------------------------------------------
    static const bool s_nofront = (std::getenv("SESSION_V2_NOFRONT") != nullptr);
    V2Front F;
    if (!s_nofront && v2sol_run_front(A, B, tol, F)) {
        rep.n_section_curves = F.n_curves;
        for (int o : F.face_operand) (o == 0 ? rep.n_faces_a : rep.n_faces_b)++;

        std::vector<int> idxA, idxB;
        for (int k = 0; k < (int)F.face_operand.size(); ++k)
            (F.face_operand[k] == 0 ? idxA : idxB).push_back(k);

        // outward sense of every face image, derived per operand the OCCT way (shell connexity +
        // per-shell signed volume), never by a weighted vote
        std::vector<double> osign((size_t)F.arena_brep.face_count(), 1.0);
        {
            const BRep sa = F.arena_brep.subset(idxA);
            const std::vector<double> s1 = v2_outward_signs(sa);
            for (size_t i = 0; i < idxA.size() && i < s1.size(); ++i) osign[idxA[i]] = s1[i];
            const BRep sb = F.arena_brep.subset(idxB);
            const std::vector<double> s2 = v2_outward_signs(sb);
            for (size_t i = 0; i < idxB.size() && i < s2.size(); ++i) osign[idxB[i]] = s2[i];
        }
        const std::vector<double> osignA = v2_outward_signs(A);
        const std::vector<double> osignB = v2_outward_signs(B);

        // stage 6: classify ONCE (guarantee I-10)
        const double cls_tol2 = std::max(1e-9, scale * 1e-5);
        const int nf = F.arena_brep.face_count();
        std::vector<SDState> st((size_t)nf, SDState::Out);
        for (int k = 0; k < nf; ++k)
            st[k] = sd_classify_face(F.arena_brep, k, F.face_operand[k] ? A : B, cls_tol2, osign,
                                     F.face_operand[k] ? osignA : osignB);
        {
            const char* p = std::getenv("SESSION_V2_SFDBG");
            if (p && p[0]) {
                static const char* nm[4] = {"In", "Out", "OnSame", "OnOpp"};
                for (int k = 0; k < nf; ++k) {
                    Point pp(0, 0, 0);
                    Vector nn;
                    const bool okp = v2_face_probe(F.arena_brep, k, pp, nn);
                    int nl = 0, nt = 0, nouter = 0;
                    for (int li : F.arena_brep.m_faces[k].loop_indices) {
                        ++nl;
                        nt += (int)F.arena_brep.m_loops[li].trim_indices.size();
                        if (F.arena_brep.m_loops[li].type == BRepLoopType::Outer) ++nouter;
                    }
                    std::vector<int> one(1, k);
                    const v2v::V2Verdict v1 = v2v::v2_verdict(F.arena_brep.subset(one));
                    std::printf("[SFDBG] cls k=%2d op=%d src=%d st=%-6s osign=%+.0f probe=%d"
                                "(%.4f,%.4f,%.4f) loops=%d outer=%d trims=%d area=%.6f\n",
                                k, F.face_operand[k], F.face_src[k], nm[(int)st[k]], osign[k],
                                (int)okp, pp[0], pp[1], pp[2], nl, nouter, nt, v1.area);
                }
            }
        }

        // stage 4 (post): fuse same-domain face images to ONE arena face index (P-1)
        std::vector<int> arena_id((size_t)nf);
        for (int k = 0; k < nf; ++k) arena_id[k] = k;
        std::vector<char> is_sd((size_t)nf, 0), sd_flip((size_t)nf, 0);
        {
            SameDomain sd(cls_tol2);
            std::vector<int> ia, ib;
            for (int k : idxA)
                if (st[k] == SDState::OnSame || st[k] == SDState::OnOpposite) {
                    sd.add_face(F.arena_brep, k, 0, 0);
                    ia.push_back(k);
                }
            const int cut = (int)ia.size();
            for (int k : idxB)
                if (st[k] == SDState::OnSame || st[k] == SDState::OnOpposite) {
                    sd.add_face(F.arena_brep, k, 1, 1);
                    ib.push_back(k);
                }
            rep.n_on = (int)(ia.size() + ib.size());
            if (!ia.empty() && !ib.empty()) {
                sd.detect();
                for (const auto& pr : sd.pairs()) {
                    int i0 = pr.i, j0 = pr.j;
                    if (i0 >= cut) std::swap(i0, j0);
                    if (i0 >= cut || j0 < cut) continue;
                    const int fa = ia[i0], fb = ib[j0 - cut];
                    if (is_sd[fb]) continue;
                    is_sd[fb] = 1;
                    arena_id[fb] = fa;
                    ++rep.n_sd_pairs;
                    Point pa;
                    Vector na;
                    if (v2_face_probe(F.arena_brep, fa, pa, na)) {
                        const int si = F.arena_brep.m_faces[fb].surface_index;
                        if (si >= 0 && si < (int)F.arena_brep.m_surfaces.size()) {
                            const NurbsSurface& sf = F.arena_brep.m_surfaces[si];
                            auto uv = sf.closest_parameters(pa);
                            const Vector nb = v2_vunit(sf.normal_at(uv.first, uv.second));
                            const Vector nb_out =
                                osign[fb] < 0 ? Vector(-nb[0], -nb[1], -nb[2]) : nb;
                            sd_flip[fb] = (nb_out.dot(na) < 0) ? 1 : 0;
                        }
                    }
                }
            }
        }

        // stage 7: selection (BuildBOP op-table, two fences)
        V2SelectionInput sin;
        V2InParts in_obj, in_too;
        for (int k : idxA) {
            V2OrientedFace of;
            of.face = arena_id[k];
            of.reversed = (osign[k] < 0);
            sin.object_faces.push_back(of);
            if (st[k] == SDState::In) {
                in_too.add(arena_id[k]);
                ++rep.n_in_b;
            }
        }
        for (int k : idxB) {
            V2OrientedFace of;
            of.face = arena_id[k];
            of.reversed = is_sd[k] ? (sd_flip[k] != 0) : (osign[k] < 0);
            sin.tool_faces.push_back(of);
            if (st[k] == SDState::In) {
                in_obj.add(arena_id[k]);
                ++rep.n_in_a;
            }
        }
        sin.in_objects = &in_obj;
        sin.in_tools = &in_too;

        std::vector<V2AlertRec> al2;
        const std::vector<V2OrientedFace> sel = v2_select_faces(sin, v2_op_states(op), al2);
        rep.n_selected = (int)sel.size();
        for (const auto& a : al2) rep.alerts.push_back(a);
        if (sel.empty()) {
            rep.empty_result = true;
            if (dbg) std::printf("[V2] %s\n", rep.str().c_str());
            if (report) *report = rep;
            return BRep();
        }

        // stage 8: assemble. NO sew, NO snap, NO co-refine, NO fuzzy: a section edge shared by a
        // face of A and a face of B is already ONE edge index of F.arena_brep, so subsetting
        // preserves the adjacency the shell walk needs.
        std::vector<int> ids;
        std::map<int, bool> revmap;
        for (const auto& of : sel) {
            if (of.face < 0 || of.face >= nf) continue;
            if (std::find(ids.begin(), ids.end(), of.face) == ids.end()) {
                ids.push_back(of.face);
                revmap[of.face] = of.reversed;
            }
        }
        std::sort(ids.begin(), ids.end());
        BRep out2 = F.arena_brep.subset(ids);
        for (size_t k = 0; k < ids.size() && k < out2.m_faces.size(); ++k)
            out2.m_faces[k].reversed = revmap[ids[k]];

        V2Topo topo2;
        topo2.build(out2);
        std::vector<V2OrientedFace> all2;
        for (int f = 0; f < topo2.nb_faces(); ++f) all2.push_back(V2OrientedFace{f, false});
        V2BuilderSolid bs2;
        bs2.set_topo(&topo2);
        bs2.set_faces(all2);
        bs2.set_avoid_internal_shapes(true);
        bs2.set_accept_open_shells(true);
        bs2.perform();
        rep.n_shells = (int)bs2.shells().size();
        rep.n_solids = (int)bs2.areas().size();
        rep.n_unused = (int)bs2.unused().size() + F.n_unused;
        for (const V2Shell& sh : bs2.shells()) {
            if (!sh.closed) ++rep.n_open_shells;
            if (sh.is_hole) ++rep.n_holes;
        }
        for (const auto& a : bs2.alerts()) rep.alerts.push_back(a);
        rep.verdict = v2_verdict(out2);
        rep.stage_fail.clear();
        if (dbg)
            std::printf("[V2] FRONT curves=%d secedges=%d secnodes=%d faces=%d/%d ext=%d inext=%d "
                        "| %s\n",
                        F.n_curves, F.n_section_edges, F.n_section_nodes, rep.n_faces_a,
                        rep.n_faces_b, F.n_extended, F.n_inextensible, rep.str().c_str());
        if (report) *report = rep;
        return out2;
    }
    rep.stage_fail = "v2-front-unavailable(v1-fallback)";

    // WHEN THE FRONT END REFUSES, THE SHIPPED KERNEL ANSWERS. Guarantee I-13 says the v1 route
    // runs instead; the v1 route is BRep::boolean, not the older v2 experiment below it, and the
    // difference is measurable: on cylinder x cylinder the front end correctly refuses (4 unused
    // section entries -- the marcher returns 3 blocks for a section that needs two closed curves)
    // and the experiment below then answers 0/20 closed where BRep::boolean answers 20/20. A
    // pipeline that refuses must not do worse than the thing it replaces. SESSION_V2_NODELEGATE
    // keeps the old behaviour so the experiment stays measurable.
    static const bool s_nodelegate = []() {
        const char* p = std::getenv("SESSION_V2_NODELEGATE");
        return p && p[0];
    }();
    if (!s_nodelegate) {
        BRep::BooleanOp bop = BRep::BooleanOp::Union;
        if (op == V2Op::Common) bop = BRep::BooleanOp::Intersection;
        else if (op == V2Op::Cut) bop = BRep::BooleanOp::Difference;
        else if (op == V2Op::Cut21) bop = BRep::BooleanOp::Difference;
        BRep out = (op == V2Op::Cut21) ? B.boolean(A, bop) : A.boolean(B, bop);
        rep.stage_fail = "v2-front-refused(delegated-to-kernel)";
        rep.verdict = v2_verdict(out);
        if (dbg) std::printf("[V2] %s\n", rep.str().c_str());
        if (report) *report = rep;
        return out;
    }

    // ---- stage 4 (pre): same-domain fence over the ORIGINAL faces ------------------------
    const auto sd_surf = v2sol_sd_surface_pairs(A, B, tol * 10.0);

    // ---- stage 3: FF section --------------------------------------------------------------
    V2SolSectionSet sec = v2sol_build_sections(A, B, tol, sd_surf);
    rep.n_surface_pairs = sec.n_pairs;
    rep.n_sd_surface_pairs = sec.n_sd_pairs;
    rep.n_section_curves = sec.n_curves;

    // ---- stage 5: split faces --------------------------------------------------------------
    // PREFERRED ROUTE (the one that satisfies P-1): build the SHARED section scaffold once, mint
    // ONE edge per section pave block from it, and drive BOTH operands' splits from that pool.
    // Every section edge then carries a provenance KEY {segment, start pave, end pave} and the
    // two operands' copies are matched by that INDEX TRIPLE below -- no distance is compared.
    // FALLBACK: the pcurve route of stage 3 when the scaffold produces no segments.
    V2SolHarvest ha, hb;
    std::map<int, std::array<int, 3>> secA, secB;      // split edge index -> {seg, v0, v1}
    std::map<int, std::array<double, 3>> spansA, spansB;
    bool used_scaffold = false;
    // MEASURED: the scaffold route is the RIGHT one in principle (it is the only route that
    // carries an index provenance key), but on quadric pairs its paving currently degrades the
    // split -- sphere x cylinder is exact at tilt 0 and wrong from 2.4 degrees on. Until v2sec
    // supplies the section, the pcurve route is the default and the scaffold is opt-in, so the
    // regression is recorded rather than shipped.
    static const bool s_scaf = (std::getenv("SESSION_V2_SCAF") != nullptr);
    if (s_scaf) {
        SectionScaffold scaf = build_section_scaffold(A, B, tol);
        if (!scaf.segments.empty()) {
            SharedEdgePool pool = build_shared_edge_pool(scaf);
            BRep a2 = A.split_by_brep(B, tol, false, nullptr, &scaf, true, &secA, nullptr,
                                      nullptr, &spansA, &pool);
            BRep b2 = B.split_by_brep(A, tol, false, nullptr, &scaf, false, &secB, nullptr,
                                      nullptr, &spansB, &pool);
            if (a2.face_count() > 0 && b2.face_count() > 0) {
                // OCCT pave-block normalisation: partial runs whose clip parameters disagree
                // across operands become identical per-block edges, so the keys line up.
                a2.recover_section_spans(scaf, spansA);
                b2.recover_section_spans(scaf, spansB);
                a2.normalize_section_blocks(scaf, spansA, &secA, "A");
                b2.normalize_section_blocks(scaf, spansB, &secB, "B");
                ha.split = a2;
                hb.split = b2;
                used_scaffold = true;
                rep.n_section_curves = (int)scaf.segments.size();
            }
        }
    }
    if (!used_scaffold) {
        secA.clear();
        secB.clear();
        bool any_a = false, any_b = false;
        for (const auto& v : sec.cuts_a)
            if (!v.empty()) any_a = true;
        for (const auto& v : sec.cuts_b)
            if (!v.empty()) any_b = true;
        ha.split = any_a ? A.split_by_brep(B, tol, false, &sec.cuts_a) : A;
        hb.split = any_b ? B.split_by_brep(A, tol, false, &sec.cuts_b) : B;
    }
    if (ha.split.face_count() == 0) ha.split = A;
    if (hb.split.face_count() == 0) hb.split = B;
    rep.stage_fail = used_scaffold ? "" : "scaffold-unavailable(pcurve-fallback)";
    rep.n_faces_a = ha.split.face_count();
    rep.n_faces_b = hb.split.face_count();

    // ---- harvest: outward orientation of every face image (BuildDraftSolid analog) ---------
    ha.osign = v2_outward_signs(ha.split, &ha.shells, &ha.holes);
    hb.osign = v2_outward_signs(hb.split, &hb.shells, &hb.holes);
    const std::vector<double> osignA = v2_outward_signs(A);
    const std::vector<double> osignB = v2_outward_signs(B);

    // ---- stage 6: classify ONCE (guarantee I-10) --------------------------------------------
    const double cls_tol = std::max(1e-9, scale * 1e-5);
    std::vector<SDState> stA((size_t)ha.split.face_count(), SDState::Out);
    std::vector<SDState> stB((size_t)hb.split.face_count(), SDState::Out);
    for (int f = 0; f < ha.split.face_count(); ++f)
        stA[f] = sd_classify_face(ha.split, f, B, cls_tol, ha.osign, osignB);
    for (int f = 0; f < hb.split.face_count(); ++f)
        stB[f] = sd_classify_face(hb.split, f, A, cls_tol, hb.osign, osignA);

    // ---- stage 4 (post): fuse same-domain face images to ONE arena face index (P-1) --------
    std::vector<int> arena_a((size_t)ha.split.face_count());
    std::vector<int> arena_b((size_t)hb.split.face_count());
    for (int f = 0; f < (int)arena_a.size(); ++f) arena_a[f] = f;
    const int nA = (int)arena_a.size();
    for (int f = 0; f < (int)arena_b.size(); ++f) arena_b[f] = nA + f;
    std::vector<char> b_is_sd((size_t)arena_b.size(), 0);
    std::vector<char> b_sd_flip((size_t)arena_b.size(), 0);
    {
        SameDomain sd(cls_tol);
        std::vector<int> ix_a, ix_b;
        for (int f = 0; f < (int)stA.size(); ++f)
            if (stA[f] == SDState::OnSame || stA[f] == SDState::OnOpposite) {
                sd.add_face(ha.split, f, 0, 0);
                ix_a.push_back(f);
            }
        const int split_at = (int)ix_a.size();
        for (int f = 0; f < (int)stB.size(); ++f)
            if (stB[f] == SDState::OnSame || stB[f] == SDState::OnOpposite) {
                sd.add_face(hb.split, f, 1, 1);
                ix_b.push_back(f);
            }
        rep.n_on = (int)(ix_a.size() + ix_b.size());
        if (!ix_a.empty() && !ix_b.empty()) {
            sd.detect();
            for (const auto& p : sd.pairs()) {
                int ia = p.i, ib = p.j;
                if (ia >= split_at) std::swap(ia, ib);
                if (ia >= split_at || ib < split_at) continue;   // same-operand pair: ignore
                const int fa = ix_a[ia];
                const int fb = ix_b[ib - split_at];
                if (b_is_sd[fb]) continue;
                b_is_sd[fb] = 1;
                arena_b[fb] = arena_a[fa];
                ++rep.n_sd_pairs;
                // express B's outward orientation against the REPRESENTATIVE's natural normal
                Point pa;
                Vector na;
                if (v2_face_probe(ha.split, fa, pa, na)) {
                    const int si = hb.split.m_faces[fb].surface_index;
                    if (si >= 0 && si < (int)hb.split.m_surfaces.size()) {
                        const NurbsSurface& s = hb.split.m_surfaces[si];
                        auto uv = s.closest_parameters(pa);
                        const Vector nb = v2_vunit(s.normal_at(uv.first, uv.second));
                        const Vector nb_out =
                            hb.osign[fb] < 0 ? Vector(-nb[0], -nb[1], -nb[2]) : nb;
                        b_sd_flip[fb] = (nb_out.dot(na) < 0) ? 1 : 0;
                    }
                }
            }
        }
    }

    // ---- stage 7: selection (BuildBOP op-table, two fences) --------------------------------
    V2SelectionInput in;
    V2InParts in_objects, in_tools;
    for (int f = 0; f < (int)stA.size(); ++f) {
        V2OrientedFace of;
        of.face = arena_a[f];
        of.reversed = (ha.osign[f] < 0);
        in.object_faces.push_back(of);
        if (stA[f] == SDState::In) {
            in_tools.add(arena_a[f]);
            ++rep.n_in_b;
        }
    }
    for (int f = 0; f < (int)stB.size(); ++f) {
        V2OrientedFace of;
        of.face = arena_b[f];
        of.reversed = b_is_sd[f] ? (b_sd_flip[f] != 0) : (hb.osign[f] < 0);
        in.tool_faces.push_back(of);
        if (stB[f] == SDState::In) {
            in_objects.add(arena_b[f]);
            ++rep.n_in_a;
        }
    }
    in.in_objects = &in_objects;
    in.in_tools = &in_tools;

    std::vector<V2AlertRec> alerts;
    const std::vector<V2OrientedFace> selected = v2_select_faces(in, v2_op_states(op), alerts);
    rep.n_selected = (int)selected.size();
    for (const auto& a : alerts) rep.alerts.push_back(a);

    if (selected.empty()) {
        rep.empty_result = true;
        if (dbg) std::printf("[V2] %s\n", rep.str().c_str());
        if (report) *report = rep;
        return BRep();
    }

    // ---- stage 8: assemble ------------------------------------------------------------------
    std::map<int, int> a_of_arena;
    for (int f = 0; f < (int)arena_a.size(); ++f) a_of_arena[arena_a[f]] = f;
    std::map<int, int> b_of_arena;
    for (int f = 0; f < (int)arena_b.size(); ++f)
        if (!b_is_sd[f]) b_of_arena[arena_b[f]] = f;

    std::vector<int> keepA, keepB;
    std::vector<char> revA, revB;
    for (const auto& of : selected) {
        auto ita = a_of_arena.find(of.face);
        if (ita != a_of_arena.end()) {
            keepA.push_back(ita->second);
            revA.push_back(of.reversed ? 1 : 0);
            continue;
        }
        auto itb = b_of_arena.find(of.face);
        if (itb != b_of_arena.end()) {
            keepB.push_back(itb->second);
            revB.push_back(of.reversed ? 1 : 0);
        }
    }

    auto build_side = [](const BRep& src, const std::vector<int>& keep,
                         const std::vector<char>& rev, std::map<int, int>& eremap) {
        std::vector<int> ids = keep;
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        BRep sub = src.subset(ids, &eremap);
        for (int k = 0; k < (int)ids.size() && k < (int)sub.m_faces.size(); ++k) {
            const auto it = std::find(keep.begin(), keep.end(), ids[k]);
            sub.m_faces[k].reversed = (rev[(size_t)(it - keep.begin())] != 0);
        }
        return sub;
    };

    BRep out;
    std::map<int, int> eremapA, eremapB;
    int edge_offset_b = 0;
    if (!keepA.empty()) out = build_side(ha.split, keepA, revA, eremapA);
    if (!keepB.empty()) {
        BRep sub = build_side(hb.split, keepB, revB, eremapB);
        if (out.face_count() == 0) {
            out = sub;
        } else {
            edge_offset_b = (int)out.m_topology_edges.size();
            out.append_brep(sub);
        }
    }
    if (out.face_count() == 0) {
        rep.empty_result = true;
        if (dbg) std::printf("[V2] %s\n", rep.str().c_str());
        if (report) *report = rep;
        return BRep();
    }

    // ---- arena edge identity ----------------------------------------------------------------
    // v2sf will hand us shared edge INDICES directly and this block disappears. Until then the
    // two operands' copies of a section edge are two entities and must be named as one. Both
    // copies came from the SAME 3D section curve computed once in stage 3, so snapping each copy
    // onto the exact sub-arc of that curve makes them identical geometry BEFORE any merge -- the
    // merge then names them, rather than guessing at a Hausdorff distance.
    // Tolerances are left at 0 so each pass derives its own from the RESULT's diagonal, exactly
    // as BRep::boolean does (src/brep.cpp:10912-10950); passing the operand-pair diagonal makes
    // the bands too wide on a small result and too narrow on a large one.
    // PROVENANCE MERGE (guarantee I-1). Two section edges are ONE edge iff their scaffold keys
    // {segment, start pave vertex, end pave vertex} are equal -- an index triple recorded by the
    // splitter, never a distance. This is the step that makes the shell walk below see A's and
    // B's face images as adjacent, and it is the piece v2sf will supply directly.
    int n_key_merged = 0;
    if (used_scaffold && !secA.empty() && !secB.empty()) {
        std::map<std::array<int, 3>, std::vector<int>> by_key;
        for (const auto& kv : secA) {
            auto it = eremapA.find(kv.first);
            if (it != eremapA.end()) by_key[kv.second].push_back(it->second);
        }
        for (const auto& kv : secB) {
            auto it = eremapB.find(kv.first);
            if (it != eremapB.end()) by_key[kv.second].push_back(it->second + edge_offset_b);
        }
        for (const auto& kv : by_key) {
            const auto& es = kv.second;
            if (es.size() < 2) continue;
            const int keep_e = es[0];
            if (keep_e < 0 || keep_e >= (int)out.m_topology_edges.size()) continue;
            for (size_t i = 1; i < es.size(); ++i) {
                const int drop_e = es[i];
                if (drop_e == keep_e || drop_e < 0 || drop_e >= (int)out.m_topology_edges.size())
                    continue;
                for (int t : out.m_topology_edges[drop_e].trim_indices) {
                    if (t < 0 || t >= (int)out.m_trims.size()) continue;
                    out.m_trims[t].edge_index = keep_e;
                    out.m_topology_edges[keep_e].trim_indices.push_back(t);
                }
                out.m_topology_edges[drop_e].trim_indices.clear();
                ++n_key_merged;
            }
        }
    }
    auto trace = [&](const char* tag) {
        if (dbg) std::printf("[V2]   merge %-11s %s\n", tag, v2_verdict(out).str().c_str());
    };
    trace("combined");
    if (n_key_merged) trace("key-merge");

    static const int s_merge = std::getenv("SESSION_V2_MERGE")
                                   ? std::atoi(std::getenv("SESSION_V2_MERGE"))
                                   : 1;
    if (s_merge != 2) {
        out.imprint_edges(0.0, false);                              // T-junctions first
        if (!sec.c3d.empty()) out.snap_section_edges(sec.c3d, 0.0); // both copies -> ONE geometry
        out.co_refine_coincident_edges(0.0);                        // 1:1 segmentation
        out.sew_coincident_edges(0.0);                              // name them one edge
        trace("geometric");
    }
    // TOLERANT CLOSURE FALLBACK (OCCT SetFuzzyValue analog, made VISIBLE). At sub-tolerance
    // grazing the two operands trim along slightly different curves, so their kept boundaries
    // stagger and no exact method names them one edge. Re-running the naming passes with an
    // inflated band closes that rim. It is approximate BY DESIGN, so it fires only after the
    // exact passes have left naked edges and it is REPORTED (rep.fuzzy_steps) -- a result built
    // along this path is distinguishable from one built along the main path (guarantee I-13).
    static const bool s_fuzzy = (std::getenv("SESSION_V2_FUZZY") != nullptr);
    if (s_fuzzy && v2_verdict(out).naked > 0) {
        const BRep exact = out;   // revert target: never DEGRADE an exact result
        const double d = std::max(1e-9, v2sol_brep_box(out).diagonal());
        bool closed_now = false;
        for (int k = 1; k <= 3 && !closed_now; ++k) {
            const double ftol = d * 5e-3 * (1 << k);
            out.imprint_edges(ftol, true);
            out.co_refine_coincident_edges(ftol);
            out.sew_coincident_edges(ftol);
            rep.fuzzy_steps = k;
            closed_now = (v2_verdict(out).naked == 0);
            trace("fuzzy");
        }
        if (!closed_now) {
            out = exact;
            rep.fuzzy_steps = 0;
        }
    }
    if (s_merge >= 1) {
        // P0 shared section-edge backbone: re-fit every still-unmated section arc to the exact
        // sub-arc of the ONE pair-once section curve and merge. This is the construction-time
        // form of the identity guarantee and is what v2sf will make unnecessary.
        const V2Verdict pre = v2_verdict(out);
        if (pre.naked > 0) {
            out.make_shared_section_edges(A, B, 0.0);
            trace("shared_sec");
        }
    }

    // ---- shells and solids ------------------------------------------------------------------
    V2Topo topo;
    topo.build(out);
    std::vector<V2OrientedFace> all;
    for (int f = 0; f < topo.nb_faces(); ++f) all.push_back(V2OrientedFace{f, false});
    V2BuilderSolid bs;
    bs.set_topo(&topo);
    bs.set_faces(all);
    bs.set_avoid_internal_shapes(true);
    bs.set_accept_open_shells(true);   // report open shells; never silently drop material
    bs.perform();

    rep.n_shells = (int)bs.shells().size();
    rep.n_solids = (int)bs.areas().size();
    rep.n_unused = (int)bs.unused().size();
    for (const V2Shell& sh : bs.shells()) {
        if (!sh.closed) ++rep.n_open_shells;
        if (sh.is_hole) ++rep.n_holes;
    }
    for (const auto& a : bs.alerts()) rep.alerts.push_back(a);
    rep.verdict = v2_verdict(out);

    if (dbg) std::printf("[V2] %s\n", rep.str().c_str());
    if (report) *report = rep;
    return out;
}

BRep v2_cut(const BRep& A, const BRep& B, double tol, V2BooleanReport* r) {
    V2BooleanOptions o;
    o.tolerance = tol;
    return v2_boolean(A, B, V2Op::Cut, o, r);
}
BRep v2_common(const BRep& A, const BRep& B, double tol, V2BooleanReport* r) {
    V2BooleanOptions o;
    o.tolerance = tol;
    return v2_boolean(A, B, V2Op::Common, o, r);
}
BRep v2_fuse(const BRep& A, const BRep& B, double tol, V2BooleanReport* r) {
    V2BooleanOptions o;
    o.tolerance = tol;
    return v2_boolean(A, B, V2Op::Fuse, o, r);
}
BRep v2_cut21(const BRep& A, const BRep& B, double tol, V2BooleanReport* r) {
    V2BooleanOptions o;
    o.tolerance = tol;
    return v2_boolean(A, B, V2Op::Cut21, o, r);
}

}  // namespace v2sol
}  // namespace session_cpp
