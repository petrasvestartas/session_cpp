// main_14 — FACE SPLITTER test driver for src/brep_v2_splitface.h/.cpp.
//
// Every cell states an ORACLE-FREE invariant: something checkable from the output alone.
//
// ORDER MATTERS. The first block validates the VERDICT METRIC itself (sf_manifold) against the
// kernel's own BRep::is_solid() on three known-good UNSPLIT solids — a sphere (degenerate pole
// edges), a cone (degenerate apex edge) and a cylinder (a seam edge used twice by ONE face).
// A bare "trims != 2 => naked" count reads all three as open; only after the metric agrees with
// is_solid() on all three is it used to score any split result.
//
// Prints one line per cell: [SF] <name> PASS|FAIL <detail>; exit 0 iff all pass.
#include "src/brep.h"
#include "src/brep_bds.h"
#include "src/brep_massprops.h"
#include "src/v2/brep_v2_splitface.h"
#include "src/v2/v2_verdict.h"
#include "src/nurbscurve.h"
#include "src/nurbssurface.h"
#include "src/primitives.h"
#include "src/point.h"
#include "src/vector.h"
#include "src/xform.h"
#include <cmath>
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace session_cpp;
using namespace session_cpp::v2sf;

static int g_pass = 0, g_fail = 0;

static void cell(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("[SF] %-58s %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL",
                detail.empty() ? "" : "  ", detail.c_str());
    ok ? ++g_pass : ++g_fail;
}

static std::string sfmt(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[1024];
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return std::string(buf);
}

static const double PI = 3.14159265358979323846;

/// Which pole convention run_sphere emits; see SfEmitter::set_pole_edges.
static bool g_pole_edges = true;

static NurbsCurve line3(const Point& a, const Point& b) {
    std::vector<Point> p;
    p.push_back(a);
    p.push_back(b);
    return NurbsCurve::create(false, 1, p);
}
static NurbsCurve uvline(double u0, double v0, double u1, double v1) {
    return line3(Point(u0, v0, 0), Point(u1, v1, 0));
}
static NurbsCurve polyline_curve(const std::vector<Point>& pts) {
    return NurbsCurve::create(false, 1, pts);
}

/// Naive "an edge with one face use is naked" count — the measurement error this driver exists
/// to avoid. Reported next to sf_manifold so the difference is visible, never used as a verdict.
static int naive_naked(const BRep& b) {
    int n = 0;
    for (size_t i = 0; i < b.m_topology_edges.size(); ++i)
        if ((int)b.m_topology_edges[i].trim_indices.size() != 2) ++n;
    return n;
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 1 — VALIDATE THE VERDICT METRIC BEFORE SCORING ANYTHING
///////////////////////////////////////////////////////////////////////////////////////////

static void run_verdict_validation() {
    struct Case { const char* name; BRep b; };
    std::vector<Case> cs;
    Case c0; c0.name = "sphere_poles";     c0.b = BRep::create_sphere(1.0);       cs.push_back(c0);
    Case c1; c1.name = "cone_apex";        c1.b = BRep::create_cone(1.0, 2.0);    cs.push_back(c1);
    Case c2; c2.name = "cylinder_seam";    c2.b = BRep::create_cylinder(1.0, 2.0);cs.push_back(c2);
    Case c3; c3.name = "box_no_degeneracy";c3.b = BRep::create_box(2, 2, 2);      cs.push_back(c3);

    bool all_agree = true;
    bool naive_would_lie = false;
    for (size_t i = 0; i < cs.size(); ++i) {
        const SfManifold m = sf_manifold(cs[i].b);
        const bool solid = cs[i].b.is_solid();
        const int naive = naive_naked(cs[i].b);
        const bool agree = (m.closed() == solid);
        all_agree = all_agree && agree;
        if (naive > 0 && solid) naive_would_lie = true;
        cell(sfmt("verdict %s metric==is_solid", cs[i].name), agree,
             sfmt("is_solid=%d sf_closed=%d edges=%d naked=%d nonman=%d degen=%d seam=%d "
                  "naive_naked=%d",
                  (int)solid, (int)m.closed(), m.edges, m.naked, m.nonmanifold, m.degenerate,
                  m.seam_edges, naive));
    }
    cell("verdict metric_agrees_on_all_four_known_good_solids", all_agree);
    (void)naive_would_lie;   // see run_verdict_pole_shape(): the kernel primitives carry no
                             // pole rows, so the trap cannot show up on them.
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 2 — THE METRIC AND THE SORTING KEY
///////////////////////////////////////////////////////////////////////////////////////////

/// Bilinear plane with |S_u| = a and |S_v| = b, domain [0,1]^2. Anisotropy is a/b.
static NurbsSurface aniso_plane(double a, double b) {
    NurbsSurface s;
    s.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    s.set_cv(0, 0, Point(0, 0, 0));
    s.set_cv(1, 0, Point(a, 0, 0));
    s.set_cv(0, 1, Point(0, b, 0));
    s.set_cv(1, 1, Point(a, b, 0));
    return s;
}

static void run_metric() {
    {
        NurbsSurface s = aniso_plane(3.0, 7.0);
        double E, F, G, sq;
        const bool ok = sf_first_form(s, 0.5, 0.5, E, F, G, sq);
        cell("metric first_form_of_orthogonal_plane",
             ok && std::fabs(E - 9.0) < 1e-9 && std::fabs(G - 49.0) < 1e-9 &&
                 std::fabs(F) < 1e-9 && std::fabs(sq - 21.0) < 1e-9,
             sfmt("E=%.9g F=%.3g G=%.9g sqrtdet=%.9g", E, F, G, sq));
    }
    {
        // On an ISOTROPIC unit patch the metric key must reduce EXACTLY to raw atan2.
        NurbsSurface s = aniso_plane(1.0, 1.0);
        bool same = true;
        for (int k = 0; k < 32; ++k) {
            const double th = 2.0 * PI * k / 32.0;
            const double m = sf_metric_angle(s, 0.5, 0.5, std::cos(th), std::sin(th), 0);
            const double r = sf_raw_angle(std::cos(th), std::sin(th));
            if (std::fabs(m - r) > 1e-14) same = false;
        }
        cell("metric reduces_to_raw_atan2_on_isotropic_patch", same);
    }
    {
        // THE DISCRIMINATING MEASUREMENT. Four directions that are exactly 90 degrees apart ON
        // THE SURFACE, on a patch with |S_u|/|S_v| = 1e6. The metric key recovers 90 degrees;
        // the raw UV key collapses all four into two clusters ~2e-6 rad wide, i.e. it loses ~6
        // decimal digits of the only information the wire walk has at a multi-edge vertex.
        const double A = 1e6, B = 1.0;
        NurbsSurface s = aniso_plane(A, B);
        double mang[4], rang[4];
        for (int k = 0; k < 4; ++k) {
            const double th = PI * 0.25 + PI * 0.5 * k;   // 45, 135, 225, 315 degrees
            const double du = std::cos(th) / A, dv = std::sin(th) / B;
            mang[k] = sf_metric_angle(s, 0.5, 0.5, du, dv, 0);
            rang[k] = sf_raw_angle(du, dv);
        }
        double mworst = 1e9, rworst = 1e9;
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j) {
                double dm = std::fabs(mang[i] - mang[j]);
                dm = std::min(dm, 2 * PI - dm);
                double dr = std::fabs(rang[i] - rang[j]);
                dr = std::min(dr, 2 * PI - dr);
                mworst = std::min(mworst, dm);
                rworst = std::min(rworst, dr);
            }
        bool exact = true;
        for (int k = 0; k < 4; ++k)
            if (std::fabs(mang[k] - (PI * 0.25 + PI * 0.5 * k)) > 1e-9) exact = false;
        cell("metric recovers_true_surface_angles_at_1e6_anisotropy", exact,
             sfmt("angles=%.6f,%.6f,%.6f,%.6f", mang[0], mang[1], mang[2], mang[3]));
        cell("metric key_separation_metric>>raw  (why the correction is load-bearing)",
             mworst > 1.5 && rworst < 1e-5,
             sfmt("min_gap metric=%.6f rad  raw=%.3e rad  ratio=%.3g", mworst, rworst,
                  mworst / rworst));
    }
    {
        // G7: no epsilon is a fraction of the UV domain. The same geometric plane presented on
        // [0,1]^2 and on [-0.04,4.04]^2 must give UV tolerances that differ by exactly the
        // parameter-scale ratio, and du*|S_u| == tol3d in both.
        NurbsSurface a = aniso_plane(2.0, 2.0);
        NurbsSurface b = aniso_plane(2.0, 2.0);
        b.set_domain(0, -0.04, 4.04);
        b.set_domain(1, -0.04, 4.04);
        double dua, dva, dub, dvb;
        const bool oa = sf_uv_tolerance(a, 0.5, 0.5, 1e-7, dua, dva);
        const bool ob = sf_uv_tolerance(b, 2.0, 2.0, 1e-7, dub, dvb);
        const double ratio = dub / dua;
        cell("metric uv_tolerance_is_a_3d_distance_not_a_domain_fraction",
             oa && ob && std::fabs(ratio - 4.08) < 1e-9 &&
                 std::fabs(dua * 2.0 - 1e-7) < 1e-20,
             sfmt("du[0,1]=%.6e du[-.04,4.04]=%.6e ratio=%.9f", dua, dub, ratio));
    }
    {
        // A pole is TYPED, not a lucky near-zero.
        NurbsSurface sph = Primitives::sphere_surface(0, 0, 0, 1.0);
        double du, dv;
        const bool ok = sf_uv_tolerance(sph, 1.3, 0.0, 1e-7, du, dv);
        double E, F, G, sq;
        const bool ff = sf_first_form(sph, 1.3, 0.0, E, F, G, sq);
        cell("metric pole_is_reported_not_absorbed", !ok && !ff,
             sfmt("uv_tol_ok=%d first_form_ok=%d", (int)ok, (int)ff));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Shared helpers for the split tests
///////////////////////////////////////////////////////////////////////////////////////////

/// Exact affine UV inverse for a bilinear parallelogram patch (all planar faces below).
static std::pair<double, double> plane_uv(const NurbsSurface& s, const Point& q) {
    const std::pair<double, double> du = s.domain(0);
    const std::pair<double, double> dv = s.domain(1);
    const Point p00 = s.point_at(du.first, dv.first);
    const Point p10 = s.point_at(du.second, dv.first);
    const Point p01 = s.point_at(du.first, dv.second);
    const Vector eu = p10 - p00, ev = p01 - p00, d = q - p00;
    const double a = eu.dot(eu), b = eu.dot(ev), c = ev.dot(ev);
    const double x = eu.dot(d), y = ev.dot(d);
    const double det = a * c - b * b;
    const double su = (x * c - y * b) / det;
    const double sv = (y * a - x * b) / det;
    return std::make_pair(du.first + su * (du.second - du.first),
                          dv.first + sv * (dv.second - dv.first));
}

/// pcurve of a pave block on a PLANAR face, in the block's NATURAL direction (pave1 -> pave2).
static NurbsCurve plane_pcurve(const NurbsSurface& s, const BdsArena& ar, const BdsPB& pb) {
    const std::pair<double, double> a = plane_uv(s, ar.vertex_point(pb->pave1.vertex));
    const std::pair<double, double> b = plane_uv(s, ar.vertex_point(pb->pave2.vertex));
    return uvline(a.first, a.second, b.first, b.second);
}

/// Canonical textual signature of a split result: for every output face, every wire, the
/// SEQUENCE of (arena edge, pave-block, sense). Two runs that agree here agree index-for-index.
static std::string result_signature(const BdsArena& ar, const std::vector<SfInputEdge>& in,
                                    const SfResult& r) {
    std::string s;
    for (size_t f = 0; f < r.faces.size(); ++f) {
        s += "F{";
        std::vector<const SfWire*> ws;
        ws.push_back(&r.faces[f].outer);
        for (size_t h = 0; h < r.faces[f].holes.size(); ++h) ws.push_back(&r.faces[f].holes[h]);
        for (size_t h = 0; h < r.faces[f].internals.size(); ++h)
            ws.push_back(&r.faces[f].internals[h]);
        for (size_t w = 0; w < ws.size(); ++w) {
            s += "W[";
            for (size_t k = 0; k < ws[w]->inputs.size(); ++k) {
                const SfInputEdge& e = in[(size_t)ws[w]->inputs[k]];
                const SfPaveBlockId id = sf_pave_block_id(ar, e.pb.get());
                s += sfmt("%d.%d.%d;", id.edge, id.block, (int)e.sense);
            }
            s += "]";
        }
        s += "}";
    }
    return s;
}


/// SCORING GOES THROUGH THE SHARED HARNESS (src/v2/v2_verdict.h, validated against
/// BRep::is_solid() on five primitives by main_17). sf_manifold is this file's cheap
/// topology-only census; every scored shape asserts the two AGREE, so a per-agent metric can
/// never quietly disagree with the kernel's own validity rule.
static void verdict_cell(const std::string& name, const BRep& b, const SfManifold& m,
                         int want_faces, double want_volume) {
    const session_cpp::v2v::V2Verdict v = session_cpp::v2v::v2_verdict(b);
    const bool agree = (v.naked_real == m.naked) && (v.nonmanifold == m.nonmanifold) &&
                       (v.degenerate == m.degenerate) && (v.seam_edges == m.seam_edges);
    const bool vol_ok = (want_volume < 0.0) ||
                        (v.volume_valid && std::fabs(v.volume - want_volume) < 1e-9);
    cell(name, agree && v.closed() && v.faces == want_faces && b.is_solid() && vol_ok,
         sfmt("v2v{faces=%d shells=%d solids=%d naked_real=%d nonman=%d seam=%d degen=%d "
              "closure=%.2e vol=%.10f valid=%d} sf{naked=%d nonman=%d seam=%d degen=%d} "
              "is_solid=%d agree=%d",
              v.faces, v.shells, v.solids, v.naked_real, v.nonmanifold, v.seam_edges,
              v.degenerate, v.closure_residual, v.volume, (int)v.volume_valid, m.naked,
              m.nonmanifold, m.seam_edges, m.degenerate, (int)b.is_solid(), (int)agree));
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 3 — THE DECISIVE TEST: a box split on a canonical and on a PADDED UV domain
///////////////////////////////////////////////////////////////////////////////////////////

static const int FV[6][4] = {
    {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}
};
static const int EV[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}
};

/// A box of half-extent h. `padded`: each face's SURFACE spans 1% beyond the face and carries
/// the domain [-0.04, 4.04], so the face's trims land at exactly {0, 4} and NEVER touch the
/// surface domain border — the STEP round-trip shape that makes a boundary snap impossible.
/// Canonical: the surface spans exactly the face on [0,1]^2, trims at {0,1} (border-snappable).
static BRep make_box(double h, bool padded) {
    BRep b;
    Point corners[8] = {
        Point(-h, -h, -h), Point(h, -h, -h), Point(h, h, -h), Point(-h, h, -h),
        Point(-h, -h, h),  Point(h, -h, h),  Point(h, h, h),  Point(-h, h, h)
    };
    for (int i = 0; i < 8; ++i) b.add_vertex(corners[i]);
    for (int i = 0; i < 12; ++i)
        b.add_curve_3d(line3(corners[EV[i][0]], corners[EV[i][1]]));
    for (int i = 0; i < 8; ++i) {
        BRepVertex tv;
        tv.point_index = i;
        b.m_topology_vertices.push_back(tv);
    }
    for (int i = 0; i < 12; ++i) b.add_edge(i, EV[i][0], EV[i][1]);

    const double d0 = padded ? -0.04 : 0.0;
    const double d1 = padded ? 4.04 : 1.0;
    const double tlo = 0.0;
    const double thi = padded ? 4.0 : 1.0;

    for (int f = 0; f < 6; ++f) {
        const int* fv = FV[f];
        Point p[4] = {corners[fv[0]], corners[fv[1]], corners[fv[2]], corners[fv[3]]};
        if (padded) {
            Point ctr((p[0][0] + p[1][0] + p[2][0] + p[3][0]) * 0.25,
                      (p[0][1] + p[1][1] + p[2][1] + p[3][1]) * 0.25,
                      (p[0][2] + p[1][2] + p[2][2] + p[3][2]) * 0.25);
            for (int k = 0; k < 4; ++k)
                p[k] = Point(ctr[0] + (p[k][0] - ctr[0]) * 1.02,
                             ctr[1] + (p[k][1] - ctr[1]) * 1.02,
                             ctr[2] + (p[k][2] - ctr[2]) * 1.02);
        }
        NurbsSurface s;
        s.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        s.set_cv(0, 0, p[0]);
        s.set_cv(1, 0, p[1]);
        s.set_cv(1, 1, p[2]);
        s.set_cv(0, 1, p[3]);
        s.set_domain(0, d0, d1);
        s.set_domain(1, d0, d1);
        const int si = b.add_surface(s);
        const int fi = b.add_face(si, false);
        const int li = b.add_loop(fi, BRepLoopType::Outer);
        Point uvc[4] = {Point(tlo, tlo, 0), Point(thi, tlo, 0), Point(thi, thi, 0),
                        Point(tlo, thi, 0)};
        for (int k = 0; k < 4; ++k) {
            const int nx = (k + 1) % 4;
            const int ci = b.add_curve_2d(line3(uvc[k], uvc[nx]));
            int e = -1;
            for (int q = 0; q < 12; ++q)
                if ((EV[q][0] == fv[k] && EV[q][1] == fv[nx]) ||
                    (EV[q][0] == fv[nx] && EV[q][1] == fv[k])) { e = q; break; }
            b.add_trim(ci, e, li, EV[e][0] != fv[k], BRepTrimType::Mated);
        }
    }
    for (int ei = 0; ei < 12; ++ei) {
        b.m_topology_vertices[(size_t)EV[ei][0]].edge_indices.push_back(ei);
        b.m_topology_vertices[(size_t)EV[ei][1]].edge_indices.push_back(ei);
    }
    return b;
}

struct BoxOut {
    BRep result;
    int split_faces = 0;
    std::string signature;
    SfManifold man;
    bool solid = false;
    int minted_e = 0, minted_v = 0;
    int alerts = 0;
    std::vector<std::pair<SfPaveBlockId, std::set<int> > > pb_faces;  ///< pave block -> face set
    double area = 0.0, volume = 0.0;
    int reused = 0;
};

static void run_box(bool padded, BoxOut& out) {
    BRep box = make_box(1.0, padded);
    BdsArena arena;
    std::vector<const BRep*> ops;
    ops.push_back(&box);
    arena.init(ops);
    // The arena builds an edge's pave-block pool LAZILY. Materialise all twelve now so every
    // face is handed the SAME BdsPB objects for its boundary intervals.
    for (int k = 0; k < 12; ++k) arena.change_pave_blocks(arena.index_of_edge(0, k));

    int midv[4];
    for (int k = 0; k < 4; ++k) {
        const int ae = arena.index_of_edge(0, 8 + k);
        const std::pair<double, double> d = arena.edge_range(ae);
        const double t = 0.5 * (d.first + d.second);
        midv[k] = arena.append_vertex(arena.edge_curve(ae)->point_at(t), SF_CONFUSION);
        arena.add_pave(ae, t, midv[k]);
    }
    arena.update_pave_blocks();

    int sec[4];
    for (int k = 0; k < 4; ++k)
        sec[k] = arena.append_edge(line3(arena.vertex_point(midv[k]),
                                         arena.vertex_point(midv[(k + 1) % 4])),
                                   midv[k], midv[(k + 1) % 4], SF_CONFUSION);

    SfEmitter em(out.result);
    std::map<SfPaveBlockId, std::set<int> > pbf;

    for (int f = 0; f < 6; ++f) {
        const NurbsSurface& srf = box.m_surfaces[(size_t)box.m_faces[(size_t)f].surface_index];
        std::deque<NurbsCurve> pool;
        std::vector<SfInputEdge> in;
        for (size_t q = 0; q < box.m_faces[(size_t)f].loop_indices.size(); ++q) {
            const BRepLoop& lp = box.m_loops[(size_t)box.m_faces[(size_t)f].loop_indices[q]];
            for (size_t z = 0; z < lp.trim_indices.size(); ++z) {
                const BRepTrim& tm = box.m_trims[(size_t)lp.trim_indices[z]];
                const int ae = arena.index_of_edge(0, tm.edge_index);
                const std::vector<BdsPB>& pool_pb = arena.pave_blocks(ae);
                for (size_t p = 0; p < pool_pb.size(); ++p) {
                    SfInputEdge e;
                    e.pb = pool_pb[p];
                    e.sense = tm.reversed ? SfSense::Reversed : SfSense::Forward;
                    pool.push_back(plane_pcurve(srf, arena, pool_pb[p]));
                    e.pcurve = &pool.back();
                    in.push_back(e);
                }
            }
        }
        if (f >= 2) {
            const std::vector<BdsPB>& sp = arena.pave_blocks(sec[f - 2]);
            for (size_t p = 0; p < sp.size(); ++p) {
                pool.push_back(plane_pcurve(srf, arena, sp[p]));
                const NurbsCurve* pc = &pool.back();
                SfInputEdge a;
                a.pb = sp[p]; a.sense = SfSense::Forward; a.pcurve = pc;
                SfInputEdge r;
                r.pb = sp[p]; r.sense = SfSense::Reversed; r.pcurve = pc;
                in.push_back(a);
                in.push_back(r);
            }
        }
        const SfResult res = sf_split_face(arena, srf, in);
        out.split_faces += (int)res.faces.size();
        out.signature += sfmt("[%d]", f) + result_signature(arena, in, res);
        out.minted_e += res.report.minted_edges;
        out.minted_v += res.report.minted_vertices;
        out.alerts += (int)res.report.events.size();

        const int base = (int)out.result.m_faces.size();
        em.emit(arena, srf, in, res, box.m_faces[(size_t)f].reversed);
        for (size_t a = 0; a < res.faces.size(); ++a) {
            const SfFace& A = res.faces[a];
            for (size_t k = 0; k < A.outer.inputs.size(); ++k)
                pbf[sf_pave_block_id(arena, in[(size_t)A.outer.inputs[k]].pb.get())]
                    .insert(base + (int)a);
            for (size_t hh = 0; hh < A.holes.size(); ++hh)
                for (size_t k = 0; k < A.holes[hh].inputs.size(); ++k)
                    pbf[sf_pave_block_id(arena, in[(size_t)A.holes[hh].inputs[k]].pb.get())]
                        .insert(base + (int)a);
        }
    }
    out.reused = em.reused_edges();
    out.man = sf_manifold(out.result);
    out.solid = out.result.is_solid();
    for (std::map<SfPaveBlockId, std::set<int> >::const_iterator it = pbf.begin();
         it != pbf.end(); ++it)
        out.pb_faces.push_back(*it);

    MassPropsOptions mo;
    mo.rel_tolerance = 1e-10;
    const MassProps mp = brep_massprops(out.result, mo);
    out.area = mp.area;
    out.volume = mp.volume;
}

static void run_decisive_box() {
    BoxOut can, pad;
    run_box(false, can);
    run_box(true, pad);

    cell("box CANONICAL[0,1] 10_faces_20_edges_0_naked",
         can.split_faces == 10 && can.man.edges == 20 && can.man.naked == 0 &&
             can.man.nonmanifold == 0 && can.solid,
         sfmt("faces=%d edges=%d naked=%d nonman=%d degen=%d solid=%d naive_naked=%d",
              can.split_faces, can.man.edges, can.man.naked, can.man.nonmanifold,
              can.man.degenerate, (int)can.solid, naive_naked(can.result)));

    cell("box PADDED[-0.04,4.04] 10_faces_20_edges_0_naked",
         pad.split_faces == 10 && pad.man.edges == 20 && pad.man.naked == 0 &&
             pad.man.nonmanifold == 0 && pad.solid,
         sfmt("faces=%d edges=%d naked=%d nonman=%d degen=%d solid=%d naive_naked=%d "
              "(v1 kernel here: 32 naked of 36)",
              pad.split_faces, pad.man.edges, pad.man.naked, pad.man.nonmanifold,
              pad.man.degenerate, (int)pad.solid, naive_naked(pad.result)));

    cell("box padded_and_canonical_topology_IDENTICAL_index_for_index",
         can.signature == pad.signature,
         sfmt("sig_len=%d/%d", (int)can.signature.size(), (int)pad.signature.size()));

    cell("box every_edge_has_exactly_2_face_uses_both_domains",
         can.man.naked == 0 && pad.man.naked == 0 && can.man.nonmanifold == 0 &&
             pad.man.nonmanifold == 0);

    cell("box zero_minting_G1", can.minted_e == 0 && can.minted_v == 0 && pad.minted_e == 0 &&
                                    pad.minted_v == 0);
    cell("box no_alerts_on_clean_input", can.alerts == 0 && pad.alerts == 0,
         sfmt("canonical=%d padded=%d", can.alerts, pad.alerts));

    // ADJACENCY BY IDENTITY: every pave block that the split consumed twice must name the SAME
    // arena (edge, block) pair in BOTH faces. This is an INDEX comparison, not a geometric one.
    int shared = 0, bad = 0;
    for (size_t i = 0; i < can.pb_faces.size(); ++i) {
        if (!can.pb_faces[i].first.valid()) ++bad;
        if (can.pb_faces[i].second.size() == 2) ++shared;
        if (can.pb_faces[i].second.size() > 2) ++bad;
    }
    cell("box adjacency_is_the_SAME_BdsPaveBlock_INDEX_in_both_faces",
         bad == 0 && shared == 20 && (int)can.pb_faces.size() == 20,
         sfmt("distinct_pave_blocks=%d shared_by_exactly_2_faces=%d bad=%d emitter_reuses=%d",
              (int)can.pb_faces.size(), shared, bad, can.reused));

    bool same_pb = (can.pb_faces.size() == pad.pb_faces.size());
    for (size_t i = 0; i < can.pb_faces.size() && same_pb; ++i)
        if (!(can.pb_faces[i].first == pad.pb_faces[i].first) ||
            can.pb_faces[i].second != pad.pb_faces[i].second)
            same_pb = false;
    cell("box adjacency_map_identical_canonical_vs_padded", same_pb);

    cell("box mass_properties_exact_both_domains",
         std::fabs(can.area - 24.0) < 1e-9 && std::fabs(can.volume - 8.0) < 1e-9 &&
             std::fabs(pad.area - 24.0) < 1e-9 && std::fabs(pad.volume - 8.0) < 1e-9,
         sfmt("canonical A=%.12f V=%.12f  padded A=%.12f V=%.12f", can.area, can.volume,
              pad.area, pad.volume));

    // Determinism: the same input twice, byte-identical output.
    BoxOut again;
    run_box(true, again);
    cell("box determinism_byte_identical_second_run", again.signature == pad.signature);

    verdict_cell("box CANONICAL shared_v2v_verdict", can.result, can.man, 10, 8.0);
    verdict_cell("box PADDED    shared_v2v_verdict", pad.result, pad.man, 10, 8.0);
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 4 — SPHERE: poles + seam + a section that wraps the seam, under 20 rotations
///////////////////////////////////////////////////////////////////////////////////////////

struct SphereOut {
    BRep brep;
    int faces = 0;
    SfManifold man;
    bool solid = false;
    double area = 0.0;
    std::string signature;
    int alerts = 0;
    int minted = 0;
};

/// The sphere as ONE face: u in [0,4] with the seam at u = 0 == u = 4, v in [0,2] with a
/// degenerate pole row at each end. Section = a closed curve wrapping the seam once,
/// v = v0 + amp*sin(pi*u/2), delivered as ONE arena edge whose two paves are the SAME vertex.
static void run_sphere(const Xform& rot, double v0, double amp, SphereOut& out) {
    NurbsSurface srf = Primitives::sphere_surface(0, 0, 0, 1.0);
    srf.transform(rot);

    BdsArena ar;
    const int Vs = ar.append_vertex(srf.point_at(0.0, 0.0), SF_CONFUSION);
    const int Vn = ar.append_vertex(srf.point_at(0.0, 2.0), SF_CONFUSION);

    const NurbsCurve meridian = srf.iso_curve(0, 0.0);
    const int Em = ar.append_edge(meridian, Vs, Vn, SF_CONFUSION);
    const int Vc = ar.append_vertex(srf.point_at(0.0, v0), SF_CONFUSION);
    ar.add_pave(Em, v0, Vc);
    ar.update_pave_blocks();
    const std::vector<BdsPB>& mb = ar.pave_blocks(Em);
    if (mb.size() != 2) { out.faces = -1; return; }
    const BdsPB Mlo = mb[0], Mhi = mb[1];

    const Point ps = srf.point_at(0.0, 0.0), pn = srf.point_at(0.0, 2.0);
    const int Eds = ar.append_edge(line3(ps, ps), Vs, Vs, SF_CONFUSION);
    const int Edn = ar.append_edge(line3(pn, pn), Vn, Vn, SF_CONFUSION);

    const int NS = 64;
    std::vector<Point> secuv, sec3d;
    for (int k = 0; k <= NS; ++k) {
        const double u = 4.0 * k / NS;
        const double v = v0 + amp * std::sin(PI * u * 0.5);
        secuv.push_back(Point(u, v, 0));
        sec3d.push_back(srf.point_at(u, v));
    }
    const NurbsCurve secpc = polyline_curve(secuv);
    const int Esec = ar.append_edge(polyline_curve(sec3d), Vc, Vc, SF_CONFUSION);
    const BdsPB Sec = ar.pave_blocks(Esec)[0];
    const BdsPB Ds = ar.pave_blocks(Eds)[0];
    const BdsPB Dn = ar.pave_blocks(Edn)[0];

    // pcurves. Every seam pave block appears TWICE with two DIFFERENT pcurves (u=4 and u=0).
    const NurbsCurve pc_ds = uvline(0, 0, 4, 0);
    const NurbsCurve pc_dn = uvline(0, 2, 4, 2);
    const NurbsCurve pc_mlo_hi = uvline(4, 0, 4, v0);
    const NurbsCurve pc_mlo_lo = uvline(0, 0, 0, v0);
    const NurbsCurve pc_mhi_hi = uvline(4, v0, 4, 2);
    const NurbsCurve pc_mhi_lo = uvline(0, v0, 0, 2);

    std::vector<SfInputEdge> in;
    SfInputEdge e;
    e.pb = Ds;  e.sense = SfSense::Forward;  e.pcurve = &pc_ds;     e.degenerate = true;  in.push_back(e);
    e = SfInputEdge();
    e.pb = Mlo; e.sense = SfSense::Forward;  e.pcurve = &pc_mlo_hi; in.push_back(e);
    e.pb = Mlo; e.sense = SfSense::Reversed; e.pcurve = &pc_mlo_lo; in.push_back(e);
    e.pb = Mhi; e.sense = SfSense::Forward;  e.pcurve = &pc_mhi_hi; in.push_back(e);
    e.pb = Mhi; e.sense = SfSense::Reversed; e.pcurve = &pc_mhi_lo; in.push_back(e);
    e = SfInputEdge();
    e.pb = Dn;  e.sense = SfSense::Reversed; e.pcurve = &pc_dn;     e.degenerate = true;  in.push_back(e);
    e = SfInputEdge();
    e.pb = Sec; e.sense = SfSense::Forward;  e.pcurve = &secpc;     in.push_back(e);
    e.pb = Sec; e.sense = SfSense::Reversed; e.pcurve = &secpc;     in.push_back(e);

    const SfResult res = sf_split_face(ar, srf, in);
    out.faces = (int)res.faces.size();
    out.signature = result_signature(ar, in, res);
    out.alerts = res.report.count(SfAlert::WalkDeadEnd) + res.report.count(SfAlert::EdgeNotConsumed) +
                 res.report.count(SfAlert::HoleUnassigned) + res.report.count(SfAlert::WalkStepCap) +
                 res.report.count(SfAlert::AngleTieUnresolved);
    out.minted = res.report.minted_edges + res.report.minted_vertices;

    BRep b;
    SfEmitter em(b);
    em.set_pole_edges(g_pole_edges);
    em.emit(ar, srf, in, res, false);
    out.man = sf_manifold(b);
    out.solid = b.is_solid();
    MassPropsOptions mo;
    const MassProps mp = brep_massprops(b, mo);
    out.area = mp.area;
    out.brep = b;
}

static void run_sphere_block() {
    SphereOut ref;
    run_sphere(Xform::identity(), 0.7, 0.35, ref);
    cell("sphere base 2_faces_5_edges_0_naked_2_seams_2_poles",
         ref.faces == 2 && ref.man.naked == 0 && ref.man.nonmanifold == 0 &&
             ref.man.degenerate == 2 && ref.man.seam_edges == 2 && ref.solid,
         sfmt("faces=%d edges=%d naked=%d nonman=%d degen=%d seam=%d solid=%d naive_naked=%d",
              ref.faces, ref.man.edges, ref.man.naked, ref.man.nonmanifold, ref.man.degenerate,
              ref.man.seam_edges, (int)ref.solid, ref.man.edges - 3));
    cell("sphere base area_sums_to_4piR2", std::fabs(ref.area - 4.0 * PI) < 1e-6,
         sfmt("area=%.10f  4pi=%.10f", ref.area, 4.0 * PI));

    int ok = 0, bad = 0;
    double worst_area = 0.0;
    std::string first_sig;
    bool sig_stable = true;
    for (int k = 0; k < 20; ++k) {
        const double a = 0.37 * (k + 1), b = 0.91 * (k + 1), c = 1.73 * (k + 1);
        const Xform rot = Xform::rotation_z(c) * Xform::rotation_y(b) * Xform::rotation_x(a);
        SphereOut o;
        run_sphere(rot, 0.55 + 0.03 * k, 0.20 + 0.01 * k, o);
        const bool good = o.faces == 2 && o.man.naked == 0 && o.man.nonmanifold == 0 &&
                          o.man.degenerate == 2 && o.man.seam_edges == 2 && o.solid &&
                          o.alerts == 0 && o.minted == 0 &&
                          std::fabs(o.area - 4.0 * PI) < 1e-5;
        worst_area = std::max(worst_area, std::fabs(o.area - 4.0 * PI));
        good ? ++ok : ++bad;
        if (k == 0) first_sig = o.signature;
        else if (o.signature != first_sig) sig_stable = false;
    }
    cell("sphere 20_arbitrary_rotations_all_2_faces_0_naked", ok == 20 && bad == 0,
         sfmt("ok=%d/20 worst_area_err=%.3e", ok, worst_area));
    cell("sphere topology_invariant_under_rotation", sig_stable);
    verdict_cell("sphere shared_v2v_verdict", ref.brep, ref.man, 2, 4.0 / 3.0 * PI);

    // POLE REPRESENTATION: the two conventions must score IDENTICALLY (see
    // SfEmitter::set_pole_edges). One emits a zero-length pole edge record (OCCT/port_08 S5),
    // the other leaves the singularity edgeless as this kernel's own primitives do.
    {
        SphereOut noedge;
        g_pole_edges = false;
        run_sphere(Xform::identity(), 0.7, 0.35, noedge);
        g_pole_edges = true;
        const session_cpp::v2v::V2Verdict a = session_cpp::v2v::v2_verdict(ref.brep);
        const session_cpp::v2v::V2Verdict b2 = session_cpp::v2v::v2_verdict(noedge.brep);
        cell("sphere pole_edge_and_edgeless_conventions_score_IDENTICALLY",
             a.closed() && b2.closed() && a.naked_real == 0 && b2.naked_real == 0 &&
                 a.faces == b2.faces && std::fabs(a.volume - b2.volume) < 1e-9 &&
                 a.degenerate == 2 && b2.degenerate == 0 &&
                 noedge.brep.m_topology_edges.size() + 2 == ref.brep.m_topology_edges.size(),
             sfmt("with_pole_edges{edges=%d degen=%d naked=%d vol=%.10f} "
                  "edgeless{edges=%d degen=%d naked=%d vol=%.10f}",
                  (int)ref.brep.m_topology_edges.size(), a.degenerate, a.naked_real, a.volume,
                  (int)noedge.brep.m_topology_edges.size(), b2.degenerate, b2.naked_real,
                  b2.volume));
    }
}

/// VERDICT VALIDATION, part 2 — a shape that ACTUALLY carries degenerate pole edges. The
/// kernel's create_sphere models a sphere as one seam edge with no pole rows, so the trap
/// cannot appear on it; the SPLIT sphere above has two zero-length pole edges with ONE trim
/// each, which is exactly the configuration that makes a bare "trims != 2" count read a
/// watertight sphere as open. This is where sf_manifold earns the right to score anything.
static void run_verdict_pole_shape() {
    SphereOut so;
    run_sphere(Xform::identity(), 0.7, 0.35, so);
    const SfManifold m = sf_manifold(so.brep);
    const bool solid = so.brep.is_solid();
    const int naive = naive_naked(so.brep);
    cell("verdict split_sphere_WITH_pole_edges metric==is_solid", m.closed() == solid && solid,
         sfmt("is_solid=%d sf_closed=%d edges=%d naked=%d degen=%d seam=%d naive_naked=%d",
              (int)solid, (int)m.closed(), m.edges, m.naked, m.degenerate, m.seam_edges, naive));
    cell("verdict naive_nt!=2_count_WOULD_HAVE_LIED_on_that_shape",
         naive == 2 && m.naked == 0 && solid,
         sfmt("naive says %d naked; sf_manifold says %d (both pole edges are zero-length)",
              naive, m.naked));
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 5 — CYLINDER: a section circle that CROSSES THE SEAM, with caps so 0 naked is real
///////////////////////////////////////////////////////////////////////////////////////////

static void run_cylinder_block() {
    const double R = 1.0, H = 2.0;
    NurbsSurface lat = Primitives::cylinder_surface(0, 0, 0, R, H);
    const std::pair<double, double> du = lat.domain(0);
    const std::pair<double, double> dv = lat.domain(1);

    BdsArena ar;
    const Point pb0 = lat.point_at(du.first, dv.first);
    const Point pt0 = lat.point_at(du.first, dv.second);
    const int Vb = ar.append_vertex(pb0, SF_CONFUSION);
    const int Vt = ar.append_vertex(pt0, SF_CONFUSION);

    const NurbsCurve seam3 = lat.iso_curve(0, du.first);
    const int Eseam = ar.append_edge(seam3, Vb, Vt, SF_CONFUSION);
    const double vmid = 0.5 * (dv.first + dv.second);
    const int Vc = ar.append_vertex(lat.point_at(du.first, vmid), SF_CONFUSION);
    ar.add_pave(Eseam, vmid, Vc);
    ar.update_pave_blocks();
    const std::vector<BdsPB>& sb = ar.pave_blocks(Eseam);
    const BdsPB Slo = sb[0], Shi = sb[1];

    const NurbsCurve cbot = lat.iso_curve(1, dv.first);
    const NurbsCurve ctop = lat.iso_curve(1, dv.second);
    const int Ebot = ar.append_edge(cbot, Vb, Vb, SF_CONFUSION);
    const int Etop = ar.append_edge(ctop, Vt, Vt, SF_CONFUSION);
    const BdsPB Bot = ar.pave_blocks(Ebot)[0];
    const BdsPB Top = ar.pave_blocks(Etop)[0];

    // The section: a closed curve wrapping the seam once, v = vmid + amp*sin(2*pi*u/period).
    const int NS = 96;
    const double amp = 0.22 * (dv.second - dv.first);
    std::vector<Point> secuv, sec3d;
    for (int k = 0; k <= NS; ++k) {
        const double u = du.first + (du.second - du.first) * k / NS;
        const double v = vmid + amp * std::sin(2.0 * PI * k / NS);
        secuv.push_back(Point(u, v, 0));
        sec3d.push_back(lat.point_at(u, v));
    }
    const NurbsCurve secpc = polyline_curve(secuv);
    const int Esec = ar.append_edge(polyline_curve(sec3d), Vc, Vc, SF_CONFUSION);
    const BdsPB Sec = ar.pave_blocks(Esec)[0];

    const NurbsCurve pc_bot = uvline(du.first, dv.first, du.second, dv.first);
    const NurbsCurve pc_top = uvline(du.first, dv.second, du.second, dv.second);
    const NurbsCurve pc_slo_hi = uvline(du.second, dv.first, du.second, vmid);
    const NurbsCurve pc_slo_lo = uvline(du.first, dv.first, du.first, vmid);
    const NurbsCurve pc_shi_hi = uvline(du.second, vmid, du.second, dv.second);
    const NurbsCurve pc_shi_lo = uvline(du.first, vmid, du.first, dv.second);

    std::vector<SfInputEdge> in;
    SfInputEdge e;
    e.pb = Bot; e.sense = SfSense::Forward;  e.pcurve = &pc_bot;    in.push_back(e);
    e.pb = Slo; e.sense = SfSense::Forward;  e.pcurve = &pc_slo_hi; in.push_back(e);
    e.pb = Slo; e.sense = SfSense::Reversed; e.pcurve = &pc_slo_lo; in.push_back(e);
    e.pb = Shi; e.sense = SfSense::Forward;  e.pcurve = &pc_shi_hi; in.push_back(e);
    e.pb = Shi; e.sense = SfSense::Reversed; e.pcurve = &pc_shi_lo; in.push_back(e);
    e.pb = Top; e.sense = SfSense::Reversed; e.pcurve = &pc_top;    in.push_back(e);
    e.pb = Sec; e.sense = SfSense::Forward;  e.pcurve = &secpc;     in.push_back(e);
    e.pb = Sec; e.sense = SfSense::Reversed; e.pcurve = &secpc;     in.push_back(e);

    const SfResult res = sf_split_face(ar, lat, in);

    BRep b;
    SfEmitter em(b);
    em.emit(ar, lat, in, res, false);

    // Caps, emitted BY HAND against the SAME emitter: they must reference the bottom/top circle
    // edges by pave-block identity, not by any coordinate match.
    const int e_bot = em.edge_of(Bot.get());
    const int e_top = em.edge_of(Top.get());
    for (int side = 0; side < 2; ++side) {
        const double z = side == 0 ? 0.0 : H;
        NurbsSurface cap;
        cap.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        cap.set_cv(0, 0, Point(-R, -R, z));
        cap.set_cv(1, 0, Point(R, -R, z));
        cap.set_cv(1, 1, Point(R, R, z));
        cap.set_cv(0, 1, Point(-R, R, z));
        const NurbsCurve& c3 = side == 0 ? cbot : ctop;
        const std::pair<double, double> dc = c3.domain();
        std::vector<Point> uv;
        for (int k = 0; k <= 96; ++k) {
            const Point q = c3.point_at(dc.first + (dc.second - dc.first) * k / 96.0);
            const std::pair<double, double> p = plane_uv(cap, q);
            uv.push_back(Point(p.first, p.second, 0));
        }
        const NurbsCurve cpc = polyline_curve(uv);
        const int si = b.add_surface(cap);
        const int fi = b.add_face(si, side == 0);
        const int li = b.add_loop(fi, BRepLoopType::Outer);
        b.add_trim(b.add_curve_2d(cpc), side == 0 ? e_bot : e_top, li, side == 0,
                   BRepTrimType::Mated);
    }

    const SfManifold man = sf_manifold(b);
    const bool solid = b.is_solid();
    cell("cylinder seam_crossing_section 2_lateral_faces", (int)res.faces.size() == 2,
         sfmt("faces=%d %s", (int)res.faces.size(), res.report.str().c_str()));
    cell("cylinder with_caps 0_naked_2_seam_edges",
         man.naked == 0 && man.nonmanifold == 0 && man.seam_edges == 2 && solid,
         sfmt("edges=%d naked=%d nonman=%d seam=%d solid=%d naive_naked=%d", man.edges, man.naked,
              man.nonmanifold, man.seam_edges, (int)solid, naive_naked(b)));

    // The seam pave blocks must appear TWICE inside ONE face's wire (two pcurves, one edge) —
    // exactly what no coordinate-based mating can produce.
    int seam_twice = 0;
    for (size_t f = 0; f < res.faces.size(); ++f) {
        std::map<const BdsPaveBlock*, int> cnt;
        for (size_t k = 0; k < res.faces[f].outer.inputs.size(); ++k)
            ++cnt[in[(size_t)res.faces[f].outer.inputs[k]].pb.get()];
        if (cnt[Slo.get()] == 2 || cnt[Shi.get()] == 2) ++seam_twice;
    }
    cell("cylinder seam_edge_appears_twice_in_one_wire", seam_twice == 2,
         sfmt("faces_with_a_doubled_seam=%d", seam_twice));

    // Adjacency of the section across the two lateral faces, by INDEX.
    const SfPaveBlockId sid = sf_pave_block_id(ar, Sec.get());
    std::set<int> secfaces;
    for (size_t f = 0; f < res.faces.size(); ++f)
        for (size_t k = 0; k < res.faces[f].outer.inputs.size(); ++k)
            if (sf_pave_block_id(ar, in[(size_t)res.faces[f].outer.inputs[k]].pb.get()) == sid)
                secfaces.insert((int)f);
    cell("cylinder section_shared_by_2_faces_via_same_pave_block_index",
         sid.valid() && secfaces.size() == 2,
         sfmt("pb_id=(%d,%d) faces=%d", sid.edge, sid.block, (int)secfaces.size()));

    // Area of the two SPLIT faces (faces 0 and 1; faces 2/3 are the hand-built cap scaffolding,
    // whose single closed planar trim the mass-properties integrator does not handle).
    MassPropsOptions mo;
    const MassProps mp = brep_massprops(b, mo);
    const double lat_area =
        (mp.faces.size() >= 2) ? mp.faces[0].area + mp.faces[1].area : 0.0;
    const double want = 2.0 * PI * R * H;
    cell("cylinder split_faces_area_sums_to_2piRh", std::fabs(lat_area - want) < 1e-9,
         sfmt("lower=%.10f upper=%.10f sum=%.10f want=%.10f",
              mp.faces.size() >= 2 ? mp.faces[0].area : 0.0,
              mp.faces.size() >= 2 ? mp.faces[1].area : 0.0, lat_area, want));
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 6 — A FACE WITH A HOLE, cut through the hole and cut clear of it
///////////////////////////////////////////////////////////////////////////////////////////

/// Unit square in the XY plane, surface domain [0,1]^2. `through`: the cut passes through the
/// circular hole; otherwise it misses it.
static void run_hole_case(bool through, int& faces, int& holes, int& sec_shared,
                          std::string& detail) {
    NurbsSurface s = aniso_plane(1.0, 1.0);

    BdsArena ar;
    // square corners, CCW
    const double cx[4] = {0, 1, 1, 0}, cy[4] = {0, 0, 1, 1};
    int cv[4];
    for (int k = 0; k < 4; ++k) cv[k] = ar.append_vertex(Point(cx[k], cy[k], 0), SF_CONFUSION);
    const double ucut = through ? 0.5 : 0.85;
    const int Vb = ar.append_vertex(Point(ucut, 0, 0), SF_CONFUSION);
    const int Vt = ar.append_vertex(Point(ucut, 1, 0), SF_CONFUSION);

    int Ebot = ar.append_edge(line3(Point(0, 0, 0), Point(1, 0, 0)), cv[0], cv[1], SF_CONFUSION);
    int Erht = ar.append_edge(line3(Point(1, 0, 0), Point(1, 1, 0)), cv[1], cv[2], SF_CONFUSION);
    int Etop = ar.append_edge(line3(Point(1, 1, 0), Point(0, 1, 0)), cv[2], cv[3], SF_CONFUSION);
    int Elft = ar.append_edge(line3(Point(0, 1, 0), Point(0, 0, 0)), cv[3], cv[0], SF_CONFUSION);
    ar.add_pave(Ebot, ucut, Vb);
    ar.add_pave(Etop, 1.0 - ucut, Vt);
    ar.update_pave_blocks();

    // hole circle r = 0.2 at (0.5,0.5), its own vertex at the BOTTOM of the circle.
    const double r = 0.2, hx = 0.5, hy = 0.5;
    const int NC = 128;
    std::vector<Point> circ;
    for (int k = 0; k <= NC; ++k) {
        const double a = -0.5 * PI + 2.0 * PI * k / NC;
        circ.push_back(Point(hx + r * std::cos(a), hy + r * std::sin(a), 0));
    }
    const int Vh = ar.append_vertex(circ[0], SF_CONFUSION);
    const int Ecirc = ar.append_edge(polyline_curve(circ), Vh, Vh, SF_CONFUSION);
    int Vh2 = -1;
    if (through) {
        const std::pair<double, double> dcc = ar.edge_range(Ecirc);
        const double tm = 0.5 * (dcc.first + dcc.second);
        Vh2 = ar.append_vertex(ar.edge_curve(Ecirc)->point_at(tm), SF_CONFUSION);
        ar.add_pave(Ecirc, tm, Vh2);
        ar.update_pave_blocks();
    }

    // section edges
    std::vector<int> secs;
    if (through) {
        secs.push_back(ar.append_edge(line3(Point(ucut, 0, 0), ar.vertex_point(Vh)), Vb, Vh,
                                      SF_CONFUSION));
        secs.push_back(ar.append_edge(line3(ar.vertex_point(Vh2), Point(ucut, 1, 0)), Vh2, Vt,
                                      SF_CONFUSION));
    } else {
        secs.push_back(ar.append_edge(line3(Point(ucut, 0, 0), Point(ucut, 1, 0)), Vb, Vt,
                                      SF_CONFUSION));
    }

    std::deque<NurbsCurve> pool;
    std::vector<SfInputEdge> in;
    const int bnd[4] = {Ebot, Erht, Etop, Elft};
    for (int q = 0; q < 4; ++q) {
        const std::vector<BdsPB>& pbs = ar.pave_blocks(bnd[q]);
        for (size_t p = 0; p < pbs.size(); ++p) {
            SfInputEdge e;
            e.pb = pbs[p];
            e.sense = SfSense::Forward;
            pool.push_back(plane_pcurve(s, ar, pbs[p]));
            e.pcurve = &pool.back();
            in.push_back(e);
        }
    }
    // hole boundary: traversed CLOCKWISE, i.e. against the circle's own CCW direction.
    {
        const std::vector<BdsPB>& pbs = ar.pave_blocks(Ecirc);
        for (size_t p = 0; p < pbs.size(); ++p) {
            SfInputEdge e;
            e.pb = pbs[p];
            e.sense = SfSense::Reversed;
            const std::pair<double, double> dc = ar.edge_range(Ecirc);
            (void)dc;
            // pcurve = the circle polyline restricted to this block, in NATURAL direction.
            const NurbsCurve* c3 = ar.edge_curve(Ecirc);
            std::vector<Point> uv;
            // one sample per polyline segment of the 3D curve, so the pcurve is EXACTLY the
            // UV image of the stored edge, independent of how the block was subdivided.
            const std::pair<double, double> er = ar.edge_range(Ecirc);
            const double frac =
                (pbs[p]->pave2.t - pbs[p]->pave1.t) / (er.second - er.first);
            const int nseg = std::max(8, (int)llround(NC * frac));
            for (int k = 0; k <= nseg; ++k) {
                const double t = pbs[p]->pave1.t +
                                 (pbs[p]->pave2.t - pbs[p]->pave1.t) * k / nseg;
                const Point w = c3->point_at(t);
                const std::pair<double, double> pu = plane_uv(s, w);
                uv.push_back(Point(pu.first, pu.second, 0));
            }
            pool.push_back(polyline_curve(uv));
            e.pcurve = &pool.back();
            in.push_back(e);
        }
    }
    for (size_t q = 0; q < secs.size(); ++q) {
        const std::vector<BdsPB>& pbs = ar.pave_blocks(secs[q]);
        for (size_t p = 0; p < pbs.size(); ++p) {
            pool.push_back(plane_pcurve(s, ar, pbs[p]));
            const NurbsCurve* pc = &pool.back();
            SfInputEdge a;
            a.pb = pbs[p]; a.sense = SfSense::Forward;  a.pcurve = pc; in.push_back(a);
            SfInputEdge rv;
            rv.pb = pbs[p]; rv.sense = SfSense::Reversed; rv.pcurve = pc; in.push_back(rv);
        }
    }

    const SfResult res = sf_split_face(ar, s, in);
    faces = (int)res.faces.size();
    holes = 0;
    for (size_t f = 0; f < res.faces.size(); ++f) holes += (int)res.faces[f].holes.size();

    sec_shared = 0;
    for (size_t q = 0; q < secs.size(); ++q) {
        const std::vector<BdsPB>& pbs = ar.pave_blocks(secs[q]);
        for (size_t p = 0; p < pbs.size(); ++p) {
            const SfPaveBlockId id = sf_pave_block_id(ar, pbs[p].get());
            std::set<int> fs;
            for (size_t f = 0; f < res.faces.size(); ++f) {
                const SfFace& A = res.faces[f];
                for (size_t k = 0; k < A.outer.inputs.size(); ++k)
                    if (sf_pave_block_id(ar, in[(size_t)A.outer.inputs[k]].pb.get()) == id)
                        fs.insert((int)f);
                for (size_t hh = 0; hh < A.holes.size(); ++hh)
                    for (size_t k = 0; k < A.holes[hh].inputs.size(); ++k)
                        if (sf_pave_block_id(ar, in[(size_t)A.holes[hh].inputs[k]].pb.get()) == id)
                            fs.insert((int)f);
            }
            if (fs.size() == 2) ++sec_shared;
        }
    }

    double atot = 0.0;
    for (size_t f = 0; f < res.faces.size(); ++f) {
        atot += res.faces[f].outer.area_uv;
        for (size_t hh = 0; hh < res.faces[f].holes.size(); ++hh)
            atot += res.faces[f].holes[hh].area_uv;
    }
    detail = sfmt("faces=%d holes=%d sec_shared=%d uv_area=%.9f %s", faces, holes, sec_shared,
                  atot, res.report.str().c_str());
    // partition check: the total signed UV area must equal the square minus the disk.
    // The hole pcurve IS an NC-gon, so the exact partition target is the polygon's area, not
    // pi*r^2. Sampling the wire polygons at the pcurve knots makes this exact to roundoff.
    const double want = 1.0 - 0.5 * NC * r * r * std::sin(2.0 * PI / NC);
    if (std::fabs(atot - want) > 1e-9) detail += sfmt(" AREA_MISMATCH want=%.9f", want);
}

static void run_hole_block() {
    int f1 = 0, h1 = 0, s1 = 0, f2 = 0, h2 = 0, s2 = 0;
    std::string d1, d2;
    run_hole_case(false, f1, h1, s1, d1);
    cell("hole cut_CLEAR_of_hole 2_faces_1_inner_wire",
         f1 == 2 && h1 == 1 && s1 == 1 && d1.find("AREA_MISMATCH") == std::string::npos, d1);
    run_hole_case(true, f2, h2, s2, d2);
    cell("hole cut_THROUGH_hole 2_faces_0_inner_wires",
         f2 == 2 && h2 == 0 && s2 == 2 && d2.find("AREA_MISMATCH") == std::string::npos, d2);
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 7 — A VALENCE-4 NODE ON A CURVED, STRONGLY ANISOTROPIC SURFACE
///////////////////////////////////////////////////////////////////////////////////////////

/// Parabolic cylinder: quadratic in u (genuinely curved), linear in v, |S_u|/|S_v| = L/W.
static NurbsSurface parabolic_strip(double L, double W, double Hh) {
    NurbsSurface s;
    s.create_raw(3, false, 3, 2, 3, 2, false, false, 1.0, 1.0);
    s.set_cv(0, 0, Point(0, 0, 0));
    s.set_cv(1, 0, Point(L * 0.5, 0, Hh));
    s.set_cv(2, 0, Point(L, 0, 0));
    s.set_cv(0, 1, Point(0, W, 0));
    s.set_cv(1, 1, Point(L * 0.5, W, Hh));
    s.set_cv(2, 1, Point(L, W, 0));
    s.set_domain(0, 0.0, 1.0);
    s.set_domain(1, 0.0, 1.0);
    return s;
}

struct XOut {
    int faces = 0;
    std::string signature;
    std::vector<double> areas;
    double node_min_metric_gap = 0.0;
    double node_min_raw_gap = 0.0;
    int alerts = 0;
};

static void run_x_case(const Xform& rot, double a, XOut& out) {
    const double L = 1e4, W = 1e-2, Hh = 2e3;
    NurbsSurface s = parabolic_strip(L, W, Hh);
    s.transform(rot);

    BdsArena ar;
    // corners
    const int c00 = ar.append_vertex(s.point_at(0, 0), SF_CONFUSION);
    const int c10 = ar.append_vertex(s.point_at(1, 0), SF_CONFUSION);
    const int c11 = ar.append_vertex(s.point_at(1, 1), SF_CONFUSION);
    const int c01 = ar.append_vertex(s.point_at(0, 1), SF_CONFUSION);

    const NurbsCurve isov0 = s.iso_curve(1, 0.0);   // v = 0, curve in u
    const NurbsCurve isov1 = s.iso_curve(1, 1.0);
    const NurbsCurve isou1 = s.iso_curve(0, 1.0);   // u = 1, curve in v
    const NurbsCurve isou0 = s.iso_curve(0, 0.0);
    const int Eb = ar.append_edge(isov0, c00, c10, SF_CONFUSION);
    const int Er = ar.append_edge(isou1, c10, c11, SF_CONFUSION);
    const int Et = ar.append_edge(isov1, c01, c11, SF_CONFUSION);   // natural: u 0 -> 1 at v=1
    const int El = ar.append_edge(isou0, c00, c01, SF_CONFUSION);

    // chord feet
    const double ul = 0.5 - a, ur = 0.5 + a;
    const int fb_l = ar.append_vertex(s.point_at(ul, 0), SF_CONFUSION);
    const int fb_r = ar.append_vertex(s.point_at(ur, 0), SF_CONFUSION);
    const int ft_l = ar.append_vertex(s.point_at(ul, 1), SF_CONFUSION);
    const int ft_r = ar.append_vertex(s.point_at(ur, 1), SF_CONFUSION);
    ar.add_pave(Eb, ul, fb_l);
    ar.add_pave(Eb, ur, fb_r);
    ar.add_pave(Et, ul, ft_l);
    ar.add_pave(Et, ur, ft_r);
    ar.update_pave_blocks();

    const int Vx = ar.append_vertex(s.point_at(0.5, 0.5), SF_CONFUSION);
    // chord 1: (ul,0) -> (ur,1); chord 2: (ur,0) -> (ul,1). Each split at the crossing.
    struct Ch { double u0, v0, u1, v1; int a, b; };
    Ch ch[4] = {
        {ul, 0.0, 0.5, 0.5, fb_l, Vx}, {0.5, 0.5, ur, 1.0, Vx, ft_r},
        {ur, 0.0, 0.5, 0.5, fb_r, Vx}, {0.5, 0.5, ul, 1.0, Vx, ft_l}
    };
    int Ech[4];
    for (int k = 0; k < 4; ++k)
        Ech[k] = ar.append_edge(line3(s.point_at(ch[k].u0, ch[k].v0),
                                      s.point_at(ch[k].u1, ch[k].v1)),
                                ch[k].a, ch[k].b, SF_CONFUSION);

    std::deque<NurbsCurve> pool;
    std::vector<SfInputEdge> in;
    // boundary, CCW: bottom (v=0, u 0->1), right (u=1, v 0->1), top (v=1, u 1->0),
    // left (u=0, v 1->0).
    struct B { int e; bool rev; };
    B bs[4] = {{Eb, false}, {Er, false}, {Et, true}, {El, true}};
    double uvB[4][4] = {{0, 0, 1, 0}, {1, 0, 1, 1}, {0, 1, 1, 1}, {0, 0, 0, 1}};
    for (int q = 0; q < 4; ++q) {
        const std::vector<BdsPB>& pbs = ar.pave_blocks(bs[q].e);
        for (size_t p = 0; p < pbs.size(); ++p) {
            const double t0 = pbs[p]->pave1.t, t1 = pbs[p]->pave2.t;
            const std::pair<double, double> dd = ar.edge_range(bs[q].e);
            const double f0 = (t0 - dd.first) / (dd.second - dd.first);
            const double f1 = (t1 - dd.first) / (dd.second - dd.first);
            const double* g = uvB[q];
            pool.push_back(uvline(g[0] + (g[2] - g[0]) * f0, g[1] + (g[3] - g[1]) * f0,
                                  g[0] + (g[2] - g[0]) * f1, g[1] + (g[3] - g[1]) * f1));
            SfInputEdge e;
            e.pb = pbs[p];
            e.sense = bs[q].rev ? SfSense::Reversed : SfSense::Forward;
            e.pcurve = &pool.back();
            in.push_back(e);
        }
    }
    for (int k = 0; k < 4; ++k) {
        const std::vector<BdsPB>& pbs = ar.pave_blocks(Ech[k]);
        pool.push_back(uvline(ch[k].u0, ch[k].v0, ch[k].u1, ch[k].v1));
        const NurbsCurve* pc = &pool.back();
        SfInputEdge f;
        f.pb = pbs[0]; f.sense = SfSense::Forward;  f.pcurve = pc; in.push_back(f);
        SfInputEdge r;
        r.pb = pbs[0]; r.sense = SfSense::Reversed; r.pcurve = pc; in.push_back(r);
    }

    const SfResult res = sf_split_face(ar, s, in);
    out.faces = (int)res.faces.size();
    out.signature = result_signature(ar, in, res);
    out.alerts = (int)res.report.events.size();
    for (size_t f = 0; f < res.faces.size(); ++f) out.areas.push_back(res.faces[f].outer.area_uv);
    std::sort(out.areas.begin(), out.areas.end());

    // Measure the conditioning of the two candidate keys at the valence-4 node itself.
    double mang[4], rang[4];
    for (int k = 0; k < 4; ++k) {
        const double du = ch[k].u1 - ch[k].u0, dv = ch[k].v1 - ch[k].v0;
        const double sgn = (k % 2 == 0) ? -1.0 : 1.0;   // direction leaving the crossing
        mang[k] = sf_metric_angle(s, 0.5, 0.5, sgn * du, sgn * dv, 0);
        rang[k] = sf_raw_angle(sgn * du, sgn * dv);
    }
    out.node_min_metric_gap = 1e9;
    out.node_min_raw_gap = 1e9;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) {
            double dm = std::fabs(mang[i] - mang[j]);
            dm = std::min(dm, 2 * PI - dm);
            double dr = std::fabs(rang[i] - rang[j]);
            dr = std::min(dr, 2 * PI - dr);
            out.node_min_metric_gap = std::min(out.node_min_metric_gap, dm);
            out.node_min_raw_gap = std::min(out.node_min_raw_gap, dr);
        }
}

static void run_valence4_block() {
    const double L = 1e4, W = 1e-2;
    const double a = W / (2.0 * L);   // chords at ~45 degrees ON THE SURFACE
    XOut ref;
    run_x_case(Xform::identity(), a, ref);

    const double tri = 0.5 * a;
    const double side = (1.0 - a) * 0.5;
    bool areas_ok = ref.areas.size() == 4;
    if (areas_ok) {
        areas_ok = std::fabs(ref.areas[0] - tri) < 1e-14 && std::fabs(ref.areas[1] - tri) < 1e-14 &&
                   std::fabs(ref.areas[2] - side) < 1e-12 && std::fabs(ref.areas[3] - side) < 1e-12;
    }
    cell("valence4 X_on_curved_surface_gives_4_faces", ref.faces == 4 && ref.alerts == 0,
         sfmt("faces=%d alerts=%d", ref.faces, ref.alerts));
    cell("valence4 angular_order_picks_the_correct_wires (exact UV areas)", areas_ok,
         ref.areas.size() == 4
             ? sfmt("areas=%.12e,%.12e,%.9f,%.9f want tri=%.12e side=%.9f", ref.areas[0],
                    ref.areas[1], ref.areas[2], ref.areas[3], tri, side)
             : sfmt("nareas=%d", (int)ref.areas.size()));
    cell("valence4 metric_key_is_well_conditioned_where_raw_UV_is_not",
         ref.node_min_metric_gap > 1.5 && ref.node_min_raw_gap < 1e-4,
         sfmt("min_gap metric=%.6f rad raw=%.3e rad ratio=%.3g", ref.node_min_metric_gap,
              ref.node_min_raw_gap, ref.node_min_metric_gap / ref.node_min_raw_gap));

    int ok = 0;
    bool stable = true;
    for (int k = 0; k < 8; ++k) {
        const Xform rot = Xform::rotation_z(0.61 * (k + 1)) * Xform::rotation_y(1.27 * (k + 1)) *
                          Xform::rotation_x(0.43 * (k + 1));
        XOut o;
        run_x_case(rot, a, o);
        if (o.faces == 4 && o.alerts == 0) ++ok;
        if (o.signature != ref.signature) stable = false;
    }
    cell("valence4 answer_unchanged_under_rotation_of_the_configuration", ok == 8 && stable,
         sfmt("ok=%d/8 signature_stable=%d", ok, (int)stable));
}


///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 7b — SEAM STRADDLE: the measured curved-solid failure
//
// A section that STRADDLES a seam leaves a lune against u = u0 and a lune against u = u1 that
// are the two halves of ONE region in 3D. Walking wires alone gives one face too many and
// leaves the seam pieces between the lunes 1-trim (naked). The composition stage must assemble
// them into ONE face, translating the spliced half by exactly one period in u.
///////////////////////////////////////////////////////////////////////////////////////////

struct StraddleOut {
    int faces = 0;
    int merges = 0;
    int composed_wires = 0;
    double uv_gap = 0.0;        ///< worst UV endpoint gap inside the composed wire
    double uv_area_sum = 0.0;   ///< signed UV areas of all output wires
    double uv_domain = 0.0;
    SfManifold man;
    bool solid = false;
    int alerts = 0;
    int naive = 0;
    BRep brep;
    std::string signature;
};

/// `sphere`: poles at v = v0 / v1 (degenerate rows). Otherwise a cylinder with real boundary
/// circles, capped by hand so 0 naked is a real claim.
static void run_straddle(bool sphere, const Xform& rot, double a, double bb, StraddleOut& out) {
    const double R = 1.0, H = 2.0;
    NurbsSurface srf = sphere ? Primitives::sphere_surface(0, 0, 0, R)
                              : Primitives::cylinder_surface(0, 0, 0, R, H);
    srf.transform(rot);
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const double u0 = du.first, u1 = du.second, v0 = dv.first, v1 = dv.second;
    const double vm = 0.5 * (v0 + v1);
    out.uv_domain = (u1 - u0) * (v1 - v0);

    BdsArena ar;
    const int Vlo = ar.append_vertex(srf.point_at(u0, v0), SF_CONFUSION);
    const int Vhi = ar.append_vertex(srf.point_at(u0, v1), SF_CONFUSION);

    const int Em = ar.append_edge(srf.iso_curve(0, u0), Vlo, Vhi, SF_CONFUSION);
    const int VP = ar.append_vertex(srf.point_at(u0, vm - bb), SF_CONFUSION);
    const int VQ = ar.append_vertex(srf.point_at(u0, vm + bb), SF_CONFUSION);
    ar.add_pave(Em, vm - bb, VP);
    ar.add_pave(Em, vm + bb, VQ);
    ar.update_pave_blocks();
    const std::vector<BdsPB>& mb = ar.pave_blocks(Em);
    if (mb.size() != 3) { out.faces = -1; return; }
    const BdsPB Mlo = mb[0], Mmid = mb[1], Mhi = mb[2];

    const NurbsCurve cbot = sphere ? line3(srf.point_at(u0, v0), srf.point_at(u0, v0))
                                   : srf.iso_curve(1, v0);
    const NurbsCurve ctop = sphere ? line3(srf.point_at(u0, v1), srf.point_at(u0, v1))
                                   : srf.iso_curve(1, v1);
    const int Ebot = ar.append_edge(cbot, Vlo, Vlo, SF_CONFUSION);
    const int Etop = ar.append_edge(ctop, Vhi, Vhi, SF_CONFUSION);
    const BdsPB Bot = ar.pave_blocks(Ebot)[0];
    const BdsPB Top = ar.pave_blocks(Etop)[0];

    const int NS = 64;
    std::vector<Point> ruv, r3, luv, l3;
    for (int k = 0; k <= NS; ++k) {
        const double th = -0.5 * PI + PI * k / NS;              // right half: u >= u0
        const double u = u0 + a * std::cos(th), v = vm + bb * std::sin(th);
        ruv.push_back(Point(u, v, 0));
        r3.push_back(srf.point_at(u, v));
    }
    for (int k = 0; k <= NS; ++k) {
        const double th = 0.5 * PI + PI * k / NS;               // left half: u <= u1
        const double u = u1 + a * std::cos(th), v = vm + bb * std::sin(th);
        luv.push_back(Point(u, v, 0));
        l3.push_back(srf.point_at(u, v));
    }
    const NurbsCurve pcR = polyline_curve(ruv), pcL = polyline_curve(luv);
    const int EarcR = ar.append_edge(polyline_curve(r3), VP, VQ, SF_CONFUSION);
    const int EarcL = ar.append_edge(polyline_curve(l3), VQ, VP, SF_CONFUSION);
    const BdsPB ArcR = ar.pave_blocks(EarcR)[0], ArcL = ar.pave_blocks(EarcL)[0];

    const NurbsCurve pc_bot = uvline(u0, v0, u1, v0);
    const NurbsCurve pc_top = uvline(u0, v1, u1, v1);
    const NurbsCurve pc_mlo_r = uvline(u1, v0, u1, vm - bb);
    const NurbsCurve pc_mlo_l = uvline(u0, v0, u0, vm - bb);
    const NurbsCurve pc_mmid_r = uvline(u1, vm - bb, u1, vm + bb);
    const NurbsCurve pc_mmid_l = uvline(u0, vm - bb, u0, vm + bb);
    const NurbsCurve pc_mhi_r = uvline(u1, vm + bb, u1, v1);
    const NurbsCurve pc_mhi_l = uvline(u0, vm + bb, u0, v1);

    std::vector<SfInputEdge> in;
    SfInputEdge e;
    e.pb = Bot;  e.sense = SfSense::Forward;  e.pcurve = &pc_bot;    e.degenerate = sphere;
    in.push_back(e);
    e = SfInputEdge();
    e.pb = Mlo;  e.sense = SfSense::Forward;  e.pcurve = &pc_mlo_r;  in.push_back(e);
    e.pb = Mlo;  e.sense = SfSense::Reversed; e.pcurve = &pc_mlo_l;  in.push_back(e);
    e.pb = Mmid; e.sense = SfSense::Forward;  e.pcurve = &pc_mmid_r; in.push_back(e);
    e.pb = Mmid; e.sense = SfSense::Reversed; e.pcurve = &pc_mmid_l; in.push_back(e);
    e.pb = Mhi;  e.sense = SfSense::Forward;  e.pcurve = &pc_mhi_r;  in.push_back(e);
    e.pb = Mhi;  e.sense = SfSense::Reversed; e.pcurve = &pc_mhi_l;  in.push_back(e);
    e = SfInputEdge();
    e.pb = Top;  e.sense = SfSense::Reversed; e.pcurve = &pc_top;    e.degenerate = sphere;
    in.push_back(e);
    e = SfInputEdge();
    e.pb = ArcR; e.sense = SfSense::Forward;  e.pcurve = &pcR;       in.push_back(e);
    e.pb = ArcR; e.sense = SfSense::Reversed; e.pcurve = &pcR;       in.push_back(e);
    e.pb = ArcL; e.sense = SfSense::Forward;  e.pcurve = &pcL;       in.push_back(e);
    e.pb = ArcL; e.sense = SfSense::Reversed; e.pcurve = &pcL;       in.push_back(e);

    const SfResult res = sf_split_face(ar, srf, in);
    out.faces = (int)res.faces.size();
    out.merges = res.report.seam_merges;
    out.alerts = res.report.count(SfAlert::WalkDeadEnd) + res.report.count(SfAlert::WalkStepCap) +
                 res.report.count(SfAlert::HoleUnassigned) +
                 res.report.count(SfAlert::AngleTieUnresolved);
    out.signature = result_signature(ar, in, res);

    // UV closure of every output wire, with the composition shifts applied. A composed wire that
    // does not close in UV would mean the period shift was wrong.
    for (size_t f = 0; f < res.faces.size(); ++f) {
        const SfWire& w = res.faces[f].outer;
        out.uv_area_sum += w.area_uv;
        if (w.seam_composed) ++out.composed_wires;
        Point prev(0, 0, 0), first(0, 0, 0);
        for (size_t k = 0; k < w.inputs.size(); ++k) {
            const SfInputEdge& ie = in[(size_t)w.inputs[k]];
            const std::pair<double, double> d = ie.pcurve->domain();
            const bool fw = (ie.sense == SfSense::Forward);
            const double su = w.shift_u.empty() ? 0.0 : w.shift_u[k];
            const double sv = w.shift_v.empty() ? 0.0 : w.shift_v[k];
            const Point p0 = ie.pcurve->point_at(fw ? d.first : d.second);
            const Point p1 = ie.pcurve->point_at(fw ? d.second : d.first);
            const Point s0(p0[0] + su, p0[1] + sv, 0), s1(p1[0] + su, p1[1] + sv, 0);
            if (k == 0) first = s0;
            else out.uv_gap = std::max(out.uv_gap, std::sqrt((s0[0] - prev[0]) * (s0[0] - prev[0]) +
                                                             (s0[1] - prev[1]) * (s0[1] - prev[1])));
            prev = s1;
        }
        out.uv_gap = std::max(out.uv_gap, std::sqrt((first[0] - prev[0]) * (first[0] - prev[0]) +
                                                    (first[1] - prev[1]) * (first[1] - prev[1])));
    }

    BRep b;
    SfEmitter em(b);
    em.emit(ar, srf, in, res, false);
    if (!sphere) {
        const int e_bot = em.edge_of(Bot.get()), e_top = em.edge_of(Top.get());
        for (int side = 0; side < 2; ++side) {
            const double z = side == 0 ? 0.0 : H;
            NurbsSurface cap;
            cap.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
            cap.set_cv(0, 0, Point(-R, -R, z));
            cap.set_cv(1, 0, Point(R, -R, z));
            cap.set_cv(1, 1, Point(R, R, z));
            cap.set_cv(0, 1, Point(-R, R, z));
            const NurbsCurve& c3 = side == 0 ? cbot : ctop;
            const std::pair<double, double> dc = c3.domain();
            std::vector<Point> uv;
            for (int k = 0; k <= 96; ++k) {
                const Point q = c3.point_at(dc.first + (dc.second - dc.first) * k / 96.0);
                const std::pair<double, double> pp = plane_uv(cap, q);
                uv.push_back(Point(pp.first, pp.second, 0));
            }
            const int si = b.add_surface(cap);
            const int fi = b.add_face(si, side == 0);
            const int li = b.add_loop(fi, BRepLoopType::Outer);
            b.add_trim(b.add_curve_2d(polyline_curve(uv)), side == 0 ? e_bot : e_top, li,
                       side == 0, BRepTrimType::Mated);
        }
    }
    out.man = sf_manifold(b);
    out.solid = b.is_solid();
    out.naive = naive_naked(b);
    out.brep = b;
}

static void run_straddle_block() {
    for (int mode = 0; mode < 2; ++mode) {
        const bool sph = (mode == 0);
        const char* nm = sph ? "sphere" : "cylinder";
        StraddleOut o;
        run_straddle(sph, Xform::identity(), sph ? 0.9 : 0.9, sph ? 0.45 : 0.25, o);
        cell(sfmt("straddle %s section_straddling_the_seam_gives_ONE_face", nm),
             o.faces == 2 && o.merges == 1 && o.composed_wires == 1 && o.alerts == 0,
             sfmt("faces=%d seam_merges=%d composed_wires=%d alerts=%d", o.faces, o.merges,
                  o.composed_wires, o.alerts));
        cell(sfmt("straddle %s composed_wire_closes_in_UV_after_period_shift", nm),
             o.uv_gap < 1e-12, sfmt("worst_uv_gap=%.3e", o.uv_gap));
        cell(sfmt("straddle %s UV_areas_partition_the_domain", nm),
             std::fabs(o.uv_area_sum - o.uv_domain) < 1e-9,
             sfmt("sum=%.12f domain=%.12f", o.uv_area_sum, o.uv_domain));
        // The seam piece BETWEEN the two crossings is interior to the composed face and to
        // nothing else, so it is never materialised at all: 6 edges, not 7, and no orphan.
        cell(sfmt("straddle %s 0_naked_2_seam_edges_interior_seam_never_materialised", nm),
             o.man.naked == 0 && o.man.nonmanifold == 0 && o.man.seam_edges == 2 &&
                 o.man.orphan == 0 && o.man.edges == 6 && o.solid,
             sfmt("edges=%d naked=%d nonman=%d degen=%d seam=%d orphan=%d solid=%d "
                  "naive_naked=%d",
                  o.man.edges, o.man.naked, o.man.nonmanifold, o.man.degenerate, o.man.seam_edges,
                  o.man.orphan, (int)o.solid, o.naive));

        int ok = 0;
        bool stable = true;
        for (int k = 0; k < 12; ++k) {
            const Xform rot = Xform::rotation_z(0.83 * (k + 1)) * Xform::rotation_y(1.19 * (k + 1)) *
                              Xform::rotation_x(0.29 * (k + 1));
            StraddleOut r;
            run_straddle(sph, rot, sph ? 0.9 : 0.9, sph ? 0.45 : 0.25, r);
            if (r.faces == 2 && r.merges == 1 && r.man.naked == 0 && r.man.nonmanifold == 0 &&
                r.man.seam_edges == 2 && r.solid && r.uv_gap < 1e-12)
                ++ok;
            if (r.signature != o.signature) stable = false;
        }
        cell(sfmt("straddle %s invariant_under_12_rotations", nm), ok == 12 && stable,
             sfmt("ok=%d/12 signature_stable=%d", ok, (int)stable));
        if (sph) {
            verdict_cell(sfmt("straddle %s shared_v2v_verdict", nm), o.brep, o.man, 2,
                         4.0 / 3.0 * PI);
        } else {
            // The capped cylinder's two CAP faces are hand-built scaffolding: a planar face
            // bounded by ONE closed trim, which brep_massprops integrates over the whole
            // parameter rectangle instead of the disk (path=FullDomain). That breaks the
            // GEOMETRIC half of the shared verdict for a reason that has nothing to do with the
            // splitter, so only the topological half is asserted here -- and the path is
            // REPORTED so the claim stays measured. The splitter's own lateral faces integrate
            // exactly (see "cylinder split_faces_area_sums_to_2piRh").
            const session_cpp::v2v::V2Verdict v = session_cpp::v2v::v2_verdict(o.brep);
            const MassProps mpc = brep_massprops(o.brep, MassPropsOptions());
            int fulldom = 0;
            for (size_t q = 0; q < mpc.faces.size(); ++q)
                if (mpc.faces[q].path == MassPropsPath::FullDomain) ++fulldom;
            cell(sfmt("straddle %s shared_v2v_verdict_TOPOLOGY (caps not integrable)", nm),
                 v.naked_real == 0 && v.nonmanifold == 0 && v.seam_edges == o.man.seam_edges &&
                     v.naked_real == o.man.naked && v.faces == 4 && o.brep.is_solid() &&
                     fulldom == 2,
                 sfmt("v2v{faces=%d naked_real=%d nonman=%d seam=%d closure=%.2e} "
                      "cap_faces_on_FullDomain_path=%d is_solid=%d",
                      v.faces, v.naked_real, v.nonmanifold, v.seam_edges, v.closure_residual,
                      fulldom, (int)o.brep.is_solid()));
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 7c — TORUS: a section that crosses BOTH seams
//
// A torus chart has TWO seams. The (1,1) diagonal section crosses both once, so it arrives as
// two pave blocks and cuts the UV square into THREE regions -- while the true answer is ONE
// face: cutting a torus along a (1,1) curve leaves a connected cylinder. Composition must run
// in BOTH parameters, and the composed wire lives in a window one period tall in v.
///////////////////////////////////////////////////////////////////////////////////////////

struct TorusOut {
    int faces = 0;
    int merges = 0;
    double uv_gap = 0.0;
    double uv_area = 0.0;
    SfManifold man;
    bool solid = false;
    int alerts = 0;
    int naive = 0;
    BRep brep;
    std::string signature;
};

static void run_torus(const Xform& rot, double c, TorusOut& out) {
    NurbsSurface srf = Primitives::torus_surface(0, 0, 0, 2.0, 0.6);
    srf.transform(rot);
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const double u0 = du.first, u1 = du.second, v0 = dv.first, v1 = dv.second;
    const double ub = u1 - c;                      // where the diagonal meets the v-seam

    BdsArena ar;
    const int VC = ar.append_vertex(srf.point_at(u0, v0), SF_CONFUSION);   // the corner
    const int Eu = ar.append_edge(srf.iso_curve(0, u0), VC, VC, SF_CONFUSION);   // u-seam
    const int Ev = ar.append_edge(srf.iso_curve(1, v0), VC, VC, SF_CONFUSION);   // v-seam
    const int VA = ar.append_vertex(srf.point_at(u0, c), SF_CONFUSION);
    const int VB = ar.append_vertex(srf.point_at(ub, v0), SF_CONFUSION);
    ar.add_pave(Eu, c, VA);
    ar.add_pave(Ev, ub, VB);
    ar.update_pave_blocks();
    const std::vector<BdsPB>& ub_pb = ar.pave_blocks(Eu);
    const std::vector<BdsPB>& vb_pb = ar.pave_blocks(Ev);
    if (ub_pb.size() != 2 || vb_pb.size() != 2) { out.faces = -1; return; }
    const BdsPB Sua = ub_pb[0], Sub = ub_pb[1], Sva = vb_pb[0], Svb = vb_pb[1];

    const int NS = 64;
    std::vector<Point> s1uv, s1p, s2uv, s2p;
    for (int k = 0; k <= NS; ++k) {
        const double f = (double)k / NS;
        const double a1u = u0 + (ub - u0) * f, a1v = c + (v1 - c) * f;
        s1uv.push_back(Point(a1u, a1v, 0));
        s1p.push_back(srf.point_at(a1u, a1v));
        const double a2u = ub + (u1 - ub) * f, a2v = v0 + (c - v0) * f;
        s2uv.push_back(Point(a2u, a2v, 0));
        s2p.push_back(srf.point_at(a2u, a2v));
    }
    const NurbsCurve pc1 = polyline_curve(s1uv), pc2 = polyline_curve(s2uv);
    const int E1 = ar.append_edge(polyline_curve(s1p), VA, VB, SF_CONFUSION);
    const int E2 = ar.append_edge(polyline_curve(s2p), VB, VA, SF_CONFUSION);
    const BdsPB Sec1 = ar.pave_blocks(E1)[0], Sec2 = ar.pave_blocks(E2)[0];

    const NurbsCurve pc_sua_r = uvline(u1, v0, u1, c),  pc_sua_l = uvline(u0, v0, u0, c);
    const NurbsCurve pc_sub_r = uvline(u1, c,  u1, v1), pc_sub_l = uvline(u0, c,  u0, v1);
    const NurbsCurve pc_sva_b = uvline(u0, v0, ub, v0), pc_sva_t = uvline(u0, v1, ub, v1);
    const NurbsCurve pc_svb_b = uvline(ub, v0, u1, v0), pc_svb_t = uvline(ub, v1, u1, v1);

    std::vector<SfInputEdge> in;
    SfInputEdge e;
    e.pb = Sva; e.sense = SfSense::Forward;  e.pcurve = &pc_sva_b; in.push_back(e);
    e.pb = Svb; e.sense = SfSense::Forward;  e.pcurve = &pc_svb_b; in.push_back(e);
    e.pb = Sua; e.sense = SfSense::Forward;  e.pcurve = &pc_sua_r; in.push_back(e);
    e.pb = Sub; e.sense = SfSense::Forward;  e.pcurve = &pc_sub_r; in.push_back(e);
    e.pb = Svb; e.sense = SfSense::Reversed; e.pcurve = &pc_svb_t; in.push_back(e);
    e.pb = Sva; e.sense = SfSense::Reversed; e.pcurve = &pc_sva_t; in.push_back(e);
    e.pb = Sub; e.sense = SfSense::Reversed; e.pcurve = &pc_sub_l; in.push_back(e);
    e.pb = Sua; e.sense = SfSense::Reversed; e.pcurve = &pc_sua_l; in.push_back(e);
    e.pb = Sec1; e.sense = SfSense::Forward;  e.pcurve = &pc1; in.push_back(e);
    e.pb = Sec1; e.sense = SfSense::Reversed; e.pcurve = &pc1; in.push_back(e);
    e.pb = Sec2; e.sense = SfSense::Forward;  e.pcurve = &pc2; in.push_back(e);
    e.pb = Sec2; e.sense = SfSense::Reversed; e.pcurve = &pc2; in.push_back(e);

    const SfResult res = sf_split_face(ar, srf, in);
    out.faces = (int)res.faces.size();
    out.merges = res.report.seam_merges;
    out.alerts = res.report.count(SfAlert::WalkDeadEnd) + res.report.count(SfAlert::WalkStepCap) +
                 res.report.count(SfAlert::HoleUnassigned) +
                 res.report.count(SfAlert::AngleTieUnresolved);
    out.signature = result_signature(ar, in, res);
    for (size_t f = 0; f < res.faces.size(); ++f) {
        const SfWire& w = res.faces[f].outer;
        out.uv_area += w.area_uv;
        Point prev(0, 0, 0), first(0, 0, 0);
        for (size_t k = 0; k < w.inputs.size(); ++k) {
            const SfInputEdge& ie = in[(size_t)w.inputs[k]];
            const std::pair<double, double> d = ie.pcurve->domain();
            const bool fw = (ie.sense == SfSense::Forward);
            const double su = w.shift_u.empty() ? 0.0 : w.shift_u[k];
            const double sv = w.shift_v.empty() ? 0.0 : w.shift_v[k];
            const Point p0 = ie.pcurve->point_at(fw ? d.first : d.second);
            const Point p1 = ie.pcurve->point_at(fw ? d.second : d.first);
            const Point a0(p0[0] + su, p0[1] + sv, 0), a1(p1[0] + su, p1[1] + sv, 0);
            if (k == 0) first = a0;
            else out.uv_gap = std::max(out.uv_gap,
                                       std::sqrt((a0[0] - prev[0]) * (a0[0] - prev[0]) +
                                                 (a0[1] - prev[1]) * (a0[1] - prev[1])));
            prev = a1;
        }
        out.uv_gap = std::max(out.uv_gap, std::sqrt((first[0] - prev[0]) * (first[0] - prev[0]) +
                                                    (first[1] - prev[1]) * (first[1] - prev[1])));
    }
    BRep b;
    SfEmitter em(b);
    em.emit(ar, srf, in, res, false);
    out.man = sf_manifold(b);
    out.solid = b.is_solid();
    out.naive = naive_naked(b);
    out.brep = b;
}

static void run_torus_block() {
    TorusOut o;
    run_torus(Xform::identity(), 1.0, o);
    const double dom = 16.0;   // torus chart is [0,4] x [0,4]
    cell("torus (1,1)_section_crossing_BOTH_seams_gives_ONE_face",
         o.faces == 1 && o.merges == 2 && o.alerts == 0,
         sfmt("faces=%d seam_merges=%d alerts=%d", o.faces, o.merges, o.alerts));
    cell("torus composed_wire_closes_in_UV_in_both_parameters", o.uv_gap < 1e-12,
         sfmt("worst_uv_gap=%.3e", o.uv_gap));
    cell("torus composed_UV_area_equals_the_whole_chart",
         std::fabs(o.uv_area - dom) < 1e-9, sfmt("area=%.12f want=%.12f", o.uv_area, dom));
    cell("torus 0_naked_every_edge_2trim",
         o.man.naked == 0 && o.man.nonmanifold == 0 && o.man.edges == 4 && o.solid,
         sfmt("edges=%d naked=%d nonman=%d seam=%d orphan=%d solid=%d naive_naked=%d",
              o.man.edges, o.man.naked, o.man.nonmanifold, o.man.seam_edges, o.man.orphan,
              (int)o.solid, o.naive));
    int ok = 0;
    bool stable = true;
    for (int k = 0; k < 8; ++k) {
        const Xform rot = Xform::rotation_z(0.71 * (k + 1)) * Xform::rotation_y(0.53 * (k + 1)) *
                          Xform::rotation_x(1.31 * (k + 1));
        TorusOut r;
        run_torus(rot, 1.0, r);
        if (r.faces == 1 && r.merges == 2 && r.man.naked == 0 && r.man.nonmanifold == 0 &&
            r.solid && r.uv_gap < 1e-12)
            ++ok;
        if (r.signature != o.signature) stable = false;
    }
    cell("torus invariant_under_8_rotations", ok == 8 && stable,
         sfmt("ok=%d/8 signature_stable=%d", ok, (int)stable));
    verdict_cell("torus shared_v2v_verdict", o.brep, o.man, 1, -1.0);
}

///////////////////////////////////////////////////////////////////////////////////////////
// BLOCK 8 — STRUCTURAL: the defect is unrepresentable, not merely absent
///////////////////////////////////////////////////////////////////////////////////////////

static void run_structural() {
    // The header must contain no sewing / welding / snapping / proximity-lookup API.
    const char* banned[] = {"sew", "weld", "snap", "quantis", "find_near", "nearest",
                            "coincident_edge", "merge_by"};
    std::string h;
    if (FILE* f = std::fopen("src/v2/brep_v2_splitface.h", "rb")) {
        char buf[4096];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) h.append(buf, n);
        std::fclose(f);
    }
    bool clean = !h.empty();
    std::string hit;
    // only inspect DECLARATIONS: strip comment lines first.
    std::string decls;
    size_t i = 0;
    while (i < h.size()) {
        const size_t e = h.find('\n', i);
        const std::string ln = h.substr(i, (e == std::string::npos ? h.size() : e) - i);
        size_t k = ln.find_first_not_of(" \t");
        if (k == std::string::npos || ln.compare(k, 2, "//") != 0) decls += ln + "\n";
        i = (e == std::string::npos) ? h.size() : e + 1;
    }
    for (size_t b = 0; b < sizeof(banned) / sizeof(banned[0]); ++b)
        if (decls.find(banned[b]) != std::string::npos) { clean = false; hit = banned[b]; }
    cell("structural no_sew_weld_snap_or_proximity_lookup_in_the_API", clean,
         hit.empty() ? "header scanned" : ("found: " + hit));

    // sf_pave_block_id is a TABLE READ: an unknown pointer resolves to nothing, it never
    // "finds the nearest".
    BdsArena ar;
    const int v0 = ar.append_vertex(Point(0, 0, 0), SF_CONFUSION);
    const int v1 = ar.append_vertex(Point(1, 0, 0), SF_CONFUSION);
    const int e0 = ar.append_edge(line3(Point(0, 0, 0), Point(1, 0, 0)), v0, v1, SF_CONFUSION);
    BdsArena br;
    const int w0 = br.append_vertex(Point(0, 0, 0), SF_CONFUSION);
    const int w1 = br.append_vertex(Point(1, 0, 0), SF_CONFUSION);
    const int f0 = br.append_edge(line3(Point(0, 0, 0), Point(1, 0, 0)), w0, w1, SF_CONFUSION);
    const SfPaveBlockId a = sf_pave_block_id(ar, ar.pave_blocks(e0)[0].get());
    const SfPaveBlockId cross = sf_pave_block_id(ar, br.pave_blocks(f0)[0].get());
    cell("structural identical_geometry_in_another_arena_is_NOT_the_same_block",
         a.valid() && !cross.valid(),
         sfmt("own=(%d,%d) foreign=(%d,%d)", a.edge, a.block, cross.edge, cross.block));
}

///////////////////////////////////////////////////////////////////////////////////////////

int main() {
    std::printf("=== main_14  FACE SPLITTER (src/brep_v2_splitface) ===\n");
    std::printf("--- 1. VERDICT METRIC VALIDATION (must precede every score) ---\n");
    run_verdict_validation();
    std::printf("--- 2. SURFACE METRIC AND THE ANGULAR SORTING KEY ---\n");
    run_metric();
    std::printf("--- 3. DECISIVE: box split, canonical vs padded UV domain ---\n");
    run_decisive_box();
    std::printf("--- 4. SPHERE: poles + seam + wrapping section, 20 rotations ---\n");
    run_sphere_block();
    run_verdict_pole_shape();
    std::printf("--- 5. CYLINDER: section crossing the seam ---\n");
    run_cylinder_block();
    std::printf("--- 6. FACE WITH A HOLE ---\n");
    run_hole_block();
    std::printf("--- 7. VALENCE-4 NODE ON A CURVED ANISOTROPIC SURFACE ---\n");
    run_valence4_block();
    std::printf("--- 7b. SEAM STRADDLE (the measured curved-solid failure) ---\n");
    run_straddle_block();
    std::printf("--- 7c. TORUS: section crossing BOTH seams ---\n");
    run_torus_block();
    std::printf("--- 8. STRUCTURAL ---\n");
    run_structural();
    std::printf("=== main_14: %d PASS, %d FAIL ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
