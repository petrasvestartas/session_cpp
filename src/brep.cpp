#include "brep.h"
#include "brep_section.h"
#include "brep_samedomain.h"
#include "remesh_nurbssurface_grid.h"
#include "remesh_cdt.h"
#include "primitives.h"
#include "intersection.h"
#include "closest.h"
#include "plane.h"
#include "line.h"
#include "polyline.h"
#include "fmt/core.h"
#include <fstream>
#include <cmath>
#include <map>
#include <set>
#include <array>
#include <functional>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <cstdio>
#include <stdexcept>
#ifdef __linux__
#include <unistd.h>
#endif
#include "brep_massprops.h"
#include "brep.pb.h"

namespace session_cpp {

// ---- Boolean resource guard (AUTO memory balloon, task #9) -----------------------------
// A grazing-config SYMEMIT re-split feeds the UV arrangement two nearly-coincident copies
// of one section polyline; their sampled micro-crossings dice the segment into a sliver
// ladder (measured z37 cut: 265 chain-lift runs of seg 21, 19k+ topology edges, 8.3 GB
// RSS, std::bad_alloc). Bounds: (a) g_segrun_cap = max chain-lift runs per segment per
// operand split; (b) g_mem_cap_mb = RSS ceiling checked at the per-face arrangement and
// every 8th capped run. Tripping either THROWS; the AUTO ladder catches, discards that
// variant and continues (ACIS retry-loop doctrine: a failed escalation is a verdict, not
// a crash). Both caps are 0 (inert, byte-identical pipeline) unless armed by the AUTO
// ladder or explicitly via SESSION_MEM_CAP_MB / SESSION_SEGRUN_CAP.
namespace {
long brep_rss_mb() {
#ifdef __linux__
    std::FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return 0;
    long pages = 0, resident = 0;
    int nr = std::fscanf(f, "%ld %ld", &pages, &resident);
    std::fclose(f);
    if (nr != 2) return 0;
    return resident * (sysconf(_SC_PAGESIZE) / 1024L) / 1024L;
#else
    return 0;
#endif
}
long g_mem_cap_mb = 0;   // 0 = off
int g_segrun_cap = 0;    // 0 = off
}  // anonymous namespace: brep_shell_count has external linkage (declared in brep.h)
// Connected-component (SHELL) count over faces linked by shared topology edges. is_solid()
// only asserts "every edge has 2 trims", which a result that has come APART into several
// separately-closed shells still satisfies -- an independent OCCT read of the result set found
// 2-4 shells on every rotated config except z30x20, invisible to the naked-edge metric. A
// config going 1 -> N shells is a stronger and earlier signal than naked count (doctrine
// Law 6: validity is a pipeline invariant, not a final check).
int brep_shell_count(const BRep& X) {
    int nf = (int)X.m_faces.size();
    if (nf == 0) return 0;
    std::vector<int> par(nf);
    for (int i = 0; i < nf; ++i) par[i] = i;
    std::function<int(int)> find = [&](int a) { return par[a] == a ? a : par[a] = find(par[a]); };
    std::map<int, int> first_face;   // edge index -> a face using it
    for (int fi = 0; fi < nf; ++fi)
        for (int li : X.m_faces[fi].loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int ei = X.m_trims[ti].edge_index;
                if (ei < 0) continue;
                auto it = first_face.find(ei);
                if (it == first_face.end()) first_face[ei] = fi;
                else par[find(it->second)] = find(fi);
            }
        }
    std::set<int> comps;
    for (int fi = 0; fi < nf; ++fi) comps.insert(find(fi));
    return (int)comps.size();
}
namespace {
void brep_seam_audit(const BRep& result, double tolerance) {
        auto trim_pts = [&](int ti, int n, std::vector<Point>& out) {
            const BRepTrim& T = result.m_trims[ti];
            if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)result.m_curves_2d.size()) return false;
            int li = T.loop_index;
            if (li < 0 || li >= (int)result.m_loops.size()) return false;
            int fi = result.m_loops[li].face_index;
            if (fi < 0 || fi >= (int)result.m_faces.size()) return false;
            int si = result.m_faces[fi].surface_index;
            if (si < 0 || si >= (int)result.m_surfaces.size()) return false;
            const NurbsCurve& pc = result.m_curves_2d[T.curve_2d_index];
            const NurbsSurface& S = result.m_surfaces[si];
            auto d = pc.domain();
            out.clear();
            for (int k = 0; k <= n; ++k) {
                Point uv = pc.point_at(d.first + (d.second - d.first) * k / n);
                out.push_back(S.point_at(uv[0], uv[1]));
            }
            return true;
        };
        auto hausdorff = [](const std::vector<Point>& a, const std::vector<Point>& b) {
            double m = 0;
            for (const auto& p : a) {
                double bd = 1e300;
                for (size_t j = 0; j + 1 < b.size(); ++j) {
                    double ex = b[j+1][0]-b[j][0], ey = b[j+1][1]-b[j][1], ez = b[j+1][2]-b[j][2];
                    double L2 = ex*ex + ey*ey + ez*ez;
                    double t = L2 > 1e-30 ? ((p[0]-b[j][0])*ex + (p[1]-b[j][1])*ey + (p[2]-b[j][2])*ez) / L2 : 0.0;
                    t = std::min(1.0, std::max(0.0, t));
                    Point q(b[j][0]+t*ex, b[j][1]+t*ey, b[j][2]+t*ez);
                    bd = std::min(bd, p.distance(q));
                }
                m = std::max(m, bd);
            }
            return m;
        };
        const int NS = 12;
        double band = tolerance > 0 ? tolerance : 1e-6;
        int n_dev = 0, n_share = 0;
        std::vector<std::vector<Point>> esamp(result.m_topology_edges.size());
        std::vector<int> ntrim(result.m_topology_edges.size(), 0);
        for (size_t e = 0; e < result.m_topology_edges.size(); ++e) {
            const auto& E = result.m_topology_edges[e];
            ntrim[e] = (int)E.trim_indices.size();
            if (E.trim_indices.empty()) continue;
            std::vector<Point> a;
            if (trim_pts(E.trim_indices[0], NS, a)) esamp[e] = a;
            if (E.trim_indices.size() == 2) {
                std::vector<Point> b;
                if (!esamp[e].empty() && trim_pts(E.trim_indices[1], NS, b) && !b.empty()) {
                    double dv = std::max(hausdorff(esamp[e], b), hausdorff(b, esamp[e]));
                    if (dv > band * 10) {
                        ++n_dev;
                        const Point& m0 = esamp[e][NS/2];
                        std::fprintf(stderr, "[SEAM-DEV] e=%zu dev=%.6f at (%.4f,%.4f,%.4f)\n",
                                     e, dv, m0[0], m0[1], m0[2]);
                    }
                }
            }
        }
        // sharing: distinct edge records whose sampled curves coincide
        for (size_t e = 0; e < esamp.size(); ++e) {
            if (esamp[e].empty()) continue;
            for (size_t f = e + 1; f < esamp.size(); ++f) {
                if (esamp[f].empty()) continue;
                if (esamp[e][NS/2].distance(esamp[f][NS/2]) > band * 1e3) continue;   // cheap reject
                double dv = std::max(hausdorff(esamp[e], esamp[f]), hausdorff(esamp[f], esamp[e]));
                if (dv <= band * 10) {
                    ++n_share;
                    const Point& m0 = esamp[e][NS/2];
                    std::fprintf(stderr, "[SEAM-SHARE] e=%zu(%dt) == e=%zu(%dt) gap=%.3e at (%.4f,%.4f,%.4f)\n",
                                 e, ntrim[e], f, ntrim[f], dv, m0[0], m0[1], m0[2]);
                }
            }
        }
        std::fprintf(stderr, "[SEAM] diverging 2-trim edges=%d unshared coincident pairs=%d (band=%.2e)\n",
                     n_dev, n_share, band);
        std::fflush(stderr);
}

// (surface_index, seg_id) pairs actually fed as cut pcurves by the LAST split_with scaffold
// pass. Read by boolean()'s SYMEMIT emission to avoid re-imprinting a section the face
// already received (see SYMEMIT-DUP note there). Single-threaded pipeline; reset per split.
std::set<std::pair<int, int>> g_fed_cuts;
void guard_check_mem(const char* where) {
    if (g_mem_cap_mb <= 0) return;
    long r = brep_rss_mb();
    if (r > g_mem_cap_mb)
        throw std::runtime_error(std::string("[MEMCAP] rss ") + std::to_string(r) + "MB > " +
                                 std::to_string(g_mem_cap_mb) + "MB at " + where);
}
}  // namespace

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

BRep::BRep() {}

BRep::BRep(const BRep& other) { deep_copy_from(other); }

BRep& BRep::operator=(const BRep& other) {
    if (this != &other) deep_copy_from(other);
    return *this;
}

bool BRep::operator==(const BRep& other) const {
    if (name != other.name) return false;
    if (width != other.width) return false;
    if (surfacecolor != other.surfacecolor) return false;
    if (m_faces.size() != other.m_faces.size()) return false;
    if (m_surfaces.size() != other.m_surfaces.size()) return false;
    if (m_topology_edges.size() != other.m_topology_edges.size()) return false;
    if (m_vertices.size() != other.m_vertices.size()) return false;
    return true;
}

bool BRep::operator!=(const BRep& other) const { return !(*this == other); }

BRep::~BRep() {}

void BRep::deep_copy_from(const BRep& src) {
    _guid.clear();
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    m_surfaces = src.m_surfaces;
    m_curves_3d = src.m_curves_3d;
    m_curves_2d = src.m_curves_2d;
    m_vertices = src.m_vertices;
    m_topology_vertices = src.m_topology_vertices;
    m_topology_edges = src.m_topology_edges;
    m_trims = src.m_trims;
    m_loops = src.m_loops;
    m_faces = src.m_faces;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Factory
///////////////////////////////////////////////////////////////////////////////////////////

BRep BRep::create_box(double sx, double sy, double sz) {
    BRep brep;
    brep.name = "box";
    double hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;

    // 8 vertices: corners of the box
    Point corners[8] = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    for (int i = 0; i < 8; ++i) brep.add_vertex(corners[i]);

    // 6 faces: bottom(0), top(1), front(2), right(3), back(4), left(5)
    // Each face defined by 4 corner indices (CCW from outside)
    int face_verts[6][4] = {
        {0, 3, 2, 1}, // bottom (z=-hz), normal -Z
        {4, 5, 6, 7}, // top (z=+hz), normal +Z
        {0, 1, 5, 4}, // front (y=-hy), normal -Y
        {1, 2, 6, 5}, // right (x=+hx), normal +X
        {2, 3, 7, 6}, // back (y=+hy), normal +Y
        {3, 0, 4, 7}, // left (x=-hx), normal -X
    };

    // 12 edges: each edge defined by (start_vertex, end_vertex)
    int edge_verts[12][2] = {
        {0,1},{1,2},{2,3},{3,0},  // bottom edges
        {4,5},{5,6},{6,7},{7,4},  // top edges
        {0,4},{1,5},{2,6},{3,7},  // vertical edges
    };

    // Create 3D edge curves (lines)
    for (int i = 0; i < 12; ++i) {
        Point p0 = corners[edge_verts[i][0]];
        Point p1 = corners[edge_verts[i][1]];
        NurbsCurve line = NurbsCurve::create(false, 1, {p0, p1});
        brep.add_curve_3d(line);
    }

    // Create topology vertices
    for (int i = 0; i < 8; ++i) {
        BRepVertex tv;
        tv.point_index = i;
        brep.m_topology_vertices.push_back(tv);
    }

    // Create topology edges
    for (int i = 0; i < 12; ++i)
        brep.add_edge(i, edge_verts[i][0], edge_verts[i][1]);

    // For each face: create bilinear surface, outer loop with 4 trims
    for (int fi = 0; fi < 6; ++fi) {
        int* fv = face_verts[fi];
        Point p00 = corners[fv[0]], p10 = corners[fv[1]];
        Point p01 = corners[fv[3]], p11 = corners[fv[2]];

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, p00); srf.set_cv(1, 0, p10);
        srf.set_cv(0, 1, p01); srf.set_cv(1, 1, p11);
        int si = brep.add_surface(srf);

        int face_idx = brep.add_face(si, false);
        int loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer);

        // 4 trim curves in UV space (unit square boundary)
        Point uv_corners[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        // Edge mapping for each face: which of the 12 edges correspond to each face edge
        // We find edges by matching vertex pairs
        auto find_edge = [&](int v0, int v1) -> int {
            for (int e = 0; e < 12; ++e) {
                if ((edge_verts[e][0] == v0 && edge_verts[e][1] == v1) ||
                    (edge_verts[e][0] == v1 && edge_verts[e][1] == v0))
                    return e;
            }
            return -1;
        };

        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            NurbsCurve trim_crv = NurbsCurve::create(false, 1,
                {uv_corners[ei], uv_corners[next]});
            int c2d_idx = brep.add_curve_2d(trim_crv);
            int edge_idx = find_edge(fv[ei], fv[next]);
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d_idx, edge_idx, loop_idx, rev, BRepTrimType::Mated);
        }
    }

    // Update topology vertex edge indices
    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        auto& e = brep.m_topology_edges[ei];
        brep.m_topology_vertices[e.start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[e.end_vertex].edge_indices.push_back(ei);
    }

    return brep;
}

BRep BRep::create_cylinder(double radius, double height) {
    BRep brep;
    brep.name = "cylinder";
    NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, radius, height);
    auto dom_u = body.domain(0);
    auto dom_v = body.domain(1);

    Point p_bot = body.point_at_corner(0, 0);
    Point p_top = body.point_at_corner(0, 1);
    int vi_bot = brep.add_vertex(p_bot);
    int vi_top = brep.add_vertex(p_top);
    brep.m_topology_vertices.push_back({vi_bot, {}});
    brep.m_topology_vertices.push_back({vi_top, {}});

    NurbsCurve circle_bot = Primitives::circle(0, 0, 0, radius);
    NurbsCurve circle_top = Primitives::circle(0, 0, height, radius);
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {p_bot, p_top});
    int ci_bot = brep.add_curve_3d(circle_bot);
    int ci_top = brep.add_curve_3d(circle_top);
    int ci_seam = brep.add_curve_3d(seam_line);
    int ei_bot = brep.add_edge(ci_bot, 0, 0);
    int ei_top = brep.add_edge(ci_top, 1, 1);
    int ei_seam = brep.add_edge(ci_seam, 0, 1);

    int si_body = brep.add_surface(body);
    NurbsSurface cap_bot; cap_bot.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    cap_bot.set_cv(0, 0, Point(-radius, -radius, 0));
    cap_bot.set_cv(1, 0, Point(radius, -radius, 0));
    cap_bot.set_cv(0, 1, Point(-radius, radius, 0));
    cap_bot.set_cv(1, 1, Point(radius, radius, 0));
    int si_bot = brep.add_surface(cap_bot);
    NurbsSurface cap_top; cap_top.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    cap_top.set_cv(0, 0, Point(-radius, -radius, height));
    cap_top.set_cv(1, 0, Point(radius, -radius, height));
    cap_top.set_cv(0, 1, Point(-radius, radius, height));
    cap_top.set_cv(1, 1, Point(radius, radius, height));
    int si_top = brep.add_surface(cap_top);

    int fi_body = brep.add_face(si_body, false);
    int li_body = brep.add_loop(fi_body, BRepLoopType::Outer);
    auto c2d_bot = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0),
        Point(dom_u.second, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_bot), ei_bot, li_body, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0),
        Point(dom_u.second, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_body, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0),
        Point(dom_u.first, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_top, li_body, true, BRepTrimType::Mated);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0),
        Point(dom_u.first, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_body, true, BRepTrimType::Seam);

    // Circular 2D trim in UV space: circle at (0.5,0.5) radius 0.5
    const double cw = std::sqrt(2.0) / 2.0;
    double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
    double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
    auto make_cap_circle = [&]() {
        NurbsCurve c(3, true, 3, 9);
        for (int i = 0; i < 10; i++) c.set_nurbsknot(i, ckn[i]);
        for (int i = 0; i < 9; i++)
            c.set_cv_4d(i, (0.5+0.5*ccx[i])*cwt[i], (0.5+0.5*ccy[i])*cwt[i], 0.0, cwt[i]);
        return c;
    };

    int fi_bot = brep.add_face(si_bot, true);
    int li_bot = brep.add_loop(fi_bot, BRepLoopType::Outer);
    brep.add_trim(brep.add_curve_2d(make_cap_circle()), ei_bot, li_bot, true, BRepTrimType::Mated);

    int fi_top = brep.add_face(si_top, false);
    int li_top = brep.add_loop(fi_top, BRepLoopType::Outer);
    brep.add_trim(brep.add_curve_2d(make_cap_circle()), ei_top, li_top, false, BRepTrimType::Mated);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_sphere(double radius) {
    BRep brep;
    brep.name = "sphere";
    NurbsSurface srf = Primitives::sphere_surface(0, 0, 0, radius);
    auto dom_u = srf.domain(0);
    auto dom_v = srf.domain(1);

    Point p_south(0, 0, -radius);
    Point p_north(0, 0, radius);
    int vi_south = brep.add_vertex(p_south);
    int vi_north = brep.add_vertex(p_north);
    brep.m_topology_vertices.push_back({vi_south, {}});
    brep.m_topology_vertices.push_back({vi_north, {}});

    // The seam's 3D curve must be the ON-SURFACE meridian (u = u0 iso), NOT the polar chord
    // through the centre: exporters hand this curve to CAD importers as the seam edge's
    // geometry, and a chord that cuts through the solid crashes strict consumers (Rhino on
    // the perturbed-sphere blob). Sampled like create_torus's iso_curve.
    std::vector<Point> seam_pts;
    {
        int n = 64;
        for (int i = 0; i <= n; ++i) {
            double t = (double)i / n;
            seam_pts.push_back(srf.point_at(dom_u.first, dom_v.first + (dom_v.second-dom_v.first)*t));
        }
    }
    NurbsCurve seam_crv = NurbsCurve::create(false, 3, seam_pts);
    int ci_seam = brep.add_curve_3d(seam_crv);
    int ei_seam = brep.add_edge(ci_seam, 0, 1);

    // POLE EDGES (SESSION_POLE_EDGE), OCCT convention. OCCT's input sphere carries THREE edges:
    // two DEGENERATED edges spanning the full u period, one per pole, plus the seam meridian --
    // a pole edge appears ONCE in the wire, the seam twice. Our results already agree with that
    // (degenerate-edge counts match res_degen exactly at roty0/24/30) but our PRIMITIVE gave the
    // pole trims edge_index = -1, so a consumer walking a wire saw a different topology for a
    // primitive than for a boolean result built from it. The degenerate curve is the v=const
    // iso, so it carries the surface's own u knot vector and hence OCCT's [u0,u1] range.
    // DEFAULT ON (SESSION_NO_POLE_EDGE opts out). Guards battery run with it on and every
    // measure unchanged: primitive matrix 60 cells 0 changed, edge grid 54/54, chairs
    // 35/46.7943 + 25/33.5025 + 50/127.0913, C++ minitests 760/760, rotated-primitive sweep
    // 95 cells 0 changed, SPHCYL sweep byte-identical. It only adds the two edge records.
    static const bool s_pole_edge = (std::getenv("SESSION_NO_POLE_EDGE") == nullptr);
    int ei_pole_s = -1, ei_pole_n = -1;
    if (s_pole_edge) {
        NurbsCurve deg_s = srf.iso_curve(0, dom_v.first);
        NurbsCurve deg_n = srf.iso_curve(0, dom_v.second);
        if (deg_s.is_valid()) ei_pole_s = brep.add_edge(brep.add_curve_3d(deg_s), 0, 0);
        if (deg_n.is_valid()) ei_pole_n = brep.add_edge(brep.add_curve_3d(deg_n), 1, 1);
    }

    int si = brep.add_surface(srf);
    int fi = brep.add_face(si, false);
    int li = brep.add_loop(fi, BRepLoopType::Outer);

    auto c2d_south = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0),
        Point(dom_u.second, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_south), ei_pole_s, li, false, BRepTrimType::Singular);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0),
        Point(dom_u.second, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li, false, BRepTrimType::Seam);
    auto c2d_north = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0),
        Point(dom_u.first, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_north), ei_pole_n, li, false, BRepTrimType::Singular);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0),
        Point(dom_u.first, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li, true, BRepTrimType::Seam);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_cone(double radius, double height) {
    // Side face = cone_surface (u in [0,4] = circle, v in [0,1] = base->apex; v=1 is a SINGULAR
    // apex pole, like a sphere pole) + one planar base cap. Mirrors create_cylinder's base+seam but
    // with a singular apex instead of a top cap.
    BRep brep;
    brep.name = "cone";
    NurbsSurface body = Primitives::cone_surface(0, 0, 0, radius, height);
    auto dom_u = body.domain(0);
    auto dom_v = body.domain(1);

    Point p_base = body.point_at_corner(0, 0);   // on base circle at u=0
    Point p_apex(0, 0, height);
    int vi_base = brep.add_vertex(p_base);
    int vi_apex = brep.add_vertex(p_apex);
    brep.m_topology_vertices.push_back({vi_base, {}});
    brep.m_topology_vertices.push_back({vi_apex, {}});

    NurbsCurve circle_base = Primitives::circle(0, 0, 0, radius);
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {p_base, p_apex});
    int ci_base = brep.add_curve_3d(circle_base);
    int ci_seam = brep.add_curve_3d(seam_line);
    int ei_base = brep.add_edge(ci_base, vi_base, vi_base);
    int ei_seam = brep.add_edge(ci_seam, vi_base, vi_apex);
    // Apex pole edge, same OCCT convention as the sphere's two poles (see create_sphere).
    // The cone has ONE singular boundary (v=v1, the apex); v=v0 is a genuine base circle.
    static const bool s_pole_edge_c = (std::getenv("SESSION_NO_POLE_EDGE") == nullptr);
    int ei_pole_a = -1;
    if (s_pole_edge_c) {
        NurbsCurve deg_a = body.iso_curve(0, dom_v.second);
        if (deg_a.is_valid()) ei_pole_a = brep.add_edge(brep.add_curve_3d(deg_a), vi_apex, vi_apex);
    }

    int si_body = brep.add_surface(body);
    NurbsSurface cap_base; cap_base.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    cap_base.set_cv(0, 0, Point(-radius, -radius, 0));
    cap_base.set_cv(1, 0, Point(radius, -radius, 0));
    cap_base.set_cv(0, 1, Point(-radius, radius, 0));
    cap_base.set_cv(1, 1, Point(radius, radius, 0));
    int si_base = brep.add_surface(cap_base);

    int fi_body = brep.add_face(si_body, false);
    int li_body = brep.add_loop(fi_body, BRepLoopType::Outer);
    auto c2d_base = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_base), ei_base, li_body, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_body, false, BRepTrimType::Seam);
    auto c2d_apex = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_apex), ei_pole_a, li_body, false, BRepTrimType::Singular);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_body, true, BRepTrimType::Seam);

    const double cw = std::sqrt(2.0) / 2.0;
    double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
    double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
    auto make_cap_circle = [&]() {
        NurbsCurve c(3, true, 3, 9);
        for (int i = 0; i < 10; i++) c.set_nurbsknot(i, ckn[i]);
        for (int i = 0; i < 9; i++)
            c.set_cv_4d(i, (0.5+0.5*ccx[i])*cwt[i], (0.5+0.5*ccy[i])*cwt[i], 0.0, cwt[i]);
        return c;
    };
    int fi_base = brep.add_face(si_base, true);
    int li_base = brep.add_loop(fi_base, BRepLoopType::Outer);
    brep.add_trim(brep.add_curve_2d(make_cap_circle()), ei_base, li_base, true, BRepTrimType::Mated);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_pyramid(double base, double height) {
    // Square pyramid: base edge `base` centered at the origin in the z=0 plane, apex at
    // (0,0,height). 5 planar faces: 1 square base + 4 triangles. The apex row of each
    // triangular bilinear patch is a DEGENERATE trim (same convention as the cone apex /
    // sphere pole): a zero-length Singular edge, excluded from the manifold count.
    BRep brep;
    brep.name = "pyramid";
    const double h = base * 0.5;

    Point corners[4] = {
        Point(-h, -h, 0.0),
        Point( h, -h, 0.0),
        Point( h,  h, 0.0),
        Point(-h,  h, 0.0),
    };
    Point apex(0.0, 0.0, height);
    for (int i = 0; i < 4; ++i) brep.add_vertex(corners[i]);
    brep.add_vertex(apex);   // vertex 4

    // 8 real edges: 4 base + 4 slant (slant stored corner -> apex)
    int edge_verts[8][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {0,4},{1,4},{2,4},{3,4},
    };
    for (int i = 0; i < 8; ++i) {
        Point p0 = i < 4 ? corners[edge_verts[i][0]] : corners[edge_verts[i][0]];
        Point p1 = (edge_verts[i][1] == 4) ? apex : corners[edge_verts[i][1]];
        NurbsCurve line = NurbsCurve::create(false, 1, {p0, p1});
        brep.add_curve_3d(line);
    }

    for (int i = 0; i < 5; ++i) {
        BRepVertex tv;
        tv.point_index = i;
        brep.m_topology_vertices.push_back(tv);
    }
    for (int i = 0; i < 8; ++i)
        brep.add_edge(i, edge_verts[i][0], edge_verts[i][1]);

    // Base quad: CCW from below (normal -Z), same winding as create_box's bottom face.
    {
        int fv[4] = {0, 3, 2, 1};
        Point p00 = corners[fv[0]], p10 = corners[fv[1]];
        Point p01 = corners[fv[3]], p11 = corners[fv[2]];
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, p00); srf.set_cv(1, 0, p10);
        srf.set_cv(0, 1, p01); srf.set_cv(1, 1, p11);
        int si = brep.add_surface(srf);
        int face_idx = brep.add_face(si, false);
        int loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer);
        Point uv_corners[4] = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
        int base_edges[4] = {3, 2, 1, 0};   // (0->3)=e3, (3->2)=e2, (2->1)=e1, (1->0)=e0
        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            NurbsCurve trim_crv = NurbsCurve::create(false, 1, {uv_corners[ei], uv_corners[next]});
            int c2d_idx = brep.add_curve_2d(trim_crv);
            int edge_idx = base_edges[ei];
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d_idx, edge_idx, loop_idx, rev, BRepTrimType::Mated);
        }
    }

    // 4 triangular side faces: (corner_i, corner_{i+1}, apex), CCW from outside.
    for (int fi = 0; fi < 4; ++fi) {
        int a = fi, b = (fi + 1) % 4;
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, corners[a]); srf.set_cv(1, 0, corners[b]);
        srf.set_cv(0, 1, apex);       srf.set_cv(1, 1, apex);   // degenerate v=1 row
        int si = brep.add_surface(srf);
        int face_idx = brep.add_face(si, false);
        int loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer);

        // Degenerate apex edge for this face (zero 3D extent, apex vertex used twice).
        NurbsCurve deg = NurbsCurve::create(false, 1, {apex, apex});
        int ei_deg = brep.add_edge(brep.add_curve_3d(deg), 4, 4);

        // UV loop: (0,0)->(1,0) base a->b; (1,0)->(1,1) slant b->apex;
        //          (1,1)->(0,1) apex row (degenerate); (0,1)->(0,0) apex->a.
        brep.add_trim(brep.add_curve_2d(NurbsCurve::create(false, 1, {Point(0,0,0), Point(1,0,0)})),
                      fi, loop_idx, edge_verts[fi][0] != a, BRepTrimType::Mated);
        brep.add_trim(brep.add_curve_2d(NurbsCurve::create(false, 1, {Point(1,0,0), Point(1,1,0)})),
                      4 + b, loop_idx, false, BRepTrimType::Mated);
        brep.add_trim(brep.add_curve_2d(NurbsCurve::create(false, 1, {Point(1,1,0), Point(0,1,0)})),
                      ei_deg, loop_idx, false, BRepTrimType::Singular);
        brep.add_trim(brep.add_curve_2d(NurbsCurve::create(false, 1, {Point(0,1,0), Point(0,0,0)})),
                      4 + a, loop_idx, true, BRepTrimType::Mated);
    }

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_torus(double major_radius, double minor_radius) {
    // Torus: a single closed face, periodic in BOTH u (major circle) and v (minor circle). No caps,
    // no poles -- two seams: the u-seam (minor circle at u=0) and the v-seam (outer major circle at
    // v=0), meeting at one corner vertex. The loop is the UV rectangle [0,4]x[0,4] = 4 Seam trims.
    BRep brep;
    brep.name = "torus";
    NurbsSurface body = Primitives::torus_surface(0, 0, 0, major_radius, minor_radius);
    auto dom_u = body.domain(0);
    auto dom_v = body.domain(1);

    Point p_corner = body.point_at_corner(0, 0);   // (major+minor, 0, 0)
    int vi = brep.add_vertex(p_corner);
    brep.m_topology_vertices.push_back({vi, {}});

    // u-seam: v -> point_at(u0, v), the minor circle at u=0. v-seam: u -> point_at(u, v0), outer
    // major circle at v=0. Sample each as a closed polyline (the exact rational pcurve is a circle).
    auto iso_curve = [&](bool along_v) {
        std::vector<Point> pts; int n = 64;
        for (int i = 0; i <= n; ++i) {
            double t = (double)i / n;
            Point p = along_v ? body.point_at(dom_u.first, dom_v.first + (dom_v.second-dom_v.first)*t)
                              : body.point_at(dom_u.first + (dom_u.second-dom_u.first)*t, dom_v.first);
            pts.push_back(p);
        }
        return NurbsCurve::create(false, 3, pts);
    };
    int ci_useam = brep.add_curve_3d(iso_curve(true));    // minor circle (varies v)
    int ci_vseam = brep.add_curve_3d(iso_curve(false));   // major circle (varies u)
    int ei_useam = brep.add_edge(ci_useam, vi, vi);
    int ei_vseam = brep.add_edge(ci_vseam, vi, vi);

    int si_body = brep.add_surface(body);
    int fi_body = brep.add_face(si_body, false);
    int li_body = brep.add_loop(fi_body, BRepLoopType::Outer);
    // Loop around the UV rectangle: bottom(v-seam), right(u-seam), top(v-seam), left(u-seam).
    auto c2d_bottom = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_bottom), ei_vseam, li_body, false, BRepTrimType::Seam);
    auto c2d_right = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_right), ei_useam, li_body, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_vseam, li_body, true, BRepTrimType::Seam);
    auto c2d_left = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_left), ei_useam, li_body, true, BRepTrimType::Seam);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_block_with_hole(double sx, double sy, double sz, double hole_radius) {
    BRep brep;
    brep.name = "block_with_hole";
    double hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;

    // 8 box corners
    Point corners[8] = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    for (int i = 0; i < 8; ++i) brep.add_vertex(corners[i]);

    // Topology vertices for box corners
    for (int i = 0; i < 8; ++i) {
        BRepVertex tv; tv.point_index = i;
        brep.m_topology_vertices.push_back(tv);
    }

    // 12 box edges
    int edge_verts[12][2] = {
        {0,1},{1,2},{2,3},{3,0},  // bottom
        {4,5},{5,6},{6,7},{7,4},  // top
        {0,4},{1,5},{2,6},{3,7},  // vertical
    };
    for (int i = 0; i < 12; ++i) {
        auto line = NurbsCurve::create(false, 1, {corners[edge_verts[i][0]], corners[edge_verts[i][1]]});
        brep.add_curve_3d(line);
    }
    for (int i = 0; i < 12; ++i)
        brep.add_edge(i, edge_verts[i][0], edge_verts[i][1]);

    // 4 side walls (full-boundary, no hole interaction)
    int side_faces[4][4] = {
        {0, 1, 5, 4}, // front
        {1, 2, 6, 5}, // right
        {2, 3, 7, 6}, // back
        {3, 0, 4, 7}, // left
    };
    auto find_edge = [&](int v0, int v1) -> int {
        for (int e = 0; e < 12; ++e) {
            if ((edge_verts[e][0] == v0 && edge_verts[e][1] == v1) ||
                (edge_verts[e][0] == v1 && edge_verts[e][1] == v0))
                return e;
        }
        return -1;
    };
    for (int fi = 0; fi < 4; ++fi) {
        int* fv = side_faces[fi];
        Point p00 = corners[fv[0]], p10 = corners[fv[1]];
        Point p01 = corners[fv[3]], p11 = corners[fv[2]];
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, p00); srf.set_cv(1, 0, p10);
        srf.set_cv(0, 1, p01); srf.set_cv(1, 1, p11);
        int si = brep.add_surface(srf);
        int face_idx = brep.add_face(si, false);
        int loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer);
        Point uv[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            auto tc = NurbsCurve::create(false, 1, {uv[ei], uv[next]});
            int c2d = brep.add_curve_2d(tc);
            int edge_idx = find_edge(fv[ei], fv[next]);
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d, edge_idx, loop_idx, rev, BRepTrimType::Mated);
        }
    }

    // Cylinder hole surface (through-hole in Z at center)
    NurbsSurface cyl_srf = Primitives::cylinder_surface(0, 0, -hz, hole_radius, sz);
    auto dom_u = cyl_srf.domain(0);
    auto dom_v = cyl_srf.domain(1);
    int si_cyl = brep.add_surface(cyl_srf);
    int fi_cyl = brep.add_face(si_cyl, true); // reversed so normals point inward
    int li_cyl = brep.add_loop(fi_cyl, BRepLoopType::Outer);

    // Hole 3D edge curves (circles at bottom and top)
    NurbsCurve circle_bot = Primitives::circle(0, 0, -hz, hole_radius);
    NurbsCurve circle_top = Primitives::circle(0, 0, hz, hole_radius);
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {
        Point(hole_radius, 0, -hz),
        Point(hole_radius, 0, hz),
    });
    int ci_bot = brep.add_curve_3d(circle_bot);
    int ci_top = brep.add_curve_3d(circle_top);
    int ci_seam = brep.add_curve_3d(seam_line);

    // Add vertices for seam points
    int vi_seam_bot = brep.add_vertex(Point(hole_radius, 0, -hz));
    int vi_seam_top = brep.add_vertex(Point(hole_radius, 0, hz));
    brep.m_topology_vertices.push_back({vi_seam_bot, {}});
    brep.m_topology_vertices.push_back({vi_seam_top, {}});

    int ei_bot = brep.add_edge(ci_bot, 8, 8);   // closed edge
    int ei_top = brep.add_edge(ci_top, 9, 9);
    int ei_seam = brep.add_edge(ci_seam, 8, 9);

    // 4 trims for cylinder body (same pattern as create_cylinder)
    auto c2d_bot = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0),
        Point(dom_u.second, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_bot), ei_bot, li_cyl, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0),
        Point(dom_u.second, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_cyl, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0),
        Point(dom_u.first, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_top, li_cyl, true, BRepTrimType::Mated);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0),
        Point(dom_u.first, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_cyl, true, BRepTrimType::Seam);

    // Bottom and top faces: planar with circular hole
    auto make_cap = [&](double z, bool reversed, int circle_edge_idx) {
        double r = std::max(hx, hy);
        NurbsSurface cap;
        cap.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        cap.set_cv(0, 0, Point(-r, -r, z)); cap.set_cv(1, 0, Point(r, -r, z));
        cap.set_cv(0, 1, Point(-r, r, z));   cap.set_cv(1, 1, Point(r, r, z));
        int si = brep.add_surface(cap);
        int fi = brep.add_face(si, reversed);

        // Outer loop: 4 box edges matching this cap
        int outer_li = brep.add_loop(fi, BRepLoopType::Outer);
        int* fv = (z < 0) ? new int[4]{0, 3, 2, 1} : new int[4]{4, 5, 6, 7};
        // Map box corners to UV on this cap surface
        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            double u0 = (corners[fv[ei]][0] + r) / (2.0 * r);
            double v0 = (corners[fv[ei]][1] + r) / (2.0 * r);
            double u1 = (corners[fv[next]][0] + r) / (2.0 * r);
            double v1 = (corners[fv[next]][1] + r) / (2.0 * r);
            auto tc = NurbsCurve::create(false, 1, {
                Point(u0, v0, 0),
                Point(u1, v1, 0),
            });
            int c2d = brep.add_curve_2d(tc);
            int edge_idx = find_edge(fv[ei], fv[next]);
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d, edge_idx, outer_li, rev, BRepTrimType::Mated);
        }
        delete[] fv;

        // Inner loop: circular hole trim in UV
        int inner_li = brep.add_loop(fi, BRepLoopType::Inner);
        const double cw = std::sqrt(2.0) / 2.0;
        double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
        double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
        NurbsCurve hole_crv(3, true, 3, 9);
        for (int i = 0; i < 10; ++i) hole_crv.set_nurbsknot(i, ckn[i]);
        double cr = hole_radius / (2.0 * r); // radius in UV
        double cx_uv = 0.5, cy_uv = 0.5;     // center in UV
        for (int i = 0; i < 9; ++i)
            hole_crv.set_cv_4d(i, (cx_uv + cr*ccx[i])*cwt[i], (cy_uv + cr*ccy[i])*cwt[i], 0.0, cwt[i]);
        brep.add_trim(brep.add_curve_2d(hole_crv), circle_edge_idx, inner_li,
                       reversed, BRepTrimType::Mated);
    };

    make_cap(-hz, true, ei_bot);
    make_cap(hz, false, ei_top);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::from_polylines(const std::vector<Polyline>& polylines) {
    BRep brep;
    brep.name = "polysurface";
    double tol = 1e-6;

    auto find_or_add = [&](const Point& p) -> int {
        for (int i = 0; i < (int)brep.m_vertices.size(); ++i) {
            double dx = p[0] - brep.m_vertices[i][0];
            double dy = p[1] - brep.m_vertices[i][1];
            double dz = p[2] - brep.m_vertices[i][2];
            if (dx*dx + dy*dy + dz*dz < tol*tol) return i;
        }
        int idx = brep.add_vertex(p);
        brep.m_topology_vertices.push_back({idx, {}});
        return idx;
    };

    std::vector<std::vector<int>> poly_vi;
    for (const auto& pl : polylines) {
        auto pts = pl.get_points();
        int n = pl.is_closed() ? (int)pts.size() - 1 : (int)pts.size();
        std::vector<int> vi;
        for (int i = 0; i < n; ++i) vi.push_back(find_or_add(pts[i]));
        poly_vi.push_back(vi);
    }

    std::map<std::pair<int,int>, int> edge_map;
    auto get_edge = [&](int v0, int v1) -> std::pair<int, bool> {
        int lo = std::min(v0, v1), hi = std::max(v0, v1);
        auto it = edge_map.find({lo, hi});
        if (it != edge_map.end()) return {it->second, v0 != lo};
        auto line = NurbsCurve::create(false, 1, {brep.m_vertices[v0], brep.m_vertices[v1]});
        int ci = brep.add_curve_3d(line);
        int ei = brep.add_edge(ci, lo, hi);
        edge_map[{lo, hi}] = ei;
        return {ei, v0 != lo};
    };

    for (int pi = 0; pi < (int)polylines.size(); ++pi) {
        const auto& vi = poly_vi[pi];
        int n = (int)vi.size();
        if (n < 3) continue;

        Point org; Plane plane;
        polylines[pi].get_fast_plane(org, plane);
        if (!plane.is_valid()) continue;
        Vector xa = plane.x_axis(), ya = plane.y_axis();

        std::vector<double> us(n), vs(n);
        double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
        for (int i = 0; i < n; ++i) {
            double dx = brep.m_vertices[vi[i]][0] - org[0];
            double dy = brep.m_vertices[vi[i]][1] - org[1];
            double dz = brep.m_vertices[vi[i]][2] - org[2];
            us[i] = dx*xa[0] + dy*xa[1] + dz*xa[2];
            vs[i] = dx*ya[0] + dy*ya[1] + dz*ya[2];
            umin = std::min(umin, us[i]); umax = std::max(umax, us[i]);
            vmin = std::min(vmin, vs[i]); vmax = std::max(vmax, vs[i]);
        }
        double pad = std::max(umax - umin, vmax - vmin) * 0.01;
        umin -= pad; umax += pad; vmin -= pad; vmax += pad;
        double du = umax - umin, dv = vmax - vmin;

        auto pt3d = [&](double u, double v) {
            return Point(org[0] + u*xa[0] + v*ya[0],
                         org[1] + u*xa[1] + v*ya[1],
                         org[2] + u*xa[2] + v*ya[2]);
        };
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, pt3d(umin, vmin)); srf.set_cv(1, 0, pt3d(umax, vmin));
        srf.set_cv(0, 1, pt3d(umin, vmax)); srf.set_cv(1, 1, pt3d(umax, vmax));
        int si = brep.add_surface(srf);

        int fi = brep.add_face(si, false);
        int li = brep.add_loop(fi, BRepLoopType::Outer);

        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            double u0 = (us[i] - umin) / du, v0 = (vs[i] - vmin) / dv;
            double u1 = (us[j] - umin) / du, v1 = (vs[j] - vmin) / dv;
            auto tc = NurbsCurve::create(false, 1, {
                Point(u0, v0, 0),
                Point(u1, v1, 0),
            });
            int c2d = brep.add_curve_2d(tc);
            auto [ei, rev] = get_edge(vi[i], vi[j]);
            BRepTrimType type = brep.m_topology_edges[ei].trim_indices.empty()
                ? BRepTrimType::Boundary : BRepTrimType::Mated;
            for (int ti : brep.m_topology_edges[ei].trim_indices)
                brep.m_trims[ti].type = BRepTrimType::Mated;
            brep.add_trim(c2d, ei, li, rev, type);
        }
    }

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::from_nurbscurves(const std::vector<NurbsCurve>& curves, const std::vector<std::vector<NurbsCurve>>& holes) {
    BRep brep;
    brep.name = "polysurface";
    double tol = 1e-6;

    auto find_or_add = [&](const Point& p) -> int {
        for (int i = 0; i < (int)brep.m_vertices.size(); ++i) {
            double dx = p[0] - brep.m_vertices[i][0];
            double dy = p[1] - brep.m_vertices[i][1];
            double dz = p[2] - brep.m_vertices[i][2];
            if (dx*dx + dy*dy + dz*dz < tol*tol) return i;
        }
        int idx = brep.add_vertex(p);
        brep.m_topology_vertices.push_back({idx, {}});
        return idx;
    };

    auto project_curve_to_uv = [&](const NurbsCurve& crv, const Point& org,
                                    const Vector& xa, const Vector& ya,
                                    double umin, double vmin, double du, double dv) -> NurbsCurve {
        NurbsCurve crv2d(3, crv.is_rational(), crv.order(), crv.cv_count());
        for (int i = 0; i < crv.nurbsknot_count(); ++i) crv2d.set_nurbsknot(i, crv.nurbsknot(i));
        for (int i = 0; i < crv.cv_count(); ++i) {
            if (crv.is_rational()) {
                auto [wx, wy, wz, w] = crv.get_cv_4d(i);
                double x = wx/w, y = wy/w, z = wz/w;
                double dx = x-org[0], dy = y-org[1], dz = z-org[2];
                double u = (dx*xa[0]+dy*xa[1]+dz*xa[2] - umin) / du;
                double v = (dx*ya[0]+dy*ya[1]+dz*ya[2] - vmin) / dv;
                crv2d.set_cv_4d(i, u*w, v*w, 0.0, w);
            } else {
                Point cv = crv.get_cv(i);
                double dx = cv[0]-org[0], dy = cv[1]-org[1], dz = cv[2]-org[2];
                double u = (dx*xa[0]+dy*xa[1]+dz*xa[2] - umin) / du;
                double v = (dx*ya[0]+dy*ya[1]+dz*ya[2] - vmin) / dv;
                crv2d.set_cv(i, Point(u, v, 0));
            }
        }
        return crv2d;
    };

    auto add_curve_loop = [&](const NurbsCurve& crv, int face_idx, BRepLoopType loop_type,
                               const Point& org, const Vector& xa, const Vector& ya,
                               double umin, double vmin, double du, double dv) {
        int li = brep.add_loop(face_idx, loop_type);
        int ci3d = brep.add_curve_3d(crv);
        NurbsCurve crv2d = project_curve_to_uv(crv, org, xa, ya, umin, vmin, du, dv);
        int c2d = brep.add_curve_2d(crv2d);
        Point sp = crv.point_at(crv.domain().first);
        Point ep = crv.point_at(crv.domain().second);
        int vi_s = find_or_add(sp), vi_e = crv.is_closed() ? vi_s : find_or_add(ep);
        int lo = std::min(vi_s, vi_e), hi = std::max(vi_s, vi_e);
        int ei = brep.add_edge(ci3d, lo, hi);
        brep.add_trim(c2d, ei, li, false, BRepTrimType::Boundary);
    };

    auto cv_points = [](const NurbsCurve& c) -> std::vector<Point> {
        std::vector<Point> pts;
        int nc = c.cv_count();
        pts.reserve(nc);
        for (int k = 0; k < nc; ++k) {
            if (c.is_rational()) {
                auto [wx, wy, wz, w] = c.get_cv_4d(k);
                if (w != 0.0) pts.emplace_back(wx/w, wy/w, wz/w);
            } else {
                pts.push_back(c.get_cv(k));
            }
        }
        return pts;
    };

    for (int ci = 0; ci < (int)curves.size(); ++ci) {
        const auto& crv = curves[ci];
        auto pts = cv_points(crv);
        // drop the repeated last point if closed (first == last)
        if (pts.size() >= 2) {
            auto& f = pts.front(); auto& b = pts.back();
            double dd = (f[0]-b[0])*(f[0]-b[0])+(f[1]-b[1])*(f[1]-b[1])+(f[2]-b[2])*(f[2]-b[2]);
            if (dd < tol*tol) pts.pop_back();
        }
        int n = (int)pts.size();
        if (n < 3) continue;

        Polyline pl(pts);
        Point org; Plane plane;
        pl.get_fast_plane(org, plane);
        if (!plane.is_valid()) continue;
        Vector xa = plane.x_axis(), ya = plane.y_axis();

        std::vector<double> us(n), vs(n);
        double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
        for (int i = 0; i < n; ++i) {
            double dx = pts[i][0] - org[0], dy = pts[i][1] - org[1], dz = pts[i][2] - org[2];
            us[i] = dx*xa[0] + dy*xa[1] + dz*xa[2];
            vs[i] = dx*ya[0] + dy*ya[1] + dz*ya[2];
            umin = std::min(umin, us[i]); umax = std::max(umax, us[i]);
            vmin = std::min(vmin, vs[i]); vmax = std::max(vmax, vs[i]);
        }
        // Include hole CVs in bounds (convex hull property guarantees curve lies within)
        if (ci < (int)holes.size()) {
            for (const auto& hcrv : holes[ci]) {
                for (const auto& hp : cv_points(hcrv)) {
                    double dx = hp[0]-org[0], dy = hp[1]-org[1], dz = hp[2]-org[2];
                    double hu = dx*xa[0]+dy*xa[1]+dz*xa[2], hv = dx*ya[0]+dy*ya[1]+dz*ya[2];
                    umin = std::min(umin, hu); umax = std::max(umax, hu);
                    vmin = std::min(vmin, hv); vmax = std::max(vmax, hv);
                }
            }
        }
        double pad = std::max(umax - umin, vmax - vmin) * 0.01;
        umin -= pad; umax += pad; vmin -= pad; vmax += pad;
        double du = umax - umin, dv = vmax - vmin;

        auto pt3d = [&](double u, double v) {
            return Point(org[0]+u*xa[0]+v*ya[0], org[1]+u*xa[1]+v*ya[1], org[2]+u*xa[2]+v*ya[2]);
        };
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, pt3d(umin, vmin)); srf.set_cv(1, 0, pt3d(umax, vmin));
        srf.set_cv(0, 1, pt3d(umin, vmax)); srf.set_cv(1, 1, pt3d(umax, vmax));
        int si = brep.add_surface(srf);
        int fi = brep.add_face(si, false);

        add_curve_loop(crv, fi, BRepLoopType::Outer, org, xa, ya, umin, vmin, du, dv);

        if (ci < (int)holes.size()) {
            for (const auto& hcrv : holes[ci]) {
                add_curve_loop(hcrv, fi, BRepLoopType::Inner, org, xa, ya, umin, vmin, du, dv);
            }
        }
    }

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

int BRep::face_count() const { return (int)m_faces.size(); }
int BRep::edge_count() const { return (int)m_topology_edges.size(); }
int BRep::vertex_count() const { return (int)m_vertices.size(); }

bool BRep::is_valid() const {
    if (m_faces.empty() || m_surfaces.empty() || m_vertices.empty()) return false;
    for (const auto& f : m_faces)
        if (f.surface_index < 0 || f.surface_index >= (int)m_surfaces.size()) return false;
    for (const auto& l : m_loops)
        if (l.face_index < 0 || l.face_index >= (int)m_faces.size()) return false;
    for (const auto& t : m_trims) {
        if (t.curve_2d_index < 0 || t.curve_2d_index >= (int)m_curves_2d.size()) return false;
        if (t.loop_index < 0 || t.loop_index >= (int)m_loops.size()) return false;
    }
    for (const auto& e : m_topology_edges) {
        if (e.start_vertex < 0 || e.start_vertex >= (int)m_topology_vertices.size()) return false;
        if (e.end_vertex < 0 || e.end_vertex >= (int)m_topology_vertices.size()) return false;
    }
    return true;
}

double brep_degenerate_tol(const BRep& X) {
    double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
    for (const auto& p : X.m_vertices) {
        xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
        xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
    }
    double diag = X.m_vertices.empty() ? 1.0 :
        std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
    if (diag <= 0) diag = 1.0;
    return std::max(diag * 1e-7, 1e-12);
}

bool brep_edge_is_degenerate(const BRep& X, const BRepEdge& e, double deg_tol) {
    int ci = e.curve_3d_index;
    if (ci < 0 || ci >= (int)X.m_curves_3d.size()) return false;
    const NurbsCurve& c = X.m_curves_3d[ci];
    auto dc = c.domain();
    Point p0 = c.point_at(dc.first);
    double ext = 0.0;
    for (int k = 1; k <= 4; ++k) {
        Point pk = c.point_at(dc.first + (dc.second - dc.first) * k / 4.0);
        ext = std::max(ext, p0.distance(pk));
    }
    return ext < deg_tol;
}

bool BRep::is_solid() const {
    if (m_topology_edges.empty()) return false;
    double deg_tol = brep_degenerate_tol(*this);
    bool solid = true;
    for (const auto& e : m_topology_edges) {
        if ((int)e.trim_indices.size() == 2) continue;
        // An ORPHANED edge (0 trims: no face references it) is not a manifold boundary edge -- it
        // is a dead record left by an alias/merge that reassigned its trims. When the boundary is
        // otherwise complete (any genuine hole would leave 1-trim NAKED edges on the adjacent faces),
        // such a record has no topological role, so it is excluded from the 2-manifold check (the
        // same scoping OCCT applies to degenerate edges below). Reported under SESSION_SOLID_DBG.
        if ((int)e.trim_indices.size() == 0) {
            if (std::getenv("SESSION_SOLID_DBG"))
                std::fprintf(stderr, "[SOLID] orphaned 0-trim edge skipped\n");
            continue;
        }
        // A DEGENERATE edge (3D curve collapsed to a point, e.g. a sphere/cone pole) is
        // watertight by construction -- the pole is a single point fully enclosed by its
        // face -- and OCCT excludes such degenerate edges from manifold checks. Skip them;
        // only genuine (non-zero-length) edges must be 2-trim.
        int ci = e.curve_3d_index;
        if (brep_edge_is_degenerate(*this, e, deg_tol)) continue;
        if (std::getenv("SESSION_SOLID_DBG")) {
            Point ps(0,0,0), pe(0,0,0);
            if (ci >= 0 && ci < (int)m_curves_3d.size()) {
                const NurbsCurve& c = m_curves_3d[ci];
                auto dc = c.domain();
                ps = c.point_at(dc.first); pe = c.point_at(dc.second);
            }
            std::fprintf(stderr, "[SOLID] e=%d nt=%d s(%.4f,%.4f,%.4f) e(%.4f,%.4f,%.4f)\n",
                (int)(&e - &m_topology_edges[0]), (int)e.trim_indices.size(),
                ps[0],ps[1],ps[2], pe[0],pe[1],pe[2]);
            solid = false;
            continue;
        }
        return false;
    }
    return solid;
}

double BRep::update_tolerances(double floor_tol, double vertex_pad, int samples_per_span) {
    // Closest point on `c` to `q` in [ta,tb], Newton-polished from `t`. The seed is a
    // parameter-fraction match along the same edge, so this is a local polish and not a search.
    auto closest_on = [](const NurbsCurve& c, const Point& q, double ta, double tb, double t) {
        if (tb < ta) std::swap(ta, tb);
        t = std::min(std::max(t, ta), tb);
        double best = 1e300;
        for (int it = 0; it < 10; ++it) {
            std::vector<Vector> d = c.evaluate(t, 2);
            if (d.size() < 3) break;
            const double rx = d[0][0]-q[0], ry = d[0][1]-q[1], rz = d[0][2]-q[2];
            best = std::min(best, rx*rx + ry*ry + rz*rz);
            const double f  = rx*d[1][0] + ry*d[1][1] + rz*d[1][2];
            const double fp = d[1][0]*d[1][0] + d[1][1]*d[1][1] + d[1][2]*d[1][2]
                            + rx*d[2][0] + ry*d[2][1] + rz*d[2][2];
            if (std::abs(fp) < 1e-300) break;
            double tn = std::min(std::max(t - f/fp, ta), tb);
            if (std::abs(tn - t) <= 1e-16 * std::max(1.0, std::abs(t))) { t = tn; break; }
            t = tn;
        }
        Point p = c.point_at(t);
        double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
        return std::sqrt(std::min(best, dx*dx + dy*dy + dz*dz));
    };
    const int K = std::max(1, samples_per_span);
    double worst = 0.0;
    for (auto& e : m_topology_edges) {
        double tol = floor_tol;
        if (e.curve_3d_index >= 0 && e.curve_3d_index < (int)m_curves_3d.size()) {
            const NurbsCurve& C = m_curves_3d[e.curve_3d_index];
            auto dc = C.domain();
            for (int ti : e.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const BRepTrim& T = m_trims[ti];
                if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)m_curves_2d.size()) continue;
                if (T.loop_index < 0 || T.loop_index >= (int)m_loops.size()) continue;
                int fi = m_loops[T.loop_index].face_index;
                if (fi < 0 || fi >= (int)m_faces.size()) continue;
                int si = m_faces[fi].surface_index;
                if (si < 0 || si >= (int)m_surfaces.size()) continue;
                const NurbsSurface& S = m_surfaces[si];
                const NurbsCurve& P = m_curves_2d[T.curve_2d_index];
                auto dp = P.domain();
                // Sample density follows the pcurve's OWN spans, so a finely represented trim
                // is measured finely and a 2-CV exact one is not oversampled.
                int nspan = std::max(1, P.cv_count() - P.degree());
                int n = std::min(std::max(nspan * K, 16), 4000);
                // The pcurve is CHORD-parametrized and the 3D curve is (typically) a rational
                // conic whose parameter is not arc length, so a same-FRACTION seed is off by
                // the whole reparametrization -- measured, that mis-seeding reported 3.0 on a
                // circle of radius 1.5, i.e. the diameter. Seed from a coarse table of the 3D
                // curve instead, then Newton-polish inside the bracketing table interval.
                const int M = std::min(std::max(nspan * 2, 64), 2048);
                std::vector<Point> tab;
                tab.reserve(M + 1);
                for (int k = 0; k <= M; ++k)
                    tab.push_back(C.point_at(dc.first + (dc.second - dc.first) * k / M));
                double tdev = 0.0, dev_a = 0.0, dev_b = 0.0;
                for (int i = 0; i <= n; ++i) {
                    double f = (double)i / n;
                    Point uv = P.point_at(dp.first + (dp.second - dp.first) * f);
                    Point q = S.point_at(uv[0], uv[1]);
                    int bk = 0; double bd = 1e300;
                    for (int k = 0; k <= M; ++k) {
                        double dd = tab[k].distance(q);
                        if (dd < bd) { bd = dd; bk = k; }
                    }
                    const double h = (dc.second - dc.first) / M;
                    double ta = dc.first + h * std::max(0, bk - 1);
                    double tb = dc.first + h * std::min(M, bk + 1);
                    double dv = closest_on(C, q, ta, tb, dc.first + h * bk);
                    tol = std::max(tol, dv);
                    tdev = std::max(tdev, dv);
                    if (i == 0) dev_a = dv;
                    if (i == n) dev_b = dv;
                }
                if (const char* td = std::getenv("SESSION_ETOL_DBG"); td && td[0]) {
                    Point a2 = S.point_at(P.point_at(dp.first)[0], P.point_at(dp.first)[1]);
                    Point b2 = S.point_at(P.point_at(dp.second)[0], P.point_at(dp.second)[1]);
                    std::fprintf(stderr,
                        "[ETOLD] e%d t%d f%d s%d pc(d%d,%d cv) dev=%.3e end0=%.3e end1=%.3e "
                        "pc0=(%.6f,%.6f,%.6f) pc1=(%.6f,%.6f,%.6f) c0=(%.6f,%.6f,%.6f) c1=(%.6f,%.6f,%.6f)\n",
                        (int)(&e - &m_topology_edges[0]), ti, fi, si, P.degree(), P.cv_count(),
                        tdev, dev_a, dev_b, a2[0],a2[1],a2[2], b2[0],b2[1],b2[2],
                        tab.front()[0],tab.front()[1],tab.front()[2],
                        tab.back()[0],tab.back()[1],tab.back()[2]);
                }
            }
        }
        e.tolerance = tol;
        worst = std::max(worst, tol);
    }
    for (auto& v : m_topology_vertices) {
        double tol = floor_tol;
        Point vp(0, 0, 0);
        bool have = v.point_index >= 0 && v.point_index < (int)m_vertices.size();
        if (have) vp = m_vertices[v.point_index];
        for (int ei : v.edge_indices) {
            if (ei < 0 || ei >= (int)m_topology_edges.size()) continue;
            const BRepEdge& E = m_topology_edges[ei];
            // OCCT enlarges a vertex to the incident curve tolerance plus exactly 1.0e-12.
            tol = std::max(tol, E.tolerance + vertex_pad);
            if (!have || E.curve_3d_index < 0 || E.curve_3d_index >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& C = m_curves_3d[E.curve_3d_index];
            auto dc = C.domain();
            tol = std::max(tol, std::min(vp.distance(C.point_at(dc.first)),
                                         vp.distance(C.point_at(dc.second))));
        }
        v.tolerance = tol;
    }
    return worst;
}

std::string BRep::topology_report(bool* valid_manifold) const {
    int nf = (int)m_faces.size();
    int ne = (int)m_topology_edges.size();
    double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
    for (const auto& p : m_vertices) {
        xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
        xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
    }
    double diag = m_vertices.empty() ? 1.0 :
        std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
    if (diag <= 0) diag = 1.0;
    double deg_tol = std::max(diag * 1e-7, 1e-12);
    auto face_of_trim = [&](int ti) -> int {
        if (ti < 0 || ti >= (int)m_trims.size()) return -1;
        int li = m_trims[ti].loop_index;
        return (li >= 0 && li < (int)m_loops.size()) ? m_loops[li].face_index : -1;
    };
    auto edge_len = [&](const BRepEdge& e) -> double {
        int ci = e.curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) return 0.0;
        const NurbsCurve& c = m_curves_3d[ci];
        auto d = c.domain(); Point p0 = c.point_at(d.first); double ext = 0.0;
        for (int k = 1; k <= 4; ++k) ext = std::max(ext, p0.distance(c.point_at(d.first+(d.second-d.first)*k/4.0)));
        return ext;
    };
    // Edge classes (skip 3D-degenerate pole/seam edges, as is_solid does).
    int naked = 0, nonman = 0, live_edges = 0;
    for (const auto& e : m_topology_edges) {
        int nt = (int)e.trim_indices.size();
        if (nt == 2) { ++live_edges; continue; }
        if (edge_len(e) < deg_tol) continue;               // degenerate: watertight by construction
        ++live_edges;
        if (nt == 1) ++naked;
        else if (nt > 2) ++nonman;
    }
    // Connected shell components: union-find over faces joined by 2-trim (manifold) edges.
    std::vector<int> uf(std::max(nf,1)); for (int i=0;i<nf;++i) uf[i]=i;
    std::function<int(int)> find=[&](int a){ while(uf[a]!=a){uf[a]=uf[uf[a]];a=uf[a];} return a; };
    auto uni=[&](int a,int b){ a=find(a); b=find(b); if(a!=b) uf[a]=b; };
    for (const auto& e : m_topology_edges) {
        if ((int)e.trim_indices.size() != 2) continue;
        int f1 = face_of_trim(e.trim_indices[0]), f2 = face_of_trim(e.trim_indices[1]);
        if (f1 >= 0 && f2 >= 0 && f1 != f2) uni(f1, f2);
    }
    std::set<int> roots; for (int i=0;i<nf;++i) if (nf>0) roots.insert(find(i));
    int nshell = (int)roots.size();
    // Duplicate topology vertices (two distinct vertices within deg_tol): a sew/weld miss.
    int dup_v = 0;
    {
        std::vector<int> vids;
        for (const auto& tv : m_topology_vertices)
            if (tv.point_index >= 0 && tv.point_index < (int)m_vertices.size()) vids.push_back(tv.point_index);
        for (size_t i=0;i<vids.size();++i)
            for (size_t j=i+1;j<vids.size();++j)
                if (m_vertices[vids[i]].distance(m_vertices[vids[j]]) < deg_tol) { ++dup_v; }
    }
    // Per-shell Euler characteristic chi = V - E + F (2 for genus-0 closed, 2-2g general).
    // V/E counted from the shell's live (non-degenerate) edges and their endpoints.
    std::map<int,int> shellF, shellE; std::map<int,std::set<int>> shellV;
    for (int fi=0; fi<nf; ++fi) shellF[find(fi)]++;
    for (const auto& e : m_topology_edges) {
        if ((int)e.trim_indices.size() < 2) continue;      // count manifold edges only for chi
        if (edge_len(e) < deg_tol) continue;
        int f = face_of_trim(e.trim_indices[0]);
        if (f < 0) continue;
        int r = find(f);
        shellE[r]++;
        if (e.start_vertex >= 0) shellV[r].insert(e.start_vertex);
        if (e.end_vertex >= 0)   shellV[r].insert(e.end_vertex);
    }
    std::string euler;
    bool all_even_chi = true;
    for (int r : roots) {
        int V = (int)shellV[r].size(), E = shellE[r], F = shellF[r];
        int chi = V - E + F;
        if (chi % 2 != 0) all_even_chi = false;
        char buf[96]; std::snprintf(buf,sizeof buf," shell@%d: V=%d E=%d F=%d chi=%d", r, V,E,F,chi);
        euler += buf;
        // Single-face shells (F=1) are the "orphan" artifacts: dump the face and WHY it is
        // isolated -- edge trim counts, self-seam count, max edge length (degenerate sliver?).
        if (F == 1 && std::getenv("SESSION_ORPHAN_DBG")) {
            int fi = -1;
            for (int q = 0; q < nf; ++q) if (find(q) == r) { fi = q; break; }
            if (fi >= 0 && fi < (int)m_faces.size()) {
                int si = m_faces[fi].surface_index;
                int nt1 = 0, nt2 = 0, nself = 0, nt3 = 0; double maxlen = 0, minlen = 1e300;
                for (int li : m_faces[fi].loop_indices) {
                    if (li < 0 || li >= (int)m_loops.size()) continue;
                    for (int ti : m_loops[li].trim_indices) {
                        if (ti < 0 || ti >= (int)m_trims.size()) continue;
                        int ei = m_trims[ti].edge_index;
                        if (ei < 0 || ei >= (int)m_topology_edges.size()) continue;
                        const auto& e = m_topology_edges[ei];
                        int tc = (int)e.trim_indices.size();
                        double L = edge_len(e);
                        maxlen = std::max(maxlen, L); minlen = std::min(minlen, L);
                        if (tc == 1) ++nt1;
                        else if (tc == 2) { ++nt2;
                            int of1 = face_of_trim(e.trim_indices[0]), of2 = face_of_trim(e.trim_indices[1]);
                            if (of1 == of2) ++nself; }
                        else if (tc >= 3) ++nt3;
                    }
                }
                std::fprintf(stderr, "[ORPHAN] shell@%d face%d surf%d edges(1t=%d 2t=%d 3t+=%d self-seam=%d) len[%.4f..%.4f]\n",
                             r, fi, si, nt1, nt2, nt3, nself, minlen, maxlen);
            }
        }
    }
    bool ok = (naked==0 && nonman==0 && all_even_chi);
    if (valid_manifold) *valid_manifold = ok;
    char head[256];
    std::snprintf(head, sizeof head,
        "[TOPO] faces=%d edges(live)=%d naked=%d nonmanifold=%d shells=%d dupV=%d manifold=%s |",
        nf, live_edges, naked, nonman, nshell, dup_v, ok ? "YES" : "NO");
    return std::string(head) + euler;
}

std::string BRepVerdict::row() const {
    char buf[256];
    std::snprintf(buf, sizeof buf,
        "VERDICT faces=%d naked=%d nonman=%d degen=%d orphan=%d shells=%d "
        "closed_shells=%d closed=%d closure=%.3e vol=%.9g",
        faces, naked, nonmanifold, degenerate, orphan, shells, closed_shells,
        closed ? 1 : 0, closure_residual, volume);
    return std::string(buf);
}

BRepVerdict BRep::verdict(bool with_volume) const {
    BRepVerdict v;
    v.faces = (int)m_faces.size();
    double deg_tol = brep_degenerate_tol(*this);
    auto face_of_trim = [&](int ti) -> int {
        if (ti < 0 || ti >= (int)m_trims.size()) return -1;
        int li = m_trims[ti].loop_index;
        return (li >= 0 && li < (int)m_loops.size()) ? m_loops[li].face_index : -1;
    };
    // One union-find over faces joined by ANY shared edge (brep_shell_count semantics:
    // a non-manifold junction still connects), so `shells` matches the numbers every
    // battery row has historically printed.
    int nf = v.faces;
    std::vector<int> uf(std::max(nf, 1));
    for (int i = 0; i < nf; ++i) uf[i] = i;
    std::function<int(int)> find = [&](int a) {
        while (uf[a] != a) { uf[a] = uf[uf[a]]; a = uf[a]; }
        return a;
    };
    for (const auto& e : m_topology_edges) {
        int prev = -1;
        for (int ti : e.trim_indices) {
            int fi = face_of_trim(ti);
            if (fi < 0) continue;
            if (prev >= 0 && fi != prev) { int a = find(prev), b = find(fi); if (a != b) uf[a] = b; }
            prev = fi;
        }
    }
    // Edge classes, degeneracy-excluded exactly as is_solid(); per-shell defect tally.
    std::map<int, int> shell_bad;
    for (const auto& e : m_topology_edges) {
        int nt = (int)e.trim_indices.size();
        if (nt == 0) { ++v.orphan; continue; }
        if (brep_edge_is_degenerate(*this, e, deg_tol)) { ++v.degenerate; continue; }
        if (nt == 2) continue;
        if (nt == 1) ++v.naked; else ++v.nonmanifold;
        int fi = face_of_trim(e.trim_indices[0]);
        if (fi >= 0) ++shell_bad[find(fi)];
    }
    std::set<int> roots;
    for (int i = 0; i < nf; ++i) roots.insert(find(i));
    v.shells = (int)roots.size();
    for (int r : roots)
        if (shell_bad.find(r) == shell_bad.end()) ++v.closed_shells;
    v.closed = (nf > 0 && v.naked == 0 && v.nonmanifold == 0);
    if (with_volume) {
        MassProps mp = brep_massprops(*this);
        v.closure_residual = mp.closure_residual;
        v.volume = v.closed ? mp.volume : std::numeric_limits<double>::quiet_NaN();
    } else {
        v.closure_residual = std::numeric_limits<double>::quiet_NaN();
        v.volume = std::numeric_limits<double>::quiet_NaN();
    }
    return v;
}

bool BRep::contains_point(const Mesh& boundary, const Point& p) const {
    // Generalized winding number (Van Oosterom-Strackee solid angles): deterministic and
    // crack-robust -- a tessellation gap perturbs the winding by its own (tiny) solid
    // angle instead of flipping a ray parity. The previous 7-ray majority still answered
    // wrong on ~14% of near-section queries on imported freeform meshes and made
    // classification a lottery. |winding| >= 0.5 => inside (orientation-sign agnostic).
    // Recognized primitives never reach here (inside_prim tests analytically).
    auto [vertices, faces] = boundary.to_vertices_and_faces();
    double omega = 0.0;
    for (const auto& face : faces) {
        if (face.size() < 3) continue;
        for (size_t j = 1; j + 1 < face.size(); ++j) {
            const Point& A = vertices[face[0]];
            const Point& B = vertices[face[j]];
            const Point& C = vertices[face[j + 1]];
            double ax = A[0]-p[0], ay = A[1]-p[1], az = A[2]-p[2];
            double bx = B[0]-p[0], by = B[1]-p[1], bz = B[2]-p[2];
            double cx = C[0]-p[0], cy = C[1]-p[1], cz = C[2]-p[2];
            double la = std::sqrt(ax*ax + ay*ay + az*az);
            double lb = std::sqrt(bx*bx + by*by + bz*bz);
            double lc = std::sqrt(cx*cx + cy*cy + cz*cz);
            if (la < 1e-15 || lb < 1e-15 || lc < 1e-15) return true;   // p ON the boundary
            double det = ax*(by*cz - bz*cy) - ay*(bx*cz - bz*cx) + az*(bx*cy - by*cx);
            double den = la*lb*lc + (ax*bx + ay*by + az*bz)*lc
                       + (bx*cx + by*cy + bz*cz)*la + (cx*ax + cy*ay + cz*az)*lb;
            omega += 2.0 * std::atan2(det, den);
        }
    }
    return std::abs(omega) > 2.0 * 3.14159265358979323846;
}

bool BRep::contains_point(const Point& p) const {
    return contains_point(mesh(), p);
}

bool BRep::contains_point_exact(const Point& p, const std::vector<double>& osign_in) const {
    // Point-in-poly on a face's trim loops sampled as UV polygons (outer minus holes).
    auto uv_in_trims = [&](int fi, double u, double v) -> bool {
        const auto& face = m_faces[fi];
        bool in_outer = false, have_outer = false;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            std::vector<std::array<double,2>> poly;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                auto dc = pc.domain();
                int ns = std::min(std::max(pc.cv_count() * 2, 16), 128);
                for (int k = 0; k < ns; ++k) {
                    Point q = pc.point_at(dc.first + (dc.second - dc.first) * k / ns);
                    poly.push_back({q[0], q[1]});
                }
            }
            if (poly.size() < 3) continue;
            bool inp = false;
            for (size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++)
                if (((poly[a][1] > v) != (poly[b][1] > v)) &&
                    (u < (poly[b][0]-poly[a][0])*(v-poly[a][1])/(poly[b][1]-poly[a][1]+1e-30)+poly[a][0]))
                    inp = !inp;
            if (loop.type == BRepLoopType::Outer) { in_outer = inp; have_outer = true; }
            else if (inp) return false;   // inside a hole
        }
        return have_outer ? in_outer : false;
    };
    const std::vector<double>& osign = osign_in;
    std::vector<double> osign_local;
    const std::vector<double>* sg = &osign;
    if (osign.empty()) { osign_local = face_outward_signs(); sg = &osign_local; }
    // Closest point on a face's TRIM BOUNDARY (the trim curves lifted to 3D). Used when the
    // surface-closest point falls OUTSIDE the face's trims: the closest point of the TRIMMED
    // face is then on its boundary, not skipped. Skipping (the old behavior) attributed a
    // near-boundary probe to a FARTHER face and read the wrong outward-normal sign -- the
    // documented grazing bug that also misclassified cone x cyl (a passing primitive test).
    auto closest_on_trim = [&](int fi, double& ou, double& ov) -> double {
        int si = m_faces[fi].surface_index;
        const NurbsSurface& S = m_surfaces[si];
        double bd = 1e300;
        for (int li : m_faces[fi].loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                auto dc = pc.domain();
                int ns = std::min(std::max(pc.cv_count(), 8), 24);   // coarse: only need the sign
                for (int k = 0; k <= ns; ++k) {
                    Point q = pc.point_at(dc.first + (dc.second - dc.first) * k / ns);
                    double dd = S.point_at(q[0], q[1]).distance(p);
                    if (dd < bd) { bd = dd; ou = q[0]; ov = q[1]; }
                }
            }
        }
        return bd;
    };
    bool exact2 = std::getenv("SESSION_NO_EXACT_PIP2") == nullptr;   // robust boundary-fallback (default on)
    // PIP-GUARD (session B defect report, kb/p3_integration_notes.md 5.1): the signed-nearest
    // rule is unsound when the nearest point lies on a TRIM BOUNDARY (the probe is nearest to an
    // EDGE, where the face normal is not the separating direction -- p and q sit at the same
    // radius but different angles, so (p-q).n = cos(dtheta)-1 < 0 ALWAYS: a systematic false
    // "inside" of order the sampling sagitta, measured -1.9e-2 on stacked unit cylinders), and
    // when the probe direction is tangential to the boundary (|dp| ~ 1e-16 = pure round-off,
    // sign flips per sample). Tier 1 (free): prefer an INTERIOR hit over an equidistant BOUNDARY
    // hit. Tier 2: if the winner is still a boundary hit or |dp| <= 1e-6*d, settle by ray parity.
    // Default ON since 2026-08-14: both unsound cases fire on flush imported operands
    // (GZ-J phi=0: A's flush face read in_p==in_m==1, the top face's tangential probes
    // coin-flipped all-inside -> cut dropped 2 faces, common invented a phantom slab).
    // With the guard, flush cut/common/fuse are exact. SESSION_NO_PIP_GUARD reverts.
    static const bool s_pip_guard = (std::getenv("SESSION_NO_PIP_GUARD") == nullptr);
    double best_d = 1e300; int best_fi = -1; double bu = 0, bv = 0;
    bool best_on_boundary = false;
    double int_d = 1e300; int int_fi = -1; double iu = 0, iv = 0;   // best INTERIOR (in-trim) hit
    // Pass 1: in-trim closest points (cheap; byte-identical to the legacy path). Off-trim faces
    // are DEFERRED so their (expensive) boundary sampling runs only when it could still win.
    std::vector<std::array<double,4>> offtrim;   // {surf-closest d, fi, u, v}
    for (int fi = 0; fi < (int)m_faces.size(); ++fi) {
        int si = m_faces[fi].surface_index;
        if (si < 0 || si >= (int)m_surfaces.size()) continue;
        auto [u, v, d] = Closest::surface_point(m_surfaces[si], p);
        if (uv_in_trims(fi, u, v)) {
            if (d < best_d) { best_d = d; best_fi = fi; bu = u; bv = v; best_on_boundary = false; }
            if (d < int_d) { int_d = d; int_fi = fi; iu = u; iv = v; }
        } else if (exact2) {
            offtrim.push_back({d, (double)fi, u, v});
        }
    }
    // Pass 2: boundary-fallback for off-trim faces only. Sorted by surface-closest distance so
    // best_d shrinks fast and far faces are pruned (a face's boundary distance >= its
    // surface-closest distance). This fires the sampling only for the very few nearest faces.
    if (exact2 && !offtrim.empty()) {
        std::sort(offtrim.begin(), offtrim.end(),
                  [](const std::array<double,4>& a, const std::array<double,4>& b){ return a[0] < b[0]; });
        for (auto& o : offtrim) {
            if (o[0] >= best_d) break;   // sorted: this and all later off-trim faces cannot win
            double ou = o[2], ov = o[3];
            double bd = closest_on_trim((int)o[1], ou, ov);
            if (bd < best_d) { best_d = bd; best_fi = (int)o[1]; bu = ou; bv = ov; best_on_boundary = true; }
        }
    }
    // Tier 1: an interior hit at (essentially) the same distance is the sound one -- its normal
    // IS the separating direction, while the boundary hit's is not.
    if (s_pip_guard && best_on_boundary && int_fi >= 0 && int_d <= best_d * (1.0 + 1e-6)) {
        best_d = int_d; best_fi = int_fi; bu = iu; bv = iv; best_on_boundary = false;
    }
    if (best_fi < 0) return false;   // no in-trim boundary point found: treat as outside
    int si = m_faces[best_fi].surface_index;
    Vector n = m_surfaces[si].normal_at(bu, bv);
    double s = (best_fi < (int)sg->size()) ? (*sg)[best_fi] : 1.0;
    Point bpt = m_surfaces[si].point_at(bu, bv);
    double dp = (p[0]-bpt[0])*n[0]*s + (p[1]-bpt[1])*n[1]*s + (p[2]-bpt[2])*n[2]*s;
    if (s_pip_guard && (best_on_boundary || std::abs(dp) <= 1e-6 * best_d)) {
        // Tier 2: the sign carries no information here -- settle by ray parity on the mesh.
        // 1-entry cache (classification probes the SAME operand thousands of times in a row;
        // mesh() is not cached and tessellating per probe would dominate the runtime).
        static thread_local const BRep* s_owner = nullptr;
        static thread_local size_t s_nf = 0, s_nv = 0;
        static thread_local Mesh s_mesh;
        if (s_owner != this || s_nf != m_faces.size() || s_nv != m_vertices.size()) {
            s_mesh = mesh();
            s_owner = this; s_nf = m_faces.size(); s_nv = m_vertices.size();
        }
        return contains_point(s_mesh, p);
    }
    return dp < 0.0;   // inner side of the outward normal
}

namespace { bool cylinder_of_surface(const NurbsSurface& s, Point& A, Vector& W, double& R); }

int BRep::check_trim_orientation(bool verbose) const {
    int violations = 0;
    for (int fi = 0; fi < (int)m_faces.size(); ++fi) {
        const BRepFace& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        auto du = srf.domain(0); auto dv = srf.domain(1);
        double ru = du.second - du.first, rv = dv.second - dv.first;
        double tol_uv = 1e-6 * std::max(ru, rv);
        double jump_uv = 0.1 * std::max(ru, rv);
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const BRepLoop& loop = m_loops[li];
            bool have_prev = false, jumpy = false;
            Point first_pt(0,0,0), prev_pt(0,0,0);
            double area2 = 0.0;   // 2*signed area, shoelace over trim endpoints + samples
            Point area_prev(0,0,0); bool area_first = true;
            int ntrim = 0;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                if (!pc.is_valid()) continue;
                auto dc = pc.domain();
                bool rev = m_trims[ti].reversed;
                Point s = pc.point_at(rev ? dc.second : dc.first);
                Point e = pc.point_at(rev ? dc.first : dc.second);
                if (have_prev) {
                    double gap = std::hypot(s[0]-prev_pt[0], s[1]-prev_pt[1]);
                    if (gap > jump_uv) jumpy = true;
                    else if (gap > tol_uv) {
                        ++violations;
                        if (verbose) std::fprintf(stderr,
                            "[ORIENT] f=%d loop=%d trim=%d chain gap %.3e at uv(%.4f,%.4f)\n",
                            fi, li, ti, gap, s[0], s[1]);
                    }
                } else { first_pt = s; }
                const int NS = 8;
                for (int k = 0; k <= NS; ++k) {
                    double f = (double)k / NS;
                    double t = rev ? dc.second - (dc.second-dc.first)*f
                                   : dc.first + (dc.second-dc.first)*f;
                    Point p = pc.point_at(t);
                    if (!area_first) area2 += area_prev[0]*p[1] - p[0]*area_prev[1];
                    area_prev = p; area_first = false;
                }
                prev_pt = e; have_prev = true; ++ntrim;
            }
            if (!have_prev || ntrim == 0) continue;
            double close_gap = std::hypot(first_pt[0]-prev_pt[0], first_pt[1]-prev_pt[1]);
            if (close_gap > jump_uv) jumpy = true;
            else if (close_gap > tol_uv) {
                ++violations;
                if (verbose) std::fprintf(stderr, "[ORIENT] f=%d loop=%d closure gap %.3e\n", fi, li, close_gap);
            }
            area2 += area_prev[0]*first_pt[1] - first_pt[0]*area_prev[1];
            if (!jumpy && std::abs(area2) > tol_uv * std::max(ru, rv)) {
                bool ccw = area2 > 0.0;
                bool want_ccw = (loop.type == BRepLoopType::Outer);
                if (ccw != want_ccw) {
                    ++violations;
                    if (verbose) std::fprintf(stderr,
                        "[ORIENT] f=%d loop=%d type=%d area2=%.3e wrong sense (ntrim=%d)\n",
                        fi, li, (int)loop.type, area2, ntrim);
                }
            } else if (jumpy && verbose) {
                std::fprintf(stderr, "[ORIENT] f=%d loop=%d SEAM-JUMP loop (ntrim=%d) - sense unchecked\n", fi, li, ntrim);
            }
        }
    }
    return violations;
}


bool BRep::loop_material_left(int li, const std::vector<std::pair<int,char>>* dirs) const {
    if (li < 0 || li >= (int)m_loops.size()) return true;
    int fi = m_loops[li].face_index;
    if (fi < 0 || fi >= (int)m_faces.size()) return true;
    int si = m_faces[fi].surface_index;
    if (si < 0 || si >= (int)m_surfaces.size()) return true;
    const NurbsSurface& S = m_surfaces[si];
    // face UV region polys (all loops of the face)
    std::vector<std::vector<std::array<double,2>>> outs, ins;
    for (int lj : m_faces[fi].loop_indices) {
        if (lj < 0 || lj >= (int)m_loops.size()) continue;
        std::vector<std::array<double,2>> poly;
        for (int ti : m_loops[lj].trim_indices) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int c2 = m_trims[ti].curve_2d_index;
            if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
            const NurbsCurve& pc = m_curves_2d[c2];
            if (!pc.is_valid()) continue;
            auto dc = pc.domain();
            int n = std::min(std::max(pc.cv_count()*2, 8), 256);
            for (int k = 0; k < n; ++k) {
                Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/n);
                poly.push_back({uv[0], uv[1]});
            }
        }
        if (poly.size() < 3) continue;
        if (m_loops[lj].type == BRepLoopType::Outer) outs.push_back(std::move(poly));
        else ins.push_back(std::move(poly));
    }
    auto in_face = [&](double u, double v) -> bool {
        auto pip = [&](const std::vector<std::array<double,2>>& p) {
            bool inside = false;
            for (size_t i = 0, j = p.size()-1; i < p.size(); j = i++) {
                if ((p[i][1] > v) != (p[j][1] > v) &&
                    u < (p[j][0]-p[i][0])*(v-p[i][1])/(p[j][1]-p[i][1]) + p[i][0])
                    inside = !inside;
            }
            return inside;
        };
        bool ok = outs.empty();
        for (auto& op : outs) if (pip(op)) { ok = true; break; }
        if (!ok) return false;
        for (auto& ip : ins) if (pip(ip)) return false;
        return true;
    };
    auto [su0,su1] = S.domain(0); auto [sv0,sv1] = S.domain(1);
    int votes_left = 0, votes_right = 0;
    for (int ti : m_loops[li].trim_indices) {
        if (ti < 0 || ti >= (int)m_trims.size()) continue;
        const BRepTrim& T = m_trims[ti];
        int c2 = T.curve_2d_index;
        if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
        const NurbsCurve& pc = m_curves_2d[c2];
        if (!pc.is_valid()) continue;
        auto dc = pc.domain();
        double tm = 0.5*(dc.first+dc.second), dt = (dc.second-dc.first)*1e-3;
        Point a = pc.point_at(tm - dt), b = pc.point_at(tm + dt);
        bool from_first = !T.reversed;
        if (dirs) {
            from_first = true;   // default when the trim is absent from the override list
            for (const auto& d : *dirs)
                if (d.first == ti) { from_first = d.second != 0; break; }
        }
        if (!from_first) std::swap(a, b);
        double tx = b[0]-a[0], ty = b[1]-a[1];
        double tn = std::sqrt(tx*tx+ty*ty);
        if (tn < 1e-15) continue;
        Point m = pc.point_at(tm);
        for (double f : {2e-3, 8e-3}) {
            double eu = (su1-su0)*f, ev = (sv1-sv0)*f;
            bool left_in = in_face(m[0] - ty/tn*eu, m[1] + tx/tn*ev);
            bool right_in = in_face(m[0] + ty/tn*eu, m[1] - tx/tn*ev);
            if (left_in == right_in) continue;
            if (left_in) ++votes_left; else ++votes_right;
            break;
        }
    }
    return votes_left >= votes_right;
}

std::vector<double> BRep::face_outward_signs(std::vector<Point>* P3s, std::vector<Vector>* Ns) const {
    auto cross = [](const Vector& a, const Vector& b) {
        return Vector(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
    };
    (void)cross;
    auto is_planar = [&](const NurbsSurface& s) {
        auto [u0,u1] = s.domain(0); auto [v0,v1] = s.domain(1);
        Vector n0 = s.normal_at(u0 + (u1-u0)*0.5, v0 + (v1-v0)*0.5);
        double uu[3] = {0.25, 0.5, 0.75}, vv[3] = {0.3, 0.6, 0.8};
        for (int i = 0; i < 3; i++) {
            Vector n = s.normal_at(u0 + (u1-u0)*uu[i], v0 + (v1-v0)*vv[i]);
            Vector c(n0[1]*n[2]-n0[2]*n[1], n0[2]*n[0]-n0[0]*n[2], n0[0]*n[1]-n0[1]*n[0]);
            if (c.magnitude() > 1e-7) return false;
        }
        return true;
    };
    // Interior UV point of a face (CDT centroid), plus its natural unit normal.
    auto face_interior = [&](const BRepFace& face, const NurbsSurface& srf, Point& P3, Vector& Nnat,
                             double& ocu, double& ocv) {
        std::vector<Polyline> outers, inners;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            std::vector<Point> pts;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                auto dc = pc.domain();
                // Cap: a dense polyline pcurve (torus pullback of a sampled section loop,
                // cv ~ 4000) does not need cv*3 samples to seed the CDT -- the uncapped
                // count produced ~50k-vertex polygons and a quadratic CDT.
                int n = std::min(std::max(pc.cv_count()*3, 6), 1024);
                for (int i = 0; i < n; ++i) {
                    Point uv = pc.point_at(dc.first + (dc.second-dc.first)*i/n);
                    pts.push_back(Point(uv[0], uv[1], 0));
                }
            }
            if (pts.size() < 3) continue;
            if (m_loops[li].type == BRepLoopType::Outer) outers.emplace_back(pts);
            else inners.emplace_back(pts);
        }
        auto [u0,u1] = srf.domain(0); auto [v0,v1] = srf.domain(1);
        double cu = 0.5*(u0+u1), cv = 0.5*(v0+v1);
        auto pip = [&](double uu, double vv, const Polyline& poly) -> bool {
            auto pp = poly.get_points();
            bool inside = false; int j = (int)pp.size() - 1;
            for (int i = 0; i < (int)pp.size(); ++i) {
                double denom = (pp[j][1] - pp[i][1]); if (denom == 0.0) denom = 1e-300;
                if (((pp[i][1] > vv) != (pp[j][1] > vv)) &&
                    (uu < (pp[j][0]-pp[i][0])*(vv-pp[i][1])/denom + pp[i][0]))
                    inside = !inside;
                j = i;
            }
            return inside;
        };
        auto in_material = [&](double uu, double vv) -> bool {
            if (!outers.empty()) {
                bool any = false;
                for (auto& op : outers) if (pip(uu, vv, op)) { any = true; break; }
                if (!any) return false;
            }
            for (auto& ip : inners) if (pip(uu, vv, ip)) return false;
            return true;
        };
        if (!outers.empty()) {
            // Wire-aware, like BRep::boolean's face_sample: a seam-merged face carries one outer
            // wire per seam side, and a hole belonging to the other wire sits OUTSIDE this one.
            // No-op for the single-outer-wire faces every default path produces.
            auto bbox_of = [](const Polyline& pl) {
                std::array<double,4> b = {1e300, -1e300, 1e300, -1e300};
                for (const auto& p : pl.get_points()) {
                    b[0] = std::min(b[0], p[0]); b[1] = std::max(b[1], p[0]);
                    b[2] = std::min(b[2], p[1]); b[3] = std::max(b[3], p[1]);
                }
                return b;
            };
            size_t obest = 0;
            if (outers.size() > 1) {
                double barea = -1.0;
                for (size_t k = 0; k < outers.size(); ++k) {
                    auto b = bbox_of(outers[k]);
                    double ar = (b[1]-b[0]) * (b[3]-b[2]);
                    if (ar > barea) { barea = ar; obest = k; }
                }
            }
            std::vector<Polyline> all; all.push_back(outers[obest]);
            {
                auto B = bbox_of(outers[obest]);
                for (auto& in : inners) {
                    if (outers.size() > 1) {
                        auto b = bbox_of(in);
                        if (b[0] < B[0] || b[1] > B[1] || b[2] < B[2] || b[3] > B[3]) continue;
                    }
                    all.push_back(in);
                }
            }
            std::vector<std::array<int,3>> tris;
            try { tris = RemeshCDT::triangulate(all); } catch (...) {}
            std::vector<Point> flat;
            for (auto& pl : all) { auto pp = pl.get_points(); for (auto& p : pp) flat.push_back(p); }
            // Largest-area triangle whose centroid is on the face MATERIAL (inside the outer
            // loop, outside every hole). The first triangle / domain centre can land in a hole
            // (e.g. an annulus' centre) -> a wrong outward-sign probe -> wrong flux.
            double best_a = -1.0;
            for (const auto& t : tris) {
                if (t[0] >= (int)flat.size() || t[1] >= (int)flat.size() || t[2] >= (int)flat.size()) continue;
                const auto& a = flat[t[0]]; const auto& b = flat[t[1]]; const auto& c = flat[t[2]];
                double tcu = (a[0]+b[0]+c[0])/3.0, tcv = (a[1]+b[1]+c[1])/3.0;
                double ar = std::abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]));
                if (ar > best_a && in_material(tcu, tcv)) { best_a = ar; cu = tcu; cv = tcv; }
            }
            if (best_a < 0.0) {
                // CDT gave nothing usable -> grid-sample for any in-material point.
                bool found = false;
                for (int iu = 1; iu < 12 && !found; ++iu)
                    for (int iv = 1; iv < 12 && !found; ++iv) {
                        double su = u0 + (u1-u0)*iu/12.0, sv = v0 + (v1-v0)*iv/12.0;
                        if (in_material(su, sv)) { cu = su; cv = sv; found = true; }
                    }
            }
        }
        P3 = srf.point_at(cu, cv);
        Nnat = srf.normal_at(cu, cv);  // unit; natural orientation = Su x Sv direction
        ocu = cu; ocv = cv;
    };

    Mesh bmesh = mesh();
    // bbox diagonal for the off-face step.
    double xmin=1e300,ymin=1e300,zmin=1e300,xmax=-1e300,ymax=-1e300,zmax=-1e300;
    for (const auto& p : m_vertices) {
        xmin=std::min(xmin,p[0]); ymin=std::min(ymin,p[1]); zmin=std::min(zmin,p[2]);
        xmax=std::max(xmax,p[0]); ymax=std::max(ymax,p[1]); zmax=std::max(zmax,p[2]);
    }
    double diag = std::sqrt((xmax-xmin)*(xmax-xmin)+(ymax-ymin)*(ymax-ymin)+(zmax-zmin)*(zmax-zmin));
    double eps = (diag > 0 ? diag : 1.0) * 1e-3;

    // ---- PASS 0: shell-consistent face signs by ORIENTATION PROPAGATION (OCCT shell healing).
    // Per-face parity votes are heuristics only (tessellation cracks near tangencies break ray
    // parity; a quadric outward reference inverts on cavity shells). The boolean's validated
    // invariant -- mated trims traverse their shared edge ANTI-PARALLEL exactly when the two
    // faces' natural normals are in the same orientation class -- makes RELATIVE signs exact:
    // propagate them across every 2-trim edge, then fix each connected shell's single global
    // flip by weighted evidence (planar supporting-plane tests are exact geometry -> weight 3;
    // double-sided parity probes that disagree between the two sides -> weight 1). A face sign
    // can then only be wrong if the evidence outvotes the WHOLE shell, and disjoint lobes /
    // cavity shells are seeded independently.
    int nfc = (int)m_faces.size();
    std::vector<Point>  fP3(nfc, Point(0,0,0));
    std::vector<Vector> fNn(nfc, Vector(0,0,0));
    std::vector<double> fCU(nfc, 0.0), fCVv(nfc, 0.0);
    std::vector<double> fsign(nfc, 1.0);
    {
        std::vector<char> fvalid(nfc, 0);
        for (int fi = 0; fi < nfc; ++fi) {
            const BRepFace& fc = m_faces[fi];
            if (fc.surface_index < 0 || fc.surface_index >= (int)m_surfaces.size()) continue;
            face_interior(fc, m_surfaces[fc.surface_index], fP3[fi], fNn[fi], fCU[fi], fCVv[fi]);
            if (fNn[fi].magnitude() >= 1e-12) fvalid[fi] = 1;
        }
        auto face_of_trim = [&](int ti) -> int {
            if (ti < 0 || ti >= (int)m_trims.size()) return -1;
            int li = m_trims[ti].loop_index;
            if (li < 0 || li >= (int)m_loops.size()) return -1;
            return m_loops[li].face_index;
        };
        // Per-face UV region polygons (light sampling) for geometric material-side tests.
        std::vector<char> fpoly_built(nfc, 0);
        std::vector<std::vector<std::vector<std::array<double,2>>>> fpoly_out(nfc), fpoly_in(nfc);
        auto build_fpolys = [&](int fi) {
            if (fpoly_built[fi]) return;
            fpoly_built[fi] = 1;
            for (int li : m_faces[fi].loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                std::vector<std::array<double,2>> poly;
                for (int ti : m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)m_trims.size()) continue;
                    int c2 = m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& pc = m_curves_2d[c2];
                    if (!pc.is_valid()) continue;
                    auto dc = pc.domain();
                    int n = std::min(std::max(pc.cv_count()*2, 8), 256);
                    for (int k = 0; k < n; ++k) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/n);
                        poly.push_back({uv[0], uv[1]});
                    }
                }
                if (poly.size() < 3) continue;
                if (m_loops[li].type == BRepLoopType::Outer) fpoly_out[fi].push_back(std::move(poly));
                else fpoly_in[fi].push_back(std::move(poly));
            }
        };
        auto in_face_uv = [&](int fi, double u, double v) -> bool {
            build_fpolys(fi);
            auto pip = [&](const std::vector<std::array<double,2>>& p) {
                bool inside = false;
                for (size_t i = 0, j = p.size()-1; i < p.size(); j = i++) {
                    if ((p[i][1] > v) != (p[j][1] > v) &&
                        u < (p[j][0]-p[i][0])*(v-p[i][1])/(p[j][1]-p[i][1]) + p[i][0])
                        inside = !inside;
                }
                return inside;
            };
            bool ok = fpoly_out[fi].empty();
            for (auto& op : fpoly_out[fi]) if (pip(op)) { ok = true; break; }
            if (!ok) return false;
            for (auto& ip : fpoly_in[fi]) if (pip(ip)) return false;
            return true;
        };
        // 3D traversal direction of a trim near its pcurve middle, NORMALIZED to material-left:
        // stored direction conventions are not reliable across construction paths (the box x sph
        // seam-straddling half-caps store material-RIGHT trims), so the material side is MEASURED
        // in UV (probe left/right of the traversal against the face's own region) and the
        // direction flipped when material sits right. After normalization, anti-parallel mates
        // <=> same natural-orientation class is a theorem, not a convention.
        auto trim_dir = [&](int ti, const Point* anchor3, Point* mid3_out) -> Vector {
            const BRepTrim& T = m_trims[ti];
            int c2 = T.curve_2d_index; int fi = face_of_trim(ti);
            if (c2 < 0 || c2 >= (int)m_curves_2d.size() || fi < 0) return Vector(0,0,0);
            int si = m_faces[fi].surface_index;
            if (si < 0 || si >= (int)m_surfaces.size()) return Vector(0,0,0);
            const NurbsCurve& pc = m_curves_2d[c2];
            if (!pc.is_valid()) return Vector(0,0,0);
            auto dc = pc.domain();
            double tm = 0.5*(dc.first+dc.second), dt = (dc.second-dc.first)*1e-3;
            // Anti-parallel-mates comparison is only meaningful AT A COMMON POINT: two
            // trims of one edge can parameterize the same closed circle from different
            // seam anchors, putting their midpoints at unrelated positions (the cone-base
            // vs annulus-hole fuse junction measured dp=+1 from tangents half a circle
            // apart). When an anchor is given, measure at this trim's closest approach.
            if (anchor3) {
                double best = 1e300;
                const int NA = 33;
                for (int k = 0; k <= NA; ++k) {
                    double t2s = dc.first + (dc.second - dc.first) * k / NA;
                    Point uv2 = pc.point_at(t2s);
                    double d3 = m_surfaces[si].point_at(uv2[0], uv2[1]).distance(*anchor3);
                    if (d3 < best) { best = d3; tm = t2s; }
                }
                double lo = std::max(dc.first, tm - (dc.second-dc.first)/NA);
                double hi = std::min(dc.second, tm + (dc.second-dc.first)/NA);
                for (int k = 0; k <= 8; ++k) {
                    double t2s = lo + (hi - lo) * k / 8.0;
                    Point uv2 = pc.point_at(t2s);
                    double d3 = m_surfaces[si].point_at(uv2[0], uv2[1]).distance(*anchor3);
                    if (d3 < best) { best = d3; tm = t2s; }
                }
                tm = std::min(std::max(tm, dc.first + dt), dc.second - dt);
            }
            Point a = pc.point_at(tm - dt), b = pc.point_at(tm + dt);
            if (T.reversed) std::swap(a, b);              // traversal order
            Point A = m_surfaces[si].point_at(a[0], a[1]);
            Point B = m_surfaces[si].point_at(b[0], b[1]);
            if (mid3_out) {
                Point muv = pc.point_at(tm);
                *mid3_out = m_surfaces[si].point_at(muv[0], muv[1]);
            }
            Vector d(B[0]-A[0], B[1]-A[1], B[2]-A[2]);
            double tx = b[0]-a[0], ty = b[1]-a[1];
            double tn = std::sqrt(tx*tx+ty*ty);
            if (tn < 1e-15) return Vector(0,0,0);
            auto [su0,su1] = m_surfaces[si].domain(0);
            auto [sv0,sv1] = m_surfaces[si].domain(1);
            Point m = pc.point_at(tm);
            for (double f : {2e-3, 8e-3}) {
                double eu = (su1-su0)*f, ev = (sv1-sv0)*f;
                double lu = m[0] - ty/tn*eu, lv = m[1] + tx/tn*ev;   // left of traversal
                double ru = m[0] + ty/tn*eu, rv = m[1] - tx/tn*ev;   // right of traversal
                bool left_in = in_face_uv(fi, lu, lv);
                bool right_in = in_face_uv(fi, ru, rv);
                if (left_in == right_in) continue;                    // probe straddles another edge
                if (std::getenv("SESSION_SIGN_DBG"))
                    std::fprintf(stderr, "[TDIR] ti=%d fi=%d rev=%d uvT(%.3f,%.3f) L=%d R=%d d3(%.3f,%.3f,%.3f)%s\n",
                                 ti, fi, T.reversed ? 1 : 0, tx/tn, ty/tn, left_in?1:0, right_in?1:0,
                                 d[0], d[1], d[2], left_in ? "" : " FLIP");
                if (!left_in) return Vector(-d[0], -d[1], -d[2]);     // material-right: normalize
                return d;
            }
            return Vector(0,0,0);                                     // ambiguous: skip this trim
        };
        std::vector<std::vector<std::pair<int,int>>> adj(nfc);
        for (const auto& E : m_topology_edges) {
            if ((int)E.trim_indices.size() != 2) continue;
            int t1 = E.trim_indices[0], t2 = E.trim_indices[1];
            int f1 = face_of_trim(t1), f2 = face_of_trim(t2);
            if (f1 < 0 || f2 < 0 || f1 == f2) continue;
            if (!fvalid[f1] || !fvalid[f2]) continue;
            // TOPOLOGICAL rel (SESSION_TOPO_REL): a shared edge's two trims must traverse it in
            // OPPOSITE directions for a consistent 2-manifold, so their `reversed` flags differ when the
            // orientations are the SAME class. This is exactly OCCT's shell-orientation criterion and is
            // well-defined even where the geometric material-left probe fails (a slit-disrupted face,
            // z90's FACE 18). Agrees with the geometric probe on correct shells, so base is unaffected.
            int rel;
            double dp = 0.0;
            static const bool s_topo_rel = (std::getenv("SESSION_TOPO_REL") != nullptr);
            if (s_topo_rel) {
                rel = (m_trims[t1].reversed != m_trims[t2].reversed) ? +1 : -1;
            } else {
            Point mid1(0,0,0);
            Vector d1 = trim_dir(t1, nullptr, &mid1);
            Vector d2 = trim_dir(t2, &mid1, nullptr);   // measure t2 AT t1's point
            double m1 = d1.magnitude(), m2 = d2.magnitude();
            if (m1 < 1e-12 || m2 < 1e-12) continue;
            dp = (d1[0]*d2[0]+d1[1]*d2[1]+d1[2]*d2[2]) / (m1*m2);
            if (std::abs(dp) < 0.5) continue;             // ambiguous tangents: skip this edge
            rel = dp < 0 ? +1 : -1;                        // anti-parallel = same orientation class
            }
            if (std::getenv("SESSION_SIGN_DBG"))
                std::fprintf(stderr, "[ADJ] e=%d f%d<->f%d rel=%+d dp=%+.3f t(%d,%d)\n",
                    (int)(&E - &m_topology_edges[0]), f1, f2, rel, dp, t1, t2);
            adj[f1].push_back({f2, rel});
            adj[f2].push_back({f1, rel});
        }
        std::vector<int> comp(nfc, -1), relsgn(nfc, 1);
        int ncomp = 0;
        for (int s0 = 0; s0 < nfc; ++s0) {
            if (comp[s0] >= 0 || !fvalid[s0]) continue;
            comp[s0] = ncomp; relsgn[s0] = 1;
            std::vector<int> stk{s0};
            while (!stk.empty()) {
                int f = stk.back(); stk.pop_back();
                for (auto& [g, rel] : adj[f]) {
                    int want = rel > 0 ? relsgn[f] : -relsgn[f];
                    if (comp[g] < 0) { comp[g] = ncomp; relsgn[g] = want; stk.push_back(g); }
                }
            }
            ++ncomp;
        }
        // Orientation consistency repair (SESSION_ORIENT_REPAIR): the BFS fixes relsgn along the FIRST
        // spanning-tree edge it reaches each face by, but SKIPS ambiguous-tangent edges (|dp|<0.5) and
        // any later cycle edge -- so if a fuzzy merge left a face linked to the tree by ONE weak/wrong
        // edge while MOST of its edges disagree, that face keeps a wrong relsgn (OCCT: BadOrientation).
        // Relax to the neighbour majority: iteratively flip any face whose relsgn disagrees with more
        // adjacent faces than it agrees with. Gated OFF (load-bearing fn; base uses the tree result).
        if (std::getenv("SESSION_ORIENT_REPAIR")) {
            for (int iter = 0; iter < 6; ++iter) {
                int nflip = 0;
                for (int f = 0; f < nfc; ++f) {
                    if (!fvalid[f] || comp[f] < 0) continue;
                    int agree = 0, disagree = 0;
                    for (auto& [g, rel] : adj[f]) {
                        if (comp[g] != comp[f]) continue;
                        int want = rel > 0 ? relsgn[g] : -relsgn[g];   // relsgn[f] implied by neighbour g
                        if (want == relsgn[f]) ++agree; else ++disagree;
                    }
                    if (disagree > agree) { relsgn[f] = -relsgn[f]; ++nflip; }
                }
                if (std::getenv("SESSION_SIGN_DBG"))
                    std::fprintf(stderr, "[ORIENT-REPAIR] iter %d flipped %d\n", iter, nflip);
                if (nflip == 0) break;
            }
        }
        // FRUSTRATION DIAGNOSTIC (SESSION_FRUST_DBG): after relsgn is finalized, count 2-trim
        // adjacencies whose relsgn violates the manifold rel (odd cycle) -- these are the
        // BadOrientation seeds. Report the faces so the source fuzzy-merged edge can be traced.
        if (std::getenv("SESSION_FRUST_DBG")) {
            int nfrust = 0;
            for (int f = 0; f < nfc; ++f) {
                if (!fvalid[f] || comp[f] < 0) continue;
                for (auto& [g, rel] : adj[f]) {
                    if (g <= f || comp[g] != comp[f]) continue;
                    int want = rel > 0 ? relsgn[f] : -relsgn[f];
                    if (want != relsgn[g]) {
                        ++nfrust;
                        std::fprintf(stderr, "[FRUST] f%d(rs%+d) <-> f%d(rs%+d) rel=%+d VIOLATED\n",
                                     f, relsgn[f], g, relsgn[g], rel);
                    }
                }
            }
            std::fprintf(stderr, "[FRUST] nfc=%d ncomp=%d frustrated_edges=%d\n", nfc, ncomp, nfrust);
        }
        const double sdirs[3][3] = {{0.5773502691, 0.6539124, 0.5023147},
                                    {-0.6172133998, 0.5330976, 0.5788126},
                                    {0.4419417382, -0.5303300859, 0.7237468644}};
        auto parity_in = [&](const Point& q) {
            int votes = 0; const double big = 1e6;
            for (int d = 0; d < 3; ++d) {
                Line ray(q[0], q[1], q[2], q[0]+sdirs[d][0]*big, q[1]+sdirs[d][1]*big, q[2]+sdirs[d][2]*big);
                auto hits = Intersection::ray_mesh(ray, bmesh, 1e-9, true);
                if ((hits.size() % 2) == 1) ++votes;
            }
            return votes >= 2;
        };
        // Per-component probe clouds (own vertices + face interior samples): the GLOBAL
        // supporting-plane test is inconclusive for a disjoint lobe (other lobes' vertices
        // straddle any disk plane) and a cap's own vertices are ALL ON its disk plane --
        // the interior samples supply the bulge side. (box x sph B-A = 6 disjoint caps:
        // 2 of 6 seeded wrong by grazing parity rays -> B-A volume 3.67 vs 10.99.)
        std::vector<std::vector<Point>> comp_pts(std::max(ncomp, 1));
        {
            std::vector<std::set<int>> comp_vids(std::max(ncomp, 1));
            for (const auto& E : m_topology_edges)
                for (int ti : E.trim_indices) {
                    int fi2 = face_of_trim(ti);
                    if (fi2 < 0 || comp[fi2] < 0) continue;
                    for (int vv2 : {E.start_vertex, E.end_vertex})
                        if (vv2 >= 0 && vv2 < (int)m_topology_vertices.size())
                            comp_vids[comp[fi2]].insert(m_topology_vertices[vv2].point_index);
                }
            for (int c2 = 0; c2 < ncomp; ++c2)
                for (int vid : comp_vids[c2])
                    if (vid >= 0 && vid < (int)m_vertices.size()) comp_pts[c2].push_back(m_vertices[vid]);
            for (int fi2 = 0; fi2 < nfc; ++fi2)
                if (fvalid[fi2] && comp[fi2] >= 0) comp_pts[comp[fi2]].push_back(fP3[fi2]);
        }
        // Per-component CAVITY flags: a component is a cavity boundary exactly when it sits
        // INSIDE the union of the OTHER components (contain-cut void shell). Tested with
        // per-component meshes so the component's own shell cannot interfere, and with a
        // per-face majority so knife-edge samples (a lobe's bite wall lies ON the neighbour
        // solid's coplanar wall) cannot flip the verdict: xor's torus bulges were inverted
        // by an enclosure probe that stepped through the shared wall into the neighbour.
        std::vector<char> comp_cavity(std::max(ncomp, 1), 0);
        if (ncomp > 1) {
            std::vector<std::vector<int>> comp_faces(ncomp);
            for (int fi2 = 0; fi2 < nfc; ++fi2)
                if (fvalid[fi2] && comp[fi2] >= 0) comp_faces[comp[fi2]].push_back(fi2);
            std::vector<Mesh> comp_mesh(ncomp);
            for (int c2 = 0; c2 < ncomp; ++c2)
                if (!comp_faces[c2].empty()) comp_mesh[c2] = subset(comp_faces[c2]).mesh();
            const double big = 1e6;
            for (int c2 = 0; c2 < ncomp; ++c2) {
                if (comp_faces[c2].empty()) continue;
                int in_votes = 0, n_votes = 0;
                for (int fi2 : comp_faces[c2]) {
                    const Point& p = fP3[fi2];
                    int votes = 0;
                    for (int d = 0; d < 3; ++d) {
                        Line ray(p[0], p[1], p[2], p[0]+sdirs[d][0]*big, p[1]+sdirs[d][1]*big, p[2]+sdirs[d][2]*big);
                        int nx = 0;
                        for (int c3v = 0; c3v < ncomp; ++c3v) {
                            if (c3v == c2 || comp_faces[c3v].empty()) continue;
                            nx += (int)Intersection::ray_mesh(ray, comp_mesh[c3v], 1e-9, true).size();
                        }
                        if (nx % 2 == 1) ++votes;
                    }
                    if (votes >= 2) ++in_votes;
                    ++n_votes;
                }
                comp_cavity[c2] = (n_votes > 0 && in_votes * 2 > n_votes) ? 1 : 0;
            }
        }
        std::vector<double> score(std::max(ncomp, 1), 0.0);
        for (int fi = 0; fi < nfc; ++fi) {
            if (!fvalid[fi]) continue;
            const NurbsSurface& srf = m_surfaces[m_faces[fi].surface_index];
            double ev = 0.0, wt = 0.0;
            if (is_planar(srf)) {
                double tol_sup = (diag > 0 ? diag : 1.0) * 5e-3;
                // TIER 1 -- component-local supporting plane (own vertices + face interior
                // samples), inverted for cavity components. The interior samples make it
                // STRICT: a cap disk's vertices all lie ON its plane while the material
                // bulges past it with no vertices there -- the vertex-only test then claims
                // the bulge side is empty (box x sph B-A: 2 of 6 caps signed wrong,
                // volume 3.67 vs 10.99).
                if (comp[fi] >= 0 && comp_pts[comp[fi]].size() >= 3) {
                    double cmax = -1e300, cmin = 1e300;
                    for (const auto& vq : comp_pts[comp[fi]]) {
                        double d = (vq[0]-fP3[fi][0])*fNn[fi][0] + (vq[1]-fP3[fi][1])*fNn[fi][1]
                                 + (vq[2]-fP3[fi][2])*fNn[fi][2];
                        cmax = std::max(cmax, d); cmin = std::min(cmin, d);
                    }
                    double ev0 = 0.0;
                    if (cmax <= tol_sup)       ev0 = +1.0;
                    else if (cmin >= -tol_sup) ev0 = -1.0;
                    if (ev0 != 0.0) {
                        ev = comp_cavity[comp[fi]] ? -ev0 : ev0;
                        wt = 5.0;
                    }
                }
                // TIER 2 -- global vertex-only supporting plane: when the whole solid lies on
                // one side of this face's plane, the outward normal is the empty side.
                if (wt == 0.0) {
                    double dmax = -1e300, dmin = 1e300;
                    for (const auto& vq : m_vertices) {
                        double d = (vq[0]-fP3[fi][0])*fNn[fi][0] + (vq[1]-fP3[fi][1])*fNn[fi][1]
                                 + (vq[2]-fP3[fi][2])*fNn[fi][2];
                        dmax = std::max(dmax, d); dmin = std::min(dmin, d);
                    }
                    // Cavity inversion (mirror TIER 1): a void-boundary component orients its
                    // outward normal INTO the void, opposite the supporting-plane empty side.
                    bool cav2 = comp[fi] >= 0 && comp_cavity[comp[fi]];
                    if (dmax <= tol_sup)       { ev = cav2 ? -1.0 : +1.0; wt = 3.0; }
                    else if (dmin >= -tol_sup) { ev = cav2 ? +1.0 : -1.0; wt = 3.0; }
                }
            }
            if (wt == 0.0) {
                // TIER 3 -- signed VOLUME FLUX (exact, ray-free): integrate x.n/3 over the
                // face's own tessellation, orientation tied to the SURFACE normal at the
                // sample point. Summed with relsgn over a closed component this is +-the
                // enclosed volume -- a huge, rotation-invariant margin. The parity double
                // probe it replaces casts fixed skew rays against the self-tessellation:
                // curved thin plates whose sag exceeds eps flip votes with the sampling
                // pattern (chairsROT z15: rotated chair scored -3.5, every osign -1, every
                // angle-method vote inverted; unrotated +6.5 was luck, not correctness).
                Mesh fm = subset(std::vector<int>{fi}).mesh();
                double fl = 0.0;
                double conv = 0.0; double best_d2 = 1e300;
                for (const auto& fkv : fm.face) {
                    const auto& poly = fkv.second;
                    if (poly.size() < 3) continue;
                    const Point a3 = fm.vertex.at(poly[0]).position();
                    for (size_t k2 = 1; k2 + 1 < poly.size(); ++k2) {
                        const Point b3 = fm.vertex.at(poly[k2]).position();
                        const Point c3 = fm.vertex.at(poly[k2+1]).position();
                        double ux = b3[0]-a3[0], uy = b3[1]-a3[1], uz = b3[2]-a3[2];
                        double vx = c3[0]-a3[0], vy = c3[1]-a3[1], vz = c3[2]-a3[2];
                        double nx = (uy*vz-uz*vy)*0.5, ny = (uz*vx-ux*vz)*0.5, nz = (ux*vy-uy*vx)*0.5;
                        double cx = (a3[0]+b3[0]+c3[0])/3.0, cy = (a3[1]+b3[1]+c3[1])/3.0, cz = (a3[2]+b3[2]+c3[2])/3.0;
                        fl += (cx*nx + cy*ny + cz*nz) / 3.0;
                        double dxs = cx-fP3[fi][0], dys = cy-fP3[fi][1], dzs = cz-fP3[fi][2];
                        double d2 = dxs*dxs+dys*dys+dzs*dzs;
                        if (d2 < best_d2) {
                            best_d2 = d2;
                            conv = nx*fNn[fi][0] + ny*fNn[fi][1] + nz*fNn[fi][2];
                        }
                    }
                }
                if (conv < 0.0) fl = -fl;   // align mesh winding with normal_at at the sample
                if (std::abs(fl) > 1e-12) {
                    ev = fl >= 0.0 ? +1.0 : -1.0;
                    // Cavity inversion (mirror TIER 1, absent until now -> the flux tier scored
                    // the sphere-in-box void shell with its own outward normal and ADDED the
                    // void volume: box-cut-inscribed-sphere read 97.5 = box+sphere instead of
                    // 30.5 = box-sphere). A void boundary must point into the void.
                    if (comp[fi] >= 0 && comp_cavity[comp[fi]]) ev = -ev;
                    wt = std::abs(fl);
                }
            }
            if (wt == 0.0) {
                Point pp(fP3[fi][0]+eps*fNn[fi][0], fP3[fi][1]+eps*fNn[fi][1], fP3[fi][2]+eps*fNn[fi][2]);
                Point pm(fP3[fi][0]-eps*fNn[fi][0], fP3[fi][1]-eps*fNn[fi][1], fP3[fi][2]-eps*fNn[fi][2]);
                bool in_p = parity_in(pp), in_m = parity_in(pm);
                if (in_p != in_m) { ev = in_p ? -1.0 : +1.0; wt = 1.0; }
                // inconsistent double probe (one side's rays graze bad triangles, e.g. the
                // inside-cavity probe next to sphere pole fans): keep the OUTWARD-side parity
                // as weak evidence rather than discarding the face's vote entirely -- a lone
                // isolated shell (box-with-cavity's sphere) has no other voters.
                else { ev = in_p ? -1.0 : +1.0; wt = 0.5; }
            }
            if (wt > 0.0) score[comp[fi]] += wt * ev * relsgn[fi];
        }
        for (int fi = 0; fi < nfc; ++fi) {
            if (!fvalid[fi]) continue;
            double flip = score[comp[fi]] < 0 ? -1.0 : 1.0;
            fsign[fi] = relsgn[fi] * flip;
            if (std::getenv("SESSION_SIGN_DBG"))
                std::fprintf(stderr, "[SIGN0] f=%d comp=%d rel=%+d score=%+.1f sign=%+.0f P3=(%.4f,%.4f,%.4f)\n",
                             fi, comp[fi], relsgn[fi], score[comp[fi]], fsign[fi],
                             fP3[fi][0], fP3[fi][1], fP3[fi][2]);
        }
        // FINAL PER-FACE OUTWARD OVERRIDE (SESSION_SHELL_ORIENT): the per-COMPONENT score flip
        // mis-orients faces in SMALL/fragmented components. Rotated freeform sections split the
        // rel-graph into many components (ncomp=7 for z90 cut) because ambiguous-tangent section
        // edges (|dp|<0.5) and fuzzy-merged mates carry no reliable rel, so orientation cannot
        // propagate across the whole shell -> one component's aggregate flip lands wrong -> a
        // single BadOrientation face. Where the orientation-INDEPENDENT ray-parity test at
        // fP3 +/- eps*fNn is CONFIDENT (one side strictly inside, the other strictly outside),
        // trust it PER FACE over the component aggregate. It is a NO-OP where the aggregate is
        // already correct (there fsign == the confident per-face verdict), so every watertight
        // cell -- base/matrix/edge -- is byte-identical; only mis-aggregated freeform faces move.
        if (std::getenv("SESSION_SHELL_ORIENT")) {
            // Only meaningful on a CLOSED shell (the final boolean result): parity_in tests
            // against this brep's own mesh, which is only a solid boundary when watertight. The
            // per-operand osignA/osignB calls run on OPEN subsets (section edges still 1-trim),
            // where the probe is meaningless -- skip them.
            int naked = 0;
            for (const auto& E : m_topology_edges)
                if ((int)E.trim_indices.size() == 1) ++naked;
            if (naked == 0) {
                const double eps_mult[4] = {1.0, 4.0, 16.0, 64.0};
                // Per-face MULTI-SAMPLE outward vote (does fNn point outward?). Single-probe parity is
                // unreliable for a face bordering the thin near-tangent section band (thin/degenerate
                // tessellation mis-counts rays); sample interior points across the face + multi-eps.
                auto outward_vote = [&](int fi, const Point& P, const Vector& N) -> int {
                    for (double em : eps_mult) {
                        double e2 = eps * em;
                        Point pp(P[0]+e2*N[0], P[1]+e2*N[1], P[2]+e2*N[2]);
                        Point pm(P[0]-e2*N[0], P[1]-e2*N[1], P[2]-e2*N[2]);
                        bool in_p = parity_in(pp), in_m = parity_in(pm);
                        if (in_p == in_m) continue;
                        int w = in_p ? -1 : +1;                        // fNn outward iff pp OUTSIDE
                        if (comp[fi] >= 0 && comp_cavity[comp[fi]]) w = -w;
                        return w;
                    }
                    return 0;
                };
                auto face_outward = [&](int fi) -> int {               // +1/-1 majority, 0 undecided
                    int si = m_faces[fi].surface_index;
                    if (si < 0 || si >= (int)m_surfaces.size()) return 0;
                    const NurbsSurface& S = m_surfaces[si];
                    auto [u0,u1] = S.domain(0); auto [v0,v1] = S.domain(1);
                    int vpos = 0, vneg = 0;
                    { int w = outward_vote(fi, fP3[fi], fNn[fi]); if (w>0) ++vpos; else if (w<0) ++vneg; }
                    const int G = 5;
                    for (int iu = 1; iu < G && (vpos+vneg) < 9; ++iu)
                        for (int iv = 1; iv < G && (vpos+vneg) < 9; ++iv) {
                            double u = u0 + (u1-u0)*iu/G, v = v0 + (v1-v0)*iv/G;
                            if (!in_face_uv(fi, u, v)) continue;
                            Vector N = S.normal_at(u, v); double nl = N.magnitude();
                            if (nl < 1e-12) continue;
                            N = Vector(N[0]/nl, N[1]/nl, N[2]/nl);
                            int w = outward_vote(fi, S.point_at(u,v), N);
                            if (w>0) ++vpos; else if (w<0) ++vneg;
                        }
                    int tot = vpos + vneg;
                    if (tot == 0 || std::max(vpos,vneg)*3 < tot*2) return 0;   // undecided / no 2/3 majority
                    return vpos >= vneg ? +1 : -1;
                };
                // PER-COMPONENT re-vote. The geometric rel makes each component INTERNALLY consistent
                // (frustrated_edges=0), so orientation must flip WHOLE COMPONENTS, never single faces
                // (a per-face flip breaks a component's internal manifold consistency and just relocates
                // the BadOrientation). For each component, the correct flip satisfies fsign[fi] =
                // relsgn[fi]*flip == (fNn outward ? +1 : -1), i.e. flip = outward*relsgn; robustly
                // majority-vote it over all the component's decided faces, then apply to the whole comp.
                std::vector<double> cvote(std::max(ncomp,1), 0.0);
                for (int fi = 0; fi < nfc; ++fi) {
                    if (!fvalid[fi] || comp[fi] < 0) continue;
                    int w = face_outward(fi);
                    if (w != 0) cvote[comp[fi]] += (double)(w * relsgn[fi]);
                }
                int nflip = 0;
                for (int fi = 0; fi < nfc; ++fi) {
                    if (!fvalid[fi] || comp[fi] < 0) continue;
                    if (cvote[comp[fi]] == 0.0) continue;              // no evidence: keep the aggregate
                    double flip = cvote[comp[fi]] > 0 ? +1.0 : -1.0;
                    double want = relsgn[fi] * flip;
                    if (want != fsign[fi]) { ++nflip; fsign[fi] = want; }
                }
                if (nflip > 0 && std::getenv("SESSION_FRUST_DBG"))
                    std::fprintf(stderr, "[SHELL-ORIENT] nfc=%d ncomp=%d reflipped %d faces (per-comp)\n",
                                 nfc, ncomp, nflip);
            }
        }
    }

    if (P3s) *P3s = fP3;
    if (Ns) *Ns = fNn;
    return fsign;
}

double BRep::volume() const {
    // MIGRATED to the Green-reduced quadrature (brep_massprops). The legacy body below computes
    // each curved face's flux by a composite Gauss over the TRIM'S UV BOUNDING BOX with a
    // point-in-trim mask. That is exact only for rectangular trims, and it returns the
    // COMPLEMENT of a region PINCHED against a chart boundary: measured on sphere r=2.5 minus
    // cylinder r=1 at the exact pole tangency 23.578178478 deg, where the remaining sphere band
    // touches v=v1 at a single u, its bounding box becomes the whole domain and this returned
    // 15.061775908 for a solid whose true volume is 50.388051 -- brep_massprops returned
    // 50.386790817 on the same BRep. It also differs from the correct integrator by ~2.5e-5
    // generally, which is the scale this campaign has been steering by.
    //
    // brep_massprops reduces the surface integral to a boundary integral over the ACTUAL trim
    // loops (Green), keeps a closed-form exact path for planar faces, is adaptive under a hard
    // evaluation budget so it always terminates, and reproduces the analytic volume AND area of
    // all five primitives to 1e-15 -- with the cylinder and cone CAPS taking the planar
    // closed-form path and measuring exactly pi r^2, so the "planar face bounded by one closed
    // trim integrated over the whole parameter rectangle" trap does not fire on them.
    //
    // CHAIRS CONSTANTS MOVED. The base-chair gate was cut 46.8114 / common 33.4951 / fuse
    // 127.0950; those were produced by the legacy integrator and could not satisfy their own
    // partition identity (cut+common vs vol(A)=80.296862 from validation/OCCT_TRUTH.md was off
    // by 1.20e-4, the fuse identity by 2.85e-5). The migrated numbers are 46.7943 / 33.5025 /
    // 127.0913, whose identities are 7.7e-7 and 6.0e-7 -- 155x and 48x better, both inside 1e-6.
    // Any gate or doc still asserting the old three will report a false regression.
    //
    // SESSION_NO_MASSPROPS_VOL restores the legacy integrator for A/B.
    static const bool s_legacy_vol = (std::getenv("SESSION_NO_MASSPROPS_VOL") != nullptr);
    if (!s_legacy_vol) {
        // SESSION_MP_INTERVALS raises brep_massprops' per-trim initial-interval budget. Any
        // pcurve with more knot spans than that is merged down to it UNIFORMLY, so a densely
        // sampled boundary (the torus pullback emits 4000 points because its spiric sections
        // have no rational form) is integrated as a uniformly decimated version of itself.
        MassPropsOptions o;
        if (const char* mi = std::getenv("SESSION_MP_INTERVALS")) {
            int q = std::atoi(mi);
            if (q > 16) o.max_init_intervals = q;
        }
        return brep_massprops(*this, o).volume;
    }

    // ---- LEGACY bbox-Gauss integrator (opt-in only) ----
    // Divergence theorem: V = (1/3) sum_faces flux_outward, flux = integral of S . n_out dA.
    // This is made independent of stored orientation flags (which differ between primitive
    // constructors and the boolean rebuilder): each face's OUTWARD sign is determined
    // geometrically (step off the face along its natural normal and test inside/outside),
    // and trimmed areas/fluxes are computed with the surface's NATURAL orientation.
    //  - Planar faces: flux_nat = (Q . N_nat) * area, area = |outer loop| - sum|inner loop|
    //    (each loop area is the magnitude of the lifted boundary integral 1/2 |C x C'|).
    //  - Curved faces: composite Gauss of S . (S_u x S_v) over the trim's UV bounding box
    //    (exact for axis-aligned/rectangular trims, e.g. cylinder z-cuts and full domains).
    static const double GN[5] = {-0.9061798459386640, -0.5384693101056831, 0.0,
                                  0.5384693101056831, 0.9061798459386640};
    static const double GW[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                                 0.4786286704993665, 0.2369268850561891};
    auto cross = [](const Vector& a, const Vector& b) {
        return Vector(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
    };
    auto is_planar = [&](const NurbsSurface& s) {
        auto [u0,u1] = s.domain(0); auto [v0,v1] = s.domain(1);
        Vector n0 = s.normal_at(u0 + (u1-u0)*0.5, v0 + (v1-v0)*0.5);
        double uu[3] = {0.25, 0.5, 0.75}, vv[3] = {0.3, 0.6, 0.8};
        for (int i = 0; i < 3; i++) {
            Vector n = s.normal_at(u0 + (u1-u0)*uu[i], v0 + (v1-v0)*vv[i]);
            if (cross(n0, n).magnitude() > 1e-7) return false;
        }
        return true;
    };
    // Lifted boundary vector-area of one loop: 1/2 * closed integral C x C'. Convention-
    // independent: the loop's pcurves are chained head-to-tail by matching UV endpoints
    // (each used forward or reversed as the chain requires), so the stored trim.reversed
    // flags are not relied upon. Exact for exact pcurves (e.g. NURBS circles).
    auto loop_vector_area = [&](const NurbsSurface& srf, const BRepLoop& loop) {
        struct Seg { const NurbsCurve* pc; double t0, t1; Point ps, pe; };
        std::vector<Seg> segs;
        for (int ti : loop.trim_indices) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int c2 = m_trims[ti].curve_2d_index;
            if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
            const NurbsCurve& pc = m_curves_2d[c2];
            auto dc = pc.domain();
            segs.push_back({&pc, dc.first, dc.second, pc.point_at(dc.first), pc.point_at(dc.second)});
        }
        if (segs.empty()) return Vector(0,0,0);
        // Chain by nearest-endpoint greedy walk; each entry is (index, forward?).
        std::vector<std::pair<int,bool>> order;
        std::vector<bool> used(segs.size(), false);
        order.push_back({0, true}); used[0] = true;
        Point tail = segs[0].pe;
        auto d2 = [](const Point& a, const Point& b){ double dx=a[0]-b[0],dy=a[1]-b[1]; return dx*dx+dy*dy; };
        for (size_t k = 1; k < segs.size(); ++k) {
            int best = -1; bool fwd = true; double bd = 1e300;
            for (size_t j = 0; j < segs.size(); ++j) {
                if (used[j]) continue;
                double ds = d2(segs[j].ps, tail), de = d2(segs[j].pe, tail);
                if (ds < bd) { bd = ds; best = (int)j; fwd = true; }
                if (de < bd) { bd = de; best = (int)j; fwd = false; }
            }
            if (best < 0) break;
            used[best] = true; order.push_back({best, fwd});
            tail = fwd ? segs[best].pe : segs[best].ps;
        }
        Vector acc(0,0,0);
        const int NS = 24;
        for (auto& od : order) {
            const Seg& sg = segs[od.first];
            bool fwd = od.second;
            // Integration cells ALIGNED with the pcurve's knot spans: a uniform NS split straddles
            // knots, putting an only-C^{deg-1} integrand under a smooth Gauss rule -- aliasing
            // error that MOVES when a trim is split (the split box x tor spiric discs shifted the
            // volume by 2e-6). Per-span Gauss-5 is EXACT for a non-rational pcurve on an affine
            // patch (polynomial integrand, degree <= 2*deg-1 <= 9); long smooth spans (exact
            // rational arcs, 1-3 spans) keep >= NS cells via per-span subdivision.
            std::vector<double> cells;
            {
                std::vector<double> ks;
                ks.push_back(sg.t0);
                for (int k = 0; k < sg.pc->nurbsknot_count(); ++k) {
                    double t = sg.pc->nurbsknot(k);
                    if (t > sg.t0 + 1e-14 && t < sg.t1 - 1e-14 && t - ks.back() > 1e-14)
                        ks.push_back(t);
                }
                ks.push_back(sg.t1);
                int nspan = (int)ks.size() - 1;
                int sub = std::max(1, (NS + nspan - 1) / nspan);
                for (int i = 0; i < nspan; ++i)
                    for (int q = 0; q < sub; ++q)
                        cells.push_back(ks[i] + (ks[i+1]-ks[i])*q/sub);
                cells.push_back(sg.t1);
            }
            for (int s = 0; s + 1 < (int)cells.size(); s++) {
                double a = cells[s], b = cells[s+1];
                double mid = 0.5*(a+b), half = 0.5*(b-a);
                for (int g = 0; g < 5; g++) {
                    double t = mid + half*GN[g];
                    auto pe = sg.pc->evaluate(t, 1);
                    if (pe.size() < 2) continue;
                    const Vector& uv = pe[0]; Vector duv = pe[1];
                    if (!fwd) duv = Vector(-duv[0], -duv[1], -duv[2]);
                    auto se = srf.evaluate(uv[0], uv[1], 1);
                    if (se.size() < 3) continue;
                    const Vector& S = se[0]; const Vector& Sv = se[1]; const Vector& Su = se[2];
                    Vector Cp(Su[0]*duv[0]+Sv[0]*duv[1], Su[1]*duv[0]+Sv[1]*duv[1], Su[2]*duv[0]+Sv[2]*duv[1]);
                    Vector cr = cross(S, Cp);
                    double w = GW[g]*half;
                    acc = Vector(acc[0]+w*cr[0], acc[1]+w*cr[1], acc[2]+w*cr[2]);
                }
            }
        }
        return Vector(0.5*acc[0], 0.5*acc[1], 0.5*acc[2]);
    };
    std::vector<Point> fP3; std::vector<Vector> fNn;
    std::vector<double> fsign = face_outward_signs(&fP3, &fNn);
    // OPEN-SHELL GUARD. Every flux path below integrates absolute world coordinates, so the
    // divergence theorem only closes when the boundary does: for a closed, consistently
    // oriented shell a translation d leaves the total invariant (the net signed vector area
    // vanishes), but on an OPEN shell it shifts by d.Avec/3 -- the number is origin- and
    // rotation-DEPENDENT and the abs() at the end can dress a large negative garbage value
    // up as a plausible positive one (chairsROT z30 read 187.4 for a true ~55.8). Report it
    // instead of silently returning it: naked-edge count is the exact, cheap certificate.
    if (std::getenv("SESSION_VOL_GUARD")) {
        double gxmn=1e300,gymn=1e300,gzmn=1e300,gxmx=-1e300,gymx=-1e300,gzmx=-1e300;
        for (const auto& p : m_vertices) {
            gxmn=std::min(gxmn,p[0]); gymn=std::min(gymn,p[1]); gzmn=std::min(gzmn,p[2]);
            gxmx=std::max(gxmx,p[0]); gymx=std::max(gymx,p[1]); gzmx=std::max(gzmx,p[2]);
        }
        double gdiag = m_vertices.empty() ? 1.0 :
            std::sqrt((gxmx-gxmn)*(gxmx-gxmn)+(gymx-gymn)*(gymx-gymn)+(gzmx-gzmn)*(gzmx-gzmn));
        if (gdiag <= 0) gdiag = 1.0;
        double gdeg = std::max(gdiag * 1e-7, 1e-12);
        int gnaked = 0;
        for (const auto& e : m_topology_edges) {
            if ((int)e.trim_indices.size() == 2) continue;
            int ci = e.curve_3d_index;
            if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& c = m_curves_3d[ci];
            auto dc = c.domain();
            Point p0 = c.point_at(dc.first);
            double ext = 0.0;
            for (int k = 1; k <= 4; ++k)
                ext = std::max(ext, p0.distance(c.point_at(dc.first + (dc.second-dc.first)*k/4.0)));
            if (ext >= gdeg) ++gnaked;
        }
        if (gnaked)
            std::fprintf(stderr, "[VOL-GUARD] OPEN SHELL: %d naked edges -- volume() is "
                                 "origin-dependent here, do NOT gate on it (diag %.4f)\n",
                         gnaked, gdiag);
    }
    double total = 0.0;
    for (const auto& face : m_faces) {
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];

        int fidx = (int)(&face - &m_faces[0]);
        const Point& P3 = fP3[fidx];
        const Vector& Nnat = fNn[fidx];
        if (Nnat.magnitude() < 1e-12) continue;
        double sign = fsign[fidx];

        // Quadric recognition (sphere/cylinder/torus), hoisted BEFORE the masked Gauss: when the
        // analytic boundary-integral flux below overrides the Gauss result (recognized kind and a
        // non-rect trim), the 384^2 masked Gauss is dead weight and is skipped entirely.
        int q_kind = 0;                     // 1=sphere, 2=cylinder, 3=torus
        Point q_C(0,0,0); Vector q_Zs(0,0,1), q_Xs(1,0,0), q_Ys(0,1,0);
        double q_R = 0, q_r2 = 0;           // sphere/cyl radius R; torus major R minor r2
        if (!is_planar(srf)) {
            auto [su0,su1]=srf.domain(0); auto [sv0,sv1]=srf.domain(1);
            auto dot3=[](const Vector&a,const Vector&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
            double um=0.5*(su0+su1);
            Point Ps=srf.point_at(um,sv0), Pn=srf.point_at(um,sv1), Pm=srf.point_at(um,0.5*(sv0+sv1));
            Vector axis(Pn[0]-Ps[0],Pn[1]-Ps[1],Pn[2]-Ps[2]);
            double Rt = 0.5*axis.magnitude();
            if (Rt > 1e-9) {
                Point Ct(0.5*(Ps[0]+Pn[0]),0.5*(Ps[1]+Pn[1]),0.5*(Ps[2]+Pn[2]));
                Vector Zt(axis[0]/(2*Rt),axis[1]/(2*Rt),axis[2]/(2*Rt));
                bool is_sphere=true;
                for(int i=0;i<=3&&is_sphere;++i)for(int j=0;j<=3&&is_sphere;++j){
                    Point p=srf.point_at(su0+(su1-su0)*i/3.0, sv0+(sv1-sv0)*j/3.0);
                    if (std::abs(p.distance(Ct)-Rt) > Rt*1e-4 + 1e-6) is_sphere=false;
                }
                if (is_sphere) {
                    Vector dm(Pm[0]-Ct[0],Pm[1]-Ct[1],Pm[2]-Ct[2]);
                    double dz=dm[0]*Zt[0]+dm[1]*Zt[1]+dm[2]*Zt[2];
                    Vector Xt(dm[0]-dz*Zt[0],dm[1]-dz*Zt[1],dm[2]-dz*Zt[2]);
                    double xn=Xt.magnitude();
                    if (xn > 1e-12) {
                        q_kind=1; q_C=Ct; q_Zs=Zt; q_R=Rt;
                        q_Xs=Vector(Xt[0]/xn,Xt[1]/xn,Xt[2]/xn); q_Ys=cross(q_Zs,q_Xs);
                    }
                }
            }
            if (!q_kind) {
                Point Ax; Vector Wc; double Rc = 0;
                if (cylinder_of_surface(srf, Ax, Wc, Rc)) {
                    Point Q0 = srf.point_at(su0, 0.5*(sv0+sv1));
                    Vector d0(Q0[0]-Ax[0], Q0[1]-Ax[1], Q0[2]-Ax[2]);
                    double dz0 = dot3(d0, Wc);
                    Vector Xc(d0[0]-dz0*Wc[0], d0[1]-dz0*Wc[1], d0[2]-dz0*Wc[2]);
                    double xl = Xc.magnitude();
                    if (xl > 1e-12) {
                        q_kind=2; q_C=Ax; q_Zs=Wc; q_R=Rc;
                        q_Xs=Vector(Xc[0]/xl,Xc[1]/xl,Xc[2]/xl); q_Ys=cross(q_Zs,q_Xs);
                    }
                }
            }
            if (!q_kind) {
                // torus: tube-circle centres at u0 and its antipode give the centre; a third
                // centre fixes the axis; verified on a grid against (rho-R, z) tube distance.
                double vm2 = 0.5*(sv0+sv1);
                auto tube_c = [&](double u){ Point a=srf.point_at(u,sv0), b=srf.point_at(u,vm2);
                    return Point(0.5*(a[0]+b[0]),0.5*(a[1]+b[1]),0.5*(a[2]+b[2])); };
                Point c1=tube_c(su0), c2=tube_c(0.5*(su0+su1)), c3p=tube_c(su0+0.25*(su1-su0));
                Point Ct(0.5*(c1[0]+c2[0]),0.5*(c1[1]+c2[1]),0.5*(c1[2]+c2[2]));
                Vector a1(c1[0]-Ct[0],c1[1]-Ct[1],c1[2]-Ct[2]);
                Vector a3(c3p[0]-Ct[0],c3p[1]-Ct[1],c3p[2]-Ct[2]);
                Vector Zt = cross(a1, a3);
                double zl = Zt.magnitude(), Rmaj = a1.magnitude();
                if (zl > 1e-12 && Rmaj > 1e-9) {
                    Zt = Vector(Zt[0]/zl, Zt[1]/zl, Zt[2]/zl);
                    double rmin = srf.point_at(su0,sv0).distance(c1);
                    bool is_torus = rmin > 1e-9 && rmin < Rmaj*(1.0-1e-6);
                    for(int i=0;i<=3&&is_torus;++i)for(int j=0;j<=2&&is_torus;++j){
                        Point p=srf.point_at(su0+(su1-su0)*i/3.0, sv0+(sv1-sv0)*j/2.0);
                        Vector w(p[0]-Ct[0],p[1]-Ct[1],p[2]-Ct[2]);
                        double z=dot3(w,Zt);
                        double rho=std::sqrt(std::max(0.0, dot3(w,w)-z*z));
                        if (std::abs(std::hypot(rho-Rmaj, z)-rmin) > rmin*1e-4 + 1e-6) is_torus=false;
                    }
                    if (is_torus) {
                        q_kind=3; q_C=Ct; q_Zs=Zt; q_R=Rmaj; q_r2=rmin;
                        q_Xs=Vector(a1[0]/Rmaj,a1[1]/Rmaj,a1[2]/Rmaj); q_Ys=cross(q_Zs,q_Xs);
                    }
                }
            }
        }

        double flux_nat = 0.0;
        bool curved_rect = false;
        if (is_planar(srf)) {
            double area = 0.0;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                double la = loop_vector_area(srf, m_loops[li]).magnitude();
                area += (m_loops[li].type == BRepLoopType::Outer) ? la : -la;
            }
            // flux_nat = integral S . N_nat dA = (Q . N_nat) * area
            double qn = P3[0]*Nnat[0] + P3[1]*Nnat[1] + P3[2]*Nnat[2];
            flux_nat = qn * area;
            if (std::getenv("SESSION_VOL_DBG2")) {
                for (int li : face.loop_indices) {
                    if (li < 0 || li >= (int)m_loops.size()) continue;
                    for (int ti : m_loops[li].trim_indices) {
                        if (ti < 0 || ti >= (int)m_trims.size()) continue;
                        int c2 = m_trims[ti].curve_2d_index;
                        if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                        const NurbsCurve& pcv = m_curves_2d[c2];
                        auto dcv = pcv.domain();
                        for (int k = 0; k <= 400; ++k) {
                            Point q = pcv.point_at(dcv.first + (dcv.second-dcv.first)*k/400.0);
                            Point q3 = srf.point_at(q[0], q[1]);
                            std::fprintf(stderr, "[LQ] %d %d %d %.9f %.9f %.9f\n",
                                (int)(&face - &m_faces[0]), li, ti, q3[0], q3[1], q3[2]);
                        }
                    }
                }
            }
            // (Supporting-plane outward evidence is folded into the PASS 0 shell-orientation
            // vote above -- the propagated fsign already reflects it, weight 3.)
        } else {
            // Trim UV bounding box AND the trim loops as UV polygons (so a non-rectangular trim --
            // a circular sphere cap, or a band with circular holes -- integrates over the actual
            // region, not the bbox). For a rectangular trim every Gauss point is inside, so this
            // reduces to the plain bbox quadrature (exact for cylinder bands).
            double umin=1e300,umax=-1e300,vmin=1e300,vmax=-1e300;
            std::vector<std::vector<std::array<double,2>>> outer_polys, inner_polys;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                std::vector<std::array<double,2>> poly;
                for (int ti : m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)m_trims.size()) continue;
                    int c2 = m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& pc = m_curves_2d[c2];
                    auto dc = pc.domain();
                    int n = std::min(std::max(pc.cv_count()*4, 12), 1024);
                    for (int i = 0; i < n; ++i) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*i/n);
                        umin=std::min(umin,uv[0]); umax=std::max(umax,uv[0]);
                        vmin=std::min(vmin,uv[1]); vmax=std::max(vmax,uv[1]);
                        poly.push_back({uv[0], uv[1]});
                    }
                }
                if (poly.size() >= 3) {
                    if (m_loops[li].type == BRepLoopType::Outer) outer_polys.push_back(poly);
                    else inner_polys.push_back(poly);
                }
            }
            if (umax <= umin || vmax <= vmin) { auto[a,b]=srf.domain(0); auto[c,d]=srf.domain(1); umin=a;umax=b;vmin=c;vmax=d; }
            auto in_poly = [](double u, double v, const std::vector<std::array<double,2>>& p) {
                bool inside = false;
                for (size_t i = 0, j = p.size()-1; i < p.size(); j = i++) {
                    if ((p[i][1] > v) != (p[j][1] > v) &&
                        u < (p[j][0]-p[i][0])*(v-p[i][1])/(p[j][1]-p[i][1]) + p[i][0])
                        inside = !inside;
                }
                return inside;
            };
            auto in_trim = [&](double u, double v) {
                bool ok = outer_polys.empty();  // no outer loop captured -> treat whole bbox as in
                for (auto& op : outer_polys) if (in_poly(u, v, op)) { ok = true; break; }
                if (!ok) return false;
                for (auto& ip : inner_polys) if (in_poly(u, v, ip)) return false;
                return true;
            };
            // A RECTANGULAR trim (cylinder band, full sphere) needs only NU=24 -- every Gauss
            // point is inside, so the quadrature is exact. A NON-rectangular trim (sphere caps,
            // a band with circular holes) has a curved mask boundary whose staircase error scales
            // ~1/NU, so use a finer grid there. (Sphere cap-cut faces are handled exactly below by
            // the analytic boundary-integral flux; this Gauss is the fallback for other curved faces
            // -- e.g. torus patches trimmed by a section curve, where NU=384 brings the residual
            // staircase under the section-pcurve fit floor (sphere-x-torus common: 1.19e-6 -> 8.6e-7).)
            auto [_du0,_du1] = srf.domain(0); auto [_dv0,_dv1] = srf.domain(1);
            bool rect_trim = inner_polys.empty()
                && std::abs(umin-_du0) < (_du1-_du0)*1e-3 && std::abs(umax-_du1) < (_du1-_du0)*1e-3
                && std::abs(vmin-_dv0) < (_dv1-_dv0)*1e-3 && std::abs(vmax-_dv1) < (_dv1-_dv0)*1e-3;
            if (rect_trim) {
                // NU=24 is exact only when EVERY Gauss point is in-mask. A full-bbox loop that
                // excludes a region (a sphere remainder whose one outer loop spans pole-to-pole
                // around a cap bite) needs the fine masked grid like any curved mask boundary.
                double oa = 0.0;
                for (auto& op2 : outer_polys) {
                    double a2 = 0.0;
                    for (size_t i = 0, j = op2.size()-1; i < op2.size(); j = i++)
                        a2 += op2[j][0]*op2[i][1] - op2[i][0]*op2[j][1];
                    oa += std::abs(0.5*a2);
                }
                if (oa < 0.999*(_du1-_du0)*(_dv1-_dv0)) rect_trim = false;
            }
            curved_rect = rect_trim;
            int NU = rect_trim ? 24 : 384; int NV = NU;
            static const bool s_force_gauss = (std::getenv("SESSION_FORCE_GAUSS") != nullptr);
            if (q_kind != 0 && !rect_trim && !s_force_gauss) NU = 0;  // analytic flux overrides -> skip Gauss
            for (int iu = 0; iu < NU; iu++) {
                double ua=umin+(umax-umin)*iu/NU, ub=umin+(umax-umin)*(iu+1)/NU;
                double um=0.5*(ua+ub), uh=0.5*(ub-ua);
                for (int iv = 0; iv < NV; iv++) {
                    double va2=vmin+(vmax-vmin)*iv/NV, vb=vmin+(vmax-vmin)*(iv+1)/NV;
                    double vm=0.5*(va2+vb), vh=0.5*(vb-va2);
                    for (int gu = 0; gu < 5; gu++) {
                        double u=um+uh*GN[gu];
                        for (int gv = 0; gv < 5; gv++) {
                            double v=vm+vh*GN[gv];
                            if (!in_trim(u, v)) continue;
                            auto d = srf.evaluate(u, v, 1);
                            if (d.size() < 3) continue;
                            Vector nrm = cross(d[2], d[1]);  // Su x Sv (evaluate = [S, Sv, Su])
                            double integ = d[0][0]*nrm[0] + d[0][1]*nrm[1] + d[0][2]*nrm[2];
                            flux_nat += GW[gu]*GW[gv]*uh*vh*integ;
                        }
                    }
                }
            }
        }
        // Analytic sphere boundary-integral flux (exact, ~300x fewer surface evals than the masked
        // Gauss above; used for sphere cap-cut faces -- a sphere minus circular caps). Derivation:
        // flux_nat = integral P.(Su x Sv) du dv. With P = C + R*er (er radial) and Su x Sv || er,
        //   P.(Su x Sv) = C.(Su x Sv) + R^3 cos(phi) theta' phi'.
        // The first term integrates to C . (vector area) = C . (1/2) closed-integral P x dP (boundary,
        // exact). The second, via Green's in (theta,phi): R^3 integral cos(phi) dtheta dphi
        //   = -R^3 closed-integral sin(phi) dtheta = -R^2 closed-integral h dtheta, with h=(P-C).Zs and
        // theta = GEOMETRIC longitude atan2((P-C).Ys,(P-C).Xs) (seam-independent, OCCT EvalPnt2d style).
        // So flux_nat = sum_loops [ C.A_loop - R^2 * H_loop ] -- depends ONLY on the boundary curve's
        // 3D geometry, sidestepping the masked-Gauss region/staircase error.
        double flux_analytic = 0.0; bool have_analytic = false; bool pole_face = false;
        if (!is_planar(srf)) {
            // FLAG-DRIVEN quadric boundary flux (OCCT BRepGProp contract): trims are traversed in
            // stored loop order with their `reversed` flags applied (validated invariant: material
            // LEFT in UV, outer CCW / holes CW). No chaining, no point-in-polygon, no containment:
            // holes subtract via orientation, seam mates cancel pairwise, degenerate edges vanish.
            // Per quadric the UV integral Green-reduces to a closed-form antiderivative carried
            // along the boundary; the trim's exact rational 3D section arc is sampled when the
            // edge carries one (co-orientation resolved at the quarter point).
            auto [su0,su1]=srf.domain(0); auto [sv0,sv1]=srf.domain(1);
            auto dot3=[](const Vector&a,const Vector&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
            const double PI = 3.14159265358979323846;

            int kind = q_kind;                  // 1=sphere, 2=cylinder, 3=torus (hoisted above)
            Point C = q_C; Vector Zs = q_Zs, Xs = q_Xs, Ys = q_Ys;
            double R = q_R, r2 = q_r2;
            if (kind) {
                have_analytic = true;
                // chart handedness (u->angle, v->second coordinate) and natural-normal sign,
                // computed once per face so transformed/mirrored primitives stay correct.
                auto theta_of = [&](const Point& P){ Vector w(P[0]-C[0],P[1]-C[1],P[2]-C[2]);
                    return std::atan2(dot3(w,Ys), dot3(w,Xs)); };
                double uda = theta_of(srf.point_at(su0+0.02*(su1-su0), 0.5*(sv0+sv1)));
                double udb = theta_of(srf.point_at(su0, 0.5*(sv0+sv1)));
                double dth0 = uda-udb; while(dth0> PI)dth0-=2*PI; while(dth0<-PI)dth0+=2*PI;
                double u2th = dth0 >= 0 ? 1.0 : -1.0;
                double CX = C[0]*Xs[0]+C[1]*Xs[1]+C[2]*Xs[2];
                double CY = C[0]*Ys[0]+C[1]*Ys[1]+C[2]*Ys[2];
                double CZ = C[0]*Zs[0]+C[1]*Zs[1]+C[2]*Zs[2];
                double v2h, natout;
                {
                    Point pa = srf.point_at(su0, sv0), pb = srf.point_at(su0, sv1);
                    if (kind == 3) {
                        auto vhat_of = [&](const Point& P){ Vector w(P[0]-C[0],P[1]-C[1],P[2]-C[2]);
                            double z=dot3(w,Zs); double rho=std::sqrt(std::max(0.0,dot3(w,w)-z*z));
                            return std::atan2(z/r2, (rho-R)/r2); };
                        double va = vhat_of(srf.point_at(su0, sv0+0.02*(sv1-sv0)));
                        double vb = vhat_of(pa);
                        double dv0 = va-vb; while(dv0> PI)dv0-=2*PI; while(dv0<-PI)dv0+=2*PI;
                        v2h = dv0 >= 0 ? 1.0 : -1.0;
                    } else {
                        double ha = dot3(Vector(pb[0]-C[0],pb[1]-C[1],pb[2]-C[2]), Zs);
                        double hb = dot3(Vector(pa[0]-C[0],pa[1]-C[1],pa[2]-C[2]), Zs);
                        v2h = (ha - hb) >= 0 ? 1.0 : -1.0;
                    }
                    Point pm2 = srf.point_at(0.5*(su0+su1), 0.5*(sv0+sv1));
                    Vector nm = srf.normal_at(0.5*(su0+su1), 0.5*(sv0+sv1));
                    Vector w(pm2[0]-C[0], pm2[1]-C[1], pm2[2]-C[2]);
                    Vector nref;
                    if (kind == 1) nref = w;                                  // radial
                    else if (kind == 2) { double z=dot3(w,Zs); nref = Vector(w[0]-z*Zs[0],w[1]-z*Zs[1],w[2]-z*Zs[2]); }
                    else { double z=dot3(w,Zs); double rho=std::sqrt(std::max(1e-30,dot3(w,w)-z*z));
                        double px=dot3(w,Xs), py=dot3(w,Ys);
                        Vector er(( px*Xs[0]+py*Ys[0])/rho, (px*Xs[1]+py*Ys[1])/rho, (px*Xs[2]+py*Ys[2])/rho);
                        double cvv=(rho-R)/r2, svv=z/r2;
                        nref = Vector(cvv*er[0]+svv*Zs[0], cvv*er[1]+svv*Zs[1], cvv*er[2]+svv*Zs[2]); }
                    natout = dot3(nm, nref) >= 0 ? 1.0 : -1.0;
                }
                double S = u2th * v2h * natout;   // material-left UV traversal -> natural flux sign

                for (int li : face.loop_indices) {
                    if (li<0||li>=(int)m_loops.size()) continue;
                    // flag-ordered sample stream of the loop boundary
                    std::vector<Point> p3s;
                    std::vector<char> vlock;    // per-sample: 0 free, 1/2 v-locked, 3 pole run
                    std::vector<double> pu;     // pcurve u per sample (theta source on pole runs)
                    auto d2p=[](const Point&a,const Point&b){double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];return dx*dx+dy*dy+dz*dz;};
                    for (int ti : m_loops[li].trim_indices) {
                        if (ti<0||ti>=(int)m_trims.size()) continue;
                        int c2=m_trims[ti].curve_2d_index; if(c2<0||c2>=(int)m_curves_2d.size()) continue;
                        const NurbsCurve& pc=m_curves_2d[c2]; if(!pc.is_valid()) continue;
                        auto dc=pc.domain();
                        bool fwd = !m_trims[ti].reversed;
                        Point uvS2=pc.point_at(fwd?dc.first:dc.second), uvE2=pc.point_at(fwd?dc.second:dc.first);
                        const NurbsCurve* c3=nullptr; double c3a=0,c3b=0; int e=m_trims[ti].edge_index;
                        if(e>=0&&e<(int)m_topology_edges.size()){ int c3i=m_topology_edges[e].curve_3d_index;
                            if(c3i>=0&&c3i<(int)m_curves_3d.size()){ c3=&m_curves_3d[c3i]; auto d3=c3->domain(); c3a=d3.first; c3b=d3.second; } }
                        // A v=const seg's OUTER coordinate is constant (phi/h/vhat all map from v):
                        // LOCK it to the seg-start value so off-surface sampling jitter cannot
                        // couple to the big theta-linear antiderivative, while theta samples still
                        // carry the unwrap/winding through the seg.
                        bool lock_v = false;
                        if (std::abs(uvS2[1]-uvE2[1]) < 1e-9*(sv1-sv0)) {
                            double tmc=0.5*(dc.first+dc.second);
                            if (std::abs(pc.point_at(tmc)[1]-uvS2[1]) < 1e-9*(sv1-sv0)) lock_v = true;
                        }
                        // POLE RUN (sphere): a 3D-degenerate seg is the chart's pole line; theta
                        // is undefined there geometrically, but the chart boundary's theta ALONG the
                        // pole line is the pcurve's u -- carry it so the two seam runs it connects
                        // keep their 2*pi offset (the winding term of every pole-adjacent face).
                        if (kind == 1) {
                            Point pa3=srf.point_at(uvS2[0],uvS2[1]), pb3=srf.point_at(uvE2[0],uvE2[1]);
                            double tmc0=0.5*(dc.first+dc.second); Point uvm0=pc.point_at(tmc0);
                            Point pm30=srf.point_at(uvm0[0],uvm0[1]);
                            double ext=std::max(pa3.distance(pb3), pa3.distance(pm30));
                            if (ext < R*1e-7) {
                                for(int s=0;s<200;++s){ double f=(double)s/200.0;
                                    double tt=fwd?dc.first+(dc.second-dc.first)*f:dc.second-(dc.second-dc.first)*f;
                                    Point uv=pc.point_at(tt);
                                    p3s.push_back(pa3); vlock.push_back(3); pu.push_back(uv[0]); }
                                continue;
                            }
                        }
                        bool use_c3=false, c3_rev=false;
                        // Only an exact RATIONAL section arc is worth sampling directly: polyline
                        // 3D curves lie off the surface (chord sag), and on a torus that off-surface
                        // jitter couples to the large theta-linear antiderivative term as a random
                        // walk (~0.5 flux on a band). Surface-evaluated pcurve samples are always
                        // exactly on-surface. Gate on BOTH endpoints plus a midpoint sanity check:
                        // neighbouring quarters share an endpoint and full halves share both, so a
                        // cross-assigned edge (its curve is the OTHER arc) passes a one-endpoint
                        // gate but sits an arc-diameter away at the middle; the pcurve fallback is
                        // geometrically correct there.
                        if(c3 && c3->is_rational()){
                            Point q0=srf.point_at(uvS2[0],uvS2[1]);
                            Point q1=srf.point_at(uvE2[0],uvE2[1]);
                            Point ca=c3->point_at(c3a), cb=c3->point_at(c3b);
                            bool fw_ok = d2p(q0,ca)<1e-6 && d2p(q1,cb)<1e-6;
                            bool bw_ok = d2p(q0,cb)<1e-6 && d2p(q1,ca)<1e-6;
                            if (fw_ok || bw_ok) {
                                double tmq=0.5*(dc.first+dc.second); Point uvq=pc.point_at(tmq);
                                Point pmq=srf.point_at(uvq[0],uvq[1]);
                                Point cmq=c3->point_at(0.5*(c3a+c3b));
                                double chord = std::max(ca.distance(cb), ca.distance(cmq)) + 1e-12;
                                if (pmq.distance(cmq) < 0.35*chord + 1e-6) {
                                    use_c3=true;
                                    if (fw_ok && bw_ok) {
                                        // closed loop: endpoints carry no direction, quarter decides
                                        double tq=fwd?dc.first+(dc.second-dc.first)*0.25:dc.second-(dc.second-dc.first)*0.25;
                                        Point uvq4=pc.point_at(tq);
                                        Point uvQ=srf.point_at(uvq4[0],uvq4[1]);
                                        double qf=d2p(uvQ, c3->point_at(fwd?c3a+(c3b-c3a)*0.25:c3b-(c3b-c3a)*0.25));
                                        double qr=d2p(uvQ, c3->point_at(fwd?c3b-(c3b-c3a)*0.25:c3a+(c3b-c3a)*0.25));
                                        c3_rev = qr < qf;
                                    } else {
                                        // open arc: cf=(fwd!=c3_rev) must walk c3a->c3b iff fw_ok
                                        c3_rev = fw_ok ? !fwd : fwd;
                                    }
                                }
                            } }
                        // corner error of the boundary trapezoid is O((span/n)^2 * |dG'|): 200
                        // floors sphere fuse remainders at ~9e-6 rel; 2000 puts corners ~1e-8.
                        if (!use_c3 && pc.degree() == 1 && !pc.is_rational() && pc.cv_count() >= 2) {
                            // deg-1 pcurve: its vertices ARE the geometry -- walk them directly
                            // (uniform point_at resampling at a fixed count ALIASES the polyline
                            // back to coarse secants); subdivide segments linearly in UV to keep
                            // the trapezoid corner-error floor (>= ~2000 samples per trim).
                            int k = pc.cv_count();
                            int m = std::max(1, (2000 + k - 2) / (k - 1));
                            bool first_s = true;
                            for (int s = 0; s < k - 1; ++s) {
                                int i0 = fwd ? s : k - 1 - s, i1 = fwd ? s + 1 : k - 2 - s;
                                Point a = pc.get_cv(i0), b = pc.get_cv(i1);
                                for (int q = 0; q < m; ++q) {
                                    double f = (double)q / m;
                                    p3s.push_back(srf.point_at(a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f));
                                    vlock.push_back(lock_v ? (first_s ? 2 : 1) : 0); pu.push_back(0.0);
                                    first_s = false;
                                }
                            }
                            continue;
                        }
                        // resampling a polyline pcurve at FEWER points than its vertex count
                        // aliases it back to coarse secants -- track its density
                        int n = (use_c3 && c3->is_rational()) ? 4000 : std::max(2000, pc.cv_count()*4);
                        for(int s=0;s<n;++s){double f=(double)s/n;
                            if(use_c3){ bool cf=(fwd!=c3_rev);
                                double t3=cf?c3a+(c3b-c3a)*f:c3b-(c3b-c3a)*f;
                                p3s.push_back(c3->point_at(t3)); }
                            else { double tt=fwd?dc.first+(dc.second-dc.first)*f:dc.second-(dc.second-dc.first)*f;
                                Point uv=pc.point_at(tt); p3s.push_back(srf.point_at(uv[0],uv[1])); }
                            vlock.push_back(lock_v ? (s==0 ? 2 : 1) : 0); pu.push_back(0.0); }
                    }
                    if(p3s.size()<3) continue;
                    // Never START the traversal on a pole run: the pole has no longitude, so a
                    // loop anchored there takes an arbitrary theta and lands the pole-exit half a
                    // turn off the seam it must meet; rotating to a normal seg gives every pole a
                    // proper incoming theta to carry from.
                    {
                        size_t s0 = 0;
                        auto at_pole = [&](size_t i){
                            if (kind != 1) return false;
                            double zz=(p3s[i][0]-C[0])*Zs[0]+(p3s[i][1]-C[1])*Zs[1]+(p3s[i][2]-C[2])*Zs[2];
                            return std::abs(zz) >= R*(1.0-1e-8);
                        };
                        while (s0 < vlock.size() && (vlock[s0] == 3 || at_pole(s0))) ++s0;
                        if (s0 > 0 && s0 < vlock.size()) {
                            std::rotate(p3s.begin(), p3s.begin()+s0, p3s.end());
                            std::rotate(vlock.begin(), vlock.begin()+s0, vlock.end());
                            std::rotate(pu.begin(), pu.begin()+s0, pu.end());
                        }
                    }
                    p3s.push_back(p3s.front()); vlock.push_back(vlock.front()); pu.push_back(pu.front());
                    // accumulate the per-kind Green boundary integral over the ordered samples
                    double I=0.0, wind=0.0;
                    double prevTh=0.0, prevH=0.0, prevG=0.0, lockval=0.0; bool first=true;
                    double prev_pu = 0.0; bool prev_pole = false;
                    for(size_t i=0;i<p3s.size();++i){
                        const Point& P=p3s[i];
                        Vector w(P[0]-C[0], P[1]-C[1], P[2]-C[2]);
                        double z = dot3(w,Zs);
                        double th;
                        if (vlock[i] == 3) {
                            th = first ? 0.0
                               : (prev_pole ? prevTh + u2th*(pu[i]-prev_pu)*2.0*PI/(su1-su0)
                                            : prevTh);
                            if(!first) wind += th - prevTh;
                            prev_pu = pu[i]; prev_pole = true;
                        } else if (kind == 1 && std::abs(z) >= R*(1.0-1e-8)) {
                            // sample sits ON the pole (a seam run's pole-side endpoint): longitude
                            // is undefined and G vanishes there (cos phi factor); hold theta so the
                            // unwrap continuity survives.
                            th = first ? 0.0 : prevTh;
                            prev_pole = false;
                        } else {
                            th = std::atan2(dot3(w,Ys), dot3(w,Xs));
                            if(!first){ double dth=th-prevTh;
                                while(dth> PI)dth-=2*PI; while(dth<-PI)dth+=2*PI;
                                th=prevTh+dth; wind+=dth; }
                            prev_pole = false;
                        }
                        if (kind == 1) {
                            // latitude-direction Green: G = int f dtheta with
                            // f = (C.n + R) R^2 cos(phi); pole runs have dphi = 0 by construction,
                            // seam runs carry the theta-winding term -- no pole special-casing.
                            double phi = std::asin(std::max(-1.0, std::min(1.0, z / R)));
                            if(vlock[i] == 2) lockval = phi;
                            else if(vlock[i] == 1) phi = lockval;
                            double cph = std::cos(phi);
                            double G = R*R*cph*( CX*cph*std::sin(th) - CY*cph*std::cos(th)
                                               + (CZ*std::sin(phi) + R)*th );
                            if(!first) I += 0.5*(G+prevG)*(phi-prevH);
                            prevG=G; prevH=phi;
                        } else if (kind == 2) {
                            // v=const seg: the seg's first sample sets the level (the corner step
                            // from the previous seg is real); the rest lock to it so jitter -> 0.
                            if(vlock[i] == 2) lockval = z;
                            else if(vlock[i] == 1) z = lockval;
                            double G = R*R*th + R*(CX*std::sin(th) - CY*std::cos(th));
                            if(!first) I += 0.5*(G+prevG)*(z-prevH);
                            prevG=G; prevH=z;
                        } else {
                            double rho=std::sqrt(std::max(0.0, dot3(w,w)-z*z));
                            double cvv=(rho-R)/r2, svv=z/r2;
                            double vh=std::atan2(svv,cvv);
                            if(!first){ double dvh=vh-prevH;
                                while(dvh> PI)dvh-=2*PI; while(dvh<-PI)dvh+=2*PI; vh=prevH+dvh; }
                            if(vlock[i] == 2) lockval = vh;
                            else if(vlock[i] == 1) vh = lockval;
                            double ring = r2*(R + r2*cvv);
                            double G = ring*( cvv*(CX*std::sin(th) - CY*std::cos(th))
                                            + (CZ*svv + (R + r2*cvv)*cvv + r2*svv*svv)*th );
                            if(!first) I += 0.5*(G+prevG)*(vh-prevH);
                            prevG=G; prevH=vh;
                        }
                        prevTh=th; first=false;
                    }
                    flux_analytic += S * I;
                }
            }
        }
        // Chart integrals are exact for recognized quadrics with REAL boundary loops; a
        // full-domain rect face's seam boundaries cancel pairwise (I = 0), so the rect Gauss
        // stays authoritative there (rect_trim is coverage-gated: masked faces never qualify).
        double gauss_val = flux_nat;
        // SESSION_NO_ANFLUX: fall back to the numeric Gauss flux even where the closed-form
        // chart integral is available. Diagnostic for the 9.4e-05 cluster -- if the error
        // VANISHES with the analytic path disabled, the closed-form formula (not the
        // quadrature) is what is wrong, which is the opposite of the standing assumption.
        static const bool s_no_anflux = (std::getenv("SESSION_NO_ANFLUX") != nullptr);
        if (have_analytic && !curved_rect && !pole_face && !s_no_anflux) flux_nat = flux_analytic;
        if (std::getenv("SESSION_VOL_DBG"))
            std::fprintf(stderr,"[VOLDBG] f=%ld planar=%d haveA=%d rect=%d pole=%d sign=%.0f gauss=%.6f an=%.6f\n",
                (long)(&face-&m_faces[0]), is_planar(srf)?1:0, have_analytic?1:0, curved_rect?1:0, pole_face?1:0, sign, gauss_val, flux_analytic);
        total += sign * flux_nat;
    }
    return std::abs(total) / 3.0;
}


///////////////////////////////////////////////////////////////////////////////////////////
// Building
///////////////////////////////////////////////////////////////////////////////////////////

int BRep::add_surface(const NurbsSurface& srf) {
    m_surfaces.push_back(srf);
    return (int)m_surfaces.size() - 1;
}

int BRep::add_curve_3d(const NurbsCurve& crv) {
    m_curves_3d.push_back(crv);
    return (int)m_curves_3d.size() - 1;
}

int BRep::add_curve_2d(const NurbsCurve& crv) {
    m_curves_2d.push_back(crv);
    return (int)m_curves_2d.size() - 1;
}

int BRep::add_vertex(const Point& pt) {
    m_vertices.push_back(pt);
    return (int)m_vertices.size() - 1;
}

int BRep::add_edge(int curve_3d_idx, int start_vertex, int end_vertex) {
    BRepEdge e;
    e.curve_3d_index = curve_3d_idx;
    e.start_vertex = start_vertex;
    e.end_vertex = end_vertex;
    m_topology_edges.push_back(e);
    return (int)m_topology_edges.size() - 1;
}

int BRep::add_trim(int curve_2d_idx, int edge_idx, int loop_idx, bool reversed, BRepTrimType type) {
    BRepTrim t;
    t.curve_2d_index = curve_2d_idx;
    t.edge_index = edge_idx;
    t.loop_index = loop_idx;
    t.reversed = reversed;
    t.type = type;
    int idx = (int)m_trims.size();
    m_trims.push_back(t);
    if (loop_idx >= 0 && loop_idx < (int)m_loops.size())
        m_loops[loop_idx].trim_indices.push_back(idx);
    if (edge_idx >= 0 && edge_idx < (int)m_topology_edges.size())
        m_topology_edges[edge_idx].trim_indices.push_back(idx);
    return idx;
}

int BRep::add_loop(int face_idx, BRepLoopType type) {
    BRepLoop l;
    l.face_index = face_idx;
    l.type = type;
    int idx = (int)m_loops.size();
    m_loops.push_back(l);
    if (face_idx >= 0 && face_idx < (int)m_faces.size())
        m_faces[face_idx].loop_indices.push_back(idx);
    return idx;
}

int BRep::add_face(int surface_idx, bool reversed) {
    BRepFace f;
    f.surface_index = surface_idx;
    f.reversed = reversed;
    m_faces.push_back(f);
    return (int)m_faces.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Splitting
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

inline long long q6(double x) { return (long long)std::llround(x * 1000000.0); }

std::pair<std::array<double, 3>, std::array<double, 3>> aabb_from_surface(const NurbsSurface& srf) {
    const int n = 6;
    auto du = srf.domain(0);
    auto dv = srf.domain(1);
    std::array<double, 3> lo = {1e30, 1e30, 1e30};
    std::array<double, 3> hi = {-1e30, -1e30, -1e30};
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) {
            double u = du.first + (du.second - du.first) * i / n;
            double v = dv.first + (dv.second - dv.first) * j / n;
            Point p = srf.point_at(u, v);
            for (int k = 0; k < 3; ++k) {
                if (p[k] < lo[k]) lo[k] = p[k];
                if (p[k] > hi[k]) hi[k] = p[k];
            }
        }
    }
    return {lo, hi};
}

std::pair<std::array<double, 3>, std::array<double, 3>> aabb_from_curve(const NurbsCurve& crv) {
    const int n = 16;
    auto dc = crv.domain();
    std::array<double, 3> lo = {1e30, 1e30, 1e30};
    std::array<double, 3> hi = {-1e30, -1e30, -1e30};
    for (int i = 0; i <= n; ++i) {
        Point p = crv.point_at(dc.first + (dc.second - dc.first) * i / n);
        for (int k = 0; k < 3; ++k) {
            if (p[k] < lo[k]) lo[k] = p[k];
            if (p[k] > hi[k]) hi[k] = p[k];
        }
    }
    return {lo, hi};
}

bool aabb_overlap(const std::pair<std::array<double, 3>, std::array<double, 3>>& a,
                  const std::pair<std::array<double, 3>, std::array<double, 3>>& b, double m) {
    for (int k = 0; k < 3; ++k) {
        if (a.first[k] - m > b.second[k] || b.first[k] - m > a.second[k]) return false;
    }
    return true;
}

// Even-odd point-in-trim test for a face's UV loops (inside outer, outside every inner hole).
bool uv_in_polys(double u, double v,
                 const std::vector<std::vector<std::array<double, 2>>>& outer,
                 const std::vector<std::vector<std::array<double, 2>>>& inner) {
    auto in_poly = [](double u, double v, const std::vector<std::array<double, 2>>& p) {
        bool inside = false;
        for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
            if ((p[i][1] > v) != (p[j][1] > v) &&
                u < (p[j][0] - p[i][0]) * (v - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0])
                inside = !inside;
        }
        return inside;
    };
    bool ok = outer.empty();
    for (const auto& op : outer) if (in_poly(u, v, op)) { ok = true; break; }
    if (!ok) return false;
    for (const auto& ip : inner) if (in_poly(u, v, ip)) return false;
    return true;
}

}  // namespace

BRep BRep::split_by_plane(const Plane& plane, double tolerance) const {
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        for (auto& pr : Intersection::surface_plane_uv(srf, plane, tolerance)) out.push_back(pr.second);
        return out;
    });
}

BRep BRep::split_by_surface(const NurbsSurface& cutter, double tolerance) const {
    auto cutter_bb = aabb_from_surface(cutter);
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        auto srf_bb = aabb_from_surface(srf);
        double margin = std::max({srf_bb.second[0] - srf_bb.first[0],
                                  srf_bb.second[1] - srf_bb.first[1],
                                  srf_bb.second[2] - srf_bb.first[2]}) * 1e-3;
        if (!aabb_overlap(srf_bb, cutter_bb, margin)) return out;
        for (auto& tr : Intersection::surface_surface(srf, cutter, tolerance)) out.push_back(std::get<1>(tr));
        return out;
    });
}

BRep BRep::split_by_curves(const std::vector<NurbsCurve>& curves, double tolerance) const {
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> curve_bbs;
    for (auto& c : curves) curve_bbs.push_back(aabb_from_curve(c));
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        auto srf_bb = aabb_from_surface(srf);
        double margin = std::max({srf_bb.second[0] - srf_bb.first[0],
                                  srf_bb.second[1] - srf_bb.first[1],
                                  srf_bb.second[2] - srf_bb.first[2]}) * 1e-3;
        for (size_t ci = 0; ci < curves.size(); ++ci) {
            if (!aabb_overlap(srf_bb, curve_bbs[ci], margin)) continue;
            for (auto& pc : Closest::surface_curve(srf, curves[ci], 0.0, 0.0, tolerance)) out.push_back(pc);
        }
        return out;
    });
}

BRep BRep::split_by_line(const Line& line, double tolerance) const {
    std::vector<Point> pts = {line.start(), line.end()};
    NurbsCurve crv = NurbsCurve::create(false, 1, pts);
    return split_by_curves({crv}, tolerance);
}

BRep BRep::subset(const std::vector<int>& face_indices, std::map<int, int>* edge_remap) const {
    BRep sub;
    sub.name = name;
    std::map<int, int> s_map, c2_map, c3_map, v_map, e_map;

    auto map_surface = [&](int i) -> int {
        auto it = s_map.find(i);
        if (it != s_map.end()) return it->second;
        int x = sub.add_surface(m_surfaces[i]);
        s_map[i] = x;
        return x;
    };
    auto map_c2 = [&](int i) -> int {
        if (i < 0 || i >= (int)m_curves_2d.size()) return -1;
        auto it = c2_map.find(i);
        if (it != c2_map.end()) return it->second;
        int x = sub.add_curve_2d(m_curves_2d[i]);
        c2_map[i] = x;
        return x;
    };
    auto map_vertex = [&](int i) -> int {
        if (i < 0 || i >= (int)m_topology_vertices.size()) return -1;
        auto it = v_map.find(i);
        if (it != v_map.end()) return it->second;
        int idx = sub.add_vertex(m_vertices[m_topology_vertices[i].point_index]);
        BRepVertex tv;
        tv.point_index = idx;
        sub.m_topology_vertices.push_back(tv);
        int nv = (int)sub.m_topology_vertices.size() - 1;
        v_map[i] = nv;
        return nv;
    };
    auto map_edge = [&](int i) -> int {
        if (i < 0 || i >= (int)m_topology_edges.size()) return -1;
        auto it = e_map.find(i);
        if (it != e_map.end()) return it->second;
        const BRepEdge& e = m_topology_edges[i];
        int ci3 = -1;
        if (e.curve_3d_index >= 0 && e.curve_3d_index < (int)m_curves_3d.size()) {
            auto c3 = c3_map.find(e.curve_3d_index);
            if (c3 != c3_map.end()) ci3 = c3->second;
            else { ci3 = sub.add_curve_3d(m_curves_3d[e.curve_3d_index]); c3_map[e.curve_3d_index] = ci3; }
        }
        int sv = map_vertex(e.start_vertex);
        int ev = map_vertex(e.end_vertex);
        int ne = sub.add_edge(ci3, sv, ev);
        e_map[i] = ne;
        return ne;
    };

    for (int fi : face_indices) {
        const BRepFace& face = m_faces[fi];
        int si = map_surface(face.surface_index);
        int new_fi = sub.add_face(si, face.reversed);
        for (int li : face.loop_indices) {
            const BRepLoop& lp = m_loops[li];
            int new_li = sub.add_loop(new_fi, lp.type);
            for (int ti : lp.trim_indices) {
                const BRepTrim& trim = m_trims[ti];
                int ci2 = map_c2(trim.curve_2d_index);
                int ei = map_edge(trim.edge_index);
                sub.add_trim(ci2, ei, new_li, trim.reversed, trim.type);
            }
        }
    }
    for (int ei = 0; ei < (int)sub.m_topology_edges.size(); ++ei) {
        int sv = sub.m_topology_edges[ei].start_vertex;
        int ev = sub.m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)sub.m_topology_vertices.size())
            sub.m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)sub.m_topology_vertices.size())
            sub.m_topology_vertices[ev].edge_indices.push_back(ei);
    }
    if (edge_remap) *edge_remap = e_map;
    return sub;
}

std::vector<BRep> BRep::split_by_plane_pieces(const Plane& plane, double tolerance) const {
    BRep whole = split_by_plane(plane, tolerance);
    const Point& o = plane.origin();
    const Vector& n = plane.z_axis();
    std::vector<int> pos, neg;
    for (int fi = 0; fi < (int)whole.m_faces.size(); ++fi) {
        const BRepFace& face = whole.m_faces[fi];
        const NurbsSurface& srf = whole.m_surfaces[face.surface_index];
        double sx = 0.0, sy = 0.0, sz = 0.0;
        int cnt = 0;
        for (int li : face.loop_indices) {
            const BRepLoop& lp = whole.m_loops[li];
            if (lp.type != BRepLoopType::Outer) continue;
            for (int ti : lp.trim_indices) {
                const NurbsCurve& pc = whole.m_curves_2d[whole.m_trims[ti].curve_2d_index];
                auto dc = pc.domain();
                for (int k = 0; k < 8; ++k) {
                    Point uv = pc.point_at(dc.first + (dc.second - dc.first) * k / 8.0);
                    Point p = srf.point_at(uv[0], uv[1]);
                    sx += p[0]; sy += p[1]; sz += p[2]; cnt += 1;
                }
            }
        }
        if (cnt == 0) continue;
        double cx = sx / cnt, cy = sy / cnt, cz = sz / cnt;
        double d = (cx - o[0]) * n[0] + (cy - o[1]) * n[1] + (cz - o[2]) * n[2];
        if (d >= 0.0) pos.push_back(fi); else neg.push_back(fi);
    }
    std::vector<BRep> pieces;
    if (!pos.empty()) pieces.push_back(whole.subset(pos));
    if (!neg.empty()) pieces.push_back(whole.subset(neg));
    return pieces;
}

void BRep::append_brep(const BRep& other) {
    // Disjoint assembly: concatenate the other BRep's geometry pools and topology tables
    // with index offsets. No sewing -- each side keeps its own (already mated) edges, so
    // two solids that touch only along coincident section edges stay two watertight shells.
    int voff=(int)m_vertices.size(), tvoff=(int)m_topology_vertices.size();
    int soff=(int)m_surfaces.size(), c2off=(int)m_curves_2d.size();
    int c3off=(int)m_curves_3d.size(), eoff=(int)m_topology_edges.size();
    int loff=(int)m_loops.size(), foff=(int)m_faces.size(), toff=(int)m_trims.size();
    for (auto& p : other.m_vertices) m_vertices.push_back(p);
    for (auto& s : other.m_surfaces) m_surfaces.push_back(s);
    for (auto& c : other.m_curves_2d) m_curves_2d.push_back(c);
    for (auto& c : other.m_curves_3d) m_curves_3d.push_back(c);
    for (auto tv : other.m_topology_vertices) { tv.point_index += voff; tv.edge_indices.clear(); m_topology_vertices.push_back(tv); }
    for (auto e : other.m_topology_edges) {
        if (e.curve_3d_index>=0) e.curve_3d_index += c3off;
        if (e.start_vertex>=0) e.start_vertex += tvoff;
        if (e.end_vertex>=0) e.end_vertex += tvoff;
        e.trim_indices.clear();
        m_topology_edges.push_back(e);
    }
    for (auto t : other.m_trims) {
        if (t.curve_2d_index>=0) t.curve_2d_index += c2off;
        if (t.edge_index>=0) t.edge_index += eoff;
        if (t.loop_index>=0) t.loop_index += loff;
        m_trims.push_back(t);
    }
    for (auto lp : other.m_loops) {
        for (auto& ti : lp.trim_indices) ti += toff;
        if (lp.face_index>=0) lp.face_index += foff;
        m_loops.push_back(lp);
    }
    for (auto f : other.m_faces) {
        if (f.surface_index>=0) f.surface_index += soff;
        for (auto& li : f.loop_indices) li += loff;
        m_faces.push_back(f);
    }
    for (auto& e : m_topology_edges) e.trim_indices.clear();
    for (int ti=0; ti<(int)m_trims.size(); ++ti) {
        int ei = m_trims[ti].edge_index;
        if (ei>=0 && ei<(int)m_topology_edges.size()) m_topology_edges[ei].trim_indices.push_back(ti);
    }
}

BRep BRep::split_by_brep(const BRep& cutter, double tolerance, bool imported_freeform,
                         const std::vector<std::vector<NurbsCurve>>* pre_cuts,
                         const SectionScaffold* scaf, bool scaf_is_A,
                         std::map<int, std::array<int, 3>>* sec_edges_out,
                         std::vector<int>* face_src_out,
                         const std::vector<std::vector<NurbsCurve>>* extra_cuts,
                         std::map<int, std::array<double, 3>>* sec_spans_out,
                       const SharedEdgePool* pool) const {
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> cutter_bbs;
    for (const auto& cs : cutter.m_surfaces) cutter_bbs.push_back(aabb_from_surface(cs));

    // Trim-aware cutting (gated): intersect against the cutter's TRIMMED faces and clip each
    // target section to the face's trim loops, so a section that leaves one cutter patch's
    // parametric rectangle stops at the true trim boundary (where it continues onto the adjacent
    // cutter face) instead of ending in the target's interior. Legacy path (gate off) loops raw
    // cutter SURFACES and is byte-identical.
    static const bool s_trimcut = (std::getenv("SESSION_TRIM_CUT") != nullptr
                                   || std::getenv("SESSION_BOOL_SHARED_EDGES") != nullptr);
    struct FaceTrim {
        int surf_index = -1;
        std::vector<std::vector<std::array<double, 2>>> outer, inner;
    };
    std::vector<FaceTrim> face_trims;
    if (s_trimcut) {
        for (const auto& cf : cutter.m_faces) {
            FaceTrim ft;
            ft.surf_index = cf.surface_index;
            for (int li : cf.loop_indices) {
                if (li < 0 || li >= (int)cutter.m_loops.size()) continue;
                const BRepLoop& bl = cutter.m_loops[li];
                std::vector<std::array<double, 2>> poly;
                for (int ti : bl.trim_indices) {
                    if (ti < 0 || ti >= (int)cutter.m_trims.size()) continue;
                    int c2 = cutter.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)cutter.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = cutter.m_curves_2d[c2];
                    auto dc = pc.domain();
                    int ns = std::max(pc.cv_count() * 2, 24);
                    for (int i = 0; i < ns; ++i) {
                        Point uv = pc.point_at(dc.first + (dc.second - dc.first) * i / ns);
                        poly.push_back({uv[0], uv[1]});
                    }
                }
                if (poly.size() >= 3) {
                    if (bl.type == BRepLoopType::Inner) ft.inner.push_back(std::move(poly));
                    else ft.outer.push_back(std::move(poly));
                }
            }
            face_trims.push_back(std::move(ft));
        }
    }

    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        auto srf_bb = aabb_from_surface(srf);
        double margin = std::max({srf_bb.second[0] - srf_bb.first[0],
                                  srf_bb.second[1] - srf_bb.first[1],
                                  srf_bb.second[2] - srf_bb.first[2]}) * 1e-3;
        // UV-space dedup across cutter faces: several cutter faces can SHARE one underlying
        // surface (xor's B-A operand = 6 caps on ONE sphere), each contributing the SAME
        // section pcurve; near-identical duplicate cuts shatter the arrangement into
        // slivers (one wall face split into 62 parts).
        auto du_r = srf.domain(0); auto dv_r = srf.domain(1);
        double dup_tol = std::max(du_r.second - du_r.first, dv_r.second - dv_r.first) * 1e-6;
        auto uv_pts = [](const NurbsCurve& pc, int n) {
            std::vector<Point> ps(n + 1);
            auto dc = pc.domain();
            for (int i = 0; i <= n; ++i) ps[i] = pc.point_at(dc.first + (dc.second - dc.first) * i / n);
            return ps;
        };
        auto pt_to_poly = [](const Point& p, const std::vector<Point>& poly) {
            double best = 1e300;
            for (size_t j = 0; j + 1 < poly.size(); ++j) {
                double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1];
                double L2 = ex*ex + ey*ey;
                double t = L2 > 1e-30 ? ((p[0]-poly[j][0])*ex + (p[1]-poly[j][1])*ey) / L2 : 0.0;
                t = std::min(std::max(t, 0.0), 1.0);
                double dx = p[0]-poly[j][0]-t*ex, dy = p[1]-poly[j][1]-t*ey;
                best = std::min(best, dx*dx + dy*dy);
            }
            return std::sqrt(best);
        };
        std::vector<std::vector<Point>> kept_polys;
        auto push_deduped = [&](const NurbsCurve& pc) {
            auto cand = uv_pts(pc, 16);
            for (auto& kp : kept_polys) {
                bool all_on = true;
                for (auto& q : cand) if (pt_to_poly(q, kp) > dup_tol) { all_on = false; break; }
                if (all_on) return;
            }
            kept_polys.push_back(uv_pts(pc, 64));
            out.push_back(pc);
        };
        if (s_trimcut) {
            // SSI once per unique cutter surface (several faces can share one surface); clip
            // each section to each face's trim loops.
            std::map<int, std::vector<NurbsCurve>> ssi_cache;
            auto get_ssi = [&](int si) -> const std::vector<NurbsCurve>& {
                auto it = ssi_cache.find(si);
                if (it != ssi_cache.end()) return it->second;
                auto& v = ssi_cache[si];
                v = Intersection::cut_curves_on_surface(srf, cutter.m_surfaces[si], tolerance);
                return v;
            };
            // Keep the spans of a target section pcurve whose lifted 3D point projects INSIDE the
            // cutter face's UV trim; endpoints are bisected onto the trim boundary so fragments
            // from adjacent cutter faces meet at the shared crossing point.
            auto clip_to_face = [&](const NurbsCurve& pc, const NurbsSurface& csurf, const FaceTrim& ft) {
                int n = std::max(pc.cv_count() * 4, 24);
                auto dc = pc.domain();
                double d0 = dc.first, d1 = dc.second;
                auto at = [&](int i) { return d0 + (d1 - d0) * i / n; };
                auto flag = [&](double t) -> bool {
                    Point uv = pc.point_at(t);
                    Point p3 = srf.point_at(uv[0], uv[1]);
                    auto pr = Closest::surface_point(csurf, p3, 0.0, 0.0, 0.0, 0.0);
                    return uv_in_polys(std::get<0>(pr), std::get<1>(pr), ft.outer, ft.inner);
                };
                auto refine = [&](double t_in, double t_out) -> double {
                    double a = t_in, b = t_out;
                    for (int k = 0; k < 20; ++k) { double m1 = 0.5 * (a + b); if (flag(m1)) a = m1; else b = m1; }
                    return 0.5 * (a + b);
                };
                std::vector<char> fl(n + 1);
                for (int i = 0; i <= n; ++i) fl[i] = flag(at(i)) ? 1 : 0;
                int i = 0;
                while (i <= n) {
                    if (fl[i]) {
                        int j = i;
                        while (j + 1 <= n && fl[j + 1]) ++j;
                        double ta = (i == 0) ? d0 : refine(at(i), at(i - 1));
                        double tb = (j == n) ? d1 : refine(at(j), at(j + 1));
                        if (tb - ta > (d1 - d0) * 1e-6) {
                            NurbsCurve piece = pc;
                            if (piece.trim(ta, tb) && piece.is_valid()) push_deduped(piece);
                        }
                        i = j + 1;
                    } else {
                        ++i;
                    }
                }
            };
            for (const auto& ft : face_trims) {
                int si = ft.surf_index;
                if (si < 0 || si >= (int)cutter.m_surfaces.size()) continue;
                if (!aabb_overlap(srf_bb, cutter_bbs[si], margin)) continue;
                for (const auto& pc : get_ssi(si)) clip_to_face(pc, cutter.m_surfaces[si], ft);
            }
            // Chain section fragments meeting at a common srf-UV endpoint into continuous cuts:
            // a section spanning several cutter faces arrives as one fragment per cutter patch,
            // each ending where the target crosses that patch's shared edge. Joined end-to-end
            // they form boundary→boundary (or closed) cuts that survive the arrangement's
            // valence-1 dangling-edge prune; left fragmented they are pruned and the face never
            // splits.
            static const bool s_tdbg = (std::getenv("SESSION_TRIM_DBG") != nullptr);
            size_t frags_in = out.size();
            size_t frags_joined = 0;
            if (out.size() > 1) {
                double diag = 0.0;
                for (int k = 0; k < 3; ++k) diag += (srf_bb.second[k] - srf_bb.first[k]) * (srf_bb.second[k] - srf_bb.first[k]);
                diag = std::sqrt(diag);
                double tol3 = std::max(tolerance * 50.0, diag * 5e-4);   // bridge the marcher endpoint gap only
                if (const char* e = std::getenv("SESSION_TRIM_TOL3")) tol3 = diag * std::atof(e);
                double tol3_2 = tol3 * tol3;
                double cos_thresh = 0.5;   // reject joins bending more than ~60 deg (3D)
                if (const char* e = std::getenv("SESSION_TRIM_COS")) cos_thresh = std::atof(e);
                // Fragment endpoints and 3D tangents (join continuity is judged in 3D, not the
                // target's distorting UV space): a true multi-cutter-face section continuation is
                // G1 across the shared cutter edge; a corner or two distinct curves touching are not.
                struct Frag {
                    std::vector<Point> uv;       // pcurve samples in target UV
                    Point p3f, p3b;              // 3D endpoints
                    std::array<double, 3> df, db; // outward 3D unit tangents at front/back
                };
                auto lift = [&](const Point& uv) { return srf.point_at(uv[0], uv[1]); };
                auto udir3 = [](const Point& a, const Point& b) -> std::array<double, 3> {
                    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2], L = std::sqrt(dx*dx+dy*dy+dz*dz);
                    return L > 1e-30 ? std::array<double,3>{dx/L,dy/L,dz/L} : std::array<double,3>{0.0,0.0,0.0};
                };
                std::vector<Frag> fr;
                fr.reserve(out.size());
                for (const auto& pc : out) {
                    auto dc = pc.domain();
                    int np = std::max(pc.cv_count() * 8, 64);   // dense: chained deg-1 stays within tol of the true section
                    Frag f;
                    f.uv.resize(np + 1);
                    for (int i = 0; i <= np; ++i)
                        f.uv[i] = pc.point_at(dc.first + (dc.second - dc.first) * i / np);
                    f.p3f = lift(f.uv.front()); f.p3b = lift(f.uv.back());
                    f.df = udir3(f.p3f, lift(f.uv[1]));
                    f.db = udir3(f.p3b, lift(f.uv[f.uv.size() - 2]));
                    fr.push_back(std::move(f));
                }
                auto d2 = [](const Point& a, const Point& b) {
                    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2]; return dx*dx+dy*dy+dz*dz;
                };
                auto smooth = [&](const std::array<double,3>& a, const std::array<double,3>& b) {
                    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] < -cos_thresh;   // outward tangents anti-parallel
                };
                if (s_tdbg) {
                    auto dot3 = [](const std::array<double,3>& a, const std::array<double,3>& b){ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; };
                    for (size_t a = 0; a < fr.size(); ++a) {
                        double best = 1e300; int who = -1, pr = -1;
                        for (size_t b = 0; b < fr.size(); ++b) {
                            if (a == b) continue;
                            double gg[4] = { d2(fr[a].p3b, fr[b].p3f), d2(fr[a].p3b, fr[b].p3b),
                                             d2(fr[a].p3f, fr[b].p3b), d2(fr[a].p3f, fr[b].p3f) };
                            for (int q = 0; q < 4; q++) if (gg[q] < best) { best = gg[q]; who = (int)b; pr = q; }
                        }
                        double dt = 2;
                        if (who >= 0) {
                            const Frag& B = fr[who];
                            if (pr == 0) dt = dot3(fr[a].db, B.df);
                            else if (pr == 1) dt = dot3(fr[a].db, B.db);
                            else if (pr == 2) dt = dot3(fr[a].df, B.db);
                            else dt = dot3(fr[a].df, B.df);
                        }
                        std::fprintf(stderr, "[TRIMGAP] frag %zu nearest %d pair=%d gap=%.5f dot=%.3f tol3=%.5f\n",
                                     a, who, pr, std::sqrt(best), dt, tol3);
                    }
                }
                std::vector<char> used(fr.size(), 0);
                std::vector<NurbsCurve> chained;
                for (size_t s = 0; s < fr.size(); ++s) {
                    if (used[s]) continue;
                    used[s] = 1;
                    std::vector<Point> chain = fr[s].uv;
                    Point cfP = fr[s].p3f, cbP = fr[s].p3b;
                    std::array<double,3> cfD = fr[s].df, cbD = fr[s].db;
                    bool joined = false, grew = true;
                    while (grew) {
                        grew = false;
                        for (size_t k = 0; k < fr.size(); ++k) {
                            if (used[k]) continue;
                            const Frag& P = fr[k];
                            if (d2(cbP, P.p3f) <= tol3_2 && smooth(cbD, P.df)) {          // chain.back -> P.front
                                chain.insert(chain.end(), P.uv.begin() + 1, P.uv.end());
                                cbP = P.p3b; cbD = P.db;
                            } else if (d2(cbP, P.p3b) <= tol3_2 && smooth(cbD, P.db)) {    // chain.back -> P reversed
                                for (size_t i = P.uv.size() - 1; i-- > 0;) chain.push_back(P.uv[i]);
                                cbP = P.p3f; cbD = P.df;
                            } else if (d2(cfP, P.p3b) <= tol3_2 && smooth(cfD, P.db)) {    // P -> chain.front
                                std::vector<Point> pre(P.uv.begin(), P.uv.end() - 1);
                                pre.insert(pre.end(), chain.begin(), chain.end());
                                chain.swap(pre); cfP = P.p3f; cfD = P.df;
                            } else if (d2(cfP, P.p3f) <= tol3_2 && smooth(cfD, P.df)) {    // P reversed -> chain.front
                                std::vector<Point> pre(P.uv.rbegin(), P.uv.rend() - 1);
                                pre.insert(pre.end(), chain.begin(), chain.end());
                                chain.swap(pre); cfP = P.p3b; cfD = P.db;
                            } else {
                                continue;
                            }
                            used[k] = 1; grew = true; joined = true; ++frags_joined;
                        }
                    }
                    if (joined) {
                        NurbsCurve c = NurbsCurve::create(false, 1, chain);
                        chained.push_back(c.is_valid() ? c : out[s]);
                    } else {
                        chained.push_back(out[s]);                       // un-chained: preserve verbatim
                    }
                }
                out.swap(chained);
            }
            if (s_tdbg && frags_in > 1)
                std::fprintf(stderr, "[TRIMCUT] frags=%zu joins=%zu out=%zu\n", frags_in, frags_joined, out.size());
            return out;
        }
        if (pre_cuts) {
            // Caller supplied the section pcurves (one SSI per surface pair, shared with the
            // other operand's split) -- consume them instead of re-running an order-sensitive
            // SSI here.
            int si = (int)(&srf - m_surfaces.data());
            if (si >= 0 && si < (int)pre_cuts->size()) {
                if (std::getenv("SESSION_MARCH_DBG") && !(*pre_cuts)[si].empty())
                    std::fprintf(stderr, "[PAIR] si=%d pre_cuts=%zu\n", si, (*pre_cuts)[si].size());
                for (const auto& pc : (*pre_cuts)[si])
                    push_deduped(pc);
            }
            return out;
        }
        for (size_t ci = 0; ci < cutter.m_surfaces.size(); ++ci) {
            if (!aabb_overlap(srf_bb, cutter_bbs[ci], margin)) continue;
            auto cc = Intersection::cut_curves_on_surface(srf, cutter.m_surfaces[ci], tolerance);
            if (std::getenv("SESSION_MARCH_DBG") && !cc.empty())
                std::fprintf(stderr, "[PAIR] ci=%zu cuts=%zu\n", ci, cc.size());
            for (auto& pc : cc)
                push_deduped(pc);
        }
        return out;
    }, imported_freeform, scaf, scaf_is_A, sec_edges_out, face_src_out, extra_cuts, sec_spans_out, pool);
}

BRep BRep::split_with(double tolerance, const std::function<std::vector<NurbsCurve>(const NurbsSurface&)>& cut_for, bool imported_freeform,
                      const SectionScaffold* scaf, bool scaf_is_A,
                      std::map<int, std::array<int, 3>>* sec_edges_out,
                      std::vector<int>* face_src_out,
                      const std::vector<std::vector<NurbsCurve>>* extra_cuts,
                      std::map<int, std::array<double, 3>>* sec_spans_out,
                      const SharedEdgePool* pool) const {
    g_fed_cuts.clear();   // records (surface, seg) cuts actually fed on the scaffold path
    BRep result;
    result.name = name;
    // BOP2 Phase 4: pre-seed the result with the SHARED arena so section runs REFERENCE
    // pool edges (identical indices in BOTH operands' results; combine unifies by index).
    int bop2_nverts = 0, bop2_nedges = 0;
    if (pool) {
        result.m_vertices = pool->arena.m_vertices;
        result.m_topology_vertices = pool->arena.m_topology_vertices;
        result.m_curves_3d = pool->arena.m_curves_3d;
        result.m_topology_edges = pool->arena.m_topology_edges;
        for (auto& e : result.m_topology_edges) e.trim_indices.clear();
        bop2_nverts = (int)result.m_topology_vertices.size();
        bop2_nedges = (int)result.m_topology_edges.size();
    }
    std::map<std::tuple<long long, long long, long long>, int> vmap;
    std::map<std::tuple<int, int, long long, long long, long long>, int> emap;
    // OCCT-adoption S2: section edges are keyed by scaffold identity (seg_id) so the two
    // fragments flanking a section within one operand share ONE edge by construction, and
    // sec_edges_out lets BRep::boolean merge the operands' copies at combine.
    std::map<int, int> sec_emap;   // seg_id -> result topology-edge index
    static const bool s_prof = (std::getenv("SESSION_BOOL_PROFILE") != nullptr);
    double prof_ssi = 0, prof_arr = 0, prof_lift = 0;
    auto pf_now = []{ return std::chrono::high_resolution_clock::now(); };
    auto pf_us = [](auto a, auto b){ return std::chrono::duration<double, std::micro>(b - a).count(); };

    auto lift_loop = [&](const NurbsSurface& srf, double devtol, const NurbsCurve& pc,
                         NurbsCurve& c3d, Point& p0, Point& p1, Point& pm) {
        // Lift a 2D pcurve onto the surface as a 3D polyline edge, refining ADAPTIVELY by 3D
        // chord deviation: a straight lift (box segment on a planar face) terminates at 2 points,
        // while a 2-CV UV line that wraps a cylinder into a full circle is subdivided to the chord
        // tolerance. This is the boolean's hot path -- a fixed dense sample count made point_at
        // dominate the whole operation.
        auto dc = pc.domain();
        auto eval = [&](double t) -> Point { Point uv = pc.point_at(t); return srf.point_at(uv[0], uv[1]); };
        std::function<void(double, const Point&, double, const Point&, int, std::vector<Point>&)> rec =
            [&](double ta, const Point& pa, double tb, const Point& pb, int depth, std::vector<Point>& acc) {
                double tm = 0.5 * (ta + tb); Point pmid = eval(tm);
                double ex = pb[0]-pa[0], ey = pb[1]-pa[1], ez = pb[2]-pa[2];
                double L2 = ex*ex + ey*ey + ez*ez, dev;
                if (L2 > 1e-30) {
                    double t = ((pmid[0]-pa[0])*ex+(pmid[1]-pa[1])*ey+(pmid[2]-pa[2])*ez)/L2;
                    double cx = pa[0]+t*ex, cy = pa[1]+t*ey, cz = pa[2]+t*ez;
                    dev = std::sqrt((pmid[0]-cx)*(pmid[0]-cx)+(pmid[1]-cy)*(pmid[1]-cy)+(pmid[2]-cz)*(pmid[2]-cz));
                } else {
                    dev = std::sqrt((pmid[0]-pa[0])*(pmid[0]-pa[0])+(pmid[1]-pa[1])*(pmid[1]-pa[1])+(pmid[2]-pa[2])*(pmid[2]-pa[2]));
                }
                if (dev > devtol && depth < 9) {
                    rec(ta, pa, tm, pmid, depth+1, acc);
                    acc.push_back(pmid);
                    rec(tm, pmid, tb, pb, depth+1, acc);
                }
            };
        int n0 = std::max(pc.cv_count() - 1, 1);
        std::vector<Point> pts3;
        Point prev = eval(dc.first);
        pts3.push_back(prev);
        for (int i = 0; i < n0; ++i) {
            double ta = dc.first + (dc.second - dc.first) * i / n0;
            double tb = dc.first + (dc.second - dc.first) * (i + 1) / n0;
            Point pa = (i == 0) ? prev : eval(ta);
            Point pb = eval(tb);
            rec(ta, pa, tb, pb, 0, pts3);
            pts3.push_back(pb);
        }
        c3d = NurbsCurve::create(false, 1, pts3);
        p0 = pts3.front();
        p1 = pts3.back();
        // ARC-LENGTH midpoint, not the index middle: two copies of one physical edge are
        // lifted with independent adaptive refinement, so their sample counts differ and the
        // index-middle lands at unrelated arc positions (up to ~20% of the length apart).
        // The 50%-arc-length point is direction- and density-invariant; two copies' arc-mids
        // differ only by their true geometric divergence (bounded by the lift deviations).
        {
            double L = 0;
            for (size_t i = 0; i + 1 < pts3.size(); ++i) L += pts3[i].distance(pts3[i+1]);
            double half = 0.5 * L, acc = 0;
            pm = pts3[pts3.size() / 2];
            for (size_t i = 0; i + 1 < pts3.size(); ++i) {
                double seg = pts3[i].distance(pts3[i+1]);
                if (acc + seg >= half) {
                    double w = seg > 1e-30 ? (half - acc) / seg : 0.0;
                    pm = Point(pts3[i][0] + (pts3[i+1][0]-pts3[i][0])*w,
                               pts3[i][1] + (pts3[i+1][1]-pts3[i][1])*w,
                               pts3[i][2] + (pts3[i+1][2]-pts3[i][2])*w);
                    break;
                }
                acc += seg;
            }
        }
    };

    // Pave points shared by ALL faces' splits: scaffold verts on the scaffold path, or the
    // 3D endpoints of the pair-SSI cut curves on the quadric path (a cut ENDS where it
    // crosses an edge = the EF interference). One vertex per pave either way.
    std::vector<Point> pave_pts;
    double pave_cap_tol = 0.0;
    std::vector<std::vector<NurbsCurve>> face_cut_cache;
    bool have_cut_cache = false;
    if (scaf) {
        pave_pts = scaf->vertices;
        pave_cap_tol = (scaf->tol3_rep > 0 ? scaf->tol3_rep : scaf->tol3) * 0.7;
    } else if (std::getenv("SESSION_EF_PAVES")) {
        // EF paves from cut endpoints (cache the per-face cuts; the face loop reuses them).
        // OPT-IN: the full identity bundle regressed the proven quadric path (reimport
        // cone x cyl 2->8 naked); needs per-mechanism isolation before default-on.
        double bmn2[3] = {1e300,1e300,1e300}, bmx2[3] = {-1e300,-1e300,-1e300};
        for (const auto& p : m_vertices)
            for (int k = 0; k < 3; ++k) { bmn2[k] = std::min(bmn2[k], p[k]); bmx2[k] = std::max(bmx2[k], p[k]); }
        double dg2 = m_vertices.empty() ? 1.0 :
            std::sqrt((bmx2[0]-bmn2[0])*(bmx2[0]-bmn2[0]) + (bmx2[1]-bmn2[1])*(bmx2[1]-bmn2[1]) +
                      (bmx2[2]-bmn2[2])*(bmx2[2]-bmn2[2]));
        pave_cap_tol = std::max(1e-9, dg2 * 2e-3);
        face_cut_cache.assign(m_faces.size(), {});
        have_cut_cache = true;
        for (int fidx = 0; fidx < (int)m_faces.size(); ++fidx) {
            const auto& face = m_faces[fidx];
            if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
            const NurbsSurface& srf2 = m_surfaces[face.surface_index];
            std::vector<NurbsCurve> cuts2;
            try { cuts2 = cut_for(srf2); } catch (...) { continue; }
            face_cut_cache[fidx] = cuts2;
            for (const auto& c2 : cuts2) {
                if (!c2.is_valid()) continue;
                auto d2 = c2.domain();
                // CLOSED cuts have no EF endpoints -- their domain ends meet at an
                // arbitrary seam point, and treating that as a pave drags real vertices
                // onto it (reimport cone x cyl regressed 2->8 naked).
                Point pf2 = c2.point_at(d2.first), pb2 = c2.point_at(d2.second);
                if (std::hypot(pf2[0]-pb2[0], pf2[1]-pb2[1]) < 1e-9) continue;
                for (double t2 : {d2.first, d2.second}) {
                    Point uv2 = c2.point_at(t2);
                    Point p3 = srf2.point_at(uv2[0], uv2[1]);
                    // a true EF pave lies ON one of this operand's edges
                    bool on_edge = false;
                    for (int ei2 = 0; ei2 < (int)m_topology_edges.size() && !on_edge; ++ei2) {
                        const auto& EE = m_topology_edges[ei2];
                        if (EE.curve_3d_index < 0 || EE.curve_3d_index >= (int)m_curves_3d.size()) continue;
                        const NurbsCurve& ec2 = m_curves_3d[EE.curve_3d_index];
                        auto ed2 = ec2.domain();
                        for (int k2 = 0; k2 <= 32 && !on_edge; ++k2)
                            if (ec2.point_at(ed2.first + (ed2.second-ed2.first)*k2/32.0).distance(p3) < pave_cap_tol)
                                on_edge = true;
                    }
                    if (!on_edge) continue;
                    bool dup = false;
                    for (const auto& q : pave_pts)
                        if (q.distance(p3) < pave_cap_tol) { dup = true; break; }
                    if (!dup) pave_pts.push_back(p3);
                }
            }
        }
    }
    // OCCT pave-capture: run endpoints that land within the scaffold weld tolerance of a
    // PAVE vertex (an authoritative, 3D-welded junction shared by BOTH operands' splits)
    // snap to that pave. Without this, each face's arrangement places the same junction
    // at its own conditioning error (up to tol3) and no downstream key can match.
    std::vector<int> pave_tv;                       // scaffold vertex id -> result tv (lazy)
    double pave_tol2 = 0.0;
    static const bool s_no_pavesnap = (std::getenv("SESSION_NO_PAVESNAP") != nullptr);
    if (!pave_pts.empty() && !s_no_pavesnap) {
        pave_tv.assign(pave_pts.size(), -1);
        // BOP2: scaffold verts are ALREADY in the pre-seeded arena -- captures land on
        // the SAME topology vertices in both operands' results by index.
        if (pool)
            for (size_t i2 = 0; i2 < pool->vert_tv.size() && i2 < pave_tv.size(); ++i2)
                pave_tv[i2] = pool->vert_tv[i2];
        // 0.7*tol3 on the scaffold path: paves must capture their spurious micro-clusters
        // (near-tangent arrangement crossings land up to ~0.55*tol3 from the true junction
        // -- the z30x20 sliver class); distinct paves are >= tol3 apart by welding.
        // Quadric path: pave_cap_tol = diag*2e-3 (EF endpoint identity).
        pave_tol2 = pave_cap_tol * pave_cap_tol;
    }
    // ORIGINAL corner capture: each face evaluates a shared corner through ITS OWN surface,
    // and imported files carry surface-to-vertex incidence gaps up to their declared
    // uncertainty (5e-4 in our STEP exports) -- far beyond the 1e-6 gridding. Snap run
    // endpoints to the operand's own topology vertices, radius capped at a third of the
    // closest original-vertex pair so distinct corners can never fuse.
    std::vector<int> orig_tv;
    double orig_tol2 = 0.0;
    if (!pave_pts.empty() && !s_no_pavesnap && !m_topology_vertices.empty()) {
        orig_tv.assign(m_topology_vertices.size(), -1);
        double ot; {
            double bmn[3] = {1e300,1e300,1e300}, bmx[3] = {-1e300,-1e300,-1e300};
            for (const auto& p : m_vertices)
                for (int k = 0; k < 3; ++k) { bmn[k] = std::min(bmn[k], p[k]); bmx[k] = std::max(bmx[k], p[k]); }
            ot = std::max(1e-9, std::sqrt((bmx[0]-bmn[0])*(bmx[0]-bmn[0]) + (bmx[1]-bmn[1])*(bmx[1]-bmn[1]) +
                                          (bmx[2]-bmn[2])*(bmx[2]-bmn[2])) * 5e-4);
        }
        double mind = 1e300;
        for (size_t a2 = 0; a2 < m_topology_vertices.size(); ++a2)
            for (size_t b2 = a2 + 1; b2 < m_topology_vertices.size(); ++b2) {
                double d = m_vertices[m_topology_vertices[a2].point_index]
                          .distance(m_vertices[m_topology_vertices[b2].point_index]);
                mind = std::min(mind, d);
            }
        ot = std::min(ot, mind / 3.0);
        orig_tol2 = ot * ot;
    }
    // SEED UNIFICATION (OCCT MakeSDVertices): an original corner within the pave capture
    // ball of a scaffold vertex IS that junction (VF interference). Without the alias the
    // two operands capture the same physical point to DIFFERENT seeds (one to its own
    // corner, the other to the shared pave) and the copies never mate.
    std::vector<int> orig_to_pave;
    if (!orig_tv.empty() && pave_tol2 > 0) {
        orig_to_pave.assign(m_topology_vertices.size(), -1);
        for (size_t vi2 = 0; vi2 < m_topology_vertices.size(); ++vi2) {
            const Point& q = m_vertices[m_topology_vertices[vi2].point_index];
            int best = -1; double bd = pave_tol2;
            for (size_t si2 = 0; si2 < pave_pts.size(); ++si2) {
                double dd = pave_pts[si2].distance(q); dd *= dd;
                if (dd < bd) { bd = dd; best = (int)si2; }
            }
            orig_to_pave[vi2] = best;
        }
    }
    // Deterministic pave-vertex mint: split points CREATED BY a pave adopt that pave's
    // vertex identity directly (no capture radius involved).
    auto mint_pave_tv = [&](int pv) -> int {
        if (pv < 0 || pv >= (int)pave_tv.size()) return -1;
        if (pave_tv[pv] < 0) {
            int idx = result.add_vertex(pave_pts[pv]);
            BRepVertex tv;
            tv.point_index = idx;
            result.m_topology_vertices.push_back(tv);
            pave_tv[pv] = idx;
        }
        return pave_tv[pv];
    };
    auto find_or_add_vertex = [&](const Point& p) -> int {
        if (!orig_tv.empty() && orig_tol2 > 0) {
            int best = -1; double bd = orig_tol2;
            for (size_t vi2 = 0; vi2 < m_topology_vertices.size(); ++vi2) {
                const Point& q = m_vertices[m_topology_vertices[vi2].point_index];
                double d0 = q[0]-p[0], d1 = q[1]-p[1], d2 = q[2]-p[2];
                double dd = d0*d0 + d1*d1 + d2*d2;
                if (dd < bd) { bd = dd; best = (int)vi2; }
            }
            if (best >= 0 && !orig_to_pave.empty() && orig_to_pave[best] >= 0) {
                int pv = orig_to_pave[best];
                if (pave_tv[pv] < 0) {
                    int idx = result.add_vertex(pave_pts[pv]);
                    BRepVertex tv;
                    tv.point_index = idx;
                    result.m_topology_vertices.push_back(tv);
                    pave_tv[pv] = idx;
                }
                return pave_tv[pv];
            }
            if (best >= 0) {
                if (orig_tv[best] < 0) {
                    int idx = result.add_vertex(m_vertices[m_topology_vertices[best].point_index]);
                    BRepVertex tv;
                    tv.point_index = idx;
                    result.m_topology_vertices.push_back(tv);
                    orig_tv[best] = idx;
                }
                return orig_tv[best];
            }
        }
        if (pave_tol2 > 0) {
            int best = -1; double bd = pave_tol2;
            for (size_t si2 = 0; si2 < pave_pts.size(); ++si2) {
                const Point& q = pave_pts[si2];
                double d0 = q[0]-p[0], d1 = q[1]-p[1], d2 = q[2]-p[2];
                double dd = d0*d0 + d1*d1 + d2*d2;
                if (dd < bd) { bd = dd; best = (int)si2; }
            }
            if (best >= 0) {
                if (pave_tv[best] < 0) {
                    int idx = result.add_vertex(pave_pts[best]);
                    BRepVertex tv;
                    tv.point_index = idx;
                    result.m_topology_vertices.push_back(tv);
                    pave_tv[best] = idx;
                }
                return pave_tv[best];
            }
        }
        // Tolerant weld, not a floor-quantizer: two copies of one physical corner that
        // diverge by the model's incidence error (1e-9..1e-7) can still straddle a 1e-6
        // grid boundary -- the exact-cell hash then mints two vertices and every edge
        // between them is unmatable. Scan the 3x3x3 neighborhood and accept the first
        // seed within the grid step.
        long long kx = q6(p[0]), ky = q6(p[1]), kz = q6(p[2]);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                for (long long dz = -1; dz <= 1; ++dz) {
                    auto it = vmap.find(std::make_tuple(kx+dx, ky+dy, kz+dz));
                    if (it == vmap.end()) continue;
                    const Point& q = result.m_vertices[result.m_topology_vertices[it->second].point_index];
                    double d0 = q[0]-p[0], d1 = q[1]-p[1], d2 = q[2]-p[2];
                    if (d0*d0 + d1*d1 + d2*d2 <= 1e-12) return it->second;
                }
        int idx = result.add_vertex(p);
        BRepVertex tv;
        tv.point_index = idx;
        result.m_topology_vertices.push_back(tv);
        vmap[std::make_tuple(kx, ky, kz)] = idx;
        return idx;
    };

    // Boundary-run pave-block identity (scaffold path): a run trimmed from boundary pcurve
    // cidx belongs to ORIGINAL operand edge scaf_bnd_edge_ids[cidx]. Copies of one physical
    // sub-edge lifted by the two flanking faces then key into ONE result edge by
    // (orig_edge, endpoints, tolerant midpoint) instead of relying on the Hausdorff sew.
    std::vector<int> scaf_bnd_edge_ids;
    struct BEnt { Point pm; int ei; double dev; };
    std::map<int, char> edge_kind_dbg;   // BEMAP_DBG: result edge -> 'S'ection/'B'oundary/'L'egacy
    // Per-edge TUBE tolerance (OCCT edge-tolerance analog, transient): the lift deviation
    // of the copy that minted the edge. Coincidence tests use the SUM of the two entities'
    // tolerances -- the kernel-wide doctrine (f64 + per-entity tolerance, no exactness).
    std::vector<double> edge_dev;
    auto note_edge_dev = [&](int ei, double dev) {
        if ((int)edge_dev.size() <= ei) edge_dev.resize(ei + 1, 0.0);
        edge_dev[ei] = dev;
    };
    // OCCT MakeSplitEdges: per ORIGINAL edge, the scaffold pave vertices lying ON it. Every
    // boundary run (split fragment or pass-through face) is subdivided at these paves, so
    // all copies of one physical edge cover IDENTICAL pave intervals with IDENTICAL welded
    // endpoints -- a pass-through face's long edge and its split neighbour's sub-edges
    // otherwise form an unmatable T-junction (the dominant 1-trim class).
    std::map<int, std::vector<std::pair<Point, int>>> edge_paves;   // (pave pos, pave id)
    if (!pave_pts.empty()) {
        double ptol = pave_cap_tol;
        for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
            const BRepEdge& E = m_topology_edges[ei];
            if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& ec = m_curves_3d[E.curve_3d_index];
            if (!ec.is_valid()) continue;
            auto ed = ec.domain();
            // dense polyline of the edge (boundary edges are short; 65 samples suffice for
            // an ON test at pave tolerance)
            std::vector<Point> poly(65);
            for (int k = 0; k <= 64; ++k)
                poly[k] = ec.point_at(ed.first + (ed.second - ed.first) * k / 64.0);
            Point pe0 = poly.front(), pe1 = poly.back();
            for (size_t svi = 0; svi < pave_pts.size(); ++svi) {
                const Point& v = pave_pts[svi];
                double best = 1e300;
                for (size_t j = 0; j + 1 < poly.size(); ++j) {
                    double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1], ez = poly[j+1][2]-poly[j][2];
                    double L2 = ex*ex + ey*ey + ez*ez;
                    double t = L2 > 1e-30 ? ((v[0]-poly[j][0])*ex + (v[1]-poly[j][1])*ey + (v[2]-poly[j][2])*ez) / L2 : 0.0;
                    t = std::min(std::max(t, 0.0), 1.0);
                    double dx = v[0]-poly[j][0]-t*ex, dy = v[1]-poly[j][1]-t*ey, dz = v[2]-poly[j][2]-t*ez;
                    best = std::min(best, dx*dx + dy*dy + dz*dz);
                }
                if (best > ptol * ptol) continue;
                if (v.distance(pe0) < ptol || v.distance(pe1) < ptol) continue;   // end paves split nothing
                edge_paves[ei].push_back({v, (int)svi});
            }
        }
        if (std::getenv("SESSION_BEMAP_DBG")) {
            size_t np = 0;
            for (auto& kv : edge_paves) np += kv.second.size();
            std::fprintf(stderr, "[EPAVE] side=%c edges=%zu paved_edges=%zu paves=%zu tol=%.4f\n",
                         scaf_is_A ? 'A' : 'B', m_topology_edges.size(), edge_paves.size(), np, ptol);
            for (size_t a3 = 0; a3 < scaf->vertices.size(); ++a3)
                for (size_t b3 = a3 + 1; b3 < scaf->vertices.size(); ++b3) {
                    double d3 = scaf->vertices[a3].distance(scaf->vertices[b3]);
                    if (d3 < scaf->tol3 * 1.5)
                        std::fprintf(stderr, "[SCAFPAIR] v%zu v%zu d=%.4f (tol3=%.4f) (%.4f,%.4f,%.4f)\n",
                                     a3, b3, d3, scaf->tol3,
                                     scaf->vertices[a3][0], scaf->vertices[a3][1], scaf->vertices[a3][2]);
                }
            std::fprintf(stderr, "[VDUMP] side=%c ntv=%zu:", scaf_is_A ? 'A' : 'B', m_topology_vertices.size());
            for (size_t k = 0; k < m_topology_vertices.size() && k < 12; ++k) {
                const Point& q = m_vertices[m_topology_vertices[k].point_index];
                std::fprintf(stderr, " (%.3f,%.3f,%.3f)", q[0], q[1], q[2]);
            }
            std::fprintf(stderr, "\n");
        }
    }
    std::map<std::tuple<int, int, int>, std::vector<BEnt>> bemap;
    double bemap_tol = 0.0; {
        double bmn[3] = {1e300,1e300,1e300}, bmx[3] = {-1e300,-1e300,-1e300};
        for (const auto& p : m_vertices)
            for (int k = 0; k < 3; ++k) { bmn[k] = std::min(bmn[k], p[k]); bmx[k] = std::max(bmx[k], p[k]); }
        double dg = m_vertices.empty() ? 1.0 :
            std::sqrt((bmx[0]-bmn[0])*(bmx[0]-bmn[0]) + (bmx[1]-bmn[1])*(bmx[1]-bmn[1]) +
                      (bmx[2]-bmn[2])*(bmx[2]-bmn[2]));
        bemap_tol = std::max(1e-9, dg * 5e-4);
    }

    // Per-face scaffold state: cut index -> scaffold seg_id (set in the face loop below,
    // read by append_face to map an arrangement run back to its shared section segment).
    std::vector<int> scaf_cut_seg_ids;
    std::vector<std::array<double, 2>> scaf_cut_spans;   // per cut: [t_lo, t_hi] of the TRUE
                                                          // segment inside the overshot pcurve
    std::vector<Point> scaf_forced_nodes;                 // pave endpoints on this face's boundary
    int scaf_n_boundary = 0;
    int scaf_fallbacks = 0;
    std::map<int, int> seg_run_count;   // resource guard: chain-lift runs per segment (whole split)
    const std::vector<NurbsCurve>* all_pcs_ref = nullptr;   // current face's boundary+cut pcurves

    auto append_face = [&](const NurbsSurface& srf,
                           const std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>>& loops,
                           const std::vector<const std::vector<std::array<double, 3>>*>* loop_srcs = nullptr) {
        int si = result.add_surface(srf);
        int fi = result.add_face(si, false);
        // One deviation tolerance per face (chord error target = 5e-4 of the surface size): a
        // straight lift stays at 2 points, a wrapped circle subdivides to within the sew tol.
        double sd; {
            double bmn[3]={1e30,1e30,1e30}, bmx[3]={-1e30,-1e30,-1e30};
            for (int i = 0; i < srf.cv_count(0); ++i) for (int j = 0; j < srf.cv_count(1); ++j) {
                Point p = srf.get_cv(i, j);
                for (int k = 0; k < 3; ++k) { bmn[k]=std::min(bmn[k],p[k]); bmx[k]=std::max(bmx[k],p[k]); }
            }
            sd = std::sqrt((bmx[0]-bmn[0])*(bmx[0]-bmn[0])+(bmx[1]-bmn[1])*(bmx[1]-bmn[1])+(bmx[2]-bmn[2])*(bmx[2]-bmn[2]));
            if (sd < 1e-12) sd = 1.0;
        }
        // one operand's section copy is now fit-accurate (~1e-5); the other's lift must
        // stay within the sew tolerance of it, not merely within 2e-3 of itself
        double devtol = sd * 2e-3;
        for (size_t lidx = 0; lidx < loops.size(); ++lidx) {
            const auto& lp = loops[lidx];
            const std::vector<std::array<double, 3>>* srcs =
                (loop_srcs && lidx < loop_srcs->size()) ? (*loop_srcs)[lidx] : nullptr;
            int li = result.add_loop(fi, lp.first);
            for (size_t pidx = 0; pidx < lp.second.size(); ++pidx) {
                const auto& pc = lp.second[pidx];
                if (!pc.is_valid()) continue;
                // OCCT-adoption S2: a run sourced from a scaffold section segment lifts
                // to the SHARED 3D chain (identical on both operands by construction) and
                // is keyed by seg_id -- one edge record per segment per operand, merged
                // across operands by BRep::boolean at combine.
                // OCCT-truth for section runs: the two operands' arrangements will NEVER
                // agree on clip params, and they don't have to -- both lift their 3D as an
                // EXACT SUB-RANGE of the one shared chain, so wherever their ranges overlap
                // the copies are pointwise identical and imprint+sew mate them exactly.
                bool lifted_from_chain = false;
                if (scaf && srcs && pidx < srcs->size()) {
                    int cidx = (int)(*srcs)[pidx][0];
                    if (cidx >= scaf_n_boundary &&
                        cidx - scaf_n_boundary < (int)scaf_cut_seg_ids.size() &&
                        scaf_cut_seg_ids[cidx - scaf_n_boundary] >= 0) {   // -1 = same-domain
                        // imprint cut (no scaffold segment): falls through to legacy lift
                        int ci = cidx - scaf_n_boundary;
                        const SectionSegment& s = scaf->segments[scaf_cut_seg_ids[ci]];
                        double t_lo = scaf_cut_spans[ci][0], t_hi = scaf_cut_spans[ci][1];
                        bool closed_seg = s.v_start == s.v_end;
                        double ra = std::min((*srcs)[pidx][1], (*srcs)[pidx][2]);
                        double rb = std::max((*srcs)[pidx][1], (*srcs)[pidx][2]);
                        int nCh = (int)s.p3.size();
                        bool full_wrap = closed_seg && rb - ra < 1e-9;
                        bool stub_legacy = false;
                        if (!full_wrap) {
                            if (rb - ra < 1e-9) {
                                // ZERO-SPAN COLLAPSE (ta==tb): this operand's arrangement clipped
                                // the shared segment to a single point -- the section runs ALONG
                                // this operand's seam/trim boundary, so the "inside" strip is
                                // ~zero-width. The OTHER operand traced the segment's real extent,
                                // so dropping this copy leaves a ONE-SIDED imprint -> the section
                                // edge stays 1-trim -> naked. Emit the WHOLE shared segment here
                                // too (Law 5, symmetric total imprint): the 3D is the exact shared
                                // sub-chain, so both operands' copies key the SAME segment and mate.
                                if (!std::getenv("SESSION_ZEROFILL") || t_hi - t_lo < 1e-9)
                                    continue;                          // default: drop zero-length artifact
                                ra = t_lo; rb = t_hi;                  // opt-in fill with the whole segment span
                            }
                            ra = std::max(ra, t_lo);                   // strip overshoot stubs
                            rb = std::min(rb, t_hi);
                            // clamped to nothing but had real extent: a boundary-to-pave stub
                            // from sampling disagreement -- legacy-lift it so the loop closes
                            if (rb - ra < 1e-9) stub_legacy = true;
                        }
                        // deg-1 uniform-by-index: vertex k of the chain sits at exactly
                        // t_lo + k*(t_hi-t_lo)/(nCh-1) -- the map is LINEAR in index space
                        double denom = t_hi - t_lo;
                        auto chain_pos = [&](double t) -> double {
                            if (denom < 1e-30) return 0.0;
                            double f = (t - t_lo) * (nCh - 1) / denom;
                            return std::min(std::max(f, 0.0), (double)(nCh - 1));
                        };
                        double fa = full_wrap ? 0.0 : chain_pos(ra);
                        double fb = full_wrap ? (double)(nCh - 1) : chain_pos(rb);
                        if (g_segrun_cap > 0 && !stub_legacy && fb - fa > 1e-9) {
                            int& rc = seg_run_count[s.seg_id];
                            if (++rc > g_segrun_cap)
                                throw std::runtime_error(
                                    "[RUNCAP] sliver ladder: seg " + std::to_string(s.seg_id) +
                                    " lifted " + std::to_string(rc) + " runs (cap " +
                                    std::to_string(g_segrun_cap) + ")");
                            if ((rc & 7) == 0) guard_check_mem("chain-lift");
                        }
                        if (!stub_legacy && fb - fa > 1e-9) {
                            // extract the shared sub-polyline (lerped fractional ends)
                            auto lerp3 = [&](double f) {
                                int k = std::min((int)f, nCh - 2);
                                double w = f - k;
                                return Point(s.p3[k][0] + (s.p3[k+1][0]-s.p3[k][0])*w,
                                             s.p3[k][1] + (s.p3[k+1][1]-s.p3[k][1])*w,
                                             s.p3[k][2] + (s.p3[k+1][2]-s.p3[k][2])*w);
                            };
                            std::vector<Point> sub;
                            sub.push_back(lerp3(fa));
                            for (int k = (int)std::ceil(fa + 1e-9); k <= (int)std::floor(fb - 1e-9) && k < nCh; ++k)
                                sub.push_back(s.p3[k]);
                            sub.push_back(lerp3(fb));
                            // snap ends onto the scaffold pave vertices when the run reaches them
                            if (fa < 1e-6) sub.front() = s.p3.front();
                            if (fb > nCh - 1 - 1e-6) sub.back() = s.p3.back();
                            if (full_wrap) sub.back() = sub.front();   // exact closed 3D loop
                            if (sub.size() >= (full_wrap ? 4u : 2u) &&
                                (full_wrap || sub.front().distance(sub.back()) > 1e-12)) {
                                NurbsCurve c3d = NurbsCurve::create(false, 1, sub);
                                if (c3d.is_valid()) {
                                    // BOP2 Phase 4: a whole-segment run REFERENCES the pool
                                    // edge (identical index in both operands' results) --
                                    // no mint, no emap, mated at combine by construction.
                                    bool wseg2 = full_wrap || (fa < 1e-2 && fb > nCh - 1 - 1e-2);
                                    if (pool && wseg2 && s.seg_id >= 0 &&
                                        s.seg_id < (int)pool->seg_edge.size() &&
                                        pool->seg_edge[s.seg_id] >= 0) {
                                        int ei = pool->seg_edge[s.seg_id];
                                        BRepTrimType ttype =
                                            result.m_topology_edges[ei].trim_indices.empty()
                                                ? BRepTrimType::Boundary : BRepTrimType::Mated;
                                        if (sec_edges_out)
                                            (*sec_edges_out)[ei] = {s.seg_id, s.v_start, s.v_end};
                                        sec_emap[s.seg_id] = ei;
                                        int ci2p = result.add_curve_2d(pc);
                                        result.add_trim(ci2p, ei, li, false, ttype);
                                        if (sec_spans_out)
                                            (*sec_spans_out)[ei] = {(double)s.seg_id, fa, fb};
                                        lifted_from_chain = true;
                                        if (std::getenv("SESSION_NT_DBG"))
                                            std::fprintf(stderr, "[P4REF] seg=%d e%d %s\n",
                                                         s.seg_id, ei,
                                                         ttype == BRepTrimType::Mated ? "MATED" : "first");
                                        continue;
                                    }
                                    if (pool && std::getenv("SESSION_NT_DBG"))
                                        std::fprintf(stderr, "[P4MISS] seg=%d fa=%.4f fb=%.4f nCh=%d\n",
                                                     s.seg_id, fa, fb, nCh);
                                    int ci3d = result.add_curve_3d(c3d);
                                    int va = find_or_add_vertex(sub.front());
                                    int vb = find_or_add_vertex(sub.back());
                                    int lo = std::min(va, vb), hi = std::max(va, vb);
                                    // Direction-invariant midpoint (see lift_loop): the two
                                    // flanking fragments traverse the chain reversed; averaging
                                    // the middle pair keys both copies identically.
                                    Point pmS; {
                                        size_t nS = sub.size();
                                        const Point& ma = sub[(nS - 1) / 2];
                                        const Point& mb = sub[nS / 2];
                                        pmS = Point((ma[0]+mb[0])*0.5, (ma[1]+mb[1])*0.5, (ma[2]+mb[2])*0.5);
                                    }
                                    auto ekey = std::make_tuple(lo, hi, q6(pmS[0]), q6(pmS[1]), q6(pmS[2]));
                                    int ei;
                                    BRepTrimType ttype;
                                    auto it = emap.find(ekey);
                                    if (it != emap.end()) {
                                        ei = it->second;
                                        ttype = BRepTrimType::Mated;
                                        if (std::getenv("SESSION_NT_DBG"))
                                            std::fprintf(stderr, "[SCAF-RUN] seg=%d fa=%.4f fb=%.4f nCh=%d MATED e%d\n",
                                                         s.seg_id, fa, fb, nCh, ei);
                                    } else {
                                        ei = result.add_edge(ci3d, lo, hi);
                                        emap[ekey] = ei;
                                        edge_kind_dbg[ei] = 'S';
                                        ttype = BRepTrimType::Boundary;
                                        // combine-alias key ONLY for whole-segment edges:
                                        // partial pieces mate via sew (identical geometry).
                                        // Index tolerance 1e-2: trim-snapped cut ends can land a
                                        // sub-micron sliver inside the segment (fb=47.99997) --
                                        // still the whole section; genuine partials end at chain
                                        // vertices (integer index) or a full chord away.
                                        bool whole_seg = full_wrap || (fa < 1e-2 && fb > nCh - 1 - 1e-2);
                                        if (std::getenv("SESSION_NT_DBG"))
                                            std::fprintf(stderr, "[SCAF-RUN] seg=%d fa=%.4f fb=%.4f nCh=%d whole=%d\n",
                                                         s.seg_id, fa, fb, nCh, whole_seg ? 1 : 0);
                                        if (whole_seg) {
                                            if (sec_edges_out)
                                                (*sec_edges_out)[ei] = {s.seg_id, s.v_start, s.v_end};
                                            sec_emap[s.seg_id] = ei;
                                        }
                                    }
                                    int ci2d = result.add_curve_2d(pc);
                                    result.add_trim(ci2d, ei, li, false, ttype);
                                    // Pave-block record for EVERY chain-lifted run (whole or
                                    // partial): normalize_section_blocks turns clip-param
                                    // disagreements into shared block edges at combine.
                                    if (sec_spans_out)
                                        (*sec_spans_out)[ei] = {(double)s.seg_id, fa, fb};
                                    lifted_from_chain = true;
                                }
                            }
                        }
                        if (!lifted_from_chain && !stub_legacy) {
                            ++scaf_fallbacks;
                            if (std::getenv("SESSION_SPLIT_DBG"))
                                std::fprintf(stderr, "[SCAF-FALL] seg=%d ra=%.5f rb=%.5f t=[%.5f,%.5f]\n",
                                             s.seg_id, ra, rb, t_lo, t_hi);
                        }
                    }
                }
                if (lifted_from_chain) continue;
                // Boundary-run pave-block identity: the two faces flanking one physical
                // sub-edge of an ORIGINAL operand edge lift slightly different polylines
                // (their own pcurves, their own sampling), so an exact-quantized geometric
                // key never matches. Scoped to the original edge with a TOLERANT midpoint
                // compare, both copies land on one edge by identity.
                int oe = -1;
                if (srcs && pidx < srcs->size()) {
                    int bidx = (int)(*srcs)[pidx][0];
                    if (bidx >= 0 && bidx < scaf_n_boundary && bidx < (int)scaf_bnd_edge_ids.size())
                        oe = scaf_bnd_edge_ids[bidx];
                }
                // OCCT MakeSplitEdges: subdivide the run at the ORIGINAL edge's pave points
                // so a pass-through face's whole-edge run and a split neighbour's partial
                // runs cover IDENTICAL pave intervals (no T-junctions by construction).
                std::vector<NurbsCurve> bpieces;
                std::vector<int> bpieces_startv, bpieces_endv;   // forced scaffold-vertex ids (-1 = free)
                auto itp = oe >= 0 ? edge_paves.find(oe) : edge_paves.end();
                int dbg_rej_end = 0, dbg_rej_far = 0;
                if (itp != edge_paves.end() && !itp->second.empty()) {
                    auto dpc = pc.domain();
                    double span = dpc.second - dpc.first;
                    std::vector<std::pair<double, int>> tsv;   // (param, scaffold vertex id)
                    for (const auto& pvp : itp->second) {
                        const Point& pv = pvp.first;
                        auto prj = Closest::surface_point(srf, pv, 0.0, 0.0, 0.0, 0.0);
                        Point uvp(std::get<0>(prj), std::get<1>(prj), 0.0);
                        auto d2at = [&](double t) {
                            Point q = pc.point_at(t);
                            double dx = q[0]-uvp[0], dy = q[1]-uvp[1];
                            return dx*dx + dy*dy;
                        };
                        const int NS = 128;
                        double bt = dpc.first, bd = 1e300;
                        for (int k = 0; k <= NS; ++k) {
                            double t = dpc.first + span * k / NS;
                            double dd = d2at(t);
                            if (dd < bd) { bd = dd; bt = t; }
                        }
                        double lo_t = std::max(dpc.first, bt - span/NS);
                        double hi_t = std::min(dpc.second, bt + span/NS);
                        for (int it2 = 0; it2 < 30; ++it2) {
                            double m1 = lo_t + (hi_t-lo_t)/3, m2 = hi_t - (hi_t-lo_t)/3;
                            if (d2at(m1) < d2at(m2)) hi_t = m2; else lo_t = m1;
                        }
                        bt = 0.5*(lo_t + hi_t);
                        if (bt - dpc.first < span*1e-3 || dpc.second - bt < span*1e-3) { ++dbg_rej_end; continue; }
                        // the pave must actually sit ON this run in 3D. Tolerance 1.2*tol3:
                        // a section GRAZING along this edge parks its endpoint paves up to
                        // the graze distance off the curve, and those paves are exactly the
                        // ones whose split lets the other operand's section edge pair here.
                        Point quv = pc.point_at(bt);
                        Point q3 = srf.point_at(quv[0], quv[1]);
                        if (q3.distance(pv) > pave_cap_tol * 1.7) { ++dbg_rej_far; continue; }
                        tsv.push_back({bt, pvp.second});
                    }
                    if (std::getenv("SESSION_BEMAP_DBG"))
                        std::fprintf(stderr, "[BSPLIT] oe=%d cand=%zu kept=%zu rej_end=%d rej_far=%d\n",
                                     oe, itp->second.size(), tsv.size(), dbg_rej_end, dbg_rej_far);
                    if (!tsv.empty()) {
                        std::sort(tsv.begin(), tsv.end());
                        tsv.erase(std::unique(tsv.begin(), tsv.end(),
                                 [&](const std::pair<double,int>& a, const std::pair<double,int>& b){
                                     return std::abs(a.first - b.first) < span*1e-6; }), tsv.end());
                        double prev = dpc.first;
                        int prev_v = -1;
                        for (const auto& tv2 : tsv) {
                            NurbsCurve piece = pc;
                            if (piece.trim(prev, tv2.first) && piece.is_valid()) {
                                bpieces.push_back(piece);
                                bpieces_startv.push_back(prev_v);
                                bpieces_endv.push_back(tv2.second);
                            }
                            prev = tv2.first;
                            prev_v = tv2.second;
                        }
                        NurbsCurve tail = pc;
                        if (tail.trim(prev, dpc.second) && tail.is_valid()) {
                            bpieces.push_back(tail);
                            bpieces_startv.push_back(prev_v);
                            bpieces_endv.push_back(-1);
                        }
                    }
                }
                if (bpieces.empty()) { bpieces.push_back(pc); bpieces_startv.push_back(-1); bpieces_endv.push_back(-1); }
                for (size_t bpi = 0; bpi < bpieces.size(); ++bpi) {
                    const NurbsCurve& pcp = bpieces[bpi];
                    NurbsCurve c3d;
                    Point p0, p1, pm;
                    lift_loop(srf, devtol, pcp, c3d, p0, p1, pm);
                    if (std::getenv("SESSION_RUN_DBG") && srcs && pidx < srcs->size()) {
                        auto dr = pcp.domain();
                        std::fprintf(stderr, "[BRUN] side=%c fi=%d oe=%d cidx=%d ta=%.5f tb=%.5f dom=[%.5f,%.5f] "
                                     "p0(%.4f,%.4f,%.4f) p1(%.4f,%.4f,%.4f)\n",
                                     scaf_is_A ? 'A' : 'B', fi, oe, (int)(*srcs)[pidx][0],
                                     (*srcs)[pidx][1], (*srcs)[pidx][2], dr.first, dr.second,
                                     p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
                    }
                    int ci3d = result.add_curve_3d(c3d);
                    int va = bpi < bpieces_startv.size() ? mint_pave_tv(bpieces_startv[bpi]) : -1;
                    int vb = bpi < bpieces_endv.size() ? mint_pave_tv(bpieces_endv[bpi]) : -1;
                    if (va < 0) va = find_or_add_vertex(p0);
                    if (vb < 0) vb = find_or_add_vertex(p1);
                    int lo = std::min(va, vb), hi = std::max(va, vb);
                    int ei = -1;
                    BRepTrimType ttype = BRepTrimType::Boundary;
                    if (oe >= 0) {
                        auto& lst = bemap[std::make_tuple(oe, lo, hi)];
                        double best_miss = 1e300;
                        for (auto& pr : lst) {
                            double d0 = pr.pm[0]-pm[0], d1 = pr.pm[1]-pm[1], d2 = pr.pm[2]-pm[2];
                            double dd = d0*d0 + d1*d1 + d2*d2;
                            // match tolerance = the two copies' combined lift deviation (each
                            // arc-mid is within its own devtol of the true curve) + slack
                            double mt = 1.5 * (devtol + pr.dev) + bemap_tol * 0.1;
                            if (dd <= mt * mt) { ei = pr.ei; break; }
                            best_miss = std::min(best_miss, dd);
                        }
                        if (std::getenv("SESSION_BEMAP_DBG")) {
                            if (ei >= 0) std::fprintf(stderr, "[BEMAP] hit oe=%d lo=%d hi=%d\n", oe, lo, hi);
                            else std::fprintf(stderr, "[BEMAP] miss oe=%d lo=%d hi=%d cand=%zu bestd=%.3e\n",
                                              oe, lo, hi, lst.size(), best_miss < 1e300 ? std::sqrt(best_miss) : -1.0);
                        }
                        if (ei >= 0) ttype = BRepTrimType::Mated;
                        else { ei = result.add_edge(ci3d, lo, hi); lst.push_back({pm, ei, devtol});
                               edge_kind_dbg[ei] = 'B'; note_edge_dev(ei, devtol); }
                    } else {
                        auto ekey = std::make_tuple(lo, hi, q6(pm[0]), q6(pm[1]), q6(pm[2]));
                        auto it = emap.find(ekey);
                        if (it != emap.end()) {
                            ei = it->second;
                            ttype = BRepTrimType::Mated;
                        } else {
                            ei = result.add_edge(ci3d, lo, hi);
                            emap[ekey] = ei;
                            edge_kind_dbg[ei] = 'L';
                            note_edge_dev(ei, devtol);
                        }
                        static const bool s_seam_dbg = (std::getenv("SESSION_SEAM_DBG") != nullptr);
                        if (s_seam_dbg)
                            std::fprintf(stderr, "[BEDGE] fi=%d li=%d lo=%d hi=%d pm(%.7f,%.7f,%.7f) ei=%d %s\n",
                                         fi, li, lo, hi, pm[0], pm[1], pm[2], ei,
                                         ttype == BRepTrimType::Mated ? "MATED" : "new");
                    }
                    int ci2d = result.add_curve_2d(pcp);
                    result.add_trim(ci2d, ei, li, false, ttype);
                }
            }
        }
    };

    int src_fi = -1;
    // record the originating face for every appended result face (angle-method classify)
    auto flush_src = [&]() {
        if (!face_src_out) return;
        while (face_src_out->size() < result.m_faces.size()) face_src_out->push_back(src_fi);
    };
    for (const auto& face : m_faces) {
        flush_src();   // faces appended by the previous iteration belong to prev src_fi
        ++src_fi;
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        std::vector<NurbsCurve> outer_pcs;
        std::vector<std::vector<NurbsCurve>> inner_loops;
        bool has_inner = false;
        scaf_bnd_edge_ids.clear();
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const BRepLoop& bloop = m_loops[li];
            std::vector<NurbsCurve> pcs;
            std::vector<int> pcs_eids;
            for (int ti : bloop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 >= 0 && c2 < (int)m_curves_2d.size()) {
                    pcs.push_back(m_curves_2d[c2]);
                    pcs_eids.push_back(m_trims[ti].edge_index);
                }
            }
            if (bloop.type == BRepLoopType::Inner) {
                has_inner = true;
                inner_loops.push_back(pcs);
            } else {
                outer_pcs = pcs;
                scaf_bnd_edge_ids = pcs_eids;   // parallel to outer_pcs == all_pcs[0..n_boundary)
            }
        }

        auto pf_t0 = pf_now();
        // OCCT-adoption S2: scaffold mode replaces the per-operand cuts with the SHARED
        // section segments' UV chains (pre-noded, pre-clipped, keep-verdicted once for both
        // operands) -- the legacy clip/border/snap-bridge stages below are skipped because
        // they would disturb the cut-index -> seg_id alignment.
        scaf_cut_seg_ids.clear();
        scaf_cut_spans.clear();
        double scaf_forced_eps = 0.0;
        std::vector<NurbsCurve> cut_pcs;
        if (scaf) {
            // Scaffold segments end exactly ON the trim boundary (paved there); the
            // arrangement only nodes transversal CROSSINGS, so a tangent-touching end is
            // valence-1 and the dangling prune eats the cut. Overshoot each open end past
            // the boundary (the proven TRIM_SNAP-bridge pattern) so a crossing node forms;
            // the run's true span [t_lo, t_hi] is remembered for append_face's seg mapping.
            auto duF = srf.domain(0); auto dvF = srf.domain(1);
            double min_rangeF = std::min(duF.second - duF.first, dvF.second - dvF.first);
            double ov = min_rangeF * 1e-2;
            // UV image of the scaffold weld tolerance: 3D pave dedup can displace a cut
            // end by up to tol3*0.5 from THIS side's boundary; the forced-node/stub
            // rescue must cover that in this face's UV scale (min_range*1e-2 alone is
            // too small on skinny chair faces -- the B-side parts=1 class). Capped so
            // true interior junctions (>=0.15 UV away) are never dragged to the boundary.
            {
                double muF = (duF.first + duF.second) * 0.5, mvF2 = (dvF.first + dvF.second) * 0.5;
                double hu2 = (duF.second - duF.first) / 64.0, hv2 = (dvF.second - dvF.first) / 64.0;
                Point pmF = srf.point_at(muF, mvF2);
                double j_u = pmF.distance(srf.point_at(muF + hu2, mvF2)) / hu2;
                double j_v = pmF.distance(srf.point_at(muF, mvF2 + hv2)) / hv2;
                double uv3dF = std::max(j_u, j_v);
                if (uv3dF < 1e-10) uv3dF = 1.0;
                // Cap 1.3e-1 (was 6e-2): a section junction whose 3D point sits a graze-
                // distance OFF this surface lands its closest-point uv up to ~tol3-scale
                // INTO the face (y30 seg4: 0.0514 vs old cap 0.0236 -> forced node MISSed,
                // cut dangled, face never split). True interior junctions are >=0.15 away.
                scaf_forced_eps = std::min(std::max(min_rangeF * 1e-2, 0.6 * scaf->tol3 / uv3dF),
                                           min_rangeF * 1.3e-1);
                ov = std::max(ov, scaf_forced_eps * 0.5);
            }
            // Boundary polylines of THIS face (for the border-coincident drop below): a
            // section running ALONG the face's own trim boundary does not cut the face --
            // it derails the arrangement (the legacy off_border lesson). Adaptive to sag:
            // these gate the overshoot stubs, and fixed-count sampling of curvy imported
            // trims sags past the stub gate itself.
            std::vector<std::vector<Point>> bnd_polys_sc;
            for (auto& bp : outer_pcs) {
                auto dbp = bp.domain();
                std::vector<std::pair<double, Point>> ent;
                for (int i2 = 0; i2 <= 64; ++i2) {
                    double t2 = dbp.first + (dbp.second - dbp.first) * i2 / 64.0;
                    ent.push_back({t2, bp.point_at(t2)});
                }
                double sag_tol = min_rangeF * 5e-4;
                for (int pass = 0; pass < 5; ++pass) {
                    int ins = 0;
                    for (size_t i2 = 0; i2 + 1 < ent.size() && ent.size() < 2048; ++i2) {
                        double tm = (ent[i2].first + ent[i2+1].first) * 0.5;
                        Point pm2 = bp.point_at(tm);
                        double exq = ent[i2+1].second[0]-ent[i2].second[0], eyq = ent[i2+1].second[1]-ent[i2].second[1];
                        double L2q = exq*exq + eyq*eyq;
                        double tq = L2q > 1e-30 ? ((pm2[0]-ent[i2].second[0])*exq + (pm2[1]-ent[i2].second[1])*eyq) / L2q : 0.0;
                        tq = std::min(std::max(tq, 0.0), 1.0);
                        double dxq = pm2[0]-ent[i2].second[0]-tq*exq, dyq = pm2[1]-ent[i2].second[1]-tq*eyq;
                        if (dxq*dxq + dyq*dyq > sag_tol * sag_tol) {
                            ent.insert(ent.begin() + i2 + 1, {tm, pm2});
                            ++ins; ++i2;
                        }
                    }
                    if (!ins) break;
                }
                std::vector<Point> ps;
                ps.reserve(ent.size());
                for (auto& e2 : ent) ps.push_back(e2.second);
                bnd_polys_sc.push_back(std::move(ps));
            }
            auto d_bnd_sc = [&](const Point& p) {
                double best = 1e300;
                for (auto& poly : bnd_polys_sc)
                    for (size_t j = 0; j + 1 < poly.size(); ++j) {
                        double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1];
                        double L2 = ex*ex + ey*ey;
                        double t = L2 > 1e-30 ? ((p[0]-poly[j][0])*ex + (p[1]-poly[j][1])*ey) / L2 : 0.0;
                        t = std::min(std::max(t, 0.0), 1.0);
                        double dx = p[0]-poly[j][0]-t*ex, dy = p[1]-poly[j][1]-t*ey;
                        best = std::min(best, dx*dx + dy*dy);
                    }
                return std::sqrt(best);
            };
            // Foot point on the boundary (for the boundary-reaching stub): mirror of d_bnd_sc
            // but returns the closest point, so an undershooting section cut can be extended
            // PAST the boundary along the real direction to it (not merely along its own tangent).
            auto foot_bnd_sc = [&](const Point& p, Point& out) {
                double best = 1e300;
                for (auto& poly : bnd_polys_sc)
                    for (size_t j = 0; j + 1 < poly.size(); ++j) {
                        double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1];
                        double L2 = ex*ex + ey*ey;
                        double t = L2 > 1e-30 ? ((p[0]-poly[j][0])*ex + (p[1]-poly[j][1])*ey) / L2 : 0.0;
                        t = std::min(std::max(t, 0.0), 1.0);
                        double qx = poly[j][0]+t*ex, qy = poly[j][1]+t*ey;
                        double dx = p[0]-qx, dy = p[1]-qy, d2 = dx*dx + dy*dy;
                        if (d2 < best) { best = d2; out = Point(qx, qy, 0.0); }
                    }
                return std::sqrt(best);
            };
            double eps_border = std::min(duF.second - duF.first, dvF.second - dvF.first) * 2e-3;
            scaf_forced_nodes.clear();
            const auto& ids = scaf_is_A ? scaf->segs_by_surfA[face.surface_index]
                                        : scaf->segs_by_surfB[face.surface_index];
            // Junction endpoints (shared by >=2 segments on this face, bit-equal after
            // canonicalization) must NEVER receive overshoot stubs: a junction that
            // happens to sit near the trim gets both incident tips displaced in
            // DIVERGENT directions (+-ov), destroying the weld (the si=19 class).
            // The forced-node path handles near-boundary junctions instead.
            std::map<std::pair<double, double>, int> end_count;
            for (int sid : ids) {
                const SectionSegment& s = scaf->segments[sid];
                const auto& uv = scaf_is_A ? s.uvA : s.uvB;
                if (uv.size() < 2 || s.v_start == s.v_end) continue;
                end_count[{uv.front()[0], uv.front()[1]}] += 1;
                end_count[{uv.back()[0], uv.back()[1]}] += 1;
            }
            for (int sid : ids) {
                const SectionSegment& s = scaf->segments[sid];
                const auto& uv = scaf_is_A ? s.uvA : s.uvB;
                if (uv.size() < 2) continue;
                // drop segments coincident with this face's own boundary (all probes ON it).
                // SESSION_KEEP_BORDER keeps them: at a graze along the boundary the section
                // must still carve the thin strip (inside the other solid -> removed), else
                // the other operand's rim meets an UNSPLIT 2-trim edge = nonmanifold (y30).
                bool on_border = true;
                for (int qk = 0; qk <= 4 && on_border; ++qk) {
                    size_t qi = (uv.size() - 1) * qk / 4;
                    on_border = d_bnd_sc(uv[qi]) < eps_border;
                }
                if (on_border && !std::getenv("SESSION_KEEP_BORDER")) continue;
                // pave endpoints become forced boundary nodes: the arrangement's boundary
                // polyline passes EXACTLY through them, so the cut connects via the shared
                // vertex no matter how grazing the crossing is (OCCT FaceInfo.On analog)
                if (s.v_start != s.v_end) {
                    scaf_forced_nodes.push_back(uv.front());
                    scaf_forced_nodes.push_back(uv.back());
                }
                bool closed_seg = s.v_start == s.v_end;
                // Overshoot ONLY ends that sit on this face's own boundary: the stub exists
                // to force a boundary crossing; at interior junction paves (segment continues
                // onto another segment) both incident cuts share the endpoint EXACTLY, so the
                // vertex pool welds them -- a stub there only injects noise crossings.
                // Stub policy (the v444 angular-tie lesson): an end the FORCED-NODE pass
                // will catch (within eps of the boundary) must NOT get a stub -- the
                // fnode snap would relocate the stub TIP onto the boundary, leaving a
                // micro-chord that runs COLLINEAR with the boundary (exact angular tie
                // in the leftmost-turn walk -> cut traversed as a slit -> no split).
                // The raw pave end snaps to the inserted foot node and the first chord
                // heads into the interior: clean valence-3 connection, OCCT-style.
                // Stubs remain only for the band the fnode pass cannot reach.
                std::vector<Point> pts;
                int n_pre = 0, n_post = 0;
                bool jf = !closed_seg && end_count[{uv.front()[0], uv.front()[1]}] >= 2;
                bool jb = !closed_seg && end_count[{uv.back()[0], uv.back()[1]}] >= 2;
                double d_f = (closed_seg || jf) ? 1e300 : d_bnd_sc(uv.front());
                double d_b = (closed_seg || jb) ? 1e300 : d_bnd_sc(uv.back());
                bool stub_reach = std::getenv("SESSION_STUB_REACH") != nullptr;  // opt-in (mixed on rotated cfgs)
                if (d_f >= scaf_forced_eps * 0.9 && d_f < std::max(ov * 5.0, scaf_forced_eps * 2.0)) {
                    if (stub_reach && d_f > ov) {
                        // BOUNDARY-REACHING STUB: a fixed-length ov stub lands SHORT whenever the
                        // trim boundary is farther than ov (the undershoot band ov < d_f <= 5*ov).
                        // The short stub then fails to cross the boundary -> the cut's open end is
                        // an interior valence-1 vertex -> the arrangement's dangling-prune deletes
                        // the whole cut -> the face is emitted UNCUT (oversized fragment, wrong
                        // classification, 1-trim naked section). Project onto the boundary and place
                        // the tip PAST the foot so the cut definitely crosses and SPLITS the face.
                        Point foot; foot_bnd_sc(uv.front(), foot);
                        double fdx = foot[0]-uv[0][0], fdy = foot[1]-uv[0][1], fl = std::hypot(fdx, fdy);
                        if (fl > 1e-30) {
                            pts.push_back(Point(foot[0] + ov*fdx/fl, foot[1] + ov*fdy/fl, 0.0));
                            n_pre = 1;
                        }
                    } else {
                        double dx = uv[0][0] - uv[1][0], dy = uv[0][1] - uv[1][1];
                        double L = std::hypot(dx, dy);
                        if (L > 1e-30) {
                            pts.push_back(Point(uv[0][0] + dx / L * ov, uv[0][1] + dy / L * ov, 0.0));
                            n_pre = 1;
                        }
                    }
                }
                for (const auto& q : uv) pts.push_back(q);
                if (d_b >= scaf_forced_eps * 0.9 && d_b < std::max(ov * 5.0, scaf_forced_eps * 2.0)) {
                    if (stub_reach && d_b > ov) {
                        Point foot; foot_bnd_sc(uv.back(), foot);
                        double fdx = foot[0]-uv.back()[0], fdy = foot[1]-uv.back()[1], fl = std::hypot(fdx, fdy);
                        if (fl > 1e-30) {
                            pts.push_back(Point(foot[0] + ov*fdx/fl, foot[1] + ov*fdy/fl, 0.0));
                            n_post = 1;
                        }
                    } else {
                        size_t n = uv.size();
                        double dx = uv[n-1][0] - uv[n-2][0], dy = uv[n-1][1] - uv[n-2][1];
                        double L = std::hypot(dx, dy);
                        if (L > 1e-30) {
                            pts.push_back(Point(uv[n-1][0] + dx / L * ov, uv[n-1][1] + dy / L * ov, 0.0));
                            n_post = 1;
                        }
                    }
                }
                NurbsCurve pc = NurbsCurve::create(false, 1, pts);
                if (!pc.is_valid()) continue;
                // deg-1 create() is UNIFORM-BY-INDEX (rescaled to [0,L]): polyline vertex k
                // sits at param k*L/(npts-1), NOT at its chord-length position. Record the
                // TRUE segment ends in that exact param space.
                int npts = (int)pts.size();
                double L = pc.domain().second;
                double step = npts > 1 ? L / (npts - 1) : L;
                cut_pcs.push_back(pc);
                scaf_cut_seg_ids.push_back(sid);
                scaf_cut_spans.push_back({n_pre * step, (n_pre + (int)uv.size() - 1) * step});
                g_fed_cuts.insert({face.surface_index, sid});
            }
            // PutSEInOtherFaces (OCCT PaveFiller): a section segment from ANOTHER surface
            // pair whose 3D chain lies ON this surface must also cut THIS face -- the
            // junction network on a face otherwise stays open where the section transits
            // through a graze (y30 segs 4/5/17/18: interior dangling end 0.47 from the
            // boundary = the un-imprinted continuation from the neighbouring pair).
            // OPT-IN (SESSION_SE_OTHER): one-sided imports create MORE unmated copies
            // (y30 13->31) until the import is made symmetric (both operands' faces that
            // contain the footprint must import the SAME piece) -- next design pass.
            if (std::getenv("SESSION_SE_OTHER")) {
                std::set<int> mine(ids.begin(), ids.end());
                double on_tol3 = scaf->tol3 * 1.2;
                for (int sid2 = 0; sid2 < (int)scaf->segments.size(); ++sid2) {
                    if (mine.count(sid2)) continue;
                    const SectionSegment& s2 = scaf->segments[sid2];
                    int nCh2 = (int)s2.p3.size();
                    if (nCh2 < 2) continue;
                    // sparse ON probe first (cheap reject)
                    int hits = 0;
                    for (int k = 0; k < 5; ++k) {
                        const Point& q = s2.p3[(size_t)((nCh2 - 1) * k / 4)];
                        auto pr = Closest::surface_point(srf, q, 0.0, 0.0, 0.0, 0.0);
                        Point sp3 = srf.point_at(std::get<0>(pr), std::get<1>(pr));
                        if (sp3.distance(q) < on_tol3) ++hits;
                    }
                    if (hits < 2) continue;
                    // full footprint: longest contiguous ON run of chain points
                    std::vector<Point> uvloc;
                    int best_ka = -1, best_kb = -1, ka = -1;
                    std::vector<Point> uvall(nCh2, Point(0, 0, 0));
                    std::vector<char> on(nCh2, 0);
                    for (int k = 0; k < nCh2; ++k) {
                        auto pr = Closest::surface_point(srf, s2.p3[k], 0.0, 0.0, 0.0, 0.0);
                        Point sp3 = srf.point_at(std::get<0>(pr), std::get<1>(pr));
                        if (sp3.distance(s2.p3[k]) < on_tol3) {
                            on[k] = 1;
                            uvall[k] = Point(std::get<0>(pr), std::get<1>(pr), 0.0);
                        }
                    }
                    for (int k = 0; k <= nCh2; ++k) {
                        if (k < nCh2 && on[k]) { if (ka < 0) ka = k; }
                        else if (ka >= 0) {
                            if (best_ka < 0 || k - 1 - ka > best_kb - best_ka) { best_ka = ka; best_kb = k - 1; }
                            ka = -1;
                        }
                    }
                    if (best_ka < 0 || best_kb - best_ka < 1) continue;
                    // require real 3D extent (not a point contact)
                    double ext3 = s2.p3[best_ka].distance(s2.p3[best_kb]);
                    if (ext3 < scaf->tol3) continue;
                    for (int k = best_ka; k <= best_kb; ++k) uvloc.push_back(uvall[k]);
                    NurbsCurve pc2 = NurbsCurve::create(false, 1, uvloc);
                    if (!pc2.is_valid()) continue;
                    // affine (t_lo,t_hi) so append_face's chain_pos maps the pcurve's
                    // [0,L] onto chain indices [best_ka, best_kb] exactly
                    double L2 = pc2.domain().second;
                    double slope = (double)(best_kb - best_ka);
                    if (slope < 1e-12 || L2 < 1e-12) continue;
                    double D = L2 * (double)(nCh2 - 1) / slope;
                    double t_lo = -(double)best_ka * L2 / slope;
                    cut_pcs.push_back(pc2);
                    scaf_cut_seg_ids.push_back(sid2);
                    scaf_cut_spans.push_back({t_lo, t_lo + D});
                    scaf_forced_nodes.push_back(uvloc.front());
                    scaf_forced_nodes.push_back(uvloc.back());
                    if (std::getenv("SESSION_SPLIT_DBG"))
                        std::fprintf(stderr, "[SEOTHER] si=%d seg=%d ka=%d kb=%d ext=%.4f\n",
                                     face.surface_index, sid2, best_ka, best_kb, ext3);
                }
            }
            // Same-domain imprint cuts (coincident-face regions have NO SSI section --
            // the other operand's boundary projected onto this surface). seg id -1 routes
            // them through the legacy lift; the split then produces an ON fragment here
            // that the keep-one-copy classification rule can pair with the other side's.
            if (extra_cuts && face.surface_index < (int)extra_cuts->size()) {
                for (const auto& pc : (*extra_cuts)[face.surface_index]) {
                    cut_pcs.push_back(pc);
                    scaf_cut_seg_ids.push_back(-1);
                    scaf_cut_spans.push_back({0.0, 0.0});
                }
            }
        } else {
            cut_pcs = (have_cut_cache && src_fi >= 0 && src_fi < (int)face_cut_cache.size())
                          ? face_cut_cache[src_fi]
                          : cut_for(srf);
        }
        if (s_prof) prof_ssi += pf_us(pf_t0, pf_now());
        if (cut_pcs.empty() || has_inner) {
            std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>> loops;
            loops.push_back({BRepLoopType::Outer, outer_pcs});
            for (auto& il : inner_loops) loops.push_back({BRepLoopType::Inner, il});
            auto pf_t1 = pf_now();
            append_face(srf, loops);
            if (s_prof) prof_lift += pf_us(pf_t1, pf_now());
            continue;
        }

        // Clip cut pcurves to the chart rectangle in NON-PERIODIC directions: a section
        // loop can exit the face's domain (the blob's silhouette at x=-2 reaches past the
        // wall's edge, its pullback dips to v=-0.05) and the arrangement cannot handle
        // out-of-domain cuts -- the wall face then never splits and loses its hole.
        if (!scaf) {
            auto duS = srf.domain(0); auto dvS = srf.domain(1);
            bool cu2 = srf.is_closed(0), cv2 = srf.is_closed(1);
            std::vector<NurbsCurve> clipped;
            for (auto& pcC : cut_pcs) {
                auto dc2 = pcC.domain();
                int nS = std::max(96, pcC.cv_count() * 4);
                auto inside = [&](const Point& q) {
                    if (!cu2 && (q[0] < duS.first - 1e-12 || q[0] > duS.second + 1e-12)) return false;
                    if (!cv2 && (q[1] < dvS.first - 1e-12 || q[1] > dvS.second + 1e-12)) return false;
                    return true;
                };
                std::vector<Point> samp(nS + 1);
                bool all_in = true;
                for (int i2 = 0; i2 <= nS; ++i2) {
                    samp[i2] = pcC.point_at(dc2.first + (dc2.second - dc2.first) * i2 / nS);
                    if (!inside(samp[i2])) all_in = false;
                }
                if (all_in) { clipped.push_back(pcC); continue; }
                bool pc_closed = samp.front().distance(samp.back()) < 1e-9;
                std::vector<Point> ring(samp.begin(), samp.end() - (pc_closed ? 1 : 0));
                int N2 = (int)ring.size();
                int start = 0;
                if (pc_closed) {
                    for (int i2 = 0; i2 < N2; ++i2)
                        if (!inside(ring[i2])) { start = i2; break; }
                }
                auto clampB = [&](Point q) {
                    if (!cu2) q = Point(std::max(duS.first, std::min(q[0], duS.second)), q[1], 0.0);
                    if (!cv2) q = Point(q[0], std::max(dvS.first, std::min(q[1], dvS.second)), 0.0);
                    return q;
                };
                auto cross_pt = [&](const Point& a2, const Point& b2) {
                    // interpolate to the violated border between an inside and outside sample
                    double tbest = 1.0;
                    if (!cu2) {
                        if (b2[0] < duS.first)  tbest = std::min(tbest, (duS.first  - a2[0]) / (b2[0] - a2[0]));
                        if (b2[0] > duS.second) tbest = std::min(tbest, (duS.second - a2[0]) / (b2[0] - a2[0]));
                    }
                    if (!cv2) {
                        if (b2[1] < dvS.first)  tbest = std::min(tbest, (dvS.first  - a2[1]) / (b2[1] - a2[1]));
                        if (b2[1] > dvS.second) tbest = std::min(tbest, (dvS.second - a2[1]) / (b2[1] - a2[1]));
                    }
                    tbest = std::max(0.0, std::min(1.0, tbest));
                    return clampB(Point(a2[0] + (b2[0]-a2[0]) * tbest, a2[1] + (b2[1]-a2[1]) * tbest, 0.0));
                };
                int total = pc_closed ? N2 : N2 - 1;
                std::vector<Point> run;
                auto flush = [&]() {
                    if (run.size() >= 2) {
                        NurbsCurve seg = NurbsCurve::create(false, 1, run);
                        if (seg.is_valid()) clipped.push_back(seg);
                    }
                    run.clear();
                };
                for (int k2 = 0; k2 <= total; ++k2) {
                    int i2 = pc_closed ? (start + k2) % N2 : k2;
                    const Point& q = ring[i2];
                    if (inside(q)) {
                        if (run.empty() && k2 > 0) {
                            int ip = pc_closed ? (start + k2 - 1 + N2) % N2 : k2 - 1;
                            run.push_back(cross_pt(q, ring[ip]));
                        }
                        run.push_back(q);
                    } else {
                        if (!run.empty()) {
                            run.push_back(cross_pt(run.back(), q));
                            flush();
                        }
                    }
                }
                flush();
            }
            cut_pcs = clipped;
            // Drop cut pcurves that run ALONG a non-periodic chart border: a section with
            // a face that merely touches this chart's edge (coplanar-face contact, e.g.
            // overlap boxes sharing z-planes) cannot split the region -- but it derails
            // the arrangement, losing the legitimate interior splits (walls stayed
            // unsplit and the result lost its caps).
            double bu = (duS.second - duS.first) * 1e-6;
            double bv = (dvS.second - dvS.first) * 1e-6;
            std::vector<NurbsCurve> off_border;
            for (auto& pcB : cut_pcs) {
                auto dB = pcB.domain();
                bool lo_u = true, hi_u = true, lo_v = true, hi_v = true;
                for (int i2 = 0; i2 <= 16; ++i2) {
                    Point q = pcB.point_at(dB.first + (dB.second - dB.first) * i2 / 16.0);
                    lo_u = lo_u && std::abs(q[0] - duS.first)  < bu;
                    hi_u = hi_u && std::abs(q[0] - duS.second) < bu;
                    lo_v = lo_v && std::abs(q[1] - dvS.first)  < bv;
                    hi_v = hi_v && std::abs(q[1] - dvS.second) < bv;
                }
                bool border = (!cu2 && (lo_u || hi_u)) || (!cv2 && (lo_v || hi_v));
                if (!border) off_border.push_back(pcB);
            }
            cut_pcs = off_border;
            // Drop cut pcurves that coincide with the face's OWN boundary loops: the cutter
            // meets this face exactly along an existing trim edge (xor of cut results: the
            // section circle IS the wall face's hole boundary). Re-cutting along the boundary
            // only seeds sliver cells in the arrangement.
            if (!cut_pcs.empty() && !outer_pcs.empty()) {
                double on_tol = std::max(duS.second - duS.first, dvS.second - dvS.first) * 1e-5;
                auto pt_to_poly2 = [](const Point& p, const std::vector<Point>& poly) {
                    double best = 1e300;
                    for (size_t j = 0; j + 1 < poly.size(); ++j) {
                        double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1];
                        double L2 = ex*ex + ey*ey;
                        double t = L2 > 1e-30 ? ((p[0]-poly[j][0])*ex + (p[1]-poly[j][1])*ey) / L2 : 0.0;
                        t = std::min(std::max(t, 0.0), 1.0);
                        double dx = p[0]-poly[j][0]-t*ex, dy = p[1]-poly[j][1]-t*ey;
                        best = std::min(best, dx*dx + dy*dy);
                    }
                    return std::sqrt(best);
                };
                std::vector<std::vector<Point>> bnd_polys;
                for (auto& bp : outer_pcs) {
                    auto db3 = bp.domain();
                    std::vector<Point> ps(65);
                    for (int i = 0; i <= 64; ++i)
                        ps[i] = bp.point_at(db3.first + (db3.second - db3.first) * i / 64.0);
                    bnd_polys.push_back(std::move(ps));
                }
                std::vector<NurbsCurve> off_bnd;
                for (auto& pcB : cut_pcs) {
                    auto dB = pcB.domain();
                    bool all_on = true;
                    for (int i = 0; i <= 16 && all_on; ++i) {
                        Point q = pcB.point_at(dB.first + (dB.second - dB.first) * i / 16.0);
                        double best = 1e300;
                        for (auto& bp : bnd_polys) best = std::min(best, pt_to_poly2(q, bp));
                        if (best > on_tol) all_on = false;
                    }
                    if (!all_on) off_bnd.push_back(pcB);
                }
                cut_pcs = off_bnd;
            }
        }
        // Boundary-snap bridges (opt-in via SESSION_TRIM_SNAP). Imported freeform cutters
        // (STEP chairs) produce SSI section pcurves whose ends land a hair OFF the face
        // boundary (db~1e-3) or float mid-face at a B-edge crossing. The arrangement only
        // nodes true segment crossings, so a cut ending *near but not on* the boundary is
        // valence-1 -> pruned as dangling -> the whole face collapses to parts=0 (uncut).
        // For each cut endpoint within snap_hi of the boundary we append a short linear
        // bridge that OVERSHOOTS the boundary polyline, forcing a crossing node so the cut
        // survives the prune and actually partitions the face. Default off => byte-identical.
        if (const char* snap_env = scaf ? nullptr : std::getenv("SESSION_TRIM_SNAP")) {
            double snap_hi = std::atof(snap_env);
            if (snap_hi <= 0.0) snap_hi = 0.05;
            const double snap_lo = 1e-6;
            std::vector<std::vector<Point>> snap_bnd;
            for (auto& bp : outer_pcs) {
                auto db3 = bp.domain();
                std::vector<Point> ps(129);
                for (int i = 0; i <= 128; ++i)
                    ps[i] = bp.point_at(db3.first + (db3.second - db3.first) * i / 128.0);
                snap_bnd.push_back(std::move(ps));
            }
            auto proj_bnd = [&](const Point& p, Point& out) {
                double best = 1e300;
                for (auto& poly : snap_bnd)
                    for (size_t j = 0; j + 1 < poly.size(); ++j) {
                        double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1];
                        double L2 = ex*ex + ey*ey;
                        double t = L2 > 1e-30 ? ((p[0]-poly[j][0])*ex + (p[1]-poly[j][1])*ey) / L2 : 0.0;
                        t = std::min(std::max(t, 0.0), 1.0);
                        double qx = poly[j][0]+t*ex, qy = poly[j][1]+t*ey;
                        double dx = p[0]-qx, dy = p[1]-qy, d2 = dx*dx + dy*dy;
                        if (d2 < best) { best = d2; out = Point(qx, qy, 0.0); }
                    }
                return std::sqrt(best);
            };
            double ov = 0.4;
            if (const char* e2 = std::getenv("SESSION_TRIM_SNAP_OV")) ov = std::atof(e2);
            std::vector<NurbsCurve> bridges;
            for (auto& cut : cut_pcs) {
                auto dc = cut.domain();
                for (double param : {dc.first, dc.second}) {
                    Point e = cut.point_at(param), proj;
                    double d = proj_bnd(e, proj);
                    if (d > snap_lo && d < snap_hi) {
                        // overshoot past the boundary so the bridge definitely crosses it
                        double dx = proj[0]-e[0], dy = proj[1]-e[1];
                        Point tip(proj[0] + ov*dx, proj[1] + ov*dy, 0.0);
                        bridges.push_back(NurbsCurve::create(false, 1, {e, tip}));
                    }
                }
            }
            if (std::getenv("SESSION_SPLIT_DBG")) {
                std::fprintf(stderr, "[SNAP] si=%d added %zu bridges (snap_hi=%.4f ov=%.2f)\n",
                             face.surface_index, bridges.size(), snap_hi, ov);
                std::fflush(stderr);
            }
            for (auto& br : bridges) cut_pcs.push_back(br);
        }
        int n_boundary = (int)outer_pcs.size();
        std::vector<NurbsCurve> all_pcs = outer_pcs;
        all_pcs.insert(all_pcs.end(), cut_pcs.begin(), cut_pcs.end());
        scaf_n_boundary = n_boundary;   // S2: append_face maps run cidx -> seg via these
        all_pcs_ref = &all_pcs;
        if (std::getenv("SESSION_SPLIT_DBG")) {
            std::fprintf(stderr, "[PRESPLIT] si=%d nbnd=%d totpc=%zu\n",
                         face.surface_index, n_boundary, all_pcs.size());
            std::fflush(stderr);
        }
        auto pf_t2 = pf_now();
        // Seam-aware WireSplitter arrangement is opt-in via SESSION_WIRESPLIT; with the flag unset
        // the kernel is byte-identical to the proven split_by_uv_curves path.
        static const bool s_wiresplit = (std::getenv("SESSION_WIRESPLIT") != nullptr);
        // Near-boundary cut-endpoint snapping (T-junction resolution for imported freeform
        // sections). Currently EXPERIMENTAL / env-gated only: SESSION_BND_SNAP=<uv> activates it
        // in split_by_uv_curves. Auto-enabling on the trim-cut path is deferred until the
        // downstream crash on degenerate snapped 2-point cuts (chair si=16) is fixed and the
        // matrix is re-verified. Default 0.0 => byte-identical to the proven path.
        // Near-boundary cut-endpoint snap: default-on ONLY for freeform x freeform (imported
        // STEP) operands; every matrix pair has a recognized primitive so this stays 0.0 there.
        // 0.05 validated on the chair pair (all 20 faces of both operands split or pass through
        // correctly); SESSION_BND_SNAP still overrides inside split_by_uv_curves.
        // Scaffold cuts are pre-noded (paved at trim crossings + overshot past the boundary):
        // the endpoint projection/jweld machinery, built for UNPAVED legacy cuts, would MOVE
        // those nodes by up to 0.05 UV and desync the run spans from the scaffold paves.
        double snap_bnd = (imported_freeform && !scaf) ? 0.05 : 0.0;
        guard_check_mem("arrangement");
        std::vector<NurbsSurfaceTrimmed> parts = s_wiresplit
            ? NurbsSurfaceTrimmed::split_face_by_wires(srf, cut_pcs, outer_pcs, tolerance)
            : NurbsSurfaceTrimmed::split_by_uv_curves(srf, all_pcs, tolerance, false, n_boundary, snap_bnd,
                                                      (scaf && !scaf_forced_nodes.empty()) ? &scaf_forced_nodes : nullptr,
                                                      scaf_forced_eps);
        if (s_prof) prof_arr += pf_us(pf_t2, pf_now());
        if (std::getenv("SESSION_SPLIT_DBG")) {
            std::fprintf(stderr, "[SPLIT] si=%d nbnd=%d cuts=%zu parts=%zu\n",
                         face.surface_index, n_boundary, cut_pcs.size(), parts.size());
            std::fflush(stderr);
            // Sample each boundary pcurve for endpoint-to-boundary distance queries.
            std::vector<std::vector<Point>> dbg_bnd;
            for (auto& bp : outer_pcs) {
                auto db3 = bp.domain();
                std::vector<Point> ps(65);
                for (int i = 0; i <= 64; ++i)
                    ps[i] = bp.point_at(db3.first + (db3.second - db3.first) * i / 64.0);
                dbg_bnd.push_back(std::move(ps));
            }
            auto dist_bnd = [&](const Point& p) {
                double best = 1e300;
                for (auto& poly : dbg_bnd)
                    for (size_t j = 0; j + 1 < poly.size(); ++j) {
                        double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1];
                        double L2 = ex*ex + ey*ey;
                        double t = L2 > 1e-30 ? ((p[0]-poly[j][0])*ex + (p[1]-poly[j][1])*ey) / L2 : 0.0;
                        t = std::min(std::max(t, 0.0), 1.0);
                        double dx = p[0]-poly[j][0]-t*ex, dy = p[1]-poly[j][1]-t*ey;
                        best = std::min(best, dx*dx + dy*dy);
                    }
                return std::sqrt(best);
            };
            for (auto& pcq : cut_pcs) {
                auto dq = pcq.domain();
                Point a2 = pcq.point_at(dq.first), b2 = pcq.point_at(dq.second);
                double xmn = 1e300, xmx = -1e300, ymn = 1e300, ymx = -1e300;
                for (int q2 = 0; q2 <= 32; ++q2) {
                    Point pq = pcq.point_at(dq.first + (dq.second - dq.first) * q2 / 32.0);
                    xmn = std::min(xmn, pq[0]); xmx = std::max(xmx, pq[0]);
                    ymn = std::min(ymn, pq[1]); ymx = std::max(ymx, pq[1]);
                }
                std::fprintf(stderr, "   cut a(%.3f,%.3f)|db=%.4f b(%.3f,%.3f)|db=%.4f closed=%d bbox[%.2f,%.2f]x[%.2f,%.2f]\n",
                             a2[0], a2[1], dist_bnd(a2), b2[0], b2[1], dist_bnd(b2),
                             a2.distance(b2) < 1e-6 ? 1 : 0, xmn, xmx, ymn, ymx);
            }
        }
        if (parts.size() <= 1) {
            std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>> loops;
            loops.push_back({BRepLoopType::Outer, outer_pcs});
            auto pf_t3 = pf_now();
            if (scaf) {
                // Pass-through faces must take the SAME boundary pave-block path as split
                // fragments (their whole-edge runs subdivide at the shared paves), else
                // every shared edge with a split neighbour is a T-junction.
                std::vector<std::array<double, 3>> synth;
                for (size_t i2 = 0; i2 < outer_pcs.size(); ++i2) {
                    auto d2 = outer_pcs[i2].domain();
                    synth.push_back({(double)i2, d2.first, d2.second});
                }
                std::vector<const std::vector<std::array<double, 3>>*> ls{&synth};
                append_face(srf, loops, &ls);
            } else {
                append_face(srf, loops);
            }
            if (s_prof) prof_lift += pf_us(pf_t3, pf_now());
            continue;
        }
        auto pf_t4 = pf_now();
        static const bool s_liftdbg = (std::getenv("SESSION_TRIM_DBG") != nullptr);
        int _pk = 0;
        for (const auto& part : parts) {
            if (s_liftdbg) {
                std::fprintf(stderr, "[LIFTPART] si=%d part %d/%zu outer_valid=%d segs=%zu inner=%zu\n",
                             face.surface_index, _pk, parts.size(), part.m_outer_loop.is_valid() ? 1 : 0,
                             part.m_outer_segments.size(), part.m_inner_loops.size());
                std::fflush(stderr);
            }
            ++_pk;
            // Prefer the per-run segmentation (each boundary run a separate pcurve) so each
            // run lifts to its own edge and mates with the matching segment edge of an
            // adjacent face -> watertight imprint. Fall back to the single joined loop.
            std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>> loops;
            std::vector<const std::vector<std::array<double, 3>>*> loop_srcs;
            loops.push_back({BRepLoopType::Outer,
                part.m_outer_segments.empty() ? std::vector<NurbsCurve>{part.m_outer_loop}
                                              : part.m_outer_segments});
            loop_srcs.push_back(part.m_outer_segments.empty() ? nullptr : &part.m_outer_segment_srcs);
            // SEAM CO-REGIONS (SESSION_SEAM_MERGE): additional OUTER wires of the SAME face --
            // the second side of a region that meets itself across the u (or v) seam. Empty
            // unless the splitter merged, so the default path is unchanged.
            for (size_t k = 0; k < part.m_extra_outer_loops.size(); ++k) {
                const std::vector<NurbsCurve>& xsegs =
                    (k < part.m_extra_outer_segments.size()) ? part.m_extra_outer_segments[k]
                                                             : std::vector<NurbsCurve>{};
                loops.push_back({BRepLoopType::Outer,
                    xsegs.empty() ? std::vector<NurbsCurve>{part.m_extra_outer_loops[k]} : xsegs});
                loop_srcs.push_back((!xsegs.empty() && k < part.m_extra_outer_segment_srcs.size())
                                        ? &part.m_extra_outer_segment_srcs[k] : nullptr);
            }
            for (size_t k = 0; k < part.m_inner_loops.size(); ++k) {
                const std::vector<NurbsCurve>& isegs =
                    (k < part.m_inner_segments.size()) ? part.m_inner_segments[k]
                                                       : std::vector<NurbsCurve>{};
                loops.push_back({BRepLoopType::Inner,
                    isegs.empty() ? std::vector<NurbsCurve>{part.m_inner_loops[k]} : isegs});
                loop_srcs.push_back((!isegs.empty() && k < part.m_inner_segment_srcs.size())
                                        ? &part.m_inner_segment_srcs[k] : nullptr);
            }
            append_face(part.m_surface, loops, scaf ? &loop_srcs : nullptr);
        }
        if (s_prof) prof_lift += pf_us(pf_t4, pf_now());
    }
    if (s_prof) std::fprintf(stderr, "[split-prof]     ssi=%.1f arr=%.1f lift=%.1f us\n", prof_ssi, prof_arr, prof_lift);
    if (scaf && std::getenv("SESSION_SPLIT_DBG")) {
        std::fprintf(stderr, "[SCAF-SPLIT] side=%c sec_edges=%zu fallbacks=%d\n",
                     scaf_is_A ? 'A' : 'B', sec_emap.size(), scaf_fallbacks);
        std::fflush(stderr);
    }

    flush_src();   // faces appended by the last iteration
    // WITHIN-OPERAND COMMON-BLOCK PASS (OCCT PerformEE coincidence, tolerance-model form):
    // two 1-trim NON-SECTION edges between the SAME welded vertex pair whose curves
    // coincide within the SUM of their tube tolerances are the two faces' copies of one
    // physical edge that escaped every identity key (chord-fallback runs, joined
    // pass-through loops). Merge them at construction, where OCCT merges them -- not in a
    // global post-hoc Hausdorff sew.
    if (!pave_pts.empty() && !s_no_pavesnap) {
        std::vector<int> uses(result.m_topology_edges.size(), 0);
        for (const auto& t : result.m_trims)
            if (t.edge_index >= 0 && t.edge_index < (int)uses.size()) ++uses[t.edge_index];
        std::map<std::pair<int, int>, std::vector<int>> by_vp;
        for (int ei = 0; ei < (int)uses.size(); ++ei) {
            if (uses[ei] != 1) continue;
            // S-class (chain-lifted section) edges participate too: a closed/partial section
            // imprint whose OTHER flank fell to a legacy chord (SEGFALL) leaves a 1-trim S +
            // a coincident 1-trim L that only a geometric common-block can pair.
            const auto& E = result.m_topology_edges[ei];
            if (E.start_vertex < 0 || E.end_vertex < 0) continue;
            by_vp[{std::min(E.start_vertex, E.end_vertex),
                   std::max(E.start_vertex, E.end_vertex)}].push_back(ei);
        }
        auto poly_of = [&](int ei) {
            std::vector<Point> ps;
            const auto& E = result.m_topology_edges[ei];
            if (E.curve_3d_index >= 0 && E.curve_3d_index < (int)result.m_curves_3d.size()) {
                const NurbsCurve& c = result.m_curves_3d[E.curve_3d_index];
                auto d = c.domain();
                for (int k = 0; k <= 16; ++k)
                    ps.push_back(c.point_at(d.first + (d.second - d.first) * k / 16.0));
            }
            return ps;
        };
        auto dev_of = [&](int ei) { return ei < (int)edge_dev.size() ? edge_dev[ei] : 0.0; };
        int cb_merged = 0;
        for (auto& kv : by_vp) {
            auto& ids = kv.second;
            if (ids.size() < 2) continue;
            std::vector<char> gone(ids.size(), 0);
            for (size_t a2 = 0; a2 < ids.size(); ++a2) {
                if (gone[a2]) continue;
                std::vector<Point> pa = poly_of(ids[a2]);
                if (pa.size() < 2) continue;
                int acc_trims = 1;                     // 2-trim cap (manifold invariant)
                for (size_t b2 = a2 + 1; b2 < ids.size() && acc_trims < 2; ++b2) {
                    if (gone[b2]) continue;
                    std::vector<Point> pb = poly_of(ids[b2]);
                    if (pb.size() < 2) continue;
                    double tube = 1.5 * (dev_of(ids[a2]) + dev_of(ids[b2])) + 1e-9;
                    bool ok = true;
                    for (int k = 2; k <= 14 && ok; k += 3) {   // interior samples of a vs poly b
                        const Point& q = pa[k];
                        double best = 1e300;
                        for (size_t j = 0; j + 1 < pb.size(); ++j) {
                            double ex = pb[j+1][0]-pb[j][0], ey = pb[j+1][1]-pb[j][1], ez = pb[j+1][2]-pb[j][2];
                            double L2 = ex*ex + ey*ey + ez*ez;
                            double t = L2 > 1e-30 ? ((q[0]-pb[j][0])*ex + (q[1]-pb[j][1])*ey + (q[2]-pb[j][2])*ez) / L2 : 0.0;
                            t = std::min(std::max(t, 0.0), 1.0);
                            double dx = q[0]-pb[j][0]-t*ex, dy = q[1]-pb[j][1]-t*ey, dz = q[2]-pb[j][2]-t*ez;
                            best = std::min(best, dx*dx + dy*dy + dz*dz);
                        }
                        if (best > tube * tube) ok = false;
                    }
                    if (!ok) continue;
                    // survivor preference: the chain-lifted SECTION edge carries the exact
                    // shared geometry AND the span identity (normalize + radial + combine
                    // alias all key on it); merging it INTO a legacy chord erases the seg
                    // from spans and the whole downstream mating chain.
                    int keep_e = ids[a2], dead_e = ids[b2];
                    auto ka = edge_kind_dbg.find(keep_e), kb = edge_kind_dbg.find(dead_e);
                    bool aS = ka != edge_kind_dbg.end() && ka->second == 'S';
                    bool bS = kb != edge_kind_dbg.end() && kb->second == 'S';
                    if (bS && !aS) std::swap(keep_e, dead_e);
                    for (auto& t : result.m_trims)
                        if (t.edge_index == dead_e) { t.edge_index = keep_e; t.type = BRepTrimType::Mated; }
                    gone[b2] = 1;
                    if (keep_e != ids[a2]) {
                        // survivor swapped: continue accumulating onto the section edge
                        ids[a2] = keep_e;
                        pa = poly_of(keep_e);
                    }
                    ++cb_merged;
                    ++acc_trims;
                }
            }
        }
        if (cb_merged) {
            for (auto& e : result.m_topology_edges) e.trim_indices.clear();
            for (int ti = 0; ti < (int)result.m_trims.size(); ++ti) {
                int ei2 = result.m_trims[ti].edge_index;
                if (ei2 >= 0 && ei2 < (int)result.m_topology_edges.size())
                    result.m_topology_edges[ei2].trim_indices.push_back(ti);
            }
        }
        if (cb_merged && std::getenv("SESSION_BEMAP_DBG"))
            std::fprintf(stderr, "[CBLOCK] side=%c merged=%d\n", scaf_is_A ? 'A' : 'B', cb_merged);
    }
    if (std::getenv("SESSION_BEMAP_DBG")) {
        std::vector<int> uses(result.m_topology_edges.size(), 0);
        for (const auto& t : result.m_trims)
            if (t.edge_index >= 0 && t.edge_index < (int)uses.size()) ++uses[t.edge_index];
        int n1[4] = {0,0,0,0};   // S, B, L, unknown
        int shown = 0;
        for (int ei = 0; ei < (int)uses.size(); ++ei) {
            if (uses[ei] != 1) continue;
            auto itk = edge_kind_dbg.find(ei);
            char k = itk == edge_kind_dbg.end() ? '?' : itk->second;
            n1[k == 'S' ? 0 : k == 'B' ? 1 : k == 'L' ? 2 : 3]++;
            if (shown < 40) {
                const auto& E = result.m_topology_edges[ei];
                Point a = result.m_vertices[result.m_topology_vertices[E.start_vertex].point_index];
                Point b = result.m_vertices[result.m_topology_vertices[E.end_vertex].point_index];
                // capture-provenance: how far each endpoint sits from the nearest ORIGINAL
                // corner seed and the nearest PAVE seed (is it in an ambiguity zone, or
                // beyond every capture radius?)
                auto near_seed = [&](const Point& p, double& d_orig, double& d_pave) {
                    d_orig = 1e300; d_pave = 1e300;
                    for (const auto& tv2 : m_topology_vertices)
                        d_orig = std::min(d_orig, m_vertices[tv2.point_index].distance(p));
                    if (scaf) for (const auto& sv2 : scaf->vertices)
                        d_pave = std::min(d_pave, sv2.distance(p));
                };
                double ao, ap, bo, bp;
                near_seed(a, ao, ap);
                near_seed(b, bo, bp);
                std::fprintf(stderr, "[NK1] e%d kind=%c v(%d,%d) a(%.4f,%.4f,%.4f)[o=%.4f p=%.4f] b(%.4f,%.4f,%.4f)[o=%.4f p=%.4f]\n",
                             ei, k, E.start_vertex, E.end_vertex, a[0], a[1], a[2], ao, ap,
                             b[0], b[1], b[2], bo, bp);
                ++shown;
            }
        }
        std::fprintf(stderr, "[NK1] side=%c 1-trim: S=%d B=%d L=%d ?=%d\n",
                     scaf_is_A ? 'A' : 'B', n1[0], n1[1], n1[2], n1[3]);
    }
    for (int ei = 0; ei < (int)result.m_topology_edges.size(); ++ei) {
        int sv = result.m_topology_edges[ei].start_vertex;
        int ev = result.m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)result.m_topology_vertices.size())
            result.m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)result.m_topology_vertices.size())
            result.m_topology_vertices[ev].edge_indices.push_back(ei);
    }
    return result;
}

void BRep::imprint_edges(double tol, bool mated_too) {
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 1e-6;

    // Candidate split positions: every topology-vertex position.
    std::vector<Point> vpos;
    vpos.reserve(m_topology_vertices.size());
    for (const auto& tv : m_topology_vertices)
        vpos.push_back((tv.point_index >= 0 && tv.point_index < (int)m_vertices.size())
                       ? m_vertices[tv.point_index] : Point(1e300, 1e300, 1e300));

    // Split a curve at a set of (sorted, strictly-interior) parameters into consecutive pieces.
    auto split_multi = [](const NurbsCurve& c, std::vector<double> params) -> std::vector<NurbsCurve> {
        std::sort(params.begin(), params.end());
        std::vector<NurbsCurve> out;
        NurbsCurve rem = c;
        for (double t : params) {
            auto [l, r] = rem.split(t);
            if (!l.is_valid() || !r.is_valid()) return {};  // fail -> caller keeps original
            out.push_back(l);
            rem = r;
        }
        out.push_back(rem);
        return out;
    };

    int ne0 = (int)m_topology_edges.size();
    for (int ei = 0; ei < ne0; ++ei) {
        // Only repair under-mated edges (the ones that break watertightness) -- unless
        // mated_too (BOP2 MakeSplitEdges mode): 2-trim edges also split at on-edge
        // vertices, so pool-edge anchors materialize as loop vertices in BOTH flanks.
        if (!mated_too && (int)m_topology_edges[ei].trim_indices.size() >= 2) continue;
        int ci = m_topology_edges[ei].curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        NurbsCurve C = m_curves_3d[ci];
        if (!C.is_valid()) continue;
        Point pA = C.point_at_start(), pB = C.point_at_end();
        double clen = pA.distance(pB);
        if (clen < tol) continue;  // closed cut circles: co-split needs the seam-aware arrangement

        // Edge AABB (sampled) -> cheap reject of far vertices before the closest_parameter solve.
        auto cdomf = C.domain();
        double ebb[6] = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= 6; ++k) {
            Point p = C.point_at(cdomf.first + (cdomf.second-cdomf.first)*k/6);
            for (int d = 0; d < 3; ++d) { ebb[d]=std::min(ebb[d],p[d]); ebb[d+3]=std::max(ebb[d+3],p[d]); }
        }

        // Interior vertices that lie on C (a T-junction split point). The end-guard and
        // dedup use a PAVE tolerance (curve-size-relative, OCCT PutPavesOnCurve): the same
        // geometric event reached through both operands' lifts differs by the lift
        // tolerance (~1e-4), and splitting that close to an end spawns a micro-stub edge
        // that can never mate (spiric tips).
        double pave_tol = std::max(tol, std::sqrt((ebb[3]-ebb[0])*(ebb[3]-ebb[0])
            + (ebb[4]-ebb[1])*(ebb[4]-ebb[1]) + (ebb[5]-ebb[2])*(ebb[5]-ebb[2])) * 2e-4);
        struct Split { double tc; Point V; };
        std::vector<Split> splits;
        for (const Point& V : vpos) {
            if (V[0] > 1e299) continue;
            if (V[0] < ebb[0]-tol || V[0] > ebb[3]+tol || V[1] < ebb[1]-tol || V[1] > ebb[4]+tol
                || V[2] < ebb[2]-tol || V[2] > ebb[5]+tol) continue;
            if (V.distance(pA) < pave_tol || V.distance(pB) < pave_tol) continue;
            double tc = C.closest_parameter(V);
            if (C.point_at(tc).distance(V) > tol) continue;
            auto dom = C.domain();
            double frac = (tc - dom.first) / (dom.second - dom.first);
            if (frac <= 1e-6 || frac >= 1.0 - 1e-6) continue;
            // Dedup near-equal split points.
            bool dup = false;
            for (auto& s : splits) if (s.V.distance(V) < pave_tol) { dup = true; break; }
            if (!dup) splits.push_back({tc, V});
        }
        if (splits.empty()) continue;
        std::sort(splits.begin(), splits.end(), [](const Split& a, const Split& b){ return a.tc < b.tc; });

        std::vector<double> cparams;
        for (auto& s : splits) cparams.push_back(s.tc);
        std::vector<NurbsCurve> c3pieces = split_multi(C, cparams);
        if ((int)c3pieces.size() != (int)splits.size() + 1) continue;  // split failed

        // Snapshot the original trims BEFORE the edge-piece loop clears edge ei's trim list.
        std::vector<int> orig_trims = m_topology_edges[ei].trim_indices;

        // New topology vertices at the split points.
        std::vector<int> vids;
        vids.push_back(m_topology_edges[ei].start_vertex);
        for (auto& s : splits) {
            int pidx = add_vertex(s.V);
            BRepVertex tv; tv.point_index = pidx;
            int tvid = (int)m_topology_vertices.size();
            m_topology_vertices.push_back(tv);
            vids.push_back(tvid);
        }
        vids.push_back(m_topology_edges[ei].end_vertex);

        // Edge pieces: piece 0 reuses ei (+ its 3D curve slot), the rest are appended.
        std::vector<int> edge_ids;
        for (size_t j = 0; j < c3pieces.size(); ++j) {
            int c3idx, eidx;
            if (j == 0) { c3idx = ci; eidx = ei; m_curves_3d[ci] = c3pieces[0]; }
            else { c3idx = add_curve_3d(c3pieces[j]); eidx = (int)m_topology_edges.size();
                   m_topology_edges.push_back(BRepEdge()); }
            m_topology_edges[eidx].curve_3d_index = c3idx;
            m_topology_edges[eidx].start_vertex = vids[j];
            m_topology_edges[eidx].end_vertex = vids[j + 1];
            m_topology_edges[eidx].trim_indices.clear();
            edge_ids.push_back(eidx);
        }

        // Split each trim of the (formerly single) edge to match the edge pieces.
        for (int ti : orig_trims) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            BRepTrim T = m_trims[ti];
            int li = T.loop_index;
            int fi = (li >= 0 && li < (int)m_loops.size()) ? m_loops[li].face_index : -1;
            int si = (fi >= 0 && fi < (int)m_faces.size()) ? m_faces[fi].surface_index : -1;
            int c2 = T.curve_2d_index;
            std::vector<NurbsCurve> p2pieces;
            if (si >= 0 && si < (int)m_surfaces.size() && c2 >= 0 && c2 < (int)m_curves_2d.size()) {
                const NurbsSurface& S = m_surfaces[si];
                NurbsCurve P = m_curves_2d[c2];
                // branch-free split param (see co_refine splice): project the 3D split point
                // onto the pcurve's surface image, never through Closest's principal branch.
                auto param_near_3d = [&](const NurbsCurve& P2, const NurbsSurface& S2, const Point& V) -> double {
                    auto pd = P2.domain(); int M = std::min(std::max(64, P2.cv_count()*4), 4096);
                    double bt = pd.first, bd = 1e300; Point prev3(0,0,0); double prevt = pd.first;
                    for (int i2 = 0; i2 <= M; ++i2) {
                        double t = pd.first + (pd.second - pd.first)*i2/M;
                        Point uv = P2.point_at(t); Point q = S2.point_at(uv[0], uv[1]);
                        if (i2 > 0) {
                            double ex=q[0]-prev3[0], ey=q[1]-prev3[1], ez=q[2]-prev3[2];
                            double L2=ex*ex+ey*ey+ez*ez;
                            double lam=(L2>1e-30)?((V[0]-prev3[0])*ex+(V[1]-prev3[1])*ey+(V[2]-prev3[2])*ez)/L2:0.0;
                            lam=std::min(std::max(lam,0.0),1.0);
                            double cx=prev3[0]+lam*ex, cy=prev3[1]+lam*ey, cz=prev3[2]+lam*ez;
                            double d=(V[0]-cx)*(V[0]-cx)+(V[1]-cy)*(V[1]-cy)+(V[2]-cz)*(V[2]-cz);
                            if (d<bd){ bd=d; bt=prevt+lam*(t-prevt); }
                        }
                        prev3=q; prevt=t;
                    }
                    auto d3at = [&](double t){ Point uv=P2.point_at(t); Point q=S2.point_at(uv[0],uv[1]);
                        double dx=q[0]-V[0], dy=q[1]-V[1], dz=q[2]-V[2]; return dx*dx+dy*dy+dz*dz; };
                    double lo = std::max(pd.first,  bt - (pd.second-pd.first)/M);
                    double hi = std::min(pd.second, bt + (pd.second-pd.first)/M);
                    for (int it = 0; it < 80 && hi-lo > 1e-14*(pd.second-pd.first); ++it) {
                        double m1 = lo + (hi-lo)/3, m2 = hi - (hi-lo)/3;
                        if (d3at(m1) < d3at(m2)) hi = m2; else lo = m1;
                    }
                    return 0.5*(lo+hi);
                };
                std::vector<double> tps;
                for (auto& s : splits) tps.push_back(param_near_3d(P, S, s.V));
                p2pieces = split_multi(P, tps);
            }
            bool ok = ((int)p2pieces.size() == (int)edge_ids.size());
            // A MATE-oriented trim's pcurve runs opposite the shared 3D curve (the edge was
            // lifted from the OTHER face's pcurve), so pieces in pcurve-param order do NOT line
            // up index-for-index with the 3D pieces -- index pairing hands each trim a WRONG edge
            // (the cross-assigned-edge corruption). Match each piece to ITS edge geometrically,
            // endpoint-compatible with a midpoint tie-break (same fix as co_refine's splice).
            std::vector<int> edge_for(edge_ids.begin(), edge_ids.end());
            if (ok && si >= 0 && si < (int)m_surfaces.size()) {
                const NurbsSurface& S2 = m_surfaces[si];
                double mtol = tol * 1e3;
                std::vector<int> ef(p2pieces.size(), -1);
                std::vector<bool> taken(c3pieces.size(), false);
                bool allm = true;
                for (size_t j = 0; j < p2pieces.size() && allm; ++j) {
                    if (!p2pieces[j].is_valid()) { allm = false; break; }
                    auto pd = p2pieces[j].domain();
                    Point a2u = p2pieces[j].point_at(pd.first), b2u = p2pieces[j].point_at(pd.second);
                    Point m2u = p2pieces[j].point_at(0.5*(pd.first+pd.second));
                    Point a3 = S2.point_at(a2u[0],a2u[1]), b3 = S2.point_at(b2u[0],b2u[1]);
                    Point m3 = S2.point_at(m2u[0],m2u[1]);
                    int hit = -1; double bestm = 1e300;
                    for (size_t k = 0; k < c3pieces.size(); ++k) {
                        if (taken[k]) continue;
                        Point ea = c3pieces[k].point_at_start(), eb = c3pieces[k].point_at_end();
                        bool epok = (a3.distance(ea) < mtol && b3.distance(eb) < mtol)
                                 || (a3.distance(eb) < mtol && b3.distance(ea) < mtol);
                        if (!epok) continue;
                        auto ad2 = c3pieces[k].domain();
                        Point em = c3pieces[k].point_at(0.5*(ad2.first+ad2.second));
                        double dm = m3.distance(em);
                        if (dm < bestm) { bestm = dm; hit = (int)k; }
                    }
                    if (hit < 0) { allm = false; break; }
                    taken[hit] = true; ef[j] = edge_ids[hit];
                }
                if (allm) edge_for.assign(ef.begin(), ef.end());
            }
            // Build the per-piece trims (forward order; boolean trims are not reversed).
            std::vector<int> newtrims;
            for (size_t j = 0; j < edge_ids.size(); ++j) {
                int c2idx, tidx;
                NurbsCurve pc = ok ? p2pieces[j] : (j == 0 ? m_curves_2d[std::max(c2,0)] : NurbsCurve());
                if (j == 0) { c2idx = (c2 >= 0 ? c2 : add_curve_2d(pc)); if (c2 >= 0) m_curves_2d[c2] = pc; tidx = ti; }
                else { c2idx = add_curve_2d(pc); tidx = (int)m_trims.size(); m_trims.push_back(BRepTrim()); }
                m_trims[tidx].curve_2d_index = c2idx;
                m_trims[tidx].edge_index = edge_for[j];
                m_trims[tidx].loop_index = li;
                m_trims[tidx].reversed = T.reversed;
                m_trims[tidx].type = T.type;
                newtrims.push_back(tidx);
                m_topology_edges[edge_for[j]].trim_indices.push_back(tidx);
            }
            if (T.reversed) std::reverse(newtrims.begin(), newtrims.end());
            // Replace ti in the loop's trim list with the piece trims.
            if (li >= 0 && li < (int)m_loops.size()) {
                auto& tl = m_loops[li].trim_indices;
                auto it = std::find(tl.begin(), tl.end(), ti);
                if (it != tl.end()) { int pos = (int)(it - tl.begin());
                    tl.erase(it); tl.insert(tl.begin() + pos, newtrims.begin(), newtrims.end()); }
            }
        }
    }

    // Rebuild vertex->edge adjacency.
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        int sv = m_topology_edges[ei].start_vertex, ev = m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei);
    }
}

namespace {
// Exact rational-quadratic arc on the circle (Cc, e1, e2, r): starts at angle a0, sweeps `swept`
// radians (signed), split into <=90-deg Bezier segments with the standard w = cos(phi/2) mid-CV
// (same construction as intersection.cpp exact_circle). Endpoints reproduce on-circle points
// exactly, so arcs built from shared snapped vertices are bit-identical on both operands.
NurbsCurve exact_arc_3d(const Point& Cc, const Vector& e1, const Vector& e2, double r,
                        double a0, double swept) {
    const double PI = 3.14159265358979323846;
    int nseg = (int)std::ceil(std::abs(swept) / (PI / 2.0) - 1e-12);
    if (nseg < 1) nseg = 1;
    double phi = swept / nseg;
    double w = std::cos(0.5 * phi);
    int ncv = 2 * nseg + 1;
    NurbsCurve crv(3, true, 3, ncv);
    for (int i = 0; i < ncv + 1; ++i) crv.set_nurbsknot(i, (double)(i / 2));
    auto onc = [&](double a, double rr) {
        return Point(Cc[0] + rr * (std::cos(a) * e1[0] + std::sin(a) * e2[0]),
                     Cc[1] + rr * (std::cos(a) * e1[1] + std::sin(a) * e2[1]),
                     Cc[2] + rr * (std::cos(a) * e1[2] + std::sin(a) * e2[2]));
    };
    Point P0 = onc(a0, r);
    crv.set_cv_4d(0, P0[0], P0[1], P0[2], 1.0);
    for (int s = 0; s < nseg; ++s) {
        double aa = a0 + phi * s, ab = aa + phi, am = 0.5 * (aa + ab);
        Point M = onc(am, r / std::cos(0.5 * phi));
        Point P1 = onc(ab, r);
        crv.set_cv_4d(2 * s + 1, M[0] * w, M[1] * w, M[2] * w, w);
        crv.set_cv_4d(2 * s + 2, P1[0], P1[1], P1[2], 1.0);
    }
    crv.set_domain(0.0, 1.0);
    return crv;
}

// Circle through 3D points: circumcenter from three spread samples, then EVERY point must lie on
// that circle (planarity AND radius) within a tight relative tolerance. The lift polylines'
// vertices are exact on-circle surface evaluations, so genuine section circles pass with ~1e-15
// residual while lines/ellipses/cone conics/freeform sections fail immediately.
// Exact sphere recognition from the surface itself (poles -> center/radius, verified on a
// sample grid). Our sphere NURBS are exact rational surfaces, so this is machine precision --
// unlike the pullback-interpolated lift vertices, which sit ~1e-4 off the section circle.
bool sphere_of_surface(const NurbsSurface& s, Point& C, double& R) {
    auto du = s.domain(0); auto dv = s.domain(1);
    double um = 0.5*(du.first + du.second);
    Point ps = s.point_at(um, dv.first), pn = s.point_at(um, dv.second);
    C = Point(0.5*(ps[0]+pn[0]), 0.5*(ps[1]+pn[1]), 0.5*(ps[2]+pn[2]));
    R = 0.5*ps.distance(pn);
    if (R < 1e-9) return false;
    for (int i = 0; i <= 3; ++i) for (int j = 1; j < 3; ++j) {
        Point p = s.point_at(du.first+(du.second-du.first)*i/3.0, dv.first+(dv.second-dv.first)*j/3.0);
        if (std::abs(p.distance(C) - R) > R*1e-7 + 1e-9) return false;
    }
    return true;
}

// Exact cylinder recognition: the v-boundary circles' centers give the axis (the rational-
// quadratic circle's antipode sits exactly at mid-u), verified on a sample grid.
bool cylinder_of_surface(const NurbsSurface& s, Point& A, Vector& W, double& R) {
    auto du = s.domain(0); auto dv = s.domain(1);
    double um2 = 0.5*(du.first + du.second);
    Point a0 = s.point_at(du.first, dv.first), a1 = s.point_at(um2, dv.first);
    Point b0 = s.point_at(du.first, dv.second), b1 = s.point_at(um2, dv.second);
    A = Point(0.5*(a0[0]+a1[0]), 0.5*(a0[1]+a1[1]), 0.5*(a0[2]+a1[2]));
    Point C1(0.5*(b0[0]+b1[0]), 0.5*(b0[1]+b1[1]), 0.5*(b0[2]+b1[2]));
    double wl = A.distance(C1);
    if (wl < 1e-9) return false;
    W = Vector((C1[0]-A[0])/wl, (C1[1]-A[1])/wl, (C1[2]-A[2])/wl);
    R = 0.5*a0.distance(a1);
    if (R < 1e-9) return false;
    for (int i = 0; i <= 3; ++i) for (int j = 0; j <= 2; ++j) {
        Point p = s.point_at(du.first+(du.second-du.first)*i/3.0, dv.first+(dv.second-dv.first)*j/2.0);
        double wx=p[0]-A[0], wy=p[1]-A[1], wz=p[2]-A[2];
        double t = wx*W[0]+wy*W[1]+wz*W[2];
        double dx=wx-t*W[0], dy=wy-t*W[1], dz=wz-t*W[2];
        if (std::abs(std::sqrt(dx*dx+dy*dy+dz*dz) - R) > R*1e-7 + 1e-9) return false;
    }
    return true;
}

bool circle_through_points(const std::vector<Point>& pts, Point& Cc, Vector& nrm,
                           Vector& e1, Vector& e2, double& r) {
    int m = (int)pts.size();
    if (m < 4) return false;
    const Point& p0 = pts[0];
    const Point& p1 = pts[m / 3];
    const Point& p2 = pts[(2 * m) / 3];
    double ax = p1[0]-p0[0], ay = p1[1]-p0[1], az = p1[2]-p0[2];
    double bx = p2[0]-p0[0], by = p2[1]-p0[1], bz = p2[2]-p0[2];
    double nx = ay*bz - az*by, ny = az*bx - ax*bz, nz = ax*by - ay*bx;
    double n2 = nx*nx + ny*ny + nz*nz;
    if (n2 < 1e-20) return false;
    // circumcenter = p0 + (|b|^2 ((a x b) x a) + |a|^2 (b x (a x b))) / (2 |a x b|^2)
    double a2 = ax*ax + ay*ay + az*az, b2 = bx*bx + by*by + bz*bz;
    double c1x = ny*az - nz*ay, c1y = nz*ax - nx*az, c1z = nx*ay - ny*ax;   // (a x b) x a
    double c2x = by*nz - bz*ny, c2y = bz*nx - bx*nz, c2z = bx*ny - by*nx;   // b x (a x b)
    Cc = Point(p0[0] + (b2*c1x + a2*c2x) / (2.0*n2),
               p0[1] + (b2*c1y + a2*c2y) / (2.0*n2),
               p0[2] + (b2*c1z + a2*c2z) / (2.0*n2));
    r = Cc.distance(p0);
    if (r < 1e-12) return false;
    double nl = std::sqrt(n2);
    nrm = Vector(nx/nl, ny/nl, nz/nl);
    e1 = Vector((p0[0]-Cc[0])/r, (p0[1]-Cc[1])/r, (p0[2]-Cc[2])/r);
    e2 = Vector(nrm[1]*e1[2]-nrm[2]*e1[1], nrm[2]*e1[0]-nrm[0]*e1[2], nrm[0]*e1[1]-nrm[1]*e1[0]);
    double rtol = std::max(1e-7 * r, 1e-9);
    for (const auto& p : pts) {
        double wx = p[0]-Cc[0], wy = p[1]-Cc[1], wz = p[2]-Cc[2];
        double h = wx*nrm[0] + wy*nrm[1] + wz*nrm[2];
        if (std::abs(h) > rtol) return false;
        double rad = std::sqrt(wx*wx + wy*wy + wz*wz);
        if (std::abs(rad - r) > rtol) return false;
    }
    return true;
}
} // namespace

void BRep::co_refine_coincident_edges(double tol) {
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 5e-3;   // sew-level coincidence tolerance

    auto split_multi = [](const NurbsCurve& c, std::vector<double> params) -> std::vector<NurbsCurve> {
        std::sort(params.begin(), params.end());
        std::vector<NurbsCurve> out; NurbsCurve rem = c;
        for (double t : params) { auto [l, r] = rem.split(t);
            if (!l.is_valid() || !r.is_valid()) return {}; out.push_back(l); rem = r; }
        out.push_back(rem); return out;
    };
    auto p2pl = [](const Point& p, const std::vector<Point>& pts) -> double {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pts.size(); ++j) { const Point& a = pts[j]; const Point& b = pts[j+1];
            double ex=b[0]-a[0],ey=b[1]-a[1],ez=b[2]-a[2],L2=ex*ex+ey*ey+ez*ez;
            double t = (L2>1e-30)?((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2:0.0;
            t = std::min(std::max(t,0.0),1.0);
            double cx=a[0]+t*ex,cy=a[1]+t*ey,cz=a[2]+t*ez;
            best = std::min(best, std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz))); }
        return best;
    };

    int ne0 = (int)m_topology_edges.size();
    auto cand = [&](int e){ return e>=0 && e<ne0 && (int)m_topology_edges[e].trim_indices.size() < 2
                                  && m_topology_edges[e].curve_3d_index >= 0
                                  && m_topology_edges[e].curve_3d_index < (int)m_curves_3d.size(); };
    // NS=24 under-resolves an elongated closed section loop: a 25-point polyline of the box x tor
    // spiric (3.9 x 1.1 lens) SAGS ~0.034 at the high-curvature tips while the true arc-to-arc
    // deviation is 0.0036 -- subset_of then fails on SAMPLING error alone (tol=diag*5e-3=0.029),
    // the closed disc edge is never split at its arcs' feet, and the result is not solid. Denser
    // sampling only reveals true proximity (measurement-side): it can only make coincidence tests
    // MORE accurate, never merge distinct edges.
    const int NS = 96;
    std::vector<std::vector<Point>> samp(ne0);
    std::vector<std::array<double,6>> bbox(ne0);
    for (int e=0;e<ne0;++e){ if(!cand(e)) continue;
        const NurbsCurve& C = m_curves_3d[m_topology_edges[e].curve_3d_index];
        auto [t0,t1]=C.domain(); std::array<double,6> bb={1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for(int k=0;k<=NS;++k){ Point p=C.point_at(t0+(t1-t0)*k/NS); samp[e].push_back(p);
            for(int d=0;d<3;++d){bb[d]=std::min(bb[d],p[d]);bb[d+3]=std::max(bb[d+3],p[d]);} }
        bbox[e]=bb; }
    auto bbox_far = [&](int i,int j){ const auto&a=bbox[i];const auto&b=bbox[j];
        return a[0]>b[3]+tol||b[0]>a[3]+tol||a[1]>b[4]+tol||b[1]>a[4]+tol||a[2]>b[5]+tol||b[2]>a[5]+tol; };
    // ej is an arc-SUBSET of ei: every ej sample lies within tol of ei's polyline. (One-directional;
    // a full circle is NOT a subset of one of its arcs, but each arc IS a subset of the circle.)
    auto subset_of = [&](int ej,int ei){ if(samp[ej].size()<2||samp[ei].size()<2) return false;
        for(const auto& p:samp[ej]) if(p2pl(p,samp[ei])>tol) return false; return true; };

    for (int ei=0; ei<ne0; ++ei) {
        if (!cand(ei)) continue;
        const NurbsCurve C = m_curves_3d[m_topology_edges[ei].curve_3d_index];
        if (!C.is_valid()) continue;
        auto dom = C.domain(); Point pA=C.point_at_start(), pB=C.point_at_end();
        bool closed = pA.distance(pB) < tol;

        // Split points on C = endpoints of DISTINCT under-mated edges that are arc-subsets of C and
        // land strictly interior on C (a circle split at its coincident arcs' shared endpoints).
        std::vector<std::pair<double,Point>> sp;
        bool seam_has_split = false;
        for (int ej=0; ej<ne0; ++ej) {
            if (ej==ei || !cand(ej) || bbox_far(ei,ej) || !subset_of(ej,ei)) continue;
            const NurbsCurve& Cj = m_curves_3d[m_topology_edges[ej].curve_3d_index];
            // Only an OPEN arc subdivides ei at its endpoints. A closed circle coincident with ei
            // (e.g. the same cut circle imprinted whole on both operands) has only its arbitrary
            // param-seam as an "endpoint" -- splitting there is spurious; sew merges them whole.
            if (Cj.point_at_start().distance(Cj.point_at_end()) < tol) continue;
            Point ends[2] = { Cj.point_at_start(), Cj.point_at_end() };
            for (const Point& V : ends) {
                double tc = C.closest_parameter(V);
                double dv = C.point_at(tc).distance(V);
                double frac = (tc-dom.first)/(dom.second-dom.first);
                if (std::getenv("SESSION_SEW_DBG"))
                    std::fprintf(stderr, "[CRD] ei=%d ej=%d frac=%.6f d=%.4f\n", ei, ej, frac, dv);
                if (dv > tol) continue;
                if (frac <= 1e-6 || frac >= 1.0-1e-6) { seam_has_split = true; continue; }
                bool dup=false; for(auto&s:sp) if(s.second.distance(V)<tol){dup=true;break;}
                if(!dup) sp.push_back({tc,V});
            }
        }
        if (std::getenv("SESSION_SEW_DBG") && (!sp.empty() || seam_has_split))
            std::fprintf(stderr, "[CR] ei=%d splits=%zu seam=%d s(%.4f,%.4f,%.4f) e(%.4f,%.4f,%.4f)\n",
                ei, sp.size(), seam_has_split ? 1 : 0, pA[0], pA[1], pA[2], pB[0], pB[1], pB[2]);
        if (sp.empty()) continue;
        // EXACT SECTION-CIRCLE REBUILD (P0): the lift stores every 3D edge as a chord polyline
        // (devtol ~2e-3 of the face size), so a split section CIRCLE - and every pcurve rebuilt
        // from its pieces - INSCRIBES the true circle: O(1/N^2) area/volume deficit (box x sphere
        // ~1.5e-3). The polyline vertices are exact on-circle points, so recover the true circle,
        // snap the split points onto it, and below rebuild each piece as the EXACT rational arc
        // between its snapped vertices. Non-circles fail the fit and keep the polyline path.
        // Recognized section conic: center fitC, unit axes fitE1/fitE2 with semi-axes fitA/fitB
        // (circle: fitA == fitB). All arc math below runs in the SCALED frame (sx,sy) where the
        // conic is the unit circle -- a rational-quadratic arc is exact under the linear map, so
        // exact_arc_3d with basis (fitE1*fitA, fitE2*fitB, r=1) emits exact ELLIPSE arcs too.
        Point fitC; Vector fitN, fitE1, fitE2; double fitR = 0, fitA = 0, fitB = 0;
        bool fit_ok = false;
        if (C.degree() == 1 && !C.is_rational()) {
            std::vector<Point> cpts;
            for (int i = 0; i < C.cv_count(); ++i) cpts.push_back(C.get_cv(i));
            fit_ok = circle_through_points(cpts, fitC, fitN, fitE1, fitE2, fitR);
        }
        if (!fit_ok && C.degree() == 1 && !C.is_rational()) {
            // The vertex fit fails when BOTH operands' lifts are pullback-interpolated (~1e-4
            // noise, e.g. sphere x sphere -- no exact planar side to trust). The operand SURFACES
            // are exact: derive the section circle from the two adjacent spheres' radical plane,
            // then verify the vertices LOOSELY (rejects edges that are not this circle at all).
            auto srf_of_edge = [&](int e) -> int {
                if (e < 0 || e >= (int)m_topology_edges.size()) return -1;
                for (int t : m_topology_edges[e].trim_indices) {
                    if (t < 0 || t >= (int)m_trims.size()) continue;
                    int l = m_trims[t].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
                    int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
                    return m_faces[f].surface_index;
                }
                return -1;
            };
            int s1 = srf_of_edge(ei), s2 = -1;
            for (int ej = 0; ej < ne0 && s2 < 0; ++ej) {
                if (ej == ei || !cand(ej) || bbox_far(ei,ej) || !subset_of(ej,ei)) continue;
                int sj = srf_of_edge(ej);
                if (sj >= 0 && sj != s1) s2 = sj;
            }
            if (s1 >= 0 && s2 >= 0 && s1 < (int)m_surfaces.size() && s2 < (int)m_surfaces.size()) {
                Point C1, C2; double R1 = 0, R2 = 0;
                if (sphere_of_surface(m_surfaces[s1], C1, R1) && sphere_of_surface(m_surfaces[s2], C2, R2)) {
                    double dx=C2[0]-C1[0], dy=C2[1]-C1[1], dz=C2[2]-C1[2];
                    double d = std::sqrt(dx*dx+dy*dy+dz*dz);
                    if (d > 1e-9 && d < R1+R2) {
                        double a = (d*d + R1*R1 - R2*R2) / (2.0*d);
                        double rr2 = R1*R1 - a*a;
                        if (rr2 > 1e-18) {
                            fitN = Vector(dx/d, dy/d, dz/d);
                            fitC = Point(C1[0]+a*fitN[0], C1[1]+a*fitN[1], C1[2]+a*fitN[2]);
                            fitR = std::sqrt(rr2);
                            Point P0 = C.point_at_start();
                            double wx=P0[0]-fitC[0], wy=P0[1]-fitC[1], wz=P0[2]-fitC[2];
                            double h = wx*fitN[0]+wy*fitN[1]+wz*fitN[2];
                            double px=wx-h*fitN[0], py=wy-h*fitN[1], pz=wz-h*fitN[2];
                            double pl = std::sqrt(px*px+py*py+pz*pz);
                            if (pl > 1e-12) {
                                fitE1 = Vector(px/pl, py/pl, pz/pl);
                                fitE2 = Vector(fitN[1]*fitE1[2]-fitN[2]*fitE1[1],
                                               fitN[2]*fitE1[0]-fitN[0]*fitE1[2],
                                               fitN[0]*fitE1[1]-fitN[1]*fitE1[0]);
                                double vtol = std::max(5e-3*fitR, 1e-6);
                                bool okv = true;
                                for (int i = 0; i < C.cv_count() && okv; ++i) {
                                    Point p = C.get_cv(i);
                                    double ax=p[0]-fitC[0], ay=p[1]-fitC[1], az=p[2]-fitC[2];
                                    double hh = ax*fitN[0]+ay*fitN[1]+az*fitN[2];
                                    double rd2 = ax*ax+ay*ay+az*az - hh*hh;
                                    double rad = rd2 > 0 ? std::sqrt(rd2) : 0.0;
                                    if (std::abs(hh) > vtol || std::abs(rad-fitR) > vtol) okv = false;
                                }
                                fit_ok = okv;
                            }
                        }
                    }
                }
                if (!fit_ok && std::getenv("SESSION_BOOL_ELLIPSE")) {
                    // CYLINDER x CYLINDER, equal radii, intersecting axes: the section is TWO
                    // exact ellipses in the (w1 +- w2) planes (Steinmetz); pick the one whose
                    // ellipse the edge's vertices actually lie on. GATED OFF pending P8: the
                    // cylinder faces' masked-Gauss flux dominates the cyl x cyl error anyway,
                    // and enabling this flipped one fuse edge to non-solid (mate mismatch TBD).
                    Point A1, A2; Vector W1, W2; double Rc1 = 0, Rc2 = 0;
                    if (cylinder_of_surface(m_surfaces[s1], A1, W1, Rc1)
                     && cylinder_of_surface(m_surfaces[s2], A2, W2, Rc2)
                     && std::abs(Rc1-Rc2) < 1e-6*std::max(Rc1,Rc2)) {
                        double b = W1[0]*W2[0]+W1[1]*W2[1]+W1[2]*W2[2];
                        double den = 1.0 - b*b;
                        if (den > 1e-12) {
                            double d0x=A2[0]-A1[0], d0y=A2[1]-A1[1], d0z=A2[2]-A1[2];
                            double dw1 = d0x*W1[0]+d0y*W1[1]+d0z*W1[2];
                            double dw2 = d0x*W2[0]+d0y*W2[1]+d0z*W2[2];
                            double t1 = (dw1 - b*dw2)/den, t2 = (b*dw1 - dw2)/den;
                            Point q1(A1[0]+t1*W1[0], A1[1]+t1*W1[1], A1[2]+t1*W1[2]);
                            Point q2(A2[0]+t2*W2[0], A2[1]+t2*W2[1], A2[2]+t2*W2[2]);
                            if (q1.distance(q2) < 1e-6) {
                                double Rr = 0.5*(Rc1+Rc2);
                                double cx=W1[1]*W2[2]-W1[2]*W2[1], cy=W1[2]*W2[0]-W1[0]*W2[2], cz=W1[0]*W2[1]-W1[1]*W2[0];
                                double cl = std::sqrt(cx*cx+cy*cy+cz*cz);
                                double ang = std::atan2(cl, b);
                                double sh = std::sin(0.5*ang), ch = std::cos(0.5*ang);
                                if (cl > 1e-9 && sh > 1e-9 && ch > 1e-9) {
                                    Vector mnr(cx/cl, cy/cl, cz/cl);
                                    Point Pc(0.5*(q1[0]+q2[0]), 0.5*(q1[1]+q2[1]), 0.5*(q1[2]+q2[2]));
                                    for (int cand = 0; cand < 2 && !fit_ok; ++cand) {
                                        double sx = cand==0 ? 1.0 : -1.0;
                                        double mx=W1[0]+sx*W2[0], my=W1[1]+sx*W2[1], mz=W1[2]+sx*W2[2];
                                        double ml = std::sqrt(mx*mx+my*my+mz*mz);
                                        if (ml < 1e-9) continue;
                                        Vector maj(mx/ml, my/ml, mz/ml);
                                        double semiA = cand==0 ? Rr/sh : Rr/ch;
                                        Vector n2(maj[1]*mnr[2]-maj[2]*mnr[1], maj[2]*mnr[0]-maj[0]*mnr[2], maj[0]*mnr[1]-maj[1]*mnr[0]);
                                        double vtol = std::max(5e-3*Rr, 1e-6);
                                        bool okv = true;
                                        for (int i = 0; i < C.cv_count() && okv; ++i) {
                                            Point p = C.get_cv(i);
                                            double ax=p[0]-Pc[0], ay=p[1]-Pc[1], az=p[2]-Pc[2];
                                            double hh = ax*n2[0]+ay*n2[1]+az*n2[2];
                                            double su = (ax*maj[0]+ay*maj[1]+az*maj[2])/semiA;
                                            double sv = (ax*mnr[0]+ay*mnr[1]+az*mnr[2])/Rr;
                                            if (std::abs(hh) > vtol || std::abs(std::sqrt(su*su+sv*sv)-1.0)*Rr > vtol) okv = false;
                                        }
                                        if (okv) {
                                            fitC = Pc; fitE1 = maj; fitE2 = mnr; fitN = n2;
                                            fitA = semiA; fitB = Rr; fit_ok = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (fit_ok && fitA == 0) { fitA = fitR; fitB = fitR; }
        if (fit_ok) for (auto& s : sp) {
            double wx=s.second[0]-fitC[0], wy=s.second[1]-fitC[1], wz=s.second[2]-fitC[2];
            double su = (wx*fitE1[0]+wy*fitE1[1]+wz*fitE1[2])/fitA;
            double sv = (wx*fitE2[0]+wy*fitE2[1]+wz*fitE2[2])/fitB;
            double rad = std::sqrt(su*su + sv*sv);
            if (rad > 1e-12) s.second = Point(fitC[0]+(su/rad)*fitA*fitE1[0]+(sv/rad)*fitB*fitE2[0],
                                              fitC[1]+(su/rad)*fitA*fitE1[1]+(sv/rad)*fitB*fitE2[1],
                                              fitC[2]+(su/rad)*fitA*fitE1[2]+(sv/rad)*fitB*fitE2[2]);
        }
        std::sort(sp.begin(), sp.end(), [](auto&a,auto&b){return a.first<b.first;});
        // Pave dedup (OCCT PutPavesOnCurve): the same geometric event reached through both
        // operands' lifts differs by the lift tolerance; splitting at both copies spawns a
        // micro-stub edge that can never mate (spiric tips: the torus pullback seam-splits
        // AT the tips, the plane side then counter-splits ~1e-4 away). Keep one event per
        // cluster and drop events indistinguishable from the curve ends.
        {
            double clen = 0.0;
            { Point pv = C.point_at_start();
              auto dc0 = C.domain();
              for (int k = 1; k <= 8; ++k) { Point q = C.point_at(dc0.first + (dc0.second-dc0.first)*k/8.0);
                  clen += pv.distance(q); pv = q; } }
            double pave_tol = std::max(clen * 2e-4, 1e-9);
            Point cs = C.point_at_start(), ce = C.point_at_end();
            std::vector<std::pair<double,Point>> ded;
            for (auto& s : sp) {
                if (s.second.distance(cs) < pave_tol || s.second.distance(ce) < pave_tol) continue;
                if (!ded.empty() && s.second.distance(ded.back().second) < pave_tol) continue;
                ded.push_back(s);
            }
            sp.swap(ded);
        }
        std::vector<double> iparams; std::vector<Point> ipts;
        for(auto&s:sp){ iparams.push_back(s.first); ipts.push_back(s.second); }

        std::vector<NurbsCurve> c3pieces = split_multi(C, iparams);
        if ((int)c3pieces.size() != (int)iparams.size()+1) {
            if (std::getenv("SESSION_SEW_DBG"))
                std::fprintf(stderr, "[CR] ei=%d SPLIT-FAIL want=%zu got=%zu\n", ei, iparams.size()+1, c3pieces.size());
            continue;  // split failed -> keep edge
        }

        // wrap-join the first+last 3D piece across the param seam ONLY for a closed edge whose seam
        // is interior to an arc (no split point at the seam). Else (open, or seam coincides with a
        // split point) the pieces are already the arcs.
        bool do_wrap = closed && !seam_has_split;

        std::vector<int> orig_trims = m_topology_edges[ei].trim_indices;
        // vertices at the interior split points
        std::vector<int> svids;
        for (const Point& V : ipts) {
            int pidx = add_vertex(V); BRepVertex tv; tv.point_index = pidx;
            svids.push_back((int)m_topology_vertices.size()); m_topology_vertices.push_back(tv);
        }
        // edge-piece curves + their (startV,endV)
        std::vector<NurbsCurve> arcs; std::vector<std::array<int,2>> arc_v;
        if (do_wrap) {
            auto wrap = NurbsCurve::join({c3pieces.back(), c3pieces.front()}, tol);
            if (wrap.size()!=1 || !wrap[0].is_valid()) continue;
            for (size_t k=1;k+1<c3pieces.size();++k){ arcs.push_back(c3pieces[k]); arc_v.push_back({svids[k-1],svids[k]}); }
            arcs.push_back(wrap[0]); arc_v.push_back({svids.back(), svids.front()});
        } else {
            std::vector<int> vids; vids.push_back(m_topology_edges[ei].start_vertex);
            for(int v:svids) vids.push_back(v); vids.push_back(m_topology_edges[ei].end_vertex);
            for (size_t k=0;k<c3pieces.size();++k){ arcs.push_back(c3pieces[k]); arc_v.push_back({vids[k],vids[k+1]}); }
        }

        if (fit_ok) {
            const double PI = 3.14159265358979323846;
            double fmin = std::min(fitA, fitB);
            double rtol = std::max(1e-7 * fmin, 1e-9);
            Vector sE1(fitE1[0]*fitA, fitE1[1]*fitA, fitE1[2]*fitA);
            Vector sE2(fitE2[0]*fitB, fitE2[1]*fitB, fitE2[2]*fitB);
            auto vpos = [&](int vid) -> Point {
                int pi = (vid>=0 && vid<(int)m_topology_vertices.size()) ? m_topology_vertices[vid].point_index : -1;
                return (pi>=0 && pi<(int)m_vertices.size()) ? m_vertices[pi] : Point(0,0,0);
            };
            auto ang_of = [&](const Point& P) {
                double wx=P[0]-fitC[0], wy=P[1]-fitC[1], wz=P[2]-fitC[2];
                return std::atan2((wx*fitE2[0]+wy*fitE2[1]+wz*fitE2[2])/fitB,
                                  (wx*fitE1[0]+wy*fitE1[1]+wz*fitE1[2])/fitA);
            };
            auto off_circle = [&](const Point& P) {
                double wx=P[0]-fitC[0], wy=P[1]-fitC[1], wz=P[2]-fitC[2];
                double h = wx*fitN[0]+wy*fitN[1]+wz*fitN[2];
                double su = (wx*fitE1[0]+wy*fitE1[1]+wz*fitE1[2])/fitA;
                double sv = (wx*fitE2[0]+wy*fitE2[1]+wz*fitE2[2])/fitB;
                return std::max(std::abs(h), std::abs(std::sqrt(su*su+sv*sv) - 1.0)*fmin);
            };
            // Rebuild `poly`'s span as the exact arc from A to B; the polyline supplies the winding
            // (which way round, how many quarters) - its endpoint gap only fixes swept mod 2*pi.
            auto rebuild = [&](const NurbsCurve& poly, const Point& A, const Point& B, NurbsCurve& out) {
                if (off_circle(A) > rtol || off_circle(B) > rtol) return false;
                double sw_poly = 0.0, prev = 0.0; bool first = true;
                for (int i = 0; i < poly.cv_count(); ++i) {
                    double a = ang_of(poly.get_cv(i));
                    if (!first) { double d = a - prev;
                        while (d >  PI) d -= 2*PI; while (d < -PI) d += 2*PI; sw_poly += d; }
                    prev = a; first = false;
                }
                double a0 = ang_of(A), a1 = ang_of(B);
                double sw = (a1 - a0) + 2*PI*std::round((sw_poly - (a1 - a0)) / (2*PI));
                if (std::abs(sw) < 1e-9 || std::abs(sw) > 2*PI + 1e-9) return false;
                out = exact_arc_3d(fitC, sE1, sE2, 1.0, a0, sw);
                return out.is_valid();
            };
            for (size_t k = 0; k < arcs.size(); ++k) {
                NurbsCurve na;
                if (rebuild(arcs[k], vpos(arc_v[k][0]), vpos(arc_v[k][1]), na)) arcs[k] = na;
            }
            // Propagate to the coincident opposite-operand arcs (co-oriented, same snapped
            // endpoints -> identical geometry on both sides): sew keeps the earliest edge as
            // representative, so BOTH candidates must carry the exact arc for it to survive.
            for (int ej = 0; ej < ne0; ++ej) {
                if (ej == ei || !cand(ej) || bbox_far(ei,ej) || !subset_of(ej,ei)) continue;
                int cj = m_topology_edges[ej].curve_3d_index;
                const NurbsCurve& Cj = m_curves_3d[cj];
                Point ja = Cj.point_at_start(), jb = Cj.point_at_end();
                if (ja.distance(jb) < tol) continue;
                for (size_t k = 0; k < arcs.size(); ++k) {
                    Point A = vpos(arc_v[k][0]), B = vpos(arc_v[k][1]);
                    bool fw = ja.distance(A) < tol && jb.distance(B) < tol;
                    bool bw = ja.distance(B) < tol && jb.distance(A) < tol;
                    if (!fw && !bw) continue;
                    NurbsCurve na;
                    if (rebuild(Cj, fw ? A : B, fw ? B : A, na)) m_curves_3d[cj] = na;
                    break;
                }
            }
        }

        // create the piece edges (piece 0 reuses ei + its 3D-curve slot)
        std::vector<int> edge_ids;
        for (size_t k=0;k<arcs.size();++k){
            int c3idx,eidx;
            if(k==0){ c3idx=m_topology_edges[ei].curve_3d_index; eidx=ei; m_curves_3d[c3idx]=arcs[0]; }
            else { c3idx=add_curve_3d(arcs[k]); eidx=(int)m_topology_edges.size(); m_topology_edges.push_back(BRepEdge()); }
            m_topology_edges[eidx].curve_3d_index=c3idx;
            m_topology_edges[eidx].start_vertex=arc_v[k][0];
            m_topology_edges[eidx].end_vertex=arc_v[k][1];
            m_topology_edges[eidx].trim_indices.clear();
            edge_ids.push_back(eidx);
        }

        // split each trim's 2D pcurve at the same params (project the split points onto the surface)
        for (int ti : orig_trims) {
            if (ti<0 || ti>=(int)m_trims.size()) continue;
            BRepTrim T = m_trims[ti];
            int li=T.loop_index, fi=(li>=0&&li<(int)m_loops.size())?m_loops[li].face_index:-1;
            int si=(fi>=0&&fi<(int)m_faces.size())?m_faces[fi].surface_index:-1;
            int c2=T.curve_2d_index;
            // Build each arc's 2D pcurve. PLANAR face: project the (already-split) 3D arc onto the
            // plane (exact, cheap). CURVED face: split the original open pcurve at the projected
            // params (cheap, exact) -- projecting onto a curved surface per-sample is slow/unstable
            // near singularities (e.g. a cone apex). (NurbsCurve::split silently fails on a CLOSED
            // rational circle, so closed circles must be on planar faces here -- they are: a closed
            // cut-circle co-refined against open arcs is the box's planar disc; coincident closed
            // circles on curved faces are skipped as ej.)
            std::vector<NurbsCurve> p2;
            if (si>=0 && si<(int)m_surfaces.size()) {
                const NurbsSurface& S=m_surfaces[si];
                // Try splitting the ORIGINAL pcurve first for EVERY face: it carries the
                // section's fit accuracy, while the 3D arcs are devtol-coarse polyline lifts
                // (the planar affine rebuild inherits their sag: the split spiric discs lost
                // 0.7% area through it). The one known failure -- NurbsCurve::split on a
                // CLOSED rational circle -- is pre-gated; any other failure falls through to
                // the planar affine-from-arcs rebuild.
                bool closed_rational = false;
                if (c2>=0 && c2<(int)m_curves_2d.size()) {
                    const NurbsCurve& P0 = m_curves_2d[c2];
                    auto pd0 = P0.domain();
                    closed_rational = P0.is_rational()
                        && P0.point_at(pd0.first).distance(P0.point_at(pd0.second)) < 1e-12;
                }
                if (c2>=0 && c2<(int)m_curves_2d.size() && !closed_rational) {
                    NurbsCurve P=m_curves_2d[c2];
                    // Split param via the SURFACE-MAPPED pcurve, not Closest::surface_point:
                    // near a periodic seam Closest clamps u to the principal branch boundary,
                    // closest_parameter then lands at a domain end, the split fails or lands
                    // 1e-2 off, and the ok=false path below binds the WHOLE pcurve to piece 0
                    // (the cross-assigned-edge corruption). Projecting the 3D split point onto
                    // the pcurve's own 3D image is branch-free and exact for polyline pcurves.
                    auto param_near_3d = [&](const NurbsCurve& P2, const NurbsSurface& S2, const Point& V) -> double {
                        auto pd = P2.domain(); int M = std::min(std::max(64, P2.cv_count()*4), 4096);
                        double bt = pd.first, bd = 1e300; Point prev3(0,0,0); double prevt = pd.first;
                        for (int i2 = 0; i2 <= M; ++i2) {
                            double t = pd.first + (pd.second - pd.first)*i2/M;
                            Point uv = P2.point_at(t); Point q = S2.point_at(uv[0], uv[1]);
                            if (i2 > 0) {
                                double ex=q[0]-prev3[0], ey=q[1]-prev3[1], ez=q[2]-prev3[2];
                                double L2=ex*ex+ey*ey+ez*ez;
                                double lam=(L2>1e-30)?((V[0]-prev3[0])*ex+(V[1]-prev3[1])*ey+(V[2]-prev3[2])*ez)/L2:0.0;
                                lam=std::min(std::max(lam,0.0),1.0);
                                double cx=prev3[0]+lam*ex, cy=prev3[1]+lam*ey, cz=prev3[2]+lam*ez;
                                double d=(V[0]-cx)*(V[0]-cx)+(V[1]-cy)*(V[1]-cy)+(V[2]-cz)*(V[2]-cz);
                                if (d<bd){ bd=d; bt=prevt+lam*(t-prevt); }
                            }
                            prev3=q; prevt=t;
                        }
                        auto d3at = [&](double t){ Point uv=P2.point_at(t); Point q=S2.point_at(uv[0],uv[1]);
                            double dx=q[0]-V[0], dy=q[1]-V[1], dz=q[2]-V[2]; return dx*dx+dy*dy+dz*dz; };
                        double lo = std::max(pd.first,  bt - (pd.second-pd.first)/M);
                        double hi = std::min(pd.second, bt + (pd.second-pd.first)/M);
                        for (int it = 0; it < 80 && hi-lo > 1e-14*(pd.second-pd.first); ++it) {
                            double m1 = lo + (hi-lo)/3, m2 = hi - (hi-lo)/3;
                            if (d3at(m1) < d3at(m2)) hi = m2; else lo = m1;
                        }
                        return 0.5*(lo+hi);
                    };
                    std::vector<double> tps;
                    for (const Point& V : ipts) tps.push_back(param_near_3d(P, S, V));
                    p2 = split_multi(P, tps);
                    if (std::getenv("SESSION_COREF_DBG")) {
                        std::fprintf(stderr, "[COREF] ei=%d ti=%d split-orig p2=%d want=%d dom=[%.6f,%.6f] tps0=%.9f nk=%d cv=%d\n",
                            ei, ti, (int)p2.size(), (int)edge_ids.size(),
                            P.domain().first, P.domain().second, tps.empty()?-1.0:tps[0],
                            P.nurbsknot_count(), P.cv_count());
                        if (!p2.empty() && p2[0].is_valid() && !tps.empty()) {
                            Point pj = p2[0].get_cv(p2[0].cv_count()-1);
                            Point pe = P.point_at(tps[0]);
                            std::fprintf(stderr, "[COREF]   junction cv=(%.9f,%.9f) P.at(tps0)=(%.9f,%.9f)\n",
                                pj[0], pj[1], pe[0], pe[1]);
                            int nP = P.cv_count();
                            for (int qq = 0; qq < nP; ++qq) {
                                Point pc0 = P.get_cv(qq);
                                Point pn0 = P.get_cv(std::min(qq+1, nP-1));
                                if ((pc0[1]-0.5)*(pn0[1]-0.5) <= 0.0 && std::abs(pc0[0]) < 0.1)
                                    std::fprintf(stderr, "[COREF]   P.cv[%d]=(%.9f,%.9f) cv[%d]=(%.9f,%.9f)\n",
                                        qq, pc0[0], pc0[1], qq+1, pn0[0], pn0[1]);
                            }
                        }
                    }
                    if (do_wrap && (int)p2.size()>=2){ auto w2=NurbsCurve::join({p2.back(),p2.front()}, tol);
                        if(w2.size()==1 && w2[0].is_valid()){ std::vector<NurbsCurve> a2(p2.begin()+1,p2.end()-1);
                            a2.push_back(w2[0]); p2=a2; } else p2.clear(); }
                }
                if ((int)p2.size() != (int)edge_ids.size() && S.is_planar(nullptr, 1e-6)) {
                    if (std::getenv("SESSION_COREF_DBG"))
                        std::fprintf(stderr, "[COREF] ei=%d ti=%d AFFINE fallback\n", ei, ti);
                    p2.clear();
                    // Planar face: for a parallelogram-CV degree-1x1 patch (every box/plane face)
                    // the UV<->3D map is AFFINE, which preserves a rational NURBS with its control
                    // points mapped and weights unchanged -> an EXACT rational-arc pcurve. The mid
                    // CVs of >=90-deg arcs lie OUTSIDE the patch, where Closest::surface_point would
                    // clamp to the domain and corrupt the arc, so invert the affine frame directly;
                    // Closest remains the per-CV fallback for non-affine planar patches.
                    auto [su0,su1] = S.domain(0); auto [sv0,sv1] = S.domain(1);
                    Point O3 = S.point_at(su0, sv0);
                    Point PU = S.point_at(su1, sv0), PV = S.point_at(su0, sv1), PW = S.point_at(su1, sv1);
                    double dux=(PU[0]-O3[0])/(su1-su0), duy=(PU[1]-O3[1])/(su1-su0), duz=(PU[2]-O3[2])/(su1-su0);
                    double dvx=(PV[0]-O3[0])/(sv1-sv0), dvy=(PV[1]-O3[1])/(sv1-sv0), dvz=(PV[2]-O3[2])/(sv1-sv0);
                    double guu=dux*dux+duy*duy+duz*duz, guv=dux*dvx+duy*dvy+duz*dvz, gvv=dvx*dvx+dvy*dvy+dvz*dvz;
                    double gdet = guu*gvv - guv*guv;
                    double psz = O3.distance(PW);
                    bool affine = S.degree(0)==1 && S.degree(1)==1 && gdet > 1e-20
                        && std::abs(PW[0]-PU[0]-PV[0]+O3[0]) + std::abs(PW[1]-PU[1]-PV[1]+O3[1])
                         + std::abs(PW[2]-PU[2]-PV[2]+O3[2]) < psz*1e-9 + 1e-12;
                    auto to_uv = [&](const Point& P, double& u, double& v) {
                        double wx=P[0]-O3[0], wy=P[1]-O3[1], wz=P[2]-O3[2];
                        double bu=wx*dux+wy*duy+wz*duz, bv=wx*dvx+wy*dvy+wz*dvz;
                        u = su0 + (bu*gvv - bv*guv)/gdet;
                        v = sv0 + (guu*bv - guv*bu)/gdet;
                    };
                    for (const NurbsCurve& arc3 : arcs) {
                        NurbsCurve pc; int ord=arc3.order(), nc=arc3.cv_count();
                        bool ok2 = nc>=2 && pc.create(3, arc3.is_rational(), ord, nc)
                                          && pc.nurbsknot_count()==arc3.nurbsknot_count();
                        for (int i=0;i<nc && ok2;++i){ Point Pe=arc3.get_cv(i); double w=arc3.weight(i);
                            double uu, vv;
                            if (affine) to_uv(Pe, uu, vv);
                            else { auto [au,av,dd]=Closest::surface_point(S,Pe); (void)dd; uu=au; vv=av; }
                            ok2 = pc.set_cv_4d(i, uu*w, vv*w, 0.0, w); }
                        for (int k=0;k<arc3.nurbsknot_count() && ok2;++k) ok2 = pc.set_nurbsknot(k, arc3.nurbsknot(k));
                        if (ok2 && pc.is_valid()) { p2.push_back(pc); continue; }
                        auto ad = arc3.domain(); int n = std::min(std::max(arc3.cv_count()*2, 12), 2048);
                        std::vector<Point> uvs;
                        for (int s=0;s<=n;++s){ Point P3=arc3.point_at(ad.first+(ad.second-ad.first)*s/n);
                            auto [uu,vv,dd]=Closest::surface_point(S,P3); (void)dd; uvs.push_back(Point(uu,vv,0.0)); }
                        p2.push_back(NurbsCurve::create(false,1,uvs));
                    }

                }
            }
            bool ok = ((int)p2.size()==(int)edge_ids.size());
            // A MATE-oriented trim's pcurve runs opposite the shared 3D curve (the edge was lifted
            // from the OTHER face's pcurve), so pieces in pcurve-param order do not line up
            // index-for-index with the 3D arcs -- index pairing hands each trim a DIFFERENT arc of
            // the circle. Keep the pieces in pcurve/loop order (meshing chains them in sequence)
            // and instead match each piece to ITS edge geometrically (endpoint-compatible,
            // midpoint tie-break across same-endpoint halves).
            std::vector<int> edge_for(edge_ids.begin(), edge_ids.end());
            if (ok && si>=0 && si<(int)m_surfaces.size()) {
                const NurbsSurface& S2 = m_surfaces[si];
                std::vector<int> ef(p2.size(), -1);
                std::vector<bool> taken(arcs.size(), false);
                bool allm = true;
                for (size_t j = 0; j < p2.size() && allm; ++j) {
                    if (!p2[j].is_valid()) { allm = false; break; }
                    auto pd = p2[j].domain();
                    Point a2u = p2[j].point_at(pd.first), b2u = p2[j].point_at(pd.second);
                    Point m2u = p2[j].point_at(0.5*(pd.first+pd.second));
                    Point a3 = S2.point_at(a2u[0],a2u[1]), b3 = S2.point_at(b2u[0],b2u[1]);
                    Point m3 = S2.point_at(m2u[0],m2u[1]);
                    int hit = -1; double bestm = 1e300;
                    for (size_t k = 0; k < arcs.size(); ++k) {
                        if (taken[k]) continue;
                        Point ea = arcs[k].point_at_start(), eb = arcs[k].point_at_end();
                        bool epok = (a3.distance(ea) < tol && b3.distance(eb) < tol)
                                 || (a3.distance(eb) < tol && b3.distance(ea) < tol);
                        if (!epok) continue;
                        auto ad2 = arcs[k].domain();
                        Point em = arcs[k].point_at(0.5*(ad2.first+ad2.second));
                        double dm = m3.distance(em);
                        if (dm < bestm) { bestm = dm; hit = (int)k; }
                    }
                    if (hit < 0) { allm = false; break; }
                    taken[hit] = true; ef[j] = edge_ids[hit];
                }
                if (allm) edge_for.assign(ef.begin(), ef.end());
            }
            std::vector<int> newtrims;
            for (size_t k=0;k<edge_ids.size();++k){
                int c2idx,tidx;
                NurbsCurve pc = ok ? p2[k] : (k==0 ? m_curves_2d[std::max(c2,0)] : NurbsCurve());
                if(k==0){ c2idx=(c2>=0?c2:add_curve_2d(pc)); if(c2>=0)m_curves_2d[c2]=pc; tidx=ti; }
                else { c2idx=add_curve_2d(pc); tidx=(int)m_trims.size(); m_trims.push_back(BRepTrim()); }
                m_trims[tidx].curve_2d_index=c2idx; m_trims[tidx].edge_index=edge_for[k];
                m_trims[tidx].loop_index=li; m_trims[tidx].reversed=T.reversed; m_trims[tidx].type=T.type;
                newtrims.push_back(tidx); m_topology_edges[edge_for[k]].trim_indices.push_back(tidx);
            }
            if (T.reversed) std::reverse(newtrims.begin(), newtrims.end());
            if (li>=0 && li<(int)m_loops.size()){ auto& tl=m_loops[li].trim_indices;
                auto it=std::find(tl.begin(),tl.end(),ti);
                if(it!=tl.end()){ int pos=(int)(it-tl.begin()); tl.erase(it); tl.insert(tl.begin()+pos,newtrims.begin(),newtrims.end()); } }
        }
    }
    // FINAL PASS: upgrade every UNSPLIT closed section circle (a densely-lifted degree-1 polyline
    // whose vertices verifiably lie on one circle) to the exact rational circle, phase-anchored at
    // its start CV and co-wound with the polyline so trim pcurve<->c3d endpoint correspondence is
    // preserved. The polyline inscribes the circle, so the sphere/torus boundary-integral flux was
    // ~0.6% short per cap loop (box x sphere: the three unsplit cap circles were the whole residual);
    // denser sampling cannot fix wrong geometry. Split circles are handled arc-by-arc above.
    {
        const double PI = 3.14159265358979323846;
        for (auto& e : m_topology_edges) {
            int ci = e.curve_3d_index;
            if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& pl = m_curves_3d[ci];
            if (!pl.is_valid() || pl.degree() != 1 || pl.is_rational() || pl.cv_count() < 12) continue;
            if (pl.point_at_start().distance(pl.point_at_end()) > 1e-9) continue;
            std::vector<Point> cpts;
            for (int i = 0; i < pl.cv_count(); ++i) cpts.push_back(pl.get_cv(i));
            Point fc; Vector fn, f1, f2; double fr = 0;
            if (!circle_through_points(cpts, fc, fn, f1, f2, fr)) continue;
            double sw = 0.0, prev = 0.0; bool first = true;
            for (const auto& p : cpts) {
                double a = std::atan2((p[0]-fc[0])*f2[0]+(p[1]-fc[1])*f2[1]+(p[2]-fc[2])*f2[2],
                                      (p[0]-fc[0])*f1[0]+(p[1]-fc[1])*f1[1]+(p[2]-fc[2])*f1[2]);
                if (!first) { double d = a - prev;
                    while (d >  PI) d -= 2*PI; while (d < -PI) d += 2*PI; sw += d; }
                prev = a; first = false;
            }
            if (std::abs(std::abs(sw) - 2*PI) > 1e-6) continue;
            NurbsCurve ex = exact_arc_3d(fc, f1, f2, fr, 0.0, sw > 0 ? 2*PI : -2*PI);
            if (ex.is_valid()) m_curves_3d[ci] = ex;
        }
    }
    // FINAL PASS 2: coincident PAIRS of under-mated OPEN polyline arcs whose two adjacent
    // surfaces form a recognized quadric pair carry an exactly-derivable section conic
    // (sphere-sphere radical circle; equal-radius crossing cylinders' Steinmetz ellipse).
    // Both sides' lift polylines inscribe it (~1e-4); rebuild BOTH as the exact rational arc
    // through the SAME snapped endpoints so they are bit-identical for sew and the volume
    // integrals. (Split arcs are handled inside the split loop above; this covers the
    // 1:1-mating imprints that never split, e.g. cyl x cyl.)
    {
        const double PI = 3.14159265358979323846;
        double ptol = 5e-3 * 1.0; // endpoint pairing tolerance, refined below from the diag
        {
            double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
            for (const auto& q : m_vertices) { xmn=std::min(xmn,q[0]); ymn=std::min(ymn,q[1]); zmn=std::min(zmn,q[2]);
                xmx=std::max(xmx,q[0]); ymx=std::max(ymx,q[1]); zmx=std::max(zmx,q[2]); }
            double dg = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
            if (dg > 0) ptol = 5e-3 * dg;
        }
        auto srf_of = [&](int e) -> int {
            for (int t : m_topology_edges[e].trim_indices) {
                if (t < 0 || t >= (int)m_trims.size()) continue;
                int l = m_trims[t].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
                int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
                return m_faces[f].surface_index;
            }
            return -1;
        };
        auto arcish_n = [&](int e, int want_trims) -> const NurbsCurve* {
            if ((int)m_topology_edges[e].trim_indices.size() != want_trims) return nullptr;
            int c = m_topology_edges[e].curve_3d_index;
            if (c < 0 || c >= (int)m_curves_3d.size()) return nullptr;
            const NurbsCurve& cv = m_curves_3d[c];
            if (!cv.is_valid() || cv.degree() != 1 || cv.is_rational() || cv.cv_count() < 8) return nullptr;
            if (cv.point_at_start().distance(cv.point_at_end()) < 1e-9) return nullptr;   // closed: pass 1
            return &cv;
        };
        auto arcish = [&](int e){ return arcish_n(e, 1); };
        // surface indices of ALL distinct faces adjacent to an edge (for the 2-trim case)
        auto srfs_of = [&](int e, int& o1, int& o2) {
            o1 = -1; o2 = -1;
            for (int t : m_topology_edges[e].trim_indices) {
                if (t < 0 || t >= (int)m_trims.size()) continue;
                int l = m_trims[t].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
                int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
                int si2 = m_faces[f].surface_index;
                if (o1 < 0) o1 = si2; else if (si2 != o1 && o2 < 0) o2 = si2;
            }
        };
        int ne2 = (int)m_topology_edges.size();
        // SameParameter guard (OCCT invariant): an edge whose 3D curve does not geometrically
        // span what its trims' pcurves span (a cross-assigned or stale curve, e.g. a sub-tol
        // sliver left at a near-duplicate split event) poisons every consumer -- the volume
        // c3 gate falls back to the sagging pcurve lift, sew mismatches, meshing tears.
        // Detect by endpoint+midpoint mismatch against the first trim's surface-mapped pcurve
        // and re-lift the 3D curve from that pcurve; the conic sweep below then upgrades the
        // re-lift to the exact arc whenever it inscribes a registered conic.
        int n_relift = 0;
        for (int e2 = 0; e2 < ne2; ++e2) {
            if (m_topology_edges[e2].trim_indices.empty()) continue;
            int c = m_topology_edges[e2].curve_3d_index;
            if (c < 0 || c >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& cv = m_curves_3d[c];
            if (!cv.is_valid()) continue;
            int ti = m_topology_edges[e2].trim_indices[0];
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int pci = m_trims[ti].curve_2d_index;
            int l = m_trims[ti].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
            int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
            int si = m_faces[f].surface_index;
            if (pci < 0 || pci >= (int)m_curves_2d.size() || si < 0 || si >= (int)m_surfaces.size()) continue;
            const NurbsCurve& pc = m_curves_2d[pci];
            const NurbsSurface& srf = m_surfaces[si];
            auto dc = pc.domain();
            Point uv0 = pc.point_at(dc.first), uv1 = pc.point_at(dc.second), uvm = pc.point_at(0.5*(dc.first+dc.second));
            Point q0 = srf.point_at(uv0[0], uv0[1]), q1 = srf.point_at(uv1[0], uv1[1]), qm = srf.point_at(uvm[0], uvm[1]);
            auto d3 = cv.domain();
            Point ca = cv.point_at(d3.first), cb = cv.point_at(d3.second);
            double dfw = std::max(q0.distance(ca), q1.distance(cb));
            double dbw = std::max(q0.distance(cb), q1.distance(ca));
            double dm = 1e300, dmt = d3.first;
            const int MK = 128;
            for (int k = 0; k <= MK; ++k) {
                double t = d3.first + (d3.second-d3.first)*k/MK;
                double d = qm.distance(cv.point_at(t));
                if (d < dm) { dm = d; dmt = t; }
            }
            {   // refine: coarse samples on a long curve overestimate by half the spacing,
                // which false-flags exact circles as mismatched
                double lo = std::max(d3.first,  dmt - (d3.second-d3.first)/MK);
                double hi = std::min(d3.second, dmt + (d3.second-d3.first)/MK);
                for (int it = 0; it < 60 && hi-lo > 1e-14*(d3.second-d3.first); ++it) {
                    double m1 = lo + (hi-lo)/3, m2 = hi - (hi-lo)/3;
                    if (qm.distance(cv.point_at(m1)) < qm.distance(cv.point_at(m2))) hi = m2; else lo = m1;
                }
                dm = std::min(dm, qm.distance(cv.point_at(0.5*(lo+hi))));
            }
            if (std::min(dfw, dbw) < ptol && dm < ptol) continue;   // curve spans the trim: OK
            int NL = 64;
            std::vector<Point> lp; lp.reserve(NL+1);
            for (int k = 0; k <= NL; ++k) {
                Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/NL);
                lp.push_back(srf.point_at(uv[0], uv[1]));
            }
            NurbsCurve nl = NurbsCurve::create(false, 1, lp);
            if (!nl.is_valid()) continue;
            m_curves_3d.push_back(nl);
            m_topology_edges[e2].curve_3d_index = (int)m_curves_3d.size() - 1;
            ++n_relift;
        }
        // Conic registry: derive each surface-pair section conic ONCE (from any coincident arc
        // pair or any 2-trim arc), then upgrade EVERY inscribed polyline arc through its OWN
        // snapped endpoints. Forcing a pair onto shared endpoints is wrong when the two sides
        // were split at different events (A's seam vs B's seam, ~1e-2 apart): each face's
        // boundary must pass through ITS OWN junctions; the pave engine mates the sides later.
        struct ConicRec { Point C; Vector E1, E2, N; double A, B; };
        std::vector<ConicRec> conics;
        auto reg_conic = [&](const Point& C2, const Vector& E1, const Vector& E2, const Vector& N2, double A2, double B2) {
            for (const auto& g : conics)
                if (g.C.distance(C2) < 1e-9 && std::abs(g.A-A2) < 1e-9 && std::abs(g.B-B2) < 1e-9
                    && std::abs(g.N[0]*N2[0]+g.N[1]*N2[1]+g.N[2]*N2[2]) > 1.0-1e-9) return;
            conics.push_back({C2, E1, E2, N2, A2, B2});
        };
        // sphere-sphere radical circle / equal-radius crossing cylinders' BOTH Steinmetz
        // ellipses; the fit test in the sweep below assigns each arc to its conic.
        auto derive_conics = [&](int s1, int s2) -> bool {
            bool got = false;
            Point C1, C2s; double R1 = 0, R2 = 0;
            if (sphere_of_surface(m_surfaces[s1], C1, R1) && sphere_of_surface(m_surfaces[s2], C2s, R2)) {
                double dx=C2s[0]-C1[0], dy=C2s[1]-C1[1], dz=C2s[2]-C1[2];
                double d = std::sqrt(dx*dx+dy*dy+dz*dz);
                if (d > 1e-9 && d < R1+R2) {
                    double a = (d*d + R1*R1 - R2*R2) / (2.0*d);
                    double rr2 = R1*R1 - a*a;
                    if (rr2 > 1e-18) {
                        Vector fN(dx/d, dy/d, dz/d);
                        Point fC(C1[0]+a*fN[0], C1[1]+a*fN[1], C1[2]+a*fN[2]);
                        double fR = std::sqrt(rr2);
                        int k = std::abs(fN[0]) <= std::abs(fN[1])
                              ? (std::abs(fN[0]) <= std::abs(fN[2]) ? 0 : 2)
                              : (std::abs(fN[1]) <= std::abs(fN[2]) ? 1 : 2);
                        Vector ax(k==0?1.0:0.0, k==1?1.0:0.0, k==2?1.0:0.0);
                        double h = ax[0]*fN[0]+ax[1]*fN[1]+ax[2]*fN[2];
                        double px=ax[0]-h*fN[0], py=ax[1]-h*fN[1], pz=ax[2]-h*fN[2];
                        double pl = std::sqrt(px*px+py*py+pz*pz);
                        if (pl > 1e-12) {
                            Vector fE1(px/pl, py/pl, pz/pl);
                            Vector fE2(fN[1]*fE1[2]-fN[2]*fE1[1], fN[2]*fE1[0]-fN[0]*fE1[2], fN[0]*fE1[1]-fN[1]*fE1[0]);
                            reg_conic(fC, fE1, fE2, fN, fR, fR);
                            got = true;
                        }
                    }
                }
                return got;
            }
            Point A1, A2; Vector W1, W2; double Rc1 = 0, Rc2 = 0;
            if (cylinder_of_surface(m_surfaces[s1], A1, W1, Rc1)
             && cylinder_of_surface(m_surfaces[s2], A2, W2, Rc2)
             && std::abs(Rc1-Rc2) < 1e-6*std::max(Rc1,Rc2)) {
                double b = W1[0]*W2[0]+W1[1]*W2[1]+W1[2]*W2[2];
                double den = 1.0 - b*b;
                if (den > 1e-12) {
                    double d0x=A2[0]-A1[0], d0y=A2[1]-A1[1], d0z=A2[2]-A1[2];
                    double dw1 = d0x*W1[0]+d0y*W1[1]+d0z*W1[2];
                    double dw2 = d0x*W2[0]+d0y*W2[1]+d0z*W2[2];
                    double t1 = (dw1 - b*dw2)/den, t2 = (b*dw1 - dw2)/den;
                    Point q1(A1[0]+t1*W1[0], A1[1]+t1*W1[1], A1[2]+t1*W1[2]);
                    Point q2(A2[0]+t2*W2[0], A2[1]+t2*W2[1], A2[2]+t2*W2[2]);
                    if (q1.distance(q2) < 1e-6) {
                        double Rr = 0.5*(Rc1+Rc2);
                        double cx=W1[1]*W2[2]-W1[2]*W2[1], cy=W1[2]*W2[0]-W1[0]*W2[2], cz=W1[0]*W2[1]-W1[1]*W2[0];
                        double cl = std::sqrt(cx*cx+cy*cy+cz*cz);
                        double ang = std::atan2(cl, b);
                        double sh = std::sin(0.5*ang), ch = std::cos(0.5*ang);
                        if (cl > 1e-9 && sh > 1e-9 && ch > 1e-9) {
                            Vector mnr(cx/cl, cy/cl, cz/cl);
                            Point Pc(0.5*(q1[0]+q2[0]), 0.5*(q1[1]+q2[1]), 0.5*(q1[2]+q2[2]));
                            for (int cand = 0; cand < 2; ++cand) {
                                double sx = cand==0 ? 1.0 : -1.0;
                                double mx=W1[0]+sx*W2[0], my=W1[1]+sx*W2[1], mz=W1[2]+sx*W2[2];
                                double ml = std::sqrt(mx*mx+my*my+mz*mz);
                                if (ml < 1e-9) continue;
                                Vector maj(mx/ml, my/ml, mz/ml);
                                double semiA = cand==0 ? Rr/sh : Rr/ch;
                                Vector n2(maj[1]*mnr[2]-maj[2]*mnr[1], maj[2]*mnr[0]-maj[0]*mnr[2], maj[0]*mnr[1]-maj[1]*mnr[0]);
                                reg_conic(Pc, maj, mnr, n2, semiA, Rr);
                                got = true;
                            }
                        }
                    }
                }
            }
            return got;
        };
        int n_arc1 = 0, n_arc2 = 0, n_up = 0;
        for (int ea = 0; ea < ne2; ++ea) {
            const NurbsCurve* Ca = arcish(ea); if (!Ca) continue; ++n_arc1;
            Point a0 = Ca->point_at_start(), a1 = Ca->point_at_end();
            Point am = Ca->point_at(0.5*(Ca->domain().first+Ca->domain().second));
            for (int eb = ea+1; eb < ne2; ++eb) {
                const NurbsCurve* Cb = arcish(eb); if (!Cb) continue;
                Point b0 = Cb->point_at_start(), b1 = Cb->point_at_end();
                bool fw2 = a0.distance(b0) < ptol && a1.distance(b1) < ptol;
                bool bw2 = a0.distance(b1) < ptol && a1.distance(b0) < ptol;
                if (!fw2 && !bw2) continue;
                Point bm = Cb->point_at(0.5*(Cb->domain().first+Cb->domain().second));
                double chord = a0.distance(a1) + 1e-12;
                if (am.distance(bm) > 0.35*chord + ptol) continue;   // opposite halves share ends
                int s1 = srf_of(ea), s2 = srf_of(eb);
                if (s1 < 0 || s2 < 0 || s1 == s2) continue;
                if (s1 >= (int)m_surfaces.size() || s2 >= (int)m_surfaces.size()) continue;
                if (derive_conics(s1, s2)) break;
            }
        }
        for (int ea = 0; ea < ne2; ++ea) {
            const NurbsCurve* Ca = arcish_n(ea, 2); if (!Ca) continue; ++n_arc2;
            int s1, s2; srfs_of(ea, s1, s2);
            if (s1 < 0 || s2 < 0 || s1 == s2) continue;
            if (s1 >= (int)m_surfaces.size() || s2 >= (int)m_surfaces.size()) continue;
            derive_conics(s1, s2);
        }
        // upgrade sweep: every open polyline arc inscribing a registered conic is rebuilt as
        // the exact rational arc through its OWN radially-snapped endpoints (crossings exact).
        int n_closed = 0;
        for (int ea = 0; ea < ne2 && !conics.empty(); ++ea) {
            const NurbsCurve* Ca = nullptr;
            {
                int nt = (int)m_topology_edges[ea].trim_indices.size();
                int c = m_topology_edges[ea].curve_3d_index;
                if (nt >= 1 && nt <= 2 && c >= 0 && c < (int)m_curves_3d.size()) {
                    const NurbsCurve& cv = m_curves_3d[c];
                    if (cv.is_valid() && cv.degree() == 1 && !cv.is_rational() && cv.cv_count() >= 8)
                        Ca = &cv;
                }
            }
            if (!Ca) continue;
            bool closed = Ca->point_at_start().distance(Ca->point_at_end()) < 1e-9;
            if (closed) ++n_closed;
            for (const auto& g : conics) {
                double mnAB = std::min(g.A, g.B);
                double vtol2 = std::max(5e-3*mnAB, 1e-6);
                bool okv = true;
                for (int ii = 0; ii < Ca->cv_count() && okv; ++ii) {
                    Point q = Ca->get_cv(ii);
                    double ax=q[0]-g.C[0], ay=q[1]-g.C[1], az=q[2]-g.C[2];
                    double hh = ax*g.N[0]+ay*g.N[1]+az*g.N[2];
                    double su = (ax*g.E1[0]+ay*g.E1[1]+az*g.E1[2])/g.A;
                    double sv = (ax*g.E2[0]+ay*g.E2[1]+az*g.E2[2])/g.B;
                    if (std::abs(hh) > vtol2 || std::abs(std::sqrt(su*su+sv*sv)-1.0)*mnAB > vtol2) okv = false;
                }
                if (!okv) continue;
                // Steinmetz crossings are EXACT: both ellipses pass through C +- B*E2 (the shared
                // minor axis); endpoints near one snap THERE so adjacent arcs of the two conics
                // stay watertight. fA==fB (circle) has no true crossing.
                Point xr1(g.C[0]+g.B*g.E2[0], g.C[1]+g.B*g.E2[1], g.C[2]+g.B*g.E2[2]);
                Point xr2(g.C[0]-g.B*g.E2[0], g.C[1]-g.B*g.E2[1], g.C[2]-g.B*g.E2[2]);
                bool have_x = (g.A != g.B);
                auto snapc = [&](const Point& P) {
                    if (have_x && P.distance(xr1) < ptol) return xr1;
                    if (have_x && P.distance(xr2) < ptol) return xr2;
                    double wx=P[0]-g.C[0], wy=P[1]-g.C[1], wz=P[2]-g.C[2];
                    double su = (wx*g.E1[0]+wy*g.E1[1]+wz*g.E1[2])/g.A;
                    double sv = (wx*g.E2[0]+wy*g.E2[1]+wz*g.E2[2])/g.B;
                    double rr = std::sqrt(su*su+sv*sv);
                    if (rr < 1e-12) return P;
                    return Point(g.C[0]+(su/rr)*g.A*g.E1[0]+(sv/rr)*g.B*g.E2[0],
                                 g.C[1]+(su/rr)*g.A*g.E1[1]+(sv/rr)*g.B*g.E2[1],
                                 g.C[2]+(su/rr)*g.A*g.E1[2]+(sv/rr)*g.B*g.E2[2]);
                };
                auto ang_of2 = [&](const Point& P) {
                    double wx=P[0]-g.C[0], wy=P[1]-g.C[1], wz=P[2]-g.C[2];
                    return std::atan2((wx*g.E2[0]+wy*g.E2[1]+wz*g.E2[2])/g.B,
                                      (wx*g.E1[0]+wy*g.E1[1]+wz*g.E1[2])/g.A);
                };
                Point SA = snapc(Ca->point_at_start()), SB = snapc(Ca->point_at_end());
                double sw_poly = 0.0, prev = 0.0; bool first2 = true;
                for (int ii = 0; ii < Ca->cv_count(); ++ii) {
                    double a = ang_of2(Ca->get_cv(ii));
                    if (!first2) { double d = a - prev;
                        while (d >  PI) d -= 2*PI; while (d < -PI) d += 2*PI; sw_poly += d; }
                    prev = a; first2 = false;
                }
                double aa0 = ang_of2(SA), aa1 = ang_of2(SB);
                double sw;
                if (closed) {
                    if (std::abs(std::abs(sw_poly) - 2*PI) > 0.1) break;   // not a simple loop
                    sw = sw_poly >= 0 ? 2*PI : -2*PI;
                } else {
                    sw = (aa1 - aa0) + 2*PI*std::round((sw_poly - (aa1 - aa0)) / (2*PI));
                }
                if (std::abs(sw) < 1e-9 || std::abs(sw) > 2*PI + 1e-9) break;
                Vector sE1(g.E1[0]*g.A, g.E1[1]*g.A, g.E1[2]*g.A);
                Vector sE2(g.E2[0]*g.B, g.E2[1]*g.B, g.E2[2]*g.B);
                NurbsCurve na2 = exact_arc_3d(g.C, sE1, sE2, 1.0, aa0, sw);
                if (na2.is_valid()) { m_curves_3d[m_topology_edges[ea].curve_3d_index] = na2; ++n_up; }
                break;
            }
        }
        if (std::getenv("SESSION_XCHK")) {
            std::fprintf(stderr, "[PAIRPASS] arc1=%d arc2=%d conics=%d up=%d closed=%d relift=%d\n", n_arc1, n_arc2, (int)conics.size(), n_up, n_closed, n_relift);
            for (int e = 0; e < ne2; ++e) {
                int c = m_topology_edges[e].curve_3d_index;
                if (c < 0 || c >= (int)m_curves_3d.size()) continue;
                const NurbsCurve& cv = m_curves_3d[c];
                if (!cv.is_valid() || cv.degree() != 1 || cv.is_rational()) continue;
                Point q0 = cv.point_at_start(), q1 = cv.point_at_end();
                Point qm = cv.point_at(0.5*(cv.domain().first+cv.domain().second));
                double dev = 0.0;
                { double cxx=q1[0]-q0[0], cyy=q1[1]-q0[1], czz=q1[2]-q0[2];
                  double cl2 = cxx*cxx+cyy*cyy+czz*czz;
                  for (int ii = 0; ii < cv.cv_count(); ++ii) { Point q = cv.get_cv(ii);
                    double wx=q[0]-q0[0], wy=q[1]-q0[1], wz=q[2]-q0[2];
                    double t = cl2 > 1e-24 ? (wx*cxx+wy*cyy+wz*czz)/cl2 : 0.0;
                    double dx=wx-t*cxx, dy=wy-t*cyy, dz=wz-t*czz;
                    dev = std::max(dev, std::sqrt(dx*dx+dy*dy+dz*dz)); } }
                std::fprintf(stderr, "[POLY] e=%d nt=%d cvs=%d dev=%.4f s(%.3f,%.3f,%.3f) m(%.3f,%.3f,%.3f) e(%.3f,%.3f,%.3f)\n",
                    e, (int)m_topology_edges[e].trim_indices.size(), cv.cv_count(), dev,
                    q0[0],q0[1],q0[2], qm[0],qm[1],qm[2], q1[0],q1[1],q1[2]);
            }
        }
    }
    // rebuild vertex->edge adjacency
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei=0; ei<(int)m_topology_edges.size(); ++ei){ int sv=m_topology_edges[ei].start_vertex, ev=m_topology_edges[ei].end_vertex;
        if(sv>=0&&sv<(int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if(ev!=sv&&ev>=0&&ev<(int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei); }
}

int BRep::recover_section_spans(const SectionScaffold& scaf,
                                std::map<int, std::array<double, 3>>& spans) {
    if (scaf.segments.empty() || m_topology_edges.empty()) return 0;
    double diag;
    {
        double mn[3] = {1e300,1e300,1e300}, mx[3] = {-1e300,-1e300,-1e300};
        for (const auto& p : m_vertices) for (int d=0; d<3; ++d) { mn[d]=std::min(mn[d],p[d]); mx[d]=std::max(mx[d],p[d]); }
        diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0])+(mx[1]-mn[1])*(mx[1]-mn[1])+(mx[2]-mn[2])*(mx[2]-mn[2]));
        if (diag <= 0) diag = 1.0;
    }
    const double tol = diag * 3e-3;   // tighter than sew: a recovered edge must HUG its segment
    // per-segment bbox
    std::vector<std::array<double,6>> sbb(scaf.segments.size());
    for (size_t si = 0; si < scaf.segments.size(); ++si) {
        std::array<double,6> bb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (const auto& p : scaf.segments[si].p3)
            for (int d=0; d<3; ++d) { bb[d]=std::min(bb[d],p[d]); bb[d+3]=std::max(bb[d+3],p[d]); }
        sbb[si] = bb;
    }
    auto pt_seg = [](const Point& p, const Point& a, const Point& b, double& t) -> double {
        double ex=b[0]-a[0], ey=b[1]-a[1], ez=b[2]-a[2], L2=ex*ex+ey*ey+ez*ez;
        t = L2 > 1e-30 ? ((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2 : 0.0;
        t = std::min(std::max(t,0.0),1.0);
        double cx=a[0]+t*ex, cy=a[1]+t*ey, cz=a[2]+t*ez;
        return std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz));
    };
    auto chain_idx = [&](const std::vector<Point>& p3, const Point& p, double& dmin) -> double {
        dmin = 1e300; double bf = 0.0;
        for (int k = 0; k+1 < (int)p3.size(); ++k) {
            double t; double d = pt_seg(p, p3[k], p3[k+1], t);
            if (d < dmin) { dmin = d; bf = (double)k + t; }
        }
        return bf;
    };
    int n_rec = 0;
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        const BRepEdge& E = m_topology_edges[ei];
        if ((int)E.trim_indices.size() >= 2) continue;   // already mated
        if (spans.count(ei)) continue;                    // already keyed
        int ci = E.curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        const NurbsCurve& C = m_curves_3d[ci];
        if (!C.is_valid()) continue;
        auto [t0, t1] = C.domain();
        const int M = 8;
        std::vector<Point> es; std::array<double,6> ebb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= M; ++k) {
            Point p = C.point_at(t0 + (t1-t0)*k/M);
            es.push_back(p);
            for (int d=0; d<3; ++d) { ebb[d]=std::min(ebb[d],p[d]); ebb[d+3]=std::max(ebb[d+3],p[d]); }
        }
        int best = -1;
        for (size_t si = 0; si < scaf.segments.size(); ++si) {
            const auto& B6 = sbb[si];
            if (ebb[0]>B6[3]+tol || B6[0]>ebb[3]+tol || ebb[1]>B6[4]+tol ||
                B6[1]>ebb[4]+tol || ebb[2]>B6[5]+tol || B6[2]>ebb[5]+tol) continue;
            const auto& p3 = scaf.segments[si].p3;
            if ((int)p3.size() < 2) continue;
            bool on = true;
            for (const auto& p : es) { double dm; chain_idx(p3, p, dm); if (dm > tol) { on = false; break; } }
            if (on) { best = (int)si; break; }
        }
        if (best < 0) continue;
        const auto& seg = scaf.segments[best];
        double dfa, dfb;
        double fa = chain_idx(seg.p3, es.front(), dfa);
        double fb = chain_idx(seg.p3, es.back(),  dfb);
        if (fa > fb) std::swap(fa, fb);
        if (fb - fa < 1e-6) continue;
        spans[ei] = {(double)seg.seg_id, fa, fb};
        ++n_rec;
    }
    if (std::getenv("SESSION_NT_DBG"))
        std::fprintf(stderr, "[RECOVER] section spans recovered: %d\n", n_rec);
    return n_rec;
}

int BRep::normalize_section_blocks(const SectionScaffold& scaf,
                                   std::map<int, std::array<double, 3>>& spans,
                                   std::map<int, std::array<int, 3>>* sec_edges,
                                   const char* tag,
                                   const std::map<int, std::vector<double>>* shared_centers) {
    // OCCT BOPDS pave/common-block analog on the scaffold's shared chains. Each chain-lifted
    // section edge covers a fractional index range [fa,fb] of its segment; flanks and
    // operands clip at different arrangement params, so copies stagger and Hausdorff sew
    // cannot pair them. Per segment, the union of all range ends clusters into PAVES; every
    // edge is split at the paves interior to its range (3D re-extracted from the shared
    // chain, so pieces are bit-identical across copies; pcurves split at the projected pave
    // UV) and same-block copies merge into one edge (capped at 2 trims).
    if (spans.empty()) return 0;
    const double eps_f = 0.03;             // pave weld radius, fractional chain-index units
    bool dbg = std::getenv("SESSION_NT_DBG") != nullptr;

    std::map<int, std::vector<int>> by_seg;
    for (auto it = spans.begin(); it != spans.end(); ) {
        int ei = it->first, sid = (int)it->second[0];
        bool ok = ei >= 0 && ei < (int)m_topology_edges.size()
               && sid >= 0 && sid < (int)scaf.segments.size()
               && !m_topology_edges[ei].trim_indices.empty()
               && m_topology_edges[ei].curve_3d_index >= 0
               && m_topology_edges[ei].curve_3d_index < (int)m_curves_3d.size();
        if (!ok) { it = spans.erase(it); continue; }
        by_seg[sid].push_back(ei);
        ++it;
    }
    int n_split = 0, n_merged = 0, n_projfail = 0, n_micro = 0, n_cap = 0;
    for (auto& sk : by_seg) {
        int sid = sk.first;
        const SectionSegment& s = scaf.segments[sid];
        int nCh = (int)s.p3.size();
        if (nCh < 2) continue;
        auto lerp3 = [&](double f) {
            f = std::min(std::max(f, 0.0), (double)(nCh - 1));
            int k = std::min((int)f, nCh - 2);
            double w = f - k;
            return Point(s.p3[k][0] + (s.p3[k+1][0]-s.p3[k][0])*w,
                         s.p3[k][1] + (s.p3[k+1][1]-s.p3[k][1])*w,
                         s.p3[k][2] + (s.p3[k+1][2]-s.p3[k][2])*w);
        };
        auto lerpUV = [&](const std::vector<Point>& uv, double f) {
            f = std::min(std::max(f, 0.0), (double)(nCh - 1));
            int k = std::min((int)f, nCh - 2);
            double w = f - k;
            return Point(uv[k][0] + (uv[k+1][0]-uv[k][0])*w,
                         uv[k][1] + (uv[k+1][1]-uv[k][1])*w, 0.0);
        };
        // Cluster all range ends into paves (single-link, gap > eps_f); chain ends exact.
        // P1b (SESSION_SHAREDPAVE): when a SHARED pave set for this segment is supplied (clustered
        // once from BOTH operands' range-ends before the two normalize calls), use it verbatim so
        // A2 and B2 split at IDENTICAL interior boundaries -> the block edges' [fa,fb] match and the
        // combine's qspan alias mates them 2-trim by construction, instead of each operand paving at
        // its own crossings and staggering. Falls back to the per-operand clustering otherwise.
        std::vector<double> centers;
        const std::vector<double>* shc = nullptr;
        if (shared_centers) {
            auto it = shared_centers->find(sid);
            if (it != shared_centers->end() && it->second.size() >= 2) shc = &it->second;
        }
        if (shc) {
            centers = *shc;
        } else {
            std::vector<double> vals = {0.0, (double)(nCh - 1)};
            for (int ei : sk.second) { vals.push_back(spans[ei][1]); vals.push_back(spans[ei][2]); }
            std::sort(vals.begin(), vals.end());
            double lo = vals[0], hi = vals[0], sum = vals[0]; int cnt = 1;
            auto flush = [&]() {
                double c = sum / std::max(cnt, 1);
                if (lo <= 1e-9) c = 0.0;
                if (hi >= nCh - 1 - 1e-9) c = (double)(nCh - 1);
                centers.push_back(c);
            };
            for (size_t i = 1; i < vals.size(); ++i) {
                if (vals[i] - hi > eps_f) { flush(); lo = vals[i]; sum = 0.0; cnt = 0; }
                hi = vals[i]; sum += vals[i]; ++cnt;
            }
            flush();
        }
        auto cl_of = [&](double f) {
            int best = 0; double bd = 1e300;
            for (int i = 0; i < (int)centers.size(); ++i) {
                double d = std::abs(centers[i] - f);
                if (d < bd) { bd = d; best = i; }
            }
            return best;
        };
        auto vert_pos = [&](int tv) -> Point {
            int pi = (tv >= 0 && tv < (int)m_topology_vertices.size())
                   ? m_topology_vertices[tv].point_index : -1;
            return (pi >= 0 && pi < (int)m_vertices.size()) ? m_vertices[pi]
                                                           : Point(1e30, 1e30, 1e30);
        };
        // Seed pave -> topology vertex from existing edge endpoints so split points reuse
        // the vertices the copies already share (first seen wins; identical 3D by lerp).
        std::map<int, int> pave_vert;
        for (int ei : sk.second) {
            const BRepEdge& E = m_topology_edges[ei];
            int ca = cl_of(spans[ei][1]), cb = cl_of(spans[ei][2]);
            if (ca == cb) continue;
            Point pa = lerp3(centers[ca]), pb = lerp3(centers[cb]);
            for (int tv : {E.start_vertex, E.end_vertex}) {
                if (tv < 0) continue;
                Point q = vert_pos(tv);
                pave_vert.emplace(q.distance(pa) <= q.distance(pb) ? ca : cb, tv);
            }
        }
        auto pave_vertex = [&](int c) {
            auto it = pave_vert.find(c);
            if (it != pave_vert.end()) return it->second;
            int pi = add_vertex(lerp3(centers[c]));
            BRepVertex tv; tv.point_index = pi;
            m_topology_vertices.push_back(tv);
            int idx = (int)m_topology_vertices.size() - 1;
            pave_vert[c] = idx;
            return idx;
        };
        auto chain_sub = [&](double f0, double f1) {
            std::vector<Point> sub;
            sub.push_back(lerp3(f0));
            for (int k = (int)std::ceil(f0 + 1e-9); k <= (int)std::floor(f1 - 1e-9) && k < nCh; ++k)
                sub.push_back(s.p3[k]);
            sub.push_back(lerp3(f1));
            return sub;
        };
        std::map<std::pair<int, int>, std::vector<int>> byblk;
        for (int ei : sk.second) {
            double fa = spans[ei][1], fb = spans[ei][2];
            int ca = cl_of(fa), cb = cl_of(fb);
            if (ca > cb) std::swap(ca, cb);
            if (ca == cb) { ++n_micro; continue; }
            std::vector<int> paves = {ca};
            for (int c = ca + 1; c < cb; ++c) paves.push_back(c);
            paves.push_back(cb);
            int nblk = (int)paves.size() - 1;
            if (nblk == 1) { byblk[{ca, cb}].push_back(ei); continue; }
            // f positions of the piece bounds: outer ends keep the edge's ORIGINAL fa/fb
            // (its endpoint vertices and pcurve ends stay put); interior cuts at pave centers.
            std::vector<double> fs = {fa};
            for (int j = 1; j < nblk; ++j) fs.push_back(centers[paves[j]]);
            fs.push_back(fb);
            // Validate every trim's pcurve projection BEFORE mutating anything.
            BRepEdge& E0 = m_topology_edges[ei];
            std::vector<int> tsnap = E0.trim_indices;
            struct TrimCut { std::vector<double> ts; bool asc; };
            std::vector<TrimCut> tcuts(tsnap.size());
            bool fail = false;
            for (size_t w = 0; w < tsnap.size() && !fail; ++w) {
                const BRepTrim& T = m_trims[tsnap[w]];
                if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)m_curves_2d.size()) { fail = true; break; }
                const NurbsCurve& pc = m_curves_2d[T.curve_2d_index];
                // side pick: whichever operand's uv footprint the pcurve actually follows
                double dA = 0.0, dB = 0.0;
                std::vector<double> tsA(nblk - 1), tsB(nblk - 1);
                for (int j = 1; j < nblk; ++j) {
                    Point ua = lerpUV(s.uvA, fs[j]), ub = lerpUV(s.uvB, fs[j]);
                    tsA[j-1] = pc.closest_parameter(ua);
                    tsB[j-1] = pc.closest_parameter(ub);
                    dA += pc.point_at(tsA[j-1]).distance(ua);
                    dB += pc.point_at(tsB[j-1]).distance(ub);
                }
                const std::vector<Point>& uvs = dA <= dB ? s.uvA : s.uvB;
                std::vector<double>& ts = dA <= dB ? tsA : tsB;
                for (int j = 1; j < nblk && !fail; ++j) {
                    Point ug = lerpUV(uvs, fs[j]);
                    Point lo2 = lerpUV(uvs, fs[j] - 0.5), hi2 = lerpUV(uvs, fs[j] + 0.5);
                    double chord2d = std::max(lo2.distance(hi2), 1e-7);
                    if (pc.point_at(ts[j-1]).distance(ug) > chord2d) fail = true;
                }
                if (fail) break;
                bool asc = true;
                if (nblk >= 3) asc = ts[0] < ts[nblk - 2];
                else {
                    auto dpc = pc.domain();
                    Point st = pc.point_at(dpc.first);
                    asc = st.distance(lerpUV(uvs, fs.front())) <= st.distance(lerpUV(uvs, fs.back()));
                }
                std::vector<double> sorted_ts = ts;
                std::sort(sorted_ts.begin(), sorted_ts.end());
                for (int j = 0; j + 1 < (int)sorted_ts.size(); ++j)
                    if (sorted_ts[j+1] - sorted_ts[j] < 1e-12) fail = true;
                auto dpc = pc.domain();
                if (!sorted_ts.empty() &&
                    (sorted_ts.front() <= dpc.first + 1e-12 || sorted_ts.back() >= dpc.second - 1e-12))
                    fail = true;
                tcuts[w].ts = sorted_ts;
                tcuts[w].asc = asc;
            }
            if (fail) { ++n_projfail; byblk[{ca, cb}].push_back(ei); continue; }
            // ---- mutate: 3D pieces + block edges ----
            int va = E0.start_vertex, vb = E0.end_vertex;
            if (vert_pos(va).distance(lerp3(fa)) > vert_pos(vb).distance(lerp3(fa))) std::swap(va, vb);
            std::vector<int> bedge(nblk, -1);
            for (int j = 0; j < nblk; ++j) {
                std::vector<Point> sub = chain_sub(fs[j], fs[j+1]);
                if ((int)sub.size() < 2) { fail = true; break; }
                NurbsCurve c3 = NurbsCurve::create(false, 1, sub);
                if (!c3.is_valid()) { fail = true; break; }
                int v0 = (j == 0) ? va : pave_vertex(paves[j]);
                int v1 = (j == nblk - 1) ? vb : pave_vertex(paves[j + 1]);
                if (j == 0) {
                    m_curves_3d[E0.curve_3d_index] = c3;
                    E0.start_vertex = v0;
                    E0.end_vertex = v1;
                    bedge[0] = ei;
                } else {
                    int c3i = add_curve_3d(c3);
                    bedge[j] = add_edge(c3i, v0, v1);
                }
            }
            if (fail) { ++n_projfail; byblk[{ca, cb}].push_back(ei); continue; }
            ++n_split;
            // ---- per trim: split pcurve, retarget/create trims, splice the loop ----
            m_topology_edges[ei].trim_indices.clear();
            for (size_t w = 0; w < tsnap.size(); ++w) {
                int ti = tsnap[w];
                BRepTrim T = m_trims[ti];
                NurbsCurve pc = m_curves_2d[T.curve_2d_index];
                std::vector<NurbsCurve> pcs;
                {
                    NurbsCurve rem = pc;
                    bool okc = true;
                    for (double t : tcuts[w].ts) {
                        auto lr = rem.split(t);
                        if (!lr.first.is_valid() || !lr.second.is_valid()) { okc = false; break; }
                        pcs.push_back(lr.first);
                        rem = lr.second;
                    }
                    if (!okc) pcs.clear(); else pcs.push_back(rem);
                }
                if ((int)pcs.size() != nblk) {
                    // pcurve split failed: keep the whole pcurve on the FIRST block edge in
                    // traversal order (degraded but consistent; sew may still rescue).
                    m_trims[ti].edge_index = tcuts[w].asc ? bedge[0] : bedge[nblk-1];
                    m_topology_edges[m_trims[ti].edge_index].trim_indices.push_back(ti);
                    ++n_projfail;
                    continue;
                }
                std::vector<int> newtrims;
                for (int j = 0; j < nblk; ++j) {
                    int blk = tcuts[w].asc ? j : (nblk - 1 - j);   // pcurve piece j -> block
                    int tidx;
                    if (j == 0) {
                        m_curves_2d[T.curve_2d_index] = pcs[0];
                        tidx = ti;
                    } else {
                        tidx = (int)m_trims.size();
                        m_trims.push_back(BRepTrim());
                        m_trims[tidx].curve_2d_index = add_curve_2d(pcs[j]);
                    }
                    m_trims[tidx].edge_index = bedge[blk];
                    m_trims[tidx].loop_index = T.loop_index;
                    m_trims[tidx].reversed = T.reversed;
                    m_trims[tidx].type = T.type;
                    newtrims.push_back(tidx);
                    m_topology_edges[bedge[blk]].trim_indices.push_back(tidx);
                }
                if (T.reversed) std::reverse(newtrims.begin(), newtrims.end());
                int li = T.loop_index;
                if (li >= 0 && li < (int)m_loops.size()) {
                    auto& tl = m_loops[li].trim_indices;
                    auto itl = std::find(tl.begin(), tl.end(), ti);
                    if (itl != tl.end()) {
                        int pos = (int)(itl - tl.begin());
                        tl.erase(itl);
                        tl.insert(tl.begin() + pos, newtrims.begin(), newtrims.end());
                    }
                }
            }
            // span records for the pieces
            for (int j = 0; j < nblk; ++j) {
                spans[bedge[j]] = {(double)sid, fs[j], fs[j+1]};
                byblk[{paves[j], paves[j+1]}].push_back(bedge[j]);
            }
            if (sec_edges) sec_edges->erase(ei);
        }
        // ---- merge same-block copies (cap 2 trims per edge) ----
        for (auto& bk : byblk) {
            auto& es = bk.second;
            if (es.size() < 2) continue;
            std::sort(es.begin(), es.end());
            es.erase(std::unique(es.begin(), es.end()), es.end());
            int rep = es[0];
            for (size_t j = 1; j < es.size(); ++j) {
                auto& rt = m_topology_edges[rep].trim_indices;
                auto& et = m_topology_edges[es[j]].trim_indices;
                if ((int)rt.size() >= 2) { if (!et.empty()) ++n_cap; break; }
                while (!et.empty() && (int)rt.size() < 2) {
                    int ti = et.back(); et.pop_back();
                    m_trims[ti].edge_index = rep;
                    rt.push_back(ti);
                }
                if (!et.empty()) ++n_cap; else ++n_merged;
            }
        }
    }
    // Re-key every surviving chain-lifted section edge so classification's flood/repair
    // treat partial blocks as true section separators (not just whole-segment edges).
    if (sec_edges)
        for (auto& kv : spans) {
            if (m_topology_edges[kv.first].trim_indices.empty()) continue;
            const BRepEdge& E = m_topology_edges[kv.first];
            (*sec_edges)[kv.first] = {(int)kv.second[0], E.start_vertex, E.end_vertex};
        }
    // Compact edges emptied by the merge (a 0-trim edge with real extent breaks is_solid).
    {
        int ne = (int)m_topology_edges.size();
        bool any_dead = false;
        for (auto& kv : spans)
            if (m_topology_edges[kv.first].trim_indices.empty()) { any_dead = true; break; }
        if (any_dead) {
            std::vector<int> old2new(ne, -1);
            std::vector<BRepEdge> keep;
            for (int i = 0; i < ne; ++i) {
                if (m_topology_edges[i].trim_indices.empty() && spans.count(i)) continue;
                old2new[i] = (int)keep.size();
                keep.push_back(m_topology_edges[i]);
            }
            m_topology_edges = std::move(keep);
            for (auto& t : m_trims)
                if (t.edge_index >= 0 && t.edge_index < ne && old2new[t.edge_index] >= 0)
                    t.edge_index = old2new[t.edge_index];
            std::map<int, std::array<double, 3>> sp2;
            for (auto& kv : spans)
                if (old2new[kv.first] >= 0) sp2[old2new[kv.first]] = kv.second;
            spans.swap(sp2);
            if (sec_edges) {
                std::map<int, std::array<int, 3>> se2;
                for (auto& kv : *sec_edges)
                    if (kv.first < ne && old2new[kv.first] >= 0) se2[old2new[kv.first]] = kv.second;
                sec_edges->swap(se2);
            }
        }
    }
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int e2 = 0; e2 < (int)m_topology_edges.size(); ++e2) {
        const BRepEdge& E = m_topology_edges[e2];
        if (E.start_vertex >= 0 && E.start_vertex < (int)m_topology_vertices.size())
            m_topology_vertices[E.start_vertex].edge_indices.push_back(e2);
        if (E.end_vertex >= 0 && E.end_vertex < (int)m_topology_vertices.size() && E.end_vertex != E.start_vertex)
            m_topology_vertices[E.end_vertex].edge_indices.push_back(e2);
    }
    if (dbg)
        std::fprintf(stderr, "[BLOCKS %s] split=%d merged=%d cap=%d micro=%d projfail=%d\n",
                     tag, n_split, n_merged, n_cap, n_micro, n_projfail);
    return n_merged;
}

void BRep::make_shared_section_edges(const BRep& A, const BRep& B, double tol) {
    // BUILDSPEC P1 — PAVE ENGINE: split-exactly-once + PostTreatFF.
    //
    // *this is the already-combined+imprinted boolean result (A's selected split faces + B's, in
    // one geometry pool). The A&B section curve is computed ONCE per surface pair (the exact
    // analytic conic), then a port of the OCCT pave model decomposes each section curve into its
    // canonical split-exactly-once segment set:
    //   - PutPavesOnCurve     : project existing On/In vertices onto the curve (parametric dedup),
    //   - PutBoundPaveOnCurve  : the two curve ends are paves,
    //   - PutClosingPaveOnCurve: a closed circle gets a closing pave (same vid, opposite bound),
    //   - PostTreatFF          : section/section crossings (3-surface corners) become paves on both,
    //   - update_paveblock     : sort paves by parameter, emit consecutive [t_i,t_{i+1}] segments.
    // Each under-mated section edge is then re-fit to the EXACT sub-arc of that single curve over
    // the pave-segment span it occupies (endpoints SNAPPED to the shared pave parameters), so the
    // box-side and periodic-side arcs of a segment become bit-identical geometry and merge into ONE
    // shared edge (two trims, one per operand) -- watertight by construction. The proven co-refine
    // (topological split of a closed circle vs its arcs) and Hausdorff sew (merge) remain the
    // split/merge backbone; the pave engine supplies the principled, shared split parameters that
    // generalise P0's seam-only paves to arbitrary multi-segment / crossing section topology.

    // ---- OCCT pave model (BOPDS_Pave / BOPDS_PaveBlock) ----
    struct Pave      { int vid; double t; };
    struct PaveBlock { int orig = -1; int edge = -1; Pave p1{-1,0.0}, p2{-1,0.0}; std::vector<Pave> ext; };

    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 5e-3;

    // ---- Phase 1 (PerformFF): the exact section curve(s), computed ONCE per surface pair ----
    struct Sec {
        std::vector<Point> samp; std::array<double,6> bb; NurbsCurve c3d; bool closed;
        double t0, t1; PaveBlock master;
        std::vector<std::array<double,2>> segs;   // canonical [t_i,t_{i+1}] spans (update_paveblock)
    };
    std::vector<Sec> secs;
    const int NSEC = 48;
    for (const auto& fa : A.m_faces) {
        if (fa.surface_index < 0 || fa.surface_index >= (int)A.m_surfaces.size()) continue;
        const NurbsSurface& sa = A.m_surfaces[fa.surface_index];
        auto abb = aabb_from_surface(sa);
        double am = std::max({abb.second[0]-abb.first[0], abb.second[1]-abb.first[1], abb.second[2]-abb.first[2]}) * 1e-3;
        for (const auto& fb : B.m_faces) {
            if (fb.surface_index < 0 || fb.surface_index >= (int)B.m_surfaces.size()) continue;
            const NurbsSurface& sb = B.m_surfaces[fb.surface_index];
            auto bbb = aabb_from_surface(sb);
            if (!aabb_overlap(abb, bbb, am)) continue;
            for (auto& tr : Intersection::surface_surface(sa, sb, tol)) {
                NurbsCurve c3 = std::get<0>(tr);
                if (!c3.is_valid()) continue;
                Sec s; s.c3d = c3;
                auto [t0, t1] = c3.domain();
                s.t0 = t0; s.t1 = t1;
                std::array<double,6> bb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
                for (int k = 0; k <= NSEC; ++k) {
                    Point p = c3.point_at(t0 + (t1-t0)*k/NSEC);
                    s.samp.push_back(p);
                    for (int d = 0; d < 3; ++d) { bb[d] = std::min(bb[d], p[d]); bb[d+3] = std::max(bb[d+3], p[d]); }
                }
                s.bb = bb;
                s.closed = c3.point_at(t0).distance(c3.point_at(t1)) < tol;
                secs.push_back(std::move(s));
            }
        }
    }
    if (secs.empty()) { sew_coincident_edges(tol); return; }   // nothing recognised -> legacy merge

    // resolution(C,t): parametric tolerance = tol3d / |C'(t)| (GeomAdaptor::Resolution analogue).
    auto resolution = [&](const NurbsCurve& C, double t) -> double {
        auto d = C.evaluate(t, 1);
        double s = (d.size() > 1) ? d[1].magnitude() : 0.0;
        return (s > 1e-12) ? (tol / s) : tol;
    };
    // ContainsParameter (BOPDS_PaveBlock): is t already represented (within param-tol) among the
    // block's bound + ext paves?
    auto contains_param = [&](const PaveBlock& pb, double t, double ptol) -> bool {
        if (std::abs(t - pb.p1.t) <= ptol || std::abs(t - pb.p2.t) <= ptol) return true;
        for (const auto& e : pb.ext) if (std::abs(t - e.t) <= ptol) return true;
        return false;
    };

    // ---- Phase 2 (paves) ----
    // PutBoundPaveOnCurve: the two curve ends are the natural bound paves of the section curve.
    for (auto& s : secs) { s.master.p1 = Pave{-1, s.t0}; s.master.p2 = Pave{-1, s.t1}; }
    // PutPavesOnCurve: project every existing topology vertex that lies ON a section curve and
    // append it as an interior pave (parametric dedup via resolution). These are the genuine split
    // vertices where the section's arcs meet on this BRep (e.g. the periodic-seam crossings).
    for (int vid = 0; vid < (int)m_topology_vertices.size(); ++vid) {
        int pidx = m_topology_vertices[vid].point_index;
        if (pidx < 0 || pidx >= (int)m_vertices.size()) continue;
        const Point& P = m_vertices[pidx];
        for (auto& s : secs) {
            if (P[0] < s.bb[0]-tol || P[0] > s.bb[3]+tol || P[1] < s.bb[1]-tol ||
                P[1] > s.bb[4]+tol || P[2] < s.bb[2]-tol || P[2] > s.bb[5]+tol) continue;
            double t = s.c3d.closest_parameter(P);
            if (s.c3d.point_at(t).distance(P) > tol) continue;
            double frac = (s.t1 > s.t0) ? (t - s.t0) / (s.t1 - s.t0) : 0.0;
            if (frac <= 1e-6 || frac >= 1.0 - 1e-6) continue;     // bounds already paves
            if (contains_param(s.master, t, resolution(s.c3d, t))) continue;
            s.master.ext.push_back(Pave{vid, t});
        }
    }
    // PutClosingPaveOnCurve: a CLOSED section curve has point_at(t0) ~ point_at(t1); its two bound
    // paves are the same vertex, so the closing pave is the bound pair itself. With K interior
    // paves it yields K segments plus the seam-wrap segment (realised by co-refine's wrap-join);
    // with none it stays one closed edge. Nothing further to append here.

    // ---- Phase 3 (PostTreatFF): intersect section curves among themselves; split at crossings ----
    // GATED on a genuine interior crossing within tol of two DISTINCT section curves (a 3-surface
    // corner). Single-section configs (box-sphere, box-cyl caps) have no crossings -> dormant.
    for (size_t i = 0; i < secs.size(); ++i) {
        for (size_t j = i + 1; j < secs.size(); ++j) {
            const auto& bi = secs[i].bb; const auto& bj = secs[j].bb;
            if (bi[0] > bj[3]+tol || bj[0] > bi[3]+tol || bi[1] > bj[4]+tol ||
                bj[1] > bi[4]+tol || bi[2] > bj[5]+tol || bj[2] > bi[5]+tol) continue;
            auto [ui, uj, dist] = Closest::curve_curve(secs[i].c3d, secs[j].c3d);
            if (dist > tol) continue;
            auto interior = [&](const Sec& s, double tc) {
                double frac = (s.t1 > s.t0) ? (tc - s.t0) / (s.t1 - s.t0) : 0.0;
                return frac > 1e-6 && frac < 1.0 - 1e-6 &&
                       !contains_param(s.master, tc, resolution(s.c3d, tc));
            };
            bool ii = interior(secs[i], ui), ij = interior(secs[j], uj);
            if (!ii && !ij) continue;                              // shared endpoint, not a crossing
            // a brand-new shared corner vertex (3-surface corner); fuse vids by reusing one point.
            Point Pc = secs[i].c3d.point_at(ui);
            int pidx = add_vertex(Pc);
            int vid = (int)m_topology_vertices.size();
            BRepVertex tv; tv.point_index = pidx; m_topology_vertices.push_back(tv);
            if (ii) secs[i].master.ext.push_back(Pave{vid, ui});
            if (ij) secs[j].master.ext.push_back(Pave{vid, uj});
        }
    }

    // ---- update_paveblock (BOPDS_PaveBlock::Update): sort ext+bound paves by parameter and emit
    // consecutive [t_i,t_{i+1}] segments, skipping zero-length (|t1-t2| < pconf) ones. ----
    for (auto& s : secs) {
        std::vector<Pave> all = s.master.ext;
        all.push_back(s.master.p1);
        all.push_back(s.master.p2);
        std::sort(all.begin(), all.end(), [](const Pave& a, const Pave& b){ return a.t < b.t; });
        double pconf = std::max(resolution(s.c3d, 0.5*(s.t0+s.t1)), 1e-9);
        std::vector<double> tp;
        for (const auto& p : all) if (tp.empty() || std::abs(p.t - tp.back()) > pconf) tp.push_back(p.t);
        // assert monotone sorted (mitigation)
        for (size_t k = 1; k < tp.size(); ++k) if (tp[k] < tp[k-1]) tp[k] = tp[k-1];
        for (size_t k = 0; k + 1 < tp.size(); ++k)
            if (tp[k+1] - tp[k] >= pconf) s.segs.push_back({tp[k], tp[k+1]});
    }

    // ---- Phase 4a: a SINGLE co-refine pass so both operands agree on the section's split set
    // (the box full-circle is topologically split at the periodic-side arcs' vertices). ----
    co_refine_coincident_edges(tol);

    // Distance from p to a polyline (point-to-segment).
    auto pt_to_poly = [](const Point& p, const std::vector<Point>& pts) -> double {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pts.size(); ++j) {
            const Point& a = pts[j]; const Point& b = pts[j+1];
            double ex=b[0]-a[0], ey=b[1]-a[1], ez=b[2]-a[2], L2=ex*ex+ey*ey+ez*ez;
            double t = (L2>1e-30) ? ((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2 : 0.0;
            t = std::min(std::max(t,0.0),1.0);
            double cx=a[0]+t*ex, cy=a[1]+t*ey, cz=a[2]+t*ez;
            best = std::min(best, std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz)));
        }
        return best;
    };
    // Sample the exact section curve C over the sub-arc whose end PARAMETERS are ts,te and that
    // passes through Pm, oriented ts->te. point_at only (no split, unreliable on a closed rational
    // circle) -> a closed full circle yields a clean open arc. ts,te are already C-domain params
    // (snapped to the shared pave set), so the two operands land on IDENTICAL endpoints.
    auto sample_subarc = [&](const NurbsCurve& C, double ts, double te, const Point& Pm) -> NurbsCurve {
        auto [d0, d1] = C.domain(); double rng = d1 - d0;
        if (rng <= 1e-12) return NurbsCurve();
        bool closed = C.point_at(d0).distance(C.point_at(d1)) < tol;
        double tm = C.closest_parameter(Pm);
        const int N = 40;
        std::vector<Point> pts;
        if (closed) {
            auto nrm = [&](double t){ double x = std::fmod(t - d0, rng); if (x < 0) x += rng; return d0 + x; };
            ts = nrm(ts); te = nrm(te); tm = nrm(tm);
            double te_f = (te >= ts) ? te : te + rng;            // forward (increasing) target
            double tm_f = (tm >= ts) ? tm : tm + rng;
            double tend;
            if (tm_f <= te_f + 1e-12) tend = te_f;               // mid lies on the forward arc
            else tend = (te <= ts) ? te : te - rng;              // else go backward
            for (int k = 0; k <= N; ++k) pts.push_back(C.point_at(nrm(ts + (tend - ts) * k / N)));
        } else {
            for (int k = 0; k <= N; ++k) pts.push_back(C.point_at(ts + (te - ts) * k / N));
        }
        if (pts.size() < 2) return NurbsCurve();
        return NurbsCurve::create(false, 1, pts);
    };

    // ---- Phase 4b: re-fit each under-mated section edge to the exact shared sub-arc. The edge's
    // projected endpoint parameters are SNAPPED to the nearest canonical pave parameter (the
    // split-exactly-once set), so the two operands' arcs of a segment land on the IDENTICAL
    // [t_i,t_j] -> bit-identical geometry that sews into one shared edge. ----
    int ne = (int)m_topology_edges.size();
    for (int e = 0; e < ne; ++e) {
        BRepEdge& E = m_topology_edges[e];
        if ((int)E.trim_indices.size() >= 2) continue;            // already mated -> not a section edge
        int ci = E.curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        const NurbsCurve C = m_curves_3d[ci];
        if (!C.is_valid()) continue;
        auto [et0, et1] = C.domain();
        const int M = 16;
        std::vector<Point> es; std::array<double,6> ebb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= M; ++k) {
            Point p = C.point_at(et0 + (et1-et0)*k/M); es.push_back(p);
            for (int d = 0; d < 3; ++d) { ebb[d]=std::min(ebb[d],p[d]); ebb[d+3]=std::max(ebb[d+3],p[d]); }
        }
        // Match to the single exact section curve that contains the WHOLE edge (every sample on it).
        int best = -1;
        for (size_t si = 0; si < secs.size(); ++si) {
            const auto& B6 = secs[si].bb;
            if (ebb[0] > B6[3]+tol || B6[0] > ebb[3]+tol || ebb[1] > B6[4]+tol ||
                B6[1] > ebb[4]+tol || ebb[2] > B6[5]+tol || B6[2] > ebb[5]+tol) continue;
            bool on = true;
            for (const auto& p : es) if (pt_to_poly(p, secs[si].samp) > tol) { on = false; break; }
            if (on) { best = (int)si; break; }
        }
        if (best < 0) continue;
        Sec& S = secs[best];
        double ts = S.c3d.closest_parameter(es.front());
        double te = S.c3d.closest_parameter(es.back());
        // snap each endpoint param to the nearest canonical pave parameter, but ONLY if it lands
        // within the parametric snap window (else keep the edge's own projected param) -- a
        // full-circle edge whose ends sit at the seam stays a full circle (snap is a no-op).
        auto snap = [&](double t) -> double {
            double best_t = t, bd = 1e300;
            auto consider = [&](double pt){ double d = std::abs(pt - t); if (d < bd) { bd = d; best_t = pt; } };
            consider(S.t0); consider(S.t1);
            for (const auto& sg : S.segs) { consider(sg[0]); consider(sg[1]); }
            double win = std::max(resolution(S.c3d, t) * 4.0, (S.t1 - S.t0) * 1e-4);
            return (bd <= win) ? best_t : t;
        };
        ts = snap(ts); te = snap(te);
        NurbsCurve sub = sample_subarc(S.c3d, ts, te, es[M/2]);
        // guard: only substitute if the result still hugs the original edge (never worsen sewing).
        if (sub.is_valid()) {
            auto [s0, s1] = sub.domain();
            bool ok = true;
            for (int k = 0; k <= 8; ++k) {
                Point sp = sub.point_at(s0 + (s1-s0)*k/8);
                if (pt_to_poly(sp, es) > std::max(tol, 1e-9)) { ok = false; break; }
            }
            if (ok) m_curves_3d[ci] = sub;
        }
    }

    // ---- Phase 4c: merge the now-identical coincident arcs into one shared edge (two trims). ----
    sew_coincident_edges(tol);
}

void BRep::snap_section_edges(const std::vector<NurbsCurve>& sections, double tol) {
    if (sections.empty()) return;
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 5e-3;      // endpoint-on-section gate (~sew tolerance)
    double band = diag * 6e-2;              // mid-span containment band (lift divergence ceiling)
    auto p2pl = [](const Point& p, const std::vector<Point>& pl) {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pl.size(); ++j) {
            const Point& a = pl[j]; const Point& b = pl[j+1];
            double ex=b[0]-a[0], ey=b[1]-a[1], ez=b[2]-a[2], L2=ex*ex+ey*ey+ez*ez;
            double t=(L2>1e-30)?((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2:0.0;
            t=std::min(std::max(t,0.0),1.0);
            double cx=a[0]+t*ex, cy=a[1]+t*ey, cz=a[2]+t*ez;
            best=std::min(best,std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz)));
        }
        return best;
    };
    int snapped = 0;
    for (auto& E : m_topology_edges) {
        if ((int)E.trim_indices.size() != 1) continue;
        int ci = E.curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        const NurbsCurve C = m_curves_3d[ci];
        if (!C.is_valid()) continue;
        auto [t0, t1] = C.domain();
        const int M = 16;
        std::vector<Point> es; double len_e = 0.0;
        for (int k = 0; k <= M; ++k) es.push_back(C.point_at(t0 + (t1 - t0) * k / M));
        for (int k = 1; k <= M; ++k) len_e += es[k].distance(es[k-1]);
        if (len_e < tol) continue;                       // micro edge: sew's collapse handles it
        bool edge_closed = es.front().distance(es.back()) < tol;
        for (const NurbsCurve& S : sections) {
            if (!S.is_valid()) continue;
            auto [d0, d1] = S.domain(); double rng = d1 - d0;
            if (rng <= 1e-12) continue;
            bool s_closed = S.point_at(d0).distance(S.point_at(d1)) < tol;
            double s0 = S.closest_parameter(es.front());
            double s1 = S.closest_parameter(es.back());
            if (S.point_at(s0).distance(es.front()) > tol || S.point_at(s1).distance(es.back()) > tol) continue;
            // sample the sub-arc s0->s1 whose interior passes near the edge midpoint
            const int N = 40;
            std::vector<Point> sub;
            if (edge_closed && s_closed) {
                for (int k = 0; k <= N; ++k) sub.push_back(S.point_at(d0 + rng * k / N));   // full wrap
            } else if (s_closed) {
                auto nrm = [&](double t){ double x = std::fmod(t - d0, rng); if (x < 0) x += rng; return d0 + x; };
                double a = nrm(s0), b = nrm(s1), m = nrm(S.closest_parameter(es[M/2]));
                double b_f = (b >= a) ? b : b + rng;
                double m_f = (m >= a) ? m : m + rng;
                double tend = (m_f <= b_f + 1e-12) ? b_f : ((b <= a) ? b : b - rng);
                for (int k = 0; k <= N; ++k) sub.push_back(S.point_at(nrm(a + (tend - a) * k / N)));
            } else {
                for (int k = 0; k <= N; ++k) sub.push_back(S.point_at(s0 + (s1 - s0) * k / N));
            }
            double len_s = 0.0;
            for (size_t k = 1; k < sub.size(); ++k) len_s += sub[k].distance(sub[k-1]);
            // wrong arc of a loop / a face-border shortcut between two crossings: lengths differ
            if (std::abs(len_s - len_e) > 0.3 * std::max(len_e, tol)) continue;
            bool inband = true;
            for (const auto& q : es) if (p2pl(q, sub) > band) { inband = false; break; }
            if (!inband) continue;
            NurbsCurve nc = NurbsCurve::create(false, 1, sub);
            if (nc.is_valid()) { m_curves_3d[ci] = nc; ++snapped; }
            break;
        }
    }
    if (std::getenv("SESSION_NT_DBG"))
        std::printf("[SECSNAP] snapped=%d under-mated edges to exact section arcs\n", snapped);
}

void BRep::sew_coincident_edges(double tol) {
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    // Two imprinted intersection edges (a circle imprinted independently on A and B) are the
    // same edge but discretised as polylines with different phase/sampling (one may be an exact
    // rational circle, the other a degree-3 spline fit), so their vertices do NOT coincide
    // point-for-point. We measure a point-to-POLYLINE (segment) directed Hausdorff both ways: a
    // vertex of one polyline lands within the other's chord sagitta. The default tolerance is
    // feature-relative (diag*5e-3) so it comfortably absorbs that sagitta yet stays far below
    // the spacing of the kernel solids' distinct edges (O(diag) apart), avoiding false merges.
    if (tol <= 0.0) tol = diag * 5e-3;
    int ne = (int)m_topology_edges.size();
    // 16 samples sag ~0.03 at the tips of an elongated section arc (see co_refine NS note): the
    // measured Hausdorff then exceeds tol for truly-coincident arcs. Denser sampling tightens the
    // measurement only -- distinct edges stay O(diag) apart regardless of sample count.
    const int NS = 64;
    std::vector<std::vector<Point>> samp(ne);
    std::vector<std::array<double,6>> bbox(ne);  // xmn,ymn,zmn,xmx,ymx,zmx per edge
    // Only UNDER-mated edges (fewer than 2 trims) are sewing candidates: a shared intersection
    // curve is imprinted as a 1-trim edge on each side and the two halves must merge into one
    // 2-trim edge. Edges that are already 2-trim (the solids' own box/cyl/sphere edges) are
    // watertight and cannot legitimately coincide with another edge, so we skip sampling AND
    // comparing them -- turning the O(ne^2) sew into O(k^2) over the few intersection edges.
    auto is_candidate = [&](int ei) { return (int)m_topology_edges[ei].trim_indices.size() < 2; };
    for (int ei = 0; ei < ne; ++ei) {
        if (!is_candidate(ei)) continue;
        int ci = m_topology_edges[ei].curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        auto [t0, t1] = m_curves_3d[ci].domain();
        std::array<double,6> bb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= NS; ++k) {
            Point p = m_curves_3d[ci].point_at(t0 + (t1 - t0) * k / NS);
            samp[ei].push_back(p);
            for (int d = 0; d < 3; ++d) { bb[d] = std::min(bb[d], p[d]); bb[d+3] = std::max(bb[d+3], p[d]); }
        }
        bbox[ei] = bb;
    }
    // Cheap reject: edge AABBs (expanded by tol) must overlap to possibly coincide.
    auto bbox_far = [&](int i, int j) -> bool {
        const auto& a = bbox[i]; const auto& b = bbox[j];
        return a[0] > b[3]+tol || b[0] > a[3]+tol || a[1] > b[4]+tol || b[1] > a[4]+tol
            || a[2] > b[5]+tol || b[2] > a[5]+tol;
    };
    // Distance from point p to the polyline through pts (point-to-segment, not point-to-vertex).
    auto pt_to_polyline = [](const Point& p, const std::vector<Point>& pts) -> double {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pts.size(); ++j) {
            const Point& a = pts[j]; const Point& b = pts[j + 1];
            double ex = b[0]-a[0], ey = b[1]-a[1], ez = b[2]-a[2];
            double L2 = ex*ex + ey*ey + ez*ez;
            double t = (L2 > 1e-30) ? ((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2 : 0.0;
            t = std::min(std::max(t, 0.0), 1.0);
            double cx = a[0]+t*ex, cy = a[1]+t*ey, cz = a[2]+t*ez;
            double d = std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz));
            best = std::min(best, d);
        }
        return best;
    };
    // Directed point-to-polyline Hausdorff both ways, but EARLY-EXIT as soon as any sample
    // exceeds `tol` (the curves are then provably not coincident). Identical accept/reject to a
    // full max-Hausdorff < tol, but rejection is O(1) instead of O(NS^2) for the common case.
    // The midpoint is checked first because non-coincident edges that share an endpoint diverge
    // most in the middle, so it rejects fastest.
    auto coincident_within = [&](const std::vector<Point>& a, const std::vector<Point>& b) -> bool {
        if (a.size() < 2 || b.size() < 2) return false;
        if (pt_to_polyline(a[a.size()/2], b) > tol) return false;
        if (pt_to_polyline(b[b.size()/2], a) > tol) return false;
        for (const auto& p : a) if (pt_to_polyline(p, b) > tol) return false;
        for (const auto& p : b) if (pt_to_polyline(p, a) > tol) return false;
        return true;
    };
    // Micro-edge collapse (OCCT BOPTools_AlgoTools::IsMicroEdge): an UNDER-MATED edge far
    // shorter than the sew tolerance is below the watertightness resolution -- the typical
    // source is a surface-TANGENCY point where two osculating section polylines cross twice
    // within sampling noise (tor x tor: 3e-4 slivers at the 4 tangent nodes), leaving a stub
    // no mate ever reproduces. Drop the edge and its trims; the loop's resulting endpoint
    // gap (== stub length) is far inside the tolerance every consumer already absorbs.
    std::vector<bool> dead(ne, false);
    {
        double micro_tol = tol * 0.2;
        for (int ei = 0; ei < ne; ++ei) {
            if (samp[ei].empty()) continue;                  // not a candidate (>= 2 trims)
            double len = 0.0;
            for (size_t k = 1; k < samp[ei].size(); ++k) len += samp[ei][k].distance(samp[ei][k-1]);
            if (len >= micro_tol) continue;
            // A 3D-degenerate edge with REAL UV extent is a pole/seam run (a sphere pole line
            // spans the full u-domain while its 3D curve is a point) -- legitimate topology,
            // never a sliver. Protect by trim type AND by UV extent.
            bool protected_run = false;
            for (int ti = 0; ti < (int)m_trims.size() && !protected_run; ++ti) {
                if (m_trims[ti].edge_index != ei) continue;
                if (m_trims[ti].type == BRepTrimType::Singular || m_trims[ti].type == BRepTrimType::Seam)
                    { protected_run = true; break; }
                int c2 = m_trims[ti].curve_2d_index;
                int li = m_trims[ti].loop_index;
                int fi = (li >= 0 && li < (int)m_loops.size()) ? m_loops[li].face_index : -1;
                int si = (fi >= 0 && fi < (int)m_faces.size()) ? m_faces[fi].surface_index : -1;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size() || si < 0 || si >= (int)m_surfaces.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                if (!pc.is_valid()) continue;
                auto pd = pc.domain();
                Point u0 = pc.point_at(pd.first), u1 = pc.point_at(pd.second);
                auto du = m_surfaces[si].domain(0); auto dv = m_surfaces[si].domain(1);
                double span = std::max(du.second - du.first, dv.second - dv.first);
                if (u0.distance(u1) > span * 1e-2) protected_run = true;
            }
            if (protected_run) continue;
            dead[ei] = true;
            for (int ti = 0; ti < (int)m_trims.size(); ++ti) {
                if (m_trims[ti].edge_index != ei) continue;
                int li = m_trims[ti].loop_index;
                if (li >= 0 && li < (int)m_loops.size()) {
                    auto& tl = m_loops[li].trim_indices;
                    tl.erase(std::remove(tl.begin(), tl.end(), ti), tl.end());
                }
                m_trims[ti].edge_index = -1;
            }
            m_topology_edges[ei].trim_indices.clear();
        }
    }
    // Merge is capped at 2 trims per representative: at a non-manifold contact (two boxes
    // fused along a shared corner line) FOUR 1-trim copies coincide; merging all of them
    // makes one 4-trim edge (not solid). Pairing them 1+1 instead yields two touching
    // 2-trim edges -- two watertight shells, matching OCCT's two-solid compound.
    std::vector<int> rep(ne, -1);
    std::vector<int> reps;
    std::vector<int> rep_trims(ne, 0);
    for (int ei = 0; ei < ne; ++ei) {
        if (dead[ei]) continue;
        int own = (int)m_topology_edges[ei].trim_indices.size();
        if (samp[ei].empty()) { rep[ei] = ei; reps.push_back(ei); rep_trims[ei] += own; continue; }
        for (int r : reps)
            if (!samp[r].empty() && rep_trims[r] + own <= 2 && !bbox_far(ei, r) && coincident_within(samp[ei], samp[r])) { rep[ei] = r; break; }
        if (rep[ei] < 0) { rep[ei] = ei; reps.push_back(ei); }
        rep_trims[rep[ei]] += own;
    }
    // Second pass, endpoint-anchored: one operand's section copy can be fit-accurate while
    // the other's is a devtol-coarse lift, pushing their Hausdorff just past the tolerance.
    // For pairs whose ENDPOINTS already agree at the base tolerance, relax the body test:
    // genuinely distinct edges that share endpoints (e.g. the two halves of a section loop)
    // diverge by O(lens height), far beyond 2.5x tol.
    {
        std::vector<int> uses(ne, 0);
        for (const auto& t : m_trims)
            if (t.edge_index >= 0 && t.edge_index < ne) uses[rep[t.edge_index]]++;
        double tol2 = tol * 2.5;
        for (int ei = 0; ei < ne; ++ei) {
            if (samp[ei].empty() || rep[ei] != ei) continue;
            if (std::getenv("SESSION_SEW_DBG") && uses[ei] == 1)
                std::fprintf(stderr, "[P2] candidate ei=%d uses=%d\n", ei, uses[ei]);
            if (uses[ei] != 1) continue;
            const Point& a0 = samp[ei].front(); const Point& a1 = samp[ei].back();
            for (int r : reps) {
                if (r == ei || samp[r].empty() || uses[r] != 1 || rep[r] != r || bbox_far(ei, r)) continue;
                const Point& b0 = samp[r].front(); const Point& b1 = samp[r].back();
                bool em = (a0.distance(b0) < tol && a1.distance(b1) < tol)
                       || (a0.distance(b1) < tol && a1.distance(b0) < tol);
                if (std::getenv("SESSION_SEW_DBG"))
                    std::fprintf(stderr, "[P2] ei=%d r=%d em=%d d00=%.4f d11=%.4f d01=%.4f d10=%.4f\n",
                        ei, r, em?1:0, a0.distance(b0), a1.distance(b1), a0.distance(b1), a1.distance(b0));
                if (!em) continue;
                bool ok2 = true;
                if (pt_to_polyline(samp[ei][samp[ei].size()/2], samp[r]) > tol2) ok2 = false;
                if (ok2) for (const auto& q : samp[ei]) if (pt_to_polyline(q, samp[r]) > tol2) { ok2 = false; break; }
                if (ok2) for (const auto& q : samp[r]) if (pt_to_polyline(q, samp[ei]) > tol2) { ok2 = false; break; }
                if (ok2) { rep[ei] = r; break; }
            }
        }
    }
    // Third pass, mutual-best em-match: on grazing surface pairs (two near-identical solids
    // slightly offset) the two operands' lifts of the SAME section edge share endpoints but
    // diverge mid-span past tol2 (each polyline hugs its own surface where the surfaces run
    // nearly parallel). A plain tolerance raise would also merge the two HALVES of a section
    // loop (they share junction endpoints too) -- so pair only edges that are each other's
    // MUTUAL best match with clear separation from the runner-up: the true copy sits at
    // lift-degradation distance, a wrong half at lens height.
    {
        std::vector<int> uses(ne, 0);
        for (const auto& t : m_trims)
            if (t.edge_index >= 0 && t.edge_index < ne) uses[rep[t.edge_index]]++;
        double tol3 = tol * 8.0;
        auto haus = [&](int a, int b) -> double {
            double h = 0.0;
            for (const auto& q : samp[a]) { double d = pt_to_polyline(q, samp[b]); if (d > h) h = d; if (h > tol3) return h; }
            for (const auto& q : samp[b]) { double d = pt_to_polyline(q, samp[a]); if (d > h) h = d; if (h > tol3) return h; }
            return h;
        };
        std::vector<int> cand;
        for (int ei = 0; ei < ne; ++ei)
            if (!samp[ei].empty() && rep[ei] == ei && uses[ei] == 1) cand.push_back(ei);
        std::vector<int> best(ne, -1); std::vector<double> bd(ne, 1e300), bd2(ne, 1e300);
        for (size_t x = 0; x < cand.size(); ++x)
            for (size_t y = x + 1; y < cand.size(); ++y) {
                int a = cand[x], b = cand[y];
                if (bbox_far(a, b)) continue;
                const Point& a0 = samp[a].front(); const Point& a1 = samp[a].back();
                const Point& b0 = samp[b].front(); const Point& b1 = samp[b].back();
                bool em = (a0.distance(b0) < tol && a1.distance(b1) < tol)
                       || (a0.distance(b1) < tol && a1.distance(b0) < tol);
                if (!em) continue;
                double h = haus(a, b);
                if (h < bd[a]) { bd2[a] = bd[a]; bd[a] = h; best[a] = b; } else if (h < bd2[a]) bd2[a] = h;
                if (h < bd[b]) { bd2[b] = bd[b]; bd[b] = h; best[b] = a; } else if (h < bd2[b]) bd2[b] = h;
            }
        for (int a : cand) {
            int b = best[a];
            if (b < 0 || best[b] != a || a > b) continue;          // mutual best, handle once
            if (bd[a] > tol3) continue;                            // too far even for a copy
            if (bd[a] > bd2[a] * 0.6 || bd[a] > bd2[b] * 0.6) continue;  // no clear separation
            if (rep[a] != a || rep[b] != b) continue;
            if (std::getenv("SESSION_SEW_DBG"))
                std::fprintf(stderr, "[P3] merge a=%d b=%d h=%.4f next=%.4f\n", a, b, bd[a], std::min(bd2[a], bd2[b]));
            rep[b] = a;
        }
    }
    if (std::getenv("SESSION_SEW_DBG")) {
        for (int ei = 0; ei < ne; ++ei) {
            if (samp[ei].empty() || rep[ei] != ei) continue;
            for (int r : reps) {
                if (r == ei || samp[r].empty() || bbox_far(ei, r)) continue;
                double h = 0.0;
                for (const auto& q : samp[ei]) h = std::max(h, pt_to_polyline(q, samp[r]));
                for (const auto& q : samp[r]) h = std::max(h, pt_to_polyline(q, samp[ei]));
                if (h < tol * 20.0) {
                    int u1 = 0, u2 = 0;
                    for (const auto& t : m_trims) {
                        if (t.edge_index >= 0 && t.edge_index < ne && rep[t.edge_index] == ei) u1++;
                        if (t.edge_index >= 0 && t.edge_index < ne && rep[t.edge_index] == r) u2++;
                    }
                    std::fprintf(stderr, "[SEWMISS] e=%d(u=%d) r=%d(u=%d) haus=%.5f tol=%.5f\n", ei, u1, r, u2, h, tol);
                    for (int ti2 = 0; ti2 < (int)m_trims.size(); ++ti2) {
                        int oe2 = m_trims[ti2].edge_index;
                        if (oe2 < 0 || oe2 >= ne) continue;
                        if (rep[oe2] != ei && rep[oe2] != r) continue;
                        int li2 = m_trims[ti2].loop_index;
                        int fi2 = (li2 >= 0 && li2 < (int)m_loops.size()) ? m_loops[li2].face_index : -1;
                        int si2 = (fi2 >= 0 && fi2 < (int)m_faces.size()) ? m_faces[fi2].surface_index : -1;
                        int pl2 = (si2 >= 0 && si2 < (int)m_surfaces.size() && m_surfaces[si2].is_planar(nullptr, 1e-6)) ? 1 : 0;
                        std::fprintf(stderr, "  [SEWM3] rep=%d trim=%d face=%d srf=%d planar=%d\n",
                            rep[oe2], ti2, fi2, si2, pl2);
                    }
                    for (int oe = 0; oe < ne; ++oe)
                        if (oe != r && oe != ei && (rep[oe] == rep[r] || rep[oe] == ei))
                            std::fprintf(stderr, "  [SEWM2] edge %d merged into rep %d\n", oe, rep[oe]);
                }
            }
        }
    }
    // POST-MERGE ORPHAN COLLAPSE (OCCT ShapeFix FixSmall at the correct pipeline position):
    // an edge still unmated after ALL merge passes (P1/P2/P3) whose 3D extent is below the
    // weld scale is a redundant arrangement copy (e.g. the odd third section copy at a
    // >2-face junction) -- no mate exists at ANY tolerance, so keeping it can only leave a
    // naked edge. Collapse it: detach its trims from their loops and bridge the sub-weld 2D
    // gap by snapping the next trim's entry CV onto the previous trim's exit point, so the
    // wire stays exactly contiguous. The earlier micro pass cannot do this: it runs BEFORE
    // the merges and cannot know which short edges would have found a mate (order-dependent
    // collateral collapses were measured: y30 naked 11->12).
    if (!std::getenv("SESSION_NO_ORPHAN")) {
        // Threshold = the SEW tolerance itself, not a fraction of it. Two reasons. (1) An
        // edge shorter than the distance at which we already declare curves coincident is
        // by construction indistinguishable from a vertex -- this is OCCT's valid-range
        // rule (a pave block whose span lies inside its vertices' tolerance spheres is
        // refused and the vertices fused), and only edges that survived EVERY merge pass
        // unmated are eligible here, so keeping one can do nothing but leave a naked edge.
        // (2) A fraction of tol made the rule OPERATION-DEPENDENT: tol derives from the
        // RESULT bbox, so the identical 0.021 junction copy in the chairs z30x20 joint was
        // collapsed for cut (tol 0.0708) and fuse (0.0897) but survived for common, whose
        // result is a much smaller shape -- the same physical sliver must not live or die
        // according to which boolean produced it.
        double orph_tol = tol;
        std::vector<int> uses(ne, 0);
        for (const auto& t : m_trims)
            if (t.edge_index >= 0 && t.edge_index < ne) uses[rep[t.edge_index]]++;
        int n_orph = 0;
        for (int r : reps) {
            if (dead[r] || samp[r].empty() || uses[r] > 1) continue;
            double len = 0.0;
            for (size_t k = 0; k + 1 < samp[r].size(); ++k) len += samp[r][k].distance(samp[r][k+1]);
            if (len >= orph_tol) continue;
            bool prot = false;
            for (int ti = 0; ti < (int)m_trims.size() && !prot; ++ti) {
                const auto& T = m_trims[ti];
                if (T.edge_index < 0 || T.edge_index >= ne || rep[T.edge_index] != r) continue;
                if (T.type == BRepTrimType::Singular || T.type == BRepTrimType::Seam) prot = true;
                // pole/seam runs collapse to a 3D point but sweep a long pcurve: removing
                // the trim would tear the loop's UV chain (sphere pole edges are 1-trim,
                // 3D-degenerate, UV-long) -- same guard as the pre-merge micro pass.
                int c2 = T.curve_2d_index;
                int li2 = T.loop_index;
                int fi2 = (li2 >= 0 && li2 < (int)m_loops.size()) ? m_loops[li2].face_index : -1;
                int si2 = (fi2 >= 0 && fi2 < (int)m_faces.size()) ? m_faces[fi2].surface_index : -1;
                if (c2 >= 0 && c2 < (int)m_curves_2d.size() && si2 >= 0 && si2 < (int)m_surfaces.size()) {
                    const NurbsCurve& pcv = m_curves_2d[c2];
                    auto dc2 = pcv.domain();
                    Point u0 = pcv.point_at(dc2.first), u1 = pcv.point_at(dc2.second);
                    auto du2 = m_surfaces[si2].domain(0); auto dv2 = m_surfaces[si2].domain(1);
                    double span = std::max(du2.second - du2.first, dv2.second - dv2.first);
                    if (u0.distance(u1) > span * 1e-2) prot = true;
                }
            }
            if (prot) continue;
            for (int ti = 0; ti < (int)m_trims.size(); ++ti) {
                auto& T = m_trims[ti];
                if (T.edge_index < 0 || T.edge_index >= ne || rep[T.edge_index] != r) continue;
                int li = T.loop_index;
                if (li >= 0 && li < (int)m_loops.size()) {
                    auto& tl = m_loops[li].trim_indices;
                    auto itl = std::find(tl.begin(), tl.end(), ti);
                    if (itl != tl.end()) {
                        int pos = (int)(itl - tl.begin());
                        tl.erase(itl);
                        int nt2 = (int)tl.size();
                        if (nt2 >= 2) {
                            // bridge: prev trim's exit point -> next trim's entry CV
                            int tp = tl[(pos - 1 + nt2) % nt2], tn = tl[pos % nt2];
                            if (tp >= 0 && tn >= 0 && tp < (int)m_trims.size() && tn < (int)m_trims.size()) {
                                int cp = m_trims[tp].curve_2d_index, cn2 = m_trims[tn].curve_2d_index;
                                if (cp >= 0 && cn2 >= 0 && cp < (int)m_curves_2d.size() && cn2 < (int)m_curves_2d.size()) {
                                    const NurbsCurve& PC = m_curves_2d[cp];
                                    auto dp2 = PC.domain();
                                    Point exit_p = PC.point_at(m_trims[tp].reversed ? dp2.first : dp2.second);
                                    NurbsCurve& NC = m_curves_2d[cn2];
                                    int cvi = m_trims[tn].reversed ? NC.cv_count() - 1 : 0;
                                    double xw, yw, zw, ww;
                                    if (NC.get_cv_4d(cvi, xw, yw, zw, ww))
                                        NC.set_cv_4d(cvi, exit_p[0] * ww, exit_p[1] * ww, zw, ww);
                                }
                            }
                        }
                    }
                }
                T.edge_index = -1;
            }
            m_topology_edges[r].trim_indices.clear();
            dead[r] = true;
            ++n_orph;
            if (std::getenv("SESSION_SEW_DBG"))
                std::fprintf(stderr, "[ORPHAN] collapsed rep=%d len=%.5f\n", r, len);
        }
        if (n_orph && std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[ORPHAN] total collapsed: %d (tol %.5f)\n", n_orph, orph_tol);
    }
    // Geometry rep upgrade: within a merged group keep the DENSEST member's 3D curve (a pullback
    // lift at n~2000 CVs beats a devtol section fit at ~40 CVs by an order of magnitude in
    // on-curve accuracy). Rational (exact conic) members always win over polylines. Only the
    // curve geometry moves -- the rep edge's index, vertices and trims are untouched.
    for (int r : reps) {
        if (samp[r].empty()) continue;
        int rci = m_topology_edges[r].curve_3d_index;
        if (rci < 0 || rci >= (int)m_curves_3d.size()) continue;
        int best_ci = rci;
        auto better = [&](int a, int b) {   // is curve a better than curve b?
            const NurbsCurve& A = m_curves_3d[a]; const NurbsCurve& B = m_curves_3d[b];
            if (A.is_rational() != B.is_rational()) return A.is_rational();
            return A.cv_count() > B.cv_count();
        };
        for (int e = 0; e < ne; ++e) {
            if (rep[e] != r || e == r) continue;
            int ci = m_topology_edges[e].curve_3d_index;
            if (ci < 0 || ci >= (int)m_curves_3d.size() || !m_curves_3d[ci].is_valid()) continue;
            if (better(ci, best_ci)) best_ci = ci;
        }
        if (best_ci != rci) {
            NurbsCurve nc = m_curves_3d[best_ci];
            const NurbsCurve& rc = m_curves_3d[rci];
            if (nc.point_at_start().distance(rc.point_at_start())
              > nc.point_at_end().distance(rc.point_at_start())) nc.reverse();
            m_curves_3d[rci] = nc;
        }
    }
    std::map<int, int> old2new;
    std::vector<BRepEdge> newedges;
    for (int r : reps) {
        if (dead[r]) continue;   // orphan-collapsed reps: physically dropped (0-trim zombies break is_solid)
        old2new[r] = (int)newedges.size();
        BRepEdge e = m_topology_edges[r];
        e.trim_indices.clear();
        newedges.push_back(e);
    }
    for (int ti = 0; ti < (int)m_trims.size(); ++ti) {
        int oe = m_trims[ti].edge_index;
        if (oe < 0 || oe >= ne) continue;
        auto itN = old2new.find(rep[oe]);
        if (itN == old2new.end()) { m_trims[ti].edge_index = -1; continue; }   // dead group straggler
        int ni = itN->second;
        m_trims[ti].edge_index = ni;
        newedges[ni].trim_indices.push_back(ti);
    }
    m_topology_edges = std::move(newedges);
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        int sv = m_topology_edges[ei].start_vertex;
        int ev = m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei);
    }
}

void BRep::sameparameter_planar_pcurves() {
    // OCCT SameParameter-lite. After sew, an A<->B section edge is ONE edge with ONE 3D curve,
    // but its two trims still integrate their OWN pcurve copies, which differ at the section-fit
    // tolerance (~2e-4): the volume error is FIRST order in that mismatch (each face's flux
    // drifts independently; box x tor common failed at 1.8e-6 rel from exactly this). For the
    // PLANAR side of a mixed planar/curved section edge the fix is exact and cheap: affine-project
    // the shared 3D curve into the plane's UV -- both faces then read identical boundary geometry.
    for (auto& E : m_topology_edges) {
        if ((int)E.trim_indices.size() != 2) continue;
        int tp = -1, tq = -1, sp = -1, sq = -1;
        for (int ti : E.trim_indices) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int li = m_trims[ti].loop_index; if (li < 0 || li >= (int)m_loops.size()) continue;
            int fi = m_loops[li].face_index; if (fi < 0 || fi >= (int)m_faces.size()) continue;
            int si = m_faces[fi].surface_index; if (si < 0 || si >= (int)m_surfaces.size()) continue;
            if (tp < 0) { tp = ti; sp = si; } else { tq = ti; sq = si; }
        }
        if (tp < 0 || tq < 0 || sp == sq) continue;
        bool p_planar = m_surfaces[sp].is_planar(nullptr, 1e-6);
        bool q_planar = m_surfaces[sq].is_planar(nullptr, 1e-6);
        if (p_planar == q_planar) continue;                  // need exactly one planar side
        int ti  = p_planar ? tp : tq;                        // planar trim: pcurve to rebuild
        int tic = p_planar ? tq : tp;                        // curved trim: geometry source
        const NurbsSurface& S  = m_surfaces[p_planar ? sp : sq];
        const NurbsSurface& Sc = m_surfaces[p_planar ? sq : sp];
        int c2 = m_trims[ti].curve_2d_index;
        int c2c = m_trims[tic].curve_2d_index;
        if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
        if (c2c < 0 || c2c >= (int)m_curves_2d.size()) continue;
        const NurbsCurve& pc = m_curves_2d[c2];
        const NurbsCurve& pcc = m_curves_2d[c2c];
        // Geometry source = the CURVED trim's pcurve lifted through ITS surface at its own
        // vertices -- byte-identical to what the curved face's boundary-flux integration walks.
        // Dense deg-1 arrangement polylines only: a 2-CV uv line (cylinder cap circles) or a
        // rational pcurve marks an exact-conic edge whose planar arc is already exact.
        if (!pc.is_valid() || !pcc.is_valid()) continue;
        if (pcc.degree() != 1 || pcc.is_rational() || pcc.cv_count() < 8) continue;
        std::vector<Point> gpts;
        gpts.reserve(pcc.cv_count());
        for (int i = 0; i < pcc.cv_count(); ++i) {
            Point uv = pcc.get_cv(i);
            gpts.push_back(Sc.point_at(uv[0], uv[1]));
        }
        // the lift must actually lie on the planar face (sew-tol scale), else this is not a
        // clean mixed section edge (e.g. a seam-adjacent sliver) -- leave it alone.
        {
            Vector pn = S.normal_at(0.5*(S.domain(0).first+S.domain(0).second),
                                    0.5*(S.domain(1).first+S.domain(1).second));
            Point p0 = S.point_at(S.domain(0).first, S.domain(1).first);
            double dmax = 0.0;
            for (const auto& g : gpts)
                dmax = std::max(dmax, std::abs((g[0]-p0[0])*pn[0]+(g[1]-p0[1])*pn[1]+(g[2]-p0[2])*pn[2]));
            double sscale = S.point_at(S.domain(0).first, S.domain(1).first)
                              .distance(S.point_at(S.domain(0).second, S.domain(1).second));
            if (dmax > std::max(sscale, 1.0) * 5e-3) continue;
        }
        // affine deg-1x1 patch check + inverse frame (same construction as co_refine's splice)
        auto [su0, su1] = S.domain(0); auto [sv0, sv1] = S.domain(1);
        Point O3 = S.point_at(su0, sv0);
        Point PU = S.point_at(su1, sv0), PV = S.point_at(su0, sv1), PW = S.point_at(su1, sv1);
        double dux=(PU[0]-O3[0])/(su1-su0), duy=(PU[1]-O3[1])/(su1-su0), duz=(PU[2]-O3[2])/(su1-su0);
        double dvx=(PV[0]-O3[0])/(sv1-sv0), dvy=(PV[1]-O3[1])/(sv1-sv0), dvz=(PV[2]-O3[2])/(sv1-sv0);
        double guu=dux*dux+duy*duy+duz*duz, guv=dux*dvx+duy*dvy+duz*dvz, gvv=dvx*dvx+dvy*dvy+dvz*dvz;
        double gdet = guu*gvv - guv*guv;
        double psz = O3.distance(PW);
        bool affine = S.degree(0)==1 && S.degree(1)==1 && gdet > 1e-20
            && std::abs(PW[0]-PU[0]-PV[0]+O3[0]) + std::abs(PW[1]-PU[1]-PV[1]+O3[1])
             + std::abs(PW[2]-PU[2]-PV[2]+O3[2]) < psz*1e-9 + 1e-12;
        if (!affine) continue;
        auto to_uv = [&](const Point& P) -> Point {
            double wx=P[0]-O3[0], wy=P[1]-O3[1], wz=P[2]-O3[2];
            double bu=wx*dux+wy*duy+wz*duz, bv=wx*dvx+wy*dvy+wz*dvz;
            return Point(su0 + (bu*gvv - bv*guv)/gdet, sv0 + (guu*bv - guv*bu)/gdet, 0.0);
        };
        std::vector<Point> uvs;
        uvs.reserve(gpts.size());
        for (const auto& g : gpts) uvs.push_back(to_uv(g));
        // direction: the replacement must traverse like the OLD pcurve so trim.reversed stays valid
        auto pd = pc.domain();
        Point old_s = pc.point_at(pd.first), old_e = pc.point_at(pd.second);
        double d2s = old_s.distance(uvs.front()), d2e = old_s.distance(uvs.back());
        if (d2e < d2s) std::reverse(uvs.begin(), uvs.end());
        // sanity: endpoints must land where the old pcurve's did (sew-tol scale in UV)
        double span_uv = std::max(su1-su0, sv1-sv0);
        if (uvs.front().distance(old_s) > span_uv*2e-2 || uvs.back().distance(old_e) > span_uv*2e-2) continue;
        NurbsCurve npc = NurbsCurve::create(false, 1, uvs);
        if (npc.is_valid()) m_curves_2d[c2] = npc;
    }
}

void BRep::merge_coplanar_faces(double tolerance) {
    int nf = (int)m_faces.size();
    if (nf < 2) return;
    double diag = 0.0;
    {
        double mn[3] = {1e300,1e300,1e300}, mx[3] = {-1e300,-1e300,-1e300};
        for (const auto& p : m_vertices)
            for (int k = 0; k < 3; ++k) { mn[k] = std::min(mn[k], p[k]); mx[k] = std::max(mx[k], p[k]); }
        diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0]) + (mx[1]-mn[1])*(mx[1]-mn[1])
                       + (mx[2]-mn[2])*(mx[2]-mn[2]));
    }
    if (!(diag > 0)) return;
    double tol = tolerance > 0 ? tolerance : diag * 1e-6;
    // planar faces: midpoint + unit normal, 3x3 planarity sample
    std::vector<char> planar(nf, 0);
    std::vector<Point> porg(nf, Point(0,0,0));
    std::vector<Vector> pnrm(nf, Vector(0,0,1));
    for (int fi = 0; fi < nf; ++fi) {
        int si = m_faces[fi].surface_index;
        if (si < 0 || si >= (int)m_surfaces.size()) continue;
        const NurbsSurface& S = m_surfaces[si];
        auto [u0,u1] = S.domain(0); auto [v0,v1] = S.domain(1);
        Point O = S.point_at(0.5*(u0+u1), 0.5*(v0+v1));
        Vector N = S.normal_at(0.5*(u0+u1), 0.5*(v0+v1));
        double nl = N.magnitude();
        if (nl < 1e-12) continue;
        N = Vector(N[0]/nl, N[1]/nl, N[2]/nl);
        bool ok = true;
        for (int a = 0; a <= 2 && ok; ++a)
            for (int b = 0; b <= 2 && ok; ++b) {
                Point q = S.point_at(u0 + (u1-u0)*a/2.0, v0 + (v1-v0)*b/2.0);
                if (std::abs((q[0]-O[0])*N[0]+(q[1]-O[1])*N[1]+(q[2]-O[2])*N[2]) > tol) ok = false;
            }
        if (!ok) continue;
        planar[fi] = 1; porg[fi] = O; pnrm[fi] = N;
    }
    auto trim_face = [&](int ti) -> int {
        if (ti < 0 || ti >= (int)m_trims.size()) return -1;
        int li = m_trims[ti].loop_index;
        if (li < 0 || li >= (int)m_loops.size()) return -1;
        return m_loops[li].face_index;
    };
    auto coplanar = [&](int fa, int fb) {
        if (fa < 0 || fb < 0 || !planar[fa] || !planar[fb]) return false;
        const Vector& na = pnrm[fa]; const Vector& nb = pnrm[fb];
        double cx = na[1]*nb[2]-na[2]*nb[1], cy = na[2]*nb[0]-na[0]*nb[2], cz = na[0]*nb[1]-na[1]*nb[0];
        if (std::sqrt(cx*cx+cy*cy+cz*cz) > 1e-6) return false;
        const Point& oa = porg[fa]; const Point& ob = porg[fb];
        return std::abs((ob[0]-oa[0])*na[0]+(ob[1]-oa[1])*na[1]+(ob[2]-oa[2])*na[2]) <= tol;
    };
    std::vector<int> uf(nf);
    for (int i = 0; i < nf; ++i) uf[i] = i;
    std::function<int(int)> find = [&](int x) { while (uf[x] != x) x = uf[x] = uf[uf[x]]; return x; };
    for (const auto& E : m_topology_edges) {
        if ((int)E.trim_indices.size() != 2) continue;
        int fa = trim_face(E.trim_indices[0]), fb = trim_face(E.trim_indices[1]);
        if (fa < 0 || fb < 0 || fa == fb || !coplanar(fa, fb)) continue;
        uf[find(fa)] = find(fb);
    }
    std::map<int, std::vector<int>> groups;
    for (int fi = 0; fi < nf; ++fi) groups[find(fi)].push_back(fi);
    for (auto it = groups.begin(); it != groups.end();) {
        if ((int)it->second.size() < 2) it = groups.erase(it); else ++it;
    }
    if (groups.empty()) return;
    std::vector<char> in_group(nf, 0);
    for (auto& [r, fl] : groups) for (int fi : fl) in_group[fi] = 1;
    const double tolc = 5e-3;
    auto d2 = [](const Point& a, const Point& b) {
        double dx = a[0]-b[0], dy = a[1]-b[1]; return std::sqrt(dx*dx + dy*dy);
    };
    // One merged-face PLAN per group, built WITHOUT committing geometry, so an infeasible group
    // can be dropped (faces kept split, shared edges preserved) before any edge is removed. Each
    // surviving trim keeps its EXACT pcurve remapped into the merged chart when its source patch
    // is affine (every planar face we make is a parallelogram): exact remap keeps circle holes as
    // circles and polygon corners as corners -- blanket 256-sample resampling degraded conics to
    // inscribed 256-gons (~1e-4 area) and cut corners (sag ~2e-3, which makes Rhino drop trims).
    struct PMT { int edge; BRepTrimType type; std::vector<Point> ab; NurbsCurve pc; };
    struct Plan {
        bool ok = false; int g0 = -1;
        Point O; Vector U{1,0,0}, V{0,1,0};
        double amn = 0, ea = 1, bmn = 0, eb = 1;
        std::vector<PMT> mts;
        std::vector<std::vector<std::pair<int,bool>>> chains;
        int outer = 0;
    };
    auto build_plan = [&](const std::vector<int>& fl) -> Plan {
        Plan P; P.g0 = fl.front();
        auto in_g = [&](int fi) { return std::find(fl.begin(), fl.end(), fi) != fl.end(); };
        auto edge_internal_to_group = [&](int ei) {
            if (ei < 0 || ei >= (int)m_topology_edges.size()) return false;
            const BRepEdge& E = m_topology_edges[ei];
            if ((int)E.trim_indices.size() != 2) return false;
            int fa = trim_face(E.trim_indices[0]), fb = trim_face(E.trim_indices[1]);
            return fa >= 0 && fb >= 0 && fa != fb && in_g(fa) && in_g(fb);
        };
        int g0 = P.g0;
        P.O = porg[g0]; Vector N = pnrm[g0];
        const NurbsSurface& S0 = m_surfaces[m_faces[g0].surface_index];
        auto [gu0, gu1] = S0.domain(0); auto [gv0, gv1] = S0.domain(1);
        Point A0 = S0.point_at(gu0, 0.5*(gv0+gv1)), A1 = S0.point_at(gu1, 0.5*(gv0+gv1));
        Vector U(A1[0]-A0[0], A1[1]-A0[1], A1[2]-A0[2]);
        double ul = U.magnitude();
        if (ul < 1e-12) { U = std::abs(N[0]) < 0.9 ? Vector(1,0,0) : Vector(0,1,0); ul = 1.0; }
        U = Vector(U[0]/ul, U[1]/ul, U[2]/ul);
        Vector V(N[1]*U[2]-N[2]*U[1], N[2]*U[0]-N[0]*U[2], N[0]*U[1]-N[1]*U[0]);
        P.U = U; P.V = V;
        const Point& O = P.O;
        double amn = 1e300, amx = -1e300, bmn = 1e300, bmx = -1e300;
        struct Src { int sfi; int c2; };
        std::vector<Src> src;
        for (int fi : fl) {
            int sfi = m_faces[fi].surface_index;
            const NurbsSurface& Sf = m_surfaces[sfi];
            for (int li : m_faces[fi].loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                for (int ti : m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)m_trims.size()) continue;
                    const BRepTrim& T = m_trims[ti];
                    if (edge_internal_to_group(T.edge_index)) continue;
                    if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& pc = m_curves_2d[T.curve_2d_index];
                    if (!pc.is_valid()) continue;
                    auto pd = pc.domain();
                    int n = (pc.cv_count() == 2 && pc.degree() == 1) ? 1 : 256;
                    PMT mt; mt.edge = T.edge_index; mt.type = T.type;
                    mt.ab.reserve(n + 1);
                    for (int i = 0; i <= n; ++i) {
                        Point uv = pc.point_at(pd.first + (pd.second - pd.first) * i / n);
                        Point q = Sf.point_at(uv[0], uv[1]);
                        double a = (q[0]-O[0])*U[0] + (q[1]-O[1])*U[1] + (q[2]-O[2])*U[2];
                        double b = (q[0]-O[0])*V[0] + (q[1]-O[1])*V[1] + (q[2]-O[2])*V[2];
                        amn = std::min(amn, a); amx = std::max(amx, a);
                        bmn = std::min(bmn, b); bmx = std::max(bmx, b);
                        mt.ab.push_back(Point(a, b, 0.0));
                    }
                    P.mts.push_back(std::move(mt));
                    src.push_back({sfi, T.curve_2d_index});
                    // Coplanar merge is display-only; a group with a pathological number of
                    // imprinted micro-segment trims (box x tor fuse imprints ~1e5) blows this
                    // O(mts^2)/O(mts*256) planner up to minutes and gigabytes. Cap it here ->
                    // the group is kept split, which is geometrically correct (P.ok stays false;
                    // only presentation differs). Legit primitive merges have a few dozen trims.
                    if ((int)P.mts.size() > 5000) return P;
                }
            }
        }
        if (P.mts.empty()) return P;
        double ea = std::max(amx - amn, tol), eb = std::max(bmx - bmn, tol);
        double ma = ea * 0.02, mb = eb * 0.02;
        amn -= ma; amx += ma; bmn -= mb; bmx += mb; ea = amx - amn; eb = bmx - bmn;
        P.amn = amn; P.ea = ea; P.bmn = bmn; P.eb = eb;
        auto to_norm = [&](const Point& q) {
            double a = (q[0]-O[0])*U[0] + (q[1]-O[1])*U[1] + (q[2]-O[2])*U[2];
            double b = (q[0]-O[0])*V[0] + (q[1]-O[1])*V[1] + (q[2]-O[2])*V[2];
            return Point((a - amn) / ea, (b - bmn) / eb, 0.0);
        };
        for (auto& mt : P.mts)
            for (auto& p : mt.ab) p = Point((p[0]-amn)/ea, (p[1]-bmn)/eb, 0.0);
        for (size_t k = 0; k < P.mts.size(); ++k) {
            const NurbsSurface& Sf = m_surfaces[src[k].sfi];
            const NurbsCurve& pc = m_curves_2d[src[k].c2];
            auto [su0, su1] = Sf.domain(0); auto [sv0, sv1] = Sf.domain(1);
            Point p0 = to_norm(Sf.point_at(su0, sv0)), pu = to_norm(Sf.point_at(su1, sv0));
            Point pv = to_norm(Sf.point_at(su0, sv1)), pw = to_norm(Sf.point_at(su1, sv1));
            double aff = std::abs(pw[0] - (pu[0] + pv[0] - p0[0]))
                       + std::abs(pw[1] - (pu[1] + pv[1] - p0[1]));
            bool ok_remap = aff < 1e-9 && (su1 - su0) != 0.0 && (sv1 - sv0) != 0.0;
            if (ok_remap) {
                NurbsCurve rc = pc;   // preserves degree / knots / rationality
                for (int i = 0; i < rc.cv_count(); ++i) {
                    double cx, cy, cz, cw;
                    if (!rc.get_cv_4d(i, cx, cy, cz, cw)) { ok_remap = false; break; }
                    double fu = (cx - su0) / (su1 - su0), fv = (cy - sv0) / (sv1 - sv0);
                    double nu = p0[0] + fu*(pu[0]-p0[0]) + fv*(pv[0]-p0[0]);
                    double nv = p0[1] + fu*(pu[1]-p0[1]) + fv*(pv[1]-p0[1]);
                    rc.set_cv_4d(i, nu, nv, 0.0, cw);
                }
                if (ok_remap && rc.is_valid()) { P.mts[k].pc = rc; continue; }
            }
            P.mts[k].pc = NurbsCurve::create(false, 1, P.mts[k].ab);   // fallback: deg-1 polyline
        }
        // Chain into loops by UV endpoint proximity. tolc is the junction weld gap; it must NOT
        // double as a closure test (the old `while(d2(tail,head)>tolc)` let any trim SHORTER than
        // tolc self-close into a bogus single-trim loop, and closed the outline early across a
        // short boundary edge, exiling it into a spurious hole -- both confirmed defects). A trim
        // is a genuine self-closed loop (circle hole) only if its endpoints coincide AND it spans
        // real arc length; otherwise it is an open segment that must chain to a neighbor.
        auto& mts = P.mts;
        std::vector<char> selfclosed(mts.size(), 0);
        for (size_t k = 0; k < mts.size(); ++k) {
            double L = 0.0;
            for (size_t i = 1; i < mts[k].ab.size(); ++i) L += d2(mts[k].ab[i-1], mts[k].ab[i]);
            selfclosed[k] = (d2(mts[k].ab.front(), mts[k].ab.back()) < tolc && L > 8.0 * tolc) ? 1 : 0;
        }
        std::vector<char> used(mts.size(), 0);
        bool merge_ok = true;
        for (size_t s = 0; s < mts.size(); ++s)
            if (selfclosed[s]) { used[s] = 1; P.chains.push_back({{(int)s, true}}); }
        for (size_t s = 0; s < mts.size() && merge_ok; ++s) {
            if (used[s]) continue;
            used[s] = 1;
            std::vector<std::pair<int,bool>> chain{{(int)s, true}};
            Point head = mts[s].ab.front();
            Point tail = mts[s].ab.back();
            for (;;) {
                int best = -1; bool fwd = true; double bd = 1e300;
                for (size_t j = 0; j < mts.size(); ++j) {
                    if (used[j]) continue;
                    double ds = d2(mts[j].ab.front(), tail), de = d2(mts[j].ab.back(), tail);
                    if (ds < bd) { bd = ds; best = (int)j; fwd = true; }
                    if (de < bd) { bd = de; best = (int)j; fwd = false; }
                }
                double ghead = d2(tail, head);
                bool can_ext = (best >= 0 && bd <= tolc);
                if (can_ext && bd <= ghead) {
                    used[best] = 1; chain.push_back({best, fwd});
                    tail = fwd ? mts[best].ab.back() : mts[best].ab.front();
                } else if (ghead <= tolc) {
                    break;
                } else if (can_ext) {
                    used[best] = 1; chain.push_back({best, fwd});
                    tail = fwd ? mts[best].ab.back() : mts[best].ab.front();
                } else {
                    merge_ok = false; break;
                }
            }
            if (merge_ok && d2(tail, head) > tolc) merge_ok = false;
            if (merge_ok) P.chains.push_back(std::move(chain));
        }
        for (size_t k = 0; k < mts.size() && merge_ok; ++k) if (!used[k]) merge_ok = false;
        if (!merge_ok) return P;
        double best_area = -1.0;   // largest |shoelace| chain is the outer loop
        for (size_t c = 0; c < P.chains.size(); ++c) {
            std::vector<const Point*> poly;
            for (auto& [idx, fwd] : P.chains[c]) {
                const auto& ab = mts[idx].ab;
                if (fwd) for (const auto& p : ab) poly.push_back(&p);
                else for (auto rit = ab.rbegin(); rit != ab.rend(); ++rit) poly.push_back(&*rit);
            }
            double A2 = 0.0;
            for (size_t i = 0; i < poly.size(); ++i) {
                const Point& p = *poly[i]; const Point& q = *poly[(i+1) % poly.size()];
                A2 += p[0]*q[1] - q[0]*p[1];
            }
            double area = std::abs(0.5 * A2);
            if (area > best_area) { best_area = area; P.outer = (int)c; }
        }
        P.ok = true;
        return P;
    };
    // Phase 1: plan every group. Infeasible groups downgrade to ungrouped (kept split).
    std::map<int, Plan> plan_of;
    for (auto& [root, fl] : groups) {
        Plan P = build_plan(fl);
        if (std::getenv("SESSION_MERGE_DBG"))
            std::fprintf(stderr, "[MERGEDBG] group root=%d faces=%zu mts=%zu chains=%zu ok=%d\n",
                         root, fl.size(), P.mts.size(), P.chains.size(), P.ok ? 1 : 0);
        if (!P.ok) { for (int fi : fl) in_group[fi] = 0; continue; }
        plan_of.emplace(root, std::move(P));
    }
    // Phase 2: internal edges are shared by two faces of the SAME feasible group only.
    std::vector<char> edge_internal(m_topology_edges.size(), 0);
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        const BRepEdge& E = m_topology_edges[ei];
        if ((int)E.trim_indices.size() != 2) continue;
        int fa = trim_face(E.trim_indices[0]), fb = trim_face(E.trim_indices[1]);
        if (fa >= 0 && fb >= 0 && fa != fb && in_group[fa] && in_group[fb]
            && find(fa) == find(fb) && plan_of.count(find(fa)))
            edge_internal[ei] = 1;
    }
    std::vector<int> emap(m_topology_edges.size(), -1);
    std::vector<BRepEdge> nedges;
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        if (edge_internal[ei]) continue;
        emap[ei] = (int)nedges.size();
        BRepEdge e = m_topology_edges[ei];
        e.trim_indices.clear();
        nedges.push_back(e);
    }
    std::vector<BRepFace> nfaces;
    std::vector<BRepLoop> nloops;
    std::vector<BRepTrim> ntrims;
    auto copy_face = [&](int fi) {
        BRepFace f = m_faces[fi];
        std::vector<int> nli;
        for (int li : f.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            BRepLoop lp = m_loops[li];
            lp.face_index = (int)nfaces.size();
            std::vector<int> nti;
            for (int ti : lp.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                BRepTrim t = m_trims[ti];
                if (t.edge_index >= 0 && t.edge_index < (int)emap.size()) t.edge_index = emap[t.edge_index];
                t.loop_index = (int)nloops.size();
                nti.push_back((int)ntrims.size());
                ntrims.push_back(t);
            }
            lp.trim_indices = nti;
            nli.push_back((int)nloops.size());
            nloops.push_back(lp);
        }
        f.loop_indices = nli;
        nfaces.push_back(f);
    };
    for (int fi = 0; fi < nf; ++fi) if (!in_group[fi]) copy_face(fi);
    for (auto& [root, P] : plan_of) {
        const Point& O = P.O; const Vector& U = P.U; const Vector& V = P.V;
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        auto at3 = [&](double a, double b) {
            return Point(O[0] + a*U[0] + b*V[0], O[1] + a*U[1] + b*V[1], O[2] + a*U[2] + b*V[2]);
        };
        double amn = P.amn, amx = P.amn + P.ea, bmn = P.bmn, bmx = P.bmn + P.eb;
        srf.set_cv(0, 0, at3(amn, bmn)); srf.set_cv(1, 0, at3(amx, bmn));
        srf.set_cv(0, 1, at3(amn, bmx)); srf.set_cv(1, 1, at3(amx, bmx));
        int nsi = add_surface(srf);
        BRepFace f;
        f.surface_index = nsi;
        f.reversed = m_faces[P.g0].reversed;
        int nfi = (int)nfaces.size();
        for (size_t c = 0; c < P.chains.size(); ++c) {
            BRepLoop lp;
            lp.face_index = nfi;
            lp.type = ((int)c == P.outer) ? BRepLoopType::Outer : BRepLoopType::Inner;
            for (auto& [idx, fwd] : P.chains[c]) {
                NurbsCurve npc = P.mts[idx].pc;
                if (!fwd) npc.reverse();
                if (!npc.is_valid()) { npc = NurbsCurve::create(false, 1, P.mts[idx].ab);
                    if (!fwd) npc.reverse(); }
                if (!npc.is_valid()) continue;
                BRepTrim t;
                t.curve_2d_index = add_curve_2d(npc);
                t.edge_index = (P.mts[idx].edge >= 0 && P.mts[idx].edge < (int)emap.size())
                             ? emap[P.mts[idx].edge] : -1;
                t.loop_index = (int)nloops.size();
                t.reversed = false;
                t.type = P.mts[idx].type;
                lp.trim_indices.push_back((int)ntrims.size());
                ntrims.push_back(t);
            }
            if (lp.trim_indices.empty()) continue;
            f.loop_indices.push_back((int)nloops.size());
            nloops.push_back(lp);
        }
        if (f.loop_indices.empty()) { for (int fi : groups[root]) copy_face(fi); continue; }
        nfaces.push_back(f);
    }
    m_faces = std::move(nfaces);
    m_loops = std::move(nloops);
    m_trims = std::move(ntrims);
    m_topology_edges = std::move(nedges);
    for (int ti = 0; ti < (int)m_trims.size(); ++ti) {
        int ei = m_trims[ti].edge_index;
        if (ei >= 0 && ei < (int)m_topology_edges.size())
            m_topology_edges[ei].trim_indices.push_back(ti);
    }
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        int sv = m_topology_edges[ei].start_vertex;
        int ev = m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei);
    }
}

namespace {
// A recognized analytic solid for O(1) point classification (skips the tessellate+ray-cast).
// Kind 1 = convex polyhedron (box/beam/plate): intersection of outward half-spaces.
// Kind 2 = cylinder: within radius of an axis segment. Kind 0 = unrecognized (caller falls
// back to the mesh ray-cast). Recognition self-VERIFIES; on any mismatch it returns kind 0,
// so a wrong guess can never produce a wrong classification -- only a slower (mesh) one.
struct PrimSolid {
    int kind = 0;
    double tol = 0;
    std::vector<std::array<double,4>> hs;  // half-spaces: inside iff n.p <= d
    Point ca{0,0,0}; Vector cd{0,0,1}; double ch = 0, cr = 0;  // cylinder axis/length/radius
};

static bool srf_is_planar(const NurbsSurface& s) {
    auto du = s.domain(0); auto dv = s.domain(1);
    Vector n0 = s.normal_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
    const double uu[2] = {0.25, 0.75}, vv[2] = {0.3, 0.8};
    for (int i = 0; i < 2; ++i) {
        Vector n = s.normal_at(du.first+(du.second-du.first)*uu[i], dv.first+(dv.second-dv.first)*vv[i]);
        Vector c(n0[1]*n[2]-n0[2]*n[1], n0[2]*n[0]-n0[0]*n[2], n0[0]*n[1]-n0[1]*n[0]);
        if (c.magnitude() > 1e-7) return false;
    }
    return true;
}

static PrimSolid recognize_solid(const BRep& X) {
    PrimSolid ps;
    if (X.m_faces.empty() || X.m_surfaces.empty()) return ps;
    // Derive geometry from the SURFACES (authoritative) rather than m_vertices, which can be
    // stale relative to the surfaces after transformed(). Sample each face's surface at its
    // corners + centre; these points define the bbox, an interior reference point, and the
    // convexity check.
    std::vector<Point> spts;
    for (const auto& f : X.m_faces) {
        if (f.surface_index < 0 || f.surface_index >= (int)X.m_surfaces.size()) return ps;
        const NurbsSurface& s = X.m_surfaces[f.surface_index];
        auto du = s.domain(0); auto dv = s.domain(1);
        for (double a : {0.0, 0.5, 1.0}) for (double b : {0.0, 0.5, 1.0})
            spts.push_back(s.point_at(du.first+(du.second-du.first)*a, dv.first+(dv.second-dv.first)*b));
    }
    if (spts.empty()) return ps;
    double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
    double cx=0,cy=0,cz=0;
    for (const auto& p : spts) {
        xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
        xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        cx+=p[0]; cy+=p[1]; cz+=p[2];
    }
    int nv=(int)spts.size(); Point C(cx/nv, cy/nv, cz/nv);
    double diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
    if (diag < 1e-12) return ps;
    double tol = diag * 1e-6;
    ps.tol = tol;

    // All-planar -> convex polyhedron (box/beam/plate). Build outward half-spaces, verify convex.
    bool all_planar = true;
    for (const auto& f : X.m_faces) {
        if (f.surface_index < 0 || f.surface_index >= (int)X.m_surfaces.size()) { all_planar=false; break; }
        if (!srf_is_planar(X.m_surfaces[f.surface_index])) { all_planar=false; break; }
    }
    if (all_planar) {
        for (const auto& f : X.m_faces) {
            const NurbsSurface& s = X.m_surfaces[f.surface_index];
            auto du = s.domain(0); auto dv = s.domain(1);
            Point q = s.point_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
            Vector n = s.normal_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
            if (n[0]*(C[0]-q[0]) + n[1]*(C[1]-q[1]) + n[2]*(C[2]-q[2]) > 0) n = Vector(-n[0],-n[1],-n[2]);
            ps.hs.push_back({n[0], n[1], n[2], n[0]*q[0]+n[1]*q[1]+n[2]*q[2]});
        }
        for (const auto& vtx : spts)
            for (const auto& h : ps.hs)
                if (h[0]*vtx[0]+h[1]*vtx[1]+h[2]*vtx[2] > h[3] + tol*50) return PrimSolid{};  // not convex
        ps.kind = 1;
        return ps;
    }

    // Sphere: a single (non-planar) face. Fit a centre by least squares from a UV grid of
    // surface points (2(Pi-P0).C = |Pi|^2-|P0|^2), then verify all points are equidistant.
    if (X.m_faces.size() == 1) {
        const NurbsSurface& s = X.m_surfaces[X.m_faces[0].surface_index];
        auto du = s.domain(0); auto dv = s.domain(1);
        std::vector<Point> sp;
        for (int i = 1; i <= 5; ++i) for (int j = 1; j <= 5; ++j)
            sp.push_back(s.point_at(du.first+(du.second-du.first)*i/6.0, dv.first+(dv.second-dv.first)*j/6.0));
        double A[3][3] = {{0,0,0},{0,0,0},{0,0,0}}, b[3] = {0,0,0};
        const Point& p0 = sp[0]; double p0d = p0[0]*p0[0]+p0[1]*p0[1]+p0[2]*p0[2];
        for (size_t i = 1; i < sp.size(); ++i) {
            double r0=2*(sp[i][0]-p0[0]), r1=2*(sp[i][1]-p0[1]), r2=2*(sp[i][2]-p0[2]);
            double rhs = sp[i][0]*sp[i][0]+sp[i][1]*sp[i][1]+sp[i][2]*sp[i][2] - p0d;
            double row[3] = {r0,r1,r2};
            for (int a=0;a<3;++a){ for (int c=0;c<3;++c) A[a][c]+=row[a]*row[c]; b[a]+=row[a]*rhs; }
        }
        // Solve 3x3 A.c=b by Cramer's rule.
        auto det3=[](double m[3][3]){return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);};
        double D = det3(A);
        if (std::abs(D) > 1e-12) {
            double c[3];
            for (int k=0;k<3;++k){ double M[3][3]; for(int a=0;a<3;++a)for(int cc=0;cc<3;++cc)M[a][cc]=(cc==k)?b[a]:A[a][cc]; c[k]=det3(M)/D; }
            Point center(c[0],c[1],c[2]);
            double rs=0; for (auto&p:sp) rs+=center.distance(p); double radius=rs/sp.size();
            bool ok = radius > tol;
            for (auto&p:sp) if (std::abs(center.distance(p)-radius) > diag*1e-3) { ok=false; break; }
            if (ok) { ps.kind=3; ps.ca=center; ps.cr=radius; return ps; }
        }
        // Torus (kind 4): tube-circle centres (isos at fixed major angle) lie on the major
        // circle -- fit centre/axis/R from them, minor r from the tube radii, then verify
        // every sample against |hypot(rho-R, h)| == r. The wrong parameter direction
        // self-rejects: its iso centres coincide on the axis (R ~ 0) and its ring radii
        // vary (R + r cos(phi)). Without this, tor x tor classified as imported_freeform
        // and the freeform pipeline silently hijacked an analytic matrix pair.
        for (int dirp = 0; dirp < 2; ++dirp) {
            const int NM = 12, NT = 16;
            std::vector<Point> tc; std::vector<double> trr;
            bool bad = false;
            for (int i = 0; i < NM && !bad; ++i) {
                double fm = (double)i / NM;
                double cxr=0, cyr=0, czr=0;
                std::vector<Point> ring;
                for (int j = 0; j < NT; ++j) {
                    double ft = (double)j / NT;
                    double uu = du.first + (du.second-du.first) * (dirp == 0 ? fm : ft);
                    double vv = dv.first + (dv.second-dv.first) * (dirp == 0 ? ft : fm);
                    Point q = s.point_at(uu, vv);
                    ring.push_back(q); cxr+=q[0]; cyr+=q[1]; czr+=q[2];
                }
                Point c(cxr/NT, cyr/NT, czr/NT);
                double rm = 0;
                for (auto& q : ring) rm += c.distance(q);
                rm /= NT;
                for (auto& q : ring)
                    if (std::abs(c.distance(q) - rm) > diag*1e-3) { bad = true; break; }
                tc.push_back(c); trr.push_back(rm);
            }
            if (bad || (int)tc.size() < NM) continue;
            double rmin_ = 0; for (double v2 : trr) rmin_ += v2; rmin_ /= trr.size();
            bool rok = rmin_ > tol;
            for (double v2 : trr) if (std::abs(v2 - rmin_) > diag*1e-3) { rok = false; break; }
            if (!rok) continue;
            Point C2(0,0,0);
            for (auto& c : tc) C2 = Point(C2[0]+c[0], C2[1]+c[1], C2[2]+c[2]);
            C2 = Point(C2[0]/tc.size(), C2[1]/tc.size(), C2[2]/tc.size());
            Vector ax(0,0,0);
            for (size_t i = 0; i < tc.size(); ++i) {
                const Point& a2 = tc[i]; const Point& b2 = tc[(i+1)%tc.size()];
                Vector va2(a2[0]-C2[0], a2[1]-C2[1], a2[2]-C2[2]);
                Vector vb2(b2[0]-C2[0], b2[1]-C2[1], b2[2]-C2[2]);
                ax = Vector(ax[0]+va2[1]*vb2[2]-va2[2]*vb2[1],
                            ax[1]+va2[2]*vb2[0]-va2[0]*vb2[2],
                            ax[2]+va2[0]*vb2[1]-va2[1]*vb2[0]);
            }
            double axl = ax.magnitude();
            if (axl < 1e-12) continue;
            ax = Vector(ax[0]/axl, ax[1]/axl, ax[2]/axl);
            double Rmaj = 0;
            for (auto& c : tc) Rmaj += C2.distance(c);
            Rmaj /= tc.size();
            if (Rmaj <= rmin_ * (1.0 + 1e-6)) continue;   // degenerate / wrong direction
            bool tok = true;
            for (int i = 0; i <= 8 && tok; ++i) for (int j = 0; j <= 8 && tok; ++j) {
                Point q = s.point_at(du.first+(du.second-du.first)*i/8.0,
                                     dv.first+(dv.second-dv.first)*j/8.0);
                double wx = q[0]-C2[0], wy = q[1]-C2[1], wz = q[2]-C2[2];
                double h = wx*ax[0]+wy*ax[1]+wz*ax[2];
                double rho = std::sqrt(std::max(0.0, wx*wx+wy*wy+wz*wz - h*h));
                if (std::abs(std::hypot(rho - Rmaj, h) - rmin_) > diag*1e-3) tok = false;
            }
            if (!tok) continue;
            ps.kind = 4; ps.ca = C2; ps.cd = ax; ps.cr = Rmaj; ps.ch = rmin_;
            return ps;
        }
    }

    // Cylinder: exactly 2 planar caps + 1 curved lateral. The two circular cap edges give the
    // axis (cap-centre to cap-centre) and radius; verify the lateral face lies on that cylinder.
    std::vector<int> planar, curved;
    for (int fi = 0; fi < (int)X.m_faces.size(); ++fi) {
        int si = X.m_faces[fi].surface_index;
        if (si < 0) return ps;
        (srf_is_planar(X.m_surfaces[si]) ? planar : curved).push_back(fi);
    }
    if (planar.size() == 2 && curved.size() == 1) {
        // Cap centre/radius from each planar face's outer-loop boundary (sampled in 3D).
        auto cap_circle = [&](int fi, Point& center, double& radius) -> bool {
            const auto& face = X.m_faces[fi];
            const NurbsSurface& s = X.m_surfaces[face.surface_index];
            std::vector<Point> bpts;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)X.m_loops.size()) continue;
                if (X.m_loops[li].type != BRepLoopType::Outer) continue;
                for (int ti : X.m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                    int c2 = X.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = X.m_curves_2d[c2];
                    auto dc = pc.domain();
                    for (int k = 0; k < 16; ++k) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/16.0);
                        bpts.push_back(s.point_at(uv[0], uv[1]));
                    }
                }
            }
            if (bpts.size() < 6) return false;
            double mx=0,my=0,mz=0; for (auto&p:bpts){mx+=p[0];my+=p[1];mz+=p[2];}
            center = Point(mx/bpts.size(), my/bpts.size(), mz/bpts.size());
            double rs=0; for (auto&p:bpts) rs += center.distance(p);
            radius = rs/bpts.size();
            for (auto&p:bpts) if (std::abs(center.distance(p)-radius) > diag*1e-3) return false;  // not a circle
            return true;
        };
        Point c0, c1; double r0, r1;
        if (cap_circle(planar[0], c0, r0) && cap_circle(planar[1], c1, r1) &&
            std::abs(r0 - r1) < diag*1e-3) {
            double h = c0.distance(c1);
            if (h > tol) {
                Vector d((c1[0]-c0[0])/h, (c1[1]-c0[1])/h, (c1[2]-c0[2])/h);
                // Verify the lateral face sits on this cylinder.
                const NurbsSurface& L = X.m_surfaces[X.m_faces[curved[0]].surface_index];
                auto du = L.domain(0); auto dv = L.domain(1); bool ok = true;
                for (int i = 0; i <= 3 && ok; ++i) for (int j = 0; j <= 3 && ok; ++j) {
                    Point p = L.point_at(du.first+(du.second-du.first)*i/3.0, dv.first+(dv.second-dv.first)*j/3.0);
                    Vector w(p[0]-c0[0], p[1]-c0[1], p[2]-c0[2]);
                    double t = w[0]*d[0]+w[1]*d[1]+w[2]*d[2];
                    double rad = std::sqrt(std::max(0.0, w.magnitude()*w.magnitude() - t*t));
                    if (std::abs(rad - r0) > diag*1e-3 || t < -diag*1e-3 || t > h+diag*1e-3) ok = false;
                }
                if (ok) { ps.kind=2; ps.ca=c0; ps.cd=d; ps.ch=h; ps.cr=r0; return ps; }
            }
        }
    }
    return ps;
}

static bool inside_prim(const PrimSolid& ps, const Point& p, double tol) {
    if (ps.kind == 1) {
        for (const auto& h : ps.hs)
            if (h[0]*p[0]+h[1]*p[1]+h[2]*p[2] > h[3] + tol) return false;
        return true;
    }
    if (ps.kind == 2) {
        Vector w(p[0]-ps.ca[0], p[1]-ps.ca[1], p[2]-ps.ca[2]);
        double t = w[0]*ps.cd[0]+w[1]*ps.cd[1]+w[2]*ps.cd[2];
        if (t < -tol || t > ps.ch + tol) return false;
        double rad = std::sqrt(std::max(0.0, w.magnitude()*w.magnitude() - t*t));
        return rad <= ps.cr + tol;
    }
    if (ps.kind == 3) {  // sphere: centre ps.ca, radius ps.cr
        return ps.ca.distance(p) <= ps.cr + tol;
    }
    if (ps.kind == 4) {  // torus: centre ps.ca, axis ps.cd, major R ps.cr, minor r ps.ch
        double wx = p[0]-ps.ca[0], wy = p[1]-ps.ca[1], wz = p[2]-ps.ca[2];
        double h = wx*ps.cd[0]+wy*ps.cd[1]+wz*ps.cd[2];
        double rho = std::sqrt(std::max(0.0, wx*wx+wy*wy+wz*wz - h*h));
        return std::hypot(rho - ps.cr, h) <= ps.ch + tol;
    }
    return false;
}
}  // namespace

static BRep (*s_boolean_backend)(const BRep&, const BRep&, BRep::BooleanOp, double) = nullptr;

void BRep::register_boolean_backend(
    BRep (*fn)(const BRep& A, const BRep& B, BooleanOp op, double tolerance)) {
    s_boolean_backend = fn;
}

BRep BRep::boolean(const BRep& other, BooleanOp op, double tolerance) const {
    // ROUTER: the registered v2 backend answers by default (any binary linking session_v2);
    // SESSION_V1_BOOL or an unregistered backend (session_core-only binaries) selects the v1
    // pipeline. The v2 front end itself delegates to boolean_v1 when it refuses a pair, so
    // routing never strands a case.
    static const bool s_v1 = (std::getenv("SESSION_V1_BOOL") != nullptr);
    if (!s_v1 && s_boolean_backend) return s_boolean_backend(*this, other, op, tolerance);
    return boolean_v1(other, op, tolerance);
}

BRep BRep::boolean_v1(const BRep& other, BooleanOp op, double tolerance) const {
    // AUTO VARIANT SELECTION (SESSION_AUTO; ACIS retry-loop / OCCT escalation doctrine).
    // The junction-repair mechanisms (bridge weld, EF-march) are decisively positive on
    // some knot geometries and negative on others, and no LOCAL acceptance test decides
    // the GLOBAL outcome (measured: z30 12 ungated vs 41 gated; z45 the reverse). So:
    // run the default pipeline; if the result has naked edges, rerun with the repair
    // set enabled and keep whichever result is closer to watertight. Deterministic,
    // Pareto-clean by construction; watertight-on-first-pass inputs never rerun.
    static bool s_in_auto = false;
    // portable env set/clear ("KEY=" clears, matching Windows _putenv semantics)
    auto put_env = [](const char* kv) {
#ifdef _WIN32
        _putenv(kv);
#else
        std::string s(kv);
        size_t eq = s.find('=');
        std::string k = s.substr(0, eq);
        std::string v = eq == std::string::npos ? "" : s.substr(eq + 1);
        if (v.empty()) unsetenv(k.c_str());
        else setenv(k.c_str(), v.c_str(), 1);
#endif
    };
    if (!s_in_auto && std::getenv("SESSION_AUTO")) {
        // selection metric: naked count; a CLOSED candidate must also pass a volume
        // sanity gate (finite, positive, and no larger than vol(A)+vol(B) at scale)
        // so an inverted/degenerate shell can never win the ladder on naked=0 alone.
        double volAB = 0.0, volA = 0.0, volB = 0.0;
        {
            BRep ca = *this, cb = other;
            double va = std::abs(ca.volume()), vb = std::abs(cb.volume());
            if (std::isfinite(va) && std::isfinite(vb)) { volA = va; volB = vb; volAB = va + vb; }
        }
        // PER-OP volume bounds. The old gate only rejected v > vol(A)+vol(B), which a CUT
        // result can satisfy while still being impossible: measured z90 cut = 92.38 with
        // vol(A) = 80.30, reported "solid 1 naked 0" and accepted by the ladder. A cut can
        // never exceed A; an intersection never exceeds min(A,B); a union is bracketed by
        // max(A,B) and A+B. Closed-but-impossible must lose to any true closure.
        auto vol_ok = [&](double v) {
            if (!std::isfinite(v) || v <= 0) return false;
            const double s = 1e-3;
            switch (op) {
                case BooleanOp::Difference:   return v <= volA * (1 + s);
                case BooleanOp::Intersection: return v <= std::min(volA, volB) * (1 + s);
                case BooleanOp::Union:        return v <= volAB * (1 + s) &&
                                                     v >= std::max(volA, volB) * (1 - s);
            }
            return true;
        };
        // Bounding boxes of the operands, for the containment invariant below.
        auto bbox_of = [](const BRep& X, double* mn, double* mx) {
            mn[0]=mn[1]=mn[2]=1e300; mx[0]=mx[1]=mx[2]=-1e300;
            for (const auto& p : X.m_vertices)
                for (int k = 0; k < 3; ++k) { mn[k]=std::min(mn[k],p[k]); mx[k]=std::max(mx[k],p[k]); }
        };
        double amn[3], amx[3], bmn[3], bmx[3];
        bbox_of(*this, amn, amx);
        bbox_of(other, bmn, bmx);
        double bbox_margin = 0.0;
        for (int k = 0; k < 3; ++k) bbox_margin = std::max(bbox_margin, amx[k]-amn[k]);
        bbox_margin *= 1e-3;
        auto metric = [&](BRep& r) -> int {
            int nk = 0, nm = 0;
            for (const auto& e : r.m_topology_edges) {
                int nt = (int)e.trim_indices.size();
                if (nt == 1) ++nk;
                // NON-MANIFOLD edges must COUNT AGAINST a candidate. Counting only 1-trim
                // edges made a 3-trim edge score ZERO -- invisible to the metric and therefore
                // *preferred* over a clean edge. That is how res_y30_cut's 13 non-manifold
                // edges were selected and never registered: OCCT's importer decomposes such an
                // edge and the free boundaries it creates are unhealable (they are where the
                // 58 import-naked edges came from). argmin over a metric blind to the fatal
                // defect class is worse than no metric.
                else if (nt > 2) ++nm;
            }
            nk += nm;
            if (nk == 0) {
                // CONTAINMENT INVARIANT (needs no oracle): a cut cannot leave A's box, an
                // intersection cannot leave EITHER operand's box. Caught instantly by this:
                // freeform_common_box spans (-2.24,-2.64,-2.18)..(2.49,2.64,2.18) while the box
                // operand is only (-2,-2,-2)..(2,2,2) -- an intersection outside an operand.
                double rmn[3], rmx[3];
                bbox_of(r, rmn, rmx);
                bool bad_box = false;
                for (int k = 0; k < 3 && !bad_box; ++k) {
                    if (op == BooleanOp::Difference || op == BooleanOp::Intersection)
                        if (rmn[k] < amn[k]-bbox_margin || rmx[k] > amx[k]+bbox_margin) bad_box = true;
                    if (op == BooleanOp::Intersection)
                        if (rmn[k] < bmn[k]-bbox_margin || rmx[k] > bmx[k]+bbox_margin) bad_box = true;
                }
                if (bad_box) {
                    if (std::getenv("SESSION_NT_DBG"))
                        std::fprintf(stderr, "[AUTO] candidate closed but bbox escapes an operand -> demoted\n");
                    nk = 1;
                }
            }
            if (nk == 0 && volAB > 0) {
                double v = r.volume();
                if (!vol_ok(v)) {
                    if (std::getenv("SESSION_NT_DBG"))
                        std::fprintf(stderr, "[AUTO] candidate closed but vol %.4f out of bounds "
                                     "(A=%.4f B=%.4f op=%d) -> demoted\n", v, volA, volB, (int)op);
                    nk = 1;                    // closed-but-impossible: demote below true closures
                }
            }
            // NOTE: shell count is REPORTED, never gated on. With naked == 0 every edge
            // already has 2 trims, so every connected component is closed BY CONSTRUCTION --
            // "several shells" then means several CLOSED solids, which is the CORRECT answer
            // for many cells (OCCT reference: z30/z45/z37/z63 cut are each 2 solids, x13y29
            // common is 3). An earlier version of this gate demoted shells != 1 and would
            // have rejected those correct answers; the real defect signature is one OPEN
            // shell plus a small closed chunk, and an open shell always shows up as naked > 0.
            if (nk == 0 && std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[AUTO] candidate closed: shells=%d\n", brep_shell_count(r));
            return nk;
        };
        // RAII: recursion flag + resource caps + variant env, all restored on ANY exit
        // path (a ballooning variant THROWS from deep inside the split -- without this the
        // env leaked and s_in_auto stayed set, poisoning every later op in the process).
        const char* touched[] = {"SESSION_BRIDGE", "SESSION_EF_MARCH", "SESSION_NO_EFGATE", "SESSION_M3", "SESSION_SYMEMIT", "SESSION_SEED2"};
        std::map<std::string, std::string> saved;
        for (const char* k : touched) {
            const char* v = std::getenv(k);
            saved[k] = v ? v : "";
        }
        struct AutoScope {
            bool& flag;
            long m0; int r0;
            std::map<std::string, std::string>& env0;
            std::function<void(const char*)> pe;
            AutoScope(bool& f, std::map<std::string, std::string>& e, std::function<void(const char*)> p)
                : flag(f), m0(g_mem_cap_mb), r0(g_segrun_cap), env0(e), pe(std::move(p)) {
                flag = true;
                const char* mc = std::getenv("SESSION_MEM_CAP_MB");
                const char* rc = std::getenv("SESSION_SEGRUN_CAP");
                g_mem_cap_mb = mc ? std::atol(mc) : 3000;
                g_segrun_cap = rc ? std::atoi(rc) : 64;
            }
            void restore_env() {
                for (auto& kv : env0) pe((kv.first + "=" + kv.second).c_str());
            }
            ~AutoScope() {
                restore_env();
                g_mem_cap_mb = m0;
                g_segrun_cap = r0;
                flag = false;
            }
        } scope(s_in_auto, saved, put_env);
        BRep base_res = boolean(other, op, tolerance);
        int nk0 = metric(base_res);
        if (nk0 == 0) return base_res;
        // escalation ladder, strictest first; each variant's env is fully restored
        static const std::vector<std::vector<const char*>> variants = {
            {"SESSION_BRIDGE=2", "SESSION_EF_MARCH=1"},
            {"SESSION_BRIDGE=1", "SESSION_EF_MARCH=1"},
            {"SESSION_BRIDGE=2", "SESSION_EF_MARCH=1", "SESSION_NO_EFGATE=1"},
            {"SESSION_BRIDGE=2", "SESSION_EF_MARCH=1", "SESSION_NO_EFGATE=1", "SESSION_M3="},
            // SEED2 = polyhedral branch DISCOVERY (IntPolyh analog): the marcher is
            // seed-limited and simply never traces branches it was not seeded on, which no
            // amount of repair downstream can invent (doctrine Law 4: discovery != refinement).
            // Measured as a ladder variant: z37 27->16, z30 6->5, x20 +1, rest flat -- so it
            // belongs in the ladder (which keeps the minimum) rather than as a global default.
            {"SESSION_SEED2=1"},
            {"SESSION_SEED2=1", "SESSION_BRIDGE=2", "SESSION_EF_MARCH=1"},
            {"SESSION_SYMEMIT=1"},
            {"SESSION_SYMEMIT=1", "SESSION_BRIDGE=2", "SESSION_EF_MARCH=1"},
        };
        BRep best = std::move(base_res);
        int nk_best = nk0;
        for (const auto& vs : variants) {
            for (const char* v : vs) put_env(v);
            BRep alt;
            int nk = -1;
            try {
                alt = boolean(other, op, tolerance);
            } catch (const std::exception& e) {
                // resource guard trip (or any variant failure) = discarded candidate
                if (std::getenv("SESSION_NT_DBG"))
                    std::fprintf(stderr, "[AUTO] variant %s(+%zu): THREW %s\n",
                                 vs[0], vs.size(), e.what());
                scope.restore_env();
                continue;
            }
            scope.restore_env();
            nk = metric(alt);
            if (std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[AUTO] variant %s(+%zu): naked %d (best %d)\n",
                             vs[0], vs.size(), nk, nk_best);
            if (nk < nk_best) { nk_best = nk; best = std::move(alt); }
            if (nk_best == 0) break;
        }
        if (std::getenv("SESSION_SEAM_AUDIT")) brep_seam_audit(best, tolerance);
        return best;
    }
    // Optional phase profiling (set SESSION_BOOL_PROFILE=1). Prints per-phase microseconds.
    static const bool s_prof = (std::getenv("SESSION_BOOL_PROFILE") != nullptr);
    auto t_now = []{ return std::chrono::high_resolution_clock::now(); };
    auto t_us = [](auto a, auto b){ return std::chrono::duration<double, std::micro>(b - a).count(); };
    auto t_start = t_now();
    auto tp = t_now();
    if (s_prof) std::fprintf(stderr, "[bool-prof] === A=%dfaces B=%dfaces op=%d ===\n",
                             (int)m_faces.size(), (int)other.m_faces.size(), (int)op);
    auto lap = [&](const char* what){ if (s_prof) { auto n = t_now(); std::fprintf(stderr, "[bool-prof]   %-16s %8.1f us\n", what, t_us(tp, n)); std::fflush(stderr); tp = n; } };

    // Imprint each solid against the other (split faces along the SSI curves).
    // Recognize operands FIRST: two unrecognized (freeform, typically imported-STEP) solids
    // need the near-boundary cut-endpoint snap in the UV arrangement; every matrix pair has
    // at least one recognized primitive, so the snap stays off there (byte-identical).
    PrimSolid prA0 = recognize_solid(*this);
    PrimSolid prB0 = recognize_solid(other);
    bool imported_freeform = (prA0.kind == 0 && prB0.kind == 0);
    if (std::getenv("SESSION_REC_DBG"))
        std::printf("[REC] A kind=%d B kind=%d\n", prA0.kind, prB0.kind);
    // Freeform x freeform: run each surface-pair SSI ONCE and feed BOTH splits from the same
    // triples (pcurve_a to A's split, pcurve_b to B's). The freeform marcher is order-sensitive
    // (seeds come from the first argument's cells), so the legacy A-by-B / B-by-A calls can
    // trace a grazing section in one order and miss it in the other -- the imprint then goes
    // asymmetric and the section edge stays naked. One call = symmetric imprint by construction
    // (and half the SSI cost). Matrix pairs (>= 1 recognized primitive) keep the legacy path.
    std::vector<std::vector<NurbsCurve>> cutsA(m_surfaces.size()), cutsB(other.m_surfaces.size());
    std::vector<NurbsCurve> sec_c3ds;   // the exact shared section curves, for snap_section_edges
    // Scaffold mode (S2) computes its own per-pair SSI inside build_section_scaffold below;
    // skip the pre_cuts pass entirely there (its output would be unused).
    // Scaffold is DEFAULT-ON for imported freeform (S6 promotion): it is the path that
    // makes imported-BRep booleans watertight (chairs: all 3 ops closed solids matching
    // OCCT). SESSION_NO_SCAFFOLD reverts to the legacy per-operand imprint+sew path.
    static const bool s_scaffold_off = (std::getenv("SESSION_NO_SCAFFOLD") != nullptr);
    // SESSION_SCAFFOLD_ALL: run the shared-section scaffold for ANY operand pair (not just
    // freeform x freeform) -- experiment gate for promoting the chairs machinery to the
    // marcher-red battery families (off-axis quadric pairs whose sections also come from
    // the walker and suffer the same per-operand imprint+sew divergence).
    static const bool s_scaffold_all = (std::getenv("SESSION_SCAFFOLD_ALL") != nullptr);
    // "Unrecognized" is not "freeform": native quadric pairs recognize_solid can't name
    // (cone x cone -- cones have no recognizer) are still exact rational deg-2 surfaces
    // whose sections the S1 shared-SSI path handles (tangent circles need seam handling
    // the scaffold lacks). Scaffold-by-default only when a genuinely freeform (deg>=3)
    // surface is present -- every imported chair qualifies, every matrix cell does not.
    auto has_freeform = [](const BRep& X) {
        for (const auto& s : X.m_surfaces)
            if (s.degree(0) >= 3 || s.degree(1) >= 3) return true;
        return false;
    };
    bool scaffold_eligible =
        ((imported_freeform && (has_freeform(*this) || has_freeform(other))) || s_scaffold_all)
        && !s_scaffold_off;
    // PRE-SPLIT SAME-DOMAIN PAIRING (2026-08-14, live; kb/p3_integration_notes.md HOOK
    // 1-PRE promoted from diagnostic to driving). Coincident/near-coincident faces are
    // recognised on the ORIGINAL operands, BEFORE any SSI: a surface must never be
    // intersected with its own coincident partner (OCCT: an FF interference between
    // same-domain faces yields an SD pair, never section curves -- the near-parallel
    // marcher otherwise minted 4 overlapping wall fragments at phi=1e-6 flush boxes,
    // all kept). Pairs are keyed by ORIGINAL face; fragments reach the verdict through
    // src_faces. Fuzz band diag*2e-4 covers the sub-tolerance angle decades (the SD
    // resolution IS the correct answer there: the wedge volume is below tolerance).
    // SESSION_SD_NOHOOKS reverts.
    std::map<int, bool> pre_pairA, pre_pairB;      // ORIGINAL face -> orient_same
    std::set<std::pair<int, int>> sd_suppress;     // (surfA, surfB) SSI suppression
    if (imported_freeform && !std::getenv("SESSION_SD_NOHOOKS")) {
        double mn[3] = {1e300, 1e300, 1e300}, mx[3] = {-1e300, -1e300, -1e300};
        for (const auto& p : m_vertices)
            for (int k = 0; k < 3; ++k) { mn[k] = std::min(mn[k], p[k]); mx[k] = std::max(mx[k], p[k]); }
        for (const auto& p : other.m_vertices)
            for (int k = 0; k < 3; ++k) { mn[k] = std::min(mn[k], p[k]); mx[k] = std::max(mx[k], p[k]); }
        const double diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0]) + (mx[1]-mn[1])*(mx[1]-mn[1])
                                      + (mx[2]-mn[2])*(mx[2]-mn[2]));
        // Fuzz at PRECISION scale (OCCT Precision::Confusion doctrine), never a fat
        // band: pairing must fire only for true sub-tolerance coincidence. A fat band
        // (diag*2e-4, first attempt) paired walls whose wedge is REAL geometry and
        // silently discarded measurable volume (phi=0.001..0.1 deg common -> 0).
        // Strictly precision-scale (diag*1e-7): the boolean's working tolerance (~1e-3)
        // must NOT widen pairing -- at 0.001..0.01 deg the wedge (1e-4..1e-3 volume) is
        // real geometry the oracle measures; silently absorbing sub-tolerance features
        // is the fuzzy-boolean's job only when a user asks for it.
        SameDomain sdp(std::max(diag * 1e-6, 1e-9), std::max(diag * 1e-7, 1e-9));
        for (int f = 0; f < (int)m_faces.size(); ++f) sdp.add_face(*this, f, 0, 0);
        for (int f = 0; f < (int)other.m_faces.size(); ++f) sdp.add_face(other, f, 1, 1);
        sdp.detect();
        for (const auto& pr : sdp.pairs()) {
            const SDFaceRec& fa = sdp.face(pr.i);
            const SDFaceRec& fb = sdp.face(pr.j);
            if (fa.operand == fb.operand) continue;
            const SDFaceRec& ra = fa.operand ? fb : fa;    // A-side record
            const SDFaceRec& rb = fa.operand ? fa : fb;
            const bool os = pr.orient == 0;
            pre_pairA[ra.face] = os;
            pre_pairB[rb.face] = os;
            sd_suppress.insert({m_faces[ra.face].surface_index,
                                other.m_faces[rb.face].surface_index});
            if (std::getenv("SESSION_SD"))
                std::fprintf(stderr, "[SD-PRE-PAIR] A f%d <-> B f%d orient=%s via=%d\n",
                             ra.face, rb.face, os ? "same" : "opposite", pr.via);
        }
    }
    if (imported_freeform && !scaffold_eligible) {
        std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> abbs, bbbs;
        for (const auto& s : m_surfaces) abbs.push_back(aabb_from_surface(s));
        for (const auto& s : other.m_surfaces) bbbs.push_back(aabb_from_surface(s));
        for (size_t ai = 0; ai < m_surfaces.size(); ++ai) {
            double am = std::max({abbs[ai].second[0] - abbs[ai].first[0],
                                  abbs[ai].second[1] - abbs[ai].first[1],
                                  abbs[ai].second[2] - abbs[ai].first[2]}) * 1e-3;
            for (size_t bi = 0; bi < other.m_surfaces.size(); ++bi) {
                if (!aabb_overlap(abbs[ai], bbbs[bi], am)) continue;
                // Coincident partners never intersect each other (SD pair, not sections).
                if (sd_suppress.count({(int)ai, (int)bi})) continue;
                auto trs = Intersection::surface_surface(m_surfaces[ai], other.m_surfaces[bi], tolerance);
                if (trs.empty()) {
                    // order-sensitive marcher found nothing this way round: retry swapped and
                    // exchange the pcurve roles so both operands still share one section set
                    for (auto& tr : Intersection::surface_surface(other.m_surfaces[bi], m_surfaces[ai], tolerance)) {
                        if (std::get<2>(tr).is_valid()) cutsA[ai].push_back(std::get<2>(tr));
                        if (std::get<1>(tr).is_valid()) cutsB[bi].push_back(std::get<1>(tr));
                        if (std::get<0>(tr).is_valid()) sec_c3ds.push_back(std::get<0>(tr));
                    }
                    continue;
                }
                for (auto& tr : trs) {
                    if (std::get<1>(tr).is_valid()) cutsA[ai].push_back(std::get<1>(tr));
                    if (std::get<2>(tr).is_valid()) cutsB[bi].push_back(std::get<2>(tr));
                    if (std::get<0>(tr).is_valid()) sec_c3ds.push_back(std::get<0>(tr));
                }
            }
        }
        // Paired (coincident) faces take NO cuts at all: their resolution is the SD
        // verdict, and at sub-tolerance angles the neighbouring faces' SSI only GRAZES
        // the wall along its boundary, minting overlapping pseudo-fragments (phi=1e-6:
        // 4 copies of A's wall, every one sampling at the wall centre). At larger
        // angles the pair does not form and cuts apply normally.
        for (const auto& sp2 : sd_suppress) {
            if (sp2.first >= 0 && sp2.first < (int)cutsA.size()) cutsA[sp2.first].clear();
            if (sp2.second >= 0 && sp2.second < (int)cutsB.size()) cutsB[sp2.second].clear();
        }
        lap("pair_ssi");
    }
    // S1 (OCCT-adoption plan): build the shared section scaffold and print its
    // self-diagnostics. Inert -- output unused until S2 wires it into the splits.
    // Runs its OWN fenced pair loop (lane decision: pre_cuts above stays untouched).
    SectionScaffold scaf;
    bool use_scaffold = false;
    std::map<int, std::array<int, 3>> secA_edges, secB_edges;   // split edge idx -> {seg,v0,v1}
    if (scaffold_eligible) {
        scaf = build_section_scaffold(*this, other, tolerance);
        use_scaffold = true;
        std::fprintf(stderr,
            "[SCAF] chains=%d segs=%zu verts=%zu paves(tA=%d tB=%d x=%d v=%d c=%d) "
            "drop(verdict=%d micro=%d) bridge(march=%d weld=%d resid=%d) dev(A=%.3e B=%.3e) tol3=%.3e\n",
            scaf.n_chains, scaf.segments.size(), scaf.vertices.size(),
            scaf.n_paves_trimA, scaf.n_paves_trimB, scaf.n_paves_xing,
            scaf.n_paves_vertex, scaf.n_paves_closing,
            scaf.n_dropped_verdict, scaf.n_dropped_micro,
            scaf.n_bridge_marched, scaf.n_bridge_welded, scaf.n_bridge_residual,
            scaf.max_devA, scaf.max_devB, scaf.tol3);
        std::fflush(stderr);
        lap("scaffold");
    }
    // BOP2 M1 (SESSION_BOP2): mint the shared-edge pool ONCE from the finished scaffold. Foundation
    // for the shared-topology split (Phase 4); does not yet change the split path. Verify: each section
    // segment -> exactly one arena edge with shared welded endpoints.
    SharedEdgePool bop2_pool;
    const bool use_bop2 = use_scaffold && std::getenv("SESSION_BOP2") != nullptr;
    if (use_bop2) {
        bop2_pool = build_shared_edge_pool(scaf);
        int nedge = (int)bop2_pool.arena.m_topology_edges.size();
        int nvert = (int)bop2_pool.arena.m_topology_vertices.size();
        int nseg_keyed = 0;
        for (int e : bop2_pool.seg_edge) if (e >= 0) ++nseg_keyed;
        std::fprintf(stderr, "[BOP2] shared-edge pool: verts=%d edges=%d seg_keyed=%d/%zu (segs=%zu)\n",
                     nvert, nedge, nseg_keyed, bop2_pool.seg_edge.size(), scaf.segments.size());
        std::fflush(stderr);
        lap("bop2_pool");
    }
    // HOOK 1-PRE (SESSION_SD): same-domain detection on the ORIGINAL faces, BEFORE any split.
    // MEASURED NECESSITY: on A-op-A (chair1 = byte copy of chair0, so every face is exactly
    // coincident) the post-split hook is useless -- the section stage has already produced 481
    // "intersection" segments between coincident surfaces and shattered A into 312 fragments
    // against B's 20, so no edge-set bucketing can pair them. A surface cannot be intersected
    // with itself; coincident pairs must be recognised HERE and routed to the same-domain
    // path instead of the marcher (OCCT: an FF interference between same-domain faces yields
    // a SD pair, never section curves).
    if (use_scaffold && std::getenv("SESSION_SD") && !std::getenv("SESSION_SD_NOHOOKS")) {
        double key_tol_pre = scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3;
        SameDomain sdp(key_tol_pre);
        for (int f = 0; f < (int)m_faces.size(); ++f) sdp.add_face(*this, f, 0, 0);
        for (int f = 0; f < (int)other.m_faces.size(); ++f) sdp.add_face(other, f, 1, 1);
        sdp.detect();
        std::fprintf(stderr, "[SD-PRE] key_tol=%.4e faces(A=%zu B=%zu) groups=%d pairs=%zu\n",
                     key_tol_pre, m_faces.size(), other.m_faces.size(),
                     sdp.group_count(), sdp.pairs().size());
        for (const auto& pr : sdp.pairs())
            std::fprintf(stderr, "[SD-PRE-PAIR] %c f%d <-> %c f%d orient=%s\n",
                         sdp.face(pr.i).operand ? 'B' : 'A', sdp.face(pr.i).face,
                         sdp.face(pr.j).operand ? 'B' : 'A', sdp.face(pr.j).face,
                         pr.orient == 0 ? "same" : "opposite");
        std::fflush(stderr);
    }
    std::vector<int> srcA_faces, srcB_faces;   // fragment -> original face (angle classify)
    // Pave-block spans: edge -> {seg_id, fa, fb} for every chain-lifted section run.
    std::map<int, std::array<double, 3>> spansA, spansB;
    static const bool s_no_blocks = (std::getenv("SESSION_NO_BLOCKS") != nullptr);
    // B splits FIRST: same-domain (coincident) contact patches are interior to both
    // operands' faces, and only one side's rim sections isolate them (chairs: the patch
    // lies inside ONE B face but straddles several A faces). B2's coincident fragments
    // then drive the imprint of the contact boundary onto A's surfaces -- without it A
    // never splits there, the ON pair is one-sided, and fuse keeps neither copy (hole).
    BRep B2, A2;
    std::vector<std::vector<NurbsCurve>> onCutsA;
    std::vector<std::vector<NurbsCurve>> onCutsB;   // P1c: symmetric same-domain imprints onto B
    // P1c (SESSION_SYMEMIT): curve-level section imprints computed in the symmetric-emission pass
    // below. When one operand's arrangement PRUNES a real scaffold section run (dangling), the other
    // operand's copy is left with no mate (naked). These carry that missing operand the section's own
    // pcurve (the segment's uvA/uvB, projected as a closed loop) so its arrangement re-imprints the
    // contact and sew mates it. Persist across run_splits() re-entries; merged into onCutsA/onCutsB.
    std::vector<std::vector<NurbsCurve>> p1c_onA, p1c_onB;
    // (surface, seg) cuts each operand's split actually fed -- snapshotted from g_fed_cuts
    // right after each split_by_brep call (see the SYMEMIT-DUP filter below).
    std::set<std::pair<int, int>> fedA, fedB;
    // One split pass over BOTH operands. Re-entrant: the preserve-identity loop below
    // re-runs it after the scaffold has been refined, so every piece of per-pass state is
    // cleared here rather than at its declaration.
    auto run_splits = [&]() {
    secB_edges.clear(); secA_edges.clear();
    spansB.clear(); spansA.clear();
    srcB_faces.clear(); srcA_faces.clear();
    onCutsA.clear();
    onCutsB.clear();
    // P1c: pre-load B's channel with the section imprints computed for it (empty on pass 1).
    if (use_scaffold && !p1c_onB.empty()) onCutsB = p1c_onB;
    B2 = other.split_by_brep(*this, tolerance, imported_freeform,
                                  (imported_freeform && !use_scaffold) ? &cutsB : nullptr,
                                  use_scaffold ? &scaf : nullptr, false,
                                  use_scaffold ? &secB_edges : nullptr,
                                  use_scaffold ? &srcB_faces : nullptr,
                                  (use_scaffold && !onCutsB.empty()) ? &onCutsB : nullptr,
                                  use_scaffold ? &spansB : nullptr,
                                  use_bop2 ? &bop2_pool : nullptr); lap("splitB");
    fedB = g_fed_cuts;
    if (use_scaffold) {
        onCutsA.resize(m_surfaces.size());
        double on_det;
        {
            double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
            for (const auto& q : m_vertices) { xmn=std::min(xmn,q[0]); ymn=std::min(ymn,q[1]); zmn=std::min(zmn,q[2]);
                xmx=std::max(xmx,q[0]); ymx=std::max(ymx,q[1]); zmx=std::max(zmx,q[2]); }
            double dg = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
            on_det = (dg > 0 ? dg : 1.0) * 2e-3;
        }
        std::vector<std::pair<std::array<double,3>, std::array<double,3>>> a_sbbs;
        for (const auto& s : m_surfaces) a_sbbs.push_back(aabb_from_surface(s));
        int n_imprint = 0;
        // SESSION_SD: this legacy ON-imprint path projects EVERY B2 fragment that rides on an
        // A surface back onto A as a same-domain cut. On coincident operands that is the whole
        // boundary (A-op-A: 20 of 20 fragments), so it re-shatters exactly the region the
        // common-block suppression just cleaned -- a clean section network is worthless if a
        // second path re-imprints the coincident region anyway. Under SESSION_SD the
        // same-domain machinery owns coincidence, so this path stands down.
        static const bool s_sd_mode = (std::getenv("SESSION_SD") != nullptr);
        for (int fb = 0; !s_sd_mode && fb < (int)B2.m_faces.size(); ++fb) {
            const auto& face = B2.m_faces[fb];
            if (face.surface_index < 0) continue;
            const NurbsSurface& sb = B2.m_surfaces[face.surface_index];
            // 3D samples of the fragment's trims
            std::vector<Point> smp;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)B2.m_loops.size()) continue;
                for (int ti : B2.m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)B2.m_trims.size()) continue;
                    int c2 = B2.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)B2.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = B2.m_curves_2d[c2];
                    auto dc = pc.domain();
                    for (int k = 0; k < 6; ++k) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*(k+0.5)/6.0);
                        smp.push_back(sb.point_at(uv[0], uv[1]));
                    }
                }
            }
            if (smp.size() < 8) continue;
            // ---- ON-relationship test ----
            // LEGACY predicate (kept for A/B): rim-only samples, "nearly all" within a flat
            // band (2 outliers tolerated), projected onto the UNTRIMMED surface, no
            // orientation check, first match wins. Every one of those is unsound:
            //  - rim-only is not AREAL: a fragment whose rim grazes a surface while its
            //    interior bulges away passes;
            //  - the 2-outlier slack is not fail-closed;
            //  - Closest::surface_point projects onto the infinite surface, so a fragment
            //    nowhere near A's actual FACE still matches;
            //  - with no orientation test a fragment lying on the far side, or meeting the
            //    surface at an angle, is imprinted as if coincident.
            // MEASURED CONSEQUENCE: on A-op-A it fires 20/20 (all spurious, re-shattering the
            // boundary); on y30 -- which has NO coincidence at all (SD groups=0) -- disabling
            // it alone moved the cut from 23.9619 to 47.6964 against a 46.9596 reference.
            // CORRECTED predicate (SESSION_ONIMP_FIX): AREAL interior probes, fail-closed (ALL
            // probes), projection required to land INSIDE one of A's real faces on that
            // surface, and normals required to agree (|cos| >= 0.9) at every probe.
            static const bool s_onimp_fix = (std::getenv("SESSION_ONIMP_FIX") != nullptr);
            static const bool s_onimp_dbg = (std::getenv("SESSION_ONIMP_DBG") != nullptr);
            // interior (in-material) probes of the fragment, in its own uv
            std::vector<std::array<double,2>> fpoly;   // uv rim polygon (outer loop)
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)B2.m_loops.size()) continue;
                if (B2.m_loops[li].type != BRepLoopType::Outer) continue;
                for (int ti : B2.m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)B2.m_trims.size()) continue;
                    int c2 = B2.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)B2.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = B2.m_curves_2d[c2];
                    auto dc = pc.domain();
                    for (int k = 0; k < 8; ++k) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/8.0);
                        fpoly.push_back({uv[0], uv[1]});
                    }
                }
            }
            auto uv_in_poly = [](const std::vector<std::array<double,2>>& poly, double u, double v) {
                bool in = false;
                for (size_t a2 = 0, b2 = poly.size()-1; a2 < poly.size(); b2 = a2++)
                    if (((poly[a2][1] > v) != (poly[b2][1] > v)) &&
                        (u < (poly[b2][0]-poly[a2][0])*(v-poly[a2][1])/(poly[b2][1]-poly[a2][1]+1e-30)+poly[a2][0]))
                        in = !in;
                return in;
            };
            std::vector<std::array<double,2>> probes_uv;
            if (fpoly.size() >= 3) {
                double umn=1e300,umx=-1e300,vmn=1e300,vmx=-1e300;
                for (const auto& p : fpoly) { umn=std::min(umn,p[0]); umx=std::max(umx,p[0]);
                                              vmn=std::min(vmn,p[1]); vmx=std::max(vmx,p[1]); }
                for (int i2 = 1; i2 <= 4; ++i2)
                    for (int j2 = 1; j2 <= 4; ++j2) {
                        double u = umn + (umx-umn)*i2/5.0, v = vmn + (vmx-vmn)*j2/5.0;
                        if (uv_in_poly(fpoly, u, v)) probes_uv.push_back({u, v});
                    }
                // THIN-FRAGMENT FALLBACK. A coarse grid misses a thin sliver entirely, and
                // "no probes" then made areal_ok false -- rejecting by ARTIFACT, not by
                // geometry. MEASURED: all 27 y30 candidates reported probes=0 while being
                // EXACTLY coincident (dmin=dmax=0.0000, |cos|=1.000), so the areal test was
                // silently behaving as a wholesale disable for every thin fragment. Fall back
                // to rim-edge midpoints pulled slightly toward the polygon centroid: always
                // in-material, and it degrades gracefully instead of vanishing.
                if (probes_uv.empty() && fpoly.size() >= 3) {
                    double cu2 = 0, cv2 = 0;
                    for (const auto& p : fpoly) { cu2 += p[0]; cv2 += p[1]; }
                    cu2 /= (double)fpoly.size(); cv2 /= (double)fpoly.size();
                    if (uv_in_poly(fpoly, cu2, cv2)) probes_uv.push_back({cu2, cv2});
                    for (size_t k2 = 0; k2 < fpoly.size(); k2 += std::max<size_t>(1, fpoly.size()/8)) {
                        double u = fpoly[k2][0] + (cu2 - fpoly[k2][0]) * 0.25;
                        double v = fpoly[k2][1] + (cv2 - fpoly[k2][1]) * 0.25;
                        if (uv_in_poly(fpoly, u, v)) probes_uv.push_back({u, v});
                    }
                }
            }
            int best_sa = -1;
            for (int sa = 0; sa < (int)m_surfaces.size() && best_sa < 0; ++sa) {
                bool overlap = true;
                for (int k = 0; k < 3; ++k)
                    if (smp[0][k] < a_sbbs[sa].first[k]-on_det*4 || smp[0][k] > a_sbbs[sa].second[k]+on_det*4) overlap = false;
                if (!overlap) continue;
                int nco = 0;
                double dmin = 1e300, dmax = 0;
                for (const auto& q : smp) {
                    auto [cu, cv2, cd] = Closest::surface_point(m_surfaces[sa], q);
                    (void)cu; (void)cv2;
                    dmin = std::min(dmin, cd); dmax = std::max(dmax, cd);
                    if (cd < on_det) ++nco;
                }
                bool legacy_ok = (nco >= (int)smp.size() - 2);
                // corrected: areal + fail-closed + on-the-FACE + orientation
                bool areal_ok = !probes_uv.empty();
                double worst_dot = 1.0;
                if (areal_ok) {
                    for (const auto& puv : probes_uv) {
                        Point q = sb.point_at(puv[0], puv[1]);
                        auto [cu, cv2, cd] = Closest::surface_point(m_surfaces[sa], q);
                        if (!(cd < on_det)) { areal_ok = false; break; }
                        // must land inside a REAL face of A on this surface
                        bool on_face = false;
                        for (int af = 0; af < (int)m_faces.size() && !on_face; ++af) {
                            if (m_faces[af].surface_index != sa) continue;
                            for (int ali : m_faces[af].loop_indices) {
                                if (ali < 0 || ali >= (int)m_loops.size()) continue;
                                if (m_loops[ali].type != BRepLoopType::Outer) continue;
                                std::vector<std::array<double,2>> apoly;
                                for (int ati : m_loops[ali].trim_indices) {
                                    if (ati < 0 || ati >= (int)m_trims.size()) continue;
                                    int ac2 = m_trims[ati].curve_2d_index;
                                    if (ac2 < 0 || ac2 >= (int)m_curves_2d.size()) continue;
                                    const NurbsCurve& apc = m_curves_2d[ac2];
                                    auto adc = apc.domain();
                                    for (int k = 0; k < 8; ++k) {
                                        Point auv = apc.point_at(adc.first + (adc.second-adc.first)*k/8.0);
                                        apoly.push_back({auv[0], auv[1]});
                                    }
                                }
                                if (apoly.size() >= 3 && uv_in_poly(apoly, cu, cv2)) { on_face = true; break; }
                            }
                        }
                        if (!on_face) { areal_ok = false; break; }
                        Vector nb2 = sb.normal_at(puv[0], puv[1]);
                        Vector na2 = m_surfaces[sa].normal_at(cu, cv2);
                        double dt = std::abs(nb2[0]*na2[0] + nb2[1]*na2[1] + nb2[2]*na2[2]);
                        worst_dot = std::min(worst_dot, dt);
                        if (dt < 0.9) { areal_ok = false; break; }
                    }
                }
                if (s_onimp_dbg && (legacy_ok || areal_ok))
                    std::fprintf(stderr, "[ONIMP-CAND] fb=%d sa=%d rim=%d/%zu dmin=%.4f dmax=%.4f "
                                 "probes=%zu legacy=%d areal=%d wdot=%.3f\n",
                                 fb, sa, nco, smp.size(), dmin, dmax, probes_uv.size(),
                                 legacy_ok ? 1 : 0, areal_ok ? 1 : 0, worst_dot);
                if (std::getenv("SESSION_ONDET_DBG") && nco > 0)
                    std::fprintf(stderr, "[ONDET] fb=%d sa=%d nco=%d/%zu dmin=%.4f dmax=%.4f band=%.4f\n",
                                 fb, sa, nco, smp.size(), dmin, dmax, on_det);
                if (s_onimp_fix ? areal_ok : legacy_ok) best_sa = sa;
            }
            if (best_sa < 0) continue;
            // imprint: project each LOOP of the fragment onto A's surface as a closed pcurve
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)B2.m_loops.size()) continue;
                std::vector<Point> uvp;
                for (int ti : B2.m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)B2.m_trims.size()) continue;
                    int c2 = B2.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)B2.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = B2.m_curves_2d[c2];
                    auto dc = pc.domain();
                    bool fwd = !B2.m_trims[ti].reversed;
                    for (int k = 0; k < 24; ++k) {
                        double f = (double)k / 24.0;
                        double t = fwd ? dc.first + (dc.second-dc.first)*f
                                       : dc.second - (dc.second-dc.first)*f;
                        Point uv = pc.point_at(t);
                        Point q3 = sb.point_at(uv[0], uv[1]);
                        auto [cu, cv2, cd] = Closest::surface_point(m_surfaces[best_sa], q3);
                        (void)cd;
                        uvp.push_back(Point(cu, cv2, 0.0));
                    }
                }
                if (uvp.size() < 8) continue;
                uvp.push_back(uvp.front());   // close
                NurbsCurve pcA = NurbsCurve::create(false, 1, uvp);
                if (pcA.is_valid()) { onCutsA[best_sa].push_back(pcA); ++n_imprint; }
            }
        }
        if (n_imprint && std::getenv("SESSION_NT_DBG"))
            std::printf("[ONIMP] same-domain imprints onto A: %d\n", n_imprint);
        // P1c: append the section imprints computed for A (empty on pass 1).
        if (!p1c_onA.empty()) {
            if ((int)onCutsA.size() < (int)m_surfaces.size()) onCutsA.resize(m_surfaces.size());
            for (int sa = 0; sa < (int)p1c_onA.size() && sa < (int)onCutsA.size(); ++sa)
                for (const auto& pc : p1c_onA[sa]) onCutsA[sa].push_back(pc);
        }
        lap("on_imprint");
    }
    A2 = split_by_brep(other, tolerance, imported_freeform,
                            (imported_freeform && !use_scaffold) ? &cutsA : nullptr,
                            use_scaffold ? &scaf : nullptr, true,
                            use_scaffold ? &secA_edges : nullptr,
                            use_scaffold ? &srcA_faces : nullptr,
                            use_scaffold ? &onCutsA : nullptr,
                            use_scaffold ? &spansA : nullptr,
                            use_bop2 ? &bop2_pool : nullptr);      lap("splitA");
    fedA = g_fed_cuts;
    };   // end run_splits
    run_splits();
    // HOOK 1 (SESSION_SD, kb/p3_integration_notes.md): same-domain DETECTION over the two
    // SPLIT operands, in OCCT's FillSameDomainFaces slot (after BuildSplitFaces, before
    // selection). Diagnostic-only at this stage: it does not mutate either operand and does
    // not yet drive selection (HOOK 2). Motivation is measured, not speculative -- x20 cut
    // deletes 12 of A's 36 fragments whose probe points sit at distance 0.00e+00 from B's
    // boundary while our own ON-detector reports on=0 for every one of them.
    // key_tol is LOAD-BEARING per B's precondition: we have no shared section entities, so
    // geometric bucketing at key_tol substitutes for OCCT's IsSame identity. Too tight =>
    // zero SD pairs => silently wrong COMMON/CUT. B recommends tol3_rep.
    // HOOK 2 (2026-08-14, live): detection now DRIVES selection. Pairs land in
    // sd_pairA/sd_pairB (split-face index -> orient_same) consumed by classify's ON
    // branch via the verified BuildBOP table (sd_select_sd_face) -- decided once per
    // PAIR, never by per-face probes, which coin-flip exactly at coincident walls
    // (GZ-J flush boxes: probe ON missed A's wall entirely). SESSION_SD_NOHOOKS
    // reverts to probe-only.
    std::map<int, bool> sd_pairA, sd_pairB;   // split-face index -> orient_same
    if (use_scaffold && !std::getenv("SESSION_SD_NOHOOKS")) {
        double key_tol = scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3;
        SameDomain sd(key_tol);
        for (int f = 0; f < A2.face_count(); ++f) sd.add_face(A2, f, 0, 0);
        for (int f = 0; f < B2.face_count(); ++f) sd.add_face(B2, f, 1, 1);
        sd.detect();
        for (const auto& pr : sd.pairs()) {
            const SDFaceRec& fi2 = sd.face(pr.i);
            const SDFaceRec& fj2 = sd.face(pr.j);
            if (fi2.operand == fj2.operand) continue;      // cross-operand pairs only
            const bool os = pr.orient == 0;
            (fi2.operand ? sd_pairB : sd_pairA)[fi2.face] = os;
            (fj2.operand ? sd_pairB : sd_pairA)[fj2.face] = os;
        }
        if (std::getenv("SESSION_SD")) {
            std::fprintf(stderr, "[SD] key_tol=%.4e faces(A=%d B=%d) groups=%d pairs=%zu\n",
                         key_tol, A2.face_count(), B2.face_count(),
                         sd.group_count(), sd.pairs().size());
            for (const auto& pr : sd.pairs()) {
                const SDFaceRec& fi2 = sd.face(pr.i);
                const SDFaceRec& fj2 = sd.face(pr.j);
                std::fprintf(stderr, "[SD-PAIR] %c f%d <-> %c f%d orient=%s via=%d\n",
                             fi2.operand ? 'B' : 'A', fi2.face,
                             fj2.operand ? 'B' : 'A', fj2.face,
                             pr.orient == 0 ? "same" : "opposite", pr.via);
            }
            std::fflush(stderr);
        }
    }
    // P1c SYMMETRIC EMISSION (SESSION_SYMEMIT): the two operands' arrangements can disagree on which
    // scaffold section runs survive the dangling-edge prune -- one operand emits a section segment the
    // other PRUNES (measured z90 cut: A drops segs 0,1,2,20,21,22 that B keeps). The pruned side leaves
    // the kept side's copy with no mate -> a naked section loop. Feed each missing operand the segment's
    // OWN pcurve (its uvA/uvB on the relevant surface) as a curve-level same-domain imprint, then
    // re-split: the imprint re-establishes the contact on the pruned side and sew mates the two copies.
    if (use_scaffold && std::getenv("SESSION_SYMEMIT")) {
        std::set<int> segsA, segsB;
        for (const auto& kv : spansA) segsA.insert((int)kv.second[0]);
        for (const auto& kv : spansB) segsB.insert((int)kv.second[0]);
        const std::vector<NurbsSurface>& a_surfs = m_surfaces;
        const std::vector<NurbsSurface>& b_surfs = other.m_surfaces;
        const int MAX_PER_SURF = 4;     // guard: a face already carries many section cuts; feeding
        const int MAX_TOTAL    = 16;    // more risks an O(n^2) arrangement blow-up on grazing configs
        auto emit_for = [&](bool forA, std::vector<std::vector<NurbsCurve>>& out, int nsurf) -> int {
            const std::set<int>& have  = forA ? segsA : segsB;
            const std::set<int>& other_have = forA ? segsB : segsA;
            const std::vector<NurbsSurface>& surfs = forA ? a_surfs : b_surfs;
            const std::set<std::pair<int, int>>& fed = forA ? fedA : fedB;
            out.assign(nsurf, {});
            int n = 0;
            for (int sid : other_have) {
                if (have.count(sid) || sid < 0 || sid >= (int)scaf.segments.size()) continue;
                const SectionSegment& s = scaf.segments[sid];
                int surf = forA ? s.surfA : s.surfB;
                const std::vector<Point>& uv = forA ? s.uvA : s.uvB;
                if (surf < 0 || surf >= nsurf || uv.size() < 2) continue;
                // SYMEMIT-DUP (task #9 root cause): "missing from spans" does NOT mean
                // "never fed". split_with feeds EVERY scaffold segment on the surface as a
                // cut (segs_by_surf*), so a segment whose runs were PRUNED downstream is
                // still in that face's cut set. Re-imprinting its pcurve here puts a SECOND
                // near-identical polyline into the same UV arrangement: the two copies
                // cross at every sampling difference and dice the segment into a sliver
                // ladder (measured z37: seg 11 lifted 459 runs, 19k edges, 8.3 GB ->
                // std::bad_alloc; x20: seg 21 at 278 runs and non-terminating). A duplicate
                // curve can never supply the missing run -- the prune is downstream of the
                // cut. Emit ONLY for segments this operand genuinely never fed.
                if (!std::getenv("SESSION_SYMEMIT_DUP") && fed.count({surf, sid})) {
                    if (std::getenv("SESSION_NT_DBG"))
                        std::fprintf(stderr, "[SYMEMIT] %s skip-fed seg=%d surf=%d\n",
                                     forA ? "A" : "B", sid, surf);
                    continue;
                }
                if ((int)out[surf].size() >= MAX_PER_SURF || n >= MAX_TOTAL) continue;
                // Validate: finite, non-degenerate, and inside the target surface's uv domain
                // (a stray/grazing segment can carry uv far outside -> arrangement explosion).
                auto du = surfs[surf].domain(0);
                auto dv = surfs[surf].domain(1);
                double mu = (du.second - du.first) * 0.05, mv = (dv.second - dv.first) * 0.05;
                double umn=1e300,umx=-1e300,vmn=1e300,vmx=-1e300; bool ok = true;
                for (const auto& p : uv) {
                    if (!std::isfinite(p[0]) || !std::isfinite(p[1])) { ok = false; break; }
                    umn=std::min(umn,p[0]); umx=std::max(umx,p[0]);
                    vmn=std::min(vmn,p[1]); vmx=std::max(vmx,p[1]);
                }
                if (ok) ok = (umn >= du.first - mu && umx <= du.second + mu &&
                              vmn >= dv.first - mv && vmx <= dv.second + mv);
                double ext = std::hypot(umx - umn, vmx - vmn);
                if (ok) ok = (ext > 1e-7 && ext < (du.second - du.first) + (dv.second - dv.first));
                if (std::getenv("SESSION_NT_DBG"))
                    std::fprintf(stderr, "[SYMEMIT] %s miss seg=%d surf=%d uv=%zu ext=%.4f%s\n",
                                 forA ? "A" : "B", sid, surf, uv.size(), ext, ok ? "" : " SKIP");
                if (!ok) continue;
                std::vector<Point> pts;
                for (const auto& p : uv) pts.push_back(Point(p[0], p[1], 0.0));
                NurbsCurve pc = NurbsCurve::create(false, 1, pts);
                if (pc.is_valid()) { out[surf].push_back(pc); ++n; }
            }
            return n;
        };
        int na = emit_for(true,  p1c_onA, (int)m_surfaces.size());
        int nb = emit_for(false, p1c_onB, (int)other.m_surfaces.size());
        if (std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[SYMEMIT] missing-section imprints: A+=%d B+=%d\n", na, nb);
        if (na + nb > 0 && na + nb <= MAX_TOTAL * 2) run_splits();
        p1c_onA.clear();
        p1c_onB.clear();
    }
    if (use_scaffold && std::getenv("SESSION_NT_DBG")) {
        auto dump_segs = [&](const char* tag, const std::map<int, std::array<double, 3>>& sp) {
            std::set<int> ss;
            for (const auto& kv : sp) ss.insert((int)kv.second[0]);
            std::printf("[SPANSEG] %s n=%zu:", tag, ss.size());
            for (int s2 : ss) std::printf(" %d", s2);
            std::printf("\n");
        };
        dump_segs("A@presym", spansA);
        dump_segs("B@presym", spansB);
        std::set<int> haveA;
        for (const auto& kv : spansA) haveA.insert((int)kv.second[0]);
        for (int s2 = 0; s2 < (int)scaf.segments.size(); ++s2) {
            if (haveA.count(s2)) continue;
            const SectionSegment& sg = scaf.segments[s2];
            std::printf("[SEGLOST] A seg=%d surfA=%d surfB=%d n=%zu closed=%d\n",
                        s2, sg.surfA, sg.surfB, sg.p3.size(), sg.v_start == sg.v_end ? 1 : 0);
        }
    }
    // P1c-DIRECT (SESSION_SYMLIFT): the CORRECT replacement for SYMEMIT. When one operand's arrangement
    // never emits a scaffold section segment the other has, DIRECTLY lift that segment's SHARED 3D chain
    // (s.p3) + its own uv (s.uvA/uvB) onto the missing operand's fragment as a section edge -- byte-
    // identical to the copy the other side already carries, so combine's whole-seg alias MATES them
    // (2-trim) and sew needs no tolerance. No re-split: SYMEMIT's pcurve-imprint+run_splits re-traced
    // the section through the SSI corrector, and on near-tangent (near-90-deg) pairs the near-singular
    // Jacobian slid to the WRONG BRANCH -- a spurious long arc (workflow-diagnosed z90 apex z=2.67) that
    // never mates. Direct lift avoids the re-trace entirely.
    if (use_scaffold && std::getenv("SESSION_SYMLIFT")) {
        std::set<int> segsA, segsB;
        for (const auto& kv : spansA) segsA.insert((int)kv.second[0]);
        for (const auto& kv : spansB) segsB.insert((int)kv.second[0]);
        auto lift_into = [&](BRep& X2, std::vector<int>& srcX, const BRep& Xorig,
                             std::map<int,std::array<double,3>>& spansX,
                             const std::set<int>& have, const std::set<int>& other_have, bool forA) -> int {
            int n = 0;
            for (int sid : other_have) {
                if (have.count(sid) || sid < 0 || sid >= (int)scaf.segments.size()) continue;
                const SectionSegment& s = scaf.segments[sid];
                int surf = forA ? s.surfA : s.surfB;
                const std::vector<Point>& uv = forA ? s.uvA : s.uvB;
                if (surf < 0 || (int)s.p3.size() < 2 || uv.size() != s.p3.size()) continue;
                Point miduv = uv[uv.size()/2];
                // pick the X2 fragment on original surface `surf` closest (in uv) to the section midpoint
                int F = -1; double bestd = 1e300;
                for (int fi = 0; fi < (int)X2.m_faces.size() && fi < (int)srcX.size(); ++fi) {
                    int of = srcX[fi];
                    if (of < 0 || of >= (int)Xorig.m_faces.size()) continue;
                    if (Xorig.m_faces[of].surface_index != surf) continue;
                    double d = 0; int nc = 0;
                    for (int li : X2.m_faces[fi].loop_indices)
                        for (int ti : X2.m_loops[li].trim_indices) {
                            int c2 = X2.m_trims[ti].curve_2d_index;
                            if (c2 < 0 || c2 >= (int)X2.m_curves_2d.size()) continue;
                            auto dd = X2.m_curves_2d[c2].domain();
                            Point q = X2.m_curves_2d[c2].point_at(0.5*(dd.first+dd.second));
                            d += std::hypot(q[0]-miduv[0], q[1]-miduv[1]); ++nc;
                        }
                    if (nc > 0) { d /= nc; if (d < bestd) { bestd = d; F = fi; } }
                }
                if (F < 0 || X2.m_faces[F].loop_indices.empty()) continue;
                int li = X2.m_faces[F].loop_indices[0];
                std::vector<int> ltr = X2.m_loops[li].trim_indices;
                if ((int)ltr.size() < 2) continue;
                // PROPER fragment-split (SESSION_SYMLIFT): divide F by the lifted section edge into two
                // regions (2-trim by construction), NOT an interior slit (which corrupts F's wire and
                // yields BadOrientation). Anchor the edge on F's own boundary vertices nearest the
                // section endpoints; if none within tolerance, SKIP (never slit).
                auto entryv = [&](int ti){ const BRepTrim& T = X2.m_trims[ti];
                    const BRepEdge& ed = X2.m_topology_edges[T.edge_index];
                    return T.reversed ? ed.end_vertex : ed.start_vertex; };
                auto vpos = [&](int v){ return X2.m_vertices[X2.m_topology_vertices[v].point_index]; };
                int ka = -1, kb = -1; double da = 1e300, db = 1e300;
                for (int k = 0; k < (int)ltr.size(); ++k) {
                    int ev = entryv(ltr[k]); if (ev < 0) continue;
                    double d0 = vpos(ev).distance(s.p3.front()), d1 = vpos(ev).distance(s.p3.back());
                    if (d0 < da) { da = d0; ka = k; }
                    if (d1 < db) { db = d1; kb = k; }
                }
                double stol = 0.06;   // junctions are shared scaffold verts -> should be near-exact
                if (ka < 0 || kb < 0 || ka == kb || da > stol || db > stol) continue;   // skip, no slit
                int Va = entryv(ltr[ka]), Vb = entryv(ltr[kb]);
                NurbsCurve c3 = NurbsCurve::create(false, 1, s.p3);
                if (!c3.is_valid()) continue;
                std::vector<Point> uvp; for (const auto& q : uv) uvp.push_back(Point(q[0],q[1],0.0));
                // Snap E's pcurve ENDS to the F-loop vertices' uv (the arc endpoints) so the two new
                // wires are continuous in uv (else a small gap -> OCCT Unorientable/BadOrientation).
                auto trim_entry_uv = [&](int ti) {
                    const BRepTrim& T = X2.m_trims[ti];
                    const NurbsCurve& pcc = X2.m_curves_2d[T.curve_2d_index];
                    auto d = pcc.domain();
                    return pcc.point_at(T.reversed ? d.second : d.first);
                };
                Point va_uv = trim_entry_uv(ltr[ka]), vb_uv = trim_entry_uv(ltr[kb]);
                uvp.front() = Point(va_uv[0], va_uv[1], 0.0);
                uvp.back()  = Point(vb_uv[0], vb_uv[1], 0.0);
                NurbsCurve pc = NurbsCurve::create(false, 1, uvp);
                if (!pc.is_valid()) continue;
                int nl = (int)ltr.size();
                std::vector<int> arc1, arc2;
                for (int k = ka; k != kb; k = (k+1)%nl) arc1.push_back(ltr[k]);
                for (int k = kb; k != ka; k = (k+1)%nl) arc2.push_back(ltr[k]);
                if (arc1.empty() || arc2.empty()) continue;
                int ci = X2.add_curve_3d(c3);
                int E = X2.add_edge(ci, Va, Vb);
                // F1 reuses F: loop li = arc1 + E traversed Vb->Va (reversed pcurve)
                int c2a = X2.add_curve_2d(pc);
                int t1 = X2.add_trim(c2a, E, li, true, BRepTrimType::Boundary);
                X2.m_loops[li].trim_indices = arc1; X2.m_loops[li].trim_indices.push_back(t1);
                // F2 = new face on the same surface: arc2 + E traversed Va->Vb
                int F2 = X2.add_face(X2.m_faces[F].surface_index, X2.m_faces[F].reversed);
                int li2 = X2.add_loop(F2, BRepLoopType::Outer);
                int c2b = X2.add_curve_2d(pc);
                int t2 = X2.add_trim(c2b, E, li2, false, BRepTrimType::Boundary);
                for (int ti : arc2) { X2.m_trims[ti].loop_index = li2; }
                X2.m_loops[li2].trim_indices = arc2; X2.m_loops[li2].trim_indices.push_back(t2);
                if ((int)srcX.size() <= F2) srcX.resize(F2 + 1, -1);
                srcX[F2] = srcX[F];   // same original face -> classify treats both fragments identically
                spansX[E] = {(double)sid, 0.0, (double)(s.p3.size()-1)};
                ++n;
            }
            return n;
        };
        int na = lift_into(A2, srcA_faces, *this, spansA, segsA, segsB, true);
        int nb = lift_into(B2, srcB_faces, other, spansB, segsB, segsA, false);
        if (std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[SYMLIFT] direct-lift A+=%d B+=%d\n", na, nb);
    }
    // PRESERVE-IDENTITY LOOP (OCCT re-compare-after-split / PostTreatFF analog).
    // A face's UV arrangement can node a section chain where the scaffold placed no pave;
    // the operand then emits a PARTIAL run, and since the two operands' arrangements never
    // agree on clip parameters, their partial copies have different endpoints and no shared
    // identity -- the defect no sewing tolerance can repair. Instead of reconstructing that
    // identity afterwards, feed the discovered breakpoints BACK into the shared scaffold,
    // split the segment there for BOTH operands at the same chain index (one welded vertex),
    // and re-split. Each pass strictly refines the pave set, so it converges; two passes are
    // allowed and the second is normally a no-op. After this, section runs span whole
    // segments and are keyed by segment identity -- shared by construction, not by distance.
    // MEASURED: OFF by default (SESSION_REFINE enables). The mechanism is correct and it
    // converges -- base chairs stay exact (35 faces / 46.7943) and it reports 0 new segments
    // on base, y30 and z30x20, i.e. it finds nothing to split. That is the finding: the
    // operands' arrangements are NOT noding section chains at un-paved interior points, so
    // there is no interior identity to recover. The staggering that FIX A repairs happens
    // within ~1% of a chord of the segment ENDS (stub clamps and trim snapping), below this
    // pass's resolution. Where it did fire (z90 cut: 1 then 2 new segments) it cost 2 naked
    // edges (9 -> 11), because re-splitting perturbs the arrangement that produced the
    // previously-mated runs. Kept, gated, because it is the right structure for inputs whose
    // section chains genuinely cross un-paved -- and because the negative result is evidence.
    if (use_scaffold && (std::getenv("SESSION_REFINE") || use_bop2)) {
        for (int pass = 0; pass < 2; ++pass) {
            std::map<int, std::vector<double>> brk;
            auto collect = [&](const std::map<int, std::array<double, 3>>& sp) {
                for (const auto& kv : sp) {
                    auto& v = brk[(int)kv.second[0]];
                    v.push_back(kv.second[1]);
                    v.push_back(kv.second[2]);
                }
            };
            collect(spansA);
            collect(spansB);
            int nnew = refine_scaffold_at_breaks(scaf, brk);
            if (std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[REFINE] pass %d: %d new shared segments (total %zu)\n",
                             pass, nnew, scaf.segments.size());
            if (nnew == 0) break;
            run_splits();
            lap("refine_split");
        }
        // Phase 4 step 1: after refinement every arrangement-discovered breakpoint is a
        // SEGMENT boundary, so the pool's one-edge-per-segment IS one-edge-per-block.
        if (use_bop2) {
            bop2_pool = build_shared_edge_pool(scaf);
            std::fprintf(stderr, "[BOP2] pool rebuilt post-refine: edges=%zu segs=%zu\n",
                         bop2_pool.arena.m_topology_edges.size(), scaf.segments.size());
        }
    }
    // Phase 4 step 4a (MakeSplitEdges on the OPERAND RESULTS): split every operand edge
    // -- mated ones included -- at on-edge pool vertices, so a section running along the
    // other operand's boundary meets sub-edges with POOL-vertex anchors (the middle piece
    // then pairs with the section edge at combine instead of forming a 3-face curve).
    if (use_bop2 && !std::getenv("SESSION_NO_P4SPLIT")) {
        A2.imprint_edges(scaf.tol3 * 0.7, true);
        B2.imprint_edges(scaf.tol3 * 0.7, true);
        lap("p4_imprint");
    }
    if (use_scaffold && !s_no_blocks) {
        // P1b (SESSION_SHAREDPAVE): pave BOTH operands at ONE shared pave set per segment, clustered
        // here from the union of A's and B's range-ends, so their block edges land on identical
        // [fa,fb] intervals and the combine's qspan alias mates them 2-trim by construction. Measured
        // root cause of the naked residual: normalize runs per-operand (spansA vs spansB), so each
        // paves at its own crossings; when A and B cross a segment at slightly different params their
        // blocks stagger and never alias -> naked section loops (z90: all runs partial, whole=0).
        std::map<int, std::vector<double>> shared_centers;
        if (std::getenv("SESSION_SHAREDPAVE")) {
            const double eps_f = 0.03;
            std::map<int, std::vector<double>> vals_by_seg;
            auto gather = [&](const std::map<int, std::array<double, 3>>& sp) {
                for (const auto& kv : sp) {
                    int sid = (int)kv.second[0];
                    if (sid < 0 || sid >= (int)scaf.segments.size()) continue;
                    vals_by_seg[sid].push_back(kv.second[1]);
                    vals_by_seg[sid].push_back(kv.second[2]);
                }
            };
            gather(spansA);
            gather(spansB);
            for (auto& kv : vals_by_seg) {
                int sid = kv.first;
                int nCh = (int)scaf.segments[sid].p3.size();
                if (nCh < 2) continue;
                auto& vals = kv.second;
                vals.push_back(0.0);
                vals.push_back((double)(nCh - 1));
                std::sort(vals.begin(), vals.end());
                std::vector<double>& centers = shared_centers[sid];
                double lo = vals[0], hi = vals[0], sum = vals[0]; int cnt = 1;
                auto flush = [&]() {
                    double c = sum / std::max(cnt, 1);
                    if (lo <= 1e-9) c = 0.0;
                    if (hi >= nCh - 1 - 1e-9) c = (double)(nCh - 1);
                    centers.push_back(c);
                };
                for (size_t i = 1; i < vals.size(); ++i) {
                    if (vals[i] - hi > eps_f) { flush(); lo = vals[i]; sum = 0.0; cnt = 0; }
                    hi = vals[i]; sum += vals[i]; ++cnt;
                }
                flush();
            }
        }
        const std::map<int, std::vector<double>>* shp = shared_centers.empty() ? nullptr : &shared_centers;
        // Safety net for anything the refinement could not pave (closed/full-wrap segments).
        B2.normalize_section_blocks(scaf, spansB, &secB_edges, "B", shp);
        A2.normalize_section_blocks(scaf, spansA, &secA_edges, "A", shp);
    }
    if (std::getenv("SESSION_NT_DBG")) {
        auto cnt1 = [](const BRep& X) { int c = 0; for (const auto& e : X.m_topology_edges) if (e.trim_indices.size() == 1) ++c; return c; };
        std::printf("[NT] A2 edges1=%d/%d  B2 edges1=%d/%d\n",
                    cnt1(A2), (int)A2.m_topology_edges.size(), cnt1(B2), (int)B2.m_topology_edges.size());
    }
    if (const char* dd = std::getenv("SESSION_DUMP_SPLITS")) {
        A2.pb_dump(std::string(dd) + "/split_A2.pb");
        B2.pb_dump(std::string(dd) + "/split_B2.pb");
    }
    // Classify fragments against the OTHER solid. The operands are typically recognized
    // primitives (box/beam/cylinder) -> test point-in-solid analytically in O(1) and skip the
    // tessellate+ray-cast entirely. Only build a mesh for an operand we could not recognize.
    PrimSolid primA = prA0;
    PrimSolid primB = prB0;
    Mesh meshA, meshB;
    if (primA.kind == 0) meshA = mesh();
    if (primB.kind == 0) meshB = other.mesh();                 lap("recognize+mesh");

    BRep result;
    result.name = "boolean";

    // A 3D point strictly inside face fi of X (CDT the UV trim region, centroid of a triangle).
    auto face_sample = [&](const BRep& X, int fi, double* ou = nullptr, double* ov = nullptr) -> Point {
        const auto& face = X.m_faces[fi];
        const NurbsSurface& srf = X.m_surfaces[face.surface_index];
        std::vector<Polyline> outers, inners;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            std::vector<Point> pts;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int c2 = X.m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                const NurbsCurve& pc = X.m_curves_2d[c2];
                auto dc = pc.domain();
                int n = std::min(std::max(pc.cv_count() * 3, 6), 1024);
                for (int i = 0; i < n; ++i) {
                    Point uv = pc.point_at(dc.first + (dc.second - dc.first) * i / n);
                    pts.push_back(Point(uv[0], uv[1], 0));
                }
            }
            if (pts.size() < 3) continue;
            if (X.m_loops[li].type == BRepLoopType::Outer) outers.emplace_back(pts);
            else inners.emplace_back(pts);
        }
        // SYMMETRIC-PROBE HAZARD (OCCT IntermediatePoint doctrine, PAR_T = 0.43213918).
        // A probe at a SYMMETRIC location -- domain centre, midpoint, equal-weight centroid --
        // lands exactly on a symmetry plane, seam or shared edge in symmetric geometry, where
        // the quantity being probed is UNDEFINED rather than merely small (distances tie at 0,
        // normals come out perpendicular). Our whole corpus is symmetric: boxes centred on the
        // origin, coaxial cylinders, spheres at the origin, A-op-A. OCCT picks an
        // irrational-looking interior parameter precisely so this cannot happen. Two different
        // constants are used for u and v so a square face's DIAGONAL symmetry is broken too.
        static const bool s_probe_fix = (std::getenv("SESSION_PROBE_FIX") != nullptr);
        const double PAR_T  = 0.43213918;   // OCCT IntermediatePoint
        const double PAR_T2 = 0.61803399;   // golden-ratio conjugate, != PAR_T and != 1-PAR_T
        auto fallback = [&]() {
            auto [u0,u1] = srf.domain(0); auto [v0,v1] = srf.domain(1);
            double fu = s_probe_fix ? PAR_T  : 0.5;
            double fv = s_probe_fix ? PAR_T2 : 0.5;
            double uu = u0 + (u1-u0)*fu, vv = v0 + (v1-v0)*fv;
            if (ou) *ou = uu; if (ov) *ov = vv;
            return srf.point_at(uu, vv);
        };
        if (outers.empty()) return fallback();
        // WIRE-AWARE PROBE. A face may carry more than one outer wire (a seam-merged region
        // has one wire per seam side, exactly like the untrimmed primitive sphere face).
        // Triangulate the LARGEST outer wire together with only the inner wires that lie
        // inside it -- feeding the CDT a hole that sits outside its outer polygon produces a
        // meaningless probe point. With a single outer wire this picks outers[0] and every
        // inner wire, i.e. it is a no-op on the default path.
        auto bbox_of = [](const Polyline& pl) {
            std::array<double,4> b = {1e300, -1e300, 1e300, -1e300};
            for (const auto& p : pl.get_points()) {
                b[0] = std::min(b[0], p[0]); b[1] = std::max(b[1], p[0]);
                b[2] = std::min(b[2], p[1]); b[3] = std::max(b[3], p[1]);
            }
            return b;
        };
        size_t obest = 0;
        if (outers.size() > 1) {
            double barea = -1.0;
            for (size_t k = 0; k < outers.size(); ++k) {
                auto b = bbox_of(outers[k]);
                double ar = (b[1]-b[0]) * (b[3]-b[2]);
                if (ar > barea) { barea = ar; obest = k; }
            }
        }
        std::vector<Polyline> all; all.push_back(outers[obest]);
        {
            auto B = bbox_of(outers[obest]);
            for (auto& in : inners) {
                if (outers.size() > 1) {
                    auto b = bbox_of(in);
                    if (b[0] < B[0] || b[1] > B[1] || b[2] < B[2] || b[3] > B[3]) continue;
                }
                all.push_back(in);
            }
        }
        std::vector<std::array<int,3>> tris;
        try { tris = RemeshCDT::triangulate(all); } catch (...) { return fallback(); }
        std::vector<Point> flat;
        for (auto& pl : all) { auto pp = pl.get_points(); for (auto& p : pp) flat.push_back(p); }
        if (tris.empty() || flat.empty()) return fallback();
        // Sample the centroid of the LARGEST triangle: it sits well inside the trim region (the
        // first triangle is often a thin sliver against the boundary, whose centroid grazes the
        // other solid's surface and misclassifies).
        int best = -1; double best_area = -1.0;
        for (size_t ti = 0; ti < tris.size(); ++ti) {
            const auto& t = tris[ti];
            if (t[0] < 0 || t[2] >= (int)flat.size()) continue;
            double ar = std::abs((flat[t[1]][0]-flat[t[0]][0])*(flat[t[2]][1]-flat[t[0]][1])
                               - (flat[t[2]][0]-flat[t[0]][0])*(flat[t[1]][1]-flat[t[0]][1])) * 0.5;
            if (ar > best_area) { best_area = ar; best = (int)ti; }
        }
        if (best < 0) return fallback();
        const auto& t = tris[best];
        // Equal-weight (1/3,1/3,1/3) is a SYMMETRIC probe: on an isoceles/equilateral triangle
        // straddling a symmetry plane the centroid sits exactly ON it. Asymmetric barycentric
        // weights (summing to 1, all strictly interior) keep the point well inside the triangle
        // while making a symmetry landing impossible.
        double w0 = s_probe_fix ? 0.43213918 : (1.0/3.0);
        double w1 = s_probe_fix ? 0.34589803 : (1.0/3.0);
        double w2 = s_probe_fix ? (1.0 - 0.43213918 - 0.34589803) : (1.0/3.0);
        double cu = w0*flat[t[0]][0] + w1*flat[t[1]][0] + w2*flat[t[2]][0];
        double cv = w0*flat[t[0]][1] + w1*flat[t[1]][1] + w2*flat[t[2]][1];
        if (ou) *ou = cu; if (ov) *ov = cv;
        return srf.point_at(cu, cv);
    };

    // K interior samples of face fi (centroids of the K largest CDT triangles): a single
    // sample answers wrong on ~14% of tessellated-freeform ray casts (r22 bucket 3), and
    // one bad answer drops a whole fragment -- majority over well-separated interior
    // points is the cheap robust version of OCCT's block classification.
    auto face_samples = [&](const BRep& X, int fi, int K) -> std::vector<Point> {
        const auto& face = X.m_faces[fi];
        const NurbsSurface& srf = X.m_surfaces[face.surface_index];
        std::vector<Polyline> outers, inners;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            std::vector<Point> pts;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int c2 = X.m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                const NurbsCurve& pc = X.m_curves_2d[c2];
                auto dc = pc.domain();
                int n = std::min(std::max(pc.cv_count() * 3, 6), 1024);
                for (int i = 0; i < n; ++i) {
                    Point uv = pc.point_at(dc.first + (dc.second - dc.first) * i / n);
                    pts.push_back(Point(uv[0], uv[1], 0));
                }
            }
            if (pts.size() < 3) continue;
            if (X.m_loops[li].type == BRepLoopType::Outer) outers.emplace_back(pts);
            else inners.emplace_back(pts);
        }
        if (outers.empty()) return {};
        std::vector<Polyline> all; all.push_back(outers[0]);
        for (auto& in : inners) all.push_back(in);
        std::vector<std::array<int,3>> tris;
        try { tris = RemeshCDT::triangulate(all); } catch (...) { return {}; }
        std::vector<Point> flat;
        for (auto& pl : all) { auto pp = pl.get_points(); for (auto& p : pp) flat.push_back(p); }
        if (tris.empty() || flat.empty()) return {};
        std::vector<std::pair<double, size_t>> by_area;
        for (size_t ti = 0; ti < tris.size(); ++ti) {
            const auto& t = tris[ti];
            if (t[0] < 0 || t[2] >= (int)flat.size()) continue;
            double ar = std::abs((flat[t[1]][0]-flat[t[0]][0])*(flat[t[2]][1]-flat[t[0]][1])
                               - (flat[t[2]][0]-flat[t[0]][0])*(flat[t[1]][1]-flat[t[0]][1])) * 0.5;
            by_area.push_back({ar, ti});
        }
        std::sort(by_area.begin(), by_area.end(), [](auto& a, auto& b) { return a.first > b.first; });
        std::vector<Point> out;
        // same symmetric-probe hazard as face_sample: equal weights land on symmetry planes
        static const bool s_probe_fix2 = (std::getenv("SESSION_PROBE_FIX") != nullptr);
        for (size_t k = 0; k < by_area.size() && (int)out.size() < K; ++k) {
            const auto& t = tris[by_area[k].second];
            double w0 = s_probe_fix2 ? 0.43213918 : (1.0/3.0);
            double w1 = s_probe_fix2 ? 0.34589803 : (1.0/3.0);
            double w2 = s_probe_fix2 ? (1.0 - 0.43213918 - 0.34589803) : (1.0/3.0);
            double cu = w0*flat[t[0]][0] + w1*flat[t[1]][0] + w2*flat[t[2]][0];
            double cv = w0*flat[t[0]][1] + w1*flat[t[1]][1] + w2*flat[t[2]][1];
            out.push_back(srf.point_at(cu, cv));
        }
        return out;
    };

    // Classify each imprinted fragment inside/outside the other solid, select per op.
    // Same-domain (ON) pieces -- a fragment lying ON the other solid's boundary, e.g. a
    // coplanar cone base on a box face -- follow OCCT's BOPAlgo_BOP rules: the pair decision
    // is made once on the A side (FUSE/COMMON keep A's copy iff the outward normals agree,
    // CUT keeps A's copy iff they oppose) and the B copy is always dropped. Detection is by
    // probing containment a step along the face normal to BOTH sides: a regular fragment
    // answers the same on both sides, an ON fragment straddles.
    double on_eps;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& q : m_vertices) { xmn=std::min(xmn,q[0]); ymn=std::min(ymn,q[1]); zmn=std::min(zmn,q[2]);
            xmx=std::max(xmx,q[0]); ymx=std::max(ymx,q[1]); zmx=std::max(zmx,q[2]); }
        double dg = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        on_eps = (dg > 0 ? dg : 1.0) * 2e-3;
    }
    std::vector<int> keptA, keptB; std::vector<bool> revB;
    // Outward orientation of the ORIGINAL operands (fragments inherit via face_src):
    // required by the angle method below. Computed geometrically, never from flags.
    std::vector<double> osignA, osignB;
    if (use_scaffold) { osignA = face_outward_signs(); osignB = other.face_outward_signs(); }
    auto classify = [&](const BRep& X2, const BRep& solid, const Mesh& solid_mesh,
                        const PrimSolid& prim, const BRep& own, const Mesh& own_mesh,
                        const PrimSolid& own_prim, bool is_first,
                        std::vector<int>& kept, std::vector<bool>* rev,
                        const std::map<int, std::array<int, 3>>* sec_edges,
                        const std::vector<int>* src_faces,
                        const std::vector<double>* osign_own,
                        const std::vector<double>* osign_oth,
                        const std::map<int, int>* radial = nullptr) {
        // For imported-freeform operands the boundary MESH can self-overlap (a rotated chair
        // patch inverts), making the winding-number/ray classifiers over- or under-count the
        // solid angle and answer wrong by a full enclosure (chairsROT z90: f2 read inside 0.8
        // OUTSIDE B, f40 read outside 0.1 INSIDE B). The mesh-free exact classifier -- closest
        // point on the TRIMMED boundary + outward-normal sign -- is correct there. Primitives
        // keep the analytic inside_prim path, so the matrix/edge batteries are untouched.
        static const bool s_no_exact = (std::getenv("SESSION_NO_EXACT_PIP") != nullptr);
        static const std::vector<double> s_empty_osign;
        auto in_other = [&](const Point& q) {
            if (prim.kind) return inside_prim(prim, q, prim.tol);
            if (!s_no_exact) return solid.contains_point_exact(q, osign_oth ? *osign_oth : s_empty_osign);
            return solid.contains_point(solid_mesh, q);
        };
        auto in_own = [&](const Point& q) {
            if (own_prim.kind) return inside_prim(own_prim, q, own_prim.tol);
            if (!s_no_exact) return own.contains_point_exact(q, osign_own ? *osign_own : s_empty_osign);
            return own.contains_point(own_mesh, q);
        };
        int nf = (int)X2.m_faces.size();
        std::vector<int> is_on(nf, 0), inside_v(nf, 0);
        std::vector<double> score(nf, -1.0);   // fraction of samples inside (non-ON only)
        std::vector<bool> keep_v(nf, false), rev_v(nf, false), valid(nf, false);
        // Phase-5 radial certification: fragments whose in/out verdict was derived EXACTLY
        // at a mated section block (material-side test at the shared curve). Certified
        // verdicts are never overridden by sampling, angle stations, block majorities,
        // parity propagation, or repair flips -- they SEED those stages instead.
        std::vector<char> certified(nf, 0);
        // majority sampling for imported freeform (contains_point on a tessellated mesh is
        // ~14% wrong per ray cluster); primitives keep the proven single-sample path
        int K = (use_scaffold && prim.kind == 0) ? 5 : 1;
        auto vote_inside = [&](int fi, int k, double* sc) -> bool {
            if (k <= 1) {
                Point sp = face_sample(X2, fi);
                bool ins = in_other(sp);
                if (sc) *sc = ins ? 1.0 : 0.0;
                return ins;
            }
            std::vector<Point> sps = face_samples(X2, fi, k);
            if (sps.empty()) sps.push_back(face_sample(X2, fi));
            int nin = 0;
            for (const auto& q : sps) if (in_other(q)) ++nin;
            if (sc) *sc = (double)nin / (double)sps.size();
            return nin * 2 > (int)sps.size();
        };
        std::vector<Point> fsample(nf, Point(0, 0, 0));   // for the [CLSF] final-verdict audit
        // MICRO-FRAGMENT COLLAPSE (OCCT micro pave-block analog): a fragment whose whole
        // 3D extent fits inside one pave-tolerance ball is a spurious sliver minted by a
        // near-tangent arrangement crossing (the z30x20 class). It cannot be classified
        // (no interior beyond tolerance) and puts a THIRD trim on its section edges ->
        // nonmanifold. Drop it at the source; its micro perimeter edges die with it.
        std::vector<char> micro(nf, 0);
        if (use_scaffold && !std::getenv("SESSION_NO_MICROFRAG")) {
            double md = (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * 2.0;
            int nmicro = 0;
            for (int fi = 0; fi < nf; ++fi) {
                const auto& F = X2.m_faces[fi];
                if (F.surface_index < 0) continue;
                double bmn[3] = {1e300,1e300,1e300}, bmx[3] = {-1e300,-1e300,-1e300};
                double emax = 0;                       // longest perimeter edge
                bool any = false;
                for (int li : F.loop_indices) {
                    if (li < 0 || li >= (int)X2.m_loops.size()) continue;
                    for (int ti : X2.m_loops[li].trim_indices) {
                        if (ti < 0 || ti >= (int)X2.m_trims.size()) continue;
                        int ei = X2.m_trims[ti].edge_index;
                        if (ei < 0 || ei >= (int)X2.m_topology_edges.size()) continue;
                        const auto& E = X2.m_topology_edges[ei];
                        if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)X2.m_curves_3d.size()) continue;
                        const NurbsCurve& c = X2.m_curves_3d[E.curve_3d_index];
                        auto d = c.domain();
                        double elen = 0;
                        Point pv = c.point_at(d.first);
                        for (int k = 0; k <= 4; ++k) {
                            Point q = c.point_at(d.first + (d.second - d.first) * k / 4.0);
                            for (int x = 0; x < 3; ++x) { bmn[x] = std::min(bmn[x], q[x]); bmx[x] = std::max(bmx[x], q[x]); }
                            if (k > 0) elen += pv.distance(q);
                            pv = q;
                            any = true;
                        }
                        emax = std::max(emax, elen);
                    }
                }
                if (!any) continue;
                double dg2 = (bmx[0]-bmn[0])*(bmx[0]-bmn[0]) + (bmx[1]-bmn[1])*(bmx[1]-bmn[1]) +
                             (bmx[2]-bmn[2])*(bmx[2]-bmn[2]);
                // a droppable sliver is a BLOB: tiny extent AND no long perimeter edge whose
                // mate would go naked (z37: real small fragments have 1-3 unit edges)
                if (dg2 < md * md && emax < md) { micro[fi] = 1; ++nmicro; }
            }
            if (nmicro && std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[MICROFRAG] %c dropped=%d tol=%.4g\n", is_first ? 'A' : 'B', nmicro, md);
        }
        for (int fi = 0; fi < nf; ++fi) {
            if (X2.m_faces[fi].surface_index < 0) continue;
            if (micro[fi]) continue;                       // sliver: stays invalid, never kept
            valid[fi] = true;
            // SD-PAIRED WALL: exact identity (edge-sig bucketing at tol3_rep), decided
            // once per PAIR by the verified BuildBOP table. Takes precedence over every
            // probe below -- probes coin-flip exactly here (both boundaries within
            // on_eps; GZ-J flush: A wall read in_p==in_m==1, ON never fired).
            {
                bool sd_hit = false, sd_orient = false;
                const std::map<int, bool>& sdmap = is_first ? sd_pairA : sd_pairB;
                auto sdit = sdmap.find(fi);
                if (sdit != sdmap.end()) {
                    sd_hit = true; sd_orient = sdit->second;
                } else {
                    // Pre-split pairs are keyed by ORIGINAL face; fragments inherit
                    // through src_faces (whole faces when the SSI was suppressed).
                    const std::map<int, bool>& pmap = is_first ? pre_pairA : pre_pairB;
                    const int src = (src_faces && fi < (int)src_faces->size())
                                        ? (*src_faces)[fi] : fi;
                    auto pit = pmap.find(src);
                    if (pit != pmap.end()) { sd_hit = true; sd_orient = pit->second; }
                }
                if (sd_hit) {
                    is_on[fi] = 1;
                    const SDOp sop = op == BooleanOp::Union ? SDOp::Fuse
                                   : op == BooleanOp::Intersection ? SDOp::Common
                                                                   : SDOp::Cut;
                    const SDVerdict v = sd_select_sd_face(sop, is_first ? 0 : 1,
                                                          sd_orient);
                    keep_v[fi] = v != SDVerdict::Drop;
                    if (std::getenv("SESSION_CLS_DBG"))
                        std::printf("[CLS] %c f%d SD-PAIR orient_%s keep=%d\n",
                                    is_first ? 'A' : 'B', fi,
                                    sd_orient ? "same" : "opp", keep_v[fi] ? 1 : 0);
                    continue;
                }
            }
            double cu = 0, cv2 = 0;
            Point sp = face_sample(X2, fi, &cu, &cv2);
            fsample[fi] = sp;
            Vector nr = X2.m_surfaces[X2.m_faces[fi].surface_index].normal_at(cu, cv2);
            double nl = nr.magnitude();
            Point pp2(sp[0]+on_eps*nr[0]/(nl>1e-12?nl:1.0), sp[1]+on_eps*nr[1]/(nl>1e-12?nl:1.0), sp[2]+on_eps*nr[2]/(nl>1e-12?nl:1.0));
            Point pm2(sp[0]-on_eps*nr[0]/(nl>1e-12?nl:1.0), sp[1]-on_eps*nr[1]/(nl>1e-12?nl:1.0), sp[2]-on_eps*nr[2]/(nl>1e-12?nl:1.0));
            bool in_p = nl > 1e-12 && in_other(pp2);
            bool in_m = nl > 1e-12 && in_other(pm2);
            bool on_now = nl > 1e-12 && in_p != in_m;
            if (std::getenv("SESSION_NO_ON")) on_now = false;   // experiment: treat ON as regular in/out
            if (on_now && use_scaffold) {
                // Coincidence must be SCALE-INVARIANT: a genuinely ON fragment straddles at
                // any probe eps, while mere proximity (nested chairs pass within on_eps of
                // each other) stops straddling once eps < gap. A false ON silently drops an
                // OUTSIDE fragment for fuse -- the exact footprint of the chair-fuse hole.
                double e2 = on_eps / 16.0;
                Point pp3(sp[0]+e2*nr[0]/nl, sp[1]+e2*nr[1]/nl, sp[2]+e2*nr[2]/nl);
                Point pm3(sp[0]-e2*nr[0]/nl, sp[1]-e2*nr[1]/nl, sp[2]-e2*nr[2]/nl);
                bool in_p3 = in_other(pp3), in_m3 = in_other(pm3);
                if (!(in_p3 != in_m3 && in_p3 == in_p)) on_now = false;   // proximity artifact
            }
            if (on_now) {
                // ON piece: orient nr outward via the OWN solid, then compare with the side
                // the other solid occupies.
                is_on[fi] = 1;
                bool own_p = in_own(pp2);
                bool same_orient = own_p ? in_p : in_m;   // other occupies this face's inner side
                if (op == BooleanOp::Union)             keep_v[fi] = is_first && same_orient;
                else if (op == BooleanOp::Intersection) keep_v[fi] = is_first && same_orient;
                else                                    keep_v[fi] = is_first && !same_orient;
            } else {
                if (radial && radial->count(fi)) {
                    inside_v[fi] = radial->at(fi);
                    score[fi] = inside_v[fi] ? 1.0 : 0.0;
                    certified[fi] = 1;
                } else {
                    inside_v[fi] = vote_inside(fi, K, &score[fi]) ? 1 : 0;
                }
            }
            if (std::getenv("SESSION_CLS_DBG"))
                std::printf("[CLS] %c f%d sp(%.3f,%.3f,%.3f) on=%d in_p=%d in_m=%d ins=%d sc=%.2f\n",
                            is_first ? 'A' : 'B', fi, sp[0], sp[1], sp[2],
                            is_on[fi], in_p ? 1 : 0, in_m ? 1 : 0, inside_v[fi], score[fi]);
        }
        // ANGLE METHOD (OCCT BOPTools_AlgoTools::IsInternalFace / GetFaceOff analog):
        // for a fragment ADJACENT to a section edge, inside/outside the other solid is
        // LOCAL GEOMETRY -- the fragment's in-surface direction away from the section
        // either dives under the other surface (inside) or lifts off it (outside).
        // Deterministic, no ray casts; overrides the sampled verdict near sections,
        // which is exactly where tessellated-mesh rays are least reliable.
        std::vector<int> angle_n(nf, 0), angle_in(nf, 0);
        std::vector<double> ang_sum(nf, 0.0), ang_abs(nf, 0.0);   // signed / total evidence
        static const bool s_no_ang = (std::getenv("SESSION_NO_ANG") != nullptr);
        if (!s_no_ang && use_scaffold && sec_edges && src_faces && osign_own && osign_oth) {
            // other operand: original face per surface (imported breps: 1 face/surface)
            const BRep& oth = solid;   // 'solid' is the opposite operand in BOTH calls
            std::map<int, int> oth_face_of_surf;
            for (int f2 = 0; f2 < (int)oth.m_faces.size(); ++f2)
                if (oth.m_faces[f2].surface_index >= 0)
                    oth_face_of_surf.emplace(oth.m_faces[f2].surface_index, f2);
            auto frag_uv_polys = [&](int fi, std::vector<std::vector<std::array<double,2>>>& outer,
                                     std::vector<std::vector<std::array<double,2>>>& inner) {
                const auto& face = X2.m_faces[fi];
                for (int li : face.loop_indices) {
                    if (li < 0 || li >= (int)X2.m_loops.size()) continue;
                    std::vector<std::array<double,2>> pts;
                    for (int ti : X2.m_loops[li].trim_indices) {
                        if (ti < 0 || ti >= (int)X2.m_trims.size()) continue;
                        int c2 = X2.m_trims[ti].curve_2d_index;
                        if (c2 < 0 || c2 >= (int)X2.m_curves_2d.size()) continue;
                        const NurbsCurve& pc = X2.m_curves_2d[c2];
                        auto dc = pc.domain();
                        int n = std::min(std::max(pc.cv_count() * 2, 8), 256);
                        for (int i = 0; i < n; ++i) {
                            Point uv = pc.point_at(dc.first + (dc.second - dc.first) * i / n);
                            pts.push_back({uv[0], uv[1]});
                        }
                    }
                    if (pts.size() < 3) continue;
                    if (X2.m_loops[li].type == BRepLoopType::Outer) outer.push_back(std::move(pts));
                    else inner.push_back(std::move(pts));
                }
            };
            for (const auto& kv : *sec_edges) {
                int ei = kv.first, seg = kv.second[0];
                if (seg < 0 || seg >= (int)scaf.segments.size()) continue;
                if (ei < 0 || ei >= (int)X2.m_topology_edges.size()) continue;
                const auto& E = X2.m_topology_edges[ei];
                const SectionSegment& s = scaf.segments[seg];
                size_t n = s.p3.size();
                if (n < 2) continue;
                // Sample the flank test at THREE stations inside the edge's own block range
                // (a block covers a sub-range of the segment, so the segment midpoint may sit
                // on a different block entirely). Multi-station evidence is what makes the
                // verdict survive a locally noisy normal at a grazing crossing.
                std::vector<size_t> mstat;
                {
                    const auto& spans_own = is_first ? spansA : spansB;
                    auto itS = spans_own.find(ei);
                    double f0 = 0.0, f1 = (double)(n - 1);
                    if (itS != spans_own.end()) { f0 = itS->second[1]; f1 = itS->second[2]; }
                    if (f1 < f0) std::swap(f0, f1);
                    for (double fr : {0.25, 0.5, 0.75}) {
                        long long mi = (long long)std::llround(f0 + (f1 - f0) * fr);
                        mi = std::min<long long>((long long)n - 1, std::max<long long>(0, mi));
                        if (mstat.empty() || mstat.back() != (size_t)mi) mstat.push_back((size_t)mi);
                    }
                    if (mstat.empty()) mstat.push_back(n / 2);
                }
                const auto& uv_own = is_first ? s.uvA : s.uvB;
                const auto& uv_oth = is_first ? s.uvB : s.uvA;
                int surf_own = is_first ? s.surfA : s.surfB;
                int surf_oth = is_first ? s.surfB : s.surfA;
                const NurbsSurface& S_own = own.m_surfaces[surf_own];
                auto ito = oth_face_of_surf.find(surf_oth);
                if (ito == oth_face_of_surf.end()) continue;
                const NurbsSurface& S_oth = oth.m_surfaces[surf_oth];
                double so = (*osign_oth)[ito->second];
                // Fragment UV polygons once per flank (independent of the station).
                int fk[2] = {-1, -1};
                std::vector<std::vector<std::array<double,2>>> outer_k[2], inner_k[2];
                for (int k = 0; k < 2; ++k) {
                    int ti = k < (int)E.trim_indices.size() ? E.trim_indices[k] : -1;
                    int li = (ti >= 0 && ti < (int)X2.m_trims.size()) ? X2.m_trims[ti].loop_index : -1;
                    int fi = (li >= 0 && li < (int)X2.m_loops.size()) ? X2.m_loops[li].face_index : -1;
                    if (fi < 0 || fi >= nf || !valid[fi] || is_on[fi]) continue;
                    frag_uv_polys(fi, outer_k[k], inner_k[k]);
                    if (!outer_k[k].empty()) fk[k] = fi;
                }
                if (fk[0] < 0 && fk[1] < 0) continue;
                auto dom_u = S_own.domain(0); auto dom_v = S_own.domain(1);
                double du_rng = 1e-3 * std::min(dom_u.second - dom_u.first, dom_v.second - dom_v.first);
                for (size_t m : mstat) {
                    Vector n_oth = S_oth.normal_at(uv_oth[m][0], uv_oth[m][1]);
                    double nol = n_oth.magnitude();
                    if (nol < 1e-12) continue;
                    n_oth = Vector(n_oth[0]*so/nol, n_oth[1]*so/nol, n_oth[2]*so/nol);
                    double tu = uv_own[std::min(m+1, n-1)][0] - uv_own[m > 0 ? m-1 : 0][0];
                    double tv = uv_own[std::min(m+1, n-1)][1] - uv_own[m > 0 ? m-1 : 0][1];
                    double tl = std::hypot(tu, tv);
                    if (tl < 1e-30) continue;
                    tu /= tl; tv /= tl;
                    auto der = S_own.evaluate(uv_own[m][0], uv_own[m][1], 1);   // [S, Sv, Su]
                    for (int k = 0; k < 2; ++k) {
                        int fi = fk[k];
                        if (fi < 0) continue;
                        double wu = -tv, wv = tu;   // left perpendicular
                        int side = 0;
                        for (int attempt = 1; attempt <= 3 && side == 0; ++attempt) {
                            double d2 = du_rng * attempt;
                            bool inL = uv_in_polys(uv_own[m][0] + wu*d2, uv_own[m][1] + wv*d2, outer_k[k], inner_k[k]);
                            bool inR = uv_in_polys(uv_own[m][0] - wu*d2, uv_own[m][1] - wv*d2, outer_k[k], inner_k[k]);
                            if (inL != inR) side = inL ? +1 : -1;
                        }
                        if (side == 0) continue;
                        Vector Su = der[2], Sv = der[1];
                        Vector w3(Su[0]*wu*side + Sv[0]*wv*side,
                                  Su[1]*wu*side + Sv[1]*wv*side,
                                  Su[2]*wu*side + Sv[2]*wv*side);
                        double wl = w3.magnitude();
                        if (wl < 1e-12) continue;
                        double dp = (w3[0]*n_oth[0] + w3[1]*n_oth[1] + w3[2]*n_oth[2]) / wl;
                        // TRANSVERSALITY-WEIGHTED evidence (was: unweighted vote with a hard
                        // |dp|<0.02 tangential skip). At a GRAZING crossing every station is
                        // near-tangential, so the hard cutoff discarded the whole local
                        // verdict and handed the fragment back to winding-number sampling --
                        // which cannot resolve a sliver thinner than the tessellation sag
                        // (chairsROT x20/z15). The sign of dp is still meaningful there (both
                        // normals come from analytic derivatives); its MAGNITUDE is exactly
                        // the confidence, so accumulate signed evidence and let strongly
                        // transversal stations dominate. Only true numerical noise is skipped.
                        if (std::abs(dp) < 1e-6) continue;
                        ang_sum[fi] += (dp < 0.0 ? 1.0 : -1.0) * std::abs(dp);
                        ang_abs[fi] += std::abs(dp);
                        ++angle_n[fi];
                        if (dp < 0.0) ++angle_in[fi];
                        if (std::getenv("SESSION_CLS_DBG"))
                            std::printf("[ANGV] %c f%d seg=%d k=%d m=%zu side=%d dp=%+.4f m3(%.3f,%.3f,%.3f)\n",
                                        is_first ? 'A' : 'B', fi, seg, k, m, side, dp,
                                        s.p3[m][0], s.p3[m][1], s.p3[m][2]);
                    }
                }
            }
            for (int fi = 0; fi < nf; ++fi) {
                if (angle_n[fi] == 0 || ang_abs[fi] <= 0.0) continue;
                double conf = std::abs(ang_sum[fi]) / ang_abs[fi];   // 1 = unanimous
                if (conf < 0.2) continue;                            // genuinely contradictory: keep sampled
                int av = ang_sum[fi] > 0.0 ? 1 : 0;
                if (std::getenv("SESSION_CLS_DBG") && av != inside_v[fi])
                    std::printf("[CLS-ANG] %c f%d sampled=%d angle=%d (sum %+.4f abs %.4f conf %.2f n=%d)\n",
                                is_first ? 'A' : 'B', fi, inside_v[fi], av,
                                ang_sum[fi], ang_abs[fi], conf, angle_n[fi]);
                if (!certified[fi]) inside_v[fi] = av;
                // Confidence carries into the flood/parity seed choice: a unanimous strongly
                // transversal verdict is hard evidence, a marginal one must not anchor a block.
                score[fi] = av ? (0.5 + 0.5 * conf) : (0.5 - 0.5 * conf);
            }
        }
        // Section-edge consistency repair (OCCT block invariant): the two fragments
        // bordering one section edge lie on OPPOSITE sides of the other solid by
        // construction -- equal inside-verdicts mean one sample cluster lied. Re-vote
        // both at K=9; if still equal, flip the one whose majority was weakest.
        if (use_scaffold && sec_edges) {
            for (const auto& kv : *sec_edges) {
                int ei = kv.first;
                if (ei < 0 || ei >= (int)X2.m_topology_edges.size()) continue;
                const auto& E = X2.m_topology_edges[ei];
                if (E.trim_indices.size() != 2) continue;
                int fs[2] = {-1, -1};
                for (int k = 0; k < 2; ++k) {
                    int ti = E.trim_indices[k];
                    int li = (ti >= 0 && ti < (int)X2.m_trims.size()) ? X2.m_trims[ti].loop_index : -1;
                    fs[k] = (li >= 0 && li < (int)X2.m_loops.size()) ? X2.m_loops[li].face_index : -1;
                }
                if (fs[0] < 0 || fs[1] < 0 || fs[0] == fs[1]) continue;
                if (!valid[fs[0]] || !valid[fs[1]] || is_on[fs[0]] || is_on[fs[1]]) continue;
                if (certified[fs[0]] && certified[fs[1]]) continue;   // radial pair is exact
                if (inside_v[fs[0]] != inside_v[fs[1]]) continue;
                for (int k = 0; k < 2; ++k)
                    if (!certified[fs[k]]) inside_v[fs[k]] = vote_inside(fs[k], 9, &score[fs[k]]) ? 1 : 0;
                if (inside_v[fs[0]] == inside_v[fs[1]]) {
                    // flip the weaker majority to the section-consistent assignment
                    double c0 = std::abs(score[fs[0]] - 0.5), c1 = std::abs(score[fs[1]] - 0.5);
                    int flip = c0 < c1 ? fs[0] : fs[1];
                    if (certified[flip]) flip = flip == fs[0] ? fs[1] : fs[0];
                    if (certified[flip]) continue;
                    inside_v[flip] = 1 - inside_v[flip];
                    if (std::getenv("SESSION_CLS_DBG"))
                        std::printf("[CLS-FIX] %c seg-edge e%d f%d/f%d equal verdicts -> flip f%d\n",
                                    is_first ? 'A' : 'B', ei, fs[0], fs[1], flip);
                }
            }
        }
        // CONNEXITY-BLOCK FLOOD (OCCT ClassifyFaces analog): inside/outside may only
        // change across SECTION edges. Fragments connected through NON-section boundary
        // -- shared 2-trim edges within a face, or touching border pieces of ADJACENT
        // original faces -- form one block with ONE majority verdict. Without this,
        // rotated configs flip verdicts across face borders and orphan whole border
        // chains (the OWN naked class: 21/30 on chairsROT z15 cut).
        static const bool s_no_flood = (std::getenv("SESSION_NO_FLOOD") != nullptr);
        if (!s_no_flood && use_scaffold && sec_edges) {
            auto face_of = [&](int ti) -> int {
                if (ti < 0 || ti >= (int)X2.m_trims.size()) return -1;
                int li = X2.m_trims[ti].loop_index;
                return (li >= 0 && li < (int)X2.m_loops.size()) ? X2.m_loops[li].face_index : -1;
            };
            std::set<int> sec_set;
            for (const auto& kv : *sec_edges) sec_set.insert(kv.first);
            // any edge whose BOTH curve endpoints weld to scaffold vertices is a section
            // piece even if unkeyed (partial runs) -- it must SEPARATE blocks, not join
            auto near_scaf_vertex = [&](const Point& p) {
                for (const auto& v : scaf.vertices)
                    if (v.distance(p) < scaf.tol3 * 2.0) return true;
                return false;
            };
            auto is_section_edge = [&](int ei) {
                if (sec_set.count(ei)) return true;
                const auto& E = X2.m_topology_edges[ei];
                int ci = E.curve_3d_index;
                if (ci < 0 || ci >= (int)X2.m_curves_3d.size()) return false;
                const NurbsCurve& C = X2.m_curves_3d[ci];
                auto dc = C.domain();
                return near_scaf_vertex(C.point_at(dc.first)) && near_scaf_vertex(C.point_at(dc.second));
            };
            std::vector<int> uf(nf);
            for (int i2 = 0; i2 < nf; ++i2) uf[i2] = i2;
            std::function<int(int)> uf_find = [&](int a) {
                while (uf[a] != a) { uf[a] = uf[uf[a]]; a = uf[a]; }
                return a;
            };
            auto unite = [&](int a, int b) { a = uf_find(a); b = uf_find(b); if (a != b) uf[a] = b; };
            // (1) same-face fragments sharing a NON-section 2-trim edge
            struct BPc { int fi; std::vector<Point> smp; };
            std::vector<BPc> border;
            for (int ei = 0; ei < (int)X2.m_topology_edges.size(); ++ei) {
                const auto& E = X2.m_topology_edges[ei];
                if (is_section_edge(ei)) continue;
                if ((int)E.trim_indices.size() == 2) {
                    int f1 = face_of(E.trim_indices[0]), f2 = face_of(E.trim_indices[1]);
                    if (f1 >= 0 && f2 >= 0 && f1 != f2) unite(f1, f2);
                } else if ((int)E.trim_indices.size() == 1) {
                    int f1 = face_of(E.trim_indices[0]);
                    int ci = E.curve_3d_index;
                    if (f1 < 0 || ci < 0 || ci >= (int)X2.m_curves_3d.size()) continue;
                    const NurbsCurve& C = X2.m_curves_3d[ci];
                    auto dc = C.domain();
                    BPc bp; bp.fi = f1;
                    for (int k = 0; k <= 8; ++k)
                        bp.smp.push_back(C.point_at(dc.first + (dc.second - dc.first) * k / 8.0));
                    border.push_back(std::move(bp));
                }
            }
            // (2) cross-face: two fragments' border pieces touching in 3D (they carry
            // copies of the same ORIGINAL shared edge). Proximity alone over-unites:
            // NESTED walls of one chair run parallel 0.018 apart (< on_eps) for their
            // whole length -- indistinguishable from true adjacency by distance. Gate
            // on ORIGINAL topology: unite only fragments of ADJACENT original faces
            // (sharing a 2-trim edge in the operand), which nested parts never are.
            std::set<std::pair<int,int>> orig_adj;
            if (src_faces) {
                for (const auto& E0 : own.m_topology_edges) {
                    if ((int)E0.trim_indices.size() != 2) continue;
                    int of[2] = {-1, -1};
                    for (int k = 0; k < 2; ++k) {
                        int ti = E0.trim_indices[k];
                        if (ti < 0 || ti >= (int)own.m_trims.size()) continue;
                        int li = own.m_trims[ti].loop_index;
                        if (li < 0 || li >= (int)own.m_loops.size()) continue;
                        of[k] = own.m_loops[li].face_index;
                    }
                    if (of[0] >= 0 && of[1] >= 0 && of[0] != of[1])
                        orig_adj.insert({std::min(of[0], of[1]), std::max(of[0], of[1])});
                }
            }
            for (size_t i2 = 0; i2 < border.size(); ++i2)
                for (size_t j2 = i2 + 1; j2 < border.size(); ++j2) {
                    if (border[i2].fi == border[j2].fi) continue;
                    if (uf_find(border[i2].fi) == uf_find(border[j2].fi)) continue;
                    if (src_faces) {
                        int o1 = border[i2].fi < (int)src_faces->size() ? (*src_faces)[border[i2].fi] : -1;
                        int o2 = border[j2].fi < (int)src_faces->size() ? (*src_faces)[border[j2].fi] : -1;
                        if (o1 < 0 || o2 < 0) continue;
                        if (o1 != o2 && !orig_adj.count({std::min(o1, o2), std::max(o1, o2)})) continue;
                    }
                    // SYMMETRIC coverage: true copies of one shared-edge sub-span score
                    // high BOTH ways; a piece spanning ACROSS a section crossing covers
                    // its neighbor's sub-span one-way only (base chairs: such asymmetric
                    // unions bridged the section and flipped correct verdicts, 35->43
                    // faces). Same-span copies keep uniting (chairsROT z15 25->8 naked).
                    int hits_i = 0, hits_j = 0;
                    for (const auto& p : border[i2].smp) {
                        double dmin2 = 1e300;
                        for (const auto& q : border[j2].smp) dmin2 = std::min(dmin2, p.distance(q));
                        if (dmin2 < on_eps) ++hits_i;
                    }
                    for (const auto& q : border[j2].smp) {
                        double dmin2 = 1e300;
                        for (const auto& p : border[i2].smp) dmin2 = std::min(dmin2, q.distance(p));
                        if (dmin2 < on_eps) ++hits_j;
                    }
                    if (hits_i >= 7 && hits_j >= 7) unite(border[i2].fi, border[j2].fi);
                }
            // STEP-1 AUDIT (SESSION_FLOOD_AUDIT): does the connexity flood BRIDGE a section? A
            // section edge MUST separate blocks; a section edge whose two fragments land in the
            // SAME block means the flood united across it (a partial/unkeyed section piece was
            // collected as a non-section border edge and united in step 2), so the parity phase
            // propagates ONE block's verdict across the section -> the wrong-volume signature.
            if (std::getenv("SESSION_FLOOD_AUDIT")) {
                std::map<int,int> bsz;
                for (int fi = 0; fi < nf; ++fi) if (valid[fi] && !is_on[fi]) bsz[uf_find(fi)]++;
                int cross_broad = 0, cross_keyed = 0, sec2 = 0, naked_sec = 0;
                for (int ei = 0; ei < (int)X2.m_topology_edges.size(); ++ei) {
                    const auto& E = X2.m_topology_edges[ei];
                    bool broad = is_section_edge(ei);
                    if ((int)E.trim_indices.size() == 1 && broad) ++naked_sec;
                    if ((int)E.trim_indices.size() != 2 || !broad) continue;
                    int f1 = face_of(E.trim_indices[0]), f2 = face_of(E.trim_indices[1]);
                    if (f1 < 0 || f2 < 0 || f1 == f2 || !valid[f1] || !valid[f2]) continue;
                    ++sec2;
                    if (uf_find(f1) == uf_find(f2)) {
                        ++cross_broad; if (sec_set.count(ei)) ++cross_keyed;
                        std::printf("[FLOOD-BRIDGE] %c e%d f%d~f%d SAME block%d keyed=%d\n",
                                    is_first ? 'A' : 'B', ei, f1, f2, uf_find(f1), sec_set.count(ei) ? 1 : 0);
                    }
                }
                std::printf("[FLOOD-AUDIT] %c blocks=%zu sec2trim=%d cross-block=%d(keyed %d) naked-sec=%d\n",
                            is_first ? 'A' : 'B', bsz.size(), sec2, cross_broad, cross_keyed, naked_sec);
            }
            // (3a) block votes (used by the parity cross-check and the fallback majority)
            std::map<int, double> vote_in, vote_n;
            for (int fi = 0; fi < nf; ++fi) {
                if (!valid[fi] || is_on[fi]) continue;
                double w = 0.5 + std::abs((score[fi] < 0 ? (inside_v[fi] ? 1.0 : 0.0) : score[fi]) - 0.5);
                int r2 = uf_find(fi);
                vote_in[r2] += inside_v[fi] ? w : 0.0;
                vote_n[r2] += w;
            }
            // (3b) Seeded BFS-parity over the section-bounded block graph (OCCT
            // FillIn3DParts / cellular-parity doctrine): inside/outside flips EXACTLY when
            // crossing a section edge, so ONE hardened verdict per connected component
            // determines every block by parity -- per-fragment sampling noise cannot flip
            // interleaved regions (chairsROT x20/y30 rim tiling). Components where parity
            // contradicts itself (odd cycle; flood united across a section) or contradicts
            // a strong (>=80%) local block majority FALL BACK to the majority (refuse, not
            // out-vote -- the CGAL impossible_operation discipline).
            static const bool s_no_parity = (std::getenv("SESSION_NO_PARITY") != nullptr);
            std::set<int> parity_done;   // fragments classified by parity
            if (!s_no_parity) {
                std::set<int> internal_bad;
                std::map<int, std::vector<int>> nbr;
                for (int ei = 0; ei < (int)X2.m_topology_edges.size(); ++ei) {
                    const auto& E = X2.m_topology_edges[ei];
                    // Parity constraints use KEYED section edges only (exact chain-block
                    // provenance). The looser endpoint-on-scaffold-vertex recognition is
                    // fine for flood SEPARATION (over-separating is safe) but a false
                    // section edge here would inject a wrong flip constraint and poison
                    // the whole component into conflict/fallback.
                    if ((int)E.trim_indices.size() != 2 || !sec_set.count(ei)) continue;
                    int f1 = face_of(E.trim_indices[0]), f2 = face_of(E.trim_indices[1]);
                    if (f1 < 0 || f2 < 0 || f1 == f2) continue;
                    if (!valid[f1] || !valid[f2] || is_on[f1] || is_on[f2]) continue;
                    int r1 = uf_find(f1), r2 = uf_find(f2);
                    if (r1 == r2) { internal_bad.insert(r1); continue; }
                    nbr[r1].push_back(r2);
                    nbr[r2].push_back(r1);
                }
                std::set<int> roots;
                for (int fi = 0; fi < nf; ++fi)
                    if (valid[fi] && !is_on[fi]) roots.insert(uf_find(fi));
                std::map<int, int> color, compid;
                std::map<int, std::vector<int>> comp_blocks;
                std::map<int, bool> comp_conf;
                int ncomp = 0;
                for (int r : roots) {
                    if (compid.count(r)) continue;
                    int cid = ncomp++;
                    bool conf = false;
                    std::vector<int> stk = {r};
                    color[r] = 0; compid[r] = cid; comp_blocks[cid].push_back(r);
                    while (!stk.empty()) {
                        int b = stk.back(); stk.pop_back();
                        for (int nb2 : nbr[b]) {
                            if (!roots.count(nb2)) continue;
                            if (!compid.count(nb2)) {
                                compid[nb2] = cid; color[nb2] = color[b] ^ 1;
                                comp_blocks[cid].push_back(nb2); stk.push_back(nb2);
                            } else if (color[nb2] == color[b]) conf = true;
                        }
                    }
                    for (int b : comp_blocks[cid]) if (internal_bad.count(b)) conf = true;
                    comp_conf[cid] = conf;
                }
                int npar = 0, nfall = 0;
                for (auto& ck : comp_blocks) {
                    int cid = ck.first;
                    if (comp_conf[cid]) { ++nfall; continue; }
                    // A 2-colouring fixes the component's verdicts up to ONE bit: the phase
                    // P = verdict XOR block-colour, constant over the component. Rather than
                    // trust a single seed, let EVERY locally certified fragment vote on P --
                    // an angle-method verdict is exact local radial-order evidence, and a
                    // component that has several of them contradicting each other is telling
                    // us its PARTITION is wrong (a flood union bridged a section, or a
                    // section edge is missing), in which case propagating any phase would
                    // spread the error. Fall back instead; only settle the phase when the
                    // certified evidence is >= 80% consistent.
                    int ev_tot = 0, ev_p1 = 0;
                    // ANGLE-PRIMARY (KB law 7, gated SESSION_ANG_PRIMARY): seed the component
                    // phase from fragments with strong ANALYTIC angle evidence (exact local
                    // radial order from surface derivatives), NOT the sampling-derived score.
                    // Point-in-solid sampling is unreliable exactly at the grazing section rims
                    // these components tile (documented 67-86% A-side accuracy; z90 winding
                    // inversion), while the angle method's dp SIGN stays valid there. Only where
                    // NO angle evidence exists in the component do we fall back to score-based
                    // evidence -- so winding/exact-PIP is demoted to a last-resort seed, never
                    // allowed to out-vote analytic radial order.
                    if (std::getenv("SESSION_ANG_PRIMARY")) {
                        for (int fi = 0; fi < nf; ++fi) {
                            if (!valid[fi] || is_on[fi]) continue;
                            int b = uf_find(fi);
                            auto itC = compid.find(b);
                            if (itC == compid.end() || itC->second != cid) continue;
                            if (ang_abs[fi] <= 0.0) continue;
                            double aconf = std::abs(ang_sum[fi]) / ang_abs[fi];
                            if (aconf < 0.5) continue;             // strong analytic evidence only
                            int av = ang_sum[fi] > 0.0 ? 1 : 0;
                            ++ev_tot;
                            if ((av ^ color[b]) == 1) ++ev_p1;
                        }
                    }
                    if (ev_tot == 0) {
                        for (int fi = 0; fi < nf; ++fi) {
                            if (!valid[fi] || is_on[fi]) continue;
                            int b = uf_find(fi);
                            auto itC = compid.find(b);
                            if (itC == compid.end() || itC->second != cid) continue;
                            if (score[fi] < 0.0 || std::abs(score[fi] - 0.5) < 0.25) continue;
                            ++ev_tot;
                            if ((inside_v[fi] ^ color[b]) == 1) ++ev_p1;
                        }
                    }
                    int P;
                    if (ev_tot > 0) {
                        int agree = std::max(ev_p1, ev_tot - ev_p1);
                        if (agree * 5 < ev_tot * 4) { ++nfall; continue; }   // contradictory
                        P = (ev_p1 * 2 > ev_tot) ? 1 : 0;
                    } else {
                        // no local evidence anywhere in the component: harden the single
                        // most-confident fragment with a K=9 winding re-vote and use it
                        int seed = -1; double bw = -1.0;
                        for (int fi = 0; fi < nf; ++fi) {
                            if (!valid[fi] || is_on[fi]) continue;
                            int b = uf_find(fi);
                            auto itC = compid.find(b);
                            if (itC == compid.end() || itC->second != cid) continue;
                            double w = std::abs((score[fi] < 0 ? (inside_v[fi] ? 1.0 : 0.0) : score[fi]) - 0.5);
                            if (w > bw) { bw = w; seed = fi; }
                        }
                        if (seed < 0) continue;
                        double ssc = -1.0;
                        int sin = vote_inside(seed, 9, &ssc) ? 1 : 0;
                        P = sin ^ color[uf_find(seed)];
                    }
                    bool contradicted = false;
                    std::map<int, int> pv;
                    for (int b : ck.second) {
                        pv[b] = (color[b] == 0) ? P : 1 - P;
                        auto itn = vote_n.find(b);
                        if (itn != vote_n.end() && itn->second > 0) {
                            double frac = vote_in[b] / itn->second;
                            if ((frac >= 0.8 && pv[b] == 0) || (frac <= 0.2 && pv[b] == 1)) contradicted = true;
                        }
                    }
                    if (contradicted) { ++nfall; continue; }
                    for (int fi = 0; fi < nf; ++fi) {
                        if (!valid[fi] || is_on[fi]) continue;
                        int b = uf_find(fi);
                        auto itC = compid.find(b);
                        if (itC == compid.end() || itC->second != cid) continue;
                        if (!certified[fi] && inside_v[fi] != pv[b]) { inside_v[fi] = pv[b]; ++npar; }
                        if (!certified[fi]) score[fi] = pv[b] ? 1.0 : 0.0;
                        parity_done.insert(fi);
                    }
                }
                if (std::getenv("SESSION_CLS_DBG"))
                    std::printf("[CLS-PARITY] %c comps=%d fallback=%d flipped=%d done=%zu\n",
                                is_first ? 'A' : 'B', ncomp, nfall, npar, parity_done.size());
            }
            // (3c) fallback: confidence-weighted majority for fragments parity didn't settle
            int nflip = 0;
            for (int fi = 0; fi < nf; ++fi) {
                if (!valid[fi] || is_on[fi] || parity_done.count(fi) || certified[fi]) continue;
                int r2 = uf_find(fi);
                if (vote_n[r2] <= 0) continue;
                int bv = vote_in[r2] * 2.0 > vote_n[r2] ? 1 : 0;
                if (bv != inside_v[fi]) { inside_v[fi] = bv; ++nflip; }
            }
            if (nflip && std::getenv("SESSION_CLS_DBG"))
                std::printf("[CLS-FLOOD] %c flipped %d fragment verdicts to block majority\n",
                            is_first ? 'A' : 'B', nflip);
            // (4) Post-flood section-edge invariant re-enforcement -- MEASURED HARMFUL,
            // OFF by default (SESSION_CLS_FIX2 to enable). The invariant itself is real (a
            // section edge separates inside from outside, so exactly one flank is kept and
            // the edge gets one trim per operand), and the flood/parity CAN re-break it
            // when the two flanks land in different blocks. But repairing it by flipping a
            // single fragment trades one invariant for another: the flipped fragment now
            // contradicts its own block across NON-section edges, which is precisely what
            // the connexity flood exists to prevent, and that costs more than it buys --
            // chairsROT z30 cut went 23 -> 35 naked edges, both with a naive weakest-flank
            // tie-break and with a conservative rule that only ever flips a sampled (never
            // a parity-derived) flank. A violation here means the BLOCK PARTITION is wrong,
            // not the individual verdict; the correct repair is to re-partition, which the
            // parity conflict detector already reports. Kept, gated, as a diagnostic.
            if (std::getenv("SESSION_CLS_FIX2")) {
                int nfix2 = 0;
                for (const auto& kv : *sec_edges) {
                    int ei = kv.first;
                    if (ei < 0 || ei >= (int)X2.m_topology_edges.size()) continue;
                    const auto& E = X2.m_topology_edges[ei];
                    if ((int)E.trim_indices.size() != 2) continue;
                    int fs[2] = {-1, -1};
                    for (int k = 0; k < 2; ++k) fs[k] = face_of(E.trim_indices[k]);
                    if (fs[0] < 0 || fs[1] < 0 || fs[0] == fs[1]) continue;
                    if (!valid[fs[0]] || !valid[fs[1]] || is_on[fs[0]] || is_on[fs[1]]) continue;
                    if (inside_v[fs[0]] != inside_v[fs[1]]) continue;
                    // Repair the SAMPLED flank, never a parity-derived one. When both sides
                    // are parity-certified the violation means the parity graph itself was
                    // inconsistent here (already surfaced as a conflicted component); an
                    // arbitrary flip there is destructive -- measured: chairsROT z30 cut
                    // 23 -> 35 naked when this pass flipped parity verdicts by tie-break.
                    bool p0 = parity_done.count(fs[0]) != 0, p1 = parity_done.count(fs[1]) != 0;
                    if (p0 && p1) continue;
                    int flip;
                    if (p0 != p1) flip = p0 ? fs[1] : fs[0];
                    else {
                        double c0 = std::abs((score[fs[0]] < 0 ? 0.5 : score[fs[0]]) - 0.5);
                        double c1 = std::abs((score[fs[1]] < 0 ? 0.5 : score[fs[1]]) - 0.5);
                        flip = c0 <= c1 ? fs[0] : fs[1];
                    }
                    inside_v[flip] = 1 - inside_v[flip];
                    score[flip] = 0.5;   // forced: no longer evidence for anything
                    ++nfix2;
                }
                if (nfix2 && std::getenv("SESSION_CLS_DBG"))
                    std::printf("[CLS-FIX2] %c re-enforced section separation on %d edges\n",
                                is_first ? 'A' : 'B', nfix2);
            }
        }
        // FINAL verdict audit line ([CLS] above prints the PRE-override sampled verdict, so
        // it measures the winding sampler, not the classifier). This one is emitted after
        // the angle method, the section repair, the flood and the parity propagation, and is
        // what cls_audit.sh compares against OCCT's BRepClass3d point oracle.
        if (std::getenv("SESSION_CLS_DBG"))
            for (int fi = 0; fi < nf; ++fi) {
                if (!valid[fi]) continue;
                std::printf("[CLSF] %c f%d sp(%.4f,%.4f,%.4f) on=%d ins=%d sc=%.2f\n",
                            is_first ? 'A' : 'B', fi, fsample[fi][0], fsample[fi][1], fsample[fi][2],
                            is_on[fi], inside_v[fi], score[fi]);
            }
        for (int fi = 0; fi < nf; ++fi) {
            if (!valid[fi]) continue;
            if (!is_on[fi]) {
                bool inside = inside_v[fi] != 0;
                if (op == BooleanOp::Union) keep_v[fi] = !inside;
                else if (op == BooleanOp::Intersection) keep_v[fi] = inside;
                else { if (is_first) keep_v[fi] = !inside; else { keep_v[fi] = inside; rev_v[fi] = true; } }
            }
        }
        // ISLAND/HOLE REPAIR (topological watertightness invariant). Keep-state may only
        // change across a SECTION edge; across a face's own (non-section) border it is
        // invariant -- the exact premise the connexity flood relies on. So a fragment whose
        // keep-state disagrees with EVERY one of its non-section 2-trim neighbours is, by
        // definition, mis-classified: a kept island (all its edges would be naked) or a
        // dropped hole. This happens when point sampling is not merely noisy but
        // SYSTEMATICALLY wrong -- a rotated freeform mesh can self-intersect and invert the
        // winding over a whole pocket, so all K samples agree on the wrong answer and neither
        // the angle method (no nearby section) nor the flood (which agrees with the confident
        // wrong sample) corrects it. Flipping such a fragment can ONLY mate naked edges, never
        // create one, so it is a monotone repair. And it is a STRICT NO-OP on any watertight
        // result: there every kept face shares each non-section edge with a kept neighbour, so
        // no fragment has all-opposite neighbours -- base chairs and the matrix stay identical.
        if (use_scaffold && sec_edges && !std::getenv("SESSION_NO_ISLAND")) {
            std::set<int> sec_set2;
            for (const auto& kv : *sec_edges) sec_set2.insert(kv.first);
            auto face_of2 = [&](int ti) -> int {
                if (ti < 0 || ti >= (int)X2.m_trims.size()) return -1;
                int li = X2.m_trims[ti].loop_index;
                return (li >= 0 && li < (int)X2.m_loops.size()) ? X2.m_loops[li].face_index : -1;
            };
            std::vector<int> nb_tot(nf, 0), nb_dis(nf, 0);
            for (int ei = 0; ei < (int)X2.m_topology_edges.size(); ++ei) {
                if (sec_set2.count(ei)) continue;                       // section edge: keep-state may flip here
                const auto& E = X2.m_topology_edges[ei];
                if ((int)E.trim_indices.size() != 2) continue;
                int f1 = face_of2(E.trim_indices[0]), f2 = face_of2(E.trim_indices[1]);
                if (f1 < 0 || f2 < 0 || f1 == f2) continue;
                if (!valid[f1] || !valid[f2] || is_on[f1] || is_on[f2]) continue;
                ++nb_tot[f1]; ++nb_tot[f2];
                if (keep_v[f1] != keep_v[f2]) { ++nb_dis[f1]; ++nb_dis[f2]; }
            }
            int n_island = 0;
            for (int fi = 0; fi < nf; ++fi) {
                if (!valid[fi] || is_on[fi] || certified[fi]) continue;
                if (nb_tot[fi] > 0 && nb_dis[fi] == nb_tot[fi]) {       // every non-section neighbour opposite
                    keep_v[fi] = !keep_v[fi];
                    ++n_island;
                    if (std::getenv("SESSION_CLS_DBG"))
                        std::printf("[CLS-ISLAND] %c f%d flipped keep (all %d non-section neighbours opposite)\n",
                                    is_first ? 'A' : 'B', fi, nb_tot[fi]);
                }
            }
            if (n_island && std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[CLS-ISLAND] %c flipped %d isolated fragments\n",
                             is_first ? 'A' : 'B', n_island);
        }
        for (int fi = 0; fi < nf; ++fi)
            if (valid[fi] && keep_v[fi]) { kept.push_back(fi); if (rev) rev->push_back(rev_v[fi]); }
    };
    // PHASE-5 RADIAL PRE-PASS (OCCT GetFaceOff material-side rule): for every section
    // block imprinted on BOTH operands, decide all four incident fragments' in/out
    // verdicts EXACTLY at the shared curve -- the in-face direction (left of the trim's
    // uv traversal, projected perpendicular to the curve tangent) against the OTHER
    // operand's oriented surface normal. Verdicts require a transversality margin and
    // an opposite-flank consistency check, then act as CERTIFIED seeds in classify.
    std::map<int, int> radialA, radialB;
    if (use_scaffold && !std::getenv("SESSION_NO_RADIAL")) {
        auto qf8 = [](double f) { return (long long)std::llround(f * 8.0); };
        std::map<std::tuple<int, long long, long long>, int> bIdx;
        for (const auto& kv : spansB)
            bIdx[{(int)kv.second[0], qf8(kv.second[1]), qf8(kv.second[2])}] = kv.first;
        std::map<int, int> aFaceOfSurf, bFaceOfSurf;
        for (int f = 0; f < (int)m_faces.size(); ++f) aFaceOfSurf[m_faces[f].surface_index] = f;
        for (int f = 0; f < (int)other.m_faces.size(); ++f) bFaceOfSurf[other.m_faces[f].surface_index] = f;
        struct Flank { int fi[2] = {-1, -1}; double d[2][3]; };
        auto flanks_of = [&](const BRep& X2, int ei, const double T[3], Flank& out) -> bool {
            const auto& E = X2.m_topology_edges[ei];
            if ((int)E.trim_indices.size() != 2) return false;
            for (int k = 0; k < 2; ++k) {
                int ti = E.trim_indices[k];
                const auto& tr = X2.m_trims[ti];
                int li = tr.loop_index;
                int fi = (li >= 0 && li < (int)X2.m_loops.size()) ? X2.m_loops[li].face_index : -1;
                if (fi < 0 || tr.curve_2d_index < 0) return false;
                const NurbsSurface& S = X2.m_surfaces[X2.m_faces[fi].surface_index];
                const NurbsCurve& pc = X2.m_curves_2d[tr.curve_2d_index];
                auto dc = pc.domain();
                double tm = 0.5 * (dc.first + dc.second);
                Point uv = pc.point_at(tm);
                Point uv2 = pc.point_at(std::min(dc.second, tm + (dc.second - dc.first) * 1e-3));
                double tx = uv2[0]-uv[0], ty = uv2[1]-uv[1];
                double tl = std::hypot(tx, ty);
                if (tl < 1e-30) return false;
                tx /= tl; ty /= tl;
                auto du2 = S.domain(0); auto dv2 = S.domain(1);
                double h = 1e-3 * std::min(du2.second - du2.first, dv2.second - dv2.first);
                // interior lies LEFT of the traversal (CCW outer / CW inner invariant)
                Point base = S.point_at(uv[0], uv[1]);
                Point q = S.point_at(uv[0] - ty*h, uv[1] + tx*h);
                double d3[3] = {q[0]-base[0], q[1]-base[1], q[2]-base[2]};
                double dt = d3[0]*T[0] + d3[1]*T[1] + d3[2]*T[2];
                d3[0] -= dt*T[0]; d3[1] -= dt*T[1]; d3[2] -= dt*T[2];
                double dl = std::sqrt(d3[0]*d3[0] + d3[1]*d3[1] + d3[2]*d3[2]);
                if (dl < 1e-14) return false;
                out.fi[k] = fi;
                for (int x = 0; x < 3; ++x) out.d[k][x] = d3[x] / dl;
            }
            return out.fi[0] != out.fi[1];
        };
        std::map<int, std::array<int, 2>> voteA, voteB;   // fragment -> {out_votes, in_votes}
        int blocks_used = 0;
        // Each operand certifies its own flanks independently: the verdict needs only the
        // seg's shared chain, this side's two incident fragments, and the OTHER SURFACE's
        // oriented normal -- no cross-operand block matching (one-sided imprints certify too).
        auto radial_side = [&](const BRep& X2, const std::map<int, std::array<double, 3>>& spans,
                               bool sideA, std::map<int, std::array<int, 2>>& vote) {
            for (const auto& kv : spans) {
                int seg = (int)kv.second[0];
                if (seg < 0 || seg >= (int)scaf.segments.size()) continue;
                int eX = kv.first;
                if (eX < 0 || eX >= (int)X2.m_topology_edges.size()) continue;
                const SectionSegment& s = scaf.segments[seg];
                size_t n = s.p3.size();
                if (n < 2 || s.uvA.size() != n || s.uvB.size() != n) continue;
                if (s.surfA < 0 || s.surfA >= (int)m_surfaces.size()) continue;
                if (s.surfB < 0 || s.surfB >= (int)other.m_surfaces.size()) continue;
                double fa = kv.second[1], fb = kv.second[2];
                if (fb < fa) std::swap(fa, fb);
                Flank FX;
                bool got = false;
                // up to 3 stations across the block: robust when one station is degenerate
                for (double fr : {0.5, 0.25, 0.75}) {
                    double fmid = fa + (fb - fa) * fr;
                    size_t mi = (size_t)std::min<double>((double)n - 2, std::max(0.0, fmid));
                    double T[3] = {s.p3[mi+1][0]-s.p3[mi][0], s.p3[mi+1][1]-s.p3[mi][1], s.p3[mi+1][2]-s.p3[mi][2]};
                    double Tl = std::sqrt(T[0]*T[0] + T[1]*T[1] + T[2]*T[2]);
                    if (Tl < 1e-14) continue;
                    T[0] /= Tl; T[1] /= Tl; T[2] /= Tl;
                    FX = Flank();
                    if (!flanks_of(X2, eX, T, FX)) continue;
                    // oriented normal of the OTHER operand's surface at this station
                    const std::vector<Point>& uvo = sideA ? s.uvB : s.uvA;
                    int surfo = sideA ? s.surfB : s.surfA;
                    const BRep& OP = sideA ? other : *this;
                    const std::vector<double>& osg = sideA ? osignB : osignA;
                    std::map<int, int>& fos = sideA ? bFaceOfSurf : aFaceOfSurf;
                    Vector nn = OP.m_surfaces[surfo].normal_at(uvo[mi][0], uvo[mi][1]);
                    double nl = nn.magnitude();
                    auto itf = fos.find(surfo);
                    if (nl < 1e-12 || itf == fos.end() || itf->second >= (int)osg.size()) continue;
                    double so = osg[itf->second];
                    if (so == 0.0) continue;
                    double no3[3] = {nn[0]*so/nl, nn[1]*so/nl, nn[2]*so/nl};
                    double a0 = FX.d[0][0]*no3[0] + FX.d[0][1]*no3[1] + FX.d[0][2]*no3[2];
                    double a1 = FX.d[1][0]*no3[0] + FX.d[1][1]*no3[1] + FX.d[1][2]*no3[2];
                    const double marg = 0.05;
                    if (a0 * a1 < 0 && std::min(std::abs(a0), std::abs(a1)) > marg) {
                        vote[FX.fi[0]][a0 < 0 ? 1 : 0]++;
                        vote[FX.fi[1]][a1 < 0 ? 1 : 0]++;
                        got = true;
                        break;
                    }
                }
                if (got) ++blocks_used;
            }
        };
        radial_side(A2, spansA, true, voteA);
        radial_side(B2, spansB, false, voteB);
        (void)bIdx;
        int nconf = 0;
        for (auto& kv : voteA) {
            if (kv.second[0] > 0 && kv.second[1] == 0) radialA[kv.first] = 0;
            else if (kv.second[1] > 0 && kv.second[0] == 0) radialA[kv.first] = 1;
            else ++nconf;
        }
        for (auto& kv : voteB) {
            if (kv.second[0] > 0 && kv.second[1] == 0) radialB[kv.first] = 0;
            else if (kv.second[1] > 0 && kv.second[0] == 0) radialB[kv.first] = 1;
            else ++nconf;
        }
        if (std::getenv("SESSION_NT_DBG"))
            std::printf("[RADIAL] blocks=%d certA=%zu certB=%zu conflicts=%d\n",
                        blocks_used, radialA.size(), radialB.size(), nconf);
    }
    classify(A2, other, meshB, primB, *this, meshA, primA, true, keptA, nullptr,
             use_scaffold ? &secA_edges : nullptr, use_scaffold ? &srcA_faces : nullptr,
             use_scaffold ? &osignA : nullptr, use_scaffold ? &osignB : nullptr,
             radialA.empty() ? nullptr : &radialA);
    classify(B2, *this, meshA, primA, other, meshB, primB, false, keptB, &revB,
             use_scaffold ? &secB_edges : nullptr, use_scaffold ? &srcB_faces : nullptr,
             use_scaffold ? &osignB : nullptr, use_scaffold ? &osignA : nullptr,
             radialB.empty() ? nullptr : &radialB);   lap("classify");

    // Select faces WITH their already-mated topology (subset preserves shared edges), then
    // combine the two sides and sew only the A<->B intersection edges. This keeps each
    // solid's internal box/cylinder edges mated -- the key to a watertight result.
    std::map<int, int> remapA, remapB;   // S2: split-edge idx -> subset-edge idx
    BRep subA = A2.subset(keptA, use_scaffold ? &remapA : nullptr);
    BRep subB = B2.subset(keptB, use_scaffold ? &remapB : nullptr);
    for (size_t k = 0; k < revB.size() && k < subB.m_faces.size(); ++k)
        if (revB[k]) subB.m_faces[k].reversed = !subB.m_faces[k].reversed;

    // OCCT-adoption S2: alias subB's scaffold-section edges onto subA's copies. Both sides
    // lifted the SAME shared chain per segment (identical 3D, identical endpoints), so the
    // alias turns the pair into ONE edge record with two trims -- no sewing of section edges.
    // Pave-block upgrade: the alias key is the edge's quantized chain SPAN (seg, fa, fb),
    // not the bare seg_id -- after per-side block normalization a segment can carry several
    // block edges per side, and last-write-wins seg keying would alias mismatched spans.
    auto qspan = [](double f) { return (long long)std::llround(f * 32.0); };
    std::map<std::tuple<int, long long, long long>, int> span_to_res;  // (seg,qa,qb) -> subA edge
    std::map<int, std::array<double, 3>> b_span_of;                    // subB edge -> span
    std::map<int, std::array<double, 3>> spansR;                       // result edge -> span
    if (use_scaffold) {
        for (const auto& kv : spansA) {
            auto itR = remapA.find(kv.first);
            if (itR == remapA.end()) continue;
            span_to_res[{(int)kv.second[0], qspan(kv.second[1]), qspan(kv.second[2])}] = itR->second;
            spansR[itR->second] = kv.second;
        }
        for (const auto& kv : spansB) {
            auto itR = remapB.find(kv.first);
            if (itR != remapB.end()) b_span_of[itR->second] = kv.second;
        }
    }

    // [SEGAUDIT] kept-coverage symmetry per scaffold segment: a correct boolean keeps, for
    // every genuine section block, exactly ONE flank per operand. Segments kept on one side
    // only ARE the naked rims (one-sided classification/coverage), printed with trim counts.
    if (use_scaffold && std::getenv("SESSION_NT_DBG")) {
        std::map<int, int> akeep, bkeep;
        for (const auto& kv : spansA) {
            auto it = remapA.find(kv.first);
            if (it == remapA.end()) continue;
            akeep[(int)kv.second[0]] += (int)subA.m_topology_edges[it->second].trim_indices.size();
        }
        for (const auto& kv : spansB) {
            auto it = remapB.find(kv.first);
            if (it == remapB.end()) continue;
            bkeep[(int)kv.second[0]] += (int)subB.m_topology_edges[it->second].trim_indices.size();
        }
        std::set<int> allsegs;
        for (auto& kv : akeep) allsegs.insert(kv.first);
        for (auto& kv : bkeep) allsegs.insert(kv.first);
        for (int s2 : allsegs) {
            int an = akeep.count(s2) ? akeep[s2] : 0;
            int bn = bkeep.count(s2) ? bkeep[s2] : 0;
            if (an == bn) continue;
            int naE = 0, nbE = 0, naT = 0, nbT = 0;
            for (const auto& kv : spansA)
                if ((int)kv.second[0] == s2) { ++naE; naT += (int)A2.m_topology_edges[kv.first].trim_indices.size(); }
            for (const auto& kv : spansB)
                if ((int)kv.second[0] == s2) { ++nbE; nbT += (int)B2.m_topology_edges[kv.first].trim_indices.size(); }
            std::printf("[SEGAUDIT] seg=%d keptA=%d keptB=%d | A2 edges=%d trims=%d B2 edges=%d trims=%d\n",
                        s2, an, bn, naE, naT, nbE, nbT);
        }
    }
    result = subA;
    result.name = "boolean";
    int voff=(int)result.m_vertices.size(), tvoff=(int)result.m_topology_vertices.size();
    int soff=(int)result.m_surfaces.size(), c2off=(int)result.m_curves_2d.size();
    int c3off=(int)result.m_curves_3d.size();
    int loff=(int)result.m_loops.size(), foff=(int)result.m_faces.size(), toff=(int)result.m_trims.size();
    for (auto& p : subB.m_vertices) result.m_vertices.push_back(p);
    for (auto& s : subB.m_surfaces) result.m_surfaces.push_back(s);
    for (auto& c : subB.m_curves_2d) result.m_curves_2d.push_back(c);
    for (auto& c : subB.m_curves_3d) result.m_curves_3d.push_back(c);
    for (auto tv : subB.m_topology_vertices) { tv.point_index += voff; tv.edge_indices.clear(); result.m_topology_vertices.push_back(tv); }
    std::vector<int> b_edge_new(subB.m_topology_edges.size(), -1);
    int merged_sec = 0;
    // BOP2 Phase 4: arena edges occupy identical PRE-subset indices in A2 and B2; unify
    // by INDEX (deterministic), before any span/geometry matching.
    int nArena = use_bop2 ? (int)bop2_pool.arena.m_topology_edges.size() : 0;
    std::map<int, int> inv_remapB;
    if (nArena) for (const auto& kv : remapB) inv_remapB[kv.second] = kv.first;
    int p4_alias = 0;
    for (int j = 0; j < (int)subB.m_topology_edges.size(); ++j) {
        if (nArena) {
            auto itO = inv_remapB.find(j);
            if (itO != inv_remapB.end() && itO->second < nArena) {
                auto itA = remapA.find(itO->second);
                if (itA != remapA.end()) {
                    b_edge_new[j] = itA->second;
                    ++merged_sec; ++p4_alias;
                    auto itSp0 = b_span_of.find(j);
                    if (itSp0 != b_span_of.end()) spansR[b_edge_new[j]] = itSp0->second;
                    continue;
                }
            }
        }
        auto itSp = b_span_of.find(j);
        if (itSp != b_span_of.end()) {
            auto itA = span_to_res.find({(int)itSp->second[0], qspan(itSp->second[1]), qspan(itSp->second[2])});
            if (itA != span_to_res.end()) { b_edge_new[j] = itA->second; ++merged_sec; continue; }
        }
        auto e = subB.m_topology_edges[j];
        if (e.curve_3d_index>=0) e.curve_3d_index += c3off;
        if (e.start_vertex>=0) e.start_vertex += tvoff;
        if (e.end_vertex>=0) e.end_vertex += tvoff;
        e.trim_indices.clear();
        b_edge_new[j] = (int)result.m_topology_edges.size();
        result.m_topology_edges.push_back(e);
        if (itSp != b_span_of.end()) spansR[b_edge_new[j]] = itSp->second;
    }
    if (use_bop2 && std::getenv("SESSION_NT_DBG"))
        std::printf("[P4] arena-alias merged %d edges at combine\n", p4_alias);
    if (use_scaffold && std::getenv("SESSION_NT_DBG")) {
        std::printf("[SCAF-MERGE] section edges merged A<->B: %d (A-side %zu, B-side %zu)\n",
                    merged_sec, span_to_res.size(), b_span_of.size());
        std::printf("[SCAF-MERGE] A segs:");
        { std::set<int> as2; for (const auto& kv : span_to_res) as2.insert(std::get<0>(kv.first));
          for (int s2 : as2) std::printf(" %d", s2); }
        std::printf("  B segs:");
        { std::set<int> bs; for (const auto& kv : b_span_of) bs.insert((int)kv.second[0]);
          for (int s2 : bs) std::printf(" %d", s2); }
        std::printf("\n");
    }
    for (auto t : subB.m_trims) {
        if (t.curve_2d_index>=0) t.curve_2d_index += c2off;
        if (t.edge_index>=0 && t.edge_index < (int)b_edge_new.size()) t.edge_index = b_edge_new[t.edge_index];
        if (t.loop_index>=0) t.loop_index += loff;
        result.m_trims.push_back(t);
    }
    for (auto lp : subB.m_loops) {
        for (auto& ti : lp.trim_indices) ti += toff;
        if (lp.face_index>=0) lp.face_index += foff;
        result.m_loops.push_back(lp);
    }
    for (auto f : subB.m_faces) {
        if (f.surface_index>=0) f.surface_index += soff;
        for (auto& li : f.loop_indices) li += loff;
        result.m_faces.push_back(f);
    }
    for (auto& e : result.m_topology_edges) e.trim_indices.clear();
    for (int ti=0; ti<(int)result.m_trims.size(); ++ti) {
        int ei = result.m_trims[ti].edge_index;
        if (ei>=0 && ei<(int)result.m_topology_edges.size()) result.m_topology_edges[ei].trim_indices.push_back(ti);
    }
    // CROSS-OPERAND WELD + COMMON-BLOCK (construction-time; replaces sew/fuzzy residue).
    // Both operands' section endpoints were snapped to the SAME scaffold pave coordinates
    // during their splits, so cross-operand copies share bit-identical endpoints: weld the
    // topology vertices at exact coincidence, then merge 1-trim edge pairs whose curves
    // coincide within a tight tube (identical chain sub-extractions for aligned spans).
    bool m3_imprint_off = false;   // trial-with-rollback control (see first call site)
    auto run_xweld = [&]() {
      if (use_scaffold && !std::getenv("SESSION_NO_XWELD")) {
        auto qx = [](double v) { return (long long)std::llround(v * 1e7); };
        std::map<std::tuple<long long, long long, long long>, int> vq;
        std::vector<int> vmapw(result.m_topology_vertices.size());
        for (int tv = 0; tv < (int)result.m_topology_vertices.size(); ++tv) {
            vmapw[tv] = tv;
            const Point& p = result.m_vertices[result.m_topology_vertices[tv].point_index];
            long long kx = qx(p[0]), ky = qx(p[1]), kz = qx(p[2]);
            int hit = -1;
            for (long long dx = -1; dx <= 1 && hit < 0; ++dx)
                for (long long dy = -1; dy <= 1 && hit < 0; ++dy)
                    for (long long dz = -1; dz <= 1 && hit < 0; ++dz) {
                        auto it = vq.find(std::make_tuple(kx+dx, ky+dy, kz+dz));
                        if (it == vq.end()) continue;
                        const Point& q = result.m_vertices[result.m_topology_vertices[it->second].point_index];
                        if (q.distance(p) <= 1e-7) hit = it->second;
                    }
            if (hit >= 0) vmapw[tv] = hit;
            else vq[std::make_tuple(kx, ky, kz)] = tv;
        }
        int nweld = 0;
        for (auto& e : result.m_topology_edges) {
            if (e.start_vertex >= 0 && vmapw[e.start_vertex] != e.start_vertex) { e.start_vertex = vmapw[e.start_vertex]; ++nweld; }
            if (e.end_vertex >= 0 && vmapw[e.end_vertex] != e.end_vertex) { e.end_vertex = vmapw[e.end_vertex]; ++nweld; }
        }
        // M3 NETWORK-VERTEX IMPRINT (SESSION_M3; OCCT PostTreatFF/SeveralWlines
        // analog): a residual 1-trim edge spanning SEVERAL logical edges of its
        // would-be mates can never pair (z90: whole face boundaries emitted as ONE
        // closed edge after a join failure). Split every 1-trim edge at each network
        // vertex lying ON it (interior, within the tube band); the merge passes
        // below then mate the pieces 1:1. Closed edges are treated as open with
        // both ends at their anchor vertex.
        int m3_splits = 0;
        if (std::getenv("SESSION_M3") && !m3_imprint_off) {
            double bm = 0.6;
            if (const char* mb = std::getenv("SESSION_M3_BAND")) bm = std::atof(mb);
            double bandM = std::max(1e-9, (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * bm);
            std::vector<int> usesM(result.m_topology_edges.size(), 0);
            std::vector<int> onlyT(result.m_topology_edges.size(), -1);
            for (int ti2 = 0; ti2 < (int)result.m_trims.size(); ++ti2) {
                int e2 = result.m_trims[ti2].edge_index;
                if (e2 >= 0 && e2 < (int)usesM.size()) { ++usesM[e2]; onlyT[e2] = ti2; }
            }
            // cut candidates = endpoints of OTHER 1-trim edges only (the mate's piece
            // boundaries). Cutting at arbitrary network vertices creates pieces with
            // no mate at all -- measured +2 x20 / +7 z30 when netv was unrestricted.
            std::set<int> netv;
            for (int e2i = 0; e2i < (int)result.m_topology_edges.size(); ++e2i) {
                if (usesM[e2i] != 1) continue;
                const auto& e2 = result.m_topology_edges[e2i];
                if (e2.start_vertex >= 0) netv.insert(e2.start_vertex);
                if (e2.end_vertex >= 0) netv.insert(e2.end_vertex);
            }
            auto vpos3 = [&](int tv) -> Point {
                return result.m_vertices[result.m_topology_vertices[tv].point_index];
            };
            int nE0 = (int)result.m_topology_edges.size();
            // an edge with an endpoint-matching 1-trim partner will mate in the merge
            // passes below -- imprinting it only perturbs a working closure (x20 +2).
            // M3 is for the SPAN-MISMATCH class, whose mates never endpoint-match.
            double repsM = std::max(1e-9, (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * 0.5);
            auto vpM = [&](int tv) -> Point {
                return result.m_vertices[result.m_topology_vertices[tv].point_index];
            };
            auto has_mate = [&](int ei) {
                const auto& E = result.m_topology_edges[ei];
                if (E.start_vertex < 0 || E.end_vertex < 0) return false;
                Point a0 = vpM(E.start_vertex), a1 = vpM(E.end_vertex);
                for (int ej = 0; ej < (int)usesM.size(); ++ej) {
                    if (ej == ei || usesM[ej] != 1) continue;
                    const auto& F = result.m_topology_edges[ej];
                    if (F.start_vertex < 0 || F.end_vertex < 0) continue;
                    Point b0 = vpM(F.start_vertex), b1 = vpM(F.end_vertex);
                    if ((a0.distance(b0) <= repsM && a1.distance(b1) <= repsM) ||
                        (a0.distance(b1) <= repsM && a1.distance(b0) <= repsM)) return true;
                }
                return false;
            };
            for (int ei = 0; ei < nE0; ++ei) {
                if (usesM[ei] != 1) continue;
                int ci, sv, ev, ti;
                {
                    const auto& E = result.m_topology_edges[ei];
                    ci = E.curve_3d_index; sv = E.start_vertex; ev = E.end_vertex;
                    ti = onlyT[ei];
                }
                if (ci < 0 || ci >= (int)result.m_curves_3d.size() || ti < 0) continue;
                if (has_mate(ei)) continue;
                const NurbsCurve C = result.m_curves_3d[ci];
                if (!C.is_valid()) continue;
                auto dm = C.domain();
                if (!(dm.second > dm.first)) continue;
                Point c0 = C.point_at(dm.first), c1 = C.point_at(dm.second);
                std::vector<std::pair<double, int>> cutsM;
                for (int w : netv) {
                    if (w == sv || w == ev) continue;
                    Point P = vpos3(w);
                    double t = C.closest_parameter(P);
                    if (C.point_at(t).distance(P) > bandM) continue;
                    Point ct = C.point_at(t);
                    if (ct.distance(c0) < 2.0 * bandM || ct.distance(c1) < 2.0 * bandM) continue;
                    cutsM.push_back({t, w});
                }
                if (cutsM.empty()) continue;
                std::sort(cutsM.begin(), cutsM.end());
                {   // drop cuts closer than the band to each other along the curve
                    std::vector<std::pair<double, int>> ded;
                    for (auto& cw : cutsM)
                        if (ded.empty() || C.point_at(cw.first).distance(C.point_at(ded.back().first)) > 2.0 * bandM)
                            ded.push_back(cw);
                    cutsM.swap(ded);
                }
                if (cutsM.empty()) continue;
                const BRepTrim Tc = result.m_trims[ti];
                if (Tc.curve_2d_index < 0 || Tc.curve_2d_index >= (int)result.m_curves_2d.size()) continue;
                const NurbsCurve PC = result.m_curves_2d[Tc.curve_2d_index];
                if (!PC.is_valid()) continue;
                auto pdm = PC.domain();
                // deg-1 polylines: corners sit at integer params; any param evaluates ON
                // the polyline, so endpoint + interior-integer sampling is exact.
                auto piece3 = [&](double ta, double tb) {
                    std::vector<Point> ps{C.point_at(ta)};
                    for (int k = (int)std::ceil(ta + 1e-9); k <= (int)std::floor(tb - 1e-9); ++k)
                        ps.push_back(C.point_at((double)k));
                    ps.push_back(C.point_at(tb));
                    return ps;
                };
                auto piece2 = [&](double ta, double tb) {
                    double fa = (ta - dm.first) / (dm.second - dm.first);
                    double fb = (tb - dm.first) / (dm.second - dm.first);
                    double sa = pdm.first + fa * (pdm.second - pdm.first);
                    double sb = pdm.first + fb * (pdm.second - pdm.first);
                    std::vector<Point> ps{PC.point_at(sa)};
                    for (int k = (int)std::ceil(sa + 1e-9); k <= (int)std::floor(sb - 1e-9); ++k)
                        ps.push_back(PC.point_at((double)k));
                    ps.push_back(PC.point_at(sb));
                    return ps;
                };
                // vertex at the t0 end = whichever original end vertex is nearer C(t0)
                int v_lo = sv, v_hi = ev;
                if (sv >= 0 && ev >= 0 && sv != ev &&
                    vpos3(ev).distance(c0) < vpos3(sv).distance(c0)) { v_lo = ev; v_hi = sv; }
                std::vector<double> bps3{dm.first};
                std::vector<int> bvs{v_lo};
                for (auto& cw : cutsM) { bps3.push_back(cw.first); bvs.push_back(cw.second); }
                bps3.push_back(dm.second);
                bvs.push_back(v_hi);
                std::vector<int> new_trims;
                bool okM = true;
                std::vector<std::array<int, 3>> made;      // (edge, c2, trimless yet)
                for (size_t k = 0; k + 1 < bps3.size() && okM; ++k) {
                    NurbsCurve c3p = NurbsCurve::create(false, 1, piece3(bps3[k], bps3[k+1]));
                    NurbsCurve c2p = NurbsCurve::create(false, 1, piece2(bps3[k], bps3[k+1]));
                    if (!c3p.is_valid() || !c2p.is_valid()) { okM = false; break; }
                    int ci3 = result.add_curve_3d(c3p);
                    int ei2 = result.add_edge(ci3, bvs[k], bvs[k+1]);
                    int ci2 = result.add_curve_2d(c2p);
                    made.push_back({ei2, ci2, 0});
                }
                if (!okM || made.empty()) continue;
                // rebuild the loop: replace trim ti by the pieces in traversal order
                int li = Tc.loop_index;
                if (li < 0 || li >= (int)result.m_loops.size()) continue;
                auto& lts = result.m_loops[li].trim_indices;
                auto pos = std::find(lts.begin(), lts.end(), ti);
                if (pos == lts.end()) continue;
                size_t pi2 = (size_t)(pos - lts.begin());
                std::vector<int> seq;
                for (size_t k = 0; k < made.size(); ++k) {
                    size_t idx = Tc.reversed ? made.size() - 1 - k : k;
                    int tnew;
                    if (k == 0) {
                        // reuse trim ti for the first traversed piece
                        result.m_trims[ti].edge_index = made[idx][0];
                        result.m_trims[ti].curve_2d_index = made[idx][1];
                        tnew = ti;
                    } else {
                        tnew = result.add_trim(made[idx][1], made[idx][0], li,
                                               Tc.reversed, Tc.type);
                    }
                    seq.push_back(tnew);
                }
                lts.erase(lts.begin() + pi2);
                lts.insert(lts.begin() + pi2, seq.begin(), seq.end());
                m3_splits += (int)made.size() - 1;
                if (std::getenv("SESSION_NT_DBG"))
                    std::printf("[M3SPLIT] e%d -> %zu pieces (cuts=%zu)\n", ei, made.size(), cutsM.size());
            }
        }
        // common-block: 1-trim edges by welded vertex pair, tight-tube coincidence
        std::vector<int> uses(result.m_topology_edges.size(), 0);
        for (const auto& t : result.m_trims)
            if (t.edge_index >= 0 && t.edge_index < (int)uses.size()) ++uses[t.edge_index];
        std::map<std::pair<int, int>, std::vector<int>> by_vp;
        for (int ei = 0; ei < (int)uses.size(); ++ei) {
            if (uses[ei] != 1) continue;
            const auto& E = result.m_topology_edges[ei];
            if (E.start_vertex < 0 || E.end_vertex < 0) continue;
            // closed 1-trim loops anchor at arbitrary start vertices, so the two copies of
            // one circular imprint rarely share a vertex pair: bucket ALL self-loops
            // together and let the (anchor-independent) tube test pair them.
            if (E.start_vertex == E.end_vertex)
                by_vp[{-1, -1}].push_back(ei);
            else
                by_vp[{std::min(E.start_vertex, E.end_vertex),
                       std::max(E.start_vertex, E.end_vertex)}].push_back(ei);
        }
        auto poly_of = [&](int ei) {
            std::vector<Point> ps;
            const auto& E = result.m_topology_edges[ei];
            if (E.curve_3d_index >= 0 && E.curve_3d_index < (int)result.m_curves_3d.size()) {
                const NurbsCurve& c = result.m_curves_3d[E.curve_3d_index];
                auto d = c.domain();
                for (int k = 0; k <= 16; ++k)
                    ps.push_back(c.point_at(d.first + (d.second - d.first) * k / 16.0));
            }
            return ps;
        };
        // 0.6*tol3: a section running ALONG the other operand's boundary edge (border-
        // coincident drop) must pair with that EXISTING edge (OCCT IsExistingPaveBlock);
        // the two curves diverge by the graze distance, which is tolerance-scale. The
        // vertex-pair grouping + 1-trim-only + 2-trim cap keep wider tubes safe.
        double tube = std::max(1e-9, (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * 0.6);
        int xcb = 0;
        // BEST-MATCH-FIRST greedy (OCCT deterministic-representative discipline): the
        // old first-come order let a looser in-tube pair consume an edge's 2-trim cap
        // before its TRUE (tighter) mate was examined (x20 under M3: e105/e115 starved
        // by an earlier merge). Collect all in-tube pairs per bucket, sort by max
        // deviation, merge tightest first.
        for (auto& kv : by_vp) {
            auto& ids = kv.second;
            if (ids.size() < 2) continue;
            std::vector<char> gone(ids.size(), 0);
            std::vector<std::vector<Point>> polys(ids.size());
            for (size_t a2 = 0; a2 < ids.size(); ++a2) polys[a2] = poly_of(ids[a2]);
            std::vector<std::tuple<double, size_t, size_t>> candsCB;
            for (size_t a2 = 0; a2 < ids.size(); ++a2) {
                const auto& pa = polys[a2];
                if (pa.size() < 2) continue;
                for (size_t b2 = a2 + 1; b2 < ids.size(); ++b2) {
                    const auto& pb = polys[b2];
                    if (pb.size() < 2) continue;
                    bool ok = true;
                    double devm = 0;
                    for (int k = 2; k <= 14 && ok; k += 3) {
                        const Point& q = pa[k];
                        double best = 1e300;
                        for (size_t j = 0; j + 1 < pb.size(); ++j) {
                            double ex = pb[j+1][0]-pb[j][0], ey = pb[j+1][1]-pb[j][1], ez = pb[j+1][2]-pb[j][2];
                            double L2 = ex*ex + ey*ey + ez*ez;
                            double t2 = L2 > 1e-30 ? ((q[0]-pb[j][0])*ex + (q[1]-pb[j][1])*ey + (q[2]-pb[j][2])*ez) / L2 : 0.0;
                            t2 = std::min(std::max(t2, 0.0), 1.0);
                            double dx = q[0]-pb[j][0]-t2*ex, dy = q[1]-pb[j][1]-t2*ey, dz = q[2]-pb[j][2]-t2*ez;
                            best = std::min(best, dx*dx + dy*dy + dz*dz);
                        }
                        if (best > tube * tube) ok = false;
                        else devm = std::max(devm, best);
                    }
                    if (ok) candsCB.push_back({devm, a2, b2});
                }
            }
            std::sort(candsCB.begin(), candsCB.end());
            for (const auto& cnd : candsCB) {
                size_t a2 = std::get<1>(cnd), b2 = std::get<2>(cnd);
                if (gone[a2] || gone[b2]) continue;
                for (auto& t : result.m_trims)
                    if (t.edge_index == ids[b2]) { t.edge_index = ids[a2]; t.type = BRepTrimType::Mated; }
                gone[a2] = gone[b2] = 1;
                ++xcb;
            }
        }
        // NK-RESCUE (tolerant mate-pair pass): the x20 class -- two 1-trim copies of the
        // SAME curve on two faces whose endpoints differ by ~1e-4: above the exact 1e-7
        // weld, so they land in different vertex-pair buckets and the tube test never
        // sees them. Regroup survivors by tolerant endpoint matching (0.15*tol3) and
        // rerun the same tube merge, then weld the paired vertices across the result.
        int nres = 0;
        if (!std::getenv("SESSION_NO_NKRESCUE")) {
            std::vector<int> uses3(result.m_topology_edges.size(), 0);
            for (const auto& t : result.m_trims)
                if (t.edge_index >= 0 && t.edge_index < (int)uses3.size()) ++uses3[t.edge_index];
            auto vpos = [&](int tv) -> const Point& {
                return result.m_vertices[result.m_topology_vertices[tv].point_index];
            };
            // under M3, imprint-created piece ends sit up to the imprint band from their
            // unsplit mates -- the rescue match radius must cover that band
            double rf = std::getenv("SESSION_M3") ? 0.5 : 0.15;
            if (const char* rp = std::getenv("SESSION_M3_REPS")) rf = std::atof(rp);
            double reps = std::max(1e-9, (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * rf);
            std::vector<int> cand;
            for (int ei = 0; ei < (int)uses3.size(); ++ei) {
                if (uses3[ei] != 1) continue;
                const auto& E = result.m_topology_edges[ei];
                if (E.start_vertex < 0 || E.end_vertex < 0 || E.start_vertex == E.end_vertex) continue;
                cand.push_back(ei);
            }
            std::map<int, int> vw;
            std::vector<char> goneR(cand.size(), 0);
            std::vector<std::vector<Point>> polysR(cand.size());
            for (size_t a2 = 0; a2 < cand.size(); ++a2) polysR[a2] = poly_of(cand[a2]);
            // best-match-first (same discipline as the common-block pass above)
            std::vector<std::tuple<double, size_t, size_t, char>> candsR;
            for (size_t a2 = 0; a2 < cand.size(); ++a2) {
                const auto& EA = result.m_topology_edges[cand[a2]];
                Point a0 = vpos(EA.start_vertex), a1 = vpos(EA.end_vertex);
                const auto& pa = polysR[a2];
                if (pa.size() < 2) continue;
                for (size_t b2 = a2 + 1; b2 < cand.size(); ++b2) {
                    const auto& EB = result.m_topology_edges[cand[b2]];
                    // under M3 the same-vertex-pair pairs are re-judged here (the exact
                    // pass can miss them via bucket caps; d=0 identical A-A/B-B pairs)
                    if ((!std::getenv("SESSION_M3") || std::getenv("SESSION_M3_NOSP")) &&
                        EA.start_vertex == EB.start_vertex && EA.end_vertex == EB.end_vertex) continue;
                    Point b0 = vpos(EB.start_vertex), b1 = vpos(EB.end_vertex);
                    bool fwd = a0.distance(b0) <= reps && a1.distance(b1) <= reps;
                    bool rev = a0.distance(b1) <= reps && a1.distance(b0) <= reps;
                    if (!fwd && !rev) continue;
                    const auto& pb = polysR[b2];
                    if (pb.size() < 2) continue;
                    bool ok = true;
                    double devm = 0;
                    for (int k = 2; k <= 14 && ok; k += 3) {
                        const Point& q = pa[k];
                        double best = 1e300;
                        for (size_t j = 0; j + 1 < pb.size(); ++j) {
                            double ex = pb[j+1][0]-pb[j][0], ey = pb[j+1][1]-pb[j][1], ez = pb[j+1][2]-pb[j][2];
                            double L2 = ex*ex + ey*ey + ez*ez;
                            double t2 = L2 > 1e-30 ? ((q[0]-pb[j][0])*ex + (q[1]-pb[j][1])*ey + (q[2]-pb[j][2])*ez) / L2 : 0.0;
                            t2 = std::min(std::max(t2, 0.0), 1.0);
                            double dx = q[0]-pb[j][0]-t2*ex, dy = q[1]-pb[j][1]-t2*ey, dz = q[2]-pb[j][2]-t2*ez;
                            best = std::min(best, dx*dx + dy*dy + dz*dz);
                        }
                        if (best > tube * tube) ok = false;
                        else devm = std::max(devm, best);
                    }
                    if (ok) candsR.push_back({devm, a2, b2, (char)(fwd ? 1 : 0)});
                }
            }
            std::sort(candsR.begin(), candsR.end());
            for (const auto& cnd : candsR) {
                size_t a2 = std::get<1>(cnd), b2 = std::get<2>(cnd);
                bool fwd = std::get<3>(cnd) != 0;
                if (goneR[a2] || goneR[b2]) continue;
                const auto& EA = result.m_topology_edges[cand[a2]];
                const auto& EB = result.m_topology_edges[cand[b2]];
                for (auto& t : result.m_trims)
                    if (t.edge_index == cand[b2]) { t.edge_index = cand[a2]; t.type = BRepTrimType::Mated; }
                vw[EB.start_vertex] = fwd ? EA.start_vertex : EA.end_vertex;
                vw[EB.end_vertex]   = fwd ? EA.end_vertex   : EA.start_vertex;
                goneR[a2] = goneR[b2] = 1;
                ++nres;
            }
            if (!vw.empty())
                for (auto& e : result.m_topology_edges) {
                    auto itW = vw.find(e.start_vertex);
                    if (itW != vw.end()) e.start_vertex = itW->second;
                    itW = vw.find(e.end_vertex);
                    if (itW != vw.end()) e.end_vertex = itW->second;
                }
        }
        // MICRO-EDGE ORPHAN COLLAPSE: a 1-trim edge shorter than the pave tolerance is the
        // perimeter residue of a dropped sliver fragment (its mate died with the sliver).
        // Detach the trim and bridge the loop's 2D gap by snapping the next trim's entry
        // CV onto the previous trim's exit point (the proven sew-orphan splice).
        int micro_collapsed = 0;
        {
            std::vector<int> uses2(result.m_topology_edges.size(), 0);
            std::vector<int> only_trim(result.m_topology_edges.size(), -1);
            for (int ti = 0; ti < (int)result.m_trims.size(); ++ti) {
                int ei2 = result.m_trims[ti].edge_index;
                if (ei2 >= 0 && ei2 < (int)uses2.size()) { ++uses2[ei2]; only_trim[ei2] = ti; }
            }
            double mtol = (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * 0.7;
            for (int ei2 = 0; ei2 < (int)uses2.size(); ++ei2) {
                if (uses2[ei2] != 1) continue;
                const auto& E = result.m_topology_edges[ei2];
                if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)result.m_curves_3d.size()) continue;
                const NurbsCurve& c = result.m_curves_3d[E.curve_3d_index];
                auto d = c.domain();
                double len = 0;
                Point prevp = c.point_at(d.first);
                for (int k = 1; k <= 8; ++k) {
                    Point q = c.point_at(d.first + (d.second - d.first) * k / 8.0);
                    len += prevp.distance(q);
                    prevp = q;
                }
                // self-loops (both ends captured to one pave) bound zero area at tolerance
                // scale. Criterion = loop DIAMETER (max excursion from the anchor), not
                // circumference: a pin-hole of diameter d has length pi*d, and thresholding
                // the length made a d=0.013 hole survive a 0.039 tolerance.
                if (E.start_vertex == E.end_vertex) {
                    double dmax = 0;
                    Point anchor = c.point_at(d.first);
                    for (int k = 1; k <= 8; ++k)
                        dmax = std::max(dmax, anchor.distance(c.point_at(d.first + (d.second - d.first) * k / 8.0)));
                    if (dmax >= (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * 0.75) continue;
                } else if (len >= mtol) continue;
                int ti = only_trim[ei2];
                int li2 = result.m_trims[ti].loop_index;
                if (li2 < 0 || li2 >= (int)result.m_loops.size()) continue;
                auto& lts = result.m_loops[li2].trim_indices;
                auto pos = std::find(lts.begin(), lts.end(), ti);
                if (pos == lts.end()) continue;
                if (E.start_vertex == E.end_vertex) {
                    // a sub-tolerance self-loop is a pin-hole/lasso artifact: removable from
                    // ANY loop (no gap results -- it left and returned to the same vertex).
                    lts.erase(pos);
                    if (lts.empty()) {
                        int fi2 = result.m_loops[li2].face_index;
                        if (fi2 >= 0 && fi2 < (int)result.m_faces.size()) {
                            auto& fl = result.m_faces[fi2].loop_indices;
                            fl.erase(std::remove(fl.begin(), fl.end(), li2), fl.end());
                        }
                    }
                    result.m_trims[ti].edge_index = -1;
                    ++micro_collapsed;
                    continue;
                }
                if (lts.size() < 3) continue;
                size_t pi2 = (size_t)(pos - lts.begin());
                int tprev = lts[(pi2 + lts.size() - 1) % lts.size()];
                int tnext = lts[(pi2 + 1) % lts.size()];
                int c2p = result.m_trims[tprev].curve_2d_index;
                int c2n = result.m_trims[tnext].curve_2d_index;
                if (c2p < 0 || c2n < 0 || c2p >= (int)result.m_curves_2d.size() || c2n >= (int)result.m_curves_2d.size()) continue;
                NurbsCurve& cp = result.m_curves_2d[c2p];
                NurbsCurve& cn = result.m_curves_2d[c2n];
                if (!cp.is_valid() || !cn.is_valid() || cp.m_is_rat || cn.m_is_rat) continue;
                auto dp2 = cp.domain();
                Point pend = cp.point_at(dp2.second);
                double* cvn = cn.cv_array();
                int st = cn.m_cv_stride;
                cvn[0] = pend[0];
                cvn[1] = pend[1];
                lts.erase(lts.begin() + pi2);
                result.m_trims[ti].edge_index = -1;
                ++micro_collapsed;
            }
        }
        if (nweld || xcb || micro_collapsed || nres || m3_splits) {
            for (auto& e : result.m_topology_edges) e.trim_indices.clear();
            for (int ti = 0; ti < (int)result.m_trims.size(); ++ti) {
                int ei2 = result.m_trims[ti].edge_index;
                if (ei2 >= 0 && ei2 < (int)result.m_topology_edges.size())
                    result.m_topology_edges[ei2].trim_indices.push_back(ti);
            }
        }
        if (std::getenv("SESSION_NT_DBG")) {
            std::printf("[XWELD] welds=%d common-blocks=%d micro=%d rescue=%d m3=%d tube=%.4g\n", nweld, xcb, micro_collapsed, nres, m3_splits, tube);
            for (int ei = 0; ei < (int)result.m_topology_edges.size(); ++ei) {
                const auto& E = result.m_topology_edges[ei];
                if ((int)E.trim_indices.size() != 1) continue;
                double len = -1;
                if (E.curve_3d_index >= 0 && E.curve_3d_index < (int)result.m_curves_3d.size()) {
                    const NurbsCurve& c = result.m_curves_3d[E.curve_3d_index];
                    auto d = c.domain();
                    len = 0; Point pp = c.point_at(d.first);
                    for (int k = 1; k <= 8; ++k) { Point q = c.point_at(d.first + (d.second-d.first)*k/8.0); len += pp.distance(q); pp = q; }
                }
                int ti = E.trim_indices[0];
                int li2 = result.m_trims[ti].loop_index;
                int f2 = li2 >= 0 && li2 < (int)result.m_loops.size() ? result.m_loops[li2].face_index : -1;
                Point ea(0,0,0), eb(0,0,0);
                if (E.start_vertex >= 0 && E.start_vertex < (int)result.m_topology_vertices.size())
                    ea = result.m_vertices[result.m_topology_vertices[E.start_vertex].point_index];
                if (E.end_vertex >= 0 && E.end_vertex < (int)result.m_topology_vertices.size())
                    eb = result.m_vertices[result.m_topology_vertices[E.end_vertex].point_index];
                std::printf("[X1T] e%d len=%.4f f%d(%s) v(%d,%d) a(%.4f,%.4f,%.4f) b(%.4f,%.4f,%.4f)\n",
                            ei, len, f2, f2 >= 0 && f2 < foff ? "A" : "B", E.start_vertex, E.end_vertex,
                            ea[0], ea[1], ea[2], eb[0], eb[1], eb[2]);
            }
            for (int ei = 0; ei < (int)result.m_topology_edges.size(); ++ei) {
                const auto& E = result.m_topology_edges[ei];
                if ((int)E.trim_indices.size() < 3) continue;
                std::printf("[XNM] e%d trims=%zu faces:", ei, E.trim_indices.size());
                for (int ti : E.trim_indices) {
                    int li2 = result.m_trims[ti].loop_index;
                    int f2 = li2 >= 0 && li2 < (int)result.m_loops.size() ? result.m_loops[li2].face_index : -1;
                    if (f2 >= foff && f2 - foff < (int)keptB.size())
                        std::printf(" f%d(B:frag%d)", f2, keptB[f2 - foff]);
                    else if (f2 >= 0 && f2 < (int)keptA.size() + foff && f2 < foff && f2 < (int)keptA.size())
                        std::printf(" f%d(A:frag%d)", f2, keptA[f2]);
                    else
                        std::printf(" f%d(?)", f2);
                }
                std::printf("\n");
            }
        }
      }
    };
    // M3 TRIAL-WITH-ROLLBACK (ACIS escalate-then-rollback doctrine): the vertex imprint
    // is correct on span-mismatch rims (z90/x13y29/y30/z63 measured -13) but exposes
    // 3-flank knots elsewhere (x20/z30 measured +9); no static discriminator exists.
    // Run the first weld pass BOTH ways and keep whichever leaves fewer naked edges.
    if (std::getenv("SESSION_M3")) {
        auto nt1 = [&]() {
            int c = 0;
            for (const auto& e : result.m_topology_edges)
                if ((int)e.trim_indices.size() == 1) ++c;
            return c;
        };
        BRep snap0 = result;
        run_xweld();
        int nk_yes = nt1();
        BRep resYes = result;
        result = snap0;
        m3_imprint_off = true;
        run_xweld();
        m3_imprint_off = false;
        int nk_no = nt1();
        if (nk_yes < nk_no) result = std::move(resYes);
        if (std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[M3TRIAL] imprint=%d plain=%d keep=%s\n",
                         nk_yes, nk_no, nk_yes < nk_no ? "imprint" : "plain");
    } else {
        run_xweld();
    }
    auto count_nt = [&](const char* tag) {
        if (!std::getenv("SESSION_NT_DBG")) return;
        int c1 = 0, c4 = 0;
        for (auto& e : result.m_topology_edges) {
            if (e.trim_indices.size() == 1) ++c1;
            if (e.trim_indices.size() >= 4) ++c4;
        }
        std::printf("[NT] %-14s edges1=%d edges4+=%d\n", tag, c1, c4);
    };
    count_nt("combine");

    // Cross-operand pave-block normalization: where the span-key alias above missed (the
    // two sides clipped one segment at different params), split both sides' copies at the
    // union of their paves and merge per block -- the BOPDS common-block guarantee.
    lap("combine");
    if (use_scaffold && !s_no_blocks) {
        // Recover spans for legacy-lifted section edges (undershoot/SEGFALL/one-sided) so EVERY
        // section edge enters the identity-based pave-block merge, not just chain-lifted ones.
        if (std::getenv("SESSION_RECOVER")) {   // opt-in: recovers spans but residual lacks mateable copies
            result.recover_section_spans(scaf, spansR);  lap("recover");
        }
        if (!spansR.empty()) {
            result.normalize_section_blocks(scaf, spansR, nullptr, "R");  lap("blocks");
            count_nt("blocks");
        }
    }
    // Resolve T-junctions first: a face whose boundary run was split (e.g. by an arrangement
    // artifact) leaves a long edge on the adjacent face spanning several shorter ones. Imprint
    // splits the long edge at those interior vertices so the pieces can mate.
    // Scaffold path: imprint at pave tolerance so a boundary edge grazed by the other
    // operand's section splits at the section's pave vertices -- the pieces then pair
    // with the section edge in the post-co_refine weld pass (IsExistingPaveBlock).
    result.imprint_edges(use_scaffold ? (scaf.tol3_rep > 0 ? scaf.tol3_rep : scaf.tol3) * 0.6 : 0.0); lap("imprint_edges");
    count_nt("imprint_edges");
    // Snap under-mated section copies to the exact pair-once section geometry: the two
    // operands' independent lifts of the same section diverge past the sew tolerance in
    // grazing regions; after the snap both carry the same polyline and sew mates them.
    if (imported_freeform && !use_scaffold && !sec_c3ds.empty() && !std::getenv("SESSION_NO_SECSNAP")) {
        result.snap_section_edges(sec_c3ds);                  lap("secsnap");
        count_nt("secsnap");
    }
    // BUILDSPEC P0: shared section-edge backbone, gated by SESSION_BOOL_SHARED_EDGES. When set,
    // recompute the A&B section curve ONCE and make each operand's section arcs reference the
    // EXACT sub-arc of that single curve, then merge -> one shared edge per section (watertight by
    // construction). When UNSET, the proven legacy path (co-refine + Hausdorff sew) runs UNCHANGED.
    static const bool s_shared = (std::getenv("SESSION_BOOL_SHARED_EDGES") != nullptr);
    if (s_shared) {
        result.make_shared_section_edges(*this, other);       lap("shared_section");
        return result;
    }
    // Co-refine the A<->B section: where one operand imprinted the shared curve as a single closed
    // circle and the other as 2+ arcs (periodic-seam straddle) -- or as partially-overlapping arcs --
    // split the longer at the shorter's endpoints so they mate 1:1. Strictly coincidence-gated, so
    // each solid's own edges are untouched. This is the OCCT "shared section edge" guarantee done as
    // a co-refinement: after it, sew merges segments that are arc-for-arc identical.
    if (!std::getenv("SESSION_NO_COREFINE")) result.co_refine_coincident_edges();
    lap("co_refine");
    count_nt("co_refine");
    // Second weld pass: co_refine has just split partially-overlapping coincident pairs
    // 1:1 (grazing section vs the other operand's boundary edge); with the Hausdorff sew
    // retired, THIS is where those refined pairs merge (tolerance-model common block).
    run_xweld();
    count_nt("xweld2");
    // Geometric sewing: the intersection curve is imprinted independently on A and B (a
    // closed self-loop on one, a seam-anchored loop on the other), so their edges share no
    // endpoints and the position-keyed emap cannot mate them. Merge edges whose 3D curves
    // trace the SAME point set (Hausdorff ~ 0), regardless of start point / discretization,
    // into one mated edge -- helps make the result a watertight solid.
    // SESSION_NO_SEW (BOP2 measurement gate): skip the Hausdorff sew entirely to expose the
    // exact residual the construction-time shared-edge path must close by itself.
    if (!std::getenv("SESSION_NO_SEW")) result.sew_coincident_edges();  lap("sew");
    count_nt("sew");
    // FUZZY FALLBACK (OCCT SetFuzzyValue analog). At sub-tolerance grazing two surfaces are
    // within tolerance over an extended region, so each operand trims its face along a
    // slightly different curve and their kept boundaries stagger -- leaving a thin naked rim
    // that no EXACT method closes (OCCT's own booleans are unsound on exactly these inputs).
    // The production answer is to trade exactness for closure: re-run the mating passes with
    // an inflated tolerance that snaps the near-coincident boundaries together. Applied ONLY
    // as a fallback (the exact result already left naked edges) and ONLY when explicitly
    // requested (SESSION_FUZZY=<multiplier>, e.g. 4 => tol = diag*5e-3*4), because it is
    // approximate BY DESIGN and must never silently fire on a case the exact path solved.
    if (const char* fz = std::getenv("SESSION_FUZZY")) {
        int nk_before = 0;
        for (const auto& e : result.m_topology_edges)
            if ((int)e.trim_indices.size() == 1) ++nk_before;
        if (nk_before > 0) {
            double mult = std::atof(fz); if (mult <= 0) mult = 4.0;
            double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
            for (const auto& p : result.m_vertices) {
                xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
                xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
            }
            double diag = result.m_vertices.empty() ? 1.0 :
                std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
            if (diag <= 0) diag = 1.0;
            double ftol = diag * 5e-3 * mult;
            result.co_refine_coincident_edges(ftol);
            result.sew_coincident_edges(ftol);
            int nk_after = 0;
            for (const auto& e : result.m_topology_edges)
                if ((int)e.trim_indices.size() == 1) ++nk_after;
            if (std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[FUZZY] mult=%.1f ftol=%.4f naked %d -> %d\n",
                             mult, ftol, nk_before, nk_after);
            lap("fuzzy");
            count_nt("fuzzy");
        }
    }
    // AUTOMATIC FUZZY LADDER (OCCT fuzzy-value auto-escalation, Law 11 tolerant/fat edge).
    // If the exact mating left naked edges (grazing/near-tangent: the two operands trimmed along
    // slightly different curves and a FIXED tolerance cannot cover their gap), escalate the mating
    // tolerance through a geometric ladder and STOP at the MINIMAL multiplier that closes the shell
    // (fewest merges = least over-approximation). Approximate BY DESIGN -- a valid TOLERANT solid,
    // which is the correct answer for a grazing contact (an exact transversal section is ill-posed
    // there). No-op when already closed (base/matrix/edge/z30x20 keep naked=0 -> ladder never runs),
    // so it cannot regress any watertight cell. Manual SESSION_FUZZY overrides (skips the ladder);
    // SESSION_NO_FUZZLADDER disables it entirely.
    if (use_scaffold && !std::getenv("SESSION_FUZZY") && std::getenv("SESSION_FUZZLADDER")) {
        auto naked_now = [&]() {
            int n = 0;
            for (const auto& e : result.m_topology_edges)
                if ((int)e.trim_indices.size() == 1) ++n;
            return n;
        };
        int nk0 = naked_now();
        if (nk0 > 0) {
            double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
            for (const auto& p : result.m_vertices) {
                xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
                xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
            }
            double diag = result.m_vertices.empty() ? 1.0 :
                std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
            if (diag <= 0) diag = 1.0;
            static const double s_cap = [] {
                const char* e = std::getenv("SESSION_FUZZLADDER_CAP");
                double v = e ? std::atof(e) : 16.0;
                return v > 0.0 ? v : 16.0;
            }();
            const double mults[] = {1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 24.0, 32.0};
            int nk = nk0;
            for (double mult : mults) {
                if (mult > s_cap) break;
                double ftol = diag * 5e-3 * mult;
                result.co_refine_coincident_edges(ftol);
                result.sew_coincident_edges(ftol);
                nk = naked_now();
                if (std::getenv("SESSION_NT_DBG"))
                    std::fprintf(stderr, "[FUZZLADDER] mult=%.1f ftol=%.4f naked %d -> %d\n",
                                 mult, ftol, nk0, nk);
                if (nk == 0) break;   // closed at the minimal tolerance
            }
            lap("fuzzladder");
        }
    }
    // P4/CAPFILL (SESSION_CAPFILL): synthesize the ABSENT mating cap. A naked (1-trim) section edge is
    // A's (or B's) section boundary whose opposite flank -- the other operand's inside cap fragment --
    // was never produced by the freeform SSI (workflow-confirmed: the cap face is absent, not dropped).
    // Since the edge already carries its scaffold identity (spansR: seg,fa,fb) and the scaffold holds the
    // section's uv ON THE OTHER OPERAND'S SURFACE (surfB/uvB), we build the cap face directly: one face
    // per (cap-side, cap-surface), each naked edge getting a second trim whose 2D pcurve is that seg's
    // uv sub-arc. The edge becomes 2-trim by construction. Gated OFF; base/matrix/edge never enter it.
    if (use_scaffold && std::getenv("SESSION_CAPFILL")) {
        // Build the ABSENT mating cap as a CLOSED face: collect the naked (1-trim) edges, chain them
        // into closed loops by shared vertices, and for each loop lay a face on the OTHER operand's
        // surface (the one the section lies on) with the loop's edges as an ORDERED closed boundary
        // (each edge's 2D pcurve = its 3D curve projected onto that surface). An open 2-edge loop is
        // invalid -> OCCT reports BadOrientation and a meaningless volume; a full closed loop is a real
        // cap. Cap surface chosen by projecting the loop and picking the operand surface it lies on.
        int ncapf = 0, ncape = 0;
        double cap_diag; {
            double mn[3] = {1e300,1e300,1e300}, mx[3] = {-1e300,-1e300,-1e300};
            for (const auto& p : result.m_vertices) for (int k = 0; k < 3; ++k) { mn[k]=std::min(mn[k],p[k]); mx[k]=std::max(mx[k],p[k]); }
            cap_diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0])+(mx[1]-mn[1])*(mx[1]-mn[1])+(mx[2]-mn[2])*(mx[2]-mn[2]));
            if (!(cap_diag > 0)) cap_diag = 1.0;
        }
        std::vector<int> nak;
        for (int e = 0; e < (int)result.m_topology_edges.size(); ++e)
            if ((int)result.m_topology_edges[e].trim_indices.size() == 1
                && result.m_topology_edges[e].curve_3d_index >= 0) nak.push_back(e);
        // Junction weld (mate-of-scoped): the two operands' copies of a shared junction vertex sit ~0.02
        // apart (distinct topology vertices), so the section loop is not vertex-connected and cannot be
        // traced. Weld naked-edge endpoints within a small tolerance onto ONE representative so the loop
        // closes. Scoped to naked (1-trim) edge endpoints only -> cannot disturb watertight topology.
        {
            double wtol = cap_diag * 3e-3;
            auto vpos = [&](int v) { return result.m_vertices[result.m_topology_vertices[v].point_index]; };
            std::vector<int> vs;
            for (int e : nak) { int a = result.m_topology_edges[e].start_vertex,
                                    b = result.m_topology_edges[e].end_vertex;
                                if (a >= 0) vs.push_back(a); if (b >= 0) vs.push_back(b); }
            std::sort(vs.begin(), vs.end()); vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
            std::map<int,int> rep;
            for (size_t i = 0; i < vs.size(); ++i) {
                int vi = vs[i]; if (rep.count(vi)) continue; rep[vi] = vi;
                for (size_t j = 0; j < i; ++j) {
                    int vj = vs[j];
                    if (vpos(vi).distance(vpos(vj)) < wtol) { rep[vi] = rep[vj]; break; }
                }
            }
            for (int e : nak) {
                auto& E = result.m_topology_edges[e];
                if (rep.count(E.start_vertex)) E.start_vertex = rep[E.start_vertex];
                if (rep.count(E.end_vertex))   E.end_vertex   = rep[E.end_vertex];
            }
        }
        // adjacency by shared topology vertex
        auto ev = [&](int e) { return std::make_pair(result.m_topology_edges[e].start_vertex,
                                                     result.m_topology_edges[e].end_vertex); };
        std::map<int, std::vector<int>> byv;   // vertex -> naked edges
        for (int e : nak) { auto [a, b] = ev(e); if (a >= 0) byv[a].push_back(e); if (b >= 0) byv[b].push_back(e); }
        std::set<int> used;
        for (int e0 : nak) {
            if (used.count(e0)) continue;
            // trace a chain from e0 following shared vertices
            std::vector<int> loop; std::vector<int> vseq;
            int cur = e0, prevv = ev(e0).first;
            for (int guard = 0; guard < (int)nak.size() + 2; ++guard) {
                if (used.count(cur)) break;
                used.insert(cur); loop.push_back(cur);
                auto [a, b] = ev(cur);
                int nextv = (a == prevv) ? b : a;      // exit vertex
                vseq.push_back(nextv);
                int nxt = -1;
                for (int cand : byv[nextv]) if (cand != cur && !used.count(cand)) { nxt = cand; break; }
                if (nxt < 0) break;
                prevv = nextv; cur = nxt;
            }
            if ((int)loop.size() < 3) continue;         // need a real loop, not a stub
            // closed? last exit vertex == first entry vertex
            bool closed = (!vseq.empty() && vseq.back() == ev(loop[0]).first);
            if (!closed) continue;
            // choose the cap surface: the OTHER-operand surface the loop lies on (min projected dist)
            std::vector<Point> mids;
            for (int e : loop) {
                const NurbsCurve& C = result.m_curves_3d[result.m_topology_edges[e].curve_3d_index];
                auto d = C.domain(); mids.push_back(C.point_at(0.5 * (d.first + d.second)));
            }
            int nAf = (int)subA.m_faces.size();
            int ti0 = result.m_topology_edges[loop[0]].trim_indices[0];
            int li0 = result.m_trims[ti0].loop_index;
            int fi0 = (li0 >= 0) ? result.m_loops[li0].face_index : -1;
            bool existing_is_A = (fi0 >= 0 && fi0 < nAf);
            const std::vector<NurbsSurface>& src = existing_is_A ? other.m_surfaces : m_surfaces;
            int best = -1; double bestd = 1e300;
            for (int si = 0; si < (int)src.size(); ++si) {
                double dsum = 0;
                for (const Point& p : mids) { auto [u, v, d] = Closest::surface_point(src[si], p); (void)u;(void)v; dsum += d; }
                if (dsum < bestd) { bestd = dsum; best = si; }
            }
            if (best < 0 || bestd > mids.size() * (cap_diag * 5e-3)) continue;   // not clearly on any surface
            int sidx = result.add_surface(src[best]);
            int fidx = result.add_face(sidx, false);
            int lidx = result.add_loop(fidx, BRepLoopType::Outer);
            for (size_t k = 0; k < loop.size(); ++k) {
                int e = loop[k];
                const NurbsCurve& C = result.m_curves_3d[result.m_topology_edges[e].curve_3d_index];
                auto d = C.domain();
                // project the edge's 3D curve onto the cap surface; direction so start->end matches vseq
                bool fwd = (ev(e).second == vseq[k]);
                std::vector<Point> uvpts;
                const int NSAMP = 12;
                for (int j = 0; j <= NSAMP; ++j) {
                    double f = fwd ? (double)j / NSAMP : 1.0 - (double)j / NSAMP;
                    Point p3 = C.point_at(d.first + (d.second - d.first) * f);
                    auto [u, v, dd] = Closest::surface_point(src[best], p3); (void)dd;
                    uvpts.push_back(Point(u, v, 0.0));
                }
                NurbsCurve pc = NurbsCurve::create(false, 1, uvpts);
                if (!pc.is_valid()) continue;
                int c2 = result.add_curve_2d(pc);
                result.add_trim(c2, e, lidx, !fwd, BRepTrimType::Mated);
                ++ncape;
            }
            ++ncapf;
        }
        if (std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[CAPFILL] closed cap faces=%d edges capped=%d (naked in=%zu)\n",
                         ncapf, ncape, nak.size());
        lap("capfill");
    }
    // WIRE-ORIENTATION REPAIR (SESSION_WIRE_REPAIR): a fuzzy sew / cap can leave a face whose OUTER loop
    // winds CW in uv (or inner CW->CCW), which OCCT flags BadOrientationOfSubshape even when the face's
    // outward SIGN is right (so the relsgn relaxation cannot fix it). Enforce the check_trim_orientation
    // invariant (brep.cpp:1222) directly: reverse any loop that violates OUTER=CCW / inner=CW by
    // reversing its trim order and toggling each trim's `reversed`. Self-contained per face; gated OFF.
    if (use_scaffold && std::getenv("SESSION_WIRE_REPAIR")) {
        int nrev = 0;
        for (int fi = 0; fi < (int)result.m_faces.size(); ++fi) {
            const BRepFace& face = result.m_faces[fi];
            if (face.surface_index < 0 || face.surface_index >= (int)result.m_surfaces.size()) continue;
            auto du = result.m_surfaces[face.surface_index].domain(0);
            auto dv = result.m_surfaces[face.surface_index].domain(1);
            double ru = du.second - du.first, rv = dv.second - dv.first;
            double tol_uv = 1e-6 * std::max(ru, rv), jump_uv = 0.1 * std::max(ru, rv);
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)result.m_loops.size()) continue;
                BRepLoop& loop = result.m_loops[li];
                double area2 = 0.0; Point aprev(0,0,0); bool afirst = true, jumpy = false;
                Point prev_pt(0,0,0), first_pt(0,0,0); bool have_prev = false;
                for (int ti : loop.trim_indices) {
                    if (ti < 0 || ti >= (int)result.m_trims.size()) continue;
                    int c2 = result.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)result.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = result.m_curves_2d[c2];
                    if (!pc.is_valid()) continue;
                    auto dc = pc.domain(); bool rev = result.m_trims[ti].reversed;
                    Point s = pc.point_at(rev ? dc.second : dc.first);
                    Point e = pc.point_at(rev ? dc.first : dc.second);
                    if (have_prev) { if (std::hypot(s[0]-prev_pt[0], s[1]-prev_pt[1]) > jump_uv) jumpy = true; }
                    else first_pt = s;
                    for (int k = 0; k <= 8; ++k) {
                        double f = k / 8.0;
                        double t = rev ? dc.second - (dc.second-dc.first)*f : dc.first + (dc.second-dc.first)*f;
                        Point p = pc.point_at(t);
                        if (!afirst) area2 += aprev[0]*p[1] - p[0]*aprev[1];
                        aprev = p; afirst = false;
                    }
                    prev_pt = e; have_prev = true;
                }
                if (!have_prev) continue;
                if (std::hypot(first_pt[0]-prev_pt[0], first_pt[1]-prev_pt[1]) > jump_uv) jumpy = true;
                area2 += aprev[0]*first_pt[1] - first_pt[0]*aprev[1];
                if (jumpy || std::abs(area2) <= tol_uv * std::max(ru, rv)) continue;  // seam-jump: skip
                bool ccw = area2 > 0.0, want_ccw = (loop.type == BRepLoopType::Outer);
                if (ccw != want_ccw) {
                    std::reverse(loop.trim_indices.begin(), loop.trim_indices.end());
                    for (int ti : loop.trim_indices)
                        if (ti >= 0 && ti < (int)result.m_trims.size())
                            result.m_trims[ti].reversed = !result.m_trims[ti].reversed;
                    ++nrev;
                }
            }
        }
        if (std::getenv("SESSION_NT_DBG")) std::fprintf(stderr, "[WIRE-REPAIR] loops reversed: %d\n", nrev);
        lap("wire_repair");
    }
    // THIRD mate pass (M3): the repair stages above (block majority, island, capfill,
    // wire repair) can drop flanks and expose fresh 1-trim pairs that the pre-repair
    // weld passes never saw (z90: pairs that only become naked after fragment drops).
    if (std::getenv("SESSION_M3") && !std::getenv("SESSION_M3_NO3RD")) { run_xweld(); count_nt("xweld3"); }
    if (std::getenv("SESSION_NT_DBG2")) {
        // ALL edges with trim counts + endpoints: mate forensics along a section rim
        for (int e = 0; e < (int)result.m_topology_edges.size(); ++e) {
            const auto& E = result.m_topology_edges[e];
            int ci = E.curve_3d_index;
            if (ci < 0 || ci >= (int)result.m_curves_3d.size()) continue;
            const NurbsCurve& C = result.m_curves_3d[ci];
            auto [t0, t1] = C.domain();
            Point e0 = C.point_at(t0), e1 = C.point_at(t1);
            std::vector<int> fs;
            for (int ti : E.trim_indices) {
                int li = (ti >= 0 && ti < (int)result.m_trims.size()) ? result.m_trims[ti].loop_index : -1;
                fs.push_back((li >= 0 && li < (int)result.m_loops.size()) ? result.m_loops[li].face_index : -1);
            }
            std::printf("[NT2] e=%d nt=%zu f=%d/%d a(%.4f,%.4f,%.4f) b(%.4f,%.4f,%.4f)\n",
                        e, E.trim_indices.size(), fs.size() > 0 ? fs[0] : -1, fs.size() > 1 ? fs[1] : -1,
                        e0[0], e0[1], e0[2], e1[0], e1[1], e1[2]);
        }
    }
    if (std::getenv("SESSION_NT_DBG")) {
        // lineage of each residual 1-trim edge: owning face + operand side + shape
        int nA = (int)subA.m_faces.size();
        for (int e = 0; e < (int)result.m_topology_edges.size(); ++e) {
            const auto& E = result.m_topology_edges[e];
            if ((int)E.trim_indices.size() != 1) continue;
            int ti = E.trim_indices[0];
            int li = (ti >= 0 && ti < (int)result.m_trims.size()) ? result.m_trims[ti].loop_index : -1;
            int fi = (li >= 0 && li < (int)result.m_loops.size()) ? result.m_loops[li].face_index : -1;
            int ci = E.curve_3d_index;
            double len = 0.0; bool closed = false;
            if (ci >= 0 && ci < (int)result.m_curves_3d.size()) {
                const NurbsCurve& C = result.m_curves_3d[ci];
                auto [t0, t1] = C.domain();
                Point pp = C.point_at(t0);
                for (int k = 1; k <= 16; ++k) { Point q = C.point_at(t0 + (t1 - t0) * k / 16.0); len += pp.distance(q); pp = q; }
                closed = C.point_at(t0).distance(C.point_at(t1)) < 1e-6;
            }
            Point e0(0,0,0), e1(0,0,0);
            if (ci >= 0 && ci < (int)result.m_curves_3d.size()) {
                const NurbsCurve& C = result.m_curves_3d[ci];
                auto [t0, t1] = C.domain();
                e0 = C.point_at(t0); e1 = C.point_at(t1);
            }
            int nk_seg = -1; double nk_fa = -1, nk_fb = -1;
            {
                auto itSp = spansR.find(e);
                if (itSp != spansR.end()) { nk_seg = (int)itSp->second[0]; nk_fa = itSp->second[1]; nk_fb = itSp->second[2]; }
            }
            // nearest other edge by mutual midpoint/endpoint distance: is the mate a
            // misaligned 1-trim, an already-2-trim edge (3-faces case), or absent?
            auto edge_poly = [&](int ee) {
                std::vector<Point> ps;
                const auto& EE = result.m_topology_edges[ee];
                if (EE.curve_3d_index >= 0 && EE.curve_3d_index < (int)result.m_curves_3d.size()) {
                    const NurbsCurve& c = result.m_curves_3d[EE.curve_3d_index];
                    auto d = c.domain();
                    for (int k = 0; k <= 8; ++k) ps.push_back(c.point_at(d.first + (d.second-d.first)*k/8.0));
                }
                return ps;
            };
            auto pd = [&](const std::vector<Point>& pa, const std::vector<Point>& pb) {
                double worst = 0;
                for (int k = 1; k < 8; k += 2) {
                    double best = 1e300;
                    for (size_t j = 0; j + 1 < pb.size(); ++j) {
                        double ex = pb[j+1][0]-pb[j][0], ey = pb[j+1][1]-pb[j][1], ez = pb[j+1][2]-pb[j][2];
                        double L2 = ex*ex+ey*ey+ez*ez;
                        double t = L2>1e-30 ? ((pa[k][0]-pb[j][0])*ex+(pa[k][1]-pb[j][1])*ey+(pa[k][2]-pb[j][2])*ez)/L2 : 0.0;
                        t = std::min(std::max(t,0.0),1.0);
                        double dx=pa[k][0]-pb[j][0]-t*ex, dy=pa[k][1]-pb[j][1]-t*ey, dz=pa[k][2]-pb[j][2]-t*ez;
                        best = std::min(best, dx*dx+dy*dy+dz*dz);
                    }
                    worst = std::max(worst, best);
                }
                return std::sqrt(worst);
            };
            std::vector<Point> mypoly = edge_poly(e);
            int best1 = -1, best2 = -1; double bd1 = 1e300, bd2 = 1e300;
            if (mypoly.size() > 2) {
                for (int e2 = 0; e2 < (int)result.m_topology_edges.size(); ++e2) {
                    if (e2 == e) continue;
                    size_t nt2 = result.m_topology_edges[e2].trim_indices.size();
                    if (nt2 == 0) continue;
                    std::vector<Point> op = edge_poly(e2);
                    if (op.size() < 3) continue;
                    double dd = pd(mypoly, op);
                    if (nt2 == 1) { if (dd < bd1) { bd1 = dd; best1 = e2; } }
                    else          { if (dd < bd2) { bd2 = dd; best2 = e2; } }
                }
                std::printf("[NKPAIR] e=%d near1(e%d d=%.4f) near2(e%d d=%.4f)\n", e, best1, bd1, best2, bd2);
            }
            std::printf("[NK] e=%d face=%d side=%c len=%.3f closed=%d seg=%d f[%.2f,%.2f] a(%.4f,%.4f,%.4f) b(%.4f,%.4f,%.4f)\n",
                        e, fi, (fi >= 0 && fi < nA) ? 'A' : 'B', len, closed ? 1 : 0,
                        nk_seg, nk_fa, nk_fb,
                        e0[0], e0[1], e0[2], e1[0], e1[1], e1[2]);
        }
    }
    // SameParameter-lite (planar pcurves rebuilt from the curved side's lifted boundary) is
    // gated OFF: it only helps once ALL section arcs are pullback-quality on both surfaces --
    // with mixed-quality copies (box x tor: e16-type arcs sit 5e-4 off the plane) the rebuilt
    // disks inherit that bias and the volume moves AWAY from truth (+9e-6 vs +1.8e-6).
    if (std::getenv("SESSION_SAMEPARAM")) { result.sameparameter_planar_pcurves(); lap("sameparam"); }
    // SEAM AUDIT (SESSION_SEAM_AUDIT): the two failure modes an independent OCCT read of the
    // exported STEP finds but every internal metric misses. (1) DEVIATION -- an edge our
    // topology calls 2-trim whose two trims trace DIFFERENT 3D curves (each trim evaluated on
    // its OWN surface): topologically closed, geometrically open (z90 class). (2) SHARING --
    // two DISTINCT edge records carrying the SAME curve (measured gap 0.0 / 4e-16 on y30),
    // each used by one face: our count sees two mated edges, the exporter writes two
    // unshared EDGE_CURVEs and the re-import is naked (y30/x13y29 class = Law 1 stitch-vs-
    // imprint in its purest observable form). Prints per-defect positions so the damage
    // cluster can be localized directly.
    // Skipped for nested AUTO variants (s_in_auto): the ladder would pay for it 8x and the
    // only result that matters is the one AUTO returns, which is audited in that branch.
    if (std::getenv("SESSION_SEAM_AUDIT") && !s_in_auto) brep_seam_audit(result, tolerance);
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Meshing
///////////////////////////////////////////////////////////////////////////////////////////

Mesh BRep::mesh() const {
    int nf = (int)m_faces.size();

    // Phase 1: Classify faces as direct (RemeshNurbsSurfaceGrid) or CDT
    std::vector<bool> face_direct(nf, false);
    for (int fi = 0; fi < nf; ++fi) {
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        bool has_inner = false, all_linear = true;
        std::vector<Point> outer_pts;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            if (loop.type == BRepLoopType::Inner) has_inner = true;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                if (crv.degree() > 1 || crv.is_rational()) all_linear = false;
                if (loop.type == BRepLoopType::Outer) {
                    if (crv.degree() <= 1 && !crv.is_rational())
                        for (int k = 0; k < crv.cv_count() - 1; ++k) outer_pts.push_back(crv.get_cv(k));
                }
            }
        }
        bool direct = !has_inner && all_linear;
        if (direct && !outer_pts.empty()) {
            auto [u0, u1] = srf.domain(0);
            auto [v0, v1] = srf.domain(1);
            double tol = std::max(u1 - u0, v1 - v0) * 0.01;
            double bb_umin = 1e30, bb_umax = -1e30, bb_vmin = 1e30, bb_vmax = -1e30;
            for (const auto& p : outer_pts) {
                bb_umin = std::min(bb_umin, p[0]); bb_umax = std::max(bb_umax, p[0]);
                bb_vmin = std::min(bb_vmin, p[1]); bb_vmax = std::max(bb_vmax, p[1]);
            }
            if (std::abs(bb_umin - u0) > tol || std::abs(bb_umax - u1) > tol ||
                std::abs(bb_vmin - v0) > tol || std::abs(bb_vmax - v1) > tol)
                direct = false;
        }
        face_direct[fi] = direct;
    }

    // Phase 2: Mesh direct faces, extract boundary 3D points for shared edges
    std::vector<Mesh> fmesh(nf);
    std::map<int, std::vector<Point>> edge_bnd;

    for (int fi = 0; fi < nf; ++fi) {
        if (!face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        fmesh[fi] = srf.mesh();

        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        double utol = (u1 - u0) * 0.001, vtol = (v1 - v0) * 0.001;

        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int eidx = m_trims[ti].edge_index;
                if (eidx < 0 || eidx >= (int)m_topology_edges.size()) continue;
                if (edge_bnd.count(eidx)) continue;

                // Only extract if edge is shared with a CDT face
                bool shared = false;
                for (int oti : m_topology_edges[eidx].trim_indices) {
                    if (oti == ti) continue;
                    int oli = m_trims[oti].loop_index;
                    if (oli < 0 || oli >= (int)m_loops.size()) continue;
                    int ofi = m_loops[oli].face_index;
                    if (ofi >= 0 && ofi < nf && !face_direct[ofi]) { shared = true; break; }
                }
                if (!shared) continue;

                // Determine which UV boundary this trim lies on
                int c2di = m_trims[ti].curve_2d_index;
                if (c2di < 0 || c2di >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& c2d = m_curves_2d[c2di];
                Point sp = c2d.get_cv(0), ep = c2d.get_cv(c2d.cv_count() - 1);

                bool at_v0 = std::abs(sp[1] - v0) < vtol && std::abs(ep[1] - v0) < vtol;
                bool at_v1 = std::abs(sp[1] - v1) < vtol && std::abs(ep[1] - v1) < vtol;
                bool at_u0 = std::abs(sp[0] - u0) < utol && std::abs(ep[0] - u0) < utol;
                bool at_u1 = std::abs(sp[0] - u1) < utol && std::abs(ep[0] - u1) < utol;
                if (!at_v0 && !at_v1 && !at_u0 && !at_u1) continue;

                std::vector<std::pair<double, Point>> pts;
                for (auto& [vk, vd] : fmesh[fi].vertex) {
                    auto iu = vd.attributes.find("u"), iv = vd.attributes.find("v");
                    if (iu == vd.attributes.end() || iv == vd.attributes.end()) continue;
                    if (at_v0 && std::abs(iv->second - v0) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_v1 && std::abs(iv->second - v1) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_u0 && std::abs(iu->second - u0) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                    else if (at_u1 && std::abs(iu->second - u1) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                }
                std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                if (pts.size() >= 2) {
                    std::vector<Point> bnd;
                    for (auto& [p, pt] : pts) bnd.push_back(pt);
                    edge_bnd[eidx] = bnd;
                }
            }
        }
    }

    // Phase 3: Mesh CDT faces, using matched boundary points for shared edges
    for (int fi = 0; fi < nf; ++fi) {
        if (face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];

        // Bilinear 3D→UV projection (works for planar cap surfaces)
        Point p00 = srf.get_cv(0, 0), p10 = srf.get_cv(1, 0), p01 = srf.get_cv(0, 1);
        double eux = p10[0]-p00[0], euy = p10[1]-p00[1], euz = p10[2]-p00[2];
        double evx = p01[0]-p00[0], evy = p01[1]-p00[1], evz = p01[2]-p00[2];
        double eu2 = eux*eux+euy*euy+euz*euz, ev2 = evx*evx+evy*evy+evz*evz;
        bool can_project = (srf.degree(0) == 1 && srf.degree(1) == 1 && eu2 > 1e-28 && ev2 > 1e-28);

        NurbsSurfaceTrimmed ts;
        ts.m_surface = srf;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            std::vector<Point> loop_pts;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.type == BRepTrimType::Singular) continue;
                int eidx = trim.edge_index;

                if (can_project && eidx >= 0 && edge_bnd.count(eidx)) {
                    // Project matched 3D boundary points to this face's UV
                    const auto& bnd = edge_bnd[eidx];
                    for (const auto& pt : bnd) {
                        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
                        double u = (dx*eux+dy*euy+dz*euz) / eu2;
                        double v = (dx*evx+dy*evy+dz*evz) / ev2;
                        loop_pts.push_back(Point(u, v, 0));
                    }
                } else {
                    if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                    if (crv.degree() <= 1 && !crv.is_rational()) {
                        for (int k = 0; k < crv.cv_count() - 1; ++k)
                            loop_pts.push_back(crv.get_cv(k));
                    } else {
                        int n = std::max(crv.cv_count() * 4, 16);
                        auto [pts, params] = crv.divide_by_count(n);
                        for (int k = 0; k < (int)pts.size() - 1; ++k)
                            loop_pts.push_back(pts[k]);
                    }
                }
            }
            if (loop_pts.size() >= 3) {
                NurbsCurve loop_crv = NurbsCurve::create(true, 1, loop_pts);
                if (loop.type == BRepLoopType::Outer)
                    ts.m_outer_loop = loop_crv;
                else
                    ts.m_inner_loops.push_back(loop_crv);
            }
        }
        fmesh[fi] = ts.mesh();
    }

    // Phase 4: Combine all face meshes
    std::vector<std::vector<Point>> all_polygons;
    for (int fi = 0; fi < nf; ++fi) {
        Mesh& fm = fmesh[fi];
        if (fm.is_empty()) continue;
        const auto& face = m_faces[fi];
        // Reversed faces must have their triangle winding flipped so the facet
        // orientation matches the face's outward normal (from_polylines rebuilds
        // vertices from positions, so flipping per-vertex normals here has no effect).
        for (auto& [fk, fverts] : fm.face) {
            std::vector<Point> poly;
            for (auto vi : fverts)
                poly.push_back(fm.vertex.at(vi).position());
            if (face.reversed)
                std::reverse(poly.begin(), poly.end());
            all_polygons.push_back(poly);
        }
    }
    return Mesh::from_polylines(all_polygons, 1e-6);
}// Returns one tessellated Mesh per BRep face, in face order. Vertices are NOT
// shared across faces so face boundaries are hard edges.
std::vector<Mesh> BRep::face_meshes() const {
    return face_meshes_q(false, 0.0, 0.0);
}

// Per-face meshes with an optional tessellation-quality override applied to the
// grid-meshed (direct) faces: when has_quality is true, (max_angle_deg, chord_factor)
// densifies them (and, via the shared-edge coordination, the CDT faces follow).
// When has_quality is false, the default NurbsSurface::mesh() density is used.
std::vector<Mesh> BRep::face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const {
    int nf = (int)m_faces.size();

    // Phase 1: classify faces as direct (RemeshNurbsSurfaceGrid) or CDT
    std::vector<bool> face_direct(nf, false);
    for (int fi = 0; fi < nf; ++fi) {
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        bool has_inner = false, all_linear = true;
        std::vector<Point> outer_pts;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            if (loop.type == BRepLoopType::Inner) has_inner = true;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                if (crv.degree() > 1 || crv.is_rational()) all_linear = false;
                if (loop.type == BRepLoopType::Outer) {
                    if (crv.degree() <= 1 && !crv.is_rational())
                        for (int k = 0; k < crv.cv_count() - 1; ++k) outer_pts.push_back(crv.get_cv(k));
                }
            }
        }
        bool direct = !has_inner && all_linear;
        if (direct && !outer_pts.empty()) {
            auto [u0, u1] = srf.domain(0);
            auto [v0, v1] = srf.domain(1);
            double tol = std::max(u1 - u0, v1 - v0) * 0.01;
            double bb_umin = 1e30, bb_umax = -1e30, bb_vmin = 1e30, bb_vmax = -1e30;
            for (const auto& p : outer_pts) {
                bb_umin = std::min(bb_umin, p[0]); bb_umax = std::max(bb_umax, p[0]);
                bb_vmin = std::min(bb_vmin, p[1]); bb_vmax = std::max(bb_vmax, p[1]);
            }
            if (std::abs(bb_umin - u0) > tol || std::abs(bb_umax - u1) > tol ||
                std::abs(bb_vmin - v0) > tol || std::abs(bb_vmax - v1) > tol)
                direct = false;
        }
        face_direct[fi] = direct;
    }

    // Phase 2: direct faces. Mesh each via the grid mesher, then record the 3D
    // boundary discretisation along every edge shared with a CDT face.
    std::vector<Mesh> fmesh(nf);
    std::map<int, std::vector<Point>> edge_bnd;
    for (int fi = 0; fi < nf; ++fi) {
        if (!face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        fmesh[fi] = has_quality
            ? RemeshNurbsSurfaceGrid::from_u_v_q(srf, 0, 0, max_angle_deg, chord_factor)
            : srf.mesh();

        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        double utol = (u1 - u0) * 0.001, vtol = (v1 - v0) * 0.001;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int eidx = m_trims[ti].edge_index;
                if (eidx < 0 || eidx >= (int)m_topology_edges.size()) continue;
                if (edge_bnd.count(eidx)) continue;
                // Only extract if this edge is shared with a CDT (non-direct) face.
                bool shared = false;
                for (int oti : m_topology_edges[eidx].trim_indices) {
                    if (oti == ti || oti < 0 || oti >= (int)m_trims.size()) continue;
                    int oli = m_trims[oti].loop_index;
                    if (oli < 0 || oli >= (int)m_loops.size()) continue;
                    int ofi = m_loops[oli].face_index;
                    if (ofi >= 0 && ofi < nf && !face_direct[ofi]) { shared = true; break; }
                }
                if (!shared) continue;
                int c2di = m_trims[ti].curve_2d_index;
                if (c2di < 0 || c2di >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& c2d = m_curves_2d[c2di];
                Point sp = c2d.get_cv(0), ep = c2d.get_cv(c2d.cv_count() - 1);
                bool at_v0 = std::abs(sp[1] - v0) < vtol && std::abs(ep[1] - v0) < vtol;
                bool at_v1 = std::abs(sp[1] - v1) < vtol && std::abs(ep[1] - v1) < vtol;
                bool at_u0 = std::abs(sp[0] - u0) < utol && std::abs(ep[0] - u0) < utol;
                bool at_u1 = std::abs(sp[0] - u1) < utol && std::abs(ep[0] - u1) < utol;
                if (!at_v0 && !at_v1 && !at_u0 && !at_u1) continue;
                std::vector<std::pair<double, Point>> pts;
                for (auto& [vk, vd] : fmesh[fi].vertex) {
                    auto iu = vd.attributes.find("u"), iv = vd.attributes.find("v");
                    if (iu == vd.attributes.end() || iv == vd.attributes.end()) continue;
                    if (at_v0 && std::abs(iv->second - v0) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_v1 && std::abs(iv->second - v1) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_u0 && std::abs(iu->second - u0) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                    else if (at_u1 && std::abs(iu->second - u1) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                }
                std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                if (pts.size() >= 2) {
                    std::vector<Point> bnd;
                    for (auto& [p, pt] : pts) bnd.push_back(pt);
                    edge_bnd[eidx] = bnd;
                }
            }
        }
    }

    // Phase 3: Mesh CDT faces via NurbsSurfaceTrimmed. For edges shared with a direct
    // face, reuse that face's boundary points (projected into this bilinear face's UV).
    for (int fi = 0; fi < nf; ++fi) {
        if (face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        // Bilinear 3D->UV projection frame (valid for the planar cap surfaces).
        Point p00 = srf.get_cv(0, 0), p10 = srf.get_cv(1, 0), p01 = srf.get_cv(0, 1);
        double eux = p10[0]-p00[0], euy = p10[1]-p00[1], euz = p10[2]-p00[2];
        double evx = p01[0]-p00[0], evy = p01[1]-p00[1], evz = p01[2]-p00[2];
        double eu2 = eux*eux+euy*euy+euz*euz, ev2 = evx*evx+evy*evy+evz*evz;
        bool can_project = (srf.degree(0) == 1 && srf.degree(1) == 1 && eu2 > 1e-28 && ev2 > 1e-28);

        NurbsSurfaceTrimmed ts;
        ts.m_surface = srf;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            std::vector<Point> loop_pts;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.type == BRepTrimType::Singular) continue;
                int eidx = trim.edge_index;
                if (can_project && eidx >= 0 && edge_bnd.count(eidx)) {
                    const auto& bnd = edge_bnd[eidx];
                    for (const auto& pt : bnd) {
                        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
                        double u = (dx*eux+dy*euy+dz*euz) / eu2;
                        double v = (dx*evx+dy*evy+dz*evz) / ev2;
                        loop_pts.push_back(Point(u, v, 0));
                    }
                } else {
                    if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                    if (crv.degree() <= 1 && !crv.is_rational()) {
                        for (int k = 0; k < crv.cv_count() - 1; ++k)
                            loop_pts.push_back(crv.get_cv(k));
                    } else {
                        int n = std::max(crv.cv_count() * 4, 16);
                        auto [pts, params] = crv.divide_by_count(n, true);
                        for (int k = 0; k < (int)pts.size() - 1; ++k)
                            loop_pts.push_back(pts[k]);
                    }
                }
            }
            if (loop_pts.size() >= 3) {
                NurbsCurve loop_crv = NurbsCurve::create(true, 1, loop_pts);
                if (loop.type == BRepLoopType::Outer)
                    ts.m_outer_loop = loop_crv;
                else
                    ts.m_inner_loops.push_back(loop_crv);
            }
        }
        fmesh[fi] = ts.mesh();
    }

    // Apply reversed flag: flip BOTH winding and normals so the shader's gl_FrontFacing
    // derivation agrees with the stored vertex normals.
    for (int fi = 0; fi < nf; ++fi) {
        if (m_faces[fi].reversed) {
            fmesh[fi].flip();
            for (auto& [vk, vd] : fmesh[fi].vertex) {
                auto n = vd.normal();
                if (n) vd.set_normal(-(*n)[0], -(*n)[1], -(*n)[2]);
            }
        }
    }

    return fmesh;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point BRep::point_at(int face_idx, double u, double v) const {
    if (face_idx < 0 || face_idx >= (int)m_faces.size()) return Point();
    int si = m_faces[face_idx].surface_index;
    if (si < 0 || si >= (int)m_surfaces.size()) return Point();
    return m_surfaces[si].point_at(u, v);
}

Vector BRep::normal_at(int face_idx, double u, double v) const {
    if (face_idx < 0 || face_idx >= (int)m_faces.size()) return Vector();
    int si = m_faces[face_idx].surface_index;
    if (si < 0 || si >= (int)m_surfaces.size()) return Vector();
    Vector n = m_surfaces[si].normal_at(u, v);
    if (m_faces[face_idx].reversed) return Vector(-n[0], -n[1], -n[2]);
    return n;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void BRep::transform(const Xform& xform) {
    for (auto& srf : m_surfaces) {
        srf.transform(xform);
    }
    for (auto& crv : m_curves_3d) {
        crv.transform(xform);
    }
    // Xform.m is COLUMN-major (see Xform::transform_point). The old manual row-major
    // multiply here applied the TRANSPOSED rotation about the origin and dropped the
    // translation entirely -- every transformed() BRep carried a vertex table that did
    // not match its surfaces (silent until vertex identity became load-bearing).
    for (auto& pt : m_vertices)
        pt = xform.transform_point(pt);
}

BRep BRep::transformed(const Xform& xform) const {
    BRep b = *this;
    b.transform(xform);
    return b;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

static nlohmann::ordered_json trim_type_to_json(BRepTrimType t) {
    switch (t) {
        case BRepTrimType::Boundary: return "boundary";
        case BRepTrimType::Mated: return "mated";
        case BRepTrimType::Seam: return "seam";
        case BRepTrimType::Singular: return "singular";
    }
    return "boundary";
}

static BRepTrimType trim_type_from_json(const std::string& s) {
    if (s == "mated") return BRepTrimType::Mated;
    if (s == "seam") return BRepTrimType::Seam;
    if (s == "singular") return BRepTrimType::Singular;
    return BRepTrimType::Boundary;
}

static nlohmann::ordered_json loop_type_to_json(BRepLoopType t) {
    return (t == BRepLoopType::Inner) ? "inner" : "outer";
}

static BRepLoopType loop_type_from_json(const std::string& s) {
    return (s == "inner") ? BRepLoopType::Inner : BRepLoopType::Outer;
}

nlohmann::ordered_json BRep::jsondump() const {
    nlohmann::ordered_json j;
    j["curves_2d"] = nlohmann::ordered_json::array();
    for (const auto& c : m_curves_2d) j["curves_2d"].push_back(c.jsondump());
    j["curves_3d"] = nlohmann::ordered_json::array();
    for (const auto& c : m_curves_3d) j["curves_3d"].push_back(c.jsondump());
    j["faces"] = nlohmann::ordered_json::array();
    for (const auto& f : m_faces) {
        nlohmann::ordered_json fj;
        if (f.facecolor.a > 0) fj["facecolor"] = f.facecolor.jsondump();
        fj["loop_indices"] = f.loop_indices;
        fj["reversed"] = f.reversed;
        fj["surface_index"] = f.surface_index;
        j["faces"].push_back(fj);
    }
    j["guid"] = guid();
    j["loops"] = nlohmann::ordered_json::array();
    for (const auto& l : m_loops) {
        nlohmann::ordered_json lj;
        lj["face_index"] = l.face_index;
        lj["trim_indices"] = l.trim_indices;
        lj["type"] = loop_type_to_json(l.type);
        j["loops"].push_back(lj);
    }
    j["name"] = name;
    j["surfaces"] = nlohmann::ordered_json::array();
    for (const auto& s : m_surfaces) j["surfaces"].push_back(s.jsondump());
    j["surfacecolor"] = surfacecolor.jsondump();
    j["topology_edges"] = nlohmann::ordered_json::array();
    for (const auto& e : m_topology_edges) {
        nlohmann::ordered_json ej;
        ej["curve_3d_index"] = e.curve_3d_index;
        ej["end_vertex"] = e.end_vertex;
        ej["start_vertex"] = e.start_vertex;
        ej["trim_indices"] = e.trim_indices;
        j["topology_edges"].push_back(ej);
    }
    j["topology_vertices"] = nlohmann::ordered_json::array();
    for (const auto& v : m_topology_vertices) {
        nlohmann::ordered_json vj;
        vj["edge_indices"] = v.edge_indices;
        vj["point_index"] = v.point_index;
        j["topology_vertices"].push_back(vj);
    }
    j["trims"] = nlohmann::ordered_json::array();
    for (const auto& t : m_trims) {
        nlohmann::ordered_json tj;
        tj["curve_2d_index"] = t.curve_2d_index;
        tj["edge_index"] = t.edge_index;
        tj["loop_index"] = t.loop_index;
        tj["reversed"] = t.reversed;
        tj["type"] = trim_type_to_json(t.type);
        j["trims"].push_back(tj);
    }
    j["type"] = "BRep";
    j["vertices"] = nlohmann::ordered_json::array();
    for (const auto& v : m_vertices)
        j["vertices"].push_back(nlohmann::ordered_json::array({v[0], v[1], v[2]}));
    j["width"] = width;
    return j;
}

BRep BRep::jsonload(const nlohmann::json& data) {
    BRep b;
    if (data.contains("guid")) b.guid() = data["guid"];
    if (data.contains("name")) b.name = data["name"];
    if (data.contains("width")) b.width = data["width"];
    if (data.contains("surfacecolor")) b.surfacecolor = Color::jsonload(data["surfacecolor"]);
    if (data.contains("curves_2d"))
        for (const auto& c : data["curves_2d"]) b.m_curves_2d.push_back(NurbsCurve::jsonload(c));
    if (data.contains("curves_3d"))
        for (const auto& c : data["curves_3d"]) b.m_curves_3d.push_back(NurbsCurve::jsonload(c));
    if (data.contains("surfaces"))
        for (const auto& s : data["surfaces"]) b.m_surfaces.push_back(NurbsSurface::jsonload(s));
    if (data.contains("vertices"))
        for (const auto& v : data["vertices"]) b.m_vertices.push_back(Point(v[0], v[1], v[2]));
    if (data.contains("topology_vertices"))
        for (const auto& v : data["topology_vertices"]) {
            BRepVertex tv;
            tv.point_index = v["point_index"];
            tv.edge_indices = v["edge_indices"].get<std::vector<int>>();
            b.m_topology_vertices.push_back(tv);
        }
    if (data.contains("topology_edges"))
        for (const auto& e : data["topology_edges"]) {
            BRepEdge te;
            te.curve_3d_index = e["curve_3d_index"];
            te.start_vertex = e["start_vertex"];
            te.end_vertex = e["end_vertex"];
            te.trim_indices = e["trim_indices"].get<std::vector<int>>();
            b.m_topology_edges.push_back(te);
        }
    if (data.contains("trims"))
        for (const auto& t : data["trims"]) {
            BRepTrim bt;
            bt.curve_2d_index = t["curve_2d_index"];
            bt.edge_index = t["edge_index"];
            bt.loop_index = t["loop_index"];
            bt.reversed = t["reversed"];
            bt.type = trim_type_from_json(t["type"]);
            b.m_trims.push_back(bt);
        }
    if (data.contains("loops"))
        for (const auto& l : data["loops"]) {
            BRepLoop bl;
            bl.face_index = l["face_index"];
            bl.trim_indices = l["trim_indices"].get<std::vector<int>>();
            bl.type = loop_type_from_json(l["type"]);
            b.m_loops.push_back(bl);
        }
    if (data.contains("faces"))
        for (const auto& f : data["faces"]) {
            BRepFace bf;
            bf.surface_index = f["surface_index"];
            bf.loop_indices = f["loop_indices"].get<std::vector<int>>();
            bf.reversed = f["reversed"];
            if (f.contains("facecolor")) bf.facecolor = Color::jsonload(f["facecolor"]);
            b.m_faces.push_back(bf);
        }
    return b;
}

std::string BRep::file_json_dumps() const { return jsondump().dump(); }
BRep BRep::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void BRep::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

BRep BRep::file_json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

std::string BRep::pb_dumps() const {
    session_proto::BRep proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_width(width);

    for (const auto& c : m_curves_2d) {
        auto* p = proto.add_curves_2d();
        p->ParseFromString(c.pb_dumps());
    }
    for (const auto& c : m_curves_3d) {
        auto* p = proto.add_curves_3d();
        p->ParseFromString(c.pb_dumps());
    }
    for (const auto& s : m_surfaces) {
        auto* p = proto.add_surfaces();
        p->ParseFromString(s.pb_dumps());
    }
    for (const auto& v : m_vertices) {
        auto* p = proto.add_vertices();
        p->set_x(v[0]); p->set_y(v[1]); p->set_z(v[2]);
    }
    for (const auto& tv : m_topology_vertices) {
        auto* p = proto.add_topology_vertices();
        p->set_point_index(tv.point_index);
        for (int ei : tv.edge_indices) p->add_edge_indices(ei);
    }
    for (const auto& te : m_topology_edges) {
        auto* p = proto.add_topology_edges();
        p->set_curve_3d_index(te.curve_3d_index);
        p->set_start_vertex(te.start_vertex);
        p->set_end_vertex(te.end_vertex);
        for (int ti : te.trim_indices) p->add_trim_indices(ti);
    }
    for (const auto& t : m_trims) {
        auto* p = proto.add_trims();
        p->set_curve_2d_index(t.curve_2d_index);
        p->set_edge_index(t.edge_index);
        p->set_loop_index(t.loop_index);
        p->set_reversed(t.reversed);
        p->set_type(static_cast<session_proto::BRepTrimType>(t.type));
    }
    for (const auto& l : m_loops) {
        auto* p = proto.add_loops();
        for (int ti : l.trim_indices) p->add_trim_indices(ti);
        p->set_face_index(l.face_index);
        p->set_type(static_cast<session_proto::BRepLoopType>(l.type));
    }
    for (const auto& f : m_faces) {
        auto* p = proto.add_faces();
        p->set_surface_index(f.surface_index);
        for (int li : f.loop_indices) p->add_loop_indices(li);
        p->set_reversed(f.reversed);
        if (f.facecolor.a > 0) {
            auto* fc = p->mutable_facecolor();
            fc->set_r(f.facecolor.r); fc->set_g(f.facecolor.g);
            fc->set_b(f.facecolor.b); fc->set_a(f.facecolor.a);
        }
    }

    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);

    return proto.SerializeAsString();
}

BRep BRep::pb_loads(const std::string& data) {
    session_proto::BRep proto;
    proto.ParseFromString(data);
    BRep b;
    b.guid() = proto.guid();
    b.name = proto.name();
    b.width = proto.width();

    for (int i = 0; i < proto.curves_2d_size(); ++i)
        b.m_curves_2d.push_back(NurbsCurve::pb_loads(proto.curves_2d(i).SerializeAsString()));
    for (int i = 0; i < proto.curves_3d_size(); ++i)
        b.m_curves_3d.push_back(NurbsCurve::pb_loads(proto.curves_3d(i).SerializeAsString()));
    for (int i = 0; i < proto.surfaces_size(); ++i)
        b.m_surfaces.push_back(NurbsSurface::pb_loads(proto.surfaces(i).SerializeAsString()));
    for (int i = 0; i < proto.vertices_size(); ++i) {
        const auto& v = proto.vertices(i);
        b.m_vertices.push_back(Point(v.x(), v.y(), v.z()));
    }
    for (int i = 0; i < proto.topology_vertices_size(); ++i) {
        const auto& tv = proto.topology_vertices(i);
        BRepVertex bv;
        bv.point_index = tv.point_index();
        for (int j = 0; j < tv.edge_indices_size(); ++j)
            bv.edge_indices.push_back(tv.edge_indices(j));
        b.m_topology_vertices.push_back(bv);
    }
    for (int i = 0; i < proto.topology_edges_size(); ++i) {
        const auto& te = proto.topology_edges(i);
        BRepEdge be;
        be.curve_3d_index = te.curve_3d_index();
        be.start_vertex = te.start_vertex();
        be.end_vertex = te.end_vertex();
        for (int j = 0; j < te.trim_indices_size(); ++j)
            be.trim_indices.push_back(te.trim_indices(j));
        b.m_topology_edges.push_back(be);
    }
    for (int i = 0; i < proto.trims_size(); ++i) {
        const auto& t = proto.trims(i);
        BRepTrim bt;
        bt.curve_2d_index = t.curve_2d_index();
        bt.edge_index = t.edge_index();
        bt.loop_index = t.loop_index();
        bt.reversed = t.reversed();
        bt.type = static_cast<BRepTrimType>(t.type());
        b.m_trims.push_back(bt);
    }
    for (int i = 0; i < proto.loops_size(); ++i) {
        const auto& l = proto.loops(i);
        BRepLoop bl;
        bl.face_index = l.face_index();
        for (int j = 0; j < l.trim_indices_size(); ++j)
            bl.trim_indices.push_back(l.trim_indices(j));
        bl.type = static_cast<BRepLoopType>(l.type());
        b.m_loops.push_back(bl);
    }
    for (int i = 0; i < proto.faces_size(); ++i) {
        const auto& f = proto.faces(i);
        BRepFace bf;
        bf.surface_index = f.surface_index();
        for (int j = 0; j < f.loop_indices_size(); ++j)
            bf.loop_indices.push_back(f.loop_indices(j));
        bf.reversed = f.reversed();
        if (f.has_facecolor())
            bf.facecolor = Color(f.facecolor().r(), f.facecolor().g(), f.facecolor().b(), f.facecolor().a());
        b.m_faces.push_back(bf);
    }

    const auto& cp = proto.surfacecolor();
    b.surfacecolor.name = cp.name();
    b.surfacecolor.r = cp.r();
    b.surfacecolor.g = cp.g();
    b.surfacecolor.b = cp.b();
    b.surfacecolor.a = cp.a();

    return b;
}

void BRep::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

BRep BRep::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string BRep::str() const {
    return fmt::format("BRep(name={}, faces={}, edges={}, vertices={})",
                       name, face_count(), edge_count(), vertex_count());
}

std::string BRep::repr() const {
    return fmt::format("BRep(\n  name={},\n  faces={},\n  edges={},\n  vertices={},\n  solid={}\n)",
                       name, face_count(), edge_count(), vertex_count(),
                       is_solid() ? "true" : "false");
}

std::ostream& operator<<(std::ostream& os, const BRep& brep) {
    os << brep.str();
    return os;
}

} // namespace session_cpp
