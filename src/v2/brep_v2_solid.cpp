#include "brep_v2_solid.h"

#include "brep_samedomain.h"
#include "nurbscurve.h"
#include "nurbssurface.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <deque>

namespace session_cpp {
namespace v2sol {

const char* v2_alert_name(V2Alert a) {
    switch (a) {
        case V2Alert::ShellSplitterFailed: return "ShellSplitterFailed";
        case V2Alert::UnusedFaces: return "UnusedFaces";
        case V2Alert::OpenShellAcceptedAsSolid: return "OpenShellAcceptedAsSolid";
        case V2Alert::ClassifierUndecided: return "ClassifierUndecided";
        case V2Alert::AngularTieUnresolved: return "AngularTieUnresolved";
        case V2Alert::HoleWithoutParent: return "HoleWithoutParent";
        case V2Alert::NonManifoldEdge: return "NonManifoldEdge";
        case V2Alert::SelectionDoubleBooking: return "SelectionDoubleBooking";
    }
    return "?";
}

///////////////////////////////////////////////////////////////////////////////////////////
// 1. V2Box
///////////////////////////////////////////////////////////////////////////////////////////

void V2Box::add(const Point& p) {
    for (int k = 0; k < 3; ++k) {
        lo[k] = std::min(lo[k], p[k]);
        hi[k] = std::max(hi[k], p[k]);
    }
}
void V2Box::add(const V2Box& o) {
    if (o.whole) { whole = true; return; }
    if (o.is_void()) return;
    for (int k = 0; k < 3; ++k) {
        lo[k] = std::min(lo[k], o.lo[k]);
        hi[k] = std::max(hi[k], o.hi[k]);
    }
}
bool V2Box::is_out(const V2Box& o, double gap) const {
    if (whole || o.whole) return false;
    if (is_void() || o.is_void()) return true;
    for (int k = 0; k < 3; ++k) {
        if (hi[k] + gap < o.lo[k]) return true;
        if (o.hi[k] + gap < lo[k]) return true;
    }
    return false;
}
Point V2Box::center() const {
    if (is_void()) return Point(0, 0, 0);
    return Point(0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2]));
}
double V2Box::diagonal() const {
    if (is_void()) return 0.0;
    const double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

///////////////////////////////////////////////////////////////////////////////////////////
// 2. V2Topo — incidence by index only (guarantee I-1)
///////////////////////////////////////////////////////////////////////////////////////////

void V2Topo::build(const BRep& brep) {
    b = &brep;
    meshes_built = false;
    face_mesh.clear();
    face_mesh_sign.clear();

    const int nf = (int)brep.m_faces.size();
    const int ne = (int)brep.m_topology_edges.size();
    const int nt = (int)brep.m_trims.size();

    uses.assign(nt, V2EdgeUse());
    face_uses.assign(nf, {});
    edge_uses.assign(ne, {});
    edge_degenerate.assign(ne, 0);
    edge_length.assign(ne, 0.0);

    // model size from the vertex pool (used only to scale the zero-length threshold; no
    // adjacency decision reads it)
    V2Box bx;
    for (const Point& p : brep.m_vertices) bx.add(p);
    model_size = std::max(1e-9, bx.diagonal());

    // edge lengths + degeneracy: a pole/apex edge is a zero-length 3D curve
    for (int e = 0; e < ne; ++e) {
        const BRepEdge& E = brep.m_topology_edges[e];
        double len = 0.0;
        if (E.curve_3d_index >= 0 && E.curve_3d_index < (int)brep.m_curves_3d.size()) {
            const NurbsCurve& c = brep.m_curves_3d[E.curve_3d_index];
            if (c.is_valid()) {
                auto d = c.domain();
                Point prev = c.point_at(d.first);
                for (int i = 1; i <= 8; ++i) {
                    Point cur = c.point_at(d.first + (d.second - d.first) * (i / 8.0));
                    len += v2_vlen(cur - prev);
                    prev = cur;
                }
            }
        } else if (E.start_vertex >= 0 && E.end_vertex >= 0 &&
                   E.start_vertex < (int)brep.m_topology_vertices.size() &&
                   E.end_vertex < (int)brep.m_topology_vertices.size()) {
            const BRepVertex& v0 = brep.m_topology_vertices[E.start_vertex];
            const BRepVertex& v1 = brep.m_topology_vertices[E.end_vertex];
            if (v0.point_index >= 0 && v1.point_index >= 0)
                len = v2_vlen(brep.m_vertices[v1.point_index] - brep.m_vertices[v0.point_index]);
        }
        edge_length[e] = len;
        if (len <= 1e-9 * model_size) edge_degenerate[e] = 1;
    }

    for (int f = 0; f < nf; ++f) {
        for (int li : brep.m_faces[f].loop_indices) {
            if (li < 0 || li >= (int)brep.m_loops.size()) continue;
            for (int ti : brep.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= nt) continue;
                const BRepTrim& T = brep.m_trims[ti];
                if (T.edge_index < 0 || T.edge_index >= ne) continue;
                V2EdgeUse& u = uses[ti];
                u.trim = ti;
                u.edge = T.edge_index;
                u.face = f;
                u.dir = T.reversed ? -1 : 1;
                u.degenerate =
                    edge_degenerate[T.edge_index] != 0 || T.type == BRepTrimType::Singular;
                face_uses[f].push_back(ti);
                edge_uses[T.edge_index].push_back(ti);
            }
        }
    }
    // an edge every one of whose trims is Singular is degenerate even if the curve is absent
    for (int e = 0; e < ne; ++e) {
        if (edge_degenerate[e] || edge_uses[e].empty()) continue;
        bool all_sing = true;
        for (int t : edge_uses[e])
            if (brep.m_trims[t].type != BRepTrimType::Singular) { all_sing = false; break; }
        if (all_sing) edge_degenerate[e] = 1;
    }
}

bool V2Topo::is_seam(int edge, int face) const {
    int n = 0;
    for (int u : edge_uses[edge])
        if (uses[u].face == face) ++n;
    return n > 1;
}

void V2Topo::ensure_massprops() const {
    if (mp_built || !b) return;
    mp_built = true;
    mp = brep_massprops(*b, MassPropsOptions());
}

void V2Topo::ensure_meshes() const {
    if (meshes_built || !b) return;
    meshes_built = true;
    static const bool s_prof3 = (std::getenv("SESSION_V2_PROF") != nullptr);
    const auto t0 = std::chrono::high_resolution_clock::now();
    face_mesh = b->face_meshes();
    if (s_prof3) {
        const auto n = std::chrono::high_resolution_clock::now();
        std::fprintf(stderr, "[v2-prof]   topo.face_meshes %10.1f ms (%zu faces)\n",
                     std::chrono::duration<double, std::milli>(n - t0).count(),
                     face_mesh.size());
        std::fflush(stderr);
    }
    face_mesh_sign.assign(face_mesh.size(), 1.0);
    for (size_t f = 0; f < face_mesh.size(); ++f) {
        if (f >= b->m_faces.size()) continue;
        const int si = b->m_faces[f].surface_index;
        if (si < 0 || si >= (int)b->m_surfaces.size()) continue;
        auto vf = face_mesh[f].to_vertices_and_faces();
        double best = 0.0;
        Vector nt(0, 0, 0);
        Point cen(0, 0, 0);
        for (const auto& tri : vf.second)
            for (size_t k = 2; k < tri.size(); ++k) {
                const Point& a = vf.first[tri[0]];
                const Point& bb = vf.first[tri[k - 1]];
                const Point& c = vf.first[tri[k]];
                const Vector n = (bb - a).cross(c - a);
                const double m = v2_vlen(n);
                if (m > best) {
                    best = m;
                    nt = n;
                    cen = Point((a[0] + bb[0] + c[0]) / 3.0, (a[1] + bb[1] + c[1]) / 3.0,
                                (a[2] + bb[2] + c[2]) / 3.0);
                }
            }
        if (best <= 0) continue;
        const NurbsSurface& srf = b->m_surfaces[si];
        auto uv = srf.closest_parameters(cen);
        const Vector ns = srf.normal_at(uv.first, uv.second);
        face_mesh_sign[f] = (nt.dot(ns) < 0) ? -1.0 : 1.0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Connexity blocks — the grouping, purely topological
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<V2ConnexityBlock> v2_make_connexity_blocks(const V2Topo& topo,
                                                       const std::vector<V2OrientedFace>& faces) {
    std::vector<V2ConnexityBlock> out;
    if (!topo.b || faces.empty()) return out;

    std::map<int, std::vector<int>> byface;
    for (int i = 0; i < (int)faces.size(); ++i) byface[faces[i].face].push_back(i);

    std::set<int> duplicated;
    for (const auto& kv : byface)
        if (kv.second.size() > 1) duplicated.insert(kv.first);

    std::vector<char> visited(faces.size(), 0);
    for (int seed = 0; seed < (int)faces.size(); ++seed) {   // deterministic: input order
        if (visited[seed]) continue;
        V2ConnexityBlock blk;
        std::deque<int> q{seed};
        visited[seed] = 1;
        while (!q.empty()) {
            const int i = q.front();
            q.pop_front();
            blk.faces.push_back(faces[i]);
            for (int u : topo.face_uses[faces[i].face]) {
                const int e = topo.uses[u].edge;
                if (topo.edge_degenerate[e]) continue;
                for (int u2 : topo.edge_uses[e]) {
                    auto it = byface.find(topo.uses[u2].face);
                    if (it == byface.end()) continue;
                    for (int j : it->second) {
                        if (visited[j]) continue;
                        visited[j] = 1;
                        q.push_back(j);
                    }
                }
            }
        }
        for (const auto& of : blk.faces)
            if (duplicated.count(of.face)) { blk.regular = false; break; }
        if (blk.regular) {
            std::map<int, int> cnt;
            for (const auto& of : blk.faces)
                for (int u : topo.face_uses[of.face]) {
                    if (topo.uses[u].degenerate) continue;
                    ++cnt[topo.uses[u].edge];
                }
            for (const auto& kv : cnt)
                if (kv.second != 2) { blk.regular = false; break; }
        }
        std::sort(blk.faces.begin(), blk.faces.end());
        out.push_back(std::move(blk));
    }
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Closure, manifoldness, relative orientation
///////////////////////////////////////////////////////////////////////////////////////////

bool v2_shell_is_closed(const V2Topo& topo, const V2Shell& shell) {
    std::set<int> parity;
    bool has_bound = false;
    for (const auto& of : shell.faces)
        for (int u : topo.face_uses[of.face]) {
            if (topo.uses[u].degenerate) continue;
            has_bound = true;
            const int e = topo.uses[u].edge;
            if (!parity.insert(e).second) parity.erase(e);
        }
    return has_bound && parity.empty();
}

bool v2_shell_is_manifold(const V2Topo& topo, const V2Shell& shell) {
    std::map<int, int> cnt;
    for (const auto& of : shell.faces)
        for (int u : topo.face_uses[of.face]) {
            if (topo.uses[u].degenerate) continue;
            ++cnt[topo.uses[u].edge];
        }
    for (const auto& kv : cnt)
        if (kv.second != 2) return false;
    return !cnt.empty();
}

int v2_shell_orientation_defects(const V2Topo& topo, const V2Shell& shell) {
    std::map<int, std::vector<std::pair<int, int>>> occ;
    for (int i = 0; i < (int)shell.faces.size(); ++i)
        for (int u : topo.face_uses[shell.faces[i].face]) {
            if (topo.uses[u].degenerate) continue;
            occ[topo.uses[u].edge].push_back({i, u});
        }
    int bad = 0;
    for (const auto& kv : occ) {
        if (kv.second.size() != 2) continue;
        const int i0 = kv.second[0].first, i1 = kv.second[1].first;
        if (shell.faces[i0].face == shell.faces[i1].face) continue;   // seam
        const int d0 = topo.use_dir(kv.second[0].second, shell.faces[i0].reversed);
        const int d1 = topo.use_dir(kv.second[1].second, shell.faces[i1].reversed);
        if (d0 == d1) ++bad;
    }
    return bad;
}

void v2_orient_faces_on_shell(const V2Topo& topo, V2Shell& shell) {
    if (shell.faces.size() < 2) {
        shell.oriented_ok = true;
        return;
    }
    std::map<int, std::vector<int>> ef;   // edge -> distinct face positions in the shell
    for (int i = 0; i < (int)shell.faces.size(); ++i)
        for (int u : topo.face_uses[shell.faces[i].face]) {
            if (topo.uses[u].degenerate) continue;
            auto& v = ef[topo.uses[u].edge];
            if (std::find(v.begin(), v.end(), i) == v.end()) v.push_back(i);
        }

    std::vector<char> processed(shell.faces.size(), 0);
    int seed = 0;   // lowest arena face index, so the result is input-order independent
    for (int i = 1; i < (int)shell.faces.size(); ++i)
        if (shell.faces[i].face < shell.faces[seed].face) seed = i;
    processed[seed] = 1;
    std::deque<int> q{seed};
    while (!q.empty()) {
        const int i = q.front();
        q.pop_front();
        for (int u : topo.face_uses[shell.faces[i].face]) {
            if (topo.uses[u].degenerate) continue;
            const int e = topo.uses[u].edge;
            auto it = ef.find(e);
            if (it == ef.end() || it->second.size() != 2) continue;
            const int j = (it->second[0] == i) ? it->second[1] : it->second[0];
            if (j == i || processed[j]) continue;
            const int du = topo.use_dir(u, shell.faces[i].reversed);
            int uj = -1;
            for (int u2 : topo.face_uses[shell.faces[j].face])
                if (topo.uses[u2].edge == e) { uj = u2; break; }
            if (uj < 0) continue;
            const int dj = topo.use_dir(uj, shell.faces[j].reversed);
            const bool seam_i = topo.is_seam(e, shell.faces[i].face);
            const bool seam_j = topo.is_seam(e, shell.faces[j].face);
            if (du == dj && !seam_i && !seam_j) shell.faces[j].reversed = !shell.faces[j].reversed;
            processed[j] = 1;
            q.push_back(j);
        }
    }
    shell.oriented_ok = (v2_shell_orientation_defects(topo, shell) == 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
// 5. The angular selector
///////////////////////////////////////////////////////////////////////////////////////////

double v2_angle_with_ref(const Vector& d1, const Vector& d2, const Vector& dref) {
    const Vector x = d1.cross(d2);
    const double cosinus = d1.dot(d2);
    double beta = (M_PI / 2.0) * (1.0 - cosinus);
    if (x.dot(dref) < 0.0) beta = -beta;
    return beta;
}

static bool v2sol_edge_probe(const V2Topo& topo, int edge, Point& px, Vector& tangent) {
    const BRep& b = *topo.b;
    const BRepEdge& E = b.m_topology_edges[edge];
    if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)b.m_curves_3d.size()) return false;
    const NurbsCurve& c = b.m_curves_3d[E.curve_3d_index];
    if (!c.is_valid()) return false;
    auto d = c.domain();
    const double t = (1.0 - V2_PAR_T) * d.first + V2_PAR_T * d.second;
    px = c.point_at(t);
    tangent = v2_vunit(c.tangent_at(t));
    return v2_vlen(tangent) > 0.5;
}

static bool v2sol_face_normal_at(const BRep& b, int face, const Point& p, Vector& n) {
    const int si = b.m_faces[face].surface_index;
    if (si < 0 || si >= (int)b.m_surfaces.size()) return false;
    const NurbsSurface& s = b.m_surfaces[si];
    auto uv = s.closest_parameters(p);
    n = v2_vunit(s.normal_at(uv.first, uv.second));
    return v2_vlen(n) > 0.5;
}

bool v2_get_face_off(const V2Topo& topo, int ref_use, V2OrientedFace ref,
                     const std::vector<std::pair<int, V2OrientedFace>>& candidates,
                     V2OrientedFace& picked, double crit) {
    if (candidates.empty()) return false;
    const int edge = topo.uses[ref_use].edge;
    Point px;
    Vector tgt;
    picked = candidates.front().second;
    if (!v2sol_edge_probe(topo, edge, px, tgt)) return false;

    const int dref = topo.use_dir(ref_use, ref.reversed);
    const Vector d_ref_tgt = dref > 0 ? tgt : Vector(-tgt[0], -tgt[1], -tgt[2]);

    Vector n1;
    if (!v2sol_face_normal_at(*topo.b, ref.face, px, n1)) return false;
    if (ref.reversed) n1 = Vector(-n1[0], -n1[1], -n1[2]);
    const Vector b1 = v2_vunit(n1.cross(d_ref_tgt));   // into the reference face's material
    const Vector dtf = v2_vunit(n1.cross(b1));         // reference axis for the signed angles

    double angle_min = 100.0;
    bool ok = true;
    for (const auto& kv : candidates) {
        const V2OrientedFace f2 = kv.second;
        const int d2 = topo.use_dir(kv.first, f2.reversed);
        const Vector t2 = d2 > 0 ? tgt : Vector(-tgt[0], -tgt[1], -tgt[2]);
        Vector n2;
        const bool computed = v2sol_face_normal_at(*topo.b, f2.face, px, n2);
        if (f2.reversed) n2 = Vector(-n2[0], -n2[1], -n2[2]);
        const Vector b2 = computed ? v2_vunit(n2.cross(t2)) : Vector(0, 0, 0);
        double angle = computed ? v2_angle_with_ref(b1, b2, dtf) : 2.0 * M_PI;
        if (std::fabs(angle) < V2_ANGULAR) {
            if (f2 == ref) angle = M_PI;
            else if (f2.face == ref.face) angle = 2.0 * M_PI;
            else if (!computed) angle = 2.0 * M_PI;
        }
        if (std::fabs(angle) < crit || std::fabs(std::fabs(angle) - angle_min) < crit) ok = false;
        if (angle < 0) angle += 2.0 * M_PI;
        if (angle < angle_min) {
            angle_min = angle;
            picked = f2;
        }
    }
    return ok;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 6. Face probe + far-field membership (never used for adjacency)
///////////////////////////////////////////////////////////////////////////////////////////

bool v2_face_probe(const BRep& b, int face, Point& p, Vector& natural_normal, double* uu,
                   double* vv) {
    double u = 0, v = 0;
    if (!SameDomain::point_in_face(b, face, p, u, v)) return false;
    const int si = b.m_faces[face].surface_index;
    if (si < 0 || si >= (int)b.m_surfaces.size()) return false;
    natural_normal = v2_vunit(b.m_surfaces[si].normal_at(u, v));
    if (uu) *uu = u;
    if (vv) *vv = v;
    return v2_vlen(natural_normal) > 0.5;
}

struct V2SolTriSoup {
    std::vector<std::array<Point, 3>> tri;
    V2Box box;
    bool empty() const { return tri.empty(); }
};

/// Ray/triangle parity. Returns -1 when the ray is unusable (a hit lands on a triangle edge or
/// too close to the origin), so the caller can retry with a different direction.
static int v2sol_soup_parity(const V2SolTriSoup& s, const Point& p, const Vector& d, double scale) {
    int cnt = 0;
    const double eps = 1e-9 * scale;
    for (const auto& t : s.tri) {
        const Vector e1 = t[1] - t[0], e2 = t[2] - t[0];
        const Vector pv = d.cross(e2);
        const double det = e1.dot(pv);
        if (std::fabs(det) < 1e-14 * scale * scale) continue;
        const double inv = 1.0 / det;
        const Vector tv = p - t[0];
        const double u = tv.dot(pv) * inv;
        if (u < -1e-9 || u > 1.0 + 1e-9) continue;
        const Vector qv = tv.cross(e1);
        const double v = d.dot(qv) * inv;
        if (v < -1e-9 || u + v > 1.0 + 1e-9) continue;
        const double tt = e2.dot(qv) * inv;
        if (tt < eps) continue;
        if (u < 1e-7 || v < 1e-7 || u + v > 1.0 - 1e-7) return -1;   // grazing: retry
        ++cnt;
    }
    return cnt & 1;
}

static bool v2sol_soup_contains(const V2SolTriSoup& s, const Point& p) {
    if (s.empty()) return false;
    const double scale = std::max(1e-9, s.box.diagonal());
    static const double dirs[8][3] = {{1, 0, 0},           {0.37, 0.61, 0.7},
                                      {-0.5, 0.83, 0.25},  {0.11, -0.93, 0.35},
                                      {0.71, 0.13, -0.69}, {-0.83, -0.31, 0.46},
                                      {0.29, 0.42, -0.86}, {-0.17, 0.66, 0.73}};
    int votes = 0, n = 0;
    for (int i = 0; i < 8 && n < 3; ++i) {
        const Vector d = v2_vunit(Vector(dirs[i][0], dirs[i][1], dirs[i][2]));
        const int r = v2sol_soup_parity(s, p, d, scale);
        if (r < 0) continue;
        votes += r;
        ++n;
    }
    if (n == 0) return false;
    return votes * 2 > n;
}

double v2_shell_signed_volume(const V2Topo& topo, const V2Shell& shell) {
    // EXACT oriented volume: sum of per-face natural-normal fluxes, each flipped by the face's
    // orientation in the shell (growth shells positive, hole shells negative). Replaces the
    // triangle-soup sum, whose CDT tessellation of 512-sample pcurve loops cost ~80 s per call
    // site on boolean results; the Green quadrature answers the same question in milliseconds.
    topo.ensure_massprops();
    double vol = 0.0;
    for (const auto& of : shell.faces) {
        if (of.face < 0 || of.face >= (int)topo.mp.faces.size()) continue;
        const FaceMassProps& F = topo.mp.faces[of.face];
        if (F.face_index != of.face) continue;
        const double s = (topo.b->m_faces[of.face].reversed ? -1.0 : 1.0) *
                         (of.reversed ? -1.0 : 1.0);
        vol += s * F.flux / 3.0;
    }
    return vol;
}

bool v2_shell_is_hole(const V2Topo& topo, const V2Shell& shell, bool* undecided) {
    if (undecided) *undecided = false;
    const double v = v2_shell_signed_volume(topo, shell);
    double scale = shell.box.diagonal();
    if (scale <= 1e-9) {
        // degenerate box: fall back to the total face area as the size proxy
        topo.ensure_massprops();
        double a = 0.0;
        for (const auto& of : shell.faces)
            if (of.face >= 0 && of.face < (int)topo.mp.faces.size())
                a += topo.mp.faces[of.face].area;
        scale = std::sqrt(std::max(a, 1e-30));
    }
    scale = std::max(1e-12, scale);
    if (std::fabs(v) < 1e-9 * scale * scale * scale) {
        if (undecided) *undecided = true;
        return false;   // OCCT G1: an undecidable shell is treated as a growth
    }
    return v < 0.0;
}

bool v2_shell_inside(const V2Topo& topo, const V2Shell& inner, const V2Shell& outer) {
    if (inner.box.is_out(outer.box)) return false;
    // Far-field containment by closest-boundary classification on the OUTER shell's own
    // faces (contains_point_exact: no tessellation anywhere). The outer shell's faces, with
    // their shell orientations, are the boundary the inner shell is tested against.
    std::vector<int> ids;
    std::map<int, bool> rev;
    for (const auto& of : outer.faces) {
        if (of.face < 0 || of.face >= (int)topo.b->m_faces.size()) continue;
        if (std::find(ids.begin(), ids.end(), of.face) == ids.end()) {
            ids.push_back(of.face);
            rev[of.face] = of.reversed;
        }
    }
    if (ids.empty()) return false;
    BRep sub = topo.b->subset(ids);
    std::vector<double> osign(sub.m_faces.size(), 1.0);
    for (size_t k = 0; k < ids.size() && k < sub.m_faces.size(); ++k) {
        const bool fr = sub.m_faces[k].reversed;
        osign[k] = (fr ? -1.0 : 1.0) * (rev[ids[k]] ? -1.0 : 1.0);
    }
    int in = 0, out = 0;
    for (const auto& of : inner.faces) {
        Point p;
        Vector nn;
        if (!v2_face_probe(*topo.b, of.face, p, nn)) continue;
        (sub.contains_point_exact(p, osign) ? in : out)++;
        if (in + out >= 5) break;
    }
    return in > out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 7. BuilderSolid
///////////////////////////////////////////////////////////////////////////////////////////

void V2BuilderSolid::perform() {
    m_avoid.clear();
    m_shells.clear();
    m_internal.clear();
    m_solids.clear();
    m_unused.clear();
    m_alerts.clear();
    m_error = false;
    if (!m_topo || !m_topo->b || m_input.empty()) return;
    perform_shapes_to_avoid();
    perform_loops();
    perform_areas();
}

// §2.3 — remove faces that cannot participate in a closed shell; iterate, because removal
// creates new free edges.
void V2BuilderSolid::perform_shapes_to_avoid() {
    const V2Topo& topo = *m_topo;
    for (;;) {
        bool found = false;
        std::map<int, std::vector<V2OrientedFace>> mef;
        for (const auto& of : m_input) {
            if (m_avoid.count(of)) continue;
            for (int u : topo.face_uses[of.face]) {
                if (topo.uses[u].degenerate) continue;
                mef[topo.uses[u].edge].push_back(of);
            }
        }
        for (const auto& kv : mef) {
            const auto& lf = kv.second;
            if (lf.empty()) continue;
            if (lf.size() == 1) {
                found = true;
                m_avoid.insert(lf[0]);
            } else if (lf.size() == 2 && lf[0].face == lf[1].face) {
                if (topo.is_seam(kv.first, lf[0].face)) continue;
                if (lf[0] == lf[1]) continue;
                found = true;
                m_avoid.insert(lf[0]);
                m_avoid.insert(lf[1]);
            }
        }
        if (!found) break;
    }
}

// §2.4 — shells by shared-edge connexity.
void V2BuilderSolid::perform_loops() {
    const V2Topo& topo = *m_topo;
    std::vector<V2OrientedFace> start;
    for (const auto& of : m_input)
        if (!m_avoid.count(of)) start.push_back(of);
    if (start.empty()) return;

    for (const auto& blk : v2_make_connexity_blocks(topo, start)) {
        if (blk.regular) {
            V2Shell sh;
            sh.faces = blk.faces;
            v2_orient_faces_on_shell(topo, sh);
            sh.closed = v2_shell_is_closed(topo, sh);
            sh.manifold = v2_shell_is_manifold(topo, sh);
            m_shells.push_back(std::move(sh));
        } else {
            std::vector<V2Shell> pieces;
            split_block(blk, pieces);
            if (pieces.empty()) {
                V2AlertRec a;
                a.kind = V2Alert::ShellSplitterFailed;
                for (const auto& of : blk.faces) a.faces.push_back(of.face);
                a.detail = "connexity block produced no shell";
                m_alerts.push_back(a);
            }
            for (auto& p : pieces) m_shells.push_back(std::move(p));
        }
    }

    // §2.4 B.9 — faces the splitter could not use are demoted, never dropped
    std::set<V2OrientedFace> used;
    for (const auto& sh : m_shells)
        for (const auto& of : sh.faces) used.insert(of);
    for (const auto& of : m_input)
        if (!used.count(of)) m_avoid.insert(of);
    for (const auto& of : m_avoid)
        if (!used.count(of)) m_unused.push_back(of);
    if (!m_unused.empty()) {
        V2AlertRec a;
        a.kind = V2Alert::UnusedFaces;
        for (const auto& of : m_unused) a.faces.push_back(of.face);
        a.detail = "faces in no shell";
        m_alerts.push_back(a);
    }
    if (!m_avoid_internal && !m_avoid.empty()) {
        std::vector<V2OrientedFace> av(m_avoid.begin(), m_avoid.end());
        for (const auto& blk : v2_make_connexity_blocks(topo, av)) {
            V2Shell sh;
            sh.faces = blk.faces;
            sh.internal = true;
            sh.closed = v2_shell_is_closed(topo, sh);
            m_internal.push_back(std::move(sh));
        }
    }
}

// §2.4 B.6 — the non-manifold walk.
void V2BuilderSolid::split_block(const V2ConnexityBlock& blk, std::vector<V2Shell>& out) {
    const V2Topo& topo = *m_topo;
    std::vector<V2OrientedFace> pool = blk.faces;

    for (;;) {   // free-edge peeling
        std::map<int, std::vector<int>> efm;
        for (int i = 0; i < (int)pool.size(); ++i)
            for (int u : topo.face_uses[pool[i].face]) {
                if (topo.uses[u].degenerate) continue;
                efm[topo.uses[u].edge].push_back(i);
            }
        std::set<int> drop;
        for (const auto& kv : efm)
            if (kv.second.size() == 1) drop.insert(kv.second[0]);
        if (drop.empty() || pool.empty()) break;
        std::vector<V2OrientedFace> keep;
        for (int i = 0; i < (int)pool.size(); ++i)
            if (!drop.count(i)) keep.push_back(pool[i]);
        if (keep.size() == pool.size()) break;
        pool.swap(keep);
    }
    if (pool.empty()) return;

    std::map<int, int> mult;
    for (const auto& of : pool) ++mult[of.face];
    auto is_boundary = [&](const V2OrientedFace& f) { return mult[f.face] == 1; };

    std::map<int, std::vector<std::pair<int, V2OrientedFace>>> efm;
    for (const auto& of : pool)
        for (int u : topo.face_uses[of.face]) {
            if (topo.uses[u].degenerate) continue;
            efm[topo.uses[u].edge].push_back({u, of});
        }

    std::set<V2OrientedFace> processed;
    for (const auto& seed : pool) {
        if (processed.count(seed)) continue;
        V2Shell sh;
        std::vector<V2OrientedFace> queue{seed};
        std::set<V2OrientedFace> in_shell{seed};
        std::map<int, int> shell_edge_count;
        size_t qi = 0;
        while (qi < queue.size()) {
            const V2OrientedFace f = queue[qi++];
            sh.faces.push_back(f);
            const bool boundary = is_boundary(f);
            for (int u : topo.face_uses[f.face]) {
                if (topo.uses[u].degenerate) continue;
                const int e = topo.uses[u].edge;
                if (shell_edge_count[e] >= 2) continue;
                std::vector<std::pair<int, V2OrientedFace>> off;
                int ways_inside = 0;
                V2OrientedFace sel{-1, false};
                const int dref = topo.use_dir(u, f.reversed);
                for (const auto& kv : efm[e]) {
                    if (kv.second == f || in_shell.count(kv.second)) continue;
                    // GetEdgeOff: the candidate must traverse the shared edge the OTHER way
                    if (topo.use_dir(kv.first, kv.second.reversed) != -dref) continue;
                    if (boundary && !is_boundary(kv.second)) {
                        ++ways_inside;
                        sel = kv.second;
                    }
                    off.push_back(kv);
                }
                if (off.empty()) continue;
                if (!boundary || ways_inside != 1) {
                    if (off.size() == 1) {
                        sel = off[0].second;
                    } else {
                        V2OrientedFace pick{-1, false};
                        if (!v2_get_face_off(topo, u, f, off, pick)) {
                            V2AlertRec a;
                            a.kind = V2Alert::AngularTieUnresolved;
                            a.faces.push_back(f.face);
                            a.detail = "GetFaceOff tie";
                            m_alerts.push_back(a);
                        }
                        sel = pick;
                    }
                }
                if (sel.face < 0 || in_shell.count(sel)) continue;
                in_shell.insert(sel);
                queue.push_back(sel);
                shell_edge_count[e] += 2;
            }
        }
        for (const auto& of : sh.faces) processed.insert(of);
        v2_orient_faces_on_shell(topo, sh);
        sh.closed = v2_shell_is_closed(topo, sh);
        sh.manifold = v2_shell_is_manifold(topo, sh);
        out.push_back(std::move(sh));
    }
}

// §2.7 — growth vs hole, then nesting.
void V2BuilderSolid::perform_areas() {
    const V2Topo& topo = *m_topo;
    for (auto& sh : m_shells) {
        sh.box = V2Box();
        for (const auto& of : sh.faces)
            for (int u : topo.face_uses[of.face]) {
                const BRepEdge& E = topo.b->m_topology_edges[topo.uses[u].edge];
                if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)topo.b->m_curves_3d.size())
                    continue;
                const NurbsCurve& c = topo.b->m_curves_3d[E.curve_3d_index];
                if (!c.is_valid()) continue;
                auto d = c.domain();
                for (int k = 0; k <= 4; ++k)
                    sh.box.add(c.point_at(d.first + (d.second - d.first) * (k / 4.0)));
            }
        Point pp;
        Vector nn;
        for (const auto& of : sh.faces)
            if (v2_face_probe(*topo.b, of.face, pp, nn)) { sh.box.add(pp); break; }
    }

    std::vector<int> growth, holes;
    for (int i = 0; i < (int)m_shells.size(); ++i) {
        if (!m_shells[i].closed && !m_accept_open) {
            V2AlertRec a;   // do NOT port G13 blind: an open shell is not a solid
            a.kind = V2Alert::OpenShellAcceptedAsSolid;
            for (const auto& of : m_shells[i].faces) a.faces.push_back(of.face);
            a.detail = "open shell not turned into a solid";
            m_alerts.push_back(a);
            m_error = true;
            continue;
        }
        bool undec = false;
        m_shells[i].is_hole = v2_shell_is_hole(topo, m_shells[i], &undec);
        if (undec) {
            V2AlertRec a;
            a.kind = V2Alert::ClassifierUndecided;
            a.faces.push_back(m_shells[i].faces.empty() ? -1 : m_shells[i].faces[0].face);
            a.detail = "growth/hole undecided; treated as growth";
            m_alerts.push_back(a);
        }
        (m_shells[i].is_hole ? holes : growth).push_back(i);
    }

    for (int g : growth) {
        V2SolidRec s;
        s.outer_shell = g;
        s.box = m_shells[g].box;
        m_solids.push_back(s);
    }
    if (holes.empty()) return;

    std::map<int, int> owner;   // hole shell -> solid index; innermost parent wins
    for (int si = 0; si < (int)m_solids.size(); ++si) {
        const V2Shell& S = m_shells[m_solids[si].outer_shell];
        for (int h : holes) {
            if (m_shells[h].box.is_out(S.box)) continue;
            if (!v2_shell_inside(topo, m_shells[h], S)) continue;
            auto it = owner.find(h);
            if (it == owner.end()) {
                owner[h] = si;
            } else {
                const V2Shell& Prev = m_shells[m_solids[it->second].outer_shell];
                if (v2_shell_inside(topo, S, Prev)) owner[h] = si;
            }
        }
    }
    for (const auto& kv : owner) m_solids[kv.second].hole_shells.push_back(kv.first);
    for (int h : holes) {
        if (owner.count(h)) continue;
        V2SolidRec s;
        s.outer_shell = h;
        s.box = m_shells[h].box;
        s.box.set_whole();
        m_solids.push_back(s);
        V2AlertRec a;
        a.kind = V2Alert::HoleWithoutParent;
        a.faces.push_back(m_shells[h].faces.empty() ? -1 : m_shells[h].faces[0].face);
        a.detail = "cavity shell inside no growth solid; kept as its own solid";
        m_alerts.push_back(a);
    }
    for (auto& s : m_solids) std::sort(s.hole_shells.begin(), s.hole_shells.end());
}

///////////////////////////////////////////////////////////////////////////////////////////
// 8. Selection
///////////////////////////////////////////////////////////////////////////////////////////

V2OpStates v2_op_states(V2Op op) {
    switch (op) {
        case V2Op::Common: return {true, true};
        case V2Op::Fuse: return {false, false};
        case V2Op::Cut: return {false, true};
        case V2Op::Cut21: return {true, false};
    }
    return {false, false};
}

std::vector<V2OrientedFace> v2_select_faces(const V2SelectionInput& in, V2OpStates st,
                                            std::vector<V2AlertRec>& alerts) {
    const bool objIN = st.objects_in;
    const bool toolIN = st.tools_in;
    const bool avoid_in = (!objIN && !toolIN);            // FUSE
    const bool avoid_in_for_both = (objIN != toolIN);     // CUT / CUT21
    const bool same_ori_needed = (objIN == toolIN);       // FUSE, COMMON

    std::set<int> unoriented_obj, unoriented_tool;
    for (const auto& f : in.object_faces) unoriented_obj.insert(f.face);
    for (const auto& f : in.tool_faces) unoriented_tool.insert(f.face);

    std::vector<V2OrientedFace> res_ori;
    std::set<int> res_fence, fence, to_avoid;
    std::set<V2OrientedFace> fence_ori;
    auto add_ori = [&](V2OrientedFace f) {
        if (std::find(res_ori.begin(), res_ori.end(), f) == res_ori.end()) res_ori.push_back(f);
    };

    static const V2InParts kEmpty;
    for (int grp = 0; grp < 2; ++grp) {
        const auto& faces = grp ? in.tool_faces : in.object_faces;
        const auto& opp_unoriented = grp ? unoriented_obj : unoriented_tool;
        const V2InParts& in_own = grp ? (in.in_tools ? *in.in_tools : kEmpty)
                                      : (in.in_objects ? *in.in_objects : kEmpty);
        const V2InParts& in_opp = grp ? (in.in_objects ? *in.in_objects : kEmpty)
                                      : (in.in_tools ? *in.in_tools : kEmpty);
        const bool take_in = grp ? toolIN : objIN;

        for (const V2OrientedFace& fim : faces) {
            const bool is_in = in_own.contains(fim.face);
            const bool is_in_opp = in_opp.contains(fim.face);

            if (avoid_in && (is_in || is_in_opp)) continue;             // :670-673
            if (avoid_in_for_both && is_in && is_in_opp) continue;      // :676-679

            if (!fence.insert(fim.face).second) {                       // :682
                if (!opp_unoriented.count(fim.face)) {
                    // duplicate WITHIN one group -- fall through, NO continue   :684-692
                    if (take_in != same_ori_needed) to_avoid.insert(fim.face);
                } else {
                    // same-domain wall shared with the other operand            :693-712
                    const bool same_ori = !fence_ori.insert(fim).second;   // :696
                    if (same_ori_needed == same_ori) {
                        if (res_fence.insert(fim.face).second) add_ori(fim);
                    } else {
                        to_avoid.insert(fim.face);
                    }
                    continue;                                           // :711
                }
            }
            if (!fence_ori.insert(fim).second) continue;                // :714

            if (take_in == is_in_opp) {                                 // :719
                if (is_in) {
                    add_ori(fim);
                    add_ori(V2OrientedFace{fim.face, !fim.reversed});
                } else if (take_in && !same_ori_needed) {               // CUT/CUT21 reversal
                    add_ori(V2OrientedFace{fim.face, !fim.reversed});
                } else {
                    add_ori(fim);
                }
                res_fence.insert(fim.face);                             // :734
            }
        }
    }

    std::vector<V2OrientedFace> out;                                    // :740-749
    for (const auto& f : res_ori)
        if (!to_avoid.count(f.face)) out.push_back(f);

    std::map<int, int> cnt;   // I-9 audit
    for (const auto& f : out) ++cnt[f.face];
    for (const auto& kv : cnt)
        if (kv.second > 2) {
            V2AlertRec a;
            a.kind = V2Alert::SelectionDoubleBooking;
            a.faces.push_back(kv.first);
            a.detail = "face image emitted more than twice";
            alerts.push_back(a);
        }
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 9. Verdict metric
///////////////////////////////////////////////////////////////////////////////////////////

std::string V2Verdict::str() const {
    char buf[256];
    std::snprintf(buf, sizeof buf,
                  "F=%d E=%d naked=%d nonmanifold=%d degen=%d seam=%d shells=%d closed=%d", faces,
                  edges, naked, nonmanifold, degenerate, seam, shells, (int)closed);
    return std::string(buf);
}

V2Verdict v2_verdict(const BRep& b) {
    V2Verdict v;
    V2Topo topo;
    topo.build(b);
    v.faces = topo.nb_faces();
    v.edges = topo.nb_edges();
    for (int e = 0; e < v.edges; ++e) {
        const int n = (int)topo.edge_uses[e].size();
        if (n == 0) continue;   // orphan edge slot: not part of any face
        if (topo.edge_degenerate[e]) {
            ++v.degenerate;
            continue;
        }
        if (n == 2 && topo.uses[topo.edge_uses[e][0]].face == topo.uses[topo.edge_uses[e][1]].face)
            ++v.seam;
        if (n < 2) ++v.naked;
        else if (n > 2) ++v.nonmanifold;
    }
    std::vector<V2OrientedFace> all;
    for (int f = 0; f < v.faces; ++f) all.push_back(V2OrientedFace{f, false});
    v.shells = (int)v2_make_connexity_blocks(topo, all).size();
    v.closed = (v.naked == 0 && v.nonmanifold == 0 && v.faces > 0);
    return v;
}

}  // namespace v2sol
}  // namespace session_cpp
