// BDS ARENA implementation — see brep_bds.h for the design, the OCCT anchors and the six
// invariants this file makes unrepresentable.

#include "brep_bds.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstring>
#include <limits>
#include <set>

namespace session_cpp {

namespace {

/// Plain euclidean distance. NOT Point::distance(), which clamps below its float_min and uses a
/// reordered formula; the BoundingVertex rule and every tolerance comparison here must be the
/// same arithmetic OCCT performs.
inline double dist3(const Point& a, const Point& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Lexicographic point order, OCCT's BRepLib_ComparePoints (BRepLib.cxx:96-113).
inline bool point_less(const Point& a, const Point& b) {
    for (int i = 0; i < 3; ++i) {
        if (a[i] < b[i]) return true;
        if (a[i] > b[i]) return false;
    }
    return false;
}

inline void hash_mix(unsigned long long& h, double v) {
    unsigned long long bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    h ^= bits + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
}
inline void hash_mix_int(unsigned long long& h, long long v) {
    h ^= (unsigned long long)v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
}

const char* type_name(BdsType t) {
    switch (t) {
        case BdsType::Vertex: return "V";
        case BdsType::Edge:   return "E";
        case BdsType::Face:   return "F";
        case BdsType::Solid:  return "S";
    }
    return "?";
}

const char* interf_name(BdsInterfType t) {
    switch (t) {
        case BdsInterfType::VV: return "VV";
        case BdsInterfType::VE: return "VE";
        case BdsInterfType::EE: return "EE";
        case BdsInterfType::VF: return "VF";
        case BdsInterfType::EF: return "EF";
        case BdsInterfType::FF: return "FF";
        case BdsInterfType::None: return "--";
    }
    return "--";
}

void append_fmt(std::string& s, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[512];
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    s += buf;
}

const std::vector<BdsPB> g_empty_pbs;
const BdsFaceInfo g_empty_fi;

}  // namespace

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Type codes and boxes
///////////////////////////////////////////////////////////////////////////////////////////

int bds_type_code(BdsType t) {
    switch (t) {
        case BdsType::Solid:  return 2;
        case BdsType::Face:   return 4;
        case BdsType::Edge:   return 6;
        case BdsType::Vertex: return 7;
    }
    return 9;
}

BdsInterfType bds_pair_type(BdsType t1, BdsType t2) {
    const int i1 = bds_type_code(t1), i2 = bds_type_code(t2);
    switch (i2 * 10 + i1) {
        case 77: return BdsInterfType::VV;
        case 76: case 67: return BdsInterfType::VE;
        case 66: return BdsInterfType::EE;
        case 74: case 47: return BdsInterfType::VF;
        case 64: case 46: return BdsInterfType::EF;
        case 44: return BdsInterfType::FF;
        default: return BdsInterfType::None;
    }
}

void BdsBox::add(const Point& p) {
    for (int k = 0; k < 3; ++k) {
        if (p[k] < lo[k]) lo[k] = p[k];
        if (p[k] > hi[k]) hi[k] = p[k];
    }
}

void BdsBox::add(const BdsBox& b) {
    if (b.is_void()) return;
    for (int k = 0; k < 3; ++k) {
        const double blo = b.lo[k] - b.gap, bhi = b.hi[k] + b.gap;
        if (blo < lo[k]) lo[k] = blo;
        if (bhi > hi[k]) hi[k] = bhi;
    }
}

bool BdsBox::is_out(const BdsBox& o) const {
    if (is_void() || o.is_void()) return true;
    for (int k = 0; k < 3; ++k) {
        if (hi[k] + gap < o.lo[k] - o.gap) return true;
        if (o.hi[k] + o.gap < lo[k] - gap) return true;
    }
    return false;
}

bool BdsShape::has_sub(int i) const {
    return std::find(subs.begin(), subs.end(), i) != subs.end();
}

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Pave blocks
///////////////////////////////////////////////////////////////////////////////////////////

bool BdsPaveBlock::has_same_bounds(const BdsPaveBlock& o) const {
    const int n11 = pave1.vertex, n12 = pave2.vertex;
    const int n21 = o.pave1.vertex, n22 = o.pave2.vertex;
    return (n11 == n21 && n12 == n22) || (n11 == n22 && n12 == n21);
}

bool BdsPaveBlock::append_ext_pave(const BdsPave& p) {
    if (std::find(fence.begin(), fence.end(), p.vertex) != fence.end()) return false;
    fence.push_back(p.vertex);
    ext.push_back(p);
    return true;
}

void BdsPaveBlock::append_ext_pave1(const BdsPave& p) {
    if (std::find(fence.begin(), fence.end(), p.vertex) == fence.end()) fence.push_back(p.vertex);
    ext.push_back(p);
}

void BdsPaveBlock::remove_ext_pave(int vertex) {
    ext.erase(std::remove_if(ext.begin(), ext.end(),
                             [vertex](const BdsPave& p) { return p.vertex == vertex; }),
              ext.end());
    fence.erase(std::remove(fence.begin(), fence.end(), vertex), fence.end());
}

bool BdsPaveBlock::contains_parameter(double t, double tol, int& vertex_out) const {
    for (const BdsPave& p : ext) {
        if (std::abs(p.t - t) < tol) { vertex_out = p.vertex; return true; }
    }
    return false;
}

void BdsPaveBlock::update(std::vector<BdsPB>& out, bool use_own) {
    std::vector<BdsPave> all;
    all.reserve(ext.size() + 2);
    if (use_own) { all.push_back(pave1); all.push_back(pave2); }
    for (const BdsPave& p : ext) all.push_back(p);
    ext.clear();
    fence.clear();
    if (all.size() <= 1) return;
    std::stable_sort(all.begin(), all.end(),
                     [](const BdsPave& a, const BdsPave& b) { return a.less(b); });
    for (size_t i = 1; i < all.size(); ++i) {
        BdsPB pb = std::make_shared<BdsPaveBlock>();
        pb->original_edge = original_edge;
        pb->pave1 = all[i - 1];
        pb->pave2 = all[i];
        out.push_back(pb);
    }
}

bool bds_split_interval(const BdsPaveBlock& pb, const BdsPave& at, BdsPB& a, BdsPB& b) {
    if (!(at.t > pb.pave1.t && at.t < pb.pave2.t)) return false;
    a = std::make_shared<BdsPaveBlock>();
    b = std::make_shared<BdsPaveBlock>();
    a->original_edge = pb.original_edge;
    b->original_edge = pb.original_edge;
    a->pave1 = pb.pave1;  a->pave2 = at;
    b->pave1 = at;        b->pave2 = pb.pave2;
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Common block
///////////////////////////////////////////////////////////////////////////////////////////

void BdsCommonBlock::add(const BdsPB& pb) {
    if (members.empty()) { members.push_back(pb); return; }
    if (pb->original_edge < members.front()->original_edge)
        members.insert(members.begin(), pb);
    else
        members.push_back(pb);
}

void BdsCommonBlock::set_members(const std::vector<BdsPB>& v) {
    members.clear();
    for (const BdsPB& pb : v) add(pb);
}

void BdsCommonBlock::set_representative(const BdsPB& pb) {
    auto it = std::find(members.begin(), members.end(), pb);
    if (it == members.end()) return;
    members.erase(it);
    members.insert(members.begin(), pb);
    representative_forced = true;
}

bool BdsCommonBlock::contains(const BdsPB& pb) const {
    return std::find(members.begin(), members.end(), pb) != members.end();
}

bool BdsCommonBlock::contains_face(int f) const {
    return std::find(faces.begin(), faces.end(), f) != faces.end();
}

BdsPB BdsCommonBlock::member_on_edge(int original_edge) const {
    for (const BdsPB& pb : members)
        if (pb->original_edge == original_edge) return pb;
    return BdsPB();
}

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Face info
///////////////////////////////////////////////////////////////////////////////////////////

bool BdsPBSet::add(const BdsPB& pb) {
    if (!pb) return false;
    if (m_ix.find(pb.get()) != m_ix.end()) return false;
    m_ix.emplace(pb.get(), (int)m_v.size());
    m_v.push_back(pb);
    return true;
}

bool BdsPBSet::contains(const BdsPB& pb) const {
    return pb && m_ix.find(pb.get()) != m_ix.end();
}

void BdsPBSet::clear() { m_v.clear(); m_ix.clear(); }

void BdsPBSet::remove(const BdsPB& pb) {
    auto it = m_ix.find(pb.get());
    if (it == m_ix.end()) return;
    m_v.erase(m_v.begin() + it->second);
    m_ix.clear();
    for (size_t i = 0; i < m_v.size(); ++i) m_ix.emplace(m_v[i].get(), (int)i);
}

void BdsFaceInfo::clear() {
    pb_on.clear(); pb_in.clear(); pb_sc.clear();
    v_on.clear(); v_in.clear(); v_sc.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////
// 6. Free geometry rules
///////////////////////////////////////////////////////////////////////////////////////////

void bds_bounding_vertex(const std::vector<Point>& pts, const std::vector<double>& tols,
                         Point& center_out, double& tol_out) {
    const size_t nb = pts.size();
    if (nb < 2 || tols.size() != nb) return;

    if (nb == 2) {
        // Exact smallest enclosing sphere of two balls (BRepLib.cxx:3025-3069).
        const double eps = std::numeric_limits<double>::epsilon();
        int m = 0, n = 1;                       // m = larger tolerance, n = smaller
        if (tols[0] < tols[1]) { m = 1; n = 0; }
        const double dR = tols[m] - tols[n];    // >= 0
        const double vx = pts[n][0] - pts[m][0];
        const double vy = pts[n][1] - pts[m][1];
        const double vz = pts[n][2] - pts[m][2];
        const double d = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (d <= dR || d < eps) {               // big ball already contains the small one
            center_out = pts[m];
            tol_out = tols[m];
            return;
        }
        const double s = dR / d;
        center_out = Point(0.5 * (pts[m][0] + pts[n][0] - vx * s),
                           0.5 * (pts[m][1] + pts[n][1] - vy * s),
                           0.5 * (pts[m][2] + pts[n][2] - vz * s));
        tol_out = 0.5 * (tols[m] + tols[n] + d);
        return;
    }

    // n > 2: sort lexicographically before summing, so the result does not depend on the
    // addition order (OCCT issue 0027540, BRepLib.cxx:3072-3098).
    std::vector<Point> sorted = pts;
    std::sort(sorted.begin(), sorted.end(), point_less);
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (const Point& p : sorted) { sx += p[0]; sy += p[1]; sz += p[2]; }
    const double inv = 1.0 / (double)nb;
    Point c(sx * inv, sy * inv, sz * inv);
    double dmax = -1.0;
    for (size_t i = 0; i < nb; ++i) {
        const double di = dist3(c, pts[i]) + tols[i];
        if (di > dmax) dmax = di;
    }
    center_out = c;
    tol_out = dmax;
}

bool bds_uv_tolerance(const NurbsSurface& s, double u, double v, double tol3d,
                      double& du_out, double& dv_out) {
    // evaluate() returns [S, Sv, Su] for num_derivs == 1 (nurbssurface.cpp:1254 documents the
    // (k,l) loop order).
    std::vector<Vector> d = s.evaluate(u, v, 1);
    if (d.size() < 3) return false;
    const double su = d[2].magnitude();
    const double sv = d[1].magnitude();
    if (!(su > 0.0) || !(sv > 0.0)) return false;   // pole: no finite UV tolerance exists
    du_out = tol3d / su;
    dv_out = tol3d / sv;
    return std::isfinite(du_out) && std::isfinite(dv_out);
}

bool bds_t_tolerance(const NurbsCurve& c, double t, double tol3d, double& dt_out) {
    std::vector<Vector> d = c.evaluate(t, 1);
    if (d.size() < 2) return false;
    const double sp = d[1].magnitude();
    if (!(sp > 0.0)) return false;
    dt_out = tol3d / sp;
    return std::isfinite(dt_out);
}

unsigned long long bds_curve_fingerprint(const NurbsCurve& c) {
    unsigned long long h = 1469598103934665603ULL;
    hash_mix_int(h, c.degree());
    hash_mix_int(h, c.cv_count());
    hash_mix_int(h, c.is_rational() ? 1 : 0);
    const int nk = c.nurbsknot_count();
    hash_mix_int(h, nk);
    for (int i = 0; i < nk; ++i) hash_mix(h, c.nurbsknot(i));
    const int stride = c.cv_size();
    for (int i = 0; i < c.cv_count(); ++i) {
        const double* p = c.cv(i);
        if (!p) continue;
        for (int k = 0; k < stride; ++k) hash_mix(h, p[k]);
    }
    return h;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 7. The arena
///////////////////////////////////////////////////////////////////////////////////////////

void BdsArena::clear() {
    m_operands.clear();
    m_lines.clear();
    m_nb_source = 0;
    m_tol_default = true;
    m_ranges.clear();
    m_ix_vertex.clear(); m_ix_edge.clear(); m_ix_face.clear();
    m_vertex_geom.clear(); m_edge_geom.clear(); m_face_geom.clear();
    m_pb_pool.clear(); m_fi_pool.clear(); m_pb_cb.clear();
    m_fusion_memo.clear(); m_sd.clear(); m_sd_cycle = false;
    m_interf.clear(); m_interf_tb.clear();
}

int BdsArena::append_shape(BdsType t, int operand, int source, double tol) {
    BdsShape s;
    s.index = (int)m_lines.size();
    s.type = t;
    s.operand = operand;
    s.source = source;
    s.tol = tol;
    m_lines.push_back(s);
    return s.index;
}

void BdsArena::init(const std::vector<const BRep*>& operands, double fuzz) {
    clear();
    m_operands = operands;
    // BRep carries no per-entity tolerance (brep.h:27-58), so init() ASSUMES BDS_CONFUSION and
    // records that fact rather than silently substituting a domain-relative value.
    m_tol_default = true;
    const double add_tol = std::max(fuzz, BDS_CONFUSION) * 0.5;

    m_ix_vertex.resize(operands.size());
    m_ix_edge.resize(operands.size());
    m_ix_face.resize(operands.size());

    for (size_t k = 0; k < operands.size(); ++k) {
        const BRep* b = operands[k];
        const int first = (int)m_lines.size();
        if (!b) { m_ranges.push_back({first, first - 1}); continue; }
        const int op = (int)k;

        // Vertices. IDENTITY IS THE BRep'S OWN INDEX — no coordinate is read (I6).
        m_ix_vertex[k].assign(b->m_topology_vertices.size(), -1);
        for (size_t i = 0; i < b->m_topology_vertices.size(); ++i) {
            const int idx = append_shape(BdsType::Vertex, op, (int)i, BDS_CONFUSION);
            BdsVertex vg;
            const int pi = b->m_topology_vertices[i].point_index;
            if (pi >= 0 && pi < (int)b->m_vertices.size()) vg.p = b->m_vertices[pi];
            vg.tol = BDS_CONFUSION;
            vg.origin = -1;
            m_lines[idx].geom = (int)m_vertex_geom.size();
            m_vertex_geom.push_back(vg);
            m_ix_vertex[k][i] = idx;
        }

        // Edges. subs = its vertex indices (one entry when closed: the seam rule).
        m_ix_edge[k].assign(b->m_topology_edges.size(), -1);
        for (size_t i = 0; i < b->m_topology_edges.size(); ++i) {
            const BRepEdge& e = b->m_topology_edges[i];
            const int idx = append_shape(BdsType::Edge, op, (int)i, BDS_CONFUSION);
            BdsEdgeGeom eg;
            if (e.curve_3d_index >= 0 && e.curve_3d_index < (int)b->m_curves_3d.size()) {
                eg.curve = &b->m_curves_3d[e.curve_3d_index];
                const std::pair<double,double> d = eg.curve->domain();
                eg.t0 = d.first;
                eg.t1 = d.second;
            }
            const int v0 = (e.start_vertex >= 0 && e.start_vertex < (int)m_ix_vertex[k].size())
                               ? m_ix_vertex[k][e.start_vertex] : -1;
            const int v1 = (e.end_vertex >= 0 && e.end_vertex < (int)m_ix_vertex[k].size())
                               ? m_ix_vertex[k][e.end_vertex] : -1;
            eg.closed = (v0 >= 0 && v0 == v1);
            m_lines[idx].geom = (int)m_edge_geom.size();
            m_edge_geom.push_back(eg);
            if (v0 >= 0) m_lines[idx].subs.push_back(v0);
            if (v1 >= 0 && v1 != v0) m_lines[idx].subs.push_back(v1);
            m_ix_edge[k][i] = idx;
        }

        // Faces. subs = {edges} u {vertices}; wires/loops are gone, exactly as OCCT's
        // prepareFaces rewrites them (BOPDS_DS.cxx:1710-1773).
        m_ix_face[k].assign(b->m_faces.size(), -1);
        for (size_t i = 0; i < b->m_faces.size(); ++i) {
            const BRepFace& f = b->m_faces[i];
            const int idx = append_shape(BdsType::Face, op, (int)i, BDS_CONFUSION);
            BdsFaceGeom fg;
            if (f.surface_index >= 0 && f.surface_index < (int)b->m_surfaces.size())
                fg.surface = &b->m_surfaces[f.surface_index];
            m_lines[idx].geom = (int)m_face_geom.size();
            m_face_geom.push_back(fg);
            std::vector<int> es, vs;
            for (int li : f.loop_indices) {
                if (li < 0 || li >= (int)b->m_loops.size()) continue;
                for (int ti : b->m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)b->m_trims.size()) continue;
                    const int ei = b->m_trims[ti].edge_index;
                    if (ei < 0 || ei >= (int)m_ix_edge[k].size()) continue;
                    const int ae = m_ix_edge[k][ei];
                    if (ae < 0) continue;
                    if (std::find(es.begin(), es.end(), ae) == es.end()) es.push_back(ae);
                    for (int av : m_lines[ae].subs)
                        if (std::find(vs.begin(), vs.end(), av) == vs.end()) vs.push_back(av);
                }
            }
            std::sort(es.begin(), es.end());
            std::sort(vs.begin(), vs.end());
            m_lines[idx].subs = es;
            m_lines[idx].subs.insert(m_lines[idx].subs.end(), vs.begin(), vs.end());
            m_ix_face[k][i] = idx;
        }

        // One solid per operand; subs = its faces.
        const int sidx = append_shape(BdsType::Solid, op, 0, 0.0);
        for (size_t i = 0; i < b->m_faces.size(); ++i)
            if (m_ix_face[k][i] >= 0) m_lines[sidx].subs.push_back(m_ix_face[k][i]);

        m_ranges.push_back({first, (int)m_lines.size() - 1});
    }
    m_nb_source = (int)m_lines.size();

    // Boxes, in the OCCT order: vertices, then edges (own box + vertex boxes, gap ADDED to the
    // existing gap), then faces (own box + edge boxes, gap added).
    for (BdsShape& s : m_lines) {
        if (s.type != BdsType::Vertex) continue;
        s.box.set_gap(s.tol + add_tol);
        s.box.add(m_vertex_geom[s.geom].p);
    }
    for (BdsShape& s : m_lines) {
        if (s.type != BdsType::Edge) continue;
        const NurbsCurve* c = m_edge_geom[s.geom].curve;
        if (c) for (int i = 0; i < c->cv_count(); ++i) s.box.add(c->get_cv(i));
        s.box.set_gap(s.tol);
        for (int v : s.subs) s.box.add(m_lines[v].box);
        s.box.set_gap(s.box.get_gap() + add_tol);
    }
    for (BdsShape& s : m_lines) {
        if (s.type != BdsType::Face) continue;
        const NurbsSurface* srf = m_face_geom[s.geom].surface;
        if (srf) {
            for (int i = 0; i < srf->cv_count(0); ++i)
                for (int j = 0; j < srf->cv_count(1); ++j) s.box.add(srf->get_cv(i, j));
        }
        s.box.set_gap(s.tol);
        for (int e : s.subs) s.box.add(m_lines[e].box);
        s.box.set_gap(s.box.get_gap() + add_tol);
    }
}

int BdsArena::rank(int i) const {
    for (size_t k = 0; k < m_ranges.size(); ++k)
        if (m_ranges[k].contains(i)) return (int)k;
    return -1;
}

int BdsArena::index_of_vertex(int operand, int local) const {
    if (operand < 0 || operand >= (int)m_ix_vertex.size()) return -1;
    if (local < 0 || local >= (int)m_ix_vertex[operand].size()) return -1;
    return m_ix_vertex[operand][local];
}
int BdsArena::index_of_edge(int operand, int local) const {
    if (operand < 0 || operand >= (int)m_ix_edge.size()) return -1;
    if (local < 0 || local >= (int)m_ix_edge[operand].size()) return -1;
    return m_ix_edge[operand][local];
}
int BdsArena::index_of_face(int operand, int local) const {
    if (operand < 0 || operand >= (int)m_ix_face.size()) return -1;
    if (local < 0 || local >= (int)m_ix_face[operand].size()) return -1;
    return m_ix_face[operand][local];
}

int BdsArena::append_vertex(const Point& p, double tol, int origin) {
    const int idx = append_shape(BdsType::Vertex, -1, -1, tol);
    BdsVertex vg;
    vg.p = p;
    vg.tol = tol;
    vg.origin = origin;
    m_lines[idx].geom = (int)m_vertex_geom.size();
    m_vertex_geom.push_back(vg);
    m_lines[idx].box.set_gap(tol);
    m_lines[idx].box.add(p);
    return idx;
}

int BdsArena::append_edge(const NurbsCurve& c, int v1, int v2, double tol) {
    const int idx = append_shape(BdsType::Edge, -1, -1, tol);
    BdsEdgeGeom eg;
    eg.owned = std::make_shared<NurbsCurve>(c);
    eg.curve = eg.owned.get();
    const std::pair<double,double> d = eg.curve->domain();
    eg.t0 = d.first;
    eg.t1 = d.second;
    eg.closed = (v1 >= 0 && v1 == v2);
    m_lines[idx].geom = (int)m_edge_geom.size();
    m_edge_geom.push_back(eg);
    if (v1 >= 0) m_lines[idx].subs.push_back(v1);
    if (v2 >= 0 && v2 != v1) m_lines[idx].subs.push_back(v2);
    for (int i = 0; i < eg.curve->cv_count(); ++i) m_lines[idx].box.add(eg.curve->get_cv(i));
    m_lines[idx].box.set_gap(tol);
    change_pave_blocks(idx);
    return idx;
}

int BdsArena::append_edge_ref(const NurbsCurve* c, double t0, double t1, int v1, int v2,
                              double tol) {
    const int idx = append_shape(BdsType::Edge, -1, -1, tol);
    BdsEdgeGeom eg;
    eg.curve = c;
    eg.t0 = t0;
    eg.t1 = t1;
    eg.closed = (v1 >= 0 && v1 == v2);
    m_lines[idx].geom = (int)m_edge_geom.size();
    m_edge_geom.push_back(eg);
    if (v1 >= 0) m_lines[idx].subs.push_back(v1);
    if (v2 >= 0 && v2 != v1) m_lines[idx].subs.push_back(v2);
    if (c) for (int i = 0; i < c->cv_count(); ++i) m_lines[idx].box.add(c->get_cv(i));
    m_lines[idx].box.set_gap(tol);
    change_pave_blocks(idx);
    return idx;
}

const BdsVertex& BdsArena::vertex(int i) const {
    static const BdsVertex empty;
    if (!is_vertex(i) || m_lines[i].geom < 0) return empty;
    return m_vertex_geom[m_lines[i].geom];
}

const Point& BdsArena::vertex_point(int i) const { return vertex(i).p; }

const BdsEdgeGeom& BdsArena::edge_geom(int i) const {
    static const BdsEdgeGeom empty;
    if (!is_edge(i) || m_lines[i].geom < 0) return empty;
    return m_edge_geom[m_lines[i].geom];
}

const NurbsCurve* BdsArena::edge_curve(int i) const { return edge_geom(i).curve; }

std::pair<double,double> BdsArena::edge_range(int i) const {
    const BdsEdgeGeom& g = edge_geom(i);
    return {g.t0, g.t1};
}

const NurbsSurface* BdsArena::face_surface(int i) const {
    if (!is_face(i) || m_lines[i].geom < 0) return nullptr;
    return m_face_geom[m_lines[i].geom].surface;
}

void BdsArena::absorb_tolerance(int i, double d) {
    if (!valid(i)) return;
    if (d > m_lines[i].tol) {
        m_lines[i].tol = d;
        m_tol_default = false;
        if (m_lines[i].type == BdsType::Vertex && m_lines[i].geom >= 0)
            m_vertex_geom[m_lines[i].geom].tol = d;
    }
}

//--- vertex fusion -------------------------------------------------------------------------

int BdsArena::fuse_vertices(std::vector<int> cluster) {
    // Order independence starts here: the caller's order never reaches the geometry.
    std::sort(cluster.begin(), cluster.end());
    cluster.erase(std::unique(cluster.begin(), cluster.end()), cluster.end());
    cluster.erase(std::remove_if(cluster.begin(), cluster.end(),
                                 [this](int v) { return !is_vertex(v); }),
                  cluster.end());
    if (cluster.empty()) return -1;
    if (cluster.size() == 1) return cluster[0];

    // Idempotence: the same member set never produces a second vertex.
    auto memo = m_fusion_memo.find(cluster);
    if (memo != m_fusion_memo.end()) return memo->second;

    std::vector<Point> pts;
    std::vector<double> tols;
    pts.reserve(cluster.size());
    tols.reserve(cluster.size());
    for (int v : cluster) {
        pts.push_back(m_vertex_geom[m_lines[v].geom].p);
        tols.push_back(m_lines[v].tol);
    }
    Point c;
    double tol = 0.0;
    bds_bounding_vertex(pts, tols, c, tol);
    const int nv = append_vertex(c, tol, cluster.front());
    for (int v : cluster) add_shape_sd(v, nv);
    m_fusion_memo.emplace(cluster, nv);
    return nv;
}

//--- same domain ----------------------------------------------------------------------------

void BdsArena::add_shape_sd(int i, int i_sd) {
    if (i == i_sd) return;
    m_sd[i] = i_sd;
}

bool BdsArena::has_shape_sd(int i, int& i_sd) const {
    auto it = m_sd.find(i);
    if (it == m_sd.end()) return false;
    i_sd = it->second;
    return true;
}

int BdsArena::resolve_sd(int i) const {
    int j = i;
    for (int hop = 0; hop < BDS_SD_MAX_HOPS; ++hop) {
        auto it = m_sd.find(j);
        if (it == m_sd.end()) return j;
        j = it->second;
    }
    m_sd_cycle = true;   // OCCT loops forever here (BOPDS_DS.cxx:1244-1253)
    return j;
}

//--- paves and pave blocks --------------------------------------------------------------------

void BdsArena::init_pave_blocks(int e) {
    BdsShape& s = m_lines[e];
    if (s.has_reference()) return;
    BdsPaveBlock seed;
    seed.original_edge = e;
    if (s.subs.empty() || s.geom < 0) {
        s.reference = (int)m_pb_pool.size();
        m_pb_pool.push_back({});
        return;
    }
    const BdsEdgeGeom& g = m_edge_geom[s.geom];
    const int v0 = resolve_sd(s.subs.front());
    if (g.closed || s.subs.size() == 1) {
        // Seam rule: ONE distinct vertex, so the second pave is unfenced (BOPDS_DS.cxx:479-483).
        seed.append_ext_pave({v0, g.t0});
        seed.append_ext_pave1({v0, g.t1});
    } else {
        const int v1 = resolve_sd(s.subs[1]);
        seed.append_ext_pave({v0, g.t0});
        seed.append_ext_pave({v1, g.t1});
    }
    std::vector<BdsPB> blocks;
    seed.update(blocks, /*use_own=*/false);
    s.reference = (int)m_pb_pool.size();
    m_pb_pool.push_back(std::move(blocks));
}

const std::vector<BdsPB>& BdsArena::pave_blocks(int e) const {
    if (!has_pave_blocks(e)) return g_empty_pbs;
    return m_pb_pool[m_lines[e].reference];
}

std::vector<BdsPB>& BdsArena::change_pave_blocks(int e) {
    if (!m_lines[e].has_reference()) init_pave_blocks(e);
    return m_pb_pool[m_lines[e].reference];
}

BdsArena::PaveStatus BdsArena::add_pave(int e, double t, int v, double fuzz) {
    if (!is_edge(e)) return PaveStatus::BadEdge;
    if (!is_vertex(v)) return PaveStatus::BadVertex;
    std::vector<BdsPB>& pool = change_pave_blocks(e);
    if (pool.empty()) return PaveStatus::BadEdge;

    // Every existing pave on this edge: block ends plus anything already pending.
    std::vector<BdsPave> existing;
    for (const BdsPB& pb : pool) {
        existing.push_back(pb->pave1);
        existing.push_back(pb->pave2);
        for (const BdsPave& p : pb->ext) existing.push_back(p);
    }
    for (const BdsPave& p : existing)
        if (p.vertex == v) return PaveStatus::DuplicateVertex;

    // MODEL-SPACE coincidence gate (I4/I5): two DISTINCT vertices at the same place are not two
    // paves. The caller must fuse them first; we refuse rather than silently accept.
    const Point& pnew = vertex_point(v);
    const double tnew = m_lines[v].tol;
    const double fz = std::max(fuzz, 0.0);
    for (const BdsPave& p : existing) {
        if (p.vertex < 0) continue;
        const double band = m_lines[p.vertex].tol + tnew + fz;
        if (dist3(vertex_point(p.vertex), pnew) <= band) return PaveStatus::CoincidentVertex;
    }

    for (BdsPB& pb : pool) {
        if (t > pb->pave1.t && t < pb->pave2.t) {
            pb->append_ext_pave({v, t});
            return PaveStatus::Inserted;
        }
    }
    return PaveStatus::OutOfRange;
}

int BdsArena::update_pave_blocks(int e) {
    if (!has_pave_blocks(e)) return 0;
    std::vector<BdsPB>& pool = m_pb_pool[m_lines[e].reference];
    bool any = false;
    for (const BdsPB& pb : pool) if (pb->is_to_update()) { any = true; break; }
    if (!any) return (int)pool.size();   // IDEMPOTENT: nothing pending, nothing created (I1)
    std::vector<BdsPB> out;
    out.reserve(pool.size() + 4);
    for (BdsPB& pb : pool) {
        if (!pb->is_to_update()) { out.push_back(pb); continue; }
        std::vector<BdsPB> sub;
        pb->update(sub, /*use_own=*/true);
        if (sub.empty()) out.push_back(pb);
        else for (BdsPB& s : sub) out.push_back(s);
    }
    pool.swap(out);
    return (int)pool.size();
}

void BdsArena::update_pave_blocks() {
    for (int i = 0; i < (int)m_lines.size(); ++i)
        if (m_lines[i].type == BdsType::Edge && m_lines[i].has_reference()) update_pave_blocks(i);
}

bool BdsArena::paves(int e, std::vector<BdsPave>& out) const {
    out.clear();
    const std::vector<BdsPB>& pool = pave_blocks(e);
    if (pool.empty()) return true;
    for (const BdsPB& pb : pool) {
        for (const BdsPave* p : {&pb->pave1, &pb->pave2}) {
            bool seen = false;
            for (const BdsPave& q : out) if (q.same(*p)) { seen = true; break; }
            if (!seen) out.push_back(*p);
        }
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const BdsPave& a, const BdsPave& b) { return a.less(b); });
    if ((int)out.size() != (int)pool.size() + 1) return false;
    for (size_t i = 1; i < out.size(); ++i)
        if (!(out[i - 1].t < out[i].t)) return false;
    for (const BdsPave& p : out)
        if (!is_vertex(p.vertex)) return false;
    return true;
}

bool BdsArena::pave_blocks_cover_edge(int e) const {
    const std::vector<BdsPB>& pool = pave_blocks(e);
    if (pool.empty()) return false;
    const BdsEdgeGeom& g = edge_geom(e);
    std::vector<const BdsPaveBlock*> s;
    s.reserve(pool.size());
    for (const BdsPB& pb : pool) s.push_back(pb.get());
    std::stable_sort(s.begin(), s.end(),
                     [](const BdsPaveBlock* a, const BdsPaveBlock* b) { return a->pave1.t < b->pave1.t; });
    if (s.front()->pave1.t != g.t0) return false;
    if (s.back()->pave2.t != g.t1) return false;
    for (size_t i = 1; i < s.size(); ++i)
        if (s[i - 1]->pave2.t != s[i]->pave1.t) return false;   // bitwise: no gap, no overlap
    return true;
}

//--- common blocks -----------------------------------------------------------------------------

bool BdsArena::is_common_block(const BdsPB& pb) const {
    return pb && m_pb_cb.find(pb.get()) != m_pb_cb.end();
}

BdsCB BdsArena::common_block(const BdsPB& pb) const {
    if (!pb) return BdsCB();
    auto it = m_pb_cb.find(pb.get());
    return it == m_pb_cb.end() ? BdsCB() : it->second;
}

void BdsArena::set_common_block(const BdsPB& pb, const BdsCB& cb) {
    if (!pb) return;
    m_pb_cb[pb.get()] = cb;
}

BdsPB BdsArena::real_pave_block(const BdsPB& pb) const {
    BdsCB cb = common_block(pb);
    if (cb && !cb->members.empty()) return cb->representative();
    return pb;
}

bool BdsArena::is_common_block_on_edge(const BdsPB& pb) const {
    BdsCB cb = common_block(pb);
    return cb && cb->members.size() > 1;
}

BdsCB BdsArena::make_common_block(const std::vector<BdsPB>& pbs) {
    if (pbs.size() < 2) return BdsCB();   // OCCT skips groups smaller than 2
    BdsCB cb;
    std::vector<int> faces;
    std::set<int> seen;
    for (const BdsPB& pb : pbs) {
        BdsCB old = common_block(pb);
        if (!old) continue;
        for (int f : old->faces) if (seen.insert(f).second) faces.push_back(f);
        if (!cb) cb = old;   // the FIRST member already in a block donates the object
    }
    if (!cb) cb = std::make_shared<BdsCommonBlock>();
    cb->set_members(pbs);
    cb->faces = faces;
    for (const BdsPB& pb : pbs) set_common_block(pb, cb);
    cb->tol = compute_tolerance_of_cb(cb);
    return cb;
}

BdsCB BdsArena::make_common_block_on_faces(const BdsPB& pb, const std::vector<int>& faces) {
    if (!pb) return BdsCB();
    BdsCB cb = common_block(pb);
    if (!cb) {
        cb = std::make_shared<BdsCommonBlock>();
        cb->add(pb);
    }
    for (int f : faces) if (!cb->contains_face(f)) cb->faces.push_back(f);
    set_common_block(pb, cb);
    cb->tol = compute_tolerance_of_cb(cb);
    return cb;
}

double BdsArena::compute_tolerance_of_cb(const BdsCB& cb) const {
    if (!cb || cb->members.empty()) return 0.0;
    const BdsPB& rep = cb->representative();
    const int ne = rep->original_edge;
    double tol_max = valid(ne) ? m_lines[ne].tol : 0.0;
    if (cb->members.size() < 2 && cb->faces.empty()) return tol_max;   // early exit, :266-269
    const NurbsCurve* c3d = edge_curve(ne);
    if (!c3d) return tol_max;

    double t1 = 0.0, t2 = 0.0;
    rep->range(t1, t2);
    const double dt = (t2 - t1) / (double)(BDS_CB_SAMPLES + 1);

    if (cb->members.size() > 1) {
        for (const BdsPB& m : cb->members) {
            if (m == rep) continue;
            const NurbsCurve* c2 = edge_curve(m->original_edge);
            if (!c2) continue;
            const double tolm = valid(m->original_edge) ? m_lines[m->original_edge].tol : 0.0;
            double t = t1;
            for (int i = 1; i <= BDS_CB_SAMPLES; ++i) {
                t += dt;
                const Point p = c3d->point_at(t);
                const double d = dist3(p, c2->closest_point(p));
                if (tolm + d > tol_max) tol_max = tolm + d;
            }
        }
    }
    for (int f : cb->faces) {
        const NurbsSurface* srf = face_surface(f);
        if (!srf) continue;
        const double tolf = valid(f) ? m_lines[f].tol : 0.0;
        double t = t1;
        for (int i = 1; i <= BDS_CB_SAMPLES; ++i) {
            t += dt;
            const Point p = c3d->point_at(t);
            const double d = dist3(p, srf->closest_point(p));
            if (tolf + d > tol_max) tol_max = tolf + d;
        }
    }
    return tol_max;
}

//--- face info -----------------------------------------------------------------------------------

void BdsArena::init_face_info(int f) {
    BdsShape& s = m_lines[f];
    if (s.has_reference()) return;
    s.reference = (int)m_fi_pool.size();
    m_fi_pool.push_back(BdsFaceInfo());
    m_fi_pool.back().index = f;
}

const BdsFaceInfo& BdsArena::face_info(int f) const {
    if (!has_face_info(f)) return g_empty_fi;
    return m_fi_pool[m_lines[f].reference];
}

BdsFaceInfo& BdsArena::change_face_info(int f) {
    if (!m_lines[f].has_reference()) {
        init_face_info(f);
        update_face_info_on(f);
    }
    return m_fi_pool[m_lines[f].reference];
}

void BdsArena::face_info_on(int f, BdsPBSet& pb_out, std::vector<int>& v_out) const {
    pb_out.clear();
    v_out.clear();
    if (!is_face(f)) return;
    for (int s : m_lines[f].subs) {
        if (!valid(s)) continue;
        if (m_lines[s].type == BdsType::Edge) {
            for (const BdsPB& pb : pave_blocks(s)) {
                v_out.push_back(pb->pave1.vertex);
                v_out.push_back(pb->pave2.vertex);
                pb_out.add(real_pave_block(pb));
            }
        } else if (m_lines[s].type == BdsType::Vertex) {
            v_out.push_back(resolve_sd(s));
        }
    }
    std::sort(v_out.begin(), v_out.end());
    v_out.erase(std::unique(v_out.begin(), v_out.end()), v_out.end());
}

void BdsArena::update_face_info_on(int f) {
    if (!is_face(f)) return;
    for (int s : m_lines[f].subs)
        if (valid(s) && m_lines[s].type == BdsType::Edge) change_pave_blocks(s);
    if (!m_lines[f].has_reference()) init_face_info(f);
    BdsFaceInfo& fi = m_fi_pool[m_lines[f].reference];
    face_info_on(f, fi.pb_on, fi.v_on);
}

void BdsArena::refine_face_info_in(int f) {
    if (!has_face_info(f)) return;
    BdsFaceInfo& fi = m_fi_pool[m_lines[f].reference];
    std::vector<BdsPB> drop;
    for (const BdsPB& pb : fi.pb_in) if (fi.pb_on.contains(pb)) drop.push_back(pb);
    for (const BdsPB& pb : drop) fi.pb_in.remove(pb);
}

//--- interferences --------------------------------------------------------------------------------

int BdsArena::add_interf(BdsInterfType t, int a, int b) {
    const std::pair<int,int> key(std::min(a, b), std::max(a, b));
    auto it = m_interf_tb.find(key);
    if (it != m_interf_tb.end()) return it->second;   // idempotent
    BdsInterf f;
    f.type = t;
    f.a = a;
    f.b = b;
    m_interf.push_back(f);
    const int idx = (int)m_interf.size() - 1;
    m_interf_tb.emplace(key, idx);
    return idx;
}

bool BdsArena::has_interf(int i) const {
    for (const auto& kv : m_interf_tb)
        if (kv.first.first == i || kv.first.second == i) return true;
    return false;
}

bool BdsArena::has_interf(int a, int b) const {
    return m_interf_tb.find({std::min(a, b), std::max(a, b)}) != m_interf_tb.end();
}

int BdsArena::nb_interf(BdsInterfType t) const {
    int n = 0;
    for (const BdsInterf& f : m_interf) if (f.type == t) ++n;
    return n;
}

//--- diagnostics ------------------------------------------------------------------------------------

std::string BdsArena::signature() const {
    std::string s;
    append_fmt(s, "shapes=%d source=%d ranges=%d fusions=%d interf=%d\n",
               nb_shapes(), nb_source_shapes(), nb_ranges(), nb_fusions(), (int)m_interf.size());
    for (size_t k = 0; k < m_ranges.size(); ++k)
        append_fmt(s, "R%zu [%d,%d]\n", k, m_ranges[k].first, m_ranges[k].last);

    // Common blocks get stable ordinals from the order their representatives appear in the
    // arena, never from pointer values: the signature must be reproducible run to run.
    std::vector<const BdsCommonBlock*> cbs;
    auto cb_id = [&](const BdsCommonBlock* c) -> int {
        for (size_t i = 0; i < cbs.size(); ++i) if (cbs[i] == c) return (int)i;
        return -1;
    };
    for (int i = 0; i < nb_shapes(); ++i) {
        if (m_lines[i].type != BdsType::Edge || !m_lines[i].has_reference()) continue;
        for (const BdsPB& pb : m_pb_pool[m_lines[i].reference]) {
            BdsCB cb = common_block(pb);
            if (cb && cb_id(cb.get()) < 0) cbs.push_back(cb.get());
        }
    }

    for (int i = 0; i < nb_shapes(); ++i) {
        const BdsShape& sh = m_lines[i];
        append_fmt(s, "%s%d op=%d src=%d tol=%.6g subs=[", type_name(sh.type), i, sh.operand,
                   sh.source, sh.tol);
        for (size_t j = 0; j < sh.subs.size(); ++j)
            append_fmt(s, "%s%d", j ? "," : "", sh.subs[j]);
        s += "]";
        if (sh.type == BdsType::Vertex) {
            const BdsVertex& v = m_vertex_geom[sh.geom];
            append_fmt(s, " p=(%.12g,%.12g,%.12g) vtol=%.6g origin=%d", v.p[0], v.p[1], v.p[2],
                       v.tol, v.origin);
        } else if (sh.type == BdsType::Edge) {
            const BdsEdgeGeom& g = m_edge_geom[sh.geom];
            append_fmt(s, " t=[%.12g,%.12g] closed=%d", g.t0, g.t1, g.closed ? 1 : 0);
        }
        int sd = -1;
        if (has_shape_sd(i, sd)) append_fmt(s, " sd=%d", sd);
        s += "\n";
        if (sh.type == BdsType::Edge && sh.has_reference()) {
            const std::vector<BdsPB>& pool = m_pb_pool[sh.reference];
            for (size_t j = 0; j < pool.size(); ++j) {
                const BdsPB& pb = pool[j];
                append_fmt(s, "  PB%zu [%d@%.12g,%d@%.12g] edge=%d cb=%d\n", j, pb->pave1.vertex,
                           pb->pave1.t, pb->pave2.vertex, pb->pave2.t, pb->edge,
                           cb_id(common_block(pb).get()));
            }
        }
        if (sh.type == BdsType::Face && sh.has_reference()) {
            const BdsFaceInfo& fi = m_fi_pool[sh.reference];
            append_fmt(s, "  FI on=%d in=%d sc=%d von=%d vin=%d vsc=%d\n", fi.pb_on.size(),
                       fi.pb_in.size(), fi.pb_sc.size(), (int)fi.v_on.size(),
                       (int)fi.v_in.size(), (int)fi.v_sc.size());
        }
    }
    for (size_t i = 0; i < cbs.size(); ++i) {
        const BdsCommonBlock* c = cbs[i];
        append_fmt(s, "CB%zu rep_edge=%d members=%d tol=%.6g forced=%d faces=[", i,
                   c->members.empty() ? -1 : c->members.front()->original_edge,
                   (int)c->members.size(), c->tol, c->representative_forced ? 1 : 0);
        for (size_t j = 0; j < c->faces.size(); ++j)
            append_fmt(s, "%s%d", j ? "," : "", c->faces[j]);
        s += "] blocks=[";
        for (size_t j = 0; j < c->members.size(); ++j)
            append_fmt(s, "%s%d:%.12g-%.12g", j ? "," : "", c->members[j]->original_edge,
                       c->members[j]->pave1.t, c->members[j]->pave2.t);
        s += "]\n";
    }
    for (size_t i = 0; i < m_interf.size(); ++i) {
        const BdsInterf& f = m_interf[i];
        append_fmt(s, "I%zu %s a=%d b=%d nv=%d\n", i, interf_name(f.type), f.a, f.b,
                   f.new_vertex);
    }
    return s;
}

bool BdsArena::check_invariants(std::string* why) const {
    auto fail = [&](const std::string& m) { if (why) *why = m; return false; };
    for (int i = 0; i < nb_shapes(); ++i) {
        if (m_lines[i].index != i) return fail("G1: shape(i).index != i");
        if (m_lines[i].tol < 0.0) return fail("I5: negative tolerance");
        for (int sv : m_lines[i].subs)
            if (!valid(sv)) return fail("G1: sub-shape index out of range");
    }
    int covered = 0;
    for (size_t k = 0; k < m_ranges.size(); ++k) {
        if (m_ranges[k].last < m_ranges[k].first - 1) return fail("G3: inverted range");
        covered += m_ranges[k].size();
        if (k && m_ranges[k].first != m_ranges[k - 1].last + 1) return fail("G3: gap in ranges");
    }
    if (!m_ranges.empty() && covered != m_nb_source) return fail("G3: ranges do not cover source");
    for (int i = 0; i < nb_shapes(); ++i) {
        if (m_lines[i].type != BdsType::Edge || !m_lines[i].has_reference()) continue;
        std::vector<BdsPave> p;
        if (!paves(i, p)) return fail("I4: paves do not tile edge " + std::to_string(i));
        if (!m_pb_pool[m_lines[i].reference].empty() && !pave_blocks_cover_edge(i))
            return fail("I1: blocks do not cover edge " + std::to_string(i));
        for (const BdsPB& pb : m_pb_pool[m_lines[i].reference]) {
            if (pb->original_edge != i) return fail("I1: block on wrong edge");
            BdsCB cb = common_block(pb);
            if (cb && !cb->contains(pb)) return fail("I2: common block does not own its member");
        }
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 8. Coincidence predicate — a bool over two NAMED blocks; never an identity lookup (I6)
///////////////////////////////////////////////////////////////////////////////////////////

bool bds_check_coincidence(const BdsArena& ds, const BdsPB& a, const BdsPB& b, double fuzz) {
    if (!a || !b) return false;
    const NurbsCurve* c1 = ds.edge_curve(a->original_edge);
    const NurbsCurve* c2 = ds.edge_curve(b->original_edge);
    if (!c1 || !c2) return false;
    double f1 = 0.0, l1 = 0.0;
    a->range(f1, l1);
    const Point p = c1->point_at(0.5 * (f1 + l1));
    const double tp = c2->closest_parameter(p);
    const double d = dist3(p, c2->point_at(tp));
    const double tol = ds.tolerance(a->pave1.vertex) + ds.tolerance(a->pave2.vertex)
                     + ds.tolerance(b->pave1.vertex) + ds.tolerance(b->pave2.vertex)
                     + std::max(fuzz, BDS_CONFUSION);
    double f2 = 0.0, l2 = 0.0;
    b->range(f2, l2);
    return d < tol && tp > f2 && tp < l2;   // STRICTLY inside b's range
}

} // namespace session_cpp
