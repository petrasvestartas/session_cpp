#include "brep_v2_splitface.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

namespace session_cpp {
namespace v2sf {

namespace sfdetail {

const double SFD_PI = 3.14159265358979323846;
const double SFD_2PI = 6.28318530717958647692;

/// Secant fractions used by the tie escalation, level 1..SF_ESCALATION_LEVELS.
const double SFD_ESC_FRACTION[SF_ESCALATION_LEVELS] = {0.01, 0.05, 0.15, 0.40};

inline double sfd_fold_2pi(double a) {
    a = std::fmod(a, SFD_2PI);
    if (a < 0.0) a += SFD_2PI;
    return a;
}

/// OCCT ClockWiseAngle (BOPAlgo_WireSplitter_1.cxx:621-659). `a_in` is the angle of the
/// ARRIVING direction; A1 is its reversal. Minimising the result selects the outgoing edge with
/// the smallest clockwise sweep from A1 — the LEFTMOST TURN, which traces regions with material
/// on the left, so bounded wires come out positive-area and holes negative.
double sfd_clockwise_angle(double a_in, double a_out) {
    double a1 = sfd_fold_2pi(sfd_fold_2pi(a_in) + SFD_PI);
    double a2 = sfd_fold_2pi(a_out);
    double d = a1 - a2;
    if (d <= 0.0) d += SFD_2PI;
    else if (d <= 1.0e-14) d = SFD_2PI;   // OCCT :654-657, exact-tie promotion
    return d;
}

/// Smallest cyclic separation of two angles, used only by the tie detector.
inline double sfd_cyclic_gap(double a, double b) {
    double d = std::fabs(sfd_fold_2pi(a) - sfd_fold_2pi(b));
    return std::min(d, SFD_2PI - d);
}

}  // namespace sfdetail
using namespace sfdetail;

///////////////////////////////////////////////////////////////////////////////////////////
// Metric
///////////////////////////////////////////////////////////////////////////////////////////

bool sf_first_form(const NurbsSurface& s, double u, double v,
                   double& E, double& F, double& G, double& sqrt_det) {
    E = F = G = sqrt_det = 0.0;
    // evaluate(u,v,1) returns [S, Sv, Su]  (nurbssurface.cpp:1253-1255, (k,l) loop order).
    const std::vector<Vector> d = s.evaluate(u, v, 1);
    if (d.size() < 3) return false;
    const Vector& sv = d[1];
    const Vector& su = d[2];
    E = su.dot(su);
    F = su.dot(sv);
    G = sv.dot(sv);
    const double det = E * G - F * F;
    if (!(det > 0.0)) return false;
    sqrt_det = std::sqrt(det);
    // A pole is not "det happens to be tiny": it is det vanishing RELATIVE to the frame's own
    // scale. Comparing against E*G keeps the test dimensionless and scale-free.
    if (!(det > 1e-24 * std::max(1e-300, E * G))) return false;
    return true;
}

double sf_raw_angle(double du, double dv) {
    if (du == 0.0 && dv == 0.0) return 0.0;
    return sfd_fold_2pi(std::atan2(dv, du));
}

double sf_metric_angle(const NurbsSurface& s, double u, double v,
                       double du, double dv, bool* degenerate) {
    if (degenerate) *degenerate = false;
    double E, F, G, sq;
    if (!sf_first_form(s, u, v, E, F, G, sq)) {
        if (degenerate) *degenerate = true;
        return sf_raw_angle(du, dv);
    }
    // angle of dS = du*S_u + dv*S_v in the orthonormal frame (e1, e2), e1 = S_u/|S_u|,
    // e2 = normalise(S_v - (F/E) S_u):
    //     dS.e1 = (du*E + dv*F)/sqrt(E)      dS.e2 = dv*sqrt(EG-F^2)/sqrt(E)
    // The common positive factor 1/sqrt(E) is irrelevant to atan2, so it is dropped.
    const double x = du * E + dv * F;
    const double y = dv * sq;
    if (x == 0.0 && y == 0.0) {
        if (degenerate) *degenerate = true;
        return sf_raw_angle(du, dv);
    }
    return sfd_fold_2pi(std::atan2(y, x));
}

bool sf_uv_tolerance(const NurbsSurface& s, double u, double v, double tol3d,
                     double& du, double& dv) {
    du = dv = 0.0;
    const std::vector<Vector> d = s.evaluate(u, v, 1);
    if (d.size() < 3) return false;
    const double lv = d[1].magnitude();
    const double lu = d[2].magnitude();
    if (!(lu > 0.0) || !(lv > 0.0)) return false;
    du = tol3d / lu;
    dv = tol3d / lv;
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Identity
///////////////////////////////////////////////////////////////////////////////////////////

SfPaveBlockId sf_pave_block_id(const BdsArena& arena, const BdsPaveBlock* pb) {
    SfPaveBlockId id;
    if (!pb) return id;
    const int oe = pb->original_edge;
    if (!arena.is_edge(oe) || !arena.has_pave_blocks(oe)) return id;
    const std::vector<BdsPB>& pool = arena.pave_blocks(oe);
    for (int k = 0; k < (int)pool.size(); ++k) {
        if (pool[k].get() == pb) { id.edge = oe; id.block = k; return id; }
    }
    return id;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Report
///////////////////////////////////////////////////////////////////////////////////////////

const char* sf_alert_name(SfAlert a) {
    switch (a) {
        case SfAlert::NoPCurve: return "NoPCurve";
        case SfAlert::MicroEdge: return "MicroEdge";
        case SfAlert::DanglingPruned: return "DanglingPruned";
        case SfAlert::DoubledHangingPruned: return "DoubledHangingPruned";
        case SfAlert::EdgeNotConsumed: return "EdgeNotConsumed";
        case SfAlert::WireDegenerateArea: return "WireDegenerateArea";
        case SfAlert::HoleUnassigned: return "HoleUnassigned";
        case SfAlert::NoLoops: return "NoLoops";
        case SfAlert::AngleTieUnresolved: return "AngleTieUnresolved";
        case SfAlert::WalkDeadEnd: return "WalkDeadEnd";
        case SfAlert::WalkStepCap: return "WalkStepCap";
        case SfAlert::MetricDegenerate: return "MetricDegenerate";
        case SfAlert::DegenerateOnlyWire: return "DegenerateOnlyWire";
    }
    return "?";
}

int SfReport::count(SfAlert a) const {
    int n = 0;
    for (size_t i = 0; i < events.size(); ++i)
        if (events[i].first == a) ++n;
    return n;
}

std::string SfReport::str() const {
    std::map<int, int> tally;
    for (size_t i = 0; i < events.size(); ++i) ++tally[(int)events[i].first];
    std::string s;
    char buf[128];
    std::snprintf(buf, sizeof buf, "nodes=%d maxval=%d steps=%d esc=%d seammerge=%d", nodes, max_valence,
                  walk_steps, angle_escalations, seam_merges);
    s += buf;
    for (std::map<int, int>::const_iterator it = tally.begin(); it != tally.end(); ++it) {
        std::snprintf(buf, sizeof buf, " %s=%d", sf_alert_name((SfAlert)it->first), it->second);
        s += buf;
    }
    return s;
}

///////////////////////////////////////////////////////////////////////////////////////////
// The splitter
///////////////////////////////////////////////////////////////////////////////////////////

namespace sfdetail {

struct SfdRec {
    int input = -1;
    bool is_in = false;      ///< this record is the ARRIVING end of the oriented occurrence
    bool is_inside = false;  ///< the pave block was supplied with BOTH senses (section/internal)
    bool passed = false;
    double t = 0.0;          ///< pcurve parameter AT the vertex
    double t_far = 0.0;      ///< pcurve parameter at the far end (sets the step's sign)
    double u = 0.0, v = 0.0; ///< UV image of the vertex on THIS occurrence's pcurve
    double angle = 0.0;
    int level = 0;           ///< secant escalation level actually used
};

struct SfdNode {
    int vertex = -1;         ///< ARENA vertex index. Identity. Never a coordinate.
    std::vector<SfdRec> recs;   ///< insertion order == input list order (determinism, G10)
    bool multi_uv = false;   ///< one 3D vertex, several UV images (seam end / pole)
    double tol_u = 0.0, tol_v = 0.0;
};

struct SfdPoly {
    std::vector<double> u, v;
};

class SfdSplitter {
public:
    SfdSplitter(const BdsArena& arena, const NurbsSurface& srf,
             const std::vector<SfInputEdge>& in, const SfOptions& opt)
        : m_arena(arena), m_srf(srf), m_in(in), m_opt(opt) {}

    SfResult run();

private:
    void classify_multiplicity();
    void build_nodes();
    void prune_to_fixpoint();
    void compute_angles();
    void walk();
    void path(int node, int rec);
    void compose_across_seams();
    void areas(SfResult& out);
    void internals(SfResult& out);

    double angle_of(const SfdRec& r, int level, bool* degen) const;
    SfdPoly wire_poly(const SfWire& w) const;
    static double poly_area(const SfdPoly& p);
    static bool point_in_poly(const SfdPoly& p, double u, double v);
    bool wire_sample(const SfWire& w, double& u, double& v) const;

    const BdsArena& m_arena;
    const NurbsSurface& m_srf;
    const std::vector<SfInputEdge>& m_in;
    const SfOptions& m_opt;

    std::vector<SfdNode> m_nodes;
    std::map<int, int> m_vnode;              ///< arena vertex -> node index
    std::vector<int> m_mult;                 ///< per input: multiplicity of its pave block
    std::vector<char> m_inside;              ///< per input: OCCT IsInside (section/IN/INTERNAL)
    std::vector<char> m_seam;                ///< per input: its pave block carries TWO pcurves here
    std::vector<char> m_avoid;               ///< per input
    std::vector<char> m_used;                ///< per input: consumed by a wire
    std::vector<int> m_out_node, m_out_rec;  ///< per input
    std::vector<int> m_in_node, m_in_rec;    ///< per input
    std::vector<SfWire> m_wires;
    SfReport m_rep;
    int m_cap = 0;
};

///////////////////////////////////////////////////////////////////////////////////////////

void SfdSplitter::classify_multiplicity() {
    const int n = (int)m_in.size();
    m_mult.assign(n, 0);
    m_avoid.assign(n, 0);
    m_used.assign(n, 0);
    std::map<const BdsPaveBlock*, int> cnt;
    std::map<const BdsPaveBlock*, std::set<const NurbsCurve*> > pcs;
    for (int i = 0; i < n; ++i) {
        if (!m_in[i].pb) { m_avoid[i] = 1; m_rep.add(SfAlert::NoPCurve, i); continue; }
        if (!m_in[i].pcurve || !m_in[i].pcurve->is_valid()) {
            m_avoid[i] = 1;
            m_rep.add(SfAlert::NoPCurve, i);
            continue;
        }
        const std::pair<double, double> d = m_in[i].pcurve->domain();
        if (!(std::fabs(d.second - d.first) > SF_PCONFUSION)) {
            m_avoid[i] = 1;
            m_rep.add(SfAlert::MicroEdge, i);
            continue;
        }
        ++cnt[m_in[i].pb.get()];
        pcs[m_in[i].pb.get()].insert(m_in[i].pcurve);
    }
    // OCCT's IsInside flag, reproduced exactly (BOPAlgo_WireSplitter_1.cxx:148-151, :309):
    //   bIsClosed = Degenerated(e) || IsClosed(e, face)     [ IsClosed == carries TWO pcurves ]
    //   the edge is removed from the fence map only when supplied twice AND !bIsClosed,
    //   so IsInside == "supplied with both senses AND not a seam AND not degenerate".
    // A SEAM and a POLE are BOUNDARY material; only a section / IN / INTERNAL edge is interior.
    m_inside.assign(n, 0);
    m_seam.assign(n, 0);
    for (int i = 0; i < n; ++i) {
        if (m_avoid[i]) continue;
        m_mult[i] = cnt[m_in[i].pb.get()];
        const bool seam = pcs[m_in[i].pb.get()].size() >= 2;
        m_seam[i] = seam ? 1 : 0;
        m_inside[i] = (m_mult[i] >= 2 && !seam && !m_in[i].degenerate) ? 1 : 0;
    }
}

void SfdSplitter::build_nodes() {
    const int n = (int)m_in.size();
    m_out_node.assign(n, -1);
    m_out_rec.assign(n, -1);
    m_in_node.assign(n, -1);
    m_in_rec.assign(n, -1);

    // Two passes so the node list order is the FIRST-APPEARANCE order of the OUT ends, which
    // makes the launch order a pure function of the input list (G10 determinism).
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 0; i < n; ++i) {
            if (m_avoid[i]) continue;
            const SfInputEdge& e = m_in[i];
            const std::pair<double, double> d = e.pcurve->domain();
            const bool fwd = (e.sense == SfSense::Forward);
            // IDENTITY IS THE RESOLVED ARENA VERTEX. A pave block minted before a
            // BdsArena::fuse_vertices call still names the pre-fusion vertex (fusion redirects
            // through the same-domain map and never rewrites provenance), so two blocks that
            // meet at ONE geometric node can carry two different raw indices. Reading them raw
            // makes the node graph disconnected and the wire walk dead-ends; resolve_sd is a
            // table read on a NAMED index, not a coordinate search.
            const int v_out = m_arena.resolve_sd(fwd ? e.pb->pave1.vertex : e.pb->pave2.vertex);
            const int v_in = m_arena.resolve_sd(fwd ? e.pb->pave2.vertex : e.pb->pave1.vertex);
            const double t_out = fwd ? d.first : d.second;
            const double t_in = fwd ? d.second : d.first;
            const int av = (pass == 0) ? v_out : v_in;
            const double tt = (pass == 0) ? t_out : t_in;
            const double tf = (pass == 0) ? t_in : t_out;

            std::map<int, int>::iterator it = m_vnode.find(av);
            int ni;
            if (it == m_vnode.end()) {
                ni = (int)m_nodes.size();
                SfdNode nd;
                nd.vertex = av;
                m_nodes.push_back(nd);
                m_vnode[av] = ni;
            } else {
                ni = it->second;
            }
            SfdRec r;
            r.input = i;
            r.is_in = (pass == 1);
            r.is_inside = (m_inside[i] != 0);
            r.t = tt;
            r.t_far = tf;
            const Point p = e.pcurve->point_at(tt);
            r.u = p[0];
            r.v = p[1];
            const int ri = (int)m_nodes[ni].recs.size();
            m_nodes[ni].recs.push_back(r);
            if (pass == 0) { m_out_node[i] = ni; m_out_rec[i] = ri; }
            else           { m_in_node[i] = ni;  m_in_rec[i] = ri; }
        }
    }

    // Per-node UV tolerance (MODEL-SPACE tolerance pushed through the surface metric at the
    // point of use) and the multi-image flag.
    for (size_t k = 0; k < m_nodes.size(); ++k) {
        SfdNode& nd = m_nodes[k];
        if (nd.recs.empty()) continue;
        double tol3d = m_opt.tol3d;
        if (m_arena.is_vertex(nd.vertex)) tol3d = std::max(tol3d, m_arena.tolerance(nd.vertex));
        double du = 0.0, dv = 0.0;
        if (!sf_uv_tolerance(m_srf, nd.recs[0].u, nd.recs[0].v, tol3d, du, dv)) {
            // A pole: no finite UV tolerance in the collapsed direction. Named, not guessed.
            m_rep.add(SfAlert::MetricDegenerate, nd.recs[0].input);
            const std::pair<double, double> d0 = m_srf.domain(0);
            const std::pair<double, double> d1 = m_srf.domain(1);
            du = std::fabs(d0.second - d0.first) * 1e-12;
            dv = std::fabs(d1.second - d1.first) * 1e-12;
        }
        nd.tol_u = du;
        nd.tol_v = dv;
        for (size_t a = 1; a < nd.recs.size() && !nd.multi_uv; ++a) {
            if (std::fabs(nd.recs[a].u - nd.recs[0].u) > 2.0 * du ||
                std::fabs(nd.recs[a].v - nd.recs[0].v) > 2.0 * dv)
                nd.multi_uv = true;
        }
        m_rep.max_valence = std::max(m_rep.max_valence, (int)nd.recs.size());
    }
    m_rep.nodes = (int)m_nodes.size();
}

/// OCCT PerformShapesToAvoid (BOPAlgo_BuilderFace.cxx:152-235), on the ORIENTED-MULTIPLICITY
/// graph, with the degenerate exemption (:200-203) and the closed-edge exemption (:223-226).
void SfdSplitter::prune_to_fixpoint() {
    const int n = (int)m_in.size();
    for (int iter = 0; iter < n + 2; ++iter) {
        std::vector<std::vector<int> > inc(m_nodes.size());
        for (int i = 0; i < n; ++i) {
            if (m_avoid[i]) continue;
            inc[m_out_node[i]].push_back(i);
            inc[m_in_node[i]].push_back(i);
        }
        bool found = false;
        for (size_t k = 0; k < inc.size(); ++k) {
            const std::vector<int>& le = inc[k];
            if (le.size() == 1) {
                const int i = le[0];
                if (m_in[i].degenerate) continue;   // a pole row is never a dangler
                m_avoid[i] = 1;
                m_rep.add(SfAlert::DanglingPruned, i);
                found = true;
            } else if (le.size() == 2) {
                const int i = le[0], j = le[1];
                if (i == j) continue;
                if (m_in[i].pb.get() != m_in[j].pb.get()) continue;
                // both records are the SAME pave block: a section listed F+R whose far end is
                // free. A CLOSED block legitimately visits one vertex twice.
                if (m_arena.resolve_sd(m_in[i].pb->pave1.vertex) ==
                    m_arena.resolve_sd(m_in[i].pb->pave2.vertex))
                    continue;
                m_avoid[i] = 1;
                m_avoid[j] = 1;
                m_rep.add(SfAlert::DoubledHangingPruned, i);
                m_rep.add(SfAlert::DoubledHangingPruned, j);
                found = true;
            }
        }
        if (!found) break;
    }
}

/// Angle of one record at escalation `level`. Level 0 uses the smallest step that escapes the
/// vertex's own tolerance ball (sampling inside it would measure noise); levels 1..4 use fixed
/// fractions of the pcurve span, which is what separates two curves that leave the vertex
/// tangentially.
double SfdSplitter::angle_of(const SfdRec& r, int level, bool* degen) const {
    const SfInputEdge& e = m_in[r.input];
    const NurbsCurve& pc = *e.pcurve;
    const std::pair<double, double> d = pc.domain();
    const double span = std::fabs(d.second - d.first);
    double step;
    if (level <= 0) {
        // dt whose UV image is about the vertex tolerance: dt = tol_uv / |dC2d/dt|.
        const double h = std::max(span * 1e-6, SF_PCONFUSION);
        const double sgn = (r.t_far > r.t) ? 1.0 : -1.0;
        const Point p0 = pc.point_at(r.t);
        const Point p1 = pc.point_at(r.t + sgn * h);
        const double du = p1[0] - p0[0], dv = p1[1] - p0[1];
        const double sp = std::sqrt(du * du + dv * dv) / h;
        // The tolerance ball radius in UV, from the surface metric at this point.
        double tu = 0.0, tv = 0.0;
        if (!sf_uv_tolerance(m_srf, r.u, r.v, m_opt.tol3d, tu, tv)) { tu = tv = 0.0; }
        const double ball = std::sqrt(tu * tu + tv * tv);
        step = (sp > 0.0) ? (2.0 * ball / sp) : SF_PCONFUSION;
        step = std::max(step, std::max(SF_PCONFUSION, span * 1e-7));
        double cap = 0.05 * span;                       // OCCT WireSplitter_1.cxx:810-820
        if (cap < 5.0e-5) cap = std::min(5.0e-5, 0.5 * span);
        step = std::min(step, cap);
    } else {
        step = SFD_ESC_FRACTION[std::min(level, SF_ESCALATION_LEVELS) - 1] * span;
    }
    const double sgn = (r.t_far > r.t) ? 1.0 : -1.0;
    double ts = r.t + sgn * step;
    if (sgn > 0.0) ts = std::min(ts, d.second);
    else           ts = std::max(ts, d.first);
    const Point pv = pc.point_at(r.t);
    const Point ps = pc.point_at(ts);
    // OUT record: direction AWAY from the vertex. IN record: direction TOWARD it.
    double du = ps[0] - pv[0], dv = ps[1] - pv[1];
    if (r.is_in) { du = -du; dv = -dv; }
    return sf_metric_angle(m_srf, r.u, r.v, du, dv, degen);
}

void SfdSplitter::compute_angles() {
    for (size_t k = 0; k < m_nodes.size(); ++k) {
        SfdNode& nd = m_nodes[k];
        for (size_t a = 0; a < nd.recs.size(); ++a) {
            bool dg = false;
            nd.recs[a].angle = angle_of(nd.recs[a], 0, &dg);
            nd.recs[a].level = 0;
            if (dg) m_rep.add(SfAlert::MetricDegenerate, nd.recs[a].input);
        }
        // SECANT ESCALATION: only the tied records are re-measured, with a progressively longer
        // secant, until the tie breaks. Replaces OCCT RefineAngles/RefineAngle2D.
        //
        // WHAT COUNTS AS A TIE. Only two OUTGOING records at the SAME UV image of the vertex
        // ever compete in the leftmost-turn comparison, so only those can be ambiguous. An
        // arriving record sharing a direction with a departing one is not an ambiguity (a seam
        // below and a seam above a node are collinear by construction); nor are two records at
        // two different UV images of one seam vertex, which are never compared at all.
        for (int level = 1; level <= SF_ESCALATION_LEVELS; ++level) {
            std::vector<char> tied(nd.recs.size(), 0);
            bool any = false;
            for (size_t a = 0; a < nd.recs.size(); ++a)
                for (size_t b = a + 1; b < nd.recs.size(); ++b) {
                    if (nd.recs[a].is_in || nd.recs[b].is_in) continue;
                    if (m_in[nd.recs[a].input].pb.get() == m_in[nd.recs[b].input].pb.get())
                        continue;
                    if (nd.multi_uv &&
                        (std::fabs(nd.recs[a].u - nd.recs[b].u) > 2.0 * nd.tol_u ||
                         std::fabs(nd.recs[a].v - nd.recs[b].v) > 2.0 * nd.tol_v))
                        continue;
                    if (sfd_cyclic_gap(nd.recs[a].angle, nd.recs[b].angle) < SF_ANGLE_TIE) {
                        tied[a] = tied[b] = 1;
                        any = true;
                    }
                }
            if (!any) break;
            for (size_t a = 0; a < nd.recs.size(); ++a) {
                if (!tied[a]) continue;
                nd.recs[a].angle = angle_of(nd.recs[a], level, 0);
                nd.recs[a].level = level;
                ++m_rep.angle_escalations;
            }
            if (level == SF_ESCALATION_LEVELS) {
                for (size_t a = 0; a < nd.recs.size(); ++a)
                    for (size_t b = a + 1; b < nd.recs.size(); ++b)
                        if (tied[a] && tied[b] &&
                            sfd_cyclic_gap(nd.recs[a].angle, nd.recs[b].angle) < SF_ANGLE_TIE)
                            m_rep.add(SfAlert::AngleTieUnresolved, nd.recs[a].input);
            }
        }
    }
}

/// OCCT Path (BOPAlgo_WireSplitter_1.cxx:358-617): a stack machine with a closure scan and
/// truncate-and-continue, so one launch can emit several wires.
void SfdSplitter::path(int start_node, int start_rec) {
    struct Frame { int node; int rec; int input; double u, v; };
    std::vector<Frame> st;
    int cn = start_node, cr = start_rec;

    for (;;) {
        if (++m_rep.walk_steps > m_cap) { m_rep.add(SfAlert::WalkStepCap, -1); return; }
        const int inp = m_nodes[cn].recs[cr].input;

        // A. never escape through the edge the walk entered by
        if (st.size() == 1 && m_in[st[0].input].pb.get() == m_in[inp].pb.get()) return;

        // B. push
        m_nodes[cn].recs[cr].passed = true;
        Frame f;
        f.node = cn;
        f.rec = cr;
        f.input = inp;
        f.u = m_nodes[cn].recs[cr].u;
        f.v = m_nodes[cn].recs[cr].v;
        st.push_back(f);

        const int vb = m_in_node[inp];
        double pbu = m_nodes[vb].recs[m_in_rec[inp]].u;
        double pbv = m_nodes[vb].recs[m_in_rec[inp]].v;

        // C. closure scan, from the top of the stack downward
        int found = -1;
        bool has_real = false;
        for (int i = (int)st.size() - 1; i >= 0; --i) {
            if (!has_real) {
                if (m_in[st[i].input].degenerate) continue;   // a wire of poles only is not a wire
                has_real = true;
            }
            if (st[i].node != vb) continue;
            if (m_nodes[vb].multi_uv) {
                if (std::fabs(st[i].u - pbu) > 2.0 * m_nodes[vb].tol_u ||
                    std::fabs(st[i].v - pbv) > 2.0 * m_nodes[vb].tol_v)
                    continue;
            }
            found = i;
            break;
        }

        int arrive = inp;
        if (found >= 0) {
            const int len = (int)st.size() - found;
            const bool slit = (len == 2 &&
                               m_in[st[found].input].pb.get() == m_in[st.back().input].pb.get());
            if (!slit) {
                SfWire w;
                for (int i = found; i < (int)st.size(); ++i) {
                    w.inputs.push_back(st[i].input);
                    m_used[st[i].input] = 1;
                }
                bool only_deg = true;
                for (size_t q = 0; q < w.inputs.size(); ++q)
                    if (!m_in[w.inputs[q]].degenerate) { only_deg = false; break; }
                if (only_deg) m_rep.add(SfAlert::DegenerateOnlyWire, w.inputs[0]);
                else m_wires.push_back(w);
            }
            st.resize((size_t)found);
            if (st.empty()) return;
            arrive = st.back().input;
            pbu = m_nodes[vb].recs[m_in_rec[arrive]].u;
            pbv = m_nodes[vb].recs[m_in_rec[arrive]].v;
        }

        // D. choose the next outgoing edge at vb
        SfdNode& nb = m_nodes[vb];
        const SfdRec& arr = nb.recs[m_in_rec[arrive]];
        const double angle_in = arr.angle;
        const bool arriving_boundary = !arr.is_inside;

        int cnt = 0;
        for (size_t a = 0; a < nb.recs.size(); ++a)
            if (!nb.recs[a].is_in && !nb.recs[a].passed) ++cnt;
        if (cnt == 0) { m_rep.add(SfAlert::WalkDeadEnd, arrive); return; }

        int best = -1, only_inside = -1, n_inside = 0;
        double best_angle = 100.0;   // OCCT aMinAngle sentinel (WireSplitter_1.cxx:530)
        for (size_t a = 0; a < nb.recs.size(); ++a) {
            const SfdRec& c = nb.recs[a];
            if (c.is_in || c.passed) continue;
            if (cnt == 1) { best = (int)a; break; }
            double ang;
            if (m_in[c.input].pb.get() == m_in[arrive].pb.get()) {
                ang = SFD_2PI;   // going straight back out the arriving edge is the last resort
            } else {
                if (nb.multi_uv) {
                    // Seam / pole disambiguation INSIDE one named arena vertex: only candidates
                    // whose UV image is the one we arrived at are reachable.
                    if (std::fabs(c.u - pbu) > 2.0 * nb.tol_u ||
                        std::fabs(c.v - pbv) > 2.0 * nb.tol_v)
                        continue;
                }
                ang = sfd_clockwise_angle(angle_in, c.angle);
            }
            if (arriving_boundary && c.is_inside) { ++n_inside; only_inside = (int)a; }
            if (ang < best_angle - SF_ANGLE_EPS) { best_angle = ang; best = (int)a; }
        }
        // Arriving on a BOUNDARY edge with exactly one unpassed interior way out forces the walk
        // onto the section, whatever the angle says (OCCT :602-605). Without it a section that
        // meets the boundary tangentially is skipped and the face does not split.
        if (n_inside == 1) best = only_inside;
        if (best < 0) { m_rep.add(SfAlert::WalkDeadEnd, arrive); return; }

        cn = vb;
        cr = best;
    }
}

void SfdSplitter::walk() {
    for (size_t k = 0; k < m_nodes.size(); ++k)
        for (size_t a = 0; a < m_nodes[k].recs.size(); ++a) {
            const SfdRec& r = m_nodes[k].recs[a];
            if (r.is_in || r.passed || m_avoid[r.input]) continue;
            path((int)k, (int)a);
        }
}

///////////////////////////////////////////////////////////////////////////////////////////

/// Sample parameters of one pcurve, in TRAVERSAL order. The samples include every distinct
/// knot of the curve, so a degree-1 pcurve (what a real section imprint produces) is reproduced
/// EXACTLY rather than being re-approximated by a coarser polygon -- otherwise the signed area
/// carries a discretisation error that a thin wire can be swamped by.
static void sfd_sample_params(const NurbsCurve& c, bool fwd, int per_span,
                              std::vector<double>& out) {
    out.clear();
    const std::pair<double, double> d = c.domain();
    std::vector<double> kn = c.get_nurbsknots();
    std::vector<double> pts;
    pts.push_back(d.first);
    for (size_t i = 0; i < kn.size(); ++i)
        if (kn[i] > d.first && kn[i] < d.second && kn[i] > pts.back()) pts.push_back(kn[i]);
    pts.push_back(d.second);
    const int k = std::max(1, per_span);
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        for (int j = 0; j < k; ++j)
            out.push_back(pts[i] + (pts[i + 1] - pts[i]) * ((double)j / k));
    out.push_back(d.second);
    if (!fwd) std::reverse(out.begin(), out.end());
}

/// SEAM COMPOSITION. See the header: a seam pave block whose two occurrences land in two
/// DIFFERENT wires is interior to their union, so those wires are ONE face. Splice them, drop
/// both occurrences and translate the spliced-in half by the whole-period offset measured
/// BETWEEN THE TWO PCURVES OF THAT SHARED BLOCK. Two occurrences inside ONE wire are a genuine
/// seam of that face and are left untouched.
void SfdSplitter::compose_across_seams() {
    const int nw = (int)m_wires.size();
    if (nw < 2) return;
    std::map<const BdsPaveBlock*, std::vector<std::pair<int, int> > > occ;
    for (int w = 0; w < nw; ++w)
        for (int k = 0; k < (int)m_wires[w].inputs.size(); ++k) {
            const int i = m_wires[w].inputs[k];
            if (!m_seam[i]) continue;
            occ[m_in[i].pb.get()].push_back(std::make_pair(w, k));
        }
    bool any_link = false;
    for (std::map<const BdsPaveBlock*, std::vector<std::pair<int, int> > >::const_iterator it =
             occ.begin(); it != occ.end(); ++it)
        if (it->second.size() == 2 && it->second[0].first != it->second[1].first) any_link = true;
    if (!any_link) return;

    std::vector<char> consumed(nw, 0);
    std::vector<SfWire> out;
    for (int root = 0; root < nw; ++root) {
        if (consumed[root]) continue;
        consumed[root] = 1;
        SfWire ch = m_wires[root];
        ch.shift_u.assign(ch.inputs.size(), 0.0);
        ch.shift_v.assign(ch.inputs.size(), 0.0);
        for (bool grew = true; grew;) {
            grew = false;
            for (int p = 0; p < (int)ch.inputs.size(); ++p) {
                const int i = ch.inputs[p];
                if (!m_seam[i]) continue;
                const std::vector<std::pair<int, int> >& o = occ[m_in[i].pb.get()];
                if (o.size() != 2) continue;
                int w2 = -1, j = -1;
                for (int q = 0; q < 2; ++q)
                    if (!consumed[o[q].first]) { w2 = o[q].first; j = o[q].second; }
                if (w2 < 0) continue;                          // both sides already in this face
                const int i2 = m_wires[w2].inputs[j];
                if (i2 == i) continue;                         // the very same occurrence
                const NurbsCurve* c1 = m_in[i].pcurve;
                const NurbsCurve* c2 = m_in[i2].pcurve;
                const std::pair<double, double> d1 = c1->domain();
                const std::pair<double, double> d2 = c2->domain();
                const Point q1 = c1->point_at(0.5 * (d1.first + d1.second));
                const Point q2 = c2->point_at(0.5 * (d2.first + d2.second));
                const double su = ch.shift_u[p] + q1[0] - q2[0];
                const double sv = ch.shift_v[p] + q1[1] - q2[1];
                const SfWire& W = m_wires[w2];
                const int n2 = (int)W.inputs.size();
                std::vector<int> ni;
                std::vector<double> nsu, nsv;
                for (int k = 0; k < p; ++k) {
                    ni.push_back(ch.inputs[k]);
                    nsu.push_back(ch.shift_u[k]);
                    nsv.push_back(ch.shift_v[k]);
                }
                for (int k = 1; k < n2; ++k) {                 // W rotated to start after j
                    ni.push_back(W.inputs[(j + k) % n2]);
                    nsu.push_back(su);
                    nsv.push_back(sv);
                }
                for (int k = p + 1; k < (int)ch.inputs.size(); ++k) {
                    ni.push_back(ch.inputs[k]);
                    nsu.push_back(ch.shift_u[k]);
                    nsv.push_back(ch.shift_v[k]);
                }
                ch.inputs.swap(ni);
                ch.shift_u.swap(nsu);
                ch.shift_v.swap(nsv);
                ch.seam_composed = true;
                consumed[w2] = 1;
                ++m_rep.seam_merges;
                grew = true;
                break;
            }
        }
        out.push_back(ch);
    }
    m_wires.swap(out);
}

SfdPoly SfdSplitter::wire_poly(const SfWire& w) const {
    SfdPoly p;
    const int per_span = std::max(1, m_opt.wire_samples / 8);
    std::vector<double> ts;
    for (size_t q = 0; q < w.inputs.size(); ++q) {
        const SfInputEdge& e = m_in[w.inputs[q]];
        const bool fwd = (e.sense == SfSense::Forward);
        sfd_sample_params(*e.pcurve, fwd, per_span, ts);
        const double su = w.shift_u.empty() ? 0.0 : w.shift_u[q];
        const double sv = w.shift_v.empty() ? 0.0 : w.shift_v[q];
        for (size_t k = (q == 0 ? 0 : 1); k < ts.size(); ++k) {
            const Point pt = e.pcurve->point_at(ts[k]);
            p.u.push_back(pt[0] + su);
            p.v.push_back(pt[1] + sv);
        }
    }
    return p;
}

double SfdSplitter::poly_area(const SfdPoly& p) {
    const size_t n = p.u.size();
    if (n < 3) return 0.0;
    double s = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        s += p.u[i] * p.v[j] - p.u[j] * p.v[i];
    }
    return 0.5 * s;
}

bool SfdSplitter::point_in_poly(const SfdPoly& p, double u, double v) {
    const size_t n = p.u.size();
    if (n < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double ui = p.u[i], vi = p.v[i], uj = p.u[j], vj = p.v[j];
        if ((vi > v) != (vj > v)) {
            const double x = uj + (v - vi) * (uj - ui) / (vj - vi);
            if (u < x) inside = !inside;
        }
    }
    return inside;
}

bool SfdSplitter::wire_sample(const SfWire& w, double& u, double& v) const {
    for (size_t q = 0; q < w.inputs.size(); ++q) {
        const SfInputEdge& e = m_in[w.inputs[q]];
        if (e.degenerate) continue;
        const std::pair<double, double> d = e.pcurve->domain();
        const Point pt = e.pcurve->point_at(0.5 * (d.first + d.second));
        u = pt[0];
        v = pt[1];
        return true;
    }
    return false;
}

/// OCCT PerformAreas (BOPAlgo_BuilderFace.cxx:387-614): wire role from the SIGN of the signed UV
/// area only (G5), then holes to their TIGHTEST containing growth wire.
void SfdSplitter::areas(SfResult& out) {
    const int nw = (int)m_wires.size();
    if (nw == 0) { m_rep.add(SfAlert::NoLoops, -1); return; }

    std::vector<SfdPoly> polys(nw);
    std::vector<std::set<const BdsPaveBlock*> > pbs(nw);
    for (int i = 0; i < nw; ++i) {
        polys[i] = wire_poly(m_wires[i]);
        m_wires[i].area_uv = poly_area(polys[i]);
        for (size_t q = 0; q < m_wires[i].inputs.size(); ++q)
            pbs[i].insert(m_in[m_wires[i].inputs[q]].pb.get());
        // The "is this wire degenerate" threshold is a MODEL-SPACE area pushed through the
        // metric, never a fraction of the UV domain.
        double tu = 0.0, tv = 0.0;
        double su = 0.0, sv = 0.0;
        if (wire_sample(m_wires[i], su, sv) &&
            sf_uv_tolerance(m_srf, su, sv, m_opt.tol3d, tu, tv)) {
            if (std::fabs(m_wires[i].area_uv) < 4.0 * tu * tv) {
                m_wires[i].degenerate_area = true;
                m_rep.add(SfAlert::WireDegenerateArea, m_wires[i].inputs[0]);
            }
        }
        m_wires[i].is_hole = (m_wires[i].area_uv < 0.0);
    }

    std::vector<int> outer_of(nw, -1);
    std::vector<int> face_of(nw, -1);
    for (int i = 0; i < nw; ++i) {
        if (m_wires[i].is_hole) continue;
        face_of[i] = (int)out.faces.size();
        SfFace f;
        f.outer = m_wires[i];
        out.faces.push_back(f);
    }
    if (out.faces.empty()) {
        // Every wire negative: the whole face is a hole in the enclosing parameterisation.
        // Reported, never silently reinterpreted.
        m_rep.add(SfAlert::NoLoops, -1);
        return;
    }
    for (int h = 0; h < nw; ++h) {
        if (!m_wires[h].is_hole) continue;
        double su = 0.0, sv = 0.0;
        if (!wire_sample(m_wires[h], su, sv)) { m_rep.add(SfAlert::HoleUnassigned, h); continue; }
        int best = -1;
        double best_area = 1e300;
        for (int o = 0; o < nw; ++o) {
            if (m_wires[o].is_hole || face_of[o] < 0) continue;
            // A wire that SHARES AN EDGE with the candidate owner is not inside it: the two are
            // the two sides of the same cut. This is an INDEX comparison (OCCT :870-875).
            bool shares = false;
            for (std::set<const BdsPaveBlock*>::const_iterator it = pbs[h].begin();
                 it != pbs[h].end() && !shares; ++it)
                if (pbs[o].count(*it)) shares = true;
            if (shares) continue;
            if (!point_in_poly(polys[o], su, sv)) continue;
            if (m_wires[o].area_uv < best_area) { best_area = m_wires[o].area_uv; best = o; }
        }
        if (best < 0) { m_rep.add(SfAlert::HoleUnassigned, h); continue; }
        outer_of[h] = best;
        out.faces[(size_t)face_of[best]].holes.push_back(m_wires[h]);
    }
}

/// OCCT PerformLoops post-treatment (:284-321) + PerformInternalShapes (:618-778): everything
/// the walk did not consume becomes an INTERNAL wire inside the face that contains it, or is
/// reported. Nothing vanishes (G9).
void SfdSplitter::internals(SfResult& out) {
    const int n = (int)m_in.size();
    std::vector<int> left;
    std::set<const BdsPaveBlock*> seen;
    for (int i = 0; i < n; ++i) {
        if (m_used[i]) continue;
        if (!m_in[i].pb || !m_in[i].pcurve) continue;
        if (seen.count(m_in[i].pb.get())) continue;
        seen.insert(m_in[i].pb.get());
        left.push_back(i);
        m_rep.add(SfAlert::EdgeNotConsumed, i);
    }
    if (left.empty() || m_opt.avoid_internal || out.faces.empty()) {
        out.unused = left;
        return;
    }
    // Flood into connexity blocks by SHARED ARENA VERTEX (not by proximity).
    std::vector<char> done(left.size(), 0);
    for (size_t s = 0; s < left.size(); ++s) {
        if (done[s]) continue;
        std::vector<int> blk;
        std::set<int> verts;
        blk.push_back(left[s]);
        done[s] = 1;
        verts.insert(m_in[left[s]].pb->pave1.vertex);
        verts.insert(m_in[left[s]].pb->pave2.vertex);
        bool grew = true;
        while (grew) {
            grew = false;
            for (size_t k = 0; k < left.size(); ++k) {
                if (done[k]) continue;
                const int a = m_in[left[k]].pb->pave1.vertex;
                const int b = m_in[left[k]].pb->pave2.vertex;
                if (!verts.count(a) && !verts.count(b)) continue;
                done[k] = 1;
                blk.push_back(left[k]);
                verts.insert(a);
                verts.insert(b);
                grew = true;
            }
        }
        SfWire w;
        w.inputs = blk;
        double su = 0.0, sv = 0.0;
        if (!wire_sample(w, su, sv)) continue;
        int best = -1;
        double best_area = 1e300;
        for (size_t f = 0; f < out.faces.size(); ++f) {
            const SfdPoly p = wire_poly(out.faces[f].outer);
            if (!point_in_poly(p, su, sv)) continue;
            if (out.faces[f].outer.area_uv < best_area) {
                best_area = out.faces[f].outer.area_uv;
                best = (int)f;
            }
        }
        if (best < 0) continue;
        out.faces[(size_t)best].internals.push_back(w);
        for (size_t q = 0; q < blk.size(); ++q) out.unused.push_back(blk[q]);
    }
}

SfResult SfdSplitter::run() {
    SfResult out;
    classify_multiplicity();
    build_nodes();
    m_cap = m_opt.max_walk_steps > 0 ? m_opt.max_walk_steps : (8 * (int)m_in.size() + 64);
    prune_to_fixpoint();
    compute_angles();
    walk();
    compose_across_seams();
    areas(out);
    internals(out);
    out.report = m_rep;
    // G1: this subsystem has no allocator for vertices or edges. The counters exist so the
    // invariant is asserted, not assumed.
    out.report.minted_edges = 0;
    out.report.minted_vertices = 0;
    return out;
}

}  // namespace sfdetail

SfResult sf_split_face(const BdsArena& arena, const NurbsSurface& surface,
                       const std::vector<SfInputEdge>& inputs, const SfOptions& opt) {
    SfdSplitter s(arena, surface, inputs, opt);
    return s.run();
}


///////////////////////////////////////////////////////////////////////////////////////////
// Periodic continuation
///////////////////////////////////////////////////////////////////////////////////////////

namespace sfdetail {

/// One period of periodic continuation, prepended (`before`) or appended, in direction `dir`.
static bool sfd_extend_one(const NurbsSurface& s, int dir, bool before, NurbsSurface& out) {
    const int o0 = s.order(0), o1 = s.order(1);
    const int n0 = s.cv_count(0), n1 = s.cv_count(1);
    const int nd = (dir == 0) ? n0 : n1;
    const int od = (dir == 0) ? o0 : o1;
    const int P = nd - 1;                      // CV period: row 0 and row nd-1 are the same row
    if (P < 1) return false;
    const std::vector<double> k = s.get_nurbsknots(dir);
    const int K = (int)k.size();
    if (K != od + nd - 2 || K < P) return false;
    const std::pair<double, double> d = s.domain(dir);
    const double T = d.second - d.first;
    if (!(T > 0.0)) return false;

    // The net must actually repeat: first row == last row, to model tolerance.
    for (int j = 0; j < ((dir == 0) ? n1 : n0); ++j) {
        double a[4], b[4];
        const bool oka = (dir == 0) ? s.get_cv_4d(0, j, a[0], a[1], a[2], a[3])
                                    : s.get_cv_4d(j, 0, a[0], a[1], a[2], a[3]);
        const bool okb = (dir == 0) ? s.get_cv_4d(nd - 1, j, b[0], b[1], b[2], b[3])
                                    : s.get_cv_4d(j, nd - 1, b[0], b[1], b[2], b[3]);
        if (!oka || !okb) return false;
        for (int c = 0; c < 4; ++c)
            if (std::fabs(a[c] - b[c]) > 1e-9 * (1.0 + std::fabs(a[c]))) return false;
    }
    // ... and the knot pattern must repeat by exactly one period.
    for (int i = 0; i + P < K; ++i)
        if (std::fabs(k[(size_t)(i + P)] - (k[(size_t)i] + T)) > 1e-9 * (1.0 + std::fabs(T)))
            return false;

    const int nd2 = nd + P;
    const int nn0 = (dir == 0) ? nd2 : n0;
    const int nn1 = (dir == 0) ? n1 : nd2;
    NurbsSurface r(3, s.is_rational() ? 1 : 0, o0, o1, nn0, nn1);

    std::vector<double> kn;
    if (before) {
        for (int i = 0; i < P; ++i) kn.push_back(k[(size_t)i] - T);
        for (int i = 0; i < K; ++i) kn.push_back(k[(size_t)i]);
    } else {
        for (int i = 0; i < K; ++i) kn.push_back(k[(size_t)i]);
        for (int i = K - P; i < K; ++i) kn.push_back(k[(size_t)i] + T);
    }
    if ((int)kn.size() != od + nd2 - 2) return false;
    for (int i = 0; i < (int)kn.size(); ++i) r.set_nurbsknot(dir, i, kn[(size_t)i]);
    const std::vector<double> ko = s.get_nurbsknots(1 - dir);
    for (int i = 0; i < (int)ko.size(); ++i) r.set_nurbsknot(1 - dir, i, ko[(size_t)i]);

    for (int i = 0; i < nd2; ++i) {
        // source row of the periodic net
        const int src = before ? (i < P ? i : i - P) : (i < nd ? i : i - P);
        for (int j = 0; j < ((dir == 0) ? n1 : n0); ++j) {
            double x, y, z, w;
            const bool ok = (dir == 0) ? s.get_cv_4d(src, j, x, y, z, w)
                                       : s.get_cv_4d(j, src, x, y, z, w);
            if (!ok) return false;
            const bool okw = (dir == 0) ? r.set_cv_4d(i, j, x, y, z, w)
                                        : r.set_cv_4d(j, i, x, y, z, w);
            if (!okw) return false;
        }
    }
    out = r;
    return true;
}

}  // namespace sfdetail

bool sf_periodic_extend(const NurbsSurface& s, int dir, double lo, double hi, NurbsSurface& out) {
    if (dir != 0 && dir != 1) return false;
    if (!s.is_valid()) return false;
    NurbsSurface cur = s;
    const double eps = 1e-12;
    for (int guard = 0; guard < 8; ++guard) {
        const std::pair<double, double> d = cur.domain(dir);
        if (lo >= d.first - eps && hi <= d.second + eps) { out = cur; return true; }
        NurbsSurface nxt;
        const bool before = (lo < d.first - eps);
        if (!sfd_extend_one(cur, dir, before, nxt)) return false;
        cur = nxt;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Emission
///////////////////////////////////////////////////////////////////////////////////////////

int SfEmitter::edge_of(const BdsPaveBlock* pb) const {
    std::unordered_map<const BdsPaveBlock*, int>::const_iterator it = m_pb_edge.find(pb);
    return it == m_pb_edge.end() ? -1 : it->second;
}

int SfEmitter::vertex_of(int av) const {
    std::unordered_map<int, int>::const_iterator it = m_v_map.find(av);
    return it == m_v_map.end() ? -1 : it->second;
}

int SfEmitter::ensure_vertex(const BdsArena& arena, int raw_av) {
    // Same identity rule as build_nodes: a fused node is named by its resolved index, so two
    // pave blocks that meet there produce ONE BRep topology vertex.
    const int av = arena.resolve_sd(raw_av);
    std::unordered_map<int, int>::const_iterator it = m_v_map.find(av);
    if (it != m_v_map.end()) return it->second;
    const int pi = m_out.add_vertex(arena.vertex_point(av));
    BRepVertex tv;
    tv.point_index = pi;
    m_out.m_topology_vertices.push_back(tv);
    const int vi = (int)m_out.m_topology_vertices.size() - 1;
    m_v_map[av] = vi;
    return vi;
}

int SfEmitter::ensure_edge(const BdsArena& arena, const BdsPB& pb, bool) {
    std::unordered_map<const BdsPaveBlock*, int>::const_iterator it = m_pb_edge.find(pb.get());
    if (it != m_pb_edge.end()) { ++m_reused; return it->second; }
    const int v1 = ensure_vertex(arena, pb->pave1.vertex);
    const int v2 = ensure_vertex(arena, pb->pave2.vertex);
    const NurbsCurve* src = arena.edge_curve(pb->original_edge);
    int ci = -1;
    if (src && src->is_valid()) {
        NurbsCurve c = *src;
        const std::pair<double, double> d = c.domain();
        if (pb->pave1.t > d.first || pb->pave2.t < d.second) c.trim(pb->pave1.t, pb->pave2.t);
        ci = m_out.add_curve_3d(c);
    }
    const int ei = m_out.add_edge(ci, v1, v2);
    m_out.m_topology_vertices[(size_t)v1].edge_indices.push_back(ei);
    if (v2 != v1) m_out.m_topology_vertices[(size_t)v2].edge_indices.push_back(ei);
    m_pb_edge[pb.get()] = ei;
    return ei;
}

int SfEmitter::emit_wire(const BdsArena& arena, const std::vector<SfInputEdge>& inputs,
                         const SfWire& w, int face_idx, BRepLoopType type, bool internal) {
    const int li = m_out.add_loop(face_idx, type);
    for (size_t q = 0; q < w.inputs.size(); ++q) {
        const SfInputEdge& e = inputs[(size_t)w.inputs[q]];
        // A degenerate (pole) input gets an edge record only under the OCCT convention; see
        // SfEmitter::set_pole_edges. `-1` means "singular trim, pcurve only, no 3D edge",
        // which is what this kernel's own primitives do.
        const int ei = (e.degenerate && !m_pole_edges) ? -1
                                                       : ensure_edge(arena, e.pb, e.degenerate);
        // PERIOD SHIFT: a face composed across a seam carries part of its wire one whole period
        // away in UV. The shift is a translation of the stored pcurve, not a new identity.
        NurbsCurve pc = *e.pcurve;
        if (!w.shift_u.empty() && (w.shift_u[q] != 0.0 || w.shift_v[q] != 0.0)) {
            for (int k = 0; k < pc.cv_count(); ++k) {
                const Point cvp = pc.get_cv(k);
                pc.set_cv(k, Point(cvp[0] + w.shift_u[q], cvp[1] + w.shift_v[q], cvp[2]));
            }
        }
        // THE KERNEL'S STORED CONVENTION, measured on its own primitives: a trim's 2D curve runs
        // in the LOOP'S TRAVERSAL ORDER and `reversed` records the pcurve-vs-3D-curve relation.
        // (Every boundary trim of every box/sphere/cylinder face satisfies "pcurve start ==
        // edge's pave1 vertex" xor "reversed".) The splitter's own contract is the opposite --
        // the input pcurve is always the pave block's NATURAL direction with the traversal in
        // `sense` -- so the direction has to be re-applied HERE, once, at the boundary between
        // the two models. Without it every consumer that builds a UV polygon by concatenating a
        // loop's pcurves head-to-tail (SameDomain::uv_in_trims, BRep::loop_material_left, the
        // STEP writer) reads a zig-zag polygon and misclassifies the region.
        if (e.sense == SfSense::Reversed) pc.reverse();
        const int c2 = m_out.add_curve_2d(pc);
        BRepTrimType tt = BRepTrimType::Mated;
        if (e.degenerate) tt = BRepTrimType::Singular;
        else if (internal) tt = BRepTrimType::Boundary;
        else {
            // A SEAM occurrence is one whose pave block is supplied twice with two DIFFERENT
            // pcurve objects on this face (BOPTools_AlgoTools3D.cxx:200-231).
            int same = 0, diff = 0;
            for (size_t k = 0; k < inputs.size(); ++k) {
                if (inputs[k].pb.get() != e.pb.get()) continue;
                ++same;
                if (inputs[k].pcurve != e.pcurve) ++diff;
            }
            if (same >= 2 && diff > 0) tt = BRepTrimType::Seam;
        }
        m_out.add_trim(c2, ei, li, e.sense == SfSense::Reversed, tt);
    }
    return li;
}

int SfEmitter::emit(const BdsArena& arena, const NurbsSurface& surface,
                    const std::vector<SfInputEdge>& inputs, const SfResult& res,
                    bool face_reversed) {
    if (res.faces.empty()) return 0;
    const int si_base = m_out.add_surface(surface);
    for (size_t f = 0; f < res.faces.size(); ++f) {
        const SfFace& a = res.faces[f];
        // A face composed ACROSS a seam reaches outside the chart's stored rectangle. Give it a
        // periodically continued surface so it is actually evaluable; without this the face is
        // topologically right and geometrically unreadable.
        int si = si_base;
        bool composed = a.outer.seam_composed;
        for (size_t h = 0; h < a.holes.size(); ++h) composed = composed || a.holes[h].seam_composed;
        if (composed) {
            double ulo = 1e300, uhi = -1e300, vlo = 1e300, vhi = -1e300;
            std::vector<const SfWire*> ws;
            ws.push_back(&a.outer);
            for (size_t h = 0; h < a.holes.size(); ++h) ws.push_back(&a.holes[h]);
            for (size_t w = 0; w < ws.size(); ++w)
                for (size_t q = 0; q < ws[w]->inputs.size(); ++q) {
                    const NurbsCurve* pc = inputs[(size_t)ws[w]->inputs[q]].pcurve;
                    const double su = ws[w]->shift_u.empty() ? 0.0 : ws[w]->shift_u[q];
                    const double sv = ws[w]->shift_v.empty() ? 0.0 : ws[w]->shift_v[q];
                    for (int c = 0; c < pc->cv_count(); ++c) {
                        const Point cp = pc->get_cv(c);
                        ulo = std::min(ulo, cp[0] + su); uhi = std::max(uhi, cp[0] + su);
                        vlo = std::min(vlo, cp[1] + sv); vhi = std::max(vhi, cp[1] + sv);
                    }
                }
            NurbsSurface ext = surface, tmp;
            bool ok = true;
            const std::pair<double, double> d0 = surface.domain(0);
            const std::pair<double, double> d1 = surface.domain(1);
            if (ulo < d0.first - 1e-12 || uhi > d0.second + 1e-12) {
                if (sf_periodic_extend(ext, 0, ulo, uhi, tmp)) ext = tmp; else ok = false;
            }
            if (ok && (vlo < d1.first - 1e-12 || vhi > d1.second + 1e-12)) {
                if (sf_periodic_extend(ext, 1, vlo, vhi, tmp)) ext = tmp; else ok = false;
            }
            if (ok) { si = m_out.add_surface(ext); ++m_extended; }
            else ++m_inextensible;
        }
        const int fi = m_out.add_face(si, face_reversed);
        emit_wire(arena, inputs, a.outer, fi, BRepLoopType::Outer, false);
        for (size_t h = 0; h < a.holes.size(); ++h)
            emit_wire(arena, inputs, a.holes[h], fi, BRepLoopType::Inner, false);
        for (size_t h = 0; h < a.internals.size(); ++h)
            emit_wire(arena, inputs, a.internals[h], fi, BRepLoopType::Inner, true);
    }
    return (int)res.faces.size();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Verdict metric
///////////////////////////////////////////////////////////////////////////////////////////

SfManifold sf_manifold(const BRep& b) {
    SfManifold m;
    // Same degeneracy scale BRep::is_solid() uses (brep.cpp:1065-1073): a fraction of the model
    // bounding-box diagonal, so it is a MODEL-SPACE length, not a parametric one.
    double xmn = 1e300, ymn = 1e300, zmn = 1e300, xmx = -1e300, ymx = -1e300, zmx = -1e300;
    for (size_t i = 0; i < b.m_vertices.size(); ++i) {
        const Point& p = b.m_vertices[i];
        xmn = std::min(xmn, p[0]); ymn = std::min(ymn, p[1]); zmn = std::min(zmn, p[2]);
        xmx = std::max(xmx, p[0]); ymx = std::max(ymx, p[1]); zmx = std::max(zmx, p[2]);
    }
    const double diag = b.m_vertices.empty() ? 1.0
        : std::sqrt((xmx - xmn) * (xmx - xmn) + (ymx - ymn) * (ymx - ymn) +
                    (zmx - zmn) * (zmx - zmn));
    const double deg_tol = std::max(diag * 1e-7, 1e-12);

    m.edges = (int)b.m_topology_edges.size();
    for (size_t i = 0; i < b.m_topology_edges.size(); ++i) {
        const BRepEdge& e = b.m_topology_edges[i];
        const int nt = (int)e.trim_indices.size();
        if (nt == 2) {
            // A SEAM edge is used twice by ONE face's wires. It is not naked and it is not a
            // second edge: it appears ONCE in the edge list.
            int f0 = -1, f1 = -1;
            for (int k = 0; k < 2; ++k) {
                const int ti = e.trim_indices[(size_t)k];
                if (ti < 0 || ti >= (int)b.m_trims.size()) continue;
                const int li = b.m_trims[(size_t)ti].loop_index;
                if (li < 0 || li >= (int)b.m_loops.size()) continue;
                (k == 0 ? f0 : f1) = b.m_loops[(size_t)li].face_index;
            }
            if (f0 >= 0 && f0 == f1) ++m.seam_edges;
            continue;
        }
        if (nt == 0) { ++m.orphan; continue; }
        // A DEGENERATE edge (pole, apex) is watertight by construction. Excluded from the
        // verdict exactly as BRep::is_solid() (brep.cpp:1090-1102) excludes it.
        const int ci = e.curve_3d_index;
        bool degen = false;
        if (ci >= 0 && ci < (int)b.m_curves_3d.size()) {
            const NurbsCurve& c = b.m_curves_3d[(size_t)ci];
            if (c.is_valid()) {
                const std::pair<double, double> dc = c.domain();
                const Point p0 = c.point_at(dc.first);
                double ext = 0.0;
                for (int k = 1; k <= 4; ++k)
                    ext = std::max(ext, p0.distance(
                        c.point_at(dc.first + (dc.second - dc.first) * k / 4.0)));
                degen = (ext < deg_tol);
            }
        }
        if (degen) { ++m.degenerate; continue; }
        if (nt == 1) ++m.naked;
        else ++m.nonmanifold;
    }
    return m;
}

}  // namespace v2sf
}  // namespace session_cpp
