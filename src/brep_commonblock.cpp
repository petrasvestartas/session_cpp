#include "brep_commonblock.h"
#include "closest.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <tuple>

namespace session_cpp {

namespace {

constexpr double CB_CONFUSION = 1e-7;

// Distance from p to an operand's existing EDGE (3D curve), nearest edge index out.
double nearest_edge_distance(const BRep& b, const Point& p, int* edge) {
    double best = 1e300;
    int bi = -1;
    for (int ei = 0; ei < (int)b.m_topology_edges.size(); ++ei) {
        int ci = b.m_topology_edges[ei].curve_3d_index;
        if (ci < 0 || ci >= (int)b.m_curves_3d.size()) continue;
        auto [t, d] = Closest::curve_point(b.m_curves_3d[ci], p);
        (void)t;
        if (d < best) { best = d; bi = ei; }
    }
    if (edge) *edge = bi;
    return best;
}

} // namespace

Point CommonBlock::point_at_frac(double s) const {
    return curve ? curve->point_at(t0 + (t1 - t0) * s) : Point(0, 0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Membership + the splitting predicate
///////////////////////////////////////////////////////////////////////////////////////////

bool cb_on_operand(const BRep& b, const Point& p, double band, CBOn on,
                   int* face, int* edge, double* dist) {
    if (face) *face = -1;
    if (edge) *edge = -1;
    double d;
    if (on == CBOn::Face) {
        int f = -1;
        d = sd_distance_to_boundary(b, p, &f);
        if (face) *face = f;
    } else {
        int e = -1;
        d = nearest_edge_distance(b, p, &e);
        if (edge) *edge = e;
    }
    if (dist) *dist = d;
    return d <= band;
}

CBSplit cb_split_chain(const NurbsCurve& curve, const BRep& A, const BRep& B,
                       const CBParams& prm) {
    CBSplit out;
    auto dm = curve.domain();
    double ta = dm.first, tb = dm.second;
    if (!(tb > ta)) return out;

    // State at a parameter: coincident iff on BOTH operands. Witnesses recorded for the members.
    struct Probe { bool on_a, on_b; int fa, fb, ea, eb; double da, db; };
    auto probe = [&](double t) {
        Probe pr{};
        Point p = curve.point_at(t);
        pr.on_a = cb_on_operand(A, p, prm.band, prm.on, &pr.fa, &pr.ea, &pr.da);
        pr.on_b = cb_on_operand(B, p, prm.band, prm.on, &pr.fb, &pr.eb, &pr.db);
        return pr;
    };
    auto coincident_of = [](const Probe& pr) { return pr.on_a && pr.on_b; };

    int n = std::max(prm.samples, 2);
    std::vector<double> ts(n + 1);
    std::vector<Probe> pv(n + 1);
    for (int i = 0; i <= n; ++i) {
        ts[i] = ta + (tb - ta) * i / n;
        pv[i] = probe(ts[i]);
    }

    // Localise every state transition by bisection, so a block boundary lands on the true edge
    // of the coincident region instead of on whichever sample happened to straddle it.
    std::vector<double> cuts;
    for (int i = 0; i < n; ++i) {
        if (coincident_of(pv[i]) == coincident_of(pv[i + 1])) continue;
        double lo = ts[i], hi = ts[i + 1];
        bool lo_state = coincident_of(pv[i]);
        for (int k = 0; k < prm.bisect && (hi - lo) > (tb - ta) * 1e-12; ++k) {
            double mid = 0.5 * (lo + hi);
            if (coincident_of(probe(mid)) == lo_state) lo = mid;
            else hi = mid;
        }
        cuts.push_back(0.5 * (lo + hi));
    }

    // Absorb micro intervals (OCCT's micro-edge policy: never emit a block inside the noise).
    std::vector<double> bounds;
    bounds.push_back(ta);
    double micro = (tb - ta) * std::max(prm.micro_frac, 0.0);
    for (double c : cuts)
        if (c - bounds.back() > micro && tb - c > micro) bounds.push_back(c);
    bounds.push_back(tb);

    for (size_t i = 0; i + 1 < bounds.size(); ++i) {
        CommonBlock blk;
        blk.curve = &curve;
        blk.t0 = bounds[i];
        blk.t1 = bounds[i + 1];
        // Interior probe decides the block's state (the ends sit ON transitions by construction).
        Probe mid = probe(0.5 * (blk.t0 + blk.t1));
        blk.coincident = coincident_of(mid);
        blk.operands_mask = (mid.on_a ? 1 : 0) | (mid.on_b ? 2 : 0);
        // Members: sample the interior and record the witness entity per operand, keeping the
        // MAX distance seen (that is what the tolerance computation consumes).
        CBMember ma, mb;
        ma.brep = &A; ma.operand = 0;
        mb.brep = &B; mb.operand = 1;
        bool has_a = false, has_b = false;
        const int ns = 9;
        for (int k = 1; k <= ns; ++k) {
            Probe pr = probe(blk.t0 + (blk.t1 - blk.t0) * k / (ns + 1));
            if (pr.on_a) {
                has_a = true; ma.face = pr.fa; ma.edge = pr.ea;
                ma.dist = std::max(ma.dist, pr.da);
            }
            if (pr.on_b) {
                has_b = true; mb.face = pr.fb; mb.edge = pr.eb;
                mb.dist = std::max(mb.dist, pr.db);
            }
        }
        if (has_a) blk.members.push_back(ma);
        if (has_b) blk.members.push_back(mb);
        blk.paves.push_back({blk.t0, i == 0 ? 0 : 1});
        blk.paves.push_back({blk.t1, i + 2 == bounds.size() ? 0 : 1});
        cb_tolerance(blk);
        if (blk.coincident) ++out.n_coincident;
        out.blocks.push_back(blk);
    }
    out.n_transitions = (int)bounds.size() - 2;
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// ComputeToleranceOfCB
///////////////////////////////////////////////////////////////////////////////////////////

double cb_tolerance(CommonBlock& blk, int samples) {
    // Seed = tolerance of the representative's ORIGINAL entity. This kernel stores no per-entity
    // tolerances (BRepEdge/BRepFace carry none -- phase P5), so every member tol is 0 and the
    // result is the pure max deviation. The OCCT structure (tol(mate) + distance) is preserved so
    // P5 wires in by filling CBMember::tol. See kb/p3_integration_notes.md.
    double tol_max = 0.0;
    for (const auto& m : blk.members) tol_max = std::max(tol_max, m.tol);
    if (blk.members.size() < 2 && blk.members.empty()) { blk.tolerance = tol_max; return tol_max; }
    if (!blk.curve) { blk.tolerance = tol_max; return tol_max; }

    // 11 interior points at dt = (t1-t0)/12 over the block's OWN range (OCCT samples the
    // representative pave block's range, never the whole source curve).
    const int n = std::max(samples, 1);
    double dt = (blk.t1 - blk.t0) / (n + 1);
    for (int i = 1; i <= n; ++i) {
        Point p = blk.curve->point_at(blk.t0 + dt * i);
        for (const auto& m : blk.members) {
            if (!m.brep) continue;
            double d = 1e300;
            if (m.edge >= 0 && m.edge < (int)m.brep->m_topology_edges.size()) {
                int ci = m.brep->m_topology_edges[m.edge].curve_3d_index;
                if (ci >= 0 && ci < (int)m.brep->m_curves_3d.size())
                    d = Closest::curve_point(m.brep->m_curves_3d[ci], p).second;
            }
            if (d > 1e299) d = sd_distance_to_boundary(*m.brep, p, nullptr);
            if (d < 1e299) tol_max = std::max(tol_max, m.tol + d);
        }
    }
    blk.tolerance = tol_max;
    return tol_max;
}

///////////////////////////////////////////////////////////////////////////////////////////
// PerformCommonBlocks
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> cb_perform_common_blocks(std::vector<CommonBlock>& blocks) {
    int n = (int)blocks.size();
    std::vector<int> rep(n);
    for (int i = 0; i < n; ++i) rep[i] = i;
    std::function<int(int)> find = [&](int i) {
        while (rep[i] != i) i = rep[i];
        return i;
    };
    // Connected components over "shares a member entity" (OCCT: MakeBlocks over the PB->list<PB>
    // map). Only coincident blocks can join a common block.
    auto key_of = [](const CBMember& m) {
        return std::make_tuple((const void*)m.brep, m.face, m.edge);
    };
    std::map<std::tuple<const void*, int, int>, std::vector<int>> by_entity;
    for (int i = 0; i < n; ++i) {
        if (!blocks[i].coincident) continue;
        for (const auto& m : blocks[i].members) by_entity[key_of(m)].push_back(i);
    }
    for (auto& kv : by_entity)
        for (size_t k = 1; k < kv.second.size(); ++k) {
            int a = find(kv.second[0]), b = find(kv.second[k]);
            if (a != b) rep[std::max(a, b)] = std::min(a, b);
        }
    for (int i = 0; i < n; ++i) rep[i] = find(i);

    std::map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i)
        if (blocks[i].coincident) groups[rep[i]].push_back(i);

    std::vector<int> out(n, -1);
    for (auto& kv : groups) {
        // "groups of fewer than 2 PBs are skipped" (BOPAlgo_Tools.cxx:134-137)
        if (kv.second.size() < 2) continue;
        // Merge member lists into the representative (min index), deduplicated.
        CommonBlock& r = blocks[kv.first];
        for (int i : kv.second) {
            out[i] = kv.first;
            if (i == kv.first) continue;
            for (const auto& m : blocks[i].members) {
                bool dup = false;
                for (const auto& e : r.members)
                    if (key_of(e) == key_of(m)) { dup = true; break; }
                if (!dup) r.members.push_back(m);
            }
        }
        cb_tolerance(r);
        for (int i : kv.second) blocks[i].tolerance = r.tolerance;
    }
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Region classification + op-table
///////////////////////////////////////////////////////////////////////////////////////////

SDState cb_classify_region(CBRegion& reg, const BRep& other, double tol,
                           const std::vector<double>& osign_S,
                           const std::vector<double>& osign_other) {
    if (!reg.brep) return SDState::Out;
    reg.state = sd_classify_samples(*reg.brep, reg.face, reg.uvs, other, tol, osign_S, osign_other);
    reg.orient = (reg.state == SDState::OnOpposite) ? 1 : 0;
    return reg.state;
}

SDVerdict cb_select_region(SDOp op, const CBRegion& reg) {
    if (reg.state == SDState::OnSame || reg.state == SDState::OnOpposite)
        return sd_select_sd_face(op, reg.operand, reg.state == SDState::OnSame);
    return sd_select_face(op, reg.operand, reg.state);
}

std::vector<CBRegion> cb_partition_face(const BRep& S, int face, int operand, const BRep& other,
                                        double tol, int probes,
                                        const std::vector<double>& osign_S,
                                        const std::vector<double>& osign_other) {
    std::vector<CBRegion> out;
    auto uvs = SameDomain::samples_in_face(S, face, std::max(probes, 1));
    if (uvs.empty()) return out;
    std::vector<double> sgS_local, sgO_local;
    const std::vector<double>* sgS = &osign_S;
    const std::vector<double>* sgO = &osign_other;
    if (osign_S.empty()) { sgS_local = S.face_outward_signs(); sgS = &sgS_local; }
    if (osign_other.empty()) { sgO_local = other.face_outward_signs(); sgO = &sgO_local; }

    // Classify each sample INDIVIDUALLY (a one-sample region is exact under the areal rule),
    // then group by state. A partially coincident face yields >= 2 groups -- that split is the
    // detection signal a whole-face verdict cannot produce.
    std::map<int, CBRegion> by_state;
    for (const auto& uv : uvs) {
        CBRegion one;
        one.brep = &S; one.operand = operand; one.face = face;
        one.uvs.push_back(uv);
        SDState st = cb_classify_region(one, other, tol, *sgS, *sgO);
        auto it = by_state.find((int)st);
        if (it == by_state.end()) {
            CBRegion r;
            r.brep = &S; r.operand = operand; r.face = face;
            r.state = st; r.orient = one.orient;
            r.uvs.push_back(uv);
            by_state.emplace((int)st, std::move(r));
        } else {
            it->second.uvs.push_back(uv);
        }
    }
    for (auto& kv : by_state) out.push_back(std::move(kv.second));
    return out;
}

} // namespace session_cpp
