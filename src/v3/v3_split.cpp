// v3 face splitting: pave-block edge splitting + UV arrangement (WireSplitter).
#include "v3_split.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace v3 {

// ============================================================================
// split_edges_at_paves
// ============================================================================

std::map<int, std::vector<std::array<double, 3>>> split_edges_at_paves(
    Solid& S, std::map<int, std::vector<double>>& paves,
    std::map<std::pair<int, double>, int>* vtx_of) {
    std::map<int, std::vector<std::array<double, 3>>> pieces_out;
    // coedges per edge
    std::map<int, std::vector<int>> edge_ces;
    for (size_t i = 0; i < S.coedges.size(); i++)
        edge_ces[S.coedges[i].edge].push_back((int)i);

    for (auto& kv : paves) {
        int ei = kv.first;
        if (ei < 0 || ei >= (int)S.edges.size()) continue;
        Edge& e = S.edges[ei];
        if (e.degenerate) continue;
        double span = e.t1 - e.t0;
        double dtol = 1e-9 * std::abs(span) + 1e-12;
        // edge length for the 3D dedupe gate (attach and EF piercings of the
        // same physical point differ by curve-deflection amounts)
        double elen = 0;
        for (int k = 0; k < 16; k++)
            elen += e.c.eval(e.t0 + span * k / 16.0)
                        .dist(e.c.eval(e.t0 + span * (k + 1) / 16.0));
        double gate3d = std::max(1e-6, 0.01 * elen);
        std::vector<double> ts;
        for (double t : kv.second)
            if (t > e.t0 + dtol && t < e.t1 - dtol) ts.push_back(t);
        std::sort(ts.begin(), ts.end());
        std::vector<double> dts;
        for (double t : ts) {
            if (!dts.empty() &&
                e.c.eval(t).dist(e.c.eval(dts.back())) < gate3d)
                continue; // same physical point as the previous pave
            dts.push_back(t);
        }
        ts = std::move(dts);
        if (ts.empty()) continue;
        // vertices per pave (dedupe by rounded t on this edge)
        std::vector<double> bounds;
        bounds.push_back(e.t0);
        for (double t : ts) bounds.push_back(t);
        bounds.push_back(e.t1);
        int npieces = (int)ts.size() + 1;
        std::vector<int> vtx(bounds.size());
        vtx.front() = e.v0;
        vtx.back() = e.v1;
        for (size_t i = 1; i + 1 < bounds.size(); i++) {
            S.verts.push_back({e.c.eval(bounds[i]), 1e-7});
            vtx[i] = (int)S.verts.size() - 1;
            if (vtx_of) (*vtx_of)[{ei, bounds[i]}] = vtx[i];
        }
        // piece edges: first piece reuses the parent slot
        std::vector<int> piece_ids(npieces);
        {
            int old_v1 = e.v1;
            double old_t1 = e.t1;
            (void)old_v1;
            e.t1 = bounds[1];
            e.v1 = vtx[1];
            piece_ids[0] = ei;
            (void)old_t1;
        }
        // copy the curve/tol BEFORE the push loop: push_back can reallocate
        // S.edges and the Edge& e above would dangle (heap-use-after-free)
        Cur ec = e.c;
        double etol = e.tol;
        for (int p = 1; p < npieces; p++) {
            Edge ne;
            ne.c = ec;
            ne.t0 = bounds[p];
            ne.t1 = bounds[p + 1];
            ne.v0 = vtx[p];
            ne.v1 = vtx[p + 1];
            ne.tol = etol;
            piece_ids[p] = (int)S.edges.size();
            S.edges.push_back(ne);
        }
        for (int p = 0; p < npieces; p++)
            pieces_out[ei].push_back({bounds[p], bounds[p + 1], (double)piece_ids[p]});

        // split coedges referencing this edge
        for (int cei : edge_ces[ei]) {
            CoEdge& ce = S.coedges[cei];
            if (ce.pt.size() < 2 || ce.pc.size() != ce.pt.size()) continue;
            // cut the pcurve chain at each pave t (insert cut points)
            std::vector<UV2> pc = ce.pc;
            std::vector<double> pt = ce.pt;
            bool fwd = ce.fwd;
            int fidx = ce.face;
            // split positions in the chain: bracket each bound
            std::vector<std::vector<int>> cuts(npieces); // sample indices per piece
            std::vector<std::vector<UV2>> piece_pcs(npieces);
            std::vector<std::vector<double>> piece_pts(npieces);
            int cur = 0;
            piece_pcs[0].push_back(pc[0]);
            piece_pts[0].push_back(pt[0]);
            for (size_t i = 0; i + 1 < pt.size(); i++) {
                double ta = pt[i], tb = pt[i + 1];
                // any bounds strictly inside (ta, tb)?
                double lo = std::min(ta, tb), hi = std::max(ta, tb);
                std::vector<double> inside;
                for (double bt : bounds)
                    if (bt > lo + 1e-12 && bt < hi - 1e-12) inside.push_back(bt);
                std::sort(inside.begin(), inside.end());
                if (ta > tb) std::reverse(inside.begin(), inside.end());
                // emit: go from i to each bound, then to i+1
                for (double bt : inside) {
                    double fr = (bt - ta) / (tb - ta);
                    UV2 cut{pc[i].u + (pc[i + 1].u - pc[i].u) * fr,
                            pc[i].v + (pc[i + 1].v - pc[i].v) * fr};
                    piece_pcs[cur].push_back(cut);
                    piece_pts[cur].push_back(bt);
                    cur++;
                    piece_pcs[cur].push_back(cut);
                    piece_pts[cur].push_back(bt);
                }
                piece_pcs[cur].push_back(pc[i + 1]);
                piece_pts[cur].push_back(tb);
            }
            // paves outside this coedge's range: keep what we have. LOCAL
            // copy — mutating the shared npieces would corrupt the piece
            // mapping of the edge's OTHER coedges (seam mate).
            int nce_pieces = npieces;
            if (cur != npieces - 1) nce_pieces = cur + 1;
            // map each chain piece to its edge id BY T-RANGE (the walk order of
            // a descending coedge is reversed relative to the ascending bounds)
            auto piece_edge = [&](int k) -> int {
                double lo = std::min(piece_pts[k].front(), piece_pts[k].back());
                double hi = std::max(piece_pts[k].front(), piece_pts[k].back());
                for (int p = 0; p < npieces; p++)
                    if (std::abs(bounds[p] - lo) < dtol * 10 &&
                        std::abs(bounds[p + 1] - hi) < dtol * 10)
                        return piece_ids[p];
                return piece_ids[0];
            };
            // replace ce by the first piece, append the rest
            ce.pc = piece_pcs[0];
            ce.pt = piece_pts[0];
            ce.edge = piece_edge(0);
            ce.fwd = fwd;
            std::vector<int> new_ce_ids;
            for (int p = 1; p < nce_pieces; p++) {
                CoEdge nce;
                nce.edge = piece_edge(p);
                nce.fwd = fwd;
                nce.face = fidx;
                nce.pc = piece_pcs[p];
                nce.pt = piece_pts[p];
                new_ce_ids.push_back((int)S.coedges.size());
                S.coedges.push_back(nce);
            }
            // insert new coedges into the loop right after cei
            for (auto& f : S.faces) {
                for (auto& l : f.loops) {
                    auto it = std::find(l.ces.begin(), l.ces.end(), cei);
                    if (it != l.ces.end()) {
                        l.ces.insert(it + 1, new_ce_ids.begin(), new_ce_ids.end());
                        goto next_face;
                    }
                }
            }
        next_face:;
        }
    }
    return pieces_out;
}

// ============================================================================
// UV arrangement (split_face)
// ============================================================================

namespace {

struct ANode {
    double u = 0, v = 0;
    int vtx = -1; // solid vertex id (-1: intersection point without vertex)
};
struct APiece {
    int n0 = -1, n1 = -1;
    std::vector<UV2> pc; // from n0 to n1
    int edge = -1;       // solid edge id
    int sec = -1;        // 1 when this piece is a section piece
    bool dead = false;
};

struct Arrangement {
    Solid& S;
    double tol3d;
    double uvtol;
    std::vector<ANode> nodes;
    std::vector<APiece> pieces;

    Arrangement(Solid& s, double t) : S(s), tol3d(t) {
        // UV tolerance from the face metric, set in split_face; default small
        uvtol = 1e-9;
    }

    int node_at(double u, double v, int vtx) {
        // merge by CHART POSITION (UV proximity). A 3D vertex appearing at
        // several chart positions (periodic faces: u and u+2pi) must become
        // DISTINCT nodes — merging them collapses seam trims to diagonals.
        for (size_t i = 0; i < nodes.size(); i++) {
            double du = nodes[i].u - u, dv = nodes[i].v - v;
            if (du * du + dv * dv < uvtol * uvtol) {
                if (vtx >= 0 && nodes[i].vtx < 0) nodes[i].vtx = vtx;
                return (int)i;
            }
        }
        nodes.push_back({u, v, vtx});
        return (int)nodes.size() - 1;
    }

    // segment intersection: proper crossing of (a1,a2) with (b1,b2)
    static bool seg_x(const UV2& a1, const UV2& a2, const UV2& b1, const UV2& b2,
                      double& s, double& t) {
        double d1x = a2.u - a1.u, d1y = a2.v - a1.v;
        double d2x = b2.u - b1.u, d2y = b2.v - b1.v;
        // zero-length segments carry no direction — never a proper crossing
        // (they show up as marched-section spurs; without the guard the
        // worklist loop can spin on them)
        if (d1x * d1x + d1y * d1y < 1e-24 || d2x * d2x + d2y * d2y < 1e-24)
            return false;
        double den = d1x * d2y - d1y * d2x;
        if (std::abs(den) < 1e-300) return false;
        double ex = b1.u - a1.u, ey = b1.v - a1.v;
        s = (ex * d2y - ey * d2x) / den;
        t = (ex * d1y - ey * d1x) / den;
        return s > 1e-9 && s < 1 - 1e-9 && t > 1e-9 && t < 1 - 1e-9;
    }

    // split piece `pi` at sample index si with fraction fr; returns new piece id
    int split_piece(int pi, size_t si, double fr, const UV2& x) {
        APiece& p = pieces[pi];
        APiece np;
        np.edge = p.edge;
        np.sec = p.sec;
        np.n0 = node_at(x.u, x.v, -1);
        int mid = np.n0;
        np.n1 = p.n1;
        np.pc.assign(p.pc.begin() + si + 1, p.pc.end());
        np.pc.insert(np.pc.begin(), x);
        p.n1 = mid;
        p.pc.erase(p.pc.begin() + si + 1, p.pc.end());
        p.pc.push_back(x);
        pieces.push_back(np);
        return (int)pieces.size() - 1;
    }

    // true when the crossing point coincides with an existing node (a
    // T-junction at a pave, not a proper crossing — must not split there)
    bool near_node(const UV2& x) const {
        double tol = uvtol * 50;
        for (auto& n : nodes) {
            double du = n.u - x.u, dv = n.v - x.v;
            if (du * du + dv * dv < tol * tol) return true;
        }
        return false;
    }

    void intersect_all() {
        // brute force with worklist: find one crossing, split, repeat
        std::vector<std::array<double, 4>> bb; // per-piece UV bbox
        for (int rounds = 0; rounds < 200; rounds++) {
            bool found = false;
            if (std::getenv("V3XDBG"))
                std::fprintf(stderr, "[xall] round %d pieces=%zu nodes=%zu\n",
                             rounds, pieces.size(), nodes.size());
            // refresh piece bboxes (splits mutate the pool); walked-section
            // polylines carry thousands of samples — bbox rejection keeps the
            // pair scan out of O(segs^2) territory
            bb.assign(pieces.size(), {0, 0, 0, 0});
            for (size_t i = 0; i < pieces.size(); i++) {
                if (pieces[i].dead || pieces[i].pc.size() < 2) continue;
                double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
                for (auto& q : pieces[i].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
                bb[i] = {u0, v0, u1, v1};
            }
            double mgn = uvtol * 50;
            for (size_t a = 0; a < pieces.size() && !found; a++) {
                if (pieces[a].dead || pieces[a].pc.size() < 2) continue;
                for (size_t b = a + 1; b < pieces.size() && !found; b++) {
                    if (pieces[b].dead || pieces[b].pc.size() < 2) continue;
                    // skip pieces of the same source edge
                    if (pieces[a].edge == pieces[b].edge) continue;
                    if (bb[a][0] > bb[b][2] + mgn || bb[b][0] > bb[a][2] + mgn ||
                        bb[a][1] > bb[b][3] + mgn || bb[b][1] > bb[a][3] + mgn)
                        continue;
                    for (size_t i = 0; i + 1 < pieces[a].pc.size() && !found; i++) {
                        for (size_t j = 0; j + 1 < pieces[b].pc.size() && !found; j++) {
                            double s, t;
                            if (!seg_x(pieces[a].pc[i], pieces[a].pc[i + 1],
                                       pieces[b].pc[j], pieces[b].pc[j + 1], s, t))
                                continue;
                            UV2 x{pieces[a].pc[i].u +
                                      (pieces[a].pc[i + 1].u - pieces[a].pc[i].u) * s,
                                  pieces[a].pc[i].v +
                                      (pieces[a].pc[i + 1].v - pieces[a].pc[i].v) * s};
                            if (near_node(x)) continue; // T-junction at a pave
                            int na = split_piece((int)a, i, s, x);
                            int nb = split_piece((int)b, j, t, x);
                            // unify the two mid nodes
                            int keep = pieces[na].n0, drop = pieces[nb].n0;
                            if (keep != drop) {
                                for (auto& p : pieces) {
                                    if (p.n0 == drop) p.n0 = keep;
                                    if (p.n1 == drop) p.n1 = keep;
                                }
                                nodes[keep].vtx = -1;
                            }
                            found = true;
                        }
                    }
                }
            }
            if (!found) break;
        }
    }

    // half-edge traversal: minimal cycles with interior on the left
    std::vector<std::vector<int>> cycles() {
        // outgoing pieces per node: (piece id, dir) dir=1 means n0->n1
        std::map<int, std::vector<std::pair<int, int>>> out;
        for (size_t i = 0; i < pieces.size(); i++) {
            if (pieces[i].dead || pieces[i].pc.size() < 2) continue;
            out[pieces[i].n0].push_back({(int)i, +1});
            out[pieces[i].n1].push_back({(int)i, -1});
        }
        // sort by angle at each node
        auto angle_of = [&](int pi, int dir) {
            const APiece& p = pieces[pi];
            if (dir > 0) {
                const UV2& a = p.pc[0];
                const UV2& b = p.pc[1];
                return std::atan2(b.v - a.v, b.u - a.u);
            } else {
                // outgoing direction at the n1 end of the backward half-edge:
                // from pc.back() toward pc[size-2] (the reversed first segment)
                const UV2& a = p.pc[p.pc.size() - 2];
                const UV2& b = p.pc.back();
                return std::atan2(a.v - b.v, a.u - b.u);
            }
        };
        for (auto& kv : out) {
            std::sort(kv.second.begin(), kv.second.end(),
                      [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                          return angle_of(a.first, a.second) < angle_of(b.first, b.second);
                      });
        }
        std::set<std::pair<int, int>> used;
        std::vector<std::vector<int>> out_cycles;
        for (size_t i = 0; i < pieces.size(); i++) {
            if (pieces[i].dead || pieces[i].pc.size() < 2) continue;
            for (int dir0 : {+1, -1}) {
                std::pair<int, int> start{(int)i, dir0};
                if (used.count(start)) continue;
                std::vector<int> cyc;
                int pi = (int)i, dir = dir0;
                int guard = 0;
                int start_node = dir > 0 ? pieces[i].n0 : pieces[i].n1;
                int cur_node = dir > 0 ? pieces[i].n1 : pieces[i].n0;
                cyc.push_back(pi * 2 + (dir > 0 ? 0 : 1));
                used.insert({pi, dir});
                bool closed = false;
                // self-loop piece (n0 == n1): closes immediately
                if (cur_node == start_node) closed = true;
                while (!closed && guard++ < 10000) {
                    if (cur_node == start_node && cyc.size() > 1) {
                        closed = true;
                        break;
                    }
                    // at cur_node: take the half-edge BEFORE the reverse of the
                    // incoming edge in CCW order (first in CW order = the
                    // face-extraction rule that keeps the face on the left)
                    auto& outs = out[cur_node];
                    if (outs.empty()) break;
                    std::pair<int, int> rev{pi, -dir};
                    size_t pos = 0;
                    for (; pos < outs.size(); pos++)
                        if (outs[pos] == rev) break;
                    if (pos == outs.size()) break;
                    bool picked = false;
                    for (size_t k = 1; k <= outs.size(); k++) {
                        auto cand = outs[(pos + outs.size() - k) % outs.size()];
                        if (cand == rev) continue;
                        if (used.count(cand)) continue;
                        pi = cand.first;
                        dir = cand.second;
                        picked = true;
                        break;
                    }
                    // dead-end node (dangling spur, e.g. a walked section that
                    // stops at a tangency interior to the face): turn back
                    // along the same piece — the spur rides out and back within
                    // the zone loop (zero net area, edge used once per side)
                    if (!picked && outs.size() == 1 && !used.count(rev)) {
                        pi = rev.first;
                        dir = rev.second;
                        picked = true;
                    }
                    if (!picked) break;
                    cyc.push_back(pi * 2 + (dir > 0 ? 0 : 1));
                    used.insert({pi, dir});
                    if (std::getenv("V3TRAVDBG"))
                        std::fprintf(stderr, "    [trav] node=%d pick edge=%d dir=%d\n",
                                     cur_node, pieces[pi].edge, dir);
                    cur_node = dir > 0 ? pieces[pi].n1 : pieces[pi].n0;
                }
                if (closed && !cyc.empty() &&
                    (cyc.size() >= 2 || pieces[cyc[0] / 2].n0 == pieces[cyc[0] / 2].n1))
                    out_cycles.push_back(cyc);
            }
        }
        return out_cycles;
    }
};

double cycle_area(const Arrangement& A, const std::vector<int>& cyc) {
    double a = 0;
    for (int h : cyc) {
        const APiece& p = A.pieces[h / 2];
        bool fwd = (h % 2) == 0;
        const auto& pc = p.pc;
        for (size_t i = 0; i + 1 < pc.size(); i++) {
            const UV2& p0 = fwd ? pc[i] : pc[pc.size() - 1 - i];
            const UV2& p1 = fwd ? pc[i + 1] : pc[pc.size() - 2 - i];
            a += p0.u * p1.v - p1.u * p0.v;
        }
    }
    return 0.5 * a;
}

bool point_in_cycle(const Arrangement& A, const std::vector<int>& cyc, double u,
                    double v) {
    int crossings = 0;
    for (int h : cyc) {
        const APiece& p = A.pieces[h / 2];
        const auto& pc = p.pc;
        for (size_t i = 0; i + 1 < pc.size(); i++) {
            const UV2& a = pc[i];
            const UV2& b = pc[i + 1];
            if ((a.v > v) != (b.v > v)) {
                double uc = a.u + (v - a.v) * (b.u - a.u) / (b.v - a.v);
                if (uc > u) crossings++;
            }
        }
    }
    return (crossings & 1) != 0;
}

} // namespace

std::vector<Face> split_face(Solid& S, const Face& F,
                             const std::vector<SecPC>& secs, double tol3d) {
    Arrangement A(S, tol3d);
    // UV tolerance from the face metric: sample the surface for scale
    {
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Loop& l : F.loops)
            for (int cei : l.ces)
                for (auto& q : S.coedges[cei].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
        double span3d = 0;
        const Srf& srf = S.srfs[F.srf];
        for (int i = 0; i < 4; i++) {
            V3 p0 = srf.eval(u0 + (u1 - u0) * i / 3.0, 0.5 * (v0 + v1));
            V3 p1 = srf.eval(u0 + (u1 - u0) * (i + 1) / 3.0, 0.5 * (v0 + v1));
            span3d += p0.dist(p1);
        }
        double uspan = std::max(u1 - u0, 1e-9);
        A.uvtol = (uspan / std::max(span3d, 1e-9)) * tol3d * 4 + 1e-12;
    }

    // boundary pieces
    std::map<int, int> ce_piece;
    for (const Loop& l : F.loops) {
        // Loop-consecutive coedges share a corner by construction; chain the
        // node explicitly instead of relying on chart-position merging:
        // pave-split/lerp endpoint drift (~1e-6) can exceed uvtol and split a
        // corner into two nodes, which kills the boundary walk (z90 B15).
        int prev_node = -1, first_pi = -1;
        for (int cei : l.ces) {
            const CoEdge& ce = S.coedges[cei];
            if (ce.pc.size() < 2) continue;
            APiece p;
            p.edge = ce.edge;
            p.sec = 0;
            const Edge& e = S.edges[ce.edge];
            int v_first = ce.fwd ? e.v0 : e.v1;
            int v_last = ce.fwd ? e.v1 : e.v0;
            p.n0 = prev_node >= 0
                       ? prev_node
                       : A.node_at(ce.pc.front().u, ce.pc.front().v, v_first);
            p.n1 = A.node_at(ce.pc.back().u, ce.pc.back().v, v_last);
            p.pc = ce.pc;
            ce_piece[cei] = (int)A.pieces.size();
            if (first_pi < 0) first_pi = (int)A.pieces.size();
            prev_node = p.n1;
            A.pieces.push_back(p);
        }
        // wrap corner: tie the loop's last junction back to its first node
        if (prev_node >= 0 && first_pi >= 0 &&
            A.pieces[first_pi].n0 != prev_node)
            A.pieces[first_pi].n0 = prev_node;
    }
    // section pieces
    for (const SecPC& sp : secs) {
        if (sp.pc.size() < 2) continue;
        if (std::getenv("V3SPLITDBG"))
            std::fprintf(stderr,
                         "  [secpc] edge=%d closed=%d first=(%.6f,%.4f) last=(%.6f,%.4f) n=%zu ev0=%d ev1=%d\n",
                         sp.sec_edge, (int)sp.closed, sp.pc.front().u,
                         sp.pc.front().v, sp.pc.back().u, sp.pc.back().v,
                         sp.pc.size(), sp.end_vtx[0], sp.end_vtx[1]);
        APiece p;
        p.edge = sp.sec_edge;
        p.sec = 1;
        p.n0 = A.node_at(sp.pc.front().u, sp.pc.front().v, sp.end_vtx[0]);
        p.n1 = A.node_at(sp.pc.back().u, sp.pc.back().v, sp.end_vtx[1]);
        p.pc = sp.pc;
        A.pieces.push_back(p);
        (void)sp.closed;
    }
    A.intersect_all();
    auto cycs = A.cycles();
    if (std::getenv("V3SPLITDBG")) {
        std::fprintf(stderr, "[split] face srf=%d: pieces=%zu nodes=%zu cycles=%zu secs=%zu\n",
                     F.srf, A.pieces.size(), A.nodes.size(), cycs.size(), secs.size());
        if (secs.size() > 0) {
            for (size_t pi = 0; pi < A.pieces.size(); pi++)
                std::fprintf(stderr,
                             "  piece %zu: edge=%d sec=%d n0=%d n1=%d nseg=%zu first=(%.4f,%.4f) last=(%.4f,%.4f)\n",
                             pi, A.pieces[pi].edge, A.pieces[pi].sec, A.pieces[pi].n0,
                             A.pieces[pi].n1, A.pieces[pi].pc.size(),
                             A.pieces[pi].pc.front().u, A.pieces[pi].pc.front().v,
                             A.pieces[pi].pc.back().u, A.pieces[pi].pc.back().v);
            for (size_t ni = 0; ni < A.nodes.size(); ni++)
                std::fprintf(stderr, "  node %zu: (%.9f,%.9f) vtx=%d uvtol=%.3g\n", ni,
                             A.nodes[ni].u, A.nodes[ni].v, A.nodes[ni].vtx, A.uvtol);
            for (size_t ci = 0; ci < cycs.size(); ci++) {
                std::fprintf(stderr, "  cycle %zu: size=%zu area=%.6f edges:", ci, cycs[ci].size(),
                             cycle_area(A, cycs[ci]));
                for (int h : cycs[ci]) {
                    const APiece& p = A.pieces[h / 2];
                    std::fprintf(stderr, " %d%c", p.edge, (h % 2) ? 'r' : 'f');
                }
                std::fprintf(stderr, "\n");
            }
        }
    }
    if (cycs.empty()) {
        // no usable arrangement: return the face unchanged
        return {F};
    }

    // cycle keep: interior point inside the original face region
    struct KC {
        std::vector<int> cyc;
        double area;
        double tx = 0, ty = 0; // interior test point
    };
    std::vector<KC> kept;
    for (auto& c : cycs) {
        double a = cycle_area(A, c);
        if (std::abs(a) < A.uvtol * A.uvtol * 0.25) continue;
        // interior test point: midpoint of the longest piece, nudged left/right
        int best_h = 0;
        double best_l = -1;
        for (int h : c) {
            const APiece& p = A.pieces[h / 2];
            for (size_t i = 0; i + 1 < p.pc.size(); i++) {
                double L = std::hypot(p.pc[i + 1].u - p.pc[i].u,
                                      p.pc[i + 1].v - p.pc[i].v);
                if (L > best_l) { best_l = L; best_h = h; }
            }
        }
        const APiece& p = A.pieces[best_h / 2];
        bool fwd = (best_h % 2) == 0;
        size_t si = p.pc.size() / 2 - 1;
        if (si + 1 >= p.pc.size()) si = p.pc.size() - 2;
        UV2 p0 = fwd ? p.pc[si] : p.pc[si + 1];
        UV2 p1 = fwd ? p.pc[si + 1] : p.pc[si];
        double dx = p1.u - p0.u, dy = p1.v - p0.v;
        double L = std::hypot(dx, dy) + 1e-300;
        // leftmost traversal keeps the cycle interior on the LEFT of the walk
        double eps = A.uvtol * 16;
        double mx = 0.5 * (p0.u + p1.u), my = 0.5 * (p0.v + p1.v);
        double tx = mx - dy / L * eps, ty = my + dx / L * eps;
        bool in = uv_in_face(S, F, tx, ty);
        if (std::getenv("V3SPLITDBG"))
            std::fprintf(stderr,
                         "[keep] area=%.4f bestpiece=%d fwd=%d dir=(%.3f,%.3f) test=(%.4f,%.4f) in=%d\n",
                         a, best_h / 2, (int)fwd, dx, dy, tx, ty, (int)in);
        if (in) kept.push_back({c, a, tx, ty});
    }
    if (std::getenv("V3SPLITDBG"))
        std::fprintf(stderr, "[split] kept=%zu\n", kept.size());
    if (kept.empty()) return {F};

    // containment tree: parent(C) = smallest-area kept cycle strictly
    // containing C's test point. own(C) = |C| - sum of opposite-sign children.
    // Faces = cycles with own != 0; holes = opposite-sign children.
    int nk = (int)kept.size();
    std::vector<int> parent(nk, -1);
    double zone_eps = A.uvtol * A.uvtol * 4; // same-zone area tie scale
    for (int i = 0; i < nk; i++) {
        double best_a = 1e300;
        for (int j = 0; j < nk; j++) {
            if (i == j || std::abs(kept[j].area) <= std::abs(kept[i].area)) continue;
            // the CW copy of the same zone is never its container (areas are
            // fp-equal; without the guard a hole's parent can become its own
            // reversed copy, orphaning the hole from the real zone face)
            if (kept[j].area * kept[i].area < 0 &&
                std::abs(kept[j].area) - std::abs(kept[i].area) < zone_eps)
                continue;
            if (!point_in_cycle(A, kept[j].cyc, kept[i].tx, kept[i].ty)) continue;
            double aj = std::abs(kept[j].area);
            // on area near-ties prefer the positive cycle: a frame and its CW
            // copy cover the same region, but zones belong to the positive one
            bool better = aj < best_a - zone_eps;
            if (!better && aj < best_a + zone_eps && parent[i] >= 0 &&
                kept[j].area > 0 && kept[parent[i]].area < 0)
                better = true;
            if (better) {
                best_a = aj;
                parent[i] = j;
            }
        }
    }
    std::vector<double> own(nk, 0);
    // A negative cycle that is the CW copy of a positive zone holes out that
    // zone's PARENT — never the zone itself or a sibling. Seam-split islands
    // need this: the CW copy's keep-test point lands across the seam in the
    // SIBLING zone, so the raw smallest-container rule mis-parents it and the
    // remainder face loses its hole loop entirely.
    for (int j = 0; j < nk; j++) {
        if (kept[j].area >= 0) continue;
        for (int z = 0; z < nk; z++) {
            if (kept[z].area <= 0 || parent[z] < 0) continue;
            if (std::abs(std::abs(kept[z].area) - std::abs(kept[j].area)) >=
                zone_eps)
                continue;
            // same region? the positive zone's test point must lie inside
            if (!point_in_cycle(A, kept[j].cyc, kept[z].tx, kept[z].ty))
                continue;
            parent[j] = parent[z];
            break;
        }
    }
    for (int i = 0; i < nk; i++) own[i] = std::abs(kept[i].area);
    for (int i = 0; i < nk; i++)
        if (parent[i] >= 0 &&
            (kept[i].area > 0) != (kept[parent[i]].area > 0))
            own[parent[i]] -= std::abs(kept[i].area);
    double area_tol = A.uvtol * A.uvtol * 4;
    if (std::getenv("V3SPLITDBG"))
        for (int i = 0; i < nk; i++)
            std::fprintf(stderr, "[cont] cycle %d area=%.4f parent=%d own=%.4f\n", i,
                         kept[i].area, parent[i], own[i]);
    std::vector<Face> out;
    auto add_loop = [&](Face& nf, const std::vector<int>& cyc, bool outer) {
        Loop l;
        l.outer = outer;
        for (int h : cyc) {
            const APiece& p = A.pieces[h / 2];
            bool fwd = (h % 2) == 0;
            CoEdge ce;
            ce.edge = p.edge;
            ce.fwd = fwd;
            ce.face = (int)S.faces.size() + (int)out.size();
            ce.pc = fwd ? p.pc : std::vector<UV2>(p.pc.rbegin(), p.pc.rend());
            l.ces.push_back((int)S.coedges.size());
            S.coedges.push_back(ce);
        }
        nf.loops.push_back(l);
    };
    for (int i = 0; i < nk; i++) {
        // Zone-outer walks are always CCW (positive area): interior is kept on
        // the LEFT of the leftmost walk. Negative cycles are hole boundaries
        // (or the exterior frame) -- never faces themselves; they are consumed
        // as holes of their nearest positive ancestor. Emitting a negative
        // cycle on its own duplicates the zone it holes out (cone-base disk
        // showed up twice).
        if (kept[i].area < area_tol) continue;
        Face nf;
        nf.srf = F.srf;
        nf.rev = F.rev;
        nf.src = F.src;
        add_loop(nf, kept[i].cyc, true);
        for (int j = 0; j < nk; j++) {
            if (kept[j].area > 0) continue; // holes are negative cycles
            int pa = parent[j];
            while (pa >= 0 && kept[pa].area < 0) pa = parent[pa];
            if (pa == i) add_loop(nf, kept[j].cyc, false);
        }
        out.push_back(nf);
    }
    if (out.empty()) return {F};
    return out;
}

// ============================================================================
// section clipping
// ============================================================================

std::vector<SecRun> section_runs(const Solid& SA, const Face& FA,
                                 const Solid& SB, const Face& FB,
                                 const std::vector<SecPoint>& pts, double tol3d) {
    (void)tol3d;
    std::vector<SecRun> runs;
    int n = (int)pts.size();
    int i = 0;
    auto inside = [&](int k) {
        return uv_in_face(SA, FA, pts[k].u1, pts[k].v1) &&
               uv_in_face(SB, FB, pts[k].u2, pts[k].v2);
    };
    while (i < n) {
        if (!inside(i)) { i++; continue; }
        int j = i;
        while (j + 1 < n && inside(j + 1)) j++;
        if (j > i || n == 1) runs.push_back({i, j});
        i = j + 1;
    }
    return runs;
}

SecPoint refine_boundary(const Solid& SA, const Face& FA, const Solid& SB,
                         const Face& FB, const SecPoint& a, const SecPoint& b,
                         bool has_exact, const Cur& c, int which, double tol3d) {
    (void)which;
    (void)tol3d;
    if (has_exact) {
        // bisect on the CURVE PARAMETER: UVs and 3D stay consistent (the lerp
        // of sample UVs vs exact eval(t) is the endpoint-error generator)
        auto inside_at = [&](double t) {
            V3 p = c.eval(t);
            double u1, v1, u2, v2;
            SA.srfs[FA.srf].uv_of(p, u1, v1);
            SB.srfs[FB.srf].uv_of(p, u2, v2);
            return uv_in_face(SA, FA, u1, v1) && uv_in_face(SB, FB, u2, v2);
        };
        double lo = a.t, hi = b.t;
        bool lo_in = inside_at(lo);
        for (int it = 0; it < 40; it++) {
            double mid = 0.5 * (lo + hi);
            if (inside_at(mid) == lo_in) lo = mid;
            else hi = mid;
        }
        double tm = 0.5 * (lo + hi);
        SecPoint x;
        x.t = tm;
        x.p = c.eval(tm);
        SA.srfs[FA.srf].uv_of(x.p, x.u1, x.v1);
        SB.srfs[FB.srf].uv_of(x.p, x.u2, x.v2);
        return x;
    }
    auto inside_at = [&](double fr) {
        double u1 = a.u1 + (b.u1 - a.u1) * fr, v1 = a.v1 + (b.v1 - a.v1) * fr;
        double u2 = a.u2 + (b.u2 - a.u2) * fr, v2 = a.v2 + (b.v2 - a.v2) * fr;
        return uv_in_face(SA, FA, u1, v1) && uv_in_face(SB, FB, u2, v2);
    };
    double lo = 0, hi = 1;
    for (int it = 0; it < 30; it++) {
        double mid = 0.5 * (lo + hi);
        if (inside_at(mid)) lo = mid;
        else hi = mid;
    }
    double fr = 0.5 * (lo + hi);
    SecPoint x;
    x.u1 = a.u1 + (b.u1 - a.u1) * fr;
    x.v1 = a.v1 + (b.v1 - a.v1) * fr;
    x.u2 = a.u2 + (b.u2 - a.u2) * fr;
    x.v2 = a.v2 + (b.v2 - a.v2) * fr;
    x.t = a.t + (b.t - a.t) * fr;
    x.p = a.p + (b.p - a.p) * fr;
    return x;
}

std::vector<SecRun> section_runs_refined(const Solid& SA, const Face& FA,
                                         const Solid& SB, const Face& FB,
                                         const SecCurve& sc,
                                         std::vector<SecPoint>& pts_out,
                                         double tol3d) {
    auto inside = [&](const SecPoint& p) {
        return uv_in_face(SA, FA, p.u1, p.v1) &&
               uv_in_face(SB, FB, p.u2, p.v2);
    };
    Cur dummy = cur_line({0, 0, 0}, {1, 0, 0});
    const Cur& c = sc.has_exact ? sc.exact : dummy;
    std::vector<SecRun> runs;
    int n = (int)sc.pts.size();
    int i = 0;
    while (i < n) {
        if (!inside(sc.pts[i])) { i++; continue; }
        if (i > 0)
            pts_out.push_back(refine_boundary(SA, FA, SB, FB, sc.pts[i - 1],
                                              sc.pts[i], sc.has_exact, c, 0, tol3d));
        int start = (int)pts_out.size();
        while (i < n && inside(sc.pts[i])) pts_out.push_back(sc.pts[i++]);
        int end = (int)pts_out.size() - 1;
        if (i < n)
            pts_out.push_back(refine_boundary(SA, FA, SB, FB, sc.pts[i - 1],
                                              sc.pts[i], sc.has_exact, c, 1, tol3d)),
            end++;
        if (end >= start) runs.push_back({start, end});
    }
    return runs;
}

} // namespace v3
