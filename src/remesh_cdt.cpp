#include "remesh_cdt.h"
#include "session_config.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace session_cpp {
namespace {

// ═══════════════════════════════════════════════════════════════════════════
// Integer geometry
// ═══════════════════════════════════════════════════════════════════════════

constexpr size_t NULL_IDX = static_cast<size_t>(-1);
constexpr double MAX_COORD64 = 9e17;
constexpr int MAX_PRECISION = 6;

/// Hash of an integer point.
struct Point64Hash {
    size_t operator()(const std::array<int64_t, 2>& p) const noexcept {

        const size_t h = std::hash<int64_t>()(p[0]);

        return h ^ (std::hash<int64_t>()(p[1]) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
    }
};

/// Round to the nearest int64.
int64_t to_int64(double x) {
    return static_cast<int64_t>(std::round(x));
}

/// Scale a 2D point to integer coordinates.
std::array<int64_t, 2> to_point64(const std::pair<double, double>& p, double scale) {
    return {to_int64(p.first * scale), to_int64(p.second * scale)};
}

/// Sign of the turn p1 -> p2 -> p3.
int cross_sign(const std::array<int64_t, 2>& p1, const std::array<int64_t, 2>& p2, const std::array<int64_t, 2>& p3) {

    const double cp = static_cast<double>(p2[0] - p1[0]) * static_cast<double>(p3[1] - p2[1]) -
        static_cast<double>(p2[1] - p1[1]) * static_cast<double>(p3[0] - p2[0]);

    if (cp > 0)
        return 1;

    if (cp < 0)
        return -1;

    return 0;
}

/// True when p1 -> p2 -> p3 turns left.
bool left_turning(const std::array<int64_t, 2>& p1, const std::array<int64_t, 2>& p2, const std::array<int64_t, 2>& p3) {
    return cross_sign(p1, p2, p3) < 0;
}

/// True when p1 -> p2 -> p3 turns right.
bool right_turning(const std::array<int64_t, 2>& p1, const std::array<int64_t, 2>& p2, const std::array<int64_t, 2>& p3) {
    return cross_sign(p1, p2, p3) > 0;
}

/// True when a is swept before b: higher y first, then lower x.
bool sweep_before(const std::array<int64_t, 2>& a, const std::array<int64_t, 2>& b) {

    if (a[1] == b[1])
        return a[0] < b[0];

    return a[1] > b[1];
}

/// Squared distance between two integer points.
double dist_sqr(const std::array<int64_t, 2>& a, const std::array<int64_t, 2>& b) {

    const double dx = static_cast<double>(a[0] - b[0]);
    const double dy = static_cast<double>(a[1] - b[1]);

    return dx * dx + dy * dy;
}

/// Positive when d lies inside the circumcircle of the counter-clockwise triangle a, b, c.
double in_circle(
    const std::array<int64_t, 2>& a,
    const std::array<int64_t, 2>& b,
    const std::array<int64_t, 2>& c,
    const std::array<int64_t, 2>& d
) {

    const double m00 = static_cast<double>(a[0] - d[0]);
    const double m01 = static_cast<double>(a[1] - d[1]);
    const double m02 = m00 * m00 + m01 * m01;
    const double m10 = static_cast<double>(b[0] - d[0]);
    const double m11 = static_cast<double>(b[1] - d[1]);
    const double m12 = m10 * m10 + m11 * m11;
    const double m20 = static_cast<double>(c[0] - d[0]);
    const double m21 = static_cast<double>(c[1] - d[1]);
    const double m22 = m20 * m20 + m21 * m21;

    return m00 * (m11 * m22 - m21 * m12) - m10 * (m01 * m22 - m21 * m02) + m20 * (m01 * m12 - m11 * m02);
}

/// Squared distance from p to the segment a-b.
double dist_sqr_segment(const std::array<int64_t, 2>& p, const std::array<int64_t, 2>& a, const std::array<int64_t, 2>& b) {

    const double dx = static_cast<double>(b[0] - a[0]);
    const double dy = static_cast<double>(b[1] - a[1]);
    const double ax = static_cast<double>(p[0] - a[0]);
    const double ay = static_cast<double>(p[1] - a[1]);
    const double q = ax * dx + ay * dy;

    if (q < 0)
        return dist_sqr(p, a);

    if (q > dx * dx + dy * dy)
        return dist_sqr(p, b);

    return (ax * dy - dx * ay) * (ax * dy - dx * ay) / (dx * dx + dy * dy);
}

/// True when a1-a2 and b1-b2 cross strictly inside both segments.
bool segments_intersect(
    const std::array<int64_t, 2>& a1,
    const std::array<int64_t, 2>& a2,
    const std::array<int64_t, 2>& b1,
    const std::array<int64_t, 2>& b2
) {

    if (a1 == b1 || a2 == b1 || a2 == b2 || a1 == b2)
        return false;

    const double dy1 = static_cast<double>(a2[1] - a1[1]);
    const double dx1 = static_cast<double>(a2[0] - a1[0]);
    const double dy2 = static_cast<double>(b2[1] - b1[1]);
    const double dx2 = static_cast<double>(b2[0] - b1[0]);
    const double cp = dy1 * dx2 - dy2 * dx1;

    if (cp == 0)
        return false;

    const double t = static_cast<double>(a1[0] - b1[0]) * dy2 - static_cast<double>(a1[1] - b1[1]) * dx2;

    if (t >= 0 && (cp < 0 || t >= cp))
        return false;

    if (t < 0 && (cp > 0 || t <= cp))
        return false;

    const double u = static_cast<double>(a1[0] - b1[0]) * dy1 - static_cast<double>(a1[1] - b1[1]) * dx1;

    if (u >= 0)
        return cp > 0 && u < cp;

    return cp < 0 && u > cp;
}

/// Even-odd test of an integer point against an integer ring.
bool inside_path64(const std::array<int64_t, 2>& p, const std::vector<std::array<int64_t, 2>>& poly) {

    bool inside = false;
    const size_t n = poly.size();
    size_t j = n - 1;

    for (size_t i = 0; i < n; ++i) {
        if ((poly[i][1] > p[1]) != (poly[j][1] > p[1])) {
            const double x = static_cast<double>(poly[i][0]) +
                static_cast<double>(p[1] - poly[i][1]) * static_cast<double>(poly[j][0] - poly[i][0]) /
                    static_cast<double>(poly[j][1] - poly[i][1]);

            if (static_cast<double>(p[0]) < x)
                inside = !inside;
        }

        j = i;
    }

    return inside;
}

/// Index before i on a ring of n.
size_t prev_index(size_t i, size_t n) {
    return i == 0 ? n - 1 : i - 1;
}

/// Index after i on a ring of n.
size_t next_index(size_t i, size_t n) {
    return (i + 1) % n;
}

/// Advance i to the next vertex that ends a rising run and starts a falling one; false when the path is flat.
bool find_loc_min(const std::vector<std::array<int64_t, 2>>& path, size_t& i) {

    const size_t n = path.size();

    if (n < 3)
        return false;

    const size_t i0 = i;
    size_t k = next_index(i, n);

    while (path[k][1] <= path[i][1]) {
        i = k;
        k = next_index(k, n);

        if (i == i0)
            return false;
    }

    while (path[k][1] >= path[i][1]) {
        i = k;
        k = next_index(k, n);
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Sweep graph
// ═══════════════════════════════════════════════════════════════════════════

/// Boundary side of an edge, or loose for a diagonal.
enum class EdgeKind {
    loose, // Diagonal between two boundary edges.
    ascend, // Boundary edge on the left side.
    descend, // Boundary edge on the right side.
};

/// Sweep vertex with its incident edges.
struct Vertex {
    std::array<int64_t, 2> pt; // Integer position.
    std::vector<size_t> edges; // Edges touching the vertex.
    bool inner_lm = false; // True at a local minimum of a hole.
};

/// Sweep edge with its endpoints, triangles and active-list links.
struct Edge {
    size_t vl = NULL_IDX; // Left vertex.
    size_t vr = NULL_IDX; // Right vertex.
    size_t vb = NULL_IDX; // Bottom vertex.
    size_t vt = NULL_IDX; // Top vertex.
    EdgeKind kind = EdgeKind::loose; // Boundary side or loose diagonal.
    size_t tri_a = NULL_IDX; // First triangle.
    size_t tri_b = NULL_IDX; // Second triangle.
    bool active = false; // True while on the active list.
    size_t next = NULL_IDX; // Next active edge.
    size_t prev = NULL_IDX; // Previous active edge.
};

/// Triangle on three edges.
struct Tri {
    std::array<size_t, 3> edges; // Edge indices.
};

/// Sweep-line constrained Delaunay: boundary edges ascend on the left and descend on the right, diagonals are loose.
class Delaunay {
private:
    std::vector<Vertex> vs; // Vertices.
    std::vector<Edge> es; // Edges.
    std::vector<Tri> ts; // Triangles.
    std::vector<size_t> pending; // Loose edges waiting to be legalized.
    std::vector<size_t> horz; // Horizontal edges deferred from the current row.
    std::vector<size_t> loc_mins; // Hole local minima on the current row.
    size_t lowermost = NULL_IDX; // Lowest vertex of the outer path.
    size_t first_active = NULL_IDX; // Head of the active edge list.

public:

    /// Triangles of the paths, empty when they hold no polygon or a hole cannot be connected.
    std::vector<std::array<std::array<int64_t, 2>, 3>> execute(const std::vector<std::vector<std::array<int64_t, 2>>>& paths);

private:

    /// True when both ends of e share a row.
    bool is_horizontal(size_t e) const;

    /// An edge is done with two triangles, or with one when it is a boundary edge.
    bool completed(size_t e) const;

    /// The endpoint of e that is not v.
    size_t other(size_t e, size_t v) const;

    /// Append a vertex and return its index.
    size_t add_vertex(const std::array<int64_t, 2>& p);

    /// Prepend e to the doubly-linked active list.
    void add_active(size_t e);

    /// Unlink e from the active list and from both endpoint edge lists.
    void remove_active(size_t e);

    /// Drop e from the edge list of v.
    void remove_from_vertex(size_t v, size_t e);

    /// New edge between v1 and v2; loose edges go straight to the active list and the legalize queue.
    size_t create_edge(size_t v1, size_t v2, EdgeKind kind);

    /// New triangle on three edges; an edge leaves the active list when it is completed.
    size_t create_tri(size_t e1, size_t e2, size_t e3);

    /// Shorten long_e to end at short_e's top and continue it with a new edge to the old top.
    void split_edge(size_t long_e, size_t short_e);

    /// Split the longer of two collinear non-horizontal edges leaving v downwards.
    void split_collinear(size_t v);

    /// Merge coincident vertices that are neighbours in sweep order into the first one.
    void merge_duplicates(const std::vector<size_t>& order);

    /// Edge of v1 that reaches v2, a loose one or one of the preferred kind first.
    size_t find_linking_edge(size_t v1, size_t v2, bool prefer_ascend) const;

    /// True when an active horizontal edge lies on the row of v1 between v1 and v2.
    bool horizontal_between(size_t v1, size_t v2) const;

    /// Nearest active edge spanning the x of v_above below it, NULL_IDX when there is none.
    size_t edge_below(size_t v_above) const;

    /// Endpoint of e_below visible from v_above, moved past every active edge crossing the connection.
    size_t visible_vertex(size_t e_below, size_t v_above) const;

    /// Connect a hole local minimum to the visible vertex of the nearest active edge below it.
    size_t create_loc_min_edge(size_t v_above);

    /// Tightest active fan candidate around pivot on the left (or right) side of edge, turns read with the side as sign; NULL_IDX when there is none.
    size_t fan_vertex(size_t edge, size_t pivot, bool left, size_t& e_alt) const;

    /// Fan triangles around pivot on one side of edge, walking onto each new diagonal, never below min_y.
    void triangulate_fan(size_t edge, size_t pivot, int64_t min_y, bool left);

    /// Of the two edges of tri other than edge, a gets the one touching vl and b the other; returns the far vertex.
    size_t opposite(size_t tri, size_t edge, size_t vl, size_t& a, size_t& b) const;

    /// Give tri the edges (edge, e1, e2) and move e1/e2 from the other triangle onto it.
    void rewire(size_t tri, size_t other, size_t edge, size_t e1, size_t e2);

    /// Flip edge when the far vertex of one triangle lies inside the circumcircle of the other.
    void force_legal(size_t edge);

    /// Walk the path from i back round to i0 creating boundary edges; false when the step budget of a degenerate path is blown.
    bool walk_path(const std::vector<std::array<int64_t, 2>>& path, size_t i0, size_t i, size_t v0);

    /// Detach the edges of every vertex added since start.
    void discard(size_t start);

    /// Register one closed path; paths that are flat, degenerate or too tiny to hold a triangle are dropped.
    void add_path(const std::vector<std::array<int64_t, 2>>& path);

    /// Register every path; false when none survives.
    bool add_paths(const std::vector<std::vector<std::array<int64_t, 2>>>& paths);

    /// The outer path was wound clockwise: swap the hole flags and the boundary sides.
    void flip_winding();

    /// Connect and fan the hole local minima collected on the finished row; false when one cannot be reached.
    bool sweep_loc_mins(int64_t curr_y);

    /// Fan the horizontal edges deferred from the finished row.
    void sweep_horizontals(int64_t curr_y);

    /// Activate the boundary edges starting at v and fan the ones ending at it.
    void sweep_vertex(size_t v);

    /// Sweep the vertices top to bottom filling triangles row by row; false when a hole cannot be connected.
    bool sweep(const std::vector<size_t>& order);

    /// Flip loose edges until Delaunay, capped so near-cocircular integer points cannot flip-flop forever.
    void legalize();

    /// Both ends of edge 0 and the far end of edge 1.
    std::array<std::array<int64_t, 2>, 3> tri_points(const Tri& t) const;

    /// Counter-clockwise triangles, flat ones dropped.
    std::vector<std::array<std::array<int64_t, 2>, 3>> triangles() const;
};

bool Delaunay::is_horizontal(size_t e) const {
    return vs[es[e].vb].pt[1] == vs[es[e].vt].pt[1];
}

bool Delaunay::completed(size_t e) const {

    if (es[e].tri_a == NULL_IDX)
        return false;

    if (es[e].tri_b != NULL_IDX)
        return true;

    return es[e].kind != EdgeKind::loose;
}

size_t Delaunay::other(size_t e, size_t v) const {
    return es[e].vb == v ? es[e].vt : es[e].vb;
}

size_t Delaunay::add_vertex(const std::array<int64_t, 2>& p) {

    vs.push_back(Vertex{p, {}, false});

    return vs.size() - 1;
}

void Delaunay::add_active(size_t e) {

    if (es[e].active)
        return;

    es[e].prev = NULL_IDX;
    es[e].next = first_active;
    es[e].active = true;

    if (first_active != NULL_IDX)
        es[first_active].prev = e;

    first_active = e;
}

void Delaunay::remove_active(size_t e) {

    remove_from_vertex(es[e].vb, e);
    remove_from_vertex(es[e].vt, e);

    const size_t prev = es[e].prev;
    const size_t next = es[e].next;

    if (next != NULL_IDX)
        es[next].prev = prev;

    if (prev != NULL_IDX)
        es[prev].next = next;

    es[e].active = false;

    if (first_active == e)
        first_active = next;
}

void Delaunay::remove_from_vertex(size_t v, size_t e) {

    std::vector<size_t>& edges = vs[v].edges;
    const auto it = std::find(edges.begin(), edges.end(), e);

    if (it != edges.end())
        edges.erase(it);
}

size_t Delaunay::create_edge(size_t v1, size_t v2, EdgeKind kind) {

    const size_t e = es.size();
    es.push_back(Edge());

    const std::array<int64_t, 2> p1 = vs[v1].pt;
    const std::array<int64_t, 2> p2 = vs[v2].pt;

    es[e].vb = p1[1] < p2[1] ? v2 : v1;
    es[e].vt = p1[1] < p2[1] ? v1 : v2;
    es[e].vl = p1[0] <= p2[0] ? v1 : v2;
    es[e].vr = p1[0] <= p2[0] ? v2 : v1;
    es[e].kind = kind;

    vs[v1].edges.push_back(e);
    vs[v2].edges.push_back(e);

    if (kind == EdgeKind::loose) {
        pending.push_back(e);
        add_active(e);
    }

    return e;
}

size_t Delaunay::create_tri(size_t e1, size_t e2, size_t e3) {

    const size_t t = ts.size();
    ts.push_back(Tri{{e1, e2, e3}});

    for (size_t e : {e1, e2, e3}) {
        if (es[e].tri_a != NULL_IDX) {
            es[e].tri_b = t;
            remove_active(e);
        } else {
            es[e].tri_a = t;

            if (es[e].kind != EdgeKind::loose)
                remove_active(e);
        }
    }

    return t;
}

void Delaunay::split_edge(size_t long_e, size_t short_e) {

    const size_t old_t = es[long_e].vt;
    const size_t new_t = es[short_e].vt;

    remove_from_vertex(old_t, long_e);
    es[long_e].vt = new_t;

    if (es[long_e].vl == old_t)
        es[long_e].vl = new_t;
    else
        es[long_e].vr = new_t;

    vs[new_t].edges.push_back(long_e);
    create_edge(new_t, old_t, es[long_e].kind);
}

void Delaunay::split_collinear(size_t v) {

    const std::vector<size_t> snapshot = vs[v].edges;

    for (size_t e1 : snapshot) {
        if (is_horizontal(e1) || es[e1].vb != v)
            continue;

        for (size_t e2 : snapshot) {
            if (e2 == e1 || es[e2].vb != v)
                continue;

            const std::array<int64_t, 2> t1 = vs[es[e1].vt].pt;
            const std::array<int64_t, 2> t2 = vs[es[e2].vt].pt;

            if (t1[1] == t2[1] || cross_sign(t1, vs[v].pt, t2) != 0)
                continue;

            if (t1[1] < t2[1])
                split_edge(e1, e2);
            else
                split_edge(e2, e1);

            break;
        }
    }
}

void Delaunay::merge_duplicates(const std::vector<size_t>& order) {

    size_t v1 = order[0];

    for (size_t k = 1; k < order.size(); ++k) {
        const size_t v2 = order[k];

        if (vs[v1].pt != vs[v2].pt) {
            v1 = v2;
            continue;
        }

        if (!vs[v1].inner_lm || !vs[v2].inner_lm)
            vs[v1].inner_lm = false;

        for (size_t e : vs[v2].edges) {
            if (es[e].vb == v2)
                es[e].vb = v1;
            else
                es[e].vt = v1;

            if (es[e].vl == v2)
                es[e].vl = v1;
            else
                es[e].vr = v1;
        }

        vs[v1].edges.insert(vs[v1].edges.end(), vs[v2].edges.begin(), vs[v2].edges.end());
        vs[v2].edges.clear();
        split_collinear(v1);
    }
}

size_t Delaunay::find_linking_edge(size_t v1, size_t v2, bool prefer_ascend) const {

    size_t res = NULL_IDX;

    for (size_t e : vs[v1].edges) {
        if (es[e].vl != v2 && es[e].vr != v2)
            continue;

        if (es[e].kind == EdgeKind::loose || (es[e].kind == EdgeKind::ascend) == prefer_ascend)
            return e;

        res = e;
    }

    return res;
}

bool Delaunay::horizontal_between(size_t v1, size_t v2) const {

    const int64_t y = vs[v1].pt[1];
    const int64_t lo = std::min(vs[v1].pt[0], vs[v2].pt[0]);
    const int64_t hi = std::max(vs[v1].pt[0], vs[v2].pt[0]);
    size_t e = first_active;

    while (e != NULL_IDX) {
        const std::array<int64_t, 2> pl = vs[es[e].vl].pt;
        const std::array<int64_t, 2> pr = vs[es[e].vr].pt;

        if (pl[1] == y && pr[1] == y && pl[0] >= lo && pr[0] <= hi && (pl[0] != lo || pl[0] != hi))
            return true;

        e = es[e].next;
    }

    return false;
}

size_t Delaunay::edge_below(size_t v_above) const {

    const std::array<int64_t, 2> pa = vs[v_above].pt;
    size_t best = NULL_IDX;
    double best_d = -1.0;
    size_t e = first_active;

    while (e != NULL_IDX) {
        const std::array<int64_t, 2> pl = vs[es[e].vl].pt;
        const std::array<int64_t, 2> pr = vs[es[e].vr].pt;
        const bool spans = pl[0] <= pa[0] && pr[0] >= pa[0] && vs[es[e].vb].pt[1] >= pa[1];

        if (spans && es[e].vb != v_above && es[e].vt != v_above && !left_turning(pl, pa, pr)) {
            const double d = dist_sqr_segment(pa, pl, pr);

            if (best == NULL_IDX || d < best_d) {
                best = e;
                best_d = d;
            }
        }

        e = es[e].next;
    }

    return best;
}

size_t Delaunay::visible_vertex(size_t e_below, size_t v_above) const {

    const std::array<int64_t, 2> pa = vs[v_above].pt;
    size_t best = vs[es[e_below].vt].pt[1] <= pa[1] ? es[e_below].vb : es[e_below].vt;
    const bool left = vs[best].pt[0] < pa[0];
    size_t e = first_active;

    while (e != NULL_IDX) {
        const std::array<int64_t, 2> pb = vs[best].pt;
        const std::array<int64_t, 2> pl = vs[es[e].vl].pt;
        const std::array<int64_t, 2> pr = vs[es[e].vr].pt;
        const std::array<int64_t, 2> eb = vs[es[e].vb].pt;
        const std::array<int64_t, 2> et = vs[es[e].vt].pt;
        const bool spans = left ? (pr[0] > pb[0] && pl[0] < pa[0]) : (pr[0] < pb[0] && pl[0] > pa[0]);

        if (spans && eb[1] > pa[1] && et[1] < pb[1] && segments_intersect(eb, et, pb, pa))
            best = et[1] > pa[1] ? es[e].vt : es[e].vb;

        e = es[e].next;
    }

    return best;
}

size_t Delaunay::create_loc_min_edge(size_t v_above) {

    const size_t below = edge_below(v_above);

    if (below == NULL_IDX)
        return NULL_IDX;

    return create_edge(visible_vertex(below, v_above), v_above, EdgeKind::loose);
}

size_t Delaunay::fan_vertex(size_t edge, size_t pivot, bool left, size_t& e_alt) const {

    const size_t v = other(edge, pivot);
    const int side = left ? 1 : -1;
    size_t v_alt = NULL_IDX;
    e_alt = NULL_IDX;

    for (size_t e : vs[pivot].edges) {
        if (e == edge || !es[e].active)
            continue;

        const size_t vx = other(e, pivot);

        if (vx == v)
            continue;

        const int sign = side * cross_sign(vs[v].pt, vs[pivot].pt, vs[vx].pt);

        if (sign == 0) {
            if ((vs[v].pt[0] > vs[pivot].pt[0]) == (vs[pivot].pt[0] > vs[vx].pt[0]))
                continue;
        } else if (sign > 0 || (v_alt != NULL_IDX && side * cross_sign(vs[vx].pt, vs[pivot].pt, vs[v_alt].pt) >= 0)) {
            continue;
        }

        v_alt = vx;
        e_alt = e;
    }

    return v_alt;
}

void Delaunay::triangulate_fan(size_t edge, size_t pivot, int64_t min_y, bool left) {

    const size_t max_fan = 2 * vs.size() + 2;

    for (size_t step = 0; step < max_fan; ++step) {
        size_t e_alt = NULL_IDX;
        const size_t v_alt = fan_vertex(edge, pivot, left, e_alt);

        if (v_alt == NULL_IDX || vs[v_alt].pt[1] < min_y)
            return;

        const EdgeKind kind_below = left ? EdgeKind::ascend : EdgeKind::descend;
        const EdgeKind kind_above = left ? EdgeKind::descend : EdgeKind::ascend;

        if (vs[v_alt].pt[1] < vs[pivot].pt[1] && es[e_alt].kind == kind_below)
            return;

        if (vs[v_alt].pt[1] > vs[pivot].pt[1] && es[e_alt].kind == kind_above)
            return;

        const size_t v = other(edge, pivot);
        const bool prefer_ascend = left ? vs[v_alt].pt[1] < vs[v].pt[1] : vs[v_alt].pt[1] > vs[v].pt[1];
        size_t ex = find_linking_edge(v_alt, v, prefer_ascend);

        if (ex == NULL_IDX) {
            if (vs[v_alt].pt[1] == vs[v].pt[1] && vs[v].pt[1] == min_y && horizontal_between(v_alt, v))
                return;

            ex = create_edge(v_alt, v, EdgeKind::loose);
        }

        if (left)
            create_tri(edge, e_alt, ex);
        else
            create_tri(edge, ex, e_alt);

        if (completed(ex))
            return;

        edge = ex;
        pivot = v_alt;
    }
}

size_t Delaunay::opposite(size_t tri, size_t edge, size_t vl, size_t& a, size_t& b) const {

    size_t far = NULL_IDX;

    for (size_t e : ts[tri].edges) {
        if (e == edge)
            continue;

        if (es[e].vl == vl) {
            a = e;
            far = es[e].vr;
        } else if (es[e].vr == vl) {
            a = e;
            far = es[e].vl;
        } else {
            b = e;
        }
    }

    return far;
}

void Delaunay::rewire(size_t tri, size_t other, size_t edge, size_t e1, size_t e2) {

    ts[tri].edges = {edge, e1, e2};

    for (size_t e : {e1, e2}) {
        if (es[e].kind == EdgeKind::loose)
            pending.push_back(e);

        if (es[e].tri_a == tri || es[e].tri_b == tri)
            continue;

        if (es[e].tri_a == other)
            es[e].tri_a = tri;
        else if (es[e].tri_b == other)
            es[e].tri_b = tri;
    }
}

void Delaunay::force_legal(size_t edge) {

    const size_t ta = es[edge].tri_a;
    const size_t tb = es[edge].tri_b;

    if (ta == NULL_IDX || tb == NULL_IDX)
        return;

    const size_t vl = es[edge].vl;
    const size_t vr = es[edge].vr;
    size_t a1 = NULL_IDX;
    size_t b1 = NULL_IDX;
    size_t a2 = NULL_IDX;
    size_t b2 = NULL_IDX;
    const size_t va = opposite(ta, edge, vl, a1, b1);
    const size_t vb = opposite(tb, edge, vl, a2, b2);

    if (va == NULL_IDX || vb == NULL_IDX || b1 == NULL_IDX || b2 == NULL_IDX)
        return;

    if (cross_sign(vs[va].pt, vs[vl].pt, vs[vr].pt) == 0)
        return;

    const double ict = in_circle(vs[va].pt, vs[vl].pt, vs[vr].pt, vs[vb].pt);

    if (ict == 0 || right_turning(vs[va].pt, vs[vl].pt, vs[vr].pt) == (ict < 0))
        return;

    es[edge].vl = va;
    es[edge].vr = vb;
    rewire(ta, tb, edge, a1, a2);
    rewire(tb, ta, edge, b1, b2);
}

bool Delaunay::walk_path(const std::vector<std::array<int64_t, 2>>& path, size_t i0, size_t i, size_t v0) {

    const size_t n = path.size();
    const size_t budget = 16 * n + 256;
    size_t steps = 0;
    size_t v_prev = v0;

    while (steps < budget) {
        ++steps;

        loc_mins.push_back(v_prev);

        if (lowermost == NULL_IDX || sweep_before(vs[v_prev].pt, vs[lowermost].pt))
            lowermost = v_prev;

        size_t i_next = next_index(i, n);

        if (cross_sign(vs[v_prev].pt, path[i], path[i_next]) == 0) {
            i = i_next;
            continue;
        }

        while (path[i][1] <= vs[v_prev].pt[1]) {
            if (++steps > budget)
                return false;

            const size_t v = add_vertex(path[i]);

            create_edge(v_prev, v, EdgeKind::ascend);
            v_prev = v;
            i = i_next;
            i_next = next_index(i, n);

            while (cross_sign(vs[v_prev].pt, path[i], path[i_next]) == 0) {
                if (++steps > budget)
                    return false;

                i = i_next;
                i_next = next_index(i, n);
            }
        }

        size_t v_prev_prev = v_prev;

        while (i != i0 && path[i][1] >= vs[v_prev].pt[1]) {
            if (++steps > budget)
                return false;

            const size_t v = add_vertex(path[i]);

            create_edge(v, v_prev, EdgeKind::descend);
            v_prev_prev = v_prev;
            v_prev = v;
            i = i_next;
            i_next = next_index(i, n);

            while (cross_sign(vs[v_prev].pt, path[i], path[i_next]) == 0) {
                if (++steps > budget)
                    return false;

                i = i_next;
                i_next = next_index(i, n);
            }
        }

        if (i == i0) {
            create_edge(v0, v_prev, EdgeKind::descend);

            return true;
        }

        if (left_turning(vs[v_prev_prev].pt, vs[v_prev].pt, path[i]))
            vs[v_prev].inner_lm = true;
    }

    return false;
}

void Delaunay::discard(size_t start) {

    for (size_t v = start; v < vs.size(); ++v)
        vs[v].edges.clear();
}

void Delaunay::add_path(const std::vector<std::array<int64_t, 2>>& path) {

    const size_t n = path.size();
    size_t i = 0;

    if (!find_loc_min(path, i))
        return;

    const size_t i0 = i;
    size_t i_prev = prev_index(i, n);

    while (path[i_prev] == path[i])
        i_prev = prev_index(i_prev, n);

    size_t i_next = next_index(i, n);

    while (cross_sign(path[i_prev], path[i], path[i_next]) == 0) {
        if (!find_loc_min(path, i) || i == i0)
            return;

        i_prev = prev_index(i, n);

        while (path[i_prev] == path[i])
            i_prev = prev_index(i_prev, n);

        i_next = next_index(i, n);
    }

    const size_t start = vs.size();
    const size_t v0 = add_vertex(path[i]);

    if (left_turning(path[i_prev], path[i], path[i_next]))
        vs[v0].inner_lm = true;

    if (!walk_path(path, i0, i_next, v0)) {
        discard(start);

        return;
    }

    const size_t count = vs.size() - start;
    const bool tiny = count == 3 &&
        (dist_sqr(vs[start].pt, vs[start + 1].pt) <= 1 || dist_sqr(vs[start + 1].pt, vs[start + 2].pt) <= 1 ||
         dist_sqr(vs[start + 2].pt, vs[start].pt) <= 1);

    if (count < 3 || tiny)
        discard(start);
}

bool Delaunay::add_paths(const std::vector<std::vector<std::array<int64_t, 2>>>& paths) {

    size_t total = 0;

    for (const std::vector<std::array<int64_t, 2>>& path : paths)
        total += path.size();

    if (total == 0)
        return false;

    vs.reserve(total);
    es.reserve(total);

    for (const std::vector<std::array<int64_t, 2>>& path : paths)
        add_path(path);

    return vs.size() > 2;
}

void Delaunay::flip_winding() {

    for (size_t v : loc_mins)
        vs[v].inner_lm = !vs[v].inner_lm;

    for (Edge& e : es)
        if (e.kind == EdgeKind::ascend)
            e.kind = EdgeKind::descend;
        else if (e.kind == EdgeKind::descend)
            e.kind = EdgeKind::ascend;
}

bool Delaunay::sweep_loc_mins(int64_t curr_y) {

    while (!loc_mins.empty()) {
        const size_t lm = loc_mins.back();
        loc_mins.pop_back();

        const size_t e = create_loc_min_edge(lm);

        if (e == NULL_IDX)
            return false;

        const size_t vb = es[e].vb;

        if (is_horizontal(e)) {
            triangulate_fan(e, vb, curr_y, es[e].vl == vb);
        } else {
            triangulate_fan(e, vb, curr_y, true);

            if (!completed(e))
                triangulate_fan(e, vb, curr_y, false);
        }

        if (vs[lm].edges.size() < 2)
            continue;

        add_active(vs[lm].edges[0]);
        add_active(vs[lm].edges[1]);
    }

    return true;
}

void Delaunay::sweep_horizontals(int64_t curr_y) {

    while (!horz.empty()) {
        const size_t e = horz.back();
        horz.pop_back();

        if (completed(e))
            continue;

        if (es[e].vb == es[e].vl) {
            if (es[e].kind == EdgeKind::ascend)
                triangulate_fan(e, es[e].vb, curr_y, true);
        } else if (es[e].kind == EdgeKind::descend) {
            triangulate_fan(e, es[e].vb, curr_y, false);
        }
    }
}

void Delaunay::sweep_vertex(size_t v) {

    for (int i = static_cast<int>(vs[v].edges.size()) - 1; i >= 0; --i) {
        if (i >= static_cast<int>(vs[v].edges.size()))
            continue;

        const size_t e = vs[v].edges[i];

        if (completed(e) || es[e].kind == EdgeKind::loose)
            continue;

        if (is_horizontal(e))
            horz.push_back(e);

        if (v == es[e].vb) {
            if (!vs[v].inner_lm)
                add_active(e);
        } else if (!is_horizontal(e)) {
            triangulate_fan(e, es[e].vb, vs[v].pt[1], es[e].kind == EdgeKind::ascend);
        }
    }
}

bool Delaunay::sweep(const std::vector<size_t>& order) {

    int64_t curr_y = vs[order[0]].pt[1];

    for (size_t v : order) {
        if (vs[v].edges.empty())
            continue;

        if (vs[v].pt[1] != curr_y) {
            if (!sweep_loc_mins(curr_y))
                return false;

            sweep_horizontals(curr_y);
            curr_y = vs[v].pt[1];
        }

        sweep_vertex(v);

        if (vs[v].inner_lm)
            loc_mins.push_back(v);
    }

    while (!horz.empty()) {
        const size_t e = horz.back();
        horz.pop_back();

        if (!completed(e) && es[e].vb == es[e].vl)
            triangulate_fan(e, es[e].vb, curr_y, true);
    }

    return true;
}

void Delaunay::legalize() {

    const size_t max_flips = 64 * vs.size() + 4096;

    for (size_t flips = 0; flips < max_flips && !pending.empty(); ++flips) {
        const size_t e = pending.back();
        pending.pop_back();

        force_legal(e);
    }
}

std::array<std::array<int64_t, 2>, 3> Delaunay::tri_points(const Tri& t) const {

    const Edge& e0 = es[t.edges[0]];
    const Edge& e1 = es[t.edges[1]];
    const std::array<int64_t, 2> p0 = vs[e0.vl].pt;
    const std::array<int64_t, 2> p1 = vs[e0.vr].pt;
    const std::array<int64_t, 2> p2 = vs[e1.vl].pt == p0 || vs[e1.vl].pt == p1 ? vs[e1.vr].pt : vs[e1.vl].pt;

    return {p0, p1, p2};
}

std::vector<std::array<std::array<int64_t, 2>, 3>> Delaunay::triangles() const {

    std::vector<std::array<std::array<int64_t, 2>, 3>> res;
    res.reserve(ts.size());

    for (const Tri& t : ts) {
        std::array<std::array<int64_t, 2>, 3> p = tri_points(t);
        const int sign = cross_sign(p[0], p[1], p[2]);

        if (sign == 0)
            continue;

        if (sign < 0)
            std::swap(p[0], p[2]);

        res.push_back(p);
    }

    return res;
}

std::vector<std::array<std::array<int64_t, 2>, 3>> Delaunay::execute(const std::vector<std::vector<std::array<int64_t, 2>>>& paths) {

    if (!add_paths(paths))
        return {};

    if (vs[lowermost].inner_lm)
        flip_winding();

    loc_mins.clear();

    std::vector<size_t> order(vs.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return sweep_before(vs[a].pt, vs[b].pt);
    });

    merge_duplicates(order);

    if (!sweep(order))
        return {};

    legalize();

    return triangles();
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangulation
// ═══════════════════════════════════════════════════════════════════════════

/// Power of ten keeping the largest coordinate inside int64 headroom.
double cdt_scale(
    const std::vector<std::pair<double, double>>& border_2d,
    const std::vector<std::vector<std::pair<double, double>>>& holes_2d
) {

    double max_coord = 1.0;

    for (const std::pair<double, double>& p : border_2d)
        max_coord = std::max({max_coord, std::abs(p.first), std::abs(p.second)});

    for (const std::vector<std::pair<double, double>>& hole : holes_2d)
        for (const std::pair<double, double>& p : hole)
            max_coord = std::max({max_coord, std::abs(p.first), std::abs(p.second)});

    int precision = MAX_PRECISION;

    while (precision > 0 && max_coord * std::pow(10.0, precision) > MAX_COORD64)
        --precision;

    return std::pow(10.0, precision);
}

/// Hole rows sharing an integer y with a border row move one unit down so the sweep never sees a collinear constraint.
std::vector<std::vector<std::pair<double, double>>> shift_hole_rows(
    const std::vector<std::pair<double, double>>& border_2d,
    const std::vector<std::vector<std::pair<double, double>>>& holes_2d,
    double scale
) {

    std::unordered_set<int64_t> border_ys;

    for (const std::pair<double, double>& p : border_2d)
        border_ys.insert(to_int64(p.second * scale));

    std::vector<std::vector<std::pair<double, double>>> holes = holes_2d;

    for (std::vector<std::pair<double, double>>& hole : holes)
        for (std::pair<double, double>& p : hole) {
            const int64_t iy = to_int64(p.second * scale);

            if (border_ys.count(iy))
                p.second = static_cast<double>(iy - 1) / scale;
        }

    return holes;
}

/// Integer ring, closing duplicate dropped.
std::vector<std::array<int64_t, 2>> to_path64(const std::vector<std::pair<double, double>>& pts, double scale) {

    std::vector<std::array<int64_t, 2>> path;
    path.reserve(pts.size());

    for (const std::pair<double, double>& p : pts)
        path.push_back(to_point64(p, scale));

    if (path.size() > 1 && path.front() == path.back())
        path.pop_back();

    return path;
}

/// Index of every integer point in the flat list [border..., hole0..., hole1...], first occurrence wins.
std::unordered_map<std::array<int64_t, 2>, int, Point64Hash> index_map(
    const std::vector<std::pair<double, double>>& border_2d,
    const std::vector<std::vector<std::pair<double, double>>>& holes_2d,
    double scale
) {

    std::unordered_map<std::array<int64_t, 2>, int, Point64Hash> indices;
    int index = 0;

    for (const std::pair<double, double>& p : border_2d) {
        indices.emplace(to_point64(p, scale), index);
        ++index;
    }

    for (const std::vector<std::pair<double, double>>& hole : holes_2d)
        for (const std::pair<double, double>& p : hole) {
            indices.emplace(to_point64(p, scale), index);
            ++index;
        }

    return indices;
}

/// A triangle lies in a hole when all its corners are on one hole ring or its centroid is outside the border or inside a hole.
bool inside_hole(
    const std::array<std::array<int64_t, 2>, 3>& tri,
    const std::vector<std::vector<std::array<int64_t, 2>>>& paths,
    const std::vector<std::unordered_set<std::array<int64_t, 2>, Point64Hash>>& hole_sets
) {

    for (const std::unordered_set<std::array<int64_t, 2>, Point64Hash>& set : hole_sets)
        if (set.count(tri[0]) && set.count(tri[1]) && set.count(tri[2]))
            return true;

    const std::array<int64_t, 2> c = {(tri[0][0] + tri[1][0] + tri[2][0]) / 3, (tri[0][1] + tri[1][1] + tri[2][1]) / 3};

    if (!inside_path64(c, paths[0]))
        return true;

    for (size_t h = 1; h < paths.size(); ++h)
        if (inside_path64(c, paths[h]))
            return true;

    return false;
}

/// Drop the triangles the sweep filled inside the holes; edge midpoints are not tested because valid triangles touch the hole rings.
void remove_hole_triangles(
    std::vector<std::array<std::array<int64_t, 2>, 3>>& tris,
    const std::vector<std::vector<std::array<int64_t, 2>>>& paths
) {

    std::vector<std::unordered_set<std::array<int64_t, 2>, Point64Hash>> hole_sets;

    for (size_t h = 1; h < paths.size(); ++h)
        hole_sets.emplace_back(paths[h].begin(), paths[h].end());

    std::vector<std::array<std::array<int64_t, 2>, 3>> kept;
    kept.reserve(tris.size());

    for (const std::array<std::array<int64_t, 2>, 3>& tri : tris)
        if (!inside_hole(tri, paths, hole_sets))
            kept.push_back(tri);

    tris = kept;
}

/// Corner indices into the flat list, triangles with an unknown corner dropped.
std::vector<std::array<int, 3>> to_indices(
    const std::vector<std::array<std::array<int64_t, 2>, 3>>& tris,
    const std::unordered_map<std::array<int64_t, 2>, int, Point64Hash>& indices
) {

    std::vector<std::array<int, 3>> out;
    out.reserve(tris.size());

    for (const std::array<std::array<int64_t, 2>, 3>& tri : tris) {
        std::array<int, 3> f = {0, 0, 0};
        bool known = true;

        for (int k = 0; k < 3; ++k) {
            const auto it = indices.find(tri[k]);

            if (it == indices.end())
                known = false;
            else
                f[k] = it->second;
        }

        if (known)
            out.push_back(f);
    }

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh assembly
// ═══════════════════════════════════════════════════════════════════════════

/// Polyline points without the closing duplicate.
std::vector<Point> strip_close(const Polyline& polyline) {

    std::vector<Point> pts = polyline.get_points();

    if (pts.size() > 1) {
        const Point& f = pts.front();
        const Point& b = pts.back();

        if (std::abs(f[0] - b[0]) < 1e-12 && std::abs(f[1] - b[1]) < 1e-12 && std::abs(f[2] - b[2]) < 1e-12)
            pts.pop_back();
    }

    return pts;
}

/// Signed area of a 2D ring, positive when counter-clockwise.
double signed_area(const std::vector<std::pair<double, double>>& pts) {

    double area = 0.0;
    const size_t n = pts.size();

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;

        area += pts[i].first * pts[j].second - pts[j].first * pts[i].second;
    }

    return area * 0.5;
}

/// Index of the polyline with the largest bounding-box diagonal.
size_t border_index(const std::vector<Polyline>& polylines) {

    size_t border = 0;
    double max_diag = 0.0;

    for (size_t i = 0; i < polylines.size(); ++i) {
        const std::vector<Point> pts = polylines[i].get_points();

        if (pts.size() < 3)
            continue;

        Point lo = pts[0];
        Point hi = pts[0];

        for (const Point& p : pts)
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }

        const double diag = lo.distance(hi);

        if (diag > max_diag) {
            max_diag = diag;
            border = i;
        }
    }

    return border;
}

/// Plane coordinates of the points in the frame (origin, xaxis, yaxis).
std::vector<std::pair<double, double>> project_2d(
    const std::vector<Point>& pts,
    const Point& origin,
    const Vector& xaxis,
    const Vector& yaxis
) {

    std::vector<std::pair<double, double>> out;
    out.reserve(pts.size());

    for (const Point& p : pts) {
        const double dx = p[0] - origin[0];
        const double dy = p[1] - origin[1];
        const double dz = p[2] - origin[2];

        out.push_back({dx * xaxis[0] + dy * xaxis[1] + dz * xaxis[2], dx * yaxis[0] + dy * yaxis[1] + dz * yaxis[2]});
    }

    return out;
}

/// Ear triangles for border vertices no triangle touches, so every vertex is drawn.
void cover_missing(std::vector<std::array<size_t, 3>>& tri_list, const std::vector<size_t>& vkeys, size_t n) {

    std::unordered_set<size_t> covered;

    for (const std::array<size_t, 3>& t : tri_list)
        covered.insert(t.begin(), t.end());

    for (size_t m = 0; m < n; ++m)
        if (!covered.count(vkeys[m]))
            tri_list.push_back({vkeys[(m + n - 1) % n], vkeys[m], vkeys[(m + 1) % n]});
}

/// One face over the border with the holes as face holes, or one face per triangle under SESSION_CONFIG.explode_mesh_faces.
Mesh build_mesh(
    const std::vector<Point>& border,
    const std::vector<std::vector<Point>>& holes,
    const std::vector<std::array<int, 3>>& tris
) {

    Mesh mesh;
    std::vector<size_t> vkeys;

    for (const Point& p : border)
        vkeys.push_back(mesh.add_vertex(p));

    for (const std::vector<Point>& hole : holes)
        for (const Point& p : hole)
            vkeys.push_back(mesh.add_vertex(p));

    if (SESSION_CONFIG.explode_mesh_faces) {
        for (const std::array<int, 3>& t : tris)
            mesh.add_face({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});

        return mesh;
    }

    const std::vector<size_t> ring(vkeys.begin(), vkeys.begin() + border.size());
    const std::optional<size_t> fkey = mesh.add_face(ring);

    if (!fkey.has_value())
        return mesh;

    std::vector<std::array<size_t, 3>> tri_list;

    for (const std::array<int, 3>& t : tris) {
        const std::array<size_t, 3> f = {vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]};

        if (f[0] != f[1] && f[1] != f[2] && f[2] != f[0])
            tri_list.push_back(f);
    }

    if (holes.empty()) {
        cover_missing(tri_list, vkeys, border.size());
    } else {
        std::vector<std::vector<size_t>> hole_rings;
        size_t off = border.size();

        for (const std::vector<Point>& hole : holes) {
            hole_rings.emplace_back(vkeys.begin() + off, vkeys.begin() + off + hole.size());
            off += hole.size();
        }

        mesh.set_face_holes(fkey.value(), hole_rings);
    }

    mesh.set_face_triangulation(fkey.value(), tri_list);

    return mesh;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// RemeshCDT
// ═══════════════════════════════════════════════════════════════════════════
std::vector<std::array<int, 3>> cdt_triangulate(
    const std::vector<std::pair<double, double>>& border_2d,
    const std::vector<std::vector<std::pair<double, double>>>& holes_2d
) {

    const double scale = cdt_scale(border_2d, holes_2d);
    const std::vector<std::vector<std::pair<double, double>>> holes = shift_hole_rows(border_2d, holes_2d, scale);
    std::vector<std::vector<std::array<int64_t, 2>>> paths;

    paths.push_back(to_path64(border_2d, scale));

    for (const std::vector<std::pair<double, double>>& hole : holes)
        paths.push_back(to_path64(hole, scale));

    Delaunay delaunay;
    std::vector<std::array<std::array<int64_t, 2>, 3>> tris = delaunay.execute(paths);

    if (!holes.empty())
        remove_hole_triangles(tris, paths);

    return to_indices(tris, index_map(border_2d, holes, scale));
}

std::vector<std::array<int, 3>> RemeshCDT::triangulate(const std::vector<Polyline>& polylines) {

    if (polylines.empty())
        return {};

    const std::vector<Point> border = strip_close(polylines[0]);

    if (border.size() < 3)
        return {};

    std::vector<std::pair<double, double>> border_2d;

    for (const Point& p : border)
        border_2d.push_back({p[0], p[1]});

    std::vector<std::vector<std::pair<double, double>>> holes_2d;

    for (size_t i = 1; i < polylines.size(); ++i) {
        std::vector<std::pair<double, double>> hole_2d;

        for (const Point& p : strip_close(polylines[i]))
            hole_2d.push_back({p[0], p[1]});

        holes_2d.push_back(hole_2d);
    }

    return cdt_triangulate(border_2d, holes_2d);
}

Mesh RemeshCDT::from_polylines(const std::vector<Polyline>& polylines, bool is_2d, bool is_first_boundary) {

    if (polylines.empty())
        return Mesh();

    const size_t border_idx = is_first_boundary || polylines.size() == 1 ? 0 : border_index(polylines);
    std::vector<Point> border = strip_close(polylines[border_idx]);

    if (border.size() < 3)
        return Mesh();

    std::vector<std::vector<Point>> holes;

    for (size_t i = 0; i < polylines.size(); ++i) {
        if (i == border_idx)
            continue;

        const std::vector<Point> hole = strip_close(polylines[i]);

        if (hole.size() >= 3)
            holes.push_back(hole);
    }

    Point origin(0.0, 0.0, 0.0);
    Vector xaxis(1.0, 0.0, 0.0);
    Vector yaxis(0.0, 1.0, 0.0);
    Vector zaxis(0.0, 0.0, 1.0);

    if (!is_2d) {
        std::vector<Point> all_pts = border;

        for (const std::vector<Point>& hole : holes)
            all_pts.insert(all_pts.end(), hole.begin(), hole.end());

        Polyline(all_pts).get_average_plane(origin, xaxis, yaxis, zaxis);
    }

    std::vector<std::pair<double, double>> border_2d = project_2d(border, origin, xaxis, yaxis);

    if (signed_area(border_2d) < 0.0) {
        std::reverse(border.begin(), border.end());
        std::reverse(border_2d.begin(), border_2d.end());
    }

    std::vector<std::vector<std::pair<double, double>>> holes_2d;

    for (std::vector<Point>& hole : holes) {
        std::vector<std::pair<double, double>> hole_2d = project_2d(hole, origin, xaxis, yaxis);

        if (signed_area(hole_2d) > 0.0) {
            std::reverse(hole.begin(), hole.end());
            std::reverse(hole_2d.begin(), hole_2d.end());
        }

        holes_2d.push_back(hole_2d);
    }

    return build_mesh(border, holes, cdt_triangulate(border_2d, holes_2d));
}

} // namespace session_cpp
