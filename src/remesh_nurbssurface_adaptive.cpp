#include "remesh_nurbssurface_adaptive.h"
#include "tolerance.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <map>
#include <optional>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
namespace {
namespace adaptive {

constexpr int MAX_DEPTH = 8;
constexpr int STACK_SIZE = 64;
constexpr double KEY_SCALE = 1e10;

/// Surface sample: position and unit normal, zero where the surface has none.
class Corner {
public:
    Point p; // Surface point.
    Vector n; // Unit normal, zero at a pole.
};

/// Quadtree cell: UV bounds, corners SW SE NE NW and the centre.
class Node {
public:
    double u0; // Lower u bound.
    double v0; // Lower v bound.
    double u1; // Upper u bound.
    double v1; // Upper v bound.
    Corner c[5]; // Corners SW SE NE NW and the centre.
    int depth; // Subdivision depth from the root.
    bool leaf; // True until the cell is split.
};

/// Quadtree over the UV domain: the surface it samples, its tolerances, the cells, the leaf corners by key and the mesh it emits.
class Quadtree {
public:
    const NurbsSurface& s; // Surface sampled.
    std::vector<double> usp; // Span vector in u.
    std::vector<double> vsp; // Span vector in v.
    bool closed[2]; // True per direction when the surface closes on itself.
    double norm_tol; // Normal turn tolerance in squared length.
    double chord_tol; // Chord height tolerance.
    double max_edge; // Longest cell edge, 0 for no limit.
    double min_edge; // Shortest cell edge still split, 0 for no limit.
    std::vector<Node> nodes; // Cell pool, root cells first.
    std::map<std::pair<int64_t, int64_t>, Corner> corners; // Leaf corners by key.
    std::map<int64_t, std::vector<int64_t>> rows; // U keys on each row.
    std::map<int64_t, std::vector<int64_t>> cols; // V keys on each column.
    Mesh mesh; // Mesh emitted.
    std::map<std::pair<int64_t, int64_t>, size_t> keys; // Mesh vertex per key.
    std::optional<size_t> south; // Pole vertex on the v0 side.
    std::optional<size_t> north; // Pole vertex on the v1 side.

    /// Construct over a surface with its span vectors, seam flags and tolerances.
    Quadtree(const NurbsSurface& surface, double norm_tol, double chord_tol, double max_edge, double min_edge)
        : s(surface), usp(surface.get_span_vector(0)), vsp(surface.get_span_vector(1)),
          closed{surface.is_closed(0), surface.is_closed(1)}, norm_tol(norm_tol), chord_tol(chord_tol),
          max_edge(max_edge), min_edge(min_edge) {}
};

// ═══════════════════════════════════════════════════════════════════════════
// Sampling
// ═══════════════════════════════════════════════════════════════════════════
/// Euclidean length without the zero gate of magnitude().
double norm(const Vector& v) {
    return std::sqrt(v.magnitude_squared());
}

/// Squared distance between two points.
double dist2(const Point& a, const Point& b) {
    return (a - b).magnitude_squared();
}

/// Midpoint of two points.
Point midpoint(const Point& a, const Point& b) {
    return Point::sum(a, b) * 0.5;
}

/// Diagonal of the control point bounding box.
double bbox_diagonal(const NurbsSurface& s) {

    Point lo(1e30, 1e30, 1e30);
    Point hi(-1e30, -1e30, -1e30);

    for (int i = 0; i < s.cv_count(0); ++i)
        for (int j = 0; j < s.cv_count(1); ++j) {
            const Point p = s.get_cv(i, j);

            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }

    return norm(hi - lo);
}

/// Point and unit normal Su x Sv at (u, v); zero normal at a pole, where normal_at would give +Z.
Corner sample(const NurbsSurface& s, double u, double v) {

    Corner c = {s.point_at(u, v), Vector(0.0, 0.0, 0.0)};
    const std::vector<Vector> derivatives = s.evaluate(u, v, 1);

    if (derivatives.size() < 3)
        return c;

    const Vector n = derivatives[2].cross(derivatives[1]);
    const double length = norm(n);

    if (length > 1e-10)
        c.n = n / length;

    return c;
}

/// Leaf cell over [u0, u1] x [v0, v1] from its sampled corners SW SE NE NW, centre sampled here.
Node compute_node(
    const NurbsSurface& s,
    double u0,
    double v0,
    double u1,
    double v1,
    const std::array<Corner, 4>& corners,
    int depth
) {

    const Corner centre = sample(s, (u0 + u1) * 0.5, (v0 + v1) * 0.5);

    return {u0, v0, u1, v1, {corners[0], corners[1], corners[2], corners[3], centre}, depth, true};
}

/// Edge midpoints S, E, N, W of the cell.
std::array<Corner, 4> sample_edges(const NurbsSurface& s, const Node& p) {

    const double um = (p.u0 + p.u1) * 0.5;
    const double vm = (p.v0 + p.v1) * 0.5;

    return {sample(s, um, p.v0), sample(s, p.u1, vm), sample(s, um, p.v1), sample(s, p.u0, vm)};
}

// ═══════════════════════════════════════════════════════════════════════════
// Splitting
// ═══════════════════════════════════════════════════════════════════════════
/// True when both normals exist and differ by more than norm_tol in squared length.
bool normals_turn(const Corner& a, const Corner& b, double norm_tol) {

    if (a.n.magnitude_squared() <= 1e-20 || b.n.magnitude_squared() <= 1e-20)
        return false;

    return (a.n - b.n).magnitude_squared() > norm_tol;
}

/// True when the edge midpoint sits more than chord_tol off the chord from a to b.
bool chord_off(const Corner& mid, const Corner& a, const Corner& b, double chord_tol) {
    return dist2(mid.p, midpoint(a.p, b.p)) > chord_tol * chord_tol;
}

/// True when every edge is shorter than min_edge, so the cell is not split further.
bool too_short(const Node& p, double min_edge) {

    if (min_edge <= 0.0)
        return false;

    double longest = 0.0;

    for (int i = 0; i < 4; ++i)
        longest = std::max(longest, dist2(p.c[i].p, p.c[(i + 1) % 4].p));

    return longest < min_edge * min_edge;
}

/// True when the centre sits more than twice chord_tol off a diagonal midpoint; never on a cell with a collapsed edge.
bool twisted(const Node& p, double chord_tol) {

    for (int i = 0; i < 4; ++i)
        if (dist2(p.c[i].p, p.c[(i + 1) % 4].p) < chord_tol * chord_tol)
            return false;

    const double twist_tol2 = 4.0 * chord_tol * chord_tol;

    return dist2(p.c[4].p, midpoint(p.c[0].p, p.c[2].p)) > twist_tol2 ||
        dist2(p.c[4].p, midpoint(p.c[1].p, p.c[3].p)) > twist_tol2;
}

/// Per direction, true when the arc at the centre, taken as a circle of its normal curvature, rises more than chord_tol over its chord.
std::pair<bool, bool> curved(const NurbsSurface& s, const Node& p, double chord_tol) {

    const std::vector<Vector> d = s.evaluate((p.u0 + p.u1) * 0.5, (p.v0 + p.v1) * 0.5, 2);

    if (d.size() < 6)
        return {false, false};

    const Vector su = d[3];
    const Vector sv = d[1];
    const Vector suu = d[5];
    const Vector svv = d[2];
    const Vector n = su.cross(sv);
    const double length = norm(n);
    const double su2 = su.magnitude_squared();
    const double sv2 = sv.magnitude_squared();

    if (length <= 1e-10 || su2 <= 1e-20 || sv2 <= 1e-20)
        return {false, false};

    const Vector unit = n * (1.0 / length);
    const double kappa_u = std::abs(suu.dot(unit)) / su2;
    const double kappa_v = std::abs(svv.dot(unit)) / sv2;
    const double span_u = std::sqrt(su2) * (p.u1 - p.u0);
    const double span_v = std::sqrt(sv2) * (p.v1 - p.v0);
    const bool curved_u = kappa_u > 1e-20 && span_u * std::sqrt(kappa_u / (8.0 * chord_tol)) > 1.0;
    const bool curved_v = kappa_v > 1e-20 && span_v * std::sqrt(kappa_v / (8.0 * chord_tol)) > 1.0;

    return {curved_u, curved_v};
}

/// Directions to split: normals turning along an edge or from a corner to its midpoint, a midpoint off its chord, a twisted centre, an edge past max_edge, or the curvature at the centre.
std::pair<bool, bool> split_flags(const Quadtree& q, const Node& p, const std::array<Corner, 4>& mids) {

    const Corner& sw = p.c[0];
    const Corner& se = p.c[1];
    const Corner& ne = p.c[2];
    const Corner& nw = p.c[3];

    bool split_u = normals_turn(sw, se, q.norm_tol) || normals_turn(ne, nw, q.norm_tol);
    bool split_v = normals_turn(se, ne, q.norm_tol) || normals_turn(nw, sw, q.norm_tol);

    split_u = split_u || chord_off(mids[0], sw, se, q.chord_tol) || chord_off(mids[2], nw, ne, q.chord_tol);
    split_u = split_u || normals_turn(mids[0], sw, q.norm_tol) || normals_turn(mids[2], nw, q.norm_tol);
    split_v = split_v || chord_off(mids[3], sw, nw, q.chord_tol) || chord_off(mids[1], se, ne, q.chord_tol);
    split_v = split_v || normals_turn(mids[3], sw, q.norm_tol) || normals_turn(mids[1], se, q.norm_tol);

    if (!split_u && !split_v && twisted(p, q.chord_tol)) {
        split_u = true;
        split_v = true;
    }

    if (q.max_edge > 0.0) {
        const double limit = q.max_edge * q.max_edge;

        split_u = split_u || dist2(sw.p, se.p) > limit || dist2(ne.p, nw.p) > limit;
        split_v = split_v || dist2(se.p, ne.p) > limit || dist2(nw.p, sw.p) > limit;
    }

    if (!split_u || !split_v) {
        const std::pair<bool, bool> curvature = curved(q.s, p, q.chord_tol);

        split_u = split_u || curvature.first;
        split_v = split_v || curvature.second;
    }

    return {split_u, split_v};
}

/// Children of cell idx appended to the pool: four quadrants, or two halves along the split direction.
void split_node(Quadtree& q, int idx, const std::array<Corner, 4>& mids, bool split_u, bool split_v) {

    const Node p = q.nodes[idx];
    const double um = (p.u0 + p.u1) * 0.5;
    const double vm = (p.v0 + p.v1) * 0.5;
    const int depth = p.depth + 1;

    q.nodes[idx].leaf = false;

    if (split_u && split_v) {
        q.nodes.push_back(compute_node(q.s, p.u0, p.v0, um, vm, {p.c[0], mids[0], p.c[4], mids[3]}, depth));
        q.nodes.push_back(compute_node(q.s, um, p.v0, p.u1, vm, {mids[0], p.c[1], mids[1], p.c[4]}, depth));
        q.nodes.push_back(compute_node(q.s, um, vm, p.u1, p.v1, {p.c[4], mids[1], p.c[2], mids[2]}, depth));
        q.nodes.push_back(compute_node(q.s, p.u0, vm, um, p.v1, {mids[3], p.c[4], mids[2], p.c[3]}, depth));
    } else if (split_u) {
        q.nodes.push_back(compute_node(q.s, p.u0, p.v0, um, p.v1, {p.c[0], mids[0], mids[2], p.c[3]}, depth));
        q.nodes.push_back(compute_node(q.s, um, p.v0, p.u1, p.v1, {mids[0], p.c[1], p.c[2], mids[2]}, depth));
    } else {
        q.nodes.push_back(compute_node(q.s, p.u0, p.v0, p.u1, vm, {p.c[0], p.c[1], mids[1], mids[3]}, depth));
        q.nodes.push_back(compute_node(q.s, p.u0, vm, p.u1, p.v1, {mids[3], mids[1], p.c[2], p.c[3]}, depth));
    }
}

/// Cells split from root down to MAX_DEPTH over an explicit stack, first child popped first so the pool fills depth first.
void subdivide(Quadtree& q, int root) {

    int stack[STACK_SIZE];
    int top = 0;

    stack[top++] = root;

    while (top > 0) {
        const int idx = stack[--top];
        const Node p = q.nodes[idx];

        if (p.depth >= MAX_DEPTH || too_short(p, q.min_edge))
            continue;

        const std::array<Corner, 4> mids = sample_edges(q.s, p);
        const std::pair<bool, bool> split = split_flags(q, p, mids);

        if (!split.first && !split.second)
            continue;

        const int first = (int)q.nodes.size();

        split_node(q, idx, mids, split.first, split.second);

        const int count = (int)q.nodes.size() - first;

        assert(top + count <= STACK_SIZE);

        for (int i = count - 1; i >= 0; --i)
            stack[top++] = first + i;
    }
}

/// One root cell per span pair, corners from the grid of span intersections, each subdivided before the next.
void build(Quadtree& q) {

    const int nu = (int)q.usp.size();
    const int nv = (int)q.vsp.size();

    std::vector<Corner> grid(nu * nv);

    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j)
            grid[i * nv + j] = sample(q.s, q.usp[i], q.vsp[j]);

    for (int i = 0; i + 1 < nu; ++i)
        for (int j = 0; j + 1 < nv; ++j) {
            const int root = (int)q.nodes.size();

            q.nodes.push_back(compute_node(
                q.s,
                q.usp[i],
                q.vsp[j],
                q.usp[i + 1],
                q.vsp[j + 1],
                {grid[i * nv + j], grid[(i + 1) * nv + j], grid[(i + 1) * nv + j + 1], grid[i * nv + j + 1]},
                0
            ));
            subdivide(q, root);
        }
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertices and faces
// ═══════════════════════════════════════════════════════════════════════════
/// t rounded at KEY_SCALE, so a parameter reached from two cells keys alike.
int64_t quantize(double t) {
    return (int64_t)std::round(t * KEY_SCALE);
}

/// t at the seam of a closed direction maps to the start of sp.
double wrap(bool closed, const std::vector<double>& sp, double t) {
    return closed && std::abs(t - sp.back()) < 1e-10 ? sp.front() : t;
}

/// Sorted without repeats.
void sort_unique(std::vector<int64_t>& line) {

    std::sort(line.begin(), line.end());
    line.erase(std::unique(line.begin(), line.end()), line.end());
}

/// Every leaf corner by key, and the u keys on each row and v keys on each column for the T-junction search.
void index_leaves(Quadtree& q) {

    for (const Node& nd : q.nodes) {
        if (!nd.leaf)
            continue;

        const double us[4] = {nd.u0, nd.u1, nd.u1, nd.u0};
        const double vs[4] = {nd.v0, nd.v0, nd.v1, nd.v1};

        for (int ci = 0; ci < 4; ++ci) {
            const std::pair<int64_t, int64_t> key = {
                quantize(wrap(q.closed[0], q.usp, us[ci])),
                quantize(wrap(q.closed[1], q.vsp, vs[ci]))
            };

            q.corners.emplace(key, nd.c[ci]);
            q.rows[key.second].push_back(quantize(us[ci]));
            q.cols[key.first].push_back(quantize(vs[ci]));
        }
    }

    for (std::pair<const int64_t, std::vector<int64_t>>& row : q.rows)
        sort_unique(row.second);

    for (std::pair<const int64_t, std::vector<int64_t>>& col : q.cols)
        sort_unique(col.second);
}

/// Parameters on one line strictly between t0 and t1, in walk order from t0 to t1.
std::vector<double> between(const std::map<int64_t, std::vector<int64_t>>& lines, int64_t line, double t0, double t1) {

    std::vector<double> result;
    const auto found = lines.find(line);

    if (found == lines.end())
        return result;

    const int64_t lo = std::min(quantize(t0), quantize(t1));
    const int64_t hi = std::max(quantize(t0), quantize(t1));
    const auto first = std::upper_bound(found->second.begin(), found->second.end(), lo);
    const auto last = std::lower_bound(found->second.begin(), found->second.end(), hi);

    for (std::vector<int64_t>::const_iterator k = first; k != last; ++k)
        result.push_back(*k / KEY_SCALE);

    if (t0 > t1)
        std::reverse(result.begin(), result.end());

    return result;
}

/// T-junction parameters between u0 and u1 on the row at v.
std::vector<double> row_mids(const Quadtree& q, double u0, double u1, double v) {
    return between(q.rows, quantize(wrap(q.closed[1], q.vsp, v)), u0, u1);
}

/// T-junction parameters between v0 and v1 on the column at u.
std::vector<double> col_mids(const Quadtree& q, double u, double v0, double v1) {
    return between(q.cols, quantize(wrap(q.closed[0], q.usp, u)), v0, v1);
}

/// Mesh vertex at (u, v): the pole on a singular side, else one per key, sampled when no leaf corner holds it.
size_t vertex_at(Quadtree& q, double u, double v) {

    if (q.south && std::abs(v - q.vsp.front()) < 1e-10)
        return *q.south;

    if (q.north && std::abs(v - q.vsp.back()) < 1e-10)
        return *q.north;

    const double uw = wrap(q.closed[0], q.usp, u);
    const double vw = wrap(q.closed[1], q.vsp, v);
    const std::pair<int64_t, int64_t> key = {quantize(uw), quantize(vw)};
    const auto found = q.keys.find(key);

    if (found != q.keys.end())
        return found->second;

    if (!q.corners.count(key))
        q.corners[key] = sample(q.s, uw, vw);

    const size_t vertex = q.mesh.add_vertex(q.corners[key].p);

    q.mesh.vertex[vertex].attributes["u"] = uw;
    q.mesh.vertex[vertex].attributes["v"] = vw;
    q.keys[key] = vertex;

    return vertex;
}

/// Vertices counter-clockwise around the leaf with the T-junction vertices on each edge, repeats at poles and seams dropped.
std::vector<size_t> leaf_polygon(Quadtree& q, const Node& nd) {

    std::vector<size_t> poly = {vertex_at(q, nd.u0, nd.v0)};

    for (double u : row_mids(q, nd.u0, nd.u1, nd.v0))
        poly.push_back(vertex_at(q, u, nd.v0));

    poly.push_back(vertex_at(q, nd.u1, nd.v0));

    for (double v : col_mids(q, nd.u1, nd.v0, nd.v1))
        poly.push_back(vertex_at(q, nd.u1, v));

    poly.push_back(vertex_at(q, nd.u1, nd.v1));

    for (double u : row_mids(q, nd.u1, nd.u0, nd.v1))
        poly.push_back(vertex_at(q, u, nd.v1));

    poly.push_back(vertex_at(q, nd.u0, nd.v1));

    for (double v : col_mids(q, nd.u0, nd.v1, nd.v0))
        poly.push_back(vertex_at(q, nd.u0, v));

    poly.erase(std::unique(poly.begin(), poly.end()), poly.end());

    while (poly.size() > 1 && poly.front() == poly.back())
        poly.pop_back();

    return poly;
}

/// Faces of one leaf: a triangle, a quad cut along its shorter diagonal, or a fan around the centre once T-junctions add vertices.
void add_leaf_faces(Quadtree& q, const Node& nd) {

    const std::vector<size_t> poly = leaf_polygon(q, nd);
    const int n = (int)poly.size();

    if (n < 3)
        return;

    if (n == 3) {
        q.mesh.add_face({poly[0], poly[1], poly[2]});

        return;
    }

    if (n == 4) {
        const Point p0 = q.mesh.vertex.at(poly[0]).position();
        const Point p1 = q.mesh.vertex.at(poly[1]).position();
        const Point p2 = q.mesh.vertex.at(poly[2]).position();
        const Point p3 = q.mesh.vertex.at(poly[3]).position();

        if (dist2(p0, p2) <= dist2(p1, p3)) {
            q.mesh.add_face({poly[0], poly[1], poly[2]});
            q.mesh.add_face({poly[0], poly[2], poly[3]});
        } else {
            q.mesh.add_face({poly[0], poly[1], poly[3]});
            q.mesh.add_face({poly[1], poly[2], poly[3]});
        }

        return;
    }

    const double cu = wrap(q.closed[0], q.usp, (nd.u0 + nd.u1) * 0.5);
    const double cv = wrap(q.closed[1], q.vsp, (nd.v0 + nd.v1) * 0.5);

    q.corners.emplace(std::pair<int64_t, int64_t>(quantize(cu), quantize(cv)), nd.c[4]);

    const size_t centre = vertex_at(q, cu, cv);

    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;

        if (poly[i] != poly[j] && poly[i] != centre && poly[j] != centre)
            q.mesh.add_face({poly[i], poly[j], centre});
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Normals
// ═══════════════════════════════════════════════════════════════════════════
/// Sum of the unnormalized face normals around each vertex key, faces taken in key order.
std::vector<Vector> fan_normals(const Mesh& mesh) {

    std::vector<Vector> sums(mesh.vertex.size(), Vector(0.0, 0.0, 0.0));

    for (const std::pair<const size_t, std::vector<size_t>>& face : mesh.face) {
        const std::vector<size_t>& vertices = face.second;
        const Point p0 = mesh.vertex.at(vertices[0]).position();
        const Point p1 = mesh.vertex.at(vertices[1]).position();
        const Point p2 = mesh.vertex.at(vertices[2]).position();
        const Vector n = (p1 - p0).cross(p2 - p0);

        for (size_t vertex : vertices)
            sums[vertex] += n;
    }

    return sums;
}

/// Unit fan normal on every vertex, zero where the fan cancels.
void set_normals(Mesh& mesh) {

    const std::vector<Vector> sums = fan_normals(mesh);

    for (std::pair<const size_t, VertexData>& vertex : mesh.vertex) {
        const size_t key = vertex.first;
        const double length = norm(sums[key]);
        const Vector n = length > 1e-15 ? sums[key] / length : sums[key];

        vertex.second.set_normal(n[0], n[1], n[2]);
    }
}

} // namespace adaptive
} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
RemeshNurbsSurfaceAdaptive::RemeshNurbsSurfaceAdaptive(const NurbsSurface& surface) : m_surface(surface) {}

// ═══════════════════════════════════════════════════════════════════════════
// Mutators
// ═══════════════════════════════════════════════════════════════════════════
RemeshNurbsSurfaceAdaptive& RemeshNurbsSurfaceAdaptive::set_max_angle(double degrees) {

    m_max_angle = degrees;

    return *this;
}

RemeshNurbsSurfaceAdaptive& RemeshNurbsSurfaceAdaptive::set_max_edge_length(double length) {

    m_max_edge_length = length;

    return *this;
}

RemeshNurbsSurfaceAdaptive& RemeshNurbsSurfaceAdaptive::set_min_edge_length(double length) {

    m_min_edge_length = length;

    return *this;
}

RemeshNurbsSurfaceAdaptive& RemeshNurbsSurfaceAdaptive::set_max_chord_height(double height) {

    m_max_chord_height = height;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshing
// ═══════════════════════════════════════════════════════════════════════════
Mesh RemeshNurbsSurfaceAdaptive::mesh() const {

    const double norm_tol = 2.0 - 2.0 * std::cos(m_max_angle * Tolerance::PI / 180.0);
    const double chord_tol = m_max_chord_height > 0.0 ? m_max_chord_height : adaptive::bbox_diagonal(m_surface) * 0.005;

    adaptive::Quadtree q(m_surface, norm_tol, chord_tol, m_max_edge_length, m_min_edge_length);

    adaptive::build(q);
    adaptive::index_leaves(q);

    if (m_surface.is_singular(0))
        q.south = q.mesh.add_vertex(m_surface.point_at(q.usp.front(), q.vsp.front()));

    if (m_surface.is_singular(2))
        q.north = q.mesh.add_vertex(m_surface.point_at(q.usp.front(), q.vsp.back()));

    for (const adaptive::Node& nd : q.nodes)
        if (nd.leaf)
            adaptive::add_leaf_faces(q, nd);

    adaptive::set_normals(q.mesh);

    return q.mesh;
}

} // namespace session_cpp
