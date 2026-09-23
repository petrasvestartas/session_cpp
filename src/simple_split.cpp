#include "simple_split.h"
#include "closest.h"
#include "line.h"
#include "polyline.h"
#include "tolerance.h"
#include "vector.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>

namespace session_cpp::simple_split {

namespace {

// ═══════════════════════════════════════════════════════════════════════════
// Types
// ═══════════════════════════════════════════════════════════════════════════
constexpr double EPSILON = Tolerance::ZERO_TOLERANCE; // Relative parameter and determinant epsilon.
constexpr BRepOrientation FORWARD = BRepOrientation::Forward; // Use along the curve direction.
constexpr BRepOrientation REVERSED = BRepOrientation::Reversed; // Use against the curve direction.
constexpr size_t WORK_LIMIT = 200000; // Upper bound on subdivision steps.

/// Axis-aligned box around the control points of a curve.
struct Bounds {
    std::array<double, 3> lo; // Minimum corner.
    std::array<double, 3> hi; // Maximum corner.

    /// Box around the control points of a curve.
    explicit Bounds(const NurbsCurve& curve) {

        Point p = curve.get_cv(0);

        for (int d = 0; d < 3; ++d)
            lo[d] = hi[d] = p[d];

        for (int i = 1; i < curve.cv_count(); ++i) {
            p = curve.get_cv(i);

            for (int d = 0; d < 3; ++d) {
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
        }
    }

    /// Length of the box diagonal.
    double diagonal() const {
        return std::hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
    }

    /// True when the boxes overlap within tolerance.
    bool overlaps(const Bounds& other, double tolerance) const {

        for (int d = 0; d < 3; ++d)
            if (hi[d] + tolerance < other.lo[d] || other.hi[d] + tolerance < lo[d])
                return false;

        return true;
    }
};

/// Two curve pieces tested for intersection.
struct Pair {
    NurbsCurve a; // Piece of the first curve.
    NurbsCurve b; // Piece of the second curve.
    int depth; // Subdivision depth.
};

/// Curve piece waiting to be flattened.
struct Part {
    NurbsCurve curve; // Piece of the curve.
    int depth; // Subdivision depth.
};

/// Trim or cutter curve in world and surface parameter space.
struct Source {
    int edge; // BRep edge, -1 for a cutter.
    NurbsCurve world; // 3D curve.
    NurbsCurve uv; // Curve in surface parameter space.
};

/// Parameter run along one source curve.
struct Run {
    size_t source; // Index into the sources.
    double a; // Start parameter.
    double b; // End parameter.
};

/// Knot span of a source curve with its cut parameters.
struct Span {
    size_t source; // Index into the sources.
    double a; // Start parameter.
    double b; // End parameter.
    std::vector<double> cuts; // Cut parameters, span ends included.
    NurbsCurve curve; // Span of the source uv curve.
};

/// Directed half-edge of the trim graph.
struct Directed {
    size_t b; // Head vertex.
    Run run; // Parameter run along the source.
};

/// Planar trim graph, twin half-edges at index ^ 1.
struct Graph {
    std::vector<Directed> edges; // Half-edges.
    std::vector<std::vector<size_t>> outgoing; // Half-edges leaving each vertex, sorted by angle.
};

/// Closed loop of runs with its sampled polygon.
struct Cycle {
    double area; // Signed area in parameter space.
    std::vector<Run> loop; // Runs around the loop.
    std::vector<Point> points; // Sampled polygon.
};

/// New BRep edge cut from a source.
struct Piece {
    size_t source; // Index into the sources.
    double lo; // Start parameter on the world curve.
    double hi; // End parameter on the world curve.
    int edge; // BRep edge.
};

// ═══════════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════════

/// Throw std::invalid_argument when the condition fails.
void require(bool condition, std::string_view message) {

    if (!condition)
        throw std::invalid_argument(std::string(message));
}

/// Reject a tolerance that is not finite and positive.
void check_tolerance(double tolerance) {
    require(std::isfinite(tolerance) && tolerance > 0.0, "Split tolerance must be finite and positive");
}

/// Reject an invalid curve or one with non-finite controls or non-positive weights.
void check_curve(const NurbsCurve& curve) {

    require(curve.is_valid(), "Split requires valid curves");

    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point p = curve.get_cv(i);
        const double w = curve.weight(i);
        require(
            std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) && std::isfinite(w) && w > 0.0,
            "Split requires finite controls and positive rational weights"
        );
    }
}

/// Reject an invalid surface or one with non-finite controls or non-positive weights.
void check_surface(const NurbsSurface& surface) {

    require(surface.is_valid(), "Split requires a valid NURBS surface");

    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const Point p = surface.get_cv(i, j);
            const double w = surface.weight(i, j);
            require(
                std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) && std::isfinite(w) && w > 0.0,
                "Split requires finite surface controls and positive rational weights"
            );
        }
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve parameters
// ═══════════════════════════════════════════════════════════════════════════

/// Copy of a curve trimmed to [a, b], clamped to its domain.
NurbsCurve interval(const NurbsCurve& curve, double a, double b) {

    NurbsCurve result = curve;
    result.refresh_guid();
    const double lo = curve.domain_start();
    const double hi = curve.domain_end();
    a = std::clamp(a, lo, hi);
    b = std::clamp(b, lo, hi);
    require(b > a, "Split produced an empty curve interval");

    if (a > lo || b < hi)
        require(result.trim(a, b), "Kernel refused a split interval");

    return result;
}

/// Closest parameter and distance on a degree-1 curve, exact per segment.
std::pair<double, double> closest_segments(const NurbsCurve& curve, const Point& point, double t) {

    const std::vector<double> spans = curve.get_span_vector();
    double best = std::numeric_limits<double>::infinity();

    for (size_t i = 1; i < spans.size(); ++i) {
        const Point a = curve.point_at(spans[i - 1]);
        const Point b = curve.point_at(spans[i]);
        const Vector v = b - a;
        const double length2 = v.dot(v);

        if (length2 <= EPSILON * EPSILON)
            continue;

        const double fraction = std::clamp((point - a).dot(v) / length2, 0.0, 1.0);
        const NurbsCurve segment = interval(curve, spans[i - 1], spans[i]);
        const double w0 = segment.weight(0);
        const double w1 = segment.weight(segment.cv_count() - 1);
        const double normalized = fraction * w0 / (w1 * (1.0 - fraction) + fraction * w0);
        const double candidate = spans[i - 1] + normalized * (spans[i] - spans[i - 1]);
        const double gap = curve.point_at(candidate).distance(point);

        if (gap < best) {
            best = gap;
            t = candidate;
        }
    }

    return {t, curve.point_at(t).distance(point)};
}

/// Closest parameter and distance from a point to a curve, polished by Newton steps.
std::pair<double, double> closest(const NurbsCurve& curve, const Point& point) {

    double t = Closest::curve_point(curve, point).first;

    if (curve.degree() == 1)
        return closest_segments(curve, point, t);

    const double lo = curve.domain_start();
    const double hi = curve.domain_end();

    for (int i = 0; i < 24; ++i) {
        const std::vector<Vector> eval = curve.evaluate(t, 1);
        const Vector d = eval[1];
        const Vector r = eval[0] - Vector(point[0], point[1], point[2]);
        const double dd = d.dot(d);

        if (dd <= EPSILON * EPSILON)
            break;

        const double next = std::clamp(t - d.dot(r) / dd, lo, hi);

        if (std::abs(next - t) <= EPSILON * (hi - lo)) {
            t = next;
            break;
        }

        t = next;
    }

    return {t, curve.point_at(t).distance(point)};
}

/// Sorted parameters clamped to [lo, hi], dropping near duplicates.
std::vector<double> unique_parameters(std::vector<double> values, double lo, double hi) {

    std::sort(values.begin(), values.end());
    std::vector<double> result;

    for (double value : values) {
        value = std::clamp(value, lo, hi);

        if (result.empty() || value - result.back() > (hi - lo) * EPSILON * 16.0)
            result.push_back(value);
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve intersection
// ═══════════════════════════════════════════════════════════════════════════

/// True when every control point lies within tolerance of the chord.
bool flat(const NurbsCurve& curve, double tolerance) {

    const Point a = curve.point_at_start();
    const Point b = curve.point_at_end();
    const Vector v = b - a;
    const double length2 = v.dot(v);

    if (length2 <= tolerance * tolerance)
        return Bounds(curve).diagonal() <= tolerance;

    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point p = curve.get_cv(i);
        const double t = (p - a).dot(v) / length2;

        if (t < -EPSILON || t > 1.0 + EPSILON || p.distance(a + v * t) > tolerance)
            return false;
    }

    return true;
}

/// Newton refinement of a curve-curve intersection seed.
std::pair<double, double> refine(const NurbsCurve& a, const NurbsCurve& b, double ta, double tb) {

    const double a0 = a.domain_start();
    const double a1 = a.domain_end();
    const double b0 = b.domain_start();
    const double b1 = b.domain_end();

    for (int k = 0; k < 40; ++k) {
        const std::vector<Vector> da = a.evaluate(ta, 1);
        const std::vector<Vector> db = b.evaluate(tb, 1);
        const Vector r = da[0] - db[0];
        const Vector u = da[1];
        const Vector v = db[1];
        const double aa = u.dot(u);
        const double ab = u.dot(v);
        const double bb = v.dot(v);
        const double det = aa * bb - ab * ab;

        if (det <= EPSILON * EPSILON * aa * bb)
            break;

        const double ar = u.dot(r);
        const double br = v.dot(r);
        const double na = std::clamp(ta + (-bb * ar + ab * br) / det, a0, a1);
        const double nb = std::clamp(tb + (-ab * ar + aa * br) / det, b0, b1);
        const bool converged = std::abs(na - ta) < EPSILON * (a1 - a0) && std::abs(nb - tb) < EPSILON * (b1 - b0);
        ta = na;
        tb = nb;

        if (converged)
            break;
    }

    return {ta, tb};
}

/// Throw when two flat pieces overlap along a shared line.
void check_overlap(const Pair& pair, double tolerance) {

    const Point ap = pair.a.point_at_start();
    const Point aq = pair.a.point_at_end();
    const Point bp = pair.b.point_at_start();
    const Point bq = pair.b.point_at_end();
    const Vector u = aq - ap;
    const Vector v = bq - bp;
    const double aa = u.dot(u);
    const double ab = u.dot(v);
    const double vv = v.dot(v);
    const bool parallel = aa > tolerance * tolerance && vv > tolerance * tolerance && aa * vv - ab * ab < EPSILON * EPSILON * aa * vv;

    if (!parallel)
        return;

    const double t0 = (bp - ap).dot(u) / aa;
    const double t1 = (bq - ap).dot(u) / aa;
    const double gap = bp.distance(ap + u * t0);
    const double shared = std::min(1.0, std::max(t0, t1)) - std::max(0.0, std::min(t0, t1));

    if (gap <= tolerance && shared > tolerance / std::sqrt(aa))
        throw std::invalid_argument("Overlapping curves do not define isolated split points");
}

/// True when a hit is already recorded within tolerance on both curves.
bool duplicate(
    const NurbsCurve& a,
    const NurbsCurve& b,
    const std::vector<std::pair<double, double>>& hits,
    double ta,
    double tb,
    double tolerance
) {

    for (const std::pair<double, double>& hit : hits) {
        const bool near_a = a.point_at(hit.first).distance(a.point_at(ta)) <= tolerance * 2.0 &&
                            a.point_at((hit.first + ta) * 0.5).distance(a.point_at(ta)) <= tolerance * 2.0;
        const bool near_b = b.point_at(hit.second).distance(b.point_at(tb)) <= tolerance * 2.0 &&
                            b.point_at((hit.second + tb) * 0.5).distance(b.point_at(tb)) <= tolerance * 2.0;

        if (near_a && near_b)
            return true;
    }

    return false;
}

/// Record the refined crossing of two flat pieces unless it is a duplicate.
void add_hit(
    const NurbsCurve& a,
    const NurbsCurve& b,
    const Pair& pair,
    double tolerance,
    std::vector<std::pair<double, double>>& hits
) {

    check_overlap(pair, tolerance);
    double ta = 0.0;
    double tb = 0.0;
    double d = 0.0;
    std::tie(ta, tb, d) = Closest::curve_curve(pair.a, pair.b);

    if (d > tolerance * 2.0)
        return;

    std::tie(ta, tb) = refine(pair.a, pair.b, ta, tb);

    if (a.point_at(ta).distance(b.point_at(tb)) > tolerance)
        return;

    if (!duplicate(a, b, hits, ta, tb, tolerance))
        hits.emplace_back(ta, tb);
}

/// Halve the piece with the larger box at its parameter midpoint.
void subdivide(const Pair& pair, const Bounds& ba, const Bounds& bb, std::vector<Pair>& work) {

    if (ba.diagonal() >= bb.diagonal()) {
        const double lo = pair.a.domain_start();
        const double hi = pair.a.domain_end();
        const double mid = (lo + hi) * 0.5;
        work.push_back({interval(pair.a, lo, mid), pair.b, pair.depth + 1});
        work.push_back({interval(pair.a, mid, hi), pair.b, pair.depth + 1});
        return;
    }

    const double lo = pair.b.domain_start();
    const double hi = pair.b.domain_end();
    const double mid = (lo + hi) * 0.5;
    work.push_back({pair.a, interval(pair.b, lo, mid), pair.depth + 1});
    work.push_back({pair.a, interval(pair.b, mid, hi), pair.depth + 1});
}

/// Sorted parameter pairs where two curves cross, drawing on a shared work budget.
std::vector<std::pair<double, double>> intersections(
    const NurbsCurve& a,
    const NurbsCurve& b,
    double tolerance,
    size_t& budget
) {

    const std::vector<double> av = a.get_span_vector();
    const std::vector<double> bv = b.get_span_vector();
    require(av.size() > 1 && bv.size() > 1, "Split requires nonempty curve spans");
    require(av.size() - 1 <= budget / (bv.size() - 1), "Curve intersection exceeds the bounded split workload");
    std::vector<Pair> work;

    for (size_t i = 1; i < av.size(); ++i)
        for (size_t j = 1; j < bv.size(); ++j)
            work.push_back({interval(a, av[i - 1], av[i]), interval(b, bv[j - 1], bv[j]), 0});

    std::vector<std::pair<double, double>> hits;

    while (!work.empty()) {
        require(budget > 0, "Curve intersection exceeds the bounded split workload");
        --budget;
        const Pair pair = std::move(work.back());
        work.pop_back();
        const Bounds ba(pair.a);
        const Bounds bb(pair.b);

        if (!ba.overlaps(bb, tolerance))
            continue;

        if ((flat(pair.a, tolerance * 0.1) && flat(pair.b, tolerance * 0.1)) || pair.depth >= 48)
            add_hit(a, b, pair, tolerance, hits);
        else
            subdivide(pair, ba, bb, work);
    }

    std::sort(hits.begin(), hits.end());
    return hits;
}

// ═══════════════════════════════════════════════════════════════════════════
// Trim polygons
// ═══════════════════════════════════════════════════════════════════════════

/// Curves in surface parameter space, exact on a bilinear parallelogram patch.
std::vector<NurbsCurve> pullback(const NurbsSurface& surface, const NurbsCurve& curve, double tolerance) {

    const bool bilinear = surface.m_cv_count[0] == 2 && surface.m_cv_count[1] == 2 && surface.m_order[0] == 2 &&
                          surface.m_order[1] == 2 && !surface.m_is_rat;

    if (!bilinear)
        return Closest::surface_curve(surface, curve, 0.0, 0.0, tolerance);

    const Point p = surface.get_cv(0, 0);
    const Vector u = surface.get_cv(1, 0) - p;
    const Vector v = surface.get_cv(0, 1) - p;
    const Point last = surface.get_cv(1, 1);
    const double uu = u.dot(u);
    const double uv = u.dot(v);
    const double vv = v.dot(v);
    const double det = uu * vv - uv * uv;
    const bool parallelogram = det > EPSILON * EPSILON * uu * vv && last.distance(p + u + v) <= tolerance;

    if (!parallelogram)
        return Closest::surface_curve(surface, curve, 0.0, 0.0, tolerance);

    NurbsCurve result = curve;
    result.refresh_guid();
    double u0 = 0.0;
    double u1 = 0.0;
    double v0 = 0.0;
    double v1 = 0.0;
    std::tie(u0, u1) = surface.domain(0);
    std::tie(v0, v1) = surface.domain(1);

    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point q = curve.get_cv(i);
        const Vector d = q - p;
        const double du = d.dot(u);
        const double dv = d.dot(v);
        const double a = (du * vv - dv * uv) / det;
        const double b = (dv * uu - du * uv) / det;

        if (q.distance(p + u * a + v * b) > tolerance)
            return {};

        const double w = curve.weight(i);
        require(result.set_cv_4d(i, (u0 + a * (u1 - u0)) * w, (v0 + b * (v1 - v0)) * w, 0.0, w), "Kernel refused a pullback control");
    }

    return {result};
}

/// Points sampling a curve until each piece is flat within tolerance.
std::vector<Point> polygon(const NurbsCurve& curve, double tolerance) {

    const std::vector<double> spans = curve.get_span_vector();
    std::vector<Part> work;

    for (size_t i = spans.size(); i > 1; --i)
        work.push_back({interval(curve, spans[i - 2], spans[i - 1]), 0});

    std::vector<Point> result;
    size_t visited = 0;

    while (!work.empty()) {
        ++visited;
        require(visited <= WORK_LIMIT, "Trim sampling exceeds the bounded workload");
        const Part part = std::move(work.back());
        work.pop_back();

        if (flat(part.curve, tolerance * 0.25)) {
            result.push_back(part.curve.point_at_start());
            continue;
        }

        require(part.depth < 40, "Trim sampling exceeds parameter precision");
        const double lo = part.curve.domain_start();
        const double hi = part.curve.domain_end();
        const double mid = (lo + hi) * 0.5;
        work.push_back({interval(part.curve, mid, hi), part.depth + 1});
        work.push_back({interval(part.curve, lo, mid), part.depth + 1});
    }

    return result;
}

/// Even-odd point in polygon test in the xy plane.
bool inside(const Point& p, const std::vector<Point>& polygon) {

    bool result = false;
    size_t j = polygon.size() - 1;

    for (size_t i = 0; i < polygon.size(); ++i) {
        const Point& a = polygon[i];
        const Point& b = polygon[j];

        if ((a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0])
            result = !result;

        j = i;
    }

    return result;
}

/// True inside the first loop and outside every hole loop.
bool inside_loops(const Point& p, const std::vector<std::vector<Point>>& loops) {

    if (loops.empty() || !inside(p, loops[0]))
        return false;

    for (size_t i = 1; i < loops.size(); ++i)
        if (inside(p, loops[i]))
            return false;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Trim arrangement
// ═══════════════════════════════════════════════════════════════════════════

/// Knot spans of every source, cut at their mutual intersections.
std::vector<Span> compute_spans(const std::vector<Source>& sources, double tolerance) {

    std::vector<Span> spans;

    for (size_t si = 0; si < sources.size(); ++si) {
        std::vector<double> knots = sources[si].uv.get_span_vector();

        if (knots.size() == 2 && sources[si].uv.is_closed()) {
            const double lo = knots.front();
            const double hi = knots.back();
            knots = {lo, lo + (hi - lo) * 0.25, (lo + hi) * 0.5, lo + (hi - lo) * 0.75, hi};
        }

        for (size_t i = 1; i < knots.size(); ++i)
            spans.push_back({si, knots[i - 1], knots[i], {knots[i - 1], knots[i]}, interval(sources[si].uv, knots[i - 1], knots[i])});
    }

    require(!spans.empty() && spans.size() <= WORK_LIMIT / spans.size(), "Face split exceeds the bounded workload");
    size_t budget = WORK_LIMIT;

    for (size_t i = 0; i < spans.size(); ++i)
        for (size_t j = i + 1; j < spans.size(); ++j) {
            const std::vector<std::pair<double, double>> hits = intersections(spans[i].curve, spans[j].curve, tolerance, budget);

            for (const std::pair<double, double>& hit : hits) {
                spans[i].cuts.push_back(hit.first);
                spans[j].cuts.push_back(hit.second);
            }
        }

    return spans;
}

/// Cut parameters of a span, snapped to its ends and deduplicated.
std::vector<double> span_cuts(const Span& span, double tolerance) {

    std::vector<double> cuts = span.cuts;

    for (double& t : cuts) {
        const Point p = span.curve.point_at(t);

        if (p.distance(span.curve.point_at(span.a)) <= tolerance)
            t = span.a;
        else if (p.distance(span.curve.point_at(span.b)) <= tolerance)
            t = span.b;
    }

    return unique_parameters(cuts, span.a, span.b);
}

/// Index of the graph vertex at a point, added when none lies within tolerance.
size_t node(Graph& graph, std::vector<Point>& vertices, const Point& p, double tolerance) {

    for (size_t i = 0; i < vertices.size(); ++i)
        if (p.distance(vertices[i]) <= tolerance * 4.0)
            return i;

    vertices.push_back(p);
    graph.outgoing.emplace_back();
    return vertices.size() - 1;
}

/// Direction angle of a run leaving its start vertex.
double angle(const std::vector<Source>& sources, const Run& run) {

    const Vector d = sources[run.source].uv.evaluate(run.a, 1)[1];
    const double sign = run.b > run.a ? 1.0 : -1.0;
    return std::atan2(sign * d[1], sign * d[0]);
}

/// Half-edge graph of the cut spans inside the original loops.
Graph compute_graph(
    const std::vector<Span>& spans,
    const std::vector<Source>& sources,
    const std::vector<std::vector<Point>>& original_loops,
    double tolerance
) {

    Graph graph;
    std::vector<Point> vertices;

    for (const Span& span : spans) {
        const std::vector<double> cuts = span_cuts(span, tolerance);
        const Source& source = sources[span.source];

        for (size_t i = 1; i < cuts.size(); ++i) {
            const double lo = cuts[i - 1];
            const double hi = cuts[i];

            if (source.edge < 0 && !inside_loops(source.uv.point_at((lo + hi) * 0.5), original_loops))
                continue;

            const size_t a = node(graph, vertices, source.uv.point_at(lo), tolerance);
            const size_t b = node(graph, vertices, source.uv.point_at(hi), tolerance);

            if (a == b)
                continue;

            const size_t index = graph.edges.size();
            graph.edges.push_back({b, {span.source, lo, hi}});
            graph.edges.push_back({a, {span.source, hi, lo}});
            graph.outgoing[a].push_back(index);
            graph.outgoing[b].push_back(index + 1);
        }
    }

    std::vector<double> angles;

    for (const Directed& edge : graph.edges)
        angles.push_back(angle(sources, edge.run));

    for (std::vector<size_t>& choices : graph.outgoing)
        std::sort(choices.begin(), choices.end(), [&angles](size_t a, size_t b) { return angles[a] < angles[b]; });

    return graph;
}

/// Signed shoelace area of a closed polygon in the xy plane.
double signed_area(const std::vector<Point>& points) {

    double area = 0.0;

    for (size_t i = 0; i < points.size(); ++i) {
        const Point& a = points[i];
        const Point& b = points[(i + 1) % points.size()];
        area += (a[0] * b[1] - b[0] * a[1]) * 0.5;
    }

    return area;
}

/// Loop traced from one half-edge by turning to the previous outgoing half-edge at each vertex.
Cycle trace_cycle(
    const Graph& graph,
    const std::vector<Source>& sources,
    size_t initial,
    std::vector<bool>& used,
    double tolerance
) {

    Cycle cycle{0.0, {}, {}};
    size_t edge = initial;

    while (!used[edge]) {
        used[edge] = true;
        const Directed& item = graph.edges[edge];
        const Run& run = item.run;
        cycle.loop.push_back(run);
        NurbsCurve part = interval(sources[run.source].uv, std::min(run.a, run.b), std::max(run.a, run.b));

        if (run.b < run.a)
            require(part.reverse(), "Kernel refused to reverse a trim fragment");

        const std::vector<Point> poly = polygon(part, tolerance);
        cycle.points.insert(cycle.points.end(), poly.begin(), poly.end());
        const std::vector<size_t>& options = graph.outgoing[item.b];
        const std::vector<size_t>::const_iterator at = std::find(options.begin(), options.end(), edge ^ 1);
        require(at != options.end(), "Invalid trim graph adjacency");
        const size_t slot = static_cast<size_t>(at - options.begin());
        edge = options[(slot + options.size() - 1) % options.size()];
    }

    require(edge == initial, "Invalid trim graph cycle");
    cycle.area = signed_area(cycle.points);
    return cycle;
}

/// Point eight tolerances left of the middle of a run.
Point left_of(const std::vector<Source>& sources, const Run& run, double tolerance) {

    const NurbsCurve& curve = sources[run.source].uv;
    const double t = (run.a + run.b) * 0.5;
    const Point p = curve.point_at(t);
    const Vector d = curve.evaluate(t, 1)[1];
    const double sign = run.b > run.a ? 1.0 : -1.0;
    const double length = std::hypot(d[0], d[1]);
    require(length > EPSILON, "Cannot orient a degenerate trim fragment");
    return Point(p[0] - sign * d[1] / length * tolerance * 8.0, p[1] + sign * d[0] / length * tolerance * 8.0, 0.0);
}

/// Non-degenerate graph cycles whose interior lies inside the original loops.
std::vector<Cycle> compute_cycles(
    const Graph& graph,
    const std::vector<Source>& sources,
    const std::vector<std::vector<Point>>& original_loops,
    double tolerance
) {

    std::vector<Cycle> cycles;
    std::vector<bool> used(graph.edges.size(), false);

    for (size_t initial = 0; initial < graph.edges.size(); ++initial) {
        if (used[initial])
            continue;

        const Cycle cycle = trace_cycle(graph, sources, initial, used, tolerance);

        if (std::abs(cycle.area) <= tolerance * tolerance)
            continue;

        if (inside_loops(left_of(sources, cycle.loop[0], tolerance), original_loops))
            cycles.push_back(cycle);
    }

    return cycles;
}

/// Regions as outer loops, each hole nested in the smallest outer loop around it.
std::vector<std::vector<std::vector<Run>>> nest_cycles(const std::vector<Cycle>& cycles, double tolerance) {

    std::vector<std::vector<std::vector<Run>>> result;
    std::vector<size_t> positive;

    for (size_t i = 0; i < cycles.size(); ++i) {
        if (cycles[i].area > 0.0) {
            positive.push_back(i);
            result.push_back({cycles[i].loop});
        }
    }

    for (const Cycle& cycle : cycles) {
        if (cycle.area >= 0.0)
            continue;

        size_t parent = result.size();
        double smallest = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < positive.size(); ++i) {
            const Cycle& outer = cycles[positive[i]];

            if (outer.area > std::abs(cycle.area) + tolerance * tolerance && outer.area < smallest && inside(cycle.points[0], outer.points)) {
                parent = i;
                smallest = outer.area;
            }
        }

        require(parent < result.size(), "Unowned interior trim loop");
        result[parent].push_back(cycle.loop);
    }

    return result;
}

/// Regions of the planar arrangement of the sources inside the original loops.
std::vector<std::vector<std::vector<Run>>> arrange(
    const std::vector<Source>& sources,
    const std::vector<std::vector<Point>>& original_loops,
    double tolerance
) {

    const std::vector<Span> spans = compute_spans(sources, tolerance);
    const Graph graph = compute_graph(spans, sources, original_loops, tolerance);
    const std::vector<Cycle> cycles = compute_cycles(graph, sources, original_loops, tolerance);
    return nest_cycles(cycles, tolerance);
}

// ═══════════════════════════════════════════════════════════════════════════
// BRep assembly
// ═══════════════════════════════════════════════════════════════════════════

/// Index of the BRep vertex at a point, added when none lies within tolerance.
int vertex(BRep& result, const Point& p, double tolerance) {

    for (size_t i = 0; i < result.m_vertices.size(); ++i)
        if (result.m_vertices[i].point.distance(p) <= tolerance)
            return static_cast<int>(i);

    return result.add_vertex(p, tolerance);
}

/// Distance from a point to the surface under a pcurve parameter.
double lifted_gap(const NurbsSurface& surface, const NurbsCurve& uv, const Point& p, double t) {

    const Point q = uv.point_at(t);
    return surface.point_at(q[0], q[1]).distance(p);
}

/// Pcurve parameter whose surface point meets a world point, sampled then refined by ternary search.
double lifted_parameter(const NurbsSurface& surface, const NurbsCurve& uv, const Point& p, double expected, double tolerance) {

    const double lo = uv.domain_start();
    const double hi = uv.domain_end();

    if (lifted_gap(surface, uv, p, expected) <= tolerance)
        return expected;

    double best = expected;
    double d = lifted_gap(surface, uv, p, best);
    int index = 0;

    for (int i = 0; i <= 128; ++i) {
        const double t = lo + (hi - lo) * i / 128.0;
        const double value = lifted_gap(surface, uv, p, t);

        if (value < d) {
            d = value;
            best = t;
            index = i;
        }
    }

    double a = lo + (hi - lo) * std::max(0, index - 1) / 128.0;
    double b = lo + (hi - lo) * std::min(128, index + 1) / 128.0;

    for (int k = 0; k < 60; ++k) {
        const double x = a + (b - a) / 3.0;
        const double y = b - (b - a) / 3.0;

        if (lifted_gap(surface, uv, p, x) < lifted_gap(surface, uv, p, y))
            b = y;
        else
            a = x;
    }

    const double mid = (a + b) * 0.5;

    if (lifted_gap(surface, uv, p, mid) < d)
        best = mid;

    require(lifted_gap(surface, uv, p, best) <= tolerance * 4.0, "Cannot keep an adjacent trim on its original shared edge");
    return best;
}

/// World curve parameter and distance for a surface point, proportional guess first.
std::pair<double, double> world_parameter(const Source& source, double t, const Point& p, double tolerance) {

    const double lo = source.world.domain_start();
    const double hi = source.world.domain_end();
    const double a = source.uv.domain_start();
    const double b = source.uv.domain_end();
    const double expected = lo + (t - a) / (b - a) * (hi - lo);
    const double gap = source.world.point_at(expected).distance(p);

    if (gap <= tolerance)
        return {expected, gap};

    return closest(source.world, p);
}

/// World curve parameters of both run ends, a closed curve's seam end moved to the domain end.
std::pair<double, double> world_run(const NurbsSurface& surface, const Source& source, const Run& run, double tolerance) {

    const Point qa = source.uv.point_at(run.a);
    const Point qb = source.uv.point_at(run.b);
    double wa = 0.0;
    double wb = 0.0;
    double da = 0.0;
    double db = 0.0;
    std::tie(wa, da) = world_parameter(source, run.a, surface.point_at(qa[0], qa[1]), tolerance);
    std::tie(wb, db) = world_parameter(source, run.b, surface.point_at(qb[0], qb[1]), tolerance);
    require(da <= tolerance * 4.0 && db <= tolerance * 4.0, "Cutter is not on the selected surface");

    const double w0 = source.world.domain_start();
    const double w1 = source.world.domain_end();
    const double c0 = source.uv.domain_start();
    const double c1 = source.uv.domain_end();

    if (source.world.is_closed()) {
        if (std::abs(wa - w0) < (w1 - w0) * EPSILON && run.a > (c0 + c1) * 0.5)
            wa = w1;

        if (std::abs(wb - w0) < (w1 - w0) * EPSILON && run.b > (c0 + c1) * 0.5)
            wb = w1;
    }

    return {wa, wb};
}

/// Pcurves of a piece of a shared BRep edge, cut from every adjacent face trim.
void add_shared_pcurves(
    BRep& result,
    const BRep& brep,
    const Source& source,
    const Piece& piece,
    const NurbsCurve& world,
    double tolerance
) {

    const double w0 = source.world.domain_start();
    const double w1 = source.world.domain_end();

    for (const BRepCurveOnSurface& pc : brep.m_edges[source.edge].pcurves) {
        const NurbsSurface& surface = brep.m_surfaces[pc.surface_index];
        const std::array<int, 2> sides = {pc.curve_2d_index, pc.curve_2d_index_2};
        std::array<int, 2> ids = {-1, -1};

        for (size_t at = 0; at < 2; ++at) {
            if (sides[at] < 0)
                continue;

            const NurbsCurve& c = brep.m_curves_2d[sides[at]];
            const double c0 = c.domain_start();
            const double c1 = c.domain_end();
            const double ca = lifted_parameter(
                surface,
                c,
                world.point_at_start(),
                c0 + (piece.lo - w0) / (w1 - w0) * (c1 - c0),
                tolerance
            );
            const double cb = lifted_parameter(
                surface,
                c,
                world.point_at_end(),
                c0 + (piece.hi - w0) / (w1 - w0) * (c1 - c0),
                tolerance
            );
            require(cb > ca, "A split crosses an unsupported periodic trim seam");
            ids[at] = result.add_curve_2d(interval(c, ca, cb));
        }

        result.add_pcurve(piece.edge, pc.surface_index, ids[0], ids[1]);
    }
}

/// Oriented BRep edge for a run, reusing the edge already cut for the same stretch of its source.
BRepRef add_run_edge(
    BRep& result,
    std::vector<Piece>& pieces,
    const BRep& brep,
    int surface_index,
    const std::vector<Source>& sources,
    const Run& run,
    double tolerance
) {

    const Source& source = sources[run.source];
    double wa = 0.0;
    double wb = 0.0;
    std::tie(wa, wb) = world_run(brep.m_surfaces[surface_index], source, run, tolerance);
    const double lo = std::min(wa, wb);
    const double hi = std::max(wa, wb);
    const double w0 = source.world.domain_start();
    const double w1 = source.world.domain_end();
    require(hi - lo > (w1 - w0) * EPSILON, "Split would create a collapsed edge");
    const BRepOrientation orientation = wa < wb ? FORWARD : REVERSED;

    for (const Piece& piece : pieces) {
        const bool same = source.edge >= 0 ? sources[piece.source].edge == source.edge : piece.source == run.source;

        if (same && source.world.point_at(lo).distance(source.world.point_at(piece.lo)) <= tolerance * 4.0 &&
            source.world.point_at(hi).distance(source.world.point_at(piece.hi)) <= tolerance * 4.0)
            return {piece.edge, orientation};
    }

    const NurbsCurve world = interval(source.world, lo, hi);
    const int a = vertex(result, world.point_at_start(), tolerance * 4.0);
    const int b = vertex(result, world.point_at_end(), tolerance * 4.0);
    const int edge = result.add_edge(result.add_curve_3d(world), a, b, tolerance);
    const Piece piece = {run.source, lo, hi, edge};

    if (source.edge >= 0) {
        add_shared_pcurves(result, brep, source, piece, world, tolerance);
    } else {
        NurbsCurve pc = interval(source.uv, std::min(run.a, run.b), std::max(run.a, run.b));

        if ((wb - wa) * (run.b - run.a) < 0.0)
            require(pc.reverse(), "Kernel refused to reverse a cutter trim");

        result.add_pcurve(edge, surface_index, result.add_curve_2d(pc));
    }

    pieces.push_back(piece);
    return {edge, orientation};
}

/// Sampled trim loops of a face, each boundary edge added to the sources.
std::vector<std::vector<Point>> boundary_loops(
    const BRep& brep,
    int face_index,
    double uv_tolerance,
    std::vector<Source>& sources
) {

    std::vector<std::vector<Point>> loops;

    for (const BRepRef& wr : brep.m_faces[face_index].wires) {
        std::vector<Point> points;

        for (const BRepRef& er : brep.wire_edges(wr)) {
            const BRepEdge& edge = brep.m_edges[er.index];
            require(!edge.degenerated, "Pole-edge splitting is not supported");
            const int ci = brep.pcurve_index(er.index, face_index, er.orientation);
            require(ci >= 0, "Face has no source UV boundary");
            NurbsCurve uv = brep.m_curves_2d[ci];
            check_curve(uv);
            check_curve(brep.m_curves_3d[edge.curve_3d_index]);
            sources.push_back({er.index, brep.m_curves_3d[edge.curve_3d_index], uv});

            if (er.orientation == REVERSED)
                require(uv.reverse(), "Kernel refused to reverse a face trim");

            const std::vector<Point> poly = polygon(uv, uv_tolerance);
            points.insert(points.end(), poly.begin(), poly.end());
        }

        require(points.size() >= 3, "Face has an invalid boundary");
        loops.push_back(points);
    }

    return loops;
}

/// Replace every use of a cut boundary edge in the wires by its pieces in order.
void replace_wires(BRep& result, const BRep& brep, const std::vector<Source>& sources, const std::vector<Piece>& pieces) {

    std::map<int, std::vector<std::pair<double, int>>> replacements;

    for (const Piece& piece : pieces)
        if (sources[piece.source].edge >= 0)
            replacements[sources[piece.source].edge].push_back({piece.lo, piece.edge});

    for (size_t wi = 0; wi < brep.m_wires.size(); ++wi) {
        std::vector<BRepRef> refs;

        for (const BRepRef& er : brep.m_wires[wi].edges) {
            if (!replacements.contains(er.index)) {
                refs.push_back(er);
                continue;
            }

            std::vector<std::pair<double, int>> items = replacements.at(er.index);
            std::sort(items.begin(), items.end());
            items.erase(std::unique(items.begin(), items.end()), items.end());

            if (er.orientation == REVERSED)
                std::reverse(items.begin(), items.end());

            for (const std::pair<double, int>& item : items)
                refs.push_back({item.second, er.orientation});
        }

        result.m_wires[wi].edges = refs;
    }
}

/// Put the first region on the split face and the others on new faces beside it in every shell.
void add_faces(BRep& result, const BRepFace& face, int face_index, const std::vector<std::vector<BRepRef>>& new_wires) {

    result.m_faces[face_index].wires = new_wires[0];
    std::vector<int> added;

    for (size_t i = 1; i < new_wires.size(); ++i) {
        BRepFace next = face;
        next.wires = new_wires[i];
        added.push_back(result.face_count());
        result.m_faces.push_back(next);
    }

    for (BRepShell& shell : result.m_shells) {
        std::vector<BRepRef> refs;

        for (const BRepRef& fr : shell.faces) {
            refs.push_back(fr);

            if (fr.index == face_index)
                for (int index : added)
                    refs.push_back({index, fr.orientation});
        }

        shell.faces = refs;
    }
}

/// Reject a face boundary whose consecutive edges do not share a vertex.
void check_wires(const BRep& result) {

    for (const BRepFace& face : result.m_faces)
        for (const BRepRef& wr : face.wires) {
            const std::vector<BRepRef> edges = result.wire_edges(wr);

            for (size_t i = 0; i < edges.size(); ++i) {
                const BRepEdge& a = result.m_edges[edges[i].index];
                const BRepRef& next = edges[(i + 1) % edges.size()];
                const BRepEdge& b = result.m_edges[next.index];
                const int tail = edges[i].orientation == REVERSED ? a.start_vertex : a.end_vertex;
                const int head = next.orientation == REVERSED ? b.end_vertex : b.start_vertex;
                require(tail == head, "Split produced an open face boundary");
            }
        }
}

/// Reject a split that opens a shell, leaves a wire open or moves an edge off its vertices or trims.
void validate(const BRep& result, const BRep& original, double tolerance) {

    require(result.is_valid(), "Split produced invalid BRep references");

    for (size_t s = 0; s < original.m_shells.size(); ++s)
        if (original.is_closed(static_cast<int>(s)))
            require(result.is_closed(static_cast<int>(s)), "Split would open a joined shell");

    check_wires(result);

    for (const BRepEdge& edge : result.m_edges) {
        if (edge.degenerated)
            continue;

        const NurbsCurve& world = result.m_curves_3d[edge.curve_3d_index];
        const double start = world.point_at_start().distance(result.m_vertices[edge.start_vertex].point);
        const double end = world.point_at_end().distance(result.m_vertices[edge.end_vertex].point);
        require(start <= tolerance * 4.0 && end <= tolerance * 4.0, "Split edge does not meet its vertices");

        for (const BRepCurveOnSurface& pc : edge.pcurves)
            for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
                if (ci < 0)
                    continue;

                const NurbsCurve& uv = result.m_curves_2d[ci];
                const double lo = uv.domain_start();
                const double hi = uv.domain_end();

                for (int k = 0; k <= 32; ++k) {
                    const Point q = uv.point_at(lo + (hi - lo) * k / 32.0);
                    const Point p = result.m_surfaces[pc.surface_index].point_at(q[0], q[1]);
                    require(closest(world, p).second <= std::max(tolerance, edge.tolerance) * 8.0, "Split edge and surface trim do not coincide");
                }
            }
    }
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Split
// ═══════════════════════════════════════════════════════════════════════════

std::vector<NurbsCurve> split_curve_by_curves(
    const NurbsCurve& curve,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
) {

    check_tolerance(tolerance);
    check_curve(curve);
    require(!cutters.empty(), "Select at least one cutter");
    const double lo = curve.domain_start();
    const double hi = curve.domain_end();
    std::vector<double> cuts = {lo, hi};
    bool cut_at_seam = false;
    size_t budget = WORK_LIMIT;

    for (const NurbsCurve& cutter : cutters) {
        check_curve(cutter);
        const std::vector<std::pair<double, double>> hits = intersections(curve, cutter, tolerance, budget);

        for (const std::pair<double, double>& hit : hits) {
            const double a = hit.first;

            if (std::abs(a - lo) <= (hi - lo) * EPSILON * 16.0 || std::abs(a - hi) <= (hi - lo) * EPSILON * 16.0)
                cut_at_seam = true;

            cuts.push_back(a);
        }
    }

    cuts = unique_parameters(cuts, lo, hi);

    if (cuts.size() == 2)
        return {curve};

    std::vector<NurbsCurve> result;

    for (size_t i = 1; i < cuts.size(); ++i)
        result.push_back(interval(curve, cuts[i - 1], cuts[i]));

    if (curve.is_closed() && result.size() > 1 && !cut_at_seam) {
        const std::vector<NurbsCurve> joined = NurbsCurve::join({result.back(), result.front()}, tolerance);
        require(joined.size() == 1, "Cannot join the uncut seam of a closed curve");
        result.front() = joined[0];
        result.pop_back();
    }

    return result;
}

BRep split_brep_face_by_curves(
    const BRep& brep,
    int face_index,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
) {

    check_tolerance(tolerance);
    require(brep.is_valid(), "Split requires a valid BRep");
    require(face_index >= 0 && face_index < brep.face_count(), "Select one BRep face to split");
    require(!cutters.empty(), "Select at least one cutter");
    const BRepFace& face = brep.m_faces[face_index];
    const NurbsSurface& surface = brep.m_surfaces[face.surface_index];
    check_surface(surface);

    double u0 = 0.0;
    double u1 = 0.0;
    double v0 = 0.0;
    double v1 = 0.0;
    std::tie(u0, u1) = surface.domain(0);
    std::tie(v0, v1) = surface.domain(1);
    const Point origin = surface.point_at(u0, v0);
    const double scale = std::max(
        origin.distance(surface.point_at(u1, v0)) / (u1 - u0),
        origin.distance(surface.point_at(u0, v1)) / (v1 - v0)
    );
    require(scale > EPSILON, "Cannot split a degenerate surface domain");
    const double uv_tolerance = tolerance / scale;

    std::vector<Source> sources;
    const std::vector<std::vector<Point>> original_loops = boundary_loops(brep, face_index, uv_tolerance, sources);

    for (const NurbsCurve& cutter : cutters) {
        check_curve(cutter);

        for (const NurbsCurve& uv : pullback(surface, cutter, tolerance))
            sources.push_back({-1, cutter, uv});
    }

    const std::vector<std::vector<std::vector<Run>>> regions = arrange(sources, original_loops, uv_tolerance);

    if (regions.size() < 2)
        return brep;

    BRep result = brep;
    std::vector<Piece> pieces;
    std::vector<std::vector<BRepRef>> new_wires;

    for (const std::vector<std::vector<Run>>& region : regions) {
        std::vector<BRepRef> wires;

        for (const std::vector<Run>& loop : region) {
            std::vector<BRepRef> refs;

            for (const Run& run : loop)
                refs.push_back(add_run_edge(result, pieces, brep, face.surface_index, sources, run, tolerance));

            wires.push_back({result.add_wire(refs), FORWARD});
        }

        new_wires.push_back(wires);
    }

    replace_wires(result, brep, sources, pieces);
    add_faces(result, face, face_index, new_wires);
    validate(result, brep, tolerance);
    return result;
}

BRep split_surface_by_curves(
    const NurbsSurface& surface,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
) {

    check_tolerance(tolerance);
    check_surface(surface);
    BRep result;
    const int si = result.add_surface(surface);
    double u0 = 0.0;
    double u1 = 0.0;
    double v0 = 0.0;
    double v1 = 0.0;
    std::tie(u0, u1) = surface.domain(0);
    std::tie(v0, v1) = surface.domain(1);
    const std::vector<Point> uv = {
        Point(u0, v0, 0.0),
        Point(u1, v0, 0.0),
        Point(u1, v1, 0.0),
        Point(u0, v1, 0.0),
    };
    const std::array<double, 4> at = {v0, u1, v1, u0};
    std::vector<BRepRef> edges;

    for (int i = 0; i < 4; ++i) {
        NurbsCurve curve = surface.iso_curve(i % 2, at[i]);

        if (i >= 2)
            require(curve.reverse(), "Kernel refused to reverse a natural boundary");

        const int a = vertex(result, curve.point_at_start(), tolerance);
        const int b = vertex(result, curve.point_at_end(), tolerance);
        require(a != b, "Closed or pole boundaries need a BRep with explicit seam topology");
        const int edge = result.add_edge(result.add_curve_3d(curve), a, b, tolerance);
        const NurbsCurve pc = NurbsCurve::create(false, 1, {uv[i], uv[(i + 1) % 4]});
        result.add_pcurve(edge, si, result.add_curve_2d(pc));
        edges.push_back({edge, FORWARD});
    }

    result.add_face(si, {{result.add_wire(edges), FORWARD}});
    return split_brep_face_by_curves(result, 0, cutters, tolerance);
}

std::vector<Line> split_line_by_curves(
    const Line& line,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
) {

    const NurbsCurve curve = NurbsCurve::create(false, 1, {line.point_at(0), line.point_at(1)});
    std::vector<Line> result;

    for (const NurbsCurve& piece : split_curve_by_curves(curve, cutters, tolerance)) {
        Line next = Line::from_points(piece.point_at_start(), piece.point_at_end());
        next.name = line.name;
        next.width = line.width;
        next.dash = line.dash;
        next.linecolor = line.linecolor;
        result.push_back(std::move(next));
    }

    return result;
}

std::vector<Polyline> split_polyline_by_curves(
    const Polyline& polyline,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
) {

    const NurbsCurve curve = NurbsCurve::create(false, 1, polyline.get_points());
    std::vector<Polyline> result;

    for (const NurbsCurve& piece : split_curve_by_curves(curve, cutters, tolerance)) {
        std::vector<Point> points;

        for (double t : piece.get_span_vector())
            points.push_back(piece.point_at(t));

        Polyline next(points);
        next.name = polyline.name;
        next.width = polyline.width;
        next.dash = polyline.dash;
        next.linecolor = polyline.linecolor;
        result.push_back(std::move(next));
    }

    return result;
}

} // namespace session_cpp::simple_split
