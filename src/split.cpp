#include "split.h"
#include "split_event_ownership.h"
#include "brep_acceptance.h"
#include "brep_face_boundary.h"
#include "brep_classify.h"
#include "closest.h"
#include "curve_intersection.h"
#include "curve_self_intersection.h"
#include "curve_surface_intersection.h"
#include "nurbs_bounds.h"
#include "nurbs_interval.h"
#include "nurbs_coincidence.h"
#include "spatial_rtree.h"
#include "line.h"
#include "polyline.h"
#include "tolerance.h"
#include "vector.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>

namespace session_cpp::split {

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
    int section = -1;
    int first_vertex = -1, last_vertex = -1;
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
    std::vector<double> events; // Genuine intersection and overlap parameters.
    std::vector<ArrangementPointIncidence> point_incidence; // Paired original raw point identities.
    std::vector<double> nonpoint_events; // Self/overlap events are not boundary-point aliases.
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
    std::vector<bool> events; // Vertices retained by intersection or overlap events.
    std::vector<std::vector<ArrangementPointIncidence>> point_incidence;
    std::vector<bool> nonpoint_events;
    std::vector<ArrangementVertexOwnership> vertex_owned_intervals;
};

/// Closed loop of runs with its sampled polygon.
struct Cycle {
    double area; // Signed area in parameter space.
    std::vector<Run> loop; // Runs around the loop.
    std::vector<NurbsCurve> curves; // Pieces around the loop.
    std::vector<Point> points; // Sampled polygon, for areas and diagnostics only.
    std::vector<size_t> edges; // Undirected graph edges, sorted by identity.
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
/// True when outward control bounds lie inside the closed chord's tolerance capsule.
bool flat(const NurbsCurve& curve, double tolerance) {

    const Point a = curve.point_at_start();
    const Point b = curve.point_at_end();
    const Vector v = b - a;
    const double length2 = v.dot(v);

    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point p = curve.get_cv(i);
        const double t = length2 > 0.0 ? std::clamp((p - a).dot(v) / length2, 0.0, 1.0) : 0.0;
        const double* cv = curve.cv(i);
        const nurbs_interval::Interval weight(curve.weight(i));
        nurbs_interval::Interval squared;
        for (int d = 0; d < 3; ++d) {
            const auto control = nurbs_interval::Interval(d < curve.dimension() ? cv[d] : 0.0) / weight;
            const auto chord = nurbs_interval::Interval(a[d]) + nurbs_interval::Interval(t) *
                (nurbs_interval::Interval(b[d]) - nurbs_interval::Interval(a[d]));
            const nurbs_interval::Interval distance(nurbs_interval::magnitude(control - chord));
            squared = squared + distance * distance;
        }
        const auto allowed = nurbs_interval::Interval(tolerance) * nurbs_interval::Interval(tolerance);
        if (!nurbs_interval::finite(squared) || squared.hi > allowed.lo)
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

/// Area between a piece inside one knot span and its chord, by 5-point Gauss-Legendre; zero for a line.
double chord_area(const NurbsCurve& piece) {

    if (piece.degree() < 2)
        return 0.0;
    constexpr std::array<std::array<double, 2>, 5> NODES = {{
        {-0.9061798459386640, 0.2369268850561891},
        {-0.5384693101056831, 0.4786286704993665},
        {0.0, 0.5688888888888889},
        {0.5384693101056831, 0.4786286704993665},
        {0.9061798459386640, 0.2369268850561891}
    }};
    const Point start = piece.point_at_start();
    const double lo = piece.domain_start();
    const double hi = piece.domain_end();
    double area = 0.0;
    for (const std::array<double, 2>& node : NODES) {
        std::array<xyz, 3> d{};
        piece.jet(0.5 * (lo + hi + node[0] * (hi - lo)), 1, d);
        area += node[1] * ((d[0][0] - start[0]) * d[1][1] - (d[0][1] - start[1]) * d[1][0]);
    }
    return 0.25 * (hi - lo) * area;
}

/// Points sampling a curve until each piece is flat within a share of its size, adding the area left under the chords.
std::vector<Point> polygon(const NurbsCurve& curve, double tolerance, double& chords) {

    tolerance = std::max(tolerance, Bounds(curve).diagonal() * 1e-4);
    const std::vector<double> spans = curve.get_span_vector();
    std::vector<Part> work;

    for (size_t i = spans.size(); i > 1; --i)
        work.push_back({interval(curve, spans[i - 2], spans[i - 1]), 0});

    std::vector<Point> result;
    size_t visited = 0;

    while (!work.empty()) {
        ++visited;
        if (visited > WORK_LIMIT)
            throw SamplingError(curve, work.back().curve, tolerance, visited);
        const Part part = std::move(work.back());
        work.pop_back();

        if (flat(part.curve, tolerance * 0.25)) {
            result.push_back(part.curve.point_at_start());
            chords += chord_area(part.curve);
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

/// Even-odd test in the xy plane: a piece is cut only where the +x ray meets its control box, and a piece all past the point crosses by its ends alone.
bool inside(const Point& p, const std::vector<NurbsCurve>& loop, double tolerance) {

    bool result = false;
    std::vector<Part> work;
    std::vector<const NurbsCurve*> ends;

    for (const NurbsCurve& curve : loop) {
        ends.push_back(&curve);
        const std::vector<double> spans = curve.get_span_vector();
        for (size_t i = 1; i < spans.size(); ++i)
            work.push_back({interval(curve, spans[i - 1], spans[i]), 0});
    }

    const double height = brep_classify::clear_ray_height(ends, p[1], tolerance);
    size_t visited = 0;

    while (!work.empty()) {
        require(++visited <= WORK_LIMIT, "Trim loop test exceeds the bounded workload");
        const Part part = std::move(work.back());
        work.pop_back();

        const Bounds box(part.curve);
        if (box.lo[1] > height || box.hi[1] <= height || box.hi[0] < p[0])
            continue;

        const Point a = part.curve.point_at_start();
        const Point b = part.curve.point_at_end();
        const bool across = (a[1] > height) != (b[1] > height);

        if (box.lo[0] > p[0]) {
            result = result != across;
            continue;
        }

        if (flat(part.curve, tolerance * 0.25)) {
            if (across && p[0] < (b[0] - a[0]) * (height - a[1]) / (b[1] - a[1]) + a[0])
                result = !result;
            continue;
        }

        require(part.depth < 40, "Trim loop test exceeds parameter precision");
        const double lo = part.curve.domain_start();
        const double hi = part.curve.domain_end();
        const double mid = (lo + hi) * 0.5;
        work.push_back({interval(part.curve, mid, hi), part.depth + 1});
        work.push_back({interval(part.curve, lo, mid), part.depth + 1});
    }

    return result;
}

/// True inside the first loop and outside every hole loop.
bool inside_loops(const Point& p, const std::vector<std::vector<NurbsCurve>>& loops, double tolerance) {

    if (loops.empty() || !inside(p, loops[0], tolerance))
        return false;

    for (size_t i = 1; i < loops.size(); ++i)
        if (inside(p, loops[i], tolerance))
            return false;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Trim arrangement
// ═══════════════════════════════════════════════════════════════════════════
/// Outward bounds of the original positive-rational basis controls active on a knot span.
std::pair<std::array<double, 2>, std::array<double, 2>> span_bounds(const NurbsCurve& curve, double parameter) {
    const double infinity = std::numeric_limits<double>::infinity();
    std::array<double, 2> lo = {infinity, infinity}, hi = {-infinity, -infinity};
    const int first = nurbsknot::find_span(curve.order(), curve.cv_count(), curve.m_nurbsknot, parameter);
    for (int i = first; i < first + curve.order(); ++i) {
        const Point point = curve.get_cv(i);
        for (int d = 0; d < 2; ++d) {
            lo[d] = std::min(lo[d], std::nextafter(point[d], -infinity));
            hi[d] = std::max(hi[d], std::nextafter(point[d], infinity));
            require(std::isfinite(lo[d]) && std::isfinite(hi[d]), "Face split bounds exceed floating-point resolution");
        }
    }
    return {lo, hi};
}

std::vector<std::pair<size_t, size_t>> span_candidates(
    const std::vector<Source>& sources,
    const std::vector<Span>& spans,
    double tolerance,
    ArrangementDiagnostic* diagnostic
) {
    SpatialRTree<size_t, double, 2> tree;
    std::vector<std::pair<size_t, size_t>> pairs;
    const double infinity = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < spans.size(); ++i) {
        const auto bounds = span_bounds(sources[spans[i].source].uv, spans[i].a);
        std::array<double, 2> lo, hi;
        for (int d = 0; d < 2; ++d) {
            lo[d] = std::nextafter(bounds.first[d] - tolerance, -infinity);
            hi[d] = std::nextafter(bounds.second[d] + tolerance, infinity);
        }
        if (diagnostic) {
            const Source& source = sources[spans[i].source];
            diagnostic->span_boxes.push_back({spans[i].source, source.edge, source.section,
                spans[i].a, spans[i].b, bounds.first, bounds.second, lo, hi});
        }
        tree.search(lo.data(), hi.data(), [&](size_t j) {
            require(pairs.size() < WORK_LIMIT, "Face split candidates exceed the bounded workload");
            pairs.emplace_back(j, i);
            return true;
        });
        tree.insert(bounds.first.data(), bounds.second.data(), i);
    }
    std::sort(pairs.begin(), pairs.end());
    if (diagnostic)
        diagnostic->candidate_pairs = pairs;
    return pairs;
}

/// True when the whole original interval lies inside one vertex tolerance ball.
bool vertex_interval(const NurbsCurve& curve, double a, double b, const Point& point, double tolerance) {
    const auto allowed = nurbs_interval::Interval(tolerance) * nurbs_interval::Interval(tolerance);
    const auto outside = [&](double parameter) {
        const auto data = nurbs_interval::curve_controls(curve, {parameter, parameter},
            {point[0], point[1], point[2]});
        if (data.empty())
            return false;
        std::array<nurbs_interval::Interval, 3> endpoint;
        bool first = true;
        for (const auto& cv : data) {
            if (!(cv[3].lo > 0.0))
                return false;
            for (int d = 0; d < 3; ++d) {
                const auto coordinate = cv[d] / cv[3];
                if (!nurbs_interval::finite(coordinate))
                    return false;
                endpoint[d] = first ? coordinate : nurbs_interval::interval_hull(endpoint[d], coordinate);
            }
            first = false;
        }
        nurbs_interval::Interval lower;
        for (const auto coordinate : endpoint) {
            const double nearest = coordinate.lo > 0.0 ? coordinate.lo : coordinate.hi < 0.0 ? -coordinate.hi : 0.0;
            const nurbs_interval::Interval distance(nearest);
            lower = lower + distance * distance;
        }
        return nurbs_interval::finite(lower) && lower.lo > allowed.hi;
    };
    if (outside(b) || outside(a))
        return false;
    nurbs_interval::CurveBounds bounds;
    if (!nurbs_interval::curve_bounds(curve, {std::min(a, b), std::max(a, b)},
        {point[0], point[1], point[2]}, bounds))
        return false;
    nurbs_interval::Interval squared;
    for (const nurbs_interval::Interval coordinate : bounds.point) {
        const nurbs_interval::Interval distance(nurbs_interval::magnitude(coordinate));
        squared = squared + distance * distance;
    }
    return nurbs_interval::finite(squared) && squared.hi <= allowed.lo;
}

/// Preserve both parameters of every original-curve self event in the affected knot spans.
void self_events(std::vector<Span>& spans, const std::vector<Source>& sources, double tolerance) {
    for (size_t source = 0; source < sources.size(); ++source) {
        const auto events = curve_self_intersection::intersect(sources[source].uv, tolerance);
        if (!events.complete)
            throw SelfIntersectionError(sources[source].world, sources[source].uv, events,
                source, sources[source].edge, sources[source].section, tolerance);
        for (const auto* hits : {&events.points, &events.endpoints})
            for (const curve_intersection::Hit& hit : *hits)
                for (Span& span : spans)
                    if (span.source == source)
                        for (double parameter : {hit.a, hit.b})
                            if (parameter >= span.a && parameter <= span.b) {
                                span.cuts.push_back(parameter);
                                span.events.push_back(parameter);
                                const auto& owner=sources[source];
                                const detail::EventSource original{&owner.uv,owner.edge>=0,owner.first_vertex,owner.last_vertex};
                                if (hits==&events.endpoints && detail::closed_endpoint_pair(original,hit.a,hit.b))
                                    span.point_incidence.push_back({source,source,parameter,
                                        parameter==hit.a?hit.b:hit.a,owner.uv.domain_start(),owner.uv.domain_end(),
                                        owner.uv.domain_start(),owner.uv.domain_end()});
                                else
                                    span.nonpoint_events.push_back(parameter);
                            }
    }
}

/// Knot spans of every source, cut at their mutual intersections.
// Only raw parameter events escape this caller; local hit points are unused.
curve_intersection::Result parameter_events(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    const Point origin = a.get_cv(0);
    const auto translate = [&](NurbsCurve& curve) {
        if (curve.dimension() < 1 || curve.dimension() > 3)
            return false;
        for (int i = 0; i < curve.cv_count(); ++i) {
            double* cv = curve.cv(i);
            const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
            for (int d = 0; d < curve.dimension(); ++d) {
                const double product = origin[d] * weight;
                double local;
                if (!nurbs_coincidence::exact_products_equal(origin[d], weight, product, 1.0)
                    || !nurbs_coincidence::exact_difference(cv[d], product, local))
                    return false;
                cv[d] = local;
            }
        }
        return true;
    };
    NurbsCurve first = a, second = b;
    if (!translate(first) || !translate(second))
        return curve_intersection::intersect(a, b, tolerance);
    return curve_intersection::intersect(first, second, tolerance);
}

std::vector<Span> compute_spans(const std::vector<Source>& sources, double tolerance, ArrangementDiagnostic* diagnostic) {

    std::vector<Span> spans;

    for (size_t si = 0; si < sources.size(); ++si) {
        std::vector<double> knots = sources[si].uv.get_span_vector();

        if (knots.size() == 2 && sources[si].uv.is_closed()) {
            const double lo = knots.front();
            const double hi = knots.back();
            knots = {lo, lo + (hi - lo) * 0.25, (lo + hi) * 0.5, lo + (hi - lo) * 0.75, hi};
        }

        for (size_t i = 1; i < knots.size(); ++i) {
            require(spans.size() < WORK_LIMIT, "Face split spans exceed the bounded workload");
            spans.push_back({si, knots[i - 1], knots[i], {knots[i - 1], knots[i]}, interval(sources[si].uv, knots[i - 1], knots[i])});
        }
    }

    require(!spans.empty(), "Face split requires nonempty spans");
    self_events(spans, sources, tolerance);
    for (const auto& [i, j] : span_candidates(sources, spans, tolerance, diagnostic)) {
        if (spans[i].source == spans[j].source)
            continue;
        const curve_intersection::Result events = parameter_events(spans[i].curve, spans[j].curve, tolerance);
        require(events.complete, events.failure);
        if (diagnostic && (sources[spans[i].source].edge < 0) != (sources[spans[j].source].edge < 0)) {
            ArrangementPairDiagnostic row;
            row.first_source = spans[i].source;
            row.second_source = spans[j].source;
            row.first = spans[i].curve;
            row.second = spans[j].curve;
            row.overlaps = events.overlaps.size();
            for (const auto& hit : events.points) {
                row.first_hits.push_back(hit.a);
                row.second_hits.push_back(hit.b);
            }
            diagnostic->boundary_cut_pairs.push_back(std::move(row));
        }
        for (const curve_intersection::Hit& hit : events.points) {
            spans[i].point_incidence.push_back({spans[i].source,spans[j].source,hit.a,hit.b,
                spans[i].a,spans[i].b,spans[j].a,spans[j].b});
            spans[j].point_incidence.push_back({spans[j].source,spans[i].source,hit.b,hit.a,
                spans[j].a,spans[j].b,spans[i].a,spans[i].b});
            spans[i].cuts.push_back(hit.a);
            spans[j].cuts.push_back(hit.b);
            if (spans[i].source != spans[j].source || hit.a != hit.b) {
                spans[i].events.push_back(hit.a);
                spans[j].events.push_back(hit.b);
            }
        }
        for (const curve_intersection::Overlap& overlap : events.overlaps) {
            spans[i].cuts.push_back(overlap.a0);
            spans[i].cuts.push_back(overlap.a1);
            spans[j].cuts.push_back(overlap.b0);
            spans[j].cuts.push_back(overlap.b1);
            spans[i].events.insert(spans[i].events.end(), {overlap.a0, overlap.a1});
            spans[j].events.insert(spans[j].events.end(), {overlap.b0, overlap.b1});
            spans[i].nonpoint_events.insert(spans[i].nonpoint_events.end(), {overlap.a0,overlap.a1});
            spans[j].nonpoint_events.insert(spans[j].nonpoint_events.end(), {overlap.b0,overlap.b1});
        }
    }

    return spans;
}

/// Cut parameters of a span, snapped to its ends and deduplicated.
std::vector<double> span_cuts(const Span& span, const NurbsCurve& curve, double tolerance) {

    std::vector<double> cuts = span.cuts;

    for (double& t : cuts) {
        t = std::clamp(t, span.a, span.b);
        if (vertex_interval(curve, span.a, t, curve.point_at(span.a), tolerance))
            t = span.a;
        else if (vertex_interval(curve, t, span.b, curve.point_at(span.b), tolerance))
            t = span.b;
    }

    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    return cuts;
}

/// Index of the graph vertex at a point, added when none lies within tolerance.
size_t node(Graph& graph, std::vector<Point>& vertices, const Point& p, double tolerance) {

    for (size_t i = 0; i < vertices.size(); ++i)
        if (p.distance(vertices[i]) <= tolerance * 4.0)
            return i;

    vertices.push_back(p);
    graph.outgoing.emplace_back();
    graph.events.push_back(false);
    graph.point_incidence.emplace_back();
    graph.nonpoint_events.push_back(false);
    return vertices.size() - 1;
}

/// First nonzero one-sided direction of a run leaving its start vertex.
Vector run_direction(const std::vector<Source>& sources, const Run& run) {
    const NurbsCurve part = interval(sources[run.source].uv, std::min(run.a, run.b), std::max(run.a, run.b));
    const bool forward = run.b > run.a;
    const double parameter = forward ? part.domain_start() : part.domain_end();
    const std::vector<Vector> derivatives = part.evaluate(parameter, part.degree());
    for (size_t order = 1; order < derivatives.size(); ++order) {
        const Vector& direction = derivatives[order];
        const double length = std::hypot(direction[0], direction[1]);
        require(std::isfinite(length), "Trim occurrence has a nonfinite endpoint direction");
        if (length > 0.0) {
            const double sign = forward || order % 2 == 0 ? 1.0 : -1.0;
            return direction * sign;
        }
    }
    throw std::invalid_argument("Trim occurrence has an unresolved endpoint direction");
}

using SectorInterval = nurbs_interval::Interval;
using SectorControl = nurbs_interval::Homogeneous;

SectorInterval sector_projection(const SectorControl& cv, const Vector& direction) {
    return cv[0] * SectorInterval(direction[0]) + cv[1] * SectorInterval(direction[1]);
}

bool sector_identical(SectorInterval a, SectorInterval b) {
    return a.lo == a.hi && b.lo == b.hi && a.lo == b.lo;
}

bool sector_origin(const SectorControl& cv, Point& point) {
    if (!(cv[3].lo > 0) || !nurbs_interval::finite(cv[3]))
        return false;
    for (int d = 0; d < 2; ++d) {
        const auto value = cv[d] / cv[3];
        if (!nurbs_interval::finite(value) || value.lo != value.hi)
            return false;
        point[d] = value.lo;
    }
    point[2] = 0;
    return true;
}

bool sector_monotone(const std::vector<SectorControl>& data, const Vector& tangent) {
    bool strict = false;
    for (size_t i = 1; i < data.size(); ++i) {
        const auto& a = data[i - 1];
        const auto& b = data[i];
        if (!(a[3].lo > 0 && b[3].lo > 0) || !nurbs_interval::finite(a[3]) || !nurbs_interval::finite(b[3]))
            return false;
        const auto first = sector_projection(a, tangent);
        const auto second = sector_projection(b, tangent);
        const bool identical = sector_identical(a[0], b[0]) && sector_identical(a[1], b[1]) && sector_identical(a[3], b[3]);
        SectorInterval difference;
        if (!identical && !(sector_identical(first, second) && sector_identical(a[3], b[3])))
            difference = second * a[3] - first * b[3];
        if (!nurbs_interval::finite(difference) || difference.lo < 0)
            return false;
        strict = strict || difference.lo > 0;
    }
    return strict;
}

int sector_sign(const std::vector<SectorControl>& data, const Point& origin, const Vector& direction) {
    bool positive = true;
    bool negative = true;
    bool strict_positive = false;
    bool strict_negative = false;
    for (auto cv : data) {
        for (int d = 0; d < 2; ++d)
            cv[d] = cv[d] - SectorInterval(origin[d]) * cv[3];
        const auto value = sector_projection(cv, direction);
        if (!nurbs_interval::finite(value))
            return 2;
        positive = positive && value.lo >= 0;
        negative = negative && value.hi <= 0;
        strict_positive = strict_positive || value.lo > 0;
        strict_negative = strict_negative || value.hi < 0;
    }
    if (positive && negative)
        return 0;
    if (positive && strict_positive)
        return 1;
    if (negative && strict_negative)
        return -1;
    return 2;
}

/// Prove a tangent cluster's side order inside a supplied open cone of width less than pi.
bool sector_sides(
    const std::vector<Source>& sources,
    const Graph& graph,
    const std::vector<size_t>& cluster,
    const Point& origin,
    const Vector& tangent,
    const Vector& left,
    const Vector& right,
    size_t& budget,
    std::vector<size_t>& ordered
) {
    const auto cross = [](const Vector& a, const Vector& b) {
        return SectorInterval(a[0]) * SectorInterval(b[1]) - SectorInterval(a[1]) * SectorInterval(b[0]);
    };
    if (cluster.empty() || cluster.size() > 3 || !(cross(left, right).lo > 0) ||
        !(cross(left, tangent).lo > 0) || !(cross(tangent, right).lo > 0))
        return false;
    std::array<size_t, 3> slots;
    slots.fill(std::numeric_limits<size_t>::max());
    for (size_t edge : cluster) {
        Run prefix = graph.edges[edge].run;
        const auto& curve = sources[prefix.source].uv;
        int side = 2;
        for (int depth = 0; depth <= 96; ++depth) {
            if (budget == 0)
                return false;
            --budget;
            auto data = nurbs_interval::curve_controls(curve,
                {std::min(prefix.a, prefix.b), std::max(prefix.a, prefix.b)}, {0, 0, 0});
            if (data.empty())
                return false;
            if (prefix.a > prefix.b)
                std::reverse(data.begin(), data.end());
            Point start;
            if (!sector_origin(data.front(), start))
                return false;
            if (start[0] != origin[0] || start[1] != origin[1])
                return false;
            if (sector_monotone(data, tangent) &&
                sector_sign(data, origin, {-left[1], left[0], 0}) == 1 &&
                sector_sign(data, origin, {right[1], -right[0], 0}) == 1) {
                side = sector_sign(data, origin, {-tangent[1], tangent[0], 0});
                if (side != 2)
                    break;
            }
            const double next = std::midpoint(prefix.a, prefix.b);
            if (next == prefix.a || next == prefix.b)
                return false;
            prefix.b = next;
        }
        if (side == 2 || slots[side + 1] != std::numeric_limits<size_t>::max())
            return false;
        slots[side + 1] = edge;
    }
    ordered.clear();
    for (size_t edge : slots)
        if (edge != std::numeric_limits<size_t>::max())
            ordered.push_back(edge);
    return true;
}

struct SectorCone {
    Vector tangent;
    Vector left;
    Vector right;
};

SectorInterval sector_cross(const Vector& a, const Vector& b) {
    return SectorInterval(a[0]) * SectorInterval(b[1]) - SectorInterval(a[1]) * SectorInterval(b[0]);
}

double sector_angle(const Vector& direction) {
    const double value = std::atan2(direction[1], direction[0]);
    return value == std::acos(-1.0) ? -value : value == 0 ? 0 : value;
}

bool sector_ambiguous(const Vector& a, const Vector& b) {
    if (sector_angle(a) == sector_angle(b))
        return true;
    const auto cross = sector_cross(a, b);
    const auto dot = SectorInterval(a[0]) * SectorInterval(b[0]) + SectorInterval(a[1]) * SectorInterval(b[1]);
    // A tangency found to tolerance leaves two tangents apart by about the square root of it, never exactly equal.
    const double size = std::hypot(a[0], a[1]) * std::hypot(b[0], b[1]);
    return dot.lo > 0 && ((cross.lo <= 0 && cross.hi >= 0) || std::abs(cross.lo) <= 1e-5 * size);
}

/// A tangent cluster counter-clockwise by the chords of its runs a short way out: curves that leave a corner together part there.
bool sector_chords(const std::vector<Source>& sources, const Graph& graph, std::vector<size_t>& cluster, const Vector& direction) {
    const auto at = [&](size_t edge, double fraction) {
        const Run& run = graph.edges[edge].run;
        return sources[run.source].uv.point_at(run.a + (run.b - run.a) * fraction);
    };
    double reach = std::numeric_limits<double>::infinity();
    for (size_t edge : cluster)
        reach = std::min(reach, at(edge, 0.5).distance(at(edge, 0.0)) * 0.5);
    const double scale = std::hypot(direction[0], direction[1]);
    if (!(reach > 0) || !(scale > 0))
        return false;
    std::vector<std::pair<double, size_t>> sides;
    for (size_t edge : cluster) {
        const Point start = at(edge, 0.0);
        double fraction = 0.5;
        while (fraction > 1e-12 && at(edge, fraction).distance(start) > reach)
            fraction *= 0.5;
        const Vector chord = at(edge, fraction) - start;
        sides.push_back({(direction[0] * chord[1] - direction[1] * chord[0]) / (scale * chord.magnitude()), edge});
    }
    std::sort(sides.begin(), sides.end());
    for (size_t i = 1; i < sides.size(); ++i)
        if (sides[i].first - sides[i - 1].first <= 1e-12)
            return false;
    for (size_t i = 0; i < sides.size(); ++i)
        cluster[i] = sides[i].second;
    return true;
}

bool sector_cones(const std::vector<SectorCone>& cones, size_t& budget, std::vector<size_t>& order) {
    struct Boundary {
        Vector ray;
        size_t cone;
        bool right;
    };
    std::vector<Boundary> boundaries;
    for (size_t i = 0; i < cones.size(); ++i) {
        if (!(sector_cross(cones[i].left, cones[i].right).lo > 0))
            return false;
        boundaries.push_back({cones[i].left, i, false});
        boundaries.push_back({cones[i].right, i, true});
    }
    const auto upper = [](const Vector& ray) {
        return ray[1] > 0 || (ray[1] == 0 && ray[0] > 0);
    };
    for (size_t i = 0; i < boundaries.size(); ++i)
        for (size_t j = i + 1; j < boundaries.size(); ++j) {
            if (budget == 0)
                return false;
            --budget;
            if (upper(boundaries[i].ray) == upper(boundaries[j].ray)) {
                const auto cross = sector_cross(boundaries[i].ray, boundaries[j].ray);
                if (!(cross.lo > 0 || cross.hi < 0))
                    return false;
            }
        }
    std::sort(boundaries.begin(), boundaries.end(), [&](const Boundary& a, const Boundary& b) {
        const bool first = upper(a.ray);
        const bool second = upper(b.ray);
        return first != second ? first : sector_cross(a.ray, b.ray).lo > 0;
    });
    order.clear();
    for (size_t i = 0; i < boundaries.size(); ++i) {
        const auto& boundary = boundaries[i];
        if (boundary.right)
            continue;
        const auto& next = boundaries[(i + 1) % boundaries.size()];
        if (!next.right || boundary.cone != next.cone)
            return false;
        order.push_back(boundary.cone);
    }
    return order.size() == cones.size();
}

/// Replace an ambiguous fan only after every branch has a disjoint original-prefix sector proof.
void order_sector_fan(
    const std::vector<Source>& sources,
    const Graph& graph,
    const std::vector<Vector>& directions,
    size_t& budget,
    std::vector<size_t>& choices
) {
    if (choices.size() < 2)
        return;
    std::vector<size_t> proposal = choices;
    std::sort(proposal.begin(), proposal.end(), [&](size_t a, size_t b) {
        return sector_angle(directions[a]) < sector_angle(directions[b]);
    });
    std::vector<std::vector<size_t>> clusters;
    for (size_t edge : proposal) {
        if (clusters.empty() || !sector_ambiguous(directions[clusters.back().back()], directions[edge]))
            clusters.push_back({});
        clusters.back().push_back(edge);
    }
    if (clusters.size() > 1 && sector_ambiguous(directions[clusters.back().back()], directions[clusters.front().front()])) {
        const auto last = std::move(clusters.back());
        clusters.pop_back();
        clusters.front().insert(clusters.front().end(), last.begin(), last.end());
    }
    if (clusters.size() == choices.size())
        return;
    const Run& initial = graph.edges[proposal.front()].run;
    auto data = nurbs_interval::curve_controls(sources[initial.source].uv,
        {std::min(initial.a, initial.b), std::max(initial.a, initial.b)}, {0, 0, 0});
    require(!data.empty(), "Tangent fan has no original-span endpoint enclosure");
    Point origin;
    require(sector_origin(initial.a < initial.b ? data.front() : data.back(), origin),
        "Tangent fan has an unresolved common endpoint");
    const double pi = std::acos(-1.0);
    const auto gap = [pi](double first, double second) {
        const double value = second - first;
        return value > 0 ? value : value + 2 * pi;
    };
    std::vector<SectorCone> cones;
    for (size_t i = 0; i < clusters.size(); ++i) {
        const Vector& direction = directions[clusters[i].front()];
        const double scale = std::max(std::abs(direction[0]), std::abs(direction[1]));
        require(std::isfinite(scale) && scale > 0, "Tangent fan has an invalid candidate direction");
        const Vector tangent(direction[0] / scale, direction[1] / scale, 0);
        const double angle = sector_angle(direction);
        const double previous = sector_angle(directions[clusters[(i + clusters.size() - 1) % clusters.size()].front()]);
        const double next = sector_angle(directions[clusters[(i + 1) % clusters.size()].front()]);
        const double width = std::min({pi * 0.25, gap(previous, angle) * 0.25, gap(angle, next) * 0.25});
        const double height = width == pi * 0.25 ? 1.0 : std::tan(width);
        require(std::isfinite(height) && height > 0, "Tangent fan has no representable candidate cone");
        const Vector offset(-height * tangent[1], height * tangent[0], 0);
        cones.push_back({tangent, tangent - offset, tangent + offset});
    }
    std::vector<size_t> order;
    std::vector<size_t> result;
    bool proved = sector_cones(cones, budget, order);
    for (size_t i = 0; proved && i < order.size(); ++i) {
        std::vector<size_t> cluster;
        const auto& cone = cones[order[i]];
        proved = sector_sides(sources, graph, clusters[order[i]], origin, cone.tangent, cone.left, cone.right, budget, cluster);
        result.insert(result.end(), cluster.begin(), cluster.end());
    }
    if (!proved) {
        result.clear();
        for (std::vector<size_t>& cluster : clusters) {
            require(cluster.size() == 1 || sector_chords(sources, graph, cluster, directions[cluster.front()]),
                "Tangent fan has unresolved prefix order or endpoint identity");
            result.insert(result.end(), cluster.begin(), cluster.end());
        }
    }
    choices = std::move(result);
}


bool covers_interval(
    const NurbsCurve& curve,
    const std::vector<curve_intersection::Overlap>& overlaps,
    bool second,
    bool exact = false
) {
    std::vector<std::pair<double, double>> intervals;
    for (const curve_intersection::Overlap& overlap : overlaps) {
        const double a = second ? overlap.b0 : overlap.a0;
        const double b = second ? overlap.b1 : overlap.a1;
        if (exact && (!std::isfinite(a) || !std::isfinite(b)))
            return false;
        intervals.push_back({std::min(a, b), std::max(a, b)});
    }
    std::sort(intervals.begin(), intervals.end());
    double last = curve.domain_start();
    const double epsilon = exact ? 0.0 : 128.0 * std::numeric_limits<double>::epsilon()
        * std::max({std::abs(last), std::abs(curve.domain_end()), curve.domain_end() - last});
    for (const std::pair<double, double>& interval : intervals) {
        if (interval.first > last + epsilon)
            return false;
        last = std::max(last, interval.second);
    }
    return last >= curve.domain_end() - epsilon;
}

bool common_arc(
    const Graph& graph,
    const std::vector<Source>& sources,
    size_t a,
    size_t b,
    const Run& run,
    double tolerance,
    size_t& budget,
    bool boundary_only = false
) {
    for (size_t index : graph.outgoing[a]) {
        const Directed& edge = graph.edges[index];
        if (boundary_only && sources[edge.run.source].edge < 0)
            continue;
        if (edge.b != b)
            continue;
        require(budget > 0, "Common trim intervals exceed the bounded workload");
        --budget;
        const NurbsCurve first = interval(sources[run.source].uv, std::min(run.a, run.b), std::max(run.a, run.b));
        const Run& previous = edge.run;
        const NurbsCurve second = interval(sources[previous.source].uv,
            std::min(previous.a, previous.b), std::max(previous.a, previous.b));
        const curve_intersection::Result events = curve_intersection::intersect(first, second, tolerance);
        require(events.complete, events.failure);
        const bool positive = !boundary_only || std::any_of(events.overlaps.begin(), events.overlaps.end(), [](const auto& overlap) {
            return overlap.a0 != overlap.a1 && overlap.b0 != overlap.b1;
        });
        if (positive && covers_interval(first, events.overlaps, false, boundary_only) &&
            covers_interval(second, events.overlaps, true, boundary_only))
            return true;
    }
    return false;
}

std::optional<ArrangementVertexOwnership> boundary_vertex_interval(
    const Span& span,const std::vector<Source>& sources,const std::vector<detail::EventSource>& original,const Graph& graph,
    const std::vector<Point>& vertices,size_t a,size_t b,double lo,double hi,double tolerance,size_t& budget) {
    if (a!=b || !graph.events[a])
        return std::nullopt;
    const bool nonpoint=graph.nonpoint_events[a] || std::any_of(span.nonpoint_events.begin(),span.nonpoint_events.end(),
        [&](double parameter){return parameter>=lo && parameter<=hi;});
    const auto key=detail::boundary_vertex_key(span.curve,span.source,original,graph.point_incidence[a],span.events,
        nonpoint,lo,hi,vertices[a],tolerance,budget,vertex_interval);
    if (!key)
        return std::nullopt;
    ArrangementVertexOwnership receipt;
    receipt.node=a;receipt.source=span.source;receipt.boundary_source=key->first;
    receipt.source_edge=sources[span.source].edge;receipt.section=sources[span.source].section;
    receipt.first=lo;receipt.last=hi;receipt.boundary_parameter=key->second;receipt.vertex=vertices[a];
    for (double parameter:span.cuts)
        if (parameter>=lo && parameter<=hi)
            receipt.raw_parameters.push_back(parameter); // Both distinct raw aliases remain, including repeats.
    receipt.incidences=graph.point_incidence[a];
    return receipt;
}

/// Half-edge graph of the cut spans inside the original loops.
Graph compute_graph(
    const std::vector<Span>& spans,
    const std::vector<Source>& sources,
    const brep_face_boundary::Result& boundary,
    double tolerance,
    ArrangementDiagnostic* diagnostic
) {

    Graph graph;
    std::vector<Point> vertices;
    size_t common_budget = WORK_LIMIT;
    std::vector<detail::EventSource> original;
    original.reserve(sources.size());
    for (const auto& source:sources)
        original.push_back({&source.uv,source.edge>=0,source.first_vertex,source.last_vertex});

    for (const Span& span : spans)
        for (double parameter : span.events) {
            const size_t index = node(graph, vertices, sources[span.source].uv.point_at(parameter), tolerance);
            graph.events[index] = true;
        }

    for (const Span& span:spans) {
        for (const auto& incidence:span.point_incidence) {
            const size_t index=node(graph,vertices,sources[span.source].uv.point_at(incidence.parameter),tolerance);
            graph.point_incidence[index].push_back(incidence);
        }
        for (double parameter:span.nonpoint_events) {
            const size_t index=node(graph,vertices,sources[span.source].uv.point_at(parameter),tolerance);
            graph.nonpoint_events[index]=true;
        }
    }

    for (const Span& span : spans) {
        const Source& source = sources[span.source];
        const std::vector<double> cuts = span_cuts(span, source.uv, tolerance);

        for (size_t i = 1; i < cuts.size(); ++i) {
            const double lo = cuts[i - 1];
            const double hi = cuts[i];

            auto state = brep_classify::State::Inside;
            Point query;
            if (source.edge < 0) {
                query = source.uv.point_at(std::midpoint(lo, hi));
                state = brep_classify::face_uv(boundary, query, tolerance);
                if (state == brep_classify::State::Unknown)
                    throw DomainError(span.source, source.edge, source.section, lo, hi, query, state,
                        source.world.domain(), source.uv.domain(), "Cutter midpoint face classification is unresolved");
                if (state == brep_classify::State::Outside)
                    continue;
            }

            const size_t a = node(graph, vertices, source.uv.point_at(lo), tolerance);
            const size_t b = node(graph, vertices, source.uv.point_at(hi), tolerance);

            if (state == brep_classify::State::Boundary) {
                if (auto owned=boundary_vertex_interval(span,sources,original,graph,vertices,a,b,lo,hi,tolerance,common_budget)) {
                    graph.vertex_owned_intervals.push_back(std::move(*owned));
                    if (diagnostic)
                        diagnostic->vertex_owned_intervals.push_back(graph.vertex_owned_intervals.back());
                    continue; // Event-owned, NEVER described as a positive common boundary arc.
                }
                if (!common_arc(graph, sources, a, b, {span.source, lo, hi}, tolerance, common_budget, true))
                    throw DomainError(span.source, source.edge, source.section, lo, hi, query, state,
                        source.world.domain(), source.uv.domain(),
                        "Cutter midpoint is on the boundary without established overlap ownership");
                continue;
            }

            if (a == b && vertex_interval(source.uv, lo, hi, vertices[a], tolerance * 4.0))
                continue;

            if (common_arc(graph, sources, a, b, {span.source, lo, hi}, tolerance, common_budget))
                continue;

            const size_t index = graph.edges.size();
            graph.edges.push_back({b, {span.source, lo, hi}});
            graph.edges.push_back({a, {span.source, hi, lo}});
            graph.outgoing[a].push_back(index);
            graph.outgoing[b].push_back(index + 1);
        }
    }

    std::vector<Vector> directions;
    std::vector<double> angles;
    for (const Directed& edge : graph.edges) {
        directions.push_back(run_direction(sources, edge.run));
        const Vector& direction = directions.back();
        angles.push_back(std::atan2(direction[1], direction[0]));
    }
    size_t sector_budget = WORK_LIMIT;
    for (std::vector<size_t>& choices : graph.outgoing) {
        std::sort(choices.begin(), choices.end(), [&angles](size_t a, size_t b) { return angles[a] < angles[b]; });
        order_sector_fan(sources, graph, directions, sector_budget, choices);
    }

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

/// Continue one original curve across an ordinary knot vertex, retaining every pave event.
bool continuation(const Graph& graph, size_t vertex, const Run& previous, const Run& next) {
    return !graph.events[vertex] && graph.outgoing[vertex].size() == 2 && previous.source == next.source &&
        previous.b == next.a && (previous.b > previous.a) == (next.b > next.a);
}

/// Loop traced from one half-edge by turning to the previous outgoing half-edge at each vertex.
Cycle trace_cycle(
    const Graph& graph,
    const std::vector<Source>& sources,
    size_t initial,
    std::vector<bool>& used,
    double tolerance
) {

    Cycle cycle{0.0, {}, {}, {}, {}};
    double chords = 0.0;
    size_t edge = initial;

    while (!used[edge]) {
        used[edge] = true;
        cycle.edges.push_back(edge / 2);
        const Directed& item = graph.edges[edge];
        const Run& run = item.run;
        const size_t vertex = graph.edges[edge ^ 1].b;
        if (cycle.loop.empty() || !continuation(graph, vertex, cycle.loop.back(), run))
            cycle.loop.push_back(run);
        else
            cycle.loop.back().b = run.b;
        NurbsCurve part = interval(sources[run.source].uv, std::min(run.a, run.b), std::max(run.a, run.b));

        if (run.b < run.a)
            require(part.reverse(), "Kernel refused to reverse a trim fragment");

        const std::vector<Point> poly = polygon(part, tolerance, chords);
        cycle.points.insert(cycle.points.end(), poly.begin(), poly.end());
        cycle.curves.push_back(std::move(part));
        const std::vector<size_t>& options = graph.outgoing[item.b];
        const std::vector<size_t>::const_iterator at = std::find(options.begin(), options.end(), edge ^ 1);
        require(at != options.end(), "Invalid trim graph adjacency");
        const size_t slot = static_cast<size_t>(at - options.begin());
        edge = options[(slot + options.size() - 1) % options.size()];
    }

    require(edge == initial, "Invalid trim graph cycle");
    if (cycle.loop.size() > 1 && continuation(graph, graph.edges[initial ^ 1].b,
        cycle.loop.back(), cycle.loop.front())) {
        cycle.loop.front().a = cycle.loop.back().a;
        cycle.loop.pop_back();
    }
    cycle.area = signed_area(cycle.points) + chords;
    std::sort(cycle.edges.begin(), cycle.edges.end());
    return cycle;
}

/// Point eight tolerances left of a regular knot-span interior of a run.
Point left_of(const std::vector<Source>& sources, const Run& run, double tolerance) {

    const NurbsCurve& curve = sources[run.source].uv;
    const double first = std::min(run.a, run.b);
    const double last = std::max(run.a, run.b);
    const std::vector<double> spans = curve.get_span_vector();
    const double sign = run.b > run.a ? 1.0 : -1.0;
    for (size_t i = 1; i < spans.size(); ++i) {
        const double a = std::max(first, spans[i - 1]);
        const double b = std::min(last, spans[i]);
        if (a >= b)
            continue;
        const double t = std::midpoint(a, b);
        require(t > a && t < b, "Trim span has no representable interior probe");
        const Vector d = curve.evaluate(t, 1)[1];
        const double length = std::hypot(d[0], d[1]);
        require(std::isfinite(length), "Trim probe has a nonfinite tangent");
        if (length <= EPSILON)
            continue;
        const Point p = curve.point_at(t);
        return Point(p[0] - sign * d[1] / length * tolerance * 8.0, p[1] + sign * d[0] / length * tolerance * 8.0, 0.0);
    }
    throw std::invalid_argument("Cannot orient a degenerate trim fragment");
}

/// Non-degenerate graph cycles whose interior lies inside the original loops.
std::vector<Cycle> compute_cycles(
    const Graph& graph,
    const std::vector<Source>& sources,
    const std::vector<std::vector<NurbsCurve>>& original_loops,
    double tolerance,
    ArrangementDiagnostic* diagnostic
) {

    std::vector<Cycle> cycles;
    std::vector<bool> used(graph.edges.size(), false);

    for (size_t initial = 0; initial < graph.edges.size(); ++initial) {
        if (used[initial])
            continue;

        const Cycle cycle = trace_cycle(graph, sources, initial, used, tolerance);

        ArrangementCycleDiagnostic observed;
        if (diagnostic) {
            observed.points = cycle.points;
            observed.edges = cycle.edges;
            observed.area = cycle.area;
            observed.runs = cycle.loop.size();
            observed.first_source = cycle.loop[0].source;
            observed.first_parameter = cycle.loop[0].a;
            observed.last_parameter = cycle.loop[0].b;
        }
        const bool small = std::abs(cycle.area) <= tolerance * tolerance;
        bool accepted = false;
        if (!small) {
            const Point probe = left_of(sources, cycle.loop[0], tolerance);
            accepted = inside_loops(probe, original_loops, tolerance);
            if (diagnostic) {
                observed.probe_evaluated = true;
                observed.probe = probe;
            }
        }
        if (accepted)
            cycles.push_back(cycle);
        if (diagnostic) {
            observed.disposition = accepted ? "retained" : small ? "area_tolerance" : "outside_original_loops";
            diagnostic->cycles.push_back(observed);
            if (!accepted && diagnostic->first_discarded_reason.empty()) {
                diagnostic->first_discarded_reason = observed.disposition;
                diagnostic->first_discarded_uv = cycle.points;
            }
        }
    }

    return cycles;
}

/// True when two cycles use the same undirected graph edge.
bool shared_edge(const Cycle& first, const Cycle& second) {
    size_t a = 0;
    size_t b = 0;
    while (a < first.edges.size() && b < second.edges.size()) {
        if (first.edges[a] == second.edges[b])
            return true;
        if (first.edges[a] < second.edges[b])
            ++a;
        else
            ++b;
    }
    return false;
}

/// Regions as outer loops, each hole nested in the smallest outer loop around it.
std::vector<std::vector<std::vector<Run>>> nest_cycles(const std::vector<Cycle>& cycles, double tolerance, ArrangementDiagnostic* diagnostic) {

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
            const bool shared = shared_edge(cycle, outer);
            const bool eligible = !shared && outer.area > std::abs(cycle.area) + tolerance * tolerance && outer.area < smallest;
            const bool contained = eligible && inside(cycle.points[0], outer.curves, tolerance);
            if (diagnostic)
                diagnostic->parent_tests.push_back({static_cast<size_t>(&cycle - cycles.data()), positive[i], shared, eligible, eligible, contained});
            if (contained) {
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
    const std::vector<std::vector<NurbsCurve>>& original_loops,
    const brep_face_boundary::Result& boundary,
    double tolerance,
    ArrangementDiagnostic* diagnostic
) {

    const std::vector<Span> spans = compute_spans(sources, tolerance, diagnostic);
    const Graph graph = compute_graph(spans, sources, boundary, tolerance, diagnostic);
    if (diagnostic)
        diagnostic->vertex_owned_intervals=graph.vertex_owned_intervals;
    const std::vector<Cycle> cycles = compute_cycles(graph, sources, original_loops, tolerance, diagnostic);
    const std::vector<std::vector<std::vector<Run>>> regions = nest_cycles(cycles, tolerance, diagnostic);
    if (diagnostic) {
        diagnostic->sources = sources.size();
        diagnostic->spans = spans.size();
        diagnostic->nodes = graph.outgoing.size();
        diagnostic->half_edges = graph.edges.size();
        diagnostic->retained_cycles = cycles.size();
        diagnostic->regions = regions.size();
        diagnostic->uv_tolerance = tolerance;
    }
    return regions;
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
    const double expected = lo == a && hi == b ? t : lo + (t - a) / (b - a) * (hi - lo);
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

/// A required raw boundary is immutable even when legacy trimming would snap it to a nearby knot.
NurbsCurve required_interval(const NurbsCurve& curve, double first, double last, int edge, double world_first,
                             double world_last) {
    try {
        NurbsCurve part = interval(curve, first, last);
        if (part.domain() != std::make_pair(first, last))
            throw EdgeEventError(edge, world_first, world_last,
                                 "A required interval changed its original raw boundary");
        return part;
    } catch (const EdgeEventError&) {
        throw;
    } catch (const std::exception& error) {
        throw EdgeEventError(edge, world_first, world_last,
                             std::string("Cannot represent a required raw interval: ") + error.what());
    }
}

/// Original normalized law for required events; legacy face splitting keeps its proposal fallback.
std::pair<double, double> shared_pcurve_range(const Source& source, const Piece& piece,
                                              const NurbsCurve& world, const NurbsCurve& uv,
                                              const NurbsSurface& surface, double tolerance, bool required) {
    const auto wd = source.world.domain(), pd = uv.domain();
    const auto mapped = [&](double t) {
        return required && t == wd.first ? pd.first
               : required && t == wd.second
                   ? pd.second
                   : pd.first + (t - wd.first) / (wd.second - wd.first) * (pd.second - pd.first);
    };
    double first = mapped(piece.lo), last = mapped(piece.hi);
    if (required) {
        std::string diagnostic;
        if (!std::isfinite(wd.second - wd.first) || !(wd.second > wd.first) ||
            !std::isfinite(pd.second - pd.first) || !(pd.second > pd.first) || !std::isfinite(first) ||
            !std::isfinite(last) || first < pd.first || last > pd.second || !(last > first) ||
            curve_surface_intersection::same_parameter(source.world, uv, surface, tolerance, &diagnostic) !=
                curve_surface_intersection::ParameterMatch::Matched)
            throw EdgeEventError(source.edge, piece.lo, piece.hi,
                                 "Required edge event lacks an original raw curve/trim law: " + diagnostic);
    } else {
        first = lifted_parameter(surface, uv, world.point_at_start(), first, tolerance);
        last = lifted_parameter(surface, uv, world.point_at_end(), last, tolerance);
    }
    return {first, last};
}

/// Pcurves of a piece of a shared BRep edge, cut from every adjacent face trim.
void add_shared_pcurves(BRep& result, const BRep& brep, const Source& source, const Piece& piece,
                        const NurbsCurve& world, double tolerance, bool required = false) {

    for (const BRepCurveOnSurface& pc : brep.m_edges[source.edge].pcurves) {
        const NurbsSurface& surface = brep.m_surfaces[pc.surface_index];
        const std::array<int, 2> sides = {pc.curve_2d_index, pc.curve_2d_index_2};
        std::array<int, 2> ids = {-1, -1};

        for (size_t at = 0; at < 2; ++at) {
            if (sides[at] < 0)
                continue;

            const NurbsCurve& c = brep.m_curves_2d[sides[at]];
            const auto [ca, cb] = shared_pcurve_range(source, piece, world, c, surface, tolerance, required);
            require(cb > ca, "A split crosses an unsupported periodic trim seam");
            NurbsCurve trim = required ? required_interval(c, ca, cb, source.edge, piece.lo, piece.hi)
                                       : interval(c, ca, cb);
            if (required && curve_surface_intersection::same_parameter(world, trim, surface, tolerance) !=
                                curve_surface_intersection::ParameterMatch::Matched)
                throw EdgeEventError(source.edge, piece.lo, piece.hi,
                                     "A required edge event changed its raw trim boundary or failed the "
                                     "whole-piece certificate");
            ids[at] = result.add_curve_2d(trim);
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
    const bool degenerated = source.edge >= 0 && brep.m_edges[source.edge].degenerated;
    double wa = 0.0;
    double wb = 0.0;
    std::tie(wa, wb) = world_run(brep.m_surfaces[surface_index], source, run, tolerance);
    const double lo = std::min(wa, wb);
    const double hi = std::max(wa, wb);
    require(hi > lo, "Split would create a collapsed edge");
    const BRepOrientation orientation = wa < wb ? FORWARD : REVERSED;

    for (const Piece& piece : pieces) {
        const bool same = source.edge >= 0 ? sources[piece.source].edge == source.edge : piece.source == run.source;
        if (!same)
            continue;

        const bool matching = (lo == piece.lo && hi == piece.hi) || (!degenerated &&
            vertex_interval(source.world, lo, piece.lo, source.world.point_at(lo), tolerance * 4.0) &&
            vertex_interval(source.world, hi, piece.hi, source.world.point_at(hi), tolerance * 4.0));
        if (matching)
            return {piece.edge, orientation};
    }

    const NurbsCurve world = interval(source.world, lo, hi);
    const int a = vertex(result, world.point_at_start(), tolerance * 4.0);
    const int b = vertex(result, world.point_at_end(), tolerance * 4.0);
    const int edge = result.add_edge(degenerated ? -1 : result.add_curve_3d(world), a, b, tolerance);
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

/// Trim loops of a face, each boundary edge added to the sources.
std::vector<std::vector<NurbsCurve>> boundary_loops(
    const BRep& brep,
    const brep_face_boundary::Result& boundary,
    double uv_tolerance,
    std::vector<Source>& sources
) {

    std::vector<std::vector<NurbsCurve>> loops;

    require(boundary.complete, boundary.failure.c_str());
    for (const auto& wire : boundary.wires) {
        std::vector<NurbsCurve> curves;
        size_t corners = 0;

        for (const auto& use : wire.uses) {
            const auto& er = use.edge;
            const BRepEdge& edge = brep.m_edges[er.index];
            const int ci = use.pcurve;
            require(ci >= 0, "Face has no source UV boundary");
            NurbsCurve uv = brep.m_curves_2d[ci];
            check_curve(uv);
            NurbsCurve world;
            if (edge.degenerated) {
                world = NurbsCurve(3, false, 2, 2);
                world.set_nurbsknot(0, uv.domain_start());
                world.set_nurbsknot(1, uv.domain_end());
                world.set_cv(0, brep.m_vertices[edge.start_vertex].point);
                world.set_cv(1, brep.m_vertices[edge.end_vertex].point);
            } else {
                world = brep.m_curves_3d[edge.curve_3d_index];
            }
            check_curve(world);
            sources.push_back({er.index, world, uv, -1, edge.start_vertex, edge.end_vertex});

            if (er.orientation == REVERSED)
                require(uv.reverse(), "Kernel refused to reverse a face trim");

            corners += flat(uv, uv_tolerance * 0.25) ? 1 : 3;
            curves.push_back(std::move(uv));
        }

        require(corners >= 3, "Face has an invalid boundary");
        loops.push_back(std::move(curves));
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

void validate_trim(const NurbsCurve& world, const NurbsCurve& uv, const NurbsSurface& surface, double tolerance) {
    std::string diagnostic;
    const auto match = curve_surface_intersection::same_parameter(world, uv, surface, tolerance, &diagnostic);
    if (match == curve_surface_intersection::ParameterMatch::Matched)
        return;

    double parameter = uv.domain_start();
    double distance = 0;
    for (int k = 0; k <= 32; ++k) {
        const double t = uv.domain_start() + (uv.domain_end() - uv.domain_start()) * k / 32.0;
        const double w = world.domain_start() + (world.domain_end() - world.domain_start()) * k / 32.0;
        const Point q = uv.point_at(t);
        const double gap = world.point_at(w).distance(surface.point_at(q[0], q[1]));
        if (gap > distance) {
            parameter = t;
            distance = gap;
        }
    }
    const std::string status = match == curve_surface_intersection::ParameterMatch::Mismatched
        ? "is mismatched: " : "is unresolved: ";
    throw CurveError(world, uv, surface, parameter, distance, tolerance, status + diagnostic);
}

void observe_failed_split(const char* path, const BRep& input, const BRep& partial, int face,
    const std::vector<FaceCut>& cuts, const std::vector<Source>& sources, const std::vector<Piece>& pieces,
    const std::vector<std::vector<std::vector<Run>>>& regions, double tolerance, double uv_tolerance,
    const char* failure) {
    nlohmann::ordered_json row = {{"failure", failure}, {"face", face}, {"tolerance", tolerance},
        {"uv_tolerance", uv_tolerance}, {"input", input.jsondump()}, {"partial", partial.jsondump()},
        {"cuts", nlohmann::ordered_json::array()}, {"sources", nlohmann::ordered_json::array()},
        {"pieces", nlohmann::ordered_json::array()}, {"regions", nlohmann::ordered_json::array()}};
    for (const FaceCut& cut : cuts)
        row["cuts"].push_back({{"world", cut.world.jsondump()}, {"pcurve", cut.pcurve.jsondump()}, {"section", cut.section}});
    for (const Source& source : sources)
        row["sources"].push_back({{"edge", source.edge}, {"section", source.section},
            {"world", source.world.jsondump()}, {"pcurve", source.uv.jsondump()}});
    for (const Piece& piece : pieces)
        row["pieces"].push_back({{"source", piece.source}, {"edge", piece.edge}, {"first", piece.lo}, {"last", piece.hi}});
    for (const auto& region : regions) {
        nlohmann::ordered_json wires = nlohmann::ordered_json::array();
        for (const auto& wire : region) {
            nlohmann::ordered_json runs = nlohmann::ordered_json::array();
            for (const Run& run : wire)
                runs.push_back({{"source", run.source}, {"first", run.a}, {"last", run.b}});
            wires.push_back(std::move(runs));
        }
        row["regions"].push_back(std::move(wires));
    }
    std::ofstream output(path, std::ios::app | std::ios::binary);
    if (!output || !(output << row.dump() << '\n'))
        throw std::runtime_error(std::string("Cannot write split failure observation: ") + path);
    output.flush();
    if (!output)
        throw std::runtime_error(std::string("Cannot flush split failure observation: ") + path);
    output.close();
    if (!output)
        throw std::runtime_error(std::string("Cannot close split failure observation: ") + path);
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
                const double limit = std::max(tolerance, edge.tolerance) * 8.0;
                validate_trim(world, uv, result.m_surfaces[pc.surface_index], limit);
            }
    }
}

/// Parameter-space tolerance of a surface: tolerance over its larger world length per unit parameter.
double surface_uv_tolerance(const NurbsSurface& surface, double tolerance) {
    return tolerance / nurbs_bounds::surface_metric(surface);
}

/// Wires of every region, one new edge per run added to result.
std::vector<std::vector<BRepRef>> region_wires(
    BRep& result,
    std::vector<Piece>& pieces,
    const BRep& brep,
    int surface_index,
    const std::vector<Source>& sources,
    const std::vector<std::vector<std::vector<Run>>>& regions,
    double tolerance
) {

    std::vector<std::vector<BRepRef>> new_wires;

    for (const std::vector<std::vector<Run>>& region : regions) {
        std::vector<BRepRef> wires;

        for (const std::vector<Run>& loop : region) {
            std::vector<BRepRef> refs;

            for (const Run& run : loop)
                refs.push_back(add_run_edge(result, pieces, brep, surface_index, sources, run, tolerance));

            wires.push_back({result.add_wire(refs), FORWARD});
        }

        new_wires.push_back(wires);
    }

    return new_wires;
}

BRepShell event_solid_boundary(const BRep& brep, const BRepSolid& solid) {
    BRepShell boundary;
    for (const auto& shell : solid.shells)
        for (const auto& face : brep.m_shells[shell.index].faces)
            boundary.faces.push_back({face.index, brep_compose(shell.orientation, face.orientation)});
    return boundary;
}

bool regular_event_edges(const BRep& brep, const BRepShell& boundary) {
    std::map<int, std::pair<int, int>> uses;
    for (const auto& face : boundary.faces)
        for (const auto& wire : brep.m_faces[face.index].wires)
            for (const auto& use : brep.wire_edges(wire)) {
                if (brep.m_edges[use.index].degenerated)
                    continue;
                const auto orientation = brep_compose(face.orientation, use.orientation);
                if (orientation == FORWARD) {
                    ++uses[use.index].first;
                    ++uses[use.index].second;
                } else if (orientation == REVERSED) {
                    ++uses[use.index].first;
                    --uses[use.index].second;
                }
            }
    for (const auto& [edge, state] : uses)
        if (state.first != 2 || state.second != 0)
            return false;
    return !uses.empty();
}

void require_event_vertex_links(const BRep& original, const BRep& result) {
    for (size_t i = 0; i < original.m_solids.size(); ++i) {
        const auto before = event_solid_boundary(original, original.m_solids[i]);
        const auto after = event_solid_boundary(result, result.m_solids[i]);
        if (!regular_event_edges(original, before) || !regular_event_edges(result, after) ||
            brep_acceptance::shell_vertex_links(original, before) != brep_acceptance::VertexLinks::Matched ||
            brep_acceptance::shell_vertex_links(result, after) != brep_acceptance::VertexLinks::Matched)
            throw EdgeEventError(
                -1, 0, 0,
                "Required edge events need regular original and result solid edge uses and vertex links");
    }
}

/// Selected face uses must have complete, unambiguous original associations before and after rebuilding.
std::vector<int> required_event_faces(const BRep& brep,
                                      const std::map<int, std::vector<EdgeEvent>>& grouped) {
    std::vector<int> faces;
    std::map<int, bool> consumed;
    for (const auto& item : grouped)
        consumed[item.first] = false;
    for (int i = 0; i < brep.face_count(); ++i) {
        int owner = -1;
        for (const auto& wire : brep.m_faces[i].wires)
            for (const auto& use : brep.wire_edges(wire))
                if (grouped.contains(use.index)) {
                    owner = use.index;
                    consumed[owner] = true;
                }
        if (owner < 0)
            continue;
        const auto boundary = brep_face_boundary::extract(brep, i);
        if (!boundary.complete)
            throw EdgeEventError(owner, grouped.at(owner).front().parameter,
                                 grouped.at(owner).front().parameter,
                                 "Required edge face incidence is unresolved: " + boundary.failure);
        faces.push_back(i);
    }
    for (const auto& [edge, used] : consumed)
        if (!used)
            throw EdgeEventError(edge, 0, 0, "Required edge has no owning face-boundary consumer");
    return faces;
}

void require_event_associations(const Source& source, const BRep& brep) {
    std::map<int, std::pair<int, int>> associations;
    for (const auto& image : brep.m_edges[source.edge].pcurves) {
        const auto slots = std::make_pair(image.curve_2d_index, image.curve_2d_index_2);
        const auto [stored, inserted] = associations.emplace(image.surface_index, slots);
        if (!inserted && stored->second != slots)
            throw EdgeEventError(
                source.edge, source.world.domain_start(), source.world.domain_end(),
                "Required edge has incompatible duplicate pcurve associations on one support");
    }
}

/// Merge only identical raw events; conflicting supplied vertex identities are unresolved.
std::vector<EdgeEvent> ordered_edge_events(const Source& source, std::vector<EdgeEvent> events,
                                           const BRepEdge& edge) {
    const auto domain = source.world.domain();
    for (const auto& event : events)
        if (!std::isfinite(event.parameter) || event.parameter < domain.first ||
            event.parameter > domain.second)
            throw EdgeEventError(source.edge, event.parameter, event.parameter,
                                 "Required edge event is outside the original world domain");
    events.push_back({source.edge, domain.first, edge.start_vertex});
    events.push_back({source.edge, domain.second, edge.end_vertex});
    std::sort(events.begin(), events.end(), [](const auto& a, const auto& b) {
        return a.parameter < b.parameter || (a.parameter == b.parameter && a.vertex < b.vertex);
    });
    std::vector<EdgeEvent> result;
    for (const auto& event : events) {
        if (!std::isfinite(event.parameter) || event.parameter < domain.first ||
            event.parameter > domain.second)
            throw EdgeEventError(source.edge, event.parameter, event.parameter,
                                 "Required edge event is outside the original world domain");
        if (!result.empty() && result.back().parameter == event.parameter) {
            if (event.vertex >= 0) {
                if (result.back().vertex >= 0 && result.back().vertex != event.vertex)
                    throw EdgeEventError(
                        source.edge, event.parameter, event.parameter,
                        "One required edge parameter has conflicting original vertex identities");
                result.back().vertex = event.vertex;
            }
        } else
            result.push_back(event);
    }
    return result;
}

/// Use explicit existing identity or allocate once per distinct raw event, never by proximity.
int required_edge_vertex(BRep& result, const BRep& original, const Source& source, const EdgeEvent& event,
                         double tolerance) {
    Point point;
    if (event.vertex >= 0) {
        if (event.vertex >= original.vertex_count())
            throw EdgeEventError(source.edge, event.parameter, event.parameter,
                                 "Required edge event references a vertex outside its original owner");
        point = original.m_vertices[event.vertex].point;
    } else
        point = source.world.point_at(event.parameter);
    if (!nurbs_interval::curve_within_ball(source.world, {event.parameter, event.parameter},
                                           {point[0], point[1], point[2]}, tolerance))
        throw EdgeEventError(source.edge, event.parameter, event.parameter,
                             "Required edge event lacks an actual canonical vertex ball certificate");
    return event.vertex >= 0 ? event.vertex : result.add_vertex(point, tolerance);
}

void required_edge_pieces(BRep& result, const BRep& original, const Source& source, size_t index,
                          const std::vector<EdgeEvent>& events, std::vector<Piece>& pieces,
                          std::vector<EdgeEvent>& resolved, double tolerance) {
    const Piece full{index, events.front().parameter, events.back().parameter, source.edge};
    for (const auto& image : original.m_edges[source.edge].pcurves) {
        require(image.curve_2d_index >= 0, "Required edge has no primary original pcurve");
        for (int pc : {image.curve_2d_index, image.curve_2d_index_2})
            if (pc >= 0)
                shared_pcurve_range(source, full, source.world, original.m_curves_2d[pc],
                                    original.m_surfaces[image.surface_index], tolerance, true);
    }
    std::vector<int> vertices;
    for (const auto& event : events) {
        const int vertex = required_edge_vertex(result, original, source, event, tolerance);
        vertices.push_back(vertex);
        resolved.push_back({source.edge, event.parameter, vertex});
    }
    if (events.size() == 2) {
        pieces.push_back({index, events.front().parameter, events.back().parameter, source.edge});
        return;
    }
    for (size_t i = 1; i < events.size(); ++i) {
        const double lo = events[i - 1].parameter, hi = events[i].parameter;
        require(pieces.size() < WORK_LIMIT, "Required edge pieces exceed the bounded workload");
        const NurbsCurve world = required_interval(source.world, lo, hi, source.edge, lo, hi);
        const int edge = result.add_edge(result.add_curve_3d(world), vertices[i - 1], vertices[i],
                                         std::max(tolerance, original.m_edges[source.edge].tolerance));
        const Piece piece{index, lo, hi, edge};
        add_shared_pcurves(result, original, source, piece, world, tolerance, true);
        pieces.push_back(piece);
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

    for (size_t i = 1; i < cuts.size(); ++i) {
        NurbsCurve piece = interval(curve, cuts[i - 1], cuts[i]);
        piece.arrowhead = arrowhead_piece(curve.arrowhead, i == 1, i + 1 == cuts.size());
        result.push_back(std::move(piece));
    }

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
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face_index].surface_index];
    std::vector<FaceCut> paired;
    for (const NurbsCurve& cutter : cutters) {
        check_curve(cutter);
        for (const NurbsCurve& uv : pullback(surface, cutter, tolerance))
            paired.push_back({cutter, uv, -1});
    }
    return split_brep_face_by_pcurves(brep, face_index, paired, tolerance).brep;
}

FaceSplit split_brep_edges_at_events(const BRep& brep, const std::vector<EdgeEvent>& events,
                                     double tolerance) {
    check_tolerance(tolerance);
    require(brep.is_valid(), "Required edge splitting needs a valid owning BRep");
    require(events.size() < WORK_LIMIT, "Required edge events exceed the bounded workload");
    if (events.empty())
        return {brep, {}, {}};
    std::map<int, std::vector<EdgeEvent>> grouped;
    for (const auto& event : events) {
        if (event.edge < 0 || event.edge >= brep.edge_count() || event.vertex < -1)
            throw EdgeEventError(event.edge, event.parameter, event.parameter,
                                 "Required edge event has invalid ownership");
        grouped[event.edge].push_back(event);
    }
    FaceSplit split{brep, required_event_faces(brep, grouped), {}};
    std::vector<Source> sources;
    std::vector<Piece> pieces;
    for (const auto& [edge, required] : grouped) {
        const auto& original = brep.m_edges[edge];
        if (original.degenerated)
            throw EdgeEventError(edge, 0, 0,
                                 "Degenerate edge events require explicit UV incidence ownership");
        Source source{edge, brep.m_curves_3d[original.curve_3d_index], {}};
        check_curve(source.world);
        const auto domain = source.world.domain();
        if (!std::isfinite(domain.second - domain.first) || !(domain.second > domain.first) ||
            original.pcurves.empty())
            throw EdgeEventError(edge, domain.first, domain.second,
                                 "Required edge lacks a finite raw domain or pcurves");
        require_event_associations(source, brep);
        const auto ordered = ordered_edge_events(source, required, original);
        sources.push_back(source);
        required_edge_pieces(split.brep, brep, sources.back(), sources.size() - 1, ordered, pieces,
                             split.edge_events, tolerance);
    }
    replace_wires(split.brep, brep, sources, pieces);
    validate(split.brep, brep, tolerance);
    require_event_vertex_links(brep, split.brep);
    for (const auto& piece : pieces)
        split.edges.push_back({sources[piece.source].edge, -1, piece.edge, piece.lo, piece.hi});
    for (int face : split.faces) {
        const auto boundary = brep_face_boundary::extract(split.brep, face);
        if (!boundary.complete)
            throw EdgeEventError(-1, 0, 0, "Required edge result lost a face incidence: " + boundary.failure);
    }
    return split;
}

FaceSplit split_brep_face_by_pcurves(
    const BRep& brep,
    int face_index,
    const std::vector<FaceCut>& cutters,
    double tolerance
) {
    return split_brep_face_by_pcurves(brep, face_index, cutters, tolerance, nullptr);
}

FaceSplit split_brep_face_by_pcurves(
    const BRep& brep, int face_index, const std::vector<FaceCut>& cutters,
    double tolerance, ArrangementDiagnostic* diagnostic
) {
    if (diagnostic)
        *diagnostic = ArrangementDiagnostic{};

    check_tolerance(tolerance);
    require(brep.is_valid(), "Split requires a valid BRep");
    require(face_index >= 0 && face_index < brep.face_count(), "Select one BRep face to split");
    require(!cutters.empty(), "Select at least one cutter");
    const BRepFace& face = brep.m_faces[face_index];
    const NurbsSurface& surface = brep.m_surfaces[face.surface_index];
    check_surface(surface);

    const double uv_tolerance = surface_uv_tolerance(surface, tolerance);

    const auto boundary = brep_face_boundary::extract(brep, face_index);
    require(boundary.complete, boundary.failure);
    std::vector<Source> sources;
    const std::vector<std::vector<NurbsCurve>> original_loops = boundary_loops(brep, boundary, uv_tolerance, sources);

    for (const FaceCut& cutter : cutters) {
        check_curve(cutter.world);
        check_curve(cutter.pcurve);
        sources.push_back({-1, cutter.world, cutter.pcurve, cutter.section});
    }

    const std::vector<std::vector<std::vector<Run>>> regions = arrange(sources, original_loops, boundary, uv_tolerance, diagnostic);

    if (regions.size() < 2)
        return {brep, {face_index}, {}};

    BRep result = brep;
    std::vector<Piece> pieces;
    const std::vector<std::vector<BRepRef>> new_wires = region_wires(result, pieces, brep, face.surface_index, sources, regions, tolerance);
    replace_wires(result, brep, sources, pieces);
    add_faces(result, face, face_index, new_wires);
    try {
        validate(result, brep, tolerance);
    } catch (const std::exception& failure) {
        const char* observation = std::getenv("SESSION_SPLIT_FAILURE_OBSERVATION");
        if (observation && *observation)
            try {
                observe_failed_split(observation, brep, result, face_index, cutters, sources, pieces,
                    regions, tolerance, uv_tolerance, failure.what());
            } catch (const std::exception& capture_error) {
                std::fprintf(stderr, "Split failure observation failed: %s\n", capture_error.what());
                std::fflush(stderr);
            } catch (...) {
                std::fprintf(stderr, "Split failure observation failed with unknown error\n");
                std::fflush(stderr);
            }
        throw;
    }

    FaceSplit split;
    split.faces.push_back(face_index);
    for (int i = brep.face_count(); i < result.face_count(); ++i)
        split.faces.push_back(i);
    for (const Piece& piece : pieces)
        split.edges.push_back({sources[piece.source].edge, sources[piece.source].section, piece.edge, piece.lo, piece.hi});
    split.brep = std::move(result);
    return split;
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

    NurbsCurve curve = NurbsCurve::create(false, 1, {line.point_at(0), line.point_at(1)});
    curve.arrowhead = line.arrowhead;
    std::vector<Line> result;

    for (const NurbsCurve& piece : split_curve_by_curves(curve, cutters, tolerance)) {
        Line next = Line::from_points(piece.point_at_start(), piece.point_at_end());
        next.name = line.name;
        next.width = line.width;
        next.dash = line.dash;
        next.linecolor = line.linecolor;
        next.arrowhead = piece.arrowhead;
        result.push_back(std::move(next));
    }

    return result;
}

std::vector<Polyline> split_polyline_by_curves(
    const Polyline& polyline,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
) {

    NurbsCurve curve = NurbsCurve::create(false, 1, polyline.get_points());
    curve.arrowhead = polyline.arrowhead;
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
        next.arrowhead = piece.arrowhead;
        result.push_back(std::move(next));
    }

    return result;
}

} // namespace session_cpp::split
