#include "simple_split.h"
#include "closest.h"
#include "line.h"
#include "polyline.h"
#include "tolerance.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace session_cpp::simple_split {
namespace {
constexpr double epsilon = Tolerance::ZERO_TOLERANCE;
constexpr auto forward = BRepOrientation::Forward;
constexpr auto reversed = BRepOrientation::Reversed;
constexpr size_t work_limit = 200000;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::invalid_argument(message);
}
void check_tolerance(double tolerance) {
  require(std::isfinite(tolerance) && tolerance > 0.,
          "Split tolerance must be finite and positive");
}
void check_curve(const NurbsCurve &curve) {
  require(curve.is_valid(), "Split requires valid curves");
  for (int i = 0; i < curve.cv_count(); ++i) {
    const auto p = curve.get_cv(i);
    require(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
                std::isfinite(curve.weight(i)) && curve.weight(i) > 0.,
            "Split requires finite controls and positive rational weights");
  }
}
void check_surface(const NurbsSurface &surface) {
  require(surface.is_valid(), "Split requires a valid NURBS surface");
  for (int i = 0; i < surface.cv_count(0); ++i)
    for (int j = 0; j < surface.cv_count(1); ++j) {
      auto p = surface.get_cv(i, j);
      double w = surface.weight(i, j);
      require(std::isfinite(p[0]) && std::isfinite(p[1]) &&
                  std::isfinite(p[2]) && std::isfinite(w) && w > 0.,
              "Split requires finite surface controls and positive rational "
              "weights");
    }
}
NurbsCurve interval(const NurbsCurve &curve, double a, double b) {
  auto result = curve;
  result.refresh_guid();
  const auto [lo, hi] = curve.domain();
  a = std::clamp(a, lo, hi);
  b = std::clamp(b, lo, hi);
  require(b > a, "Split produced an empty curve interval");
  if (a > lo || b < hi)
    require(result.trim(a, b), "Kernel refused a split interval");
  return result;
}
double distance(const Point &a, const Point &b) { return a.distance(b); }
double dot(const Point &a, const Point &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Point subtract(const Point &a, const Point &b) {
  return Point(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
std::pair<double, double> closest(const NurbsCurve &curve, const Point &point) {
  auto [t, distance_] = Closest::curve_point(curve, point);
  (void)distance_;
  auto [lo, hi] = curve.domain();
  if (curve.degree() == 1) {
    // Every polyline span is a separate closest-point candidate, including
    // closed seams.
    auto spans = curve.get_span_vector();
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < spans.size(); ++i) {
      const auto a = curve.point_at(spans[i - 1]), b = curve.point_at(spans[i]),
                 v = subtract(b, a);
      double length2 = dot(v, v);
      if (length2 <= epsilon * epsilon)
        continue;
      double fraction =
          std::clamp(dot(subtract(point, a), v) / length2, 0., 1.);
      auto segment = interval(curve, spans[i - 1], spans[i]);
      double w0 = segment.weight(0),
             w1 = segment.weight(segment.cv_count() - 1);
      double normalized =
          fraction * w0 / (w1 * (1. - fraction) + fraction * w0);
      double candidate = spans[i - 1] + normalized * (spans[i] - spans[i - 1]);
      double gap = distance(curve.point_at(candidate), point);
      if (gap < best) {
        best = gap;
        t = candidate;
      }
    }
    return {t, distance(curve.point_at(t), point)};
  }
  for (int i = 0; i < 24; ++i) {
    auto eval = curve.evaluate(t, 1);
    Point d(eval[1][0], eval[1][1], eval[1][2]);
    Point r(eval[0][0] - point[0], eval[0][1] - point[1],
            eval[0][2] - point[2]);
    double dd = dot(d, d);
    if (dd <= epsilon * epsilon)
      break;
    double next = std::clamp(t - dot(d, r) / dd, lo, hi);
    if (std::abs(next - t) <= epsilon * (hi - lo)) {
      t = next;
      break;
    }
    t = next;
  }
  return {t, distance(curve.point_at(t), point)};
}
std::vector<double> unique_parameters(std::vector<double> values, double lo,
                                      double hi) {
  std::sort(values.begin(), values.end());
  std::vector<double> result;
  for (double value : values) {
    value = std::clamp(value, lo, hi);
    if (result.empty() || value - result.back() > (hi - lo) * epsilon * 16.)
      result.push_back(value);
  }
  return result;
}

struct Box {
  std::array<double, 3> lo, hi;
  explicit Box(const NurbsCurve &curve) {
    auto p = curve.get_cv(0);
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
  double diagonal() const {
    return std::hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
  }
  bool overlaps(const Box &other, double tolerance) const {
    for (int d = 0; d < 3; ++d)
      if (hi[d] + tolerance < other.lo[d] || other.hi[d] + tolerance < lo[d])
        return false;
    return true;
  }
};
bool flat(const NurbsCurve &curve, double tolerance) {
  const auto a = curve.point_at_start(), b = curve.point_at_end(),
             v = subtract(b, a);
  const double length2 = dot(v, v);
  if (length2 <= tolerance * tolerance)
    return Box(curve).diagonal() <= tolerance;
  for (int i = 0; i < curve.cv_count(); ++i) {
    const auto p = curve.get_cv(i);
    const double t = dot(subtract(p, a), v) / length2;
    if (t < -epsilon || t > 1. + epsilon ||
        distance(p, Point(a[0] + v[0] * t, a[1] + v[1] * t, a[2] + v[2] * t)) >
            tolerance)
      return false;
  }
  return true;
}
std::pair<double, double> refine(const NurbsCurve &a, const NurbsCurve &b,
                                 double ta, double tb) {
  const auto [a0, a1] = a.domain();
  const auto [b0, b1] = b.domain();
  for (int k = 0; k < 40; ++k) {
    auto da = a.evaluate(ta, 1), db = b.evaluate(tb, 1);
    Point r(da[0][0] - db[0][0], da[0][1] - db[0][1], da[0][2] - db[0][2]);
    Point u(da[1][0], da[1][1], da[1][2]), v(db[1][0], db[1][1], db[1][2]);
    double aa = dot(u, u), ab = dot(u, v), bb = dot(v, v),
           det = aa * bb - ab * ab;
    if (det <= epsilon * epsilon * aa * bb)
      break;
    double ar = dot(u, r), br = dot(v, r);
    double na = std::clamp(ta + (-bb * ar + ab * br) / det, a0, a1);
    double nb = std::clamp(tb + (-ab * ar + aa * br) / det, b0, b1);
    if (std::abs(na - ta) < epsilon * (a1 - a0) &&
        std::abs(nb - tb) < epsilon * (b1 - b0)) {
      ta = na;
      tb = nb;
      break;
    }
    ta = na;
    tb = nb;
  }
  return {ta, tb};
}
std::vector<std::pair<double, double>> intersections(const NurbsCurve &a,
                                                     const NurbsCurve &b,
                                                     double tolerance,
                                                     size_t &budget) {
  struct Pair {
    NurbsCurve a, b;
    int depth;
  };
  std::vector<Pair> work;
  auto av = a.get_span_vector(), bv = b.get_span_vector();
  require(av.size() > 1 && bv.size() > 1,
          "Split requires nonempty curve spans");
  require(av.size() - 1 <= budget / (bv.size() - 1),
          "Curve intersection exceeds the bounded split workload");
  for (size_t i = 1; i < av.size(); ++i)
    for (size_t j = 1; j < bv.size(); ++j)
      work.push_back(
          {interval(a, av[i - 1], av[i]), interval(b, bv[j - 1], bv[j]), 0});
  std::vector<std::pair<double, double>> hits;
  while (!work.empty()) {
    require(budget > 0,
            "Curve intersection exceeds the bounded split workload");
    --budget;
    auto pair = std::move(work.back());
    work.pop_back();
    Box ba(pair.a), bb(pair.b);
    if (!ba.overlaps(bb, tolerance))
      continue;
    if ((flat(pair.a, tolerance * .1) && flat(pair.b, tolerance * .1)) ||
        pair.depth >= 48) {
      const auto ap = pair.a.point_at_start(), aq = pair.a.point_at_end(),
                 bp = pair.b.point_at_start(), bq = pair.b.point_at_end();
      auto u = subtract(aq, ap), v = subtract(bq, bp);
      double aa = dot(u, u), ab = dot(u, v), vv = dot(v, v);
      if (aa > tolerance * tolerance && vv > tolerance * tolerance &&
          aa * vv - ab * ab < epsilon * epsilon * aa * vv) {
        double t0 = dot(subtract(bp, ap), u) / aa,
               t1 = dot(subtract(bq, ap), u) / aa;
        double gap = distance(
            bp, Point(ap[0] + u[0] * t0, ap[1] + u[1] * t0, ap[2] + u[2] * t0));
        if (gap <= tolerance &&
            std::min(1., std::max(t0, t1)) - std::max(0., std::min(t0, t1)) >
                tolerance / std::sqrt(aa))
          throw std::invalid_argument(
              "Overlapping curves do not define isolated split points");
      }
      auto [ta, tb, d] = Closest::curve_curve(pair.a, pair.b);
      if (d > tolerance * 2.)
        continue;
      std::tie(ta, tb) = refine(pair.a, pair.b, ta, tb);
      if (distance(a.point_at(ta), b.point_at(tb)) > tolerance)
        continue;
      bool duplicate = false;
      for (auto hit : hits)
        if (distance(a.point_at(hit.first), a.point_at(ta)) <= tolerance * 2. &&
            distance(a.point_at((hit.first + ta) * .5), a.point_at(ta)) <=
                tolerance * 2. &&
            distance(b.point_at(hit.second), b.point_at(tb)) <=
                tolerance * 2. &&
            distance(b.point_at((hit.second + tb) * .5), b.point_at(tb)) <=
                tolerance * 2.) {
          duplicate = true;
          break;
        }
      if (!duplicate)
        hits.emplace_back(ta, tb);
      continue;
    }
    if (ba.diagonal() >= bb.diagonal()) {
      auto [lo, hi] = pair.a.domain();
      double mid = (lo + hi) * .5;
      work.push_back({interval(pair.a, lo, mid), pair.b, pair.depth + 1});
      work.push_back({interval(pair.a, mid, hi), pair.b, pair.depth + 1});
    } else {
      auto [lo, hi] = pair.b.domain();
      double mid = (lo + hi) * .5;
      work.push_back({pair.a, interval(pair.b, lo, mid), pair.depth + 1});
      work.push_back({pair.a, interval(pair.b, mid, hi), pair.depth + 1});
    }
  }
  std::sort(hits.begin(), hits.end());
  return hits;
}

std::vector<NurbsCurve> pullback(const NurbsSurface &surface,
                                 const NurbsCurve &curve, double tolerance) {
  // Affine patches preserve the cutter's exact rational representation and
  // parameterization.
  if (surface.m_cv_count[0] == 2 && surface.m_cv_count[1] == 2 &&
      surface.m_order[0] == 2 && surface.m_order[1] == 2 && !surface.m_is_rat) {
    auto p = surface.get_cv(0, 0), u = subtract(surface.get_cv(1, 0), p),
         v = subtract(surface.get_cv(0, 1), p);
    auto last = surface.get_cv(1, 1);
    double uu = dot(u, u), uv = dot(u, v), vv = dot(v, v),
           det = uu * vv - uv * uv;
    if (det > epsilon * epsilon * uu * vv &&
        distance(last, Point(p[0] + u[0] + v[0], p[1] + u[1] + v[1],
                             p[2] + u[2] + v[2])) <= tolerance) {
      auto result = curve;
      result.refresh_guid();
      auto [u0, u1] = surface.domain(0);
      auto [v0, v1] = surface.domain(1);
      for (int i = 0; i < curve.cv_count(); ++i) {
        auto q = curve.get_cv(i), d = subtract(q, p);
        double du = dot(d, u), dv = dot(d, v);
        double a = (du * vv - dv * uv) / det, b = (dv * uu - du * uv) / det;
        if (distance(q, Point(p[0] + a * u[0] + b * v[0],
                              p[1] + a * u[1] + b * v[1],
                              p[2] + a * u[2] + b * v[2])) > tolerance)
          return {};
        double w = curve.weight(i);
        result.set_cv_4d(i, (u0 + a * (u1 - u0)) * w, (v0 + b * (v1 - v0)) * w,
                         0., w);
      }
      return {result};
    }
  }
  return Closest::surface_curve(surface, curve, 0., 0., tolerance);
}

std::vector<Point> polygon(const NurbsCurve &curve, double tolerance) {
  struct Part {
    NurbsCurve curve;
    int depth;
  };
  std::vector<Part> work;
  auto spans = curve.get_span_vector();
  for (size_t i = spans.size(); i > 1; --i)
    work.push_back({interval(curve, spans[i - 2], spans[i - 1]), 0});
  std::vector<Point> result;
  size_t visited = 0;
  while (!work.empty()) {
    require(++visited <= work_limit,
            "Trim sampling exceeds the bounded workload");
    auto part = std::move(work.back());
    work.pop_back();
    if (flat(part.curve, tolerance * .25)) {
      result.push_back(part.curve.point_at_start());
      continue;
    }
    require(part.depth < 40, "Trim sampling exceeds parameter precision");
    auto [lo, hi] = part.curve.domain();
    double mid = (lo + hi) * .5;
    work.push_back({interval(part.curve, mid, hi), part.depth + 1});
    work.push_back({interval(part.curve, lo, mid), part.depth + 1});
  }
  return result;
}
bool inside(const Point &p, const std::vector<Point> &polygon) {
  bool result = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto &a = polygon[i];
    const auto &b = polygon[j];
    if ((a[1] > p[1]) != (b[1] > p[1]) &&
        p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0])
      result = !result;
  }
  return result;
}
bool inside_loops(const Point &p,
                  const std::vector<std::vector<Point>> &loops) {
  if (loops.empty() || !inside(p, loops[0]))
    return false;
  for (size_t i = 1; i < loops.size(); ++i)
    if (inside(p, loops[i]))
      return false;
  return true;
}
struct Source {
  int edge;
  NurbsCurve world, uv;
};
struct Run {
  size_t source;
  double a, b;
};
using Loop = std::vector<Run>;
using Region = std::vector<Loop>;

// Walk directed trim fragments with the region on their left. Source intervals
// survive intact.
std::vector<Region>
arrange(const std::vector<Source> &sources,
        const std::vector<std::vector<Point>> &original_loops,
        double tolerance) {
  struct Span {
    size_t source;
    double a, b;
    std::vector<double> cuts;
    NurbsCurve curve;
  };
  std::vector<Span> spans;
  for (size_t si = 0; si < sources.size(); ++si) {
    auto knots = sources[si].uv.get_span_vector();
    // A closed Bezier span needs distinct graph nodes on its interior.
    if (knots.size() == 2 && sources[si].uv.is_closed()) {
      double lo = knots.front(), hi = knots.back();
      knots = {lo, lo + (hi - lo) * .25, (lo + hi) * .5, lo + (hi - lo) * .75,
               hi};
    }
    for (size_t i = 1; i < knots.size(); ++i)
      spans.push_back({si,
                       knots[i - 1],
                       knots[i],
                       {knots[i - 1], knots[i]},
                       interval(sources[si].uv, knots[i - 1], knots[i])});
  }
  require(!spans.empty() && spans.size() <= work_limit / spans.size(),
          "Face split exceeds the bounded workload");
  size_t budget = work_limit;
  for (size_t i = 0; i < spans.size(); ++i)
    for (size_t j = i + 1; j < spans.size(); ++j)
      for (auto [a, b] :
           intersections(spans[i].curve, spans[j].curve, tolerance, budget)) {
        spans[i].cuts.push_back(a);
        spans[j].cuts.push_back(b);
      }
  struct Directed {
    size_t a, b;
    Run run;
  };
  std::vector<Point> vertices;
  std::vector<Directed> edges;
  std::vector<std::vector<size_t>> outgoing;
  auto node = [&](const Point &p) {
    for (size_t i = 0; i < vertices.size(); ++i)
      if (distance(p, vertices[i]) <= tolerance * 4.)
        return i;
    vertices.push_back(p);
    outgoing.emplace_back();
    return vertices.size() - 1;
  };
  auto angle = [&](size_t edge) {
    const auto &run = edges[edge].run;
    auto derivative = sources[run.source].uv.evaluate(run.a, 1)[1];
    double sign = run.b > run.a ? 1. : -1.;
    return std::atan2(sign * derivative[1], sign * derivative[0]);
  };
  for (auto &span : spans) {
    for (auto &t : span.cuts) {
      auto p = span.curve.point_at(t);
      if (distance(p, span.curve.point_at(span.a)) <= tolerance)
        t = span.a;
      else if (distance(p, span.curve.point_at(span.b)) <= tolerance)
        t = span.b;
    }
    auto cuts = unique_parameters(span.cuts, span.a, span.b);
    for (size_t i = 1; i < cuts.size(); ++i) {
      double lo = cuts[i - 1], hi = cuts[i];
      const auto &source = sources[span.source];
      if (source.edge < 0 &&
          !inside_loops(source.uv.point_at((lo + hi) * .5), original_loops))
        continue;
      size_t a = node(source.uv.point_at(lo)), b = node(source.uv.point_at(hi));
      if (a == b)
        continue;
      size_t index = edges.size();
      edges.push_back({a, b, {span.source, lo, hi}});
      edges.push_back({b, a, {span.source, hi, lo}});
      outgoing[a].push_back(index);
      outgoing[b].push_back(index + 1);
    }
  }
  for (auto &choices : outgoing)
    std::sort(choices.begin(), choices.end(),
              [&](size_t a, size_t b) { return angle(a) < angle(b); });
  struct Cycle {
    double area;
    Loop loop;
    std::vector<Point> points;
  };
  std::vector<Cycle> cycles;
  std::vector<bool> used(edges.size(), false);
  for (size_t initial = 0; initial < edges.size(); ++initial) {
    if (used[initial])
      continue;
    Loop loop;
    std::vector<Point> points;
    size_t edge = initial;
    while (!used[edge]) {
      used[edge] = true;
      const auto &item = edges[edge];
      const auto &run = item.run;
      loop.push_back(run);
      auto part = interval(sources[run.source].uv, std::min(run.a, run.b),
                           std::max(run.a, run.b));
      if (run.b < run.a)
        part.reverse();
      auto poly = polygon(part, tolerance);
      points.insert(points.end(), poly.begin(), poly.end());
      const auto &options = outgoing[item.b];
      auto at = std::find(options.begin(), options.end(), edge ^ 1);
      require(at != options.end(), "Invalid trim graph adjacency");
      size_t slot = static_cast<size_t>(at - options.begin());
      edge = options[(slot + options.size() - 1) % options.size()];
    }
    require(edge == initial, "Invalid trim graph cycle");
    double area = 0.;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto &a = points[i];
      const auto &b = points[(i + 1) % points.size()];
      area += (a[0] * b[1] - b[0] * a[1]) * .5;
    }
    if (std::abs(area) <= tolerance * tolerance)
      continue;
    const auto &run = loop[0];
    const auto &curve = sources[run.source].uv;
    double t = (run.a + run.b) * .5;
    auto p = curve.point_at(t);
    auto d = curve.evaluate(t, 1)[1];
    double sign = run.b > run.a ? 1. : -1., length = std::hypot(d[0], d[1]);
    require(length > epsilon, "Cannot orient a degenerate trim fragment");
    Point left(p[0] - sign * d[1] / length * tolerance * 8.,
               p[1] + sign * d[0] / length * tolerance * 8., 0.);
    if (!inside_loops(left, original_loops))
      continue;
    cycles.push_back({area, loop, points});
  }
  std::vector<Region> result;
  std::vector<size_t> positive;
  for (size_t i = 0; i < cycles.size(); ++i)
    if (cycles[i].area > 0.) {
      positive.push_back(i);
      result.push_back({cycles[i].loop});
    }
  for (const auto &cycle : cycles) {
    if (cycle.area >= 0.)
      continue;
    size_t parent = result.size();
    double smallest = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < positive.size(); ++i) {
      const auto &outer = cycles[positive[i]];
      if (outer.area > std::abs(cycle.area) + tolerance * tolerance &&
          outer.area < smallest && inside(cycle.points[0], outer.points)) {
        parent = i;
        smallest = outer.area;
      }
    }
    require(parent < result.size(), "Unowned interior trim loop");
    result[parent].push_back(cycle.loop);
  }
  return result;
}

int vertex(BRep &result, const Point &p, double tolerance) {
  for (size_t i = 0; i < result.m_vertices.size(); ++i)
    if (distance(result.m_vertices[i].point, p) <= tolerance)
      return static_cast<int>(i);
  return result.add_vertex(p, tolerance);
}

double lifted_parameter(const NurbsSurface &surface, const NurbsCurve &uv,
                        const Point &p, double expected, double tolerance) {
  auto [lo, hi] = uv.domain();
  auto gap = [&](double t) {
    auto q = uv.point_at(t);
    return distance(surface.point_at(q[0], q[1]), p);
  };
  if (gap(expected) <= tolerance)
    return expected;
  double best = expected, d = gap(best);
  int index = 0;
  for (int i = 0; i <= 128; ++i) {
    double t = lo + (hi - lo) * i / 128., value = gap(t);
    if (value < d) {
      d = value;
      best = t;
      index = i;
    }
  }
  double a = lo + (hi - lo) * std::max(0, index - 1) / 128.,
         b = lo + (hi - lo) * std::min(128, index + 1) / 128.;
  for (int k = 0; k < 60; ++k) {
    double x = a + (b - a) / 3., y = b - (b - a) / 3.;
    if (gap(x) < gap(y))
      b = y;
    else
      a = x;
  }
  double mid = (a + b) * .5;
  if (gap(mid) < d)
    best = mid;
  require(gap(best) <= tolerance * 4.,
          "Cannot keep an adjacent trim on its original shared edge");
  return best;
}

void validate(const BRep &result, const BRep &original, double tolerance) {
  require(result.is_valid(), "Split produced invalid BRep references");
  for (size_t s = 0; s < original.m_shells.size(); ++s)
    if (original.is_closed(static_cast<int>(s)))
      require(result.is_closed(static_cast<int>(s)),
              "Split would open a joined shell");
  for (const auto &face : result.m_faces)
    for (const auto &wr : face.wires) {
      auto edges = result.wire_edges(wr);
      for (size_t i = 0; i < edges.size(); ++i) {
        const auto &a = result.m_edges[edges[i].index];
        const auto &next = edges[(i + 1) % edges.size()];
        const auto &b = result.m_edges[next.index];
        int tail =
            edges[i].orientation == reversed ? a.start_vertex : a.end_vertex;
        int head = next.orientation == reversed ? b.end_vertex : b.start_vertex;
        require(tail == head, "Split produced an open face boundary");
      }
    }
  for (const auto &edge : result.m_edges) {
    if (edge.degenerated)
      continue;
    const auto &world = result.m_curves_3d[edge.curve_3d_index];
    require(distance(world.point_at_start(),
                     result.m_vertices[edge.start_vertex].point) <=
                    tolerance * 4. &&
                distance(world.point_at_end(),
                         result.m_vertices[edge.end_vertex].point) <=
                    tolerance * 4.,
            "Split edge does not meet its vertices");
    for (const auto &pc : edge.pcurves)
      for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2})
        if (ci >= 0) {
          const auto &uv = result.m_curves_2d[ci];
          auto [lo, hi] = uv.domain();
          for (int k = 0; k <= 32; ++k) {
            auto q = uv.point_at(lo + (hi - lo) * k / 32.);
            auto p = result.m_surfaces[pc.surface_index].point_at(q[0], q[1]);
            require(closest(world, p).second <=
                        std::max(tolerance, edge.tolerance) * 8.,
                    "Split edge and surface trim do not coincide");
          }
        }
  }
}
} // namespace

std::vector<NurbsCurve>
split_curve_by_curves(const NurbsCurve &curve,
                      const std::vector<NurbsCurve> &cutters,
                      double tolerance) {
  check_tolerance(tolerance);
  check_curve(curve);
  require(!cutters.empty(), "Select at least one cutter");
  auto [lo, hi] = curve.domain();
  std::vector<double> cuts = {lo, hi};
  bool cut_at_seam = false;
  size_t budget = work_limit;
  for (const auto &cutter : cutters) {
    check_curve(cutter);
    for (auto [a, b] : intersections(curve, cutter, tolerance, budget)) {
      (void)b;
      if (std::abs(a - lo) <= (hi - lo) * epsilon * 16. ||
          std::abs(a - hi) <= (hi - lo) * epsilon * 16.)
        cut_at_seam = true;
      cuts.push_back(a);
    }
  }
  cuts = unique_parameters(cuts, lo, hi);
  std::vector<NurbsCurve> result;
  if (cuts.size() == 2)
    return {curve};
  for (size_t i = 1; i < cuts.size(); ++i)
    result.push_back(interval(curve, cuts[i - 1], cuts[i]));
  // A closed curve's arbitrary storage seam is not an additional cut.
  if (curve.is_closed() && result.size() > 1 && !cut_at_seam) {
    auto joined = NurbsCurve::join({result.back(), result.front()}, tolerance);
    require(joined.size() == 1, "Cannot join the uncut seam of a closed curve");
    result.front() = joined[0];
    result.pop_back();
  }
  return result;
}

BRep split_brep_face_by_curves(const BRep &brep, int face_index,
                               const std::vector<NurbsCurve> &cutters,
                               double tolerance) {
  check_tolerance(tolerance);
  require(brep.is_valid(), "Split requires a valid BRep");
  require(face_index >= 0 && face_index < brep.face_count(),
          "Select one BRep face to split");
  require(!cutters.empty(), "Select at least one cutter");
  const auto &face = brep.m_faces[face_index];
  const auto &surface = brep.m_surfaces[face.surface_index];
  check_surface(surface);
  auto [u0, u1] = surface.domain(0);
  auto [v0, v1] = surface.domain(1);
  double scale = std::max(
      distance(surface.point_at(u0, v0), surface.point_at(u1, v0)) / (u1 - u0),
      distance(surface.point_at(u0, v0), surface.point_at(u0, v1)) / (v1 - v0));
  require(scale > epsilon, "Cannot split a degenerate surface domain");
  double uv_tolerance = tolerance / scale;
  std::vector<Source> sources;
  std::vector<std::vector<Point>> original_loops;
  for (const auto &wr : face.wires) {
    std::vector<Point> points;
    for (const auto &er : brep.wire_edges(wr)) {
      const auto &edge = brep.m_edges[er.index];
      require(!edge.degenerated, "Pole-edge splitting is not supported");
      int ci = brep.pcurve_index(er.index, face_index, er.orientation);
      require(ci >= 0, "Face has no source UV boundary");
      auto uv = brep.m_curves_2d[ci];
      check_curve(uv);
      check_curve(brep.m_curves_3d[edge.curve_3d_index]);
      sources.push_back({er.index, brep.m_curves_3d[edge.curve_3d_index], uv});
      if (er.orientation == reversed)
        uv.reverse();
      auto poly = polygon(uv, uv_tolerance);
      points.insert(points.end(), poly.begin(), poly.end());
    }
    require(points.size() >= 3, "Face has an invalid boundary");
    original_loops.push_back(points);
  }
  for (const auto &cutter : cutters) {
    check_curve(cutter);
    for (const auto &uv : pullback(surface, cutter, tolerance))
      sources.push_back({-1, cutter, uv});
  }
  auto regions = arrange(sources, original_loops, uv_tolerance);
  if (regions.size() < 2)
    return brep;
  BRep result = brep;
  struct Piece {
    size_t source;
    double lo, hi;
    int edge;
  };
  std::vector<Piece> pieces;
  std::map<int, std::vector<std::pair<double, int>>> replacements;
  auto make_edge = [&](const Run &run) -> BRepRef {
    const auto &source = sources[run.source];
    const auto &uv = source.uv;
    auto qa = uv.point_at(run.a), qb = uv.point_at(run.b);
    auto pa = surface.point_at(qa[0], qa[1]),
         pb = surface.point_at(qb[0], qb[1]);
    auto parameter = [&](double t, const Point &p) {
      const auto [lo, hi] = source.world.domain();
      const auto [a, b] = uv.domain();
      double expected = lo + (t - a) / (b - a) * (hi - lo);
      double gap = distance(source.world.point_at(expected), p);
      return gap <= tolerance ? std::pair{expected, gap}
                              : closest(source.world, p);
    };
    auto [wa, da] = parameter(run.a, pa);
    auto [wb, db] = parameter(run.b, pb);
    require(da <= tolerance * 4. && db <= tolerance * 4.,
            "Cutter is not on the selected surface");
    auto [w0, w1] = source.world.domain();
    auto [c0, c1] = uv.domain();
    if (source.world.is_closed()) {
      if (std::abs(wa - w0) < (w1 - w0) * epsilon && run.a > (c0 + c1) * .5)
        wa = w1;
      if (std::abs(wb - w0) < (w1 - w0) * epsilon && run.b > (c0 + c1) * .5)
        wb = w1;
    }
    double lo = std::min(wa, wb), hi = std::max(wa, wb);
    require(hi - lo > (w1 - w0) * epsilon,
            "Split would create a collapsed edge");
    for (const auto &piece : pieces) {
      bool same = source.edge >= 0 ? sources[piece.source].edge == source.edge
                                   : piece.source == run.source;
      if (same &&
          distance(source.world.point_at(lo),
                   source.world.point_at(piece.lo)) <= tolerance * 4. &&
          distance(source.world.point_at(hi),
                   source.world.point_at(piece.hi)) <= tolerance * 4.)
        return {piece.edge, wa < wb ? forward : reversed};
    }
    auto world = interval(source.world, lo, hi);
    int a = vertex(result, world.point_at_start(), tolerance * 4.),
        b = vertex(result, world.point_at_end(), tolerance * 4.);
    int ei = result.add_edge(result.add_curve_3d(world), a, b, tolerance);
    if (source.edge >= 0) {
      const auto &old = brep.m_edges[source.edge];
      for (const auto &pc : old.pcurves) {
        int ids[2] = {-1, -1};
        int at = 0;
        for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
          if (ci >= 0) {
            const auto &c = brep.m_curves_2d[ci];
            auto [c0, c1] = c.domain();
            double ca = lifted_parameter(
                brep.m_surfaces[pc.surface_index], c, world.point_at_start(),
                c0 + (lo - w0) / (w1 - w0) * (c1 - c0), tolerance);
            double cb = lifted_parameter(
                brep.m_surfaces[pc.surface_index], c, world.point_at_end(),
                c0 + (hi - w0) / (w1 - w0) * (c1 - c0), tolerance);
            require(cb > ca,
                    "A split crosses an unsupported periodic trim seam");
            ids[at] = result.add_curve_2d(interval(c, ca, cb));
          }
          ++at;
        }
        result.add_pcurve(ei, pc.surface_index, ids[0], ids[1]);
      }
      replacements[source.edge].push_back({lo, ei});
    } else {
      auto pc = interval(uv, std::min(run.a, run.b), std::max(run.a, run.b));
      if ((wb - wa) * (run.b - run.a) < 0.)
        pc.reverse();
      result.add_pcurve(ei, face.surface_index, result.add_curve_2d(pc));
    }
    pieces.push_back({run.source, lo, hi, ei});
    return {ei, wa < wb ? forward : reversed};
  };
  std::vector<std::vector<BRepRef>> new_wires;
  for (const auto &region : regions) {
    std::vector<BRepRef> wires;
    for (const auto &loop : region) {
      std::vector<BRepRef> refs;
      for (const auto &run : loop)
        refs.push_back(make_edge(run));
      wires.push_back({result.add_wire(refs), forward});
    }
    new_wires.push_back(wires);
  }
  // Every old wire using a subdivided edge receives the same ordered edge
  // pieces.
  for (auto &[edge, items] : replacements) {
    std::sort(items.begin(), items.end());
    items.erase(std::unique(items.begin(), items.end()), items.end());
  }
  for (size_t wi = 0; wi < brep.m_wires.size(); ++wi) {
    std::vector<BRepRef> refs;
    for (const auto &er : brep.m_wires[wi].edges) {
      auto it = replacements.find(er.index);
      if (it == replacements.end()) {
        refs.push_back(er);
        continue;
      }
      auto items = it->second;
      if (er.orientation == reversed)
        std::reverse(items.begin(), items.end());
      for (auto [t, e] : items) {
        (void)t;
        refs.push_back({e, er.orientation});
      }
    }
    result.m_wires[wi].edges = refs;
  }
  result.m_faces[face_index].wires = new_wires[0];
  std::vector<int> added;
  for (size_t i = 1; i < new_wires.size(); ++i) {
    auto next = face;
    next.wires = new_wires[i];
    added.push_back(result.face_count());
    result.m_faces.push_back(next);
  }
  for (auto &shell : result.m_shells) {
    std::vector<BRepRef> refs;
    for (const auto &fr : shell.faces) {
      refs.push_back(fr);
      if (fr.index == face_index)
        for (int index : added)
          refs.push_back({index, fr.orientation});
    }
    shell.faces = refs;
  }
  validate(result, brep, tolerance);
  return result;
}

BRep split_surface_by_curves(const NurbsSurface &surface,
                             const std::vector<NurbsCurve> &cutters,
                             double tolerance) {
  check_tolerance(tolerance);
  check_surface(surface);
  BRep result;
  int si = result.add_surface(surface);
  auto [u0, u1] = surface.domain(0);
  auto [v0, v1] = surface.domain(1);
  std::vector<Point> uv = {Point(u0, v0, 0), Point(u1, v0, 0), Point(u1, v1, 0),
                           Point(u0, v1, 0)};
  std::vector<BRepRef> edges;
  for (int i = 0; i < 4; ++i) {
    auto curve = surface.iso_curve(i % 2 == 0 ? 0 : 1, i == 0   ? v0
                                                       : i == 1 ? u1
                                                       : i == 2 ? v1
                                                                : u0);
    if (i >= 2)
      curve.reverse();
    int a = vertex(result, curve.point_at_start(), tolerance),
        b = vertex(result, curve.point_at_end(), tolerance);
    require(
        a != b,
        "Closed or pole boundaries need a BRep with explicit seam topology");
    int edge = result.add_edge(result.add_curve_3d(curve), a, b, tolerance);
    auto pc = NurbsCurve::create(false, 1, {uv[i], uv[(i + 1) % 4]});
    result.add_pcurve(edge, si, result.add_curve_2d(pc));
    edges.push_back({edge, forward});
  }
  result.add_face(si, {{result.add_wire(edges), forward}});
  return split_brep_face_by_curves(result, 0, cutters, tolerance);
}
std::vector<Line> split_line_by_curves(const Line &line,
                                       const std::vector<NurbsCurve> &cutters,
                                       double tolerance) {
  auto curve =
      NurbsCurve::create(false, 1, {line.point_at(0), line.point_at(1)});
  std::vector<Line> result;
  for (const auto &piece : split_curve_by_curves(curve, cutters, tolerance)) {
    auto next = Line::from_points(piece.point_at_start(), piece.point_at_end());
    next.name = line.name;
    next.width = line.width;
    next.dash = line.dash;
    next.linecolor = line.linecolor;
    result.push_back(std::move(next));
  }
  return result;
}
std::vector<Polyline>
split_polyline_by_curves(const Polyline &polyline,
                         const std::vector<NurbsCurve> &cutters,
                         double tolerance) {
  auto curve = NurbsCurve::create(false, 1, polyline.get_points());
  std::vector<Polyline> result;
  for (const auto &piece : split_curve_by_curves(curve, cutters, tolerance)) {
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
