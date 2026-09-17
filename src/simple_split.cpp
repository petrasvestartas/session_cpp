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
    const Point p = curve.get_cv(i);
    require(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
                std::isfinite(curve.weight(i)) && curve.weight(i) > 0.,
            "Split requires finite controls and positive rational weights");
  }
}
void check_surface(const NurbsSurface &surface) {
  require(surface.is_valid(), "Split requires a valid NURBS surface");
  for (int i = 0; i < surface.cv_count(0); ++i)
    for (int j = 0; j < surface.cv_count(1); ++j) {
      const Point p = surface.get_cv(i, j);
      const double w = surface.weight(i, j);
      require(std::isfinite(p[0]) && std::isfinite(p[1]) &&
                  std::isfinite(p[2]) && std::isfinite(w) && w > 0.,
              "Split requires finite surface controls and positive rational "
              "weights");
    }
}
NurbsCurve interval(const NurbsCurve &curve, double a, double b) {
  NurbsCurve result = curve;
  result.refresh_guid();
  const auto [lo, hi] = curve.domain();
  a = std::clamp(a, lo, hi);
  b = std::clamp(b, lo, hi);
  require(b > a, "Split produced an empty curve interval");
  if (a > lo || b < hi)
    require(result.trim(a, b), "Kernel refused a split interval");
  return result;
}
std::pair<double, double> closest(const NurbsCurve &curve, const Point &point) {
  auto [t, gap] = Closest::curve_point(curve, point);
  const auto [lo, hi] = curve.domain();
  if (curve.degree() == 1) {
    const std::vector<double> spans = curve.get_span_vector();
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < spans.size(); ++i) {
      const Point a = curve.point_at(spans[i - 1]);
      const Point b = curve.point_at(spans[i]);
      const Vector v = b - a;
      const double length2 = v.dot(v);
      if (length2 <= epsilon * epsilon)
        continue;
      const double fraction = std::clamp((point - a).dot(v) / length2, 0., 1.);
      const NurbsCurve segment = interval(curve, spans[i - 1], spans[i]);
      const double w0 = segment.weight(0);
      const double w1 = segment.weight(segment.cv_count() - 1);
      const double normalized =
          fraction * w0 / (w1 * (1. - fraction) + fraction * w0);
      const double candidate =
          spans[i - 1] + normalized * (spans[i] - spans[i - 1]);
      gap = curve.point_at(candidate).distance(point);
      if (gap < best) {
        best = gap;
        t = candidate;
      }
    }
    return {t, curve.point_at(t).distance(point)};
  }
  for (int i = 0; i < 24; ++i) {
    const std::vector<Vector> eval = curve.evaluate(t, 1);
    const Vector d = eval[1];
    const Vector r = eval[0] - Vector(point[0], point[1], point[2]);
    const double dd = d.dot(d);
    if (dd <= epsilon * epsilon)
      break;
    const double next = std::clamp(t - d.dot(r) / dd, lo, hi);
    if (std::abs(next - t) <= epsilon * (hi - lo)) {
      t = next;
      break;
    }
    t = next;
  }
  return {t, curve.point_at(t).distance(point)};
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
  std::array<double, 3> lo;
  std::array<double, 3> hi;
  explicit Box(const NurbsCurve &curve) {
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
  const Point a = curve.point_at_start();
  const Point b = curve.point_at_end();
  const Vector v = b - a;
  const double length2 = v.dot(v);
  if (length2 <= tolerance * tolerance)
    return Box(curve).diagonal() <= tolerance;
  for (int i = 0; i < curve.cv_count(); ++i) {
    const Point p = curve.get_cv(i);
    const double t = (p - a).dot(v) / length2;
    if (t < -epsilon || t > 1. + epsilon || p.distance(a + v * t) > tolerance)
      return false;
  }
  return true;
}
std::pair<double, double> refine(const NurbsCurve &a, const NurbsCurve &b,
                                 double ta, double tb) {
  const auto [a0, a1] = a.domain();
  const auto [b0, b1] = b.domain();
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
    if (det <= epsilon * epsilon * aa * bb)
      break;
    const double ar = u.dot(r);
    const double br = v.dot(r);
    const double na = std::clamp(ta + (-bb * ar + ab * br) / det, a0, a1);
    const double nb = std::clamp(tb + (-ab * ar + aa * br) / det, b0, b1);
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
    NurbsCurve a;
    NurbsCurve b;
    int depth;
  };
  std::vector<Pair> work;
  const std::vector<double> av = a.get_span_vector();
  const std::vector<double> bv = b.get_span_vector();
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
    const Pair pair = std::move(work.back());
    work.pop_back();
    const Box ba(pair.a);
    const Box bb(pair.b);
    if (!ba.overlaps(bb, tolerance))
      continue;
    if ((flat(pair.a, tolerance * .1) && flat(pair.b, tolerance * .1)) ||
        pair.depth >= 48) {
      const Point ap = pair.a.point_at_start();
      const Point aq = pair.a.point_at_end();
      const Point bp = pair.b.point_at_start();
      const Point bq = pair.b.point_at_end();
      const Vector u = aq - ap;
      const Vector v = bq - bp;
      const double aa = u.dot(u);
      const double ab = u.dot(v);
      const double vv = v.dot(v);
      if (aa > tolerance * tolerance && vv > tolerance * tolerance &&
          aa * vv - ab * ab < epsilon * epsilon * aa * vv) {
        const double t0 = (bp - ap).dot(u) / aa;
        const double t1 = (bq - ap).dot(u) / aa;
        const double gap = bp.distance(ap + u * t0);
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
      if (a.point_at(ta).distance(b.point_at(tb)) > tolerance)
        continue;
      bool duplicate = false;
      for (const auto &hit : hits)
        if (a.point_at(hit.first).distance(a.point_at(ta)) <= tolerance * 2. &&
            a.point_at((hit.first + ta) * .5).distance(a.point_at(ta)) <=
                tolerance * 2. &&
            b.point_at(hit.second).distance(b.point_at(tb)) <=
                tolerance * 2. &&
            b.point_at((hit.second + tb) * .5).distance(b.point_at(tb)) <=
                tolerance * 2.) {
          duplicate = true;
          break;
        }
      if (!duplicate)
        hits.emplace_back(ta, tb);
      continue;
    }
    if (ba.diagonal() >= bb.diagonal()) {
      const auto [lo, hi] = pair.a.domain();
      const double mid = (lo + hi) * .5;
      work.push_back({interval(pair.a, lo, mid), pair.b, pair.depth + 1});
      work.push_back({interval(pair.a, mid, hi), pair.b, pair.depth + 1});
    } else {
      const auto [lo, hi] = pair.b.domain();
      const double mid = (lo + hi) * .5;
      work.push_back({pair.a, interval(pair.b, lo, mid), pair.depth + 1});
      work.push_back({pair.a, interval(pair.b, mid, hi), pair.depth + 1});
    }
  }
  std::sort(hits.begin(), hits.end());
  return hits;
}

std::vector<NurbsCurve> pullback(const NurbsSurface &surface,
                                 const NurbsCurve &curve, double tolerance) {
  if (surface.m_cv_count[0] == 2 && surface.m_cv_count[1] == 2 &&
      surface.m_order[0] == 2 && surface.m_order[1] == 2 && !surface.m_is_rat) {
    const Point p = surface.get_cv(0, 0);
    const Vector u = surface.get_cv(1, 0) - p;
    const Vector v = surface.get_cv(0, 1) - p;
    const Point last = surface.get_cv(1, 1);
    const double uu = u.dot(u);
    const double uv = u.dot(v);
    const double vv = v.dot(v);
    const double det = uu * vv - uv * uv;
    if (det > epsilon * epsilon * uu * vv &&
        last.distance(p + u + v) <= tolerance) {
      NurbsCurve result = curve;
      result.refresh_guid();
      const auto [u0, u1] = surface.domain(0);
      const auto [v0, v1] = surface.domain(1);
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
  const std::vector<double> spans = curve.get_span_vector();
  for (size_t i = spans.size(); i > 1; --i)
    work.push_back({interval(curve, spans[i - 2], spans[i - 1]), 0});
  std::vector<Point> result;
  size_t visited = 0;
  while (!work.empty()) {
    require(++visited <= work_limit,
            "Trim sampling exceeds the bounded workload");
    const Part part = std::move(work.back());
    work.pop_back();
    if (flat(part.curve, tolerance * .25)) {
      result.push_back(part.curve.point_at_start());
      continue;
    }
    require(part.depth < 40, "Trim sampling exceeds parameter precision");
    const auto [lo, hi] = part.curve.domain();
    const double mid = (lo + hi) * .5;
    work.push_back({interval(part.curve, mid, hi), part.depth + 1});
    work.push_back({interval(part.curve, lo, mid), part.depth + 1});
  }
  return result;
}
bool inside(const Point &p, const std::vector<Point> &polygon) {
  bool result = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Point &a = polygon[i];
    const Point &b = polygon[j];
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
  NurbsCurve world;
  NurbsCurve uv;
};
struct Run {
  size_t source;
  double a;
  double b;
};
using Loop = std::vector<Run>;
using Region = std::vector<Loop>;

std::vector<Region>
arrange(const std::vector<Source> &sources,
        const std::vector<std::vector<Point>> &original_loops,
        double tolerance) {
  struct Span {
    size_t source;
    double a;
    double b;
    std::vector<double> cuts;
    NurbsCurve curve;
  };
  std::vector<Span> spans;
  for (size_t si = 0; si < sources.size(); ++si) {
    std::vector<double> knots = sources[si].uv.get_span_vector();
    if (knots.size() == 2 && sources[si].uv.is_closed()) {
      const double lo = knots.front();
      const double hi = knots.back();
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
    size_t a;
    size_t b;
    Run run;
  };
  std::vector<Point> vertices;
  std::vector<Directed> edges;
  std::vector<std::vector<size_t>> outgoing;
  auto node = [&](const Point &p) {
    for (size_t i = 0; i < vertices.size(); ++i)
      if (p.distance(vertices[i]) <= tolerance * 4.)
        return i;
    vertices.push_back(p);
    outgoing.emplace_back();
    return vertices.size() - 1;
  };
  auto angle = [&](size_t edge) {
    const Run &run = edges[edge].run;
    const Vector d = sources[run.source].uv.evaluate(run.a, 1)[1];
    const double sign = run.b > run.a ? 1. : -1.;
    return std::atan2(sign * d[1], sign * d[0]);
  };
  for (Span &span : spans) {
    for (double &t : span.cuts) {
      const Point p = span.curve.point_at(t);
      if (p.distance(span.curve.point_at(span.a)) <= tolerance)
        t = span.a;
      else if (p.distance(span.curve.point_at(span.b)) <= tolerance)
        t = span.b;
    }
    const std::vector<double> cuts = unique_parameters(span.cuts, span.a, span.b);
    for (size_t i = 1; i < cuts.size(); ++i) {
      const double lo = cuts[i - 1];
      const double hi = cuts[i];
      const Source &source = sources[span.source];
      if (source.edge < 0 &&
          !inside_loops(source.uv.point_at((lo + hi) * .5), original_loops))
        continue;
      const size_t a = node(source.uv.point_at(lo));
      const size_t b = node(source.uv.point_at(hi));
      if (a == b)
        continue;
      const size_t index = edges.size();
      edges.push_back({a, b, {span.source, lo, hi}});
      edges.push_back({b, a, {span.source, hi, lo}});
      outgoing[a].push_back(index);
      outgoing[b].push_back(index + 1);
    }
  }
  for (std::vector<size_t> &choices : outgoing)
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
      const Directed &item = edges[edge];
      const Run &run = item.run;
      loop.push_back(run);
      NurbsCurve part = interval(sources[run.source].uv, std::min(run.a, run.b),
                                 std::max(run.a, run.b));
      if (run.b < run.a)
        part.reverse();
      const std::vector<Point> poly = polygon(part, tolerance);
      points.insert(points.end(), poly.begin(), poly.end());
      const std::vector<size_t> &options = outgoing[item.b];
      const auto at = std::find(options.begin(), options.end(), edge ^ 1);
      require(at != options.end(), "Invalid trim graph adjacency");
      const size_t slot = static_cast<size_t>(at - options.begin());
      edge = options[(slot + options.size() - 1) % options.size()];
    }
    require(edge == initial, "Invalid trim graph cycle");
    double area = 0.;
    for (size_t i = 0; i < points.size(); ++i) {
      const Point &a = points[i];
      const Point &b = points[(i + 1) % points.size()];
      area += (a[0] * b[1] - b[0] * a[1]) * .5;
    }
    if (std::abs(area) <= tolerance * tolerance)
      continue;
    const Run &run = loop[0];
    const NurbsCurve &curve = sources[run.source].uv;
    const double t = (run.a + run.b) * .5;
    const Point p = curve.point_at(t);
    const Vector d = curve.evaluate(t, 1)[1];
    const double sign = run.b > run.a ? 1. : -1.;
    const double length = std::hypot(d[0], d[1]);
    require(length > epsilon, "Cannot orient a degenerate trim fragment");
    const Point left(p[0] - sign * d[1] / length * tolerance * 8.,
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
  for (const Cycle &cycle : cycles) {
    if (cycle.area >= 0.)
      continue;
    size_t parent = result.size();
    double smallest = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < positive.size(); ++i) {
      const Cycle &outer = cycles[positive[i]];
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
    if (result.m_vertices[i].point.distance(p) <= tolerance)
      return static_cast<int>(i);
  return result.add_vertex(p, tolerance);
}

double lifted_parameter(const NurbsSurface &surface, const NurbsCurve &uv,
                        const Point &p, double expected, double tolerance) {
  const auto [lo, hi] = uv.domain();
  auto gap = [&](double t) {
    const Point q = uv.point_at(t);
    return surface.point_at(q[0], q[1]).distance(p);
  };
  if (gap(expected) <= tolerance)
    return expected;
  double best = expected;
  double d = gap(best);
  int index = 0;
  for (int i = 0; i <= 128; ++i) {
    const double t = lo + (hi - lo) * i / 128.;
    const double value = gap(t);
    if (value < d) {
      d = value;
      best = t;
      index = i;
    }
  }
  double a = lo + (hi - lo) * std::max(0, index - 1) / 128.;
  double b = lo + (hi - lo) * std::min(128, index + 1) / 128.;
  for (int k = 0; k < 60; ++k) {
    const double x = a + (b - a) / 3.;
    const double y = b - (b - a) / 3.;
    if (gap(x) < gap(y))
      b = y;
    else
      a = x;
  }
  const double mid = (a + b) * .5;
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
  for (const BRepFace &face : result.m_faces)
    for (const BRepRef &wr : face.wires) {
      const std::vector<BRepRef> edges = result.wire_edges(wr);
      for (size_t i = 0; i < edges.size(); ++i) {
        const BRepEdge &a = result.m_edges[edges[i].index];
        const BRepRef &next = edges[(i + 1) % edges.size()];
        const BRepEdge &b = result.m_edges[next.index];
        const int tail =
            edges[i].orientation == reversed ? a.start_vertex : a.end_vertex;
        const int head =
            next.orientation == reversed ? b.end_vertex : b.start_vertex;
        require(tail == head, "Split produced an open face boundary");
      }
    }
  for (const BRepEdge &edge : result.m_edges) {
    if (edge.degenerated)
      continue;
    const NurbsCurve &world = result.m_curves_3d[edge.curve_3d_index];
    require(world.point_at_start().distance(
                result.m_vertices[edge.start_vertex].point) <= tolerance * 4. &&
                world.point_at_end().distance(
                    result.m_vertices[edge.end_vertex].point) <= tolerance * 4.,
            "Split edge does not meet its vertices");
    for (const auto &pc : edge.pcurves)
      for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
        if (ci < 0)
          continue;
        const NurbsCurve &uv = result.m_curves_2d[ci];
        const auto [lo, hi] = uv.domain();
        for (int k = 0; k <= 32; ++k) {
          const Point q = uv.point_at(lo + (hi - lo) * k / 32.);
          const Point p =
              result.m_surfaces[pc.surface_index].point_at(q[0], q[1]);
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
  const auto [lo, hi] = curve.domain();
  std::vector<double> cuts = {lo, hi};
  bool cut_at_seam = false;
  size_t budget = work_limit;
  for (const NurbsCurve &cutter : cutters) {
    check_curve(cutter);
    for (const auto &hit : intersections(curve, cutter, tolerance, budget)) {
      const double a = hit.first;
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
  if (curve.is_closed() && result.size() > 1 && !cut_at_seam) {
    const std::vector<NurbsCurve> joined =
        NurbsCurve::join({result.back(), result.front()}, tolerance);
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
  const BRepFace &face = brep.m_faces[face_index];
  const NurbsSurface &surface = brep.m_surfaces[face.surface_index];
  check_surface(surface);
  const auto [u0, u1] = surface.domain(0);
  const auto [v0, v1] = surface.domain(1);
  const Point origin = surface.point_at(u0, v0);
  const double scale =
      std::max(origin.distance(surface.point_at(u1, v0)) / (u1 - u0),
               origin.distance(surface.point_at(u0, v1)) / (v1 - v0));
  require(scale > epsilon, "Cannot split a degenerate surface domain");
  const double uv_tolerance = tolerance / scale;
  std::vector<Source> sources;
  std::vector<std::vector<Point>> original_loops;
  for (const BRepRef &wr : face.wires) {
    std::vector<Point> points;
    for (const BRepRef &er : brep.wire_edges(wr)) {
      const BRepEdge &edge = brep.m_edges[er.index];
      require(!edge.degenerated, "Pole-edge splitting is not supported");
      const int ci = brep.pcurve_index(er.index, face_index, er.orientation);
      require(ci >= 0, "Face has no source UV boundary");
      NurbsCurve uv = brep.m_curves_2d[ci];
      check_curve(uv);
      check_curve(brep.m_curves_3d[edge.curve_3d_index]);
      sources.push_back({er.index, brep.m_curves_3d[edge.curve_3d_index], uv});
      if (er.orientation == reversed)
        uv.reverse();
      const std::vector<Point> poly = polygon(uv, uv_tolerance);
      points.insert(points.end(), poly.begin(), poly.end());
    }
    require(points.size() >= 3, "Face has an invalid boundary");
    original_loops.push_back(points);
  }
  for (const NurbsCurve &cutter : cutters) {
    check_curve(cutter);
    for (const NurbsCurve &uv : pullback(surface, cutter, tolerance))
      sources.push_back({-1, cutter, uv});
  }
  const std::vector<Region> regions =
      arrange(sources, original_loops, uv_tolerance);
  if (regions.size() < 2)
    return brep;
  BRep result = brep;
  struct Piece {
    size_t source;
    double lo;
    double hi;
    int edge;
  };
  std::vector<Piece> pieces;
  std::map<int, std::vector<std::pair<double, int>>> replacements;
  auto make_edge = [&](const Run &run) -> BRepRef {
    const Source &source = sources[run.source];
    const NurbsCurve &uv = source.uv;
    const Point qa = uv.point_at(run.a);
    const Point qb = uv.point_at(run.b);
    const Point pa = surface.point_at(qa[0], qa[1]);
    const Point pb = surface.point_at(qb[0], qb[1]);
    auto parameter = [&](double t, const Point &p) {
      const auto [lo, hi] = source.world.domain();
      const auto [a, b] = uv.domain();
      const double expected = lo + (t - a) / (b - a) * (hi - lo);
      const double gap = source.world.point_at(expected).distance(p);
      return gap <= tolerance ? std::pair{expected, gap}
                              : closest(source.world, p);
    };
    auto [wa, da] = parameter(run.a, pa);
    auto [wb, db] = parameter(run.b, pb);
    require(da <= tolerance * 4. && db <= tolerance * 4.,
            "Cutter is not on the selected surface");
    const auto [w0, w1] = source.world.domain();
    const auto [c0, c1] = uv.domain();
    if (source.world.is_closed()) {
      if (std::abs(wa - w0) < (w1 - w0) * epsilon && run.a > (c0 + c1) * .5)
        wa = w1;
      if (std::abs(wb - w0) < (w1 - w0) * epsilon && run.b > (c0 + c1) * .5)
        wb = w1;
    }
    const double lo = std::min(wa, wb);
    const double hi = std::max(wa, wb);
    require(hi - lo > (w1 - w0) * epsilon,
            "Split would create a collapsed edge");
    const BRepOrientation orientation = wa < wb ? forward : reversed;
    for (const Piece &piece : pieces) {
      const bool same = source.edge >= 0
                            ? sources[piece.source].edge == source.edge
                            : piece.source == run.source;
      if (same &&
          source.world.point_at(lo).distance(source.world.point_at(piece.lo)) <=
              tolerance * 4. &&
          source.world.point_at(hi).distance(source.world.point_at(piece.hi)) <=
              tolerance * 4.)
        return {piece.edge, orientation};
    }
    const NurbsCurve world = interval(source.world, lo, hi);
    const int a = vertex(result, world.point_at_start(), tolerance * 4.);
    const int b = vertex(result, world.point_at_end(), tolerance * 4.);
    const int ei = result.add_edge(result.add_curve_3d(world), a, b, tolerance);
    if (source.edge >= 0) {
      const BRepEdge &old = brep.m_edges[source.edge];
      for (const auto &pc : old.pcurves) {
        int ids[2] = {-1, -1};
        int at = 0;
        for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
          if (ci >= 0) {
            const NurbsCurve &c = brep.m_curves_2d[ci];
            const auto [c0, c1] = c.domain();
            const double ca = lifted_parameter(
                brep.m_surfaces[pc.surface_index], c, world.point_at_start(),
                c0 + (lo - w0) / (w1 - w0) * (c1 - c0), tolerance);
            const double cb = lifted_parameter(
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
      NurbsCurve pc =
          interval(uv, std::min(run.a, run.b), std::max(run.a, run.b));
      if ((wb - wa) * (run.b - run.a) < 0.)
        pc.reverse();
      result.add_pcurve(ei, face.surface_index, result.add_curve_2d(pc));
    }
    pieces.push_back({run.source, lo, hi, ei});
    return {ei, orientation};
  };
  std::vector<std::vector<BRepRef>> new_wires;
  for (const Region &region : regions) {
    std::vector<BRepRef> wires;
    for (const Loop &loop : region) {
      std::vector<BRepRef> refs;
      for (const Run &run : loop)
        refs.push_back(make_edge(run));
      wires.push_back({result.add_wire(refs), forward});
    }
    new_wires.push_back(wires);
  }
  for (auto &[edge, items] : replacements) {
    std::sort(items.begin(), items.end());
    items.erase(std::unique(items.begin(), items.end()), items.end());
  }
  for (size_t wi = 0; wi < brep.m_wires.size(); ++wi) {
    std::vector<BRepRef> refs;
    for (const BRepRef &er : brep.m_wires[wi].edges) {
      const auto it = replacements.find(er.index);
      if (it == replacements.end()) {
        refs.push_back(er);
        continue;
      }
      std::vector<std::pair<double, int>> items = it->second;
      if (er.orientation == reversed)
        std::reverse(items.begin(), items.end());
      for (const auto &item : items)
        refs.push_back({item.second, er.orientation});
    }
    result.m_wires[wi].edges = refs;
  }
  result.m_faces[face_index].wires = new_wires[0];
  std::vector<int> added;
  for (size_t i = 1; i < new_wires.size(); ++i) {
    BRepFace next = face;
    next.wires = new_wires[i];
    added.push_back(result.face_count());
    result.m_faces.push_back(next);
  }
  for (BRepShell &shell : result.m_shells) {
    std::vector<BRepRef> refs;
    for (const BRepRef &fr : shell.faces) {
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
  const int si = result.add_surface(surface);
  const auto [u0, u1] = surface.domain(0);
  const auto [v0, v1] = surface.domain(1);
  const std::vector<Point> uv = {Point(u0, v0, 0), Point(u1, v0, 0),
                                 Point(u1, v1, 0), Point(u0, v1, 0)};
  const double at[4] = {v0, u1, v1, u0};
  std::vector<BRepRef> edges;
  for (int i = 0; i < 4; ++i) {
    NurbsCurve curve = surface.iso_curve(i % 2, at[i]);
    if (i >= 2)
      curve.reverse();
    const int a = vertex(result, curve.point_at_start(), tolerance);
    const int b = vertex(result, curve.point_at_end(), tolerance);
    require(
        a != b,
        "Closed or pole boundaries need a BRep with explicit seam topology");
    const int edge = result.add_edge(result.add_curve_3d(curve), a, b, tolerance);
    const NurbsCurve pc = NurbsCurve::create(false, 1, {uv[i], uv[(i + 1) % 4]});
    result.add_pcurve(edge, si, result.add_curve_2d(pc));
    edges.push_back({edge, forward});
  }
  result.add_face(si, {{result.add_wire(edges), forward}});
  return split_brep_face_by_curves(result, 0, cutters, tolerance);
}
std::vector<Line> split_line_by_curves(const Line &line,
                                       const std::vector<NurbsCurve> &cutters,
                                       double tolerance) {
  const NurbsCurve curve =
      NurbsCurve::create(false, 1, {line.point_at(0), line.point_at(1)});
  std::vector<Line> result;
  for (const NurbsCurve &piece :
       split_curve_by_curves(curve, cutters, tolerance)) {
    Line next = Line::from_points(piece.point_at_start(), piece.point_at_end());
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
  const NurbsCurve curve = NurbsCurve::create(false, 1, polyline.get_points());
  std::vector<Polyline> result;
  for (const NurbsCurve &piece :
       split_curve_by_curves(curve, cutters, tolerance)) {
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
