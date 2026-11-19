#include "bop_builder_face.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <tuple>

namespace session_cpp::bop {
namespace {

double heading(const Span& c, bool at_end) {
    const Point end = c.at(at_end ? 1.0 : 0.0);
    for (double f : {1e-4, 1e-3, 1e-2, 1e-1}) {
        const Point q = c.at(at_end ? 1.0 - f : f);
        const double dx = at_end ? end[0] - q[0] : q[0] - end[0], dy = at_end ? end[1] - q[1] : q[1] - end[1];
        if (std::hypot(dx, dy) > 1e-13)
            return std::atan2(dy, dx);
    }
    return 0.0;
}

double area(const std::vector<Point>& polygon) {
    double sum = 0.0;
    for (size_t k = 0; k < polygon.size(); ++k) {
        const Point& a = polygon[k];
        const Point& b = polygon[(k + 1) % polygon.size()];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    return 0.5 * sum;
}

}

std::vector<Point> loop_polygon(const std::vector<EdgeUse>& uses, const std::vector<int>& loop) {
    std::vector<Point> polygon;
    for (int i : loop) {
        const Span& c = uses[i].pcurve;
        const int n = std::clamp(4 * c.curve->nurbs().cv_count(), 8, 256);
        for (int k = 0; k < n; ++k)
            polygon.push_back(c.at(double(k) / n));
    }
    return polygon;
}

std::vector<std::vector<std::vector<int>>> build_faces(std::vector<EdgeUse>& uses, const Region& region, std::vector<std::string>& log) {
    const size_t n = uses.size();
    for (EdgeUse& use : uses) {
        if (!use.internal)
            continue;
        const Point mid = use.pcurve.at(0.5);
        for (int d = 0; d < 2; ++d)
            if (region.period[d] > 0.0)
                use.pcurve.shift[d] -= std::floor((mid[d] - region.lo[d] + region.eps) / region.period[d]) * region.period[d];
    }
    std::vector<Point> p0(n), p1(n);
    std::vector<double> out(n), in(n);
    std::vector<int> twin(n, -1);
    for (size_t i = 0; i < n; ++i) {
        p0[i] = uses[i].pcurve.at(0.0);
        p1[i] = uses[i].pcurve.at(1.0);
        out[i] = heading(uses[i].pcurve, false);
        in[i] = heading(uses[i].pcurve, true);
        for (size_t j = 0; j < n; ++j)
            if (j != i && uses[i].internal && uses[j].internal && uses[j].image == uses[i].image && uses[j].reversed != uses[i].reversed)
                twin[i] = static_cast<int>(j);
    }
    const double diag = std::hypot(region.hi[0] - region.lo[0], region.hi[1] - region.lo[1]);
    const double near = 1e-6 * diag + 10.0 * region.eps, node = 1e-3 * diag + 10.0 * region.eps;
    std::vector<bool> alive(n, true);
    // Ends of uses at one vertex and one place of the surface: the same uv up to whole periods.
    const auto apart = [&](const Point& p, const Point& q) {
        double d[2];
        for (int k = 0; k < 2; ++k)
            d[k] = region.period[k] > 0.0 ? std::remainder(p[k] - q[k], region.period[k]) : p[k] - q[k];
        return std::hypot(d[0], d[1]);
    };
    const auto degree = [&](int v, const Point& q) {
        int count = 0;
        for (size_t k = 0; k < n; ++k)
            if (alive[k])
                count += (uses[k].start == v && apart(p0[k], q) <= node + uses[k].gap) + (uses[k].end == v && apart(p1[k], q) <= node + uses[k].gap);
        return count;
    };
    for (bool pruned = true; pruned;) {
        pruned = false;
        for (size_t i = 0; i < n; ++i)
            if (alive[i] && uses[i].internal && twin[i] >= 0
                && (degree(uses[i].start, p0[i]) <= 2 || degree(uses[i].end, p1[i]) <= 2)) {
                alive[i] = alive[twin[i]] = false;
                pruned = true;
            }
    }
    std::vector<bool> used(n, false);
    std::vector<std::vector<int>> loops;
    for (size_t s = 0; s < n; ++s) {
        if (used[s] || !alive[s])
            continue;
        std::vector<int> loop{static_cast<int>(s)};
        used[s] = true;
        int cur = static_cast<int>(s);
        bool closed = false;
        for (size_t step = 0; step <= n; ++step) {
            // The next use leaves from where this one arrives; where only its own return does, the surface is periodic there and the others leave a whole period away.
            double onward = 1e300, nearest = 1e300;
            for (size_t k = 0; k < n; ++k)
                if (alive[k] && (!used[k] || k == s) && uses[k].start == uses[cur].end && static_cast<int>(k) != twin[cur])
                    onward = std::min(onward, p0[k].distance(p1[cur]));
            const bool wrapped = onward > node + uses[cur].gap;
            const auto distance = [&](size_t k) { return wrapped ? apart(p0[k], p1[cur]) : p0[k].distance(p1[cur]); };
            for (size_t k = 0; k < n; ++k)
                if (alive[k] && (!used[k] || k == s) && uses[k].start == uses[cur].end)
                    nearest = std::min(nearest, distance(k));
            int best = -1;
            double best_angle = 1e300;
            for (size_t k = 0; k < n; ++k) {
                if (!alive[k] || (used[k] && k != s) || uses[k].start != uses[cur].end || distance(k) > nearest + near + uses[k].gap + uses[cur].gap)
                    continue;
                double angle = std::fmod(in[cur] + std::numbers::pi - out[k], 2.0 * std::numbers::pi);
                if (angle < 0.0)
                    angle += 2.0 * std::numbers::pi;
                if (angle < 1e-12)
                    angle = 2.0 * std::numbers::pi;
                if (static_cast<int>(k) == twin[cur])
                    angle = 3.0 * std::numbers::pi;
                if (angle < best_angle) {
                    best_angle = angle;
                    best = static_cast<int>(k);
                }
            }
            if (best < 0)
                break;
            if (best == static_cast<int>(s)) {
                closed = true;
                break;
            }
            used[best] = true;
            loop.push_back(best);
            cur = best;
        }
        if (closed)
            loops.push_back(std::move(loop));
        else
            log.push_back("face split: open loop");
    }
    std::vector<std::vector<Point>> polygons;
    std::vector<double> areas;
    for (const auto& loop : loops) {
        polygons.push_back(loop_polygon(uses, loop));
        areas.push_back(area(polygons.back()));
    }
    const double small = 1e-12 * diag * diag;
    const auto images = [&](const std::vector<int>& loop) {
        std::vector<int> result;
        for (int k : loop)
            result.push_back(uses[k].image);
        std::sort(result.begin(), result.end());
        return result;
    };
    std::vector<std::vector<std::vector<int>>> faces;
    std::vector<int> outer_of;
    // A loop that runs once around a periodic surface closes no area: with the next such loop across the period it bounds a band (the face lies to its left).
    std::vector<int> winds(loops.size(), -1);
    std::vector<std::tuple<double, bool, int>> around; // level across the period, whether the face lies above, loop
    for (size_t i = 0; i < loops.size(); ++i) {
        winds[i] = region.winding(p0[loops[i].front()], p1[loops[i].back()]);
        if (winds[i] < 0)
            continue;
        polygons[i].push_back(p1[loops[i].back()]);
        double level = 0.0;
        for (const Point& p : polygons[i])
            level += p[1 - winds[i]] / polygons[i].size();
        around.push_back({level, (p1[loops[i].back()][winds[i]] > p0[loops[i].front()][winds[i]]) == (winds[i] == 0), static_cast<int>(i)});
    }
    std::sort(around.begin(), around.end());
    std::vector<Region> bands;
    for (size_t k = 0; k < around.size(); ++k) {
        const auto [level, below, i] = around[k];
        const int d = winds[i], partner = below && k + 1 < around.size() && !std::get<1>(around[k + 1]) ? std::get<2>(around[k + 1]) : -1;
        if (partner >= 0 && winds[partner] == d) {
            outer_of.push_back(-static_cast<int>(bands.size()) - 1);
            faces.push_back({loops[i], loops[partner]});
            Region band;
            band.eps = near;
            band.period = region.period;
            band.add_loop(polygons[i], d);
            band.add_loop(polygons[partner], d);
            bands.push_back(std::move(band));
            ++k;
        } else
            log.push_back("face split: a loop around the surface without its partner");
    }
    for (size_t i = 0; i < loops.size(); ++i)
        if (winds[i] < 0 && areas[i] > small) {
            outer_of.push_back(static_cast<int>(i));
            faces.push_back({loops[i]});
        }
    for (size_t i = 0; i < loops.size(); ++i) {
        if (winds[i] >= 0 || areas[i] >= -small)
            continue;
        int best = -1;
        const std::vector<int> own = images(loops[i]);
        for (size_t f = 0; f < outer_of.size(); ++f) {
            // The same edges walked the other way bound the face on the other side, not a hole of it.
            if (outer_of[f] >= 0 && images(loops[outer_of[f]]) == own)
                continue;
            Region r;
            if (outer_of[f] >= 0) {
                r.eps = near;
                r.add_loop(polygons[outer_of[f]]);
            } else
                r = bands[-outer_of[f] - 1];
            int inside = 0;
            for (const Point& p : polygons[i])
                inside += r.state(p);
            // A band holds a hole only when no closed loop inside it does.
            if (inside > 0 && (best < 0 || outer_of[best] < 0 || (outer_of[f] >= 0 && areas[outer_of[f]] < areas[outer_of[best]])))
                best = static_cast<int>(f);
        }
        if (best >= 0)
            faces[best].push_back(loops[i]);
        else
            log.push_back("face split: hole outside every outer loop");
    }
    return faces;
}

}
