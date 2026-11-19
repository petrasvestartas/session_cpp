#include "brep.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numbers>

namespace session_cpp {

namespace {

using Uv = std::array<double, 2>;

/// Knots of the NURBS of an arc or segment from a0 to a1, as Curve::arc cuts it.
std::vector<double> span_knots(double a0, double a1, bool angular) {
    const int spans = angular ? std::max(1, static_cast<int>(std::ceil((a1 - a0) / (std::numbers::pi / 2) - 1e-9))) : 1;
    std::vector<double> knots;
    for (int k = 0; k <= spans; ++k)
        knots.push_back(k == spans ? a1 : a0 + (a1 - a0) * k / spans);
    return knots;
}

/// A pcurve given in a surface's true parameters, in the parameter of the surface's nurbs() and over the parameter of the edge's own NURBS: what a consumer of NURBS reads. Exact where the image is a line with a rational-linear parameter (a circle or a ruling of the surface, a line on a plane), else a cubic through values and slopes refined to `slack`; `deviation` is the distance in 3D left between the two.
NurbsCurve nurbs_pcurve(const Surface& surface, const Curve* world, const Curve2d& pcurve, double slack, double& deviation) {
    const auto [p0, p1] = pcurve.domain();
    const double a = world ? world->domain().first : p0, b = world ? world->domain().second : p1;
    const bool round = world && world->period() > 0.0;
    const std::array<bool, 2> angular = {surface.period(0) > 0.0, surface.period(1) > 0.0 || surface.kind() == Surface::Kind::Sphere};
    const std::array<std::pair<double, double>, 2> window = {surface.domain(0), surface.domain(1)};
    const auto truth = [&](double t) {
        const double tau = round ? Curve::angle(t, a, b) : t;
        const Point uv = pcurve.point_at(p0 + (tau - a) / (b - a) * (p1 - p0));
        return Uv{uv[0], uv[1]};
    };
    const auto h = [&](double t) {
        Uv uv = truth(t);
        for (int d = 0; d < 2; ++d)
            if (angular[d])
                uv[d] = Curve::rational(std::clamp(uv[d], window[d].first, window[d].second), window[d].first, window[d].second);
        return uv;
    };
    // Pieces end at the knots of the edge's NURBS and where the pcurve crosses a knot line of the surface's.
    std::vector<double> breaks = span_knots(a, b, round);
    for (int d = 0; d < 2; ++d)
        for (double knot : span_knots(window[d].first, window[d].second, angular[d]))
            for (size_t i = 0, count = breaks.size(); i + 1 < count; ++i)
                for (int k = 0; k < 16; ++k) {
                    double lo = breaks[i] + (breaks[i + 1] - breaks[i]) * k / 16, hi = breaks[i] + (breaks[i + 1] - breaks[i]) * (k + 1) / 16;
                    if (!((truth(lo)[d] - knot) * (truth(hi)[d] - knot) < 0.0))
                        continue;
                    for (int it = 0; it < 80 && std::nextafter(lo, hi) < hi; ++it)
                        ((truth(0.5 * (lo + hi))[d] - knot) * (truth(lo)[d] - knot) > 0.0 ? lo : hi) = 0.5 * (lo + hi);
                    breaks.push_back(0.5 * (lo + hi));
                }
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end(), [&](double x, double y) { return y - x <= 1e-12 * (b - a); }), breaks.end());
    const double scale = 1.0 + std::abs(window[0].first) + std::abs(window[0].second) + std::abs(window[1].first) + std::abs(window[1].second);
    // Rational-linear pieces: the weight ratio that carries the middle of the piece to its image, kept where the quarter points agree.
    std::vector<Uv> ends = {h(breaks[0])};
    std::vector<double> weights = {1.0};
    bool exact = true;
    for (size_t i = 0; i + 1 < breaks.size() && exact; ++i) {
        const Uv first = ends.back(), last = h(breaks[i + 1]), middle = h(0.5 * (breaks[i] + breaks[i + 1]));
        const int d = std::abs(last[0] - first[0]) >= std::abs(last[1] - first[1]) ? 0 : 1;
        const double ratio = last[d] == first[d] ? 1.0 : (middle[d] - first[d]) / (last[d] - middle[d]);
        exact = ratio > 0.0 && std::isfinite(ratio);
        for (double f : {0.25, 0.5, 0.75}) {
            const Uv value = h(breaks[i] + f * (breaks[i + 1] - breaks[i]));
            for (int c = 0; c < 2 && exact; ++c)
                exact = std::abs(((1.0 - f) * first[c] + f * ratio * last[c]) / ((1.0 - f) + f * ratio) - value[c]) <= 1e-13 * scale;
        }
        ends.push_back(last);
        weights.push_back(weights.back() * ratio);
    }
    const NurbsSurface& support = surface.nurbs();
    const auto gap = [&](const NurbsCurve& c) {
        double worst = 0.0;
        for (size_t i = 0; world && i + 1 < breaks.size(); ++i)
            for (double f : {0.0, 0.31, 0.5, 0.73, 1.0}) {
                const double t = breaks[i] + f * (breaks[i + 1] - breaks[i]);
                const Point uv = c.point_at(t);
                worst = std::max(worst, support.point_at(uv[0], uv[1]).distance(world->nurbs().point_at(t)));
            }
        return worst;
    };
    // On a plane or a freeform surface the parameters are the NURBS's own: a line under a line, or a conic under a conic of the same extent, is its own NURBS.
    if (!angular[0] && !angular[1] && world && pcurve.kind() == world->kind() && pcurve.kind() != Curve::Kind::Nurbs && std::abs((p1 - p0) - (b - a)) <= 1e-12 * (b - a)) {
        NurbsCurve c = pcurve.nurbs();
        c.set_domain(a, b);
        deviation = gap(c);
        return c;
    }
    if (exact) {
        const bool rational = std::any_of(weights.begin(), weights.end(), [](double w) { return std::abs(w - 1.0) > 1e-14; });
        // Pieces that are one line at one speed are one piece.
        bool straight = !rational;
        for (size_t i = 1; i + 1 < ends.size() && straight; ++i)
            for (int d = 0; d < 2; ++d)
                straight = straight && std::abs(ends[0][d] + (ends.back()[d] - ends[0][d]) * (breaks[i] - a) / (b - a) - ends[i][d]) <= 1e-14 * scale;
        if (straight) {
            ends = {ends[0], ends.back()};
            breaks = {a, b};
        }
        NurbsCurve c(3, rational, 2, static_cast<int>(ends.size()));
        for (size_t i = 0; i < ends.size(); ++i) {
            c.set_nurbsknot(static_cast<int>(i), breaks[i]);
            if (rational)
                c.set_cv_4d(static_cast<int>(i), ends[i][0] * weights[i], ends[i][1] * weights[i], 0.0, weights[i]);
            else
                c.set_cv(static_cast<int>(i), Point(ends[i][0], ends[i][1], 0.0));
        }
        deviation = gap(c);
        return c;
    }
    // Else a cubic through the values and slopes at nodes, halved where its middle strays. Slopes are one-sided: a node may sit on a knot line, where the slope turns.
    const auto slope = [&](double t, double reach) {
        const Uv f0 = h(t), f1 = h(t + 1e-3 * reach), f2 = h(t + 2e-3 * reach);
        return Uv{(-3.0 * f0[0] + 4.0 * f1[0] - f2[0]) / 2e-3, (-3.0 * f0[1] + 4.0 * f1[1] - f2[1]) / 2e-3};
    };
    std::vector<double> nodes = {breaks[0]};
    const std::function<void(double, double, int)> refine = [&](double lo, double hi, int depth) {
        const Uv f0 = h(lo), f1 = h(hi), s0 = slope(lo, hi - lo), s1 = slope(hi, lo - hi), fm = h(0.5 * (lo + hi));
        double error = 0.0;
        for (int c = 0; c < 2; ++c)
            error = std::max(error, std::abs(0.5 * (f0[c] + f1[c]) + 0.125 * (s0[c] + s1[c]) - fm[c]));
        if (error > slack * scale && depth < 10) {
            refine(lo, 0.5 * (lo + hi), depth + 1);
            refine(0.5 * (lo + hi), hi, depth + 1);
        } else
            nodes.push_back(hi);
    };
    for (size_t i = 0; i + 1 < breaks.size(); ++i)
        refine(breaks[i], breaks[i + 1], 0);
    NurbsCurve c(3, false, 4, 3 * static_cast<int>(nodes.size()) - 2);
    for (size_t i = 0; i < nodes.size(); ++i)
        for (int k = 0; k < 3; ++k)
            c.set_nurbsknot(static_cast<int>(3 * i) + k, nodes[i]);
    for (size_t i = 0; i < nodes.size(); ++i) {
        const Uv f = h(nodes[i]);
        c.set_cv(static_cast<int>(3 * i), Point(f[0], f[1], 0.0));
        if (i > 0) {
            const Uv s = slope(nodes[i], nodes[i - 1] - nodes[i]);
            c.set_cv(static_cast<int>(3 * i) - 1, Point(f[0] + s[0] / 3.0, f[1] + s[1] / 3.0, 0.0));
        }
        if (i + 1 < nodes.size()) {
            const Uv s = slope(nodes[i], nodes[i + 1] - nodes[i]);
            c.set_cv(static_cast<int>(3 * i) + 1, Point(f[0] + s[0] / 3.0, f[1] + s[1] / 3.0, 0.0));
        }
    }
    deviation = gap(c);
    return c;
}

}

Curve BRep::edge_curve(int edge_index) const {
    const BRepEdge& e = m_edges[edge_index];
    Curve c = m_curves_3d[e.curve_3d_index];
    if (e.range[0] < e.range[1])
        c.set_domain(e.range[0], e.range[1]);
    return c;
}

Curve2d BRep::pcurve(const BRepCurveOnSurface& use, bool second) const {
    const std::array<double, 2>& range = second ? use.range_2 : use.range;
    Curve2d c = m_curves_2d[second ? use.curve_2d_index_2 : use.curve_2d_index];
    if (range[0] < range[1])
        c.set_domain(range[0], range[1]);
    return c;
}

BRep BRep::unshared() const {
    BRep b = *this;
    for (size_t i = 0; i < m_edges.size(); ++i) {
        BRepEdge& e = b.m_edges[i];
        if (e.range[0] < e.range[1])
            e.curve_3d_index = b.add_curve_3d(edge_curve(static_cast<int>(i)));
        for (size_t k = 0; k < e.pcurves.size(); ++k) {
            BRepCurveOnSurface& pc = e.pcurves[k];
            if (pc.range[0] < pc.range[1])
                pc.curve_2d_index = b.add_curve_2d(pcurve(m_edges[i].pcurves[k]));
            if (pc.range_2[0] < pc.range_2[1])
                pc.curve_2d_index_2 = b.add_curve_2d(pcurve(m_edges[i].pcurves[k], true));
            pc.range = pc.range_2 = {0.0, 0.0};
        }
        e.range = {0.0, 0.0};
    }
    return b;
}

bool BRep::is_freeform() const {
    return std::all_of(m_surfaces.begin(), m_surfaces.end(), [](const Surface& s) { return s.kind() == Surface::Kind::Nurbs; }) &&
        std::all_of(m_curves_3d.begin(), m_curves_3d.end(), [](const Curve& c) { return c.kind() == Curve::Kind::Nurbs; }) &&
        std::all_of(m_curves_2d.begin(), m_curves_2d.end(), [](const Curve2d& c) { return c.kind() == Curve::Kind::Nurbs; });
}

bool BRep::is_nurbs() const {
    return !has_ranges() && is_freeform();
}

BRep BRep::as_nurbs() const {
    if (is_nurbs())
        return *this;
    BRep b = unshared();
    const BRep source = b;
    // Consumers of NURBS were written on parameters of about one unit a span: a quarter turn is one unit, a length runs from 0 to 1 (a line keeps its length).
    const double quarter = std::numbers::pi / 2;
    const auto stretch = [&](const Surface& s, int d) {
        const bool angular = s.period(d) > 0.0 || (d == 1 && s.kind() == Surface::Kind::Sphere);
        return (angular ? 1.0 / quarter : 1.0 / (s.domain(d).second - s.domain(d).first));
    };
    // Knots of evenly cut spans moved to run from 0 to `end`: whole numbers where a span is one unit, without the rounding of a rescale.
    const auto units = [](std::vector<double>& knots, double end) {
        const double first = knots.front(), last = knots.back();
        const int spans = std::max(1, static_cast<int>(std::round(std::count_if(knots.begin(), knots.end(), [&](double k) { return k != first; }) / 2.0)));
        for (double& k : knots) {
            const double j = std::round((k - first) / (last - first) * spans);
            k = j == spans ? end : end * j / spans;
        }
    };
    for (BRepEdge& e : b.m_edges) {
        const Curve* world = e.curve_3d_index >= 0 ? &source.m_curves_3d[e.curve_3d_index] : nullptr;
        for (BRepCurveOnSurface& use : e.pcurves)
            for (int* index : {&use.curve_2d_index, &use.curve_2d_index_2}) {
                if (*index < 0)
                    continue;
                const Surface& s = source.m_surfaces[use.surface_index];
                const Curve2d& pc = source.m_curves_2d[*index];
                if (s.kind() == Surface::Kind::Nurbs && pc.kind() == Curve::Kind::Nurbs && (!world || world->kind() == Curve::Kind::Nurbs))
                    continue;
                double deviation = 0.0;
                NurbsCurve c = nurbs_pcurve(s, world, pc, 1e-11, deviation);
                if (s.kind() != Surface::Kind::Nurbs)
                    c.transform(Xform::scale_xyz(stretch(s, 0), stretch(s, 1), 1.0) * Xform::translation(-s.domain(0).first, -s.domain(1).first, 0.0));
                if (world && world->period() > 0.0) {
                    c.set_domain(0.0, (world->domain().second - world->domain().first) / quarter);
                    for (double& k : c.m_nurbsknot)
                        k = std::abs(k - std::round(k)) < 1e-12 ? std::round(k) : k;
                }
                *index = b.add_curve_2d(c);
                if (deviation > 1e-9)
                    e.tolerance = std::max(e.tolerance, deviation);
            }
    }
    for (Surface& s : b.m_surfaces)
        if (s.kind() != Surface::Kind::Nurbs) {
            NurbsSurface n = s.nurbs();
            for (int d = 0; d < 2; ++d)
                units(n.m_nurbsknot[d], (s.domain(d).second - s.domain(d).first) * stretch(s, d));
            s = Surface(n);
        }
    for (Curve& c : b.m_curves_3d)
        if (c.kind() != Curve::Kind::Nurbs) {
            NurbsCurve n = c.nurbs();
            if (c.period() > 0.0)
                units(n.m_nurbsknot, (c.domain().second - c.domain().first) / quarter);
            c = Curve(n);
        }
    for (Curve2d& c : b.m_curves_2d)
        if (c.kind() != Curve::Kind::Nurbs)
            c = Curve2d(c.nurbs());
    return b;
}

}
