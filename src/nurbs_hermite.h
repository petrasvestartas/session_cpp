#pragma once
#include "nurbscurve.h"
#include <array>
#include <cmath>
#include <vector>

namespace session_cpp::nurbs_hermite {
struct QuinticSpan {
    double first, last;
    Point start, end;
    Vector start_derivative, end_derivative, start_second, end_second;
};

// Endpoint value, first derivative and second derivative determine each quintic
// Bernstein polynomial. A join whose one-sided jets are equal is C2 and takes three knots;
// any other takes five and keeps its independent jets.
inline NurbsCurve quintic(const std::vector<QuinticSpan>& spans) {
    if (spans.empty() || spans.size() > 4096)
        return {};
    const auto same = [](const Vector& a, const Vector& b) { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]; };
    std::vector<double> knots(5, spans.front().first);
    for (size_t i = 0; i < spans.size(); ++i) {
        const auto& s = spans[i];
        const double h = s.last - s.first;
        if (!(h > 0) || !std::isfinite(s.first) || !std::isfinite(s.last))
            return {};
        if (i) {
            if (s.first != spans[i - 1].last)
                return {};
            for (int d = 0; d < 3; ++d)
                if (s.start[d] != spans[i - 1].end[d])
                    return {};
        }
        const bool smooth = i + 1 < spans.size() && same(s.end_derivative, spans[i + 1].start_derivative) &&
            same(s.end_second, spans[i + 1].start_second);
        knots.insert(knots.end(), smooth ? 3 : 5, s.last);
    }
    NurbsCurve result(3, false, 6, static_cast<int>(knots.size()) - 4);
    result.m_nurbsknot = knots;
    size_t right = 0; // the span that starts at the knot in the middle of the five
    for (int i = 0; i < result.cv_count(); ++i) {
        // A control point is the blossom at its five knots; three of them are one break, whose jet gives it.
        const double t = knots[i + 2];
        while (right < spans.size() && spans[right].first < t)
            ++right;
        double sum = 0, squares = 0;
        for (int k = 0; k < 5; ++k) {
            sum += knots[i + k] - t;
            squares += (knots[i + k] - t) * (knots[i + k] - t);
        }
        const bool after = right < spans.size() && (sum > 0 || right == 0);
        const auto& s = spans[after ? right : right - 1];
        const Point control = (after ? s.start : s.end) + (after ? s.start_derivative : s.end_derivative) * (sum / 5) +
            (after ? s.start_second : s.end_second) * ((sum * sum - squares) / 40);
        for (int d = 0; d < 3; ++d)
            if (!std::isfinite(control[d]))
                return {};
        result.set_cv(i, control);
    }
    return result.is_valid() ? result : NurbsCurve{};
}
} // namespace session_cpp::nurbs_hermite
