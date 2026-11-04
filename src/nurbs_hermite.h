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
// Bernstein polynomial. Interior knots retain independent one-sided jets.
inline NurbsCurve quintic(const std::vector<QuinticSpan>& spans) {
    if (spans.empty() || spans.size() > 4096)
        return {};
    NurbsCurve result(3, false, 6, static_cast<int>(5 * spans.size() + 1));
    result.m_nurbsknot.clear();
    result.m_nurbsknot.insert(result.m_nurbsknot.end(), 5, spans.front().first);
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
        const std::array<Point, 6> controls{
            s.start,
            s.start + s.start_derivative * (h / 5),
            s.start + s.start_derivative * (2 * h / 5) + s.start_second * (h * h / 20),
            s.end - s.end_derivative * (2 * h / 5) + s.end_second * (h * h / 20),
            s.end - s.end_derivative * (h / 5),
            s.end};
        for (size_t j = 0; j < controls.size(); ++j) {
            for (int d = 0; d < 3; ++d)
                if (!std::isfinite(controls[j][d]))
                    return {};
            result.set_cv(static_cast<int>(5 * i + j), controls[j]);
        }
        result.m_nurbsknot.insert(result.m_nurbsknot.end(), 5, s.last);
    }
    return result.is_valid() ? result : NurbsCurve{};
}
} // namespace session_cpp::nurbs_hermite
