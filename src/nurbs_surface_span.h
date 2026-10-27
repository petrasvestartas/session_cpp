#pragma once

#include "nurbssurface.h"
#include "nurbsknot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace session_cpp::nurbs_detail {

/// Evaluate an immutable validated surface through its original active control window.
class SurfaceEvaluator {
    const NurbsSurface& surface;
    NurbsSurface local;
    std::array<int, 2> active{-1, -1};

    void activate(double u, double v) {
        if (!std::isfinite(u) || !std::isfinite(v))
            throw std::invalid_argument("Local surface evaluation requires finite parameters");
        const std::array<int, 2> span = {
            nurbsknot::find_span(surface.order(0), surface.cv_count(0), surface.m_nurbsknot[0], u),
            nurbsknot::find_span(surface.order(1), surface.cv_count(1), surface.m_nurbsknot[1], v)
        };
        if (span == active)
            return;
        for (int dir = 0; dir < 2; ++dir)
            std::copy_n(surface.m_nurbsknot[dir].begin() + span[dir], local.m_nurbsknot[dir].size(), local.m_nurbsknot[dir].begin());
        for (int i = 0; i < local.cv_count(0); ++i)
            for (int j = 0; j < local.cv_count(1); ++j)
                std::copy_n(surface.cv(span[0] + i, span[1] + j), surface.cv_size(), local.cv(i, j));
        active = span;
    }

public:
    explicit SurfaceEvaluator(const NurbsSurface& source)
        : surface(source) {
        if (!source.is_valid() || source.dimension() != 3)
            throw std::invalid_argument("Local surface evaluation requires a valid three-dimensional support");
        local = NurbsSurface(3, source.is_rational(), source.order(0), source.order(1), source.order(0), source.order(1));
    }

    Point point_at(double u, double v) {
        activate(u, v);
        return local.point_at(u, v);
    }

    std::vector<Vector> evaluate(double u, double v, int derivatives) {
        if (derivatives < 0)
            return {};
        activate(u, v);
        return local.evaluate(u, v, derivatives);
    }
};

}
