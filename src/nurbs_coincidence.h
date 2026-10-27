#pragma once

#include "nurbssurface.h"
#include <algorithm>
#include <cmath>

namespace session_cpp::nurbs_coincidence {

/// Bound same-parameter surface separation using a shared positive rational basis and control points.
inline bool same_parameters(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    for (int d = 0; d < 2; ++d)
        if (a.degree(d) != b.degree(d) || a.cv_count(d) != b.cv_count(d)
            || a.m_nurbsknot[d] != b.m_nurbsknot[d])
            return false;
    const double wa = a.weight(0, 0);
    const double wb = b.weight(0, 0);
    if (!(wa > 0.0) || !(wb > 0.0))
        return false;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const double x = a.weight(i, j) / wa;
            const double y = b.weight(i, j) / wb;
            const double gap = a.get_cv(i, j).distance(b.get_cv(i, j));
            if (!(x > 0.0) || !std::isfinite(x) || x != y
                || !std::isfinite(gap) || gap > tolerance * 0.25)
                return false;
        }
    return true;
}

/// Bound both positive-weight control hulls against one nondegenerate plane.
inline bool coplanar(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    const Point origin = a.get_cv(0, 0);
    Vector axis;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const Vector candidate = a.get_cv(i, j) - origin;
            if (candidate.magnitude() > axis.magnitude())
                axis = candidate;
        }
    Vector normal;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const Vector candidate = axis.cross(a.get_cv(i, j) - origin);
            if (candidate.magnitude() > normal.magnitude())
                normal = candidate;
        }
    if (!normal.normalize_self())
        return false;
    for (const NurbsSurface* surface : {&a, &b})
        for (int i = 0; i < surface->cv_count(0); ++i)
            for (int j = 0; j < surface->cv_count(1); ++j) {
                const double distance = std::abs((surface->get_cv(i, j) - origin).dot(normal));
                if (!(surface->weight(i, j) > 0.0)
                    || !std::isfinite(distance) || distance > tolerance * 0.125)
                    return false;
            }
    return true;
}

}
