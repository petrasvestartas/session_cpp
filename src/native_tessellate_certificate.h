#pragma once
#include "nurbs_interval.h"

namespace session_cpp::native_tessellate {
using nurbs_interval::Interval;
using Box = std::array<Interval, 3>;
using UV = std::array<double, 2>;

inline Box subtract(const Box& a, const Box& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline Box cross(const Box& a, const Box& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]};
}
inline Interval dot(const Box& a, const Box& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Box exact(const Point& point) {
    return {Interval(point[0]), Interval(point[1]), Interval(point[2])};
}
inline Interval absolute(Interval value) {
    if (!nurbs_interval::finite(value))
        return value;
    return {0, nurbs_interval::magnitude(value)};
}

inline bool triangle(const NurbsSurface& surface, const std::array<UV, 3>& uv,
    const std::array<Point, 3>& vertices, double deflection, double angle) {
    if (!(std::isfinite(deflection) && deflection > 0 && std::isfinite(angle) && angle > 0))
        return false;
    std::array<double, 4> rectangle = {uv[0][0], uv[0][0], uv[0][1], uv[0][1]};
    for (const UV& point : uv)
        for (int d = 0; d < 2; ++d) {
            if (!std::isfinite(point[d]))
                return false;
            rectangle[2 * d] = std::min(rectangle[2 * d], point[d]);
            rectangle[2 * d + 1] = std::max(rectangle[2 * d + 1], point[d]);
        }
    const Interval du1 = Interval(uv[1][0]) - Interval(uv[0][0]);
    const Interval dv1 = Interval(uv[1][1]) - Interval(uv[0][1]);
    const Interval du2 = Interval(uv[2][0]) - Interval(uv[0][0]);
    const Interval dv2 = Interval(uv[2][1]) - Interval(uv[0][1]);
    const Interval determinant = du1 * dv2 - du2 * dv1;
    if (!nurbs_interval::finite(determinant) || (determinant.lo <= 0 && determinant.hi >= 0))
        return false;
    nurbs_interval::SurfaceBounds bounds, anchor;
    if (!nurbs_interval::surface_bounds(surface, rectangle, {}, bounds)
        || !nurbs_interval::surface_bounds(surface,
            {uv[0][0], uv[0][0], uv[0][1], uv[0][1]}, {}, anchor, false))
        return false;
    const Box first = subtract(exact(vertices[1]), exact(vertices[0]));
    const Box second = subtract(exact(vertices[2]), exact(vertices[0]));
    const Interval du = Interval(rectangle[0], rectangle[1]) - Interval(uv[0][0]);
    const Interval dv = Interval(rectangle[2], rectangle[3]) - Interval(uv[0][1]);
    Interval distance;
    for (int d = 0; d < 3; ++d) {
        const Interval linear_u = (first[d] * dv2 - second[d] * dv1) / determinant;
        const Interval linear_v = (second[d] * du1 - first[d] * du2) / determinant;
        const Interval residual = anchor.point[d] - Interval(vertices[0][d])
            + (bounds.derivative[0][d] - linear_u) * du
            + (bounds.derivative[1][d] - linear_v) * dv;
        distance = distance + absolute(residual);
    }
    if (!nurbs_interval::finite(distance) || distance.hi > deflection)
        return false;
    Box normal = cross(bounds.derivative[0], bounds.derivative[1]);
    if (determinant.hi < 0)
        for (Interval& coordinate : normal)
            coordinate = Interval(-1) * coordinate;
    const Box plane = cross(first, second);
    const Interval alignment = dot(normal, plane);
    const Box deviation = cross(normal, plane);
    Interval turn;
    for (Interval coordinate : deviation)
        turn = turn + absolute(coordinate);
    const Interval allowed = Interval(std::min(angle, 1.0)) * alignment;
    return nurbs_interval::finite(alignment) && alignment.lo > 0
        && nurbs_interval::finite(turn) && nurbs_interval::finite(allowed)
        && turn.hi <= allowed.lo;
}
}
