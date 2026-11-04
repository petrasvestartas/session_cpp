#pragma once
#include "brep_face_boundary.h"
#include "nurbs_interval.h"
#include <numeric>
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>

namespace session_cpp::brep_section::detail {
// Only active topological uses establish material connectivity, never pool records.
inline std::vector<int> active_components(const BRep& owner,const std::vector<int>& faces) {
    std::vector<int> parent(owner.vertex_count());
    std::iota(parent.begin(),parent.end(),0);
    const auto root=[&](int vertex) {
        while (parent[vertex]!=vertex)
            vertex=parent[vertex];
        return vertex;
    };
    for (int face:faces) {
        const auto boundary=brep_face_boundary::extract(owner,face);
        if (!boundary.complete)
            throw std::runtime_error(boundary.failure);
        for (const auto& wire:boundary.wires) {
            if (!brep_face_boundary::detail::active(wire.reference.orientation))
                continue;
            for (const auto& use:wire.uses) {
                if (!brep_face_boundary::detail::active(use.edge.orientation))
                    continue;
                const auto& edge=owner.m_edges[use.edge.index];
                const int a=root(edge.start_vertex),b=root(edge.end_vertex);
                if (a!=b)
                    parent[b]=a;
            }
        }
    }
    for (int& vertex:parent)
        vertex=root(vertex);
    return parent;
}

// Positive homogeneous weights put the entire original curve in its Cartesian
// control hull. This is a global negative certificate, not a sampled distance.
inline bool curve_excludes_point(const NurbsCurve& curve,const Point& point,double tolerance) {
    using I=nurbs_interval::Interval;
    if (!curve.is_valid() || curve.dimension()!=3 || !std::isfinite(tolerance) || !(tolerance>0))
        return false;
    for (int d=0;d<3;++d)
        if (!std::isfinite(point[d]))
            return false;
    std::array<I,3> hull;
    for (int i=0;i<curve.cv_count();++i) {
        const double weight=curve.weight(i);
        if (!std::isfinite(weight) || !(weight>0))
            return false;
        for (int d=0;d<3;++d) {
            if (!std::isfinite(curve.cv(i)[d]))
                return false;
            const I coordinate=(I(curve.cv(i)[d])-I(weight)*I(point[d]))/I(weight);
            if (!nurbs_interval::finite(coordinate))
                return false;
            hull[d]=i==0?coordinate:I(std::min(hull[d].lo,coordinate.lo),std::max(hull[d].hi,coordinate.hi));
        }
    }
    I lower;
    for (const I coordinate:hull) {
        const double nearest=coordinate.lo>0?coordinate.lo:coordinate.hi<0?-coordinate.hi:0;
        lower=lower+I(nearest)*I(nearest);
    }
    const I allowed=I(tolerance)*I(tolerance);
    return nurbs_interval::finite(lower) && nurbs_interval::finite(allowed) && lower.lo>allowed.hi;
}

// An existing CC residual may already consume Tol. Certify the actual target
// singleton against the actual public point independently, without adding gaps.
inline bool owns_point(const NurbsCurve& target,double parameter,const Point& point,double tolerance) {
    return std::isfinite(parameter) && parameter>=target.domain_start() && parameter<=target.domain_end() &&
        nurbs_interval::curve_within_ball(target,{parameter,parameter},
            {point[0],point[1],point[2]},tolerance);
}
} // namespace session_cpp::brep_section::detail
