#include "bop_ff.h"
#include "intersection.h"
#include "nurbs_coincidence.h"
#include "surface_section.h"
#include <algorithm>

namespace session_cpp::bop_ff {
namespace {

/// Whether every control point of b lies within the tolerance of a's plane, a being flat within it
bool on_plane(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    const session_cpp::Point o = a.get_cv(0, 0), u = a.get_cv(a.cv_count(0) - 1, 0), v = a.get_cv(0, a.cv_count(1) - 1);
    Vector n = Vector(u[0] - o[0], u[1] - o[1], u[2] - o[2]).cross(Vector(v[0] - o[0], v[1] - o[1], v[2] - o[2]));
    if (!n.normalize_self())
        return false;
    for (const NurbsSurface* s : {&a, &b})
        for (int i = 0; i < s->cv_count(0); ++i)
            for (int j = 0; j < s->cv_count(1); ++j) {
                const session_cpp::Point p = s->get_cv(i, j);
                if (std::abs(n.dot(Vector(p[0] - o[0], p[1] - o[1], p[2] - o[2]))) > tolerance)
                    return false;
            }
    return true;
}

}

Result intersect(const BRep& a, int face_a, const BRep& b, int face_b, double fuzzy) {
    const NurbsSurface& sa = a.m_surfaces[a.m_faces[face_a].surface_index];
    const NurbsSurface& sb = b.m_surfaces[b.m_faces[face_b].surface_index];
    const double tolerance = std::max({a.m_faces[face_a].tolerance, b.m_faces[face_b].tolerance, 1e-7}) + fuzzy;
    Result result;
    nurbs_coincidence::ParameterMap mapping;
    if (nurbs_coincidence::same_parameters(sa, sb, tolerance) || nurbs_coincidence::parameter_mapping(sa, sb, tolerance, mapping)
        || nurbs_coincidence::common_basis_agreement(sa, sb, tolerance) || nurbs_coincidence::coplanar(sa, sb, tolerance)
        || (on_plane(sa, sb, tolerance) && on_plane(sb, sa, tolerance))) {
        result.same_domain = true;
        return result;
    }
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections;
    try {
        surface_section::Result events = surface_section::intersect(sa, sb, tolerance);
        sections = std::move(events.curves);
        for (const auto& p : events.points)
            result.points.push_back({p.point, p.first, p.second, tolerance});
    } catch (const std::exception&) {
    }
    if (sections.empty())
        try {
            sections = Intersection::surface_surface(sa, sb, tolerance);
        } catch (const std::exception&) {
        }
    for (auto& [world, ua, ub] : sections) {
        Curve c{world, ua, ub, tolerance, world.point_at_start().distance(world.point_at_end()) <= tolerance};
        for (int i = 0; i <= 16; ++i) {
            const double s = i / 16.0;
            const session_cpp::Point p = world.point_at(world.domain_start() + s * (world.domain_end() - world.domain_start()));
            const session_cpp::Point pa = ua.point_at(ua.domain_start() + s * (ua.domain_end() - ua.domain_start()));
            const session_cpp::Point pb = ub.point_at(ub.domain_start() + s * (ub.domain_end() - ub.domain_start()));
            c.tolerance = std::max({c.tolerance, p.distance(sa.point_at(pa[0], pa[1])), p.distance(sb.point_at(pb[0], pb[1]))});
        }
        result.curves.push_back(std::move(c));
    }
    return result;
}

}
