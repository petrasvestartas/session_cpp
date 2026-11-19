#include "bop_ff.h"
#include "bop_ds.h"
#include "intersection.h"
#include "nurbs_coincidence.h"
#include "nurbs_implicit.h"
#include "surface_section.h"
#include <algorithm>
#include <tuple>

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

/// Whether two surfaces are recognized as one sphere, cylinder, cone or torus within the tolerance
bool same_form(const NurbsSurface& sa, const NurbsSurface& sb, double tolerance) {
    if (sa.degree(0) + sa.degree(1) < 3 || sb.degree(0) + sb.degree(1) < 3)
        return false;
    nurbs_implicit::Implicit a, b;
    try {
        a = nurbs_implicit::recognize(sa, tolerance);
        b = nurbs_implicit::recognize(sb, tolerance);
    } catch (const std::exception&) {
        return false;
    }
    if (a.kind < 2 || a.kind != b.kind)
        return false;
    const Vector u(a.axis[0], a.axis[1], a.axis[2]), v(b.axis[0], b.axis[1], b.axis[2]);
    const Vector d(b.point[0] - a.point[0], b.point[1] - a.point[1], b.point[2] - a.point[2]);
    const double size = std::max({1.0, a.radius, b.radius});
    const bool parallel = a.kind == 2 || u.cross(v).magnitude() * size <= tolerance;
    const bool on_axis = a.kind == 3 ? u.cross(d).magnitude() <= tolerance : d.magnitude() <= tolerance;
    const bool cone_sense = a.kind != 4 || u.dot(v) > 0;
    return parallel && on_axis && cone_sense && std::abs(a.radius - b.radius) * (a.kind == 4 ? size : 1.0) <= tolerance
        && std::abs(a.minor - b.minor) <= tolerance;
}

/// The section of two flat bilinear patches: their planes' line clipped to both parallelograms
bool plane_plane(const NurbsSurface& sa, const NurbsSurface& sb, double tolerance, Result& result) {
    session_cpp::Point oa, ob;
    Vector a1, a2, b1, b2;
    if (!bop::affine_patch(sa, oa, a1, a2) || !bop::affine_patch(sb, ob, b1, b2))
        return false;
    Vector na = a1.cross(a2), nb = b1.cross(b2);
    na.normalize_self();
    nb.normalize_self();
    Vector d = na.cross(nb);
    if (d.magnitude() < 1e-12)
        return true;
    d.normalize_self();
    // The point of the line nearest oa: on both planes, solved in the span of the normals.
    const double ca = na.dot(Vector(oa[0], oa[1], oa[2])), cb = nb.dot(Vector(ob[0], ob[1], ob[2])), k = na.dot(nb);
    const double x = (ca - k * cb) / (1 - k * k), y = (cb - k * ca) / (1 - k * k);
    const Vector base = na * x + nb * y;
    double lo = -1e300, hi = 1e300;
    for (const auto& [o, e1, e2] : {std::tuple{oa, a1, a2}, std::tuple{ob, b1, b2}}) {
        const double ee = e1.dot(e1), ef = e1.dot(e2), ff = e2.dot(e2), det = ee * ff - ef * ef;
        const Vector r0(base[0] - o[0], base[1] - o[1], base[2] - o[2]);
        const double s0 = (ff * r0.dot(e1) - ef * r0.dot(e2)) / det, s1 = (ff * d.dot(e1) - ef * d.dot(e2)) / det;
        const double q0 = (ee * r0.dot(e2) - ef * r0.dot(e1)) / det, q1 = (ee * d.dot(e2) - ef * d.dot(e1)) / det;
        for (const auto& [c0, c1] : {std::pair{s0, s1}, std::pair{q0, q1}}) {
            if (std::abs(c1) < 1e-15) {
                if (c0 < 0 || c0 > 1)
                    return true;
                continue;
            }
            const double t0 = -c0 / c1, t1 = (1 - c0) / c1;
            lo = std::max(lo, std::min(t0, t1));
            hi = std::min(hi, std::max(t0, t1));
        }
    }
    if (hi - lo <= tolerance)
        return true;
    const session_cpp::Point p0(base[0] + d[0] * lo, base[1] + d[1] * lo, base[2] + d[2] * lo), p1(base[0] + d[0] * hi, base[1] + d[1] * hi, base[2] + d[2] * hi);
    const NurbsCurve line = NurbsCurve::create(false, 1, {p0, p1});
    result.curves.push_back({line, bop::project(line, line.domain_start(), line.domain_end(), sa),
        bop::project(line, line.domain_start(), line.domain_end(), sb), tolerance, false});
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
        || (on_plane(sa, sb, tolerance) && on_plane(sb, sa, tolerance)) || same_form(sa, sb, tolerance)) {
        result.same_domain = true;
        return result;
    }
    if (plane_plane(sa, sb, tolerance, result))
        return result;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections;
    bool complete = false;
    try {
        surface_section::Result events = surface_section::intersect(sa, sb, tolerance);
        sections = std::move(events.curves);
        complete = events.complete;
        for (const auto& p : events.points)
            result.points.push_back({p.point, p.first, p.second, tolerance});
    } catch (const std::exception&) {
    }
    if (sections.empty() && !complete)
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
