#pragma once
#include "brep_classify.h"
#include "nurbs_bounds.h"
#include "nurbs_coincidence.h"
#include "nurbs_normal_sweep.h"

namespace session_cpp::normal_sweep {
struct FaceResult {
    Result proof;
    bool original_trim_domain_proved = false;
    std::array<double, 4> rectangle{0, 0, 0, 0};
};
inline bool original_trim_bounds(const BRep& source, int face, std::array<double, 4>& rectangle) {
    const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
    bool first = true;
    for (const BRepRef& wire : source.m_faces[face].wires)
        for (const BRepRef& use : source.wire_edges(wire)) {
            const int index = source.pcurve_index(use.index, face, use.orientation);
            if (index < 0)
                return false;
            const NurbsCurve& curve = source.m_curves_2d[index];
            for (int i = 0; i < curve.cv_count(); ++i) {
                double x, y, z, w;
                if (!curve.get_cv_4d(i, x, y, z, w) || !(w > 0) || !std::isfinite(w))
                    return false;
                const double raw[2] = {x, y};
                for (int d = 0; d < 2; ++d) {
                    const std::pair<double, double> domain = surface.domain(d);
                    Interval value = Interval(raw[d]) / Interval(w);
                    if (nurbs_coincidence::exact_products_equal(raw[d], 1, domain.first, w))
                        value = Interval(domain.first);
                    else if (nurbs_coincidence::exact_products_equal(raw[d], 1, domain.second, w))
                        value = Interval(domain.second);
                    if (!nurbs_interval::finite(value) || value.lo < domain.first || value.hi > domain.second)
                        return false;
                    rectangle[2 * d] = first ? value.lo : std::min(rectangle[2 * d], value.lo);
                    rectangle[2 * d + 1] = first ? value.hi : std::max(rectangle[2 * d + 1], value.hi);
                }
                first = false;
            }
        }
    return !first && rectangle[0] < rectangle[1] && rectangle[2] < rectangle[3];
}
inline FaceResult original_face(const BRep& source, int face, double signed_distance,
                               double tolerance, size_t max_cells) {
    FaceResult result;
    if (!source.is_valid() || face < 0 || face >= source.face_count()
        || !std::isfinite(tolerance) || !(tolerance > 0)
        || !original_trim_bounds(source, face, result.rectangle))
        return result;
    const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
    const double metric = nurbs_bounds::surface_metric(surface);
    if (!(metric > 0) || !std::isfinite(metric))
        return result;
    const double trim_tolerance = tolerance / metric;
    if (!(trim_tolerance > 0) || !std::isfinite(trim_tolerance))
        return result;
    result.original_trim_domain_proved = true;
    result.proof = check(surface, result.rectangle, signed_distance, max_cells,
        [&](double u, double v) {
            return brep_classify::face_uv(source, face, {u, v, 0}, trim_tolerance)
                == brep_classify::State::Inside;
        });
    return result;
}
}
