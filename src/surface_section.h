#pragma once

#include "nurbssurface.h"
#include "plane.h"
#include "surface_point_preimages.h"
#include <string>
#include <optional>
#include <utility>
#include <tuple>
#include <vector>

namespace session_cpp::surface_section {

/// Return exact paired extrusion/plane sections; nullopt denotes unsupported geometry or numerical conditioning.
std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> extrusion_plane(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance = 1e-7
);

/// Complete two-branch zero set for exact bilinear graphs with a common affine
/// projected footprint; nullopt retains the generic SSI route and its proof gate.
std::optional<std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>>
bilinear_crossing(const NurbsSurface& first, const NurbsSurface& second, double tolerance);


/// Full-domain qualification is independent of successful legacy locus fitting.
enum class Coverage { Unverified, OriginalDomainExcluded, GlobalContactPoint,
                      LegacyLocusOnly, CoincidentUnverified, Unresolved };
inline bool qualified(Coverage coverage) {
    return coverage == Coverage::OriginalDomainExcluded || coverage == Coverage::GlobalContactPoint;
}
inline const char* coverage_name(Coverage coverage) {
    switch (coverage) {
    case Coverage::OriginalDomainExcluded: return "original_domain_excluded";
    case Coverage::GlobalContactPoint: return "global_contact_point";
    case Coverage::LegacyLocusOnly: return "legacy_locus_only";
    case Coverage::CoincidentUnverified: return "coincident_unverified";
    case Coverage::Unresolved: return "unresolved";
    default: return "unverified";
    }
}
struct PointEvent {
    Point point, first, second; // UVs are incidence witnesses, not exhaustive preimages.
    surface_point_preimages::ProjectivePoint exact_witness{};
    surface_point_preimages::Result first_preimages, second_preimages;
};
struct Result {
    std::vector<std::tuple<NurbsCurve,NurbsCurve,NurbsCurve>> curves;
    std::vector<PointEvent> points;
    bool coincident = false;
    bool complete = false; // Execution status; inspect coverage separately.
    Coverage coverage = Coverage::Unverified;
    std::string failure;
    std::array<double,8> unresolved_domains{}; // Original scopes, not an invented local root box.
    bool world_coverage_complete() const { return complete && qualified(coverage); }
};
/// Shared support inventory: legacy positive curves remain explicitly unqualified.
Result intersect(const NurbsSurface& first, const NurbsSurface& second, double tolerance = 1e-7);

} // namespace session_cpp::surface_section
