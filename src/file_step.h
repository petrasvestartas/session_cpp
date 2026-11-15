#pragma once
#include <map>
#include <string>
#include <vector>
#include "point.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"

namespace session_cpp {
namespace file_step {

/// Every CARTESIAN_POINT of the file in entity-id order.
std::vector<Point> read_file_step_points(const std::string& filepath);

/// Every B_SPLINE_CURVE_WITH_KNOTS of the file that reads as a valid curve.
std::vector<NurbsCurve> read_file_step_nurbscurves(const std::string& filepath);

/// Every B_SPLINE_SURFACE_WITH_KNOTS of the file that reads as a valid surface.
std::vector<NurbsSurface> read_file_step_nurbssurfaces(const std::string& filepath);

/// Every ADVANCED_FACE on a B-spline surface with its first edge loop sampled as the outer trim.
std::vector<NurbsSurfaceTrimmed> read_file_step_nurbssurfaces_trimmed(const std::string& filepath);

/// Entity ids behind a BRep read from STEP: the face entity of each face, and the EDGE_CURVE of each edge, negated for a piece of an edge split at a seam, 0 for an edge the reader made (a seam or a pole).
struct StepIds {
    std::vector<int> faces;
    std::vector<int> edges;
};

/// One BRep per connected part of every shell of every MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS and SHELL_BASED_SURFACE_MODEL, in file order;
/// throws std::runtime_error naming the first face whose geometry or topology cannot be read, so no face is dropped.
/// Linear tolerance for pcurve certification and fitting, in input-coordinate units; edge and vertex tolerances carry larger gaps of the file.
/// ids, when given, receives the entity ids of each BRep.
std::vector<BRep> read_file_step_breps(const std::string& filepath, double tolerance = 1e-7, std::vector<StepIds>* ids = nullptr);

/// Number of entity instances per type, a complex instance named by its types joined with '+'.
std::map<std::string, int> read_file_step_entity_counts(const std::string& filepath);

/// One file holding the curves as bare B_SPLINE_CURVE_WITH_KNOTS entities; false when the file cannot be written.
bool write_file_step_nurbscurves(const std::vector<NurbsCurve>& curves, const std::string& filepath);

/// One file holding the surfaces as bare B_SPLINE_SURFACE_WITH_KNOTS entities; false when the file cannot be written.
bool write_file_step_nurbssurfaces(const std::vector<NurbsSurface>& surfaces, const std::string& filepath);

/// One file holding the trimmed surfaces as ADVANCED_FACEs of an open shell; false when the file cannot be written.
bool write_file_step_nurbssurfaces_trimmed(
    const std::vector<NurbsSurfaceTrimmed>& trimmed,
    const std::string& filepath
);

/// One AP214 file holding the brep, one body per shell; false when the file cannot be written.
bool write_file_step_brep(const BRep& brep, const std::string& filepath);

/// One AP214 file holding several breps side by side, each face colored from its brep's surfacecolor; false when the file cannot be written.
bool write_file_step_breps(const std::vector<const BRep*>& breps, const std::string& name, const std::string& filepath);

} // namespace file_step
} // namespace session_cpp
