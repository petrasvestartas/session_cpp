#pragma once
#include <string>
#include <vector>
#include "point.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"

namespace session_cpp { namespace file_step {

// Reader — collect all matching geometry from a STEP file
std::vector<Point>               read_file_step_points(const std::string& filepath);
std::vector<NurbsCurve>          read_file_step_nurbscurves(const std::string& filepath);
std::vector<NurbsSurface>        read_file_step_nurbssurfaces(const std::string& filepath);
std::vector<NurbsSurfaceTrimmed> read_file_step_nurbssurfaces_trimmed(const std::string& filepath);
std::vector<BRep>                read_file_step_breps(const std::string& filepath);

// Writer — emit one STEP file with all given geometry
void write_file_step_nurbscurves(const std::vector<NurbsCurve>& curves, const std::string& filepath);
void write_file_step_nurbssurfaces(const std::vector<NurbsSurface>& surfaces, const std::string& filepath);
void write_file_step_nurbssurfaces_trimmed(const std::vector<NurbsSurfaceTrimmed>& trimmed, const std::string& filepath);
void write_file_step_brep(const BRep& brep, const std::string& filepath);
// Several solids in one file, each colored from its own surfacecolor (AP214 STYLED_ITEM
// chain per face). Ships boolean operands side by side before the operation is made.
void write_file_step_breps(const std::vector<const BRep*>& breps, const std::string& name,
                           const std::string& filepath);

} } // namespace session_cpp::file_step
