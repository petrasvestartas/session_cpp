#pragma once

// V2 PIPELINE TRACER — dumps the v2 boolean pipeline's internal state in EXACTLY the record
// format of validation/occt_trace/occt_trace.cpp, so an OCCT trace and a v2 trace of the same
// case can be diffed record by record and the FIRST DIVERGING STAGE named.
//
// Format contract (copied from occt_trace.cpp, not re-invented):
//   * one record per line, "TAG key=value key=value ..."
//   * every float via %.9g with |x| < 1e-12 snapped to 0
//   * every list-valued field sorted; every set of like records emitted sorted
//   * byte-stable across runs of the same binary on the same input
//
// WHAT IS AND IS NOT AN IN-SITU HOOK  (say it here so no reader has to infer it)
// -----------------------------------------------------------------------------
// v2sol::v2_boolean() runs its front end inside a file-static (v2sol_run_front) and returns only
// a BRep plus a census, so there is no place to hang a per-stage callback without editing
// brep_v2_boolean.cpp — a file this module does not own. Instead:
//
//   PHASE 1  ARENA REPLICA. BdsArena::init on the same two operands with the SAME tolerance
//            formula the driver uses, the same lazy-pave-block materialisation, and v2sec's
//            V2Section with the same V2SectionParams (trail_samples = 768 unless
//            SESSION_V2_TRAIL_N is set, exactly as v2sol_run_front does). This is a replica, not
//            a hook: the stages are deterministic functions of the operands, so it reproduces
//            the driver's arena, but it is re-run rather than observed. Records: DS / RANGE / SI
//            / DSVERT / SD / PAVE / PB / CB / FI / FIPB / IFF / SEC / SEC2D / SECPB.
//
//   PHASE 2  INTERFERENCE PROBE. v2int::V2Interf (VV/VE/EE/VF/EF) is a green component
//            (main_15 29/29) that the production driver DOES NOT CALL — brep_v2_boolean.cpp says
//            so in its own comment at stage 0/1/2. Running it into the phase-1 arena would
//            change what v2sec then sees, so it is run into a SEPARATE arena and its records
//            carry `src=probe`. Any consumer that forgets this is comparing OCCT's production
//            interferences against our non-production ones, which is why the key is on the line.
//
//   PHASE 3  RESULT. v2sol::v2_boolean() itself, dumped through the SHARED verdict harness
//            (src/v2/v2_verdict.h) and brep_massprops — never through metric code written here.
//            Records: RES / RESSOLID / RESSHELL / RESFACE / RESFEDGE / RESEDGE / RESVERT.
//
// A CAP record at the top of every trace lists exactly which stages this kernel populates, so
// the comparison tool can distinguish "diverged" from "not implemented".
//
// NEW MODULE. It reads BReps and builds its own arena; it mutates nothing the kernel owns.

#include "brep.h"
#include "brep_bds.h"
#include "brep_v2_interf.h"
#include "brep_v2_section.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"

#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace session_cpp {
namespace v2dump {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Formatting — byte-identical to occt_trace's R()/P3()/IntListStr()
///////////////////////////////////////////////////////////////////////////////////////////

/// %.9g, |x| < 1e-12 snapped to 0, "nan" for NaN. occt_trace.cpp:86-95.
std::string v2dump_r(double v);
std::string v2dump_p3(const Point& p);
std::string v2dump_ints(const std::vector<int>& v);   ///< "-" when empty

/// Analytic recognition of a 3D curve from samples: Line / Circle / BSpline / Degenerated.
/// OCCT reports GeomAbs types; ours are NURBS throughout, so the type has to be MEASURED.
/// A curve whose sampled points all lie within 1e-9*scale of the chord is a Line; one whose
/// samples are coplanar and equidistant from the circumcentre of three of them is a Circle;
/// a curve of zero length is Degenerated.
/// LIMIT, stated so no reader over-reads the field: Ellipse / Hyperbola / Parabola are NOT
/// recognised and come back "BSpline". The comparison tool therefore treats the type field as
/// INFO, never as a divergence.
std::string v2dump_curve_kind(const NurbsCurve& c);

/// "Plane" when NurbsSurface::is_planar, else "Curved". We deliberately do NOT guess
/// Sphere/Cylinder/Cone/Torus: the comparison tool folds OCCT's types the same way, so the
/// comparison stays honest instead of matching on a recogniser that could be wrong.
std::string v2dump_surface_kind(const NurbsSurface& s);

///////////////////////////////////////////////////////////////////////////////////////////
// 1. Operand specs — the SAME grammar occt_trace uses, so one script drives both
///////////////////////////////////////////////////////////////////////////////////////////

/// <type>[,<key>=<val>]*  with type in {sphere, cylinder, box, cone, torus} and the transform
/// keys center / rotx / roty / rotz / tx / ty / tz applied in that order about the origin.
struct V2DumpSpec {
    std::string raw, type;
    std::map<std::string, double> num;
    bool center = false;

    bool has(const char* k) const { return num.count(k) != 0; }
    double get(const char* k, double d) const;
};

V2DumpSpec v2dump_parse_spec(const std::string& s);

/// Build the BRep for a spec. `why` receives the reason when the spec cannot be built
/// (unknown type, or a truncated cone: BRep::create_cone only makes an apex cone).
BRep v2dump_build_shape(const V2DumpSpec& sp, std::string* why = nullptr);

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Per-section dumpers (public so a driver can dump a stage in isolation)
///////////////////////////////////////////////////////////////////////////////////////////

/// ARG / AFACE / AFEDGE / AEDGE / AVERT for one operand. Indices are 1-BASED, as occt_trace's
/// TopExp maps are, so counts and ids line up on inspection.
void v2dump_operand(std::ostream& os, int argi, const V2DumpSpec& sp, const BRep& b);

/// DS / RANGE / SI / DSVERT / SD / PAVE / PB / CB / FI / FIPB for one arena state.
void v2dump_arena(std::ostream& os, const BdsArena& ds, const char* tag);

/// IVV / IVE / IVF / IEE / IEF from a probe arena + its V2Interf records. Every line carries
/// `src=probe`: these stages are NOT part of the production v2 boolean.
void v2dump_interf(std::ostream& os, const BdsArena& ds, const v2int::V2Interf& in,
                   const char* tag);

/// IFF / SEC / SEC2D / SECPB from the section stage.
void v2dump_section(std::ostream& os, const BdsArena& ds, const v2sec::V2Section& sec,
                    const char* tag);

/// RES / RESSOLID / RESSHELL / RESFACE / RESFEDGE / RESEDGE / RESVERT for a result BRep.
/// Counts come from v2v::v2_verdict and brep_massprops only.
struct V2DumpResultCounts {
    int nsolid = 0, nshell = 0, nface = 0, nedge = 0, nvert = 0, nnaked = 0, ndegen = 0;
    double vol = 0, area = 0, closure = 1.0;
    int valid = 0;
};
V2DumpResultCounts v2dump_result(std::ostream& os, const BRep& res);

///////////////////////////////////////////////////////////////////////////////////////////
// 3. The whole case
///////////////////////////////////////////////////////////////////////////////////////////

struct V2DumpOptions {
    std::string name = "case";
    std::string op = "cut";       ///< cut | common | fuse
    bool run_interf = true;       ///< phase 2 probe
    bool run_result = true;       ///< phase 3
    int trail_samples = 0;        ///< 0 -> the driver's own default (768 / SESSION_V2_TRAIL_N)
};

/// Writes a complete trace. Returns 0 on success, 2 when an operand could not be built.
int v2dump_case(std::ostream& os, const V2DumpSpec& spa, const V2DumpSpec& spb,
                const V2DumpOptions& opt);

}  // namespace v2dump
}  // namespace session_cpp
