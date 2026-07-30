#pragma once

// Mass properties (volume, surface area, centroid) for trimmed-NURBS B-Reps.
//
// DESIGN CONTRACT
// ---------------
// 1. ALWAYS TERMINATES. Every integral is a *finite* list of intervals refined by a
//    bounded loop: each refinement step consumes at least one surface evaluation from a
//    hard budget (MassPropsOptions::max_surface_evals) and each interval has a bounded
//    subdivision depth (max_depth). When the budget or the depth cap is reached the
//    routine STOPS and returns the value it has together with an error estimate and
//    `converged=false`. There is no `while (err > tol)` anywhere; non-termination is
//    impossible by construction.
//
// 2. CORRECT ON TRIMMED NURBS. The integrand is evaluated only over the TRIMMED region.
//    The 2D area integral over the trimmed UV region D is reduced to a 1D integral over
//    the trim boundary by Green's theorem (the same reduction OCCT uses in
//    BRepGProp_VinertGK / BRepGProp_TFunction / BRepGProp_UFunction):
//
//        II_D f(u,v) du dv  =  oint_{dD, CCW} H(u,v) dv,   H(u,v) = I_{u_ref}^{u} f(s,v) ds
//
//    The value is independent of u_ref because the two choices differ by a closed-loop
//    integral of an exact 1-form. No point-in-polygon mask, no staircase error: the trim
//    curves ARE the domain of integration, so a trimmed region is handled exactly to
//    quadrature accuracy, and degenerate (pole) trims contribute correctly because they
//    are ordinary curves in UV.
//
// 3. STATED ACCURACY. Every 1D integral uses Gauss-Kronrod 7/15: the 15-point Kronrod
//    value is the estimate, |K15 - G7| (QUADPACK scaling) is the error estimate.
//    Refinement is driven by that estimate under the bound from (1). The returned
//    MassProps carries volume_error / area_error.
//
// 4. EXACT WHERE EXACT IS POSSIBLE. Planar faces take a closed-form path: for a plane the
//    whole face integral collapses to a 1D integral over the 3D boundary curve with a
//    polynomial integrand, which Gauss-Kronrod integrates exactly. Quadric/free-form
//    faces take the Green path above. FaceMassProps::path records which was used.
//
// 5. ORIENTATION-AWARE. Per-face traversal sense is recovered from the SIGN of the
//    Green-reduced AREA integral (the true area is positive by definition, so the sign of
//    the raw value is the traversal sense) -- no reliance on stored `reversed` flags.
//    Relative face orientation is propagated across every 2-trim edge, each connected
//    shell is normalised to positive enclosed volume, and shells nested inside another
//    shell (cavities) contribute negatively. Disjoint shells simply add.

#include "brep.h"
#include "point.h"
#include <vector>

namespace session_cpp {

struct MassPropsOptions {
    /// Target RELATIVE accuracy of the reported volume / area. Chasing a tolerance below the
    /// nested-quadrature noise floor (the inner integral's roundoff, ~1e-13 relative) would
    /// make the outer error estimate stall and burn the whole budget for no accuracy gain,
    /// so the outer criterion is clamped at 1e-11 and the inner at 1e-13 internally.
    double rel_tolerance = 1e-10;
    /// HARD upper bound on total surface evaluations for the whole BRep. Reaching it
    /// stops refinement immediately and sets MassProps::converged = false.
    long long max_surface_evals = 60000000;
    /// Per-face share of the global budget is  max(min_face_evals, max_surface_evals/faces).
    long long min_face_evals = 250000;
    /// Maximum bisection depth of any single quadrature interval.
    int max_depth = 20;
    /// Maximum number of *initial* intervals per boundary trim; a pcurve with more knot
    /// spans than this is merged uniformly down to it (adaptivity then re-refines).
    int max_init_intervals = 512;
    /// Use the planar closed-form path when a face's surface is geometrically planar.
    bool use_analytic = true;
};

enum class MassPropsPath : int {
    Skipped = 0,      ///< face contributed nothing (invalid surface)
    PlanarExact = 1,  ///< closed-form planar boundary integral (exact for a plane)
    GreenUV = 2,      ///< Green-reduced UV quadrature over the trimmed region
    FullDomain = 3    ///< trim loops absent or degenerate -> the whole parameter rectangle
};

struct FaceMassProps {
    int face_index = -1;
    double area = 0.0;        ///< |trimmed area|
    double area_err = 0.0;
    double flux = 0.0;        ///< II S.(Su x Sv) du dv over the trimmed region (NATURAL normal)
    double flux_err = 0.0;
    double mom[3] = {0, 0, 0};///< II (S_k^2/2)(Su x Sv)_k du dv, natural normal, k = x,y,z
    double vec_area[3] = {0, 0, 0};  ///< II (Su x Sv) du dv, natural normal (closure certificate)
    double traversal = 1.0;   ///< +1 if stored loop traversal is CCW in UV, else -1
    double outward = 1.0;     ///< +1 if the natural normal points OUT of the solid
    double chain_gap = 0.0;   ///< worst UV gap when chaining this face's trims head-to-tail,
                              ///< relative to the surface domain size (0 = perfect chain)
    int shell = -1;
    MassPropsPath path = MassPropsPath::Skipped;
    long long evals = 0;
    bool budget_hit = false;
};

struct MassProps {
    double volume = 0.0;
    double volume_error = 0.0;   ///< absolute error estimate on `volume`
    double area = 0.0;
    double area_error = 0.0;
    Point centroid = Point(0, 0, 0);
    bool converged = true;       ///< false => the budget/depth cap stopped refinement early
    bool closed = true;          ///< false => open shell, the divergence theorem does not apply
    /// |sum of outward face vector areas| / total area. Exactly 0 for a closed shell. When it
    /// is not small the boundary does not close GEOMETRICALLY (even if it closes
    /// topologically), the flux integral is origin-dependent, and `volume` must not be
    /// trusted -- `volume_error` is inflated by the resulting uncertainty and `closed` is set
    /// false. `area` and `closure_residual` themselves stay meaningful.
    double closure_residual = 0.0;
    /// Worst per-face trim chain gap (see FaceMassProps::chain_gap). Green's theorem needs a
    /// CLOSED boundary in UV; a large value means the input's trim loops do not chain and the
    /// per-face integral over "the trimmed region" is not well posed.
    double max_chain_gap = 0.0;
    int naked_edges = 0;         ///< edges with != 2 trims
    int shell_count = 0;
    std::vector<double> shell_volumes;  ///< signed (cavity shells are negative)
    long long surface_evals = 0;
    double seconds = 0.0;
    std::vector<FaceMassProps> faces;
};

/// Full mass properties of `brep`. Never loops unboundedly (see the contract above).
MassProps brep_massprops(const BRep& brep, const MassPropsOptions& opt = MassPropsOptions());

/// Convenience wrappers.
double brep_volume(const BRep& brep, const MassPropsOptions& opt = MassPropsOptions());
double brep_area(const BRep& brep, const MassPropsOptions& opt = MassPropsOptions());
Point  brep_centroid(const BRep& brep, const MassPropsOptions& opt = MassPropsOptions());

}  // namespace session_cpp
