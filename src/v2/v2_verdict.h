#pragma once

// THE SINGLE SHARED VERDICT HARNESS  (kb/PLAN_v2_execution.md §1 0.2)
// ------------------------------------------------------------------
// One function decides whether a BRep is a closed solid. Every v2 test driver and every sweep
// calls it; no agent scores anything with its own code. Five measurement errors in one session
// -- three of which INVERTED a headline conclusion -- came from per-agent verdict metrics that
// disagreed with the kernel's own validity rule, always on the same shapes: a sphere's poles, a
// cone's apex, a cylinder's or torus's seam.
//
// The two rules that produce those errors, stated once, here:
//
//   * A DEGENERATE edge (3D curve collapsed to a point: a pole, an apex) is watertight by
//     construction -- the point is fully enclosed by its own face -- and is EXCLUDED from the
//     manifold count. This is what BRep::is_solid() does (brep.cpp) and what OCCT does.
//   * A SEAM edge is used TWICE by the SAME face's wires. It has two uses, so it is manifold and
//     is NOT naked. Counting distinct adjacent faces instead of uses calls an intact sphere open.
//
// Topology alone is not enough: a shell can close topologically while its faces do not meet
// GEOMETRICALLY. So the verdict also carries the closure certificate from brep_massprops --
// |sum of outward face vector areas| / area, exactly 0 for a closed boundary -- and closed()
// requires both. The mass properties come from src/brep_massprops.h (28/28 tests, exact to
// 1e-15, always terminates by construction); nothing here integrates anything itself.
//
// The harness is validated against BRep::is_solid() on box, sphere, cone, cylinder and torus,
// un-split and under rigid motion, by main_17.cpp. It must not be used to score anything until
// that driver passes.
//
// USE:
//     #include "src/v2/v2_verdict.h"
//     using session_cpp::v2v::v2_verdict;
//     const auto v = v2_verdict(result);
//     if (!v.closed()) ...            // v.str() prints every field
//
// Call v2_verdict explicitly through the v2v namespace: v2sol::V2Verdict is a different,
// topology-only struct and an unqualified `using namespace` of both would be ambiguous.

#include "brep.h"
#include "brep_massprops.h"

#include <string>

namespace session_cpp {
namespace v2v {

struct V2Verdict {
    int faces = 0;
    int shells = 0;            ///< connected components of faces joined across 2-use edges
    int solids = 0;            ///< outer (positive-volume) shells; 0 unless closed()

    int naked_real = 0;        ///< edges with ONE face use, EXCLUDING zero-length degenerates
    int nonmanifold = 0;       ///< edges with MORE than two face uses
    int seam_edges = 0;        ///< used twice by ONE face's wires -- manifold, NOT naked

    int edges = 0;             ///< non-degenerate edges carrying at least one use
    int degenerate = 0;        ///< zero-length edges (poles, apexes): excluded from naked_real
    int orphan = 0;            ///< edge records with zero uses: dead entries, no topological role

    double closure_residual = 1.0;  ///< |sum of outward vector areas| / area  (brep_massprops)
    double area = 0.0;
    double volume = 0.0;
    bool volume_valid = false;      ///< closed() AND the quadrature converged
    bool converged = true;          ///< brep_massprops finished inside its evaluation budget
    bool has_topology = false;      ///< at least one face and one edge record, as is_solid needs

    /// The verdict. Topologically 2-manifold AND geometrically closed.
    bool closed() const {
        return has_topology && naked_real == 0 && nonmanifold == 0 && closure_residual < 1e-9;
    }

    std::string str() const;
};

/// Default mass-properties settings for scoring: full 1e-10 relative accuracy, but a bounded
/// budget so a sweep of thousands of cells cannot spend minutes on one broken shape.
MassPropsOptions v2_verdict_options();

/// THE verdict of `b`. Degeneracy-aware, seam-aware, geometry-checked.
V2Verdict v2_verdict(const BRep& b);
V2Verdict v2_verdict(const BRep& b, const MassPropsOptions& opt);

/// Topology only -- no quadrature. closure_residual stays at its 1.0 sentinel and closed() is
/// therefore always false, so this must NEVER be used to declare a success. It exists for
/// cheap inner-loop diagnostics (counting naked edges while debugging a splitter).
V2Verdict v2_verdict_topology_only(const BRep& b);

}  // namespace v2v
}  // namespace session_cpp
