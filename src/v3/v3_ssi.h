#pragma once

// v3 surface-surface intersection (IntAna + IntWalk equivalent).
// Dispatch: exact closed-form cases for analytic pairs (line/circle/ellipse
// results with EXACT pcurves), otherwise a seeded Newton walker that produces
// deflection-controlled walk polylines carrying UV on both surfaces.
// Every SecCurve — exact or walked — carries sampled points with both UVs;
// the face splitter only ever consumes those samples (one code path).

#include "v3_geom.h"
#include <vector>

namespace v3 {

struct SecPoint {
    V3 p;
    double u1 = 0, v1 = 0, u2 = 0, v2 = 0; // UV on surface 1 / surface 2
    double t = 0;                          // parameter on exact curve (if any)
};

struct SecCurve {
    bool has_exact = false;
    Cur exact;                  // LINE/CIRCLE/ELLIPSE when has_exact
    double t0 = 0, t1 = 0;      // parameter range on exact (walk covers [t0,t1])
    bool closed = false;        // walk forms a closed loop
    std::vector<SecPoint> pts;  // ordered samples along the branch
};

struct SSIResult {
    std::vector<SecCurve> curves;
    bool same_domain = false; // surfaces coincide (caller handles same-domain)
};

// Intersect two full surfaces. `tol3d` is the model-space chord tolerance for
// walk sampling and coincidence decisions.
SSIResult ssi(const Srf& a, const Srf& b, double tol3d = 1e-7);

// Exact cases only (no marching). Returns false when no closed form exists —
// caller then runs the walker. Exposed for testing.
bool ssi_exact(const Srf& a, const Srf& b, SSIResult& out, double tol3d);

// Sample an exact curve branch into SecPoints with UVs on both surfaces,
// adaptive to chord deflection `tol3d`. Appends to out.
void sample_exact(const Srf& a, const Srf& b, const Cur& c, double t0, double t1,
                  bool closed, SSIResult& out, double tol3d);

// ---------------------------------------------------------------------------
// Curve-surface intersection (pave computation for edges).
// Returns (t on curve, point, u,v on surface) for every intersection found.
struct CSIPoint {
    double t;
    V3 p;
    double u = 0, v = 0;
};
std::vector<CSIPoint> csi(const Cur& c, double t0, double t1, const Srf& s,
                          double tol3d = 1e-7);

} // namespace v3
