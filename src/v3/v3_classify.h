#pragma once

// v3 solid classifier + orientation.
// Classification is exact and mesh-free (BRepClass3d equivalent): the closest
// boundary point decides IN/OUT via the outward face normal; ray parity is the
// fallback when the closest feature is an edge/vertex (boundary-dominant).

#include "v3_topo.h"

namespace v3 {

enum class PtCls { IN, OUT, ON };

// outward unit normal of a face at (u,v): Srf::normal with the rev flag applied
V3 face_outward_normal(const Solid& s, const Face& f, double u, double v);

// closest boundary point on one face (UV inside loops); returns false if the
// face is empty. `dist2` is squared distance to `p`. When `on_bnd`/`edge_out`
// are given they report whether the winning point lies on the face's boundary
// (and which edge) -- the face half-space classification test is only valid
// for interior winners.
bool closest_on_face(const Solid& s, const Face& f, const V3& p,
                     double& u, double& v, double& dist2,
                     bool* on_bnd = nullptr, int* edge_out = nullptr);

// closest point on the solid boundary: face + uv (or edge when boundary-
// dominant: edge >= 0), squared distance.
struct ClosestInfo {
    int face = -1;
    double u = 0, v = 0;
    int edge = -1;      // >= 0 when the closest feature is an edge/vertex
    double et = 0;
    bool on_boundary = false; // face winner lies on the face boundary
    double dist2 = 1e300;
};
ClosestInfo closest_on_solid(const Solid& s, const V3& p);

PtCls classify_point(const Solid& s, const V3& p, double tol = 1e-7);

// Flip face rev flags so the closed shell is consistently outward-oriented:
// BFS propagation of the mated-edge traversal rule (the two coedges of an
// edge must run in opposite directions), then one signed-volume sign check
// per connected component. No-op for already-consistent solids.
void orient_solid(Solid& s);

} // namespace v3
