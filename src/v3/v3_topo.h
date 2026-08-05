#pragma once

// v3 topology: indexed B-rep with first-class analytic geometry.
// Design laws (kb/BOOL_V3_MEMORY.md):
//  - one entity per geometric feature: split edges SHARE one Cur with
//    different [t0,t1] ranges; section curves are created once and referenced
//    by both operands (no sewing of copies).
//  - pcurves are DERIVED from the 3D edge curve through the face surface's
//    exact inverse (uv_of) — SameParameter-exact by construction; measured,
//    model-space tolerances only.
//  - degenerate (pole) edges are typed, never rediscovered.

#include "v3_geom.h"
#include "../brep.h"
#include <vector>

namespace v3 {

struct UV2 { double u = 0, v = 0; };

struct Vertex {
    V3 p;
    double tol = 1e-7;
};

struct Edge {
    Cur c;
    double t0 = 0, t1 = 0;   // v0 at t0, v1 at t1 (t along curve direction)
    int v0 = -1, v1 = -1;    // vertex indices
    bool degenerate = false; // pole edge: zero-length in 3D, segment in UV
    double tol = 1e-7;
    V3 p0() const { return c.eval(t0); }
    V3 p1() const { return c.eval(t1); }
};

struct CoEdge {
    int edge = -1;
    bool fwd = true;            // traversal along increasing t of the edge
    int face = -1;              // owning face
    std::vector<UV2> pc;        // pcurve polyline along TRAVERSAL (face chart,
                                // unwrapped/continuous within the owning loop)
    std::vector<double> pt;     // parallel: edge curve parameter per pc sample
};

struct Loop {
    std::vector<int> ces;       // coedge indices, chained head-to-tail
    bool outer = true;
};

struct Face {
    int srf = -1;
    bool rev = false;           // face normal opposite to surface normal
    std::vector<Loop> loops;
    int src = -1;               // provenance: operand face index (debug/SD)
};

struct Solid {
    std::vector<Vertex> verts;
    std::vector<Edge> edges;
    std::vector<CoEdge> coedges;
    std::vector<Face> faces;
    std::vector<Srf> srfs;

    void bbox(V3& lo, V3& hi) const;
    // closed: every edge has exactly 2 coedges
    bool is_closed(int* naked = nullptr, int* nonmanifold = nullptr) const;
    // divergence-theorem volume over grid-tessellated faces (sign = outward
    // orientation when faces are oriented outward)
    double signed_volume(int grid = 20) const;
    // UV-area of a loop's pcurve chain (signed; meaningful in the face chart)
    double loop_uv_area(const Face& f, const Loop& l) const;
};

// ---- conversion ---------------------------------------------------------------
Solid from_brep(const session_cpp::BRep& b);
session_cpp::BRep to_brep(const Solid& s);

// ---- face region queries ------------------------------------------------------
// signed distance from UV point to the face boundary in the face chart
// (positive inside), even-odd per loop with outer/inner signs. Uses the
// loop pcurve polylines (unwrapped); `shift_u/shift_v` re-anchor periodic
// charts around the query region.
bool uv_in_face(const Solid& s, const Face& f, double u, double v,
                double* signed_dist = nullptr);

// tessellate a face within its loops (uniform UV grid + inside test).
// Triangles in 3D, oriented along the surface normal (rev applied by caller).
void tessellate_face(const Solid& s, const Face& f, int grid,
                     std::vector<V3>& tris_out); // 3 verts per triangle

// shift a whole loop's coedge pcurves by (du, dv) — periodic chart re-anchoring
void shift_loop_uv(Solid& s, Face& f, Loop& l, double du, double dv);

} // namespace v3
