#pragma once
#include "mesh.h"
#include "plane.h"

namespace session_cpp {

struct MeshOffset {
    struct Layers {
        Mesh top;
        Mesh bottom;
        Mesh sides;
    };

    // Returns a closed shell: bottom (original) + top (offset) + side quads along boundary edges.
    static Mesh from_mesh(const Mesh& mesh, double distance);

    // Returns separate layers: top (offset faces), bottom (original faces), sides (boundary quads).
    static Layers from_mesh_layers(const Mesh& mesh, double distance);

    // Public helpers for visualization.
    // Per-face offset planes: each face plane translated by distance along its normal.
    static std::map<size_t, Plane> offset_planes(const Mesh& mesh, double distance);

    // Per-vertex offset positions: intersection of all adjacent offset planes.
    static std::map<size_t, Point> offset_vertices(
        const Mesh& mesh,
        const std::map<size_t, Plane>& planes);
};

} // namespace session_cpp
