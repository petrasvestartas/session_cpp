#pragma once
#include <map>
#include "mesh.h"
#include "plane.h"

namespace session_cpp {

/// Thick shell of a mesh: original faces, offset faces, quads on naked edges.
struct MeshOffset {
    /// Top, bottom and side meshes of a shell.
    struct Layers {
        Mesh top; // Offset faces.
        Mesh bottom; // Reversed original faces.
        Mesh sides; // One quad per naked edge.
    };

    /// One closed mesh: reversed bottom, offset top, one quad per naked edge.
    static Mesh from_mesh(const Mesh& mesh, double distance);

    /// The same shell as three meshes: top, bottom and sides.
    static Layers from_mesh_layers(const Mesh& mesh, double distance);

    /// Plane of each face translated by distance along its normal, by face key.
    static std::map<size_t, Plane> offset_planes(const Mesh& mesh, double distance);

    /// Offset position of each vertex: least-squares meet of its face planes, by vertex key.
    static std::map<size_t, Point> offset_vertices(const Mesh& mesh, const std::map<size_t, Plane>& planes);
};

} // namespace session_cpp
