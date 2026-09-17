#pragma once
#include <array>
#include <utility>
#include <vector>
#include "mesh.h"
#include "polyline.h"

namespace session_cpp {

/// Constrained Delaunay triangulation of a border polyline with hole polylines.
struct RemeshCDT {
    /// Triangle index triples into the flat list [border..., hole0..., hole1...], closing duplicates stripped.
    static std::vector<std::array<int, 3>> triangulate(const std::vector<Polyline>& polylines);

    /// Mesh of one face with holes, or one face per triangle under SESSION_CONFIG.explode_mesh_faces; is_2d skips the plane projection, is_first_boundary=false picks the border by largest bbox diagonal.
    static Mesh from_polylines(
        const std::vector<Polyline>& polylines,
        bool is_2d = false,
        bool is_first_boundary = true
    );
};

} // namespace session_cpp
