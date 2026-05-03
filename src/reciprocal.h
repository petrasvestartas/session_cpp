#pragma once
#include "mesh.h"
#include "line.h"
#include "plane.h"
#include <array>
#include <vector>

namespace session_cpp {

struct Reciprocal {
    struct Result {
        std::vector<Line>                center;     // rotated+extended lines (no offset)
        std::vector<Line>                top;        // shifted +height along Y axis
        std::vector<Line>                bottom;     // shifted -height along Y axis
        std::vector<Plane>               lineplanes; // plane per edge midpoint (X=dir, Y=normal)
        std::vector<std::array<Plane,2>> endplanes;  // [start_plane, end_plane] per edge
    };

    static Result from_mesh(
        const Mesh& mesh,
        double angle,
        double scale,
        bool   use_ngon_normals,
        double height
    );

private:
    static std::vector<Line> get_lines(
        const std::vector<Line>&             lines,
        const std::vector<Plane>&            line_planes,
        const std::vector<std::vector<int>>& face_edges,
        std::vector<std::array<Plane,2>>&    end_planes,
        double move = 0.0
    );
};

} // namespace session_cpp
