#pragma once
#include <vector>
#include <array>
#include <utility>
#include "mesh.h"
#include "polyline.h"

namespace session_cpp {

struct RemeshCDT {
    // polylines[0]=border, rest=holes. Strips closing duplicate.
    // Returns (i,j,k) into flat [border..., hole0..., hole1...].
    // To build a Mesh from the result:
    //   Polyline border({Point(0,0,0), Point(4,0,0), Point(4,4,0), Point(0,4,0)});
    //   Polyline hole({Point(1,1,0), Point(1,3,0), Point(3,3,0), Point(3,1,0)});
    //   auto tris = RemeshCDT::triangulate({border, hole});
    //   std::vector<Point> flat;
    //   for (const auto& p : border.get_points()) flat.push_back(p);
    //   for (const auto& p : hole.get_points())   flat.push_back(p);
    //   Mesh m;
    //   std::vector<size_t> vkeys;
    //   for (const auto& p : flat) vkeys.push_back(m.add_vertex(p));
    //   for (const auto& t : tris) m.add_face({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});
    static std::vector<std::array<int,3>> triangulate(const std::vector<Polyline>& polylines);

    // Polylines → Mesh. is_2d=true skips plane projection (already XY).
    // is_first_boundary=false detects border by largest bbox diagonal.
    // Strips closing duplicate. Respects SESSION_CONFIG.explode_mesh_faces.
    static Mesh from_polylines(const std::vector<Polyline>& polylines,
                               bool is_2d = false,
                               bool is_first_boundary = true);
};

} // namespace session_cpp
