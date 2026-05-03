#include "mini_test.h"
#include "reciprocal.h"
#include "mesh.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Reciprocal", "from_mesh") {
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(2, 0, 0),
            Point(0, 1, 0),
            Point(1, 1, 0),
            Point(2, 1, 0),
            Point(0, 2, 0),
            Point(1, 2, 0),
            Point(2, 2, 0),
        };
        std::vector<std::vector<size_t>> faces = {
            {0, 1, 4, 3},
            {1, 2, 5, 4},
            {3, 4, 7, 6},
            {4, 5, 8, 7},
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, faces);
        auto r = Reciprocal::from_mesh(mesh, 0.7, 1.4, true, 1.0);
        int ne = (int)mesh.number_of_edges();
        MINI_CHECK((int)r.center.size() == ne);
        MINI_CHECK((int)r.top.size() == ne);
        MINI_CHECK((int)r.bottom.size() == ne);
        MINI_CHECK((int)r.lineplanes.size() == ne);
        MINI_CHECK((int)r.endplanes.size() == ne);
        MINI_CHECK(r.center[0].length() > 0.0);
        MINI_CHECK(r.lineplanes[0].is_valid());
    }

} // namespace session_cpp
