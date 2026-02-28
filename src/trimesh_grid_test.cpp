#include "mini_test.h"
#include "trimesh_grid.h"
#include "primitives.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("TrimeshGrid", "Constructor") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    TrimeshGrid tg(s);

    MINI_CHECK(tg.get_max_angle() == 20.0);
    MINI_CHECK(tg.get_max_edge_length() == 0.0);
    MINI_CHECK(tg.get_min_edge_length() == 0.0);
    MINI_CHECK(tg.get_max_chord_height() == 0.0);
}

MINI_TEST("TrimeshGrid", "Parameters") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    TrimeshGrid tg(s);
    tg.set_max_angle(15.0)
      .set_max_edge_length(2.0)
      .set_min_edge_length(0.1)
      .set_max_chord_height(0.05);

    MINI_CHECK(tg.get_max_angle() == 15.0);
    MINI_CHECK(tg.get_max_edge_length() == 2.0);
    MINI_CHECK(tg.get_min_edge_length() == 0.1);
    MINI_CHECK(tg.get_max_chord_height() == 0.05);
}

MINI_TEST("TrimeshGrid", "Mesh") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    TrimeshGrid tg(s);
    Mesh m = tg.mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

} // namespace session_cpp
