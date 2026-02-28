#include "mini_test.h"
#include "trimesh_adaptive.h"
#include "primitives.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("TrimeshAdaptive", "Constructor") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    TrimeshAdaptive ta(s);

    MINI_CHECK(ta.get_max_angle() == 20.0);
    MINI_CHECK(ta.get_max_edge_length() == 0.0);
    MINI_CHECK(ta.get_min_edge_length() == 0.0);
    MINI_CHECK(ta.get_max_chord_height() == 0.0);
}

MINI_TEST("TrimeshAdaptive", "Parameters") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    TrimeshAdaptive ta(s);
    ta.set_max_angle(15.0)
      .set_max_edge_length(2.0)
      .set_min_edge_length(0.1)
      .set_max_chord_height(0.05);

    MINI_CHECK(ta.get_max_angle() == 15.0);
    MINI_CHECK(ta.get_max_edge_length() == 2.0);
    MINI_CHECK(ta.get_min_edge_length() == 0.1);
    MINI_CHECK(ta.get_max_chord_height() == 0.05);
}

MINI_TEST("TrimeshAdaptive", "Mesh") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    TrimeshAdaptive ta(s);
    Mesh m = ta.mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

} // namespace session_cpp
