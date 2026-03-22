#include "mini_test.h"
#include "remesh_nurbssurface_adaptive.h"
#include "primitives.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("RemeshNurbssurfaceAdaptive", "Constructor") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    RemeshNurbssurfaceAdaptive ta(s);

    MINI_CHECK(ta.get_max_angle() == 20.0);
    MINI_CHECK(ta.get_max_edge_length() == 0.0);
    MINI_CHECK(ta.get_min_edge_length() == 0.0);
    MINI_CHECK(ta.get_max_chord_height() == 0.0);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Parameters") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    RemeshNurbssurfaceAdaptive ta(s);
    ta.set_max_angle(15.0)
      .set_max_edge_length(2.0)
      .set_min_edge_length(0.1)
      .set_max_chord_height(0.05);

    MINI_CHECK(ta.get_max_angle() == 15.0);
    MINI_CHECK(ta.get_max_edge_length() == 2.0);
    MINI_CHECK(ta.get_min_edge_length() == 0.1);
    MINI_CHECK(ta.get_max_chord_height() == 0.05);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Mesh") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    RemeshNurbssurfaceAdaptive ta(s);
    Mesh m = ta.mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Torus") {
    NurbsSurface s = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
    Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Cylinder") {
    NurbsSurface s = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Cone") {
    NurbsSurface s = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Doubly Curved") {
    NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

MINI_TEST("RemeshNurbssurfaceAdaptive", "Flat") {
    NurbsSurface s = Primitives::wave_surface(1.0, 0.0);
    Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

} // namespace session_cpp
