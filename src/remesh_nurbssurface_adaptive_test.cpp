#include "mini_test.h"
#include "remesh_nurbssurface_adaptive.h"
#include "primitives.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Constructor") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;

    const NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    const RemeshNurbsSurfaceAdaptive ta(s);

    MINI_CHECK(ta.get_max_angle() == 20.0);
    MINI_CHECK(ta.get_max_edge_length() == 0.0);
    MINI_CHECK(ta.get_min_edge_length() == 0.0);
    MINI_CHECK(ta.get_max_chord_height() == 0.0);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Parameters") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;

    const NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    RemeshNurbsSurfaceAdaptive ta(s);

    ta.set_max_angle(15.0).set_max_edge_length(2.0).set_min_edge_length(0.1).set_max_chord_height(0.05);

    MINI_CHECK(ta.get_max_angle() == 15.0);
    MINI_CHECK(ta.get_max_edge_length() == 2.0);
    MINI_CHECK(ta.get_min_edge_length() == 0.1);
    MINI_CHECK(ta.get_max_chord_height() == 0.05);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Mesh") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    const RemeshNurbsSurfaceAdaptive ta(s);
    const Mesh m = ta.mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 418);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Torus") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface s = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 1024);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Cylinder") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface s = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 64);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Cone") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface s = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 33);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Doubly Curved") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 1169);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Flat") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface s = Primitives::wave_surface(1.0, 0.0);
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 169);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Singular Triangle") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const NurbsSurface s = NurbsSurface::create(
        false,
        false,
        2,
        1,
        3,
        2,
        {
            Point(0, 0, 0),
            Point(2, 0, 3),
            Point(4, 0, 0),
            Point(2, 4, 0),
            Point(2, 4, 0),
            Point(2, 4, 0),
        }
    );
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 83);
}

MINI_TEST("RemeshNurbsSurfaceAdaptive", "Double-Curved Triangle") {
    // using session_cpp::RemeshNurbsSurfaceAdaptive;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const NurbsSurface s = NurbsSurface::create(
        false,
        false,
        2,
        2,
        3,
        3,
        {
            Point(0, 0, 0),
            Point(2, 0, 3),
            Point(4, 0, 0),
            Point(0, 2, 2),
            Point(2, 2, 5),
            Point(4, 2, 2),
            Point(2, 4, 0),
            Point(2, 4, 0),
            Point(2, 4, 0),
        }
    );
    const Mesh m = RemeshNurbsSurfaceAdaptive(s).mesh();

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 91);
}

} // namespace session_cpp
