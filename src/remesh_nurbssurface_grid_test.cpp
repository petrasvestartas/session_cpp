#include "mini_test.h"
#include "remesh_nurbssurface_grid.h"
#include "primitives.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("RemeshNurbsSurfaceGrid", "Sphere") {
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 191);
    MINI_CHECK(m.number_of_faces() == 378);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Torus") {
    NurbsSurface s = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 693);
    MINI_CHECK(m.number_of_faces() == 1386);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Cylinder") {
    NurbsSurface s = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 42);
    MINI_CHECK(m.number_of_faces() == 42);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Cone") {
    NurbsSurface s = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 22);
    MINI_CHECK(m.number_of_faces() == 21);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Doubly Curved") {
    NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 961);
    MINI_CHECK(m.number_of_faces() == 1800);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Grid Target") {
    NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    Mesh m_lo = RemeshNurbsSurfaceGrid::from_u_v(s, 8, 8);
    Mesh m_hi = RemeshNurbsSurfaceGrid::from_u_v(s, 32, 32);

    MINI_CHECK(m_lo.is_valid());
    MINI_CHECK(m_lo.number_of_vertices() == 64);
    MINI_CHECK(m_hi.is_valid());
    MINI_CHECK(m_hi.number_of_vertices() > m_lo.number_of_vertices());
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Flat Quad") {
    NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0,0,0), Point(0,4,0),
        Point(4,0,0), Point(4,4,0),
    });
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 4);
    MINI_CHECK(m.number_of_faces() == 2);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Flat Triangle") {
    NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0,0,0), Point(2,4,0),
        Point(4,0,0), Point(2,4,0),
    });
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 3);
    MINI_CHECK(m.number_of_faces() == 1);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Double-Curved Triangle") {
    NurbsSurface s = NurbsSurface::create(false, false, 2, 2, 3, 3, {
        Point(0, 0, 0),
        Point(2, 0, 3),
        Point(4, 0, 0),
        Point(0, 2, 2),
        Point(2, 2, 5),
        Point(4, 2, 2),
        Point(2, 4, 0),
        Point(2, 4, 0),
        Point(2, 4, 0),
    });
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 64);
    MINI_CHECK(m.number_of_faces() == 98);
}

} // namespace session_cpp
