#include "mini_test.h"
#include "remesh_nurbssurface_grid.h"
#include "primitives.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

// U-collapsed corners retain their fan normal on every nondegenerate triangle.
MINI_TEST("RemeshNurbsSurfaceGrid", "Singular Planar Normal") {
    NurbsSurface surface = NurbsSurface::create(false,false,1,1,2,2,{
        Point(0,0,1),Point(0,0,1),Point(-1,0,0),Point(1,0,0)});
    Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v_q(surface,0,0,5.0,0.001);
    bool apex = false;
    for (const auto& [key, face] : mesh.face) {
        const auto& a = mesh.vertex.at(face[0]);
        const auto& b = mesh.vertex.at(face[1]);
        const auto& c = mesh.vertex.at(face[2]);
        if (std::abs((b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x)) <= 1e-14) continue;
        for (size_t vertex_key : face) {
            const auto& vertex = mesh.vertex.at(vertex_key);
            auto normal = vertex.normal().value();
            MINI_CHECK(std::abs(normal[0]) < 1e-12 && std::abs(normal[2]) < 1e-12);
            MINI_CHECK(std::abs(std::abs(normal[1])-1.0) < 1e-12);
            apex = apex || vertex.z == 1.0;
        }
    }
    MINI_CHECK(apex);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Crease Normals") {
    NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 3, 2, {
        Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0),
        Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0),
        Point(2.0, 0.0, 1.0), Point(2.0, 1.0, 1.0),
    });
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);
    MINI_CHECK(m.vertex.size() == 8);
    MINI_CHECK(m.face.size() == 4);
    int flat = 0, tilted = 0;
    for (const auto& [key, vd] : m.vertex) {
        if (vd.x != 1.0) continue;
        auto n = vd.normal().value();
        if (std::abs(n[0]) < Tolerance::ZERO_TOLERANCE) ++flat;
        if (std::abs(n[0] + std::sqrt(0.5)) < Tolerance::ZERO_TOLERANCE) ++tilted;
    }
    MINI_CHECK(flat == 2 && tilted == 2);
}


MINI_TEST("RemeshNurbsSurfaceGrid", "Analytic Normals") {
    std::vector<NurbsSurface> surfaces = {
        Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0),
        Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0),
        Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0),
    };
    for (size_t index = 0; index < surfaces.size(); ++index) {
        const auto& s = surfaces[index];
        Mesh m = RemeshNurbsSurfaceGrid::from_u_v_q(s, 0, 0, 30.0, 0.01);
        for (const auto& [key, vd] : m.vertex) {
            const auto n = vd.normal().value();
            double length = n[0]*n[0] + n[1]*n[1] + n[2]*n[2];
            MINI_CHECK(std::abs(length - 1.0) < Tolerance::ZERO_TOLERANCE);
            if (index < 2) {
                double z = index == 0 ? vd.z : 0.0;
                double dot = vd.x*n[0] + vd.y*n[1] + z*n[2];
                MINI_CHECK(std::abs(dot - 1.0) < Tolerance::ZERO_TOLERANCE);
            }
        }
    }
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Sphere") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "primitives.h"
    NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 191);
    MINI_CHECK(m.number_of_faces() == 378);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Torus") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "primitives.h"
    NurbsSurface s = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 693);
    MINI_CHECK(m.number_of_faces() == 1386);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Cylinder") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "primitives.h"
    NurbsSurface s = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 42);
    MINI_CHECK(m.number_of_faces() == 42);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Cone") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "primitives.h"
    NurbsSurface s = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 22);
    MINI_CHECK(m.number_of_faces() == 21);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Doubly Curved") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "primitives.h"
    NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 961);
    MINI_CHECK(m.number_of_faces() == 1800);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Grid Target") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "primitives.h"
    NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    Mesh m_lo = RemeshNurbsSurfaceGrid::from_u_v(s, 8, 8);
    Mesh m_hi = RemeshNurbsSurfaceGrid::from_u_v(s, 32, 32);

    MINI_CHECK(m_lo.is_valid());
    MINI_CHECK(m_lo.number_of_vertices() == 64);
    MINI_CHECK(m_hi.is_valid());
    MINI_CHECK(m_hi.number_of_vertices() > m_lo.number_of_vertices());
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Flat Quad") {
    // uncomment #include "remesh_nurbssurface_grid.h"
    // uncomment #include "mesh.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "point.h"
    NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0, 0, 0),
        Point(0, 4, 0),
        Point(4, 0, 0),
        Point(4, 4, 0),
    });
    Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() == 4);
    MINI_CHECK(m.number_of_faces() == 2);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Flat Triangle") {
    NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0, 0, 0),
        Point(2, 4, 0),
        Point(4, 0, 0),
        Point(2, 4, 0),
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
