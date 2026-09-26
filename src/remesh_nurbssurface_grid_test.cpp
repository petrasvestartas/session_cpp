#include "mini_test.h"
#include "remesh_nurbssurface_grid.h"
#include "mesh.h"
#include "nurbssurface.h"
#include "point.h"
#include "primitives.h"
#include "tolerance.h"
#include <array>
#include <cmath>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("RemeshNurbsSurfaceGrid", "Singular Planar Normal") {
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Mesh;
    // using session_cpp::VertexData;

    const NurbsSurface surface = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {Point(0, 0, 1), Point(0, 0, 1), Point(-1, 0, 0), Point(1, 0, 0)}
    );
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v_q(surface, 0, 0, 5.0, 0.001);
    bool apex = false;

    for (const std::pair<const size_t, std::vector<size_t>>& entry : mesh.face) {
        const std::vector<size_t>& face = entry.second;
        const VertexData& a = mesh.vertex.at(face[0]);
        const VertexData& b = mesh.vertex.at(face[1]);
        const VertexData& c = mesh.vertex.at(face[2]);

        if (std::abs((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)) <= 1e-14)
            continue;

        for (size_t vertex_key : face) {
            const VertexData& vertex = mesh.vertex.at(vertex_key);
            const std::array<double, 3> normal = vertex.normal().value();

            MINI_CHECK(std::abs(normal[0]) < 1e-12 && std::abs(normal[2]) < 1e-12);
            MINI_CHECK(std::abs(std::abs(normal[1]) - 1.0) < 1e-12);

            apex = apex || vertex.z == 1.0;
        }
    }

    MINI_CHECK(apex);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Crease Normals") {
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Mesh;
    // using session_cpp::VertexData;

    const NurbsSurface surface = NurbsSurface::create(
        false,
        false,
        1,
        1,
        3,
        2,
        {
            Point(0.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 1.0),
            Point(2.0, 1.0, 1.0),
        }
    );
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.vertex.size() == 8);
    MINI_CHECK(mesh.face.size() == 4);

    int flat = 0;
    int tilted = 0;

    for (const std::pair<const size_t, VertexData>& entry : mesh.vertex) {
        const VertexData& vd = entry.second;

        if (vd.x != 1.0)
            continue;

        const std::array<double, 3> normal = vd.normal().value();

        if (std::abs(normal[0]) < Tolerance::ZERO_TOLERANCE)
            ++flat;

        if (std::abs(normal[0] + std::sqrt(0.5)) < Tolerance::ZERO_TOLERANCE)
            ++tilted;
    }

    MINI_CHECK(flat == 2 && tilted == 2);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Analytic Normals") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;
    // using session_cpp::VertexData;

    const std::vector<NurbsSurface> surfaces = {
        Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0),
        Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0),
        Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0),
    };

    for (size_t index = 0; index < surfaces.size(); ++index) {
        const NurbsSurface& surface = surfaces[index];
        const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v_q(surface, 0, 0, 30.0, 0.01);

        for (const std::pair<const size_t, VertexData>& entry : mesh.vertex) {
            const VertexData& vd = entry.second;
            const std::array<double, 3> normal = vd.normal().value();
            const double length = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];

            MINI_CHECK(std::abs(length - 1.0) < Tolerance::ZERO_TOLERANCE);

            if (index < 2) {
                const double z = index == 0 ? vd.z : 0.0;
                const double dot = vd.x * normal[0] + vd.y * normal[1] + z * normal[2];

                MINI_CHECK(std::abs(dot - 1.0) < Tolerance::ZERO_TOLERANCE);
            }
        }
    }
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Sphere") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::sphere_surface(0, 0, 0, 1.0);
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 191);
    MINI_CHECK(mesh.number_of_faces() == 378);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Sphere Few Rows") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::sphere_surface(0, 0, 0, 1.0);
    const Mesh one = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 1);
    const Mesh two = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 2);
    const Mesh three = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 3);

    MINI_CHECK(one.number_of_vertices() == 0);
    MINI_CHECK(two.number_of_vertices() == 0);
    MINI_CHECK(three.is_valid());
    MINI_CHECK(three.number_of_vertices() == 23);
    MINI_CHECK(three.number_of_faces() == 42);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Torus") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 693);
    MINI_CHECK(mesh.number_of_faces() == 1386);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Cylinder") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 42);
    MINI_CHECK(mesh.number_of_faces() == 42);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Cone") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 22);
    MINI_CHECK(mesh.number_of_faces() == 21);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Doubly Curved") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::wave_surface(1.0, 0.5);
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 961);
    MINI_CHECK(mesh.number_of_faces() == 1800);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Grid Target") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;

    const NurbsSurface surface = Primitives::wave_surface(1.0, 0.5);
    const Mesh mesh_lo = RemeshNurbsSurfaceGrid::from_u_v(surface, 8, 8);
    const Mesh mesh_hi = RemeshNurbsSurfaceGrid::from_u_v(surface, 32, 32);

    MINI_CHECK(mesh_lo.is_valid());
    MINI_CHECK(mesh_lo.number_of_vertices() == 64);
    MINI_CHECK(mesh_hi.is_valid());
    MINI_CHECK(mesh_hi.number_of_vertices() > mesh_lo.number_of_vertices());
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Flat Quad") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const NurbsSurface surface = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {
            Point(0, 0, 0),
            Point(0, 4, 0),
            Point(4, 0, 0),
            Point(4, 4, 0),
        }
    );
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 4);
    MINI_CHECK(mesh.number_of_faces() == 2);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Flat Triangle") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const NurbsSurface surface = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {
            Point(0, 0, 0),
            Point(2, 4, 0),
            Point(4, 0, 0),
            Point(2, 4, 0),
        }
    );
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 3);
    MINI_CHECK(mesh.number_of_faces() == 1);
}

MINI_TEST("RemeshNurbsSurfaceGrid", "Double-Curved Triangle") {
    // using session_cpp::RemeshNurbsSurfaceGrid;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const NurbsSurface surface = NurbsSurface::create(
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
    const Mesh mesh = RemeshNurbsSurfaceGrid::from_u_v(surface, 0, 0);

    MINI_CHECK(mesh.is_valid());
    MINI_CHECK(mesh.number_of_vertices() == 64);
    MINI_CHECK(mesh.number_of_faces() == 98);
}

} // namespace session_cpp
