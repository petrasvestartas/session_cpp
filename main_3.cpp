#include <filesystem>
#include "mesh.h"
#include "nurbssurface.h"
#include "primitives.h"
#include "remesh_nurbssurface_adaptive.h"
#include "session.h"
using namespace session_cpp;

int main() {
    Session session("RemeshNurbssurfaceAdaptive");

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "RemeshNurbssurfaceAdaptive.pb").string();

    // Sphere
    {
        NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Sphere: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Torus
    {
        NurbsSurface s = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Torus: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Cylinder
    {
        NurbsSurface s = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Cylinder: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Cone
    {
        NurbsSurface s = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Cone: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Doubly curved
    {
        NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Doubly Curved: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Flat quad — 2x2 bilinear patch
    {
        NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
            Point(0,0,0), Point(0,4,0),
            Point(4,0,0), Point(4,4,0),
        });
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Flat Quad: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Curved triangle — degree-2 surface with singular north edge (apex)
    {
        NurbsSurface s = NurbsSurface::create(false, false, 2, 2, 3, 3, {
            Point(0,0,0), Point(2,0,3), Point(4,0,0),
            Point(0,2,2), Point(2,2,5), Point(4,2,2),
            Point(2,4,0), Point(2,4,0), Point(2,4,0),
        });
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Curved Triangle: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Flat triangle — v=max edge collapsed to apex
    {
        NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
            Point(0,0,0), Point(2,4,0),
            Point(4,0,0), Point(2,4,0),
        });
        Mesh m = RemeshNurbssurfaceAdaptive(s).mesh();

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Flat Triangle: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    session.pb_dump(fp);
    return 0;
}
