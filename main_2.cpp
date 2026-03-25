#include <filesystem>
#include "mesh.h"
#include "nurbssurface.h"
#include "primitives.h"
#include "remesh_nurbssurface_grid.h"
#include "session.h"
using namespace session_cpp;

int main() {
    Session session("RemeshNurbsSurfaceGrid");

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "RemeshNurbsSurfaceGrid.pb").string();

    // // Sphere
    // {
    //     NurbsSurface s = Primitives::sphere_surface(0, 0, 0, 1.0);
    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 191;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Sphere: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";


    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // // Torus
    // {
    //     NurbsSurface s = Primitives::torus_surface(0, 0, 0, 3.0, 1.0);
    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 693;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Cone: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // // Cylinder
    // {
    //     NurbsSurface s = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0);
    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 42;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Cylinder: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";


    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // // Cone
    // {
    //     NurbsSurface s = Primitives::cone_surface(0, 0, 0, 1.0, 5.0);
    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 22;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Cone: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // // Doubly curved — quality vs grid target
    // {
    //     NurbsSurface s = Primitives::wave_surface(1.0, 0.5);
    
    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 961;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Cone: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // // Flat quad — 2x2 bilinear patch, expect 4 vertices, 2 faces
    // {
    //     NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
    //         Point(0,0,0), Point(0,4,0),
    //         Point(4,0,0), Point(4,4,0),
    //     });

    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 4;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Cone: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // // Flat triangle — v=max edge collapsed to apex, expect 3 vertices, 1 face
    // {
    //     NurbsSurface s = NurbsSurface::create(false, false, 1, 1, 2, 2, {
    //         Point(0,0,0), Point(2,4,0),
    //         Point(4,0,0), Point(2,4,0),
    //     });
    //     Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);
    //     bool is_valid = m.is_valid();
    //     bool has_correct_vertex_count = m.number_of_vertices() == 3;
    //     std::cout << is_valid << " " << has_correct_vertex_count << " Cone: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

    //     session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    // }

    // Double-curved triangle — degree-2 in U and V, singular apex at v=max
    {
        NurbsSurface s = NurbsSurface::create(false, false, 2, 2, 3, 3, {
            Point(0,0,0), Point(2,0,3), Point(4,0,0),
            Point(0,2,2), Point(2,2,5), Point(4,2,2),
            Point(2,4,0), Point(2,4,0), Point(2,4,0),
        });
        Mesh m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        bool is_valid = m.is_valid();
        std::cout << is_valid << " Double-Curved Triangle: is_valid=" << is_valid << ", vertex_count=" << m.number_of_vertices() << "\n";

        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    session.pb_dump(fp);
    return 0;
}
