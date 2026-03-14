#include "session.h"
#include "primitives.h"
#include "xform.h"

using namespace session_cpp;

int main() {
    Session session("mesh_color");

    Mesh mesh = Mesh::from_polylines({
        {
            {1.28955, 0, 1.127558},
            {0.85791, 0, 0.225512},
            {0.64209, -0.866025, -0.225512},
            {0.85791, -1.732051, 0.225512},
            {1.458565, -1.732051, 1.127558},
            {1.50537, -0.866025, 1.578581},
        },
        {
            {0.64209, 0.866025, -0.225512},
            {0.114274, 0.866025, -0.686294},
            {-0.00537, 0, -1.578581},
            {0.21045, -0.866025, -1.127558},
            {0.64209, -0.866025, -0.225512},
            {0.85791, 0, 0.225512},
        },
        {
            {1.28955, 1.732051, 1.127558},
            {0.85791, 1.732051, 0.225512},
            {0.64209, 0.866025, -0.225512},
            {0.85791, 0, 0.225512},
            {1.28955, -0, 1.127558},
            {1.853404, 0.866025, 1.578581},
        },
    }, 0.001);

    size_t v0 = 1;
    size_t v1 = 2;
    size_t v2 = 3;
    size_t f0 = 0;


    bool not_empty = mesh.is_empty();

    bool valid = mesh.is_valid();

    bool closed = mesh.is_closed();

    bool vertex_on_boundary = mesh.is_vertex_on_boundary(v0);

    bool edge_not_on_boundary = mesh.is_edge_on_boundary(v0, v1);
    bool edge_on_boundary = mesh.is_edge_on_boundary(v1, v2);

    bool face_on_boundary = mesh.is_face_on_boundary(f0);

    std::cout << "Mesh is_empty: " << not_empty << std::endl;
    std::cout << "Mesh is_valid: " << valid << std::endl;
    std::cout << "Mesh is_closed: " << closed << std::endl;
    std::cout << "Vertex " << v0 << " on boundary: " << vertex_on_boundary << std::endl;
    std::cout << "Edge (" << v0 << ", " << v1 << ") on boundary: " << edge_not_on_boundary << std::endl;
    std::cout << "Edge (" << v1 << ", " << v2 << ") on boundary: " << edge_on_boundary << std::endl;

    std::cout << "Face " << f0 << " on boundary: " << face_on_boundary << std::endl;

    session.add_mesh(std::make_shared<Mesh>(mesh));
    for (size_t i = 0; i < mesh.vertex.size(); ++i) {
        Point p = mesh.vertex[i].position();
        p.name = std::to_string(i);
        session.add_point(std::make_shared<Point>(p));
    }
    session.add_point(std::make_shared<Point>(mesh.vertex[0].position()));


    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path()
                            / "session_data" / "mesh.pb").string();
    session.pb_dump(filepath);
    return 0;
}
