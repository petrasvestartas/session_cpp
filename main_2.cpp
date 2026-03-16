#include <cassert>
#include <filesystem>
#include "fmt/format.h"
#include "mesh.h"
#include "session.h"

using namespace session_cpp;

int main() {


    // --- visual debug dump ---
    Session session("loft_debug");


    // Create Mesh
    double hx = 0.5, hy = 0.5, hz = 0.5;
    std::vector<Point> vertices = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1},  // bottom
        {4, 5, 6, 7},  // top
        {0, 1, 5, 4},  // front
        {2, 3, 7, 6},  // back
        {0, 4, 7, 3},  // left
        {1, 2, 6, 5},  // right
    };

    Mesh mesh = Mesh();

    for (const auto& v : vertices)
        mesh.add_vertex(v);

    for (const auto& f : faces)
        mesh.add_face(f);

    // Remove
    mesh.remove_vertex(0);
    mesh.remove_edge(1,2);
    mesh.remove_face(0);
    for (const auto& e : mesh.edges())
        std::cout << e.first << " " << e.second << std::endl;
    for (const auto& f : mesh.faces())
        std::cout << f << std::endl;
    session.add_mesh(std::make_shared<Mesh>(mesh));


    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "vertex_and_face_attributes.pb").string();
    std::cout << "Dumping session to " << fp << std::endl;
    session.pb_dump(fp);

    return 0;
}
