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

    // Removes vertex 0 + its 3 adjacent faces (0,2,4)
    // vertices() → [1,2,3,4,5,6,7]   ← position 0 is now key 1
    // faces()    → [1,3,5]
    mesh.remove_vertex(0);

    // Removes faces adjacent to (1,2): face 5 [1,2,6,5] (face 0 already gone)
    // vertices() → [1,2,3,4,5,6,7]   ← vertex keys unchanged
    // faces()    → [1,3]
    // edges()    → (1,2) pair gone
    mesh.remove_edge(1,2);

    // Removes face 1 [4,5,6,7]
    // faces()    → [3]                ← only face 3 [2,3,7,6] remains
    mesh.remove_face(1);

    // Removes everything
    mesh.clear();

    // Bring back original mesh for testing batch operations
    for (const auto& v : vertices)
        mesh.add_vertex(v);

    for (const auto& f : faces)
        mesh.add_face(f);

    session.add_mesh(std::make_shared<Mesh>(mesh));


    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "vertex_and_face_attributes.pb").string();
    std::cout << "Dumping session to " << fp << std::endl;
    session.pb_dump(fp);

    return 0;
}
