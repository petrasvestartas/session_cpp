#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "mesh.h"
#include "line.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "boundingbox.h"
#include "xform.h"
#include <iostream>

using namespace session_cpp;

int main() {
    Session session("surface_plane");
        const int sides = 6;

    // Create hexagon vertices in XY plane
    std::vector<Point> vertices;
    for (int i = 0; i < sides; ++i) {
        double angle = 2.0 * TOLERANCE.PI * i / sides;
        double x = 1.0 * std::cos(angle);
        double y = 1.0 * std::sin(angle);
        vertices.push_back({x, y, 0.0});
    }

    // Add center point as last vertex
    // vertices.push_back({0.0, 0.0, 0.0});
    // std::vector<std::vector<size_t>> faces = {
    //     {0, 1, 6},
    //     {1, 2, 6},
    //     {2, 3, 6},
    //     {3, 4, 6},
    //     {4, 5, 6},
    //     {5, 0, 6}
    // };
    std::vector<std::vector<size_t>> faces = {
        {0, 1, 2, 3, 4, 5},
    };

    Mesh mesh = Mesh::from_vertices_and_faces(vertices, faces);

    size_t num_vertices = mesh.number_of_vertices();
    size_t num_faces = mesh.number_of_faces();
    size_t num_edges = mesh.number_of_edges();
    bool is_empty = mesh.is_empty();
    int euler = mesh.euler();

    // String representations
    std::string sstr = mesh.str();
    std::string srepr = mesh.repr();

    // Copy (new guid)
    Mesh mcopy = mesh;

    // MINI_CHECK(num_vertices == 7);
    // MINI_CHECK(num_faces == 6);
    // MINI_CHECK(num_edges == 12);
    // MINI_CHECK(!is_empty);
    // MINI_CHECK(euler == 1);
    // MINI_CHECK(mesh.name == "my_mesh");
    // MINI_CHECK(!mesh.guid.empty());
    // MINI_CHECK(sstr.find("Mesh") != std::string::npos);
    // MINI_CHECK(srepr.find("name=my_mesh") != std::string::npos);
    // MINI_CHECK(mcopy.guid != mesh.guid);
    // MINI_CHECK(mcopy == mesh);
    // MINI_CHECK(!(mcopy != mesh));

    std::cout << "is mesh valid: " << mesh.is_valid() << "\n";

    session.add_mesh(std::make_shared<Mesh>(mesh));

    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/mesh.pb");

    return 0;
}
