#include "mini_test.h"
#include "obj.h"
#include "mesh.h"
#include "tolerance.h"
#include <fstream>
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("OBJ", "Read_bunny") {
    std::ifstream test_file("../session_data/bunny.obj");
    if (!test_file.good()) {
        // Data file not found, skip test
        return;
    }
    test_file.close();

    Mesh mesh = obj::read_obj("../session_data/bunny.obj");

    MINI_CHECK(mesh.number_of_vertices() == 2503);
    MINI_CHECK(mesh.number_of_faces() == 4968);

    auto [vertices, faces] = mesh.to_vertices_and_faces();
    MINI_CHECK(vertices.size() == 2503);
    MINI_CHECK(faces.size() == 4968);

    bool has_non_zero = false;
    for (const auto& v : vertices) {
        if (v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0) {
            has_non_zero = true;
            break;
        }
    }
    MINI_CHECK(has_non_zero);

    for (const auto& face : faces) {
        MINI_CHECK(face.size() >= 3);
    }
}

MINI_TEST("OBJ", "Write_read_roundtrip") {
    std::filesystem::create_directories("./serialization");
    Mesh original_mesh;
    auto v0 = original_mesh.add_vertex(Point(0.0, 0.0, 0.0));
    auto v1 = original_mesh.add_vertex(Point(1.0, 0.0, 0.0));
    auto v2 = original_mesh.add_vertex(Point(0.0, 1.0, 0.0));
    auto v3 = original_mesh.add_vertex(Point(0.0, 0.0, 1.0));

    original_mesh.add_face({v0, v1, v2});
    original_mesh.add_face({v0, v1, v3});

    MINI_CHECK(original_mesh.number_of_vertices() == 4);
    MINI_CHECK(original_mesh.number_of_faces() == 2);

    std::string temp_file = "./serialization/test_temp_roundtrip.obj";
    obj::write_obj(original_mesh, temp_file);

    std::ifstream check(temp_file);
    MINI_CHECK(check.good());
    check.close();

    Mesh loaded_mesh = obj::read_obj(temp_file);

    MINI_CHECK(loaded_mesh.number_of_vertices() == original_mesh.number_of_vertices());
    MINI_CHECK(loaded_mesh.number_of_faces() == original_mesh.number_of_faces());

    std::remove(temp_file.c_str());
}

} // namespace session_cpp
