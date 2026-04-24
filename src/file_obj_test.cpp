#include "mini_test.h"
#include "file_obj.h"
#include "mesh.h"
#include <algorithm>
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("FileObj", "Read Bunny") {
    // load Stanford Bunny (real-world OBJ: 2503 vertices, 4968 faces)
    if (!std::filesystem::exists("session_data/bunny.obj"))
        return;

    Mesh mesh = file_obj::read_file_obj("session_data/bunny.obj");

    MINI_CHECK(mesh.number_of_vertices() == 2503);
    MINI_CHECK(mesh.number_of_faces() == 4968);

    auto [vertices, faces] = mesh.to_vertices_and_faces();
    MINI_CHECK(vertices.size() == 2503);
    MINI_CHECK(faces.size() == 4968);

    bool has_non_zero = std::any_of(vertices.begin(), vertices.end(),
        [](const auto& v) { return v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0; });
    MINI_CHECK(has_non_zero);

    MINI_CHECK(std::all_of(faces.begin(), faces.end(),
        [](const auto& f) { return f.size() >= 3; }));
}

MINI_TEST("FileObj", "Write Read Roundtrip") {
    // build a small mesh (4 verts, 2 faces), write to OBJ, read back, compare counts
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
    file_obj::write_file_obj(original_mesh, temp_file);

    MINI_CHECK(std::filesystem::exists(temp_file));

    Mesh loaded_mesh = file_obj::read_file_obj(temp_file);

    MINI_CHECK(loaded_mesh.number_of_vertices() == original_mesh.number_of_vertices());
    MINI_CHECK(loaded_mesh.number_of_faces() == original_mesh.number_of_faces());

    std::filesystem::remove(temp_file);
}

} // namespace session_cpp
