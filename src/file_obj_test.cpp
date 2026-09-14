#include "mini_test.h"
#include "file_obj.h"
#include "mesh.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("FileObj", "Read Bunny") {
    if (!std::filesystem::exists("session_data/bunny.obj"))
        return;
    Mesh mesh = file_obj::read_file_obj("session_data/bunny.obj");

    MINI_CHECK(mesh.number_of_vertices() == 2503);
    MINI_CHECK(mesh.number_of_faces() == 4968);
    auto [vertices, faces] = mesh.to_vertices_and_faces();
    MINI_CHECK(vertices.size() == 2503);
    MINI_CHECK(faces.size() == 4968);
    bool has_non_zero = false;
    for (const Point& v : vertices)
        if (v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0)
            has_non_zero = true;
    MINI_CHECK(has_non_zero);
    bool all_polygons = true;
    for (const std::vector<size_t>& f : faces)
        if (f.size() < 3)
            all_polygons = false;
    MINI_CHECK(all_polygons);
}

MINI_TEST("FileObj", "Write Read Roundtrip") {
    std::filesystem::create_directories("./serialization");
    Mesh original_mesh;
    const size_t v0 = original_mesh.add_vertex(Point(0.0, 0.0, 0.0));
    const size_t v1 = original_mesh.add_vertex(Point(1.0, 0.0, 0.0));
    const size_t v2 = original_mesh.add_vertex(Point(0.0, 1.0, 0.0));
    const size_t v3 = original_mesh.add_vertex(Point(0.0, 0.0, 1.0));
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

MINI_TEST("FileObj", "String Roundtrip") {
    Mesh original_mesh;
    const size_t v0 = original_mesh.add_vertex(Point(0.0, 0.0, 0.0));
    const size_t v1 = original_mesh.add_vertex(Point(1.0, 0.0, 0.0));
    const size_t v2 = original_mesh.add_vertex(Point(0.0, 1.0, 0.0));
    const size_t v3 = original_mesh.add_vertex(Point(0.0, 0.0, 1.0));
    original_mesh.add_face({v0, v1, v2});
    original_mesh.add_face({v0, v1, v3});
    std::string s = file_obj::write_file_obj_to_string(original_mesh);
    Mesh loaded_mesh = file_obj::read_file_obj_from_str(s);

    MINI_CHECK(loaded_mesh.number_of_vertices() == original_mesh.number_of_vertices());
    MINI_CHECK(loaded_mesh.number_of_faces() == original_mesh.number_of_faces());
    MINI_CHECK(TOLERANCE.is_close(loaded_mesh.area(), original_mesh.area()));
}

} // namespace session_cpp
