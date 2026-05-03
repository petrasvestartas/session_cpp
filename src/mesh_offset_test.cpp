#include "mini_test.h"
#include "mesh_offset.h"
#include "mesh.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("MeshOffset", "from_mesh") {
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0, 1, 2, 3}});
        Mesh result = MeshOffset::from_mesh(mesh, 1.0);
        Mesh copy = result;
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result == copy);
        MINI_CHECK(!(result != copy));
        MINI_CHECK(result.number_of_vertices() == 8);
        MINI_CHECK(result.number_of_faces() == 6);
    }

    MINI_TEST("MeshOffset", "from_mesh_grid") {
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(2, 0, 0),
            Point(0, 1, 0),
            Point(1, 1, 0),
            Point(2, 1, 0),
            Point(0, 2, 0),
            Point(1, 2, 0),
            Point(2, 2, 0),
        };
        std::vector<std::vector<size_t>> faces = {
            {0, 1, 4, 3},
            {1, 2, 5, 4},
            {3, 4, 7, 6},
            {4, 5, 8, 7},
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, faces);
        Mesh result = MeshOffset::from_mesh(mesh, 2.0);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.number_of_vertices() == 18);
        MINI_CHECK(result.number_of_faces() == 16);
    }

    MINI_TEST("MeshOffset", "from_mesh_layers") {
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0, 1, 2, 3}});
        auto layers = MeshOffset::from_mesh_layers(mesh, 1.0);
        MINI_CHECK(layers.bottom.is_valid());
        MINI_CHECK(layers.top.is_valid());
        MINI_CHECK(layers.sides.is_valid());
        MINI_CHECK(layers.bottom.number_of_vertices() == 4);
        MINI_CHECK(layers.bottom.number_of_faces() == 1);
        MINI_CHECK(layers.top.number_of_vertices() == 4);
        MINI_CHECK(layers.top.number_of_faces() == 1);
        MINI_CHECK(layers.sides.number_of_faces() == 4);
    }

    MINI_TEST("MeshOffset", "file_json_dump") {
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0, 1, 2, 3}});
        Mesh result = MeshOffset::from_mesh(mesh, 1.0);
        result.file_json_dump("mesh_offset_test_dump.json");
        Mesh loaded = Mesh::file_json_load("mesh_offset_test_dump.json");
        MINI_CHECK(loaded.is_valid());
        MINI_CHECK(loaded.number_of_vertices() == result.number_of_vertices());
        MINI_CHECK(loaded.number_of_faces() == result.number_of_faces());
    }

    MINI_TEST("MeshOffset", "file_json_load") {
        Mesh loaded = Mesh::file_json_load("mesh_offset_test_dump.json");
        MINI_CHECK(loaded.is_valid());
        MINI_CHECK(loaded.number_of_vertices() == 8);
        MINI_CHECK(loaded.number_of_faces() == 6);
    }

    MINI_TEST("MeshOffset", "to_proto") {
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0, 1, 2, 3}});
        Mesh result = MeshOffset::from_mesh(mesh, 1.0);
        result.pb_dump("mesh_offset_test.pb");
        Mesh loaded = Mesh::pb_load("mesh_offset_test.pb");
        MINI_CHECK(loaded.is_valid());
        MINI_CHECK(loaded.number_of_vertices() == result.number_of_vertices());
        MINI_CHECK(loaded.number_of_faces() == result.number_of_faces());
    }

    MINI_TEST("MeshOffset", "from_proto") {
        Mesh loaded = Mesh::pb_load("mesh_offset_test.pb");
        MINI_CHECK(loaded.is_valid());
        MINI_CHECK(loaded.number_of_vertices() == 8);
        MINI_CHECK(loaded.number_of_faces() == 6);
    }

} // namespace session_cpp
