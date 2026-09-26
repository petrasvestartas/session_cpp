#include "mini_test.h"
#include "mesh_offset.h"
#include "mesh.h"
#include "plane.h"
#include "point.h"
#include "tolerance.h"
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("MeshOffset", "From Mesh") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;

    const std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, {{0, 1, 2, 3}});
    const Mesh result = MeshOffset::from_mesh(mesh, 1.0);
    const Mesh copy = result;

    MINI_CHECK(result.is_valid());
    MINI_CHECK(result.is_closed());
    MINI_CHECK(result == copy);
    MINI_CHECK(!(result != copy));
    MINI_CHECK(result.number_of_vertices() == 8);
    MINI_CHECK(result.number_of_faces() == 6);
}

MINI_TEST("MeshOffset", "From Mesh Grid") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;

    const std::vector<Point> points = {
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
    const std::vector<std::vector<size_t>> faces = {
        {0, 1, 4, 3},
        {1, 2, 5, 4},
        {3, 4, 7, 6},
        {4, 5, 8, 7},
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, faces);
    const Mesh result = MeshOffset::from_mesh(mesh, 2.0);

    MINI_CHECK(result.is_valid());
    MINI_CHECK(result.is_closed());
    MINI_CHECK(result.number_of_vertices() == 18);
    MINI_CHECK(result.number_of_faces() == 16);
}

MINI_TEST("MeshOffset", "From Mesh Layers") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;

    const std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, {{0, 1, 2, 3}});
    const MeshOffset::Layers layers = MeshOffset::from_mesh_layers(mesh, 1.0);

    MINI_CHECK(layers.bottom.is_valid());
    MINI_CHECK(layers.top.is_valid());
    MINI_CHECK(layers.sides.is_valid());
    MINI_CHECK(layers.bottom.number_of_vertices() == 4);
    MINI_CHECK(layers.bottom.number_of_faces() == 1);
    MINI_CHECK(layers.top.number_of_vertices() == 4);
    MINI_CHECK(layers.top.number_of_faces() == 1);
    MINI_CHECK(layers.sides.number_of_faces() == 4);
}

MINI_TEST("MeshOffset", "Offset Planes") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Plane;

    const std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, {{0, 1, 2, 3}});
    const std::map<size_t, Plane> planes = MeshOffset::offset_planes(mesh, 1.0);

    MINI_CHECK(planes.size() == 1);

    const Plane& plane = planes.at(0);

    MINI_CHECK(TOLERANCE.is_close(plane.a(), 0.0));
    MINI_CHECK(TOLERANCE.is_close(plane.b(), 0.0));
    MINI_CHECK(TOLERANCE.is_close(plane.c(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(plane.d(), -1.0));
    MINI_CHECK(TOLERANCE.is_close(plane.origin()[2], 1.0));
}

MINI_TEST("MeshOffset", "Offset Vertices") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Plane;

    const std::vector<Point> points = {
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
    const std::vector<std::vector<size_t>> faces = {
        {0, 1, 4, 3},
        {1, 2, 5, 4},
        {3, 4, 7, 6},
        {4, 5, 8, 7},
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, faces);
    const std::map<size_t, Plane> planes = MeshOffset::offset_planes(mesh, 2.0);
    const std::map<size_t, Point> offsets = MeshOffset::offset_vertices(mesh, planes);

    MINI_CHECK(planes.size() == 4);
    MINI_CHECK(offsets.size() == 9);

    for (size_t vkey = 0; vkey < 9; ++vkey) {
        MINI_CHECK(TOLERANCE.is_close(offsets.at(vkey)[0], points[vkey][0]));
        MINI_CHECK(TOLERANCE.is_close(offsets.at(vkey)[1], points[vkey][1]));
        MINI_CHECK(TOLERANCE.is_close(offsets.at(vkey)[2], 2.0));
    }
}

MINI_TEST("MeshOffset", "Json Roundtrip") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;

    const std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, {{0, 1, 2, 3}});
    const Mesh result = MeshOffset::from_mesh(mesh, 1.0);
    const std::string filename =
        (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh_offset.json")
            .string();

    result.file_json_dump(filename);
    const Mesh loaded = Mesh::file_json_load(filename);

    MINI_CHECK(loaded == result);
    MINI_CHECK(loaded.number_of_vertices() == 8);
    MINI_CHECK(loaded.number_of_faces() == 6);
}

MINI_TEST("MeshOffset", "Protobuf Roundtrip") {
    // using session_cpp::MeshOffset;
    // using session_cpp::Mesh;
    // using session_cpp::Point;

    const std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    const Mesh mesh = Mesh::from_vertices_and_faces(points, {{0, 1, 2, 3}});
    const Mesh result = MeshOffset::from_mesh(mesh, 1.0);
    const std::string filename =
        (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh_offset.bin")
            .string();

    result.pb_dump(filename);
    const Mesh loaded = Mesh::pb_load(filename);

    MINI_CHECK(loaded == result);
    MINI_CHECK(loaded.number_of_vertices() == 8);
    MINI_CHECK(loaded.number_of_faces() == 6);
}

} // namespace session_cpp
