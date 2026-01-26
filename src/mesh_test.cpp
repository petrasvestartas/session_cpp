#include "mini_test.h"
#include "mesh.h"
#include "point.h"
#include "xform.h"
#include "tolerance.h"
#include "encoders.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Mesh", "constructor") {
        // uncomment #include "mesh.h"

        Mesh mesh;

        size_t num_vertices = mesh.number_of_vertices();
        size_t num_faces = mesh.number_of_faces();
        size_t num_edges = mesh.number_of_edges();
        bool is_empty = mesh.is_empty();
        int euler = mesh.euler();

        MINI_CHECK(num_vertices == 0);
        MINI_CHECK(num_faces == 0);
        MINI_CHECK(num_edges == 0);
        MINI_CHECK(is_empty);
        MINI_CHECK(euler == 0);
        MINI_CHECK(mesh.name == "my_mesh");
        MINI_CHECK(!mesh.guid.empty());
    }

    MINI_TEST("Mesh", "add_vertex") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(1.0, 2.0, 3.0), std::nullopt);

        MINI_CHECK(mesh.number_of_vertices() == 1);
        MINI_CHECK(!mesh.is_empty());

        auto pos = mesh.vertex_position(v0);
        MINI_CHECK(pos.has_value());
        MINI_CHECK((*pos)[0] == 1.0 && (*pos)[1] == 2.0 && (*pos)[2] == 3.0);

        size_t v1 = mesh.add_vertex(Point(4.0, 5.0, 6.0), 42);
        MINI_CHECK(v1 == 42);
        MINI_CHECK(mesh.number_of_vertices() == 2);
    }

    MINI_TEST("Mesh", "add_face") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        auto f = mesh.add_face({v0, v1, v2}, std::nullopt);
        MINI_CHECK(f.has_value());
        MINI_CHECK(mesh.number_of_faces() == 1);
        MINI_CHECK(mesh.number_of_edges() == 3);
        MINI_CHECK(mesh.euler() == 1);

        auto invalid1 = mesh.add_face({v0, v1}, std::nullopt);
        MINI_CHECK(!invalid1.has_value());

        auto invalid2 = mesh.add_face({v0, v1, 999}, std::nullopt);
        MINI_CHECK(!invalid2.has_value());

        auto invalid3 = mesh.add_face({v0, v1, v0}, std::nullopt);
        MINI_CHECK(!invalid3.has_value());
    }

    MINI_TEST("Mesh", "face_vertices") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        auto f = mesh.add_face({v0, v1, v2}, std::nullopt);
        auto vertices = mesh.face_vertices(*f);

        MINI_CHECK(vertices.has_value());
        MINI_CHECK(vertices->size() == 3);
        MINI_CHECK((*vertices)[0] == v0);
        MINI_CHECK((*vertices)[1] == v1);
        MINI_CHECK((*vertices)[2] == v2);
    }

    MINI_TEST("Mesh", "vertex_neighbors") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        mesh.add_face({v0, v1, v2}, std::nullopt);

        auto neighbors = mesh.vertex_neighbors(v0);
        MINI_CHECK(neighbors.size() == 2);
        MINI_CHECK(std::find(neighbors.begin(), neighbors.end(), v1) != neighbors.end());
        MINI_CHECK(std::find(neighbors.begin(), neighbors.end(), v2) != neighbors.end());
    }

    MINI_TEST("Mesh", "vertex_faces") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        size_t v3 = mesh.add_vertex(Point(1.0, 1.0, 0.0), std::nullopt);

        auto f1 = mesh.add_face({v0, v1, v2}, std::nullopt);
        auto f2 = mesh.add_face({v1, v3, v2}, std::nullopt);

        auto faces = mesh.vertex_faces(v1);
        MINI_CHECK(faces.size() == 2);
        MINI_CHECK(std::find(faces.begin(), faces.end(), *f1) != faces.end());
        MINI_CHECK(std::find(faces.begin(), faces.end(), *f2) != faces.end());
    }

    MINI_TEST("Mesh", "is_vertex_on_boundary") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        mesh.add_face({v0, v1, v2}, std::nullopt);

        MINI_CHECK(mesh.is_vertex_on_boundary(v0));
        MINI_CHECK(mesh.is_vertex_on_boundary(v1));
        MINI_CHECK(mesh.is_vertex_on_boundary(v2));
    }

    MINI_TEST("Mesh", "face_normal") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"
        // uncomment #include "tolerance.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        auto f = mesh.add_face({v0, v1, v2}, std::nullopt);
        auto normal = mesh.face_normal(*f);

        MINI_CHECK(normal.has_value());
        MINI_CHECK(TOLERANCE.is_close((*normal)[2], 1.0));
        MINI_CHECK(TOLERANCE.is_close((*normal)[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close((*normal)[1], 0.0));
    }

    MINI_TEST("Mesh", "face_area") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"
        // uncomment #include "tolerance.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        auto f = mesh.add_face({v0, v1, v2}, std::nullopt);
        auto area = mesh.face_area(*f);

        MINI_CHECK(area.has_value());
        MINI_CHECK(TOLERANCE.is_close(*area, 0.5));
    }

    MINI_TEST("Mesh", "from_polygons") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        std::vector<Point> triangle = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
        };

        Mesh mesh = Mesh::from_polygons({triangle}, std::nullopt);
        MINI_CHECK(mesh.number_of_vertices() == 3);
        MINI_CHECK(mesh.number_of_faces() == 1);
        MINI_CHECK(mesh.number_of_edges() == 3);

        std::vector<Point> tri1 = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
        };
        std::vector<Point> tri2 = {
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(1.0, 1.0, 0.0),
        };

        Mesh mesh2 = Mesh::from_polygons({tri1, tri2}, std::nullopt);
        MINI_CHECK(mesh2.number_of_vertices() == 4);
        MINI_CHECK(mesh2.number_of_faces() == 2);
    }

    MINI_TEST("Mesh", "clear") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

        MINI_CHECK(!mesh.is_empty());

        mesh.clear();

        MINI_CHECK(mesh.is_empty());
        MINI_CHECK(mesh.number_of_vertices() == 0);
        MINI_CHECK(mesh.number_of_faces() == 0);
    }

    MINI_TEST("Mesh", "transformation") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

        mesh.xform = Xform::translation(10.0, 20.0, 30.0);
        Mesh mesh_transformed = mesh.transformed();
        mesh.transform();

        auto pos0 = mesh.vertex_position(v0);
        MINI_CHECK(pos0.has_value());
        MINI_CHECK((*pos0)[0] == 10.0);
        MINI_CHECK((*pos0)[1] == 20.0);
        MINI_CHECK((*pos0)[2] == 30.0);
        MINI_CHECK(mesh.xform == Xform::identity());
        MINI_CHECK(mesh_transformed.xform == Xform::identity());
    }

    MINI_TEST("Mesh", "json_roundtrip") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        mesh.name = "test_mesh";
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

        std::string filename = "serialization/test_mesh.json";
        encoders::json_dump(mesh, filename);
        Mesh loaded = encoders::json_load<Mesh>(filename);

        MINI_CHECK(loaded.name == mesh.name);
        MINI_CHECK(loaded.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded.number_of_faces() == mesh.number_of_faces());
    }

    MINI_TEST("Mesh", "protobuf_roundtrip") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        mesh.name = "test_mesh_proto";
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

        std::string filename = "serialization/test_mesh.bin";
        mesh.protobuf_dump(filename);
        Mesh loaded = Mesh::protobuf_load(filename);

        MINI_CHECK(loaded.name == mesh.name);
        MINI_CHECK(loaded.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK(loaded.guid == mesh.guid);
    }

    MINI_TEST("Mesh", "vertex_position") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(1.0, 2.0, 3.0), std::nullopt);

        auto pos = mesh.vertex_position(v0);
        MINI_CHECK(pos.has_value());
        MINI_CHECK((*pos)[0] == 1.0);
        MINI_CHECK((*pos)[1] == 2.0);
        MINI_CHECK((*pos)[2] == 3.0);
        MINI_CHECK(!mesh.vertex_position(999).has_value());
    }

    MINI_TEST("Mesh", "vertex_normal") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        size_t v3 = mesh.add_vertex(Point(1.0, 1.0, 0.0), std::nullopt);

        mesh.add_face({v0, v1, v3}, std::nullopt);
        mesh.add_face({v0, v3, v2}, std::nullopt);

        auto normal = mesh.vertex_normal(v0);
        MINI_CHECK(normal.has_value());
        MINI_CHECK(std::abs((*normal)[2]) == 1.0);
    }

    MINI_TEST("Mesh", "to_vertices_and_faces") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);

        mesh.add_face({v0, v1, v2}, std::nullopt);

        auto [vertices, faces] = mesh.to_vertices_and_faces();

        MINI_CHECK(vertices.size() == 3);
        MINI_CHECK(faces.size() == 1);
        MINI_CHECK(faces[0].size() == 3);
    }

} // namespace session_cpp
