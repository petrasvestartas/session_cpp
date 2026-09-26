#include "mini_test.h"
#include "file_obj.h"
#include "mesh.h"
#include "tolerance.h"
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("FileObj", "Read Bunny") {
        // using session_cpp::file_obj::read_file_obj;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const std::filesystem::path bunny_path = std::filesystem::path(__FILE__).parent_path().parent_path() /
            "session_data" / "bunny.obj";

        MINI_CHECK(std::filesystem::exists(bunny_path));

        const Mesh mesh = file_obj::read_file_obj(bunny_path.string());
        const std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> indexed = mesh.to_vertices_and_faces();
        const std::vector<Point>& vertices = indexed.first;
        const std::vector<std::vector<size_t>>& faces = indexed.second;
        bool has_non_zero = false;

        for (const Point& v : vertices)
            if (v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0)
                has_non_zero = true;

        bool all_polygons = true;

        for (const std::vector<size_t>& f : faces)
            if (f.size() < 3)
                all_polygons = false;

        MINI_CHECK(mesh.number_of_vertices() == 2503);
        MINI_CHECK(mesh.number_of_faces() == 4968);
        MINI_CHECK(vertices.size() == 2503);
        MINI_CHECK(faces.size() == 4968);
        MINI_CHECK(has_non_zero);
        MINI_CHECK(all_polygons);
    }

    MINI_TEST("FileObj", "Write Read Roundtrip") {
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::file_obj::read_file_obj;
        // using session_cpp::file_obj::write_file_obj;

        std::filesystem::create_directories("./serialization");

        Mesh original;
        const size_t v0 = original.add_vertex(Point(0.0, 0.0, 0.0));
        const size_t v1 = original.add_vertex(Point(1.0, 0.0, 0.0));
        const size_t v2 = original.add_vertex(Point(0.0, 1.0, 0.0));
        const size_t v3 = original.add_vertex(Point(0.0, 0.0, 1.0));
        original.add_face({v0, v1, v2});
        original.add_face({v0, v1, v3});

        const std::string filepath = "./serialization/test_temp_roundtrip.obj";
        file_obj::write_file_obj(original, filepath);
        const bool exists = std::filesystem::exists(filepath);
        const Mesh loaded = file_obj::read_file_obj(filepath);

        MINI_CHECK(original.number_of_vertices() == 4);
        MINI_CHECK(original.number_of_faces() == 2);
        MINI_CHECK(exists);
        MINI_CHECK(loaded.number_of_vertices() == original.number_of_vertices());
        MINI_CHECK(loaded.number_of_faces() == original.number_of_faces());

        std::filesystem::remove(filepath);
    }

    MINI_TEST("FileObj", "String Roundtrip") {
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::file_obj::read_file_obj_from_str;
        // using session_cpp::file_obj::write_file_obj_to_string;

        Mesh original;
        const size_t v0 = original.add_vertex(Point(0.0, 0.0, 0.0));
        const size_t v1 = original.add_vertex(Point(1.0, 0.0, 0.0));
        const size_t v2 = original.add_vertex(Point(0.0, 1.0, 0.0));
        const size_t v3 = original.add_vertex(Point(0.0, 0.0, 1.0));
        original.add_face({v0, v1, v2});
        original.add_face({v0, v1, v3});

        const std::string content = file_obj::write_file_obj_to_string(original);
        const Mesh loaded = file_obj::read_file_obj_from_str(content);

        MINI_CHECK(loaded.number_of_vertices() == original.number_of_vertices());
        MINI_CHECK(loaded.number_of_faces() == original.number_of_faces());
        MINI_CHECK(TOLERANCE.is_close(loaded.area(), original.area()));
    }

} // namespace session_cpp
