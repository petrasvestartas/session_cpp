#include "mini_test.h"
#include "session.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>
#include <fstream>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Session", "Constructor") {
    Session session;
    MINI_CHECK(session.name == "my_session");
    MINI_CHECK(!session.guid.empty());
}

MINI_TEST("Session", "Jsondump") {
    Session session;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(point1);
    session.add_point(point2);
    session.add_edge(point1->guid, point2->guid, "connection");

    auto data = session.jsondump();
    MINI_CHECK(data["name"] == "my_session");
    MINI_CHECK(data.contains("guid"));
    MINI_CHECK(data["objects"]["points"].size() == 2);
    MINI_CHECK(data["graph"]["vertices"].size() == 2);
    MINI_CHECK(data["graph"]["edges"].size() == 1);

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(session, "./serialization/test_session.json");
}

MINI_TEST("Session", "Jsonload") {
    Session session;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(point1);
    session.add_point(point2);
    session.add_edge(point1->guid, point2->guid, "connection");

    auto data = session.jsondump();
    Session session2 = Session::jsonload(data);
    MINI_CHECK(session2.name == "my_session");
    MINI_CHECK(session2.lookup.size() == 2);
    MINI_CHECK(session2.graph.number_of_vertices() == 2);
}

MINI_TEST("Session", "File_io") {
    Session session;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(point1);
    session.add_point(point2);
    session.add_edge(point1->guid, point2->guid, "connection");
    std::string filename = "./serialization/test_session_roundtrip.json";

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(session, filename);
    Session loaded_session = encoders::json_load<Session>(filename);

    MINI_CHECK(loaded_session.name == session.name);
    MINI_CHECK(loaded_session.lookup.size() == session.lookup.size());
    MINI_CHECK(loaded_session.graph.number_of_vertices() == session.graph.number_of_vertices());

    std::filesystem::remove(filename);
}

MINI_TEST("Session", "Add_point") {
    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.lookup.count(point->guid) == 1);
    MINI_CHECK(session.graph.has_node(point->guid));
}

MINI_TEST("Session", "Add_edge") {
    Session session;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(point1);
    session.add_point(point2);
    session.add_edge(point1->guid, point2->guid, "connection");

    MINI_CHECK(session.graph.has_edge({point1->guid, point2->guid}));
}

MINI_TEST("Session", "Get_object") {
    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    auto retrieved = session.get_object<Point>(point->guid);
    MINI_CHECK(retrieved != nullptr);
    MINI_CHECK(retrieved->guid == point->guid);
}

MINI_TEST("Session", "File_io_comprehensive") {
    Session session("./serialization/test_session");
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(point1);
    session.add_point(point2);
    session.add_edge(point1->guid, point2->guid, "./serialization/test_connection");
    std::string filename = "./serialization/test_session_comprehensive.json";

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(session, filename);
    Session loaded_session = encoders::json_load<Session>(filename);

    MINI_CHECK(loaded_session.name == session.name);
    MINI_CHECK(loaded_session.objects.points->size() == session.objects.points->size());
    MINI_CHECK(loaded_session.graph.number_of_vertices() == session.graph.number_of_vertices());
    MINI_CHECK(loaded_session.graph.number_of_edges() == session.graph.number_of_edges());

    std::filesystem::remove(filename);
}

MINI_TEST("Session", "Tree_transformation_hierarchy") {
    Session scene("tree_transformation_test");

    auto create_box = [](const Point& center, double size) -> std::shared_ptr<Mesh> {
        auto mesh = std::make_shared<Mesh>();
        double h = size * 0.5;
        std::vector<Point> verts = {
            Point(center[0] - h, center[1] - h, center[2] - h),
            Point(center[0] + h, center[1] - h, center[2] - h),
            Point(center[0] + h, center[1] + h, center[2] - h),
            Point(center[0] - h, center[1] + h, center[2] - h),
            Point(center[0] - h, center[1] - h, center[2] + h),
            Point(center[0] + h, center[1] - h, center[2] + h),
            Point(center[0] + h, center[1] + h, center[2] + h),
            Point(center[0] - h, center[1] + h, center[2] + h)
        };
        for (size_t i = 0; i < verts.size(); ++i) mesh->add_vertex(verts[i], i);
        std::vector<std::vector<size_t>> faces = {
            {0,1,2,3}, {4,7,6,5}, {0,4,5,1}, {2,6,7,3}, {0,3,7,4}, {1,5,6,2}
        };
        for (const auto& f : faces) mesh->add_face(f);
        return mesh;
    };

    auto box1 = create_box(Point(0, 0, 0), 2.0);
    box1->name = "box_1";
    auto box1_node = scene.add_mesh(box1);

    auto box2 = create_box(Point(0, 0, 0), 2.0);
    box2->name = "box_2";
    auto box2_node = scene.add_mesh(box2);

    auto box3 = create_box(Point(0, 0, 0), 2.0);
    box3->name = "box_3";
    auto box3_node = scene.add_mesh(box3);

    scene.add(box1_node);
    scene.add(box2_node, box1_node);
    scene.add(box3_node, box2_node);

    Point box1_top(0, 0, 1.0);
    Vector normal(0, 0, 1), x(1, 0, 0), y(0, 1, 0);
    Point xy_origin(0, 0, 0);
    Vector xy_x(1, 0, 0), xy_y(0, 1, 0);

    Plane plane_from(xy_origin, xy_x, xy_y);
    Plane plane_to(box1_top, x, y);
    Xform xy_to_top = Xform::plane_to_plane(plane_from, plane_to);
    box1->xform = Xform::rotation_z(Tolerance::PI / 1.5) * xy_to_top;

    box2->xform = Xform::translation(2.0, 0, 0) * Xform::rotation_z(Tolerance::PI / 6.0);
    box3->xform = Xform::translation(2.0, 0, 0);

    Objects transformed = scene.get_geometry();

    MINI_CHECK(transformed.meshes->size() == 3);

    std::vector<std::array<double, 3>> expected_box1 = {
        {1.36603, -0.366025, 0}, {0.366025, 1.36603, 0}, {-1.36603, 0.366025, 0},
        {-0.366025, -1.36603, 0}, {1.36603, -0.366025, 2}, {0.366025, 1.36603, 2},
        {-1.36603, 0.366025, 2}, {-0.366025, -1.36603, 2}
    };

    std::vector<std::array<double, 3>> expected_box2 = {
        {0.366025, 2.09808, 0}, {-1.36603, 3.09808, 0}, {-2.36603, 1.36603, 0},
        {-0.633975, 0.366025, 0}, {0.366025, 2.09808, 2}, {-1.36603, 3.09808, 2},
        {-2.36603, 1.36603, 2}, {-0.633975, 0.366025, 2}
    };

    std::vector<std::array<double, 3>> expected_box3 = {
        {-1.36603, 3.09808, 0}, {-3.09808, 4.09808, 0}, {-4.09808, 2.36603, 0},
        {-2.36603, 1.36603, 0}, {-1.36603, 3.09808, 2}, {-3.09808, 4.09808, 2},
        {-4.09808, 2.36603, 2}, {-2.36603, 1.36603, 2}
    };

    std::vector<std::vector<size_t>> expected_faces = {
        {0,1,2,3}, {4,7,6,5}, {0,4,5,1}, {2,6,7,3}, {0,3,7,4}, {1,5,6,2}
    };

    auto& m1 = (*transformed.meshes)[0];
    MINI_CHECK(m1->vertex.size() == 8);
    for (size_t i = 0; i < 8; ++i) {
        const auto& v = m1->vertex.at(i);
        MINI_CHECK(std::abs(v.x - expected_box1[i][0]) < 1e-4);
        MINI_CHECK(std::abs(v.y - expected_box1[i][1]) < 1e-4);
        MINI_CHECK(std::abs(v.z - expected_box1[i][2]) < 1e-4);
    }

    auto& m2 = (*transformed.meshes)[1];
    MINI_CHECK(m2->vertex.size() == 8);
    for (size_t i = 0; i < 8; ++i) {
        const auto& v = m2->vertex.at(i);
        MINI_CHECK(std::abs(v.x - expected_box2[i][0]) < 1e-4);
        MINI_CHECK(std::abs(v.y - expected_box2[i][1]) < 1e-4);
        MINI_CHECK(std::abs(v.z - expected_box2[i][2]) < 1e-4);
    }

    auto& m3 = (*transformed.meshes)[2];
    MINI_CHECK(m3->vertex.size() == 8);
    for (size_t i = 0; i < 8; ++i) {
        const auto& v = m3->vertex.at(i);
        MINI_CHECK(std::abs(v.x - expected_box3[i][0]) < 1e-4);
        MINI_CHECK(std::abs(v.y - expected_box3[i][1]) < 1e-4);
        MINI_CHECK(std::abs(v.z - expected_box3[i][2]) < 1e-4);
    }

    for (auto* mesh : {&m1, &m2, &m3}) {
        MINI_CHECK((*mesh)->face.size() == 6);
        size_t face_idx = 0;
        for (const auto& [key, face] : (*mesh)->face) {
            MINI_CHECK(face.size() == expected_faces[face_idx].size());
            for (size_t i = 0; i < face.size(); ++i) {
                MINI_CHECK(face[i] == expected_faces[face_idx][i]);
            }
            face_idx++;
        }
    }
}

} // namespace session_cpp
