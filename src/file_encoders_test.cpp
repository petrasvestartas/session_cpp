#include "mini_test.h"
#include "file_encoders.h"
#include "point.h"
#include "vector.h"
#include "line.h"
#include "mesh.h"
#include "instance_ref.h"
#include "xform.h"
#include "polyline.h"
#include "element.h"
#include "objects.h"
#include "brep.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "tolerance.h"
#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    using namespace session_cpp::file_encoders;

    MINI_TEST("FileEncoders", "Json Dump Load") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_json_dump;
        // using session_cpp::file_encoders::file_json_load;

        Point original(1.5, 2.5, 3.5);
        original.name = "test_point";

        const std::string filepath = "serialization/test_encoders_point.json";
        file_json_dump(original, filepath);

        const Point loaded = file_json_load<Point>(filepath);

        MINI_CHECK(TOLERANCE.is_close(loaded[0], original[0]));
        MINI_CHECK(TOLERANCE.is_close(loaded[1], original[1]));
        MINI_CHECK(TOLERANCE.is_close(loaded[2], original[2]));
        MINI_CHECK(loaded.name == original.name);

        std::filesystem::remove(filepath);
    }

    MINI_TEST("FileEncoders", "Json Dumps Loads") {
        // using session_cpp::Vector;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        Vector original(42.1, 84.2, 126.3);
        original.name = "test_vector";

        const std::string json_str = file_json_dumps(original);

        MINI_CHECK(!json_str.empty());
        MINI_CHECK(json_str.find("Vector") != std::string::npos);

        const Vector loaded = file_json_loads<Vector>(json_str);

        MINI_CHECK(TOLERANCE.is_close(loaded[0], original[0]));
        MINI_CHECK(TOLERANCE.is_close(loaded[1], original[1]));
        MINI_CHECK(TOLERANCE.is_close(loaded[2], original[2]));
        MINI_CHECK(loaded.name == original.name);
    }

    MINI_TEST("FileEncoders", "Encode Collection Values") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_encode_collection;

        std::vector<Point> points;
        points.push_back(Point(1.0, 2.0, 3.0));
        points.push_back(Point(4.0, 5.0, 6.0));
        points.push_back(Point(7.0, 8.0, 9.0));

        const nlohmann::ordered_json json_arr = file_encode_collection(points);

        MINI_CHECK(json_arr.is_array());
        MINI_CHECK(json_arr.size() == 3);
        MINI_CHECK(json_arr[0]["type"] == "Point");
        MINI_CHECK(json_arr[1]["x"] == 4.0);
        MINI_CHECK(json_arr[2]["z"] == 9.0);
    }

    MINI_TEST("FileEncoders", "Encode Collection Shared Ptr") {
        // using session_cpp::Line;
        // using session_cpp::file_encoders::file_encode_collection;

        std::vector<std::shared_ptr<Line>> lines;
        lines.push_back(std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
        lines.push_back(std::make_shared<Line>(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));

        const nlohmann::ordered_json json_arr = file_encode_collection(lines);

        MINI_CHECK(json_arr.is_array());
        MINI_CHECK(json_arr.size() == 2);
        MINI_CHECK(json_arr[0]["type"] == "Line");
        MINI_CHECK(json_arr[1]["type"] == "Line");
    }

    MINI_TEST("FileEncoders", "Decode Collection") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_encode_collection;
        // using session_cpp::file_encoders::file_decode_collection;

        std::vector<Point> original_points;
        original_points.push_back(Point(1.0, 2.0, 3.0));
        original_points.push_back(Point(4.0, 5.0, 6.0));

        const nlohmann::ordered_json json_arr = file_encode_collection(original_points);
        const std::vector<Point> decoded_points = file_decode_collection<Point>(json_arr);

        MINI_CHECK(decoded_points.size() == 2);
        MINI_CHECK(TOLERANCE.is_close(decoded_points[0][0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(decoded_points[1][1], 5.0));
    }

    MINI_TEST("FileEncoders", "Decode Collection Ptr") {
        // using session_cpp::Vector;
        // using session_cpp::file_encoders::file_encode_collection;

        std::vector<std::shared_ptr<Vector>> original_vectors;
        original_vectors.push_back(std::make_shared<Vector>(1.0, 0.0, 0.0));
        original_vectors.push_back(std::make_shared<Vector>(0.0, 1.0, 0.0));

        const nlohmann::ordered_json json_arr = file_encode_collection(original_vectors);
        const std::vector<std::shared_ptr<Vector>> decoded_vectors = file_decode_collection_ptr<Vector>(json_arr);

        MINI_CHECK(decoded_vectors.size() == 2);
        MINI_CHECK(TOLERANCE.is_close((*decoded_vectors[0])[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close((*decoded_vectors[1])[1], 1.0));
    }

    MINI_TEST("FileEncoders", "Nested Collections") {
        // using session_cpp::Line;
        // using session_cpp::file_encoders::file_encode_collection;
        // using session_cpp::file_encoders::file_decode_collection;

        std::vector<Line> lines;
        lines.push_back(Line(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
        lines.push_back(Line(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));

        const nlohmann::ordered_json json_arr = file_encode_collection(lines);
        const std::string json_str = json_arr.dump();

        MINI_CHECK(!json_str.empty());

        const nlohmann::json loaded_json = nlohmann::json::parse(json_str);
        const std::vector<Line> loaded = file_decode_collection<Line>(loaded_json);

        MINI_CHECK(loaded.size() == 2);
        MINI_CHECK(TOLERANCE.is_close(loaded[0].end()[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded[1].end()[1], 1.0));
    }

    MINI_TEST("FileEncoders", "Roundtrip File Io") {
        // using session_cpp::Vector;
        // using session_cpp::file_encoders::file_encode_collection;
        // using session_cpp::file_encoders::file_decode_collection;
        // using session_cpp::file_encoders::file_json_dump;
        // using session_cpp::file_encoders::file_json_load_data;

        std::vector<Vector> vectors;
        vectors.push_back(Vector(1.0, 0.0, 0.0));
        vectors.push_back(Vector(0.0, 1.0, 0.0));
        vectors.push_back(Vector(0.0, 0.0, 1.0));

        const std::string filepath = "serialization/test_encoders_collection.json";
        const nlohmann::ordered_json json_arr = file_encode_collection(vectors);
        file_json_dump(json_arr, filepath);

        const nlohmann::ordered_json loaded_json = file_json_load_data(filepath);
        const std::vector<Vector> decoded_vectors = file_decode_collection<Vector>(loaded_json);

        MINI_CHECK(decoded_vectors.size() == 3);
        MINI_CHECK(TOLERANCE.is_close(decoded_vectors[0][0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(decoded_vectors[1][1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(decoded_vectors[2][2], 1.0));

        std::filesystem::remove(filepath);
    }

    MINI_TEST("FileEncoders", "Pretty Vs Compact") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        const Point point(1.0, 2.0, 3.0);

        const std::string pretty = file_json_dumps(point, true);
        const std::string compact = file_json_dumps(point, false);

        MINI_CHECK(pretty.length() > compact.length());
        MINI_CHECK(pretty.find("\n") != std::string::npos);
        MINI_CHECK(compact.find("\n") == std::string::npos);

        const Point loaded_pretty = file_json_loads<Point>(pretty);
        const Point loaded_compact = file_json_loads<Point>(compact);

        MINI_CHECK(TOLERANCE.is_close(loaded_pretty[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded_compact[0], 1.0));
    }

    MINI_TEST("FileEncoders", "Decode Primitives") {

        const nlohmann::json num = 42;
        std::string json_str = num.dump();
        nlohmann::json loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded.get<int>() == 42);

        const nlohmann::json float_val = 3.14;
        json_str = float_val.dump();
        loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(TOLERANCE.is_close(loaded.get<double>(), 3.14));

        const nlohmann::json text = "hello";
        json_str = text.dump();
        loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded.get<std::string>() == "hello");

        const nlohmann::json flag = true;
        json_str = flag.dump();
        loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded.get<bool>());
    }

    MINI_TEST("FileEncoders", "Decode List") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_encode_collection;
        // using session_cpp::file_encoders::file_decode_collection;

        const std::vector<int> data = {1, 2, 3};
        const nlohmann::json json = data;
        const std::string json_str = json.dump();
        const nlohmann::json loaded = nlohmann::json::parse(json_str);
        const std::vector<int> loaded_vec = loaded.get<std::vector<int>>();

        MINI_CHECK(loaded_vec.size() == 3);
        MINI_CHECK(loaded_vec[0] == 1);
        MINI_CHECK(loaded_vec[2] == 3);

        std::vector<Point> points;
        points.push_back(Point(1.0, 2.0, 3.0));
        points.push_back(Point(4.0, 5.0, 6.0));

        const nlohmann::ordered_json json_arr = file_encode_collection(points);
        const std::vector<Point> decoded = file_decode_collection<Point>(json_arr);

        MINI_CHECK(decoded.size() == 2);
        MINI_CHECK(TOLERANCE.is_close(decoded[0][0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(decoded[1][0], 4.0));
    }

    MINI_TEST("FileEncoders", "Decode Dict") {
        // using session_cpp::Vector;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        std::map<std::string, int> data;
        data["a"] = 1;
        data["b"] = 2;

        const nlohmann::json json = data;
        const std::string json_str = json.dump();
        const nlohmann::json loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded["a"].get<int>() == 1);
        MINI_CHECK(loaded["b"].get<int>() == 2);

        const Vector vec(1.0, 2.0, 3.0);
        const std::string vec_json = file_json_dumps(vec);
        const Vector loaded_vec = file_json_loads<Vector>(vec_json);

        MINI_CHECK(TOLERANCE.is_close(loaded_vec[0], 1.0));
    }

    MINI_TEST("FileEncoders", "Decode Mesh") {
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        const Mesh mesh = Mesh::from_vertices_and_faces(
            {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
            {{0, 1, 2}}
        );
        const std::string json_str = file_json_dumps(mesh);
        const Mesh loaded = file_json_loads<Mesh>(json_str);

        MINI_CHECK(loaded.number_of_vertices() == 3);
        MINI_CHECK(loaded.number_of_faces() == 1);
    }

    MINI_TEST("FileEncoders", "Decode Instance Ref") {
        // using session_cpp::InstanceRef;
        // using session_cpp::Xform;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        const InstanceRef instance("def-abc", Xform::translation(1.0, 2.0, 3.0));
        const std::string json_str = file_json_dumps(instance);
        const InstanceRef loaded = file_json_loads<InstanceRef>(json_str);

        MINI_CHECK(loaded.definition_guid == "def-abc");
        MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
    }

    MINI_TEST("FileEncoders", "Decode Element Feature") {
        // using session_cpp::ElementFeature;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        const Polyline outline({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
        const ElementFeature feature("cut", 2, {outline}, "notch");
        const std::string json_str = file_json_dumps(feature);
        const ElementFeature loaded = file_json_loads<ElementFeature>(json_str);

        MINI_CHECK(loaded.feature_type == "cut");
        MINI_CHECK(loaded.face_index == 2);
        MINI_CHECK(loaded.outlines.size() == 1);
        MINI_CHECK(loaded.outlines[0].point_count() == 3);
    }

    MINI_TEST("FileEncoders", "Decode Component") {
        // using session_cpp::Component;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        Component component;
        component.type_name = "FloorBuilder";
        component.name = "floor";
        component.extra["height"] = 650;
        const std::string json_str = file_json_dumps(component);
        const Component loaded = file_json_loads<Component>(json_str);

        MINI_CHECK(loaded.type_name == "FloorBuilder");
        MINI_CHECK(loaded.name == "floor");
        MINI_CHECK(loaded.extra["height"] == 650);
    }

    MINI_TEST("FileEncoders", "Decode Nurbs Surface Trimmed") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::NurbsSurface;
        // using session_cpp::NurbsSurfaceTrimmed;
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        NurbsSurface surface(3, false, 2, 2, 2, 2);
        surface.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surface.set_cv(1, 0, Point(5.0, 0.0, 0.0));
        surface.set_cv(0, 1, Point(0.0, 5.0, 0.0));
        surface.set_cv(1, 1, Point(5.0, 5.0, 0.0));

        const NurbsCurve outer = NurbsCurve::create(
            true,
            1,
            {Point(0.1, 0.1, 0.0), Point(0.9, 0.1, 0.0), Point(0.9, 0.9, 0.0), Point(0.1, 0.9, 0.0)}
        );
        const NurbsCurve inner = NurbsCurve::create(
            true,
            1,
            {Point(0.4, 0.4, 0.0), Point(0.6, 0.4, 0.0), Point(0.6, 0.6, 0.0)}
        );

        NurbsSurfaceTrimmed trimmed = NurbsSurfaceTrimmed::create(surface, outer);
        trimmed.add_inner_loop(inner);
        trimmed.name = "trimmed";
        const std::string json_str = file_json_dumps(trimmed);
        const NurbsSurfaceTrimmed loaded = file_json_loads<NurbsSurfaceTrimmed>(json_str);

        MINI_CHECK(loaded.name == "trimmed");
        MINI_CHECK(loaded.is_trimmed());
        MINI_CHECK(loaded.inner_loop_count() == 1);
    }

    MINI_TEST("FileEncoders", "Decode Nurbs Surface") {
        // using session_cpp::NurbsSurface;
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;
        // using session_cpp::Mesh;

        NurbsSurface surface(3, false, 2, 2, 2, 2);
        surface.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surface.set_cv(1, 0, Point(5.0, 0.0, 0.0));
        surface.set_cv(0, 1, Point(0.0, 5.0, 0.0));
        surface.set_cv(1, 1, Point(5.0, 5.0, 0.0));
        const Mesh mesh = surface.mesh();
        const std::string json_str = file_json_dumps(surface);
        const NurbsSurface loaded = file_json_loads<NurbsSurface>(json_str);

        MINI_CHECK(loaded.mesh().number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded.cv_count(0) == 2);
        MINI_CHECK(loaded.cv_count(1) == 2);
    }

    MINI_TEST("FileEncoders", "Decode BRep") {
        // using session_cpp::BRep;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        const BRep brep = BRep::create_box(1.0, 2.0, 3.0);
        const std::string json_str = file_json_dumps(brep);
        const BRep loaded = file_json_loads<BRep>(json_str);

        MINI_CHECK(loaded.face_count() == 6);
    }

    MINI_TEST("FileEncoders", "Decode Element") {
        // using session_cpp::Element;
        // using session_cpp::ElementFeature;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        const Mesh mesh = Mesh::from_vertices_and_faces(
            {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
            {{0, 1, 2}}
        );
        const Polyline outline({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});

        Element element(mesh, "plate");
        element.add_feature(ElementFeature("cut", 0, {outline}, "notch"));
        const std::string json_str = file_json_dumps(element);
        const Element loaded = file_json_loads<Element>(json_str);

        MINI_CHECK(loaded.name == "plate");
        MINI_CHECK(loaded.features_count() == 1);
    }

    MINI_TEST("FileEncoders", "Decode Objects") {
        // using session_cpp::Component;
        // using session_cpp::ElementFeature;
        // using session_cpp::InstanceRef;
        // using session_cpp::Objects;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::Xform;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        Component component;
        component.type_name = "FloorBuilder";
        const Polyline outline({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
        const std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>("def-abc", Xform::translation(1.0, 2.0, 3.0));
        instance->features.push_back(ElementFeature("drill", 0, {outline}, "hole"));

        Objects objects;
        objects.points->push_back(std::make_shared<Point>(1.0, 2.0, 3.0));
        objects.components->push_back(component);
        objects.instances->push_back(instance);
        const std::string json_str = file_json_dumps(objects);
        const Objects loaded = file_json_loads<Objects>(json_str);

        MINI_CHECK(loaded.points->size() == 1);
        MINI_CHECK(loaded.components->size() == 1);
        MINI_CHECK(loaded.instances->size() == 1);
        MINI_CHECK(loaded.instances->at(0)->features.size() == 1);
    }

    MINI_TEST("FileEncoders", "Decode Tolerance") {
        // using session_cpp::Tolerance;
        // using session_cpp::file_encoders::file_json_dumps;
        // using session_cpp::file_encoders::file_json_loads;

        Tolerance tolerance("MM");
        tolerance.set_absolute(0.01);
        const std::string json_str = file_json_dumps(tolerance);
        const Tolerance loaded = file_json_loads<Tolerance>(json_str);

        MINI_CHECK(loaded.unit() == "MM");
        MINI_CHECK(TOLERANCE.is_close(loaded.absolute(), 0.01));
    }

    MINI_TEST("FileEncoders", "List In List In List") {

        const nlohmann::json data = {{{1, 2}, {3, 4}}, {{5, 6}, {7, 8}}};
        const std::string json_str = data.dump();
        const nlohmann::json loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded[0][0][0] == 1);
        MINI_CHECK(loaded[1][1][1] == 8);
        MINI_CHECK(loaded.size() == 2);
    }

    MINI_TEST("FileEncoders", "Dict Of Lists") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_encode_collection;
        // using session_cpp::file_encoders::file_decode_collection;

        std::vector<Point> points;
        points.push_back(Point(1.0, 0.0, 0.0));
        points.push_back(Point(0.0, 1.0, 0.0));

        nlohmann::json data;
        data["numbers"] = {1, 2, 3};
        data["letters"] = {"a", "b", "c"};
        data["points"] = file_encode_collection(points);

        const std::string json_str = data.dump();
        const nlohmann::json loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded["numbers"].size() == 3);
        MINI_CHECK(loaded["letters"][0] == "a");

        const std::vector<Point> loaded_points = file_decode_collection<Point>(loaded["points"]);

        MINI_CHECK(loaded_points.size() == 2);
        MINI_CHECK(TOLERANCE.is_close(loaded_points[0][0], 1.0));
    }

    MINI_TEST("FileEncoders", "List Of Dict") {
        // using session_cpp::Point;

        const Point point(1.0, 2.0, 3.0);

        nlohmann::json data = nlohmann::json::array();
        data.push_back({{"name", "point1"}, {"value", 10}});
        data.push_back({{"name", "point2"}, {"value", 20}});
        data.push_back({{"geometry", point.jsondump()}});

        const std::string json_str = data.dump();
        const nlohmann::json loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(loaded.size() == 3);
        MINI_CHECK(loaded[0]["name"] == "point1");
        MINI_CHECK(loaded[1]["value"] == 20);

        const Point loaded_point = Point::jsonload(loaded[2]["geometry"]);

        MINI_CHECK(TOLERANCE.is_close(loaded_point[2], 3.0));
    }

    MINI_TEST("FileEncoders", "Dict Of Dicts") {
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point point(1.0, 2.0, 3.0);
        const Vector vec(0.0, 0.0, 1.0);

        nlohmann::json data;
        data["config"]["tolerance"] = 0.001;
        data["config"]["scale"] = 1000;
        data["geometry"]["point"] = point.jsondump();
        data["geometry"]["vector"] = vec.jsondump();

        const std::string json_str = data.dump();
        const nlohmann::json loaded = nlohmann::json::parse(json_str);

        MINI_CHECK(TOLERANCE.is_close(loaded["config"]["tolerance"].get<double>(), 0.001));
        MINI_CHECK(loaded["config"]["scale"] == 1000);

        const Point loaded_point = Point::jsonload(loaded["geometry"]["point"]);
        const Vector loaded_vec = Vector::jsonload(loaded["geometry"]["vector"]);

        MINI_CHECK(TOLERANCE.is_close(loaded_point[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded_vec[2], 1.0));
    }

    MINI_TEST("FileEncoders", "Write Error") {
        // using session_cpp::Point;
        // using session_cpp::file_encoders::file_json_dump;

        const Point point(1.0, 2.0, 3.0);
        bool threw = false;

        try {
            file_json_dump(point, "serialization/missing-directory/test.json");
        } catch (const std::runtime_error&) {
            threw = true;
        }

        MINI_CHECK(threw);
    }

} // namespace session_cpp
