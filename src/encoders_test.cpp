#include "mini_test.h"
#include "encoders.h"
#include "point.h"
#include "vector.h"
#include "line.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;
using namespace session_cpp::encoders;

MINI_TEST("Encoders", "json_dump_load") {
    std::filesystem::create_directories("./serialization");
    Point original(1.5, 2.5, 3.5);
    original.name = "test_point";

    std::string filepath = "./serialization/test_encoders_point.json";
    json_dump(original, filepath);

    Point loaded = json_load<Point>(filepath);

    MINI_CHECK(TOLERANCE.is_close(loaded[0], original[0]));
    MINI_CHECK(TOLERANCE.is_close(loaded[1], original[1]));
    MINI_CHECK(TOLERANCE.is_close(loaded[2], original[2]));
    MINI_CHECK(loaded.name == original.name);

    std::filesystem::remove(filepath);
}

MINI_TEST("Encoders", "json_dumps_loads") {
    Vector original(42.1, 84.2, 126.3);
    original.name = "test_vector";

    std::string json_str = json_dumps(original);
    MINI_CHECK(!json_str.empty());
    MINI_CHECK(json_str.find("Vector") != std::string::npos);

    Vector loaded = json_loads<Vector>(json_str);

    MINI_CHECK(TOLERANCE.is_close(loaded[0], original[0]));
    MINI_CHECK(TOLERANCE.is_close(loaded[1], original[1]));
    MINI_CHECK(TOLERANCE.is_close(loaded[2], original[2]));
    MINI_CHECK(loaded.name == original.name);
}

MINI_TEST("Encoders", "encode_collection_values") {
    std::vector<Point> points;
    points.push_back(Point(1.0, 2.0, 3.0));
    points.push_back(Point(4.0, 5.0, 6.0));
    points.push_back(Point(7.0, 8.0, 9.0));

    auto json_arr = encode_collection(points);

    MINI_CHECK(json_arr.is_array());
    MINI_CHECK(json_arr.size() == 3);
    MINI_CHECK(json_arr[0]["type"] == "Point");
    MINI_CHECK(json_arr[1]["x"] == 4.0);
    MINI_CHECK(json_arr[2]["z"] == 9.0);
}

MINI_TEST("Encoders", "encode_collection_shared_ptr") {
    std::vector<std::shared_ptr<Line>> lines;
    lines.push_back(std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    lines.push_back(std::make_shared<Line>(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));

    auto json_arr = encode_collection(lines);

    MINI_CHECK(json_arr.is_array());
    MINI_CHECK(json_arr.size() == 2);
    MINI_CHECK(json_arr[0]["type"] == "Line");
    MINI_CHECK(json_arr[1]["type"] == "Line");
}

MINI_TEST("Encoders", "decode_collection") {
    std::vector<Point> original_points;
    original_points.push_back(Point(1.0, 2.0, 3.0));
    original_points.push_back(Point(4.0, 5.0, 6.0));

    auto json_arr = encode_collection(original_points);
    auto decoded_points = decode_collection<Point>(json_arr);

    MINI_CHECK(decoded_points.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(decoded_points[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(decoded_points[1][1], 5.0));
}

MINI_TEST("Encoders", "decode_collection_ptr") {
    std::vector<std::shared_ptr<Vector>> original_vectors;
    original_vectors.push_back(std::make_shared<Vector>(1.0, 0.0, 0.0));
    original_vectors.push_back(std::make_shared<Vector>(0.0, 1.0, 0.0));

    auto json_arr = encode_collection(original_vectors);
    auto decoded_vectors = decode_collection_ptr<Vector>(json_arr);

    MINI_CHECK(decoded_vectors.size() == 2);
    MINI_CHECK(TOLERANCE.is_close((*decoded_vectors[0])[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close((*decoded_vectors[1])[1], 1.0));
}

MINI_TEST("Encoders", "nested_collections") {
    std::vector<Line> lines;
    lines.push_back(Line(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    lines.push_back(Line(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));

    auto json_arr = encode_collection(lines);
    std::string json_str = json_arr.dump();
    MINI_CHECK(!json_str.empty());

    auto loaded_json = nlohmann::json::parse(json_str);
    auto loaded = decode_collection<Line>(loaded_json);

    MINI_CHECK(loaded.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(loaded[0].end()[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded[1].end()[1], 1.0));
}

MINI_TEST("Encoders", "roundtrip_file_io") {
    std::filesystem::create_directories("./serialization");
    std::vector<Vector> vectors;
    vectors.push_back(Vector(1.0, 0.0, 0.0));
    vectors.push_back(Vector(0.0, 1.0, 0.0));
    vectors.push_back(Vector(0.0, 0.0, 1.0));

    std::string filepath = "./serialization/test_encoders_collection.json";
    auto json_arr = encode_collection(vectors);
    json_dump(json_arr, filepath);

    auto loaded_json = json_load_data(filepath);
    auto decoded_vectors = decode_collection<Vector>(loaded_json);

    MINI_CHECK(decoded_vectors.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(decoded_vectors[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(decoded_vectors[1][1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(decoded_vectors[2][2], 1.0));

    std::filesystem::remove(filepath);
}

MINI_TEST("Encoders", "pretty_vs_compact") {
    Point point(1.0, 2.0, 3.0);

    std::string pretty = json_dumps(point, true);
    std::string compact = json_dumps(point, false);

    MINI_CHECK(pretty.length() > compact.length());
    MINI_CHECK(pretty.find("\n") != std::string::npos);
    MINI_CHECK(compact.find("\n") == std::string::npos);

    Point loaded_pretty = json_loads<Point>(pretty);
    Point loaded_compact = json_loads<Point>(compact);

    MINI_CHECK(TOLERANCE.is_close(loaded_pretty[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded_compact[0], 1.0));
}

MINI_TEST("Encoders", "decode_primitives") {
    nlohmann::json num = 42;
    std::string json_str = num.dump();
    auto loaded = nlohmann::json::parse(json_str);
    MINI_CHECK(loaded.get<int>() == 42);

    nlohmann::json float_val = 3.14;
    json_str = float_val.dump();
    loaded = nlohmann::json::parse(json_str);
    MINI_CHECK(TOLERANCE.is_close(loaded.get<double>(), 3.14));

    nlohmann::json text = "hello";
    json_str = text.dump();
    loaded = nlohmann::json::parse(json_str);
    MINI_CHECK(loaded.get<std::string>() == "hello");

    nlohmann::json flag = true;
    json_str = flag.dump();
    loaded = nlohmann::json::parse(json_str);
    MINI_CHECK(loaded.get<bool>() == true);
}

MINI_TEST("Encoders", "decode_list") {
    std::vector<int> data = {1, 2, 3};
    nlohmann::json j = data;
    std::string json_str = j.dump();
    auto loaded = nlohmann::json::parse(json_str);
    auto loaded_vec = loaded.get<std::vector<int>>();
    MINI_CHECK(loaded_vec.size() == 3);
    MINI_CHECK(loaded_vec[0] == 1);
    MINI_CHECK(loaded_vec[2] == 3);

    std::vector<Point> points;
    points.push_back(Point(1.0, 2.0, 3.0));
    points.push_back(Point(4.0, 5.0, 6.0));

    auto json_arr = encode_collection(points);
    auto decoded = decode_collection<Point>(json_arr);
    MINI_CHECK(decoded.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(decoded[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(decoded[1][0], 4.0));
}

MINI_TEST("Encoders", "decode_dict") {
    std::map<std::string, int> data;
    data["a"] = 1;
    data["b"] = 2;
    nlohmann::json j = data;
    std::string json_str = j.dump();
    auto loaded = nlohmann::json::parse(json_str);
    MINI_CHECK(loaded["a"].get<int>() == 1);
    MINI_CHECK(loaded["b"].get<int>() == 2);

    Vector vec(1.0, 2.0, 3.0);
    std::string vec_json = json_dumps(vec);
    Vector loaded_vec = json_loads<Vector>(vec_json);
    MINI_CHECK(TOLERANCE.is_close(loaded_vec[0], 1.0));
}

MINI_TEST("Encoders", "list_in_list_in_list") {
    nlohmann::json data = {{{1, 2}, {3, 4}}, {{5, 6}, {7, 8}}};
    std::string json_str = data.dump();
    auto loaded = nlohmann::json::parse(json_str);

    MINI_CHECK(loaded[0][0][0] == 1);
    MINI_CHECK(loaded[1][1][1] == 8);
    MINI_CHECK(loaded.size() == 2);
}

MINI_TEST("Encoders", "dict_of_lists") {
    std::vector<Point> points;
    points.push_back(Point(1.0, 0.0, 0.0));
    points.push_back(Point(0.0, 1.0, 0.0));

    nlohmann::json data;
    data["numbers"] = {1, 2, 3};
    data["letters"] = {"a", "b", "c"};
    data["points"] = encode_collection(points);

    std::string json_str = data.dump();
    auto loaded = nlohmann::json::parse(json_str);

    MINI_CHECK(loaded["numbers"].size() == 3);
    MINI_CHECK(loaded["letters"][0] == "a");
    auto loaded_points = decode_collection<Point>(loaded["points"]);
    MINI_CHECK(loaded_points.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(loaded_points[0][0], 1.0));
}

MINI_TEST("Encoders", "list_of_dict") {
    Point point(1.0, 2.0, 3.0);

    nlohmann::json data = nlohmann::json::array();
    data.push_back({{"name", "point1"}, {"value", 10}});
    data.push_back({{"name", "point2"}, {"value", 20}});
    data.push_back({{"geometry", point.jsondump()}});

    std::string json_str = data.dump();
    auto loaded = nlohmann::json::parse(json_str);

    MINI_CHECK(loaded.size() == 3);
    MINI_CHECK(loaded[0]["name"] == "point1");
    MINI_CHECK(loaded[1]["value"] == 20);
    Point loaded_point = Point::jsonload(loaded[2]["geometry"]);
    MINI_CHECK(TOLERANCE.is_close(loaded_point[2], 3.0));
}

MINI_TEST("Encoders", "dict_of_dicts") {
    Point point(1.0, 2.0, 3.0);
    Vector vec(0.0, 0.0, 1.0);

    nlohmann::json data;
    data["config"]["tolerance"] = 0.001;
    data["config"]["scale"] = 1000;
    data["geometry"]["point"] = point.jsondump();
    data["geometry"]["vector"] = vec.jsondump();

    std::string json_str = data.dump();
    auto loaded = nlohmann::json::parse(json_str);

    MINI_CHECK(TOLERANCE.is_close(loaded["config"]["tolerance"].get<double>(), 0.001));
    MINI_CHECK(loaded["config"]["scale"] == 1000);
    Point loaded_point = Point::jsonload(loaded["geometry"]["point"]);
    Vector loaded_vec = Vector::jsonload(loaded["geometry"]["vector"]);
    MINI_CHECK(TOLERANCE.is_close(loaded_point[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded_vec[2], 1.0));
}

} // namespace session_cpp
