#include "mini_test.h"
#include "objects.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Objects", "Json_roundtrip") {
    std::filesystem::create_directories("./serialization");
    Objects original;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    original.points->push_back(point1);
    original.points->push_back(point2);

    std::string filename = "./serialization/test_objects.json";
    encoders::json_dump(original, filename);
    Objects loaded = encoders::json_load<Objects>(filename);

    MINI_CHECK(loaded.points->size() == original.points->size());

    std::filesystem::remove(filename);
}

} // namespace session_cpp
