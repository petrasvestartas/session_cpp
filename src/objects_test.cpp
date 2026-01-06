#include "catch_amalgamated.hpp"
#include "objects.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Objects JSON roundtrip", "[objects]") {
    std::filesystem::create_directories("../serialization");
    Objects original;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    original.points->push_back(point1);
    original.points->push_back(point2);
    
    std::string filename = "../serialization/test_objects.json";
    encoders::json_dump(original, filename);
    Objects loaded = encoders::json_load<Objects>(filename);

    REQUIRE(loaded.points->size() == original.points->size());
    
    std::filesystem::remove(filename);
}
