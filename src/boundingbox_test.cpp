#include "catch_amalgamated.hpp"
#include "boundingbox.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("BoundingBox JSON roundtrip", "[boundingbox]") {
    BoundingBox original = BoundingBox::from_point(Point(1.0, 2.0, 3.0), 5.0);
    original.name = "test_bbox";

    std::filesystem::create_directories("../serialization");
    encoders::json_dump(original, "../serialization/test_boundingbox.json");
    BoundingBox loaded = encoders::json_load<BoundingBox>("../serialization/test_boundingbox.json");

    REQUIRE(loaded.name == original.name);    
    std::filesystem::remove("../serialization/test_boundingbox.json");
}
