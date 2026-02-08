#include "mini_test.h"
#include "boundingbox.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("BoundingBox", "json_roundtrip") {
    BoundingBox original = BoundingBox::from_point(Point(1.0, 2.0, 3.0), 5.0);
    original.name = "test_bbox";

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_boundingbox.json");
    BoundingBox loaded = encoders::json_load<BoundingBox>("./serialization/test_boundingbox.json");

    MINI_CHECK(loaded.name == original.name);
    std::filesystem::remove("./serialization/test_boundingbox.json");
}

} // namespace session_cpp
