#include "mini_test.h"
#include "edge.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Edge", "json_roundtrip") {
    Edge original("./serialization/test_edge", "v0", "v1");

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_edge.json");
    Edge loaded = encoders::json_load<Edge>("./serialization/test_edge.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.v0 == original.v0);
    MINI_CHECK(loaded.v1 == original.v1);
    std::filesystem::remove("./serialization/test_edge.json");
}

} // namespace session_cpp
