#include "catch_amalgamated.hpp"
#include "edge.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Edge JSON roundtrip", "[edge]") {
    Edge original("../serialization/test_edge", "v0", "v1");

    std::filesystem::create_directories("../serialization");
    encoders::json_dump(original, "../serialization/test_edge.json");
    Edge loaded = encoders::json_load<Edge>("../serialization/test_edge.json");

    REQUIRE(loaded.name == original.name);
    REQUIRE(loaded.v0 == original.v0);
    REQUIRE(loaded.v1 == original.v1);    
    std::filesystem::remove("../serialization/test_edge.json");
}
