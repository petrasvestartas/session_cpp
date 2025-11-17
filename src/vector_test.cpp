#include "catch_amalgamated.hpp"
#include "vector.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Vector JSON roundtrip", "[vector]") {
    Vector original(42.1, 84.2, 126.3);
    original.name = "test_vector";
    
    std::string filename = "test_vector.json";
    encoders::json_dump(original, filename);
    Vector loaded = encoders::json_load<Vector>(filename);
    
    REQUIRE(loaded.x() == Catch::Approx(original.x()));
    REQUIRE(loaded.y() == Catch::Approx(original.y()));
    REQUIRE(loaded.z() == Catch::Approx(original.z()));
    REQUIRE(loaded.name == original.name);
    
    std::filesystem::remove(filename);
}
