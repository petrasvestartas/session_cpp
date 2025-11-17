#include "catch_amalgamated.hpp"
#include "plane.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Plane JSON roundtrip", "[plane]") {
    Plane original = Plane::xy_plane();
    original.name = "test_plane";
    
    std::string filename = "test_plane.json";
    encoders::json_dump(original, filename);
    Plane loaded = encoders::json_load<Plane>(filename);
    
    REQUIRE(loaded.name == original.name);
    
    std::filesystem::remove(filename);
}
