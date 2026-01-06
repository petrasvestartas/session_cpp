#include "catch_amalgamated.hpp"
#include "arrow.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Arrow JSON roundtrip", "[arrow]") {
    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Arrow original(line, 1.0);
    original.name = "test_arrow";

    std::filesystem::create_directories("../serialization");
    encoders::json_dump(original, "../serialization/test_arrow.json");
    Arrow loaded = encoders::json_load<Arrow>("../serialization/test_arrow.json");

    REQUIRE(loaded.radius == original.radius);
    REQUIRE(loaded.name == original.name);
    REQUIRE(loaded.mesh.number_of_vertices() == 29);
    REQUIRE(loaded.mesh.number_of_faces() == 28);}
