#include "catch_amalgamated.hpp"
#include "cylinder.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Cylinder JSON roundtrip", "[cylinder]") {
    std::filesystem::create_directories("./serialization");
    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Cylinder original(line, 1.0);
    original.name = "test_cylinder";
    
    std::string filename = "./serialization/test_cylinder.json";
    encoders::json_dump(original, filename);
    Cylinder loaded = encoders::json_load<Cylinder>(filename);
    
    REQUIRE(loaded.radius == Catch::Approx(original.radius));
    REQUIRE(loaded.name == original.name);
    
    std::filesystem::remove(filename);
}
