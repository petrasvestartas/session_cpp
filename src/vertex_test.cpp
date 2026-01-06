#include "catch_amalgamated.hpp"
#include "vertex.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Vertex JSON roundtrip", "[vertex]") {
    Vertex original("v0", "serialization/test_attribute");
    
    encoders::json_dump(original, "serialization/test_vertex.json");
    Vertex loaded = encoders::json_load<Vertex>("serialization/test_vertex.json");
    
    REQUIRE(loaded.name == original.name);
    REQUIRE(loaded.attribute == original.attribute);
    
    std::filesystem::remove("serialization/test_vertex.json");
}
