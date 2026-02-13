#include "mini_test.h"
#include "vertex.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Vertex", "Json_roundtrip") {
    Vertex original("v0", "./serialization/test_attribute");

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_vertex.json");
    Vertex loaded = encoders::json_load<Vertex>("./serialization/test_vertex.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.attribute == original.attribute);

    std::filesystem::remove("./serialization/test_vertex.json");
}

} // namespace session_cpp
