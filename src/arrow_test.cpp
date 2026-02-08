#include "mini_test.h"
#include "arrow.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Arrow", "json_roundtrip") {
    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Arrow original(line, 1.0);
    original.name = "test_arrow";

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_arrow.json");
    Arrow loaded = encoders::json_load<Arrow>("./serialization/test_arrow.json");

    MINI_CHECK(TOLERANCE.is_close(loaded.radius, original.radius));
    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.mesh.number_of_vertices() == 29);
    MINI_CHECK(loaded.mesh.number_of_faces() == 28);
}

} // namespace session_cpp
