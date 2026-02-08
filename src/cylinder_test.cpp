#include "mini_test.h"
#include "cylinder.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Cylinder", "json_roundtrip") {
    std::filesystem::create_directories("./serialization");
    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
    Cylinder original(line, 1.0);
    original.name = "test_cylinder";

    std::string filename = "./serialization/test_cylinder.json";
    encoders::json_dump(original, filename);
    Cylinder loaded = encoders::json_load<Cylinder>(filename);

    MINI_CHECK(TOLERANCE.is_close(loaded.radius, original.radius));
    MINI_CHECK(loaded.name == original.name);

    std::filesystem::remove(filename);
}

} // namespace session_cpp
