#include "mini_test.h"
#include "quaternion.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Quaternion", "json_roundtrip") {
    Vector axis(0.0, 0.0, 1.0);
    Quaternion original = Quaternion::from_axis_angle(axis, 1.5708);

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_quaternion.json");
    Quaternion loaded = encoders::json_load<Quaternion>("./serialization/test_quaternion.json");

    MINI_CHECK(TOLERANCE.is_close(loaded.s, original.s));
}

} // namespace session_cpp
