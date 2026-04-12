#include "mini_test.h"
#include "objects.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Objects", "Constructor") {
    // uncomment #include "objects.h"
    Objects obj;
    Objects named("custom_objects");

    MINI_CHECK(obj.name == "my_objects");
    MINI_CHECK(!obj.guid().empty());
    MINI_CHECK(!obj.str().empty());
    MINI_CHECK(named.name == "custom_objects");
}

MINI_TEST("Objects", "Json Roundtrip") {
    // uncomment #include "objects.h"
    // uncomment #include "encoders.h"
    // uncomment #include "point.h"
    Objects original;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    original.points->push_back(point1);
    original.points->push_back(point2);

    std::string filename = "serialization/test_objects.json";
    encoders::json_dump(original, filename);
    Objects loaded = encoders::json_load<Objects>(filename);

    MINI_CHECK(loaded.points->size() == original.points->size());
}

MINI_TEST("Objects", "Protobuf Roundtrip") {
    // uncomment #include "objects.h"
    // uncomment #include "point.h"
    Objects original;
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto point2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    original.points->push_back(point1);
    original.points->push_back(point2);

    std::string filename = "serialization/test_objects.bin";
    original.pb_dump(filename);
    Objects loaded = Objects::pb_load(filename);

    MINI_CHECK(loaded.points->size() == original.points->size());
}

} // namespace session_cpp
