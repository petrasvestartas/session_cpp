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

MINI_TEST("Objects", "Component Constructor") {
    // Component is a generic envelope for custom domain objects.
    // type_name identifies the class; extra holds all custom fields as JSON.
    Component c;
    c.type_name = "FloorBuilder";
    c.name      = "floor_builder";
    c.extra     = {{"size", 3000}, {"height", 650}};

    MINI_CHECK(c.type_name == "FloorBuilder");
    MINI_CHECK(c.name == "floor_builder");
    MINI_CHECK(!c.guid().empty());
    MINI_CHECK(c.extra["size"] == 3000);
}

MINI_TEST("Objects", "Component Json Roundtrip") {
    // Round-trip a Component through JSON: all custom fields must survive.
    Component original;
    original.type_name = "FloorBuilder";
    original.name      = "floor_builder";
    original.extra     = {{"size", 3000}, {"height", 650}, {"rise", 453}};
    std::string original_guid = original.guid();

    // jsondump produces a flat dict: type/guid/name + all extra fields
    auto j = original.jsondump();
    MINI_CHECK(j["type"]   == "FloorBuilder");
    MINI_CHECK(j["guid"]   == original_guid);
    MINI_CHECK(j["size"]   == 3000);
    MINI_CHECK(j["height"] == 650);

    // jsonload reconstructs the Component from that dict
    Component loaded = Component::jsonload(j);
    MINI_CHECK(loaded.type_name        == "FloorBuilder");
    MINI_CHECK(loaded.guid()           == original_guid);
    MINI_CHECK(loaded.extra["size"]    == 3000);
    MINI_CHECK(loaded.extra["rise"]    == 453);
}

MINI_TEST("Objects", "Objects Component Json Roundtrip") {
    // An Objects collection serializes components alongside geometry.
    Objects original;
    Component c;
    c.type_name = "FloorBuilder";
    c.name      = "floor_builder";
    c.extra     = {{"size", 3000}, {"height", 650}};
    original.components->push_back(c);

    std::string filename = "serialization/test_objects_component.json";
    encoders::json_dump(original, filename);
    Objects loaded = encoders::json_load<Objects>(filename);

    MINI_CHECK(loaded.components->size() == 1);
    MINI_CHECK(loaded.components->at(0).type_name        == "FloorBuilder");
    MINI_CHECK(loaded.components->at(0).extra["size"]    == 3000);
    MINI_CHECK(loaded.components->at(0).guid()           == c.guid());
}

} // namespace session_cpp
