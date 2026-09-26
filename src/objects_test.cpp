#include "mini_test.h"
#include "objects.h"
#include "file_encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Objects", "Constructor") {
    // using session_cpp::Objects;

    Objects objects;
    Objects named("custom_objects");

    MINI_CHECK(objects.name == "my_objects");
    MINI_CHECK(!objects.guid().empty());
    MINI_CHECK(objects.points->empty());
    MINI_CHECK(objects.instances->empty());
    MINI_CHECK(objects.str() == "Objects(name=my_objects, guid=" + objects.guid() + ", points=0)");
    MINI_CHECK(objects.repr() == objects.str());
    MINI_CHECK(named.name == "custom_objects");
}

MINI_TEST("Objects", "Json Roundtrip") {
    // using session_cpp::InstanceRef;
    // using session_cpp::Line;
    // using session_cpp::Mesh;
    // using session_cpp::Objects;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Xform;

    Objects original;
    original.points->push_back(std::make_shared<Point>(1.0, 2.0, 3.0));
    original.points->push_back(std::make_shared<Point>(4.0, 5.0, 6.0));
    original.lines->push_back(std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    original.planes->push_back(std::make_shared<Plane>(Plane::xy_plane()));
    original.meshes->push_back(std::make_shared<Mesh>(Mesh::create_box(1.0, 1.0, 1.0)));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>("def-abc", Xform::identity());
    const std::string guid = instance->guid();
    original.instances->push_back(instance);

    const std::string filename = "serialization/test_objects.json";
    original.file_json_dump(filename);
    const Objects loaded = Objects::file_json_load(filename);
    const Objects parsed = Objects::file_json_loads(original.file_json_dumps());

    MINI_CHECK(loaded.guid() == original.guid());
    MINI_CHECK(parsed.guid() == original.guid());
    MINI_CHECK(loaded.points->size() == 2);
    MINI_CHECK((*loaded.points->at(1))[0] == 4.0);
    MINI_CHECK(loaded.lines->size() == 1);
    MINI_CHECK(loaded.lines->at(0)->end()[0] == 1.0);
    MINI_CHECK(loaded.planes->size() == 1);
    MINI_CHECK(loaded.planes->at(0)->z_axis()[2] == 1.0);
    MINI_CHECK(loaded.meshes->size() == 1);
    MINI_CHECK(loaded.meshes->at(0)->number_of_faces() == 6);
    MINI_CHECK(loaded.instances->size() == 1);
    MINI_CHECK(loaded.instances->at(0)->guid() == guid);
    MINI_CHECK(loaded.instances->at(0)->definition_guid == "def-abc");
}

MINI_TEST("Objects", "Protobuf Roundtrip") {
    // using session_cpp::InstanceRef;
    // using session_cpp::Line;
    // using session_cpp::Mesh;
    // using session_cpp::Objects;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Xform;

    Objects original;
    original.points->push_back(std::make_shared<Point>(1.0, 2.0, 3.0));
    original.points->push_back(std::make_shared<Point>(4.0, 5.0, 6.0));
    original.lines->push_back(std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    original.planes->push_back(std::make_shared<Plane>(Plane::xy_plane()));
    original.meshes->push_back(std::make_shared<Mesh>(Mesh::create_box(1.0, 1.0, 1.0)));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>("def-abc", Xform::identity());
    const std::string guid = instance->guid();
    original.instances->push_back(instance);

    const std::string filename = "serialization/test_objects.bin";
    original.pb_dump(filename);
    const Objects loaded = Objects::pb_load(filename);
    const Objects parsed = Objects::pb_loads(original.pb_dumps());

    MINI_CHECK(parsed.points->size() == 2);
    MINI_CHECK(loaded.points->size() == 2);
    MINI_CHECK((*loaded.points->at(1))[0] == 4.0);
    MINI_CHECK(loaded.lines->size() == 1);
    MINI_CHECK(loaded.lines->at(0)->end()[0] == 1.0);
    MINI_CHECK(loaded.planes->size() == 1);
    MINI_CHECK(loaded.planes->at(0)->z_axis()[2] == 1.0);
    MINI_CHECK(loaded.meshes->size() == 1);
    MINI_CHECK(loaded.meshes->at(0)->number_of_faces() == 6);
    MINI_CHECK(loaded.instances->size() == 1);
    MINI_CHECK(loaded.instances->at(0)->guid() == guid);
    MINI_CHECK(loaded.instances->at(0)->definition_guid == "def-abc");
}

MINI_TEST("Objects", "Component Constructor") {
    // using session_cpp::Component;

    Component component;
    component.type_name = "FloorBuilder";
    component.name = "floor_builder";
    component.extra = {{"size", 3000}, {"height", 650}};

    MINI_CHECK(Component().name == "my_component");
    MINI_CHECK(component.type_name == "FloorBuilder");
    MINI_CHECK(component.name == "floor_builder");
    MINI_CHECK(!component.guid().empty());
    MINI_CHECK(component.extra["size"] == 3000);
}

MINI_TEST("Objects", "Component Json Roundtrip") {
    // using session_cpp::Component;

    Component original;
    original.type_name = "FloorBuilder";
    original.name = "floor_builder";
    original.extra = {{"size", 3000}, {"height", 650}, {"rise", 453}};
    const std::string guid = original.guid();

    const nlohmann::ordered_json data = original.jsondump();

    MINI_CHECK(data["type"] == "FloorBuilder");
    MINI_CHECK(data["guid"] == guid);
    MINI_CHECK(data["size"] == 3000);
    MINI_CHECK(data["height"] == 650);

    const Component loaded = Component::jsonload(data);

    MINI_CHECK(loaded.type_name == "FloorBuilder");
    MINI_CHECK(loaded.guid() == guid);
    MINI_CHECK(loaded.extra["size"] == 3000);
    MINI_CHECK(loaded.extra["rise"] == 453);
}

MINI_TEST("Objects", "Objects Component Json Roundtrip") {
    // using session_cpp::Component;
    // using session_cpp::Objects;
    // using session_cpp::file_encoders::file_json_dump;
    // using session_cpp::file_encoders::file_json_load;

    Objects original;
    Component component;
    component.type_name = "FloorBuilder";
    component.name = "floor_builder";
    component.extra = {{"size", 3000}, {"height", 650}};
    const std::string guid = component.guid();
    original.components->push_back(component);

    const std::string filename = "serialization/test_objects_component.json";
    file_encoders::file_json_dump(original, filename);
    const Objects loaded = file_encoders::file_json_load<Objects>(filename);

    MINI_CHECK(loaded.components->size() == 1);
    MINI_CHECK(loaded.components->at(0).type_name == "FloorBuilder");
    MINI_CHECK(loaded.components->at(0).extra["size"] == 3000);
    MINI_CHECK(loaded.components->at(0).guid() == guid);
}

MINI_TEST("Objects", "Component Protobuf Roundtrip") {
    // using session_cpp::Component;
    // using session_cpp::Objects;

    Objects original;
    Component component;
    component.type_name = "FloorBuilder";
    component.name = "floor_builder";
    component.extra = {{"size", 3000}, {"height", 650}};
    const std::string guid = component.guid();
    original.components->push_back(component);

    const std::string filename = "serialization/test_objects_component.bin";
    original.pb_dump(filename);
    const Objects loaded = Objects::pb_load(filename);

    MINI_CHECK(loaded.components->size() == 1);
    MINI_CHECK(loaded.components->at(0).type_name == "FloorBuilder");
    MINI_CHECK(loaded.components->at(0).guid() == guid);
    MINI_CHECK(loaded.components->at(0).extra["size"] == 3000);
}

} // namespace session_cpp
