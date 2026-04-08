#include "mini_test.h"
#include "graph.h"
#include "encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Vertex", "Constructor") {
    // uncomment #include "graph.h"

    // Default constructor
    Vertex v0;

    // Constructor with name + attribute
    Vertex v("v_named", "attr");

    // Minimal string representation
    std::string vstr = v.str();

    MINI_CHECK(v0.name == "my_vertex");
    MINI_CHECK(v0.attribute == "");
    MINI_CHECK(!v0.guid().empty());
    MINI_CHECK(v.name == "v_named");
    MINI_CHECK(v.attribute == "attr");
    MINI_CHECK(vstr.find("Vertex") != std::string::npos);
}

MINI_TEST("Vertex", "Json Roundtrip") {
    // uncomment #include "graph.h"
    // uncomment #include "encoders.h"

    Vertex original("v0", "test_attribute");
    encoders::json_dump(original, "serialization/test_vertex.json");
    Vertex loaded = encoders::json_load<Vertex>("serialization/test_vertex.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.attribute == original.attribute);
}

} // namespace session_cpp
