#include "mini_test.h"
#include "graph.h"
#include "encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Edge", "Constructor") {
    // uncomment #include "graph.h"

    // Default constructor
    Edge e0;

    // Constructor with v0/v1/attribute
    Edge e("a", "b", "attr");

    // Minimal string representation
    std::string estr = e.str();

    MINI_CHECK(e0.v0 == "");
    MINI_CHECK(e0.v1 == "");
    MINI_CHECK(!e0.guid().empty());
    MINI_CHECK(e.v0 == "a");
    MINI_CHECK(e.v1 == "b");
    MINI_CHECK(e.attribute == "attr");
    MINI_CHECK(estr.find("Edge") != std::string::npos);
}

MINI_TEST("Edge", "Json Roundtrip") {
    // uncomment #include "graph.h"
    // uncomment #include "encoders.h"

    Edge original("v0", "v1", "test_edge_attr");
    encoders::json_dump(original, "serialization/test_edge.json");
    Edge loaded = encoders::json_load<Edge>("serialization/test_edge.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.v0 == original.v0);
    MINI_CHECK(loaded.v1 == original.v1);
}

MINI_TEST("Edge", "Vertices") {
    // uncomment #include "graph.h"

    Edge e("a", "b");
    auto [u, v] = e.vertices();

    MINI_CHECK(u == "a" && v == "b");
}

MINI_TEST("Edge", "Connects") {
    // uncomment #include "graph.h"

    Edge e("a", "b");

    MINI_CHECK(e.connects("a"));
    MINI_CHECK(e.connects("b"));
    MINI_CHECK(!e.connects("c"));
}

MINI_TEST("Edge", "Other Vertex") {
    // uncomment #include "graph.h"

    Edge e("a", "b");

    MINI_CHECK(e.other_vertex("a") == "b");
    MINI_CHECK(e.other_vertex("b") == "a");
}

} // namespace session_cpp
