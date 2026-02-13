#include "mini_test.h"
#include "graph.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Graph", "Json_roundtrip") {
    std::filesystem::create_directories("./serialization");
    Graph original("./serialization/test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");

    std::string filename = "./serialization/test_graph.json";
    encoders::json_dump(original, filename);
    Graph loaded = encoders::json_load<Graph>(filename);

    MINI_CHECK(loaded.number_of_vertices() == 2);
    MINI_CHECK(loaded.number_of_edges() == 1);
    MINI_CHECK(loaded.has_edge(std::make_tuple("node1", "node2")));

    std::filesystem::remove(filename);
}

} // namespace session_cpp
