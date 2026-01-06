#include "catch_amalgamated.hpp"
#include "graph.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("Graph JSON roundtrip", "[graph]") {
    std::filesystem::create_directories("./serialization");
    Graph original("./serialization/test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");
    
    std::string filename = "./serialization/test_graph.json";
    encoders::json_dump(original, filename);
    Graph loaded = encoders::json_load<Graph>(filename);
    
    REQUIRE(loaded.number_of_vertices() == 2);
    REQUIRE(loaded.number_of_edges() == 1);
    REQUIRE(loaded.has_edge(std::make_tuple("node1", "node2")));
    
    std::filesystem::remove(filename);
}
