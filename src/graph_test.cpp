#include "mini_test.h"
#include "graph.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Vertex", "Json Roundtrip") {
    Vertex original("v0", "./serialization/test_attribute");

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_vertex.json");
    Vertex loaded = encoders::json_load<Vertex>("./serialization/test_vertex.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.attribute == original.attribute);

    std::filesystem::remove("./serialization/test_vertex.json");
}

MINI_TEST("Edge", "Json Roundtrip") {
    Edge original("./serialization/test_edge", "v0", "v1");

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_edge.json");
    Edge loaded = encoders::json_load<Edge>("./serialization/test_edge.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.v0 == original.v0);
    MINI_CHECK(loaded.v1 == original.v1);
    std::filesystem::remove("./serialization/test_edge.json");
}

MINI_TEST("Graph", "Json Roundtrip") {
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

MINI_TEST("Graph", "Bfs") {
    Graph g("test");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto result = g.bfs("a");

    MINI_CHECK(result == (std::vector<std::string>{"a", "b", "c", "d"}));
}

MINI_TEST("Graph", "Dfs") {
    Graph g("test");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto result = g.dfs("a");

    MINI_CHECK(result == (std::vector<std::string>{"a", "b", "c", "d"}));
}

MINI_TEST("Graph", "Connected Components") {
    Graph g("test");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto comps = g.connected_components();

    MINI_CHECK(comps.size() == 2);
    MINI_CHECK(comps[0] == (std::vector<std::string>{"a", "b", "c", "d"}));
    MINI_CHECK(comps[1] == (std::vector<std::string>{"e", "f"}));
    MINI_CHECK(g.is_connected() == false);
    MINI_CHECK(g.number_connected_components() == 2);
}

MINI_TEST("Graph", "Shortest Path") {
    Graph g("test");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");

    MINI_CHECK(g.shortest_path("a", "d") == (std::vector<std::string>{"a", "b", "d"}));
    MINI_CHECK(g.shortest_path_length("a", "d") == 2);
    MINI_CHECK(g.shortest_path("a", "e") == (std::vector<std::string>{}));
    MINI_CHECK(g.shortest_path_length("a", "e") == -1);
}

MINI_TEST("Graph", "Has Cycle") {
    Graph g("test");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");

    MINI_CHECK(g.has_cycle() == true);
    Graph g2("test2");
    g2.add_edge("x", "y");
    g2.add_edge("y", "z");
    MINI_CHECK(g2.has_cycle() == false);
}

MINI_TEST("Graph", "Cycle Basis") {
    Graph g("test");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto cycles = g.cycle_basis();

    MINI_CHECK(cycles.size() == 1);
    std::set<std::string> cycle_set(cycles[0].begin(), cycles[0].end());
    MINI_CHECK(cycle_set == (std::set<std::string>{"a", "b", "c"}));
}

} // namespace session_cpp
