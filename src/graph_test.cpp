#include "mini_test.h"
#include "graph.h"
#include "encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Graph", "Constructor") {
    // uncomment #include "graph.h"

    // Default constructor
    Graph g0;

    // Constructor with name
    Graph g("my_named_graph");

    // Minimal string representation
    std::string gstr = g.str();

    MINI_CHECK(g0.name == "my_graph");
    MINI_CHECK(!g0.guid().empty());
    MINI_CHECK(g.name == "my_named_graph");
    MINI_CHECK(gstr.find("Graph") != std::string::npos);
}

MINI_TEST("Graph", "Json Roundtrip") {
    // uncomment #include "graph.h"
    // uncomment #include "encoders.h"

    Graph original("test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");

    std::string fname = "serialization/test_graph.json";
    encoders::json_dump(original, fname);
    Graph loaded = encoders::json_load<Graph>(fname);

    MINI_CHECK(loaded.number_of_vertices() == 2);
    MINI_CHECK(loaded.number_of_edges() == 1);
    auto edge_key = std::make_tuple<std::string, std::string>("node1", "node2");
    MINI_CHECK(loaded.has_edge(edge_key));
}

MINI_TEST("Graph", "Has Node") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_node("a");

    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(!g.has_node("missing"));
}

MINI_TEST("Graph", "Has Edge") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");

    auto ab = std::make_tuple<std::string, std::string>("a", "b");
    auto ac = std::make_tuple<std::string, std::string>("a", "c");
    MINI_CHECK(g.has_edge(ab));
    MINI_CHECK(!g.has_edge(ac));
}

MINI_TEST("Graph", "Add Node") {
    // uncomment #include "graph.h"

    Graph g("g");
    auto key = g.add_node("a");

    MINI_CHECK(key == "a");
    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(g.number_of_vertices() == 1);
}

MINI_TEST("Graph", "Add Edge") {
    // uncomment #include "graph.h"

    Graph g("g");
    auto edge = g.add_edge("a", "b");
    auto [u, v] = edge;

    MINI_CHECK(u == "a" && v == "b");
    MINI_CHECK(g.number_of_edges() == 1);
}

MINI_TEST("Graph", "Remove Node") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.remove_node("a");

    MINI_CHECK(!g.has_node("a"));
    MINI_CHECK(g.number_of_edges() == 0);
}

MINI_TEST("Graph", "Remove Edge") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    auto edge_key = std::make_tuple<std::string, std::string>("a", "b");
    g.remove_edge(edge_key);

    MINI_CHECK(g.number_of_edges() == 0);
    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(g.has_node("b"));
}

MINI_TEST("Graph", "Get Vertices") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_node("a");
    g.add_node("b");

    auto verts = g.get_vertices();

    MINI_CHECK(verts.size() == 2);
}

MINI_TEST("Graph", "Get Edges") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");

    auto edges = g.get_edges();

    MINI_CHECK(edges.size() == 2);
}

MINI_TEST("Graph", "Neighbors") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("a", "c");

    auto neigh = g.neighbors("a");

    MINI_CHECK(neigh.size() == 2);
}

MINI_TEST("Graph", "Number Of Vertices") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_node("a");
    g.add_node("b");
    g.add_node("c");

    MINI_CHECK(g.number_of_vertices() == 3);
}

MINI_TEST("Graph", "Number Of Edges") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");

    MINI_CHECK(g.number_of_edges() == 2);
}

MINI_TEST("Graph", "Clear") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.clear();

    MINI_CHECK(g.number_of_vertices() == 0);
    MINI_CHECK(g.number_of_edges() == 0);
}

MINI_TEST("Graph", "Node Attribute") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_node("a", "initial");
    g.node_attribute("a", "updated");

    MINI_CHECK(g.node_attribute("a") == "updated");
}

MINI_TEST("Graph", "Edge Attribute") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b", "initial");
    g.edge_attribute("a", "b", "updated");

    MINI_CHECK(g.edge_attribute("a", "b") == "updated");
}

MINI_TEST("Graph", "Bfs") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto result = g.bfs("a");

    MINI_CHECK(result == (std::vector<std::string>{"a", "b", "c", "d"}));
}

MINI_TEST("Graph", "Dfs") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto result = g.dfs("a");

    MINI_CHECK(result == (std::vector<std::string>{"a", "b", "c", "d"}));
}

MINI_TEST("Graph", "Connected Components") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");
    auto comps = g.connected_components();

    MINI_CHECK(comps.size() == 2);
    MINI_CHECK(g.is_connected() == false);
    MINI_CHECK(g.number_connected_components() == 2);
}

MINI_TEST("Graph", "Shortest Path") {
    // uncomment #include "graph.h"

    Graph g("g");
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
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");

    MINI_CHECK(g.has_cycle() == true);
    Graph g2("g2");
    g2.add_edge("x", "y");
    g2.add_edge("y", "z");
    MINI_CHECK(g2.has_cycle() == false);
}

MINI_TEST("Graph", "Cycle Basis") {
    // uncomment #include "graph.h"

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    auto cycles = g.cycle_basis();

    MINI_CHECK(cycles.size() == 1);
}

} // namespace session_cpp
