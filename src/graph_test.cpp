#include "mini_test.h"
#include "graph.h"
#include "encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

///////////////////////////////////////////////////////////////////////////////////////////
// Vertex
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Vertex", "Constructor") {
    // Default constructor
    Vertex v0;

    // Constructor with name + attribute
    Vertex v("v_named", "attr");

    MINI_CHECK(v0.name == "my_vertex");
    MINI_CHECK(v0.attribute == "");
    MINI_CHECK(!v0.guid().empty());
    MINI_CHECK(v.name == "v_named");
    MINI_CHECK(v.attribute == "attr");
}

MINI_TEST("Vertex", "Json Roundtrip") {
    Vertex original("v0", "test_attribute");

    std::string fname = "serialization/test_vertex.json";
    encoders::json_dump(original, fname);
    Vertex loaded = encoders::json_load<Vertex>(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.attribute == original.attribute);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Edge
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Edge", "Constructor") {
    // Constructor with v0/v1/attribute
    Edge e("a", "b", "attr");

    MINI_CHECK(e.v0 == "a");
    MINI_CHECK(e.v1 == "b");
    MINI_CHECK(e.attribute == "attr");
    MINI_CHECK(!e.guid().empty());
}

MINI_TEST("Edge", "Json Roundtrip") {
    Edge original("v0", "v1", "test_edge_attr");

    std::string fname = "serialization/test_edge.json";
    encoders::json_dump(original, fname);
    Edge loaded = encoders::json_load<Edge>(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.v0 == original.v0);
    MINI_CHECK(loaded.v1 == original.v1);
}

MINI_TEST("Edge", "Vertices") {
    Edge e("a", "b");
    auto [u, v] = e.vertices();

    MINI_CHECK(u == "a" && v == "b");
}

MINI_TEST("Edge", "Connects") {
    Edge e("a", "b");

    MINI_CHECK(e.connects("a"));
    MINI_CHECK(e.connects("b"));
    MINI_CHECK(!e.connects("c"));
}

MINI_TEST("Edge", "Other Vertex") {
    Edge e("a", "b");

    MINI_CHECK(e.other_vertex("a") == "b");
    MINI_CHECK(e.other_vertex("b") == "a");
}

///////////////////////////////////////////////////////////////////////////////////////////
// Graph
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Graph", "Constructor") {
    // Default constructor
    Graph g0;

    // Constructor with name
    Graph g("my_named_graph");

    // Minimal string representation
    std::string gstr = g0.str();

    MINI_CHECK(g0.name == "my_graph");
    MINI_CHECK(!g0.guid().empty());
    MINI_CHECK(g0.vertex_count == 0);
    MINI_CHECK(g0.edge_count == 0);
    MINI_CHECK(g.name == "my_named_graph");
    MINI_CHECK(gstr.find("my_graph") != std::string::npos);
}

MINI_TEST("Graph", "Json Roundtrip") {
    Graph original("test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   json_dumps()    │ std::string  │ to JSON string
    //   json_loads(s)   │ std::string  │ from JSON string
    //   json_dump(path) │ file         │ write to file
    //   json_load(path) │ file         │ read from file

    std::string fname = "serialization/test_graph.json";
    original.json_dump(fname);
    Graph loaded = Graph::json_load(fname);

    MINI_CHECK(loaded.number_of_vertices() == 2);
    MINI_CHECK(loaded.number_of_edges() == 1);
    auto edge_key = std::make_tuple<std::string, std::string>("node1", "node2");
    MINI_CHECK(loaded.has_edge(edge_key));
}

MINI_TEST("Graph", "Protobuf Roundtrip") {
    Graph original("test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");

    //   pb_dumps()      │ std::string  │ to protobuf binary string
    //   pb_loads(data)  │ std::string  │ from protobuf binary string
    //   pb_dump(path)   │ file         │ write to file
    //   pb_load(path)   │ file         │ read from file

    std::string filename = "serialization/test_graph.bin";
    original.pb_dump(filename);
    Graph loaded = Graph::pb_load(filename);

    MINI_CHECK(loaded.number_of_vertices() == 2);
    MINI_CHECK(loaded.number_of_edges() == 1);
    auto edge_key = std::make_tuple<std::string, std::string>("node1", "node2");
    MINI_CHECK(loaded.has_edge(edge_key));
}

MINI_TEST("Graph", "Has Node") {
    Graph g("g");
    g.add_node("a");

    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(!g.has_node("missing"));
}

MINI_TEST("Graph", "Has Edge") {
    Graph g("g");
    g.add_edge("a", "b");

    auto ab = std::make_tuple<std::string, std::string>("a", "b");
    auto ac = std::make_tuple<std::string, std::string>("a", "c");
    MINI_CHECK(g.has_edge(ab));
    MINI_CHECK(!g.has_edge(ac));
}

MINI_TEST("Graph", "Add Node") {
    Graph g("g");
    auto key = g.add_node("a");

    MINI_CHECK(key == "a");
    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(g.number_of_vertices() == 1);
}

MINI_TEST("Graph", "Add Edge") {
    Graph g("g");
    auto edge = g.add_edge("a", "b");
    auto [u, v] = edge;

    MINI_CHECK(u == "a" && v == "b");
    MINI_CHECK(g.number_of_edges() == 1);
}

MINI_TEST("Graph", "Remove Node") {
    Graph g("g");
    g.add_edge("a", "b");
    g.remove_node("a");

    MINI_CHECK(!g.has_node("a"));
    MINI_CHECK(g.number_of_edges() == 0);
}

MINI_TEST("Graph", "Remove Edge") {
    Graph g("g");
    g.add_edge("a", "b");
    auto edge_key = std::make_tuple<std::string, std::string>("a", "b");
    g.remove_edge(edge_key);

    MINI_CHECK(g.number_of_edges() == 0);
    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(g.has_node("b"));
}

MINI_TEST("Graph", "Get Vertices") {
    Graph g("g");
    g.add_node("a");
    g.add_node("b");

    auto verts = g.get_vertices();

    MINI_CHECK(verts.size() == 2);
}

MINI_TEST("Graph", "Get Edges") {
    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");

    auto edges = g.get_edges();

    MINI_CHECK(edges.size() == 2);
}

MINI_TEST("Graph", "Neighbors") {
    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("a", "c");

    auto neigh = g.neighbors("a");

    MINI_CHECK(neigh.size() == 2);
}

MINI_TEST("Graph", "Get Neighbors") {
    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("a", "c");

    auto neigh = g.get_neighbors("a");

    MINI_CHECK(neigh.size() == 2);
}

MINI_TEST("Graph", "Number Of Vertices") {
    Graph g("g");
    g.add_node("a");
    g.add_node("b");
    g.add_node("c");

    MINI_CHECK(g.number_of_vertices() == 3);
}

MINI_TEST("Graph", "Number Of Edges") {
    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");

    MINI_CHECK(g.number_of_edges() == 2);
}

MINI_TEST("Graph", "Clear") {
    Graph g("g");
    g.add_edge("a", "b");
    g.clear();

    MINI_CHECK(g.number_of_vertices() == 0);
    MINI_CHECK(g.number_of_edges() == 0);
}

MINI_TEST("Graph", "Node Attribute") {
    Graph g("g");
    g.add_node("a", "initial");
    g.node_attribute("a", "updated");

    MINI_CHECK(g.node_attribute("a") == "updated");
}

MINI_TEST("Graph", "Edge Attribute") {
    Graph g("g");
    g.add_edge("a", "b", "initial");
    g.edge_attribute("a", "b", "updated");

    MINI_CHECK(g.edge_attribute("a", "b") == "updated");
}

MINI_TEST("Graph", "Bfs") {
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
    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    auto cycles = g.cycle_basis();

    MINI_CHECK(cycles.size() == 1);
}

} // namespace session_cpp
