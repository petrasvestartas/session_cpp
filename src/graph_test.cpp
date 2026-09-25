#include "mini_test.h"
#include "graph.h"
#include "graph.pb.h"
#include "file_encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

// ═══════════════════════════════════════════════════════════════════════════
// Vertex
// ═══════════════════════════════════════════════════════════════════════════
MINI_TEST("Vertex", "Constructor") {

    Vertex v0;
    Vertex v("v_named", "attr");

    MINI_CHECK(v0.name == "my_vertex");
    MINI_CHECK(v0.attribute == "");
    MINI_CHECK(!v0.guid().empty());
    MINI_CHECK(v.name == "v_named");
    MINI_CHECK(v.attribute == "attr");
}

MINI_TEST("Vertex", "Json Roundtrip") {

    Vertex original("v0", "test_attribute");
    original.attributes["load"] = 1.5;

    const std::string fname = "serialization/test_vertex.json";
    file_encoders::file_json_dump(original, fname);
    const Vertex loaded = file_encoders::file_json_load<Vertex>(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.attribute == original.attribute);
    MINI_CHECK(loaded.attributes == original.attributes);
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge
// ═══════════════════════════════════════════════════════════════════════════
MINI_TEST("Edge", "Constructor") {

    const Edge e("a", "b", "attr");

    MINI_CHECK(e.v0 == "a");
    MINI_CHECK(e.v1 == "b");
    MINI_CHECK(e.attribute == "attr");
    MINI_CHECK(!e.guid().empty());
}

MINI_TEST("Edge", "Json Roundtrip") {

    Edge original("v0", "v1", "test_edge_attr");
    original.attributes["weight"] = 2.5;

    const std::string fname = "serialization/test_edge.json";
    file_encoders::file_json_dump(original, fname);
    const Edge loaded = file_encoders::file_json_load<Edge>(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.v0 == original.v0);
    MINI_CHECK(loaded.v1 == original.v1);
    MINI_CHECK(loaded.attributes == original.attributes);
}

MINI_TEST("Edge", "Vertices") {

    const Edge e("a", "b");
    std::string u;
    std::string v;
    std::tie(u, v) = e.vertices();

    MINI_CHECK(u == "a" && v == "b");
}

MINI_TEST("Edge", "Connects") {

    const Edge e("a", "b");

    MINI_CHECK(e.connects("a"));
    MINI_CHECK(e.connects("b"));
    MINI_CHECK(!e.connects("c"));
}

MINI_TEST("Edge", "Other Vertex") {

    const Edge e("a", "b");

    MINI_CHECK(e.other_vertex("a") == "b");
    MINI_CHECK(e.other_vertex("b") == "a");
}

// ═══════════════════════════════════════════════════════════════════════════
// Graph
// ═══════════════════════════════════════════════════════════════════════════
MINI_TEST("Graph", "Constructor") {

    const Graph g0;
    const Graph g("my_named_graph");
    const std::string gstr = g0.str();
    const std::string grepr = g0.repr();

    MINI_CHECK(g0.name == "my_graph");
    MINI_CHECK(!g0.guid().empty());
    MINI_CHECK(g0.vertex_count == 0);
    MINI_CHECK(g0.edge_count == 0);
    MINI_CHECK(g.name == "my_named_graph");
    MINI_CHECK(gstr == "<Graph with 0 vertices, 0 edges: my_graph>");
    MINI_CHECK(grepr == "Graph(" + g0.guid() + ", my_graph, 0, 0)");
}

MINI_TEST("Graph", "Json Roundtrip") {

    Graph original("test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");

    const std::tuple<std::string, std::string> edge_key = std::make_tuple<std::string, std::string>("node1", "node2");
    original.update_default_vertex_attributes({{"load", 1.0}});
    original.update_default_edge_attributes({{"weight", 2.0}});
    original.set_vertex_attribute("node1", "load", 3.0);
    original.set_edge_attribute(edge_key, "weight", 4.0);

    const std::string fname = "serialization/test_graph.json";
    original.file_json_dump(fname);
    const Graph loaded = Graph::file_json_load(fname);

    MINI_CHECK(loaded.number_of_vertices() == 2);
    MINI_CHECK(loaded.number_of_edges() == 1);
    MINI_CHECK(loaded.has_edge(edge_key));
    MINI_CHECK(loaded.default_vertex_attributes == original.default_vertex_attributes);
    MINI_CHECK(loaded.default_edge_attributes == original.default_edge_attributes);
    MINI_CHECK(loaded.vertex_attribute("node1", "load") == 3.0);
    MINI_CHECK(loaded.vertex_attribute("node2", "load") == 1.0);
    MINI_CHECK(loaded.edge_attribute(edge_key, "weight") == 4.0);
}

MINI_TEST("Graph", "Protobuf Roundtrip") {

    Graph original("test_graph");
    original.add_node("node1", "Node 1");
    original.add_node("node2", "Node 2");
    original.add_edge("node1", "node2", "edge1");

    const std::tuple<std::string, std::string> edge_key = std::make_tuple<std::string, std::string>("node1", "node2");
    original.update_default_vertex_attributes({{"load", 1.0}});
    original.update_default_edge_attributes({{"weight", 2.0}});
    original.set_vertex_attribute("node1", "load", 3.0);
    original.set_edge_attribute(edge_key, "weight", 4.0);

    const std::string guid = original.guid();
    const std::string filename = "serialization/test_graph.bin";
    original.pb_dump(filename);

    const Graph loaded = Graph::pb_load(filename);
    const Graph converted = Graph::from_proto(original.to_proto());

    MINI_CHECK(loaded.number_of_vertices() == 2);
    MINI_CHECK(loaded.number_of_edges() == 1);
    MINI_CHECK(loaded.has_edge(edge_key));
    MINI_CHECK(loaded.guid() == guid);
    MINI_CHECK(loaded.default_vertex_attributes == original.default_vertex_attributes);
    MINI_CHECK(loaded.default_edge_attributes == original.default_edge_attributes);
    MINI_CHECK(loaded.vertex_attribute("node1", "load") == 3.0);
    MINI_CHECK(loaded.vertex_attribute("node2", "load") == 1.0);
    MINI_CHECK(loaded.edge_attribute(edge_key, "weight") == 4.0);
    MINI_CHECK(converted.number_of_edges() == 1);
    MINI_CHECK(converted.guid() == guid);
    MINI_CHECK(converted.edge_attribute(edge_key, "weight") == 4.0);
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

    const std::tuple<std::string, std::string> ab = std::make_tuple<std::string, std::string>("a", "b");
    const std::tuple<std::string, std::string> ac = std::make_tuple<std::string, std::string>("a", "c");

    MINI_CHECK(g.has_edge(ab));
    MINI_CHECK(!g.has_edge(ac));
}

MINI_TEST("Graph", "Has Guid") {

    const Vertex v("a");
    const Edge e("a", "b");

    MINI_CHECK(!v.has_guid());
    MINI_CHECK(!e.has_guid());

    const std::string minted = v.guid();

    MINI_CHECK(!minted.empty());
    MINI_CHECK(v.has_guid());
    MINI_CHECK(v.guid() == minted);
}

MINI_TEST("Graph", "Add Node") {

    Graph g("g");
    const std::string key = g.add_node("a");

    MINI_CHECK(key == "a");
    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(g.number_of_vertices() == 1);
}

MINI_TEST("Graph", "Add Edge") {

    Graph g("g");
    const std::tuple<std::string, std::string> edge = g.add_edge("a", "b");
    std::string u;
    std::string v;
    std::tie(u, v) = edge;
    g.add_edge("b", "a", "updated");

    MINI_CHECK(u == "a" && v == "b");
    MINI_CHECK(g.number_of_edges() == 1);
    MINI_CHECK(g.edge_count == 1);
    MINI_CHECK(g.edge_label("a", "b") == "updated");
    MINI_CHECK(g.edges.at("a").at("b").guid() == g.edges.at("b").at("a").guid());
}

MINI_TEST("Graph", "Remove Node") {

    Graph g("g");
    g.add_edge("a", "b");
    g.remove_node("a");

    MINI_CHECK(!g.has_node("a"));
    MINI_CHECK(g.number_of_edges() == 0);
    MINI_CHECK(g.edge_count == 0);
}

MINI_TEST("Graph", "Remove Edge") {

    Graph g("g");
    g.add_edge("a", "b");
    const std::tuple<std::string, std::string> edge_key = std::make_tuple<std::string, std::string>("a", "b");
    g.remove_edge(edge_key);

    MINI_CHECK(g.number_of_edges() == 0);
    MINI_CHECK(g.has_node("a"));
    MINI_CHECK(g.has_node("b"));
}

MINI_TEST("Graph", "Get Vertices") {

    Graph g("g");
    g.add_node("a");
    g.add_node("b");

    const std::vector<Vertex> verts = g.get_vertices();

    MINI_CHECK(verts.size() == 2);
}

MINI_TEST("Graph", "Get Edges") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");

    const std::vector<std::tuple<std::string, std::string>> edges = g.get_edges();

    MINI_CHECK(edges.size() == 2);
}

MINI_TEST("Graph", "Neighbors") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("a", "c");

    const std::vector<std::string> neigh = g.neighbors("a");

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

MINI_TEST("Graph", "Node Label") {

    Graph g("g");
    g.add_node("a", "initial");
    g.node_label("a", "updated");

    MINI_CHECK(g.node_label("a") == "updated");
}

MINI_TEST("Graph", "Edge Label") {

    Graph g("g");
    g.add_edge("a", "b", "initial");
    g.edge_label("a", "b", "updated");

    MINI_CHECK(g.edge_label("a", "b") == "updated");
}

MINI_TEST("Graph", "Update Default Vertex Attributes") {

    Graph g("g");
    g.update_default_vertex_attributes({{"is_support", 0.0}, {"load", 0.0}});
    g.update_default_vertex_attributes({{"load", -1.0}});

    MINI_CHECK(g.default_vertex_attributes.size() == 2);
    MINI_CHECK(g.default_vertex_attributes["is_support"] == 0.0);
    MINI_CHECK(g.default_vertex_attributes["load"] == -1.0);
}

MINI_TEST("Graph", "Update Default Edge Attributes") {

    Graph g("g");
    g.update_default_edge_attributes({{"weight", 1.0}, {"stiffness", 0.0}});
    g.update_default_edge_attributes({{"weight", 2.0}});

    MINI_CHECK(g.default_edge_attributes.size() == 2);
    MINI_CHECK(g.default_edge_attributes["weight"] == 2.0);
    MINI_CHECK(g.default_edge_attributes["stiffness"] == 0.0);
}

MINI_TEST("Graph", "Vertex Attribute") {

    Graph g("g");
    g.add_node("a");
    g.add_node("b");
    g.update_default_vertex_attributes({{"is_support", 0.0}});
    g.set_vertex_attribute("a", "is_support", 1.0);

    MINI_CHECK(g.vertex_attribute("a", "is_support") == 1.0);
    MINI_CHECK(g.vertex_attribute("b", "is_support") == 0.0);
    MINI_CHECK(g.vertex_attribute("a", "missing") == std::nullopt);
    MINI_CHECK(g.vertex_attribute("missing", "is_support") == std::nullopt);
}

MINI_TEST("Graph", "Set Vertex Attribute") {

    Graph g("g");
    g.add_node("a");
    g.set_vertex_attribute("a", "load", -2.5);
    g.set_vertex_attribute("missing", "load", 1.0);

    const std::vector<Vertex> vertices = g.get_vertices();

    MINI_CHECK(vertices[0].attributes.at("load") == -2.5);
    MINI_CHECK(!g.has_node("missing"));
}

MINI_TEST("Graph", "Edge Attribute") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.update_default_edge_attributes({{"weight", 1.0}});

    const std::tuple<std::string, std::string> ab("a", "b");
    const std::tuple<std::string, std::string> ba("b", "a");
    const std::tuple<std::string, std::string> bc("b", "c");
    const std::tuple<std::string, std::string> ac("a", "c");
    g.set_edge_attribute(ab, "weight", 5.0);

    MINI_CHECK(g.edge_attribute(ab, "weight") == 5.0);
    MINI_CHECK(g.edge_attribute(ba, "weight") == 5.0);
    MINI_CHECK(g.edge_attribute(bc, "weight") == 1.0);
    MINI_CHECK(g.edge_attribute(ab, "missing") == std::nullopt);
    MINI_CHECK(g.edge_attribute(ac, "weight") == std::nullopt);
}

MINI_TEST("Graph", "Set Edge Attribute") {

    Graph g("g");
    g.add_edge("a", "b");

    const std::tuple<std::string, std::string> ba("b", "a");
    const std::tuple<std::string, std::string> ac("a", "c");
    g.set_edge_attribute(ba, "weight", 3.0);
    g.set_edge_attribute(ac, "weight", 1.0);
    g.add_edge("a", "b");

    MINI_CHECK(g.edges["a"]["b"].attributes["weight"] == 3.0);
    MINI_CHECK(g.edges["b"]["a"].attributes["weight"] == 3.0);
    MINI_CHECK(!g.has_edge(ac));
}

MINI_TEST("Graph", "Vertices Where") {

    Graph g("g");
    g.add_node("a");
    g.add_node("b");
    g.add_node("c");
    g.update_default_vertex_attributes({{"is_support", 0.0}, {"level", 1.0}});
    g.set_vertex_attribute("a", "is_support", 1.0);
    g.set_vertex_attribute("c", "is_support", 1.0);
    g.set_vertex_attribute("c", "level", 2.0);

    MINI_CHECK(g.vertices_where({{"is_support", 1.0}}) == (std::vector<std::string>{"a", "c"}));
    MINI_CHECK(g.vertices_where({{"is_support", 1.0}, {"level", 1.0}}) == (std::vector<std::string>{"a"}));
    MINI_CHECK(g.vertices_where({{"is_support", 0.0}}) == (std::vector<std::string>{"b"}));
    MINI_CHECK(g.vertices_where({{"missing", 0.0}}).empty());
}

MINI_TEST("Graph", "Edges Where") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "d");
    g.update_default_edge_attributes({{"weight", 0.0}});

    const std::tuple<std::string, std::string> bc("b", "c");
    const std::tuple<std::string, std::string> cd("c", "d");
    const std::tuple<std::string, std::string> dc("d", "c");
    g.set_edge_attribute(bc, "weight", 3.0);
    g.set_edge_attribute(dc, "weight", 3.0);

    const std::vector<std::tuple<std::string, std::string>> heavy = g.edges_where({{"weight", 3.0}});
    const std::vector<std::tuple<std::string, std::string>> light = g.edges_where({{"weight", 0.0}});

    MINI_CHECK(heavy.size() == 2);
    MINI_CHECK(heavy[0] == bc);
    MINI_CHECK(heavy[1] == cd);
    MINI_CHECK(light.size() == 1);
}

MINI_TEST("Graph", "Vertices Where Predicate") {

    Graph g("g");
    g.add_node("a");
    g.add_node("b");
    g.add_node("c");
    g.update_default_vertex_attributes({{"load", 1.0}});
    g.set_vertex_attribute("b", "load", 5.0);
    g.set_vertex_attribute("c", "load", 10.0);

    const std::vector<std::string> heavy = g.vertices_where_predicate(
        [](const std::string&, const std::map<std::string, double>& attributes) {
            return attributes.at("load") > 4.0;
        }
    );

    MINI_CHECK(heavy == (std::vector<std::string>{"b", "c"}));
}

MINI_TEST("Graph", "Edges Where Predicate") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.update_default_edge_attributes({{"weight", 1.0}});

    const std::tuple<std::string, std::string> bc("b", "c");
    g.set_edge_attribute(bc, "weight", 5.0);

    const std::vector<std::tuple<std::string, std::string>> heavy = g.edges_where_predicate(
        [](const std::tuple<std::string, std::string>&, const std::map<std::string, double>& attributes) {
            return attributes.at("weight") > 4.0;
        }
    );

    MINI_CHECK(heavy.size() == 1);
    MINI_CHECK(heavy[0] == bc);
}

MINI_TEST("Graph", "Bfs") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");

    const std::vector<std::string> result = g.bfs("a");

    MINI_CHECK(result == (std::vector<std::string>{"a", "b", "c", "d"}));
}

MINI_TEST("Graph", "Dfs") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");

    const std::vector<std::string> result = g.dfs("a");

    MINI_CHECK(result == (std::vector<std::string>{"a", "b", "c", "d"}));
}

MINI_TEST("Graph", "Connected Components") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");
    g.add_edge("b", "d");
    g.add_edge("e", "f");

    const std::vector<std::vector<std::string>> comps = g.connected_components();

    MINI_CHECK(comps.size() == 2);
    MINI_CHECK(!g.is_connected());
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

    Graph g2("g2");
    g2.add_edge("x", "y");
    g2.add_edge("y", "z");

    MINI_CHECK(g.has_cycle());
    MINI_CHECK(!g2.has_cycle());
}

MINI_TEST("Graph", "Cycle Basis") {

    Graph g("g");
    g.add_edge("a", "b");
    g.add_edge("b", "c");
    g.add_edge("c", "a");

    const std::vector<std::vector<std::string>> cycles = g.cycle_basis();

    MINI_CHECK(cycles.size() == 1);
}

MINI_TEST("Graph", "Take Node") {

    Graph g("g");
    g.add_node("a", "");
    g.add_node("b", "bee");
    g.add_edge("a", "b", "ab");
    g.add_edge("b", "c", "bc");
    g.set_vertex_attribute("b", "load", 2.0);
    g.set_edge_attribute(std::make_tuple("a", "b"), "weight", 3.0);
    const std::string before = g.jsondump().dump();
    const std::pair<Vertex, std::vector<Edge>> taken = *g.take_node("b");
    const Vertex& vertex = taken.first;
    const std::vector<Edge>& edges = taken.second;

    MINI_CHECK(before.find(vertex.guid()) != std::string::npos && vertex.index == 1 && vertex.attribute == "bee");
    MINI_CHECK(vertex.attributes.at("load") == 2.0);
    MINI_CHECK(edges.size() == 2 && std::all_of(edges.begin(), edges.end(), [](const Edge& e) { return e.v0 == "b" || e.v1 == "b"; }));
    MINI_CHECK(std::any_of(edges.begin(), edges.end(), [](const Edge& e) { return e.attributes.count("weight") && e.attributes.at("weight") == 3.0; }));
    MINI_CHECK(!g.has_node("b") && !g.has_edge(std::make_tuple("a", "b")) && !g.has_edge(std::make_tuple("c", "b")));
    MINI_CHECK(g.vertex_count == 3 && g.edge_count == 2 && g.edges.empty());
    MINI_CHECK(g.get_vertices()[0].index == 0 && g.get_vertices()[1].index == 2);
    MINI_CHECK(g.str() == "<Graph with 2 vertices, 0 edges: g>");
    MINI_CHECK(!g.take_node("b"));
}

MINI_TEST("Graph", "Put Node") {

    Graph g("g");
    g.add_edge("a", "b", "ab");
    g.add_edge("b", "c", "bc");
    g.set_vertex_attribute("b", "load", 2.0);
    g.set_edge_attribute(std::make_tuple("b", "c"), "weight", 3.0);
    const std::string before = g.jsondump().dump();
    std::pair<Vertex, std::vector<Edge>> taken = *g.take_node("b");
    g.put_node(taken.first, taken.second);
    const std::string after = g.jsondump().dump();
    const std::string ab = g.edges["a"]["b"].guid();
    taken = *g.take_node("b");
    g.remove_node("c");
    g.add_edge("b", "d", "bd");
    const std::string guid = taken.first.guid();
    g.put_node(taken.first, taken.second);

    MINI_CHECK(before == after);
    MINI_CHECK(g.edges["a"]["b"].guid() == ab && g.edges["b"]["a"].guid() == ab);
    MINI_CHECK(!g.has_edge(std::make_tuple("b", "c")) && g.has_edge(std::make_tuple("b", "d")));
    MINI_CHECK(g.get_vertices()[1].guid() == guid && g.vertex_attribute("b", "load") == 2.0);
    MINI_CHECK(g.number_of_edges() == 2);
}

MINI_TEST("Graph", "Renumber") {

    Graph g("g");
    g.add_edge("a", "b", "");
    g.add_edge("a", "c", "");
    g.add_edge("b", "d", "");
    g.add_edge("c", "e", "");
    g.add_edge("d", "e", "");
    g.add_edge("a", "e", "");
    g.take_node("b");
    g.take_node("d");
    g.renumber();
    std::vector<int> indices;

    for (const Vertex& vertex : g.get_vertices())
        indices.push_back(vertex.index);

    MINI_CHECK((indices == std::vector<int>{0, 1, 2}));
    MINI_CHECK(g.edges["a"]["c"].index == 0 && g.edges["e"]["c"].index == 1);
    MINI_CHECK(g.edges["a"]["e"].index == 2 && g.edges["e"]["a"].index == 2);
    MINI_CHECK(g.vertex_count == g.number_of_vertices() && g.vertex_count == 3);
    MINI_CHECK(g.edge_count == g.number_of_edges() && g.edge_count == 3);
}

}
