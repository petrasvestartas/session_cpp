#pragma once
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <algorithm>
#include <climits>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace session_proto {
class Graph;
}

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Vertex
// ═══════════════════════════════════════════════════════════════════════════
/// A graph vertex with a name, attribute string and integer index.
class Vertex {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_vertex"; // Vertex name, also the key in Graph::vertices.
    std::string attribute = ""; // Vertex attribute data as string.
    std::map<std::string, double> attributes; // Name -> value, overriding the graph defaults.
    int index = -1; // Integer index of the vertex, assigned by Graph.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from name and attribute.
    Vertex(std::string name = "my_vertex", std::string attribute = "")
        : name(name), attribute(attribute) {}

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the GUID, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return a mutable GUID, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Vertex jsonload(const nlohmann::json& data);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Vertex(guid, name, attribute, index)".
    std::string str() const;
};

// ═══════════════════════════════════════════════════════════════════════════
// Edge
// ═══════════════════════════════════════════════════════════════════════════
/// A graph edge connecting two vertices by name.
class Edge {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_edge"; // Edge name.
    std::string v0 = ""; // First vertex name.
    std::string v1 = ""; // Second vertex name.
    std::string attribute = ""; // Edge attribute data as string.
    std::map<std::string, double> attributes; // Name -> value, overriding the graph defaults.
    int index = -1; // Integer index of the edge, assigned by Graph.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from endpoints and attribute.
    Edge(std::string v0 = "", std::string v1 = "", std::string attribute = "")
        : v0(v0), v1(v1), attribute(attribute) {}

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the GUID, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return a mutable GUID, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the (v0, v1) tuple.
    std::tuple<std::string, std::string> vertices() const;

    /// Return whether this edge touches the given vertex.
    bool connects(const std::string& vertex_id) const;

    /// Return the other endpoint given one endpoint, empty if not connected.
    std::string other_vertex(const std::string& vertex_id) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Edge jsonload(const nlohmann::json& data);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Edge(guid, name, v0, v1, attribute)".
    std::string str() const;
};

// ═══════════════════════════════════════════════════════════════════════════
// Graph
// ═══════════════════════════════════════════════════════════════════════════
/// An undirected graph with string vertices, string labels and double attributes.
class Graph {
private:
    mutable std::string _guid; // Lazily minted GUID.
    std::map<std::string, Vertex> vertices; // name -> Vertex.

public:
    std::string name = "my_graph"; // Graph name.
    int vertex_count = 0; // Next available vertex index.
    int edge_count = 0; // Next available edge index.
    std::map<std::string, std::map<std::string, Edge>> edges; // node_name -> {neighbor_name -> Edge}, every edge stored in both directions.
    std::map<std::string, double> default_vertex_attributes; // Vertex attribute defaults.
    std::map<std::string, double> default_edge_attributes; // Edge attribute defaults.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from name.
    Graph(std::string name = "my_graph") : name(name) {}

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the GUID, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return a mutable GUID, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether a node with the given key exists.
    bool has_node(const std::string& key) const;

    /// Return whether an edge between the given endpoints exists.
    bool has_edge(const std::tuple<std::string, std::string>& key) const;

    /// Add a node and return its key.
    std::string add_node(const std::string& key, const std::string& attribute = "");

    /// Add an edge between u and v, creating missing nodes, and return (u, v).
    std::tuple<std::string, std::string> add_edge(const std::string& u, const std::string& v, const std::string& attribute = "");

    /// Remove a node and all its edges.
    void remove_node(const std::string& key);

    /// Remove an edge, keeping its nodes.
    void remove_edge(const std::tuple<std::string, std::string>& edge);

    /// Return all vertices in the graph.
    std::vector<Vertex> get_vertices() const;

    /// Return all edges in the graph as (u, v) tuples, each once.
    std::vector<std::tuple<std::string, std::string>> get_edges() const;

    /// Return all neighbors of a node.
    std::vector<std::string> neighbors(const std::string& node) const;

    /// Return incident edges as (other, attribute, forward); forward when node is the edge's v0.
    std::vector<std::tuple<std::string, std::string, bool>> edges_of(const std::string& node) const;

    /// Return the number of vertices in the graph.
    int number_of_vertices() const;

    /// Return the number of edges in the graph.
    int number_of_edges() const;

    /// Remove all vertices and edges.
    void clear();

    /// Get or set a node label (sets if value is non-empty).
    std::string node_label(const std::string& node, const std::string& value = "");

    /// Get or set an edge label (sets if value is non-empty).
    std::string edge_label(const std::string& u, const std::string& v, const std::string& value = "");

    // ═══════════════════════════════════════════════════════════════════════════
    // Attribute API
    // ═══════════════════════════════════════════════════════════════════════════
    /// Merge attrs into the default vertex attributes.
    void update_default_vertex_attributes(const std::vector<std::pair<std::string, double>>& attrs);

    /// Merge attrs into the default edge attributes.
    void update_default_edge_attributes(const std::vector<std::pair<std::string, double>>& attrs);

    /// Return the attribute of a vertex, falling back to the default; nullopt when neither exists.
    std::optional<double> vertex_attribute(const std::string& key, const std::string& name) const;

    /// Store an attribute on a vertex.
    void set_vertex_attribute(const std::string& key, const std::string& name, double value);

    /// Return the attribute of an edge, falling back to the default; nullopt when neither exists.
    std::optional<double> edge_attribute(const std::tuple<std::string, std::string>& edge, const std::string& name) const;

    /// Store an attribute on an edge, in both stored directions.
    void set_edge_attribute(const std::tuple<std::string, std::string>& edge, const std::string& name, double value);

    /// Return the vertices whose attributes match every (name, value) condition.
    std::vector<std::string> vertices_where(const std::vector<std::pair<std::string, double>>& conditions) const;

    /// Return the edges whose attributes match every (name, value) condition.
    std::vector<std::tuple<std::string, std::string>> edges_where(
        const std::vector<std::pair<std::string, double>>& conditions
    ) const;

    /// Return the vertices for which pred(key, attributes) is true.
    std::vector<std::string> vertices_where_predicate(
        const std::function<bool(const std::string&, const std::map<std::string, double>&)>& pred
    ) const;

    /// Return the edges for which pred(edge, attributes) is true.
    std::vector<std::tuple<std::string, std::string>> edges_where_predicate(
        const std::function<bool(const std::tuple<std::string, std::string>&, const std::map<std::string, double>&)>& pred
    ) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Algorithms
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the breadth-first order from start.
    std::vector<std::string> bfs(const std::string& start) const;

    /// Return the depth-first order from start.
    std::vector<std::string> dfs(const std::string& start) const;

    /// Return the connected components as sorted node name lists.
    std::vector<std::vector<std::string>> connected_components() const;

    /// Return whether the graph has at most one connected component.
    bool is_connected() const;

    /// Return the number of connected components.
    int number_connected_components() const;

    /// Return the shortest path between u and v, empty if disconnected.
    std::vector<std::string> shortest_path(const std::string& u, const std::string& v) const;

    /// Return the length of the shortest path between u and v, -1 if disconnected.
    int shortest_path_length(const std::string& u, const std::string& v) const;

    /// Return whether the graph contains a cycle.
    bool has_cycle() const;

    /// Return a basis of fundamental cycles.
    std::vector<std::vector<std::string>> cycle_basis() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Graph jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Graph file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Graph file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message, each edge once.
    session_proto::Graph to_proto() const;

    /// Construct from the protobuf message.
    static Graph from_proto(const session_proto::Graph& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Graph pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Graph pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// "<Graph with V vertices, E edges: name>"
    std::string str() const;

    /// "Graph(guid, name, vertex_count, edge_count)"
    std::string repr() const;

private:
    /// Renumber vertex indices 0, 1, 2, ... keeping their relative order.
    void _reassign_indices();

    /// Renumber edge indices 0, 1, 2, ... keeping their relative order.
    void _reassign_edge_indices();
};

// ═══════════════════════════════════════════════════════════════════════════
// Stream operators
// ═══════════════════════════════════════════════════════════════════════════
/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const Vertex& vertex);

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const Edge& edge);

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const Graph& graph);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Vertex> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::Vertex& vertex, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", vertex.str());
    }
};

template <> struct fmt::formatter<session_cpp::Edge> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::Edge& edge, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", edge.str());
    }
};

template <> struct fmt::formatter<session_cpp::Graph> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::Graph& graph, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", graph.str());
    }
};
