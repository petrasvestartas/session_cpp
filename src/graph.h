#pragma once
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <algorithm>
#include <climits>
#include <deque>
#include <fstream>
#include <map>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Vertex
// ═══════════════════════════════════════════════════════════════════════════

/// A graph vertex with a name, attribute string and integer index
class Vertex {
public:
  /// Vertex name, also the key in Graph::vertices
  std::string name = "my_vertex";

  /// Vertex attribute data as string
  std::string attribute = "";

  /// Integer index of the vertex, assigned by Graph
  int index = -1;

  /// Construct from name and attribute
  Vertex(std::string name = "my_vertex", std::string attribute = "")
      : name(name), attribute(attribute) {}

  /// True once a guid has been minted; asking guid() mints one
  bool has_guid() const { return !_guid.empty(); }

  /// Lazy guid accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy guid accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Vertex jsonload(const nlohmann::json &data);

  /// "Vertex(guid, name, attribute, index)"
  std::string str() const;

private:
  mutable std::string _guid;
};

// ═══════════════════════════════════════════════════════════════════════════
// Edge
// ═══════════════════════════════════════════════════════════════════════════

/// A graph edge connecting two vertices by name
class Edge {
public:
  /// Edge name
  std::string name = "my_edge";

  /// First vertex name
  std::string v0 = "";

  /// Second vertex name
  std::string v1 = "";

  /// Edge attribute data as string
  std::string attribute = "";

  /// Integer index of the edge, assigned by Graph
  int index = -1;

  /// Construct from endpoints and attribute
  Edge(std::string v0 = "", std::string v1 = "", std::string attribute = "")
      : v0(v0), v1(v1), attribute(attribute) {}

  /// True once a guid has been minted; asking guid() mints one
  bool has_guid() const { return !_guid.empty(); }

  /// Lazy guid accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy guid accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// The (v0, v1) tuple
  std::tuple<std::string, std::string> vertices() const;

  /// True if this edge touches the given vertex
  bool connects(const std::string &vertex_id) const;

  /// The other endpoint given one endpoint, empty if not connected
  std::string other_vertex(const std::string &vertex_id) const;

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Edge jsonload(const nlohmann::json &data);

  /// "Edge(guid, name, v0, v1, attribute)"
  std::string str() const;

private:
  mutable std::string _guid;
};

// ═══════════════════════════════════════════════════════════════════════════
// Graph
// ═══════════════════════════════════════════════════════════════════════════

/// An undirected graph with string vertices and string attributes
class Graph {
public:
  /// node_name -> {neighbor_name -> Edge}, every edge stored in both directions
  std::map<std::string, std::map<std::string, Edge>> edges;

  /// Graph name
  std::string name = "my_graph";

  /// Next available vertex index
  int vertex_count = 0;

  /// Next available edge index
  int edge_count = 0;

  /// Construct from name
  Graph(std::string name = "my_graph") : name(name) {}

  /// True once a guid has been minted; asking guid() mints one
  bool has_guid() const { return !_guid.empty(); }

  /// Lazy guid accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy guid accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Details
  // ═══════════════════════════════════════════════════════════════════════════

  /// True if a node with the given key exists
  bool has_node(const std::string &key) const;

  /// True if an edge between the given endpoints exists
  bool has_edge(const std::tuple<std::string, std::string> &key) const;

  /// Add a node and return its key
  std::string add_node(const std::string &key, const std::string &attribute = "");

  /// Add an edge between u and v, creating missing nodes, and return (u, v)
  std::tuple<std::string, std::string> add_edge(const std::string &u, const std::string &v, const std::string &attribute = "");

  /// Remove a node and all its edges
  void remove_node(const std::string &key);

  /// Remove an edge, keeping its nodes
  void remove_edge(const std::tuple<std::string, std::string> &edge);

  /// All vertices in the graph
  std::vector<Vertex> get_vertices() const;

  /// All edges in the graph as (u, v) tuples, each once
  std::vector<std::tuple<std::string, std::string>> get_edges() const;

  /// All neighbors of a node
  std::vector<std::string> neighbors(const std::string &node) const;

  /// Alias for neighbors()
  std::vector<std::string> get_neighbors(const std::string &node) const;

  /// Incident edges as (other, attribute, forward); forward when node is the edge's v0
  std::vector<std::tuple<std::string, std::string, bool>> edges_of(const std::string &node) const;

  /// Number of vertices in the graph
  int number_of_vertices() const;

  /// Number of edges in the graph
  int number_of_edges() const;

  /// Remove all vertices and edges
  void clear();

  /// Get or set node attribute (sets if value is non-empty)
  std::string node_attribute(const std::string &node, const std::string &value = "");

  /// Get or set edge attribute (sets if value is non-empty)
  std::string edge_attribute(const std::string &u, const std::string &v, const std::string &value = "");

  // ═══════════════════════════════════════════════════════════════════════════
  // Algorithms
  // ═══════════════════════════════════════════════════════════════════════════

  /// Breadth-first order from start
  std::vector<std::string> bfs(const std::string &start) const;

  /// Depth-first order from start
  std::vector<std::string> dfs(const std::string &start) const;

  /// Connected components as sorted node name lists
  std::vector<std::vector<std::string>> connected_components() const;

  /// True if the graph has at most one connected component
  bool is_connected() const;

  /// Number of connected components
  int number_connected_components() const;

  /// Shortest path between u and v, empty if disconnected
  std::vector<std::string> shortest_path(const std::string &u, const std::string &v) const;

  /// Length of the shortest path between u and v, -1 if disconnected
  int shortest_path_length(const std::string &u, const std::string &v) const;

  /// True if the graph contains a cycle
  bool has_cycle() const;

  /// A basis of fundamental cycles
  std::vector<std::vector<std::string>> cycle_basis() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Graph jsonload(const nlohmann::json &data);

  /// Convert to JSON string
  std::string file_json_dumps() const;

  /// Load from JSON string
  static Graph file_json_loads(const std::string &json_string);

  /// Write JSON to file
  void file_json_dump(const std::string &filename) const;

  /// Read JSON from file
  static Graph file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  /// Convert to protobuf binary string
  std::string pb_dumps() const;

  /// Load from protobuf binary string
  static Graph pb_loads(const std::string &data);

  /// Write protobuf to file
  void pb_dump(const std::string &filename) const;

  /// Read protobuf from file
  static Graph pb_load(const std::string &filename);

  /// "Graph(guid, name, vertex_count, edge_count)"
  std::string str() const;

private:
  std::map<std::string, Vertex> vertices;
  mutable std::string _guid;

  /// Renumber vertex indices 0, 1, 2, ... keeping their relative order
  void _reassign_indices();

  /// Renumber edge indices 0, 1, 2, ... keeping their relative order
  void _reassign_edge_indices();
};

// ═══════════════════════════════════════════════════════════════════════════
// Stream operators
// ═══════════════════════════════════════════════════════════════════════════

std::ostream &operator<<(std::ostream &os, const Vertex &vertex);
std::ostream &operator<<(std::ostream &os, const Edge &edge);
std::ostream &operator<<(std::ostream &os, const Graph &graph);
} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Vertex> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Vertex &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};

template <> struct fmt::formatter<session_cpp::Edge> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Edge &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};

template <> struct fmt::formatter<session_cpp::Graph> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Graph &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
