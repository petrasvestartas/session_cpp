#pragma once
#include "fmt/core.h" // for fmt::formatter specializations in this header
#include "guid.h"     // for ::guid() used in in-class member initializers
#include "json.h"     // for nlohmann::json types used in function signatures
#include <array>
#include <map>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>
// Implementation-only headers required by this translation unit
#include <algorithm> // std::sort
#include <climits>   // INT_MAX
#include <deque>     // std::deque
#include <fstream>   // std::ifstream, std::ofstream
#include <set>       // std::set
#include <stdexcept> // std::runtime_error
namespace session_cpp {

/**
 * @class Vertex
 * @brief A graph vertex with a name, attribute string, and integer index.
 */
class Vertex {
public:
  /// Vertex name (also used as key in Graph::vertices)
  std::string name = "my_vertex";

  /// Lazy GUID accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy GUID accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Vertex attribute data as string
  std::string attribute = "";

  /// Integer index of the vertex (assigned by Graph)
  int index = -1;

  /// Default / named constructor
  Vertex(std::string name = "my_vertex", std::string attribute = "")
      : name(name), attribute(attribute) {}

  /// Simple string form (like Python __str__)
  std::string str() const;

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Vertex jsonload(const nlohmann::json &data);

private:
  mutable std::string _guid; ///< Lazily generated unique identifier
};

/**
 * @class Edge
 * @brief A graph edge connecting two vertices by name.
 */
class Edge {
public:
  /// Edge name
  std::string name = "my_edge";

  /// Lazy GUID accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy GUID accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// First vertex name
  std::string v0 = "";

  /// Second vertex name
  std::string v1 = "";

  /// Edge attribute data as string
  std::string attribute = "";

  /// Integer index of the edge (assigned by Graph)
  int index = -1;

  /// Default / parameterized constructor
  Edge(std::string v0 = "", std::string v1 = "", std::string attribute = "")
      : v0(v0), v1(v1), attribute(attribute) {}

  /// Simple string form (like Python __str__)
  std::string str() const;

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Edge jsonload(const nlohmann::json &data);

  /// Return (v0, v1) tuple
  std::tuple<std::string, std::string> vertices() const;

  /// True if this edge connects the given vertex
  bool connects(const std::string &vertex_id);

  /// Return the other endpoint of an edge given one endpoint
  std::string other_vertex(const std::string &vertex_id);

private:
  mutable std::string _guid; ///< Lazily generated unique identifier
};

/**
 * @class Graph
 * @brief A graph data structure with string-only vertices and attributes.
 */
class Graph {
private:
  std::map<std::string, Vertex> vertices; ///< node_name -> Vertex object

public:
  /// node_name -> {neighbor_name -> Edge object}
  std::map<std::string, std::map<std::string, Edge>> edges;

  /// Graph identifier/name
  std::string name = "my_graph";

  /// Lazy GUID accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy GUID accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Track next available vertex index
  int vertex_count = 0;

  /// Track next available edge index
  int edge_count = 0;

  /// Default / named constructor
  Graph(std::string name = "my_graph")
      : name(name), vertex_count(0), edge_count(0) {}

  /// Simple string form (like Python __str__)
  std::string str() const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Graph jsonload(const nlohmann::json &data);

  /// Convert to JSON string
  std::string json_dumps() const;

  /// Load from JSON string
  static Graph json_loads(const std::string &json_string);

  /// Write JSON to file
  void json_dump(const std::string &filename) const;

  /// Read JSON from file
  static Graph json_load(const std::string &filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Protobuf
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to protobuf binary string
  std::string pb_dumps() const;

  /// Load from protobuf binary string
  static Graph pb_loads(const std::string &data);

  /// Write protobuf to file
  void pb_dump(const std::string &filename) const;

  /// Read protobuf from file
  static Graph pb_load(const std::string &filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// True if a node with the given key exists
  bool has_node(const std::string &key);

  /// True if an edge between the given endpoints exists
  bool has_edge(const std::tuple<std::string, std::string> &key);

  /// Add a node and return its key
  std::string add_node(const std::string &key,
                       const std::string &attribute = "");

  /// Add an edge between u and v and return the edge tuple
  std::tuple<std::string, std::string>
  add_edge(const std::string &u, const std::string &v,
           const std::string &attribute = "");

  /// Remove a node and all its edges
  void remove_node(const std::string &key);

  /// Remove an edge from the graph
  void remove_edge(const std::tuple<std::string, std::string> &edge);

private:
  mutable std::string _guid;

  /// Reassign vertex indices to maintain contiguous sequence 0, 1, 2, ...
  void _reassign_indices();

  /// Reassign edge indices to maintain contiguous sequence 0, 1, 2, ...
  void _reassign_edge_indices();

public:
  /// Get all vertices in the graph
  std::vector<Vertex> get_vertices();

  /// Get all edges in the graph as (u, v) tuples
  std::vector<std::tuple<std::string, std::string>> get_edges();

  /// All neighbors of a node
  std::vector<std::string> neighbors(const std::string &node);

  /// Alias for neighbors()
  std::vector<std::string> get_neighbors(const std::string &node);

  /// Number of vertices in the graph
  int number_of_vertices() const;

  /// Number of edges in the graph
  int number_of_edges() const;

  /// Remove all vertices and edges
  void clear();

  /// Get or set node attribute (sets if value is non-empty)
  std::string node_attribute(const std::string &node,
                             const std::string &value = "");

  /// Get or set edge attribute (sets if value is non-empty)
  std::string edge_attribute(const std::string &u, const std::string &v,
                             const std::string &value = "");

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Algorithms
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Breadth-first search starting at start
  std::vector<std::string> bfs(const std::string &start);

  /// Depth-first search starting at start
  std::vector<std::string> dfs(const std::string &start);

  /// Connected components as vectors of node names
  std::vector<std::vector<std::string>> connected_components();

  /// True if the graph has exactly one connected component
  bool is_connected();

  /// Number of connected components
  int number_connected_components();

  /// Shortest path between u and v (empty if disconnected)
  std::vector<std::string> shortest_path(const std::string &u,
                                         const std::string &v);

  /// Length of the shortest path between u and v (-1 if disconnected)
  int shortest_path_length(const std::string &u, const std::string &v);

  /// True if the graph contains a cycle
  bool has_cycle();

  /// A basis of fundamental cycles in the graph
  std::vector<std::vector<std::string>> cycle_basis();
};

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operators
///////////////////////////////////////////////////////////////////////////////////////////

/// Stream output operator for vertex
std::ostream &operator<<(std::ostream &os, const Vertex &vertex);

/// Stream output operator for edge
std::ostream &operator<<(std::ostream &os, const Edge &edge);

/// Stream output operator for graph
std::ostream &operator<<(std::ostream &os, const Graph &graph);
} // namespace session_cpp

// fmt formatter specialization for Vertex - enables direct fmt::print(vertex)
template <> struct fmt::formatter<session_cpp::Vertex> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Vertex &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};

// fmt formatter specialization for Edge - enables direct fmt::print(edge)
template <> struct fmt::formatter<session_cpp::Edge> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Edge &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};

// fmt formatter specialization for Graph - enables direct fmt::print(graph)
template <> struct fmt::formatter<session_cpp::Graph> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Graph &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
