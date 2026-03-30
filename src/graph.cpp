#include "graph.h"
#include "graph.pb.h"
#include "vertex.pb.h"
#include "edge.pb.h"

namespace session_cpp {

std::string Vertex::str() const {
  return fmt::format("Vertex({}, {}, {}, {})", guid(), name, attribute, index);
}

nlohmann::ordered_json Vertex::jsondump() const {
  return nlohmann::ordered_json{{"type", "Vertex"},
                                {"name", name},
                                {"guid", guid()},
                                {"attribute", attribute},
                                {"index", index}};
}

Vertex Vertex::jsonload(const nlohmann::json &data) {
  Vertex vertex(data["name"], data["attribute"]);
  vertex.index = data["index"];
  vertex.guid() = data["guid"];
  return vertex;
}

std::string Edge::str() const {
  return fmt::format("Edge({}, {}, {}, {}, {})", guid(), name, v0, v1, attribute);
}

nlohmann::ordered_json Edge::jsondump() const {
  return nlohmann::ordered_json{{"type", "Edge"},
                                {"name", name},
                                {"guid", guid()},
                                {"v0", v0},
                                {"v1", v1},
                                {"attribute", attribute},
                                {"index", index}};
}

Edge Edge::jsonload(const nlohmann::json &data) {
  Edge edge(data["v0"], data["v1"], data["attribute"]);
  edge.name = data["name"];
  edge.guid() = data["guid"];
  edge.index = data["index"];
  return edge;
}

std::tuple<std::string, std::string> Edge::vertices() const {
  return std::make_tuple(v0, v1);
}

bool Edge::connects(const std::string &vertex_id) {
  return v0 == vertex_id || v1 == vertex_id;
}

std::string Edge::other_vertex(const std::string &vertex_id) {
  if (v0 == vertex_id)
    return v1;
  else if (v1 == vertex_id)
    return v0;
  return "";
}

std::string Graph::str() const {
  return fmt::format("Graph({}, {}, {}, {})", guid(), name, vertex_count,
                     edge_count);
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Graph::jsondump() const {
  // Convert vertices to JSON array
  nlohmann::ordered_json vertices_json = nlohmann::ordered_json::array();
  for (auto &[vertex_name, vertex] : vertices) {
    vertices_json.push_back(vertex.jsondump());
  }

  // Convert edges to JSON array (store each edge only once)
  nlohmann::ordered_json edges_json = nlohmann::ordered_json::array();
  for (auto &[u, neighbors] : edges) {
    for (auto &[v, edge] : neighbors) {
      if (u < v) { // Only store each edge once
        edges_json.push_back(edge.jsondump());
      }
    }
  }

  return nlohmann::ordered_json{{"type", "Graph"},
                                {"name", name},
                                {"guid", guid()},
                                {"vertices", vertices_json},
                                {"edges", edges_json},
                                {"vertex_count", vertex_count},
                                {"edge_count", edge_count}};
}

Graph Graph::jsonload(const nlohmann::json &data) {
  Graph graph(data["name"]);
  graph.guid() = data["guid"];
  graph.vertex_count = data["vertex_count"];
  graph.edge_count = data["edge_count"];

  // Restore vertices
  for (const auto &vertex_data : data["vertices"]) {
    Vertex vertex = Vertex::jsonload(vertex_data);
    graph.vertices[vertex.name] = vertex;
  }

  // Restore edges
  for (const auto &edge_data : data["edges"]) {
    Edge edge = Edge::jsonload(edge_data);
    std::string u = edge.v0;
    std::string v = edge.v1;
    if (graph.edges.find(u) == graph.edges.end()) {
      graph.edges[u] = std::map<std::string, Edge>();
    }
    if (graph.edges.find(v) == graph.edges.end()) {
      graph.edges[v] = std::map<std::string, Edge>();
    }
    graph.edges[u][v] = edge;
    graph.edges[v][u] = edge;
  }

  return graph;
}

std::string Graph::json_dumps() const {
  return jsondump().dump();
}

Graph Graph::json_loads(const std::string &json_string) {
  return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Graph::json_dump(const std::string &filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

Graph Graph::json_load(const std::string &filename) {
  std::ifstream file(filename);
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

std::string Graph::pb_dumps() const {
  session_proto::Graph proto;
  proto.set_name(name);
  proto.set_guid(guid());
  proto.set_vertex_count(vertex_count);
  proto.set_edge_count(edge_count);

  for (const auto &[vname, vertex] : vertices) {
    auto &v = (*proto.mutable_vertices())[vname];
    v.set_name(vertex.name);
    v.set_guid(vertex.guid());
    v.set_attribute(vertex.attribute);
    v.set_index(vertex.index);
  }

  std::set<std::pair<std::string, std::string>> seen;
  for (const auto &[u, neighbors] : edges) {
    for (const auto &[v, edge] : neighbors) {
      auto key = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
      if (seen.find(key) == seen.end()) {
        seen.insert(key);
        auto *e = proto.add_edges();
        e->set_guid(edge.guid());
        e->set_name(edge.name);
        e->set_v0(edge.v0);
        e->set_v1(edge.v1);
        e->set_attribute(edge.attribute);
        e->set_index(edge.index);
      }
    }
  }

  return proto.SerializeAsString();
}

Graph Graph::pb_loads(const std::string &data) {
  session_proto::Graph proto;
  proto.ParseFromString(data);

  Graph graph(proto.name());
  graph.guid() = proto.guid();
  graph.vertex_count = proto.vertex_count();
  graph.edge_count = proto.edge_count();

  for (const auto &[vname, v] : proto.vertices()) {
    Vertex vertex(v.name(), v.attribute());
    vertex.guid() = v.guid();
    vertex.index = v.index();
    graph.vertices[vname] = vertex;
  }

  for (const auto &e : proto.edges()) {
    Edge edge(e.v0(), e.v1(), e.attribute());
    edge.guid() = e.guid();
    edge.name = e.name();
    edge.index = e.index();
    graph.edges[e.v0()][e.v1()] = edge;
    graph.edges[e.v1()][e.v0()] = edge;
  }

  return graph;
}

void Graph::pb_dump(const std::string &filename) const {
  std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Graph Graph::pb_load(const std::string &filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////

bool Graph::has_node(const std::string &key) {
  return vertices.find(key) != vertices.end();
}
bool Graph::has_edge(const std::tuple<std::string, std::string> &key) {
  auto u_it = edges.find(std::get<0>(key));
  if (u_it == edges.end()) {
    return false;
  }
  return u_it->second.find(std::get<1>(key)) != u_it->second.end();
}

std::string Graph::add_node(const std::string &key,
                            const std::string &attribute) {
  if (this->has_node(key)) {
    return vertices[key].name;
  } else {
    // Create new vertex
    Vertex vertex(key, attribute);
    vertex.index = vertex_count; // Set index internally
    vertices[key] = vertex;
    vertex_count += 1;
    return vertex.name;
  }
}

std::tuple<std::string, std::string>
Graph::add_edge(const std::string &u, const std::string &v,
                const std::string &attribute) {
  // Add vertices if they don't exist
  if (!this->has_node(u)) {
    this->add_node(u);
  }
  if (!this->has_node(v)) {
    this->add_node(v);
  }

  // Add edge (store in both directions for undirected graph)
  Edge edge(u, v, attribute);
  edge.index = edge_count; // Set index internally
  if (edges.find(u) == edges.end()) {
    edges[u] = std::map<std::string, Edge>();
  }
  if (edges.find(v) == edges.end()) {
    edges[v] = std::map<std::string, Edge>();
  }
  edges[u][v] = edge;
  edges[v][u] = edge;
  edge_count += 1;

  return std::make_tuple(u, v);
}

void Graph::remove_node(const std::string &key) {
  if (!this->has_node(key)) {
    throw std::runtime_error("Node " + key + " not in graph");
  }

  // Remove all edges connected to this node
  auto edges_it = edges.find(key);
  if (edges_it != edges.end()) {
    // Get all neighbors and remove this node from their adjacency lists
    for (const auto &[neighbor, edge] : edges_it->second) {
      auto neighbor_it = edges.find(neighbor);
      if (neighbor_it != edges.end()) {
        neighbor_it->second.erase(key);
      }
    }
    // Remove this node's adjacency list
    edges.erase(edges_it);
  }

  // Remove the node itself
  vertices.erase(key);

  // Reassign indices to maintain contiguous sequence
  _reassign_indices();
}

void Graph::remove_edge(const std::tuple<std::string, std::string> &edge) {
  std::string u = std::get<0>(edge);
  std::string v = std::get<1>(edge);

  if (this->has_edge(edge)) {
    // Remove edge from u's adjacency list
    auto u_it = edges.find(u);
    if (u_it != edges.end() && u_it->second.find(v) != u_it->second.end()) {
      u_it->second.erase(v);
    }

    // Remove edge from v's adjacency list
    auto v_it = edges.find(v);
    if (v_it != edges.end() && v_it->second.find(u) != v_it->second.end()) {
      v_it->second.erase(u);
    }

    // Reassign edge indices to maintain contiguous sequence
    _reassign_edge_indices();
  }
}

void Graph::_reassign_indices() {
  std::vector<Vertex *> vertex_list;
  // Collect all vertices
  for (auto &[vertex_name, vertex] : this->vertices) {
    vertex_list.push_back(&vertex);
  }

  // Sort by current index to maintain relative order
  std::sort(vertex_list.begin(), vertex_list.end(),
            [](const Vertex *a, const Vertex *b) {
              int a_index = (a->index != -1) ? a->index : INT_MAX;
              int b_index = (b->index != -1) ? b->index : INT_MAX;
              return a_index < b_index;
            });

  // Reassign indices
  for (size_t i = 0; i < vertex_list.size(); ++i) {
    vertex_list[i]->index = static_cast<int>(i);
  }

  vertex_count = static_cast<int>(vertex_list.size());
}

void Graph::_reassign_edge_indices() {
  std::vector<Edge *> edge_list;
  std::set<std::pair<std::string, std::string>> seen;

  // Collect all unique edges
  for (auto &[u, neighbors] : this->edges) {
    for (auto &[v, edge] : neighbors) {
      std::pair<std::string, std::string> edge_tuple =
          (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
      if (seen.find(edge_tuple) == seen.end()) {
        seen.insert(edge_tuple);
        edge_list.push_back(&edge);
      }
    }
  }

  // Sort by current index to maintain relative order
  std::sort(edge_list.begin(), edge_list.end(),
            [](const Edge *a, const Edge *b) {
              int a_index = (a->index != -1) ? a->index : INT_MAX;
              int b_index = (b->index != -1) ? b->index : INT_MAX;
              return a_index < b_index;
            });

  // Reassign indices
  for (size_t i = 0; i < edge_list.size(); ++i) {
    edge_list[i]->index = static_cast<int>(i);
  }

  // Update the indices in the main edges map
  for (auto &[u, neighbors] : this->edges) {
    for (auto &[v, edge] : neighbors) {
      // Find the updated edge in the sorted list to get its new index
      for (const auto &sorted_edge : edge_list) {
        if (sorted_edge->guid() == edge.guid()) {
          edge.index = sorted_edge->index;
          break;
        }
      }
    }
  }

  edge_count = static_cast<int>(edge_list.size());
}

std::vector<Vertex> Graph::get_vertices() {
  std::vector<Vertex> result;
  for (auto const &[vertex_name, vertex] : this->vertices) {
    result.push_back(vertex);
  }
  return result;
}

std::vector<std::tuple<std::string, std::string>> Graph::get_edges() {
  std::vector<std::tuple<std::string, std::string>> result;
  std::set<std::pair<std::string, std::string>> seen;

  for (const auto &[u, neighbors] : edges) {
    for (const auto &[v, edge] : neighbors) {
      std::pair<std::string, std::string> edge_tuple =
          (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
      if (seen.find(edge_tuple) == seen.end()) {
        seen.insert(edge_tuple);
        result.push_back(std::make_tuple(edge_tuple.first, edge_tuple.second));
      }
    }
  }

  return result;
}

std::vector<std::string> Graph::neighbors(const std::string &node) {
  if (!has_node(node)) {
    throw std::runtime_error("Node " + node + " not in graph");
  }
  std::vector<std::string> result;
  auto it = edges.find(node);
  if (it != edges.end()) {
    for (const auto &[neighbor, edge] : it->second) {
      result.push_back(neighbor);
    }
  }
  return result;
}

std::vector<std::string> Graph::get_neighbors(const std::string &node) {
  return neighbors(node);
}

int Graph::number_of_vertices() const {
  return static_cast<int>(vertices.size());
}

int Graph::number_of_edges() const {
  // Count unique undirected edges similarly to edges()
  std::set<std::pair<std::string, std::string>> seen;
  for (const auto &[u, neighbors] : edges) {
    for (const auto &[v, edge] : neighbors) {
      auto edge_tuple = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
      seen.insert(edge_tuple);
    }
  }
  return static_cast<int>(seen.size());
}

void Graph::clear() {
  vertices.clear();
  edges.clear();
  vertex_count = 0;
  edge_count = 0;
}

std::string Graph::node_attribute(const std::string &node,
                                  const std::string &value) {
  if (!has_node(node)) {
    throw std::runtime_error("Node " + node + " not in graph");
  }

  if (value == "") {
    return vertices[node].attribute;
  } else {
    vertices[node].attribute = value;
    return value;
  }
}

std::string Graph::edge_attribute(const std::string &u, const std::string &v,
                                  const std::string &value) {
  // Check if edge exists in either direction
  bool has_u_v =
      edges.find(u) != edges.end() && edges[u].find(v) != edges[u].end();
  bool has_v_u =
      edges.find(v) != edges.end() && edges[v].find(u) != edges[v].end();

  if (!has_u_v && !has_v_u) {
    throw std::runtime_error("Edge (" + u + ", " + v + ") not in graph");
  }

  // Get the edge object (exists in at least one direction)
  Edge &edge = has_u_v ? edges[u][v] : edges[v][u];

  if (value.empty()) {
    return edge.attribute;
  } else {
    // Update attribute in both directions
    edge.attribute = value;
    if (has_u_v)
      edges[u][v].attribute = value;
    if (has_v_u)
      edges[v][u].attribute = value;
    return value;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Not class methods
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Algorithms
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> Graph::bfs(const std::string &start) {
  if (!has_node(start)) return {};
  std::set<std::string> visited;
  std::deque<std::string> queue;
  queue.push_back(start);
  visited.insert(start);
  std::vector<std::string> result;
  while (!queue.empty()) {
    std::string node = queue.front();
    queue.pop_front();
    result.push_back(node);
    auto nbrs = get_neighbors(node);
    std::sort(nbrs.begin(), nbrs.end());
    for (const auto &neighbor : nbrs) {
      if (!visited.count(neighbor)) {
        visited.insert(neighbor);
        queue.push_back(neighbor);
      }
    }
  }
  return result;
}

std::vector<std::string> Graph::dfs(const std::string &start) {
  if (!has_node(start)) return {};
  std::set<std::string> visited;
  std::vector<std::string> result;
  std::vector<std::string> stack = {start};
  while (!stack.empty()) {
    std::string node = stack.back();
    stack.pop_back();
    if (visited.count(node)) continue;
    visited.insert(node);
    result.push_back(node);
    auto nbrs = get_neighbors(node);
    std::sort(nbrs.begin(), nbrs.end());
    for (auto it = nbrs.rbegin(); it != nbrs.rend(); ++it) {
      if (!visited.count(*it)) {
        stack.push_back(*it);
      }
    }
  }
  return result;
}

std::vector<std::vector<std::string>> Graph::connected_components() {
  std::set<std::string> visited;
  std::vector<std::vector<std::string>> comps;
  for (const auto &[vname, vertex] : vertices) {
    if (visited.count(vname)) continue;
    auto comp = bfs(vname);
    for (const auto &n : comp) visited.insert(n);
    std::sort(comp.begin(), comp.end());
    comps.push_back(comp);
  }
  return comps;
}

bool Graph::is_connected() {
  return connected_components().size() <= 1;
}

int Graph::number_connected_components() {
  return static_cast<int>(connected_components().size());
}

std::vector<std::string> Graph::shortest_path(const std::string &u,
                                               const std::string &v) {
  if (!has_node(u) || !has_node(v)) return {};
  if (u == v) return {u};
  std::map<std::string, std::string> parent;
  parent[u] = "";
  std::deque<std::string> queue;
  queue.push_back(u);
  while (!queue.empty()) {
    std::string node = queue.front();
    queue.pop_front();
    auto nbrs = get_neighbors(node);
    std::sort(nbrs.begin(), nbrs.end());
    for (const auto &neighbor : nbrs) {
      if (!parent.count(neighbor)) {
        parent[neighbor] = node;
        if (neighbor == v) {
          std::vector<std::string> path;
          std::string cur = v;
          while (cur != u) {
            path.push_back(cur);
            cur = parent[cur];
          }
          path.push_back(u);
          std::reverse(path.begin(), path.end());
          return path;
        }
        queue.push_back(neighbor);
      }
    }
  }
  return {};
}

int Graph::shortest_path_length(const std::string &u, const std::string &v) {
  auto path = shortest_path(u, v);
  if (path.empty()) return -1;
  return static_cast<int>(path.size()) - 1;
}

bool Graph::has_cycle() {
  std::set<std::string> visited;
  for (const auto &[vname, vertex] : vertices) {
    if (visited.count(vname)) continue;
    std::map<std::string, std::string> parent;
    parent[vname] = "";
    std::deque<std::string> queue;
    queue.push_back(vname);
    visited.insert(vname);
    while (!queue.empty()) {
      std::string node = queue.front();
      queue.pop_front();
      for (const auto &neighbor : get_neighbors(node)) {
        if (!visited.count(neighbor)) {
          visited.insert(neighbor);
          parent[neighbor] = node;
          queue.push_back(neighbor);
        } else if (parent[node] != neighbor) {
          return true;
        }
      }
    }
  }
  return false;
}

std::vector<std::vector<std::string>> Graph::cycle_basis() {
  std::vector<std::vector<std::string>> result;
  std::map<std::string, int> disc;
  std::map<std::string, std::string> par;
  int timer = 0;
  for (const auto &[vname, vertex] : vertices) {
    if (disc.count(vname)) continue;
    par[vname] = "";
    disc[vname] = timer++;
    auto nbrs = get_neighbors(vname);
    std::sort(nbrs.begin(), nbrs.end());
    using Frame = std::tuple<std::string, std::string, std::vector<std::string>, int>;
    std::vector<Frame> stk;
    stk.reserve(number_of_vertices());
    stk.emplace_back(vname, "", nbrs, 0);
    while (!stk.empty()) {
      auto &[u, p, nbrs_u, idx] = stk.back();
      if (idx < (int)nbrs_u.size()) {
        std::string v = nbrs_u[idx];
        idx++;
        if (!disc.count(v)) {
          par[v] = u;
          disc[v] = timer++;
          auto v_nbrs = get_neighbors(v);
          std::sort(v_nbrs.begin(), v_nbrs.end());
          stk.emplace_back(v, u, v_nbrs, 0);
        } else if (v != p && disc[v] < disc[u]) {
          std::vector<std::string> cycle;
          std::string node = u;
          while (node != v) {
            cycle.push_back(node);
            node = par[node];
          }
          cycle.push_back(v);
          result.push_back(cycle);
        }
      } else {
        stk.pop_back();
      }
    }
  }
  return result;
}

std::ostream &operator<<(std::ostream &os, const Vertex &vertex) {
  return os << vertex.str();
}

std::ostream &operator<<(std::ostream &os, const Edge &edge) {
  return os << edge.str();
}

std::ostream &operator<<(std::ostream &os, const Graph &graph) {
  return os << graph.str();
}
} // namespace session_cpp