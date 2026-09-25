#include "graph.h"
#include "graph.pb.h"
#include "vertex.pb.h"
#include "edge.pb.h"

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Vertex
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Vertex::jsondump() const {

    nlohmann::ordered_json data;
    data["attribute"] = attribute;
    data["attributes"] = attributes;
    data["guid"] = guid();
    data["index"] = index;
    data["name"] = name;
    data["type"] = "Vertex";

    return data;
}

Vertex Vertex::jsonload(const nlohmann::json& data) {

    Vertex vertex(data["name"], data["attribute"]);
    vertex.guid() = data["guid"];
    vertex.index = data["index"];

    if (data.contains("attributes"))
        vertex.attributes = data["attributes"];

    return vertex;
}

std::string Vertex::str() const {
    return fmt::format("Vertex({}, {}, {}, {})", guid(), name, attribute, index);
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge
// ═══════════════════════════════════════════════════════════════════════════
std::tuple<std::string, std::string> Edge::vertices() const {
    return std::make_tuple(v0, v1);
}

bool Edge::connects(const std::string& vertex_id) const {
    return v0 == vertex_id || v1 == vertex_id;
}

std::string Edge::other_vertex(const std::string& vertex_id) const {

    if (v0 == vertex_id)
        return v1;

    if (v1 == vertex_id)
        return v0;

    return "";
}

nlohmann::ordered_json Edge::jsondump() const {

    nlohmann::ordered_json data;
    data["attribute"] = attribute;
    data["attributes"] = attributes;
    data["guid"] = guid();
    data["index"] = index;
    data["name"] = name;
    data["type"] = "Edge";
    data["v0"] = v0;
    data["v1"] = v1;

    return data;
}

Edge Edge::jsonload(const nlohmann::json& data) {

    Edge edge(data["v0"], data["v1"], data["attribute"]);
    edge.name = data["name"];
    edge.guid() = data["guid"];
    edge.index = data["index"];

    if (data.contains("attributes"))
        edge.attributes = data["attributes"];

    return edge;
}

std::string Edge::str() const {
    return fmt::format("Edge({}, {}, {}, {}, {})", guid(), name, v0, v1, attribute);
}

// ═══════════════════════════════════════════════════════════════════════════
// Details
// ═══════════════════════════════════════════════════════════════════════════
bool Graph::has_node(const std::string& key) const {
    return vertices.find(key) != vertices.end();
}

bool Graph::has_edge(const std::tuple<std::string, std::string>& key) const {

    auto it = edges.find(std::get<0>(key));

    if (it == edges.end())
        return false;

    return it->second.find(std::get<1>(key)) != it->second.end();
}

std::string Graph::add_node(const std::string& key, const std::string& attribute) {

    if (has_node(key))
        return vertices[key].name;

    Vertex vertex(key, attribute);
    vertex.index = vertex_count;

    vertices[key] = vertex;
    vertex_count += 1;

    return vertex.name;
}

std::tuple<std::string, std::string> Graph::add_edge(const std::string& u, const std::string& v, const std::string& attribute) {

    if (!has_node(u))
        add_node(u);

    if (!has_node(v))
        add_node(v);

    if (has_edge(std::make_tuple(u, v))) {
        edges[u][v].attribute = attribute;
        edges[v][u] = edges[u][v];

        return std::make_tuple(u, v);
    }

    Edge edge(u, v, attribute);
    edge.index = edge_count;
    edge.guid() = ::guid();

    edges[u][v] = edge;
    edges[v][u] = edge;
    edge_count += 1;

    return std::make_tuple(u, v);
}

void Graph::remove_node(const std::string& key) {

    if (!has_node(key))
        throw std::runtime_error("Node " + key + " not in graph");

    auto it = edges.find(key);

    if (it != edges.end()) {
        for (const std::pair<const std::string, Edge>& neighbor : it->second)
            edges[neighbor.first].erase(key);

        edges.erase(it);
    }

    vertices.erase(key);
    _reassign_indices();
    _reassign_edge_indices();
}

void Graph::remove_edge(const std::tuple<std::string, std::string>& edge) {

    if (!has_edge(edge))
        return;

    const std::string u = std::get<0>(edge);
    const std::string v = std::get<1>(edge);

    edges[u].erase(v);
    edges[v].erase(u);
    _reassign_edge_indices();
}

void Graph::_reassign_indices() {

    std::vector<std::tuple<int, std::string>> list;

    for (const std::pair<const std::string, Vertex>& vertex : vertices)
        list.emplace_back(vertex.second.index, vertex.first);

    std::sort(list.begin(), list.end());

    for (size_t i = 0; i < list.size(); ++i)
        vertices[std::get<1>(list[i])].index = static_cast<int>(i);

    vertex_count = static_cast<int>(list.size());
}

void Graph::_reassign_edge_indices() {

    std::vector<std::tuple<int, std::string, std::string>> list;

    for (const std::pair<const std::string, std::map<std::string, Edge>>& adjacency : edges)
        for (const std::pair<const std::string, Edge>& neighbor : adjacency.second)
            if (adjacency.first < neighbor.first)
                list.emplace_back(neighbor.second.index, adjacency.first, neighbor.first);

    std::sort(list.begin(), list.end());

    for (size_t i = 0; i < list.size(); ++i) {
        const std::string& u = std::get<1>(list[i]);
        const std::string& v = std::get<2>(list[i]);

        edges[u][v].index = static_cast<int>(i);
        edges[v][u].index = static_cast<int>(i);
    }

    edge_count = static_cast<int>(list.size());
}

std::vector<Vertex> Graph::get_vertices() const {

    std::vector<Vertex> result;

    for (const std::pair<const std::string, Vertex>& vertex : vertices)
        result.push_back(vertex.second);

    return result;
}

std::vector<std::tuple<std::string, std::string>> Graph::get_edges() const {

    std::vector<std::tuple<std::string, std::string>> result;

    for (const std::pair<const std::string, std::map<std::string, Edge>>& adjacency : edges)
        for (const std::pair<const std::string, Edge>& neighbor : adjacency.second)
            if (adjacency.first < neighbor.first)
                result.push_back(std::make_tuple(adjacency.first, neighbor.first));

    return result;
}

std::vector<std::string> Graph::neighbors(const std::string& node) const {

    if (!has_node(node))
        throw std::runtime_error("Node " + node + " not in graph");

    std::vector<std::string> result;
    auto it = edges.find(node);

    if (it == edges.end())
        return result;

    for (const std::pair<const std::string, Edge>& neighbor : it->second)
        result.push_back(neighbor.first);

    return result;
}

std::vector<std::tuple<std::string, std::string, bool>> Graph::edges_of(const std::string& node) const {

    std::vector<std::tuple<std::string, std::string, bool>> result;
    auto it = edges.find(node);

    if (it == edges.end())
        return result;

    for (const std::pair<const std::string, Edge>& neighbor : it->second)
        result.emplace_back(neighbor.first, neighbor.second.attribute, neighbor.second.v0 == node);

    return result;
}

int Graph::number_of_vertices() const {
    return static_cast<int>(vertices.size());
}

int Graph::number_of_edges() const {

    int count = 0;

    for (const std::pair<const std::string, std::map<std::string, Edge>>& adjacency : edges)
        for (const std::pair<const std::string, Edge>& neighbor : adjacency.second)
            if (adjacency.first < neighbor.first)
                count += 1;

    return count;
}

void Graph::clear() {

    vertices.clear();
    edges.clear();
    vertex_count = 0;
    edge_count = 0;
}

std::string Graph::node_label(const std::string& node, const std::string& value) {

    if (!has_node(node))
        throw std::runtime_error("Node " + node + " not in graph");

    if (value.empty())
        return vertices[node].attribute;

    vertices[node].attribute = value;

    return value;
}

std::string Graph::edge_label(const std::string& u, const std::string& v, const std::string& value) {

    if (!has_edge(std::make_tuple(u, v)))
        throw std::runtime_error("Edge (" + u + ", " + v + ") not in graph");

    if (value.empty())
        return edges[u][v].attribute;

    edges[u][v].attribute = value;
    edges[v][u].attribute = value;

    return value;
}

// ═══════════════════════════════════════════════════════════════════════════
// Attribute API
// ═══════════════════════════════════════════════════════════════════════════
void Graph::update_default_vertex_attributes(const std::vector<std::pair<std::string, double>>& attrs) {

    for (const std::pair<std::string, double>& attr : attrs)
        default_vertex_attributes[attr.first] = attr.second;
}

void Graph::update_default_edge_attributes(const std::vector<std::pair<std::string, double>>& attrs) {

    for (const std::pair<std::string, double>& attr : attrs)
        default_edge_attributes[attr.first] = attr.second;
}

std::optional<double> Graph::vertex_attribute(const std::string& key, const std::string& name) const {

    auto it = vertices.find(key);

    if (it == vertices.end())
        return std::nullopt;

    auto value = it->second.attributes.find(name);

    if (value != it->second.attributes.end())
        return value->second;

    auto fallback = default_vertex_attributes.find(name);

    if (fallback != default_vertex_attributes.end())
        return fallback->second;

    return std::nullopt;
}

void Graph::set_vertex_attribute(const std::string& key, const std::string& name, double value) {

    if (has_node(key))
        vertices[key].attributes[name] = value;
}

std::optional<double> Graph::edge_attribute(const std::tuple<std::string, std::string>& edge, const std::string& name) const {

    if (!has_edge(edge))
        return std::nullopt;

    const Edge& stored = edges.at(std::get<0>(edge)).at(std::get<1>(edge));
    auto value = stored.attributes.find(name);

    if (value != stored.attributes.end())
        return value->second;

    auto fallback = default_edge_attributes.find(name);

    if (fallback != default_edge_attributes.end())
        return fallback->second;

    return std::nullopt;
}

void Graph::set_edge_attribute(const std::tuple<std::string, std::string>& edge, const std::string& name, double value) {

    if (!has_edge(edge))
        return;

    const std::string& u = std::get<0>(edge);
    const std::string& v = std::get<1>(edge);

    edges[u][v].attributes[name] = value;
    edges[v][u].attributes[name] = value;
}

std::vector<std::string> Graph::vertices_where(const std::vector<std::pair<std::string, double>>& conditions) const {

    std::vector<std::string> result;

    for (const std::pair<const std::string, Vertex>& vertex : vertices) {
        bool matched = true;

        for (const std::pair<std::string, double>& condition : conditions)
            if (vertex_attribute(vertex.first, condition.first) != condition.second)
                matched = false;

        if (matched)
            result.push_back(vertex.first);
    }

    return result;
}

std::vector<std::tuple<std::string, std::string>> Graph::edges_where(
    const std::vector<std::pair<std::string, double>>& conditions
) const {

    std::vector<std::tuple<std::string, std::string>> result;

    for (const std::tuple<std::string, std::string>& edge : get_edges()) {
        bool matched = true;

        for (const std::pair<std::string, double>& condition : conditions)
            if (edge_attribute(edge, condition.first) != condition.second)
                matched = false;

        if (matched)
            result.push_back(edge);
    }

    return result;
}

std::vector<std::string> Graph::vertices_where_predicate(
    const std::function<bool(const std::string&, const std::map<std::string, double>&)>& pred
) const {

    std::vector<std::string> result;

    for (const std::pair<const std::string, Vertex>& vertex : vertices) {
        std::map<std::string, double> attributes = default_vertex_attributes;

        for (const std::pair<const std::string, double>& attribute : vertex.second.attributes)
            attributes[attribute.first] = attribute.second;

        if (pred(vertex.first, attributes))
            result.push_back(vertex.first);
    }

    return result;
}

std::vector<std::tuple<std::string, std::string>> Graph::edges_where_predicate(
    const std::function<bool(const std::tuple<std::string, std::string>&, const std::map<std::string, double>&)>& pred
) const {

    std::vector<std::tuple<std::string, std::string>> result;

    for (const std::tuple<std::string, std::string>& edge : get_edges()) {
        const Edge& stored = edges.at(std::get<0>(edge)).at(std::get<1>(edge));
        std::map<std::string, double> attributes = default_edge_attributes;

        for (const std::pair<const std::string, double>& attribute : stored.attributes)
            attributes[attribute.first] = attribute.second;

        if (pred(edge, attributes))
            result.push_back(edge);
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Algorithms
// ═══════════════════════════════════════════════════════════════════════════
std::vector<std::string> Graph::bfs(const std::string& start) const {

    std::vector<std::string> result;

    if (!has_node(start))
        return result;

    std::set<std::string> visited;
    std::deque<std::string> queue;
    queue.push_back(start);
    visited.insert(start);

    while (!queue.empty()) {
        const std::string node = queue.front();
        queue.pop_front();
        result.push_back(node);

        for (const std::string& neighbor : neighbors(node)) {
            if (visited.count(neighbor))
                continue;

            visited.insert(neighbor);
            queue.push_back(neighbor);
        }
    }

    return result;
}

std::vector<std::string> Graph::dfs(const std::string& start) const {

    std::vector<std::string> result;

    if (!has_node(start))
        return result;

    std::set<std::string> visited;
    std::vector<std::string> stack = {start};

    while (!stack.empty()) {
        const std::string node = stack.back();
        stack.pop_back();

        if (visited.count(node))
            continue;

        visited.insert(node);
        result.push_back(node);

        const std::vector<std::string> nbrs = neighbors(node);

        for (int i = static_cast<int>(nbrs.size()) - 1; i >= 0; --i)
            if (!visited.count(nbrs[i]))
                stack.push_back(nbrs[i]);
    }

    return result;
}

std::vector<std::vector<std::string>> Graph::connected_components() const {

    std::set<std::string> visited;
    std::vector<std::vector<std::string>> components;

    for (const std::pair<const std::string, Vertex>& vertex : vertices) {
        if (visited.count(vertex.first))
            continue;

        std::vector<std::string> component = bfs(vertex.first);

        for (const std::string& node : component)
            visited.insert(node);

        std::sort(component.begin(), component.end());
        components.push_back(component);
    }

    return components;
}

bool Graph::is_connected() const {
    return connected_components().size() <= 1;
}

int Graph::number_connected_components() const {
    return static_cast<int>(connected_components().size());
}

std::vector<std::string> Graph::shortest_path(const std::string& u, const std::string& v) const {

    std::vector<std::string> path;

    if (!has_node(u) || !has_node(v))
        return path;

    if (u == v)
        return {u};

    std::map<std::string, std::string> parent;
    parent[u] = "";

    std::deque<std::string> queue;
    queue.push_back(u);

    while (!queue.empty()) {
        const std::string node = queue.front();
        queue.pop_front();

        for (const std::string& neighbor : neighbors(node)) {
            if (parent.count(neighbor))
                continue;

            parent[neighbor] = node;

            if (neighbor == v) {
                std::string current = v;

                while (current != u) {
                    path.push_back(current);
                    current = parent[current];
                }

                path.push_back(u);
                std::reverse(path.begin(), path.end());

                return path;
            }

            queue.push_back(neighbor);
        }
    }

    return path;
}

int Graph::shortest_path_length(const std::string& u, const std::string& v) const {

    const std::vector<std::string> path = shortest_path(u, v);

    if (path.empty())
        return -1;

    return static_cast<int>(path.size()) - 1;
}

bool Graph::has_cycle() const {

    std::set<std::string> visited;

    for (const std::pair<const std::string, Vertex>& vertex : vertices) {
        if (visited.count(vertex.first))
            continue;

        std::map<std::string, std::string> parent;
        parent[vertex.first] = "";

        std::deque<std::string> queue;
        queue.push_back(vertex.first);
        visited.insert(vertex.first);

        while (!queue.empty()) {
            const std::string node = queue.front();
            queue.pop_front();

            for (const std::string& neighbor : neighbors(node)) {
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

std::vector<std::vector<std::string>> Graph::cycle_basis() const {

    std::vector<std::vector<std::string>> result;
    std::map<std::string, int> order;
    std::map<std::string, std::string> parent;
    int timer = 0;

    for (const std::pair<const std::string, Vertex>& vertex : vertices) {
        if (order.count(vertex.first))
            continue;

        parent[vertex.first] = "";
        order[vertex.first] = timer++;

        std::vector<std::tuple<std::string, std::string, std::vector<std::string>, int>> stack;
        stack.reserve(vertices.size());
        stack.emplace_back(vertex.first, "", neighbors(vertex.first), 0);

        while (!stack.empty()) {
            std::tuple<std::string, std::string, std::vector<std::string>, int>& frame = stack.back();
            const std::string u = std::get<0>(frame);
            const std::string p = std::get<1>(frame);

            if (std::get<3>(frame) >= static_cast<int>(std::get<2>(frame).size())) {
                stack.pop_back();
                continue;
            }

            const std::string v = std::get<2>(frame)[std::get<3>(frame)];
            std::get<3>(frame) += 1;

            if (!order.count(v)) {
                parent[v] = u;
                order[v] = timer++;
                stack.emplace_back(v, u, neighbors(v), 0);
            } else if (v != p && order[v] < order[u]) {
                std::vector<std::string> cycle;
                std::string node = u;

                while (node != v) {
                    cycle.push_back(node);
                    node = parent[node];
                }

                cycle.push_back(v);
                result.push_back(cycle);
            }
        }
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Graph::jsondump() const {

    nlohmann::ordered_json vertices_json = nlohmann::ordered_json::array();

    for (const std::pair<const std::string, Vertex>& vertex : vertices)
        vertices_json.push_back(vertex.second.jsondump());

    nlohmann::ordered_json edges_json = nlohmann::ordered_json::array();

    for (const std::pair<const std::string, std::map<std::string, Edge>>& adjacency : edges)
        for (const std::pair<const std::string, Edge>& neighbor : adjacency.second)
            if (adjacency.first < neighbor.first)
                edges_json.push_back(neighbor.second.jsondump());

    nlohmann::ordered_json data;
    data["default_edge_attributes"] = default_edge_attributes;
    data["default_vertex_attributes"] = default_vertex_attributes;
    data["edge_count"] = edge_count;
    data["edges"] = edges_json;
    data["guid"] = guid();
    data["name"] = name;
    data["type"] = "Graph";
    data["vertex_count"] = vertex_count;
    data["vertices"] = vertices_json;

    return data;
}

Graph Graph::jsonload(const nlohmann::json& data) {

    Graph graph(data["name"]);
    graph.guid() = data["guid"];
    graph.vertex_count = data["vertex_count"];
    graph.edge_count = data["edge_count"];

    if (data.contains("default_edge_attributes"))
        graph.default_edge_attributes = data["default_edge_attributes"];

    if (data.contains("default_vertex_attributes"))
        graph.default_vertex_attributes = data["default_vertex_attributes"];

    for (const nlohmann::json& vertex_data : data["vertices"]) {
        const Vertex vertex = Vertex::jsonload(vertex_data);
        graph.vertices[vertex.name] = vertex;
    }

    for (const nlohmann::json& edge_data : data["edges"]) {
        const Edge edge = Edge::jsonload(edge_data);
        graph.edges[edge.v0][edge.v1] = edge;
        graph.edges[edge.v1][edge.v0] = edge;
    }

    return graph;
}

std::string Graph::file_json_dumps() const {
    return jsondump().dump();
}

Graph Graph::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Graph::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Graph Graph::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Graph Graph::to_proto() const {

    session_proto::Graph proto;
    proto.set_name(name);

    if (has_guid())
        proto.set_guid(guid());

    proto.set_vertex_count(vertex_count);
    proto.set_edge_count(edge_count);

    for (const std::pair<const std::string, double>& attribute : default_vertex_attributes)
        (*proto.mutable_default_vertex_attributes())[attribute.first] = attribute.second;

    for (const std::pair<const std::string, double>& attribute : default_edge_attributes)
        (*proto.mutable_default_edge_attributes())[attribute.first] = attribute.second;

    for (const std::pair<const std::string, Vertex>& vertex : vertices) {
        session_proto::Vertex& v = (*proto.mutable_vertices())[vertex.first];
        v.set_name(vertex.second.name);

        if (vertex.second.has_guid())
            v.set_guid(vertex.second.guid());

        v.set_attribute(vertex.second.attribute);
        v.set_index(vertex.second.index);

        for (const std::pair<const std::string, double>& attribute : vertex.second.attributes)
            (*v.mutable_attributes())[attribute.first] = attribute.second;
    }

    for (const std::pair<const std::string, std::map<std::string, Edge>>& adjacency : edges) {
        for (const std::pair<const std::string, Edge>& neighbor : adjacency.second) {
            if (adjacency.first > neighbor.first)
                continue;

            const Edge& edge = neighbor.second;
            session_proto::Edge* e = proto.add_edges();

            if (edge.has_guid())
                e->set_guid(edge.guid());

            e->set_name(edge.name);
            e->set_v0(edge.v0);
            e->set_v1(edge.v1);
            e->set_attribute(edge.attribute);
            e->set_index(edge.index);

            for (const std::pair<const std::string, double>& attribute : edge.attributes)
                (*e->mutable_attributes())[attribute.first] = attribute.second;
        }
    }

    return proto;
}

Graph Graph::from_proto(const session_proto::Graph& proto) {

    Graph graph(proto.name());

    if (!proto.guid().empty())
        graph.guid() = proto.guid();

    graph.vertex_count = proto.vertex_count();
    graph.edge_count = proto.edge_count();

    for (const google::protobuf::MapPair<std::string, double>& attribute : proto.default_vertex_attributes())
        graph.default_vertex_attributes[attribute.first] = attribute.second;

    for (const google::protobuf::MapPair<std::string, double>& attribute : proto.default_edge_attributes())
        graph.default_edge_attributes[attribute.first] = attribute.second;

    for (const google::protobuf::MapPair<std::string, session_proto::Vertex>& pair : proto.vertices()) {
        const session_proto::Vertex& v = pair.second;
        Vertex vertex(v.name(), v.attribute());
        vertex.guid() = v.guid();
        vertex.index = v.index();

        for (const google::protobuf::MapPair<std::string, double>& attribute : v.attributes())
            vertex.attributes[attribute.first] = attribute.second;

        graph.vertices[pair.first] = vertex;
    }

    for (const session_proto::Edge& e : proto.edges()) {
        Edge edge(e.v0(), e.v1(), e.attribute());
        edge.name = e.name();
        edge.guid() = e.guid();
        edge.index = e.index();

        for (const google::protobuf::MapPair<std::string, double>& attribute : e.attributes())
            edge.attributes[attribute.first] = attribute.second;

        graph.edges[e.v0()][e.v1()] = edge;
        graph.edges[e.v1()][e.v0()] = edge;
    }

    return graph;
}

std::string Graph::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Graph Graph::pb_loads(const std::string& data) {

    session_proto::Graph proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Graph protobuf data");

    return from_proto(proto);
}

void Graph::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Graph Graph::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Graph::str() const {
    return fmt::format("<Graph with {} vertices, {} edges: {}>", vertex_count, edge_count, name);
}

std::string Graph::repr() const {
    return fmt::format("Graph({}, {}, {}, {})", guid(), name, vertex_count, edge_count);
}

// ═══════════════════════════════════════════════════════════════════════════
// Stream operators
// ═══════════════════════════════════════════════════════════════════════════
std::ostream& operator<<(std::ostream& os, const Vertex& vertex) {
    return os << vertex.str();
}

std::ostream& operator<<(std::ostream& os, const Edge& edge) {
    return os << edge.str();
}

std::ostream& operator<<(std::ostream& os, const Graph& graph) {
    return os << graph.str();
}

} // namespace session_cpp
