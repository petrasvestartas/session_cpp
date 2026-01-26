#include "edge.h"

namespace session_cpp {

std::string Edge::str() const {
    return fmt::format("Edge({}, {}, {}, {}, {})", guid, name, v0, v1, attribute);
}

nlohmann::ordered_json Edge::jsondump() const {
    return nlohmann::ordered_json{{"type", "Edge"},
                                  {"name", name},
                                  {"guid", guid},
                                  {"v0", v0},
                                  {"v1", v1},
                                  {"attribute", attribute},
                                  {"index", index}};
}

Edge Edge::jsonload(const nlohmann::json &data) {
    Edge edge(data["v0"], data["v1"], data["attribute"]);
    edge.name = data["name"];
    edge.guid = data["guid"];
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
    if (v0 == vertex_id) {
        return v1;
    } else if (v1 == vertex_id) {
        return v0;
    }
    return "";
}

} // namespace session_cpp
