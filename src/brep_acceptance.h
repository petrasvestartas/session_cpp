#pragma once
#include "brep.h"
#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace session_cpp::brep_acceptance {

enum class VertexLinks { Matched, NonManifold, Unsupported };

struct VertexLink {
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> neighbours;
};

inline bool link_cycle(const VertexLink& link) {
    if (link.neighbours.empty())
        return false;
    for (const std::pair<const std::pair<int, int>, std::vector<std::pair<int, int>>>& node : link.neighbours)
        if (node.second.size() != 2)
            return false;
    std::set<std::pair<int, int>> visited;
    std::vector<std::pair<int, int>> queue{link.neighbours.begin()->first};
    visited.insert(queue.front());
    for (size_t i = 0; i < queue.size(); ++i)
        for (const std::pair<int, int>& next : link.neighbours.at(queue[i]))
            if (visited.insert(next).second)
                queue.push_back(next);
    return visited.size() == link.neighbours.size();
}

inline VertexLinks regular_sector(const BRep& brep, const BRepRef& face, const BRepRef& wire,
                                  std::vector<BRepRef>& regular, std::map<int, VertexLink>& links) {
    std::vector<BRepRef> uses = brep.wire_edges(wire);
    if (face.orientation == BRepOrientation::Reversed)
        std::reverse(uses.begin(), uses.end());
    for (BRepRef& use : uses) {
        use.orientation = brep_compose(face.orientation, use.orientation);
        if (use.orientation != BRepOrientation::Forward && use.orientation != BRepOrientation::Reversed)
            return VertexLinks::Unsupported;
    }
    if (uses.empty())
        return VertexLinks::Unsupported;
    for (size_t i = 0; i < uses.size(); ++i) {
        const BRepRef& use = uses[i];
        const BRepRef& next = uses[(i + 1) % uses.size()];
        const BRepEdge& edge = brep.m_edges[use.index];
        const BRepEdge& following = brep.m_edges[next.index];
        const int end = use.orientation == BRepOrientation::Forward ? edge.end_vertex : edge.start_vertex;
        const int start = next.orientation == BRepOrientation::Forward ? following.start_vertex : following.end_vertex;
        if (edge.degenerated && edge.start_vertex != edge.end_vertex)
            return VertexLinks::Unsupported;
        if (end != start)
            return VertexLinks::NonManifold;
        links[edge.start_vertex];
        links[edge.end_vertex];
        if (!edge.degenerated)
            regular.push_back(use);
    }
    return regular.empty() ? VertexLinks::Unsupported : VertexLinks::Matched;
}

inline void append_sector(const BRep& brep, const std::vector<BRepRef>& regular,
                          std::map<int, VertexLink>& links) {
    for (size_t i = 0; i < regular.size(); ++i) {
        const BRepRef& a = regular[i];
        const BRepRef& b = regular[(i + 1) % regular.size()];
        const int ae = a.orientation == BRepOrientation::Forward ? 1 : 0;
        const int be = b.orientation == BRepOrientation::Forward ? 0 : 1;
        const BRepEdge& edge = brep.m_edges[a.index];
        const int vertex = ae ? edge.end_vertex : edge.start_vertex;
        const std::pair<int, int> first{a.index, ae}, second{b.index, be};
        links[vertex].neighbours[first].push_back(second);
        links[vertex].neighbours[second].push_back(first);
    }
}

// Requires valid references and closed connected shell ownership; no geometric certificate.
inline VertexLinks shell_vertex_links(const BRep& brep, const BRepShell& shell) {
    std::map<int, VertexLink> links;
    for (const BRepRef& face : shell.faces) {
        if (face.orientation != BRepOrientation::Forward && face.orientation != BRepOrientation::Reversed)
            return VertexLinks::Unsupported;
        for (const BRepRef& wire : brep.m_faces[face.index].wires) {
            std::vector<BRepRef> regular;
            const VertexLinks status = regular_sector(brep, face, wire, regular, links);
            if (status != VertexLinks::Matched)
                return status;
            append_sector(brep, regular, links);
        }
    }
    for (const std::pair<const int, VertexLink>& vertex : links)
        if (!link_cycle(vertex.second))
            return VertexLinks::NonManifold;
    return VertexLinks::Matched;
}

}
