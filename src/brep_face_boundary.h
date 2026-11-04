#pragma once

#include "brep.h"
#include <string>
#include <utility>
#include <vector>

namespace session_cpp::brep_face_boundary {

/// Owner indices and raw domains with wire-edge orientation; material orientation is separate.
struct Use {
    BRepRef edge;
    int pcurve = -1;
    int first_vertex = -1, last_vertex = -1;
    bool degenerated = false, has_world_domain = false;
    std::pair<double, double> world_domain{}, pcurve_domain{};
};
struct Wire {
    BRepRef reference;
    std::vector<Use> uses;
};
struct Result {
    const BRep* owner = nullptr;
    int face = -1;
    std::vector<Wire> wires;
    bool complete = false;
    std::string failure;
};

namespace detail {
inline bool active(BRepOrientation orientation) {
    return orientation == BRepOrientation::Forward || orientation == BRepOrientation::Reversed;
}

inline bool unique_association(const BRepEdge& edge, int support, BRepOrientation orientation) {
    int associated = -1;
    for (const auto& image : edge.pcurves) {
        if (image.surface_index != support)
            continue;
        const int selected = orientation == BRepOrientation::Reversed && image.curve_2d_index_2 >= 0
            ? image.curve_2d_index_2 : image.curve_2d_index;
        if (associated >= 0 && selected != associated && active(orientation))
            return false;
        associated = selected;
    }
    return true;
}

inline bool extract_use(const BRep& brep, int face, const BRepRef& ref, Use& use, std::string& failure) {
    const auto fail = [&](const std::string& reason) {
        failure = reason;
        return false;
    };
    if (ref.index < 0 || ref.index >= static_cast<int>(brep.m_edges.size()))
        return fail("Boundary edge index is invalid");
    const auto& edge = brep.m_edges[ref.index];
    for (int vertex : {edge.start_vertex, edge.end_vertex})
        if (vertex < 0 || vertex >= static_cast<int>(brep.m_vertices.size()))
            return fail("Boundary vertex index is invalid");
    use.edge = ref;
    use.degenerated = edge.degenerated;
    use.first_vertex = ref.orientation == BRepOrientation::Reversed ? edge.end_vertex : edge.start_vertex;
    use.last_vertex = ref.orientation == BRepOrientation::Reversed ? edge.start_vertex : edge.end_vertex;
    if (edge.degenerated && edge.start_vertex != edge.end_vertex)
        return fail("Degenerated boundary has distinct vertex identities");
    if (!edge.degenerated) {
        if (edge.curve_3d_index < 0 || edge.curve_3d_index >= static_cast<int>(brep.m_curves_3d.size()))
            return fail("Boundary world curve index is invalid");
        use.has_world_domain = true;
        use.world_domain = brep.m_curves_3d[edge.curve_3d_index].domain();
    }
    if (!unique_association(edge, brep.m_faces[face].surface_index, ref.orientation))
        return fail("Boundary has ambiguous pcurve associations on one support");
    use.pcurve = brep.pcurve_index(ref.index, face, ref.orientation);
    if (use.pcurve >= static_cast<int>(brep.m_curves_2d.size()))
        return fail("Boundary pcurve index is invalid");
    if (use.pcurve >= 0)
        use.pcurve_domain = brep.m_curves_2d[use.pcurve].domain();
    if (active(ref.orientation) && use.pcurve < 0)
        return fail("Boundary use has no pcurve on its support");
    return true;
}

inline bool extract_wire(const BRep& brep, int face, const BRepRef& ref, Wire& wire, std::string& failure) {
    if (ref.index < 0 || ref.index >= static_cast<int>(brep.m_wires.size())) {
        failure = "Boundary wire index is invalid";
        return false;
    }
    wire.reference = ref;
    const auto edges = brep.wire_edges(ref);
    if (edges.empty()) {
        failure = "Boundary wire is empty";
        return false;
    }
    int first = -1, previous = -1;
    for (const auto& edge : edges) {
        Use use;
        if (!extract_use(brep, face, edge, use, failure))
            return false;
        if (active(edge.orientation)) {
            if (previous >= 0 && previous != use.first_vertex) {
                failure = "Boundary vertex chain is open";
                return false;
            }
            if (first < 0)
                first = use.first_vertex;
            previous = use.last_vertex;
        }
        wire.uses.push_back(use);
    }
    if (first >= 0 && previous != first) {
        failure = "Boundary vertex chain is not closed";
        return false;
    }
    return true;
}
} // namespace detail

/// Extract structural incidence without certifying UV closure or geometric correspondence.
inline Result extract(const BRep& brep, int face) {
    Result result;
    result.owner = &brep;
    result.face = face;
    if (face < 0 || face >= brep.face_count()) {
        result.failure = "Face index is invalid";
        return result;
    }
    const auto& boundary = brep.m_faces[face];
    if (boundary.surface_index < 0 || boundary.surface_index >= static_cast<int>(brep.m_surfaces.size())) {
        result.failure = "Face support index is invalid";
        return result;
    }
    if (boundary.wires.empty()) {
        result.failure = "Face has no boundary wires";
        return result;
    }
    for (const auto& reference : boundary.wires) {
        Wire wire;
        if (!detail::extract_wire(brep, face, reference, wire, result.failure))
            return result;
        result.wires.push_back(std::move(wire));
    }
    result.complete = true;
    return result;
}

} // namespace session_cpp::brep_face_boundary
