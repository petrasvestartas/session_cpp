#include "brep.h"
#include "native_tessellate_certificate.h"
#include "native_tessellate_diagnostics.h"
#include "mesh_parametric_metadata.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <set>
#include <sstream>
#include <tuple>

namespace session_cpp {
namespace {
struct TessellationTriangle {
    int face;
    std::array<native_tessellate::UV, 3> uv;
    std::array<Point, 3> points;
    std::array<size_t, 3> source_vertices;
    std::array<std::set<int>, 3> boundary_edges;
    std::array<std::set<int>, 3> boundary_vertices;
};

bool measured(const NurbsSurface& surface, const TessellationTriangle& triangle,
              double deflection, double angle) {
    const Vector first = triangle.points[1] - triangle.points[0];
    const Vector second = triangle.points[2] - triangle.points[0];
    Vector plane = first.cross(second);
    if (!plane.normalize_self())
        return false;
    for (int i = 0; i <= 4; ++i)
        for (int j = 0; j <= 4 - i; ++j) {
            const double a = i * .25, b = j * .25, c = 1 - a - b;
            const double u = a * triangle.uv[0][0] + b * triangle.uv[1][0] + c * triangle.uv[2][0];
            const double v = a * triangle.uv[0][1] + b * triangle.uv[1][1] + c * triangle.uv[2][1];
            const Point linear(a * triangle.points[0][0] + b * triangle.points[1][0] + c * triangle.points[2][0],
                a * triangle.points[0][1] + b * triangle.points[1][1] + c * triangle.points[2][1],
                a * triangle.points[0][2] + b * triangle.points[1][2] + c * triangle.points[2][2]);
            const double error = surface.point_at(u, v).distance(linear);
            if (!std::isfinite(error) || error > deflection)
                return false;
            const double center_u = (triangle.uv[0][0] + triangle.uv[1][0] + triangle.uv[2][0]) / 3;
            const double center_v = (triangle.uv[0][1] + triangle.uv[1][1] + triangle.uv[2][1]) / 3;
            const std::vector<Vector> derivatives = surface.evaluate(
                u + (center_u - u) * 1e-6, v + (center_v - v) * 1e-6, 1);
            Vector normal = derivatives[1].cross(derivatives[2]);
            if (!normal.normalize_self()) {
                if (i && j && i + j < 4)
                    return false;
                continue;
            }
            const double cosine = std::clamp(std::abs(normal.dot(plane)), 0.0, 1.0);
            if (!std::isfinite(cosine) || std::acos(cosine) > angle)
                return false;
        }
    return true;
}

bool proposed_triangles(const BRep& brep, const std::vector<Mesh>& faces,
    double allowed, std::vector<TessellationTriangle>& triangles,
    native_tessellate_diagnostics::State* diagnostic) {
    const auto fail = [&](const char* reason) {
        if (diagnostic)
            diagnostic->metadata_failure = reason;
        return false;
    };
    if (faces.size() != brep.m_faces.size())
        return fail("face_count");
    for (size_t face = 0; face < faces.size(); ++face) {
        if (faces[face].face.empty())
            return fail("empty_face");
        if (diagnostic)
            diagnostic->face = static_cast<int>(face);
        const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
        const auto& triangulation = faces[face].get_triangulation();
        for (const std::pair<const size_t, std::vector<size_t>>& polygon : faces[face].face) {
            const std::vector<size_t>& keys = polygon.second;
            std::vector<std::array<size_t, 3>> pieces;
            if (keys.size() == 3)
                pieces.push_back({keys[0], keys[1], keys[2]});
            else {
                const auto found = triangulation.find(polygon.first);
                if (found == triangulation.end())
                    return fail("missing_triangulation");
                pieces = found->second;
            }
            for (const std::array<size_t, 3>& piece : pieces) {
                TessellationTriangle triangle;
                triangle.face = static_cast<int>(face);
                for (int i = 0; i < 3; ++i) {
                    const VertexData& vertex = faces[face].vertex.at(piece[i]);
                    triangle.points[i] = vertex.position();
                    triangle.source_vertices[i] = piece[i];
                    for (const std::pair<const std::string, double>& attribute : vertex.attributes.map()) {
                        if (attribute.first.starts_with("tessellation_origin/")) {
                            std::istringstream text(attribute.first.substr(20));
                            size_t origin;
                            if (text >> origin)
                                triangle.source_vertices[i] = origin;
                        }
                        if (attribute.first.starts_with("brep_vertex/")) {
                            std::istringstream text(attribute.first.substr(12));
                            int vertex_index;
                            if (text >> vertex_index && vertex_index >= 0)
                                triangle.boundary_vertices[i].insert(vertex_index);
                        }
                        const bool sample = attribute.first.starts_with("brep_edge/");
                        const bool interval = attribute.first.starts_with("brep_edge_interval/");
                        if (!sample && !interval)
                            continue;
                        std::istringstream text(attribute.first.substr(sample ? 10 : 19));
                        int edge;
                        char separator;
                        if (text >> edge >> separator && separator == '/' && edge >= 0)
                            triangle.boundary_edges[i].insert(edge);
                    }
                    if (!mesh_parametric_metadata::read(faces[face], polygon.first, piece[i], triangle.uv[i]))
                        return fail("missing_uv");
                    const double error = surface.point_at(triangle.uv[i][0], triangle.uv[i][1]).distance(vertex.position());
                    if (!std::isfinite(error) || error > allowed)
                        return fail("lift_residual");
                }
                triangles.push_back(std::move(triangle));
            }
        }
    }
    return true;
}

bool closed_edge_incidence(const Mesh& mesh) {
    std::map<std::pair<size_t, size_t>, std::pair<int, int>> uses;
    for (const std::pair<const size_t, std::vector<size_t>>& face : mesh.face) {
        const std::vector<size_t>& vertices = face.second;
        if (vertices.size() != 3)
            return false;
        for (size_t i = 0; i < 3; ++i) {
            const size_t first = vertices[i], last = vertices[(i + 1) % 3];
            if (first == last)
                return false;
            std::pair<int, int>& count = uses[{std::min(first, last), std::max(first, last)}];
            ++count.first;
            count.second += first < last ? 1 : -1;
        }
    }
    for (const std::pair<const std::pair<size_t, size_t>, std::pair<int, int>>& edge : uses)
        if (edge.second.first != 2 || edge.second.second != 0)
            return false;
    return !uses.empty();
}

Mesh joined(const std::vector<TessellationTriangle>& triangles) {
    Mesh result;
    std::map<std::pair<int, size_t>, size_t> local;
    std::map<std::tuple<int, double, double, double>, size_t> boundaries;
    std::map<std::tuple<int, double, double, double>, size_t> vertices;
    for (const TessellationTriangle& triangle : triangles) {
        std::vector<size_t> indices;
        for (int i = 0; i < 3; ++i) {
            const Point& point = triangle.points[i];
            const std::pair<int, size_t> source(triangle.face, triangle.source_vertices[i]);
            const auto existing = local.find(source);
            size_t key = existing == local.end() ? static_cast<size_t>(-1) : existing->second;
            for (int edge : triangle.boundary_edges[i]) {
                const auto found = boundaries.find({edge, point[0], point[1], point[2]});
                if (found != boundaries.end()) {
                    if (key != static_cast<size_t>(-1) && key != found->second)
                        throw std::runtime_error("Native tessellation boundary ancestry is inconsistent");
                    key = found->second;
                }
            }
            for (int vertex : triangle.boundary_vertices[i]) {
                const auto found = vertices.find({vertex, point[0], point[1], point[2]});
                if (found != vertices.end()) {
                    if (key != static_cast<size_t>(-1) && key != found->second)
                        throw std::runtime_error("Native tessellation vertex ancestry is inconsistent");
                    key = found->second;
                }
            }
            if (key == static_cast<size_t>(-1))
                key = result.add_vertex(point);
            local[source] = key;
            for (int edge : triangle.boundary_edges[i])
                boundaries[{edge, point[0], point[1], point[2]}] = key;
            for (int vertex : triangle.boundary_vertices[i])
                vertices[{vertex, point[0], point[1], point[2]}] = key;
            indices.push_back(key);
        }
        if (indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2]
            || !result.add_face(indices))
            throw std::runtime_error("Native tessellation has a degenerate triangle");
    }
    return result;
}
}

Mesh BRep::tessellate(double deflection, double angle) const {
    if (!std::isfinite(deflection) || !(deflection > 0)
        || !std::isfinite(angle) || !(angle > 0))
        throw std::invalid_argument("Native tessellation requires finite positive deflection and angle");
    if (!is_valid())
        throw std::invalid_argument("Native tessellation requires valid boundary geometry");
    if (m_faces.empty())
        return {};
    Point low, high;
    bool first = true;
    for (const NurbsSurface& surface : m_surfaces)
        for (int i = 0; i < surface.m_cv_count[0]; ++i)
            for (int j = 0; j < surface.m_cv_count[1]; ++j) {
                const Point point = surface.get_cv(i, j);
                for (int d = 0; d < 3; ++d) {
                    low[d] = first ? point[d] : std::min(low[d], point[d]);
                    high[d] = first ? point[d] : std::max(high[d], point[d]);
                }
                first = false;
            }
    const double scale = low.distance(high);
    if (!std::isfinite(scale) || !(scale > 0))
        throw std::runtime_error("Native tessellation has unresolved geometry scale");
    native_tessellate_diagnostics::Progress progress;
    progress.phase("call");
    for (int pass = 0; pass < 10; ++pass) {
        progress.state = {};
        progress.state.pass = pass;
        progress.phase("meshing");
        const double tightening = std::ldexp(1.0, -pass);
        const mesh_parametric_metadata::Capture capture;
        const std::vector<Mesh> faces = face_meshes_q(true,
            std::min(angle, 1.0) * (180 / std::acos(-1.0)) * .5 * tightening,
            (deflection / scale) * .5 * tightening);
        progress.phase("metadata");
        std::vector<TessellationTriangle> triangles;
        if (!proposed_triangles(*this, faces, deflection * .125, triangles,
                native_tessellate_diagnostics::observer ? &progress.state : nullptr)) {
            progress.phase("metadata_failed");
            continue;
        }
        progress.state.triangles = triangles.size();
        progress.phase("joining");
        if (triangles.size() > 2000000)
            throw std::runtime_error("Native tessellation exceeded its triangle budget");
        Mesh result = joined(triangles);
        progress.phase("certification");
        bool acceptable = true;
        for (const TessellationTriangle& triangle : triangles) {
            const NurbsSurface& surface = m_surfaces[m_faces[triangle.face].surface_index];
            if (native_tessellate_diagnostics::observer) {
                ++progress.state.visited;
                progress.state.face = triangle.face;
                progress.state.uv = triangle.uv;
                if (progress.state.visited == 1 || progress.state.visited % 64 == 0)
                    progress.checkpoint();
            }
            const bool certified = native_tessellate::triangle(surface, triangle.uv, triangle.points, deflection, angle);
            if (certified) {
                if (native_tessellate_diagnostics::observer)
                    ++progress.state.certified;
            } else {
                if (native_tessellate_diagnostics::observer)
                    ++progress.state.sampled;
                if (!measured(surface, triangle, deflection, angle)) {
                    acceptable = false;
                    break;
                }
                if (native_tessellate_diagnostics::observer)
                    ++progress.state.measured_passed;
            }
        }
        progress.phase("incidence");
        if (acceptable && (!is_solid() || closed_edge_incidence(result))) {
            progress.phase("complete");
            return result;
        }
        progress.phase("retry");
    }
    throw std::runtime_error("Native tessellation could not resolve the requested chord and angular tolerances");
}
}
