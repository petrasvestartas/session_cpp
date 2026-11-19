#include "bop_builder_solid.h"
#include "bop_ds.h"
#include "brep_classify.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <numeric>
#include <tuple>

namespace session_cpp::bop {

BRep subset(const BRep& brep, const std::vector<BRepRef>& faces) {
    BRep part;
    part.m_surfaces = brep.m_surfaces;
    part.m_curves_3d = brep.m_curves_3d;
    part.m_curves_2d = brep.m_curves_2d;
    part.m_vertices = brep.m_vertices;
    part.m_edges = brep.m_edges;
    part.m_wires = brep.m_wires;
    std::vector<BRepRef> refs;
    for (const BRepRef& face : faces) {
        refs.push_back({part.face_count(), face.orientation});
        part.m_faces.push_back(brep.m_faces[face.index]);
    }
    part.add_solid({{part.add_shell(refs), BRepOrientation::Forward}});
    return part;
}

double winding(const Mesh& mesh, const Point& p) {
    const auto [points, faces] = mesh.to_vertices_and_faces();
    double sum = 0.0;
    for (const auto& face : faces)
        for (size_t k = 1; k + 1 < face.size(); ++k) {
            const Point& a = points[face[0]];
            const Point& b = points[face[k]];
            const Point& c = points[face[k + 1]];
            const Vector x(a[0] - p[0], a[1] - p[1], a[2] - p[2]), y(b[0] - p[0], b[1] - p[1], b[2] - p[2]), z(c[0] - p[0], c[1] - p[1], c[2] - p[2]);
            const double lx = x.magnitude(), ly = y.magnitude(), lz = z.magnitude();
            const double numerator = x.dot(y.cross(z));
            const double denominator = lx * ly * lz + x.dot(y) * lz + y.dot(z) * lx + z.dot(x) * ly;
            sum += 2.0 * std::atan2(numerator, denominator);
        }
    return sum / (4.0 * std::numbers::pi);
}

void build_solids(BRep& result, const std::vector<BRepRef>& faces) {
    const size_t n = faces.size();
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    const auto root = [&](int k) {
        while (parent[k] != k)
            k = parent[k] = parent[parent[k]];
        return k;
    };
    std::map<int, std::vector<std::tuple<int, BRepOrientation, BRepOrientation>>> by_edge;
    for (size_t k = 0; k < n; ++k)
        for (const BRepRef& wire : result.m_faces[faces[k].index].wires)
            for (const BRepRef& use : result.m_wires[wire.index].edges)
                if (!result.m_edges[use.index].degenerated)
                    by_edge[use.index].push_back({static_cast<int>(k), brep_compose(faces[k].orientation, use.orientation), use.orientation});
    for (const auto& [edge, uses] : by_edge) {
        if (uses.size() <= 2) {
            for (const auto& use : uses)
                parent[root(std::get<0>(use))] = root(std::get<0>(uses[0]));
            continue;
        }
        // GetFaceOff: around the edge, a face whose use runs against the edge closes the material with the next face counterclockwise.
        const Span c = edge_span(result, edge);
        const Point p = c.at(0.5);
        Vector axis = c.curve->tangent_at(0.5 * (c.t0 + c.t1));
        axis.normalize_self();
        std::vector<std::pair<double, int>> around;
        Vector reference;
        for (size_t i = 0; i < uses.size(); ++i) {
            const auto& [k, orientation, own] = uses[i];
            const BRepFace& face = result.m_faces[faces[k].index];
            const Span pc = pcurve_span(result, edge, faces[k].index, own);
            const NurbsSurface& s = result.m_surfaces[face.surface_index].nurbs();
            Point uv;
            if (pc.curve)
                uv = pc.at(0.5);
            else {
                const auto [u, v] = s.closest_parameters(p);
                uv = Point(u, v, 0);
            }
            Vector normal = s.normal_at(uv[0], uv[1]);
            normal.normalize_self();
            if (faces[k].orientation == BRepOrientation::Reversed)
                normal *= -1.0;
            const double sense = orientation == BRepOrientation::Reversed ? -1.0 : 1.0;
            Vector inward = normal.cross(axis * sense);
            if (i == 0)
                reference = inward;
            around.push_back({std::atan2(reference.cross(inward).dot(axis), reference.dot(inward)), static_cast<int>(i)});
        }
        std::sort(around.begin(), around.end());
        for (size_t i = 0; i < around.size(); ++i) {
            const auto& [k, orientation, own] = uses[around[i].second];
            if (orientation == BRepOrientation::Reversed)
                parent[root(k)] = root(std::get<0>(uses[around[(i + 1) % around.size()].second]));
        }
    }
    std::map<int, std::vector<BRepRef>> shells;
    for (size_t k = 0; k < n; ++k)
        shells[root(static_cast<int>(k))].push_back(faces[k]);
    // IsHole: a shell inside another one bounds a cavity of the smallest shell around it.
    std::vector<std::vector<BRepRef>> list;
    std::vector<Box> boxes;
    for (auto& [unused, shell] : shells) {
        list.push_back(shell);
        Box box;
        for (const BRepRef& face : shell)
            for (const BRepRef& wire : result.m_faces[face.index].wires)
                for (const BRepRef& use : result.m_wires[wire.index].edges)
                    for (int v : {result.m_edges[use.index].start_vertex, result.m_edges[use.index].end_vertex})
                        box.add(result.m_vertices[v].point);
        boxes.push_back(box);
    }
    const auto size = [](const Box& b) { return (b.hi[0] - b.lo[0]) * (b.hi[1] - b.lo[1]) * (b.hi[2] - b.lo[2]); };
    const auto inside = [](const Box& in, const Box& out) {
        for (int d = 0; d < 3; ++d)
            if (in.lo[d] < out.lo[d] || in.hi[d] > out.hi[d])
                return false;
        return true;
    };
    std::vector<int> around(list.size(), -1);
    for (size_t h = 0; h < list.size(); ++h) {
        Point p;
        bool sampled = false;
        for (size_t i = 0; i < list[h].size() && !sampled; ++i) {
            Point uv;
            const Region region = face_region(result, list[h][i].index, 1e-7);
            if (region.interior(uv)) {
                p = result.m_surfaces[result.m_faces[list[h][i].index].surface_index].nurbs().point_at(uv[0], uv[1]);
                sampled = true;
            }
        }
        for (size_t o = 0; o < list.size() && sampled; ++o) {
            if (o == h || !inside(boxes[h], boxes[o]) || size(boxes[o]) <= size(boxes[h]) || (around[h] >= 0 && size(boxes[o]) >= size(boxes[around[h]])))
                continue;
            const BRep part = subset(result, list[o]);
            brep_classify::State state = brep_classify::State::Unknown;
            try {
                state = brep_classify::point(part, p, 1e-7);
            } catch (const std::exception&) {
            }
            if (state == brep_classify::State::Unknown)
                try {
                    state = std::abs(winding(part.tessellate(1e-2 * std::cbrt(size(boxes[o])), 0.8), p)) > 0.5 ? brep_classify::State::Inside : brep_classify::State::Outside;
                } catch (const std::exception&) {
                }
            if (state == brep_classify::State::Inside)
                around[h] = static_cast<int>(o);
        }
    }
    std::vector<std::vector<BRepRef>> outer;
    std::vector<std::vector<int>> inner;
    std::vector<int> slot(list.size(), -1);
    for (size_t k = 0; k < list.size(); ++k)
        if (around[k] < 0 || around[around[k]] >= 0) {
            slot[k] = static_cast<int>(outer.size());
            outer.push_back(list[k]);
            inner.push_back({});
        }
    std::vector<std::vector<BRepRef>> holes;
    for (size_t k = 0; k < list.size(); ++k)
        if (slot[k] < 0) {
            inner[slot[around[k]]].push_back(static_cast<int>(holes.size()));
            holes.push_back(list[k]);
        }
    for (size_t o = 0; o < outer.size(); ++o) {
        std::vector<BRepRef> solid{{result.add_shell(outer[o]), BRepOrientation::Forward}};
        for (int h : inner[o])
            solid.push_back({result.add_shell(holes[h]), BRepOrientation::Forward});
        result.add_solid(solid);
    }
}

}
