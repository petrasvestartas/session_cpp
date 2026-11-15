#include "bop_builder_solid.h"
#include "bop_ds.h"
#include "brep_classify.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <numeric>

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

/// The divergence theorem over a grid of each face's parameters: enough for the sign and the size of a shell
double signed_volume(const BRep& brep, const std::vector<BRepRef>& shell) {
    double volume = 0.0;
    for (const BRepRef& face : shell) {
        const Region region = face_region(brep, face.index, 1e-7);
        const NurbsSurface& s = brep.m_surfaces[brep.m_faces[face.index].surface_index];
        const int n = 24;
        const double du = (region.hi[0] - region.lo[0]) / n, dv = (region.hi[1] - region.lo[1]) / n;
        const double sign = face.orientation == BRepOrientation::Reversed ? -1.0 : 1.0;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                const double u = region.lo[0] + (i + 0.5) * du, v = region.lo[1] + (j + 0.5) * dv;
                if (region.state(Point(u, v, 0)) != 1)
                    continue;
                const std::vector<Vector> d = s.evaluate(u, v, 1);
                volume += sign * Vector(d[0][0], d[0][1], d[0][2]).dot(d[1].cross(d[2])) * du * dv / 3.0;
            }
    }
    return volume;
}

void build_solids(BRep& result, const std::vector<BRepRef>& faces, const std::vector<int>& labels) {
    const size_t n = faces.size();
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    const auto root = [&](int k) {
        while (parent[k] != k)
            k = parent[k] = parent[parent[k]];
        return k;
    };
    std::map<std::pair<int, int>, std::vector<int>> by_edge;
    for (size_t k = 0; k < n; ++k)
        for (const BRepRef& wire : result.m_faces[faces[k].index].wires)
            for (const BRepRef& use : result.m_wires[wire.index].edges)
                if (!result.m_edges[use.index].degenerated)
                    by_edge[{labels[k], use.index}].push_back(static_cast<int>(k));
    for (const auto& [unused, slots] : by_edge)
        for (int k : slots)
            parent[root(k)] = root(slots[0]);
    std::map<int, std::vector<BRepRef>> shells;
    for (size_t k = 0; k < n; ++k)
        shells[root(static_cast<int>(k))].push_back(faces[k]);
    std::vector<std::vector<BRepRef>> list, outer, holes;
    std::vector<double> volumes;
    for (auto& [unused, shell] : shells) {
        list.push_back(shell);
        volumes.push_back(shells.size() > 1 ? signed_volume(result, shell) : 1.0);
    }
    const size_t largest = std::max_element(volumes.begin(), volumes.end(), [](double x, double y) { return std::abs(x) < std::abs(y); }) - volumes.begin();
    std::vector<double> outer_volumes;
    for (size_t k = 0; k < list.size(); ++k)
        if ((volumes[k] > 0) == (volumes[largest] > 0)) {
            outer.push_back(list[k]);
            outer_volumes.push_back(std::abs(volumes[k]));
        } else
            holes.push_back(list[k]);
    std::vector<std::vector<int>> inner(outer.size());
    for (size_t h = 0; h < holes.size(); ++h) {
        Point uv;
        const Region region = face_region(result, holes[h][0].index, 1e-7);
        if (!region.interior(uv))
            continue;
        const NurbsSurface& s = result.m_surfaces[result.m_faces[holes[h][0].index].surface_index];
        const Point p = s.point_at(uv[0], uv[1]);
        int best = -1;
        for (size_t o = 0; o < outer.size(); ++o) {
            if (best >= 0 && outer_volumes[o] >= outer_volumes[best])
                continue;
            const BRep part = subset(result, outer[o]);
            brep_classify::State state = brep_classify::State::Unknown;
            try {
                state = brep_classify::point(part, p, 1e-7);
            } catch (const std::exception&) {
            }
            if (state == brep_classify::State::Inside || (state == brep_classify::State::Unknown && std::abs(winding(part.tessellate(1e-2 * std::cbrt(outer_volumes[o]), 0.8), p)) > 0.5))
                best = static_cast<int>(o);
        }
        if (best >= 0)
            inner[best].push_back(static_cast<int>(h));
    }
    for (size_t o = 0; o < outer.size(); ++o) {
        std::vector<BRepRef> solid{{result.add_shell(outer[o]), BRepOrientation::Forward}};
        for (int h : inner[o])
            solid.push_back({result.add_shell(holes[h]), BRepOrientation::Forward});
        result.add_solid(solid);
    }
}

}
