#include "obj.h"
#include "mesh.h"
#include "element.h"
#include "rtree.h"
#include "aabb.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

namespace session_cpp { namespace obj {

void write_obj(const Mesh& mesh, const std::string& filepath) {
    auto vf = mesh.to_vertices_and_faces();
    const auto& vertices = vf.first;
    const auto& faces = vf.second;

    std::ofstream out(filepath);
    if (!out.is_open()) {
        return; // Failed to open file
    }
    
    for (const auto& p : vertices) {
        out << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
    }
    for (const auto& face : faces) {
        if (face.size() < 3) continue;
        out << "f";
        for (auto i : face) {
            out << " " << (i + 1);
        }
        out << "\n";
    }
    
    out.close(); // Explicitly close and flush
}

Mesh read_obj(const std::string& filepath) {
    std::ifstream in(filepath);
    std::string line;
    std::vector<Point> verts;
    std::vector<std::vector<size_t>> faces;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.rfind("v ", 0) == 0) {
            std::istringstream iss(line);
            char v; double x, y, z; iss >> v >> x >> y >> z;
            verts.emplace_back(x, y, z);
        } else if (line.rfind("f ", 0) == 0) {
            std::istringstream iss(line.substr(2));
            std::string tok;
            std::vector<size_t> face;
            while (iss >> tok) {
                auto slash = tok.find('/');
                std::string sidx = (slash == std::string::npos) ? tok : tok.substr(0, slash);
                if (sidx.empty()) continue;
                long long idx = std::stoll(sidx);
                if (idx == 0) continue;
                size_t vidx = (idx > 0) ? static_cast<size_t>(idx - 1)
                                        : static_cast<size_t>(static_cast<long long>(verts.size()) + idx);
                face.push_back(vidx);
            }
            if (face.size() >= 3) faces.push_back(std::move(face));
        }
    }

    Mesh mesh;
    std::vector<size_t> vkeys; vkeys.reserve(verts.size());
    for (const auto& p : verts) vkeys.push_back(mesh.add_vertex(p));
    for (const auto& f : faces) {
        std::vector<size_t> vlist; vlist.reserve(f.size());
        for (auto i : f) vlist.push_back(vkeys[i]);
        mesh.add_face(vlist);
    }
    return mesh;
}

std::vector<Polyline> read_obj_polylines(const std::string& filepath) {
    std::ifstream in(filepath);
    std::string line;
    std::vector<Point> verts;
    std::vector<Polyline> polylines;
    std::vector<int> curv_indices;
    bool in_curv = false;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.rfind("v ", 0) == 0) {
            std::istringstream iss(line);
            char v; double x, y, z; iss >> v >> x >> y >> z;
            verts.emplace_back(x, y, z);
        } else if (line.rfind("curv ", 0) == 0) {
            std::istringstream iss(line.substr(5));
            double u0, u1; iss >> u0 >> u1;
            curv_indices.clear();
            int idx;
            while (iss >> idx) curv_indices.push_back(idx);
            in_curv = true;
        } else if (line.rfind("end", 0) == 0 && in_curv) {
            if (!curv_indices.empty()) {
                std::vector<Point> pts;
                for (int idx : curv_indices) {
                    size_t vi = static_cast<size_t>(idx - 1);
                    if (vi < verts.size()) pts.push_back(verts[vi]);
                }
                if (pts.size() >= 3) polylines.emplace_back(pts);
            }
            in_curv = false;
        }
    }
    return polylines;
}

std::vector<std::pair<int,int>> pair_polylines(const std::vector<Polyline>& polylines, double search_radius) {
    size_t NP = polylines.size();
    std::vector<Point> centroids(NP);
    std::vector<Vector> normals(NP);
    std::vector<AABB> aabbs(NP);

    // Helper to strip closing point
    auto open_pts = [](const Polyline& pl) {
        auto pts = pl.get_points();
        if (pts.size() > 3) {
            auto& f = pts.front(); auto& l = pts.back();
            if (std::abs(f[0]-l[0])<1e-6 && std::abs(f[1]-l[1])<1e-6 && std::abs(f[2]-l[2])<1e-6)
                pts.pop_back();
        }
        return pts;
    };

    RTree<int, double, 3> tree;
    for (size_t i = 0; i < NP; i++) {
        auto pts = open_pts(polylines[i]);
        double cx=0,cy=0,cz=0;
        for (auto& p : pts) { cx+=p[0]; cy+=p[1]; cz+=p[2]; }
        centroids[i] = Point(cx/pts.size(), cy/pts.size(), cz/pts.size());
        normals[i] = PlateElement::polygon_normal(pts);
        aabbs[i] = AABB::from_polyline(polylines[i], search_radius);
        double mn[3]={aabbs[i].cx-aabbs[i].hx, aabbs[i].cy-aabbs[i].hy, aabbs[i].cz-aabbs[i].hz};
        double mx[3]={aabbs[i].cx+aabbs[i].hx, aabbs[i].cy+aabbs[i].hy, aabbs[i].cz+aabbs[i].hz};
        tree.insert(mn, mx, (int)i);
    }

    std::vector<bool> paired(NP, false);
    std::vector<std::pair<int,int>> pairs;
    for (size_t i = 0; i < NP; i++) {
        if (paired[i]) continue;
        int best = -1; double best_d = 1e30;
        double mn[3]={aabbs[i].cx-aabbs[i].hx, aabbs[i].cy-aabbs[i].hy, aabbs[i].cz-aabbs[i].hz};
        double mx[3]={aabbs[i].cx+aabbs[i].hx, aabbs[i].cy+aabbs[i].hy, aabbs[i].cz+aabbs[i].hz};
        tree.search(mn, mx, [&](int j) {
            if (j <= (int)i || paired[j]) return true;
            double dot = normals[i][0]*normals[j][0]+normals[i][1]*normals[j][1]+normals[i][2]*normals[j][2];
            if (std::abs(dot) < 0.7) return true;
            double dx=centroids[i][0]-centroids[j][0], dy=centroids[i][1]-centroids[j][1], dz=centroids[i][2]-centroids[j][2];
            double d = dx*dx+dy*dy+dz*dz;
            if (d < best_d) { best_d = d; best = j; }
            return true;
        });
        if (best >= 0) { pairs.push_back({(int)i, best}); paired[i] = paired[best] = true; }
    }
    return pairs;
}

} } // namespace session_cpp::obj
