#include <filesystem>
#include <fstream>
#include <chrono>
#include "session.h"
#include "element.h"
#include "polyline.h"
#include "line.h"
#include "rtree.h"
#include "aabb.h"
#include "obb.h"
#include "json.h"
using namespace session_cpp;

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto json_in = (base / "session_data" / "WoodStep2_data.json").string();
    auto pb_path = (base / "session_data" / "WoodStep3.pb").string();
    auto json_out = (base / "session_data" / "WoodStep3_data.json").string();

    // 1. Load elements from Step 2
    std::ifstream fin(json_in);
    nlohmann::json data = nlohmann::json::parse(fin);
    std::vector<PlateElement> elements;
    nlohmann::json elem_array_out = nlohmann::json::array();
    for (auto& e : data["elements"]) {
        std::vector<Point> polygon;
        for (auto& p : e["polygon"])
            polygon.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        double thickness = e["thickness"].get<double>();
        std::string name = e["name"].get<std::string>();
        elements.emplace_back(polygon, thickness, name);
        elem_array_out.push_back(e);
    }
    fmt::print("Loaded {} elements\n", elements.size());

    auto t0 = std::chrono::high_resolution_clock::now();

    // 2. Compute AABB + OBB for each element
    size_t N = elements.size();
    std::vector<AABB> aabbs(N);
    std::vector<OBB> obbs(N);
    for (size_t i = 0; i < N; i++) {
        auto pls = elements[i].compute_polylines();
        std::vector<Point> all_pts;
        for (auto& pl : pls)
            for (auto& p : pl.get_points())
                all_pts.push_back(p);
        aabbs[i] = AABB::from_points(all_pts);
        obbs[i] = OBB::from_points(all_pts);
    }

    // 3. Build RTree
    RTree<int, double, 3> tree;
    for (size_t i = 0; i < N; i++) {
        double mn[3] = {aabbs[i].cx - aabbs[i].hx, aabbs[i].cy - aabbs[i].hy, aabbs[i].cz - aabbs[i].hz};
        double mx[3] = {aabbs[i].cx + aabbs[i].hx, aabbs[i].cy + aabbs[i].hy, aabbs[i].cz + aabbs[i].hz};
        tree.insert(mn, mx, static_cast<int>(i));
    }

    // 4. Search: AABB broad phase + OBB narrow phase
    std::vector<int> adjacency;
    for (size_t i = 0; i < N; i++) {
        double mn[3] = {aabbs[i].cx - aabbs[i].hx, aabbs[i].cy - aabbs[i].hy, aabbs[i].cz - aabbs[i].hz};
        double mx[3] = {aabbs[i].cx + aabbs[i].hx, aabbs[i].cy + aabbs[i].hy, aabbs[i].cz + aabbs[i].hz};

        auto callback = [&](int j) -> bool {
            if ((int)i < j && obbs[i].collides_with(obbs[j])) {
                adjacency.push_back(static_cast<int>(i));
                adjacency.push_back(j);
                adjacency.push_back(-1);
                adjacency.push_back(-1);
            }
            return true;
        };
        tree.search(mn, mx, callback);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    fmt::print("RTree+OBB: {} adjacency pairs in {:.1f} ms (precompute+search)\n", adjacency.size() / 4, ms);

    // 4b. Combined benchmark: RTree + face-to-face using raw arrays (no Polyline overhead)
    {
        auto tc0 = std::chrono::high_resolution_clock::now();

        // Lightweight face cache: just coords + plane data as flat doubles
        // Each element has nf faces. Each face has np points (4 for quads).
        // Store: coords[face][point*3+dim], normal[face*3+dim], origin[face*3+dim], aabb[face]
        struct FaceData {
            int nf;                         // number of faces
            std::vector<std::vector<double>> coords; // [face][pt*3+xyz]
            std::vector<int> npts;          // points per face (open, no closing)
            std::vector<double> normals;    // [face*3+xyz]
            std::vector<double> origins;    // [face*3+xyz]
            std::vector<AABB> aabbs;        // [face]
        };
        std::vector<FaceData> fd(N);
        for (size_t ei = 0; ei < N; ei++) {
            auto pls = elements[ei].compute_polylines();
            auto plns = elements[ei].compute_planes();
            int nf = (int)pls.size();
            fd[ei].nf = nf;
            fd[ei].coords.resize(nf);
            fd[ei].npts.resize(nf);
            fd[ei].normals.resize(nf * 3);
            fd[ei].origins.resize(nf * 3);
            fd[ei].aabbs.resize(nf);
            for (int f = 0; f < nf; f++) {
                auto pts = pls[f].get_points();
                int np = (int)pts.size();
                // Remove closing point
                if (np > 3 && std::abs(pts[np-1][0]-pts[0][0])<1e-6 &&
                    std::abs(pts[np-1][1]-pts[0][1])<1e-6 &&
                    std::abs(pts[np-1][2]-pts[0][2])<1e-6) np--;
                fd[ei].npts[f] = np;
                fd[ei].coords[f].resize(np * 3);
                double minx=1e30,miny=1e30,minz=1e30,maxx=-1e30,maxy=-1e30,maxz=-1e30;
                for (int p = 0; p < np; p++) {
                    double x=pts[p][0], y=pts[p][1], z=pts[p][2];
                    fd[ei].coords[f][p*3]=x; fd[ei].coords[f][p*3+1]=y; fd[ei].coords[f][p*3+2]=z;
                    if(x<minx)minx=x; if(y<miny)miny=y; if(z<minz)minz=z;
                    if(x>maxx)maxx=x; if(y>maxy)maxy=y; if(z>maxz)maxz=z;
                }
                double inf = 25.0;
                fd[ei].aabbs[f] = AABB{(minx+maxx)*0.5,(miny+maxy)*0.5,(minz+maxz)*0.5,
                                       (maxx-minx)*0.5+inf,(maxy-miny)*0.5+inf,(maxz-minz)*0.5+inf};
                Vector n = plns[f].z_axis(); Point o = plns[f].origin();
                fd[ei].normals[f*3]=n[0]; fd[ei].normals[f*3+1]=n[1]; fd[ei].normals[f*3+2]=n[2];
                fd[ei].origins[f*3]=o[0]; fd[ei].origins[f*3+1]=o[1]; fd[ei].origins[f*3+2]=o[2];
            }
        }

        auto tc1 = std::chrono::high_resolution_clock::now();

        int combined_joints = 0;
        for (size_t idx = 0; idx < adjacency.size(); idx += 4) {
            int a = adjacency[idx], b = adjacency[idx+1];
            auto& ca = fd[a]; auto& cb = fd[b];
            bool found = false;
            for (int i = 0; i < ca.nf && !found; i++) {
                double nix=ca.normals[i*3], niy=ca.normals[i*3+1], niz=ca.normals[i*3+2];
                for (int j = 0; j < cb.nf; j++) {
                    double dot = nix*cb.normals[j*3]+niy*cb.normals[j*3+1]+niz*cb.normals[j*3+2];
                    if (std::abs(std::abs(dot)-1.0) > 0.01) continue;
                    double dx=cb.origins[j*3]-ca.origins[i*3];
                    double dy=cb.origins[j*3+1]-ca.origins[i*3+1];
                    double dz=cb.origins[j*3+2]-ca.origins[i*3+2];
                    if (std::abs(dx*nix+dy*niy+dz*niz) > 50.0) continue;
                    if (!ca.aabbs[i].intersects(cb.aabbs[j])) continue;
                    combined_joints++;
                    found = true;
                    break;
                }
            }
        }

        auto tc2 = std::chrono::high_resolution_clock::now();
        double precomp_ms = std::chrono::duration<double, std::milli>(tc1 - tc0).count();
        double detect_ms = std::chrono::duration<double, std::milli>(tc2 - tc1).count();
        double total_ms = ms + std::chrono::duration<double, std::milli>(tc2 - tc0).count();
        fmt::print("Combined: {:.1f} ms total (RTree={:.1f} + precompute={:.1f} + detect={:.1f}), joints={}\n",
                   total_ms, ms, precomp_ms, detect_ms, combined_joints);
    }

    // 5. Visualize — join all plates into ONE mesh, all adj lines into polylines
    Session session("WoodStep3_Adjacency");

    // Build one combined mesh from all plates
    std::vector<Point> all_verts;
    std::vector<std::vector<size_t>> all_faces;
    for (auto& e : data["elements"]) {
        std::vector<Point> bot, top;
        for (auto& p : e["polygon"]) bot.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        if (e.contains("polygon_top"))
            for (auto& p : e["polygon_top"]) top.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        else top = bot;
        size_t np = std::min(bot.size(), top.size());
        Vector na = PlateElement::polygon_normal(bot);
        double dx = 0, dy = 0, dz = 0;
        for (size_t k = 0; k < np; k++) { dx += top[k][0]-bot[k][0]; dy += top[k][1]-bot[k][1]; dz += top[k][2]-bot[k][2]; }
        if (dx*na[0]+dy*na[1]+dz*na[2] < 0) std::swap(bot, top);
        size_t base = all_verts.size();
        for (size_t k = 0; k < np; k++) all_verts.push_back(bot[k]);
        for (size_t k = 0; k < np; k++) all_verts.push_back(top[k]);
        std::vector<size_t> bf; for (size_t k = np; k-- > 0;) bf.push_back(base+k); all_faces.push_back(bf);
        std::vector<size_t> tf; for (size_t k = 0; k < np; k++) tf.push_back(base+np+k); all_faces.push_back(tf);
        for (size_t k = 0; k < np; k++) { size_t k1=(k+1)%np; all_faces.push_back({base+k,base+k1,base+k1+np,base+k+np}); }
    }
    auto g_mesh = session.add_group("Elements");
    auto m = std::make_shared<Mesh>(Mesh::from_vertices_and_faces(all_verts, all_faces));
    session.add(session.add_mesh(m), g_mesh);

    // Adjacency lines as polylines (faster in Rhino than individual Lines)
    auto g_adj = session.add_group("Adjacency");
    for (size_t i = 0; i < adjacency.size(); i += 4) {
        int a = adjacency[i], b = adjacency[i + 1];
        Point ca = elements[a].point();
        Point cb = elements[b].point();
        auto pl = std::make_shared<Polyline>(std::vector<Point>{ca, cb});
        session.add(session.add_polyline(pl), g_adj);
    }
    session.pb_dump(pb_path);

    // 6. Write intermediate data
    nlohmann::ordered_json out_data;
    out_data["elements"] = elem_array_out;
    out_data["adjacency"] = adjacency;
    std::ofstream fout(json_out);
    fout << out_data.dump(2);

    fmt::print("Wrote {} and {}\n", pb_path, json_out);
    return 0;
}
