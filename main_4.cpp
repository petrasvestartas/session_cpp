#include <filesystem>
#include <fstream>
#include <chrono>
#include "session.h"
#include "element.h"
#include "polyline.h"
#include "plane.h"
#include "line.h"
#include "aabb.h"
#include "json.h"
using namespace session_cpp;

struct Joint {
    int id;
    int element_a, element_b;
    int face_a, face_b;
    int type; // 0=side-side, 1=side-top, 2=top-top
    Polyline joint_area;
};

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto json_in = (base / "session_data" / "WoodStep3_data.json").string();
    auto pb_path = (base / "session_data" / "WoodStep4.pb").string();
    auto json_out = (base / "session_data" / "WoodStep4_data.json").string();

    // 1. Load
    std::ifstream fin(json_in);
    nlohmann::json data = nlohmann::json::parse(fin);
    std::vector<int> adjacency = data["adjacency"].get<std::vector<int>>();
    size_t N = data["elements"].size();

    auto t0 = std::chrono::high_resolution_clock::now();

    // 2. Create PlateElements from actual bottom+top polylines
    std::vector<PlateElement> elems;
    elems.reserve(N);
    for (size_t ei = 0; ei < N; ei++) {
        auto& e = data["elements"][ei];
        std::vector<Point> bot, top;
        for (auto& p : e["polygon"]) bot.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        if (e.contains("polygon_top"))
            for (auto& p : e["polygon_top"]) top.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        else top = bot;
        elems.emplace_back(bot, top, e.value("name", "plate"));
    }

    // Precompute polylines + planes (cached by PlateElement)
    std::vector<std::vector<Polyline>> all_polys(N);
    std::vector<std::vector<Plane>> all_planes(N);
    for (size_t i = 0; i < N; i++) {
        all_polys[i] = elems[i].compute_polylines();
        all_planes[i] = elems[i].compute_planes();
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    // 3. Face-to-face: coplanar check → 3D boolean intersection
    std::vector<Joint> joints;
    joints.reserve(2000);
    int joint_id = 0;

    for (size_t idx = 0; idx < adjacency.size(); idx += 4) {
        int a = adjacency[idx], b = adjacency[idx+1];

        bool found = false;
        for (int i = 0; i < (int)all_planes[a].size() && !found; i++) {
            for (int j = 0; j < (int)all_planes[b].size(); j++) {
                if (!Plane::is_coplanar(all_planes[a][i], all_planes[b][j], false))
                    continue;

                // Build plane from face_i's first edge for robust 2D projection
                auto pts_i = all_polys[a][i].get_points();
                Vector edge(pts_i[1][0]-pts_i[0][0], pts_i[1][1]-pts_i[0][1], pts_i[1][2]-pts_i[0][2]);
                edge.normalize_self();
                Vector yax = all_planes[a][i].z_axis().cross(edge);
                yax.normalize_self();
                Plane pln(pts_i[0], edge, yax);

                auto results = Polyline::boolean_op(all_polys[a][i], all_polys[b][j], pln, 0);
                if (results.empty() || results[0].point_count() < 3) continue;

                joints.push_back({joint_id++, a, b, i, j,
                    (i>1?0:1)+(j>1?0:1), std::move(results[0])});
                found = true;
                break;
            }
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    fmt::print("Precompute: {:.1f} ms\n", std::chrono::duration<double,std::milli>(t1-t0).count());
    fmt::print("Detect: {} joints in {:.1f} ms\n", joints.size(), std::chrono::duration<double,std::milli>(t2-t1).count());
    fmt::print("Total: {:.1f} ms\n", std::chrono::duration<double,std::milli>(t2-t0).count());
    int t0c=0,t1c=0,t2c=0;
    for(auto&j:joints){if(j.type==0)t0c++;else if(j.type==1)t1c++;else t2c++;}
    fmt::print("  side-side: {}, side-top: {}, top-top: {}\n", t0c, t1c, t2c);

    // 4. Visualize — merged meshes only
    Session session("WoodStep4_Joints");

    // All plates as one mesh
    {
        std::vector<Point> av; std::vector<std::vector<size_t>> af;
        for (auto& e : data["elements"]) {
            std::vector<Point> bot, top;
            for (auto& p : e["polygon"]) bot.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
            if (e.contains("polygon_top"))
                for (auto& p : e["polygon_top"]) top.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
            else top = bot;
            size_t np = std::min(bot.size(), top.size());
            Vector na = PlateElement::polygon_normal(bot);
            double dx=0,dy=0,dz=0;
            for(size_t k=0;k<np;k++){dx+=top[k][0]-bot[k][0];dy+=top[k][1]-bot[k][1];dz+=top[k][2]-bot[k][2];}
            if(dx*na[0]+dy*na[1]+dz*na[2]<0) std::swap(bot,top);
            size_t base=av.size();
            for(size_t k=0;k<np;k++) av.push_back(bot[k]);
            for(size_t k=0;k<np;k++) av.push_back(top[k]);
            std::vector<size_t> bf; for(size_t k=np;k-->0;) bf.push_back(base+k); af.push_back(bf);
            std::vector<size_t> tf; for(size_t k=0;k<np;k++) tf.push_back(base+np+k); af.push_back(tf);
            for(size_t k=0;k<np;k++){size_t k1=(k+1)%np; af.push_back({base+k,base+k1,base+k1+np,base+k+np});}
        }
        auto g = session.add_group("Elements");
        session.add(session.add_mesh(std::make_shared<Mesh>(Mesh::from_vertices_and_faces(av, af))), g);
    }

    // Joint areas colored by type
    {
        const char* names[] = {"Joints Side-Side", "Joints Side-Top", "Joints Top-Top"};
        Color colors[] = {Color(255,50,50), Color(50,255,50), Color(50,50,255)};
        for (int t = 0; t < 3; t++) {
            std::vector<Point> jv; std::vector<std::vector<size_t>> jf;
            for (auto& j : joints) {
                if (j.type != t) continue;
                auto pts = j.joint_area.get_points();
                size_t np = pts.size();
                if(np>3 && std::abs(pts[np-1][0]-pts[0][0])<1e-6 &&
                   std::abs(pts[np-1][1]-pts[0][1])<1e-6 &&
                   std::abs(pts[np-1][2]-pts[0][2])<1e-6) np--;
                if (np < 3) continue;
                size_t base=jv.size();
                for(size_t k=0;k<np;k++) jv.push_back(pts[k]);
                for(size_t k=1;k+1<np;k++) jf.push_back({base,base+k,base+k+1});
            }
            if(jf.empty()) continue;
            auto g = session.add_group(names[t]);
            auto m = std::make_shared<Mesh>(Mesh::from_vertices_and_faces(jv, jf));
            m->set_objectcolor(colors[t]);
            session.add(session.add_mesh(m), g);
        }
    }
    session.pb_dump(pb_path);

    // 5. Write intermediate data
    nlohmann::ordered_json out_data;
    out_data["elements"] = data["elements"];
    out_data["adjacency"] = adjacency;
    nlohmann::json joints_json = nlohmann::json::array();
    for (auto& j : joints) {
        nlohmann::json jj;
        jj["id"]=j.id; jj["element_a"]=j.element_a; jj["element_b"]=j.element_b;
        jj["face_a"]=j.face_a; jj["face_b"]=j.face_b; jj["type"]=j.type;
        nlohmann::json area = nlohmann::json::array();
        for (auto& p : j.joint_area.get_points()) area.push_back({p[0], p[1], p[2]});
        jj["joint_area"] = area;
        joints_json.push_back(jj);
    }
    out_data["joints"] = joints_json;
    std::ofstream fout(json_out);
    fout << out_data.dump(2);
    fmt::print("Wrote {} and {}\n", pb_path, json_out);
    return 0;
}
