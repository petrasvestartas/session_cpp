#include <filesystem>
#include <fstream>
#include "session.h"
#include "element.h"
#include "polyline.h"
#include "json.h"
using namespace session_cpp;

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto json_in = (base / "session_data" / "WoodStep4_data.json").string();
    auto pb_path = (base / "session_data" / "WoodStep5.pb").string();

    std::ifstream fin(json_in);
    if (!fin.is_open()) { fmt::print("ERROR: cannot open {}\n", json_in); return 1; }
    nlohmann::json data = nlohmann::json::parse(fin);
    fmt::print("Loaded {} elements, {} joints\n", data["elements"].size(), data["joints"].size());

    Session session("WoodStep5_Complete");

    // Merged plate mesh
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
            for (auto& j : data["joints"]) {
                if (j["type"].get<int>() != t) continue;
                std::vector<Point> pts;
                for (auto& p : j["joint_area"]) pts.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
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
    fmt::print("Wrote {}\n", pb_path);
    return 0;
}
