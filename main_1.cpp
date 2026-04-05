#include <filesystem>
#include <fstream>
#include "session.h"
#include "obj.h"
#include "polyline.h"
#include "json.h"
using namespace session_cpp;

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto obj_path = (base / "session_data" / "annen_polylines.obj").string();
    auto pb_path = (base / "session_data" / "WoodStep1.pb").string();
    auto json_path = (base / "session_data" / "WoodStep1_data.json").string();

    // 1. Import polylines from OBJ
    auto polylines = obj::read_obj_polylines(obj_path);
    fmt::print("Imported {} polylines\n", polylines.size());

    // 2. Visualize in Session
    Session session("WoodStep1_Polylines");
    auto g = session.add_group("Polylines");
    for (size_t i = 0; i < polylines.size(); i++) {
        auto pl = std::make_shared<Polyline>(polylines[i]);
        session.add(session.add_polyline(pl), g);
    }
    session.pb_dump(pb_path);

    // 3. Write intermediate data JSON
    nlohmann::ordered_json data;
    nlohmann::json pl_array = nlohmann::json::array();
    for (size_t i = 0; i < polylines.size(); i++) {
        nlohmann::json entry;
        auto pts = polylines[i].get_points();
        nlohmann::json coords = nlohmann::json::array();
        for (auto& p : pts) coords.push_back({p[0], p[1], p[2]});
        entry["coords"] = coords;
        entry["name"] = "pl_" + std::to_string(i);
        pl_array.push_back(entry);
    }
    data["polylines"] = pl_array;
    std::ofstream out(json_path);
    out << data.dump(2);

    fmt::print("Wrote {} and {}\n", pb_path, json_path);
    return 0;
}
