#include <filesystem>
#include <fstream>
#include <chrono>
#include "session.h"
#include "element.h"
#include "intersection.h"
#include "json.h"
using namespace session_cpp;

static std::vector<Point> json_to_points(const nlohmann::json& arr) {
    std::vector<Point> pts;
    pts.reserve(arr.size());
    for (auto& p : arr)
        pts.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    return pts;
}

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto json_in = (base / "session_data" / "WoodStep3_data.json").string();
    auto pb_path = (base / "session_data" / "WoodStep4.pb").string();

    // 1. Load adjacency data from Step 3
    std::ifstream fin(json_in);
    nlohmann::json data = nlohmann::json::parse(fin);
    std::vector<int> adjacency = data["adjacency"].get<std::vector<int>>();
    size_t N = data["elements"].size();

    auto t0 = std::chrono::high_resolution_clock::now();

    // 2. Create ElementPlates and add to session
    Session session("WoodStep4_Joints");
    auto g_elem = session.add_group("Elements");
    std::vector<Element*> elem_ptrs(N);
    for (size_t i = 0; i < N; i++) {
        auto& e = data["elements"][i];
        auto bottom = json_to_points(e["polygon"]);
        auto top = e.contains("polygon_top") ? json_to_points(e["polygon_top"]) : bottom;
        auto elem = std::make_shared<ElementPlate>(bottom, top, "plate_" + std::to_string(i));
        session.add_element(elem, g_elem);
        elem_ptrs[i] = elem.get();
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    // Face-to-face: coplanar check → 3D boolean intersection → graph edge
    double coplanar_tolerance = 5.0; // mm — user-controllable
    auto g_joints = session.add_group("Joints");
    int t0c = 0, t1c = 0, t2c = 0;

    auto joints = Intersection::face_to_face(adjacency, elem_ptrs, coplanar_tolerance);

    for (size_t k = 0; k < joints.size(); k++) {
        auto& [a, b, i, j, type, poly] = joints[k];
        if (type == 0) t0c++; else if (type == 1) t1c++; else t2c++;

        auto jpl = std::make_shared<Polyline>(std::move(poly));
        jpl->name = "joint_" + std::to_string(k);
        session.add_polyline(jpl, g_joints);

        session.add_edge(elem_ptrs[a]->guid(), elem_ptrs[b]->guid(),
            std::to_string(i) + "," + std::to_string(j) + "," +
            std::to_string(type) + "," + jpl->guid());
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    fmt::print("Precompute: {:.1f} ms\n", std::chrono::duration<double, std::milli>(t1 - t0).count());
    fmt::print("Detect: {} joints in {:.1f} ms\n", joints.size(), std::chrono::duration<double, std::milli>(t2 - t1).count());
    fmt::print("Total: {:.1f} ms\n", std::chrono::duration<double, std::milli>(t2 - t0).count());
    fmt::print("  side-side: {}, side-top: {}, top-top: {}\n", t0c, t1c, t2c);
    fmt::print("  Graph: {} nodes, {} edges\n", session.graph.number_of_vertices(), session.graph.number_of_edges());

    session.pb_dump(pb_path);
    fmt::print("Wrote {}\n", pb_path);
    return 0;
}
