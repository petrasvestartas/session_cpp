#include <filesystem>
#include <chrono>
#include "session.h"
#include "element.h"
#include "feature.h"
#include "obj.h"
#include "intersection.h"
using namespace session_cpp;

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now();

    // 1. Import cross polylines (top/bottom pairs of crossed plate elements)
    auto polylines = obj::read_obj_polylines((base / "session_data" / "cross_polylines.obj").string());
    auto pairs = obj::pair_polylines(polylines);
    auto t1 = Clock::now();

    // 2. Build session + plate elements + input polylines
    Session session("WoodCross");
    auto g_input = session.add_group("Input");
    auto g_elems = session.add_group("Elements");
    std::vector<std::shared_ptr<ElementPlate>> plates;
    for (auto [a, b] : pairs) {
        auto pa = std::make_shared<Polyline>(polylines[a]);
        pa->name = "input_" + std::to_string(a);
        session.add_polyline(pa, g_input);
        auto pb = std::make_shared<Polyline>(polylines[b]);
        pb->name = "input_" + std::to_string(b);
        session.add_polyline(pb, g_input);
        auto plate = std::make_shared<ElementPlate>(
            polylines[a], polylines[b], "plate_" + std::to_string(a));
        plates.push_back(plate);
        session.add_element(plate, g_elems);
    }
    auto t2 = Clock::now();

    // 3. All-pairs cross joint detection
    auto g_joints = session.add_group("Joints");
    int joint_count = 0;
    for (size_t i = 0; i < plates.size(); i++) {
        for (size_t j = i + 1; j < plates.size(); j++) {
            Intersection::CrossJoint cj;
            if (!Intersection::plane_to_face(plates[i].get(), plates[j].get(), cj)) continue;

            auto area = std::make_shared<Polyline>(std::move(cj.joint_area));
            area->name = "joint_area_" + std::to_string(joint_count);
            session.add_polyline(area, g_joints);

            std::vector<std::string> vol_guids;
            vol_guids.reserve(2);
            for (int k = 0; k < 2; k++) {
                auto vol = std::make_shared<Polyline>(std::move(cj.joint_volumes[k]));
                vol->name = "joint_vol_" + std::to_string(joint_count) + "_" + std::to_string(k);
                session.add_polyline(vol, g_joints);
                vol_guids.push_back(vol->guid());
            }

            std::vector<std::string> line_guids;
            line_guids.reserve(2);
            for (int k = 0; k < 2; k++) {
                auto ln = std::make_shared<Polyline>(std::move(cj.joint_lines[k]));
                ln->name = "joint_line_" + std::to_string(joint_count) + "_" + std::to_string(k);
                session.add_polyline(ln, g_joints);
                line_guids.push_back(ln->guid());
            }

            CrossFeature cf;
            cf.face_ids_a = cj.face_ids_a;
            cf.face_ids_b = cj.face_ids_b;
            cf.joint_area_guid = area->guid();
            cf.joint_volume_guids[0] = vol_guids[0];
            cf.joint_volume_guids[1] = vol_guids[1];
            cf.joint_line_guids[0] = line_guids[0];
            cf.joint_line_guids[1] = line_guids[1];

            session.add_feature(plates[i]->guid(), plates[j]->guid(), std::move(cf));
            joint_count++;
        }
    }
    auto t3 = Clock::now();

    // 4. Save
    session.pb_dump((base / "session_data" / "WoodCross.pb").string());
    auto t4 = Clock::now();

    auto ms = [](auto a, auto b) { return std::chrono::duration<double,std::milli>(b-a).count(); };
    fmt::print("{} polylines -> {} elements -> {} cross joints\n",
        polylines.size(), pairs.size(), joint_count);
    fmt::print("  import+pair: {:.0f}ms  elements: {:.0f}ms  joints: {:.0f}ms  save: {:.0f}ms  total: {:.0f}ms\n",
        ms(t0,t1), ms(t1,t2), ms(t2,t3), ms(t3,t4), ms(t0,t4));
    return 0;
}
