#include <filesystem>
#include <chrono>
#include "session.h"
#include "element.h"
#include "obj.h"
using namespace session_cpp;

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now();

    // 1. Import + pair polylines
    auto polylines = obj::read_obj_polylines((base / "session_data" / "annen_polylines.obj").string());
    auto pairs = obj::pair_polylines(polylines);
    auto t1 = Clock::now();

    // 2. Create session with elements
    Session session("WoodComplete");
    auto g = session.add_group("Elements");
    for (auto [a, b] : pairs)
        session.add_element(std::make_shared<PlateElement>(
            polylines[a], polylines[b], "plate_" + std::to_string(a)), g);
    auto t2 = Clock::now();

    // 3. Compute face-to-face contacts (adjacency + boolean intersection → graph edges)
    session.compute_face_to_face(5.0, 50.0);
    auto t3 = Clock::now();

    // 4. Save
    session.pb_dump((base / "session_data" / "WoodComplete.pb").string());
    auto t4 = Clock::now();

    auto ms = [](auto a, auto b) { return std::chrono::duration<double,std::milli>(b-a).count(); };
    fmt::print("{} polylines -> {} elements -> {} joints\n",
        polylines.size(), pairs.size(), session.graph.number_of_edges());
    fmt::print("  import+pair: {:.0f}ms  elements: {:.0f}ms  contacts: {:.0f}ms  save: {:.0f}ms  total: {:.0f}ms\n",
        ms(t0,t1), ms(t1,t2), ms(t2,t3), ms(t3,t4), ms(t0,t4));
    return 0;
}
