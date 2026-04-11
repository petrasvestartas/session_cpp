#include "obj.h"
#include "polyline.h"
#include <fstream>
#include <filesystem>
#include <fmt/core.h>
using namespace session_cpp;

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto polylines = obj::read_obj_polylines(
        (base / "session_data" / "annen_polylines.obj").string());
    auto pairs = obj::pair_polylines(polylines);

    fmt::print("{} polylines, {} pairs\n", polylines.size(), pairs.size());

    std::ofstream out((base / "session_data" / "annen_for_wood.xml").string());
    out << "<?xml version=\"1.0\" encoding=\"utf-8\"?><input_polylines>";
    for (auto [a, b] : pairs) {
        for (int idx : {a, b}) {
            out << "<polyline>";
            for (size_t k = 0; k < polylines[idx].point_count(); k++) {
                auto p = polylines[idx].get_point(k);
                out << fmt::format("<point><x>{}</x><y>{}</y><z>{}</z></point>",
                    p[0], p[1], p[2]);
            }
            out << "</polyline>";
        }
    }
    out << "</input_polylines>";
    out.close();
    fmt::print("Wrote {}\n", (base / "session_data" / "annen_for_wood.xml").string());
    return 0;
}
