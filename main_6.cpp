#include <filesystem>
#include "file_step.h"
#include "brep.h"
#include "fmt/core.h"
using namespace session_cpp;

int main() {

    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto step_dir = base / "session_data" / "elements";
    auto out_dir = base / "serialization";
    std::filesystem::create_directories(out_dir);

    for (const auto& entry : std::filesystem::directory_iterator(step_dir)) {
        auto ext = entry.path().extension();
        if (ext != ".step" && ext != ".stp") continue;
        auto step_path = entry.path().string();
        auto stem = entry.path().stem().string();

        auto breps = file_step::read_file_step_breps(step_path);
        fmt::print("{}: {} solid(s)\n", stem, breps.size());
        for (size_t i = 0; i < breps.size(); i++) {
            auto pb_path = (out_dir / (stem + "_" + std::to_string(i) + ".pb")).string();
            breps[i].pb_dump(pb_path);
            fmt::print("  [{}] faces={} edges={} verts={} -> {}\n",
                i, breps[i].face_count(), breps[i].edge_count(), breps[i].vertex_count(),
                pb_path);
        }
    }
    return 0;
}
