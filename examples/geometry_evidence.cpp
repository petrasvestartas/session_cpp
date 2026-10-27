#include "brep.h"
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace session_cpp;

int main() {
    const std::filesystem::path folder = std::filesystem::path("build") / "geometry-tests";
    const BRep exclusive = BRep::file_json_load((folder / "rhino_chairs_xor.brep.json").string());
    nlohmann::ordered_json components = nlohmann::ordered_json::array();
    double volume = 0.0;
    for (size_t i = 0; i < exclusive.m_solids.size(); ++i) {
        BRep component = exclusive;
        component.m_solids = {exclusive.m_solids[i]};
        const Mesh mesh = component.tessellate(0.005, 0.15);
        const std::string name = "rhino_xor_component_" + std::to_string(i);
        mesh.file_json_dump((folder / (name + ".component.json")).string());
        const double measured = component.volume_exact();
        components.push_back({{"name", name}, {"volume", measured}});
        volume += measured;
    }
    if (std::abs(volume - exclusive.volume_exact()) > volume * 1e-6)
        throw std::runtime_error("XOR component volumes do not sum to the whole");
    std::ofstream(folder / "rhino_xor_components.json") << components.dump(2);

    nlohmann::ordered_json offsets = nlohmann::ordered_json::array();
    int failures = 0;
    for (int i = 0; i < 2; ++i) {
        const BRep chair = BRep::file_json_load((folder / ("rhino_chair_" + std::to_string(i) + ".brep.json")).string());
        nlohmann::ordered_json result = {{"chair", i}, {"distance", 0.01}};
        try {
            const BRep offset = chair.offset(0.01);
            result["valid"] = offset.is_solid();
            result["volume"] = offset.volume_exact();
            if (!offset.is_solid())
                ++failures;
        } catch (const std::exception& error) {
            result["valid"] = false;
            result["error"] = error.what();
            ++failures;
        }
        offsets.push_back(result);
    }
    std::ofstream(folder / "cpp_outward_offset_probes.json") << offsets.dump(2);
    std::cout << components.size() << " XOR components exported; " << failures << " outward offset probes failed\n";
    return failures;
}
