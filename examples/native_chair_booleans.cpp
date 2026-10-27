#include "brep_boolean.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "curve_surface_intersection.h"
#include "file_step.h"
#include "session.h"
#include "split.h"
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#ifdef SESSION_WITH_OCCT
#error Native chair acceptance must be built without OCCT
#endif

using namespace session_cpp;

namespace {

const double Precision = 1e-6;
const std::array<std::string, 5> Names = {"intersection", "union", "difference_ab", "difference_ba", "xor"};
const std::array<double, 5> Volumes = {310.498362778068, 5578.9103277598,
    2634.2059829083214, 2634.2059838660166, 5268.411966140004};
const std::array<int, 5> Solids = {2, 1, 2, 1, 3};
const std::array<brep_boolean::Operation, 5> Operations = {brep_boolean::Operation::Intersection,
    brep_boolean::Operation::Union, brep_boolean::Operation::Difference,
    brep_boolean::Operation::Difference, brep_boolean::Operation::Xor};

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

void save(const std::filesystem::path& folder, const std::string& name, const BRep& shape) {
    shape.file_json_dump((folder / (name + ".brep.json")).string());
    BRep visible = shape;
    visible.surfacecolor = Color(180 / 255.f, 110 / 255.f, 155 / 255.f);
    Session scene(name);
    scene.add_brep(visible);
    scene.pb_dump((folder / (name + ".pb")).string());
}

void check_pcurves(const BRep& shape) {
    for (const BRepEdge& edge : shape.m_edges) {
        if (edge.degenerated)
            continue;
        const NurbsCurve& world = shape.m_curves_3d[edge.curve_3d_index];
        for (const BRepCurveOnSurface& record : edge.pcurves)
            for (const int index : {record.curve_2d_index, record.curve_2d_index_2}) {
                if (index < 0)
                    continue;
                const auto status = curve_surface_intersection::same_parameter(world,
                    shape.m_curves_2d[index], shape.m_surfaces[record.surface_index], Precision);
                require(status == curve_surface_intersection::ParameterMatch::Matched,
                    "Chair result has an uncertified edge/pcurve correspondence");
            }
    }
}

bool truth(bool a, bool b, size_t operation) {
    switch (operation) {
        case 0: return a && b;
        case 1: return a || b;
        case 2: return a && !b;
        case 3: return b && !a;
        default: return a != b;
    }
}

size_t check_probes(const BRep& result, const std::vector<BRep>& chairs,
                    const nlohmann::json& probes, size_t operation) {
    size_t checked = 0;
    for (const auto& probe : probes) {
        const Point point(probe["point"][0], probe["point"][1], probe["point"][2]);
        const Vector normal(probe["normal"][0], probe["normal"][1], probe["normal"][2]);
        for (const double distance : {-0.001, 0.001}) {
            const Point p = point + normal * distance;
            const auto a = brep_classify::point(chairs[0], p, Precision);
            const auto b = brep_classify::point(chairs[1], p, Precision);
            const auto c = brep_classify::point(result, p, Precision);
            require(a != brep_classify::State::Unknown && b != brep_classify::State::Unknown
                && c != brep_classify::State::Unknown, "Chair Boolean probe classification is unresolved");
            if (a == brep_classify::State::Boundary || b == brep_classify::State::Boundary)
                continue;
            const bool expected = truth(a == brep_classify::State::Inside, b == brep_classify::State::Inside, operation);
            require(c == (expected ? brep_classify::State::Inside : brep_classify::State::Outside),
                "Chair result violates the Boolean occupancy truth table");
            ++checked;
        }
    }
    require(checked > 0, "Chair Boolean acceptance has no interior/exterior probes");
    return checked;
}

void check_result(const BRep& result, const std::vector<BRep>& chairs,
                  const nlohmann::json& probes, size_t operation, nlohmann::ordered_json& record) {
    require(result.is_valid() && result.is_solid(), "Chair result is not a valid closed oriented solid");
    require(result.solid_count() == Solids[operation], "Chair result has the wrong connected solid count");
    record["faces"] = result.face_count();
    record["solids"] = result.solid_count();
    const double volume = result.volume_exact(1e-8);
    record["volume"] = volume;
    require(std::abs(volume - Volumes[operation]) <= Volumes[operation] * 1e-7,
        "Chair result volume disagrees with the independent OCCT STEP oracle");
    check_pcurves(result);
    record["pcurves_certified"] = true;
    record["probes_checked"] = check_probes(result, chairs, probes, operation);
    const BRep restored = BRep::pb_loads(result.pb_dumps());
    require(restored.is_solid() && restored.face_count() == result.face_count()
        && restored.edge_count() == result.edge_count(), "Chair protobuf round trip changed topology");
    require(std::abs(restored.volume_exact(1e-8) - volume) <= volume * 1e-10,
        "Chair protobuf round trip changed volume");
}

}

int main(int argc, char** argv) {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const auto folder = root / "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    const auto chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    require(chairs.size() == 2, "Chair fixture must contain two solids");
    std::ifstream input(root / "build/native-geometry-tests/chair_classification.json");
    require(input.good(), "Native chair classification fixture is missing");
    nlohmann::json probes;
    input >> probes;
    int failures = 0;
    for (size_t i = 0; i < Names.size(); ++i) {
        if (argc > 1 && Names[i] != argv[1])
            continue;
        const auto start = std::chrono::steady_clock::now();
        const std::string name = "native_chairs_" + Names[i];
        const auto path = folder / (name + ".acceptance.json");
        nlohmann::ordered_json record = {{"backend", "session_cpp_native"}, {"operation", Names[i]},
            {"tolerance", Precision}, {"expected_volume", Volumes[i]}, {"expected_solids", Solids[i]},
            {"status", "running"}, {"passed", false}};
        std::ofstream(path) << record.dump(2);
        std::cout << "RUN " << name << std::endl;
        try {
            const BRep result = brep_boolean::apply(chairs[i == 3 ? 1 : 0], chairs[i == 3 ? 0 : 1], Operations[i], Precision);
            save(folder, name, result);
            check_result(result, chairs, probes, i, record);
            record["passed"] = true;
            record["status"] = "passed";
        } catch (const brep_assemble::ParameterError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            const nlohmann::ordered_json diagnostic = {{"world", error.curve.jsondump()},
                {"pcurve", error.pcurve.jsondump()}, {"surface", error.surface.jsondump()},
                {"tolerance", error.tolerance}, {"error", error.what()}};
            std::ofstream(folder / (name + ".parameter_failure.json")) << diagnostic.dump(2);
        } catch (const split::CurveError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            const nlohmann::ordered_json diagnostic = {{"world", error.curve.jsondump()},
                {"pcurve", error.pcurve.jsondump()}, {"surface", error.surface.jsondump()},
                {"parameter", error.parameter}, {"distance", error.distance},
                {"tolerance", error.tolerance}, {"error", error.what()}};
            std::ofstream(folder / (name + ".split_parameter_failure.json")) << diagnostic.dump(2);
        } catch (const brep_boolean::ClassificationError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            const nlohmann::ordered_json diagnostic = {{"shape", error.shape.jsondump()},
                {"point", {error.query[0], error.query[1], error.query[2]}},
                {"source_face", error.source_face}, {"second_operand", error.second_operand},
                {"tolerance", error.tolerance}, {"error", error.what()}};
            std::ofstream(folder / (name + ".classification_failure.json")) << diagnostic.dump(2);
        } catch (const std::exception& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
        }
        record["seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::ofstream(path) << record.dump(2);
        std::cout << record.dump() << std::endl;
    }
    return failures ? 1 : 0;
}
