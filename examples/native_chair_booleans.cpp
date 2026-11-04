#include "brep_boolean.h"
#include "brep_acceptance.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "curve_surface_intersection.h"
#include "file_step.h"
#include "session.h"
#include "split.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
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
using PublicBoolean = BRep (BRep::*)(const BRep&, double) const;
const std::array<PublicBoolean, 5> Operations = {&BRep::boolean_intersection, &BRep::boolean_union,
    &BRep::boolean_difference, &BRep::boolean_difference, &BRep::boolean_xor};

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

void write_bytes(const std::filesystem::path& path, const std::string& bytes, bool exclusive = false) {
#ifdef _WIN32
    std::FILE* file = _wfopen(path.c_str(), exclusive ? L"wbx" : L"wb");
#else
    std::FILE* file = std::fopen(path.c_str(), exclusive ? "wbx" : "wb");
#endif
    require(file != nullptr, "Cannot open evidence output: " + path.string());
    const bool written = std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
    const bool closed = std::fclose(file) == 0;
    require(written && closed, "Cannot write or close evidence output: " + path.string());
}

void save_diagnostic(const std::filesystem::path& path, const nlohmann::ordered_json& diagnostic,
                     nlohmann::ordered_json& record) {
    try {
        write_bytes(path, diagnostic.dump(2), true);
    } catch (const std::exception& error) {
        record["diagnostic_write_error"] = error.what();
        std::cerr << error.what() << std::endl;
    }
}

void preflight_outputs(const std::filesystem::path& folder, int argc, char** argv) {
    for (const std::string& operation : Names) {
        if (argc == 2 && operation != argv[1])
            continue;
        for (const std::string& suffix : {".acceptance.json", ".brep.json", ".pb", ".parameter_failure.json",
                ".split_parameter_failure.json", ".classification_failure.json", ".topology_failure.json"})
            require(!std::filesystem::exists(folder / ("native_chairs_" + operation + suffix)),
                "Fresh chair output already exists; choose a new SESSION_NATIVE_CHAIR_OUTPUT directory");
    }
}

void save(const std::filesystem::path& folder, const std::string& name, const BRep& shape) {
    write_bytes(folder / (name + ".brep.json"), shape.jsondump().dump(4), true);
    BRep visible = shape;
    visible.surfacecolor = Color(180 / 255.f, 110 / 255.f, 155 / 255.f);
    Session scene(name);
    scene.add_brep(visible);
    write_bytes(folder / (name + ".pb"), scene.pb_dumps(), true);
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
            require(a != brep_classify::State::Boundary && b != brep_classify::State::Boundary,
                "Original chair probe is not resolved away from operand boundaries");
            const bool expected = truth(a == brep_classify::State::Inside, b == brep_classify::State::Inside, operation);
            require(c == (expected ? brep_classify::State::Inside : brep_classify::State::Outside),
                "Chair result violates the Boolean occupancy truth table");
            ++checked;
        }
    }
    require(checked == 80, "Chair Boolean acceptance requires all 80 resolved original probes");
    return checked;
}

bool connected_shell(const BRep& brep, const BRepShell& shell) {
    std::map<int, std::vector<int>> incidence;
    std::map<int, int> balance;
    std::set<int> faces;
    for (const BRepRef& face : shell.faces) {
        if (!faces.insert(face.index).second)
            return false;
        for (const BRepRef& wire : brep.m_faces[face.index].wires)
            for (const BRepRef& edge : brep.wire_edges(wire)) {
                if (brep.m_edges[edge.index].degenerated)
                    continue;
                const BRepOrientation orientation = brep_compose(face.orientation, edge.orientation);
                if (orientation != BRepOrientation::Forward && orientation != BRepOrientation::Reversed)
                    return false;
                incidence[edge.index].push_back(face.index);
                balance[edge.index] += orientation == BRepOrientation::Forward ? 1 : -1;
            }
    }
    std::map<int, std::vector<int>> neighbours;
    for (const std::pair<const int, std::vector<int>>& edge : incidence) {
        if (edge.second.size() != 2 || balance[edge.first] != 0)
            return false;
        neighbours[edge.second[0]].push_back(edge.second[1]);
        neighbours[edge.second[1]].push_back(edge.second[0]);
    }
    if (faces.empty())
        return false;
    std::vector<int> queue{*faces.begin()};
    faces.erase(queue.front());
    for (size_t i = 0; i < queue.size(); ++i)
        for (int neighbour : neighbours[queue[i]])
            if (faces.erase(neighbour))
                queue.push_back(neighbour);
    return faces.empty();
}

bool manifold_components(const BRep& brep) {
    if (brep.face_count() == 0)
        return brep.edge_count() == 0 && brep.shell_count() == 0 && brep.solid_count() == 0;
    if (!brep.is_valid() || !brep.is_solid())
        return false;
    std::set<int> faces;
    std::set<int> shells;
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& use : solid.shells) {
            if (!shells.insert(use.index).second)
                return false;
            const BRepShell& shell = brep.m_shells[use.index];
            if (!connected_shell(brep, shell))
                return false;
            for (const BRepRef& face : shell.faces)
                if (!faces.insert(face.index).second)
                    return false;
        }
    return faces.size() == brep.m_faces.size() && shells.size() == brep.m_shells.size();
}

bool referenced_output(const BRep& brep) {
    std::set<int> edges;
    std::set<int> vertices;
    for (const BRepFace& face : brep.m_faces)
        for (const BRepRef& wire : face.wires)
            for (const BRepRef& use : brep.wire_edges(wire)) {
                edges.insert(use.index);
                const BRepEdge& edge = brep.m_edges[use.index];
                vertices.insert(edge.start_vertex);
                vertices.insert(edge.end_vertex);
            }
    for (int i = 0; i < brep.edge_count(); ++i)
        if (!brep.m_edges[i].degenerated && !edges.count(i))
            return false;
    return vertices.size() == brep.m_vertices.size();
}

void check_restored_geometry(const BRep& original, const BRep& restored) {
    const auto curves = [](const std::vector<NurbsCurve>& before, const std::vector<NurbsCurve>& after) {
        require(before.size() == after.size(), "Chair protobuf changed a curve pool size");
        for (size_t i = 0; i < before.size(); ++i)
            require(before[i].m_dim == after[i].m_dim && before[i].m_is_rat == after[i].m_is_rat
                && before[i].m_order == after[i].m_order && before[i].m_cv_count == after[i].m_cv_count
                && before[i].m_cv == after[i].m_cv && before[i].m_nurbsknot == after[i].m_nurbsknot
                && before[i].domain() == after[i].domain(), "Chair protobuf changed curve geometry or its raw domain");
    };
    curves(original.m_curves_3d, restored.m_curves_3d);
    curves(original.m_curves_2d, restored.m_curves_2d);
    require(original.m_surfaces.size() == restored.m_surfaces.size(), "Chair protobuf changed the surface pool size");
    for (size_t i = 0; i < original.m_surfaces.size(); ++i) {
        const auto& before = original.m_surfaces[i];
        const auto& after = restored.m_surfaces[i];
        require(before.m_dim == after.m_dim && before.m_is_rat == after.m_is_rat && before.m_cv == after.m_cv,
            "Chair protobuf changed homogeneous surface geometry");
        for (int d = 0; d < 2; ++d)
            require(before.m_order[d] == after.m_order[d] && before.m_cv_count[d] == after.m_cv_count[d]
                && before.m_nurbsknot[d] == after.m_nurbsknot[d] && before.domain(d) == after.domain(d),
                "Chair protobuf changed a surface basis or its raw domain");
    }
    const auto before = original.jsondump(), after = restored.jsondump();
    for (const char* key : {"vertices", "edges", "wires", "faces", "shells", "solids"})
        require(before.at(key) == after.at(key), "Chair protobuf changed topology, ownership or stored tolerances");
}

void check_result(const BRep& result, const std::vector<BRep>& chairs,
                  const nlohmann::json& probes, size_t operation, nlohmann::ordered_json& record) {
    require(result.is_valid() && result.is_solid(), "Chair result is not a valid closed oriented solid");
    require(manifold_components(result), "Chair result has invalid connected manifold component ownership");
    record["closed_connected_shell_ownership"] = true;
    for (const BRepShell& shell : result.m_shells)
        require(brep_acceptance::shell_vertex_links(result, shell) == brep_acceptance::VertexLinks::Matched,
            "Chair vertex-link validation failed or has unsupported degenerate sectors");
    record["per_shell_vertex_links"] = "matched";
    require(referenced_output(result), "Chair result contains unreferenced edges or vertices");
    record["referenced_output_edges_vertices"] = true;
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
    require(restored.vertex_count() == result.vertex_count() && restored.wire_count() == result.wire_count()
        && restored.shell_count() == result.shell_count() && restored.solid_count() == result.solid_count(),
        "Chair protobuf round trip changed component counts");
    require(restored.is_valid() && manifold_components(restored) && referenced_output(restored),
        "Chair protobuf round trip changed manifold ownership or references");
    for (const BRepShell& shell : restored.m_shells)
        require(brep_acceptance::shell_vertex_links(restored, shell) == brep_acceptance::VertexLinks::Matched,
            "Restored chair vertex-link validation failed");
    check_restored_geometry(result, restored);
    check_pcurves(restored);
    record["protobuf_restored_pcurves_certified"] = true;
    record["protobuf_restored_geometry_domains_preserved"] = true;
    record["protobuf_restored_probes_checked"] = check_probes(restored, chairs, probes, operation);
    record["protobuf_restored_topology_checked"] = true;
}

void check_saved_probes(const BRep& result, const nlohmann::json& oracle,
                        const nlohmann::json* expected, nlohmann::ordered_json& report) {
    size_t mismatches = 0;
    for (size_t i = 0; i < 80; ++i) {
        const nlohmann::json& probe = oracle.at("probes").at(i);
        const Point point(probe.at("point").at(0).get<double>(), probe.at("point").at(1).get<double>(),
            probe.at("point").at(2).get<double>());
        const brep_classify::State state = brep_classify::point(result, point, Precision);
        const std::string actual = state == brep_classify::State::Inside ? "inside" :
            state == brep_classify::State::Outside ? "outside" : state == brep_classify::State::Boundary ? "boundary" : "unknown";
        const std::string label = expected->at("probe_labels").at(i).get<std::string>();
        const bool mismatch = label == "unknown" || actual == "unknown" || actual != label;
        mismatches += mismatch ? 1 : 0;
        report["native_states"].push_back({{"index", i}, {"point", probe.at("point")},
            {"chair", probe.at("chair")}, {"source_face", probe.at("source_face")},
            {"normal_sign", probe.at("normal_sign")}, {"actual", actual}, {"oracle_expected", label}, {"mismatch", mismatch}});
    }
    report["checked"] = 80;
    report["mismatches"] = mismatches;
    require(mismatches == 0, "Saved result differs from independent OCCT probe labels");
}

void check_saved_measurements(const BRep& result, size_t index,
                              const nlohmann::json* expected, nlohmann::ordered_json& report) {
    require(result.solid_count() == Solids[index] && result.solid_count() == expected->at("solids").get<int>(),
        "Saved chair connected solid count disagrees with acceptance or independent oracle");
    require(result.shell_count() == expected->at("shells").get<int>(), "Saved chair shell count disagrees with oracle");
    report["volume"] = result.volume_exact(1e-8);
    const double volume = report.at("volume").get<double>();
    const double oracle_volume = expected->at("absolute_volume").get<double>();
    require(std::abs(volume - Volumes[index]) <= Volumes[index] * 1e-7 &&
        std::abs(volume - oracle_volume) <= oracle_volume * 1e-7,
        "Saved chair volume disagrees with original acceptance or independent oracle");
    std::vector<std::pair<double, int>> native_components, oracle_components;
    for (const BRepSolid& solid : result.m_solids) {
        BRep component = result;
        component.m_solids = {solid};
        native_components.push_back({component.volume_exact(1e-8), static_cast<int>(solid.shells.size())});
    }
    for (const nlohmann::json& component : expected->at("components")) {
        require(component.at("valid").get<bool>(), "Independent component is invalid");
        oracle_components.push_back({component.at("absolute_volume").get<double>(), component.at("shells").get<int>()});
    }
    std::sort(native_components.begin(), native_components.end());
    std::sort(oracle_components.begin(), oracle_components.end());
    require(native_components.size() == oracle_components.size(), "Component oracle size mismatch");
    for (size_t c = 0; c < native_components.size(); ++c)
        require(native_components[c].second == oracle_components[c].second &&
            std::abs(native_components[c].first - oracle_components[c].first) <= oracle_components[c].first * 1e-7,
            "Saved component volume or shell count disagrees with independent oracle");
    report["native_components"] = native_components;
    report["oracle_components"] = oracle_components;
    check_pcurves(result);
    report["pcurves_certified"] = true;
    const BRep restored = BRep::pb_loads(result.pb_dumps());
    require(restored.is_solid() && restored.face_count() == result.face_count() && restored.edge_count() == result.edge_count(),
        "Saved result protobuf round trip changed topology");
    require(std::abs(restored.volume_exact(1e-8) - volume) <= volume * 1e-10,
        "Saved result protobuf round trip changed volume");
}

int validate_saved(int argc, char** argv) {
    require(argc == 6, "Usage: booleans validate-saved operation result.brep.json oracle.json report.json");
    const std::string operation = argv[2];
    size_t index = 0;
    while (index < Names.size() && Names[index] != operation)
        ++index;
    require(index < Names.size(), "Unknown saved Boolean operation");
    std::ifstream result_input(argv[3]), oracle_input(argv[4]);
    require(result_input.good() && oracle_input.good(), "Saved result or independent oracle is missing");
    const BRep result = BRep::jsonload(nlohmann::json::parse(result_input));
    const nlohmann::json oracle = nlohmann::json::parse(oracle_input);
    require(oracle.at("schema") == "independent_chair_oracle_v1" && oracle.at("complete").get<bool>(),
        "Independent oracle is incomplete or has an unsupported schema");
    require(oracle.at("step_sha256") == "6cb6b9d1723a0570fd72e1eb8ecedf109a750a468f5f8e771a624ed4224aac19",
        "Oracle is for a different STEP fixture; actual file hashes must also be verified by the external harness");
    require(oracle.at("probes_sha256").is_string() && oracle.at("probes_sha256").get<std::string>().size() == 64,
        "Oracle has no probe digest");
    const nlohmann::json* expected = nullptr;
    for (const nlohmann::json& item : oracle.at("operations"))
        if (item.at("operation") == operation) {
            require(expected == nullptr, "Oracle operation is duplicated");
            expected = &item;
        }
    require(expected && expected->at("valid").get<bool>(), "Oracle operation is missing or invalid");
    require(oracle.at("probes").size() == 80 && expected->at("probe_labels").size() == 80,
        "Saved validation requires all 80 independent oracle probe labels");
    nlohmann::ordered_json report = {{"operation", operation}, {"mode", "saved_result_independent_validation"},
        {"passed", false}, {"complete", false}, {"native_states", nlohmann::ordered_json::array()},
        {"step_sha256", oracle.at("step_sha256")}, {"probes_sha256", oracle.at("probes_sha256")},
        {"actual_file_digest_binding", "required external before/after harness receipt"}};
    write_bytes(argv[5], report.dump(2), true);
    const auto save_report = [&]() { write_bytes(argv[5], report.dump(2)); };
    try {
        require(result.is_valid() && result.is_solid() && manifold_components(result),
            "Saved chair result fails solid validity or closed connected shell ownership");
        require(referenced_output(result), "Saved chair result contains unreferenced edges or vertices");
        report["closed_connected_shell_ownership"] = true;
        for (const BRepShell& shell : result.m_shells)
            require(brep_acceptance::shell_vertex_links(result, shell) == brep_acceptance::VertexLinks::Matched,
                "Chair vertex-link validation failed or has unsupported degenerate sectors");
        report["per_shell_vertex_links"] = "matched";
        report["referenced_output_edges_vertices"] = true;
        report["faces"] = result.face_count();
        report["solids"] = result.solid_count();
        report["shells"] = result.shell_count();
        check_saved_probes(result, oracle, expected, report);
        check_saved_measurements(result, index, expected, report);
        report["passed"] = true;
        report["complete"] = true;
        save_report();
        return 0;
    } catch (const std::exception& error) {
        report["failure"] = error.what();
        report["complete"] = true;
        save_report();
        throw;
    }
}

}

namespace native_chair_booleans {

int run(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "validate-saved")
        return validate_saved(argc, argv);
    require(argc == 1 || argc == 2, "Usage: booleans [intersection|union|difference_ab|difference_ba|xor]");
    if (argc == 2)
        require(std::find(Names.begin(), Names.end(), std::string(argv[1])) != Names.end(), "Unknown Boolean operation");
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const char* output = std::getenv("SESSION_NATIVE_CHAIR_OUTPUT");
    const std::filesystem::path folder = output && *output ? std::filesystem::path(output)
        : root / "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    preflight_outputs(folder, argc, argv);
    const auto chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    require(chairs.size() == 2, "Chair fixture must contain two solids");
    // Materialize lazy identifiers before the exact nonmutation baseline.
    const std::array<std::string, 2> chair_bytes = {chairs[0].jsondump().dump(), chairs[1].jsondump().dump()};
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
        nlohmann::ordered_json record = {{"backend", "session_cpp_native"}, {"entrypoint", "BRep public Boolean API"}, {"operation", Names[i]},
            {"tolerance", Precision}, {"expected_volume", Volumes[i]}, {"expected_solids", Solids[i]},
            {"acceptance_contract", "public_chair_bound_inputs_pb_v2"},
            {"status", "running"}, {"passed", false}};
        write_bytes(path, record.dump(2), true);
        std::cout << "RUN " << name << std::endl;
        try {
            const BRep result = (chairs[i == 3 ? 1 : 0].*Operations[i])(chairs[i == 3 ? 0 : 1], Precision);
            const std::chrono::steady_clock::time_point built = std::chrono::steady_clock::now();
            record["boolean_seconds"] = std::chrono::duration<double>(built - start).count();
            record["status"] = "exporting";
            write_bytes(path, record.dump(2));
            save(folder, name, result);
            const std::chrono::steady_clock::time_point exported = std::chrono::steady_clock::now();
            record["export_seconds"] = std::chrono::duration<double>(exported - built).count();
            record["status"] = "validating";
            write_bytes(path, record.dump(2));
            check_result(result, chairs, probes, i, record);
            record["validation_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - exported).count();
            record["passed"] = true;
            record["status"] = "passed";
        } catch (const brep_assemble::TopologyError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            nlohmann::ordered_json diagnostic = {{"operation", Names[i]}, {"operation_index", i},
                {"tolerance", Precision}, {"error", error.what()}, {"backend", "session_cpp_native"},
                {"call_operand_chair_indices", {i == 3 ? 1 : 0, i == 3 ? 0 : 1}},
                {"original_chairs", {chairs[0].jsondump(), chairs[1].jsondump()}},
                {"partial", error.partial.jsondump()}, {"face_orientations", nlohmann::ordered_json::array()},
                {"images", nlohmann::ordered_json::array()},
                {"input_index_contract", "image.input is the first selected FaceUse for its BRep owner, not an operand index"},
                {"output_index_contract", "image.output_edge indexes partial.edges; original_edge indexes its input owner"},
                {"orientation_encoding", {{"forward", static_cast<int>(BRepOrientation::Forward)},
                    {"reversed", static_cast<int>(BRepOrientation::Reversed)},
                    {"internal", static_cast<int>(BRepOrientation::Internal)},
                    {"external", static_cast<int>(BRepOrientation::External)}}}};
            for (const BRepOrientation orientation : error.face_orientations)
                diagnostic["face_orientations"].push_back(static_cast<int>(orientation));
            for (const auto& image : error.images)
                diagnostic["images"].push_back({{"input", image.input}, {"local_edge", image.local_edge},
                    {"original_edge", image.original_edge}, {"section", image.section},
                    {"output_edge", image.output_edge}, {"first", image.first}, {"last", image.last},
                    {"reversed", image.reversed}});
            const auto diagnostic_path = folder / (name + ".topology_failure.json");
            save_diagnostic(diagnostic_path, diagnostic, record);
            record["topology_diagnostic"] = diagnostic_path.string();
        } catch (const brep_assemble::ParameterError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            const nlohmann::ordered_json diagnostic = {{"world", error.curve.jsondump()},
                {"pcurve", error.pcurve.jsondump()}, {"surface", error.surface.jsondump()},
                {"tolerance", error.tolerance}, {"error", error.what()}};
            save_diagnostic(folder / (name + ".parameter_failure.json"), diagnostic, record);
        } catch (const split::CurveError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            const nlohmann::ordered_json diagnostic = {{"world", error.curve.jsondump()},
                {"pcurve", error.pcurve.jsondump()}, {"surface", error.surface.jsondump()},
                {"parameter", error.parameter}, {"distance", error.distance},
                {"tolerance", error.tolerance}, {"error", error.what()}};
            save_diagnostic(folder / (name + ".split_parameter_failure.json"), diagnostic, record);
        } catch (const brep_boolean::ClassificationError& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
            const nlohmann::ordered_json diagnostic = {{"shape", error.shape.jsondump()},
                {"point", {error.query[0], error.query[1], error.query[2]}},
                {"source_face", error.source_face}, {"second_operand", error.second_operand},
                {"tolerance", error.tolerance}, {"error", error.what()}};
            save_diagnostic(folder / (name + ".classification_failure.json"), diagnostic, record);
        } catch (const std::exception& error) {
            ++failures;
            record["status"] = "failed";
            record["error"] = error.what();
        }
        const bool inputs_unchanged = chairs[0].jsondump().dump() == chair_bytes[0]
            && chairs[1].jsondump().dump() == chair_bytes[1];
        record["inputs_unchanged"] = inputs_unchanged;
        record["input_nonmutation_contract"] = "Exact pre-run versus after-operation JSON bytes; lazy IDs materialized first";
        if (!inputs_unchanged) {
            if (record["passed"].get<bool>()) ++failures;
            record["passed"] = false;
            record["status"] = "failed";
            record["input_nonmutation_error"] = "A public Boolean or acceptance check mutated an original operand";
        }
        record["seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        write_bytes(path, record.dump(2));
        std::cout << record.dump() << std::endl;
    }
    return failures ? 1 : 0;
}

}
