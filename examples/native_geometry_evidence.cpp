#include "intersection.h"
#include "curve_surface_intersection.h"
#include "native_tessellate_diagnostics.h"
#include "curve_intersection.h"
#include <cstdio>
#include "brep_measure.h"
#include "brep_acceptance.h"
#include "file_step.h"
#include "session.h"
#include <iterator>
#include <limits>
#include <chrono>
#include "brep_assemble.h"
#include "split.h"
#include "section_fit_error.h"
#include "ssi_diagnostics.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <exception>
#include <iostream>
#include <string>

namespace native_chair_booleans {
int run(int argc, char** argv);
}

namespace native_chair_sections {
int run(int argc, char** argv);
}

namespace native_viewer_evidence {
int run(int argc, char** argv);
int intersections(int argc, char** argv);
}


namespace native_ssi_pair {
using namespace session_cpp;

nlohmann::ordered_json statistics(const ssi_diagnostics::Statistics& value) {
    nlohmann::ordered_json seconds;
    for (size_t i = 0; i < ssi_diagnostics::names.size(); ++i)
        seconds[ssi_diagnostics::names[i]] = value.seconds[i];
    return {{"timings_are_inclusive", true}, {"seconds", seconds},
        {"cells", value.cells}, {"max_depth", value.max_depth}, {"box_rejections", value.box_rejections},
        {"projection_rejections", value.projection_rejections}, {"root_searches", value.root_searches},
        {"corrections", value.corrections}, {"roots_found", value.roots_found},
        {"coverage_checks", value.coverage_checks}, {"covered", value.covered},
        {"seed_comparisons", value.seed_comparisons}, {"traces", value.traces}, {"pieces", value.pieces},
        {"continuation_arcs", value.continuation_arcs}, {"local_projections", value.local_projections},
        {"junction_polishes", value.junction_polishes}, {"polished_junctions", value.polished_junctions},
        {"continuation_failures", value.continuation_failures},
        {"continuation_max_depth", value.continuation_max_depth},
        {"continuation_first", value.continuation_first}, {"continuation_last", value.continuation_last},
        {"continuation_middle", value.continuation_middle}, {"continuation_failure_stage", value.continuation_failure_stage},
        {"continuation_diagnostic_calls", value.continuation_diagnostic_calls},
        {"continuation_diagnostic_depth", value.continuation_diagnostic_depth},
        {"continuation_diagnostic", value.continuation_diagnostic.empty() ? nlohmann::ordered_json() :
            nlohmann::ordered_json::parse(value.continuation_diagnostic)},
        {"fit_iterations", value.fit_iterations}, {"fit_sizes", value.fit_sizes},
        {"accepted_seeds", value.accepted_seeds}, {"trace_starts", value.trace_starts},
        {"duplicate_seeds", value.duplicate_seeds}, {"consumed_seeds", value.consumed_seeds}};
}

void save(const std::filesystem::path& path, const nlohmann::ordered_json& value) {
    std::ofstream output(path);
    if (!(output << value.dump(2)))
        throw std::runtime_error("Could not save SSI pair evidence: " + path.string());
}

void progress(void* context, const ssi_diagnostics::Statistics& value,
    ssi_diagnostics::Stage stage, double elapsed) noexcept {
    try {
        const std::filesystem::path& path = *static_cast<const std::filesystem::path*>(context);
        nlohmann::ordered_json snapshot = statistics(value);
        snapshot["elapsed_seconds"] = elapsed;
        snapshot["last_completed_stage"] = stage == ssi_diagnostics::Stage::Count ? "none" :
            ssi_diagnostics::names[static_cast<size_t>(stage)];
        save(path, snapshot);
    } catch (...) {
    }
}

nlohmann::ordered_json points(const std::vector<Point>& values) {
    nlohmann::ordered_json result = nlohmann::ordered_json::array();
    for (const Point& value : values)
        result.push_back({value[0], value[1], value[2]});
    return result;
}

int run(int argc, char** argv) {
    if (argc < 5 || argc > 6)
        throw std::runtime_error("Usage: native_geometry_evidence ssi-pair input.json first-index second-index output.json [tolerance]");
    const std::filesystem::path input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const std::filesystem::path output_path = std::filesystem::absolute(argv[4]).lexically_normal();
    if (input_path == output_path || (std::filesystem::exists(output_path) && std::filesystem::equivalent(input_path, output_path)))
        throw std::runtime_error("SSI pair output must differ from input");
    std::ifstream input(input_path);
    if (!input)
        throw std::runtime_error("Could not open SSI pair input");
    const nlohmann::ordered_json fixture = nlohmann::ordered_json::parse(input);
    const int first = std::stoi(argv[2]);
    const int second = std::stoi(argv[3]);
    const nlohmann::ordered_json& surfaces = fixture.at("surfaces");
    if (first < 0 || second < 0 || first >= static_cast<int>(surfaces.size()) || second >= static_cast<int>(surfaces.size()))
        throw std::runtime_error("SSI pair surface index is out of range");
    const double tolerance = argc == 6 ? std::stod(argv[5]) : fixture.at("tolerance").get<double>() / 16;
    if (!std::isfinite(tolerance) || tolerance <= 0)
        throw std::runtime_error("SSI pair tolerance must be positive and finite");
    const NurbsSurface a = NurbsSurface::jsonload(surfaces[first]);
    const NurbsSurface b = NurbsSurface::jsonload(surfaces[second]);
    if (!a.is_valid() || !b.is_valid())
        throw std::runtime_error("SSI pair input contains an invalid surface");
    std::filesystem::create_directories(output_path.parent_path());
    std::filesystem::path execution_path = output_path;
    execution_path.replace_extension(".input.json");
    std::filesystem::path progress_path = output_path;
    progress_path.replace_extension(".progress.json");
    if (execution_path == input_path || progress_path == input_path)
        throw std::runtime_error("SSI pair sidecar must differ from input");
    nlohmann::ordered_json report = {{"status", "running"}, {"input", input_path.string()},
        {"surface_indices", {first, second}}, {"tolerance", tolerance},
        {"tolerance_source", argc == 6 ? "explicit CLI" : "InwardError tolerance divided by 16"},
        {"source_edge", fixture.value("source_edge", -1)}, {"source_faces", fixture.value("source_faces", nlohmann::ordered_json::array())},
        {"surfaces", {surfaces[first], surfaces[second]}}};
    save(execution_path, report);
    save(output_path, report);
    ssi_diagnostics::Statistics captured;
    progress(&progress_path, captured, ssi_diagnostics::Stage::Count, 0);
    int result = 0;
    {
        ssi_diagnostics::Capture capture(captured, progress, &progress_path);
        try {
            const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections =
                Intersection::surface_surface(a, b, tolerance);
            report["sections"] = nlohmann::ordered_json::array();
            for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections)
                report["sections"].push_back({{"world", std::get<0>(section).jsondump()},
                    {"first_pcurve", std::get<1>(section).jsondump()}, {"second_pcurve", std::get<2>(section).jsondump()}});
            report["section_count"] = sections.size();
            report["status"] = "complete";
        } catch (const section_fit::Error& error) {
            report["status"] = "section_fit_error";
            report["failure"] = error.what();
            report["fit_failure"] = {{"closed", error.closed}, {"samples", points(error.samples)},
                {"parameters", error.parameters}, {"world_samples", points(error.world_samples)},
                {"first_pcurve_samples", points(error.pcurve_samples)},
                {"second_pcurve_samples", points(error.second_pcurve_samples)}};
            result = 1;
        } catch (const std::exception& error) {
            report["status"] = "exception";
            report["failure"] = error.what();
            result = 1;
        }
    }
    report["statistics"] = statistics(captured);
    save(output_path, report);
    std::cout << "SSI pair " << first << ',' << second << ": " << report["status"].get<std::string>()
        << "; evidence " << output_path.string() << '\n';
    return result;
}
}

namespace native_arrangement {
using namespace session_cpp;

int run(int argc, char** argv) {
    if (argc != 3)
        throw std::runtime_error("Usage: native_geometry_evidence arrangement input.json output.json");
    const std::filesystem::path input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const std::filesystem::path output_path = std::filesystem::absolute(argv[2]).lexically_normal();
    if (input_path == output_path || (std::filesystem::exists(output_path) && std::filesystem::equivalent(input_path, output_path)))
        throw std::runtime_error("Arrangement output must differ from input");
    std::ifstream input(input_path);
    if (!input)
        throw std::runtime_error("Could not open arrangement input");
    const nlohmann::ordered_json fixture = nlohmann::ordered_json::parse(input);
    const BRep support = BRep::jsonload(fixture.at("arrangement_support"));
    const double tolerance = fixture.at("arrangement_tolerance").get<double>();
    if (support.face_count() != 1 || !support.is_valid() || !std::isfinite(tolerance) || tolerance <= 0)
        throw std::runtime_error("Arrangement requires one valid captured support and its positive original tolerance");
    std::vector<split::FaceCut> cuts;
    for (const nlohmann::ordered_json& cut : fixture.at("arrangement_cuts"))
        cuts.push_back({NurbsCurve::jsonload(cut.at("world")), NurbsCurve::jsonload(cut.at("pcurve")),
            cut.at("section").get<int>()});
    std::filesystem::create_directories(output_path.parent_path());
    nlohmann::ordered_json report = {{"status", "running"}, {"input", input_path.string()},
        {"support_index", fixture.at("support_index")}, {"source_face", fixture.at("source_face")},
        {"arrangement_tolerance", tolerance}, {"arrangement_support", fixture.at("arrangement_support")},
        {"arrangement_cuts", fixture.at("arrangement_cuts")}};
    native_ssi_pair::save(output_path, report);
    int code = 0;
    try {
        const split::FaceSplit result = split::split_brep_face_by_pcurves(support, 0, cuts, tolerance);
        report["status"] = "complete";
        report["partition"] = result.brep.jsondump();
        report["faces"] = result.faces;
        report["pieces"] = nlohmann::ordered_json::array();
        for (const split::EdgePiece& piece : result.edges)
            report["pieces"].push_back({{"source_edge", piece.source_edge}, {"section", piece.section},
                {"edge", piece.edge}, {"first", piece.first}, {"last", piece.last}});
    } catch (const std::exception& error) {
        report["status"] = "exception";
        report["failure"] = error.what();
        code = 1;
    }
    native_ssi_pair::save(output_path, report);
    std::cout << "Arrangement: " << report["status"].get<std::string>() << "; evidence " << output_path.string() << '\n';
    return code;
}
}

namespace native_assembly {
using namespace session_cpp;

int run(int argc, char** argv) {
    if (argc != 3)
        throw std::runtime_error("Usage: native_geometry_evidence assembly input.json output.json");
    const std::filesystem::path input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const std::filesystem::path output_path = std::filesystem::absolute(argv[2]).lexically_normal();
    if (input_path == output_path || (std::filesystem::exists(output_path) && std::filesystem::equivalent(input_path, output_path)))
        throw std::runtime_error("Assembly output must differ from input");
    std::ifstream input(input_path);
    if (!input)
        throw std::runtime_error("Could not open assembly input");
    const nlohmann::ordered_json fixture = nlohmann::ordered_json::parse(input);
    const double tolerance = fixture.at("tolerance").get<double>();
    const bool shared_shapes = fixture.contains("assembly_shapes");
    const nlohmann::ordered_json& inputs = fixture.at(shared_shapes ? "assembly_shapes" : "assembly_inputs");
    std::vector<BRep> shapes;
    std::vector<std::vector<split::EdgePiece>> pieces;
    for (const nlohmann::ordered_json& item : inputs) {
        shapes.push_back(BRep::jsonload(item.at("brep")));
        pieces.push_back({});
        for (const nlohmann::ordered_json& piece : item.at("pieces"))
            pieces.back().push_back({piece.at("source_edge").get<int>(), piece.at("section").get<int>(),
                piece.at("edge").get<int>(), piece.at("first").get<double>(), piece.at("last").get<double>()});
    }
    std::vector<brep_assemble::FaceUse> faces;
    if (shared_shapes) {
        for (const auto& face : fixture.at("assembly_faces")) {
            const size_t owner = face.at("shape").get<size_t>();
            if (owner >= shapes.size())
                throw std::runtime_error("Assembly face references an unknown shape owner");
            faces.push_back({&shapes[owner], face.at("face").get<int>(), face.at("reversed").get<bool>(),
                &pieces[owner], face.at("operand").get<int>()});
        }
    } else
        for (size_t i = 0; i < shapes.size(); ++i)
            faces.push_back({&shapes[i], inputs.at(i).at("face").get<int>(), inputs.at(i).at("reversed").get<bool>(),
                &pieces[i], inputs.at(i).at("operand").get<int>()});
    std::filesystem::create_directories(output_path.parent_path());
    nlohmann::ordered_json report = {{"status", "running"}, {"input", input_path.string()},
        {"tolerance", tolerance}, {"assembly_identity_contract", shared_shapes
            ? "shared shape owners preserved across selected FaceUses" : "legacy independent owner per assembly_inputs entry"}};
    report[shared_shapes ? "assembly_shapes" : "assembly_inputs"] = inputs;
    if (shared_shapes)
        report["assembly_faces"] = fixture.at("assembly_faces");
    native_ssi_pair::save(output_path, report);
    int code = 0;
    try {
        const BRep result = brep_assemble::assemble(faces, tolerance);
        report["status"] = "complete";
        report["result"] = result.jsondump();
    } catch (const brep_assemble::TopologyError& error) {
        report["status"] = "topology_error";
        report["failure"] = error.what();
        report["partial"] = error.partial.jsondump();
        report["face_orientations"] = nlohmann::ordered_json::array();
        for (BRepOrientation orientation : error.face_orientations)
            report["face_orientations"].push_back(static_cast<int>(orientation));
        report["canonical_images"] = nlohmann::ordered_json::array();
        for (const brep_assemble::TopologyImage& image : error.images)
            report["canonical_images"].push_back({{"input", image.input}, {"local_edge", image.local_edge},
                {"original_edge", image.original_edge}, {"section", image.section}, {"output_edge", image.output_edge},
                {"first", image.first}, {"last", image.last}, {"reversed", image.reversed}});
        code = 1;
    } catch (const std::exception& error) {
        report["status"] = "exception";
        report["failure"] = error.what();
        code = 1;
    }
    native_ssi_pair::save(output_path, report);
    std::cout << "Assembly: " << report["status"].get<std::string>() << "; evidence " << output_path.string() << '\n';
    return code;
}
}

namespace native_measure {
int run(int argc, char** argv) {
    if (argc < 3 || argc > 4)
        throw std::runtime_error("Usage: native_geometry_evidence measure input.brep.json output.json [tolerance]");
    const std::filesystem::path input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const std::filesystem::path output_path = std::filesystem::absolute(argv[2]).lexically_normal();
    if (input_path == output_path || (std::filesystem::exists(output_path) && std::filesystem::equivalent(input_path, output_path)))
        throw std::runtime_error("Volume evidence output must differ from input");
    std::ifstream input(input_path);
    if (!input)
        throw std::runtime_error("Could not open saved volume input");
    const session_cpp::BRep brep = session_cpp::BRep::jsonload(nlohmann::ordered_json::parse(input));
    const double tolerance = argc == 4 ? std::stod(argv[3]) : 1e-7;
    nlohmann::ordered_json report = {{"input", input_path.string()}, {"tolerance", tolerance}};
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    int code = 0;
    try {
        report["volume"] = session_cpp::brep_measure::volume(brep, tolerance);
        report["status"] = "complete";
    } catch (const std::exception& error) {
        report["status"] = "exception";
        report["failure"] = error.what();
        code = 1;
    }
    report["elapsed_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    native_ssi_pair::save(output_path, report);
    std::cout << "Volume: " << report["status"].get<std::string>() << "; evidence " << output_path.string() << '\n';
    return code;
}
}

namespace native_tessellation_evidence {
void write(const std::filesystem::path& path, const std::string& bytes, bool append = false) {
    if (bytes.size() > static_cast<size_t>(std::numeric_limits<std::streamsize>::max()))
        throw std::runtime_error("Tessellation evidence exceeds stream size");
    std::ofstream output(path, std::ios::binary | (append ? std::ios::app : std::ios::trunc));
    if (!output)
        throw std::runtime_error("Cannot open tessellation evidence: " + path.string());
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    output.flush();
    if (!output)
        throw std::runtime_error("Cannot write tessellation evidence: " + path.string());
    output.close();
    if (output.fail())
        throw std::runtime_error("Cannot close tessellation evidence: " + path.string());
}

double positive_argument(const char* text) {
    size_t consumed = 0;
    const std::string value(text);
    const double number = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(number) || !(number > 0))
        throw std::invalid_argument("Tessellation chord and angle must be finite positive numbers");
    return number;
}

int run(int argc, char** argv) {
    if (argc != 5)
        throw std::invalid_argument("Usage: native_geometry_evidence tessellate input.brep.json NEW-output-folder chord angle-radians");
    const double chord = positive_argument(argv[3]), angle = positive_argument(argv[4]);
    const std::filesystem::path input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const std::filesystem::path folder = std::filesystem::absolute(argv[2]).lexically_normal();
    if (std::filesystem::exists(folder) || !std::filesystem::create_directories(folder))
        throw std::invalid_argument("Tessellation evidence requires a new output directory");
    nlohmann::ordered_json report = {{"mode", "native_brep_tessellation_export"}, {"export_complete", false},
        {"input_path", input_path.string()}, {"immutable_input", "source.brep.json"},
        {"input_scene", "source.pb"}, {"scene", "tessellated.pb"}, {"chord", chord}, {"angle_radians", angle},
        {"renderer_recomputes_tessellation", false}, {"physical_brep_changed", false},
        {"kernel_acceptance", "not established by this exporter or its image; separate acceptance required"},
        {"sha256_binding", "required external input/executable/artifact receipt"}};
    const auto started = std::chrono::steady_clock::now();
    session_cpp::native_tessellate_diagnostics::State last;
    try {
        std::ifstream input(input_path, std::ios::binary);
        if (!input)
            throw std::runtime_error("Cannot open original BRep tessellation input");
        const std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (input.bad())
            throw std::runtime_error("Cannot read original BRep tessellation input");
        write(folder / "source.brep.json", bytes);
        const session_cpp::BRep brep = session_cpp::BRep::jsonload(nlohmann::json::parse(bytes));
        if (!brep.is_valid() || brep.face_count() == 0)
            throw std::invalid_argument("Tessellation evidence requires a valid nonempty BRep");
        session_cpp::Session original("Original BRep input");
        original.add_brep(brep);
        write(folder / "source.pb", original.pb_dumps());
        const std::function<void(const session_cpp::native_tessellate_diagnostics::State&)> sink =
            [&](const session_cpp::native_tessellate_diagnostics::State& state) {
                last = state;
                const nlohmann::ordered_json progress = {{"phase",state.phase},{"pass",state.pass},
                    {"triangles",state.triangles},{"visited",state.visited},{"certified",state.certified},
                    {"sampled",state.sampled},{"measured_passed",state.measured_passed},
                    {"checkpoint",state.checkpoint},{"face",state.face},{"uv",state.uv},
                    {"metadata_failure",state.metadata_failure},{"previous_phase",state.previous_phase},
                    {"elapsed_previous_phase_seconds",state.elapsed_previous_phase_seconds},
                    {"elapsed_current_phase_seconds",state.elapsed_current_phase_seconds}};
                write(folder / "tessellation.progress.jsonl",progress.dump()+"\n",true);
            };
        session_cpp::Mesh mesh;
        {
            const session_cpp::native_tessellate_diagnostics::Capture capture(sink);
            mesh = brep.tessellate(chord, angle);
        }
        const bool completed = std::string(last.phase)=="complete";
        const bool all_certified = completed && last.triangles>0 && last.visited==last.triangles &&
            last.certified==last.triangles && last.sampled==0;
        report["triangle_checks"] = {{"complete",completed},{"accepted_pass",last.pass},
            {"triangles",last.triangles},{"visited",last.visited},{"interval_certified",last.certified},
            {"sampled",last.sampled},{"measured_passed",last.measured_passed},
            {"all_triangles_interval_certified",all_certified}};
        report["triangle_check_contract"] = "Measured fallback is reported separately; export success alone is not an interval certificate";
        if (mesh.is_empty())
            throw std::runtime_error("Native tessellation returned an empty mesh for nonempty input");
        for (const auto& face : mesh.face)
            if (face.second.size() != 3)
                throw std::runtime_error("Native tessellation returned a nontriangular face");
        const double volume = mesh.volume();
        if (!std::isfinite(volume))
            throw std::runtime_error("Native tessellation mesh volume is nonfinite");
        mesh.set_objectcolor(session_cpp::Color(75 / 255.f, 125 / 255.f, 170 / 255.f));
        session_cpp::Session scene("Native C++ tessellation mesh");
        scene.add_mesh(mesh);
        write(folder / "tessellated.pb", scene.pb_dumps());
        report.update({{"vertices", mesh.number_of_vertices()}, {"triangles", mesh.number_of_faces()},
            {"closed", mesh.is_closed()}, {"mesh_absolute_volume", volume}, {"input_faces", brep.face_count()},
            {"input_solids", brep.solid_count()},
            {"mesh_volume_interpretation", "Absolute value of signed triangle-volume sum; descriptive for open meshes, not solid acceptance"},
            {"export_complete", true}, {"status", "exported"}});
        report["seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        write(folder / "tessellated.assertions.json", report.dump(2));
        return 0;
    } catch (const std::exception& error) {
        report.update({{"export_complete", false}, {"status", "failed"}, {"failure", error.what()}});
        report["last_tessellation_phase"] = last.phase;
        report["all_triangles_interval_certified"] = false;
        report["seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        write(folder / "tessellated.assertions.json", report.dump(2));
        throw;
    }
}
}


namespace native_parameter_evidence {
int run(int argc, char** argv) {
    using namespace session_cpp;
    if (argc != 3)
        throw std::invalid_argument("Usage: native_geometry_evidence same-parameter input.json NEW-output.json");
    const std::filesystem::path input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const std::filesystem::path output_path = std::filesystem::absolute(argv[2]).lexically_normal();
    if (input_path == output_path || std::filesystem::exists(output_path))
        throw std::invalid_argument("Same-parameter replay requires a new output distinct from input");
    std::ifstream input(input_path, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open same-parameter input");
    const std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (input.bad())
        throw std::runtime_error("Cannot read same-parameter input");
    const nlohmann::ordered_json fixture = nlohmann::ordered_json::parse(bytes);
    const NurbsCurve world = NurbsCurve::jsonload(fixture.at(fixture.contains("curve") ? "curve" : "world"));
    const NurbsCurve pc = NurbsCurve::jsonload(fixture.at("pcurve"));
    const NurbsSurface support = NurbsSurface::jsonload(fixture.at("surface"));
    const double tolerance = fixture.at("tolerance").get<double>();
    if (!std::isfinite(tolerance) || tolerance <= 0 || !world.is_valid() || !pc.is_valid() || !support.is_valid())
        throw std::invalid_argument("Same-parameter replay requires valid geometry and finite positive tolerance");
    std::string diagnostic;
    const auto started = std::chrono::steady_clock::now();
    const curve_surface_intersection::ParameterMatch state =
        curve_surface_intersection::same_parameter(world, pc, support, tolerance, &diagnostic);
    const char* name = state == curve_surface_intersection::ParameterMatch::Matched ? "matched" :
        state == curve_surface_intersection::ParameterMatch::Mismatched ? "mismatched" : "unresolved";
    const nlohmann::ordered_json report = {{"mode", "native_same_parameter_replay"}, {"complete", true},
        {"state", name}, {"state_value", static_cast<int>(state)}, {"diagnostic", diagnostic},
        {"tolerance", tolerance}, {"input_path", input_path.string()},
        {"world_domain", {world.domain_start(), world.domain_end()}},
        {"pcurve_domain", {pc.domain_start(), pc.domain_end()}},
        {"support_u_domain", {support.domain(0).first, support.domain(0).second}},
        {"support_v_domain", {support.domain(1).first, support.domain(1).second}},
        {"raw_domains_preserved", true}, {"world_cvs", world.cv_count()}, {"pcurve_cvs", pc.cv_count()},
        {"support_cvs", {support.cv_count(0), support.cv_count(1)}},
        {"seconds", std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()},
        {"sha256_binding", "required external input/executable/report receipt"}};
    const std::string output_bytes = report.dump(2);
#ifdef _WIN32
    std::FILE* output = _wfopen(output_path.c_str(), L"wbx");
#else
    std::FILE* output = std::fopen(output_path.c_str(), "wbx");
#endif
    if (!output)
        throw std::runtime_error("Cannot exclusively create same-parameter report");
    const bool written = std::fwrite(output_bytes.data(), 1, output_bytes.size(), output) == output_bytes.size();
    const bool flushed = std::fflush(output) == 0;
    const bool closed = std::fclose(output) == 0;
    if (!written || !flushed || !closed)
        throw std::runtime_error("Cannot write, flush or close same-parameter report");
    return state == curve_surface_intersection::ParameterMatch::Matched ? 0 : 1;
}
}


namespace native_curve_pair_evidence {
int run(int argc, char** argv) {
    using namespace session_cpp;
    if (argc != 3)
        throw std::invalid_argument("Usage: native_geometry_evidence curve-pair input.json[l] NEW-output.json");
    const auto input_path = std::filesystem::absolute(argv[1]).lexically_normal();
    const auto output_path = std::filesystem::absolute(argv[2]).lexically_normal();
    if (input_path == output_path || std::filesystem::exists(output_path))
        throw std::invalid_argument("Curve-pair replay requires a new output distinct from input");
    std::ifstream input(input_path, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open curve-pair input");
    const std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (input.bad())
        throw std::runtime_error("Cannot read curve-pair input");
    // Exactly one JSON object: single-record JSONL or an extracted observer record.
    // Parsing rejects multiple records rather than silently selecting one.
    const auto fixture = nlohmann::ordered_json::parse(bytes);
    const NurbsCurve a = NurbsCurve::jsonload(fixture.at("section_pcurve"));
    const NurbsCurve b = NurbsCurve::jsonload(fixture.at("boundary_pcurve"));
    const double tolerance = fixture.at("uv_tolerance").get<double>();
    if (!a.is_valid() || !b.is_valid() || !std::isfinite(tolerance) || tolerance <= 0)
        throw std::invalid_argument("Curve-pair replay requires valid curves and finite positive tolerance");
    const auto started = std::chrono::steady_clock::now();
    curve_intersection::Result events;
    bool threw = false;
    try {
        events = curve_intersection::intersect(a, b, tolerance);
    } catch (const std::exception& error) {
        threw = true;
        events.complete = false;
        events.failure = error.what();
    }
    nlohmann::ordered_json points = nlohmann::ordered_json::array();
    for (const auto& hit : events.points)
        points.push_back({{"point", {hit.point[0], hit.point[1], hit.point[2]}},
            {"a", hit.a}, {"b", hit.b}, {"tangent", hit.tangent}});
    nlohmann::ordered_json overlaps = nlohmann::ordered_json::array();
    for (const auto& range : events.overlaps)
        overlaps.push_back({{"a0", range.a0}, {"a1", range.a1}, {"b0", range.b0}, {"b1", range.b1}});
    const nlohmann::ordered_json report = {{"mode", "native_curve_pair_replay"},
        {"complete", events.complete}, {"failure", events.failure}, {"intersection_threw", threw},
        {"uv_tolerance", tolerance}, {"input_path", input_path.string()},
        {"a_domain", {a.domain_start(), a.domain_end()}}, {"b_domain", {b.domain_start(), b.domain_end()}},
        {"raw_domains_preserved", true}, {"points", points}, {"overlaps", overlaps},
        {"a_cvs", a.cv_count()}, {"b_cvs", b.cv_count()},
        {"seconds", std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()},
        {"evidence_scope", "intersection diagnostics only; no independent geometric acceptance"},
        {"sha256_binding", "required external input/executable/report receipt"}};
    const std::string output_bytes = report.dump(2);
#ifdef _WIN32
    std::FILE* output = _wfopen(output_path.c_str(), L"wbx");
#else
    std::FILE* output = std::fopen(output_path.c_str(), "wbx");
#endif
    if (!output)
        throw std::runtime_error("Cannot exclusively create curve-pair report");
    const bool written = std::fwrite(output_bytes.data(), 1, output_bytes.size(), output) == output_bytes.size();
    const bool flushed = std::fflush(output) == 0;
    const bool closed = std::fclose(output) == 0;
    if (!written || !flushed || !closed)
        throw std::runtime_error("Cannot write, flush or close curve-pair report");
    return events.complete ? 0 : 1;
}
}

namespace native_step_import_evidence {
using namespace session_cpp;

NurbsCurve edge_world(const BRep& shape, const BRepEdge& edge, const NurbsCurve& pc) {
    if (edge.curve_3d_index >= 0)
        return shape.m_curves_3d.at(edge.curve_3d_index);
    if (!edge.degenerated || edge.start_vertex != edge.end_vertex)
        throw std::runtime_error("Imported edge has no original world curve");
    // A degenerate edge's intrinsic world geometry is its ORIGINAL vertex.
    NurbsCurve point(3, false, 2, 2);
    point.m_nurbsknot = {pc.domain_start(), pc.domain_end()};
    const Point vertex = shape.m_vertices.at(edge.start_vertex).point;
    if (!point.set_cv(0, vertex) || !point.set_cv(1, vertex) || !point.is_valid())
        throw std::runtime_error("Cannot represent imported degenerate vertex image");
    return point;
}

bool check_pcurves(const BRep& shape, double tolerance, const std::filesystem::path& folder,
                   const std::string& prefix, nlohmann::ordered_json& report) {
    report["pcurves"] = nlohmann::ordered_json::array();
    bool complete = true, witness = false;
    for (size_t ei = 0; ei < shape.m_edges.size(); ++ei) {
        const auto& edge = shape.m_edges[ei];
        if (edge.pcurves.empty()) {
            complete = false;
            report["pcurves"].push_back({{"edge", ei}, {"failure", "Missing pcurve relations"}});
        }
        for (const auto& relation : edge.pcurves) {
            const std::array<std::pair<const char*, int>, 2> slots{
                {{"primary", relation.curve_2d_index}, {"secondary", relation.curve_2d_index_2}}};
            for (const auto& [slot, index] : slots) {
                if (index < 0 && std::string(slot) == "secondary")
                    continue;
                nlohmann::ordered_json row = {{"edge", ei},
                                              {"surface_index", relation.surface_index},
                                              {"slot", slot},
                                              {"pcurve_index", index},
                                              {"degenerate", edge.degenerated},
                                              {"tolerance", tolerance}};
                NurbsCurve world, pc;
                try {
                    pc = shape.m_curves_2d.at(index);
                    world = edge_world(shape, edge, pc);
                    const auto& support = shape.m_surfaces.at(relation.surface_index);
                    std::string diagnostic;
                    const auto match = curve_surface_intersection::same_parameter(world, pc, support,
                                                                                  tolerance, &diagnostic);
                    row["match"] = static_cast<int>(match);
                    row["diagnostic"] = diagnostic;
                    row["world_domain"] = {world.domain_start(), world.domain_end()};
                    row["pcurve_domain"] = {pc.domain_start(), pc.domain_end()};
                    row["raw_domains_equal"] = world.domain() == pc.domain();
                    row["raw_domain_identity_applicable"] = edge.curve_3d_index >= 0;
                    row["agreement_contract"] =
                        "independent normalized fractions; raw identity is diagnostic";
                    row["world_source"] =
                        edge.curve_3d_index >= 0 ? "original_curve" : "degenerate_original_vertex";
                    row["matched"] = match == curve_surface_intersection::ParameterMatch::Matched;
                } catch (const std::exception& error) {
                    row["matched"] = false;
                    row["exception"] = error.what();
                }
                if (!row.value("matched", false)) {
                    complete = false;
                    if (!witness) {
                        const auto filename = prefix + "-first-pcurve-failure.json";
                        auto failure = row;
                        failure["world"] = world.jsondump();
                        failure["pcurve"] = pc.jsondump();
                        if (relation.surface_index >= 0 &&
                            relation.surface_index < static_cast<int>(shape.m_surfaces.size()))
                            failure["surface"] = shape.m_surfaces[relation.surface_index].jsondump();
                        native_tessellation_evidence::write(folder / filename, failure.dump(2));
                        report["first_pcurve_failure"] = filename;
                        witness = true;
                    }
                }
                report["pcurves"].push_back(std::move(row));
            }
        }
    }
    report["whole_pcurves_certified"] = complete;
    return complete;
}

bool inspect_object(const BRep& shape, size_t index, double tolerance, const std::filesystem::path& folder,
                    nlohmann::ordered_json& report) {
    const std::string prefix = "object-" + std::to_string(index);
    report = {{"index", index},
              {"brep", prefix + ".brep.json"},
              {"faces", shape.face_count()},
              {"shells", shape.shell_count()},
              {"solids", shape.solid_count()},
              {"edges", shape.edge_count()},
              {"passed", false}};
    native_tessellation_evidence::write(folder / (prefix + ".brep.json"), shape.jsondump().dump());
    report["native_valid"] = shape.is_valid();
    report["native_solid"] = shape.is_solid();
    if (!report["native_valid"].get<bool>()) {
        report["failure"] = "Invalid native topology/references";
        return false;
    }
    bool links = true;
    report["shell_vertex_links"] = nlohmann::ordered_json::array();
    for (const auto& shell : shape.m_shells) {
        const auto status = brep_acceptance::shell_vertex_links(shape, shell);
        report["shell_vertex_links"].push_back(static_cast<int>(status));
        links &= status == brep_acceptance::VertexLinks::Matched;
    }
    report["vertex_links_certified"] = links;
    const bool pcurves = check_pcurves(shape, tolerance, folder, prefix, report);
    if (!report["native_solid"].get<bool>() || !links || !pcurves) {
        report["volume_status"] = "skipped_unvalidated_geometry";
        return false;
    }
    // Physical linear tolerance is NOT a relative volume tolerance.
    report["volume_relative_accuracy"] = 1e-7;
    report["volume_status"] = "running";
    report["signed_volume"] = shape.volume_exact(1e-7);
    report["volume_status"] = "complete";
    const double volume = report["signed_volume"].get<double>();
    const bool positive = std::isfinite(volume) && volume > 0;
    report["positive_signed_volume"] = positive;
    report["passed"] = positive;
    return positive;
}

int run(int argc, char** argv) {
    if (argc != 4)
        throw std::invalid_argument(
            "Usage: native_geometry_evidence step-import input.step NEW-output-folder linear-tolerance");
    size_t consumed = 0;
    const std::string argument(argv[3]);
    const double tolerance = std::stod(argument, &consumed);
    if (consumed != argument.size() || !std::isfinite(tolerance) || !(tolerance > 0))
        throw std::invalid_argument("STEP import linear tolerance must be finite and positive");
    const auto input = std::filesystem::absolute(argv[1]).lexically_normal();
    const auto folder = std::filesystem::absolute(argv[2]).lexically_normal();
    if (!std::filesystem::is_regular_file(input) || std::filesystem::exists(folder) ||
        !std::filesystem::create_directories(folder))
        throw std::invalid_argument("STEP import needs an existing input and NEW output folder");
    nlohmann::ordered_json report = {
        {"mode", "native_step_import_diagnostic"},
        {"status", "running"},
        {"input", input.string()},
        {"snapshot", "source.step"},
        {"linear_tolerance", tolerance},
        {"geometry_backend", "native_cpp_file_step"},
        {"independent_acceptance", false},
        {"sha256_binding", "required external STEP/GUID/executable/report receipt"},
        {"objects", nlohmann::ordered_json::array()},
        {"passed", false}};
    const auto report_path = folder / "report.json";
    const auto save = [&] { native_tessellation_evidence::write(report_path, report.dump(2)); };
    save();
    try {
        std::ifstream stream(input, std::ios::binary);
        if (!stream)
            throw std::runtime_error("Cannot open STEP input");
        const std::string bytes((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
        if (stream.bad())
            throw std::runtime_error("Cannot read STEP input");
        native_tessellation_evidence::write(folder / "source.step", bytes);
        const auto objects = file_step::read_file_step_breps((folder / "source.step").string(), tolerance);
        report["imported_brep_count"] = objects.size();
        save();
        bool passed = !objects.empty();
        if (objects.empty())
            report["failure"] = "No native BRep was imported";
        for (size_t i = 0; i < objects.size(); ++i) {
            report["objects"].push_back(nlohmann::ordered_json());
            auto& record = report["objects"].back();
            report["active_object"] = i;
            save();
            try {
                passed = inspect_object(objects[i], i, tolerance, folder, record) && passed;
            } catch (const std::exception& error) {
                record["failure"] = error.what();
                record["passed"] = false;
                passed = false;
            }
            save();
        }
        report["status"] = "complete";
        report["passed"] = passed;
    } catch (const std::exception& error) {
        report["status"] = "exception";
        report["failure"] = error.what();
    }
    save();
    std::cout << "STEP import: " << report["status"].get<std::string>() << "; evidence "
              << report_path.string() << '\n';
    return report["passed"].get<bool>() ? 0 : 1;
}
} // namespace native_step_import_evidence

int main(int argc, char** argv) {
    try {
        if (argc > 1) {
            const std::string mode = argv[1];
            if (mode == "step-import")
                return native_step_import_evidence::run(argc - 1, argv + 1);
            if (mode == "curve-pair")
                return native_curve_pair_evidence::run(argc - 1, argv + 1);
            if (mode == "same-parameter")
                return native_parameter_evidence::run(argc - 1, argv + 1);
            if (mode == "tessellate")
                return native_tessellation_evidence::run(argc - 1, argv + 1);
            if (mode == "measure")
                return native_measure::run(argc - 1, argv + 1);
            if (mode == "assembly")
                return native_assembly::run(argc - 1, argv + 1);
            if (mode == "arrangement")
                return native_arrangement::run(argc - 1, argv + 1);
            if (mode == "ssi-pair")
                return native_ssi_pair::run(argc - 1, argv + 1);
            if (mode == "booleans")
                return native_chair_booleans::run(argc - 1, argv + 1);
            if (mode == "sections")
                return native_chair_sections::run(argc - 1, argv + 1);
            if (mode == "intersections")
                return native_viewer_evidence::intersections(argc - 1, argv + 1);
            if (mode == "viewer")
                return native_viewer_evidence::run(argc - 1, argv + 1);
        }
        std::cerr << "Usage: native_geometry_evidence step-import input.step NEW-output-folder linear-tolerance | curve-pair input.json[l] NEW-output.json | same-parameter input.json NEW-output.json | tessellate input.brep.json NEW-output-folder chord angle-radians | measure input.brep.json output.json [tolerance] | booleans [operation] | sections [first second] | intersections NEW-output-folder [case] | viewer [--reexport-brep input output source|result] | ssi-pair input.json first-index second-index output.json [tolerance] | arrangement input.json output.json | assembly input.json output.json\n";
        return 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
