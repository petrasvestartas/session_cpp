#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "intersection.h"
#include "nurbs_offset.h"
#include "primitives.h"
#include "session.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#ifdef SESSION_WITH_OCCT
#error Native viewer evidence must be built without OCCT
#endif

using namespace session_cpp;

namespace {

const double EvidenceTolerance = 1e-6;
const Color SourceColor(75 / 255.f, 125 / 255.f, 170 / 255.f);
const Color ToolColor(200 / 255.f, 145 / 255.f, 85 / 255.f);
const Color ResultColor(180 / 255.f, 110 / 255.f, 155 / 255.f);
const Color IsoColor(55 / 255.f, 45 / 255.f, 65 / 255.f);

struct Evidence {
    std::filesystem::path folder;
    nlohmann::ordered_json cases = nlohmann::ordered_json::array();
    int failures = 0;

    void checkpoint() const {
        const nlohmann::ordered_json report = {
            {"backend", "session_cpp_native"}, {"boolean_acceptance", "not covered by this exporter"},
            {"cases", cases}, {"failures", failures},
            {"geometry_placement", "Original parameterized world coordinates, without display transforms"},
            {"legend", {{"source_a", "muted blue (75,125,170)"}, {"source_b", "muted orange (200,145,85)"},
                {"result", "muted mauve (180,110,155)"}, {"surface_iso_curves", "dark plum (55,45,65); exact native iso-curves"}}},
            {"renderer", "session_viewer native selftest; this program exports scenes only"}
        };
        std::ofstream(folder / "native_geometry_cases.json") << report.dump(2);
    }

    void save(const std::string& name, Session& scene, nlohmann::ordered_json report) {
        const std::string file = name + ".pb";
        scene.pb_dump((folder / file).string());
        report["name"] = name;
        report["scene"] = file;
        if (!report.value("passed", false))
            ++failures;
        cases.push_back(report);
        checkpoint();
        std::cout << name << ": " << (report.value("passed", false) ? "passed" : "failed") << '\n';
    }

    void fail(const std::string& name, const std::exception& error) {
        ++failures;
        cases.push_back({{"error", error.what()}, {"name", name}, {"passed", false}});
        checkpoint();
        std::cout << name << ": " << error.what() << '\n';
    }
};

void add_curve(Session& scene, NurbsCurve curve, const Color& color) {
    curve.linecolors = {color};
    curve.width = 3.0;
    scene.add_nurbscurve(curve);
}

void add_surface(Session& scene, NurbsSurface surface, const Color& color) {
    surface.facecolors = {color};
    surface.linecolors = {Color::black()};
    scene.add_nurbssurface(surface);
}

void add_iso_curves(Session& scene, const NurbsSurface& surface) {
    for (int direction = 0; direction < 2; ++direction) {
        const std::pair<double, double> fixed = surface.domain(1 - direction);
        for (int i = 1; i < 8; ++i) {
            const double parameter = fixed.first + (fixed.second - fixed.first) * i / 8.0;
            NurbsCurve curve = surface.iso_curve(direction, parameter);
            if (!curve.is_valid())
                throw std::runtime_error("Native surface iso-curve is invalid");
            curve.name = "native offset surface iso-curve";
            curve.linecolors = {IsoColor};
            curve.width = 1.3;
            scene.add_nurbscurve(curve);
        }
    }
}

void add_hit(Session& scene, Point point) {
    point.name = "native intersection";
    point.pointcolor = ResultColor;
    point.width = 10.0;
    scene.add_point(point);
}

NurbsSurface plane(double height) {
    return NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {{-1, -1, height}, {-1, 1, height}, {1, -1, height}, {1, 1, height}}
    );
}

NurbsSurface saddle(double sign) {
    std::vector<Point> points;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            points.push_back({double(i), double(j), 0.15 * sign * (i - 1.5) * (j - 1.5) + (sign < 0.0 ? 0.1 : 0.0)});
    return NurbsSurface::create(
        false,
        false,
        3,
        3,
        4,
        4,
        points
    );
}

NurbsSurface bowl() {
    const std::array<double, 3> heights = {1.0, -1.0, 1.0};
    std::vector<Point> points;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({i - 1.0, j - 1.0, heights[i] + heights[j]});
    return NurbsSurface::create(
        false,
        false,
        2,
        2,
        3,
        3,
        points
    );
}

void curve_offset_case(
    Evidence& evidence,
    const std::string& name,
    NurbsCurve source,
    double distance
) {
    source.set_domain(-7.0, 3.0);
    source.name = "source";
    NurbsCurve result = nurbs_offset::curve(source, distance, {0, 0, 1}, EvidenceTolerance);
    result.name = "native offset";
    double error = 0.0;
    for (int i = 0; i <= 997; ++i) {
        const double t = -7.0 + 10.0 * i / 997.0;
        const std::vector<Vector> derivatives = source.evaluate(t, 1);
        const Vector side = derivatives[1].cross({0, 0, 1}).normalized();
        const Point expected = source.point_at(t) + side * distance;
        error = std::max(error, expected.distance(result.point_at(t)));
    }
    Session scene(name);
    add_curve(scene, source, SourceColor);
    add_curve(scene, result, ResultColor);
    evidence.save(name, scene, {
        {"distance", distance}, {"kind", "curve_offset"}, {"max_sampled_residual", error},
        {"parameter_domain", {source.domain_start(), source.domain_end()}},
        {"passed", result.is_valid() && result.domain() == source.domain() && error <= EvidenceTolerance},
        {"result_control_points", result.cv_count()}, {"samples", 998}, {"tolerance", EvidenceTolerance}
    });
}

void surface_offset_case(
    Evidence& evidence,
    const std::string& name,
    NurbsSurface source,
    double distance
) {
    source.set_domain(0, 13.0, 17.0);
    source.set_domain(1, -9.0, -2.0);
    source.name = "source";
    NurbsSurface result = nurbs_offset::surface(source, distance, EvidenceTolerance);
    result.name = "native offset";
    double error = 0.0;
    for (int i = 0; i < 47; ++i)
        for (int j = 0; j < 43; ++j) {
            const double u = 13.0 + 4.0 * (i + 0.37) / 47.0;
            const double v = -9.0 + 7.0 * (j + 0.61) / 43.0;
            const Point expected = source.point_at(u, v) + source.normal_at(u, v) * distance;
            error = std::max(error, expected.distance(result.point_at(u, v)));
        }
    Session scene(name);
    add_surface(scene, source, SourceColor);
    add_surface(scene, result, ResultColor);
    Session result_scene(name + " result only");
    add_surface(result_scene, result, ResultColor);
    add_iso_curves(result_scene, result);
    const std::string result_file = name + "_result.pb";
    result_scene.pb_dump((evidence.folder / result_file).string());
    evidence.save(name, scene, {
        {"distance", distance}, {"kind", "surface_offset"}, {"max_sampled_residual", error},
        {"parameter_domain_u", {13.0, 17.0}}, {"parameter_domain_v", {-9.0, -2.0}},
        {"passed", result.is_valid() && result.domain(0) == source.domain(0)
            && result.domain(1) == source.domain(1) && error <= EvidenceTolerance},
        {"result_control_points", {result.cv_count(0), result.cv_count(1)}},
        {"result_scene", result_file}, {"result_iso_curves", 14},
        {"samples", 47 * 43}, {"tolerance", EvidenceTolerance}
    });
}

void curve_curve_case(Evidence& evidence, int kind) {
    const std::string names[] = {"curve_crossing", "curve_tangent", "curve_overlap"};
    const NurbsCurve a = NurbsCurve::create(false, 1, {{-3, 0, 0}, {3, 0, 0}});
    const std::array<NurbsCurve, 3> others = {
        NurbsCurve::create(false, 1, {{0, -2, 0}, {0, 2, 0}}),
        Primitives::circle(0, 1, 0, 1),
        NurbsCurve::create(false, 1, {{-1, 0, 0}, {4, 0, 0}})
    };
    const NurbsCurve& b = others[kind];
    const curve_intersection::Result result = curve_intersection::intersect(a, b, EvidenceTolerance);
    Session scene(names[kind]);
    add_curve(scene, a, SourceColor);
    add_curve(scene, b, ToolColor);
    double error = 0.0;
    for (const curve_intersection::Hit& hit : result.points) {
        error = std::max(error, a.point_at(hit.a).distance(b.point_at(hit.b)));
        add_hit(scene, hit.point);
    }
    for (const curve_intersection::Overlap& overlap : result.overlaps) {
        NurbsCurve shared = a;
        if (!shared.trim(overlap.a0, overlap.a1))
            throw std::runtime_error("Failed to trim native overlap for viewer scene");
        add_curve(scene, shared, ResultColor);
    }
    const bool counts = kind == 2 ? result.overlaps.size() == 1 && result.points.empty()
        : result.points.size() == 1 && result.overlaps.empty();
    evidence.save(names[kind], scene, {
        {"complete", result.complete}, {"failure", result.failure}, {"kind", "curve_curve"},
        {"max_sampled_residual", error}, {"overlaps", result.overlaps.size()},
        {"passed", result.complete && counts && error <= EvidenceTolerance},
        {"points", result.points.size()}, {"tolerance", EvidenceTolerance}
    });
}

void curve_surface_case(Evidence& evidence, bool tangent) {
    const std::string name = tangent ? "freeform_curve_surface_tangent" : "freeform_curve_surface_two_roots";
    const NurbsSurface source = bowl();
    const double z = tangent ? 0.04 : 0.29;
    const NurbsCurve curve = NurbsCurve::create(false, 1, {{-1, 0.2, z}, {1, 0.2, z}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, source, EvidenceTolerance);
    Session scene(name);
    add_surface(scene, source, SourceColor);
    add_curve(scene, curve, ToolColor);
    double error = 0.0;
    for (const curve_surface_intersection::Hit& hit : result.points) {
        error = std::max(error, curve.point_at(hit.t).distance(source.point_at(hit.u, hit.v)));
        add_hit(scene, hit.point);
    }
    evidence.save(name, scene, {
        {"complete", result.complete}, {"failure", result.failure}, {"kind", "curve_surface"},
        {"max_sampled_residual", error}, {"overlaps", result.overlaps.size()},
        {"passed", result.complete && result.points.size() == (tangent ? 1 : 2) && error <= EvidenceTolerance},
        {"points", result.points.size()}, {"tolerance", EvidenceTolerance}
    });
}

void surface_surface_case(Evidence& evidence, bool interior_loop) {
    const std::string name = interior_loop ? "freeform_small_interior_loop" : "freeform_two_surface_branches";
    const NurbsSurface a = interior_loop ? bowl() : saddle(1.0);
    const NurbsSurface b = interior_loop ? plane(0.01) : saddle(-1.0);
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections = Intersection::surface_surface(a, b, 1e-7);
    Session scene(name);
    add_surface(scene, a, SourceColor);
    add_surface(scene, b, ToolColor);
    double error = 0.0;
    double length = 0.0;
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections) {
        const NurbsCurve& curve = std::get<0>(section);
        const NurbsCurve& pa = std::get<1>(section);
        const NurbsCurve& pb = std::get<2>(section);
        add_curve(scene, curve, ResultColor);
        length += curve.length();
        for (int i = 0; i <= 509; ++i) {
            const double t = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 509.0;
            const Point world = curve.point_at(t);
            const Point ua = pa.point_at(t);
            const Point ub = pb.point_at(t);
            error = std::max({error, world.distance(a.point_at(ua[0], ua[1])), world.distance(b.point_at(ub[0], ub[1]))});
        }
    }
    const bool geometry = interior_loop ? std::abs(length - 0.2 * 3.141592653589793) < 1e-5 : sections.size() == 2;
    evidence.save(name, scene, {
        {"acceptance_tolerance", 2e-6}, {"kind", "surface_surface"}, {"length", length},
        {"max_sampled_residual", error}, {"passed", geometry && !sections.empty() && error <= 2e-6},
        {"samples_per_section", 510}, {"sections", sections.size()}, {"tolerance", 1e-7}
    });
}

}

namespace native_viewer_evidence {

int run(int argc, char** argv) {
    if (argc > 1) {
        try {
            if (argc != 5 || std::string(argv[1]) != "--reexport-brep")
                throw std::invalid_argument("Usage: native_viewer_evidence --reexport-brep input.brep.json output.pb source|result");
            const std::filesystem::path input = argv[2];
            const std::filesystem::path output = argv[3];
            const std::string role = argv[4];
            if (role != "source" && role != "result")
                throw std::invalid_argument("Re-export role must be source or result");
            if (output.extension() != ".pb" || std::filesystem::exists(output))
                throw std::invalid_argument("Re-export requires a new .pb output path");
            BRep geometry = BRep::file_json_load(input.string());
            if (!geometry.is_valid())
                throw std::invalid_argument("Re-export input has invalid BRep references");
            auto original = geometry.jsondump();
            const auto old_color = original.at("surfacecolor");
            geometry.surfacecolor = role == "source" ? SourceColor : ResultColor;
            auto recolored = geometry.jsondump();
            original.erase("surfacecolor");
            recolored.erase("surfacecolor");
            if (original != recolored)
                throw std::runtime_error("Display re-export changed fields other than surfacecolor");
            if (!output.parent_path().empty())
                std::filesystem::create_directories(output.parent_path());
            Session scene(geometry.name);
            scene.add_brep(geometry);
            scene.pb_dump(output.string());
            const nlohmann::ordered_json report = {{"source", std::filesystem::absolute(input).string()},
                {"scene", output.filename().string()}, {"role", role}, {"original_color", old_color},
                {"display_color", geometry.surfacecolor.jsondump()}, {"geometry_unchanged", true},
                {"offset_recomputed", false}, {"display_transforms", false}};
            std::ofstream(output.string() + ".reexport.json") << report.dump(2);
            std::cout << "Re-exported unchanged BRep geometry to " << output.string() << '\n';
            return 0;
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            return 1;
        }
    }
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    Evidence evidence{root / "build/native-geometry-tests/viewer"};
    std::filesystem::create_directories(evidence.folder);
    const std::array<NurbsCurve, 3> curves = {
        NurbsCurve::create(false, 1, {{0, 0, 0}, {3, 0, 0}}),
        Primitives::circle(0, 0, 0, 2),
        NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0.4}, {2, -1, -0.2}, {3, 1, 1}})
    };
    const std::string curve_names[] = {"line", "circle", "freeform_curve"};
    for (int i = 0; i < 3; ++i)
        for (double distance : {-0.2, 0.2}) {
            const std::string name = curve_names[i] + (distance < 0 ? "_offset_negative" : "_offset_positive");
            try {
                curve_offset_case(evidence, name, curves[i], distance);
            } catch (const std::exception& error) {
                evidence.fail(name, error);
            }
        }
    const std::array<NurbsSurface, 4> surfaces = {plane(0.0), Primitives::sphere_surface(0, 0, 0, 2),
        Primitives::cylinder_surface(0, 0, 0, 2, 3), saddle(1.0)};
    const std::string surface_names[] = {"plane", "sphere", "cylinder", "freeform_surface"};
    for (int i = 0; i < 4; ++i)
        for (double distance : {-0.2, 0.2}) {
            const std::string name = surface_names[i] + (distance < 0 ? "_offset_negative" : "_offset_positive");
            try {
                surface_offset_case(evidence, name, surfaces[i], distance);
            } catch (const std::exception& error) {
                evidence.fail(name, error);
            }
        }
    for (int i = 0; i < 7; ++i)
        try {
            if (i < 3)
                curve_curve_case(evidence, i);
            else if (i < 5)
                curve_surface_case(evidence, i == 4);
            else
                surface_surface_case(evidence, i == 6);
        } catch (const std::exception& error) {
            evidence.fail("intersection_case_" + std::to_string(i), error);
        }
    evidence.checkpoint();
    return evidence.failures ? 1 : 0;
}


}

#include <limits>
#include "tolerance.h"

namespace {

void intersection_bytes(const std::filesystem::path& path, const std::string& bytes) {
    if (bytes.size() > static_cast<size_t>(std::numeric_limits<std::streamsize>::max()))
        throw std::runtime_error("Native evidence file exceeds stream size: " + path.string());
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("Could not open native evidence file: " + path.string());
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    output.flush();
    if (!output)
        throw std::runtime_error("Could not write native evidence file: " + path.string());
    output.close();
    if (output.fail())
        throw std::runtime_error("Could not close native evidence file: " + path.string());
}

void intersection_checkpoint(const Evidence& evidence) {
    const nlohmann::ordered_json report = {{"backend", "session_cpp_native"}, {"cases", evidence.cases},
        {"failures", evidence.failures}, {"geometry_placement", "Original world coordinates; no display transforms"},
        {"legend", {{"source_a", "blue"}, {"source_b", "orange"}, {"computed_result", "mauve"}}},
        {"renderer", "actual SessionViewer loads exported scenes; does not recompute intersections"},
        {"numerical_acceptance_from_image", false}};
    const std::string bytes = report.dump(2);
    intersection_bytes(evidence.folder / "native_geometry_cases.json", bytes);
}

void intersection_inputs(Evidence& evidence, const std::string& name, Session& scene,
                         const nlohmann::ordered_json& geometry) {
    const std::string scene_bytes = scene.pb_dumps();
    intersection_bytes(evidence.folder / (name + ".inputs.pb"), scene_bytes);
    const std::string geometry_bytes = geometry.dump(2);
    intersection_bytes(evidence.folder / (name + ".inputs.json"), geometry_bytes);
}

void intersection_result(Evidence& evidence, const std::string& name, Session& scene,
                         nlohmann::ordered_json report) {
    report["inputs"] = name + ".inputs.json";
    report["input_scene"] = name + ".inputs.pb";
    report["renderer_recomputes_intersection"] = false;
    report["numerical_acceptance_from_image"] = false;
    report["scene"] = name + ".pb";
    report["name"] = name;
    const std::string scene_bytes = scene.pb_dumps();
    intersection_bytes(evidence.folder / (name + ".pb"), scene_bytes);
    const std::string assertion_bytes = report.dump(2);
    intersection_bytes(evidence.folder / (name + ".assertions.json"), assertion_bytes);
    if (!report.value("passed", false))
        ++evidence.failures;
    evidence.cases.push_back(std::move(report));
    intersection_checkpoint(evidence);
}

void intersection_failure(Evidence& evidence, const std::string& name, const std::exception& error) {
    ++evidence.failures;
    const nlohmann::ordered_json report = {{"passed", false}, {"name", name}, {"error", error.what()},
        {"input_scene", name + ".inputs.pb"}, {"inputs", name + ".inputs.json"},
        {"input_scene_exists", std::filesystem::exists(evidence.folder / (name + ".inputs.pb"))},
        {"input_geometry_exists", std::filesystem::exists(evidence.folder / (name + ".inputs.json"))},
        {"renderer_recomputes_intersection", false}, {"numerical_acceptance_from_image", false}};
    const std::string bytes = report.dump(2);
    intersection_bytes(evidence.folder / (name + ".assertions.json"), bytes);
    evidence.cases.push_back(report);
    intersection_checkpoint(evidence);
}

void intersection_curves(Evidence& evidence, const std::string& name, const NurbsCurve& a,
                         const NurbsCurve& b, size_t points, size_t overlaps, bool tangent) {
    const double tolerance = 1e-7;
    Session scene(name);
    add_curve(scene, a, SourceColor);
    add_curve(scene, b, ToolColor);
    intersection_inputs(evidence, name, scene, {{"first_curve", a.jsondump()}, {"second_curve", b.jsondump()}});
    const curve_intersection::Result result = curve_intersection::intersect(a, b, tolerance);
    nlohmann::ordered_json hits = nlohmann::ordered_json::array(), shared = nlohmann::ordered_json::array();
    double residual = 0;
    bool domains = true, tangencies = true, full_overlap_coverage = true;
    for (const curve_intersection::Hit& hit : result.points) {
        const double gap = a.point_at(hit.a).distance(b.point_at(hit.b));
        const double first_gap = hit.point.distance(a.point_at(hit.a));
        const double second_gap = hit.point.distance(b.point_at(hit.b));
        domains &= std::isfinite(gap) && std::isfinite(first_gap) && std::isfinite(second_gap);
        residual = std::max({residual, gap, first_gap, second_gap});
        domains &= hit.a >= a.domain_start() && hit.a <= a.domain_end()
            && hit.b >= b.domain_start() && hit.b <= b.domain_end();
        tangencies &= hit.tangent == tangent;
        hits.push_back({{"point", {hit.point[0], hit.point[1], hit.point[2]}}, {"first_parameter", hit.a},
            {"second_parameter", hit.b}, {"tangent", hit.tangent}, {"residual", gap}});
        add_hit(scene, hit.point);
    }
    for (const curve_intersection::Overlap& overlap : result.overlaps) {
        NurbsCurve curve = a;
        if (!curve.trim(overlap.a0, overlap.a1))
            throw std::runtime_error("Could not trim computed native curve overlap");
        const double first = a.point_at(overlap.a0).distance(b.point_at(overlap.b0));
        const double last = a.point_at(overlap.a1).distance(b.point_at(overlap.b1));
        domains &= std::isfinite(first) && std::isfinite(last);
        residual = std::max({residual, first, last});
        domains &= overlap.a0 >= a.domain_start() && overlap.a1 <= a.domain_end() && overlap.a0 < overlap.a1
            && std::min(overlap.b0, overlap.b1) >= b.domain_start() && std::max(overlap.b0, overlap.b1) <= b.domain_end();
        const bool full = std::abs(overlap.a0 - a.domain_start()) < 1e-9
            && std::abs(overlap.a1 - a.domain_end()) < 1e-9
            && std::abs(overlap.b0 - b.domain_start()) < 1e-9
            && std::abs(overlap.b1 - b.domain_end()) < 1e-9;
        full_overlap_coverage &= full;
        shared.push_back({{"full_expected_coverage", full}, {"first_interval", {overlap.a0, overlap.a1}}, {"second_interval", {overlap.b0, overlap.b1}},
            {"endpoint_residuals", {first, last}}, {"world", curve.jsondump()}});
        add_curve(scene, curve, ResultColor);
    }
    intersection_result(evidence, name, scene, {{"kind", "curve_curve"}, {"complete", result.complete},
        {"failure", result.failure}, {"points", hits}, {"overlaps", shared}, {"expected_points", points},
        {"expected_overlaps", overlaps}, {"expected_tangent", tangent}, {"original_parameter_domains", domains},
        {"full_expected_overlap_coverage", full_overlap_coverage}, {"overlap_endpoint_parameter_tolerance", 1e-9},
        {"expected_first_interval", {a.domain_start(), a.domain_end()}},
        {"expected_second_interval", {b.domain_start(), b.domain_end()}},
        {"max_event_or_overlap_endpoint_residual", residual}, {"tolerance", tolerance},
        {"passed", result.complete && result.points.size() == points && result.overlaps.size() == overlaps
            && domains && tangencies && full_overlap_coverage && std::isfinite(residual) && residual <= tolerance}});
}

void intersection_curve_surface(Evidence& evidence, const std::string& name, const NurbsCurve& curve,
    const NurbsSurface& surface, size_t expected, int tangent, const Point& target) {
    const double tolerance = 1e-7;
    Session scene(name);
    add_surface(scene, surface, SourceColor);
    add_curve(scene, curve, ToolColor);
    intersection_inputs(evidence, name, scene, {{"curve", curve.jsondump()}, {"surface", surface.jsondump()}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface, tolerance);
    nlohmann::ordered_json hits = nlohmann::ordered_json::array();
    double residual = 0, target_gap = 0;
    bool domains = true, tangencies = true;
    const std::pair<double, double> u = surface.domain(0), v = surface.domain(1);
    for (const curve_surface_intersection::Hit& hit : result.points) {
        const double gap = curve.point_at(hit.t).distance(surface.point_at(hit.u, hit.v));
        const double curve_gap = hit.point.distance(curve.point_at(hit.t));
        const double surface_gap = hit.point.distance(surface.point_at(hit.u, hit.v));
        const double target_error = hit.point.distance(target);
        domains &= std::isfinite(gap) && std::isfinite(curve_gap) && std::isfinite(surface_gap) && std::isfinite(target_error);
        residual = std::max({residual, gap, curve_gap, surface_gap});
        target_gap = std::max(target_gap, target_error);
        domains &= hit.t >= curve.domain_start() && hit.t <= curve.domain_end()
            && hit.u >= u.first && hit.u <= u.second && hit.v >= v.first && hit.v <= v.second;
        tangencies &= tangent < 0 || hit.tangent == bool(tangent);
        hits.push_back({{"point", {hit.point[0], hit.point[1], hit.point[2]}}, {"t", hit.t}, {"uv", {hit.u, hit.v}},
            {"tangent", hit.tangent}, {"residual", gap}});
        add_hit(scene, hit.point);
    }
    intersection_result(evidence, name, scene, {{"kind", "curve_surface"}, {"complete", result.complete},
        {"failure", result.failure}, {"points", hits}, {"overlaps", result.overlaps.size()},
        {"expected_points", expected}, {"expected_tangent", tangent}, {"expected_target", {target[0], target[1], target[2]}},
        {"max_target_gap", target_gap}, {"original_parameter_domains", domains}, {"max_event_residual", residual},
        {"tolerance", tolerance}, {"passed", result.complete && result.points.size() == expected && result.overlaps.empty()
            && domains && tangencies && std::isfinite(residual) && residual <= tolerance && target_gap <= tolerance}});
}

void intersection_seam_sections(Evidence& evidence) {
    const std::string name = "ssi_sphere_cylinder_raw_seams";
    const double tolerance = Tolerance::ZERO_TOLERANCE;
    const NurbsSurface a = Primitives::sphere_surface(0, 0, 0, 2);
    const NurbsSurface b = Primitives::cylinder_surface(1.3, 0, -3, .3, 6);
    Session scene(name);
    add_surface(scene, a, SourceColor);
    add_surface(scene, b, ToolColor);
    intersection_inputs(evidence, name, scene, {{"first_surface", a.jsondump()}, {"second_surface", b.jsondump()}});
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections = Intersection::surface_surface(a, b);
    nlohmann::ordered_json curves = nlohmann::ordered_json::array();
    bool paired = true;
    double residual = 0, analytic = 0, seam_gap = 0;
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections) {
        const NurbsCurve& world = std::get<0>(section);
        const NurbsCurve& first = std::get<1>(section);
        const NurbsCurve& second = std::get<2>(section);
        const bool same_domains = world.domain() == first.domain() && world.domain() == second.domain();
        const bool first_proved = same_domains && curve_surface_intersection::same_parameter(world, first, a, 1e-6)
            == curve_surface_intersection::ParameterMatch::Matched;
        const bool second_proved = same_domains && curve_surface_intersection::same_parameter(world, second, b, 1e-6)
            == curve_surface_intersection::ParameterMatch::Matched;
        paired &= first_proved && second_proved;
        for (int i = 0; i <= 512; ++i) {
            const double t = world.domain_start() + (world.domain_end() - world.domain_start()) * i / 512;
            const Point point = world.point_at(t), ua = first.point_at(t), ub = second.point_at(t);
            const double first_error = point.distance(a.point_at(ua[0], ua[1]));
            const double second_error = point.distance(b.point_at(ub[0], ub[1]));
            const double sphere_error = std::abs(std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]) - 2);
            const double cylinder_error = std::abs(std::hypot(point[0] - 1.3, point[1]) - .3);
            paired &= std::isfinite(first_error) && std::isfinite(second_error) && std::isfinite(sphere_error) && std::isfinite(cylinder_error);
            residual = std::max({residual, first_error, second_error});
            analytic = std::max({analytic, sphere_error, cylinder_error});
        }
        for (const Point& uv : {first.point_at_start(), first.point_at_end()})
            seam_gap = std::max(seam_gap, std::min(std::abs(uv[0]), std::abs(uv[0] - 4)));
        curves.push_back({{"world", world.jsondump()}, {"first_pcurve", first.jsondump()}, {"second_pcurve", second.jsondump()},
            {"common_raw_parameter_domain", same_domains}, {"first_same_parameter", first_proved}, {"second_same_parameter", second_proved}});
        add_curve(scene, world, ResultColor);
    }
    intersection_result(evidence, name, scene, {{"kind", "surface_surface"}, {"sections", curves}, {"expected_branches", 4},
        {"max_sampled_paired_residual", residual}, {"max_sampled_analytic_residual", analytic}, {"seam_endpoint_gap", seam_gap},
        {"samples_per_section", 513}, {"paired_interval_certificates", paired}, {"tolerance", tolerance},
        {"paired_certificate_tolerance", 1e-6}, {"passed", sections.size() == 4 && paired && std::isfinite(residual)
            && residual <= 1e-6 && analytic < 1e-5 && seam_gap < 1e-8}});
}

void intersection_fixture(Evidence& evidence, const std::string& name) {
    if (name == "cc_rational_tangent")
        return intersection_curves(evidence, name, Primitives::circle(0, 0, 0, 1), Primitives::circle(0, 2, 0, 1), 1, 0, true);
    if (name == "cc_rational_refined_overlap") {
        NurbsCurve a = Primitives::circle(0, 0, 0, 1);
        if (!a.set_domain(0, 1) || !a.trim(.03, .68))
            throw std::runtime_error("Could not prepare tested rational overlap input");
        NurbsCurve b = a;
        if (!b.increase_degree(4) || !b.insert_nurbsknot(.32, 2))
            throw std::runtime_error("Could not refine tested rational overlap input");
        return intersection_curves(evidence, name, a, b, 0, 1, false);
    }
    if (name == "cs_isolated_corner")
        return intersection_curve_surface(evidence, name, NurbsCurve::create(false, 1, {{0, 2, 0}, {2, 0, 0}}),
            plane(0), 1, -1, {1, 1, 0});
    if (name == "cs_planar_tangent")
        return intersection_curve_surface(evidence, name,
            NurbsCurve::create(false, 2, {{-.5, 0, .25}, {0, 0, -.25}, {.5, 0, .25}}), plane(0), 1, true, {0, 0, 0});
    if (name == "cs_rational_freeform") {
        const NurbsSurface polynomial = bowl();
        NurbsSurface surface(3, true, 3, 3, 3, 3);
        for (int axis = 0; axis < 2; ++axis)
            for (int i = 0; i < surface.nurbsknot_count(axis); ++i)
                if (!surface.set_nurbsknot(axis, i, polynomial.nurbsknot(axis, i)))
                    throw std::runtime_error("Could not prepare tested rational support knots");
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                const Point point = polynomial.get_cv(i, j);
                const double weight = 1 + .2 * i + .15 * j;
                if (!surface.set_cv_4d(i, j, point[0] * weight, point[1] * weight, point[2] * weight, weight))
                    throw std::runtime_error("Could not prepare tested rational support controls");
            }
        const Point target = surface.point_at(.37, .61);
        return intersection_curve_surface(evidence, name, NurbsCurve::create(false, 1,
            {target + Vector(0, 0, -3), target + Vector(0, 0, 3)}), surface, 1, false, target);
    }
    if (name == "ssi_sphere_cylinder_raw_seams")
        return intersection_seam_sections(evidence);
    throw std::invalid_argument("Unknown native intersection evidence case");
}
}

namespace native_viewer_evidence {
int intersections(int argc, char** argv) {
    if (argc < 2 || argc > 3)
        throw std::invalid_argument("Usage: native_geometry_evidence intersections NEW-output-folder [case]");
    Evidence evidence{std::filesystem::absolute(argv[1])};
    if (std::filesystem::exists(evidence.folder))
        throw std::invalid_argument("Intersection evidence requires a new output folder");
    if (!std::filesystem::create_directories(evidence.folder))
        throw std::invalid_argument("Intersection evidence output folder already exists");
    const std::array<std::string, 6> names = {"cc_rational_tangent", "cc_rational_refined_overlap", "cs_isolated_corner",
        "cs_planar_tangent", "cs_rational_freeform", "ssi_sphere_cylinder_raw_seams"};
    const std::string selected = argc == 3 ? argv[2] : "all";
    if (selected != "all" && std::find(names.begin(), names.end(), selected) == names.end())
        throw std::invalid_argument("Unknown native intersection evidence case");
    for (const std::string& name : names) {
        if (selected != "all" && selected != name)
            continue;
        try {
            intersection_fixture(evidence, name);
        } catch (const std::exception& error) {
            intersection_failure(evidence, name, error);
        }
    }
    intersection_checkpoint(evidence);
    return evidence.failures ? 1 : 0;
}
}
