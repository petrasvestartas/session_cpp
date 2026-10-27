#include "surface_section.h"
#include "intersection.h"
#include "curve_surface_intersection.h"
#include "ssi_diagnostics.h"
#include "mini_test.h"
#include "primitives.h"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

nlohmann::ordered_json ssi_statistics(const ssi_diagnostics::Statistics& stats) {
    nlohmann::ordered_json timings;
    for (size_t i = 0; i < ssi_diagnostics::names.size(); ++i)
        timings[ssi_diagnostics::names[i]] = stats.seconds[i];
    const nlohmann::ordered_json result = {{"timings_are_inclusive", true}, {"seconds", timings},
        {"cells", stats.cells}, {"max_depth", stats.max_depth}, {"box_rejections", stats.box_rejections},
        {"projection_rejections", stats.projection_rejections}, {"root_searches", stats.root_searches},
        {"corrections", stats.corrections}, {"roots_found", stats.roots_found},
        {"coverage_checks", stats.coverage_checks}, {"covered", stats.covered},
        {"seed_comparisons", stats.seed_comparisons}, {"traces", stats.traces}, {"pieces", stats.pieces},
        {"continuation_arcs", stats.continuation_arcs}, {"local_projections", stats.local_projections},
        {"junction_polishes", stats.junction_polishes}, {"polished_junctions", stats.polished_junctions},
        {"continuation_failures", stats.continuation_failures}, {"continuation_max_depth", stats.continuation_max_depth},
        {"continuation_first", stats.continuation_first}, {"continuation_last", stats.continuation_last},
        {"continuation_middle", stats.continuation_middle}, {"continuation_failure_stage", stats.continuation_failure_stage},
        {"continuation_diagnostic_calls", stats.continuation_diagnostic_calls},
        {"continuation_diagnostic_depth", stats.continuation_diagnostic_depth},
        {"continuation_diagnostic", stats.continuation_diagnostic.empty() ? nlohmann::ordered_json() :
            nlohmann::ordered_json::parse(stats.continuation_diagnostic)},
        {"fit_iterations", stats.fit_iterations}, {"fit_sizes", stats.fit_sizes},
        {"accepted_seeds", stats.accepted_seeds}, {"trace_starts", stats.trace_starts},
        {"duplicate_seeds", stats.duplicate_seeds}, {"consumed_seeds", stats.consumed_seeds}};
    return result;
}

void save_ssi_statistics(const std::filesystem::path& path, const ssi_diagnostics::Statistics& stats) {
    std::ofstream(path) << ssi_statistics(stats).dump(2);
}

void save_ssi_progress(
    void* context,
    const ssi_diagnostics::Statistics& stats,
    ssi_diagnostics::Stage stage,
    double elapsed
) noexcept {
    try {
        const auto& path = *static_cast<const std::filesystem::path*>(context);
        auto snapshot = ssi_statistics(stats);
        snapshot["elapsed_seconds"] = elapsed;
        snapshot["last_completed_stage"] = stage == ssi_diagnostics::Stage::Count ? "none" :
            ssi_diagnostics::names[static_cast<size_t>(stage)];
        std::ofstream(path) << snapshot.dump(2);
    } catch (...) {
    }
}

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> profiled_sections(
    const NurbsSurface& a, const NurbsSurface& b, double tolerance, const std::filesystem::path& path) {
    ssi_diagnostics::Statistics statistics;
    std::filesystem::path progress = path;
    progress.replace_extension(".progress.json");
    save_ssi_progress(&progress, statistics, ssi_diagnostics::Stage::Count, 0.0);
    ssi_diagnostics::Capture capture(statistics, save_ssi_progress, &progress);
    try {
        auto result = Intersection::surface_surface(a, b, tolerance);
        save_ssi_statistics(path, statistics);
        return result;
    } catch (...) {
        save_ssi_statistics(path, statistics);
        throw;
    }
}

std::array<double, 25> circle_polynomial(double u, double v, double radius) {
    std::array<double, 25> coefficients = {};
    coefficients[0] = u * u + v * v - radius * radius;
    coefficients[5] = -2.0 * u;
    coefficients[1] = -2.0 * v;
    coefficients[10] = coefficients[2] = 1.0;
    return coefficients;
}

NurbsSurface polynomial_surface(const std::array<double, 25>& coefficients) {
    const int binomial[5][5] = {{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1}, {1, 4, 6, 4, 1}};
    std::vector<Point> points;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
            double height = 0.0;
            for (int u = 0; u <= i; ++u)
                for (int v = 0; v <= j; ++v)
                    height += coefficients[u * 5 + v] * binomial[i][u] / binomial[4][u] *
                        binomial[j][v] / binomial[4][v];
            points.push_back({i / 4.0, j / 4.0, height});
        }
    return NurbsSurface::create(false, false, 4, 4, 5, 5, points);
}

std::pair<NurbsSurface, NurbsSurface> two_loop_surfaces() {
    std::array<double, 25> coefficients = {};
    coefficients[10] = coefficients[2] = 0.1;
    const NurbsSurface a = polynomial_surface(coefficients);
    const std::array<double, 25> first = circle_polynomial(0.31, 0.32, 0.025);
    const std::array<double, 25> second = circle_polynomial(0.43, 0.42, 0.025);
    for (int u = 0; u < 5; ++u)
        for (int v = 0; v < 5; ++v)
            for (int s = 0; s + u < 5; ++s)
                for (int t = 0; t + v < 5; ++t)
                    coefficients[(u + s) * 5 + v + t] += 100.0 * first[u * 5 + v] * second[s * 5 + t];
    return {a, polynomial_surface(coefficients)};
}

void check_sections(
    const std::vector<std::pair<NurbsCurve, NurbsCurve>>& sections,
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance
) {
    for (const std::pair<NurbsCurve, NurbsCurve>& section : sections) {
        MINI_CHECK(section.first.domain_start() == 0.0 && section.first.domain_end() == 1.0);
        MINI_CHECK(section.second.domain_start() == 0.0 && section.second.domain_end() == 1.0);
        for (int i = 0; i <= 512; ++i) {
            const double parameter = i / 512.0;
            const Point point = section.first.point_at(parameter);
            const Point uv = section.second.point_at(parameter);
            MINI_CHECK(point.distance(surface.point_at(uv[0], uv[1])) <= tolerance);
            MINI_CHECK(std::abs((point - plane.origin()).dot(plane.z_axis())) <= tolerance);
            MINI_CHECK(uv[0] >= surface.domain(0).first - tolerance && uv[0] <= surface.domain(0).second + tolerance);
            MINI_CHECK(uv[1] >= surface.domain(1).first - tolerance && uv[1] <= surface.domain(1).second + tolerance);
        }
    }
}

}

MINI_TEST("Surface Section Native", "Exact Oblique Cylinder Plane") {
    const NurbsSurface surface = Primitives::cylinder_surface(0, 0, 0, 2, 4);
    const Plane plane = Plane::from_point_normal({0, 0, 2}, {0.4, 0.2, 1});
    const std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> sections = surface_section::extrusion_plane(surface, plane, 1e-12);
    MINI_CHECK(sections.has_value());
    MINI_CHECK(sections->size() == 1);
    MINI_CHECK(sections->front().first.is_closed());
    check_sections(*sections, surface, plane, 1e-12);
}

MINI_TEST("Surface Section Native", "Finite Cylinder Clipping") {
    const NurbsSurface surface = Primitives::cylinder_surface(0, 0, 0, 2, 1);
    const Plane plane = Plane::from_point_normal({0, 0, 0.5}, {1, 0, 1});
    const std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> sections = surface_section::extrusion_plane(surface, plane, 1e-12);
    MINI_CHECK(sections.has_value());
    MINI_CHECK(sections->size() == 2);
    check_sections(*sections, surface, plane, 1e-12);
    for (const std::pair<NurbsCurve, NurbsCurve>& section : *sections) {
        const double start = section.first.point_at_start()[2];
        const double end = section.first.point_at_end()[2];
        MINI_CHECK(std::min(std::abs(start), std::abs(start - 1.0)) < 1e-12);
        MINI_CHECK(std::min(std::abs(end), std::abs(end - 1.0)) < 1e-12);
    }
}

MINI_TEST("Surface Section Native", "Rotated And Transposed Domains") {
    NurbsSurface surface = Primitives::cylinder_surface(0, 0, 0, 2, 4);
    MINI_CHECK(surface.set_domain(0, 10, 22));
    MINI_CHECK(surface.set_domain(1, -5, -2));
    MINI_CHECK(surface.transpose());
    const Xform rotation = Xform::rotation({1, 2, 3}, 0.73);
    surface = surface.transformed(rotation);
    const Plane original = Plane::from_point_normal({0, 0, 2}, {0.4, 0.2, 1});
    const Plane plane = original.transformed(rotation);
    const std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> sections = surface_section::extrusion_plane(surface, plane, 1e-12);
    MINI_CHECK(sections.has_value() && sections->size() == 1);
    check_sections(*sections, surface, plane, 1e-12);
}

MINI_TEST("Surface Section Native", "Rational Freeform Extrusion") {
    NurbsCurve curve = NurbsCurve::create(false, 3, {{-2, 0, 0}, {-1, 2, 0.3}, {1, -1, -0.2}, {2, 1, 0}});
    MINI_CHECK(curve.set_weight(1, 0.4));
    MINI_CHECK(curve.set_weight(2, 1.3));
    MINI_CHECK(curve.set_domain(8, 13));
    const NurbsSurface surface = Primitives::create_extrusion(curve, {0.3, 0.1, 4});
    const Plane plane = Plane::from_point_normal({0, 0, 2}, {0.1, -0.1, 1});
    const std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> sections = surface_section::extrusion_plane(surface, plane, 1e-12);
    MINI_CHECK(sections.has_value() && sections->size() == 1);
    check_sections(*sections, surface, plane, 1e-12);
}

MINI_TEST("Surface Section Native", "Miss And Parallel Unsupported") {
    const NurbsSurface surface = Primitives::cylinder_surface(0, 0, 0, 2, 1);
    const Plane distant = Plane::from_point_normal({0, 0, 4}, {0.1, 0, 1});
    const std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> miss = surface_section::extrusion_plane(surface, distant, 1e-12);
    MINI_CHECK(miss.has_value() && miss->empty());
    const Plane parallel = Plane::from_point_normal({0, 0, 0}, {1, 0, 0});
    MINI_CHECK(!surface_section::extrusion_plane(surface, parallel, 1e-12).has_value());
    const NurbsSurface cone = Primitives::cone_surface(0, 0, 0, 2, 4);
    MINI_CHECK(!surface_section::extrusion_plane(cone, distant, 1e-12).has_value());
}

MINI_TEST("Surface Section Native", "Narrow Knot Span Freeform Branches") {
    NurbsSurface bump(3, false, 4, 3, 10, 3);
    bump.m_nurbsknot[0] = {0, 0, 0, 1e-9, 1e-9, 1e-9, 2e-9, 2e-9, 2e-9, 1, 1, 1};
    bump.m_nurbsknot[1] = {0, 0, 1, 1};
    const double heights[] = {0, 0, 0.5, 1, 0.5, 0, 0, 0, 0, 0};
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 3; ++j)
            bump.set_cv(i, j, {i / 3.0, j - 1.0, heights[i]});
    const double squared[] = {1, -1, 1};
    std::vector<Point> points;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({i * 1.5, j - 1.0, 0.1 + 0.02 * squared[j]});
    const NurbsSurface cutter = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
    MINI_CHECK(bump.is_valid() && cutter.is_valid());
    const auto sections = Intersection::surface_surface(bump, cutter, 1e-7);
    MINI_CHECK(sections.size() == 2);
    for (const auto& section : sections)
        for (int i = 0; i <= 512; ++i) {
            const double t = i / 512.0;
            const Point world = std::get<0>(section).point_at(t);
            const Point pa = std::get<1>(section).point_at(t);
            const Point pb = std::get<2>(section).point_at(t);
            MINI_CHECK(pa[0] >= 0 && pa[0] <= 2e-9);
            MINI_CHECK(world.distance(bump.point_at(pa[0], pa[1])) <= 1e-7);
            MINI_CHECK(world.distance(cutter.point_at(pb[0], pb[1])) <= 1e-7);
        }
}

MINI_TEST("Surface Section Native", "Coincident Freeform Reports Unresolved") {
    const double heights[] = {1, -1, 1};
    std::vector<Point> points;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({i - 1.0, j - 1.0, heights[i] + heights[j]});
    const NurbsSurface bowl = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
    bool failed = false;
    try {
        Intersection::surface_surface(bowl, bowl, 1e-7);
    } catch (const std::runtime_error&) {
        failed = true;
    }
    MINI_CHECK(failed);
}

MINI_TEST("Surface Section Native", "Two Closed Loops Inside One Knot Cell") {
    const auto [a, b] = two_loop_surfaces();
    const std::filesystem::path folder = "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    const nlohmann::ordered_json fixture = {{"surfaces", {a.jsondump(), b.jsondump()}},
        {"expected_uv_circles", {{0.31, 0.32, 0.025}, {0.43, 0.42, 0.025}}}, {"tolerance", 1e-7}};
    std::ofstream(folder / "two_closed_loops_one_cell.fixture.json") << fixture.dump(2);
    const auto sections = profiled_sections(a, b, 1e-7, folder / "two_closed_loops_one_cell.statistics.json");
    nlohmann::ordered_json observed = {{"component_count", sections.size()}, {"sections", nlohmann::ordered_json::array()}};
    for (const auto& section : sections)
        observed["sections"].push_back({{"world", std::get<0>(section).jsondump()},
            {"pcurve_a", std::get<1>(section).jsondump()}, {"pcurve_b", std::get<2>(section).jsondump()}});
    std::ofstream(folder / "two_closed_loops_one_cell.result.json") << observed.dump(2);
    if (sections.size() != 2)
        throw MiniTestAssertionError("Expected 2 closed components, obtained " + std::to_string(sections.size()));
    std::array<bool, 2> found = {};
    for (const auto& section : sections) {
        const NurbsCurve& world = std::get<0>(section);
        MINI_CHECK(world.is_closed());
        const Point first = std::get<1>(section).point_at_start();
        const int circle = std::hypot(first[0] - 0.31, first[1] - 0.32) < 0.05 ? 0 : 1;
        MINI_CHECK(!found[circle]);
        found[circle] = true;
        const double u = circle == 0 ? 0.31 : 0.43;
        const double v = circle == 0 ? 0.32 : 0.42;
        for (int i = 0; i <= 256; ++i) {
            const double t = i / 256.0;
            const Point point = world.point_at(t);
            const Point pa = std::get<1>(section).point_at(t);
            const Point pb = std::get<2>(section).point_at(t);
            MINI_CHECK(std::abs(std::hypot(pa[0] - u, pa[1] - v) - 0.025) < 1e-6);
            MINI_CHECK(point.distance(a.point_at(pa[0], pa[1])) <= 1e-7);
            MINI_CHECK(point.distance(b.point_at(pb[0], pb[1])) <= 1e-7);
        }
    }
    MINI_CHECK(found[0] && found[1]);
}

MINI_TEST("Surface Section Native", "Mobius Folded Parameter Branches") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "docs/fixtures/mobius_folded_parameter_branches.json");
    MINI_CHECK(input.good());
    const nlohmann::json fixture = nlohmann::json::parse(input);
    const NurbsSurface a = NurbsSurface::jsonload(fixture["surfaces"][0]);
    const NurbsSurface b = NurbsSurface::jsonload(fixture["surfaces"][1]);
    const double scale = fixture["mobius_scale"].get<double>();
    const double tolerance = fixture["tolerance"].get<double>();
    MINI_CHECK(a.is_valid() && b.is_valid());
    const std::filesystem::path folder = "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    std::ofstream(folder / "mobius_folded_parameter_branches.fixture.json") << fixture.dump(2);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections;
    try {
        sections = profiled_sections(a, b, tolerance, folder / "mobius_folded_parameter_branches.statistics.json");
    } catch (const std::exception& error) {
        const nlohmann::ordered_json result = {{"complete", false}, {"error", error.what()}};
        std::ofstream(folder / "mobius_folded_parameter_branches.result.json") << result.dump(2);
        throw;
    }
    nlohmann::ordered_json result = {{"complete", true}, {"component_count", sections.size()},
        {"sections", nlohmann::ordered_json::array()}};
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections)
        result["sections"].push_back({{"world", std::get<0>(section).jsondump()},
            {"pcurve_a", std::get<1>(section).jsondump()}, {"pcurve_b", std::get<2>(section).jsondump()}});
    std::ofstream(folder / "mobius_folded_parameter_branches.result.json") << result.dump(2);
    if (sections.size() != 2)
        throw MiniTestAssertionError("Expected 2 parameter branches, obtained " + std::to_string(sections.size()));
    std::ifstream statistics_input(folder / "mobius_folded_parameter_branches.statistics.json");
    const nlohmann::json statistics = nlohmann::json::parse(statistics_input);
    for (const char* key : {"duplicate_seeds", "consumed_seeds"})
        for (const auto& pair : statistics[key]) {
            const double u0 = pair[0][0].get<double>();
            const double u1 = pair[1][0].get<double>();
            const double s0 = scale * u0 / (1.0 + (scale - 1.0) * u0);
            const double s1 = scale * u1 / (1.0 + (scale - 1.0) * u1);
            MINI_CHECK((s0 < 0.5) == (s1 < 0.5));
        }
    std::array<bool, 2> found = {};
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections) {
        const NurbsCurve& world = std::get<0>(section);
        const Point start = world.point_at_start();
        const Point end = world.point_at_end();
        MINI_CHECK(std::abs(std::abs(start[1]) - 1.0) <= tolerance);
        MINI_CHECK(std::abs(std::abs(end[1]) - 1.0) <= tolerance);
        MINI_CHECK(start[1] * end[1] < 0.0);
        const double u = std::get<1>(section).point_at_start()[0];
        const double s = scale * u / (1.0 + (scale - 1.0) * u);
        const int branch = s < 0.5 ? 0 : 1;
        MINI_CHECK(!found[branch]);
        found[branch] = true;
        for (int i = 0; i <= 512; ++i) {
            const double t = world.domain_start() + (world.domain_end() - world.domain_start()) * i / 512.0;
            const Point point = world.point_at(t);
            const Point pa = std::get<1>(section).point_at(t);
            const Point pb = std::get<2>(section).point_at(t);
            const double recovered = scale * pa[0] / (1.0 + (scale - 1.0) * pa[0]);
            MINI_CHECK(std::abs(recovered - (branch == 0 ? 0.25 : 0.75)) <= tolerance * 4.0);
            MINI_CHECK(std::abs(point[0] - 0.0625) <= tolerance);
            MINI_CHECK(std::abs(point[2] - 0.125 * point[1] * point[1]) <= tolerance);
            MINI_CHECK(point.distance(a.point_at(pa[0], pa[1])) <= tolerance);
            MINI_CHECK(point.distance(b.point_at(pb[0], pb[1])) <= tolerance);
        }
    }
    MINI_CHECK(found[0] && found[1]);
}

MINI_TEST("Surface Section Native", "Chair Duplicate Walking Endpoints") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    for (const std::string name : {"chair_section_15_18.json", "chair_section_19_0.json"}) {
        std::ifstream input(root / "tests/fixtures" / name);
        MINI_CHECK(input.good());
        const nlohmann::json fixture = nlohmann::json::parse(input);
        const NurbsSurface a = NurbsSurface::jsonload(fixture["first_surface"]);
        const NurbsSurface b = NurbsSurface::jsonload(fixture["second_surface"]);
        const double tolerance = fixture["tolerance"].get<double>();
        const auto sections = Intersection::surface_surface(a, b, tolerance);
        MINI_CHECK(sections.size() == fixture["occt_support_curves"].get<size_t>());
        for (const nlohmann::json& ends : fixture["occt_support_endpoints"])
            for (const nlohmann::json& end : ends) {
                const Point expected(end[0].get<double>(), end[1].get<double>(), end[2].get<double>());
                bool covered = false;
                for (const auto& section : sections) {
                    const NurbsCurve& world = std::get<0>(section);
                    covered = covered || world.point_at(world.domain_start()).distance(expected) <= tolerance ||
                        world.point_at(world.domain_end()).distance(expected) <= tolerance;
                }
                MINI_CHECK(covered);
            }
        for (const auto& section : sections) {
            const NurbsCurve& world = std::get<0>(section);
            const NurbsCurve& ua = std::get<1>(section);
            const NurbsCurve& ub = std::get<2>(section);
            MINI_CHECK(world.domain() == ua.domain() && world.domain() == ub.domain());
            for (const double t : {world.domain_start(), world.domain_end()}) {
                const Point pa = ua.point_at(t), pb = ub.point_at(t);
                const std::array<double, 4> parameters = {pa[0], pa[1], pb[0], pb[1]};
                const NurbsSurface* supports[2] = {&a, &b};
                bool boundary = false;
                for (int k = 0; k < 4; ++k) {
                    const std::pair<double, double> domain = supports[k / 2]->domain(k % 2);
                    MINI_CHECK(parameters[k] >= domain.first && parameters[k] <= domain.second);
                    boundary = boundary || parameters[k] == domain.first || parameters[k] == domain.second;
                }
                MINI_CHECK(boundary);
            }
            for (int i = 0; i <= 1000; ++i) {
                const double t = world.domain_start() + (world.domain_end() - world.domain_start()) * i / 1000.0;
                const Point point = world.point_at(t), pa = ua.point_at(t), pb = ub.point_at(t);
                MINI_CHECK(point.distance(a.point_at(pa[0], pa[1])) <= tolerance);
                MINI_CHECK(point.distance(b.point_at(pb[0], pb[1])) <= tolerance);
            }
            using curve_surface_intersection::ParameterMatch;
            MINI_CHECK(curve_surface_intersection::same_parameter(world, ua, a, tolerance) == ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, ub, b, tolerance) == ParameterMatch::Matched);
        }
    }
}
