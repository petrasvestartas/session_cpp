#include "surface_section.h"
#include "intersection.h"
#include "curve_surface_intersection.h"
#include "ssi_diagnostics.h"
#include "nurbs_interval.h"
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
    std::ifstream input(root / "tests/fixtures/mobius_folded_parameter_branches.json");
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


namespace {
void check_rejected_seam_chart(const NurbsSurface& surface, const std::array<double, 4>& root) {
    NurbsSurface other(3, false, 2, 3, 2, 3);
    other.m_nurbsknot[0] = {-1, 1};
    other.m_nurbsknot[1] = {-1, -1, 1, 1};
    const double heights[] = {1.5, -0.5, 1.5};
    for (int u = 0; u < 2; ++u)
        for (int v = 0; v < 3; ++v)
            MINI_CHECK(other.set_cv(u, v, Point(2 * u - 1, v - 1, heights[v])));
    const std::array<double, 4> lower = {-0.125, 0.25, -1, -1};
    const std::array<double, 4> upper = {0.125, 0.75, 1, 1};
    const nlohmann::ordered_json witness = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(surface, other, lower, upper, root, 1e-8));
    MINI_CHECK(!witness["covered"].get<bool>());
    MINI_CHECK(witness["candidate_lower"] == nlohmann::ordered_json(lower));
    MINI_CHECK(witness["candidate_upper"] == nlohmann::ordered_json(upper));
    MINI_CHECK(witness["charts"].size() == 1);
    MINI_CHECK(witness["charts"][0]["status"] == "candidate_outside_original_domain");
    MINI_CHECK(witness["charts"][0]["lower"][0].get<double>() < surface.domain(0).first);
    std::array<double, 4> outside = root;
    outside[0] = -0.125;
    const std::array<double, 4> retained_lower = {0, 0.25, -1, -1};
    const nlohmann::ordered_json outside_witness = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(surface, other, retained_lower, upper, outside, 1e-8));
    MINI_CHECK(!outside_witness["covered"].get<bool>());
    MINI_CHECK(outside_witness["charts"].size() == 1);
    MINI_CHECK(outside_witness["charts"][0]["status"] == "root_outside_original_domain");
}
}

MINI_TEST("Surface Section Native", "Closed Cubic Extension Is Not Wrapped Field") {
    NurbsSurface surface(3, false, 4, 2, 4, 2);
    surface.m_nurbsknot[0] = {0, 0, 0, 1, 1, 1};
    surface.m_nurbsknot[1] = {0, 1};
    const double x[] = {0, 1.0 / 3, 1.0 / 3, 0};
    const double y[] = {0, -1.0 / 6, 1.0 / 6, 0};
    for (int u = 0; u < 4; ++u)
        for (int v = 0; v < 2; ++v)
            MINI_CHECK(surface.set_cv(u, v, Point(x[u], y[u], v)));
    MINI_CHECK(surface.is_valid());
    MINI_CHECK(surface.is_closed(0));
    MINI_CHECK(!surface.is_periodic(0));
    constexpr double e = 0.125;
    nurbs_interval::SurfaceBounds extension;
    MINI_CHECK(nurbs_interval::surface_bounds(surface, {-e, -e, 0.5, 0.5}, {0, 0, 0}, extension, false, true));
    nurbs_interval::SurfaceBounds derivative_box;
    MINI_CHECK(nurbs_interval::surface_bounds(surface, {-e, -e + 1e-6, 0.5, 0.500001},
        {0, 0, 0}, derivative_box, true, true));
    const std::vector<Vector> wrapped = surface.evaluate(1 - e, 0.5, 1);
    MINI_CHECK(extension.point[0].hi < wrapped[0][0]);
    MINI_CHECK(derivative_box.derivative[0][0].lo > wrapped[2][0]);
    MINI_CHECK(extension.point[0].lo <= -e - e * e && extension.point[0].hi >= -e - e * e);
    MINI_CHECK(std::abs(wrapped[0][0] - (e - e * e)) < 1e-12);
    MINI_CHECK(std::abs(wrapped[0][0] - (-e - e * e) - 2 * e) < 1e-12);
    check_rejected_seam_chart(surface, {0, 0.5, 0, 0});
}

MINI_TEST("Surface Section Native", "Periodic Cubic Extension Is Not Wrapped Field") {
    NurbsSurface surface(3, false, 4, 2, 7, 2);
    surface.m_nurbsknot[0] = {-2, -1, 0, 1, 2, 3, 4, 5, 6};
    surface.m_nurbsknot[1] = {0, 1};
    const double x[] = {0, 1, 1, 0, 0, 1, 1};
    const double y[] = {0, 0, 1, 1, 0, 0, 1};
    for (int u = 0; u < 7; ++u)
        for (int v = 0; v < 2; ++v)
            MINI_CHECK(surface.set_cv(u, v, Point(x[u], y[u], v)));
    MINI_CHECK(surface.is_valid());
    MINI_CHECK(surface.is_periodic(0));
    MINI_CHECK(surface.is_closed(0));
    constexpr double e = 0.125;
    nurbs_interval::SurfaceBounds extension;
    MINI_CHECK(nurbs_interval::surface_bounds(surface, {-e, -e, 0.5, 0.5}, {0, 0, 0}, extension, false, true));
    nurbs_interval::SurfaceBounds derivative_box;
    MINI_CHECK(nurbs_interval::surface_bounds(surface, {-e, -e + 1e-6, 0.5, 0.500001},
        {0, 0, 0}, derivative_box, true, true));
    const std::vector<Vector> wrapped = surface.evaluate(4 - e, 0.5, 1);
    const double extrapolated_x = (5 - 3 * e - 3 * e * e) / 6;
    MINI_CHECK(extension.point[0].lo <= extrapolated_x && extension.point[0].hi >= extrapolated_x);
    MINI_CHECK(extension.point[0].hi < wrapped[0][0]);
    MINI_CHECK(derivative_box.derivative[0][0].lo > wrapped[2][0]);
    MINI_CHECK(std::abs(wrapped[0][0] - extrapolated_x - e * e * e / 3) < 1e-12);
    MINI_CHECK(std::abs((0.5 + e) - wrapped[2][0] - e * e) < 1e-12);
    check_rejected_seam_chart(surface, {0, 0.5 + 1.0 / 36, 5.0 / 6, 1.0 / 6});
}

MINI_TEST("Surface Section Native", "Bounded Quarter Cylinder Plane Paired Parameters") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_quarter_cylinder_plane_parameter_failure.json");
    MINI_CHECK(input.good());
    const nlohmann::json fixture = nlohmann::json::parse(input);
    const NurbsSurface tube = NurbsSurface::jsonload(fixture["surface"]);
    NurbsSurface plane(3, false, 2, 2, 2, 2);
    plane.m_nurbsknot[0] = {0, 1};
    plane.m_nurbsknot[1] = {0, 1};
    for (int u = 0; u < 2; ++u)
        for (int v = 0; v < 2; ++v)
            MINI_CHECK(plane.set_cv(u, v, Point(0.5 + u, 0.5 + v, 0.1)));
    for (bool clipped : {false, true}) {
        for (int v = 0; v < 2; ++v)
            MINI_CHECK(plane.set_cv(1, v, Point(clipped ? 0.95 : 1.5, 0.5 + v, 0.1)));
        for (bool reverse : {false, true}) {
            const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections =
                reverse ? Intersection::surface_surface(plane, tube, 3e-5) :
                Intersection::surface_surface(tube, plane, 3e-5);
            MINI_CHECK(sections.size() == 1);
            const NurbsCurve& world = std::get<0>(sections[0]);
            const NurbsCurve& tube_uv = reverse ? std::get<2>(sections[0]) : std::get<1>(sections[0]);
            const NurbsCurve& plane_uv = reverse ? std::get<1>(sections[0]) : std::get<2>(sections[0]);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, tube_uv, tube, 7.5e-6) ==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, plane_uv, plane, 7.5e-6) ==
                curve_surface_intersection::ParameterMatch::Matched);
            if (clipped)
                MINI_CHECK(std::abs(std::max(world.point_at_start()[0], world.point_at_end()[0]) - 0.95) < 1e-10);
            for (int i = 0; i <= 32; ++i) {
                const double t = world.domain_start() + (world.domain_end() - world.domain_start()) * i / 32.0;
                const Point p = world.point_at(t);
                MINI_CHECK(p[0] >= 0.9 - 1e-12 && p[0] <= (clipped ? 0.95 : 1.0) + 1e-12);
                MINI_CHECK(p[1] >= 0.9 - 1e-12 && p[1] <= 1.0 + 1e-12);
                MINI_CHECK(std::abs(p[2] - 0.1) < 1e-12);
                MINI_CHECK(std::abs((p[0] - 1) * (p[0] - 1) + (p[1] - 1) * (p[1] - 1) - 0.01) < 1e-12);
            }
        }
    }
}

MINI_TEST("Surface Section Native", "Repeated Polished Plane Section Endpoints") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/inward_freeform_duplicate_plane_section.json");
    MINI_CHECK(input.good());
    const nlohmann::json fixture = nlohmann::json::parse(input);
    const NurbsSurface a = NurbsSurface::jsonload(fixture["first_surface"]);
    const NurbsSurface b = NurbsSurface::jsonload(fixture["second_surface"]);
    const double tolerance = fixture["tolerance"].get<double>();
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections =
        Intersection::surface_surface(a, b, tolerance);
    MINI_CHECK(sections.size() == 1);
    const NurbsCurve& world = std::get<0>(sections[0]);
    MINI_CHECK(world.point_at_start().distance(world.point_at_end()) > tolerance);
    for (int i = 0; i <= 512; ++i) {
        const double t = world.domain_start() + (world.domain_end() - world.domain_start()) * i / 512;
        const Point point = world.point_at(t);
        const Point ua = std::get<1>(sections[0]).point_at(t);
        const Point ub = std::get<2>(sections[0]).point_at(t);
        MINI_CHECK(point.distance(a.point_at(ua[0], ua[1])) <= tolerance);
        MINI_CHECK(point.distance(b.point_at(ub[0], ub[1])) <= tolerance);
    }
}

MINI_TEST("Surface Section Native", "Auxiliary Graph Identity Does Not Wrap Parameters") {
    const std::array<double, 4> lower = {0, 0, 0, 0};
    const std::array<double, 4> upper = {0.25, 1, 1, 1};
    const std::array<double, 4> inside = {0.125, 0.5, 0.5, 0.5};
    const std::array<double, 4> shifted = {1.125, 0.5, 0.5, 0.5};
    MINI_CHECK(ssi_diagnostics::raw_graph_contains(lower, upper, inside, inside));
    MINI_CHECK(!ssi_diagnostics::raw_graph_contains(lower, upper, shifted, shifted));
    std::array<double, 4> outside = inside;
    outside[0] = std::nextafter(0.25, 1.0);
    MINI_CHECK(!ssi_diagnostics::raw_graph_contains(lower, upper, outside, outside));
}

MINI_TEST("Surface Section Native", "Exact Double Boundary Graph Anchor") {
    const NurbsSurface a = Primitives::sphere_surface(0, 0, 0, 2);
    const NurbsSurface b = Primitives::cylinder_surface(1.3, 0, -3, 0.3, 6);
    const std::array<double, 4> lower = {0, 0.585784912109375, 3.99951171875, 0.29998779296875};
    const std::array<double, 4> upper = {6.103515625e-05, 0.5858154296875, 4, 0.3000030517578125};
    const std::array<double, 4> root = {0, 0.585786437626905, 4, 0.30000000000000004};
    const nlohmann::ordered_json witness = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(a, b, lower, upper, root, 1e-6));
    MINI_CHECK(witness["covered"].get<bool>());
    bool boundary_certified = false;
    for (const nlohmann::ordered_json& chart : witness["charts"])
        if (chart.value("anchor_kind", std::string()) == "exact_zero_plane_boundary" && chart["status"] == "covered") {
            const nlohmann::ordered_json& anchor = chart["boundary_anchor"];
            MINI_CHECK(anchor[0][0].get<double>() == 0 && anchor[0][1].get<double>() == 0);
            MINI_CHECK(anchor[2][0].get<double>() == 4 && anchor[2][1].get<double>() == 4);
            MINI_CHECK(anchor[1][0].get<double>() > 0 && anchor[1][1].get<double>() < 1);
            MINI_CHECK(anchor[3][0].get<double>() > 0 && anchor[3][1].get<double>() < 1);
            boundary_certified = true;
        }
    MINI_CHECK(boundary_certified);
    NurbsSurface perturbed = b;
    for (int j = 0; j < perturbed.cv_count(1); ++j)
        perturbed.cv(perturbed.cv_count(0) - 1, j)[1] = 1e-6;
    const nlohmann::ordered_json rejected = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(a, perturbed, lower, upper, root, 1e-6));
    for (const nlohmann::ordered_json& chart : rejected["charts"])
        MINI_CHECK(chart.value("anchor_kind", std::string()) != "exact_zero_plane_boundary");
    NurbsSurface transformed_a = a;
    NurbsSurface transformed_b = b;
    MINI_CHECK(transformed_a.reverse(0) && transformed_b.reverse(0));
    MINI_CHECK(transformed_a.set_domain(0, 8, 16) && transformed_a.set_domain(1, -3, 1));
    MINI_CHECK(transformed_b.set_domain(0, -12, -4) && transformed_b.set_domain(1, 20, 28));
    const std::array<double, 4> transformed_lower = {
        16 - 2 * upper[0], -3 + 2 * lower[1], -12, 20 + 8 * lower[3]};
    const std::array<double, 4> transformed_upper = {
        16, -3 + 2 * upper[1], -12 + 2 * (4 - lower[2]), 20 + 8 * upper[3]};
    const std::array<double, 4> transformed_root = {16, -3 + 2 * root[1], -12, 20 + 8 * root[3]};
    const nlohmann::ordered_json transformed = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(transformed_a, transformed_b,
            transformed_lower, transformed_upper, transformed_root, 1e-6));
    MINI_CHECK(transformed["covered"].get<bool>());
    bool transformed_boundary_certified = false;
    for (const nlohmann::ordered_json& chart : transformed["charts"])
        if (chart.value("anchor_kind", std::string()) == "exact_zero_plane_boundary" && chart["status"] == "covered") {
            const nlohmann::ordered_json& anchor = chart["boundary_anchor"];
            MINI_CHECK(anchor[0][0].get<double>() == 16 && anchor[0][1].get<double>() == 16);
            MINI_CHECK(anchor[2][0].get<double>() == -12 && anchor[2][1].get<double>() == -12);
            MINI_CHECK(anchor[1][0].get<double>() > -3 && anchor[1][1].get<double>() < 1);
            MINI_CHECK(anchor[3][0].get<double>() > 20 && anchor[3][1].get<double>() < 28);
            transformed_boundary_certified = true;
        }
    MINI_CHECK(transformed_boundary_certified);
}

MINI_TEST("Surface Section Native", "Boundary Graph Anchor Propagates To Near Seam Parameter") {
    const NurbsSurface a = Primitives::sphere_surface(0, 0, 0, 2);
    const NurbsSurface b = Primitives::cylinder_surface(1.3, 0, -3, 0.3, 6);
    const std::array<double, 4> lower = {0, 0.585786437626867, 0, 0.2999999999999791};
    const std::array<double, 4> upper = {5.639246822509432e-14, 0.5857864376269427,
        3.0173093107285425e-13, 0.30000000000002075};
    const std::array<double, 4> root = {1.1494257904886467e-15, 0.5857864376269049,
        7.101371421825311e-15, 0.29999999999999993};
    const nlohmann::ordered_json witness = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(a, b, lower, upper, root, 1e-6));
    MINI_CHECK(witness["covered"].get<bool>());
    MINI_CHECK(witness["root_box_proved"].get<bool>());
    const nlohmann::ordered_json& box = witness["root_box"];
    MINI_CHECK(box[0][0].get<double>() > 0 && box[0][1].get<double>() < 4);
    MINI_CHECK(box[2][0].get<double>() > 0 && box[2][1].get<double>() < 4);
    MINI_CHECK(box[1][0].get<double>() > 0 && box[1][1].get<double>() < 2);
    MINI_CHECK(box[3][0].get<double>() > 0 && box[3][1].get<double>() < 1);
}

MINI_TEST("Surface Section Native", "Reciprocal Joint Seam Event Identity") {
    const NurbsSurface a = Primitives::sphere_surface(0, 0, 0, 2);
    const NurbsSurface b = Primitives::cylinder_surface(1.3, 0, -3, 0.3, 6);
    const std::array<double, 4> first = {0.005896226337477924, 0.5856641578837348,
        0.03122309431729166, 0.2999462485864679};
    const std::array<double, 4> last = {-0.004381638118343928, 0.5857189731434406,
        -0.02324366999882702, 0.29997034322667815};
    const nlohmann::ordered_json original = nlohmann::ordered_json::parse(
        ssi_diagnostics::seam_step_diagnostic(a, b, first, last, 1e-6));
    MINI_CHECK(original["descriptors"].size() == 2);
    MINI_CHECK(original["cuts"].size() == 1);
    MINI_CHECK(original["joint_proofs"].size() == 1);
    MINI_CHECK(original["joint_proofs"][0]["incident_graphs"].get<int>() == 4);
    const std::array<double, 4> joint = original["output"][1].get<std::array<double, 4>>();
    MINI_CHECK(joint[0] == 0 && joint[2] == 0);
    MINI_CHECK(a.point_at(joint[0], joint[1]).distance(b.point_at(joint[2], joint[3])) <= 1e-6);
    NurbsSurface reversed_a = a;
    NurbsSurface reversed_b = b;
    MINI_CHECK(reversed_a.reverse(0) && reversed_b.reverse(0));
    MINI_CHECK(reversed_a.set_domain(0, 8, 16) && reversed_a.set_domain(1, -3, 1));
    MINI_CHECK(reversed_b.set_domain(0, -12, -4) && reversed_b.set_domain(1, 20, 28));
    const std::array<double, 4> transformed_first = {
        16 - 2 * first[0], -3 + 2 * first[1], -4 - 2 * first[2], 20 + 8 * first[3]};
    const std::array<double, 4> transformed_last = {
        16 - 2 * last[0], -3 + 2 * last[1], -4 - 2 * last[2], 20 + 8 * last[3]};
    const nlohmann::ordered_json transformed = nlohmann::ordered_json::parse(
        ssi_diagnostics::seam_step_diagnostic(reversed_a, reversed_b,
            transformed_first, transformed_last, 1e-6));
    MINI_CHECK(transformed["descriptors"].size() == 2);
    MINI_CHECK(transformed["cuts"].size() == 1);
    MINI_CHECK(transformed["joint_proofs"].size() == 1);
    const std::array<double, 4> transformed_joint = transformed["output"][1].get<std::array<double, 4>>();
    MINI_CHECK(transformed_joint[0] == 16 && transformed_joint[2] == -4);
    MINI_CHECK(reversed_a.point_at(transformed_joint[0], transformed_joint[1]).distance(
        reversed_b.point_at(transformed_joint[2], transformed_joint[3])) <= 1e-6);
    const NurbsSurface nearby_b = Primitives::cylinder_surface(1.3, 1e-8, -3, 0.3, 6);
    const nlohmann::ordered_json distinct = nlohmann::ordered_json::parse(
        ssi_diagnostics::seam_step_diagnostic(a, nearby_b, first, last, 1e-6));
    MINI_CHECK(distinct["descriptors"].size() == 2);
    MINI_CHECK(distinct["cuts"].size() == 2);
    MINI_CHECK(distinct["joint_proofs"].empty());
    const std::array<double, 4> separate_first = {-0.004062330104737367, 0.34110679344816053,
        -1.9864942997088835, 0.21133176454470626};
    const std::array<double, 4> separate_last = {0.011068202627609047, 0.3412710643698366,
        -2.0366590605382022, 0.21137636842352447};
    const nlohmann::ordered_json separate = nlohmann::ordered_json::parse(
        ssi_diagnostics::seam_step_diagnostic(a, b, separate_first, separate_last, 1e-6));
    MINI_CHECK(separate["descriptors"].size() == 1);
    MINI_CHECK(separate["cuts"].size() == 1);
    MINI_CHECK(separate["joint_proofs"].empty());
    MINI_CHECK(separate["output"][1][0].get<double>() == 0);
    const std::array<double, 4> separate_joint = separate["output"][1].get<std::array<double, 4>>();
    MINI_CHECK(separate_joint[2] > separate_last[2] && separate_joint[2] < separate_first[2]);
    MINI_CHECK(a.point_at(separate_joint[0], separate_joint[1]).distance(
        b.point_at(separate_joint[2] + 4, separate_joint[3])) <= 1e-6);
}

MINI_TEST("Surface Section Native", "Certified Boundary Trace Start And Exact Loop Closure") {
    const NurbsSurface a = Primitives::sphere_surface(0, 0, 0, 2);
    const NurbsSurface b = Primitives::cylinder_surface(1.3, 0, -3, 0.3, 6);
    const std::array<double, 4> lower = {0, 0.585784912109375, 3.99951171875, 0.29998779296875};
    const std::array<double, 4> upper = {6.103515625e-05, 0.5858154296875, 4, 0.3000030517578125};
    const std::array<double, 4> root = {9.31215743542365e-16, 0.5857864376269016, 4, 0.2999999999999979};
    const nlohmann::ordered_json certified = nlohmann::ordered_json::parse(
        ssi_diagnostics::seed_chart_diagnostic(a, b, lower, upper, root, 1e-6));
    MINI_CHECK(certified["covered"].get<bool>());
    MINI_CHECK(certified["boundary_start_proved"].get<bool>());
    const std::array<double, 4> start = certified["boundary_start"].get<std::array<double, 4>>();
    MINI_CHECK(start[0] == 0 && start[2] == 4);
    const std::array<double, 4> last = {0.003835990437281993, 0.5857347468545971,
        0.020362295393816135, 0.29997727700601756};
    const nlohmann::ordered_json loop = nlohmann::ordered_json::parse(
        ssi_diagnostics::loop_seam_diagnostic(a, b, {start, last}, 1e-6));
    const std::array<double, 4> end = loop["closed_quad"].back().get<std::array<double, 4>>();
    MINI_CHECK(end[0] == 0 && end[2] == 0);
    MINI_CHECK(end[1] == start[1] && end[3] == start[3]);
    MINI_CHECK(loop["closure"][0].get<double>() == 0);
    MINI_CHECK(loop["closure"][2].get<double>() == -4);
    MINI_CHECK(loop["cuts"].front().get<int>() == 0);
    MINI_CHECK(loop["pieces"].size() == 1);
    MINI_CHECK(loop["pieces"][0].size() == 3);
    NurbsSurface perturbed = b;
    for (int v = 0; v < perturbed.cv_count(1); ++v)
        perturbed.cv(perturbed.cv_count(0) - 1, v)[1] = 1e-14;
    MINI_CHECK(perturbed.is_closed(0));
    const nlohmann::ordered_json unsupported = nlohmann::ordered_json::parse(
        ssi_diagnostics::loop_seam_diagnostic(a, perturbed, {start, last}, 1e-6));
    const double old_endpoint = last[2] + ((start[2] - last[2]) - 4);
    MINI_CHECK(unsupported["closed_quad"].back()[2].get<double>() == old_endpoint);
    const std::array<double, 4> interior = {0.125, root[1], 0.25, root[3]};
    const std::array<double, 4> interior_last = {0.25, last[1], 0.375, last[3]};
    const nlohmann::ordered_json no_turn = nlohmann::ordered_json::parse(
        ssi_diagnostics::loop_seam_diagnostic(a, b, {interior, interior_last}, 1e-6));
    const std::array<double, 4> no_turn_end = no_turn["closed_quad"].back().get<std::array<double, 4>>();
    const std::array<double, 4> no_turn_shift = no_turn["closure"].get<std::array<double, 4>>();
    const std::array<double, 4> zero{};
    MINI_CHECK(no_turn_end == interior);
    MINI_CHECK(no_turn_shift == zero);
}

MINI_TEST("Surface Section Native", "Isolated Raw Boundary Root Image Ownership") {
    const NurbsSurface a = Primitives::sphere_surface(0, 0, 0, 2);
    const NurbsSurface b = Primitives::cylinder_surface(1.3, 0, -3, 0.3, 6);
    const std::array<ssi_diagnostics::Parameters, 3> source = {{
        {0, 0.585784912109375, 3.99951171875, 0.29998779296875},
        {6.103515625e-05, 0.5858154296875, 4, 0.3000030517578125},
        {0, 0.585786437626905, 4, 0.3}}};
    const std::array<ssi_diagnostics::Parameters, 3> destination = {{
        {3.99993896484375, 0.585784912109375, 3.99951171875, 0.29998779296875},
        {4, 0.5858154296875, 4, 0.3000030517578125},
        {4, 0.585786437626905, 4, 0.3}}};
    const nlohmann::ordered_json isolated = nlohmann::ordered_json::parse(
        ssi_diagnostics::boundary_point_image_diagnostic(a, b, source, destination, {2, 2}, 1e-6));
    MINI_CHECK(isolated["covered"][0].get<bool>() && isolated["covered"][1].get<bool>());
    MINI_CHECK(isolated["source_is_point"].get<bool>());
    MINI_CHECK(isolated["same_boundary_point"].get<bool>());
    std::array<ssi_diagnostics::Parameters, 3> interior_source = source;
    std::array<ssi_diagnostics::Parameters, 3> interior_destination = destination;
    for (std::array<ssi_diagnostics::Parameters, 3>* chart : {&interior_source, &interior_destination}) {
        (*chart)[0][2] = 0;
        (*chart)[1][2] = 0.00048828125;
        (*chart)[2][2] = 0;
    }
    const nlohmann::ordered_json interior = nlohmann::ordered_json::parse(
        ssi_diagnostics::boundary_point_image_diagnostic(a, b,
            interior_source, interior_destination, {2, 2}, 1e-6));
    MINI_CHECK(interior["covered"][0].get<bool>() && interior["covered"][1].get<bool>());
    MINI_CHECK(!interior["source_is_point"].get<bool>());
    MINI_CHECK(!interior["same_boundary_point"].get<bool>());
    NurbsSurface unequal = a;
    for (int v = 0; v < unequal.cv_count(1); ++v)
        unequal.cv(unequal.cv_count(0) - 1, v)[0] += 1e-14;
    MINI_CHECK(unequal.is_closed(0));
    const nlohmann::ordered_json unsupported = nlohmann::ordered_json::parse(
        ssi_diagnostics::boundary_point_image_diagnostic(unequal, b, source, destination, {2, 2}, 1e-6));
    MINI_CHECK(unsupported["covered"][0].get<bool>() && unsupported["covered"][1].get<bool>());
    MINI_CHECK(unsupported["source_is_point"].get<bool>());
    MINI_CHECK(!unsupported["same_boundary_point"].get<bool>());
}
