#include "brep_classify.h"
#include "mini_test.h"
#include "nurbs_interval.h"
#include "session.h"
#include "split.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <set>

namespace session_cpp {
namespace split_direction_tests {

constexpr double Tolerance = 1e-7;
const std::filesystem::path Folder = "build/native-geometry-tests/viewer";

struct Fixture {
    double radius;
    std::vector<NurbsCurve> curves;
    std::vector<Point> probes;
};

BRep rectangle(double radius) {
    NurbsSurface surface(3, false, 2, 2, 2, 2);
    surface.m_nurbsknot[0] = {-radius, radius};
    surface.m_nurbsknot[1] = {-radius, radius};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(surface.set_cv(i, j, {(2 * i - 1) * radius, (2 * j - 1) * radius, 0}));
    BRep result;
    result.add_surface(surface);
    const std::array<Point, 4> points = {Point(-radius, -radius, 0), Point(radius, -radius, 0),
        Point(radius, radius, 0), Point(-radius, radius, 0)};
    for (const Point& point : points)
        result.add_vertex(point, Tolerance);
    std::vector<BRepRef> edges;
    for (int i = 0; i < 4; ++i) {
        const NurbsCurve curve = NurbsCurve::create(false, 1, {points[i], points[(i + 1) % 4]});
        const int edge = result.add_edge(result.add_curve_3d(curve), i, (i + 1) % 4, Tolerance);
        result.add_pcurve(edge, 0, result.add_curve_2d(curve));
        edges.push_back({edge, BRepOrientation::Forward});
    }
    result.add_face(0, {{result.add_wire(edges), BRepOrientation::Forward}}, Tolerance);
    result.add_shell({{0, BRepOrientation::Forward}});
    MINI_CHECK(result.is_valid());
    return result;
}

Fixture even() {
    return {3, {
        NurbsCurve::create(false, 3, {{0, 0, 0}, {0, 0, 0}, {0, 1, 0}, {3, 3, 0}}),
        NurbsCurve::create(false, 1, {{0, -3, 0}, {0, 0, 0}}),
        NurbsCurve::create(false, 1, {{-3, 0, 0}, {3, 0, 0}})
    }, {{-1, 1, 0}, {2, 1, 0}, {-1, -1, 0}, {1, -1, 0}}};
}

Fixture odd() {
    Fixture result{4, {
        NurbsCurve::create(false, 4, {{4, -4, 0}, {0, -1, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}),
        NurbsCurve::create(false, 1, {{0, 4, 0}, {0, 0, 0}}),
        NurbsCurve::create(false, 1, {{-4, 0, 0}, {4, 0, 0}})
    }, {{-1, 1, 0}, {1, 1, 0}, {-1, -1, 0}, {3, -1, 0}}};
    MINI_CHECK(result.curves[0].set_domain(-1, 0));
    return result;
}

Fixture tangent() {
    return {2, {
        NurbsCurve::create(false, 2, {{0, 0, 0}, {1, 0, 0}, {2, 2, 0}}),
        NurbsCurve::create(false, 2, {{0, 0, 0}, {1, 0, 0}, {2, -2, 0}}),
        NurbsCurve::create(false, 1, {{-2, 0, 0}, {0, 0, 0}})
    }, {{0, 1.5, 0}, {1, 0, 0}, {0, -1.5, 0}}};
}

void affine_reversed(Fixture& fixture, unsigned int reversed) {
    for (size_t i = 0; i < fixture.curves.size(); ++i) {
        if (reversed & (1u << i))
            MINI_CHECK(fixture.curves[i].reverse());
        MINI_CHECK(fixture.curves[i].set_domain(3, 11));
    }
}

void save_geometry(const BRep& geometry, const std::string& name, const Color& color) {
    BRep copy = geometry;
    copy.surfacecolor = color;
    copy.name = name;
    copy.file_json_dump((Folder / (name + ".brep.json")).string());
    Session document(name);
    document.add_brep(copy);
    document.pb_dump((Folder / (name + ".pb")).string());
}

double region_area(const split::FaceSplit& partition, int face) {
    const std::array<double, 4> nodes = {-0.8611363115940526, -0.3399810435848563,
        0.3399810435848563, 0.8611363115940526};
    const std::array<double, 4> weights = {0.3478548451374538, 0.6521451548625461,
        0.6521451548625461, 0.3478548451374538};
    double area = 0;
    for (const BRepRef& wire : partition.brep.m_faces[face].wires)
        for (const BRepRef& edge : partition.brep.wire_edges(wire)) {
            const int index = partition.brep.pcurve_index(edge.index, face, edge.orientation);
            MINI_CHECK(index >= 0);
            const NurbsCurve& curve = partition.brep.m_curves_2d[index];
            MINI_CHECK(!curve.is_rational() && curve.degree() <= 4);
            const double sign = edge.orientation == BRepOrientation::Reversed ? -1 : 1;
            const std::vector<double> spans = curve.get_span_vector();
            for (size_t i = 1; i < spans.size(); ++i) {
                const double center = (spans[i - 1] + spans[i]) * 0.5;
                const double half = (spans[i] - spans[i - 1]) * 0.5;
                for (size_t j = 0; j < nodes.size(); ++j) {
                    const std::vector<Vector> values = curve.evaluate(center + half * nodes[j], 1);
                    area += sign * half * weights[j] * (values[0][0] * values[1][1] - values[0][1] * values[1][0]) * 0.5;
                }
            }
        }
    return area;
}

void check_regions(
    const split::FaceSplit& result,
    const Fixture& fixture,
    nlohmann::ordered_json& report,
    const std::function<void()>& save
) {
    MINI_CHECK(result.brep.is_valid());
    report["faces"] = result.faces.size();
    save();
    MINI_CHECK(result.faces.size() == fixture.probes.size());
    std::set<int> occupied;
    report["probes"] = nlohmann::ordered_json::array();
    for (const Point& query : fixture.probes) {
        nlohmann::ordered_json probe = {{"point", {query[0], query[1], query[2]}},
            {"states", nlohmann::ordered_json::array()}};
        std::vector<int> inside;
        bool resolved = true;
        for (int face : result.faces) {
            const brep_classify::State state = brep_classify::face_uv(result.brep, face, query, Tolerance);
            probe["states"].push_back({{"face", face}, {"state", static_cast<int>(state)}});
            if (state == brep_classify::State::Inside)
                inside.push_back(face);
            resolved = resolved && (state == brep_classify::State::Inside || state == brep_classify::State::Outside);
        }
        report["probes"].push_back(probe);
        save();
        MINI_CHECK(resolved && inside.size() == 1);
        MINI_CHECK(occupied.insert(inside[0]).second);
    }
    MINI_CHECK(occupied.size() == result.faces.size());
    double total_area = 0;
    for (int face : result.faces) {
        MINI_CHECK(result.brep.m_faces[face].wires.size() == 1);
        const double area = region_area(result, face);
        MINI_CHECK(area > 0);
        total_area += area;
    }
    report["area"] = total_area;
    save();
    MINI_CHECK(std::abs(total_area - 4 * fixture.radius * fixture.radius) <= 1e-7);
    std::set<int> sections;
    for (const split::EdgePiece& piece : result.edges)
        if (piece.section >= 0)
            sections.insert(piece.section);
    MINI_CHECK(sections == std::set<int>({0, 1, 2}));
}

void run(
    const Fixture& fixture,
    const std::array<int, 3>& order,
    const std::string& name
) {
    std::filesystem::create_directories(Folder);
    const BRep source = rectangle(fixture.radius);
    save_geometry(source, name + "_source", Color(75 / 255.f, 125 / 255.f, 170 / 255.f));
    std::vector<split::FaceCut> cuts;
    nlohmann::ordered_json input = {{"source", source.jsondump()}, {"order", order},
        {"tolerance", Tolerance}, {"cuts", nlohmann::ordered_json::array()}};
    for (int index : order) {
        cuts.push_back({fixture.curves[index], fixture.curves[index], index});
        input["cuts"].push_back({{"section", index}, {"world", fixture.curves[index].jsondump()},
            {"pcurve", fixture.curves[index].jsondump()}});
    }
    std::ofstream(Folder / (name + ".input.json")) << input.dump(2);
    nlohmann::ordered_json report = {{"passed", false}, {"stage", "partition"},
        {"source_scene", name + "_source.pb"}, {"result_scene", name + "_result.pb"},
        {"expected_faces", fixture.probes.size()}, {"expected_area", 4 * fixture.radius * fixture.radius}};
    const auto started = std::chrono::steady_clock::now();
    const auto save = [&]() {
        report["wall_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::ofstream(Folder / (name + ".assertions.json")) << report.dump(2);
    };
    save();
    try {
        const split::FaceSplit result = split::split_brep_face_by_pcurves(source, 0, cuts, Tolerance);
        save_geometry(result.brep, name + "_result", Color(180 / 255.f, 110 / 255.f, 155 / 255.f));
        report["stage"] = "region validation";
        check_regions(result, fixture, report, save);
        report["passed"] = true;
        report["stage"] = "complete";
        save();
    } catch (const split::CurveError& error) {
        report["failure"] = error.what();
        report["parameter_failure"] = {{"curve", error.curve.jsondump()}, {"pcurve", error.pcurve.jsondump()},
            {"surface", error.surface.jsondump()}, {"parameter", error.parameter},
            {"distance", error.distance}, {"tolerance", error.tolerance}};
        save();
        throw;
    } catch (const std::exception& error) {
        report["failure"] = error.what();
        save();
        throw;
    }
}

}

MINI_TEST("Split Direction", "Stationary Even Endpoint Regions") {
    split_direction_tests::run(split_direction_tests::even(), {0, 1, 2}, "native_split_stationary_even");
}

MINI_TEST("Split Direction", "Stationary Even Reversed Affine Regions") {
    auto fixture = split_direction_tests::even();
    split_direction_tests::affine_reversed(fixture, 1);
    split_direction_tests::run(fixture, {2, 1, 0}, "native_split_stationary_even_reversed_affine");
}

MINI_TEST("Split Direction", "Stationary Odd Reverse Endpoint Regions") {
    split_direction_tests::run(split_direction_tests::odd(), {0, 1, 2}, "native_split_stationary_odd_reverse");
}

MINI_TEST("Split Direction", "Stationary Odd Reversed Affine Regions") {
    auto fixture = split_direction_tests::odd();
    split_direction_tests::affine_reversed(fixture, 1);
    split_direction_tests::run(fixture, {2, 1, 0}, "native_split_stationary_odd_reversed_affine");
}

MINI_TEST("Split Direction", "Tangent Sector Input Permutations") {
    std::array<int, 3> order = {0, 1, 2};
    do {
        const std::string suffix = std::to_string(order[0]) + std::to_string(order[1]) + std::to_string(order[2]);
        split_direction_tests::run(split_direction_tests::tangent(), order, "native_split_tangent_sectors_" + suffix);
    } while (std::next_permutation(order.begin(), order.end()));
}

MINI_TEST("Split Direction", "Tangent Sector Reversed Affine Regions") {
    auto fixture = split_direction_tests::tangent();
    split_direction_tests::affine_reversed(fixture, 5);
    split_direction_tests::run(fixture, {1, 2, 0}, "native_split_tangent_sectors_reversed_affine");
}

MINI_TEST("Split Direction", "Interval Exact Negation And Cancellation") {
    using nurbs_interval::Interval;
    for (double x : {4.0, -4.0, std::numeric_limits<double>::denorm_min(),
            std::numeric_limits<double>::max()}) {
        const Interval zero = Interval(x) + Interval(-x);
        MINI_CHECK(zero.lo == 0.0 && zero.hi == 0.0);
        const Interval negative = Interval(x) * Interval(-1.0);
        MINI_CHECK(negative.lo == -x && negative.hi == -x);
    }
    const Interval a = Interval(-1.0) * Interval(-4.0, 7.0);
    const Interval b = Interval(-4.0, 7.0) * Interval(-1.0);
    MINI_CHECK(a.lo == -7.0 && a.hi == 4.0);
    MINI_CHECK(b.lo == a.lo && b.hi == a.hi);
    const Interval noninverse = Interval(4.0) + Interval(-3.0);
    MINI_CHECK(noninverse.lo < 1.0 && noninverse.hi > 1.0);
    const Interval uncertain = Interval(4.0, 5.0) + Interval(-5.0, -4.0);
    MINI_CHECK(uncertain.lo < -1.0 && uncertain.hi > 1.0);
    for (double z : {0.0, -0.0}) {
        const Interval zero = Interval(z) * Interval(-1.0);
        MINI_CHECK(zero.lo == 0.0 && zero.hi == 0.0);
        MINI_CHECK(!std::signbit(zero.lo) && !std::signbit(zero.hi));
    }
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (Interval invalid : {Interval(inf), Interval(-inf), Interval(nan), Interval(2.0, 1.0)}) {
        const Interval a = invalid * Interval(-1.0);
        const Interval b = Interval(-1.0) * invalid;
        MINI_CHECK(a.lo == -inf && a.hi == inf);
        MINI_CHECK(b.lo == -inf && b.hi == inf);
    }
    const Interval infinite_sum = Interval(inf) + Interval(-inf);
    MINI_CHECK(infinite_sum.lo == -inf && infinite_sum.hi == inf);
}

MINI_TEST("Split Direction", "Rotated Tangent Sector Regions") {
    const split_direction_tests::Fixture fixture{4, {
        NurbsCurve::create(false, 2, {{0, 0, 0}, {1, 1, 0}, {0, 4, 0}}),
        NurbsCurve::create(false, 2, {{0, 0, 0}, {1, 1, 0}, {4, 0, 0}}),
        NurbsCurve::create(false, 1, {{-4, -4, 0}, {0, 0, 0}})
    }, {{-1.5, 1.5, 0}, {1, 1, 0}, {1.5, -1.5, 0}}};
    split_direction_tests::run(fixture, {0, 1, 2}, "native_split_rotated_tangent_sectors");
}

}
