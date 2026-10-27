#include "curve_self_intersection.h"
#include "mini_test.h"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

NurbsCurve cubic() {
    NurbsCurve result = NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 0.1875, 0},
        {1, -0.625, 0}, {0, 0.5625, 0}});
    if (!result.set_domain(0, 1))
        throw std::runtime_error("Could not set self-intersection fixture domain");
    return result;
}

curve_self_intersection::Result run(
    const std::string& name,
    const NurbsCurve& curve,
    double tolerance = 1e-7,
    const curve_self_intersection::Options& options = {}
) {
    const auto result = curve_self_intersection::intersect(curve, tolerance, options);
    nlohmann::ordered_json data = {{"curve", curve.jsondump()}, {"tolerance", tolerance},
        {"complete", result.complete}, {"failure", result.failure}, {"cells", result.cells},
        {"points", nlohmann::ordered_json::array()}, {"endpoints", nlohmann::ordered_json::array()},
        {"overlaps", nlohmann::ordered_json::array()}};
    for (const auto& hit : result.points)
        data["points"].push_back({{"a", hit.a}, {"b", hit.b}, {"tangent", hit.tangent},
            {"point", {hit.point[0], hit.point[1], hit.point[2]}}});
    for (const auto& hit : result.endpoints)
        data["endpoints"].push_back({{"a", hit.a}, {"b", hit.b}});
    for (const auto& overlap : result.overlaps)
        data["overlaps"].push_back({overlap.a0, overlap.a1, overlap.b0, overlap.b1});
    const std::filesystem::path folder("build/native-geometry-tests/curve-self-intersection");
    std::filesystem::create_directories(folder);
    std::ofstream output(folder / (name + ".json"));
    if (!(output << data.dump(2)))
        throw std::runtime_error("Could not preserve self-intersection result");
    return result;
}

void complete(const curve_self_intersection::Result& result) {
    if (!result.complete)
        throw std::runtime_error(result.failure + "; cells=" + std::to_string(result.cells));
}

void crossing(
    const NurbsCurve& curve,
    const curve_self_intersection::Result& result,
    double a,
    double b,
    double tolerance
) {
    complete(result);
    MINI_CHECK(result.points.size() == 1);
    MINI_CHECK(result.endpoints.empty());
    MINI_CHECK(result.overlaps.empty());
    const auto& hit = result.points.front();
    MINI_CHECK(hit.a < hit.b);
    MINI_CHECK(std::abs(hit.a - a) <= (curve.domain_end() - curve.domain_start()) * 1e-9);
    MINI_CHECK(std::abs(hit.b - b) <= (curve.domain_end() - curve.domain_start()) * 1e-9);
    MINI_CHECK(curve.point_at(hit.a).distance(curve.point_at(hit.b)) <= tolerance);
    MINI_CHECK(!hit.tangent);
}

}

MINI_TEST("Curve Self Intersection", "Single Open Cubic Crossing") {
    const auto curve = cubic();
    MINI_CHECK(curve.get_span_vector().size() == 2);
    MINI_CHECK(!curve.is_closed());
    const auto result = run("single_open_cubic", curve);
    crossing(curve, result, 0.25, 0.75, 1e-7);
    MINI_CHECK(result.points.front().point.distance({0.5625, 0, 0}) <= 1e-7);
}

MINI_TEST("Curve Self Intersection", "Affine Domains And Scale") {
    int index = 0;
    for (double scale : {0.001, 1.0, 1000.0}) {
        const Xform transform = Xform::translation(scale * 13, scale * -7, scale * 3) *
            Xform::scale_xyz(scale, scale, scale);
        NurbsCurve curve = cubic().transformed(transform);
        MINI_CHECK(curve.set_domain(-17.0, -17.0 + scale));
        const auto result = run("affine_" + std::to_string(index++), curve, scale * 1e-7);
        crossing(curve, result, -17 + scale * 0.25, -17 + scale * 0.75, scale * 1e-7);
    }
}

MINI_TEST("Curve Self Intersection", "Rational Compressed Parameter Branches") {
    for (double factor : {2.0, 1e8}) {
        NurbsCurve curve = cubic();
        MINI_CHECK(curve.to_rational());
        for (int i = 0; i < curve.cv_count(); ++i) {
            const Point point = curve.get_cv(i);
            const double weight = std::pow(factor, i);
            MINI_CHECK(curve.set_cv_4d(i, point[0] * weight, point[1] * weight, point[2] * weight, weight));
        }
        const auto result = run("rational_" + std::to_string(factor), curve);
        crossing(curve, result, 1.0 / (3 * factor + 1), 3.0 / (factor + 3), 1e-7);
        const auto& hit = result.points.front();
        const double a = factor * hit.a / (1 + (factor - 1) * hit.a);
        const double b = factor * hit.b / (1 + (factor - 1) * hit.b);
        MINI_CHECK(std::abs(a - 0.25) < 1e-7);
        MINI_CHECK(std::abs(b - 0.75) < 1e-7);
    }
}

MINI_TEST("Curve Self Intersection", "Closed Seam And Ordinary Joins") {
    NurbsCurve square = NurbsCurve::create(false, 1, {{0, 0, 0}, {1, 0, 0},
        {1, 1, 0}, {0, 1, 0}, {0, 0, 0}});
    MINI_CHECK(square.set_domain(0, 1));
    const auto result = run("square_seam", square);
    complete(result);
    MINI_CHECK(result.points.empty());
    MINI_CHECK(result.endpoints.size() == 1);
    MINI_CHECK(result.endpoints.front().a == 0);
    MINI_CHECK(result.endpoints.front().b == 1);
    MINI_CHECK(result.overlaps.empty());
    const NurbsCurve line = NurbsCurve::create(false, 1, {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}});
    const auto ordinary = run("ordinary_joins", line);
    complete(ordinary);
    MINI_CHECK(ordinary.points.empty() && ordinary.endpoints.empty() && ordinary.overlaps.empty());
}

MINI_TEST("Curve Self Intersection", "Distinct Crossings Under Parameter Compression") {
    const std::array<double, 6> x = {0.25, -1, 1, 0, 0, 0};
    std::array<double, 6> y{};
    const auto choose = [](int n, int k) {
        double value = 1;
        for (int i = 1; i <= k; ++i)
            value *= static_cast<double>(n + 1 - i) / i;
        return value;
    };
    const double a = 0.125;
    const double b = 0.375;
    for (int k = 0; k <= 5; ++k) {
        y[k] = choose(5, k) * std::pow(-0.5, 5 - k);
        if (k <= 3)
            y[k] -= (a * a + b * b) * choose(3, k) * std::pow(-0.5, 3 - k);
        if (k <= 1)
            y[k] += a * a * b * b * std::pow(-0.5, 1 - k);
    }
    std::vector<Point> poles;
    for (int i = 0; i <= 5; ++i) {
        Point pole(0, 0, 0);
        for (int k = 0; k <= i; ++k) {
            pole[0] += choose(i, k) * x[k] / choose(5, k);
            pole[1] += choose(i, k) * y[k] / choose(5, k);
        }
        poles.push_back(pole);
    }
    for (double factor : {1.0, 1e8}) {
        NurbsCurve curve = NurbsCurve::create(false, 5, poles);
        MINI_CHECK(curve.set_domain(0, 1));
        MINI_CHECK(curve.to_rational());
        for (int i = 0; i < curve.cv_count(); ++i) {
            const double weight = std::pow(factor, i);
            MINI_CHECK(curve.set_cv_4d(i, poles[i][0] * weight, poles[i][1] * weight, 0, weight));
        }
        const auto result = run("two_crossings_" + std::to_string(factor), curve, 1e-9);
        complete(result);
        MINI_CHECK(result.points.size() == 2);
        MINI_CHECK(result.endpoints.empty() && result.overlaps.empty());
        for (int i = 0; i < 2; ++i) {
            const auto& hit = result.points[i];
            const double first = factor * hit.a / (1 + (factor - 1) * hit.a);
            const double second = factor * hit.b / (1 + (factor - 1) * hit.b);
            MINI_CHECK(std::abs(first - (i == 0 ? 0.125 : 0.375)) < 1e-7);
            MINI_CHECK(std::abs(second - (i == 0 ? 0.875 : 0.625)) < 1e-7);
            MINI_CHECK(curve.point_at(hit.a).distance(curve.point_at(hit.b)) <= 1e-9);
        }
    }
}

MINI_TEST("Curve Self Intersection", "Skew Curve Has No Crossing") {
    NurbsCurve curve = cubic();
    for (int i = 0; i < curve.cv_count(); ++i) {
        Point point = curve.get_cv(i);
        point[2] = i / 3.0;
        MINI_CHECK(curve.set_cv(i, point));
    }
    const auto result = run("skew", curve);
    complete(result);
    MINI_CHECK(result.points.empty() && result.endpoints.empty() && result.overlaps.empty());
}

MINI_TEST("Curve Self Intersection", "Singular And Budget Are Explicit") {
    const NurbsCurve retraced = NurbsCurve::create(false, 1, {{0, 0, 0}, {1, 0, 0}, {0, 0, 0}});
    const auto singular = run("retraced", retraced, 1e-7, {4096, 32});
    MINI_CHECK(!singular.complete);
    MINI_CHECK(!singular.failure.empty());
    const auto limited = run("limited", cubic(), 1e-7, {1, 0});
    MINI_CHECK(!limited.complete);
    MINI_CHECK(!limited.failure.empty());
}

MINI_TEST("Curve Self Intersection", "Stationary Endpoint Monotone Curve") {
    NurbsCurve curve = NurbsCurve::create(false, 4,
        {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {3, 0, 0}, {6, 1, 0}});
    MINI_CHECK(curve.set_domain(0, 1));
    MINI_CHECK(curve.evaluate(0, 1)[1].magnitude() == 0.0);
    const auto result = run("stationary_endpoint_monotone", curve);
    complete(result);
    MINI_CHECK(result.points.empty() && result.endpoints.empty() && result.overlaps.empty());
}

MINI_TEST("Curve Self Intersection", "Stationary Endpoint Reversed Representations") {
    for (const std::string name : {"native_split_stationary_even_reversed_affine.input",
        "native_split_stationary_odd_reverse.input"}) {
        std::ifstream input("tests/fixtures/" + name + ".json");
        nlohmann::json fixture;
        input >> fixture;
        int count = 0;
        for (const auto& cut : fixture.at("cuts")) {
            if (cut.at("pcurve").at("order").get<int>() <= 2)
                continue;
            const NurbsCurve original = NurbsCurve::jsonload(cut.at("pcurve"));
            for (double weight : {1.0, 0.125, 1024.0}) {
                NurbsCurve curve = original;
                if (weight != 1.0) {
                    MINI_CHECK(curve.to_rational());
                    for (int i = 0; i < curve.cv_count(); ++i) {
                        const Point point = original.get_cv(i);
                        MINI_CHECK(curve.set_cv_4d(i, point[0] * weight, point[1] * weight,
                            point[2] * weight, weight));
                    }
                }
                const auto result = run(name + "_" + std::to_string(weight), curve, fixture.at("tolerance"));
                complete(result);
                MINI_CHECK(result.points.empty() && result.endpoints.empty() && result.overlaps.empty());
            }
            ++count;
        }
        MINI_CHECK(count == 1);
    }
}

MINI_TEST("Curve Self Intersection", "Stationary Derivative Across Original Knots") {
    NurbsCurve original = NurbsCurve::create(false, 3, {{0, 0, 0}, {0, 0, 0}, {1, 1, 0},
        {2, -1, 0}, {3, 2, 0}, {4, 0, 0}, {4, 0, 0}});
    MINI_CHECK(original.set_domain(-13, 29));
    MINI_CHECK(original.get_span_vector().size() > 2);
    for (bool reversed : {false, true}) {
        NurbsCurve curve = original;
        if (reversed)
            MINI_CHECK(curve.reverse());
        const auto result = run("stationary_original_knots_" + std::to_string(reversed), curve);
        complete(result);
        MINI_CHECK(result.points.empty() && result.endpoints.empty() && result.overlaps.empty());
    }
}
