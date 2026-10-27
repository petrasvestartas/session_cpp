#include "brep_measure.h"
#include "file_step.h"
#include "mini_test.h"
#include "xform.h"
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

NurbsSurface dense_plane(const NurbsSurface& source) {
    NurbsSurface result(3, false, 4, 4, 131, 131);
    for (int dir = 0; dir < 2; ++dir) {
        const std::pair<double, double> domain = source.domain(dir);
        std::vector<double>& knots = result.m_nurbsknot[dir];
        knots.assign(3, domain.first);
        for (int i = 1; i < 128; ++i)
            knots.push_back(domain.first + (domain.second - domain.first) * std::pow(i / 128.0, dir ? 1.3 : 1.7));
        knots.insert(knots.end(), 3, domain.second);
    }
    for (int i = 0; i < 131; ++i)
        for (int j = 0; j < 131; ++j) {
            const std::vector<double>& u = result.m_nurbsknot[0];
            const std::vector<double>& v = result.m_nurbsknot[1];
            const Point point = source.point_at((u[i] + u[i + 1] + u[i + 2]) / 3,
                (v[j] + v[j + 1] + v[j + 2]) / 3);
            MINI_CHECK(result.set_cv(i, j, point));
        }
    return result;
}

NurbsSurface rational_plane(const NurbsSurface& source) {
    NurbsSurface result(3, true, 3, 2, 3, 5);
    const std::pair<double, double> u = source.domain(0);
    const std::pair<double, double> v = source.domain(1);
    result.m_nurbsknot[0] = {u.first, u.first, u.second, u.second};
    for (int j = 0; j < 5; ++j) {
        const double parameter = v.first + (v.second - v.first) * std::array<double, 5>{0, 0.17, 0.43, 0.81, 1}[j];
        result.m_nurbsknot[1][j] = parameter;
        const Point a = source.point_at(u.first, parameter);
        const Point b = source.point_at(u.second, parameter);
        const std::array<Point, 3> controls = {a, a + (b - a) * 0.25, b};
        for (int i = 0; i < 3; ++i)
            MINI_CHECK(result.set_cv_4d(i, j, controls[i][0] * (i + 1), controls[i][1] * (i + 1), controls[i][2] * (i + 1), i + 1));
    }
    MINI_CHECK(result.increase_degree(0, 6));
    return result;
}

double measured_fixture(const BRep& source, const std::string& name) {
    const std::filesystem::path folder = "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    source.file_json_dump((folder / (name + ".brep.json")).string());
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    nlohmann::ordered_json record = {{"stage", "volume"}, {"tolerance", 1e-8}};
    const auto save = [&]() {
        record["elapsed_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::ofstream(folder / (name + ".measure.json")) << record.dump(2);
    };
    save();
    try {
        const double value = brep_measure::volume(source, 1e-8);
        record["volume"] = value;
        record["stage"] = "complete";
        save();
        return value;
    } catch (const std::exception& error) {
        record["failure"] = error.what();
        record["stage"] = "failed";
        save();
        throw;
    }
}

}

MINI_TEST("BRep Native Measure", "Analytical Primitive Volumes") {
    const double pi = Tolerance::PI;
    const std::vector<std::pair<BRep, double>> cases = {
        {BRep::create_box(2, 3, 4), 24}, {BRep::create_sphere(2), 32 * pi / 3},
        {BRep::create_cylinder(2, 3), 12 * pi}, {BRep::create_cone(2, 3), 4 * pi},
        {BRep::create_torus(3, 0.5), 1.5 * pi * pi}, {BRep::create_block_with_hole(4, 4, 2, 1), 32 - 2 * pi}
    };
    for (const auto& [shape, expected] : cases) {
        const double measured = shape.volume_exact(1e-8);
        MINI_CHECK(std::abs(measured - expected) <= std::abs(expected) * 1e-8);
    }
}

MINI_TEST("BRep Native Measure", "Translation Scale And Inactive Geometry") {
    const BRep original = BRep::create_sphere(2);
    const double expected = 32 * Tolerance::PI / 3;
    for (double scale : {0.001, 1.0, 1000.0}) {
        BRep shape = original.transformed(Xform::scale_xyz(scale, scale, scale));
        shape.transform(Xform::translation(1e5, -2e5, 3e5));
        const double measured = shape.volume_exact(1e-7);
        MINI_CHECK(std::abs(measured / (scale * scale * scale) - expected) < expected * 1e-6);
    }
    BRep extra = original;
    extra.add_surface(BRep::create_box(100, 100, 100).m_surfaces[0]);
    MINI_CHECK(std::abs(brep_measure::volume(extra, 1e-8) - expected) < expected * 1e-8);
}

MINI_TEST("BRep Native Measure", "Original Rhino Chair Volumes") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const auto chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    MINI_CHECK(chairs.size() == 2);
    const double expected[] = {2944.704344237949, 2944.7043442379395};
    nlohmann::ordered_json values = nlohmann::ordered_json::array();
    for (size_t i = 0; i < chairs.size(); ++i) {
        const double measured = chairs[i].volume_exact(1e-8);
        MINI_CHECK(std::abs(measured - expected[i]) <= expected[i] * 1e-7);
        values.push_back({{"chair", i}, {"native_volume", measured}, {"occt_direct_step_volume", expected[i]}});
    }
    std::ofstream(root / "build/native-geometry-tests/chair_native_volumes.json") << values.dump(2);
}

MINI_TEST("BRep Native Measure", "Nonuniform Dense Polynomial Support") {
    BRep source = BRep::create_box(2, 3, 4);
    const NurbsSurface original = source.m_surfaces[0];
    source.m_surfaces[0] = dense_plane(original);
    for (int i = 0; i <= 9; ++i)
        for (int j = 0; j <= 11; ++j) {
            const double u = original.domain(0).first + (original.domain(0).second - original.domain(0).first) * i / 9.0;
            const double v = original.domain(1).first + (original.domain(1).second - original.domain(1).first) * j / 11.0;
            MINI_CHECK(source.m_surfaces[0].point_at(u, v).distance(original.point_at(u, v)) < 1e-12);
        }
    MINI_CHECK(std::abs(measured_fixture(source, "native_measure_dense_polynomial") - 24) < 1e-7);
}

MINI_TEST("BRep Native Measure", "Exact Rational Walls And Weighted Boundary Parameters") {
    BRep source = BRep::create_box(2, 3, 4);
    for (NurbsSurface& surface : source.m_surfaces)
        surface = rational_plane(surface);
    for (std::vector<NurbsCurve>* curves : {&source.m_curves_3d, &source.m_curves_2d})
        for (NurbsCurve& curve : *curves) {
            MINI_CHECK(curve.cv_count() == 2 && curve.to_rational());
            for (int d = 0; d < curve.cv_size(); ++d)
                curve.cv(1)[d] *= 4;
        }
    MINI_CHECK(std::abs(measured_fixture(source, "native_measure_rational_walls") - 24) < 1e-7);
}

MINI_TEST("BRep Native Measure", "Preserved Rational Thickening Span Growth") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_thicken_rational_saddle_result.brep.json");
    MINI_CHECK(input.good());
    const BRep source = BRep::jsonload(nlohmann::json::parse(input));
    MINI_CHECK(source.m_surfaces[1].cv_count(0) == 131 && source.m_surfaces[1].cv_count(1) == 131);
    for (size_t i = 2; i < source.m_surfaces.size(); ++i)
        MINI_CHECK(source.m_surfaces[i].degree(0) == 6 && source.m_surfaces[i].is_rational());
    MINI_CHECK(std::abs(measured_fixture(source, "native_measure_rational_thickening") - 0.4052333254726519) < 3e-5);
}

MINI_TEST("BRep Native Measure", "Polynomial Rectangle Orientation And Partial Trim") {
    BRep source = BRep::create_box(2, 3, 4);
    source.m_solids[0].shells[0].orientation = BRepOrientation::Reversed;
    MINI_CHECK(std::abs(measured_fixture(source, "native_measure_reversed_rectangle") + 24) < 1e-7);
    source.m_solids[0].shells[0].orientation = BRepOrientation::Forward;
    const NurbsSurface original = source.m_surfaces[0];
    const std::pair<double, double> u = original.domain(0);
    const std::pair<double, double> v = original.domain(1);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(source.m_surfaces[0].set_cv(i, j, original.point_at(
                u.first + (2 * i - 0.5) * (u.second - u.first), v.first + (2 * j - 0.5) * (v.second - v.first))));
    for (const BRepEdge& edge : source.m_edges)
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            if (relation.surface_index == 0) {
                NurbsCurve& pc = source.m_curves_2d[relation.curve_2d_index];
                for (int i = 0; i < pc.cv_count(); ++i) {
                    const Point point = pc.get_cv(i);
                    MINI_CHECK(pc.set_cv(i, {u.first + 0.25 * (u.second - u.first) + 0.5 * (point[0] - u.first),
                        v.first + 0.25 * (v.second - v.first) + 0.5 * (point[1] - v.first), 0}));
                }
            }
    MINI_CHECK(std::abs(measured_fixture(source, "native_measure_partial_rectangle") - 24) < 1e-7);
}
