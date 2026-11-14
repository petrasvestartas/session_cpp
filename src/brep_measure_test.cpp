#include "brep_classify.h"
#include "brep_assemble.h"
#include "tolerance.h"
#include "brep_measure.h"
#include "brep_measure_polynomial.h"
#include "brep_measure_plane.h"
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

MINI_TEST("BRep Native Measure", "Passes Share Sums Of One Support Only") {
    const double pi = Tolerance::PI;
    BRep pair = BRep::create_sphere(1.0).boolean_union(BRep::create_sphere(2.0).transformed(Xform::translation(5, 0, 0)));
    MINI_CHECK(pair.is_valid() && pair.is_solid() && pair.m_surfaces.size() == 2);
    for (double tolerance : {1e-6, 1e-8, 1e-10})
        MINI_CHECK(std::abs(pair.volume_exact(tolerance) - 12 * pi) <= 12 * pi * tolerance);
    const BRep torus = BRep::create_torus(3, 0.5).transformed(Xform::translation(0.3, -0.2, 0.1));
    for (double tolerance : {1e-6, 1e-8, 1e-10})
        MINI_CHECK(std::abs(torus.volume_exact(tolerance) - 1.5 * pi * pi) <= 1.5 * pi * pi * tolerance);
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

MINI_TEST("BRep Native Measure", "Signed Polynomial Primitive And Span Sides") {
    NurbsSurface plane(3, false, 2, 2, 2, 2);
    plane.m_nurbsknot[0] = {0, 1};
    plane.m_nurbsknot[1] = {0, 1};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(plane.set_cv(i, j, {double(i), double(j), 1}));
    size_t work = 0;
    const auto charge = [&]() {
        if (++work > 200000)
            throw std::runtime_error("test cache budget");
    };
    const brep_measure::polynomial::Primitive positive(plane, 200000 - work, charge);
    for (double u : {0.125, 0.375, 0.875}) {
        const nurbs_interval::Interval value = positive.at(u, .375);
        MINI_CHECK(nurbs_interval::finite(value) && value.lo > 0);
        MINI_CHECK(value.lo <= u / 3 && value.hi >= u / 3);
    }
    NurbsSurface corner(3, false, 2, 2, 2, 3);
    corner.m_nurbsknot[0] = {-2, 3};
    corner.m_nurbsknot[1] = {0, 1, 2};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            MINI_CHECK(corner.set_cv(i, j, {i ? 3.0 : -2.0, double(j), j == 2 ? 4.0 : 2.0}));
    const brep_measure::polynomial::Primitive sided(corner, 200000 - work, charge);
    const nurbs_interval::Interval left = sided.at(-.5, .5), knot = sided.at(-.5, 1), right = sided.at(-.5, 1.5);
    MINI_CHECK(left.lo <= 1 && left.hi >= 1);
    MINI_CHECK(knot.lo <= 0 && knot.hi >= 0 && nurbs_interval::magnitude(knot) < 1e-12);
    MINI_CHECK(right.lo <= 0 && right.hi >= 0 && nurbs_interval::magnitude(right) < 1e-12);
    MINI_CHECK(work > 0 && work < 200000);
    const size_t before = work;
    bool rejected = false;
    try {
        const brep_measure::polynomial::Primitive insufficient(corner, 1, charge);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    MINI_CHECK(rejected && work == before);
}

MINI_TEST("BRep Native Measure", "Cached Primitive Bias Remains In Error Check") {
    const nurbs_interval::Interval biased(1 - 1e-4, 1 + 1e-4);
    MINI_CHECK(!brep_measure::polynomial::resolved(biased, biased, 1e-6));
    MINI_CHECK(brep_measure::polynomial::resolved(nurbs_interval::Interval(1), nurbs_interval::Interval(1), 1e-6));
}

MINI_TEST("BRep Native Measure", "Curved Trim Representation And Reversed Volume") {
    BRep shape = BRep::create_box(2, 3, 4);
    for (NurbsCurve& curve : shape.m_curves_2d)
        MINI_CHECK(curve.increase_degree(2));
    MINI_CHECK(std::abs(brep_measure::volume(shape, 1e-8) - 24) < 24e-8);
    for (NurbsCurve& curve : shape.m_curves_2d) {
        const std::pair<double, double> domain = curve.domain();
        MINI_CHECK(curve.insert_nurbsknot(domain.first + (domain.second - domain.first) * .25));
        MINI_CHECK(curve.insert_nurbsknot(domain.first + (domain.second - domain.first) * .75));
    }
    MINI_CHECK(std::abs(brep_measure::volume(shape, 1e-8) - 24) < 24e-8);
    shape.m_solids.front().shells.front().orientation = BRepOrientation::Reversed;
    MINI_CHECK(std::abs(brep_measure::volume(shape, 1e-8) + 24) < 24e-8);
}

MINI_TEST("BRep Native Measure", "Analytic Saddle Primitive Off Node And Reversed Domain") {
    NurbsSurface source(3, false, 2, 2, 2, 2);
    source.m_nurbsknot[0] = {-2, 3};
    source.m_nurbsknot[1] = {1, 5};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            const double u = i ? 3 : -2, v = j ? 5 : 1;
            MINI_CHECK(source.set_cv(i, j, {u, v, u * v + 32}));
        }
    MINI_CHECK(!source.is_planar(nullptr, 1e-12));
    NurbsSurface reversed = source;
    MINI_CHECK(reversed.reverse(0));
    MINI_CHECK(reversed.set_domain(0, 4, 14) && reversed.set_domain(1, -8, 8));
    size_t work = 0;
    const auto charge = [&]() {
        if (++work > 200000)
            throw std::runtime_error("test cache budget");
    };
    const brep_measure::polynomial::Primitive original_cache(source, 200000 - work, charge);
    const brep_measure::polynomial::Primitive reversed_cache(reversed, 200000 - work, charge);
    for (double u : {-1.75, -.25, 1.75, 2.5})
        for (double v : {1.5, 2.25, 4.5}) {
            const double expected = (32 * (u + 2) - v * (u * u - 4) * .5) / 3;
            const double expected_reversed = (32 * (u - 3) - v * (u * u - 9) * .5) / 12;
            const nurbs_interval::Interval forward = original_cache.at(u, v);
            const nurbs_interval::Interval backward = reversed_cache.at(10 - 2 * u, 4 * v - 12);
            MINI_CHECK(nurbs_interval::finite(forward) && forward.lo > 0);
            MINI_CHECK(nurbs_interval::finite(backward) && backward.hi < 0);
            MINI_CHECK(forward.lo <= expected && forward.hi >= expected);
            MINI_CHECK(backward.lo <= expected_reversed && backward.hi >= expected_reversed);
            MINI_CHECK(forward.hi - forward.lo < 1e-10 * std::abs(expected));
            MINI_CHECK(backward.hi - backward.lo < 1e-10 * std::abs(expected_reversed));
        }
}

MINI_TEST("BRep Native Measure", "Repeated Bicubic Primitive Fits Shared Work Budget") {
    NurbsSurface surface(3, false, 2, 2, 2, 2);
    surface.m_nurbsknot[0] = {0, 1};
    surface.m_nurbsknot[1] = {0, 1};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(surface.set_cv(i, j, {double(i), double(j), 1 + double(i * j)}));
    for (int direction = 0; direction < 2; ++direction) {
        MINI_CHECK(surface.increase_degree(direction, 3));
        for (int k = 1; k < 8; ++k)
            MINI_CHECK(surface.insert_nurbsknot(direction, k / 8.0));
    }
    size_t work = 0;
    const auto charge = [&]() {
        if (++work > 200000)
            throw std::runtime_error("test cache budget");
    };
    for (int face = 0; face < 6; ++face) {
        const brep_measure::polynomial::Primitive primitive(surface, 200000 - work, charge);
        for (double u : {.17, .43, .91}) {
            const double v = .37;
            const double expected = (u - v * u * u / 2) / 3;
            const nurbs_interval::Interval value = primitive.at(u, v);
            MINI_CHECK(nurbs_interval::finite(value));
            MINI_CHECK(value.lo <= expected && value.hi >= expected);
            MINI_CHECK(value.hi - value.lo < 1e-9);
        }
    }
    MINI_CHECK(work < 200000);
}


MINI_TEST("BRep Native Measure", "Unaffordable Dense Cache Keeps Trimmed Adaptive Volume") {
    BRep shape = BRep::create_box(2, 3, 4);
    const NurbsSurface original = shape.m_surfaces[0];
    const auto u = original.domain(0), v = original.domain(1);
    // Same plane extended to twice its old chart footprint; the box trims
    // occupy only the central half, so rectangle direct integration cannot apply.
    NurbsSurface extended = original;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(extended.set_cv(i, j, original.point_at(
                u.first + (2 * i - .5) * (u.second - u.first),
                v.first + (2 * j - .5) * (v.second - v.first))));
    NurbsSurface dense(3, false, 4, 4, 7, 104);
    for (int dir = 0; dir < 2; ++dir) {
        const auto domain = extended.domain(dir);
        const int spans = dir ? 101 : 4;
        auto& knots = dense.m_nurbsknot[dir];
        knots.assign(3, domain.first);
        for (int i = 1; i < spans; ++i)
            knots.push_back(domain.first + (domain.second - domain.first) * i / spans);
        knots.insert(knots.end(), 3, domain.second);
    }
    for (int i = 0; i < dense.cv_count(0); ++i)
        for (int j = 0; j < dense.cv_count(1); ++j) {
            const auto& a = dense.m_nurbsknot[0];
            const auto& b = dense.m_nurbsknot[1];
            MINI_CHECK(dense.set_cv(i, j, extended.point_at(
                (a[i] + a[i + 1] + a[i + 2]) / 3,
                (b[j] + b[j + 1] + b[j + 2]) / 3)));
        }
    MINI_CHECK(dense.is_valid());
    const size_t estimate = 404 * (54 * 3 * 3 + 3 * 3 + 3 * 3);
    MINI_CHECK(estimate > 200000); // Optional cache cannot fit unchanged budget.
    shape.m_surfaces[0] = dense;
    for (const BRepEdge& edge : shape.m_edges)
        for (const auto& relation : edge.pcurves)
            if (relation.surface_index == 0) {
                auto& pc = shape.m_curves_2d[relation.curve_2d_index];
                for (int i = 0; i < pc.cv_count(); ++i) {
                    const Point point = pc.get_cv(i);
                    MINI_CHECK(pc.set_cv(i, {u.first + .25 * (u.second - u.first) + .5 * (point[0] - u.first),
                        v.first + .25 * (v.second - v.first) + .5 * (point[1] - v.first), 0}));
                }
            }
    MINI_CHECK(shape.is_valid() && shape.is_solid());
    MINI_CHECK(std::abs(shape.volume_exact(1e-8) - 24) <= 24e-8);
    shape.m_solids[0].shells[0].orientation = BRepOrientation::Reversed;
    MINI_CHECK(std::abs(shape.volume_exact(1e-8) + 24) <= 24e-8);
}

MINI_TEST("BRep Native Measure", "Plane Primitive Reuses Certified C1 Derivative Edges") {
    // Exact dyadic Greville controls reproduce S=(u,v,32+u*v).
    // Nonuniform count/aspect and off-knot samples independently check the primitive.
    NurbsSurface surface(3, false, 3, 3, 23, 22);
    for (int dir = 0; dir < 2; ++dir) {
        const int spans = dir ? 20 : 21;
        auto& knots = surface.m_nurbsknot[dir];
        knots = {0, 0};
        for (int i = 1; i < spans; ++i)
            knots.push_back(i / 32.);
        knots.push_back(spans / 32.);
        knots.push_back(spans / 32.);
        // Unequal adjacent dyadic widths exercise cross-cell derivative reuse.
        knots[dir ? 11 : 10] += dir ? -1 / 64. : 1 / 64.;
    }
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const auto& u = surface.m_nurbsknot[0];
            const auto& v = surface.m_nurbsknot[1];
            const double x = (u[i] + u[i + 1]) * .5, y = (v[j] + v[j + 1]) * .5;
            MINI_CHECK(surface.set_cv(i, j, {x, y, 32 + x * y}));
        }
    MINI_CHECK(surface.is_valid() && brep_measure::plane::c1(surface));
    size_t charged = 0;
    const brep_measure::plane::Primitive cache(surface, {0, 21 / 32., 0, 20 / 32.}, 200000, [&]() {
        if (++charged > 200000)
            throw std::runtime_error("test cache exceeded fixed work budget");
    });
    MINI_CHECK(cache.shared && cache.cells.size() == 420);
    const size_t expected_charge = 2 * (22 * 41 + 43 * 21) + 420 * 88;
    MINI_CHECK(charged == expected_charge);
    for (double u : {5 / 128., 41 / 128., 79 / 128.})
        for (double v : {7 / 128., 31 / 128., 73 / 128.}) {
            const auto value = cache.at(u, v);
            const double exact = 32 * u + v * u * u * .5;
            MINI_CHECK(nurbs_interval::finite(value));
            MINI_CHECK(value.lo <= exact && value.hi >= exact);
            MINI_CHECK(value.hi - value.lo < 1e-9);
        }
    // A zero-stride degree-one mosaic is NEVER shared, even though it is cheap.
    NurbsSurface linear(3, false, 2, 2, 3, 2);
    linear.m_nurbsknot[0] = {0, .5, 1};
    linear.m_nurbsknot[1] = {0, 1};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(linear.set_cv(i, j, {i * .5, double(j), i == 2 ? 4. : 2.}));
    const brep_measure::plane::Primitive no_share(linear, {0, 1, 0, 1}, 200000, []() {});
    MINI_CHECK(!no_share.shared && no_share.cells.size() == 2);
    MINI_CHECK(!brep_measure::plane::c1(linear));
    NurbsSurface crease(3, false, 3, 3, 5, 3);
    crease.m_nurbsknot[0] = {0, 0, .5, .5, 1, 1};
    crease.m_nurbsknot[1] = {0, 0, 1, 1};
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 3; ++j)
            MINI_CHECK(crease.set_cv(i, j, {i * .25, j * .5, i < 3 ? 2. : double(i)}));
    MINI_CHECK(crease.is_valid() && !brep_measure::plane::c1(crease));
    const brep_measure::plane::Primitive c0(crease, {0, 1, 0, 1}, 200000, []() {});
    MINI_CHECK(!c0.shared && c0.cells.size() == 2);
    const auto beyond = c0.at(.75, .25);
    MINI_CHECK(beyond.lo <= 1.625 && beyond.hi >= 1.625);
}

MINI_TEST("BRep Native Measure", "Trim Gauge Preserves Tolerance Closed Rotated Boundary") {
    BRep shape = BRep::create_box(2, 3, 4);
    const auto original = shape.m_surfaces[0];
    const auto u = original.domain(0), v = original.domain(1);
    NurbsSurface extended(3, false, 2, 2, 5, 5);
    extended.m_nurbsknot[0] = {u.first, u.first + .25 * (u.second - u.first),
                               u.first + .5 * (u.second - u.first), u.first + .75 * (u.second - u.first),
                               u.second};
    extended.m_nurbsknot[1] = {v.first, v.first + .25 * (v.second - v.first),
                               v.first + .5 * (v.second - v.first), v.first + .75 * (v.second - v.first),
                               v.second};
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            MINI_CHECK(extended.set_cv(i, j,
                                       original.point_at(u.first + (3 * i / 4. - 1) * (u.second - u.first),
                                                         v.first + (3 * j / 4. - 1) * (v.second - v.first))));
    shape.m_surfaces[0] = extended;
    int altered = -1;
    for (const auto& edge : shape.m_edges)
        for (const auto& relation : edge.pcurves)
            if (relation.surface_index == 0) {
                auto& pc = shape.m_curves_2d[relation.curve_2d_index];
                for (int i = 0; i < pc.cv_count(); ++i) {
                    const auto point = pc.get_cv(i);
                    MINI_CHECK(
                        pc.set_cv(i, {u.first + ((point[0] - u.first) + (u.second - u.first)) / 3,
                                      v.first + ((point[1] - v.first) + (v.second - v.first)) / 3, 0}));
                }
                altered = relation.curve_2d_index;
            }
    MINI_CHECK(altered >= 0);
    auto& pc = shape.m_curves_2d[altered];
    const double before = pc.cv(0)[1];
    pc.cv(0)[1] = std::nextafter(before, std::numeric_limits<double>::infinity());
    MINI_CHECK(pc.cv(0)[1] != before); // Preserve a genuine nonzero UV closure gap.
    MINI_CHECK(shape.is_valid() && shape.is_solid());
    MINI_CHECK(std::abs(shape.volume_exact(1e-8) - 24) <= 24e-8);
    BRep rotated = shape.transformed(Xform::rotation({1, 2, 3}, .73));
    rotated = rotated.transformed(Xform::translation(1048576, -524288, 262144));
    MINI_CHECK(std::abs(rotated.volume_exact(1e-8) - 24) <= 24e-8);
    rotated.m_solids[0].shells[0].orientation = BRepOrientation::Reversed;
    MINI_CHECK(std::abs(rotated.volume_exact(1e-8) + 24) <= 24e-8);
}

namespace session_cpp {
MINI_TEST("BRep Native Measure", "Dense Rational Sphere Lens Inner Integration") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const BRep source = BRep::file_json_load((root / "tests/fixtures/native_sphere_lens_nesting_boundary_targets.json").string());
    const auto before = source.jsondump();
    MINI_CHECK(source.is_valid() && source.is_solid());
    MINI_CHECK(std::abs(source.volume_exact(1e-8) - 5 * Tolerance::PI / 12) < 1e-8);
    BRep reversed = source;
    reversed.m_solids.front().shells.front().orientation = BRepOrientation::Reversed;
    MINI_CHECK(std::abs(reversed.volume_exact(1e-8) + 5 * Tolerance::PI / 12) < 1e-8);
    MINI_CHECK(source.jsondump() == before);
}
}

namespace session_cpp {
MINI_TEST("BRep Native Measure", "Sphere Xor Shared Support Primitive Reuse") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const BRep source = BRep::file_json_load(
        (root / "tests/fixtures/native_sphere_xor_containment_boundary_target.json").string());
    const auto before = source.jsondump();
    std::vector<brep_assemble::FaceUse> uses;
    for (int face=0;face<source.face_count();++face)
        uses.push_back({&source,face,false});
    const BRep result = brep_assemble::assemble(uses,1e-7);
    MINI_CHECK(result.is_valid() && result.is_solid() && result.solid_count()==2);
    const double pi=std::acos(-1.);
    // Independent unit-sphere crescent oracle: each 4pi/3 - overlap5pi/12.
    MINI_CHECK(std::abs(result.volume_exact(1e-8)-11*pi/6)<1e-8);
    MINI_CHECK(brep_classify::point(result,{-.25,0,0},1e-7)==brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(result,{.5,.25,.25},1e-7)==brep_classify::State::Outside);
    for (const auto& solid : result.m_solids) {
        BRep component=result;
        component.m_solids={solid};
        MINI_CHECK(std::abs(component.volume_exact(1e-8)-11*pi/12)<1e-8);
        for (auto& shell : component.m_solids.front().shells)
            shell.orientation=brep_reverse(shell.orientation);
        MINI_CHECK(std::abs(component.volume_exact(1e-8)+11*pi/12)<1e-8);
    }
    MINI_CHECK(source.jsondump()==before);
}
}

MINI_TEST("BRep Native Measure", "Volume Keeps Its Value On Spans That Are Not Bezier Patches") {
    for (BRep shape : {BRep::create_sphere(2), BRep::create_torus(3, 0.5), BRep::create_cylinder(2, 3)}) {
        const double before = shape.volume_exact(1e-9);
        BRep raised = shape;
        for (BRep* changed : {&shape, &raised})
            for (NurbsSurface& surface : changed->m_surfaces)
                for (int dir = 0; dir < 2; ++dir) {
                    const std::pair<double, double> domain = surface.domain(dir);
                    if (surface.degree(dir) < 2)
                        continue;
                    if (changed == &raised)
                        MINI_CHECK(surface.increase_degree(dir, 3));
                    for (double share : {0.3, 0.71})
                        MINI_CHECK(surface.insert_nurbsknot(dir, std::lerp(domain.first, domain.second, share), 1));
                }
        for (const BRep* changed : {&shape, &raised}) {
            MINI_CHECK(changed->is_valid() && changed->is_solid());
            MINI_CHECK(std::abs(changed->volume_exact(1e-9) - before) <= std::abs(before) * 1e-9);
        }
    }
}

MINI_TEST("BRep Native Measure", "Enclosure Of A Point Is One Control") {
    const std::array<double, 3> origin = {0.1, 0.2, 0.3};
    const auto same = [](const nurbs_interval::Interval& a, const nurbs_interval::Interval& b) { return a.lo == b.lo && a.hi == b.hi; };
    const BRep torus = BRep::create_torus(1.0, 0.25);
    const NurbsSurface& surface = torus.m_surfaces[0];
    const NurbsCurve& curve = torus.m_curves_3d[0];
    for (double share : {0.0, 0.137, 0.25, 0.61, 1.0}) {
        const double t = std::lerp(curve.domain_start(), curve.domain_end(), share);
        const auto controls = nurbs_interval::curve_controls(curve, {t, t}, origin);
        nurbs_interval::CurveBounds bounds;
        MINI_CHECK(static_cast<int>(controls.size()) == curve.order() && nurbs_interval::curve_bounds(curve, {t, t}, origin, bounds));
        const Point point = curve.point_at(t);
        for (int d = 0; d < 3 && !controls.empty(); ++d) {
            for (const auto& control : controls)
                MINI_CHECK(same(control[d], controls[0][d]));
            const nurbs_interval::Interval value = controls[0][d] / controls[0][3];
            MINI_CHECK(same(bounds.point[d], value) && value.hi - value.lo < 1e-13);
            MINI_CHECK(value.lo <= point[d] - origin[d] + 1e-14 && value.hi >= point[d] - origin[d] - 1e-14);
        }
        const double u = std::lerp(surface.domain(0).first, surface.domain(0).second, share);
        const double v = std::lerp(surface.domain(1).first, surface.domain(1).second, 1.0 - share);
        const auto net = nurbs_interval::surface_controls(surface, {u, u, v, v}, origin);
        nurbs_interval::SurfaceBounds box;
        MINI_CHECK(net.size() == 9 && nurbs_interval::surface_bounds(surface, {u, u, v, v}, origin, box, false));
        const Point on = surface.point_at(u, v);
        for (int d = 0; d < 3 && !net.empty(); ++d) {
            for (const auto& control : net)
                MINI_CHECK(same(control[d], net[0][d]));
            const nurbs_interval::Interval value = net[0][d] / net[0][3];
            MINI_CHECK(same(box.point[d], value) && value.hi - value.lo < 1e-13);
            MINI_CHECK(value.lo <= on[d] - origin[d] + 1e-14 && value.hi >= on[d] - origin[d] - 1e-14);
        }
        // A point in u alone: every row of the net is the first.
        const double beyond = std::min(v + 0.01, surface.domain(1).second);
        const auto rows = nurbs_interval::surface_controls(surface, {u, u, std::min(v, beyond - 0.005), beyond}, origin, true);
        for (size_t k = 3; k < rows.size(); ++k)
            for (int d = 0; d < 4; ++d)
                MINI_CHECK(same(rows[k][d], rows[k % 3][d]));
    }
    // A piece inside one span is enclosed from that span's controls; across a knot the whole curve has to be valid.
    NurbsCurve broken = NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0.5}, {2, -1, 1}, {4, 0.5, 0}, {5, 3, 2}, {7, 1, 1}, {8, 0, 3}});
    const std::vector<double> spans = broken.get_span_vector();
    broken.m_cv.back() = std::numeric_limits<double>::infinity();
    nurbs_interval::CurveBounds piece;
    MINI_CHECK(spans.size() > 2 && !broken.is_valid());
    MINI_CHECK(nurbs_interval::curve_bounds(broken, {spans[0], std::midpoint(spans[0], spans[1])}, origin, piece));
    MINI_CHECK(!nurbs_interval::curve_bounds(broken, {spans[0], std::midpoint(spans[1], spans[2])}, origin, piece));
}
