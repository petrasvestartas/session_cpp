#include "nurbs_surface_span.h"
#include "brep.h"
#include "mini_test.h"

#include <cmath>
#include <fstream>
#include <limits>

namespace session_cpp {
namespace {

NurbsSurface affine_rational(double scale) {
    NurbsSurface source(3, true, 4, 3, 7, 6);
    source.m_nurbsknot[0] = {-3, -3, -3, -2, -0.3, 0.8, 4, 4, 4};
    source.m_nurbsknot[1] = {-7, -7, -5, 1, 6, 9, 9};
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < source.cv_count(1); ++j) {
            const double weight = scale * (0.75 + (i * 3 + j * 2) % 7 * 0.13);
            const Point point(i * 0.3, j * 0.4, std::sin(i * 0.4) * std::cos(j * 0.7));
            MINI_CHECK(source.set_cv_4d(i, j, point[0] * weight, point[1] * weight, point[2] * weight, weight));
        }
    return source;
}

void compare(
    const NurbsSurface& source,
    nurbs_detail::SurfaceEvaluator& local,
    double u,
    double v
) {
    const Point point = source.point_at(u, v);
    const Point actual = local.point_at(u, v);
    MINI_CHECK(point.distance(actual) <= 1e-12);
    const std::vector<Vector> reference = source.evaluate(u, v, 2);
    const std::vector<Vector> derivatives = local.evaluate(u, v, 2);
    MINI_CHECK(reference.size() == 6 && derivatives.size() == 6);
    for (size_t i = 0; i < reference.size(); ++i)
        for (int d = 0; d < 3; ++d)
            MINI_CHECK(std::abs(reference[i][d] - derivatives[i][d]) <= 1e-12 * std::max(1.0, std::abs(reference[i][d])));
}

std::vector<double> parameters(const NurbsSurface& source, int direction) {
    const std::vector<double> spans = source.get_span_vector(direction);
    std::vector<double> result;
    for (size_t i = 0; i < spans.size(); ++i) {
        result.push_back(std::nextafter(spans[i], -std::numeric_limits<double>::infinity()));
        result.push_back(spans[i]);
        result.push_back(std::nextafter(spans[i], std::numeric_limits<double>::infinity()));
        if (i + 1 < spans.size())
            result.push_back((spans[i] + spans[i + 1]) * 0.5);
    }
    return result;
}

}

MINI_TEST("NurbsSurfaceSpan", "Rational Affine Knots And Global Weight Scale") {
    for (double scale : {1e-100, 1.0, 1e100}) {
        const NurbsSurface source = affine_rational(scale);
        MINI_CHECK(source.is_valid());
        nurbs_detail::SurfaceEvaluator local(source);
        const std::vector<double> u = parameters(source, 0);
        const std::vector<double> v = parameters(source, 1);
        for (size_t i = 0; i < u.size(); ++i)
            for (size_t j = 0; j < v.size(); ++j)
                compare(source, local, u[u.size() - i - 1], v[j]);
    }
}

MINI_TEST("NurbsSurfaceSpan", "Preserved Dense Offset Support") {
    std::ifstream input("tests/fixtures/native_thicken_rational_saddle_result.brep.json");
    MINI_CHECK(input.good());
    const BRep saved = BRep::jsonload(nlohmann::json::parse(input));
    const NurbsSurface* source = nullptr;
    for (const NurbsSurface& surface : saved.m_surfaces)
        if (surface.cv_count(0) == 131 && surface.cv_count(1) == 131)
            source = &surface;
    MINI_CHECK(source != nullptr && !source->is_rational());
    nurbs_detail::SurfaceEvaluator local(*source);
    const std::vector<double> u = source->get_span_vector(0);
    const std::vector<double> v = source->get_span_vector(1);
    for (size_t i = 0; i < 97; ++i) {
        const size_t a = i * 71 % (u.size() - 1);
        const size_t b = i * 43 % (v.size() - 1);
        compare(*source, local, u[a], v[b]);
        compare(*source, local, (u[a] + u[a + 1]) * 0.5, (v[b] + v[b + 1]) * 0.5);
    }
}

MINI_TEST("NurbsSurfaceSpan", "Validate Entire Input Before Local Evaluation") {
    NurbsSurface source = affine_rational(1.0);
    source.cv(6, 5)[0] = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try {
        nurbs_detail::SurfaceEvaluator local(source);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    MINI_CHECK(rejected);
}

MINI_TEST("NurbsSurfaceSpan", "Finite Parameters And Negative Derivatives") {
    const NurbsSurface source = affine_rational(1.0);
    nurbs_detail::SurfaceEvaluator local(source);
    MINI_CHECK(local.evaluate(0.0, 0.0, -1).empty());
    for (double parameter : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        bool point_rejected = false;
        bool derivative_rejected = false;
        try {
            local.point_at(parameter, 0.0);
        } catch (const std::invalid_argument&) {
            point_rejected = true;
        }
        try {
            local.evaluate(0.0, parameter, 2);
        } catch (const std::invalid_argument&) {
            derivative_rejected = true;
        }
        MINI_CHECK(point_rejected && derivative_rejected);
    }
}

}
