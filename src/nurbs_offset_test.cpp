#include "mini_test.h"
#include "nurbs_offset.h"
#include "primitives.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

namespace session_cpp {
namespace {

Point curve_offset_point(
    const NurbsCurve& source,
    double parameter,
    double distance,
    const Vector& normal
) {
    const std::vector<Vector> derivatives = source.evaluate(parameter, 1);
    return source.point_at(parameter) + derivatives[1].cross(normal).normalized() * distance;
}

NurbsSurface offset_plane() {
    return NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {{0, 0, 0}, {0, 3, 1}, {2, 0, 0}, {2, 3, 1}}
    );
}

NurbsSurface offset_saddle() {
    std::vector<Point> points;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            points.push_back({double(i), double(j), 0.3 * (i - 1.5) * (j - 1.5)});
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

void save_curve_offset(
    const NurbsCurve& source,
    const NurbsCurve& result,
    double distance,
    const std::string& name
) {
    const std::filesystem::path folder = std::filesystem::path("build") / "native-geometry-tests";
    std::filesystem::create_directories(folder);
    nlohmann::ordered_json data = {
        {"distance", distance}, {"kind", "curve"}, {"offset", result.jsondump()},
        {"original", source.jsondump()}, {"samples", nlohmann::ordered_json::array()}
    };
    for (int i = 0; i <= 200; ++i) {
        const double t = source.domain_start() + (source.domain_end() - source.domain_start()) * i / 200.0;
        const Point a = source.point_at(t);
        const Point b = result.point_at(t);
        data["samples"].push_back({a[0], a[1], a[2], b[0], b[1], b[2]});
    }
    std::ofstream(folder / (name + ".offset.json")) << data.dump(2);
}

void save_surface_offset(
    const NurbsSurface& source,
    const NurbsSurface& result,
    double distance,
    const std::string& name
) {
    const std::filesystem::path folder = std::filesystem::path("build") / "native-geometry-tests";
    std::filesystem::create_directories(folder);
    nlohmann::ordered_json data = {
        {"distance", distance}, {"kind", "surface"}, {"offset", result.jsondump()},
        {"original", source.jsondump()}, {"samples", nlohmann::ordered_json::array()}
    };
    const std::pair<double, double> u = source.domain(0);
    const std::pair<double, double> v = source.domain(1);
    for (int i = 0; i <= 30; ++i)
        for (int j = 0; j <= 30; ++j) {
            const double x = u.first + (u.second - u.first) * i / 30.0;
            const double y = v.first + (v.second - v.first) * j / 30.0;
            const Point a = source.point_at(x, y);
            const Point b = result.point_at(x, y);
            data["samples"].push_back({a[0], a[1], a[2], b[0], b[1], b[2]});
        }
    std::ofstream(folder / (name + ".offset.json")) << data.dump(2);
}

}

MINI_TEST("NurbsOffset", "Signed Exact Line") {
    NurbsCurve source = NurbsCurve::create(false, 1, {{0, 0, 0}, {2, 0, 0}});
    source.set_domain(17.0, 23.0);
    for (double distance : {-0.3, 0.3}) {
        const NurbsCurve result = nurbs_offset::curve(source, distance, {0, 0, 7}, 1e-8);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.degree() == 1);
        MINI_CHECK(result.domain() == source.domain());
        for (int i = 0; i <= 20; ++i) {
            const double t = 17.0 + 6.0 * i / 20.0;
            MINI_CHECK(result.point_at(t).distance({2.0 * i / 20.0, -distance, 0}) < 1e-12);
        }
    }
}

MINI_TEST("NurbsOffset", "Signed Exact Rational Circle") {
    NurbsCurve source = Primitives::circle(3, -4, 5, 2);
    source.set_domain(7.0, 11.0);
    for (double distance : {-0.4, 0.4}) {
        const NurbsCurve result = nurbs_offset::curve(source, distance, {0, 0, 1}, 1e-8);
        MINI_CHECK(result.is_rational());
        MINI_CHECK(result.degree() == 2);
        MINI_CHECK(result.cv_count() == source.cv_count());
        MINI_CHECK(result.domain() == source.domain());
        for (int i = 0; i <= 300; ++i) {
            const double t = 7.0 + 4.0 * i / 300.0;
            MINI_CHECK(result.point_at(t).distance(curve_offset_point(source, t, distance, {0, 0, 1})) < 1e-11);
            MINI_CHECK(std::abs(result.point_at(t).distance({3, -4, 5}) - (2.0 + distance)) < 1e-11);
        }
        save_curve_offset(source, result, distance, distance < 0 ? "native_circle_inward" : "native_circle_outward");
    }
}

MINI_TEST("NurbsOffset", "Freeform Curve Parameter Correspondence") {
    NurbsCurve source = NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0.4}, {2, -1, -0.2}, {3, 1, 1}});
    source.set_domain(-8.0, -2.0);
    const Vector normal(0, 0, 1);
    for (double distance : {-0.1, 0.1}) {
        const NurbsCurve result = nurbs_offset::curve(source, distance, normal, 1e-6);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.domain() == source.domain());
        for (int i = 0; i <= 997; ++i) {
            const double t = -8.0 + 6.0 * i / 997.0;
            const Point expected = curve_offset_point(source, t, distance, normal);
            MINI_CHECK(result.point_at(t).distance(expected) < 1e-6);
        }
        save_curve_offset(source, result, distance, distance < 0 ? "native_freeform_curve_negative" : "native_freeform_curve_positive");
    }
}

MINI_TEST("NurbsOffset", "Tilted Circle Normal And Rational Scale") {
    NurbsCurve source = Primitives::circle(0, 0, 0, 2);
    for (double& coordinate : source.m_cv)
        coordinate *= 1e12;
    const Vector normal(1e-7, 0, 1);
    const NurbsCurve result = nurbs_offset::curve(source, 0.3, normal, 1e-8);
    for (int i = 0; i <= 401; ++i) {
        const double t = source.domain_start() + (source.domain_end() - source.domain_start()) * i / 401.0;
        MINI_CHECK(result.point_at(t).distance(curve_offset_point(source, t, 0.3, normal)) < 1e-8);
    }
    source.reverse();
    const NurbsCurve reversed = nurbs_offset::curve(source, 0.3, {0, 0, 1}, 1e-8);
    for (int i = 0; i <= 51; ++i) {
        const double t = source.domain_start() + (source.domain_end() - source.domain_start()) * i / 51.0;
        MINI_CHECK(std::abs(reversed.point_at(t).distance({0, 0, 0}) - 1.7) < 1e-11);
    }
}

MINI_TEST("NurbsOffset", "Curve Singularities") {
    const std::array<NurbsCurve, 3> sources = {
        NurbsCurve::create(false, 1, {{0, 0, 0}, {0, 0, 1}}),
        NurbsCurve::create(false, 2, {{1, 0, 0}, {-1, 0, 0}, {1, 0, 0}}),
        Primitives::circle(0, 0, 0, 1)
    };
    for (int i = 0; i < 3; ++i) {
        bool rejected = false;
        try {
            nurbs_offset::curve(sources[i], i == 2 ? -1.0 : 0.1, {0, 0, 1}, 1e-7);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
    }
}

MINI_TEST("NurbsOffset", "Signed Exact Plane") {
    NurbsSurface source = offset_plane();
    source.set_domain(0, -7.0, 3.0);
    source.set_domain(1, 9.0, 12.0);
    for (double distance : {-0.25, 0.25}) {
        const NurbsSurface result = nurbs_offset::surface(source, distance, 1e-9);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.degree(0) == 1);
        MINI_CHECK(result.degree(1) == 1);
        MINI_CHECK(result.domain(0) == source.domain(0));
        MINI_CHECK(result.domain(1) == source.domain(1));
        for (int i = 0; i <= 11; ++i)
            for (int j = 0; j <= 13; ++j) {
                const double u = -7.0 + 10.0 * i / 11.0;
                const double v = 9.0 + 3.0 * j / 13.0;
                const Point expected = source.point_at(u, v) + source.normal_at(u, v) * distance;
                MINI_CHECK(result.point_at(u, v).distance(expected) < 1e-12);
            }
    }
    source.reverse(0);
    const NurbsSurface reversed = nurbs_offset::surface(source, 0.25, 1e-9);
    const Point expected = source.point_at(-2.0, 10.0) + source.normal_at(-2.0, 10.0) * 0.25;
    MINI_CHECK(reversed.point_at(-2.0, 10.0).distance(expected) < 1e-12);
}

MINI_TEST("NurbsOffset", "Exact Rational Sphere Including Poles") {
    const Point center(3, -4, 5);
    NurbsSurface source = Primitives::sphere_surface(3, -4, 5, 2);
    source.set_domain(0, 17.0, 21.0);
    source.set_domain(1, -3.0, 2.0);
    const double sign = source.normal_at(19.0, -0.5).dot(source.point_at(19.0, -0.5) - center) > 0.0 ? 1.0 : -1.0;
    for (double distance : {-0.4, 0.4}) {
        const NurbsSurface result = nurbs_offset::surface(source, distance, 1e-8);
        MINI_CHECK(result.is_rational());
        MINI_CHECK(result.degree(0) == 2);
        MINI_CHECK(result.degree(1) == 2);
        MINI_CHECK(result.domain(0) == source.domain(0));
        MINI_CHECK(result.domain(1) == source.domain(1));
        for (int i = 0; i <= 31; ++i)
            for (int j = 0; j <= 29; ++j) {
                const double u = 17.0 + 4.0 * i / 31.0;
                const double v = -3.0 + 5.0 * j / 29.0;
                const Point point = source.point_at(u, v);
                const Point expected = center + (point - center) * (1.0 + sign * distance / 2.0);
                MINI_CHECK(result.point_at(u, v).distance(expected) < 1e-10);
            }
        save_surface_offset(source, result, distance, distance < 0 ? "native_sphere_negative" : "native_sphere_positive");
    }
}

MINI_TEST("NurbsOffset", "Rational Cylinder") {
    NurbsSurface source = Primitives::cylinder_surface(0, 0, -1, 2, 3);
    source.set_domain(0, 3.0, 5.0);
    source.set_domain(1, -2.0, 1.0);
    for (double distance : {-0.2, 0.2}) {
        const NurbsSurface result = nurbs_offset::surface(source, distance, 1e-5);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.is_rational());
        MINI_CHECK(result.degree(0) == 2);
        MINI_CHECK(result.degree(1) == 1);
        MINI_CHECK(result.domain(0) == source.domain(0));
        MINI_CHECK(result.domain(1) == source.domain(1));
        for (int i = 0; i <= 71; ++i)
            for (int j = 0; j <= 17; ++j) {
                const double u = 3.0 + 2.0 * i / 71.0;
                const double v = -2.0 + 3.0 * j / 17.0;
                const Point expected = source.point_at(u, v) + source.normal_at(u, v) * distance;
                MINI_CHECK(result.point_at(u, v).distance(expected) < 1e-10);
            }
    }
}

MINI_TEST("NurbsOffset", "Doubly Curved Surface Parameter Correspondence") {
    NurbsSurface source = offset_saddle();
    source.set_domain(0, -4.0, -1.0);
    source.set_domain(1, 11.0, 18.0);
    for (double distance : {-0.1, 0.1}) {
        const NurbsSurface result = nurbs_offset::surface(source, distance, 1e-6);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.domain(0) == source.domain(0));
        MINI_CHECK(result.domain(1) == source.domain(1));
        for (int i = 0; i <= 53; ++i)
            for (int j = 0; j <= 47; ++j) {
                const double u = -4.0 + 3.0 * i / 53.0;
                const double v = 11.0 + 7.0 * j / 47.0;
                const Point expected = source.point_at(u, v) + source.normal_at(u, v) * distance;
                MINI_CHECK(result.point_at(u, v).distance(expected) < 1e-6);
            }
        save_surface_offset(source, result, distance, distance < 0 ? "native_saddle_negative" : "native_saddle_positive");
    }
}

MINI_TEST("NurbsOffset", "Small Near Planar Patch Preserves Normal Field") {
    const NurbsSurface source = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{0, 0, 0}, {0, 1e-6, 0}, {1e-6, 0, 0}, {1e-6, 1e-6, 5e-13}});
    const double tolerance = 1e-10;
    for (double distance : {-0.1, 0.1}) {
        const std::string name = distance > 0 ? "native_near_plane_positive" : "native_near_plane_negative";
        const NurbsSurface result = nurbs_offset::surface(source, distance, tolerance);
        save_surface_offset(source, result, distance, name);
        double residual = 0;
        for (int i = 0; i <= 13; ++i)
            for (int j = 0; j <= 17; ++j) {
                const double u = i / 13.0;
                const double v = j / 17.0;
                const Vector normal = Vector(-5e-7 * v, -5e-7 * u, 1).normalized();
                const Point expected = Point(1e-6 * u, 1e-6 * v, 5e-13 * u * v) + normal * distance;
                residual = std::max(residual, expected.distance(result.point_at(u, v)));
            }
        std::ofstream("build/native-geometry-tests/" + name + ".assertions.json")
            << nlohmann::ordered_json({{"distance", distance}, {"tolerance", tolerance}, {"residual", residual}}).dump(2);
        MINI_CHECK(residual <= tolerance);
    }
}

MINI_TEST("NurbsOffset", "Small Differential Scale And Affine Domains") {
    for (double scale : {0.001, 1.0, 1000.0}) {
        NurbsSurface source = NurbsSurface::create(false, false, 1, 1, 2, 2,
            {{0, 0, 0}, {0, 1e-6 * scale, 0}, {1e-6 * scale, 0, 0}, {1e-6 * scale, 1e-6 * scale, 5e-13 * scale}});
        source.set_domain(0, 3, 7);
        source.set_domain(1, -2, 4);
        for (double sign : {-1.0, 1.0}) {
            const double distance = sign * 0.1 * scale;
            const double tolerance = 1e-10 * scale;
            const NurbsSurface result = nurbs_offset::surface(source, distance, tolerance);
            const std::string name = "native_near_plane_scaled_" + std::to_string(scale) + (sign < 0 ? "_negative" : "_positive");
            save_surface_offset(source, result, distance, name);
            double residual = 0;
            for (int i = 0; i <= 13; ++i)
                for (int j = 0; j <= 17; ++j) {
                    const double u = i / 13.0;
                    const double v = j / 17.0;
                    const Vector normal = Vector(-5e-7 * v, -5e-7 * u, 1) / std::hypot(5e-7 * v, 5e-7 * u, 1.0);
                    const Point expected = Point(1e-6 * scale * u, 1e-6 * scale * v, 5e-13 * scale * u * v) + normal * distance;
                    residual = std::max(residual, expected.distance(result.point_at(3 + 4 * u, -2 + 6 * v)));
                }
            std::ofstream("build/native-geometry-tests/" + name + ".assertions.json")
                << nlohmann::ordered_json({{"scale", scale}, {"distance", distance}, {"tolerance", tolerance}, {"residual", residual}}).dump(2);
            MINI_CHECK(result.domain(0) == source.domain(0));
            MINI_CHECK(result.domain(1) == source.domain(1));
            MINI_CHECK(residual <= tolerance);
        }
    }
}

MINI_TEST("NurbsOffset", "Surface Singularities") {
    const std::array<NurbsSurface, 2> sources = {
        Primitives::cone_surface(0, 0, 0, 2, 3),
        Primitives::sphere_surface(0, 0, 0, 2)
    };
    for (int i = 0; i < 2; ++i) {
        bool rejected = false;
        try {
            const Point point = sources[i].point_at(0.5, 0.5);
            const double sign = sources[i].normal_at(0.5, 0.5).dot(point - Point(0, 0, 0)) > 0.0 ? 1.0 : -1.0;
            nurbs_offset::surface(sources[i], i == 1 ? -2.0 * sign : 0.1, 1e-7);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
    }
}

MINI_TEST("NurbsOffset", "Homogeneous Scale Invariance") {
    std::array<NurbsCurve, 2> curves = {
        Primitives::circle(3, -4, 5, 2),
        NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0.4}, {2, -1, -0.2}, {3, 1, 1}})
    };
    for (NurbsCurve& source : curves) {
        source.to_rational();
        const NurbsCurve reference = nurbs_offset::curve(source, 0.1, {0, 0, 1}, 1e-6);
        for (double scale : {1e-100, 1e100}) {
            NurbsCurve scaled = source;
            for (double& coordinate : scaled.m_cv)
                coordinate *= scale;
            const NurbsCurve result = nurbs_offset::curve(scaled, 0.1, {0, 0, 1}, 1e-6);
            MINI_CHECK(result.is_valid());
            for (int i = 0; i <= 97; ++i) {
                const double t = source.domain_start() + (source.domain_end() - source.domain_start()) * i / 97.0;
                MINI_CHECK(result.point_at(t).distance(reference.point_at(t)) < 1e-8);
            }
        }
    }
    std::array<NurbsSurface, 2> surfaces = {Primitives::sphere_surface(3, -4, 5, 2), offset_saddle()};
    for (NurbsSurface& source : surfaces) {
        source.to_rational();
        const NurbsSurface reference = nurbs_offset::surface(source, 0.1, 1e-6);
        const std::pair<double, double> u = source.domain(0);
        const std::pair<double, double> v = source.domain(1);
        for (double scale : {1e-100, 1e100}) {
            NurbsSurface scaled = source;
            for (double& coordinate : scaled.m_cv)
                coordinate *= scale;
            const NurbsSurface result = nurbs_offset::surface(scaled, 0.1, 1e-6);
            MINI_CHECK(result.is_valid());
            for (int i = 0; i <= 23; ++i)
                for (int j = 0; j <= 19; ++j) {
                    const double x = u.first + (u.second - u.first) * i / 23.0;
                    const double y = v.first + (v.second - v.first) * j / 19.0;
                    MINI_CHECK(result.point_at(x, y).distance(reference.point_at(x, y)) < 1e-8);
                }
        }
    }
}

MINI_TEST("NurbsOffset", "Invalid Data And Zero Distance") {
    const NurbsCurve source = Primitives::circle(0, 0, 0, 2);
    const NurbsSurface plane = offset_plane();
    MINI_CHECK(nurbs_offset::curve(source, 0.0, {0, 0, 1}, 1e-7).m_cv == source.m_cv);
    MINI_CHECK(nurbs_offset::surface(plane, 0.0, 1e-7).m_cv == plane.m_cv);
    for (int i = 0; i < 5; ++i) {
        bool rejected = false;
        try {
            NurbsCurve invalid = source;
            if (i == 0)
                invalid.set_weight(0, -1.0);
            if (i == 1)
                invalid.m_cv[0] = std::numeric_limits<double>::infinity();
            const double tolerance = i == 2 ? 0.0 : 1e-7;
            const double distance = i == 3 ? std::numeric_limits<double>::quiet_NaN() : 0.1;
            const Vector normal = i == 4 ? Vector(0, 0, 0) : Vector(0, 0, 1);
            nurbs_offset::curve(invalid, distance, normal, tolerance);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
    }
    NurbsSurface invalid = plane;
    invalid.to_rational();
    invalid.set_weight(0, 0, -1.0);
    bool rejected = false;
    try {
        nurbs_offset::surface(invalid, 0.1, 1e-7);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    MINI_CHECK(rejected);
}

}
