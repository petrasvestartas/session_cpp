#include "mini_test.h"
#include "nurbscurve.h"
#include "nurbs_coincidence.h"
#include "nurbscurve.pb.h"
#include "point.h"
#include "plane.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "primitives.h"
#include <array>
#include <cmath>
#include <numeric>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("NurbsCurve", "Arc Length Across Knots And Affine Domains") {
        NurbsCurve curve = NurbsCurve::create(false, 1,
            {Point(0, 0, 0), Point(2, 0, 0), Point(2, 3, 0), Point(6, 3, 0)});
        curve.set_nurbsknot(1, 0.0137);
        curve.set_nurbsknot(2, 0.5003);
        for (int pass = 0; pass < 2; ++pass) {
            const auto by_count = curve.divide_by_count(10, true).first;
            const auto by_length = curve.divide_by_length(1.0).first;
            MINI_CHECK(by_count.size() == 10);
            MINI_CHECK(by_length.size() == 10);
            for (int i = 0; i < 10; ++i) {
                const Point expected = i <= 2 ? Point(i, 0, 0)
                    : i <= 5 ? Point(2, i - 2, 0) : Point(i - 3, 3, 0);
                MINI_CHECK(by_count[i].distance(expected) < 1e-10);
                MINI_CHECK(by_length[i].distance(expected) < 1e-10);
            }
            MINI_CHECK(curve.set_domain(-9, 13));
        }
    }

    MINI_TEST("NurbsCurve", "Knot Insertion Preserves Constant Components") {
        const double alpha = 0.043163167015943436;
        NurbsCurve line = NurbsCurve::create(false, 1, {{7, -4, 0}, {7, 5, 0}});
        MINI_CHECK(line.set_domain(0, 1));
        const NurbsCurve original = line;
        MINI_CHECK(line.trim(alpha, 0.94));
        for (int i = 0; i < line.cv_count(); ++i)
            MINI_CHECK(line.cv(i)[0] == 7 && line.cv(i)[2] == 0);
        for (int i = 0; i <= 16; ++i) {
            const double t = alpha + (0.94 - alpha) * i / 16.0;
            MINI_CHECK(line.point_at(t).distance(original.point_at(t)) < 1e-12);
        }
        NurbsCurve rational(3, true, 2, 2);
        rational.m_nurbsknot = {0, 1};
        MINI_CHECK(rational.set_cv_4d(0, 7, -4, 0, 1));
        MINI_CHECK(rational.set_cv_4d(1, 7, 10, 0, 2));
        const NurbsCurve rational_original = rational;
        MINI_CHECK(rational.insert_nurbsknot(alpha, 1));
        for (int i = 0; i < rational.cv_count(); ++i)
            MINI_CHECK(rational.cv(i)[0] == 7 && rational.cv(i)[2] == 0);
        MINI_CHECK(rational.point_at_start()[0] == 7 && rational.point_at_end()[0] == 3.5);
        for (int i = 0; i <= 16; ++i)
            MINI_CHECK(rational.point_at(i / 16.0).distance(rational_original.point_at(i / 16.0)) < 1e-12);
        NurbsCurve periodic = NurbsCurve::create(true, 2, {{7, 0, 0}, {7, 2, 0}, {7, 2, 2}, {7, 0, 2}});
        const NurbsCurve periodic_original = periodic;
        const double first = periodic.domain_start();
        const double last = periodic.domain_end();
        MINI_CHECK(periodic.insert_nurbsknot(first + alpha * (last - first), 1));
        for (int i = 0; i < periodic.cv_count(); ++i)
            MINI_CHECK(periodic.cv(i)[0] == 7);
        for (int i = 0; i <= 16; ++i) {
            const double t = first + (last - first) * i / 16.0;
            MINI_CHECK(periodic.point_at(t).distance(periodic_original.point_at(t)) < 1e-12);
        }
    }

    MINI_TEST("NurbsCurve", "Chair Pcurve Arc Length Reparameterization") {
        const auto path = std::filesystem::path(__FILE__).parent_path().parent_path()
            / "tests/fixtures/chair_arc_length_pcurve.json";
        std::ifstream input(path);
        const NurbsCurve original = NurbsCurve::jsonload(nlohmann::json::parse(input));
        NurbsCurve normalized = original;
        MINI_CHECK(normalized.set_domain(0, 1));
        const auto before = original.divide_by_count(192, true).first;
        const auto after = normalized.divide_by_count(192, true).first;
        MINI_CHECK(before.size() == after.size());
        for (size_t i = 0; i < before.size(); ++i)
            MINI_CHECK(before[i].distance(after[i]) < 1e-10);
    }

    MINI_TEST("NurbsCurve", "Homogeneous Weight Scaling") {
        const NurbsCurve original = Primitives::circle(3, -4, 5, 2);
        for (double factor : {1e-100, 1e100}) {
            NurbsCurve curve = original;
            for (double& value : curve.m_cv)
                value *= factor;
            MINI_CHECK(curve.is_valid());
            for (int i = 0; i < curve.cv_count(); ++i)
                MINI_CHECK(curve.get_cv(i).distance(original.get_cv(i)) < 1e-13);
            for (int i = 0; i <= 20; ++i) {
                const double t = original.domain_start() + (original.domain_end() - original.domain_start()) * i / 20.0;
                MINI_CHECK(curve.point_at(t).distance(original.point_at(t)) < 1e-13);
                const auto a = curve.evaluate(t, 3);
                const auto b = original.evaluate(t, 3);
                for (int k = 0; k <= 3; ++k)
                    MINI_CHECK((a[k] - b[k]).magnitude() < 1e-10);
            }
        }
    }

    MINI_TEST("NurbsCurve", "Rational Derivatives Above Degree") {
        NurbsCurve curve(3, true, 2, 2);
        MINI_CHECK(curve.set_nurbsknot(0, 0) && curve.set_nurbsknot(1, 1));
        MINI_CHECK(curve.set_cv_4d(0, 0, 0, 0, 1));
        MINI_CHECK(curve.set_cv_4d(1, 2, 0, 0, 2));
        for (double t : {0.0, 0.2, 0.5, 1.0}) {
            const std::vector<Vector> derivatives = curve.evaluate(t, 5);
            MINI_CHECK(derivatives.size() == 6);
            MINI_CHECK(std::abs(derivatives[0][0] - 2.0 * t / (1.0 + t)) < 1e-13);
            double factorial = 1.0;
            for (int k = 1; k <= 5; ++k) {
                factorial *= k;
                const double expected = (k % 2 ? 2.0 : -2.0) * factorial / std::pow(1.0 + t, k + 1);
                MINI_CHECK(std::abs(derivatives[k][0] - expected) < 1e-11);
                MINI_CHECK(derivatives[k][1] == 0.0 && derivatives[k][2] == 0.0);
            }
        }
    }

    MINI_TEST("NurbsCurve", "Shared Interpolation Parameters") {
        const std::vector<double> parameters = {2.0, 2.01, 2.2, 2.9, 3.0};
        std::vector<Point> points;
        std::vector<Point> mapped;
        for (double t : parameters) {
            const Point p(t, t * t, t * t * t);
            points.push_back(p);
            mapped.push_back({3.0 * p[0] - p[1], 2.0 * p[2], p[1] + 7.0});
        }
        const NurbsCurve a = NurbsCurve::create_interpolated(points, parameters);
        const NurbsCurve b = NurbsCurve::create_interpolated(mapped, parameters);
        MINI_CHECK(a.is_valid() && b.is_valid());
        for (size_t i = 0; i < points.size(); ++i)
            MINI_CHECK(a.point_at(parameters[i]).distance(points[i]) < 1e-11);
        for (int i = 0; i <= 100; ++i) {
            const double t = 2.0 + i / 100.0;
            const Point p = a.point_at(t);
            MINI_CHECK(p.distance({t, t * t, t * t * t}) < 1e-11);
            MINI_CHECK(b.point_at(t).distance({3.0 * p[0] - p[1], 2.0 * p[2], p[1] + 7.0}) < 1e-11);
        }
        MINI_CHECK(!NurbsCurve::create_interpolated(points, std::vector<double>{0, 1, 1, 2, 3}).is_valid());
        MINI_CHECK(!NurbsCurve::create_interpolated(points, std::vector<double>{0, 1}).is_valid());
        const std::vector<Point> loop = {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};
        const std::vector<double> periodic_parameters = {0, 0.1, 0.5, 0.8, 1};
        const NurbsCurve periodic = NurbsCurve::create_interpolated(loop, periodic_parameters, true);
        MINI_CHECK(periodic.is_valid() && periodic.is_closed());
        for (size_t i = 0; i < loop.size(); ++i)
            MINI_CHECK(periodic.point_at(periodic_parameters[i]).distance(loop[i]) < 1e-11);
    }

    MINI_TEST("NurbsCurve", "Dense Periodic Interpolation") {
        std::vector<Point> points;
        for (int i = 0; i < 2048; ++i) {
            const double angle = Tolerance::TWO_PI * i / 2048.0;
            points.push_back({std::cos(angle), std::sin(angle), 0.15 * std::sin(3.0 * angle)});
        }
        const NurbsCurve curve = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::UniformPeriodic);
        MINI_CHECK(curve.is_valid() && curve.is_closed());
        for (int i = 0; i < 2048; ++i) {
            const double t = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 2048.0;
            MINI_CHECK(curve.point_at(t).distance(points[i]) < 1e-10);
        }
        const std::vector<Vector> start = curve.evaluate(curve.domain_start(), 2);
        const std::vector<Vector> end = curve.evaluate(curve.domain_end(), 2);
        MINI_CHECK((start[1] - end[1]).magnitude() < 1e-10);
        MINI_CHECK((start[2] - end[2]).magnitude() < 1e-10);
    }

    MINI_TEST("NurbsCurve", "Offset") {
        const std::array<NurbsCurve, 3> curves = {
            NurbsCurve::create(false, 1, {{0, 0, 0}, {2, 0, 0}}),
            NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0}, {2, -1, 0}, {3, 1, 0}}),
            Primitives::circle(0, 0, 0, 2)
        };
        const Vector normal(0, 0, 1);
        const std::filesystem::path folder = std::filesystem::path("build") / "geometry-tests";
        std::filesystem::create_directories(folder);
        for (size_t curve_index = 0; curve_index < curves.size(); ++curve_index) {
            const NurbsCurve& curve = curves[curve_index];
            for (double distance : {-0.1, 0.1}) {
                const NurbsCurve offset = curve.offset(distance, normal, 1e-6);
                MINI_CHECK(offset.is_valid());
                nlohmann::ordered_json data = {
                    {"group", "NurbsCurve"}, {"test", "Offset"}, {"kind", "curve"}, {"distance", distance},
                    {"original", curve.jsondump()}, {"offset", offset.jsondump()}, {"samples", nlohmann::ordered_json::array()}
                };
                for (int i = 0; i <= 100; ++i) {
                    const double parameter = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 100.0;
                    const Vector side = curve.tangent_at(parameter).cross(normal).normalized();
                    const Point expected = curve.point_at(parameter) + side * distance;
                    MINI_CHECK(offset.point_at(parameter).distance(expected) < 5e-6);
                    const Point point = curve.point_at(parameter);
                    const Point displaced = offset.point_at(parameter);
                    data["samples"].push_back({point[0], point[1], point[2], displaced[0], displaced[1], displaced[2]});
                }
                const std::string name = "curve_offset_" + std::to_string(curve_index) + (distance < 0 ? "_negative" : "_positive");
                std::ofstream(folder / (name + ".offset.json")) << data.dump(2);
            }
        }
    }

    MINI_TEST("NurbsCurve", "Constructor") {
        // using session_cpp::Arrowhead;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);

        const std::string cstr = curve.str();
        const std::string crepr = curve.repr();

        const NurbsCurve ccopy = curve;
        const NurbsCurve cother = NurbsCurve::create(false, 2, points);

        NurbsCurve carrow = curve;
        carrow.arrowhead = Arrowhead::BOTH;
        const NurbsCurve carrowcopy = carrow;

        MINI_CHECK(curve.is_valid());
        MINI_CHECK(curve.cv_count() == 4);
        MINI_CHECK(curve.degree() == 2);
        MINI_CHECK(curve.order() == 3);
        MINI_CHECK(curve.name == "my_nurbscurve");
        MINI_CHECK(!curve.guid().empty());
        MINI_CHECK(cstr == "NurbsCurve(name=my_nurbscurve, degree=2, cvs=4)");
        MINI_CHECK(crepr.find("name=my_nurbscurve") != std::string::npos);
        MINI_CHECK(ccopy.cv_count() == curve.cv_count());
        MINI_CHECK(ccopy.guid() != curve.guid());
        MINI_CHECK(ccopy == curve);
        MINI_CHECK(cother != curve);
        MINI_CHECK(curve.arrowhead == Arrowhead::NONE && carrowcopy == carrow && carrow != curve);
    }

    MINI_TEST("NurbsCurve", "Create Interpolated") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::CurveNurbsKnotStyle;
        // using session_cpp::CurveInterpStyle;

        const std::vector<Point> points = {
            Point(14, 9, 0),
            Point(21, 22, 0),
            Point(26, 10, 0),
            Point(35, 19, 0),
            Point(41, 13, 0)
        };

        const NurbsCurve c = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Chord);

        MINI_CHECK(c.is_valid());
        MINI_CHECK(c.degree() == 3);
        MINI_CHECK(c.order() == 4);
        MINI_CHECK(c.cv_count() == 7);
        MINI_CHECK(!c.is_rational());
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_start()), points[0]));
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_end()), points[4]));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(0), points[0]));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(6), points[4]));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(1), Point(15.342776949, 13.734888836, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(3), Point(24.678472471, 0.354555126, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(5), Point(39.626394361, 15.472490151, 0.0)));

        const NurbsCurve co = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Chord, CurveInterpStyle::Occt);

        MINI_CHECK(co.cv_count() == 7);
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(0), points[0]));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(6), points[4]));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(1), Point(17.3526678158, 24.4472657919, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(3), Point(24.7854378511, 2.1457823679, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(5), Point(39.1865250566, 18.5349257754, 0.0)));

        const std::vector<Point> closed_pts = {
            Point(4, 20, 0),
            Point(-2, 20, 0),
            Point(-2, 25, 0),
            Point(-3, 28, 0),
            Point(-10, 28, 0),
            Point(-10, 21, 0),
            Point(-13, 16, 0),
            Point(-8, 14, 0),
            Point(-6, 11, 0),
            Point(0, 15, 0)
        };

        const NurbsCurve cp = NurbsCurve::create_interpolated(closed_pts, CurveNurbsKnotStyle::ChordPeriodic);

        MINI_CHECK(cp.is_valid());
        MINI_CHECK(cp.degree() == 3);
        MINI_CHECK(cp.cv_count() == 13);
        MINI_CHECK(cp.is_closed());
    }

    MINI_TEST("NurbsCurve", "Create From Parameters") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> p4 = {
            Point(0, 0, 0),
            Point(3, 6, 0),
            Point(6, -3, 3),
            Point(10, 0, 0)
        };

        const NurbsCurve c = NurbsCurve::create_from_parameters(
            p4,
            {1.0, 1.0, 1.0, 1.0},
            {0.0, 1.0},
            {4, 4},
            3
        );

        MINI_CHECK(c.is_valid());
        MINI_CHECK(c.degree() == 3);
        MINI_CHECK(c.cv_count() == 4);
        MINI_CHECK(!c.is_rational());
        MINI_CHECK(std::abs(c.domain_start() - 0.0) < 1e-12 && std::abs(c.domain_end() - 1.0) < 1e-12);
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(0), Point(0, 0, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(3), Point(10, 0, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(0.5), Point(4.625, 1.125, 1.125)));

        const double w = 0.5 * std::sqrt(2.0);
        const std::vector<Point> cpts = {
            Point(0, -1, 0),
            Point(-1, -1, 0),
            Point(-1, 0, 0),
            Point(-1, 1, 0),
            Point(0, 1, 0),
            Point(1, 1, 0),
            Point(1, 0, 0),
            Point(1, -1, 0),
            Point(0, -1, 0)
        };

        const NurbsCurve circle = NurbsCurve::create_from_parameters(
            cpts,
            {1, w, 1, w, 1, w, 1, w, 1},
            {0.0, 0.25, 0.5, 0.75, 1.0},
            {3, 2, 2, 2, 3},
            2
        );

        MINI_CHECK(circle.is_valid());
        MINI_CHECK(circle.degree() == 2);
        MINI_CHECK(circle.cv_count() == 9);
        MINI_CHECK(circle.is_rational());
        MINI_CHECK(TOLERANCE.is_point_close(circle.point_at(0.5), Point(0, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(circle.point_at(0.125), Point(-w, -w, 0)));

        for (int k = 0; k <= 16; k++) {
            const Point pp = circle.point_at(k / 16.0);

            MINI_CHECK(std::abs(std::sqrt(pp[0] * pp[0] + pp[1] * pp[1]) - 1.0) < 1e-9);
        }
    }

    MINI_TEST("NurbsCurve", "Create Fitted") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        std::vector<Point> pts;

        for (int i = 0; i <= 20; i++) {
            const double t = i * 2.0 * Tolerance::PI / 20.0;
            pts.push_back(Point(t, 3.0 * std::sin(t), 0.0));
        }

        const NurbsCurve c = NurbsCurve::create_fitted(pts, 8, 3, false);

        MINI_CHECK(c.is_valid());
        MINI_CHECK(c.degree() == 3);
        MINI_CHECK(c.cv_count() == 8);
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_start()), pts[0]));
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(c.domain_end()), pts[20]));

        std::vector<Point> cpts;

        for (int i = 0; i < 24; i++) {
            const double a = i * 2.0 * Tolerance::PI / 24.0;
            cpts.push_back(Point(std::cos(a), std::sin(a), 0.0));
        }

        const NurbsCurve cp = NurbsCurve::create_fitted(cpts, 10, 3, true);

        MINI_CHECK(cp.is_valid());
        MINI_CHECK(cp.is_closed());
        MINI_CHECK(cp.cv_count() == 13);
    }

    MINI_TEST("NurbsCurve", "Join") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Primitives;

        const NurbsCurve arc1 = Primitives::arc(Point(-1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0), Point(1.0, 0.0, 0.0));
        NurbsCurve arc2 = Primitives::arc(Point(1.0, 0.0, 0.0), Point(1.5, -1.0, 0.0), Point(1.0, -2.0, 0.0));
        const std::vector<Point> pts = {
            Point(1.0, -2.0, 0.0),
            Point(-1.0, 0.0, 0.0)
        };
        const NurbsCurve line = NurbsCurve::create(false, 1, pts);
        arc2.reverse();

        const std::vector<NurbsCurve> joined = NurbsCurve::join({line, arc1, arc2});

        MINI_CHECK(joined.size() == 1);
        MINI_CHECK(joined[0].is_valid());
        MINI_CHECK(joined[0].is_closed());
        MINI_CHECK(joined[0].degree() == 2);
        MINI_CHECK(joined[0].cv_count() == 7);

        const NurbsCurve l1 = NurbsCurve::create(false, 1, {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0)});
        const NurbsCurve l2 = NurbsCurve::create(false, 1, {Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
        const NurbsCurve l3 = NurbsCurve::create(false, 1, {Point(9.0, 9.0, 0.0), Point(8.0, 8.0, 0.0)});

        const std::vector<NurbsCurve> separate = NurbsCurve::join({l1, l3, l2});

        MINI_CHECK(separate.size() == 2);
        MINI_CHECK(separate[0].cv_count() == 3);
        MINI_CHECK(std::fabs(separate[0].length() - 2.0) < 1e-9);
    }

    MINI_TEST("NurbsCurve", "Attributes") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Plane;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        const bool is_valid = curve.is_valid();
        const bool is_valid_nurbsknot_vector = curve.is_valid_nurbsknot_vector();
        const bool is_clamped_start = curve.is_clamped(0);
        const bool is_clamped_end = curve.is_clamped(1);
        const bool is_clamped_both = curve.is_clamped(2);

        MINI_CHECK(is_valid);
        MINI_CHECK(is_valid_nurbsknot_vector);
        MINI_CHECK(is_clamped_start && is_clamped_end && is_clamped_both);

        const bool is_rational = curve.is_rational();
        const bool closed = curve.is_closed();
        const bool periodic = curve.is_periodic();
        const bool linear = curve.is_linear();
        const bool planar = curve.is_planar();
        const bool arc = curve.is_arc();
        const Plane plane = Plane::xy_plane();
        const bool on_plane = curve.is_in_plane(plane);
        const bool is_open = curve.is_natural();
        const bool is_polyline = curve.is_polyline();
        const bool is_singular = curve.is_singular();
        const bool is_duplicate = curve.is_duplicate(curve, false);
        const bool is_continuous = curve.is_continuous(1, curve.domain_middle());

        MINI_CHECK(!is_rational);
        MINI_CHECK(!closed);
        MINI_CHECK(!periodic);
        MINI_CHECK(!linear);
        MINI_CHECK(planar);
        MINI_CHECK(!arc);
        MINI_CHECK(on_plane);
        MINI_CHECK(!is_open);
        MINI_CHECK(!is_polyline);
        MINI_CHECK(!is_singular);
        MINI_CHECK(is_duplicate);
        MINI_CHECK(is_continuous);

        NurbsCurve copy_curve = curve;
        const Point before_pt = copy_curve.point_at(1.5);
        copy_curve.insert_nurbsknot(1.5, 1);

        MINI_CHECK(TOLERANCE.is_point_close(before_pt, copy_curve.point_at(1.5)));

        const double greville0 = curve.greville_abcissa(0);
        const std::vector<double> greville = curve.get_greville_abcissae();

        MINI_CHECK(TOLERANCE.is_close(greville0, 0.0));
        MINI_CHECK(greville.size() == 4);
        MINI_CHECK(TOLERANCE.is_close(greville[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(greville[1], 0.879872167739067));
        MINI_CHECK(TOLERANCE.is_close(greville[2], 2.639616503217201));
        MINI_CHECK(TOLERANCE.is_close(greville[3], 3.519488670956267));

        const int dimension = curve.dimension();
        const int degree = curve.degree();
        const int order = curve.order();
        const int cv_count = curve.cv_count();
        const int cv_size = curve.cv_size();
        const int nurbsknot_count = curve.nurbsknot_count();
        const int span_count = curve.span_count();

        MINI_CHECK(dimension == 3);
        MINI_CHECK(degree == 2);
        MINI_CHECK(order == 3);
        MINI_CHECK(cv_count == 4);
        MINI_CHECK(cv_size == 3);
        MINI_CHECK(nurbsknot_count == 5);
        MINI_CHECK(span_count == 2);

        const double* p = curve.cv(1);
        const Point cv_point = curve.get_cv(1);
        const std::tuple<double, double, double, double> cv4 = curve.get_cv_4d(1);

        MINI_CHECK(p[0] == 1.0 && p[1] == 1.0 && p[2] == 0.0);
        MINI_CHECK(cv_point == Point(1.0, 1.0, 0.0));
        MINI_CHECK(std::get<0>(cv4) == 1.0 && std::get<1>(cv4) == 1.0 && std::get<2>(cv4) == 0.0 && std::get<3>(cv4) == 1.0);

        curve.set_cv(2, Point(2.0, 0.0, 0.5));

        MINI_CHECK(curve.get_cv(2)[0] == 2.0);
        MINI_CHECK(curve.get_cv(2)[1] == 0.0);
        MINI_CHECK(curve.get_cv(2)[2] == 0.5);

        curve.set_cv_4d(2, 2.0, 0.0, 0.5, 0.707);

        const std::tuple<double, double, double, double> cv4_weighted = curve.get_cv_4d(2);
        const double weight = curve.weight(2);

        MINI_CHECK(std::get<0>(cv4_weighted) == 2.0 && std::get<1>(cv4_weighted) == 0.0 && std::get<2>(cv4_weighted) == 0.5 && std::get<3>(cv4_weighted) == 0.707);
        MINI_CHECK(weight == 0.707);

        curve.set_weight(2, 0.5);

        MINI_CHECK(curve.weight(2) == 0.5);

        const double nurbsknot3 = curve.nurbsknot(3);
        const double end_nurbsknot = curve.nurbsknot(4);
        curve.set_nurbsknot(4, end_nurbsknot);

        MINI_CHECK(TOLERANCE.is_close(nurbsknot3, 3.519488670956267));
        MINI_CHECK(TOLERANCE.is_close(curve.nurbsknot(4), end_nurbsknot));

        const int m0 = curve.nurbsknot_multiplicity(0);
        const int m1 = curve.nurbsknot_multiplicity(1);
        const int m2 = curve.nurbsknot_multiplicity(2);
        const int m3 = curve.nurbsknot_multiplicity(3);
        const int m4 = curve.nurbsknot_multiplicity(4);
        const double superfluous_nurbsknot = curve.superfluous_nurbsknot(1);

        MINI_CHECK(m0 == 2);
        MINI_CHECK(m1 == 2);
        MINI_CHECK(m2 == 1);
        MINI_CHECK(m3 == 2);
        MINI_CHECK(m4 == 2);
        MINI_CHECK(TOLERANCE.is_close(superfluous_nurbsknot, 7.038977341912535));

        const double* nurbsknots = curve.nurbsknot_array();
        const double k0 = nurbsknots[0];
        const std::vector<double> nurbsknot_vector = curve.get_nurbsknots();
        const double* cvs = curve.cv_array();
        const double cx0 = cvs[0];

        MINI_CHECK(k0 == 0.0);
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[2], 1.759744335478134));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[3], 3.519488670956267));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[4], 3.519488670956267));
        MINI_CHECK(cx0 == 0.0);

        const std::pair<double, double> interval = curve.domain();
        const double start = curve.domain_start();
        const double middle = curve.domain_middle();
        const double end = curve.domain_end();

        MINI_CHECK(TOLERANCE.is_close(interval.first, 0.0) && TOLERANCE.is_close(interval.second, 3.519488670956267));
        MINI_CHECK(TOLERANCE.is_close(start, 0.0));
        MINI_CHECK(TOLERANCE.is_close(middle, 1.759744335478134));
        MINI_CHECK(TOLERANCE.is_close(end, 3.519488670956267));

        curve.set_domain(0.0, 1.0);

        const std::vector<double> intervals = curve.get_span_vector();
        const std::pair<bool, double> discontinuity = curve.get_next_discontinuity(2, curve.domain_start(), curve.domain_end());

        MINI_CHECK(curve.domain_start() == 0.0);
        MINI_CHECK(curve.domain_middle() == 0.5);
        MINI_CHECK(curve.domain_end() == 1.0);
        MINI_CHECK(TOLERANCE.is_close(intervals[0], 0.0) && TOLERANCE.is_close(intervals[1], 0.5) && TOLERANCE.is_close(intervals[2], 1.0));
        MINI_CHECK(discontinuity.first && TOLERANCE.is_close(discontinuity.second, 0.5));
    }

    MINI_TEST("NurbsCurve", "Conversions") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        const NurbsCurve curve = NurbsCurve::create(false, 2, points);
        const std::vector<Point> adaptive_pts = curve.to_polyline_adaptive(0.1, 0.0, 0.0).first;
        const std::vector<Point> div_pts = curve.divide_by_count(10, true).first;
        const std::vector<Point> len_pts = curve.divide_by_length(0.5).first;

        MINI_CHECK(adaptive_pts.size() == 27);
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[13], Point(2.0, 0.5, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[26], Point(4.0, 0.0, 0.0)));
        MINI_CHECK(div_pts.size() == 10);
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[0], Point(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[1], Point(0.328571042584482, 0.598213549686919, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[2], Point(0.740745014933581, 1.140321305195565, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[3], Point(1.338524108277686, 1.232715936161493, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[4], Point(1.712929651058201, 0.664818770483133, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[5], Point(2.287070348941800, 0.664818770483133, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[6], Point(2.661475891722314, 1.232715936161493, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[7], Point(3.259254985066419, 1.140321305195565, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[8], Point(3.671428957415518, 0.598213549686919, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[9], Point(4.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(len_pts.size() == 13);
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[6], Point(1.928691289557961, 0.510169864369813, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[12], Point(3.934494398161146, 0.128829840160734, 0.000000000000000)));
    }

    MINI_TEST("NurbsCurve", "Evaluation") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Vector;
        // using session_cpp::Plane;

        const std::vector<Point> points = {
            Point(1.957614, 1.140253, -0.191281),
            Point(0.912252, 1.886721, 0),
            Point(3.089381, 2.701879, -0.696251),
            Point(5.015145, 1.189141, 0.35799),
            Point(1.854155, 0.514663, 0.347694),
            Point(3.309532, 1.328666, 0),
            Point(3.544072, 2.194233, 0.696217),
            Point(2.903513, 2.091287, 0.696217),
            Point(2.752484, 1.45432, 0),
            Point(2.406227, 1.288248, 0),
            Point(2.15032, 1.868606, 0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        const double length = curve.length();
        const Point point_at = curve.point_at(0.5);
        const std::vector<Vector> derivatives = curve.evaluate(0.5, 2);
        const Vector tangent = curve.tangent_at(0.5);

        MINI_CHECK(TOLERANCE.is_close(length, 11.3010276326));
        MINI_CHECK(TOLERANCE.is_close(point_at[0], 1.463452399002842));
        MINI_CHECK(TOLERANCE.is_close(point_at[1], 1.680997287875395));
        MINI_CHECK(TOLERANCE.is_close(point_at[2], -0.124474565996108));
        MINI_CHECK(derivatives.size() == 3);
        MINI_CHECK(TOLERANCE.is_close(derivatives[0][0], 1.463452399002842));
        MINI_CHECK(TOLERANCE.is_close(derivatives[0][1], 1.680997287875395));
        MINI_CHECK(TOLERANCE.is_close(derivatives[0][2], -0.124474565996108));
        MINI_CHECK(TOLERANCE.is_close(derivatives[1][0], -0.311619416021204));
        MINI_CHECK(TOLERANCE.is_close(derivatives[1][1], 0.974021205471335));
        MINI_CHECK(TOLERANCE.is_close(derivatives[1][2], -0.037441955449586));
        MINI_CHECK(TOLERANCE.is_close(derivatives[2][0], 2.706815143892446));
        MINI_CHECK(TOLERANCE.is_close(derivatives[2][1], -0.429869481117820));
        MINI_CHECK(TOLERANCE.is_close(derivatives[2][2], -0.684219293829483));
        MINI_CHECK(TOLERANCE.is_close(tangent[0], -0.304511941745027));
        MINI_CHECK(TOLERANCE.is_close(tangent[1], 0.951805546117607));
        MINI_CHECK(TOLERANCE.is_close(tangent[2], -0.036587972264639));

        const Plane f = curve.plane_at(0.5, true);

        MINI_CHECK(TOLERANCE.is_close(f.origin()[0], 3.156927375000000));
        MINI_CHECK(TOLERANCE.is_close(f.origin()[1], 1.335111500000000));
        MINI_CHECK(TOLERANCE.is_close(f.origin()[2], 0.130488875000000));
        MINI_CHECK(TOLERANCE.is_close(f.x_axis()[0], 0.701806140304030));
        MINI_CHECK(TOLERANCE.is_close(f.x_axis()[1], 0.697509131556264));
        MINI_CHECK(TOLERANCE.is_close(f.x_axis()[2], 0.144738221721788));
        MINI_CHECK(TOLERANCE.is_close(f.y_axis()[0], -0.513930504714161));
        MINI_CHECK(TOLERANCE.is_close(f.y_axis()[1], 0.355053088776962));
        MINI_CHECK(TOLERANCE.is_close(f.y_axis()[2], 0.780905077761815));
        MINI_CHECK(TOLERANCE.is_close(f.z_axis()[0], 0.493298669931115));
        MINI_CHECK(TOLERANCE.is_close(f.z_axis()[1], -0.622429365908747));
        MINI_CHECK(TOLERANCE.is_close(f.z_axis()[2], 0.607649657861031));

        MINI_CHECK(!curve.plane_at(-0.1, true).is_valid());
        MINI_CHECK(!curve.plane_at(1.1, true).is_valid());
        MINI_CHECK(curve.plane_at(curve.domain_start(), false).is_valid());
        MINI_CHECK(curve.plane_at(curve.domain_end(), false).is_valid());
        MINI_CHECK(!curve.plane_at(curve.domain_start() - 0.1, false).is_valid());

        const Plane pf = curve.perpendicular_plane_at(0.5, true);

        MINI_CHECK(TOLERANCE.is_point_close(pf.origin(), Point(3.156927375000000, 1.335111500000000, 0.130488875000000)));
        MINI_CHECK(TOLERANCE.is_vector_close(pf.x_axis(), Vector(0.632703652329189, -0.703685357647999, 0.323284713157168)));
        MINI_CHECK(TOLERANCE.is_vector_close(pf.y_axis(), Vector(0.327344206830723, -0.135306795251661, -0.935167279909370)));
        MINI_CHECK(TOLERANCE.is_vector_close(pf.z_axis(), Vector(0.701806140314880, 0.697509131546342, 0.144738221716994)));
        MINI_CHECK(!curve.perpendicular_plane_at(-0.1, true).is_valid());
        MINI_CHECK(!curve.perpendicular_plane_at(1.1, true).is_valid());
        MINI_CHECK(curve.perpendicular_plane_at(curve.domain_start(), false).is_valid());
        MINI_CHECK(curve.perpendicular_plane_at(curve.domain_end(), false).is_valid());
        MINI_CHECK(!curve.perpendicular_plane_at(curve.domain_start() - 0.1, false).is_valid());

        const std::vector<Plane> frames = curve.get_perpendicular_planes(4);

        MINI_CHECK(frames.size() == 5);
        MINI_CHECK(TOLERANCE.is_point_close(frames[0].origin(), Point(1.957614, 1.140253, -0.191281)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[0].x_axis(), Vector(0.532767753269467, 0.809398954921174, -0.247046256496055)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[0].y_axis(), Vector(-0.261213903019039, -0.120386647366337, -0.957744408496052)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[0].z_axis(), Vector(-0.804938393882267, 0.574787253606414, 0.147288136473484)));
        MINI_CHECK(TOLERANCE.is_point_close(frames[2].origin(), Point(3.676075526826347, 0.909844934818883, 0.350126174959864)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[2].x_axis(), Vector(-0.188216290968568, 0.616421099680873, -0.764591168980037)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[2].y_axis(), Vector(0.183061244251456, -0.742843014267013, -0.643950958542502)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[2].z_axis(), Vector(-0.964916166652391, -0.261169071719684, 0.026972343426817)));
        MINI_CHECK(TOLERANCE.is_point_close(frames[4].origin(), Point(2.150320000000000, 1.868606000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[4].x_axis(), Vector(0.183261707646767, 0.080808692310795, 0.979737261594868)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[4].y_axis(), Vector(0.896455027441244, 0.395289116385372, -0.200287039627106)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[4].z_axis(), Vector(-0.403464410184726, 0.914995338629816, 0.000000000000000)));

        const Point p0 = curve.point_at_start();
        const Point p1 = curve.point_at_middle();
        const Point p2 = curve.point_at_end();

        MINI_CHECK(TOLERANCE.is_close(p0[0], 1.957614));
        MINI_CHECK(TOLERANCE.is_close(p0[1], 1.140253));
        MINI_CHECK(TOLERANCE.is_close(p0[2], -0.191281));
        MINI_CHECK(TOLERANCE.is_close(p1[0], 3.156927375));
        MINI_CHECK(TOLERANCE.is_close(p1[1], 1.3351115));
        MINI_CHECK(TOLERANCE.is_close(p1[2], 0.130488875));
        MINI_CHECK(TOLERANCE.is_close(p2[0], 2.15032));
        MINI_CHECK(TOLERANCE.is_close(p2[1], 1.868606));
        MINI_CHECK(TOLERANCE.is_close(p2[2], 0.0));

        curve.set_start_point(Point(1.957614, 1.140253, 2.0));
        curve.set_end_point(Point(2.15032, 1.868606, 2.0));

        MINI_CHECK(TOLERANCE.is_close(curve.point_at_start()[2], 2.0));
        MINI_CHECK(TOLERANCE.is_close(curve.point_at_end()[2], 2.0));
    }

    MINI_TEST("NurbsCurve", "Modifications") {
        // using session_cpp::Arrowhead;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        NurbsCurve curve_reversed = curve;
        curve_reversed.arrowhead = Arrowhead::START;
        curve_reversed.reverse();

        MINI_CHECK(TOLERANCE.is_point_close(curve_reversed.point_at_start(), curve.point_at_end()));
        MINI_CHECK(curve_reversed.arrowhead == Arrowhead::END);

        curve.swap_coordinates(0, 1);

        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(0), Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(1), Point(2.0, 1.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(2), Point(0.0, 2.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(3), Point(2.0, 3.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(4), Point(0.0, 4.0, 0.0)));

        NurbsCurve ct = curve;
        const double a = ct.domain_start() + (ct.domain_end() - ct.domain_start()) / 3.0;
        const double b = ct.domain_start() + 2.0 * (ct.domain_end() - ct.domain_start()) / 3.0;
        ct.trim(a, b);

        MINI_CHECK(ct.length() < curve.length());

        const double split_t = curve.domain_middle();
        const std::pair<NurbsCurve, NurbsCurve> halves = curve.split(split_t);

        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(split_t), halves.first.point_at_end()));
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(split_t), halves.second.point_at_start()));

        NurbsCurve curve_arrow = curve;
        curve_arrow.arrowhead = Arrowhead::BOTH;
        const std::pair<NurbsCurve, NurbsCurve> arrow_halves = curve_arrow.split(split_t);

        MINI_CHECK(arrow_halves.first.arrowhead == Arrowhead::START);
        MINI_CHECK(arrow_halves.second.arrowhead == Arrowhead::END);
        const std::vector<NurbsCurve> arrow_joined = NurbsCurve::join({arrow_halves.first, arrow_halves.second});

        MINI_CHECK(arrow_joined[0].arrowhead == Arrowhead::BOTH);

        NurbsCurve curve_extended = curve;
        curve_extended.extend(curve.domain_start() - 0.5, curve.domain_end() + 0.5);

        MINI_CHECK(curve_extended.length() > curve.length());

        NurbsCurve curve_rational = curve;
        const double original_length = curve.length();
        curve_rational.to_rational();
        curve_rational.set_weight(2, 10);

        MINI_CHECK(curve_rational.length() != original_length);

        curve_rational.to_non_rational(true);

        MINI_CHECK(curve_rational.length() == original_length);

        const std::vector<Point> points_open = points;
        NurbsCurve curve_open(3, false, 3, 5);

        for (int i = 0; i < 5; ++i)
            curve_open.set_cv(i, points_open[i]);

        for (int i = 0; i < curve_open.nurbsknot_count(); ++i)
            curve_open.set_nurbsknot(i, i * 1.0);

        curve_open.clamp_end(2);

        const std::vector<double> nurbsknots = curve_open.get_nurbsknots();

        MINI_CHECK(TOLERANCE.is_close(nurbsknots[0], nurbsknots[1]));
        MINI_CHECK(TOLERANCE.is_close(nurbsknots[nurbsknots.size() - 2], nurbsknots[nurbsknots.size() - 1]));

        NurbsCurve raised = curve;
        raised.increase_degree(3);

        MINI_CHECK(curve.degree() != raised.degree());
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at_middle(), raised.point_at_middle()));

        const std::vector<Point> closed_pts = {
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(-1.0, 0.0, 0.0),
            Point(0.0, -1.0, 0.0)
        };

        NurbsCurve c = NurbsCurve::create(true, 2, closed_pts);
        const Point expected_start = c.point_at(c.domain_middle());
        c.change_closed_curve_seam(c.domain_middle());

        MINI_CHECK(TOLERANCE.is_point_close(c.point_at_start(), expected_start));
    }

    MINI_TEST("NurbsCurve", "Transformations") {
        // using session_cpp::Arrowhead;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Xform;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve1 = NurbsCurve::create(false, 2, points);
        curve1.arrowhead = Arrowhead::BOTH;
        const Xform curve1_xf = Xform::translation(0.0, 0.0, 1.0);
        curve1.transform(curve1_xf);

        NurbsCurve curve2 = NurbsCurve::create(false, 2, points);
        Xform x = Xform::translation(0.0, 0.0, 1.0);
        curve2.transform(x);

        const NurbsCurve curve3 = NurbsCurve::create(false, 2, points);
        const Xform curve3_xf = Xform::translation(0.0, 0.0, 10.0);
        const NurbsCurve curve3_transformed = curve3.transformed(curve3_xf);

        const NurbsCurve curve4 = NurbsCurve::create(false, 2, points);
        x = Xform::translation(0.0, 0.0, 10.0);
        const NurbsCurve curve4_transformed = curve4.transformed(x);

        MINI_CHECK(curve1.cv(0)[2] == 1.0 && curve1.arrowhead == Arrowhead::BOTH);
        MINI_CHECK(curve2.cv(0)[2] == 1.0);
        MINI_CHECK(curve3_transformed.cv(0)[2] == 10.0);
        MINI_CHECK(curve4_transformed.cv(0)[2] == 10.0);
    }

    MINI_TEST("NurbsCurve", "Json Roundtrip") {
        // using session_cpp::Arrowhead;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.arrowhead = Arrowhead::END;
        const std::string guid = curve.guid();
        const std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbscurve.json").string();
        curve.file_json_dump(filename);

        const nlohmann::ordered_json json = curve.jsondump();
        const NurbsCurve loaded_json = NurbsCurve::jsonload(json);
        const NurbsCurve loaded_json_string = NurbsCurve::file_json_loads(curve.file_json_dumps());
        const NurbsCurve loaded_from_file = NurbsCurve::file_json_load(filename);

        MINI_CHECK(loaded_json == curve);
        MINI_CHECK(loaded_json_string == curve);
        MINI_CHECK(loaded_from_file == curve);
        MINI_CHECK(loaded_from_file.guid() == guid);
        MINI_CHECK(loaded_from_file.arrowhead == Arrowhead::END);
        MINI_CHECK(curve.file_json_dumps().find("\"arrowhead\":\"end\"") != std::string::npos);
    }

    MINI_TEST("NurbsCurve", "Protobuf Roundtrip") {
        // using session_cpp::Arrowhead;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.arrowhead = Arrowhead::END;
        const std::string guid = curve.guid();
        const std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbscurve.bin").string();
        curve.pb_dump(filename);

        const NurbsCurve loaded_proto_string = NurbsCurve::pb_loads(curve.pb_dumps());
        const NurbsCurve loaded = NurbsCurve::pb_load(filename);
        const NurbsCurve converted = NurbsCurve::from_proto(curve.to_proto());

        MINI_CHECK(loaded_proto_string == curve);
        MINI_CHECK(loaded == curve);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(loaded.arrowhead == Arrowhead::END);
        MINI_CHECK(converted == curve);
        MINI_CHECK(converted.guid() == guid);
    }

    MINI_TEST("NurbsCurve", "Curvature") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Primitives;

        const double R = 2.0;
        const NurbsCurve circle = Primitives::circle(0, 0, 0, R);
        const double t0 = circle.domain_start();
        const double t1 = circle.domain_end();

        for (int i = 0; i <= 8; ++i) {
            const double t = t0 + (t1 - t0) * i / 8.0;

            MINI_CHECK(std::abs(circle.curvature_at(t) - 1.0 / R) < 1e-6);
        }

        const std::vector<Point> line_pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(2, 0, 0),
            Point(3, 0, 0)
        };

        const NurbsCurve line = NurbsCurve::create(false, 1, line_pts);

        MINI_CHECK(line.curvature_at(line.domain_middle()) < 1e-9);
    }

    MINI_TEST("NurbsCurve", "Closest Point") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::Primitives;
        // using session_cpp::CurveNurbsKnotStyle;
        // using session_cpp::CurveInterpStyle;

        const NurbsCurve circle = Primitives::circle(0, 0, 0, 2.0);
        const Point cp = circle.closest_point(Point(5, 0, 0));
        const Point cp2 = circle.closest_point(Point(0, 5, 0));

        MINI_CHECK(std::abs(cp[0] - 2.0) < 1e-5 && std::abs(cp[1]) < 1e-5 && std::abs(cp[2]) < 1e-5);
        MINI_CHECK(std::abs(cp2[0]) < 1e-5 && std::abs(cp2[1] - 2.0) < 1e-5);

        const std::vector<Point> ipts = {
            Point(0, 0, 0),
            Point(3, 0, 2),
            Point(6, 0, -3),
            Point(8, 0, 0)
        };

        const NurbsCurve ic = NurbsCurve::create_interpolated(ipts, CurveNurbsKnotStyle::Chord, CurveInterpStyle::Occt);
        const Point pc = ic.closest_point(Point(2, -1, 0));

        MINI_CHECK(TOLERANCE.is_point_close(pc, Point(0.5808155659, 0.0, 0.9672315271)));

        const std::vector<Point> p0 = {
            Point(0, 0, 0),
            Point(3, 6, 0),
            Point(6, -3, 3),
            Point(10, 0, 0)
        };

        const std::vector<Point> p1 = {
            Point(6, -3, 0),
            Point(3, 1, 0),
            Point(6, 6, 3),
            Point(3, 12, 0)
        };

        const NurbsCurve c0 = NurbsCurve::create_from_parameters(p0, {1, 1, 1, 1}, {0, 1}, {4, 4}, 3);
        const NurbsCurve c1 = NurbsCurve::create_from_parameters(p1, {1, 1, 1, 1}, {0, 1}, {4, 4}, 3);
        const std::pair<double, double> params = c0.closest_parameters_curve(c1);
        const std::pair<Point, Point> closest = c0.closest_points_curve(c1);

        MINI_CHECK(std::abs(params.first - 0.4757682937) < 1e-6 && std::abs(params.second - 0.3366914716) < 1e-6);
        MINI_CHECK(TOLERANCE.is_point_close(closest.first, Point(4.389607399, 1.285537564, 1.067964425)));
        MINI_CHECK(TOLERANCE.is_point_close(closest.second, Point(4.552264625, 1.380381100, 0.676740741)));
    }

    MINI_TEST("NurbsCurve", "Length Repeated Knot") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0, 0, 0),
            Point(1, 2, 0),
            Point(3, 2, 1),
            Point(4, 0, 0),
            Point(6, 1, 2),
            Point(7, 3, 0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 3, points);
        const double length = curve.length();
        curve.insert_nurbsknot(1.5, 2);

        MINI_CHECK(curve.span_count() == 4);
        MINI_CHECK(TOLERANCE.is_close(curve.length(), length));
    }

    MINI_TEST("NurbsCurve", "Insert Knot Keeps Shape") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0, 0, 0),
            Point(1, 2, 0),
            Point(3, 2, 1),
            Point(4, 0, 0),
            Point(6, 1, 2),
            Point(7, 3, 0)
        };

        const NurbsCurve curve = NurbsCurve::create(false, 3, points);
        NurbsCurve inserted = curve;
        const bool ok = inserted.insert_nurbsknot(1.5, 1);

        MINI_CHECK(ok);
        MINI_CHECK(inserted.cv_count() == 7);
        MINI_CHECK(inserted.nurbsknot_count() == inserted.cv_count() + inserted.order() - 2);

        for (int i = 0; i < 5; i++) {
            const double t = curve.domain_end() * i / 4.0;

            MINI_CHECK(TOLERANCE.is_point_close(inserted.point_at(t), curve.point_at(t)));
        }
    }

    MINI_TEST("NurbsCurve", "Insert Knot Periodic Wrap") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(2, 0, 0),
            Point(0, 2, 0),
            Point(-2, 0, 0),
            Point(0, -2, 0)
        };

        NurbsCurve curve = NurbsCurve::create(true, 3, points);
        curve.set_domain(0.0, 1.0);
        NurbsCurve inside = curve;
        NurbsCurve outside = curve;
        NurbsCurve doubled = curve;
        NurbsCurve open_curve = NurbsCurve::create(false, 3, points);
        const bool ok_inside = inside.insert_nurbsknot(0.3, 1);
        const bool ok_outside = outside.insert_nurbsknot(2.3, 1);
        const bool ok_doubled = doubled.insert_nurbsknot(-0.4, 2);
        const bool ok_open = open_curve.insert_nurbsknot(open_curve.domain_end() + 0.5, 1);

        MINI_CHECK(ok_inside);
        MINI_CHECK(ok_outside);
        MINI_CHECK(ok_doubled);
        MINI_CHECK(!ok_open);
        MINI_CHECK(outside.cv_count() == 8);
        MINI_CHECK(doubled.cv_count() == 9);
        MINI_CHECK(doubled.is_valid());
        MINI_CHECK(open_curve.cv_count() == 4);

        for (int i = 0; i < outside.nurbsknot_count(); i++)
            MINI_CHECK(TOLERANCE.is_close(outside.nurbsknot(i), inside.nurbsknot(i)));

        for (int i = 0; i < outside.nurbsknot_count() - 5; i++)
            MINI_CHECK(TOLERANCE.is_close(outside.nurbsknot(i + 5) - outside.nurbsknot(i), 1.0));

        for (int i = 0; i < 3; i++)
            MINI_CHECK(TOLERANCE.is_point_close(outside.get_cv(i), outside.get_cv(i + 5)));

        for (int i = 0; i < 5; i++) {
            const double t = 0.1 + 0.2 * i;

            MINI_CHECK(TOLERANCE.is_point_close(outside.point_at(t), curve.point_at(t)));
            MINI_CHECK(TOLERANCE.is_point_close(doubled.point_at(t), curve.point_at(t)));
        }
    }

    MINI_TEST("NurbsCurve", "Insert Knot Multiplicity Limit") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {
            Point(0, 0, 0),
            Point(1, 2, 0),
            Point(3, 2, 1),
            Point(4, 0, 0),
            Point(6, 1, 2),
            Point(7, 3, 0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 3, points);
        NurbsCurve periodic = NurbsCurve::create(true, 3, points);
        const Point point = curve.point_at(1.5);
        const bool ok_over = curve.insert_nurbsknot(1.5, 4);
        const bool ok_nan = curve.insert_nurbsknot(std::numeric_limits<double>::quiet_NaN(), 1);
        const bool ok_full = curve.insert_nurbsknot(1.5, 3);
        const bool ok_again = curve.insert_nurbsknot(1.5, 3);
        const bool ok_seam = periodic.insert_nurbsknot(periodic.domain_end(), 2);

        MINI_CHECK(!ok_over);
        MINI_CHECK(!ok_nan);
        MINI_CHECK(ok_full);
        MINI_CHECK(ok_again);
        MINI_CHECK(!ok_seam);
        MINI_CHECK(curve.cv_count() == 9);
        MINI_CHECK(curve.nurbsknot_multiplicity(3) == 3);
        MINI_CHECK(periodic.cv_count() == 9);
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(1.5), point));
    }

    MINI_TEST("NurbsCurve", "Span Vector Empty") {
        // using session_cpp::NurbsCurve;

        const NurbsCurve curve;

        MINI_CHECK(curve.get_span_vector().empty());
    }

    MINI_TEST("NurbsCurve", "Periodic Too Few Points") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        NurbsCurve curve;
        const bool ok = curve.create_periodic_uniform(3, 4, {Point(0, 0, 0), Point(1, 0, 0)});

        MINI_CHECK(!ok);
    }

    MINI_TEST("NurbsCurve", "Polyline Adaptive Closed") {
        // using session_cpp::Primitives;
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const NurbsCurve circle = Primitives::circle(0, 0, 0, 2.0);
        const std::pair<std::vector<Point>, std::vector<double>> polyline = circle.to_polyline_adaptive(0.1, 0.0, 0.0);

        MINI_CHECK(polyline.first.size() == 25);
        MINI_CHECK(TOLERANCE.is_point_close(polyline.first.front(), polyline.first.back()));
    }

    MINI_TEST("NurbsCurve", "Circle Length") {
        // using session_cpp::Primitives;
        // using session_cpp::NurbsCurve;

        const NurbsCurve circle = Primitives::circle(0, 0, 0, 2.0);

        MINI_CHECK(std::abs(circle.length() - 4.0 * Tolerance::PI) < 1e-9);
    }

    MINI_TEST("NurbsCurve", "Is Visible Round Trip") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)};
        NurbsCurve hidden = NurbsCurve::create(false, 2, points);

        MINI_CHECK(hidden.is_visible);

        hidden.is_visible = false;
        const NurbsCurve json = NurbsCurve::file_json_loads(hidden.file_json_dumps());
        const NurbsCurve proto = NurbsCurve::pb_loads(hidden.pb_dumps());

        MINI_CHECK(!json.is_visible);
        MINI_CHECK(!proto.is_visible);
    }

    MINI_TEST("NurbsCurve", "Is Locked Round Trip") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;

        const std::vector<Point> points = {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)};
        NurbsCurve locked = NurbsCurve::create(false, 2, points);

        MINI_CHECK(!locked.is_locked);

        locked.is_locked = true;
        const NurbsCurve json = NurbsCurve::file_json_loads(locked.file_json_dumps());
        const NurbsCurve proto = NurbsCurve::pb_loads(locked.pb_dumps());

        MINI_CHECK(json.is_locked);
        MINI_CHECK(proto.is_locked);
    }

    MINI_TEST("NurbsCurve", "Linear Domain Endpoint Normalization") {
        NurbsCurve curve = NurbsCurve::create(false, 1, {{0.1, 0.2, 0}, {0.9, 0.8, 0}});
        MINI_CHECK(curve.set_domain(-1e-9, 3e-9));
        MINI_CHECK(curve.domain_start() == -1e-9 && curve.domain_end() == 3e-9);
        MINI_CHECK(curve.set_domain(0, 1));
        MINI_CHECK(curve.domain_start() == 0 && curve.domain_end() == 1);
        MINI_CHECK(curve.point_at_start().distance({0.1, 0.2, 0}) < 1e-15);
        MINI_CHECK(curve.point_at_end().distance({0.9, 0.8, 0}) < 1e-15);
        MINI_CHECK(curve.point_at_middle().distance({0.5, 0.5, 0}) < 1e-15);
    }

    MINI_TEST("NurbsCurve", "Exact Constant Coordinate Evaluation") {
        NurbsCurve quadratic = NurbsCurve::create(false, 2,
            {{1, 1, 0}, {1, 0.5, 0}, {1, 0, 0}});
        MINI_CHECK(quadratic.set_domain(0, 2));
        const double parameter = 0.9049874901623626;
        const std::vector<Vector> derivatives = quadratic.evaluate(parameter, 3);
        MINI_CHECK(quadratic.point_at(parameter)[0] == 1.0);
        MINI_CHECK(derivatives[0][0] == 1.0);
        for (int i = 1; i <= 3; ++i)
            MINI_CHECK(derivatives[i][0] == 0.0);
        MINI_CHECK(std::abs(derivatives[0][1] - (1 - parameter / 2)) < 1e-15);
        MINI_CHECK(std::abs(derivatives[1][1] + 0.5) < 1e-15);

        NurbsCurve affine = quadratic;
        MINI_CHECK(affine.set_domain(-3, 5));
        for (double t : {-3.0, -0.75, 5.0}) {
            const std::vector<Vector> values = affine.evaluate(t, 4);
            MINI_CHECK(affine.point_at(t)[0] == 1.0);
            MINI_CHECK(values[0][0] == 1.0);
            for (int k = 1; k <= 4; ++k)
                MINI_CHECK(values[k][0] == 0.0);
        }
        NurbsCurve local = NurbsCurve::create(false, 1,
            {{1, 0, 0}, {1, 1, 0}, {2, 2, 0}});
        MINI_CHECK(local.set_domain(0, 2));
        MINI_CHECK(local.evaluate(0.5, 1)[0][0] == 1.0);
        MINI_CHECK(local.evaluate(0.5, 1)[1][0] == 0.0);
        MINI_CHECK(std::abs(local.evaluate(1.5, 1)[0][0] - 1.5) < 1e-15);
        MINI_CHECK(std::abs(local.evaluate(1.5, 1)[1][0] - 1.0) < 1e-15);

        const double rhino_u = 44.0474124723583;
        NurbsCurve rhino_span = NurbsCurve::create(false, 1,
            {{rhino_u, 0, 0}, {rhino_u, 0.36656699268305354, 0}});
        const double rhino_parameter = 0.1222977958224985;
        MINI_CHECK(rhino_span.evaluate(rhino_parameter, 1)[0][0] == rhino_u);
        MINI_CHECK(rhino_span.point_at(rhino_parameter)[0] == rhino_u);

        const NurbsCurve negative = NurbsCurve::create(false, 1,
            {{-rhino_u, 0, 0}, {-rhino_u, 1, 0}});
        MINI_CHECK(negative.evaluate(rhino_parameter, 1)[0][0] == -rhino_u);
        MINI_CHECK(negative.evaluate(rhino_parameter, 1)[1][0] == 0.0);
        const NurbsCurve rational = NurbsCurve::create_from_parameters(
            {{7, 0, 0}, {3.5, 1, 0}}, {1, 2}, {0, 1}, {2, 2}, 1);
        MINI_CHECK(std::abs(rational.point_at(0.5)[0] - 14.0 / 3) < 1e-14);
        MINI_CHECK(std::abs(rational.evaluate(0.5, 1)[0][0] - 14.0 / 3) < 1e-14);

        NurbsCurve varying = NurbsCurve::create(false, 1, {{1, 0, 0}, {2, 1, 0}});
        MINI_CHECK(varying.set_domain(0, 1));
        MINI_CHECK(std::abs(varying.point_at(0.25)[0] - 1.25) < 1e-15);
        MINI_CHECK(std::abs(varying.evaluate(0.25, 1)[1][0] - 1.0) < 1e-15);
    }

    MINI_TEST("NurbsCurve", "Linear Span Preserves Convex Coordinate Bounds") {
        const double a = 11.4332773479222, b = 11.433277347922198;
        NurbsCurve curve = NurbsCurve::create(false, 1, {{a, 0, 0}, {b, .37224401342001423, 0}}, 2);
        MINI_CHECK(curve.set_domain(0, 1));
        const double witness = .06718439880608412;
        MINI_CHECK(curve.point_at(witness)[0] == a);
        MINI_CHECK(curve.evaluate(witness, 1)[0][0] == a);
        for (double t : {0.0, witness, .25, .5, .75, 1.0}) {
            const std::vector<Vector> values = curve.evaluate(t, 3);
            MINI_CHECK(values.size() == 4);
            MINI_CHECK(curve.point_at(t)[0] >= b && curve.point_at(t)[0] <= a);
            MINI_CHECK(values[0][0] >= b && values[0][0] <= a);
            MINI_CHECK(values[1][0] == b - a);
            MINI_CHECK(values[1][1] == .37224401342001423);
            MINI_CHECK(values[2].magnitude() == 0 && values[3].magnitude() == 0);
        }
        MINI_CHECK(curve.reverse());
        MINI_CHECK(curve.set_domain(-3, 5));
        for (double t : {-3.0, -.75, 1.0, 5.0}) {
            const std::vector<Vector> values = curve.evaluate(t, 1);
            MINI_CHECK(values[0][0] >= b && values[0][0] <= a);
            MINI_CHECK(values[1][0] == (a - b) / 8);
        }
        const NurbsCurve wide = NurbsCurve::create_from_parameters(
            {{-1e308, 0, 0}, {1e308, 0, 0}}, {1, 1}, {0, 1e308}, {2, 2}, 1);
        MINI_CHECK(wide.is_valid() && !wide.is_rational());
        MINI_CHECK(wide.point_at(5e307)[0] == 0);
        MINI_CHECK(std::abs(wide.evaluate(5e307, 1)[1][0] - 2) < 1e-15);
        NurbsCurve scalar = NurbsCurve::create(false, 1, {{3, 0, 0}, {5, 0, 0}}, 1);
        MINI_CHECK(scalar.set_domain(0, 1));
        MINI_CHECK(scalar.point_at(.5)[0] == 4 && scalar.point_at(.5)[1] == 0);
        const std::vector<Vector> scalar_values = scalar.evaluate(.5, 1);
        MINI_CHECK(scalar_values[1][0] == 2 && scalar_values[1][1] == 0 && scalar_values[1][2] == 0);
    }

    MINI_TEST("NurbsCurve", "One Dimensional Control Coordinates") {
        for (bool rational : {false, true}) {
            NurbsCurve curve(1, rational, 2, 2);
            MINI_CHECK(curve.set_nurbsknot(0, 0) && curve.set_nurbsknot(1, 1));
            MINI_CHECK(curve.set_cv_4d(0, rational ? 6 : 3, 17, 19, rational ? 2 : 1));
            MINI_CHECK(curve.set_cv_4d(1, rational ? 20 : 5, 23, 29, rational ? 4 : 1));
            MINI_CHECK(curve.is_valid() && curve.dimension() == 1);
            for (int i = 0; i < 2; ++i) {
                const Point point = curve.get_cv(i);
                MINI_CHECK(point[0] == (i == 0 ? 3 : 5));
                MINI_CHECK(point[1] == 0 && point[2] == 0);
                double x, y, z, w;
                MINI_CHECK(curve.get_cv_4d(i, x, y, z, w));
                MINI_CHECK(x / w == point[0] && y == point[1] && z == point[2]);
            }
        }
    }


    namespace {
    void trim_preserves_geometry(const NurbsCurve& source, double first, double last) {
        NurbsCurve part = source;
        MINI_CHECK(part.trim(first, last));
        MINI_CHECK(part.is_valid() && part.domain_start() == first && part.domain_end() == last);
        MINI_CHECK(part.point_at_start().distance(source.point_at(first)) < 1e-10);
        MINI_CHECK(part.point_at_end().distance(source.point_at(last)) < 1e-10);
        for (int i = 0; i <= 128; ++i) {
            const double t = i == 128 ? last : first + (last - first) * i / 128;
            MINI_CHECK(part.point_at(t).distance(source.point_at(t)) < 1e-10);
        }
        for (double knot : source.get_span_vector())
            if (knot >= first && knot <= last)
                MINI_CHECK(part.point_at(knot).distance(source.point_at(knot)) < 1e-10);
    }

    void trim_endpoint_cases(const NurbsCurve& source) {
        MINI_CHECK(source.is_valid() && !source.is_clamped());
        const double first = source.domain_start(), last = source.domain_end(), width = last - first;
        trim_preserves_geometry(source, first, first + .75 * width);
        trim_preserves_geometry(source, first + .25 * width, last);
        trim_preserves_geometry(source, first + .25 * width, first + .75 * width);
        NurbsCurve identity = source;
        MINI_CHECK(identity.trim(first, last));
        MINI_CHECK(identity.m_nurbsknot == source.m_nurbsknot && identity.m_cv == source.m_cv);
        MINI_CHECK(identity.cv_count() == source.cv_count());
    }

    NurbsCurve unclamped_trim_source(bool rational) {
        NurbsCurve curve(3, rational, 4, 6);
        for (int i = 0; i < curve.nurbsknot_count(); ++i)
            MINI_CHECK(curve.set_nurbsknot(i, i));
        for (int i = 0; i < curve.cv_count(); ++i) {
            const double weight = rational ? 1 + .25 * (i % 3) : 1;
            MINI_CHECK(curve.set_cv_4d(i, i * weight, i * i * weight, 0, weight));
        }
        return curve;
    }
    }

    MINI_TEST("NurbsCurve", "Unclamped Trim Preserves Geometry") {
        const NurbsCurve polynomial = unclamped_trim_source(false);
        MINI_CHECK(polynomial.domain_start() == 2 && polynomial.domain_end() == 5);
        MINI_CHECK(polynomial.point_at_start().distance({1, 4.0 / 3, 0}) < 1e-12);
        trim_preserves_geometry(polynomial, 2, 4);
        trim_endpoint_cases(polynomial);
        trim_endpoint_cases(unclamped_trim_source(true));
        const std::vector<Point> points = {{2, 0, 0}, {1, 1, .5}, {0, 2, 0}, {-1, 1, -.5},
            {-2, 0, 0}, {-1, -1, .5}, {0, -2, 0}, {1, -1, -.5}};
        for (bool rational : {false, true}) {
            NurbsCurve periodic = NurbsCurve::create(true, 3, points);
            MINI_CHECK(periodic.is_periodic());
            if (rational) {
                MINI_CHECK(periodic.to_rational());
                for (int i = 0; i < periodic.cv_count(); ++i) {
                    const Point point = periodic.get_cv(i);
                    const double weight = 1 + .25 * ((i % points.size()) % 3);
                    MINI_CHECK(periodic.set_cv_4d(i, point[0] * weight, point[1] * weight, point[2] * weight, weight));
                }
                MINI_CHECK(periodic.is_periodic());
            }
            trim_endpoint_cases(periodic);
        }
    }


    MINI_TEST("NurbsCurve", "Domain Mapping Preserves Unclamped Geometry") {
        for (bool rational : {false, true})
            for (bool clamped : {false, true})
                for (double scale : {std::ldexp(1.0, -44), 1.0, std::ldexp(1.0, 30)}) {
                    NurbsCurve source = unclamped_trim_source(rational);
                    if (clamped)
                        MINI_CHECK(source.clamp_end(2));
                    for (double& knot : source.m_nurbsknot)
                        knot *= scale;
                    MINI_CHECK(source.is_valid());
                    const double first = source.domain_start(), last = source.domain_end();
                    if (!rational && !clamped)
                        MINI_CHECK(source.point_at_start().distance({1, 4.0 / 3, 0}) < 1e-12);
                    const bool left_clamped = source.m_nurbsknot.front() == first;
                    const bool right_clamped = source.m_nurbsknot.back() == last;
                    NurbsCurve mapped = source;
                    MINI_CHECK(mapped.set_domain(-3, 5));
                    MINI_CHECK(mapped.is_valid() && mapped.domain_start() == -3 && mapped.domain_end() == 5);
                    MINI_CHECK(mapped.m_cv == source.m_cv && mapped.cv_count() == source.cv_count());
                    MINI_CHECK((mapped.m_nurbsknot.front() == -3) == left_clamped);
                    MINI_CHECK((mapped.m_nurbsknot.back() == 5) == right_clamped);
                    for (int i = 0; i <= 128; ++i) {
                        const double fraction = i / 128.0;
                        const double old_t = i == 128 ? last : first + fraction * (last - first);
                        const double new_t = -3 + 8 * fraction;
                        MINI_CHECK(mapped.point_at(new_t).distance(source.point_at(old_t)) < 1e-10);
                    }
                }
    }

} // namespace session_cpp

namespace session_cpp {
MINI_TEST("NurbsCurve", "Raw Parameter Endpoint Derivative Interpolation") {
    const std::vector<double> ts{-3, -2.75, -1.5, 0, 2};
    std::vector<Point> points;
    for (double t : ts) points.emplace_back(t*t*t, 2*t*t-t, 3*t+1);
    const auto before = points;
    const auto curve = NurbsCurve::create_interpolated_with_end_derivatives(
        points, ts, Vector(27, -13, 3), Vector(12, 7, 3));
    MINI_CHECK(curve.is_valid() && curve.domain_start() == -3 && curve.domain_end() == 2);
    for (double t : ts) MINI_CHECK(curve.point_at(t).distance(Point(t*t*t, 2*t*t-t, 3*t+1)) < 1e-11);
    for (int i = 0; i <= 32; ++i) {
        const double t = -3 + 5*i/32.;
        MINI_CHECK(curve.point_at(t).distance(Point(t*t*t, 2*t*t-t, 3*t+1)) < 1e-10);
    }
    const auto d0 = curve.evaluate(-3, 1)[1], d1 = curve.evaluate(2, 1)[1];
    for (int d=0; d<3; ++d) {
        MINI_CHECK(std::abs(d0[d] - Vector(27,-13,3)[d]) < 1e-11);
        MINI_CHECK(std::abs(d1[d] - Vector(12,7,3)[d]) < 1e-11);
    }
    MINI_CHECK(points == before);
    MINI_CHECK(!NurbsCurve::create_interpolated_with_end_derivatives(points, {-3,-3,-1.5,0,2}, Vector(), Vector()).is_valid());
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("NurbsCurve", "Degree Elevation Preserves Tiny Distinct Raw Knots") {
    const double end = 3.543711097672513e-33;
    for (bool rational : {false,true}) {
        NurbsCurve curve(3,rational,2,2);
        MINI_CHECK(curve.set_cv(0,Point(0,0,0)));
        MINI_CHECK(curve.set_cv(1,Point(1,2,-1)));
        if (rational) {
            MINI_CHECK(curve.set_cv_4d(0,0,0,0,2));
            MINI_CHECK(curve.set_cv_4d(1,3,6,-3,3));
        }
        curve.m_nurbsknot={0,end};
        MINI_CHECK(curve.is_valid());
        // Independently check the input geometry, before degree elevation.
        for(int i=0;i<=16;++i) {
            const double s=i/16.;
            const double x=rational ? 3*s/(2+s) : s;
            MINI_CHECK(curve.point_at(s*end).distance(Point(x,2*x,-x))<1e-12);
        }
        MINI_CHECK(curve.increase_degree(3));
        MINI_CHECK(curve.is_valid() && curve.degree()==3 && curve.cv_count()==4);
        MINI_CHECK(curve.domain_start()==0 && curve.domain_end()==end);
        MINI_CHECK(curve.m_nurbsknot==std::vector<double>({0,0,0,end,end,end}));
        for(int i=0;i<=16;++i) {
            const double s=i/16.;
            const double x=rational ? 3*s/(2+s) : s;
            MINI_CHECK(curve.point_at(s*end).distance(Point(x,2*x,-x))<1e-12);
        }
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("NurbsCurve", "Degree Elevation Rejects Excessive Exact Interior Multiplicity") {
    // Syntax validation currently permits this overmultiple interior knot.
    // Degree elevation must decline before using a negative CV skip/index.
    NurbsCurve curve(3, false, 2, 5);
    for (int i = 0; i < 5; ++i)
        MINI_CHECK(curve.set_cv(i, Point(i, 0, 0)));
    curve.m_nurbsknot = {0, .5, .5, .5, 1};
    MINI_CHECK(curve.is_valid());
    MINI_CHECK(!curve.increase_degree(2));
    // Failure is deliberately not required to preserve the mutated curve.
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("NurbsCurve", "Trim Preserves Exact Adjacent ULP Knots And Short Features") {
    for (int degree : {1, 2, 3, 5})
        for (bool rational : {false, true})
            for (int ulps : {1, 2})
                for (bool shifted : {false, true}) {
                    const double lo = shifted ? -7. : 0., hi = shifted ? 13. : 1.;
                    const double a = std::midpoint(lo, hi);
                    double b = a;
                    for (int j = 0; j < ulps; ++j)
                        b = std::nextafter(b, hi);
                    const double height = 1e-3;
                    NurbsCurve curve(3, rational, degree + 1, 3 * degree + 1);
                    curve.m_nurbsknot.clear();
                    curve.m_nurbsknot.insert(curve.m_nurbsknot.end(), degree, lo);
                    const std::array<double, 4> breaks{lo, a, b, hi};
                    for (int span = 0; span < 3; ++span) {
                        for (int j = 0; j <= degree; ++j) {
                            const int index = span * degree + j;
                            const double f = static_cast<double>(j) / degree;
                            const double x = std::lerp(breaks[span], breaks[span + 1], f);
                            const double y = span == 0 ? 0. : span == 1 ? height * f : height * (1 - f);
                            const double weight = 2. + index % 2;
                            if (rational)
                                MINI_CHECK(curve.set_cv_4d(index, x * weight, y * weight, 0, weight));
                            else
                                MINI_CHECK(curve.set_cv(index, Point(x, y, 0)));
                        }
                        curve.m_nurbsknot.insert(curve.m_nurbsknot.end(), degree, breaks[span + 1]);
                    }
                    MINI_CHECK(curve.is_valid());
                    const auto before = curve.jsondump();
                    auto left = curve;
                    MINI_CHECK(left.trim(lo, a));
                    MINI_CHECK(left.is_valid() && left.cv_count() == degree + 1);
                    MINI_CHECK(left.domain_start() == lo && left.domain_end() == a);
                    MINI_CHECK(left.point_at_end().distance(Point(a, 0, 0)) < 1e-14);
                    auto feature = curve;
                    MINI_CHECK(feature.trim(a, b));
                    MINI_CHECK(feature.is_valid() && feature.cv_count() == degree + 1);
                    MINI_CHECK(feature.domain_start() == a && feature.domain_end() == b);
                    MINI_CHECK(std::abs(feature.point_at_start()[1]) < 1e-14);
                    MINI_CHECK(std::abs(feature.point_at_end()[1] - height) < 1e-14);
                    if (ulps == 2) {
                        const auto p = feature.point_at(std::midpoint(a, b));
                        MINI_CHECK(p[1] > height * .1 && p[1] < height * .9);
                    }
                    NurbsCurve split_left, split_right;
                    MINI_CHECK(curve.split(a, split_left, split_right));
                    MINI_CHECK(split_left.domain_end() == a && split_right.domain_start() == a);
                    MINI_CHECK(split_left.cv_count() == degree + 1 &&
                               split_right.cv_count() == 2 * degree + 1);
                    MINI_CHECK(curve.jsondump() == before);
                }
}

MINI_TEST("NurbsCurve", "Divide By Count Stations Are Equal Arcs On Long Curves") {
    const NurbsCurve rib = NurbsCurve::create(false, 2, {Point(0, 0, 0), Point(1500, 900, 2400), Point(3000, 3000, 0)});
    const NurbsCurve bunched = NurbsCurve::create(false, 3, {Point(0, 0, 0), Point(0.001, 0.001, 0), Point(0.002, 0, 0),
        Point(1000, 500, 0), Point(1000.1, 500, 3), Point(2000, 0, 0)});
    for (const NurbsCurve& curve : {rib, bunched}) {
        const std::vector<double> stations = curve.divide_by_count(57, true).second;
        MINI_CHECK(stations.size() == 57 && stations.front() == curve.domain_start() && stations.back() == curve.domain_end());
        const double share = curve.length() / 56.0;
        for (size_t i = 1; i < stations.size(); ++i) {
            NurbsCurve piece = curve;
            MINI_CHECK(piece.trim(stations[i - 1], stations[i]));
            MINI_CHECK(std::abs(piece.length() - share) < 1e-7);
        }
    }
}
} // namespace session_cpp


namespace session_cpp {
namespace {
nlohmann::ordered_json raw_trim_fixture() {
    const auto root=std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root/"tests/fixtures/native_exact_trim_endpoint_intervals.json");
    MINI_CHECK(input.good());
    return nlohmann::ordered_json::parse(input);
}
}

MINI_TEST("NurbsCurve", "Actual Representable Positive Raw Prefix Is Preserved") {
    const auto record=raw_trim_fixture().at("cases").at(1).at("record");
    auto curve=NurbsCurve::jsonload(record.at("curve"));
    const double first=record.at("first"),last=record.at("last");
    MINI_CHECK(first==0 && last>0 && curve.domain_start()==0 && curve.domain_end()==1);
    MINI_CHECK(curve.trim(first,last));
    MINI_CHECK(curve.is_valid() && curve.domain_start()==first && curve.domain_end()==last);
    MINI_CHECK(!nurbs_coincidence::curve_constant_image(curve));
    for (double fraction:{0.,.25,.5,.75,1.}) {
        const double t=fraction*last;
        const auto point=curve.point_at(t);
        MINI_CHECK(std::abs(point[0]-.61)<=2*std::numeric_limits<double>::epsilon());
        MINI_CHECK(std::abs(point[1]-t)<=8*std::numeric_limits<double>::epsilon()*std::abs(t));
    }
    MINI_CHECK(curve.point_at_end()[1]>0);
}

MINI_TEST("NurbsCurve", "Unrepresentable Positive Bezier Prefix Declines Without Becoming Point") {
    const auto record=raw_trim_fixture().at("cases").at(0).at("record");
    auto curve=NurbsCurve::jsonload(record.at("curve"));
    const double first=record.at("first"),last=record.at("last");
    MINI_CHECK(first==0 && last>0 && !nurbs_coincidence::curve_constant_image(curve));
    const double start=curve.cv(0)[2],end=curve.cv(1)[2];
    MINI_CHECK(end>start && (end-start)*last>0);
    MINI_CHECK((end-start)*last*2<std::nextafter(start,std::numeric_limits<double>::infinity())-start);
    const auto frozen=curve.jsondump();
    // Safety prerequisite: the real source interval is positive, but binary64
    // endpoint materialization loses its variation. Keep its original lineage.
    MINI_CHECK(!curve.trim(first,last));
    MINI_CHECK(curve.jsondump()==frozen && !nurbs_coincidence::curve_constant_image(curve));
}

MINI_TEST("NurbsCurve", "Distinct Off Knot Raw Values Keep Amplified Physical Feature") {
    const double a=std::ldexp(1.,-55),b=std::ldexp(1.,-54),amplitude=std::ldexp(1.,45);
    NurbsCurve curve(3,false,2,2);
    curve.m_nurbsknot={0,1};
    MINI_CHECK(curve.set_cv(0,{0,0,0}) && curve.set_cv(1,{1,amplitude,0}));
    MINI_CHECK(curve.insert_nurbsknot(a,1));
    MINI_CHECK(curve.insert_nurbsknot(b,1));
    MINI_CHECK(curve.cv_count()==4);
    MINI_CHECK(std::count(curve.m_nurbsknot.begin(),curve.m_nurbsknot.end(),a)==1);
    MINI_CHECK(std::count(curve.m_nurbsknot.begin(),curve.m_nurbsknot.end(),b)==1);
    const auto frozen=curve.jsondump();
    auto piece=curve;
    MINI_CHECK(piece.trim(a,b));
    MINI_CHECK(piece.is_valid() && piece.domain_start()==a && piece.domain_end()==b);
    MINI_CHECK(piece.point_at_start()[1]==amplitude*a && piece.point_at_end()[1]==amplitude*b);
    MINI_CHECK(piece.point_at_end()[1]-piece.point_at_start()[1]>.0009);
    MINI_CHECK(curve.jsondump()==frozen);
}

MINI_TEST("NurbsCurve", "Off Knot Shifted Reversed Raw Interval Retains Real Feature") {
    const double a=3,b=std::nextafter(a,13.),amplitude=std::ldexp(1.,45);
    const double height=amplitude*(b-a);
    MINI_CHECK(a<b && height>.01);
    for (bool reverse:{false,true}) {
        NurbsCurve curve(3,false,2,2);
        curve.m_nurbsknot={-7,13};
        MINI_CHECK(curve.set_cv(0,{-10,-10*amplitude,0}) && curve.set_cv(1,{10,10*amplitude,0}));
        if (reverse) MINI_CHECK(curve.reverse());
        const auto frozen=curve.jsondump();
        auto piece=curve;
        MINI_CHECK(piece.trim(a,b));
        MINI_CHECK(piece.is_valid() && piece.domain_start()==a && piece.domain_end()==b);
        const double sign=reverse?-1.:1.;
        MINI_CHECK(piece.point_at_start()[1]==0);
        MINI_CHECK(sign*piece.point_at_end()[1]>.9*height);
        MINI_CHECK(std::abs(piece.point_at_end()[1]-sign*height)<=16*std::numeric_limits<double>::epsilon()*height);
        MINI_CHECK(!nurbs_coincidence::curve_constant_image(piece));
        MINI_CHECK(curve.jsondump()==frozen);
    }
}

MINI_TEST("NurbsCurve", "Variable Positive Weights Keep Tiny Raw Prefix And Reversed Suffix") {
    const double tiny=std::ldexp(1.,-55);
    for (bool reverse:{false,true}) {
        NurbsCurve source(3,true,2,2);
        source.m_nurbsknot={0,1};
        MINI_CHECK(source.set_cv_4d(0,0,0,0,2) && source.set_cv_4d(1,3,6,-3,3));
        if (reverse) MINI_CHECK(source.reverse());
        const double a=reverse?std::nextafter(1.,0.):0.,b=reverse?1.:tiny;
        const auto frozen=source.jsondump();
        auto piece=source;
        MINI_CHECK(piece.trim(a,b));
        MINI_CHECK(piece.is_valid() && piece.is_rational());
        MINI_CHECK(piece.domain_start()==a && piece.domain_end()==b);
        MINI_CHECK(!nurbs_coincidence::curve_constant_image(piece));
        for (double t:{a,b}) {
            const double expected=reverse?3*(1-t)/(3-t):3*t/(2+t);
            const auto point=piece.point_at(t);
            const double error=16*std::numeric_limits<double>::epsilon()*std::abs(expected);
            MINI_CHECK(std::abs(point[0]-expected)<=error);
            MINI_CHECK(std::abs(point[1]-2*expected)<=2*error);
            MINI_CHECK(std::abs(point[2]+expected)<=error);
        }
        MINI_CHECK(source.jsondump()==frozen);
    }
    NurbsCurve constant(3,true,2,2);
    constant.m_nurbsknot={0,1};
    MINI_CHECK(constant.set_cv_4d(0,1,-2,4,3) && constant.set_cv_4d(1,2,-4,8,6));
    MINI_CHECK(nurbs_coincidence::curve_constant_image(constant));
    MINI_CHECK(constant.trim(0,tiny));
    MINI_CHECK(constant.domain_start()==0 && constant.domain_end()==tiny);
    MINI_CHECK(nurbs_coincidence::curve_constant_image(constant));
}

MINI_TEST("NurbsCurve", "Clamped Outside Positive Request Declines Before Mutation") {
    NurbsCurve source(3,false,3,4);
    source.m_nurbsknot={-1,0,1,2,3};
    for (int i=0;i<4;++i) MINI_CHECK(source.set_cv(i,{static_cast<double>(i),static_cast<double>(i%2),0}));
    MINI_CHECK(source.is_valid() && !source.is_clamped());
    const double tiny=std::ldexp(1.,-55),beyond=std::nextafter(2.,3.);
    for (const auto range:{std::pair<double,double>{-2*tiny,-tiny},
                           std::pair<double,double>{beyond,std::nextafter(beyond,3.)}}) {
        auto curve=source;
        const auto frozen=curve.jsondump();
        MINI_CHECK(range.first<range.second);
        MINI_CHECK(!curve.trim(range.first,range.second));
        MINI_CHECK(curve.jsondump()==frozen);
    }
}
} // namespace session_cpp


namespace session_cpp {
MINI_TEST("NurbsCurve", "Original Linear Trim Keeps Exact Representable Endpoints") {
    for (int dimension:{1,2,3}) for (bool rational:{false,true})
        for (double weight:{2.,3.,5.}) for (bool shifted:{false,true}) {
            NurbsCurve curve(dimension,rational,2,2);
            const double lo=shifted?-7.:0.,hi=shifted?13.:1.;
            const double first=shifted?-2.:.25,last=shifted?8.:.75;
            curve.m_nurbsknot={lo,hi};
            const std::array<Point,2> points={Point(-2,.5,-7),Point(2,-.5,13)};
            for (int i=0;i<2;++i) {
                if (rational) MINI_CHECK(curve.set_cv_4d(i,points[i][0]*weight,points[i][1]*weight,points[i][2]*weight,weight));
                else MINI_CHECK(curve.set_cv(i,points[i]));
            }
            const auto guid=curve.jsondump().at("guid");
            MINI_CHECK(curve.trim(first,last));
            MINI_CHECK(curve.is_valid() && curve.cv_count()==2 && curve.degree()==1);
            MINI_CHECK(curve.domain_start()==first && curve.domain_end()==last);
            MINI_CHECK(curve.is_rational()==rational && curve.jsondump().at("guid")==guid);
            const std::array<Point,2> expected={Point(-1,.25,-2),Point(1,-.25,8)};
            for (int i=0;i<2;++i) for (int d=0;d<dimension;++d) {
                MINI_CHECK(curve.get_cv(i)[d]==expected[i][d]);
                MINI_CHECK(curve.cv(i)[d]==expected[i][d]*(rational?weight:1.));
            }
            if (rational) MINI_CHECK(curve.weight(0)==weight && curve.weight(1)==weight);
        }
}
MINI_TEST("NurbsCurve", "Original Rational Linear Trim Preserves Homogeneous Raw Law") {
    NurbsCurve curve(3,true,2,2);curve.m_nurbsknot={0,1};
    MINI_CHECK(curve.set_cv_4d(0,-4,2,0,2) && curve.set_cv_4d(1,12,0,6,3));
    MINI_CHECK(curve.trim(.25,.75));
    const std::array<std::array<double,4>,2> expected={{{0,1.5,1.5,2.25},{8,.5,4.5,2.75}}};
    MINI_CHECK(curve.is_valid() && curve.domain_start()==.25 && curve.domain_end()==.75);
    for (int i=0;i<2;++i) for (int d=0;d<4;++d) MINI_CHECK(curve.cv(i)[d]==expected[i][d]);
    MINI_CHECK(!nurbs_coincidence::curve_constant_image(curve));
}
MINI_TEST("NurbsCurve", "Original Linear Trim Declines Actual Nonrepresentable Positive Interval") {
    NurbsCurve curve(3,false,2,2);curve.m_nurbsknot={0,1};
    MINI_CHECK(curve.set_cv(0,{0,.0009613037109375,1.02}));
    MINI_CHECK(curve.set_cv(1,{0,.0009613037109375,-1.02}));
    const double first=.009803921568627416,last=.00980392156862746;
    MINI_CHECK(first<last && 2*1.02*(last-first)>0);
    MINI_CHECK(2*1.02*(last-first)<std::numeric_limits<double>::epsilon());
    const auto frozen=curve.jsondump();
    MINI_CHECK(!curve.trim(first,last));
    MINI_CHECK(curve.jsondump()==frozen && !nurbs_coincidence::curve_constant_image(curve));
}
} // namespace session_cpp
