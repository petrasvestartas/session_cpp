#include "mini_test.h"
#include "nurbscurve.h"
#include "nurbscurve.pb.h"
#include "point.h"
#include "plane.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "primitives.h"
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("NurbsCurve", "Constructor") {

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
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[1], Point(0.328571016773017, 0.598213507757063, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[2], Point(0.740744944144815, 1.140321237310326, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[3], Point(1.338524001477341, 1.232716038191446, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[4], Point(1.712929668000343, 0.664818751028787, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[5], Point(2.287070333148604, 0.664818752348101, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[6], Point(2.661475999779531, 1.232716039392177, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[7], Point(3.259255057037078, 1.140321236176910, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[8], Point(3.671428983538974, 0.598213507250245, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[9], Point(4.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(len_pts.size() == 13);
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[6], Point(1.928691288503169, 0.510169864670676, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[12], Point(3.934494396222682, 0.128829843907475, 0.0)));
    }

    MINI_TEST("NurbsCurve", "Evaluation") {

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
        MINI_CHECK(TOLERANCE.is_point_close(frames[2].origin(), Point(3.676077075808618, 0.909845354074582, 0.350126131660904)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[2].x_axis(), Vector(-0.188216728828592, 0.616420980974357, -0.764591156896073)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[2].y_axis(), Vector(0.183061410483993, -0.742842969436200, -0.643950963001702)));
        MINI_CHECK(TOLERANCE.is_vector_close(frames[2].z_axis(), Vector(-0.964916049706230, -0.261169479407185, 0.026972579511507)));
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
        NurbsCurve open_curve = NurbsCurve::create(false, 3, points);
        const bool ok_inside = inside.insert_nurbsknot(0.3, 1);
        const bool ok_outside = outside.insert_nurbsknot(2.3, 1);
        const bool ok_open = open_curve.insert_nurbsknot(open_curve.domain_end() + 0.5, 1);

        MINI_CHECK(ok_inside);
        MINI_CHECK(ok_outside);
        MINI_CHECK(!ok_open);
        MINI_CHECK(outside.cv_count() == 8);
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
        }
    }

    MINI_TEST("NurbsCurve", "Insert Knot Multiplicity Limit") {

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

        const NurbsCurve curve;

        MINI_CHECK(curve.get_span_vector().empty());
    }

    MINI_TEST("NurbsCurve", "Periodic Too Few Points") {

        NurbsCurve curve;
        const bool ok = curve.create_periodic_uniform(3, 4, {Point(0, 0, 0), Point(1, 0, 0)});

        MINI_CHECK(!ok);
    }

    MINI_TEST("NurbsCurve", "Polyline Adaptive Closed") {

        const NurbsCurve circle = Primitives::circle(0, 0, 0, 2.0);
        const std::pair<std::vector<Point>, std::vector<double>> polyline = circle.to_polyline_adaptive(0.1, 0.0, 0.0);

        MINI_CHECK(polyline.first.size() == 25);
        MINI_CHECK(TOLERANCE.is_point_close(polyline.first.front(), polyline.first.back()));
    }

    MINI_TEST("NurbsCurve", "Circle Length") {

        const NurbsCurve circle = Primitives::circle(0, 0, 0, 2.0);

        MINI_CHECK(std::abs(circle.length() - 4.0 * Tolerance::PI) < 1e-9);
    }

} // namespace session_cpp
