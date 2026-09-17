#include "mini_test.h"
#include "nurbscurve.h"
#include "point.h"
#include "plane.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "primitives.h"
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("NurbsCurve", "Constructor") {

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);

        std::string cstr = curve.str();
        std::string crepr = curve.repr();

        NurbsCurve ccopy = curve;
        NurbsCurve cother = NurbsCurve::create(false, 2, points);

        auto [divided, _] = curve.divide_by_count(10, true);

        MINI_CHECK(curve.is_valid() == true);
        MINI_CHECK(curve.cv_count() == 4);
        MINI_CHECK(curve.degree() == 2);
        MINI_CHECK(curve.order() == 3);
        MINI_CHECK(curve.name == "my_nurbscurve");
        MINI_CHECK(!curve.guid().empty());
        MINI_CHECK(cstr == "NurbsCurve(name=my_nurbscurve, degree=2, cvs=4)");
        MINI_CHECK(crepr.find("name=my_nurbscurve") != std::string::npos);
        MINI_CHECK(ccopy.cv_count() == curve.cv_count());
        MINI_CHECK(ccopy.guid() != curve.guid());
    }

    MINI_TEST("NurbsCurve", "Create Interpolated") {

        std::vector<Point> points = {
            Point(14, 9, 0),
            Point(21, 22, 0),
            Point(26, 10, 0),
            Point(35, 19, 0),
            Point(41, 13, 0)
        };

        NurbsCurve c = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Chord);

        MINI_CHECK(c.is_valid());
        MINI_CHECK(c.degree() == 3);
        MINI_CHECK(c.order() == 4);
        MINI_CHECK(c.cv_count() == 7);
        MINI_CHECK(c.is_rational() == false);

        auto [d0, d1] = c.domain();
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d0), points[0]));
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d1), points[4]));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(0), points[0]));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(6), points[4]));

        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(1), Point(15.342776949, 13.734888836, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(3), Point(24.678472471, 0.354555126, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(5), Point(39.626394361, 15.472490151, 0.0)));

        NurbsCurve co = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Chord, CurveInterpStyle::Occt);
        MINI_CHECK(co.cv_count() == 7);
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(0), points[0]));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(6), points[4]));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(1), Point(17.3526678158, 24.4472657919, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(3), Point(24.7854378511, 2.1457823679, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(co.get_cv(5), Point(39.1865250566, 18.5349257754, 0.0)));

        std::vector<Point> closed_pts = {
            Point(4, 20, 0),
            Point(-2, 20, 0),
            Point(-2, 25, 0),
            Point(-3, 28, 0),
            Point(-10, 28, 0),
            Point(-10, 21, 0),
            Point(-13, 16, 0),
            Point(-8, 14, 0),
            Point(-6, 11, 0),
            Point(0, 15, 0),
        };

        NurbsCurve cp = NurbsCurve::create_interpolated(closed_pts, CurveNurbsKnotStyle::ChordPeriodic);

        MINI_CHECK(cp.is_valid());
        MINI_CHECK(cp.degree() == 3);
        MINI_CHECK(cp.cv_count() == 13);
        MINI_CHECK(cp.is_closed());
    }

    MINI_TEST("NurbsCurve", "Create From Parameters") {

        std::vector<Point> p4 = {Point(0,0,0), Point(3,6,0), Point(6,-3,3), Point(10,0,0)};
        NurbsCurve c = NurbsCurve::create_from_parameters(
            p4, {1.0,1.0,1.0,1.0}, {0.0,1.0}, {4,4}, 3);
        MINI_CHECK(c.is_valid());
        MINI_CHECK(c.degree() == 3);
        MINI_CHECK(c.cv_count() == 4);
        MINI_CHECK(!c.is_rational());
        auto [d0, d1] = c.domain();
        MINI_CHECK(std::abs(d0 - 0.0) < 1e-12 && std::abs(d1 - 1.0) < 1e-12);
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(0), Point(0, 0, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.get_cv(3), Point(10, 0, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(0.5), Point(4.625, 1.125, 1.125)));

        const double w = 0.5 * std::sqrt(2.0);
        std::vector<Point> cpts = {
            Point(0,-1,0), Point(-1,-1,0), Point(-1,0,0), Point(-1,1,0), Point(0,1,0),
            Point(1,1,0), Point(1,0,0), Point(1,-1,0), Point(0,-1,0)};

        NurbsCurve circle = NurbsCurve::create_from_parameters(
            cpts, {1,w,1,w,1,w,1,w,1}, {0.0, 0.25, 0.5, 0.75, 1.0}, {3, 2, 2, 2, 3}, 2);
        MINI_CHECK(circle.is_valid());
        MINI_CHECK(circle.degree() == 2);
        MINI_CHECK(circle.cv_count() == 9);
        MINI_CHECK(circle.is_rational());
        MINI_CHECK(TOLERANCE.is_point_close(circle.point_at(0.5), Point(0, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(circle.point_at(0.125), Point(-w, -w, 0)));

        for (int k = 0; k <= 16; k++) {
            Point pp = circle.point_at(k / 16.0);
            MINI_CHECK(std::abs(std::sqrt(pp[0]*pp[0] + pp[1]*pp[1]) - 1.0) < 1e-9);
        }
    }

    MINI_TEST("NurbsCurve", "Create Fitted") {

        std::vector<Point> pts;

        for (int i = 0; i <= 20; i++) {
            double t = i * 2.0 * Tolerance::PI / 20.0;
            pts.push_back(Point(t, 3.0 * std::sin(t), 0.0));
        }

        NurbsCurve c = NurbsCurve::create_fitted(pts, 8, 3, false);

        MINI_CHECK(c.is_valid());
        MINI_CHECK(c.degree() == 3);
        MINI_CHECK(c.cv_count() == 8);
        auto [d0, d1] = c.domain();
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d0), pts[0]));
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at(d1), pts[20]));

        std::vector<Point> cpts;

        for (int i = 0; i < 24; i++) {
            double a = i * 2.0 * Tolerance::PI / 24.0;
            cpts.push_back(Point(std::cos(a), std::sin(a), 0.0));
        }

        NurbsCurve cp = NurbsCurve::create_fitted(cpts, 10, 3, true);

        MINI_CHECK(cp.is_valid());
        MINI_CHECK(cp.is_closed());
        MINI_CHECK(cp.cv_count() == 13);
    }

    MINI_TEST("NurbsCurve", "Join") {

        NurbsCurve arc1 = Primitives::arc(Point(-1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0), Point(1.0, 0.0, 0.0));
        NurbsCurve arc2 = Primitives::arc(Point(1.0, 0.0, 0.0), Point(1.5, -1.0, 0.0), Point(1.0, -2.0, 0.0));
        std::vector<Point> pts = {Point(1.0, -2.0, 0.0), Point(-1.0, 0.0, 0.0)};
        NurbsCurve line = NurbsCurve::create(false, 1, pts);
        arc2.reverse();

        std::vector<NurbsCurve> joined = NurbsCurve::join({line, arc1, arc2});

        MINI_CHECK(joined.size() == 1);
        MINI_CHECK(joined[0].is_valid());
        MINI_CHECK(joined[0].is_closed());
        MINI_CHECK(joined[0].degree() == 2);
        MINI_CHECK(joined[0].cv_count() == 7);

        NurbsCurve l1 = NurbsCurve::create(false, 1, {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0)});
        NurbsCurve l2 = NurbsCurve::create(false, 1, {Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
        NurbsCurve l3 = NurbsCurve::create(false, 1, {Point(9.0, 9.0, 0.0), Point(8.0, 8.0, 0.0)});

        std::vector<NurbsCurve> separate = NurbsCurve::join({l1, l3, l2});

        MINI_CHECK(separate.size() == 2);
        MINI_CHECK(separate[0].cv_count() == 3);
        MINI_CHECK(std::fabs(separate[0].length() - 2.0) < 1e-9);
    }

    MINI_TEST("NurbsCurve", "Attributes") {

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        bool is_valid = curve.is_valid();

        MINI_CHECK(is_valid);

        bool is_valid_nurbsknot_vector = curve.is_valid_nurbsknot_vector();
        MINI_CHECK(is_valid_nurbsknot_vector);

        bool is_clamped_start = curve.is_clamped(0);
        bool is_clamped_end = curve.is_clamped(1);
        bool is_clamped_both = curve.is_clamped(2);
        MINI_CHECK(is_clamped_start && is_clamped_end && is_clamped_both);

        bool is_rational = curve.is_rational();
        bool closed = curve.is_closed();
        bool periodic = curve.is_periodic();
        bool linear = curve.is_linear();
        bool planar = curve.is_planar();
        bool arc = curve.is_arc();
        Plane plane = Plane::xy_plane();
        bool on_plane = curve.is_in_plane(plane);
        bool is_open = curve.is_natural();
        bool is_polyline = curve.is_polyline();
        bool is_singular = curve.is_singular();
        bool is_duplicate = curve.is_duplicate(curve, false);
        bool is_continuous = curve.is_continuous(1, curve.domain_middle());

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
        Point before_pt = copy_curve.point_at(1.5);
        copy_curve.insert_nurbsknot(1.5, 1);
        MINI_CHECK(TOLERANCE.is_point_close(before_pt, copy_curve.point_at(1.5)));

        double greville0 = curve.greville_abcissa(0);
        MINI_CHECK(TOLERANCE.is_close(greville0, 0.0));

        std::vector<double> greville = curve.get_greville_abcissae();
        MINI_CHECK(greville.size() == 4);
        MINI_CHECK(TOLERANCE.is_close(greville[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(greville[1], 0.879872167739067));
        MINI_CHECK(TOLERANCE.is_close(greville[2], 2.639616503217201));
        MINI_CHECK(TOLERANCE.is_close(greville[3], 3.519488670956267));

        int dimension = curve.dimension();
        MINI_CHECK(dimension == 3);
        int degree = curve.degree();
        MINI_CHECK(degree == 2);
        int order = curve.order();
        MINI_CHECK(order == 3);
        int cv_count = curve.cv_count();
        MINI_CHECK(cv_count == 4);
        int cv_size = curve.cv_size();
        MINI_CHECK(cv_size == 3);
        int nurbsknot_count = curve.nurbsknot_count();
        MINI_CHECK(nurbsknot_count == 5);
        int span_count = curve.span_count();
        MINI_CHECK(span_count == 2);

        double* p = curve.cv(1);
        MINI_CHECK(p[0] == 1.0 && p[1] == 1.0 && p[2] == 0.0);

        Point cv_point = curve.get_cv(1);
        MINI_CHECK(cv_point == Point(1.0, 1.0, 0.0));

        auto [x, y, z, w] = curve.get_cv_4d(1);
        MINI_CHECK(x == 1.0 && y == 1.0 && z == 0.0 && w == 1.0);

        curve.set_cv(2, Point(2.0, 0.0, 0.5));
        MINI_CHECK(curve.get_cv(2)[0] == 2.0);
        MINI_CHECK(curve.get_cv(2)[1] == 0.0);
        MINI_CHECK(curve.get_cv(2)[2] == 0.5);

        curve.set_cv_4d(2, 2.0, 0.0, 0.5, 0.707);
        auto [x2, y2, z2, w2] = curve.get_cv_4d(2);
        MINI_CHECK(x2 == 2.0 && y2 == 0.0 && z2 == 0.5 && w2 == 0.707);

        double weight = curve.weight(2);
        MINI_CHECK(weight == 0.707);

        curve.set_weight(2, 0.5);
        MINI_CHECK(curve.weight(2) == 0.5);

        double nurbsknot3 = curve.nurbsknot(3);
        MINI_CHECK(TOLERANCE.is_close(nurbsknot3, 3.519488670956267));

        double end_nurbsknot = curve.nurbsknot(4);
        curve.set_nurbsknot(4, end_nurbsknot);
        MINI_CHECK(TOLERANCE.is_close(curve.nurbsknot(4), end_nurbsknot));

        int m0 = curve.nurbsknot_multiplicity(0);
        int m1 = curve.nurbsknot_multiplicity(1);
        int m2 = curve.nurbsknot_multiplicity(2);
        int m3 = curve.nurbsknot_multiplicity(3);
        int m4 = curve.nurbsknot_multiplicity(4);
        MINI_CHECK(m0 == 2);
        MINI_CHECK(m1 == 2);
        MINI_CHECK(m2 == 1);
        MINI_CHECK(m3 == 2);
        MINI_CHECK(m4 == 2);

        double superfluous_nurbsknot = curve.superfluous_nurbsknot(1);
        MINI_CHECK(TOLERANCE.is_close(superfluous_nurbsknot, 7.038977341912535));

        const double* nurbsknots = curve.nurbsknot_array();
        double k0 = nurbsknots[0];
        std::vector<double> nurbsknot_vector = curve.get_nurbsknots();
        MINI_CHECK(k0 == 0.0);
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[2], 1.759744335478134));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[3], 3.519488670956267));
        MINI_CHECK(TOLERANCE.is_close(nurbsknot_vector[4], 3.519488670956267));

        const double* cvs = curve.cv_array();
        double cx0 = cvs[0];
        MINI_CHECK(cx0 == 0.0);

        std::pair<double, double> interval = curve.domain();
        double start = interval.first;
        double end = interval.second;
        MINI_CHECK(TOLERANCE.is_close(start, 0.0));
        MINI_CHECK(TOLERANCE.is_close(end, 3.519488670956267));

        start = curve.domain_start();
        double middle = curve.domain_middle();
        end = curve.domain_end();
        MINI_CHECK(TOLERANCE.is_close(start, 0.0));
        MINI_CHECK(TOLERANCE.is_close(middle, 1.759744335478134));
        MINI_CHECK(TOLERANCE.is_close(end, 3.519488670956267));

        curve.set_domain(0.0, 1.0);
        MINI_CHECK(curve.domain_start() == 0.0);
        MINI_CHECK(curve.domain_middle() == 0.5);
        MINI_CHECK(curve.domain_end() == 1.0);

        std::vector<double> intervals =  curve.get_span_vector();
        MINI_CHECK(TOLERANCE.is_close(intervals[0], 0.0) && TOLERANCE.is_close(intervals[1], 0.5) && TOLERANCE.is_close(intervals[2], 1.0));

        auto [found, t_out] = curve.get_next_discontinuity(2, curve.domain_start(), curve.domain_end());
        MINI_CHECK(found && TOLERANCE.is_close(t_out, 0.5));
    }

    MINI_TEST("NurbsCurve", "Conversions") {

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        auto [adaptive_pts, adaptive_params] = curve.to_polyline_adaptive(0.1, 0.0, 0.0);

        MINI_CHECK(adaptive_pts.size() == 27);
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[13], Point(2.0, 0.5, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[26], Point(4.0, 0.0, 0.0)));

        auto [div_pts, div_params] = curve.divide_by_count(10, true);

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

        auto [len_pts, len_params] = curve.divide_by_length(0.5);

        MINI_CHECK(len_pts.size() == 13);
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[6], Point(1.928691288503169, 0.510169864670676, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[12], Point(3.934494396222682, 0.128829843907475, 0.0)));
    }

    MINI_TEST("NurbsCurve", "Evaluation"){

        std::vector<Point> points = {
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

        MINI_CHECK(TOLERANCE.is_close(curve.length(), 11.3010276326));

        Point point_at = curve.point_at(0.5);
        MINI_CHECK(TOLERANCE.is_close(point_at[0], 1.463452399002842));
        MINI_CHECK(TOLERANCE.is_close(point_at[1], 1.680997287875395));
        MINI_CHECK(TOLERANCE.is_close(point_at[2], -0.124474565996108));

        std::vector<Vector> derivatives = curve.evaluate(0.5, 2);
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

        Vector tangent = curve.tangent_at(0.5);
        MINI_CHECK(TOLERANCE.is_close(tangent[0], -0.304511941745027));
        MINI_CHECK(TOLERANCE.is_close(tangent[1], 0.951805546117607));
        MINI_CHECK(TOLERANCE.is_close(tangent[2], -0.036587972264639));

        Plane f = curve.plane_at(0.5, true);

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

        MINI_CHECK(curve.plane_at(-0.1, true).is_valid() == false);
        MINI_CHECK(curve.plane_at(1.1, true).is_valid() == false);
        MINI_CHECK(curve.plane_at(curve.domain_start(), false).is_valid() == true);
        MINI_CHECK(curve.plane_at(curve.domain_end(), false).is_valid() == true);
        MINI_CHECK(curve.plane_at(curve.domain_start() - 0.1, false).is_valid() == false);

        Plane pf = curve.perpendicular_plane_at(0.5, true);
        MINI_CHECK(TOLERANCE.is_point_close(pf.origin(), Point(3.156927375000000, 1.335111500000000, 0.130488875000000)));
        MINI_CHECK(TOLERANCE.is_vector_close(pf.x_axis(), Vector(0.632703652329189, -0.703685357647999, 0.323284713157168)));
        MINI_CHECK(TOLERANCE.is_vector_close(pf.y_axis(), Vector(0.327344206830723, -0.135306795251661, -0.935167279909370)));
        MINI_CHECK(TOLERANCE.is_vector_close(pf.z_axis(), Vector(0.701806140314880, 0.697509131546342, 0.144738221716994)));
        MINI_CHECK(curve.perpendicular_plane_at(-0.1, true).is_valid() == false);
        MINI_CHECK(curve.perpendicular_plane_at(1.1, true).is_valid() == false);
        MINI_CHECK(curve.perpendicular_plane_at(curve.domain_start(), false).is_valid() == true);
        MINI_CHECK(curve.perpendicular_plane_at(curve.domain_end(), false).is_valid() == true);
        MINI_CHECK(curve.perpendicular_plane_at(curve.domain_start() - 0.1, false).is_valid() == false);

        std::vector<Plane> frames = curve.get_perpendicular_planes(4);
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

        Point p0 = curve.point_at_start();
        Point p1 = curve.point_at_middle();
        Point p2 = curve.point_at_end();
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

    MINI_TEST("NurbsCurve", "Modifications"){

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        NurbsCurve curve_reversed = curve;
        curve_reversed.reverse();

        MINI_CHECK(TOLERANCE.is_point_close(curve_reversed.point_at_start(), curve.point_at_end()));

        curve.swap_coordinates(0, 1);
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(0), Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(1), Point(2.0, 1.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(2), Point(0.0, 2.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(3), Point(2.0, 3.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(4), Point(0.0, 4.0, 0.0)));

        NurbsCurve ct = curve;
        double a = ct.domain_start() + (ct.domain_end() - ct.domain_start()) / 3.0;
        double b = ct.domain_start() + 2.0 * (ct.domain_end() - ct.domain_start()) / 3.0;
        ct.trim(a, b);
        MINI_CHECK(ct.length() < curve.length());

        double split_t = curve.domain_middle();
        auto [curve_left, curve_right] = curve.split(split_t);
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(split_t), curve_left.point_at_end()));
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(split_t), curve_right.point_at_start()));

        NurbsCurve curve_extended = curve;
        curve_extended.extend(curve.domain_start()-0.5, curve.domain_end()+0.5);
        MINI_CHECK(curve_extended.length() > curve.length());

        NurbsCurve curve_rational = curve;
        double original_length = curve.length();
        curve_rational.make_rational();
        curve_rational.set_weight(2, 10);
        MINI_CHECK(curve_rational.length() != original_length);

        curve_rational.make_non_rational(true);
        MINI_CHECK(curve_rational.length() == original_length);

        std::vector<Point> points_open = points;
        NurbsCurve curve_open(3, false, 3, 5);

        for (int i = 0; i < 5; ++i)
            curve_open.set_cv(i, points_open[i]);

        for (int i = 0; i < curve_open.nurbsknot_count(); ++i)
            curve_open.set_nurbsknot(i, i * 1.0);

        curve_open.clamp_end(2);
        std::vector<double> nurbsknots = curve_open.get_nurbsknots();
        MINI_CHECK(TOLERANCE.is_close(nurbsknots[0], nurbsknots[1]));
        MINI_CHECK(TOLERANCE.is_close(nurbsknots[nurbsknots.size() - 2], nurbsknots[nurbsknots.size() - 1]));

        NurbsCurve raised = curve;
        raised.increase_degree(3);
        MINI_CHECK(curve.degree() != raised.degree());
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at_middle(), raised.point_at_middle()));

        std::vector<Point> closed_pts = {
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(-1.0, 0.0, 0.0),
            Point(0.0, -1.0, 0.0)
        };
        NurbsCurve c = NurbsCurve::create(true, 2, closed_pts);
        Point expected_start = c.point_at(c.domain_middle());
        c.change_closed_curve_seam(c.domain_middle());
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at_start(), expected_start));
    }

    MINI_TEST("NurbsCurve", "Transformations"){

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve1 = NurbsCurve::create(false, 2, points);
        Xform curve1_xf = Xform::translation(0.0, 0.0, 1.0);
        curve1.transform(curve1_xf);
        MINI_CHECK(curve1.cv(0)[2] == 1.0);

        NurbsCurve curve2 = NurbsCurve::create(false, 2, points);
        Xform x = Xform::translation(0.0, 0.0, 1.0);
        curve2.transform(x);
        MINI_CHECK(curve2.cv(0)[2] == 1.0);

        NurbsCurve curve3 = NurbsCurve::create(false, 2, points);
        Xform curve3_xf = Xform::translation(0.0, 0.0, 10.0);
        NurbsCurve curve3_transformed = curve3.transformed(curve3_xf);
        MINI_CHECK(curve3_transformed.cv(0)[2] == 10.0);

        NurbsCurve curve4 = NurbsCurve::create(false, 2, points);
        x = Xform::translation(0.0, 0.0, 10.0);
        NurbsCurve curve4_transformed = curve4.transformed(x);
        MINI_CHECK(curve4_transformed.cv(0)[2] == 10.0);
    }

    MINI_TEST("NurbsCurve", "Json Roundtrip") {

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };
        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        nlohmann::ordered_json json = curve.jsondump();
        NurbsCurve loaded_json = NurbsCurve::jsonload(json);

        std::string json_string = curve.file_json_dumps();
        NurbsCurve loaded_json_string = NurbsCurve::file_json_loads(json_string);

        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbscurve.json").string();
        curve.file_json_dump(filename);
        NurbsCurve loaded_from_file = NurbsCurve::file_json_load(filename);

        MINI_CHECK(loaded_json == curve);
        MINI_CHECK(loaded_json_string == curve);
        MINI_CHECK(loaded_from_file == curve);
    }

    MINI_TEST("NurbsCurve", "Protobuf Roundtrip") {

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };
        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        std::string proto_string = curve.pb_dumps();
        NurbsCurve loaded_proto_string = NurbsCurve::pb_loads(proto_string);

        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbscurve.bin").string();
        curve.pb_dump(filename);
        NurbsCurve loaded = NurbsCurve::pb_load(filename);

        MINI_CHECK(loaded_proto_string == curve);
        MINI_CHECK(loaded == curve);
    }

    MINI_TEST("NurbsCurve", "Curvature") {

        const double R = 2.0;
        NurbsCurve circle = Primitives::circle(0, 0, 0, R);
        auto [t0, t1] = circle.domain();

        for (int i = 0; i <= 8; ++i) {
            double t = t0 + (t1 - t0) * i / 8.0;
            MINI_CHECK(std::abs(circle.curvature_at(t) - 1.0 / R) < 1e-6);
        }

        std::vector<Point> line_pts = {Point(0, 0, 0), Point(1, 0, 0), Point(2, 0, 0), Point(3, 0, 0)};
        NurbsCurve line = NurbsCurve::create(false, 1, line_pts);
        MINI_CHECK(line.curvature_at(line.domain_middle()) < 1e-9);
    }

    MINI_TEST("NurbsCurve", "Closest Point") {

        NurbsCurve circle = Primitives::circle(0, 0, 0, 2.0);
        Point cp = circle.closest_point(Point(5, 0, 0));
        MINI_CHECK(std::abs(cp[0] - 2.0) < 1e-5 && std::abs(cp[1]) < 1e-5 && std::abs(cp[2]) < 1e-5);
        Point cp2 = circle.closest_point(Point(0, 5, 0));
        MINI_CHECK(std::abs(cp2[0]) < 1e-5 && std::abs(cp2[1] - 2.0) < 1e-5);

        std::vector<Point> ipts = {Point(0,0,0), Point(3,0,2), Point(6,0,-3), Point(8,0,0)};
        NurbsCurve ic = NurbsCurve::create_interpolated(ipts, CurveNurbsKnotStyle::Chord, CurveInterpStyle::Occt);
        Point pc = ic.closest_point(Point(2, -1, 0));
        MINI_CHECK(TOLERANCE.is_point_close(pc, Point(0.5808155659, 0.0, 0.9672315271)));

        NurbsCurve c0 = NurbsCurve::create_from_parameters(
        {Point(0,0,0),Point(3,6,0),Point(6,-3,3),Point(10,0,0)}, {1,1,1,1}, {0,1}, {4,4}, 3);
        NurbsCurve c1 = NurbsCurve::create_from_parameters(
        {Point(6,-3,0),Point(3,1,0),Point(6,6,3),Point(3,12,0)}, {1,1,1,1}, {0,1}, {4,4}, 3);
        auto [u, v] = c0.closest_parameters_curve(c1);
        MINI_CHECK(std::abs(u - 0.4757682937) < 1e-6 && std::abs(v - 0.3366914716) < 1e-6);
        auto [pa, pb] = c0.closest_points_curve(c1);
        MINI_CHECK(TOLERANCE.is_point_close(pa, Point(4.389607399, 1.285537564, 1.067964425)));
        MINI_CHECK(TOLERANCE.is_point_close(pb, Point(4.552264625, 1.380381100, 0.676740741)));
    }
}
