#include "mini_test.h"
#include "line.h"
#include "line.pb.h"
#include "color.h"
#include "point.h"
#include "tolerance.h"
#include "vector.h"
#include "xform.h"
#include <string>
#include <tuple>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Line", "Constructor") {

        Line line(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        line[0] = 10.0;
        line[1] = 20.0;
        line[2] = 30.0;
        line[3] = 40.0;
        line[4] = 50.0;
        line[5] = 60.0;

        const double x0 = line[0];
        const double y0 = line[1];
        const double z0 = line[2];
        const double x1 = line[3];
        const double y1 = line[4];
        const double z1 = line[5];

        const std::string lstr = line.str();
        const std::string lrepr = line.repr();

        const Line lcopy = line;
        const Line lother(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

        Line lmult = line;
        lmult *= 2.0;
        Line ldiv = line;
        ldiv /= 2.0;
        Line ladd = line;
        ladd += Vector(1.0, 1.0, 1.0);
        Line lsub = line;
        lsub -= Vector(1.0, 1.0, 1.0);

        const Line rmul = line * 2.0;
        const Line rdiv = line / 2.0;
        const Line radd = line + Vector(1.0, 1.0, 1.0);
        const Line rdif = line - Vector(1.0, 1.0, 1.0);

        const Line lneg(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        const Line neg = -lneg;

        const Point p0(1.0, 2.0, 3.0);
        const Point p1(4.0, 5.0, 6.0);
        const Line l2p = Line::from_points(p0, p1);

        const Point pv(1.0, 2.0, 3.0);
        const Vector vv(3.0, 4.0, 5.0);
        const Line l_pv = Line::from_point_and_vector(pv, vv);

        const Point pd(0.0, 0.0, 0.0);
        const Vector dd(1.0, 0.0, 0.0);
        const Line l_pdl = Line::from_point_direction_length(pd, dd, 5.0);

        Line lc(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        lc.linecolor = Color(1.0f, 0.0f, 0.0f, 1.0f, "red");
        lc.width = 2.5;

        const Line lwn = Line::with_name("custom", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

        Point ms;
        Point me;
        Line::get_middle_line(
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(0.0, 2.0, 0.0),
            Point(2.0, 2.0, 0.0),
            ms,
            me
        );

        MINI_CHECK(line.name == "my_line");
        MINI_CHECK(line[0] == 10.0 && line[1] == 20.0 && line[2] == 30.0);
        MINI_CHECK(line.width == 1.0);
        MINI_CHECK(line.linecolor == Color::black());
        MINI_CHECK(line.guid() != "");
        MINI_CHECK(x0 == 10.0 && y0 == 20.0 && z0 == 30.0 && x1 == 40.0 && y1 == 50.0 && z1 == 60.0);
        MINI_CHECK(lstr == "10.000000, 20.000000, 30.000000, 40.000000, 50.000000, 60.000000");
        MINI_CHECK(lrepr == "Line(my_line, 10.000000, 20.000000, 30.000000, 40.000000, 50.000000, 60.000000, Color(black, 0.0, 0.0, 0.0, 1.0), 1.000000)");
        MINI_CHECK(lcopy == line && lcopy.guid() != line.guid());
        MINI_CHECK(lother == line && lneg != line);
        MINI_CHECK(lmult[0] == 20.0 && lmult[3] == 80.0);
        MINI_CHECK(ldiv[0] == 5.0 && ldiv[3] == 20.0);
        MINI_CHECK(ladd[0] == 11.0 && ladd[3] == 41.0);
        MINI_CHECK(lsub[0] == 9.0 && lsub[3] == 39.0);
        MINI_CHECK(rmul[0] == 20.0 && rmul[3] == 80.0);
        MINI_CHECK(rdiv[0] == 5.0 && rdiv[3] == 20.0);
        MINI_CHECK(radd[0] == 11.0 && radd[3] == 41.0);
        MINI_CHECK(rdif[0] == 9.0 && rdif[3] == 39.0);
        MINI_CHECK(neg[0] == 4.0 && neg[1] == 5.0 && neg[2] == 6.0);
        MINI_CHECK(neg[3] == 1.0 && neg[4] == 2.0 && neg[5] == 3.0);
        MINI_CHECK(l2p[0] == 1.0 && l2p[3] == 4.0);
        MINI_CHECK(l_pv[0] == 1.0 && l_pv[1] == 2.0 && l_pv[2] == 3.0);
        MINI_CHECK(l_pv[3] == 4.0 && l_pv[4] == 6.0 && l_pv[5] == 8.0);
        MINI_CHECK(l_pdl[0] == 0.0 && l_pdl[3] == 5.0);
        MINI_CHECK(lc.linecolor[0] == 1.0f && lc.linecolor[1] == 0.0f && lc.width == 2.5);
        MINI_CHECK(lwn.name == "custom" && lwn[3] == 1.0);
        MINI_CHECK(TOLERANCE.is_close(ms[1], 1.0) && TOLERANCE.is_close(me[1], 1.0));
    }

    MINI_TEST("Line", "Transformation") {

        Line line(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        const Xform xform = Xform::translation(10.0, 0.0, 0.0);
        const Line moved = line.transformed(xform);
        line.transform(xform);

        MINI_CHECK(moved[0] == 10.0 && moved[3] == 11.0);
        MINI_CHECK(line[0] == 10.0 && line[3] == 11.0);
    }

    MINI_TEST("Line", "Json Roundtrip") {

        Line line(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
        line.name = "test_line";
        line.dash = {3.0, 2.0};

        const nlohmann::ordered_json j = line.jsondump();
        const Line loaded_j = Line::jsonload(j);

        const std::string s = line.file_json_dumps();
        const Line loaded_s = Line::file_json_loads(s);

        const std::string fname = "serialization/test_line.json";
        line.file_json_dump(fname);
        const Line loaded = Line::file_json_load(fname);

        MINI_CHECK(loaded_j.name == "test_line");
        MINI_CHECK(TOLERANCE.is_close(loaded_j[0], 42.1));
        MINI_CHECK(loaded_s.name == "test_line");
        MINI_CHECK(TOLERANCE.is_close(loaded_s[0], 42.1));
        MINI_CHECK(loaded.name == "test_line");
        MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK(TOLERANCE.is_close(loaded[3], 168.4));
        MINI_CHECK(TOLERANCE.is_close(loaded[4], 210.5));
        MINI_CHECK(TOLERANCE.is_close(loaded[5], 252.6));
        MINI_CHECK(loaded.dash == std::vector<double>({3.0, 2.0}));
    }

    MINI_TEST("Line", "Protobuf Roundtrip") {

        Line line(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
        line.name = "test_line";
        line.dash = {3.0, 2.0};

        const std::string guid = line.guid();
        const std::string s = line.pb_dumps();
        const Line loaded_s = Line::pb_loads(s);

        const std::string fname = "serialization/test_line.bin";
        line.pb_dump(fname);
        const Line loaded = Line::pb_load(fname);
        const Line converted = Line::from_proto(line.to_proto());

        MINI_CHECK(loaded_s.name == "test_line");
        MINI_CHECK(TOLERANCE.is_close(loaded_s[0], 42.1));
        MINI_CHECK(loaded_s.guid() == guid);
        MINI_CHECK(loaded.name == "test_line");
        MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK(TOLERANCE.is_close(loaded[3], 168.4));
        MINI_CHECK(TOLERANCE.is_close(loaded[4], 210.5));
        MINI_CHECK(TOLERANCE.is_close(loaded[5], 252.6));
        MINI_CHECK(loaded.dash == std::vector<double>({3.0, 2.0}));
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(converted == line);
        MINI_CHECK(converted.guid() == guid);
    }

    MINI_TEST("Line", "Length") {

        const Line line(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        const double ln = line.length();
        const double lsq = line.squared_length();

        MINI_CHECK(TOLERANCE.is_close(ln, 5.0));
        MINI_CHECK(TOLERANCE.is_close(lsq, 25.0));
    }

    MINI_TEST("Line", "To Vector") {

        const Line line(1.0, 2.0, 3.0, 4.0, 6.0, 9.0);
        const Vector v = line.to_vector();

        MINI_CHECK(v[0] == 3.0 && v[1] == 4.0 && v[2] == 6.0);
    }

    MINI_TEST("Line", "To Direction") {

        const Line line(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        const Vector d = line.to_direction();

        MINI_CHECK(TOLERANCE.is_close(d[0], 0.6));
        MINI_CHECK(TOLERANCE.is_close(d[1], 0.8));
        MINI_CHECK(TOLERANCE.is_close(d[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(d.magnitude(), 1.0));
    }

    MINI_TEST("Line", "Point At") {

        const Line line(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        const Point ps = line.point_at(0.0);
        const Point pm = line.point_at(0.5);
        const Point pe = line.point_at(1.0);

        MINI_CHECK(ps[0] == 0.0 && ps[1] == 0.0 && ps[2] == 0.0);
        MINI_CHECK(pm[0] == 5.0 && pm[1] == 5.0 && pm[2] == 5.0);
        MINI_CHECK(pe[0] == 10.0 && pe[1] == 10.0 && pe[2] == 10.0);
    }

    MINI_TEST("Line", "Closest Point") {

        const Line line(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        const Point p1(5.0, 5.0, 0.0);
        const Point p2(-5.0, 0.0, 0.0);
        const Point p3(15.0, 0.0, 0.0);

        double t1 = 0.0;
        double t2 = 0.0;
        double t3 = 0.0;
        Point cp1;
        Point cp2;
        Point cp3;
        std::tie(t1, cp1) = line.closest_point(p1);
        std::tie(t2, cp2) = line.closest_point(p2);
        std::tie(t3, cp3) = line.closest_point(p3);

        MINI_CHECK(cp1[0] == 5.0 && cp1[1] == 0.0 && cp1[2] == 0.0);
        MINI_CHECK(cp2[0] == 0.0 && cp2[1] == 0.0 && cp2[2] == 0.0);
        MINI_CHECK(cp3[0] == 10.0 && cp3[1] == 0.0 && cp3[2] == 0.0);
        MINI_CHECK(TOLERANCE.is_close(t1, 0.5));
        MINI_CHECK(TOLERANCE.is_close(t2, 0.0));
        MINI_CHECK(TOLERANCE.is_close(t3, 1.0));
    }

    MINI_TEST("Line", "Closest Point Unlimited") {

        const Line line(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        const std::pair<double, Point> before = line.closest_point(Point(-5.0, 2.0, 0.0), false);
        const std::pair<double, Point> after = line.closest_point(Point(15.0, 3.0, 0.0), false);

        MINI_CHECK(TOLERANCE.is_close(before.first, -0.5));
        MINI_CHECK(TOLERANCE.is_point_close(before.second, Point(-5.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_close(after.first, 1.5));
        MINI_CHECK(TOLERANCE.is_point_close(after.second, Point(15.0, 0.0, 0.0)));
    }

    MINI_TEST("Line", "Start End Center") {

        const Line line(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        const Point start = line.start();
        const Point end = line.end();
        const Point center = line.center();

        MINI_CHECK(start[0] == 1.0 && start[1] == 2.0 && start[2] == 3.0);
        MINI_CHECK(end[0] == 4.0 && end[1] == 5.0 && end[2] == 6.0);
        MINI_CHECK(center[0] == 2.5 && center[1] == 3.5 && center[2] == 4.5);
    }

    MINI_TEST("Line", "Fit Points") {

        const std::vector<Point> fit_pts = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.5),
            Point(2.0, 2.0, 1.0),
            Point(3.0, 3.0, 1.5),
        };
        const Line l_fit = Line::fit_points(fit_pts);

        MINI_CHECK(l_fit.length() > 0.0);

        const Line l_vertical = Line::fit_points({
            Point(0.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 2.0, 0.0),
            Point(0.0, 3.0, 0.0),
        });

        MINI_CHECK(std::fabs(l_vertical.to_direction()[1]) > 0.99);

        const Line l_skew = Line::fit_points({
            Point(3.0, 0.0, 0.0),
            Point(-3.0, 0.0, 0.0),
            Point(0.0, 2.4, 2.4),
            Point(0.0, -2.4, -2.4),
        });
        const Vector skew = l_skew.to_direction();

        MINI_CHECK(TOLERANCE.is_close(skew[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(std::fabs(skew[1]), std::sqrt(0.5)));
        MINI_CHECK(TOLERANCE.is_close(skew[1], skew[2]));
    }

    MINI_TEST("Line", "Fit Points Uneven") {

        const Line line = Line::fit_points({
            Point(0.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 9.0, 0.0),
        });

        MINI_CHECK(TOLERANCE.is_close(line.length(), 9.0));
        MINI_CHECK(TOLERANCE.is_point_close(line.start(), Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(line.end(), Point(0.0, 9.0, 0.0)));
    }

    MINI_TEST("Line", "Subdivide") {

        const Line line(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        const std::vector<Point> pts = line.subdivide(3);
        const std::vector<Point> pts_dist = line.subdivide_by_distance(2.5);

        MINI_CHECK(pts.size() == 3);
        MINI_CHECK(pts[0][0] == 0.0);
        MINI_CHECK(pts[1][0] == 5.0);
        MINI_CHECK(pts[2][0] == 10.0);
        MINI_CHECK(pts_dist.size() == 5);
        MINI_CHECK(TOLERANCE.is_close(pts_dist[0][0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pts_dist[1][0], 2.5));
        MINI_CHECK(TOLERANCE.is_close(pts_dist[4][0], 10.0));
    }

    MINI_TEST("Line", "Overlap") {

        const Line l0 = Line::from_points(Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0));
        const Line l1 = Line::from_points(Point(5.0, 0.0, 0.0), Point(15.0, 0.0, 0.0));
        Line out;
        const bool ok = l0.overlap(l1, out);

        MINI_CHECK(ok);
        MINI_CHECK(TOLERANCE.is_close(out.start()[0], 5.0));
        MINI_CHECK(TOLERANCE.is_close(out.end()[0], 10.0));
    }

    MINI_TEST("Line", "Overlap Average") {

        const Line l0 = Line::from_points(Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0));
        const Line l1 = Line::from_points(Point(5.0, 0.0, 0.0), Point(15.0, 0.0, 0.0));
        Line out;
        const bool ok = l0.overlap_average(l1, out);

        MINI_CHECK(ok);
        MINI_CHECK(TOLERANCE.is_close(out.start()[0], 5.0));
        MINI_CHECK(TOLERANCE.is_close(out.end()[0], 10.0));
    }

    MINI_TEST("Line", "Extend") {

        Line line = Line::from_points(Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0));
        line.extend(1.0, 2.0);

        MINI_CHECK(TOLERANCE.is_close(line.start()[0], -1.0));
        MINI_CHECK(TOLERANCE.is_close(line.end()[0], 12.0));
    }

    MINI_TEST("Line", "Extend Keeps Properties") {

        Line line = Line::from_points(Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0));
        line.name = "beam";
        line.width = 3.0;
        line.dash = {2.0, 1.0};
        line.linecolor = Color::red();
        const std::string guid = line.guid();
        line.extend(1.0, 2.0);

        MINI_CHECK(line.name == "beam");
        MINI_CHECK(line.width == 3.0);
        MINI_CHECK(line.dash == std::vector<double>({2.0, 1.0}));
        MINI_CHECK(line.linecolor == Color::red());
        MINI_CHECK(line.guid() == guid);
    }

}
