#include "mini_test.h"
#include "polyline.h"
#include "polyline.pb.h"
#include "color.h"
#include "line.h"
#include "plane.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Polyline", "Constructor") {

        const Point p0(0.0, 0.0, 0.0);
        const Point p1(1.0, 0.0, 0.0);
        const Point p2(1.0, 1.0, 0.0);
        const Point p3(0.0, 1.0, 0.0);
        const Polyline pl({p0, p1, p2, p3});
        const size_t point_count = pl.len();
        const size_t segment_count = pl.segment_count();
        const bool is_empty = pl.is_empty();
        const Point pt = pl.get_point(1);
        const Point pt_idx = pl[1];
        Polyline pl_copy = pl;
        pl_copy.set_point(0, Point(5.0, 6.0, 7.0));
        const std::string plstr = pl.str();
        const std::string plrepr = pl.repr();
        const Polyline plcopy = pl;
        const Polyline plother({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });

        Polyline plmult = pl;
        plmult *= 2.0;
        Polyline pldiv = pl;
        pldiv /= 2.0;
        Polyline pladd = pl;
        pladd += Vector(1.0, 1.0, 1.0);
        Polyline plsub = pl;
        plsub -= Vector(1.0, 1.0, 1.0);

        const Polyline rmul = pl * 2.0;
        const Polyline rdiv = pl / 2.0;
        const Polyline radd = pl + Vector(1.0, 1.0, 1.0);
        const Polyline rdif = pl - Vector(1.0, 1.0, 1.0);

        const Polyline plneg({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
        });
        const Polyline neg = -plneg;

        Polyline plc({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        plc.linecolor = Color(1.0f, 0.0f, 0.0f, 1.0f, "red");
        plc.width = 2.5;

        MINI_CHECK(pl.name == "my_polyline" && !pl.guid().empty() && point_count == 4);
        MINI_CHECK(segment_count == 3 && !is_empty);
        MINI_CHECK(pt[0] == 1.0 && pt[1] == 0.0 && pt[2] == 0.0);
        MINI_CHECK(pt_idx[0] == 1.0 && pl_copy[0][0] == 5.0 && pl_copy[0][1] == 6.0);
        MINI_CHECK(plstr.find("(0, 0, 0)") != std::string::npos);
        MINI_CHECK(plrepr.find("Polyline(my_polyline") != std::string::npos);
        MINI_CHECK(plrepr.find("4 points") != std::string::npos);
        MINI_CHECK(plcopy == plother);
        MINI_CHECK(plcopy.guid() != pl.guid());
        MINI_CHECK(plmult.get_point(1)[0] == 2.0);
        MINI_CHECK(pldiv.get_point(1)[0] == 0.5);
        MINI_CHECK(pladd.get_point(0)[0] == 1.0 && pladd.get_point(0)[1] == 1.0);
        MINI_CHECK(plsub.get_point(0)[0] == -1.0 && plsub.get_point(0)[1] == -1.0);
        MINI_CHECK(rmul.get_point(1)[0] == 2.0);
        MINI_CHECK(rdiv.get_point(1)[0] == 0.5);
        MINI_CHECK(radd.get_point(0)[0] == 1.0 && radd.get_point(0)[1] == 1.0);
        MINI_CHECK(rdif.get_point(0)[0] == -1.0 && rdif.get_point(0)[1] == -1.0);
        MINI_CHECK(neg.get_point(0)[0] == 3.0 && neg.get_point(3)[0] == 0.0);
        MINI_CHECK(plc.linecolor[0] == 1.0f && plc.linecolor[1] == 0.0f && plc.width == 2.5);
    }

    MINI_TEST("Polyline", "From Coords") {

        const std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0};
        const Polyline pl = Polyline::from_coords(coords);

        MINI_CHECK(pl.point_count() == 3);
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(0)[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(1)[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(2)[1], 1.0));
    }

    MINI_TEST("Polyline", "From Sides") {

        const Polyline sq = Polyline::from_sides(4, 1.0, false);
        const Polyline sq_closed = Polyline::from_sides(4, 1.0, true);

        MINI_CHECK(sq.point_count() == 4);
        MINI_CHECK(sq_closed.point_count() == 5);
        MINI_CHECK(sq_closed.is_closed());
    }

    MINI_TEST("Polyline", "Rectangle") {

        const Point o(0.0, 0.0, 0.0);
        const Polyline r = Polyline::rectangle(o, Vector::x_axis(), Vector::y_axis(), 2.0, 1.0, true);

        MINI_CHECK(r.point_count() == 5);
        MINI_CHECK(r.is_closed());
        MINI_CHECK(TOLERANCE.is_close(r.get_point(2)[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(r.get_point(2)[1], 1.0));
    }

    MINI_TEST("Polyline", "Transformation") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const Xform pl_xf = Xform::translation(10.0, 0.0, 0.0);
        const Polyline pl_transformed = pl.transformed(pl_xf);
        pl.transform(pl_xf);

        MINI_CHECK(pl_transformed.get_point(0)[0] == 10.0 && pl_transformed.get_point(1)[0] == 11.0);
        MINI_CHECK(pl.get_point(0)[0] == 10.0 && pl.get_point(1)[0] == 11.0);
    }

    MINI_TEST("Polyline", "Json Roundtrip") {

        Polyline pl({
            Point(1.0, 2.0, 3.0),
            Point(4.0, 5.0, 6.0),
            Point(7.0, 8.0, 9.0),
            Point(10.0, 11.0, 12.0),
        });
        pl.name = "test_polyline";
        pl.dash = {3.0, 2.0};

        const nlohmann::ordered_json j = pl.jsondump();
        const Polyline loaded_j = Polyline::jsonload(j);

        const std::string s = pl.file_json_dumps();
        const Polyline loaded_s = Polyline::file_json_loads(s);

        const std::string fname = "serialization/test_polyline.json";
        pl.file_json_dump(fname);
        const Polyline loaded = Polyline::file_json_load(fname);

        MINI_CHECK(loaded_j.name == "test_polyline");
        MINI_CHECK(TOLERANCE.is_close(loaded_j.get_point(0)[0], 1.0));
        MINI_CHECK(loaded_s.name == "test_polyline");
        MINI_CHECK(TOLERANCE.is_close(loaded_s.get_point(0)[0], 1.0));
        MINI_CHECK(loaded.name == "test_polyline");
        MINI_CHECK(loaded.len() == 4);
        MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded.get_point(1)[1], 5.0));
        MINI_CHECK(TOLERANCE.is_close(loaded.get_point(2)[2], 9.0));
        MINI_CHECK(loaded.dash == std::vector<double>({3.0, 2.0}));
        MINI_CHECK(loaded.guid() == pl.guid());
    }

    MINI_TEST("Polyline", "Protobuf Roundtrip") {

        Polyline pl({
            Point(1.0, 2.0, 3.0),
            Point(4.0, 5.0, 6.0),
            Point(7.0, 8.0, 9.0),
            Point(10.0, 11.0, 12.0),
        });
        pl.name = "test_polyline";
        pl.dash = {3.0, 2.0};

        const std::string guid = pl.guid();
        const std::string s = pl.pb_dumps();
        const Polyline loaded_s = Polyline::pb_loads(s);

        const std::string fname = "serialization/test_polyline.bin";
        pl.pb_dump(fname);
        const Polyline loaded = Polyline::pb_load(fname);
        const Polyline converted = Polyline::from_proto(pl.to_proto());

        MINI_CHECK(loaded_s.name == "test_polyline");
        MINI_CHECK(TOLERANCE.is_close(loaded_s.get_point(0)[0], 1.0));
        MINI_CHECK(loaded_s.guid() == guid);
        MINI_CHECK(loaded.name == "test_polyline");
        MINI_CHECK(loaded.len() == 4);
        MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded.get_point(1)[1], 5.0));
        MINI_CHECK(TOLERANCE.is_close(loaded.get_point(2)[2], 9.0));
        MINI_CHECK(loaded.dash == std::vector<double>({3.0, 2.0}));
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(converted == pl);
        MINI_CHECK(converted.guid() == guid);
    }

    MINI_TEST("Polyline", "Length") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const double ln = pl.length();
        const double mag_sq = pl.length_squared();

        MINI_CHECK(TOLERANCE.is_close(ln, 3.0));
        MINI_CHECK(TOLERANCE.is_close(mag_sq, 3.0));
    }

    MINI_TEST("Polyline", "Center") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(0.0, 2.0, 0.0),
        });
        const Point c = pl.center();

        MINI_CHECK(TOLERANCE.is_close(c[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(c[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(c[2], 0.0));
    }

    MINI_TEST("Polyline", "Is Closed") {

        const Polyline open_pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const bool is_open = open_pl.is_closed();

        const Polyline closed_pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });
        const bool is_closed = closed_pl.is_closed();

        MINI_CHECK(!is_open);
        MINI_CHECK(is_closed);
    }

    MINI_TEST("Polyline", "Closed") {

        const Polyline open_pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const Polyline closed_from_open = open_pl.closed();

        const Polyline closed_pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });
        const Polyline closed_from_closed = closed_pl.closed();

        MINI_CHECK(closed_from_open.point_count() == 5);
        MINI_CHECK(closed_from_open.is_closed());
        MINI_CHECK(closed_from_closed.point_count() == 5);
        MINI_CHECK(closed_from_closed.is_closed());
    }

    MINI_TEST("Polyline", "Reverse") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
        });

        const Polyline rev = pl.reversed();
        const double orig_first = pl.get_point(0)[0];
        const double rev_first = rev.get_point(0)[0];

        pl.reverse();
        const double in_place_first = pl.get_point(0)[0];

        MINI_CHECK(orig_first == 0.0);
        MINI_CHECK(rev_first == 3.0);
        MINI_CHECK(in_place_first == 3.0);
    }

    MINI_TEST("Polyline", "Closest Point") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(0.0, 2.0, 0.0),
        });
        const Point test_pt(1.0, 1.0, 0.0);
        size_t edge_id = 0;
        Point closest;
        const double distance = pl.closest_distance_and_point(test_pt, edge_id, closest);

        MINI_CHECK(edge_id == 0);
        MINI_CHECK(TOLERANCE.is_close(closest[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(closest[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(distance, 1.0));
    }

    MINI_TEST("Polyline", "Closest Point To Line") {

        const Point line_start(0.0, 0.0, 0.0);
        const Point line_end(2.0, 0.0, 0.0);
        const Point pt(1.0, 1.0, 0.0);
        double t = 0.0;
        Polyline::closest_point_to_line(pt, line_start, line_end, t);

        MINI_CHECK(TOLERANCE.is_close(t, 0.5));
    }

    MINI_TEST("Polyline", "Line Line Overlap") {

        const Point s0(0.0, 0.0, 0.0);
        const Point e0(2.0, 0.0, 0.0);
        const Point s1(1.0, 0.0, 0.0);
        const Point e1(3.0, 0.0, 0.0);
        Point os;
        Point oe;
        const bool overlaps = Polyline::line_line_overlap(s0, e0, s1, e1, os, oe);

        MINI_CHECK(overlaps);
        MINI_CHECK(TOLERANCE.is_close(os[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(oe[0], 2.0));

        const Point s2(5.0, 0.0, 0.0);
        const Point e2(6.0, 0.0, 0.0);
        Point os2;
        Point oe2;
        const bool no_overlap = Polyline::line_line_overlap(s0, e0, s2, e2, os2, oe2);

        MINI_CHECK(!no_overlap);
    }

    MINI_TEST("Polyline", "Line Line Average") {

        const Point s0(0.0, 0.0, 0.0);
        const Point e0(2.0, 0.0, 0.0);
        const Point s1(0.0, 2.0, 0.0);
        const Point e1(2.0, 2.0, 0.0);
        Point os;
        Point oe;
        Polyline::line_line_average(s0, e0, s1, e1, os, oe);

        MINI_CHECK(TOLERANCE.is_close(os[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(os[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(oe[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(oe[1], 1.0));
    }

    MINI_TEST("Polyline", "Line Line Overlap Average") {

        const Point s0(0.0, 0.0, 0.0);
        const Point e0(2.0, 0.0, 0.0);
        const Point s1(1.0, 2.0, 0.0);
        const Point e1(3.0, 2.0, 0.0);
        Point os;
        Point oe;
        Polyline::line_line_overlap_average(s0, e0, s1, e1, os, oe);

        MINI_CHECK(TOLERANCE.is_close(os[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(oe[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(os[1], 1.0));
    }

    MINI_TEST("Polyline", "Line From Projected Points") {

        const Point s(0.0, 0.0, 0.0);
        const Point e(4.0, 0.0, 0.0);
        const std::vector<Point> pts = {
            Point(1.0, 1.0, 0.0),
            Point(3.0, -1.0, 0.0),
        };
        Point os;
        Point oe;
        const bool ok = Polyline::line_from_projected_points(s, e, pts, os, oe);

        MINI_CHECK(ok);
        MINI_CHECK(TOLERANCE.is_close(os[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(oe[0], 3.0));
    }

    MINI_TEST("Polyline", "Point In Polygon 2d") {

        const Polyline sq({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });

        MINI_CHECK(sq.point_in_polygon_2d(Point(0.5, 0.5, 0.0)));
        MINI_CHECK(!sq.point_in_polygon_2d(Point(2.0, 2.0, 0.0)));
    }

    MINI_TEST("Polyline", "Trim Rectangles By Plane") {

        Polyline first({
            Point(0.0, 0.0, 0.0),
            Point(4.0, 0.0, 0.0),
            Point(4.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });
        Polyline second({
            Point(0.0, 0.0, 1.0),
            Point(4.0, 0.0, 1.0),
            Point(4.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
            Point(0.0, 0.0, 1.0),
        });
        const Plane plane = Plane::from_point_normal(Point(3.0, 0.0, 0.0), Vector(-1.0, 0.0, 0.0));
        const bool ok = Polyline::trim_rectangles_by_plane(first, second, plane);

        MINI_CHECK(ok);
        MINI_CHECK(TOLERANCE.is_close(first[1][0], 3.0));
        MINI_CHECK(TOLERANCE.is_close(first[2][0], 3.0));
        MINI_CHECK(TOLERANCE.is_close(second[1][0], 3.0));
        MINI_CHECK(TOLERANCE.is_close(first[0][0], 0.0));
    }

    MINI_TEST("Polyline", "Extend Segment") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
        });
        pl.extend_segment(0, 0.5, 0.5, 0.0, 0.0);
        const double first = pl.get_point(0)[0];
        const double second = pl.get_point(1)[0];

        MINI_CHECK(TOLERANCE.is_close(first, -0.5));
        MINI_CHECK(TOLERANCE.is_close(second, 1.5));
    }

    MINI_TEST("Polyline", "Extend Segment Equally") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
        });
        pl.extend_segment_equally(0, 0.5, 0.0);
        const double first = pl.get_point(0)[0];
        const double second = pl.get_point(1)[0];

        MINI_CHECK(TOLERANCE.is_close(first, -0.5));
        MINI_CHECK(TOLERANCE.is_close(second, 1.5));
    }

    MINI_TEST("Polyline", "Extend Line Segment") {

        Point start(1.0, 0.0, 0.0);
        Point end(3.0, 0.0, 0.0);
        Polyline::extend_line_segment(start, end, 0.5, 0.5);

        MINI_CHECK(TOLERANCE.is_close(start[0], 0.5));
        MINI_CHECK(TOLERANCE.is_close(end[0], 3.5));
    }

    MINI_TEST("Polyline", "Shrink Line Segment") {

        Point start(0.0, 0.0, 0.0);
        Point end(10.0, 0.0, 0.0);
        Polyline::shrink_line_segment(start, end, 0.1);

        MINI_CHECK(TOLERANCE.is_close(start[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(end[0], 9.0));
    }

    MINI_TEST("Polyline", "Get Points") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const std::vector<Point> points = pl.get_points();

        MINI_CHECK(points.size() == 4);
        MINI_CHECK(TOLERANCE.is_close(points[0][0], 0.0) && TOLERANCE.is_close(points[0][1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(points[1][0], 1.0) && TOLERANCE.is_close(points[1][1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(points[2][0], 1.0) && TOLERANCE.is_close(points[2][1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(points[3][0], 0.0) && TOLERANCE.is_close(points[3][1], 1.0));
    }

    MINI_TEST("Polyline", "Get Lines") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const std::vector<Line> lines = pl.get_lines();

        MINI_CHECK(lines.size() == 3);
        MINI_CHECK(TOLERANCE.is_close(lines[0][0], 0.0) && TOLERANCE.is_close(lines[0][3], 1.0));
        MINI_CHECK(TOLERANCE.is_close(lines[1][0], 1.0) && TOLERANCE.is_close(lines[1][4], 1.0));
        MINI_CHECK(TOLERANCE.is_close(lines[2][0], 1.0) && TOLERANCE.is_close(lines[2][3], 0.0));
    }

    MINI_TEST("Polyline", "Add Point") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
        });
        pl.add_point(Point(2.0, 0.0, 0.0));

        MINI_CHECK(pl.point_count() == 3);
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(2)[0], 2.0));
    }

    MINI_TEST("Polyline", "Insert Point") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
        });
        pl.insert_point(1, Point(1.0, 0.0, 0.0));

        MINI_CHECK(pl.point_count() == 3);
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(1)[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(2)[0], 2.0));
    }

    MINI_TEST("Polyline", "Remove Point") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
        });
        Point out;
        const bool removed = pl.remove_point(1, out);

        MINI_CHECK(removed);
        MINI_CHECK(pl.point_count() == 2);
        MINI_CHECK(TOLERANCE.is_close(out[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(1)[0], 2.0));
    }

    MINI_TEST("Polyline", "Shift") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
        });
        pl.shift(1);
        const double first_after_shift = pl.get_point(0)[0];
        pl.shift(-1);
        const double first_after_unshift = pl.get_point(0)[0];

        MINI_CHECK(TOLERANCE.is_close(first_after_shift, 1.0));
        MINI_CHECK(TOLERANCE.is_close(first_after_unshift, 0.0));
    }

    MINI_TEST("Polyline", "Point At") {

        const Point start(0.0, 0.0, 0.0);
        const Point end(2.0, 0.0, 0.0);
        const Point mid = Polyline::point_at(start, end, 0.5);
        const Point quarter = Polyline::point_at(start, end, 0.25);

        MINI_CHECK(TOLERANCE.is_close(mid[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(quarter[0], 0.5));
    }

    MINI_TEST("Polyline", "Is Clockwise") {

        const Polyline cw_pl({
            Point(0.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(1.0, 0.0, 0.0),
        });
        const Polyline ccw_pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const Plane plane;

        MINI_CHECK(cw_pl.is_clockwise(plane));
        MINI_CHECK(!ccw_pl.is_clockwise(plane));
    }

    MINI_TEST("Polyline", "Convex Corners") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        std::vector<bool> corners;
        pl.get_convex_corners(corners);

        MINI_CHECK(corners.size() == 4);
    }

    MINI_TEST("Polyline", "Tween") {

        const Polyline pl0({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        const Polyline pl1({
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0),
            Point(2.0, 1.0, 0.0),
        });
        const Polyline tweened = Polyline::tween_two_polylines(pl0, pl1, 0.5);

        MINI_CHECK(TOLERANCE.is_close(tweened.get_point(0)[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(tweened.get_point(1)[0], 2.0));
    }

    MINI_TEST("Polyline", "Average Plane") {

        const Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(0.0, 2.0, 0.0),
        });
        Point origin;
        Vector x_axis;
        Vector y_axis;
        Vector z_axis;
        pl.get_average_plane(origin, x_axis, y_axis, z_axis);
        Point fast_origin;
        Plane fast_plane;
        pl.get_fast_plane(fast_origin, fast_plane);

        MINI_CHECK(TOLERANCE.is_close(origin[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(origin[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(std::abs(z_axis[2]), 1.0));
        MINI_CHECK(fast_origin[0] >= 0.0);
    }

    MINI_TEST("Polyline", "Interpolate Points") {

        const Point a(0.0, 0.0, 0.0);
        const Point b(4.0, 0.0, 0.0);

        const std::vector<Point> pts0 = Polyline::interpolate_points(a, b, 3, 0);
        const std::vector<Point> pts1 = Polyline::interpolate_points(a, b, 3, 1);
        const std::vector<Point> pts2 = Polyline::interpolate_points(a, b, 3, 2);

        MINI_CHECK(pts0.size() == 3);
        MINI_CHECK(TOLERANCE.is_close(pts0[0][0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pts0[1][0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pts0[2][0], 3.0));
        MINI_CHECK(pts1.size() == 5);
        MINI_CHECK(TOLERANCE.is_close(pts1[0][0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pts1[4][0], 4.0));
        MINI_CHECK(pts2.size() == 4);
        MINI_CHECK(TOLERANCE.is_close(pts2[0][0], 0.0));
    }

    MINI_TEST("Polyline", "Quick Hull") {

        const Polyline poly({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(0.0, 2.0, 0.0),
            Point(1.0, 1.0, 0.0),
        });
        const Polyline hull = Polyline::quick_hull(poly);

        MINI_CHECK(hull.point_count() == 4);
    }

    MINI_TEST("Polyline", "Bounding Rectangle") {

        const Polyline poly({
            Point(0.0, 0.0, 0.0),
            Point(4.0, 0.0, 0.0),
            Point(4.0, 3.0, 0.0),
            Point(0.0, 3.0, 0.0),
        });
        const std::optional<Polyline> rect = Polyline::bounding_rectangle(poly);

        MINI_CHECK(rect.has_value() && rect->point_count() == 5);
        MINI_CHECK(TOLERANCE.is_close(rect->get_point(0)[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(rect->get_point(0)[0], rect->get_point(4)[0]));
    }

    MINI_TEST("Polyline", "Grid Of Points In Polygon") {

        const Polyline poly({
            Point(0.0, 0.0, 0.0),
            Point(4.0, 0.0, 0.0),
            Point(4.0, 4.0, 0.0),
            Point(0.0, 4.0, 0.0),
        });
        const std::vector<Point> pts = Polyline::grid_of_points_in_polygon(poly, 0.0, 1.0, 100);

        MINI_CHECK(pts.size() > 0);

        for (const Point& p : pts) {
            MINI_CHECK(p[0] >= 0.0 && p[0] <= 4.0);
            MINI_CHECK(p[1] >= 0.0 && p[1] <= 4.0);
        }
    }

    MINI_TEST("Polyline", "Polylabel") {

        const Polyline poly({
            Point(0.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
            Point(0.0, 10.0, 0.0),
        });
        const std::vector<Polyline> polys = {poly};
        const std::tuple<Point, Plane, double> res = Polyline::polylabel(polys, 0.5);
        const Point& c = std::get<0>(res);
        const double r = std::get<2>(res);

        MINI_CHECK(std::abs(c[0] - 5.0) < 0.6);
        MINI_CHECK(std::abs(c[1] - 5.0) < 0.6);
        MINI_CHECK(std::abs(r - 5.0) < 0.6);
    }

    MINI_TEST("Polyline", "Polylabel Circle Division Points") {

        const Polyline poly({
            Point(0.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
            Point(0.0, 10.0, 0.0),
        });
        const std::vector<Polyline> polys = {poly};
        const Vector dir(0.0, 0.0, 0.0);
        const std::vector<Point> pts = Polyline::polylabel_circle_division_points(dir, polys, 4, 0.5, 1.0, true);

        MINI_CHECK(pts.size() == 4);

        for (const Point& p : pts)
            MINI_CHECK(std::abs(p[2]) < 1e-6);
    }

    MINI_TEST("Polyline", "Boolean Op") {

        const Polyline sq_a({
            Point(-1.0, -1.0, 0.0),
            Point(1.0, -1.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(-1.0, 1.0, 0.0),
        });
        const Polyline sq_b({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(0.0, 2.0, 0.0),
        });
        const Polyline sq_inside({
            Point(-0.5, -0.5, 0.0),
            Point(0.5, -0.5, 0.0),
            Point(0.5, 0.5, 0.0),
            Point(-0.5, 0.5, 0.0),
        });
        const Polyline sq_disjoint({
            Point(5.0, 5.0, 0.0),
            Point(6.0, 5.0, 0.0),
            Point(6.0, 6.0, 0.0),
            Point(5.0, 6.0, 0.0),
        });

        const std::vector<Polyline> isect = Polyline::boolean_op(sq_a, sq_b, 0);
        const std::vector<Polyline> uni = Polyline::boolean_op(sq_a, sq_b, 1);
        const std::vector<Polyline> diff = Polyline::boolean_op(sq_a, sq_b, 2);

        MINI_CHECK(isect.size() == 1);
        MINI_CHECK(isect[0].point_count() == 4);
        MINI_CHECK(uni.size() == 1);
        MINI_CHECK(uni[0].point_count() == 8);
        MINI_CHECK(diff.size() == 1);
        MINI_CHECK(diff[0].point_count() == 6);

        const std::vector<Polyline> isect_in = Polyline::boolean_op(sq_a, sq_inside, 0);
        const std::vector<Polyline> uni_in = Polyline::boolean_op(sq_a, sq_inside, 1);
        const std::vector<Polyline> diff_in = Polyline::boolean_op(sq_a, sq_inside, 2);

        MINI_CHECK(isect_in.size() == 1);
        MINI_CHECK(isect_in[0].point_count() == 4);
        MINI_CHECK(uni_in.size() == 1);
        MINI_CHECK(uni_in[0].point_count() == 4);
        MINI_CHECK(diff_in.size() == 1);
        MINI_CHECK(diff_in[0].point_count() == 4);

        const std::vector<Polyline> isect_dis = Polyline::boolean_op(sq_a, sq_disjoint, 0);
        const std::vector<Polyline> uni_dis = Polyline::boolean_op(sq_a, sq_disjoint, 1);
        const std::vector<Polyline> diff_dis = Polyline::boolean_op(sq_a, sq_disjoint, 2);

        MINI_CHECK(isect_dis.size() == 0);
        MINI_CHECK(uni_dis.size() == 2);
        MINI_CHECK(diff_dis.size() == 1);
    }

    MINI_TEST("Polyline", "Boolean Op Plane") {

        const Point origin(0.0, 0.0, 5.0);
        const Vector normal(0.0, 0.0, 1.0);
        const Plane plane = Plane::from_point_normal(origin, normal);
        const Polyline sq_a({
            Point(-1.0, -1.0, 5.0),
            Point(1.0, -1.0, 5.0),
            Point(1.0, 1.0, 5.0),
            Point(-1.0, 1.0, 5.0),
            Point(-1.0, -1.0, 5.0),
        });
        const Polyline sq_b({
            Point(0.0, 0.0, 5.0),
            Point(2.0, 0.0, 5.0),
            Point(2.0, 2.0, 5.0),
            Point(0.0, 2.0, 5.0),
            Point(0.0, 0.0, 5.0),
        });
        const std::vector<Polyline> isect = Polyline::boolean_op(sq_a, sq_b, plane, 0);
        const std::vector<Polyline> uni = Polyline::boolean_op(sq_a, sq_b, plane, 1);
        const std::vector<Polyline> diff = Polyline::boolean_op(sq_a, sq_b, plane, 2);

        MINI_CHECK(isect.size() == 1);
        MINI_CHECK(uni.size() == 1);
        MINI_CHECK(diff.size() == 1);

        for (const Point& p : isect[0].get_points())
            MINI_CHECK(TOLERANCE.is_close(p[2], 5.0));

        for (const Point& p : uni[0].get_points())
            MINI_CHECK(TOLERANCE.is_close(p[2], 5.0));

        for (const Point& p : diff[0].get_points())
            MINI_CHECK(TOLERANCE.is_close(p[2], 5.0));
    }

    MINI_TEST("Polyline", "Merge Collinear") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 1.0, 0.0),
        });
        pl.merge_collinear(Tolerance::APPROXIMATION);

        MINI_CHECK(pl.point_count() == 3);
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(1)[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(2)[1], 1.0));
    }

    MINI_TEST("Polyline", "Simplify Points") {

        std::vector<Point> pts;

        for (int i = 0; i < 100; i++) {
            const double x = static_cast<double>(i);
            const double y = std::sin(static_cast<double>(i) * 0.1) * 0.001;
            const double z = 0.0;
            pts.push_back(Point(x, y, z));
        }

        const std::vector<Point> result_tight = Polyline::simplify_points(pts, 0.0001);
        const std::vector<Point> result_loose = Polyline::simplify_points(pts, 0.01);
        const std::vector<Point> result_very_loose = Polyline::simplify_points(pts, 1.0);

        MINI_CHECK(result_tight.size() <= pts.size());
        MINI_CHECK(result_loose.size() <= result_tight.size());
        MINI_CHECK(result_very_loose.size() <= result_loose.size());
        MINI_CHECK(result_tight.front()[0] == pts.front()[0]);
        MINI_CHECK(result_tight.back()[0] == pts.back()[0]);
    }

    MINI_TEST("Polyline", "Simplify") {

        std::vector<Point> pts;

        for (int i = 0; i < 20; i++) {
            const double x = static_cast<double>(i);
            const double y = 0.0;
            const double z = 0.0;
            pts.push_back(Point(x, y, z));
        }

        const Polyline pl(pts);
        const Polyline result = pl.simplify(0.001);

        MINI_CHECK(result.point_count() == 2);
        MINI_CHECK(TOLERANCE.is_close(result.get_point(0)[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(result.get_point(1)[0], 19.0));
    }

    MINI_TEST("Polyline", "Simplify Collinear") {

        const std::vector<Point> pts = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 0.0, 0.0),
            Point(4.0, 0.0, 0.0),
        };
        const std::vector<Point> result = Polyline::simplify_points(pts, 0.001);

        MINI_CHECK(result.size() == 2);
        MINI_CHECK(TOLERANCE.is_close(result.front()[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(result.back()[0], 4.0));
    }

    MINI_TEST("Polyline", "Simplify Zigzag") {

        std::vector<Point> pts;

        for (int i = 0; i < 10; i++) {
            const double x = static_cast<double>(i);
            const double y = i % 2 == 1 ? 1.0 : 0.0;
            const double z = 0.0;
            pts.push_back(Point(x, y, z));
        }

        const std::vector<Point> result_tight = Polyline::simplify_points(pts, 0.1);
        const std::vector<Point> result_loose = Polyline::simplify_points(pts, 2.0);

        MINI_CHECK(result_tight.size() == 10);
        MINI_CHECK(result_loose.size() < result_tight.size());
    }

    MINI_TEST("Polyline", "Simplify Two Points") {

        const std::vector<Point> pts = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 1.0),
        };
        const std::vector<Point> result = Polyline::simplify_points(pts, 0.001);

        MINI_CHECK(result.size() == 2);
    }

    MINI_TEST("Polyline", "Translate") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        });
        pl.translate(Vector(5.0, 0.0, 0.0));

        MINI_CHECK(TOLERANCE.is_close(pl.get_point(0)[0], 5.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(2)[0], 6.0));
    }

    MINI_TEST("Polyline", "Extend Edge Equally") {

        Polyline pl({
            Point(0.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
            Point(0.0, 10.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });
        pl.extend_edge_equally(0, 1.0);

        MINI_CHECK(TOLERANCE.is_close(pl.get_point(0)[0], -1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(1)[0], 11.0));
        MINI_CHECK(TOLERANCE.is_close(pl.get_point(4)[0], -1.0));
    }

    MINI_TEST("Polyline", "Offset Sides") {

        Polyline square(std::vector<Point>{
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(0.0, 2.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });
        Polyline split(std::vector<Point>{
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0),
        });
        Polyline moved = square.offset_sides({1.0, 0.0, 0.0, 0.0});
        Polyline stepped = split.offset_sides({1.0, 2.0, 0.0, 0.0, 0.0});

        MINI_CHECK(moved.point_count() == 5);
        MINI_CHECK(moved.is_closed());
        MINI_CHECK(TOLERANCE.is_close(moved.get_point(0)[1], -1.0));
        MINI_CHECK(TOLERANCE.is_close(moved.get_point(1)[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(moved.get_point(1)[1], -1.0));
        MINI_CHECK(TOLERANCE.is_close(moved.get_point(2)[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(stepped.get_point(0)[1], -1.0));
        MINI_CHECK(TOLERANCE.is_close(stepped.get_point(1)[1], -2.0));
        MINI_CHECK(TOLERANCE.is_close(stepped.get_point(2)[1], -2.0));
    }

} // namespace session_cpp
