#include "mini_test.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "tolerance.h"
#include "plane.h"

#include <cmath>
#include <vector>
#include <sstream>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Polyline", "constructor") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"

    // Constructor with points
    Point p0(0.0, 0.0, 0.0);
    Point p1(1.0, 0.0, 0.0);
    Point p2(1.0, 1.0, 0.0);
    Point p3(0.0, 1.0, 0.0);
    Polyline pl({p0, p1, p2, p3});

    // Basic properties
    size_t point_count = pl.len();
    size_t segment_count = pl.segment_count();
    bool is_empty = pl.is_empty();

    // Get point
    Point pt = pl.get_point(1);

    // Minimal and Full String Representation
    std::string plstr = pl.str();
    std::string plrepr = pl.repr();

    // Copy (duplicates everything except guid)
    Polyline plcopy = pl;
    Polyline plother({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});

    // No-copy operators
    Polyline plmult = pl;
    plmult *= 2.0;
    Polyline pldiv = pl;
    pldiv /= 2.0;
    Polyline pladd = pl;
    pladd += Vector(1.0, 1.0, 1.0);
    Polyline plsub = pl;
    plsub -= Vector(1.0, 1.0, 1.0);

    // Copy operators
    Polyline rmul = pl * 2.0;
    Polyline rdiv = pl / 2.0;
    Polyline radd = pl + Vector(1.0, 1.0, 1.0);
    Polyline rdif = pl - Vector(1.0, 1.0, 1.0);

    // Negation (reverse point order)
    Polyline plneg({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(3.0, 0.0, 0.0)});
    Polyline neg = -plneg;

    // Polyline with custom color and width
    Polyline plc({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    plc.linecolor = Color(255, 0, 0, 255, "red");
    plc.width = 2.5;

    MINI_CHECK(pl.name == "my_polyline" && !pl.guid.empty() && point_count == 4);
    MINI_CHECK(segment_count == 3 && is_empty == false);
    MINI_CHECK(pt[0] == 1.0 && pt[1] == 0.0 && pt[2] == 0.0);
    MINI_CHECK(plstr.find("(0, 0, 0)") != std::string::npos);
    MINI_CHECK(plrepr.find("Polyline(my_polyline") != std::string::npos && plrepr.find("4 points") != std::string::npos);
    MINI_CHECK(plcopy == plother);
    MINI_CHECK(plcopy.guid != pl.guid);
    MINI_CHECK(plmult.get_point(1)[0] == 2.0);
    MINI_CHECK(pldiv.get_point(1)[0] == 0.5);
    MINI_CHECK(pladd.get_point(0)[0] == 1.0 && pladd.get_point(0)[1] == 1.0);
    MINI_CHECK(plsub.get_point(0)[0] == -1.0 && plsub.get_point(0)[1] == -1.0);
    MINI_CHECK(rmul.get_point(1)[0] == 2.0);
    MINI_CHECK(rdiv.get_point(1)[0] == 0.5);
    MINI_CHECK(radd.get_point(0)[0] == 1.0 && radd.get_point(0)[1] == 1.0);
    MINI_CHECK(rdif.get_point(0)[0] == -1.0 && rdif.get_point(0)[1] == -1.0);
    MINI_CHECK(neg.get_point(0)[0] == 3.0 && neg.get_point(3)[0] == 0.0);
    MINI_CHECK(plc.linecolor[0] == 255 && plc.linecolor[1] == 0 && plc.width == 2.5);


}

MINI_TEST("Polyline", "transformation") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"

    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    pl.xform = Xform::translation(10.0, 0.0, 0.0);
    Polyline pl_transformed = pl.transformed();
    pl.transform();

    MINI_CHECK(pl_transformed.get_point(0)[0] == 10.0 && pl_transformed.get_point(1)[0] == 11.0);
    MINI_CHECK(pl.get_point(0)[0] == 10.0 && pl.get_point(1)[0] == 11.0);
    MINI_CHECK(pl.xform == Xform::identity());
}

MINI_TEST("Polyline", "json_roundtrip") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    Polyline pl({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), Point(7.0, 8.0, 9.0), Point(10.0, 11.0, 12.0)});
    pl.name = "test_polyline";

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   json_dumps()    │ std::string  │ to JSON string
    //   json_loads(s)   │ std::string  │ from JSON string
    //   json_dump(path) │ file         │ write to file
    //   json_load(path) │ file         │ read from file

    // json_dump(fname) / json_load(fname) - file-based serialization
    std::string fname = "serialization/test_polyline.json";
    pl.json_dump(fname);
    Polyline loaded = Polyline::json_load(fname);

    MINI_CHECK(loaded.name == "test_polyline");
    MINI_CHECK(loaded.len() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(1)[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(2)[2], 9.0));



}

MINI_TEST("Polyline", "protobuf_roundtrip") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    Polyline pl({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), Point(7.0, 8.0, 9.0), Point(10.0, 11.0, 12.0)});
    pl.name = "test_polyline";

    // pb_dump(fname) / pb_load(fname) - file-based serialization
    std::string fname = "serialization/test_polyline.bin";
    pl.pb_dump(fname);
    Polyline loaded = Polyline::pb_load(fname);

    MINI_CHECK(loaded.name == "test_polyline");
    MINI_CHECK(loaded.len() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(1)[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(2)[2], 9.0));
}

MINI_TEST("Polyline", "length") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    // L-shaped polyline: 1 unit right, 1 unit up, 1 unit left = 3 units total
    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    double ln = pl.length();
    double mag_sq = pl.length_squared();

    MINI_CHECK(TOLERANCE.is_close(ln, 3.0));
    MINI_CHECK(TOLERANCE.is_close(mag_sq, 3.0));


}

MINI_TEST("Polyline", "center") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    // Square polyline
    Polyline pl({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 2.0, 0.0),
        Point(0.0, 2.0, 0.0)
    });
    Point c = pl.center();

    MINI_CHECK(TOLERANCE.is_close(c[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(c[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(c[2], 0.0));


}

MINI_TEST("Polyline", "is_closed") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    // Open polyline
    Polyline open_pl({
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(0.0, 1.0, 0.0)
    });
    bool is_open = open_pl.is_closed();

    // Closed polyline (first and last point same)
    Polyline closed_pl({
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(0.0, 0.0, 0.0)
    });
    bool is_closed = closed_pl.is_closed();

    MINI_CHECK(is_open == false);
    MINI_CHECK(is_closed == true);


}

MINI_TEST("Polyline", "reverse") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(3.0, 0.0, 0.0)});

    // Test reversed() returns new polyline
    Polyline rev = pl.reversed();
    double orig_first = pl.get_point(0)[0];
    double rev_first = rev.get_point(0)[0];

    // Test reverse() in place
    pl.reverse();
    double in_place_first = pl.get_point(0)[0];

    MINI_CHECK(orig_first == 0.0);
    MINI_CHECK(rev_first == 3.0);
    MINI_CHECK(in_place_first == 3.0);


}

MINI_TEST("Polyline", "closest_point") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    Polyline pl({Point(0.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(2.0, 2.0, 0.0), Point(0.0, 2.0, 0.0)});
    Point test_pt(1.0, 1.0, 0.0);
    size_t edge_id = 0;
    Point closest;
    double distance = pl.closest_distance_and_point(test_pt, edge_id, closest);

    MINI_CHECK(edge_id == 0);
    MINI_CHECK(TOLERANCE.is_close(closest[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(closest[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(distance, 1.0));


}

MINI_TEST("Polyline", "extend_segment") {
    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(3.0, 0.0, 0.0)});
    pl.extend_segment(0, 0.5, 0.5);
    double first = pl.get_point(0)[0];
    double second = pl.get_point(1)[0];

    MINI_CHECK(TOLERANCE.is_close(first, -0.5));
    MINI_CHECK(TOLERANCE.is_close(second, 1.5));
}

MINI_TEST("Polyline", "extend_segment_equally") {
    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(3.0, 0.0, 0.0)});
    pl.extend_segment_equally(0, 0.5);
    double first = pl.get_point(0)[0];
    double second = pl.get_point(1)[0];

    MINI_CHECK(TOLERANCE.is_close(first, -0.5));
    MINI_CHECK(TOLERANCE.is_close(second, 1.5));
}

MINI_TEST("Polyline", "get_points") {
    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    std::vector<Point> points = pl.get_points();

    MINI_CHECK(points.size() == 4);
    MINI_CHECK(TOLERANCE.is_close(points[0][0], 0.0) && TOLERANCE.is_close(points[0][1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(points[1][0], 1.0) && TOLERANCE.is_close(points[1][1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(points[2][0], 1.0) && TOLERANCE.is_close(points[2][1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(points[3][0], 0.0) && TOLERANCE.is_close(points[3][1], 1.0));
}

MINI_TEST("Polyline", "shift") {
    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(3.0, 0.0, 0.0)});
    pl.shift(1);
    double first_after_shift = pl.get_point(0)[0];
    pl.shift(-1);
    double first_after_unshift = pl.get_point(0)[0];

    MINI_CHECK(TOLERANCE.is_close(first_after_shift, 1.0));
    MINI_CHECK(TOLERANCE.is_close(first_after_unshift, 0.0));
}

MINI_TEST("Polyline", "point_at") {
    Point start(0.0, 0.0, 0.0);
    Point end(2.0, 0.0, 0.0);
    Point mid = Polyline::point_at(start, end, 0.5);
    Point quarter = Polyline::point_at(start, end, 0.25);

    MINI_CHECK(TOLERANCE.is_close(mid[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(quarter[0], 0.5));
}

MINI_TEST("Polyline", "is_clockwise") {
    // Clockwise square (when viewed from +Z)
    Polyline cw_pl({Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0), Point(1.0, 1.0, 0.0), Point(1.0, 0.0, 0.0)});
    // Counter-clockwise square
    Polyline ccw_pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    Plane plane;

    MINI_CHECK(cw_pl.is_clockwise(plane) == true);
    MINI_CHECK(ccw_pl.is_clockwise(plane) == false);
}

MINI_TEST("Polyline", "convex_corners") {
    Polyline pl({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    std::vector<bool> corners;
    pl.get_convex_corners(corners);

    MINI_CHECK(corners.size() == 4);
}

MINI_TEST("Polyline", "tween") {
    Polyline pl0({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    Polyline pl1({Point(2.0, 0.0, 0.0), Point(3.0, 0.0, 0.0), Point(3.0, 1.0, 0.0), Point(2.0, 1.0, 0.0)});
    Polyline tweened = Polyline::tween_two_polylines(pl0, pl1, 0.5);

    MINI_CHECK(TOLERANCE.is_close(tweened.get_point(0)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(tweened.get_point(1)[0], 2.0));
}

MINI_TEST("Polyline", "average_plane") {
    Polyline pl({Point(0.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(2.0, 2.0, 0.0), Point(0.0, 2.0, 0.0)});
    Point origin;
    Vector x_axis, y_axis, z_axis;
    pl.get_average_plane(origin, x_axis, y_axis, z_axis);
    Point fast_origin;
    Plane fast_plane;
    pl.get_fast_plane(fast_origin, fast_plane);

    MINI_CHECK(TOLERANCE.is_close(origin[0], 1.0) && TOLERANCE.is_close(origin[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(std::abs(z_axis[2]), 1.0));
    MINI_CHECK(fast_origin[0] >= 0.0);
}

} // namespace session_cpp
