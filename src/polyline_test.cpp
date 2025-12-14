#include "mini_test.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "tolerance.h"

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

    // Translation operators
    Polyline pl2({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)});
    Vector v(1.0, 1.0, 1.0);
    Polyline pl_add = pl2 + v;
    Polyline pl_sub = pl2 - v;

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
    MINI_CHECK(pl_add.get_point(0)[0] == 1.0 && pl_add.get_point(0)[1] == 1.0);
    MINI_CHECK(pl_sub.get_point(0)[0] == -1.0 && pl_sub.get_point(0)[1] == -1.0);
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

    // json_dump(fname) / json_load(fname) - file-based serialization
    std::string fname = "test_polyline.json";
    pl.json_dump(fname);
    Polyline loaded = Polyline::json_load(fname);

    MINI_CHECK(loaded.name == "test_polyline");
    MINI_CHECK(loaded.len() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(1)[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(2)[2], 9.0));



}

#ifdef ENABLE_PROTOBUF
MINI_TEST("Polyline", "protobuf_roundtrip") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"

    Polyline pl({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), Point(7.0, 8.0, 9.0), Point(10.0, 11.0, 12.0)});
    pl.name = "test_polyline";

    // protobuf_dump(fname) / protobuf_load(fname) - file-based serialization
    std::string fname = "test_polyline.bin";
    pl.protobuf_dump(fname);
    Polyline loaded = Polyline::protobuf_load(fname);

    MINI_CHECK(loaded.name == "test_polyline");
    MINI_CHECK(loaded.len() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(1)[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(2)[2], 9.0));
}
#endif

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
    Vector cv = pl.center_vec();

    MINI_CHECK(TOLERANCE.is_close(c[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(c[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(c[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(cv[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(cv[1], 1.0));


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

} // namespace session_cpp
