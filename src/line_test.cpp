#include "mini_test.h"
#include "line.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "tolerance.h"
#include "xform.h"

#include <cmath>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Line", "constructor") {
    // uncomment #include "line.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"

    // Constructor
    Line l(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    // Setters
    l[0] = 10.0;
    l[1] = 20.0;
    l[2] = 30.0;
    l[3] = 40.0;
    l[4] = 50.0;
    l[5] = 60.0;

    // Getters
    double x0 = l[0];
    double y0 = l[1];
    double z0 = l[2];
    double x1 = l[3];
    double y1 = l[4];
    double z1 = l[5];

    // Minimal and Full String Representation
    std::string lstr = l.str();
    std::string lrepr = l.repr();

    // Copy (duplicate everything but guid)
    Line lcopy = l;
    Line lother(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

    // No-copy operators
    Line lmult = l;
    lmult *= 2.0;
    Line ldiv = l;
    ldiv /= 2.0;
    Line ladd = l;
    ladd += Vector(1.0, 1.0, 1.0);
    Line lsub = l;
    lsub -= Vector(1.0, 1.0, 1.0);

    // Copy operators
    Line rmul = l * 2.0;
    Line rdiv = l / 2.0;
    Line radd = l + Vector(1.0, 1.0, 1.0);
    Line rdif = l - Vector(1.0, 1.0, 1.0);

    // Negation (flip start and end)
    Line lneg(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    Line neg = -lneg;

    // From points constructor
    Point p0(1.0, 2.0, 3.0);
    Point p1(4.0, 5.0, 6.0);
    Line l2p = Line::from_points(p0, p1);

    // from_point_and_vector constructor
    Point pv(1.0, 2.0, 3.0);
    Vector vv(3.0, 4.0, 5.0);
    Line l_pv = Line::from_point_and_vector(pv, vv);

    // from_point_direction_length constructor
    Point pd(0.0, 0.0, 0.0);
    Vector dd(1.0, 0.0, 0.0);
    Line l_pdl = Line::from_point_direction_length(pd, dd, 5.0);

    // Line with custom color and width
    Line lc(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    lc.linecolor = Color(255, 0, 0, 255, "red");
    lc.width = 2.5;

    MINI_CHECK(l.name == "my_line" && l[0] == 10.0 && l[1] == 20.0 && l[2] == 30.0 && !l.guid.empty());
    MINI_CHECK(x0 == 10.0 && y0 == 20.0 && z0 == 30.0 && x1 == 40.0 && y1 == 50.0 && z1 == 60.0);
    MINI_CHECK(lstr.find("10") != std::string::npos && lstr.find("20") != std::string::npos && lstr.find("60") != std::string::npos);
    MINI_CHECK(lrepr.find("my_line") != std::string::npos && lrepr.find("10") != std::string::npos && lrepr.find("Color") != std::string::npos);
    MINI_CHECK(lcopy.guid != l.guid);
    MINI_CHECK(lmult[0] == 20.0 && lmult[3] == 80.0);
    MINI_CHECK(ldiv[0] == 5.0 && ldiv[3] == 20.0);
    MINI_CHECK(ladd[0] == 11.0 && ladd[3] == 41.0);
    MINI_CHECK(lsub[0] == 9.0 && lsub[3] == 39.0);
    MINI_CHECK(rmul[0] == 20.0 && rmul[3] == 80.0);
    MINI_CHECK(rdiv[0] == 5.0 && rdiv[3] == 20.0);
    MINI_CHECK(radd[0] == 11.0 && radd[3] == 41.0);
    MINI_CHECK(rdif[0] == 9.0 && rdif[3] == 39.0);
    MINI_CHECK(neg[0] == 4.0 && neg[1] == 5.0 && neg[2] == 6.0 && neg[3] == 1.0 && neg[4] == 2.0 && neg[5] == 3.0);
    MINI_CHECK(l2p[0] == 1.0 && l2p[3] == 4.0);
    MINI_CHECK(l_pv[0] == 1.0 && l_pv[1] == 2.0 && l_pv[2] == 3.0 && l_pv[3] == 4.0 && l_pv[4] == 6.0 && l_pv[5] == 8.0);
    MINI_CHECK(l_pdl[0] == 0.0 && l_pdl[3] == 5.0);
    MINI_CHECK(lc.linecolor[0] == 255 && lc.linecolor[1] == 0 && lc.width == 2.5);
}

MINI_TEST("Line", "length") {
    // uncomment #include "line.h"

    Line l(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
    double ln = l.length();
    double lsq = l.squared_length();

    MINI_CHECK(TOLERANCE.is_close(ln, 5.0));
    MINI_CHECK(TOLERANCE.is_close(lsq, 25.0));
}

MINI_TEST("Line", "to_vector") {
    // uncomment #include "line.h"

    Line l(1.0, 2.0, 3.0, 4.0, 6.0, 9.0);
    Vector v = l.to_vector();

    MINI_CHECK(v[0] == 3.0 && v[1] == 4.0 && v[2] == 6.0);
}

MINI_TEST("Line", "to_direction") {
    // uncomment #include "line.h"

    Line l(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
    Vector d = l.to_direction();

    MINI_CHECK(TOLERANCE.is_close(d[0], 0.6));
    MINI_CHECK(TOLERANCE.is_close(d[1], 0.8));
    MINI_CHECK(TOLERANCE.is_close(d[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(d.magnitude(), 1.0));
}

MINI_TEST("Line", "point_at") {
    // uncomment #include "line.h"

    Line l(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    Point ps = l.point_at(0.0);
    Point pm = l.point_at(0.5);
    Point pe = l.point_at(1.0);

    MINI_CHECK(ps[0] == 0.0 && ps[1] == 0.0 && ps[2] == 0.0);
    MINI_CHECK(pm[0] == 5.0 && pm[1] == 5.0 && pm[2] == 5.0);
    MINI_CHECK(pe[0] == 10.0 && pe[1] == 10.0 && pe[2] == 10.0);
}

MINI_TEST("Line", "closest_point") {
    // uncomment #include "line.h"
    // uncomment #include "point.h"

    Line l(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    Point p1(5.0, 5.0, 0.0);
    Point p2(-5.0, 0.0, 0.0);
    Point p3(15.0, 0.0, 0.0);
    Point cp1 = l.closest_point(p1);
    Point cp2 = l.closest_point(p2);
    Point cp3 = l.closest_point(p3);

    MINI_CHECK(cp1[0] == 5.0 && cp1[1] == 0.0 && cp1[2] == 0.0);
    MINI_CHECK(cp2[0] == 0.0 && cp2[1] == 0.0 && cp2[2] == 0.0);
    MINI_CHECK(cp3[0] == 10.0 && cp3[1] == 0.0 && cp3[2] == 0.0);
}

MINI_TEST("Line", "start_end_center") {
    // uncomment #include "line.h"

    Line l(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    Point start = l.start();
    Point end = l.end();
    Point center = l.center();

    MINI_CHECK(start[0] == 1.0 && start[1] == 2.0 && start[2] == 3.0);
    MINI_CHECK(end[0] == 4.0 && end[1] == 5.0 && end[2] == 6.0);
    MINI_CHECK(center[0] == 2.5 && center[1] == 3.5 && center[2] == 4.5);
}

MINI_TEST("Line", "fit_points") {
    // uncomment #include "line.h"
    // uncomment #include "point.h"
    // uncomment #include <vector>

    std::vector<Point> fit_pts = {Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 0.5), Point(2.0, 2.0, 1.0), Point(3.0, 3.0, 1.5)};
    Line l_fit = Line::fit_points(fit_pts);

    MINI_CHECK(l_fit.length() > 0.0);
}

MINI_TEST("Line", "subdivide") {
    // uncomment #include "line.h"
    // uncomment #include <vector>

    Line l(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);

    // subdivide by count
    std::vector<Point> pts = l.subdivide(3);

    // subdivide_by_distance
    std::vector<Point> pts_dist = l.subdivide_by_distance(2.5);

    MINI_CHECK(pts.size() == 3);
    MINI_CHECK(pts[0][0] == 0.0);
    MINI_CHECK(pts[1][0] == 5.0);
    MINI_CHECK(pts[2][0] == 10.0);

    MINI_CHECK(pts_dist.size() == 5);
    MINI_CHECK(TOLERANCE.is_close(pts_dist[0][0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pts_dist[1][0], 2.5));
    MINI_CHECK(TOLERANCE.is_close(pts_dist[4][0], 10.0));
}

MINI_TEST("Line", "transformation") {
    // uncomment #include "line.h"
    // uncomment #include "xform.h"

    Line l(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    l.xform = Xform::translation(10.0, 0.0, 0.0);
    Line l_transformed = l.transformed(); // Make a copy
    l.transform(); // After the call, "xform" is reset

    MINI_CHECK(l_transformed[0] == 10.0 && l_transformed[3] == 11.0);
    MINI_CHECK(l[0] == 10.0 && l[3] == 11.0);
    MINI_CHECK(l.xform == Xform::identity());
}

MINI_TEST("Line", "json_roundtrip") {
    // uncomment #include "line.h"

    Line l(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
    l.name = "test_line";

    // json_dump(fname) / json_load(fname) - file-based serialization
    std::string fname = "test_line.json";
    l.json_dump(fname);
    Line loaded = Line::json_load(fname);

    MINI_CHECK(loaded.name == "test_line");
    MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
    MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
    MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
    MINI_CHECK(TOLERANCE.is_close(loaded[3], 168.4));
    MINI_CHECK(TOLERANCE.is_close(loaded[4], 210.5));
    MINI_CHECK(TOLERANCE.is_close(loaded[5], 252.6));
}

#ifdef ENABLE_PROTOBUF
MINI_TEST("Line", "protobuf_roundtrip") {
    // uncomment #include "line.h"

    Line l(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
    l.name = "test_line";

    // protobuf_dump(fname) / protobuf_load(fname) - file-based serialization
    std::string fname = "test_line.bin";
    l.protobuf_dump(fname);
    Line loaded = Line::protobuf_load(fname);

    MINI_CHECK(loaded.name == "test_line");
    MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
    MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
    MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
    MINI_CHECK(TOLERANCE.is_close(loaded[3], 168.4));
    MINI_CHECK(TOLERANCE.is_close(loaded[4], 210.5));
    MINI_CHECK(TOLERANCE.is_close(loaded[5], 252.6));
}
#endif

} // namespace session_cpp
