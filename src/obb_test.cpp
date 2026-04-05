#include "mini_test.h"
#include "obb.h"
#include "plane.h"
#include "xform.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("OBB", "Constructor") {
    // uncomment #include "obb.h"
    // uncomment #include "aabb.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // from_point
    OBB bb1 = OBB::from_point(Point(5.0, 5.0, 5.0), 2.0);

    MINI_CHECK(TOLERANCE.is_close(bb1.center[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(bb1.half_size[0], 2.0));

    // from_points (AABB)
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 3.0, 4.0),
    };
    OBB bb2 = OBB::from_points(pts);
    Point mn = bb2.min_point();
    Point mx = bb2.max_point();

    MINI_CHECK(TOLERANCE.is_close(mn[0], 0.0) && TOLERANCE.is_close(mn[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(mx[0], 2.0) && TOLERANCE.is_close(mx[2], 4.0));

    // OBB constructor
    OBB box(
        Point(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(1.0, 2.0, 3.0)
    );

    MINI_CHECK(TOLERANCE.is_close(box.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(box.half_size[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(box.half_size[2], 3.0));

    // aabb
    AABB bb_aabb = bb2.aabb();

    MINI_CHECK(TOLERANCE.is_close(bb_aabb.min_point()[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(bb_aabb.max_point()[2], 4.0));

    // corners
    std::array<Point, 8> corners = bb2.corners();

    MINI_CHECK(corners.size() == 8);

    // point_at: center + x*x_axis + y*y_axis + z*z_axis (raw OBB offsets)
    Point p_center = bb2.point_at(0.0, 0.0, 0.0);
    double hx = bb2.half_size[0], hy = bb2.half_size[1], hz = bb2.half_size[2];
    Point p_max_pt = bb2.point_at(hx, hy, hz);

    MINI_CHECK(TOLERANCE.is_close(p_center[0], 1.0) && TOLERANCE.is_close(p_center[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(p_max_pt[0], 2.0) && TOLERANCE.is_close(p_max_pt[2], 4.0));

    // inflate
    OBB bb3 = OBB::from_points({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 2.0, 2.0),
    });
    bb3.inflate(1.0);

    MINI_CHECK(TOLERANCE.is_close(bb3.min_point()[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close(bb3.max_point()[0], 3.0));

    // guid() and name
    MINI_CHECK(!bb1.guid().empty());
    bb1.name = "test_bbox";
    MINI_CHECK(bb1.name == "test_bbox");
}

MINI_TEST("OBB", "Collision") {
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    OBB bb1 = OBB::from_point(Point(0.0, 0.0, 0.0), 1.0);
    OBB bb2 = OBB::from_point(Point(1.5, 0.0, 0.0), 1.0);
    OBB bb3 = OBB::from_point(Point(5.0, 5.0, 5.0), 0.5);

    MINI_CHECK(bb1.collides_with(bb2));
    MINI_CHECK(!bb1.collides_with(bb3));
    MINI_CHECK(bb1.collides_with_broad(bb2));
    MINI_CHECK(!bb1.collides_with_broad(bb3));
    MINI_CHECK(bb1.collides_with_rtcd(bb2));
    MINI_CHECK(!bb1.collides_with_rtcd(bb3));
    MINI_CHECK(bb1.collides_with_naive(bb2));
    MINI_CHECK(!bb1.collides_with_naive(bb3));
}

MINI_TEST("OBB", "Transformation") {
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
    };
    OBB bb = OBB::from_points(pts);
    bb.xform = Xform::translation(0.0, 0.0, 5.0);

    OBB bbt = bb.transformed();

    MINI_CHECK(TOLERANCE.is_close(bbt.center[2], 5.0));

    bb.transform();

    MINI_CHECK(bb.xform == Xform::identity());
    MINI_CHECK(TOLERANCE.is_close(bb.center[2], 5.0));
}

MINI_TEST("OBB", "Json Roundtrip") {
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    OBB bb = OBB::from_point(Point(1.0, 2.0, 3.0), 5.0);
    bb.name = "test_bbox";

    // JSON object
    nlohmann::ordered_json j = bb.jsondump();
    OBB loaded_j = OBB::jsonload(j);

    MINI_CHECK(loaded_j.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded_j.center[0], 1.0));

    // String
    std::string s = bb.json_dumps();
    OBB loaded_s = OBB::json_loads(s);

    MINI_CHECK(loaded_s.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded_s.half_size[0], 5.0));

    // File
    std::string fname = "serialization/test_obb.json";
    bb.json_dump(fname);
    OBB loaded = OBB::json_load(fname);

    MINI_CHECK(loaded.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded.center[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.half_size[0], 5.0));
}

MINI_TEST("OBB", "Protobuf Roundtrip") {
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    OBB bb = OBB::from_point(Point(1.0, 2.0, 3.0), 5.0);
    bb.name = "test_bbox_proto";

    // String
    std::string s = bb.pb_dumps();
    OBB loaded_s = OBB::pb_loads(s);

    MINI_CHECK(loaded_s.name == "test_bbox_proto");
    MINI_CHECK(loaded_s.guid() == bb.guid());
    MINI_CHECK(TOLERANCE.is_close(loaded_s.center[0], 1.0));

    // File
    std::string fname = "serialization/test_obb.bin";
    bb.pb_dump(fname);
    OBB loaded = OBB::pb_load(fname);

    MINI_CHECK(loaded.name == "test_bbox_proto");
    MINI_CHECK(loaded.guid() == bb.guid());
    MINI_CHECK(TOLERANCE.is_close(loaded.center[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.half_size[0], 5.0));
}

MINI_TEST("OBB", "Accessors") {
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    // axis-aligned OBB: center=(1,2,3), half_size=(1,2,3), dims 2×4×6
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 4.0, 0.0),
        Point(0.0, 4.0, 0.0),
        Point(0.0, 0.0, 6.0),
        Point(2.0, 4.0, 6.0),
    };
    OBB b = OBB::from_points(pts);

    MINI_CHECK(TOLERANCE.is_close(b.area(), 88.0));
    MINI_CHECK(TOLERANCE.is_close(b.diagonal(), 2.0 * std::sqrt(14.0)));
    MINI_CHECK(b.is_valid());
    MINI_CHECK(TOLERANCE.is_close(b.volume(), 48.0));

    MINI_CHECK(b.closest_point(Point(1.0, 2.0, 3.0)) == Point(1.0, 2.0, 3.0));
    MINI_CHECK(b.closest_point(Point(10.0, 2.0, 3.0)) == Point(2.0, 2.0, 3.0));
    MINI_CHECK(b.contains(Point(1.0, 2.0, 3.0)));
    MINI_CHECK(!b.contains(Point(10.0, 2.0, 3.0)));

    MINI_CHECK(b.corner(false, false, false) == Point(0.0, 0.0, 0.0));
    MINI_CHECK(b.corner(true, true, true) == Point(2.0, 4.0, 6.0));
    MINI_CHECK(b.get_corners().size() == 8);
    MINI_CHECK(b.get_edges().size() == 12);

    OBB c = OBB::from_point(Point(5.0, 2.0, 3.0), 1.0);
    b.union_with(c);

    MINI_CHECK(TOLERANCE.is_close(b.half_size[0], 3.0));
}

} // namespace session_cpp
