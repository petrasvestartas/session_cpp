#include "mini_test.h"
#include "boundingbox.h"
#include "plane.h"
#include "xform.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("BoundingBox", "Constructor") {
    // from_point
    BoundingBox bb1 = BoundingBox::from_point(Point(5.0, 5.0, 5.0), 2.0);
    MINI_CHECK(TOLERANCE.is_close(bb1.center[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(bb1.half_size[0], 2.0));

    // from_points (AABB)
    std::vector<Point> pts = {Point(0.0, 0.0, 0.0), Point(2.0, 3.0, 4.0)};
    BoundingBox bb2 = BoundingBox::from_points(pts);
    Point mn = bb2.min_point();
    Point mx = bb2.max_point();
    MINI_CHECK(TOLERANCE.is_close(mn[0], 0.0) && TOLERANCE.is_close(mn[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(mx[0], 2.0) && TOLERANCE.is_close(mx[2], 4.0));

    // OBB constructor
    BoundingBox obb(
        Point(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(1.0, 2.0, 3.0)
    );
    MINI_CHECK(TOLERANCE.is_close(obb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(obb.half_size[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(obb.half_size[2], 3.0));

    // aabb
    BoundingBox bb_aabb = bb2.aabb();
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
    BoundingBox bb3 = BoundingBox::from_points({Point(0.0, 0.0, 0.0), Point(2.0, 2.0, 2.0)});
    bb3.inflate(1.0);
    MINI_CHECK(TOLERANCE.is_close(bb3.min_point()[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close(bb3.max_point()[0], 3.0));

    // guid and name
    MINI_CHECK(!bb1.guid.empty());
    bb1.name = "test_bbox";
    MINI_CHECK(bb1.name == "test_bbox");
}

MINI_TEST("BoundingBox", "Collision") {
    BoundingBox bb1 = BoundingBox::from_point(Point(0.0, 0.0, 0.0), 1.0);
    BoundingBox bb2 = BoundingBox::from_point(Point(1.5, 0.0, 0.0), 1.0);
    BoundingBox bb3 = BoundingBox::from_point(Point(5.0, 5.0, 5.0), 0.5);

    MINI_CHECK(bb1.collides_with(bb2));
    MINI_CHECK(!bb1.collides_with(bb3));
    MINI_CHECK(bb1.collides_with_rtcd(bb2));
    MINI_CHECK(!bb1.collides_with_rtcd(bb3));
    MINI_CHECK(bb1.collides_with_naive(bb2));
    MINI_CHECK(!bb1.collides_with_naive(bb3));
}

MINI_TEST("BoundingBox", "Transformation") {
    std::vector<Point> pts = {Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)};
    BoundingBox bb = BoundingBox::from_points(pts);
    bb.xform = Xform::translation(0.0, 0.0, 5.0);

    BoundingBox bbt = bb.transformed();
    MINI_CHECK(TOLERANCE.is_close(bbt.center[2], 5.0));

    bb.transform();
    MINI_CHECK(bb.xform == Xform::identity());
    MINI_CHECK(TOLERANCE.is_close(bb.center[2], 5.0));
}

MINI_TEST("BoundingBox", "Json Roundtrip") {
    BoundingBox bb = BoundingBox::from_point(Point(1.0, 2.0, 3.0), 5.0);
    bb.name = "test_bbox";

    // JSON object
    nlohmann::ordered_json j = bb.jsondump();
    BoundingBox loaded_j = BoundingBox::jsonload(j);
    MINI_CHECK(loaded_j.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded_j.center[0], 1.0));

    // String
    std::string s = bb.json_dumps();
    BoundingBox loaded_s = BoundingBox::json_loads(s);
    MINI_CHECK(loaded_s.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded_s.half_size[0], 5.0));

    // File
    std::string fname = "serialization/test_boundingbox.json";
    bb.json_dump(fname);
    BoundingBox loaded = BoundingBox::json_load(fname);
    MINI_CHECK(loaded.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded.center[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.half_size[0], 5.0));
}

MINI_TEST("BoundingBox", "Protobuf Roundtrip") {
    BoundingBox bb = BoundingBox::from_point(Point(1.0, 2.0, 3.0), 5.0);
    bb.name = "test_bbox_proto";

    // String
    std::string s = bb.pb_dumps();
    BoundingBox loaded_s = BoundingBox::pb_loads(s);
    MINI_CHECK(loaded_s.name == "test_bbox_proto");
    MINI_CHECK(loaded_s.guid == bb.guid);
    MINI_CHECK(TOLERANCE.is_close(loaded_s.center[0], 1.0));

    // File
    std::string fname = "serialization/test_boundingbox.bin";
    bb.pb_dump(fname);
    BoundingBox loaded = BoundingBox::pb_load(fname);
    MINI_CHECK(loaded.name == "test_bbox_proto");
    MINI_CHECK(loaded.guid == bb.guid);
    MINI_CHECK(TOLERANCE.is_close(loaded.center[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.half_size[0], 5.0));
}

} // namespace session_cpp
