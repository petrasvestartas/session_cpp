#include "mini_test.h"
#include "obb.h"
#include "aabb.h"
#include "boundingbox.pb.h"
#include "color.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "plane.h"
#include "point.h"
#include "pointcloud.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include "vector.h"
#include "xform.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("OBB", "Constructor") {
    // using session_cpp::OBB;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::AABB;

    OBB bb1 = OBB::from_point(Point(5.0, 5.0, 5.0), 2.0);

    MINI_CHECK(TOLERANCE.is_close(bb1.center[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(bb1.half_size[0], 2.0));

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 3.0, 4.0),
    };
    const OBB bb2 = OBB::from_points(pts);
    const Point mn = bb2.min_point();
    const Point mx = bb2.max_point();

    MINI_CHECK(TOLERANCE.is_close(mn[0], 0.0) && TOLERANCE.is_close(mn[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(mx[0], 2.0) && TOLERANCE.is_close(mx[2], 4.0));

    const OBB box(
        Point(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(1.0, 2.0, 3.0)
    );

    MINI_CHECK(TOLERANCE.is_close(box.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(box.half_size[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(box.half_size[2], 3.0));

    const OBB same(box);

    MINI_CHECK(box == same);
    MINI_CHECK(box != bb1);
    MINI_CHECK(box.guid() != same.guid());
    MINI_CHECK(box.str() == "0.000000, 0.000000, 0.000000\n1.000000, 0.000000, 0.000000\n0.000000, 1.000000, 0.000000\n0.000000, 0.000000, 1.000000\n1.000000, 2.000000, 3.000000");
    MINI_CHECK(box.repr() == "OBB(my_obb, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 1.000000, 2.000000, 3.000000)");

    const AABB bb_aabb = bb2.aabb();

    MINI_CHECK(TOLERANCE.is_close(bb_aabb.min_point()[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(bb_aabb.max_point()[2], 4.0));

    const std::array<Point, 8> corners = bb2.corners();

    MINI_CHECK(corners.size() == 8);

    const Point p_center = bb2.point_at(0.0, 0.0, 0.0);
    const double hx = bb2.half_size[0];
    const double hy = bb2.half_size[1];
    const double hz = bb2.half_size[2];
    const Point p_max_pt = bb2.point_at(hx, hy, hz);

    MINI_CHECK(TOLERANCE.is_close(p_center[0], 1.0) && TOLERANCE.is_close(p_center[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(p_max_pt[0], 2.0) && TOLERANCE.is_close(p_max_pt[2], 4.0));

    OBB bb3 = OBB::from_points({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 2.0, 2.0),
    });
    bb3.inflate(1.0);

    MINI_CHECK(TOLERANCE.is_close(bb3.min_point()[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close(bb3.max_point()[0], 3.0));

    bb1.name = "test_bbox";

    MINI_CHECK(!bb1.guid().empty());
    MINI_CHECK(bb1.name == "test_bbox");
}

MINI_TEST("OBB", "Collision") {
    // using session_cpp::OBB;
    // using session_cpp::Point;

    const OBB bb1 = OBB::from_point(Point(0.0, 0.0, 0.0), 1.0);
    const OBB bb2 = OBB::from_point(Point(1.5, 0.0, 0.0), 1.0);
    const OBB bb3 = OBB::from_point(Point(5.0, 5.0, 5.0), 0.5);

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
    // using session_cpp::OBB;
    // using session_cpp::Point;
    // using session_cpp::Xform;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
    };
    OBB bb = OBB::from_points(pts);
    const Xform bb_xf = Xform::translation(0.0, 0.0, 5.0);
    const OBB bbt = bb.transformed(bb_xf);

    MINI_CHECK(TOLERANCE.is_close(bbt.center[2], 5.0));

    bb.transform(bb_xf);

    MINI_CHECK(TOLERANCE.is_close(bb.center[2], 5.0));
}

MINI_TEST("OBB", "Json Roundtrip") {
    // using session_cpp::OBB;
    // using session_cpp::Point;

    OBB bb = OBB::from_point(Point(1.0, 2.0, 3.0), 5.0);
    bb.name = "test_bbox";

    const nlohmann::ordered_json data = bb.jsondump();
    const OBB loaded_j = OBB::jsonload(data);

    MINI_CHECK(loaded_j.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded_j.center[0], 1.0));

    const std::string text = bb.file_json_dumps();
    const OBB loaded_s = OBB::file_json_loads(text);

    MINI_CHECK(loaded_s.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded_s.half_size[0], 5.0));

    const std::string fname = "serialization/test_obb.json";
    bb.file_json_dump(fname);

    const OBB loaded = OBB::file_json_load(fname);

    MINI_CHECK(loaded.name == "test_bbox");
    MINI_CHECK(TOLERANCE.is_close(loaded.center[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.half_size[0], 5.0));
}

MINI_TEST("OBB", "Protobuf Roundtrip") {
    // using session_cpp::OBB;
    // using session_cpp::Point;

    OBB bb = OBB::from_point(Point(1.0, 2.0, 3.0), 5.0);
    bb.name = "test_bbox_proto";

    const std::string guid = bb.guid();
    const std::string data = bb.pb_dumps();
    const OBB loaded_s = OBB::pb_loads(data);
    const session_proto::BoundingBox proto = bb.to_proto();
    const OBB converted = OBB::from_proto(proto);

    MINI_CHECK(loaded_s.name == "test_bbox_proto");
    MINI_CHECK(loaded_s.guid() == guid);
    MINI_CHECK(TOLERANCE.is_close(loaded_s.center[0], 1.0));
    MINI_CHECK(proto.guid() == guid);
    MINI_CHECK(converted == bb);
    MINI_CHECK(converted.guid() == guid);

    const std::string fname = "serialization/test_obb.bin";
    bb.pb_dump(fname);

    const OBB loaded = OBB::pb_load(fname);

    MINI_CHECK(loaded.name == "test_bbox_proto");
    MINI_CHECK(loaded.guid() == guid);
    MINI_CHECK(TOLERANCE.is_close(loaded.center[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.half_size[0], 5.0));
}

MINI_TEST("OBB", "Accessors") {
    // using session_cpp::OBB;
    // using session_cpp::Point;

    const std::vector<Point> pts = {
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

    const OBB other = OBB::from_point(Point(5.0, 2.0, 3.0), 1.0);
    b.union_with(other);

    MINI_CHECK(TOLERANCE.is_close(b.half_size[0], 3.0));
}

MINI_TEST("OBB", "From Geometry") {
    // using session_cpp::AABB;
    // using session_cpp::Color;
    // using session_cpp::Line;
    // using session_cpp::NurbsCurve;
    // using session_cpp::NurbsSurface;
    // using session_cpp::OBB;
    // using session_cpp::Point;
    // using session_cpp::PointCloud;
    // using session_cpp::Polyline;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;

    const OBB bb_aabb = OBB::from_aabb(AABB(1.0, 2.0, 3.0, 0.5, 1.0, 1.5));

    MINI_CHECK(bb_aabb.center == Point(1.0, 2.0, 3.0));
    MINI_CHECK(TOLERANCE.is_close(bb_aabb.half_size[2], 1.5));
    MINI_CHECK(TOLERANCE.is_close(bb_aabb.x_axis[0], 1.0));

    const OBB bb_line = OBB::from_line(Line(0.0, 0.0, 0.0, 4.0, 0.0, 0.0), 0.1);

    MINI_CHECK(bb_line.is_valid());
    MINI_CHECK(TOLERANCE.is_close(bb_line.center[0], 2.0));

    const OBB bb_pl = OBB::from_polyline(Polyline({
        Point(0.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
        Point(4.0, 4.0, 4.0),
    }), 0.0);

    MINI_CHECK(bb_pl.is_valid());
    MINI_CHECK(bb_pl.volume() > 0.0);

    const OBB bb_mesh = OBB::from_mesh(Primitives::cube(2.0), 0.0);

    MINI_CHECK(bb_mesh.is_valid());
    MINI_CHECK(TOLERANCE.is_close(bb_mesh.center[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(bb_mesh.volume(), 8.0));

    const OBB bb_pc = OBB::from_pointcloud(PointCloud(
        {
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(0.0, 2.0, 0.0),
            Point(0.0, 0.0, 2.0),
        },
        {
            Vector(0.0, 0.0, 1.0),
            Vector(0.0, 0.0, 1.0),
            Vector(0.0, 0.0, 1.0),
            Vector(0.0, 0.0, 1.0),
        },
        {
            Color(255, 0, 0, 255),
            Color(0, 255, 0, 255),
            Color(0, 0, 255, 255),
            Color(255, 255, 0, 255),
        }
    ), 0.0);

    MINI_CHECK(bb_pc.is_valid());
    MINI_CHECK(bb_pc.volume() > 0.0);

    const OBB bb_nc = OBB::from_nurbscurve(NurbsCurve::create(false, 2, {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
    }), 0.5, false);

    MINI_CHECK(bb_nc.is_valid());

    const OBB bb_ns = OBB::from_nurbssurface(NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(2.0, 2.0, 2.0),
    }), 0.0);

    MINI_CHECK(bb_ns.is_valid());
}

MINI_TEST("OBB", "From Plane") {
    // using session_cpp::OBB;
    // using session_cpp::Plane;
    // using session_cpp::Point;

    const Plane plane = Plane::xy_plane();
    const OBB box = OBB::from_plane(plane, 2.0, 3.0, 4.0);

    MINI_CHECK(TOLERANCE.is_close(box.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(box.half_size[1], 1.5));
    MINI_CHECK(TOLERANCE.is_close(box.half_size[2], 2.0));
    MINI_CHECK(box.center == Point(0.0, 0.0, 0.0));

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 3.0, 0.0),
        Point(0.0, 3.0, 0.0),
    };
    const OBB bb = OBB::from_points(pts, plane, 0.0);

    MINI_CHECK(TOLERANCE.is_close(bb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(bb.half_size[1], 1.5));
    MINI_CHECK(TOLERANCE.is_close(bb.x_axis[0], 1.0));
}

MINI_TEST("OBB", "Two Rectangles") {
    // using session_cpp::OBB;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    const OBB bb(
        Point(1.0, 2.0, 3.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(2.0, 3.0, 4.0)
    );
    const std::array<Point, 10> rects = bb.two_rectangles();

    MINI_CHECK(rects.size() == 10);
    MINI_CHECK(rects[0] == Point(3.0, 5.0, -1.0));
    MINI_CHECK(rects[2] == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(rects[4] == rects[0]);
    MINI_CHECK(rects[5] == Point(3.0, 5.0, 7.0));
    MINI_CHECK(rects[7] == Point(-1.0, -1.0, 7.0));
    MINI_CHECK(rects[9] == rects[5]);
}

}
