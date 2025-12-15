#include "mini_test.h"
#include "pointcloud.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "tolerance.h"

#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("PointCloud", "constructor") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"
    // uncomment #include <vector>

    // Constructor with points, normals, colors
    Point p0(0.0, 0.0, 0.0);
    Point p1(1.0, 0.0, 0.0);
    Point p2(0.0, 1.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Vector n1(0.0, 0.0, 1.0);
    Vector n2(0.0, 0.0, 1.0);
    Color c0(255, 0, 0, 255);
    Color c1(0, 255, 0, 255);
    Color c2(0, 0, 255, 255);
    PointCloud pc({p0, p1, p2}, {n0, n1, n2}, {c0, c1, c2});

    // Basic properties
    size_t point_count = pc.len();
    size_t color_count = pc.color_count();
    size_t normal_count = pc.normal_count();
    bool is_empty = pc.is_empty();

    // Minimal and Full String Representation
    std::string pcstr = pc.str();
    std::string pcrepr = pc.repr();

    // Copy (duplicates everything except guid)
    PointCloud pccopy = pc.duplicate();

    // Get point/color/normal at index
    Point pt0 = pc.get_point(0);
    Color col0 = pc.get_color(0);
    Vector norm0 = pc.get_normal(0);

    MINI_CHECK(pc.name == "my_pointcloud" && !pc.guid.empty() && point_count == 3);
    MINI_CHECK(color_count == 3 && normal_count == 3 && is_empty == false);
    MINI_CHECK(pcstr.find("3 points") != std::string::npos);
    MINI_CHECK(pcrepr.find("PointCloud(my_pointcloud") != std::string::npos);
    MINI_CHECK(pccopy == pc && pccopy.guid != pc.guid);
    MINI_CHECK(TOLERANCE.is_close(pt0[0], 0.0) && TOLERANCE.is_close(pt0[1], 0.0) && TOLERANCE.is_close(pt0[2], 0.0));
    MINI_CHECK(col0.r == 255 && col0.g == 0 && col0.b == 0 && col0.a == 255);
    MINI_CHECK(TOLERANCE.is_close(norm0[0], 0.0) && TOLERANCE.is_close(norm0[1], 0.0) && TOLERANCE.is_close(norm0[2], 1.0));
}

MINI_TEST("PointCloud", "from_coords") {
    // uncomment #include "pointcloud.h"
    // uncomment #include <vector>

    // Create from flat arrays
    std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    std::vector<int> colors = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255};
    std::vector<double> normals = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
    PointCloud pc = PointCloud::from_coords(coords, colors, normals);

    size_t point_count = pc.len();
    size_t color_count = pc.color_count();
    size_t normal_count = pc.normal_count();

    Point pt1 = pc.get_point(1);
    Color col1 = pc.get_color(1);
    Vector norm1 = pc.get_normal(1);

    MINI_CHECK(point_count == 3 && color_count == 3 && normal_count == 3);
    MINI_CHECK(TOLERANCE.is_close(pt1[0], 1.0) && TOLERANCE.is_close(pt1[1], 0.0) && TOLERANCE.is_close(pt1[2], 0.0));
    MINI_CHECK(col1.r == 0 && col1.g == 255 && col1.b == 0 && col1.a == 255);
    MINI_CHECK(TOLERANCE.is_close(norm1[0], 0.0) && TOLERANCE.is_close(norm1[1], 0.0) && TOLERANCE.is_close(norm1[2], 1.0));
}

MINI_TEST("PointCloud", "add_and_set") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"

    // Empty cloud
    PointCloud pc;

    // Add points, colors, normals
    pc.add_point(Point(1.0, 2.0, 3.0));
    pc.add_color(Color(128, 64, 32, 255));
    pc.add_normal(Vector(1.0, 0.0, 0.0));

    size_t point_count = pc.len();
    size_t color_count = pc.color_count();
    size_t normal_count = pc.normal_count();

    // Set point/color/normal at index
    pc.set_point(0, Point(4.0, 5.0, 6.0));
    pc.set_color(0, Color(200, 100, 50, 255));
    pc.set_normal(0, Vector(0.0, 1.0, 0.0));

    Point pt0 = pc.get_point(0);
    Color col0 = pc.get_color(0);
    Vector norm0 = pc.get_normal(0);

    MINI_CHECK(point_count == 1 && color_count == 1 && normal_count == 1);
    MINI_CHECK(TOLERANCE.is_close(pt0[0], 4.0) && TOLERANCE.is_close(pt0[1], 5.0) && TOLERANCE.is_close(pt0[2], 6.0));
    MINI_CHECK(col0.r == 200 && col0.g == 100 && col0.b == 50 && col0.a == 255);
    MINI_CHECK(TOLERANCE.is_close(norm0[0], 0.0) && TOLERANCE.is_close(norm0[1], 1.0) && TOLERANCE.is_close(norm0[2], 0.0));
}

MINI_TEST("PointCloud", "translate") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"

    // Create cloud with one point
    PointCloud pc({Point(1.0, 2.0, 3.0)}, {}, {});
    Vector offset(10.0, 20.0, 30.0);

    // In-place add
    PointCloud pc_iadd({Point(1.0, 2.0, 3.0)}, {}, {});
    pc_iadd += offset;

    // In-place subtract
    PointCloud pc_isub({Point(1.0, 2.0, 3.0)}, {}, {});
    pc_isub -= offset;

    // Copy add/subtract
    PointCloud pc_add = pc + offset;
    PointCloud pc_sub = pc - offset;

    Point pt_iadd = pc_iadd.get_point(0);
    Point pt_isub = pc_isub.get_point(0);
    Point pt_add = pc_add.get_point(0);
    Point pt_sub = pc_sub.get_point(0);
    Point pt_orig = pc.get_point(0);

    MINI_CHECK(TOLERANCE.is_close(pt_iadd[0], 11.0) && TOLERANCE.is_close(pt_iadd[1], 22.0) && TOLERANCE.is_close(pt_iadd[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pt_isub[0], -9.0) && TOLERANCE.is_close(pt_isub[1], -18.0) && TOLERANCE.is_close(pt_isub[2], -27.0));
    MINI_CHECK(TOLERANCE.is_close(pt_add[0], 11.0) && TOLERANCE.is_close(pt_add[1], 22.0) && TOLERANCE.is_close(pt_add[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pt_sub[0], -9.0) && TOLERANCE.is_close(pt_sub[1], -18.0) && TOLERANCE.is_close(pt_sub[2], -27.0));
    MINI_CHECK(TOLERANCE.is_close(pt_orig[0], 1.0) && TOLERANCE.is_close(pt_orig[1], 2.0) && TOLERANCE.is_close(pt_orig[2], 3.0));
}

MINI_TEST("PointCloud", "transform") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    // Transform - in-place transformation
    PointCloud pc({Point(1.0, 2.0, 3.0)}, {}, {});
    pc.xform = Xform::translation(10.0, 20.0, 30.0);
    pc.transform();

    // Transformed - returns new cloud
    PointCloud pc2({Point(1.0, 2.0, 3.0)}, {}, {});
    pc2.xform = Xform::translation(10.0, 20.0, 30.0);
    PointCloud pc3 = pc2.transformed();

    Point pt = pc.get_point(0);
    Point pt3 = pc3.get_point(0);
    Point pt2 = pc2.get_point(0);

    MINI_CHECK(TOLERANCE.is_close(pt[0], 11.0) && TOLERANCE.is_close(pt[1], 22.0) && TOLERANCE.is_close(pt[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pt3[0], 11.0) && TOLERANCE.is_close(pt3[1], 22.0) && TOLERANCE.is_close(pt3[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pt2[0], 1.0) && TOLERANCE.is_close(pt2[1], 2.0) && TOLERANCE.is_close(pt2[2], 3.0));
}

MINI_TEST("PointCloud", "json_roundtrip") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"

    PointCloud pc(
        {Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)},
        {Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0)},
        {Color(255, 0, 0, 255), Color(0, 255, 0, 255)}
    );
    pc.name = "test_pointcloud";

    std::string fname = "test_pointcloud.json";
    pc.json_dump(fname);
    PointCloud loaded = PointCloud::json_load(fname);

    Point pt0 = loaded.get_point(0);
    Color col0 = loaded.get_color(0);
    Vector norm0 = loaded.get_normal(0);

    MINI_CHECK(loaded.name == "test_pointcloud");
    MINI_CHECK(loaded.len() == 2);
    MINI_CHECK(TOLERANCE.is_close(pt0[0], 1.0) && TOLERANCE.is_close(pt0[1], 2.0) && TOLERANCE.is_close(pt0[2], 3.0));
    MINI_CHECK(col0.r == 255 && col0.g == 0 && col0.b == 0);
    MINI_CHECK(TOLERANCE.is_close(norm0[2], 1.0));
}

#ifdef ENABLE_PROTOBUF
MINI_TEST("PointCloud", "protobuf_roundtrip") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"

    PointCloud pc(
        {Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)},
        {Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0)},
        {Color(255, 0, 0, 255), Color(0, 255, 0, 255)}
    );
    pc.name = "test_pointcloud";

    std::string fname = "test_pointcloud.bin";
    pc.protobuf_dump(fname);
    PointCloud loaded = PointCloud::protobuf_load(fname);

    Point pt0 = loaded.get_point(0);
    Color col0 = loaded.get_color(0);
    Vector norm0 = loaded.get_normal(0);

    MINI_CHECK(loaded.name == "test_pointcloud");
    MINI_CHECK(loaded.len() == 2);
    MINI_CHECK(TOLERANCE.is_close(pt0[0], 1.0) && TOLERANCE.is_close(pt0[1], 2.0) && TOLERANCE.is_close(pt0[2], 3.0));
    MINI_CHECK(col0.r == 255 && col0.g == 0 && col0.b == 0);
    MINI_CHECK(TOLERANCE.is_close(norm0[2], 1.0));
}
#endif

} // namespace session_cpp
