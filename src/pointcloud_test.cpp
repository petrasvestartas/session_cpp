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
    PointCloud pccopy = pc;

    // Get point/color/normal at index
    Point pt0 = pc.get_point(0);
    Color col0 = pc.get_color(0);
    Vector norm0 = pc.get_normal(0);

    // Add points, colors, normals to empty cloud
    PointCloud pc2;
    pc2.add_point(Point(1.0, 2.0, 3.0));
    pc2.add_color(Color(128, 64, 32, 255));
    pc2.add_normal(Vector(1.0, 0.0, 0.0));

    // Set point/color/normal at index
    pc2.set_point(0, Point(4.0, 5.0, 6.0));
    pc2.set_color(0, Color(200, 100, 50, 255));
    pc2.set_normal(0, Vector(0.0, 1.0, 0.0));

    // Translate with Vector offset
    PointCloud pc3({Point(1.0, 2.0, 3.0)}, {}, {});
    Vector offset(10.0, 20.0, 30.0);
    PointCloud pc_iadd({Point(1.0, 2.0, 3.0)}, {}, {});
    pc_iadd += offset;
    PointCloud pc_isub({Point(1.0, 2.0, 3.0)}, {}, {});
    pc_isub -= offset;
    PointCloud pc_add = pc3 + offset;
    PointCloud pc_sub = pc3 - offset;

    // Create from flat arrays
    std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    std::vector<int> colors_arr = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255};
    std::vector<double> normals_arr = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
    PointCloud pc4 = PointCloud::from_coords(coords, colors_arr, normals_arr);

    MINI_CHECK(pc.name == "my_pointcloud" && !pc.guid.empty() && point_count == 3);
    MINI_CHECK(color_count == 3 && normal_count == 3 && is_empty == false);
    MINI_CHECK(pcstr.find("3 points") != std::string::npos);
    MINI_CHECK(pcrepr.find("PointCloud(my_pointcloud") != std::string::npos);
    MINI_CHECK(pccopy == pc && pccopy.guid != pc.guid);
    MINI_CHECK(TOLERANCE.is_close(pt0[0], 0.0) && TOLERANCE.is_close(pt0[1], 0.0) && TOLERANCE.is_close(pt0[2], 0.0));
    MINI_CHECK(col0.r == 255 && col0.g == 0 && col0.b == 0 && col0.a == 255);
    MINI_CHECK(TOLERANCE.is_close(norm0[0], 0.0) && TOLERANCE.is_close(norm0[1], 0.0) && TOLERANCE.is_close(norm0[2], 1.0));
    MINI_CHECK(pc2.len() == 1 && pc2.color_count() == 1 && pc2.normal_count() == 1);
    MINI_CHECK(TOLERANCE.is_close(pc2.get_point(0)[0], 4.0) && TOLERANCE.is_close(pc2.get_point(0)[1], 5.0) && TOLERANCE.is_close(pc2.get_point(0)[2], 6.0));
    MINI_CHECK(pc2.get_color(0).r == 200 && pc2.get_color(0).g == 100 && pc2.get_color(0).b == 50 && pc2.get_color(0).a == 255);
    MINI_CHECK(TOLERANCE.is_close(pc2.get_normal(0)[0], 0.0) && TOLERANCE.is_close(pc2.get_normal(0)[1], 1.0) && TOLERANCE.is_close(pc2.get_normal(0)[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pc_iadd.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc_isub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_isub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_isub.get_point(0)[2], -27.0));
    MINI_CHECK(TOLERANCE.is_close(pc_add.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_add.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_add.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc_sub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_sub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_sub.get_point(0)[2], -27.0));
    MINI_CHECK(TOLERANCE.is_close(pc3.get_point(0)[0], 1.0) && TOLERANCE.is_close(pc3.get_point(0)[1], 2.0) && TOLERANCE.is_close(pc3.get_point(0)[2], 3.0));
    MINI_CHECK(pc4.len() == 3 && pc4.color_count() == 3 && pc4.normal_count() == 3);
    MINI_CHECK(TOLERANCE.is_close(pc4.get_point(1)[0], 1.0) && TOLERANCE.is_close(pc4.get_point(1)[1], 0.0) && TOLERANCE.is_close(pc4.get_point(1)[2], 0.0));
    MINI_CHECK(pc4.get_color(1).r == 0 && pc4.get_color(1).g == 255 && pc4.get_color(1).b == 0 && pc4.get_color(1).a == 255);
    MINI_CHECK(TOLERANCE.is_close(pc4.get_normal(1)[0], 0.0) && TOLERANCE.is_close(pc4.get_normal(1)[1], 0.0) && TOLERANCE.is_close(pc4.get_normal(1)[2], 1.0));
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

    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc3.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc3.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc3.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc2.get_point(0)[0], 1.0) && TOLERANCE.is_close(pc2.get_point(0)[1], 2.0) && TOLERANCE.is_close(pc2.get_point(0)[2], 3.0));
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

    std::string fname = "serialization/test_pointcloud.json";
    pc.json_dump(fname);
    PointCloud loaded = PointCloud::json_load(fname);

    MINI_CHECK(loaded.name == "test_pointcloud");
    MINI_CHECK(loaded.len() == 2);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0) && TOLERANCE.is_close(loaded.get_point(0)[1], 2.0) && TOLERANCE.is_close(loaded.get_point(0)[2], 3.0));
    MINI_CHECK(loaded.get_color(0).r == 255 && loaded.get_color(0).g == 0 && loaded.get_color(0).b == 0);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
}

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

    std::string fname = "serialization/test_pointcloud.bin";
    pc.protobuf_dump(fname);
    PointCloud loaded = PointCloud::protobuf_load(fname);

    MINI_CHECK(loaded.name == "test_pointcloud");
    MINI_CHECK(loaded.len() == 2);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0) && TOLERANCE.is_close(loaded.get_point(0)[1], 2.0) && TOLERANCE.is_close(loaded.get_point(0)[2], 3.0));
    MINI_CHECK(loaded.get_color(0).r == 255 && loaded.get_color(0).g == 0 && loaded.get_color(0).b == 0);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
}

} // namespace session_cpp
