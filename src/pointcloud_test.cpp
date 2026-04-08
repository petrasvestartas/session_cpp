#include "mini_test.h"
#include "pointcloud.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "tolerance.h"

#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("PointCloud", "Constructor") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"
    // uncomment #include <vector>

    // Default constructor (empty cloud)
    PointCloud pc0;

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

    // Minimal and Full String Representation
    std::string pcstr = pc.str();
    std::string pcrepr = pc.repr();

    // Copy (duplicates everything except guid)
    PointCloud pccopy = pc;
    PointCloud pcother;

    // Copy operators
    Vector offset(10.0, 20.0, 30.0);
    PointCloud pc_iadd({Point(1.0, 2.0, 3.0)}, {}, {});
    pc_iadd += offset;
    PointCloud pc_isub({Point(1.0, 2.0, 3.0)}, {}, {});
    pc_isub -= offset;
    PointCloud pc3({Point(1.0, 2.0, 3.0)}, {}, {});
    PointCloud pc_add = pc3 + offset;
    PointCloud pc_sub = pc3 - offset;

    MINI_CHECK(pc0.name == "my_pointcloud");
    MINI_CHECK(!pc0.guid().empty());
    MINI_CHECK(pc0.is_empty());
    MINI_CHECK(pc.len() == 3);
    MINI_CHECK(pcstr == "3 points");
    MINI_CHECK(pcrepr == "PointCloud(my_pointcloud, 3 points, 3 colors, 3 normals)");
    MINI_CHECK(pccopy == pc && pccopy.guid() != pc.guid());
    MINI_CHECK(pcother != pc);
    MINI_CHECK(TOLERANCE.is_close(pc_iadd.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc_isub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_isub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_isub.get_point(0)[2], -27.0));
    MINI_CHECK(TOLERANCE.is_close(pc_add.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_add.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_add.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc_sub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_sub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_sub.get_point(0)[2], -27.0));
}

MINI_TEST("PointCloud", "From Coords") {
    // uncomment #include "pointcloud.h"
    // uncomment #include <vector>

    std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    std::vector<int> colors = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255};
    std::vector<double> normals = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
    PointCloud pc = PointCloud::from_coords(coords, colors, normals);

    MINI_CHECK(pc.len() == 3 && pc.color_count() == 3 && pc.normal_count() == 3);
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(1)[0], 1.0));
    MINI_CHECK(pc.get_color(1).g == 255);
    MINI_CHECK(TOLERANCE.is_close(pc.get_normal(1)[2], 1.0));
}

MINI_TEST("PointCloud", "Point Count") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)}, {}, {});

    MINI_CHECK(pc.point_count() == 3);
}

MINI_TEST("PointCloud", "Len") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0)}, {}, {});

    MINI_CHECK(pc.len() == 2);
}

MINI_TEST("PointCloud", "Is Empty") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc0;
    PointCloud pc1({Point(0.0, 0.0, 0.0)}, {}, {});

    MINI_CHECK(pc0.is_empty());
    MINI_CHECK(!pc1.is_empty());
}

MINI_TEST("PointCloud", "Get Point") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)}, {}, {});
    Point pt = pc.get_point(1);

    MINI_CHECK(TOLERANCE.is_close(pt[0], 4.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 6.0));
}

MINI_TEST("PointCloud", "Set Point") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc({Point(0.0, 0.0, 0.0)}, {}, {});
    pc.set_point(0, Point(4.0, 5.0, 6.0));

    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[0], 4.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[2], 6.0));
}

MINI_TEST("PointCloud", "Add Point") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc;
    pc.add_point(Point(1.0, 2.0, 3.0));

    MINI_CHECK(pc.len() == 1);
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[2], 3.0));
}

MINI_TEST("PointCloud", "Get Points") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"

    PointCloud pc({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)}, {}, {});
    std::vector<Point> points = pc.get_points();

    MINI_CHECK(points.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(points[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(points[1][2], 6.0));
}

MINI_TEST("PointCloud", "Color Count") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "color.h"

    PointCloud pc({}, {}, {Color(255, 0, 0, 255), Color(0, 255, 0, 255)});

    MINI_CHECK(pc.color_count() == 2);
}

MINI_TEST("PointCloud", "Get Color") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "color.h"

    PointCloud pc({}, {}, {Color(255, 0, 0, 255), Color(0, 255, 0, 255)});
    Color c = pc.get_color(1);

    MINI_CHECK(c.r == 0 && c.g == 255 && c.b == 0 && c.a == 255);
}

MINI_TEST("PointCloud", "Set Color") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "color.h"

    PointCloud pc({}, {}, {Color(0, 0, 0, 0)});
    pc.set_color(0, Color(200, 100, 50, 255));

    MINI_CHECK(pc.get_color(0).r == 200 && pc.get_color(0).g == 100 && pc.get_color(0).b == 50 && pc.get_color(0).a == 255);
}

MINI_TEST("PointCloud", "Add Color") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "color.h"

    PointCloud pc;
    pc.add_color(Color(128, 64, 32, 255));

    MINI_CHECK(pc.color_count() == 1);
    MINI_CHECK(pc.get_color(0).r == 128 && pc.get_color(0).g == 64 && pc.get_color(0).b == 32);
}

MINI_TEST("PointCloud", "Get Colors") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "color.h"

    PointCloud pc({}, {}, {Color(255, 0, 0, 255), Color(0, 255, 0, 255)});
    std::vector<Color> colors = pc.get_colors();

    MINI_CHECK(colors.size() == 2);
    MINI_CHECK(colors[0].r == 255);
    MINI_CHECK(colors[1].g == 255);
}

MINI_TEST("PointCloud", "Normal Count") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "vector.h"

    PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0)}, {});

    MINI_CHECK(pc.normal_count() == 2);
}

MINI_TEST("PointCloud", "Get Normal") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "vector.h"

    PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(1.0, 0.0, 0.0)}, {});
    Vector n = pc.get_normal(1);

    MINI_CHECK(TOLERANCE.is_close(n[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(n[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(n[2], 0.0));
}

MINI_TEST("PointCloud", "Set Normal") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "vector.h"

    PointCloud pc({}, {Vector(0.0, 0.0, 1.0)}, {});
    pc.set_normal(0, Vector(0.0, 1.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(pc.get_normal(0)[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_normal(0)[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_normal(0)[2], 0.0));
}

MINI_TEST("PointCloud", "Add Normal") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "vector.h"

    PointCloud pc;
    pc.add_normal(Vector(1.0, 0.0, 0.0));

    MINI_CHECK(pc.normal_count() == 1);
    MINI_CHECK(TOLERANCE.is_close(pc.get_normal(0)[0], 1.0));
}

MINI_TEST("PointCloud", "Get Normals") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "vector.h"

    PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(1.0, 0.0, 0.0)}, {});
    std::vector<Vector> normals = pc.get_normals();

    MINI_CHECK(normals.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(normals[0][2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(normals[1][0], 1.0));
}

MINI_TEST("PointCloud", "Transform") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    PointCloud pc({Point(1.0, 2.0, 3.0)}, {}, {});
    pc.xform = Xform::translation(10.0, 20.0, 30.0);
    pc.transform();

    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[0], 11.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[1], 22.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[2], 33.0));
}

MINI_TEST("PointCloud", "Transformed") {
    // uncomment #include "pointcloud.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    PointCloud pc({Point(1.0, 2.0, 3.0)}, {}, {});
    pc.xform = Xform::translation(10.0, 20.0, 30.0);
    PointCloud pc2 = pc.transformed();

    MINI_CHECK(TOLERANCE.is_close(pc2.get_point(0)[0], 11.0));
    MINI_CHECK(TOLERANCE.is_close(pc2.get_point(0)[1], 22.0));
    MINI_CHECK(TOLERANCE.is_close(pc2.get_point(0)[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(pc.get_point(0)[0], 1.0));
}

MINI_TEST("PointCloud", "Json Roundtrip") {
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

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   json_dumps()    │ std::string  │ to JSON string
    //   json_loads(s)   │ std::string  │ from JSON string
    //   json_dump(path) │ file         │ write to file
    //   json_load(path) │ file         │ read from file

    std::string fname = "serialization/test_pointcloud.json";
    pc.json_dump(fname);
    PointCloud loaded = PointCloud::json_load(fname);

    MINI_CHECK(loaded.name == "test_pointcloud");
    MINI_CHECK(loaded.len() == 2);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
    MINI_CHECK(loaded.get_color(0).r == 255);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
}

MINI_TEST("PointCloud", "Protobuf Roundtrip") {
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
    pc.pb_dump(fname);
    PointCloud loaded = PointCloud::pb_load(fname);

    MINI_CHECK(loaded.name == "test_pointcloud");
    MINI_CHECK(loaded.len() == 2);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
    MINI_CHECK(loaded.get_color(0).r == 255);
    MINI_CHECK(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
}

} // namespace session_cpp
