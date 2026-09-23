#include "mini_test.h"
#include "pointcloud.h"
#include "pointcloud.pb.h"
#include "color.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include <string>
#include <utility>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("PointCloud", "Constructor") {

        const PointCloud pc0;

        const Point p0(0.0, 0.0, 0.0);
        const Point p1(1.0, 0.0, 0.0);
        const Point p2(0.0, 1.0, 0.0);
        const Vector n0(0.0, 0.0, 1.0);
        const Vector n1(0.0, 0.0, 1.0);
        const Vector n2(0.0, 0.0, 1.0);
        const Color c0(1.0f, 0.0f, 0.0f, 1.0f);
        const Color c1(0.0f, 1.0f, 0.0f, 1.0f);
        const Color c2(0.0f, 0.0f, 1.0f, 1.0f);
        const PointCloud pc({p0, p1, p2}, {n0, n1, n2}, {c0, c1, c2});

        const std::string pcstr = pc.str();
        const std::string pcrepr = pc.repr();

        const PointCloud pccopy = pc;
        const PointCloud pcother;

        const Vector offset(10.0, 20.0, 30.0);
        const PointCloud pc3({Point(1.0, 2.0, 3.0)}, {}, {});

        PointCloud pc_iadd = pc3;
        pc_iadd += offset;

        PointCloud pc_isub = pc3;
        pc_isub -= offset;

        const PointCloud pc_add = pc3 + offset;
        const PointCloud pc_sub = pc3 - offset;

        MINI_CHECK(pc0.name == "my_pointcloud");
        MINI_CHECK(!pc0.guid().empty());
        MINI_CHECK(pc0.is_empty());
        MINI_CHECK(pc.len() == 3);
        MINI_CHECK(pcstr == "3 points");
        MINI_CHECK(pcrepr == "PointCloud(my_pointcloud, 3 points, 3 colors, 3 normals)");
        MINI_CHECK(pccopy == pc && pccopy.guid() != pc.guid());
        MINI_CHECK(pcother != pc);
        MINI_CHECK(pc_iadd.get_point(0) == Point(11.0, 22.0, 33.0));
        MINI_CHECK(pc_isub.get_point(0) == Point(-9.0, -18.0, -27.0));
        MINI_CHECK(pc_add.get_point(0) == Point(11.0, 22.0, 33.0));
        MINI_CHECK(pc_sub.get_point(0) == Point(-9.0, -18.0, -27.0));
    }

    MINI_TEST("PointCloud", "From Coords") {

        const std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
        const std::vector<int> colors = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255};
        const std::vector<double> normals = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
        const PointCloud pc = PointCloud::from_coords(coords, colors, normals);

        MINI_CHECK(pc.len() == 3 && pc.color_count() == 3 && pc.normal_count() == 3);
        MINI_CHECK(pc.get_point(1) == Point(1.0, 0.0, 0.0));
        MINI_CHECK(pc.get_color(1) == Color(0.0f, 1.0f, 0.0f, 1.0f));
        MINI_CHECK(pc.get_normal(1) == Vector(0.0, 0.0, 1.0));
    }

    MINI_TEST("PointCloud", "Transform") {

        PointCloud pc({Point(1.0, 2.0, 3.0)}, {Vector(1.0, 0.0, 0.0)}, {});
        const Xform xform = Xform::translation(10.0, 20.0, 30.0);
        pc.transform(xform);

        MINI_CHECK(pc.get_point(0) == Point(11.0, 22.0, 33.0));
        MINI_CHECK(pc.get_normal(0) == Vector(1.0, 0.0, 0.0));
    }

    MINI_TEST("PointCloud", "Transformed") {

        const PointCloud pc({Point(1.0, 2.0, 3.0)}, {}, {});
        const Xform xform = Xform::translation(10.0, 20.0, 30.0);
        const PointCloud moved = pc.transformed(xform);

        MINI_CHECK(moved.get_point(0) == Point(11.0, 22.0, 33.0));
        MINI_CHECK(pc.get_point(0) == Point(1.0, 2.0, 3.0));
    }

    MINI_TEST("PointCloud", "Point Count") {

        const PointCloud pc({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)}, {}, {});

        MINI_CHECK(pc.point_count() == 3);
    }

    MINI_TEST("PointCloud", "Len") {

        const PointCloud pc({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0)}, {}, {});

        MINI_CHECK(pc.len() == 2);
    }

    MINI_TEST("PointCloud", "Is Empty") {

        const PointCloud pc0;
        const PointCloud pc1({Point(0.0, 0.0, 0.0)}, {}, {});

        MINI_CHECK(pc0.is_empty());
        MINI_CHECK(!pc1.is_empty());
    }

    MINI_TEST("PointCloud", "Get Point") {

        const PointCloud pc({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)}, {}, {});
        const Point point = pc.get_point(1);

        MINI_CHECK(point == Point(4.0, 5.0, 6.0));
    }

    MINI_TEST("PointCloud", "Set Point") {

        PointCloud pc({Point(0.0, 0.0, 0.0)}, {}, {});
        pc.set_point(0, Point(4.0, 5.0, 6.0));

        MINI_CHECK(pc.get_point(0) == Point(4.0, 5.0, 6.0));
    }

    MINI_TEST("PointCloud", "Add Point") {

        PointCloud pc;
        pc.add_point(Point(1.0, 2.0, 3.0));

        MINI_CHECK(pc.len() == 1);
        MINI_CHECK(pc.get_point(0) == Point(1.0, 2.0, 3.0));
    }

    MINI_TEST("PointCloud", "Get Points") {

        const PointCloud pc({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)}, {}, {});
        const std::vector<Point> points = pc.get_points();

        MINI_CHECK(points.size() == 2);
        MINI_CHECK(points[0] == Point(1.0, 2.0, 3.0));
        MINI_CHECK(points[1] == Point(4.0, 5.0, 6.0));
    }

    MINI_TEST("PointCloud", "Coords") {

        const PointCloud pc({Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)}, {}, {});
        const std::vector<double>& coords = pc.coords();

        MINI_CHECK(coords.size() == 6);
        MINI_CHECK(coords[0] == 1.0 && coords[5] == 6.0);
    }

    MINI_TEST("PointCloud", "Color Count") {

        const PointCloud pc({}, {}, {Color(1.0f, 0.0f, 0.0f, 1.0f), Color(0.0f, 1.0f, 0.0f, 1.0f)});

        MINI_CHECK(pc.color_count() == 2);
    }

    MINI_TEST("PointCloud", "Get Color") {

        const PointCloud pc({}, {}, {Color(1.0f, 0.0f, 0.0f, 1.0f), Color(0.0f, 1.0f, 0.0f, 1.0f)});
        const Color color = pc.get_color(1);

        MINI_CHECK(color == Color(0.0f, 1.0f, 0.0f, 1.0f));
    }

    MINI_TEST("PointCloud", "Set Color") {

        PointCloud pc({}, {}, {Color(0.0f, 0.0f, 0.0f, 0.0f)});
        pc.set_color(0, Color(1.0f, 0.0f, 0.0f, 1.0f));

        MINI_CHECK(pc.get_color(0) == Color(1.0f, 0.0f, 0.0f, 1.0f));
    }

    MINI_TEST("PointCloud", "Add Color") {

        PointCloud pc;
        pc.add_color(Color(1.0f, 0.0f, 1.0f, 1.0f));

        MINI_CHECK(pc.color_count() == 1);
        MINI_CHECK(pc.get_color(0) == Color(1.0f, 0.0f, 1.0f, 1.0f));
    }

    MINI_TEST("PointCloud", "Get Colors") {

        const PointCloud pc({}, {}, {Color(1.0f, 0.0f, 0.0f, 1.0f), Color(0.0f, 1.0f, 0.0f, 1.0f)});
        const std::vector<Color> colors = pc.get_colors();

        MINI_CHECK(colors.size() == 2);
        MINI_CHECK(colors[0] == Color(1.0f, 0.0f, 0.0f, 1.0f));
        MINI_CHECK(colors[1] == Color(0.0f, 1.0f, 0.0f, 1.0f));
    }

    MINI_TEST("PointCloud", "Colors") {

        const PointCloud pc({}, {}, {Color(1.0f, 0.0f, 0.0f, 1.0f)});
        const std::vector<int>& colors = pc.colors();

        MINI_CHECK(colors.size() == 4);
        MINI_CHECK(colors[0] == 255 && colors[1] == 0 && colors[2] == 0 && colors[3] == 255);
    }

    MINI_TEST("PointCloud", "Normal Count") {

        const PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0)}, {});

        MINI_CHECK(pc.normal_count() == 2);
    }

    MINI_TEST("PointCloud", "Get Normal") {

        const PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(1.0, 0.0, 0.0)}, {});
        const Vector normal = pc.get_normal(1);

        MINI_CHECK(normal == Vector(1.0, 0.0, 0.0));
    }

    MINI_TEST("PointCloud", "Set Normal") {

        PointCloud pc({}, {Vector(0.0, 0.0, 1.0)}, {});
        pc.set_normal(0, Vector(0.0, 1.0, 0.0));

        MINI_CHECK(pc.get_normal(0) == Vector(0.0, 1.0, 0.0));
    }

    MINI_TEST("PointCloud", "Add Normal") {

        PointCloud pc;
        pc.add_normal(Vector(1.0, 0.0, 0.0));

        MINI_CHECK(pc.normal_count() == 1);
        MINI_CHECK(pc.get_normal(0) == Vector(1.0, 0.0, 0.0));
    }

    MINI_TEST("PointCloud", "Get Normals") {

        const PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(1.0, 0.0, 0.0)}, {});
        const std::vector<Vector> normals = pc.get_normals();

        MINI_CHECK(normals.size() == 2);
        MINI_CHECK(normals[0] == Vector(0.0, 0.0, 1.0));
        MINI_CHECK(normals[1] == Vector(1.0, 0.0, 0.0));
    }

    MINI_TEST("PointCloud", "Normals") {

        const PointCloud pc({}, {Vector(0.0, 0.0, 1.0), Vector(1.0, 0.0, 0.0)}, {});
        const std::vector<double>& normals = pc.normals();

        MINI_CHECK(normals.size() == 6);
        MINI_CHECK(normals[2] == 1.0 && normals[3] == 1.0);
    }

    MINI_TEST("PointCloud", "Build Lod") {

        const std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        PointCloud pc = PointCloud::from_coords(coords, {}, {});
        pc.build_lod(1.0, 2);

        const std::pair<Point, double> cube = pc.lod_cube(0);
        const std::pair<int, int> span = pc.lod_range(0);
        const std::vector<int> children = pc.lod_children(0);

        MINI_CHECK(pc.has_lod());
        MINI_CHECK(pc.lod_node_count() == 8);
        MINI_CHECK(cube.first == Point(0.5, 0.5, 0.5) && cube.second == 1.0);
        MINI_CHECK(pc.lod_spacing(0) == 1.0 && pc.lod_level(1) == 1);
        MINI_CHECK(span.first == 0 && span.second == 1);
        MINI_CHECK(children[0] == 1 && children[7] == -1);
        MINI_CHECK(pc.coords().size() == 24);
    }

    MINI_TEST("PointCloud", "Point Ids") {

        const std::vector<double> coords = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        PointCloud pc = PointCloud::from_coords(coords, {}, {});
        const Point before = pc.get_point(5);
        pc.build_lod(1.0, 2);

        const int index = pc.index_of_id(5);

        MINI_CHECK(pc.point_ids().size() == 8);
        MINI_CHECK(index >= 0);
        MINI_CHECK(pc.point_id(index) == 5);
        MINI_CHECK(pc.get_point(index) == before);
    }

    MINI_TEST("PointCloud", "Json Roundtrip") {

        PointCloud pc(
            {Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)},
            {Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0)},
            {Color(1.0f, 0.0f, 0.0f, 1.0f), Color(0.0f, 1.0f, 0.0f, 1.0f)}
        );
        pc.name = "test_pointcloud";

        const std::string guid = pc.guid();
        const std::string filename = "serialization/test_pointcloud.json";
        pc.file_json_dump(filename);

        const PointCloud loaded = PointCloud::file_json_load(filename);
        const PointCloud parsed = PointCloud::file_json_loads(pc.file_json_dumps());

        MINI_CHECK(loaded == pc);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed == pc);
        MINI_CHECK(parsed.guid() == guid);
    }

    MINI_TEST("PointCloud", "Protobuf Roundtrip") {

        const PointCloud fresh;
        const session_proto::PointCloud fresh_proto = fresh.to_proto();
        PointCloud pc(
            {Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0)},
            {Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0)},
            {Color(1.0f, 0.0f, 0.0f, 1.0f), Color(0.0f, 1.0f, 0.0f, 1.0f)}
        );
        pc.name = "test_pointcloud";

        const std::string guid = pc.guid();
        const std::string filename = "serialization/test_pointcloud.bin";
        pc.pb_dump(filename);

        const PointCloud loaded = PointCloud::pb_load(filename);
        const PointCloud parsed = PointCloud::pb_loads(pc.pb_dumps());
        const PointCloud converted = PointCloud::from_proto(pc.to_proto());

        MINI_CHECK(!fresh.has_guid());
        MINI_CHECK(fresh_proto.guid().empty());
        MINI_CHECK(loaded == pc);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed == pc);
        MINI_CHECK(parsed.guid() == guid);
        MINI_CHECK(converted == pc);
        MINI_CHECK(converted.guid() == guid);
    }

} // namespace session_cpp
