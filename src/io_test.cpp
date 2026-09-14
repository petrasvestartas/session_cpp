#include "mini_test.h"
#include "io.h"
#include "pointcloud.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Io", "Read Bunny") {
    if (!std::filesystem::exists("session_data/bunny.xyz"))
        return;
    PointCloud cloud = io::read_xyz("session_data/bunny.xyz");

    MINI_CHECK(cloud.point_count() == 397);
    std::vector<Point> points = cloud.get_points();
    MINI_CHECK(points.size() == 397);
    bool has_non_zero = false;
    for (const Point& p : points)
        if (p[0] != 0.0 || p[1] != 0.0 || p[2] != 0.0)
            has_non_zero = true;
    MINI_CHECK(has_non_zero);
}

MINI_TEST("Io", "Write Read Roundtrip") {
    std::filesystem::create_directories("./serialization");
    PointCloud original;
    original.add_point(Point(0.0, 0.0, 0.0));
    original.add_point(Point(1.0, 0.0, 0.0));
    original.add_point(Point(0.0, 1.0, 0.0));
    original.add_point(Point(0.0, 0.0, 1.0));

    MINI_CHECK(original.point_count() == 4);
    std::string temp_file = "./serialization/test_temp_roundtrip.xyz";
    io::write_xyz(original, temp_file);
    MINI_CHECK(std::filesystem::exists(temp_file));
    PointCloud loaded = io::read_xyz(temp_file);
    MINI_CHECK(loaded.point_count() == original.point_count());
    std::filesystem::remove(temp_file);
}

MINI_TEST("Io", "String Roundtrip") {
    PointCloud original;
    original.add_point(Point(0.0, 0.0, 0.0));
    original.add_point(Point(1.0, 0.0, 0.0));
    original.add_point(Point(0.0, 1.0, 0.0));
    original.add_point(Point(0.0, 0.0, 1.0));
    std::string s = io::write_xyz_to_string(original);
    PointCloud loaded = io::read_xyz_from_str(s);

    MINI_CHECK(loaded.point_count() == original.point_count());
    MINI_CHECK(TOLERANCE.is_close(loaded.get_points()[1][0], 1.0));
}

} // namespace session_cpp
