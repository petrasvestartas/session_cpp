#include "mini_test.h"
#include "io_xyz.h"
#include "pointcloud.h"
#include "tolerance.h"
#include <filesystem>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("IoXyz", "Read Bunny") {

        if (!std::filesystem::exists("session_data/bunny.xyz"))
            return;

        const PointCloud cloud = io_xyz::read_xyz("session_data/bunny.xyz");
        const std::vector<Point> points = cloud.get_points();
        bool has_non_zero = false;

        for (const Point& p : points)
            if (p[0] != 0.0 || p[1] != 0.0 || p[2] != 0.0)
                has_non_zero = true;

        MINI_CHECK(cloud.point_count() == 397);
        MINI_CHECK(points.size() == 397);
        MINI_CHECK(has_non_zero);
    }

    MINI_TEST("IoXyz", "Write Read Roundtrip") {

        std::filesystem::create_directories("./serialization");

        PointCloud original;
        original.add_point(Point(0.0, 0.0, 0.0));
        original.add_point(Point(1.0, 0.0, 0.0));
        original.add_point(Point(0.0, 1.0, 0.0));
        original.add_point(Point(0.0, 0.0, 1.0));

        const std::string filepath = "./serialization/test_temp_roundtrip.xyz";
        io_xyz::write_xyz(original, filepath);
        const bool exists = std::filesystem::exists(filepath);
        const PointCloud loaded = io_xyz::read_xyz(filepath);

        MINI_CHECK(original.point_count() == 4);
        MINI_CHECK(exists);
        MINI_CHECK(loaded.point_count() == original.point_count());

        std::filesystem::remove(filepath);
    }

    MINI_TEST("IoXyz", "String Roundtrip") {

        PointCloud original;
        original.add_point(Point(0.0, 0.0, 0.0));
        original.add_point(Point(1.0, 0.0, 0.0));
        original.add_point(Point(0.0, 1.0, 0.0));
        original.add_point(Point(0.0, 0.0, 1.0));

        const std::string content = io_xyz::write_xyz_to_string(original);
        const PointCloud loaded = io_xyz::read_xyz_from_str(content);

        MINI_CHECK(loaded.point_count() == original.point_count());
        MINI_CHECK(TOLERANCE.is_close(loaded.get_points()[1][0], 1.0));
    }

} // namespace session_cpp
