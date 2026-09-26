#include "mini_test.h"
#include "io_xyz.h"
#include "pointcloud.h"
#include "tolerance.h"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
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

    MINI_TEST("IoXyz", "Write Exact Text") {

        std::filesystem::create_directories("./serialization");

        PointCloud original;
        original.add_point(Point(1.0, 2.5, -3.0));
        original.add_point(Point(0.1, 1e-05, 1e+16));
        original.add_point(Point(123456.789, -0.0, 1.0 / 3.0));
        original.add_point(Point(7.120236347223045e-307, 6.386688990511104e+293, 0.0001220703125));

        const std::string filepath = "./serialization/test_temp_exact.xyz";
        io_xyz::write_xyz(original, filepath);
        std::ifstream file(filepath, std::ios::binary);
        const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();
        const PointCloud loaded = io_xyz::read_xyz(filepath);

        MINI_CHECK(text == "1 2.5 -3\n0.1 1e-05 1e+16\n123456.789 -0 0.3333333333333333\n7.120236347223045e-307 6.386688990511104e+293 0.0001220703125\n");
        MINI_CHECK(io_xyz::write_xyz_to_string(loaded) == text);
        MINI_CHECK(loaded.get_points()[2][2] == 1.0 / 3.0);

        std::filesystem::remove(filepath);
    }

    MINI_TEST("IoXyz", "File Errors") {

        const PointCloud cloud;
        bool read_failed = false;
        bool write_failed = false;

        try {
            io_xyz::read_xyz("./serialization/test_temp_missing.xyz");
        } catch (const std::runtime_error&) {
            read_failed = true;
        }

        try {
            io_xyz::write_xyz(cloud, "");
        } catch (const std::runtime_error&) {
            write_failed = true;
        }

        MINI_CHECK(read_failed);
        MINI_CHECK(write_failed);
    }

} // namespace session_cpp
