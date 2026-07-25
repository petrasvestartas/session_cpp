#include "io_xyz.h"
#include "pointcloud.h"
#include "point.h"
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <limits>

namespace session_cpp { namespace io {

std::string write_xyz_to_string(const PointCloud& cloud) {
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& p : cloud.get_points()) {
        out << p[0] << " " << p[1] << " " << p[2] << "\n";
    }
    return out.str();
}

void write_xyz(const PointCloud& cloud, const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        return; // Failed to open file
    }
    out << write_xyz_to_string(cloud);
    out.close(); // Explicitly close and flush
}

PointCloud read_xyz(const std::string& filepath) {
    std::ifstream in(filepath);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return read_xyz_from_str(buffer.str());
}

PointCloud read_xyz_from_str(const std::string& content) {
    std::istringstream in(content);
    std::string line;
    PointCloud cloud;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        double x, y, z;
        if (!(iss >> x >> y >> z)) continue;
        cloud.add_point(Point(x, y, z));
    }
    return cloud;
}

} } // namespace session_cpp::io
