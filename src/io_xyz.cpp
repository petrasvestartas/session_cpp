#include "io_xyz.h"
#include "fmt/core.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace session_cpp {
namespace io_xyz {

// ═══════════════════════════════════════════════════════════════════════════
// Write
// ═══════════════════════════════════════════════════════════════════════════
std::string write_xyz_to_string(const PointCloud& cloud) {

    std::string out;

    for (const Point& p : cloud.get_points())
        out += fmt::format("{} {} {}\n", p[0], p[1], p[2]);

    return out;
}

void write_xyz(const PointCloud& cloud, const std::string& filepath) {

    std::ofstream out(filepath, std::ios::binary);

    if (!out)
        throw std::runtime_error("Failed to open XYZ file: " + filepath);

    out << write_xyz_to_string(cloud);
}

// ═══════════════════════════════════════════════════════════════════════════
// Read
// ═══════════════════════════════════════════════════════════════════════════
PointCloud read_xyz_from_str(const std::string& content) {

    PointCloud cloud;
    std::istringstream in(content);
    std::string line;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        double x;
        double y;
        double z;

        if (!(iss >> x >> y >> z))
            continue;

        cloud.add_point(Point(x, y, z));
    }

    return cloud;
}

PointCloud read_xyz(const std::string& filepath) {

    std::ifstream in(filepath, std::ios::binary);

    if (!in)
        throw std::runtime_error("Failed to open XYZ file: " + filepath);

    std::stringstream buffer;
    buffer << in.rdbuf();

    return read_xyz_from_str(buffer.str());
}

} // namespace io_xyz
} // namespace session_cpp
