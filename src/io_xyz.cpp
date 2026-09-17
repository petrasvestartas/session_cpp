#include "io_xyz.h"
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace session_cpp { namespace io_xyz {

std::string write_xyz_to_string(const PointCloud& cloud) {

    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10);

    for (const Point& p : cloud.get_points())
        out << p[0] << " " << p[1] << " " << p[2] << "\n";

    return out.str();
}

void write_xyz(const PointCloud& cloud, const std::string& filepath) {

    std::ofstream out(filepath);

    if (!out.is_open())
        return;

    out << write_xyz_to_string(cloud);
}

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

    std::ifstream in(filepath);
    std::stringstream buffer;
    buffer << in.rdbuf();

    return read_xyz_from_str(buffer.str());
}

} } // namespace session_cpp::io_xyz
