#pragma once
#include <string>
#include "pointcloud.h"

namespace session_cpp { namespace io {

std::string write_xyz_to_string(const PointCloud& cloud);
void write_xyz(const PointCloud& cloud, const std::string& filepath);
PointCloud read_xyz_from_str(const std::string& content);
PointCloud read_xyz(const std::string& filepath);

} } // namespace session_cpp::io
