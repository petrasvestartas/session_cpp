#pragma once
#include "pointcloud.h"
#include <string>

namespace session_cpp {
namespace io_xyz {

// ═══════════════════════════════════════════════════════════════════════════
// Write
// ═══════════════════════════════════════════════════════════════════════════
/// Return the cloud points as "x y z" lines, each number the shortest round-trip text.
std::string write_xyz_to_string(const PointCloud& cloud);

/// Write the cloud points as "x y z" lines to filepath; throws if it cannot be opened.
void write_xyz(const PointCloud& cloud, const std::string& filepath);

// ═══════════════════════════════════════════════════════════════════════════
// Read
// ═══════════════════════════════════════════════════════════════════════════
/// Return the cloud read from "x y z" lines; blank and # lines skipped.
PointCloud read_xyz_from_str(const std::string& content);

/// Return the cloud read from an .xyz file; throws if it cannot be opened.
PointCloud read_xyz(const std::string& filepath);

} // namespace io_xyz
} // namespace session_cpp
