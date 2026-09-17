#pragma once
#include <string>
#include <vector>
#include "mesh.h"
#include "polyline.h"

namespace session_cpp {
namespace file_obj {

/// OBJ text of the mesh: one v line per vertex, one f line per face with 1-based indices.
std::string write_file_obj_to_string(const Mesh& mesh);

/// Write the mesh as an OBJ file.
void write_file_obj(const Mesh& mesh, const std::string& filepath);

/// Mesh from OBJ text; v and f lines only, negative indices count from the end.
Mesh read_file_obj_from_str(const std::string& content);

/// Mesh from an OBJ file.
Mesh read_file_obj(const std::string& filepath);

/// Polylines from the curv blocks of an OBJ file.
std::vector<Polyline> read_file_obj_polylines(const std::string& filepath);

} // namespace file_obj
} // namespace session_cpp
