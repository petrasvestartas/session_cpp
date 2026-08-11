#pragma once
#include <string>
#include <vector>
#include "mesh.h"
#include "polyline.h"

namespace session_cpp { namespace file_obj {

std::string write_file_obj_to_string(const Mesh& mesh);
void write_file_obj(const Mesh& mesh, const std::string& filepath);
Mesh read_file_obj_from_str(const std::string& content);
Mesh read_file_obj(const std::string& filepath);
std::vector<Polyline> read_file_obj_polylines(const std::string& filepath);

} } // namespace session_cpp::file_obj
