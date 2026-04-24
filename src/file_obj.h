#pragma once
#include <string>
#include <vector>
#include "mesh.h"
#include "polyline.h"

namespace session_cpp { namespace file_obj {

void write_file_obj(const Mesh& mesh, const std::string& filepath);
Mesh read_file_obj(const std::string& filepath);
std::vector<Polyline> read_file_obj_polylines(const std::string& filepath);
std::vector<std::pair<int,int>> pair_polylines(const std::vector<Polyline>& polylines, double search_radius = 500.0);

} } // namespace session_cpp::file_obj
