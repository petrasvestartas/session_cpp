#pragma once
#include <string>
#include <vector>
#include "mesh.h"
#include "polyline.h"

namespace session_cpp { namespace obj {

void write_obj(const Mesh& mesh, const std::string& filepath);
Mesh read_obj(const std::string& filepath);
std::vector<Polyline> read_obj_polylines(const std::string& filepath);

} } // namespace session_cpp::obj
