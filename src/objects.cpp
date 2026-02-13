#include "objects.h"
#include "objects.pb.h"

namespace session_cpp {

std::string Objects::str() const {
  return fmt::format("Objects(name={}, guid={}, points={})", name, guid,
                     points->size());
}

nlohmann::ordered_json Objects::jsondump() const {
  // Build JSON arrays for all geometry types (in alphabetical order)
  std::vector<nlohmann::ordered_json> bboxes_json;
  bboxes_json.reserve(bboxes->size());
  for (const auto &b : *bboxes) {
    bboxes_json.push_back(b->jsondump());
  }
  
  std::vector<nlohmann::ordered_json> lines_json;
  lines_json.reserve(lines->size());
  for (const auto &l : *lines) {
    lines_json.push_back(l->jsondump());
  }
  
  std::vector<nlohmann::ordered_json> meshes_json;
  meshes_json.reserve(meshes->size());
  for (const auto &m : *meshes) {
    meshes_json.push_back(m->jsondump());
  }

  std::vector<nlohmann::ordered_json> nurbscurves_json;
  nurbscurves_json.reserve(nurbscurves->size());
  for (const auto &nc : *nurbscurves) {
    nurbscurves_json.push_back(nc->jsondump());
  }

  std::vector<nlohmann::ordered_json> nurbssurfaces_json;
  nurbssurfaces_json.reserve(nurbssurfaces->size());
  for (const auto &ns : *nurbssurfaces) {
    nurbssurfaces_json.push_back(ns->jsondump());
  }
  
  std::vector<nlohmann::ordered_json> planes_json;
  planes_json.reserve(planes->size());
  for (const auto &p : *planes) {
    planes_json.push_back(p->jsondump());
  }
  
  std::vector<nlohmann::ordered_json> points_json;
  points_json.reserve(points->size());
  for (const auto &p : *points) {
    points_json.push_back(p->jsondump());
  }
  
  std::vector<nlohmann::ordered_json> pointclouds_json;
  pointclouds_json.reserve(pointclouds->size());
  for (const auto &pc : *pointclouds) {
    pointclouds_json.push_back(pc->jsondump());
  }
  
  std::vector<nlohmann::ordered_json> polylines_json;
  polylines_json.reserve(polylines->size());
  for (const auto &pl : *polylines) {
    polylines_json.push_back(pl->jsondump());
  }

  return nlohmann::ordered_json{{"type", "Objects"},
                                {"guid", guid},
                                {"name", name},
                                {"bboxes", bboxes_json},
                                {"lines", lines_json},
                                {"meshes", meshes_json},
                                {"nurbscurves", nurbscurves_json},
                                {"nurbssurfaces", nurbssurfaces_json},
                                {"planes", planes_json},
                                {"points", points_json},
                                {"pointclouds", pointclouds_json},
                                {"polylines", polylines_json}};
}

Objects Objects::jsonload(const nlohmann::json &data) {
  // Create Objects instance
  Objects objects(data["name"].get<std::string>());
  
  // Load bboxes
  if (data.contains("bboxes")) {
    std::vector<std::shared_ptr<BoundingBox>> bboxes;
    bboxes.reserve(data["bboxes"].size());
    for (const auto &bbox_data : data["bboxes"])
      bboxes.push_back(std::make_shared<BoundingBox>(BoundingBox::jsonload(bbox_data)));
    *objects.bboxes = std::move(bboxes);
  }
  
  // Load lines
  if (data.contains("lines")) {
    std::vector<std::shared_ptr<Line>> lines;
    lines.reserve(data["lines"].size());
    for (const auto &line_data : data["lines"])
      lines.push_back(std::make_shared<Line>(Line::jsonload(line_data)));
    *objects.lines = std::move(lines);
  }
  
  // Load meshes
  if (data.contains("meshes")) {
    std::vector<std::shared_ptr<Mesh>> meshes;
    meshes.reserve(data["meshes"].size());
    for (const auto &mesh_data : data["meshes"])
      meshes.push_back(std::make_shared<Mesh>(Mesh::jsonload(mesh_data)));
    *objects.meshes = std::move(meshes);
  }
  
  // Load nurbscurves
  if (data.contains("nurbscurves")) {
    std::vector<std::shared_ptr<NurbsCurve>> nurbscurves;
    nurbscurves.reserve(data["nurbscurves"].size());
    for (const auto &nc_data : data["nurbscurves"])
      nurbscurves.push_back(std::make_shared<NurbsCurve>(NurbsCurve::jsonload(nc_data)));
    *objects.nurbscurves = std::move(nurbscurves);
  }

  // Load nurbssurfaces
  if (data.contains("nurbssurfaces")) {
    std::vector<std::shared_ptr<NurbsSurface>> nurbssurfaces;
    nurbssurfaces.reserve(data["nurbssurfaces"].size());
    for (const auto &ns_data : data["nurbssurfaces"])
      nurbssurfaces.push_back(std::make_shared<NurbsSurface>(NurbsSurface::jsonload(ns_data)));
    *objects.nurbssurfaces = std::move(nurbssurfaces);
  }

  // Load planes
  if (data.contains("planes")) {
    std::vector<std::shared_ptr<Plane>> planes;
    planes.reserve(data["planes"].size());
    for (const auto &plane_data : data["planes"])
      planes.push_back(std::make_shared<Plane>(Plane::jsonload(plane_data)));
    *objects.planes = std::move(planes);
  }
  
  // Load points
  if (data.contains("points")) {
    std::vector<std::shared_ptr<Point>> points;
    points.reserve(data["points"].size());
    for (const auto &point_data : data["points"])
      points.push_back(std::make_shared<Point>(Point::jsonload(point_data)));
    *objects.points = std::move(points);
  }
  
  // Load pointclouds
  if (data.contains("pointclouds")) {
    std::vector<std::shared_ptr<PointCloud>> pointclouds;
    pointclouds.reserve(data["pointclouds"].size());
    for (const auto &pointcloud_data : data["pointclouds"])
      pointclouds.push_back(std::make_shared<PointCloud>(PointCloud::jsonload(pointcloud_data)));
    *objects.pointclouds = std::move(pointclouds);
  }
  
  // Load polylines
  if (data.contains("polylines")) {
    std::vector<std::shared_ptr<Polyline>> polylines;
    polylines.reserve(data["polylines"].size());
    for (const auto &polyline_data : data["polylines"])
      polylines.push_back(std::make_shared<Polyline>(Polyline::jsonload(polyline_data)));
    *objects.polylines = std::move(polylines);
  }

  // Set guid if provided, otherwise generate a new one
  objects.guid =
      data.contains("guid") ? data["guid"].get<std::string>() : ::guid();

  return objects;
}

std::string Objects::json_dumps() const {
  return jsondump().dump();
}

Objects Objects::json_loads(const std::string& json_string) {
  return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Objects::json_dump(const std::string& filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

Objects Objects::json_load(const std::string& filename) {
  std::ifstream file(filename);
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

std::string Objects::pb_dumps() const {
  session_proto::Objects proto;
  proto.set_name(name);
  proto.set_guid(guid);
  for (const auto& p : *points) proto.add_points()->ParseFromString(p->pb_dumps());
  for (const auto& l : *lines) proto.add_lines()->ParseFromString(l->pb_dumps());
  for (const auto& pl : *planes) proto.add_planes()->ParseFromString(pl->pb_dumps());
  for (const auto& b : *bboxes) proto.add_bboxes()->ParseFromString(b->pb_dumps());
  for (const auto& pl : *polylines) proto.add_polylines()->ParseFromString(pl->pb_dumps());
  for (const auto& pc : *pointclouds) proto.add_pointclouds()->ParseFromString(pc->pb_dumps());
  for (const auto& m : *meshes) proto.add_meshes()->ParseFromString(m->pb_dumps());
  for (const auto& nc : *nurbscurves) proto.add_nurbscurves()->ParseFromString(nc->pb_dumps());
  for (const auto& ns : *nurbssurfaces) proto.add_nurbssurfaces()->ParseFromString(ns->pb_dumps());
  return proto.SerializeAsString();
}

Objects Objects::pb_loads(const std::string& data) {
  session_proto::Objects proto;
  proto.ParseFromString(data);
  Objects objects(proto.name());
  objects.guid = proto.guid();
  for (const auto& p : proto.points())
    objects.points->push_back(std::make_shared<Point>(Point::pb_loads(p.SerializeAsString())));
  for (const auto& l : proto.lines())
    objects.lines->push_back(std::make_shared<Line>(Line::pb_loads(l.SerializeAsString())));
  for (const auto& p : proto.planes())
    objects.planes->push_back(std::make_shared<Plane>(Plane::pb_loads(p.SerializeAsString())));
  for (const auto& b : proto.bboxes())
    objects.bboxes->push_back(std::make_shared<BoundingBox>(BoundingBox::pb_loads(b.SerializeAsString())));
  for (const auto& p : proto.polylines())
    objects.polylines->push_back(std::make_shared<Polyline>(Polyline::pb_loads(p.SerializeAsString())));
  for (const auto& p : proto.pointclouds())
    objects.pointclouds->push_back(std::make_shared<PointCloud>(PointCloud::pb_loads(p.SerializeAsString())));
  for (const auto& m : proto.meshes())
    objects.meshes->push_back(std::make_shared<Mesh>(Mesh::pb_loads(m.SerializeAsString())));
  for (const auto& nc : proto.nurbscurves())
    objects.nurbscurves->push_back(std::make_shared<NurbsCurve>(NurbsCurve::pb_loads(nc.SerializeAsString())));
  for (const auto& ns : proto.nurbssurfaces())
    objects.nurbssurfaces->push_back(std::make_shared<NurbsSurface>(NurbsSurface::pb_loads(ns.SerializeAsString())));
  return objects;
}

void Objects::pb_dump(const std::string& filename) const {
  std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Objects Objects::pb_load(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return pb_loads(data);
}

std::ostream &operator<<(std::ostream &os, const Objects &objects) {
  return os << objects.str();
}
} // namespace session_cpp
