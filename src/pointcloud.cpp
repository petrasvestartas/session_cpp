#include "pointcloud.h"
#include "spatial_octree.h"
#include "pointcloud.pb.h"
#include <cmath>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

PointCloud::PointCloud() {}

PointCloud::PointCloud(const std::vector<Point> &points, const std::vector<Vector> &normals, const std::vector<Color> &colors) {
  _coords.reserve(points.size() * 3);
  for (const Point &p : points)
    add_point(p);
  _normals.reserve(normals.size() * 3);
  for (const Vector &n : normals)
    add_normal(n);
  _colors.reserve(colors.size() * 4);
  for (const Color &c : colors)
    add_color(c);
}

PointCloud::PointCloud(const PointCloud &other)
    : name(other.name), point_size(other.point_size), _coords(other._coords), _colors(other._colors),
      _normals(other._normals), _lod_min(other._lod_min), _lod_size(other._lod_size), _lod_spacing(other._lod_spacing),
      _lod_level(other._lod_level), _lod_first(other._lod_first), _lod_count(other._lod_count),
      _lod_children(other._lod_children), _point_ids(other._point_ids) {}

PointCloud &PointCloud::operator=(const PointCloud &other) {
  if (this == &other)
    return *this;
  _guid.clear();
  name = other.name;
  point_size = other.point_size;
  _coords = other._coords;
  _colors = other._colors;
  _normals = other._normals;
  _lod_min = other._lod_min;
  _lod_size = other._lod_size;
  _lod_spacing = other._lod_spacing;
  _lod_level = other._lod_level;
  _lod_first = other._lod_first;
  _lod_count = other._lod_count;
  _lod_children = other._lod_children;
  _point_ids = other._point_ids;
  return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════

PointCloud PointCloud::from_coords(const std::vector<double> &coords, const std::vector<int> &colors, const std::vector<double> &normals) {
  PointCloud cloud;
  cloud._coords = coords;
  cloud._colors = colors;
  cloud._normals = normals;
  return cloud;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

bool PointCloud::operator==(const PointCloud &other) const {
  return name == other.name && _coords == other._coords && _colors == other._colors && _normals == other._normals &&
         _lod_first == other._lod_first && _lod_count == other._lod_count && _point_ids == other._point_ids;
}

bool PointCloud::operator!=(const PointCloud &other) const { return !(*this == other); }

PointCloud &PointCloud::operator+=(const Vector &other) {
  for (size_t i = 0; i < _coords.size(); i += 3) {
    _coords[i] += other[0];
    _coords[i + 1] += other[1];
    _coords[i + 2] += other[2];
  }
  return *this;
}

PointCloud &PointCloud::operator-=(const Vector &other) {
  for (size_t i = 0; i < _coords.size(); i += 3) {
    _coords[i] -= other[0];
    _coords[i + 1] -= other[1];
    _coords[i + 2] -= other[2];
  }
  return *this;
}

PointCloud PointCloud::operator+(const Vector &other) const {
  PointCloud result = *this;
  result += other;
  return result;
}

PointCloud PointCloud::operator-(const Vector &other) const {
  PointCloud result = *this;
  result -= other;
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

void PointCloud::transform(const Xform &xform) {
  for (size_t i = 0; i < point_count(); i++)
    set_point(i, get_point(i).transformed(xform));
  for (size_t i = 0; i < normal_count(); i++)
    set_normal(i, get_normal(i).transformed(xform));
}

PointCloud PointCloud::transformed(const Xform &xform) const {
  PointCloud result = *this;
  result.transform(xform);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Points
// ═══════════════════════════════════════════════════════════════════════════

Point PointCloud::get_point(size_t index) const {
  const size_t idx = index * 3;
  return Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
}

void PointCloud::set_point(size_t index, const Point &point) {
  const size_t idx = index * 3;
  _coords[idx] = point[0];
  _coords[idx + 1] = point[1];
  _coords[idx + 2] = point[2];
}

void PointCloud::add_point(const Point &point) {
  _coords.push_back(point[0]);
  _coords.push_back(point[1]);
  _coords.push_back(point[2]);
}

std::vector<Point> PointCloud::get_points() const {
  std::vector<Point> points;
  points.reserve(point_count());
  for (size_t i = 0; i < point_count(); i++)
    points.push_back(get_point(i));
  return points;
}

// ═══════════════════════════════════════════════════════════════════════════
// Colors
// ═══════════════════════════════════════════════════════════════════════════

Color PointCloud::get_color(size_t index) const {
  const size_t idx = index * 4;
  return Color(_colors[idx] / 255.0f, _colors[idx + 1] / 255.0f, _colors[idx + 2] / 255.0f, _colors[idx + 3] / 255.0f);
}

void PointCloud::set_color(size_t index, const Color &color) {
  const size_t idx = index * 4;
  _colors[idx] = static_cast<int>(std::lround(color.r * 255.0f));
  _colors[idx + 1] = static_cast<int>(std::lround(color.g * 255.0f));
  _colors[idx + 2] = static_cast<int>(std::lround(color.b * 255.0f));
  _colors[idx + 3] = static_cast<int>(std::lround(color.a * 255.0f));
}

void PointCloud::add_color(const Color &color) {
  _colors.push_back(static_cast<int>(std::lround(color.r * 255.0f)));
  _colors.push_back(static_cast<int>(std::lround(color.g * 255.0f)));
  _colors.push_back(static_cast<int>(std::lround(color.b * 255.0f)));
  _colors.push_back(static_cast<int>(std::lround(color.a * 255.0f)));
}

std::vector<Color> PointCloud::get_colors() const {
  std::vector<Color> colors;
  colors.reserve(color_count());
  for (size_t i = 0; i < color_count(); i++)
    colors.push_back(get_color(i));
  return colors;
}

// ═══════════════════════════════════════════════════════════════════════════
// Normals
// ═══════════════════════════════════════════════════════════════════════════

Vector PointCloud::get_normal(size_t index) const {
  const size_t idx = index * 3;
  return Vector(_normals[idx], _normals[idx + 1], _normals[idx + 2]);
}

void PointCloud::set_normal(size_t index, const Vector &normal) {
  const size_t idx = index * 3;
  _normals[idx] = normal[0];
  _normals[idx + 1] = normal[1];
  _normals[idx + 2] = normal[2];
}

void PointCloud::add_normal(const Vector &normal) {
  _normals.push_back(normal[0]);
  _normals.push_back(normal[1]);
  _normals.push_back(normal[2]);
}

std::vector<Vector> PointCloud::get_normals() const {
  std::vector<Vector> normals;
  normals.reserve(normal_count());
  for (size_t i = 0; i < normal_count(); i++)
    normals.push_back(get_normal(i));
  return normals;
}

// ═══════════════════════════════════════════════════════════════════════════
// LOD octree
// ═══════════════════════════════════════════════════════════════════════════

void PointCloud::build_lod(double root_spacing, int leaf_capacity) {
  const SpatialOctree tree = SpatialOctree::from_coords(_coords, root_spacing, leaf_capacity);
  const std::vector<int> &order = tree.order();
  if (_point_ids.empty())
    for (size_t i = 0; i < point_count(); i++)
      _point_ids.push_back(static_cast<int>(i));

  const bool has_colors = _colors.size() == order.size() * 4;
  const bool has_normals = _normals.size() == order.size() * 3;
  std::vector<double> coords;
  std::vector<int> colors;
  std::vector<double> normals;
  std::vector<int> ids;
  coords.reserve(_coords.size());
  colors.reserve(_colors.size());
  normals.reserve(_normals.size());
  ids.reserve(_point_ids.size());
  for (int idx : order) {
    ids.push_back(_point_ids[idx]);
    for (int k = 0; k < 3; k++)
      coords.push_back(_coords[idx * 3 + k]);
    if (has_colors)
      for (int k = 0; k < 4; k++)
        colors.push_back(_colors[idx * 4 + k]);
    if (has_normals)
      for (int k = 0; k < 3; k++)
        normals.push_back(_normals[idx * 3 + k]);
  }
  _coords = std::move(coords);
  _point_ids = std::move(ids);
  if (has_colors)
    _colors = std::move(colors);
  if (has_normals)
    _normals = std::move(normals);

  _lod_min.clear();
  _lod_size.clear();
  _lod_spacing.clear();
  _lod_level.clear();
  _lod_first.clear();
  _lod_count.clear();
  _lod_children.clear();
  for (int i = 0; i < tree.node_count(); i++) {
    const std::pair<Point, double> cube = tree.node_cube(i);
    const std::pair<int, int> range = tree.node_range(i);
    const std::vector<int> kids = tree.children(i);
    for (int k = 0; k < 3; k++)
      _lod_min.push_back(cube.first[k] - cube.second * 0.5);
    _lod_size.push_back(cube.second);
    _lod_spacing.push_back(tree.node_spacing(i));
    _lod_level.push_back(tree.node_level(i));
    _lod_first.push_back(range.first);
    _lod_count.push_back(range.second);
    for (int k = 0; k < 8; k++)
      _lod_children.push_back(k < static_cast<int>(kids.size()) ? kids[k] : -1);
  }
}

std::pair<Point, double> PointCloud::lod_cube(int i) const {
  const double half = _lod_size[i] * 0.5;
  return {Point(_lod_min[i * 3] + half, _lod_min[i * 3 + 1] + half, _lod_min[i * 3 + 2] + half), _lod_size[i]};
}

std::vector<int> PointCloud::lod_children(int i) const {
  return std::vector<int>(_lod_children.begin() + i * 8, _lod_children.begin() + i * 8 + 8);
}

// ═══════════════════════════════════════════════════════════════════════════
// Point ids
// ═══════════════════════════════════════════════════════════════════════════

int PointCloud::index_of_id(int id) const {
  if (_point_ids.empty())
    return id >= 0 && id < static_cast<int>(point_count()) ? id : -1;
  for (size_t i = 0; i < _point_ids.size(); i++)
    if (_point_ids[i] == id)
      return static_cast<int>(i);
  return -1;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json PointCloud::jsondump() const {
  nlohmann::ordered_json data;
  data["colors"] = _colors;
  data["coords"] = _coords;
  data["guid"] = guid();
  data["lod_children"] = _lod_children;
  data["lod_count"] = _lod_count;
  data["lod_first"] = _lod_first;
  data["lod_level"] = _lod_level;
  data["lod_min"] = _lod_min;
  data["lod_size"] = _lod_size;
  data["lod_spacing"] = _lod_spacing;
  data["name"] = name;
  data["normals"] = _normals;
  data["point_ids"] = _point_ids;
  data["point_size"] = point_size;
  data["type"] = "PointCloud";
  return data;
}

PointCloud PointCloud::jsonload(const nlohmann::json &data) {
  PointCloud cloud = from_coords(
      data.value("coords", std::vector<double>{}),
      data.value("colors", std::vector<int>{}),
      data.value("normals", std::vector<double>{})
  );
  cloud.guid() = data.value("guid", cloud.guid());
  cloud.name = data.value("name", cloud.name);
  cloud.point_size = data.value("point_size", 1.0);
  cloud._lod_min = data.value("lod_min", std::vector<double>{});
  cloud._lod_size = data.value("lod_size", std::vector<double>{});
  cloud._lod_spacing = data.value("lod_spacing", std::vector<double>{});
  cloud._lod_level = data.value("lod_level", std::vector<int>{});
  cloud._lod_first = data.value("lod_first", std::vector<int>{});
  cloud._lod_count = data.value("lod_count", std::vector<int>{});
  cloud._lod_children = data.value("lod_children", std::vector<int>{});
  cloud._point_ids = data.value("point_ids", std::vector<int>{});
  return cloud;
}

std::string PointCloud::file_json_dumps() const { return jsondump().dump(); }

PointCloud PointCloud::file_json_loads(const std::string &json_string) {
  return jsonload(nlohmann::ordered_json::parse(json_string));
}

void PointCloud::file_json_dump(const std::string &filename) const {
  std::ofstream ofs(filename);
  ofs << jsondump().dump(2);
}

PointCloud PointCloud::file_json_load(const std::string &filename) {
  std::ifstream ifs(filename);
  nlohmann::json data;
  ifs >> data;
  return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

std::string PointCloud::pb_dumps() const {
  session_proto::PointCloud proto;
  if (has_guid())
    proto.set_guid(guid());
  proto.set_name(name);
  proto.set_point_size(point_size);
  for (double v : _coords)
    proto.add_coords(v);
  for (int v : _colors)
    proto.add_colors(static_cast<uint32_t>(v));
  for (double v : _normals)
    proto.add_normals(v);
  for (double v : _lod_min)
    proto.add_lod_min(v);
  for (double v : _lod_size)
    proto.add_lod_size(v);
  for (double v : _lod_spacing)
    proto.add_lod_spacing(v);
  for (int v : _lod_level)
    proto.add_lod_level(v);
  for (int v : _lod_first)
    proto.add_lod_first(v);
  for (int v : _lod_count)
    proto.add_lod_count(v);
  for (int v : _lod_children)
    proto.add_lod_children(v);
  for (int v : _point_ids)
    proto.add_point_ids(static_cast<uint32_t>(v));
  return proto.SerializeAsString();
}

PointCloud PointCloud::pb_loads(const std::string &data) {
  session_proto::PointCloud proto;
  proto.ParseFromString(data);
  PointCloud cloud;
  cloud._coords.assign(proto.coords().begin(), proto.coords().end());
  cloud._colors.assign(proto.colors().begin(), proto.colors().end());
  cloud._normals.assign(proto.normals().begin(), proto.normals().end());
  if (!proto.guid().empty())
    cloud.guid() = proto.guid();
  cloud.name = proto.name();
  if (proto.point_size() > 0.0)
    cloud.point_size = proto.point_size();
  cloud._lod_min.assign(proto.lod_min().begin(), proto.lod_min().end());
  cloud._lod_size.assign(proto.lod_size().begin(), proto.lod_size().end());
  cloud._lod_spacing.assign(proto.lod_spacing().begin(), proto.lod_spacing().end());
  cloud._lod_level.assign(proto.lod_level().begin(), proto.lod_level().end());
  cloud._lod_first.assign(proto.lod_first().begin(), proto.lod_first().end());
  cloud._lod_count.assign(proto.lod_count().begin(), proto.lod_count().end());
  cloud._lod_children.assign(proto.lod_children().begin(), proto.lod_children().end());
  cloud._point_ids.assign(proto.point_ids().begin(), proto.point_ids().end());
  return cloud;
}

void PointCloud::pb_dump(const std::string &filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  ofs << pb_dumps();
}

PointCloud PointCloud::pb_load(const std::string &filename) {
  std::ifstream ifs(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════

std::string PointCloud::str() const { return fmt::format("{} points", point_count()); }

std::string PointCloud::repr() const {
  return fmt::format("PointCloud({}, {} points, {} colors, {} normals)", name, point_count(), color_count(), normal_count());
}

std::ostream &operator<<(std::ostream &os, const PointCloud &cloud) {
  os << cloud.str();
  return os;
}

} // namespace session_cpp
