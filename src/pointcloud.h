#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <fstream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace session_cpp {

/// A point cloud as flat coordinate, color and normal arrays with an optional LOD octree
class PointCloud {
public:
  std::string name = "my_pointcloud";
  double point_size = 1.0;

  PointCloud();
  PointCloud(const std::vector<Point> &points, const std::vector<Vector> &normals, const std::vector<Color> &colors);

  /// Copy constructor (new guid, same data)
  PointCloud(const PointCloud &other);

  /// Copy assignment (new guid, same data)
  PointCloud &operator=(const PointCloud &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  PointCloud(PointCloud &&other) noexcept = default;
  PointCloud &operator=(PointCloud &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Clear the guid so a fresh one mints lazily on next read
  void refresh_guid() { _guid.clear(); }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Cloud from flat arrays: coords [x, y, z, ...], colors [r, g, b, a, ...] as 0-255, normals [nx, ny, nz, ...]
  static PointCloud from_coords(const std::vector<double> &coords, const std::vector<int> &colors = {}, const std::vector<double> &normals = {});

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Same name, arrays, LOD ranges and point ids; guid ignored
  bool operator==(const PointCloud &other) const;
  bool operator!=(const PointCloud &other) const;

  PointCloud &operator+=(const Vector &other);
  PointCloud &operator-=(const Vector &other);

  PointCloud operator+(const Vector &other) const;
  PointCloud operator-(const Vector &other) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform points and normals in place
  void transform(const Xform &xform);

  /// Transformed copy
  PointCloud transformed(const Xform &xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Points
  // ═══════════════════════════════════════════════════════════════════════════

  size_t point_count() const { return _coords.size() / 3; }
  size_t len() const { return point_count(); }
  bool is_empty() const { return _coords.empty(); }
  Point get_point(size_t index) const;
  void set_point(size_t index, const Point &point);
  void add_point(const Point &point);
  std::vector<Point> get_points() const;

  /// The flat coordinate array itself; get_point builds a Point per call
  const std::vector<double> &coords() const { return _coords; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Colors
  // ═══════════════════════════════════════════════════════════════════════════

  size_t color_count() const { return _colors.size() / 4; }
  Color get_color(size_t index) const;
  void set_color(size_t index, const Color &color);
  void add_color(const Color &color);
  std::vector<Color> get_colors() const;

  /// The flat 0-255 color array itself, the encoding the proto carries
  const std::vector<int> &colors() const { return _colors; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Normals
  // ═══════════════════════════════════════════════════════════════════════════

  size_t normal_count() const { return _normals.size() / 3; }
  Vector get_normal(size_t index) const;
  void set_normal(size_t index, const Vector &normal);
  void add_normal(const Vector &normal);
  std::vector<Vector> get_normals() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // LOD octree
  // ═══════════════════════════════════════════════════════════════════════════

  /// Build the octree and permute the arrays into octree order, so a node is one contiguous range
  void build_lod(double root_spacing, int leaf_capacity);

  bool has_lod() const { return !_lod_size.empty(); }
  size_t lod_node_count() const { return _lod_size.size(); }

  /// Node cube center and edge length
  std::pair<Point, double> lod_cube(int i) const;

  /// Grid-accept spacing of a node
  double lod_spacing(int i) const { return _lod_spacing[i]; }

  /// Node depth from the root
  int lod_level(int i) const { return _lod_level[i]; }

  /// Node point range as (first, count) into the reordered arrays
  std::pair<int, int> lod_range(int i) const { return {_lod_first[i], _lod_count[i]}; }

  /// Present child node indices compacted into 8 slots, -1 unused
  std::vector<int> lod_children(int i) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Point ids
  // ═══════════════════════════════════════════════════════════════════════════

  /// Stable ids parallel to the points, minted by the first build_lod; empty before that
  const std::vector<int> &point_ids() const { return _point_ids; }

  /// Stable id of the point at index; the index itself before a tree is built
  int point_id(int index) const { return _point_ids.empty() ? index : _point_ids[index]; }

  /// Current index of a stable id, -1 when the cloud has no such point
  int index_of_id(int id) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static PointCloud jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static PointCloud file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static PointCloud file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static PointCloud pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static PointCloud pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "N points"
  std::string str() const;

  /// "PointCloud(name, N points, N colors, N normals)"
  std::string repr() const;

private:
  mutable std::string _guid;
  std::vector<double> _coords;
  std::vector<int> _colors;
  std::vector<double> _normals;
  std::vector<double> _lod_min;
  std::vector<double> _lod_size;
  std::vector<double> _lod_spacing;
  std::vector<int> _lod_level;
  std::vector<int> _lod_first;
  std::vector<int> _lod_count;
  std::vector<int> _lod_children;
  std::vector<int> _point_ids;
};

std::ostream &operator<<(std::ostream &os, const PointCloud &cloud);

} // namespace session_cpp
