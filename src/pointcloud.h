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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace session_proto {
class PointCloud;
}

namespace session_cpp {

class SpatialOctree;

/// A point cloud as flat coordinate, color and normal arrays with an optional LOD octree.
class PointCloud {
private:
    mutable std::string _guid; // Lazily minted GUID.
    std::vector<double> _coords; // Flat [x, y, z, ...].
    std::vector<int> _colors; // Flat [r, g, b, a, ...] as 0-255.
    std::vector<double> _normals; // Flat [nx, ny, nz, ...].
    std::vector<double> _lod_min; // Node cube min corner, 3 per node.
    std::vector<double> _lod_size; // Node cube edge length.
    std::vector<double> _lod_spacing; // Node grid-accept spacing.
    std::vector<int> _lod_level; // Node depth from the root.
    std::vector<int> _lod_first; // Node first point index.
    std::vector<int> _lod_count; // Node point count.
    std::vector<int> _lod_children; // Node child indices, 8 per node, -1 unused.
    std::vector<int> _point_ids; // Stable point ids parallel to the points.

public:
    std::string name = "my_pointcloud"; // Cloud name.
    double point_size = 1.0; // Display point size.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty cloud.
    PointCloud() {}

    /// Construct from points, normals and colors.
    PointCloud(const std::vector<Point>& points, const std::vector<Vector>& normals, const std::vector<Color>& colors);

    /// Copy with a new guid and the same data.
    PointCloud(const PointCloud& other);

    /// Copy-assign with a new guid and the same data.
    PointCloud& operator=(const PointCloud& other);

    /// Move while preserving the guid.
    PointCloud(PointCloud&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    PointCloud& operator=(PointCloud&& other) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const;

    /// Return the mutable guid, creating it on first access.
    std::string& guid();

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from flat arrays: coords [x, y, z, ...], colors [r, g, b, a, ...] as 0-255, normals [nx, ny, nz, ...].
    static PointCloud from_coords(const std::vector<double>& coords, const std::vector<int>& colors = {}, const std::vector<double>& normals = {});

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, arrays, LOD ranges and point ids; guid ignored.
    bool operator==(const PointCloud& other) const;

    /// Compare name, arrays, LOD ranges and point ids; guid ignored.
    bool operator!=(const PointCloud& other) const;

    /// Translate in place.
    PointCloud& operator+=(const Vector& other);

    /// Translate back in place.
    PointCloud& operator-=(const Vector& other);

    /// Return a translated copy.
    PointCloud operator+(const Vector& other) const;

    /// Return a copy translated back.
    PointCloud operator-(const Vector& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform points and normals in place.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    PointCloud transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Points
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the number of points.
    size_t point_count() const {
        return _coords.size() / 3;
    }

    /// Return the number of points.
    size_t len() const {
        return point_count();
    }

    /// Return whether the cloud has no points.
    bool is_empty() const {
        return _coords.empty();
    }

    /// Return the point at index.
    Point get_point(size_t index) const;

    /// Set the point at index.
    void set_point(size_t index, const Point& point);

    /// Append a point.
    void add_point(const Point& point);

    /// Return all points.
    std::vector<Point> get_points() const;

    /// Return the flat coordinate array itself; get_point builds a Point per call.
    const std::vector<double>& coords() const {
        return _coords;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Colors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the number of colors.
    size_t color_count() const {
        return _colors.size() / 4;
    }

    /// Return the color at index.
    Color get_color(size_t index) const;

    /// Set the color at index.
    void set_color(size_t index, const Color& color);

    /// Append a color.
    void add_color(const Color& color);

    /// Return all colors.
    std::vector<Color> get_colors() const;

    /// Return the flat 0-255 color array itself, the encoding the proto carries.
    const std::vector<int>& colors() const {
        return _colors;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Normals
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the number of normals.
    size_t normal_count() const {
        return _normals.size() / 3;
    }

    /// Return the normal at index.
    Vector get_normal(size_t index) const;

    /// Set the normal at index.
    void set_normal(size_t index, const Vector& normal);

    /// Append a normal.
    void add_normal(const Vector& normal);

    /// Return all normals.
    std::vector<Vector> get_normals() const;

    /// Return the flat normal array itself; get_normal builds a Vector per call.
    const std::vector<double>& normals() const {
        return _normals;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOD octree
    // ═══════════════════════════════════════════════════════════════════════════
    /// Build the octree and permute the arrays into octree order, so a node is one contiguous range.
    void build_lod(double root_spacing, int leaf_capacity);

    /// Return whether an octree has been built.
    bool has_lod() const {
        return !_lod_size.empty();
    }

    /// Return the number of octree nodes.
    size_t lod_node_count() const {
        return _lod_size.size();
    }

    /// Return the node cube center and edge length.
    std::pair<Point, double> lod_cube(int i) const;

    /// Return the grid-accept spacing of a node.
    double lod_spacing(int i) const {
        return _lod_spacing[i];
    }

    /// Return the node depth from the root.
    int lod_level(int i) const {
        return _lod_level[i];
    }

    /// Return the node point range as (first, count) into the reordered arrays.
    std::pair<int, int> lod_range(int i) const {
        return {_lod_first[i], _lod_count[i]};
    }

    /// Return the present child node indices compacted into 8 slots, -1 unused.
    std::vector<int> lod_children(int i) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Point ids
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the stable ids parallel to the points, minted by the first build_lod; empty before that.
    const std::vector<int>& point_ids() const {
        return _point_ids;
    }

    /// Return the stable id of the point at index; the index itself before a tree is built.
    int point_id(int index) const {
        return _point_ids.empty() ? index : _point_ids[index];
    }

    /// Return the current index of a stable id, -1 when the cloud has no such point.
    int index_of_id(int id) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static PointCloud jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static PointCloud file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static PointCloud file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::PointCloud to_proto() const;

    /// Construct from the protobuf message.
    static PointCloud from_proto(const session_proto::PointCloud& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static PointCloud pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static PointCloud pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "N points".
    std::string str() const;

    /// Return "PointCloud(name, N points, N colors, N normals)".
    std::string repr() const;

private:
    /// Permute points, ids, colors and normals into the given order.
    void lod_reorder(const std::vector<int>& order);

    /// Replace the LOD node arrays with the nodes of tree.
    void lod_store_nodes(const SpatialOctree& tree);
};

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const PointCloud& cloud);

} // namespace session_cpp
