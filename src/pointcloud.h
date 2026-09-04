#pragma once

#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "json.h"
#include <vector>
#include <string>

namespace session_cpp {

/**
 * @class PointCloud
 * @brief A point cloud with coordinates, normals, and colors stored as flat arrays.
 */
class PointCloud {
public:
    /// Lazy GUID accessor (const)
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Lazy GUID accessor (mutable)
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Human-readable name
    std::string name = "my_pointcloud";

    /// Point size for rendering
    double point_size = 1.0;


private:
    mutable std::string _guid; ///< Lazily generated unique identifier
    std::vector<double> _coords;  ///< Flat coords [x0, y0, z0, x1, y1, z1, ...]
    std::vector<int> _colors;     ///< Flat colors [r0, g0, b0, a0, ...]
    std::vector<double> _normals; ///< Flat normals [nx0, ny0, nz0, ...]
    /// LOD octree over the points, one flat array per SpatialOctree node field. Built by
    /// build_lod(), which PERMUTES the three arrays above into octree order - so a node is one
    /// contiguous (_lod_first, _lod_count) range and the order permutation never has to be
    /// stored. Empty means no octree.
    std::vector<double> _lod_min;    ///< node cube minimum, 3 per node
    std::vector<double> _lod_size;   ///< node cube edge, 1 per node
    std::vector<double> _lod_spacing;///< grid-accept spacing, 1 per node
    std::vector<int> _lod_level;     ///< depth from the root, 1 per node
    std::vector<int> _lod_first;     ///< first point row of the node, 1 per node
    std::vector<int> _lod_count;     ///< points in the node, 1 per node
    std::vector<int> _lod_children;  ///< present children compacted into 8 slots, -1 = unused
    /// STABLE per-point ids, one per point, parallel to _coords. Assigned once by the first
    /// build_lod and permuted with the points ever after, so an index that moves does not take
    /// the point's identity with it. Empty = no tree yet, so the index IS the id.
    std::vector<int> _point_ids;

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Default constructor (empty cloud)
    PointCloud();

    /// Constructor with points, normals, and colors
    PointCloud(const std::vector<Point>& points,
               const std::vector<Vector>& normals,
               const std::vector<Color>& colors);

    /// Copy constructor (creates a new guid while copying data)
    PointCloud(const PointCloud& other);

    /// Move constructor and assignment: identity SURVIVES a move.
    ///
    /// A copy is a new object and mints a new guid; a move is the SAME object in a new place, so
    /// `_guid` transfers. Declaring these is also what keeps `return x;` safe: the copy below is
    /// user-declared, which suppresses the implicit move, so a `pb_loads` that missed NRVO fell back
    /// to the COPY and silently dropped the guid it had just deserialized - every other field
    /// survived, so the object looked right and only its identity was wrong. That is what broke Line
    /// on MSVC when its pb_loads changed shape; see line.h.
    PointCloud(PointCloud&& other) noexcept = default;
    PointCloud& operator=(PointCloud&& other) noexcept = default;

    /// Copy assignment (creates a new guid while copying data)
    PointCloud& operator=(const PointCloud& other);

    /// Create from flat arrays of coords, colors, and normals
    static PointCloud from_coords(const std::vector<double>& coords,
                                  const std::vector<int>& colors = {},
                                  const std::vector<double>& normals = {});

    // ═══════════════════════════════════════════════════════════════════════════
    // Point Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Number of points
    size_t point_count() const { return _coords.size() / 3; }

    /// Alias for point_count()
    size_t len() const { return point_count(); }

    /// True when the cloud has no points
    bool is_empty() const { return _coords.empty(); }

    /// Get point at index
    Point get_point(size_t index) const;

    /// Set point at index
    void set_point(size_t index, const Point& point);

    /// Append a point to the cloud
    void add_point(const Point& point);

    /// Get all points as a vector
    /// The flat coordinate array itself, [x0, y0, z0, x1, ...]. A renderer walking millions of
    /// points cannot afford get_point per point: that builds a Point, which owns a name and a
    /// colour, so a large scan spends most of its walk in the allocator.
    const std::vector<double>& coords() const { return _coords; }

    std::vector<Point> get_points() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Color Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Number of colors
    size_t color_count() const { return _colors.size() / 4; }

    /// Get color at index
    Color get_color(size_t index) const;

    /// Set color at index
    void set_color(size_t index, const Color& color);

    /// Append a color to the cloud
    void add_color(const Color& color);

    /// Get all colors as a vector
    /// The flat colour array itself, [r0, g0, b0, a0, r1, ...] as 0-255 - the same encoding the
    /// proto carries. Same reason as coords(): get_color builds a Color, which owns a name.
    const std::vector<int>& colors() const { return _colors; }

    std::vector<Color> get_colors() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Normal Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Number of normals
    size_t normal_count() const { return _normals.size() / 3; }

    /// Get normal at index
    Vector get_normal(size_t index) const;

    /// Set normal at index
    void set_normal(size_t index, const Vector& normal);

    /// Append a normal to the cloud
    void add_normal(const Vector& normal);

    /// Get all normals as a vector
    std::vector<Vector> get_normals() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // LOD Octree
    // ═══════════════════════════════════════════════════════════════════════════

    /// Build the LOD octree and REORDER the points into octree order. Every point's index
    /// changes; a node becomes one contiguous range. Expensive - about 10 s on 14 M points -
    /// so it is called once by whoever writes the cloud, never per construction.
    void build_lod(double root_spacing, int leaf_capacity);

    /// True when an octree has been built
    bool has_lod() const { return !_lod_size.empty(); }

    /// Number of octree nodes
    size_t lod_node_count() const { return _lod_size.size(); }

    /// Node cube: minimum corner and edge length
    std::pair<Point, double> lod_cube(int i) const;

    /// Grid-accept spacing of a node
    double lod_spacing(int i) const { return _lod_spacing[i]; }

    /// Node depth from the root
    int lod_level(int i) const { return _lod_level[i]; }

    /// Node point range as (first, count) into the reordered arrays
    std::pair<int, int> lod_range(int i) const { return {_lod_first[i], _lod_count[i]}; }

    /// Present child node indices, compacted, -1 padding
    std::vector<int> lod_children(int i) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Stable Point Ids
    // ═══════════════════════════════════════════════════════════════════════════

    /// The stable ids, parallel to the points. Empty until a tree is built.
    const std::vector<int>& point_ids() const { return _point_ids; }

    /// The stable id of a point, by its CURRENT index. Falls back to the index itself while no
    /// tree has been built, which is exactly what the id would have been.
    int point_id(int index) const { return _point_ids.empty() ? index : _point_ids[index]; }

    /// Where a stable id lives NOW, or -1 if this cloud has no such point. Linear: a caller
    /// resolving many ids should build its own map.
    int index_of_id(int id) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // String Representations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Simple string form (like Python __str__)
    std::string str() const;

    /// Detailed representation (like Python __repr__)
    std::string repr() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Equality
    // ═══════════════════════════════════════════════════════════════════════════

    /// Equality / inequality
    bool operator==(const PointCloud& other) const;
    bool operator!=(const PointCloud& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Apply a transformation to this cloud in place
    void transform(const Xform& xform);

    /// Return a copy of this cloud with the transformation applied
    PointCloud transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // No-copy Operators
    // ═══════════════════════════════════════════════════════════════════════════

    /// In-place translation by vector
    PointCloud& operator+=(const Vector& v);

    /// In-place translation by negative vector
    PointCloud& operator-=(const Vector& v);

    // ═══════════════════════════════════════════════════════════════════════════
    // Copy Operators
    // ═══════════════════════════════════════════════════════════════════════════

    /// Translation by vector (returns new cloud)
    PointCloud operator+(const Vector& v) const;

    /// Translation by negative vector (returns new cloud)
    PointCloud operator-(const Vector& v) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to ordered JSON object
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from JSON object
    static PointCloud jsonload(const nlohmann::json& data);

    /// Convert to JSON string
    std::string file_json_dumps() const;

    /// Load from JSON string
    static PointCloud file_json_loads(const std::string& json_string);

    /// Write JSON to file
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from file
    static PointCloud file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static PointCloud pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static PointCloud pb_load(const std::string& filename);
};

/// Stream output operator for point cloud
std::ostream& operator<<(std::ostream& os, const PointCloud& cloud);

} // namespace session_cpp
