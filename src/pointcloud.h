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

    /// Transformation matrix applied by transform()/transformed()
    Xform xform;

private:
    mutable std::string _guid; ///< Lazily generated unique identifier
    std::vector<double> _coords;  ///< Flat coords [x0, y0, z0, x1, y1, z1, ...]
    std::vector<int> _colors;     ///< Flat colors [r0, g0, b0, a0, ...]
    std::vector<double> _normals; ///< Flat normals [nx0, ny0, nz0, ...]

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor (empty cloud)
    PointCloud();

    /// Constructor with points, normals, and colors
    PointCloud(const std::vector<Point>& points,
               const std::vector<Vector>& normals,
               const std::vector<Color>& colors);

    /// Copy constructor (creates a new guid while copying data)
    PointCloud(const PointCloud& other);

    /// Copy assignment (creates a new guid while copying data)
    PointCloud& operator=(const PointCloud& other);

    /// Create from flat arrays of coords, colors, and normals
    static PointCloud from_coords(const std::vector<double>& coords,
                                  const std::vector<int>& colors = {},
                                  const std::vector<double>& normals = {});

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Point Access
    ///////////////////////////////////////////////////////////////////////////////////////////

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
    std::vector<Point> get_points() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Color Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Number of colors
    size_t color_count() const { return _colors.size() / 4; }

    /// Get color at index
    Color get_color(size_t index) const;

    /// Set color at index
    void set_color(size_t index, const Color& color);

    /// Append a color to the cloud
    void add_color(const Color& color);

    /// Get all colors as a vector
    std::vector<Color> get_colors() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Normal Access
    ///////////////////////////////////////////////////////////////////////////////////////////

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

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Simple string form (like Python __str__)
    std::string str() const;

    /// Detailed representation (like Python __repr__)
    std::string repr() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Equality
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Equality / inequality
    bool operator==(const PointCloud& other) const;
    bool operator!=(const PointCloud& other) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply this cloud's xform in place
    void transform();

    /// Return a copy of this cloud with its xform applied
    PointCloud transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // No-copy Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// In-place translation by vector
    PointCloud& operator+=(const Vector& v);

    /// In-place translation by negative vector
    PointCloud& operator-=(const Vector& v);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Copy Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Translation by vector (returns new cloud)
    PointCloud operator+(const Vector& v) const;

    /// Translation by negative vector (returns new cloud)
    PointCloud operator-(const Vector& v) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to ordered JSON object
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from JSON object
    static PointCloud jsonload(const nlohmann::json& data);

    /// Convert to JSON string
    std::string json_dumps() const;

    /// Load from JSON string
    static PointCloud json_loads(const std::string& json_string);

    /// Write JSON to file
    void json_dump(const std::string& filename) const;

    /// Read JSON from file
    static PointCloud json_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

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
