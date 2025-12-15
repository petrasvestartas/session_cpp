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
 *
 * Internally stores data as flat arrays for efficient serialization:
 * - _coords: [x0, y0, z0, x1, y1, z1, ...]
 * - _colors: [r0, g0, b0, a0, r1, g1, b1, a1, ...]
 * - _normals: [nx0, ny0, nz0, nx1, ny1, nz1, ...]
 */
class PointCloud {
public:
    std::string guid = ::guid();
    std::string name = "my_pointcloud";
    double point_size = 1.0;
    Xform xform;

private:
    std::vector<double> _coords;
    std::vector<int> _colors;
    std::vector<double> _normals;

public:
    /**
     * @brief Default constructor.
     */
    PointCloud();

    /**
     * @brief Constructor with points, normals, and colors.
     */
    PointCloud(const std::vector<Point>& points,
               const std::vector<Vector>& normals,
               const std::vector<Color>& colors);

    /**
     * @brief Copy constructor (creates new guid).
     */
    PointCloud(const PointCloud& other);

    /**
     * @brief Assignment operator (creates new guid).
     */
    PointCloud& operator=(const PointCloud& other);

    /**
     * @brief Create from flat arrays.
     */
    static PointCloud from_coords(const std::vector<double>& coords,
                                  const std::vector<int>& colors = {},
                                  const std::vector<double>& normals = {});

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Point Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    size_t point_count() const { return _coords.size() / 3; }
    size_t len() const { return point_count(); }
    bool is_empty() const { return _coords.empty(); }

    Point get_point(size_t index) const;
    void set_point(size_t index, const Point& point);
    void add_point(const Point& point);
    std::vector<Point> get_points() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Color Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    size_t color_count() const { return _colors.size() / 4; }

    Color get_color(size_t index) const;
    void set_color(size_t index, const Color& color);
    void add_color(const Color& color);
    std::vector<Color> get_colors() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Normal Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    size_t normal_count() const { return _normals.size() / 3; }

    Vector get_normal(size_t index) const;
    void set_normal(size_t index, const Vector& normal);
    void add_normal(const Vector& normal);
    std::vector<Vector> get_normals() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representations
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string str() const;
    std::string repr() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Duplicate and Equality
    ///////////////////////////////////////////////////////////////////////////////////////////

    PointCloud duplicate() const;
    bool operator==(const PointCloud& other) const;
    bool operator!=(const PointCloud& other) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    void transform();
    PointCloud transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // No-copy Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    PointCloud& operator+=(const Vector& v);
    PointCloud& operator-=(const Vector& v);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Copy Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    PointCloud operator+(const Vector& v) const;
    PointCloud operator-(const Vector& v) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    nlohmann::ordered_json jsondump() const;
    static PointCloud jsonload(const nlohmann::json& data);
    void json_dump(const std::string& filename) const;
    static PointCloud json_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string to_protobuf() const;
    static PointCloud from_protobuf(const std::string& data);
    void protobuf_dump(const std::string& filename) const;
    static PointCloud protobuf_load(const std::string& filename);
};

/// Stream output operator for point cloud
std::ostream& operator<<(std::ostream& os, const PointCloud& cloud);

} // namespace session_cpp
