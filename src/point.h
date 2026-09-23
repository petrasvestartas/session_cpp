#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <cmath>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace session_proto {
class Point;
}

namespace session_cpp {

/// A 3D point with display width and color.
class Point {
private:
    mutable std::string _guid; // Lazily minted GUID.
    double _x = 0.0; // X coordinate.
    double _y = 0.0; // Y coordinate.
    double _z = 0.0; // Z coordinate.

public:
    std::string name = "my_point"; // Point name.
    double width = 1.0; // Display width.
    Color pointcolor = Color::black(); // Display color.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the origin.
    Point() : _x(0.0), _y(0.0), _z(0.0) {}

    /// Construct from coordinates and a name.
    Point(double x, double y, double z, std::string name = "my_point")
        : _x(x), _y(y), _z(z), name(std::move(name)) {}

    /// Copy with a new guid and the same data.
    Point(const Point& other);

    /// Copy-assign with a new guid and the same data.
    Point& operator=(const Point& other);

    /// Move while preserving the guid.
    Point(Point&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Point& operator=(Point&& other) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const;

    /// Return the mutable guid, creating it on first access.
    std::string& guid();

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the mutable coordinate by index (0=x, 1=y, 2=z).
    double& operator[](int index);

    /// Return the coordinate by index (0=x, 1=y, 2=z).
    const double& operator[](int index) const;

    /// Compare name, coordinates, width and color within rounding.
    bool operator==(const Point& other) const;

    /// Compare name, coordinates, width and color within rounding.
    bool operator!=(const Point& other) const;

    /// Scale in place.
    Point& operator*=(double factor);

    /// Divide in place.
    Point& operator/=(double factor);

    /// Translate in place.
    Point& operator+=(const Vector& other);

    /// Translate back in place.
    Point& operator-=(const Vector& other);

    /// Return a scaled copy.
    Point operator*(double factor) const;

    /// Return a divided copy.
    Point operator/(double factor) const;

    /// Return a translated copy.
    Point operator+(const Vector& other) const;

    /// Return a copy translated back.
    Point operator-(const Vector& other) const;

    /// Return the vector from other to this point.
    Vector operator-(const Point& other) const;

    /// Return the coordinate-wise sum of two points.
    static Point sum(const Point& p0, const Point& p1);

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    Point transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether a, b, c turn counter-clockwise in the xy plane.
    static bool is_ccw(const Point& a, const Point& b, const Point& c);

    /// Return the mid point between this point and p.
    Point mid_point(const Point& p) const;

    /// Return the mid point between a and b.
    static Point mid_point(const Point& a, const Point& b);

    /// Return the distance to p, scaled to stay finite for large coordinates.
    double distance(const Point& p, double double_min = 1e-12) const;

    /// Return the distance between a and b, scaled to stay finite for large coordinates.
    static double distance(const Point& a, const Point& b, double double_min = 1e-12);

    /// Return the squared distance to p, scaled to stay finite for large coordinates.
    double squared_distance(const Point& p, double double_min = 1e-12) const;

    /// Return the squared distance between a and b, scaled to stay finite for large coordinates.
    static double squared_distance(const Point& a, const Point& b, double double_min = 1e-12);

    /// Return the point at parameter t in [0, 1] between a and b.
    static Point lerp(const Point& a, const Point& b, double t);

    /// Return evenly spaced points between from and to (kind: 0=no endpoints, 1=both, 2=start only).
    static std::vector<Point> interpolate(const Point& from, const Point& to, int steps, int kind = 0);

    /// Return the shoelace area of a polygon in the xy plane.
    static double area(const std::vector<Point>& points);

    /// Return the area-weighted centroid of a quadrilateral.
    static Point centroid_quad(const std::vector<Point>& vertices);

    /// Return the arithmetic mean of points; empty input returns the origin.
    static Point centroid(const std::vector<Point>& points);

    /// Return the unsigned dihedral angle in degrees of edge pq between half-planes pqr and pqs.
    static double dihedral_angle_deg(const Point& p, const Point& q, const Point& r, const Point& s);

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Point jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Point file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Point file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Point to_proto() const;

    /// Construct from the protobuf message.
    static Point from_proto(const session_proto::Point& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Point pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Point pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "x, y, z".
    std::string str() const;

    /// Return "Point(name, x, y, z, Color(...), width)".
    std::string repr() const;
};

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const Point& point);

} // namespace session_cpp
