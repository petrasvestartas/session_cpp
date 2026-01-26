#pragma once
#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace session_cpp {

/**
 * @class Line
 * @brief A class representing a line in 3D space with display properties.
 */
class Line {
public:
    std::string guid = ::guid();       ///< Unique identifier for the line
    std::string name = "my_line";      ///< Line identifier/name
    double width = 1.0;                ///< Line width in pixels
    Color linecolor = Color::white();  ///< Color of the line
    Xform xform;   ///< Transformation matrix

private:
    double _x0 = 0.0;                  ///< X coordinate of start point
    double _y0 = 0.0;                  ///< Y coordinate of start point
    double _z0 = 0.0;                  ///< Z coordinate of start point
    double _x1 = 0.0;                  ///< X coordinate of end point
    double _y1 = 0.0;                  ///< Y coordinate of end point
    double _z1 = 1.0;                  ///< Z coordinate of end point

public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor.
    Line();

    /// Constructor with coordinates.
    Line(double x0, double y0, double z0, double x1, double y1, double z1);

    /// Constructor from two points.
    static Line from_points(const Point& p1, const Point& p2);

    /// Fit a line to a set of points using least squares (PCA).
    static Line fit_points(const std::vector<Point>& points, double length = 0.0);

    /// Constructor with name and coordinates.
    static Line with_name(const std::string& name, double x0, double y0, double z0, double x1, double y1, double z1);

    /// Copy constructor (creates a new guid while copying data)
    Line(const Line& other);

    /// Copy assignment (creates a new guid while copying data)
    Line& operator=(const Line& other);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometry Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Calculates the length of the line.
    double length() const;

    /// Calculates the squared length of the line.
    double squared_length() const;

    /// Convert line to vector from start to end.
    Vector to_vector() const;

    /// Convert line to unit direction vector.
    Vector to_direction() const;

    /// Calculates the point at a given parameter t.
    Point point_at(double t) const;

    /// Subdivide line into n points.
    std::vector<Point> subdivide(int n) const;

    /// Subdivide line by approximate distance between points.
    std::vector<Point> subdivide_by_distance(double distance) const;

    /// Get start point.
    Point start() const;

    /// Get end point.
    Point end() const;

    /// Get center point (average of start and end).
    Point center() const;

    /// Find the closest point on the line to a given point.
    Point closest_point(const Point& point) const;

    /// Create a line from a point and a vector.
    static Line from_point_and_vector(const Point& point, const Vector& vector);

    /// Create a line from a point, direction, and length.
    static Line from_point_direction_length(const Point& point, const Vector& direction, double length);

    /// Calculate middle line between two line segments.
    static void get_middle_line(const Point& line0_start, const Point& line0_end,
                               const Point& line1_start, const Point& line1_end,
                               Point& output_start, Point& output_end);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Subscript operator for non-const access (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
    double& operator[](int index);

    /// Subscript operator for const access.
    const double& operator[](int index) const;

    Line& operator+=(const Vector& other);
    Line& operator-=(const Vector& other);
    Line& operator*=(double factor);
    Line& operator/=(double factor);

    Line operator+(const Vector& other) const;
    Line operator-(const Vector& other) const;
    Line operator*(double factor) const;
    Line operator/(double factor) const;
    Line operator-() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string str() const;    ///< simple coordinate string (like Python str)
    std::string repr() const;   ///< detailed representation (like Python repr)

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply the stored xform transformation to the line coordinates (in-place).
    void transform();

    /// Return a transformed copy of the line.
    Line transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON-serializable object.
    nlohmann::ordered_json jsondump() const;

    /// Create line from JSON data.
    static Line jsonload(const nlohmann::json& data);

    /// Serialize to JSON file.
    void json_dump(const std::string& filename) const;

    /// Deserialize from JSON file.
    static Line json_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf string.
    std::string to_protobuf() const;

    /// Create line from protobuf data.
    static Line from_protobuf(const std::string& data);

    /// Serialize to protobuf file.
    void protobuf_dump(const std::string& filename) const;

    /// Deserialize from protobuf file.
    static Line protobuf_load(const std::string& filename);
};

/// Output stream operator for Line.
std::ostream& operator<<(std::ostream& os, const Line& line);

}  // namespace session_cpp

// fmt formatter specialization for Line
template <> struct fmt::formatter<session_cpp::Line> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  auto format(const session_cpp::Line& o, fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
