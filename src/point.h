#pragma once
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
#include <vector.h>

namespace session_cpp {

/**
 * @class Point
 * @brief A point defined by XYZ coordinates with display properties.
 */
class Point {
public:
  std::string guid = ::guid();       ///< Unique identifier for the point
  std::string name = "my_point";     ///< Point identifier/name
  double width = 1.0;                ///< Point diameter in pixels
  Color pointcolor = Color::blue();  ///< Color of the point (default: blue)
  Xform xform = Xform::identity();   ///< Transformation matrix

private:
  double _x = 0.0;                   ///< X coordinate (private)
  double _y = 0.0;                   ///< Y coordinate (private)
  double _z = 0.0;                   ///< Z coordinate (private)

public:

  /**
   * @brief Constructor.
   * @param x The X coordinate of the point.
   * @param y The Y coordinate of the point.
   * @param z The Z coordinate of the point.
   * @param point_name Optional name for the point (default: "my_point").
   */
  Point(double x, double y, double z, std::string point_name = "my_point")
      : name(std::move(point_name)), _x(x), _y(y), _z(z) {}
  Point() : _x(0.0), _y(0.0), _z(0.0) {}

  /// Copy constructor (creates a new guid while copying data)
  Point(const Point &other);

  /// Copy assignment (creates a new guid while copying data)
  Point &operator=(const Point &other);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Operators - const because they oinly read values, dont modify them
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert point to string representation
  std::string str() const;    ///< simple coordinate string (like Python str)
  std::string repr() const;   ///< detailed representation (like Python repr)

  /// Equality operator
  bool operator==(const Point &other) const;

  /// Inequality operator
  bool operator!=(const Point &other) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Transformation
  ///////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Apply the stored xform transformation to the point coordinates.
   * 
   * Transforms the point in-place and resets xform to identity.
   */
  void transform();

  /**
   * @brief Return a transformed copy of the point.
   * 
   * Returns a new point with the transformation applied.
   * The original point and its xform remain unchanged.
   * @return A new transformed point.
   */
  Point transformed() const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to JSON-serializable object
  nlohmann::ordered_json jsondump() const;

  /// Create point from JSON data
  static Point jsonload(const nlohmann::json &data);

  /// Write JSON to file
  void json_dump(const std::string& filename) const;

  /// Read JSON from file
  static Point json_load(const std::string& filename);

  /// Convert to JSON string
  std::string json_dumps() const;

  /// Load from JSON string
  static Point json_loads(const std::string& json_string);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Protobuf
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to protobuf binary string
  std::string pb_dumps() const;

  /// Load from protobuf binary string
  static Point pb_loads(const std::string& data);

  /// Write protobuf to file
  void pb_dump(const std::string& filename) const;

  /// Read protobuf from file
  static Point pb_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // No-copy Operators
  ///////////////////////////////////////////////////////////////////////////////////////////

  double &operator[](int index);

  const double &operator[](int index) const;

  Point &operator*=(double factor);
  
  Point &operator/=(double factor);

  Point &operator+=(const Vector &other);

  Point &operator-=(const Vector &other);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Copy Operators
  ///////////////////////////////////////////////////////////////////////////////////////////

  Point operator*(double factor) const;

  Point operator/(double factor) const;

  Point operator+(const Vector& other) const;

  Point operator-(const Vector& other) const;

  Vector operator-(const Point& other) const;

  /// Returns a new point that is the sum of two points.
  static Point sum(const Point& p0, const Point& p1);

  /// Returns a new point that is the difference of two points.
  static Point sub(const Point& p0, const Point& p1);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details
  ///////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Check if the points are in counter-clockwise order.
   * 
   * @param a First point.
   * @param b Second point.
   * @param c Third point.
   * @return True if the points are in counter-clockwise order, False otherwise.
   */
  static bool ccw(const Point& a, const Point& b, const Point& c);

  static bool is_ccw(const Point& a, const Point& b, const Point& c);

  /**
   * @brief Calculate the mid point between this point and another point.
   * 
   * @param p The other point.
   * @return The mid point between this point and the other point.
   */
  Point mid_point(const Point& p) const;

  static Point mid_point(const Point& a, const Point& b);

  /**
   * @brief Calculate the distance between this point and another point.
   * 
   * @param p The other point.
   * @param float_min The minimum value for the distance. Defaults to 1e-12.
   * @return The distance between this point and the other point.
   */
  double distance(const Point& p, double float_min = 1e-12) const;

  static double distance(const Point& a, const Point& b, double float_min = 1e-12);

  double squared_distance(const Point& p, double float_min = 1e-12) const;

  static double squared_distance(const Point& a, const Point& b, double float_min = 1e-12);

  /**
   * @brief Calculate the area of a polygon.
   * 
   * @param points The points of the polygon.
   * @return The area of the polygon.
   */
  static double area(const std::vector<Point>& points);

  /**
   * @brief Calculate the centroid of a quadrilateral.
   * 
   * @param vertices The vertices of the quadrilateral.
   * @return The centroid of the quadrilateral.
   */
  static Point centroid_quad(const std::vector<Point>& vertices);

}; // End of Point class

///////////////////////////////////////////////////////////////////////////////////////////
// Not class methods
///////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  To use this operator, you can do:
 *         Point point(1.5, 2.5, 3.5);
 *         std::cout << "Created point: " << point << std::endl;
 * @param os The output stream.
 * @param point The Point to insert into the stream.
 * @return A reference to the output stream.
 */
std::ostream &operator<<(std::ostream &os, const Point &point);

} // namespace session_cpp