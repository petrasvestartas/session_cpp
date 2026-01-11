#pragma once
#include "color.h"
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <array>
#include <cmath>
#include <tuple>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "tolerance.h"

namespace session_cpp {

class Point; // Forward declaration
/**
 * @class Vector
 * @brief A 3D vector with visual properties and JSON serialization support.
 * 
 * The Vector class represents a mathematical vector in 3D space with x, y, z
 * components. It includes caching for magnitude calculations and supports various
 * geometric operations like dot product, cross product, normalization, and
 * angle calculations.
 */
class Vector {
public:
  std::string guid = ::guid();    ///< Unique identifier for the vector
  std::string name = "my_vector"; ///< Vector identifier/name

private:
  double _x = 0.0;                   ///< X coordinate (access via [0])
  double _y = 0.0;                   ///< Y coordinate (access via [1])
  double _z = 0.0;                   ///< Z coordinate (access via [2])
  mutable double _magnitude = 0.0;      ///< Cached magnitude value
  mutable bool _has_magnitude = false;  ///< Cache validity flag

  /// Invalidates the cached magnitude when coordinates change
  void invalidate_magnitude_cache() { _has_magnitude = false; }

  /// Gets cached magnitude, computing if necessary (internal only)
  double cached_magnitude() const;

  /// Computes the magnitude without caching (internal only)
  double compute_magnitude() const;

public:
  /**
   * @brief Constructor.
   * @param x The X coordinate of the vector.
   * @param y The Y coordinate of the vector.
   * @param z The Z coordinate of the vector.
   */
  Vector(double x, double y, double z) : _x(x), _y(y), _z(z) {}
  Vector() : _x(0.0), _y(0.0), _z(0.0) {}

  /// Copy constructor - creates new GUID for the copy
  Vector(const Vector &other)
      : guid(::guid()), name(other.name), _x(other._x), _y(other._y), _z(other._z),
        _magnitude(other._magnitude), _has_magnitude(other._has_magnitude) {}

  /// Copy assignment operator - creates new GUID for the copy
  Vector &operator=(const Vector &other) {
    if (this != &other) {
      guid = ::guid();
      name = other.name;
      _x = other._x;
      _y = other._y;
      _z = other._z;
      _magnitude = other._magnitude;
      _has_magnitude = other._has_magnitude;
    }
    return *this;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Operators
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert vector to string representation
  std::string to_string() const;

  /// Simple string form (like Python __str__): just coordinates
  std::string str() const;

  /// Detailed representation (like Python __repr__)
  std::string repr();

  /// Equality operator
  bool operator==(const Vector &other) const;

  /// Inequality operator
  bool operator!=(const Vector &other) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // No-copy Operators (Index Access)
  ///////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Access coordinate by index (0=x, 1=y, 2=z).
   * @param index Index of coordinate (0, 1, or 2).
   * @return Reference to the coordinate value.
   * @note Invalidates magnitude cache when used for modification.
   */
  double &operator[](int index);

  /**
   * @brief Access coordinate by index (0=x, 1=y, 2=z) - const version.
   * @param index Index of coordinate (0, 1, or 2).
   * @return Const reference to the coordinate value.
   */
  const double &operator[](int index) const;

  Vector &operator*=(double factor);
  Vector &operator/=(double factor);
  Vector &operator+=(const Vector &other);
  Vector &operator-=(const Vector &other);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Copy Operators
  ///////////////////////////////////////////////////////////////////////////////////////////

  Vector operator*(double factor) const;
  Vector operator/(double factor) const;
  Vector operator+(const Vector &other) const;
  Vector operator-(const Vector &other) const;
  friend Vector operator*(double factor, const Vector &v);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to JSON-serializable object
  nlohmann::ordered_json jsondump() const;

  /// Create vector from JSON data
  static Vector jsonload(const nlohmann::json &data);

  /// Serialize to JSON file.
  ///
  /// Parameters
  /// ----------
  /// filename : const std::string&
  ///     Path to the output file.
  void json_dump(const std::string& filename) const;

  /// Deserialize from JSON file.
  ///
  /// Parameters
  /// ----------
  /// filename : const std::string&
  ///     Path to the JSON file.
  ///
  /// Returns
  /// -------
  /// Vector
  ///     The deserialized Vector.
  static Vector json_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Protobuf Serialization
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to protobuf binary format.
  ///
  /// Returns
  /// -------
  /// std::string
  ///     Serialized protobuf data.
  std::string to_protobuf() const;

  /// Create Vector from protobuf binary data.
  ///
  /// Parameters
  /// ----------
  /// data : const std::string&
  ///     Protobuf-encoded vector data.
  ///
  /// Returns
  /// -------
  /// Vector
  ///     The deserialized Vector.
  static Vector from_protobuf(const std::string& data);

  /// Write protobuf to file.
  ///
  /// Parameters
  /// ----------
  /// filename : const std::string&
  ///     Path to the output file.
  void protobuf_dump(const std::string& filename) const;

  /// Read protobuf from file.
  ///
  /// Parameters
  /// ----------
  /// filename : const std::string&
  ///     Path to the protobuf file.
  ///
  /// Returns
  /// -------
  /// Vector
  ///     The deserialized Vector.
  static Vector protobuf_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Static Methods
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Get a zero vector (0, 0, 0).
  /// 
  /// Returns
  /// -------
  /// Vector
  ///     Zero vector (0, 0, 0).
  static Vector zero();

  /// Get unit vector along the x-axis.
  /// 
  /// Returns
  /// -------
  /// Vector
  ///     Unit vector (1, 0, 0).
  static Vector x_axis();
  
  /// Get unit vector along the y-axis.
  /// 
  /// Returns
  /// -------
  /// Vector
  ///     Unit vector (0, 1, 0).
  static Vector y_axis();
  
  /// Get unit vector along the z-axis.
  /// 
  /// Returns
  /// -------
  /// Vector
  ///     Unit vector (0, 0, 1).
  static Vector z_axis();

  /// Get vector from two points (p1 - p0).
  ///
  /// Parameters
  /// ----------
  /// p0 : const Point&
  ///     Start point.
  /// p1 : const Point&
  ///     End point.
  ///
  /// Returns
  /// -------
  /// Vector
  static Vector from_points(const Point &p0, const Point &p1);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details / Geometry
  ///////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Reverse the vector (negate all components).
   */
  void reverse();

  /// Returns the (cached) magnitude. Does not rescale.
  ///
  /// Returns
  /// -------
  /// double
  ///     The magnitude of the vector.
  double magnitude() const;

  /// Get the squared magnitude of the vector (avoids sqrt for performance).
  ///
  /// Returns
  /// -------
  /// double
  ///     The squared magnitude of the vector.
  double magnitude_squared() const;

  /// Normalize the vector in place (make it unit magnitude).
  ///
  /// Returns
  /// -------
  /// bool
  ///     True if successful, false if vector has zero magnitude.
  bool normalize_self();

  /// Return a normalized copy of the vector.
  ///
  /// Returns
  /// -------
  /// Vector
  ///     A new Vector with unit length pointing in the same direction.
  Vector normalize() const;

  /// Project this vector onto `projection_vector`.
  /// Returns: (projection_vector, projected_length, perpendicular_vector, perpendicular_length)
  std::tuple<Vector, double, Vector, double>
  projection(Vector &projection_vector,
             double tolerance = static_cast<double>(session_cpp::Tolerance::ZERO_TOLERANCE));

  /// Check if this vector is parallel/antiparallel to another.
  ///
  /// Parameters
  /// ----------
  /// other : const Vector&
  ///     Other vector.
  ///
  /// Returns
  /// -------
  /// int
  ///     1 for parallel, -1 for antiparallel, 0 for not parallel.
  int is_parallel_to(const Vector &other);
  
  /// Calculate dot product with another vector.
  ///
  /// Parameters
  /// ----------
  /// other : const Vector&
  ///     Other vector.
  ///
  /// Returns
  /// -------
  /// double
  ///     Dot product value.
  double dot(const Vector &other) const;
  
  /// Calculate cross product with another vector.
  ///
  /// Parameters
  /// ----------
  /// other : const Vector&
  ///     Other vector.
  ///
  /// Returns
  /// -------
  /// Vector
  ///     Cross product vector (orthogonal to inputs).
  Vector cross(const Vector &other) const;
  
  /// Angle between this vector and another.
  ///
  /// Parameters
  /// ----------
  /// other : const Vector&
  ///     Other vector.
  /// sign_by_cross_product : bool
  ///     Whether to use cross product for sign determination.
  /// degrees : bool
  ///     Return angle in degrees if true, radians if false.
  /// tolerance : double
  ///     Tolerance for zero-length vectors.
  ///
  /// Returns
  /// -------
  /// double
  ///     Angle between vectors.
  double angle(const Vector &other, bool sign_by_cross_product = true,
               bool degrees = true,
               double tolerance = static_cast<double>(session_cpp::Tolerance::ZERO_TOLERANCE));
  /**
   * @brief Get a copy scaled by a vertical height along the Z-axis.
   * @param vertical_height Target vertical height.
   * @return Scaled copy.
   */
  Vector get_leveled_vector(double &vertical_height);

  /**
   * @brief Calculate third side of triangle using the cosine law.
   * @param triangle_edge_length_a Length of side a.
   * @param triangle_edge_length_b Length of side b.
   * @param angle_in_between_edges Angle between a and b.
   * @param degrees If true, the angle is provided in degrees.
   * @return Length of the third side.
   */
  static double cosine_law(double &triangle_edge_length_a,
                           double &triangle_edge_length_b,
                           double &angle_in_between_edges, bool degrees = true);

  /**
   * @brief Calculate angle using the sine law.
   * @param triangle_edge_length_a Length of side a.
   * @param angle_in_front_of_a Angle opposite to side a.
   * @param triangle_edge_length_b Length of side b.
   * @param degrees If true, return angle in degrees.
   * @return Angle opposite to side b.
   */
  static double sine_law_angle(double &triangle_edge_length_a,
                               double &angle_in_front_of_a,
                               double &triangle_edge_length_b,
                               bool degrees = true);

  /**
   * @brief Calculate side length using the sine law.
   * @param triangle_edge_length_a Length of side a.
   * @param angle_in_front_of_a Angle opposite to side a.
   * @param angle_in_front_of_b Angle opposite to side b.
   * @param degrees If true, angles are provided in degrees.
   * @return Length of side b.
   */
  static double sine_law_length(double &triangle_edge_length_a,
                                double &angle_in_front_of_a,
                                double &angle_in_front_of_b,
                                bool degrees = true);

  /**
   * @brief Calculate angle opposite to side c using the cosine law.
   * @param triangle_edge_length_a Length of side a (adjacent to angle C).
   * @param triangle_edge_length_b Length of side b (adjacent to angle C).
   * @param triangle_edge_length_c Length of side c (opposite to angle C).
   * @param degrees If true, return degrees; otherwise radians.
   * @return Angle opposite to side c.
   */
  static double angle_from_cosine_law(double triangle_edge_length_a,
                                      double triangle_edge_length_b,
                                      double triangle_edge_length_c,
                                      bool degrees = true);

  /**
   * @brief Calculate side length using the sine law (given two angles).
   * @param angle_in_front_of_result_side Angle opposite to the side we want.
   * @param angle_in_front_of_known_side Angle opposite to the known side.
   * @param known_side_length Length of the known side.
   * @param degrees If true, angles are in degrees; otherwise radians.
   * @return Length of the side opposite to the first angle.
   */
  static double side_from_sine_law(double angle_in_front_of_result_side,
                                   double angle_in_front_of_known_side,
                                   double known_side_length,
                                   bool degrees = true);

  /// Angle between XY components in degrees.
  static double angle_between_vector_xy_components(Vector &vector);

  /**
   * @brief Sum a list of vectors (component-wise).
   * @param vectors Vectors to sum.
   * @return The component-wise sum.
   */
  static Vector sum_of_vectors(std::vector<Vector> &vectors);

  /**
   * @brief Compute the average of a list of vectors.
   * @param vectors Vectors to average.
   * @return The component-wise average, or zero vector if empty.
   */
  static Vector average(std::vector<Vector> &vectors);

  /**
   * @brief Check if this vector is perpendicular to another.
   * @param other The other vector.
   * @return True if perpendicular (dot product within tolerance of zero).
   */
  bool is_perpendicular_to(const Vector &other) const;

  /**
   * @brief Check if this vector is a zero vector.
   * @return True if length is within tolerance of zero.
   */
  bool is_zero() const;

  /**
   * @brief Compute coordinate direction angles (alpha, beta, gamma).
   * @param degrees Return angles in degrees if true, radians if false.
   * @return Array [alpha, beta, gamma].
   */
  std::array<double, 3> coordinate_direction_3angles(bool degrees = false);

  /**
   * @brief Compute coordinate direction angles (phi, theta).
   * @param degrees Return angles in degrees if true, radians if false.
   * @return Array [phi, theta].
   */
  std::array<double, 2> coordinate_direction_2angles(bool degrees = false);

  /**
   * @brief Set this vector to be perpendicular to `v`.
   * @param v Reference vector.
   * @return True on success, false otherwise.
   */
  bool perpendicular_to(Vector &v);

  void scale(double factor);
  void scale_up();
  void scale_down();

  /// Reflect this vector through a plane with the given normal.
  ///
  /// Parameters
  /// ----------
  /// plane_normal : const Vector&
  ///     Normal vector of the reflection plane (should be unit length).
  ///
  /// Returns
  /// -------
  /// Vector
  ///     The reflected vector: V - 2*(V·N)*N
  Vector reflect(const Vector& plane_normal) const;


}; // End of Vector class

/**
 * @brief  To use this operator, you can do:
 *         Vector vector(1.5, 2.5, 3.5);
 *         std::cout << "Created vector: " << vector << std::endl;
 * @param os The output stream.
 * @param vector The Vector to insert into the stream.
 * @return A reference to the output stream.
 */
std::ostream &operator<<(std::ostream &os, const Vector &vector);

} // namespace session_cpp

// fmt formatter specialization for Vector - enables direct fmt::print(vector)
template <> struct fmt::formatter<session_cpp::Vector> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Vector &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.to_string());
  }
};
