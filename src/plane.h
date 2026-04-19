#pragma once
#include "color.h"
#include "xform.h"
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
#include <point.h>

namespace session_cpp {

    class Polyline; // forward declaration

    /**
     * @class Plane
     * @brief A plane defined by origina and x-axis, z-axis, y-axis.
     */
    class Plane {
        public:
        std::string name = "my_plane";     ///< Plane identifier/name
        const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
        std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
        double width = 1.0;                ///< Width for plane visualization
        Color linecolor = Color::blue();   ///< Color of the plane (default: blue)
        Xform xform;   ///< Transformation matrix

        private:
        mutable std::string _guid;         ///< Lazily generated unique identifier
        Point _origin = Point();   ///< Origin (private)
        Vector _x_axis = Vector::x_axis(); ///< X axis (private)
        Vector _y_axis = Vector::y_axis(); ///< Y axis (private)
        Vector _z_axis = Vector::z_axis(); ///< Z axis (private)
        double _a = 0.0; ///< X coordinate (private)
        double _b = 0.0; ///< Y coordinate (private)
        double _c = 1.0; ///< Z coordinate (private)
        double _d = 0.0; ///< W coordinate (private)

        public:

    /// Getters plane attributes
        const Point& origin() const { return _origin; }
        const Vector& x_axis() const { return _x_axis; } 
        const Vector& y_axis() const { return _y_axis; }
        const Vector& z_axis() const { return _z_axis; }
        double a() const { return _a; }
        double b() const { return _b; }
        double c() const { return _c; }
        double d() const { return _d; }

        Plane();
        Plane(const Plane& other);
        Plane& operator=(const Plane& other);
        Plane(const Point& point, const Vector& x_axis, const Vector& y_axis, std::string name = "my_plane");
        Plane(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);
        static Plane from_point_normal(Point& point, Vector& normal, bool normalize = true);
        static Plane from_points(std::vector<Point>& points);
        static Plane from_points_pca(const std::vector<Point>& points);
        static Plane from_two_points(Point& point1, Point& point2);
        static Plane invalid();
        bool is_valid() const;
        static Plane xy_plane();
        static Plane yz_plane();
        static Plane xz_plane();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators - const because they oinly read values, dont modify them
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Minimal string representation
    std::string str() const;

    /// Full string representation
    std::string repr() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply the stored xform transformation to the plane (in-place)
    void transform();

    /// Return a transformed copy of the plane
    Plane transformed() const;

    /// Equality operator (compares origin and axes, ignores guid)
    bool operator==(const Plane &other) const;

    /// Inequality operator
    bool operator!=(const Plane &other) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // No-copy Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    Vector &operator[](int index);
    const Vector &operator[](int index) const;

    Plane &operator+=(const Vector &other);
    Plane &operator-=(const Vector &other);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Copy Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    Plane operator+(const Vector &other) const;
    Plane operator-(const Vector &other) const;


  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to JSON-serializable object
  nlohmann::ordered_json jsondump() const;

  /// Create plane from JSON data
  static Plane jsonload(const nlohmann::json &data);

  /// Serialize to JSON file
  void json_dump(const std::string& filename) const;

  /// Deserialize from JSON file
  static Plane json_load(const std::string& filename);

  /// Convert to JSON string
  std::string json_dumps() const;

  /// Load from JSON string
  static Plane json_loads(const std::string& json_string);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Protobuf
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to protobuf binary string
  std::string pb_dumps() const;

  /// Load from protobuf binary string
  static Plane pb_loads(const std::string& data);

  /// Write protobuf to file
  void pb_dump(const std::string& filename) const;

  /// Read protobuf from file
  static Plane pb_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details
  ///////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Reverse the plane's normal direction.
   */
  void reverse();

  /**
   * @brief Rotate the plane around its normal.
   * 
   * @param angles_in_radians The rotation angle in radians.
   */
  void rotate(double angles_in_radians);

  /**
   * @brief Check if the plane follows the right-hand rule.
   * 
   * @return True if x_axis × y_axis = z_axis (right-handed).
   */
  bool is_right_hand() const;

  /**
   * @brief Check if two planes have the same or flipped normal.
   * 
   * @param plane0 First plane.
   * @param plane1 Second plane.
   * @param can_be_flipped Bool flag to indicate if the normal can be flipped.
   * @return True if two planes pointing to the same or flipped normal.
   */
  static bool is_same_direction(const Plane &plane0, const Plane &plane1, bool can_be_flipped = true);

  /**
   * @brief Check if two planes are in the same position.
   * 
   * @param plane0 First plane.
   * @param plane1 Second plane.
   * @return True if origins are very close to each other.
   */
  static bool is_same_position(const Plane &plane0, const Plane &plane1);

  /**
   * @brief Check if two planes have the same or flipped normal and share the same origin.
   * 
   * @param plane0 First plane.
   * @param plane1 Second plane.
   * @param can_be_flipped Bool flag to indicate if the normal can be flipped.
   * @return True if two planes are pointing to the same or flipped normal and share the same origin.
   */
  static bool is_coplanar(const Plane &plane0, const Plane plane1, bool can_be_flipped = true);
  static bool is_coplanar(const Point& origin0, const Vector& normal0,
                          const Point& origin1, const Vector& normal1,
                          bool can_be_flipped = true, double tolerance = -1.0);

  /**
   * @brief Translate (move) a plane along its normal direction by a specified distance.
   *
   * @param distance Distance to move the plane along its normal (positive = normal direction, negative = opposite).
   * @return New plane translated by the specified distance.
   */
  Plane translate_by_normal(double distance) const;

  /**
   * @brief Project a point onto this plane (orthogonal projection).
   *
   * @param p The point to project.
   * @return The projected point on the plane.
   */
  std::vector<Polyline> to_polylines(double scale = 1.0) const;

  Point project(const Point& p) const {
      const Vector& n = z_axis();
      const Point&  o = origin();
      double dist = (p[0]-o[0])*n[0] + (p[1]-o[1])*n[1] + (p[2]-o[2])*n[2];
      return Point(p[0]-dist*n[0], p[1]-dist*n[1], p[2]-dist*n[2]);
  }

  /**
   * @brief Canonical first in-plane axis using smallest-magnitude pivot.
   *
   * Selects the coordinate of the plane normal with the smallest absolute
   * value, zeroes that coordinate in the output, and populates the other
   * two with the negated pair. The result is orthogonal to the normal and
   * numerically well-conditioned for near-axis-aligned normals.
   *
   * Rule:
   *   if |nx| is smallest → (0, -nz, ny)
   *   if |ny| is smallest → (-nz, 0, nx)
   *   else               → (-ny, nx, 0)
   *
   * Output is unit length.
   */
  Vector base1() const;

  /**
   * @brief Canonical second in-plane axis = z_axis × base1, unit length.
   *
   * Together with `base1()` and `z_axis()` forms a right-handed orthonormal
   * frame on the plane suitable for 2D projection of polygons.
   */
  Vector base2() const;

  /**
   * @brief Sign test using the cached plane equation `ax+by+cz+d`.
   *
   * Returns true if `p` is on the negative side (`ax+by+cz+d < 0`).
   * Mirrors CGAL's `Plane_3::has_on_negative_side()`.
   *
   * @param p Point to test.
   * @return true if `a()*p[0] + b()*p[1] + c()*p[2] + d() < 0`.
   */
  bool has_on_negative_side(const Point& p) const {
      return (a() * p[0] + b() * p[1] + c() * p[2] + d()) < 0.0;
  }


}; // End of Plane class

/**
 * @brief  To use this operator, you can do:
 *         Plane plane();
 *         std::cout << "Created plane: " << plane << std::endl;
 * @param os The output stream.
 * @param plane The Plane to insert into the stream.
 * @return A reference to the output stream.
 */
std::ostream &operator<<(std::ostream &os, const Plane &plane);

} // namespace session_cpp


// fmt formatter specialization for Vector - enables direct fmt::print(plane)
template <> struct fmt::formatter<session_cpp::Plane> {
    constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }
  
    auto format(const session_cpp::Plane &o, fmt::format_context &ctx) const {
      return fmt::format_to(ctx.out(), "{}", o.str());
    }
  };
