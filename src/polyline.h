#pragma once
#include "plane.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "xform.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

namespace session_cpp {


/**
 * @class Polyline
 * @brief A polyline defined by a collection of coordinates with an associated plane.
 *
 * Internally stores coordinates as a flat array [x0, y0, z0, x1, y1, z1, ...] for
 * efficient serialization. Provides Point-based API for compatibility.
 */
class Polyline {
public:
    std::string guid = ::guid();
    std::string name = "my_polyline";
    std::vector<double> _coords;  // Flat array [x0, y0, z0, x1, y1, z1, ...]
    Plane plane;
    double width = 1.0;
    Color linecolor = Color::white();

    Xform xform;

    /// Default constructor
    Polyline();

    /// Copy constructor (creates a new guid while copying data)
    Polyline(const Polyline& other);

    /// Copy assignment (creates a new guid while copying data)
    Polyline& operator=(const Polyline& other);

    /// Constructor with points (converts to flat coords internally)
    explicit Polyline(const std::vector<Point>& pts);

    /// Constructor with flat coords
    static Polyline from_coords(const std::vector<double>& coords);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Core Methods (str, repr, duplicate, eq)
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Returns minimal string representation
    std::string str() const;

    /// Returns detailed string representation
    std::string repr() const;

    /// Equality operator (compares values, ignores GUIDs)
    bool operator==(const Polyline& other) const;
    bool operator!=(const Polyline& other) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Point Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Returns the number of points in the polyline
    size_t point_count() const;

    /// Returns the number of points (alias for point_count)
    size_t len() const;

    /// Returns all points as Point objects
    std::vector<Point> get_points() const;

    /// Returns true if the polyline has no points
    bool is_empty() const;

    /// Returns the number of segments (n-1 for n points)
    size_t segment_count() const;

    /// Calculates the total length of the polyline
    double length() const;

    /// Returns the point at the given index
    Point get_point(size_t index) const;

    /// Sets the point at the given index
    void set_point(size_t index, const Point& point);

    /// Adds a point to the end of the polyline
    void add_point(const Point& point);

    /// Inserts a point at the specified index
    void insert_point(size_t index, const Point& point);

    /// Removes and returns the point at the specified index
    bool remove_point(size_t index, Point& out_point);

    /// Reverses the order of points in the polyline
    void reverse();

    /// Returns a new polyline with reversed point order
    Polyline reversed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Translates all points by a vector (in-place)
    Polyline& operator+=(const Vector& v);

    /// Translates all points by a vector (returns new polyline)
    Polyline operator+(const Vector& v) const;

    /// Translates all points by negative vector (in-place)
    Polyline& operator-=(const Vector& v);

    /// Translates all points by negative vector (returns new polyline)
    Polyline operator-(const Vector& v) const;

    /// Multiply all coordinates by scalar (in-place)
    Polyline& operator*=(double factor);

    /// Multiply polyline by scalar (returns new polyline)
    Polyline operator*(double factor) const;

    /// Divide all coordinates by scalar (in-place)
    Polyline& operator/=(double factor);

    /// Divide polyline by scalar (returns new polyline)
    Polyline operator/(double factor) const;

    /// Negate polyline (reverse point order)
    Polyline operator-() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    void transform();
    Polyline transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON-serializable object (uses compact coords format)
    nlohmann::ordered_json jsondump() const;

    /// Create polyline from JSON data (supports both coords and legacy points format)
    static Polyline jsonload(const nlohmann::json& data);

    /// Serialize to JSON file
    void json_dump(const std::string& filename) const;

    /// Deserialize from JSON file
    static Polyline json_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary format
    std::string to_protobuf() const;

    /// Create Polyline from protobuf binary data
    static Polyline from_protobuf(const std::string& data);

    /// Write protobuf to file
    void protobuf_dump(const std::string& filename) const;

    /// Read protobuf from file
    static Polyline protobuf_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Utilities
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Shift polyline points by specified number of positions
    void shift(int times);

    /// Calculate squared length of polyline (faster, no sqrt)
    double length_squared() const;

    /// Get point at parameter t along a line segment (t=0 is start, t=1 is end)
    static Point point_at(const Point& start, const Point& end, double t);

    /// Find closest point on line segment to given point, returns parameter t
    static void closest_point_to_line(const Point& point, const Point& line_start, 
                                     const Point& line_end, double& t);

    /// Check if two line segments overlap and return the overlapping segment
    static bool line_line_overlap(const Point& line0_start, const Point& line0_end,
                                 const Point& line1_start, const Point& line1_end,
                                 Point& overlap_start, Point& overlap_end);

    /// Calculate average of two line segments
    static void line_line_average(const Point& line0_start, const Point& line0_end,
                                 const Point& line1_start, const Point& line1_end,
                                 Point& output_start, Point& output_end);

    /// Calculate overlap average of two line segments
    static void line_line_overlap_average(const Point& line0_start, const Point& line0_end,
                                         const Point& line1_start, const Point& line1_end,
                                         Point& output_start, Point& output_end);

    /// Create line from projected points onto a base line
    static bool line_from_projected_points(const Point& line_start, const Point& line_end,
                                          const std::vector<Point>& points,
                                          Point& output_start, Point& output_end);

    /// Find closest distance and point from a point to this polyline
    double closest_distance_and_point(const Point& point, size_t& edge_id, Point& closest_point) const;

    /// Check if polyline is closed (first and last points are the same)
    bool is_closed() const;

    /// Calculate center point of polyline
    Point center() const;

    /// Get average plane from polyline points
    void get_average_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) const;

    /// Get fast plane calculation from polyline
    void get_fast_plane(Point& origin, Plane& pln) const;

    /// Extend polyline segment
    void extend_segment(int segment_id, double dist0, double dist1, 
                       double proportion0 = 0.0, double proportion1 = 0.0);

    /// Extend segment equally on both ends (static utility)
    static void extend_segment_equally(Point& segment_start, Point& segment_end, 
                                      double dist, double proportion = 0.0);

    /// Extend polyline segment equally
    void extend_segment_equally(int segment_id, double dist, double proportion = 0.0);

    /// Check if polyline is clockwise oriented
    bool is_clockwise(const Plane& pln) const;

    /// Get convex/concave corners of polyline
    void get_convex_corners(std::vector<bool>& convex_or_concave) const;

    /// Interpolate between two polylines
    static Polyline tween_two_polylines(const Polyline& polyline0, const Polyline& polyline1, 
                                       double weight);

private:
    /// Helper to recompute plane when points change
    void recompute_plane_if_needed();

    /// Calculate average normal from polyline points
    void average_normal(Vector& average_normal) const;
};

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const Polyline& polyline);

} // namespace session_cpp

// fmt formatter specialization for Polyline
template <> struct fmt::formatter<session_cpp::Polyline> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const session_cpp::Polyline& polyline, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "Polyline(guid={}, name={}, points={})",
                            polyline.guid, polyline.name, polyline.point_count());
    }
};
