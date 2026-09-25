#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <cmath>
#include <fstream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace session_proto {
class Line;
}

namespace session_cpp {

/// A 3D line segment with display width, dash pattern and color.
class Line {
private:
    mutable std::string _guid; // Lazily minted GUID.
    double _x0 = 0.0; // Start x.
    double _y0 = 0.0; // Start y.
    double _z0 = 0.0; // Start z.
    double _x1 = 0.0; // End x.
    double _y1 = 0.0; // End y.
    double _z1 = 1.0; // End z.

public:
    std::string name = "my_line"; // Line name.
    double width = 1.0; // Display width.
    std::vector<double> dash; // Dash pattern lengths.
    Color linecolor = Color::black(); // Display color.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the unit segment from the origin along z.
    Line() = default;

    /// Construct from start and end coordinates.
    Line(double x0, double y0, double z0, double x1, double y1, double z1);

    /// Copy with a new guid and the same data.
    Line(const Line& other);

    /// Copy-assign with a new guid and the same data.
    Line& operator=(const Line& other);

    /// Move while preserving the guid.
    Line(Line&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Line& operator=(Line&& other) noexcept = default;

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
    /// Construct from two points.
    static Line from_points(const Point& p1, const Point& p2);

    /// Construct from point to point + vector.
    static Line from_point_and_vector(const Point& point, const Vector& vector);

    /// Construct from point along the normalized direction.
    static Line from_point_direction_length(const Point& point, const Vector& direction, double length);

    /// Construct the least-squares line through points by power-iteration PCA; length <= 0 spans the projected extent.
    static Line fit_points(const std::vector<Point>& points, double length = 0.0);

    /// Construct a named line from coordinates.
    static Line with_name(const std::string& name, double x0, double y0, double z0, double x1, double y1, double z1);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the mutable coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
    double& operator[](int index);

    /// Return the coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1).
    const double& operator[](int index) const;

    /// Compare name, coordinates to 1e-6, width and linecolor; guid ignored.
    bool operator==(const Line& other) const;

    /// Compare name, coordinates to 1e-6, width and linecolor; guid ignored.
    bool operator!=(const Line& other) const;

    /// Translate in place.
    Line& operator+=(const Vector& other);

    /// Translate back in place.
    Line& operator-=(const Vector& other);

    /// Scale both ends in place.
    Line& operator*=(double factor);

    /// Divide both ends in place.
    Line& operator/=(double factor);

    /// Return a translated copy.
    Line operator+(const Vector& other) const;

    /// Return a copy translated back.
    Line operator-(const Vector& other) const;

    /// Return a copy with both ends scaled.
    Line operator*(double factor) const;

    /// Return a copy with both ends divided.
    Line operator/(double factor) const;

    /// Return a flipped copy (end to start).
    Line operator-() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    Line transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the length.
    double length() const;

    /// Return the squared length.
    double squared_length() const;

    /// Return the vector from start to end.
    Vector to_vector() const;

    /// Return the unit vector from start to end.
    Vector to_direction() const;

    /// Return the start point.
    Point start() const;

    /// Return the end point.
    Point end() const;

    /// Return the midpoint.
    Point center() const;

    /// Return the point at parameter t (0 = start, 1 = end).
    Point point_at(double t) const;

    /// Return n evenly spaced points including both ends.
    std::vector<Point> subdivide(int n) const;

    /// Return points spaced approximately distance apart including both ends.
    std::vector<Point> subdivide_by_distance(double distance) const;

    /// Return the parameter and closest point; limited clamps t to [0, 1].
    std::pair<double, Point> closest_point(const Point& point, bool limited = true) const;

    /// Compute the midpoints of the paired starts and ends.
    static void get_middle_line(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& output_start, Point& output_end);

    /// Compute the line through the midpoints of the paired starts and ends.
    static void get_middle_line(const Line& l0, const Line& l1, Line& out);

    /// Compute the extreme sub-segment of line spanned by the projected points.
    static bool from_projected_points(const Line& line, const std::vector<Point>& points, Line& out);

    /// Split lines and boundary lines in xy at every crossing within tolerance, a collinear overlap kept by the boundary, else by the earlier line; split points within merge welded onto boundary ends, then boundary crossings, then the rest; dangling pieces dropped. Returns the pieces, a piece two lines share kept once, and the index of the line of each, boundary lines numbered after lines.
    static std::pair<std::vector<Line>, std::vector<size_t>> split_at_crossings(const std::vector<Line>& lines, const std::vector<Line>& boundary, double tolerance, double merge);

    /// Compute the collinear overlap with other; false when none or a single point.
    bool overlap(const Line& other, Line& out) const;

    /// Compute the longer of the two midpoint pairings of overlap(other) and other.overlap(this).
    bool overlap_average(const Line& other, Line& out) const;

    /// Grow start by ext_start and end by ext_end.
    void extend(double ext_start, double ext_end);

    /// Grow both ends by dist, or by proportion of the length when non-zero.
    void extend_equally(double dist = 0.0, double proportion = 0.0);

    /// Shrink both ends by dist as a fraction of the length.
    void scale(double dist);

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Line jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Line file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static Line file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Line to_proto() const;

    /// Construct from the protobuf message.
    static Line from_proto(const session_proto::Line& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Line pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static Line pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "x0, y0, z0, x1, y1, z1".
    std::string str() const;

    /// Return "Line(name, x0, y0, z0, x1, y1, z1, Color(...), width)".
    std::string repr() const;
};

/// Write the line string to a stream.
std::ostream& operator<<(std::ostream& os, const Line& line);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Line> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::Line& line, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", line.str());
    }
};
