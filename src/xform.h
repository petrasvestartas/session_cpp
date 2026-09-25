#pragma once
#include "guid.h"
#include "json.h"
#include <array>
#include <cmath>
#include <optional>
#include <string>

namespace session_proto {
class Xform;
}

namespace session_cpp {

class Point;
class Vector;
class Plane;
class Line;
class Polyline;

/// A 4x4 column-major transformation matrix.
class Xform {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_xform"; // Xform name.
    std::array<double, 16> m; // Column-major values.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the identity.
    Xform();

    /// Construct from column-major values.
    Xform(const std::array<double, 16>& matrix);

    /// Copy with a new guid and the same data.
    Xform(const Xform& other);

    /// Copy-assign with a new guid and the same data.
    Xform& operator=(const Xform& other);

    /// Move while preserving the guid.
    Xform(Xform&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Xform& operator=(Xform&& other) noexcept = default;

    /// Construct the identity.
    static Xform identity();

    /// Construct from column-major values.
    static Xform from_matrix(const std::array<double, 16>& matrix);

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

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Multiply two transforms.
    Xform operator*(const Xform& other) const;

    /// Multiply in place.
    Xform& operator*=(const Xform& other);

    /// Return the mutable element at (row, col).
    double& operator()(int row, int col);

    /// Return the element at (row, col).
    const double& operator()(int row, int col) const;

    /// Compare all elements within tolerance.
    bool operator==(const Xform& other) const;

    /// Compare all elements within tolerance.
    bool operator!=(const Xform& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformations
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a pure rotation from three column axis vectors.
    static Xform from_axes(const Vector& col_x, const Vector& col_y, const Vector& col_z);

    /// Construct a translation.
    static Xform translation(double x, double y, double z);

    /// Construct a rotation about the x axis.
    static Xform rotation_x(double angle, bool degrees = false);

    /// Construct a rotation about the y axis.
    static Xform rotation_y(double angle, bool degrees = false);

    /// Construct a rotation about the z axis.
    static Xform rotation_z(double angle, bool degrees = false);

    /// Construct a rotation about an arbitrary axis through the origin.
    static Xform rotation(const Vector& axis, double angle, bool degrees = false);

    /// Construct a rotation about a line.
    static Xform rotation_around_line(const Line& line, double angle, bool degrees = false);

    /// Construct a change of basis from frame 1 to frame 0.
    static Xform change_basis(
        const Point& origin_1,
        const Vector& x_axis_1,
        const Vector& y_axis_1,
        const Vector& z_axis_1,
        const Point& origin_0,
        const Vector& x_axis_0,
        const Vector& y_axis_0,
        const Vector& z_axis_0
    );

    /// Map the unit cube [-0.5, 0.5]^3 to the joint volume frame spanned by rect0 (x, y) and rect1[0] (z).
    static Xform from_change_of_basis(const Polyline& rect0, const Polyline& rect1);

    /// Construct the transform taking one plane to another.
    static Xform plane_to_plane(const Plane& plane_from, const Plane& plane_to);

    /// Construct the world point to frame coordinates transform (axes as rows).
    static Xform world_to_frame(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);

    /// Construct the frame coordinates to world point transform (axes as columns).
    static Xform frame_to_world(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);

    /// Construct the world XY to frame plane transform (COMPAS from_frame).
    static Xform to_frame(const Plane& frame);

    /// Construct a scale about the origin.
    static Xform scale_xyz(double scale_x, double scale_y, double scale_z);

    /// Construct a uniform scale about a point.
    static Xform scale_uniform(const Point& origin, double scale_value);

    /// Construct a non-uniform scale about a point.
    static Xform scale_non_uniform(const Point& origin, double scale_x, double scale_y, double scale_z);

    /// Construct a Rodrigues rotation about a unit axis.
    static Xform axis_rotation(double angle, const Vector& axis, bool degrees = false);

    /// Construct a right-handed view matrix looking at a target (camera looks down -Z, up must not be parallel to the view).
    static Xform look_at_right_handed(const Point& eye, const Point& target, const Vector& up);

    /// Construct a right-handed view matrix looking along a direction.
    static Xform look_to_right_handed(const Point& eye, const Vector& direction, const Vector& up);

    /// Construct a right-handed perspective projection with depth [0, 1].
    static Xform perspective(double fov_y, double aspect, double near, double far);

    /// Construct a right-handed orthographic projection with depth [0, 1].
    static Xform orthographic(double left, double right, double bottom, double top, double near, double far);

    /// Construct an orthogonal projection onto a plane.
    static Xform project_to_plane(const Plane& plane);

    /// Construct a projection onto a plane along a direction.
    static Xform project_to_plane_by_axis(const Plane& plane, const Vector& direction);

    // ═══════════════════════════════════════════════════════════════════════════
    // Apply transformations
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform a point with a homogeneous multiply, dividing by w when projective.
    Point transform_point(const Point& p) const;

    /// Transform a vector with rotation and scale only.
    Vector transform_vector(const Vector& v) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the inverse, or nullopt when singular.
    std::optional<Xform> inverse() const;

    /// Return whether the matrix is the identity.
    bool is_identity() const;

    /// Return four columns of four rows.
    std::array<std::array<double, 4>, 4> to_cols() const;

    /// Return the length of the first column: the uniform scale the matrix applies.
    double uniform_scale() const;

    /// Return the eye of a view-projection: where clip x, y and w vanish at once; orthographic has none, so the view direction pushed far back.
    Point eye() const;

    /// Return the half-height of an orthographic view-projection in world units, 0 in perspective.
    double ortho_half_height() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Xform jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Xform file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Xform file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Xform to_proto() const;

    /// Construct from the protobuf message.
    static Xform from_proto(const session_proto::Xform& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Xform pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Xform pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the four matrix rows.
    std::string str() const;

    /// Return the name and guid prefix.
    std::string repr() const;

private:
    /// Return the determinant of a 3x3 given by rows.
    static double det3(const std::array<std::array<double, 3>, 3>& rows);
};

} // namespace session_cpp
