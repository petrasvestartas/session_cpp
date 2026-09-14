#pragma once
#include "guid.h"
#include "json.h"
#include <array>
#include <cmath>
#include <optional>
#include <string>

namespace session_cpp {

class Point;
class Vector;
class Plane;
class Line;
class Polyline;

/// A 4x4 column-major transformation matrix
class Xform {
public:
    std::string name = "my_xform";
    std::array<double, 16> m;

    /// Identity
    Xform();

    /// Construct from column-major values
    Xform(const std::array<double, 16>& matrix);

    /// Copy constructor (new guid, same data)
    Xform(const Xform& other);

    /// Copy assignment (new guid, same data)
    Xform& operator=(const Xform& other);

    /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
    Xform(Xform&& other) noexcept = default;
    Xform& operator=(Xform&& other) noexcept = default;

    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    static Xform identity();
    static Xform from_matrix(const std::array<double, 16>& matrix);

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Pure rotation from three column axis vectors
    static Xform from_axes(const Vector& col_x, const Vector& col_y, const Vector& col_z);
    static Xform translation(double x, double y, double z);
    static Xform rotation_x(double angle, bool degrees = false);
    static Xform rotation_y(double angle, bool degrees = false);
    static Xform rotation_z(double angle, bool degrees = false);
    static Xform rotation(const Vector& axis, double angle, bool degrees = false);
    static Xform rotation_around_line(const Line& line, double angle, bool degrees = false);

    /// Change of basis from frame 1 to frame 0
    static Xform change_basis(
        const Point& origin_1, const Vector& x_axis_1, const Vector& y_axis_1, const Vector& z_axis_1,
        const Point& origin_0, const Vector& x_axis_0, const Vector& y_axis_0, const Vector& z_axis_0
    );

    /// Unit cube [-0.5, 0.5]^3 to the joint volume frame spanned by rect0 (x, y) and rect1[0] (z)
    static Xform from_change_of_basis(const Polyline& rect0, const Polyline& rect1);
    static Xform plane_to_plane(const Plane& plane_from, const Plane& plane_to);

    /// Frame axes as columns, minus origin (local-to-world despite the name)
    static Xform plane_to_xy(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);
    static Xform xy_to_plane(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);

    /// World point to frame coordinates (axes as rows)
    static Xform world_to_frame(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);

    /// Frame coordinates to world point (axes as columns)
    static Xform frame_to_world(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);

    /// World XY to the frame plane (COMPAS from_frame)
    static Xform to_frame(const Plane& frame);
    static Xform scale_xyz(double scale_x, double scale_y, double scale_z);
    static Xform scale_uniform(const Point& origin, double scale_value);
    static Xform scale_non_uniform(const Point& origin, double scale_x, double scale_y, double scale_z);

    /// Rodrigues rotation about a unit axis
    static Xform axis_rotation(double angle, const Vector& axis, bool degrees = false);

    /// Right-handed view matrix (camera looks down -Z, up must not be parallel to the view)
    static Xform look_at_right_handed(const Point& eye, const Point& target, const Vector& up);
    static Xform look_to_right_handed(const Point& eye, const Vector& direction, const Vector& up);

    /// Right-handed projection, depth [0, 1]
    static Xform perspective(double fov_y, double aspect, double near, double far);
    static Xform orthographic(double left, double right, double bottom, double top, double near, double far);
    static Xform project_to_plane(const Plane& plane);
    static Xform project_to_plane_by_axis(const Plane& plane, const Vector& direction);

    // ═══════════════════════════════════════════════════════════════════════════
    // Apply Transformations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Homogeneous multiply, divides by w when projective
    Point transform_point(const Point& p) const;

    /// Rotation and scale only
    Vector transform_vector(const Vector& v) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════

    std::optional<Xform> inverse() const;
    bool is_identity() const;

    /// Four columns of four rows
    std::array<std::array<double, 4>, 4> to_cols() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    nlohmann::ordered_json jsondump() const;
    static Xform jsonload(const nlohmann::json& data);
    void file_json_dump(const std::string& filename) const;
    static Xform file_json_load(const std::string& filename);
    std::string file_json_dumps() const;
    static Xform file_json_loads(const std::string& json_string);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    std::string pb_dumps() const;
    static Xform pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static Xform pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════

    Xform operator*(const Xform& other) const;
    Xform& operator*=(const Xform& other);

    /// Element at (row, col)
    double& operator()(int row, int col);
    const double& operator()(int row, int col) const;
    bool operator==(const Xform& other) const;
    bool operator!=(const Xform& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // String Representations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Four matrix rows
    std::string str() const;

    /// Name and guid prefix
    std::string repr() const;

private:
    mutable std::string _guid;
};

} // namespace session_cpp
