
#pragma once
#include "color.h"
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace session_cpp {

class Point;    // Forward declaration
class Vector;   // Forward declaration
class Plane;    // Forward declaration
class Line;     // Forward declaration
class Polyline; // Forward declaration

/**
 * @class Xform
 * @brief A 4x4 transformation matrix.
 */
class Xform {
public:
    /// Human-readable name
    std::string name = "my_xform";

    /// Lazy GUID accessor (const)
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Lazy GUID accessor (mutable)
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Column-major 4x4 matrix values
    std::array<double, 16> m;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor (identity)
    Xform();

    /// Construct from matrix values (column-major)
    Xform(const std::array<double, 16>& matrix);

    /// Copy constructor (creates a new guid while copying data)
    Xform(const Xform& other);

    /// Copy assignment (creates a new guid while copying data)
    Xform& operator=(const Xform& other);

    /// Identity matrix
    static Xform identity();

    /// Build from array values (column-major)
    static Xform from_matrix(const std::array<double, 16>& matrix);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Build a pure rotation (no translation) from three column axis vectors.
    /// Port of wood `internal::rotation_in_xy_plane(x, y, z)`.
    static Xform from_axes(const Vector& col_x, const Vector& col_y, const Vector& col_z);

    /// Translation by x, y, z
    static Xform translation(double x, double y, double z);

    /// Rotation around X axis
    static Xform rotation_x(double angle, bool degrees = false);

    /// Rotation around Y axis
    static Xform rotation_y(double angle, bool degrees = false);

    /// Rotation around Z axis
    static Xform rotation_z(double angle, bool degrees = false);

    /// Rotation around arbitrary axis
    static Xform rotation(Vector& axis, double angle, bool degrees = false);

    /// Rotation around a line (point + direction)
    static Xform rotation_around_line(const Line& line, double angle, bool degrees = false);

    /// Change of basis between two coordinate systems
    static Xform change_basis(Point& origin_1, Vector& x_axis_1, Vector& y_axis_1, Vector& z_axis_1,
                               Point& origin_0, Vector& x_axis_0, Vector& y_axis_0, Vector& z_axis_0);

    /**
     * @brief Build a change-of-basis Xform that maps the unit cube
     *        `[-0.5, +0.5]³` to the world frame defined by two
     *        joint-volume rectangles.
     *
     * Source frame (unit cube): O1=(-0.5,-0.5,-0.5),
     * X1=(1,0,0), Y1=(0,1,0), Z1=(0,0,1).
     *
     * Target frame: O0=rect0[0],
     * X0=rect0[1]-rect0[0], Y0=rect0[3]-rect0[0], Z0=rect1[0]-rect0[0].
     *
     * Verbatim port of the wood `change_basis(rect0, rect1)` overload
     * used by the joint orientation pipeline. Wood reference:
     * `wood_joint.cpp:103-245`.
     *
     * @param rect0 First joint volume rectangle (5 points, closed quad).
     * @param rect1 Second joint volume rectangle (5 points, closed quad).
     * @return The change-of-basis matrix, or `Xform::identity()` if the
     *         input frame is degenerate.
     */
    static Xform from_change_of_basis(const Polyline& rect0, const Polyline& rect1);

    /// Transform mapping one plane to another
    static Xform plane_to_plane(const Plane& plane_from, const Plane& plane_to);

    /// Transform from a plane coordinate system to XY
    static Xform plane_to_xy(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis);

    /// Transform from XY to a plane coordinate system
    static Xform xy_to_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis);

    /// Transform world points INTO a local frame defined by (origin, x, y, z).
    /// Given a world point p, returns (u, v, w) such that
    /// `p = origin + u*x_hat + v*y_hat + w*z_hat`. Stores the basis as matrix
    /// ROWS (world-to-local), unlike `plane_to_xy` which stores them as
    /// COLUMNS and therefore actually does local-to-world despite its name.
    /// Use this when you need a faithful 3D projection — especially when the
    /// input geometry's normal can align with one of the basis axes (where
    /// `plane_to_xy` collapses a dimension). See wood_main.cpp type-13 branch.
    static Xform world_to_frame(const Point& origin, Vector x_axis, Vector y_axis, Vector z_axis);

    /// Inverse of `world_to_frame`: local (u,v,w) → world point at
    /// `origin + u*x_hat + v*y_hat + w*z_hat`.
    static Xform frame_to_world(const Point& origin, Vector x_axis, Vector y_axis, Vector z_axis);

    /// Transform from world XY to target frame/plane (same as COMPAS from_frame)
    static Xform to_frame(const Plane& frame);

    /// Scale along XYZ about the world origin
    static Xform scale_xyz(double scale_x, double scale_y, double scale_z);

    /// Uniform scale about a point
    static Xform scale_uniform(Point& origin, double scale_value);

    /// Non-uniform scale about a point
    static Xform scale_non_uniform(Point& origin, double scale_x, double scale_y, double scale_z);

    /// Axis-angle rotation using Rodrigues' formula
    static Xform axis_rotation(double angle, Vector& axis, bool degrees = false);

    /// Right-handed view matrix: camera at eye looking at target point.
    /// Right-handed means X cross Y = Z (OpenGL/Vulkan convention). Camera looks down -Z.
    /// Left-handed (DirectX) would look down +Z instead.
    /// Not a projection — rigid transform (rotation + translation), no distortion.
    /// up should be (0,0,1) for Z-up convention.
    /// up must not be parallel to the view direction — produces NaN.
    /// Handling parallel up is the camera controller's responsibility, not this function's.
    static Xform look_at_right_handed(const Point& eye, const Point& target, const Vector& up);

    /// Same as look_at_right_handed but takes a direction vector instead of a target point.
    /// look_to(eye, dir, up) == look_at(eye, eye+dir, up)
    static Xform look_to_right_handed(const Point& eye, const Vector& direction, const Vector& up);

    /// Perspective projection (right-handed, depth [0,1])
    static Xform perspective(double fov_y, double aspect, double near, double far);

    /// Orthographic projection (right-handed, depth [0,1])
    static Xform orthographic(double left, double right, double bottom, double top, double near, double far);

    /// Orthogonal projection onto an arbitrary plane
    static Xform project_to_plane(const Plane& plane);

    /// Projection onto an arbitrary plane along a direction vector
    static Xform project_to_plane_by_axis(const Plane& plane, const Vector& direction);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Matrix inverse if invertible
    std::optional<Xform> inverse() const;

    /// Check if matrix is identity (within tolerance)
    bool is_identity() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to ordered JSON object
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from JSON object
    static Xform jsonload(const nlohmann::json& data);

    /// Write JSON to file
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from file
    static Xform file_json_load(const std::string& filename);

    /// Convert to JSON string
    std::string file_json_dumps() const;

    /// Load from JSON string
    static Xform file_json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static Xform pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static Xform pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Matrix multiplication
    Xform operator*(const Xform& other) const;

    /// In-place matrix multiplication
    Xform& operator*=(const Xform& other);

    /// Matrix element (row, col) mutable accessor
    double& operator()(int row, int col);

    /// Matrix element (row, col) const accessor
    const double& operator()(int row, int col) const;

    /// Equality operator (compare matrix values with tolerance)
    bool operator==(const Xform& other) const;

    /// Inequality operator
    bool operator!=(const Xform& other) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Minimal string representation (matrix values)
    std::string str() const;

    /// Full string representation (name, guid prefix)
    std::string repr() const;

private:
    mutable std::string _guid; ///< Lazily generated unique identifier
};

} // namespace session_cpp
