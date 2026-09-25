#include "xform.h"
#include "vector.h"
#include "point.h"
#include "plane.h"
#include "line.h"
#include "polyline.h"
#include "tolerance.h"
#include "xform.pb.h"
#include "fmt/core.h"
#include <algorithm>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Xform::Xform() {

    m.fill(0.0);
    m[0] = 1.0;
    m[5] = 1.0;
    m[10] = 1.0;
    m[15] = 1.0;
}

Xform::Xform(const std::array<double, 16>& matrix) : m(matrix) {}

Xform::Xform(const Xform& other) : name(other.name), m(other.m) {}

Xform& Xform::operator=(const Xform& other) {

    if (this != &other) {
        _guid.clear();
        name = other.name;
        m = other.m;
    }

    return *this;
}

Xform Xform::identity() {
    return Xform();
}

Xform Xform::from_matrix(const std::array<double, 16>& matrix) {
    return Xform(matrix);
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& Xform::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& Xform::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
Xform Xform::operator*(const Xform& other) const {

    Xform result;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.0;

            for (int k = 0; k < 4; k++)
                sum += m[k * 4 + i] * other.m[j * 4 + k];

            result.m[j * 4 + i] = sum;
        }
    }

    return result;
}

Xform& Xform::operator*=(const Xform& other) {

    *this = *this * other;

    return *this;
}

double& Xform::operator()(int row, int col) {

    if (row < 0 || row >= 4 || col < 0 || col >= 4)
        throw std::out_of_range(fmt::format("Index out of bounds: ({}, {})", row, col));

    return m[col * 4 + row];
}

const double& Xform::operator()(int row, int col) const {

    if (row < 0 || row >= 4 || col < 0 || col >= 4)
        throw std::out_of_range(fmt::format("Index out of bounds: ({}, {})", row, col));

    return m[col * 4 + row];
}

bool Xform::operator==(const Xform& other) const {

    for (int i = 0; i < 16; i++)
        if (std::abs(m[i] - other.m[i]) > 1e-10)
            return false;

    return true;
}

bool Xform::operator!=(const Xform& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformations
// ═══════════════════════════════════════════════════════════════════════════
Xform Xform::from_axes(const Vector& col_x, const Vector& col_y, const Vector& col_z) {

    Xform xform;
    xform.m[0] = col_x[0];
    xform.m[1] = col_x[1];
    xform.m[2] = col_x[2];
    xform.m[4] = col_y[0];
    xform.m[5] = col_y[1];
    xform.m[6] = col_y[2];
    xform.m[8] = col_z[0];
    xform.m[9] = col_z[1];
    xform.m[10] = col_z[2];

    return xform;
}

Xform Xform::translation(double x, double y, double z) {

    Xform xform;
    xform.m[12] = x;
    xform.m[13] = y;
    xform.m[14] = z;

    return xform;
}

Xform Xform::rotation_x(double angle, bool degrees) {

    if (degrees)
        angle *= Tolerance::TO_RADIANS;

    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);

    Xform xform;
    xform.m[5] = cos_angle;
    xform.m[6] = sin_angle;
    xform.m[9] = -sin_angle;
    xform.m[10] = cos_angle;

    return xform;
}

Xform Xform::rotation_y(double angle, bool degrees) {

    if (degrees)
        angle *= Tolerance::TO_RADIANS;

    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);

    Xform xform;
    xform.m[0] = cos_angle;
    xform.m[2] = -sin_angle;
    xform.m[8] = sin_angle;
    xform.m[10] = cos_angle;

    return xform;
}

Xform Xform::rotation_z(double angle, bool degrees) {

    if (degrees)
        angle *= Tolerance::TO_RADIANS;

    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);

    Xform xform;
    xform.m[0] = cos_angle;
    xform.m[1] = sin_angle;
    xform.m[4] = -sin_angle;
    xform.m[5] = cos_angle;

    return xform;
}

Xform Xform::rotation(const Vector& axis, double angle, bool degrees) {

    if (degrees)
        angle *= Tolerance::TO_RADIANS;

    if (axis.is_zero())
        return Xform::identity();

    const Vector unit = axis.normalized();
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    const double one_minus_cos = 1.0 - cos_angle;
    const double xx = unit[0] * unit[0];
    const double xy = unit[0] * unit[1];
    const double xz = unit[0] * unit[2];
    const double yy = unit[1] * unit[1];
    const double yz = unit[1] * unit[2];
    const double zz = unit[2] * unit[2];

    Xform xform;
    xform.m[0] = cos_angle + xx * one_minus_cos;
    xform.m[1] = xy * one_minus_cos + unit[2] * sin_angle;
    xform.m[2] = xz * one_minus_cos - unit[1] * sin_angle;
    xform.m[4] = xy * one_minus_cos - unit[2] * sin_angle;
    xform.m[5] = cos_angle + yy * one_minus_cos;
    xform.m[6] = yz * one_minus_cos + unit[0] * sin_angle;
    xform.m[8] = xz * one_minus_cos + unit[1] * sin_angle;
    xform.m[9] = yz * one_minus_cos - unit[0] * sin_angle;
    xform.m[10] = cos_angle + zz * one_minus_cos;

    return xform;
}

Xform Xform::rotation_around_line(const Line& line, double angle, bool degrees) {

    const Point p = line.start();
    const Vector d = line.to_direction();
    const Xform t0 = translation(-p[0], -p[1], -p[2]);
    const Xform r = rotation(d, angle, degrees);
    const Xform t1 = translation(p[0], p[1], p[2]);

    return t1 * (r * t0);
}

/// Scale row p to a unit pivot, then clear column p in rows a and b; false on a zero pivot.
static bool change_basis_pivot(double r[3][6], int p, int a, int b) {

    if (r[p][p] == 0.0)
        return false;

    double d = 1.0 / r[p][p];

    for (int j = 0; j < 6; j++)
        r[p][j] *= d;

    r[p][p] = 1.0;

    if (r[a][p] != 0.0) {
        d = -r[a][p];

        for (int j = 0; j < 6; j++)
            r[a][j] += d * r[p][j];

        r[a][p] = 0.0;
    }

    if (r[b][p] != 0.0) {
        d = -r[b][p];

        for (int j = 0; j < 6; j++)
            r[b][j] += d * r[p][j];

        r[b][p] = 0.0;
    }

    return true;
}

Xform Xform::change_basis(
    const Point& origin_1,
    const Vector& x_axis_1,
    const Vector& y_axis_1,
    const Vector& z_axis_1,
    const Point& origin_0,
    const Vector& x_axis_0,
    const Vector& y_axis_0,
    const Vector& z_axis_0
) {

    const double a = x_axis_1.dot(y_axis_1);
    const double b = x_axis_1.dot(z_axis_1);
    const double c = y_axis_1.dot(z_axis_1);
    double r[3][6] = {
        {x_axis_1.dot(x_axis_1), a, b, x_axis_1.dot(x_axis_0), x_axis_1.dot(y_axis_0), x_axis_1.dot(z_axis_0)},
        {a, y_axis_1.dot(y_axis_1), c, y_axis_1.dot(x_axis_0), y_axis_1.dot(y_axis_0), y_axis_1.dot(z_axis_0)},
        {b, c, z_axis_1.dot(z_axis_1), z_axis_1.dot(x_axis_0), z_axis_1.dot(y_axis_0), z_axis_1.dot(z_axis_0)}
    };

    int i0 = (r[0][0] >= r[1][1]) ? 0 : 1;

    if (r[2][2] > r[i0][i0])
        i0 = 2;

    int i1 = (i0 + 1) % 3;
    int i2 = (i1 + 1) % 3;

    if (!change_basis_pivot(r, i0, i1, i2))
        return Xform::identity();

    if (std::abs(r[i1][i1]) < std::abs(r[i2][i2]))
        std::swap(i1, i2);

    if (!change_basis_pivot(r, i1, i0, i2))
        return Xform::identity();

    if (!change_basis_pivot(r, i2, i0, i1))
        return Xform::identity();

    Xform m_xform;
    m_xform.m[0] = r[0][3];
    m_xform.m[4] = r[0][4];
    m_xform.m[8] = r[0][5];
    m_xform.m[1] = r[1][3];
    m_xform.m[5] = r[1][4];
    m_xform.m[9] = r[1][5];
    m_xform.m[2] = r[2][3];
    m_xform.m[6] = r[2][4];
    m_xform.m[10] = r[2][5];

    const Xform t0 = translation(-origin_1[0], -origin_1[1], -origin_1[2]);
    const Xform t2 = translation(origin_0[0], origin_0[1], origin_0[2]);

    return t2 * (m_xform * t0);
}

Xform Xform::from_change_of_basis(const Polyline& rect0, const Polyline& rect1) {

    if (rect0.point_count() < 4 || rect1.point_count() < 1)
        return Xform::identity();

    const Point origin_1(-0.5, -0.5, -0.5);
    const Vector x_axis_1(1, 0, 0);
    const Vector y_axis_1(0, 1, 0);
    const Vector z_axis_1(0, 0, 1);
    const Point origin_0 = rect0.get_point(0);
    const Vector x_axis_0 = rect0.get_point(1) - origin_0;
    const Vector y_axis_0 = rect0.get_point(3) - origin_0;
    const Vector z_axis_0 = rect1.get_point(0) - origin_0;

    return change_basis(origin_1, x_axis_1, y_axis_1, z_axis_1, origin_0, x_axis_0, y_axis_0, z_axis_0);
}

Xform Xform::plane_to_plane(const Plane& plane_from, const Plane& plane_to) {

    const Vector x0 = plane_from.x_axis().normalized();
    const Vector y0 = plane_from.y_axis().normalized();
    const Vector z0 = plane_from.z_axis().normalized();
    const Vector x1 = plane_to.x_axis().normalized();
    const Vector y1 = plane_to.y_axis().normalized();
    const Vector z1 = plane_to.z_axis().normalized();
    const Point& origin_0 = plane_from.origin();
    const Point& origin_1 = plane_to.origin();

    Xform f0;
    f0.m[0] = x0[0];
    f0.m[1] = x0[1];
    f0.m[2] = x0[2];
    f0.m[4] = y0[0];
    f0.m[5] = y0[1];
    f0.m[6] = y0[2];
    f0.m[8] = z0[0];
    f0.m[9] = z0[1];
    f0.m[10] = z0[2];

    Xform f1;
    f1.m[0] = x1[0];
    f1.m[4] = x1[1];
    f1.m[8] = x1[2];
    f1.m[1] = y1[0];
    f1.m[5] = y1[1];
    f1.m[9] = y1[2];
    f1.m[2] = z1[0];
    f1.m[6] = z1[1];
    f1.m[10] = z1[2];

    const Xform t0 = translation(-origin_0[0], -origin_0[1], -origin_0[2]);
    const Xform r = f1 * f0;
    const Xform t1 = translation(origin_1[0], origin_1[1], origin_1[2]);

    return t1 * (r * t0);
}

Xform Xform::world_to_frame(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis) {

    const Vector x = x_axis.normalized();
    const Vector y = y_axis.normalized();
    const Vector z = z_axis.normalized();

    Xform f;
    f.m[0] = x[0];
    f.m[4] = x[1];
    f.m[8] = x[2];
    f.m[1] = y[0];
    f.m[5] = y[1];
    f.m[9] = y[2];
    f.m[2] = z[0];
    f.m[6] = z[1];
    f.m[10] = z[2];

    const Xform t = translation(-origin[0], -origin[1], -origin[2]);

    return f * t;
}

Xform Xform::frame_to_world(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis) {

    const Vector x = x_axis.normalized();
    const Vector y = y_axis.normalized();
    const Vector z = z_axis.normalized();

    Xform f;
    f.m[0] = x[0];
    f.m[1] = x[1];
    f.m[2] = x[2];
    f.m[4] = y[0];
    f.m[5] = y[1];
    f.m[6] = y[2];
    f.m[8] = z[0];
    f.m[9] = z[1];
    f.m[10] = z[2];

    const Xform t = translation(origin[0], origin[1], origin[2]);

    return t * f;
}

Xform Xform::to_frame(const Plane& frame) {

    const Vector x = frame.x_axis().normalized();
    const Vector y = frame.y_axis().normalized();
    const Vector z = frame.z_axis().normalized();
    const Point& o = frame.origin();

    Xform xform;
    xform.m[0] = x[0];
    xform.m[4] = y[0];
    xform.m[8] = z[0];
    xform.m[12] = o[0];
    xform.m[1] = x[1];
    xform.m[5] = y[1];
    xform.m[9] = z[1];
    xform.m[13] = o[1];
    xform.m[2] = x[2];
    xform.m[6] = y[2];
    xform.m[10] = z[2];
    xform.m[14] = o[2];

    return xform;
}

Xform Xform::scale_xyz(double scale_x, double scale_y, double scale_z) {

    Xform xform;
    xform.m[0] = scale_x;
    xform.m[5] = scale_y;
    xform.m[10] = scale_z;

    return xform;
}

Xform Xform::scale_uniform(const Point& origin, double scale_value) {

    const Xform t0 = translation(-origin[0], -origin[1], -origin[2]);
    const Xform t1 = scale_xyz(scale_value, scale_value, scale_value);
    const Xform t2 = translation(origin[0], origin[1], origin[2]);

    return t2 * (t1 * t0);
}

Xform Xform::scale_non_uniform(const Point& origin, double scale_x, double scale_y, double scale_z) {

    const Xform t0 = translation(-origin[0], -origin[1], -origin[2]);
    const Xform t1 = scale_xyz(scale_x, scale_y, scale_z);
    const Xform t2 = translation(origin[0], origin[1], origin[2]);

    return t2 * (t1 * t0);
}

Xform Xform::axis_rotation(double angle, const Vector& axis, bool degrees) {

    if (degrees)
        angle *= Tolerance::TO_RADIANS;

    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;
    const double ux = axis[0];
    const double uy = axis[1];
    const double uz = axis[2];

    Xform xform;
    xform.m[0] = t * ux * ux + c;
    xform.m[4] = t * ux * uy - uz * s;
    xform.m[8] = t * ux * uz + uy * s;
    xform.m[1] = t * ux * uy + uz * s;
    xform.m[5] = t * uy * uy + c;
    xform.m[9] = t * uy * uz - ux * s;
    xform.m[2] = t * ux * uz - uy * s;
    xform.m[6] = t * uy * uz + ux * s;
    xform.m[10] = t * uz * uz + c;

    return xform;
}

Xform Xform::look_at_right_handed(const Point& eye, const Point& target, const Vector& up) {
    return look_to_right_handed(eye, target - eye, up);
}

Xform Xform::look_to_right_handed(const Point& eye, const Vector& direction, const Vector& up) {

    const Vector f = direction.normalized();
    const Vector s = f.cross(up.normalized()).normalized();
    const Vector u = s.cross(f);
    const Vector eye_vector(eye[0], eye[1], eye[2]);

    Xform xform;
    xform.m[0] = s[0];
    xform.m[4] = s[1];
    xform.m[8] = s[2];
    xform.m[1] = u[0];
    xform.m[5] = u[1];
    xform.m[9] = u[2];
    xform.m[2] = -f[0];
    xform.m[6] = -f[1];
    xform.m[10] = -f[2];
    xform.m[12] = -s.dot(eye_vector);
    xform.m[13] = -u.dot(eye_vector);
    xform.m[14] = f.dot(eye_vector);

    return xform;
}

Xform Xform::perspective(double fov_y, double aspect, double near, double far) {

    const double f = 1.0 / std::tan(fov_y / 2.0);
    const double nf = near - far;

    Xform xform;
    xform.m.fill(0.0);
    xform.m[0] = f / aspect;
    xform.m[5] = f;
    xform.m[10] = far / nf;
    xform.m[11] = -1.0;
    xform.m[14] = (near * far) / nf;

    return xform;
}

Xform Xform::orthographic(double left, double right, double bottom, double top, double near, double far) {

    const double rl = right - left;
    const double tb = top - bottom;
    const double nf = near - far;

    Xform xform;
    xform.m.fill(0.0);
    xform.m[0] = 2.0 / rl;
    xform.m[5] = 2.0 / tb;
    xform.m[10] = 1.0 / nf;
    xform.m[12] = (left + right) / (left - right);
    xform.m[13] = (bottom + top) / (bottom - top);
    xform.m[14] = near / nf;
    xform.m[15] = 1.0;

    return xform;
}

Xform Xform::project_to_plane(const Plane& plane) {

    const Vector& n = plane.z_axis();
    const Point& o = plane.origin();
    const double nx = n[0];
    const double ny = n[1];
    const double nz = n[2];
    const double d = o[0] * nx + o[1] * ny + o[2] * nz;

    Xform xform;
    xform.m[0] = 1.0 - nx * nx;
    xform.m[4] = -nx * ny;
    xform.m[8] = -nx * nz;
    xform.m[12] = nx * d;
    xform.m[1] = -ny * nx;
    xform.m[5] = 1.0 - ny * ny;
    xform.m[9] = -ny * nz;
    xform.m[13] = ny * d;
    xform.m[2] = -nz * nx;
    xform.m[6] = -nz * ny;
    xform.m[10] = 1.0 - nz * nz;
    xform.m[14] = nz * d;

    return xform;
}

Xform Xform::project_to_plane_by_axis(const Plane& plane, const Vector& direction) {

    const Vector& n = plane.z_axis();
    const Point& o = plane.origin();
    const double nx = n[0];
    const double ny = n[1];
    const double nz = n[2];
    const double dx = direction[0];
    const double dy = direction[1];
    const double dz = direction[2];
    const double s = 1.0 / (nx * dx + ny * dy + nz * dz);
    const double d = o[0] * nx + o[1] * ny + o[2] * nz;

    Xform xform;
    xform.m[0] = 1.0 - dx * s * nx;
    xform.m[4] = -dx * s * ny;
    xform.m[8] = -dx * s * nz;
    xform.m[12] = dx * s * d;
    xform.m[1] = -dy * s * nx;
    xform.m[5] = 1.0 - dy * s * ny;
    xform.m[9] = -dy * s * nz;
    xform.m[13] = dy * s * d;
    xform.m[2] = -dz * s * nx;
    xform.m[6] = -dz * s * ny;
    xform.m[10] = 1.0 - dz * s * nz;
    xform.m[14] = dz * s * d;

    return xform;
}

// ═══════════════════════════════════════════════════════════════════════════
// Apply transformations
// ═══════════════════════════════════════════════════════════════════════════
Point Xform::transform_point(const Point& p) const {

    const double x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
    const double y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
    const double z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14];
    const double w = m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15];

    if (std::abs(w) < 1e-12)
        return Point(x, y, z);

    return Point(x / w, y / w, z / w);
}

Vector Xform::transform_vector(const Vector& v) const {

    const double x = m[0] * v[0] + m[4] * v[1] + m[8] * v[2];
    const double y = m[1] * v[0] + m[5] * v[1] + m[9] * v[2];
    const double z = m[2] * v[0] + m[6] * v[1] + m[10] * v[2];

    return Vector(x, y, z);
}

// ═══════════════════════════════════════════════════════════════════════════
// Details
// ═══════════════════════════════════════════════════════════════════════════
std::optional<Xform> Xform::inverse() const {

    const double s0 = m[0] * m[5] - m[1] * m[4];
    const double s1 = m[0] * m[9] - m[1] * m[8];
    const double s2 = m[0] * m[13] - m[1] * m[12];
    const double s3 = m[4] * m[9] - m[5] * m[8];
    const double s4 = m[4] * m[13] - m[5] * m[12];
    const double s5 = m[8] * m[13] - m[9] * m[12];
    const double c5 = m[10] * m[15] - m[11] * m[14];
    const double c4 = m[6] * m[15] - m[7] * m[14];
    const double c3 = m[6] * m[11] - m[7] * m[10];
    const double c2 = m[2] * m[15] - m[3] * m[14];
    const double c1 = m[2] * m[11] - m[3] * m[10];
    const double c0 = m[2] * m[7] - m[3] * m[6];
    const double det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;

    if (std::abs(det) < 1e-12)
        return std::nullopt;

    const double inv_det = 1.0 / det;

    Xform result;
    result.m[0] = (m[5] * c5 - m[9] * c4 + m[13] * c3) * inv_det;
    result.m[4] = (-m[4] * c5 + m[8] * c4 - m[12] * c3) * inv_det;
    result.m[8] = (m[7] * s5 - m[11] * s4 + m[15] * s3) * inv_det;
    result.m[12] = (-m[6] * s5 + m[10] * s4 - m[14] * s3) * inv_det;
    result.m[1] = (-m[1] * c5 + m[9] * c2 - m[13] * c1) * inv_det;
    result.m[5] = (m[0] * c5 - m[8] * c2 + m[12] * c1) * inv_det;
    result.m[9] = (-m[3] * s5 + m[11] * s2 - m[15] * s1) * inv_det;
    result.m[13] = (m[2] * s5 - m[10] * s2 + m[14] * s1) * inv_det;
    result.m[2] = (m[1] * c4 - m[5] * c2 + m[13] * c0) * inv_det;
    result.m[6] = (-m[0] * c4 + m[4] * c2 - m[12] * c0) * inv_det;
    result.m[10] = (m[3] * s4 - m[7] * s2 + m[15] * s0) * inv_det;
    result.m[14] = (-m[2] * s4 + m[6] * s2 - m[14] * s0) * inv_det;
    result.m[3] = (-m[1] * c3 + m[5] * c1 - m[9] * c0) * inv_det;
    result.m[7] = (m[0] * c3 - m[4] * c1 + m[8] * c0) * inv_det;
    result.m[11] = (-m[3] * s3 + m[7] * s1 - m[11] * s0) * inv_det;
    result.m[15] = (m[2] * s3 - m[6] * s1 + m[10] * s0) * inv_det;

    return result;
}

bool Xform::is_identity() const {
    return *this == Xform();
}

std::array<std::array<double, 4>, 4> Xform::to_cols() const {

    return {{
        {m[0], m[1], m[2], m[3]},
        {m[4], m[5], m[6], m[7]},
        {m[8], m[9], m[10], m[11]},
        {m[12], m[13], m[14], m[15]},
    }};
}

double Xform::uniform_scale() const {
    return std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
}

Point Xform::eye() const {

    const std::array<std::array<double, 3>, 3> rows = {{
        {m[0], m[4], m[8]},
        {m[1], m[5], m[9]},
        {m[3], m[7], m[11]},
    }};
    const std::array<double, 3> rhs = {-m[12], -m[13], -m[15]};
    const double d = det3(rows);
    double norm = 1.0;

    for (const std::array<double, 3>& row : rows)
        norm *= std::sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]);

    if (std::abs(d) <= 1e-9 * std::max(norm, 1e-30)) {
        const double length = std::max(std::sqrt(m[2] * m[2] + m[6] * m[6] + m[10] * m[10]), 1e-30);

        return Point(m[2] / length * 1.0e9, m[6] / length * 1.0e9, m[10] / length * 1.0e9);
    }

    std::array<double, 3> eye = {0.0, 0.0, 0.0};

    for (int k = 0; k < 3; k++) {
        std::array<std::array<double, 3>, 3> replaced = rows;

        for (int row = 0; row < 3; row++)
            replaced[row][k] = rhs[row];

        eye[k] = det3(replaced) / d;
    }

    return Point(eye[0], eye[1], eye[2]);
}

double Xform::ortho_half_height() const {

    const double w2 = m[3] * m[3] + m[7] * m[7] + m[11] * m[11];

    if (w2 > 1e-12)
        return 0.0;

    const double r1 = m[1] * m[1] + m[5] * m[5] + m[9] * m[9];

    if (r1 <= 1e-30)
        return 0.0;

    return 1.0 / std::sqrt(r1);
}

double Xform::det3(const std::array<std::array<double, 3>, 3>& rows) {
    return rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Xform::jsondump() const {

    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["m"] = m;
    data["name"] = name;
    data["type"] = "Xform";

    return data;
}

Xform Xform::jsonload(const nlohmann::json& data) {

    Xform xform(data["m"].get<std::array<double, 16>>());
    xform.guid() = data["guid"].get<std::string>();
    xform.name = data["name"].get<std::string>();

    return xform;
}

std::string Xform::file_json_dumps() const {
    return jsondump().dump();
}

Xform Xform::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::json::parse(json_string));
}

void Xform::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Xform Xform::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Xform Xform::to_proto() const {

    session_proto::Xform proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);

    for (int i = 0; i < 16; i++)
        proto.add_matrix(m[i]);

    return proto;
}

Xform Xform::from_proto(const session_proto::Xform& proto) {

    Xform xform;

    if (!proto.guid().empty())
        xform.guid() = proto.guid();

    xform.name = proto.name();

    for (int i = 0; i < 16 && i < proto.matrix_size(); i++)
        xform.m[i] = proto.matrix(i);

    return xform;
}

std::string Xform::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Xform Xform::pb_loads(const std::string& data) {

    session_proto::Xform proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Xform protobuf data");

    return from_proto(proto);
}

void Xform::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Xform Xform::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Xform::str() const {

    std::string rows;

    for (int i = 0; i < 4; i++) {
        rows += fmt::format("[{:.6f}, {:.6f}, {:.6f}, {:.6f}]", m[i], m[4 + i], m[8 + i], m[12 + i]);

        if (i < 3)
            rows += "\n";
    }

    return rows;
}

std::string Xform::repr() const {
    return fmt::format("Xform({}, {})", name, guid().substr(0, 8));
}

} // namespace session_cpp
