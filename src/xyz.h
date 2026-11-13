#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

namespace session_cpp {

/// Three doubles, a point or a direction inside the kernel: no name, guid or color, every operation inline.
struct xyz {
    double x, y, z;

    /// Return the mutable coordinate by index (0=x, 1=y, 2=z).
    double& operator[](int index) {
        return index == 0 ? x : index == 1 ? y : z;
    }

    /// Return the coordinate by index (0=x, 1=y, 2=z).
    const double& operator[](int index) const {
        return index == 0 ? x : index == 1 ? y : z;
    }

    /// Compare coordinates exactly.
    friend bool operator==(const xyz& a, const xyz& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    /// Add in place.
    xyz& operator+=(const xyz& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /// Subtract in place.
    xyz& operator-=(const xyz& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    /// Scale in place.
    xyz& operator*=(double factor) {
        x *= factor;
        y *= factor;
        z *= factor;
        return *this;
    }

    /// Divide in place.
    xyz& operator/=(double factor) {
        x /= factor;
        y /= factor;
        z /= factor;
        return *this;
    }

    /// Return the sum.
    friend xyz operator+(const xyz& a, const xyz& b) {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    /// Return the difference.
    friend xyz operator-(const xyz& a, const xyz& b) {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    /// Return the negation.
    friend xyz operator-(const xyz& a) {
        return {-a.x, -a.y, -a.z};
    }

    /// Return a scaled copy.
    friend xyz operator*(const xyz& a, double factor) {
        return {a.x * factor, a.y * factor, a.z * factor};
    }

    /// Return a copy scaled by a factor on the left.
    friend xyz operator*(double factor, const xyz& a) {
        return {a.x * factor, a.y * factor, a.z * factor};
    }

    /// Return a divided copy.
    friend xyz operator/(const xyz& a, double factor) {
        return {a.x / factor, a.y / factor, a.z / factor};
    }

    /// Return the dot product.
    double dot(const xyz& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    /// Return the cross product.
    xyz cross(const xyz& other) const {
        return {y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x};
    }

    /// Return the squared length without the square root.
    double length_squared() const {
        return x * x + y * y + z * z;
    }

    /// Return the length scaled to stay finite for large coordinates, the value of Vector::magnitude.
    double length() const {
        const double scale = std::max({std::abs(x), std::abs(y), std::abs(z)});
        if (scale > 0.0 && std::isfinite(scale))
            return scale * std::hypot(x / scale, y / scale, z / scale);
        return std::hypot(x, y, z);
    }

    /// Return the distance to other scaled to stay finite for large coordinates, the value of Point::distance.
    double distance(const xyz& other, double double_min = 1e-12) const {
        double dx = std::abs(x - other.x);
        double dy = std::abs(y - other.y);
        double dz = std::abs(z - other.z);

        if (dy >= dx && dy >= dz)
            std::swap(dx, dy);
        else if (dz >= dx && dz >= dy)
            std::swap(dx, dz);

        if (dx > double_min) {
            dy /= dx;
            dz /= dx;

            return dx * std::sqrt(1.0 + dy * dy + dz * dz);
        }

        if (dx > 0.0 && std::isfinite(dx))
            return dx;

        return 0.0;
    }
};

/// Three floats, for display buffers.
struct xyzf {
    float x, y, z;

    /// Return the mutable coordinate by index (0=x, 1=y, 2=z).
    float& operator[](int index) {
        return index == 0 ? x : index == 1 ? y : z;
    }

    /// Return the coordinate by index (0=x, 1=y, 2=z).
    const float& operator[](int index) const {
        return index == 0 ? x : index == 1 ? y : z;
    }

    /// Compare coordinates exactly.
    friend bool operator==(const xyzf& a, const xyzf& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

/// Three 32-bit integers, for index buffers.
struct ijk {
    std::int32_t i, j, k;

    /// Return the mutable index by position (0=i, 1=j, 2=k).
    std::int32_t& operator[](int index) {
        return index == 0 ? i : index == 1 ? j : k;
    }

    /// Return the index by position (0=i, 1=j, 2=k).
    const std::int32_t& operator[](int index) const {
        return index == 0 ? i : index == 1 ? j : k;
    }

    /// Compare indices.
    friend bool operator==(const ijk& a, const ijk& b) {
        return a.i == b.i && a.j == b.j && a.k == b.k;
    }
};

static_assert(sizeof(xyz) == 24 && std::is_trivially_copyable_v<xyz> && std::is_standard_layout_v<xyz>);
static_assert(sizeof(xyzf) == 12 && std::is_trivially_copyable_v<xyzf> && std::is_standard_layout_v<xyzf>);
static_assert(sizeof(ijk) == 12 && std::is_trivially_copyable_v<ijk> && std::is_standard_layout_v<ijk>);

/// Return single precision coordinates.
inline xyzf to_xyzf(const xyz& value) {
    return {static_cast<float>(value.x), static_cast<float>(value.y), static_cast<float>(value.z)};
}

/// Return the 3N doubles of N triples in place, x0 y0 z0 x1 y1 z1 ..., without copying.
inline std::span<const double> flat(const std::vector<xyz>& values) {
    return {values.empty() ? nullptr : &values.front().x, values.size() * 3};
}

/// Return the 3N mutable doubles of N triples in place, without copying.
inline std::span<double> flat(std::vector<xyz>& values) {
    return {values.empty() ? nullptr : &values.front().x, values.size() * 3};
}

/// Return the 3N floats of N triples in place, without copying.
inline std::span<const float> flat(const std::vector<xyzf>& values) {
    return {values.empty() ? nullptr : &values.front().x, values.size() * 3};
}

/// Return the 3N indices of N triples in place, without copying.
inline std::span<const std::int32_t> flat(const std::vector<ijk>& values) {
    return {values.empty() ? nullptr : &values.front().i, values.size() * 3};
}

} // namespace session_cpp
