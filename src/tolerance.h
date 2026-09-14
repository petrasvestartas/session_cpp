#pragma once
#include "json.h"
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace session_proto {
class Tolerance;
}

namespace session_cpp {

class Point;
class Vector;
class ToleranceGuard;

constexpr double SCALE = 1e6; ///< Default coordinate-key scale.

/** @class Tolerance
 * @brief Tolerance settings for geometric comparisons.
 */
class Tolerance {
public:
    static constexpr double PI = 3.14159265358979323846; ///< Circle constant.
    static constexpr double TWO_PI = 2.0 * PI;           ///< Full turn in radians.
    static constexpr double HALF_PI = PI / 2.0;          ///< Quarter turn in radians.
    static constexpr double TO_DEGREES = 180.0 / PI;     ///< Radian-to-degree factor.
    static constexpr double TO_RADIANS = PI / 180.0;     ///< Degree-to-radian factor.

    static constexpr double ABSOLUTE = 1e-9;              ///< Default absolute tolerance.
    static constexpr double RELATIVE = 1e-6;              ///< Default relative tolerance.
    static constexpr double ANGULAR = 1e-6;               ///< Default angular tolerance.
    static constexpr double APPROXIMATION = 1e-3;         ///< Default approximation tolerance.
    static constexpr int PRECISION = 3;                   ///< Default decimal precision.
    static constexpr double LINEARDEFLECTION = 1e-3;      ///< Default linear deflection.
    static constexpr double ANGULARDEFLECTION = 1e-1;     ///< Default angular deflection.
    static constexpr double ANGLE_TOLERANCE_DEGREES = 0.11; ///< Angular tolerance in degrees.
    /// Used heavily by algorithms; do not change
    static constexpr double ZERO_TOLERANCE = 1e-12;
    static constexpr int ROUNDING = 6; ///< Default coordinate-key rounding.

private:
    std::string _unit;
    std::optional<double> _absolute;
    std::optional<double> _relative;
    std::optional<double> _angular;
    std::optional<double> _approximation;
    std::optional<int> _precision;
    std::optional<double> _lineardeflection;
    std::optional<double> _angulardeflection;

public:
    /// Construct tolerance with a unit system ("M" or "MM")
    explicit Tolerance(const std::string& unit = "M");

    /// Reset all overrides to default constants
    void reset();

    /// Current unit system
    std::string unit() const { return _unit; }
    /// Absolute tolerance value (or default ABSOLUTE)
    double absolute() const { return _absolute.value_or(ABSOLUTE); }
    /// Relative tolerance value (or default RELATIVE)
    double relative() const { return _relative.value_or(RELATIVE); }
    /// Angular tolerance value in radians (or default ANGULAR)
    double angular() const { return _angular.value_or(ANGULAR); }
    /// Approximation tolerance (or default APPROXIMATION)
    double approximation() const { return _approximation.value_or(APPROXIMATION); }
    /// Decimal precision used for formatting (or default PRECISION)
    int precision() const { return _precision.value_or(PRECISION); }
    /// Linear deflection value (or default LINEARDEFLECTION)
    double lineardeflection() const { return _lineardeflection.value_or(LINEARDEFLECTION); }
    /// Angular deflection value (or default ANGULARDEFLECTION)
    double angulardeflection() const { return _angulardeflection.value_or(ANGULARDEFLECTION); }

    /// Set current unit system
    void set_unit(const std::string& value);
    /// Override absolute tolerance
    void set_absolute(double value);
    /// Override relative tolerance
    void set_relative(double value);
    /// Override angular tolerance (radians)
    void set_angular(double value);
    /// Override approximation tolerance
    void set_approximation(double value);
    /// Override decimal precision for formatting
    void set_precision(int value);
    /// Override linear deflection
    void set_lineardeflection(double value);
    /// Override angular deflection
    void set_angulardeflection(double value);

    /// Compute combined tolerance from relative and absolute components
    double tolerance(double truevalue, double rtol, double atol) const;
    /// Compare two values within tolerance
    bool compare(double a, double b, double rtol, double atol) const;
    /// Check if value is within zero tolerance
    bool is_zero(double a) const;
    /// Check if value is positive within tolerance
    bool is_positive(double a) const;
    /// Check if value is negative within tolerance
    bool is_negative(double a) const;
    /// Check if value is within a range with absolute tolerance
    bool is_between(double value, double minval, double maxval) const;
    /// Check closeness between two values using rtol/atol
    bool is_close(double a, double b) const;
    /// Check if an angle is effectively zero (radians)
    bool is_angle_zero(double a) const;
    /// Check if two angles are close (radians)
    bool is_angles_close(double a, double b) const;
    /// Check if two 3D points are equal within absolute tolerance
    bool is_point_close(const Point& a, const Point& b) const;
    /// Check if two 3D vectors are equal within absolute tolerance
    bool is_vector_close(const Vector& a, const Vector& b) const;
    /// Check if two lists of values are element-wise close
    bool is_allclose(const std::vector<double>& a, const std::vector<double>& b) const;

    /// Create a RAII guard that restores tolerance on destruction
    ToleranceGuard temporary();

    /// Create a geometric key string for 3D point with optional precision
    std::string key(double x, double y, double z, int precision = -999) const;
    /// Create a geometric key string for 2D point with optional precision
    std::string key_xy(double x, double y, int precision = -999) const;
    /// Format a number with optional precision override
    std::string format_number(double number, int precision = -999) const;
    /// Determine decimal precision from a tolerance value
    int precision_from_tolerance(double tol = -1) const;

    /// Serialize to an ordered JSON object
    nlohmann::ordered_json jsondump() const;
    /// Deserialize from a JSON object
    static Tolerance jsonload(const nlohmann::json& data);
    /// Serialize to a JSON string
    std::string file_json_dumps() const;
    /// Deserialize from a JSON string
    static Tolerance file_json_loads(const std::string& json_string);
    /// Write JSON to a file
    void file_json_dump(const std::string& filename) const;
    /// Read JSON from a file
    static Tolerance file_json_load(const std::string& filename);

    /// Convert to the protobuf message
    session_proto::Tolerance to_proto() const;
    /// Construct from the protobuf message
    static Tolerance from_proto(const session_proto::Tolerance& proto);
    /// Serialize to protobuf bytes
    std::string pb_dumps() const;
    /// Deserialize from protobuf bytes
    static Tolerance pb_loads(const std::string& data);
    /// Write protobuf bytes to a file
    void pb_dump(const std::string& filename) const;
    /// Read protobuf bytes from a file
    static Tolerance pb_load(const std::string& filename);

    /// Convert degrees to radians
    static double to_radians(double degrees) { return degrees * TO_RADIANS; }
    /// Convert radians to degrees
    static double to_degrees(double radians) { return radians * TO_DEGREES; }
    /// Round a value to a given number of decimal places
    static double round_to(double value, int ndigits);
};

/** @class ToleranceGuard
 * @brief RAII guard that restores Tolerance state on destruction.
 */
class ToleranceGuard {
    Tolerance* _target;
    Tolerance _saved;
public:
    /// Save the target's current settings.
    explicit ToleranceGuard(Tolerance& target) : _target(&target), _saved(target) {}
    /// Restore the saved settings.
    ~ToleranceGuard() { if (_target) *_target = _saved; }
    /// Guards cannot be copied.
    ToleranceGuard(const ToleranceGuard&) = delete;
    /// Guards cannot be copy-assigned.
    ToleranceGuard& operator=(const ToleranceGuard&) = delete;
    /// Transfer restoration responsibility.
    ToleranceGuard(ToleranceGuard&& other) noexcept
        : _target(std::exchange(other._target, nullptr)), _saved(std::move(other._saved)) {}
    /// Guards cannot be move-assigned.
    ToleranceGuard& operator=(ToleranceGuard&&) = delete;

    /// Access the guarded tolerance.
    Tolerance& operator*() { return *_target; }
    /// Access the guarded tolerance.
    Tolerance* operator->() { return _target; }
};

/// Global tolerance instance
extern Tolerance TOLERANCE;

// ═══════════════════════════════════════════════════════════════════════════
// Utilities
// ═══════════════════════════════════════════════════════════════════════════

/// Check if a number is finite
bool is_finite(double x);
/// Order-independent key from two ints: larger in the high 32 bits
uint64_t unique_from_two_int(int a, int b);
/// Signed modulo into [0, n-1]; 0 when n == 0
int wrap_index(int index, int n);
/// Opposite side of a right triangle: edge_length * tan(angle_deg)
double triangle_edge_by_angle(double edge_length, double angle_deg);
/// Convert radians to degrees
double rad_to_deg(double radians);
/// Convert degrees to radians
double deg_to_rad(double degrees);
/// Number of decimal digits of the integer part of |n|; 0 when |n| < 1
int count_digits(double n);

} // namespace session_cpp
