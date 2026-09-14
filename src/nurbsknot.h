#pragma once
#include <limits>
#include <utility>
#include <vector>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Knot styles
// ═══════════════════════════════════════════════════════════════════════════

/** @enum CurveNurbsKnotStyle
 *  @brief Parameter spacing for interpolated curves.
 */
enum class CurveNurbsKnotStyle {
    Uniform = 0,                ///< Equal parameter spacing.
    Chord = 1,                  ///< Spacing proportional to chord length.
    ChordSquareRoot = 2,        ///< Spacing proportional to the square root of chord length.
    UniformPeriodic = 3,        ///< Equal spacing for a periodic curve.
    ChordPeriodic = 4,          ///< Chord-length spacing for a periodic curve.
    ChordSquareRootPeriodic = 5 ///< Square-root chord spacing for a periodic curve.
};

/** @enum CurveInterpStyle
 *  @brief End-tangent estimate for cubic curve interpolation.
 */
enum class CurveInterpStyle {
    Rhino = 0, ///< Bessel end tangents matching Rhino.
    Occt = 1   ///< Cubic Lagrange end tangents matching OCCT.
};

namespace nurbsknot {

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════

/// Return the number of nurbsknots for an order and control-point count.
inline int nurbsknot_count(int order, int cv_count) {
    if (order < 2 || cv_count < order) return 0;
    const long long count = static_cast<long long>(order) + cv_count - 2;
    return count <= std::numeric_limits<int>::max() ? static_cast<int>(count) : 0;
}

/// Return the floating-point tolerance associated with the domain interval [a,
/// b].
double domain_tolerance(double a, double b);

/// Return a clamped uniform nurbsknot vector, or an empty vector for invalid
/// arguments.
std::vector<double> make_clamped_uniform(int order, int cv_count, double delta = 1.0);

/// Return a periodic uniform nurbsknot vector, or an empty vector for invalid
/// arguments.
std::vector<double> make_periodic_uniform(int order, int cv_count, double delta = 1.0);

/// Clamp the selected ends in place, where end is 0 for left, 1 for right, or 2
/// for both.
bool clamp(int order, int cv_count, std::vector<double>& nurbsknot, int end = 2);

// ═══════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════

/// Return whether the vector has the required length, finite values, and valid
/// spans.
bool is_valid(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Return whether the selected ends contain order - 1 equal nurbsknots.
bool is_clamped(int order, int cv_count, const std::vector<double>& nurbsknot, int end = 2);

/// Return whether the nurbsknot vector has finite, positive, uniform spacing.
bool is_periodic(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Return the finite domain endpoints, or (0, 0) when the required entries
/// cannot be read.
std::pair<double, double> get_domain(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Rescale the nurbsknot vector in place to the finite domain [t0, t1].
bool set_domain(int order, int cv_count, std::vector<double>& nurbsknot, double t0, double t1);

/// Reverse a finite nurbsknot vector in place while preserving its domain.
bool reverse(int order, int cv_count, std::vector<double>& nurbsknot);

/// Return the multiplicity at nurbsknot_index, or zero for invalid arguments.
int multiplicity(int order, int cv_count, const std::vector<double>& nurbsknot,
                 int nurbsknot_index);

/// Return the number of non-empty spans, or zero for invalid arguments.
int span_count(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Return the index of the span containing finite parameter t.
///
/// The nurbsknot vector must be valid and nondecreasing; the search checks only
/// the endpoints and binary-search entries that it reads.
int find_span(int order, int cv_count, const std::vector<double>& nurbsknot, double t, int side = 0,
              int hint = 0);

/// Return the Greville abscissae, or an empty vector for invalid arguments.
std::vector<double> get_greville_abcissae(int order, int cv_count,
                                          const std::vector<double>& nurbsknot,
                                          bool periodic = false);

// ═══════════════════════════════════════════════════════════════════════════
// Interpolation
// ═══════════════════════════════════════════════════════════════════════════

/// Solve a finite tridiagonal system with the Thomas algorithm, returning false
/// if invalid or singular.
bool solve_tridiagonal(int dim, int n, std::vector<double>& lower, std::vector<double>& diag,
                       std::vector<double>& upper, const std::vector<double>& rhs,
                       std::vector<double>& solution);

/// Return one parameter per point from a flat point_count by dim coordinate
/// array.
std::vector<double> compute_parameters(const double* points, int point_count, int dim,
                                       CurveNurbsKnotStyle style);

/// Return a clamped interpolation nurbsknot vector with natural end conditions.
std::vector<double> build_interp_nurbsknots(const std::vector<double>& params, int degree);

/// Return the order non-zero B-spline basis values at t using Cox-de Boor
/// evaluation. The local nurbsknot window read for span must be finite.
std::vector<double> eval_basis(int order, const std::vector<double>& nurbsknot, int span, double t);

// ═══════════════════════════════════════════════════════════════════════════
// Fitting
// ═══════════════════════════════════════════════════════════════════════════

/// Return a clamped fitting vector with denser nurbsknots where the points
/// turn.
std::vector<double> build_fitted_nurbsknots_adaptive(const std::vector<double>& params,
                                                     const double* points, int point_count, int dim,
                                                     int num_cvs, int degree, double scale = 3.0);

/// Return a periodic fitting vector with denser nurbsknots where the closed
/// points turn.
std::vector<double> build_fitted_nurbsknots_periodic_adaptive(const std::vector<double>& params,
                                                              const double* points, int n, int dim,
                                                              int num_cvs, int degree,
                                                              double scale = 3.0);

/// Solve a finite banded symmetric positive-definite system in place with
/// Cholesky factorization.
bool solve_banded_spd(int dim, int n, int half_bw, std::vector<double>& band,
                      std::vector<double>& rhs);

} // namespace nurbsknot

} // namespace session_cpp
