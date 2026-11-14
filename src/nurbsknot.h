#pragma once
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Knot styles
// ═══════════════════════════════════════════════════════════════════════════
/// Parameter spacing for interpolated curves.
enum class CurveNurbsKnotStyle {
    Uniform = 0,                // Equal parameter spacing.
    Chord = 1,                  // Spacing proportional to chord length.
    ChordSquareRoot = 2,        // Spacing proportional to the square root of chord length.
    UniformPeriodic = 3,        // Equal spacing for a periodic curve.
    ChordPeriodic = 4,          // Chord-length spacing for a periodic curve.
    ChordSquareRootPeriodic = 5 // Square-root chord spacing for a periodic curve.
};

/// End-tangent estimate for cubic curve interpolation.
enum class CurveInterpStyle {
    Rhino = 0, // Bessel end tangents matching Rhino.
    Occt = 1   // Cubic Lagrange end tangents matching OCCT.
};

namespace nurbsknot {

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════
/// Return the number of nurbsknots for an order and control-point count.
inline int nurbsknot_count(int order, int cv_count) {

    if (order < 2 || cv_count < order)
        return 0;

    const long long count = static_cast<long long>(order) + cv_count - 2;

    return count <= std::numeric_limits<int>::max() ? static_cast<int>(count) : 0;
}

/// Return the floating-point tolerance associated with the domain interval [a, b].
double domain_tolerance(double a, double b);

/// Return a clamped uniform nurbsknot vector, or an empty vector for invalid arguments.
std::vector<double> compute_clamped_uniform(int order, int cv_count, double delta = 1.0);

/// Return a periodic uniform nurbsknot vector, or an empty vector for invalid arguments.
std::vector<double> compute_periodic_uniform(int order, int cv_count, double delta = 1.0);

/// Clamp the selected ends in place, where end is 0 for left, 1 for right, or 2 for both.
bool clamp(int order, int cv_count, std::vector<double>& nurbsknot, int end = 2);

// ═══════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════
/// Return whether the vector has the required length, finite values, and valid spans.
bool is_valid(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Return whether the selected ends contain order - 1 equal nurbsknots.
bool is_clamped(int order, int cv_count, const std::vector<double>& nurbsknot, int end = 2);

/// Return whether the nurbsknot vector has finite, positive, uniform spacing.
bool is_periodic(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Return the finite domain endpoints, or (0, 0) when the required entries cannot be read.
std::pair<double, double> get_domain(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Rescale the nurbsknot vector in place to the finite domain [t0, t1].
bool set_domain(int order, int cv_count, std::vector<double>& nurbsknot, double t0, double t1);

/// Reverse a finite nurbsknot vector in place while preserving its domain.
bool reverse(int order, int cv_count, std::vector<double>& nurbsknot);

/// Return the multiplicity at nurbsknot_index, or zero for invalid arguments.
int multiplicity(int order, int cv_count, const std::vector<double>& nurbsknot, int nurbsknot_index);

/// Return the number of non-empty spans, or zero for invalid arguments.
int span_count(int order, int cv_count, const std::vector<double>& nurbsknot);

/// Return the index of the span containing finite parameter t in a valid nondecreasing nurbsknot vector.
int find_span(int order, int cv_count, const std::vector<double>& nurbsknot, double t, int side = 0, int hint = 0);

/// Return the Greville abscissae, or an empty vector for invalid arguments.
std::vector<double> get_greville_abcissae(int order, int cv_count, const std::vector<double>& nurbsknot, bool periodic = false);

// ═══════════════════════════════════════════════════════════════════════════
// Interpolation
// ═══════════════════════════════════════════════════════════════════════════
/// Solve a finite tridiagonal system with the Thomas algorithm, returning false if invalid or singular.
bool solve_tridiagonal(int dim, int n, std::vector<double>& lower, std::vector<double>& diag, std::vector<double>& upper, const std::vector<double>& rhs, std::vector<double>& solution);

/// Return one parameter per point from a flat point_count by dim coordinate array.
std::vector<double> compute_parameters(const double* points, int point_count, int dim, CurveNurbsKnotStyle style);

/// Return a clamped interpolation nurbsknot vector with natural end conditions.
std::vector<double> build_interp_nurbsknots(const std::vector<double>& params, int degree);

/// Return the order non-zero B-spline basis values at t by Cox-de Boor evaluation over a finite span window.
std::vector<double> eval_basis(int order, const std::vector<double>& nurbsknot, int span, double t);

/// Working storage of count values for one evaluation: in place and unset up to N, on the heap above it.
template <typename T, std::size_t N>
struct Scratch {
    alignas(T) std::byte local[N * sizeof(T)];
    std::vector<T> heap;
    T* data;

    explicit Scratch(std::size_t count) : data(reinterpret_cast<T*>(local)) {
        if (count > N) {
            heap.resize(count);
            data = heap.data();
        }
    }
    Scratch(const Scratch&) = delete;
    Scratch& operator=(const Scratch&) = delete;
};

/// Whether every one of count values is finite: a finite value times zero is zero, any other is not a number, and so is a sum that holds one.
inline bool all_finite(const double* values, std::size_t count) {
    double sum = 0.0;
    std::size_t i = 0;
    if (count >= 16) {
        double part[4] = {0.0, 0.0, 0.0, 0.0};
        for (; i + 4 <= count; i += 4)
            for (int k = 0; k < 4; ++k)
                part[k] += values[i + k] * 0.0;
        sum = part[0] + part[1] + part[2] + part[3];
    }
    for (; i < count; ++i)
        sum += values[i] * 0.0;
    return sum == 0.0;
}

/// The arithmetic of basis_values for one order; an order FIXED above zero is known at compile time, so its small loops unroll.
template <int FIXED>
inline void basis_values_of(int order, const double* knot, double t, double* values, double* scratch) {

    [[maybe_unused]] double fixed[FIXED ? 2 * FIXED : 1];
    double* left = scratch;

    if constexpr (FIXED != 0) {
        order = FIXED;
        left = fixed;
    }

    double* right = left + order;
    values[0] = 1.0;

    for (int j = 1; j < order; j++) {
        left[j] = t - knot[order - 1 - j];
        right[j] = knot[order - 2 + j] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            const double denom = right[r + 1] + left[j - r];
            const double temp = denom != 0.0 ? values[r] / denom : 0.0;
            values[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        values[j] = saved;
    }
}

/// Write the order non-zero basis values at t, the arithmetic of eval_basis without allocation: knot points at the first of the 2 * order - 2 knots of the span, scratch holds 2 * order doubles.
inline void basis_values(int order, const double* knot, double t, double* values, double* scratch) {

    switch (order) {
        case 2: return basis_values_of<2>(order, knot, t, values, scratch);
        case 3: return basis_values_of<3>(order, knot, t, values, scratch);
        case 4: return basis_values_of<4>(order, knot, t, values, scratch);
        case 6: return basis_values_of<6>(order, knot, t, values, scratch);
        default: return basis_values_of<0>(order, knot, t, values, scratch);
    }
}

/// The arithmetic of basis_derivatives for one order; an order FIXED above zero is known at compile time, so its small loops unroll.
template <int FIXED>
inline void basis_derivatives_of(int order, const double* knot, double t, int derivative_count, double* values, double* scratch) {

    [[maybe_unused]] double fixed[FIXED ? FIXED * (FIXED + 4) : 1];
    double* ndu = scratch;

    if constexpr (FIXED != 0) {
        order = FIXED;
        ndu = fixed;
    }

    const int degree = order - 1;
    double* left = ndu + order * order;
    double* right = left + order;
    double* a = right + order;
    ndu[0] = 1.0;

    for (int j = 1; j <= degree; j++) {
        left[j] = t - knot[degree - j];
        right[j] = knot[degree + j - 1] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            ndu[j * order + r] = right[r + 1] + left[j - r];
            const double temp = ndu[r * order + j - 1] / ndu[j * order + r];
            ndu[r * order + j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        ndu[j * order + j] = saved;
    }

    for (int j = 0; j <= degree; j++)
        values[j] = ndu[j * order + degree];

    for (int j = 0; j < 2 * order; j++)
        a[j] = 0.0;

    for (int r = 0; r <= degree; r++) {
        int s1 = 0;
        int s2 = order;
        a[0] = 1.0;

        for (int k = 1; k <= derivative_count; k++) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = degree - k;

            if (r >= k) {
                a[s2] = a[s1] / ndu[(pk + 1) * order + rk];
                d = a[s2] * ndu[rk * order + pk];
            }

            const int j1 = rk >= -1 ? 1 : -rk;
            const int j2 = r - 1 <= pk ? k - 1 : degree - r;

            for (int j = j1; j <= j2; j++) {
                a[s2 + j] = (a[s1 + j] - a[s1 + j - 1]) / ndu[(pk + 1) * order + rk + j];
                d += a[s2 + j] * ndu[(rk + j) * order + pk];
            }

            if (r <= pk) {
                a[s2 + k] = -a[s1 + k - 1] / ndu[(pk + 1) * order + r];
                d += a[s2 + k] * ndu[r * order + pk];
            }

            values[k * order + r] = d;
            const int swapped = s1;
            s1 = s2;
            s2 = swapped;
        }
    }

    double factor = degree;

    for (int k = 1; k <= derivative_count; k++) {
        for (int j = 0; j <= degree; j++)
            values[k * order + j] *= factor;

        factor *= degree - k;
    }
}

/// Write the basis derivatives values[k * order + j] for k up to derivative_count without allocation (Piegl & Tiller A2.3): knot points at the first of the 2 * order - 2 knots of the span, scratch holds order * (order + 4) doubles.
inline void basis_derivatives(int order, const double* knot, double t, int derivative_count, double* values, double* scratch) {

    switch (order) {
        case 2: return basis_derivatives_of<2>(order, knot, t, derivative_count, values, scratch);
        case 3: return basis_derivatives_of<3>(order, knot, t, derivative_count, values, scratch);
        case 4: return basis_derivatives_of<4>(order, knot, t, derivative_count, values, scratch);
        case 6: return basis_derivatives_of<6>(order, knot, t, derivative_count, values, scratch);
        default: return basis_derivatives_of<0>(order, knot, t, derivative_count, values, scratch);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Fitting
// ═══════════════════════════════════════════════════════════════════════════
/// Return a clamped fitting vector with denser nurbsknots where the points turn.
std::vector<double> build_fitted_nurbsknots_adaptive(const std::vector<double>& params, const double* points, int point_count, int dim, int num_cvs, int degree, double scale = 3.0);

/// Return a periodic fitting vector with denser nurbsknots where the closed points turn.
std::vector<double> build_fitted_nurbsknots_periodic_adaptive(const std::vector<double>& params, const double* points, int n, int dim, int num_cvs, int degree, double scale = 3.0);

/// Solve a finite banded symmetric positive-definite system in place with Cholesky factorization.
bool solve_banded_spd(int dim, int n, int half_bw, std::vector<double>& band, std::vector<double>& rhs);

} // namespace nurbsknot

} // namespace session_cpp
