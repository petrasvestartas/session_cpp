#include "nurbsknot.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace session_cpp {
namespace nurbsknot {

constexpr double KNOT_TOLERANCE = Tolerance::ABSOLUTE / 10.0;
constexpr double PIVOT_TOLERANCE = Tolerance::ZERO_TOLERANCE / 100.0;
constexpr double POSITIVE_DEFINITE_TOLERANCE =
    Tolerance::ABSOLUTE * Tolerance::ABSOLUTE * Tolerance::ZERO_TOLERANCE;

static bool are_finite(const std::vector<double>& values, std::size_t count) {

    if (values.size() < count)
        return false;

    for (std::size_t i = 0; i < count; i++)
        if (!std::isfinite(values[i]))
            return false;

    return true;
}

static bool are_finite(const double* values, std::size_t count) {

    if (!values)
        return false;

    for (std::size_t i = 0; i < count; i++)
        if (!std::isfinite(values[i]))
            return false;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════

double domain_tolerance(double a, double b) {

    if (a == b)
        return 0.0;

    const double epsilon = std::numeric_limits<double>::epsilon();
    const double tol = (std::fabs(a) + std::fabs(b) + std::fabs(a - b)) * std::sqrt(epsilon);

    return tol < epsilon ? epsilon : tol;
}

std::vector<double> make_clamped_uniform(int order, int cv_count, double delta) {

    if (order < 2 || cv_count < order || !std::isfinite(delta) || delta <= 0.0)
        return std::vector<double>();

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0)
        return std::vector<double>();

    std::vector<double> nurbsknot(kc, 0.0);

    double k = 0.0;

    for (int i = order - 2; i < cv_count; i++) {
        nurbsknot[i] = k;
        k += delta;
    }

    clamp(order, cv_count, nurbsknot, 2);

    return nurbsknot;
}

std::vector<double> make_periodic_uniform(int order, int cv_count, double delta) {

    if (order < 2 || cv_count < order || !std::isfinite(delta) || delta <= 0.0)
        return std::vector<double>();

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0)
        return std::vector<double>();

    std::vector<double> nurbsknot(kc, 0.0);

    double k = 0.0;

    for (int i = 0; i < kc; i++) {
        nurbsknot[i] = k;
        k += delta;
    }

    return nurbsknot;
}

bool clamp(int order, int cv_count, std::vector<double>& nurbsknot, int end) {

    if (order < 2 || cv_count < order || end < 0 || end > 2)
        return false;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return false;

    if (end == 0 || end == 2) {
        const double clamp_value = nurbsknot[order - 2];

        for (int i = 0; i < order - 2; i++)
            nurbsknot[i] = clamp_value;
    }

    if (end == 1 || end == 2) {
        const double clamp_value = nurbsknot[cv_count - 1];

        for (int i = cv_count; i < kc; i++)
            nurbsknot[i] = clamp_value;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════

bool is_valid(int order, int cv_count, const std::vector<double>& nurbsknot) {

    if (order < 2 || cv_count < order)
        return false;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return false;

    for (int i = 1; i < kc; i++)
        if (nurbsknot[i] < nurbsknot[i - 1])
            return false;

    for (int i = 0; i < kc - order + 1; i++)
        if (nurbsknot[i] >= nurbsknot[i + order - 1])
            return false;

    return true;
}

bool is_clamped(int order, int cv_count, const std::vector<double>& nurbsknot, int end) {

    if (order < 2 || cv_count < order || end < 0 || end > 2)
        return false;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return false;

    const int mult = order - 1;
    const double tol = KNOT_TOLERANCE;

    if (end == 0 || end == 2) {
        if (mult > kc)
            return false;

        const double start_value = nurbsknot[0];

        for (int i = 1; i < mult; i++)
            if (std::fabs(nurbsknot[i] - start_value) > tol)
                return false;
    }

    if (end == 1 || end == 2) {
        if (mult > kc)
            return false;

        const double end_value = nurbsknot[kc - 1];

        for (int i = 1; i < mult; i++)
            if (std::fabs(nurbsknot[kc - 1 - i] - end_value) > tol)
                return false;
    }

    return true;
}

bool is_periodic(int order, int cv_count, const std::vector<double>& nurbsknot) {

    if (order < 2 || cv_count < order)
        return false;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc < 2 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return false;

    const double delta = nurbsknot[1] - nurbsknot[0];

    if (delta <= 0)
        return false;

    const double tol = KNOT_TOLERANCE;

    for (int i = 2; i < kc; i++)
        if (std::fabs((nurbsknot[i] - nurbsknot[i - 1]) - delta) > tol)
            return false;

    return true;
}

std::pair<double, double> get_domain(int order, int cv_count,
                                     const std::vector<double>& nurbsknot) {
    if (order < 2 || cv_count < order)
        return std::make_pair(0.0, 0.0);

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() < static_cast<std::size_t>(kc))
        return std::make_pair(0.0, 0.0);

    const double start = nurbsknot[order - 2];
    const double end = nurbsknot[cv_count - 1];

    if (!std::isfinite(start) || !std::isfinite(end))
        return std::make_pair(0.0, 0.0);

    return std::make_pair(start, end);
}

bool set_domain(int order, int cv_count, std::vector<double>& nurbsknot, double t0, double t1) {

    if (order < 2 || cv_count < order || !std::isfinite(t0) || !std::isfinite(t1) || t0 >= t1)
        return false;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return false;

    const auto [old_t0, old_t1] = get_domain(order, cv_count, nurbsknot);

    if (old_t1 <= old_t0)
        return false;

    const double scale = (t1 - t0) / (old_t1 - old_t0);

    for (int i = 0; i < kc; i++)
        nurbsknot[i] = t0 + (nurbsknot[i] - old_t0) * scale;

    return true;
}

bool reverse(int order, int cv_count, std::vector<double>& nurbsknot) {

    if (order < 2 || cv_count < order)
        return false;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return false;

    std::reverse(nurbsknot.begin(), nurbsknot.end());

    const double t0 = nurbsknot[0];
    const double t1 = nurbsknot[kc - 1];

    for (int i = 0; i < kc; i++)
        nurbsknot[i] = t0 + t1 - nurbsknot[i];

    return true;
}

int multiplicity(int order, int cv_count, const std::vector<double>& nurbsknot,
                 int nurbsknot_index) {
    if (order < 2 || cv_count < order)
        return 0;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || nurbsknot_index < 0 ||
        nurbsknot_index >= kc || !are_finite(nurbsknot, kc))

        return 0;

    const double nurbsknot_value = nurbsknot[nurbsknot_index];
    const double tol = PIVOT_TOLERANCE;
    int mult = 1;

    int i = nurbsknot_index - 1;

    while (i >= 0 && std::fabs(nurbsknot[i] - nurbsknot_value) < tol) {
        mult++;
        i--;
    }

    i = nurbsknot_index + 1;

    while (i < kc && std::fabs(nurbsknot[i] - nurbsknot_value) < tol) {
        mult++;
        i++;
    }

    return mult;
}

int span_count(int order, int cv_count, const std::vector<double>& nurbsknot) {

    if (order < 2 || cv_count < order)
        return 0;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return 0;

    const int d = order - 1;
    int count = 0;

    for (int i = 0; i < cv_count - order + 1; i++)
        if (nurbsknot[i + d - 1] < nurbsknot[i + d])
            count++;

    return count;
}

int find_span(int order, int cv_count, const std::vector<double>& nurbsknot, double t, int, int) {

    if (order < 2 || cv_count < order || !std::isfinite(t))
        return 0;

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc))
        return 0;

    const int nurbsknot_offset = order - 2;
    const int span_len = cv_count - order + 2;
    const double start = nurbsknot[nurbsknot_offset];
    const double end = nurbsknot[nurbsknot_offset + span_len - 1];

    if (!std::isfinite(start) || !std::isfinite(end))
        return 0;

    if (t <= start)
        return 0;

    if (t >= end)
        return span_len - 2;

    int low = 0;
    int high = span_len - 1;

    while (high > low + 1) {
        const int mid = low + (high - low) / 2;
        const double mid_value = nurbsknot[nurbsknot_offset + mid];

        if (!std::isfinite(mid_value))
            return 0;

        if (t < mid_value)
            high = mid;
        else
            low = mid;
    }

    return low;
}

std::vector<double> get_greville_abcissae(int order, int cv_count,
                                          const std::vector<double>& nurbsknot, bool periodic) {
    if (order < 2 || cv_count < order)
        return std::vector<double>();

    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0 || nurbsknot.size() != static_cast<std::size_t>(kc) || !are_finite(nurbsknot, kc))
        return std::vector<double>();

    const int d = order - 1;
    const int count = periodic ? cv_count - order + 1 : cv_count;
    std::vector<double> g(count);

    for (int i = 0; i < count; i++) {
        double sum = 0.0;

        for (int j = 0; j < d; j++)
            sum += nurbsknot[i + j];

        g[i] = sum / d;
    }

    return g;
}

// ═══════════════════════════════════════════════════════════════════════════
// Interpolation
// ═══════════════════════════════════════════════════════════════════════════

bool solve_tridiagonal(int dim, int n, std::vector<double>& lower, std::vector<double>& diag,
                       std::vector<double>& upper, const std::vector<double>& rhs,
                       std::vector<double>& solution) {
    if (n < 1 || dim < 1)
        return false;

    const std::size_t rhs_count = static_cast<std::size_t>(n) * static_cast<std::size_t>(dim);

    if (lower.size() < static_cast<std::size_t>(n) || diag.size() < static_cast<std::size_t>(n) ||
        upper.size() < static_cast<std::size_t>(n) || rhs.size() < rhs_count)

        return false;

    if (!are_finite(lower, n) || !are_finite(diag, n) || !are_finite(upper, n) ||
        !are_finite(rhs, rhs_count))

        return false;

    const double eps = PIVOT_TOLERANCE;
    std::vector<double> c_star(n);
    std::vector<double> d_star(rhs_count);
    solution.resize(rhs_count);

    if (std::fabs(diag[0]) < eps)
        return false;

    c_star[0] = upper[0] / diag[0];

    for (int d = 0; d < dim; d++)
        d_star[d] = rhs[d] / diag[0];

    for (int i = 1; i < n; i++) {
        const double denom = diag[i] - lower[i] * c_star[i - 1];

        if (std::fabs(denom) < eps)
            return false;

        c_star[i] = i < n - 1 ? upper[i] / denom : 0.0;

        for (int d = 0; d < dim; d++) {
            const std::size_t index = static_cast<std::size_t>(i) * dim + d;
            const std::size_t previous = static_cast<std::size_t>(i - 1) * dim + d;
            d_star[index] = (rhs[index] - lower[i] * d_star[previous]) / denom;
        }
    }

    for (int d = 0; d < dim; d++)
        solution[static_cast<std::size_t>(n - 1) * dim + d] =
            d_star[static_cast<std::size_t>(n - 1) * dim + d];

    for (int i = n - 2; i >= 0; i--)
        for (int d = 0; d < dim; d++) {
            const std::size_t index = static_cast<std::size_t>(i) * dim + d;
            const std::size_t next = static_cast<std::size_t>(i + 1) * dim + d;
            solution[index] = d_star[index] - c_star[i] * solution[next];
        }

    return true;
}

std::vector<double> compute_parameters(const double* points, int point_count, int dim,
                                       CurveNurbsKnotStyle style) {
    if (point_count < 1 || dim < 1)
        return std::vector<double>();

    const std::size_t value_count =
        static_cast<std::size_t>(point_count) * static_cast<std::size_t>(dim);

    if (!are_finite(points, value_count))
        return std::vector<double>();

    std::vector<double> params(point_count, 0.0);

    if (point_count < 2)
        return params;

    const int base_style = static_cast<int>(style) % 3;

    for (int i = 1; i < point_count; i++) {
        double dist = 0.0;

        for (int d = 0; d < dim; d++) {
            const std::size_t index = static_cast<std::size_t>(i) * dim + d;
            const std::size_t previous = static_cast<std::size_t>(i - 1) * dim + d;
            const double diff = points[index] - points[previous];
            dist += diff * diff;
        }

        dist = std::sqrt(dist);

        double delta = dist;

        if (base_style == 0)
            delta = 1.0;
        else if (base_style == 2)
            delta = std::sqrt(dist);

        params[i] = params[i - 1] + delta;
    }

    return params;
}

std::vector<double> build_interp_nurbsknots(const std::vector<double>& params, int degree) {

    if (params.size() > static_cast<std::size_t>(std::numeric_limits<int>::max() - 2))
        return std::vector<double>();

    const int n = static_cast<int>(params.size());

    if (n < 2 || degree < 1 || degree == std::numeric_limits<int>::max() || !are_finite(params, n))
        return std::vector<double>();

    const int order = degree + 1;
    const int cv_count = n + 2;
    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0)
        return std::vector<double>();

    const double t_max = params[n - 1];
    std::vector<double> nurbsknots(kc, 0.0);

    for (int i = 1; i < n - 1; i++)
        nurbsknots[order - 2 + i] = params[i];

    for (int i = 0; i < order - 1; i++)
        nurbsknots[kc - 1 - i] = t_max;

    return nurbsknots;
}

std::vector<double> eval_basis(int order, const std::vector<double>& nurbsknot, int span,
                               double t) {
    if (order < 1 || span < 0 || !std::isfinite(t))
        return std::vector<double>();

    if (order == 1)
        return std::vector<double>{1.0};

    const std::size_t first = static_cast<std::size_t>(span);
    const std::size_t width = 2 * static_cast<std::size_t>(order) - 2;

    if (first > std::numeric_limits<std::size_t>::max() - width)
        return std::vector<double>();

    const std::size_t end = first + width;

    if (nurbsknot.size() < end)
        return std::vector<double>();

    for (std::size_t i = first; i < end; i++)
        if (!std::isfinite(nurbsknot[i]))
            return std::vector<double>();

    std::vector<double> basis(order, 0.0);
    std::vector<double> left(order, 0.0);
    std::vector<double> right(order, 0.0);

    const std::size_t k_offset = first + static_cast<std::size_t>(order) - 2;
    basis[0] = 1.0;

    for (int j = 1; j < order; j++) {
        left[j] = t - nurbsknot[k_offset + 1 - j];
        right[j] = nurbsknot[k_offset + j] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            const double denom = right[r + 1] + left[j - r];
            const double temp = denom != 0.0 ? basis[r] / denom : 0.0;
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        basis[j] = saved;
    }

    return basis;
}

// ═══════════════════════════════════════════════════════════════════════════
// Fitting
// ═══════════════════════════════════════════════════════════════════════════

static std::vector<double> build_fitted_nurbsknots(const std::vector<double>& params, int num_cvs,
                                                   int degree) {
    const int m = static_cast<int>(params.size());
    const int n_interior = num_cvs - degree - 1;
    const int order = degree + 1;
    const int kc = nurbsknot_count(order, num_cvs);

    if (kc == 0)
        return std::vector<double>();

    std::vector<double> nurbsknots(kc, 0.0);

    for (int i = 0; i < degree; i++)
        nurbsknots[i] = params[0];

    const double d = static_cast<double>(m) / (num_cvs - degree);

    for (int j = 1; j <= n_interior; j++) {
        const int i = static_cast<int>(j * d);
        const double alpha = j * d - i;
        nurbsknots[degree - 1 + j] = (1.0 - alpha) * params[i - 1] + alpha * params[i];
    }

    for (int i = num_cvs - 1; i < kc; i++)
        nurbsknots[i] = params[m - 1];

    return nurbsknots;
}

static double turn_angle(const double* points, int dim, int prev, int i, int next) {

    double dot = 0.0;
    double len1sq = 0.0;
    double len2sq = 0.0;

    for (int d = 0; d < dim; d++) {
        const std::size_t index = static_cast<std::size_t>(i) * dim + d;
        const double a = points[index] - points[static_cast<std::size_t>(prev) * dim + d];
        const double b = points[static_cast<std::size_t>(next) * dim + d] - points[index];
        dot += a * b;
        len1sq += a * a;
        len2sq += b * b;
    }

    const double len1 = std::sqrt(len1sq);
    const double len2 = std::sqrt(len2sq);

    if (len1 <= PIVOT_TOLERANCE || len2 <= PIVOT_TOLERANCE)
        return 0.0;

    return std::acos(std::max(-1.0, std::min(1.0, dot / (len1 * len2))));
}

static double locate_target(const std::vector<double>& params, const std::vector<double>& cum,
                            int last, double target) {
    int lo = 0;
    int hi = last;

    while (lo < hi) {
        const int mid = lo + (hi - lo) / 2;

        if (cum[mid + 1] < target)
            lo = mid + 1;
        else
            hi = mid;
    }

    const double frac = cum[lo + 1] > cum[lo] ? (target - cum[lo]) / (cum[lo + 1] - cum[lo]) : 0.0;

    return params[lo] + frac * (params[lo + 1] - params[lo]);
}

std::vector<double> build_fitted_nurbsknots_adaptive(const std::vector<double>& params,
                                                     const double* points, int point_count, int dim,
                                                     int num_cvs, int degree, double scale) {
    const int m = point_count;

    if (m < 2 || dim < 1 || num_cvs <= degree || degree < 1 || !std::isfinite(scale) ||
        params.size() < static_cast<std::size_t>(m) || !are_finite(params, m))

        return std::vector<double>();

    if (m < 3 || !points) {
        if (m < num_cvs - degree)
            return std::vector<double>();

        return build_fitted_nurbsknots(params, num_cvs, degree);
    }

    if (!are_finite(points, static_cast<std::size_t>(m) * static_cast<std::size_t>(dim)))
        return std::vector<double>();

    std::vector<double> turn(m, 0.0);

    for (int i = 1; i < m - 1; i++)
        turn[i] = turn_angle(points, dim, i - 1, i, i + 1);

    std::vector<double> cum(m, 0.0);

    for (int i = 0; i < m - 1; i++) {
        const double chord = std::max(params[i + 1] - params[i], PIVOT_TOLERANCE);
        cum[i + 1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[i + 1]) * 0.5);
    }

    const double total = cum[m - 1];

    const int n_interior = num_cvs - degree - 1;
    const int order = degree + 1;
    const int kc = nurbsknot_count(order, num_cvs);

    if (kc == 0)
        return std::vector<double>();

    std::vector<double> nurbsknots(kc, 0.0);

    for (int i = 0; i < degree; i++)
        nurbsknots[i] = params[0];

    for (int j = 1; j <= n_interior; j++)
        nurbsknots[degree - 1 + j] =
            locate_target(params, cum, m - 2, total * j / (n_interior + 1));

    for (int i = num_cvs - 1; i < kc; i++)
        nurbsknots[i] = params[m - 1];

    return nurbsknots;
}

std::vector<double> build_fitted_nurbsknots_periodic_adaptive(const std::vector<double>& params,
                                                              const double* points, int n, int dim,
                                                              int num_cvs, int degree,
                                                              double scale) {
    if (n < 0 || degree < 1 || degree - 1 > num_cvs || degree == std::numeric_limits<int>::max() ||
        !std::isfinite(scale) || params.size() <= static_cast<std::size_t>(n) ||
        !are_finite(params, static_cast<std::size_t>(n) + 1))

        return std::vector<double>();

    const long long kc64 = static_cast<long long>(num_cvs) + 2LL * degree - 1;

    if (kc64 > std::numeric_limits<int>::max())
        return std::vector<double>();

    const int cv_count = num_cvs + degree;
    const int order = degree + 1;
    const int kc = nurbsknot_count(order, cv_count);

    if (kc == 0)
        return std::vector<double>();

    const double period = params[n];
    std::vector<double> nurbsknots(kc, 0.0);

    if (!std::isfinite(period) || period <= 0.0)
        return std::vector<double>();

    if (n < 3 || !points) {
        const double delta = period / num_cvs;

        for (int i = 0; i < kc; i++)
            nurbsknots[i] = (i - degree + 1) * delta;

        return nurbsknots;
    }

    if (dim < 1 || !are_finite(points, static_cast<std::size_t>(n) * static_cast<std::size_t>(dim)))
        return std::vector<double>();

    std::vector<double> turn(n, 0.0);

    for (int i = 0; i < n; i++)
        turn[i] = turn_angle(points, dim, i == 0 ? n - 1 : i - 1, i, (i + 1) % n);

    std::vector<double> cum(n + 1, 0.0);

    for (int i = 0; i < n; i++) {
        const double chord = std::max(params[i + 1] - params[i], PIVOT_TOLERANCE);
        cum[i + 1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[(i + 1) % n]) * 0.5);
    }

    const double total = cum[n];

    std::vector<double> base(num_cvs, 0.0);

    for (int j = 0; j < num_cvs; j++)
        base[j] = locate_target(params, cum, n - 1, total * j / num_cvs);

    std::vector<double> intervals(num_cvs, 0.0);

    for (int j = 0; j < num_cvs - 1; j++)
        intervals[j] = base[j + 1] - base[j];

    intervals[num_cvs - 1] = period - base[num_cvs - 1];

    for (int i = 1; i < degree; i++)
        nurbsknots[degree - 1 - i] = nurbsknots[degree - i] - intervals[num_cvs - i];

    for (int i = 0; i < kc - degree; i++)
        nurbsknots[degree + i] = nurbsknots[degree - 1 + i] + intervals[i % num_cvs];

    return nurbsknots;
}

bool solve_banded_spd(int dim, int n, int half_bw, std::vector<double>& band,
                      std::vector<double>& rhs) {
    if (dim < 1 || n < 1 || half_bw < 0 || half_bw == std::numeric_limits<int>::max())
        return false;

    const std::size_t band_count =
        static_cast<std::size_t>(n) * (static_cast<std::size_t>(half_bw) + 1);

    const std::size_t rhs_count = static_cast<std::size_t>(n) * static_cast<std::size_t>(dim);

    if (!are_finite(band, band_count) || !are_finite(rhs, rhs_count))
        return false;

    const int bw1 = half_bw + 1;

    for (int i = 0; i < n; i++) {
        for (int j = std::max(0, i - half_bw); j <= i; j++) {
            double sum = 0.0;

            for (int k = std::max(0, i - half_bw); k < j; k++) {
                const std::size_t i_index = static_cast<std::size_t>(i) * bw1 + i - k;
                const std::size_t j_index = static_cast<std::size_t>(j) * bw1 + j - k;
                sum += band[i_index] * band[j_index];
            }

            const std::size_t index = static_cast<std::size_t>(i) * bw1 + i - j;

            if (i == j) {
                const double val = band[index] - sum;

                if (val <= POSITIVE_DEFINITE_TOLERANCE)
                    return false;

                band[index] = std::sqrt(val);
            } else {
                band[index] = (band[index] - sum) / band[static_cast<std::size_t>(j) * bw1];
            }
        }
    }

    for (int i = 0; i < n; i++) {
        for (int d = 0; d < dim; d++) {
            double sum = 0.0;

            for (int k = std::max(0, i - half_bw); k < i; k++)
                sum += band[static_cast<std::size_t>(i) * bw1 + i - k] *
                       rhs[static_cast<std::size_t>(k) * dim + d];

            const std::size_t index = static_cast<std::size_t>(i) * dim + d;
            rhs[index] = (rhs[index] - sum) / band[static_cast<std::size_t>(i) * bw1];
        }
    }

    for (int i = n - 1; i >= 0; i--) {
        for (int d = 0; d < dim; d++) {
            double sum = 0.0;
            const int upper = static_cast<int>(
                std::min(static_cast<std::size_t>(n), static_cast<std::size_t>(i) + bw1));
            for (int k = i + 1; k < upper; k++)
                sum += band[static_cast<std::size_t>(k) * bw1 + k - i] *
                       rhs[static_cast<std::size_t>(k) * dim + d];

            const std::size_t index = static_cast<std::size_t>(i) * dim + d;
            rhs[index] = (rhs[index] - sum) / band[static_cast<std::size_t>(i) * bw1];
        }
    }

    return true;
}

} // namespace nurbsknot
} // namespace session_cpp
