/**
 * @file nurbsknot.cpp
 * @brief Implementation of nurbsknot vector utility functions.
 */

#include "nurbsknot.h"
#include <algorithm>
#include <cmath>

namespace session_cpp {
namespace nurbsknot {

std::vector<double> make_clamped_uniform(int order, int cv_count, double delta) {
    if (order < 2 || cv_count < order || delta <= 0.0) {
        return std::vector<double>();
    }
    
    int kc = nurbsknot_count(order, cv_count);
    std::vector<double> nurbsknot(kc, 0.0);
    
    // Fill interior nurbsknots: from index (order-2) to (cv_count-1)
    double k = 0.0;
    for (int i = order - 2; i < cv_count; i++) {
        nurbsknot[i] = k;
        k += delta;
    }
    
    // Clamp both ends
    clamp(order, cv_count, nurbsknot, 2);
    
    return nurbsknot;
}

std::vector<double> make_periodic_uniform(int order, int cv_count, double delta) {
    if (order < 2 || cv_count < order || delta <= 0.0) {
        return std::vector<double>();
    }
    
    int kc = nurbsknot_count(order, cv_count);
    std::vector<double> nurbsknot(kc, 0.0);
    
    double k = 0.0;
    for (int i = 0; i < kc; i++) {
        nurbsknot[i] = k;
        k += delta;
    }
    
    return nurbsknot;
}

bool clamp(int order, int cv_count, std::vector<double>& nurbsknot, int end) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return false;
    }
    
    // Clamp left end
    if (end == 0 || end == 2) {
        double clamp_value = nurbsknot[order - 2];
        for (int i = 0; i < order - 2; i++) {
            nurbsknot[i] = clamp_value;
        }
    }
    
    // Clamp right end
    if (end == 1 || end == 2) {
        double clamp_value = nurbsknot[cv_count - 1];
        for (int i = cv_count; i < kc; i++) {
            nurbsknot[i] = clamp_value;
        }
    }
    
    return true;
}

bool is_valid(int order, int cv_count, const std::vector<double>& nurbsknot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return false;
    }
    
    // Check non-decreasing
    for (int i = 1; i < kc; i++) {
        if (nurbsknot[i] < nurbsknot[i - 1]) {
            return false;
        }
    }
    
    // Check no degenerate spans (nurbsknot[i] < nurbsknot[i + order - 1])
    for (int i = 0; i < kc - order + 1; i++) {
        if (nurbsknot[i] >= nurbsknot[i + order - 1]) {
            return false;
        }
    }
    
    return true;
}

bool is_clamped(int order, int cv_count, const std::vector<double>& nurbsknot, int end) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return false;
    }
    
    int mult = order - 1;
    const double tol = 1e-10;
    
    // Check left end
    if (end == 0 || end == 2) {
        if (mult > kc) {
            return false;
        }
        double start_value = nurbsknot[0];
        for (int i = 1; i < mult; i++) {
            if (std::fabs(nurbsknot[i] - start_value) > tol) {
                return false;
            }
        }
    }
    
    // Check right end
    if (end == 1 || end == 2) {
        if (mult > kc) {
            return false;
        }
        double end_value = nurbsknot[kc - 1];
        for (int i = 1; i < mult; i++) {
            if (std::fabs(nurbsknot[kc - 1 - i] - end_value) > tol) {
                return false;
            }
        }
    }
    
    return true;
}

bool is_periodic(int order, int cv_count, const std::vector<double>& nurbsknot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc || kc < 2) {
        return false;
    }
    
    double delta = nurbsknot[1] - nurbsknot[0];
    if (delta <= 0) {
        return false;
    }
    
    const double tol = 1e-10;
    for (int i = 2; i < kc; i++) {
        if (std::fabs((nurbsknot[i] - nurbsknot[i - 1]) - delta) > tol) {
            return false;
        }
    }
    
    return true;
}

std::pair<double, double> get_domain(int order, int cv_count, const std::vector<double>& nurbsknot) {
    if (order < 2 || cv_count < order || 
        static_cast<int>(nurbsknot.size()) < nurbsknot_count(order, cv_count)) {
        return std::make_pair(0.0, 0.0);
    }
    
    return std::make_pair(nurbsknot[order - 2], nurbsknot[cv_count - 1]);
}

bool set_domain(int order, int cv_count, std::vector<double>& nurbsknot, double t0, double t1) {
    if (order < 2 || cv_count < order || t0 >= t1) {
        return false;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return false;
    }
    
    auto [old_t0, old_t1] = get_domain(order, cv_count, nurbsknot);
    if (old_t1 <= old_t0) {
        return false;
    }
    
    double scale = (t1 - t0) / (old_t1 - old_t0);
    for (int i = 0; i < kc; i++) {
        nurbsknot[i] = t0 + (nurbsknot[i] - old_t0) * scale;
    }
    
    return true;
}

bool reverse(int order, int cv_count, std::vector<double>& nurbsknot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return false;
    }
    
    // Reverse the array
    std::reverse(nurbsknot.begin(), nurbsknot.end());
    
    // Negate and shift to maintain same domain direction
    double t0 = nurbsknot[0];
    double t1 = nurbsknot[kc - 1];
    for (int i = 0; i < kc; i++) {
        nurbsknot[i] = t0 + t1 - nurbsknot[i];
    }
    
    return true;
}

int multiplicity(int order, int cv_count, const std::vector<double>& nurbsknot, int nurbsknot_index) {
    if (order < 2 || cv_count < order) {
        return 0;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc || nurbsknot_index < 0 || nurbsknot_index >= kc) {
        return 0;
    }
    
    double nurbsknot_value = nurbsknot[nurbsknot_index];
    int mult = 1;
    const double tol = 1e-14;
    
    // Count preceding equal nurbsknots
    int i = nurbsknot_index - 1;
    while (i >= 0 && std::fabs(nurbsknot[i] - nurbsknot_value) < tol) {
        mult++;
        i--;
    }
    
    // Count following equal nurbsknots
    i = nurbsknot_index + 1;
    while (i < kc && std::fabs(nurbsknot[i] - nurbsknot_value) < tol) {
        mult++;
        i++;
    }
    
    return mult;
}

int span_count(int order, int cv_count, const std::vector<double>& nurbsknot) {
    if (order < 2 || cv_count < order) {
        return 0;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return 0;
    }
    
    int count = 0;
    int d = order - 1;  // degree
    
    for (int i = 0; i < cv_count - order + 1; i++) {
        if (nurbsknot[i + d - 1] < nurbsknot[i + d]) {
            count++;
        }
    }
    
    return count;
}

int find_span(int order, int cv_count, const std::vector<double>& nurbsknot, 
              double t, int side, int hint) {
    (void)side;  // Not used in this implementation
    (void)hint;  // Not used in this implementation
    
    if (order < 2 || cv_count < order) {
        return 0;
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return 0;
    }
    
    // Shift by (order - 2) as in OpenNURBS
    int nurbsknot_offset = order - 2;
    int span_len = cv_count - order + 2;
    
    // Handle boundary cases
    if (t <= nurbsknot[nurbsknot_offset]) {
        return 0;
    }
    if (t >= nurbsknot[nurbsknot_offset + span_len - 1]) {
        return span_len - 2;
    }
    
    // Binary search
    int low = 0;
    int high = span_len - 1;
    
    while (high > low + 1) {
        int mid = (low + high) / 2;
        if (t < nurbsknot[nurbsknot_offset + mid]) {
            high = mid;
        } else {
            low = mid;
        }
    }
    
    return low;
}

std::vector<double> get_greville_abcissae(int order, int cv_count, 
                                           const std::vector<double>& nurbsknot,
                                           bool periodic) {
    if (order < 2 || cv_count < order) {
        return std::vector<double>();
    }
    
    int kc = nurbsknot_count(order, cv_count);
    if (static_cast<int>(nurbsknot.size()) != kc) {
        return std::vector<double>();
    }
    
    int d = order - 1;  // degree
    int count = periodic ? (cv_count - order + 1) : cv_count;
    
    std::vector<double> g(count);
    
    for (int i = 0; i < count; i++) {
        double sum = 0.0;
        for (int j = 0; j < d; j++) {
            sum += nurbsknot[i + j];
        }
        g[i] = sum / d;
    }
    
    return g;
}

bool solve_tridiagonal(int dim, int n,
                       std::vector<double>& lower,
                       std::vector<double>& diag,
                       std::vector<double>& upper,
                       const std::vector<double>& rhs,
                       std::vector<double>& solution) {
    if (n < 1 || dim < 1) return false;
    if (static_cast<int>(lower.size()) < n ||
        static_cast<int>(diag.size()) < n ||
        static_cast<int>(upper.size()) < n ||
        static_cast<int>(rhs.size()) < n * dim) {
        return false;
    }

    solution.resize(n * dim);
    const double eps = 1e-14;

    // Forward elimination (Thomas algorithm)
    std::vector<double> c_star(n), d_star(n * dim);

    if (std::fabs(diag[0]) < eps) return false;
    c_star[0] = upper[0] / diag[0];
    for (int d = 0; d < dim; d++) {
        d_star[d] = rhs[d] / diag[0];
    }

    for (int i = 1; i < n; i++) {
        double denom = diag[i] - lower[i] * c_star[i-1];
        if (std::fabs(denom) < eps) return false;

        c_star[i] = (i < n-1) ? upper[i] / denom : 0.0;
        for (int d = 0; d < dim; d++) {
            d_star[i * dim + d] = (rhs[i * dim + d] - lower[i] * d_star[(i-1) * dim + d]) / denom;
        }
    }

    // Back substitution
    for (int d = 0; d < dim; d++) {
        solution[(n-1) * dim + d] = d_star[(n-1) * dim + d];
    }

    for (int i = n - 2; i >= 0; i--) {
        for (int d = 0; d < dim; d++) {
            solution[i * dim + d] = d_star[i * dim + d] - c_star[i] * solution[(i+1) * dim + d];
        }
    }

    return true;
}

std::vector<double> compute_parameters(const double* points, int point_count,
                                       int dim, CurveNurbsKnotStyle style) {
    std::vector<double> params(point_count);
    if (point_count < 2) return params;

    params[0] = 0.0;

    int base_style = static_cast<int>(style) % 3;  // 0=Uniform, 1=Chord, 2=ChordSquareRoot

    for (int i = 1; i < point_count; i++) {
        double dist = 0.0;
        for (int d = 0; d < dim; d++) {
            double diff = points[i * dim + d] - points[(i-1) * dim + d];
            dist += diff * diff;
        }
        dist = std::sqrt(dist);

        double delta;
        switch (base_style) {
            case 0:  // Uniform
                delta = 1.0;
                break;
            case 1:  // Chord
                delta = dist;
                break;
            case 2:  // ChordSquareRoot (centripetal)
                delta = std::sqrt(dist);
                break;
            default:
                delta = dist;
        }
        params[i] = params[i-1] + delta;
    }

    return params;
}

std::vector<double> build_interp_nurbsknots(const std::vector<double>& params, int degree) {
    int n = static_cast<int>(params.size());
    if (n < 2 || degree < 1) return {};

    int order = degree + 1;
    int cv_count = n + 2;  // Natural end conditions add 2 CVs
    int kc = nurbsknot_count(order, cv_count);

    std::vector<double> nurbsknots(kc);
    double t_max = params[n - 1];

    // Clamped start: first (order-1) nurbsknots = 0
    for (int i = 0; i < order - 1; i++) {
        nurbsknots[i] = 0.0;
    }

    // Interior nurbsknots from parameters (skip first and last)
    for (int i = 1; i < n - 1; i++) {
        nurbsknots[order - 2 + i] = params[i];
    }

    // Clamped end: last (order-1) nurbsknots = t_max
    for (int i = 0; i < order - 1; i++) {
        nurbsknots[kc - 1 - i] = t_max;
    }

    return nurbsknots;
}

std::vector<double> eval_basis(int order, const std::vector<double>& nurbsknot, int span, double t) {
    std::vector<double> basis(order, 0.0);
    std::vector<double> left(order);
    std::vector<double> right(order);

    const double* k = nurbsknot.data() + (order - 2) + span;

    basis[0] = 1.0;

    for (int j = 1; j < order; j++) {
        left[j] = t - k[1 - j];
        right[j] = k[j] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            double denom = right[r + 1] + left[j - r];
            double temp = (denom != 0.0) ? basis[r] / denom : 0.0;
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis[j] = saved;
    }

    return basis;
}

static std::vector<double> build_fitted_nurbsknots(const std::vector<double>& params, int num_cvs, int degree) {
    int m = static_cast<int>(params.size());
    int n_interior = num_cvs - degree - 1;
    int order = degree + 1;
    int kc = nurbsknot_count(order, num_cvs);
    std::vector<double> nurbsknots(kc);

    for (int i = 0; i < degree; i++) nurbsknots[i] = params[0];

    double d = (double)m / (num_cvs - degree);
    for (int j = 1; j <= n_interior; j++) {
        int i = (int)(j * d);
        double alpha = j * d - i;
        nurbsknots[degree - 1 + j] = (1.0 - alpha) * params[i - 1] + alpha * params[i];
    }

    for (int i = num_cvs - 1; i < kc; i++) nurbsknots[i] = params[m - 1];

    return nurbsknots;
}

std::vector<double> build_fitted_nurbsknots_adaptive(const std::vector<double>& params,
    const double* points, int point_count, int dim, int num_cvs, int degree, double scale) {
    int m = point_count;
    if (m < 3 || !points)
        return build_fitted_nurbsknots(params, num_cvs, degree);

    std::vector<double> turn(m, 0.0);
    for (int i = 1; i < m - 1; i++) {
        double dot = 0, len1sq = 0, len2sq = 0;
        for (int d = 0; d < dim; d++) {
            double a = points[i*dim+d] - points[(i-1)*dim+d];
            double b = points[(i+1)*dim+d] - points[i*dim+d];
            dot += a * b; len1sq += a * a; len2sq += b * b;
        }
        double len1 = std::sqrt(len1sq), len2 = std::sqrt(len2sq);
        if (len1 > 1e-14 && len2 > 1e-14) {
            double c = std::max(-1.0, std::min(1.0, dot / (len1 * len2)));
            turn[i] = std::acos(c);
        }
    }

    std::vector<double> cum(m, 0.0);
    for (int i = 0; i < m - 1; i++) {
        double chord = params[i+1] - params[i];
        if (chord < 1e-14) chord = 1e-14;
        cum[i+1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[i+1]) * 0.5);
    }
    double total = cum[m-1];

    int n_interior = num_cvs - degree - 1;
    int order = degree + 1;
    int kc = nurbsknot_count(order, num_cvs);
    std::vector<double> nurbsknots(kc);
    for (int i = 0; i < degree; i++) nurbsknots[i] = params[0];

    for (int j = 1; j <= n_interior; j++) {
        double target = total * j / (n_interior + 1);
        int lo = 0, hi = m - 2;
        while (lo < hi) { int mid = (lo + hi) / 2; if (cum[mid+1] < target) lo = mid + 1; else hi = mid; }
        double frac = (cum[lo+1] > cum[lo]) ? (target - cum[lo]) / (cum[lo+1] - cum[lo]) : 0.0;
        nurbsknots[degree - 1 + j] = params[lo] + frac * (params[lo+1] - params[lo]);
    }

    for (int i = num_cvs - 1; i < kc; i++) nurbsknots[i] = params[m - 1];
    return nurbsknots;
}

std::vector<double> build_fitted_nurbsknots_periodic_adaptive(const std::vector<double>& params,
    const double* points, int n, int dim, int num_cvs, int degree, double scale) {
    int cv_count = num_cvs + degree;
    int order = degree + 1;
    int kc = cv_count + order - 2;
    double T = params[n];

    if (n < 3 || !points) {
        double delta = T / num_cvs;
        std::vector<double> nurbsknots(kc);
        for (int i = 0; i < kc; i++) nurbsknots[i] = (i - degree + 1) * delta;
        return nurbsknots;
    }

    std::vector<double> turn(n, 0.0);
    for (int i = 0; i < n; i++) {
        int prev = (i - 1 + n) % n, next = (i + 1) % n;
        double dot = 0, len1sq = 0, len2sq = 0;
        for (int d = 0; d < dim; d++) {
            double a = points[i*dim+d] - points[prev*dim+d];
            double b = points[next*dim+d] - points[i*dim+d];
            dot += a * b; len1sq += a * a; len2sq += b * b;
        }
        double len1 = std::sqrt(len1sq), len2 = std::sqrt(len2sq);
        if (len1 > 1e-14 && len2 > 1e-14) {
            double c = std::max(-1.0, std::min(1.0, dot / (len1 * len2)));
            turn[i] = std::acos(c);
        }
    }

    std::vector<double> cum(n + 1, 0.0);
    for (int i = 0; i < n; i++) {
        double chord = params[i+1] - params[i];
        if (chord < 1e-14) chord = 1e-14;
        int next = (i + 1) % n;
        cum[i+1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[next]) * 0.5);
    }
    double total = cum[n];

    std::vector<double> base(num_cvs);
    for (int j = 0; j < num_cvs; j++) {
        double target = total * j / num_cvs;
        int lo = 0, hi = n - 1;
        while (lo < hi) { int mid = (lo + hi) / 2; if (cum[mid+1] < target) lo = mid + 1; else hi = mid; }
        double frac = (cum[lo+1] > cum[lo]) ? (target - cum[lo]) / (cum[lo+1] - cum[lo]) : 0.0;
        base[j] = params[lo] + frac * (params[lo+1] - params[lo]);
    }

    std::vector<double> intervals(num_cvs);
    for (int j = 0; j < num_cvs - 1; j++) intervals[j] = base[j+1] - base[j];
    intervals[num_cvs - 1] = T - base[num_cvs - 1];

    std::vector<double> nurbsknots(kc);
    nurbsknots[degree - 1] = 0.0;
    for (int i = 1; i < degree; i++)
        nurbsknots[degree - 1 - i] = nurbsknots[degree - i] - intervals[num_cvs - i];
    for (int i = 0; i < kc - degree; i++)
        nurbsknots[degree + i] = nurbsknots[degree - 1 + i] + intervals[i % num_cvs];

    return nurbsknots;
}

bool solve_banded_spd(int dim, int n, int half_bw, std::vector<double>& band, std::vector<double>& rhs) {
    int bw1 = half_bw + 1;

    // Cholesky factorization: A = L * L^T
    for (int i = 0; i < n; i++) {
        for (int j = std::max(0, i - half_bw); j <= i; j++) {
            double sum = 0.0;
            for (int k = std::max(0, i - half_bw); k < j; k++)
                sum += band[i * bw1 + (i - k)] * band[j * bw1 + (j - k)];
            if (i == j) {
                double val = band[i * bw1] - sum;
                if (val <= 1e-30) return false;
                band[i * bw1] = std::sqrt(val);
            } else {
                band[i * bw1 + (i - j)] = (band[i * bw1 + (i - j)] - sum) / band[j * bw1];
            }
        }
    }

    // Forward substitution: L * y = rhs
    for (int i = 0; i < n; i++) {
        for (int d = 0; d < dim; d++) {
            double sum = 0.0;
            for (int k = std::max(0, i - half_bw); k < i; k++)
                sum += band[i * bw1 + (i - k)] * rhs[k * dim + d];
            rhs[i * dim + d] = (rhs[i * dim + d] - sum) / band[i * bw1];
        }
    }

    // Back substitution: L^T * x = y
    for (int i = n - 1; i >= 0; i--) {
        for (int d = 0; d < dim; d++) {
            double sum = 0.0;
            for (int k = i + 1; k < std::min(n, i + half_bw + 1); k++)
                sum += band[k * bw1 + (k - i)] * rhs[k * dim + d];
            rhs[i * dim + d] = (rhs[i * dim + d] - sum) / band[i * bw1];
        }
    }

    return true;
}

} // namespace nurbsknot
} // namespace session_cpp
