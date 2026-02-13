/**
 * @file knot.cpp
 * @brief Implementation of knot vector utility functions.
 */

#include "knot.h"
#include <algorithm>
#include <cmath>

namespace session_cpp {
namespace knot {

std::vector<double> make_clamped_uniform(int order, int cv_count, double delta) {
    if (order < 2 || cv_count < order || delta <= 0.0) {
        return std::vector<double>();
    }
    
    int kc = knot_count(order, cv_count);
    std::vector<double> knot(kc, 0.0);
    
    // Fill interior knots: from index (order-2) to (cv_count-1)
    double k = 0.0;
    for (int i = order - 2; i < cv_count; i++) {
        knot[i] = k;
        k += delta;
    }
    
    // Clamp both ends
    clamp(order, cv_count, knot, 2);
    
    return knot;
}

std::vector<double> make_periodic_uniform(int order, int cv_count, double delta) {
    if (order < 2 || cv_count < order || delta <= 0.0) {
        return std::vector<double>();
    }
    
    int kc = knot_count(order, cv_count);
    std::vector<double> knot(kc, 0.0);
    
    double k = 0.0;
    for (int i = 0; i < kc; i++) {
        knot[i] = k;
        k += delta;
    }
    
    return knot;
}

bool clamp(int order, int cv_count, std::vector<double>& knot, int end) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return false;
    }
    
    // Clamp left end
    if (end == 0 || end == 2) {
        double clamp_value = knot[order - 2];
        for (int i = 0; i < order - 2; i++) {
            knot[i] = clamp_value;
        }
    }
    
    // Clamp right end
    if (end == 1 || end == 2) {
        double clamp_value = knot[cv_count - 1];
        for (int i = cv_count; i < kc; i++) {
            knot[i] = clamp_value;
        }
    }
    
    return true;
}

bool is_valid(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return false;
    }
    
    // Check non-decreasing
    for (int i = 1; i < kc; i++) {
        if (knot[i] < knot[i - 1]) {
            return false;
        }
    }
    
    // Check no degenerate spans (knot[i] < knot[i + order - 1])
    for (int i = 0; i < kc - order + 1; i++) {
        if (knot[i] >= knot[i + order - 1]) {
            return false;
        }
    }
    
    return true;
}

bool is_clamped(int order, int cv_count, const std::vector<double>& knot, int end) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return false;
    }
    
    int mult = order - 1;
    const double tol = 1e-10;
    
    // Check left end
    if (end == 0 || end == 2) {
        if (mult > kc) {
            return false;
        }
        double start_value = knot[0];
        for (int i = 1; i < mult; i++) {
            if (std::fabs(knot[i] - start_value) > tol) {
                return false;
            }
        }
    }
    
    // Check right end
    if (end == 1 || end == 2) {
        if (mult > kc) {
            return false;
        }
        double end_value = knot[kc - 1];
        for (int i = 1; i < mult; i++) {
            if (std::fabs(knot[kc - 1 - i] - end_value) > tol) {
                return false;
            }
        }
    }
    
    return true;
}

bool is_periodic(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc || kc < 2) {
        return false;
    }
    
    double delta = knot[1] - knot[0];
    if (delta <= 0) {
        return false;
    }
    
    const double tol = 1e-10;
    for (int i = 2; i < kc; i++) {
        if (std::fabs((knot[i] - knot[i - 1]) - delta) > tol) {
            return false;
        }
    }
    
    return true;
}

bool is_uniform(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return false;
    }
    
    // Check interior knots (from order-2 to cv_count-1)
    if (cv_count <= order) {
        return true;  // No interior knots
    }
    
    int start_idx = order - 2;
    int end_idx = cv_count - 1;
    
    if (end_idx <= start_idx) {
        return true;
    }
    
    double delta = knot[start_idx + 1] - knot[start_idx];
    if (delta <= 0) {
        return false;
    }
    
    const double tol = 1e-10;
    for (int i = start_idx + 2; i <= end_idx; i++) {
        if (std::fabs((knot[i] - knot[i - 1]) - delta) > tol) {
            return false;
        }
    }
    
    return true;
}

std::pair<double, double> get_domain(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order || 
        static_cast<int>(knot.size()) < knot_count(order, cv_count)) {
        return std::make_pair(0.0, 0.0);
    }
    
    return std::make_pair(knot[order - 2], knot[cv_count - 1]);
}

bool set_domain(int order, int cv_count, std::vector<double>& knot, double t0, double t1) {
    if (order < 2 || cv_count < order || t0 >= t1) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return false;
    }
    
    auto [old_t0, old_t1] = get_domain(order, cv_count, knot);
    if (old_t1 <= old_t0) {
        return false;
    }
    
    double scale = (t1 - t0) / (old_t1 - old_t0);
    for (int i = 0; i < kc; i++) {
        knot[i] = t0 + (knot[i] - old_t0) * scale;
    }
    
    return true;
}

bool reverse(int order, int cv_count, std::vector<double>& knot) {
    if (order < 2 || cv_count < order) {
        return false;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return false;
    }
    
    // Reverse the array
    std::reverse(knot.begin(), knot.end());
    
    // Negate and shift to maintain same domain direction
    double t0 = knot[0];
    double t1 = knot[kc - 1];
    for (int i = 0; i < kc; i++) {
        knot[i] = t0 + t1 - knot[i];
    }
    
    return true;
}

int multiplicity(int order, int cv_count, const std::vector<double>& knot, int knot_index) {
    if (order < 2 || cv_count < order) {
        return 0;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc || knot_index < 0 || knot_index >= kc) {
        return 0;
    }
    
    double knot_value = knot[knot_index];
    int mult = 1;
    const double tol = 1e-14;
    
    // Count preceding equal knots
    int i = knot_index - 1;
    while (i >= 0 && std::fabs(knot[i] - knot_value) < tol) {
        mult++;
        i--;
    }
    
    // Count following equal knots
    i = knot_index + 1;
    while (i < kc && std::fabs(knot[i] - knot_value) < tol) {
        mult++;
        i++;
    }
    
    return mult;
}

int span_count(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) {
        return 0;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return 0;
    }
    
    int count = 0;
    int d = order - 1;  // degree
    
    for (int i = 0; i < cv_count - order + 1; i++) {
        if (knot[i + d - 1] < knot[i + d]) {
            count++;
        }
    }
    
    return count;
}

std::vector<double> get_span_vector(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) {
        return std::vector<double>();
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return std::vector<double>();
    }
    
    std::vector<double> spans;
    const double tol = 1e-14;
    
    for (int i = 0; i < kc - 1; i++) {
        if (std::fabs(knot[i + 1] - knot[i]) > tol) {
            spans.push_back(knot[i]);
        }
    }
    
    if (kc > 0) {
        spans.push_back(knot[kc - 1]);
    }
    
    return spans;
}

int find_span(int order, int cv_count, const std::vector<double>& knot, 
              double t, int side, int hint) {
    (void)side;  // Not used in this implementation
    (void)hint;  // Not used in this implementation
    
    if (order < 2 || cv_count < order) {
        return 0;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return 0;
    }
    
    // Shift by (order - 2) as in OpenNURBS
    int knot_offset = order - 2;
    int span_len = cv_count - order + 2;
    
    // Handle boundary cases
    if (t <= knot[knot_offset]) {
        return 0;
    }
    if (t >= knot[knot_offset + span_len - 1]) {
        return span_len - 2;
    }
    
    // Binary search
    int low = 0;
    int high = span_len - 1;
    
    while (high > low + 1) {
        int mid = (low + high) / 2;
        if (t < knot[knot_offset + mid]) {
            high = mid;
        } else {
            low = mid;
        }
    }
    
    return low;
}

double superfluous_knot(int order, int cv_count, const std::vector<double>& knot, int end) {
    if (order < 2 || cv_count < order) {
        return 0.0;
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return 0.0;
    }
    
    if (end == 0) {
        // First superfluous knot
        return 2.0 * knot[0] - knot[order - 2];
    } else {
        // Last superfluous knot
        return 2.0 * knot[kc - 1] - knot[cv_count - order];
    }
}

double greville_abcissa(int order, const double* knot) {
    if (order < 2 || knot == nullptr) {
        return 0.0;
    }
    
    int d = order - 1;  // degree
    double sum = 0.0;
    for (int i = 0; i < d; i++) {
        sum += knot[i];
    }
    return sum / d;
}

std::vector<double> get_greville_abcissae(int order, int cv_count, 
                                           const std::vector<double>& knot,
                                           bool periodic) {
    if (order < 2 || cv_count < order) {
        return std::vector<double>();
    }
    
    int kc = knot_count(order, cv_count);
    if (static_cast<int>(knot.size()) != kc) {
        return std::vector<double>();
    }
    
    int d = order - 1;  // degree
    int count = periodic ? (cv_count - order + 1) : cv_count;
    
    std::vector<double> g(count);
    
    for (int i = 0; i < count; i++) {
        double sum = 0.0;
        for (int j = 0; j < d; j++) {
            sum += knot[i + j];
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
                                       int dim, CurveKnotStyle style) {
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

std::vector<double> build_interp_knots(const std::vector<double>& params, int degree) {
    int n = static_cast<int>(params.size());
    if (n < 2 || degree < 1) return {};

    int order = degree + 1;
    int cv_count = n + 2;  // Natural end conditions add 2 CVs
    int kc = knot_count(order, cv_count);

    std::vector<double> knots(kc);
    double t_max = params[n - 1];

    // Clamped start: first (order-1) knots = 0
    for (int i = 0; i < order - 1; i++) {
        knots[i] = 0.0;
    }

    // Interior knots from parameters (skip first and last)
    for (int i = 1; i < n - 1; i++) {
        knots[order - 2 + i] = params[i];
    }

    // Clamped end: last (order-1) knots = t_max
    for (int i = 0; i < order - 1; i++) {
        knots[kc - 1 - i] = t_max;
    }

    return knots;
}

std::vector<double> eval_basis(int order, const std::vector<double>& knot, int span, double t) {
    std::vector<double> basis(order, 0.0);
    std::vector<double> left(order);
    std::vector<double> right(order);

    const double* k = knot.data() + (order - 2) + span;

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

} // namespace knot
} // namespace session_cpp
