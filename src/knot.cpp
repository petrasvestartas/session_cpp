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

} // namespace knot
} // namespace session_cpp
