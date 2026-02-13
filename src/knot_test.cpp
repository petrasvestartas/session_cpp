/**
 * @file knot_test.cpp
 * @brief Tests for knot module.
 */

#include "mini_test.h"
#include "knot.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Knot", "Knot_count") {
    // uncomment #include "knot.h"

    // Calculate knot counts for various order/cv_count combinations
    int count1 = knot::knot_count(2, 2);
    int count2 = knot::knot_count(3, 3);
    int count3 = knot::knot_count(4, 4);
    int count4 = knot::knot_count(4, 5);
    int count5 = knot::knot_count(3, 4);

    MINI_CHECK(count1 == 2);
    MINI_CHECK(count2 == 4);
    MINI_CHECK(count3 == 6);
    MINI_CHECK(count4 == 7);
    MINI_CHECK(count5 == 5);
}

MINI_TEST("Knot", "Make_clamped_uniform") {
    // uncomment #include "knot.h"

    // Basic clamped uniform knot vector
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    size_t k_len = k.size();
    double k0 = k[0], k1 = k[1], k2 = k[2];
    double k3 = k[3], k4 = k[4], k5 = k[5];

    // With custom delta
    auto k2_vec = knot::make_clamped_uniform(3, 4, 2.5);
    size_t k2_len = k2_vec.size();
    auto [t0, t1] = knot::get_domain(3, 4, k2_vec);

    // Invalid params
    auto k_invalid1 = knot::make_clamped_uniform(1, 2, 1.0);
    auto k_invalid2 = knot::make_clamped_uniform(4, 3, 1.0);

    MINI_CHECK(k_len == 6);
    MINI_CHECK(k0 == 0.0 && k1 == 0.0 && k2 == 0.0);
    MINI_CHECK(k3 == 1.0 && k4 == 1.0 && k5 == 1.0);
    MINI_CHECK(k2_len == 5);
    MINI_CHECK(t0 == 0.0 && t1 == 5.0);
    MINI_CHECK(k_invalid1.empty());
    MINI_CHECK(k_invalid2.empty());
}

MINI_TEST("Knot", "Make_periodic_uniform") {
    // uncomment #include "knot.h"

    // Create periodic uniform knot vector
    auto k = knot::make_periodic_uniform(3, 4, 1.0);
    size_t k_len = k.size();
    double k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3], k4 = k[4];

    MINI_CHECK(k_len == 5);
    MINI_CHECK(k0 == 0.0 && k1 == 1.0 && k2 == 2.0 && k3 == 3.0 && k4 == 4.0);
}

MINI_TEST("Knot", "Clamp") {
    // uncomment #include "knot.h"

    // Clamp a periodic knot vector
    std::vector<double> k = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    bool clamp_result = knot::clamp(4, 4, k, 2);
    bool first_clamped = k[0] == k[1] && k[1] == k[2];
    bool last_clamped = k[3] == k[4] && k[4] == k[5];

    MINI_CHECK(clamp_result);
    MINI_CHECK(first_clamped);
    MINI_CHECK(last_clamped);
}

MINI_TEST("Knot", "Is_valid") {
    // uncomment #include "knot.h"

    // Valid clamped knot vector
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    bool valid = knot::is_valid(4, 4, k);

    // Invalid - decreasing values
    std::vector<double> k_invalid = {0.0, 0.0, 1.0, 0.5, 1.0, 1.0};
    bool invalid = knot::is_valid(4, 4, k_invalid);

    MINI_CHECK(valid);
    MINI_CHECK(!invalid);
}

MINI_TEST("Knot", "Is_clamped") {
    // uncomment #include "knot.h"

    // Clamped knot vector
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    bool clamped_both = knot::is_clamped(4, 4, k, 2);
    bool clamped_start = knot::is_clamped(4, 4, k, 0);
    bool clamped_end = knot::is_clamped(4, 4, k, 1);

    // Periodic knot vector (not clamped)
    auto k2 = knot::make_periodic_uniform(4, 5, 1.0);
    bool not_clamped = knot::is_clamped(4, 5, k2, 2);

    MINI_CHECK(clamped_both);
    MINI_CHECK(clamped_start);
    MINI_CHECK(clamped_end);
    MINI_CHECK(!not_clamped);
}

MINI_TEST("Knot", "Is_periodic") {
    // uncomment #include "knot.h"

    // Periodic knot vector
    auto k = knot::make_periodic_uniform(3, 4, 1.0);
    bool periodic = knot::is_periodic(3, 4, k);

    // Clamped knot vector (not periodic)
    auto k2 = knot::make_clamped_uniform(4, 4, 1.0);
    bool not_periodic = knot::is_periodic(4, 4, k2);

    MINI_CHECK(periodic);
    MINI_CHECK(!not_periodic);
}

MINI_TEST("Knot", "Get_domain") {
    // uncomment #include "knot.h"

    // Get domain of clamped knot vector
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    auto [t0, t1] = knot::get_domain(4, 4, k);

    MINI_CHECK(t0 == 0.0);
    MINI_CHECK(t1 == 1.0);
}

MINI_TEST("Knot", "Set_domain") {
    // uncomment #include "knot.h"

    // Create knot vector and set domain
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    bool set_result = knot::set_domain(4, 4, k, 5.0, 10.0);
    auto [t0, t1] = knot::get_domain(4, 4, k);
    bool t0_close = std::fabs(t0 - 5.0) < 1e-10;
    bool t1_close = std::fabs(t1 - 10.0) < 1e-10;

    MINI_CHECK(set_result);
    MINI_CHECK(t0_close);
    MINI_CHECK(t1_close);
}

MINI_TEST("Knot", "Reverse") {
    // uncomment #include "knot.h"

    // Reverse knot vector
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    auto [t0_orig, t1_orig] = knot::get_domain(4, 4, k);
    bool reverse_result = knot::reverse(4, 4, k);
    auto [t0, t1] = knot::get_domain(4, 4, k);
    bool t0_preserved = std::fabs(t0 - t0_orig) < 1e-10;
    bool t1_preserved = std::fabs(t1 - t1_orig) < 1e-10;

    MINI_CHECK(reverse_result);
    MINI_CHECK(t0_preserved);
    MINI_CHECK(t1_preserved);
}

MINI_TEST("Knot", "Multiplicity") {
    // uncomment #include "knot.h"

    // Check multiplicity at clamped ends
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    int mult_first = knot::multiplicity(4, 4, k, 0);
    int mult_last = knot::multiplicity(4, 4, k, 5);

    MINI_CHECK(mult_first == 3);
    MINI_CHECK(mult_last == 3);
}

MINI_TEST("Knot", "Span_count") {
    // uncomment #include "knot.h"

    // Single Bezier span
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    int span1 = knot::span_count(4, 4, k);

    // Multiple spans
    auto k2 = knot::make_clamped_uniform(3, 5, 1.0);
    int span2 = knot::span_count(3, 5, k2);

    MINI_CHECK(span1 == 1);
    MINI_CHECK(span2 == 3);
}

MINI_TEST("Knot", "Find_span") {
    // uncomment #include "knot.h"

    // Find span in single-span knot vector
    auto k = knot::make_clamped_uniform(4, 4, 1.0);
    int span_0 = knot::find_span(4, 4, k, 0.0);
    int span_mid = knot::find_span(4, 4, k, 0.5);
    int span_1 = knot::find_span(4, 4, k, 1.0);

    // Find span in multi-span knot vector
    auto k2 = knot::make_clamped_uniform(3, 5, 1.0);
    int span2_0 = knot::find_span(3, 5, k2, 0.0);
    int span2_mid = knot::find_span(3, 5, k2, 1.5);
    int span2_end = knot::find_span(3, 5, k2, 2.5);

    MINI_CHECK(span_0 == 0 && span_mid == 0 && span_1 == 0);
    MINI_CHECK(span2_0 == 0 && span2_mid == 1 && span2_end == 2);
}

MINI_TEST("Knot", "Greville_abcissae") {
    // uncomment #include "knot.h"

    // Get Greville abcissae (control point parameter values)
    auto k = knot::make_clamped_uniform(3, 4, 1.0);
    auto g = knot::get_greville_abcissae(3, 4, k);
    size_t g_len = g.size();

    MINI_CHECK(g_len == 4);
}

MINI_TEST("Knot", "Domain_tolerance") {
    // uncomment #include "knot.h"

    // Calculate domain tolerance
    double tol_same = knot::domain_tolerance(1.0, 1.0);
    double tol_diff = knot::domain_tolerance(0.0, 1.0);

    MINI_CHECK(tol_same == 0.0);
    MINI_CHECK(tol_diff > 0.0);
}

} // namespace session_cpp
