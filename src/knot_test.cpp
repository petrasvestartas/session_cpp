/**
 * @file knot_test.cpp
 * @brief Tests for knot module.
 */

#include "mini_test.h"
#include "knot.h"
#include <cmath>
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Knot", "Make Clamped Uniform") {
    // uncomment #include "knot.h"

    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> knots = knot::make_clamped_uniform(order, cv_count);
    MINI_CHECK(TOLERANCE.is_allclose(knots, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));
}

MINI_TEST("Knot", "Make Periodic Uniform") {
    // uncomment #include "knot.h"

    // 0 1 2 3 4 5 6
    int order = 4;
    int cv_count = 5;
    std::vector<double> knots = knot::make_periodic_uniform(order, cv_count);
    MINI_CHECK(TOLERANCE.is_allclose(knots, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
}

MINI_TEST("Knot", "Is Clamped") {
    // uncomment #include "knot.h"

    // 0 0 0 1 2 2 2
    // 0 1 2 3 4 5 6
    int order = 4;
    int cv_count = 5;
    std::vector<double> knots_periodic = knot::make_periodic_uniform(order, cv_count);
    std::vector<double> knots_clamped = knot::make_clamped_uniform(order, cv_count);
    bool is_not_clamped = knot::is_clamped(order, cv_count, knots_periodic);
    bool is_clamped = knot::is_clamped(order, cv_count, knots_clamped);

    MINI_CHECK(!is_not_clamped && is_clamped);
}

MINI_TEST("Knot", "Reverse") {
    // uncomment #include "knot.h"

    // Reverses NURBS curve direction: flip control points + reverse knots
    // Steps: 1) reverse array  2) remap knot[i] = t0 + t1 - knot[i] to preserve domain

    // Symmetric knot vector -> reverse gives back the same (palindrome)
    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> knots_sym = knot::make_clamped_uniform(order, cv_count);
    knot::reverse(order, cv_count, knots_sym);
    MINI_CHECK(TOLERANCE.is_allclose(knots_sym, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));

    // Asymmetric knot vector -> extra knot at 0.5 shifts to 1.5 after reverse
    // 0 0 0 0.5 1 2 2 2 -> 0 0 0 1 1.5 2 2 2
    std::vector<double> knots_asym = {0.0, 0.0, 0.0, 0.5, 1.0, 2.0, 2.0, 2.0};
    knot::reverse(4, 6, knots_asym);
    MINI_CHECK(TOLERANCE.is_allclose(knots_asym, {0.0, 0.0, 0.0, 1.0, 1.5, 2.0, 2.0, 2.0}));
}

MINI_TEST("Knot", "Find Span") {
    // uncomment #include "knot.h"

    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> knots_clamped = knot::make_clamped_uniform(order, cv_count);
    //   - 0.5 falls in span [0, 1] -> index 0
    //   - 1.5 falls in span [1, 2] -> index 1
    int spancount0 = knot::find_span(order, cv_count, knots_clamped, 0.5);
    int spancount1 = knot::find_span(order, cv_count, knots_clamped, 1.5);
    MINI_CHECK(spancount0 == 0 && spancount1 == 1);
}

MINI_TEST("Knot", "Solve Tridiagonal") {
    // uncomment #include "knot.h"

    // Thomas algorithm -- an O(n) solver for tridiagonal linear systems
    //   | 2 1 | |x0|   |3|
    //   | 1 2 | |x1| = |3|
    //   -> solution: x0 = 1, x1 = 1
    std::vector<double> lo={0,1}, di={2,2}, up={1,0}, rh={3,3}, sol;
    knot::solve_tridiagonal(1, 2, lo, di, up, rh, sol);
    MINI_CHECK(TOLERANCE.is_allclose(sol, {1.0, 1.0}));
}

MINI_TEST("Knot", "Compute Parameters") {
    // uncomment #include "knot.h"

    double pts[] = {0,0,0, 1,0,0, 2,0,0, 3,0,0};
    // Chord-length parameterization: since all gaps are 1.0, params = {0, 1, 2, 3}
    std::vector<double> t = knot::compute_parameters(pts, 4, 3, CurveKnotStyle::Chord);
    MINI_CHECK(TOLERANCE.is_allclose(t, {0.0, 1.0, 2.0, 3.0}));
}

MINI_TEST("Knot", "Build Interpolation Knots") {
    // uncomment #include "knot.h"

    std::vector<double> params = {0.0, 1.0, 2.0, 3.0};
    int degree = 3;
    // cv_count = n + 2 = 6 (natural end conditions add 2 CVs)
    // kc = order + cv_count - 2 = 4 + 6 - 2 = 8
    //   [0, 0, 0,  |  1, 2,  |  3, 3, 3]
    //   <-clamp->    interior    <-clamp->
    std::vector<double> knots = knot::build_interp_knots(params, degree);
    MINI_CHECK(TOLERANCE.is_allclose(knots, {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0}));
}

MINI_TEST("Knot", "Evaluation Basis") {
    // uncomment #include "knot.h"

    // Cox-de Boor recursive evaluation of B-spline basis functions
    // At parameter t, exactly 'order' basis functions are non-zero
    // Partition of unity: they always sum to 1.0
    // Used to evaluate NURBS curves/surfaces: C(t) = sum(N_i(t) * P_i)
    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> knots = knot::make_clamped_uniform(order, cv_count);
    int span = knot::find_span(order, cv_count, knots, 0.5);
    std::vector<double> basis = knot::eval_basis(order, knots, span, 0.5);
    MINI_CHECK(TOLERANCE.is_allclose(basis, {0.125, 0.59375, 0.25, 0.03125}));
}

MINI_TEST("Knot", "Build Fitted Knots Adaptive") {
    // uncomment #include "knot.h"

    // Builds knot vectors for least-squares fitting
    // Concentrates knots where curvature is high (sharp turns)
    // For collinear points (zero curvature), interior knots are evenly distributed
    double pts[] = {0,0,0, 1,0,0, 2,0,0, 3,0,0, 4,0,0};
    std::vector<double> params = knot::compute_parameters(pts, 5, 3, CurveKnotStyle::Chord);
    std::vector<double> knots = knot::build_fitted_knots_adaptive(params, pts, 5, 3, 5, 3);
    MINI_CHECK(TOLERANCE.is_allclose(knots, {0.0, 0.0, 0.0, 2.0, 4.0, 4.0, 4.0}));
}

MINI_TEST("Knot", "Build Fitted Knots Periodic Adaptive") {
    // uncomment #include "knot.h"

    // Periodic version for closed curves -- knots wrap around
    // For a regular square (equal turns, equal chords), knots are uniformly spaced
    double pts[] = {0,0,0, 1,0,0, 1,1,0, 0,1,0};
    std::vector<double> params = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> knots = knot::build_fitted_knots_periodic_adaptive(params, pts, 4, 3, 4, 3);
    MINI_CHECK(TOLERANCE.is_allclose(knots, {-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
}

MINI_TEST("Knot", "Solve Banded SPD") {
    // uncomment #include "knot.h"

    // Cholesky solver for banded symmetric positive-definite systems
    //   | 4 2 0 |       |8 |       |1|
    //   | 2 5 1 | * x = |13| -> x = |2|
    //   | 0 1 3 |       |5 |       |1|
    std::vector<double> band = {4, 0, 5, 2, 3, 1};
    std::vector<double> rhs = {8, 13, 5};
    knot::solve_banded_spd(1, 3, 1, band, rhs);
    MINI_CHECK(TOLERANCE.is_allclose(rhs, {1.0, 2.0, 1.0}));
}

} // namespace session_cpp
