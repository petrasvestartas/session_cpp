/**
 * @file nurbsknot_test.cpp
 * @brief Tests for nurbsknot module.
 */

#include "mini_test.h"
#include "nurbsknot.h"
#include <cmath>
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("NurbsKnot", "Make Clamped Uniform") {
    // uncomment #include "nurbsknot.h"

    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));
}

MINI_TEST("NurbsKnot", "Make Periodic Uniform") {
    // uncomment #include "nurbsknot.h"

    // 0 1 2 3 4 5 6
    int order = 4;
    int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_periodic_uniform(order, cv_count);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
}

MINI_TEST("NurbsKnot", "Is Clamped") {
    // uncomment #include "nurbsknot.h"

    // 0 0 0 1 2 2 2
    // 0 1 2 3 4 5 6
    int order = 4;
    int cv_count = 5;
    std::vector<double> nurbsknots_periodic = nurbsknot::make_periodic_uniform(order, cv_count);
    std::vector<double> nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count);
    bool is_not_clamped = nurbsknot::is_clamped(order, cv_count, nurbsknots_periodic);
    bool is_clamped = nurbsknot::is_clamped(order, cv_count, nurbsknots_clamped);

    MINI_CHECK(!is_not_clamped && is_clamped);
}

MINI_TEST("NurbsKnot", "Reverse") {
    // uncomment #include "nurbsknot.h"

    // Reverses NURBS curve direction: flip control points + reverse nurbsknots
    // Steps: 1) reverse array  2) remap nurbsknot[i] = t0 + t1 - nurbsknot[i] to preserve domain

    // Symmetric nurbsknot vector -> reverse gives back the same (palindrome)
    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> nurbsknots_sym = nurbsknot::make_clamped_uniform(order, cv_count);
    nurbsknot::reverse(order, cv_count, nurbsknots_sym);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots_sym, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));

    // Asymmetric nurbsknot vector -> extra nurbsknot at 0.5 shifts to 1.5 after reverse
    // 0 0 0 0.5 1 2 2 2 -> 0 0 0 1 1.5 2 2 2
    std::vector<double> nurbsknots_asym = {0.0, 0.0, 0.0, 0.5, 1.0, 2.0, 2.0, 2.0};
    nurbsknot::reverse(4, 6, nurbsknots_asym);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots_asym, {0.0, 0.0, 0.0, 1.0, 1.5, 2.0, 2.0, 2.0}));
}

MINI_TEST("NurbsKnot", "Find Span") {
    // uncomment #include "nurbsknot.h"

    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count);
    //   - 0.5 falls in span [0, 1] -> index 0
    //   - 1.5 falls in span [1, 2] -> index 1
    int spancount0 = nurbsknot::find_span(order, cv_count, nurbsknots_clamped, 0.5);
    int spancount1 = nurbsknot::find_span(order, cv_count, nurbsknots_clamped, 1.5);
    MINI_CHECK(spancount0 == 0 && spancount1 == 1);
}

MINI_TEST("NurbsKnot", "Solve Tridiagonal") {
    // uncomment #include "nurbsknot.h"

    // Thomas algorithm -- an O(n) solver for tridiagonal linear systems
    //   | 2 1 | |x0|   |3|
    //   | 1 2 | |x1| = |3|
    //   -> solution: x0 = 1, x1 = 1
    std::vector<double> lo={0,1}, di={2,2}, up={1,0}, rh={3,3}, sol;
    nurbsknot::solve_tridiagonal(1, 2, lo, di, up, rh, sol);
    MINI_CHECK(TOLERANCE.is_allclose(sol, {1.0, 1.0}));
}

MINI_TEST("NurbsKnot", "Compute Parameters") {
    // uncomment #include "nurbsknot.h"

    double pts[] = {0,0,0, 1,0,0, 2,0,0, 3,0,0};
    // Chord-length parameterization: since all gaps are 1.0, params = {0, 1, 2, 3}
    std::vector<double> t = nurbsknot::compute_parameters(pts, 4, 3, CurveNurbsKnotStyle::Chord);
    MINI_CHECK(TOLERANCE.is_allclose(t, {0.0, 1.0, 2.0, 3.0}));
}

MINI_TEST("NurbsKnot", "Build Interpolation NurbsKnots") {
    // uncomment #include "nurbsknot.h"

    std::vector<double> params = {0.0, 1.0, 2.0, 3.0};
    int degree = 3;
    // cv_count = n + 2 = 6 (natural end conditions add 2 CVs)
    // kc = order + cv_count - 2 = 4 + 6 - 2 = 8
    //   [0, 0, 0,  |  1, 2,  |  3, 3, 3]
    //   <-clamp->    interior    <-clamp->
    std::vector<double> nurbsknots = nurbsknot::build_interp_nurbsknots(params, degree);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0}));
}

MINI_TEST("NurbsKnot", "Evaluation Basis") {
    // uncomment #include "nurbsknot.h"

    // Cox-de Boor recursive evaluation of B-spline basis functions
    // At parameter t, exactly 'order' basis functions are non-zero
    // Partition of unity: they always sum to 1.0
    // Used to evaluate NURBS curves/surfaces: C(t) = sum(N_i(t) * P_i)
    // 0 0 0 1 2 2 2
    int order = 4;
    int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);
    int span = nurbsknot::find_span(order, cv_count, nurbsknots, 0.5);
    std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, span, 0.5);
    MINI_CHECK(TOLERANCE.is_allclose(basis, {0.125, 0.59375, 0.25, 0.03125}));
}

MINI_TEST("NurbsKnot", "Build Fitted NurbsKnots Adaptive") {
    // uncomment #include "nurbsknot.h"

    // Builds nurbsknot vectors for least-squares fitting
    // Concentrates nurbsknots where curvature is high (sharp turns)
    // For collinear points (zero curvature), interior nurbsknots are evenly distributed
    double pts[] = {0,0,0, 1,0,0, 2,0,0, 3,0,0, 4,0,0};
    std::vector<double> params = nurbsknot::compute_parameters(pts, 5, 3, CurveNurbsKnotStyle::Chord);
    std::vector<double> nurbsknots = nurbsknot::build_fitted_nurbsknots_adaptive(params, pts, 5, 3, 5, 3);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 2.0, 4.0, 4.0, 4.0}));
}

MINI_TEST("NurbsKnot", "Build Fitted NurbsKnots Periodic Adaptive") {
    // uncomment #include "nurbsknot.h"

    // Periodic version for closed curves -- nurbsknots wrap around
    // For a regular square (equal turns, equal chords), nurbsknots are uniformly spaced
    double pts[] = {0,0,0, 1,0,0, 1,1,0, 0,1,0};
    std::vector<double> params = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> nurbsknots = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(params, pts, 4, 3, 4, 3);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
}

MINI_TEST("NurbsKnot", "Solve Banded SPD") {
    // uncomment #include "nurbsknot.h"

    // Cholesky solver for banded symmetric positive-definite systems
    //   | 4 2 0 |       |8 |       |1|
    //   | 2 5 1 | * x = |13| -> x = |2|
    //   | 0 1 3 |       |5 |       |1|
    std::vector<double> band = {4, 0, 5, 2, 3, 1};
    std::vector<double> rhs = {8, 13, 5};
    nurbsknot::solve_banded_spd(1, 3, 1, band, rhs);
    MINI_CHECK(TOLERANCE.is_allclose(rhs, {1.0, 2.0, 1.0}));
}

} // namespace session_cpp
