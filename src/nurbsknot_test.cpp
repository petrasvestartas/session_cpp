#include "mini_test.h"
#include "nurbsknot.h"
#include "tolerance.h"
#include <cmath>
#include <limits>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("NurbsKnot", "Nurbsknot Count") {

    MINI_CHECK(nurbsknot::nurbsknot_count(4, 5) == 7);
    MINI_CHECK(nurbsknot::nurbsknot_count(0, 0) == 0);
    MINI_CHECK(nurbsknot::nurbsknot_count(4, 3) == 0);
    MINI_CHECK(nurbsknot::nurbsknot_count(2, std::numeric_limits<int>::max()) == std::numeric_limits<int>::max());
    MINI_CHECK(nurbsknot::nurbsknot_count(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()) == 0);
}

MINI_TEST("NurbsKnot", "Domain Tolerance") {

    MINI_CHECK(nurbsknot::domain_tolerance(1.0, 1.0) == 0.0);
    MINI_CHECK(TOLERANCE.is_close(nurbsknot::domain_tolerance(0.0, 1.0), 2.980232238769531e-08));
    MINI_CHECK(nurbsknot::domain_tolerance(0.0, std::numeric_limits<double>::denorm_min()) == std::numeric_limits<double>::epsilon());
}

MINI_TEST("NurbsKnot", "Make Clamped Uniform") {

    const int order = 4;
    const int cv_count = 5;
    const std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);

    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));
    MINI_CHECK(nurbsknot::make_clamped_uniform(1, cv_count).empty());
    MINI_CHECK(nurbsknot::make_clamped_uniform(order, cv_count, std::numeric_limits<double>::quiet_NaN()).empty());
    MINI_CHECK(nurbsknot::make_clamped_uniform(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()).empty());
}

MINI_TEST("NurbsKnot", "Make Periodic Uniform") {

    const int order = 4;
    const int cv_count = 5;
    const std::vector<double> nurbsknots = nurbsknot::make_periodic_uniform(order, cv_count);

    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
    MINI_CHECK(nurbsknot::make_periodic_uniform(order, cv_count, 0.0).empty());
    MINI_CHECK(nurbsknot::make_periodic_uniform(order, cv_count, std::numeric_limits<double>::infinity()).empty());
}

MINI_TEST("NurbsKnot", "Clamp") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots = {9.0, 9.0, 0.0, 1.0, 2.0, 9.0, 9.0};
    const bool ok = nurbsknot::clamp(order, cv_count, nurbsknots);

    MINI_CHECK(ok);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));

    std::vector<double> left = {9.0, 9.0, 0.0, 1.0, 2.0, 8.0, 9.0};
    std::vector<double> right = {9.0, 8.0, 0.0, 1.0, 2.0, 9.0, 9.0};

    MINI_CHECK(nurbsknot::clamp(order, cv_count, left, 0));
    MINI_CHECK(nurbsknot::clamp(order, cv_count, right, 1));
    MINI_CHECK(TOLERANCE.is_allclose(left, {0.0, 0.0, 0.0, 1.0, 2.0, 8.0, 9.0}));
    MINI_CHECK(TOLERANCE.is_allclose(right, {9.0, 8.0, 0.0, 1.0, 2.0, 2.0, 2.0}));
    MINI_CHECK(!nurbsknot::clamp(order, cv_count, right, 3));
}

MINI_TEST("NurbsKnot", "Is Valid") {

    const int order = 4;
    const int cv_count = 5;
    const std::vector<double> nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count);
    const std::vector<double> nurbsknots_flat = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> nurbsknots_nan = nurbsknots_clamped;
    nurbsknots_nan[3] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(nurbsknot::is_valid(order, cv_count, nurbsknots_clamped));
    MINI_CHECK(!nurbsknot::is_valid(order, cv_count, nurbsknots_flat));
    MINI_CHECK(!nurbsknot::is_valid(order, cv_count, nurbsknots_nan));
    MINI_CHECK(!nurbsknot::is_valid(order, cv_count, {0.0, 1.0}));
}

MINI_TEST("NurbsKnot", "Is Clamped") {

    const int order = 4;
    const int cv_count = 5;
    const std::vector<double> nurbsknots_periodic = nurbsknot::make_periodic_uniform(order, cv_count);
    const std::vector<double> nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count);
    const bool is_not_clamped = nurbsknot::is_clamped(order, cv_count, nurbsknots_periodic);
    const bool is_clamped = nurbsknot::is_clamped(order, cv_count, nurbsknots_clamped);

    MINI_CHECK(!is_not_clamped && is_clamped);
    MINI_CHECK(nurbsknot::is_clamped(order, cv_count, nurbsknots_clamped, 0));
    MINI_CHECK(nurbsknot::is_clamped(order, cv_count, nurbsknots_clamped, 1));
    MINI_CHECK(!nurbsknot::is_clamped(order, cv_count, nurbsknots_clamped, 3));
}

MINI_TEST("NurbsKnot", "Is Periodic") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots_periodic = nurbsknot::make_periodic_uniform(order, cv_count);
    const std::vector<double> nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count);

    MINI_CHECK(nurbsknot::is_periodic(order, cv_count, nurbsknots_periodic));
    MINI_CHECK(!nurbsknot::is_periodic(order, cv_count, nurbsknots_clamped));

    nurbsknots_periodic[3] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(!nurbsknot::is_periodic(order, cv_count, nurbsknots_periodic));
}

MINI_TEST("NurbsKnot", "Get Domain") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);
    const std::pair<double, double> domain = nurbsknot::get_domain(order, cv_count, nurbsknots);

    MINI_CHECK(TOLERANCE.is_close(domain.first, 0.0));
    MINI_CHECK(TOLERANCE.is_close(domain.second, 2.0));

    nurbsknots[3] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(nurbsknot::get_domain(order, cv_count, nurbsknots) == domain);

    nurbsknots[2] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(nurbsknot::get_domain(order, cv_count, nurbsknots) == std::make_pair(0.0, 0.0));
}

MINI_TEST("NurbsKnot", "Set Domain") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);
    const bool ok = nurbsknot::set_domain(order, cv_count, nurbsknots, 0.0, 1.0);

    MINI_CHECK(ok);
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0}));
    MINI_CHECK(!nurbsknot::set_domain(order, cv_count, nurbsknots, 1.0, 1.0));
    MINI_CHECK(!nurbsknot::set_domain(order, cv_count, nurbsknots, 0.0, std::numeric_limits<double>::quiet_NaN()));
}

MINI_TEST("NurbsKnot", "Reverse") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots_sym = nurbsknot::make_clamped_uniform(order, cv_count);

    MINI_CHECK(nurbsknot::reverse(order, cv_count, nurbsknots_sym));
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots_sym, {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0}));

    std::vector<double> nurbsknots_asym = {0.0, 0.0, 0.0, 0.5, 1.0, 2.0, 2.0, 2.0};

    MINI_CHECK(nurbsknot::reverse(4, 6, nurbsknots_asym));
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots_asym, {0.0, 0.0, 0.0, 1.0, 1.5, 2.0, 2.0, 2.0}));

    nurbsknots_asym[3] = std::numeric_limits<double>::infinity();

    MINI_CHECK(!nurbsknot::reverse(4, 6, nurbsknots_asym));
}

MINI_TEST("NurbsKnot", "Multiplicity") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);

    MINI_CHECK(nurbsknot::multiplicity(order, cv_count, nurbsknots, 0) == 3);
    MINI_CHECK(nurbsknot::multiplicity(order, cv_count, nurbsknots, 3) == 1);
    MINI_CHECK(nurbsknot::multiplicity(order, cv_count, nurbsknots, 7) == 0);

    nurbsknots[3] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(nurbsknot::multiplicity(order, cv_count, nurbsknots, 3) == 0);
}

MINI_TEST("NurbsKnot", "Span Count") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);

    MINI_CHECK(nurbsknot::span_count(order, cv_count, nurbsknots) == 2);

    nurbsknots[3] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(nurbsknot::span_count(order, cv_count, nurbsknots) == 0);
}

MINI_TEST("NurbsKnot", "Find Span") {

    const int order = 4;
    const int cv_count = 5;
    const std::vector<double> nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count);
    const int spancount0 = nurbsknot::find_span(order, cv_count, nurbsknots_clamped, 0.5);
    const int spancount1 = nurbsknot::find_span(order, cv_count, nurbsknots_clamped, 1.5);

    MINI_CHECK(spancount0 == 0 && spancount1 == 1);
    MINI_CHECK(nurbsknot::find_span(order, cv_count, nurbsknots_clamped, -1.0) == 0);
    MINI_CHECK(nurbsknot::find_span(order, cv_count, nurbsknots_clamped, 3.0) == 1);
    MINI_CHECK(nurbsknot::find_span(order, cv_count, nurbsknots_clamped, 0.5, -1, 42) == 0);
    MINI_CHECK(nurbsknot::find_span(order, cv_count, nurbsknots_clamped, std::numeric_limits<double>::quiet_NaN()) == 0);
}

MINI_TEST("NurbsKnot", "Get Greville Abcissae") {

    const int order = 4;
    const int cv_count = 5;
    std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);
    const std::vector<double> greville = nurbsknot::get_greville_abcissae(order, cv_count, nurbsknots);
    const std::vector<double> periodic = nurbsknot::get_greville_abcissae(order, cv_count, nurbsknots, true);

    MINI_CHECK(TOLERANCE.is_allclose(greville, {0.0, 1.0 / 3.0, 1.0, 5.0 / 3.0, 2.0}));
    MINI_CHECK(TOLERANCE.is_allclose(periodic, {0.0, 1.0 / 3.0}));

    nurbsknots[2] = std::numeric_limits<double>::infinity();

    MINI_CHECK(nurbsknot::get_greville_abcissae(order, cv_count, nurbsknots).empty());
}

MINI_TEST("NurbsKnot", "Solve Tridiagonal") {

    std::vector<double> lo = {0.0, 1.0};
    std::vector<double> di = {2.0, 2.0};
    std::vector<double> up = {1.0, 0.0};
    const std::vector<double> rh = {3.0, 3.0};
    std::vector<double> sol;

    MINI_CHECK(nurbsknot::solve_tridiagonal(1, 2, lo, di, up, rh, sol));
    MINI_CHECK(TOLERANCE.is_allclose(sol, {1.0, 1.0}));

    const std::vector<double> rh2 = {3.0, 0.0, 3.0, 3.0};

    MINI_CHECK(nurbsknot::solve_tridiagonal(2, 2, lo, di, up, rh2, sol));
    MINI_CHECK(TOLERANCE.is_allclose(sol, {1.0, -1.0, 1.0, 2.0}));

    std::vector<double> singular = {0.0, 2.0};

    MINI_CHECK(!nurbsknot::solve_tridiagonal(1, 2, lo, singular, up, rh, sol));
    MINI_CHECK(!nurbsknot::solve_tridiagonal(std::numeric_limits<int>::max(), 2, lo, di, up, rh, sol));
}

MINI_TEST("NurbsKnot", "Compute Parameters") {

    const std::vector<double> pts = {0.0, 0.0, 4.0, 0.0, 4.0, 9.0};
    const std::vector<double> uniform = nurbsknot::compute_parameters(pts.data(), 3, 2, CurveNurbsKnotStyle::Uniform);
    const std::vector<double> chord = nurbsknot::compute_parameters(pts.data(), 3, 2, CurveNurbsKnotStyle::Chord);
    const std::vector<double> root = nurbsknot::compute_parameters(pts.data(), 3, 2, CurveNurbsKnotStyle::ChordSquareRoot);
    const std::vector<double> periodic = nurbsknot::compute_parameters(pts.data(), 3, 2, CurveNurbsKnotStyle::ChordPeriodic);

    MINI_CHECK(TOLERANCE.is_allclose(uniform, {0.0, 1.0, 2.0}));
    MINI_CHECK(TOLERANCE.is_allclose(chord, {0.0, 4.0, 13.0}));
    MINI_CHECK(TOLERANCE.is_allclose(root, {0.0, 2.0, 5.0}));
    MINI_CHECK(TOLERANCE.is_allclose(periodic, chord));
    MINI_CHECK(static_cast<int>(CurveInterpStyle::Rhino) == 0 && static_cast<int>(CurveInterpStyle::Occt) == 1);
    MINI_CHECK(nurbsknot::compute_parameters(nullptr, 3, 2, CurveNurbsKnotStyle::Chord).empty());
}

MINI_TEST("NurbsKnot", "Build Interp Nurbsknots") {

    std::vector<double> params = {0.0, 1.0, 2.0, 3.0};
    const int degree = 3;
    const std::vector<double> nurbsknots = nurbsknot::build_interp_nurbsknots(params, degree);

    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0}));

    params[2] = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(nurbsknot::build_interp_nurbsknots(params, degree).empty());
    MINI_CHECK(nurbsknot::build_interp_nurbsknots({0.0, 1.0, 2.0}, 5).empty());
}

MINI_TEST("NurbsKnot", "Eval Basis") {

    const int order = 4;
    const int cv_count = 5;
    const std::vector<double> nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count);
    const int span = nurbsknot::find_span(order, cv_count, nurbsknots, 0.5);
    const std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, span, 0.5);
    const double nan = std::numeric_limits<double>::quiet_NaN();

    MINI_CHECK(TOLERANCE.is_allclose(basis, {0.125, 0.59375, 0.25, 0.03125}));
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknot::eval_basis(1, {}, 0, 0.5), {1.0}));
    MINI_CHECK(nurbsknot::eval_basis(0, {}, 0, 0.5).empty());
    MINI_CHECK(nurbsknot::eval_basis(order, {0.0}, span, 0.5).empty());
    MINI_CHECK(TOLERANCE.is_allclose(nurbsknot::eval_basis(3, {nan, -1.0, 0.0, 1.0, 2.0, 3.0}, 2, 1.5), {0.125, 0.75, 0.125}));
    MINI_CHECK(nurbsknot::eval_basis(3, {-2.0, -1.0, nan, 1.0, 2.0, 3.0}, 2, 1.5).empty());
}

MINI_TEST("NurbsKnot", "Build Fitted Nurbsknots Adaptive") {

    const std::vector<double> pts = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 3.0, 0.0, 0.0, 4.0, 0.0, 0.0};
    const std::vector<double> params = nurbsknot::compute_parameters(pts.data(), 5, 3, CurveNurbsKnotStyle::Chord);
    const std::vector<double> nurbsknots = nurbsknot::build_fitted_nurbsknots_adaptive(params, pts.data(), 5, 3, 5, 3);
    const std::vector<double> fallback = nurbsknot::build_fitted_nurbsknots_adaptive(params, nullptr, 5, 3, 5, 3);
    const std::vector<double> dense = nurbsknot::build_fitted_nurbsknots_adaptive({0.0, 1.0, 2.0}, pts.data(), 3, 3, 5, 1, 1.0);

    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {0.0, 0.0, 0.0, 2.0, 4.0, 4.0, 4.0}));
    MINI_CHECK(TOLERANCE.is_allclose(fallback, {0.0, 0.0, 0.0, 1.5, 4.0, 4.0, 4.0}));
    MINI_CHECK(nurbsknot::build_fitted_nurbsknots_adaptive(params, pts.data(), 5, 3, 3, 3).empty());
    MINI_CHECK(nurbsknot::build_fitted_nurbsknots_adaptive({0.0, 1.0}, nullptr, 2, 3, 4, 1, 1.0).empty());
    MINI_CHECK(TOLERANCE.is_allclose(dense, {0.0, 0.5, 1.0, 1.5, 2.0}));
}

MINI_TEST("NurbsKnot", "Build Fitted Nurbsknots Periodic Adaptive") {

    const std::vector<double> pts = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0};
    const std::vector<double> params = {0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> nurbsknots = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(params, pts.data(), 4, 3, 4, 3);
    const std::vector<double> fallback = nurbsknot::build_fitted_nurbsknots_periodic_adaptive({0.0, 1.0, 2.0}, nullptr, 2, 3, 4, 3);
    const std::vector<double> boundary = nurbsknot::build_fitted_nurbsknots_periodic_adaptive({0.0, 1.0, 2.0, 3.0}, pts.data(), 3, 3, 1, 2, 1.0);

    MINI_CHECK(TOLERANCE.is_allclose(nurbsknots, {-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
    MINI_CHECK(TOLERANCE.is_allclose(fallback, {-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0}));
    MINI_CHECK(nurbsknot::build_fitted_nurbsknots_periodic_adaptive({0.0}, nullptr, 0, 3, 4, 3).empty());
    MINI_CHECK(nurbsknot::build_fitted_nurbsknots_periodic_adaptive({0.0, 1.0, 2.0, 3.0}, pts.data(), 3, 3, 1, 3, 1.0).empty());
    MINI_CHECK(TOLERANCE.is_allclose(boundary, {-3.0, 0.0, 3.0, 6.0}));
}

MINI_TEST("NurbsKnot", "Solve Banded SPD") {

    std::vector<double> band = {4.0, 0.0, 5.0, 2.0, 3.0, 1.0};
    std::vector<double> rhs = {8.0, 13.0, 5.0};

    MINI_CHECK(nurbsknot::solve_banded_spd(1, 3, 1, band, rhs));
    MINI_CHECK(TOLERANCE.is_allclose(rhs, {1.0, 2.0, 1.0}));

    std::vector<double> singular = {0.0, 0.0};
    std::vector<double> value = {1.0};

    MINI_CHECK(!nurbsknot::solve_banded_spd(1, 1, 1, singular, value));
    MINI_CHECK(!nurbsknot::solve_banded_spd(1, 2, 1, singular, value));

    const double cutoff_value = Tolerance::ABSOLUTE * Tolerance::ABSOLUTE * Tolerance::ZERO_TOLERANCE;
    std::vector<double> cutoff = {cutoff_value};
    value = {1.0};

    MINI_CHECK(!nurbsknot::solve_banded_spd(1, 1, 0, cutoff, value));

    cutoff = {std::nextafter(cutoff_value, std::numeric_limits<double>::infinity())};
    value = {1.0};

    MINI_CHECK(nurbsknot::solve_banded_spd(1, 1, 0, cutoff, value));
    MINI_CHECK(!nurbsknot::solve_banded_spd(std::numeric_limits<int>::max(), 2, 1, singular, value));
}

} // namespace session_cpp
