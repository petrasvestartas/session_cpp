#include <iostream>
#include "knot.h"
using namespace session_cpp;

int main() {
    int order = 4, cv = 5;

    // knot_count
    std::cout << knot::knot_count(order, cv) << "\n"; // 7

    // domain_tolerance
    std::cout << knot::domain_tolerance(0, 10) << "\n"; // ~2.98e-07

    // make_clamped_uniform
    auto k = knot::make_clamped_uniform(order, cv);
    for (auto v : k) std::cout << v << " "; std::cout << "\n"; // 0 0 0 1 2 2 2

    // make_periodic_uniform
    auto p = knot::make_periodic_uniform(order, cv);
    for (auto v : p) std::cout << v << " "; std::cout << "\n"; // 0 1 2 3 4 5 6

    // clamp
    auto c = p;
    knot::clamp(order, cv, c, 2);
    for (auto v : c) std::cout << v << " "; std::cout << "\n"; // 2 2 2 3 4 4 4

    // is_valid
    std::cout << knot::is_valid(order, cv, k) << " " << knot::is_valid(2, 2, {}) << "\n"; // 1 0

    // is_clamped
    std::cout << knot::is_clamped(order, cv, k, 2) << " " << knot::is_clamped(order, cv, p, 2) << "\n"; // 1 0

    // is_periodic
    std::cout << knot::is_periodic(order, cv, k) << " " << knot::is_periodic(order, cv, p) << "\n"; // 0 1

    // get_domain
    auto [t0, t1] = knot::get_domain(order, cv, k);
    std::cout << t0 << " " << t1 << "\n"; // 0 2

    // set_domain
    auto d = k;
    knot::set_domain(order, cv, d, 5, 15);
    for (auto v : d) std::cout << v << " "; std::cout << "\n"; // 5 5 5 10 15 15 15

    // reverse
    auto r = k;
    knot::reverse(order, cv, r);
    for (auto v : r) std::cout << v << " "; std::cout << "\n"; // 0 0 0 1 2 2 2

    // multiplicity
    std::cout << knot::multiplicity(order, cv, k, 0) << " " << knot::multiplicity(order, cv, k, 3) << "\n"; // 3 1

    // span_count
    std::cout << knot::span_count(order, cv, k) << "\n"; // 2

    // find_span
    std::cout << knot::find_span(order, cv, k, 0.5) << " " << knot::find_span(order, cv, k, 1.5) << "\n"; // 0 1

    // get_greville_abcissae
    auto g = knot::get_greville_abcissae(order, cv, k);
    for (auto v : g) std::cout << v << " "; std::cout << "\n"; // 0 0.333 1 1.667 2

    // solve_tridiagonal
    std::vector<double> lo={0,1}, di={2,2}, up={1,0}, rh={3,3}, sol;
    knot::solve_tridiagonal(1, 2, lo, di, up, rh, sol);
    std::cout << sol[0] << " " << sol[1] << "\n"; // 1 1

    // compute_parameters
    double pts[] = {0,0,0, 1,0,0, 2,0,0, 3,0,0};
    auto par = knot::compute_parameters(pts, 4, 3, CurveKnotStyle::Chord);
    for (auto v : par) std::cout << v << " "; std::cout << "\n"; // 0 1 2 3

    // build_interp_knots
    auto ik = knot::build_interp_knots(par, 3);
    for (auto v : ik) std::cout << v << " "; std::cout << "\n";

    // eval_basis
    int span = knot::find_span(order, cv, k, 1.0);
    auto b = knot::eval_basis(order, k, span, 1.0);
    for (auto v : b) std::cout << v << " "; std::cout << "\n"; // partition of unity

    // build_fitted_knots_adaptive
    auto fk = knot::build_fitted_knots_adaptive(par, pts, 4, 3, 4, 3);
    std::cout << fk.size() << "\n";

    // build_fitted_knots_periodic_adaptive — n=4 unique points, params has 5 entries (including closing segment)
    double cpts[] = {1,0,0, 0,1,0, -1,0,0, 0,-1,0, 1,0,0};
    auto cpar = knot::compute_parameters(cpts, 5, 3, CurveKnotStyle::Chord);
    auto fpk = knot::build_fitted_knots_periodic_adaptive(cpar, cpts, 4, 3, 4, 3);
    std::cout << fpk.size() << "\n";

    // solve_banded_spd — 3x3 tridiagonal SPD: [4 1 0; 1 4 1; 0 1 4] x = [5 6 5]
    int bw = 1, n = 3;
    std::vector<double> band(n * (bw + 1)); // lower triangle in banded storage
    band[0*2+0]=4; band[0*2+1]=0; // row0: diag=4, no sub-diag
    band[1*2+0]=4; band[1*2+1]=1; // row1: diag=4, sub=A[1][0]=1
    band[2*2+0]=4; band[2*2+1]=1; // row2: diag=4, sub=A[2][1]=1
    std::vector<double> rhs2 = {5, 6, 5};
    knot::solve_banded_spd(1, n, bw, band, rhs2);
    for (auto v : rhs2) std::cout << v << " "; std::cout << "\n"; // 1 1 1

    return 0;
}
