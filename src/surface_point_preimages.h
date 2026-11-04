#pragma once
#include "nurbssurface.h"
#include "nurbs_coincidence.h"
#include "nurbs_interval.h"
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace session_cpp::surface_point_preimages {
// Original stored homogeneous witness, NOT the rounded public world Point.
using ProjectivePoint = std::array<double, 4>;
struct Domain {
    std::array<double, 4> uv{}; // u0,u1,v0,v1; point, collapsed edge, or collapsed cell
};
struct Result {
    std::vector<Domain> domains;
    bool complete = false;
    std::string failure;
};

// Sufficient Bernstein preimage certificate on clamped C0 tensor Bezier charts.
// Keeps distinct raw seam aliases; domains describe constant maps to the witness.
inline Result enumerate(const NurbsSurface &surface, const ProjectivePoint &point) {
    Result result;
    const auto fail = [&](const char *message) {
        result.failure = message;
        return result;
    };
    if (!surface.is_valid() || surface.dimension() != 3 || !(point[3] > 0))
        return fail("Invalid point preimage input");
    for (double value : point)
        if (!std::isfinite(value))
            return fail("Nonfinite projective witness");
    std::array<std::vector<double>, 2> knots;
    for (int d = 0; d < 2; ++d) {
        const int degree = surface.degree(d);
        if (degree < 1 || degree > 5)
            return fail("Unsupported preimage degree");
        const auto &source = surface.m_nurbsknot[d];
        for (size_t i = 0; i < source.size();) {
            size_t j = i + 1;
            while (j < source.size() && source[j] == source[i])
                ++j;
            if (j - i != static_cast<size_t>(degree) || !std::isfinite(source[i]))
                return fail("Preimage chart is not clamped C0 Bezier form");
            knots[d].push_back(source[i]);
            i = j;
        }
        if (knots[d].size() < 2 ||
            surface.cv_count(d) != static_cast<int>((knots[d].size() - 1) * degree + 1))
            return fail("Invalid preimage chart topology");
    }
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j)
            if (!(surface.weight(i, j) > 0) || !std::isfinite(surface.weight(i, j)))
                return fail("Preimage certificate requires positive weights");
    const auto sign = [&](int i, int j, int coordinate) {
        const double *cv = surface.cv(i, j);
        const double weight = surface.weight(i, j);
        if (nurbs_coincidence::exact_products_equal(cv[coordinate], point[3], point[coordinate], weight))
            return 0;
        const auto residual = nurbs_interval::Interval(cv[coordinate]) * nurbs_interval::Interval(point[3]) -
                              nurbs_interval::Interval(point[coordinate]) * nurbs_interval::Interval(weight);
        if (!nurbs_interval::finite(residual))
            return 2;
        return residual.lo > 0 ? 1 : residual.hi < 0 ? -1 : 2;
    };
    bool unresolved = false;
    const int p = surface.degree(0), q = surface.degree(1);
    for (size_t u = 0; u + 1 < knots[0].size(); ++u)
        for (size_t v = 0; v + 1 < knots[1].size(); ++v)
            // Each parameter direction partitions into left endpoint, OPEN interior,
            // right endpoint. Thus all nine disjoint strata are considered.
            for (int su = 0; su < 3; ++su)
                for (int sv = 0; sv < 3; ++sv) {
                    const int i0 = su == 2 ? p : 0, i1 = su == 0 ? 0 : p;
                    const int j0 = sv == 2 ? q : 0, j1 = sv == 0 ? 0 : q;
                    bool constant = true, excluded = false;
                    for (int d = 0; d < 3; ++d) {
                        bool nonnegative = true, nonpositive = true, positive = false, negative = false;
                        for (int i = i0; i <= i1; ++i)
                            for (int j = j0; j <= j1; ++j) {
                                const int s =
                                    sign(static_cast<int>(u) * p + i, static_cast<int>(v) * q + j, d);
                                constant = constant && s == 0;
                                nonnegative = nonnegative && (s == 0 || s == 1);
                                nonpositive = nonpositive && (s == 0 || s == -1);
                                positive = positive || s == 1;
                                negative = negative || s == -1;
                            }
                        excluded = excluded || (nonnegative && positive) || (nonpositive && negative);
                    }
                    if (constant) {
                        Domain domain{{su == 2 ? knots[0][u + 1] : knots[0][u],
                                       su == 0 ? knots[0][u] : knots[0][u + 1],
                                       sv == 2 ? knots[1][v + 1] : knots[1][v],
                                       sv == 0 ? knots[1][v] : knots[1][v + 1]}};
                        bool contained = false;
                        for (const auto &old : result.domains)
                            contained =
                                contained || (old.uv[0] <= domain.uv[0] && old.uv[1] >= domain.uv[1] &&
                                              old.uv[2] <= domain.uv[2] && old.uv[3] >= domain.uv[3]);
                        if (!contained)
                            result.domains.push_back(domain);
                    } else if (!excluded) {
                        unresolved = true;
                    }
                }
    result.complete = !unresolved;
    if (unresolved)
        result.failure = "Some original UV strata have unproved point preimages";
    return result;
}

enum class Membership { Included, Excluded, Unresolved };
// classify(domain) must certify the WHOLE raw domain (not its midpoint).
// Included is existential: a certified nonempty included subset suffices.
// Exclusion is universal and requires exhaustive preimages and every domain excluded.
template <typename Classify> Membership trimmed_membership(const Result &result, Classify &&classify) {
    bool unresolved = !result.complete;
    for (const auto &domain : result.domains) {
        const auto state = classify(domain);
        if (state == Membership::Included)
            return state;
        unresolved = unresolved || state == Membership::Unresolved;
    }
    return unresolved ? Membership::Unresolved : Membership::Excluded;
}
} // namespace session_cpp::surface_point_preimages
