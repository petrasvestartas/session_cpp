/**
 * @file nurbsknot.h
 * @brief NurbsKnot vector utility functions for NURBS curves and surfaces.
 * 
 * This module provides standalone functions for working with nurbsknot vectors,
 * following the OpenNURBS pattern (opennurbs_nurbsknot.h).
 * 
 * These functions operate on raw double arrays and can be used independently
 * or called by NurbsCurve and NurbsSurface classes.
 */

#ifndef SESSION_NURBSKNOT_H
#define SESSION_NURBSKNOT_H

#include <vector>
#include <cmath>
#include <utility>

namespace session_cpp {

/**
 * @brief NurbsKnot spacing style for interpolated curves.
 * Matches Rhino's CurveNurbsKnotStyle enum.
 */
enum class CurveNurbsKnotStyle {
    Uniform = 0,              // Parameter spacing = 1.0
    Chord = 1,                // Chord-length parameterization
    ChordSquareRoot = 2,      // Centripetal (sqrt chord) parameterization
    UniformPeriodic = 3,      // Periodic + uniform
    ChordPeriodic = 4,        // Periodic + chord
    ChordSquareRootPeriodic = 5  // Periodic + centripetal
};

/**
 * @brief NurbsKnot vector utility functions.
 *
 * All functions are static and operate on nurbsknot vector arrays.
 * This matches the OpenNURBS opennurbs_nurbsknot.h pattern.
 */
namespace nurbsknot {

    /**
     * @brief Compute the number of nurbsknots in a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1), must be >= 2.
     * @param cv_count Number of control vertices, must be >= order.
     * @return Number of nurbsknots: order + cv_count - 2
     */
    inline int nurbsknot_count(int order, int cv_count) {
        return order + cv_count - 2;
    }

    /**
     * @brief Compute tolerance associated with a domain interval.
     * @param a Start of domain.
     * @param b End of domain.
     * @return Tolerance value.
     */
    inline double domain_tolerance(double a, double b) {
        if (a == b) return 0.0;
        const double SQRT_EPSILON = 1.4901161193847656e-08;
        const double EPSILON = 2.220446049250313e-16;
        double tol = (std::fabs(a) + std::fabs(b) + std::fabs(a - b)) * SQRT_EPSILON;
        if (tol < EPSILON) tol = EPSILON;
        return tol;
    }

    /**
     * @brief Create a clamped uniform nurbsknot vector.
     * @param order Order of the NURBS (degree + 1), must be >= 2.
     * @param cv_count Number of control vertices, must be >= order.
     * @param delta Spacing between interior nurbsknots. Default is 1.0.
     * @return Clamped uniform nurbsknot vector, or empty if invalid params.
     */
    std::vector<double> make_clamped_uniform(int order, int cv_count, double delta = 1.0);

    /**
     * @brief Create a periodic uniform nurbsknot vector.
     * @param order Order of the NURBS (degree + 1), must be >= 2.
     * @param cv_count Number of control vertices, must be >= order.
     * @param delta Spacing between nurbsknots. Default is 1.0.
     * @return Periodic uniform nurbsknot vector, or empty if invalid params.
     */
    std::vector<double> make_periodic_uniform(int order, int cv_count, double delta = 1.0);

    /**
     * @brief Clamp the ends of a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector to clamp (modified in place).
     * @param end Which end to clamp: 0 = left, 1 = right, 2 = both.
     * @return True if successful.
     */
    bool clamp(int order, int cv_count, std::vector<double>& nurbsknot, int end = 2);

    /**
     * @brief Check if a nurbsknot vector is valid.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector to validate.
     * @return True if the nurbsknot vector is valid.
     */
    bool is_valid(int order, int cv_count, const std::vector<double>& nurbsknot);

    /**
     * @brief Check if a nurbsknot vector is clamped.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector to check.
     * @param end Which end to check: 0 = left, 1 = right, 2 = both.
     * @return True if the nurbsknot vector is clamped at the specified end(s).
     */
    bool is_clamped(int order, int cv_count, const std::vector<double>& nurbsknot, int end = 2);

    /**
     * @brief Check if a nurbsknot vector is periodic.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector to check.
     * @return True if the nurbsknot vector is periodic.
     */
    bool is_periodic(int order, int cv_count, const std::vector<double>& nurbsknot);

    /**
     * @brief Get the domain of a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector.
     * @return Pair (t0, t1) domain interval.
     */
    std::pair<double, double> get_domain(int order, int cv_count, const std::vector<double>& nurbsknot);

    /**
     * @brief Set the domain of a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector (modified in place).
     * @param t0 New domain start.
     * @param t1 New domain end.
     * @return True if successful.
     */
    bool set_domain(int order, int cv_count, std::vector<double>& nurbsknot, double t0, double t1);

    /**
     * @brief Reverse a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector (modified in place).
     * @return True if successful.
     */
    bool reverse(int order, int cv_count, std::vector<double>& nurbsknot);

    /**
     * @brief Get the multiplicity of a nurbsknot.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector.
     * @param nurbsknot_index Index of the nurbsknot to check.
     * @return Multiplicity of the nurbsknot at the given index.
     */
    int multiplicity(int order, int cv_count, const std::vector<double>& nurbsknot, int nurbsknot_index);

    /**
     * @brief Get the number of spans in a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector.
     * @return Number of non-empty spans.
     */
    int span_count(int order, int cv_count, const std::vector<double>& nurbsknot);

    /**
     * @brief Find the nurbsknot span containing parameter t.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector.
     * @param t Parameter value to locate.
     * @param side When t is at a nurbsknot: 0 = from above, -1 = from below, 1 = from above.
     * @param hint Search hint (not used in this implementation).
     * @return Span index in range [0, cv_count - order].
     */
    int find_span(int order, int cv_count, const std::vector<double>& nurbsknot, 
                  double t, int side = 0, int hint = 0);

    /**
     * @brief Get all Greville abscissae for a nurbsknot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param nurbsknot NurbsKnot vector.
     * @param periodic True for periodic curves.
     * @return Vector of Greville abscissae.
     */
    std::vector<double> get_greville_abcissae(int order, int cv_count, 
                                               const std::vector<double>& nurbsknot,
                                               bool periodic = false);

    /**
     * @brief Solve tridiagonal linear system using Thomas algorithm.
     * This is the core solver for NURBS curve interpolation.
     * @param dim Dimension of each variable (e.g., 3 for 3D points).
     * @param n Number of equations.
     * @param lower Lower diagonal coefficients (length n, first element unused).
     * @param diag Main diagonal coefficients (length n).
     * @param upper Upper diagonal coefficients (length n, last element unused).
     * @param rhs Right-hand side values (length n * dim).
     * @param solution Output solution (length n * dim).
     * @return True if successful, false if singular.
     */
    bool solve_tridiagonal(int dim, int n,
                           std::vector<double>& lower,
                           std::vector<double>& diag,
                           std::vector<double>& upper,
                           const std::vector<double>& rhs,
                           std::vector<double>& solution);

    /**
     * @brief Compute parameters for interpolation based on nurbsknot style.
     * @param points Input points (flat array: x0,y0,z0,x1,y1,z1,...).
     * @param point_count Number of points.
     * @param dim Dimension (2 or 3).
     * @param style NurbsKnot style (Uniform, Chord, or ChordSquareRoot).
     * @return Parameter values for each point.
     */
    std::vector<double> compute_parameters(const double* points, int point_count,
                                           int dim, CurveNurbsKnotStyle style);

    /**
     * @brief Build clamped nurbsknot vector from parameters for interpolation.
     * @param params Parameter values for each input point.
     * @param degree Curve degree.
     * @return NurbsKnot vector for interpolated curve.
     */
    std::vector<double> build_interp_nurbsknots(const std::vector<double>& params, int degree);

    /**
     * @brief Evaluate B-spline basis functions at parameter t (Cox-de Boor).
     * @param order Order of the B-spline (degree + 1).
     * @param nurbsknot Full nurbsknot vector.
     * @param span Span index (from find_span).
     * @param t Parameter value.
     * @return Vector of 'order' basis function values.
     */
    std::vector<double> eval_basis(int order, const std::vector<double>& nurbsknot, int span, double t);

    std::vector<double> build_fitted_nurbsknots_adaptive(const std::vector<double>& params,
        const double* points, int point_count, int dim, int num_cvs, int degree, double scale = 3.0);

    std::vector<double> build_fitted_nurbsknots_periodic_adaptive(const std::vector<double>& params,
        const double* points, int n, int dim, int num_cvs, int degree, double scale = 3.0);

    /**
     * @brief Solve banded symmetric positive-definite system via Cholesky.
     * @param dim Dimension of each variable (e.g. 3 for 3D).
     * @param n Number of equations.
     * @param half_bw Half-bandwidth of the matrix.
     * @param band Compact lower-triangle storage (n*(half_bw+1)), modified in place.
     * @param rhs Right-hand side (n*dim), replaced by solution in place.
     * @return True if successful, false if not positive definite.
     */
    bool solve_banded_spd(int dim, int n, int half_bw, std::vector<double>& band, std::vector<double>& rhs);

} // namespace nurbsknot

} // namespace session_cpp

#endif // SESSION_NURBSKNOT_H
