/**
 * @file knot.h
 * @brief Knot vector utility functions for NURBS curves and surfaces.
 * 
 * This module provides standalone functions for working with knot vectors,
 * following the OpenNURBS pattern (opennurbs_knot.h).
 * 
 * These functions operate on raw double arrays and can be used independently
 * or called by NurbsCurve and NurbsSurface classes.
 */

#ifndef SESSION_KNOT_H
#define SESSION_KNOT_H

#include <vector>
#include <cmath>
#include <utility>

namespace session_cpp {

/**
 * @brief Knot vector utility functions.
 * 
 * All functions are static and operate on knot vector arrays.
 * This matches the OpenNURBS opennurbs_knot.h pattern.
 */
namespace knot {

    /**
     * @brief Compute the number of knots in a knot vector.
     * @param order Order of the NURBS (degree + 1), must be >= 2.
     * @param cv_count Number of control vertices, must be >= order.
     * @return Number of knots: order + cv_count - 2
     */
    inline int knot_count(int order, int cv_count) {
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
     * @brief Create a clamped uniform knot vector.
     * @param order Order of the NURBS (degree + 1), must be >= 2.
     * @param cv_count Number of control vertices, must be >= order.
     * @param delta Spacing between interior knots. Default is 1.0.
     * @return Clamped uniform knot vector, or empty if invalid params.
     */
    std::vector<double> make_clamped_uniform(int order, int cv_count, double delta = 1.0);

    /**
     * @brief Create a periodic uniform knot vector.
     * @param order Order of the NURBS (degree + 1), must be >= 2.
     * @param cv_count Number of control vertices, must be >= order.
     * @param delta Spacing between knots. Default is 1.0.
     * @return Periodic uniform knot vector, or empty if invalid params.
     */
    std::vector<double> make_periodic_uniform(int order, int cv_count, double delta = 1.0);

    /**
     * @brief Clamp the ends of a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector to clamp (modified in place).
     * @param end Which end to clamp: 0 = left, 1 = right, 2 = both.
     * @return True if successful.
     */
    bool clamp(int order, int cv_count, std::vector<double>& knot, int end = 2);

    /**
     * @brief Check if a knot vector is valid.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector to validate.
     * @return True if the knot vector is valid.
     */
    bool is_valid(int order, int cv_count, const std::vector<double>& knot);

    /**
     * @brief Check if a knot vector is clamped.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector to check.
     * @param end Which end to check: 0 = left, 1 = right, 2 = both.
     * @return True if the knot vector is clamped at the specified end(s).
     */
    bool is_clamped(int order, int cv_count, const std::vector<double>& knot, int end = 2);

    /**
     * @brief Check if a knot vector is periodic.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector to check.
     * @return True if the knot vector is periodic.
     */
    bool is_periodic(int order, int cv_count, const std::vector<double>& knot);

    /**
     * @brief Check if a knot vector is uniform (interior knots evenly spaced).
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector to check.
     * @return True if the interior knots are uniformly spaced.
     */
    bool is_uniform(int order, int cv_count, const std::vector<double>& knot);

    /**
     * @brief Get the domain of a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @return Pair (t0, t1) domain interval.
     */
    std::pair<double, double> get_domain(int order, int cv_count, const std::vector<double>& knot);

    /**
     * @brief Set the domain of a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector (modified in place).
     * @param t0 New domain start.
     * @param t1 New domain end.
     * @return True if successful.
     */
    bool set_domain(int order, int cv_count, std::vector<double>& knot, double t0, double t1);

    /**
     * @brief Reverse a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector (modified in place).
     * @return True if successful.
     */
    bool reverse(int order, int cv_count, std::vector<double>& knot);

    /**
     * @brief Get the multiplicity of a knot.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @param knot_index Index of the knot to check.
     * @return Multiplicity of the knot at the given index.
     */
    int multiplicity(int order, int cv_count, const std::vector<double>& knot, int knot_index);

    /**
     * @brief Get the number of spans in a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @return Number of non-empty spans.
     */
    int span_count(int order, int cv_count, const std::vector<double>& knot);

    /**
     * @brief Get the span breakpoints of a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @return Vector of unique knot values that define span boundaries.
     */
    std::vector<double> get_span_vector(int order, int cv_count, const std::vector<double>& knot);

    /**
     * @brief Find the knot span containing parameter t.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @param t Parameter value to locate.
     * @param side When t is at a knot: 0 = from above, -1 = from below, 1 = from above.
     * @param hint Search hint (not used in this implementation).
     * @return Span index in range [0, cv_count - order].
     */
    int find_span(int order, int cv_count, const std::vector<double>& knot, 
                  double t, int side = 0, int hint = 0);

    /**
     * @brief Get the superfluous knot value at the specified end.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @param end 0 = first superfluous knot, 1 = last superfluous knot.
     * @return Superfluous knot value.
     */
    double superfluous_knot(int order, int cv_count, const std::vector<double>& knot, int end);

    /**
     * @brief Compute a single Greville abscissa.
     * @param order Order of the NURBS (degree + 1).
     * @param knot Array of (order - 1) knot values.
     * @return Greville abscissa (average of the knots).
     */
    double greville_abcissa(int order, const double* knot);

    /**
     * @brief Get all Greville abscissae for a knot vector.
     * @param order Order of the NURBS (degree + 1).
     * @param cv_count Number of control vertices.
     * @param knot Knot vector.
     * @param periodic True for periodic curves.
     * @return Vector of Greville abscissae.
     */
    std::vector<double> get_greville_abcissae(int order, int cv_count, 
                                               const std::vector<double>& knot,
                                               bool periodic = false);

} // namespace knot

} // namespace session_cpp

#endif // SESSION_KNOT_H
