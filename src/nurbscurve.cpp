#include "nurbscurve.h"
#include "intersection.h"
#include "knot.h"
#include <cstring>
#include <sstream>
#include <limits>
#include <fstream>

#include "nurbscurve.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Static Factory Methods
///////////////////////////////////////////////////////////////////////////////////////////

NurbsCurve NurbsCurve::create(bool periodic, int degree, const std::vector<Point>& points,
                              int dimension, double knot_delta) {
    NurbsCurve curve;
    int order = degree + 1;

    if (periodic) {
        curve.create_periodic_uniform(dimension, order, points, knot_delta);
    } else {
        curve.create_clamped_uniform(dimension, order, points, knot_delta);
    }

    // Arc-length parameterization: rescale domain to [0, arc_length]
    // Matches Rhino's CreateControlPointCurve behavior
    if (curve.is_valid()) {
        double L = curve.length();
        if (L > 0.0) {
            curve.set_domain(0.0, L);
        }
    }

    return curve;
}

NurbsCurve NurbsCurve::create_interpolated(const std::vector<Point>& points,
                                           CurveKnotStyle parameterization) {
    int n = static_cast<int>(points.size());
    if (n < 2) return NurbsCurve();
    int dim = 3;
    int degree = 3;
    int order = degree + 1;

    bool periodic = (parameterization == CurveKnotStyle::UniformPeriodic ||
                     parameterization == CurveKnotStyle::ChordPeriodic ||
                     parameterization == CurveKnotStyle::ChordSquareRootPeriodic);

    if (periodic && n < 3) return NurbsCurve();

    auto pdist = [](const Point& a, const Point& b) {
        double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    if (periodic) {
        int cv_count = n + 3;
        int kc = cv_count + order - 2;

        CurveKnotStyle base_style = CurveKnotStyle::Chord;
        if (parameterization == CurveKnotStyle::UniformPeriodic) base_style = CurveKnotStyle::Uniform;
        if (parameterization == CurveKnotStyle::ChordSquareRootPeriodic) base_style = CurveKnotStyle::ChordSquareRoot;

        std::vector<double> params(n + 1, 0.0);
        if (base_style == CurveKnotStyle::Uniform) {
            for (int i = 1; i <= n; i++) params[i] = (double)i;
        } else {
            for (int i = 1; i < n; i++) {
                double d = pdist(points[i-1], points[i]);
                if (base_style == CurveKnotStyle::ChordSquareRoot) d = std::sqrt(d);
                params[i] = params[i-1] + d;
            }
            double d_close = pdist(points[n-1], points[0]);
            if (base_style == CurveKnotStyle::ChordSquareRoot) d_close = std::sqrt(d_close);
            params[n] = params[n-1] + d_close;
        }

        double dmin = 1e300, dmax = 0;
        for (int i = 0; i < n; i++) {
            double d = params[i+1] - params[i];
            if (d < dmin) dmin = d;
            if (d > dmax) dmax = d;
        }
        if (dmax <= 0.0 || dmax * 1.490116119385e-8 >= dmin)
            return NurbsCurve();

        std::vector<double> knots(kc);
        for (int i = 0; i <= n; i++) knots[i + 2] = params[i];
        knots[cv_count]     = knots[3] - knots[2] + knots[cv_count - 1];
        knots[1]            = knots[cv_count - 2] - knots[cv_count - 1] + knots[2];
        knots[cv_count + 1] = knots[4] - knots[3] + knots[cv_count];
        knots[0]            = knots[cv_count - 3] - knots[cv_count - 2] + knots[1];

        std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));
        std::vector<double> rhs(n * dim);

        for (int i = 0; i < n; i++) {
            auto basis = knot::eval_basis(order, knots, i, params[i]);
            int c0 = i % n;
            int c1 = (i + 1) % n;
            int c2 = (i + 2) % n;
            A[i][c0] += basis[0];
            A[i][c1] += basis[1];
            A[i][c2] += basis[2];
            for (int d = 0; d < dim; d++)
                rhs[i * dim + d] = points[i][d];
        }

        std::vector<double> cv(n * dim);
        for (int i = 0; i < n; i++)
            for (int d = 0; d < dim; d++)
                cv[i * dim + d] = rhs[i * dim + d];

        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++)
                if (std::fabs(A[row][col]) > std::fabs(A[pivot][col])) pivot = row;
            if (pivot != col) {
                std::swap(A[col], A[pivot]);
                for (int d = 0; d < dim; d++)
                    std::swap(cv[col*dim+d], cv[pivot*dim+d]);
            }
            if (std::fabs(A[col][col]) < 1e-300) return NurbsCurve();
            for (int row = col + 1; row < n; row++) {
                double factor = A[row][col] / A[col][col];
                for (int j = col; j < n; j++) A[row][j] -= factor * A[col][j];
                for (int d = 0; d < dim; d++)
                    cv[row*dim+d] -= factor * cv[col*dim+d];
            }
        }
        for (int i = n - 1; i >= 0; i--) {
            for (int d = 0; d < dim; d++) {
                double sum = cv[i*dim+d];
                for (int j = i + 1; j < n; j++) sum -= A[i][j] * cv[j*dim+d];
                cv[i*dim+d] = sum / A[i][i];
            }
        }

        NurbsCurve curve(dim, false, order, cv_count);
        for (int i = 0; i < kc; i++) curve.set_knot(i, knots[i]);
        for (int i = 0; i < n; i++)
            curve.set_cv(i, Point(cv[i*3], cv[i*3+1], cv[i*3+2]));
        curve.set_cv(n, curve.get_cv(0));
        curve.set_cv(n + 1, curve.get_cv(1));
        curve.set_cv(n + 2, curve.get_cv(2));
        return curve;
    }

    // Open interpolation
    int cv_count = n + 2;

    std::vector<double> pts(n * dim);
    for (int i = 0; i < n; i++) {
        pts[i*3]   = points[i][0];
        pts[i*3+1] = points[i][1];
        pts[i*3+2] = points[i][2];
    }

    auto params = knot::compute_parameters(pts.data(), n, dim, parameterization);
    auto knots = knot::build_interp_knots(params, degree);
    int kc = static_cast<int>(knots.size());

    auto estimate_tangent = [&](int i0, int i1, int i2) -> Vector {
        double d01 = pdist(points[i0], points[i1]);
        double d21 = pdist(points[i2], points[i1]);
        if (d01 + d21 < 1e-300) return Vector(0,0,0);
        double s = d01 / (d01 + d21);
        double t = 1.0 - s;
        double denom = 2.0 * s * t;
        if (denom < 1e-16) {
            double dx = points[i1][0]-points[i0][0];
            double dy = points[i1][1]-points[i0][1];
            double dz = points[i1][2]-points[i0][2];
            double len = std::sqrt(dx*dx+dy*dy+dz*dz);
            return len > 0 ? Vector(dx/len, dy/len, dz/len) : Vector(0,0,0);
        }
        double cvx = (-t*t*points[i0][0] + points[i1][0] - s*s*points[i2][0]) / denom;
        double cvy = (-t*t*points[i0][1] + points[i1][1] - s*s*points[i2][1]) / denom;
        double cvz = (-t*t*points[i0][2] + points[i1][2] - s*s*points[i2][2]) / denom;
        double dx = cvx - points[i0][0];
        double dy = cvy - points[i0][1];
        double dz = cvz - points[i0][2];
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        return len > 0 ? Vector(dx/len, dy/len, dz/len) : Vector(0,0,0);
    };

    Vector tan_start, tan_end;
    if (n >= 3) {
        tan_start = estimate_tangent(0, 1, 2);
        Vector end_raw = estimate_tangent(n-1, n-2, n-3);
        tan_end = Vector(-end_raw[0], -end_raw[1], -end_raw[2]);
    } else {
        double dx = points[1][0]-points[0][0];
        double dy = points[1][1]-points[0][1];
        double dz = points[1][2]-points[0][2];
        double len = std::sqrt(dx*dx+dy*dy+dz*dz);
        if (len > 0) { tan_start = Vector(dx/len, dy/len, dz/len); tan_end = tan_start; }
    }

    double d_start = pdist(points[0], points[1]);
    double d_end = pdist(points[n-1], points[n-2]);

    std::vector<double> cv(cv_count * dim);
    for (int d = 0; d < dim; d++) cv[d] = points[0][d];
    double s0 = d_start / 3.0;
    for (int d = 0; d < dim; d++)
        cv[dim + d] = points[0][d] + s0 * tan_start[d];
    for (int i = 1; i <= n-2; i++)
        for (int d = 0; d < dim; d++)
            cv[(i+1) * dim + d] = points[i][d];
    double s1 = -d_end / 3.0;
    for (int d = 0; d < dim; d++)
        cv[n * dim + d] = points[n-1][d] + s1 * tan_end[d];
    for (int d = 0; d < dim; d++) cv[(n+1) * dim + d] = points[n-1][d];

    int sys_n = n;
    std::vector<double> lower(sys_n, 0.0), diag(sys_n, 0.0), upper(sys_n, 0.0);
    std::vector<double> rhs(sys_n * dim);

    diag[0] = 1.0;
    for (int d = 0; d < dim; d++) rhs[d] = cv[dim + d];

    for (int i = 1; i <= n-2; i++) {
        auto basis = knot::eval_basis(order, knots, i, params[i]);
        lower[i] = basis[0];
        diag[i] = basis[1];
        upper[i] = basis[2];
        for (int d = 0; d < dim; d++)
            rhs[i * dim + d] = points[i][d];
    }

    diag[n-1] = 1.0;
    for (int d = 0; d < dim; d++) rhs[(n-1) * dim + d] = cv[n * dim + d];

    std::vector<double> solution;
    knot::solve_tridiagonal(dim, sys_n, lower, diag, upper, rhs, solution);

    for (int i = 0; i < sys_n; i++)
        for (int d = 0; d < dim; d++)
            cv[(i+1) * dim + d] = solution[i * dim + d];

    NurbsCurve curve(dim, false, order, cv_count);
    for (int i = 0; i < kc; i++) curve.set_knot(i, knots[i]);
    for (int i = 0; i < cv_count; i++)
        curve.set_cv(i, Point(cv[i*3], cv[i*3+1], cv[i*3+2]));

    return curve;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors & Destructor
///////////////////////////////////////////////////////////////////////////////////////////

NurbsCurve::NurbsCurve() {
    initialize();
}

NurbsCurve::NurbsCurve(int dimension, bool is_rational, int order, int cv_count) {
    initialize();
    create(dimension, is_rational, order, cv_count);
}

NurbsCurve::NurbsCurve(const NurbsCurve& other) {
    initialize();
    deep_copy_from(other);
}

NurbsCurve& NurbsCurve::operator=(const NurbsCurve& other) {
    if (this != &other) {
        deep_copy_from(other);
    }
    return *this;
}

bool NurbsCurve::operator==(const NurbsCurve& other) const {
    if (m_dim != other.m_dim || m_is_rat != other.m_is_rat) return false;
    if (m_order != other.m_order || m_cv_count != other.m_cv_count) return false;
    if (m_cv_stride != other.m_cv_stride) return false;
    if (name != other.name) return false;
    if (std::abs(width - other.width) > Tolerance::ZERO_TOLERANCE) return false;
    if (pointcolors != other.pointcolors) return false;
    if (linecolors != other.linecolors) return false;
    if (m_knot.size() != other.m_knot.size()) return false;
    for (size_t i = 0; i < m_knot.size(); i++) {
        if (std::abs(m_knot[i] - other.m_knot[i]) > Tolerance::ZERO_TOLERANCE) return false;
    }
    if (m_cv.size() != other.m_cv.size()) return false;
    for (size_t i = 0; i < m_cv.size(); i++) {
        if (std::abs(m_cv[i] - other.m_cv[i]) > Tolerance::ZERO_TOLERANCE) return false;
    }
    return true;
}

bool NurbsCurve::operator!=(const NurbsCurve& other) const {
    return !(*this == other);
}

NurbsCurve::~NurbsCurve() {
    destroy();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Initialization & Creation
///////////////////////////////////////////////////////////////////////////////////////////

void NurbsCurve::initialize() {
    m_dim = 0;
    m_is_rat = 0;
    m_order = 0;
    m_cv_count = 0;
    m_cv_stride = 0;
    m_cv_capacity = 0;
    m_knot.clear();
    m_cv.clear();
}
bool NurbsCurve::create(int dimension, bool is_rational, int order, int cv_count) {
    if (dimension < 1 || order < 2 || cv_count < order) {
        return false;
    }
    
    destroy();
    
    m_dim = dimension;
    m_is_rat = is_rational ? 1 : 0;
    m_order = order;
    m_cv_count = cv_count;
    m_cv_stride = is_rational ? (dimension + 1) : dimension;
    
    int knot_count = m_order + m_cv_count - 2;
    m_knot.resize(knot_count, 0.0);
    
    m_cv_capacity = m_cv_count * m_cv_stride;
    m_cv.resize(m_cv_capacity, 0.0);
    
    return true;
}

bool NurbsCurve::create_clamped_uniform(int dimension, int order, 
                                        const std::vector<Point>& points,
                                        double knot_delta) {
    int point_count = static_cast<int>(points.size());
    if (!create(dimension, false, order, point_count)) {
        return false;
    }
    
    // Set control points
    for (int i = 0; i < point_count; i++) {
        set_cv(i, points[i]);
    }
    
    // Create clamped uniform knot vector
    // Implementation matches OpenNURBS ON_MakeClampedUniformKnotVector
    int knot_count = m_order + m_cv_count - 2;
    
    // Fill interior knots with uniform spacing
    // Start from index (order-2) up to (cv_count-1)
    double k = 0.0;
    for (int i = m_order - 2; i < m_cv_count; i++, k += knot_delta) {
        m_knot[i] = k;
    }
    
    // Clamp both ends: sets first (order-2) and last (order-2) knots
    // Left clamp: knot[0..order-3] = knot[order-2]
    int i0 = m_order - 2;
    for (int i = 0; i < i0; i++) {
        m_knot[i] = m_knot[i0];
    }
    
    // Right clamp: knot[cv_count..knot_count-1] = knot[cv_count-1]
    i0 = m_cv_count - 1;
    for (int i = i0 + 1; i < knot_count; i++) {
        m_knot[i] = m_knot[i0];
    }
    
    return true;
}

bool NurbsCurve::create_periodic_uniform(int dimension, int order,
                                        const std::vector<Point>& points,
                                        double knot_delta) {
    int point_count = static_cast<int>(points.size());
    if (!create(dimension, false, order, point_count + order - 1)) {
        return false;
    }
    
    // Set control points with wrapping
    for (int i = 0; i < point_count; i++) {
        set_cv(i, points[i]);
    }
    for (int i = 0; i < order - 1; i++) {
        set_cv(point_count + i, points[i]);
    }
    
    // Create periodic uniform knot vector
    int knot_count = m_order + m_cv_count - 2;
    for (int i = 0; i < knot_count; i++) {
        m_knot[i] = (i - m_order + 1) * knot_delta;
    }
    
    return true;
}

void NurbsCurve::destroy() {
    m_knot.clear();
    m_cv.clear();
    initialize();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Validation
///////////////////////////////////////////////////////////////////////////////////////////

bool NurbsCurve::is_valid() const {
    if (m_dim <= 0) return false;
    if (m_order < 2) return false;
    if (m_cv_count < m_order) return false;
    if (m_cv_stride < cv_size()) return false;
    if (m_cv.empty() || m_knot.empty()) return false;
    if (!is_valid_knot_vector()) return false;
    
    // Check CVs for valid values
    for (size_t i = 0; i < m_cv.size(); i++) {
        if (!std::isfinite(m_cv[i])) return false;
    }
    
    return true;
}

bool NurbsCurve::is_valid_knot_vector() const {
    int kc = knot_count();
    if (static_cast<int>(m_knot.size()) != kc) return false;
    
    // Check for non-decreasing knot values
    for (int i = 1; i < kc; i++) {
        if (m_knot[i] < m_knot[i-1]) return false;
    }
    
    // Check for sufficient distinct knots
    if (m_knot[m_order-2] >= m_knot[m_cv_count-1]) return false;
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

int NurbsCurve::cv_size() const {
    return (m_dim > 0) ? (m_is_rat ? (m_dim + 1) : m_dim) : 0;
}

int NurbsCurve::knot_count() const {
    return m_order + m_cv_count - 2;
}

int NurbsCurve::span_count() const {
    int count = 0;
    int kc = knot_count();
    for (int i = m_order - 2; i < m_cv_count - 1; i++) {
        if (i >= 0 && i + 1 < kc && m_knot[i] < m_knot[i + 1]) {
            count++;
        }
    }
    return count;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Control Vertex Access
///////////////////////////////////////////////////////////////////////////////////////////

double* NurbsCurve::cv(int cv_index) {
    if (cv_index < 0 || cv_index >= m_cv_count) return nullptr;
    return &m_cv[cv_index * m_cv_stride];
}

const double* NurbsCurve::cv(int cv_index) const {
    if (cv_index < 0 || cv_index >= m_cv_count) return nullptr;
    return &m_cv[cv_index * m_cv_stride];
}

Point NurbsCurve::get_cv(int cv_index) const {
    const double* cv_ptr = cv(cv_index);
    if (!cv_ptr) return Point(0, 0, 0);
    if (m_is_rat) {
        double w = cv_ptr[m_dim];
        if (std::abs(w) < 1e-14) return Point(0, 0, 0);
        return Point(cv_ptr[0] / w, cv_ptr[1] / w, m_dim > 2 ? cv_ptr[2] / w : 0.0);
    }
    return Point(cv_ptr[0], cv_ptr[1], m_dim > 2 ? cv_ptr[2] : 0.0);
}

bool NurbsCurve::get_cv_4d(int cv_index, double& x, double& y, double& z, double& w) const {
    const double* cv_ptr = cv(cv_index);
    if (!cv_ptr) return false;
    
    x = cv_ptr[0];
    y = m_dim > 1 ? cv_ptr[1] : 0.0;
    z = m_dim > 2 ? cv_ptr[2] : 0.0;
    w = m_is_rat ? cv_ptr[m_dim] : 1.0;
    return true;
}

bool NurbsCurve::set_cv(int cv_index, const Point& point) {
    double* cv_ptr = cv(cv_index);
    if (!cv_ptr) return false;

    cv_ptr[0] = point[0];
    if (m_dim > 1) cv_ptr[1] = point[1];
    if (m_dim > 2) cv_ptr[2] = point[2];
    if (m_is_rat) cv_ptr[m_dim] = 1.0;

    invalidate_rmf_cache();
    return true;
}

bool NurbsCurve::set_cv_4d(int cv_index, double x, double y, double z, double w) {
    if (cv_index < 0 || cv_index >= m_cv_count) return false;

    if (!m_is_rat && w != 1.0) {
        make_rational();
    }

    double* cv_ptr = cv(cv_index);
    if (!cv_ptr) return false;

    if (m_is_rat) {
        cv_ptr[0] = x;
        if (m_dim > 1) cv_ptr[1] = y;
        if (m_dim > 2) cv_ptr[2] = z;
        cv_ptr[m_dim] = w;
    } else {
        cv_ptr[0] = x;
        if (m_dim > 1) cv_ptr[1] = y;
        if (m_dim > 2) cv_ptr[2] = z;
    }
    invalidate_rmf_cache();
    return true;
}

double NurbsCurve::weight(int cv_index) const {
    if (!m_is_rat) return 1.0;
    const double* cv_ptr = cv(cv_index);
    return cv_ptr ? cv_ptr[m_dim] : 1.0;
}

bool NurbsCurve::set_weight(int cv_index, double w) {
    if (!m_is_rat) {
        if (!make_rational()) return false;
    }
    double* cv_ptr = cv(cv_index);
    if (!cv_ptr) return false;
    cv_ptr[m_dim] = w;
    invalidate_rmf_cache();
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Knot Access
///////////////////////////////////////////////////////////////////////////////////////////

double NurbsCurve::knot(int knot_index) const {
    if (knot_index < 0 || knot_index >= static_cast<int>(m_knot.size())) {
        return 0.0;
    }
    return m_knot[knot_index];
}

bool NurbsCurve::set_knot(int knot_index, double knot_value) {
    if (knot_index < 0 || knot_index >= static_cast<int>(m_knot.size())) {
        return false;
    }
    m_knot[knot_index] = knot_value;
    invalidate_rmf_cache();
    return true;
}
// Get knot multiplicity
int NurbsCurve::knot_multiplicity(int knot_index) const {
    if (knot_index < 0 || knot_index >= knot_count()) return 0;
    
    double knot_value = m_knot[knot_index];
    int mult = 1;
    
    // Count knots equal to this value after current index
    for (int i = knot_index + 1; i < knot_count(); i++) {
        if (std::abs(m_knot[i] - knot_value) < Tolerance::ZERO_TOLERANCE) {
            mult++;
        } else {
            break;
        }
    }
    
    // Count knots equal to this value before current index
    for (int i = knot_index - 1; i >= 0; i--) {
        if (std::abs(m_knot[i] - knot_value) < Tolerance::ZERO_TOLERANCE) {
            mult++;
        } else {
            break;
        }
    }
    
    return mult;
}
// Get superfluous knot (matches OpenNURBS ON_SuperfluousKnot)
double NurbsCurve::superfluous_knot(int end) const {
    if (!is_valid()) return 0.0;

    int kc = knot_count();
    if (end == 0) {
        // First superfluous knot: reflect first knot across knot[order-2]
        return 2.0 * m_knot[0] - m_knot[m_order - 2];
    } else {
        // Last superfluous knot: reflect last knot across knot[cv_count-order]
        return 2.0 * m_knot[kc - 1] - m_knot[m_cv_count - m_order];
    }
}
// Check if knot vector is clamped
bool NurbsCurve::is_clamped(int end) const {
    if (!is_valid()) return false;
    
    // Use knot module function
    return knot::is_clamped(m_order, m_cv_count, m_knot, end);
}
// Get Greville abcissa for a control point (aligned with opennurbs ON_GrevilleAbcissa)
double NurbsCurve::greville_abcissa(int cv_index) const {
    if (cv_index < 0 || cv_index >= m_cv_count) return 0.0;

    const double* knot = m_knot.data() + cv_index;
    int order = m_order;

    if (order <= 2 || knot[0] == knot[order - 2]) {
        return knot[0];
    }

    int p = order - 1;
    const double k0 = knot[0];
    const double k = knot[p / 2];
    const double k1 = knot[p - 1];
    const double tol = (k1 - k0) * 1.490116119385e-8; // ON_SQRT_EPSILON
    const double dp = static_cast<double>(p);

    double g = 0.0;
    for (int i = 0; i < p; i++)
        g += knot[i];
    g /= dp;

    // Snap to exact middle knot for uniform knot vectors
    if (std::fabs(2.0 * k - (k0 + k1)) <= tol &&
        std::fabs(g - k) <= (std::fabs(g) * 1.490116119385e-8 + tol))
        g = k;

    return g;
}

// Get all Greville abcissae
bool NurbsCurve::get_greville_abcissae(std::vector<double>& abcissae) const {
    if (!is_valid()) return false;
    
    abcissae.resize(m_cv_count);
    for (int i = 0; i < m_cv_count; i++) {
        abcissae[i] = greville_abcissa(i);
    }
    return true;
}
bool NurbsCurve::insert_knot(double knot_value, int knot_multiplicity) {
    if (!is_valid()) return false;

    int p = degree();
    if (knot_multiplicity < 1 || knot_multiplicity > p) {
        return false;
    }

    auto [d0, d1] = domain();
    if (knot_value < d0 || knot_value > d1) {
        return false;
    }

    // Handle end knots similar to OpenNURBS: clamp or no-op
    if (knot_value == d0) {
        if (knot_multiplicity == p) {
            return clamp_end(0);
        }
        if (knot_multiplicity == 1) {
            return true; // nothing to do
        }
        return false;
    }
    if (knot_value == d1) {
        if (knot_multiplicity == p) {
            return clamp_end(1);
        }
        if (knot_multiplicity == 1) {
            return true; // nothing to do
        }
        return false;
    }

    // Reconstruct full knot vector U from compressed m_knot.
    // Full size = m_cv_count + m_order.
    int n = m_cv_count - 1;
    int full_knot_count = m_cv_count + m_order;

    for (int insert_iter = 0; insert_iter < knot_multiplicity; ++insert_iter) {
        // Build full knot vector
        std::vector<double> U(full_knot_count);
        U[0] = m_knot.front();
        for (int i = 0; i < static_cast<int>(m_knot.size()); ++i) {
            U[i + 1] = m_knot[i];
        }
        U[full_knot_count - 1] = m_knot.back();

        // Count current multiplicity of knot_value in full vector (interior knots)
        int mult = 0;
        const double tol = (std::abs(d0) + std::abs(d1) + std::abs(d1 - d0)) * std::sqrt(std::numeric_limits<double>::epsilon());
        for (int i = 0; i < full_knot_count; ++i) {
            if (std::abs(U[i] - knot_value) <= tol) {
                ++mult;
            }
        }
        if (mult >= p) {
            // Cannot increase multiplicity beyond degree for interior knots
            return false;
        }

        // Find B-spline span index k in full knot vector
        // find_span returns index relative to m_knot[order-2], map to full: k = span + order - 1
        int span = find_span(knot_value);
        int k = span + m_order - 1;

        // Single-knot insertion (Algorithm A5.1 specialized to r = 1)
        int m_full = full_knot_count - 1; // last index in U
        int new_full_knot_count = full_knot_count + 1;
        int new_cv_count = m_cv_count + 1;

        std::vector<double> U_new(new_full_knot_count);
        std::vector<double> cv_new(new_cv_count * m_cv_stride);

        // Copy unaffected knots before insertion position
        for (int i = 0; i <= k; ++i) {
            U_new[i] = U[i];
        }
        // Insert new knot
        U_new[k + 1] = knot_value;
        // Copy remaining knots
        for (int i = k + 1; i <= m_full; ++i) {
            U_new[i + 1] = U[i];
        }

        // Copy unaffected control points
        // Indices follow standard B-spline notation: P[0..n]
        // Q[0..k-p] = P[0..k-p]
        for (int i = 0; i <= k - p; ++i) {
            const double* src = &m_cv[i * m_cv_stride];
            double* dst = &cv_new[i * m_cv_stride];
            std::copy(src, src + m_cv_stride, dst);
        }

        // Q[k+1..n+1] = P[k..n]
        for (int i = k + 1; i <= n + 1; ++i) {
            const double* src = &m_cv[(i - 1) * m_cv_stride];
            double* dst = &cv_new[i * m_cv_stride];
            std::copy(src, src + m_cv_stride, dst);
        }

        // Compute new control points in the affected region
        for (int i = k - p + 1; i <= k; ++i) {
            double alpha = 0.0;
            double denom = U[i + p] - U[i];
            if (denom != 0.0) {
                alpha = (knot_value - U[i]) / denom;
            }

            const double* Pi_1 = &m_cv[(i - 1) * m_cv_stride];
            const double* Pi   = &m_cv[i * m_cv_stride];
            double* Qi         = &cv_new[i * m_cv_stride];

            for (int d = 0; d < m_cv_stride; ++d) {
                Qi[d] = (1.0 - alpha) * Pi_1[d] + alpha * Pi[d];
            }
        }

        // Update internal data from full vectors
        m_cv_count = new_cv_count;
        m_cv = std::move(cv_new);

        // Compress U_new back into m_knot: drop first and last entries
        int new_compressed_knot_count = m_order + m_cv_count - 2;
        std::vector<double> knot_new(new_compressed_knot_count);
        for (int i = 0; i < new_compressed_knot_count; ++i) {
            knot_new[i] = U_new[i + 1];
        }
        m_knot = std::move(knot_new);

        // Prepare for potential subsequent insertion
        full_knot_count = new_full_knot_count;
        n = m_cv_count - 1;
    }

    invalidate_rmf_cache();
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Domain & Parameterization
///////////////////////////////////////////////////////////////////////////////////////////

std::pair<double, double> NurbsCurve::domain() const {
    if (m_knot.empty()) return {0.0, 0.0};
    return {m_knot[m_order-2], m_knot[m_cv_count-1]};
}

double NurbsCurve::domain_start() const {
    if (m_knot.empty()) return 0.0;
    return m_knot[m_order-2];
}

double NurbsCurve::domain_middle() const{
    if (m_knot.empty()) return 0.0;
    return (m_knot[m_order-2]+m_knot[m_cv_count-1])*0.5;
}

double NurbsCurve::domain_end() const {
    if (m_knot.empty()) return 0.0;
    return m_knot[m_cv_count-1];
}

bool NurbsCurve::set_domain(double t0, double t1) {
    if (t0 >= t1 || !is_valid()) return false;

    auto [d0, d1] = domain();
    if (d0 >= d1) return false;

    double scale = (t1 - t0) / (d1 - d0);

    for (auto& k : m_knot) {
        k = t0 + (k - d0) * scale;
    }

    invalidate_rmf_cache();
    return true;
}

std::vector<double> NurbsCurve::get_span_vector() const {
    std::vector<double> spans;
    spans.push_back(m_knot[m_order-2]);
    
    for (int i = m_order - 1; i < m_cv_count; i++) {
        if (m_knot[i] > spans.back()) {
            spans.push_back(m_knot[i]);
        }
    }
    
    return spans;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Geometric Queries
///////////////////////////////////////////////////////////////////////////////////////////

// Get next discontinuity
bool NurbsCurve::get_next_discontinuity(int continuity_type,
                                       double t0, double t1,
                                       double& t_out,
                                       int* hint,
                                       double cos_angle_tolerance,
                                       double curvature_tolerance) const {
    if (!is_valid()) return false;
    if (t0 >= t1) return false;
    
    auto [d0, d1] = domain();
    if (t0 < d0) t0 = d0;
    if (t1 > d1) t1 = d1;
    if (t0 >= t1) return false;
    
    // Check each interior knot
    for (int i = m_order - 1; i < m_cv_count - 1; i++) {
        double t = m_knot[i];
        if (t <= t0 || t >= t1) continue;
        
        // Check if there's a discontinuity at this knot
        int mult = knot_multiplicity(i);
        
        // C0: check if multiplicity >= order
        if (continuity_type == 0 && mult >= m_order) {
            t_out = t;
            if (hint) *hint = i;
            return true;
        }
        
        // C1: check if multiplicity >= order - 1
        if (continuity_type == 1 && mult >= m_order - 1) {
            t_out = t;
            if (hint) *hint = i;
            return true;
        }
        
        // C2: check if multiplicity >= order - 2
        if (continuity_type == 2 && mult >= m_order - 2) {
            t_out = t;
            if (hint) *hint = i;
            return true;
        }
        
        // For G1/G2, would need to evaluate derivatives and check angles
        // Simplified: treat like C1/C2
        if ((continuity_type == 3 || continuity_type == 4) && mult >= m_order - 1) {
            t_out = t;
            if (hint) *hint = i;
            return true;
        }
    }
    
    // Suppress warnings
    (void)cos_angle_tolerance;
    (void)curvature_tolerance;
    
    return false;
}
bool NurbsCurve::is_closed() const {
    if (!is_valid()) return false;
    Point p0 = point_at_start();
    Point p1 = point_at_end();
    return p0.distance(p1) < Tolerance::ZERO_TOLERANCE;
}

bool NurbsCurve::is_periodic() const {
    if (m_order < 2) return false;
    
    // Check if last degree CVs match first degree CVs
    int deg = degree();
    for (int i = 0; i < deg; i++) {
        Point p0 = get_cv(i);
        Point p1 = get_cv(m_cv_count - deg + i);
        if (p0.distance(p1) > Tolerance::ZERO_TOLERANCE) {
            return false;
        }
    }
    
    // Check knot spacing
    int kc = knot_count();
    double delta = m_knot[m_order-1] - m_knot[m_order-2];
    for (int i = m_order; i < kc - m_order + 1; i++) {
        if (std::abs((m_knot[i] - m_knot[i-1]) - delta) > Tolerance::ZERO_TOLERANCE) {
            return false;
        }
    }
    
    return true;
}

bool NurbsCurve::is_linear(double tolerance) const {
    if (!is_valid() || m_cv_count < 2) return false;
    
    Point p0 = get_cv(0);
    Point p1 = get_cv(m_cv_count - 1);
    Vector line_vec(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    double line_length = line_vec.magnitude();
    
    if (line_length < tolerance) return true;
    
    for (int i = 1; i < m_cv_count - 1; i++) {
        Point p = get_cv(i);
        Vector v(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
        Vector cross = line_vec.cross(v);
        double dist = cross.magnitude() / line_length;
        if (dist > tolerance) return false;
    }
    
    return true;
}

bool NurbsCurve::is_planar(Plane* plane, double tolerance) const {
    // Simplified planar check
    if (!is_valid() || m_cv_count < 3) return true;
    
    // Get three non-collinear points
    Point p0 = get_cv(0);
    Point p1 = get_cv(m_cv_count / 2);
    Point p2 = get_cv(m_cv_count - 1);
    
    Vector v1(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Vector v2(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
    Vector normal = v1.cross(v2);
    
    if (normal.magnitude() < tolerance) return true;
    
    // Check all CVs against this plane
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        Vector v(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
        double dist = std::abs(v.dot(normal)) / normal.magnitude();
        if (dist > tolerance) return false;
    }
    
    // If plane pointer provided, fill it with the computed plane
    if (plane) {
        normal.normalize_self();
        Vector x_axis = v1;
        x_axis.normalize_self();
        Vector y_axis = normal.cross(x_axis);
        // Plane constructor takes (origin, x_axis, y_axis)
        // The z_axis/normal is computed internally
        *plane = Plane(p0, x_axis, y_axis);
    }
    
    return true;
}
// Check if arc
bool NurbsCurve::is_arc(Plane* plane, double tolerance) const {
    if (!is_valid()) return false;
    if (m_dim != 2 && m_dim != 3) return false;
    if (m_order < 3) return false;
    if (is_linear(tolerance)) return false;

    Plane test_plane;
    if (!is_planar(&test_plane, tolerance)) return false;

    auto [t0, t1] = domain();
    double tmid = (t0 + t1) * 0.5;

    Point p0 = point_at(t0);
    Point p1 = point_at(tmid);
    Point p2 = point_at(t1);

    // Compute circle center from 3 points via perpendicular bisectors
    Vector d1(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Vector d2(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
    Vector normal = d1.cross(d2);
    if (normal.magnitude() < Tolerance::ZERO_TOLERANCE) return false;
    normal = normal.normalize();

    Point m1((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5);
    Point m2((p1[0] + p2[0]) * 0.5, (p1[1] + p2[1]) * 0.5, (p1[2] + p2[2]) * 0.5);
    Vector perp1 = d1.cross(normal).normalize();
    Vector perp2 = d2.cross(normal).normalize();

    double denom = perp1[0] * perp2[1] - perp1[1] * perp2[0];
    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE)
        denom = perp1[0] * perp2[2] - perp1[2] * perp2[0];
    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE) return false;

    double dx = m2[0] - m1[0];
    double dy = m2[1] - m1[1];
    double s = (dx * perp2[1] - dy * perp2[0]) / denom;
    Point center(m1[0] + s * perp1[0], m1[1] + s * perp1[1], m1[2] + s * perp1[2]);
    double radius = center.distance(p0);
    if (radius < Tolerance::ZERO_TOLERANCE) return false;

    // Sample along curve and check radius deviation
    int samples_per_span = 2 * degree() + 1;
    if (samples_per_span < 4) samples_per_span = 4;
    int num_samples = span_count() * samples_per_span;
    for (int i = 0; i <= num_samples; i++) {
        double t = t0 + (t1 - t0) * i / num_samples;
        Point pt = point_at(t);
        if (std::abs(pt.distance(center) - radius) > tolerance) return false;
    }

    if (plane) *plane = test_plane;
    return true;
}
// Check if in specific plane
bool NurbsCurve::is_in_plane(const Plane& test_plane, double tolerance) const {
    if (!is_valid()) return false;
    
    // Check if all control points lie in the plane
    for (int i = 0; i < m_cv_count; i++) {
        Point pt = get_cv(i);
        Vector v(pt[0] - test_plane.origin()[0],
                pt[1] - test_plane.origin()[1],
                pt[2] - test_plane.origin()[2]);
        
        double dist = std::abs(v.dot(test_plane.z_axis()));
        if (dist > tolerance) {
            return false;
        }
    }
    
    return true;
}
// Check if natural end (zero 2nd derivative) - From OpenNURBS IsNatural
bool NurbsCurve::is_natural(int end) const {
    if (!is_valid()) return false;
    
    const double tol_factor = 1e-8;
    auto [t0, t1] = domain();
    
    for (int pass = ((end == 0 || end == 2) ? 0 : 1); pass < ((end == 1 || end == 2) ? 2 : 1); ++pass) {
        double t = (pass == 0) ? t0 : t1;
        
        // Evaluate 2nd derivative
        auto derivs = evaluate(t, 2);
        if (derivs.size() < 3) return false;
        
        Vector d2(derivs[2][0], derivs[2][1], derivs[2][2]);
        double d2_len = d2.magnitude();
        
        // Get control polygon length for tolerance
        Point cv0 = get_cv((pass == 0) ? 0 : m_cv_count - 1);
        Point cv2 = get_cv((pass == 0) ? std::min(2, m_cv_count - 1) : std::max(0, m_cv_count - 3));
        double tol = cv0.distance(cv2) * tol_factor;
        
        if (d2_len > tol) return false;
    }
    
    return true;
}
// Check if polyline
int NurbsCurve::is_polyline(std::vector<Point>* points, 
                           std::vector<double>* params) const {
    if (!is_valid()) return 0;
    
    // Degree 1 curves are polylines
    if (m_order == 2) {
        if (points) {
            points->clear();
            points->reserve(m_cv_count);
            for (int i = 0; i < m_cv_count; i++) {
                points->push_back(get_cv(i));
            }
        }
        if (params) {
            params->clear();
            params->reserve(m_cv_count);
            for (int i = 0; i < m_cv_count; i++) {
                params->push_back(m_knot[i]);
            }
        }
        return m_cv_count;
    }
    
    // Higher degree: check if each span is linear
    if (m_order > 2 && m_dim >= 2 && m_dim <= 3) {
        int span_cnt = span_count();
        std::vector<Point> span_points;
        std::vector<double> span_params;
        
        bool all_linear = true;
        for (int i = 0; i < span_cnt; i++) {
            if (!span_is_linear(i, Tolerance::ZERO_TOLERANCE, Tolerance::ZERO_TOLERANCE)) {
                all_linear = false;
                break;
            }
        }
        
        if (all_linear && span_cnt > 0) {
            if (points) {
                points->clear();
                points->push_back(get_cv(0));
                for (int i = 0; i < span_cnt; i++) {
                    points->push_back(get_cv(i * (m_order - 1) + (m_order - 1)));
                }
            }
            if (params) {
                params->clear();
                auto spans = get_span_vector();
                *params = spans;
            }
            return span_cnt + 1;
        }
    }
    
    return 0;
}
// Check if entire curve is singular
bool NurbsCurve::is_singular() const {
    if (!is_valid()) return false;

    int span_cnt = this->span_count();
    for (int i = 0; i < span_cnt; i++) {
        if (!span_is_singular(i)) {
            return false;
        }
    }

    return true;
}
// Check if duplicate
bool NurbsCurve::is_duplicate(const NurbsCurve& other,
                              bool ignore_parameterization,
                              double tolerance) const {
    if (!is_valid() || !other.is_valid()) return false;
    if (m_dim != other.m_dim) return false;
    if (m_is_rat != other.m_is_rat) return false;
    if (m_order != other.m_order) return false;
    if (m_cv_count != other.m_cv_count) return false;

    // Check control points
    for (int i = 0; i < m_cv_count; i++) {
        Point p1 = get_cv(i);
        Point p2 = other.get_cv(i);
        if (p1.distance(p2) > tolerance) {
            return false;
        }

        // Check weights for rational curves
        if (m_is_rat) {
            if (std::abs(weight(i) - other.weight(i)) > tolerance) {
                return false;
            }
        }
    }

    // Check knots if not ignoring parameterization
    if (!ignore_parameterization) {
        for (int i = 0; i < knot_count(); i++) {
            if (std::abs(m_knot[i] - other.m_knot[i]) > tolerance) {
                return false;
            }
        }
    }

    return true;
}
// Test continuity
bool NurbsCurve::is_continuous(int continuity_type,
                              double t,
                              int* hint,
                              double point_tolerance,
                              double d1_tolerance,
                              double d2_tolerance,
                              double cos_angle_tolerance,
                              double curvature_tolerance) const {
    if (!is_valid()) return false;
    
    auto [d0, d1] = domain();
    if (t < d0 || t > d1) return false;
    
    // Find knot span
    int span = find_span(t);
    
    // Check if t is at a knot
    bool at_knot = false;
    int knot_idx = -1;
    for (int i = 0; i < knot_count(); i++) {
        if (std::abs(m_knot[i] - t) < Tolerance::ZERO_TOLERANCE) {
            at_knot = true;
            knot_idx = i;
            break;
        }
    }
    
    if (!at_knot) {
        // Interior of span - always continuous
        if (hint) *hint = span;
        return true;
    }
    
    // At knot - check multiplicity
    int mult = knot_multiplicity(knot_idx);
    
    // C0: continuous if mult < order
    if (continuity_type == 0) {
        bool result = mult < m_order;
        if (hint) *hint = span;
        return result;
    }
    
    // C1: continuous if mult < order - 1
    if (continuity_type == 1) {
        bool result = mult < m_order - 1;
        if (hint) *hint = span;
        return result;
    }
    
    // C2: continuous if mult < order - 2
    if (continuity_type == 2) {
        bool result = mult < m_order - 2;
        if (hint) *hint = span;
        return result;
    }
    
    // Suppress warnings
    (void)point_tolerance;
    (void)d1_tolerance;
    (void)d2_tolerance;
    (void)cos_angle_tolerance;
    (void)curvature_tolerance;
    
    // For G1/G2, simplified implementation
    return mult < m_order - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Conversion Methods
///////////////////////////////////////////////////////////////////////////////////////////

// Compute arc length using Gauss-Legendre quadrature
double NurbsCurve::length(double /*tolerance*/) const {
    if (!is_valid()) return 0.0;

    static const double GL_X[10] = {
        -0.9739065285171717, -0.8650633666889845, -0.6794095682990244,
        -0.4333953941292472, -0.1488743389816312,
         0.1488743389816312,  0.4333953941292472,  0.6794095682990244,
         0.8650633666889845,  0.9739065285171717
    };
    static const double GL_W[10] = {
        0.0666713443086881, 0.1494513491505806, 0.2190863625159820,
        0.2692667193099963, 0.2955242247147529,
        0.2955242247147529, 0.2692667193099963, 0.2190863625159820,
        0.1494513491505806, 0.0666713443086881
    };

    double total = 0.0;
    int n_spans = span_count();
    const int SUBDIVISIONS = 4;

    for (int span = 0; span < n_spans; span++) {
        double span_a = m_knot[m_order - 2 + span];
        double span_b = m_knot[m_order - 1 + span];
        if (span_b <= span_a) continue;

        double span_width = (span_b - span_a) / SUBDIVISIONS;
        for (int sub = 0; sub < SUBDIVISIONS; sub++) {
            double a = span_a + sub * span_width;
            double b = a + span_width;
            double mid = (a + b) * 0.5;
            double half = (b - a) * 0.5;
            double s = 0.0;

            for (int i = 0; i < 10; i++) {
                double t = mid + half * GL_X[i];
                auto derivs = evaluate(t, 1);
                s += GL_W[i] * derivs[1].magnitude();
            }
            total += half * s;
        }
    }

    return total;
}
// Convert to polyline with adaptive sampling (curvature-based)
bool NurbsCurve::to_polyline_adaptive(std::vector<Point>& points,
                                     std::vector<double>* params,
                                     double angle_tolerance,
                                     double min_edge_length,
                                     double max_edge_length) const {
    points.clear();
    if (params) params->clear();

    if (!is_valid()) return false;
    if (angle_tolerance <= 0.0) angle_tolerance = 0.1;

    auto [t0, t1] = domain();
    double curve_len = length();

    if (max_edge_length <= 0.0) max_edge_length = curve_len / 10.0;
    if (min_edge_length <= 0.0) min_edge_length = curve_len / 1000.0;
    if (min_edge_length > max_edge_length) min_edge_length = max_edge_length * 0.1;

    // Collect (param, point) pairs, then sort by param
    std::vector<std::pair<double, Point>> samples;
    samples.push_back({t0, point_at(t0)});
    samples.push_back({t1, point_at(t1)});

    // Work queue: segments to potentially subdivide (ta, tb)
    std::vector<std::pair<double, double>> work_queue;
    work_queue.push_back({t0, t1});

    const int max_iterations = 10000;
    int iterations = 0;

    while (!work_queue.empty() && iterations++ < max_iterations) {
        auto [ta, tb] = work_queue.back();
        work_queue.pop_back();

        Point pa = point_at(ta);
        Point pb = point_at(tb);
        double chord_length = pa.distance(pb);

        if (chord_length < min_edge_length) continue;

        double tm = (ta + tb) * 0.5;
        Point pm = point_at(tm);

        // Check deviation: distance from midpoint to chord
        Vector chord = Vector(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
        Vector to_mid = Vector(pm[0] - pa[0], pm[1] - pa[1], pm[2] - pa[2]);
        double chord_len_sq = chord.dot(chord);
        double deviation = 0.0;

        if (chord_len_sq > 1e-20) {
            double proj = to_mid.dot(chord) / chord_len_sq;
            Point projected(pa[0] + proj * chord[0], pa[1] + proj * chord[1], pa[2] + proj * chord[2]);
            deviation = pm.distance(projected);
        }

        // Convert angle tolerance to approximate deviation tolerance
        // For small angles: deviation ≈ chord_length * sin(angle/2) ≈ chord_length * angle/2
        double deviation_tolerance = chord_length * angle_tolerance * 0.5;

        bool need_subdivide = (deviation > deviation_tolerance) || (chord_length > max_edge_length);

        if (need_subdivide) {
            samples.push_back({tm, pm});
            work_queue.push_back({ta, tm});
            work_queue.push_back({tm, tb});
        }
    }

    // Sort by parameter
    std::sort(samples.begin(), samples.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // Extract results
    points.reserve(samples.size());
    if (params) params->reserve(samples.size());

    for (const auto& [t, p] : samples) {
        points.push_back(p);
        if (params) params->push_back(t);
    }

    return points.size() >= 2;
}
// Divide curve into equal arc-length segments using Gauss-Legendre quadrature
bool NurbsCurve::divide_by_count(int count, std::vector<Point>& points,
                                std::vector<double>* params,
                                bool include_endpoints) const {
    points.clear();
    if (params) params->clear();

    if (!is_valid()) return false;
    if (count < 2) return false;

    auto [t0, t1] = domain();
    double dom_len = t1 - t0;
    double h = dom_len * 1e-8;

    // Compute derivative (un-normalized) at parameter t
    auto derivative_at = [&](double t) -> Vector {
        Point p1, p2;
        double dt;
        if (t <= t0 + h) {
            p1 = point_at(t0);
            p2 = point_at(t0 + h);
            dt = h;
        } else if (t >= t1 - h) {
            p1 = point_at(t1 - h);
            p2 = point_at(t1);
            dt = h;
        } else {
            p1 = point_at(t - h);
            p2 = point_at(t + h);
            dt = 2.0 * h;
        }
        return Vector((p2[0] - p1[0]) / dt, (p2[1] - p1[1]) / dt, (p2[2] - p1[2]) / dt);
    };

    // 5-point Gauss-Legendre nodes and weights for [-1, 1]
    static const double GL_NODES[5] = {-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640};
    static const double GL_WEIGHTS[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};

    // Arc length via Gauss-Legendre quadrature
    auto arc_length_gauss = [&](double ta, double tb) -> double {
        double mid = (ta + tb) * 0.5;
        double half = (tb - ta) * 0.5;
        double sum = 0.0;
        for (int i = 0; i < 5; i++) {
            double t = mid + half * GL_NODES[i];
            sum += GL_WEIGHTS[i] * derivative_at(t).magnitude();
        }
        return half * sum;
    };

    // Build arc-length table with high resolution
    int n_samples = std::max(1000, count * 100);
    double dt = (t1 - t0) / n_samples;

    std::vector<double> t_vals(n_samples + 1);
    std::vector<double> s_vals(n_samples + 1);

    t_vals[0] = t0;
    s_vals[0] = 0.0;

    for (int i = 1; i <= n_samples; i++) {
        t_vals[i] = t0 + i * dt;
        s_vals[i] = s_vals[i-1] + arc_length_gauss(t_vals[i-1], t_vals[i]);
    }

    double total_len = s_vals[n_samples];
    int n_segs = include_endpoints ? (count - 1) : (count + 1);
    double seg_len = total_len / n_segs;

    // Find parameter at target arc length with Newton-Raphson refinement
    auto find_t_at_s = [&](double s_target) -> double {
        if (s_target <= 0.0) return t0;
        if (s_target >= total_len) return t1;

        // Binary search for bracket
        int lo = 0, hi = n_samples;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (s_vals[mid] < s_target) lo = mid;
            else hi = mid;
        }

        // Initial guess: linear interpolation
        double frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
        double t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);

        // Newton-Raphson refinement
        double t_lo = t_vals[lo], t_hi = t_vals[hi];
        for (int iter = 0; iter < 20; iter++) {
            double s_cur = s_vals[lo] + arc_length_gauss(t_vals[lo], t);
            double error = s_cur - s_target;

            if (std::abs(error) < 1e-12) break;

            double speed = derivative_at(t).magnitude();
            if (speed < 1e-14) {
                if (error > 0) { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
                continue;
            }

            double t_new = t - error / speed;
            if (t_new <= t_lo || t_new >= t_hi) {
                if (error > 0) { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
            } else {
                t = t_new;
            }
        }
        return t;
    };

    points.reserve(count);
    if (params) params->reserve(count);

    for (int i = 0; i < count; i++) {
        double s_target;
        if (include_endpoints) {
            s_target = seg_len * i;
        } else {
            s_target = seg_len * (i + 1);
        }

        double t = find_t_at_s(s_target);
        points.push_back(point_at(t));
        if (params) params->push_back(t);
    }

    return true;
}
// Divide curve by arc length using Gauss-Legendre quadrature
bool NurbsCurve::divide_by_length(double segment_length, std::vector<Point>& points,
                                 std::vector<double>* params) const {
    points.clear();
    if (params) params->clear();

    if (!is_valid()) return false;
    if (segment_length <= 0.0) return false;

    auto [t0, t1] = domain();
    double dom_len = t1 - t0;
    double h = dom_len * 1e-8;

    // Compute derivative (un-normalized) at parameter t
    auto derivative_at = [&](double t) -> Vector {
        Point p1, p2;
        double dt;
        if (t <= t0 + h) {
            p1 = point_at(t0);
            p2 = point_at(t0 + h);
            dt = h;
        } else if (t >= t1 - h) {
            p1 = point_at(t1 - h);
            p2 = point_at(t1);
            dt = h;
        } else {
            p1 = point_at(t - h);
            p2 = point_at(t + h);
            dt = 2.0 * h;
        }
        return Vector((p2[0] - p1[0]) / dt, (p2[1] - p1[1]) / dt, (p2[2] - p1[2]) / dt);
    };

    // 5-point Gauss-Legendre nodes and weights for [-1, 1]
    static const double GL_NODES[5] = {-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640};
    static const double GL_WEIGHTS[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};

    // Arc length via Gauss-Legendre quadrature
    auto arc_length_gauss = [&](double ta, double tb) -> double {
        double mid = (ta + tb) * 0.5;
        double half = (tb - ta) * 0.5;
        double sum = 0.0;
        for (int i = 0; i < 5; i++) {
            double t = mid + half * GL_NODES[i];
            sum += GL_WEIGHTS[i] * derivative_at(t).magnitude();
        }
        return half * sum;
    };

    // Build arc-length table with high resolution
    int n_samples = std::max(1000, static_cast<int>(length() / segment_length) * 100);
    double dt = (t1 - t0) / n_samples;

    std::vector<double> t_vals(n_samples + 1);
    std::vector<double> s_vals(n_samples + 1);

    t_vals[0] = t0;
    s_vals[0] = 0.0;

    for (int i = 1; i <= n_samples; i++) {
        t_vals[i] = t0 + i * dt;
        s_vals[i] = s_vals[i-1] + arc_length_gauss(t_vals[i-1], t_vals[i]);
    }

    double total_len = s_vals[n_samples];

    // Find parameter at target arc length with Newton-Raphson refinement
    auto find_t_at_s = [&](double s_target) -> double {
        if (s_target <= 0.0) return t0;
        if (s_target >= total_len) return t1;

        // Binary search for bracket
        int lo = 0, hi = n_samples;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (s_vals[mid] < s_target) lo = mid;
            else hi = mid;
        }

        // Initial guess: linear interpolation
        double frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
        double t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);

        // Newton-Raphson refinement
        double t_lo = t_vals[lo], t_hi = t_vals[hi];
        for (int iter = 0; iter < 20; iter++) {
            double s_cur = s_vals[lo] + arc_length_gauss(t_vals[lo], t);
            double error = s_cur - s_target;

            if (std::abs(error) < 1e-12) break;

            double speed = derivative_at(t).magnitude();
            if (speed < 1e-14) {
                if (error > 0) { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
                continue;
            }

            double t_new = t - error / speed;
            if (t_new <= t_lo || t_new >= t_hi) {
                if (error > 0) { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
            } else {
                t = t_new;
            }
        }
        return t;
    };

    // Add points at each segment_length interval
    for (double s = 0.0; s <= total_len + 1e-10; s += segment_length) {
        double t = find_t_at_s(s);
        points.push_back(point_at(t));
        if (params) params->push_back(t);
    }

    return points.size() >= 2;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point NurbsCurve::point_at(double t) const {
    if (!is_valid()) return Point(0, 0, 0);
    
    // find_span returns index relative to shifted knot array
    int span = find_span(t);
    std::vector<double> basis;
    basis_functions(span, t, basis);
    
    double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
    
    // In OpenNURBS, span index directly corresponds to CV starting index
    for (int i = 0; i < m_order; i++) {
        int cv_idx = span + i;
        const double* cv_ptr = cv(cv_idx);
        if (!cv_ptr) continue;
        
        double N = basis[i];
        if (m_is_rat) {
            // CVs stored in homogeneous form: (x*w, y*w, z*w, w)
            x += N * cv_ptr[0];
            y += N * (m_dim > 1 ? cv_ptr[1] : 0.0);
            z += N * (m_dim > 2 ? cv_ptr[2] : 0.0);
            w += N * cv_ptr[m_dim];
        } else {
            x += N * cv_ptr[0];
            y += N * (m_dim > 1 ? cv_ptr[1] : 0.0);
            z += N * (m_dim > 2 ? cv_ptr[2] : 0.0);
            w = 1.0;
        }
    }
    
    if (m_is_rat && w != 0.0) {
        return Point(x / w, y / w, z / w);
    }
    return Point(x, y, z);
}

std::vector<Vector> NurbsCurve::evaluate(double t, int derivative_count) const {
    std::vector<Vector> result;

    if (!is_valid()) {
        result.push_back(Vector(0, 0, 0));
        return result;
    }

    // Clamp derivative order to degree
    int max_derivs = std::min(derivative_count, degree());

    int span = find_span(t);
    std::vector<std::vector<double>> ders;
    basis_functions_derivatives(span, t, max_derivs, ders);

    // Evaluate non-rational or homogeneous coordinates and derivatives
    int p = degree();
    std::vector<std::array<double, 4>> Aders(max_derivs + 1);
    for (int k = 0; k <= max_derivs; ++k) {
        Aders[k] = {0.0, 0.0, 0.0, 0.0};
        for (int j = 0; j <= p; ++j) {
            int cv_idx = span + j;
            const double* cv_ptr = cv(cv_idx);
            if (!cv_ptr) continue;

            double Nx = ders[k][j];
            double cx = cv_ptr[0];
            double cy = (m_dim > 1) ? cv_ptr[1] : 0.0;
            double cz = (m_dim > 2) ? cv_ptr[2] : 0.0;
            double wv = m_is_rat ? cv_ptr[m_dim] : 1.0;

            // CVs stored in homogeneous form: cx=x*w, cy=y*w, cz=z*w
            Aders[k][0] += Nx * cx;
            Aders[k][1] += Nx * cy;
            Aders[k][2] += Nx * cz;
            Aders[k][3] += Nx * wv;
        }
    }

    // Convert from homogeneous derivatives (Aders) to Cartesian derivatives
    std::vector<std::array<double, 3>> Cders(max_derivs + 1);
    if (!m_is_rat) {
        // Non-rational: derivatives are directly Aders (w == 1)
        for (int k = 0; k <= max_derivs; ++k) {
            Cders[k] = {Aders[k][0], Aders[k][1], Aders[k][2]};
        }
    } else {
        // Rational: use standard formula (Piegl & Tiller, Eq. 2.28)
        for (int k = 0; k <= max_derivs; ++k) {
            double w = Aders[0][3];
            double inv_w = (w != 0.0) ? 1.0 / w : 0.0;

            // Initialize derivative to homogeneous derivative
            double Ck_x = Aders[k][0];
            double Ck_y = Aders[k][1];
            double Ck_z = Aders[k][2];

            // Subtract contributions of weight derivatives
            for (int j = 1; j <= k; ++j) {
                double coeff = (double)std::tgamma(k + 1) / (std::tgamma(j + 1) * std::tgamma(k - j + 1));
                double wj = Aders[j][3];
                Ck_x -= coeff * wj * Cders[k - j][0];
                Ck_y -= coeff * wj * Cders[k - j][1];
                Ck_z -= coeff * wj * Cders[k - j][2];
            }

            Ck_x *= inv_w;
            Ck_y *= inv_w;
            Ck_z *= inv_w;
            Cders[k] = {Ck_x, Ck_y, Ck_z};
        }
    }

    // Fill result vectors (0th derivative = point)
    for (int k = 0; k <= max_derivs; ++k) {
        result.emplace_back(Cders[k][0], Cders[k][1], Cders[k][2]);
    }

    // If caller requested more derivatives than degree, pad with zeros
    for (int k = max_derivs + 1; k <= derivative_count; ++k) {
        result.emplace_back(0.0, 0.0, 0.0);
    }

    return result;
}

Vector NurbsCurve::tangent_at(double t) const {
    if (!is_valid()) return Vector(0, 0, 0);

    auto [t0, t1] = domain();
    double h = (t1 - t0) * 1e-7;

    Point p1, p2;
    if (t <= t0 + h) {
        p1 = point_at(t0);
        p2 = point_at(t0 + h);
    } else if (t >= t1 - h) {
        p1 = point_at(t1 - h);
        p2 = point_at(t1);
    } else {
        p1 = point_at(t - h);
        p2 = point_at(t + h);
    }

    Vector tan(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
    double mag = tan.magnitude();
    if (mag > 1e-14) {
        tan.normalize_self();
    }
    return tan;
}
Plane NurbsCurve::plane_at(double t, bool normalized) const {
    if (!is_valid()) return Plane::invalid();

    auto [t0, t1] = domain();
    double param;
    if (normalized) {
        if (t < 0.0 || t > 1.0) return Plane::invalid();
        param = t0 + t * (t1 - t0);
    } else {
        if (t < t0 || t > t1) return Plane::invalid();
        param = t;
    }
    double h = (t1 - t0) * 1e-5;

    Point origin = point_at(param);

    Point pm, p0, pp;
    if (param <= t0 + h) {
        p0 = point_at(t0);
        pp = point_at(t0 + h);
        Point pp2 = point_at(t0 + 2*h);
        pm = p0;
        Vector d1(pp[0] - p0[0], pp[1] - p0[1], pp[2] - p0[2]);
        Vector d2((pp2[0] - 2*pp[0] + p0[0])/(h*h), (pp2[1] - 2*pp[1] + p0[1])/(h*h), (pp2[2] - 2*pp[2] + p0[2])/(h*h));

        double d1_mag = d1.magnitude();
        if (d1_mag < 1e-14) return Plane::invalid();

        Vector T = d1;
        T.normalize_self();

        double d2_dot_T = d2.dot(T);
        Vector N(d2[0] - d2_dot_T * T[0], d2[1] - d2_dot_T * T[1], d2[2] - d2_dot_T * T[2]);
        double n_mag = N.magnitude();
        if (n_mag < 1e-14) {
            Vector worldZ(0, 0, 1);
            N = T.cross(worldZ);
            n_mag = N.magnitude();
            if (n_mag < 1e-14) {
                Vector worldY(0, 1, 0);
                N = T.cross(worldY);
                n_mag = N.magnitude();
            }
        }
        if (n_mag > 1e-14) N.normalize_self();

        Vector B = T.cross(N);
        B.normalize_self();

        return Plane(origin, T, N, B);
    } else if (param >= t1 - h) {
        pm = point_at(t1 - h);
        p0 = point_at(t1);
        Point pm2 = point_at(t1 - 2*h);
        Vector d1(p0[0] - pm[0], p0[1] - pm[1], p0[2] - pm[2]);
        Vector d2((p0[0] - 2*pm[0] + pm2[0])/(h*h), (p0[1] - 2*pm[1] + pm2[1])/(h*h), (p0[2] - 2*pm[2] + pm2[2])/(h*h));

        double d1_mag = d1.magnitude();
        if (d1_mag < 1e-14) return Plane::invalid();

        Vector T = d1;
        T.normalize_self();

        double d2_dot_T = d2.dot(T);
        Vector N(d2[0] - d2_dot_T * T[0], d2[1] - d2_dot_T * T[1], d2[2] - d2_dot_T * T[2]);
        double n_mag = N.magnitude();
        if (n_mag < 1e-14) {
            Vector worldZ(0, 0, 1);
            N = T.cross(worldZ);
            n_mag = N.magnitude();
            if (n_mag < 1e-14) {
                Vector worldY(0, 1, 0);
                N = T.cross(worldY);
                n_mag = N.magnitude();
            }
        }
        if (n_mag > 1e-14) N.normalize_self();

        Vector B = T.cross(N);
        B.normalize_self();

        return Plane(origin, T, N, B);
    }

    pm = point_at(param - h);
    p0 = point_at(param);
    pp = point_at(param + h);

    Vector d1((pp[0] - pm[0])/(2*h), (pp[1] - pm[1])/(2*h), (pp[2] - pm[2])/(2*h));
    Vector d2((pp[0] - 2*p0[0] + pm[0])/(h*h), (pp[1] - 2*p0[1] + pm[1])/(h*h), (pp[2] - 2*p0[2] + pm[2])/(h*h));

    double d1_mag = d1.magnitude();
    if (d1_mag < 1e-14) return Plane::invalid();

    Vector T = d1;
    T.normalize_self();

    double d2_dot_T = d2.dot(T);
    Vector N(d2[0] - d2_dot_T * T[0], d2[1] - d2_dot_T * T[1], d2[2] - d2_dot_T * T[2]);
    double n_mag = N.magnitude();
    if (n_mag < 1e-14) {
        Vector worldZ(0, 0, 1);
        N = T.cross(worldZ);
        n_mag = N.magnitude();
        if (n_mag < 1e-14) {
            Vector worldY(0, 1, 0);
            N = T.cross(worldY);
            n_mag = N.magnitude();
        }
    }
    if (n_mag > 1e-14) N.normalize_self();

    Vector B = T.cross(N);
    B.normalize_self();

    return Plane(origin, T, N, B);
}
Plane NurbsCurve::perpendicular_plane_at(double t, bool normalized) const {
    if (!is_valid()) return Plane::invalid();

    auto [t0, t1] = domain();
    double param;
    if (normalized) {
        if (t < 0.0 || t > 1.0) return Plane::invalid();
        param = t0 + t * (t1 - t0);
    } else {
        if (t < t0 || t > t1) return Plane::invalid();
        param = t;
    }

    // Get initial frame at t0 using Frenet (curvature-based)
    auto derivs0 = evaluate(t0, 2);
    Vector D1_0(derivs0[1][0], derivs0[1][1], derivs0[1][2]);
    Vector D2_0(derivs0[2][0], derivs0[2][1], derivs0[2][2]);

    double D1_0_mag = D1_0.magnitude();
    if (D1_0_mag < 1e-14) return Plane::invalid();

    Vector T0 = D1_0 / D1_0_mag;

    // Initial normal from curvature (Frenet)
    double D2_dot_D1 = D2_0.dot(D1_0);
    double D1_0_mag_sq = D1_0_mag * D1_0_mag;
    Vector N0_unnorm(D2_0[0] - (D2_dot_D1 / D1_0_mag_sq) * D1_0[0],
                     D2_0[1] - (D2_dot_D1 / D1_0_mag_sq) * D1_0[1],
                     D2_0[2] - (D2_dot_D1 / D1_0_mag_sq) * D1_0[2]);

    double N0_mag = N0_unnorm.magnitude();
    if (N0_mag < 1e-14) {
        Vector worldZ(0, 0, 1);
        N0_unnorm = worldZ.cross(T0);
        N0_mag = N0_unnorm.magnitude();
        if (N0_mag < 1e-14) {
            Vector worldY(0, 1, 0);
            N0_unnorm = worldY.cross(T0);
            N0_mag = N0_unnorm.magnitude();
        }
    }
    Vector r0 = N0_unnorm / N0_mag;

    Point origin = point_at(param);

    // If at start, return Frenet frame directly
    if (std::abs(param - t0) < 1e-14) {
        Vector s0 = T0.cross(r0);
        s0.normalize_self();
        return Plane(origin, r0, s0, T0);
    }

    // Propagate frame using Double Reflection (RMF) algorithm
    int num_steps = std::max(10, static_cast<int>((param - t0) / (t1 - t0) * 100));
    double dt = (param - t0) / num_steps;

    Vector ri = r0;
    double ti = t0;
    Point xi = point_at(ti);
    Vector Ti = T0;

    for (int i = 0; i < num_steps && ti < param - 1e-14; i++) {
        double ti_next = std::min(ti + dt, param);
        Point xi_next = point_at(ti_next);
        Vector Ti_next = tangent_at(ti_next);
        Ti_next.normalize_self();

        Vector v1(xi_next[0] - xi[0], xi_next[1] - xi[1], xi_next[2] - xi[2]);
        double c1 = v1.dot(v1);
        if (c1 < 1e-28) {
            ti = ti_next;
            xi = xi_next;
            Ti = Ti_next;
            continue;
        }

        double ri_dot_v1 = ri.dot(v1);
        Vector rL(ri[0] - 2.0 * ri_dot_v1 / c1 * v1[0],
                  ri[1] - 2.0 * ri_dot_v1 / c1 * v1[1],
                  ri[2] - 2.0 * ri_dot_v1 / c1 * v1[2]);

        double Ti_dot_v1 = Ti.dot(v1);
        Vector TL(Ti[0] - 2.0 * Ti_dot_v1 / c1 * v1[0],
                  Ti[1] - 2.0 * Ti_dot_v1 / c1 * v1[1],
                  Ti[2] - 2.0 * Ti_dot_v1 / c1 * v1[2]);

        Vector v2(Ti_next[0] - TL[0], Ti_next[1] - TL[1], Ti_next[2] - TL[2]);
        double c2 = v2.dot(v2);
        if (c2 < 1e-28) {
            ri = rL;
        } else {
            double rL_dot_v2 = rL.dot(v2);
            ri = Vector(rL[0] - 2.0 * rL_dot_v2 / c2 * v2[0],
                        rL[1] - 2.0 * rL_dot_v2 / c2 * v2[1],
                        rL[2] - 2.0 * rL_dot_v2 / c2 * v2[2]);
        }

        double ri_mag = ri.magnitude();
        if (ri_mag > 1e-14) ri.normalize_self();

        ti = ti_next;
        xi = xi_next;
        Ti = Ti_next;
    }

    Vector T = tangent_at(param);
    T.normalize_self();

    // Ensure ri is perpendicular to T
    double ri_dot_T = ri.dot(T);
    ri = Vector(ri[0] - ri_dot_T * T[0], ri[1] - ri_dot_T * T[1], ri[2] - ri_dot_T * T[2]);
    double ri_mag = ri.magnitude();
    if (ri_mag > 1e-14) ri.normalize_self();

    Vector s = T.cross(ri);
    s.normalize_self();

    return Plane(origin, ri, s, T);
}

std::vector<Plane>
NurbsCurve::get_perpendicular_planes(int count) const {
    std::vector<Plane> frames;
    std::vector<Point> pts;
    std::vector<double> params;
    divide_by_count(count + 1, pts, &params, true);
    for (double t : params)
        frames.push_back(perpendicular_plane_at(t, false));
    return frames;
}
Point NurbsCurve::point_at_start() const {
    auto [t0, t1] = domain();
    return point_at(t0);
}

Point NurbsCurve::point_at_middle() const {
    auto [t0, t1] = domain();
    return point_at((t0 + t1) / 2.0);
}

Point NurbsCurve::point_at_end() const {
    auto [t0, t1] = domain();
    return point_at(t1);
}
// Force curve to start at point
bool NurbsCurve::set_start_point(const Point& start_point) {
    if (!is_valid()) return false;
    
    clamp_end(2);
    
    double w = 1.0;
    if (m_is_rat) {
        w = weight(0);
    }
    
    Point scaled_point = start_point;
    if (m_is_rat && w != 1.0) {
        // Scale by weight for rational curves
        set_cv_4d(0, start_point[0] * w, start_point[1] * w, start_point[2] * w, w);
    } else {
        set_cv(0, start_point);
        if (m_is_rat) {
            set_weight(0, w);
        }
    }
    
    return true;
}

// Force curve to end at point
bool NurbsCurve::set_end_point(const Point& end_point) {
    if (!is_valid()) return false;
    
    clamp_end(2);
    
    double w = 1.0;
    if (m_is_rat) {
        w = weight(m_cv_count - 1);
    }
    
    if (m_is_rat && w != 1.0) {
        // Scale by weight for rational curves
        set_cv_4d(m_cv_count - 1, end_point[0] * w, end_point[1] * w, end_point[2] * w, w);
    } else {
        set_cv(m_cv_count - 1, end_point);
        if (m_is_rat) {
            set_weight(m_cv_count - 1, w);
        }
    }
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Modification Operations
///////////////////////////////////////////////////////////////////////////////////////////

bool NurbsCurve::reverse() {
    if (!is_valid()) return false;

    // Reverse knots
    auto [d0, d1] = domain();
    for (auto& k : m_knot) {
        k = d0 + d1 - k;
    }
    std::reverse(m_knot.begin(), m_knot.end());

    // Reverse CVs
    for (int i = 0; i < m_cv_count / 2; i++) {
        int j = m_cv_count - 1 - i;
        Point pi = get_cv(i);
        Point pj = get_cv(j);
        set_cv(i, pj);
        set_cv(j, pi);
    }

    invalidate_rmf_cache();
    return true;
}
// Swap coordinates
bool NurbsCurve::swap_coordinates(int axis_i, int axis_j) {
    if (!is_valid()) return false;
    if (axis_i < 0 || axis_i >= m_dim) return false;
    if (axis_j < 0 || axis_j >= m_dim) return false;
    if (axis_i == axis_j) return true;
    
    // Swap coordinates in all control vertices
    for (int cv_idx = 0; cv_idx < m_cv_count; cv_idx++) {
        double* cv_ptr = cv(cv_idx);
        std::swap(cv_ptr[axis_i], cv_ptr[axis_j]);
    }
    
    return true;
}
// Trim curve to interval
bool NurbsCurve::trim(double t0, double t1) {
    if (!is_valid() || t0 >= t1) return false;

    auto [d0, d1] = domain();
    if (t0 < d0 - Tolerance::ZERO_TOLERANCE || t1 > d1 + Tolerance::ZERO_TOLERANCE) return false;
    t0 = std::max(t0, d0);
    t1 = std::min(t1, d1);
    if (std::abs(t0 - d0) < Tolerance::ZERO_TOLERANCE && std::abs(t1 - d1) < Tolerance::ZERO_TOLERANCE)
        return true;

    int p = degree();
    bool trim_start = (t0 > d0 + Tolerance::ZERO_TOLERANCE);
    bool trim_end = (t1 < d1 - Tolerance::ZERO_TOLERANCE);

    // Insert knots at trim boundaries to multiplicity = degree
    if (trim_start) {
        if (!insert_knot(t0, p)) return false;
    }
    if (trim_end) {
        if (!insert_knot(t1, p)) return false;
    }

    // Build full knot vector
    int full_knot_count = m_cv_count + m_order;
    std::vector<double> U(full_knot_count);
    U[0] = m_knot.front();
    for (int i = 0; i < static_cast<int>(m_knot.size()); ++i) {
        U[i + 1] = m_knot[i];
    }
    U[full_knot_count - 1] = m_knot.back();

    // Find span indices for t0 and t1
    // After knot insertion, t0 and t1 have multiplicity p at interior points
    const double tol = Tolerance::ZERO_TOLERANCE;

    // Find the LAST knot equal to t0 (this is where the curve "enters" the trimmed domain)
    int start_span = -1;
    for (int i = full_knot_count - 1; i >= 0; --i) {
        if (std::abs(U[i] - t0) < tol) {
            start_span = i;
            break;
        }
    }

    // Find the FIRST knot equal to t1 (this is where the curve "exits" the trimmed domain)
    int end_span = -1;
    for (int i = 0; i < full_knot_count; ++i) {
        if (std::abs(U[i] - t1) < tol) {
            end_span = i;
            break;
        }
    }

    if (start_span < 0 || end_span < 0 || start_span >= end_span) return false;

    // For trimmed curve:
    // - First CV index = start_span - p (but at least 0)
    // - Number of CVs = end_span - start_span + p
    int first_cv = start_span - p;
    if (first_cv < 0) first_cv = 0;

    // CV count: from CV[first_cv] to the CV just before end_span
    int last_cv = end_span - 1;
    if (last_cv >= m_cv_count) last_cv = m_cv_count - 1;

    // Ensure we have enough CVs for a valid curve of this order
    int new_cv_count = last_cv - first_cv + 1;
    if (new_cv_count < m_order) {
        // Try to expand range to get minimum CVs
        new_cv_count = m_order;
        if (first_cv + new_cv_count - 1 < m_cv_count) {
            last_cv = first_cv + new_cv_count - 1;
        } else {
            return false;
        }
    }

    // Extract knot vector - we need new_cv_count + m_order - 2 knots (compressed form)
    int new_knot_count = new_cv_count + m_order - 2;

    // Build the new knot vector ensuring proper clamping at both ends
    std::vector<double> new_knot(new_knot_count);

    // For start: first p-1 knots should all be t0
    // For end: last p-1 knots should all be t1
    // In between: copy from the original
    for (int i = 0; i < p - 1; ++i) {
        new_knot[i] = t0;
    }

    // Middle knots: copy from position start_span to end_span - p
    int mid_count = new_knot_count - 2 * (p - 1);
    if (mid_count > 0) {
        int src_start = start_span;
        for (int i = 0; i < mid_count; ++i) {
            int src_idx = src_start + i;
            if (src_idx < full_knot_count) {
                new_knot[p - 1 + i] = U[src_idx];
            } else {
                new_knot[p - 1 + i] = t1;
            }
        }
    }

    for (int i = 0; i < p - 1; ++i) {
        new_knot[new_knot_count - p + 1 + i] = t1;
    }

    // Extract control points
    std::vector<double> new_cv(new_cv_count * m_cv_stride);
    for (int i = 0; i < new_cv_count; ++i) {
        const double* src = &m_cv[(first_cv + i) * m_cv_stride];
        double* dst = &new_cv[i * m_cv_stride];
        std::copy(src, src + m_cv_stride, dst);
    }

    m_cv_count = new_cv_count;
    m_cv_capacity = new_cv_count;
    m_cv = std::move(new_cv);
    m_knot = std::move(new_knot);

    invalidate_rmf_cache();
    return true;
}
// Split curve at parameter t
bool NurbsCurve::split(double t, NurbsCurve& left, NurbsCurve& right) const {
    if (!is_valid()) return false;
    
    auto [t0, t1] = domain();
    if (t <= t0 || t >= t1) return false;
    
    // Simplified split - copy curve and trim each half
    left = *this;
    right = *this;
    
    left.trim(t0, t);
    right.trim(t, t1);
    
    return true;
}
// Extend curve domain using natural NURBS extrapolation (De Boor algorithm)
bool NurbsCurve::extend(double t0, double t1) {
    if (!is_valid() || is_closed()) return false;

    auto [d0, d1] = domain();
    int cvdim = cv_size();
    bool changed = false;

    // Extend start (t0 < current domain start)
    if (t0 < d0) {
        clamp_end(0);
        evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[0], &m_knot[0], 1, t0);
        for (int i = 0; i < m_order - 1; i++) {
            m_knot[i] = t0;
        }
        changed = true;
    }

    // Extend end (t1 > current domain end)
    if (t1 > d1) {
        clamp_end(1);
        int i0 = m_cv_count - m_order;
        evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[i0 * m_cv_stride], &m_knot[i0], -1, t1);
        int kc = knot_count();
        for (int i = m_cv_count - 1; i < kc; i++) {
            m_knot[i] = t1;
        }
        changed = true;
    }

    return changed;
}
bool NurbsCurve::make_rational() {
    if (m_is_rat) return true;
    
    int new_stride = m_dim + 1;
    std::vector<double> new_cv(m_cv_count * new_stride);
    
    for (int i = 0; i < m_cv_count; i++) {
        const double* old_cv = cv(i);
        double* new_cv_ptr = &new_cv[i * new_stride];
        for (int j = 0; j < m_dim; j++) {
            new_cv_ptr[j] = old_cv[j];
        }
        new_cv_ptr[m_dim] = 1.0;
    }
    
    m_cv = new_cv;
    m_is_rat = 1;
    m_cv_stride = new_stride;
    m_cv_capacity = m_cv_count * m_cv_stride;
    
    return true;
}

bool NurbsCurve::make_non_rational(bool force) {
    if (!m_is_rat) return true;

    if (force) {
        for (int i = 0; i < m_cv_count; i++) {
            double* cv_ptr = cv(i);
            if (cv_ptr) cv_ptr[m_dim] = 1.0;
        }
    } else {
        double w0 = weight(0);
        for (int i = 1; i < m_cv_count; i++) {
            if (std::abs(weight(i) - w0) > Tolerance::ZERO_TOLERANCE) {
                return false;
            }
        }
    }
    
    int new_stride = m_dim;
    std::vector<double> new_cv(m_cv_count * new_stride);
    
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        double* new_cv_ptr = &new_cv[i * new_stride];
        new_cv_ptr[0] = p[0];
        if (m_dim > 1) new_cv_ptr[1] = p[1];
        if (m_dim > 2) new_cv_ptr[2] = p[2];
    }
    
    m_cv = new_cv;
    m_is_rat = 0;
    m_cv_stride = new_stride;
    m_cv_capacity = m_cv_count * m_cv_stride;
    
    return true;
}
// Clamp end knots with proper CV adjustment using De Boor algorithm
bool NurbsCurve::clamp_end(int end) {
    if (!is_valid()) return false;
    if (end < 0 || end > 2) return false;

    int cvdim = cv_size();
    bool rc = true;

    // Clamp start
    if (end == 0 || end == 2) {
        double t = m_knot[m_order - 2];
        if (evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[0], &m_knot[0], 1, t)) {
            for (int i = 0; i < m_order - 2; i++) {
                m_knot[i] = t;
            }
        } else {
            rc = false;
        }
    }

    // Clamp end
    if (end == 1 || end == 2) {
        int i0 = m_cv_count - m_order;
        double t = m_knot[m_cv_count - 1];
        if (evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[i0 * m_cv_stride], &m_knot[i0], -1, t)) {
            int kc = knot_count();
            for (int i = m_cv_count; i < kc; i++) {
                m_knot[i] = t;
            }
        } else {
            rc = false;
        }
    }

    return rc;
}

// Evaluate NURBS blossom at order-1 parameter values using de Boor-like recurrence
static bool evaluate_nurbs_blossom(int cvdim, int order, int cv_stride,
    const double* CV, const double* knot, const double* t, double* P) {
    if (!CV || !t || !knot) return false;
    if (cv_stride < cvdim) return false;
    int degree = order - 1;
    for (int i = 1; i < 2 * degree; i++) {
        if (knot[i] - knot[i - 1] < 0.0) return false;
    }
    if (knot[degree] - knot[degree - 1] < Tolerance::ZERO_TOLERANCE) return false;
    std::vector<double> space(order);
    for (int i = 0; i < cvdim; i++) {
        const double* cv = CV + i;
        for (int j = 0; j < order; j++) {
            space[j] = *cv;
            cv += cv_stride;
        }
        for (int j = 1; j < order; j++) {
            for (int k = j; k < order; k++) {
                double denom = knot[degree + k - j] - knot[k - 1];
                space[k - j] = (knot[degree + k - j] - t[j - 1]) / denom * space[k - j] +
                    (t[j - 1] - knot[k - 1]) / denom * space[k - j + 1];
            }
        }
        P[i] = space[0];
    }
    return true;
}

// Compute one new CV for raised degree curve using blossom averaging
static bool get_raised_degree_cv(int old_order, int cvdim, int old_cv_stride,
    const double* oldCV, const double* oldkn, const double* newkn,
    int cv_id, double* newCV) {
    if (!oldCV || !oldkn || !newkn || !newCV || cv_id < 0 || cv_id > old_order)
        return false;
    int old_degree = old_order - 1;
    int new_degree = old_degree + 1;
    std::vector<double> t(old_degree);
    std::vector<double> P(cvdim);
    memset(newCV, 0, cvdim * sizeof(double));
    const double* kn = newkn + cv_id;
    for (int i = 0; i < new_degree; i++) {
        int k = 0;
        for (int j = 0; j < new_degree; j++) {
            if (j != i) { t[k] = kn[j]; k++; }
        }
        if (!evaluate_nurbs_blossom(cvdim, old_order, old_cv_stride, oldCV, oldkn, t.data(), P.data()))
            return false;
        for (k = 0; k < cvdim; k++) newCV[k] += P[k];
    }
    double denom = (double)new_degree;
    for (int i = 0; i < cvdim; i++) newCV[i] /= denom;
    return true;
}

// Advance span index past degenerate spans
static int next_span_index(int order, int cv_count, const double* knot, int span_index) {
    if (span_index < 0 || span_index > cv_count - order || !knot)
        return -1;
    if (span_index < cv_count - order) {
        do { span_index++; }
        while (span_index < cv_count - order &&
               knot[span_index + order - 2] == knot[span_index + order - 1]);
    }
    return span_index;
}

// Increment NURBS degree by 1 (helper for increase_degree)
static bool increment_nurbs_degree(NurbsCurve& N) {
    NurbsCurve M = N;
    int sc = M.span_count();
    int new_kcount = M.knot_count() + sc + 1;
    int new_order = M.order() + 1;
    int new_cv_count = new_kcount - new_order + 2;
    int cvdim = M.cv_size();

    N.m_order = new_order;
    N.m_cv_count = new_cv_count;
    N.m_knot.resize(new_order + new_cv_count - 2);
    N.m_cv.resize(new_cv_count * N.m_cv_stride, 0.0);

    // Build new knot vector: each distinct knot gets mult+1 copies
    int ki = 0, ko = 0;
    int mkc = M.knot_count();
    while (ki < mkc) {
        double kn = M.m_knot[ki];
        int mult = 1;
        while (ki + mult < mkc && std::abs(M.m_knot[ki + mult] - kn) < Tolerance::ZERO_TOLERANCE)
            mult++;
        for (int j = 0; j <= mult; j++) {
            N.m_knot[ko++] = kn;
        }
        ki += mult;
    }

    // Zero out N's CVs
    std::fill(N.m_cv.begin(), N.m_cv.end(), 0.0);

    // Compute new CVs per span using blossom
    int siN = 0, siM = 0;
    for (int i = 0; i < sc; i++) {
        const double* knotN = &N.m_knot[siN];
        const double* knotM = &M.m_knot[siM];
        const double* cvM = &M.m_cv[siM * M.m_cv_stride];
        int span_mult = N.knot_multiplicity(siN + N.degree() - 1);
        int skip = N.order() - span_mult;
        for (int j = skip; j < N.order(); j++) {
            double* cvN = &N.m_cv[(siN + j) * N.m_cv_stride];
            get_raised_degree_cv(M.order(), cvdim, M.m_cv_stride,
                cvM, knotM, knotN, j, cvN);
        }
        siN = next_span_index(N.order(), N.cv_count(), N.m_knot.data(), siN);
        siM = next_span_index(M.order(), M.cv_count(), M.m_knot.data(), siM);
    }

    // Copy first and last CVs from original
    for (int i = 0; i < cvdim; i++) {
        N.m_cv[i] = M.m_cv[i];
        N.m_cv[(N.cv_count() - 1) * N.m_cv_stride + i] =
            M.m_cv[(M.cv_count() - 1) * M.m_cv_stride + i];
    }
    return true;
}

// Increase degree
bool NurbsCurve::increase_degree(int desired_degree) {
    if (!is_valid()) return false;
    if (desired_degree < 1 || desired_degree < degree()) return false;
    if (desired_degree == degree()) return true;
    if (!clamp_end(2)) return false;

    int del = desired_degree - degree();
    int sc = span_count();
    int new_order = m_order + del;
    int new_kcount = knot_count() + (sc + 1) * del;
    int new_cv_count = new_kcount - new_order + 2;

    m_knot.reserve(new_order + new_cv_count - 2);
    m_cv.reserve(new_cv_count * m_cv_stride);

    for (int i = 0; i < del; i++) {
        if (!increment_nurbs_degree(*this)) return false;
    }
    return true;
}
// Change closed curve seam
bool NurbsCurve::change_closed_curve_seam(double t) {
    if (!is_valid()) return false;
    if (!is_closed()) return false;

    auto [t0, t1] = domain();
    double dom_len = t1 - t0;

    double s = (t - t0) / dom_len;
    if (s < 0.0 || s > 1.0) {
        s = fmod(s, 1.0);
        if (s < 0.0) s += 1.0;
        t = t0 + s * dom_len;
    }

    if (std::abs(t - t0) < Tolerance::ZERO_TOLERANCE ||
        std::abs(t - t1) < Tolerance::ZERO_TOLERANCE)
        return true;

    if (t <= t0 || t >= t1) return true;

    int p = degree();
    int order = m_order;

    if (is_periodic()) {
        int sc = span_count();
        int kc = knot_count();
        if (sc >= kc - 2 * p + 1) {
            int knot_index = -1;
            for (int i = 0; i < kc; i++) {
                if (m_knot[i] > t) { knot_index = i; break; }
            }
            if (knot_index >= p && knot_index <= kc - p) {
                double k0 = m_knot[knot_index - 1];
                double k1 = m_knot[knot_index];
                double d0 = t - k0;
                double d1 = k1 - t;
                bool need_insert = true;
                if (d0 <= d1) {
                    if (d0 < Tolerance::ZERO_TOLERANCE) {
                        knot_index--;
                        need_insert = false;
                    }
                } else {
                    if (d1 < Tolerance::ZERO_TOLERANCE)
                        need_insert = false;
                }
                if (need_insert) {
                    if (!insert_knot(t, 1)) return false;
                    kc = knot_count();
                    sc = span_count();
                    knot_index = -1;
                    for (int i = 0; i < kc; i++) {
                        if (m_knot[i] > t + Tolerance::ZERO_TOLERANCE) {
                            knot_index = i; break;
                        }
                    }
                    if (knot_index < 0) return false;
                }
                if (knot_index >= p && knot_index < kc - p) {
                    int cvc = m_cv_count;
                    int distinct_cvc = cvc - p;
                    int cvdim = cv_size();
                    std::vector<double> old_knots = m_knot;
                    std::vector<double> old_cv = m_cv;

                    int curr = p - 1;
                    for (int i = knot_index; i < sc + p - 1; i++) {
                        m_knot[curr] = old_knots[i];
                        curr++;
                    }
                    for (int i = 0; i <= knot_index - p + 1; i++) {
                        m_knot[curr] = old_knots[p - 1 + i] + dom_len;
                        curr++;
                    }
                    for (int i = 0; i < p - 1; i++) {
                        m_knot[curr + i] = m_knot[curr + i - 1] + m_knot[p + i] - m_knot[p + i - 1];
                        m_knot[p - 2 - i] = m_knot[p - i - 1] - m_knot[curr - 1 - i] + m_knot[curr - 2 - i];
                    }

                    int cv_id = knot_index - p + 1;
                    for (int i = 0; i < cvc; i++) {
                        int src = cv_id % distinct_cvc;
                        if (src < 0) src += distinct_cvc;
                        for (int j = 0; j < cvdim; j++)
                            m_cv[i * m_cv_stride + j] = old_cv[src * m_cv_stride + j];
                        cv_id++;
                    }

                    set_domain(t, t + dom_len);
                    invalidate_rmf_cache();
                    return true;
                }
            }
        }
    }

    // Non-periodic closed curve: split at t, join right + left
    NurbsCurve left_crv, right_crv;
    if (!split(t, left_crv, right_crv)) return false;

    double shift = t1 - t0;
    int cvdim = cv_size();
    int new_cv_count = right_crv.m_cv_count + left_crv.m_cv_count - 1;
    int new_kc = order + new_cv_count - 2;

    std::vector<double> new_cv(new_cv_count * m_cv_stride);
    std::vector<double> new_knots(new_kc);

    for (int i = 0; i < right_crv.m_cv_count; i++)
        for (int j = 0; j < cvdim; j++)
            new_cv[i * m_cv_stride + j] = right_crv.m_cv[i * right_crv.m_cv_stride + j];

    for (int i = 1; i < left_crv.m_cv_count; i++) {
        int dst = right_crv.m_cv_count + i - 1;
        for (int j = 0; j < cvdim; j++)
            new_cv[dst * m_cv_stride + j] = left_crv.m_cv[i * left_crv.m_cv_stride + j];
    }

    int rkc = right_crv.knot_count();
    for (int i = 0; i < rkc; i++)
        new_knots[i] = right_crv.m_knot[i];

    int lkc = left_crv.knot_count();
    for (int i = order - 1; i < lkc; i++)
        new_knots[rkc + i - (order - 1)] = left_crv.m_knot[i] + shift;

    m_cv_count = new_cv_count;
    m_cv_capacity = new_cv_count * m_cv_stride;
    m_cv = std::move(new_cv);
    m_knot = std::move(new_knots);

    set_domain(t, t + dom_len);
    invalidate_rmf_cache();
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void NurbsCurve::transform() {
    transform(xform);
}

bool NurbsCurve::transform(const Xform& xf) {
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        double x = xf.m[0] * p[0] + xf.m[4] * p[1] + xf.m[8] * p[2] + xf.m[12];
        double y = xf.m[1] * p[0] + xf.m[5] * p[1] + xf.m[9] * p[2] + xf.m[13];
        double z = xf.m[2] * p[0] + xf.m[6] * p[1] + xf.m[10] * p[2] + xf.m[14];
        if (m_is_rat) {
            double w = weight(i);
            set_cv_4d(i, x * w, y * w, z * w, w);
        } else {
            set_cv(i, Point(x, y, z));
        }
    }
    return true;
}

NurbsCurve NurbsCurve::transformed() const {
    NurbsCurve result = *this;
    result.transform();
    return result;
}

NurbsCurve NurbsCurve::transformed(const Xform& xf) const {
    NurbsCurve result = *this;
    result.transform(xf);
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json NurbsCurve::jsondump() const {
    nlohmann::ordered_json j;

    // Build control_points array
    nlohmann::json cps = nlohmann::json::array();
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        cps.push_back({p[0], p[1], p[2]});
    }

    // Fields in alphabetical order (per CLAUDE.md)
    j["control_points"] = cps;
    j["cv_count"] = m_cv_count;
    j["cv_stride"] = m_cv_stride;
    j["dimension"] = m_dim;
    j["guid"] = guid;
    j["is_rational"] = m_is_rat != 0;
    j["knots"] = m_knot;

    nlohmann::ordered_json linecolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : linecolors) {
        linecolors_arr.push_back(c.r); linecolors_arr.push_back(c.g);
        linecolors_arr.push_back(c.b); linecolors_arr.push_back(c.a);
    }
    j["linecolors"] = linecolors_arr;

    j["name"] = name;
    j["order"] = m_order;

    nlohmann::ordered_json pointcolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : pointcolors) {
        pointcolors_arr.push_back(c.r); pointcolors_arr.push_back(c.g);
        pointcolors_arr.push_back(c.b); pointcolors_arr.push_back(c.a);
    }
    j["pointcolors"] = pointcolors_arr;

    j["type"] = "NurbsCurve";
    j["width"] = width;
    j["xform"] = xform.jsondump();

    return j;
}

NurbsCurve NurbsCurve::jsonload(const nlohmann::json& data) {
    NurbsCurve curve;

    if (data.contains("dimension") && data.contains("order") && data.contains("cv_count")) {
        int dim = data["dimension"];
        bool is_rat = data.value("is_rational", false);
        int order = data["order"];
        int cv_count = data["cv_count"];

        curve.create(dim, is_rat, order, cv_count);

        if (data.contains("knots")) {
            curve.m_knot = data["knots"].get<std::vector<double>>();
        }

        if (data.contains("control_points")) {
            auto cps = data["control_points"];
            for (int i = 0; i < std::min(cv_count, static_cast<int>(cps.size())); i++) {
                double x = cps[i][0];
                double y = cps[i][1];
                double z = (cps[i].size() > 2) ? cps[i][2].get<double>() : 0.0;
                Point p(x, y, z);
                curve.set_cv(i, p);
            }
        }

        curve.guid = data.value("guid", ::guid());
        curve.name = data.value("name", "my_nurbscurve");
        curve.width = data.value("width", 1.0);
        if (data.contains("pointcolors") && data["pointcolors"].is_array()) {
            const auto& arr = data["pointcolors"];
            for (size_t i = 0; i + 3 < arr.size(); i += 4) {
                curve.pointcolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                    arr[i+2].get<int>(), arr[i+3].get<int>()));
            }
        }
        if (data.contains("linecolors") && data["linecolors"].is_array()) {
            const auto& arr = data["linecolors"];
            for (size_t i = 0; i + 3 < arr.size(); i += 4) {
                curve.linecolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                    arr[i+2].get<int>(), arr[i+3].get<int>()));
            }
        }
        if (data.contains("xform")) {
            curve.xform = Xform::jsonload(data["xform"]);
        }
    }

    return curve;
}

std::string NurbsCurve::json_dumps() const {
    return jsondump().dump();
}

NurbsCurve NurbsCurve::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void NurbsCurve::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

NurbsCurve NurbsCurve::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

void NurbsCurve::pb_dump(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    file << pb_dumps();
}

NurbsCurve NurbsCurve::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    return pb_loads(data);
}

std::string NurbsCurve::pb_dumps() const {
    session_proto::NurbsCurve proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_dimension(m_dim);
    proto.set_is_rational(m_is_rat != 0);
    proto.set_order(m_order);
    proto.set_cv_count(m_cv_count);
    proto.set_cv_stride(m_cv_stride);
    for (double k : m_knot) {
        proto.add_knots(k);
    }
    for (double c : m_cv) {
        proto.add_cvs(c);
    }
    proto.set_width(width);

    for (const auto& c : pointcolors) {
        auto* cp = proto.add_pointcolors();
        cp->set_r(c.r); cp->set_g(c.g); cp->set_b(c.b); cp->set_a(c.a);
    }
    for (const auto& c : linecolors) {
        auto* cp = proto.add_linecolors();
        cp->set_r(c.r); cp->set_g(c.g); cp->set_b(c.b); cp->set_a(c.a);
    }

    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }

    return proto.SerializeAsString();
}

NurbsCurve NurbsCurve::pb_loads(const std::string& data) {
    session_proto::NurbsCurve proto;
    proto.ParseFromString(data);

    NurbsCurve curve(proto.dimension(), proto.is_rational(), proto.order(), proto.cv_count());
    curve.guid = proto.guid();
    curve.name = proto.name();
    curve.width = proto.width() != 0.0 ? proto.width() : 1.0;

    curve.m_knot.clear();
    for (int i = 0; i < proto.knots_size(); ++i) {
        curve.m_knot.push_back(proto.knots(i));
    }

    curve.m_cv.clear();
    for (int i = 0; i < proto.cvs_size(); ++i) {
        curve.m_cv.push_back(proto.cvs(i));
    }

    for (int i = 0; i < proto.pointcolors_size(); ++i) {
        const auto& c = proto.pointcolors(i);
        curve.pointcolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }
    for (int i = 0; i < proto.linecolors_size(); ++i) {
        const auto& c = proto.linecolors(i);
        curve.linecolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }

    if (proto.has_xform()) {
        const auto& x = proto.xform();
        curve.xform.guid = x.guid();
        curve.xform.name = x.name();
        for (int i = 0; i < 16 && i < x.matrix_size(); ++i) {
            curve.xform.m[i] = x.matrix(i);
        }
    }

    return curve;
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string NurbsCurve::str() const {
    return fmt::format("NurbsCurve(name={}, degree={}, cvs={})", name, degree(), cv_count());
}

std::string NurbsCurve::repr() const {
    std::string result = fmt::format("NurbsCurve(\n  name={},\n  degree={},\n  cvs={},\n  rational={},\n  control_points=[\n",
                                     name, degree(), m_cv_count, m_is_rat ? "true" : "false");
    for (int i = 0; i < m_cv_count; ++i) {
        Point p = get_cv(i);
        result += fmt::format("    {}, {}, {}\n", p[0], p[1], p[2]);
    }
    result += "  ]\n)";
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Internal Helpers
///////////////////////////////////////////////////////////////////////////////////////////

bool NurbsCurve::reserve_cv_capacity(int capacity) {
    if (capacity > static_cast<int>(m_cv.size())) {
        m_cv.resize(capacity);
        m_cv_capacity = capacity;
    }
    return true;
}

bool NurbsCurve::reserve_knot_capacity(int capacity) {
    if (capacity > static_cast<int>(m_knot.size())) {
        m_knot.resize(capacity);
    }
    return true;
}
// Clean up knots
bool NurbsCurve::clean_knots(double knot_tolerance) {
    if (!is_valid()) return false;
    if (knot_tolerance < 0.0) knot_tolerance = Tolerance::ZERO_TOLERANCE;
    
    // Remove duplicate knots within tolerance
    std::vector<double> cleaned_knots;
    cleaned_knots.push_back(m_knot[0]);
    
    for (size_t i = 1; i < m_knot.size(); i++) {
        if (std::abs(m_knot[i] - cleaned_knots.back()) > knot_tolerance) {
            cleaned_knots.push_back(m_knot[i]);
        }
    }
    
    // Only update if we actually cleaned something
    if (cleaned_knots.size() < m_knot.size()) {
        m_knot = cleaned_knots;
        return true;
    }
    
    return false;
}
// Check if span is linear
bool NurbsCurve::span_is_linear(int span_index, double min_length, double tolerance) const {
    if (!is_valid()) return false;
    if (span_index < 0 || span_index >= m_cv_count - m_order) return false;
    if (m_dim < 2 || m_dim > 3) return false;
    
    // Check if knots at span ends have full multiplicity
    int ki = span_index + m_order - 2;
    if (m_knot[ki] >= m_knot[ki + 1]) return false;
    
    // Check multiplicity at start
    int mult_start = 1;
    for (int i = ki - 1; i >= 0 && m_knot[i] == m_knot[ki]; i--) {
        mult_start++;
    }
    
    // Check multiplicity at end
    int mult_end = 1;
    int kc = knot_count();
    for (int i = ki + 2; i < kc && m_knot[i] == m_knot[ki + 1]; i++) {
        mult_end++;
    }
    
    if (mult_start < m_order - 1 || mult_end < m_order - 1) {
        return false; // Not full multiplicity at ends
    }
    
    // Get endpoints of the span
    Point p0 = get_cv(span_index);
    Point p1 = get_cv(span_index + m_order - 1);
    
    Vector line_vec(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    double line_length = line_vec.magnitude();
    
    if (line_length < min_length) return false;
    
    // Check interior control points
    for (int i = 1; i < m_order - 1; i++) {
        Point p = get_cv(span_index + i);
        Vector v(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
        
        // Compute distance from point to line
        Vector cross = line_vec.cross(v);
        double dist = cross.magnitude() / line_length;
        
        if (dist > tolerance) return false;
        
        // Check that projection is between endpoints
        double t = v.dot(line_vec) / (line_length * line_length);
        if (t < -0.01 || t > 1.01) return false;
    }
    
    return true;
}
// Span is linear with line output
bool NurbsCurve::span_is_linear(int span_index, double min_length, double tolerance,
                               Point* line_start, Point* line_end) const {
    bool is_lin = span_is_linear(span_index, min_length, tolerance);
    
    if (is_lin && line_start && line_end) {
        *line_start = get_cv(span_index);
        *line_end = get_cv(span_index + m_order - 1);
    }
    
    return is_lin;
}
// Check if span is singular
bool NurbsCurve::span_is_singular(int span_index) const {
    if (!is_valid()) return false;
    if (span_index < 0 || span_index >= m_cv_count - m_order) return false;
    
    // Check if span is non-empty
    int ki = span_index + m_order - 2;
    if (m_knot[ki] >= m_knot[ki + 1]) return true; // Empty span
    
    // Check if all CVs in span are coincident
    Point p0 = get_cv(span_index);
    for (int i = 1; i < m_order; i++) {
        Point p = get_cv(span_index + i);
        if (p0.distance(p) > Tolerance::ZERO_TOLERANCE) {
            return false;
        }
    }
    
    return true;
}
// Repair bad knots
bool NurbsCurve::repair_bad_knots(double knot_tolerance, bool repair) {
    if (!is_valid()) return false;
    if (knot_tolerance < 0.0) knot_tolerance = 0.0;
    
    bool found_bad_knots = false;
    int kc = knot_count();
    
    // Check for knots that are too close together
    for (int i = 1; i < kc; i++) {
        double delta = m_knot[i] - m_knot[i-1];
        if (delta < 0.0) {
            found_bad_knots = true;
            break;
        }
        if (delta > 0.0 && delta < knot_tolerance) {
            found_bad_knots = true;
            break;
        }
    }
    
    // Check for excessive multiplicity
    for (int i = 0; i < kc; i++) {
        int mult = knot_multiplicity(i);
        if (mult >= m_order) {
            found_bad_knots = true;
            break;
        }
    }
    
    if (!found_bad_knots) return false;
    
    if (repair) {
        // Simple repair: remove knots that are too close
        std::vector<double> clean_knots;
        clean_knots.push_back(m_knot[0]);
        
        for (int i = 1; i < kc; i++) {
            double delta = m_knot[i] - clean_knots.back();
            if (delta >= knot_tolerance || i == kc - 1) {
                clean_knots.push_back(m_knot[i]);
            }
        }
        
        // Only update if we have valid knot count
        int expected_kc = m_order + m_cv_count - 2;
        if (static_cast<int>(clean_knots.size()) == expected_kc) {
            m_knot = clean_knots;
            return true;
        }
    }
    
    return found_bad_knots;
}
// Get parameter tolerance
bool NurbsCurve::get_parameter_tolerance(double t, double* tminus, double* tplus) const {
    if (!is_valid() || !tminus || !tplus) return false;
    
    auto [t0, t1] = domain();
    if (t < t0 || t > t1) return false;
    
    // Simple implementation: use knot spacing as tolerance
    double delta = (t1 - t0) * std::sqrt(std::numeric_limits<double>::epsilon());
    
    *tminus = t - delta;
    *tplus = t + delta;
    
    // Clamp to domain
    if (*tminus < t0) *tminus = t0;
    if (*tplus > t1) *tplus = t1;
    
    return true;
}
// Zero all control vertices
bool NurbsCurve::zero_cvs() {
    if (!is_valid()) return false;
    
    for (auto& val : m_cv) {
        val = 0.0;
    }
    
    // Set weights to 1.0 if rational
    if (m_is_rat) {
        for (int i = 0; i < m_cv_count; i++) {
            double* cv_ptr = cv(i);
            if (cv_ptr) {
                cv_ptr[m_dim] = 1.0;
            }
        }
    }
    
    return true;
}
// Find span using binary search
int NurbsCurve::find_span(double t) const {
    // Domain is knot[order-2] to knot[cv_count-1]
    const double* knot = m_knot.data() + (m_order - 2);
    int len = m_cv_count - m_order + 2;

    if (t <= knot[0]) return 0;
    if (t >= knot[len - 1]) return len - 2;

    // Binary search
    int low = 0;
    int high = len - 1;
    while (high > low + 1) {
        int mid = (low + high) / 2;
        if (t < knot[mid]) {
            high = mid;
        } else {
            low = mid;
        }
    }

    return low;
}

// Compute basis functions (Cox-de Boor algorithm)
void NurbsCurve::basis_functions(int span, double t, std::vector<double>& basis) const {
    basis.resize(m_order);
    std::vector<double> left(m_order);
    std::vector<double> right(m_order);

    const double* knot = m_knot.data() + (m_order - 2) + span;

    basis[0] = 1.0;

    for (int j = 1; j < m_order; j++) {
        left[j] = t - knot[1 - j];
        right[j] = knot[j] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            double denom = right[r + 1] + left[j - r];
            double temp = (denom != 0.0) ? basis[r] / denom : 0.0;
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis[j] = saved;
    }
}
// Compute basis function derivatives using Piegl & Tiller Algorithm A2.3
void NurbsCurve::basis_functions_derivatives(int span, double t, int deriv_order,
                                            std::vector<std::vector<double>>& ders) const {
    int p = degree();
    int n_der = std::min(deriv_order, p);

    ders.assign(n_der + 1, std::vector<double>(p + 1, 0.0));

    std::vector<double> left(p + 1);
    std::vector<double> right(p + 1);
    std::vector<std::vector<double>> ndu(p + 1, std::vector<double>(p + 1, 0.0));

    // Use same knot offset as basis_functions
    const double* knot = m_knot.data() + (m_order - 2) + span;

    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = t - knot[1 - j];
        right[j] = knot[j] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];  // Store knot differences
            double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    // Load basis functions
    for (int j = 0; j <= p; ++j) {
        ders[0][j] = ndu[j][p];
    }

    // Compute derivatives using Eq. 2.10 from The NURBS Book
    std::vector<std::vector<double>> a(2, std::vector<double>(p + 1, 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;

        for (int k = 1; k <= n_der; ++k) {
            double d = 0.0;
            int rk = r - k;
            int pk = p - k;

            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            int j1 = (rk >= -1) ? 1 : -rk;
            int j2 = (r - 1 <= pk) ? k - 1 : p - r;

            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }

            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }

    // Apply factorial scaling: p!/(p-k)!
    double r = (double)p;
    for (int k = 1; k <= n_der; ++k) {
        for (int j = 0; j <= p; ++j) {
            ders[k][j] *= r;
        }
        r *= (double)(p - k);
    }
}
void NurbsCurve::deep_copy_from(const NurbsCurve& src) {
    m_dim = src.m_dim;
    m_is_rat = src.m_is_rat;
    m_order = src.m_order;
    m_cv_count = src.m_cv_count;
    m_cv_stride = src.m_cv_stride;
    m_cv_capacity = src.m_cv_capacity;
    m_knot = src.m_knot;
    m_cv = src.m_cv;
    guid = ::guid();
    name = src.name;
    width = src.width;
    pointcolors = src.pointcolors;
    linecolors = src.linecolors;
    xform = src.xform;
    invalidate_rmf_cache();
}
// De Boor algorithm for trimming/extending B-spline spans
// Based on OpenNURBS ON_EvaluateNurbsDeBoor
bool NurbsCurve::evaluate_nurbs_de_boor(int cv_dim, int order, int cv_stride,
                                        double* cv, const double* knots,
                                        int side, double t) {
    int degree = order - 1;
    double t0 = knots[degree - 1];
    double t1 = knots[degree];

    if (t0 == t1) return false;

    int cv_inc = cv_stride - cv_dim;

    if (side < 0) {
        // Return left side of span (for extending right or clamping end)
        if (t == t1 && t1 == knots[2 * degree - 1]) return true;

        // Check if left end is fully multiple
        bool fully_multiple = (t0 == knots[0]);

        // Advance knots pointer by degree-1
        knots += degree - 1;

        if (fully_multiple) {
            double dt = t - t0;
            cv += order * cv_stride;
            int k = order;
            while (--k) {
                double* cv1 = cv;
                double* cv0 = cv1 - cv_stride;
                const double* k1 = knots + k;
                int i = k;
                while (i--) {
                    double alpha1 = dt / (*k1-- - t0);
                    double alpha0 = 1.0 - alpha1;
                    cv0 -= cv_inc;
                    cv1 -= cv_inc;
                    int j = cv_dim;
                    while (j--) {
                        cv0--;
                        cv1--;
                        *cv1 = *cv0 * alpha0 + *cv1 * alpha1;
                    }
                }
            }
        } else {
            // Variable left end knots: delta_t = {t - knots[d-1], t - knots[d-2], ..., t - knots[0]}
            std::vector<double> delta_t(degree);
            const double* k0 = knots;
            for (int idx = 0; idx < degree; idx++) {
                delta_t[idx] = t - *k0--;
            }

            cv += order * cv_stride;
            int k = order;
            while (--k) {
                double* cv1 = cv;
                double* cv0 = cv1 - cv_stride;
                k0 = knots;
                const double* k1 = k0 + k;
                int di = 0;
                int i = k;
                while (i--) {
                    double alpha1 = delta_t[di++] / (*k1-- - *k0--);
                    double alpha0 = 1.0 - alpha1;
                    cv0 -= cv_inc;
                    cv1 -= cv_inc;
                    int j = cv_dim;
                    while (j--) {
                        cv0--;
                        cv1--;
                        *cv1 = *cv0 * alpha0 + *cv1 * alpha1;
                    }
                }
            }
        }
    } else {
        // Return right side of span (for extending left or clamping start)
        if (t == t0 && t0 == knots[0]) return true;

        // Check if right end is fully multiple
        bool fully_multiple = (t1 == knots[2 * degree - 1]);

        // Advance knots pointer by degree
        knots += degree;

        if (fully_multiple) {
            double dt = t1 - t;
            int k = order;
            while (--k) {
                double* cv0 = cv;
                double* cv1 = cv0 + cv_stride;
                const double* k0 = knots - k;
                int i = k;
                while (i--) {
                    double alpha0 = dt / (t1 - *k0++);
                    double alpha1 = 1.0 - alpha0;
                    int j = cv_dim;
                    while (j--) {
                        *cv0 = *cv0 * alpha0 + *cv1 * alpha1;
                        cv0++;
                        cv1++;
                    }
                    cv0 += cv_inc;
                    cv1 += cv_inc;
                }
            }
        } else {
            // Variable right end knots: delta_t = {knots[d] - t, knots[d+1] - t, ...}
            std::vector<double> delta_t(degree);
            const double* k1 = knots;
            for (int idx = 0; idx < degree; idx++) {
                delta_t[idx] = *k1++ - t;
            }

            int k = order;
            while (--k) {
                double* cv0 = cv;
                double* cv1 = cv0 + cv_stride;
                k1 = knots;
                const double* k0 = k1 - k;
                int di = 0;
                int i = k;
                while (i--) {
                    double alpha0 = delta_t[di++] / (*k1++ - *k0++);
                    double alpha1 = 1.0 - alpha0;
                    int j = cv_dim;
                    while (j--) {
                        *cv0 = *cv0 * alpha0 + *cv1 * alpha1;
                        cv0++;
                        cv1++;
                    }
                    cv0 += cv_inc;
                    cv1 += cv_inc;
                }
            }
        }
    }

    return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
// RMF Cache
///////////////////////////////////////////////////////////////////////////////////////////

// Quaternion helpers for RMF caching
std::array<double, 4> NurbsCurve::frame_to_quaternion(const Vector& r, const Vector& s, const Vector& t) {
    double trace = r[0] + s[1] + t[2];
    double w, x, y, z;

    if (trace > 0) {
        double S = std::sqrt(trace + 1.0) * 2;
        w = 0.25 * S;
        x = (s[2] - t[1]) / S;
        y = (t[0] - r[2]) / S;
        z = (r[1] - s[0]) / S;
    } else if (r[0] > s[1] && r[0] > t[2]) {
        double S = std::sqrt(1.0 + r[0] - s[1] - t[2]) * 2;
        w = (s[2] - t[1]) / S;
        x = 0.25 * S;
        y = (s[0] + r[1]) / S;
        z = (t[0] + r[2]) / S;
    } else if (s[1] > t[2]) {
        double S = std::sqrt(1.0 + s[1] - r[0] - t[2]) * 2;
        w = (t[0] - r[2]) / S;
        x = (s[0] + r[1]) / S;
        y = 0.25 * S;
        z = (t[1] + s[2]) / S;
    } else {
        double S = std::sqrt(1.0 + t[2] - r[0] - s[1]) * 2;
        w = (r[1] - s[0]) / S;
        x = (t[0] + r[2]) / S;
        y = (t[1] + s[2]) / S;
        z = 0.25 * S;
    }
    return {w, x, y, z};
}

void NurbsCurve::quaternion_to_frame(const std::array<double, 4>& q, Vector& r, Vector& s, Vector& t) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    r = Vector(1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y));
    s = Vector(2*(x*y - w*z), 1 - 2*(x*x + z*z), 2*(y*z + w*x));
    t = Vector(2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y));
}

std::array<double, 4> NurbsCurve::slerp(const std::array<double, 4>& q0, const std::array<double, 4>& q1, double u) {
    double dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

    std::array<double, 4> q1_adj = q1;
    if (dot < 0) {
        q1_adj = {-q1[0], -q1[1], -q1[2], -q1[3]};
        dot = -dot;
    }

    if (dot > 0.9995) {
        std::array<double, 4> result = {
            q0[0] + u * (q1_adj[0] - q0[0]),
            q0[1] + u * (q1_adj[1] - q0[1]),
            q0[2] + u * (q1_adj[2] - q0[2]),
            q0[3] + u * (q1_adj[3] - q0[3])
        };
        double norm = std::sqrt(result[0]*result[0] + result[1]*result[1] +
                                result[2]*result[2] + result[3]*result[3]);
        return {result[0]/norm, result[1]/norm, result[2]/norm, result[3]/norm};
    }

    double theta = std::acos(dot);
    double sin_theta = std::sin(theta);
    double w0 = std::sin((1-u) * theta) / sin_theta;
    double w1 = std::sin(u * theta) / sin_theta;

    return {
        w0*q0[0] + w1*q1_adj[0],
        w0*q0[1] + w1*q1_adj[1],
        w0*q0[2] + w1*q1_adj[2],
        w0*q0[3] + w1*q1_adj[3]
    };
}

void NurbsCurve::invalidate_rmf_cache() const {
    m_rmf_cached = false;
    m_rmf_params.clear();
    m_rmf_quaternions.clear();
    m_rmf_origins.clear();
}

void NurbsCurve::ensure_rmf_cache() const {
    if (m_rmf_cached) return;

    int num_samples = std::max(20, span_count() * 4);
    auto [t0, t1] = domain();
    double dt = (t1 - t0) / (num_samples - 1);

    m_rmf_params.resize(num_samples);
    m_rmf_quaternions.resize(num_samples);
    m_rmf_origins.resize(num_samples);

    for (int i = 0; i < num_samples; i++) {
        double t = t0 + i * dt;
        m_rmf_params[i] = t;

        Plane pl = perpendicular_plane_at(t, false);
        m_rmf_origins[i] = pl.origin();
        m_rmf_quaternions[i] = frame_to_quaternion(pl.x_axis(), pl.y_axis(), pl.z_axis());
    }

    m_rmf_cached = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Return-value overloads
///////////////////////////////////////////////////////////////////////////////////////////

std::tuple<double, double, double, double> NurbsCurve::get_cv_4d(int cv_index) const {
    double x = 0, y = 0, z = 0, w = 1;
    get_cv_4d(cv_index, x, y, z, w);
    return {x, y, z, w};
}

std::vector<double> NurbsCurve::get_greville_abcissae() const {
    std::vector<double> result;
    get_greville_abcissae(result);
    return result;
}

std::pair<bool, double> NurbsCurve::get_next_discontinuity(int continuity_type, double t0, double t1) const {
    double t_out = 0.0;
    bool found = get_next_discontinuity(continuity_type, t0, t1, t_out);
    return {found, t_out};
}

std::pair<std::vector<Point>, std::vector<double>> NurbsCurve::to_polyline_adaptive(
    double angle_tolerance, double min_edge_length, double max_edge_length) const {
    std::vector<Point> pts;
    std::vector<double> params;
    to_polyline_adaptive(pts, &params, angle_tolerance, min_edge_length, max_edge_length);
    return {pts, params};
}

std::pair<std::vector<Point>, std::vector<double>> NurbsCurve::divide_by_count(int count, bool include_endpoints) const {
    std::vector<Point> pts;
    std::vector<double> params;
    divide_by_count(count, pts, &params, include_endpoints);
    return {pts, params};
}

std::pair<std::vector<Point>, std::vector<double>> NurbsCurve::divide_by_length(double segment_length) const {
    std::vector<Point> pts;
    std::vector<double> params;
    divide_by_length(segment_length, pts, &params);
    return {pts, params};
}

std::pair<NurbsCurve, NurbsCurve> NurbsCurve::split(double t) const {
    NurbsCurve left, right;
    split(t, left, right);
    return {left, right};
}

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const NurbsCurve& curve) {
    os << curve.str();
    return os;
}

} // namespace session_cpp
