#include "nurbscurve.h"
#include "knot.h"
#include <cstring>
#include <sstream>
#include <limits>

namespace session_cpp {

// Constructors & Destructor
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

NurbsCurve::~NurbsCurve() {
    destroy();
}

// Initialization
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

// Static factory method - RhinoCommon-style unified API
NurbsCurve NurbsCurve::create(bool periodic, int degree, const std::vector<Point>& points,
                              int dimension, double knot_delta) {
    NurbsCurve curve;
    int order = degree + 1;
    
    if (periodic) {
        curve.create_periodic_uniform(dimension, order, points, knot_delta);
    } else {
        curve.create_clamped_uniform(dimension, order, points, knot_delta);
    }
    
    return curve;
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

// Validation
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

// Accessors
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

// CV Access
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
        if (w != 0.0) {
            return Point(cv_ptr[0]/w, cv_ptr[1]/w, m_dim > 2 ? cv_ptr[2]/w : 0.0);
        }
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
    
    return true;
}

bool NurbsCurve::set_cv_4d(int cv_index, double x, double y, double z, double w) {
    double* cv_ptr = cv(cv_index);
    if (!cv_ptr) return false;
    
    if (m_is_rat) {
        cv_ptr[0] = x;
        if (m_dim > 1) cv_ptr[1] = y;
        if (m_dim > 2) cv_ptr[2] = z;
        cv_ptr[m_dim] = w;
    } else {
        if (w != 0.0) {
            cv_ptr[0] = x / w;
            if (m_dim > 1) cv_ptr[1] = y / w;
            if (m_dim > 2) cv_ptr[2] = z / w;
        }
    }
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
    return true;
}

// Knot Access
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
    return true;
}

// Domain
std::pair<double, double> NurbsCurve::domain() const {
    if (m_knot.empty()) return {0.0, 0.0};
    return {m_knot[m_order-2], m_knot[m_cv_count-1]};
}

double NurbsCurve::domain_start() const {
    if (m_knot.empty()) return 0.0;
    return m_knot[m_order-2];
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

// Geometric Queries
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

// Find span using binary search
// Implementation matches OpenNURBS ON_NurbsSpanIndex
int NurbsCurve::find_span(double t) const {
    // OpenNURBS shifts knot pointer by (order-2) to work with compressed format
    // Domain is knot[order-2] to knot[cv_count-1]
    const double* knot = m_knot.data() + (m_order - 2);
    int len = m_cv_count - m_order + 2;
    
    // Binary search for span
    int low = 0;
    int high = len - 1;
    
    if (t <= knot[0]) return 0;
    if (t >= knot[len-1]) return len - 2;
    
    // Binary search
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
// Defensive implementation to prevent divide-by-zero at repeated knots
void NurbsCurve::basis_functions(int span, double t, std::vector<double>& basis) const {
    basis.resize(m_order);
    std::vector<double> left(m_order);
    std::vector<double> right(m_order);
    
    const double eps = 1e-14;
    
    // Offset knot pointer like OpenNURBS does
    const double* knot = m_knot.data() + (m_order - 2) + span;
    
    basis[0] = 1.0;
    
    for (int j = 1; j < m_order; j++) {
        left[j] = t - knot[1 - j];
        right[j] = knot[j] - t;
        double saved = 0.0;
        
        for (int r = 0; r < j; r++) {
            double denom = right[r + 1] + left[j - r];
            double temp;
            if (std::abs(denom) <= eps) {
                temp = 0.0;  // Safe fallback for zero denominator
            } else {
                temp = basis[r] / denom;
            }
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis[j] = saved;
    }
}

// Evaluate point on curve
// Implementation matches OpenNURBS evaluation approach
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
            double ww = cv_ptr[m_dim];
            x += N * cv_ptr[0];
            y += N * (m_dim > 1 ? cv_ptr[1] : 0.0);
            z += N * (m_dim > 2 ? cv_ptr[2] : 0.0);
            w += N * ww;
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
            int cv_idx = span - p + j;
            const double* cv_ptr = cv(cv_idx);
            if (!cv_ptr) continue;

            double Nx = ders[k][j];
            double cx = cv_ptr[0];
            double cy = (m_dim > 1) ? cv_ptr[1] : 0.0;
            double cz = (m_dim > 2) ? cv_ptr[2] : 0.0;
            double wv = m_is_rat ? cv_ptr[m_dim] : 1.0;

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

bool NurbsCurve::frame_at(double t, bool normalized, Point& origin,
                          Vector& xaxis, Vector& yaxis, Vector& zaxis) const {
    if (!is_valid()) return false;

    auto [t0, t1] = domain();
    double param;
    if (normalized) {
        if (t < 0.0 || t > 1.0) return false;
        param = t0 + t * (t1 - t0);
    } else {
        if (t < t0 || t > t1) return false;
        param = t;
    }
    double h = (t1 - t0) * 1e-5;

    origin = point_at(param);

    Point pm, p0, pp;
    if (param <= t0 + h) {
        p0 = point_at(t0);
        pp = point_at(t0 + h);
        Point pp2 = point_at(t0 + 2*h);
        pm = p0;
        Vector d1(pp[0] - p0[0], pp[1] - p0[1], pp[2] - p0[2]);
        Vector d2((pp2[0] - 2*pp[0] + p0[0])/(h*h), (pp2[1] - 2*pp[1] + p0[1])/(h*h), (pp2[2] - 2*pp[2] + p0[2])/(h*h));

        double d1_mag = d1.magnitude();
        if (d1_mag < 1e-14) return false;

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

        xaxis = T;
        yaxis = N;
        zaxis = B;
        return true;
    } else if (param >= t1 - h) {
        pm = point_at(t1 - h);
        p0 = point_at(t1);
        Point pm2 = point_at(t1 - 2*h);
        Vector d1(p0[0] - pm[0], p0[1] - pm[1], p0[2] - pm[2]);
        Vector d2((p0[0] - 2*pm[0] + pm2[0])/(h*h), (p0[1] - 2*pm[1] + pm2[1])/(h*h), (p0[2] - 2*pm[2] + pm2[2])/(h*h));

        double d1_mag = d1.magnitude();
        if (d1_mag < 1e-14) return false;

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

        xaxis = T;
        yaxis = N;
        zaxis = B;
        return true;
    }

    pm = point_at(param - h);
    p0 = point_at(param);
    pp = point_at(param + h);

    Vector d1((pp[0] - pm[0])/(2*h), (pp[1] - pm[1])/(2*h), (pp[2] - pm[2])/(2*h));
    Vector d2((pp[0] - 2*p0[0] + pm[0])/(h*h), (pp[1] - 2*p0[1] + pm[1])/(h*h), (pp[2] - 2*p0[2] + pm[2])/(h*h));

    double d1_mag = d1.magnitude();
    if (d1_mag < 1e-14) return false;

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

    xaxis = T;
    yaxis = N;
    zaxis = B;
    return true;
}

bool NurbsCurve::perpendicular_frame_at(double t, bool normalized, Point& origin,
                                        Vector& xaxis, Vector& yaxis, Vector& zaxis) const {
    if (!is_valid()) return false;

    auto [t0, t1] = domain();
    double param;
    if (normalized) {
        if (t < 0.0 || t > 1.0) return false;
        param = t0 + t * (t1 - t0);
    } else {
        if (t < t0 || t > t1) return false;
        param = t;
    }

    origin = point_at(param);

    Vector T0 = tangent_at(t0);
    double T0_mag = T0.magnitude();
    if (T0_mag < 1e-14) return false;
    T0.normalize_self();

    Vector worldZ(0, 0, 1);
    Vector r0 = worldZ.cross(T0);
    double r0_mag = r0.magnitude();
    if (r0_mag < 1e-14) {
        Vector worldY(0, 1, 0);
        r0 = worldY.cross(T0);
        r0_mag = r0.magnitude();
    }
    if (r0_mag > 1e-14) r0.normalize_self();

    if (std::abs(param - t0) < 1e-14) {
        Vector s0 = T0.cross(r0);
        s0.normalize_self();
        xaxis = r0;
        yaxis = s0;
        zaxis = T0;
        return true;
    }

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

    double ri_dot_T = ri.dot(T);
    ri = Vector(ri[0] - ri_dot_T * T[0], ri[1] - ri_dot_T * T[1], ri[2] - ri_dot_T * T[2]);
    double ri_mag = ri.magnitude();
    if (ri_mag > 1e-14) ri.normalize_self();

    Vector s = T.cross(ri);
    s.normalize_self();

    xaxis = ri;
    yaxis = s;
    zaxis = T;
    return true;
}

std::vector<std::tuple<Point, Vector, Vector, Vector>>
NurbsCurve::get_perpendicular_frames(const std::vector<double>& params) const {
    std::vector<std::tuple<Point, Vector, Vector, Vector>> frames;
    for (double t : params) {
        Point o;
        Vector x, y, z;
        perpendicular_frame_at(t, true, o, x, y, z);
        frames.emplace_back(o, x, y, z);
    }
    return frames;
}

Point NurbsCurve::point_at_start() const {
    auto [t0, t1] = domain();
    return point_at(t0);
}

Point NurbsCurve::point_at_end() const {
    auto [t0, t1] = domain();
    return point_at(t1);
}

// Transformation
void NurbsCurve::transform() {
    transform(xform);
}

bool NurbsCurve::transform(const Xform& xf) {
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        // Apply xform matrix (column-major: m[col*4 + row])
        double x = xf.m[0] * p[0] + xf.m[4] * p[1] + xf.m[8] * p[2] + xf.m[12];
        double y = xf.m[1] * p[0] + xf.m[5] * p[1] + xf.m[9] * p[2] + xf.m[13];
        double z = xf.m[2] * p[0] + xf.m[6] * p[1] + xf.m[10] * p[2] + xf.m[14];
        set_cv(i, Point(x, y, z));
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

// Modification
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
    
    return true;
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

bool NurbsCurve::make_non_rational() {
    if (!m_is_rat) return true;
    
    // Check if all weights are equal
    double w0 = weight(0);
    for (int i = 1; i < m_cv_count; i++) {
        if (std::abs(weight(i) - w0) > Tolerance::ZERO_TOLERANCE) {
            return false;
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

// Helpers
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
        // Use existing compressed find_span and map to full index: k = span + 1
        int span = find_span(knot_value);
        int k = span + 1;

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

    return true;
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
    guid = ::guid();  // Generate new GUID for copy
    name = src.name;
    width = src.width;
    linecolor = src.linecolor;
    xform = src.xform;
}

// String & JSON
std::string NurbsCurve::to_string() const {
    std::ostringstream oss;
    oss << "NurbsCurve(dim=" << m_dim << ", order=" << m_order
        << ", cv_count=" << m_cv_count << ", rational=" << (m_is_rat ? "true" : "false") << ")";
    return oss.str();
}

std::string NurbsCurve::str() const {
    return fmt::format("degree={}, cvs={}", degree(), cv_count());
}

std::string NurbsCurve::repr() const {
    return fmt::format("NurbsCurve({}, dim={}, order={}, cvs={}, rational={})",
                       name, m_dim, m_order, m_cv_count, m_is_rat ? "true" : "false");
}

nlohmann::ordered_json NurbsCurve::jsondump() const {
    nlohmann::ordered_json j;
    j["guid"] = guid;
    j["name"] = name;
    j["dimension"] = m_dim;
    j["is_rational"] = m_is_rat != 0;
    j["order"] = m_order;
    j["cv_count"] = m_cv_count;
    j["knots"] = m_knot;
    j["control_points"] = nlohmann::json::array();
    
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        j["control_points"].push_back({p[0], p[1], p[2]});
    }
    
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
    }
    
    return curve;
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

void NurbsCurve::protobuf_dump(const std::string& filename) const {
    // Stub: uses JSON fallback (replace .bin with .json)
    std::string json_filename = filename;
    size_t pos = json_filename.rfind(".bin");
    if (pos != std::string::npos) {
        json_filename.replace(pos, 4, ".json");
    }
    json_dump(json_filename);
}

NurbsCurve NurbsCurve::protobuf_load(const std::string& filename) {
    // Stub: uses JSON fallback (replace .bin with .json)
    std::string json_filename = filename;
    size_t pos = json_filename.rfind(".bin");
    if (pos != std::string::npos) {
        json_filename.replace(pos, 4, ".json");
    }
    return json_load(json_filename);
}

// Stream operator
std::ostream& operator<<(std::ostream& os, const NurbsCurve& curve) {
    os << curve.to_string();
    return os;
}

// Clamp end knots
bool NurbsCurve::clamp_end(int end) {
    if (!is_valid()) return false;
    
    // end: 0 = start, 1 = end, 2 = both
    if (end < 0 || end > 2) return false;
    
    // Clamp start
    if (end == 0 || end == 2) {
        for (int i = 0; i < m_order - 1; i++) {
            m_knot[i] = m_knot[m_order - 2];
        }
    }
    
    // Clamp end
    if (end == 1 || end == 2) {
        int knot_count = this->knot_count();
        for (int i = m_cv_count; i < knot_count; i++) {
            m_knot[i] = m_knot[m_cv_count - 1];
        }
    }
    
    return true;
}

// Trim curve to interval
bool NurbsCurve::trim(double t0, double t1) {
    if (!is_valid() || t0 >= t1) return false;
    
    auto [d0, d1] = domain();
    if (t0 == d0 && t1 == d1) return true; // Already at desired domain
    
    // This is a simplified trim - for production use, need full de Boor algorithm
    // For now, just adjust domain
    return set_domain(t0, t1);
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

// Extend curve domain
bool NurbsCurve::extend(double t0, double t1) {
    if (!is_valid() || is_closed()) return false;
    
    auto [d0, d1] = domain();
    bool changed = false;
    
    if (t0 < d0) {
        clamp_end(0);
        // Adjust start knots
        for (int i = 0; i < m_order - 1; i++) {
            m_knot[i] = t0;
        }
        changed = true;
    }
    
    if (t1 > d1) {
        clamp_end(1);
        // Adjust end knots
        int knot_count = this->knot_count();
        for (int i = m_cv_count; i < knot_count; i++) {
            m_knot[i] = t1;
        }
        changed = true;
    }
    
    return changed;
}

// Get bounding box
BoundingBox NurbsCurve::get_bounding_box() const {
    if (!is_valid() || m_cv_count == 0) {
        return BoundingBox();
    }
    
    std::vector<Point> points;
    for (int i = 0; i < m_cv_count; i++) {
        points.push_back(get_cv(i));
    }
    
    return BoundingBox::from_points(points);
}

// Compute arc length (approximate)
double NurbsCurve::length(double tolerance) const {
    if (!is_valid()) return 0.0;
    
    auto [t0, t1] = domain();
    
    // Adaptive sampling based on tolerance
    // Smaller tolerance = more samples for better accuracy
    int num_samples = std::max(50, static_cast<int>(100.0 / (tolerance + 1e-10)));
    num_samples = std::min(num_samples, 1000); // Cap at 1000 samples
    
    double dt = (t1 - t0) / num_samples;
    double total_length = 0.0;
    
    Point prev = point_at(t0);
    for (int i = 1; i <= num_samples; i++) {
        Point curr = point_at(t0 + i * dt);
        total_length += prev.distance(curr);
        prev = curr;
    }
    
    return total_length;
}

// Find closest point on curve
Point NurbsCurve::closest_point(const Point& point, double& t_out) const {
    if (!is_valid()) {
        t_out = 0.0;
        return Point(0, 0, 0);
    }
    
    auto [t0, t1] = domain();
    
    // Simple search using sampling
    int num_samples = 100;
    double dt = (t1 - t0) / num_samples;
    double min_dist = std::numeric_limits<double>::max();
    double best_t = t0;
    
    for (int i = 0; i <= num_samples; i++) {
        double t = t0 + i * dt;
        Point p = point_at(t);
        double dist = point.distance(p);
        if (dist < min_dist) {
            min_dist = dist;
            best_t = t;
        }
    }
    
    t_out = best_t;
    return point_at(best_t);
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

// Compute basis function derivatives
void NurbsCurve::basis_functions_derivatives(int span, double t, int deriv_order,
                                            std::vector<std::vector<double>>& ders) const {
    // Algorithm A2.3 from "The NURBS Book" (Piegl & Tiller)
    int p = degree();
    int n_der = std::min(deriv_order, p);

    ders.assign(n_der + 1, std::vector<double>(p + 1, 0.0));

    std::vector<double> left(p + 1);
    std::vector<double> right(p + 1);
    std::vector<std::vector<double>> ndu(p + 1, std::vector<double>(p + 1, 0.0));

    // Offset knot pointer like OpenNURBS and basis_functions do
    const double* knot = m_knot.data() + (m_order - 2) + span;
    
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = t - knot[1 - j];
        right[j] = knot[j] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double temp = ndu[r][j - 1] / (right[r + 1] + left[j - r]);
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    // Load basis functions
    for (int j = 0; j <= p; ++j) {
        ders[0][j] = ndu[j][p];
    }

    // Compute derivatives
    std::vector<std::vector<double>> a(2, std::vector<double>(p + 1, 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;

        for (int k = 1; k <= n_der; ++k) {
            double d = 0.0;
            int rk = r - k;
            int pk = p - k;
            int j1, j2;
            if (r >= k) {
                a[s2][0] = a[s1][0] / (right[rk + 1] + left[k]);
                d = a[s2][0] * ndu[rk][pk];
            } else {
                a[s2][0] = 0.0;
            }
            if (rk >= -1) {
                j1 = 1;
            } else {
                j1 = -rk;
            }
            if (r - 1 <= pk) {
                j2 = k - 1;
            } else {
                j2 = p - r;
            }
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / (right[rk + j + 1] + left[k - j]);
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / (right[r + 1] + left[pk - r]);
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }

    // Multiply derivatives by the correct factorial term
    double factorial = 1.0;
    for (int k = 1; k <= n_der; ++k) {
        factorial *= k;
        for (int j = 0; j <= p; ++j) {
            ders[k][j] *= factorial;
        }
    }
}

// Check if knot vector is clamped
bool NurbsCurve::is_clamped(int end) const {
    if (!is_valid()) return false;
    
    // Use knot module function
    return knot::is_clamped(m_order, m_cv_count, m_knot, end);
}

// Get control polygon length
double NurbsCurve::control_polygon_length() const {
    if (!is_valid() || m_cv_count < 2) return 0.0;
    
    double length = 0.0;
    Point prev = get_cv(0);
    
    for (int i = 1; i < m_cv_count; i++) {
        Point curr = get_cv(i);
        length += prev.distance(curr);
        prev = curr;
    }
    
    return length;
}

// Get Greville abcissa for a control point
double NurbsCurve::greville_abcissa(int cv_index) const {
    if (cv_index < 0 || cv_index >= m_cv_count) return 0.0;
    
    // Greville abcissa is the average of p consecutive knots starting at cv_index
    int p = degree();
    double sum = 0.0;
    for (int i = 0; i < p; i++) {
        sum += m_knot[cv_index + i];
    }
    return sum / p;
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

// Check if entire curve is singular
bool NurbsCurve::is_singular() const {
    if (!is_valid()) return false;
    
    int span_count = this->span_count();
    for (int i = 0; i < span_count; i++) {
        if (!span_is_singular(i)) {
            return false;
        }
    }
    
    return true;
}

// Check if curve has bezier spans
bool NurbsCurve::has_bezier_spans() const {
    if (!is_valid()) return false;
    
    int p = degree();
    int kc = knot_count();
    
    // Check each distinct knot has multiplicity = degree
    std::vector<double> distinct_knots;
    std::vector<int> multiplicities;
    
    distinct_knots.push_back(m_knot[0]);
    multiplicities.push_back(1);
    
    for (int i = 1; i < kc; i++) {
        if (std::abs(m_knot[i] - distinct_knots.back()) < Tolerance::ZERO_TOLERANCE) {
            multiplicities.back()++;
        } else {
            distinct_knots.push_back(m_knot[i]);
            multiplicities.push_back(1);
        }
    }
    
    // Check interior knots have multiplicity = degree
    for (size_t i = 1; i < distinct_knots.size() - 1; i++) {
        if (multiplicities[i] != p) {
            return false;
        }
    }
    
    return true;
}

// Make piecewise bezier
bool NurbsCurve::make_piecewise_bezier(bool set_end_weights_to_one) {
    if (has_bezier_spans()) return true;
    if (!is_valid()) return false;
    
    // First clamp the ends
    if (!clamp_end(2)) return false;
    
    // For each span, insert knots to achieve multiplicity = degree
    int p = degree();
    std::vector<double> span_params = get_span_vector();
    
    // Insert knots at each interior span parameter
    for (size_t i = 1; i < span_params.size() - 1; i++) {
        double t = span_params[i];
        
        // Count current multiplicity
        int mult = 0;
        for (int j = 0; j < knot_count(); j++) {
            if (std::abs(m_knot[j] - t) < Tolerance::ZERO_TOLERANCE) {
                mult++;
            }
        }
        
        // Insert knots to reach degree multiplicity
        if (mult < p) {
            if (!insert_knot(t, p - mult)) {
                return false;
            }
        }
    }
    
    // TODO: Implement set_end_weights_to_one if needed
    (void)set_end_weights_to_one; // Suppress unused warning
    
    return true;
}

// Make clamped uniform knot vector
bool NurbsCurve::make_clamped_uniform_knot_vector(double delta) {
    // Don't call is_valid() here as it checks knot vector which we're about to create
    if (delta <= 0.0) return false;
    if (m_dim <= 0) return false;
    if (m_order < 2 || m_cv_count < m_order) return false;
    
    // Use knot module function
    m_knot = knot::make_clamped_uniform(m_order, m_cv_count, delta);
    return !m_knot.empty();
}

// Make periodic uniform knot vector
bool NurbsCurve::make_periodic_uniform_knot_vector(double delta) {
    // Don't call is_valid() here as it checks knot vector which we're about to create
    if (delta <= 0.0) return false;
    if (m_dim <= 0) return false;
    if (m_order < 2 || m_cv_count < m_order) return false;
    
    // Use knot module function
    m_knot = knot::make_periodic_uniform(m_order, m_cv_count, delta);
    return !m_knot.empty();
}

// Increase degree
bool NurbsCurve::increase_degree(int desired_degree) {
    if (!is_valid()) return false;
    if (desired_degree <= degree()) return true; // Already at or above desired degree
    
    int degree_inc = desired_degree - degree();
    
    // Increase degree one at a time
    for (int inc = 0; inc < degree_inc; inc++) {
        int old_order = m_order;
        int old_cv_count = m_cv_count;
        int new_order = old_order + 1;
        int new_cv_count = old_cv_count + old_cv_count - old_order + 1;
        
        // Get old data
        std::vector<double> old_knots = m_knot;
        std::vector<double> old_cvs = m_cv;
        
        // Prepare new data
        int new_knot_count = new_order + new_cv_count - 2;
        std::vector<double> new_knots(new_knot_count);
        std::vector<double> new_cvs(new_cv_count * cv_size());
        
        // Each distinct knot value gets one more copy
        int new_k = 0;
        for (int old_k = 0; old_k < (int)old_knots.size(); ) {
            double knot_value = old_knots[old_k];
            int mult = 1;
            
            // Count multiplicity
            while (old_k + mult < (int)old_knots.size() && 
                   std::abs(old_knots[old_k + mult] - knot_value) < Tolerance::ZERO_TOLERANCE) {
                mult++;
            }
            
            // Insert mult+1 copies in new knot vector
            for (int i = 0; i <= mult; i++) {
                if (new_k < new_knot_count) {
                    new_knots[new_k++] = knot_value;
                }
            }
            
            old_k += mult;
        }
        
        // Compute new control points using degree elevation formula
        int cvs = cv_size();
        for (int i = 0; i < new_cv_count; i++) {
            if (i == 0) {
                // First CV stays the same
                for (int j = 0; j < cvs; j++) {
                    new_cvs[i * cvs + j] = old_cvs[j];
                }
            } else if (i >= old_cv_count) {
                // Last CV stays the same
                for (int j = 0; j < cvs; j++) {
                    new_cvs[i * cvs + j] = old_cvs[(old_cv_count - 1) * cvs + j];
                }
            } else {
                // Interior CVs are weighted average
                double alpha = (double)i / (double)new_order;
                for (int j = 0; j < cvs; j++) {
                    double cv_prev = (i - 1 >= 0 && i - 1 < old_cv_count) ? old_cvs[(i - 1) * cvs + j] : 0.0;
                    double cv_curr = (i < old_cv_count) ? old_cvs[i * cvs + j] : 0.0;
                    new_cvs[i * cvs + j] = alpha * cv_prev + (1.0 - alpha) * cv_curr;
                }
            }
        }
        
        // Update curve data
        m_order = new_order;
        m_cv_count = new_cv_count;
        m_knot = new_knots;
        m_cv = new_cvs;
    }
    
    return true;
}

// Append another curve (full implementation)
bool NurbsCurve::append(const NurbsCurve& other) {
    if (!is_valid() || !other.is_valid()) return false;
    if (m_dim != other.m_dim) return false;
    if (m_is_rat != other.m_is_rat) return false;
    
    // Check if curves are connected
    Point this_end = point_at_end();
    Point other_start = other.point_at_start();
    double gap = this_end.distance(other_start);
    if (gap > Tolerance::ZERO_TOLERANCE * 10.0) {
        return false; // Curves must be connected
    }
    
    // Make a copy to work with
    NurbsCurve other_copy = other;
    
    // Make both curves same degree
    int max_degree = std::max(degree(), other_copy.degree());
    if (degree() < max_degree) {
        if (!increase_degree(max_degree)) return false;
    }
    if (other_copy.degree() < max_degree) {
        if (!other_copy.increase_degree(max_degree)) return false;
    }
    
    // Get domain information
    auto [t0_this, t1_this] = domain();
    auto [t0_other, t1_other] = other_copy.domain();
    
    // Reparameterize other curve to continue from this curve's end
    double domain_shift = t1_this - t0_other;
    for (int i = 0; i < other_copy.knot_count(); i++) {
        other_copy.m_knot[i] += domain_shift;
    }
    
    // Merge knot vectors - remove overlapping knots at join
    std::vector<double> new_knots;
    int this_knot_count = knot_count();
    int other_knot_count = other_copy.knot_count();
    
    // Add all knots from this curve
    for (int i = 0; i < this_knot_count; i++) {
        new_knots.push_back(m_knot[i]);
    }
    
    // Add knots from other curve, skipping the first 'order' knots (overlap region)
    for (int i = m_order; i < other_knot_count; i++) {
        new_knots.push_back(other_copy.m_knot[i]);
    }
    
    // Merge control points - average the overlapping CV at join
    std::vector<double> new_cvs;
    int cvs = cv_size();
    
    // Add all CVs from this curve except the last one
    for (int i = 0; i < m_cv_count - 1; i++) {
        for (int j = 0; j < cvs; j++) {
            new_cvs.push_back(m_cv[i * cvs + j]);
        }
    }
    
    // Average the last CV of this curve with first CV of other curve
    for (int j = 0; j < cvs; j++) {
        double val_this = m_cv[(m_cv_count - 1) * cvs + j];
        double val_other = other_copy.m_cv[j];
        new_cvs.push_back((val_this + val_other) * 0.5);
    }
    
    // Add remaining CVs from other curve (skip first one)
    for (int i = 1; i < other_copy.m_cv_count; i++) {
        for (int j = 0; j < cvs; j++) {
            new_cvs.push_back(other_copy.m_cv[i * cvs + j]);
        }
    }
    
    // Update this curve
    int new_cv_count = m_cv_count + other_copy.m_cv_count - 1;
    m_cv_count = new_cv_count;
    m_knot = new_knots;
    m_cv = new_cvs;
    m_cv_capacity = new_cv_count * cvs;
    
    return true;
}

// Change dimension
bool NurbsCurve::change_dimension(int desired_dimension) {
    if (desired_dimension < 1) return false;
    if (desired_dimension == m_dim) return true;
    
    int new_stride = m_is_rat ? (desired_dimension + 1) : desired_dimension;
    std::vector<double> new_cv(m_cv_count * new_stride, 0.0);
    
    int copy_dim = std::min(m_dim, desired_dimension);
    
    for (int i = 0; i < m_cv_count; i++) {
        const double* old_cv = cv(i);
        double* new_cv_ptr = &new_cv[i * new_stride];
        
        // Copy existing dimensions
        for (int j = 0; j < copy_dim; j++) {
            new_cv_ptr[j] = old_cv[j];
        }
        
        // Copy weight if rational
        if (m_is_rat) {
            new_cv_ptr[desired_dimension] = old_cv[m_dim];
        }
    }
    
    m_dim = desired_dimension;
    m_cv_stride = new_stride;
    m_cv = new_cv;
    m_cv_capacity = m_cv_count * m_cv_stride;
    
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

// Get superfluous knot
double NurbsCurve::superfluous_knot(int end) const {
    if (!is_valid()) return 0.0;
    
    if (end == 0) {
        // Start: return knot before first domain knot
        if (m_order >= 3) {
            return 2.0 * m_knot[m_order-2] - m_knot[m_order-1];
        }
    } else {
        // End: return knot after last domain knot
        int kc = knot_count();
        if (kc >= 2 && m_cv_count >= m_order) {
            return 2.0 * m_knot[m_cv_count-1] - m_knot[m_cv_count-2];
        }
    }
    
    return 0.0;
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

// Get control point spans
std::pair<int, int> NurbsCurve::control_point_spans(int cv_index) const {
    if (cv_index < 0 || cv_index >= m_cv_count) {
        return {0, 0};
    }
    
    int p = degree();
    int span_start = std::max(0, cv_index - p);
    int span_end = std::min(m_cv_count - m_order, cv_index);
    
    return {span_start, span_end};
}

// Get control point support (parameter interval)
std::pair<double, double> NurbsCurve::control_point_support(int cv_index) const {
    if (cv_index < 0 || cv_index >= m_cv_count) {
        return {0.0, 0.0};
    }
    
    // Support is from knot[cv_index] to knot[cv_index + order]
    int ki_start = cv_index;
    int ki_end = cv_index + m_order;
    
    if (ki_start < 0) ki_start = 0;
    if (ki_end >= knot_count()) ki_end = knot_count() - 1;
    
    double t_start = (ki_start < knot_count()) ? m_knot[ki_start] : 0.0;
    double t_end = (ki_end < knot_count()) ? m_knot[ki_end] : 0.0;
    
    // Clamp to domain
    auto [d0, d1] = domain();
    t_start = std::max(t_start, d0);
    t_end = std::min(t_end, d1);
    
    return {t_start, t_end};
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

// Check if arc
bool NurbsCurve::is_arc(Plane* plane, double tolerance) const {
    if (!is_valid()) return false;
    if (m_dim != 2 && m_dim != 3) return false;
    if (m_order < 3) return false;
    
    // First check if it's linear (can't be both line and arc)
    if (is_linear(tolerance)) return false;
    
    // Check if planar
    Plane test_plane;
    if (!is_planar(&test_plane, tolerance)) {
        return false;
    }
    
    // Sample points and check if they're equidistant from a center
    auto [t0, t1] = domain();
    
    // Adaptive sampling based on tolerance
    // Smaller tolerance = more samples for better accuracy
    int num_samples = std::max(50, static_cast<int>(100.0 / (tolerance + 1e-10)));
    num_samples = std::min(num_samples, 1000); // Cap at 1000 samples
    
    double dt = (t1 - t0) / num_samples;
    double total_length = 0.0;
    
    Point prev = point_at(t0);
    for (int i = 1; i <= num_samples; i++) {
        Point curr = point_at(t0 + i * dt);
        total_length += prev.distance(curr);
        prev = curr;
    }
    
    // Find approximate center using first 3 points
    Point p0 = point_at(t0);
    Point p1 = point_at(t0 + dt);
    Point p2 = point_at(t0 + 2 * dt);
    
    // Check if collinear
    Vector v1(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Vector v2(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
    if (v1.cross(v2).magnitude() < tolerance) {
        return false; // Collinear
    }
    
    // Simple arc test: check if all points are approximately same distance from midpoint
    double r0 = p0.distance(p1);
    bool is_arc_candidate = true;
    for (int i = 0; i < num_samples; i++) {
        double ri = point_at(t0 + i * dt).distance(p1);
        if (std::abs(ri - r0) > tolerance * 10.0) {
            is_arc_candidate = false;
            break;
        }
    }
    
    if (is_arc_candidate && plane) {
        *plane = test_plane;
    }
    
    return is_arc_candidate;
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

// Reparameterize rational curve - Based on OpenNURBS ON_ReparameterizeRationalNurbsCurve
// Reference: E. T. Y. Lee and M. L. Lucian, "Mobius reparameterization of rational B-splines", CAGD Vol8 pp 213-215 1991
bool NurbsCurve::reparameterize(double c) {
    if (!std::isfinite(c) || c == 0.0) return false;
    if (c == 1.0) return true;
    
    // Must be rational for this operation
    if (!m_is_rat) {
        if (!make_rational()) return false;
    }
    
    if (!is_valid()) return false;
    
    const double c1 = c - 1.0;
    if (!std::isfinite(c1)) return false;
    
    // Change domain to [0,1]
    double k0 = m_knot[m_order - 2];
    double k1 = m_knot[m_cv_count - 1];
    double d = k1 - k0;
    if (!std::isfinite(d) || d <= 0.0) return false;
    d = 1.0 / d;
    
    int knot_count = m_cv_count + m_order - 2;
    
    // Adjust knots to [0,1] then apply transformation
    for (int i = 0; i < knot_count; i++) {
        double k = m_knot[i];
        k = (k - k0) * d;  // Normalize to [0,1]
        m_knot[i] = c * k / (c1 * k + 1.0);  // Apply Mobius transformation
    }
    
    // Adjust CVs
    std::vector<double> knot_backup(m_knot.begin(), m_knot.begin() + m_cv_count);
    
    for (int i = 0; i < m_cv_count; i++) {
        double* cv_ptr = cv(i);
        
        // Compute product: d = (c - c1*knot[i]) * (c - c1*knot[i+1]) * ... * (c - c1*knot[i+order-2])
        d = c - c1 * knot_backup[i];
        for (int j = 1; j < m_order - 1; j++) {
            if (i + j < static_cast<int>(knot_backup.size())) {
                d *= c - c1 * knot_backup[i + j];
            }
        }
        
        // Scale homogeneous coordinates
        for (int j = 0; j < m_dim; j++) {
            cv_ptr[j] *= d;
        }
        cv_ptr[m_dim] *= d;  // Weight
    }
    
    // Change domain back to [k0, k1]
    for (int i = 0; i < knot_count; i++) {
        double k = m_knot[i];
        m_knot[i] = (1.0 - k) * k0 + k * k1;
    }
    
    return true;
}

// Change end weights - Based on OpenNURBS ON_ChangeRationalNurbsCurveEndWeights
bool NurbsCurve::change_end_weights(double w0, double w1) {
    if (m_cv_count < m_order || m_order < 2) return false;
    if (!std::isfinite(w0) || !std::isfinite(w1)) return false;
    if (w0 == 0.0 || w1 == 0.0) return false;
    if ((w0 < 0.0 && w1 > 0.0) || (w0 > 0.0 && w1 < 0.0)) return false;
    
    // Make rational if needed
    if (!m_is_rat) {
        if (!make_rational()) return false;
    }
    
    // Clamp ends
    if (!clamp_end(2)) return false;
    
    // Get current end weights
    double v0 = weight(0);
    double v1 = weight(m_cv_count - 1);
    
    if (!std::isfinite(v0) || !std::isfinite(v1) || v0 == 0.0 || v1 == 0.0) return false;
    if ((v0 < 0.0 && v1 > 0.0) || (v0 > 0.0 && v1 < 0.0)) return false;
    
    // Compute scaling factors
    double r = w0 / v0;
    double s = w1 / v1;
    
    // If r and s are close enough, use simple uniform scaling
    if (std::abs(r - s) <= std::abs(s) * std::sqrt(std::numeric_limits<double>::epsilon())) {
        if (r != s) {
            s = 0.5 * (r + s);
        }
        r = s;
    }
    
    // Scale to get last weight set to w1
    if (s != 1.0 && v1 != w1) {
        for (int i = 0; i < m_cv_count; i++) {
            double* cv_ptr = cv(i);
            for (int j = 0; j <= m_dim; j++) {
                cv_ptr[j] *= s;
            }
        }
    }
    
    // If r != s, need to reparameterize
    if (r != s) {
        v0 = weight(0);
        v1 = weight(m_cv_count - 1);
        
        if (std::isfinite(v0) && std::isfinite(v1) && v0 != 0.0) {
            // Compute reparameterization factor
            double p = static_cast<double>(m_order - 1);
            r = std::pow(w0 / v0, 1.0 / p);
            
            if (!std::isfinite(r)) return false;
            if (!reparameterize(r)) return false;
        }
    }
    
    // Make sure weights agree exactly
    double* cv0 = cv(0);
    double* cv1 = cv(m_cv_count - 1);
    cv0[m_dim] = w0;
    cv1[m_dim] = w1;
    
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

// Change closed curve seam
bool NurbsCurve::change_closed_curve_seam(double t) {
    if (!is_valid()) return false;
    if (!is_closed()) return false;
    
    auto [t0, t1] = domain();
    if (t <= t0 || t >= t1) return false;
    
    // For periodic curves, this would rotate the control points
    // Simplified implementation: insert full multiplicity knot and reorganize
    if (!is_periodic()) return false;
    
    // Find the span
    int span = find_span(t);
    if (span < 0) return false;
    
    // Insert knot with full multiplicity
    int p = degree();
    if (!insert_knot(t, p)) return false;
    
    // Now reorganize CVs and knots to make t the new seam
    // This is complex - simplified version
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

///////////////////////////////////////////////////////////////////////////////////////////
// Intersection Operations
///////////////////////////////////////////////////////////////////////////////////////////

// Helper function: Evaluate signed distance from point to plane
namespace {
    double signed_distance_to_plane(const Point& pt, const Plane& plane) {
        Vector v(pt[0] - plane.origin()[0],
                pt[1] - plane.origin()[1],
                pt[2] - plane.origin()[2]);
        return v.dot(plane.z_axis());
    }
    
    // Find root in interval using bisection
    bool find_root_bisection(const NurbsCurve& curve, const Plane& plane,
                            double t0, double t1, double tolerance,
                            double& t_result) {
        const int max_iterations = 50;
        double d0 = signed_distance_to_plane(curve.point_at(t0), plane);
        double d1 = signed_distance_to_plane(curve.point_at(t1), plane);
        
        // No sign change, no root in interval
        if (d0 * d1 > 0) return false;
        
        // Bisection method
        for (int iter = 0; iter < max_iterations; iter++) {
            double t_mid = (t0 + t1) * 0.5;
            double d_mid = signed_distance_to_plane(curve.point_at(t_mid), plane);
            
            if (std::abs(d_mid) < tolerance || (t1 - t0) < tolerance) {
                t_result = t_mid;
                return true;
            }
            
            if (d0 * d_mid < 0) {
                t1 = t_mid;
                d1 = d_mid;
            } else {
                t0 = t_mid;
                d0 = d_mid;
            }
        }
        
        t_result = (t0 + t1) * 0.5;
        return std::abs(signed_distance_to_plane(curve.point_at(t_result), plane)) < tolerance * 10.0;
    }
    
    // Newton-Raphson refinement for intersection
    bool refine_intersection_newton(const NurbsCurve& curve, const Plane& plane,
                                   double& t, double tolerance) {
        const int max_iterations = 10;
        const double step_tolerance = tolerance * 0.01;
        
        for (int iter = 0; iter < max_iterations; iter++) {
            Point pt = curve.point_at(t);
            Vector tangent = curve.tangent_at(t);
            
            double f = signed_distance_to_plane(pt, plane);
            double df = tangent.dot(plane.z_axis());
            
            if (std::abs(f) < tolerance) return true;
            if (std::abs(df) < 1e-12) return false;  // Tangent parallel to plane
            
            double dt = -f / df;
            if (std::abs(dt) < step_tolerance) return true;
            
            t += dt;
            
            // Clamp to domain
            auto [t0, t1] = curve.domain();
            if (t < t0) t = t0;
            if (t > t1) t = t1;
        }
        
        return std::abs(signed_distance_to_plane(curve.point_at(t), plane)) < tolerance * 2.0;
    }
}

// Find all curve-plane intersections (parameter values)
std::vector<double> NurbsCurve::intersect_plane(const Plane& plane, double tolerance) const {
    std::vector<double> intersections;
    
    if (!is_valid()) return intersections;
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;
    
    auto [t_start, t_end] = domain();
    
    // Subdivide curve into spans and look for sign changes
    std::vector<double> span_params = get_span_vector();
    
    for (size_t i = 0; i < span_params.size() - 1; i++) {
        double t0 = span_params[i];
        double t1 = span_params[i + 1];
        
        // Skip zero-length spans
        if (std::abs(t1 - t0) < tolerance) continue;
        
        // Check for sign change (intersection) in this span
        double d0 = signed_distance_to_plane(point_at(t0), plane);
        double d1 = signed_distance_to_plane(point_at(t1), plane);
        
        // Check if span crosses plane
        if (d0 * d1 < 0) {
            // Sign change - there's an intersection
            double t_intersection;
            if (find_root_bisection(*this, plane, t0, t1, tolerance, t_intersection)) {
                // Refine with Newton-Raphson
                refine_intersection_newton(*this, plane, t_intersection, tolerance);
                intersections.push_back(t_intersection);
            }
        } else if (std::abs(d0) < tolerance) {
            // Start point is on plane
            bool add = true;
            // Avoid duplicates
            if (!intersections.empty() && std::abs(intersections.back() - t0) < tolerance) {
                add = false;
            }
            if (add) intersections.push_back(t0);
        }
    }
    
    // Check end point
    double d_end = signed_distance_to_plane(point_at(t_end), plane);
    if (std::abs(d_end) < tolerance) {
        bool add = true;
        if (!intersections.empty() && std::abs(intersections.back() - t_end) < tolerance) {
            add = false;
        }
        if (add) intersections.push_back(t_end);
    }
    
    // Further subdivision for high-degree curves or if we might have missed intersections
    if (degree() > 3 && intersections.size() < static_cast<size_t>(degree())) {
        // Sample more points to find potential intersections
        int num_samples = degree() * 4;
        double dt = (t_end - t_start) / num_samples;
        
        for (int i = 0; i < num_samples; i++) {
            double t0 = t_start + i * dt;
            double t1 = t_start + (i + 1) * dt;
            
            double d0 = signed_distance_to_plane(point_at(t0), plane);
            double d1 = signed_distance_to_plane(point_at(t1), plane);
            
            if (d0 * d1 < 0) {
                double t_intersection;
                if (find_root_bisection(*this, plane, t0, t1, tolerance, t_intersection)) {
                    // Check if this is a new intersection
                    bool is_new = true;
                    for (double existing : intersections) {
                        if (std::abs(existing - t_intersection) < tolerance * 2.0) {
                            is_new = false;
                            break;
                        }
                    }
                    if (is_new) {
                        refine_intersection_newton(*this, plane, t_intersection, tolerance);
                        intersections.push_back(t_intersection);
                    }
                }
            }
        }
    }
    
    // Sort intersections
    std::sort(intersections.begin(), intersections.end());
    
    // Remove duplicates
    intersections.erase(
        std::unique(intersections.begin(), intersections.end(),
                   [tolerance](double a, double b) {
                       return std::abs(a - b) < tolerance * 2.0;
                   }),
        intersections.end()
    );
    
    return intersections;
}

// Find all curve-plane intersections (points)
std::vector<Point> NurbsCurve::intersect_plane_points(const Plane& plane, double tolerance) const {
    std::vector<double> params = intersect_plane(plane, tolerance);
    std::vector<Point> points;
    points.reserve(params.size());
    
    for (double t : params) {
        points.push_back(point_at(t));
    }
    
    return points;
}

// Get closest parameter on curve to a point (improved version)
std::pair<double, double> NurbsCurve::closest_point_to(const Point& test_point, 
                                                        double t0, double t1) const {
    if (!is_valid()) return {0.0, std::numeric_limits<double>::infinity()};
    
    auto [domain_start, domain_end] = domain();
    if (t0 < 0.0) t0 = domain_start;
    if (t1 < 0.0) t1 = domain_end;
    
    t0 = std::max(t0, domain_start);
    t1 = std::min(t1, domain_end);
    
    // Multi-start optimization to avoid local minima
    const int num_samples = std::max(10, degree() * 2);
    double dt = (t1 - t0) / num_samples;
    
    double best_t = t0;
    double best_dist = point_at(t0).distance(test_point);
    
    // Sample curve to find good starting points
    for (int i = 0; i <= num_samples; i++) {
        double t = t0 + i * dt;
        double dist = point_at(t).distance(test_point);
        if (dist < best_dist) {
            best_dist = dist;
            best_t = t;
        }
    }
    
    // Newton-Raphson refinement from best sample
    const int max_iterations = 20;
    const double step_tolerance = (t1 - t0) * 1e-10;
    
    double t = best_t;
    
    for (int iter = 0; iter < max_iterations; iter++) {
        Point pt = point_at(t);
        Vector tangent = tangent_at(t);
        
        Vector delta(test_point[0] - pt[0],
                    test_point[1] - pt[1],
                    test_point[2] - pt[2]);
        
        // f(t) = (C(t) - P) · C'(t) should be zero at closest point
        double f = -delta.dot(tangent);
        
        if (std::abs(f) < step_tolerance) break;
        
        // f'(t) = C''(t) · (C(t) - P) - |C'(t)|^2
        auto derivs = evaluate(t, 2);
        if (derivs.size() < 3) break;
        
        Vector d2(derivs[2][0], derivs[2][1], derivs[2][2]);
        double tangent_mag = tangent.magnitude();
        double df = delta.dot(d2) - tangent_mag * tangent_mag;
        
        if (std::abs(df) < 1e-12) break;
        
        double dt_step = -f / df;
        
        // Limit step size
        if (std::abs(dt_step) > (t1 - t0) * 0.5) {
            dt_step = std::copysign((t1 - t0) * 0.5, dt_step);
        }
        
        t += dt_step;
        
        // Clamp to interval
        if (t < t0) t = t0;
        if (t > t1) t = t1;
        
        if (std::abs(dt_step) < step_tolerance) break;
    }
    
    double final_dist = point_at(t).distance(test_point);
    
    // Also check endpoints
    double dist_start = point_at(t0).distance(test_point);
    double dist_end = point_at(t1).distance(test_point);
    
    if (dist_start < final_dist) {
        t = t0;
        final_dist = dist_start;
    }
    if (dist_end < final_dist) {
        t = t1;
        final_dist = dist_end;
    }
    
    return {t, final_dist};
}

// Bézier clipping for curve-plane intersection
std::vector<double> NurbsCurve::intersect_plane_bezier_clipping(const Plane& plane, double tolerance) const {
    std::vector<double> results;
    
    if (!is_valid()) return results;
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;
    
    auto [t0, t1] = domain();
    
    // Helper: compute signed distance from point to plane
    auto signed_distance = [&](const Point& p) -> double {
        Vector v(p[0] - plane.origin()[0],
                p[1] - plane.origin()[1],
                p[2] - plane.origin()[2]);
        return v.dot(plane.z_axis());
    };
    
    // Helper: Bézier clipping on interval [ta, tb]
    std::function<void(double, double, int)> clip_recursive;
    clip_recursive = [&](double ta, double tb, int depth) {
        // Prevent infinite recursion
        if (depth > 50) {
            // Fall back to Newton-Raphson
            double tm = (ta + tb) * 0.5;
            Point pm = point_at(tm);
            double dist = signed_distance(pm);
            
            if (std::abs(dist) < tolerance) {
                results.push_back(tm);
            }
            return;
        }
        
        // Check if interval is small enough
        if (std::abs(tb - ta) < tolerance * 0.01) {
            double tm = (ta + tb) * 0.5;
            Point pm = point_at(tm);
            double dist = signed_distance(pm);
            
            if (std::abs(dist) < tolerance) {
                // Newton refinement
                double t = tm;
                for (int iter = 0; iter < 10; iter++) {
                    Point pt = point_at(t);
                    Vector tangent = tangent_at(t);
                    
                    double f = signed_distance(pt);
                    double df = tangent.dot(plane.z_axis());
                    
                    if (std::abs(df) < 1e-12) break;
                    
                    double dt = -f / df;
                    t += dt;
                    
                    if (std::abs(dt) < tolerance * 0.01) break;
                    if (t < ta || t > tb) {
                        t = tm;
                        break;
                    }
                }
                
                // Verify solution
                Point pt_final = point_at(t);
                if (std::abs(signed_distance(pt_final)) < tolerance && t >= ta && t <= tb) {
                    results.push_back(t);
                }
            }
            return;
        }
        
        // Sample control points (use order+1 points for Bézier-like behavior)
        int num_samples = std::min(order() + 1, 10);
        std::vector<double> distances;
        std::vector<double> params;
        
        double dt = (tb - ta) / (num_samples - 1);
        for (int i = 0; i < num_samples; i++) {
            double t = ta + i * dt;
            Point p = point_at(t);
            distances.push_back(signed_distance(p));
            params.push_back(t);
        }
        
        // Find min and max distances
        double d_min = *std::min_element(distances.begin(), distances.end());
        double d_max = *std::max_element(distances.begin(), distances.end());
        
        // Check if curve segment is entirely on one side
        if (d_min > tolerance || d_max < -tolerance) {
            return; // No intersection in this interval
        }
        
        // If curve crosses plane, find clipping bounds using convex hull
        double t_min = ta;
        double t_max = tb;
        
        // Simple clipping: find where distance function changes sign
        for (size_t i = 0; i < distances.size() - 1; i++) {
            if (distances[i] * distances[i + 1] < 0) {
                // Sign change between params[i] and params[i+1]
                // Use linear interpolation to clip
                double d0 = distances[i];
                double d1 = distances[i + 1];
                double t_clip = params[i] - d0 * (params[i + 1] - params[i]) / (d1 - d0);
                
                if (d0 > 0) {
                    t_max = std::min(t_max, t_clip + (tb - ta) * 0.1);
                } else {
                    t_min = std::max(t_min, t_clip - (tb - ta) * 0.1);
                }
            }
        }
        
        // Ensure valid interval
        if (t_min >= t_max) {
            t_min = ta;
            t_max = tb;
        }
        
        // Clamp to original interval
        t_min = std::max(ta, t_min);
        t_max = std::min(tb, t_max);
        
        // Check if clipping reduced interval significantly
        double reduction = (t_max - t_min) / (tb - ta);
        
        if (reduction > 0.8 || (t_max - t_min) < tolerance * 0.1) {
            // Not much reduction, split in half
            double tm = (ta + tb) * 0.5;
            clip_recursive(ta, tm, depth + 1);
            clip_recursive(tm, tb, depth + 1);
        } else {
            // Good reduction, continue on clipped interval
            clip_recursive(t_min, t_max, depth + 1);
        }
    };
    
    // Start recursive clipping
    clip_recursive(t0, t1, 0);
    
    // Sort results
    std::sort(results.begin(), results.end());
    
    // Remove duplicates
    auto last = std::unique(results.begin(), results.end(),
                           [tolerance](double a, double b) {
                               return std::abs(a - b) < tolerance * 2.0;
                           });
    results.erase(last, results.end());
    
    return results;
}

// Convert span to Bezier - Extract Bezier segment from span
bool NurbsCurve::convert_span_to_bezier(int span_index, std::vector<Point>& bezier_cvs) const {
    bezier_cvs.clear();
    
    if (!is_valid()) return false;
    if (span_index < 0 || span_index > m_cv_count - m_order) return false;
    
    // Check if span is non-empty
    int ki0 = span_index + m_order - 2;
    int ki1 = span_index + m_order - 1;
    if (ki0 >= knot_count() || ki1 >= knot_count()) return false;
    if (m_knot[ki0] >= m_knot[ki1]) return false;  // Empty span
    
    // For a proper Bezier extraction, we need the span to have full multiplicity knots
    // Simple implementation: just return the order CVs that define this span
    bezier_cvs.reserve(m_order);
    for (int i = 0; i < m_order; i++) {
        bezier_cvs.push_back(get_cv(span_index + i));
    }
    
    return true;
}

// Remove span from curve
bool NurbsCurve::remove_span(int span_index) {
    if (!is_valid()) return false;
    if (span_index < 0 || span_index > m_cv_count - m_order) return false;
    
    // Need at least 2 spans
    if (span_count() < 2) return false;
    
    // Check if span is non-empty
    int ki0 = span_index + m_order - 2;
    int ki1 = span_index + m_order - 1;
    if (ki0 >= knot_count() || ki1 >= knot_count()) return false;
    if (m_knot[ki0] >= m_knot[ki1]) return false;
    
    // Get multiplicities
    int m0 = knot_multiplicity(ki0);
    int m1 = knot_multiplicity(ki1);
    
    // Calculate how many CVs to remove
    int cvs_to_remove = m_order - (m0 + m1);
    if (cvs_to_remove <= 0) return false;
    
    // Remove knots
    int knots_to_remove = m_order;
    m_knot.erase(m_knot.begin() + ki0, m_knot.begin() + ki0 + knots_to_remove);
    
    // Remove CVs
    int cv_start = span_index;
    for (int i = 0; i < cvs_to_remove; i++) {
        int offset = cv_start * m_cv_stride;
        m_cv.erase(m_cv.begin() + offset, m_cv.begin() + offset + m_cv_stride);
    }
    
    m_cv_count -= cvs_to_remove;
    
    return is_valid();
}

// Remove all singular spans
int NurbsCurve::remove_singular_spans() {
    if (!is_valid()) return 0;
    
    int removed_count = 0;
    int span_cnt = span_count();
    
    // Iterate backwards to avoid index shifting issues
    for (int i = span_cnt - 1; i >= 0; i--) {
        if (span_is_singular(i)) {
            if (remove_span(i)) {
                removed_count++;
            }
        }
    }
    
    return removed_count;
}

// Get cubic Bezier approximation of entire curve
double NurbsCurve::get_cubic_bezier_approximation(double max_deviation, std::vector<Point>& bezier_cvs) const {
    bezier_cvs.clear();
    
    if (!is_valid()) return std::numeric_limits<double>::quiet_NaN();
    if (m_cv_count < 2) return std::numeric_limits<double>::quiet_NaN();
    
    // Get Greville abcissae for sampling
    std::vector<double> greville;
    if (!get_greville_abcissae(greville)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // For simple approximation: fit cubic through start, end, and weighted interior
    bezier_cvs.resize(4);
    
    // Get domain
    auto [t0, t1] = domain();
    
    // Start and end points
    bezier_cvs[0] = point_at(t0);
    bezier_cvs[3] = point_at(t1);
    
    // Tangents at ends for control point estimation
    Vector tan0 = tangent_at(t0);
    Vector tan1 = tangent_at(t1);
    
    // Estimate interior control points from end tangents
    double arc_length = length();
    double scale = arc_length / 3.0;
    
    bezier_cvs[1] = Point(
        bezier_cvs[0][0] + tan0[0] * scale,
        bezier_cvs[0][1] + tan0[1] * scale,
        bezier_cvs[0][2] + tan0[2] * scale
    );
    
    bezier_cvs[2] = Point(
        bezier_cvs[3][0] - tan1[0] * scale,
        bezier_cvs[3][1] - tan1[1] * scale,
        bezier_cvs[3][2] - tan1[2] * scale
    );
    
    // Calculate maximum deviation at Greville points
    double max_dev = 0.0;
    for (double t : greville) {
        Point pt_nurbs = point_at(t);
        
        // Evaluate cubic Bezier at same parameter (normalized)
        double u = (t - t0) / (t1 - t0);
        double u2 = u * u;
        double u3 = u2 * u;
        double omu = 1.0 - u;
        double omu2 = omu * omu;
        double omu3 = omu2 * omu;
        
        Point pt_bezier(
            omu3 * bezier_cvs[0][0] + 3.0 * omu2 * u * bezier_cvs[1][0] +
            3.0 * omu * u2 * bezier_cvs[2][0] + u3 * bezier_cvs[3][0],
            
            omu3 * bezier_cvs[0][1] + 3.0 * omu2 * u * bezier_cvs[1][1] +
            3.0 * omu * u2 * bezier_cvs[2][1] + u3 * bezier_cvs[3][1],
            
            omu3 * bezier_cvs[0][2] + 3.0 * omu2 * u * bezier_cvs[1][2] +
            3.0 * omu * u2 * bezier_cvs[2][2] + u3 * bezier_cvs[3][2]
        );
        
        double dev = pt_nurbs.distance(pt_bezier);
        if (dev > max_dev) max_dev = dev;
    }
    
    // Check if deviation is acceptable
    if (max_deviation >= 0.0 && max_dev > max_deviation) {
        bezier_cvs.clear();
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    return max_dev;
}

// Get NURBS form (for NURBS curve, just copy self)
int NurbsCurve::get_nurbs_form(NurbsCurve& nurbs_form, double tolerance) const {
    (void)tolerance;  // Not used for NURBS curve
    
    if (!is_valid()) return 0;
    
    // For a NURBS curve, the NURBS form is itself
    nurbs_form.deep_copy_from(*this);
    
    return 1;  // Perfect parameterization match
}

// Check if has NURBS form (always true for NURBS curve)
int NurbsCurve::has_nurbs_form() const {
    return is_valid() ? 1 : 0;
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
    if (angle_tolerance <= 0.0) angle_tolerance = 0.1;  // ~5.7 degrees
    
    auto [t0, t1] = domain();
    double curve_len = length();
    
    // Set reasonable defaults for edge lengths if not specified
    if (max_edge_length <= 0.0) {
        max_edge_length = curve_len / 10.0;  // At least 10 segments
    }
    if (min_edge_length <= 0.0) {
        min_edge_length = curve_len / 1000.0;  // Max 1000 segments
    }
    
    // Ensure min < max
    if (min_edge_length > max_edge_length) {
        min_edge_length = max_edge_length * 0.1;
    }
    
    // Start with endpoints
    points.push_back(point_at(t0));
    if (params) params->push_back(t0);
    
    // Recursive subdivision function
    std::function<void(double, double, const Point&, const Point&, const Vector&, const Vector&)> subdivide;
    subdivide = [&](double ta, double tb, const Point& pa, const Point& pb,
                    const Vector& va, const Vector& vb) {
        // Check if segment is good enough
        double chord_length = pa.distance(pb);
        
        // Don't subdivide if too small
        if (chord_length < min_edge_length) return;
        
        // Check angle between tangents
        double cos_angle = va.dot(vb) / (va.magnitude() * vb.magnitude());
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));  // Clamp
        double angle = std::acos(cos_angle);
        
        // Subdivide if angle too large or edge too long
        bool need_subdivide = (angle > angle_tolerance) || (chord_length > max_edge_length);
        
        if (!need_subdivide) {
            // Accept this segment
            return;
        }
        
        // Subdivide at midpoint
        double tm = (ta + tb) * 0.5;
        Point pm = point_at(tm);
        Vector vm = tangent_at(tm);
        
        // Recurse on both halves
        subdivide(ta, tm, pa, pm, va, vm);
        
        // Add midpoint
        points.push_back(pm);
        if (params) params->push_back(tm);
        
        subdivide(tm, tb, pm, pb, vm, vb);
    };
    
    // Start recursive subdivision on entire curve
    Point p0 = point_at(t0);
    Point p1 = point_at(t1);
    Vector v0 = tangent_at(t0);
    Vector v1 = tangent_at(t1);
    
    subdivide(t0, t1, p0, p1, v0, v1);
    
    // Add endpoint
    points.push_back(point_at(t1));
    if (params) params->push_back(t1);
    
    return points.size() >= 2;
}

// Divide curve into uniform number of points (simple)
bool NurbsCurve::divide_by_count(int count, std::vector<Point>& points,
                                std::vector<double>* params,
                                bool include_endpoints) const {
    points.clear();
    if (params) params->clear();
    
    if (!is_valid()) return false;
    if (count < 2) return false;
    
    auto [t0, t1] = domain();
    double range = t1 - t0;
    
    if (include_endpoints) {
        // Divide into count points including endpoints
        // This gives (count-1) equal segments
        for (int i = 0; i < count; i++) {
            double t = t0 + (range * i) / (count - 1);
            points.push_back(point_at(t));
            if (params) params->push_back(t);
        }
    } else {
        // Divide into count interior points
        // This gives (count+1) equal segments
        for (int i = 0; i < count; i++) {
            double t = t0 + (range * (i + 1)) / (count + 1);
            points.push_back(point_at(t));
            if (params) params->push_back(t);
        }
    }
    
    return true;
}

// Divide curve by approximate arc length
bool NurbsCurve::divide_by_length(double segment_length, std::vector<Point>& points,
                                 std::vector<double>* params) const {
    points.clear();
    if (params) params->clear();
    
    if (!is_valid()) return false;
    if (segment_length <= 0.0) return false;
    
    double curve_len = length();
    int approx_count = static_cast<int>(std::ceil(curve_len / segment_length)) + 1;
    
    // Use adaptive approach to get approximately equal arc lengths
    auto [t0, t1] = domain();
    
    points.push_back(point_at(t0));
    if (params) params->push_back(t0);
    
    double accumulated_length = 0.0;
    double target_length = segment_length;
    Point p_current = point_at(t0);
    
    // Sample densely and accumulate arc length
    int num_samples = std::max(100, approx_count * 10);
    double dt = (t1 - t0) / num_samples;
    
    for (int i = 1; i <= num_samples; i++) {
        double t_next = t0 + i * dt;
        Point p_next = point_at(t_next);
        double seg_len = p_current.distance(p_next);
        
        accumulated_length += seg_len;
        
        // Check if we've reached target length
        if (accumulated_length >= target_length) {
            points.push_back(p_next);
            if (params) params->push_back(t_next);
            
            accumulated_length = 0.0;
            target_length = segment_length;
        }
        
        p_current = p_next;
    }
    
    // Always add endpoint if not already there
    if (points.back().distance(point_at(t1)) > segment_length * 0.1) {
        points.push_back(point_at(t1));
        if (params) params->push_back(t1);
    }
    
    return points.size() >= 2;
}

// Curve-plane intersection using algebraic/hodograph method
std::vector<double> NurbsCurve::intersect_plane_algebraic(const Plane& plane, double tolerance) const {
    if (!is_valid()) return {};
    
    std::vector<double> results;
    
    // Get all spans
    std::vector<double> spans = get_span_vector();
    if (spans.size() < 2) return {};
    
    // Process each span separately
    for (size_t i = 0; i < spans.size() - 1; i++) {
        double span_t0 = spans[i];
        double span_t1 = spans[i + 1];
        
        // Skip degenerate spans
        if (std::abs(span_t1 - span_t0) < tolerance) continue;
        
        // Recursive subdivision helper
        std::function<void(double, double, int)> subdivide = [&](double a, double b, int depth) {
            if (depth > 30) return; // Max recursion depth
            
            // Evaluate at endpoints
            Point p_a = point_at(a);
            Point p_b = point_at(b);
            
            // Signed distances to plane (using z_axis as normal)
            Vector normal = plane.z_axis();
            double f_a = normal.dot(p_a - plane.origin());
            double f_b = normal.dot(p_b - plane.origin());
            
            // No sign change → no root (or even number)
            if (f_a * f_b > 0) return;
            
            // Check if segment is nearly linear
            double mid_t = (a + b) * 0.5;
            Point p_mid = point_at(mid_t);
            
            // Distance from midpoint to line connecting endpoints
            Vector line_dir = p_b - p_a;
            double line_len = line_dir.magnitude();
            if (line_len > 1e-14) {
                line_dir = line_dir / line_len; // Normalize
            }
            double deviation = std::abs((p_mid - p_a).cross(line_dir).magnitude());
            
            if (deviation < tolerance * 10.0 || (b - a) < tolerance * 10.0) {
                // Nearly linear - apply Newton-Raphson
                double t = mid_t;
                bool converged = false;
                
                for (int iter = 0; iter < 10; iter++) {
                    Point p = point_at(t);
                    double f = normal.dot(p - plane.origin());
                    
                    if (std::abs(f) < tolerance) {
                        converged = true;
                        break;
                    }
                    
                    // Derivative: df/dt = normal · C'(t)
                    Vector tangent = tangent_at(t);
                    double df = normal.dot(tangent);
                    
                    if (std::abs(df) < 1e-14) {
                        // Tangent to plane - use bisection
                        t = (a + b) * 0.5;
                        break;
                    }
                    
                    // Newton step
                    double t_new = t - f / df;
                    
                    // Keep in bounds
                    if (t_new < a || t_new > b) {
                        t_new = (a + b) * 0.5; // Fallback to bisection
                    }
                    
                    if (std::abs(t_new - t) < tolerance) {
                        t = t_new;
                        converged = true;
                        break;
                    }
                    
                    t = t_new;
                }
                
                if (converged && t >= a && t <= b) {
                    // Check for duplicates
                    bool is_duplicate = false;
                    for (double existing : results) {
                        if (std::abs(existing - t) < tolerance * 10.0) {
                            is_duplicate = true;
                            break;
                        }
                    }
                    if (!is_duplicate) {
                        results.push_back(t);
                    }
                }
            } else {
                // Subdivide further
                subdivide(a, mid_t, depth + 1);
                subdivide(mid_t, b, depth + 1);
            }
        };
        
        // Start subdivision for this span
        subdivide(span_t0, span_t1, 0);
    }
    
    // Sort and remove duplicates
    std::sort(results.begin(), results.end());
    results.erase(std::unique(results.begin(), results.end(),
                              [tolerance](double a, double b) {
                                  return std::abs(a - b) < tolerance * 10.0;
                              }), results.end());
    
    return results;
}

// Curve-plane intersection using production CAD kernel method (INDUSTRY STANDARD)
std::vector<double> NurbsCurve::intersect_plane_production(const Plane& plane, double tolerance) const {
    if (!is_valid()) return {};
    
    std::vector<double> results;
    
    // Get all spans
    std::vector<double> spans = get_span_vector();
    if (spans.size() < 2) return {};
    
    // Helper: Check if segment is nearly linear
    auto is_nearly_linear = [this, tolerance](double a, double b) -> bool {
        Point p_a = point_at(a);
        Point p_b = point_at(b);
        Point p_mid = point_at((a + b) * 0.5);
        
        // Distance from midpoint to line connecting endpoints
        Vector ab = p_b - p_a;
        double line_length = ab.magnitude();
        if (line_length < 1e-14) return true;
        
        Vector am = p_mid - p_a;
        double cross_mag = ab.cross(am).magnitude();
        double deviation = cross_mag / line_length;
        
        return deviation < tolerance * 10.0;
    };
    
    // Recursive subdivision with Newton refinement
    std::function<void(double, double, int)> subdivide = [&](double a, double b, int depth) {
        if (depth > 30) return; // Max recursion depth
        
        // Evaluate at endpoints
        Point p_a = point_at(a);
        Point p_b = point_at(b);
        
        // Signed distances to plane (using z_axis as normal)
        Vector normal = plane.z_axis();
        double f_a = normal.dot(p_a - plane.origin());
        double f_b = normal.dot(p_b - plane.origin());
        
        // No sign change → no root (or even number of roots)
        if (f_a * f_b > 0) return;
        
        // Check if nearly linear
        if (is_nearly_linear(a, b) || (b - a) < tolerance * 10.0) {
            // Apply Newton-Raphson with robust bracketing
            double t = (a + b) * 0.5;
            bool converged = false;
            
            for (int iter = 0; iter < 10; iter++) {
                Point p = point_at(t);
                double f = normal.dot(p - plane.origin());
                
                if (std::abs(f) < tolerance) {
                    converged = true;
                    break;
                }
                
                // Derivative
                Vector tangent = tangent_at(t);
                double df = normal.dot(tangent);
                
                if (std::abs(df) < 1e-14) {
                    // Nearly tangent - use bisection
                    if (f * f_a < 0) {
                        b = t;
                        f_b = f;
                    } else {
                        a = t;
                        f_a = f;
                    }
                    t = (a + b) * 0.5;
                    continue;
                }
                
                // Newton step
                double t_new = t - f / df;
                
                // Keep in bounds [a, b]
                if (t_new < a || t_new > b) {
                    t_new = (a + b) * 0.5; // Fallback to bisection
                }
                
                if (std::abs(t_new - t) < tolerance) {
                    t = t_new;
                    converged = true;
                    break;
                }
                
                t = t_new;
            }
            
            if (converged && t >= a && t <= b) {
                // Check for duplicates
                bool is_duplicate = false;
                for (double existing : results) {
                    if (std::abs(existing - t) < tolerance * 10.0) {
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate) {
                    results.push_back(t);
                }
            }
        } else {
            // Subdivide at midpoint
            double mid = (a + b) * 0.5;
            subdivide(a, mid, depth + 1);
            subdivide(mid, b, depth + 1);
        }
    };
    
    // Process each span
    for (size_t i = 0; i < spans.size() - 1; i++) {
        double span_t0 = spans[i];
        double span_t1 = spans[i + 1];
        
        // Skip degenerate spans
        if (std::abs(span_t1 - span_t0) < tolerance) continue;
        
        subdivide(span_t0, span_t1, 0);
    }
    
    // Sort and remove duplicates
    std::sort(results.begin(), results.end());
    results.erase(std::unique(results.begin(), results.end(),
                              [tolerance](double a, double b) {
                                  return std::abs(a - b) < tolerance * 10.0;
                              }), results.end());
    
    return results;
}

} // namespace session_cpp
