#include "nurbscurve.h"
#include "closest.h"
#include "nurbscurve.pb.h"
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace session_cpp {

constexpr double SQRT_EPSILON = 1.490116119385e-8;

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
NurbsCurve::NurbsCurve() { initialize(); }

NurbsCurve::NurbsCurve(int dimension, bool is_rational, int order, int cv_count) {

    initialize();
    create(dimension, is_rational, order, cv_count);
}

NurbsCurve::NurbsCurve(const NurbsCurve& other) {

    initialize();
    deep_copy_from(other);
}

NurbsCurve& NurbsCurve::operator=(const NurbsCurve& other) {

    if (this != &other)
        deep_copy_from(other);

    return *this;
}

NurbsCurve::~NurbsCurve() { destroy(); }

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
NurbsCurve NurbsCurve::create(bool periodic, int degree, const std::vector<Point>& points, int dimension, double nurbsknot_delta) {

    NurbsCurve curve;
    const int order = degree + 1;

    if (periodic)
        curve.create_periodic_uniform(dimension, order, points, nurbsknot_delta);
    else
        curve.create_clamped_uniform(dimension, order, points, nurbsknot_delta);

    if (!curve.is_valid())
        return curve;

    double length = 0.0;

    if (degree == 1) {
        const int np = static_cast<int>(points.size());

        for (int i = 1; i < np; ++i)
            length += points[i - 1].distance(points[i]);

        if (periodic && np > 1)
            length += points[np - 1].distance(points[0]);
    } else {
        length = curve.length();
    }

    if (length > 0.0)
        curve.set_domain(0.0, length);

    return curve;
}

NurbsCurve NurbsCurve::create_interpolated(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization, CurveInterpStyle end_condition) {

    const int n = static_cast<int>(points.size());

    if (n < 2)
        return NurbsCurve();

    const bool periodic = (parameterization == CurveNurbsKnotStyle::UniformPeriodic || parameterization == CurveNurbsKnotStyle::ChordPeriodic || parameterization == CurveNurbsKnotStyle::ChordSquareRootPeriodic);

    if (periodic && n < 3)
        return NurbsCurve();

    if (n == 2 && !periodic)
        return NurbsCurve::create(false, 1, points);

    if (periodic)
        return create_interpolated_periodic(points, parameterization);

    return create_interpolated_clamped(points, parameterization, end_condition);
}

NurbsCurve NurbsCurve::create_from_parameters(const std::vector<Point>& points, const std::vector<double>& weights, const std::vector<double>& knots, const std::vector<int>& mults, int degree, bool periodic) {

    const int n = static_cast<int>(points.size());
    const int order = degree + 1;

    if (n < order)
        return NurbsCurve();

    if (static_cast<int>(weights.size()) != n)
        return NurbsCurve();

    if (knots.size() != mults.size() || knots.empty())
        return NurbsCurve();

    if (periodic)
        return NurbsCurve();

    bool rational = false;

    for (double w : weights)
        if (std::abs(w - 1.0) > Tolerance::ZERO_TOLERANCE)
            rational = true;

    std::vector<double> full;

    for (size_t i = 0; i < knots.size(); i++)
        for (int m = 0; m < mults[i]; m++)
            full.push_back(knots[i]);

    const int kc = order + n - 2;

    if (static_cast<int>(full.size()) != kc + 2)
        return NurbsCurve();

    NurbsCurve curve;

    if (!curve.create(3, rational, order, n))
        return NurbsCurve();

    for (int i = 0; i < kc; i++)
        curve.set_nurbsknot(i, full[i + 1]);

    for (int i = 0; i < n; i++) {
        if (rational) {
            const double w = weights[i];
            curve.set_cv_4d(i, points[i][0] * w, points[i][1] * w, points[i][2] * w, w);
        } else {
            curve.set_cv(i, points[i]);
        }
    }

    return curve;
}

NurbsCurve NurbsCurve::create_fitted(const std::vector<Point>& points, int num_cvs, int degree, bool is_periodic) {

    if (is_periodic)
        return create_fitted_periodic(points, num_cvs, degree);

    return create_fitted_clamped(points, num_cvs, degree);
}

std::vector<NurbsCurve> NurbsCurve::join(const std::vector<NurbsCurve>& curves, double tolerance) {

    std::vector<NurbsCurve> segs;

    for (const NurbsCurve& c : curves)
        if (c.is_valid())
            segs.push_back(c);

    promote_to_3d(segs);

    std::vector<std::vector<NurbsCurve>> chains = chain_segments(segs, tolerance);
    std::vector<NurbsCurve> result;

    for (std::vector<NurbsCurve>& chain : chains)
        join_chain(chain, result);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsCurve::operator==(const NurbsCurve& other) const {

    if (m_dim != other.m_dim || m_is_rat != other.m_is_rat)
        return false;

    if (m_order != other.m_order || m_cv_count != other.m_cv_count)
        return false;

    if (m_cv_stride != other.m_cv_stride)
        return false;

    if (name != other.name)
        return false;

    if (std::abs(width - other.width) > Tolerance::ZERO_TOLERANCE)
        return false;

    if (pointcolors != other.pointcolors)
        return false;

    if (linecolors != other.linecolors)
        return false;

    if (m_nurbsknot.size() != other.m_nurbsknot.size())
        return false;

    for (size_t i = 0; i < m_nurbsknot.size(); i++)
        if (std::abs(m_nurbsknot[i] - other.m_nurbsknot[i]) > Tolerance::ZERO_TOLERANCE)
            return false;

    if (m_cv.size() != other.m_cv.size())
        return false;

    for (size_t i = 0; i < m_cv.size(); i++)
        if (std::abs(m_cv[i] - other.m_cv[i]) > Tolerance::ZERO_TOLERANCE)
            return false;

    return true;
}

bool NurbsCurve::operator!=(const NurbsCurve& other) const { return !(*this == other); }

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsCurve::transform(const Xform& xform) {

    for (int i = 0; i < m_cv_count; i++) {
        const Point p = get_cv(i);
        const double x = xform.m[0] * p[0] + xform.m[4] * p[1] + xform.m[8] * p[2] + xform.m[12];
        const double y = xform.m[1] * p[0] + xform.m[5] * p[1] + xform.m[9] * p[2] + xform.m[13];
        const double z = xform.m[2] * p[0] + xform.m[6] * p[1] + xform.m[10] * p[2] + xform.m[14];

        if (m_is_rat) {
            const double w = weight(i);
            set_cv_4d(i, x * w, y * w, z * w, w);
        } else {
            set_cv(i, Point(x, y, z));
        }
    }

    return true;
}

NurbsCurve NurbsCurve::transformed(const Xform& xform) const {

    NurbsCurve result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════
void NurbsCurve::initialize() {

    m_dim = 0;
    m_is_rat = 0;
    m_order = 0;
    m_cv_count = 0;
    m_cv_stride = 0;
    m_nurbsknot.clear();
    m_cv.clear();
}

bool NurbsCurve::create(int dimension, bool is_rational, int order, int cv_count) {

    if (dimension < 1 || order < 2 || cv_count < order)
        return false;

    destroy();
    m_dim = dimension;
    m_is_rat = is_rational ? 1 : 0;
    m_order = order;
    m_cv_count = cv_count;
    m_cv_stride = is_rational ? (dimension + 1) : dimension;
    m_nurbsknot.resize(m_order + m_cv_count - 2, 0.0);
    m_cv.resize(m_cv_count * m_cv_stride, 0.0);

    return true;
}

bool NurbsCurve::create_clamped_uniform(int dimension, int order, const std::vector<Point>& points, double nurbsknot_delta) {

    const int point_count = static_cast<int>(points.size());

    if (!create(dimension, false, order, point_count))
        return false;

    for (int i = 0; i < point_count; i++)
        set_cv(i, points[i]);

    const int kc = m_order + m_cv_count - 2;
    double k = 0.0;

    for (int i = m_order - 2; i < m_cv_count; i++, k += nurbsknot_delta)
        m_nurbsknot[i] = k;

    int i0 = m_order - 2;

    for (int i = 0; i < i0; i++)
        m_nurbsknot[i] = m_nurbsknot[i0];

    i0 = m_cv_count - 1;

    for (int i = i0 + 1; i < kc; i++)
        m_nurbsknot[i] = m_nurbsknot[i0];

    return true;
}

bool NurbsCurve::create_periodic_uniform(int dimension, int order, const std::vector<Point>& points, double nurbsknot_delta) {

    const int point_count = static_cast<int>(points.size());

    if (!create(dimension, false, order, point_count + order - 1))
        return false;

    for (int i = 0; i < point_count; i++)
        set_cv(i, points[i]);

    for (int i = 0; i < order - 1; i++)
        set_cv(point_count + i, points[i]);

    const int kc = m_order + m_cv_count - 2;

    for (int i = 0; i < kc; i++)
        m_nurbsknot[i] = (i - m_order + 1) * nurbsknot_delta;

    return true;
}

void NurbsCurve::destroy() {

    m_nurbsknot.clear();
    m_cv.clear();
    initialize();
}

// ═══════════════════════════════════════════════════════════════════════════
// Boolean queries
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsCurve::is_valid() const {

    if (m_dim <= 0)
        return false;

    if (m_order < 2)
        return false;

    if (m_cv_count < m_order)
        return false;

    if (m_cv_stride < cv_size())
        return false;

    if (m_cv.empty() || m_nurbsknot.empty())
        return false;

    if (static_cast<int>(m_cv.size()) < (m_cv_count - 1) * m_cv_stride + cv_size())
        return false;

    if (!is_valid_nurbsknot_vector())
        return false;

    for (size_t i = 0; i < m_cv.size(); i++)
        if (!std::isfinite(m_cv[i]))
            return false;

    return true;
}

bool NurbsCurve::is_closed() const {

    if (!is_valid())
        return false;

    return point_at_start().distance(point_at_end()) < Tolerance::ZERO_TOLERANCE;
}

bool NurbsCurve::is_periodic() const {

    if (m_order < 2)
        return false;

    const int deg = degree();

    for (int i = 0; i < deg; i++)
        if (get_cv(i).distance(get_cv(m_cv_count - deg + i)) > Tolerance::ZERO_TOLERANCE)
            return false;

    const int kc = nurbsknot_count();

    if (kc < 2)
        return false;

    const double delta = m_nurbsknot[m_order - 1] - m_nurbsknot[m_order - 2];

    if (delta < Tolerance::ZERO_TOLERANCE)
        return false;

    for (int i = 1; i < kc; i++)
        if (std::abs((m_nurbsknot[i] - m_nurbsknot[i - 1]) - delta) > Tolerance::ZERO_TOLERANCE)
            return false;

    return true;
}

bool NurbsCurve::is_linear(double tolerance) const {

    if (!is_valid() || m_cv_count < 2)
        return false;

    const Point p0 = get_cv(0);
    const Point p1 = get_cv(m_cv_count - 1);
    const Vector line_vec = p1 - p0;
    const double line_length = line_vec.magnitude();

    if (line_length < tolerance)
        return true;

    for (int i = 1; i < m_cv_count - 1; i++) {
        const Point p = get_cv(i);
        const Vector v = p - p0;

        if (line_vec.cross(v).magnitude() / line_length > tolerance)
            return false;
    }

    return true;
}

bool NurbsCurve::is_planar(Plane* plane, double tolerance) const {

    if (!is_valid() || m_cv_count < 3)
        return true;

    const Point p0 = get_cv(0);
    const Point p1 = get_cv(m_cv_count / 2);
    const Point p2 = get_cv(m_cv_count - 1);
    const Vector v1 = p1 - p0;
    const Vector v2 = p2 - p0;
    Vector normal = v1.cross(v2);

    if (normal.magnitude() < tolerance)
        return true;

    for (int i = 0; i < m_cv_count; i++) {
        const Point p = get_cv(i);
        const Vector v = p - p0;

        if (std::abs(v.dot(normal)) / normal.magnitude() > tolerance)
            return false;
    }

    if (plane) {
        normal.normalize_self();

        Vector x_axis = v1;
        x_axis.normalize_self();
        *plane = Plane(p0, x_axis, normal.cross(x_axis));
    }

    return true;
}

bool NurbsCurve::is_arc(Plane* plane, double tolerance) const {

    if (!is_valid())
        return false;

    if (m_dim != 2 && m_dim != 3)
        return false;

    if (m_order < 3)
        return false;

    if (is_linear(tolerance))
        return false;

    Plane test_plane;

    if (!is_planar(&test_plane, tolerance))
        return false;

    const double t0 = domain_start();
    const double t1 = domain_end();
    const Point p0 = point_at(t0);
    Point center;

    if (!circle_center(p0, point_at((t0 + t1) * 0.5), point_at(t1), center))
        return false;

    const double radius = center.distance(p0);

    if (radius < Tolerance::ZERO_TOLERANCE)
        return false;

    const int samples_per_span = std::max(4, 2 * degree() + 1);
    const int num_samples = span_count() * samples_per_span;

    for (int i = 0; i <= num_samples; i++) {
        const double t = t0 + (t1 - t0) * i / num_samples;

        if (std::abs(point_at(t).distance(center) - radius) > tolerance)
            return false;
    }

    if (plane)
        *plane = test_plane;

    return true;
}

bool NurbsCurve::is_in_plane(const Plane& test_plane, double tolerance) const {

    if (!is_valid())
        return false;

    for (int i = 0; i < m_cv_count; i++) {
        const Point pt = get_cv(i);
        const Vector v = pt - test_plane.origin();

        if (std::abs(v.dot(test_plane.z_axis())) > tolerance)
            return false;
    }

    return true;
}

bool NurbsCurve::is_natural(int end) const {

    if (!is_valid())
        return false;

    const double tol_factor = 1e-8;
    const double t0 = domain_start();
    const double t1 = domain_end();

    for (int pass = ((end == 0 || end == 2) ? 0 : 1); pass < ((end == 1 || end == 2) ? 2 : 1); ++pass) {
        const double t = (pass == 0) ? t0 : t1;
        const std::vector<Vector> derivs = evaluate(t, 2);

        if (derivs.size() < 3)
            return false;

        const double d2_len = derivs[2].magnitude();
        const Point cv0 = get_cv((pass == 0) ? 0 : m_cv_count - 1);
        const Point cv2 = get_cv((pass == 0) ? std::min(2, m_cv_count - 1) : std::max(0, m_cv_count - 3));

        if (d2_len > cv0.distance(cv2) * tol_factor)
            return false;
    }

    return true;
}

int NurbsCurve::is_polyline(std::vector<Point>* points, std::vector<double>* params) const {

    if (!is_valid())
        return 0;

    if (m_order == 2) {
        if (points) {
            points->clear();
            points->reserve(m_cv_count);

            for (int i = 0; i < m_cv_count; i++)
                points->push_back(get_cv(i));
        }

        if (params) {
            params->clear();
            params->reserve(m_cv_count);

            for (int i = 0; i < m_cv_count; i++)
                params->push_back(m_nurbsknot[i]);
        }

        return m_cv_count;
    }

    if (m_order > 2 && m_dim >= 2 && m_dim <= 3) {
        const int span_cnt = span_count();
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

                for (int i = 0; i < span_cnt; i++)
                    points->push_back(get_cv(i * (m_order - 1) + (m_order - 1)));
            }

            if (params)
                *params = get_span_vector();

            return span_cnt + 1;
        }
    }

    return 0;
}

bool NurbsCurve::is_singular() const {

    if (!is_valid())
        return false;

    const int span_cnt = span_count();

    for (int i = 0; i < span_cnt; i++)
        if (!span_is_singular(i))
            return false;

    return true;
}

bool NurbsCurve::is_duplicate(const NurbsCurve& other, bool ignore_parameterization, double tolerance) const {

    if (!is_valid() || !other.is_valid())
        return false;

    if (m_dim != other.m_dim)
        return false;

    if (m_is_rat != other.m_is_rat)
        return false;

    if (m_order != other.m_order)
        return false;

    if (m_cv_count != other.m_cv_count)
        return false;

    for (int i = 0; i < m_cv_count; i++) {
        if (get_cv(i).distance(other.get_cv(i)) > tolerance)
            return false;

        if (m_is_rat && std::abs(weight(i) - other.weight(i)) > tolerance)
            return false;
    }

    if (!ignore_parameterization)
        for (int i = 0; i < nurbsknot_count(); i++)
            if (std::abs(m_nurbsknot[i] - other.m_nurbsknot[i]) > tolerance)
                return false;

    return true;
}

bool NurbsCurve::is_continuous(int continuity_type, double t, int* hint, double point_tolerance, double d1_tolerance, double d2_tolerance, double cos_angle_tolerance, double curvature_tolerance) const {

    (void)point_tolerance;
    (void)d1_tolerance;
    (void)d2_tolerance;
    (void)cos_angle_tolerance;
    (void)curvature_tolerance;

    if (!is_valid())
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();

    if (t < d0 || t > d1)
        return false;

    const int span = find_span(t);

    if (hint)
        *hint = span;

    int nurbsknot_idx = -1;

    for (int i = 0; i < nurbsknot_count(); i++) {
        if (std::abs(m_nurbsknot[i] - t) < Tolerance::ZERO_TOLERANCE) {
            nurbsknot_idx = i;
            break;
        }
    }

    if (nurbsknot_idx < 0)
        return true;

    const int mult = nurbsknot_multiplicity(nurbsknot_idx);

    if (continuity_type == 0)
        return mult < m_order;

    if (continuity_type == 1)
        return mult < m_order - 1;

    if (continuity_type == 2)
        return mult < m_order - 2;

    return mult < m_order - 1;
}

bool NurbsCurve::is_valid_nurbsknot_vector() const {

    const int kc = nurbsknot_count();

    if (static_cast<int>(m_nurbsknot.size()) != kc)
        return false;

    for (int i = 1; i < kc; i++)
        if (m_nurbsknot[i] < m_nurbsknot[i - 1])
            return false;

    if (m_nurbsknot[m_order - 2] >= m_nurbsknot[m_cv_count - 1])
        return false;

    return true;
}

bool NurbsCurve::is_clamped(int end) const {

    if (!is_valid())
        return false;

    return nurbsknot::is_clamped(m_order, m_cv_count, m_nurbsknot, end);
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
int NurbsCurve::cv_size() const { return (m_dim > 0) ? (m_is_rat ? (m_dim + 1) : m_dim) : 0; }

int NurbsCurve::nurbsknot_count() const { return m_order + m_cv_count - 2; }

int NurbsCurve::span_count() const {

    int count = 0;
    const int kc = nurbsknot_count();

    for (int i = m_order - 2; i < m_cv_count - 1; i++)
        if (i >= 0 && i + 1 < kc && m_nurbsknot[i] < m_nurbsknot[i + 1])
            count++;

    return count;
}

// ═══════════════════════════════════════════════════════════════════════════
// Control vertex access
// ═══════════════════════════════════════════════════════════════════════════
double* NurbsCurve::cv(int cv_index) {

    if (cv_index < 0 || cv_index >= m_cv_count)
        return nullptr;

    return &m_cv[cv_index * m_cv_stride];
}

const double* NurbsCurve::cv(int cv_index) const {

    if (cv_index < 0 || cv_index >= m_cv_count)
        return nullptr;

    return &m_cv[cv_index * m_cv_stride];
}

Point NurbsCurve::get_cv(int cv_index) const {

    const double* cv_ptr = cv(cv_index);

    if (!cv_ptr)
        return Point(0, 0, 0);

    if (m_is_rat) {
        const double w = cv_ptr[m_dim];

        if (std::abs(w) < 1e-14)
            return Point(0, 0, 0);

        return Point(cv_ptr[0] / w, cv_ptr[1] / w, m_dim > 2 ? cv_ptr[2] / w : 0.0);
    }

    return Point(cv_ptr[0], cv_ptr[1], m_dim > 2 ? cv_ptr[2] : 0.0);
}

bool NurbsCurve::get_cv_4d(int cv_index, double& x, double& y, double& z, double& w) const {

    const double* cv_ptr = cv(cv_index);

    if (!cv_ptr)
        return false;

    x = cv_ptr[0];
    y = m_dim > 1 ? cv_ptr[1] : 0.0;
    z = m_dim > 2 ? cv_ptr[2] : 0.0;
    w = m_is_rat ? cv_ptr[m_dim] : 1.0;

    return true;
}

std::tuple<double, double, double, double> NurbsCurve::get_cv_4d(int cv_index) const {

    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;
    get_cv_4d(cv_index, x, y, z, w);

    return {x, y, z, w};
}

bool NurbsCurve::set_cv(int cv_index, const Point& point) {

    double* cv_ptr = cv(cv_index);

    if (!cv_ptr)
        return false;

    cv_ptr[0] = point[0];

    if (m_dim > 1)
        cv_ptr[1] = point[1];

    if (m_dim > 2)
        cv_ptr[2] = point[2];

    if (m_is_rat)
        cv_ptr[m_dim] = 1.0;

    return true;
}

bool NurbsCurve::set_cv_4d(int cv_index, double x, double y, double z, double w) {

    if (cv_index < 0 || cv_index >= m_cv_count)
        return false;

    if (!m_is_rat && w != 1.0 && !to_rational())
        return false;

    double* cv_ptr = cv(cv_index);

    if (!cv_ptr)
        return false;

    cv_ptr[0] = x;

    if (m_dim > 1)
        cv_ptr[1] = y;

    if (m_dim > 2)
        cv_ptr[2] = z;

    if (m_is_rat)
        cv_ptr[m_dim] = w;

    return true;
}

double NurbsCurve::weight(int cv_index) const {

    if (!m_is_rat)
        return 1.0;

    const double* cv_ptr = cv(cv_index);

    return cv_ptr ? cv_ptr[m_dim] : 1.0;
}

bool NurbsCurve::set_weight(int cv_index, double weight) {

    if (!m_is_rat && !to_rational())
        return false;

    double* cv_ptr = cv(cv_index);

    if (!cv_ptr)
        return false;

    cv_ptr[m_dim] = weight;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsKnot access
// ═══════════════════════════════════════════════════════════════════════════
double NurbsCurve::nurbsknot(int nurbsknot_index) const {

    if (nurbsknot_index < 0 || nurbsknot_index >= static_cast<int>(m_nurbsknot.size()))
        return 0.0;

    return m_nurbsknot[nurbsknot_index];
}

bool NurbsCurve::set_nurbsknot(int nurbsknot_index, double nurbsknot_value) {

    if (nurbsknot_index < 0 || nurbsknot_index >= static_cast<int>(m_nurbsknot.size()))
        return false;

    m_nurbsknot[nurbsknot_index] = nurbsknot_value;

    return true;
}

int NurbsCurve::nurbsknot_multiplicity(int nurbsknot_index) const {

    if (nurbsknot_index < 0 || nurbsknot_index >= nurbsknot_count())
        return 0;

    const double nurbsknot_value = m_nurbsknot[nurbsknot_index];
    int mult = 1;

    for (int i = nurbsknot_index + 1; i < nurbsknot_count(); i++) {
        if (std::abs(m_nurbsknot[i] - nurbsknot_value) >= Tolerance::ZERO_TOLERANCE)
            break;

        mult++;
    }

    for (int i = nurbsknot_index - 1; i >= 0; i--) {
        if (std::abs(m_nurbsknot[i] - nurbsknot_value) >= Tolerance::ZERO_TOLERANCE)
            break;

        mult++;
    }

    return mult;
}

double NurbsCurve::superfluous_nurbsknot(int end) const {

    if (!is_valid())
        return 0.0;

    if (end == 0)
        return 2.0 * m_nurbsknot[0] - m_nurbsknot[m_order - 2];

    return 2.0 * m_nurbsknot[nurbsknot_count() - 1] - m_nurbsknot[m_cv_count - m_order];
}

bool NurbsCurve::insert_nurbsknot(double nurbsknot_value, int nurbsknot_multiplicity) {

    if (!is_valid())
        return false;

    const int p = degree();

    if (nurbsknot_multiplicity < 1 || nurbsknot_multiplicity > p)
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();

    if (nurbsknot_value < d0 || nurbsknot_value > d1)
        return false;

    if (nurbsknot_value == d0) {
        if (nurbsknot_multiplicity == p)
            return clamp_end(0);

        return nurbsknot_multiplicity == 1;
    }

    if (nurbsknot_value == d1) {
        if (nurbsknot_multiplicity == p)
            return clamp_end(1);

        return nurbsknot_multiplicity == 1;
    }

    const double tol = (std::abs(d0) + std::abs(d1) + std::abs(d1 - d0)) * SQRT_EPSILON;

    for (int insert_iter = 0; insert_iter < nurbsknot_multiplicity; ++insert_iter) {
        const std::vector<double> U = full_nurbsknots();
        int mult = 0;

        for (double knot : U)
            if (std::abs(knot - nurbsknot_value) <= tol)
                ++mult;

        if (mult >= nurbsknot_multiplicity)
            return true;

        if (mult >= p)
            return false;

        insert_nurbsknot_once(nurbsknot_value, U);
    }

    return true;
}

double NurbsCurve::greville_abcissa(int cv_index) const {

    if (cv_index < 0 || cv_index >= m_cv_count)
        return 0.0;

    const double* nurbsknot = m_nurbsknot.data() + cv_index;
    const int order = m_order;

    if (order <= 2 || nurbsknot[0] == nurbsknot[order - 2])
        return nurbsknot[0];

    const int p = order - 1;
    const double k0 = nurbsknot[0];
    const double k = nurbsknot[p / 2];
    const double k1 = nurbsknot[p - 1];
    const double tol = (k1 - k0) * SQRT_EPSILON;
    double g = 0.0;

    for (int i = 0; i < p; i++)
        g += nurbsknot[i];

    g /= static_cast<double>(p);

    if (std::fabs(2.0 * k - (k0 + k1)) <= tol && std::fabs(g - k) <= (std::fabs(g) * SQRT_EPSILON + tol))
        g = k;

    return g;
}

bool NurbsCurve::get_greville_abcissae(std::vector<double>& abcissae) const {

    if (!is_valid())
        return false;

    abcissae.resize(m_cv_count);

    for (int i = 0; i < m_cv_count; i++)
        abcissae[i] = greville_abcissa(i);

    return true;
}

std::vector<double> NurbsCurve::get_greville_abcissae() const {

    std::vector<double> result;
    get_greville_abcissae(result);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Domain
// ═══════════════════════════════════════════════════════════════════════════
std::pair<double, double> NurbsCurve::domain() const {

    if (m_nurbsknot.empty())
        return {0.0, 0.0};

    return {m_nurbsknot[m_order - 2], m_nurbsknot[m_cv_count - 1]};
}

double NurbsCurve::domain_start() const {

    if (m_nurbsknot.empty())
        return 0.0;

    return m_nurbsknot[m_order - 2];
}

double NurbsCurve::domain_end() const {

    if (m_nurbsknot.empty())
        return 0.0;

    return m_nurbsknot[m_cv_count - 1];
}

double NurbsCurve::domain_middle() const {

    if (m_nurbsknot.empty())
        return 0.0;

    return (m_nurbsknot[m_order - 2] + m_nurbsknot[m_cv_count - 1]) * 0.5;
}

bool NurbsCurve::set_domain(double t0, double t1) {

    if (t0 >= t1 || !is_valid())
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();

    if (d0 >= d1)
        return false;

    const bool clamped_start = (m_order >= 2 && std::abs(m_nurbsknot[0] - m_nurbsknot[m_order - 2]) < Tolerance::ZERO_TOLERANCE);
    const bool clamped_end = (m_cv_count < static_cast<int>(m_nurbsknot.size()) && std::abs(m_nurbsknot.back() - m_nurbsknot[m_cv_count - 1]) < Tolerance::ZERO_TOLERANCE);
    const double scale = (t1 - t0) / (d1 - d0);

    for (double& k : m_nurbsknot)
        k = t0 + (k - d0) * scale;

    if (clamped_start)
        for (int i = 0; i < m_order - 1; i++)
            m_nurbsknot[i] = t0;

    if (clamped_end)
        for (int i = m_cv_count - 1; i < static_cast<int>(m_nurbsknot.size()); i++)
            m_nurbsknot[i] = t1;

    return true;
}

std::vector<double> NurbsCurve::get_span_vector() const {

    std::vector<double> spans;
    spans.push_back(m_nurbsknot[m_order - 2]);

    for (int i = m_order - 1; i < m_cv_count; i++)
        if (m_nurbsknot[i] > spans.back())
            spans.push_back(m_nurbsknot[i]);

    return spans;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsCurve::get_next_discontinuity(int continuity_type, double t0, double t1, double& t_out, int* hint, double cos_angle_tolerance, double curvature_tolerance) const {

    (void)cos_angle_tolerance;
    (void)curvature_tolerance;

    if (!is_valid())
        return false;

    if (t0 >= t1)
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();

    if (t0 < d0)
        t0 = d0;

    if (t1 > d1)
        t1 = d1;

    if (t0 >= t1)
        return false;

    for (int i = m_order - 1; i < m_cv_count - 1; i++) {
        const double t = m_nurbsknot[i];

        if (t <= t0 || t >= t1)
            continue;

        const int mult = nurbsknot_multiplicity(i);
        bool found = false;

        if (continuity_type == 0)
            found = mult >= m_order;
        else if (continuity_type == 1 || continuity_type == 3 || continuity_type == 4)
            found = mult >= m_order - 1;
        else if (continuity_type == 2)
            found = mult >= m_order - 2;

        if (!found)
            continue;

        t_out = t;

        if (hint)
            *hint = i;

        return true;
    }

    return false;
}

std::pair<bool, double> NurbsCurve::get_next_discontinuity(int continuity_type, double t0, double t1) const {

    double t_out = 0.0;
    const bool found = get_next_discontinuity(continuity_type, t0, t1, t_out);

    return {found, t_out};
}

double NurbsCurve::length(double tolerance) const {

    (void)tolerance;

    if (!is_valid())
        return 0.0;

    static const double GL_X[10] = {-0.9739065285171717, -0.8650633666889845, -0.6794095682990244, -0.4333953941292472, -0.1488743389816312, 0.1488743389816312, 0.4333953941292472, 0.6794095682990244, 0.8650633666889845, 0.9739065285171717};
    static const double GL_W[10] = {0.0666713443086881, 0.1494513491505806, 0.2190863625159820, 0.2692667193099963, 0.2955242247147529, 0.2955242247147529, 0.2692667193099963, 0.2190863625159820, 0.1494513491505806, 0.0666713443086881};

    const int SUBDIVISIONS = 4;
    double total = 0.0;
    const int n_spans = span_count();

    for (int span = 0; span < n_spans; span++) {
        const double span_a = m_nurbsknot[m_order - 2 + span];
        const double span_b = m_nurbsknot[m_order - 1 + span];

        if (span_b <= span_a)
            continue;

        const double span_width = (span_b - span_a) / SUBDIVISIONS;

        for (int sub = 0; sub < SUBDIVISIONS; sub++) {
            const double a = span_a + sub * span_width;
            const double b = a + span_width;
            const double mid = (a + b) * 0.5;
            const double half = (b - a) * 0.5;
            double s = 0.0;

            for (int i = 0; i < 10; i++)
                s += GL_W[i] * evaluate(mid + half * GL_X[i], 1)[1].magnitude();

            total += half * s;
        }
    }

    return total;
}

/// Order two (t, point) samples by parameter.
static bool sample_before(const std::pair<double, Point>& a, const std::pair<double, Point>& b) { return a.first < b.first; }

bool NurbsCurve::to_polyline_adaptive(std::vector<Point>& points, std::vector<double>* params, double angle_tolerance, double min_edge_length, double max_edge_length) const {

    points.clear();

    if (params)
        params->clear();

    if (!is_valid())
        return false;

    if (angle_tolerance <= 0.0)
        angle_tolerance = 0.1;

    const double curve_len = length();

    if (max_edge_length <= 0.0)
        max_edge_length = curve_len / 10.0;

    if (min_edge_length <= 0.0)
        min_edge_length = curve_len / 1000.0;

    if (min_edge_length > max_edge_length)
        min_edge_length = max_edge_length * 0.1;

    const std::vector<std::pair<double, Point>> samples = adaptive_samples(angle_tolerance, min_edge_length, max_edge_length);
    points.reserve(samples.size());

    if (params)
        params->reserve(samples.size());

    for (const std::pair<double, Point>& sample : samples) {
        points.push_back(sample.second);

        if (params)
            params->push_back(sample.first);
    }

    return points.size() >= 2;
}

std::pair<std::vector<Point>, std::vector<double>> NurbsCurve::to_polyline_adaptive(double angle_tolerance, double min_edge_length, double max_edge_length) const {

    std::vector<Point> pts;
    std::vector<double> params;
    to_polyline_adaptive(pts, &params, angle_tolerance, min_edge_length, max_edge_length);

    return {pts, params};
}

bool NurbsCurve::divide_by_count(int count, std::vector<Point>& points, std::vector<double>* params, bool include_endpoints) const {

    points.clear();

    if (params)
        params->clear();

    if (!is_valid())
        return false;

    if (count < 2)
        return false;

    const double t0 = domain_start();
    const double t1 = domain_end();
    const double h = (t1 - t0) * 1e-8;
    const int n_samples = std::max(1000, count * 100);
    const double dt = (t1 - t0) / n_samples;
    std::vector<double> t_vals(n_samples + 1);
    std::vector<double> s_vals(n_samples + 1);
    t_vals[0] = t0;
    s_vals[0] = 0.0;

    for (int i = 1; i <= n_samples; i++) {
        t_vals[i] = t0 + i * dt;
        s_vals[i] = s_vals[i - 1] + arc_length_gauss(t_vals[i - 1], t_vals[i], h);
    }

    const int n_segs = include_endpoints ? (count - 1) : (count + 1);
    const double seg_len = s_vals[n_samples] / n_segs;
    points.reserve(count);

    if (params)
        params->reserve(count);

    for (int i = 0; i < count; i++) {
        const double s_target = include_endpoints ? seg_len * i : seg_len * (i + 1);
        const double t = find_t_at_s(s_target, t_vals, s_vals, h);
        points.push_back(point_at(t));

        if (params)
            params->push_back(t);
    }

    return true;
}

std::pair<std::vector<Point>, std::vector<double>> NurbsCurve::divide_by_count(int count, bool include_endpoints) const {

    std::vector<Point> pts;
    std::vector<double> params;
    divide_by_count(count, pts, &params, include_endpoints);

    return {pts, params};
}

bool NurbsCurve::divide_by_length(double segment_length, std::vector<Point>& points, std::vector<double>* params) const {

    points.clear();

    if (params)
        params->clear();

    if (!is_valid())
        return false;

    if (segment_length <= 0.0)
        return false;

    const double t0 = domain_start();
    const double t1 = domain_end();
    const double h = (t1 - t0) * 1e-8;
    const int n_samples = std::max(1000, static_cast<int>(length() / segment_length) * 100);
    const double dt = (t1 - t0) / n_samples;
    std::vector<double> t_vals(n_samples + 1);
    std::vector<double> s_vals(n_samples + 1);
    t_vals[0] = t0;
    s_vals[0] = 0.0;

    for (int i = 1; i <= n_samples; i++) {
        t_vals[i] = t0 + i * dt;
        s_vals[i] = s_vals[i - 1] + arc_length_gauss(t_vals[i - 1], t_vals[i], h);
    }

    const double total_len = s_vals[n_samples];

    for (double s = 0.0; s <= total_len + 1e-10; s += segment_length) {
        const double t = find_t_at_s(s, t_vals, s_vals, h);
        points.push_back(point_at(t));

        if (params)
            params->push_back(t);
    }

    return points.size() >= 2;
}

std::pair<std::vector<Point>, std::vector<double>> NurbsCurve::divide_by_length(double segment_length) const {

    std::vector<Point> pts;
    std::vector<double> params;
    divide_by_length(segment_length, pts, &params);

    return {pts, params};
}

// ═══════════════════════════════════════════════════════════════════════════
// Evaluation
// ═══════════════════════════════════════════════════════════════════════════
Point NurbsCurve::point_at(double t) const {

    if (!is_valid())
        return Point(0, 0, 0);

    const int span = find_span(t);
    std::vector<double> basis;
    basis_functions(span, t, basis);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;

    for (int i = 0; i < m_order; i++) {
        const double* cv_ptr = cv(span + i);

        if (!cv_ptr)
            continue;

        const double N = basis[i];
        x += N * cv_ptr[0];
        y += N * (m_dim > 1 ? cv_ptr[1] : 0.0);
        z += N * (m_dim > 2 ? cv_ptr[2] : 0.0);

        if (m_is_rat)
            w += N * cv_ptr[m_dim];
        else
            w = 1.0;
    }

    if (m_is_rat && w != 0.0)
        return Point(x / w, y / w, z / w);

    return Point(x, y, z);
}

std::vector<Vector> NurbsCurve::evaluate(double t, int derivative_count) const {

    std::vector<Vector> result;

    if (!is_valid()) {
        result.push_back(Vector(0, 0, 0));

        return result;
    }

    const int max_derivs = std::min(derivative_count, degree());
    const int span = find_span(t);
    std::vector<std::vector<double>> ders;
    basis_functions_derivatives(span, t, max_derivs, ders);

    const std::vector<std::array<double, 4>> Aders = homogeneous_derivatives(span, ders);
    std::vector<std::array<double, 3>> Cders(max_derivs + 1);

    if (!m_is_rat) {
        for (int k = 0; k <= max_derivs; ++k)
            Cders[k] = {Aders[k][0], Aders[k][1], Aders[k][2]};
    } else {
        for (int k = 0; k <= max_derivs; ++k) {
            const double w = Aders[0][3];
            const double inv_w = (w != 0.0) ? 1.0 / w : 0.0;
            double Ck_x = Aders[k][0];
            double Ck_y = Aders[k][1];
            double Ck_z = Aders[k][2];

            for (int j = 1; j <= k; ++j) {
                const double coeff = std::tgamma(k + 1) / (std::tgamma(j + 1) * std::tgamma(k - j + 1));
                const double wj = Aders[j][3];
                Ck_x -= coeff * wj * Cders[k - j][0];
                Ck_y -= coeff * wj * Cders[k - j][1];
                Ck_z -= coeff * wj * Cders[k - j][2];
            }

            Cders[k] = {Ck_x * inv_w, Ck_y * inv_w, Ck_z * inv_w};
        }
    }

    for (int k = 0; k <= max_derivs; ++k)
        result.emplace_back(Cders[k][0], Cders[k][1], Cders[k][2]);

    for (int k = max_derivs + 1; k <= derivative_count; ++k)
        result.emplace_back(0.0, 0.0, 0.0);

    return result;
}

Vector NurbsCurve::tangent_at(double t) const {

    if (!is_valid())
        return Vector(0, 0, 0);

    const double t0 = domain_start();
    const double t1 = domain_end();
    const double h = (t1 - t0) * 1e-7;
    Point p1;
    Point p2;

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

    Vector tan = p2 - p1;

    if (tan.magnitude() > 1e-14)
        tan.normalize_self();

    return tan;
}

double NurbsCurve::curvature_at(double t) const {

    const std::vector<Vector> d = evaluate(t, 2);

    if (d.size() < 3)
        return 0.0;

    const double s = d[1].magnitude();

    if (s < Tolerance::ZERO_TOLERANCE)
        return 0.0;

    return d[1].cross(d[2]).magnitude() / (s * s * s);
}

double NurbsCurve::closest_parameter(const Point& test_point) const { return Closest::curve_point(*this, test_point).first; }

Point NurbsCurve::closest_point(const Point& test_point) const { return point_at(closest_parameter(test_point)); }

std::pair<double, double> NurbsCurve::closest_parameters_curve(const NurbsCurve& other) const {

    const std::tuple<double, double, double> result = Closest::curve_curve(*this, other);

    return {std::get<0>(result), std::get<1>(result)};
}

std::pair<Point, Point> NurbsCurve::closest_points_curve(const NurbsCurve& other) const {

    const std::pair<double, double> params = closest_parameters_curve(other);

    return {point_at(params.first), other.point_at(params.second)};
}

Plane NurbsCurve::plane_at(double t, bool normalized) const {

    if (!is_valid())
        return Plane::invalid();

    const double t0 = domain_start();
    const double t1 = domain_end();
    double param;

    if (normalized) {
        if (t < 0.0 || t > 1.0)
            return Plane::invalid();

        param = t0 + t * (t1 - t0);
    } else {
        if (t < t0 || t > t1)
            return Plane::invalid();

        param = t;
    }

    const double h = (t1 - t0) * 1e-5;
    const Point origin = point_at(param);

    if (param <= t0 + h) {
        const Point p0 = point_at(t0);
        const Point pp = point_at(t0 + h);
        const Point pp2 = point_at(t0 + 2 * h);
        const Vector d1 = pp - p0;
        const Vector d2((pp2[0] - 2 * pp[0] + p0[0]) / (h * h), (pp2[1] - 2 * pp[1] + p0[1]) / (h * h), (pp2[2] - 2 * pp[2] + p0[2]) / (h * h));

        return frenet_frame(origin, d1, d2);
    }

    if (param >= t1 - h) {
        const Point pm = point_at(t1 - h);
        const Point p0 = point_at(t1);
        const Point pm2 = point_at(t1 - 2 * h);
        const Vector d1 = p0 - pm;
        const Vector d2((p0[0] - 2 * pm[0] + pm2[0]) / (h * h), (p0[1] - 2 * pm[1] + pm2[1]) / (h * h), (p0[2] - 2 * pm[2] + pm2[2]) / (h * h));

        return frenet_frame(origin, d1, d2);
    }

    const Point pm = point_at(param - h);
    const Point p0 = point_at(param);
    const Point pp = point_at(param + h);
    const Vector d1 = (pp - pm) / (2 * h);
    const Vector d2((pp[0] - 2 * p0[0] + pm[0]) / (h * h), (pp[1] - 2 * p0[1] + pm[1]) / (h * h), (pp[2] - 2 * p0[2] + pm[2]) / (h * h));

    return frenet_frame(origin, d1, d2);
}

Plane NurbsCurve::perpendicular_plane_at(double t, bool normalized) const {

    if (!is_valid())
        return Plane::invalid();

    const double t0 = domain_start();
    const double t1 = domain_end();
    double param;

    if (normalized) {
        if (t < 0.0 || t > 1.0)
            return Plane::invalid();

        param = t0 + t * (t1 - t0);
    } else {
        if (t < t0 || t > t1)
            return Plane::invalid();

        param = t;
    }

    Vector T0;
    Vector r0;

    if (!start_frame(T0, r0))
        return Plane::invalid();

    const Point origin = point_at(param);

    if (std::abs(param - t0) < 1e-14) {
        Vector s0 = T0.cross(r0);
        s0.normalize_self();

        return Plane::from_frame(origin, r0, s0, T0);
    }

    Vector ri = double_reflection(param, r0, T0);
    Vector T = tangent_at(param);
    T.normalize_self();

    const double ri_dot_T = ri.dot(T);
    ri -= T * ri_dot_T;

    if (ri.magnitude() > 1e-14)
        ri.normalize_self();

    Vector s = T.cross(ri);
    s.normalize_self();

    return Plane::from_frame(origin, ri, s, T);
}

std::vector<Plane> NurbsCurve::get_perpendicular_planes(int count) const {

    std::vector<Plane> frames;
    std::vector<Point> pts;
    std::vector<double> params;
    divide_by_count(count + 1, pts, &params, true);

    for (double t : params)
        frames.push_back(perpendicular_plane_at(t, false));

    return frames;
}

Point NurbsCurve::point_at_start() const { return point_at(domain_start()); }

Point NurbsCurve::point_at_middle() const { return point_at(domain_middle()); }

Point NurbsCurve::point_at_end() const { return point_at(domain_end()); }

bool NurbsCurve::set_start_point(const Point& start_point) {

    if (!is_valid() || !clamp_end(2))
        return false;

    const double w = m_is_rat ? weight(0) : 1.0;

    if (m_is_rat && w != 1.0) {
        set_cv_4d(0, start_point[0] * w, start_point[1] * w, start_point[2] * w, w);
    } else {
        set_cv(0, start_point);

        if (m_is_rat)
            set_weight(0, w);
    }

    return true;
}

bool NurbsCurve::set_end_point(const Point& end_point) {

    if (!is_valid() || !clamp_end(2))
        return false;

    const int last = m_cv_count - 1;
    const double w = m_is_rat ? weight(last) : 1.0;

    if (m_is_rat && w != 1.0) {
        set_cv_4d(last, end_point[0] * w, end_point[1] * w, end_point[2] * w, w);
    } else {
        set_cv(last, end_point);

        if (m_is_rat)
            set_weight(last, w);
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Modifications
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsCurve::reverse() {

    if (!is_valid())
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();

    for (double& k : m_nurbsknot)
        k = d0 + d1 - k;

    std::reverse(m_nurbsknot.begin(), m_nurbsknot.end());

    for (int i = 0; i < m_cv_count / 2; i++) {
        const int j = m_cv_count - 1 - i;
        double xi;
        double yi;
        double zi;
        double wi;
        double xj;
        double yj;
        double zj;
        double wj;

        if (!get_cv_4d(i, xi, yi, zi, wi) || !get_cv_4d(j, xj, yj, zj, wj))
            continue;

        set_cv_4d(i, xj, yj, zj, wj);
        set_cv_4d(j, xi, yi, zi, wi);
    }

    return true;
}

bool NurbsCurve::swap_coordinates(int axis_i, int axis_j) {

    if (!is_valid())
        return false;

    if (axis_i < 0 || axis_i >= m_dim)
        return false;

    if (axis_j < 0 || axis_j >= m_dim)
        return false;

    if (axis_i == axis_j)
        return true;

    for (int cv_idx = 0; cv_idx < m_cv_count; cv_idx++) {
        double* cv_ptr = cv(cv_idx);
        std::swap(cv_ptr[axis_i], cv_ptr[axis_j]);
    }

    return true;
}

bool NurbsCurve::trim(double t0, double t1) {

    if (!is_valid() || t0 >= t1)
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();

    if (t0 < d0 - Tolerance::ZERO_TOLERANCE || t1 > d1 + Tolerance::ZERO_TOLERANCE)
        return false;

    t0 = std::max(t0, d0);
    t1 = std::min(t1, d1);

    if (std::abs(t0 - d0) < Tolerance::ZERO_TOLERANCE && std::abs(t1 - d1) < Tolerance::ZERO_TOLERANCE)
        return true;

    const int p = degree();
    const bool trim_start = (t0 > d0 + Tolerance::ZERO_TOLERANCE);
    const bool trim_end = (t1 < d1 - Tolerance::ZERO_TOLERANCE);

    const double stol = (std::abs(d0) + std::abs(d1) + std::abs(d1 - d0)) * SQRT_EPSILON;

    for (double k : m_nurbsknot) {
        if (trim_start && std::abs(k - t0) <= stol && std::abs(k - t0) > 0.0)
            t0 = k;

        if (trim_end && std::abs(k - t1) <= stol && std::abs(k - t1) > 0.0)
            t1 = k;
    }

    if (t0 >= t1)
        return false;

    if (trim_start && !insert_nurbsknot(t0, p))
        return false;

    if (trim_end && !insert_nurbsknot(t1, p))
        return false;

    return keep_span_range(t0, t1);
}

bool NurbsCurve::split(double t, NurbsCurve& left_curve, NurbsCurve& right_curve) const {

    if (!is_valid())
        return false;

    const double t0 = domain_start();
    const double t1 = domain_end();

    if (t <= t0 || t >= t1)
        return false;

    left_curve = *this;
    right_curve = *this;

    if (!left_curve.trim(t0, t))
        return false;

    if (!right_curve.trim(t, t1))
        return false;

    return true;
}

std::pair<NurbsCurve, NurbsCurve> NurbsCurve::split(double t) const {

    NurbsCurve left;
    NurbsCurve right;
    split(t, left, right);

    return {left, right};
}

bool NurbsCurve::extend(double t0, double t1) {

    if (!is_valid() || is_closed())
        return false;

    const double d0 = domain_start();
    const double d1 = domain_end();
    const int cvdim = cv_size();
    bool changed = false;

    if (t0 < d0) {
        if (!clamp_end(0) || !evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[0], &m_nurbsknot[0], 1, t0))
            return false;

        for (int i = 0; i < m_order - 1; i++)
            m_nurbsknot[i] = t0;

        changed = true;
    }

    if (t1 > d1) {
        if (!clamp_end(1))
            return false;

        const int i0 = m_cv_count - m_order;

        if (!evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[i0 * m_cv_stride], &m_nurbsknot[i0], -1, t1))
            return false;

        const int kc = nurbsknot_count();

        for (int i = m_cv_count - 1; i < kc; i++)
            m_nurbsknot[i] = t1;

        changed = true;
    }

    return changed;
}

bool NurbsCurve::to_rational() {

    if (m_is_rat)
        return true;

    const int new_stride = m_dim + 1;
    std::vector<double> new_cv(m_cv_count * new_stride);

    for (int i = 0; i < m_cv_count; i++) {
        const double* old_cv = cv(i);
        double* new_cv_ptr = &new_cv[i * new_stride];

        for (int j = 0; j < m_dim; j++)
            new_cv_ptr[j] = old_cv[j];

        new_cv_ptr[m_dim] = 1.0;
    }

    m_cv = new_cv;
    m_is_rat = 1;
    m_cv_stride = new_stride;

    return true;
}

bool NurbsCurve::to_non_rational(bool force) {

    if (!m_is_rat)
        return true;

    if (force) {
        for (int i = 0; i < m_cv_count; i++) {
            double* cv_ptr = cv(i);

            if (cv_ptr)
                cv_ptr[m_dim] = 1.0;
        }
    } else {
        const double w0 = weight(0);

        for (int i = 1; i < m_cv_count; i++)
            if (std::abs(weight(i) - w0) > Tolerance::ZERO_TOLERANCE)
                return false;
    }

    const int new_stride = m_dim;
    std::vector<double> new_cv(m_cv_count * new_stride);

    for (int i = 0; i < m_cv_count; i++) {
        const Point p = get_cv(i);
        double* new_cv_ptr = &new_cv[i * new_stride];
        new_cv_ptr[0] = p[0];

        if (m_dim > 1)
            new_cv_ptr[1] = p[1];

        if (m_dim > 2)
            new_cv_ptr[2] = p[2];
    }

    m_cv = new_cv;
    m_is_rat = 0;
    m_cv_stride = new_stride;

    return true;
}

bool NurbsCurve::clamp_end(int end) {

    if (!is_valid())
        return false;

    if (end < 0 || end > 2)
        return false;

    const int cvdim = cv_size();
    bool rc = true;

    if (end == 0 || end == 2) {
        const double t = m_nurbsknot[m_order - 2];

        if (evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[0], &m_nurbsknot[0], 1, t)) {
            for (int i = 0; i < m_order - 2; i++)
                m_nurbsknot[i] = t;
        } else {
            rc = false;
        }
    }

    if (end == 1 || end == 2) {
        const int i0 = m_cv_count - m_order;
        const double t = m_nurbsknot[m_cv_count - 1];

        if (evaluate_nurbs_de_boor(cvdim, m_order, m_cv_stride, &m_cv[i0 * m_cv_stride], &m_nurbsknot[i0], -1, t)) {
            const int kc = nurbsknot_count();

            for (int i = m_cv_count; i < kc; i++)
                m_nurbsknot[i] = t;
        } else {
            rc = false;
        }
    }

    return rc;
}

/// Compute the blossom of one span at order - 1 parameters by the de Boor recurrence.
static bool evaluate_nurbs_blossom(int cvdim, int order, int cv_stride, const double* CV, const double* nurbsknot, const double* t, double* P) {

    if (!CV || !t || !nurbsknot)
        return false;

    if (cv_stride < cvdim)
        return false;

    const int degree = order - 1;

    for (int i = 1; i < 2 * degree; i++)
        if (nurbsknot[i] - nurbsknot[i - 1] < 0.0)
            return false;

    if (nurbsknot[degree] - nurbsknot[degree - 1] < Tolerance::ZERO_TOLERANCE)
        return false;

    std::vector<double> space(order);

    for (int i = 0; i < cvdim; i++) {
        const double* cv = CV + i;

        for (int j = 0; j < order; j++) {
            space[j] = *cv;
            cv += cv_stride;
        }

        for (int j = 1; j < order; j++) {
            for (int k = j; k < order; k++) {
                const double denom = nurbsknot[degree + k - j] - nurbsknot[k - 1];
                space[k - j] = (nurbsknot[degree + k - j] - t[j - 1]) / denom * space[k - j] + (t[j - 1] - nurbsknot[k - 1]) / denom * space[k - j + 1];
            }
        }

        P[i] = space[0];
    }

    return true;
}

/// Compute one CV of the degree-raised span as the average of blossoms.
static bool get_raised_degree_cv(int old_order, int cvdim, int old_cv_stride, const double* oldCV, const double* oldkn, const double* newkn, int cv_id, double* newCV) {

    if (!oldCV || !oldkn || !newkn || !newCV || cv_id < 0 || cv_id > old_order)
        return false;

    const int old_degree = old_order - 1;
    const int new_degree = old_degree + 1;
    std::vector<double> t(old_degree);
    std::vector<double> P(cvdim);
    memset(newCV, 0, cvdim * sizeof(double));

    const double* kn = newkn + cv_id;

    for (int i = 0; i < new_degree; i++) {
        int k = 0;

        for (int j = 0; j < new_degree; j++) {
            if (j != i) {
                t[k] = kn[j];
                k++;
            }
        }

        if (!evaluate_nurbs_blossom(cvdim, old_order, old_cv_stride, oldCV, oldkn, t.data(), P.data()))
            return false;

        for (k = 0; k < cvdim; k++)
            newCV[k] += P[k];
    }

    for (int i = 0; i < cvdim; i++)
        newCV[i] /= static_cast<double>(new_degree);

    return true;
}

/// Return the next span index past degenerate spans.
static int next_span_index(int order, int cv_count, const double* nurbsknot, int span_index) {

    if (span_index < 0 || span_index > cv_count - order || !nurbsknot)
        return -1;

    if (span_index < cv_count - order) {
        span_index++;

        while (span_index < cv_count - order && nurbsknot[span_index + order - 2] == nurbsknot[span_index + order - 1])
            span_index++;
    }

    return span_index;
}

/// Raise the degree of N by one.
static bool increment_nurbs_degree(NurbsCurve& N) {

    const NurbsCurve M = N;
    const int sc = M.span_count();
    const int new_kcount = M.nurbsknot_count() + sc + 1;
    const int new_order = M.order() + 1;
    const int new_cv_count = new_kcount - new_order + 2;
    const int cvdim = M.cv_size();
    N.m_order = new_order;
    N.m_cv_count = new_cv_count;
    N.m_nurbsknot.resize(new_order + new_cv_count - 2);
    N.m_cv.assign(new_cv_count * N.m_cv_stride, 0.0);

    int ki = 0;
    int ko = 0;
    const int mkc = M.nurbsknot_count();

    while (ki < mkc) {
        const double kn = M.m_nurbsknot[ki];
        int mult = 1;

        while (ki + mult < mkc && std::abs(M.m_nurbsknot[ki + mult] - kn) < Tolerance::ZERO_TOLERANCE)
            mult++;

        for (int j = 0; j <= mult; j++)
            N.m_nurbsknot[ko++] = kn;

        ki += mult;
    }

    int siN = 0;
    int siM = 0;

    for (int i = 0; i < sc; i++) {
        const double* nurbsknotN = &N.m_nurbsknot[siN];
        const double* nurbsknotM = &M.m_nurbsknot[siM];
        const double* cvM = &M.m_cv[siM * M.m_cv_stride];
        const int span_mult = N.nurbsknot_multiplicity(siN + N.degree() - 1);
        const int skip = N.order() - span_mult;

        for (int j = skip; j < N.order(); j++)
            get_raised_degree_cv(M.order(), cvdim, M.m_cv_stride, cvM, nurbsknotM, nurbsknotN, j, &N.m_cv[(siN + j) * N.m_cv_stride]);

        siN = next_span_index(N.order(), N.cv_count(), N.m_nurbsknot.data(), siN);
        siM = next_span_index(M.order(), M.cv_count(), M.m_nurbsknot.data(), siM);
    }

    for (int i = 0; i < cvdim; i++) {
        N.m_cv[i] = M.m_cv[i];
        N.m_cv[(N.cv_count() - 1) * N.m_cv_stride + i] = M.m_cv[(M.cv_count() - 1) * M.m_cv_stride + i];
    }

    return true;
}

bool NurbsCurve::increase_degree(int desired_degree) {

    if (!is_valid())
        return false;

    if (desired_degree < 1 || desired_degree < degree())
        return false;

    if (desired_degree == degree())
        return true;

    if (!clamp_end(2))
        return false;

    const int del = desired_degree - degree();

    for (int i = 0; i < del; i++)
        if (!increment_nurbs_degree(*this))
            return false;

    return true;
}

bool NurbsCurve::change_closed_curve_seam(double t) {

    if (!is_valid())
        return false;

    if (!is_closed())
        return false;

    const double t0 = domain_start();
    const double t1 = domain_end();
    const double dom_len = t1 - t0;
    double s = (t - t0) / dom_len;

    if (s < 0.0 || s > 1.0) {
        s = fmod(s, 1.0);

        if (s < 0.0)
            s += 1.0;

        t = t0 + s * dom_len;
    }

    if (std::abs(t - t0) < Tolerance::ZERO_TOLERANCE || std::abs(t - t1) < Tolerance::ZERO_TOLERANCE)
        return true;

    if (t <= t0 || t >= t1)
        return true;

    const int p = degree();

    if (is_periodic()) {
        const int kc = nurbsknot_count();

        if (span_count() + 2 * p > kc) {
            int nurbsknot_index = first_nurbsknot_above(t);

            if (nurbsknot_index >= p && nurbsknot_index <= kc - p) {
                nurbsknot_index = seam_nurbsknot_index(t, nurbsknot_index);

                if (nurbsknot_index < 0)
                    return false;

                if (nurbsknot_index >= p && nurbsknot_index < nurbsknot_count() - p)
                    return rotate_periodic_seam(nurbsknot_index, t, dom_len);
            }
        }
    }

    return split_seam(t, dom_len);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json NurbsCurve::jsondump() const {

    nlohmann::ordered_json j;
    nlohmann::json cps = nlohmann::json::array();

    for (int i = 0; i < m_cv_count; i++) {
        if (m_is_rat) {
            const std::tuple<double, double, double, double> xyzw = get_cv_4d(i);
            cps.push_back({std::get<0>(xyzw), std::get<1>(xyzw), std::get<2>(xyzw), std::get<3>(xyzw)});
        } else {
            const Point p = get_cv(i);
            cps.push_back({p[0], p[1], p[2]});
        }
    }

    nlohmann::ordered_json linecolors_arr = nlohmann::ordered_json::array();

    for (const Color& c : linecolors) {
        linecolors_arr.push_back(c.r);
        linecolors_arr.push_back(c.g);
        linecolors_arr.push_back(c.b);
        linecolors_arr.push_back(c.a);
    }

    nlohmann::ordered_json pointcolors_arr = nlohmann::ordered_json::array();

    for (const Color& c : pointcolors) {
        pointcolors_arr.push_back(c.r);
        pointcolors_arr.push_back(c.g);
        pointcolors_arr.push_back(c.b);
        pointcolors_arr.push_back(c.a);
    }

    j["control_points"] = cps;
    j["cv_count"] = m_cv_count;
    j["cv_stride"] = m_cv_stride;
    j["dimension"] = m_dim;
    j["guid"] = guid();
    j["is_rational"] = m_is_rat != 0;
    j["linecolors"] = linecolors_arr;
    j["name"] = name;
    j["nurbsknots"] = m_nurbsknot;
    j["order"] = m_order;
    j["pointcolors"] = pointcolors_arr;
    j["type"] = "NurbsCurve";
    j["width"] = width;

    return j;
}

NurbsCurve NurbsCurve::jsonload(const nlohmann::json& data) {

    NurbsCurve curve;

    if (!data.contains("dimension") || !data.contains("order") || !data.contains("cv_count"))
        return curve;

    const int dim = data["dimension"];
    const bool is_rat = data.value("is_rational", false);
    const int order = data["order"];
    const int cv_count = data["cv_count"];
    curve.create(dim, is_rat, order, cv_count);

    if (data.contains("nurbsknots"))
        curve.m_nurbsknot = data["nurbsknots"].get<std::vector<double>>();

    if (data.contains("control_points")) {
        const nlohmann::json& cps = data["control_points"];
        const int n = std::min(cv_count, static_cast<int>(cps.size()));

        for (int i = 0; i < n; i++) {
            const double x = cps[i][0];
            const double y = cps[i][1];
            const double z = (cps[i].size() > 2) ? cps[i][2].get<double>() : 0.0;

            if (is_rat && cps[i].size() > 3)
                curve.set_cv_4d(i, x, y, z, cps[i][3].get<double>());
            else
                curve.set_cv(i, Point(x, y, z));
        }
    }

    curve.guid() = data.value("guid", ::guid());
    curve.name = data.value("name", "my_nurbscurve");
    curve.width = data.value("width", 1.0);

    if (data.contains("pointcolors") && data["pointcolors"].is_array()) {
        const nlohmann::json& arr = data["pointcolors"];

        for (size_t i = 0; i + 3 < arr.size(); i += 4)
            curve.pointcolors.push_back(Color(arr[i].get<float>(), arr[i + 1].get<float>(), arr[i + 2].get<float>(), arr[i + 3].get<float>()));
    }

    if (data.contains("linecolors") && data["linecolors"].is_array()) {
        const nlohmann::json& arr = data["linecolors"];

        for (size_t i = 0; i + 3 < arr.size(); i += 4)
            curve.linecolors.push_back(Color(arr[i].get<float>(), arr[i + 1].get<float>(), arr[i + 2].get<float>(), arr[i + 3].get<float>()));
    }

    return curve;
}

std::string NurbsCurve::file_json_dumps() const { return jsondump().dump(); }

NurbsCurve NurbsCurve::file_json_loads(const std::string& json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void NurbsCurve::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

NurbsCurve NurbsCurve::file_json_load(const std::string& filename) {

    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;

    return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::NurbsCurve NurbsCurve::to_proto() const {

    session_proto::NurbsCurve proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_dimension(m_dim);
    proto.set_is_rational(m_is_rat != 0);
    proto.set_order(m_order);
    proto.set_cv_count(m_cv_count);
    proto.set_cv_stride(m_cv_stride);

    for (double k : m_nurbsknot)
        proto.add_nurbsknots(k);

    for (double c : m_cv)
        proto.add_cvs(c);

    proto.set_width(width);

    for (const Color& c : pointcolors) {
        session_proto::Color* cp = proto.add_pointcolors();
        cp->set_r(c.r);
        cp->set_g(c.g);
        cp->set_b(c.b);
        cp->set_a(c.a);
    }

    for (const Color& c : linecolors) {
        session_proto::Color* cp = proto.add_linecolors();
        cp->set_r(c.r);
        cp->set_g(c.g);
        cp->set_b(c.b);
        cp->set_a(c.a);
    }

    return proto;
}

NurbsCurve NurbsCurve::from_proto(const session_proto::NurbsCurve& proto) {

    NurbsCurve curve(proto.dimension(), proto.is_rational(), proto.order(), proto.cv_count());

    if (!proto.guid().empty())
        curve.guid() = proto.guid();

    curve.name = proto.name();
    curve.width = proto.width() != 0.0 ? proto.width() : 1.0;
    curve.m_nurbsknot.clear();

    for (int i = 0; i < proto.nurbsknots_size(); ++i)
        curve.m_nurbsknot.push_back(proto.nurbsknots(i));

    curve.m_cv.clear();

    for (int i = 0; i < proto.cvs_size(); ++i)
        curve.m_cv.push_back(proto.cvs(i));

    for (int i = 0; i < proto.pointcolors_size(); ++i) {
        const session_proto::Color& c = proto.pointcolors(i);
        curve.pointcolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }

    for (int i = 0; i < proto.linecolors_size(); ++i) {
        const session_proto::Color& c = proto.linecolors(i);
        curve.linecolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }

    return curve;
}

std::string NurbsCurve::pb_dumps() const { return to_proto().SerializeAsString(); }

NurbsCurve NurbsCurve::pb_loads(const std::string& data) {

    session_proto::NurbsCurve proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse NurbsCurve protobuf data");

    return from_proto(proto);
}

void NurbsCurve::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

NurbsCurve NurbsCurve::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string NurbsCurve::str() const { return fmt::format("NurbsCurve(name={}, degree={}, cvs={})", name, degree(), cv_count()); }

std::string NurbsCurve::repr() const {

    const int prec = static_cast<int>(Tolerance::ROUNDING);
    std::string result = fmt::format("NurbsCurve(\n  name={},\n  degree={},\n  cvs={},\n  rational={},\n  control_points=[\n", name, degree(), m_cv_count, m_is_rat ? "true" : "false");

    for (int i = 0; i < m_cv_count; ++i) {
        const Point p = get_cv(i);
        result += fmt::format("    {}, {}, {}\n", TOLERANCE.format_number(p[0], prec), TOLERANCE.format_number(p[1], prec), TOLERANCE.format_number(p[2], prec));
    }

    result += "  ]\n)";

    return result;
}

std::ostream& operator<<(std::ostream& os, const NurbsCurve& curve) {

    os << curve.str();

    return os;
}

// ═══════════════════════════════════════════════════════════════════════════
// Private helpers
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsCurve::span_is_linear(int span_index, double min_length, double tolerance) const {

    if (!is_valid())
        return false;

    if (span_index < 0 || span_index >= m_cv_count - m_order)
        return false;

    if (m_dim < 2 || m_dim > 3)
        return false;

    const int ki = span_index + m_order - 2;

    if (m_nurbsknot[ki] >= m_nurbsknot[ki + 1])
        return false;

    int mult_start = 1;

    for (int i = ki - 1; i >= 0 && m_nurbsknot[i] == m_nurbsknot[ki]; i--)
        mult_start++;

    int mult_end = 1;
    const int kc = nurbsknot_count();

    for (int i = ki + 2; i < kc && m_nurbsknot[i] == m_nurbsknot[ki + 1]; i++)
        mult_end++;

    if (mult_start < m_order - 1 || mult_end < m_order - 1)
        return false;

    const Point p0 = get_cv(span_index);
    const Point p1 = get_cv(span_index + m_order - 1);
    const Vector line_vec = p1 - p0;
    const double line_length = line_vec.magnitude();

    if (line_length < min_length)
        return false;

    for (int i = 1; i < m_order - 1; i++) {
        const Point p = get_cv(span_index + i);
        const Vector v = p - p0;

        if (line_vec.cross(v).magnitude() / line_length > tolerance)
            return false;

        const double t = v.dot(line_vec) / (line_length * line_length);

        if (t < -0.01 || t > 1.01)
            return false;
    }

    return true;
}

bool NurbsCurve::span_is_singular(int span_index) const {

    if (!is_valid())
        return false;

    if (span_index < 0 || span_index >= m_cv_count - m_order)
        return false;

    const int ki = span_index + m_order - 2;

    if (m_nurbsknot[ki] >= m_nurbsknot[ki + 1])
        return true;

    const Point p0 = get_cv(span_index);

    for (int i = 1; i < m_order; i++)
        if (p0.distance(get_cv(span_index + i)) > Tolerance::ZERO_TOLERANCE)
            return false;

    return true;
}

int NurbsCurve::find_span(double t) const {

    const double* nurbsknot = m_nurbsknot.data() + (m_order - 2);
    const int len = m_cv_count - m_order + 2;

    if (t <= nurbsknot[0])
        return 0;

    if (t >= nurbsknot[len - 1])
        return len - 2;

    int low = 0;
    int high = len - 1;

    while (high > low + 1) {
        const int mid = (low + high) / 2;

        if (t < nurbsknot[mid])
            high = mid;
        else
            low = mid;
    }

    return low;
}

void NurbsCurve::basis_functions(int span, double t, std::vector<double>& basis) const {

    basis.resize(m_order);

    std::vector<double> left(m_order);
    std::vector<double> right(m_order);
    const double* nurbsknot = m_nurbsknot.data() + (m_order - 2) + span;
    basis[0] = 1.0;

    for (int j = 1; j < m_order; j++) {
        left[j] = t - nurbsknot[1 - j];
        right[j] = nurbsknot[j] - t;

        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            const double denom = right[r + 1] + left[j - r];
            const double temp = (denom != 0.0) ? basis[r] / denom : 0.0;
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        basis[j] = saved;
    }
}

void NurbsCurve::basis_functions_derivatives(int span, double t, int deriv_order, std::vector<std::vector<double>>& ders) const {

    const int p = degree();
    const int n_der = std::min(deriv_order, p);
    ders.assign(n_der + 1, std::vector<double>(p + 1, 0.0));

    std::vector<std::vector<double>> ndu;
    basis_functions_ndu(span, t, ndu);

    for (int j = 0; j <= p; ++j)
        ders[0][j] = ndu[j][p];

    std::vector<std::vector<double>> a(2, std::vector<double>(p + 1, 0.0));

    for (int r = 0; r <= p; ++r) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;

        for (int k = 1; k <= n_der; ++k) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = p - k;

            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            const int j1 = (rk >= -1) ? 1 : -rk;
            const int j2 = (r - 1 <= pk) ? k - 1 : p - r;

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

    double scale = static_cast<double>(p);

    for (int k = 1; k <= n_der; ++k) {
        for (int j = 0; j <= p; ++j)
            ders[k][j] *= scale;

        scale *= static_cast<double>(p - k);
    }
}

void NurbsCurve::basis_functions_ndu(int span, double t, std::vector<std::vector<double>>& ndu) const {

    const int p = degree();
    std::vector<double> left(p + 1);
    std::vector<double> right(p + 1);
    const double* nurbsknot = m_nurbsknot.data() + (m_order - 2) + span;
    ndu.assign(p + 1, std::vector<double>(p + 1, 0.0));
    ndu[0][0] = 1.0;

    for (int j = 1; j <= p; ++j) {
        left[j] = t - nurbsknot[1 - j];
        right[j] = nurbsknot[j] - t;

        double saved = 0.0;

        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        ndu[j][j] = saved;
    }
}

void NurbsCurve::deep_copy_from(const NurbsCurve& src) {

    m_dim = src.m_dim;
    m_is_rat = src.m_is_rat;
    m_order = src.m_order;
    m_cv_count = src.m_cv_count;
    m_cv_stride = src.m_cv_stride;
    m_nurbsknot = src.m_nurbsknot;
    m_cv = src.m_cv;
    _guid.clear();
    name = src.name;
    width = src.width;
    pointcolors = src.pointcolors;
    linecolors = src.linecolors;
}

bool NurbsCurve::evaluate_nurbs_de_boor(int cv_dim, int order, int cv_stride, double* cv, const double* nurbsknots, int side, double t) {

    const int degree = order - 1;

    if (nurbsknots[degree - 1] == nurbsknots[degree])
        return false;

    if (side < 0)
        return de_boor_end(cv_dim, order, cv_stride, cv, nurbsknots, t);

    return de_boor_start(cv_dim, order, cv_stride, cv, nurbsknots, t);
}

bool NurbsCurve::de_boor_end(int cv_dim, int order, int cv_stride, double* cv, const double* nurbsknots, double t) {

    const int degree = order - 1;
    const double t0 = nurbsknots[degree - 1];
    const double t1 = nurbsknots[degree];

    if (t == t1 && t1 == nurbsknots[2 * degree - 1])
        return true;

    const bool fully_multiple = (t0 == nurbsknots[0]);
    const int kn = degree - 1;
    std::vector<double> delta_t(degree, 0.0);

    if (!fully_multiple)
        for (int idx = 0; idx < degree; idx++)
            delta_t[idx] = t - nurbsknots[kn - idx];

    for (int k = order - 1; k >= 1; k--) {
        for (int i = k - 1; i >= 0; i--) {
            const int di = k - 1 - i;
            const double alpha1 = fully_multiple ? (t - t0) / (nurbsknots[kn + k - di] - t0) : delta_t[di] / (nurbsknots[kn + k - di] - nurbsknots[kn - di]);
            const double alpha0 = 1.0 - alpha1;
            const int row1 = (order - k + i) * cv_stride;
            const int row0 = row1 - cv_stride;

            for (int j = 0; j < cv_dim; j++)
                cv[row1 + j] = cv[row0 + j] * alpha0 + cv[row1 + j] * alpha1;
        }
    }

    return true;
}

bool NurbsCurve::de_boor_start(int cv_dim, int order, int cv_stride, double* cv, const double* nurbsknots, double t) {

    const int degree = order - 1;
    const double t0 = nurbsknots[degree - 1];
    const double t1 = nurbsknots[degree];

    if (t == t0 && t0 == nurbsknots[0])
        return true;

    const bool fully_multiple = (t1 == nurbsknots[2 * degree - 1]);
    const int kn = degree;
    std::vector<double> delta_t(degree, 0.0);

    if (!fully_multiple)
        for (int idx = 0; idx < degree; idx++)
            delta_t[idx] = nurbsknots[kn + idx] - t;

    for (int k = order - 1; k >= 1; k--) {
        for (int i = 0; i < k; i++) {
            const double alpha0 = fully_multiple ? (t1 - t) / (t1 - nurbsknots[kn - k + i]) : delta_t[i] / (nurbsknots[kn + i] - nurbsknots[kn - k + i]);
            const double alpha1 = 1.0 - alpha0;
            const int row0 = i * cv_stride;
            const int row1 = row0 + cv_stride;

            for (int j = 0; j < cv_dim; j++)
                cv[row0 + j] = cv[row0 + j] * alpha0 + cv[row1 + j] * alpha1;
        }
    }

    return true;
}

bool NurbsCurve::solve_dense(std::vector<std::vector<double>>& matrix, std::vector<double>& rhs, int n, int dim) {

    for (int col = 0; col < n; col++) {
        int pivot = col;

        for (int row = col + 1; row < n; row++)
            if (std::fabs(matrix[row][col]) > std::fabs(matrix[pivot][col]))
                pivot = row;

        if (pivot != col) {
            std::swap(matrix[col], matrix[pivot]);

            for (int d = 0; d < dim; d++)
                std::swap(rhs[col * dim + d], rhs[pivot * dim + d]);
        }

        if (std::fabs(matrix[col][col]) < 1e-300)
            return false;

        for (int row = col + 1; row < n; row++) {
            const double factor = matrix[row][col] / matrix[col][col];

            for (int j = col; j < n; j++)
                matrix[row][j] -= factor * matrix[col][j];

            for (int d = 0; d < dim; d++)
                rhs[row * dim + d] -= factor * rhs[col * dim + d];
        }
    }

    for (int i = n - 1; i >= 0; i--) {
        for (int d = 0; d < dim; d++) {
            double sum = rhs[i * dim + d];

            for (int j = i + 1; j < n; j++)
                sum -= matrix[i][j] * rhs[j * dim + d];

            rhs[i * dim + d] = sum / matrix[i][i];
        }
    }

    return true;
}

Vector NurbsCurve::derivative_at(double t, double h) const {

    const double t0 = domain_start();
    const double t1 = domain_end();
    Point p1;
    Point p2;
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

    return (p2 - p1) / dt;
}

double NurbsCurve::arc_length_gauss(double ta, double tb, double h) const {

    static const double GL_NODES[5] = {-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640};
    static const double GL_WEIGHTS[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};

    const double mid = (ta + tb) * 0.5;
    const double half = (tb - ta) * 0.5;
    double sum = 0.0;

    for (int i = 0; i < 5; i++)
        sum += GL_WEIGHTS[i] * derivative_at(mid + half * GL_NODES[i], h).magnitude();

    return half * sum;
}

double NurbsCurve::find_t_at_s(double s_target, const std::vector<double>& t_vals, const std::vector<double>& s_vals, double h) const {

    const int n_samples = static_cast<int>(t_vals.size()) - 1;

    if (s_target <= 0.0)
        return t_vals[0];

    if (s_target >= s_vals[n_samples])
        return t_vals[n_samples];

    int lo = 0;
    int hi = n_samples;

    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;

        if (s_vals[mid] < s_target)
            lo = mid;
        else
            hi = mid;
    }

    const double frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
    double t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);
    double t_lo = t_vals[lo];
    double t_hi = t_vals[hi];

    for (int iter = 0; iter < 20; iter++) {
        const double error = s_vals[lo] + arc_length_gauss(t_vals[lo], t, h) - s_target;

        if (std::abs(error) < 1e-12)
            break;

        const double speed = derivative_at(t, h).magnitude();
        const double t_new = t - error / speed;

        if (speed < 1e-14 || t_new <= t_lo || t_new >= t_hi) {
            if (error > 0)
                t_hi = t;
            else
                t_lo = t;

            t = (t_lo + t_hi) * 0.5;
        } else {
            t = t_new;
        }
    }

    return t;
}

Plane NurbsCurve::frenet_frame(const Point& origin, const Vector& d1, const Vector& d2) {

    if (d1.magnitude() < 1e-14)
        return Plane::invalid();

    Vector T = d1;
    T.normalize_self();

    const double d2_dot_T = d2.dot(T);
    Vector N = d2 - T * d2_dot_T;
    double n_mag = N.magnitude();

    if (n_mag < 1e-14) {
        N = T.cross(Vector(0, 0, 1));
        n_mag = N.magnitude();

        if (n_mag < 1e-14) {
            N = T.cross(Vector(0, 1, 0));
            n_mag = N.magnitude();
        }
    }

    if (n_mag > 1e-14)
        N.normalize_self();

    Vector B = T.cross(N);
    B.normalize_self();

    return Plane::from_frame(origin, T, N, B);
}

Vector NurbsCurve::bessel_tangent(const std::vector<Point>& points, int i0, int i1, int i2) {

    const double d01 = points[i0].distance(points[i1]);
    const double d21 = points[i2].distance(points[i1]);

    if (d01 + d21 < 1e-300)
        return Vector(0, 0, 0);

    const double s = d01 / (d01 + d21);
    const double t = 1.0 - s;
    const double denom = 2.0 * s * t;

    if (denom < 1e-16) {
        Vector chord = points[i1] - points[i0];

        return chord.normalize_self() ? chord : Vector(0, 0, 0);
    }

    const double cvx = (-t * t * points[i0][0] + points[i1][0] - s * s * points[i2][0]) / denom;
    const double cvy = (-t * t * points[i0][1] + points[i1][1] - s * s * points[i2][1]) / denom;
    const double cvz = (-t * t * points[i0][2] + points[i1][2] - s * s * points[i2][2]) / denom;
    Vector tangent = Point(cvx, cvy, cvz) - points[i0];

    return tangent.normalize_self() ? tangent : Vector(0, 0, 0);
}

Vector NurbsCurve::lagrange_tangent(const std::vector<Point>& points, const std::vector<double>& params, int i0, int m, double t) {

    Vector result(0, 0, 0);

    for (int j = 0; j < m; j++) {
        const double uj = params[i0 + j];
        double dsum = 0.0;

        for (int i = 0; i < m; i++) {
            if (i == j)
                continue;

            double term = 1.0 / (uj - params[i0 + i]);

            for (int k = 0; k < m; k++) {
                if (k == j || k == i)
                    continue;

                term *= (t - params[i0 + k]) / (uj - params[i0 + k]);
            }

            dsum += term;
        }

        const Point& Pj = points[i0 + j];
        result += Vector(Pj[0], Pj[1], Pj[2]) * dsum;
    }

    return result;
}

NurbsCurve NurbsCurve::create_interpolated_periodic(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization) {

    const int n = static_cast<int>(points.size());
    const int dim = 3;
    const int order = 4;
    const int cv_count = n + 3;
    const int kc = cv_count + order - 2;
    const std::vector<double> params = periodic_interpolation_parameters(points, parameterization);
    double dmin = 1e300;
    double dmax = 0.0;

    for (int i = 0; i < n; i++) {
        const double d = params[i + 1] - params[i];

        if (d < dmin)
            dmin = d;

        if (d > dmax)
            dmax = d;
    }

    if (dmax <= 0.0 || dmax * SQRT_EPSILON >= dmin)
        return NurbsCurve();

    const std::vector<double> nurbsknots = periodic_interpolation_nurbsknots(params, cv_count);
    std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));
    std::vector<double> cv(n * dim);

    for (int i = 0; i < n; i++) {
        const std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, i, params[i]);
        A[i][i % n] += basis[0];
        A[i][(i + 1) % n] += basis[1];
        A[i][(i + 2) % n] += basis[2];

        for (int d = 0; d < dim; d++)
            cv[i * dim + d] = points[i][d];
    }

    if (!solve_dense(A, cv, n, dim))
        return NurbsCurve();

    NurbsCurve curve(dim, false, order, cv_count);

    for (int i = 0; i < kc; i++)
        curve.set_nurbsknot(i, nurbsknots[i]);

    for (int i = 0; i < n; i++)
        curve.set_cv(i, Point(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));

    curve.set_cv(n, curve.get_cv(0));
    curve.set_cv(n + 1, curve.get_cv(1));
    curve.set_cv(n + 2, curve.get_cv(2));

    return curve;
}

std::vector<double> NurbsCurve::periodic_interpolation_parameters(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization) {

    const int n = static_cast<int>(points.size());
    CurveNurbsKnotStyle base_style = CurveNurbsKnotStyle::Chord;

    if (parameterization == CurveNurbsKnotStyle::UniformPeriodic)
        base_style = CurveNurbsKnotStyle::Uniform;

    if (parameterization == CurveNurbsKnotStyle::ChordSquareRootPeriodic)
        base_style = CurveNurbsKnotStyle::ChordSquareRoot;

    std::vector<double> params(n + 1, 0.0);

    if (base_style == CurveNurbsKnotStyle::Uniform) {
        for (int i = 1; i <= n; i++)
            params[i] = static_cast<double>(i);

        return params;
    }

    for (int i = 1; i < n; i++) {
        double d = points[i - 1].distance(points[i]);

        if (base_style == CurveNurbsKnotStyle::ChordSquareRoot)
            d = std::sqrt(d);

        params[i] = params[i - 1] + d;
    }

    double d_close = points[n - 1].distance(points[0]);

    if (base_style == CurveNurbsKnotStyle::ChordSquareRoot)
        d_close = std::sqrt(d_close);

    params[n] = params[n - 1] + d_close;

    return params;
}

std::vector<double> NurbsCurve::periodic_interpolation_nurbsknots(const std::vector<double>& params, int cv_count) {

    const int n = static_cast<int>(params.size()) - 1;
    std::vector<double> nurbsknots(cv_count + 2);

    for (int i = 0; i <= n; i++)
        nurbsknots[i + 2] = params[i];

    nurbsknots[cv_count] = nurbsknots[3] - nurbsknots[2] + nurbsknots[cv_count - 1];
    nurbsknots[1] = nurbsknots[cv_count - 2] - nurbsknots[cv_count - 1] + nurbsknots[2];
    nurbsknots[cv_count + 1] = nurbsknots[4] - nurbsknots[3] + nurbsknots[cv_count];
    nurbsknots[0] = nurbsknots[cv_count - 3] - nurbsknots[cv_count - 2] + nurbsknots[1];

    return nurbsknots;
}

NurbsCurve NurbsCurve::create_interpolated_clamped(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization, CurveInterpStyle end_condition) {

    const int n = static_cast<int>(points.size());
    const int dim = 3;
    const int degree = 3;
    const int cv_count = n + 2;
    const std::vector<double> pts = flatten_points(points, n);
    const std::vector<double> params = nurbsknot::compute_parameters(pts.data(), n, dim, parameterization);
    const std::vector<double> nurbsknots = nurbsknot::build_interp_nurbsknots(params, degree);
    const int kc = static_cast<int>(nurbsknots.size());
    std::vector<double> cv = interpolation_end_cvs(points, params, end_condition);

    if (!solve_interpolation_cvs(points, params, nurbsknots, cv))
        return NurbsCurve();

    NurbsCurve curve(dim, false, degree + 1, cv_count);

    for (int i = 0; i < kc; i++)
        curve.set_nurbsknot(i, nurbsknots[i]);

    for (int i = 0; i < cv_count; i++)
        curve.set_cv(i, Point(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));

    return curve;
}

std::vector<double> NurbsCurve::interpolation_end_cvs(const std::vector<Point>& points, const std::vector<double>& params, CurveInterpStyle end_condition) {

    const int n = static_cast<int>(points.size());
    const int dim = 3;
    Vector tan_start;
    Vector tan_end;
    double s0;
    double s1;

    if (end_condition == CurveInterpStyle::Occt) {
        const int deg_t = (n == 3) ? 2 : 3;
        tan_start = lagrange_tangent(points, params, 0, deg_t + 1, params[0]);
        tan_end = lagrange_tangent(points, params, n - 1 - deg_t, deg_t + 1, params[n - 1]);
        s0 = (params[1] - params[0]) / 3.0;
        s1 = -(params[n - 1] - params[n - 2]) / 3.0;
    } else {
        tan_start = bessel_tangent(points, 0, 1, 2);

        const Vector end_raw = bessel_tangent(points, n - 1, n - 2, n - 3);
        tan_end = -end_raw;
        s0 = points[0].distance(points[1]) / 3.0;
        s1 = -points[n - 1].distance(points[n - 2]) / 3.0;
    }

    std::vector<double> cv((n + 2) * dim);

    for (int d = 0; d < dim; d++)
        cv[d] = points[0][d];

    for (int d = 0; d < dim; d++)
        cv[dim + d] = points[0][d] + s0 * tan_start[d];

    for (int i = 1; i <= n - 2; i++)
        for (int d = 0; d < dim; d++)
            cv[(i + 1) * dim + d] = points[i][d];

    for (int d = 0; d < dim; d++)
        cv[n * dim + d] = points[n - 1][d] + s1 * tan_end[d];

    for (int d = 0; d < dim; d++)
        cv[(n + 1) * dim + d] = points[n - 1][d];

    return cv;
}

bool NurbsCurve::solve_interpolation_cvs(const std::vector<Point>& points, const std::vector<double>& params, const std::vector<double>& nurbsknots, std::vector<double>& cv) {

    const int n = static_cast<int>(points.size());
    const int dim = 3;
    const int order = 4;
    const int sys_n = n;
    std::vector<double> lower(sys_n, 0.0);
    std::vector<double> diag(sys_n, 0.0);
    std::vector<double> upper(sys_n, 0.0);
    std::vector<double> rhs(sys_n * dim);
    diag[0] = 1.0;

    for (int d = 0; d < dim; d++)
        rhs[d] = cv[dim + d];

    for (int i = 1; i <= n - 2; i++) {
        const std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, i, params[i]);
        lower[i] = basis[0];
        diag[i] = basis[1];
        upper[i] = basis[2];

        for (int d = 0; d < dim; d++)
            rhs[i * dim + d] = points[i][d];
    }

    diag[n - 1] = 1.0;

    for (int d = 0; d < dim; d++)
        rhs[(n - 1) * dim + d] = cv[n * dim + d];

    std::vector<double> solution;

    if (!nurbsknot::solve_tridiagonal(dim, sys_n, lower, diag, upper, rhs, solution))
        return false;

    for (int i = 0; i < sys_n; i++)
        for (int d = 0; d < dim; d++)
            cv[(i + 1) * dim + d] = solution[i * dim + d];

    return true;
}

std::vector<double> NurbsCurve::flatten_points(const std::vector<Point>& points, int count) {

    std::vector<double> flat(count * 3);

    for (int i = 0; i < count; i++) {
        flat[i * 3] = points[i][0];
        flat[i * 3 + 1] = points[i][1];
        flat[i * 3 + 2] = points[i][2];
    }

    return flat;
}

NurbsCurve NurbsCurve::create_fitted_periodic(const std::vector<Point>& points, int num_cvs, int degree) {

    const int dim = 3;
    const int order = degree + 1;
    int n = static_cast<int>(points.size());

    if (n >= 2 && points[0].distance(points[n - 1]) < 1e-10)
        n--;

    if (n <= num_cvs || num_cvs < order)
        return n < 3 ? NurbsCurve() : create_interpolated(std::vector<Point>(points.begin(), points.begin() + n), CurveNurbsKnotStyle::ChordPeriodic);

    const int cv_count = num_cvs + degree;
    const int kc = cv_count + order - 2;
    std::vector<double> params(n + 1, 0.0);

    for (int i = 1; i < n; i++)
        params[i] = params[i - 1] + points[i - 1].distance(points[i]);

    params[n] = params[n - 1] + points[n - 1].distance(points[0]);

    if (params[n] < 1e-14)
        return NurbsCurve();

    const std::vector<double> ppts = flatten_points(points, n);
    const std::vector<double> nurbsknots = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(params, ppts.data(), n, dim, num_cvs, degree);
    std::vector<std::vector<double>> NtN(num_cvs, std::vector<double>(num_cvs, 0.0));
    std::vector<double> cv(num_cvs * dim, 0.0);

    for (int k = 0; k < n; k++) {
        const int span = nurbsknot::find_span(order, cv_count, nurbsknots, params[k]);
        const std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, span, params[k]);

        for (int a = 0; a < order; a++) {
            const int ci = (span + a) % num_cvs;

            for (int d = 0; d < dim; d++)
                cv[ci * dim + d] += basis[a] * points[k][d];

            for (int b = 0; b < order; b++)
                NtN[ci][(span + b) % num_cvs] += basis[a] * basis[b];
        }
    }

    if (!solve_dense(NtN, cv, num_cvs, dim))
        return NurbsCurve();

    NurbsCurve curve(dim, false, order, cv_count);

    for (int i = 0; i < kc; i++)
        curve.set_nurbsknot(i, nurbsknots[i]);

    for (int i = 0; i < num_cvs; i++)
        curve.set_cv(i, Point(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));

    for (int i = 0; i < degree; i++)
        curve.set_cv(num_cvs + i, curve.get_cv(i));

    return curve;
}

NurbsCurve NurbsCurve::create_fitted_clamped(const std::vector<Point>& points, int num_cvs, int degree) {

    const int m = static_cast<int>(points.size());
    const int dim = 3;

    if (m <= num_cvs || num_cvs < degree + 1)
        return create_interpolated(points);

    const std::vector<double> pts = flatten_points(points, m);
    const std::vector<double> params = nurbsknot::compute_parameters(pts.data(), m, dim, CurveNurbsKnotStyle::Chord);
    const std::vector<double> nurbsknots = nurbsknot::build_fitted_nurbsknots_adaptive(params, pts.data(), m, dim, num_cvs, degree);
    const int sys_n = num_cvs - 2;
    std::vector<double> band(sys_n * (degree + 1), 0.0);
    std::vector<double> rhs(sys_n * dim, 0.0);
    fitted_band_system(points, params, nurbsknots, num_cvs, degree, band, rhs);

    if (!nurbsknot::solve_banded_spd(dim, sys_n, degree, band, rhs))
        return create_interpolated(points);

    const int kc = static_cast<int>(nurbsknots.size());
    NurbsCurve curve(dim, false, degree + 1, num_cvs);

    for (int i = 0; i < kc; i++)
        curve.set_nurbsknot(i, nurbsknots[i]);

    curve.set_cv(0, points[0]);

    for (int i = 0; i < sys_n; i++)
        curve.set_cv(i + 1, Point(rhs[i * 3], rhs[i * 3 + 1], rhs[i * 3 + 2]));

    curve.set_cv(num_cvs - 1, points[m - 1]);

    return curve;
}

void NurbsCurve::fitted_band_system(const std::vector<Point>& points, const std::vector<double>& params, const std::vector<double>& nurbsknots, int num_cvs, int degree, std::vector<double>& band, std::vector<double>& rhs) {

    const int m = static_cast<int>(points.size());
    const int dim = 3;
    const int order = degree + 1;
    const int n = num_cvs - 1;
    const int bw1 = degree + 1;

    for (int k = 1; k < m - 1; k++) {
        const int span = nurbsknot::find_span(order, num_cvs, nurbsknots, params[k]);
        const std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, span, params[k]);
        double rk[3];

        for (int d = 0; d < dim; d++)
            rk[d] = points[k][d];

        for (int a = 0; a < order; a++) {
            const int ci = span + a;

            if (ci == 0)
                for (int d = 0; d < dim; d++)
                    rk[d] -= basis[a] * points[0][d];

            if (ci == n)
                for (int d = 0; d < dim; d++)
                    rk[d] -= basis[a] * points[m - 1][d];
        }

        for (int a = 0; a < order; a++) {
            const int ci = span + a;

            if (ci < 1 || ci > n - 1)
                continue;

            const int ri = ci - 1;

            for (int d = 0; d < dim; d++)
                rhs[ri * dim + d] += basis[a] * rk[d];

            for (int b = a; b < order; b++) {
                const int cj = span + b;

                if (cj < 1 || cj > n - 1)
                    continue;

                const int rj = cj - 1;
                band[rj * bw1 + (rj - ri)] += basis[a] * basis[b];
            }
        }
    }
}

void NurbsCurve::promote_to_3d(std::vector<NurbsCurve>& segs) {

    bool any2 = false;
    bool any3 = false;

    for (const NurbsCurve& c : segs) {
        if (c.m_dim == 2)
            any2 = true;
        else if (c.m_dim == 3)
            any3 = true;
    }

    if (!any2 || !any3)
        return;

    for (NurbsCurve& c : segs) {
        if (c.m_dim != 2)
            continue;

        const int os = c.m_cv_stride;
        const int ns = os + 1;
        std::vector<double> cv(static_cast<size_t>(c.m_cv_count) * ns, 0.0);

        for (int i = 0; i < c.m_cv_count; ++i) {
            cv[static_cast<size_t>(i) * ns] = c.m_cv[static_cast<size_t>(i) * os];
            cv[static_cast<size_t>(i) * ns + 1] = c.m_cv[static_cast<size_t>(i) * os + 1];

            if (c.m_is_rat)
                cv[static_cast<size_t>(i) * ns + 3] = c.m_cv[static_cast<size_t>(i) * os + 2];
        }

        c.m_cv = std::move(cv);
        c.m_cv_stride = ns;
        c.m_dim = 3;
    }
}

std::vector<std::vector<NurbsCurve>> NurbsCurve::chain_segments(const std::vector<NurbsCurve>& segs, double tolerance) {

    std::vector<std::vector<NurbsCurve>> chains;
    std::vector<bool> used(segs.size(), false);

    for (size_t i = 0; i < segs.size(); i++) {
        if (used[i])
            continue;

        used[i] = true;

        std::vector<NurbsCurve> chain;
        chain.push_back(segs[i]);
        bool grown = !segs[i].is_closed();

        while (grown) {
            grown = false;
            const Point start = chain.front().point_at_start();
            const Point end = chain.back().point_at_end();

            for (size_t j = 0; j < segs.size(); j++) {
                if (used[j] || segs[j].is_closed())
                    continue;

                const Point s = segs[j].point_at_start();
                const Point e = segs[j].point_at_end();

                if (s.distance(end) <= tolerance) {
                    chain.push_back(segs[j]);
                } else if (e.distance(end) <= tolerance) {
                    NurbsCurve r = segs[j];
                    r.reverse();
                    chain.push_back(r);
                } else if (e.distance(start) <= tolerance) {
                    chain.insert(chain.begin(), segs[j]);
                } else if (s.distance(start) <= tolerance) {
                    NurbsCurve r = segs[j];
                    r.reverse();
                    chain.insert(chain.begin(), r);
                } else {
                    continue;
                }

                used[j] = true;
                grown = true;
                break;
            }
        }

        chains.push_back(chain);
    }

    return chains;
}

void NurbsCurve::join_chain(std::vector<NurbsCurve>& chain, std::vector<NurbsCurve>& result) {

    if (chain.size() == 1) {
        result.push_back(chain[0]);

        return;
    }

    bool rational = false;
    int max_degree = 1;

    for (const NurbsCurve& c : chain) {
        if (c.is_rational())
            rational = true;

        if (c.degree() > max_degree)
            max_degree = c.degree();
    }

    bool aligned = true;

    for (NurbsCurve& c : chain) {
        if (rational)
            c.to_rational();

        if (!c.clamp_end(2) || !c.increase_degree(max_degree))
            aligned = false;
    }

    NurbsCurve joined = chain[0];

    if (aligned)
        for (size_t ci = 1; ci < chain.size(); ci++)
            append_segment(joined, chain[ci], rational);

    if (!aligned || static_cast<int>(joined.m_cv.size()) < (joined.m_cv_count - 1) * joined.m_cv_stride + joined.cv_size() || static_cast<int>(joined.m_nurbsknot.size()) != joined.m_cv_count + joined.m_order - 2) {
        for (NurbsCurve& c : chain)
            result.push_back(c);

        return;
    }

    result.push_back(joined);
}

void NurbsCurve::append_segment(NurbsCurve& joined, NurbsCurve& segment, bool rational) {

    const int stride = joined.m_cv_stride;
    const int cvdim = joined.cv_size();
    const double a1 = joined.domain_end();
    const double s0 = segment.domain_start();
    const double s1 = segment.domain_end();
    segment.set_domain(a1, a1 + (s1 - s0));

    if (rational) {
        const double w_end = joined.weight(joined.m_cv_count - 1);
        const double w_start = segment.weight(0);

        if (std::fabs(w_start) > Tolerance::ZERO_TOLERANCE) {
            const double scale = w_end / w_start;

            for (size_t k = 0; k < segment.m_cv.size(); k++)
                segment.m_cv[k] = segment.m_cv[k] * scale;
        }
    }

    const int last = (joined.m_cv_count - 1) * stride;

    if (stride <= 0 || cvdim <= 0 || segment.m_order != joined.m_order || segment.m_cv_stride != stride || segment.cv_size() != cvdim || static_cast<int>(joined.m_cv.size()) < last + cvdim || static_cast<int>(segment.m_cv.size()) < segment.m_cv_count * stride || static_cast<int>(segment.m_cv.size()) <= stride || static_cast<int>(segment.m_nurbsknot.size()) != segment.m_cv_count + segment.m_order - 2)
        return;

    for (int k = 0; k < cvdim; k++)
        joined.m_cv[last + k] = 0.5 * (joined.m_cv[last + k] + segment.m_cv[k]);

    joined.m_nurbsknot.insert(joined.m_nurbsknot.end(), segment.m_nurbsknot.begin() + (joined.m_order - 1), segment.m_nurbsknot.end());
    joined.m_cv.insert(joined.m_cv.end(), segment.m_cv.begin() + stride, segment.m_cv.end());
    joined.m_cv_count = joined.m_cv_count + segment.m_cv_count - 1;
}

bool NurbsCurve::circle_center(const Point& p0, const Point& p1, const Point& p2, Point& center) {

    const Vector d1 = p1 - p0;
    const Vector d2 = p2 - p1;
    Vector normal = d1.cross(d2);

    if (normal.magnitude() < Tolerance::ZERO_TOLERANCE)
        return false;

    normal = normal.normalized();

    const Point m1 = Point::sum(p0, p1) * 0.5;
    const Point m2 = Point::sum(p1, p2) * 0.5;
    const Vector perp1 = d1.cross(normal).normalized();
    const Vector perp2 = d2.cross(normal).normalized();
    double denom = perp1[0] * perp2[1] - perp1[1] * perp2[0];

    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE)
        denom = perp1[0] * perp2[2] - perp1[2] * perp2[0];

    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE)
        return false;

    const double dx = m2[0] - m1[0];
    const double dy = m2[1] - m1[1];
    const double s = (dx * perp2[1] - dy * perp2[0]) / denom;
    center = m1 + perp1 * s;

    return true;
}

std::vector<double> NurbsCurve::full_nurbsknots() const {

    const int full_nurbsknot_count = m_cv_count + m_order;
    std::vector<double> U(full_nurbsknot_count);
    U[0] = m_nurbsknot.front();

    for (int i = 0; i < static_cast<int>(m_nurbsknot.size()); ++i)
        U[i + 1] = m_nurbsknot[i];

    U[full_nurbsknot_count - 1] = m_nurbsknot.back();

    return U;
}

void NurbsCurve::insert_nurbsknot_once(double nurbsknot_value, const std::vector<double>& U) {

    const int p = degree();
    const int n = m_cv_count - 1;
    const int full_nurbsknot_count = m_cv_count + m_order;
    const int k = find_span(nurbsknot_value) + m_order - 1;
    const int new_cv_count = m_cv_count + 1;
    std::vector<double> U_new(full_nurbsknot_count + 1);
    std::vector<double> cv_new(new_cv_count * m_cv_stride);

    for (int i = 0; i <= k; ++i)
        U_new[i] = U[i];

    U_new[k + 1] = nurbsknot_value;

    for (int i = k + 1; i < full_nurbsknot_count; ++i)
        U_new[i + 1] = U[i];

    for (int i = 0; i <= k - p; ++i)
        std::copy(&m_cv[i * m_cv_stride], &m_cv[i * m_cv_stride] + m_cv_stride, &cv_new[i * m_cv_stride]);

    for (int i = k + 1; i <= n + 1; ++i)
        std::copy(&m_cv[(i - 1) * m_cv_stride], &m_cv[(i - 1) * m_cv_stride] + m_cv_stride, &cv_new[i * m_cv_stride]);

    for (int i = k - p + 1; i <= k; ++i) {
        double alpha = 0.0;
        const double denom = U[i + p] - U[i];

        if (denom != 0.0)
            alpha = (nurbsknot_value - U[i]) / denom;

        const double* Pi_1 = &m_cv[(i - 1) * m_cv_stride];
        const double* Pi = &m_cv[i * m_cv_stride];
        double* Qi = &cv_new[i * m_cv_stride];

        for (int d = 0; d < m_cv_stride; ++d)
            Qi[d] = (1.0 - alpha) * Pi_1[d] + alpha * Pi[d];
    }

    m_cv_count = new_cv_count;
    m_cv = std::move(cv_new);

    const int kc = m_order + m_cv_count - 2;
    std::vector<double> nurbsknot_new(kc);

    for (int i = 0; i < kc; ++i)
        nurbsknot_new[i] = U_new[i + 1];

    m_nurbsknot = std::move(nurbsknot_new);
}

std::vector<std::pair<double, Point>> NurbsCurve::adaptive_samples(double angle_tolerance, double min_edge_length, double max_edge_length) const {

    const double t0 = domain_start();
    const double t1 = domain_end();
    std::vector<std::pair<double, Point>> samples;
    samples.push_back({t0, point_at(t0)});
    samples.push_back({t1, point_at(t1)});

    std::vector<std::pair<double, double>> work_queue;
    work_queue.push_back({t0, t1});

    const int max_iterations = 10000;
    int iterations = 0;

    while (!work_queue.empty() && iterations++ < max_iterations) {
        const double ta = work_queue.back().first;
        const double tb = work_queue.back().second;
        work_queue.pop_back();

        const Point pa = point_at(ta);
        const Point pb = point_at(tb);
        const double chord_length = pa.distance(pb);

        if (chord_length < min_edge_length)
            continue;

        const double tm = (ta + tb) * 0.5;
        const Point pm = point_at(tm);
        const Vector chord = pb - pa;
        const Vector to_mid = pm - pa;
        const double chord_len_sq = chord.dot(chord);
        double deviation = 0.0;

        if (chord_len_sq > 1e-20) {
            const double proj = to_mid.dot(chord) / chord_len_sq;
            deviation = pm.distance(pa + chord * proj);
        }

        const double deviation_tolerance = chord_length * angle_tolerance * 0.5;

        if (deviation > deviation_tolerance || chord_length > max_edge_length) {
            samples.push_back({tm, pm});
            work_queue.push_back({ta, tm});
            work_queue.push_back({tm, tb});
        }
    }

    std::sort(samples.begin(), samples.end(), sample_before);

    return samples;
}

std::vector<std::array<double, 4>> NurbsCurve::homogeneous_derivatives(int span, const std::vector<std::vector<double>>& ders) const {

    const int p = degree();
    const int count = static_cast<int>(ders.size());
    std::vector<std::array<double, 4>> Aders(count);

    for (int k = 0; k < count; ++k) {
        Aders[k] = {0.0, 0.0, 0.0, 0.0};

        for (int j = 0; j <= p; ++j) {
            const double* cv_ptr = cv(span + j);

            if (!cv_ptr)
                continue;

            const double Nx = ders[k][j];
            Aders[k][0] += Nx * cv_ptr[0];
            Aders[k][1] += Nx * ((m_dim > 1) ? cv_ptr[1] : 0.0);
            Aders[k][2] += Nx * ((m_dim > 2) ? cv_ptr[2] : 0.0);
            Aders[k][3] += Nx * (m_is_rat ? cv_ptr[m_dim] : 1.0);
        }
    }

    return Aders;
}

bool NurbsCurve::start_frame(Vector& T0, Vector& r0) const {

    const std::vector<Vector> derivs0 = evaluate(domain_start(), 2);
    const Vector D1_0 = derivs0[1];
    const Vector D2_0 = derivs0[2];
    const double D1_0_mag = D1_0.magnitude();

    if (D1_0_mag < 1e-14)
        return false;

    T0 = D1_0 / D1_0_mag;

    const double D2_dot_D1 = D2_0.dot(D1_0);
    const double D1_0_mag_sq = D1_0_mag * D1_0_mag;
    Vector N0_unnorm = D2_0 - D1_0 * (D2_dot_D1 / D1_0_mag_sq);
    double N0_mag = N0_unnorm.magnitude();

    if (N0_mag < 1e-14) {
        N0_unnorm = Vector(0, 0, 1).cross(T0);
        N0_mag = N0_unnorm.magnitude();

        if (N0_mag < 1e-14) {
            N0_unnorm = Vector(0, 1, 0).cross(T0);
            N0_mag = N0_unnorm.magnitude();
        }
    }

    r0 = N0_unnorm / N0_mag;

    return true;
}

Vector NurbsCurve::double_reflection(double param, const Vector& r0, const Vector& T0) const {

    const double t0 = domain_start();
    const double t1 = domain_end();
    const int num_steps = std::max(10, static_cast<int>((param - t0) / (t1 - t0) * 100));
    const double dt = (param - t0) / num_steps;
    Vector ri = r0;
    double ti = t0;
    Point xi = point_at(ti);
    Vector Ti = T0;

    for (int i = 0; i < num_steps && ti < param - 1e-14; i++) {
        const double ti_next = std::min(ti + dt, param);
        const Point xi_next = point_at(ti_next);
        Vector Ti_next = tangent_at(ti_next);
        Ti_next.normalize_self();

        const Vector v1 = xi_next - xi;
        const double c1 = v1.dot(v1);

        if (c1 < 1e-28) {
            ti = ti_next;
            xi = xi_next;
            Ti = Ti_next;
            continue;
        }

        const double ri_dot_v1 = ri.dot(v1);
        const Vector rL = ri - v1 * (2.0 * ri_dot_v1 / c1);
        const double Ti_dot_v1 = Ti.dot(v1);
        const Vector TL = Ti - v1 * (2.0 * Ti_dot_v1 / c1);
        const Vector v2 = Ti_next - TL;
        const double c2 = v2.dot(v2);

        if (c2 < 1e-28) {
            ri = rL;
        } else {
            const double rL_dot_v2 = rL.dot(v2);
            ri = rL - v2 * (2.0 * rL_dot_v2 / c2);
        }

        if (ri.magnitude() > 1e-14)
            ri.normalize_self();

        ti = ti_next;
        xi = xi_next;
        Ti = Ti_next;
    }

    return ri;
}

bool NurbsCurve::keep_span_range(double t0, double t1) {

    const int p = degree();
    const std::vector<double> U = full_nurbsknots();
    const int full_nurbsknot_count = static_cast<int>(U.size());
    const double tol = Tolerance::ZERO_TOLERANCE;
    int start_span = -1;

    for (int i = full_nurbsknot_count - 1; i >= 0; --i) {
        if (std::abs(U[i] - t0) < tol) {
            start_span = i;
            break;
        }
    }

    int end_span = -1;

    for (int i = 0; i < full_nurbsknot_count; ++i) {
        if (std::abs(U[i] - t1) < tol) {
            end_span = i;
            break;
        }
    }

    if (start_span < 0 || end_span < 0 || start_span >= end_span)
        return false;

    const int first_cv = std::max(0, start_span - p);
    const int last_cv = std::min(end_span - 1, m_cv_count - 1);
    int new_cv_count = last_cv - first_cv + 1;

    if (new_cv_count < m_order) {
        new_cv_count = m_order;

        if (first_cv + new_cv_count > m_cv_count)
            return false;
    }

    std::vector<double> new_nurbsknot = trimmed_nurbsknots(U, start_span, new_cv_count, t0, t1);
    std::vector<double> new_cv(new_cv_count * m_cv_stride);

    for (int i = 0; i < new_cv_count; ++i)
        std::copy(&m_cv[(first_cv + i) * m_cv_stride], &m_cv[(first_cv + i) * m_cv_stride] + m_cv_stride, &new_cv[i * m_cv_stride]);

    m_cv_count = new_cv_count;
    m_cv = std::move(new_cv);
    m_nurbsknot = std::move(new_nurbsknot);

    return true;
}

std::vector<double> NurbsCurve::trimmed_nurbsknots(const std::vector<double>& U, int start_span, int new_cv_count, double t0, double t1) const {

    const int p = degree();
    const int full_nurbsknot_count = static_cast<int>(U.size());
    const int new_nurbsknot_count = new_cv_count + m_order - 2;
    std::vector<double> new_nurbsknot(new_nurbsknot_count);

    for (int i = 0; i < p - 1; ++i)
        new_nurbsknot[i] = t0;

    const int mid_count = new_nurbsknot_count - 2 * (p - 1);

    for (int i = 0; i < mid_count; ++i) {
        const int src_idx = start_span + i;
        new_nurbsknot[p - 1 + i] = src_idx < full_nurbsknot_count ? U[src_idx] : t1;
    }

    for (int i = 0; i < p - 1; ++i)
        new_nurbsknot[new_nurbsknot_count - p + 1 + i] = t1;

    return new_nurbsknot;
}

int NurbsCurve::first_nurbsknot_above(double value) const {

    const int kc = nurbsknot_count();

    for (int i = 0; i < kc; i++)
        if (m_nurbsknot[i] > value)
            return i;

    return -1;
}

int NurbsCurve::seam_nurbsknot_index(double t, int nurbsknot_index) {

    const double d0 = t - m_nurbsknot[nurbsknot_index - 1];
    const double d1 = m_nurbsknot[nurbsknot_index] - t;

    if (d0 <= d1 && d0 < Tolerance::ZERO_TOLERANCE)
        return nurbsknot_index - 1;

    if (d0 > d1 && d1 < Tolerance::ZERO_TOLERANCE)
        return nurbsknot_index;

    if (!insert_nurbsknot(t, 1))
        return -1;

    return first_nurbsknot_above(t + Tolerance::ZERO_TOLERANCE);
}

bool NurbsCurve::rotate_periodic_seam(int nurbsknot_index, double t, double dom_len) {

    const int p = degree();
    const int sc = span_count();
    const int cvc = m_cv_count;
    const int distinct_cvc = cvc - p;
    const int cvdim = cv_size();
    const std::vector<double> old_nurbsknots = m_nurbsknot;
    const std::vector<double> old_cv = m_cv;
    int curr = p - 1;

    for (int i = nurbsknot_index; i < sc + p - 1; i++) {
        m_nurbsknot[curr] = old_nurbsknots[i];
        curr++;
    }

    for (int i = 0; i <= nurbsknot_index - p + 1; i++) {
        m_nurbsknot[curr] = old_nurbsknots[p - 1 + i] + dom_len;
        curr++;
    }

    for (int i = 0; i < p - 1; i++) {
        m_nurbsknot[curr + i] = m_nurbsknot[curr + i - 1] + m_nurbsknot[p + i] - m_nurbsknot[p + i - 1];
        m_nurbsknot[p - 2 - i] = m_nurbsknot[p - i - 1] - m_nurbsknot[curr - 1 - i] + m_nurbsknot[curr - 2 - i];
    }

    int cv_id = nurbsknot_index - p + 1;

    for (int i = 0; i < cvc; i++) {
        int src = cv_id % distinct_cvc;

        if (src < 0)
            src += distinct_cvc;

        for (int j = 0; j < cvdim; j++)
            m_cv[i * m_cv_stride + j] = old_cv[src * m_cv_stride + j];

        cv_id++;
    }

    return set_domain(t, t + dom_len);
}

bool NurbsCurve::split_seam(double t, double dom_len) {

    NurbsCurve left_crv;
    NurbsCurve right_crv;

    if (!split(t, left_crv, right_crv))
        return false;

    const int order = m_order;
    const int cvdim = cv_size();
    const int new_cv_count = right_crv.m_cv_count + left_crv.m_cv_count - 1;
    const int new_kc = order + new_cv_count - 2;
    std::vector<double> new_cv(new_cv_count * m_cv_stride);
    std::vector<double> new_nurbsknots(new_kc);

    for (int i = 0; i < right_crv.m_cv_count; i++)
        for (int j = 0; j < cvdim; j++)
            new_cv[i * m_cv_stride + j] = right_crv.m_cv[i * right_crv.m_cv_stride + j];

    for (int i = 1; i < left_crv.m_cv_count; i++) {
        const int dst = right_crv.m_cv_count + i - 1;

        for (int j = 0; j < cvdim; j++)
            new_cv[dst * m_cv_stride + j] = left_crv.m_cv[i * left_crv.m_cv_stride + j];
    }

    const int rkc = right_crv.nurbsknot_count();

    for (int i = 0; i < rkc; i++)
        new_nurbsknots[i] = right_crv.m_nurbsknot[i];

    const int lkc = left_crv.nurbsknot_count();

    for (int i = order - 1; i < lkc; i++)
        new_nurbsknots[rkc + i - (order - 1)] = left_crv.m_nurbsknot[i] + dom_len;

    m_cv_count = new_cv_count;
    m_cv = std::move(new_cv);
    m_nurbsknot = std::move(new_nurbsknots);

    return set_domain(t, t + dom_len);
}

} // namespace session_cpp
