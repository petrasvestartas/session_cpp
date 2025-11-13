#include "nurbscurve.hpp"
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
    int knot_count = m_order + m_cv_count - 2;
    for (int i = 0; i < m_order; i++) {
        m_knot[i] = 0.0;
    }
    for (int i = m_order; i < knot_count - m_order + 1; i++) {
        m_knot[i] = (i - m_order + 1) * knot_delta;
    }
    for (int i = knot_count - m_order + 1; i < knot_count; i++) {
        m_knot[i] = (knot_count - 2 * m_order + 2) * knot_delta;
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
    
    cv_ptr[0] = point.x();
    if (m_dim > 1) cv_ptr[1] = point.y();
    if (m_dim > 2) cv_ptr[2] = point.z();
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
    Vector line_vec(p1.x() - p0.x(), p1.y() - p0.y(), p1.z() - p0.z());
    double line_length = line_vec.magnitude();
    
    if (line_length < tolerance) return true;
    
    for (int i = 1; i < m_cv_count - 1; i++) {
        Point p = get_cv(i);
        Vector v(p.x() - p0.x(), p.y() - p0.y(), p.z() - p0.z());
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
    
    Vector v1(p1.x() - p0.x(), p1.y() - p0.y(), p1.z() - p0.z());
    Vector v2(p2.x() - p0.x(), p2.y() - p0.y(), p2.z() - p0.z());
    Vector normal = v1.cross(v2);
    
    if (normal.magnitude() < tolerance) return true;
    
    // Check all CVs against this plane
    for (int i = 0; i < m_cv_count; i++) {
        Point p = get_cv(i);
        Vector v(p.x() - p0.x(), p.y() - p0.y(), p.z() - p0.z());
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
int NurbsCurve::find_span(double t) const {
    int n = m_cv_count - 1;
    int p = m_order - 1;
    
    if (t >= m_knot[n+1]) return n;
    if (t <= m_knot[p]) return p;
    
    int low = p;
    int high = n + 1;
    int mid = (low + high) / 2;
    
    while (t < m_knot[mid] || t >= m_knot[mid+1]) {
        if (t < m_knot[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    
    return mid;
}

// Compute basis functions (Cox-de Boor algorithm)
void NurbsCurve::basis_functions(int span, double t, std::vector<double>& basis) const {
    basis.resize(m_order);
    std::vector<double> left(m_order);
    std::vector<double> right(m_order);
    
    basis[0] = 1.0;
    
    for (int j = 1; j < m_order; j++) {
        left[j] = t - m_knot[span + 1 - j];
        right[j] = m_knot[span + j] - t;
        double saved = 0.0;
        
        for (int r = 0; r < j; r++) {
            double temp = basis[r] / (right[r + 1] + left[j - r]);
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis[j] = saved;
    }
}

// Evaluate point on curve
Point NurbsCurve::point_at(double t) const {
    if (!is_valid()) return Point(0, 0, 0);
    
    int span = find_span(t);
    std::vector<double> basis;
    basis_functions(span, t, basis);
    
    double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
    
    for (int i = 0; i < m_order; i++) {
        int cv_idx = span - m_order + 1 + i;
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
    
    Point p = point_at(t);
    result.push_back(Vector(p.x(), p.y(), p.z()));
    
    // Compute derivatives using finite differences
    if (derivative_count >= 1) {
        Vector tangent = tangent_at(t);
        result.push_back(tangent);
    }
    
    if (derivative_count >= 2) {
        // Second derivative (curvature direction)
        double dt = 1e-6;
        Vector tan1 = tangent_at(t - dt);
        Vector tan2 = tangent_at(t + dt);
        Vector second_deriv((tan2.x() - tan1.x()) / (2.0 * dt),
                           (tan2.y() - tan1.y()) / (2.0 * dt),
                           (tan2.z() - tan1.z()) / (2.0 * dt));
        result.push_back(second_deriv);
    }
    
    return result;
}

Vector NurbsCurve::tangent_at(double t) const {
    // Simple finite difference approximation
    double dt = 1e-6;
    Point p1 = point_at(t);
    Point p2 = point_at(t + dt);
    return Vector((p2.x() - p1.x()) / dt, (p2.y() - p1.y()) / dt, (p2.z() - p1.z()) / dt);
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
        double x = xf.m[0] * p.x() + xf.m[4] * p.y() + xf.m[8] * p.z() + xf.m[12];
        double y = xf.m[1] * p.x() + xf.m[5] * p.y() + xf.m[9] * p.z() + xf.m[13];
        double z = xf.m[2] * p.x() + xf.m[6] * p.y() + xf.m[10] * p.z() + xf.m[14];
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
        new_cv_ptr[0] = p.x();
        if (m_dim > 1) new_cv_ptr[1] = p.y();
        if (m_dim > 2) new_cv_ptr[2] = p.z();
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

void NurbsCurve::deep_copy_from(const NurbsCurve& src) {
    m_dim = src.m_dim;
    m_is_rat = src.m_is_rat;
    m_order = src.m_order;
    m_cv_count = src.m_cv_count;
    m_cv_stride = src.m_cv_stride;
    m_cv_capacity = src.m_cv_capacity;
    m_knot = src.m_knot;
    m_cv = src.m_cv;
    guid = src.guid;
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
        j["control_points"].push_back({p.x(), p.y(), p.z()});
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
    // Initialize result matrix
    ders.resize(deriv_order + 1);
    for (auto& row : ders) {
        row.resize(m_order, 0.0);
    }
    
    if (deriv_order == 0) {
        basis_functions(span, t, ders[0]);
        return;
    }
    
    // Simplified derivative computation
    // For full implementation, use algorithm A2.3 from "The NURBS Book"
    
    basis_functions(span, t, ders[0]);
    
    // Approximate derivatives using finite differences
    double dt = 1e-6;
    std::vector<double> basis_plus, basis_minus;
    basis_functions(span, t + dt, basis_plus);
    basis_functions(span, t - dt, basis_minus);
    
    if (deriv_order >= 1) {
        for (int i = 0; i < m_order; i++) {
            ders[1][i] = (basis_plus[i] - basis_minus[i]) / (2.0 * dt);
        }
    }
}

} // namespace session_cpp
