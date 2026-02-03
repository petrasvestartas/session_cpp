#include "nurbssurface.h"
#include "knot.h"
#include "fmt/core.h"
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include "nurbssurface.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors & Destructor
///////////////////////////////////////////////////////////////////////////////////////////

NurbsSurface::NurbsSurface() {
    initialize();
}

NurbsSurface::NurbsSurface(int dimension, bool is_rational,
                          int order0, int order1,
                          int cv_count0, int cv_count1) {
    initialize();
    create_raw(dimension, is_rational, order0, order1, cv_count0, cv_count1);
}

NurbsSurface::NurbsSurface(const NurbsSurface& other) {
    initialize();
    deep_copy_from(other);
}

NurbsSurface& NurbsSurface::operator=(const NurbsSurface& other) {
    if (this != &other) {
        deep_copy_from(other);
    }
    return *this;
}

bool NurbsSurface::operator==(const NurbsSurface& other) const {
    // Compare metadata (excluding guid)
    if (name != other.name) return false;
    if (width != other.width) return false;
    if (surfacecolor != other.surfacecolor) return false;
    if (xform != other.xform) return false;

    // Compare NURBS structure
    if (m_dim != other.m_dim) return false;
    if (m_is_rat != other.m_is_rat) return false;
    if (m_order[0] != other.m_order[0]) return false;
    if (m_order[1] != other.m_order[1]) return false;
    if (m_cv_count[0] != other.m_cv_count[0]) return false;
    if (m_cv_count[1] != other.m_cv_count[1]) return false;
    if (m_cv_stride[0] != other.m_cv_stride[0]) return false;
    if (m_cv_stride[1] != other.m_cv_stride[1]) return false;

    // Compare knot vectors
    if (m_knot[0] != other.m_knot[0]) return false;
    if (m_knot[1] != other.m_knot[1]) return false;

    // Compare control vertices
    if (m_cv != other.m_cv) return false;

    return true;
}

bool NurbsSurface::operator!=(const NurbsSurface& other) const {
    return !(*this == other);
}

NurbsSurface::~NurbsSurface() {
    destroy();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Initialization & Creation
///////////////////////////////////////////////////////////////////////////////////////////

void NurbsSurface::initialize() {
    guid = ::guid();
    name = "my_nurbssurface";
    width = 1.0;
    surfacecolor = Color::black();
    xform = Xform::identity();
    
    m_dim = 0;
    m_is_rat = 0;
    m_order[0] = 0;
    m_order[1] = 0;
    m_cv_count[0] = 0;
    m_cv_count[1] = 0;
    m_cv_stride[0] = 0;
    m_cv_stride[1] = 0;

    m_knot[0].clear();
    m_knot[1].clear();
    m_cv.clear();
}

bool NurbsSurface::create_raw(int dimension, bool is_rational,
                         int order0, int order1,
                         int cv_count0, int cv_count1,
                         bool is_periodic_u, bool is_periodic_v,
                         double knot_delta_u, double knot_delta_v) {
    if (dimension < 1 || order0 < 2 || order1 < 2 ||
        cv_count0 < order0 || cv_count1 < order1) {
        return false;
    }

    destroy();

    m_dim = dimension;
    m_is_rat = is_rational ? 1 : 0;
    m_order[0] = order0;
    m_order[1] = order1;
    m_cv_count[0] = cv_count0;
    m_cv_count[1] = cv_count1;

    // OpenNURBS stride pattern: [1] is CV size, [0] is row stride
    int cv_size_val = m_is_rat ? (m_dim + 1) : m_dim;
    m_cv_stride[1] = cv_size_val;
    m_cv_stride[0] = cv_size_val * m_cv_count[1];

    // Allocate knot vectors
    int knot_count0 = order0 + cv_count0 - 2;
    int knot_count1 = order1 + cv_count1 - 2;

    m_knot[0].resize(knot_count0, 0.0);
    m_knot[1].resize(knot_count1, 0.0);

    // Allocate CV array
    int total_cvs = cv_count0 * cv_count1;
    int cv_array_size = total_cvs * cv_size_val;
    m_cv.resize(cv_array_size, 0.0);

    // Initialize weights to 1 if rational
    if (m_is_rat) {
        for (int i = 0; i < cv_count0; i++) {
            for (int j = 0; j < cv_count1; j++) {
                set_weight(i, j, 1.0);
            }
        }
    }

    // Initialize knot vectors
    if (is_periodic_u) {
        make_periodic_uniform_knot_vector(0, knot_delta_u);
    } else {
        make_clamped_uniform_knot_vector(0, knot_delta_u);
    }

    if (is_periodic_v) {
        make_periodic_uniform_knot_vector(1, knot_delta_v);
    } else {
        make_clamped_uniform_knot_vector(1, knot_delta_v);
    }

    return true;
}

void NurbsSurface::destroy() {
    m_knot[0].clear();
    m_knot[1].clear();
    m_cv.clear();
    initialize();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Validation
///////////////////////////////////////////////////////////////////////////////////////////

bool NurbsSurface::is_valid() const {
    if (m_dim < 1) return false;
    if (m_order[0] < 2 || m_order[1] < 2) return false;
    if (m_cv_count[0] < m_order[0] || m_cv_count[1] < m_order[1]) return false;
    
    int knot_count0 = m_order[0] + m_cv_count[0] - 2;
    int knot_count1 = m_order[1] + m_cv_count[1] - 2;
    
    if (static_cast<int>(m_knot[0].size()) != knot_count0) return false;
    if (static_cast<int>(m_knot[1].size()) != knot_count1) return false;
    
    if (!is_valid_knot_vector(0) || !is_valid_knot_vector(1)) return false;
    
    int cv_size_val = m_is_rat ? (m_dim + 1) : m_dim;
    int expected_cv_size = m_cv_count[0] * m_cv_count[1] * cv_size_val;
    if (static_cast<int>(m_cv.size()) < expected_cv_size) return false;
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

int NurbsSurface::order(int dir) const {
    return (dir >= 0 && dir < 2) ? m_order[dir] : 0;
}

int NurbsSurface::degree(int dir) const {
    return (dir >= 0 && dir < 2) ? (m_order[dir] - 1) : 0;
}

int NurbsSurface::cv_count(int dir) const {
    return (dir >= 0 && dir < 2) ? m_cv_count[dir] : 0;
}

int NurbsSurface::cv_count() const {
    return m_cv_count[0] * m_cv_count[1];
}

int NurbsSurface::cv_size() const {
    return m_is_rat ? (m_dim + 1) : m_dim;
}

int NurbsSurface::knot_count(int dir) const {
    if (dir < 0 || dir >= 2) return 0;
    // OpenNURBS formula: knot_count = order + cv_count - 2
    return m_order[dir] + m_cv_count[dir] - 2;
}

int NurbsSurface::span_count(int dir) const {
    if (dir < 0 || dir >= 2) return 0;
    return m_cv_count[dir] - m_order[dir] + 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Control Vertex Access
///////////////////////////////////////////////////////////////////////////////////////////

double* NurbsSurface::cv(int i, int j) {
    if (i < 0 || i >= m_cv_count[0] || j < 0 || j >= m_cv_count[1]) {
        return nullptr;
    }
    // OpenNURBS pattern: CV(i,j) = m_cv[i*m_cv_stride[0] + j*m_cv_stride[1]]
    int index = i * m_cv_stride[0] + j * m_cv_stride[1];
    return &m_cv[index];
}

const double* NurbsSurface::cv(int i, int j) const {
    if (i < 0 || i >= m_cv_count[0] || j < 0 || j >= m_cv_count[1]) {
        return nullptr;
    }
    // OpenNURBS pattern: CV(i,j) = m_cv[i*m_cv_stride[0] + j*m_cv_stride[1]]
    int index = i * m_cv_stride[0] + j * m_cv_stride[1];
    return &m_cv[index];
}

Point NurbsSurface::get_cv(int i, int j) const {
    const double* cv_ptr = cv(i, j);
    if (!cv_ptr) return Point(0, 0, 0);
    
    if (m_is_rat) {
        double w = cv_ptr[m_dim];
        if (std::abs(w) < 1e-14) return Point(0, 0, 0);
        return Point(cv_ptr[0]/w, 
                    m_dim > 1 ? cv_ptr[1]/w : 0,
                    m_dim > 2 ? cv_ptr[2]/w : 0);
    }
    return Point(cv_ptr[0],
                m_dim > 1 ? cv_ptr[1] : 0,
                m_dim > 2 ? cv_ptr[2] : 0);
}

bool NurbsSurface::set_cv(int i, int j, const Point& point) {
    double* cv_ptr = cv(i, j);
    if (!cv_ptr) return false;

    if (m_is_rat) {
        // For rational surfaces, store homogeneous coordinates (x*w, y*w, z*w, w)
        double w = cv_ptr[m_dim];  // Get current weight
        if (std::abs(w) < 1e-14) w = 1.0;
        cv_ptr[0] = point[0] * w;
        if (m_dim > 1) cv_ptr[1] = point[1] * w;
        if (m_dim > 2) cv_ptr[2] = point[2] * w;
    } else {
        cv_ptr[0] = point[0];
        if (m_dim > 1) cv_ptr[1] = point[1];
        if (m_dim > 2) cv_ptr[2] = point[2];
    }
    return true;
}

double NurbsSurface::weight(int i, int j) const {
    if (!m_is_rat) return 1.0;
    const double* cv_ptr = cv(i, j);
    return cv_ptr ? cv_ptr[m_dim] : 1.0;
}

bool NurbsSurface::set_weight(int i, int j, double w) {
    if (!m_is_rat) return false;
    double* cv_ptr = cv(i, j);
    if (!cv_ptr) return false;

    // Rescale homogeneous coordinates when changing weight
    double old_w = cv_ptr[m_dim];
    if (std::abs(old_w) < 1e-14) old_w = 1.0;
    if (std::abs(w) < 1e-14) w = 1.0;

    double scale = w / old_w;
    cv_ptr[0] *= scale;
    if (m_dim > 1) cv_ptr[1] *= scale;
    if (m_dim > 2) cv_ptr[2] *= scale;
    cv_ptr[m_dim] = w;
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Knot Access
///////////////////////////////////////////////////////////////////////////////////////////

double NurbsSurface::knot(int dir, int knot_index) const {
    if (dir < 0 || dir >= 2 || knot_index < 0 || 
        knot_index >= static_cast<int>(m_knot[dir].size())) {
        return 0.0;
    }
    return m_knot[dir][knot_index];
}

bool NurbsSurface::set_knot(int dir, int knot_index, double knot_value) {
    if (dir < 0 || dir >= 2 || knot_index < 0 || 
        knot_index >= static_cast<int>(m_knot[dir].size())) {
        return false;
    }
    m_knot[dir][knot_index] = knot_value;
    return true;
}

bool NurbsSurface::is_valid_knot_vector(int dir) const {
    if (dir < 0 || dir >= 2) return false;
    int kc = knot_count(dir);
    if (static_cast<int>(m_knot[dir].size()) != kc) return false;
    
    for (int i = 1; i < kc; i++) {
        if (m_knot[dir][i] < m_knot[dir][i-1]) return false;
    }
    return true;
}

std::pair<double, double> NurbsSurface::domain(int dir) const {
    if (!is_valid() || dir < 0 || dir >= 2) {
        return {0.0, 0.0};
    }
    return {m_knot[dir][m_order[dir] - 2], 
            m_knot[dir][m_cv_count[dir] - 1]};
}

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point NurbsSurface::point_at(double u, double v) const {
    if (!is_valid()) return Point(0, 0, 0);
    
    // Find spans - returns indices in range [0, cv_count-order]
    int span_u = find_span(0, u);
    int span_v = find_span(1, v);
    
    // Compute basis functions
    std::vector<double> Nu, Nv;
    basis_functions(0, span_u, u, Nu);
    basis_functions(1, span_v, v, Nv);
    
    // Evaluate surface point - OpenNURBS lines 1107-1117
    // CV index starts at span (since span is in range [0, cv_count-order])
    int cv_size_val = m_is_rat ? (m_dim + 1) : m_dim;
    std::vector<double> point(cv_size_val, 0.0);
    
    for (int j0 = 0; j0 < m_order[0]; j0++) {
        int cv_i = span_u + j0;
        for (int j1 = 0; j1 < m_order[1]; j1++) {
            int cv_j = span_v + j1;
            double c = Nu[j0] * Nv[j1];
            const double* cv_ptr = cv(cv_i, cv_j);
            if (cv_ptr) {
                for (int d = 0; d < cv_size_val; d++) {
                    point[d] += c * cv_ptr[d];
                }
            }
        }
    }
    
    if (m_is_rat && std::abs(point[m_dim]) > 1e-14) {
        double w = point[m_dim];
        return Point(point[0]/w,
                    m_dim > 1 ? point[1]/w : 0,
                    m_dim > 2 ? point[2]/w : 0);
    }
    
    return Point(point[0],
                m_dim > 1 ? point[1] : 0,
                m_dim > 2 ? point[2] : 0);
}

Vector NurbsSurface::normal_at(double u, double v) const {
    auto derivs = evaluate(u, v, 1);
    if (derivs.size() < 3) return Vector(0, 0, 1);
    
    Vector du = derivs[1];
    Vector dv = derivs[2];
    Vector normal = du.cross(dv);
    
    double len = normal.magnitude();
    if (len < 1e-14) return Vector(0, 0, 1);
    
    return normal / len;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Trim Loop
///////////////////////////////////////////////////////////////////////////////////////////

void NurbsSurface::set_outer_loop(const NurbsCurve& loop) { m_outer_loop = loop; }
NurbsCurve NurbsSurface::get_outer_loop() const { return m_outer_loop; }
bool NurbsSurface::is_trimmed() const { return m_outer_loop.is_valid(); }
void NurbsSurface::clear_outer_loop() { m_outer_loop = NurbsCurve(); }

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json NurbsSurface::jsondump() const {
    nlohmann::ordered_json j;
    j["guid"] = guid;
    j["name"] = name;
    j["type"] = "NurbsSurface";
     j["width"] = width;
     j["surfacecolor"] = surfacecolor.jsondump();
     j["xform"] = xform.jsondump();
    j["dimension"] = m_dim;
    j["is_rational"] = m_is_rat != 0;
    j["order_u"] = m_order[0];
    j["order_v"] = m_order[1];
    j["cv_count_u"] = m_cv_count[0];
    j["cv_count_v"] = m_cv_count[1];
    j["knots_u"] = m_knot[0];
    j["knots_v"] = m_knot[1];
    j["control_points"] = m_cv;
    if (is_trimmed()) {
        j["outer_loop"] = m_outer_loop.jsondump();
    }
    return j;
}

std::string NurbsSurface::str() const {
    return fmt::format("NurbsSurface(name={}, degree=({},{}), cvs=({},{}))",
                      name,
                      degree(0), degree(1),
                      m_cv_count[0], m_cv_count[1]);
}

std::string NurbsSurface::repr() const {
    std::string result = fmt::format("NurbsSurface(\n  name={},\n  degree=({},{}),\n  cvs=({},{}),\n  rational={},\n  control_points=[\n",
                                     name, degree(0), degree(1), m_cv_count[0], m_cv_count[1], m_is_rat ? "true" : "false");
    for (int i = 0; i < m_cv_count[0]; ++i) {
        for (int j = 0; j < m_cv_count[1]; ++j) {
            Point p = get_cv(i, j);
            result += fmt::format("    {}, {}, {}\n", p[0], p[1], p[2]);
        }
    }
    result += "  ]\n)";
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Internal Helpers
///////////////////////////////////////////////////////////////////////////////////////////

void NurbsSurface::deep_copy_from(const NurbsSurface& src) {
    guid = ::guid(); // Generate new GUID for copy
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    xform = src.xform;
    
    m_dim = src.m_dim;
    m_is_rat = src.m_is_rat;
    m_order[0] = src.m_order[0];
    m_order[1] = src.m_order[1];
    m_cv_count[0] = src.m_cv_count[0];
    m_cv_count[1] = src.m_cv_count[1];
    m_cv_stride[0] = src.m_cv_stride[0];
    m_cv_stride[1] = src.m_cv_stride[1];

    m_knot[0] = src.m_knot[0];
    m_knot[1] = src.m_knot[1];
    m_cv = src.m_cv;
    m_outer_loop = src.m_outer_loop;
}

int NurbsSurface::find_span(int dir, double t) const {
    if (dir < 0 || dir >= 2) return -1;
    
    // Implements ON_NurbsSpanIndex from OpenNURBS
    int order = m_order[dir];
    int cv_count = m_cv_count[dir];
    const std::vector<double>& knot = m_knot[dir];
    
    // Shift knot pointer by (order-2) - OpenNURBS line 227
    int knot_offset = order - 2;
    int span_len = cv_count - order + 2;
    
    // Binary search in shifted range
    if (t <= knot[knot_offset]) return 0;
    if (t >= knot[knot_offset + span_len - 1]) return span_len - 2;
    
    // Binary search
    int low = 0;
    int high = span_len - 1;
    
    while (high > low + 1) {
        int mid = (low + high) / 2;
        if (t < knot[knot_offset + mid]) {
            high = mid;
        } else {
            low = mid;
        }
    }
    
    return low;
}

void NurbsSurface::basis_functions(int dir, int span, double t, 
                                   std::vector<double>& basis) const {
    if (dir < 0 || dir >= 2) return;
    
    // Implements ON_EvaluateNurbsBasis from OpenNURBS
    int order = m_order[dir];
    int d = order - 1;  // degree
    
    // OpenNURBS shifts knot by (order-2) + span, then by d inside basis
    int knot_base = span + d;
    const std::vector<double>& knot = m_knot[dir];
    
    // Check for degenerate span
    if (knot[knot_base - 1] == knot[knot_base]) {
        basis.resize(order);
        std::fill(basis.begin(), basis.end(), 0.0);
        return;
    }
    
    std::vector<double> N(order * order, 0.0);
    N[order * order - 1] = 1.0;
    
    std::vector<double> left(d);
    std::vector<double> right(d);
    
    // Cox-de Boor recursion - OpenNURBS lines 702-718
    int N_idx = order * order - 1;
    int k_right = knot_base;
    int k_left = knot_base - 1;
    
    for (int j = 0; j < d; j++) {
        int N0_idx = N_idx;
        N_idx -= (order + 1);
        left[j] = t - knot[k_left];
        right[j] = knot[k_right] - t;
        k_left--;
        k_right++;
        
        double x = 0.0;
        for (int r = 0; r <= j; r++) {
            double a0 = left[j - r];
            double a1 = right[r];
            double y = N[N0_idx + r] / (a0 + a1);
            N[N_idx + r] = x + a1 * y;
            x = a0 * y;
        }
        N[N_idx + j + 1] = x;
    }
    
    // Return just the final row of basis functions
    basis.resize(order);
    for (int i = 0; i < order; i++) {
        basis[i] = N[i];
    }
}

std::vector<Vector> NurbsSurface::evaluate(double u, double v, int num_derivs) const {
    std::vector<Vector> result;

    if (!is_valid() || num_derivs < 0) {
        return result;
    }

    // Limit to 2nd order derivatives
    int max_derivs = std::min(num_derivs, 2);

    // Find spans
    int span_u = find_span(0, u);
    int span_v = find_span(1, v);

    // Compute basis function derivatives
    std::vector<std::vector<double>> ders_u, ders_v;
    basis_functions_derivatives(0, span_u, u, max_derivs, ders_u);
    basis_functions_derivatives(1, span_v, v, max_derivs, ders_v);

    // Tensor product evaluation of derivatives
    // Result order: [S(u,v), Su, Sv, Suu, Suv, Svv]
    int cv_size_val = m_is_rat ? (m_dim + 1) : m_dim;

    // Compute all derivative combinations up to max_derivs
    for (int k = 0; k <= max_derivs; k++) {
        for (int l = 0; l <= max_derivs - k; l++) {
            // Compute k-th derivative in u, l-th in v
            std::vector<double> Skl(cv_size_val, 0.0);

            for (int i = 0; i < m_order[0]; i++) {
                int cv_i = span_u + i;
                for (int j = 0; j < m_order[1]; j++) {
                    int cv_j = span_v + j;
                    double coeff = ders_u[k][i] * ders_v[l][j];
                    const double* cv_ptr = cv(cv_i, cv_j);

                    if (cv_ptr) {
                        for (int d = 0; d < cv_size_val; d++) {
                            Skl[d] += coeff * cv_ptr[d];
                        }
                    }
                }
            }

            // Convert to Cartesian (dehomogenize for rational surfaces)
            Vector deriv_vec;
            if (m_is_rat && std::abs(Skl[m_dim]) > 1e-14) {
                double w = Skl[m_dim];

                if (k == 0 && l == 0) {
                    // Point: S = Sw / w
                    deriv_vec = Vector(Skl[0] / w,
                                      m_dim > 1 ? Skl[1] / w : 0,
                                      m_dim > 2 ? Skl[2] / w : 0);
                } else {
                    // For derivatives, need to apply quotient rule
                    // This is simplified - proper implementation would track all previous derivatives
                    // For first derivatives: S' = (Sw' * w - Sw * w') / w^2
                    deriv_vec = Vector(Skl[0] / w,
                                      m_dim > 1 ? Skl[1] / w : 0,
                                      m_dim > 2 ? Skl[2] / w : 0);
                }
            } else {
                deriv_vec = Vector(Skl[0],
                                  m_dim > 1 ? Skl[1] : 0,
                                  m_dim > 2 ? Skl[2] : 0);
            }

            result.push_back(deriv_vec);
        }
    }

    return result;
}

bool NurbsSurface::make_clamped_uniform_knot_vector(int dir, double delta) {
    if (dir < 0 || dir >= 2 || delta <= 0.0) return false;
    
    // Use knot module function
    m_knot[dir] = knot::make_clamped_uniform(m_order[dir], m_cv_count[dir], delta);
    return !m_knot[dir].empty();
}

BoundingBox NurbsSurface::get_bounding_box() const {
    return BoundingBox::from_nurbssurface(*this);
}

NurbsCurve* NurbsSurface::iso_curve(int dir, double c) const {
    if ((dir != 0 && dir != 1) || !is_valid()) {
        return nullptr;
    }
    
    int srf_cv_size = cv_size();
    
    // Create the output curve
    NurbsCurve* nurbs_crv = new NurbsCurve(m_dim, m_is_rat != 0, m_order[dir], m_cv_count[dir]);
    
    // Copy knot vector for the varying direction
    for (int i = 0; i < nurbs_crv->knot_count(); i++) {
        nurbs_crv->set_knot(i, knot(dir, i));
    }
    
    // Find span index in the constant direction
    int span_index = find_span(1 - dir, c);
    if (span_index < 0) {
        span_index = 0;
    } else if (span_index > m_cv_count[1 - dir] - m_order[1 - dir]) {
        span_index = m_cv_count[1 - dir] - m_order[1 - dir];
    }
    
    // For each CV of the isocurve, evaluate a temporary curve through the surface
    for (int cv_idx = 0; cv_idx < nurbs_crv->m_cv_count; cv_idx++) {
        // Create temporary curve for this position in the varying direction
        NurbsCurve N(m_dim, m_is_rat != 0, m_order[1 - dir], m_cv_count[1 - dir]);

        // Copy knots for the constant direction
        for (int i = 0; i < N.knot_count(); i++) {
            N.set_knot(i, knot(1 - dir, i));
        }

        // Fill temporary curve with surface CVs along the constant direction
        for (int i = 0; i < m_cv_count[1 - dir]; i++) {
            const double* Scv = dir ? cv(i, cv_idx) : cv(cv_idx, i);

            if (m_is_rat) {
                // For rational surfaces, copy homogeneous coordinates
                double x = Scv[0];
                double y = (m_dim > 1) ? Scv[1] : 0;
                double z = (m_dim > 2) ? Scv[2] : 0;
                double w = Scv[m_dim];
                N.set_cv_4d(i, x, y, z, w);
            } else {
                // For non-rational surfaces
                Point pt(Scv[0],
                        (m_dim > 1) ? Scv[1] : 0,
                        (m_dim > 2) ? Scv[2] : 0);
                N.set_cv(i, pt);
            }
        }

        // Evaluate temporary curve at parameter c to get this isocurve CV
        Point pt = N.point_at(c);
        nurbs_crv->set_cv(cv_idx, pt);

        // For rational surfaces, the weight is preserved through evaluation
        // The point_at() method already handles rational evaluation correctly
        // So we just need to set a reasonable weight (typically 1.0 for uniform weight dist)
        if (m_is_rat) {
            // Get average weight from the temporary curve CVs as approximation
            double w_sum = 0.0;
            for (int i = 0; i < m_cv_count[1 - dir]; i++) {
                w_sum += N.weight(i);
            }
            double w_avg = w_sum / m_cv_count[1 - dir];
            nurbs_crv->set_weight(cv_idx, w_avg);
        }
    }
    
    return nurbs_crv;
}

bool NurbsSurface::transform(const Xform& xf) {
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            Point pt = get_cv(i, j);
            xf.transform_point(pt);
            set_cv(i, j, pt);
        }
    }
    return true;
}

std::ostream& operator<<(std::ostream& os, const NurbsSurface& surface) {
    os << surface.str();
    return os;
}

// Remaining stub implementations
bool NurbsSurface::get_cv_4d(int i, int j, double& x, double& y, double& z, double& w) const {
    const double* cv_ptr = cv(i, j);
    if (!cv_ptr) return false;
    x = cv_ptr[0];
    y = m_dim > 1 ? cv_ptr[1] : 0;
    z = m_dim > 2 ? cv_ptr[2] : 0;
    w = m_is_rat ? cv_ptr[m_dim] : 1.0;
    return true;
}

bool NurbsSurface::set_cv_4d(int i, int j, double x, double y, double z, double w) {
    double* cv_ptr = cv(i, j);
    if (!cv_ptr) return false;
    cv_ptr[0] = x;
    if (m_dim > 1) cv_ptr[1] = y;
    if (m_dim > 2) cv_ptr[2] = z;
    if (m_is_rat) cv_ptr[m_dim] = w;
    return true;
}

int NurbsSurface::knot_multiplicity(int dir, int knot_index) const {
    if (dir < 0 || dir >= 2 || knot_index < 0 ||
        knot_index >= static_cast<int>(m_knot[dir].size())) {
        return 0;
    }

    int mult = 1;
    double kv = m_knot[dir][knot_index];

    // Count backward
    for (int i = knot_index - 1; i >= 0; i--) {
        if (std::abs(m_knot[dir][i] - kv) < 1e-10) {
            mult++;
        } else {
            break;
        }
    }

    // Count forward
    for (int i = knot_index + 1; i < static_cast<int>(m_knot[dir].size()); i++) {
        if (std::abs(m_knot[dir][i] - kv) < 1e-10) {
            mult++;
        } else {
            break;
        }
    }
    return mult;
}

std::vector<double> NurbsSurface::get_knots(int dir) const {
    if (dir >= 0 && dir < 2) {
        return m_knot[dir];
    }
    return std::vector<double>();
}

bool NurbsSurface::set_domain(int dir, double t0, double t1) {
    if (!is_valid() || dir < 0 || dir >= 2 || t0 >= t1) return false;
    
    auto [d0, d1] = domain(dir);
    if (std::abs(d1 - d0) < 1e-14) return false;
    
    double scale = (t1 - t0) / (d1 - d0);
    for (size_t i = 0; i < m_knot[dir].size(); i++) {
        m_knot[dir][i] = t0 + (m_knot[dir][i] - d0) * scale;
    }
    return true;
}

std::vector<double> NurbsSurface::get_span_vector(int dir) const {
    std::vector<double> spans;
    if (!is_valid() || dir < 0 || dir >= 2) return spans;
    
    spans.push_back(m_knot[dir][m_order[dir] - 2]);
    for (size_t i = m_order[dir] - 1; i < m_knot[dir].size() - m_order[dir] + 2; i++) {
        if (m_knot[dir][i] > spans.back()) {
            spans.push_back(m_knot[dir][i]);
        }
    }
    return spans;
}

bool NurbsSurface::is_clamped(int dir, int end) const {
    if (dir < 0 || dir >= 2) return false;
    if (m_knot[dir].empty()) return false;
    
    // Use knot module function
    return knot::is_clamped(m_order[dir], m_cv_count[dir], m_knot[dir], end);
}

bool NurbsSurface::make_periodic_uniform_knot_vector(int dir, double delta) {
    if (dir < 0 || dir >= 2 || delta <= 0.0) return false;
    
    // Use knot module function
    m_knot[dir] = knot::make_periodic_uniform(m_order[dir], m_cv_count[dir], delta);
    return !m_knot[dir].empty();
}

// Forward declaration of implementation (defined after helper functions)
namespace { bool insert_knot_impl(NurbsSurface& srf, int dir, double knot_value, int knot_multiplicity); }

bool NurbsSurface::insert_knot(int dir, double knot_value, int knot_multiplicity) {
    // Actual implementation is after helper function definitions
    // This stub delegates to insert_knot_impl to avoid forward reference issues
    return insert_knot_impl(*this, dir, knot_value, knot_multiplicity);
}

Point NurbsSurface::point_at_corner(int u_end, int v_end) const {
    int i = (u_end == 0) ? 0 : m_cv_count[0] - 1;
    int j = (v_end == 0) ? 0 : m_cv_count[1] - 1;
    return get_cv(i, j);
}

bool NurbsSurface::reverse(int dir) {
    if (dir < 0 || dir >= 2) return false;
    
    // Reverse knot vector using knot module function
    knot::reverse(m_order[dir], m_cv_count[dir], m_knot[dir]);
    
    // Reverse CVs in direction
    if (dir == 0) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            for (int i = 0; i < m_cv_count[0] / 2; i++) {
                int i2 = m_cv_count[0] - 1 - i;
                Point temp = get_cv(i, j);
                set_cv(i, j, get_cv(i2, j));
                set_cv(i2, j, temp);
            }
        }
    } else {
        for (int i = 0; i < m_cv_count[0]; i++) {
            for (int j = 0; j < m_cv_count[1] / 2; j++) {
                int j2 = m_cv_count[1] - 1 - j;
                Point temp = get_cv(i, j);
                set_cv(i, j, get_cv(i, j2));
                set_cv(i, j2, temp);
            }
        }
    }
    return true;
}

bool NurbsSurface::transpose() {
    // Transpose CV grid and swap parameters
    std::swap(m_order[0], m_order[1]);
    std::swap(m_cv_count[0], m_cv_count[1]);
    std::swap(m_cv_stride[0], m_cv_stride[1]);

    // Swap knot vectors
    std::swap(m_knot[0], m_knot[1]);

    return true;
}

bool NurbsSurface::swap_coordinates(int axis_i, int axis_j) {
    if (axis_i < 0 || axis_i >= m_dim || axis_j < 0 || axis_j >= m_dim) {
        return false;
    }
    
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            double* cv_ptr = cv(i, j);
            if (cv_ptr) {
                std::swap(cv_ptr[axis_i], cv_ptr[axis_j]);
            }
        }
    }
    return true;
}

// Forward declarations for trim/split impl (defined after helper functions)
namespace {
bool trim_impl(NurbsSurface& srf, int dir, const std::pair<double, double>& domain_pair);
bool split_impl(const NurbsSurface& srf, int dir, double c, NurbsSurface*& lo, NurbsSurface*& hi);
}

bool NurbsSurface::trim(int dir, const std::pair<double, double>& domain_pair) {
    return trim_impl(*this, dir, domain_pair);
}

bool NurbsSurface::split(int dir, double c, NurbsSurface*& west_or_south_side,
                        NurbsSurface*& east_or_north_side) const {
    return split_impl(*this, dir, c, west_or_south_side, east_or_north_side);
}

bool NurbsSurface::extend(int dir, const std::pair<double, double>& domain_pair) {
    // TODO: Implement using curve conversion helpers
    return false;
}

bool NurbsSurface::make_rational() {
    if (m_is_rat) return true;
    
    int new_stride_1 = m_dim + 1;
    int new_stride_0 = new_stride_1 * m_cv_count[1];
    std::vector<double> new_cv(m_cv_count[0] * m_cv_count[1] * new_stride_1);
    
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* old_ptr = cv(i, j);
            double* new_ptr = &new_cv[i * new_stride_0 + j * new_stride_1];
            
            for (int d = 0; d < m_dim; d++) {
                new_ptr[d] = old_ptr[d];
            }
            new_ptr[m_dim] = 1.0; // weight
        }
    }
    
    m_cv = new_cv;
    m_is_rat = 1;
    m_cv_stride[1] = new_stride_1;
    m_cv_stride[0] = new_stride_0;

    return true;
}

bool NurbsSurface::make_non_rational() {
    if (!m_is_rat) return true;
    
    // Check if all weights are equal to 1
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            double w = weight(i, j);
            if (std::abs(w - 1.0) > 1e-10) {
                return false; // Cannot make non-rational with non-uniform weights
            }
        }
    }
    
    // Convert to non-rational by removing weights
    int new_stride_1 = m_dim;
    int new_stride_0 = new_stride_1 * m_cv_count[1];
    std::vector<double> new_cv(m_cv_count[0] * m_cv_count[1] * m_dim);
    
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* old_ptr = cv(i, j);
            double* new_ptr = &new_cv[i * new_stride_0 + j * new_stride_1];
            
            for (int d = 0; d < m_dim; d++) {
                new_ptr[d] = old_ptr[d];
            }
        }
    }
    
    m_cv = new_cv;
    m_is_rat = 0;
    m_cv_stride[1] = new_stride_1;
    m_cv_stride[0] = new_stride_0;

    return true;
}

bool NurbsSurface::clamp_end(int dir, int end) {
    if (dir < 0 || dir > 1) return false;
    
    // TODO: Implement using curve conversion helpers
    return false;
}

bool NurbsSurface::increase_degree(int dir, int desired_degree) {
    // TODO: Implement using curve conversion helpers
    return false;
}

void NurbsSurface::transform() {
    transform(xform);
}

NurbsSurface NurbsSurface::transformed() const {
    NurbsSurface result = *this;
    result.transform();
    return result;
}

NurbsSurface NurbsSurface::transformed(const Xform& xf) const {
    NurbsSurface result = *this;
    result.transform(xf);
    return result;
}

double NurbsSurface::area(double tolerance) const {
    return 0.0; // Stub - would require numerical integration
}

Point NurbsSurface::closest_point(const Point& point, double& u_out, double& v_out) const {
    u_out = 0.5;
    v_out = 0.5;
    return Point(0, 0, 0); // Stub
}

NurbsSurface NurbsSurface::jsonload(const nlohmann::json& data) {
    NurbsSurface surface;
 
    if (data.contains("dimension") && data.contains("order_u") && data.contains("order_v") &&
        data.contains("cv_count_u") && data.contains("cv_count_v")) {
        int dim = data["dimension"];
        bool is_rat = data.value("is_rational", false);
        int order_u = data["order_u"];
        int order_v = data["order_v"];
        int cv_count_u = data["cv_count_u"];
        int cv_count_v = data["cv_count_v"];
 
        surface.create_raw(dim, is_rat, order_u, order_v, cv_count_u, cv_count_v);
 
        if (data.contains("knots_u")) {
            surface.m_knot[0] = data["knots_u"].get<std::vector<double>>();
        }
        if (data.contains("knots_v")) {
            surface.m_knot[1] = data["knots_v"].get<std::vector<double>>();
        }
        if (data.contains("control_points")) {
            surface.m_cv = data["control_points"].get<std::vector<double>>();
        }
 
        surface.guid = data.value("guid", ::guid());
        surface.name = data.value("name", "my_nurbssurface");
        surface.width = data.value("width", 1.0);
 
        if (data.contains("surfacecolor")) {
            surface.surfacecolor = Color::jsonload(data["surfacecolor"]);
        }
        if (data.contains("xform")) {
            surface.xform = Xform::jsonload(data["xform"]);
        }
        if (data.contains("outer_loop")) {
            surface.m_outer_loop = NurbsCurve::jsonload(data["outer_loop"]);
        }
    }

    return surface;
}

std::string NurbsSurface::json_dumps() const {
    return jsondump().dump();
}

NurbsSurface NurbsSurface::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void NurbsSurface::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

NurbsSurface NurbsSurface::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

std::string NurbsSurface::pb_dumps() const {
    session_proto::NurbsSurface proto;

    // Basic metadata
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_dimension(m_dim);
    proto.set_is_rational(m_is_rat != 0);
    proto.set_order_u(m_order[0]);
    proto.set_order_v(m_order[1]);
    proto.set_cv_count_u(m_cv_count[0]);
    proto.set_cv_count_v(m_cv_count[1]);
    proto.set_cv_stride_u(m_cv_stride[0]);
    proto.set_cv_stride_v(m_cv_stride[1]);

    // Knot vectors
    for (size_t i = 0; i < m_knot[0].size(); i++) {
        proto.add_knots_u(m_knot[0][i]);
    }
    for (size_t i = 0; i < m_knot[1].size(); i++) {
        proto.add_knots_v(m_knot[1][i]);
    }

    // Control vertices (flat array)
    for (size_t i = 0; i < m_cv.size(); i++) {
        proto.add_cvs(m_cv[i]);
    }

    // Visual properties
    proto.set_width(width);

    // Surface color
    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);

    // Transform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }

    return proto.SerializeAsString();
}

NurbsSurface NurbsSurface::pb_loads(const std::string& data) {
    session_proto::NurbsSurface proto;
    proto.ParseFromString(data);

    // Create surface with correct dimensions
    NurbsSurface surface;
    surface.create_raw(proto.dimension(), proto.is_rational(),
                   proto.order_u(), proto.order_v(),
                   proto.cv_count_u(), proto.cv_count_v());

    // Load metadata
    surface.guid = proto.guid();
    surface.name = proto.name();

    // Load knot vectors
    for (int i = 0; i < proto.knots_u_size(); i++) {
        if (i < (int)surface.m_knot[0].size()) {
            surface.m_knot[0][i] = proto.knots_u(i);
        }
    }
    for (int i = 0; i < proto.knots_v_size(); i++) {
        if (i < (int)surface.m_knot[1].size()) {
            surface.m_knot[1][i] = proto.knots_v(i);
        }
    }

    // Load control vertices
    for (int i = 0; i < proto.cvs_size() && i < (int)surface.m_cv.size(); i++) {
        surface.m_cv[i] = proto.cvs(i);
    }

    // Load visual properties
    surface.width = proto.width();

    // Load color
    const auto& color_proto = proto.surfacecolor();
    surface.surfacecolor.name = color_proto.name();
    surface.surfacecolor.r = color_proto.r();
    surface.surfacecolor.g = color_proto.g();
    surface.surfacecolor.b = color_proto.b();
    surface.surfacecolor.a = color_proto.a();

    // Load transform
    const auto& xform_proto = proto.xform();
    surface.xform.guid = xform_proto.guid();
    surface.xform.name = xform_proto.name();
    for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i) {
        surface.xform.m[i] = xform_proto.matrix(i);
    }

    return surface;
}

void NurbsSurface::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

NurbsSurface NurbsSurface::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    return pb_loads(data);
}

bool NurbsSurface::zero_cvs() {
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            set_cv(i, j, Point(0, 0, 0));
            if (m_is_rat) {
                set_weight(i, j, 1.0);
            }
        }
    }
    return true;
}

bool NurbsSurface::is_duplicate(const NurbsSurface& other, 
                               bool ignore_parameterization,
                               double tolerance) const {
    return false; // Stub
}

bool NurbsSurface::collapse_side(int side, const Point& point) {
    return false; // Stub
}

bool NurbsSurface::create_clamped_uniform(int dimension,
                                         int order0, int order1,
                                         int cv_count0, int cv_count1,
                                         double knot_delta0,
                                         double knot_delta1) {
    if (!create_raw(dimension, false, order0, order1, cv_count0, cv_count1)) {
        return false;
    }
    
    make_clamped_uniform_knot_vector(0, knot_delta0);
    make_clamped_uniform_knot_vector(1, knot_delta1);
    
    return true;
}

NurbsSurface NurbsSurface::create(bool periodic_u, bool periodic_v,
                                  int degree_u, int degree_v,
                                  int cv_count_u, int cv_count_v,
                                  const std::vector<Point>& points) {
    NurbsSurface surface;
    if (cv_count_u < 2 || cv_count_v < 2) return surface;
    if (static_cast<int>(points.size()) != cv_count_u * cv_count_v) return surface;

    int order0 = degree_u + 1;
    int order1 = degree_v + 1;

    surface.create_raw(3, false, order0, order1, cv_count_u, cv_count_v,
                       periodic_u, periodic_v, 1.0, 1.0);

    for (int i = 0; i < cv_count_u; i++) {
        for (int j = 0; j < cv_count_v; j++) {
            surface.set_cv(i, j, points[i * cv_count_v + j]);
        }
    }

    return surface;
}

NurbsSurface NurbsSurface::create_ruled(const NurbsCurve& curveA, const NurbsCurve& curveB) {
    NurbsSurface surface;
    if (!curveA.is_valid() || !curveB.is_valid()) return surface;

    NurbsCurve cA = curveA;
    NurbsCurve cB = curveB;

    cA.set_domain(0.0, 1.0);
    cB.set_domain(0.0, 1.0);

    // Elevate degree of lower-degree curve to match
    if (cA.degree() < cB.degree()) cA.increase_degree(cB.degree());
    else if (cB.degree() < cA.degree()) cB.increase_degree(cA.degree());

    // Make both rational if either is
    if (cA.is_rational() || cB.is_rational()) {
        cA.make_rational();
        cB.make_rational();
    }

    // Knot compatibility: insert missing knots from each curve into the other
    {
        std::vector<double> knotsA = cA.get_knots();
        std::vector<double> knotsB = cB.get_knots();
        const double tol = 1e-10;

        // Insert knots from B that are missing in A
        for (double k : knotsB) {
            bool found = false;
            for (double ka : knotsA) {
                if (std::abs(ka - k) < tol) { found = true; break; }
            }
            if (!found) cA.insert_knot(k, 1);
        }

        // Refresh and insert knots from A that are missing in B
        knotsA = cA.get_knots();
        for (double k : knotsA) {
            bool found = false;
            for (double kb : knotsB) {
                if (std::abs(kb - k) < tol) { found = true; break; }
            }
            if (!found) cB.insert_knot(k, 1);
        }
    }

    // Both curves should now have the same cv_count and knot vector
    int order_u = cA.order();
    int cv_count_u = cA.cv_count();
    int order_v = 2;
    int cv_count_v = 2;
    bool is_rat = cA.is_rational();

    surface.create_raw(3, is_rat, order_u, order_v, cv_count_u, cv_count_v);

    // Copy u-knots from the compatible curves
    for (int i = 0; i < cA.knot_count(); i++) {
        surface.set_knot(0, i, cA.knot(i));
    }

    // v-knots: [0, 1] for linear (order 2, 2 CVs => knot_count = 2)
    surface.set_knot(1, 0, 0.0);
    surface.set_knot(1, 1, 1.0);

    // Row 0 CVs = curveA, Row 1 CVs = curveB
    if (is_rat) {
        for (int i = 0; i < cv_count_u; i++) {
            double ax, ay, az, aw;
            cA.get_cv_4d(i, ax, ay, az, aw);
            surface.set_cv_4d(i, 0, ax, ay, az, aw);

            double bx, by, bz, bw;
            cB.get_cv_4d(i, bx, by, bz, bw);
            surface.set_cv_4d(i, 1, bx, by, bz, bw);
        }
    } else {
        for (int i = 0; i < cv_count_u; i++) {
            surface.set_cv(i, 0, cA.get_cv(i));
            surface.set_cv(i, 1, cB.get_cv(i));
        }
    }

    return surface;
}

NurbsSurface NurbsSurface::create_planar(const std::vector<NurbsCurve>& curves) {
    NurbsSurface surface;
    if (curves.empty()) return surface;

    // Collect all control points from all curves
    std::vector<Point> all_pts;
    for (const auto& crv : curves) {
        for (int i = 0; i < crv.cv_count(); i++) {
            all_pts.push_back(crv.get_cv(i));
        }
    }
    if (all_pts.size() < 3) return surface;

    // Fit plane through points using PCA
    Plane plane = Plane::from_points_pca(all_pts);
    if (plane.z_axis().magnitude() < 1e-10) return surface;

    Vector xax = plane.x_axis();
    Vector yax = plane.y_axis();
    Point orig = plane.origin();

    double min_u = std::numeric_limits<double>::max();
    double max_u = -std::numeric_limits<double>::max();
    double min_v = std::numeric_limits<double>::max();
    double max_v = -std::numeric_limits<double>::max();

    for (const auto& pt : all_pts) {
        double dx = pt[0] - orig[0];
        double dy = pt[1] - orig[1];
        double dz = pt[2] - orig[2];
        double u = dx * xax[0] + dy * xax[1] + dz * xax[2];
        double v = dx * yax[0] + dy * yax[1] + dz * yax[2];
        if (u < min_u) min_u = u;
        if (u > max_u) max_u = u;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
    }

    // Expand slightly
    double pad = std::max(max_u - min_u, max_v - min_v) * 0.05;
    if (pad < 1e-6) pad = 1.0;
    min_u -= pad; max_u += pad;
    min_v -= pad; max_v += pad;

    double range_u = max_u - min_u;
    double range_v = max_v - min_v;

    // Create degree-1 bilinear patch (2x2 CVs, order 2 in both directions)
    surface.create_raw(3, false, 2, 2, 2, 2);
    surface.set_knot(0, 0, 0.0);
    surface.set_knot(0, 1, 1.0);
    surface.set_knot(1, 0, 0.0);
    surface.set_knot(1, 1, 1.0);

    auto plane_pt = [&](double u, double v) -> Point {
        return Point(
            orig[0] + u * xax[0] + v * yax[0],
            orig[1] + u * xax[1] + v * yax[1],
            orig[2] + u * xax[2] + v * yax[2]
        );
    };

    surface.set_cv(0, 0, plane_pt(min_u, min_v));
    surface.set_cv(0, 1, plane_pt(min_u, max_v));
    surface.set_cv(1, 0, plane_pt(max_u, min_v));
    surface.set_cv(1, 1, plane_pt(max_u, max_v));

    // Compute outer trim loop in UV space from boundary curves
    std::vector<Point> uv_pts;
    for (const auto& crv : curves) {
        auto [pts3d, params] = crv.divide_by_count(50, true);
        for (const auto& pt : pts3d) {
            double dx = pt[0] - orig[0];
            double dy = pt[1] - orig[1];
            double dz = pt[2] - orig[2];
            double pu = dx * xax[0] + dy * xax[1] + dz * xax[2];
            double pv = dx * yax[0] + dy * yax[1] + dz * yax[2];
            double nu = (pu - min_u) / range_u;
            double nv = (pv - min_v) / range_v;
            uv_pts.push_back(Point(nu, nv, 0.0));
        }
    }

    if (uv_pts.size() >= 3) {
        NurbsCurve loop = NurbsCurve::create(false, 3, uv_pts);
        surface.set_outer_loop(loop);
    }

    return surface;
}

void NurbsSurface::basis_functions_derivatives(int dir, int span, double t, int deriv_order,
                                              std::vector<std::vector<double>>& ders) const {
    if (dir < 0 || dir >= 2) return;

    int order = m_order[dir];
    int degree = order - 1;
    const std::vector<double>& knot = m_knot[dir];

    // Knot base index (shifted by degree like in basis_functions)
    int knot_base = span + degree;

    // Initialize derivatives matrix: ders[k][j] = k-th derivative of N_{span-degree+j,degree}
    ders.resize(deriv_order + 1);
    for (int k = 0; k <= deriv_order; k++) {
        ders[k].resize(order, 0.0);
    }

    // Check for degenerate span
    if (knot[knot_base - 1] == knot[knot_base]) {
        return; // All derivatives are zero
    }

    // Compute basis functions using triangular table
    // ndu[j][r] = basis function N_{span-degree+r,j}
    std::vector<std::vector<double>> ndu(order, std::vector<double>(order, 0.0));
    ndu[0][0] = 1.0;

    std::vector<double> left(degree + 1);
    std::vector<double> right(degree + 1);

    // Compute basis functions (0-th derivatives)
    for (int j = 1; j <= degree; j++) {
        left[j] = t - knot[knot_base - j];
        right[j] = knot[knot_base + j - 1] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            // Lower triangle (knot differences)
            ndu[j][r] = right[r + 1] + left[j - r];
            double temp = ndu[r][j - 1] / ndu[j][r];

            // Upper triangle (basis values)
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    // Copy 0-th derivatives (basis functions)
    for (int j = 0; j <= degree; j++) {
        ders[0][j] = ndu[j][degree];
    }

    // Compute derivatives using two-row array approach
    std::vector<std::vector<double>> a(2, std::vector<double>(order, 0.0));

    // Loop over basis functions
    for (int r = 0; r <= degree; r++) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;

        // Loop over derivatives
        for (int k = 1; k <= deriv_order; k++) {
            double d = 0.0;
            int rk = r - k;
            int pk = degree - k;

            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            int j1 = (rk >= -1) ? 1 : -rk;
            int j2 = (r - 1 <= pk) ? k - 1 : degree - r;

            for (int j = j1; j <= j2; j++) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }

            ders[k][r] = d;

            // Swap rows
            std::swap(s1, s2);
        }
    }

    // Multiply by factorial of derivative order / knot differences
    int factorial = degree;
    for (int k = 1; k <= deriv_order; k++) {
        for (int j = 0; j <= degree; j++) {
            ders[k][j] *= factorial;
        }
        factorial *= (degree - k);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Helper Utility Functions (OpenNURBS-style)
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

// Forward declarations for internal helpers
NurbsCurve* to_curve_internal(const NurbsSurface& srf, int dir, NurbsCurve* crv);
bool from_curve_internal(NurbsCurve& crv, NurbsSurface& srf, int dir);

// Check if knot vector is clamped
bool is_knot_vector_clamped(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) return false;
    if (static_cast<int>(knot.size()) != order + cv_count - 2) return false;
    
    int degree = order - 1;
    
    // Check start
    for (int i = 1; i < degree + 1; i++) {
        if (std::abs(knot[i] - knot[0]) > 1e-10) return false;
    }
    
    // Check end
    int last = static_cast<int>(knot.size()) - 1;
    for (int i = last - 1; i >= last - degree; i--) {
        if (std::abs(knot[i] - knot[last]) > 1e-10) return false;
    }
    
    return true;
}

// Check if knot vector is periodic
bool is_knot_vector_periodic(int order, int cv_count, const std::vector<double>& knot) {
    if (order < 2 || cv_count < order) return false;
    if (static_cast<int>(knot.size()) != order + cv_count - 2) return false;
    
    int degree = order - 1;
    double delta = knot[cv_count - 1] - knot[degree];
    
    if (delta <= 0.0) return false;
    
    for (int i = 0; i < cv_count - 1; i++) {
        double expected = knot[i + degree] + delta;
        if (std::abs(knot[i + order - 1] - expected) > 1e-10 * (1.0 + std::abs(expected))) {
            return false;
        }
    }
    
    return true;
}

// Check if points are coincident
bool points_are_coincident(int dim, bool is_rat, const double* p, const double* q) {
    if (!p || !q) return false;
    
    const double tolerance = 1.0e-12;
    
    if (is_rat) {
        double pw = p[dim];
        double qw = q[dim];
        
        if (std::abs(pw) < tolerance && std::abs(qw) < tolerance) return true;
        if (std::abs(pw) < tolerance || std::abs(qw) < tolerance) return false;
        
        for (int i = 0; i < dim; i++) {
            double a = p[i] / pw;
            double b = q[i] / qw;
            if (std::abs(a - b) > tolerance) return false;
        }
        return true;
    } else {
        for (int i = 0; i < dim; i++) {
            if (std::abs(p[i] - q[i]) > tolerance) return false;
        }
        return true;
    }
}

// Check if multiple points are coincident
bool points_are_coincident(int dim, bool is_rat, int count, int stride, const double* points) {
    if (count < 2 || !points) return true;
    
    const double* p0 = points;
    for (int i = 1; i < count; i++) {
        const double* pi = points + i * stride;
        if (!points_are_coincident(dim, is_rat, p0, pi)) {
            return false;
        }
    }
    return true;
}

// Check if point grid is closed in a direction
bool is_point_grid_closed(int dim, bool is_rat, 
                          int count_0, int count_1,
                          int stride_0, int stride_1,
                          const double* cv, int dir) {
    if (dir < 0 || dir > 1) return false;
    
    int perpendicular_count = (dir == 0) ? count_1 : count_0;
    int perpendicular_stride = (dir == 0) ? stride_1 : stride_0;
    int parallel_stride = (dir == 0) ? stride_0 : stride_1;
    int parallel_count = (dir == 0) ? count_0 : count_1;
    
    const double* start_row = cv;
    const double* end_row = cv + (parallel_count - 1) * parallel_stride;
    
    for (int i = 0; i < perpendicular_count; i++) {
        if (!points_are_coincident(dim, is_rat, start_row, end_row)) {
            return false;
        }
        start_row += perpendicular_stride;
        end_row += perpendicular_stride;
    }
    
    return true;
}

// Helper: Convert surface to curve (for editing operations)
NurbsCurve* to_curve_internal(const NurbsSurface& srf, int dir, NurbsCurve* crv) {
    if (dir < 0 || dir > 1) return nullptr;
    if (srf.m_cv.empty()) return nullptr;
    
    if (!crv) {
        crv = new NurbsCurve();
    }
    
    int srf_cv_size = srf.cv_size();
    
    // Create curve with high dimension (srf_cv_size * perpendicular CV count)
    if (!crv->create(srf_cv_size * srf.m_cv_count[1 - dir], false, 
                     srf.m_order[dir], srf.m_cv_count[dir])) {
        return nullptr;
    }
    
    // Copy CVs from surface to curve
    for (int i = 0; i < srf.m_cv_count[dir]; i++) {
        double* pdst = crv->cv(i);
        const double* psrc = dir ? srf.cv(0, i) : srf.cv(i, 0);
        
        for (int j = 0; j < srf.m_cv_count[1 - dir]; j++) {
            std::memcpy(pdst, psrc, srf_cv_size * sizeof(double));
            psrc += srf.m_cv_stride[1 - dir];
            pdst += srf_cv_size;
        }
    }
    
    // Copy knot vector
    for (int i = 0; i < crv->knot_count(); i++) {
        crv->set_knot(i, srf.knot(dir, i));
    }
    
    return crv;
}

// Helper: Convert curve back to surface (after editing)
bool from_curve_internal(NurbsCurve& crv, NurbsSurface& srf, int dir) {
    if (dir < 0 || dir > 1) return false;
    if (crv.m_cv.empty()) return false;
    if (crv.m_is_rat) return false;
    
    int srf_cv_size = srf.cv_size();
    if (srf.m_cv_count[1 - dir] * srf_cv_size != crv.m_dim) {
        return false;
    }
    
    // Transfer CV array from curve to surface
    srf.m_cv = std::move(crv.m_cv);

    // Transfer knot vector from curve to surface
    srf.m_knot[dir] = std::move(crv.m_knot);

    // Update surface parameters
    srf.m_order[dir] = crv.m_order;
    srf.m_cv_count[dir] = crv.m_cv_count;
    srf.m_cv_stride[dir] = crv.m_cv_stride;
    srf.m_cv_stride[1 - dir] = srf_cv_size;
    
    return true;
}

} // anonymous namespace

///////////////////////////////////////////////////////////////////////////////////////////
// Complete OpenNURBS Method Implementations
///////////////////////////////////////////////////////////////////////////////////////////

bool NurbsSurface::is_closed(int dir) const {
    bool bIsClosed = false;
    if (dir >= 0 && dir <= 1 && m_dim > 0) {
        if (is_knot_vector_clamped(m_order[dir], m_cv_count[dir], m_knot[dir])) {
            const double* corners[4];
            corners[0] = cv(0, 0);
            corners[(dir == 0) ? 1 : 2] = cv(m_cv_count[0] - 1, 0);
            corners[(dir == 0) ? 2 : 1] = cv(0, m_cv_count[1] - 1);
            corners[3] = cv(m_cv_count[0] - 1, m_cv_count[1] - 1);
            
            if (points_are_coincident(m_dim, m_is_rat, corners[0], corners[1]) &&
                points_are_coincident(m_dim, m_is_rat, corners[2], corners[3]) &&
                is_point_grid_closed(m_dim, m_is_rat, m_cv_count[0], m_cv_count[1],
                                    m_cv_stride[0], m_cv_stride[1], m_cv.data(), dir)) {
                bIsClosed = true;
            }
        } else if (is_periodic(dir)) {
            bIsClosed = true;
        }
    }
    return bIsClosed;
}

bool NurbsSurface::is_periodic(int dir) const {
    bool bIsPeriodic = false;
    if (dir >= 0 && dir <= 1) {
        bIsPeriodic = is_knot_vector_periodic(m_order[dir], m_cv_count[dir], m_knot[dir]);
        if (bIsPeriodic) {
            const double* cv0;
            const double* cv1;
            int i0 = m_order[dir] - 2;
            int i1 = m_cv_count[dir] - 1;
            
            for (int k = 0; k < m_cv_count[1 - dir]; k++) {
                int check_i0 = i0;
                int check_i1 = i1;
                cv0 = dir ? cv(k, check_i0) : cv(check_i0, k);
                cv1 = dir ? cv(k, check_i1) : cv(check_i1, k);
                
                while (check_i0 >= 0) {
                    if (!points_are_coincident(m_dim, m_is_rat, cv0, cv1)) {
                        return false;
                    }
                    check_i0--;
                    check_i1--;
                    if (check_i0 >= 0) {
                        cv0 -= m_cv_stride[dir];
                        cv1 -= m_cv_stride[dir];
                    }
                }
            }
        }
    }
    return bIsPeriodic;
}

bool NurbsSurface::is_singular(int side) const {
    bool rc = false;
    const double* points = nullptr;
    int point_count = 0;
    int point_stride = 0;
    
    switch (side) {
    case 0: // south
        rc = is_clamped(1, 0);
        if (rc) {
            points = cv(0, 0);
            point_count = m_cv_count[0];
            point_stride = m_cv_stride[0];
        }
        break;
        
    case 1: // east
        rc = is_clamped(0, 1);
        if (rc) {
            points = cv(m_cv_count[0] - 1, 0);
            point_count = m_cv_count[1];
            point_stride = m_cv_stride[1];
        }
        break;
        
    case 2: // north
        rc = is_clamped(1, 1);
        if (rc) {
            points = cv(0, m_cv_count[1] - 1);
            point_count = m_cv_count[0];
            point_stride = m_cv_stride[0];
        }
        break;
        
    case 3: // west
        rc = is_clamped(0, 0);
        if (rc) {
            points = cv(0, 0);
            point_count = m_cv_count[1];
            point_stride = m_cv_stride[1];
        }
        break;
        
    default:
        rc = false;
        break;
    }
    
    if (rc && points) {
        rc = points_are_coincident(m_dim, m_is_rat, point_count, point_stride, points);
    }
    
    return rc;
}

std::pair<std::vector<std::vector<Point>>, std::vector<std::vector<std::pair<double,double>>>>
NurbsSurface::divide_by_count(int nu, int nv) const {
    std::vector<std::vector<Point>> grid;
    std::vector<std::vector<std::pair<double,double>>> params;

    if (!is_valid()) {
        return {grid, params};
    }

    auto domain_u = domain(0);
    auto domain_v = domain(1);
    double u0 = domain_u.first;
    double u1 = domain_u.second;
    double v0 = domain_v.first;
    double v1 = domain_v.second;

    grid.resize(nu + 1);
    params.resize(nu + 1);

    for (int i = 0; i <= nu; i++) {
        grid[i].resize(nv + 1);
        params[i].resize(nv + 1);

        double u = (nu > 0) ? (u0 + (u1 - u0) * (static_cast<double>(i) / nu)) : u0;

        for (int j = 0; j <= nv; j++) {
            double v = (nv > 0) ? (v0 + (v1 - v0) * (static_cast<double>(j) / nv)) : v0;
            grid[i][j] = point_at(u, v);
            params[i][j] = {u, v};
        }
    }

    return {grid, params};
}

bool NurbsSurface::is_planar(Plane* plane, double tolerance) const {
    if (!is_valid()) return false;
    
    // Check if all control points are coplanar
    if (m_cv_count[0] < 2 || m_cv_count[1] < 2) return false;
    
    // For 2x2 or smaller, all points define a plane (4 points always coplanar in practice)
    if (m_cv_count[0] <= 2 && m_cv_count[1] <= 2) return true;
    
    // Get three non-colinear points
    Point p0 = get_cv(0, 0);
    Point p1 = get_cv(m_cv_count[0] - 1, 0);
    Point p2 = get_cv(0, m_cv_count[1] - 1);
    
    Vector v1(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Vector v2(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
    Vector normal = v1.cross(v2);
    
    double len = normal.magnitude();
    if (len < 1e-14) return false;
    
    normal = normal / len;
    
    // Check all CVs against this plane
    for (int i = 0; i < m_cv_count[0]; i++) {
        for (int j = 0; j < m_cv_count[1]; j++) {
            Point pt = get_cv(i, j);
            Vector v(pt[0] - p0[0], pt[1] - p0[1], pt[2] - p0[2]);
            double dist = std::abs(v.dot(normal));
            if (dist > tolerance) return false;
        }
    }
    
    if (plane) {
        *plane = Plane::from_point_normal(p0, normal);
    }
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Knot Insertion (implementation after helper functions to allow access)
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

// Actual implementation of knot insertion using helper functions
bool insert_knot_impl(NurbsSurface& srf, int dir, double knot_value, int knot_multiplicity) {
    if ((dir != 0 && dir != 1) || !srf.is_valid() || knot_multiplicity <= 0) {
        return false;
    }

    if (knot_multiplicity >= srf.m_order[dir]) {
        return false; // Multiplicity must be less than order
    }

    // Check that knot_value is inside domain
    auto [t0, t1] = srf.domain(dir);
    if (knot_value < t0 || knot_value > t1) {
        return false; // Knot value must be inside domain
    }

    // Convert surface to high-dimensional curve
    NurbsCurve* crv = to_curve_internal(srf, dir, nullptr);
    if (!crv) {
        return false;
    }

    // Insert knots into the curve using Boehm's algorithm
    bool success = crv->insert_knot(knot_value, knot_multiplicity);

    if (success) {
        // Convert curve back to surface
        success = from_curve_internal(*crv, srf, dir);
    }

    delete crv;
    return success;
}

bool trim_impl(NurbsSurface& srf, int dir, const std::pair<double, double>& domain_pair) {
    if (dir < 0 || dir > 1 || !srf.is_valid()) return false;

    NurbsCurve* crv = to_curve_internal(srf, dir, nullptr);
    if (!crv) return false;

    bool ok = crv->trim(domain_pair.first, domain_pair.second);
    if (ok) ok = from_curve_internal(*crv, srf, dir);

    delete crv;
    return ok;
}

bool split_impl(const NurbsSurface& srf, int dir, double c,
                NurbsSurface*& lo, NurbsSurface*& hi) {
    if (dir < 0 || dir > 1 || !srf.is_valid()) return false;

    auto [t0, t1] = srf.domain(dir);
    if (c <= t0 || c >= t1) return false;

    lo = new NurbsSurface(srf);
    hi = new NurbsSurface(srf);

    if (!lo->trim(dir, {t0, c}) || !hi->trim(dir, {c, t1})) {
        delete lo; lo = nullptr;
        delete hi; hi = nullptr;
        return false;
    }
    return true;
}

} // anonymous namespace

///////////////////////////////////////////////////////////////////////////////////////////
// Loft, Revolve, Sweep1, Sweep2
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

// Merge two knot vectors into a unified knot vector containing all knots from both
std::vector<double> merge_knot_vectors(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-10) {
    std::vector<double> merged;
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        if (std::abs(a[i] - b[j]) < tol) {
            merged.push_back(a[i]);
            i++; j++;
        } else if (a[i] < b[j]) {
            merged.push_back(a[i]);
            i++;
        } else {
            merged.push_back(b[j]);
            j++;
        }
    }
    while (i < a.size()) { merged.push_back(a[i]); i++; }
    while (j < b.size()) { merged.push_back(b[j]); j++; }
    return merged;
}

// Check if two knot vectors are identical within tolerance
bool knot_vectors_equal(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-10) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) {
        if (std::abs(a[i] - b[i]) > tol) return false;
    }
    return true;
}

// Make a set of curves compatible: same degree, same knot vector
void make_curves_compatible(std::vector<NurbsCurve>& curves) {
    if (curves.size() < 2) return;

    // Find max degree
    int max_deg = 0;
    for (auto& c : curves) {
        if (c.degree() > max_deg) max_deg = c.degree();
    }

    // Degree elevate all curves to max degree
    for (auto& c : curves) {
        if (c.degree() < max_deg) c.increase_degree(max_deg);
    }

    // Make all rational if any is rational
    bool any_rational = false;
    for (auto& c : curves) {
        if (c.is_rational()) { any_rational = true; break; }
    }
    if (any_rational) {
        for (auto& c : curves) c.make_rational();
    }

    // Check if all curves already have same knot vectors and cv counts
    bool already_compatible = true;
    for (size_t i = 1; i < curves.size(); i++) {
        if (curves[i].cv_count() != curves[0].cv_count() ||
            !knot_vectors_equal(curves[i].get_knots(), curves[0].get_knots())) {
            already_compatible = false;
            break;
        }
    }
    if (already_compatible) return;

    // Reparameterize all curves to [0,1]
    for (auto& c : curves) {
        c.set_domain(0.0, 1.0);
    }

    // Merge knot vectors iteratively
    std::vector<double> unified = curves[0].get_knots();
    for (size_t i = 1; i < curves.size(); i++) {
        unified = merge_knot_vectors(unified, curves[i].get_knots());
    }

    // Insert missing knots into each curve
    const double tol = 1e-10;
    for (auto& c : curves) {
        std::vector<double> cur_knots = c.get_knots();
        for (double k : unified) {
            bool found = false;
            for (double ck : cur_knots) {
                if (std::abs(ck - k) < tol) { found = true; break; }
            }
            if (!found) {
                c.insert_knot(k, 1);
                cur_knots = c.get_knots();
            }
        }
    }
}

} // anonymous namespace

NurbsSurface NurbsSurface::create_loft(const std::vector<NurbsCurve>& input_curves, int degree_v) {
    NurbsSurface surface;
    if (input_curves.size() < 2) return surface;

    // Validate all curves
    for (auto& c : input_curves) {
        if (!c.is_valid()) return surface;
    }

    // Copy and make compatible
    std::vector<NurbsCurve> curves = input_curves;
    make_curves_compatible(curves);

    int n_sections = static_cast<int>(curves.size());
    int cv_count_u = curves[0].cv_count();
    int order_u = curves[0].order();
    bool is_rat = curves[0].is_rational();

    // Clamp degree_v to valid range
    if (degree_v >= n_sections) degree_v = n_sections - 1;
    if (degree_v < 1) degree_v = 1;
    int order_v = degree_v + 1;

    // Compute v-parameters using chord-length between section midpoints
    std::vector<double> v_params(n_sections, 0.0);
    for (int k = 1; k < n_sections; k++) {
        Point pk_prev = curves[k - 1].point_at_middle();
        Point pk_curr = curves[k].point_at_middle();
        double dx = pk_curr[0] - pk_prev[0];
        double dy = pk_curr[1] - pk_prev[1];
        double dz = pk_curr[2] - pk_prev[2];
        v_params[k] = v_params[k - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    // Normalize to [0,1]
    double total_len = v_params.back();
    if (total_len > 1e-14) {
        for (int k = 0; k < n_sections; k++) v_params[k] /= total_len;
    } else {
        // Uniform fallback
        for (int k = 0; k < n_sections; k++) v_params[k] = static_cast<double>(k) / (n_sections - 1);
    }

    // Build v-direction knot vector for interpolation with n_sections CVs
    // (NOT using build_interp_knots which adds extra CVs for natural end conditions)
    int cv_count_v = n_sections;
    int knot_count_v = order_v + cv_count_v - 2;
    std::vector<double> knots_v(knot_count_v);

    if (degree_v >= n_sections - 1) {
        // Bezier case: clamped at both ends
        // For degree d, cv_count = d+1: knot_count = d + (d+1) - 2 + 2 = 2d
        // No wait: knot_count = order + cv_count - 2
        // degree-1 zeros, then degree-1 ones
        int d = degree_v;
        for (int i = 0; i < d; i++) knots_v[i] = 0.0;
        for (int i = d; i < knot_count_v; i++) knots_v[i] = 1.0;
    } else {
        // Clamped start
        for (int i = 0; i < order_v - 1; i++) knots_v[i] = v_params[0];
        // Interior knots using averaging (Piegl & Tiller method)
        for (int j = 1; j <= n_sections - order_v; j++) {
            double sum = 0.0;
            for (int i = j; i < j + degree_v; i++) sum += v_params[i];
            knots_v[order_v - 2 + j] = sum / degree_v;
        }
        // Clamped end
        for (int i = knot_count_v - order_v + 1; i < knot_count_v; i++) knots_v[i] = v_params[n_sections - 1];
    }

    // Create the surface
    surface.create_raw(3, is_rat, order_u, order_v, cv_count_u, cv_count_v);

    // Copy u-knots from the first (compatible) curve
    for (int i = 0; i < surface.knot_count(0); i++) {
        surface.set_knot(0, i, curves[0].knot(i));
    }

    // Set v-knots
    for (int i = 0; i < static_cast<int>(knots_v.size()) && i < surface.knot_count(1); i++) {
        surface.set_knot(1, i, knots_v[i]);
    }

    // For each u-index, interpolate through the section curve CVs in the v-direction
    // When degree_v == n_sections - 1, the knot vector is fully clamped and interpolation
    // is exact, so we can just copy CVs directly. For lower degrees, we need to solve.
    if (degree_v == n_sections - 1) {
        // Direct copy: each section's CVs become a row of the surface
        for (int k = 0; k < n_sections; k++) {
            for (int i = 0; i < cv_count_u; i++) {
                if (is_rat) {
                    double x, y, z, w;
                    curves[k].get_cv_4d(i, x, y, z, w);
                    surface.set_cv_4d(i, k, x, y, z, w);
                } else {
                    surface.set_cv(i, k, curves[k].get_cv(i));
                }
            }
        }
    } else {
        // Solve B-spline interpolation in v-direction for each u-index
        // Build basis matrix N[k][j] = N_j(v_params[k])
        // We need to evaluate basis functions at each v_param

        // For non-rational case, solve per u-index:
        // For each i in [0, cv_count_u), collect points P[k] = curve[k].get_cv(i)
        // Solve: sum_j N_j(v_k) * Q[j] = P[k] for all k
        // This is a general linear system (not necessarily tridiagonal for arbitrary degree)

        // Build the basis matrix
        int n = n_sections;
        std::vector<std::vector<double>> N_matrix(n, std::vector<double>(n, 0.0));

        // Create a temporary curve to evaluate basis functions
        NurbsCurve temp_crv(1, false, order_v, cv_count_v);
        for (int i = 0; i < static_cast<int>(knots_v.size()); i++) {
            temp_crv.set_knot(i, knots_v[i]);
        }

        for (int k = 0; k < n; k++) {
            // Evaluate all basis functions at v_params[k]
            double t = v_params[k];
            // Clamp t to domain
            auto [t0, t1] = temp_crv.domain();
            if (t < t0) t = t0;
            if (t > t1) t = t1;

            // Find span and compute basis
            int span = knot::find_span(order_v, cv_count_v, knots_v, t);
            std::vector<double> basis;
            // Use NurbsSurface's internal method pattern — replicate basis computation
            int d = order_v - 1;
            int knot_base = span + d;

            if (knots_v[knot_base - 1] == knots_v[knot_base]) {
                // Degenerate
                continue;
            }

            std::vector<double> Nvals(order_v * order_v, 0.0);
            Nvals[order_v * order_v - 1] = 1.0;
            std::vector<double> left(d), right(d);
            int N_idx = order_v * order_v - 1;
            int k_right = knot_base;
            int k_left = knot_base - 1;

            for (int j = 0; j < d; j++) {
                int N0_idx = N_idx;
                N_idx -= (order_v + 1);
                left[j] = t - knots_v[k_left];
                right[j] = knots_v[k_right] - t;
                k_left--;
                k_right++;

                double x = 0.0;
                for (int r = 0; r <= j; r++) {
                    double a0 = left[j - r];
                    double a1 = right[r];
                    double y = Nvals[N0_idx + r] / (a0 + a1);
                    Nvals[N_idx + r] = x + a1 * y;
                    x = a0 * y;
                }
                Nvals[N_idx + j + 1] = x;
            }

            for (int j = 0; j < order_v; j++) {
                int col = span + j;
                if (col >= 0 && col < n) {
                    N_matrix[k][col] = Nvals[j];
                }
            }
        }

        // Solve the system N_matrix * Q = P for each u-index using Gaussian elimination
        int dim = is_rat ? 4 : 3;
        for (int i = 0; i < cv_count_u; i++) {
            // Collect RHS: points from each section curve at u-index i
            std::vector<std::vector<double>> rhs(n, std::vector<double>(dim, 0.0));
            for (int k = 0; k < n; k++) {
                if (is_rat) {
                    double x, y, z, w;
                    curves[k].get_cv_4d(i, x, y, z, w);
                    rhs[k] = {x, y, z, w};
                } else {
                    Point p = curves[k].get_cv(i);
                    rhs[k] = {p[0], p[1], p[2]};
                }
            }

            // Gaussian elimination with partial pivoting
            std::vector<std::vector<double>> A = N_matrix;
            std::vector<std::vector<double>> b = rhs;

            for (int col = 0; col < n; col++) {
                // Find pivot
                int max_row = col;
                double max_val = std::abs(A[col][col]);
                for (int row = col + 1; row < n; row++) {
                    if (std::abs(A[row][col]) > max_val) {
                        max_val = std::abs(A[row][col]);
                        max_row = row;
                    }
                }
                if (max_val < 1e-14) continue;

                std::swap(A[col], A[max_row]);
                std::swap(b[col], b[max_row]);

                for (int row = col + 1; row < n; row++) {
                    double factor = A[row][col] / A[col][col];
                    for (int c = col; c < n; c++) {
                        A[row][c] -= factor * A[col][c];
                    }
                    for (int d2 = 0; d2 < dim; d2++) {
                        b[row][d2] -= factor * b[col][d2];
                    }
                }
            }

            // Back substitution
            std::vector<std::vector<double>> Q(n, std::vector<double>(dim, 0.0));
            for (int row = n - 1; row >= 0; row--) {
                for (int d2 = 0; d2 < dim; d2++) {
                    Q[row][d2] = b[row][d2];
                    for (int c = row + 1; c < n; c++) {
                        Q[row][d2] -= A[row][c] * Q[c][d2];
                    }
                    if (std::abs(A[row][row]) > 1e-14) {
                        Q[row][d2] /= A[row][row];
                    }
                }
            }

            // Set surface CVs
            for (int j = 0; j < n; j++) {
                if (is_rat) {
                    surface.set_cv_4d(i, j, Q[j][0], Q[j][1], Q[j][2], Q[j][3]);
                } else {
                    surface.set_cv(i, j, Point(Q[j][0], Q[j][1], Q[j][2]));
                }
            }
        }
    }

    return surface;
}

NurbsSurface NurbsSurface::create_revolve(const NurbsCurve& profile, const Point& axis_origin,
                                            const Vector& axis_direction, double angle) {
    NurbsSurface surface;
    if (!profile.is_valid()) return surface;

    // Normalize axis direction
    double ax_len = axis_direction.magnitude();
    if (ax_len < 1e-14) return surface;
    Vector axis_dir = axis_direction / ax_len;

    // Clamp angle to (0, 2*PI]
    double PI = Tolerance::PI;
    if (angle < 0) angle = -angle;
    if (angle > 2.0 * PI) angle = 2.0 * PI;
    if (angle < 1e-14) return surface;

    // Determine number of arcs and u-CPs
    int n_arcs;
    if (angle <= PI / 2.0 + 1e-10) n_arcs = 1;
    else if (angle <= PI + 1e-10) n_arcs = 2;
    else if (angle <= 3.0 * PI / 2.0 + 1e-10) n_arcs = 3;
    else n_arcs = 4;

    double d_theta = angle / n_arcs;
    double w_mid = std::cos(d_theta / 2.0);
    int n_u = 2 * n_arcs + 1;

    // Build u-knots
    std::vector<double> knots_u;
    knots_u.push_back(0.0);
    knots_u.push_back(0.0);
    for (int i = 1; i <= n_arcs; i++) {
        double kv = i * d_theta;
        knots_u.push_back(kv);
        knots_u.push_back(kv);
    }
    // The last two are already pushed; we need to structure it as clamped
    // Clamped degree-2: first 2 and last 2 knots are repeated
    // For n_arcs arcs: knot vector has 2*n_arcs + 2 entries
    // Actually for degree 2 with n_u CVs: knot_count = n_u + 2 - 2 = n_u
    // n_u = 2*n_arcs + 1, so knot_count = 2*n_arcs + 1
    // Rebuild properly
    knots_u.clear();
    // order=3, cv_count=n_u, knot_count = 3 + n_u - 2 = n_u + 1
    int knot_count_u = n_u + 1;
    knots_u.resize(knot_count_u);
    knots_u[0] = 0.0;
    knots_u[1] = 0.0;
    for (int i = 1; i <= n_arcs; i++) {
        double kv = i * d_theta;
        knots_u[2 * i] = kv;
        knots_u[2 * i + 1] = kv;
    }
    // The last knot should equal angle
    knots_u[knot_count_u - 1] = angle;
    knots_u[knot_count_u - 2] = angle;

    // Profile parameters
    int cv_count_v = profile.cv_count();
    int order_v = profile.order();
    bool profile_rational = profile.is_rational();

    // Surface is always rational (circle arcs need weights)
    surface.create_raw(3, true, 3, order_v, n_u, cv_count_v);

    // Set u-knots
    for (int i = 0; i < knot_count_u && i < surface.knot_count(0); i++) {
        surface.set_knot(0, i, knots_u[i]);
    }

    // Set v-knots from profile
    for (int i = 0; i < profile.knot_count() && i < surface.knot_count(1); i++) {
        surface.set_knot(1, i, profile.knot(i));
    }

    // Precompute arc angles and weights for each u-CP
    std::vector<double> u_angles(n_u);
    std::vector<double> u_weights(n_u);
    for (int i = 0; i < n_u; i++) {
        if (i % 2 == 0) {
            // On-arc point
            u_angles[i] = (i / 2) * d_theta;
            u_weights[i] = 1.0;
        } else {
            // Mid-arc point
            u_angles[i] = (i / 2) * d_theta + d_theta / 2.0;
            u_weights[i] = w_mid;
        }
    }

    // For each profile CV, generate the revolution CVs
    for (int j = 0; j < cv_count_v; j++) {
        Point P_j = profile.get_cv(j);
        double profile_w = profile_rational ? profile.weight(j) : 1.0;

        // Project P_j onto axis to find O_j
        double dx = P_j[0] - axis_origin[0];
        double dy = P_j[1] - axis_origin[1];
        double dz = P_j[2] - axis_origin[2];
        double proj = dx * axis_dir[0] + dy * axis_dir[1] + dz * axis_dir[2];
        Point O_j(axis_origin[0] + proj * axis_dir[0],
                  axis_origin[1] + proj * axis_dir[1],
                  axis_origin[2] + proj * axis_dir[2]);

        // Compute radius
        double rx = P_j[0] - O_j[0];
        double ry = P_j[1] - O_j[1];
        double rz = P_j[2] - O_j[2];
        double r_j = std::sqrt(rx * rx + ry * ry + rz * rz);

        if (r_j < 1e-14) {
            // Degenerate: point is on axis — all u-CPs collapse
            for (int i = 0; i < n_u; i++) {
                double combined_w = u_weights[i] * profile_w;
                surface.set_cv(i, j, O_j);
                surface.set_weight(i, j, combined_w);
            }
        } else {
            // Local coordinate system perpendicular to axis
            Vector x_local(rx / r_j, ry / r_j, rz / r_j);
            Vector y_local = axis_dir.cross(x_local);
            double y_len = y_local.magnitude();
            if (y_len > 1e-14) {
                y_local = y_local / y_len;
            }

            for (int i = 0; i < n_u; i++) {
                double theta = u_angles[i];
                double cos_t = std::cos(theta);
                double sin_t = std::sin(theta);

                double effective_r = r_j;
                if (i % 2 == 1) {
                    // Mid-arc point: radius divided by cos(half-arc) to maintain circle
                    effective_r = r_j / w_mid;
                }

                double px = O_j[0] + effective_r * (cos_t * x_local[0] + sin_t * y_local[0]);
                double py = O_j[1] + effective_r * (cos_t * x_local[1] + sin_t * y_local[1]);
                double pz = O_j[2] + effective_r * (cos_t * x_local[2] + sin_t * y_local[2]);

                double combined_w = u_weights[i] * profile_w;
                // Store homogeneous coordinates
                surface.set_cv_4d(i, j, px * combined_w, py * combined_w, pz * combined_w, combined_w);
            }
        }
    }

    return surface;
}

NurbsSurface NurbsSurface::create_sweep1(const NurbsCurve& rail, const NurbsCurve& profile) {
    NurbsSurface surface;
    if (!rail.is_valid() || !profile.is_valid()) return surface;

    // Sample RMF frames along rail
    int N = std::max(rail.cv_count() * 2, 20);
    std::vector<Plane> frames = rail.get_perpendicular_planes(N);
    if (frames.empty()) return surface;

    // Get profile's local frame at its start
    Point profile_origin = profile.point_at_start();

    // Position profile copies at each frame
    std::vector<NurbsCurve> positioned_profiles;
    positioned_profiles.reserve(frames.size());

    for (size_t i = 0; i < frames.size(); i++) {
        NurbsCurve prof_copy = profile;
        Plane& frame = frames[i];
        Point frame_origin = frame.origin();
        Vector frame_x = frame.x_axis();
        Vector frame_y = frame.y_axis();
        Vector frame_z = frame.z_axis();

        // Build transformation: profile local space -> frame
        // Profile is in world space, so we translate to origin, then map to frame
        Xform xf = Xform::identity();

        // Step 1: Translate profile so its start is at origin
        Xform t1 = Xform::translation(-profile_origin[0], -profile_origin[1], -profile_origin[2]);

        // Step 2: Rotate + scale to align with frame
        // Map standard axes to frame axes
        Xform rot = Xform::identity();
        rot.m[0] = frame_x[0]; rot.m[1] = frame_y[0]; rot.m[2] = frame_z[0]; rot.m[3] = frame_origin[0];
        rot.m[4] = frame_x[1]; rot.m[5] = frame_y[1]; rot.m[6] = frame_z[1]; rot.m[7] = frame_origin[1];
        rot.m[8] = frame_x[2]; rot.m[9] = frame_y[2]; rot.m[10] = frame_z[2]; rot.m[11] = frame_origin[2];
        rot.m[12] = 0; rot.m[13] = 0; rot.m[14] = 0; rot.m[15] = 1;

        // Compose: first translate to origin, then apply rotation+translation
        // Combined: rot * t1
        // Apply translation first, then rotation
        prof_copy.transform(t1);
        prof_copy.transform(rot);

        positioned_profiles.push_back(prof_copy);
    }

    // Loft through positioned profiles
    int loft_degree = std::min(3, static_cast<int>(positioned_profiles.size()) - 1);
    surface = create_loft(positioned_profiles, loft_degree);

    return surface;
}

NurbsSurface NurbsSurface::create_sweep2(const NurbsCurve& rail1, const NurbsCurve& rail2,
                                           const NurbsCurve& profile) {
    NurbsSurface surface;
    if (!rail1.is_valid() || !rail2.is_valid() || !profile.is_valid()) return surface;

    // Sample both rails at corresponding parameters
    int N = std::max(std::max(rail1.cv_count(), rail2.cv_count()) * 2, 20);

    std::vector<Point> pts1, pts2;
    std::vector<double> params1, params2;
    rail1.divide_by_count(N + 1, pts1, &params1, true);
    rail2.divide_by_count(N + 1, pts2, &params2, true);

    // Get RMF frames on rail1
    std::vector<Plane> frames1 = rail1.get_perpendicular_planes(N);
    if (frames1.empty()) return surface;

    // Profile info
    Point profile_start = profile.point_at_start();
    Point profile_end = profile.point_at_end();
    double profile_width = 0.0;
    {
        double dx = profile_end[0] - profile_start[0];
        double dy = profile_end[1] - profile_start[1];
        double dz = profile_end[2] - profile_start[2];
        profile_width = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    if (profile_width < 1e-14) profile_width = 1.0;

    // Position and scale profile at each sample
    std::vector<NurbsCurve> positioned_profiles;
    positioned_profiles.reserve(frames1.size());

    for (size_t i = 0; i < frames1.size() && i < pts1.size() && i < pts2.size(); i++) {
        Point p1 = pts1[i];
        Point p2 = pts2[i];

        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];
        double dz = p2[2] - p1[2];
        double rail_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        double scale_factor = (rail_dist > 1e-14) ? rail_dist / profile_width : 1.0;

        // Midpoint between rails
        Point mid((p1[0] + p2[0]) * 0.5, (p1[1] + p2[1]) * 0.5, (p1[2] + p2[2]) * 0.5);

        NurbsCurve prof_copy = profile;

        // Translate profile start to origin
        Xform t1 = Xform::translation(-profile_start[0], -profile_start[1], -profile_start[2]);
        prof_copy.transform(t1);

        // Scale
        Xform sc = Xform::scale_xyz(scale_factor, scale_factor, scale_factor);
        prof_copy.transform(sc);

        // Build orientation: x-axis from rail1 to rail2
        Plane& frame = frames1[i];
        Vector frame_z = frame.z_axis(); // tangent along rail

        Vector x_dir(dx, dy, dz);
        double x_len = x_dir.magnitude();
        if (x_len > 1e-14) x_dir = x_dir / x_len;
        else x_dir = frame.x_axis();

        Vector y_dir = frame_z.cross(x_dir);
        double y_len = y_dir.magnitude();
        if (y_len > 1e-14) y_dir = y_dir / y_len;
        else y_dir = frame.y_axis();

        // Rotation matrix
        Xform rot = Xform::identity();
        rot.m[0] = x_dir[0]; rot.m[1] = y_dir[0]; rot.m[2] = frame_z[0]; rot.m[3] = p1[0];
        rot.m[4] = x_dir[1]; rot.m[5] = y_dir[1]; rot.m[6] = frame_z[1]; rot.m[7] = p1[1];
        rot.m[8] = x_dir[2]; rot.m[9] = y_dir[2]; rot.m[10] = frame_z[2]; rot.m[11] = p1[2];
        rot.m[12] = 0; rot.m[13] = 0; rot.m[14] = 0; rot.m[15] = 1;

        prof_copy.transform(rot);
        positioned_profiles.push_back(prof_copy);
    }

    int loft_degree = std::min(3, static_cast<int>(positioned_profiles.size()) - 1);
    surface = create_loft(positioned_profiles, loft_degree);

    return surface;
}

} // namespace session_cpp
