#include "nurbssurface.h"
#include "trimesh_grid.h"
#include "trimesh_delaunay.h"
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

static void eval_basis_stack(const double* knot, int order, int span, double t, double* out) {
    int d = order - 1;
    int kb = span + d;
    // Clamped boundary: degenerate initial interval -> basis is endpoint function
    if (knot[kb - 1] == knot[kb]) {
        for (int i = 0; i < order; i++) out[i] = 0;
        if (t <= knot[kb]) out[0] = 1.0;
        else out[order - 1] = 1.0;
        return;
    }
    double N[100];
    std::memset(N, 0, order * order * sizeof(double));
    N[order * order - 1] = 1.0;
    double left[10], right[10];
    int N_idx = order * order - 1;
    int kr = kb, kl = kb - 1;
    for (int j = 0; j < d; j++) {
        int N0 = N_idx;
        N_idx -= (order + 1);
        left[j] = t - knot[kl];
        right[j] = knot[kr] - t;
        kl--; kr++;
        double x = 0.0;
        for (int r = 0; r <= j; r++) {
            double denom = left[j - r] + right[r];
            double y = (std::abs(denom) > 0.0) ? N[N0 + r] / denom : 0.0;
            N[N_idx + r] = x + right[r] * y;
            x = left[j - r] * y;
        }
        N[N_idx + j + 1] = x;
    }
    for (int i = 0; i < order; i++) out[i] = N[i];
}

static void eval_basis_derivs_stack(const double* knot, int order, int span, double t,
                                     double* basis, double* deriv) {
    int degree = order - 1;
    int kb = span + degree;
    for (int i = 0; i < order; i++) { basis[i] = 0; deriv[i] = 0; }
    if (knot[kb - 1] == knot[kb]) {
        if (t <= knot[kb]) basis[0] = 1.0;
        else basis[order - 1] = 1.0;
        return;
    }

    double ndu[10][10];
    std::memset(ndu, 0, order * order * sizeof(double));
    ndu[0][0] = 1.0;
    double left[10], right[10];
    for (int j = 1; j <= degree; j++) {
        left[j] = t - knot[kb - j];
        right[j] = knot[kb + j - 1] - t;
        double saved = 0.0;
        for (int r = 0; r < j; r++) {
            ndu[j][r] = right[r + 1] + left[j - r];
            double temp = (std::abs(ndu[j][r]) > 0.0) ? ndu[r][j - 1] / ndu[j][r] : 0.0;
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for (int j = 0; j <= degree; j++) basis[j] = ndu[j][degree];

    double a[2][10];
    std::memset(a, 0, 2 * order * sizeof(double));
    for (int r = 0; r <= degree; r++) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        double d = 0.0;
        int rk = r - 1, pk = degree - 1;
        if (r >= 1) {
            double denom = ndu[pk + 1][rk];
            a[s2][0] = (std::abs(denom) > 0.0) ? a[s1][0] / denom : 0.0;
            d = a[s2][0] * ndu[rk][pk];
        }
        int j1 = (rk >= -1) ? 1 : -rk;
        int j2 = (r - 1 <= pk) ? 0 : degree - r;
        for (int j = j1; j <= j2; j++) {
            double denom = ndu[pk + 1][rk + j];
            a[s2][j] = (std::abs(denom) > 0.0) ? (a[s1][j] - a[s1][j - 1]) / denom : 0.0;
            d += a[s2][j] * ndu[rk + j][pk];
        }
        if (r <= pk) {
            double denom = ndu[pk + 1][r];
            a[s2][1] = (std::abs(denom) > 0.0) ? -a[s1][0] / denom : 0.0;
            d += a[s2][1] * ndu[r][pk];
        }
        deriv[r] = d;
        std::memset(a[0], 0, order * sizeof(double));
        std::memset(a[1], 0, order * sizeof(double));
    }
    for (int j = 0; j <= degree; j++) deriv[j] *= degree;
}

void NurbsSurface::point_at(double u, double v, double& ox, double& oy, double& oz) const {
    if (!is_valid()) { ox = oy = oz = 0; return; }
    int su = find_span(0, u), sv = find_span(1, v);
    double Nu[10], Nv[10];
    eval_basis_stack(m_knot[0].data(), m_order[0], su, u, Nu);
    eval_basis_stack(m_knot[1].data(), m_order[1], sv, v, Nv);
    int cvs = m_is_rat ? (m_dim + 1) : m_dim;
    double pt[4] = {};
    for (int i = 0; i < m_order[0]; i++)
        for (int j = 0; j < m_order[1]; j++) {
            double c = Nu[i] * Nv[j];
            const double* p = cv(su + i, sv + j);
            if (p) for (int d = 0; d < cvs; d++) pt[d] += c * p[d];
        }
    if (m_is_rat && std::abs(pt[m_dim]) > 1e-14) {
        double w = pt[m_dim]; ox = pt[0]/w; oy = pt[1]/w; oz = pt[2]/w;
    } else {
        ox = pt[0]; oy = m_dim > 1 ? pt[1] : 0; oz = m_dim > 2 ? pt[2] : 0;
    }
}

void NurbsSurface::point_and_normal_at(double u, double v,
                                       double& px, double& py, double& pz,
                                       double& nx, double& ny, double& nz) const {
    if (!is_valid()) { px = py = pz = 0; nx = 0; ny = 0; nz = 1; return; }
    int su = find_span(0, u), sv = find_span(1, v);
    double Nu0[10], Nu1[10], Nv0[10], Nv1[10];
    eval_basis_derivs_stack(m_knot[0].data(), m_order[0], su, u, Nu0, Nu1);
    eval_basis_derivs_stack(m_knot[1].data(), m_order[1], sv, v, Nv0, Nv1);
    int cvs = m_is_rat ? (m_dim + 1) : m_dim;
    double S0[4] = {}, Su[4] = {}, Sv[4] = {};
    for (int i = 0; i < m_order[0]; i++)
        for (int j = 0; j < m_order[1]; j++) {
            const double* p = cv(su + i, sv + j);
            if (!p) continue;
            double c0 = Nu0[i] * Nv0[j], cu = Nu1[i] * Nv0[j], cv_c = Nu0[i] * Nv1[j];
            for (int d = 0; d < cvs; d++) {
                S0[d] += c0 * p[d]; Su[d] += cu * p[d]; Sv[d] += cv_c * p[d];
            }
        }
    double sux, suy, suz, svx, svy, svz;
    if (m_is_rat && std::abs(S0[m_dim]) > 1e-14) {
        double w = S0[m_dim];
        px = S0[0]/w; py = S0[1]/w; pz = S0[2]/w;
        sux = Su[0]/w; suy = Su[1]/w; suz = Su[2]/w;
        svx = Sv[0]/w; svy = Sv[1]/w; svz = Sv[2]/w;
    } else {
        px = S0[0]; py = m_dim > 1 ? S0[1] : 0; pz = m_dim > 2 ? S0[2] : 0;
        sux = Su[0]; suy = m_dim > 1 ? Su[1] : 0; suz = m_dim > 2 ? Su[2] : 0;
        svx = Sv[0]; svy = m_dim > 1 ? Sv[1] : 0; svz = m_dim > 2 ? Sv[2] : 0;
    }
    nx = suy * svz - suz * svy;
    ny = suz * svx - sux * svz;
    nz = sux * svy - suy * svx;
    double len = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (len < 1e-14) { nx = 0; ny = 0; nz = 1; return; }
    nx /= len; ny /= len; nz /= len;
}

void NurbsSurface::normal_at(double u, double v, double& nx, double& ny, double& nz) const {
    if (!is_valid()) { nx = 0; ny = 0; nz = 1; return; }
    int su = find_span(0, u), sv = find_span(1, v);
    double Nu0[10], Nu1[10], Nv0[10], Nv1[10];
    eval_basis_derivs_stack(m_knot[0].data(), m_order[0], su, u, Nu0, Nu1);
    eval_basis_derivs_stack(m_knot[1].data(), m_order[1], sv, v, Nv0, Nv1);
    int cvs = m_is_rat ? (m_dim + 1) : m_dim;
    double Su[4] = {}, Sv[4] = {};
    for (int i = 0; i < m_order[0]; i++)
        for (int j = 0; j < m_order[1]; j++) {
            const double* p = cv(su + i, sv + j);
            if (!p) continue;
            double cu = Nu1[i] * Nv0[j], cv_c = Nu0[i] * Nv1[j];
            for (int d = 0; d < cvs; d++) { Su[d] += cu * p[d]; Sv[d] += cv_c * p[d]; }
        }
    double sux, suy, suz, svx, svy, svz;
    if (m_is_rat) {
        double S0[4] = {};
        for (int i = 0; i < m_order[0]; i++)
            for (int j = 0; j < m_order[1]; j++) {
                const double* p = cv(su + i, sv + j);
                if (!p) continue;
                double c = Nu0[i] * Nv0[j];
                for (int d = 0; d < cvs; d++) S0[d] += c * p[d];
            }
        double w = S0[m_dim]; if (std::abs(w) < 1e-14) { nx = 0; ny = 0; nz = 1; return; }
        sux = Su[0]/w; suy = Su[1]/w; suz = Su[2]/w;
        svx = Sv[0]/w; svy = Sv[1]/w; svz = Sv[2]/w;
    } else {
        sux = Su[0]; suy = m_dim > 1 ? Su[1] : 0; suz = m_dim > 2 ? Su[2] : 0;
        svx = Sv[0]; svy = m_dim > 1 ? Sv[1] : 0; svz = m_dim > 2 ? Sv[2] : 0;
    }
    nx = suy * svz - suz * svy;
    ny = suz * svx - sux * svz;
    nz = sux * svy - suy * svx;
    double len = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (len < 1e-14) { nx = 0; ny = 0; nz = 1; return; }
    nx /= len; ny /= len; nz /= len;
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
void NurbsSurface::add_inner_loop(const NurbsCurve& loop) { m_inner_loops.push_back(loop); }

void NurbsSurface::add_hole(const NurbsCurve& curve_3d) {
    auto dom = curve_3d.domain();
    auto sdom_u = domain(0);
    auto sdom_v = domain(1);
    double range_u = sdom_u.second - sdom_u.first;
    double range_v = sdom_v.second - sdom_v.first;

    // Sample the 3D curve and project each point to UV
    int n_samples = std::max(curve_3d.cv_count() * 4, 32);
    std::vector<Point> uv_pts;
    for (int i = 0; i < n_samples; ++i) {
        double t = dom.first + (dom.second - dom.first) * i / n_samples;
        Point pt3d = curve_3d.point_at(t);
        double u, v;
        closest_point(pt3d, u, v);
        // Normalize to [0,1] UV space
        double nu = (u - sdom_u.first) / range_u;
        double nv = (v - sdom_v.first) / range_v;
        uv_pts.push_back(Point(nu, nv, 0.0));
    }
    if (uv_pts.size() >= 3)
        add_inner_loop(NurbsCurve::create(true, 1, uv_pts));
}

void NurbsSurface::add_holes(const std::vector<NurbsCurve>& curves_3d) {
    for (const auto& crv : curves_3d) add_hole(crv);
}
NurbsCurve NurbsSurface::get_inner_loop(int index) const { return m_inner_loops[index]; }
int NurbsSurface::inner_loop_count() const { return static_cast<int>(m_inner_loops.size()); }
void NurbsSurface::clear_inner_loops() { m_inner_loops.clear(); }

Mesh NurbsSurface::mesh_grid(double max_angle, double max_edge_length,
                             double min_edge_length, double max_chord_height) const {
    if (m_mesh.number_of_vertices() == 0 && is_valid()) {
        TrimeshGrid mesher(*this);
        mesher.set_max_angle(max_angle)
              .set_max_edge_length(max_edge_length)
              .set_min_edge_length(min_edge_length)
              .set_max_chord_height(max_chord_height);
        m_mesh = mesher.mesh();
    }
    return m_mesh;
}

Mesh NurbsSurface::mesh_delaunay(double max_angle, double max_edge_length,
                                  double min_edge_length, double max_chord_height) const {
    if (m_mesh.number_of_vertices() == 0 && is_valid()) {
        TrimeshDelaunay mesher(*this);
        mesher.set_max_angle(max_angle)
              .set_max_edge_length(max_edge_length)
              .set_min_edge_length(min_edge_length)
              .set_max_chord_height(max_chord_height);
        m_mesh = mesher.mesh();
    }
    return m_mesh;
}

Mesh NurbsSurface::mesh(double max_angle, double max_edge_length,
                        double min_edge_length, double max_chord_height) const {
    if (is_trimmed())
        return mesh_delaunay(max_angle, max_edge_length, min_edge_length, max_chord_height);
    return mesh_grid(max_angle, max_edge_length, min_edge_length, max_chord_height);
}

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
    if (!m_inner_loops.empty()) {
        nlohmann::ordered_json arr = nlohmann::ordered_json::array();
        for (const auto& loop : m_inner_loops) {
            arr.push_back(loop.jsondump());
        }
        j["inner_loops"] = arr;
    }
    if (m_mesh.number_of_vertices() > 0) {
        j["mesh"] = m_mesh.jsondump();
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
    m_inner_loops = src.m_inner_loops;
    m_mesh = src.m_mesh;
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
    auto dom_u = domain(0);
    auto dom_v = domain(1);

    // Coarse grid search for initial guess
    int nu = 16, nv = 16;
    double best_dist2 = 1e300;
    double best_u = (dom_u.first + dom_u.second) * 0.5;
    double best_v = (dom_v.first + dom_v.second) * 0.5;
    for (int i = 0; i <= nu; ++i) {
        double u = dom_u.first + (dom_u.second - dom_u.first) * i / nu;
        for (int j = 0; j <= nv; ++j) {
            double v = dom_v.first + (dom_v.second - dom_v.first) * j / nv;
            double px, py, pz;
            point_at(u, v, px, py, pz);
            double dx = px - point[0], dy = py - point[1], dz = pz - point[2];
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < best_dist2) { best_dist2 = d2; best_u = u; best_v = v; }
        }
    }

    // Newton iteration to refine
    double u = best_u, v = best_v;
    for (int iter = 0; iter < 20; ++iter) {
        auto derivs = evaluate(u, v, 1);
        double dx = derivs[0][0] - point[0];
        double dy = derivs[0][1] - point[1];
        double dz = derivs[0][2] - point[2];
        double su0 = derivs[1][0], su1 = derivs[1][1], su2 = derivs[1][2];
        double sv0 = derivs[2][0], sv1 = derivs[2][1], sv2 = derivs[2][2];
        double fu = dx*su0 + dy*su1 + dz*su2;
        double fv = dx*sv0 + dy*sv1 + dz*sv2;
        if (std::abs(fu) < 1e-14 && std::abs(fv) < 1e-14) break;
        double juu = su0*su0 + su1*su1 + su2*su2;
        double juv = su0*sv0 + su1*sv1 + su2*sv2;
        double jvv = sv0*sv0 + sv1*sv1 + sv2*sv2;
        double det = juu*jvv - juv*juv;
        if (std::abs(det) < 1e-30) break;
        double du = -(jvv*fu - juv*fv) / det;
        double dv = -(juu*fv - juv*fu) / det;
        u += du; v += dv;
        u = std::max(dom_u.first, std::min(dom_u.second, u));
        v = std::max(dom_v.first, std::min(dom_v.second, v));
        if (du*du + dv*dv < 1e-28) break;
    }

    u_out = u;
    v_out = v;
    return point_at(u, v);
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
        if (data.contains("inner_loops")) {
            for (const auto& loop_data : data["inner_loops"]) {
                surface.m_inner_loops.push_back(NurbsCurve::jsonload(loop_data));
            }
        }
        if (data.contains("mesh") && !data["mesh"].is_null()) {
            surface.m_mesh = Mesh::jsonload(data["mesh"]);
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

    // Outer loop
    if (is_trimmed()) {
        std::string loop_data = m_outer_loop.pb_dumps();
        auto* ol = proto.mutable_outer_loop();
        ol->ParseFromString(loop_data);
    }

    // Inner loops
    for (const auto& inner : m_inner_loops) {
        std::string loop_data = inner.pb_dumps();
        auto* il = proto.add_inner_loops();
        il->ParseFromString(loop_data);
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

    // Load outer loop
    if (proto.has_outer_loop()) {
        std::string loop_data = proto.outer_loop().SerializeAsString();
        surface.m_outer_loop = NurbsCurve::pb_loads(loop_data);
    }

    // Load inner loops
    for (int i = 0; i < proto.inner_loops_size(); ++i) {
        std::string loop_data = proto.inner_loops(i).SerializeAsString();
        surface.m_inner_loops.push_back(NurbsCurve::pb_loads(loop_data));
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

NurbsSurface NurbsSurface::create_planar(const NurbsCurve& boundary) {
    NurbsSurface surface;

    // Collect control points from boundary
    std::vector<Point> all_pts;
    for (int i = 0; i < boundary.cv_count(); i++)
        all_pts.push_back(boundary.get_cv(i));
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
    auto project_to_uv = [&](const Point& pt) -> Point {
        double dx = pt[0] - orig[0], dy = pt[1] - orig[1], dz = pt[2] - orig[2];
        double nu = (dx * xax[0] + dy * xax[1] + dz * xax[2] - min_u) / range_u;
        double nv = (dx * yax[0] + dy * yax[1] + dz * yax[2] - min_v) / range_v;
        return Point(nu, nv, 0.0);
    };

    // Project boundary curve to UV
    auto project_curve_to_uv = [&](const NurbsCurve& crv) -> std::vector<Point> {
        std::vector<Point> pts;
        if (crv.degree() <= 1) {
            for (int i = 0; i < crv.cv_count(); ++i)
                pts.push_back(project_to_uv(crv.get_cv(i)));
        } else {
            auto spans = crv.get_span_vector();
            for (size_t si = 0; si + 1 < spans.size(); ++si) {
                int n_sub = 10;
                for (int k = 0; k <= n_sub; ++k) {
                    double t = spans[si] + (spans[si + 1] - spans[si]) * k / n_sub;
                    Point uv = project_to_uv(crv.point_at(t));
                    if (pts.empty() || (uv[0] - pts.back()[0]) * (uv[0] - pts.back()[0]) +
                        (uv[1] - pts.back()[1]) * (uv[1] - pts.back()[1]) > 1e-24)
                        pts.push_back(uv);
                }
            }
        }
        return pts;
    };

    std::vector<Point> uv_pts = project_curve_to_uv(boundary);

    if (uv_pts.size() >= 3) {
        NurbsCurve loop = NurbsCurve::create(false, 1, uv_pts);
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

    // OpenNURBS convention: first (order-1) knots equal, last (order-1) knots equal
    // Check start: knot[0] == knot[1] == ... == knot[degree-1]
    for (int i = 1; i < degree; i++) {
        if (std::abs(knot[i] - knot[0]) > 1e-10) return false;
    }

    // Check end: knot[last] == knot[last-1] == ... == knot[last-degree+1]
    int last = static_cast<int>(knot.size()) - 1;
    for (int i = last - 1; i > last - degree; i--) {
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
    // Always solve the interpolation system N * Q = P to find surface control points
    {
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
                // Clamped boundary: set endpoint basis function
                if (t <= knots_v[knot_base]) {
                    N_matrix[k][span] = 1.0;
                } else {
                    N_matrix[k][span + order_v - 1] = 1.0;
                }
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
                    double denom = a0 + a1;
                    double y = (denom != 0.0) ? Nvals[N0_idx + r] / denom : 0.0;
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

    // Convert closed-but-clamped profile to periodic for smooth seam
    NurbsCurve working_profile = profile;
    if (profile.is_closed() && !profile.is_periodic()) {
        int nc_orig = profile.cv_count();
        int deg = profile.degree();
        int ord = profile.order();
        bool is_rat = profile.is_rational();
        Point first_cv = profile.get_cv(0);
        Point last_cv = profile.get_cv(nc_orig - 1);
        if (first_cv.distance(last_cv) < 1e-10) {
            int unique_cv = nc_orig - 1;
            int periodic_cv = unique_cv + deg;
            NurbsCurve pcrv(3, is_rat, ord, periodic_cv);
            for (int i = 0; i < unique_cv; i++) pcrv.set_cv(i, profile.get_cv(i));
            for (int i = 0; i < deg; i++) pcrv.set_cv(unique_cv + i, profile.get_cv(i));
            if (is_rat) {
                for (int i = 0; i < unique_cv; i++) pcrv.set_weight(i, profile.weight(i));
                for (int i = 0; i < deg; i++) pcrv.set_weight(unique_cv + i, profile.weight(i));
            }
            int kc = ord + periodic_cv - 2;
            for (int i = 0; i < kc; i++) pcrv.set_knot(i, (i - (ord - 2)) * 1.0);
            pcrv.set_domain(0.0, pcrv.length());
            working_profile = pcrv;
        }
    }

    int N = std::min(std::max(rail.span_count() * 2 + 1, 5), 20);
    std::vector<Plane> frames = rail.get_perpendicular_planes(N);
    if (frames.empty()) return surface;

    // Center at bounding box center of profile CVs
    double cx = 0, cy = 0, cz = 0;
    int nc = working_profile.cv_count();
    for (int k = 0; k < nc; k++) {
        Point cv = working_profile.get_cv(k);
        cx += cv[0]; cy += cv[1]; cz += cv[2];
    }
    cx /= nc; cy /= nc; cz /= nc;

    // Compute profile plane from 3 evaluated points
    auto dom = working_profile.domain();
    double t0 = dom.first, t1 = dom.second;
    Point pa = working_profile.point_at(t0);
    Point pb = working_profile.point_at(t0 + (t1 - t0) / 3.0);
    Point pc = working_profile.point_at(t0 + 2.0 * (t1 - t0) / 3.0);
    Vector v1(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
    Vector v2(pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]);
    Vector prof_normal = v1.cross(v2);
    double nlen = prof_normal.magnitude();
    if (nlen < 1e-14) prof_normal = Vector(1, 0, 0);
    else prof_normal = prof_normal / nlen;

    // Profile local frame: prof_x from center to first eval point, orthogonalized
    Vector prof_x(pa[0] - cx, pa[1] - cy, pa[2] - cz);
    double pxlen = prof_x.magnitude();
    if (pxlen < 1e-14) prof_x = Vector(0, 1, 0);
    else prof_x = prof_x / pxlen;
    // Remove component along normal
    double dot = prof_x[0] * prof_normal[0] + prof_x[1] * prof_normal[1] + prof_x[2] * prof_normal[2];
    prof_x = Vector(prof_x[0] - dot * prof_normal[0], prof_x[1] - dot * prof_normal[1], prof_x[2] - dot * prof_normal[2]);
    pxlen = prof_x.magnitude();
    if (pxlen < 1e-14) prof_x = Vector(0, 1, 0);
    else prof_x = prof_x / pxlen;
    Vector prof_y = prof_normal.cross(prof_x);
    double pylen = prof_y.magnitude();
    if (pylen > 1e-14) prof_y = prof_y / pylen;

    std::vector<NurbsCurve> positioned_profiles;
    positioned_profiles.reserve(frames.size());

    for (size_t i = 0; i < frames.size(); i++) {
        NurbsCurve prof_copy = working_profile;
        Plane& frame = frames[i];
        Point fo = frame.origin();
        Vector fx = frame.x_axis();
        Vector fy = frame.y_axis();
        Vector fz = frame.z_axis();

        Xform t1x = Xform::translation(-cx, -cy, -cz);

        // R = T * S^T: maps prof_normal→fz(tangent), prof_x→fx, prof_y→fy
        // S = [prof_x | prof_y | prof_normal] (source frame columns)
        // T = [fx | fy | fz] (target frame columns)
        // R = T * S^T where T=[fx|fy|fz], S=[prof_x|prof_y|prof_normal]
        Xform rot = Xform::identity();
        rot.m[0]  = fx[0]*prof_x[0] + fy[0]*prof_y[0] + fz[0]*prof_normal[0];
        rot.m[1]  = fx[1]*prof_x[0] + fy[1]*prof_y[0] + fz[1]*prof_normal[0];
        rot.m[2]  = fx[2]*prof_x[0] + fy[2]*prof_y[0] + fz[2]*prof_normal[0];
        rot.m[4]  = fx[0]*prof_x[1] + fy[0]*prof_y[1] + fz[0]*prof_normal[1];
        rot.m[5]  = fx[1]*prof_x[1] + fy[1]*prof_y[1] + fz[1]*prof_normal[1];
        rot.m[6]  = fx[2]*prof_x[1] + fy[2]*prof_y[1] + fz[2]*prof_normal[1];
        rot.m[8]  = fx[0]*prof_x[2] + fy[0]*prof_y[2] + fz[0]*prof_normal[2];
        rot.m[9]  = fx[1]*prof_x[2] + fy[1]*prof_y[2] + fz[1]*prof_normal[2];
        rot.m[10] = fx[2]*prof_x[2] + fy[2]*prof_y[2] + fz[2]*prof_normal[2];
        rot.m[12] = fo[0]; rot.m[13] = fo[1]; rot.m[14] = fo[2];

        prof_copy.transform(t1x);
        prof_copy.transform(rot);
        positioned_profiles.push_back(prof_copy);
    }

    int loft_degree = std::min(3, static_cast<int>(positioned_profiles.size()) - 1);
    surface = create_loft(positioned_profiles, loft_degree);
    return surface;
}

NurbsSurface NurbsSurface::create_sweep2(const NurbsCurve& rail1, const NurbsCurve& rail2,
                                           const std::vector<NurbsCurve>& shapes) {
    NurbsSurface surface;
    if (!rail1.is_valid() || !rail2.is_valid() || shapes.empty()) return surface;
    for (auto& s : shapes) { if (!s.is_valid()) return surface; }

    // Make shapes compatible (same degree, knots, cv_count)
    std::vector<NurbsCurve> compat_shapes = shapes;
    if (compat_shapes.size() >= 2) make_curves_compatible(compat_shapes);

    int n_shapes = static_cast<int>(compat_shapes.size());

    // Shape parameters: evenly distributed [0..1]
    std::vector<double> shape_params(n_shapes);
    for (int k = 0; k < n_shapes; k++)
        shape_params[k] = (n_shapes == 1) ? 0.0 : (double)k / (n_shapes - 1);

    int N = std::min(std::max(std::max(rail1.span_count(), rail2.span_count()) * 2 + 1, 5), 20);

    std::vector<Point> pts1, pts2;
    std::vector<double> params1, params2;
    rail1.divide_by_count(N + 1, pts1, &params1, true);
    rail2.divide_by_count(N + 1, pts2, &params2, true);

    std::vector<Plane> frames1 = rail1.get_perpendicular_planes(N);
    if (frames1.empty()) return surface;

    // Precompute per-shape: start, end, span, width, direction, local frame
    struct ShapeInfo {
        Point start, end;
        double width;
        Vector dir, side, up;
    };
    std::vector<ShapeInfo> sinfo(n_shapes);
    for (int k = 0; k < n_shapes; k++) {
        auto& si = sinfo[k];
        si.start = compat_shapes[k].point_at_start();
        si.end = compat_shapes[k].point_at_end();
        Vector span(si.end[0]-si.start[0], si.end[1]-si.start[1], si.end[2]-si.start[2]);
        si.width = span.magnitude();
        if (si.width < 1e-14) si.width = 1.0;
        si.dir = span / si.width;
        Vector up_try(0, 0, 1);
        si.side = si.dir.cross(up_try);
        if (si.side.magnitude() < 1e-10) {
            up_try = Vector(0, 1, 0);
            si.side = si.dir.cross(up_try);
        }
        si.side = si.side / si.side.magnitude();
        si.up = si.side.cross(si.dir);
        double ulen = si.up.magnitude();
        if (ulen > 1e-14) si.up = si.up / ulen;
    }

    std::vector<NurbsCurve> positioned_profiles;
    positioned_profiles.reserve(frames1.size());

    for (size_t i = 0; i < frames1.size() && i < pts1.size() && i < pts2.size(); i++) {
        // Section parameter t in [0,1]
        double t = (frames1.size() <= 1) ? 0.0 : (double)i / (frames1.size() - 1);

        // Find bracketing shapes and interpolation factor
        int j = 0;
        double s = 0.0;
        if (n_shapes == 1) {
            j = 0; s = 0.0;
        } else {
            for (int k = 0; k < n_shapes - 1; k++) {
                if (t <= shape_params[k + 1] + 1e-14) { j = k; break; }
                j = k;
            }
            double denom = shape_params[j + 1] - shape_params[j];
            s = (denom > 1e-14) ? (t - shape_params[j]) / denom : 0.0;
            s = std::max(0.0, std::min(1.0, s));
        }

        // Interpolate CVs between shapes[j] and shapes[j+1] (or just use shapes[j] if single)
        NurbsCurve interp_shape = compat_shapes[j];
        if (n_shapes > 1 && j + 1 < n_shapes) {
            int nc = compat_shapes[j].cv_count();
            for (int c = 0; c < nc; c++) {
                Point cv0 = compat_shapes[j].get_cv(c);
                Point cv1 = compat_shapes[j + 1].get_cv(c);
                Point lerped(cv0[0]*(1-s) + cv1[0]*s, cv0[1]*(1-s) + cv1[1]*s, cv0[2]*(1-s) + cv1[2]*s);
                interp_shape.set_cv(c, lerped);
            }
        }

        // Interpolate shape info
        double shape_width = sinfo[j].width * (1-s) + ((n_shapes > 1 && j+1 < n_shapes) ? sinfo[j+1].width * s : 0.0);
        if (n_shapes == 1) shape_width = sinfo[0].width;
        Vector prof_dir(
            sinfo[j].dir[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].dir[0]*s : 0.0),
            sinfo[j].dir[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].dir[1]*s : 0.0),
            sinfo[j].dir[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].dir[2]*s : 0.0));
        double pdlen = prof_dir.magnitude();
        if (pdlen > 1e-14) prof_dir = prof_dir / pdlen;
        Vector prof_side(
            sinfo[j].side[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].side[0]*s : 0.0),
            sinfo[j].side[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].side[1]*s : 0.0),
            sinfo[j].side[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].side[2]*s : 0.0));
        double pslen = prof_side.magnitude();
        if (pslen > 1e-14) prof_side = prof_side / pslen;
        Vector prof_up(
            sinfo[j].up[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].up[0]*s : 0.0),
            sinfo[j].up[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].up[1]*s : 0.0),
            sinfo[j].up[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].up[2]*s : 0.0));
        double pulen = prof_up.magnitude();
        if (pulen > 1e-14) prof_up = prof_up / pulen;

        Point interp_start(
            sinfo[j].start[0]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].start[0]*s : 0.0),
            sinfo[j].start[1]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].start[1]*s : 0.0),
            sinfo[j].start[2]*(1-s) + ((n_shapes>1 && j+1<n_shapes) ? sinfo[j+1].start[2]*s : 0.0));
        if (n_shapes == 1) interp_start = sinfo[0].start;

        Point p1 = pts1[i];
        Point p2 = pts2[i];
        double dx = p2[0] - p1[0], dy = p2[1] - p1[1], dz = p2[2] - p1[2];
        double rail_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        double scale_factor = (rail_dist > 1e-14 && shape_width > 1e-14) ? rail_dist / shape_width : 1.0;

        NurbsCurve prof_copy = interp_shape;
        Xform t1 = Xform::translation(-interp_start[0], -interp_start[1], -interp_start[2]);
        prof_copy.transform(t1);

        Xform sc = Xform::scale_xyz(scale_factor, scale_factor, scale_factor);
        prof_copy.transform(sc);

        Plane& frame = frames1[i];
        Vector tangent = frame.z_axis();
        Vector x_dir(dx, dy, dz);
        double x_len = x_dir.magnitude();
        if (x_len > 1e-14) x_dir = x_dir / x_len;
        else x_dir = frame.x_axis();
        Vector y_dir = tangent.cross(x_dir);
        double y_len = y_dir.magnitude();
        if (y_len > 1e-14) y_dir = y_dir / y_len;
        else y_dir = frame.y_axis();
        // Flip y_dir to match shape's up direction
        double dot_up = y_dir[0]*prof_up[0] + y_dir[1]*prof_up[1] + y_dir[2]*prof_up[2];
        if (dot_up < 0) y_dir = Vector(-y_dir[0], -y_dir[1], -y_dir[2]);
        tangent = x_dir.cross(y_dir);
        double tz = tangent.magnitude();
        if (tz > 1e-14) tangent = tangent / tz;

        // R = T * S^T where T=[tangent|x_dir|y_dir], S=[prof_side|prof_dir|prof_up]
        Xform rot = Xform::identity();
        rot.m[0]  = tangent[0]*prof_side[0] + x_dir[0]*prof_dir[0] + y_dir[0]*prof_up[0];
        rot.m[1]  = tangent[1]*prof_side[0] + x_dir[1]*prof_dir[0] + y_dir[1]*prof_up[0];
        rot.m[2]  = tangent[2]*prof_side[0] + x_dir[2]*prof_dir[0] + y_dir[2]*prof_up[0];
        rot.m[4]  = tangent[0]*prof_side[1] + x_dir[0]*prof_dir[1] + y_dir[0]*prof_up[1];
        rot.m[5]  = tangent[1]*prof_side[1] + x_dir[1]*prof_dir[1] + y_dir[1]*prof_up[1];
        rot.m[6]  = tangent[2]*prof_side[1] + x_dir[2]*prof_dir[1] + y_dir[2]*prof_up[1];
        rot.m[8]  = tangent[0]*prof_side[2] + x_dir[0]*prof_dir[2] + y_dir[0]*prof_up[2];
        rot.m[9]  = tangent[1]*prof_side[2] + x_dir[1]*prof_dir[2] + y_dir[1]*prof_up[2];
        rot.m[10] = tangent[2]*prof_side[2] + x_dir[2]*prof_dir[2] + y_dir[2]*prof_up[2];
        rot.m[12] = p1[0]; rot.m[13] = p1[1]; rot.m[14] = p1[2];

        prof_copy.transform(rot);
        positioned_profiles.push_back(prof_copy);
    }

    int loft_degree = std::min(3, static_cast<int>(positioned_profiles.size()) - 1);
    surface = create_loft(positioned_profiles, loft_degree);
    return surface;
}

NurbsSurface NurbsSurface::create_edge_surface(
    const NurbsCurve& c0, const NurbsCurve& c1,
    const NurbsCurve& c2, const NurbsCurve& c3)
{
    NurbsSurface surface;

    // Validate all 4 curves
    if (!c0.is_valid() || !c1.is_valid() || !c2.is_valid() || !c3.is_valid())
        return surface;

    // Build a boundary loop by matching endpoints
    // Input: 4 curves in arbitrary order, we chain them into a consistent loop
    std::vector<NurbsCurve> input = {c0, c1, c2, c3};
    std::vector<NurbsCurve> loop;
    std::vector<bool> used(4, false);

    // Start with curve 0
    loop.push_back(input[0]);
    used[0] = true;
    const double tol = 1e-6;

    for (int step = 0; step < 3; step++) {
        Point tail = loop.back().point_at_end();
        bool found = false;
        for (int i = 0; i < 4; i++) {
            if (used[i]) continue;
            Point s = input[i].point_at_start();
            Point e = input[i].point_at_end();
            if (s.distance(tail) < tol) {
                loop.push_back(input[i]);
                used[i] = true;
                found = true;
                break;
            }
            if (e.distance(tail) < tol) {
                NurbsCurve rev = input[i];
                rev.reverse();
                loop.push_back(rev);
                used[i] = true;
                found = true;
                break;
            }
        }
        if (!found) return surface;
    }

    // Verify loop closure
    if (loop[3].point_at_end().distance(loop[0].point_at_start()) > tol)
        return surface;

    // loop[0]=south, loop[1]=east, loop[2]=north(reversed), loop[3]=west(reversed)
    // Orient for surface: south & north same v-direction, west & east same u-direction
    NurbsCurve south = loop[0];                  // u=0, v varies
    NurbsCurve east  = loop[1];                  // v=1, u varies (u=0 to u=1)
    NurbsCurve north = loop[2]; north.reverse(); // u=1, v varies (same dir as south)
    NurbsCurve west  = loop[3]; west.reverse();  // v=0, u varies (same dir as east)

    // Make opposite pairs compatible
    std::vector<NurbsCurve> v_pair = {south, north};
    make_curves_compatible(v_pair);
    south = v_pair[0];
    north = v_pair[1];

    std::vector<NurbsCurve> u_pair = {west, east};
    make_curves_compatible(u_pair);
    west = u_pair[0];
    east = u_pair[1];

    int order_v = south.order();
    int cv_count_v = south.cv_count();
    int order_u = west.order();
    int cv_count_u = west.cv_count();
    bool is_rat = south.is_rational() || west.is_rational();

    // Create surface structure
    surface.create_raw(3, is_rat, order_u, order_v, cv_count_u, cv_count_v);

    // Set u-knots from west/east
    for (int i = 0; i < surface.knot_count(0); i++)
        surface.set_knot(0, i, west.knot(i));

    // Set v-knots from south/north
    for (int i = 0; i < surface.knot_count(1); i++)
        surface.set_knot(1, i, south.knot(i));

    // Compute Greville abscissae for blending
    std::vector<double> u_grev = west.get_greville_abcissae();
    std::vector<double> v_grev = south.get_greville_abcissae();

    // Normalize to [0,1]
    auto [u0, u1] = west.domain();
    auto [v0, v1] = south.domain();
    for (auto& g : u_grev) g = (u1 > u0) ? (g - u0) / (u1 - u0) : 0.0;
    for (auto& g : v_grev) g = (v1 > v0) ? (g - v0) / (v1 - v0) : 0.0;

    // Corner points
    Point C00 = south.get_cv(0);
    Point C01 = south.get_cv(cv_count_v - 1);
    Point C10 = north.get_cv(0);
    Point C11 = north.get_cv(cv_count_v - 1);

    // Coons patch: P(i,j) = (1-ui)*S[j] + ui*N[j] + (1-vj)*W[i] + vj*E[i]
    //                       - (1-ui)*(1-vj)*C00 - (1-ui)*vj*C01
    //                       - ui*(1-vj)*C10 - ui*vj*C11
    for (int i = 0; i < cv_count_u; i++) {
        double ui = u_grev[i];
        Point wi = west.get_cv(i);
        Point ei = east.get_cv(i);
        for (int j = 0; j < cv_count_v; j++) {
            double vj = v_grev[j];
            Point sj = south.get_cv(j);
            Point nj = north.get_cv(j);

            double x = (1-ui)*sj[0] + ui*nj[0] + (1-vj)*wi[0] + vj*ei[0]
                      - (1-ui)*(1-vj)*C00[0] - (1-ui)*vj*C01[0]
                      - ui*(1-vj)*C10[0] - ui*vj*C11[0];
            double y = (1-ui)*sj[1] + ui*nj[1] + (1-vj)*wi[1] + vj*ei[1]
                      - (1-ui)*(1-vj)*C00[1] - (1-ui)*vj*C01[1]
                      - ui*(1-vj)*C10[1] - ui*vj*C11[1];
            double z = (1-ui)*sj[2] + ui*nj[2] + (1-vj)*wi[2] + vj*ei[2]
                      - (1-ui)*(1-vj)*C00[2] - (1-ui)*vj*C01[2]
                      - ui*(1-vj)*C10[2] - ui*vj*C11[2];

            surface.set_cv(i, j, Point(x, y, z));
        }
    }

    return surface;
}

NurbsSurface NurbsSurface::create_network(
    const std::vector<NurbsCurve>& u_curves,
    const std::vector<NurbsCurve>& v_curves)
{
    NurbsSurface surface;
    int n_u = static_cast<int>(u_curves.size());
    int n_v = static_cast<int>(v_curves.size());
    if (n_u < 2 || n_v < 2) return surface;

    auto u_crvs = u_curves;
    auto v_crvs = v_curves;

    // Helper: min squared distance from point to sampled curve
    auto min_dist_sq = [](const NurbsCurve& crv, const Point& pt) -> double {
        auto [t0, t1] = crv.domain();
        double best = 1e30;
        for (int i = 0; i <= 50; i++) {
            double t = t0 + (t1 - t0) * i / 50.0;
            Point p = crv.point_at(t);
            double d = (p[0]-pt[0])*(p[0]-pt[0]) + (p[1]-pt[1])*(p[1]-pt[1]) + (p[2]-pt[2])*(p[2]-pt[2]);
            if (d < best) best = d;
        }
        return best;
    };

    // Helper: find closest parameter on curve to point (sampling + Newton)
    auto find_param = [](const NurbsCurve& crv, const Point& pt) -> double {
        auto [t0, t1] = crv.domain();
        double best_t = t0, best_d = 1e30;
        int ns = 200;
        for (int i = 0; i <= ns; i++) {
            double t = t0 + (t1 - t0) * i / ns;
            Point p = crv.point_at(t);
            double d = (p[0]-pt[0])*(p[0]-pt[0]) + (p[1]-pt[1])*(p[1]-pt[1]) + (p[2]-pt[2])*(p[2]-pt[2]);
            if (d < best_d) { best_d = d; best_t = t; }
        }
        for (int iter = 0; iter < 20; iter++) {
            auto derivs = crv.evaluate(best_t, 2);
            double dx = derivs[0][0]-pt[0], dy = derivs[0][1]-pt[1], dz = derivs[0][2]-pt[2];
            double f1 = 2.0*(dx*derivs[1][0] + dy*derivs[1][1] + dz*derivs[1][2]);
            double f2 = 2.0*(derivs[1][0]*derivs[1][0] + derivs[1][1]*derivs[1][1] + derivs[1][2]*derivs[1][2]
                            + dx*derivs[2][0] + dy*derivs[2][1] + dz*derivs[2][2]);
            if (std::abs(f2) < 1e-14) break;
            double dt = f1 / f2;
            best_t -= dt;
            if (best_t < t0) best_t = t0;
            if (best_t > t1) best_t = t1;
            if (std::abs(dt) < 1e-14) break;
        }
        return best_t;
    };

    // Orient v-curves: all should start near u_curve[0]
    for (auto& vc : v_crvs) {
        Point vs = vc.point_at(vc.domain_start());
        Point ve = vc.point_at(vc.domain_end());
        if (min_dist_sq(u_crvs[0], ve) < min_dist_sq(u_crvs[0], vs))
            vc.reverse();
    }

    // Find u-params for each v-curve on u_curve[0]
    std::vector<double> u_params_raw(n_v);
    for (int j = 0; j < n_v; j++)
        u_params_raw[j] = find_param(u_crvs[0], v_crvs[j].point_at(v_crvs[j].domain_start()));

    // Sort v-curves by u-parameter
    std::vector<int> v_order(n_v);
    std::iota(v_order.begin(), v_order.end(), 0);
    std::sort(v_order.begin(), v_order.end(), [&](int a, int b) {
        return u_params_raw[a] < u_params_raw[b];
    });
    {
        std::vector<NurbsCurve> tmp(n_v);
        for (int j = 0; j < n_v; j++) tmp[j] = v_crvs[v_order[j]];
        v_crvs = tmp;
    }

    // Orient u-curves: all should start near first sorted v-curve
    for (auto& uc : u_crvs) {
        Point us = uc.point_at(uc.domain_start());
        Point ue = uc.point_at(uc.domain_end());
        if (min_dist_sq(v_crvs[0], ue) < min_dist_sq(v_crvs[0], us))
            uc.reverse();
    }

    // Make curves compatible (unifies degree & knots)
    make_curves_compatible(u_crvs);
    make_curves_compatible(v_crvs);
    // Ensure [0,1] domain (make_curves_compatible may skip if already compatible)
    for (auto& c : u_crvs) c.set_domain(0.0, 1.0);
    for (auto& c : v_crvs) c.set_domain(0.0, 1.0);

    int cv_u = u_crvs[0].cv_count();
    int cv_v = v_crvs[0].cv_count();

    // Find intersection parameters after compatibility
    std::vector<double> u_params(n_v);
    for (int j = 0; j < n_v; j++)
        u_params[j] = find_param(u_crvs[0], v_crvs[j].point_at(v_crvs[j].domain_start()));

    std::vector<double> v_params(n_u);
    for (int i = 0; i < n_u; i++)
        v_params[i] = find_param(v_crvs[0], u_crvs[i].point_at(u_crvs[i].domain_start()));

    // Intersection points
    std::vector<std::vector<Point>> P_ij(n_u, std::vector<Point>(n_v));
    for (int i = 0; i < n_u; i++)
        for (int j = 0; j < n_v; j++)
            P_ij[i][j] = u_crvs[i].point_at(u_params[j]);

    // Lagrange basis function
    auto lagrange = [](const std::vector<double>& params, int k, double t) -> double {
        double r = 1.0;
        for (int j = 0; j < static_cast<int>(params.size()); j++)
            if (j != k) r *= (t - params[j]) / (params[k] - params[j]);
        return r;
    };

    // Determine sample counts
    int n_u_samples = n_v * (cv_u + 2) - 1;
    int n_v_samples = n_u * (cv_v + 2) - 1;

    // Gordon formula evaluator
    auto eval_gordon = [&](double u, double v) -> Point {
        std::vector<double> Lk(n_v);
        for (int k = 0; k < n_v; k++) Lk[k] = lagrange(u_params, k, u);
        std::vector<double> Ml(n_u);
        for (int l = 0; l < n_u; l++) Ml[l] = lagrange(v_params, l, v);
        double x = 0, y = 0, z = 0;
        for (int k = 0; k < n_v; k++) {
            Point p = v_crvs[k].point_at(v);
            x += p[0] * Lk[k]; y += p[1] * Lk[k]; z += p[2] * Lk[k];
        }
        for (int l = 0; l < n_u; l++) {
            Point p = u_crvs[l].point_at(u);
            x += p[0] * Ml[l]; y += p[1] * Ml[l]; z += p[2] * Ml[l];
        }
        for (int l = 0; l < n_u; l++)
            for (int k = 0; k < n_v; k++) {
                x -= P_ij[l][k][0] * Lk[k] * Ml[l];
                y -= P_ij[l][k][1] * Lk[k] * Ml[l];
                z -= P_ij[l][k][2] * Lk[k] * Ml[l];
            }
        return Point(x, y, z);
    };

    // Cosine (half-cosine) spacing within each interval
    auto interval_cosine = [](const std::vector<double>& anchors, int n_total) {
        int n_intervals = static_cast<int>(anchors.size()) - 1;
        int spi = (n_total - 1) / n_intervals + 1;
        std::vector<double> params;
        params.reserve(n_total);
        for (int j = 0; j < n_intervals; j++) {
            double a = anchors[j], b = anchors[j + 1];
            int start_k = (j == 0) ? 0 : 1;
            for (int k = start_k; k < spi; k++) {
                double t = (1.0 - std::cos(Tolerance::PI * k / (spi - 1))) / 2.0;
                params.push_back(a + (b - a) * t);
            }
        }
        return params;
    };

    // Power-graded: concentrate at surface boundaries, uniform in middle
    auto interval_graded = [](const std::vector<double>& anchors, int n_total, double alpha) {
        int n_intervals = static_cast<int>(anchors.size()) - 1;
        int spi = (n_total - 1) / n_intervals + 1;
        std::vector<double> params;
        params.reserve(n_total);
        for (int j = 0; j < n_intervals; j++) {
            double a = anchors[j], b = anchors[j + 1];
            int start_k = (j == 0) ? 0 : 1;
            for (int k = start_k; k < spi; k++) {
                double u = static_cast<double>(k) / (spi - 1);
                double t;
                if (j == 0) // first interval: concentrate at start
                    t = std::pow(u, alpha);
                else if (j == n_intervals - 1) // last interval: concentrate at end
                    t = 1.0 - std::pow(1.0 - u, alpha);
                else // middle intervals: uniform
                    t = u;
                params.push_back(a + (b - a) * t);
            }
        }
        return params;
    };
    auto u_eval = interval_graded(u_params, n_u_samples, 1.5);
    auto v_eval = interval_cosine(v_params, n_v_samples);

    // Evaluate Gordon at sample points
    std::vector<std::vector<Point>> grid(n_u_samples, std::vector<Point>(n_v_samples));
    for (int si = 0; si < n_u_samples; si++)
        for (int sj = 0; sj < n_v_samples; sj++)
            grid[si][sj] = eval_gordon(u_eval[si], v_eval[sj]);

    // Reverse u-direction so surface goes from first v-curve (min u) to last
    std::reverse(grid.begin(), grid.end());

    // Use evaluation parameters for fitting (after reversal adjustment)
    std::vector<double> u_sample(n_u_samples);
    for (int i = 0; i < n_u_samples; i++)
        u_sample[i] = 1.0 - u_eval[n_u_samples - 1 - i];
    auto v_sample = v_eval;

    // Global surface interpolation through sampled grid
    int degree = 3;
    int order = degree + 1;

    // Build knot vectors (Piegl-Tiller averaging)
    auto build_knots = [&](int n_pts, const std::vector<double>& params) {
        int kc = order + n_pts - 2;
        std::vector<double> knots(kc);
        for (int i = 0; i < order - 1; i++) knots[i] = 0.0;
        for (int j = 1; j <= n_pts - order; j++) {
            double sum = 0;
            for (int i = j; i < j + degree; i++) sum += params[i];
            knots[order - 2 + j] = sum / degree;
        }
        for (int i = kc - order + 1; i < kc; i++) knots[i] = 1.0;
        return knots;
    };

    auto u_knots = build_knots(n_u_samples, u_sample);
    auto v_knots = build_knots(n_v_samples, v_sample);

    // Build basis function matrix for interpolation
    auto build_basis_matrix = [&](int n, const std::vector<double>& params,
                                  const std::vector<double>& knots) {
        std::vector<std::vector<double>> N(n, std::vector<double>(n, 0.0));
        for (int row = 0; row < n; row++) {
            double t = params[row];
            int span = knot::find_span(order, n, knots, t);
            int d = order - 1;
            int kb = span + d;
            if (knots[kb - 1] == knots[kb]) {
                if (t <= knots[kb]) N[row][span] = 1.0;
                else N[row][span + order - 1] = 1.0;
                continue;
            }
            std::vector<double> Nv(order * order, 0.0);
            Nv[order * order - 1] = 1.0;
            std::vector<double> left(d), right(d);
            int ni = order * order - 1;
            int kr = kb, kl = kb - 1;
            for (int j = 0; j < d; j++) {
                int n0 = ni; ni -= (order + 1);
                left[j] = t - knots[kl]; right[j] = knots[kr] - t;
                kl--; kr++;
                double xv = 0.0;
                for (int r = 0; r <= j; r++) {
                    double a0 = left[j - r], a1 = right[r];
                    double den = a0 + a1;
                    double yv = (den != 0.0) ? Nv[n0 + r] / den : 0.0;
                    Nv[ni + r] = xv + a1 * yv;
                    xv = a0 * yv;
                }
                Nv[ni + j + 1] = xv;
            }
            for (int j = 0; j < order; j++) {
                int col = span + j;
                if (col >= 0 && col < n) N[row][col] = Nv[j];
            }
        }
        return N;
    };

    auto u_N = build_basis_matrix(n_u_samples, u_sample, u_knots);
    auto v_N = build_basis_matrix(n_v_samples, v_sample, v_knots);

    // Gaussian elimination solver
    auto solve = [](int n, int dim, std::vector<std::vector<double>>& A,
                    std::vector<std::vector<double>>& b) {
        for (int col = 0; col < n; col++) {
            int mr = col; double mv = std::abs(A[col][col]);
            for (int r = col + 1; r < n; r++)
                if (std::abs(A[r][col]) > mv) { mv = std::abs(A[r][col]); mr = r; }
            if (mv < 1e-14) continue;
            std::swap(A[col], A[mr]); std::swap(b[col], b[mr]);
            for (int r = col + 1; r < n; r++) {
                double f = A[r][col] / A[col][col];
                for (int c = col; c < n; c++) A[r][c] -= f * A[col][c];
                for (int d = 0; d < dim; d++) b[r][d] -= f * b[col][d];
            }
        }
        for (int r = n - 1; r >= 0; r--)
            for (int d = 0; d < dim; d++) {
                for (int c = r + 1; c < n; c++) b[r][d] -= A[r][c] * b[c][d];
                if (std::abs(A[r][r]) > 1e-14) b[r][d] /= A[r][r];
            }
    };

    // Interpolate in u first → intermediate R[i][j]
    std::vector<std::vector<Point>> R(n_u_samples, std::vector<Point>(n_v_samples));
    for (int sj = 0; sj < n_v_samples; sj++) {
        auto A = u_N;
        std::vector<std::vector<double>> rhs(n_u_samples, std::vector<double>(3));
        for (int si = 0; si < n_u_samples; si++)
            rhs[si] = {grid[si][sj][0], grid[si][sj][1], grid[si][sj][2]};
        solve(n_u_samples, 3, A, rhs);
        for (int si = 0; si < n_u_samples; si++)
            R[si][sj] = Point(rhs[si][0], rhs[si][1], rhs[si][2]);
    }

    // Interpolate in v → final surface CVs
    int u_kc = static_cast<int>(u_knots.size());
    int v_kc = static_cast<int>(v_knots.size());
    surface.create_raw(3, false, order, order, n_u_samples, n_v_samples);
    for (int i = 0; i < u_kc; i++) surface.set_knot(0, i, u_knots[i]);
    for (int i = 0; i < v_kc; i++) surface.set_knot(1, i, v_knots[i]);

    for (int si = 0; si < n_u_samples; si++) {
        auto A = v_N;
        std::vector<std::vector<double>> rhs(n_v_samples, std::vector<double>(3));
        for (int sj = 0; sj < n_v_samples; sj++)
            rhs[sj] = {R[si][sj][0], R[si][sj][1], R[si][sj][2]};
        solve(n_v_samples, 3, A, rhs);
        for (int sj = 0; sj < n_v_samples; sj++)
            surface.set_cv(si, sj, Point(rhs[sj][0], rhs[sj][1], rhs[sj][2]));
    }

    return surface;
}

} // namespace session_cpp
