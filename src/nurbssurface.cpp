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
    if (pointcolors != other.pointcolors) return false;
    if (facecolors != other.facecolors) return false;
    if (linecolors != other.linecolors) return false;
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
    pointcolors.clear();
    facecolors.clear();
    linecolors.clear();
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
        double w = S0[m_dim], wu = Su[m_dim], wv = Sv[m_dim];
        px = S0[0]/w; py = S0[1]/w; pz = S0[2]/w;
        sux = (Su[0] - wu * px) / w;
        suy = (Su[1] - wu * py) / w;
        suz = (Su[2] - wu * pz) / w;
        svx = (Sv[0] - wv * px) / w;
        svy = (Sv[1] - wv * py) / w;
        svz = (Sv[2] - wv * pz) / w;
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
        double w = S0[m_dim], wu = Su[m_dim], wv = Sv[m_dim];
        if (std::abs(w) < 1e-14) { nx = 0; ny = 0; nz = 1; return; }
        double px = S0[0]/w, py = S0[1]/w, pz = S0[2]/w;
        sux = (Su[0] - wu * px) / w;
        suy = (Su[1] - wu * py) / w;
        suz = (Su[2] - wu * pz) / w;
        svx = (Sv[0] - wv * px) / w;
        svy = (Sv[1] - wv * py) / w;
        svz = (Sv[2] - wv * pz) / w;
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
    if (m_mesh.number_of_vertices() == 0 && is_valid() && is_planar(nullptr, 1e-6)) {
        Mesh result;
        Point p00 = point_at_corner(0, 0);
        Point p10 = point_at_corner(1, 0);
        Point p11 = point_at_corner(1, 1);
        Point p01 = point_at_corner(0, 1);
        double d2 = (p00[0]-p01[0])*(p00[0]-p01[0]) + (p00[1]-p01[1])*(p00[1]-p01[1]) + (p00[2]-p01[2])*(p00[2]-p01[2]);
        Vector normal;
        if (d2 < 1e-20) {
            auto v0 = result.add_vertex(p00);
            auto v1 = result.add_vertex(p10);
            auto v2 = result.add_vertex(p11);
            result.add_face({v0, v1, v2});
            Vector e1(p10[0]-p00[0], p10[1]-p00[1], p10[2]-p00[2]);
            Vector e2(p11[0]-p00[0], p11[1]-p00[1], p11[2]-p00[2]);
            normal = e1.cross(e2);
        } else {
            auto v0 = result.add_vertex(p00);
            auto v1 = result.add_vertex(p10);
            auto v2 = result.add_vertex(p11);
            auto v3 = result.add_vertex(p01);
            result.add_face({v0, v1, v2});
            result.add_face({v0, v2, v3});
            auto derivs = evaluate(0.5, 0.5, 1);
            if (derivs.size() >= 3) normal = derivs[1].cross(derivs[2]);
        }
        double nlen = normal.magnitude();
        if (nlen > 1e-15) normal = normal * (1.0 / nlen);
        for (auto& [vi, pt] : result.vertex)
            pt.set_normal(normal[0], normal[1], normal[2]);
        m_mesh = result;
        return m_mesh;
    }
    if (m_outer_loop.is_valid())
        return mesh_delaunay(max_angle, max_edge_length, min_edge_length, max_chord_height);
    return mesh_grid(max_angle, max_edge_length, min_edge_length, max_chord_height);
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json NurbsSurface::jsondump() const {
    nlohmann::ordered_json j;

    // control_points
    {
        int cv_sz = m_is_rat ? (m_dim + 1) : m_dim;
        std::vector<double> row_major_cvs;
        row_major_cvs.reserve(m_cv_count[0] * m_cv_count[1] * cv_sz);
        for (int ci = 0; ci < m_cv_count[0]; ci++)
            for (int cj = 0; cj < m_cv_count[1]; cj++) {
                const double* cvp = cv(ci, cj);
                for (int d = 0; d < cv_sz; d++) row_major_cvs.push_back(cvp[d]);
            }
        j["control_points"] = row_major_cvs;
    }
    j["cv_count_u"] = m_cv_count[0];
    j["cv_count_v"] = m_cv_count[1];
    j["dimension"] = m_dim;

    nlohmann::ordered_json facecolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : facecolors) {
        facecolors_arr.push_back(c.r); facecolors_arr.push_back(c.g);
        facecolors_arr.push_back(c.b); facecolors_arr.push_back(c.a);
    }
    j["facecolors"] = facecolors_arr;

    j["guid"] = guid;
    if (!m_inner_loops.empty()) {
        nlohmann::ordered_json arr = nlohmann::ordered_json::array();
        for (const auto& loop : m_inner_loops) {
            arr.push_back(loop.jsondump());
        }
        j["inner_loops"] = arr;
    }
    j["is_rational"] = m_is_rat != 0;
    j["knots_u"] = m_knot[0];
    j["knots_v"] = m_knot[1];

    nlohmann::ordered_json linecolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : linecolors) {
        linecolors_arr.push_back(c.r); linecolors_arr.push_back(c.g);
        linecolors_arr.push_back(c.b); linecolors_arr.push_back(c.a);
    }
    j["linecolors"] = linecolors_arr;

    if (m_mesh.number_of_vertices() > 0) {
        j["mesh"] = m_mesh.jsondump();
    }
    j["name"] = name;
    j["order_u"] = m_order[0];
    j["order_v"] = m_order[1];
    if (m_outer_loop.is_valid()) {
        j["outer_loop"] = m_outer_loop.jsondump();
    }

    nlohmann::ordered_json pointcolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : pointcolors) {
        pointcolors_arr.push_back(c.r); pointcolors_arr.push_back(c.g);
        pointcolors_arr.push_back(c.b); pointcolors_arr.push_back(c.a);
    }
    j["pointcolors"] = pointcolors_arr;

    j["type"] = "NurbsSurface";
    j["width"] = width;
    j["xform"] = xform.jsondump();
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
    pointcolors = src.pointcolors;
    facecolors = src.facecolors;
    linecolors = src.linecolors;
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

NurbsCurve* NurbsSurface::iso_curve(int dir, double c) const {
    if ((dir != 0 && dir != 1) || !is_valid()) {
        return nullptr;
    }
    
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
namespace { bool increase_degree_impl(NurbsSurface& srf, int dir, int desired_degree); }

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

bool NurbsSurface::extend(int /*dir*/, const std::pair<double, double>& /*domain_pair*/) {
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

bool NurbsSurface::clamp_end(int dir, int /*end*/) {
    if (dir < 0 || dir > 1) return false;

    // TODO: Implement using curve conversion helpers
    return false;
}

bool NurbsSurface::increase_degree(int dir, int desired_degree) {
    // Actual implementation after helper function definitions (see increase_degree_impl)
    return increase_degree_impl(*this, dir, desired_degree);
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

double NurbsSurface::area(double /*tolerance*/) const {
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
 
        if (data.contains("pointcolors") && data["pointcolors"].is_array()) {
            const auto& arr = data["pointcolors"];
            for (size_t i = 0; i + 3 < arr.size(); i += 4) {
                surface.pointcolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                    arr[i+2].get<int>(), arr[i+3].get<int>()));
            }
        }
        if (data.contains("facecolors") && data["facecolors"].is_array()) {
            const auto& arr = data["facecolors"];
            for (size_t i = 0; i + 3 < arr.size(); i += 4) {
                surface.facecolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                    arr[i+2].get<int>(), arr[i+3].get<int>()));
            }
        }
        if (data.contains("linecolors") && data["linecolors"].is_array()) {
            const auto& arr = data["linecolors"];
            for (size_t i = 0; i + 3 < arr.size(); i += 4) {
                surface.linecolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                    arr[i+2].get<int>(), arr[i+3].get<int>()));
            }
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

    // Control vertices (always row-major: stride[0]=cv_size*cv_count[1], stride[1]=cv_size)
    int cv_sz = m_is_rat ? (m_dim + 1) : m_dim;
    for (int ci = 0; ci < m_cv_count[0]; ci++) {
        for (int cj = 0; cj < m_cv_count[1]; cj++) {
            const double* cvp = cv(ci, cj);
            for (int d = 0; d < cv_sz; d++) proto.add_cvs(cvp[d]);
        }
    }

    // Visual properties
    proto.set_width(width);

    for (const auto& c : pointcolors) {
        auto* cp = proto.add_pointcolors();
        cp->set_r(c.r); cp->set_g(c.g); cp->set_b(c.b); cp->set_a(c.a);
    }
    for (const auto& c : facecolors) {
        auto* cp = proto.add_facecolors();
        cp->set_r(c.r); cp->set_g(c.g); cp->set_b(c.b); cp->set_a(c.a);
    }
    for (const auto& c : linecolors) {
        auto* cp = proto.add_linecolors();
        cp->set_r(c.r); cp->set_g(c.g); cp->set_b(c.b); cp->set_a(c.a);
    }

    // Transform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }

    // Outer loop
    if (m_outer_loop.is_valid()) {
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

    // Cached mesh
    if (m_mesh.number_of_vertices() > 0) {
        std::string mesh_data = m_mesh.pb_dumps();
        proto.mutable_cached_mesh()->ParseFromString(mesh_data);
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

    for (int i = 0; i < proto.pointcolors_size(); ++i) {
        const auto& c = proto.pointcolors(i);
        surface.pointcolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }
    for (int i = 0; i < proto.facecolors_size(); ++i) {
        const auto& c = proto.facecolors(i);
        surface.facecolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }
    for (int i = 0; i < proto.linecolors_size(); ++i) {
        const auto& c = proto.linecolors(i);
        surface.linecolors.push_back(Color(c.r(), c.g(), c.b(), c.a()));
    }

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

    // Load cached mesh
    if (proto.has_cached_mesh() && proto.cached_mesh().vertices_size() > 0) {
        std::string mesh_data = proto.cached_mesh().SerializeAsString();
        surface.m_mesh = Mesh::pb_loads(mesh_data);
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

bool NurbsSurface::is_duplicate(const NurbsSurface& /*other*/,
                               bool /*ignore_parameterization*/,
                               double /*tolerance*/) const {
    return false; // Stub
}

bool NurbsSurface::collapse_side(int /*side*/, const Point& /*point*/) {
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

// Factory methods (create_ruled, create_extrusion, create_planar, create_loft, create_revolve,
// create_sweep1, create_sweep2, create_edge) moved to Primitives class in primitives.cpp


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

bool increase_degree_impl(NurbsSurface& srf, int dir, int desired_degree) {
    if (dir < 0 || dir > 1 || !srf.is_valid()) return false;
    if (desired_degree < srf.degree(dir)) return false;
    if (desired_degree == srf.degree(dir)) return true;

    NurbsCurve* crv = to_curve_internal(srf, dir, nullptr);
    if (!crv) return false;

    bool success = crv->increase_degree(desired_degree);
    if (success) {
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

// Loft, Revolve, Sweep1, Sweep2, Edge — moved to Primitives class in primitives.cpp

} // namespace session_cpp
