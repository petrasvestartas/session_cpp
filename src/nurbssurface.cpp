#include "nurbssurface.h"
#include "brep.h"
#include "closest.h"
#include "intersection.h"
#include "line.h"
#include "nurbsknot.h"
#include "nurbssurface.pb.h"
#include "nurbssurface_trimmed.h"
#include "remesh_nurbssurface_adaptive.h"
#include "remesh_nurbssurface_grid.h"
#include "fmt/core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace session_cpp {

namespace {

// ═══════════════════════════════════════════════════════════════════════════
// File helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Repeat each distinct knot by its multiplicity.
std::vector<double> expand_nurbsknots(const std::vector<double>& knots, const std::vector<int>& mults) {

    std::vector<double> full;

    for (size_t i = 0; i < knots.size(); i++)
        for (int m = 0; m < mults[i]; m++)
            full.push_back(knots[i]);

    return full;
}

/// C(n, k).
double binomial(int n, int k) {

    double r = 1.0;

    for (int i = 0; i < k; i++)
        r = r * (n - i) / (i + 1);

    return r;
}

/// Bounding box of a 7 x 7 sample of the surface.
std::pair<std::array<double, 3>, std::array<double, 3>> surface_aabb(const NurbsSurface& srf) {

    const int n = 6;
    const auto [u0, u1] = srf.domain(0);
    const auto [v0, v1] = srf.domain(1);
    std::array<double, 3> lo = {1e30, 1e30, 1e30};
    std::array<double, 3> hi = {-1e30, -1e30, -1e30};

    for (int i = 0; i <= n; i++)
        for (int j = 0; j <= n; j++) {
            const Point p = srf.point_at(u0 + (u1 - u0) * i / n, v0 + (v1 - v0) * j / n);

            for (int k = 0; k < 3; k++) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }

    return {lo, hi};
}

/// Boxes overlap once a is padded by a thousandth of its longest side.
bool aabb_overlap_pad(
    const std::pair<std::array<double, 3>, std::array<double, 3>>& a,
    const std::pair<std::array<double, 3>, std::array<double, 3>>& b
) {

    const double m = std::max({a.second[0] - a.first[0], a.second[1] - a.first[1], a.second[2] - a.first[2]}) * 1e-3;

    for (int k = 0; k < 3; k++)
        if (a.first[k] - m > b.second[k] || b.first[k] - m > a.second[k])
            return false;

    return true;
}

/// Flatten colors to r, g, b, a values.
nlohmann::ordered_json colors_to_json(const std::vector<Color>& colors) {

    nlohmann::ordered_json arr = nlohmann::ordered_json::array();

    for (const Color& c : colors) {
        arr.push_back(c.r);
        arr.push_back(c.g);
        arr.push_back(c.b);
        arr.push_back(c.a);
    }

    return arr;
}

/// Read colors from a flat r, g, b, a list under key.
std::vector<Color> colors_from_json(const nlohmann::json& data, const char* key) {

    std::vector<Color> colors;

    if (!data.contains(key) || !data[key].is_array())
        return colors;

    const nlohmann::json& arr = data[key];

    for (size_t i = 0; i + 3 < arr.size(); i += 4)
        colors.push_back(
            Color(arr[i].get<float>(), arr[i + 1].get<float>(), arr[i + 2].get<float>(), arr[i + 3].get<float>())
        );

    return colors;
}

/// Append colors to a repeated proto field.
void colors_to_proto(
    const std::vector<Color>& colors,
    google::protobuf::RepeatedPtrField<session_proto::Color>* field
) {

    for (const Color& c : colors) {
        session_proto::Color* cp = field->Add();
        cp->set_r(c.r);
        cp->set_g(c.g);
        cp->set_b(c.b);
        cp->set_a(c.a);
    }
}

/// Read colors from a repeated proto field.
std::vector<Color> colors_from_proto(const google::protobuf::RepeatedPtrField<session_proto::Color>& field) {

    std::vector<Color> colors;

    for (const session_proto::Color& c : field)
        colors.push_back(Color(c.r(), c.g(), c.b(), c.a()));

    return colors;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════

NurbsSurface NurbsSurface::create(
    bool periodic_u,
    bool periodic_v,
    int degree_u,
    int degree_v,
    int cv_count_u,
    int cv_count_v,
    const std::vector<Point>& points
) {

    if (degree_u < 1 || degree_v < 1)
        throw std::invalid_argument(
            fmt::format("NurbsSurface::create: degree must be >= 1, got degree_u={}, degree_v={}", degree_u, degree_v)
        );

    if (cv_count_u < degree_u + 1)
        throw std::invalid_argument(
            fmt::format("NurbsSurface::create: cv_count_u ({}) must be >= degree_u+1 ({})", cv_count_u, degree_u + 1)
        );

    if (cv_count_v < degree_v + 1)
        throw std::invalid_argument(
            fmt::format("NurbsSurface::create: cv_count_v ({}) must be >= degree_v+1 ({})", cv_count_v, degree_v + 1)
        );

    const int expected = cv_count_u * cv_count_v;

    if (static_cast<int>(points.size()) != expected)
        throw std::invalid_argument(
            fmt::format(
                "NurbsSurface::create: expected {} points ({}x{}), got {}",
                expected,
                cv_count_u,
                cv_count_v,
                static_cast<int>(points.size())
            )
        );

    NurbsSurface surface;
    surface.create_raw(3, false, degree_u + 1, degree_v + 1, cv_count_u, cv_count_v, periodic_u, periodic_v, 1.0, 1.0);

    for (int i = 0; i < cv_count_u; i++)
        for (int j = 0; j < cv_count_v; j++)
            surface.set_cv(i, j, points[i * cv_count_v + j]);

    return surface;
}

NurbsSurface NurbsSurface::create_from_parameters(
    const std::vector<std::vector<Point>>& points,
    const std::vector<std::vector<double>>& weights,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const std::vector<int>& mults_u,
    const std::vector<int>& mults_v,
    int degree_u,
    int degree_v,
    bool periodic_u,
    bool periodic_v
) {

    const int nv = static_cast<int>(points.size());
    const int nu = nv > 0 ? static_cast<int>(points[0].size()) : 0;
    const int order_u = degree_u + 1;
    const int order_v = degree_v + 1;

    if (nu < order_u || nv < order_v || periodic_u || periodic_v)
        return NurbsSurface();

    if (knots_u.size() != mults_u.size() || knots_v.size() != mults_v.size())
        return NurbsSurface();

    bool rational = false;

    for (const std::vector<double>& row : weights)
        for (double w : row)
            if (std::abs(w - 1.0) > Tolerance::ZERO_TOLERANCE)
                rational = true;

    const std::vector<double> full_u = expand_nurbsknots(knots_u, mults_u);
    const std::vector<double> full_v = expand_nurbsknots(knots_v, mults_v);
    const int kc_u = order_u + nu - 2;
    const int kc_v = order_v + nv - 2;

    if (static_cast<int>(full_u.size()) != kc_u + 2 || static_cast<int>(full_v.size()) != kc_v + 2)
        return NurbsSurface();

    NurbsSurface surface;

    if (!surface.create_raw(3, rational, order_u, order_v, nu, nv))
        return NurbsSurface();

    for (int i = 0; i < kc_u; i++)
        surface.set_nurbsknot(0, i, full_u[i + 1]);

    for (int i = 0; i < kc_v; i++)
        surface.set_nurbsknot(1, i, full_v[i + 1]);

    for (int i = 0; i < nu; i++)
        for (int j = 0; j < nv; j++) {
            const Point& p = points[j][i];

            if (rational) {
                const double w = weights[j][i];
                surface.set_cv_4d(i, j, p[0] * w, p[1] * w, p[2] * w, w);
            } else {
                surface.set_cv(i, j, p);
            }
        }

    return surface;
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

NurbsSurface::NurbsSurface() {
    initialize();
}

NurbsSurface::NurbsSurface(int dimension, bool is_rational, int order0, int order1, int cv_count0, int cv_count1) {
    initialize();
    create_raw(dimension, is_rational, order0, order1, cv_count0, cv_count1);
}

NurbsSurface::NurbsSurface(const NurbsSurface& other) {
    initialize();
    deep_copy_from(other);
}

NurbsSurface& NurbsSurface::operator=(const NurbsSurface& other) {
    if (this != &other)
        deep_copy_from(other);

    return *this;
}

bool NurbsSurface::operator==(const NurbsSurface& other) const {

    if (name != other.name || width != other.width)
        return false;

    if (pointcolors != other.pointcolors || facecolors != other.facecolors || linecolors != other.linecolors)
        return false;

    if (m_dim != other.m_dim || m_is_rat != other.m_is_rat)
        return false;

    if (m_order[0] != other.m_order[0] || m_order[1] != other.m_order[1])
        return false;

    if (m_cv_count[0] != other.m_cv_count[0] || m_cv_count[1] != other.m_cv_count[1])
        return false;

    if (m_cv_stride[0] != other.m_cv_stride[0] || m_cv_stride[1] != other.m_cv_stride[1])
        return false;

    if (m_nurbsknot[0] != other.m_nurbsknot[0] || m_nurbsknot[1] != other.m_nurbsknot[1])
        return false;

    return m_cv == other.m_cv;
}

bool NurbsSurface::operator!=(const NurbsSurface& other) const {
    return !(*this == other);
}

NurbsSurface::~NurbsSurface() {
    destroy();
}

// ═══════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════

void NurbsSurface::initialize() {

    _guid.clear();
    name = "my_nurbssurface";
    width = 1.0;
    pointcolors.clear();
    facecolors.clear();
    linecolors.clear();
    m_dim = 0;
    m_is_rat = 0;
    m_order[0] = 0;
    m_order[1] = 0;
    m_cv_count[0] = 0;
    m_cv_count[1] = 0;
    m_cv_stride[0] = 0;
    m_cv_stride[1] = 0;
    m_nurbsknot[0].clear();
    m_nurbsknot[1].clear();
    m_cv.clear();
}

bool NurbsSurface::create_raw(
    int dimension,
    bool is_rational,
    int order0,
    int order1,
    int cv_count0,
    int cv_count1,
    bool is_periodic_u,
    bool is_periodic_v,
    double nurbsknot_delta_u,
    double nurbsknot_delta_v
) {

    if (dimension < 1 || order0 < 2 || order1 < 2 || cv_count0 < order0 || cv_count1 < order1)
        return false;

    destroy();
    m_dim = dimension;
    m_is_rat = is_rational ? 1 : 0;
    m_order[0] = order0;
    m_order[1] = order1;
    m_cv_count[0] = cv_count0;
    m_cv_count[1] = cv_count1;
    m_cv_stride[1] = cv_size();
    m_cv_stride[0] = cv_size() * cv_count1;
    m_nurbsknot[0].resize(order0 + cv_count0 - 2, 0.0);
    m_nurbsknot[1].resize(order1 + cv_count1 - 2, 0.0);
    m_cv.resize(cv_count0 * cv_count1 * cv_size(), 0.0);
    zero_cvs();

    if (is_periodic_u)
        make_periodic_uniform_nurbsknot_vector(0, nurbsknot_delta_u);
    else
        make_clamped_uniform_nurbsknot_vector(0, nurbsknot_delta_u);

    if (is_periodic_v)
        make_periodic_uniform_nurbsknot_vector(1, nurbsknot_delta_v);
    else
        make_clamped_uniform_nurbsknot_vector(1, nurbsknot_delta_v);

    return true;
}

bool NurbsSurface::create_clamped_uniform(
    int dimension,
    int order0,
    int order1,
    int cv_count0,
    int cv_count1,
    double nurbsknot_delta0,
    double nurbsknot_delta1
) {

    return create_raw(
        dimension,
        false,
        order0,
        order1,
        cv_count0,
        cv_count1,
        false,
        false,
        nurbsknot_delta0,
        nurbsknot_delta1
    );
}

void NurbsSurface::destroy() {
    initialize();
}

// ═══════════════════════════════════════════════════════════════════════════
// Boolean queries
// ═══════════════════════════════════════════════════════════════════════════

bool NurbsSurface::is_valid() const {

    if (m_dim < 1 || m_order[0] < 2 || m_order[1] < 2)
        return false;

    if (m_cv_count[0] < m_order[0] || m_cv_count[1] < m_order[1])
        return false;

    if (!is_valid_nurbsknot_vector(0) || !is_valid_nurbsknot_vector(1))
        return false;

    return static_cast<int>(m_cv.size()) >= cv_count() * cv_size();
}

bool NurbsSurface::is_valid_nurbsknot_vector(int dir) const {

    if (dir < 0 || dir > 1)
        return false;

    const int kc = nurbsknot_count(dir);

    if (static_cast<int>(m_nurbsknot[dir].size()) != kc)
        return false;

    for (int i = 1; i < kc; i++)
        if (m_nurbsknot[dir][i] < m_nurbsknot[dir][i - 1])
            return false;

    return true;
}

bool NurbsSurface::is_closed(int dir) const {

    if (dir < 0 || dir > 1 || !is_valid())
        return false;

    if (!is_clamped(dir, 2))
        return is_periodic(dir);

    const int last = m_cv_count[dir] - 1;

    for (int k = 0; k < m_cv_count[1 - dir]; k++) {
        const Point a = dir ? get_cv(k, 0) : get_cv(0, k);
        const Point b = dir ? get_cv(k, last) : get_cv(last, k);

        if (a.distance(b) > Tolerance::ZERO_TOLERANCE)
            return false;
    }

    return true;
}

bool NurbsSurface::is_periodic(int dir) const {

    if (dir < 0 || dir > 1 || !is_valid())
        return false;

    if (!nurbsknot::is_periodic(m_order[dir], m_cv_count[dir], m_nurbsknot[dir]))
        return false;

    const int deg = degree(dir);
    const int n = m_cv_count[dir];

    for (int k = 0; k < m_cv_count[1 - dir]; k++)
        for (int i = 0; i < deg; i++) {
            const Point a = dir ? get_cv(k, i) : get_cv(i, k);
            const Point b = dir ? get_cv(k, n - deg + i) : get_cv(n - deg + i, k);

            if (a.distance(b) > Tolerance::ZERO_TOLERANCE)
                return false;
        }

    return true;
}

bool NurbsSurface::is_planar(Plane* plane, double tolerance) const {

    if (!is_valid())
        return false;

    const Point p0 = get_cv(0, 0);
    Vector va;
    Vector normal;

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const Point p = get_cv(i, j);
            const Vector v = p - p0;

            if (va.magnitude() < 1e-14)
                va = v;
            else if (normal.magnitude() < 1e-14)
                normal = va.cross(v);
        }

    if (normal.magnitude() < 1e-14)
        return true;

    normal = normal / normal.magnitude();

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const Point p = get_cv(i, j);
            const Vector v = p - p0;

            if (std::abs(v.dot(normal)) > tolerance)
                return false;
        }

    if (plane)
        *plane = Plane::from_point_normal(p0, normal);

    return true;
}

bool NurbsSurface::is_singular(int side) const {

    if (side < 0 || side > 3 || !is_valid())
        return false;

    const int fix = (side % 2 == 0) ? 1 : 0;
    const int end = (side == 0 || side == 3) ? 0 : 1;

    if (!is_clamped(fix, end))
        return false;

    const int at = end ? m_cv_count[fix] - 1 : 0;
    const Point first = fix ? get_cv(0, at) : get_cv(at, 0);

    for (int k = 1; k < m_cv_count[1 - fix]; k++) {
        const Point p = fix ? get_cv(k, at) : get_cv(at, k);

        if (p.distance(first) > Tolerance::ZERO_TOLERANCE)
            return false;
    }

    return true;
}

bool NurbsSurface::is_clamped(int dir, int end) const {
    if (dir < 0 || dir > 1)
        return false;

    return nurbsknot::is_clamped(m_order[dir], m_cv_count[dir], m_nurbsknot[dir], end);
}

bool NurbsSurface::is_duplicate(const NurbsSurface& other, bool ignore_parameterization, double tolerance) const {

    if (!is_valid() || !other.is_valid())
        return false;

    if (m_dim != other.m_dim || m_is_rat != other.m_is_rat)
        return false;

    if (m_order[0] != other.m_order[0] || m_order[1] != other.m_order[1])
        return false;

    if (m_cv_count[0] != other.m_cv_count[0] || m_cv_count[1] != other.m_cv_count[1])
        return false;

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            if (get_cv(i, j).distance(other.get_cv(i, j)) > tolerance)
                return false;

            if (std::abs(weight(i, j) - other.weight(i, j)) > tolerance)
                return false;
        }

    if (ignore_parameterization)
        return true;

    for (int dir = 0; dir < 2; dir++)
        for (int i = 0; i < nurbsknot_count(dir); i++)
            if (std::abs(nurbsknot(dir, i) - other.nurbsknot(dir, i)) > tolerance)
                return false;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Attributes
// ═══════════════════════════════════════════════════════════════════════════

int NurbsSurface::order(int dir) const {
    return (dir == 0 || dir == 1) ? m_order[dir] : 0;
}

int NurbsSurface::degree(int dir) const {
    return (dir == 0 || dir == 1) ? m_order[dir] - 1 : 0;
}

int NurbsSurface::cv_count(int dir) const {
    return (dir == 0 || dir == 1) ? m_cv_count[dir] : 0;
}

int NurbsSurface::cv_count() const {
    return m_cv_count[0] * m_cv_count[1];
}

int NurbsSurface::cv_size() const {
    return m_is_rat ? m_dim + 1 : m_dim;
}

int NurbsSurface::nurbsknot_count(int dir) const {
    return (dir == 0 || dir == 1) ? m_order[dir] + m_cv_count[dir] - 2 : 0;
}

int NurbsSurface::span_count(int dir) const {
    return (dir == 0 || dir == 1) ? m_cv_count[dir] - m_order[dir] + 1 : 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Control vertex access
// ═══════════════════════════════════════════════════════════════════════════

double* NurbsSurface::cv(int i, int j) {
    if (i < 0 || i >= m_cv_count[0] || j < 0 || j >= m_cv_count[1])
        return nullptr;

    return &m_cv[i * m_cv_stride[0] + j * m_cv_stride[1]];
}

const double* NurbsSurface::cv(int i, int j) const {
    if (i < 0 || i >= m_cv_count[0] || j < 0 || j >= m_cv_count[1])
        return nullptr;

    return &m_cv[i * m_cv_stride[0] + j * m_cv_stride[1]];
}

Point NurbsSurface::get_cv(int i, int j) const {

    const double* cv_ptr = cv(i, j);

    if (!cv_ptr)
        return Point(0, 0, 0);

    return dehomogenize(cv_ptr);
}

bool NurbsSurface::get_cv_4d(int i, int j, double& x, double& y, double& z, double& w) const {

    const double* cv_ptr = cv(i, j);

    if (!cv_ptr)
        return false;

    x = cv_ptr[0];
    y = m_dim > 1 ? cv_ptr[1] : 0.0;
    z = m_dim > 2 ? cv_ptr[2] : 0.0;
    w = m_is_rat ? cv_ptr[m_dim] : 1.0;

    return true;
}

bool NurbsSurface::set_cv(int i, int j, const Point& point) {

    double* cv_ptr = cv(i, j);

    if (!cv_ptr)
        return false;

    const double w = (m_is_rat && std::abs(cv_ptr[m_dim]) > 1e-14) ? cv_ptr[m_dim] : 1.0;
    cv_ptr[0] = point[0] * w;

    if (m_dim > 1)
        cv_ptr[1] = point[1] * w;

    if (m_dim > 2)
        cv_ptr[2] = point[2] * w;

    return true;
}

bool NurbsSurface::set_cv_4d(int i, int j, double x, double y, double z, double w) {

    double* cv_ptr = cv(i, j);

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

double NurbsSurface::weight(int i, int j) const {
    const double* cv_ptr = cv(i, j);

    return (m_is_rat && cv_ptr) ? cv_ptr[m_dim] : 1.0;
}

bool NurbsSurface::set_weight(int i, int j, double w) {

    double* cv_ptr = cv(i, j);

    if (!m_is_rat || !cv_ptr)
        return false;

    const double old_w = std::abs(cv_ptr[m_dim]) > 1e-14 ? cv_ptr[m_dim] : 1.0;
    const double new_w = std::abs(w) > 1e-14 ? w : 1.0;
    const double scale = new_w / old_w;

    for (int d = 0; d < m_dim; d++)
        cv_ptr[d] *= scale;

    cv_ptr[m_dim] = new_w;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// NurbsKnot access
// ═══════════════════════════════════════════════════════════════════════════

double NurbsSurface::nurbsknot(int dir, int nurbsknot_index) const {
    if (dir < 0 || dir > 1 || nurbsknot_index < 0 || nurbsknot_index >= static_cast<int>(m_nurbsknot[dir].size()))
        return 0.0;

    return m_nurbsknot[dir][nurbsknot_index];
}

bool NurbsSurface::set_nurbsknot(int dir, int nurbsknot_index, double nurbsknot_value) {

    if (dir < 0 || dir > 1 || nurbsknot_index < 0 || nurbsknot_index >= static_cast<int>(m_nurbsknot[dir].size()))
        return false;

    m_nurbsknot[dir][nurbsknot_index] = nurbsknot_value;

    return true;
}

int NurbsSurface::nurbsknot_multiplicity(int dir, int nurbsknot_index) const {
    if (dir < 0 || dir > 1)
        return 0;

    return nurbsknot::multiplicity(m_order[dir], m_cv_count[dir], m_nurbsknot[dir], nurbsknot_index);
}

std::vector<double> NurbsSurface::get_nurbsknots(int dir) const {
    return (dir == 0 || dir == 1) ? m_nurbsknot[dir] : std::vector<double>();
}

bool NurbsSurface::insert_nurbsknot(int dir, double nurbsknot_value, int nurbsknot_mult) {

    if (dir < 0 || dir > 1 || !is_valid() || nurbsknot_mult <= 0 || nurbsknot_mult >= m_order[dir])
        return false;

    const auto [t0, t1] = domain(dir);

    if (nurbsknot_value < t0 || nurbsknot_value > t1)
        return false;

    NurbsCurve crv = to_curve(dir);

    if (!crv.insert_nurbsknot(nurbsknot_value, nurbsknot_mult))
        return false;

    return from_curve(crv, dir);
}

// ═══════════════════════════════════════════════════════════════════════════
// Domain
// ═══════════════════════════════════════════════════════════════════════════

std::pair<double, double> NurbsSurface::domain(int dir) const {
    if (dir < 0 || dir > 1 || !is_valid())
        return {0.0, 0.0};

    return {m_nurbsknot[dir][m_order[dir] - 2], m_nurbsknot[dir][m_cv_count[dir] - 1]};
}

bool NurbsSurface::set_domain(int dir, double t0, double t1) {

    if (dir < 0 || dir > 1 || !is_valid() || t0 >= t1)
        return false;

    const auto [d0, d1] = domain(dir);

    if (std::abs(d1 - d0) < 1e-14)
        return false;

    const double scale = (t1 - t0) / (d1 - d0);

    for (double& k : m_nurbsknot[dir])
        k = t0 + (k - d0) * scale;

    return true;
}

std::vector<double> NurbsSurface::get_span_vector(int dir) const {

    std::vector<double> spans;

    if (dir < 0 || dir > 1 || !is_valid())
        return spans;

    spans.push_back(m_nurbsknot[dir][m_order[dir] - 2]);

    for (int i = m_order[dir] - 1; i < m_cv_count[dir]; i++)
        if (m_nurbsknot[dir][i] > spans.back())
            spans.push_back(m_nurbsknot[dir][i]);

    return spans;
}

// ═══════════════════════════════════════════════════════════════════════════
// Division
// ═══════════════════════════════════════════════════════════════════════════

std::tuple<
    std::vector<std::vector<Point>>,
    std::vector<std::vector<Vector>>,
    std::vector<std::vector<std::pair<double, double>>>>
NurbsSurface::divide_by_count_points(int nu, int nv) const {

    std::vector<std::vector<Point>> grid;
    std::vector<std::vector<Vector>> normals;
    std::vector<std::vector<std::pair<double, double>>> params;

    if (!is_valid())
        return {grid, normals, params};

    const auto [u0, u1] = domain(0);
    const auto [v0, v1] = domain(1);
    grid.resize(nu + 1);
    normals.resize(nu + 1);
    params.resize(nu + 1);

    for (int i = 0; i <= nu; i++) {
        const double u = nu > 0 ? u0 + (u1 - u0) * i / nu : u0;

        for (int j = 0; j <= nv; j++) {
            const double v = nv > 0 ? v0 + (v1 - v0) * j / nv : v0;
            grid[i].push_back(point_at(u, v));
            normals[i].push_back(normal_at(u, v));
            params[i].push_back({u, v});
        }
    }

    return {grid, normals, params};
}

std::pair<std::vector<std::vector<Plane>>, std::vector<std::vector<std::pair<double, double>>>>
NurbsSurface::divide_by_count_planes(int nu, int nv) const {

    std::vector<std::vector<Plane>> grid;
    std::vector<std::vector<std::pair<double, double>>> params;

    if (!is_valid())
        return {grid, params};

    const auto [u0, u1] = domain(0);
    const auto [v0, v1] = domain(1);
    grid.resize(nu + 1);
    params.resize(nu + 1);

    for (int i = 0; i <= nu; i++) {
        const double u = nu > 0 ? u0 + (u1 - u0) * i / nu : u0;

        for (int j = 0; j <= nv; j++) {
            const double v = nv > 0 ? v0 + (v1 - v0) * j / nv : v0;
            const std::vector<Vector> derivs = evaluate(u, v, 1);
            Vector x_axis = derivs[2];
            Vector y_axis = derivs[1];

            if (x_axis.magnitude() > 1e-14)
                x_axis = x_axis.normalized();

            if (y_axis.magnitude() > 1e-14)
                y_axis = y_axis.normalized();

            grid[i].push_back(Plane::from_frame(point_at(u, v), x_axis, y_axis, normal_at(u, v)));
            params[i].push_back({u, v});
        }
    }

    return {grid, params};
}

// ═══════════════════════════════════════════════════════════════════════════
// Evaluation
// ═══════════════════════════════════════════════════════════════════════════

Point NurbsSurface::point_at(double u, double v) const {

    if (!is_valid())
        return Point(0, 0, 0);

    const int span_u = find_span(0, u);
    const int span_v = find_span(1, v);
    const std::vector<double> nu = nurbsknot::eval_basis(m_order[0], m_nurbsknot[0], span_u, u);
    const std::vector<double> nv = nurbsknot::eval_basis(m_order[1], m_nurbsknot[1], span_v, v);
    const int size = cv_size();
    std::vector<double> sum(size, 0.0);

    for (int i = 0; i < m_order[0]; i++)
        for (int j = 0; j < m_order[1]; j++) {
            const double c = nu[i] * nv[j];
            const double* cv_ptr = cv(span_u + i, span_v + j);

            for (int d = 0; d < size; d++)
                sum[d] += c * cv_ptr[d];
        }

    return dehomogenize(sum.data());
}

std::pair<double, double> NurbsSurface::closest_parameters(const Point& test_point) const {
    const std::tuple<double, double, double> hit = Closest::surface_point(*this, test_point);

    return {std::get<0>(hit), std::get<1>(hit)};
}

Point NurbsSurface::closest_point(const Point& test_point) const {
    const auto [u, v] = closest_parameters(test_point);

    return point_at(u, v);
}

double NurbsSurface::gaussian_curvature(double u, double v) const {

    double E;
    double F;
    double G;
    double L;
    double M;
    double N;

    if (!fundamental_forms(u, v, E, F, G, L, M, N))
        return 0.0;

    const double denom = E * G - F * F;

    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE)
        return 0.0;

    return (L * N - M * M) / denom;
}

double NurbsSurface::mean_curvature(double u, double v) const {

    double E;
    double F;
    double G;
    double L;
    double M;
    double N;

    if (!fundamental_forms(u, v, E, F, G, L, M, N))
        return 0.0;

    const double denom = E * G - F * F;

    if (std::abs(denom) < Tolerance::ZERO_TOLERANCE)
        return 0.0;

    return (E * N - 2.0 * F * M + G * L) / (2.0 * denom);
}

Vector NurbsSurface::normal_at(double u, double v) const {

    const std::vector<Vector> derivs = evaluate(u, v, 1);

    if (derivs.size() < 3)
        return Vector(0, 0, 1);

    const Vector normal = derivs[2].cross(derivs[1]);
    const double len = normal.magnitude();

    if (len < 1e-14)
        return Vector(0, 0, 1);

    return normal / len;
}

Plane NurbsSurface::frame_at(double u, double v) const {

    const std::vector<Vector> derivs = evaluate(u, v, 1);

    if (derivs.size() < 3)
        return Plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));

    return Plane(Point(derivs[0][0], derivs[0][1], derivs[0][2]), derivs[2], derivs[1]);
}

std::vector<Point> NurbsSurface::intersections_with_line(const Line& line) const {

    std::vector<Point> results;

    if (!is_valid())
        return results;

    const Point p0 = line.start();
    const Point pe = line.end();
    Vector d = pe - p0;

    if (d.magnitude() < 1e-14)
        return results;

    d = d.normalized();
    const Vector helper = std::abs(d[0]) < 0.9 ? Vector(1, 0, 0) : Vector(0, 1, 0);
    const Vector n1 = d.cross(helper).normalized();
    const Vector n2 = d.cross(n1).normalized();
    const auto [u0, u1] = domain(0);
    const auto [v0, v1] = domain(1);
    const int nu = std::max(12, cv_count(0) * 4);
    const int nv = std::max(12, cv_count(1) * 4);

    for (int a = 0; a <= nu; a++)
        for (int b = 0; b <= nv; b++) {
            double u = u0 + (u1 - u0) * a / nu;
            double v = v0 + (v1 - v0) * b / nv;

            if (!line_newton(u, v, p0, n1, n2))
                continue;

            const Point p = point_at(u, v);
            const Vector r = p - p0;

            if (std::abs(n1.dot(r)) > 1e-7 || std::abs(n2.dot(r)) > 1e-7)
                continue;

            bool dup = false;

            for (const Point& q : results)
                if (p.distance(q) < 1e-6)
                    dup = true;

            if (!dup)
                results.push_back(p);
        }

    return results;
}

std::vector<Vector> NurbsSurface::evaluate(double u, double v, int num_derivs) const {

    std::vector<Vector> result;

    if (!is_valid() || num_derivs < 0)
        return result;

    const int n = std::min(num_derivs, 2);
    const int span_u = find_span(0, u);
    const int span_v = find_span(1, v);
    const std::vector<std::vector<double>> ders_u = basis_functions_derivatives(0, span_u, u, n);
    const std::vector<std::vector<double>> ders_v = basis_functions_derivatives(1, span_v, v, n);
    const int size = cv_size();
    std::vector<std::vector<double>> skl;

    for (int k = 0; k <= n; k++)
        for (int l = 0; l <= n - k; l++) {
            std::vector<double> sum(size, 0.0);

            for (int i = 0; i < m_order[0]; i++)
                for (int j = 0; j < m_order[1]; j++) {
                    const double c = ders_u[k][i] * ders_v[l][j];
                    const double* cv_ptr = cv(span_u + i, span_v + j);

                    for (int d = 0; d < size; d++)
                        sum[d] += c * cv_ptr[d];
                }

            skl.push_back(sum);
        }

    if (m_is_rat)
        return rational_derivatives(skl, n);

    for (const std::vector<double>& s : skl)
        result.push_back(Vector(s[0], m_dim > 1 ? s[1] : 0.0, m_dim > 2 ? s[2] : 0.0));

    return result;
}

Point NurbsSurface::point_at_corner(int u_end, int v_end) const {
    const int i = u_end == 0 ? 0 : m_cv_count[0] - 1;
    const int j = v_end == 0 ? 0 : m_cv_count[1] - 1;

    return get_cv(i, j);
}

NurbsCurve NurbsSurface::iso_curve(int dir, double c) const {

    if (dir < 0 || dir > 1 || !is_valid())
        return NurbsCurve();

    NurbsCurve crv(m_dim, m_is_rat != 0, m_order[dir], m_cv_count[dir]);

    for (int i = 0; i < crv.nurbsknot_count(); i++)
        crv.set_nurbsknot(i, nurbsknot(dir, i));

    const int other = 1 - dir;
    const int span = find_span(other, c);
    const std::vector<double> basis = nurbsknot::eval_basis(m_order[other], m_nurbsknot[other], span, c);
    const int size = cv_size();

    for (int i = 0; i < m_cv_count[dir]; i++) {
        std::vector<double> sum(size, 0.0);

        for (int k = 0; k < m_order[other]; k++) {
            const double* cv_ptr = dir ? cv(span + k, i) : cv(i, span + k);

            for (int d = 0; d < size; d++)
                sum[d] += basis[k] * cv_ptr[d];
        }

        const Point p(sum[0], m_dim > 1 ? sum[1] : 0.0, m_dim > 2 ? sum[2] : 0.0);

        if (m_is_rat)
            crv.set_cv_4d(i, p[0], p[1], p[2], sum[m_dim]);
        else
            crv.set_cv(i, p);
    }

    return crv;
}

// ═══════════════════════════════════════════════════════════════════════════
// Modification
// ═══════════════════════════════════════════════════════════════════════════

bool NurbsSurface::reverse(int dir) {

    if (dir < 0 || dir > 1 || !is_valid())
        return false;

    nurbsknot::reverse(m_order[dir], m_cv_count[dir], m_nurbsknot[dir]);
    const int n = m_cv_count[dir];
    const int size = cv_size();

    for (int k = 0; k < m_cv_count[1 - dir]; k++)
        for (int i = 0; i < n / 2; i++) {
            double* a = dir ? cv(k, i) : cv(i, k);
            double* b = dir ? cv(k, n - 1 - i) : cv(n - 1 - i, k);

            for (int d = 0; d < size; d++)
                std::swap(a[d], b[d]);
        }

    return true;
}

bool NurbsSurface::transpose() {

    if (!is_valid())
        return false;

    const int size = cv_size();
    std::vector<double> new_cv(m_cv.size());

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* src = cv(i, j);
            double* dst = &new_cv[(j * m_cv_count[0] + i) * size];

            for (int d = 0; d < size; d++)
                dst[d] = src[d];
        }

    m_cv = new_cv;
    std::swap(m_order[0], m_order[1]);
    std::swap(m_cv_count[0], m_cv_count[1]);
    std::swap(m_nurbsknot[0], m_nurbsknot[1]);
    m_cv_stride[0] = size * m_cv_count[1];

    return true;
}

bool NurbsSurface::swap_coordinates(int axis_i, int axis_j) {

    if (axis_i < 0 || axis_i >= m_dim || axis_j < 0 || axis_j >= m_dim)
        return false;

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            double* cv_ptr = cv(i, j);
            std::swap(cv_ptr[axis_i], cv_ptr[axis_j]);
        }

    return true;
}

bool NurbsSurface::trim(int dir, const std::pair<double, double>& domain_pair) {

    if (dir < 0 || dir > 1 || !is_valid())
        return false;

    NurbsCurve crv = to_curve(dir);

    if (!crv.trim(domain_pair.first, domain_pair.second))
        return false;

    return from_curve(crv, dir);
}

std::pair<NurbsSurface, NurbsSurface> NurbsSurface::split(int dir, double c) const {

    if (dir < 0 || dir > 1 || !is_valid())
        return {NurbsSurface(), NurbsSurface()};

    const auto [t0, t1] = domain(dir);

    if (c <= t0 || c >= t1)
        return {NurbsSurface(), NurbsSurface()};

    NurbsSurface lo = *this;
    NurbsSurface hi = *this;

    if (!lo.trim(dir, {t0, c}) || !hi.trim(dir, {c, t1}))
        return {NurbsSurface(), NurbsSurface()};

    return {lo, hi};
}

bool NurbsSurface::make_rational() {

    if (m_is_rat)
        return true;

    std::vector<double> new_cv(cv_count() * (m_dim + 1));

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* src = cv(i, j);
            double* dst = &new_cv[(i * m_cv_count[1] + j) * (m_dim + 1)];

            for (int d = 0; d < m_dim; d++)
                dst[d] = src[d];

            dst[m_dim] = 1.0;
        }

    m_cv = new_cv;
    m_is_rat = 1;
    m_cv_stride[1] = m_dim + 1;
    m_cv_stride[0] = (m_dim + 1) * m_cv_count[1];

    return true;
}

bool NurbsSurface::make_non_rational() {

    if (!m_is_rat)
        return true;

    std::vector<double> new_cv(cv_count() * m_dim);

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* src = cv(i, j);
            double* dst = &new_cv[(i * m_cv_count[1] + j) * m_dim];
            const double w = std::abs(src[m_dim]) > 1e-14 ? src[m_dim] : 1.0;

            for (int d = 0; d < m_dim; d++)
                dst[d] = src[d] / w;
        }

    m_cv = new_cv;
    m_is_rat = 0;
    m_cv_stride[1] = m_dim;
    m_cv_stride[0] = m_dim * m_cv_count[1];

    return true;
}

bool NurbsSurface::increase_degree(int dir, int desired_degree) {

    if (dir < 0 || dir > 1 || !is_valid() || desired_degree < degree(dir))
        return false;

    if (desired_degree == degree(dir))
        return true;

    NurbsCurve crv = to_curve(dir);

    if (!crv.increase_degree(desired_degree))
        return false;

    return from_curve(crv, dir);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

bool NurbsSurface::transform(const Xform& xform) {

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            Point p = get_cv(i, j);
            p.transform(xform);
            set_cv(i, j, p);
        }

    return true;
}

NurbsSurface NurbsSurface::transformed(const Xform& xform) const {
    NurbsSurface result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Splitting
// ═══════════════════════════════════════════════════════════════════════════

std::vector<NurbsSurfaceTrimmed> NurbsSurface::split_by_plane(const Plane& plane, double tolerance) const {

    std::vector<NurbsCurve> pcurves;

    for (const std::pair<NurbsCurve, NurbsCurve>& pair : Intersection::surface_plane_uv(*this, plane, tolerance))
        pcurves.push_back(pair.second);

    return NurbsSurfaceTrimmed::split_by_uv_curves(*this, pcurves, tolerance);
}

std::vector<NurbsSurfaceTrimmed> NurbsSurface::split_by_curves(
    const std::vector<NurbsCurve>& curves,
    double tolerance
) const {

    std::vector<NurbsCurve> pcurves;

    for (const NurbsCurve& crv : curves)
        for (const NurbsCurve& pcurve : Closest::surface_curve(*this, crv, 0.0, 0.0, tolerance))
            pcurves.push_back(pcurve);

    return NurbsSurfaceTrimmed::split_by_uv_curves(*this, pcurves, tolerance);
}

std::vector<NurbsSurfaceTrimmed> NurbsSurface::split_by_line(const Line& line, double tolerance) const {
    const std::vector<Point> points = {line.start(), line.end()};

    return split_by_curves({NurbsCurve::create(false, 1, points)}, tolerance);
}

std::vector<NurbsSurfaceTrimmed> NurbsSurface::split_by_surface(const NurbsSurface& cutter, double tolerance) const {

    std::vector<NurbsCurve> pcurves;

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& triple : Intersection::surface_surface(*this, cutter, tolerance))
        pcurves.push_back(std::get<1>(triple));

    return NurbsSurfaceTrimmed::split_by_uv_curves(*this, pcurves, tolerance);
}

std::vector<NurbsSurfaceTrimmed> NurbsSurface::split_by_brep(const BRep& brep, double tolerance) const {

    const std::pair<std::array<double, 3>, std::array<double, 3>> target_bb = surface_aabb(*this);
    std::vector<NurbsCurve> pcurves;

    for (const NurbsSurface& cutter : brep.m_surfaces) {
        if (!aabb_overlap_pad(target_bb, surface_aabb(cutter)))
            continue;

        for (const NurbsCurve& pcurve : Intersection::cut_curves_on_surface(*this, cutter, tolerance))
            pcurves.push_back(pcurve);
    }

    return NurbsSurfaceTrimmed::split_by_uv_curves(*this, pcurves, tolerance);
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshing
// ═══════════════════════════════════════════════════════════════════════════

Mesh NurbsSurface::mesh_adaptive(
    double max_angle,
    double max_edge_length,
    double min_edge_length,
    double max_chord_height
) const {

    if (m_mesh.number_of_vertices() == 0 && is_valid()) {
        RemeshNurbsSurfaceAdaptive mesher(*this);
        mesher.set_max_angle(max_angle)
            .set_max_edge_length(max_edge_length)
            .set_min_edge_length(min_edge_length)
            .set_max_chord_height(max_chord_height);

        m_mesh = mesher.mesh();
    }

    return m_mesh;
}

Mesh NurbsSurface::mesh() const {
    if (m_mesh.number_of_vertices() == 0 && is_valid())
        m_mesh = is_planar(nullptr, 1e-6) ? mesh_planar() : RemeshNurbsSurfaceGrid::from_u_v(*this, 0, 0);

    return m_mesh;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json NurbsSurface::jsondump() const {

    std::vector<double> control_points;

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* cv_ptr = cv(i, j);

            for (int d = 0; d < cv_size(); d++)
                control_points.push_back(cv_ptr[d]);
        }

    nlohmann::ordered_json j;
    j["control_points"] = control_points;
    j["cv_count_u"] = m_cv_count[0];
    j["cv_count_v"] = m_cv_count[1];
    j["dimension"] = m_dim;
    j["facecolors"] = colors_to_json(facecolors);
    j["guid"] = guid();
    j["is_rational"] = m_is_rat != 0;
    j["linecolors"] = colors_to_json(linecolors);

    if (m_mesh.number_of_vertices() > 0)
        j["mesh"] = m_mesh.jsondump();

    j["name"] = name;
    j["nurbsknots_u"] = m_nurbsknot[0];
    j["nurbsknots_v"] = m_nurbsknot[1];
    j["order_u"] = m_order[0];
    j["order_v"] = m_order[1];
    j["pointcolors"] = colors_to_json(pointcolors);
    j["type"] = "NurbsSurface";
    j["width"] = width;

    return j;
}

NurbsSurface NurbsSurface::jsonload(const nlohmann::json& data) {

    NurbsSurface surface;

    if (!data.contains("dimension") || !data.contains("order_u") || !data.contains("order_v") ||
        !data.contains("cv_count_u") || !data.contains("cv_count_v"))
        return surface;

    surface.create_raw(
        data["dimension"],
        data.value("is_rational", false),
        data["order_u"],
        data["order_v"],
        data["cv_count_u"],
        data["cv_count_v"]
    );

    if (data.contains("nurbsknots_u"))
        surface.m_nurbsknot[0] = data["nurbsknots_u"].get<std::vector<double>>();

    if (data.contains("nurbsknots_v"))
        surface.m_nurbsknot[1] = data["nurbsknots_v"].get<std::vector<double>>();

    if (data.contains("control_points"))
        surface.m_cv = data["control_points"].get<std::vector<double>>();

    surface.guid() = data.value("guid", ::guid());
    surface.name = data.value("name", "my_nurbssurface");
    surface.width = data.value("width", 1.0);
    surface.pointcolors = colors_from_json(data, "pointcolors");
    surface.facecolors = colors_from_json(data, "facecolors");
    surface.linecolors = colors_from_json(data, "linecolors");

    if (data.contains("mesh") && !data["mesh"].is_null())
        surface.m_mesh = Mesh::jsonload(data["mesh"]);

    return surface;
}

void NurbsSurface::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

NurbsSurface NurbsSurface::file_json_load(const std::string& filename) {

    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;

    return jsonload(data);
}

std::string NurbsSurface::file_json_dumps() const {
    return jsondump().dump();
}

NurbsSurface NurbsSurface::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

std::string NurbsSurface::pb_dumps() const {

    session_proto::NurbsSurface proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_dimension(m_dim);
    proto.set_is_rational(m_is_rat != 0);
    proto.set_order_u(m_order[0]);
    proto.set_order_v(m_order[1]);
    proto.set_cv_count_u(m_cv_count[0]);
    proto.set_cv_count_v(m_cv_count[1]);
    proto.set_cv_stride_u(m_cv_stride[0]);
    proto.set_cv_stride_v(m_cv_stride[1]);

    for (double k : m_nurbsknot[0])
        proto.add_nurbsknots_u(k);

    for (double k : m_nurbsknot[1])
        proto.add_nurbsknots_v(k);

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const double* cv_ptr = cv(i, j);

            for (int d = 0; d < cv_size(); d++)
                proto.add_cvs(cv_ptr[d]);
        }

    proto.set_width(width);
    colors_to_proto(pointcolors, proto.mutable_pointcolors());
    colors_to_proto(facecolors, proto.mutable_facecolors());
    colors_to_proto(linecolors, proto.mutable_linecolors());

    if (m_mesh.number_of_vertices() > 0)
        proto.mutable_cached_mesh()->ParseFromString(m_mesh.pb_dumps());

    return proto.SerializeAsString();
}

NurbsSurface NurbsSurface::pb_loads(const std::string& data) {

    session_proto::NurbsSurface proto;
    proto.ParseFromString(data);
    NurbsSurface surface;
    surface.create_raw(
        proto.dimension(),
        proto.is_rational(),
        proto.order_u(),
        proto.order_v(),
        proto.cv_count_u(),
        proto.cv_count_v()
    );

    if (!proto.guid().empty())
        surface.guid() = proto.guid();

    surface.name = proto.name();

    for (int i = 0; i < proto.nurbsknots_u_size() && i < static_cast<int>(surface.m_nurbsknot[0].size()); i++)
        surface.m_nurbsknot[0][i] = proto.nurbsknots_u(i);

    for (int i = 0; i < proto.nurbsknots_v_size() && i < static_cast<int>(surface.m_nurbsknot[1].size()); i++)
        surface.m_nurbsknot[1][i] = proto.nurbsknots_v(i);

    const int size = surface.cv_size();
    const int stride_u = proto.cv_stride_u() > 0 ? proto.cv_stride_u() : size * surface.m_cv_count[1];
    const int stride_v = proto.cv_stride_v() > 0 ? proto.cv_stride_v() : size;

    for (int i = 0; i < surface.m_cv_count[0]; i++)
        for (int j = 0; j < surface.m_cv_count[1]; j++) {
            const int src = i * stride_u + j * stride_v;
            double* dst = surface.cv(i, j);

            for (int d = 0; d < size && src + d < proto.cvs_size(); d++)
                dst[d] = proto.cvs(src + d);
        }

    surface.width = proto.width();
    surface.pointcolors = colors_from_proto(proto.pointcolors());
    surface.facecolors = colors_from_proto(proto.facecolors());
    surface.linecolors = colors_from_proto(proto.linecolors());

    if (proto.has_cached_mesh() && proto.cached_mesh().vertices_size() > 0)
        surface.m_mesh = Mesh::pb_loads(proto.cached_mesh().SerializeAsString());

    return surface;
}

void NurbsSurface::pb_dump(const std::string& filename) const {
    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

NurbsSurface NurbsSurface::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════

std::string NurbsSurface::str() const {

    return fmt::format(
        "NurbsSurface(name={}, degree=({},{}), cvs=({},{}))",
        name,
        degree(0),
        degree(1),
        m_cv_count[0],
        m_cv_count[1]
    );
}

std::string NurbsSurface::repr() const {

    std::string result = fmt::format(
        "NurbsSurface(\n  name={},\n  degree=({},{}),\n  cvs=({},{}),\n  rational={},\n  control_points=[\n",
        name,
        degree(0),
        degree(1),
        m_cv_count[0],
        m_cv_count[1],
        m_is_rat ? "true" : "false"
    );

    for (int i = 0; i < m_cv_count[0]; i++)
        for (int j = 0; j < m_cv_count[1]; j++) {
            const Point p = get_cv(i, j);
            result += fmt::format("    {}, {}, {}\n", p[0], p[1], p[2]);
        }

    result += "  ]\n)";

    return result;
}

std::ostream& operator<<(std::ostream& os, const NurbsSurface& surface) {
    os << surface.str();

    return os;
}

// ═══════════════════════════════════════════════════════════════════════════
// Private helpers
// ═══════════════════════════════════════════════════════════════════════════

void NurbsSurface::deep_copy_from(const NurbsSurface& src) {

    _guid.clear();
    name = src.name;
    width = src.width;
    pointcolors = src.pointcolors;
    facecolors = src.facecolors;
    linecolors = src.linecolors;
    m_dim = src.m_dim;
    m_is_rat = src.m_is_rat;
    m_order[0] = src.m_order[0];
    m_order[1] = src.m_order[1];
    m_cv_count[0] = src.m_cv_count[0];
    m_cv_count[1] = src.m_cv_count[1];
    m_cv_stride[0] = src.m_cv_stride[0];
    m_cv_stride[1] = src.m_cv_stride[1];
    m_nurbsknot[0] = src.m_nurbsknot[0];
    m_nurbsknot[1] = src.m_nurbsknot[1];
    m_cv = src.m_cv;
    m_mesh = src.m_mesh;
}

bool NurbsSurface::zero_cvs() {

    std::fill(m_cv.begin(), m_cv.end(), 0.0);

    if (m_is_rat)
        for (int i = 0; i < m_cv_count[0]; i++)
            for (int j = 0; j < m_cv_count[1]; j++)
                cv(i, j)[m_dim] = 1.0;

    return true;
}

bool NurbsSurface::make_clamped_uniform_nurbsknot_vector(int dir, double delta) {

    if (dir < 0 || dir > 1 || delta <= 0.0)
        return false;

    m_nurbsknot[dir] = nurbsknot::make_clamped_uniform(m_order[dir], m_cv_count[dir], delta);

    return !m_nurbsknot[dir].empty();
}

bool NurbsSurface::make_periodic_uniform_nurbsknot_vector(int dir, double delta) {

    if (dir < 0 || dir > 1 || delta <= 0.0)
        return false;

    m_nurbsknot[dir] = nurbsknot::make_periodic_uniform(m_order[dir], m_cv_count[dir], delta);

    return !m_nurbsknot[dir].empty();
}

Point NurbsSurface::dehomogenize(const double* h) const {
    const double w = (m_is_rat && std::abs(h[m_dim]) > 1e-14) ? h[m_dim] : 1.0;

    return Point(h[0] / w, m_dim > 1 ? h[1] / w : 0.0, m_dim > 2 ? h[2] / w : 0.0);
}

int NurbsSurface::find_span(int dir, double t) const {
    return nurbsknot::find_span(m_order[dir], m_cv_count[dir], m_nurbsknot[dir], t);
}

std::vector<std::vector<double>> NurbsSurface::basis_functions_derivatives(
    int dir,
    int span,
    double t,
    int deriv_order
) const {

    const int order = m_order[dir];
    const int degree = order - 1;
    const std::vector<double>& knot = m_nurbsknot[dir];
    const int base = span + degree;
    std::vector<std::vector<double>> ders(deriv_order + 1, std::vector<double>(order, 0.0));

    if (knot[base - 1] == knot[base])
        return ders;

    std::vector<std::vector<double>> ndu(order, std::vector<double>(order, 0.0));
    ndu[0][0] = 1.0;
    std::vector<double> left(order, 0.0);
    std::vector<double> right(order, 0.0);

    for (int j = 1; j <= degree; j++) {
        left[j] = t - knot[base - j];
        right[j] = knot[base + j - 1] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        ndu[j][j] = saved;
    }

    for (int j = 0; j <= degree; j++)
        ders[0][j] = ndu[j][degree];

    std::vector<std::vector<double>> a(2, std::vector<double>(order, 0.0));

    for (int r = 0; r <= degree; r++) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;

        for (int k = 1; k <= deriv_order; k++) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = degree - k;

            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            const int j1 = rk >= -1 ? 1 : -rk;
            const int j2 = r - 1 <= pk ? k - 1 : degree - r;

            for (int j = j1; j <= j2; j++) {
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

    double factor = degree;

    for (int k = 1; k <= deriv_order; k++) {
        for (int j = 0; j <= degree; j++)
            ders[k][j] *= factor;

        factor *= degree - k;
    }

    return ders;
}

std::vector<Vector> NurbsSurface::rational_derivatives(
    const std::vector<std::vector<double>>& skl,
    int num_derivs
) const {

    std::vector<Vector> result;
    const int n = num_derivs;
    const double w00 = skl[0][m_dim];

    if (std::abs(w00) < 1e-14)
        return std::vector<Vector>(skl.size(), Vector(0, 0, 0));

    for (int k = 0; k <= n; k++)
        for (int l = 0; l <= n - k; l++) {
            const std::vector<double>& s = skl[k * (n + 1) - k * (k - 1) / 2 + l];
            Vector a(s[0], m_dim > 1 ? s[1] : 0.0, m_dim > 2 ? s[2] : 0.0);

            for (int i = 0; i <= k; i++)
                for (int j = 0; j <= l; j++) {
                    if (i == 0 && j == 0)
                        continue;

                    const double c = binomial(k, i) * binomial(l, j) * skl[i * (n + 1) - i * (i - 1) / 2 + j][m_dim];
                    a -= result[(k - i) * (n + 1) - (k - i) * (k - i - 1) / 2 + (l - j)] * c;
                }

            result.push_back(a / w00);
        }

    return result;
}

bool NurbsSurface::line_newton(double& u, double& v, const Point& p0, const Vector& n1, const Vector& n2) const {

    const auto [u0, u1] = domain(0);
    const auto [v0, v1] = domain(1);

    for (int it = 0; it < 40; it++) {
        const std::vector<Vector> der = evaluate(u, v, 1);

        if (der.size() < 3)
            return false;

        const Vector r(der[0][0] - p0[0], der[0][1] - p0[1], der[0][2] - p0[2]);
        const double f1 = n1.dot(r);
        const double f2 = n2.dot(r);

        if (std::abs(f1) < 1e-12 && std::abs(f2) < 1e-12)
            return true;

        const double j11 = n1.dot(der[2]);
        const double j12 = n1.dot(der[1]);
        const double j21 = n2.dot(der[2]);
        const double j22 = n2.dot(der[1]);
        const double det = j11 * j22 - j12 * j21;

        if (std::abs(det) < 1e-14)
            return false;

        const double du = -(j22 * f1 - j12 * f2) / det;
        const double dv = -(-j21 * f1 + j11 * f2) / det;
        u += du;
        v += dv;

        if (u < u0 || u > u1 || v < v0 || v > v1)
            return false;

        if (std::abs(du) < 1e-13 && std::abs(dv) < 1e-13)
            return true;
    }

    return true;
}

bool NurbsSurface::fundamental_forms(
    double u,
    double v,
    double& E,
    double& F,
    double& G,
    double& L,
    double& M,
    double& N
) const {

    const std::vector<Vector> d = evaluate(u, v, 2);

    if (d.size() < 6)
        return false;

    const Vector& sv = d[1];
    const Vector& svv = d[2];
    const Vector& su = d[3];
    const Vector& suv = d[4];
    const Vector& suu = d[5];
    const Vector cr = su.cross(sv);

    if (cr.magnitude() < Tolerance::ZERO_TOLERANCE)
        return false;

    const Vector n = cr.normalized();
    E = su.dot(su);
    F = su.dot(sv);
    G = sv.dot(sv);
    L = suu.dot(n);
    M = suv.dot(n);
    N = svv.dot(n);

    return true;
}

Mesh NurbsSurface::mesh_planar() const {

    Mesh result;
    const Point p00 = point_at_corner(0, 0);
    const Point p10 = point_at_corner(1, 0);
    const Point p11 = point_at_corner(1, 1);
    const Point p01 = point_at_corner(0, 1);
    const size_t v0 = result.add_vertex(p00);
    const size_t v1 = result.add_vertex(p10);
    const size_t v2 = result.add_vertex(p11);
    result.add_face({v0, v1, v2});
    Vector normal;

    if (p00.distance(p01) < 1e-10) {
        const Vector e1 = p10 - p00;
        const Vector e2 = p11 - p00;
        normal = e1.cross(e2);
    } else {
        const size_t v3 = result.add_vertex(p01);
        result.add_face({v0, v2, v3});
        const std::vector<Vector> derivs = evaluate(0.5, 0.5, 1);
        normal = derivs[1].cross(derivs[2]);
    }

    if (normal.magnitude() > 1e-15)
        normal = normal.normalized();

    for (auto& [vi, pt] : result.vertex)
        pt.set_normal(normal[0], normal[1], normal[2]);

    return result;
}

NurbsCurve NurbsSurface::to_curve(int dir) const {

    const int other = 1 - dir;
    const int size = cv_size();
    NurbsCurve crv(size * m_cv_count[other], false, m_order[dir], m_cv_count[dir]);

    for (int i = 0; i < crv.nurbsknot_count(); i++)
        crv.set_nurbsknot(i, nurbsknot(dir, i));

    for (int i = 0; i < m_cv_count[dir]; i++) {
        double* dst = crv.cv(i);

        for (int j = 0; j < m_cv_count[other]; j++) {
            const double* src = dir ? cv(j, i) : cv(i, j);

            for (int d = 0; d < size; d++)
                dst[j * size + d] = src[d];
        }
    }

    return crv;
}

bool NurbsSurface::from_curve(const NurbsCurve& crv, int dir) {

    const int other = 1 - dir;
    const int size = cv_size();

    if (crv.m_is_rat || crv.m_dim != size * m_cv_count[other])
        return false;

    NurbsSurface srf;

    if (dir == 0)
        srf.create_raw(m_dim, m_is_rat != 0, crv.m_order, m_order[1], crv.m_cv_count, m_cv_count[1]);
    else
        srf.create_raw(m_dim, m_is_rat != 0, m_order[0], crv.m_order, m_cv_count[0], crv.m_cv_count);

    srf.m_nurbsknot[dir] = crv.m_nurbsknot;
    srf.m_nurbsknot[other] = m_nurbsknot[other];

    for (int i = 0; i < crv.m_cv_count; i++) {
        const double* src = crv.cv(i);

        for (int j = 0; j < m_cv_count[other]; j++) {
            double* dst = dir ? srf.cv(j, i) : srf.cv(i, j);

            for (int d = 0; d < size; d++)
                dst[d] = src[j * size + d];
        }
    }

    m_order[dir] = srf.m_order[dir];
    m_cv_count[dir] = srf.m_cv_count[dir];
    m_cv_stride[0] = srf.m_cv_stride[0];
    m_cv_stride[1] = srf.m_cv_stride[1];
    m_nurbsknot[dir] = srf.m_nurbsknot[dir];
    m_cv = srf.m_cv;

    return true;
}

} // namespace session_cpp
