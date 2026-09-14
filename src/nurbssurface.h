#pragma once

#include "color.h"
#include "guid.h"
#include "json.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "plane.h"
#include "point.h"
#include "tolerance.h"
#include "vector.h"
#include "xform.h"
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace session_cpp {

class Line;
class BRep;
class NurbsSurfaceTrimmed;

/// A NURBS surface: OpenNURBS layout, nurbsknot count = order + cv_count - 2 per direction, homogeneous row-major CVs when rational
class NurbsSurface {
public:
    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    /// Clear the guid so a fresh one mints lazily on next read
    void refresh_guid() { _guid.clear(); }
    std::string name = "my_nurbssurface";
    double width = 1.0;
    std::vector<Color> pointcolors;
    std::vector<Color> facecolors;
    std::vector<Color> linecolors;

    int m_dim;
    int m_is_rat;
    int m_order[2];
    int m_cv_count[2];
    int m_cv_stride[2];
    std::vector<double> m_nurbsknot[2];
    std::vector<double> m_cv;
    mutable Mesh m_mesh;

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Clamped or periodic uniform surface through cv_count_u x cv_count_v points in row-major order (u slowest)
    static NurbsSurface create(bool periodic_u, bool periodic_v, int degree_u, int degree_v, int cv_count_u, int cv_count_v, const std::vector<Point>& points);

    /// OCCT convention: points[iv][iu], weights[iv][iu], distinct knots with multiplicities per direction
    static NurbsSurface create_from_parameters(
        const std::vector<std::vector<Point>>& points,
        const std::vector<std::vector<double>>& weights,
        const std::vector<double>& knots_u,
        const std::vector<double>& knots_v,
        const std::vector<int>& mults_u,
        const std::vector<int>& mults_v,
        int degree_u,
        int degree_v,
        bool periodic_u = false,
        bool periodic_v = false
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    NurbsSurface();
    NurbsSurface(int dimension, bool is_rational, int order0, int order1, int cv_count0, int cv_count1);
    /// Copy (new guid, same data)
    NurbsSurface(const NurbsSurface& other);
    /// Move keeps the guid: the same object in a new place
    NurbsSurface(NurbsSurface&& other) noexcept = default;
    NurbsSurface& operator=(NurbsSurface&& other) noexcept = default;
    NurbsSurface& operator=(const NurbsSurface& other);
    /// Same name, width, colors, layout, nurbsknots and CVs; guid ignored
    bool operator==(const NurbsSurface& other) const;
    bool operator!=(const NurbsSurface& other) const;
    ~NurbsSurface();

    // ═══════════════════════════════════════════════════════════════════════════
    // Initialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Reset every field to the empty invalid surface
    void initialize();

    /// Allocate nurbsknots (clamped or periodic uniform) and zeroed CVs; false when order < 2 or cv_count < order
    bool create_raw(int dimension, bool is_rational, int order0, int order1, int cv_count0, int cv_count1, bool is_periodic_u = false, bool is_periodic_v = false, double nurbsknot_delta_u = 1.0, double nurbsknot_delta_v = 1.0);

    /// Non-rational surface with clamped uniform nurbsknots of the given spacing
    bool create_clamped_uniform(int dimension, int order0, int order1, int cv_count0, int cv_count1, double nurbsknot_delta0 = 1.0, double nurbsknot_delta1 = 1.0);

    /// Clear all data; is_valid() is false afterwards
    void destroy();

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// Orders >= 2, cv_count >= order, nurbsknot vectors of the right length and non-decreasing, CV array large enough
    bool is_valid() const;

    /// Nurbsknot vector in dir has the right length and is non-decreasing
    bool is_valid_nurbsknot_vector(int dir) const;

    bool is_rational() const { return m_is_rat != 0; }

    /// First and last CV rows across dir coincide when clamped, else periodic
    bool is_closed(int dir) const;

    /// Uniform nurbsknot spacing in dir and the first degree CV rows repeat the last
    bool is_periodic(int dir) const;

    /// Every CV within tolerance of one plane, written to plane when given
    bool is_planar(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Clamped side collapsed to one point; side: 0 south (v0), 1 east (u1), 2 north (v1), 3 west (u0)
    bool is_singular(int side) const;

    /// Full end multiplicity in dir; end: 0 start, 1 end, 2 both
    bool is_clamped(int dir, int end = 2) const;

    /// Same layout, CVs and weights within tolerance; nurbsknots too unless ignore_parameterization
    bool is_duplicate(const NurbsSurface& other, bool ignore_parameterization, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Attributes
    // ═══════════════════════════════════════════════════════════════════════════

    int dimension() const { return m_dim; }
    int order(int dir) const;
    int degree(int dir) const;
    int cv_count(int dir) const;
    /// cv_count(0) * cv_count(1)
    int cv_count() const;
    /// Doubles per CV: dim + 1 when rational
    int cv_size() const;
    /// order + cv_count - 2
    int nurbsknot_count(int dir) const;
    /// cv_count - order + 1
    int span_count(int dir) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Control vertex access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Pointer to CV[i][j], cv_size() doubles (x*w, y*w, z*w, w when rational), null when out of range
    double* cv(int i, int j);
    const double* cv(int i, int j) const;
    /// Euclidean CV (divided by weight when rational), origin when out of range
    Point get_cv(int i, int j) const;
    /// Homogeneous CV (x, y, z, w), w = 1 when non-rational
    bool get_cv_4d(int i, int j, double& x, double& y, double& z, double& w) const;
    /// Set the Euclidean CV, keeping its weight
    bool set_cv(int i, int j, const Point& point);
    /// Set the homogeneous CV; w ignored when non-rational
    bool set_cv_4d(int i, int j, double x, double y, double z, double w);
    double weight(int i, int j) const;
    /// Rescale the homogeneous CV to the new weight so the Euclidean point stays; false when non-rational
    bool set_weight(int i, int j, double weight);

    // ═══════════════════════════════════════════════════════════════════════════
    // NurbsKnot access
    // ═══════════════════════════════════════════════════════════════════════════

    double nurbsknot(int dir, int nurbsknot_index) const;
    bool set_nurbsknot(int dir, int nurbsknot_index, double nurbsknot_value);
    int nurbsknot_multiplicity(int dir, int nurbsknot_index) const;
    std::vector<double> get_nurbsknots(int dir) const;
    /// Insert a nurbsknot with the given multiplicity in dir without changing the shape
    bool insert_nurbsknot(int dir, double nurbsknot_value, int nurbsknot_multiplicity = 1);

    // ═══════════════════════════════════════════════════════════════════════════
    // Domain
    // ═══════════════════════════════════════════════════════════════════════════

    /// [nurbsknot[order - 2], nurbsknot[cv_count - 1]] in dir
    std::pair<double, double> domain(int dir) const;
    /// Linearly remap the nurbsknots in dir onto [t0, t1]
    bool set_domain(int dir, double t0, double t1);
    /// Distinct nurbsknot values inside the domain of dir
    std::vector<double> get_span_vector(int dir) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Division
    // ═══════════════════════════════════════════════════════════════════════════

    /// Points, normals and (u, v) on a (nu + 1) x (nv + 1) grid over the domain
    std::tuple<std::vector<std::vector<Point>>, std::vector<std::vector<Vector>>, std::vector<std::vector<std::pair<double, double>>>> divide_by_count_points(int nu, int nv) const;

    /// Frames (x = dS/du, y = dS/dv) and (u, v) on a (nu + 1) x (nv + 1) grid over the domain
    std::pair<std::vector<std::vector<Plane>>, std::vector<std::vector<std::pair<double, double>>>> divide_by_count_planes(int nu, int nv) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════

    /// S(u, v) by the tensor-product basis; origin when invalid
    Point point_at(double u, double v) const;

    /// S(u, v) into three doubles
    void point_at(double u, double v, double& px, double& py, double& pz) const {
        const Point p = point_at(u, v);
        px = p[0];
        py = p[1];
        pz = p[2];
    }

    /// (u, v) of the closest surface point (grid seed + Newton)
    std::pair<double, double> closest_parameters(const Point& test_point) const;
    Point closest_point(const Point& test_point) const;

    /// K = (LN - M^2) / (EG - F^2)
    double gaussian_curvature(double u, double v) const;
    /// H = (EN - 2FM + GL) / (2(EG - F^2)), sign following Su x Sv
    double mean_curvature(double u, double v) const;

    /// Unit normal dS/dv x dS/du, z-axis at singular points
    Vector normal_at(double u, double v) const;

    /// Frame at (u, v): origin S, x-axis dS/du, y-axis dS/dv
    Plane frame_at(double u, double v) const;

    /// Points where the infinite line pierces the surface (grid seed + Newton)
    std::vector<Point> intersections_with_line(const Line& line) const;

    /// Point and partials up to num_derivs (max 2) in (k, l) loop order: [S, Sv, Svv, Su, Suv, Suu]
    std::vector<Vector> evaluate(double u, double v, int num_derivs = 0) const;

    /// Corner CV; u_end and v_end are 0 or 1
    Point point_at_corner(int u_end, int v_end) const;

    /// Iso-curve varying along dir at the other parameter c; rational surfaces give their exact rational curve
    NurbsCurve iso_curve(int dir, double c) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Modification
    // ═══════════════════════════════════════════════════════════════════════════

    /// Flip the parameterization in dir
    bool reverse(int dir);
    /// Swap u and v
    bool transpose();
    /// Swap two coordinate axes in every CV
    bool swap_coordinates(int axis_i, int axis_j);
    /// Restrict dir to the sub-domain
    bool trim(int dir, const std::pair<double, double>& domain);
    /// Two surfaces split at c in dir; both invalid when c is outside the domain
    std::pair<NurbsSurface, NurbsSurface> split(int dir, double c) const;
    /// Add weights of 1
    bool make_rational();
    /// Drop weights, dividing each CV by its own
    bool make_non_rational();
    /// Elevate the degree in dir without changing the shape
    bool increase_degree(int dir, int desired_degree);

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    bool transform(const Xform& xform);
    NurbsSurface transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Splitting
    // ═══════════════════════════════════════════════════════════════════════════

    /// Trimmed faces on each side of the plane
    std::vector<NurbsSurfaceTrimmed> split_by_plane(const Plane& plane, double tolerance = 0.0) const;
    /// Trimmed faces cut by curves pulled onto the surface; off-surface curves are skipped
    std::vector<NurbsSurfaceTrimmed> split_by_curves(const std::vector<NurbsCurve>& curves, double tolerance = 0.0) const;
    /// Trimmed faces cut by a line pulled onto the surface
    std::vector<NurbsSurfaceTrimmed> split_by_line(const Line& line, double tolerance = 0.0) const;
    /// Trimmed faces cut by the surface/surface intersection
    std::vector<NurbsSurfaceTrimmed> split_by_surface(const NurbsSurface& cutter, double tolerance = 0.0) const;
    /// Trimmed faces cut by every overlapping face of the brep
    std::vector<NurbsSurfaceTrimmed> split_by_brep(const BRep& brep, double tolerance = 0.0) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════

    /// Quadtree subdivision in UV up to depth 8; cached in m_mesh
    Mesh mesh_adaptive(double max_angle = 20.0, double max_edge_length = 0.0, double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    /// Two triangles for a planar surface, else the span grid; cached in m_mesh
    Mesh mesh() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    nlohmann::ordered_json jsondump() const;
    static NurbsSurface jsonload(const nlohmann::json& data);
    void file_json_dump(const std::string& filename) const;
    static NurbsSurface file_json_load(const std::string& filename);
    std::string file_json_dumps() const;
    static NurbsSurface file_json_loads(const std::string& json_string);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    std::string pb_dumps() const;
    static NurbsSurface pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static NurbsSurface pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "NurbsSurface(name=..., degree=(u, v), cvs=(u, v))"
    std::string str() const;
    /// Multi-line form with every control point
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const NurbsSurface& surface);

private:
    mutable std::string _guid;

    // ═══════════════════════════════════════════════════════════════════════════
    // Private helpers
    // ═══════════════════════════════════════════════════════════════════════════

    /// Copy every field but the guid
    void deep_copy_from(const NurbsSurface& src);
    bool zero_cvs();
    bool make_clamped_uniform_nurbsknot_vector(int dir, double delta = 1.0);
    bool make_periodic_uniform_nurbsknot_vector(int dir, double delta = 1.0);
    /// Euclidean point of a homogeneous CV or blend
    Point dehomogenize(const double* h) const;
    /// Span index in dir containing t
    int find_span(int dir, double t) const;
    /// Basis derivatives ders[k][j] of the order functions on the span (Piegl & Tiller A2.3)
    std::vector<std::vector<double>> basis_functions_derivatives(int dir, int span, double t, int deriv_order) const;
    /// Rational quotient rule on homogeneous partials in (k, l) loop order (Piegl & Tiller A4.4)
    std::vector<Vector> rational_derivatives(const std::vector<std::vector<double>>& skl, int num_derivs) const;
    /// Newton on (n1, n2) . (S - p0) = 0 from (u, v); false when it leaves the domain or stalls
    bool line_newton(double& u, double& v, const Point& p0, const Vector& n1, const Vector& n2) const;
    /// First and second fundamental forms at (u, v); false at a singular point
    bool fundamental_forms(double u, double v, double& E, double& F, double& G, double& L, double& M, double& N) const;
    /// Two triangles through the four corners with one shared normal
    Mesh mesh_planar() const;
    /// Pack the CV rows across dir into one curve along dir with cv_size * cv_count(1 - dir) doubles per CV
    NurbsCurve to_curve(int dir) const;
    /// Unpack a curve made by to_curve back into this surface along dir
    bool from_curve(const NurbsCurve& crv, int dir);
};

} // namespace session_cpp
