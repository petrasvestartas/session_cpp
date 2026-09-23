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

namespace session_proto {
class NurbsSurface;
}

namespace session_cpp {

class Line;
class BRep;
class NurbsSurfaceTrimmed;

/// A NURBS surface: OpenNURBS layout, nurbsknot count = order + cv_count - 2 per direction, homogeneous row-major CVs when rational.
class NurbsSurface {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_nurbssurface"; // Surface name.
    double width = 1.0; // Display width.
    std::vector<Color> pointcolors; // Display color per control point.
    std::vector<Color> facecolors; // Display color per mesh face.
    std::vector<Color> linecolors; // Display color per control polygon segment.
    int m_dim; // Coordinate dimension.
    int m_is_rat; // 1 when rational, 0 otherwise.
    int m_order[2]; // Degree + 1 per direction.
    int m_cv_count[2]; // Number of control vertices per direction.
    int m_cv_stride[2]; // Doubles between consecutive CVs per direction.
    std::vector<double> m_nurbsknot[2]; // NurbsKnot vector per direction, order + cv_count - 2 values.
    std::vector<double> m_cv; // Flat CV array, homogeneous when rational.
    mutable Mesh m_mesh; // Cached mesh from mesh() or mesh_adaptive().

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty surface.
    NurbsSurface();

    /// Construct an unset surface with the given layout.
    NurbsSurface(int dimension, bool is_rational, int order0, int order1, int cv_count0, int cv_count1);

    /// Copy with a new guid and the same data.
    NurbsSurface(const NurbsSurface& other);

    /// Copy-assign with a new guid and the same data.
    NurbsSurface& operator=(const NurbsSurface& other);

    /// Move while preserving the guid.
    NurbsSurface(NurbsSurface&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    NurbsSurface& operator=(NurbsSurface&& other) noexcept = default;

    /// Destroy the surface.
    ~NurbsSurface();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a clamped or periodic uniform surface through cv_count_u x cv_count_v points in row-major order (u slowest).
    static NurbsSurface create(
        bool periodic_u,
        bool periodic_v,
        int degree_u,
        int degree_v,
        int cv_count_u,
        int cv_count_v,
        const std::vector<Point>& points
    );

    /// Construct from points[iv][iu], weights[iv][iu], distinct knots and multiplicities per direction (OCCT convention).
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
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, width, colors, layout, nurbsknots and CVs; guid ignored.
    bool operator==(const NurbsSurface& other) const;

    /// Compare name, width, colors, layout, nurbsknots and CVs; guid ignored.
    bool operator!=(const NurbsSurface& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform every CV in place.
    bool transform(const Xform& xform);

    /// Return a transformed copy.
    NurbsSurface transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Initialization
    // ═══════════════════════════════════════════════════════════════════════════
    /// Reset every field to the empty invalid surface.
    void initialize();

    /// Allocate nurbsknots (clamped or periodic uniform) and zeroed CVs; false when order < 2 or cv_count < order.
    bool create_raw(
        int dimension,
        bool is_rational,
        int order0,
        int order1,
        int cv_count0,
        int cv_count1,
        bool is_periodic_u = false,
        bool is_periodic_v = false,
        double nurbsknot_delta_u = 1.0,
        double nurbsknot_delta_v = 1.0
    );

    /// Allocate a non-rational surface with clamped uniform nurbsknots of the given spacing.
    bool create_clamped_uniform(
        int dimension,
        int order0,
        int order1,
        int cv_count0,
        int cv_count1,
        double nurbsknot_delta0 = 1.0,
        double nurbsknot_delta1 = 1.0
    );

    /// Clear all data; is_valid() is false afterwards.
    void destroy();

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether orders >= 2, cv_count >= order, nurbsknot vectors are valid and the CV array is large enough.
    bool is_valid() const;

    /// Return whether the nurbsknot vector in dir has the right length and is non-decreasing.
    bool is_valid_nurbsknot_vector(int dir) const;

    /// Return whether the CVs carry weights.
    bool is_rational() const { return m_is_rat != 0; }

    /// Return whether the first and last CV rows across dir coincide when clamped, else whether dir is periodic.
    bool is_closed(int dir) const;

    /// Return whether dir has uniform nurbsknot spacing and the first degree CV rows repeat the last.
    bool is_periodic(int dir) const;

    /// Return whether every CV is within tolerance of one plane, written to plane when given.
    bool is_planar(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Return whether a clamped side collapses to one point; side: 0 south (v0), 1 east (u1), 2 north (v1), 3 west (u0).
    bool is_singular(int side) const;

    /// Return whether dir has full end multiplicity; end: 0 start, 1 end, 2 both.
    bool is_clamped(int dir, int end = 2) const;

    /// Return whether layout, CVs and weights match within tolerance; nurbsknots too unless ignore_parameterization.
    bool is_duplicate(
        const NurbsSurface& other,
        bool ignore_parameterization,
        double tolerance = Tolerance::ZERO_TOLERANCE
    ) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid() { _guid.clear(); }

    /// Return the coordinate dimension.
    int dimension() const { return m_dim; }

    /// Return the order (degree + 1) in dir.
    int order(int dir) const;

    /// Return the degree in dir.
    int degree(int dir) const;

    /// Return the number of control vertices in dir.
    int cv_count(int dir) const;

    /// Return cv_count(0) * cv_count(1).
    int cv_count() const;

    /// Return the doubles per CV: dimension + 1 when rational.
    int cv_size() const;

    /// Return order + cv_count - 2 in dir.
    int nurbsknot_count(int dir) const;

    /// Return cv_count - order + 1 in dir.
    int span_count(int dir) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Control vertex access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the mutable pointer to CV[i][j], cv_size() doubles (x*w, y*w, z*w, w when rational), nullptr when out of range.
    double* cv(int i, int j);

    /// Return the pointer to CV[i][j], nullptr when out of range.
    const double* cv(int i, int j) const;

    /// Return the Euclidean CV (divided by weight when rational), origin when out of range.
    Point get_cv(int i, int j) const;

    /// Return the homogeneous CV (x, y, z, w), w = 1 when non-rational.
    bool get_cv_4d(int i, int j, double& x, double& y, double& z, double& w) const;

    /// Set the Euclidean CV, keeping its weight.
    bool set_cv(int i, int j, const Point& point);

    /// Set the homogeneous CV; w ignored when non-rational.
    bool set_cv_4d(int i, int j, double x, double y, double z, double w);

    /// Return the weight of CV[i][j], 1 when non-rational.
    double weight(int i, int j) const;

    /// Rescale the homogeneous CV to the new weight so the Euclidean point stays; false when non-rational.
    bool set_weight(int i, int j, double weight);

    // ═══════════════════════════════════════════════════════════════════════════
    // NurbsKnot access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the nurbsknot at nurbsknot_index in dir.
    double nurbsknot(int dir, int nurbsknot_index) const;

    /// Set the nurbsknot at nurbsknot_index in dir.
    bool set_nurbsknot(int dir, int nurbsknot_index, double nurbsknot_value);

    /// Return the multiplicity of the nurbsknot at nurbsknot_index in dir.
    int nurbsknot_multiplicity(int dir, int nurbsknot_index) const;

    /// Return a copy of the nurbsknot vector in dir.
    std::vector<double> get_nurbsknots(int dir) const;

    /// Insert a nurbsknot with the given multiplicity in dir without changing the shape.
    bool insert_nurbsknot(int dir, double nurbsknot_value, int nurbsknot_multiplicity = 1);

    // ═══════════════════════════════════════════════════════════════════════════
    // Domain
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return [nurbsknot[order - 2], nurbsknot[cv_count - 1]] in dir.
    std::pair<double, double> domain(int dir) const;

    /// Linearly remap the nurbsknots in dir onto [t0, t1].
    bool set_domain(int dir, double t0, double t1);

    /// Return the distinct nurbsknot values inside the domain of dir.
    std::vector<double> get_span_vector(int dir) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Division
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return points, normals and (u, v) on a (nu + 1) x (nv + 1) grid over the domain.
    std::tuple<
        std::vector<std::vector<Point>>,
        std::vector<std::vector<Vector>>,
        std::vector<std::vector<std::pair<double, double>>>>
    divide_by_count_points(int nu, int nv) const;

    /// Return frames (x = dS/du, y = dS/dv) and (u, v) on a (nu + 1) x (nv + 1) grid over the domain.
    std::pair<std::vector<std::vector<Plane>>, std::vector<std::vector<std::pair<double, double>>>>
    divide_by_count_planes(int nu, int nv) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return S(u, v) by the tensor-product basis, origin when invalid.
    Point point_at(double u, double v) const;

    /// Write S(u, v) into three doubles.
    void point_at(double u, double v, double& px, double& py, double& pz) const {

        const Point p = point_at(u, v);
        px = p[0];
        py = p[1];
        pz = p[2];
    }

    /// Return (u, v) of the closest surface point (grid seed + Newton).
    std::pair<double, double> closest_parameters(const Point& test_point) const;

    /// Return the closest surface point to test_point.
    Point closest_point(const Point& test_point) const;

    /// Return K = (LN - M^2) / (EG - F^2).
    double gaussian_curvature(double u, double v) const;

    /// Return H = (EN - 2FM + GL) / (2(EG - F^2)), sign following Su x Sv.
    double mean_curvature(double u, double v) const;

    /// Return the unit normal dS/dv x dS/du, z-axis at singular points.
    Vector normal_at(double u, double v) const;

    /// Return the frame at (u, v): origin S, x-axis dS/du, y-axis dS/dv.
    Plane frame_at(double u, double v) const;

    /// Return the points where the infinite line pierces the surface (grid seed + Newton).
    std::vector<Point> intersections_with_line(const Line& line) const;

    /// Return the point and partials up to num_derivs (max 2) in (k, l) loop order: [S, Sv, Svv, Su, Suv, Suu].
    std::vector<Vector> evaluate(double u, double v, int num_derivs = 0) const;

    /// Return the corner CV; u_end and v_end are 0 or 1.
    Point point_at_corner(int u_end, int v_end) const;

    /// Return the iso-curve along dir at the other parameter c; rational surfaces give their exact rational curve.
    NurbsCurve iso_curve(int dir, double c) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Modifications
    // ═══════════════════════════════════════════════════════════════════════════
    /// Flip the parameterization in dir.
    bool reverse(int dir);

    /// Swap u and v.
    bool transpose();

    /// Swap two coordinate axes in every CV.
    bool swap_coordinates(int axis_i, int axis_j);

    /// Restrict dir to the sub-domain.
    bool trim(int dir, const std::pair<double, double>& domain);

    /// Return two surfaces split at c in dir; both invalid when c is outside the domain.
    std::pair<NurbsSurface, NurbsSurface> split(int dir, double c) const;

    /// Add weights of 1.
    bool make_rational();

    /// Drop weights, dividing each CV by its own.
    bool make_non_rational();

    /// Elevate the degree in dir without changing the shape.
    bool increase_degree(int dir, int desired_degree);

    // ═══════════════════════════════════════════════════════════════════════════
    // Splitting
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the trimmed faces on each side of the plane.
    std::vector<NurbsSurfaceTrimmed> split_by_plane(const Plane& plane, double tolerance = 0.0) const;

    /// Return the trimmed faces cut by curves pulled onto the surface; off-surface curves are skipped.
    std::vector<NurbsSurfaceTrimmed> split_by_curves(
        const std::vector<NurbsCurve>& curves,
        double tolerance = 0.0
    ) const;

    /// Return the trimmed faces cut by a line pulled onto the surface.
    std::vector<NurbsSurfaceTrimmed> split_by_line(const Line& line, double tolerance = 0.0) const;

    /// Return the trimmed faces cut by the surface/surface intersection.
    std::vector<NurbsSurfaceTrimmed> split_by_surface(const NurbsSurface& cutter, double tolerance = 0.0) const;

    /// Return the trimmed faces cut by every overlapping face of the brep.
    std::vector<NurbsSurfaceTrimmed> split_by_brep(const BRep& brep, double tolerance = 0.0) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the quadtree subdivision in UV up to depth 8, cached in m_mesh.
    Mesh mesh_adaptive(
        double max_angle = 20.0,
        double max_edge_length = 0.0,
        double min_edge_length = 0.0,
        double max_chord_height = 0.0
    ) const;

    /// Return two triangles for a planar surface, else the span grid, cached in m_mesh.
    Mesh mesh() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static NurbsSurface jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static NurbsSurface file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static NurbsSurface file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::NurbsSurface to_proto() const;

    /// Construct from the protobuf message.
    static NurbsSurface from_proto(const session_proto::NurbsSurface& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static NurbsSurface pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static NurbsSurface pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "NurbsSurface(name=..., degree=(u, v), cvs=(u, v))".
    std::string str() const;

    /// Return the multi-line form with every control point.
    std::string repr() const;

    /// Stream the str() form.
    friend std::ostream& operator<<(std::ostream& os, const NurbsSurface& surface);

private:
    /// Copy every field but the guid.
    void deep_copy_from(const NurbsSurface& src);

    /// Zero every CV, false when the layout is unset.
    bool zero_cvs();

    /// Fill the nurbsknot vector in dir with clamped uniform values of the given spacing.
    bool make_clamped_uniform_nurbsknot_vector(int dir, double delta = 1.0);

    /// Fill the nurbsknot vector in dir with periodic uniform values of the given spacing.
    bool make_periodic_uniform_nurbsknot_vector(int dir, double delta = 1.0);

    /// Return the Euclidean point of a homogeneous CV or blend.
    Point dehomogenize(const double* h) const;

    /// Return the span index in dir containing t.
    int find_span(int dir, double t) const;

    /// Return the basis derivatives ders[k][j] of the order functions on the span (Piegl & Tiller A2.3).
    std::vector<std::vector<double>> basis_functions_derivatives(int dir, int span, double t, int deriv_order) const;

    /// Apply the rational quotient rule to homogeneous partials in (k, l) loop order (Piegl & Tiller A4.4).
    std::vector<Vector> rational_derivatives(const std::vector<std::vector<double>>& skl, int num_derivs) const;

    /// Run Newton on (n1, n2) . (S - p0) = 0 from (u, v); false when it leaves the domain or stalls.
    bool line_newton(double& u, double& v, const Point& p0, const Vector& n1, const Vector& n2) const;

    /// Compute the first and second fundamental forms at (u, v); false at a singular point.
    bool fundamental_forms(double u, double v, double& E, double& F, double& G, double& L, double& M, double& N) const;

    /// Return two triangles through the four corners with one shared normal.
    Mesh mesh_planar() const;

    /// Pack the CV rows across dir into one curve along dir with cv_size * cv_count(1 - dir) doubles per CV.
    NurbsCurve to_curve(int dir) const;

    /// Unpack a curve made by to_curve back into this surface along dir.
    bool from_curve(const NurbsCurve& crv, int dir);
};

} // namespace session_cpp
