#include "primitives.h"
#include "plane.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>

namespace session_cpp {

namespace {

// ═══════════════════════════════════════════════════════════════════════════
// Rational quadratic circle pattern
// ═══════════════════════════════════════════════════════════════════════════

const double CIRCLE_W = 0.7071067811865476;
const double CIRCLE_X[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
const double CIRCLE_Y[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
const double CIRCLE_WEIGHTS[9] = {1, CIRCLE_W, 1, CIRCLE_W, 1, CIRCLE_W, 1, CIRCLE_W, 1};
const double CIRCLE_NURBSKNOTS[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};

/// Row j of a surface set to a circle in the plane z = cz, weights scaled by weight.
void set_circle_row(NurbsSurface& srf, int j, double cx, double cy, double cz, double radius, double weight) {

    for (int i = 0; i < 9; i++) {
        const double w = CIRCLE_WEIGHTS[i] * weight;
        const double px = cx + radius * CIRCLE_X[i];
        const double py = cy + radius * CIRCLE_Y[i];
        srf.set_cv_4d(i, j, px * w, py * w, cz * w, w);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Appends n points of a circle of the given radius in the plane z.
void add_ring(std::vector<Point>& vertices, size_t n, double radius, double z) {

    for (size_t i = 0; i < n; i++) {
        const double angle = 2.0 * Tolerance::PI * i / n;
        vertices.push_back(Point(radius * std::cos(angle), radius * std::sin(angle), z));
    }
}

/// Face without consecutive duplicate vertices.
std::vector<size_t> dedup_face(const std::vector<size_t>& face) {

    std::vector<size_t> unique;

    for (size_t k = 0; k < face.size(); k++)
        if (face[k] != face[(k + 1) % face.size()])
            unique.push_back(face[k]);

    return unique;
}

/// Vertex keys of the surface sampled on a (u_count + 1) x (v_count + 1) grid; seam and poles share keys.
std::vector<std::vector<size_t>> surface_grid(const NurbsSurface& surface, int u_count, int v_count, Mesh& mesh) {

    const auto [u0, u1] = surface.domain(0);
    const auto [v0, v1] = surface.domain(1);
    const bool closed_u = surface.is_closed(0);
    const bool singular_south = surface.is_singular(0);
    const bool singular_north = surface.is_singular(2);
    std::vector<std::vector<size_t>> grid(u_count + 1, std::vector<size_t>(v_count + 1, 0));

    for (int i = 0; i <= u_count; i++) {
        const double u = u0 + (u1 - u0) * i / u_count;

        for (int j = 0; j <= v_count; j++) {
            const double v = v0 + (v1 - v0) * j / v_count;

            if (closed_u && i == u_count)
                grid[i][j] = grid[0][j];
            else if (singular_south && j == 0 && i > 0)
                grid[i][j] = grid[0][0];
            else if (singular_north && j == v_count && i > 0)
                grid[i][j] = grid[0][v_count];
            else
                grid[i][j] = mesh.add_vertex(surface.point_at(u, v));
        }
    }

    return grid;
}

/// Vertex keys of the surface sampled at v offset by t cells on a (u_count + 1) x v_count grid; seam shares keys.
std::vector<std::vector<size_t>> surface_mid_grid(
    const NurbsSurface& surface,
    int u_count,
    int v_count,
    double t,
    Mesh& mesh
) {

    const auto [u0, u1] = surface.domain(0);
    const auto [v0, v1] = surface.domain(1);
    const bool closed_u = surface.is_closed(0);
    std::vector<std::vector<size_t>> grid(u_count + 1, std::vector<size_t>(v_count, 0));

    for (int i = 0; i <= u_count; i++) {
        const double u = u0 + (u1 - u0) * i / u_count;

        for (int j = 0; j < v_count; j++) {
            const double v = v0 + (v1 - v0) * (j + t) / v_count;

            if (closed_u && i == u_count)
                grid[i][j] = grid[0][j];
            else
                grid[i][j] = mesh.add_vertex(surface.point_at(u, v));
        }
    }

    return grid;
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve compatibility
// ═══════════════════════════════════════════════════════════════════════════

/// Sorted union of two nurbsknot vectors, equal values kept once.
std::vector<double> merge_nurbsknot_vectors(const std::vector<double>& a, const std::vector<double>& b) {

    const double tol = 1e-10;
    std::vector<double> merged;
    size_t i = 0;
    size_t j = 0;

    while (i < a.size() && j < b.size()) {
        if (std::abs(a[i] - b[j]) < tol) {
            merged.push_back(a[i]);
            i++;
            j++;
        } else if (a[i] < b[j]) {
            merged.push_back(a[i]);
            i++;
        } else {
            merged.push_back(b[j]);
            j++;
        }
    }

    for (; i < a.size(); i++)
        merged.push_back(a[i]);

    for (; j < b.size(); j++)
        merged.push_back(b[j]);

    return merged;
}

/// True when both nurbsknot vectors match within 1e-10.
bool nurbsknot_vectors_equal(const std::vector<double>& a, const std::vector<double>& b) {

    const double tol = 1e-10;

    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); i++)
        if (std::abs(a[i] - b[i]) > tol)
            return false;

    return true;
}

/// Same degree, rationality, domain [0, 1] and nurbsknot vector for every curve.
void make_curves_compatible(std::vector<NurbsCurve>& curves) {

    if (curves.size() < 2)
        return;

    int max_degree = 0;
    bool any_rational = false;

    for (const NurbsCurve& c : curves) {
        max_degree = std::max(max_degree, c.degree());
        any_rational = any_rational || c.is_rational();
    }

    for (NurbsCurve& c : curves) {
        if (c.degree() < max_degree)
            c.increase_degree(max_degree);

        if (any_rational)
            c.make_rational();
    }

    bool compatible = true;

    for (size_t i = 1; i < curves.size(); i++)
        if (curves[i].cv_count() != curves[0].cv_count() ||
            !nurbsknot_vectors_equal(curves[i].get_nurbsknots(), curves[0].get_nurbsknots()))
            compatible = false;

    if (compatible)
        return;

    for (NurbsCurve& c : curves)
        c.set_domain(0.0, 1.0);

    std::vector<double> unified = curves[0].get_nurbsknots();

    for (size_t i = 1; i < curves.size(); i++)
        unified = merge_nurbsknot_vectors(unified, curves[i].get_nurbsknots());

    const double tol = 1e-10;

    for (NurbsCurve& c : curves) {
        const std::vector<double> nurbsknots = c.get_nurbsknots();
        size_t ci = 0;

        for (size_t ui = 0; ui < unified.size(); ui++)
            if (ci < nurbsknots.size() && std::abs(nurbsknots[ci] - unified[ui]) < tol)
                ci++;
            else
                c.insert_nurbsknot(unified[ui], 1);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Planar helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Bilinear patch: u runs p00 to p10, v runs p00 to p01.
NurbsSurface bilinear_patch(const Point& p00, const Point& p10, const Point& p01, const Point& p11) {

    NurbsSurface srf(3, false, 2, 2, 2, 2);
    srf.set_cv(0, 0, p00);
    srf.set_cv(1, 0, p10);
    srf.set_cv(0, 1, p01);
    srf.set_cv(1, 1, p11);

    return srf;
}

/// Unit direction of the longest edge of a closed polygon.
Vector longest_edge_dir(const std::vector<Point>& pts) {

    Vector best(0.0, 0.0, 0.0);

    for (size_t i = 0; i < pts.size(); i++) {
        const Vector edge = pts[(i + 1) % pts.size()] - pts[i];

        if (edge.magnitude() > best.magnitude())
            best = edge;
    }

    return best.normalized();
}

/// Bilinear patch in the frame covering the points with a 5% margin.
NurbsSurface bounded_patch(
    const std::vector<Point>& pts,
    const Point& origin,
    const Vector& x_axis,
    const Vector& y_axis
) {

    double min_u = 1e30;
    double max_u = -1e30;
    double min_v = 1e30;
    double max_v = -1e30;

    for (const Point& pt : pts) {
        const Vector d = pt - origin;
        min_u = std::min(min_u, d.dot(x_axis));
        max_u = std::max(max_u, d.dot(x_axis));
        min_v = std::min(min_v, d.dot(y_axis));
        max_v = std::max(max_v, d.dot(y_axis));
    }

    double pad = std::max(max_u - min_u, max_v - min_v) * 0.05;

    if (pad < 1e-6)
        pad = 1.0;

    min_u -= pad;
    max_u += pad;
    min_v -= pad;
    max_v += pad;

    return bilinear_patch(
        origin + x_axis * min_u + y_axis * min_v,
        origin + x_axis * max_u + y_axis * min_v,
        origin + x_axis * min_u + y_axis * max_v,
        origin + x_axis * max_u + y_axis * max_v
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Loft helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Section parameters in [0, 1] from the mean CV distance between consecutive sections.
std::vector<double> loft_section_params(const std::vector<NurbsCurve>& curves) {

    const int n = static_cast<int>(curves.size());
    const int cv_count = curves[0].cv_count();
    std::vector<double> v_params(n, 0.0);

    for (int k = 1; k < n; k++) {
        double sum = 0.0;

        for (int i = 0; i < cv_count; i++)
            sum += curves[k - 1].get_cv(i).distance(curves[k].get_cv(i));

        v_params[k] = v_params[k - 1] + sum / cv_count;
    }

    const double total = v_params[n - 1];

    for (int k = 0; k < n; k++)
        v_params[k] = total > 1e-14 ? v_params[k] / total : static_cast<double>(k) / (n - 1);

    return v_params;
}

/// Clamped nurbsknot vector averaging the section parameters.
std::vector<double> loft_nurbsknots(const std::vector<double>& v_params, int order_v) {

    const int n = static_cast<int>(v_params.size());
    const int degree_v = order_v - 1;
    std::vector<double> nurbsknots(order_v + n - 2, v_params[0]);

    for (int j = 1; j <= n - order_v; j++) {
        double sum = 0.0;

        for (int i = j; i < j + degree_v; i++)
            sum += v_params[i];

        nurbsknots[degree_v - 1 + j] = sum / degree_v;
    }

    for (int i = n - 1; i < order_v + n - 2; i++)
        nurbsknots[i] = v_params[n - 1];

    return nurbsknots;
}

/// Row of the collocation matrix: the cv_count basis values at t.
std::vector<double> loft_basis_row(const std::vector<double>& nurbsknots, int order, int cv_count, double t) {

    std::vector<double> row(cv_count, 0.0);
    const int span = nurbsknot::find_span(order, cv_count, nurbsknots, t);
    const int base = span + order - 1;

    if (nurbsknots[base - 1] == nurbsknots[base]) {
        row[t <= nurbsknots[base] ? span : span + order - 1] = 1.0;

        return row;
    }

    const std::vector<double> basis = nurbsknot::eval_basis(order, nurbsknots, span, t);

    for (int j = 0; j < order && span + j < cv_count; j++)
        row[span + j] = basis[j];

    return row;
}

/// Solves a x = b by Gaussian elimination with partial pivoting, one right-hand side per column of b.
std::vector<std::vector<double>> solve_linear(std::vector<std::vector<double>> a, std::vector<std::vector<double>> b) {

    const int n = static_cast<int>(a.size());
    const int dim = static_cast<int>(b[0].size());

    for (int col = 0; col < n; col++) {
        int max_row = col;

        for (int row = col + 1; row < n; row++)
            if (std::abs(a[row][col]) > std::abs(a[max_row][col]))
                max_row = row;

        if (std::abs(a[max_row][col]) < 1e-14)
            continue;

        std::swap(a[col], a[max_row]);
        std::swap(b[col], b[max_row]);

        for (int row = col + 1; row < n; row++) {
            const double factor = a[row][col] / a[col][col];

            for (int c = col; c < n; c++)
                a[row][c] -= factor * a[col][c];

            for (int d = 0; d < dim; d++)
                b[row][d] -= factor * b[col][d];
        }
    }

    std::vector<std::vector<double>> x(n, std::vector<double>(dim, 0.0));

    for (int row = n - 1; row >= 0; row--) {
        for (int d = 0; d < dim; d++) {
            x[row][d] = b[row][d];

            for (int c = row + 1; c < n; c++)
                x[row][d] -= a[row][c] * x[c][d];

            if (std::abs(a[row][row]) > 1e-14)
                x[row][d] /= a[row][row];
        }
    }

    return x;
}

// ═══════════════════════════════════════════════════════════════════════════
// Sweep helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Point at fraction s from a to b.
Point lerp_point(const Point& a, const Point& b, double s) {
    return a + (b - a) * s;
}

/// Vector at fraction s from a to b.
Vector lerp_vector(const Vector& a, const Vector& b, double s) {
    return a + (b - a) * s;
}

/// World to the profile frame: centroid origin, x toward the start point, z the profile normal.
Xform profile_to_xy(const NurbsCurve& profile) {

    Vector centroid(0.0, 0.0, 0.0);

    for (int i = 0; i < profile.cv_count(); i++)
        centroid += profile.get_cv(i) - Point(0.0, 0.0, 0.0);

    const Point origin = Point(0.0, 0.0, 0.0) + centroid / profile.cv_count();
    const auto [t0, t1] = profile.domain();
    const Point pa = profile.point_at(t0);
    const Point pb = profile.point_at(t0 + (t1 - t0) / 3.0);
    const Point pc = profile.point_at(t0 + 2.0 * (t1 - t0) / 3.0);
    Vector normal = (pb - pa).cross(pc - pa);

    if (!normal.normalize_self())
        normal = Vector(1.0, 0.0, 0.0);

    Vector x_axis = pa - origin;

    if (!x_axis.normalize_self())
        x_axis = Vector(0.0, 1.0, 0.0);

    x_axis -= normal * x_axis.dot(normal);

    if (!x_axis.normalize_self())
        x_axis = Vector(0.0, 1.0, 0.0);

    return Xform::world_to_frame(origin, x_axis, normal.cross(x_axis), normal);
}

/// Frame of a sweep shape: start point origin, x along the chord, z across it.
Plane shape_plane(const NurbsCurve& shape) {

    const Point start = shape.point_at_start();
    Vector dir = shape.point_at_end() - start;

    if (!dir.normalize_self())
        dir = Vector(1.0, 0.0, 0.0);

    Vector side = dir.cross(Vector(0.0, 0.0, 1.0));

    if (side.magnitude() < 1e-10)
        side = dir.cross(Vector(0.0, 1.0, 0.0));

    return Plane(start, dir, side.cross(dir));
}

/// Chord length of a shape, 1 when degenerate.
double shape_width(const NurbsCurve& shape) {
    const double width = shape.point_at_start().distance(shape.point_at_end());

    return width < 1e-14 ? 1.0 : width;
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Curves ordered head to tail, reversed where needed; empty when they do not close a loop.
std::vector<NurbsCurve> chain_curves(const std::vector<NurbsCurve>& input) {

    const double tol = 1e-6;
    std::vector<NurbsCurve> loop = {input[0]};
    std::vector<bool> used(input.size(), false);
    used[0] = true;

    for (size_t step = 1; step < input.size(); step++) {
        const Point tail = loop.back().point_at_end();
        bool found = false;

        for (size_t i = 0; i < input.size() && !found; i++) {
            if (used[i])
                continue;

            NurbsCurve next = input[i];

            if (next.point_at_start().distance(tail) >= tol && next.point_at_end().distance(tail) < tol)
                next.reverse();

            if (next.point_at_start().distance(tail) >= tol)
                continue;

            loop.push_back(next);
            used[i] = true;
            found = true;
        }

        if (!found)
            return {};
    }

    if (loop.back().point_at_end().distance(loop[0].point_at_start()) > tol)
        return {};

    return loop;
}

/// Greville abcissae mapped to [0, 1].
std::vector<double> normalized_greville(const NurbsCurve& curve) {

    std::vector<double> grev = curve.get_greville_abcissae();
    const auto [t0, t1] = curve.domain();

    for (double& g : grev)
        g = t1 > t0 ? (g - t0) / (t1 - t0) : 0.0;

    return grev;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Mesh primitives
// ═══════════════════════════════════════════════════════════════════════════

Mesh Primitives::arrow_mesh(const Line& line, double radius) {

    const Point start = line.start();
    const Vector axis = line.to_vector();
    const double length = line.length();
    const Xform body =
        line_frame(line, start + axis * 0.4) * Xform::scale_xyz(radius * 2.0, radius * 2.0, length * 0.8);

    const Xform head =
        line_frame(line, start + axis * 0.9) * Xform::scale_xyz(radius * 3.0, radius * 3.0, length * 0.2);

    Mesh mesh;
    add_geometry(mesh, unit_cylinder_geometry(), body);
    add_geometry(mesh, unit_cone_geometry(), head);

    return mesh;
}

Mesh Primitives::cylinder_mesh(const Line& line, double radius) {

    const Point start = line.start();
    const Vector axis = line.to_vector();
    const Xform xform =
        line_frame(line, start + axis * 0.5) * Xform::scale_xyz(radius * 2.0, radius * 2.0, line.length());

    Mesh mesh;
    add_geometry(mesh, unit_cylinder_geometry(), xform);

    return mesh;
}

Mesh Primitives::capsule_mesh(const Line& line, double radius) {
    Mesh mesh;
    add_geometry(mesh, capsule_geometry(line.length(), radius), line_frame(line, line.start()));

    return mesh;
}

std::vector<Mesh> Primitives::edge_pipes(const Mesh& mesh, double radius) {

    const std::vector<std::pair<size_t, size_t>> edges = mesh.edges();
    const std::vector<Color>& colors = mesh.get_linecolors();
    const size_t count = std::min(edges.size(), colors.size());
    std::vector<Mesh> pipes;

    for (size_t i = 0; i < count; i++) {
        const auto [u, v] = edges[i];
        Mesh pipe = capsule_mesh(Line::from_points(mesh.vertex.at(u).position(), mesh.vertex.at(v).position()), radius);
        pipe.set_facecolors(std::vector<Color>(pipe.number_of_faces(), colors[i]));
        pipes.push_back(pipe);
    }

    return pipes;
}

Mesh Primitives::tetrahedron(double edge) {

    const double a = edge / 2.0;
    const double h = edge * std::sqrt(2.0 / 3.0);
    const double r = edge / std::sqrt(3.0);
    const double z0 = -h / 4.0;
    const double z1 = 3.0 * h / 4.0;
    const std::vector<std::vector<Point>> faces = {
        {Point(a, -r / 2.0, z0), Point(-a, -r / 2.0, z0), Point(0.0, r, z0)},
        {Point(0.0, 0.0, z1), Point(-a, -r / 2.0, z0), Point(a, -r / 2.0, z0)},
        {Point(0.0, 0.0, z1), Point(0.0, r, z0), Point(-a, -r / 2.0, z0)},
        {Point(0.0, 0.0, z1), Point(a, -r / 2.0, z0), Point(0.0, r, z0)},
    };

    return Mesh::from_polylines(faces, 1e-10);
}

Mesh Primitives::cube(double edge) {

    const double a = edge / 2.0;
    const Point v0(-a, -a, -a);
    const Point v1(a, -a, -a);
    const Point v2(a, a, -a);
    const Point v3(-a, a, -a);
    const Point v4(-a, -a, a);
    const Point v5(a, -a, a);
    const Point v6(a, a, a);
    const Point v7(-a, a, a);
    const std::vector<std::vector<Point>> faces = {
        {v3, v2, v1, v0},
        {v4, v5, v6, v7},
        {v0, v1, v5, v4},
        {v2, v3, v7, v6},
        {v0, v4, v7, v3},
        {v1, v2, v6, v5},
    };

    return Mesh::from_polylines(faces, 1e-10);
}

Mesh Primitives::octahedron(double edge) {

    const double a = edge / std::sqrt(2.0);
    const Point px(a, 0.0, 0.0);
    const Point nx(-a, 0.0, 0.0);
    const Point py(0.0, a, 0.0);
    const Point ny(0.0, -a, 0.0);
    const Point pz(0.0, 0.0, a);
    const Point nz(0.0, 0.0, -a);
    const std::vector<std::vector<Point>> faces = {
        {pz, px, py},
        {pz, py, nx},
        {pz, nx, ny},
        {pz, ny, px},
        {nz, py, px},
        {nz, nx, py},
        {nz, ny, nx},
        {nz, px, ny},
    };

    return Mesh::from_polylines(faces, 1e-10);
}

Mesh Primitives::icosahedron(double edge) {

    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double s = edge / 2.0;
    const double sp = s * phi;
    const std::vector<Point> verts = {
        Point(-s, sp, 0.0),
        Point(s, sp, 0.0),
        Point(-s, -sp, 0.0),
        Point(s, -sp, 0.0),
        Point(0.0, -s, sp),
        Point(0.0, s, sp),
        Point(0.0, -s, -sp),
        Point(0.0, s, -sp),
        Point(sp, 0.0, -s),
        Point(sp, 0.0, s),
        Point(-sp, 0.0, -s),
        Point(-sp, 0.0, s),
    };
    const std::vector<std::array<size_t, 3>> idx = {
        {0, 11, 5},  {0, 5, 1},  {0, 1, 7},  {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
        {11, 10, 2}, {10, 7, 6}, {7, 1, 8},  {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
        {3, 8, 9},   {4, 9, 5},  {2, 4, 11}, {6, 2, 10}, {8, 6, 7},   {9, 8, 1},
    };
    std::vector<std::vector<Point>> faces;

    for (const std::array<size_t, 3>& f : idx)
        faces.push_back({verts[f[0]], verts[f[1]], verts[f[2]]});

    return Mesh::from_polylines(faces, 1e-10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve primitives
// ═══════════════════════════════════════════════════════════════════════════

NurbsCurve Primitives::circle(double cx, double cy, double cz, double radius) {
    return ellipse(cx, cy, cz, radius, radius);
}

NurbsCurve Primitives::ellipse(double cx, double cy, double cz, double major_radius, double minor_radius) {

    NurbsCurve curve(3, true, 3, 9);

    for (int i = 0; i < 10; i++)
        curve.set_nurbsknot(i, CIRCLE_NURBSKNOTS[i]);

    for (int i = 0; i < 9; i++) {
        const double w = CIRCLE_WEIGHTS[i];
        const double px = cx + major_radius * CIRCLE_X[i];
        const double py = cy + minor_radius * CIRCLE_Y[i];
        curve.set_cv_4d(i, px * w, py * w, cz * w, w);
    }

    return curve;
}

NurbsCurve Primitives::arc(const Point& start, const Point& mid, const Point& end) {

    const Vector chord = end - start;
    const Point chord_mid = start + chord * 0.5;
    const Vector sagitta = mid - chord_mid;

    if (chord.cross(sagitta).magnitude() < Tolerance::ZERO_TOLERANCE)
        return NurbsCurve::create(false, 1, {start, end});

    const double h = chord.magnitude() * 0.5;
    const double s = sagitta.magnitude();
    const double radius = (h * h + s * s) / (2.0 * s);
    double w = (radius - s) / radius;

    if (std::abs(w) < Tolerance::ZERO_TOLERANCE)
        w = Tolerance::ZERO_TOLERANCE;

    NurbsCurve curve(3, true, 3, 3);
    curve.m_nurbsknot = {0.0, 0.0, 1.0, 1.0};
    curve.set_cv_4d(0, start[0], start[1], start[2], 1.0);
    curve.set_cv_4d(1, chord_mid[0] * w + sagitta[0], chord_mid[1] * w + sagitta[1], chord_mid[2] * w + sagitta[2], w);
    curve.set_cv_4d(2, end[0], end[1], end[2], 1.0);

    return curve;
}

NurbsCurve Primitives::parabola(const Point& p0, const Point& p1, const Point& p2) {

    NurbsCurve curve(3, false, 3, 3);
    curve.m_nurbsknot = {0.0, 0.0, 1.0, 1.0};
    curve.set_cv(0, p0);
    curve.set_cv(
        1,
        Point(
            2.0 * p1[0] - (p0[0] + p2[0]) / 2.0,
            2.0 * p1[1] - (p0[1] + p2[1]) / 2.0,
            2.0 * p1[2] - (p0[2] + p2[2]) / 2.0
        )
    );
    curve.set_cv(2, p2);

    return curve;
}

NurbsCurve Primitives::hyperbola(const Point& center, double a, double b, double extent) {

    const int segments = 8;
    std::vector<Point> points;

    for (int i = 0; i <= segments; i++) {
        const double t = -extent + 2.0 * extent * i / segments;
        points.push_back(Point(center[0] + a * std::cosh(t), center[1] + b * std::sinh(t), center[2]));
    }

    NurbsCurve curve;

    if (!curve.create_clamped_uniform(3, 4, points, 1.0))
        return NurbsCurve();

    return curve;
}

NurbsCurve Primitives::spiral(double start_radius, double end_radius, double pitch, double turns) {

    const int segments = std::max(4, static_cast<int>(turns * 8));
    std::vector<Point> points;

    for (int i = 0; i <= segments; i++) {
        const double t = static_cast<double>(i) / segments;
        const double angle = t * turns * 2.0 * Tolerance::PI;
        const double r = start_radius + t * (end_radius - start_radius);
        points.push_back(Point(r * std::cos(angle), r * std::sin(angle), t * turns * pitch));
    }

    NurbsCurve curve;

    if (!curve.create_clamped_uniform(3, 4, points, 1.0))
        return NurbsCurve();

    return curve;
}

NurbsCurve Primitives::create_interpolated(
    const std::vector<Point>& points,
    CurveNurbsKnotStyle parameterization,
    CurveInterpStyle end_condition
) {
    return NurbsCurve::create_interpolated(points, parameterization, end_condition);
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface primitives
// ═══════════════════════════════════════════════════════════════════════════

NurbsSurface Primitives::cylinder_surface(double cx, double cy, double cz, double radius, double height) {

    NurbsSurface srf(3, true, 3, 2, 9, 2);

    for (int i = 0; i < 10; i++)
        srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);

    set_circle_row(srf, 0, cx, cy, cz, radius, 1.0);
    set_circle_row(srf, 1, cx, cy, cz + height, radius, 1.0);

    return srf;
}

NurbsSurface Primitives::cone_surface(double cx, double cy, double cz, double radius, double height) {

    NurbsSurface srf(3, true, 3, 2, 9, 2);

    for (int i = 0; i < 10; i++)
        srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);

    set_circle_row(srf, 0, cx, cy, cz, radius, 1.0);
    set_circle_row(srf, 1, cx, cy, cz + height, 0.0, 1.0);

    return srf;
}

NurbsSurface Primitives::torus_surface(double cx, double cy, double cz, double major_radius, double minor_radius) {

    NurbsSurface srf(3, true, 3, 3, 9, 9);

    for (int i = 0; i < 10; i++) {
        srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);
        srf.set_nurbsknot(1, i, CIRCLE_NURBSKNOTS[i]);
    }

    for (int j = 0; j < 9; j++)
        set_circle_row(
            srf,
            j,
            cx,
            cy,
            cz + minor_radius * CIRCLE_Y[j],
            major_radius + minor_radius * CIRCLE_X[j],
            CIRCLE_WEIGHTS[j]
        );
    return srf;
}

NurbsSurface Primitives::sphere_surface(double cx, double cy, double cz, double radius) {

    const double lat_r[5] = {0, 1, 1, 1, 0};
    const double lat_z[5] = {-1, -1, 0, 1, 1};
    const double lat_w[5] = {1, CIRCLE_W, 1, CIRCLE_W, 1};
    const double v_nurbsknots[6] = {0, 0, 1, 1, 2, 2};
    NurbsSurface srf(3, true, 3, 3, 9, 5);

    for (int i = 0; i < 10; i++)
        srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);

    for (int i = 0; i < 6; i++)
        srf.set_nurbsknot(1, i, v_nurbsknots[i]);

    for (int j = 0; j < 5; j++)
        set_circle_row(srf, j, cx, cy, cz + radius * lat_z[j], radius * lat_r[j], lat_w[j]);

    return srf;
}

std::vector<NurbsSurface> Primitives::quad_sphere(double cx, double cy, double cz, double radius) {

    const double a = radius / std::sqrt(3.0);
    const double e = radius * std::sqrt(3.0) / 2.0;
    const double wk = std::sqrt(2.0 / 3.0);
    const double wc = (-72.0 - 32.0 * std::sqrt(6.0) + 48.0 * std::sqrt(3.0) + 56.0 * std::sqrt(2.0)) /
        (48.0 * (1.0 + std::sqrt(2.0 / 3.0) - 1.0 / std::sqrt(3.0) - 1.0 / std::sqrt(2.0)));

    const double k = radius * (1.0 - 1.0 / std::sqrt(3.0) + 2.0 * std::sqrt(2.0 / 3.0) - std::sqrt(2.0));
    const double h = radius + k / wc;
    const double zf[3][3][4] = {
        {{-a, -a, a, 1.0}, {-e, 0.0, e, wk}, {-a, a, a, 1.0}},
        {{0.0, -e, e, wk}, {0.0, 0.0, h, wc}, {0.0, e, e, wk}},
        {{a, -a, a, 1.0}, {e, 0.0, e, wk}, {a, a, a, 1.0}},
    };
    const double rot[6][3][3] = {
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},
        {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}},
        {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}},
        {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}},
        {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}},
    };
    std::vector<NurbsSurface> faces;

    for (int f = 0; f < 6; f++) {
        NurbsSurface srf(3, true, 3, 3, 3, 3);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                const double* p = zf[i][j];
                const double rx = rot[f][0][0] * p[0] + rot[f][0][1] * p[1] + rot[f][0][2] * p[2] + cx;
                const double ry = rot[f][1][0] * p[0] + rot[f][1][1] * p[1] + rot[f][1][2] * p[2] + cy;
                const double rz = rot[f][2][0] * p[0] + rot[f][2][1] * p[1] + rot[f][2][2] * p[2] + cz;
                srf.set_cv_4d(i, j, rx * p[3], ry * p[3], rz * p[3], p[3]);
            }
        }

        faces.push_back(srf);
    }

    return faces;
}

NurbsSurface Primitives::wave_surface(double size, double amplitude) {

    const int n = 13;
    std::vector<Point> pts;

    for (int i = 0; i < n; i++) {
        const double u = static_cast<double>(i) / (n - 1);

        for (int j = 0; j < n; j++) {
            const double v = static_cast<double>(j) / (n - 1);
            pts.push_back(Point(
                size * u,
                size * v,
                amplitude * std::sin(2.0 * Tolerance::PI * u) * std::sin(2.0 * Tolerance::PI * v)
            ));
        }
    }

    return NurbsSurface::create(false, false, 3, 3, n, n, pts);
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface factories
// ═══════════════════════════════════════════════════════════════════════════

NurbsSurface Primitives::create_ruled(const NurbsCurve& curve_a, const NurbsCurve& curve_b) {

    if (!curve_a.is_valid() || !curve_b.is_valid())
        return NurbsSurface();

    std::vector<NurbsCurve> curves = {curve_a, curve_b};
    curves[0].set_domain(0.0, 1.0);
    curves[1].set_domain(0.0, 1.0);
    make_curves_compatible(curves);
    const int cv_count_u = curves[0].cv_count();
    const bool is_rat = curves[0].is_rational();
    NurbsSurface surface(3, is_rat, curves[0].order(), 2, cv_count_u, 2);

    if (!surface.is_valid())
        return NurbsSurface();

    for (int i = 0; i < surface.nurbsknot_count(0); i++)
        surface.set_nurbsknot(0, i, curves[0].nurbsknot(i));

    for (int i = 0; i < cv_count_u; i++) {
        for (int j = 0; j < 2; j++) {
            if (is_rat) {
                const auto [x, y, z, w] = curves[j].get_cv_4d(i);
                surface.set_cv_4d(i, j, x, y, z, w);
            } else {
                surface.set_cv(i, j, curves[j].get_cv(i));
            }
        }
    }

    return surface;
}

NurbsSurface Primitives::create_extrusion(const NurbsCurve& curve, const Vector& direction) {

    if (!curve.is_valid())
        return NurbsSurface();

    NurbsCurve translated = curve;
    translated.transform(Xform::translation(direction[0], direction[1], direction[2]));

    return create_ruled(curve, translated);
}

NurbsSurface Primitives::create_planar(const NurbsCurve& boundary) {

    if (!boundary.is_valid())
        return NurbsSurface();

    std::vector<Point> pts;

    for (int i = 0; i < boundary.cv_count(); i++)
        pts.push_back(boundary.get_cv(i));

    if (pts.size() >= 2 && pts.front().distance(pts.back()) < 1e-10)
        pts.pop_back();

    if (pts.size() < 3)
        return NurbsSurface();

    if (boundary.degree() <= 1 && pts.size() == 3)
        return bilinear_patch(pts[0], pts[1], pts[0], pts[2]);

    if (boundary.degree() <= 1 && pts.size() == 4)
        return bilinear_patch(pts[0], pts[1], pts[3], pts[2]);

    if (boundary.degree() <= 1) {
        Vector normal = (pts[1] - pts[0]).cross(pts[2] - pts[0]);

        if (!normal.normalize_self())
            return NurbsSurface();

        const Vector x_axis = longest_edge_dir(pts);
        Vector y_axis = normal.cross(x_axis);

        if (!y_axis.normalize_self())
            return NurbsSurface();

        return bounded_patch(pts, pts[0], x_axis, y_axis);
    }

    const auto [samples, params] = boundary.divide_by_count(std::max(20, boundary.cv_count() * 4));
    const Plane plane = Plane::from_points_pca(samples);

    if (plane.z_axis().magnitude() < 1e-10)
        return NurbsSurface();

    return bounded_patch(samples, plane.origin(), plane.x_axis(), plane.y_axis());
}

NurbsSurface Primitives::create_loft(const std::vector<NurbsCurve>& input_curves, int degree_v) {

    if (input_curves.size() < 2)
        return NurbsSurface();

    for (const NurbsCurve& c : input_curves)
        if (!c.is_valid())
            return NurbsSurface();

    std::vector<NurbsCurve> curves = input_curves;
    make_curves_compatible(curves);
    const int n = static_cast<int>(curves.size());
    const int cv_count_u = curves[0].cv_count();
    const bool is_rat = curves[0].is_rational();
    const int order_v = std::clamp(degree_v, 1, n - 1) + 1;
    const std::vector<double> v_params = loft_section_params(curves);
    const std::vector<double> nurbsknots_v = loft_nurbsknots(v_params, order_v);
    NurbsSurface surface(3, is_rat, curves[0].order(), order_v, cv_count_u, n);

    if (!surface.is_valid())
        return NurbsSurface();

    for (int i = 0; i < surface.nurbsknot_count(0); i++)
        surface.set_nurbsknot(0, i, curves[0].nurbsknot(i));

    for (int i = 0; i < surface.nurbsknot_count(1); i++)
        surface.set_nurbsknot(1, i, nurbsknots_v[i]);

    std::vector<std::vector<double>> basis(n);

    for (int k = 0; k < n; k++)
        basis[k] = loft_basis_row(nurbsknots_v, order_v, n, v_params[k]);

    const int dim = is_rat ? 4 : 3;

    for (int i = 0; i < cv_count_u; i++) {
        std::vector<std::vector<double>> rhs(n, std::vector<double>(dim, 0.0));

        for (int k = 0; k < n; k++) {
            if (is_rat) {
                const auto [x, y, z, w] = curves[k].get_cv_4d(i);
                rhs[k] = {x, y, z, w};
            } else {
                const Point p = curves[k].get_cv(i);
                rhs[k] = {p[0], p[1], p[2]};
            }
        }

        const std::vector<std::vector<double>> q = solve_linear(basis, rhs);

        for (int j = 0; j < n; j++)
            if (is_rat)
                surface.set_cv_4d(i, j, q[j][0], q[j][1], q[j][2], q[j][3]);
            else
                surface.set_cv(i, j, Point(q[j][0], q[j][1], q[j][2]));
    }

    return surface;
}

NurbsSurface Primitives::create_revolve(
    const NurbsCurve& profile,
    const Point& axis_origin,
    const Vector& axis_direction,
    double angle
) {

    if (!profile.is_valid())
        return NurbsSurface();

    Vector axis = axis_direction;

    if (!axis.normalize_self())
        return NurbsSurface();

    angle = std::min(std::abs(angle), 2.0 * Tolerance::PI);

    if (angle < 1e-14)
        return NurbsSurface();

    int n_arcs = 4;

    if (angle <= Tolerance::PI / 2.0 + 1e-10)
        n_arcs = 1;
    else if (angle <= Tolerance::PI + 1e-10)
        n_arcs = 2;
    else if (angle <= 3.0 * Tolerance::PI / 2.0 + 1e-10)
        n_arcs = 3;

    const double d_theta = angle / n_arcs;
    const double w_mid = std::cos(d_theta / 2.0);
    const int n_u = 2 * n_arcs + 1;
    const int cv_count_v = profile.cv_count();
    NurbsSurface surface(3, true, 3, profile.order(), n_u, cv_count_v);

    if (!surface.is_valid())
        return NurbsSurface();

    for (int i = 0; i < surface.nurbsknot_count(0); i++)
        surface.set_nurbsknot(0, i, i / 2 == n_arcs ? angle : (i / 2) * d_theta);

    for (int i = 0; i < surface.nurbsknot_count(1); i++)
        surface.set_nurbsknot(1, i, profile.nurbsknot(i));

    for (int j = 0; j < cv_count_v; j++) {
        const Point p = profile.get_cv(j);
        const double profile_w = profile.is_rational() ? profile.weight(j) : 1.0;
        const Point center = axis_origin + axis * (p - axis_origin).dot(axis);
        Vector x_local = p - center;
        const double r = x_local.magnitude();

        if (r > 1e-14)
            x_local /= r;

        const Vector y_local = axis.cross(x_local);

        for (int i = 0; i < n_u; i++) {
            const bool shoulder = i % 2 == 1;
            const double theta = (i / 2) * d_theta + (shoulder ? d_theta / 2.0 : 0.0);
            const double w = (shoulder ? w_mid : 1.0) * profile_w;
            const Point q =
                center + (x_local * std::cos(theta) + y_local * std::sin(theta)) * (shoulder ? r / w_mid : r);

            surface.set_cv_4d(i, j, q[0] * w, q[1] * w, q[2] * w, w);
        }
    }

    return surface;
}

NurbsSurface Primitives::create_sweep1(const NurbsCurve& rail, const NurbsCurve& profile) {

    if (!rail.is_valid() || !profile.is_valid())
        return NurbsSurface();

    const int count = std::clamp(rail.span_count() * 2 + 1, 5, 200);
    const std::vector<Plane> frames = rail.get_perpendicular_planes(count);

    if (frames.empty())
        return NurbsSurface();

    const Xform to_xy = profile_to_xy(profile);
    std::vector<NurbsCurve> sections;

    for (const Plane& frame : frames) {
        NurbsCurve section = profile;
        section.transform(Xform::to_frame(frame) * to_xy);
        sections.push_back(section);
    }

    return create_loft(sections, std::min(3, static_cast<int>(sections.size()) - 1));
}

NurbsSurface Primitives::create_sweep2(
    const NurbsCurve& rail1,
    const NurbsCurve& rail2,
    const std::vector<NurbsCurve>& shapes
) {

    if (!rail1.is_valid() || !rail2.is_valid() || shapes.empty())
        return NurbsSurface();

    for (const NurbsCurve& shape : shapes)
        if (!shape.is_valid())
            return NurbsSurface();

    std::vector<NurbsCurve> compat = shapes;
    make_curves_compatible(compat);
    const int n_shapes = static_cast<int>(compat.size());
    std::vector<Plane> planes;
    std::vector<double> widths;

    for (const NurbsCurve& shape : compat) {
        planes.push_back(shape_plane(shape));
        widths.push_back(shape_width(shape));
    }

    const int count = std::clamp(std::max(rail1.span_count(), rail2.span_count()) * 2 + 1, 5, 200);
    const auto [pts1, params1] = rail1.divide_by_count(count + 1);
    const auto [pts2, params2] = rail2.divide_by_count(count + 1);
    const std::vector<Plane> frames = rail1.get_perpendicular_planes(count);

    if (frames.empty())
        return NurbsSurface();

    std::vector<NurbsCurve> sections;

    for (size_t i = 0; i < frames.size() && i < pts1.size() && i < pts2.size(); i++) {
        const double t = frames.size() <= 1 ? 0.0 : static_cast<double>(i) / (frames.size() - 1);
        const int j = n_shapes == 1 ? 0 : std::min(static_cast<int>(t * (n_shapes - 1)), n_shapes - 2);
        const int j1 = n_shapes == 1 ? 0 : j + 1;
        const double s = n_shapes == 1 ? 0.0 : std::clamp(t * (n_shapes - 1) - j, 0.0, 1.0);
        NurbsCurve section = compat[j];

        for (int c = 0; c < section.cv_count(); c++)
            section.set_cv(c, lerp_point(compat[j].get_cv(c), compat[j1].get_cv(c), s));

        const Plane source(
            lerp_point(planes[j].origin(), planes[j1].origin(), s),
            lerp_vector(planes[j].x_axis(), planes[j1].x_axis(), s),
            lerp_vector(planes[j].y_axis(), planes[j1].y_axis(), s)
        );
        const double width = widths[j] * (1.0 - s) + widths[j1] * s;
        const Point p1 = pts1[i];
        Vector x_dir = pts2[i] - p1;
        const double rail_dist = x_dir.magnitude();

        if (!x_dir.normalize_self())
            x_dir = frames[i].x_axis();

        Vector y_dir = frames[i].z_axis().cross(x_dir);

        if (!y_dir.normalize_self())
            y_dir = frames[i].y_axis();

        if (y_dir.dot(source.y_axis()) < 0.0)
            y_dir = -y_dir;

        const double scale = rail_dist > 1e-14 && width > 1e-14 ? rail_dist / width : 1.0;
        const Plane target(p1, x_dir, y_dir);
        const Xform to_source =
            Xform::world_to_frame(source.origin(), source.x_axis(), source.y_axis(), source.z_axis());

        section.transform(Xform::to_frame(target) * Xform::scale_xyz(scale, scale, scale) * to_source);
        sections.push_back(section);
    }

    return create_loft(sections, std::min(3, static_cast<int>(sections.size()) - 1));
}

NurbsSurface Primitives::create_edge(
    const NurbsCurve& c0,
    const NurbsCurve& c1,
    const NurbsCurve& c2,
    const NurbsCurve& c3
) {

    if (!c0.is_valid() || !c1.is_valid() || !c2.is_valid() || !c3.is_valid())
        return NurbsSurface();

    const std::vector<NurbsCurve> loop = chain_curves({c0, c1, c2, c3});

    if (loop.empty())
        return NurbsSurface();

    std::vector<NurbsCurve> v_pair = {loop[0], loop[2]};
    v_pair[1].reverse();
    make_curves_compatible(v_pair);
    std::vector<NurbsCurve> u_pair = {loop[3], loop[1]};
    u_pair[0].reverse();
    make_curves_compatible(u_pair);
    const NurbsCurve& south = v_pair[0];
    const NurbsCurve& north = v_pair[1];
    const NurbsCurve& west = u_pair[0];
    const NurbsCurve& east = u_pair[1];
    const int cv_count_u = west.cv_count();
    const int cv_count_v = south.cv_count();
    NurbsSurface
        surface(3, south.is_rational() || west.is_rational(), west.order(), south.order(), cv_count_u, cv_count_v);

    if (!surface.is_valid())
        return NurbsSurface();

    for (int i = 0; i < surface.nurbsknot_count(0); i++)
        surface.set_nurbsknot(0, i, west.nurbsknot(i));

    for (int i = 0; i < surface.nurbsknot_count(1); i++)
        surface.set_nurbsknot(1, i, south.nurbsknot(i));

    const std::vector<double> u_grev = normalized_greville(west);
    const std::vector<double> v_grev = normalized_greville(south);
    const Point c00 = south.get_cv(0);
    const Point c01 = south.get_cv(cv_count_v - 1);
    const Point c10 = north.get_cv(0);
    const Point c11 = north.get_cv(cv_count_v - 1);

    for (int i = 0; i < cv_count_u; i++) {
        const double ui = u_grev[i];
        const Point wi = west.get_cv(i);
        const Point ei = east.get_cv(i);

        for (int j = 0; j < cv_count_v; j++) {
            const double vj = v_grev[j];
            const Point sj = south.get_cv(j);
            const Point nj = north.get_cv(j);
            double q[3];

            for (int axis = 0; axis < 3; axis++)
                q[axis] = (1.0 - ui) * sj[axis] + ui * nj[axis] + (1.0 - vj) * wi[axis] + vj * ei[axis] -
                    (1.0 - ui) * (1.0 - vj) * c00[axis] - (1.0 - ui) * vj * c01[axis] - ui * (1.0 - vj) * c10[axis] -
                    ui * vj * c11[axis];

            surface.set_cv(i, j, Point(q[0], q[1], q[2]));
        }
    }

    return surface;
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface to mesh
// ═══════════════════════════════════════════════════════════════════════════

Mesh Primitives::quad_mesh(const NurbsSurface& surface, int u_count, int v_count) {

    Mesh mesh;
    const std::vector<std::vector<size_t>> grid = surface_grid(surface, u_count, v_count, mesh);
    const bool singular_south = surface.is_singular(0);
    const bool singular_north = surface.is_singular(2);

    if (singular_south)
        for (int i = 0; i < u_count; i++)
            mesh.add_face({grid[0][0], grid[i + 1][1], grid[i][1]});

    if (singular_north)
        for (int i = 0; i < u_count; i++)
            mesh.add_face({grid[0][v_count], grid[i][v_count - 1], grid[i + 1][v_count - 1]});

    const int j0 = singular_south ? 1 : 0;
    const int j1 = singular_north ? v_count - 1 : v_count;

    for (int i = 0; i < u_count; i++)
        for (int j = j0; j < j1; j++)
            mesh.add_face({grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]});

    return mesh;
}

Mesh Primitives::diamond_mesh(const NurbsSurface& surface, int u_count, int v_count) {

    Mesh mesh;
    const std::vector<std::vector<size_t>> grid = surface_grid(surface, u_count, v_count, mesh);
    const bool closed_u = surface.is_closed(0);
    const int u_end = closed_u ? u_count - 1 : u_count;

    for (int i = 0; i <= u_end; i++) {
        for (int j = 0; j <= v_count; j++) {
            if ((i + j) % 2 != 0)
                continue;

            const size_t center = grid[i][j];
            const int il = i > 0 ? i - 1 : (closed_u ? u_count - 1 : -1);
            const size_t left = il >= 0 ? grid[il][j] : center;
            const size_t bottom = j > 0 ? grid[i][j - 1] : center;
            const size_t right = i < u_count ? grid[i + 1][j] : center;
            const size_t top = j < v_count ? grid[i][j + 1] : center;
            const std::vector<size_t> face = dedup_face({left, bottom, right, top});

            if (face.size() >= 3)
                mesh.add_face(face);
        }
    }

    return mesh;
}

Mesh Primitives::hex_mesh(const NurbsSurface& surface, int u_count, int v_count, double t) {

    Mesh mesh;
    const std::vector<std::vector<size_t>> grid = surface_grid(surface, u_count, v_count, mesh);
    const std::vector<std::vector<size_t>> mid_a = surface_mid_grid(surface, u_count, v_count, t, mesh);
    const std::vector<std::vector<size_t>> mid_b = surface_mid_grid(surface, u_count, v_count, 1.0 - t, mesh);
    const bool closed_u = surface.is_closed(0);
    const int u_end = closed_u ? u_count - 1 : u_count;

    for (int i = 0; i <= u_end; i++) {
        for (int j = 0; j <= v_count; j++) {
            if ((i + j) % 2 != 0)
                continue;

            const size_t center = grid[i][j];
            const int il = i > 0 ? i - 1 : (closed_u ? u_count - 1 : -1);
            const size_t ul = il >= 0 && j < v_count ? mid_a[il][j] : (il >= 0 ? grid[il][j] : center);
            const size_t ll = il >= 0 && j > 0 ? mid_b[il][j - 1] : (il >= 0 ? grid[il][j] : center);
            const size_t bt = j > 0 ? mid_a[i][j - 1] : center;
            const size_t lr = i < u_count && j > 0 ? mid_b[i + 1][j - 1] : (i < u_count ? grid[i + 1][j] : center);
            const size_t ur = i < u_count && j < v_count ? mid_a[i + 1][j] : (i < u_count ? grid[i + 1][j] : center);
            const size_t tp = j < v_count ? mid_b[i][j] : center;
            const std::vector<size_t> face = dedup_face({ul, ll, bt, lr, ur, tp});

            if (face.size() >= 3)
                mesh.add_face(face);
        }
    }

    return mesh;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh geometry
// ═══════════════════════════════════════════════════════════════════════════

Primitives::Geometry Primitives::unit_cylinder_geometry() {

    const size_t n = 10;
    std::vector<Point> vertices;
    add_ring(vertices, n, 0.5, -0.5);
    add_ring(vertices, n, 0.5, 0.5);
    std::vector<std::array<size_t, 3>> triangles;

    for (size_t i = 0; i < n; i++) {
        const size_t next = (i + 1) % n;
        triangles.push_back({i, next, n + next});
        triangles.push_back({i, n + next, n + i});
    }

    return {vertices, triangles};
}

Primitives::Geometry Primitives::unit_cone_geometry() {

    const size_t n = 8;
    std::vector<Point> vertices = {Point(0.0, 0.0, 0.5)};
    add_ring(vertices, n, 0.5, -0.5);
    std::vector<std::array<size_t, 3>> triangles;

    for (size_t i = 0; i < n; i++)
        triangles.push_back({0, 1 + i, 1 + (i + 1) % n});

    return {vertices, triangles};
}

Primitives::Geometry Primitives::capsule_geometry(double length, double radius) {

    const size_t n = 10;
    const double r_hemi = radius * std::sin(Tolerance::PI / 4.0);
    const double off = radius * std::cos(Tolerance::PI / 4.0);
    const size_t top = n;
    const size_t hemi_a = 2 * n;
    const size_t pole_a = 3 * n;
    const size_t hemi_b = 3 * n + 1;
    const size_t pole_b = 4 * n + 1;
    std::vector<Point> vertices;
    add_ring(vertices, n, radius, 0.0);
    add_ring(vertices, n, radius, length);
    add_ring(vertices, n, r_hemi, -off);
    vertices.push_back(Point(0.0, 0.0, -radius));
    add_ring(vertices, n, r_hemi, length + off);
    vertices.push_back(Point(0.0, 0.0, length + radius));
    std::vector<std::array<size_t, 3>> triangles;

    for (size_t i = 0; i < n; i++) {
        const size_t next = (i + 1) % n;
        triangles.push_back({i, next, top + next});
        triangles.push_back({i, top + next, top + i});
        triangles.push_back({hemi_a + i, next, i});
        triangles.push_back({hemi_a + i, hemi_a + next, next});
        triangles.push_back({top + i, top + next, hemi_b + next});
        triangles.push_back({top + i, hemi_b + next, hemi_b + i});
        triangles.push_back({pole_a, hemi_a + next, hemi_a + i});
        triangles.push_back({pole_b, hemi_b + i, hemi_b + next});
    }

    return {vertices, triangles};
}

Xform Primitives::line_frame(const Line& line, const Point& origin) {

    Vector z_axis = line.to_vector();

    if (!z_axis.normalize_self())
        z_axis = Vector(0.0, 0.0, 1.0);

    const Vector pole = std::abs(z_axis[2]) < 0.9 ? Vector(0.0, 0.0, 1.0) : Vector(1.0, 0.0, 0.0);
    const Vector x_axis = pole.cross(z_axis);

    return Xform::xy_to_plane(origin, x_axis, z_axis.cross(x_axis), z_axis);
}

void Primitives::add_geometry(Mesh& mesh, const Geometry& geometry, const Xform& xform) {

    const auto& [vertices, triangles] = geometry;
    std::vector<size_t> keys;

    for (const Point& v : vertices)
        keys.push_back(mesh.add_vertex(v.transformed(xform)));

    for (const std::array<size_t, 3>& tri : triangles)
        mesh.add_face({keys[tri[0]], keys[tri[1]], keys[tri[2]]});
}

} // namespace session_cpp
