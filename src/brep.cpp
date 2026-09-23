#include "brep.h"
#include "brep.pb.h"
#include "fmt/core.h"
#include "nurbssurface_trimmed.h"
#include "primitives.h"
#include "remesh_nurbssurface_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Orientation
// ═══════════════════════════════════════════════════════════════════════════
BRepOrientation brep_reverse(BRepOrientation o) {

    if (o == BRepOrientation::Forward)
        return BRepOrientation::Reversed;

    if (o == BRepOrientation::Reversed)
        return BRepOrientation::Forward;

    return o;
}

BRepOrientation brep_compose(BRepOrientation a, BRepOrientation b) {

    if (a == BRepOrientation::Internal || a == BRepOrientation::External)
        return a;

    if (a == BRepOrientation::Forward)
        return b;

    return brep_reverse(b);
}

/// remesh_cdt.cpp: triangle index triples of a counter-clockwise 2D border with clockwise holes into the flat list [border..., hole0..., hole1...]
std::vector<std::array<int, 3>> cdt_triangulate(
    const std::vector<std::pair<double, double>>& border_2d,
    const std::vector<std::vector<std::pair<double, double>>>& holes_2d
);

namespace {

const BRepOrientation F = BRepOrientation::Forward;
const BRepOrientation R = BRepOrientation::Reversed;

/// JSON name of an orientation
std::string orientation_to_str(BRepOrientation o) {

    if (o == BRepOrientation::Reversed)
        return "reversed";

    if (o == BRepOrientation::Internal)
        return "internal";

    if (o == BRepOrientation::External)
        return "external";

    return "forward";
}

/// Orientation of a JSON name, Forward when unknown
BRepOrientation orientation_from_str(const std::string& s) {

    if (s == "reversed")
        return BRepOrientation::Reversed;

    if (s == "internal")
        return BRepOrientation::Internal;

    if (s == "external")
        return BRepOrientation::External;

    return BRepOrientation::Forward;
}

/// True when `index` addresses one of `count` table entries
bool in_range(int index, size_t count) {

    return index >= 0 && index < (int)count;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Bilinear planar patch: u runs p00 -> p10, v runs p00 -> p01, natural normal = u x v
NurbsSurface bilinear_patch(const Point& p00, const Point& p10, const Point& p01, const Point& p11) {

    NurbsSurface srf(3, false, 2, 2, 2, 2);
    srf.set_cv(0, 0, p00);
    srf.set_cv(1, 0, p10);
    srf.set_cv(0, 1, p01);
    srf.set_cv(1, 1, p11);

    return srf;
}

/// Straight pcurve from (u0, v0) to (u1, v1)
NurbsCurve uv_line(double u0, double v0, double u1, double v1) {

    return NurbsCurve::create(false, 1, {
        Point(u0, v0, 0),
        Point(u1, v1, 0)
    });
}

/// Exact pcurve of a 3D curve lying on a bilinear planar patch: the affine image of its CVs
NurbsCurve project_to_patch(const NurbsCurve& crv, const NurbsSurface& srf) {

    const Point p00 = srf.get_cv(0, 0);
    const Vector eu = srf.get_cv(1, 0) - p00;
    const Vector ev = srf.get_cv(0, 1) - p00;
    const double eu2 = eu.dot(eu);
    const double ev2 = ev.dot(ev);
    NurbsCurve c2(3, crv.is_rational(), crv.order(), crv.cv_count());

    for (int i = 0; i < crv.nurbsknot_count(); ++i)
        c2.set_nurbsknot(i, crv.nurbsknot(i));

    for (int i = 0; i < crv.cv_count(); ++i) {
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        double w = 0.0;
        std::tie(wx, wy, wz, w) = crv.get_cv_4d(i);
        const Vector d = Point(wx / w, wy / w, wz / w) - p00;
        const double u = d.dot(eu) / eu2;
        const double v = d.dot(ev) / ev2;

        if (crv.is_rational())
            c2.set_cv_4d(i, u * w, v * w, 0.0, w);
        else
            c2.set_cv(i, Point(u, v, 0));
    }

    return c2;
}

/// Signed area of a closed pcurve's sampled polygon (positive = counter-clockwise)
double uv_signed_area(const NurbsCurve& c2d) {

    const std::vector<Point> pts = c2d.divide_by_count(std::max(c2d.cv_count() * 4, 16), true).first;
    double area = 0.0;

    for (size_t i = 0; i + 1 < pts.size(); ++i)
        area += pts[i][0] * pts[i + 1][1] - pts[i + 1][0] * pts[i][1];

    return 0.5 * area;
}

/// Signed area of a closed UV polygon (positive = counter-clockwise)
double polygon_signed_area(const std::vector<Point>& pts) {

    const size_t n = pts.size();
    double area = 0.0;

    for (size_t i = 0; i < n; ++i) {
        const Point& p = pts[i];
        const Point& q = pts[(i + 1) % n];
        area += p[0] * q[1] - q[0] * p[1];
    }

    return 0.5 * area;
}

/// Diagonal of the control point bounding box
double bbox_diagonal(const NurbsSurface& srf) {

    Point lo(1e30, 1e30, 1e30);
    Point hi(-1e30, -1e30, -1e30);

    for (int i = 0; i < srf.cv_count(0); ++i)
        for (int j = 0; j < srf.cv_count(1); ++j) {
            const Point p = srf.get_cv(i, j);

            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }

    return hi.distance(lo);
}

// ═══════════════════════════════════════════════════════════════════════════
// Factory helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Planar polygon faces from a vertex table: edges run lo -> hi vertex and are shared, a face lists its vertices counter-clockwise seen from outside so the patch normal points outward
struct PolyFaceBuilder {
    BRep& b; // BRep being built.
    std::map<std::pair<int, int>, int> edge_map; // Edge per (lo, hi) pair.

    /// Straight edge between two vertices, shared by every face that uses it
    int edge(int v0, int v1) {

        const int lo = std::min(v0, v1);
        const int hi = std::max(v0, v1);
        const auto it = edge_map.find({lo, hi});

        if (it != edge_map.end())
            return it->second;

        const NurbsCurve line = NurbsCurve::create(false, 1, {b.m_vertices[lo].point, b.m_vertices[hi].point});
        const int ei = b.add_edge(b.add_curve_3d(line), lo, hi);
        edge_map[{lo, hi}] = ei;

        return ei;
    }

    /// Oriented edge references of the vertex cycle `vi`, each with its pcurve on surface `si`
    std::vector<BRepRef> wire_refs(int si, const std::vector<int>& vi) {

        const NurbsSurface& srf = b.m_surfaces[si];
        std::vector<BRepRef> refs;
        const int n = (int)vi.size();

        for (int i = 0; i < n; ++i) {
            const int va = vi[i];
            const int vb = vi[(i + 1) % n];
            const int ei = edge(va, vb);
            b.add_pcurve(ei, si, b.add_curve_2d(project_to_patch(b.m_curves_3d[b.m_edges[ei].curve_3d_index], srf)));
            refs.push_back({ei, b.m_edges[ei].start_vertex == va ? F : R});
        }

        return refs;
    }

    /// Face on `srf` bounded by the vertex cycle `vi`, with one inner wire per hole cycle; returns the face index
    int face(const NurbsSurface& srf, const std::vector<int>& vi, const std::vector<std::vector<int>>& holes = {}) {

        const int si = b.add_surface(srf);
        std::vector<BRepRef> wires{{b.add_wire(wire_refs(si, vi)), F}};

        for (const std::vector<int>& hole : holes)
            wires.push_back({b.add_wire(wire_refs(si, hole)), F});

        return b.add_face(si, wires);
    }
};

const std::vector<int> BOX_FACES[6] = {
    {0, 3, 2, 1},
    {4, 5, 6, 7},
    {0, 1, 5, 4},
    {1, 2, 6, 5},
    {2, 3, 7, 6},
    {3, 0, 4, 7},
};

/// Bilinear patch spanned by four vertex indices in face order (p00, p10, p11, p01)
NurbsSurface quad_patch(const BRep& b, const std::vector<int>& fv) {

    return bilinear_patch(b.m_vertices[fv[0]].point, b.m_vertices[fv[1]].point, b.m_vertices[fv[3]].point, b.m_vertices[fv[2]].point);
}

/// The eight corners of an origin-centered box, bottom ring then top ring
void box_corners(BRep& b, double sx, double sy, double sz) {

    const double hx = sx * 0.5;
    const double hy = sy * 0.5;
    const double hz = sz * 0.5;

    b.add_vertex(Point(-hx, -hy, -hz));
    b.add_vertex(Point(hx, -hy, -hz));
    b.add_vertex(Point(hx, hy, -hz));
    b.add_vertex(Point(-hx, hy, -hz));
    b.add_vertex(Point(-hx, -hy, hz));
    b.add_vertex(Point(hx, -hy, hz));
    b.add_vertex(Point(hx, hy, hz));
    b.add_vertex(Point(-hx, hy, hz));
}

/// Planar cap at height z with natural normal +Z (up) or -Z (down), spanning [-r, r]^2
NurbsSurface cap_patch(double r, double z, bool up) {

    if (up)
        return bilinear_patch(Point(-r, -r, z), Point(r, -r, z), Point(-r, r, z), Point(r, r, z));

    return bilinear_patch(Point(-r, -r, z), Point(-r, r, z), Point(r, -r, z), Point(r, r, z));
}

/// Cap face bounded by one closed edge: outer wire counter-clockwise in the patch's UV
int cap_face(BRep& b, const NurbsSurface& cap, int edge) {

    const int si = b.add_surface(cap);
    const NurbsCurve c2d = project_to_patch(b.m_curves_3d[b.m_edges[edge].curve_3d_index], cap);
    const BRepOrientation o = uv_signed_area(c2d) > 0.0 ? F : R;
    b.add_pcurve(edge, si, b.add_curve_2d(c2d));

    return b.add_face(si, {{b.add_wire({{edge, o}}), F}});
}

/// Periodic body face (cylinder / cone / bore): seam from v0 to v1 at u0 == u1, bottom ring forward at v0, top ring (or degenerated apex) reversed at v1
int body_face(BRep& b, int si, int e_bot, int e_seam, int e_top) {

    const std::pair<double, double> du = b.m_surfaces[si].domain(0);
    const std::pair<double, double> dv = b.m_surfaces[si].domain(1);
    const double u0 = du.first;
    const double u1 = du.second;
    const double v0 = dv.first;
    const double v1 = dv.second;

    b.add_pcurve(e_bot, si, b.add_curve_2d(uv_line(u0, v0, u1, v0)));
    b.add_pcurve(e_top, si, b.add_curve_2d(uv_line(u0, v1, u1, v1)));
    b.add_pcurve(e_seam, si, b.add_curve_2d(uv_line(u1, v0, u1, v1)), b.add_curve_2d(uv_line(u0, v0, u0, v1)));

    return b.add_face(si, {{b.add_wire({{e_bot, F}, {e_seam, F}, {e_top, R}, {e_seam, R}}), F}});
}

/// Point of the plane (org, xa, ya) at (u, v)
Point plane_point(const Point& org, const Vector& xa, const Vector& ya, double u, double v) {

    return org + xa * u + ya * v;
}

/// Padded bilinear patch through `pts` in the plane (org, xa, ya)
NurbsSurface planar_patch_through(const std::vector<Point>& pts, const Point& org, const Vector& xa, const Vector& ya) {

    double umin = 1e30;
    double umax = -1e30;
    double vmin = 1e30;
    double vmax = -1e30;

    for (const Point& p : pts) {
        const Vector d = p - org;
        const double u = d.dot(xa);
        const double v = d.dot(ya);
        umin = std::min(umin, u);
        umax = std::max(umax, u);
        vmin = std::min(vmin, v);
        vmax = std::max(vmax, v);
    }

    const double pad = std::max(umax - umin, vmax - vmin) * 0.01;
    umin -= pad;
    umax += pad;
    vmin -= pad;
    vmax += pad;

    return bilinear_patch(
        plane_point(org, xa, ya, umin, vmin),
        plane_point(org, xa, ya, umax, vmin),
        plane_point(org, xa, ya, umin, vmax),
        plane_point(org, xa, ya, umax, vmax)
    );
}

/// Signed area of a closed cycle of points seen in the plane (org, xa, ya): positive when it runs counter-clockwise
double signed_area_in_plane(const std::vector<Point>& pts, const Point& org, const Vector& xa, const Vector& ya) {

    double area = 0.0;
    const size_t n = pts.size();

    for (size_t i = 0; i < n; ++i) {
        const Vector a = pts[i] - org;
        const Vector b = pts[(i + 1) % n] - org;
        area += a.dot(xa) * b.dot(ya) - b.dot(xa) * a.dot(ya);
    }

    return area * 0.5;
}

/// The vertices of a polyline without the closing duplicate
std::vector<Point> open_points(const Polyline& pl) {

    std::vector<Point> pts = pl.get_points();
    const size_t n = pl.is_closed() ? (pts.empty() ? 0 : pts.size() - 1) : pts.size();
    pts.resize(n);

    return pts;
}

/// Index of the first vertex within `tol` of `p`, a new vertex when none is
int find_or_add_vertex(BRep& b, const Point& p, double tol) {

    for (int i = 0; i < (int)b.m_vertices.size(); ++i)
        if (b.m_vertices[i].point.distance(p) < tol)
            return i;

    return b.add_vertex(p);
}

/// Euclidean control points of a curve, zero weights skipped
std::vector<Point> cv_points(const NurbsCurve& c) {

    std::vector<Point> pts;

    for (int k = 0; k < c.cv_count(); ++k) {
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        double w = 0.0;
        std::tie(wx, wy, wz, w) = c.get_cv_4d(k);

        if (w != 0.0)
            pts.push_back(Point(wx / w, wy / w, wz / w));
    }

    return pts;
}

/// One-edge wire of a closed or open curve on planar surface `si`, sharing vertices within `tol`
int curve_wire(BRep& b, const NurbsCurve& crv, int si, double tol) {

    const Point sp = crv.point_at(crv.domain().first);
    const Point ep = crv.point_at(crv.domain().second);
    const int vs = find_or_add_vertex(b, sp, tol);
    const int ve = crv.is_closed() ? vs : find_or_add_vertex(b, ep, tol);
    const int ei = b.add_edge(b.add_curve_3d(crv), vs, ve);
    b.add_pcurve(ei, si, b.add_curve_2d(project_to_patch(crv, b.m_surfaces[si])));

    return b.add_wire({{ei, F}});
}

// ═══════════════════════════════════════════════════════════════════════════
// Sewing helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Signed volume of face meshes (positive when the windings point outward)
double signed_volume(const std::vector<Mesh>& meshes) {

    double total = 0.0;

    for (const Mesh& fm : meshes)
        for (const std::pair<const size_t, std::vector<size_t>>& entry : fm.face)
            for (size_t k = 1; k + 1 < entry.second.size(); ++k) {
                const Point a = fm.vertex.at(entry.second[0]).position();
                const Point b = fm.vertex.at(entry.second[k]).position();
                const Point c = fm.vertex.at(entry.second[k + 1]).position();
                total += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
            }

    return total / 6.0;
}

/// Face uses of every edge as (face, composed orientation); empty when some edge is not used exactly twice
std::vector<std::vector<std::pair<int, BRepOrientation>>> edge_uses(const BRep& b) {

    std::vector<std::vector<std::pair<int, BRepOrientation>>> uses(b.m_edges.size());

    for (int fi = 0; fi < b.face_count(); ++fi)
        for (const BRepRef& wr : b.m_faces[fi].wires)
            for (const BRepRef& er : b.wire_edges(wr))
                uses[er.index].push_back({fi, er.orientation});

    for (const std::vector<std::pair<int, BRepOrientation>>& use : uses)
        if (use.size() != 2)
            return {};

    return uses;
}

/// Connected components of faces, each face oriented consistently with the neighbour it was reached from
std::vector<std::vector<int>> face_components(
    const BRep& b,
    const std::vector<std::vector<std::pair<int, BRepOrientation>>>& uses,
    std::vector<BRepOrientation>& fo
) {

    const int nf = b.face_count();
    std::vector<bool> seen(nf, false);
    std::vector<std::vector<int>> components;

    for (int seed = 0; seed < nf; ++seed) {
        if (seen[seed])
            continue;

        std::vector<int> comp;
        std::vector<int> stack = {seed};
        seen[seed] = true;

        while (!stack.empty()) {
            const int fi = stack.back();
            stack.pop_back();
            comp.push_back(fi);

            for (const BRepRef& wr : b.m_faces[fi].wires)
                for (const BRepRef& er : b.wire_edges(wr))
                    for (const std::pair<int, BRepOrientation>& use : uses[er.index]) {
                        if (use.first == fi || seen[use.first])
                            continue;

                        fo[use.first] = use.second == er.orientation ? brep_reverse(fo[fi]) : fo[fi];
                        seen[use.first] = true;
                        stack.push_back(use.first);
                    }
        }

        components.push_back(comp);
    }

    return components;
}

/// BRepBuilderAPI_Sewing + MakeSolid for free faces: when every edge is shared by exactly two face uses, one shell per connected component wound outward and one solid per shell
void close_free_faces(BRep& b) {

    const int nf = b.face_count();

    if (nf == 0)
        return;

    const std::vector<std::vector<std::pair<int, BRepOrientation>>> uses = edge_uses(b);

    if (uses.empty())
        return;

    std::vector<BRepOrientation> fo(nf, F);
    std::vector<BRepRef> shells;

    for (const std::vector<int>& comp : face_components(b, uses, fo)) {
        std::vector<BRepRef> refs;

        for (const int fi : comp)
            refs.push_back({fi, fo[fi]});

        shells.push_back({b.add_shell(refs), F});
    }

    const std::vector<Mesh> fm = b.face_meshes();

    for (const BRepRef& sr : shells) {
        std::vector<Mesh> part;

        for (const BRepRef& fr : b.m_shells[sr.index].faces)
            part.push_back(fm[fr.index]);

        if (signed_volume(part) < 0.0)
            for (BRepRef& fr : b.m_shells[sr.index].faces)
                fr.orientation = brep_reverse(fr.orientation);

        b.add_solid({sr});
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Planar face helpers
// ═══════════════════════════════════════════════════════════════════════════
const int CURVED_EDGE_SAMPLES = 16; // Samples per curved edge of a planar face.

/// Open outline of a face's outer wire in wire order: vertices of straight edges, samples of curved ones
std::vector<Point> face_outline(const BRep& b, int fi) {

    std::vector<Point> points;

    for (const BRepRef& er : b.wire_edges(b.m_faces[fi].wires[0])) {
        if (er.index < 0 || er.index >= (int)b.m_edges.size())
            continue;

        const BRepEdge& edge = b.m_edges[er.index];

        if (edge.degenerated)
            continue;

        const bool reversed = er.orientation == BRepOrientation::Reversed;
        const bool curved = edge.curve_3d_index >= 0 && b.m_curves_3d[edge.curve_3d_index].degree() > 1;

        if (curved) {
            const NurbsCurve& c = b.m_curves_3d[edge.curve_3d_index];
            const double d0 = c.domain().first;
            const double d1 = c.domain().second;

            for (int s = 0; s < CURVED_EDGE_SAMPLES; ++s) {
                const double u = (double)s / (double)CURVED_EDGE_SAMPLES;
                const double t = reversed ? d1 + (d0 - d1) * u : d0 + (d1 - d0) * u;
                points.push_back(c.point_at(t));
            }
        } else {
            const int start = reversed ? edge.end_vertex : edge.start_vertex;

            if (start >= 0 && start < (int)b.m_vertices.size())
                points.push_back(b.m_vertices[start].point);
        }
    }

    return points;
}

/// Signed volume enclosed by closed outlines (tetrahedra fans from the origin), positive when wound outward
double outline_volume(const std::vector<Polyline>& polylines) {

    double total = 0.0;

    for (const Polyline& pl : polylines) {
        const std::vector<Point> pts = pl.get_points();

        if (pts.size() < 3)
            continue;

        const Point& p0 = pts[0];

        for (size_t k = 1; k + 1 < pts.size(); ++k) {
            const Point& p1 = pts[k];
            const Point& p2 = pts[k + 1];
            total += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1]) + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2]) + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
        }
    }

    return total / 6.0;
}

/// Outer polyline and outward plane of every planar face in one walk, so the two stay index-aligned
std::pair<std::vector<Polyline>, std::vector<Plane>> planar_faces(const BRep& b) {

    std::vector<Polyline> polylines;
    std::vector<Plane> planes;

    for (int fi = 0; fi < b.face_count(); ++fi) {
        const BRepFace& face = b.m_faces[fi];

        if (face.surface_index < 0 || face.wires.empty())
            continue;

        if (!b.m_surfaces[face.surface_index].is_planar())
            continue;

        std::vector<Point> points = face_outline(b, fi);

        if (points.size() < 3)
            continue;

        const Point origin = Point::centroid(points);
        Vector normal = Vector::average_normal(points);

        if (b.face_orientation(fi) == BRepOrientation::Reversed)
            normal.reverse();

        points.push_back(points.front());
        polylines.emplace_back(points);
        planes.push_back(Plane::from_point_normal(origin, normal));
    }

    if (b.is_solid() && outline_volume(polylines) < 0.0)
        for (Plane& pl : planes) {
            Vector n = pl.z_axis();
            n.reverse();
            pl = Plane::from_point_normal(pl.origin(), n);
        }

    return {polylines, planes};
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshing helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Canonical boundary of every shared edge: model points, the (face, pcurve, parameters) that produced them, and refined (t, uv) samples
struct EdgeBoundary {
    std::map<int, std::vector<Point>> points; // Canonical model points per edge.
    std::map<int, std::tuple<int, int, std::vector<double>>> basis; // (face, pcurve, parameters) per edge.
    std::map<int, std::vector<std::pair<double, Point>>> samples; // Refined (t, uv) samples per edge.
};

/// Order (parameter, point) pairs by parameter
bool parameter_less(const std::pair<double, Point>& a, const std::pair<double, Point>& b) {

    return a.first < b.first;
}

/// Same parameter of two (parameter, point) pairs
bool parameter_equal(const std::pair<double, Point>& a, const std::pair<double, Point>& b) {

    return a.first == b.first;
}

/// Order boundary samples (t, uv, point) by t
bool sample_less(const std::tuple<double, Point, Point>& a, const std::tuple<double, Point, Point>& b) {

    return std::get<0>(a) < std::get<0>(b);
}

/// Same t of two boundary samples
bool sample_equal(const std::tuple<double, Point, Point>& a, const std::tuple<double, Point, Point>& b) {

    return std::get<0>(a) == std::get<0>(b);
}

/// Order UV points by u, then v
bool uv_less(const Point& a, const Point& b) {

    return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
}

/// UV polygon of one wire of a face (pcurves sampled in traversal order)
std::vector<Point> wire_uv_points(const BRep& b, int face_index, const BRepRef& wire) {

    std::vector<Point> pts;

    for (const BRepRef& er : b.wire_edges(wire)) {
        const int ci = b.pcurve_index(er.index, face_index, er.orientation);

        if (ci < 0)
            continue;

        const NurbsCurve& crv = b.m_curves_2d[ci];
        std::vector<Point> seg;

        if (crv.degree() <= 1 && !crv.is_rational())
            for (int k = 0; k < crv.cv_count(); ++k)
                seg.push_back(crv.get_cv(k));
        else
            seg = crv.divide_by_count(std::max(crv.cv_count() * 4, 16), true).first;

        if (er.orientation == BRepOrientation::Reversed)
            std::reverse(seg.begin(), seg.end());

        for (size_t k = 0; k + 1 < seg.size(); ++k)
            pts.push_back(seg[k]);
    }

    return pts;
}

/// Distance from `point` to the surface point the pcurve reaches at t
double lifted_distance(const NurbsSurface& surface, const NurbsCurve& curve, const Point& point, double t) {

    const Point uv = curve.point_at(t);

    return surface.point_at(uv[0], uv[1]).distance(point);
}

/// Parameter of the lifted pcurve closest to `point`: a coarse scan then 64 golden-section steps in the best cell
double boundary_parameter(const NurbsSurface& surface, const NurbsCurve& curve, const Point& point) {

    const double start = curve.domain().first;
    const double end = curve.domain().second;
    const int count = std::clamp(curve.cv_count() * 4, 32, 4096);
    const double step = (end - start) / count;
    double best = start;
    double error = lifted_distance(surface, curve, point, start);

    for (int index = 1; index <= count; ++index) {
        const double t = index == count ? end : start + index * step;
        const double candidate = lifted_distance(surface, curve, point, t);

        if (candidate < error) {
            best = t;
            error = candidate;
        }
    }

    double left = std::max(best - step, start);
    double right = std::min(best + step, end);
    const double ratio = (std::sqrt(5.0) - 1.0) * 0.5;
    double a = right - ratio * (right - left);
    double b = left + ratio * (right - left);
    double da = lifted_distance(surface, curve, point, a);
    double db = lifted_distance(surface, curve, point, b);

    for (int i = 0; i < 64; ++i) {
        if (da < db) {
            right = b;
            b = a;
            db = da;
            a = right - ratio * (right - left);
            da = lifted_distance(surface, curve, point, a);
        } else {
            left = a;
            a = b;
            da = db;
            b = left + ratio * (right - left);
            db = lifted_distance(surface, curve, point, b);
        }
    }

    if (da < error) {
        best = a;
        error = da;
    }

    if (db < error)
        best = b;

    return best;
}

/// Unit normal on a boundary, taking the one-sided limit toward `toward` at a singular endpoint
std::optional<Vector> boundary_normal(const NurbsSurface& surface, const NurbsCurve& curve, double t, double toward) {

    for (const double at : {t, t + (toward - t) * 1e-6}) {
        const Point uv = curve.point_at(at);
        const std::vector<Vector> derivatives = surface.evaluate(uv[0], uv[1], 1);

        if (derivatives.size() < 3)
            continue;

        Vector n = derivatives[1].cross(derivatives[2]);
        const double scale = std::max({std::abs(n[0]), std::abs(n[1]), std::abs(n[2])});

        if (!std::isfinite(scale) || scale == 0.0)
            continue;

        n = n / scale;
        const double length = n.magnitude();

        if (std::isfinite(length) && length > 0.0)
            return n / length;
    }

    return std::nullopt;
}

/// True when the normals at ta, t and tb turn more than the angle whose cosine is given
bool boundary_turns(const NurbsSurface& surface, const NurbsCurve& curve, double ta, double t, double tb, double cosine) {

    const std::optional<Vector> normals[3] = {
        boundary_normal(surface, curve, ta, tb),
        boundary_normal(surface, curve, t, ta),
        boundary_normal(surface, curve, tb, ta),
    };

    for (int j = 0; j < 3; ++j)
        for (int k = j + 1; k < 3; ++k) {
            if (!normals[j] || !normals[k])
                continue;

            if (normals[j]->dot(*normals[k]) < cosine)
                return true;
        }

    return false;
}

/// Refine samples of a lifted pcurve until chord and angle hold; existing samples stay exact, eight split levels and 4096 added points per edge bound the work
std::vector<std::tuple<double, Point, Point>> refine_surface_boundary(
    const NurbsSurface& surface,
    const NurbsCurve& curve,
    const std::vector<std::tuple<double, Point, Point>>& samples,
    double angle,
    double chord
) {

    if (samples.size() < 2)
        return samples;

    const double tolerance = bbox_diagonal(surface) * chord;
    const double cosine = std::cos(std::clamp(angle, 0.1, 179.0) * Tolerance::PI / 180.0);
    std::vector<std::tuple<double, Point, Point>> result;
    int added = 0;

    for (size_t i = 1; i < samples.size(); ++i) {
        std::vector<std::tuple<std::tuple<double, Point, Point>, std::tuple<double, Point, Point>, int>> stack = {{samples[i - 1], samples[i], 0}};

        while (!stack.empty()) {
            std::tuple<double, Point, Point> a;
            std::tuple<double, Point, Point> b;
            int depth = 0;
            std::tie(a, b, depth) = stack.back();
            stack.pop_back();
            const double t = (std::get<0>(a) + std::get<0>(b)) * 0.5;
            const Point uv = curve.point_at(t);
            const Point point = surface.point_at(uv[0], uv[1]);
            const Point& pa = std::get<2>(a);
            const Point& pb = std::get<2>(b);
            const Point center((pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5, (pa[2] + pb[2]) * 0.5);
            const bool split = (point.distance(center) > tolerance || boundary_turns(surface, curve, std::get<0>(a), t, std::get<0>(b), cosine)) && depth < 8 && added < 4096;

            if (!split) {
                result.push_back(a);
                continue;
            }

            ++added;
            const std::tuple<double, Point, Point> middle{t, uv, point};
            stack.push_back({middle, b, depth + 1});
            stack.push_back({a, middle, depth + 1});
        }
    }

    result.push_back(samples.back());

    return result;
}

/// Compare canonical boundary positions exactly, without tolerance
bool same_boundary_point(const Point& a, const Point& b) {

    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

/// Phase 1: the outer wire is the full UV rectangle (straight pcurves enclosing the whole domain, no holes), so the face meshes directly on the surface grid
bool direct_face(const BRep& b, int fi) {

    const BRepFace& face = b.m_faces[fi];

    if (face.wires.size() != 1)
        return false;

    for (const BRepRef& er : b.wire_edges(face.wires[0])) {
        const int ci = b.pcurve_index(er.index, fi, er.orientation);

        if (ci < 0)
            continue;

        if (b.m_curves_2d[ci].degree() > 1 || b.m_curves_2d[ci].is_rational())
            return false;
    }

    const std::vector<Point> outer = wire_uv_points(b, fi, face.wires[0]);

    if (outer.size() < 3)
        return false;

    const NurbsSurface& srf = b.m_surfaces[face.surface_index];
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);

    for (const BRepRef& er : b.wire_edges(face.wires[0])) {
        const int ci = b.pcurve_index(er.index, fi, er.orientation);

        if (ci < 0)
            continue;

        const NurbsCurve& curve = b.m_curves_2d[ci];

        for (int k : {0, std::max(0, curve.cv_count() - 1)}) {
            const Point p = curve.get_cv(k);
            const bool corner_u = std::min(std::abs(p[0] - du.first), std::abs(p[0] - du.second)) <= (du.second - du.first) * 1e-9;
            const bool corner_v = std::min(std::abs(p[1] - dv.first), std::abs(p[1] - dv.second)) <= (dv.second - dv.first) * 1e-9;

            if (!corner_u || !corner_v)
                return false;
        }
    }

    const double domain_area = (du.second - du.first) * (dv.second - dv.first);

    return std::abs(std::abs(polygon_signed_area(outer)) - domain_area) < 1e-3 * domain_area;
}

/// Grid vertices on the domain sides flagged by (at_v0, at_v1, at_u0, at_u1), as (parameter along the side, model point) sorted and unique
std::vector<std::pair<double, Point>> grid_side_points(
    const Mesh& grid,
    const NurbsSurface& srf,
    bool at_v0,
    bool at_v1,
    bool at_u0,
    bool at_u1
) {

    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const double utol = (du.second - du.first) * 0.001;
    const double vtol = (dv.second - dv.first) * 0.001;
    std::vector<std::pair<double, Point>> pts;

    for (const std::pair<const size_t, VertexData>& entry : grid.vertex) {
        const VertexData& vd = entry.second;
        const auto iu = vd.attributes.find("u");
        const auto iv = vd.attributes.find("v");

        if (iu == vd.attributes.end() || iv == vd.attributes.end())
            continue;

        if ((at_v0 && std::abs(iv->second - dv.first) < vtol * 0.1) || (at_v1 && std::abs(iv->second - dv.second) < vtol * 0.1))
            pts.push_back({iu->second, vd.position()});
        else if ((at_u0 && std::abs(iu->second - du.first) < utol * 0.1) || (at_u1 && std::abs(iu->second - du.second) < utol * 0.1))
            pts.push_back({iv->second, vd.position()});
    }

    std::sort(pts.begin(), pts.end(), parameter_less);
    pts.erase(std::unique(pts.begin(), pts.end(), parameter_equal), pts.end());

    return pts;
}

/// Phase 2: grid vertices of a direct face along a shared edge that runs on a domain side, as (pcurve parameter, model point) sorted along the edge; empty elsewhere
std::vector<std::pair<double, Point>> grid_edge_samples(const BRep& b, int fi, const Mesh& grid, const BRepRef& er) {

    std::vector<std::pair<double, Point>> samples;
    bool shared = false;

    for (const BRepRef& fr : b.edge_faces(er.index))
        if (fr.index != fi)
            shared = true;

    if (!shared)
        return samples;

    const int ci = b.pcurve_index(er.index, fi, er.orientation);

    if (ci < 0)
        return samples;

    const NurbsSurface& srf = b.m_surfaces[b.m_faces[fi].surface_index];
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const double utol = (du.second - du.first) * 0.001;
    const double vtol = (dv.second - dv.first) * 0.001;
    const NurbsCurve& c2d = b.m_curves_2d[ci];
    const Point sp = c2d.get_cv(0);
    const Point ep = c2d.get_cv(c2d.cv_count() - 1);
    const bool at_v0 = std::abs(sp[1] - dv.first) < vtol && std::abs(ep[1] - dv.first) < vtol;
    const bool at_v1 = std::abs(sp[1] - dv.second) < vtol && std::abs(ep[1] - dv.second) < vtol;
    const bool at_u0 = std::abs(sp[0] - du.first) < utol && std::abs(ep[0] - du.first) < utol;
    const bool at_u1 = std::abs(sp[0] - du.second) < utol && std::abs(ep[0] - du.second) < utol;

    if (!at_v0 && !at_v1 && !at_u0 && !at_u1)
        return samples;

    const std::vector<std::pair<double, Point>> pts = grid_side_points(grid, srf, at_v0, at_v1, at_u0, at_u1);

    if (pts.size() < 2)
        return samples;

    const int varying = (at_v0 || at_v1) ? 0 : 1;
    const double t0 = c2d.domain().first;
    const double t1 = c2d.domain().second;

    for (const std::pair<double, Point>& pt : pts)
        samples.push_back({t0 + (pt.first - sp[varying]) / (ep[varying] - sp[varying]) * (t1 - t0), pt.second});

    return samples;
}

/// Phase 2: the first incident grid supplies the canonical polygon of every shared edge; true when this face's grid disagrees with an earlier one and must be rebuilt
bool grid_boundaries(const BRep& b, int fi, const Mesh& grid, EdgeBoundary& boundary) {

    bool rebuild = false;

    for (const BRepRef& er : b.wire_edges(b.m_faces[fi].wires[0])) {
        const int eidx = er.index;
        const std::vector<std::pair<double, Point>> samples = grid_edge_samples(b, fi, grid, er);

        if (samples.empty())
            continue;

        std::vector<double> parameters;
        std::vector<Point> bnd;

        for (const std::pair<double, Point>& sample : samples) {
            parameters.push_back(sample.first);
            bnd.push_back(sample.second);
        }

        if (!boundary.points.count(eidx)) {
            boundary.points[eidx] = bnd;
            boundary.basis[eidx] = {fi, b.pcurve_index(eidx, fi, er.orientation), parameters};
            continue;
        }

        const std::vector<Point>& canonical = boundary.points.at(eidx);
        const bool matches = canonical.size() == bnd.size()
            && (std::equal(canonical.begin(), canonical.end(), bnd.begin(), same_boundary_point)
                || std::equal(canonical.begin(), canonical.end(), bnd.rbegin(), same_boundary_point));
        rebuild = rebuild || !matches;
    }

    return rebuild;
}

/// Refine the canonical polygon of every edge shared with a curved CDT face, then mark every incident face for rebuild with the same refined polygon
void refine_shared_boundaries(
    const BRep& b,
    const std::vector<bool>& face_direct,
    std::vector<bool>& rebuild_grid,
    EdgeBoundary& boundary,
    double angle,
    double chord
) {

    for (const std::pair<const int, std::tuple<int, int, std::vector<double>>>& entry : boundary.basis) {
        const int edge = entry.first;
        const std::vector<double>& parameters = std::get<2>(entry.second);
        bool curved_cdt = false;

        for (const BRepRef& incident : b.edge_faces(edge)) {
            const int fi = incident.index;
            const bool cdt = !face_direct[fi] || rebuild_grid[fi];
            curved_cdt = curved_cdt || (cdt && !b.m_surfaces[b.m_faces[fi].surface_index].is_planar(nullptr, 0.0));
        }

        if (!curved_cdt)
            continue;

        const NurbsSurface& surface = b.m_surfaces[b.m_faces[std::get<0>(entry.second)].surface_index];
        const NurbsCurve& curve = b.m_curves_2d[std::get<1>(entry.second)];
        const std::vector<Point>& points = boundary.points.at(edge);
        std::vector<std::tuple<double, Point, Point>> samples;

        for (size_t i = 0; i < parameters.size(); ++i)
            samples.emplace_back(parameters[i], curve.point_at(parameters[i]), points[i]);

        std::sort(samples.begin(), samples.end(), sample_less);
        const double end = curve.domain().second;

        if (b.m_edges[edge].start_vertex == b.m_edges[edge].end_vertex && !samples.empty() && std::get<0>(samples.back()) < end)
            samples.emplace_back(end, curve.point_at(end), std::get<2>(samples.front()));

        const std::vector<std::tuple<double, Point, Point>> refined = refine_surface_boundary(surface, curve, samples, angle, chord);

        if (refined.size() <= samples.size())
            continue;

        std::vector<Point> refined_points;
        std::vector<std::pair<double, Point>> refined_samples;

        for (const std::tuple<double, Point, Point>& sample : refined) {
            refined_points.push_back(std::get<2>(sample));
            refined_samples.emplace_back(std::get<0>(sample), std::get<1>(sample));
        }

        boundary.points[edge] = refined_points;
        boundary.samples[edge] = refined_samples;

        for (const BRepRef& incident : b.edge_faces(edge))
            rebuild_grid[incident.index] = true;
    }
}

/// Interior UV seeds of a rebuilt face: its grid vertices strictly inside the domain
std::vector<Point> grid_interior_uv(const NurbsSurface& srf, const Mesh& grid) {

    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    std::vector<Point> seeds;

    for (const std::pair<const size_t, VertexData>& entry : grid.vertex) {
        const auto u = entry.second.attributes.find("u");
        const auto v = entry.second.attributes.find("v");

        if (u == entry.second.attributes.end() || v == entry.second.attributes.end())
            continue;

        if (u->second > du.first && u->second < du.second && v->second > dv.first && v->second < dv.second)
            seeds.push_back(Point(u->second, v->second, 0.0));
    }

    std::sort(seeds.begin(), seeds.end(), uv_less);

    return seeds;
}

/// Planarity tolerance for a surface of any size: 1e-9 of its control-point bounding box diagonal, never below the zero tolerance
double planar_patch_tolerance(const NurbsSurface& srf) {

    double lo[3] = {1e300, 1e300, 1e300};
    double hi[3] = {-1e300, -1e300, -1e300};

    for (int i = 0; i < srf.cv_count(0); ++i)
        for (int j = 0; j < srf.cv_count(1); ++j) {
            const Point p = srf.get_cv(i, j);

            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }

    const double diagonal = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) + (hi[2] - lo[2]) * (hi[2] - lo[2]));

    return std::max(1e-9 * diagonal, Tolerance::ZERO_TOLERANCE);
}

/// True for a surface flat within planar_patch_tolerance, whatever its coordinates
bool is_planar_patch(const NurbsSurface& srf) {

    return srf.is_planar(nullptr, planar_patch_tolerance(srf));
}

/// Surface parameters of a point on a degree-1 parallelogram patch by two dot products; false when the patch is not that shape
bool planar_patch_uv(const NurbsSurface& srf, const Point& p, double& u, double& v) {

    if (srf.degree(0) != 1 || srf.degree(1) != 1 || srf.cv_count(0) != 2 || srf.cv_count(1) != 2)
        return false;

    const Point p00 = srf.get_cv(0, 0);
    const Point p10 = srf.get_cv(1, 0);
    const Point p01 = srf.get_cv(0, 1);
    const Point p11 = srf.get_cv(1, 1);
    const Vector eu = p10 - p00;
    const Vector ev = p01 - p00;
    const Vector skew = (p11 - p10) - ev;

    if (skew.magnitude() > planar_patch_tolerance(srf))
        return false;

    const double eu2 = eu.dot(eu);
    const double ev2 = ev.dot(ev);

    if (eu2 <= 0.0 || ev2 <= 0.0)
        return false;

    const Vector d = p - p00;
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    u = du.first + d.dot(eu) / eu2 * (du.second - du.first);
    v = dv.first + d.dot(ev) / ev2 * (dv.second - dv.first);

    return true;
}

/// Parameter of the closest point on a two-point degree-1 pcurve by one projection; false for any other curve
bool linear_pcurve_parameter(const NurbsCurve& crv, double u, double v, double& t) {

    if (crv.degree() != 1 || crv.is_rational() || crv.cv_count() != 2)
        return false;

    const Point c0 = crv.get_cv(0);
    const Point c1 = crv.get_cv(1);
    const double dx = c1[0] - c0[0];
    const double dy = c1[1] - c0[1];
    const double length_squared = dx * dx + dy * dy;

    if (length_squared <= 0.0)
        return false;

    const double fraction = std::clamp(((u - c0[0]) * dx + (v - c0[1]) * dy) / length_squared, 0.0, 1.0);
    const std::pair<double, double> domain = crv.domain();
    t = domain.first + fraction * (domain.second - domain.first);

    return true;
}

/// Phase 3: map the canonical points of edge `ei` onto pcurve `ci` of face `fi`, checked in model space; false when a point cannot be lifted
bool lift_canonical(
    const BRep& b,
    int fi,
    int ei,
    int ci,
    EdgeBoundary& boundary,
    std::vector<std::tuple<double, Point, Point>>& samples
) {

    const BRepFace& face = b.m_faces[fi];
    const BRepEdge& edge = b.m_edges[ei];
    const NurbsSurface& srf = b.m_surfaces[face.surface_index];
    const NurbsCurve& crv = b.m_curves_2d[ci];
    const bool cached = boundary.basis.count(ei) && std::get<0>(boundary.basis[ei]) == fi && std::get<1>(boundary.basis[ei]) == ci && boundary.samples.count(ei);
    const std::vector<Point>& points = boundary.points[ei];
    const bool planar = is_planar_patch(srf);

    for (size_t index = 0; index < points.size(); ++index) {
        const Point& p = points[index];
        double t = 0.0;
        Point q;

        if (cached) {
            t = boundary.samples[ei][index].first;
            q = boundary.samples[ei][index].second;
        } else {
            double u = 0.0;
            double v = 0.0;
            const bool on_patch = planar && planar_patch_uv(srf, p, u, v);

            if (!on_patch)
                std::tie(u, v) = srf.closest_parameters(p);

            if (!on_patch || !linear_pcurve_parameter(crv, u, v, t))
                t = crv.closest_parameter(Point(u, v, 0.0));

            q = crv.point_at(t);
        }

        const double scale = std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2]), 1.0});
        const double tolerance = std::max({edge.tolerance, face.tolerance, std::sqrt(std::numeric_limits<double>::epsilon()) * scale});

        if (srf.point_at(q[0], q[1]).distance(p) > tolerance) {
            t = boundary_parameter(srf, crv, p);
            q = crv.point_at(t);

            if (srf.point_at(q[0], q[1]).distance(p) > tolerance)
                return false;
        }

        samples.push_back({t, q, p});
    }

    std::sort(samples.begin(), samples.end(), sample_less);
    samples.erase(std::unique(samples.begin(), samples.end(), sample_equal), samples.end());

    return true;
}

/// Phase 3: fresh samples of a pcurve nobody has sampled yet, refined to the face's angle and chord
std::vector<std::tuple<double, Point, Point>> fresh_samples(const NurbsSurface& srf, const NurbsCurve& crv, double angle, double chord) {

    std::vector<Point> points;
    std::vector<double> parameters;

    if (crv.degree() <= 1 && !crv.is_rational() && is_planar_patch(srf)) {
        for (int k = 0; k < crv.cv_count(); ++k) {
            points.push_back(crv.get_cv(k));
            parameters.push_back(crv.greville_abcissa(k));
        }
    } else {
        const int count = std::min(std::max(crv.cv_count() * 4, (int)std::ceil(360.0 / std::max(angle, 0.1))), 4096);
        std::tie(points, parameters) = crv.divide_by_count(count, true);
    }

    std::vector<std::tuple<double, Point, Point>> samples;

    for (size_t k = 0; k < points.size(); ++k) {
        const Point& q = points[k];
        samples.push_back({parameters[k], q, srf.point_at(q[0], q[1])});
    }

    return refine_surface_boundary(srf, crv, samples, angle, chord);
}

/// Phase 3: samples of one edge use of a CDT face in traversal direction, a closed edge repeating its first point at the end; false when the edge has no pcurve or cannot be lifted
bool edge_use_samples(
    const BRep& b,
    int fi,
    const BRepRef& er,
    EdgeBoundary& boundary,
    double angle,
    double chord,
    std::vector<std::tuple<double, Point, Point>>& samples
) {

    const int ei = er.index;
    const BRepEdge& edge = b.m_edges[ei];
    const int ci = b.pcurve_index(ei, fi, er.orientation);

    if (ci < 0)
        return false;

    const NurbsCurve& crv = b.m_curves_2d[ci];

    if (boundary.points.count(ei)) {
        if (!lift_canonical(b, fi, ei, ci, boundary, samples))
            return false;
    } else {
        samples = fresh_samples(b.m_surfaces[b.m_faces[fi].surface_index], crv, angle, chord);
        std::vector<Point> positions;

        for (const std::tuple<double, Point, Point>& sample : samples)
            positions.push_back(std::get<2>(sample));

        boundary.points[ei] = positions;
    }

    if (edge.start_vertex == edge.end_vertex && samples.size() > 1) {
        const std::tuple<double, Point, Point> first = samples.front();

        if (!same_boundary_point(std::get<2>(first), std::get<2>(samples.back())))
            samples.push_back({crv.domain().second, std::get<1>(first), std::get<2>(first)});
    }

    if (er.orientation == BRepOrientation::Reversed)
        std::reverse(samples.begin(), samples.end());

    return samples.size() >= 2;
}

/// Phase 3: trim loops of a CDT face, every edge use keeping its boundary-node identities; false when some edge cannot be sampled
bool trim_loops(
    const BRep& b,
    int fi,
    EdgeBoundary& boundary,
    double angle,
    double chord,
    TrimLoops& loops,
    std::vector<std::tuple<int, size_t, size_t, size_t>>& uses
) {

    const BRepFace& face = b.m_faces[fi];

    for (size_t wi = 0; wi < face.wires.size(); ++wi) {
        std::vector<Point> uv;
        std::vector<Point> xyz;

        for (const BRepRef& er : b.wire_edges(face.wires[wi])) {
            std::vector<std::tuple<double, Point, Point>> samples;

            if (!edge_use_samples(b, fi, er, boundary, angle, chord, samples))
                return false;

            uses.push_back({er.index, wi, uv.size(), samples.size()});

            for (size_t k = 0; k + 1 < samples.size(); ++k) {
                uv.push_back(std::get<1>(samples[k]));
                xyz.push_back(std::get<2>(samples[k]));
            }
        }

        loops.uv.push_back(uv);
        loops.xyz.push_back(xyz);
    }

    return true;
}

/// Plane coordinates of the points in the frame (origin, xaxis, yaxis)
std::vector<std::pair<double, double>> plane_coordinates(const std::vector<Point>& pts, const Point& origin, const Vector& xaxis, const Vector& yaxis) {

    std::vector<std::pair<double, double>> out;
    out.reserve(pts.size());

    for (const Point& p : pts) {
        const Vector d = p - origin;
        out.emplace_back(d.dot(xaxis), d.dot(yaxis));
    }

    return out;
}

/// Signed area of a 2D ring, positive when counter-clockwise
double ring_signed_area(const std::vector<std::pair<double, double>>& pts) {

    double area = 0.0;
    const size_t n = pts.size();

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        area += pts[i].first * pts[j].second - pts[j].first * pts[i].second;
    }

    return area * 0.5;
}

/// Set every vertex normal and tag every loop vertex boundary/{loop}/{sample} as mesh_loops does
void tag_loop_vertices(Mesh& mesh, const TrimLoops& loops, const Vector& normal) {

    std::map<std::array<double, 3>, std::pair<size_t, size_t>> lookup;

    for (size_t li = 0; li < loops.xyz.size(); ++li)
        for (size_t k = 0; k < loops.xyz[li].size(); ++k) {
            const Point& p = loops.xyz[li][k];
            lookup.emplace(std::array<double, 3>{p[0], p[1], p[2]}, std::make_pair(li, k));
        }

    for (std::pair<const size_t, VertexData>& entry : mesh.vertex) {
        VertexData& vd = entry.second;
        vd.set_normal(normal[0], normal[1], normal[2]);
        const Point position = vd.position();
        const auto hit = lookup.find(std::array<double, 3>{position[0], position[1], position[2]});

        if (hit != lookup.end())
            vd.attributes[fmt::format("boundary/{}/{}", hit->second.first, hit->second.second)] = 1.0;
    }
}

/// Phase 3 for a planar face: the sampled loops triangulated as one polygon with holes, wound to the surface normal, every loop vertex tagged boundary/{loop}/{sample} as mesh_loops does; no grid, no surface evaluation
Mesh planar_loops_mesh(const NurbsSurface& srf, const TrimLoops& loops) {

    Mesh mesh;

    if (loops.xyz.empty() || loops.xyz[0].size() < 3)
        return mesh;

    std::vector<Point> all_pts;

    for (const std::vector<Point>& loop : loops.xyz)
        all_pts.insert(all_pts.end(), loop.begin(), loop.end());

    Point origin;
    Vector xaxis;
    Vector yaxis;
    Vector zaxis;
    Polyline(all_pts).get_average_plane(origin, xaxis, yaxis, zaxis);
    std::vector<Point> border = loops.xyz[0];
    std::vector<std::pair<double, double>> border_2d = plane_coordinates(border, origin, xaxis, yaxis);

    if (ring_signed_area(border_2d) < 0.0) {
        std::reverse(border.begin(), border.end());
        std::reverse(border_2d.begin(), border_2d.end());
    }

    std::vector<std::vector<Point>> holes;
    std::vector<std::vector<std::pair<double, double>>> holes_2d;

    for (size_t li = 1; li < loops.xyz.size(); ++li) {
        if (loops.xyz[li].size() < 3)
            continue;

        std::vector<Point> hole = loops.xyz[li];
        std::vector<std::pair<double, double>> hole_2d = plane_coordinates(hole, origin, xaxis, yaxis);

        if (ring_signed_area(hole_2d) > 0.0) {
            std::reverse(hole.begin(), hole.end());
            std::reverse(hole_2d.begin(), hole_2d.end());
        }

        holes.push_back(hole);
        holes_2d.push_back(hole_2d);
    }

    std::vector<size_t> vkeys;

    for (const Point& p : border)
        vkeys.push_back(mesh.add_vertex(p));

    for (const std::vector<Point>& hole : holes)
        for (const Point& p : hole)
            vkeys.push_back(mesh.add_vertex(p));

    for (const std::array<int, 3>& t : cdt_triangulate(border_2d, holes_2d))
        if (t[0] != t[1] && t[1] != t[2] && t[2] != t[0])
            mesh.add_face({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});

    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const Vector normal = srf.normal_at(0.5 * (du.first + du.second), 0.5 * (dv.first + dv.second));

    if (!mesh.face.empty()) {
        const std::vector<size_t>& fverts = mesh.face.begin()->second;
        const Point a = mesh.vertex[fverts[0]].position();
        const Point b = mesh.vertex[fverts[1]].position();
        const Point c = mesh.vertex[fverts[2]].position();

        if ((b - a).cross(c - a).dot(normal) < 0.0)
            mesh.flip();
    }

    tag_loop_vertices(mesh, loops, normal);

    return mesh;
}

/// Tag every boundary vertex of a CDT mesh with the edge use it samples; each use keeps both ends, including the next edge's start
void tag_edge_uses(Mesh& mesh, const TrimLoops& loops, const std::vector<std::tuple<int, size_t, size_t, size_t>>& uses) {

    for (size_t use_id = 0; use_id < uses.size(); ++use_id) {
        int edge = 0;
        size_t li = 0;
        size_t start = 0;
        size_t count = 0;
        std::tie(edge, li, start, count) = uses[use_id];
        const size_t length = loops.uv[li].size();

        if (length == 0)
            continue;

        for (size_t sample = 0; sample < count; ++sample) {
            const std::string key = fmt::format("boundary/{}/{}", li, (start + sample) % length);
            const std::string tag = fmt::format("brep_edge/{}/{}/{}", edge, use_id, sample);

            for (std::pair<const size_t, VertexData>& entry : mesh.vertex)
                if (entry.second.attributes.count(key))
                    entry.second.attributes[tag] = 1.0;

            if (sample + 1 >= count)
                continue;

            const std::string interval = fmt::format("boundary_interval/{}/{}", li, (start + sample) % length);
            const std::string interval_tag = fmt::format("brep_edge_interval/{}/{}/{}", edge, use_id, sample);

            for (std::pair<const size_t, VertexData>& entry : mesh.vertex)
                if (entry.second.attributes.count(interval))
                    entry.second.attributes[interval_tag] = entry.second.attributes[interval];
        }
    }
}

/// Flip every face mesh of a face Reversed in its shell, vertex normals included
void flip_reversed_faces(const BRep& b, std::vector<Mesh>& fmesh) {

    for (int fi = 0; fi < (int)fmesh.size(); ++fi) {
        if (b.face_orientation(fi) != BRepOrientation::Reversed)
            continue;

        fmesh[fi].flip();

        for (std::pair<const size_t, VertexData>& entry : fmesh[fi].vertex) {
            const std::optional<std::array<double, 3>> n = entry.second.normal();

            if (n)
                entry.second.set_normal(-(*n)[0], -(*n)[1], -(*n)[2]);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Cutting helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Vertex rings of every face in one mesh keyed by BRep vertex index, outer rings wound to the face normal, holes as face holes; false when some face or edge is curved
bool face_rings(const BRep& b, Mesh& rings) {

    bool planar = true;

    for (int vi = 0; vi < b.vertex_count(); ++vi)
        rings.add_vertex(b.m_vertices[vi].point, vi);

    for (int fi = 0; fi < b.face_count(); ++fi) {
        const NurbsSurface& surface = b.m_surfaces[b.m_faces[fi].surface_index];
        const std::pair<double, double> u = surface.domain(0);
        const std::pair<double, double> v = surface.domain(1);
        const Plane frame = Plane::from_point_normal(Point(), b.normal_at(fi, (u.first + u.second) * 0.5, (v.first + v.second) * 0.5));
        std::vector<std::vector<size_t>> loops;
        std::vector<Point> outer;
        planar = planar && is_planar_patch(surface);

        for (const BRepRef& wire : b.m_faces[fi].wires) {
            loops.emplace_back();

            for (const BRepRef& er : b.wire_edges(wire)) {
                const BRepEdge& edge = b.m_edges[er.index];

                if (edge.degenerated)
                    continue;

                planar = planar && b.m_curves_3d[edge.curve_3d_index].degree() == 1;
                loops.back().push_back(er.orientation == BRepOrientation::Reversed ? edge.end_vertex : edge.start_vertex);
            }
        }

        for (size_t vi : loops[0])
            outer.push_back(b.m_vertices[vi].point);

        if (signed_area_in_plane(outer, frame.origin(), frame.x_axis(), frame.y_axis()) < 0.0)
            std::reverse(loops[0].begin(), loops[0].end());

        const std::optional<size_t> fk = rings.add_face(loops[0]);

        if (fk && loops.size() > 1)
            rings.set_face_holes(*fk, {loops.begin() + 1, loops.end()});
    }

    return planar;
}

/// Closed polyline through the positions of a vertex ring of `mesh`
Polyline ring_polyline(const Mesh& mesh, const std::vector<size_t>& ring) {

    std::vector<Point> points;

    for (size_t vk : ring)
        points.push_back(mesh.vertex.at(vk).position());

    points.push_back(points.front());

    return Polyline(points);
}

// ═══════════════════════════════════════════════════════════════════════════
// Serialization helpers
// ═══════════════════════════════════════════════════════════════════════════
/// JSON array of oriented references
nlohmann::ordered_json refs_to_json(const std::vector<BRepRef>& refs) {

    nlohmann::ordered_json arr = nlohmann::ordered_json::array();

    for (const BRepRef& r : refs) {
        nlohmann::ordered_json rj;
        rj["index"] = r.index;
        rj["orientation"] = orientation_to_str(r.orientation);
        arr.push_back(rj);
    }

    return arr;
}

/// Oriented references of a JSON array
std::vector<BRepRef> refs_from_json(const nlohmann::json& arr) {

    std::vector<BRepRef> refs;

    for (const nlohmann::json& r : arr)
        refs.push_back({r["index"], orientation_from_str(r["orientation"])});

    return refs;
}

/// JSON object of an edge, fields in alphabetical order
nlohmann::ordered_json edge_to_json(const BRepEdge& e) {

    nlohmann::ordered_json ej;
    ej["curve_3d_index"] = e.curve_3d_index;
    ej["degenerated"] = e.degenerated;
    ej["end_vertex"] = e.end_vertex;
    ej["pcurves"] = nlohmann::ordered_json::array();

    for (const BRepCurveOnSurface& pc : e.pcurves) {
        nlohmann::ordered_json pj;
        pj["curve_2d_index"] = pc.curve_2d_index;
        pj["curve_2d_index_2"] = pc.curve_2d_index_2;
        pj["surface_index"] = pc.surface_index;
        ej["pcurves"].push_back(pj);
    }

    ej["start_vertex"] = e.start_vertex;
    ej["tolerance"] = e.tolerance;

    return ej;
}

/// Edge of a JSON object
BRepEdge edge_from_json(const nlohmann::json& e) {

    BRepEdge be;
    be.curve_3d_index = e["curve_3d_index"];
    be.degenerated = e["degenerated"];
    be.end_vertex = e["end_vertex"];

    for (const nlohmann::json& pc : e["pcurves"])
        be.pcurves.push_back({pc["surface_index"], pc["curve_2d_index"], pc["curve_2d_index_2"]});

    be.start_vertex = e["start_vertex"];
    be.tolerance = e["tolerance"];

    return be;
}

/// JSON object of a face, facecolor only when set
nlohmann::ordered_json face_to_json(const BRepFace& f) {

    nlohmann::ordered_json fj;

    if (f.facecolor.a > 0)
        fj["facecolor"] = f.facecolor.jsondump();

    fj["surface_index"] = f.surface_index;
    fj["tolerance"] = f.tolerance;
    fj["wires"] = refs_to_json(f.wires);

    return fj;
}

/// Face of a JSON object
BRepFace face_from_json(const nlohmann::json& f) {

    BRepFace bf;

    if (f.contains("facecolor"))
        bf.facecolor = Color::jsonload(f["facecolor"]);

    bf.surface_index = f["surface_index"];
    bf.tolerance = f["tolerance"];
    bf.wires = refs_from_json(f["wires"]);

    return bf;
}

/// Append oriented references to a repeated proto field
void refs_to_proto(const std::vector<BRepRef>& refs, google::protobuf::RepeatedPtrField<session_proto::BRepRef>* out) {

    for (const BRepRef& r : refs) {
        session_proto::BRepRef* p = out->Add();
        p->set_index(r.index);
        p->set_orientation(static_cast<session_proto::BRepOrientation>(r.orientation));
    }
}

/// Oriented references of a repeated proto field
std::vector<BRepRef> refs_from_proto(const google::protobuf::RepeatedPtrField<session_proto::BRepRef>& in) {

    std::vector<BRepRef> refs;

    for (const session_proto::BRepRef& r : in)
        refs.push_back({r.index(), static_cast<BRepOrientation>(r.orientation())});

    return refs;
}

/// Fill a proto edge
void edge_to_proto(const BRepEdge& e, session_proto::BRepEdge* p) {

    p->set_curve_3d_index(e.curve_3d_index);
    p->set_start_vertex(e.start_vertex);
    p->set_end_vertex(e.end_vertex);
    p->set_tolerance(e.tolerance);
    p->set_degenerated(e.degenerated);

    for (const BRepCurveOnSurface& pc : e.pcurves) {
        session_proto::BRepCurveOnSurface* q = p->add_pcurves();
        q->set_surface_index(pc.surface_index);
        q->set_curve_2d_index(pc.curve_2d_index);
        q->set_curve_2d_index_2(pc.curve_2d_index_2);
    }
}

/// Edge of a proto edge
BRepEdge edge_from_proto(const session_proto::BRepEdge& e) {

    BRepEdge be;
    be.curve_3d_index = e.curve_3d_index();
    be.start_vertex = e.start_vertex();
    be.end_vertex = e.end_vertex();
    be.tolerance = e.tolerance();
    be.degenerated = e.degenerated();

    for (const session_proto::BRepCurveOnSurface& pc : e.pcurves())
        be.pcurves.push_back({pc.surface_index(), pc.curve_2d_index(), pc.curve_2d_index_2()});

    return be;
}

/// Fill a proto face, facecolor only when set
void face_to_proto(const BRepFace& f, session_proto::BRepFace* p) {

    p->set_surface_index(f.surface_index);
    refs_to_proto(f.wires, p->mutable_wires());
    p->set_tolerance(f.tolerance);

    if (f.facecolor.a > 0)
        *p->mutable_facecolor() = f.facecolor.to_proto();
}

/// Face of a proto face
BRepFace face_from_proto(const session_proto::BRepFace& f) {

    BRepFace bf;
    bf.surface_index = f.surface_index();
    bf.wires = refs_from_proto(f.wires());
    bf.tolerance = f.tolerance();

    if (f.has_facecolor())
        bf.facecolor = Color::from_proto(f.facecolor());

    return bf;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
BRep::BRep() {}

BRep::BRep(const BRep& other) { *this = other; }

BRep& BRep::operator=(const BRep& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    name = other.name;
    width = other.width;
    surfacecolor = other.surfacecolor;
    m_surfaces = other.m_surfaces;
    m_curves_3d = other.m_curves_3d;
    m_curves_2d = other.m_curves_2d;
    m_vertices = other.m_vertices;
    m_edges = other.m_edges;
    m_wires = other.m_wires;
    m_faces = other.m_faces;
    m_shells = other.m_shells;
    m_solids = other.m_solids;

    return *this;
}

BRep::~BRep() {}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
BRep BRep::create_box(double sx, double sy, double sz) {

    BRep b;
    b.name = "box";
    box_corners(b, sx, sy, sz);
    PolyFaceBuilder pb{b, {}};
    std::vector<BRepRef> faces;

    for (const std::vector<int>& fv : BOX_FACES)
        faces.push_back({pb.face(quad_patch(b, fv), fv), F});

    b.add_solid({{b.add_shell(faces), F}});

    return b;
}

BRep BRep::create_cylinder(double radius, double height) {

    BRep b;
    b.name = "cylinder";
    const NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, radius, height);
    const Point p_bot = body.point_at_corner(0, 0);
    const Point p_top = body.point_at_corner(0, 1);
    const int v_bot = b.add_vertex(p_bot);
    const int v_top = b.add_vertex(p_top);
    const int e_bot = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, 0, radius)), v_bot, v_bot);
    const int e_top = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, height, radius)), v_top, v_top);
    const int e_seam = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, {p_bot, p_top})), v_bot, v_top);
    const int f_body = body_face(b, b.add_surface(body), e_bot, e_seam, e_top);
    const int f_bot = cap_face(b, cap_patch(radius, 0, false), e_bot);
    const int f_top = cap_face(b, cap_patch(radius, height, true), e_top);
    b.add_solid({{b.add_shell({{f_body, F}, {f_bot, F}, {f_top, F}}), F}});

    return b;
}

BRep BRep::create_sphere(double radius) {

    BRep b;
    b.name = "sphere";
    const NurbsSurface srf = Primitives::sphere_surface(0, 0, 0, radius);
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const double u0 = du.first;
    const double u1 = du.second;
    const double v0 = dv.first;
    const double v1 = dv.second;
    const int v_s = b.add_vertex(Point(0, 0, -radius));
    const int v_n = b.add_vertex(Point(0, 0, radius));
    const int e_seam = b.add_edge(b.add_curve_3d(srf.iso_curve(1, u0)), v_s, v_n);
    const int e_south = b.add_edge(-1, v_s, v_s);
    const int e_north = b.add_edge(-1, v_n, v_n);
    const int si = b.add_surface(srf);
    b.add_pcurve(e_south, si, b.add_curve_2d(uv_line(u0, v0, u1, v0)));
    b.add_pcurve(e_north, si, b.add_curve_2d(uv_line(u0, v1, u1, v1)));
    b.add_pcurve(e_seam, si, b.add_curve_2d(uv_line(u1, v0, u1, v1)), b.add_curve_2d(uv_line(u0, v0, u0, v1)));
    const int fi = b.add_face(si, {{b.add_wire({{e_south, F}, {e_seam, F}, {e_north, R}, {e_seam, R}}), F}});
    b.add_solid({{b.add_shell({{fi, F}}), F}});

    return b;
}

BRep BRep::create_cone(double radius, double height) {

    BRep b;
    b.name = "cone";
    const NurbsSurface body = Primitives::cone_surface(0, 0, 0, radius, height);
    const Point p_base = body.point_at_corner(0, 0);
    const Point p_apex(0, 0, height);
    const int v_base = b.add_vertex(p_base);
    const int v_apex = b.add_vertex(p_apex);
    const int e_base = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, 0, radius)), v_base, v_base);
    const int e_seam = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, {p_base, p_apex})), v_base, v_apex);
    const int e_apex = b.add_edge(-1, v_apex, v_apex);
    const int f_body = body_face(b, b.add_surface(body), e_base, e_seam, e_apex);
    const int f_base = cap_face(b, cap_patch(radius, 0, false), e_base);
    b.add_solid({{b.add_shell({{f_body, F}, {f_base, F}}), F}});

    return b;
}

BRep BRep::create_pyramid(double base, double height) {

    BRep b;
    b.name = "pyramid";
    const double h = base * 0.5;
    b.add_vertex(Point(-h, -h, 0.0));
    b.add_vertex(Point(h, -h, 0.0));
    b.add_vertex(Point(h, h, 0.0));
    b.add_vertex(Point(-h, h, 0.0));
    const int v_apex = b.add_vertex(Point(0.0, 0.0, height));
    PolyFaceBuilder pb{b, {}};
    const std::vector<int> fv = {0, 3, 2, 1};
    std::vector<BRepRef> faces = {{pb.face(quad_patch(b, fv), fv), F}};

    for (int i = 0; i < 4; ++i) {
        const int a = i;
        const int c = (i + 1) % 4;
        const NurbsSurface srf = bilinear_patch(b.m_vertices[a].point, b.m_vertices[c].point, b.m_vertices[v_apex].point, b.m_vertices[v_apex].point);
        const int si = b.add_surface(srf);
        const int e_ac = pb.edge(a, c);
        const int e_c = pb.edge(c, v_apex);
        const int e_a = pb.edge(a, v_apex);
        const int e_deg = b.add_edge(-1, v_apex, v_apex);
        const bool ac_fwd = b.m_edges[e_ac].start_vertex == a;
        b.add_pcurve(e_ac, si, b.add_curve_2d(ac_fwd ? uv_line(0, 0, 1, 0) : uv_line(1, 0, 0, 0)));
        b.add_pcurve(e_c, si, b.add_curve_2d(uv_line(1, 0, 1, 1)));
        b.add_pcurve(e_deg, si, b.add_curve_2d(uv_line(1, 1, 0, 1)));
        b.add_pcurve(e_a, si, b.add_curve_2d(uv_line(0, 0, 0, 1)));
        const int wire = b.add_wire({{e_ac, ac_fwd ? F : R}, {e_c, F}, {e_deg, F}, {e_a, R}});
        faces.push_back({b.add_face(si, {{wire, F}}), F});
    }

    b.add_solid({{b.add_shell(faces), F}});

    return b;
}

BRep BRep::create_torus(double major_radius, double minor_radius) {

    BRep b;
    b.name = "torus";
    const NurbsSurface srf = Primitives::torus_surface(0, 0, 0, major_radius, minor_radius);
    const std::pair<double, double> du = srf.domain(0);
    const std::pair<double, double> dv = srf.domain(1);
    const double u0 = du.first;
    const double u1 = du.second;
    const double v0 = dv.first;
    const double v1 = dv.second;
    const int v = b.add_vertex(srf.point_at_corner(0, 0));
    const int e_u = b.add_edge(b.add_curve_3d(srf.iso_curve(1, u0)), v, v);
    const int e_v = b.add_edge(b.add_curve_3d(srf.iso_curve(0, v0)), v, v);
    const int si = b.add_surface(srf);
    b.add_pcurve(e_v, si, b.add_curve_2d(uv_line(u0, v0, u1, v0)), b.add_curve_2d(uv_line(u0, v1, u1, v1)));
    b.add_pcurve(e_u, si, b.add_curve_2d(uv_line(u1, v0, u1, v1)), b.add_curve_2d(uv_line(u0, v0, u0, v1)));
    const int fi = b.add_face(si, {{b.add_wire({{e_v, F}, {e_u, F}, {e_v, R}, {e_u, R}}), F}});
    b.add_solid({{b.add_shell({{fi, F}}), F}});

    return b;
}

BRep BRep::create_block_with_hole(double sx, double sy, double sz, double hole_radius) {

    BRep b;
    b.name = "block_with_hole";
    const double hz = sz * 0.5;
    box_corners(b, sx, sy, sz);
    PolyFaceBuilder pb{b, {}};
    std::vector<BRepRef> faces;

    for (int fi = 2; fi < 6; ++fi)
        faces.push_back({pb.face(quad_patch(b, BOX_FACES[fi]), BOX_FACES[fi]), F});

    const Point p_bot(hole_radius, 0, -hz);
    const Point p_top(hole_radius, 0, hz);
    const int v_bot = b.add_vertex(p_bot);
    const int v_top = b.add_vertex(p_top);
    const int e_bot = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, -hz, hole_radius)), v_bot, v_bot);
    const int e_top = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, hz, hole_radius)), v_top, v_top);
    const int e_seam = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, {p_bot, p_top})), v_bot, v_top);
    const NurbsSurface bore = Primitives::cylinder_surface(0, 0, -hz, hole_radius, sz);
    faces.push_back({body_face(b, b.add_surface(bore), e_bot, e_seam, e_top), R});

    for (int fi = 0; fi < 2; ++fi) {
        const std::vector<int>& fv = BOX_FACES[fi];
        const NurbsSurface cap = quad_patch(b, fv);
        const int si = b.add_surface(cap);
        const std::vector<BRepRef> outer = pb.wire_refs(si, fv);
        const int e_hole = fi == 0 ? e_bot : e_top;
        const NurbsCurve c2d = project_to_patch(b.m_curves_3d[b.m_edges[e_hole].curve_3d_index], cap);
        const BRepOrientation o = uv_signed_area(c2d) < 0.0 ? F : R;
        b.add_pcurve(e_hole, si, b.add_curve_2d(c2d));
        faces.push_back({b.add_face(si, {{b.add_wire(outer), F}, {b.add_wire({{e_hole, o}}), F}}), F});
    }

    b.add_solid({{b.add_shell(faces), F}});

    return b;
}

BRep BRep::from_polylines(const std::vector<Polyline>& polylines, const std::vector<std::vector<Polyline>>& holes) {

    BRep b;
    b.name = "polysurface";
    const double tol = 1e-6;
    PolyFaceBuilder pb{b, {}};

    for (size_t pi = 0; pi < polylines.size(); ++pi) {
        const std::vector<Point> pts = open_points(polylines[pi]);

        if (pts.size() < 3)
            continue;

        Point org;
        Plane plane;
        polylines[pi].get_fast_plane(org, plane);

        if (!plane.is_valid())
            continue;

        const Vector xa = plane.x_axis();
        const Vector ya = plane.y_axis();
        const double outer_area = signed_area_in_plane(pts, org, xa, ya);
        std::vector<int> vi;

        for (const Point& pt : pts)
            vi.push_back(find_or_add_vertex(b, pt, tol));

        std::vector<Point> all_pts = pts;
        std::vector<std::vector<int>> hole_cycles;

        if (pi < holes.size())
            for (const Polyline& h : holes[pi]) {
                std::vector<Point> hp = open_points(h);

                if (hp.size() < 3)
                    continue;

                if (signed_area_in_plane(hp, org, xa, ya) * outer_area > 0.0)
                    std::reverse(hp.begin(), hp.end());

                std::vector<int> cycle;

                for (const Point& pt : hp)
                    cycle.push_back(find_or_add_vertex(b, pt, tol));

                hole_cycles.push_back(cycle);
                all_pts.insert(all_pts.end(), hp.begin(), hp.end());
            }

        pb.face(planar_patch_through(all_pts, org, xa, ya), vi, hole_cycles);
    }

    close_free_faces(b);

    return b;
}

BRep BRep::from_nurbscurves(const std::vector<NurbsCurve>& curves, const std::vector<std::vector<NurbsCurve>>& holes) {

    BRep b;
    b.name = "polysurface";
    const double tol = 1e-6;

    for (int ci = 0; ci < (int)curves.size(); ++ci) {
        const NurbsCurve& crv = curves[ci];
        std::vector<Point> pts = cv_points(crv);

        if (pts.size() >= 2 && pts.front().distance(pts.back()) < tol)
            pts.pop_back();

        if (pts.size() < 3)
            continue;

        Point org;
        Plane plane;
        Polyline(pts).get_fast_plane(org, plane);

        if (!plane.is_valid())
            continue;

        if (ci < (int)holes.size())
            for (const NurbsCurve& h : holes[ci]) {
                const std::vector<Point> hp = cv_points(h);
                pts.insert(pts.end(), hp.begin(), hp.end());
            }

        const int si = b.add_surface(planar_patch_through(pts, org, plane.x_axis(), plane.y_axis()));
        std::vector<BRepRef> wires = {{curve_wire(b, crv, si, tol), F}};

        if (ci < (int)holes.size())
            for (const NurbsCurve& h : holes[ci])
                wires.push_back({curve_wire(b, h, si, tol), F});

        b.add_face(si, wires);
    }

    close_free_faces(b);

    return b;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool BRep::operator==(const BRep& other) const {

    return name == other.name && width == other.width && surfacecolor == other.surfacecolor
        && m_surfaces.size() == other.m_surfaces.size()
        && m_vertices.size() == other.m_vertices.size()
        && m_edges.size() == other.m_edges.size()
        && m_wires.size() == other.m_wires.size()
        && m_faces.size() == other.m_faces.size()
        && m_shells.size() == other.m_shells.size()
        && m_solids.size() == other.m_solids.size();
}

bool BRep::operator!=(const BRep& other) const { return !(*this == other); }

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
int BRep::vertex_count() const { return (int)m_vertices.size(); }

int BRep::edge_count() const { return (int)m_edges.size(); }

int BRep::wire_count() const { return (int)m_wires.size(); }

int BRep::face_count() const { return (int)m_faces.size(); }

int BRep::shell_count() const { return (int)m_shells.size(); }

int BRep::solid_count() const { return (int)m_solids.size(); }

bool BRep::is_valid() const {

    if (m_faces.empty())
        return false;

    for (const BRepEdge& e : m_edges) {
        if (!in_range(e.start_vertex, m_vertices.size()) || !in_range(e.end_vertex, m_vertices.size()))
            return false;

        if (!e.degenerated && !in_range(e.curve_3d_index, m_curves_3d.size()))
            return false;

        for (const BRepCurveOnSurface& pc : e.pcurves) {
            if (!in_range(pc.surface_index, m_surfaces.size()) || !in_range(pc.curve_2d_index, m_curves_2d.size()))
                return false;

            if (pc.curve_2d_index_2 >= 0 && !in_range(pc.curve_2d_index_2, m_curves_2d.size()))
                return false;
        }
    }

    for (const BRepWire& w : m_wires) {
        if (w.edges.empty())
            return false;

        for (const BRepRef& r : w.edges)
            if (!in_range(r.index, m_edges.size()))
                return false;
    }

    for (const BRepFace& f : m_faces) {
        if (!in_range(f.surface_index, m_surfaces.size()) || f.wires.empty())
            return false;

        for (const BRepRef& r : f.wires)
            if (!in_range(r.index, m_wires.size()))
                return false;
    }

    for (const BRepShell& s : m_shells)
        for (const BRepRef& r : s.faces)
            if (!in_range(r.index, m_faces.size()))
                return false;

    for (const BRepSolid& s : m_solids)
        for (const BRepRef& r : s.shells)
            if (!in_range(r.index, m_shells.size()))
                return false;

    return true;
}

bool BRep::is_closed(int shell_index) const {

    if (shell_index < 0 || shell_index >= (int)m_shells.size())
        return false;

    std::vector<int> uses(m_edges.size(), 0);

    for (const BRepRef& fr : m_shells[shell_index].faces)
        for (const BRepRef& wr : m_faces[fr.index].wires)
            for (const BRepRef& er : wire_edges(wr))
                ++uses[er.index];

    for (size_t i = 0; i < m_edges.size(); ++i)
        if (!m_edges[i].degenerated && uses[i] != 0 && uses[i] != 2)
            return false;

    return !m_shells[shell_index].faces.empty();
}

bool BRep::is_solid() const {

    if (m_solids.empty())
        return false;

    for (const BRepSolid& s : m_solids)
        for (const BRepRef& r : s.shells)
            if (!is_closed(r.index))
                return false;

    return true;
}

BRepOrientation BRep::face_orientation(int face_index) const {

    for (const BRepShell& s : m_shells)
        for (const BRepRef& r : s.faces)
            if (r.index == face_index)
                return r.orientation;

    return BRepOrientation::Forward;
}

int BRep::pcurve_index(int edge_index, int face_index, BRepOrientation orientation) const {

    if (edge_index < 0 || edge_index >= (int)m_edges.size())
        return -1;

    if (face_index < 0 || face_index >= (int)m_faces.size())
        return -1;

    const int si = m_faces[face_index].surface_index;

    for (const BRepCurveOnSurface& pc : m_edges[edge_index].pcurves)
        if (pc.surface_index == si)
            return (orientation == BRepOrientation::Reversed && pc.curve_2d_index_2 >= 0) ? pc.curve_2d_index_2 : pc.curve_2d_index;

    return -1;
}

std::vector<BRepRef> BRep::wire_edges(const BRepRef& wire) const {

    std::vector<BRepRef> out;

    if (wire.index < 0 || wire.index >= (int)m_wires.size())
        return out;

    for (const BRepRef& r : m_wires[wire.index].edges)
        out.push_back({r.index, brep_compose(wire.orientation, r.orientation)});

    if (wire.orientation == BRepOrientation::Reversed)
        std::reverse(out.begin(), out.end());

    return out;
}

std::vector<BRepRef> BRep::edge_faces(int edge_index) const {

    std::vector<BRepRef> out;

    for (int fi = 0; fi < (int)m_faces.size(); ++fi) {
        const BRepOrientation fo = face_orientation(fi);

        for (const BRepRef& wr : m_faces[fi].wires)
            for (const BRepRef& er : wire_edges(wr))
                if (er.index == edge_index)
                    out.push_back({fi, brep_compose(fo, er.orientation)});
    }

    return out;
}

std::vector<Point> BRep::vertex_points() const {

    std::vector<Point> pts;

    for (const BRepVertex& v : m_vertices)
        pts.push_back(v.point);

    return pts;
}

std::vector<Polyline> BRep::face_polylines() const { return planar_faces(*this).first; }

std::vector<Plane> BRep::face_planes() const { return planar_faces(*this).second; }

double BRep::update_tolerances() {

    double worst = 0.0;

    for (BRepEdge& e : m_edges) {
        BRepVertex& vs = m_vertices[e.start_vertex];
        BRepVertex& ve = m_vertices[e.end_vertex];
        double tol = e.tolerance;

        if (e.curve_3d_index >= 0) {
            const NurbsCurve& c = m_curves_3d[e.curve_3d_index];
            tol = std::max(tol, c.point_at(c.domain().first).distance(vs.point));
            tol = std::max(tol, c.point_at(c.domain().second).distance(ve.point));
        }

        for (const BRepCurveOnSurface& pc : e.pcurves) {
            const NurbsSurface& srf = m_surfaces[pc.surface_index];

            for (const int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
                if (ci < 0)
                    continue;

                const NurbsCurve& c2 = m_curves_2d[ci];
                const Point a = c2.point_at(c2.domain().first);
                const Point z = c2.point_at(c2.domain().second);
                tol = std::max(tol, srf.point_at(a[0], a[1]).distance(vs.point));
                tol = std::max(tol, srf.point_at(z[0], z[1]).distance(ve.point));
            }
        }

        e.tolerance = tol;
        vs.tolerance = std::max(vs.tolerance, tol);
        ve.tolerance = std::max(ve.tolerance, tol);
        worst = std::max(worst, tol);
    }

    return worst;
}

double BRep::volume() const { return mesh().volume(); }

// ═══════════════════════════════════════════════════════════════════════════
// Building
// ═══════════════════════════════════════════════════════════════════════════
int BRep::add_surface(const NurbsSurface& srf) {

    m_surfaces.push_back(srf);

    return (int)m_surfaces.size() - 1;
}

int BRep::add_curve_3d(const NurbsCurve& crv) {

    m_curves_3d.push_back(crv);

    return (int)m_curves_3d.size() - 1;
}

int BRep::add_curve_2d(const NurbsCurve& crv) {

    m_curves_2d.push_back(crv);

    return (int)m_curves_2d.size() - 1;
}

int BRep::add_vertex(const Point& pt, double tolerance) {

    m_vertices.push_back({pt, tolerance});

    return (int)m_vertices.size() - 1;
}

int BRep::add_edge(int curve_3d_index, int start_vertex, int end_vertex, double tolerance) {

    BRepEdge e;
    e.curve_3d_index = curve_3d_index;
    e.start_vertex = start_vertex;
    e.end_vertex = end_vertex;
    e.tolerance = tolerance;
    e.degenerated = curve_3d_index < 0;
    m_edges.push_back(e);

    return (int)m_edges.size() - 1;
}

void BRep::add_pcurve(int edge_index, int surface_index, int curve_2d_index, int curve_2d_index_2) {

    for (BRepCurveOnSurface& pc : m_edges[edge_index].pcurves)
        if (pc.surface_index == surface_index) {
            pc.curve_2d_index = curve_2d_index;
            pc.curve_2d_index_2 = curve_2d_index_2;
            return;
        }

    m_edges[edge_index].pcurves.push_back({surface_index, curve_2d_index, curve_2d_index_2});
}

int BRep::add_wire(const std::vector<BRepRef>& edges) {

    m_wires.push_back({edges});

    return (int)m_wires.size() - 1;
}

int BRep::add_face(int surface_index, const std::vector<BRepRef>& wires, double tolerance) {

    BRepFace f;
    f.surface_index = surface_index;
    f.wires = wires;
    f.tolerance = tolerance;
    m_faces.push_back(f);

    return (int)m_faces.size() - 1;
}

int BRep::add_shell(const std::vector<BRepRef>& faces) {

    m_shells.push_back({faces});

    return (int)m_shells.size() - 1;
}

int BRep::add_solid(const std::vector<BRepRef>& shells) {

    m_solids.push_back({shells});

    return (int)m_solids.size() - 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshing
// ═══════════════════════════════════════════════════════════════════════════
Mesh BRep::mesh() const {

    std::vector<std::vector<Point>> polygons;

    for (const Mesh& fm : face_meshes())
        for (const std::pair<const size_t, std::vector<size_t>>& entry : fm.face) {
            std::vector<Point> poly;

            for (const size_t vi : entry.second)
                poly.push_back(fm.vertex.at(vi).position());

            polygons.push_back(poly);
        }

    return Mesh::from_polylines(polygons, 1e-6);
}

std::vector<Mesh> BRep::face_meshes() const { return face_meshes_q(false, 0.0, 0.0); }

std::vector<Mesh> BRep::face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const {

    const int nf = (int)m_faces.size();
    const double angle = has_quality ? max_angle_deg : 20.0;
    const double chord = has_quality ? chord_factor : 0.005;
    std::vector<bool> face_direct(nf, false);
    std::vector<bool> rebuild_grid(nf, false);
    std::vector<Mesh> fmesh(nf);
    EdgeBoundary boundary;

    for (int fi = 0; fi < nf; ++fi)
        face_direct[fi] = direct_face(*this, fi);

    for (int fi = 0; fi < nf; ++fi) {
        if (!face_direct[fi])
            continue;

        const NurbsSurface& srf = m_surfaces[m_faces[fi].surface_index];
        fmesh[fi] = has_quality ? RemeshNurbsSurfaceGrid::from_u_v_q(srf, 0, 0, max_angle_deg, chord_factor) : srf.mesh();
        rebuild_grid[fi] = grid_boundaries(*this, fi, fmesh[fi], boundary);
    }

    refine_shared_boundaries(*this, face_direct, rebuild_grid, boundary, angle, chord);

    for (int fi = 0; fi < nf; ++fi)
        if (rebuild_grid[fi])
            face_direct[fi] = false;

    for (int fi = 0; fi < nf; ++fi) {
        if (face_direct[fi])
            continue;

        const NurbsSurface& srf = m_surfaces[m_faces[fi].surface_index];
        TrimLoops loops;

        if (rebuild_grid[fi])
            loops.interior_uv = grid_interior_uv(srf, fmesh[fi]);

        std::vector<std::tuple<int, size_t, size_t, size_t>> uses;

        if (!trim_loops(*this, fi, boundary, angle, chord, loops, uses))
            continue;

        if (loops.interior_uv.empty() && is_planar_patch(srf)) {
            fmesh[fi] = planar_loops_mesh(srf, loops);
        } else {
            NurbsSurfaceTrimmed ts;
            ts.m_surface = srf;
            fmesh[fi] = ts.mesh_loops(loops, angle, chord);
        }

        tag_edge_uses(fmesh[fi], loops, uses);
    }

    flip_reversed_faces(*this, fmesh);

    return fmesh;
}

// ═══════════════════════════════════════════════════════════════════════════
// Evaluation
// ═══════════════════════════════════════════════════════════════════════════
Point BRep::point_at(int face_index, double u, double v) const {

    if (face_index < 0 || face_index >= (int)m_faces.size())
        return Point();

    return m_surfaces[m_faces[face_index].surface_index].point_at(u, v);
}

Vector BRep::normal_at(int face_index, double u, double v) const {

    if (face_index < 0 || face_index >= (int)m_faces.size())
        return Vector();

    const Vector n = m_surfaces[m_faces[face_index].surface_index].normal_at(u, v);

    if (face_orientation(face_index) == BRepOrientation::Reversed)
        return -n;

    return n;
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void BRep::transform(const Xform& xform) {

    for (NurbsSurface& srf : m_surfaces)
        srf.transform(xform);

    for (NurbsCurve& crv : m_curves_3d)
        crv.transform(xform);

    for (BRepVertex& v : m_vertices)
        v.point = xform.transform_point(v.point);
}

BRep BRep::transformed(const Xform& xform) const {

    BRep b = *this;
    b.transform(xform);

    return b;
}

// ═══════════════════════════════════════════════════════════════════════════
// Cutting
// ═══════════════════════════════════════════════════════════════════════════
BRep BRep::cut_by_plane(const Plane& plane) const {

    Mesh rings;

    if (!face_rings(*this, rings)) {
        const Mesh tessellation = mesh();

        return tessellation.cut_by_plane(plane) == tessellation ? *this : BRep();
    }

    const Mesh cut = rings.cut_by_plane(plane);

    if (cut == rings)
        return *this;

    if (cut.is_empty())
        return BRep();

    std::vector<Polyline> polylines;
    std::vector<std::vector<Polyline>> holes;

    for (const std::pair<const size_t, std::vector<size_t>>& entry : cut.face) {
        polylines.push_back(ring_polyline(cut, entry.second));
        holes.emplace_back();
        const auto it = cut.face_holes.find(entry.first);

        if (it != cut.face_holes.end())
            for (const std::vector<size_t>& hole : it->second)
                holes.back().push_back(ring_polyline(cut, hole));
    }

    BRep result = from_polylines(polylines, holes);
    result.name = name;
    result.width = width;
    result.surfacecolor = surfacecolor;

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json BRep::jsondump() const {

    nlohmann::ordered_json j;
    j["curves_2d"] = nlohmann::ordered_json::array();

    for (const NurbsCurve& c : m_curves_2d)
        j["curves_2d"].push_back(c.jsondump());

    j["curves_3d"] = nlohmann::ordered_json::array();

    for (const NurbsCurve& c : m_curves_3d)
        j["curves_3d"].push_back(c.jsondump());

    j["edges"] = nlohmann::ordered_json::array();

    for (const BRepEdge& e : m_edges)
        j["edges"].push_back(edge_to_json(e));

    j["faces"] = nlohmann::ordered_json::array();

    for (const BRepFace& f : m_faces)
        j["faces"].push_back(face_to_json(f));

    j["guid"] = guid();
    j["name"] = name;
    j["shells"] = nlohmann::ordered_json::array();

    for (const BRepShell& s : m_shells)
        j["shells"].push_back({{"faces", refs_to_json(s.faces)}});

    j["solids"] = nlohmann::ordered_json::array();

    for (const BRepSolid& s : m_solids)
        j["solids"].push_back({{"shells", refs_to_json(s.shells)}});

    j["surfacecolor"] = surfacecolor.jsondump();
    j["surfaces"] = nlohmann::ordered_json::array();

    for (const NurbsSurface& s : m_surfaces)
        j["surfaces"].push_back(s.jsondump());

    j["type"] = "BRep";
    j["vertices"] = nlohmann::ordered_json::array();

    for (const BRepVertex& v : m_vertices) {
        nlohmann::ordered_json vj;
        vj["point"] = nlohmann::ordered_json::array({v.point[0], v.point[1], v.point[2]});
        vj["tolerance"] = v.tolerance;
        j["vertices"].push_back(vj);
    }

    j["width"] = width;
    j["wires"] = nlohmann::ordered_json::array();

    for (const BRepWire& w : m_wires)
        j["wires"].push_back({{"edges", refs_to_json(w.edges)}});

    return j;
}

BRep BRep::jsonload(const nlohmann::json& data) {

    BRep b;

    if (data.contains("guid"))
        b.guid() = data["guid"];

    if (data.contains("name"))
        b.name = data["name"];

    if (data.contains("width"))
        b.width = data["width"];

    if (data.contains("surfacecolor"))
        b.surfacecolor = Color::jsonload(data["surfacecolor"]);

    if (data.contains("curves_2d"))
        for (const nlohmann::json& c : data["curves_2d"])
            b.m_curves_2d.push_back(NurbsCurve::jsonload(c));

    if (data.contains("curves_3d"))
        for (const nlohmann::json& c : data["curves_3d"])
            b.m_curves_3d.push_back(NurbsCurve::jsonload(c));

    if (data.contains("surfaces"))
        for (const nlohmann::json& s : data["surfaces"])
            b.m_surfaces.push_back(NurbsSurface::jsonload(s));

    if (data.contains("vertices"))
        for (const nlohmann::json& v : data["vertices"])
            b.m_vertices.push_back({Point(v["point"][0], v["point"][1], v["point"][2]), v["tolerance"]});

    if (data.contains("edges"))
        for (const nlohmann::json& e : data["edges"])
            b.m_edges.push_back(edge_from_json(e));

    if (data.contains("wires"))
        for (const nlohmann::json& w : data["wires"])
            b.m_wires.push_back({refs_from_json(w["edges"])});

    if (data.contains("faces"))
        for (const nlohmann::json& f : data["faces"])
            b.m_faces.push_back(face_from_json(f));

    if (data.contains("shells"))
        for (const nlohmann::json& s : data["shells"])
            b.m_shells.push_back({refs_from_json(s["faces"])});

    if (data.contains("solids"))
        for (const nlohmann::json& s : data["solids"])
            b.m_solids.push_back({refs_from_json(s["shells"])});

    return b;
}

std::string BRep::file_json_dumps() const { return jsondump().dump(); }

BRep BRep::file_json_loads(const std::string& json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void BRep::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

BRep BRep::file_json_load(const std::string& filename) {

    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;

    return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::BRep BRep::to_proto() const {

    session_proto::BRep proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_width(width);

    for (const NurbsCurve& c : m_curves_2d)
        *proto.add_curves_2d() = c.to_proto();

    for (const NurbsCurve& c : m_curves_3d)
        *proto.add_curves_3d() = c.to_proto();

    for (const NurbsSurface& s : m_surfaces)
        *proto.add_surfaces() = s.to_proto();

    for (const BRepVertex& v : m_vertices) {
        session_proto::BRepVertex* p = proto.add_vertices();
        p->mutable_point()->set_x(v.point[0]);
        p->mutable_point()->set_y(v.point[1]);
        p->mutable_point()->set_z(v.point[2]);
        p->set_tolerance(v.tolerance);
    }

    for (const BRepEdge& e : m_edges)
        edge_to_proto(e, proto.add_edges());

    for (const BRepWire& w : m_wires)
        refs_to_proto(w.edges, proto.add_wires()->mutable_edges());

    for (const BRepFace& f : m_faces)
        face_to_proto(f, proto.add_faces());

    for (const BRepShell& s : m_shells)
        refs_to_proto(s.faces, proto.add_shells()->mutable_faces());

    for (const BRepSolid& s : m_solids)
        refs_to_proto(s.shells, proto.add_solids()->mutable_shells());

    *proto.mutable_surfacecolor() = surfacecolor.to_proto();

    return proto;
}

BRep BRep::from_proto(const session_proto::BRep& proto) {

    BRep b;

    if (!proto.guid().empty())
        b.guid() = proto.guid();

    b.name = proto.name();
    b.width = proto.width();

    for (const session_proto::NurbsCurve& c : proto.curves_2d())
        b.m_curves_2d.push_back(NurbsCurve::from_proto(c));

    for (const session_proto::NurbsCurve& c : proto.curves_3d())
        b.m_curves_3d.push_back(NurbsCurve::from_proto(c));

    for (const session_proto::NurbsSurface& s : proto.surfaces())
        b.m_surfaces.push_back(NurbsSurface::from_proto(s));

    for (const session_proto::BRepVertex& v : proto.vertices())
        b.m_vertices.push_back({Point(v.point().x(), v.point().y(), v.point().z()), v.tolerance()});

    for (const session_proto::BRepEdge& e : proto.edges())
        b.m_edges.push_back(edge_from_proto(e));

    for (const session_proto::BRepWire& w : proto.wires())
        b.m_wires.push_back({refs_from_proto(w.edges())});

    for (const session_proto::BRepFace& f : proto.faces())
        b.m_faces.push_back(face_from_proto(f));

    for (const session_proto::BRepShell& s : proto.shells())
        b.m_shells.push_back({refs_from_proto(s.faces())});

    for (const session_proto::BRepSolid& s : proto.solids())
        b.m_solids.push_back({refs_from_proto(s.shells())});

    b.surfacecolor = Color::from_proto(proto.surfacecolor());

    return b;
}

std::string BRep::pb_dumps() const { return to_proto().SerializeAsString(); }

BRep BRep::pb_loads(const std::string& data) {

    session_proto::BRep proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse BRep protobuf data");

    return from_proto(proto);
}

void BRep::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

BRep BRep::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string BRep::str() const {

    return fmt::format("BRep(name={}, faces={}, edges={}, vertices={})", name, face_count(), edge_count(), vertex_count());
}

std::string BRep::repr() const {

    return fmt::format(
        "BRep(\n  name={},\n  faces={},\n  edges={},\n  vertices={},\n  solid={}\n)",
        name,
        face_count(),
        edge_count(),
        vertex_count(),
        is_solid() ? "true" : "false"
    );
}

std::ostream& operator<<(std::ostream& os, const BRep& brep) {

    os << brep.str();

    return os;
}

} // namespace session_cpp
