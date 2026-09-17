#pragma once

#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbsknot.h"
#include "mesh.h"
#include "line.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include <array>
#include <utility>
#include <vector>

namespace session_cpp {

/// Factory for primitive meshes, NURBS curves and NURBS surfaces.
class Primitives {
public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Mesh primitives
    // ═══════════════════════════════════════════════════════════════════════════

    /// Arrow mesh along a line: cylinder body over 80% of the length, cone head of 1.5x radius over 20%.
    static Mesh arrow_mesh(const Line& line, double radius);

    /// Ten-sided cylinder mesh along a line.
    static Mesh cylinder_mesh(const Line& line, double radius);

    /// Ten-sided cylinder mesh with hemispherical caps along a line.
    static Mesh capsule_mesh(const Line& line, double radius);

    /// One capsule mesh per edge, colored by mesh.linecolors[i].
    static std::vector<Mesh> edge_pipes(const Mesh& mesh, double radius);

    /// Tetrahedron mesh (4 triangles) with the given edge length.
    static Mesh tetrahedron(double edge = 2.0);

    /// Cube mesh (6 quads) with the given edge length.
    static Mesh cube(double edge = 2.0);

    /// Octahedron mesh (8 triangles) with the given edge length.
    static Mesh octahedron(double edge = 2.0);

    /// Icosahedron mesh (20 triangles) with the given edge length.
    static Mesh icosahedron(double edge = 2.0);

    // ═══════════════════════════════════════════════════════════════════════════
    // Curve primitives
    // ═══════════════════════════════════════════════════════════════════════════

    /// Full circle as a rational quadratic NURBS (9 CVs).
    static NurbsCurve circle(double cx, double cy, double cz, double radius);

    /// Full ellipse as a rational quadratic NURBS (9 CVs).
    static NurbsCurve ellipse(double cx, double cy, double cz, double major_radius, double minor_radius);

    /// Circular arc from start through the arc midpoint to end as a rational quadratic NURBS; a line when collinear.
    static NurbsCurve arc(const Point& start, const Point& mid, const Point& end);

    /// Parabola through three points with p1 as the apex, as a quadratic NURBS.
    static NurbsCurve parabola(const Point& p0, const Point& p1, const Point& p2);

    /// Hyperbola x = a cosh(t), y = b sinh(t) for t in [-extent, extent] as a cubic NURBS through 9 points.
    static NurbsCurve hyperbola(const Point& center, double a, double b, double extent);

    /// Helix with linearly varying radius as a cubic NURBS, 8 points per turn.
    static NurbsCurve spiral(double start_radius, double end_radius, double pitch, double turns);

    /// Interpolated cubic NURBS through points.
    static NurbsCurve create_interpolated(
        const std::vector<Point>& points,
        CurveNurbsKnotStyle parameterization = CurveNurbsKnotStyle::Chord,
        CurveInterpStyle end_condition = CurveInterpStyle::Rhino
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Surface primitives
    // ═══════════════════════════════════════════════════════════════════════════

    /// Rational cylinder surface of degree 2x1 around the z axis through (cx, cy, cz).
    static NurbsSurface cylinder_surface(double cx, double cy, double cz, double radius, double height);

    /// Rational cone surface of degree 2x1 with the apex at cz + height.
    static NurbsSurface cone_surface(double cx, double cy, double cz, double radius, double height);

    /// Rational torus surface of degree 2x2.
    static NurbsSurface torus_surface(double cx, double cy, double cz, double major_radius, double minor_radius);

    /// Rational sphere surface of degree 2x2 with poles on the z axis.
    static NurbsSurface sphere_surface(double cx, double cy, double cz, double radius);

    /// Sphere as 6 rational biquadratic patches projected from the cube faces.
    static std::vector<NurbsSurface> quad_sphere(double cx, double cy, double cz, double radius);

    /// Tileable egg-crate surface z = amplitude sin(2 pi x / size) sin(2 pi y / size) as a 13x13 cubic NURBS.
    static NurbsSurface wave_surface(double size, double amplitude);

    // ═══════════════════════════════════════════════════════════════════════════
    // Surface factories
    // ═══════════════════════════════════════════════════════════════════════════

    /// Ruled surface between two curves.
    static NurbsSurface create_ruled(const NurbsCurve& curve_a, const NurbsCurve& curve_b);

    /// Extrusion of a curve along a direction.
    static NurbsSurface create_extrusion(const NurbsCurve& curve, const Vector& direction);

    /// Bilinear planar patch containing a closed boundary curve.
    static NurbsSurface create_planar(const NurbsCurve& boundary);

    /// Loft through section curves, interpolating them in v.
    static NurbsSurface create_loft(const std::vector<NurbsCurve>& curves, int degree_v = 3);

    /// Surface of revolution of a profile around an axis.
    static NurbsSurface create_revolve(
        const NurbsCurve& profile,
        const Point& axis_origin,
        const Vector& axis_direction,
        double angle = 2.0 * 3.14159265358979323846
    );

    /// Sweep of a closed profile along one rail.
    static NurbsSurface create_sweep1(const NurbsCurve& rail, const NurbsCurve& profile);

    /// Sweep of shape curves between two rails.
    static NurbsSurface create_sweep2(
        const NurbsCurve& rail1,
        const NurbsCurve& rail2,
        const std::vector<NurbsCurve>& shapes
    );

    /// Coons patch from four boundary curves in any order and direction.
    static NurbsSurface create_edge(
        const NurbsCurve& c0,
        const NurbsCurve& c1,
        const NurbsCurve& c2,
        const NurbsCurve& c3
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Surface to mesh
    // ═══════════════════════════════════════════════════════════════════════════

    /// Quad mesh sampled on a u_count x v_count grid.
    static Mesh quad_mesh(const NurbsSurface& surface, int u_count, int v_count);

    /// Diamond mesh sampled on a u_count x v_count grid.
    static Mesh diamond_mesh(const NurbsSurface& surface, int u_count, int v_count);

    /// Hexagonal mesh sampled on a u_count x v_count grid, t the split of each v cell.
    static Mesh hex_mesh(const NurbsSurface& surface, int u_count, int v_count, double t = 1.0 / 3.0);

private:
    using Geometry = std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>>;

    /// Ten-sided unit cylinder: radius 0.5, z from -0.5 to 0.5.
    static Geometry unit_cylinder_geometry();

    /// Eight-sided unit cone: base radius 0.5 at z = -0.5, apex at z = 0.5.
    static Geometry unit_cone_geometry();

    /// Ten-sided capsule along z from 0 to length with hemispherical caps.
    static Geometry capsule_geometry(double length, double radius);

    /// Frame at origin with z along the line.
    static Xform line_frame(const Line& line, const Point& origin);

    /// Appends transformed geometry to a mesh.
    static void add_geometry(Mesh& mesh, const Geometry& geometry, const Xform& xform);
};

} // namespace session_cpp
