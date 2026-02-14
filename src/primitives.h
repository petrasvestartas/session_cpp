#pragma once

#include "nurbscurve.h"
#include "nurbssurface.h"
#include "mesh.h"
#include "line.h"
#include "point.h"
#include "vector.h"
#include "plane.h"
#include "xform.h"
#include "knot.h"
#include "tolerance.h"

namespace session_cpp {

/// Factory class for creating primitive meshes, NURBS curves, and NURBS surfaces.
class Primitives {
public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Mesh primitives
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create an arrow mesh (cylinder body + cone head) oriented along a line.
    /// Body is 80% of line length, cone head is 20%. Cone radius is 1.5× body radius.
    static Mesh arrow_mesh(const Line& line, double radius);

    /// Create a cylinder mesh (10-sided) oriented along a line.
    static Mesh cylinder_mesh(const Line& line, double radius);

    /// Create a tetrahedron mesh (4 triangular faces) with given edge length.
    static Mesh tetrahedron(double edge = 2.0);

    /// Create a cube mesh (6 quad faces) with given edge length.
    static Mesh cube(double edge = 2.0);

    /// Create an octahedron mesh (8 triangular faces) with given edge length.
    static Mesh octahedron(double edge = 2.0);

    /// Create an icosahedron mesh (20 triangular faces) with given edge length.
    static Mesh icosahedron(double edge = 2.0);

    /// Create a dodecahedron mesh (12 pentagonal faces) with given edge length.
    static Mesh dodecahedron(double edge = 2.0);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Curve primitives
    ///////////////////////////////////////////////////////////////////////////////////////////


    /// Create a full circle as a rational quadratic NURBS (9 CVs, degree 2).
    static NurbsCurve circle(double cx, double cy, double cz, double radius);

    /// Create a full ellipse as a rational quadratic NURBS (9 CVs, degree 2).
    static NurbsCurve ellipse(double cx, double cy, double cz, double major_radius, double minor_radius);

    /// Create a circular arc through three points as a rational quadratic NURBS.
    /// Falls back to a degree-1 line if the points are collinear.
    static NurbsCurve arc(const Point& start, const Point& mid, const Point& end);

    /// Create a parabola through three points as a non-rational quadratic NURBS.
    /// p1 is treated as the apex of the parabola.
    static NurbsCurve parabola(const Point& p0, const Point& p1, const Point& p2);

    /// Create a hyperbola approximation as a cubic NURBS (x = a·cosh(t), y = b·sinh(t)).
    static NurbsCurve hyperbola(const Point& center, double a, double b, double extent);

    /// Create a spiral (helix with linearly varying radius) as a cubic NURBS.
    static NurbsCurve spiral(double start_radius, double end_radius, double pitch, double turns);

    /// Create an interpolated cubic NURBS curve through points (Bessel end tangents).
    static NurbsCurve create_interpolated(const std::vector<Point>& points,
                                           CurveKnotStyle parameterization = CurveKnotStyle::Chord);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Surface primitives
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create a NURBS cylinder surface (rational, degree 2x1).
    static NurbsSurface cylinder_surface(double cx, double cy, double cz, double radius, double height);

    /// Create a NURBS cone surface (rational, degree 2x1) tapering to an apex.
    static NurbsSurface cone_surface(double cx, double cy, double cz, double radius, double height);

    /// Create a NURBS torus surface (rational, degree 2x2).
    static NurbsSurface torus_surface(double cx, double cy, double cz, double major_radius, double minor_radius);

    /// Create a NURBS sphere surface (rational, degree 2x2).
    static NurbsSurface sphere_surface(double cx, double cy, double cz, double radius);

    /// Create a quad sphere from 6 rational biquadratic Bézier patches (cube projection).
    static std::vector<NurbsSurface> quad_sphere(double cx, double cy, double cz, double radius);

    /// Create a Schwarz P triply-periodic minimal surface (TPMS) as 8 NURBS patches.
    /// The unit cell is centered at (cx,cy,cz) with edge length `size`. Tileable in 3D.
    static std::vector<NurbsSurface> schwarz_p(double cx, double cy, double cz, double size);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Surface factory methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create a ruled (linear loft) surface between two curves.
    static NurbsSurface create_ruled(const NurbsCurve& curveA, const NurbsCurve& curveB);

    /// Create an extrusion surface by translating a curve along a direction vector.
    static NurbsSurface create_extrusion(const NurbsCurve& curve, const Vector& direction);

    /// Create a planar surface that contains the boundary curve's control polygon.
    static NurbsSurface create_planar(const NurbsCurve& boundary);

    /// Create a loft (skinned) surface through multiple section curves.
    static NurbsSurface create_loft(const std::vector<NurbsCurve>& curves, int degree_v = 3);

    /// Create a surface of revolution by rotating a profile curve around an axis.
    static NurbsSurface create_revolve(const NurbsCurve& profile, const Point& axis_origin,
                                        const Vector& axis_direction, double angle = 2.0 * 3.14159265358979323846);

    /// Create a single-rail sweep surface by moving a profile along a rail curve.
    static NurbsSurface create_sweep1(const NurbsCurve& rail, const NurbsCurve& profile);

    /// Create a two-rail sweep surface.
    static NurbsSurface create_sweep2(const NurbsCurve& rail1, const NurbsCurve& rail2,
                                       const std::vector<NurbsCurve>& shapes);

    /// Create a Coons-patch surface from four boundary curves.
    static NurbsSurface create_edge(const NurbsCurve& c0, const NurbsCurve& c1,
                                             const NurbsCurve& c2, const NurbsCurve& c3);

    /// Create a doubly-periodic sinusoidal wave surface (egg-crate pattern).
    /// Approximates z = amplitude * sin(2*pi*x/size) * sin(2*pi*y/size).
    /// Tileable: edges match for seamless stacking in both x and y.
    static NurbsSurface wave_surface(double size, double amplitude);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Surface-to-mesh subdivision
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create a quad mesh by sampling a surface on a regular u×v grid.
    static Mesh quad_mesh(const NurbsSurface& surface, int u_count, int v_count);

    /// Create a diamond mesh by sampling a surface on a rotated grid pattern.
    static Mesh diamond_mesh(const NurbsSurface& surface, int u_count, int v_count);

    /// Create a hexagonal mesh by sampling a surface with hex pattern.
    static Mesh hex_mesh(const NurbsSurface& surface, int u_count, int v_count, double t = 1.0 / 3.0);

    /// Create a hexagonal mesh variant with adjustable lerp parameter.
    static Mesh hex_mesh2(const NurbsSurface& surface, int u_count, int v_count, double t = 2.0 / 3.0);

private:
    /// Unit cylinder geometry (10-sided, radius 0.5, height 1, centered at origin).
    static std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>> unit_cylinder_geometry();

    /// Unit cone geometry (8-sided, apex at (0,0,0.5), base radius 0.5 at z=-0.5).
    static std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>> unit_cone_geometry();

    /// Compute a transform that maps the unit cylinder to a cylinder along a line with given radius.
    static Xform line_to_cylinder_transform(const Line& line, double radius);

    /// Apply a transform to unit geometry vertices and build a Mesh.
    static Mesh transform_geometry(const std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>>& geometry, const Xform& xform);
};

} // namespace session_cpp
