#pragma once

// v3 geometry kernel — analytic curves and surfaces as FIRST-CLASS types.
// The hard-won law (kb/BOOL_V3_MEMORY.md §3/§4): analytic identity is carried,
// never re-fitted. Quadrics and the torus live here in exact OCCT-style
// parametrizations; only genuinely freeform geometry is a NurbsSurface wrapper.
//
// Conventions (OCCT-compatible):
//   plane    : P(u,v) = o + u·X + v·Y                       (z = normal)
//   cylinder : P(u,v) = o + r·(cosu·X + sinu·Y) + v·Z       u∈[0,2π)
//   cone     : P(u,v) = apex + (r0 + v·sinα)·(cosu·X + sinu·Y) + v·cosα·Z
//   sphere   : P(u,v) = c + r·cosv·(cosu·X + sinu·Y) + r·sinv·Z  v∈[-π/2,π/2]
//   torus    : P(u,v) = c + (R + r·cosv)·(cosu·X + sinu·Y) + r·sinv·Z
// All except NURBS have exact implicit F(P)=0 and exact inverse uv_of(P).

#include "../point.h"
#include "../vector.h"
#include "../nurbscurve.h"
#include "../nurbssurface.h"
#include <cmath>
#include <vector>
#include <string>

namespace v3 {

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

struct V3 {
    double x = 0, y = 0, z = 0;
    V3() = default;
    V3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator-() const { return {-x, -y, -z}; }
    V3 operator*(double s) const { return {x * s, y * s, z * s}; }
    V3 operator/(double s) const { return {x / s, y / s, z / s}; }
    V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    double dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
    V3 cross(const V3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double norm2() const { return x * x + y * y + z * z; }
    double norm() const { return std::sqrt(norm2()); }
    V3 normalized() const { double n = norm(); return n > 1e-300 ? *this / n : V3{0, 0, 0}; }
    double dist(const V3& o) const { return (*this - o).norm(); }
    double& operator[](int i) { return (&x)[i]; }
    const double& operator[](int i) const { return (&x)[i]; }
};

// Right-handed orthonormal frame. z is the meaningful axis (surface normal /
// axis of revolution); x,y complete it. from_z picks a stable arbitrary x ⊥ z.
struct Frame {
    V3 o, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    static Frame from_z(const V3& origin, const V3& zaxis);
    V3 to_world(double a, double b, double c) const {
        return o + x * a + y * b + z * c;
    }
};

// Analytic / NURBS / polyline curve. t is: line — arc length along x;
// circle/ellipse — angle (radians, CCW around z from x); NURBS — the curve
// parameter; POLY — sample index fraction (0..n-1, linear between samples).
// POLY carries marched section curves: dense samples ARE the geometry (with
// measured deflection tolerance), never re-fitted into bogus charts.
struct Cur {
    enum K { LINE, CIRCLE, ELLIPSE, NURBS, POLY } k = LINE;
    Frame f;                    // line: o=point, x=direction(unit)
    double r = 0, ry = 0;       // circle radius / ellipse major,minor radii
    session_cpp::NurbsCurve nrb; // valid only when k==NURBS
    std::vector<V3> poly;       // valid only when k==POLY
    bool poly_closed = false;
    V3 eval(double t) const;
    V3 d1(double t) const;      // first derivative
    V3 d2(double t) const;      // second derivative
    // Closest parameter to p (Newton for NURBS). Returns false on failure.
    bool project(const V3& p, double& t) const;
    bool is_closed() const { return k == CIRCLE || k == ELLIPSE || poly_closed; }
    double period() const { return (k == CIRCLE || k == ELLIPSE) ? TWO_PI : 0.0; }
};

// Analytic / NURBS surface. See file header for parametrizations.
struct Srf {
    enum K { PLANE, CYLINDER, CONE, SPHERE, TORUS, NURBS } k = PLANE;
    Frame f;
    double r = 0;    // cylinder/sphere radius, torus major R, cone reference radius r0
    double r2 = 0;   // torus minor r, cone semi-angle α (rad)
    // Normal sign: normal() = nflip · normalize(du × dv). Set so that normal()
    // matches the SOURCE NurbsSurface::normal_at convention (normalize(dv×du))
    // at corresponding points; loop orientations inherited from the BRep stay
    // valid without any re-winding.
    double nflip = 1.0;
    session_cpp::NurbsSurface nrb; // valid only when k==NURBS

    V3 eval(double u, double v) const;
    void d0d1(double u, double v, V3& p, V3& du, V3& dv) const;
    V3 normal(double u, double v) const; // unit; near-degenerate at poles — callers guard
    bool periodic_u() const { return k == CYLINDER || k == CONE || k == SPHERE || k == TORUS; }
    bool periodic_v() const { return k == TORUS; }
    double period_u() const { return periodic_u() ? TWO_PI : 0.0; }
    double period_v() const { return periodic_v() ? TWO_PI : 0.0; }

    bool has_implicit() const { return k != NURBS; }
    double F(const V3& p) const;                 // 0 on surface (double cone for CONE)
    V3 gradF(const V3& p) const;

    // Exact inverse: u,v such that eval(u,v) ≈ p (u wrapped to [0,2π) when
    // periodic). For NURBS: Newton projection, false if it does not converge.
    bool uv_of(const V3& p, double& u, double& v) const;

    // Natural domain extents for a FULL surface (finite direction of cylinders
    // is unbounded; callers use face trim bounds). Used for seeding.
    void canonical_domain(double& u0, double& u1, double& v0, double& v1) const;
};

// ---- construction helpers ----------------------------------------------------
Srf srf_plane(const V3& origin, const V3& normal);
Srf srf_plane(const V3& origin, const V3& normal, const V3& xdir);
Srf srf_cylinder(const V3& axis_pt, const V3& axis_dir, double radius);
Srf srf_cone(const V3& apex, const V3& axis_dir, double semi_angle, double ref_radius);
Srf srf_sphere(const V3& center, double radius);
Srf srf_torus(const V3& center, const V3& axis_dir, double major_r, double minor_r);
Srf srf_nurbs(const session_cpp::NurbsSurface& s);

Cur cur_line(const V3& p, const V3& dir);
Cur cur_circle(const Frame& fr, double radius);
Cur cur_circle(const V3& center, const V3& normal, double radius);
Cur cur_ellipse(const Frame& fr, double major_r, double minor_r);
Cur cur_nurbs(const session_cpp::NurbsCurve& c);
Cur cur_poly(const std::vector<V3>& pts, bool closed);

// Robust angle wrap to [0, 2π).
double wrap_2pi(double a);
// Bring `a` within ±π of `ref` (period 2π) — continuity-preserving unwrap.
double unwrap_near(double a, double ref);

// Recognize an analytic surface from a NurbsSurface (residual-gated: the fit
// must reproduce the surface to `tol` over a sample grid, else NURBS wrapper).
Srf recognize(const session_cpp::NurbsSurface& s, double tol = 1e-7);
Cur recognize_curve(const session_cpp::NurbsCurve& c, double tol = 1e-7);
// Same recognition from raw 3D samples (line -> planar conic -> circle/ellipse,
// residual-gated). False when nothing analytic fits (caller fits/falls back).
bool recognize_curve_pts(const V3* pts, int n, double tol, Cur& out);

// conversions at the boundary
V3 to_v3(const session_cpp::Point& p);
V3 to_v3(const session_cpp::Vector& v);
session_cpp::Point to_point(const V3& p);

} // namespace v3
