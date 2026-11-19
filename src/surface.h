#pragma once

#include "aabb.h"
#include "curve.h"
#include "json.h"
#include "nurbssurface.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace session_cpp {

/// Geom_Surface: a plane, cylinder, cone, sphere or torus kept as such, in its true parameters (length, and angle about the axis z from x toward y, periodic), or a NURBS surface for freeform. With e(u) = cos u x + sin u y:
/// plane O + u x + v y; cylinder O + R e(u) + v z; cone O + (R + v sin a) e(u) + v cos a z; sphere O + R cos v e(u) + R sin v z; torus O + (R + r cos v) e(u) + r sin v z.
/// A surface of revolution turns a profile curve C about the axis by u (Geom_SurfaceOfRevolution): O + rot(u) (C(v) - O), periodic in u; an extrusion moves it along the unit direction z (Geom_SurfaceOfLinearExtrusion): C(u) + v z.
/// One interface for all; the NURBS of an analytic surface over its domain is made on demand
class Surface {
public:
    enum class Kind { Nurbs, Plane, Cylinder, Cone, Sphere, Torus, Revolution, Extrusion };

private:
    Kind m_kind = Kind::Nurbs; // What the surface is.
    Point m_origin = Point(0, 0, 0); // A point of the plane or of the axis, the centre of a sphere or torus.
    Vector m_x = Vector(1, 0, 0); // Direction of u on a plane, of angle 0 about the axis.
    Vector m_y = Vector(0, 1, 0); // Direction of v on a plane, of angle pi / 2 about the axis.
    Vector m_z = Vector(0, 0, 1); // Normal of the plane, the axis; the three are unit and normal to each other, of either hand.
    double m_radius = 0.0; // Radius; of a cone at the origin; major radius of a torus.
    double m_minor = 0.0; // Semi-angle of a cone, minor radius of a torus.
    std::array<double, 4> m_domain = {0.0, 0.0, 0.0, 0.0}; // Parameter rectangle in use: u0, u1, v0, v1.
    mutable std::shared_ptr<const NurbsSurface> m_nurbs; // The freeform surface; the cached NURBS of an analytic one.
    std::shared_ptr<const Curve> m_profile; // The curve a revolution turns or an extrusion moves, in space.

    static Surface analytic(Kind kind, const Point& origin, const Vector& z, const Vector& x, double radius, double minor, const std::array<double, 4>& domain);

    /// Distance from the axis and height along it at v, with their derivatives up to the second: rho, rho', rho'', h, h', h''.
    std::array<double, 6> profile(double v) const;

    /// The profile over the surface's domain along it.
    Curve swept() const;
    std::vector<Vector> sweep_evaluate(double u, double v, int num_derivs) const;
    std::pair<double, double> sweep_parameters(const Point& point) const;
    int sweep_coincident(const Surface& other, double tolerance) const;
    bool sweep_transform(const Xform& xform);
    void sweep_reverse(int dir);
    const NurbsSurface& sweep_nurbs() const;

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an invalid surface.
    Surface() = default;

    /// Construct a freeform surface.
    Surface(const NurbsSurface& nurbs);

    /// Plane through origin with normal z and u along x (gp_Ax3: y = z cross x), over the rectangle {u0, u1, v0, v1}
    static Surface plane(const Point& origin, const Vector& z, const Vector& x, const std::array<double, 4>& domain = {-1.0, 1.0, -1.0, 1.0});

    /// Cylinder about the axis z through origin, angle 0 at x
    static Surface cylinder(const Point& origin, const Vector& z, const Vector& x, double radius, const std::array<double, 4>& domain = {0.0, 2.0 * 3.14159265358979323846, 0.0, 1.0});

    /// Cone about the axis z: `radius` at the origin, widening along z by the semi-angle; v is length along a generator
    static Surface cone(const Point& origin, const Vector& z, const Vector& x, double radius, double semi_angle, const std::array<double, 4>& domain = {0.0, 2.0 * 3.14159265358979323846, 0.0, 1.0});

    /// Sphere about origin, v the latitude from -pi / 2 to pi / 2
    static Surface sphere(const Point& origin, const Vector& z, const Vector& x, double radius, const std::array<double, 4>& domain = {0.0, 2.0 * 3.14159265358979323846, -0.5 * 3.14159265358979323846, 0.5 * 3.14159265358979323846});

    /// Torus about the axis z through origin, v the angle about the tube from its outer equator
    static Surface torus(const Point& origin, const Vector& z, const Vector& x, double radius, double minor, const std::array<double, 4>& domain = {0.0, 2.0 * 3.14159265358979323846, 0.0, 2.0 * 3.14159265358979323846});

    /// The profile turned about the axis z through origin, u the angle from the profile itself over [u0, u1], v the profile's parameter over its domain; the profile lies anywhere in space
    static Surface revolution(const Point& origin, const Vector& z, const Curve& profile, double u0 = 0.0, double u1 = 2.0 * 3.14159265358979323846);

    /// The profile moved along the direction, made unit: u the profile's parameter over its domain, v the length over [v0, v1]
    static Surface extrusion(const Curve& profile, const Vector& direction, double v0 = 0.0, double v1 = 1.0);

    /// The plane, cylinder, cone, sphere or torus a NURBS surface lies on within tolerance, over the extent of the surface with angle 0 at its first corner; the freeform surface otherwise
    static Surface recognized(const NurbsSurface& nurbs, double tolerance = 1e-7);

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    Kind kind() const { return m_kind; }
    const Point& origin() const { return m_origin; }
    const Vector& x() const { return m_x; }
    const Vector& y() const { return m_y; }
    const Vector& z() const { return m_z; }
    double radius() const { return m_radius; }
    double minor() const { return m_minor; }

    /// The profile of a revolution or extrusion over the surface's domain along it; invalid for any other kind.
    Curve profile() const { return m_profile ? swept() : Curve(); }

    /// Whether x, y, z are a right-handed frame: the normal Su cross Sv of a right-handed plane is z, of a right-handed cylinder, cone, sphere or torus points away from the axis or tube.
    bool right_handed() const;

    /// Whether the surface is analytic or holds a valid NURBS surface.
    bool is_valid() const;

    /// Parameter range in use along u (dir 0) or v (dir 1).
    std::pair<double, double> domain(int dir) const;

    /// Set the parameter range in use along a direction; a freeform surface is trimmed to it.
    bool set_domain(int dir, double t0, double t1);

    /// 2 pi along an angle (u of a cylinder, cone, sphere, torus or revolution, v of a torus), the period of the profile along it, 0 otherwise.
    double period(int dir) const;

    /// (u, v) moved by whole periods to lie in, or else nearest to, the rectangle {u0, u1, v0, v1}, which may run past a period.
    std::pair<double, double> adjusted(double u, double v, const std::array<double, 4>& box) const;

    /// Whether two analytic surfaces are one surface whatever their parameter origins and directions: 1 with the normals alike, -1 opposed, 0 when they differ or one is freeform (freeform surfaces are one when they are the same object). A revolution or extrusion is compared with any analytic surface over the domains of both.
    int coincident(const Surface& other, double tolerance) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Point at (u, v), inside the domain or not.
    Point point_at(double u, double v) const;

    /// Point and partials up to num_derivs (at most 2) in NurbsSurface::evaluate's order: [S, Sv, Svv, Su, Suv, Suu].
    std::vector<Vector> evaluate(double u, double v, int num_derivs = 0) const;

    /// Unit normal Su cross Sv; away from the centre at a pole of a right-handed sphere, zero at the apex of a cone.
    Vector normal_at(double u, double v) const;

    /// Parameters of the closest point of the whole plane, cylinder, cone, sphere or torus (ElSLib::Parameters), an angle in [domain start, domain start + 2 pi); of a revolution or extrusion by Newton along the profile from its nearest sample in the domain, the angle or length in closed form; of a freeform surface, within its domain
    std::pair<double, double> closest_parameters(const Point& point) const;

    /// Closest point, as closest_parameters.
    Point closest_point(const Point& point) const;

    /// Distance of a point from the whole surface, closed form for an analytic one.
    double distance(const Point& point) const;

    /// Box of the surface over its domain.
    AABB aabb() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place: analytic under a similarity (a plane under any map that keeps its frame square, an extrusion under any map), freeform under any other map.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    Surface transformed(const Xform& xform) const;

    /// Reverse a parameter: S(u, v) becomes S(-u, v) (dir 0) or S(u, -v) (dir 1), the domain following; the normal turns over. A freeform surface keeps its domain, as does a sweep along a freeform profile.
    void reverse(int dir);

    // ═══════════════════════════════════════════════════════════════════════════
    // NURBS
    // ═══════════════════════════════════════════════════════════════════════════
    /// The surface over its domain as a NURBS surface, made once: rational quadratic along an angle in arcs of at most a quarter turn with knots at their angles, degree 1 along a length in its own parameter.
    const NurbsSurface& nurbs() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Surface jsonload(const nlohmann::json& data);
};

} // namespace session_cpp
