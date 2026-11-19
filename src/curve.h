#pragma once

#include "aabb.h"
#include "json.h"
#include "nurbscurve.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace session_cpp {

/// Geom_Curve: a line, circle, ellipse, parabola or hyperbola kept as such, in its true parameter (length along a line, angle from x toward y on a circle or ellipse, periodic; ElCLib's parameter on a parabola and hyperbola), or a NURBS curve for freeform. One interface for all; the NURBS of an analytic curve over its domain is made on demand
class Curve {
public:
    enum class Kind { Nurbs, Line, Circle, Ellipse, Parabola, Hyperbola };

private:
    Kind m_kind = Kind::Nurbs; // What the curve is.
    Point m_origin = Point(0, 0, 0); // A point of the line, the centre of a conic.
    Vector m_x = Vector(1, 0, 0); // Unit direction of the line; direction of angle 0 of a conic.
    Vector m_y = Vector(0, 1, 0); // Direction of angle pi / 2 of a conic, unit and normal to x.
    double m_radius = 0.0; // Radius of a circle, semi-axis of an ellipse along x.
    double m_minor = 0.0; // Semi-axis of an ellipse along y.
    std::array<double, 2> m_domain = {0.0, 0.0}; // Parameter range in use.
    mutable std::shared_ptr<const NurbsCurve> m_nurbs; // The freeform curve; the cached NURBS of an analytic one.

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an invalid curve.
    Curve() = default;

    /// Construct a freeform curve.
    Curve(const NurbsCurve& nurbs);

    /// Line origin + t direction over [t0, t1]; the direction is made unit, t is length
    static Curve line(const Point& origin, const Vector& direction, double t0, double t1);

    /// Segment from a to b, t from 0 to its length
    static Curve line(const Point& a, const Point& b);

    /// Circle center + radius (cos t x + sin t y) over [t0, t1]; y is made unit and normal to x
    static Curve circle(const Point& center, const Vector& x, const Vector& y, double radius, double t0 = 0.0, double t1 = 2.0 * 3.14159265358979323846);

    /// Ellipse center + radius cos t x + minor sin t y over [t0, t1]
    static Curve ellipse(const Point& center, const Vector& x, const Vector& y, double radius, double minor, double t0 = 0.0, double t1 = 2.0 * 3.14159265358979323846);

    /// Parabola vertex + t t / (4 focal) x + t y over [t0, t1] (Geom_Parabola): x the axis toward the focus at distance `focal`, t the length along y
    static Curve parabola(const Point& vertex, const Vector& x, const Vector& y, double focal, double t0, double t1);

    /// The branch center + radius cosh t x + minor sinh t y of a hyperbola over [t0, t1] (Geom_Hyperbola)
    static Curve hyperbola(const Point& center, const Vector& x, const Vector& y, double radius, double minor, double t0, double t1);

    /// The line, circle or ellipse a NURBS curve lies on within tolerance times its size, over the same extent and in the same direction; the freeform curve otherwise
    static Curve recognized(const NurbsCurve& nurbs, double tolerance = 1e-10);

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    Kind kind() const { return m_kind; }
    const Point& origin() const { return m_origin; }
    const Vector& x() const { return m_x; }
    const Vector& y() const { return m_y; }
    double radius() const { return m_radius; }
    double minor() const { return m_minor; }

    /// Normal x cross y of a conic's plane.
    Vector z() const { return m_x.cross(m_y); }

    /// Whether the curve is analytic or holds a valid NURBS curve.
    bool is_valid() const;

    /// Parameter range in use.
    std::pair<double, double> domain() const;

    /// Set the parameter range in use; a freeform curve is trimmed to it.
    bool set_domain(double t0, double t1);

    /// 2 pi for a circle or ellipse, 0 otherwise.
    double period() const;

    /// Whether the ends of the domain meet.
    bool is_closed() const;

    /// t moved by whole periods into [start, start + period); t itself without a period
    static double wrapped(double t, double start, double period);

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Point at t, inside the domain or not.
    Point point_at(double t) const;

    /// Point and derivatives up to derivative_count (at most 2): [C, C', C''].
    std::vector<Vector> evaluate(double t, int derivative_count = 0) const;

    /// Unit tangent at t.
    Vector tangent_at(double t) const;

    /// Parameter of the closest point of the whole line, circle or ellipse (ElCLib::Parameter), on a conic in [domain start, domain start + 2 pi); of a freeform curve, within its domain
    double closest_parameter(const Point& point) const;

    /// Closest point, as closest_parameter.
    Point closest_point(const Point& point) const;

    /// Box of the curve over its domain.
    AABB aabb() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place: a line stays a line (its parameter stays length), a conic stays one under a similarity and becomes freeform under any other map.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    Curve transformed(const Xform& xform) const;

    /// Reverse the direction: C(t) becomes C(-t), the domain [-t1, -t0]; a freeform curve keeps its domain.
    void reverse();

    /// The uniform scale of a similarity (rigid motion, mirror, uniform scale), 0 for any other map.
    static double similarity(const Xform& xform);

    // ═══════════════════════════════════════════════════════════════════════════
    // NURBS
    // ═══════════════════════════════════════════════════════════════════════════
    /// The curve over its domain as a NURBS curve, made once: a line of degree 1 in its own parameter, a circle or ellipse as rational quadratic arcs of at most a quarter turn with knots at their angles, a parabola as one polynomial arc of degree 2 in its own parameter, a hyperbola as rational quadratic arcs of at most a unit of its parameter.
    const NurbsCurve& nurbs() const;

    /// Controls (cos, sin, weight) and knots of the unit arc from a0 to a1 in quarter turns or less.
    static void arc(double a0, double a1, std::vector<std::array<double, 3>>& controls, std::vector<double>& knots);

    /// The parameter of that arc's NURBS at an angle: between two knots the rational parameter is not the angle.
    static double rational(double angle, double a0, double a1);

    /// The angle at a parameter of that arc's NURBS: the inverse of rational.
    static double angle(double parameter, double a0, double a1);

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Curve jsonload(const nlohmann::json& data);
};

/// Geom2d_Curve, a pcurve: a Curve in the plane (u, v, 0) of a surface's true parameters. On a periodic surface it may lie anywhere modulo the period and run across the seam
class Curve2d : public Curve {
public:
    Curve2d() = default;
    Curve2d(const Curve& curve) : Curve(curve) {}
    Curve2d(const NurbsCurve& nurbs) : Curve(nurbs) {}

    /// Straight pcurve from (u0, v0) to (u1, v1), t from 0 to its length
    static Curve2d line(double u0, double v0, double u1, double v1);

    /// Moved by (du, dv), as by whole periods of its surface.
    Curve2d shifted(double du, double dv) const;
};

} // namespace session_cpp
