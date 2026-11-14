#pragma once

#include "brep.h"
#include "brep_face_boundary.h"
#include <array>
#include <vector>

namespace session_cpp::brep_classify {

enum class State {
    Outside,
    Inside,
    Boundary,
    Unknown
};

struct RayHit {
    Point point;
    double distance = 0.0;
    double u = 0.0;
    double v = 0.0;
    int face = -1;
    int crossing = 0;
    State trim = State::Unknown;
    std::array<double, 2> distance_enclosure{};
};

struct RayResult {
    std::vector<RayHit> hits;
    bool complete = true;
    std::string failure;
    int unresolved_face = -1;
    std::array<double, 4> unresolved_domain{};
};

/// One face of a prepared model: its boundary, the v of both ends of each wire's pcurves, and its orientation in its solid.
struct Face {
    brep_face_boundary::Result boundary;
    std::vector<std::vector<std::array<double, 2>>> ends;
    BRepOrientation orientation = BRepOrientation::Forward;
    bool active = false;
};

/// A model checked and laid out once for many queries; it must stay as it is while this lives. For a model asked once the boundaries can be left to the queries that need them.
struct Solid {
    const BRep& brep;
    bool solid; // BRep::is_solid
    bool valid; // BRep::is_valid
    std::vector<Face> faces;
    std::vector<std::array<xyz, 2>> boxes; // Lowest and highest control point of each surface.
    explicit Solid(const BRep& model, bool boundaries = true);
};

/// A height for a +x parity ray from a point, within a quarter of tolerance of it and clear of every curve end: two curves meeting at a corner can differ in their last bits, and a ray through that gap counts the corner wrong.
double clear_ray_height(const std::vector<const NurbsCurve*>& loop, double height, double tolerance);

/// Classify a parameter-space point against the face's NURBS trimming wires.
State face_uv(const BRep& brep, int face, const Point& uv, double tolerance);

/// Query a pre-extracted boundary of an unchanged, already validated owner.
State face_uv(const brep_face_boundary::Result& boundary, const Point& uv, double tolerance);

/// Intersect a forward ray with trimmed NURBS faces; distance uses a unit direction.
RayResult ray(
    const BRep& brep,
    const Point& origin,
    const Vector& direction,
    double tolerance
);

/// Classify a point against oriented solid shells without tessellation or OCCT.
State point(const BRep& brep, const Point& query, double tolerance);

/// The same queries on a prepared model, which is checked once and not per query.
State face_uv(const Solid& solid, int face, const Point& uv, double tolerance);
RayResult ray(const Solid& solid, const Point& origin, const Vector& direction, double tolerance);
State point(const Solid& solid, const Point& query, double tolerance);

}
