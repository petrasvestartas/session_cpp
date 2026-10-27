#pragma once

#include "brep.h"
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

/// Classify a parameter-space point against the face's NURBS trimming wires.
State face_uv(const BRep& brep, int face, const Point& uv, double tolerance);

/// Intersect a forward ray with trimmed NURBS faces; distance uses a unit direction.
RayResult ray(
    const BRep& brep,
    const Point& origin,
    const Vector& direction,
    double tolerance
);

/// Classify a point against oriented solid shells without tessellation or OCCT.
State point(const BRep& brep, const Point& query, double tolerance);

}
