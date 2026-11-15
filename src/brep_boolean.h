#pragma once

#include "brep.h"
#include <stdexcept>

namespace session_cpp::brep_boolean {

enum class Operation { Union, Intersection, Difference, Xor };

/// Preserve an unresolved classification for exact reproduction by the caller.
struct ClassificationError : std::runtime_error {
    BRep shape;
    Point query;
    int source_face;
    bool second_operand;
    double tolerance;

    ClassificationError(const BRep& other, const Point& point, int face, bool second, double tol)
        : std::runtime_error("Native Boolean face classification is unresolved"),
          shape(other), query(point), source_face(face), second_operand(second), tolerance(tol) {}
};

/// Keep each edge whole, as OCCT does: two edges at a vertex no other edge meets, used one after the other by the same wires and lying on the same surfaces, become one
BRep joined_edges(BRep b, double tolerance);

/// Build a solid from native face intersections, trimmed regions and oriented shells.
BRep apply(const BRep& first, const BRep& second, Operation operation, double tolerance);

}
