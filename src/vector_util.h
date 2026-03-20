#pragma once
#include "vector.h"

// ── session_cpp vector utilities ─────────────────────────────────────────────
// Convenience wrappers that match the old cgal_vector_util naming so existing
// wood call sites (`session_cpp::unitize(v)`, `session_cpp::length(x,y,z)`)
// compile without modification.

namespace session_cpp {

    /// Normalize a vector in-place; returns false if near-zero length.
    inline bool unitize(Vector& v) { return v.normalize_self(); }

    /// Euclidean length of a 3-component vector.
    inline double length(double x, double y, double z) { return Vector(x, y, z).magnitude(); }

} // namespace session_cpp
