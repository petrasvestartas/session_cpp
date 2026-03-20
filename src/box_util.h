#pragma once
#include "vector.h"
#include "xform.h"
#include "point.h"
#include <cmath>

// ── session_cpp OBB (Oriented Bounding Box) collision utilities ───────────────
// Replaces cgal_box_util.h/.cpp using session_cpp types.
// Each OBB is represented as Vector[5]:
//   [0] = center (as Vector), [1..3] = local axes, [4] = half-extents per axis.

namespace session_cpp {
namespace obb {

namespace internal {

inline bool get_separating_plane(
    const Vector& rp, const Vector& axis,
    const Vector (&box1)[5], const Vector (&box2)[5])
{
    return fabs(rp.dot(axis)) >
        (  fabs((box1[1] * box1[4][0]).dot(axis))
         + fabs((box1[2] * box1[4][1]).dot(axis))
         + fabs((box1[3] * box1[4][2]).dot(axis))
         + fabs((box2[1] * box2[4][0]).dot(axis))
         + fabs((box2[2] * box2[4][1]).dot(axis))
         + fabs((box2[3] * box2[4][2]).dot(axis)));
}

} // namespace internal

/// Transform the center component (index 0) using xform.transform_point and
/// axes (indices 1–3) using xform.transform_vector.
inline void transform_plane_as_vector_array(Vector* plane, const Xform& xform)
{
    Point p = {plane[0][0], plane[0][1], plane[0][2]};
    xform.transform_point(p);
    plane[0] = {p[0], p[1], p[2]};
    for (int i = 1; i < 4; i++)
        xform.transform_vector(plane[i]);
}

/// Copy n Vector elements from source to target.
inline void assign(const Vector* source, Vector* target, int n)
{
    for (int i = 0; i < n; i++)
        target[i] = source[i];
}

/// SAT OBB–OBB collision test; returns true when the two boxes overlap.
inline bool get_collision(const Vector (&box1)[5], const Vector (&box2)[5])
{
    Vector rp = box2[0] - box1[0];
    return !(
        internal::get_separating_plane(rp, box1[1], box1, box2) ||
        internal::get_separating_plane(rp, box1[2], box1, box2) ||
        internal::get_separating_plane(rp, box1[3], box1, box2) ||
        internal::get_separating_plane(rp, box2[1], box1, box2) ||
        internal::get_separating_plane(rp, box2[2], box1, box2) ||
        internal::get_separating_plane(rp, box2[3], box1, box2) ||
        internal::get_separating_plane(rp, box1[1].cross(box2[1]), box1, box2) ||
        internal::get_separating_plane(rp, box1[1].cross(box2[2]), box1, box2) ||
        internal::get_separating_plane(rp, box1[1].cross(box2[3]), box1, box2) ||
        internal::get_separating_plane(rp, box1[2].cross(box2[1]), box1, box2) ||
        internal::get_separating_plane(rp, box1[2].cross(box2[2]), box1, box2) ||
        internal::get_separating_plane(rp, box1[2].cross(box2[3]), box1, box2) ||
        internal::get_separating_plane(rp, box1[3].cross(box2[1]), box1, box2) ||
        internal::get_separating_plane(rp, box1[3].cross(box2[2]), box1, box2) ||
        internal::get_separating_plane(rp, box1[3].cross(box2[3]), box1, box2));
}

} // namespace obb
} // namespace session_cpp
