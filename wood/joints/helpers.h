// Wood verbatim: `interpolate_points(from, to, steps, include_ends=false, ...)`
// from wood_joint_lib.cpp:392-416 produces `steps` points at parameters
//   i / (1 + steps)  for i = 1..steps
// — i.e. NEITHER endpoint is included. ts_e_p_3 / ss_e_op_1 rely on this:
// arrays have `steps` points (not steps+1), so the loop `for j = 0..size-1`
// stops one iteration earlier than it would if the endpoint was included,
// and the explicit `pline.push_back(arrays[*][size-1])` at the end of each
// joint outline build adds the array's LAST point — which is the
// `steps/(1+steps)` parameter point, NOT the geometric endpoint.
//
// Producing `steps+1` points with include-ends semantics introduces a
// duplicate at the end of the outline (the explicit append duplicates the
// last loop push) and creates a visible back-and-forth spike where the
// joint outline meets the surrounding plate polyline.
// Wrapper around `session_cpp::Polyline::interpolate_points(a, b, steps, 0)`.
// The session primitive already has the no-endpoints variant wood needs;
// keeping a thin free function here preserves the short call-site spelling
// used across every joint constructor below.
static inline std::vector<Point> interpolate_points(const Point& a, const Point& b, int steps) {
    return Point::interpolate(a, b, steps, /*kind=*/0);
}

// Wood's `internal::remap_numbers` already exists in session as
// `Intersection::remap(value, in_min, in_max, out_min, out_max)` —
// callers use it directly.
