"""
bvh_query_aabb_example.py
=========================
Pure-Python reference implementation of BVH AABB range query.

This mirrors the logic of the C++ ``BVH::query_aabb`` and
``AABBTree::query_aabb`` methods added in session_cpp/src/bvh.h and
aabb.h.  It is runnable without building any C++ or Rust code and
serves as a readable correctness reference.

Usage
-----
    python bvh_query_aabb_example.py
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional
import math


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class AABB:
    """Axis-aligned bounding box stored as center + half-extents."""
    cx: float = 0.0
    cy: float = 0.0
    cz: float = 0.0
    hx: float = 0.0
    hy: float = 0.0
    hz: float = 0.0

    @staticmethod
    def from_min_max(xmin, ymin, zmin, xmax, ymax, zmax) -> "AABB":
        return AABB(
            cx=(xmin + xmax) * 0.5, cy=(ymin + ymax) * 0.5, cz=(zmin + zmax) * 0.5,
            hx=(xmax - xmin) * 0.5, hy=(ymax - ymin) * 0.5, hz=(zmax - zmin) * 0.5,
        )

    def intersects(self, other: "AABB") -> bool:
        return (
            abs(self.cx - other.cx) <= self.hx + other.hx and
            abs(self.cy - other.cy) <= self.hy + other.hy and
            abs(self.cz - other.cz) <= self.hz + other.hz
        )


# ---------------------------------------------------------------------------
# Linear scan reference (O(n), no tree)
# ---------------------------------------------------------------------------

def query_aabb_linear(segment_aabbs: List[AABB], query: AABB) -> List[int]:
    """Return indices of all segments whose AABB overlaps *query*."""
    return [i for i, s in enumerate(segment_aabbs) if s.intersects(query)]


# ---------------------------------------------------------------------------
# Simple BVH node for the pure-Python tree
# ---------------------------------------------------------------------------

@dataclass
class BVHNode:
    aabb: AABB = field(default_factory=AABB)
    object_id: int = -1          # >= 0 for leaves, -1 for internal nodes
    left: Optional["BVHNode"] = None
    right: Optional["BVHNode"] = None

    def is_leaf(self) -> bool:
        return self.object_id >= 0


def _merge(a: AABB, b: AABB) -> AABB:
    xmin = min(a.cx - a.hx, b.cx - b.hx)
    ymin = min(a.cy - a.hy, b.cy - b.hy)
    zmin = min(a.cz - a.hz, b.cz - b.hz)
    xmax = max(a.cx + a.hx, b.cx + b.hx)
    ymax = max(a.cy + a.hy, b.cy + b.hy)
    zmax = max(a.cz + a.hz, b.cz + b.hz)
    return AABB.from_min_max(xmin, ymin, zmin, xmax, ymax, zmax)


def _build_bvh(ids: List[int], aabbs: List[AABB]) -> BVHNode:
    """Recursive top-down BVH build (median split on longest axis)."""
    node = BVHNode()
    # Compute bounding AABB of all objects in this partition
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for i in ids:
        a = aabbs[i]
        lo[0] = min(lo[0], a.cx - a.hx)
        lo[1] = min(lo[1], a.cy - a.hy)
        lo[2] = min(lo[2], a.cz - a.hz)
        hi[0] = max(hi[0], a.cx + a.hx)
        hi[1] = max(hi[1], a.cy + a.hy)
        hi[2] = max(hi[2], a.cz + a.hz)
    node.aabb = AABB.from_min_max(*lo, *hi)

    if len(ids) == 1:
        node.object_id = ids[0]
        return node

    # Split on longest axis
    extents = [hi[k] - lo[k] for k in range(3)]
    axis = extents.index(max(extents))
    key = [lambda i: aabbs[i].cx, lambda i: aabbs[i].cy, lambda i: aabbs[i].cz][axis]
    ids_sorted = sorted(ids, key=key)
    mid = len(ids_sorted) // 2
    node.left  = _build_bvh(ids_sorted[:mid], aabbs)
    node.right = _build_bvh(ids_sorted[mid:], aabbs)
    return node


class BVH:
    """Minimal pure-Python BVH that mirrors the C++ ``BVH::query_aabb`` API."""

    def __init__(self, aabbs: List[AABB]):
        if not aabbs:
            self._root: Optional[BVHNode] = None
            return
        self._root = _build_bvh(list(range(len(aabbs))), aabbs)

    def query_aabb(self, query: AABB) -> List[int]:
        """Return object_ids of all leaves whose AABB overlaps *query*."""
        hits: List[int] = []
        if self._root is None:
            return hits
        stack: List[BVHNode] = [self._root]
        while stack:
            node = stack.pop()
            if not node.aabb.intersects(query):
                continue
            if node.is_leaf():
                hits.append(node.object_id)
            else:
                if node.left:
                    stack.append(node.left)
                if node.right:
                    stack.append(node.right)
        return hits


# ---------------------------------------------------------------------------
# Demo / self-test
# ---------------------------------------------------------------------------

def _run_demo() -> None:
    # Build a small set of segment AABBs
    segments = [
        AABB.from_min_max( 0,  0,  0,  2,  1,  1),  # id 0 — near origin
        AABB.from_min_max(10,  0,  0, 12,  1,  1),  # id 1 — far right
        AABB.from_min_max( 0, 10,  0,  2, 11,  1),  # id 2 — far up
        AABB.from_min_max( 1,  0,  0,  3,  1,  1),  # id 3 — overlaps id 0
    ]

    query = AABB.from_min_max(-0.5, -0.5, -0.5, 2.5, 1.5, 1.5)

    linear = query_aabb_linear(segments, query)
    bvh    = BVH(segments)
    tree   = bvh.query_aabb(query)

    print("Query AABB:", query)
    print("Linear scan hits:", sorted(linear))
    print("BVH      hits:   ", sorted(tree))

    assert sorted(linear) == sorted(tree), "Mismatch between linear and BVH!"
    assert 0 in linear, "Expected segment 0 to be hit"
    assert 3 in linear, "Expected segment 3 to be hit"
    assert 1 not in linear, "Segment 1 should not be hit"
    assert 2 not in linear, "Segment 2 should not be hit"
    print("All assertions passed.")


if __name__ == "__main__":
    _run_demo()
