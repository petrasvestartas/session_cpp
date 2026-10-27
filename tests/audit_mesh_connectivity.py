"""Separate changed vertex positions from changed triangle connectivity.

Reads saved Session Mesh JSON only; does not invoke either geometry kernel.
Requires NumPy and SciPy. The report does not certify surface deflection.
"""

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_mesh(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    ids = sorted(map(int, data["vertex"]))
    indices = {key: index for index, key in enumerate(ids)}
    points = np.array([[data["vertex"][str(key)][axis] for axis in "xyz"] for key in ids])
    triangles = [tuple(indices[key] for key in face) for face in data["face"].values()]
    if any(len(triangle) != 3 for triangle in triangles):
        raise ValueError("Expected a triangle mesh")
    return points, triangles


def signed_volume(points, triangles, origin):
    return math.fsum(float(np.dot(points[a] - origin,
        np.cross(points[b] - origin, points[c] - origin))) / 6
        for a, b, c in triangles)


def oriented_boundary(triangles):
    counts = Counter()
    for triangle in triangles:
        for a, b in zip(triangle, triangle[1:] + triangle[:1]):
            counts[min(a, b), max(a, b)] += 1 if a < b else -1
    return {edge: count for edge, count in counts.items() if count}


def incidence(triangles):
    counts = Counter()
    for triangle in triangles:
        for a, b in zip(triangle, triangle[1:] + triangle[:1]):
            counts[min(a, b), max(a, b)] += 1
    return {"edges": len(counts), "edges_without_two_uses": sum(count != 2 for count in counts.values()),
        "edges_with_orientation_imbalance": len(oriented_boundary(triangles))}


def components(removed, added, points, origin):
    result = []
    remaining = set(range(len(removed)))
    while remaining:
        group = {min(remaining)}
        remaining.difference_update(group)
        vertices = set(removed[next(iter(group))])
        changed = True
        while changed:
            changed = False
            for index in sorted(remaining):
                if len(vertices.intersection(removed[index])) >= 2:
                    remaining.remove(index)
                    group.add(index)
                    vertices.update(removed[index])
                    changed = True
        before = [removed[index] for index in sorted(group)]
        after = [triangle for triangle in added if set(triangle) <= vertices]
        result.append({"removed_triangles": before, "added_triangles": after,
            "same_oriented_boundary": oriented_boundary(before) == oriented_boundary(after),
            "volume_delta_at_original_positions": signed_volume(points, after, origin)
                - signed_volume(points, before, origin)})
    return result


def compare(original, replacement, tolerance):
    a, ta = read_mesh(original)
    b, tb = read_mesh(replacement)
    distances, mapping = cKDTree(a).query(b)
    inverse_distances, _ = cKDTree(b).query(a)
    bijective = len(a) == len(b) and len(set(mapping)) == len(mapping)
    report = {"original": str(original.resolve()), "replacement": str(replacement.resolve()),
        "original_sha256": hashlib.sha256(original.read_bytes()).hexdigest(),
        "replacement_sha256": hashlib.sha256(replacement.read_bytes()).hexdigest(),
        "vertex_count_original": len(a), "vertex_count_replacement": len(b),
        "nearest_vertex_map_bijective": bijective,
        "bidirectional_max_vertex_gap": max(float(distances.max()), float(inverse_distances.max())),
        "matching_tolerance": tolerance, "original_incidence": incidence(ta),
        "replacement_incidence": incidence(tb), "certifies_surface_deflection": False}
    if not bijective or report["bidirectional_max_vertex_gap"] > tolerance:
        raise ValueError("Meshes do not have a unique vertex match within the requested tolerance")
    mapped = [tuple(int(mapping[index]) for index in triangle) for triangle in tb]
    before = {tuple(sorted(triangle)): triangle for triangle in ta}
    after = {tuple(sorted(triangle)): triangle for triangle in mapped}
    if len(before) != len(ta) or len(after) != len(tb):
        raise ValueError("Duplicate triangle detected")
    removed = [before[key] for key in sorted(before.keys() - after.keys())]
    added = [after[key] for key in sorted(after.keys() - before.keys())]
    origin = a[0]
    actual = signed_volume(b, tb, origin) - signed_volume(a, ta, origin)
    connectivity = signed_volume(a, mapped, origin) - signed_volume(a, ta, origin)
    report.update({"removed_triangle_count": len(removed), "added_triangle_count": len(added),
        "changed_regions_have_same_oriented_boundary": oriented_boundary(removed) == oriented_boundary(added),
        "actual_volume_delta": actual, "connectivity_only_volume_delta": connectivity,
        "vertex_motion_volume_delta": actual - connectivity,
        "changed_regions": components(removed, added, a, origin)})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original", type=Path)
    parser.add_argument("replacement", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-8)
    args = parser.parse_args()
    report = compare(args.original, args.replacement, args.tolerance)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "changed_regions"}, indent=2))


if __name__ == "__main__":
    main()
