"""Volume of a bent beam on the normal side of a plane, from cylindrical coordinates alone.

The beam is r <= rho <= r + w, 0 <= z <= h, a0 <= theta <= a1. At fixed theta and z the plane bounds rho, so the rho integral
is closed; the z integral is piecewise quadratic and closed too. Theta is Gauss-Legendre between the kinks, found exactly.
"""

import sys

import numpy as np

GX, GW = np.polynomial.legendre.leggauss(64)


def slab(theta, r, w, h, origin, normal):
    """Integral over z and rho at theta of rho, on the side the normal points to."""
    a = normal[0] * np.cos(theta) + normal[1] * np.sin(theta)
    d = normal @ origin
    outer = r + w
    # The kept set is a * rho + n_z * z >= d, linear in (rho, z): clip the rectangle and integrate rho over the polygon.
    corners = [(r, 0.0), (outer, 0.0), (outer, h), (r, h)]
    value = lambda p: a * p[0] + normal[2] * p[1] - d
    polygon = []
    for i in range(4):
        p, q = corners[i], corners[(i + 1) % 4]
        vp, vq = value(p), value(q)
        if vp >= 0.0:
            polygon.append(p)
        if (vp >= 0.0) != (vq >= 0.0):
            t = vp / (vp - vq)
            polygon.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    if len(polygon) < 3:
        return 0.0
    total = 0.0
    for i in range(1, len(polygon) - 1):
        (x0, y0), (x1, y1), (x2, y2) = polygon[0], polygon[i], polygon[i + 1]
        area = 0.5 * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
        total += area * (x0 + x1 + x2) / 3.0
    return abs(total)


def kinks(r, w, h, a0, a1, origin, normal):
    """Theta where the plane passes through a corner of the section rectangle."""
    d = normal @ origin
    amplitude = np.hypot(normal[0], normal[1])
    phase = np.arctan2(normal[1], normal[0])
    found = [a0, a1]
    for rho in (r, r + w):
        for z in (0.0, h):
            if amplitude == 0.0:
                continue
            c = (d - normal[2] * z) / (rho * amplitude)
            if abs(c) <= 1.0:
                for sign in (-1.0, 1.0):
                    theta = phase + sign * np.arccos(c)
                    for wrap in (-2.0, 0.0, 2.0):
                        t = theta + wrap * np.pi
                        if a0 < t < a1:
                            found.append(t)
    return sorted(found)


def volume(r, w, h, a0, a1, origin, normal, panels):
    origin = np.array(origin, float)
    normal = np.array(normal, float)
    normal = normal / np.sqrt(normal @ normal)
    total = 0.0
    edges = kinks(r, w, h, a0, a1, origin, normal)
    for lo, hi in zip(edges[:-1], edges[1:]):
        grid = np.linspace(lo, hi, panels + 1)
        for p, q in zip(grid[:-1], grid[1:]):
            mid, half = 0.5 * (p + q), 0.5 * (q - p)
            total += half * sum(
                weight * slab(mid + half * x, r, w, h, origin, normal)
                for x, weight in zip(GX, GW)
            )
    return total


CASES = (
    (
        "beam",
        2000.0,
        120.0,
        240.0,
        0.2,
        1.2,
        (
            ((0, 0, 0), (-np.sin(0.65), np.cos(0.65), 0)),
            ((1400, 0, 0), (1, 0, 0)),
            ((1400, 1400, 120), (0.5, 0.7, 0.25)),
            ((1500, 1200, 120), (0.1, 0.2, 1.0)),
            ((1500, 1300, 100), (1, 1, 0.3)),
        ),
    ),
    (
        "plate",
        3000.0,
        40.0,
        1000.0,
        0.3,
        1.1,
        (
            ((0, 0, 0), (-np.sin(0.7), np.cos(0.7), 0)),
            ((0, 0, 400), (0, 0, 1)),
            ((2200, 1800, 500), (1, -0.4, 0.5)),
        ),
    ),
)
for name, r, w, h, a0, a1, planes in CASES:
    whole = 0.5 * ((r + w) ** 2 - r**2) * h * (a1 - a0)
    print(name, "whole", f"{whole:.6f}")
    for index, (origin, normal) in enumerate(planes):
        coarse = volume(r, w, h, a0, a1, origin, normal, 2)
        fine = volume(r, w, h, a0, a1, origin, normal, 6)
        print(
            f"  plane {index + 1} keep {fine:.7f}  other {whole - fine:.7f}  convergence {abs(fine - coarse):.1e}",
            flush=True,
        )
