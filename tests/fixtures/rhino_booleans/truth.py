"""Exact intersection volumes of the Rhino torus and cone fixtures, independent of any BRep kernel.

V = 1/3 * flux of (x - c) over the boundary of A and B, each surface taken inside the other solid.
Inner integral per parameter line: exact interval ends, Gauss on each interval.
Outer integral: tanh-sinh between breakpoints, so sqrt ends at tangent lines converge.
"""

import json
import re
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__)) + "/"
GX, GW = np.polynomial.legendre.leggauss(40)


def parse(name):
    text = open(ROOT + "inputs/" + name, errors="replace").read()
    entities = {
        int(m.group(1)): re.sub(r"\s+", "", m.group(2))
        for m in re.finditer(r"#(\d+)\s*=\s*(.*?);", text, re.S)
    }
    points = {}
    for key, value in entities.items():
        m = re.match(r"CARTESIAN_POINT\('[^']*',\(([^)]*)\)\)", value)
        if m:
            parts = [float(x) for x in m.group(1).split(",")]
            if len(parts) == 3:
                points[key] = np.array(parts)
    surfaces = []
    for key, value in entities.items():
        if "RATIONAL_B_SPLINE_SURFACE" not in value:
            continue
        m = re.search(r"B_SPLINE_SURFACE\(\d+,\d+,\(\((.*?)\)\),\.", value)
        rows = [[int(x[1:]) for x in r.split(",")] for r in m.group(1).split("),(")]
        m = re.search(r"RATIONAL_B_SPLINE_SURFACE\(\(\((.*?)\)\)\)", value)
        weights = [[float(x) for x in r.split(",")] for r in m.group(1).split("),(")]
        surfaces.append(
            (np.array([[points[i] for i in r] for r in rows]), np.array(weights))
        )
    assert len(surfaces) == 1
    return surfaces[0]


def circle(points):
    """Centre, unit normal and radius of points on a circle."""
    mean = points.mean(axis=0)
    normal = np.linalg.svd(points - mean)[2][2]
    e1 = points[0] - mean
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)
    x = (points - mean) @ e1
    y = (points - mean) @ e2
    m = np.c_[2 * x, 2 * y, np.ones(len(x))]
    a, b, c = np.linalg.lstsq(m, x * x + y * y, rcond=None)[0]
    centre = mean + a * e1 + b * e2
    return centre, normal, float(np.sqrt(c + a * a + b * b))


def frame(axis):
    e1 = np.cross(axis, [1.0, 0.0, 0.0] if abs(axis[0]) < 0.7 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(axis, e1)


class Torus:
    def __init__(self, grid, weights):
        on = grid[
            ::2, ::2
        ]  # Poles of weight 1 or 0.7071^2 scaling sit on the surface at even indices.
        n = on.shape[0] - 1
        best = None
        for j in range(on.shape[1] - 1):
            c, a, r = circle(on[:n, j])
            best = (c, a, r) if best is None or r > best[2] else best
        self.c, self.a, self.outer = best
        self.e1, self.e2 = frame(self.a)
        tube = on[0, : on.shape[1] - 1]
        c2, _, self.r = circle(tube)
        self.R = float(np.linalg.norm(c2 - self.c))
        err = np.abs(self.f(on.reshape(-1, 3))) / (4 * self.R * self.R)
        print(f"  torus R {self.R:.12f} r {self.r:.12f} surface fit {err.max():.2e}")

    def f(self, p):
        q = p - self.c
        z = q @ self.a
        s = (q * q).sum(axis=-1) + self.R**2 - self.r**2
        return s * s - 4 * self.R**2 * ((q * q).sum(axis=-1) - z * z)

    def patch(self, u, v):
        """Point and d/du x d/dv with outward orientation; arrays broadcast."""
        cu, su, cv, sv = np.cos(u), np.sin(u), np.cos(v), np.sin(v)
        radial = cu[..., None] * self.e1 + su[..., None] * self.e2
        tangent = -su[..., None] * self.e1 + cu[..., None] * self.e2
        rho = self.R + self.r * cv
        p = self.c + rho[..., None] * radial + (self.r * sv)[..., None] * self.a
        pu = rho[..., None] * tangent
        pv = self.r * (-sv[..., None] * radial + cv[..., None] * self.a)
        return p, np.cross(pu, pv)

    def faces(self):
        return [(self, 0.0, 2.0 * np.pi, 0.0, 2.0 * np.pi)]

    def inner_cuts(self, other, u):
        """v values where a tube circle crosses the other solid: a degree 4 trig polynomial in v."""
        count = 16
        v = 2.0 * np.pi * np.arange(count) / count
        p, _ = self.patch(np.full(count, u), v)
        coefficients = np.fft.fft(other.f(p)) / count
        poly = np.array(
            [coefficients[k % count] for k in range(4, -5, -1)], dtype=complex
        )
        z = np.roots(poly)
        real = z[np.abs(np.abs(z) - 1.0) < 1e-6]
        return np.sort(np.mod(np.angle(real), 2.0 * np.pi))


class Cone:
    def __init__(self, grid, weights):
        ends = [grid[:, 0], grid[:, -1], grid[0, :], grid[-1, :]]
        apex = None
        for row in ends:
            if np.ptp(row, axis=0).max() < 1e-9:
                apex = row[0]
        rim = None
        for row in ends:
            if (
                np.ptp(row, axis=0).max() > 1e-6
                and np.linalg.norm(row[0] - row[-1]) < 1e-9
            ):
                rim = row
        self.apex = apex
        self.base, _, self.radius = circle(rim[::2][:-1])
        self.h = float(np.linalg.norm(apex - self.base))
        self.a = (apex - self.base) / self.h
        self.e1, self.e2 = frame(self.a)
        print(
            f"  cone radius {self.radius:.12f} height {self.h:.12f} volume {np.pi * self.radius**2 * self.h / 3:.10f}"
        )

    def clip_line(self, origin, direction):
        """Parameter range of a line inside the convex solid, and which surface bounds each end."""
        q = origin - self.apex
        k = (self.radius / self.h) ** 2
        ax, bx = direction @ self.a, q @ self.a
        A = direction @ direction - (1 + k) * ax * ax
        B = 2 * (direction @ q - (1 + k) * ax * bx)
        C = q @ q - (1 + k) * bx * bx
        # Below the apex: axial coordinate from the apex is non-positive.
        if abs(A) < 1e-14:
            return None
        d = B * B - 4 * A * C
        if d <= 0:
            return None
        s = np.sqrt(d)
        t0, t1 = sorted(((-B - s) / (2 * A), (-B + s) / (2 * A)))
        if A > 0:
            pieces = [(t0, t1)]
        else:
            pieces = [(-np.inf, t0), (t1, np.inf)]
        solid = []
        for a, b in pieces:
            a2, b2 = a, b
            ta, tb = 0, 0
            # Half space towards the base of the apex and the base plane.
            h0 = q @ self.a  # negative on the solid's side of the apex.
            for sign, plane, tag in ((1.0, h0, 2), (-1.0, h0 + self.h, 1)):
                # sign*(plane + t*ax) <= 0
                slope = sign * ax
                value = sign * plane
                if abs(slope) < 1e-14:
                    if value > 0:
                        a2, b2 = 1.0, 0.0
                    continue
                t = -value / slope
                if slope > 0:
                    if t < b2:
                        b2, tb = t, tag
                else:
                    if t > a2:
                        a2, ta = t, tag
            if a2 < b2:
                solid.append((a2, b2, ta, tb))
        if not solid:
            return None
        assert len(solid) == 1
        return solid[0]


class ConeSide:
    """Rim to apex along straight generators, u around the axis, v in [0, 1]."""

    def __init__(self, cone):
        self.cone = cone

    def line(self, u):
        c = self.cone
        rim = c.base + c.radius * (np.cos(u) * c.e1 + np.sin(u) * c.e2)
        return rim, c.apex - rim

    def density(self, u, v, centre):
        c = self.cone
        e = np.cos(u) * c.e1 + np.sin(u) * c.e2
        t = -np.sin(u) * c.e1 + np.cos(u) * c.e2
        rim = c.base + c.radius * e
        p = rim[None, :] + v[:, None] * (c.apex - rim)
        pu = c.radius * (1.0 - v)[:, None] * t[None, :]
        pv = c.apex - rim
        return ((p - centre) * np.cross(pu, pv[None, :])).sum(axis=1) / 3.0


class ConeCap:
    """Base disc along radial lines, u around the axis, v in [0, radius]."""

    def __init__(self, cone):
        self.cone = cone

    def line(self, u):
        c = self.cone
        return c.base, c.radius * (np.cos(u) * c.e1 + np.sin(u) * c.e2)

    def density(self, u, v, centre):
        c = self.cone
        e = np.cos(u) * c.e1 + np.sin(u) * c.e2
        p = c.base + c.radius * v[:, None] * e[None, :]
        normal = -c.a
        return ((p - centre) @ normal) * c.radius**2 * v / 3.0


def tanh_sinh(fun, a, b, tolerance=1e-10):
    """Tanh-sinh over [a, b]; fun may be singular at the ends."""
    half = 0.5 * (b - a)
    total = None
    h = 0.5
    previous = None
    seen = 0
    for level in range(9):
        ks = np.arange(-int(4.0 / h), int(4.0 / h) + 1)
        if level > 0:
            ks = ks[ks % 2 != 0]
        new = 0.0
        for k in ks:
            t = k * h
            s = 0.5 * np.pi * np.sinh(t)
            weight = 0.5 * np.pi * np.cosh(t) / np.cosh(s) ** 2
            x = np.tanh(s)
            if abs(x) >= 1.0 or weight < 1e-300:
                continue
            new += weight * fun(a + half * (1.0 + x))
            seen += 1
        total = new if total is None else total + new
        value = half * h * total
        if previous is not None and abs(value - previous) < tolerance:
            return value, seen
        previous = value
        h *= 0.5
    return value, seen


def torus_inside(face, other, u, centre):
    cuts = face.inner_cuts(other, u)
    if len(cuts) == 0:
        return 0.0, 0
    assert len(cuts) % 2 == 0, cuts
    total = 0.0
    edges = np.r_[cuts, cuts[0] + 2.0 * np.pi]
    for a, b in zip(edges[:-1], edges[1:]):
        mid = 0.5 * (a + b)
        p, _ = face.patch(np.array([u]), np.array([mid]))
        if other.f(p)[0] < 0:
            v = 0.5 * (b - a) * GX + mid
            p, n = face.patch(np.full(len(v), u), v)
            total += 0.5 * (b - a) * (GW * ((p - centre) * n).sum(axis=1) / 3.0).sum()
    return total, len(cuts)


def breakpoints(signature, lo, hi, count=20000):
    grid = np.linspace(lo, hi, count + 1)
    keys = [signature(u) for u in grid]
    cuts = [lo]
    for i in range(count):
        if keys[i] != keys[i + 1]:
            a, b = grid[i], grid[i + 1]
            ka = keys[i]
            for _ in range(60):
                m = 0.5 * (a + b)
                if signature(m) == ka:
                    a = m
                else:
                    b = m
            cuts.append(0.5 * (a + b))
    cuts.append(hi)
    return cuts


def integrate(inside, signature):
    cuts = breakpoints(signature, 0.0, 2.0 * np.pi)
    total = 0.0
    evals = 0
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b - a < 1e-13:
            continue
        value, n = tanh_sinh(inside, a, b)
        total += value
        evals += n
    return total, len(cuts) - 2, evals


def torus_family(centre_shift):
    a = Torus(*parse("InputTorus-object-0.step"))
    b = Torus(*parse("InputTorus-object-1.step"))
    centre = 0.5 * (a.c + b.c) + centre_shift
    total = 0.0
    for face, other in ((a, b), (b, a)):
        value, breaks, evals = integrate(
            lambda u: torus_inside(face, other, u, centre)[0],
            lambda u: len(face.inner_cuts(other, u)),
        )
        print(f"  piece {value:.9f} breaks {breaks} evals {evals}")
        total += value
    return total, a, b


def cone_family(centre_shift):
    a = Cone(*parse("InputCones-object-0.step"))
    b = Cone(*parse("InputCones-object-1.step"))
    centre = 0.5 * (a.base + b.base) + centre_shift
    total = 0.0

    def ranged(face, other, u):
        origin, direction = face.line(u)
        hit = other.clip_line(origin, direction)
        if hit is None:
            return None
        low, high = max(hit[0], 0.0), min(hit[1], 1.0)
        if low >= high:
            return None
        return (
            low,
            high,
            (hit[2] if hit[0] > 0.0 else 3),
            (hit[3] if hit[1] < 1.0 else 3),
        )

    for cone, other in ((a, b), (b, a)):
        for face in (ConeSide(cone), ConeCap(cone)):

            def inside(u, face=face, other=other):
                r = ranged(face, other, u)
                if r is None:
                    return 0.0
                mid, half = 0.5 * (r[0] + r[1]), 0.5 * (r[1] - r[0])
                v = mid + half * GX
                return half * (GW * face.density(u, v, centre)).sum()

            def signature(u, face=face, other=other):
                r = ranged(face, other, u)
                return None if r is None else (r[2], r[3])

            value, breaks, evals = integrate(inside, signature)
            print(
                f"  piece {type(face).__name__} {value:.9f} breaks {breaks} evals {evals}"
            )
            total += value
    return total, a, b


def main():
    reference = json.load(open(ROOT + "compact-reference.json"))
    for family, run in (("Torus", torus_family), ("Cones", cone_family)):
        item = next(f for f in reference["families"] if f["family"] == family)
        va = item["input_order"][0]["signed_volume"]
        vb = item["input_order"][1]["signed_volume"]
        out = {
            o["operation"]: o["reference"]["signed_volume"]
            for o in item["output_references"]
        }
        print(family, "va", va, "vb", vb)
        print(
            "  rhino intersection",
            out["intersection"],
            " from rhino union",
            va + vb - out["union"],
        )
        values = []
        for shift in ((0.0, 0.0, 0.0), (3.7, -11.3, 5.9)):
            value, a, b = run(np.array(shift))
            print(f"  shift {shift}: intersection {value:.9f}")
            values.append(value)
        print(f"  agreement between origins {abs(values[0] - values[1]):.2e}")
        sys.stdout.flush()


main()
