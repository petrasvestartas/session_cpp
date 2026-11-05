"""Second, independent route to the Torus and Cones intersection volumes of the Rhino fixture.

Shares no code or method with truth.py. truth.py integrates a boundary flux over parameter lines.
This casts parallel rays along a generic direction, takes the exact length of each ray inside both
solids from a quartic or quadratic, and sums length times pitch squared: V = sum L h^2.
Repeating over K random grid shifts gives the error bar, mean +- 2 std / sqrt(K).
The same integrator is first run on solids with a closed-form volume, so the bar can be judged.

usage: truth_check.py [grid] [shifts] [intersections]
The third argument skips the closed forms and the operands with themselves.
"""

import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SEED = int(os.environ.get("RAY_SEED", 20261104))
DIRECTION = np.array([0.31, -0.57, 0.76])
DIRECTION = DIRECTION / np.sqrt(DIRECTION @ DIRECTION)


def perpendicular(d):
    first = np.cross(d, [0.0, 0.0, 1.0])
    first = first / np.sqrt(first @ first)
    return first, np.cross(d, first)


def control_grid(path):
    """The pole grid of the one rational surface in a STEP file, as a list of rows of points."""
    text = re.sub(r"\s+", "", open(path, errors="replace").read())
    points = {}
    for key, body in re.findall(
        r"#(\d+)=CARTESIAN_POINT\('[^']*',\(([^()]*)\)\);", text
    ):
        values = [float(x) for x in body.split(",")]
        if len(values) == 3:
            points[key] = values
    entity = next(
        statement
        for statement in text.split(";")
        if "RATIONAL_B_SPLINE_SURFACE" in statement
    )
    net = re.search(r"B_SPLINE_SURFACE\(\d+,\d+,\(\((#[^.]*?)\)\),\.", entity).group(1)
    return [[points[i] for i in re.findall(r"#(\d+)", row)] for row in net.split("),(")]


class Torus:
    """An exact torus read off the poles at even indices, which lie on the surface."""

    def __init__(self, path):
        grid = np.array(control_grid(path))
        samples = grid[:-1:2, :-1:2].reshape(-1, 3)
        self.c = samples.mean(axis=0)
        spread = np.cov((samples - self.c).T)
        self.a = np.linalg.eigh(spread)[1][:, 0]
        q = samples - self.c
        z = q @ self.a
        rho = np.sqrt(np.maximum((q * q).sum(axis=1) - z * z, 0.0))
        self.r = float(np.abs(z).max())
        self.R = float(0.5 * (rho.max() + rho.min()))

    def volume(self):
        return 2.0 * np.pi**2 * self.R * self.r**2

    def spans(self, origins, d):
        """Up to two inside intervals per ray, as a list of (low, high) arrays."""
        q = origins - self.c
        alpha = (q * q).sum(axis=1)
        beta = q @ d
        axial = d @ self.a
        height = q @ self.a
        k = alpha + self.R**2 - self.r**2
        four = 4.0 * self.R**2
        c3 = 4.0 * beta
        c2 = 4.0 * beta**2 + 2.0 * k - four * (1.0 - axial**2)
        c1 = 4.0 * beta * k - 2.0 * four * (beta - height * axial)
        c0 = k**2 - four * (alpha - height**2)
        n = len(origins)
        companion = np.zeros((n, 4, 4))
        companion[:, 1, 0] = 1.0
        companion[:, 2, 1] = 1.0
        companion[:, 3, 2] = 1.0
        companion[:, 0, 3] = -c0
        companion[:, 1, 3] = -c1
        companion[:, 2, 3] = -c2
        companion[:, 3, 3] = -c3
        roots = np.linalg.eigvals(companion)
        values = np.where(np.abs(roots.imag) < 1e-7, roots.real, np.inf)
        values.sort(axis=1)
        found = np.isfinite(values).sum(axis=1)
        values[found % 2 == 1] = np.inf
        return [(values[:, 0], values[:, 1]), (values[:, 2], values[:, 3])]


class Cone:
    """An exact cone from its collapsed pole row and its rim poles."""

    def __init__(self, path):
        grid = np.array(control_grid(path))
        rows = [grid[:, 0], grid[:, -1], grid[0, :], grid[-1, :]]
        self.apex = next(row[0] for row in rows if np.ptp(row, axis=0).max() < 1e-9)
        rim = next(
            row
            for row in rows
            if np.ptp(row, axis=0).max() > 1e-6
            and np.abs(row[0] - row[-1]).max() < 1e-9
        )
        points = rim[:-1:2]
        centre = points.mean(axis=0)
        self.R = float(np.sqrt(((points - centre) ** 2).sum(axis=1)).mean())
        axis = centre - self.apex
        self.H = float(np.sqrt(axis @ axis))
        self.a = axis / self.H  # Apex towards the base.

    def volume(self):
        return np.pi * self.R**2 * self.H / 3.0

    def spans(self, origins, d):
        q = origins - self.apex
        slope = (self.R / self.H) ** 2
        rate = d @ self.a
        depth = q @ self.a
        a = 1.0 - (1.0 + slope) * rate**2
        b = 2.0 * (q @ d - (1.0 + slope) * rate * depth)
        c = (q * q).sum(axis=1) - (1.0 + slope) * depth**2
        disc = b * b - 4.0 * a * c
        n = len(origins)
        at_apex, at_base = -depth / rate, (self.H - depth) / rate
        low, high = np.minimum(at_apex, at_base), np.maximum(at_apex, at_base)
        if a > 0.0:
            root = np.sqrt(np.maximum(disc, 0.0))
            low = np.maximum(low, (-b - root) / (2.0 * a))
            high = np.minimum(high, (-b + root) / (2.0 * a))
            high = np.where(disc > 0.0, high, -np.inf)
        else:
            root = np.sqrt(np.maximum(disc, 0.0))
            first = np.minimum((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))
            second = np.maximum((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))
            if rate > 0.0:
                low = np.where(disc > 0.0, np.maximum(low, second), low)
            else:
                high = np.where(disc > 0.0, np.minimum(high, first), high)
        return [(low, high), (np.full(n, np.inf), np.full(n, np.inf))]


class Ball:
    def __init__(self, centre, radius):
        self.c, self.r = np.array(centre, float), radius

    def spans(self, origins, d):
        q = origins - self.c
        b = q @ d
        disc = b * b - ((q * q).sum(axis=1) - self.r**2)
        root = np.sqrt(np.maximum(disc, 0.0))
        low, high = -b - root, -b + root
        return [(np.where(disc > 0, low, np.inf), np.where(disc > 0, high, np.inf))]


class Rod:
    """An infinite round bar through the origin."""

    def __init__(self, axis, radius):
        self.a, self.r = np.array(axis, float), radius

    def spans(self, origins, d):
        rate = d @ self.a
        side = d - rate * self.a
        q = origins - np.outer(origins @ self.a, self.a)
        a = side @ side
        b = q @ side
        disc = b * b - a * ((q * q).sum(axis=1) - self.r**2)
        root = np.sqrt(np.maximum(disc, 0.0))
        low, high = (-b - root) / a, (-b + root) / a
        return [(np.where(disc > 0, low, np.inf), np.where(disc > 0, high, np.inf))]


def shared_length(first, second):
    total = 0.0
    for low_a, high_a in first:
        for low_b, high_b in second:
            with np.errstate(invalid="ignore"):
                length = np.minimum(high_a, high_b) - np.maximum(low_a, low_b)
            total = total + np.where(np.isfinite(length) & (length > 0.0), length, 0.0)
    return total


def rays(centre, x, y, reach):
    e1, e2 = perpendicular(DIRECTION)
    return centre + x.reshape(-1, 1) * e1 + y.reshape(-1, 1) * e2 - DIRECTION * reach


def window(a, b, centre, half):
    """The rectangle of rays that can hit both solids, from a coarse pass, padded by two coarse pitches."""
    coarse = 256
    pitch = 2.0 * half / coarse
    across = -half + (np.arange(coarse) + 0.5) * pitch
    x, y = np.meshgrid(across, across)
    origins = rays(centre, x, y, 0.0)
    length = shared_length(a.spans(origins, DIRECTION), b.spans(origins, DIRECTION)).reshape(x.shape)
    hit_x, hit_y = x[length > 0.0], y[length > 0.0]
    return hit_x.min() - 2.0 * pitch, hit_x.max() + 2.0 * pitch, hit_y.min() - 2.0 * pitch, hit_y.max() + 2.0 * pitch


def one_shift(a, b, centre, half, box, grid, shift):
    pitch = max(box[1] - box[0], box[3] - box[2]) / grid
    across = box[0] + (np.arange(int(np.ceil((box[1] - box[0]) / pitch))) + shift[0]) * pitch
    down = box[2] + (np.arange(int(np.ceil((box[3] - box[2]) / pitch))) + shift[1]) * pitch
    total = 0.0
    for start in range(0, len(down), 64):
        x, y = np.meshgrid(across, down[start : start + 64])
        origins = rays(centre, x, y, 0.0)
        total += shared_length(a.spans(origins, DIRECTION), b.spans(origins, DIRECTION)).sum()
    return total * pitch * pitch


def measure(a, b, centre, half, grid, shifts):
    generator = np.random.default_rng(SEED)
    box = window(a, b, centre, half)
    values = np.array([one_shift(a, b, centre, half, box, grid, generator.random(2)) for _ in range(shifts)])
    return float(values.mean()), float(2.0 * values.std(ddof=1) / np.sqrt(shifts))


def row(name, value, bar, exact):
    print(
        f"{name:34s} {value:.7f} +- {bar:.2e}  exact {exact:.7f}  off {value - exact:+.2e}  off/bar {abs(value - exact) / bar:.2f}",
        flush=True,
    )


def main():
    grid = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
    shifts = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    inputs = os.environ.get("RHINO_FIXTURE_INPUTS", os.path.join(HERE, "inputs"))
    reference = os.path.join(os.path.dirname(inputs), "compact-reference.json")
    families = {f["family"]: f for f in json.load(open(reference))["families"]}
    print(
        f"rays {grid} x {grid}, {shifts} shifts, seed {SEED}, error bar 2 std / sqrt(shifts)"
    )

    only = len(sys.argv) > 3
    if not only:
        print("closed forms")
        row(
            "sphere lens",
            *measure(
                Ball((-3, 0, 0), 10), Ball((3, 0, 0), 10), np.zeros(3), 12.0, grid, shifts
            ),
            np.pi * (40 + 6) * (20 - 6) ** 2 / 12,
        )
        row(
            "Steinmetz solid",
            *measure(
                Rod((1, 0, 0), 10), Rod((0, 1, 0), 10), np.zeros(3), 18.0, grid, shifts
            ),
            16.0 / 3.0 * 1000.0,
        )

    for family, shape in (("Torus", Torus), ("Cones", Cone)):
        folder = inputs
        a = shape(os.path.join(folder, f"Input{family}-object-0.step"))
        b = shape(os.path.join(folder, f"Input{family}-object-1.step"))
        item = families[family]
        for index, solid in enumerate((a, b)):
            print(
                f"{family} {index}: R {solid.R:.10f} {'r' if shape is Torus else 'H'} {(solid.r if shape is Torus else solid.H):.10f}"
                f" closed form {solid.volume():.9f} rhino input {item['input_order'][index]['signed_volume']:.9f}"
            )
        reach = max(a.R + a.r, b.R + b.r) if shape is Torus else max(a.H, b.H)
        centre = (
            0.5 * (a.c + b.c)
            if shape is Torus
            else 0.5 * (a.apex + b.apex + a.a * a.H * 0.5 + b.a * b.H * 0.5)
        )
        half = 1.5 * reach
        if not only:
            row(
                family + " operand with itself",
                *measure(
                    a,
                    a,
                    a.c if shape is Torus else a.apex + a.a * a.H * 0.5,
                    1.5 * reach,
                    grid,
                    shifts,
                ),
                a.volume(),
            )
        value, bar = measure(a, b, centre, half, grid, shifts)
        rhino = next(
            o for o in item["output_references"] if o["operation"] == "intersection"
        )["reference"]["signed_volume"]
        print(
            f"{family} intersection {value:.7f} +- {bar:.2e}  rhino {rhino:.7f}  rhino off {rhino - value:+.3e} = {abs(rhino - value) / bar:.1f} bars",
            flush=True,
        )


main()
