"""Rigid-motion transform of a STEP file (for metamorphic equivariance testing).

Rotating/translating BOTH operands of a boolean must leave every measured quantity
(volume, face count, solid count) unchanged — a kernel that only works axis-aligned
fails this. The kernel has no "transform the whole scene" switch, so the motion is
applied to the INPUT STEP instead: every 3-coordinate CARTESIAN_POINT is mapped
p -> R p + t and every 3-coordinate DIRECTION d -> R d. 2-coordinate entities are
pcurve/parameter-space data and are left untouched (they are invariant by
construction), as are all scalar parameters (radii, angles, knots), which a rigid
motion preserves.

Usage: python corpus/step_rigid.py in.step out.step --axis 0.3 0.5 0.81 --deg 37
                                                    [--trans 1 2 3]
"""
import argparse
import math
import re

PT = re.compile(r"(CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\()([^)]*)(\))")
DR = re.compile(r"(DIRECTION\s*\(\s*'[^']*'\s*,\s*\()([^)]*)(\))")


def fmt(v):
    if v == 0.0:
        return "0."
    s = "%.15G" % v
    if "E" in s:
        m, e = s.split("E")
        if "." not in m:
            m += "."
        return m + "E" + e
    if "." not in s:
        s += "."
    return s


def rot_matrix(axis, deg):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("degenerate axis")
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(math.radians(deg)), math.sin(math.radians(deg)), None
    C = 1.0 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def apply(R, v, t=(0.0, 0.0, 0.0)):
    return [sum(R[i][j] * v[j] for j in range(3)) + t[i] for i in range(3)]


def transform_text(text, R, t):
    def sub(m, translate):
        nums = [x.strip() for x in m.group(2).split(",")]
        if len(nums) != 3:
            return m.group(0)                 # 2D (pcurve) data: untouched
        try:
            v = [float(x) for x in nums]
        except ValueError:
            return m.group(0)
        w = apply(R, v, t if translate else (0.0, 0.0, 0.0))
        return m.group(1) + ",".join(fmt(c) for c in w) + m.group(3)

    text = PT.sub(lambda m: sub(m, True), text)
    text = DR.sub(lambda m: sub(m, False), text)
    return text


def transform_file(src, dst, axis, deg, trans=(0.0, 0.0, 0.0)):
    R = rot_matrix(axis, deg)
    out = transform_text(open(src, errors="replace").read(), R, trans)
    open(dst, "w").write(out)
    return dst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--axis", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    ap.add_argument("--deg", type=float, default=0.0)
    ap.add_argument("--trans", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    a = ap.parse_args()
    transform_file(a.src, a.dst, a.axis, a.deg, a.trans)
    print(a.dst)


if __name__ == "__main__":
    main()
