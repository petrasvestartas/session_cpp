"""Volumes cut from a vault strip, r 1880 to 2000 round the y axis, by tools along a radius. Shares nothing with the kernel.

A line along the radius at sideways offset s stays in the shell for length sqrt(2000^2 - s^2) - sqrt(1880^2 - s^2),
so each volume is that length integrated across the tool's section, by Gauss-Legendre.
"""

import numpy as np

X, W = np.polynomial.legendre.leggauss(64)
OUTER, INNER = 2000.0, 1880.0


def length(s, low=None):
    start = np.sqrt(INNER**2 - s**2) if low is None else low
    return np.sqrt(OUTER**2 - s**2) - start


def across(half, low=None):
    return half * (W * length(half * X, low)).sum()


def disc(radius):
    # Polar rule over the disc: Gauss in rho, even steps in the angle, exact for smooth periodic integrands.
    rho = 0.5 * radius * (X + 1.0)
    angles = 2.0 * np.pi * (np.arange(256) + 0.5) / 256
    total = 0.0
    for r, w in zip(rho, W):
        total += (
            0.5 * radius * w * r * length(r * np.cos(angles)).sum() * 2.0 * np.pi / 256
        )
    return total


print(f"whole            {np.pi * 0.25 * (OUTER**2 - INNER**2) * 600:.6f}")
print(f"drill radial     {disc(30.0):.9f}")
print(f"mortise through  {80.0 * across(60.0):.9f}")
print(f"mortise blind    {80.0 * across(60.0, 1940.0):.9f}")
print(f"edge notch       {60.0 * across(50.0):.9f}")
print(f"pin along y      {np.pi * 64.0 * 50.0:.6f}")
