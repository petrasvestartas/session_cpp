// occt_suite.h -- boolean battery harvested from the OCCT draw-harness test suite
// (tests/boolean/* and tests/bugs/modalg_*). Generated from the Tcl scripts; each
// cell is one OCCT regression case rebuilt with this kernel's primitives.
//   kind: box(sx sy sz, CENTERED at origin -- OCCT corner-origin boxes get +half-size
//         folded into tx ty tz), sphere(r), cylinder(r h, base z=0..h), cone(r h, base
//         z=0 apex z=h), torus(R r, centered, axis z) -- all matching OCCT p* commands.
//   xf: {tx ty tz ax ay az deg} applied as translation * single-axis rotation, exactly
//       like main_7.cpp xf_of(). OCCT trotate about a point p folds to T(p - R p) * R.
//   op: "cut" = A \ B, "common", "fuse". boptuc/bbop-3 cases (B \ A) are stored with
//       operands already swapped, so op is always A-relative.
//   src: test path under occt/tests/ (boptuc_* sources imply the operand swap).
#pragma once

struct OcctSuiteCell {
    const char* label;
    const char* kindA; double pA[3]; double xfA[7];
    const char* kindB; double pB[3]; double xfB[7];
    const char* op;
    const char* src;
};

static const OcctSuiteCell OCCT_SUITE[] = {
    {"cylxbox R1",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 0, 1, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/R1"},  // box 2x2 circumscribes cyl r1: 4 wall line-tangencies
    {"cylxbox R1",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 0, 1, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/R1"},
    {"cylxbox R1",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 0, 1, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/R1"},
    {"boxxcyl R1",
     "box", {2, 2, 2}, {0, 0, 1, 0, 0, 1, 0},
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/boptuc_simple/R1"},  // box minus inscribed cyl
    {"cylxbox R2",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 0, 1, 0, 0, 1, 45},
     "cut", "boolean/bopcut_simple/R2"},  // circumscribed box rotated 45
    {"cylxbox R2",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 0, 1, 0, 0, 1, 45},
     "common", "boolean/bopcommon_simple/R2"},
    {"cylxbox R3",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 1.5, 2}, {0, -0.25, 1, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/R3"},  // one wall chords the cylinder
    {"cylxbox R8",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1, 2, 2}, {-0.5, 0, 1, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/R8"},  // wall through axis: half cylinder
    {"cylxbox S6",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1.8, 1.8, 2}, {0.09999999999999998, 0.09999999999999998, 1, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/S6"},  // two chord walls, off-center
    {"cylxbox T4",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1.5, 1.5, 2}, {0.34150635094610965, -0.09150635094610969, 1, 0, 0, 1, 30},
     "cut", "boolean/bopcut_simple/T4"},  // box rotated 30, chords
    {"cylxbox U2",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1.6, 1.8, 2}, {0.09999999999999998, 0, 1, 0, 0, 1, 90},
     "cut", "boolean/bopcut_simple/U2"},  // non-square box rotated 90
    {"cylxbox U7",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 2, 1, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/U7"},  // external wall tangency: void common
    {"cylxbox U7",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {0, 2, 1, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/U7"},  // fuse across line tangency
    {"cylxbox V5",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 1}, {0, 0, 0.5, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/V5"},  // half-height box, base midway
    {"cylxbox V6",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 1}, {0, 0, 0.5, 0, 0, 1, 45},
     "cut", "boolean/bopcut_simple/V6"},  // half-height box rotated 45
    {"cylxbox X8",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1.5, 1.5, 1}, {0.34150635094610965, -0.09150635094610969, 0.5, 0, 0, 1, 30},
     "cut", "boolean/bopcut_simple/X8"},  // half-height box rotated 30
    {"cylxbox Z3",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 1}, {2, 0, 0.5, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/Z3"},  // external side tangency fuse
    {"cylxbox ZA6",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 1, 2}, {1, 0, 1, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZA6"},  // wall exactly through axis
    {"cylxbox ZA7",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 1, 2}, {0.8660254037844387, 0.49999999999999994, 1, 0, 0, 1, 30},
     "cut", "boolean/bopcut_simple/ZA7"},  // wall through axis, rotated 30
    {"cylxbox ZA8",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 2, 2}, {1, 1, 1, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZA8"},  // quarter cylinder (corner at axis)
    {"cylxbox ZB8",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {2, 1, 1}, {1, 0, 0.5, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZB8"},  // half cylinder, half height
    {"boxxcyl ZC7",
     "box", {2, 2, 2}, {0, 1, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZC7"},  // box sidesaddle on taller cylinder
    {"boxxcyl ZC7",
     "box", {2, 2, 2}, {0, 1, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZC7"},
    {"boxxcyl ZN3",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 6, 0}, {2, 2, -2, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZN3"},  // thin cyl pierces 4^3 box, protrudes both ends
    {"boxxcyl ZN4",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 2, 0}, {2, 2, 1, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZN4"},  // cyl caps flush with box top/bottom
    {"boxxcyl ZN5",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {2, 4, 0}, {4, 2, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZN5"},  // r2 cyl axis on box face: half embedded
    {"boxxcyl ZN5",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {2, 4, 0}, {4, 2, 0, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZN5"},
    {"boxxcyl ZN6",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {2, 4, 0}, {2, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZN6"},  // cyl axis on adjacent box face
    {"boxxcyl ZN8",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {4, 2, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZN8"},  // r1 cyl axis on box face
    {"boxxcyl ZO2",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {4, 3, 0, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZO2"},  // axis 1 off face, r1: external wall tangency, void
    {"boxxcyl ZO3",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {3, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZO3"},  // internal tangency to wall x=4
    {"boxxcyl ZO4",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0, 3, 0, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZO4"},  // internal tangency to wall y=4, fuse
    {"boxxcyl ZO5",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {1, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZO5"},  // internal tangency to wall x=0
    {"boxxcyl ZO6",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {4, 4, 0}, {-2, 2, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZO6"},  // r4 cyl face-size through box
    {"boxxcyl ZO7",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {4, 4, 0}, {-2, 2, 0, 0, 0, 1, 30},
     "cut", "boolean/bopcut_simple/ZO7"},  // same, seam rotated 30
    {"boxxcyl ZO9",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {3, 1, 4, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZO9"},  // cyl base coincident with box top face, off-center
    {"boxxcyl ZP1",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {1, 1, 4, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZP1"},  // cyl base coincident with box top face
    {"boxxcyl Z9",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {2, 6, 0}, {0, 0, -2, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/Z9"},  // r2 cyl through 4^3 box, both ends out
    {"boxxcyl Z9",
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cylinder", {2, 6, 0}, {0, 0, -2, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/Z9"},
    {"cylxbox Z9",
     "cylinder", {2, 6, 0}, {0, 0, -2, 0, 0, 1, 0},
     "box", {4, 4, 4}, {2, 2, 2, 0, 0, 1, 0},
     "cut", "boolean/boptuc_simple/Z9"},  // box minus through-cylinder
    {"boxxcyl A4",
     "box", {5, 5, 1}, {2.5, 2.5, 0.5, 0, 0, 1, 0},
     "cylinder", {2, 2, 0}, {2.5, 2.5, 0, 0, 0, 1, 0},
     "cut", "boolean/periodicity/A4"},  // thin plate, through hole
    {"cylxcyl J1",
     "cylinder", {20, 100, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {20, 100, 0}, {0, 0, 50, 0, 0, 1, 0},
     "common", "boolean/bcommon_simple/J1"},  // coaxial equal r, half overlap
    {"cylxcyl L8",
     "cylinder", {20, 100, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {20, 100, 0}, {0, 0, 50, 0, 0, 1, 0},
     "cut", "boolean/bcut_simple/L8"},
    {"cylxcyl L2",
     "cylinder", {20, 100, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {20, 100, 0}, {0, 0, 50, 0, 0, 1, 0},
     "fuse", "boolean/bfuse_simple/L2"},
    {"cylxcyl J1",
     "cylinder", {10, 20, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {5, 20, 0}, {5, 0, 10, 0, 0, 1, 0},
     "fuse", "boolean/bfuse_complex/J1"},  // parallel axes, internal tangency (r diff = offset)
    {"cylxcyl J5",
     "cylinder", {50, 150, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {50, 150, 0}, {-75, 0, 75, 0, 1, 0, 90},
     "fuse", "boolean/bfuse_complex/J5"},  // equal r50 crossed 90
    {"cylxcyl P4",
     "cylinder", {10, 20, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {6, 15, 0}, {0, 0, 10, 1, 0, 0, 90},
     "fuse", "boolean/bfuse_complex/P4"},  // perpendicular T joint
    {"cylxcyl ZC8",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 1, 0}, {0, 0, 2, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZC8"},  // small cyl stacked on cap, face coincident
    {"cylxcyl ZC8",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 1, 0}, {0, 0, 2, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZC8"},
    {"cylxcyl ZC9",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 1, 0}, {0, 0, 1, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZC9"},  // small cyl half sunk in cap
    {"cylxcyl ZD1",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 1, 0}, {0, 0, 0.5, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZD1"},
    {"cylxcyl ZD2",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZD2"},  // coaxial nested equal height: tube
    {"cylxcyl ZD2",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZD2"},
    {"cylxcyl ZD2",
     "cylinder", {0.5, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/boptuc_simple/ZD2"},  // inner minus outer: void
    {"cylxcyl ZD3",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 3, 0}, {0, 0, -1, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZD3"},  // coaxial nested, inner protrudes both ends
    {"cylxcyl ZD4",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 2, 0}, {0, 0, 2, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZD4"},  // stacked equal cyls, cap coincident
    {"cylxcyl ZD5",
     "cylinder", {1, 2, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 2, 0}, {0, 0, 2, 0, 0, 1, 90},
     "fuse", "boolean/bopfuse_simple/ZD5"},  // same stack, seam rotated 90
    {"cylxcyl ZD8",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0, 2, 1.9999999999999998, 1, 0, 0, 90},
     "cut", "boolean/bopcut_simple/ZD8"},  // equal r crossed 90 through mid
    {"cylxcyl ZD8",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0, 2, 1.9999999999999998, 1, 0, 0, 90},
     "common", "boolean/bopcommon_simple/ZD8"},
    {"cylxcyl ZD8",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0, 2, 1.9999999999999998, 1, 0, 0, 90},
     "fuse", "boolean/bopfuse_simple/ZD8"},
    {"cylxcyl ZE3",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 4, 0}, {0, 2, 1.9999999999999998, 1, 0, 0, 90},
     "cut", "boolean/bopcut_simple/ZE3"},  // r0.5 pierces r1 at 90
    {"cylxcyl ZE7",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {0.5, 4, 0}, {0.5, 2, 1.9999999999999998, 1, 0, 0, 90},
     "cut", "boolean/bopcut_simple/ZE7"},  // r0.5 at 90, internally tangent to wall
    {"cylxcyl ZF2",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0.5000000000000001, 0.8660254037844386, 0, 0, 0, 1, -60},
     "fuse", "boolean/bopfuse_simple/ZF2"},  // parallel axes offset 1, seam -60
    {"cylxcyl ZF3",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {0.5000000000000001, 0.8660254037844386, 0, 0, 0, 1, 180},
     "cut", "boolean/bopcut_simple/ZF3"},  // parallel axes offset 1, seam 180
    {"cylxcyl ZF4",
     "cylinder", {1, 4, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {1, 4, 0}, {1, 0, 0, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZF4"},  // parallel axes offset 1
    {"cylxcyl 25657_1",
     "cylinder", {140, 220, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {206, 120, 0}, {100, 100, 120, 0, 0, 1, 0},
     "cut", "bugs/modalg_5/bug25657_1"},  // large cyls, partial-height overlap
    {"cylxcyl 28828_4",
     "cylinder", {10, 50, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {15, 50, 0}, {25, 0, 0, 0, 1, 0, -90},
     "fuse", "bugs/modalg_7/bug28828_4"},  // perpendicular T, r10 vs r15
    {"cylxsph ZH5",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {4, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZH5"},  // sphere r=cyl r centered on cap: contact at cap edge circle
    {"cylxsph ZH5",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {4, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZH5"},
    {"cylxsph ZH5",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {4, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZH5"},
    {"sphxcyl ZH5",
     "sphere", {4, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/boptuc_simple/ZH5"},  // sphere minus cylinder
    {"cylxsph ZH9",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {4, 0, 0}, {0, 0, 8, 1, 0, 0, 90},
     "cut", "boolean/bopcut_simple/ZH9"},  // same, sphere seam tilted 90 about x
    {"cylxsph ZI4",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {2, 0, 0}, {0, 0, 8, 1, 0, 0, 90},
     "cut", "boolean/bopcut_simple/ZI4"},  // small sphere half sunk in cap
    {"cylxsph ZI8",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {6, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZI8"},  // big sphere swallows cap rim
    {"cylxsph ZI8",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {6, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZI8"},
    {"sphxcyl ZI8",
     "sphere", {6, 0, 0}, {0, 0, 8, 0, 0, 1, 0},
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/boptuc_simple/ZI8"},
    {"cylxsph ZJ3",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {6, 0, 0}, {0, 0, 8, 0, 1, 0, 90},
     "common", "boolean/bopcommon_simple/ZJ3"},  // big sphere, seam about y
    {"sphxcyl 25939",
     "sphere", {5, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {83, 100, 0}, {-83, 0, -50, 0, 0, 1, 0},
     "cut", "bugs/modalg_7/bug25939"},  // r83 wall passes through r5 sphere center: near-flat graze
    {"cylxsph 33541",
     "cylinder", {1, 1, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {1, 0, 0}, {1, 1, 1, 0, 0, 1, 0},
     "cut", "bugs/modalg_8/bug33541"},  // unit cyl vs unit sphere at corner (1,1,1)
    {"sphxcyl 32502",
     "sphere", {10, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cylinder", {3, 40, 0}, {0, -20, 7, 1, 0, 0, -90},
     "fuse", "bugs/modalg_7/bug32502"},  // skewer cyl through sphere across seam
    {"cylxtor ZL2",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {4, 1, 0}, {0, 0, 4, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/ZL2"},  // torus centerline on cyl wall (R=r_cyl)
    {"cylxtor ZL2",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {4, 1, 0}, {0, 0, 4, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/ZL2"},
    {"cylxtor ZL2",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {4, 1, 0}, {0, 0, 4, 0, 0, 1, 0},
     "fuse", "boolean/bopfuse_simple/ZL2"},
    {"torxcyl ZL2",
     "torus", {4, 1, 0}, {0, 0, 4, 0, 0, 1, 0},
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/boptuc_simple/ZL2"},  // torus minus cylinder
    {"cylxtor ZL3",
     "cylinder", {4, 8, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {4, 1, 0}, {0, 0, 4, 0, 0, 1, 90},
     "cut", "boolean/bopcut_simple/ZL3"},  // same, torus seam rotated 90
    {"cylxtor 31858_1",
     "cylinder", {500, 200, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {449.367136080235, 50.6328639197654, 0}, {0, 0, 200, 0, 0, 1, 0},
     "common", "bugs/modalg_7/bug31858_1"},  // torus R+r = cyl r: internal equator tangency at cyl top
    {"cylxtor 31858_1",
     "cylinder", {500, 200, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {449.367136080235, 50.6328639197654, 0}, {0, 0, 200, 0, 0, 1, 0},
     "fuse", "bugs/modalg_7/bug31858_1"},  // torus R+r = cyl r: internal equator tangency at cyl top
    {"cylxtor 31858_1",
     "cylinder", {500, 200, 0}, {0, 0, 0, 0, 0, 1, 0},
     "torus", {449.367136080235, 50.6328639197654, 0}, {0, 0, 200, 0, 0, 1, 0},
     "cut", "bugs/modalg_7/bug31858_1"},  // torus R+r = cyl r: internal equator tangency at cyl top
    {"torxcyl 31858_1",
     "torus", {449.367136080235, 50.6328639197654, 0}, {0, 0, 200, 0, 0, 1, 0},
     "cylinder", {500, 200, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "bugs/modalg_7/bug31858_1"},  // torus R+r = cyl r: internal equator tangency at cyl top
    {"sphxbox A1",
     "sphere", {1, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "cut", "boolean/bcut_simple/A1"},  // unit box corner at sphere center
    {"sphxbox A1",
     "sphere", {1, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "common", "boolean/bcommon_simple/A1"},
    {"sphxbox A1",
     "sphere", {1, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "fuse", "boolean/bfuse_simple/A1"},
    {"sphxbox A4",
     "sphere", {1, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {1, 1, 1}, {-0.49999999999999994, 0.5, 0.49999999999999994, 0, 1, 0, 90},
     "cut", "boolean/bcut_simple/A4"},  // same, box arrives rotated 90 about y
    {"boxxsph G1",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "sphere", {1, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "boolean/bcut_simple/G1"},  // box minus sphere-at-corner
    {"sphxbox 7626_1",
     "sphere", {10, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {20, 20, 20}, {10, 14.142135623730951, 10, 0, 0, 1, 45},
     "cut", "bugs/modalg_4/bug7626_1"},  // sphere r10 vs 45-rotated box 20^3
    {"sphxbox 7626_2",
     "sphere", {10, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {20, 20, 20}, {10, 14.142135623730951, 10, 0, 0, 1, 45},
     "common", "bugs/modalg_4/bug7626_2"},
    {"boxxsph 24706",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "sphere", {2, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "cut", "bugs/modalg_5/bug24706"},  // small sphere centered at box corner
    {"sphxbox 32470",
     "sphere", {10, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {20, 100, 100}, {14.142135623730951, 0, 10, 0, 1, 0, -45},
     "cut", "bugs/modalg_7/bug32470"},  // rotated box face plane grazes sphere
    {"sphxsph 23855",
     "sphere", {10, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "sphere", {10, 0, 0}, {0, 0, 0, 0, 0, 1, 0},
     "common", "bugs/modalg_5/bug23855"},  // identical spheres: same-domain common
    {"sphxsph 33570",
     "sphere", {4, 0, 0}, {0, 5.5, 18, 0, 0, 1, 0},
     "sphere", {1, 0, 0}, {0, 4.5, 14, 0, 0, 1, 0},
     "cut", "bugs/modalg_8/bug33570"},  // small sphere lens-overlaps big sphere
    {"conexbox 28828_19",
     "cone", {40, 70, 0}, {0, 0, 0, 0, 0, 1, 0},
     "box", {100, 100, 40}, {0, 0, 0, 0, 0, 1, 0},
     "fuse", "bugs/modalg_7/bug28828_19"},  // cone base plane inside box, apex above
    {"boxxbox A1",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "cut", "boolean/bopcut_simple/A1"},  // identical boxes: cut = void
    {"boxxbox A1",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "common", "boolean/bopcommon_simple/A1"},  // identical boxes: common = box
    {"boxxbox C9",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {0.5, 0.5, 1}, {0, 0.3535533905932738, 0.5, 0, 0, 1, 45},
     "cut", "boolean/bopcut_simple/C9"},  // half-size slab rotated 45
    {"boxxbox D3",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {1.5, 0.25, 1}, {0.4419417382415923, 0.6187184335382292, 0.5, 0, 0, 1, 45},
     "cut", "boolean/bopcut_simple/D3"},  // 0.25-thin slab rotated 45: sliver cuts
    {"boxxbox D5",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {0.5, 0.5, 1}, {0.09150635094610969, 0.34150635094610965, 0.5, 0, 0, 1, 30},
     "cut", "boolean/bopcut_simple/D5"},  // slab rotated 30
    {"boxxbox D6",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {0.5, 0.5, 1}, {-0.09150635094610962, 0.3415063509461097, 0.5, 0, 0, 1, 60},
     "common", "boolean/bopcommon_simple/D6"},  // slab rotated 60
    {"boxxbox D9",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {0.5, 1.7, 1}, {0.49044087422949645, 0.737880579013303, 0.5, 0, 0, 1, -50},
     "cut", "boolean/bopcut_simple/D9"},  // tall slab rotated -50
    {"boxxbox G7",
     "box", {1, 1, 1}, {0.5, 0.5, 0.5, 0, 0, 1, 0},
     "box", {0.5, 0.5, 1}, {0, 0.6035533905932737, 0.5, 0, 0, 1, 45},
     "fuse", "boolean/bopfuse_simple/G7"},  // rotated 45, edge on face
    {"boxxbox A7",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "box", {10, 10, 10}, {10, 10, 10, 0, 0, 1, 0},
     "fuse", "boolean/history/A7"},  // vertex-to-vertex contact fuse
    {"boxxbox A8",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "box", {10, 10, 10}, {10, 5, 5, 0, 0, 1, 0},
     "fuse", "boolean/history/A8"},  // half-face overlap fuse
    {"boxxbox 25722",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "box", {10, 10, 10}, {15.0001, 5, 5, 0, 0, 1, 0},
     "fuse", "bugs/modalg_5/bug25722"},  // 1e-4 face gap; source runs it with bfuzzyvalue 1e-4 and 0
    {"boxxbox 25477_1",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "box", {10, 10, 10}, {15.00001, 5, 5, 0, 0, 1, 0},
     "fuse", "bugs/modalg_5/bug25477_1"},  // 1e-5 face gap under bfuzzyvalue 2e-5: fuzzy weld
    {"boxxbox 25478_2",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "box", {10, 10, 12}, {10, 10, 5, 0, 0, 1, 0},
     "cut", "bugs/modalg_7/bug25478_2"},  // taller box shares two faces
    {"boxxbox 27383_1",
     "box", {10, 10, 10}, {5, 5, 5, 0, 0, 1, 0},
     "box", {10, 10, 10}, {10, 10, 5, 0, 0, 1, 0},
     "fuse", "bugs/modalg_6/bug27383_1"},  // edge-to-edge contact fuse
    {"boxxbox B1",
     "box", {381, 508, 762}, {508, 508, -317.5, 0, 1, 0, 90},
     "box", {254, 304.8, 762}, {635, 508, -317.5, 1, 0, 0, -90},
     "fuse", "boolean/gdml_public/B1"},  // boxes rotated about y and x
    {"boxxbox C4",
     "box", {40, 40, 5}, {20, 20, 2.5, 0, 0, 1, 0},
     "box", {10, 10, 14.197}, {20, 20, 12.0985, 0, 0, 1, 90},
     "fuse", "boolean/gdml_public/C4"},  // plate + 90-rotated post
    {"boxxbox E2",
     "box", {3, 3, 3}, {1.5, 1.5, 1.5, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 3.5, 0.5, 0, 0, 1, 0},
     "fuse", "boolean/bfuse_simple/E2"},  // 3^3 vs unit box, face contact only
    {"boxxbox G7",
     "box", {3, 3, 3}, {1.5, 1.5, 1.5, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 3.5, 0.5, 0, 0, 1, 0},
     "cut", "boolean/bcut_simple/G7"},
    {"boxxbox I5",
     "box", {3, 3, 3}, {1.5, 1.5, 1.5, 0, 0, 1, 0},
     "box", {1, 1, 1}, {0.5, 3.5, 0.5, 0, 0, 1, 0},
     "common", "boolean/bcommon_simple/I5"},  // face contact only: void common
};

static const int OCCT_SUITE_COUNT = 120;
