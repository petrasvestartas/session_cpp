// main_7 — fast boolean dev loop. Builds the 15-pair x 3-op primitive matrix, runs our BRep
// boolean, compares against the OCCT oracle, and prints a scorecard with per-cell timing.
//
//   cmake --build build --config Release --target main_7 --parallel 8
//   ./build/Release/main_7.exe ["substr filter"] [--refresh]
//   SESSION_BOOL_SHARED_EDGES=1 ./build/Release/main_7.exe        # exercise the shared-edge path
//
// OCCT reference values are DETERMINISTIC for these fixed placements, so they are cached full-
// precision in validation/occt_cache.txt. The first run (or --refresh) shells oracle.exe to fill
// the cache (~20 min, oracle-subprocess-bound); every run after reads the cache (~30s, our booleans
// only). Pass --refresh to regenerate. Delete the cache file to force a rebuild.
//
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <cmath>
#include <iomanip>
#include <set>
#include "brep.h"
#include "brep_massprops.h"
#include "closest.h"
#include "intersection.h"
#include "file_step.h"
#include "xform.h"
#include "tolerance.h"
#include "occt_suite.h"
using namespace session_cpp;

namespace {

// NAKED/NON-MANIFOLD COUNT THAT EXCLUDES DEGENERATE EDGES, matching BRep::is_solid() and
// OCCT. A sphere pole and a cone apex are zero-length edges: watertight by construction, and
// excluded from manifold checks. Counting them as naked produces FALSE FAILURES on every
// result containing a pole or apex -- measured: sphere x cylinder at 0 deg scored "3 faces /
// 2 naked / FAIL" while actually being is_solid=1 with volume 15.061795, the exact analytic
// value. Any corpus scored with a naive nt==1 count is wrong wherever a pole survives.
void count_naked_nonmani(const BRep& X, int& naked, int& nonmani) {
    naked = 0; nonmani = 0;
    for (const auto& e : X.m_topology_edges) {
        int nt = (int)e.trim_indices.size();
        if (nt == 2 || nt == 0) continue;
        bool degenerate = false;
        int ci = e.curve_3d_index;
        if (ci >= 0 && ci < (int)X.m_curves_3d.size()) {
            const NurbsCurve& c = X.m_curves_3d[ci];
            auto dc = c.domain();
            double len = 0.0; Point prev = c.point_at(dc.first);
            for (int k = 1; k <= 8; ++k) {
                Point q = c.point_at(dc.first + (dc.second - dc.first) * k / 8.0);
                len += prev.distance(q); prev = q;
            }
            double diag = 0.0;
            if (!X.m_vertices.empty()) {
                double mn[3] = {1e300,1e300,1e300}, mx[3] = {-1e300,-1e300,-1e300};
                for (const auto& p : X.m_vertices)
                    for (int k = 0; k < 3; ++k) { mn[k]=std::min(mn[k],p[k]); mx[k]=std::max(mx[k],p[k]); }
                diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0])+(mx[1]-mn[1])*(mx[1]-mn[1])+(mx[2]-mn[2])*(mx[2]-mn[2]));
            }
            degenerate = (len <= std::max(diag * 1e-7, 1e-12));
        }
        if (degenerate) continue;
        if (nt == 1) ++naked; else ++nonmani;
    }
}

// PER-FACE VALIDITY (session C's splitter audit: the standout gate -- caught 2 of 32 broken
// faces on y30 with ZERO false positives on the verified reference's 32 faces, and found a
// second broken face that wire analysis alone missed). Two independent checks:
//   (a) OUTER WIRE CLOSED  -- the outer loop's trim chain must close on itself.
//   (b) TRIMMED AREA <= UNTRIMMED PATCH AREA -- a trimmed face cannot have more area than the
//       surface patch it lives on. y30's f3 measured ratio 1.9806 (35.379663 on a 17.862850
//       patch), which no tolerance argument can explain. Area catches what volume cannot:
//       y30's result area is +11.2% over the reference while its volume looks plausible.
// Returns {invalid_face_count, total_trimmed_area}. Numeric double integral of |Su x Sv|,
// masked by point-in-trim, so it needs no new kernel API.
std::pair<int,double> face_validity_report(const BRep& X, bool verbose) {
    int n_bad = 0; double area_tot = 0.0;
    for (int fi = 0; fi < (int)X.m_faces.size(); ++fi) {
        const auto& F = X.m_faces[fi];
        if (F.surface_index < 0 || F.surface_index >= (int)X.m_surfaces.size()) continue;
        const NurbsSurface& S = X.m_surfaces[F.surface_index];
        auto du = S.domain(0); auto dv = S.domain(1);
        // (a) outer wire closure, measured in 3D on the trim curves
        bool wire_open = false;
        for (int li : F.loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            if (X.m_loops[li].type != BRepLoopType::Outer) continue;
            std::vector<Point> ends;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int c2 = X.m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                const NurbsCurve& pc = X.m_curves_2d[c2];
                auto dc = pc.domain();
                Point a = pc.point_at(dc.first), b = pc.point_at(dc.second);
                ends.push_back(S.point_at(a[0], a[1]));
                ends.push_back(S.point_at(b[0], b[1]));
            }
            if (ends.size() >= 4) {
                double diag = 0; for (auto& p : ends) diag = std::max(diag, p.distance(ends[0]));
                double tol = std::max(1e-9, diag * 1e-4);
                // every endpoint must be matched by another endpoint (chain closes)
                for (size_t k = 0; k < ends.size() && !wire_open; ++k) {
                    double best = 1e300;
                    for (size_t j = 0; j < ends.size(); ++j)
                        if (j / 2 != k / 2) best = std::min(best, ends[k].distance(ends[j]));
                    if (best > tol) wire_open = true;
                }
            }
        }
        // (b) trimmed vs untrimmed area
        std::vector<std::array<double,2>> poly;
        for (int li : F.loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            if (X.m_loops[li].type != BRepLoopType::Outer) continue;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int c2 = X.m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                const NurbsCurve& pc = X.m_curves_2d[c2];
                auto dc = pc.domain();
                for (int k = 0; k < 16; ++k) {
                    Point q = pc.point_at(dc.first + (dc.second-dc.first)*k/16.0);
                    poly.push_back({q[0], q[1]});
                }
            }
        }
        auto inside = [&](double u, double v) {
            if (poly.size() < 3) return true;
            bool in = false;
            for (size_t a2 = 0, b2 = poly.size()-1; a2 < poly.size(); b2 = a2++)
                if (((poly[a2][1] > v) != (poly[b2][1] > v)) &&
                    (u < (poly[b2][0]-poly[a2][0])*(v-poly[a2][1])/(poly[b2][1]-poly[a2][1]+1e-30)+poly[a2][0]))
                    in = !in;
            return in;
        };
        const int NG = 40;
        double du_s = (du.second-du.first)/NG, dv_s = (dv.second-dv.first)/NG;
        double a_full = 0, a_trim = 0;
        for (int i = 0; i < NG; ++i)
            for (int j = 0; j < NG; ++j) {
                double u = du.first + du_s*(i+0.5), v = dv.first + dv_s*(j+0.5);
                // evaluate() returns [S, Sv, Su] for num_derivs=1 (k,l-loop order)
                std::vector<Vector> d = S.evaluate(u, v, 1);
                if (d.size() < 3) continue;
                const Vector& Sv = d[1];
                const Vector& Su = d[2];
                Vector cr(Su[1]*Sv[2]-Su[2]*Sv[1], Su[2]*Sv[0]-Su[0]*Sv[2], Su[0]*Sv[1]-Su[1]*Sv[0]);
                double da = cr.magnitude() * du_s * dv_s;
                a_full += da;
                if (inside(u, v)) a_trim += da;
            }
        area_tot += a_trim;
        bool area_bad = (a_full > 1e-12) && (a_trim / a_full > 1.02);   // 2% slack for the mask
        if (wire_open || area_bad) {
            ++n_bad;
            if (verbose)
                std::printf("   [FACEBAD] f%d %s%s trimmed %.6f patch %.6f ratio %.4f\n",
                            fi, wire_open ? "OPEN-WIRE " : "", area_bad ? "AREA>PATCH" : "",
                            a_trim, a_full, a_full > 1e-12 ? a_trim/a_full : 0.0);
        }
    }
    return {n_bad, area_tot};
}

// SHELL COUNT: connected components of faces linked by shared topology edges. is_solid() only
// asserts "every edge has 2 trims", which a result that has come APART into several separately
// closed shells still satisfies (an independent OCCT read of the shipped result set found 2-4
// shells on every rotated config except z30x20 -- invisible to the naked-edge metric).
int shell_count_of(const BRep& X) {
    int nf = (int)X.m_faces.size();
    if (nf == 0) return 0;
    std::vector<int> par(nf);
    for (int i = 0; i < nf; ++i) par[i] = i;
    std::function<int(int)> find = [&](int a) { return par[a] == a ? a : par[a] = find(par[a]); };
    std::map<int, int> first_face;
    for (int fi = 0; fi < nf; ++fi)
        for (int li : X.m_faces[fi].loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int ei = X.m_trims[ti].edge_index;
                if (ei < 0) continue;
                auto it = first_face.find(ei);
                if (it == first_face.end()) first_face[ei] = fi;
                else par[find(it->second)] = find(fi);
            }
        }
    std::set<int> comps;
    for (int fi = 0; fi < nf; ++fi) comps.insert(find(fi));
    return (int)comps.size();
}

struct Place { std::string kind; std::vector<double> p; std::array<double,7> xf; };  // xf: tx ty tz ax ay az deg
struct Ref { double vol; int nf; };

// Mirrors brep_test.cpp:783-792 and validation/compare_boolean.py.
static const std::array<double,7> ID = {0,0,0, 0,0,1, 0};
std::map<std::string, Place> placements() {
    return {
        {"box",   {"box",      {4,4,4},   ID}},
        {"sph",   {"sphere",   {2.5},     ID}},
        {"cyl",   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}},
        {"cone",  {"cone",     {2.0,4.0}, {0,0,-2, 0,0,1, 0}}},
        {"tor",   {"torus",    {2.0,0.8}, ID}},
        {"box2",  {"box",      {2,2,2},   {2,0,0, 0,0,1, 0}}},
        {"sph2",  {"sphere",   {2.0},     {2,0,0, 0,0,1, 0}}},
        {"cyl2",  {"cylinder", {1.5,6},   {-3,0,0, 0,1,0, 90}}},
        {"cone2", {"cone",     {2.0,4.0}, {0,0,2, 1,0,0, 180}}},
        {"tor2",  {"torus",    {2.0,0.8}, {2,0,0, 0,0,1, 0}}},
        // 45-deg tilted cone piercing the box bottom + one wall: every box-plane section is
        // an analytic ELLIPSE (45 deg > half-angle 24.2 deg; r=1.8 never reaches the y walls).
        // Upright cones vs box walls give plane-PARALLEL-TO-AXIS hyperbolas -- a known
        // analytic-SSI gap (IntAna_QuadQuadGeo port pending), kept out of the gate.
        {"cone3", {"cone",     {1.8,4.0}, {0,0,-2, 0,1,0, 45}}},
        {"tor3",  {"torus",    {2.0,0.8}, {0,0,-1, 0,0,1, 0}}},  // tube crosses the cone surface
        // Oriented battery: nothing axis-aligned about these -- deep penetrations with the
        // second operand rotated in space, so sections leave the iso/axis-aligned comfort zone.
        {"cylR",  {"cylinder", {1.5,6},   {-1,0,-1.8, 0,1,0, 45}}},  // 45-deg skew, axes cross at (0,0,-0.8)
        {"boxR",  {"box",      {4,4,4},   {2,1,0, 0,0,1, 30}}},      // rotated 30 deg about z, offset
        {"torR",  {"torus",    {2.0,0.8}, {0,0,0, 1,0,0, 45}}},      // tilted torus
        {"coneR", {"cone",     {2.0,4.0}, {0,-2.8,0, 1,0,0, -90}}},  // cone on its side, axis +y
    };
}

// (label, keyA, keyB)
std::vector<std::array<std::string,3>> pairs() {
    return {
        {"box  x box ", "box",  "box2"}, {"box  x sph ", "box",  "sph"},
        {"box  x cone", "box",  "cone"}, {"box  x cyl ", "box",  "cyl"},
        {"box  x tor ", "box",  "tor"},  {"sph  x sph ", "sph",  "sph2"},
        {"sph  x cone", "sph",  "cone"}, {"sph  x cyl ", "sph",  "cyl"},
        {"sph  x tor ", "sph",  "tor"},  {"cone x cone", "cone", "cone2"},
        {"cone x cyl ", "cone", "cyl"},  {"cone x tor3", "cone", "tor3"},
        {"cyl  x cyl ", "cyl",  "cyl2"}, {"cyl  x tor ", "cyl",  "tor"},
        {"tor  x tor ", "tor",  "tor2"},
        {"cyl  x cylR", "cyl",  "cylR"}, {"box  x boxR", "box",  "boxR"},
        {"boxR x sph ", "boxR", "sph"},  {"box  x torR", "box",  "torR"},
        {"coneRx cyl ", "coneR","cyl"},  {"boxR x cyl ", "boxR", "cyl"},
    };
}

// Classic BRep-kernel edge-case configurations (OCCT bop-grid style): same-domain
// (A op A), coincident faces (full / partial / coplanar-overlap), edge contact,
// vertex contact, containment (void result), disjoint, tangencies (point / line /
// inscribed), flush caps, linked-but-disjoint. SESSION_EDGE=1 runs this grid.
std::map<std::string, Place> edge_placements() {
    return {
        {"ebox",   {"box",      {4,4,4},   ID}},
        {"eboxT",  {"box",      {4,4,4},   {4,0,0, 0,0,1, 0}}},   // full-face contact at x=2
        {"eboxP",  {"box",      {2,2,2},   {3,0,0, 0,0,1, 0}}},   // partial-face contact at x=2
        {"eboxC",  {"box",      {4,4,4},   {2,2,0, 0,0,1, 0}}},   // overlap, z-faces coplanar
        {"eboxE",  {"box",      {4,4,4},   {4,4,0, 0,0,1, 0}}},   // edge contact (x=2,y=2)
        {"eboxV",  {"box",      {4,4,4},   {4,4,4, 0,0,1, 0}}},   // vertex contact (2,2,2)
        {"eboxIN", {"box",      {2,2,2},   ID}},                  // strictly inside ebox
        {"eboxD",  {"box",      {2,2,2},   {6,0,0, 0,0,1, 0}}},   // disjoint
        {"esph",   {"sphere",   {2.5},     ID}},
        {"esphX",  {"sphere",   {1.0},     {3.5,0,0, 0,0,1, 0}}}, // external tangency at (2.5,0,0)
        {"esphY",  {"sphere",   {1.0},     {1.5,0,0, 0,0,1, 0}}}, // internal tangency at (2.5,0,0)
        {"esphZ",  {"sphere",   {2.0},     ID}},                  // inscribed in ebox (6 tangencies)
        {"ecyl",   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}},
        {"ecylP",  {"cylinder", {1.5,6},   {3,0,-3, 0,0,1, 0}}},  // parallel axes, line tangency x=1.5
        {"ecylF",  {"cylinder", {1.5,4},   {0,0,-2, 0,0,1, 0}}},  // caps flush with ebox z-faces
        {"ecylT",  {"cylinder", {0.8,6},   {0,0,-3, 0,0,1, 0}}},  // through etor hole, no contact
        {"ecylO",  {"cylinder", {1.5,6},   {2,0,-3, 0,0,1, 0}}},  // parallel axes, PROPER overlap
        {"eboxC3", {"box",      {4,4,4},   {2,2,2, 0,0,1, 0}}},   // overlap shifted in ALL 3 axes
        {"etor",   {"torus",    {2.0,0.8}, ID}},
    };
}
std::vector<std::array<std::string,3>> edge_pairs() {
    return {
        {"eq boxbox  ", "ebox", "ebox"},  {"eq sphsph  ", "esph", "esph"},
        {"eq cylcyl  ", "ecyl", "ecyl"},  {"eq tortor  ", "etor", "etor"},
        {"face full  ", "ebox", "eboxT"}, {"face part  ", "ebox", "eboxP"},
        {"face copl  ", "ebox", "eboxC"}, {"edge touch ", "ebox", "eboxE"},
        {"vert touch ", "ebox", "eboxV"}, {"contain    ", "ebox", "eboxIN"},
        {"disjoint   ", "ebox", "eboxD"}, {"sph tanext ", "esph", "esphX"},
        {"sph tanint ", "esph", "esphY"}, {"sph inscr  ", "ebox", "esphZ"},
        {"cyl tanline", "ecyl", "ecylP"}, {"cyl flush  ", "ebox", "ecylF"},
        {"tor linked ", "etor", "ecylT"}, {"face copl3d", "ebox", "eboxC3"},
    };
}

// OCCT test-suite ports (tests/boolean/*_simple + bugs/modalg_*): complex primitive
// configurations OCCT itself uses as regression cases -- tangent contacts, seam-crossing
// periodic intersections, grazing rotated operands, multi-solid results, and (via the
// chain cells below) boolean-of-boolean stress. SESSION_OCCT2=1 runs this grid.
std::map<std::string, Place> occt2_placements() {
    return {
        // bopfuse_simple/ZF6: cone base plane coincident with box top face
        {"oCONE1", {"cone",     {1,2},       {0,0,2, 0,0,1, 0}}},
        // bopfuse_simple/ZL2: torus wraps a coaxial cylinder (R == r_cyl), sections at seams
        {"oCYL48", {"cylinder", {4,8},       {0,0,-4, 0,0,1, 0}}},
        {"oTOR41", {"torus",    {4,1},       ID}},
        // bugs/modalg_7/bug32502 (/4): off-axis cylinder pierces sphere across its seam
        {"oCYLW",  {"cylinder", {0.75,10},   {0,-5,1.75, 1,0,0, -90}}},
        // bugs/modalg_7/bug32470 (/4): box rotated -45 about y through (0,0,2.5) grazes sphere
        {"oBOXG",  {"box",      {5,25,25},   {3.5355339059327378,0,2.5, 0,1,0, -45}}},
        // bugs/modalg_7/bug27274 (/20): equal cylinders crossed at 45 deg, bases at origin
        {"oCYLA",  {"cylinder", {2.5,7.25},  ID}},
        {"oCYLB",  {"cylinder", {2.5,7.25},  {0,0,0, 1,0,0, 45}}},
        // bugs/modalg_7/bug29910_1 (/50): identical tori offset by the major radius
        {"oTORT1", {"torus",    {2,0.2},     ID}},
        {"oTORT2", {"torus",    {2,0.2},     {2,0,0, 0,0,1, 0}}},
        // bopfuse_simple/ZP6 (/50): three mutually orthogonal tori (chain cell)
        {"oTORX",  {"torus",    {2,0.4},     ID}},
        {"oTORY",  {"torus",    {2,0.4},     {0,0,0, 1,0,0, 90}}},
        {"oTORZ",  {"torus",    {2,0.4},     {0,0,0, 0,1,0, 90}}},
        // bugs/modalg_7/bug31835_1: Steinmetz tricylinder via chained common (chain cell)
        {"oTCZ",   {"cylinder", {1.5,8},     {0,0,-4, 0,0,1, 0}}},
        {"oTCX",   {"cylinder", {1.5,8},     {-4,0,0, 0,1,0, 90}}},
        {"oTCY",   {"cylinder", {1.5,8},     {0,4,0, 1,0,0, 90}}},
        // bugs/modalg_5/bug23876 (/10): washer (cyl-cyl cut) vs torus at its top rim (chain)
        {"oCYLO",  {"cylinder", {2,5},       ID}},
        {"oCYLI",  {"cylinder", {1,5.4},     {0,0,-0.2, 0,0,1, 0}}},
        {"oTORA",  {"torus",    {1.5,0.5},   {0,0,5, 0,0,1, 0}}},
        // bugs/modalg_5/bug24359 (2x2x2 corner of the 27-sphere grid): unit-spaced r=2
        // spheres, heavily overlapping -- progressive fuse (chain cell)
        {"oS0",    {"sphere",   {2},         ID}},
        {"oS1",    {"sphere",   {2},         {1,0,0, 0,0,1, 0}}},
        {"oS2",    {"sphere",   {2},         {0,1,0, 0,0,1, 0}}},
        {"oS3",    {"sphere",   {2},         {1,1,0, 0,0,1, 0}}},
        {"oS4",    {"sphere",   {2},         {0,0,1, 0,0,1, 0}}},
        {"oS5",    {"sphere",   {2},         {1,0,1, 0,0,1, 0}}},
        {"oS6",    {"sphere",   {2},         {0,1,1, 0,0,1, 0}}},
        {"oS7",    {"sphere",   {2},         {1,1,1, 0,0,1, 0}}},
    };
}
std::vector<std::array<std::string,3>> occt2_pairs() {
    return {
        {"o box coneT ", "box",    "oCONE1"},
        {"o cyl4 tor41", "oCYL48", "oTOR41"},
        {"o sph cylW  ", "sph",    "oCYLW"},
        {"o sph boxG  ", "sph",    "oBOXG"},
        {"o cylX45    ", "oCYLA",  "oCYLB"},
        {"o torTwin   ", "oTORT1", "oTORT2"},
    };
}
// (label, shape keys, modes between them) -- left-fold: r = k0; r = m_i(r, k_{i+1})
struct ChainCell { std::string label; std::vector<std::string> keys; std::vector<std::string> modes; };
std::vector<ChainCell> occt2_chains() {
    return {
        {"o tor3x fuse ", {"oTORX","oTORY","oTORZ"}, {"fuse","fuse"}},
        {"o tricyl com ", {"oTCZ","oTCX","oTCY"},    {"common","common"}},
        {"o annulus    ", {"oCYLO","oCYLI","oTORA"}, {"cut","common"}},
        {"o sph8 fuse  ", {"oS0","oS1","oS2","oS3","oS4","oS5","oS6","oS7"},
                          {"fuse","fuse","fuse","fuse","fuse","fuse","fuse"}},
    };
}

Xform xf_of(const std::array<double,7>& x) {
    Xform t = Xform::translation(x[0], x[1], x[2]);
    if (x[6] == 0.0) return t;
    Xform r = x[3] ? Xform::rotation_x(x[6], true)
            : x[4] ? Xform::rotation_y(x[6], true)
                   : Xform::rotation_z(x[6], true);
    return t * r;
}

BRep build(const Place& pl) {
    BRep b = pl.kind == "box"      ? BRep::create_box(pl.p[0], pl.p[1], pl.p[2])
           : pl.kind == "sphere"   ? BRep::create_sphere(pl.p[0])
           : pl.kind == "cylinder" ? BRep::create_cylinder(pl.p[0], pl.p[1])
           : pl.kind == "cone"     ? BRep::create_cone(pl.p[0], pl.p[1])
                                    : BRep::create_torus(pl.p[0], pl.p[1]);
    b.transform(xf_of(pl.xf));
    return b;
}

// RIGID-MOTION EQUIVARIANCE SWEEP (SESSION_POSE_SWEEP). The decisive experiment for P1:
// rotate BOTH operands by the SAME rigid motion and re-run. A rigid motion cannot change
// volumes or face counts, so ANY deviation from the angle-0 answer is pure pose dependence
// -- no oracle needed. A CLIFF at some angle means a recogniser predicate with an angular
// tolerance; a smooth RAMP means accumulated numerical error. One run distinguishes them.
void pose_sweep(const Place& pa, const Place& pb, const char* label) {
    const double angles[] = {0.0, 1e-6, 1e-4, 1e-2, 0.1, 1.0, 5.0, 15.0, 30.0, 45.0};
    const char* opn3[3] = {"cut", "common", "fuse"};
    double base[3] = {0, 0, 0};
    int basef[3] = {0, 0, 0};
    std::printf("\n[POSE] %s : rigid-motion equivariance (both operands rotated together)\n", label);
    std::printf("[POSE] %-10s %-8s %12s %12s %10s %6s\n", "angle_deg", "op", "vol", "d_vol", "rel", "faces");
    for (double ang : angles) {
        // arbitrary non-axis-aligned rotation axis so the motion is generic
        Xform R = Xform::rotation_around_line(Line(0, 0, 0, 0.4082, 0.8165, 0.4082), ang, true);
        for (int m = 0; m < 3; ++m) {
            BRep A2 = build(pa), B2 = build(pb);
            A2 = A2.transformed(R);
            B2 = B2.transformed(R);
            double v = 0; int nf = 0;
            try {
                BRep r = m == 0 ? A2.boolean_difference(B2)
                       : m == 1 ? A2.boolean_intersection(B2)
                                : A2.boolean_union(B2);
                v = r.volume(); nf = r.face_count();
            } catch (const std::exception& e) {
                std::printf("[POSE] %-10.6f %-8s THREW %s\n", ang, opn3[m], e.what());
                continue;
            }
            if (ang == 0.0) { base[m] = v; basef[m] = nf; }
            double dv = v - base[m];
            double rel = std::abs(base[m]) > 1e-12 ? std::abs(dv) / std::abs(base[m]) : std::abs(dv);
            std::printf("[POSE] %-10.6f %-8s %12.6f %12.3e %10.2e %6d%s\n",
                        ang, opn3[m], v, dv, rel, nf,
                        nf != basef[m] ? "  FACES-DIFFER" : (rel > 1e-9 ? "  <== BREAK" : ""));
        }
        std::fflush(stdout);
    }
}

// RELATIVE-POSE SWEEP (SESSION_RELPOSE_SWEEP). Rotating only B changes the answer, so
// volumes cannot be compared across angles. The oracle-free invariant that IS valid at every
// relative pose is the partition identity vol(cut) + vol(common) == vol(A). Sweeping the
// relative angle and watching that residual finds the angles where an analytic branch is
// missing (the marcher takes over and leaks volume) -- the boundaries of the recogniser
// coverage, measured rather than read out of the code.
void relpose_sweep(const Place& pa, const Place& pb, const char* label) {
    std::printf("\n[RELPOSE] %s : partition identity vs relative angle\n", label);
    std::printf("[RELPOSE] %-9s %10s %10s %10s %10s %5s %5s\n",
                "angle", "cut", "common", "sum", "rel", "fc", "fo");
    BRep A0 = build(pa);
    double vA = A0.volume();
    for (double ang : {0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0}) {
        Xform R = Xform::rotation_around_line(Line(0, 0, 0, 0.4082, 0.8165, 0.4082), ang, true);
        BRep A2 = build(pa);
        BRep B2 = build(pb); B2 = B2.transformed(R);
        double vc = 0, vo = 0; int fc = 0, fo = 0; bool ok = true;
        try {
            BRep rc = A2.boolean_difference(B2);  vc = rc.volume(); fc = rc.face_count();
            BRep ro = A2.boolean_intersection(B2); vo = ro.volume(); fo = ro.face_count();
        } catch (const std::exception& e) {
            std::printf("[RELPOSE] %-9.2f THREW %s\n", ang, e.what()); ok = false;
        }
        if (!ok) continue;
        double sum = vc + vo, rel = std::abs(sum - vA) / std::max(1.0, std::abs(vA));
        std::printf("[RELPOSE] %-9.2f %10.5f %10.5f %10.5f %10.2e %5d %5d%s\n",
                    ang, vc, vo, sum, rel, fc, fo, rel > 1e-6 ? "  ** VIOLATION **" : "");
        std::fflush(stdout);
    }
}

std::string shape_str(const Place& pl) {
    std::ostringstream s;
    s << "SHAPE " << pl.kind;
    for (double v : pl.p) s << " " << v;
    s << " XF"; for (double v : pl.xf) s << " " << v;
    return s.str();
}

std::map<std::string, Ref> load_cache(const std::string& path) {
    std::map<std::string, Ref> m;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto t1 = line.find('\t'); if (t1 == std::string::npos) continue;
        auto t2 = line.find('\t', t1 + 1); if (t2 == std::string::npos) continue;
        Ref r; r.vol = std::strtod(line.substr(t1 + 1, t2 - t1 - 1).c_str(), nullptr);
        r.nf = std::atoi(line.substr(t2 + 1).c_str());
        m[line.substr(0, t1)] = r;
    }
    return m;
}

void save_cache(const std::string& path, const std::map<std::string, Ref>& m) {
    std::ofstream f(path);
    f << "# OCCT boolean reference cache: key<TAB>volume<TAB>nfaces. Pass --refresh (or delete) to regenerate.\n";
    for (auto& kv : m)
        f << kv.first << "\t" << std::setprecision(17) << kv.second.vol << "\t" << kv.second.nf << "\n";
}

// returns {volume, nfaces, ok}. Uses the cache unless refresh; on a miss, shells the oracle and
// records the full-precision result so the next run is fast.
std::array<double,3> occt(const std::string& key, const std::string& mode, const Place& a, const Place& b,
                          bool have_oracle, const std::string& oracle, const std::string& req, const std::string& res,
                          std::map<std::string, Ref>& cache, bool refresh, bool& dirty) {
    if (!refresh) { auto it = cache.find(key); if (it != cache.end()) return {it->second.vol, (double)it->second.nf, 1.0}; }
    if (!have_oracle) return {0, 0, 0};
    { std::ofstream f(req); f << "OP boolean\nMODE " << mode << "\n" << shape_str(a) << "\n" << shape_str(b) << "\n"; }
    std::string cmd = oracle + " " + req + " " + res;
    if (std::system(cmd.c_str()) != 0) return {0, 0, 0};
    std::ifstream f(res); std::string tok; double vol = 0; int nf = 0; bool okv = false, okf = false;
    while (f >> tok) { if (tok == "VOLUME") { f >> vol; okv = true; } else if (tok == "NFACES") { f >> nf; okf = true; } }
    if (okv && okf) { cache[key] = {vol, nf}; dirty = true; return {vol, (double)nf, 1.0}; }
    return {vol, (double)nf, 0.0};
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // unbuffered: stream each row, survive a hang
    std::string filter; bool refresh = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--refresh") refresh = true;
        else if (filter.empty()) filter = a;   // substring pair-label filter (skip other/hanging cells)
    }
    auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::string oracle = (root / "validation" / "occt_oracle" / "build" / "Release" / "oracle.exe").string();
    std::string req = (root / "validation" / "_m7_req.txt").string();
    std::string res = (root / "validation" / "_m7_out.txt").string();
    std::string cachePath = (root / "validation" / "occt_cache.txt").string();
    bool have_oracle = std::filesystem::exists(oracle);
    auto cache = load_cache(cachePath);
    bool dirty = false;
    if (!have_oracle && cache.empty())
        std::fprintf(stderr, "(no oracle.exe and empty cache -> printing our numbers only)\n");
    if (refresh) std::fprintf(stderr, "(--refresh: re-shelling oracle for every cell)\n");

    auto PL = placements();
    if (std::getenv("SESSION_MPCHECK")) {
        // INTEGRATOR A/B on the five primitives, where the answer is analytic. Columns:
        // BRep::volume()/legacy bbox-Gauss, brep_massprops' Green-reduced path, and the analytic
        // truth. Also area, because the known trap (a planar face bounded by ONE closed trim
        // integrated over the whole parameter rectangle) shows up in area first.
        const double PI = Tolerance::PI;
        struct C { const char* nm; BRep b; double vol, area; };
        std::vector<C> cs;
        cs.push_back({"box 4x4x4", BRep::create_box(4,4,4), 64.0, 96.0});
        cs.push_back({"sphere 2.5", BRep::create_sphere(2.5), 4.0/3.0*PI*15.625, 4*PI*6.25});
        cs.push_back({"cylinder 1.5x6", BRep::create_cylinder(1.5,6), PI*2.25*6, 2*PI*2.25 + 2*PI*1.5*6});
        cs.push_back({"cone r2 h5", BRep::create_cone(2.0,5.0), PI*4.0*5.0/3.0,
                      PI*4.0 + PI*2.0*std::sqrt(4.0+25.0)});
        cs.push_back({"torus 2/0.8", BRep::create_torus(2.0,0.8), 2*PI*PI*2.0*0.64,
                      4*PI*PI*2.0*0.8});
        std::printf("%-16s %18s %18s %18s | %18s %18s %18s\n",
                    "primitive", "vol_legacy", "vol_massprops", "vol_analytic",
                    "area_massprops", "area_analytic", "rel_area");
        {   // Boolean-produced faces, where the planar caps are bounded by ONE CLOSED trim --
            // the configuration the "whole parameter rectangle instead of the disk" trap needs.
            BRep box2 = BRep::create_box(2,2,2);
            BRep cyl2 = BRep::create_cylinder(0.7, 3.0);
            cyl2 = cyl2.transformed(Xform::translation(0,0,-1.5));
            cs.push_back({"box2 ^ cyl0.7", box2.boolean_intersection(cyl2), PI*0.49*2.0, -1.0});
            BRep box4 = BRep::create_box(4,4,4);
            BRep sph15 = BRep::create_sphere(1.5);
            cs.push_back({"box4 ^ sph1.5", box4.boolean_intersection(sph15),
                          4.0/3.0*PI*3.375, 4*PI*2.25});
        }
        for (auto& c : cs) {
            MassProps m = brep_massprops(c.b);
            std::printf("%-16s %18.9f %18.9f %18.9f | %18.9f %18.9f %18.3e\n",
                        c.nm, c.b.volume(), m.volume, c.vol, m.area, c.area,
                        c.area > 0 ? std::abs(m.area - c.area) / c.area : 0.0);
            std::printf("    shells=%d closed=%d converged=%d closure=%.3e naked=%d vols=[",
                        m.shell_count, m.closed?1:0, m.converged?1:0, m.closure_residual, m.naked_edges);
            for (double sv2 : m.shell_volumes) std::printf("%.9f ", sv2);
            std::printf("]  sum(outward*flux)/3=%.9f\n",
                        [&]{ double a=0; for (const auto& f2 : m.faces) a += f2.outward*f2.flux; return a/3.0; }());
            for (const auto& fm : m.faces)
                std::printf("    face %d path=%d shell=%d area=%.9f flux=%.9f outward=%+.0f traversal=%+.0f chain_gap=%.2e\n",
                            fm.face_index, (int)fm.path, fm.shell, fm.area, fm.flux, fm.outward,
                            fm.traversal, fm.chain_gap);
        }
        return 0;
    }
    if (std::getenv("SESSION_TRIMKIND")) {
        // What does an emitted split face's BRepTrim actually CARRY as its 2D curve?
        // An exact conic pcurve is degree>=2 and rational; a sampled polyline is degree 1.
        // OCCT stores the exact Geom2d curve here, so a face bounded by a circular arc IS
        // bounded by an arc; if ours stores a polygon, the face area is the inscribed polygon's.
        auto dump = [](const BRep& X, const char* nm) {
            int exact = 0, poly = 0, polycv = 0;
            std::printf("[TRIMKIND] %s: faces=%d\n", nm, X.face_count());
            for (int fi = 0; fi < (int)X.m_faces.size(); ++fi) {
                const auto& F = X.m_faces[fi];
                if (F.surface_index < 0 || F.surface_index >= (int)X.m_surfaces.size()) continue;
                bool planar = X.m_surfaces[F.surface_index].is_planar(nullptr, 1e-6);
                for (int li : F.loop_indices) {
                    if (li < 0 || li >= (int)X.m_loops.size()) continue;
                    for (int ti : X.m_loops[li].trim_indices) {
                        if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                        int c2 = X.m_trims[ti].curve_2d_index;
                        if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                        const NurbsCurve& pc = X.m_curves_2d[c2];
                        bool ex = pc.degree() >= 2 && pc.is_rational();
                        if (ex) ++exact; else { ++poly; polycv += pc.cv_count(); }
                        std::printf("   f%-2d %-6s deg=%d rat=%d cvs=%-5d %s\n",
                                    fi, planar ? "PLANE" : "curved", pc.degree(),
                                    pc.is_rational() ? 1 : 0, pc.cv_count(),
                                    ex ? "EXACT-CONIC" : "POLYLINE");
                    }
                }
            }
            std::printf("[TRIMKIND] %s: exact=%d polyline=%d (avg %d cvs)\n",
                        nm, exact, poly, poly ? polycv/poly : 0);
        };
        BRep box = BRep::create_box(4, 4, 4);
        BRep sph = BRep::create_sphere(2.5);
        dump(box.boolean_intersection(sph), "box x sph common");
        BRep cyl = BRep::create_cylinder(1.5, 6);
        dump(box.boolean_intersection(cyl), "box x cyl common (matrix: EXACT 3.8e-16)");
        return 0;
    }
    if (std::getenv("SESSION_SPLITCOUNT")) {
        // The minitest "BRep::Boolean Sphere Split" asserts 8 here and its own comment records
        // why: "OCCT keeps the cap as a single seam-spanning face = 7; joining the halves needs
        // full seam identification". This probe prints the number so the seam merge's effect on
        // that documented defect is measurable without editing the test.
        BRep sph = BRep::create_sphere(2.5);
        BRep box = BRep::create_box(4, 4, 4);
        // PRIMITIVE TOPOLOGY vs the OCCT tracer's input dumps (kb/occt_trace_findings.md Q4):
        // BRepPrimAPI_MakeSphere(2.5) = 1 face / 3 edges / 2 vertices (2 degenerated poles over
        // the full u period + the seam meridian); BRepPrimAPI_MakeCone = 2 faces / 3 edges /
        // 2 vertices. "wire" counts how many trims reference each edge -- a pole must appear
        // ONCE, a seam TWICE.
        {
            BRep cone = BRep::create_cone(2.0, 5.0);
            auto dump = [](const char* nm, const BRep& X, int occt_f, int occt_e, int occt_v) {
                int degn = 0;
                std::vector<int> uses(X.m_topology_edges.size(), 0);
                for (const auto& t : X.m_trims)
                    if (t.edge_index >= 0 && t.edge_index < (int)uses.size()) ++uses[t.edge_index];
                std::string useq;
                for (size_t e = 0; e < X.m_topology_edges.size(); ++e) {
                    int ci = X.m_topology_edges[e].curve_3d_index;
                    bool dg = false;
                    if (ci >= 0 && ci < (int)X.m_curves_3d.size()) {
                        const NurbsCurve& c = X.m_curves_3d[ci];
                        auto dc = c.domain(); Point p0 = c.point_at(dc.first); double ext = 0.0;
                        for (int k = 1; k <= 4; ++k)
                            ext = std::max(ext, p0.distance(c.point_at(dc.first + (dc.second-dc.first)*k/4.0)));
                        dg = ext < 1e-9;
                    }
                    if (dg) ++degn;
                    useq += (dg ? "D" : "e") + std::to_string(uses[e]) + " ";
                }
                int nsing = 0;
                for (const auto& t : X.m_trims) if (t.type == BRepTrimType::Singular) ++nsing;
                std::printf("[PRIMTOP] %-6s faces=%d edges=%zu verts=%zu degen=%d singular_trims=%d "
                            "wire_uses=[%s] solid=%d vol=%.6f  (OCCT %d/%d/%d)\n",
                            nm, X.face_count(), X.m_topology_edges.size(), X.m_topology_vertices.size(),
                            degn, nsing, useq.c_str(), X.is_solid() ? 1 : 0, X.volume(),
                            occt_f, occt_e, occt_v);
            };
            dump("sphere", sph, 1, 3, 2);
            dump("cone", cone, 2, 3, 2);
        }
        BRep B2 = sph.split_by_brep(box);
        BRep bcut = box.boolean_difference(sph), bcom = box.boolean_intersection(sph);
        int nk1, nm1, nk2, nm2;
        count_naked_nonmani(bcut, nk1, nm1);
        count_naked_nonmani(bcom, nk2, nm2);
        std::printf("[SPLITCOUNT] sphere.split_by_brep(box) faces=%d (OCCT 7)\n", B2.face_count());
        std::printf("[SPLITCOUNT] box-sph cut  faces=%d nk=%d nm=%d solid=%d vol=%.9f (occt 9.545724581)\n",
                    bcut.face_count(), nk1, nm1, bcut.is_solid() ? 1 : 0, bcut.volume());
        std::printf("[SPLITCOUNT] box-sph com  faces=%d nk=%d nm=%d solid=%d vol=%.9f (occt 54.454275630)\n",
                    bcom.face_count(), nk2, nm2, bcom.is_solid() ? 1 : 0, bcom.volume());
        // THE WRAP CASE (kb Q3 case A): OCCT's sphere-intersect-box keeps the central band as ONE
        // sphere face spanning u0=0..2pi with the seam-straddling +X hole opened into its outer
        // wire -- 1 spherical + 6 planar = 7. Count curved vs planar to prove we do the same
        // rather than reaching 7 some other way.
        auto split_kind = [](const BRep& X, const char* nm) {
            int planar = 0, curved = 0;
            for (const auto& f : X.m_faces) {
                if (f.surface_index < 0 || f.surface_index >= (int)X.m_surfaces.size()) continue;
                if (X.m_surfaces[f.surface_index].is_planar(nullptr, 1e-6)) ++planar; else ++curved;
            }
            std::printf("[SPLITCOUNT] %s curved=%d planar=%d\n", nm, curved, planar);
        };
        split_kind(bcom, "box-sph com  (OCCT curved=1 planar=6)");
        split_kind(bcut, "box-sph cut  (OCCT curved=7 planar=6)");
        return 0;
    }
    if (std::getenv("SESSION_SPHCYL")) {
        // MINIMAL CURVED REPRODUCER. Sphere x cylinder, cylinder axis through the sphere
        // centre, tilted. One section circle, exact analytic inputs, no coincidence, no
        // trimming complexity. Sweep the tilt finely and report BOTH ops so the exact angle
        // where behaviour changes is visible. vol(A) is analytic, so the partition identity
        // cut+common == vol(A) is an exact oracle-free accuracy test at every angle.
        double sr = 2.5, cr = 1.0, ch = 8.0;
        if (const char* e = std::getenv("SESSION_SPHCYL_R")) cr = std::atof(e);
        BRep S0 = BRep::create_sphere(sr);
        double volA = S0.volume();
        // analytic truth: V(sphere ∩ infinite cylinder r through centre) -- spherical cylinder
        double a = std::sqrt(std::max(0.0, sr*sr - cr*cr));
        double v_int = 4.0*Tolerance::PI/3.0*(sr*sr*sr - std::pow(sr*sr - cr*cr, 1.5));
        std::printf("[SPHCYL] sphere r=%.3f (vol %.6f)  cylinder r=%.3f h=%.1f through centre\n",
                    sr, volA, cr, ch);
        std::printf("[SPHCYL] analytic: common=%.6f  cut=%.6f  (half-height of cap a=%.6f)\n",
                    v_int, volA - v_int, a);
        std::printf("[SPHCYL] %8s | %s | %s | %12s %12s %12s | %s\n",
                    "tilt", "cut f/nk/nm", "com f/nk/nm", "vol_cut", "vol_com", "sum", "verdict");
        const char* angs = std::getenv("SESSION_SPHCYL_ANGLES");
        std::vector<double> tilts;
        if (angs) { std::string s(angs); size_t p = 0; while (p < s.size()) { size_t q = s.find(',', p);
                    tilts.push_back(std::atof(s.substr(p, q==std::string::npos?q:q-p).c_str()));
                    if (q == std::string::npos) break; p = q+1; } }
        else tilts = {0.0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 5.0, 15.0, 30.0, 45.0};
        for (double t : tilts) {
            BRep S = BRep::create_sphere(sr);
            // SESSION_SPHCYL_SPIN=<deg>: rotate the SPHERE about its own polar axis. This is a
            // GEOMETRIC NO-OP for the answer (the sphere is rotationally symmetric about Z and
            // the cylinder axis passes through the centre), but it MOVES THE SEAM. If the
            // result depends on it, the seam is proven to be the cause.
            if (const char* spn = std::getenv("SESSION_SPHCYL_SPIN")) {
                if (spn[0]) {
                    S = S.transformed(Xform::rotation_around_line(Line(0,0,0, 0,0,1), std::atof(spn), true));
                }
            }
            BRep C = BRep::create_cylinder(cr, ch);
            C = C.transformed(Xform::translation(0, 0, -ch*0.5));
            BRep C2 = C;
            // SESSION_SPHCYL_AXIS=110 rotates about (1,1,0) instead of Y. Same pair, same
            // angle, different axis: Y works at every tilt, (1,1,0) fails at 20 deg.
            const char* axe = std::getenv("SESSION_SPHCYL_AXIS");
            bool a110 = (axe && axe[0] && std::string(axe) == "110");
            C2 = C2.transformed(a110
                ? Xform::rotation_around_line(Line(0,0,0, 0.70710678,0.70710678,0), t, true)
                : Xform::rotation_around_line(Line(0,0,0, 0,1,0), t, true));
            // SESSION_SPHCYL_SSI: raw section probe. Prints, per surface pair, the pcurves the
            // intersector hands the splitter (count + UV bbox + 3D length). Tells apart "the
            // section was never produced" from "the arrangement lost it".
            if (std::getenv("SESSION_SPHCYL_SSI")) {
                auto probe = [&](const char* who, const BRep& T, const BRep& Cx) {
                    for (size_t ti = 0; ti < T.m_surfaces.size(); ++ti)
                        for (size_t ci2 = 0; ci2 < Cx.m_surfaces.size(); ++ci2) {
                            auto pcs = Intersection::cut_curves_on_surface(
                                T.m_surfaces[ti], Cx.m_surfaces[ci2], 1e-6);
                            if (pcs.empty()) continue;
                            std::printf("[SSI] tilt=%.3f %s tsi=%zu csi=%zu n=%zu\n",
                                        t, who, ti, ci2, pcs.size());
                            for (size_t k = 0; k < pcs.size(); ++k) {
                                auto dq = pcs[k].domain();
                                double xmn=1e300,xmx=-1e300,ymn=1e300,ymx=-1e300,len3=0.0;
                                Point prev(0,0,0);
                                for (int q2 = 0; q2 <= 64; ++q2) {
                                    Point uvq = pcs[k].point_at(dq.first + (dq.second-dq.first)*q2/64.0);
                                    xmn=std::min(xmn,uvq[0]); xmx=std::max(xmx,uvq[0]);
                                    ymn=std::min(ymn,uvq[1]); ymx=std::max(ymx,uvq[1]);
                                    Point p3q = T.m_surfaces[ti].point_at(uvq[0], uvq[1]);
                                    if (q2) len3 += prev.distance(p3q);
                                    prev = p3q;
                                }
                                Point qa = pcs[k].point_at(dq.first), qb = pcs[k].point_at(dq.second);
                                std::printf("[SSI]   %zu a(%.4f,%.4f) b(%.4f,%.4f) bbox[%.3f,%.3f]x[%.3f,%.3f] len3=%.4f closed=%d\n",
                                            k, qa[0],qa[1], qb[0],qb[1], xmn,xmx,ymn,ymx, len3,
                                            qa.distance(qb) < 1e-9 ? 1 : 0);
                            }
                        }
                };
                probe("S", S, C2);
                probe("C", C2, S);
                std::fflush(stdout);
                continue;
            }
            // naked_real / nonmanifold EXCLUDE zero-length pole edges (count_naked_nonmani,
            // validated against is_solid on this very pole shape). Volume alone hides the
            // defect (35-44 deg: exact volume, open cut), so closure_residual is scored too.
            int nkc=0,nmc=0,nko=0,nmo=0,nfc=0,nfo=0; double vc=std::nan(""), vo=std::nan("");
            double clc = std::nan(""), clo = std::nan("");
            try {
                BRep rc = S.boolean_difference(C2);
                count_naked_nonmani(rc, nkc, nmc);
                nfc = rc.face_count(); vc = rc.volume();
                clc = brep_massprops(rc).closure_residual;
                BRep ro = S.boolean_intersection(C2);
                count_naked_nonmani(ro, nko, nmo);
                nfo = ro.face_count(); vo = ro.volume();
                clo = brep_massprops(ro).closure_residual;
                if (std::getenv("SESSION_SPHCYL_DBG")) {
                    // deg = zero-length edge RECORDS, directly comparable to OCCT's res_degen.
                    // OCCT carries a DEGENERATED edge at every pole in BOTH input and result;
                    // our create_sphere gives the pole trims edge_index -1 (no edge record at
                    // all), so input and result use different conventions -- the mismatch that
                    // made a raw nt==1 count misreport these results as open.
                    auto degcount = [](const BRep& X) {
                        double mn[3] = {1e300,1e300,1e300}, mx[3] = {-1e300,-1e300,-1e300};
                        for (const auto& p : X.m_vertices)
                            for (int k = 0; k < 3; ++k) { mn[k]=std::min(mn[k],p[k]); mx[k]=std::max(mx[k],p[k]); }
                        double diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0])+(mx[1]-mn[1])*(mx[1]-mn[1])+(mx[2]-mn[2])*(mx[2]-mn[2]));
                        double tol = std::max(diag * 1e-7, 1e-12);
                        int n = 0;
                        for (const auto& e : X.m_topology_edges) {
                            int ci = e.curve_3d_index;
                            if (ci < 0 || ci >= (int)X.m_curves_3d.size()) continue;
                            const NurbsCurve& c = X.m_curves_3d[ci];
                            auto dc = c.domain(); Point p0 = c.point_at(dc.first); double ext = 0.0;
                            for (int k = 1; k <= 4; ++k)
                                ext = std::max(ext, p0.distance(c.point_at(dc.first + (dc.second-dc.first)*k/4.0)));
                            if (ext < tol) ++n;
                        }
                        return n;
                    };
                    std::printf("[SPHCYL-DBG] tilt %.3f cut: solid=%d f=%d nk=%d nm=%d deg=%d cl=%.2e ar=%.6f | "
                                "com: solid=%d f=%d nk=%d nm=%d deg=%d cl=%.2e ar=%.6f\n",
                                t, rc.is_solid()?1:0, nfc, nkc, nmc, degcount(rc), clc,
                                brep_massprops(rc).area,
                                ro.is_solid()?1:0, nfo, nko, nmo, degcount(ro), clo,
                                brep_massprops(ro).area);
                }
                if (std::getenv("SESSION_SPHCYL_MESH")) {
                    // INDEPENDENT THIRD MEASUREMENT to arbitrate between the two integrators:
                    // tessellate and sum signed tetrahedra, (1/6) (a x b) . c. Uses no trim
                    // quadrature at all, so it shares no failure mode with either.
                    auto mesh_vol = [](const BRep& X) {
                        Mesh m = X.mesh();
                        auto [vs, fs] = m.to_vertices_and_faces();
                        double v = 0.0;
                        for (const auto& f : fs) {
                            if (f.size() < 3) continue;
                            for (size_t j = 1; j + 1 < f.size(); ++j) {
                                const Point& A = vs[f[0]]; const Point& B = vs[f[j]]; const Point& C = vs[f[j+1]];
                                v += (A[0]*(B[1]*C[2]-B[2]*C[1])
                                    - A[1]*(B[0]*C[2]-B[2]*C[0])
                                    + A[2]*(B[0]*C[1]-B[1]*C[0])) / 6.0;
                            }
                        }
                        return std::abs(v);
                    };
                    std::printf("[MESHVOL] tilt %.6f cut mesh=%.6f vol=%.6f | com mesh=%.6f vol=%.6f\n",
                                t, mesh_vol(rc), vc, mesh_vol(ro), vo);
                }
                if (std::getenv("SESSION_SPHCYL_ORIENT")) {
                    // Per-face orientation/flux of the CUT. At the exact pole tangency the cut's
                    // AREA is right (100.7746 vs OCCT 100.776103, i.e. the correct two faces) but
                    // its VOLUME comes back as the complement, so the suspect is the per-face
                    // `outward` sense, not the face selection. OCCT's reference for this case is
                    // RESFACE i=1 Sphere ori=FWD area=71.9829307 and i=2 Cylinder ori=REV.
                    MassProps mpc = brep_massprops(rc);
                    std::printf("[ORIENT] tilt %.6f cut  BRep::volume=%.9f  brep_massprops=%.9f  (truth 50.388051)\n",
                                t, rc.volume(), mpc.volume);
                    for (const auto& fm : mpc.faces)
                        std::printf("[ORIENT] tilt %.6f cut f=%d area=%.6f flux=%.6f traversal=%+.0f outward=%+.0f path=%d\n",
                                    t, fm.face_index, fm.area, fm.flux, fm.traversal, fm.outward, (int)fm.path);
                    // BRep::volume()'s own per-face sign probe (a CDT-interior point stepped along
                    // the natural normal). This is the suspect: at the tangency the sphere band's
                    // UV polygon is PINCHED at the pole, and a pinched polygon breaks the
                    // parity/CDT reasoning the probe relies on.
                    std::vector<Point> P3; std::vector<Vector> Nn;
                    std::vector<double> sg = rc.face_outward_signs(&P3, &Nn);
                    for (size_t k = 0; k < sg.size(); ++k)
                        std::printf("[ORIENT] tilt %.6f cut f=%zu sign=%+.0f probe=(%.5f,%.5f,%.5f) nat=(%.4f,%.4f,%.4f)\n",
                                    t, k, sg[k],
                                    k < P3.size() ? P3[k][0] : 0.0, k < P3.size() ? P3[k][1] : 0.0,
                                    k < P3.size() ? P3[k][2] : 0.0,
                                    k < Nn.size() ? Nn[k][0] : 0.0, k < Nn.size() ? Nn[k][1] : 0.0,
                                    k < Nn.size() ? Nn[k][2] : 0.0);
                }
            } catch (const std::exception& e) {
                std::printf("[SPHCYL] %8.3f THREW %s\n", t, e.what()); continue;
            }
            double sum = vc + vo;
            double tolv = 1e-6;   // relative
            if (const char* tv = std::getenv("SESSION_SPHCYL_TOL")) tolv = std::atof(tv);
            bool ok = !nkc && !nmc && !nko && !nmo &&
                      std::isfinite(vc) && std::isfinite(vo) &&
                      clc < 1e-9 && clo < 1e-9 &&
                      std::abs(vo - v_int)/v_int < tolv &&
                      std::abs(vc - (volA - v_int))/(volA - v_int) < tolv;
            std::printf("[SPHCYL] %8.3f | %2d/%d/%d      | %2d/%d/%d      | %12.6f %12.6f %12.6f | %s\n",
                        t, nfc,nkc,nmc, nfo,nko,nmo, vc, vo, sum, ok ? "PASS" : "FAIL");
            std::fflush(stdout);
        }
        return 0;
    }
    if (const char* insp = std::getenv("SESSION_INSPECT")) {
        // CURATED IN-MEMORY INSPECTION SET. Operands are built by create_* and rotated in
        // memory; STEP is OUTPUT ONLY, so nothing here is contaminated by the reader. One
        // file per cell: A red, rotated B blue, result green, one product per BRep.
        // Accuracy is oracle-free: both cut and common are computed and the partition
        // identity cut+common == vol(A) is checked (vol(A) is analytic for a primitive).
        std::string dir = insp;
        std::filesystem::create_directories(dir);
        std::printf("| file | pair | rotation | faces | solid | shells | naked | nonmani | our vol | cut+common | vol(A) | verdict |\n");
        std::printf("|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|\n");
        auto emit = [&](const std::string& fname, const std::string& pairname,
                        const std::string& rotdesc, BRep A2, BRep B2, int op) {
            const char* opn3[3] = {"cut", "common", "fuse"};
            double volA = A2.volume();
            BRep res, other;
            try {
                res   = op == 0 ? A2.boolean_difference(B2)
                      : op == 1 ? A2.boolean_intersection(B2) : A2.boolean_union(B2);
                other = op == 0 ? A2.boolean_intersection(B2) : A2.boolean_difference(B2);
            } catch (const std::exception& e) {
                std::printf("| %s | %s | %s | THREW %s |\n", fname.c_str(), pairname.c_str(),
                            rotdesc.c_str(), e.what());
                return;
            }
            // degeneracy-aware, matching is_solid(): a sphere pole / cone apex is a
            // zero-length edge and is watertight by construction, not a naked edge.
            int nk = 0, nm = 0, nk2 = 0, nm2 = 0;
            count_naked_nonmani(res, nk, nm);
            count_naked_nonmani(other, nk2, nm2);
            bool closed = (nk == 0 && nm == 0);
            double vres = closed ? res.volume() : std::nan("");
            double vsum = std::nan("");
            if (closed && nk2 == 0 && nm2 == 0) vsum = vres + other.volume();
            bool pass = closed && std::isfinite(vsum) &&
                        std::abs(vsum - volA) / std::max(1.0, volA) < 1e-6;
            BRep ac = A2, bc = B2, rc = res;
            ac.surfacecolor = Color(0.85f, 0.25f, 0.20f);
            bc.surfacecolor = Color(0.20f, 0.45f, 0.85f);
            rc.surfacecolor = Color(0.35f, 0.75f, 0.40f);
            std::vector<const BRep*> parts = {&ac, &bc};
            if (rc.face_count() > 0) parts.push_back(&rc);
            file_step::write_file_step_breps(parts, fname, dir + "/" + fname + ".step");
            std::printf("| %s.step | %s %s | %s | %d | %d | %d | %d | %d | %.4f | %.4f | %.4f | %s |\n",
                        fname.c_str(), pairname.c_str(), opn3[op], rotdesc.c_str(),
                        res.face_count(), res.is_solid() ? 1 : 0, shell_count_of(res), nk, nm,
                        vres, vsum, volA, pass ? "PASS" : (rc.face_count() == 0 ? "EMPTY" : "FAIL"));
            std::fflush(stdout);
        };
        auto rotY = [](double deg) { return Xform::rotation_around_line(Line(0,0,0, 0,1,0), deg, true); };
        auto rot110 = [](double deg) { return Xform::rotation_around_line(Line(0,0,0, 0.70710678,0.70710678,0), deg, true); };
        // (a) MINIMAL REPRODUCER: sphere x cylinder, cylinder axis through the sphere centre,
        //     tilted by a fraction of a degree. Exact analytic inputs; closure collapses.
        for (double t : {0.0, 0.1, 0.2, 0.3, 0.5}) {
            BRep S = BRep::create_sphere(2.5);
            BRep C = BRep::create_cylinder(1.0, 8.0);
            C = C.transformed(Xform::translation(0,0,-4.0));
            BRep C2 = C; C2 = C2.transformed(rotY(t));
            char nm[64]; std::snprintf(nm, sizeof nm, "A_sphcyl_tilt%.1f_cut", t);
            char rd[32]; std::snprintf(rd, sizeof rd, "%.1f deg about Y", t);
            emit(nm, "sph x cyl", rd, S, C2, 0);
        }
        // (b) THE 0.01-DEGREE FLIP about (1,1,0)
        for (double t : {20.00, 20.01, 20.02, 20.03}) {
            BRep S = BRep::create_sphere(2.5);
            BRep C = BRep::create_cylinder(1.0, 8.0);
            C = C.transformed(Xform::translation(0,0,-4.0));
            BRep C2 = C; C2 = C2.transformed(rot110(t));
            char nm[64]; std::snprintf(nm, sizeof nm, "B_sphcyl_110axis_%.2f_cut", t);
            char rd[32]; std::snprintf(rd, sizeof rd, "%.2f deg about (1,1,0)", t);
            emit(nm, "sph x cyl", rd, S, C2, 0);
        }
        // (c) THE WORKING CASE: box x box at clearly different arbitrary rotations
        {
            unsigned sd = 20260726u;
            auto rnd = [&]() { sd = sd*1664525u + 1013904223u; return (sd>>8)/16777216.0; };
            for (int k = 0; k < 4; ++k) {
                double ax=rnd()*2-1, ay=rnd()*2-1, az=rnd()*2-1;
                double m=std::sqrt(ax*ax+ay*ay+az*az); ax/=m; ay/=m; az/=m;
                double ang=rnd()*360.0, d=1.5+rnd()*1.5;
                BRep A2 = BRep::create_box(4,4,4);
                BRep B2 = BRep::create_box(4,4,4);
                B2 = B2.transformed(Xform::translation(d,d*0.3,d*0.6) * Xform::rotation_around_line(Line(0,0,0,ax,ay,az), ang, true));
                char nm[64]; std::snprintf(nm, sizeof nm, "C_boxbox_pose%d_cut", k);
                char rd[48]; std::snprintf(rd, sizeof rd, "%.0f deg arbitrary axis", ang);
                emit(nm, "box x box", rd, A2, B2, 0);
            }
        }
        // (d) FAILING CURVED FAMILIES, genuinely interfering poses
        struct FC { const char* nm; BRep (*ma)(); BRep (*mb)(); double d; };
        static const FC fams[] = {
            {"sphsph",  []{ return BRep::create_sphere(2.5); },      []{ return BRep::create_sphere(2.0); }, 2.0},
            {"boxsph",  []{ return BRep::create_box(4,4,4); },       []{ return BRep::create_sphere(2.5); }, 1.8},
            {"conecone",[]{ return BRep::create_cone(2.0,4.0); },    []{ return BRep::create_cone(2.0,4.0); }, 1.5},
            {"cylcyl",  []{ return BRep::create_cylinder(1.5,6); },  []{ return BRep::create_cylinder(1.5,6); }, 1.5},
            {"boxcone", []{ return BRep::create_box(4,4,4); },       []{ return BRep::create_cone(2.0,4.0); }, 1.2},
            {"cylcone", []{ return BRep::create_cylinder(1.5,6); },  []{ return BRep::create_cone(2.0,4.0); }, 1.2},
            {"boxtor",  []{ return BRep::create_box(4,4,4); },       []{ return BRep::create_torus(2.0,0.8); }, 1.0},
        };
        for (const auto& f : fams)
            for (int k = 0; k < 2; ++k) {
                double ang = (k == 0) ? 25.0 : 55.0;
                BRep A2 = f.ma(), B2 = f.mb();
                B2 = B2.transformed(Xform::translation(f.d, f.d*0.4, f.d*0.2)
                         * Xform::rotation_around_line(Line(0,0,0, 0.4082,0.8165,0.4082), ang, true));
                char nm[64]; std::snprintf(nm, sizeof nm, "D_%s_rot%.0f_cut", f.nm, ang);
                char rd[48]; std::snprintf(rd, sizeof rd, "%.0f deg tilted axis", ang);
                emit(nm, f.nm, rd, A2, B2, 0);
            }
        return 0;
    }
    if (const char* ps = std::getenv("SESSION_PRIMSWEEP")) {
        // DEFINITIVE IN-MEMORY ROTATED-PRIMITIVE SWEEP. No STEP anywhere in the loop, so
        // rotation is isolated from the loader/domain defect. One PAIR per process so a hang
        // kills only that pair and is counted, never silently dropped.
        // Verdict rule: naked == 0 AND non-manifold == 0 (with naked 0 every shell is closed
        // by construction). Accuracy is ORACLE-FREE: partition identity cut+common == vol(A),
        // exact here because in-memory operand volumes are analytic.
        struct K { const char* name; BRep (*make)(); double rad; };
        static const K kinds[] = {
            {"box", []{ return BRep::create_box(4, 4, 4); }, 3.47},
            {"sph", []{ return BRep::create_sphere(2.5); }, 2.5},
            {"cyl", []{ return BRep::create_cylinder(1.5, 6); }, 3.35},
            {"cone", []{ return BRep::create_cone(2.0, 4.0); }, 2.83},
            {"tor", []{ return BRep::create_torus(2.0, 0.8); }, 2.8},
        };
        std::vector<std::pair<int,int>> pairs_i;
        for (int a = 0; a < 5; ++a) for (int b = a; b < 5; ++b) pairs_i.push_back({a, b});
        int pi = std::atoi(ps);
        if (pi < 0 || pi >= (int)pairs_i.size()) return 0;
        int ka = pairs_i[pi].first, kb = pairs_i[pi].second;
        int NPOSE = 20;
        if (const char* np = std::getenv("SESSION_PRIMSWEEP_N")) NPOSE = std::atoi(np);
        unsigned seed = 20260726u + 7919u * (unsigned)pi;
        auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return (seed >> 8) / 16777216.0; };
        BRep A0 = kinds[ka].make();
        double volA = A0.volume();
        std::printf("[PRIM] pair %-9s vol(A)=%.6f\n",
                    (std::string(kinds[ka].name) + " x " + kinds[kb].name).c_str(), volA);
        for (int p = 0; p < NPOSE; ++p) {
            double ax = rnd()*2-1, ay = rnd()*2-1, az = rnd()*2-1;
            double m = std::sqrt(ax*ax+ay*ay+az*az); if (m < 1e-6) { ax=1; ay=0; az=0; m=1; }
            ax/=m; ay/=m; az/=m;
            double ang = rnd()*360.0;
            // translation magnitude spread so the overlap fraction spans shallow..deep
            double f = 0.15 + 0.70 * rnd();
            double d = f * (kinds[ka].rad + kinds[kb].rad);
            double tx = rnd()*2-1, ty = rnd()*2-1, tz = rnd()*2-1;
            double tm = std::sqrt(tx*tx+ty*ty+tz*tz); if (tm < 1e-6) { tx=1; ty=0; tz=0; tm=1; }
            tx = tx/tm*d; ty = ty/tm*d; tz = tz/tm*d;
            Xform R = Xform::rotation_around_line(Line(0,0,0, ax,ay,az), ang, true);
            Xform M = Xform::translation(tx, ty, tz) * R;
            double v[3] = {0,0,0}; int nk[3] = {0,0,0}, nm[3] = {0,0,0}, nf[3] = {0,0,0}, sol[3] = {0,0,0};
            double cl[3] = {0,0,0};
            long ms_tot = 0; bool threw = false;
            for (int op = 0; op < 3; ++op) {
                BRep A2 = kinds[ka].make();
                BRep B2 = kinds[kb].make(); B2 = B2.transformed(M);
                auto t0 = std::chrono::steady_clock::now();
                try {
                    BRep r = op == 0 ? A2.boolean_difference(B2)
                           : op == 1 ? A2.boolean_intersection(B2)
                                     : A2.boolean_union(B2);
                    count_naked_nonmani(r, nk[op], nm[op]);   // degeneracy-aware
                    nf[op] = r.face_count(); sol[op] = r.is_solid() ? 1 : 0;
                    // SESSION_PRIMSWEEP_NOVOL: skip volume() so a hang can be attributed to
                    // the BOOLEAN or to volume() -- session C measured volume() failing to
                    // terminate on NURBS solids, which would misattribute 127 "timeouts".
                    // NB: an EMPTY value must count as UNSET. "VAR=" makes getenv return "" (non-null), which
                    // silently disabled every volume and scored a whole 251-cell sweep CLOSED-WRONG.
                    const char* nv_env = std::getenv("SESSION_PRIMSWEEP_NOVOL");
                    static const bool s_novol = (nv_env != nullptr && nv_env[0] != '\0');
                    v[op] = (!s_novol && nk[op] == 0 && nm[op] == 0) ? r.volume() : std::nan("");
                    // CLOSURE. Volume alone hides the defect: unmerged/duplicated pieces can
                    // sum to the correct volume while the shell is not geometrically closed
                    // (measured: sphere x cylinder 35-44 deg). closure_residual = |sum of
                    // outward vector areas| / area; our own solids read 1e-17, legitimate
                    // curved results 1e-12, and brep_massprops itself declares a shell open
                    // above 1e-6.
                    cl[op] = (!s_novol) ? brep_massprops(r).closure_residual : 0.0;
                } catch (...) { threw = true; }
                ms_tot += (long)std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - t0).count();
            }
            bool clean = !threw && nk[0]==0 && nk[1]==0 && nm[0]==0 && nm[1]==0;
            double prel = std::nan("");
            if (clean && std::isfinite(v[0]) && std::isfinite(v[1]))
                prel = std::abs(v[0] + v[1] - volA) / std::max(1.0, volA);
            double clmax = std::max(std::max(cl[0], cl[1]), cl[2]);
            const char* verdict = threw ? "THREW"
                                : (nk[0]||nk[1]||nk[2]||nm[0]||nm[1]||nm[2]) ? "OPEN"
                                : (clmax > 1e-6) ? "UNCLOSED"
                                : (std::isfinite(prel) && prel < 1e-6) ? "EXACT" : "CLOSED-WRONG";
            std::printf("[PRIM] %-9s p%02d ang %6.1f f %.2f | cut %2df/%dnk/%dnm common %2df/%dnk/%dnm fuse %2df/%dnk/%dnm | part %9.2e | cl %8.1e | %5ldms | %s\n",
                        (std::string(kinds[ka].name)+"x"+kinds[kb].name).c_str(), p, ang, f,
                        nf[0], nk[0], nm[0], nf[1], nk[1], nm[1], nf[2], nk[2], nm[2],
                        prel, clmax, ms_tot, verdict);
            std::fflush(stdout);
        }
        return 0;
    }
    if (std::getenv("SESSION_REPARAM_CTRL")) {
        // THE CONTROL THAT DECIDES WHETHER THIS IS A STEP BUG OR A PARAMETERISATION BUG.
        // Take an IN-MEMORY box -- never written, never read -- and rescale each face's UV
        // domain WITHOUT touching geometry (same CVs, same surface, affine knot rescale; the
        // trims are rescaled identically so the face is geometrically unchanged). If the
        // boolean then breaks the same way the STEP round trip breaks, the defect is
        // PARAMETERISATION SENSITIVITY and has nothing to do with STEP -- STEP is merely how
        // we encountered it. Domain-relative tolerances (samp_tol = max(range)*2e-5,
        // eps_border = min(range)*2e-3, snap_uv, scaf_forced_eps = min_range*1.3e-1) are
        // measuring parameter space, which is an arbitrary modelling choice.
        auto run_cut = [&](const char* tag, BRep A2, BRep B2) {
            try {
                BRep cu = A2.boolean_difference(B2);
                int nk = 0; for (const auto& e : cu.m_topology_edges) if ((int)e.trim_indices.size() == 1) ++nk;
                std::printf("[REPARAM] %-22s cut faces %d solid %d naked %d vol %.6f\n",
                            tag, cu.face_count(), cu.is_solid() ? 1 : 0, nk, cu.volume());
            } catch (const std::exception& e) {
                std::printf("[REPARAM] %-22s THREW %s\n", tag, e.what());
            }
            std::fflush(stdout);
        };
        double scales[] = {1.0, 4.08, 100.0};
        for (double sc : scales) {
            BRep A2 = BRep::create_box(4, 4, 4);
            BRep B2 = BRep::create_box(4, 4, 4);
            B2 = B2.transformed(Xform::translation(1, 0, 0));
            if (sc != 1.0) {
                // rescale surface domains AND the trim pcurves by the same affine map
                auto reparam = [&](BRep& X) {
                    for (auto& s : X.m_surfaces) {
                        auto du = s.domain(0); auto dv = s.domain(1);
                        s.set_domain(0, du.first * sc, du.second * sc);
                        s.set_domain(1, dv.first * sc, dv.second * sc);
                    }
                    for (auto& pc : X.m_curves_2d)
                        for (int i = 0; i < pc.cv_count(); ++i) {
                            Point q = pc.get_cv(i);
                            pc.set_cv(i, Point(q[0]*sc, q[1]*sc, q[2]));
                        }
                };
                reparam(A2); reparam(B2);
            }
            char tag[64]; std::snprintf(tag, sizeof tag, "in-memory scale x%.2f", sc);
            run_cut(tag, A2, B2);
        }
        // THE ACTUAL DIFFERENCE THE READER INTRODUCES: the domain is PADDED BEYOND THE TRIMS,
        // so the face no longer FILLS its domain (STEP box face: domain [-0.04,4.04], trims
        // spanning [0,4] -- a 1% margin all round). In-memory faces always fill their domain
        // exactly, and several code paths special-case a full-domain rectangular face
        // (curved_rect / rect_trim / use_domain_border). Reproduce that here with geometry
        // and trims untouched: only the surface's domain is widened.
        for (double pad : {0.01, 0.04, 0.25}) {
            BRep A2 = BRep::create_box(4, 4, 4);
            BRep B2 = BRep::create_box(4, 4, 4);
            B2 = B2.transformed(Xform::translation(1, 0, 0));
            auto widen = [&](BRep& X) {
                for (auto& s : X.m_surfaces) {
                    auto du = s.domain(0); auto dv = s.domain(1);
                    double ru = du.second-du.first, rv = dv.second-dv.first;
                    s.set_domain(0, du.first - ru*pad, du.second + ru*pad);
                    s.set_domain(1, dv.first - rv*pad, dv.second + rv*pad);
                }
            };
            widen(A2); widen(B2);
            char tag[64]; std::snprintf(tag, sizeof tag, "domain PAD %.0f%% (trims fixed)", pad*100);
            run_cut(tag, A2, B2);
        }
        return 0;
    }
    if (std::getenv("SESSION_ROUNDTRIP_RESID")) {
        // THE DECISIVE COMPARISON (session C's ticket + session A's re-derivation reading):
        // the SAME primitive, in memory vs after a STEP round trip. In memory its analytic
        // parameters are exact by construction; from STEP they must be RE-FITTED from samples
        // of the returned NURBS (fit_cylinder 5x5 + PCA, etc). If analytic identity is
        // preserved through I/O the two residuals match; if it is discarded, the round-tripped
        // one is orders of magnitude worse -- and that single fact explains "exact in memory,
        // garbage from STEP" AND the 9.4e-05 rotated cluster with one mechanism.
        std::string dir = "/tmp/claude-1000/rt_resid";
        std::filesystem::create_directories(dir);
        struct C { const char* name; BRep b; };
        std::vector<C> cells;
        cells.push_back({"box",      BRep::create_box(4, 4, 4)});
        cells.push_back({"cylinder", BRep::create_cylinder(1.5, 6)});
        cells.push_back({"cone",     BRep::create_cone(2.0, 4.0)});
        cells.push_back({"sphere",   BRep::create_sphere(2.5)});
        cells.push_back({"torus",    BRep::create_torus(2.0, 0.8)});
        for (auto& c : cells) {
            std::string path = dir + "/" + c.name + ".step";
            file_step::write_file_step_brep(c.b, path);
            std::vector<BRep> back = file_step::read_file_step_breps(path);
            std::printf("\n[RT] %-9s in-memory vol %.9f | round-trip breps %zu vol %.9f\n",
                        c.name, c.b.volume(), back.size(),
                        back.empty() ? 0.0 : back[0].volume());
            std::fflush(stdout);
            // trigger recognition on both paths; SESSION_RECOG_RESID prints the residuals
            if (!back.empty()) {
                const BRep& r = back[0];
                std::printf("[RT] %-9s topo  in-mem F=%zu L=%zu T=%zu E=%zu V=%zu | STEP F=%zu L=%zu T=%zu E=%zu V=%zu | solid %d/%d\n",
                            c.name, c.b.m_faces.size(), c.b.m_loops.size(), c.b.m_trims.size(),
                            c.b.m_topology_edges.size(), c.b.m_topology_vertices.size(),
                            r.m_faces.size(), r.m_loops.size(), r.m_trims.size(),
                            r.m_topology_edges.size(), r.m_topology_vertices.size(),
                            c.b.is_solid() ? 1 : 0, r.is_solid() ? 1 : 0);
                // C's ticket: same solid vs itself translated +1, in memory vs from STEP
                BRep t_mem = c.b;  t_mem = t_mem.transformed(Xform::translation(1, 0, 0));
                BRep t_stp = r;    t_stp = t_stp.transformed(Xform::translation(1, 0, 0));
                auto rep = [&](const char* tag, const BRep& A2, const BRep& B2) {
                    try {
                        BRep cu = A2.boolean_difference(B2);
                        int nk = 0; for (const auto& e : cu.m_topology_edges) if ((int)e.trim_indices.size() == 1) ++nk;
                        std::printf("[RT] %-9s %-10s cut faces %d solid %d naked %d vol %.6f\n",
                                    c.name, tag, cu.face_count(), cu.is_solid() ? 1 : 0, nk, cu.volume());
                    } catch (const std::exception& e) {
                        std::printf("[RT] %-9s %-10s THREW %s\n", c.name, tag, e.what());
                    }
                };
                // Domains and degrees: identical topology + identical surfaces but a
                // different boolean means the difference is in the PARAMETERIZATION, and many
                // splitter tolerances are domain-relative (samp_tol = max(range)*2e-5,
                // eps_border = min(range)*2e-3, snap_uv ...), so a rescaled domain silently
                // rescales every one of them.
                for (size_t si = 0; si < c.b.m_surfaces.size() && si < 3; ++si) {
                    auto du0 = c.b.m_surfaces[si].domain(0); auto dv0 = c.b.m_surfaces[si].domain(1);
                    auto du1 = r.m_surfaces[si].domain(0);   auto dv1 = r.m_surfaces[si].domain(1);
                    std::printf("[RT] %-9s s%zu dom mem u[%.4f,%.4f] v[%.4f,%.4f] deg %d/%d | STEP u[%.4f,%.4f] v[%.4f,%.4f] deg %d/%d\n",
                                c.name, si, du0.first, du0.second, dv0.first, dv0.second,
                                c.b.m_surfaces[si].degree(0), c.b.m_surfaces[si].degree(1),
                                du1.first, du1.second, dv1.first, dv1.second,
                                r.m_surfaces[si].degree(0), r.m_surfaces[si].degree(1));
                }
                rep("mem-x-mem", c.b, t_mem);
                rep("stp-x-stp", r, t_stp);
                std::fflush(stdout);
            }
            std::fprintf(stderr, "[RT] === %s IN-MEMORY ===\n", c.name);
            for (const auto& s : c.b.m_surfaces) Intersection::surface_surface(s, s, 1e-6);
            if (!back.empty()) {
                std::fprintf(stderr, "[RT] === %s ROUND-TRIPPED ===\n", c.name);
                for (const auto& s : back[0].m_surfaces) Intersection::surface_surface(s, s, 1e-6);
            }
        }
        return 0;
    }
    if (std::getenv("SESSION_OPVOL")) {
        // Does a RIGID MOTION change an operand's own computed volume? It must not. This is
        // the cheapest possible check and it sits upstream of every boolean: if vol(boxR) is
        // not exactly vol(box), every cell using boxR inherits the error and no amount of
        // section/pcurve/flux work can fix it.
        std::printf("%-8s %14s %14s\n", "placement", "volume", "rel_vs_exact");
        struct { const char* key; double exact; } cells[] = {
            {"box", 64.0}, {"boxR", 64.0}, {"sph", 0.0}, {"cyl", 0.0},
            {"cylR", 0.0}, {"cone", 0.0}, {"coneR", 0.0}, {"tor", 0.0}, {"torR", 0.0},
        };
        for (auto& c : cells) {
            auto it = PL.find(c.key);
            if (it == PL.end()) continue;
            BRep b = build(it->second);
            double v = b.volume();
            if (c.exact > 0)
                std::printf("%-8s %14.9f  rel %.3e\n", c.key, v, std::abs(v-c.exact)/c.exact);
            else
                std::printf("%-8s %14.9f\n", c.key, v);
        }
        return 0;
    }
    if (std::getenv("SESSION_RELPOSE_SWEEP")) {
        relpose_sweep(PL["box"], PL["box2"], "box x box2 (planar)");
        relpose_sweep(PL["box"], PL["cyl"],  "box x cyl  (planar x cylinder)");
        relpose_sweep(PL["cyl"], PL["cyl2"], "cyl x cyl2 (cylinder x cylinder)");
        relpose_sweep(PL["box"], PL["sph"],  "box x sph  (planar x sphere)");
        relpose_sweep(PL["box"], PL["tor"],  "box x tor  (planar x torus)");
        relpose_sweep(PL["sph"], PL["cyl"],  "sph x cyl  (sphere x cylinder)");
        return 0;
    }
    if (std::getenv("SESSION_POSE_SWEEP")) {
        // pairs chosen to separate the variables: planar (exact when rotated), curved
        // axis-aligned (exact), curved+curved, and the torus (worst measured).
        pose_sweep(PL["box"],  PL["box2"], "box  x box2  (planar x planar)");
        pose_sweep(PL["cyl"],  PL["cyl2"], "cyl  x cyl2  (curved x curved, axis-aligned at 0)");
        pose_sweep(PL["box"],  PL["cyl"],  "box  x cyl   (planar x curved)");
        pose_sweep(PL["sph"],  PL["cyl"],  "sph  x cyl   (curved x curved)");
        pose_sweep(PL["box"],  PL["tor"],  "box  x tor   (torus)");
        return 0;
    }
    auto PRS = pairs();
    if (std::getenv("SESSION_EDGE")) {
        auto ep = edge_placements();
        PL.insert(ep.begin(), ep.end());
        PRS = edge_pairs();
    }
    bool occt2 = std::getenv("SESSION_OCCT2") != nullptr;
    if (occt2) {
        auto op2 = occt2_placements();
        PL.insert(op2.begin(), op2.end());
        PRS = occt2_pairs();
    }
    std::printf("%-13s %-4s | %11s %11s %9s | %4s %5s | s | %8s | verdict\n",
                "pair", "op", "our_vol", "occt_vol", "rel", "ourF", "occtF", "us");
    int fails = 0, total = 0;
    // SESSION_NO_ROT: skip the oriented battery (keys ending in 'R') -- their marcher cells
    // can hang and their volumes fail the gate; used when regenerating the shipped STEP set.
    bool no_rot = std::getenv("SESSION_NO_ROT") != nullptr;
    for (auto& pr : PRS) {
        if (!filter.empty() && pr[0].find(filter) == std::string::npos) continue;
        if (no_rot && (pr[1].back() == 'R' || pr[2].back() == 'R')) continue;
        for (const char* mode : {"cut", "common", "fuse"}) {
            const char* oponly = std::getenv("SESSION_OP");   // run a single op for diagnostics
            if (oponly && std::string(mode) != oponly) continue;
            ++total;
            const Place& A = PL[pr[1]]; const Place& B = PL[pr[2]];
            double v = 0; int nf = 0; int solid = 0; long us = 0;
            try {
                BRep ba = build(A), bb = build(B);
                if (std::getenv("SESSION_PAIR_SSI")) {
                    for (size_t ia = 0; ia < ba.m_surfaces.size(); ++ia)
                        for (size_t ib = 0; ib < bb.m_surfaces.size(); ++ib) {
                            auto trs = Intersection::surface_surface(ba.m_surfaces[ia], bb.m_surfaces[ib], 1e-6);
                            if (trs.empty()) continue;
                            std::printf("[PSSI] A%zu x B%zu: %zu curves:", ia, ib, trs.size());
                            for (auto& tr : trs) std::printf(" len=%.4f", std::get<0>(tr).length());
                            std::printf("\n");
                        }
                }
                auto t0 = std::chrono::steady_clock::now();
                BRep r = std::string(mode) == "cut"    ? ba.boolean_difference(bb)
                       : std::string(mode) == "common" ? ba.boolean_intersection(bb)
                                                        : ba.boolean_union(bb);
                us = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0).count();
                if (std::getenv("SESSION_ORIENT_DBG")) {
                    std::fprintf(stderr, "== %s %s ==\n", pr[0].c_str(), mode);
                    int nviol = r.check_trim_orientation(true);
                    std::fprintf(stderr, "== orient violations: %d ==\n", nviol);
                }
                if (const char* dp = std::getenv("SESSION_DUMP_PB")) r.pb_dump(dp);
                if (const char* sd = std::getenv("SESSION_STEP_DIR")) {
                    std::filesystem::create_directories(sd);
                    // ONE file per test operation: operand A (red), operand B (blue) and
                    // the boolean result (green) side by side in their tested positions.
                    // The written result is unified Rhino-style (coplanar splits merged);
                    // the gate below still checks the RAW result against raw OCCT counts.
                    ba.surfacecolor = Color(0.85f, 0.25f, 0.20f);
                    bb.surfacecolor = Color(0.20f, 0.45f, 0.85f);
                    // Cache the raw result BRep so pb2step can re-emit this STEP later without
                    // re-running the boolean (SESSION_PB_DUMP=<dir>): dump A, B, result as .pb.
                    if (const char* pd = std::getenv("SESSION_PB_DUMP")) {
                        std::filesystem::create_directories(pd);
                        std::string b = std::string(pd) + "/" + pr[1] + "_" + mode + "_" + pr[2];
                        ba.pb_dump(b + ".a.pb");
                        bb.pb_dump(b + ".b.pb");
                        if (r.face_count() > 0) r.pb_dump(b + ".r.pb");
                    }
                    BRep rm = r;
                    // Coplanar merge is DISPLAY-ONLY and OPT-IN (SESSION_MERGE): it stalls on
                    // box x tor fuse, so default is OFF -- ship the raw two-arc-split result.
                    if (std::getenv("SESSION_MERGE")) rm.merge_coplanar_faces();
                    // Merge is display-only, but it must not change the solid: check volume +
                    // watertightness against the raw result so a bad merge is caught, not shipped.
                    double vr = r.volume(), vm = rm.volume();
                    double mrel = std::abs(vr) > 1e-9 ? std::abs(vm - vr) / std::abs(vr) : std::abs(vm - vr);
                    if (mrel > 1e-6 || (r.is_solid() && !rm.is_solid()))
                        std::fprintf(stderr, "[MERGE-WARN] %s %s: vol raw=%.6f merged=%.6f rel=%.2e faces %d->%d solid %d->%d\n",
                                     pr[0].c_str(), mode, vr, vm, mrel, r.face_count(), rm.face_count(),
                                     r.is_solid() ? 1 : 0, rm.is_solid() ? 1 : 0);
                    rm.surfacecolor = Color(0.35f, 0.75f, 0.40f);
                    std::vector<const BRep*> parts = {&ba, &bb};
                    if (rm.face_count() > 0) parts.push_back(&rm);
                    file_step::write_file_step_breps(parts, pr[1] + "_" + mode + "_" + pr[2],
                        std::string(sd) + "/" + pr[1] + "_" + mode + "_" + pr[2] + ".step");
                }
                v = r.volume(); nf = r.face_count(); solid = r.is_solid() ? 1 : 0;
                // PER-FACE ATTRIBUTION. A cell that is wrong by 1e-5 in TOTAL says nothing
                // about which face carries it; the per-face area/flux does. Prints each
                // face's trim kinds too, so an exact rational pcurve and a fitted sampled
                // one are told apart at a glance.
                if (const char* mf = std::getenv("SESSION_MPFACE"); mf && mf[0]) {
                    MassProps m = brep_massprops(r);
                    {   // MEASURED edge tolerances: how far each face's pcurve, lifted through
                        // its surface, actually sits from the edge's own 3D curve.
                        BRep rt = r;
                        double wt = rt.update_tolerances();
                        std::vector<std::pair<double,int>> tl;
                        for (int ei = 0; ei < (int)rt.m_topology_edges.size(); ++ei)
                            tl.push_back({rt.m_topology_edges[ei].tolerance, ei});
                        std::sort(tl.rbegin(), tl.rend());
                        std::printf("[ETOL] %s %s edges=%d worst=%.3e top:",
                                    pr[0].c_str(), mode, (int)tl.size(), wt);
                        for (size_t i = 0; i < tl.size() && i < 8; ++i)
                            std::printf(" e%d=%.2e", tl[i].second, tl[i].first);
                        std::printf("\n");
                    }
                    std::printf("[MPF] %s %s vol=%.12f err=%.3e conv=%d closed=%d closure=%.2e gap=%.2e evals=%lld\n",
                                pr[0].c_str(), mode, m.volume, m.volume_error, m.converged?1:0,
                                m.closed?1:0, m.closure_residual, m.max_chain_gap, m.surface_evals);
                    for (const auto& fm : m.faces) {
                        int fi = fm.face_index;
                        std::string kinds;
                        if (fi >= 0 && fi < (int)r.m_faces.size())
                            for (int li : r.m_faces[fi].loop_indices) {
                                if (li < 0 || li >= (int)r.m_loops.size()) continue;
                                for (int ti : r.m_loops[li].trim_indices) {
                                    if (ti < 0 || ti >= (int)r.m_trims.size()) continue;
                                    int c2 = r.m_trims[ti].curve_2d_index;
                                    if (c2 < 0 || c2 >= (int)r.m_curves_2d.size()) continue;
                                    const NurbsCurve& pc = r.m_curves_2d[c2];
                                    kinds += " d" + std::to_string(pc.degree())
                                           + (pc.is_rational() ? "r" : "p")
                                           + "/" + std::to_string(pc.cv_count());
                                }
                            }
                        std::printf("[MPF]   f%-2d path=%d area=%.12f flux=%.12f aerr=%.2e ferr=%.2e out=%+.0f gap=%.1e |%s\n",
                                    fi, (int)fm.path, fm.area, fm.flux, fm.area_err, fm.flux_err,
                                    fm.outward, fm.chain_gap, kinds.c_str());
                    }
                }
            } catch (const std::exception& e) {
                std::printf("%-13s %-4s | THREW: %s\n", pr[0].c_str(), mode, e.what()); ++fails; continue;
            } catch (...) { std::printf("%-13s %-4s | THREW\n", pr[0].c_str(), mode); ++fails; continue; }

            std::string key = pr[0] + "|" + mode;
            auto k = occt(key, mode, A, B, have_oracle, oracle, req, res, cache, refresh, dirty);
            if (k[2] == 0.0) {
                std::printf("%-13s %-4s | %11.4f %11s %9s | %4d %5s | %d | %8ld | %s\n",
                            pr[0].c_str(), mode, v, "-", "-", nf, "-", solid, us,
                            have_oracle ? "OCCT-FAIL" : "no-ref");
                ++fails; continue;
            }
            double rel = k[0] != 0 ? std::abs(v - k[0]) / std::abs(k[0]) : std::abs(v - k[0]);
            // A genuinely EMPTY result matching OCCT's empty result is correct (e.g. cone x torus
            // common: the cone solid never reaches the tube); is_solid() on an empty BRep is false.
            bool both_empty = std::abs(k[0]) < 1e-12 && (int)k[1] == 0 && std::abs(v) < 1e-12 && nf == 0;
            bool ok = both_empty || (rel < 1e-6 && nf == (int)k[1] && solid);
            if (!ok) ++fails;
            std::printf("%-13s %-4s | %11.4f %11.4f %9.2e | %4d %5d | %d | %8ld | %s\n",
                        pr[0].c_str(), mode, v, k[0], rel, nf, (int)k[1], solid, us, ok ? "OK" : "FAIL");
            // Full-precision echo. A 3-digit `rel` cannot tell "the change did nothing" from
            // "the change moved it by 0.4%", and that distinction decided whether the pcurve
            // deviation is the error source at all.
            if (const char* vp = std::getenv("SESSION_VOLPREC"); vp && vp[0])
                std::printf("   [VP] %-13s %-6s our %.12f occt %.12f rel %.6e\n",
                            pr[0].c_str(), mode, v, k[0], rel);
        }
        // Composite-op identities against the cached references:
        //   vol(xor)  == fuse - common,   sum(split fragments) == fuse.
        if (std::getenv("SESSION_XOR_CHECK")) {
            auto kf = cache.find(pr[0] + "|fuse"); auto kc = cache.find(pr[0] + "|common");
            if (kf != cache.end() && kc != cache.end()) {
                const Place& A = PL[pr[1]]; const Place& B = PL[pr[2]];
                BRep ba = build(A), bb = build(B);
                double vfuse = kf->second.vol, vcom = kc->second.vol;
                try {
                    BRep x = ba.boolean_xor(bb);
                    auto frags = ba.boolean_split(bb);
                    double vx = x.volume(), vs = 0;
                    for (auto& f : frags) vs += f.volume();
                    if (const char* xd = std::getenv("SESSION_XOR_DBG")) {
                        for (size_t fi = 0; fi < frags.size(); ++fi)
                            std::fprintf(stderr, "[XORDBG] frag %zu faces %d solid %d vol %.10f\n",
                                         fi, frags[fi].face_count(), frags[fi].is_solid() ? 1 : 0,
                                         frags[fi].volume());
                        if (xd[0] == 'd')
                            for (size_t fi = 0; fi < frags.size(); ++fi)
                                frags[fi].pb_dump("xor_frag_" + std::to_string(fi) + ".pb");
                    }
                    double xref = vfuse - vcom;
                    double xr = xref != 0 ? std::abs(vx - xref) / std::abs(xref) : std::abs(vx);
                    double sr = vfuse != 0 ? std::abs(vs - vfuse) / vfuse : std::abs(vs);
                    ++total; if (!(xr < 1e-6 && sr < 1e-6 && x.is_solid())) ++fails;
                    std::printf("%-13s xor  | %11.4f %11.4f %9.2e | slds %d xf %d frags %d sum %11.4f %9.2e | %s\n",
                                pr[0].c_str(), vx, xref, xr, x.is_solid()?1:0, x.face_count(), (int)frags.size(), vs, sr,
                                (xr < 1e-6 && sr < 1e-6 && x.is_solid()) ? "OK" : "FAIL");
                } catch (const std::exception& e) {
                    ++total; ++fails;
                    std::printf("%-13s xor  | THREW: %s\n", pr[0].c_str(), e.what());
                }
            }
        }
    }
    // OCCT-port chain cells: boolean-of-boolean stress (three-tori fuse, Steinmetz
    // tricylinder, washer x torus, sphere-grid progressive fuse). Oracle side uses the
    // boolean_chain op; cache key = label.
    if (occt2) {
        for (const auto& cc : occt2_chains()) {
            if (!filter.empty() && cc.label.find(filter) == std::string::npos) continue;
            ++total;
            double v = 0; int nf = 0, solid = 0; long us = 0;
            try {
                auto t0 = std::chrono::steady_clock::now();
                BRep r = build(PL[cc.keys[0]]);
                for (size_t i = 0; i < cc.modes.size(); ++i) {
                    BRep s = build(PL[cc.keys[i + 1]]);
                    r = cc.modes[i] == "cut"    ? r.boolean_difference(s)
                      : cc.modes[i] == "common" ? r.boolean_intersection(s)
                                                 : r.boolean_union(s);
                }
                us = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0).count();
                v = r.volume(); nf = r.face_count(); solid = r.is_solid() ? 1 : 0;
            } catch (const std::exception& e) {
                std::printf("%-13s chn  | THREW: %s\n", cc.label.c_str(), e.what()); ++fails; continue;
            } catch (...) { std::printf("%-13s chn  | THREW\n", cc.label.c_str()); ++fails; continue; }
            std::array<double,3> k = {0, 0, 0};
            auto it = cache.find(cc.label);
            if (!refresh && it != cache.end()) k = {it->second.vol, (double)it->second.nf, 1.0};
            else if (have_oracle) {
                {
                    std::ofstream f(req);
                    f << "OP boolean_chain\nNOPS " << cc.modes.size() << "\nMODES";
                    for (const auto& m : cc.modes) f << " " << m;
                    f << "\n";
                    for (const auto& key : cc.keys) f << shape_str(PL[key]) << "\n";
                }
                if (std::system((oracle + " " + req + " " + res).c_str()) == 0) {
                    std::ifstream f(res); std::string tok2; double vol = 0; int nf2 = 0; bool okv = false, okf = false;
                    while (f >> tok2) { if (tok2 == "VOLUME") { f >> vol; okv = true; }
                                        else if (tok2 == "NFACES") { f >> nf2; okf = true; } }
                    if (okv && okf) { cache[cc.label] = {vol, nf2}; dirty = true; k = {vol, (double)nf2, 1.0}; }
                }
            }
            if (k[2] == 0.0) {
                std::printf("%-13s chn  | %11.4f %11s %9s | %4d %5s | %d | %8ld | %s\n",
                            cc.label.c_str(), v, "-", "-", nf, "-", solid, us,
                            have_oracle ? "OCCT-FAIL" : "no-ref");
                ++fails; continue;
            }
            double rel = k[0] != 0 ? std::abs(v - k[0]) / std::abs(k[0]) : std::abs(v - k[0]);
            bool ok = rel < 1e-6 && nf == (int)k[1] && solid;
            if (!ok) ++fails;
            std::printf("%-13s chn  | %11.4f %11.4f %9.2e | %4d %5d | %d | %8ld | %s\n",
                        cc.label.c_str(), v, k[0], rel, nf, (int)k[1], solid, us, ok ? "OK" : "FAIL");
        }
    }

    // Freeform probe: a perturbed-sphere closed NURBS solid (interior CVs moved radially by
    // +-12%; pole rows and seam columns untouched so create_sphere's topology stays valid --
    // the seam gains a C0 tangent kink, which a BRep legitimately allows). Quadric recognition
    // rejects it (1e-4 gate), so SSI runs the general marcher: this cell exercises exactly the
    // freeform path. No oracle can build this shape; verification is by EXACT volume identities:
    //   [id1] vol(box-ff) + vol(box&ff) == vol(box) = 64   (partition of the box)
    //   [id2] vol(fuse) == 64 + vol(ff) - vol(box&ff)      (inclusion-exclusion)
    if (std::getenv("SESSION_FREEFORM")) {
        BRep ff = BRep::create_sphere(2.5);
        NurbsSurface s = ff.m_surfaces[0];
        int nu = s.cv_count(0), nv = s.cv_count(1);
        for (int i = 1; i + 1 < nu; ++i)
            for (int j = 1; j + 1 < nv; ++j) {
                double x, y, z, w;
                if (!s.get_cv_4d(i, j, x, y, z, w)) continue;
                double f = 1.0 + 0.12 * std::sin(2.1 * i + 1.3 * j);
                s.set_cv_4d(i, j, x * f, y * f, z * f, w);
            }
        ff.m_surfaces[0] = s;
        double vff = ff.volume();
        // MULTI-FACE freeform brep (chair-cushion class): box whose 6 planar faces become
        // bulged bicubic 6x6 patches. The bulge is sin(pi u)*sin(pi v) -- ZERO on every
        // boundary, and boundary CVs sample the original planes, so all 12 box edges stay
        // EXACT straight lines: the solid remains watertight with untouched topology.
        {
            BRep pil = BRep::create_box(4, 4, 2);
            pil.name = "pillow";
            const double PI_ = 3.14159265358979323846;
            for (size_t sf = 0; sf < pil.m_surfaces.size(); ++sf) {
                NurbsSurface& S0 = pil.m_surfaces[sf];
                auto du = S0.domain(0); auto dv = S0.domain(1);
                Vector nm = S0.normal_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
                const int N = 6;
                std::vector<std::vector<Point>> grid(N, std::vector<Point>(N, Point(0,0,0)));
                std::vector<std::vector<double>> wts(N, std::vector<double>(N, 1.0));
                for (int iv = 0; iv < N; ++iv)
                    for (int iu = 0; iu < N; ++iu) {
                        double fu = (double)iu / (N-1), fv = (double)iv / (N-1);
                        Point q = S0.point_at(du.first + (du.second-du.first)*fu,
                                              dv.first + (dv.second-dv.first)*fv);
                        double h = 0.45 * std::sin(PI_*fu) * std::sin(PI_*fv)
                                 * (1.0 + 0.3*std::sin(2.0*fu + 3.0*fv + (double)sf));
                        grid[iv][iu] = Point(q[0]+nm[0]*h, q[1]+nm[1]*h, q[2]+nm[2]*h);
                    }
                double a = du.first, b = du.second, c = dv.first, d = dv.second;
                std::vector<double> KU = {a, a+(b-a)/3, a+2*(b-a)/3, b};
                std::vector<double> KV = {c, c+(d-c)/3, c+2*(d-c)/3, d};
                std::vector<int> MU = {4, 1, 1, 4}, MV = {4, 1, 1, 4};
                NurbsSurface S1 = NurbsSurface::create_from_parameters(grid, wts, KU, KV, MU, MV, 3, 3);
                if (S1.is_valid()) pil.m_surfaces[sf] = S1;
            }
            std::printf("freeform pillow: vol %.4f solid %d faces %d\n",
                        pil.volume(), pil.is_solid() ? 1 : 0, pil.face_count());
            // single-operand file: gated for the same reason as freeform_blob.step above.
            if (const char* sd = std::getenv("SESSION_STEP_DIR"))
                if (std::getenv("SESSION_STEP_EXTRAS"))
                    file_step::write_file_step_brep(pil, std::string(sd) + "/freeform_pillow.step");
        }
        BRep box = BRep::create_box(4, 4, 4);
        if (std::getenv("SESSION_SSI_DBG")) {
            const NurbsSurface& bs = ff.m_surfaces[0];
            for (size_t k = 0; k < box.m_surfaces.size(); ++k) {
                auto trs = Intersection::surface_surface(bs, box.m_surfaces[k], 1e-6);
                std::printf("[SSIDBG] boxface %zu: %zu sections\n", k, trs.size());
                for (auto& tr : trs) {
                    NurbsCurve& c = std::get<0>(tr);
                    auto d = c.domain();
                    Point a = c.point_at(d.first), b2 = c.point_at(d.second);
                    NurbsCurve& pa = std::get<1>(tr);
                    auto dp = pa.domain();
                    Point ua = pa.point_at(dp.first), ub = pa.point_at(dp.second);
                    double lift_len = 0, lift_dev = 0;
                    Point prev = bs.point_at(ua[0], ua[1]);
                    for (int k2 = 1; k2 <= 512; ++k2) {
                        Point uv = pa.point_at(dp.first + (dp.second - dp.first) * k2 / 512.0);
                        Point q = bs.point_at(uv[0], uv[1]);
                        lift_len += prev.distance(q); prev = q;
                        Point cp = c.closest_point(q);
                        lift_dev = std::max(lift_dev, q.distance(cp));
                    }
                    std::printf("   len=%.3f liftA=%.3f devA=%.4f closed=%d a(%.2f,%.2f,%.2f) b(%.2f,%.2f,%.2f) pcA(%.3f,%.3f)->(%.3f,%.3f)\n",
                                c.length(), lift_len, lift_dev, a.distance(b2) < 1e-5 ? 1 : 0,
                                a[0], a[1], a[2], b2[0], b2[1], b2[2],
                                ua[0], ua[1], ub[0], ub[1]);
                }
            }
        }
        double vcut = 0, vcom = 0, vfus = 0;
        int fcut = 0, fcom = 0, ffus = 0; int scut = 0, scom = 0, sfus = 0;
        long our_us[3] = {0, 0, 0};
        const char* modes[3] = {"cut", "common", "fuse"};
        for (int m = 0; m < 3; ++m) {
            auto t0 = std::chrono::steady_clock::now();
            BRep r = m == 0 ? box.boolean_difference(ff)
                   : m == 1 ? box.boolean_intersection(ff)
                            : box.boolean_union(ff);
            our_us[m] = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - t0).count();
            if (const char* sd = std::getenv("SESSION_STEP_DIR")) {
                std::filesystem::create_directories(sd);
                BRep ia = box, ib = ff;
                ia.surfacecolor = Color(0.85f, 0.25f, 0.20f);
                ib.surfacecolor = Color(0.20f, 0.45f, 0.85f);
                r.surfacecolor  = Color(0.35f, 0.75f, 0.40f);
                file_step::write_file_step_breps({&ia, &ib, &r},
                    std::string("box_") + modes[m] + "_blob",
                    std::string(sd) + "/freeform_" + modes[m] + "_box.step");
                // operand-only file: kept behind a gate so the SHIPPED inspection set stays
                // one-file-per-cell (A+B+result). The user inspects every file and asked for
                // no extra variants alongside the result files.
                if (m == 0 && std::getenv("SESSION_STEP_EXTRAS"))
                    file_step::write_file_step_brep(ff, std::string(sd) + "/freeform_blob.step");
            }
            if (std::getenv("SESSION_SOLID_DBG")) {
                int bad = 0;
                for (auto& e : r.m_topology_edges) {
                    if ((int)e.trim_indices.size() == 2) continue;
                    int ci = e.curve_3d_index;
                    if (ci < 0 || ci >= (int)r.m_curves_3d.size()) continue;
                    const NurbsCurve& c = r.m_curves_3d[ci];
                    auto dc = c.domain();
                    Point a = c.point_at(dc.first), b2 = c.point_at(dc.second);
                    if (c.length() < 1e-6) continue;
                    if (bad < 8) {
                        std::string owners;
                        for (int ti : e.trim_indices) {
                            const auto& tr2 = r.m_trims[ti];
                            int fi = r.m_loops[tr2.loop_index].face_index;
                            int si = r.m_faces[fi].surface_index;
                            char ob[64];
                            std::snprintf(ob, sizeof(ob), " f%d/s%d", fi, si);
                            owners += ob;
                        }
                        std::printf("[UNMATED] %s e=%d trims=%zu len=%.4f a(%.2f,%.2f,%.2f) b(%.2f,%.2f,%.2f)%s\n",
                                    modes[m], (int)(&e - &r.m_topology_edges[0]), e.trim_indices.size(), c.length(),
                                    a[0], a[1], a[2], b2[0], b2[1], b2[2], owners.c_str());
                    }
                    ++bad;
                }
                std::printf("[UNMATED] %s total=%d\n", modes[m], bad);
            }
            double v = r.volume();
            if (m == 0) { vcut = v; fcut = r.face_count(); scut = r.is_solid(); }
            if (m == 1) { vcom = v; fcom = r.face_count(); scom = r.is_solid(); }
            if (m == 2) { vfus = v; ffus = r.face_count(); sfus = r.is_solid(); }
        }
        double id1 = std::abs(vcut + vcom - 64.0) / 64.0;
        double id2 = std::abs(vfus - (64.0 + vff - vcom)) / std::abs(vfus);
        // OCCT reference: ship the blob surface VERBATIM (SHAPE nurbs) so the oracle builds
        // the identical solid; compare volume/face counts and wall time per op.
        double occ_v[3] = {0, 0, 0}; int occ_f[3] = {0, 0, 0}; long occ_us[3] = {0, 0, 0};
        bool occ_ok = have_oracle;
        if (have_oracle) {
            std::ostringstream nb;
            nb << std::setprecision(17);
            nb << "SHAPE nurbs " << (s.m_is_rat ? 1 : 0) << " " << s.degree(0) << " " << s.degree(1)
               << " " << s.cv_count(0) << " " << s.cv_count(1) << "\n";
            for (double k : s.m_nurbsknot[0]) nb << k << " ";
            nb << "\n";
            for (double k : s.m_nurbsknot[1]) nb << k << " ";
            nb << "\n";
            for (int i = 0; i < s.cv_count(0); ++i)
                for (int j = 0; j < s.cv_count(1); ++j) {
                    Point p = s.get_cv(i, j);
                    nb << p[0] << " " << p[1] << " " << p[2] << " " << s.weight(i, j) << "\n";
                }
            nb << "XF 0 0 0 0 0 1 0\n";
            for (int m = 0; m < 3 && occ_ok; ++m) {
                { std::ofstream f(req);
                  f << "OP boolean\nMODE " << modes[m]
                    << "\nSHAPE box 4 4 4 XF 0 0 0 0 0 1 0\n" << nb.str(); }
                auto t0 = std::chrono::steady_clock::now();
                if (std::system((oracle + " " + req + " " + res).c_str()) != 0) { occ_ok = false; break; }
                occ_us[m] = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0).count();
                std::ifstream f(res); std::string tok; bool okv = false;
                while (f >> tok) {
                    if (tok == "VOLUME") { f >> occ_v[m]; okv = true; }
                    else if (tok == "NFACES") { f >> occ_f[m]; }
                }
                if (!okv) occ_ok = false;
            }
        }
        std::printf("\nfreeform blob (perturbed sphere, vol %.4f)  x  box  [ours vs OCCT verbatim-NURBS]:\n", vff);
        double vv[3] = {vcut, vcom, vfus}; int fc2[3] = {fcut, fcom, ffus}; int sl[3] = {scut, scom, sfus};
        for (int m = 0; m < 3; ++m) {
            double rel = (occ_ok && occ_v[m] != 0) ? std::abs(vv[m]-occ_v[m])/std::abs(occ_v[m]) : -1.0;
            std::printf("  %-6s ours %11.4f f%-3d s%d %7ld us | occt %11.4f f%-3d %7ld us | rel %9.2e\n",
                        modes[m], vv[m], fc2[m], sl[m], our_us[m],
                        occ_ok ? occ_v[m] : 0.0, occ_ok ? occ_f[m] : -1, occ_us[m], rel);
        }
        std::printf("  [id1] cut+com-64       rel %9.2e\n", id1);
        std::printf("  [id2] fuse-(64+ff-com) rel %9.2e\n", id2);
        bool ok = id1 < 1e-6 && scut && scom && sfus;
        if (occ_ok) for (int m = 0; m < 3; ++m)
            if (occ_v[m] != 0 && std::abs(vv[m]-occ_v[m])/std::abs(occ_v[m]) > 1e-6) ok = false;
        ++total; if (!ok) ++fails;
        std::printf("  freeform verdict: %s\n", ok ? "OK" : "FAIL");
    }
    // Offline naked-edge trichotomy: for each 1-trim edge of the dumped boolean result, find
    // its nearest counterpart among the FULL pre-classification splits (A2/B2) and among the
    // result's own edges -- separates classification drops (counterpart in split only) from
    // SSI incompleteness (no counterpart anywhere) from sew misses (counterpart kept, unmated).
    if (const char* ad = std::getenv("SESSION_ANALYZE_NAKED")) {
        BRep R = BRep::pb_load(std::string(ad) + "/chair_cut.pb");
        BRep A2 = BRep::pb_load(std::string(ad) + "/split_A2.pb");
        BRep B2 = BRep::pb_load(std::string(ad) + "/split_B2.pb");
        auto sample_edges = [](const BRep& X) {
            std::vector<std::vector<Point>> out(X.m_topology_edges.size());
            for (size_t e = 0; e < X.m_topology_edges.size(); ++e) {
                int ci = X.m_topology_edges[e].curve_3d_index;
                if (ci < 0 || ci >= (int)X.m_curves_3d.size()) continue;
                const NurbsCurve& C = X.m_curves_3d[ci];
                auto [t0, t1] = C.domain();
                for (int k = 0; k <= 24; ++k) out[e].push_back(C.point_at(t0 + (t1 - t0) * k / 24.0));
            }
            return out;
        };
        auto sR = sample_edges(R), sA = sample_edges(A2), sB = sample_edges(B2);
        auto p2seg = [](const Point& p, const std::vector<Point>& pl) {
            double best = 1e300;
            for (size_t j = 0; j + 1 < pl.size(); ++j) {
                const Point& a = pl[j]; const Point& b = pl[j + 1];
                double ex=b[0]-a[0], ey=b[1]-a[1], ez=b[2]-a[2], L2=ex*ex+ey*ey+ez*ez;
                double t=(L2>1e-30)?((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2:0.0;
                t=std::min(std::max(t,0.0),1.0);
                double cx=a[0]+t*ex, cy=a[1]+t*ey, cz=a[2]+t*ez;
                best=std::min(best,std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz)));
            }
            return best;
        };
        auto nearest = [&](const std::vector<Point>& probe, const std::vector<std::vector<Point>>& pool, int skip) {
            double bh = 1e300; int bi = -1;
            for (size_t j = 0; j < pool.size(); ++j) {
                if ((int)j == skip || pool[j].size() < 2) continue;
                double h = 0.0;
                for (const auto& p : probe) { h = std::max(h, p2seg(p, pool[j])); if (h >= bh) break; }
                if (h < bh) { bh = h; bi = (int)j; }
            }
            return std::make_pair(bh, bi);
        };
        for (size_t e = 0; e < R.m_topology_edges.size(); ++e) {
            if ((int)R.m_topology_edges[e].trim_indices.size() != 1 || sR[e].size() < 2) continue;
            auto [ha, ia] = nearest(sR[e], sA, -1);
            auto [hb, ib] = nearest(sR[e], sB, -1);
            auto [hr, ir] = nearest(sR[e], sR, (int)e);
            std::printf("[AN] e=%zu A2(h=%.4f e=%d) B2(h=%.4f e=%d) R(h=%.4f e=%d nt=%d)\n",
                        e, ha, ia, hb, ib, hr, ir,
                        ir >= 0 ? (int)R.m_topology_edges[ir].trim_indices.size() : -1);
        }
        return 0;
    }
    // Real-world STEP round-trip: read two chair models, run all three booleans, write one
    // colored STEP per op (A red / B blue / result green) and check the volume identities.
    if (const char* cd = std::getenv("SESSION_CHAIRS")) {
        auto as = file_step::read_file_step_breps(std::string(cd) + "/chair0.stp");
        auto bs = file_step::read_file_step_breps(std::string(cd) + "/chair1.stp");
        std::printf("chairs: A breps %zu, B breps %zu\n", as.size(), bs.size());
        if (!as.empty() && !bs.empty()) {
            BRep& A = as[0]; BRep& B = bs[0];
            if (std::getenv("SESSION_CHAIRS_DUMP")) {
                A.pb_dump(std::string(cd) + "/chair0.pb");
                B.pb_dump(std::string(cd) + "/chair1.pb");
            }
            bool fast = std::getenv("SESSION_FAST") != nullptr;  // skip slow volume()/STEP for topology-only sweeps
            if (fast)
                std::printf("A: faces %d solid %d | B: faces %d solid %d\n",
                            A.face_count(), A.is_solid() ? 1 : 0,
                            B.face_count(), B.is_solid() ? 1 : 0);
            else
                std::printf("A: faces %d solid %d vol %.4f | B: faces %d solid %d vol %.4f\n",
                            A.face_count(), A.is_solid() ? 1 : 0, A.volume(),
                            B.face_count(), B.is_solid() ? 1 : 0, B.volume());
            A.surfacecolor = Color(0.85f, 0.25f, 0.20f);
            B.surfacecolor = Color(0.20f, 0.45f, 0.85f);
            // Z90 CLASSIFIER PROBE (no boolean, seconds): rotate B by z90 about the joint
            // centroid, then at a set of test points compare point-in-B classifiers against
            // each other -- current winding on B.mesh(), the raw winding omega, a finer mesh,
            // and a multi-ray parity vote. Feed the known-bad z90 fragment samples (f2, f40)
            // and a small grid; compare to the OCCT oracle offline. Lets classifier ideas
            // iterate in seconds instead of 10-min booleans.
            if (std::getenv("SESSION_Z90_PROBE")) {
                auto vmeanp = [](const BRep& X) {
                    Point c(0,0,0);
                    for (const auto& p : X.m_vertices) c = Point(c[0]+p[0], c[1]+p[1], c[2]+p[2]);
                    double n = (double)std::max<size_t>(1, X.m_vertices.size());
                    return Point(c[0]/n, c[1]/n, c[2]/n);
                };
                Point ca = vmeanp(A), cb = vmeanp(B);
                Point C((ca[0]+cb[0])*0.5, (ca[1]+cb[1])*0.5, (ca[2]+cb[2])*0.5);
                Xform M = Xform::translation(C[0],C[1],C[2]) * Xform::rotation_z(90.0, true)
                        * Xform::translation(-C[0],-C[1],-C[2]);
                BRep Bp = B; Bp = Bp.transformed(M);
                Mesh mb = Bp.mesh();
                Mesh mbq = Bp.mesh();   // placeholder; refine below if a quality mesh helps
                // winding omega (raw) at p against a mesh
                auto omega_of = [](const Mesh& mesh, const Point& p) {
                    auto [vs, fs] = mesh.to_vertices_and_faces();
                    double om = 0.0;
                    for (const auto& f : fs) {
                        if (f.size() < 3) continue;
                        for (size_t j = 1; j + 1 < f.size(); ++j) {
                            const Point& A2 = vs[f[0]]; const Point& B2 = vs[f[j]]; const Point& Cc = vs[f[j+1]];
                            double ax=A2[0]-p[0],ay=A2[1]-p[1],az=A2[2]-p[2];
                            double bx=B2[0]-p[0],by=B2[1]-p[1],bz=B2[2]-p[2];
                            double cx=Cc[0]-p[0],cy=Cc[1]-p[1],cz=Cc[2]-p[2];
                            double la=std::sqrt(ax*ax+ay*ay+az*az),lb=std::sqrt(bx*bx+by*by+bz*bz),lc=std::sqrt(cx*cx+cy*cy+cz*cz);
                            if (la<1e-15||lb<1e-15||lc<1e-15) return 12.6;
                            double det=ax*(by*cz-bz*cy)-ay*(bx*cz-bz*cx)+az*(bx*cy-by*cx);
                            double den=la*lb*lc+(ax*bx+ay*by+az*bz)*lc+(bx*cx+by*cy+bz*cz)*la+(cx*ax+cy*ay+cz*az)*lb;
                            om += 2.0*std::atan2(det,den);
                        }
                    }
                    return om;
                };
                // multi-ray parity: cast N rays in fixed skew directions, count crossings, majority-in
                auto ray_parity = [&](const Mesh& mesh, const Point& p) {
                    double D[7][3] = {{1,0.03,0.017},{0.021,1,0.033},{0.013,0.027,1},
                                      {1,1,0.7},{1,-0.6,0.4},{-0.5,1,0.8},{0.9,0.4,-1}};
                    int inv = 0, tot = 0;
                    for (auto& d : D) {
                        Line ray(p[0],p[1],p[2], p[0]+d[0]*1000, p[1]+d[1]*1000, p[2]+d[2]*1000);
                        auto hits = Intersection::ray_mesh(ray, mesh, 1e-9, true);
                        ++tot; if ((int)hits.size() % 2 == 1) ++inv;
                    }
                    return inv * 2 > tot;
                };
                // consistently-ORIENTED winding: sum per-face solid angle * outward sign, so a
                // mesh with inconsistent per-face winding no longer partially cancels to garbage.
                std::vector<Mesh> fms = Bp.face_meshes();
                std::vector<double> fsg = Bp.face_outward_signs();
                auto oriented_omega = [&](const Point& p) {
                    double om = 0.0;
                    for (size_t fi = 0; fi < fms.size(); ++fi) {
                        double s = (fi < fsg.size()) ? fsg[fi] : 1.0;
                        om += s * omega_of(fms[fi], p);
                    }
                    return om;
                };
                // EXACT classifier (no mesh): closest point on the TRIMMED boundary + outward
                // normal sign. Immune to any tessellation over/under-coverage.
                auto uv_in_trims = [&](const BRep& X, int fi, double u, double v) -> bool {
                    const auto& face = X.m_faces[fi];
                    bool in_outer = false; bool have_outer = false;
                    for (int li : face.loop_indices) {
                        if (li < 0 || li >= (int)X.m_loops.size()) continue;
                        const auto& loop = X.m_loops[li];
                        std::vector<std::array<double,2>> poly;
                        for (int ti : loop.trim_indices) {
                            if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                            int c2 = X.m_trims[ti].curve_2d_index;
                            if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                            const NurbsCurve& pc = X.m_curves_2d[c2];
                            auto dc = pc.domain();
                            int ns = std::min(std::max(pc.cv_count()*2, 16), 128);
                            for (int k = 0; k < ns; ++k) {
                                Point q = pc.point_at(dc.first + (dc.second-dc.first)*k/ns);
                                poly.push_back({q[0], q[1]});
                            }
                        }
                        if (poly.size() < 3) continue;
                        bool inp = false;
                        for (size_t a = 0, b = poly.size()-1; a < poly.size(); b = a++) {
                            if (((poly[a][1] > v) != (poly[b][1] > v)) &&
                                (u < (poly[b][0]-poly[a][0])*(v-poly[a][1])/(poly[b][1]-poly[a][1]+1e-30)+poly[a][0]))
                                inp = !inp;
                        }
                        if (loop.type == BRepLoopType::Outer) { in_outer = inp; have_outer = true; }
                        else if (inp) return false;   // inside a hole
                    }
                    return have_outer ? in_outer : false;
                };
                auto exact_inside = [&](const BRep& X, const std::vector<double>& sg, const Point& p) -> bool {
                    double best_d = 1e300; int best_fi = -1; double bu = 0, bv = 0;
                    for (int fi = 0; fi < (int)X.m_faces.size(); ++fi) {
                        int si = X.m_faces[fi].surface_index;
                        if (si < 0 || si >= (int)X.m_surfaces.size()) continue;
                        auto [u, v, d] = Closest::surface_point(X.m_surfaces[si], p);
                        if (d >= best_d) continue;
                        if (!uv_in_trims(X, fi, u, v)) continue;
                        best_d = d; best_fi = fi; bu = u; bv = v;
                    }
                    if (best_fi < 0) return false;
                    int si = X.m_faces[best_fi].surface_index;
                    Vector n = X.m_surfaces[si].normal_at(bu, bv);
                    double s = (best_fi < (int)sg.size()) ? sg[best_fi] : 1.0;
                    Point bpt = X.m_surfaces[si].point_at(bu, bv);
                    double dp = (p[0]-bpt[0])*n[0]*s + (p[1]-bpt[1])*n[1]*s + (p[2]-bpt[2])*n[2]*s;
                    return dp < 0.0;   // p on the inner side of the outward normal
                };
                struct TP { const char* name; double x,y,z; const char* occt; };
                std::vector<TP> pts = {
                    {"f2", 10.0533,2.2465,3.2647, "OUT"},
                    {"f40", 9.4671,1.9436,-0.7861, "IN"},
                };
                std::printf("[Z90PROBE] B rotated: faces %d solid %d | face_meshes %zu signs %zu\n",
                            Bp.face_count(), Bp.is_solid()?1:0, fms.size(), fsg.size());
                for (auto& tp : pts) {
                    Point p(tp.x,tp.y,tp.z);
                    bool cur = Bp.contains_point(mb, p);
                    double om = omega_of(mb, p);
                    bool rp = ray_parity(mb, p);
                    double omo = oriented_omega(p);
                    bool ex = exact_inside(Bp, fsg, p);
                    std::printf("[Z90PROBE] %-4s occt=%-3s | winding=%d | rayparity=%d | oriented=%d | EXACT=%d\n",
                                tp.name, tp.occt, cur?1:0, rp?1:0,
                                std::abs(omo) > 6.2831853 ? 1 : 0, ex?1:0);
                    (void)om;
                }
                (void)mbq;
                return 0;
            }
            // FAST classification probe (no boolean): rotate B about the joint centroid
            // through the same config set and print volume() -- which orients every face
            // via face_outward_signs. A rotation-robust classifier keeps volume() equal to
            // the unrotated value (80.30) and positive for every config; a fragile one
            // flips sign or drifts. Seconds, not the 5-min scaffold marcher.
            if (std::getenv("SESSION_ROT_VOL")) {
                auto vmean0 = [](const BRep& X) {
                    Point c(0, 0, 0);
                    for (const auto& p : X.m_vertices) c = Point(c[0]+p[0], c[1]+p[1], c[2]+p[2]);
                    double n = (double)std::max<size_t>(1, X.m_vertices.size());
                    return Point(c[0]/n, c[1]/n, c[2]/n);
                };
                Point ca0 = vmean0(A), cb0 = vmean0(B);
                Point C0((ca0[0]+cb0[0])*0.5, (ca0[1]+cb0[1])*0.5, (ca0[2]+cb0[2])*0.5);
                struct RCV { const char* label; int ax; double deg; int ax2; double deg2; };
                std::vector<RCV> cv = {
                    {"id", -1, 0, -1, 0}, {"z15", 2, 15, -1, 0}, {"z30", 2, 30, -1, 0},
                    {"z45", 2, 45, -1, 0}, {"z90", 2, 90, -1, 0}, {"x20", 0, 20, -1, 0},
                    {"y30", 1, 30, -1, 0}, {"z30x20", 2, 30, 0, 20}, {"z37", 2, 37, -1, 0},
                    {"x13y29", 0, 13, 1, 29}, {"z63", 2, 63, -1, 0},
                };
                auto rot_ofv = [](int ax, double deg) {
                    return ax == 0 ? Xform::rotation_x(deg, true)
                         : ax == 1 ? Xform::rotation_y(deg, true)
                                   : Xform::rotation_z(deg, true);
                };
                // Cheap signed volume from the classifier's per-face signs and each face's
                // OWN tessellation flux (no masked Gauss): V = (1/3) sum fsign[f] * flux_nat[f].
                // A rotation-robust classifier makes this positive and stable near +80.30 for
                // every config; a fragile one flips it negative.
                auto probe = [](const BRep& X) {
                    std::vector<Point> P3; std::vector<Vector> Nn;
                    std::vector<double> fs = X.face_outward_signs(&P3, &Nn);
                    double V = 0.0; int nflip = 0, nf = (int)X.m_faces.size();
                    for (int fi = 0; fi < nf; ++fi) {
                        Mesh fm = X.subset(std::vector<int>{fi}).mesh();
                        double fl = 0.0, conv = 0.0, best = 1e300;
                        for (const auto& kv : fm.face) {
                            const auto& p = kv.second; if (p.size() < 3) continue;
                            Point a = fm.vertex.at(p[0]).position();
                            for (size_t k = 1; k + 1 < p.size(); ++k) {
                                Point b = fm.vertex.at(p[k]).position(), c = fm.vertex.at(p[k+1]).position();
                                double ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2],vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
                                double nx=(uy*vz-uz*vy)*.5,ny=(uz*vx-ux*vz)*.5,nz=(ux*vy-uy*vx)*.5;
                                double cx=(a[0]+b[0]+c[0])/3,cy=(a[1]+b[1]+c[1])/3,cz=(a[2]+b[2]+c[2])/3;
                                fl += (cx*nx+cy*ny+cz*nz)/3.0;
                                double dx=cx-P3[fi][0],dy=cy-P3[fi][1],dz=cz-P3[fi][2],d2=dx*dx+dy*dy+dz*dz;
                                if (d2<best){best=d2;conv=nx*Nn[fi][0]+ny*Nn[fi][1]+nz*Nn[fi][2];}
                            }
                        }
                        if (conv < 0) fl = -fl;         // flux in natural-normal orientation
                        V += fs[fi] * fl;               // fsign already = outward orientation
                        if (fs[fi] < 0) ++nflip;
                    }
                    return std::make_pair(V, nflip);
                };
                auto [va, na] = probe(A);
                std::printf("chairsVOL A          V %+8.4f flips %d\n", va, na);
                for (const auto& rc : cv) {
                    Xform R = (rc.ax < 0) ? Xform::identity() : rot_ofv(rc.ax, rc.deg);
                    if (rc.ax2 >= 0) R = R * rot_ofv(rc.ax2, rc.deg2);
                    Xform M = Xform::translation(C0[0], C0[1], C0[2]) * R
                            * Xform::translation(-C0[0], -C0[1], -C0[2]);
                    BRep Bv = B; Bv = Bv.transformed(M);
                    auto [vb, nb] = probe(Bv);
                    std::printf("chairsVOL B_%-7s V %+8.4f flips %d\n", rc.label, vb, nb);
                }
                return 0;
            }
            const char* opn[3] = {"cut", "common", "fuse"};
            double vols[3] = {0, 0, 0};
            const char* oponly_c = std::getenv("SESSION_OP");   // iterate one op cheaply
            const char* rot_only = std::getenv("SESSION_ROT_ONLY");  // one rot config, no base ops
            for (int m = 0; m < 3 && !rot_only; ++m) {
                if (oponly_c && std::string(opn[m]) != oponly_c) continue;
                try {
                    BRep r = (m == 0) ? A.boolean_difference(B)
                           : (m == 1) ? A.boolean_intersection(B)
                                      : A.boolean_union(B);
                    int nkb = 0;
                    for (const auto& Eb : r.m_topology_edges)
                        if ((int)Eb.trim_indices.size() == 1) ++nkb;
                    vols[m] = fast ? -1.0 : r.volume();
                    std::printf("chairs %-6s: faces %d solid %d naked %d vol %.4f\n",
                                opn[m], r.face_count(), r.is_solid() ? 1 : 0, nkb, vols[m]);
                    // Corner accuracy audit: a good corner vertex lies ON every adjacent
                    // face's surface. Junction welds carry chord-lerp sag (weld_tol band),
                    // so corners can sit visibly off the true 3-surface intersection.
                    if (std::getenv("SESSION_CORNER_AUDIT")) {
                        double worst = 0; int nbad = 0, nv = 0;
                        std::vector<std::pair<double,int>> rows;
                        for (int vi = 0; vi < (int)r.m_topology_vertices.size(); ++vi) {
                            const auto& tv = r.m_topology_vertices[vi];
                            if (tv.point_index < 0 || (int)tv.edge_indices.size() < 3) continue;
                            const Point& P = r.m_vertices[tv.point_index];
                            std::set<int> sids;
                            for (int ei : tv.edge_indices) {
                                if (ei < 0 || ei >= (int)r.m_topology_edges.size()) continue;
                                for (int ti : r.m_topology_edges[ei].trim_indices) {
                                    int li = r.m_trims[ti].loop_index;
                                    if (li < 0) continue;
                                    int fi2 = r.m_loops[li].face_index;
                                    if (fi2 >= 0) sids.insert(r.m_faces[fi2].surface_index);
                                }
                            }
                            double dmax = 0;
                            for (int si2 : sids) {
                                auto [cu2, cv3, cd2] = Closest::surface_point(r.m_surfaces[si2], P);
                                (void)cu2; (void)cv3;
                                dmax = std::max(dmax, cd2);
                            }
                            ++nv;
                            worst = std::max(worst, dmax);
                            if (dmax > 1e-3) ++nbad;
                            rows.push_back({dmax, vi});
                        }
                        std::sort(rows.rbegin(), rows.rend());
                        std::printf("[CORNER] %s verts>=3edges: %d worst=%.4f bad(>1e-3): %d\n",
                                    opn[m], nv, worst, nbad);
                        for (int k = 0; k < 5 && k < (int)rows.size(); ++k) {
                            const Point& P = r.m_vertices[r.m_topology_vertices[rows[k].second].point_index];
                            std::printf("[CORNER]   v%d d=%.4f p(%.3f,%.3f,%.3f)\n",
                                        rows[k].second, rows[k].first, P[0], P[1], P[2]);
                        }
                    }
                    // Rhino-acceptance audit: max 3D gap between consecutive trims'
                    // pcurve-image endpoints per loop. Rhino re-projects trims and drops
                    // wires whose corners exceed its tolerance -- OCCT heals them, so
                    // step_probe VALID does NOT imply Rhino closes the solid.
                    if (std::getenv("SESSION_WIREGAP")) {
                        double gmaxall = 0; int nloops_bad = 0;
                        for (const auto& lp : r.m_loops) {
                            double gmax = 0;
                            int nt2 = (int)lp.trim_indices.size();
                            for (int t2 = 0; t2 < nt2; ++t2) {
                                const auto& tr0 = r.m_trims[lp.trim_indices[t2]];
                                const auto& tr1 = r.m_trims[lp.trim_indices[(t2+1)%nt2]];
                                if (tr0.curve_2d_index < 0 || tr1.curve_2d_index < 0) continue;
                                int si2 = r.m_faces[lp.face_index].surface_index;
                                const NurbsSurface& S2 = r.m_surfaces[si2];
                                const NurbsCurve& c0 = r.m_curves_2d[tr0.curve_2d_index];
                                const NurbsCurve& c1 = r.m_curves_2d[tr1.curve_2d_index];
                                auto d0 = c0.domain(); auto d1 = c1.domain();
                                Point e0 = c0.point_at(tr0.reversed ? d0.first : d0.second);
                                Point s1 = c1.point_at(tr1.reversed ? d1.second : d1.first);
                                gmax = std::max(gmax, S2.point_at(e0[0], e0[1]).distance(S2.point_at(s1[0], s1[1])));
                            }
                            gmaxall = std::max(gmaxall, gmax);
                            if (gmax > 1e-4) ++nloops_bad;
                        }
                        std::printf("[WIREGAP] %s max=%.3e loops>1e-4: %d/%zu\n",
                                    opn[m], gmaxall, nloops_bad, r.m_loops.size());
                    }
                    if (std::getenv("SESSION_CHAIRS_DUMP"))
                        r.pb_dump(std::string(cd) + "/chair_" + opn[m] + ".pb");
                    r.surfacecolor = Color(0.35f, 0.75f, 0.40f);
                    std::vector<const BRep*> parts = {&A, &B};
                    if (r.face_count() > 0) parts.push_back(&r);
                    std::string nm = std::string("chair0_") + opn[m] + "_chair1";
                    if (!fast)
                        file_step::write_file_step_breps(parts, nm, std::string(cd) + "/" + nm + ".step");
                } catch (const std::exception& e) { std::printf("chairs %s THREW: %s\n", opn[m], e.what()); }
            }
            // Rotated-chair robustness battery: rotate B about the JOINT CENTROID through
            // a config set and run all ops -- a professional kernel must close every one.
            // Writes each rotated B to chairs/rot/ so step_probe can produce OCCT
            // references: step_probe --cut chair0.stp rot/B_<cfg>.step
            if (std::getenv("SESSION_CHAIRS_ROT")) {
                auto vmean = [](const BRep& X) {
                    Point c(0, 0, 0);
                    for (const auto& p : X.m_vertices) c = Point(c[0]+p[0], c[1]+p[1], c[2]+p[2]);
                    double n = (double)std::max<size_t>(1, X.m_vertices.size());
                    return Point(c[0]/n, c[1]/n, c[2]/n);
                };
                Point ca = vmean(A), cb2 = vmean(B);
                Point C((ca[0]+cb2[0])*0.5, (ca[1]+cb2[1])*0.5, (ca[2]+cb2[2])*0.5);
                struct RC { const char* label; int ax; double deg; int ax2; double deg2; };
                // Ten deterministic orientations about the joint centroid: single-axis at
                // several magnitudes, two compound (Euler) cases, and three odd angles that
                // land nothing on a symmetry (z37/x13y29/z63) so no cell can pass by luck.
                std::vector<RC> cfgs = {
                    {"z15", 2, 15, -1, 0}, {"z30", 2, 30, -1, 0}, {"z45", 2, 45, -1, 0},
                    {"z90", 2, 90, -1, 0}, {"x20", 0, 20, -1, 0}, {"y30", 1, 30, -1, 0},
                    {"z30x20", 2, 30, 0, 20},
                    {"z37", 2, 37, -1, 0}, {"x13y29", 0, 13, 1, 29}, {"z63", 2, 63, -1, 0},
                };
                std::string rotdir = std::string(cd) + "/rot";
                std::filesystem::create_directories(rotdir);
                auto rot_of = [](int ax, double deg) {
                    return ax == 0 ? Xform::rotation_x(deg, true)
                         : ax == 1 ? Xform::rotation_y(deg, true)
                                   : Xform::rotation_z(deg, true);
                };
                for (const auto& rc : cfgs) {
                    if (rot_only && std::string(rc.label) != rot_only) continue;
                    Xform R = rot_of(rc.ax, rc.deg);
                    if (rc.ax2 >= 0) R = R * rot_of(rc.ax2, rc.deg2);
                    Xform M = Xform::translation(C[0], C[1], C[2]) * R
                            * Xform::translation(-C[0], -C[1], -C[2]);
                    BRep Brot = B;
                    Brot = Brot.transformed(M);
                    if (!fast)
                        file_step::write_file_step_brep(Brot, rotdir + "/B_" + rc.label + ".step");
                    // PARTITION IDENTITY (oracle-free): vol(cut) + vol(common) == vol(A).
                    // The one self-validation that catches a CLOSED-but-WRONG result without
                    // any reference -- y30's cut of 23.9619 against vol(A) 80.30 demands a
                    // common of 56.34, which nothing produces. It is cross-op, so it is
                    // unavailable to a single result in isolation; that is exactly why every
                    // single-result invariant we own (naked, nonmanifold, shells, bbox,
                    // per-op volume bound) passes y30's wrong answer.
                    double rot_vol[3] = {0, 0, 0};
                    bool rot_closed[3] = {false, false, false};
                    for (int m2 = 0; m2 < 3; ++m2) {
                        const char* oponly2 = std::getenv("SESSION_OP");
                        if (oponly2 && std::string(opn[m2]) != oponly2) continue;
                        try {
                            BRep r2 = (m2 == 0) ? A.boolean_difference(Brot)
                                    : (m2 == 1) ? A.boolean_intersection(Brot)
                                                : A.boolean_union(Brot);
                            int nk2 = 0, nm2 = 0;
                            for (const auto& E2 : r2.m_topology_edges) {
                                int nt = (int)E2.trim_indices.size();
                                if (nt == 1) ++nk2;
                                // NON-MANIFOLD (>2 faces on one edge) = the MEASURED cause of the
                                // "naked 0 but the exported result is open" contradiction:
                                // res_y30_cut.step has 268 EDGE_CURVEs, ZERO referenced once
                                // (nothing is unshared -- the kernel's edge sharing is faithful)
                                // but THIRTEEN referenced MORE than twice. OCCT cannot sew a
                                // manifold shell at a 3-face edge, so it splits there -> open
                                // shells + 58 import-naked edges. Invisible to the naked count
                                // (1-trim only) AND to is_solid() (continues on nt==2, ignores >2).
                                else if (nt > 2) ++nm2;
                            }
                            // shells: is_solid() cannot see a boundary that has come APART into
                            // several separately-closed shells (independent OCCT read found 2-4
                            // shells on every rotated config but z30x20). Report it always.
                            // VOLUME IS UNDEFINED ON AN OPEN BOUNDARY. volume() applies the
                            // divergence theorem, which requires a closed surface; on a shell
                            // with naked edges the number is meaningless and has been actively
                            // misleading (y30 cut printed 23.9619 vs reference 46.9596, x13y29
                            // 68.0078 vs 48.4734 -- both were open shells, so neither number was
                            // a wrong volume, it was a non-volume). Print it ONLY when closed.
                            // A result is only a candidate solid when naked==0 AND nonmanifold==0.
                            bool closed2 = (nk2 == 0 && nm2 == 0);
                            if (fast)
                                std::printf("chairsROT %-7s %-6s: faces %d solid %d shells %d naked %d nonmani %d\n",
                                            rc.label, opn[m2], r2.face_count(),
                                            r2.is_solid() ? 1 : 0, shell_count_of(r2), nk2, nm2);
                            else if (closed2) {
                                double v2 = r2.volume();
                                rot_vol[m2] = v2; rot_closed[m2] = true;
                                std::printf("chairsROT %-7s %-6s: faces %d solid %d shells %d naked %d nonmani %d vol %.4f\n",
                                            rc.label, opn[m2], r2.face_count(),
                                            r2.is_solid() ? 1 : 0, shell_count_of(r2), nk2, nm2, v2);
                            }
                            else
                                std::printf("chairsROT %-7s %-6s: faces %d solid %d shells %d naked %d nonmani %d vol UNDEFINED(open/nonmanifold)\n",
                                            rc.label, opn[m2], r2.face_count(),
                                            r2.is_solid() ? 1 : 0, shell_count_of(r2), nk2, nm2);
                            if (std::getenv("SESSION_TOPO_CHECK"))
                                std::printf("   %s\n", r2.topology_report().c_str());
                            // INSPECTION FILE (user-facing): one file per cell containing the
                            // OPERANDS AND THE RESULT, colour-coded exactly like the base path
                            // (A red, rotated B blue, result green) -- a result-only file gives
                            // the user no way to see what was cut from what. When the op yields
                            // nothing, still write A+B so the inputs are visible and the empty
                            // result is evident from the file rather than from a missing file.
                            if (std::getenv("SESSION_ROT_STEP")) {
                                BRep rc2 = r2;
                                rc2.surfacecolor = Color(0.35f, 0.75f, 0.40f);
                                std::vector<const BRep*> parts = {&A, &Brot};
                                if (rc2.face_count() > 0) parts.push_back(&rc2);
                                std::string path = rotdir + "/res_" + rc.label + "_" + opn[m2] + ".step";
                                file_step::write_file_step_breps(parts, "res_" + std::string(rc.label)
                                                                 + "_" + opn[m2], path);
                                if (rc2.face_count() == 0)
                                    std::printf("   [ROTSTEP] %s %s: EMPTY result -- wrote operands only\n",
                                                rc.label, opn[m2]);
                            }
                        } catch (const std::exception& e2) {
                            std::printf("chairsROT %-7s %-6s: THREW %s\n", rc.label, opn[m2], e2.what());
                        } catch (...) {
                            std::printf("chairsROT %-7s %-6s: THREW\n", rc.label, opn[m2]);
                        }
                    }
                    // Partition identity, reported whenever BOTH cut and common are closed
                    // (needs no oracle -- see the note above the op loop).
                    if (rot_closed[0] && rot_closed[1]) {
                        double va2 = A.volume();
                        double sum = rot_vol[0] + rot_vol[1];
                        double rel = std::abs(sum - va2) / std::max(1.0, std::abs(va2));
                        std::printf("chairsROT %-7s [PARTITION] cut %.4f + common %.4f = %.4f vs A %.4f  rel %.2e  %s\n",
                                    rc.label, rot_vol[0], rot_vol[1], sum, va2, rel,
                                    rel < 1e-3 ? "OK" : "** VIOLATION **");
                    }
                }
            }
            // Randomized-rotation validation battery: SESSION_ROT_RANDOM=N rotates B about the
            // joint centroid through N deterministic pseudo-random orientations (alternating
            // axis-angle and Euler), writes each B to chairs/rnd/B_rNNN.step for the OCCT oracle
            // (step_probe --cut chair0.stp rnd/B_rNNN.step), and reports faces/solid/naked/vol.
            // Seeded (SESSION_ROT_SEED, default fixed) so the suite is fully reproducible.
            if (const char* rn = std::getenv("SESSION_ROT_RANDOM")) {
                int N = std::atoi(rn); if (N < 1) N = 1;
                unsigned long long st = 0x9E3779B97F4A7C15ULL;
                if (const char* sd = std::getenv("SESSION_ROT_SEED"))
                    st ^= (unsigned long long)std::atoll(sd) * 0x100000001B3ULL + 1ULL;
                auto u01 = [&st]() {
                    st = st * 6364136223846793005ULL + 1442695040888963407ULL;
                    return (double)((st >> 11) & ((1ULL << 53) - 1)) / (double)(1ULL << 53);
                };
                auto vmeanR = [](const BRep& X) {
                    Point c(0, 0, 0);
                    for (const auto& p : X.m_vertices) c = Point(c[0]+p[0], c[1]+p[1], c[2]+p[2]);
                    double n = (double)std::max<size_t>(1, X.m_vertices.size());
                    return Point(c[0]/n, c[1]/n, c[2]/n);
                };
                Point caR = vmeanR(A), cbR = vmeanR(B);
                Point C((caR[0]+cbR[0])*0.5, (caR[1]+cbR[1])*0.5, (caR[2]+cbR[2])*0.5);
                std::string rnddir = std::string(cd) + "/rnd";
                std::filesystem::create_directories(rnddir);
                const char* opn2[3] = {"cut", "common", "fuse"};
                const char* oponlyR = std::getenv("SESSION_OP");
                for (int k = 0; k < N; ++k) {
                    Xform M;
                    if (k % 2 == 0) {
                        double ax, ay, az, m;
                        do { ax = u01()*2-1; ay = u01()*2-1; az = u01()*2-1;
                             m = std::sqrt(ax*ax+ay*ay+az*az); } while (m < 1e-3);
                        ax/=m; ay/=m; az/=m;
                        double ang = 10.0 + u01()*80.0;
                        Line axis(C[0], C[1], C[2], C[0]+ax, C[1]+ay, C[2]+az);
                        M = Xform::rotation_around_line(axis, ang, true);
                    } else {
                        double ex = u01()*120-60, ey = u01()*120-60, ez = u01()*120-60;
                        Xform R = Xform::rotation_z(ez, true) * Xform::rotation_y(ey, true)
                                * Xform::rotation_x(ex, true);
                        M = Xform::translation(C[0], C[1], C[2]) * R
                          * Xform::translation(-C[0], -C[1], -C[2]);
                    }
                    char lab[16]; std::snprintf(lab, sizeof lab, "r%03d", k);
                    BRep Brot = B; Brot = Brot.transformed(M);
                    if (!fast)
                        file_step::write_file_step_brep(Brot, rnddir + "/B_" + lab + ".step");
                    for (int m2 = 0; m2 < 3; ++m2) {
                        if (oponlyR && std::string(opn2[m2]) != oponlyR) continue;
                        try {
                            BRep r2 = (m2 == 0) ? A.boolean_difference(Brot)
                                    : (m2 == 1) ? A.boolean_intersection(Brot)
                                                : A.boolean_union(Brot);
                            int nk2 = 0;
                            for (const auto& E2 : r2.m_topology_edges)
                                if ((int)E2.trim_indices.size() == 1) ++nk2;
                            std::printf("chairsRND %-5s %-6s: faces %d solid %d naked %d vol %.4f\n",
                                        lab, opn2[m2], r2.face_count(),
                                        r2.is_solid() ? 1 : 0, nk2, fast ? -1.0 : r2.volume());
                            if (std::getenv("SESSION_TOPO_CHECK"))
                                std::printf("   %s\n", r2.topology_report().c_str());
                            // same inspection format as the deterministic battery: operands +
                            // result in one colour-coded file (A red, rotated B blue, result green)
                            if (std::getenv("SESSION_ROT_STEP")) {
                                BRep rc2 = r2;
                                rc2.surfacecolor = Color(0.35f, 0.75f, 0.40f);
                                std::vector<const BRep*> parts = {&A, &Brot};
                                if (rc2.face_count() > 0) parts.push_back(&rc2);
                                std::string nm = "res_" + std::string(lab) + "_" + opn2[m2];
                                file_step::write_file_step_breps(parts, nm, rnddir + "/" + nm + ".step");
                                if (rc2.face_count() == 0)
                                    std::printf("   [ROTSTEP] %s %s: EMPTY result -- wrote operands only\n",
                                                lab, opn2[m2]);
                            }
                        } catch (const std::exception& e2) {
                            std::printf("chairsRND %-5s %-6s: THREW %s\n", lab, opn2[m2], e2.what());
                        } catch (...) {
                            std::printf("chairsRND %-5s %-6s: THREW\n", lab, opn2[m2]);
                        }
                    }
                }
            }
            // identities: cut + common = A ; fuse = A + B - common
            double va = A.volume(), vb = B.volume();
            std::printf("chairs [id1] cut+common-A    rel %9.2e\n",
                        std::abs(vols[0] + vols[1] - va) / std::max(1.0, std::abs(va)));
            std::printf("chairs [id2] fuse-(A+B-com)  rel %9.2e\n",
                        std::abs(vols[2] - (va + vb - vols[1])) / std::max(1.0, std::abs(va)));
        }
    }
    if (const char* sd = std::getenv("SESSION_STEP_PRIMS")) {
        std::filesystem::create_directories(sd);
        file_step::write_file_step_brep(BRep::create_box(4, 4, 4), std::string(sd) + "/prim_box.step");
        file_step::write_file_step_brep(BRep::create_sphere(2.5), std::string(sd) + "/prim_sphere.step");
        file_step::write_file_step_brep(BRep::create_cylinder(1.5, 6), std::string(sd) + "/prim_cylinder.step");
        file_step::write_file_step_brep(BRep::create_cone(2.0, 4.0), std::string(sd) + "/prim_cone.step");
        file_step::write_file_step_brep(BRep::create_torus(2.0, 0.8), std::string(sd) + "/prim_torus.step");
        std::printf("primitive STEP files written to %s\n", sd);
    }
    if (std::getenv("SESSION_CAVITY")) {
        BRep bx = BRep::create_box(4, 4, 4);
        BRep sp2 = BRep::create_sphere(1.5);
        BRep ct = bx.boolean_difference(sp2);
        double ref = 64.0 - (4.0/3.0)*3.14159265358979323846*1.5*1.5*1.5;
        std::printf("cavity cut vol %.6f (ref %.6f, rel %9.2e) faces %d solid %d\n",
                    ct.volume(), ref, std::abs(ct.volume()-ref)/ref, ct.face_count(), ct.is_solid()?1:0);
    }
    // OCCT test-suite battery (SESSION_OCCT_SUITE): 120 cells harvested from OCCT's own
    // tests/boolean + bugs/modalg scripts (occt_suite.h, OCCT_SUITE_NOTES.md). One op per
    // cell as in the source script; reference via the same cached OCCT oracle.
    if (std::getenv("SESSION_OCCT_SUITE")) {
        int sfails = 0, stotal = 0;
        auto mk_place = [](const char* kind, const double* p, const double* xf) {
            Place pl;
            pl.kind = kind;
            int np = std::string(kind) == "box" ? 3 : std::string(kind) == "sphere" ? 1 : 2;
            pl.p.assign(p, p + np);
            for (int i = 0; i < 7; ++i) pl.xf[i] = xf[i];
            return pl;
        };
        for (const auto& c : OCCT_SUITE) {
            if (!filter.empty() && std::string(c.label).find(filter) == std::string::npos) continue;
            ++stotal;
            Place A = mk_place(c.kindA, c.pA, c.xfA);
            Place B = mk_place(c.kindB, c.pB, c.xfB);
            double v = 0; int nf = 0, solid = 0; long us = 0;
            try {
                BRep ba = build(A), bb = build(B);
                auto t0 = std::chrono::steady_clock::now();
                BRep r = std::string(c.op) == "cut"    ? ba.boolean_difference(bb)
                       : std::string(c.op) == "common" ? ba.boolean_intersection(bb)
                                                       : ba.boolean_union(bb);
                us = (long)std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0).count();
                v = r.volume(); nf = r.face_count(); solid = r.is_solid() ? 1 : 0;
            } catch (const std::exception& e) {
                std::printf("%-14s %-6s | THREW: %s\n", c.label, c.op, e.what()); ++sfails; continue;
            } catch (...) { std::printf("%-14s %-6s | THREW\n", c.label, c.op); ++sfails; continue; }
            std::string key = std::string("OS|") + c.label + "|" + c.op;
            auto k = occt(key, c.op, A, B, have_oracle, oracle, req, res, cache, refresh, dirty);
            if (k[2] == 0.0) {
                std::printf("%-14s %-6s | %11.4f %11s %9s | %4d %5s | %d | %8ld | %s\n",
                            c.label, c.op, v, "-", "-", nf, "-", solid, us,
                            have_oracle ? "OCCT-FAIL" : "no-ref");
                ++sfails; continue;
            }
            double rel = k[0] != 0 ? std::abs(v - k[0]) / std::abs(k[0]) : std::abs(v - k[0]);
            bool both_empty = std::abs(k[0]) < 1e-12 && (int)k[1] == 0 && std::abs(v) < 1e-12 && nf == 0;
            bool ok = both_empty || (rel < 1e-6 && nf == (int)k[1] && solid);
            if (!ok) ++sfails;
            std::printf("%-14s %-6s | %11.4f %11.4f %9.2e | %4d %5d | %d | %8ld | %s  (%s)\n",
                        c.label, c.op, v, k[0], rel, nf, (int)k[1], solid, us, ok ? "OK" : "FAIL", c.src);
        }
        std::printf("OCCT-SUITE: %d/%d cells OK\n", stotal - sfails, stotal);
    }
    if (dirty) save_cache(cachePath, cache);
    std::printf("\n%d/%d cells OK (vol rel<1e-6 AND exact faces AND is_solid)\n", total - fails, total);
    return 0;
}
