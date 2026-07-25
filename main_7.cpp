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
#include "closest.h"
#include "intersection.h"
#include "file_step.h"
#include "xform.h"
#include "tolerance.h"
#include "occt_suite.h"
using namespace session_cpp;

namespace {

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
    b.xform = xf_of(pl.xf);
    return b.transformed();
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
            if (const char* sd = std::getenv("SESSION_STEP_DIR"))
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
                if (m == 0) file_step::write_file_step_brep(ff, std::string(sd) + "/freeform_blob.step");
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
                BRep Bp = B; Bp.xform = M; Bp = Bp.transformed();
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
                    BRep Bv = B; Bv.xform = M; Bv = Bv.transformed();
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
                    Brot.xform = M;
                    Brot = Brot.transformed();
                    if (!fast)
                        file_step::write_file_step_brep(Brot, rotdir + "/B_" + rc.label + ".step");
                    for (int m2 = 0; m2 < 3; ++m2) {
                        const char* oponly2 = std::getenv("SESSION_OP");
                        if (oponly2 && std::string(opn[m2]) != oponly2) continue;
                        try {
                            BRep r2 = (m2 == 0) ? A.boolean_difference(Brot)
                                    : (m2 == 1) ? A.boolean_intersection(Brot)
                                                : A.boolean_union(Brot);
                            int nk2 = 0;
                            for (const auto& E2 : r2.m_topology_edges)
                                if ((int)E2.trim_indices.size() == 1) ++nk2;
                            std::printf("chairsROT %-7s %-6s: faces %d solid %d naked %d vol %.4f\n",
                                        rc.label, opn[m2], r2.face_count(),
                                        r2.is_solid() ? 1 : 0, nk2, fast ? -1.0 : r2.volume());
                            if (std::getenv("SESSION_TOPO_CHECK"))
                                std::printf("   %s\n", r2.topology_report().c_str());
                            if (std::getenv("SESSION_ROT_STEP") && r2.face_count() > 0)
                                file_step::write_file_step_brep(r2, rotdir + "/res_"
                                    + rc.label + "_" + opn[m2] + ".step");
                        } catch (const std::exception& e2) {
                            std::printf("chairsROT %-7s %-6s: THREW %s\n", rc.label, opn[m2], e2.what());
                        } catch (...) {
                            std::printf("chairsROT %-7s %-6s: THREW\n", rc.label, opn[m2]);
                        }
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
                    BRep Brot = B; Brot.xform = M; Brot = Brot.transformed();
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
                            if (std::getenv("SESSION_ROT_STEP") && r2.face_count() > 0)
                                file_step::write_file_step_brep(r2, rnddir + "/res_"
                                    + std::string(lab) + "_" + opn2[m2] + ".step");
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
