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
#include "brep.h"
#include "intersection.h"
#include "file_step.h"
#include "xform.h"
#include "tolerance.h"
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
            std::printf("A: faces %d solid %d vol %.4f | B: faces %d solid %d vol %.4f\n",
                        A.face_count(), A.is_solid() ? 1 : 0, A.volume(),
                        B.face_count(), B.is_solid() ? 1 : 0, B.volume());
            A.surfacecolor = Color(0.85f, 0.25f, 0.20f);
            B.surfacecolor = Color(0.20f, 0.45f, 0.85f);
            const char* opn[3] = {"cut", "common", "fuse"};
            double vols[3] = {0, 0, 0};
            for (int m = 0; m < 3; ++m) {
                try {
                    BRep r = (m == 0) ? A.boolean_difference(B)
                           : (m == 1) ? A.boolean_intersection(B)
                                      : A.boolean_union(B);
                    vols[m] = r.volume();
                    std::printf("chairs %-6s: faces %d solid %d vol %.4f\n",
                                opn[m], r.face_count(), r.is_solid() ? 1 : 0, vols[m]);
                    if (std::getenv("SESSION_CHAIRS_DUMP"))
                        r.pb_dump(std::string(cd) + "/chair_" + opn[m] + ".pb");
                    r.surfacecolor = Color(0.35f, 0.75f, 0.40f);
                    std::vector<const BRep*> parts = {&A, &B};
                    if (r.face_count() > 0) parts.push_back(&r);
                    std::string nm = std::string("chair0_") + opn[m] + "_chair1";
                    file_step::write_file_step_breps(parts, nm, std::string(cd) + "/" + nm + ".step");
                } catch (const std::exception& e) { std::printf("chairs %s THREW: %s\n", opn[m], e.what()); }
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
    if (dirty) save_cache(cachePath, cache);
    std::printf("\n%d/%d cells OK (vol rel<1e-6 AND exact faces AND is_solid)\n", total - fails, total);
    return 0;
}
