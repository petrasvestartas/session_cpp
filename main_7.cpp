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
        {"cone x cyl ", "cone", "cyl"},  {"cone x tor ", "cone", "tor"},
        {"cyl  x cyl ", "cyl",  "cyl2"}, {"cyl  x tor ", "cyl",  "tor"},
        {"tor  x tor ", "tor",  "tor2"},
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
    std::printf("%-13s %-4s | %11s %11s %9s | %4s %5s | s | %8s | verdict\n",
                "pair", "op", "our_vol", "occt_vol", "rel", "ourF", "occtF", "us");
    int fails = 0, total = 0;
    for (auto& pr : pairs()) {
        if (!filter.empty() && pr[0].find(filter) == std::string::npos) continue;
        for (const char* mode : {"cut", "common", "fuse"}) {
            ++total;
            const Place& A = PL[pr[1]]; const Place& B = PL[pr[2]];
            double v = 0; int nf = 0; int solid = 0; long us = 0;
            try {
                BRep ba = build(A), bb = build(B);
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
    }
    if (dirty) save_cache(cachePath, cache);
    std::printf("\n%d/%d cells OK (vol rel<1e-6 AND exact faces AND is_solid)\n", total - fails, total);
    return 0;
}
