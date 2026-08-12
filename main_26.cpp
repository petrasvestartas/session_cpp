// main_26 — G3 gate: v3 booleans on STEP-imported analytic primitives
// (kb/BOOL_V3_MEMORY.md, remaining-work item 2).
//
// Loads occt_prim_{box,cone,cylinder,sphere,torus}.step from
// serialization/step_import/ (truth volumes in truth.txt), then booleans every
// pair (each primitive vs itself and vs each other primitive) under 4 seeded
// random rigid poses of B, all 3 ops. Gates per case:
//   - all three results closed (BRep::is_solid: 0 naked, 0 nonmanifold)
//   - partition identity  vol(cut)+vol(common) == vol(A)         (2% rel)
//   - fuse identity       vol(fuse) == vol(A)+vol(B)-vol(common) (2% rel)
//   - vol(A), vol(B) vs truth.txt <= 0.5%
// Exit 0 iff every check passes.
//
// Usage: main_26 [pair-filter]     (substring match, e.g. "box_x_cyl")
// Env:   V3STEPDIR=<dir>           override the step_import directory.
#include "v3_geom.h"
#include "v3_topo.h"
#include "v3_bool.h"
#include "brep.h"
#include "file_step.h"
#include "xform.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace session_cpp;
using namespace v3;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::printf("FAIL [%s:%d] ", __FILE__, __LINE__); \
           std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static std::mt19937 rng(20260803);

// Same contract as main_25: axis_rotation does NOT normalize its axis.
static Xform random_rotation() {
    std::uniform_real_distribution<double> U(0, 1);
    V3 axis{U(rng) - 0.5, U(rng) - 0.5, U(rng) - 0.5};
    axis = axis.normalized();
    Vector ax(axis.x, axis.y, axis.z);
    double ang = U(rng) * 2.0 * PI;
    return Xform::axis_rotation(ang, ax, false);
}

// ---- truth table --------------------------------------------------------------
static std::map<std::string, double> load_truth(const std::string& dir) {
    std::map<std::string, double> truth;
    std::ifstream in(dir + "/truth.txt");
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> cols;
        size_t p0 = 0;
        for (;;) {
            size_t t = line.find('\t', p0);
            cols.push_back(line.substr(p0, t == std::string::npos ? t : t - p0));
            if (t == std::string::npos) break;
            p0 = t + 1;
        }
        if (cols.size() >= 4) truth[cols[0]] = std::stod(cols[3]);
    }
    return truth;
}

// ---- pose generation ------------------------------------------------------------
// Rotate the copy about its own bbox center (mesh vertices), then translate by a
// random vector of magnitude <= 0.5 * cbrt(vol(B)) so the pair (almost) always
// still intersects.
static void pose_of(const BRep& b, double vol_b, Xform& xf) {
    auto mesh = b.mesh();
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (const auto& kv : mesh.vertex) {
        double q[3] = {kv.second.x, kv.second.y, kv.second.z};
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], q[k]);
            hi[k] = std::max(hi[k], q[k]);
        }
    }
    double c[3] = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    std::uniform_real_distribution<double> U(0, 1);
    double d[3] = {U(rng) - 0.5, U(rng) - 0.5, U(rng) - 0.5};
    double dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    double mag = (dn > 1e-12 ? U(rng) * 0.5 * std::cbrt(vol_b) / dn : 0.0);
    for (int k = 0; k < 3; k++) d[k] *= mag;
    // column-major: apply rightmost first -> T(c+d) * R * T(-c)
    xf = Xform::translation(c[0] + d[0], c[1] + d[1], c[2] + d[2]) * random_rotation() *
         Xform::translation(-c[0], -c[1], -c[2]);
}

// ---- one pair x one pose --------------------------------------------------------
static void run_case(const BRep& a, const BRep& b0, double va, double vb,
                     const char* name) {
    // Per-case RNG (seeded from the case name) so a case reproduces standalone
    // under the pair-filter regardless of which cases ran before it.
    rng.seed(20260803u ^ (unsigned)std::hash<std::string>{}(name));
    std::printf("  [case] %s\n", name);
    std::fflush(stdout);
    for (int attempt = 0; attempt < 4; attempt++) {
        BRep b = b0;
        Xform xf;
        pose_of(b, vb, xf);
        b.transform(xf);
        v3::Solid sa = v3::from_brep(a), sb = v3::from_brep(b);
        bool ixc = false, ixm = false, ixf = false;
        v3::Solid cut = v3::boolean(sa, sb, v3::BoolOp::CUT, 1e-6, &ixc);
        v3::Solid com = v3::boolean(sa, sb, v3::BoolOp::COMMON, 1e-6, &ixm);
        v3::Solid fus = v3::boolean(sa, sb, v3::BoolOp::FUSE, 1e-6, &ixf);
        if (!(ixc && ixm && ixf)) {
            if (attempt < 3) continue; // pose drifted apart; try another
            CHECK(false, "%s: operands not intersecting (4 poses tried)", name);
            return;
        }
        BRep bc = v3::to_brep(cut), bm = v3::to_brep(com), bf = v3::to_brep(fus);
        double vc = bc.volume(), vm = bm.volume(), vf = bf.volume();
        bool cc = bc.is_solid(), cm = bm.is_solid(), cf = bf.is_solid();
        double part = vc + vm - va;
        double fid = vf - (va + vb - vm);
        std::printf("  %-18s cut %8.4f(%d) com %8.4f(%d) fus %8.4f(%d)"
                    "  part %+8.4f  fuse %+8.4f\n",
                    name, vc, (int)cc, vm, (int)cm, vf, (int)cf, part, fid);
        CHECK(cc, "%s: cut not solid", name);
        CHECK(cm, "%s: common not solid", name);
        CHECK(cf, "%s: fuse not solid", name);
        CHECK(std::abs(part) < 0.02 * va,
              "%s: partition cut(%.4f)+common(%.4f)-A(%.4f) = %.4f", name, vc, vm,
              va, part);
        CHECK(std::abs(fid) < 0.02 * (va + vb), "%s: fuse identity %.4f", name, fid);
        return;
    }
}

int main(int argc, char** argv) {
    const char* only = argc > 1 ? argv[1] : nullptr;
    std::string dir;
    if (const char* e = std::getenv("V3STEPDIR")) dir = e;
    else if (std::filesystem::exists("../serialization/step_import/truth.txt"))
        dir = "../serialization/step_import";
    else dir = "serialization/step_import";

    std::map<std::string, double> truth = load_truth(dir);
    CHECK(!truth.empty(), "truth.txt not found in %s", dir.c_str());
    if (truth.empty()) return 1;

    struct Prim { const char* file; const char* name; };
    static const Prim PRIMS[] = {
        {"occt_prim_box.step", "box"},
        {"occt_prim_cone.step", "cone"},
        {"occt_prim_cylinder.step", "cyl"},
        {"occt_prim_sphere.step", "sph"},
        {"occt_prim_torus.step", "torus"},
    };

    std::printf("[G3-step] import gate (volume vs truth.txt, <= 0.5%%)\n");
    std::vector<BRep> breps;
    std::vector<double> vols;
    for (const auto& p : PRIMS) {
        std::vector<BRep> bs;
        try { bs = file_step::read_file_step_breps(dir + "/" + p.file); }
        catch (const std::exception& e) {
            CHECK(false, "%s: STEP read threw: %s", p.file, e.what());
        }
        CHECK(bs.size() == 1, "%s: expected 1 brep, got %zu", p.file, bs.size());
        if (bs.size() != 1) { breps.emplace_back(); vols.push_back(0); continue; }
        auto it = truth.find(p.file);
        CHECK(it != truth.end(), "%s: no truth entry", p.file);
        double vt = it != truth.end() ? it->second : 0;
        double va = bs[0].volume();
        bool solid = bs[0].is_solid();
        double rel = vt > 0 ? std::abs(va - vt) / vt : 1e9;
        std::printf("  %-22s faces %2d  vol %9.4f vs truth %9.4f  (%6.3f%%)  solid %d\n",
                    p.name, bs[0].face_count(), va, vt, rel * 100.0, (int)solid);
        CHECK(solid, "%s: imported brep not solid", p.name);
        CHECK(rel <= 0.005, "%s: vol %.4f vs truth %.4f (%.3f%%)", p.name, va, vt,
              rel * 100.0);
        breps.push_back(bs[0]);
        vols.push_back(va);
    }

    std::printf("[G3-step] pairwise booleans x 4 poses x 3 ops\n");
    for (size_t i = 0; i < breps.size(); i++) {
        for (size_t j = 0; j < breps.size(); j++) {
            for (int pose = 0; pose < 4; pose++) {
                char nm[48];
                std::snprintf(nm, 48, "%s_x_%s_p%d", PRIMS[i].name, PRIMS[j].name,
                              pose);
                if (only && !std::strstr(nm, only)) continue;
                run_case(breps[i], breps[j], vols[i], vols[j], nm);
            }
        }
    }

    std::printf("\nG3-step totals: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
