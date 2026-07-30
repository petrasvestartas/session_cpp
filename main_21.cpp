// main_21 — TIER 2: THE ROTATED CHAIRS THROUGH BOTH KERNELS.
//
// The chairs are trimmed multi-face solids read from STEP (20 faces each) — the closest thing
// in this corpus to real CAD input. This driver runs ONE cell per process so that 30 cells x 2
// kernels can be spread over the machine and so that a hang or a crash costs one cell instead
// of the whole sweep. It performs NO scoring of its own: every number printed comes from the
// shared harness src/v2/v2_verdict.h (validated by main_17), and the operands are the STORED
// files serialization/boolean_steps/chairs/rot/B_<cfg>.step — byte-identical to the ones that
// produced every historical number — never a re-derived rotation.
//
// ENV (all required except where noted):
//   SESSION_T2_CFG     base | z15 | z30 | z45 | z90 | x20 | y30 | z30x20 | z37 | x13y29 | z63
//                      or  self   (A-op-A: chair0 against an independently re-read copy)
//   SESSION_T2_OP      cut | common | fuse
//   SESSION_T2_KERNEL  v1 (BRep::boolean) | v2 (v2sol::v2_boolean)
//   SESSION_T2_DIR     chairs directory (default serialization/boolean_steps/chairs)
//   SESSION_T2_OPERANDS  optional: also print the verdict of A and B and exit
//
// Output: exactly one line beginning "T2CELL " with key=value fields, plus a trailing
// "T2DONE" so a collector can tell a completed run from a killed one.

#include "src/brep.h"
#include "src/file_step.h"
#include "src/v2/brep_v2_boolean.h"
#include "src/v2/v2_verdict.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;
using session_cpp::v2v::V2Verdict;
using session_cpp::v2v::v2_verdict;

static const char* env_or(const char* k, const char* dflt) {
    const char* p = std::getenv(k);
    return (p && p[0]) ? p : dflt;   // getenv()!=nullptr is TRUE for an EMPTY value
}

static double secs_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

/// THE SCORING OPTIONS. The harness's own defaults (4e6 global / 4e4 per-face evaluations) are
/// MEASURED to be insufficient on this corpus: at the default budget the un-booleaned operand
/// chair0 integrates to volume 83.1539 (OCCT truth 80.2969, +3.6%) with closure_residual
/// 9.117e-03 and converged=0 — one face (index 16) hits its per-face cap. With the budget raised
/// 100x the SAME operand gives 80.2941 (3.4e-05 relative) and residual 6.37e-06. Scoring a
/// boolean result against a reference volume with a quadrature that is itself 3.6% wrong would
/// measure the quadrature, not the kernel, so every number in this driver uses the raised
/// budget. Nothing else about the harness is changed: v2v::v2_verdict(b, opt) is a documented
/// entry point of the shared harness and all the topology rules are untouched.
static MassPropsOptions score_options() {
    MassPropsOptions o = v2v::v2_verdict_options();
    long long mult = 100;
    const char* p = std::getenv("SESSION_T2_BUDGET");
    if (p && p[0]) mult = std::atoll(p);
    if (mult < 1) mult = 1;
    o.max_surface_evals *= mult;
    o.min_face_evals *= mult;
    return o;
}

static void print_verdict(const char* tag, const std::string& cfg, const std::string& op,
                          const std::string& kern, const V2Verdict& v, double secs,
                          const char* note) {
    std::printf("%s cfg=%s op=%s kernel=%s F=%d shells=%d solids=%d naked=%d nonman=%d seam=%d "
                "degen=%d orphan=%d edges=%d resid=%.3e area=%.6f vol=%.6f volvalid=%d "
                "converged=%d closed=%d secs=%.1f note=%s\n",
                tag, cfg.c_str(), op.c_str(), kern.c_str(), v.faces, v.shells, v.solids,
                v.naked_real, v.nonmanifold, v.seam_edges, v.degenerate, v.orphan, v.edges,
                v.closure_residual, v.area, v.volume, (int)v.volume_valid, (int)v.converged,
                (int)v.closed(), secs, note);
    std::fflush(stdout);
}

int main() {
    const std::string dir = env_or("SESSION_T2_DIR", "serialization/boolean_steps/chairs");
    const std::string cfg = env_or("SESSION_T2_CFG", "base");
    const std::string op = env_or("SESSION_T2_OP", "cut");
    const std::string kern = env_or("SESSION_T2_KERNEL", "v2");

    std::vector<BRep> as, bs;
    try {
        as = file_step::read_file_step_breps(dir + "/chair0.stp");
        if (cfg == "base")
            bs = file_step::read_file_step_breps(dir + "/chair1.stp");
        else if (cfg == "self")
            bs = file_step::read_file_step_breps(dir + "/chair0.stp");   // independent re-read
        else
            bs = file_step::read_file_step_breps(dir + "/rot/B_" + cfg + ".step");
    } catch (const std::exception& e) {
        std::printf("T2CELL cfg=%s op=%s kernel=%s LOAD_THREW %s\nT2DONE\n", cfg.c_str(),
                    op.c_str(), kern.c_str(), e.what());
        return 1;
    }
    if (as.empty() || bs.empty()) {
        std::printf("T2CELL cfg=%s op=%s kernel=%s LOAD_EMPTY a=%zu b=%zu\nT2DONE\n", cfg.c_str(),
                    op.c_str(), kern.c_str(), as.size(), bs.size());
        return 1;
    }
    const BRep& A = as[0];
    const BRep& B = bs[0];

    if (std::getenv("SESSION_T2_OPERANDS")) {
        // The harness is only usable on this corpus if it certifies the OPERANDS. Print the
        // full mass-properties census so a failure to certify can be attributed (budget,
        // trim-chain gap, or a genuinely open input) instead of guessed at.
        const BRep* ops[2] = {&A, &B};
        const char* nm[2] = {"A", "B"};
        for (int i = 0; i < 2; ++i) {
            auto t0 = std::chrono::steady_clock::now();
            const V2Verdict v = v2_verdict(*ops[i]);
            print_verdict("T2OPERAND", cfg, "-", nm[i], v, secs_since(t0),
                          ops[i]->is_solid() ? "is_solid=1" : "is_solid=0");
            const MassProps mp = brep_massprops(*ops[i], v2v::v2_verdict_options());
            std::printf("T2MP %s kernel_volume=%.6f mp_volume=%.6f volerr=%.3e area=%.6f "
                        "converged=%d closed=%d resid=%.3e max_chain_gap=%.3e naked=%d "
                        "shells=%d evals=%lld secs=%.1f\n",
                        nm[i], ops[i]->volume(), mp.volume, mp.volume_error, mp.area,
                        (int)mp.converged, (int)mp.closed, mp.closure_residual, mp.max_chain_gap,
                        mp.naked_edges, mp.shell_count, mp.surface_evals, mp.seconds);
            for (const auto& f : mp.faces)
                std::printf("T2MPF %s face=%d path=%d area=%.6f aerr=%.2e gap=%.3e vec=(%.3e,%.3e,%.3e) "
                            "evals=%lld budget_hit=%d\n",
                            nm[i], f.face_index, (int)f.path, f.area, f.area_err, f.chain_gap,
                            f.vec_area[0], f.vec_area[1], f.vec_area[2], f.evals,
                            (int)f.budget_hit);
        }
        std::printf("T2DONE\n");
        return 0;
    }

    if (std::getenv("SESSION_T2_SELFTEST")) {
        // WHY the harness refuses the chairs. Three questions, each answered by measurement:
        //  (a) is it the BUDGET?      -> re-score chair0 with a 100x larger evaluation budget
        //  (b) is it STEP IMPORT?     -> round-trip a box and a sphere through STEP and re-score
        //  (c) is it CHAIR-SPECIFIC?  -> compare the two answers
        MassPropsOptions big = v2v::v2_verdict_options();
        big.max_surface_evals = 400000000;
        big.min_face_evals = 20000000;
        const V2Verdict vb = v2_verdict(A, big);
        std::printf("T2SELF chair0_big_budget %s\n", vb.str().c_str());

        struct RT { const char* name; BRep b; };
        std::vector<RT> rts;
        rts.push_back({"box2", BRep::create_box(2, 2, 2)});
        rts.push_back({"sphere1", BRep::create_sphere(1.0)});
        rts.push_back({"cylinder", BRep::create_cylinder(1.0, 2.0)});
        const std::string tmp = env_or("SESSION_T2_TMP", "/tmp");
        for (auto& rt : rts) {
            const V2Verdict v0 = v2_verdict(rt.b);
            const std::string p = tmp + "/t2_rt_" + rt.name + ".step";
            file_step::write_file_step_brep(rt.b, p);
            auto back = file_step::read_file_step_breps(p);
            std::printf("T2SELF %-9s before{%s}\n", rt.name, v0.str().c_str());
            if (back.empty()) { std::printf("T2SELF %-9s after{NO_BREP}\n", rt.name); continue; }
            const V2Verdict v1 = v2_verdict(back[0]);
            const MassProps m1 = brep_massprops(back[0], v2v::v2_verdict_options());
            std::printf("T2SELF %-9s after {%s} max_chain_gap=%.3e kernel_vol=%.6f\n", rt.name,
                        v1.str().c_str(), m1.max_chain_gap, back[0].volume());
        }
        std::printf("T2DONE\n");
        return 0;
    }

    const auto t0 = std::chrono::steady_clock::now();
    BRep r;
    try {
        if (kern == "v1") {
            r = (op == "cut")      ? A.boolean_difference(B)
                : (op == "common") ? A.boolean_intersection(B)
                                   : A.boolean_union(B);
        } else {
            v2sol::V2BooleanReport rep;
            r = (op == "cut")      ? v2sol::v2_cut(A, B, 0.0, &rep)
                : (op == "common") ? v2sol::v2_common(A, B, 0.0, &rep)
                                   : v2sol::v2_fuse(A, B, 0.0, &rep);
            std::printf("T2REPORT cfg=%s op=%s %s\n", cfg.c_str(), op.c_str(),
                        rep.str().c_str());
            std::fflush(stdout);
        }
    } catch (const std::exception& e) {
        std::printf("T2CELL cfg=%s op=%s kernel=%s THREW secs=%.1f what=%s\nT2DONE\n", cfg.c_str(),
                    op.c_str(), kern.c_str(), secs_since(t0), e.what());
        return 2;
    } catch (...) {
        std::printf("T2CELL cfg=%s op=%s kernel=%s THREW secs=%.1f what=?\nT2DONE\n", cfg.c_str(),
                    op.c_str(), kern.c_str(), secs_since(t0));
        return 2;
    }
    const double t_bool = secs_since(t0);

    if (r.face_count() == 0) {
        std::printf("T2CELL cfg=%s op=%s kernel=%s F=0 shells=0 solids=0 naked=0 nonman=0 seam=0 "
                    "degen=0 orphan=0 edges=0 resid=1.000e+00 area=0.000000 vol=0.000000 "
                    "volvalid=0 converged=1 closed=0 secs=%.1f note=EMPTY_RESULT\n",
                    cfg.c_str(), op.c_str(), kern.c_str(), t_bool);
        std::printf("T2DONE\n");
        return 0;
    }

    const auto t1 = std::chrono::steady_clock::now();
    const V2Verdict v = v2_verdict(r, score_options());  // THE shared harness, raised budget.
    char note[96];
    std::snprintf(note, sizeof note, "kvol=%.6f score_secs=%.1f",
                  (v.naked_real == 0 && v.nonmanifold == 0) ? r.volume() : 0.0, secs_since(t1));
    print_verdict("T2CELL", cfg, op, kern, v, t_bool, note);
    std::printf("T2DONE\n");
    return 0;
}
