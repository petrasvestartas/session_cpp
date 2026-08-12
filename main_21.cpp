// main_21 — random-rotation boolean battery for the six primitives, with STEP output.
//
// For every unordered pair (with repetition) of {box, sphere, pyramid, torus, cone, cylinder}
// and every op {cut, common, fuse}, operand B is given a seeded random rotation + translation
// (deterministic per pair/op/seed), the boolean runs through the v2 pipeline
// (v2sol::v2_boolean; its report says whether the v2 front end answered or it delegated to
// the kernel's v1 route), and the result is written as a STEP file for visual inspection.
//
// Every produced STEP is gated against the OCCT oracle (validation/step_probe): the oracle
// booleans THE SAME two operand STEP files, and our result must import VALID with a volume
// within 1% of the oracle's (empty/degenerate cells follow the OCCT_TRUTH.md doctrine).
//
//   main_21 [--seeds N] [--seed0 S] [--out DIR] [--pairs a,b] [--ops cut,common,fuse]
//           [--probe PATH] [--no-oracle]
//
// Defaults: --seeds 2 --out validation/boolean_steps_v2 --probe validation/step_probe/build/step_probe
// Run from the repository root.

#include "src/brep.h"
#include "src/file_step.h"
#include "src/xform.h"
#include "src/v2/brep_v2_boolean.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <vector>

using namespace session_cpp;

namespace {

struct Prim {
    std::string name;
    std::function<BRep()> make;
    double radius;   // bounding-sphere radius of the canonical placement (for overlap placement)
};

BRep centered(BRep b, double dz) {
    b.transform(Xform::translation(0, 0, dz));
    return b;
}

const std::vector<Prim>& prims() {
    static const std::vector<Prim> ps = {
        {"box",      [] { return BRep::create_box(4, 4, 4); },                    3.4641},
        {"sphere",   [] { return BRep::create_sphere(2.5); },                     2.5},
        {"pyramid",  [] { return centered(BRep::create_pyramid(4, 5), -2.5); },   3.7749},
        {"torus",    [] { return BRep::create_torus(3, 1); },                     4.0},
        {"cone",     [] { return centered(BRep::create_cone(2, 5), -2.5); },      3.2016},
        {"cylinder", [] { return centered(BRep::create_cylinder(1.5, 6), -3.0); },3.3541},
    };
    return ps;
}

uint64_t mix(uint64_t h, uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
}
uint64_t hash_str(const std::string& s) {
    uint64_t h = 1469598103934665603ULL;
    for (char c : s) { h ^= (uint64_t)(unsigned char)c; h *= 1099511628211ULL; }
    return h;
}

std::string run_capture(const std::string& cmd) {
    std::string out;
    FILE* fp = popen(cmd.c_str(), "r");
    if (!fp) return out;
    char buf[4096];
    while (fgets(buf, sizeof buf, fp)) out += buf;
    pclose(fp);
    return out;
}

// Find "KEY=<number>" or "KEY <number>" in probe output.
bool find_kv(const std::string& text, const std::string& key, double& val) {
    size_t p = text.find(key);
    while (p != std::string::npos) {
        size_t e = p + key.size();
        if (e < text.size() && (text[e] == '=' || text[e] == ' ')) {
            while (e < text.size() && (text[e] == '=' || text[e] == ' ')) ++e;
            char* end = nullptr;
            val = std::strtod(text.c_str() + e, &end);
            if (end != text.c_str() + e) return true;
        }
        p = text.find(key, p + 1);
    }
    return false;
}

struct OracleResult {
    bool ran = false;
    double volume = 0.0, solids = -1, valid = -1, faces = -1;
};
OracleResult oracle_boolean(const std::string& probe, const std::string& op,
                            const std::string& fa, const std::string& fb) {
    OracleResult o;
    std::string out = run_capture("\"" + probe + "\" --" + op + " \"" + fa + "\" \"" + fb + "\" 2>/dev/null");
    if (out.empty()) return o;
    o.ran = true;
    find_kv(out, "OP_VOLUME", o.volume);
    find_kv(out, "OP_SOLIDS", o.solids);
    find_kv(out, "OP_VALID", o.valid);
    find_kv(out, "OP_FACES", o.faces);
    return o;
}

struct ProbeResult {
    bool ran = false;
    double volume = 0.0, solids = -1, valid = -1, naked = -1;
};
ProbeResult probe_result(const std::string& probe, const std::string& f) {
    ProbeResult r;
    std::string out = run_capture("\"" + probe + "\" \"" + f + "\" -n 2>/dev/null");
    if (out.empty()) return r;
    r.ran = true;
    find_kv(out, "VOLUME", r.volume);
    find_kv(out, "SOLIDS", r.solids);
    find_kv(out, "VALID", r.valid);
    find_kv(out, "NAKED", r.naked);
    return r;
}

void ensure_dir(const std::string& path) {
    std::string cur;
    for (char c : path) {
        cur += c;
        if (c == '/' && cur.size() > 1) mkdir(cur.c_str(), 0775);
    }
    mkdir(path.c_str(), 0775);
}

}  // namespace

int main(int argc, char** argv) {
    int seeds = 2, seed0 = 1;
    std::string outdir = "validation/boolean_steps_v2";
    std::string probe = "validation/step_probe/build/step_probe";
    std::string pair_filter, op_filter = "cut,common,fuse";
    bool use_oracle = true;

    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto nxt = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--seeds") seeds = std::max(1, std::atoi(nxt().c_str()));
        else if (k == "--seed0") seed0 = std::atoi(nxt().c_str());
        else if (k == "--out") outdir = nxt();
        else if (k == "--probe") probe = nxt();
        else if (k == "--pairs") pair_filter = nxt();
        else if (k == "--ops") op_filter = nxt();
        else if (k == "--no-oracle") use_oracle = false;
        else { std::fprintf(stderr, "main_21: unknown option %s\n", k.c_str()); return 2; }
    }
    if (use_oracle) {
        std::ifstream chk(probe);
        if (!chk.good()) {
            std::fprintf(stderr, "main_21: no oracle at %s (--probe, or --no-oracle)\n", probe.c_str());
            return 2;
        }
    }

    std::vector<std::string> ops;
    {
        std::stringstream ss(op_filter);
        std::string o;
        while (std::getline(ss, o, ',')) ops.push_back(o);
    }

    ensure_dir(outdir);
    std::ofstream card(outdir + "/_scorecard.md");
    card << "# Boolean battery scorecard — random rotations, v2 pipeline vs OCCT step_probe\n\n";
    card << "| case | op | engine | our valid/naked/solids/vol | occt valid/solids/vol | verdict |\n";
    card << "|---|---|---|---|---|---|\n";

    int npass = 0, nfail = 0, nskip = 0;
    const auto& ps = prims();

    for (size_t ia = 0; ia < ps.size(); ++ia) {
        for (size_t ib = ia; ib < ps.size(); ++ib) {
            const std::string pair = ps[ia].name + "_" + ps[ib].name;
            if (!pair_filter.empty() && pair_filter.find(pair) == std::string::npos) continue;

            BRep A = ps[ia].make();
            const std::string pdir = outdir + "/" + pair;
            ensure_dir(pdir);

            for (int seed = seed0; seed < seed0 + seeds; ++seed) {
                // Deterministic per (pair, seed): rotate B about a random axis by a random
                // angle, then translate so the centres are 10..60% of (rA+rB) apart.
                uint64_t h = mix(hash_str(pair), (uint64_t)seed * 0x2545F4914F6CDD1DULL);
                std::mt19937_64 rng(h);
                std::normal_distribution<double> gauss(0.0, 1.0);
                std::uniform_real_distribution<double> uni(0.0, 1.0);
                Vector axis(gauss(rng), gauss(rng), gauss(rng));
                const double alen = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
                axis = Vector(axis[0]/alen, axis[1]/alen, axis[2]/alen);
                const double ang = uni(rng) * 360.0;
                Vector dir(gauss(rng), gauss(rng), gauss(rng));
                const double dlen = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
                const double dist = (0.10 + 0.50 * uni(rng)) * (ps[ia].radius + ps[ib].radius);
                const double tx = dir[0]/dlen*dist, ty = dir[1]/dlen*dist, tz = dir[2]/dlen*dist;

                BRep B = ps[ib].make();
                B = B.transformed(Xform::translation(tx, ty, tz) * Xform::rotation(axis, ang, true));

                char sbuf[8];
                std::snprintf(sbuf, sizeof sbuf, "s%02d", seed);
                const std::string fa = pdir + "/_A_" + sbuf + ".step";
                const std::string fb = pdir + "/_B_" + sbuf + ".step";
                file_step::write_file_step_brep(A, fa);
                file_step::write_file_step_brep(B, fb);

                // Run every op first; the verdict is per-TRIPLE (OCCT_TRUTH.md doctrine):
                // cells where the ORACLE's own answer is broken (OP_SOLIDS=0 with real volume,
                // or OP_VALID=0) are gated on the partition identities cut+common=vol(A),
                // fuse=vol(A)+vol(B)-common, never on the oracle's wrong volume.
                struct OpRec {
                    std::string op, engine = "-", why;
                    bool wrote = false, threw = false;
                    ProbeResult pr;
                    OracleResult oc;
                    int verdict = 0;   // 0 skip, 1 pass, 2 fail
                };
                std::vector<OpRec> recs;
                for (const std::string& op : ops) {
                    OpRec rc;
                    rc.op = op;
                    const std::string tag = pair + ":" + op + ":" + sbuf;
                    if (std::getenv("SESSION_M21_DBG"))
                        std::fprintf(stderr, "[M21] %s axis=(%.4f,%.4f,%.4f) ang=%.3f t=(%.4f,%.4f,%.4f)\n",
                                     tag.c_str(), axis[0], axis[1], axis[2], ang, tx, ty, tz);
                    v2sol::V2BooleanReport rep;
                    v2sol::V2Op vop = v2sol::V2Op::Cut;
                    if (op == "common") vop = v2sol::V2Op::Common;
                    else if (op == "fuse") vop = v2sol::V2Op::Fuse;
                    else if (op == "cut21") vop = v2sol::V2Op::Cut21;
                    BRep R;
                    try {
                        R = v2sol::v2_boolean(A, B, vop, v2sol::V2BooleanOptions{}, &rep);
                    } catch (const std::exception& e) {
                        rc.threw = true;
                        rc.why = std::string("threw ") + e.what();
                        recs.push_back(rc);
                        continue;
                    }
                    rc.engine = rep.stage_fail.empty() ? "v2front" : "v1deleg";
                    const std::string fr = pdir + "/" + pair + "_" + op + "_" + sbuf + ".step";
                    if (R.face_count() > 0) {
                        file_step::write_file_step_brep(R, fr);
                        rc.wrote = true;
                    }
                    if (use_oracle) {
                        rc.oc = oracle_boolean(probe, op, fa, fb);
                        if (rc.wrote) rc.pr = probe_result(probe, fr);
                    }
                    recs.push_back(rc);
                }

                const double volA = std::abs(A.volume()), volB = std::abs(B.volume());
                auto find_rec = [&](const char* o) -> const OpRec* {
                    for (const auto& r : recs) if (r.op == o) return &r;
                    return nullptr;
                };
                // partition identities over OUR measured volumes (empty = 0)
                bool ident_ran = false, ident_ok = true;
                std::string ident_why;
                if (use_oracle) {
                    const OpRec *rcut = find_rec("cut"), *rcom = find_rec("common"),
                                *rfus = find_rec("fuse");
                    if (rcut && rcom && rfus && !rcut->threw && !rcom->threw && !rfus->threw) {
                        auto vol_of = [](const OpRec* r) {
                            return (r->wrote && r->pr.ran) ? r->pr.volume : 0.0;
                        };
                        const double vc = vol_of(rcut), vm = vol_of(rcom), vf = vol_of(rfus);
                        const double e1 = std::fabs(vc + vm - volA);
                        const double e2 = std::fabs(vf + vm - (volA + volB));
                        ident_ran = true;
                        if (e1 > 0.01 * volA) {
                            ident_ok = false;
                            char b[64];
                            std::snprintf(b, sizeof b, " cut+common-A=%.4g", e1);
                            ident_why += b;
                        }
                        if (e2 > 0.01 * (volA + volB)) {
                            ident_ok = false;
                            char b[64];
                            std::snprintf(b, sizeof b, " fuse+common-A-B=%.4g", e2);
                            ident_why += b;
                        }
                    }
                }

                for (OpRec& rc : recs) {
                    if (rc.threw) { rc.verdict = 2; continue; }
                    if (!use_oracle) { rc.verdict = 0; continue; }
                    const bool oracle_broken =
                        rc.oc.ran && rc.oc.faces >= 0 &&
                        ((rc.oc.solids == 0 && std::fabs(rc.oc.volume) > 0.01) || rc.oc.valid == 0);
                    if (!rc.wrote) {
                        if (rc.oc.ran && rc.oc.solids == 0) { rc.verdict = 1; rc.why = "empty=empty"; }
                        else { rc.verdict = 2; rc.why = "empty, oracle not empty"; }
                        continue;
                    }
                    if (!rc.pr.ran || !rc.oc.ran) { rc.verdict = 0; rc.why = "probe failed"; continue; }
                    // our STEP must import VALID, always (oracle topology is never gated)
                    if (rc.pr.valid != 1) { rc.verdict = 2; rc.why = "invalid"; continue; }
                    const bool degenerate = std::fabs(rc.oc.volume) <= 0.01;
                    if (degenerate) {   // grazing/contact cell: never gate volume
                        rc.verdict = (std::fabs(rc.pr.volume) <= 0.05) ? 1 : 2;
                        if (rc.verdict == 2) rc.why = "degenerate but vol=" +
                            std::to_string(rc.pr.volume);
                        continue;
                    }
                    if (oracle_broken) {
                        // OCCT's own cell is defective: the identities are the only truth
                        if (ident_ran && ident_ok) { rc.verdict = 1; rc.why = "occt-broken,ident-ok"; }
                        else { rc.verdict = 2; rc.why = "occt-broken" + ident_why; }
                        continue;
                    }
                    if (std::fabs(rc.pr.volume - rc.oc.volume) > 0.01 * std::fabs(rc.oc.volume)) {
                        rc.verdict = 2;
                        rc.why = "vol";
                        continue;
                    }
                    rc.verdict = 1;
                }

                for (const OpRec& rc : recs) {
                    const char* vstr = rc.verdict == 1 ? "PASS" : rc.verdict == 2 ? "FAIL" : "SKIP";
                    if (rc.verdict == 1) ++npass;
                    else if (rc.verdict == 2) ++nfail;
                    else ++nskip;
                    char ours[128], theirs[128];
                    std::snprintf(ours, sizeof ours, "%.0f/%.0f/%.0f/%.6g",
                                  rc.pr.valid, rc.pr.naked, rc.pr.solids, rc.pr.volume);
                    std::snprintf(theirs, sizeof theirs, "%.0f/%.0f/%.6g",
                                  rc.oc.valid, rc.oc.solids, rc.oc.volume);
                    const std::string tag = pair + ":" + rc.op + ":" + sbuf;
                    card << "| " << pair << " " << sbuf << " | " << rc.op << " | " << rc.engine
                         << " | " << (rc.wrote ? ours : (rc.threw ? "THREW" : "empty")) << " | "
                         << (use_oracle ? theirs : "-") << " | " << vstr
                         << (rc.why.empty() ? "" : (" " + rc.why)) << " |\n";
                    std::printf("%-4s %-44s %-8s ours=%-28s occt=%s %s\n", vstr, tag.c_str(),
                                rc.engine.c_str(), rc.wrote ? ours : (rc.threw ? "THREW" : "empty"),
                                use_oracle ? theirs : "-", rc.why.c_str());
                }
            }
        }
    }
    card << "\nPASS " << npass << " FAIL " << nfail << " SKIP " << nskip << "\n";
    std::printf("\nPASS %d FAIL %d SKIP %d\nscorecard: %s\n", npass, nfail, nskip,
                (outdir + "/_scorecard.md").c_str());
    return nfail > 0 ? 1 : 0;
}
