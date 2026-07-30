// main_20 — v2_trace: runs the v2 boolean pipeline and dumps its complete internal state in
// validation/occt_trace's record format, so an OCCT trace and a v2 trace of the SAME case can be
// diffed stage by stage (validation/v2_trace/v2_trace_diff.py).
//
// Usage (deliberately identical to occt_trace's, so one script drives both):
//   main_20 --op cut|common|fuse --a <spec> --b <spec> [--name <id>] [--out <file>]
//           [--no-interf] [--no-result] [--trail <n>]
//
//   <spec> := <type>[,<key>=<val>]*
//     sphere,r=2.5   cylinder,r=1,h=8,center   box,dx=4,dy=4,dz=4,center
//     cone,r1=2,r2=0,h=5,center                torus,r1=3,r2=1
//   transform keys, applied in this order about the origin:
//     center (flag) rotx=<deg> roty=<deg> rotz=<deg> tx= ty= tz=

#include "src/v2/v2_dump.h"

#include <fstream>
#include <iostream>
#include <string>

using namespace session_cpp;

int main(int argc, char** argv) {
    v2dump::V2DumpOptions opt;
    std::string aS, bS, outS;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto nxt = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--op") opt.op = nxt();
        else if (k == "--a") aS = nxt();
        else if (k == "--b") bS = nxt();
        else if (k == "--out") outS = nxt();
        else if (k == "--name") opt.name = nxt();
        else if (k == "--no-interf") opt.run_interf = false;
        else if (k == "--no-result") opt.run_result = false;
        else if (k == "--trail") opt.trail_samples = std::atoi(nxt().c_str());
        else {
            std::cerr << "main_20: unknown option " << k << "\n";
            return 2;
        }
    }
    if (aS.empty() || bS.empty()) {
        std::cerr << "usage: main_20 --op cut|common|fuse --a <spec> --b <spec> "
                     "[--name id] [--out file] [--no-interf] [--no-result] [--trail n]\n";
        return 2;
    }

    std::ofstream fout;
    if (!outS.empty()) fout.open(outS.c_str());
    std::ostream& os = outS.empty() ? std::cout : fout;

    const v2dump::V2DumpSpec spa = v2dump::v2dump_parse_spec(aS);
    const v2dump::V2DumpSpec spb = v2dump::v2dump_parse_spec(bS);
    return v2dump::v2dump_case(os, spa, spb, opt);
}
