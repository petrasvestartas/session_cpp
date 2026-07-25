// main_9 — standalone STEP boolean harness (step_crash_test / arbitrary imported pairs).
// usage: main_9 <A.step[:i]> <B.step[:j]> <cut|common|fuse|all> [out_dir]
//   A/B: STEP files; ':i' picks solid i of the file (default 0). A single file holding
//   several solids (our colored triples) can supply both operands: f.step:0 f.step:1.
// Prints one machine-parseable line per op:
//   RES op=<op> faces=N solid=0|1 naked=N vol=V
// plus topology_report, and writes a colored triple STEP per op when out_dir is given.
#include "src/brep.h"
#include "src/file_step.h"
#include "src/color.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;

static bool parse_spec(const std::string& spec, std::string& path, int& idx) {
    idx = 0;
    path = spec;
    auto pos = spec.find_last_of(':');
    if (pos != std::string::npos && pos > 1) {   // pos>1 skips the drive-letter colon
        bool digits = pos + 1 < spec.size();
        for (size_t i = pos + 1; i < spec.size(); ++i)
            if (!isdigit((unsigned char)spec[i])) digits = false;
        if (digits) {
            path = spec.substr(0, pos);
            idx = std::atoi(spec.c_str() + pos + 1);
        }
    }
    return true;
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc < 4) {
        std::printf("usage: main_9 <A.step[:i]> <B.step[:j]> <cut|common|fuse|all> [out_dir]\n");
        return 2;
    }
    std::string pa, pb;
    int ia = 0, ib = 0;
    parse_spec(argv[1], pa, ia);
    parse_spec(argv[2], pb, ib);
    std::string opsel = argv[3];
    std::string outdir = argc >= 5 ? argv[4] : "";

    auto as = file_step::read_file_step_breps(pa);
    auto bs = file_step::read_file_step_breps(pb);
    std::printf("A: %s breps %zu pick %d | B: %s breps %zu pick %d\n",
                pa.c_str(), as.size(), ia, pb.c_str(), bs.size(), ib);
    if (ia >= (int)as.size() || ib >= (int)bs.size()) { std::printf("PICK_FAIL\n"); return 1; }
    BRep& A = as[ia];
    BRep& B = bs[ib];
    std::printf("A: faces %d solid %d vol %.4f | B: faces %d solid %d vol %.4f\n",
                A.face_count(), A.is_solid() ? 1 : 0, A.volume(),
                B.face_count(), B.is_solid() ? 1 : 0, B.volume());
    A.surfacecolor = Color(0.85f, 0.25f, 0.20f);
    B.surfacecolor = Color(0.20f, 0.45f, 0.85f);

    const char* opn[3] = {"cut", "common", "fuse"};
    int rc = 0;
    for (int m = 0; m < 3; ++m) {
        if (opsel != "all" && opsel != opn[m]) continue;
        try {
            BRep r = (m == 0) ? A.boolean_difference(B)
                   : (m == 1) ? A.boolean_intersection(B)
                              : A.boolean_union(B);
            int nkb = 0;
            for (const auto& Eb : r.m_topology_edges)
                if ((int)Eb.trim_indices.size() == 1) ++nkb;
            double vol = r.volume();
            std::printf("RES op=%s faces=%d solid=%d naked=%d vol=%.6f\n",
                        opn[m], r.face_count(), r.is_solid() ? 1 : 0, nkb, vol);
            bool ok = false;
            std::string rep = r.topology_report(&ok);
            std::printf("%s", rep.c_str());
            if (!ok || nkb > 0 || !r.is_solid()) rc = 1;
            if (!outdir.empty() && r.face_count() > 0) {
                r.surfacecolor = Color(0.35f, 0.75f, 0.40f);
                std::vector<const BRep*> parts = {&A, &B, &r};
                std::string nm = std::string("res_") + opn[m];
                file_step::write_file_step_breps(parts, nm, outdir + "/" + nm + ".step");
            }
        } catch (const std::exception& e) {
            std::printf("RES op=%s THREW: %s\n", opn[m], e.what());
            rc = 1;
        }
    }
    return rc;
}
