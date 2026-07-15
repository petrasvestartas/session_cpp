// pb2step — write STEP from cached BRep protobufs, WITHOUT re-running any boolean.
//
// The STEP writer used to be reachable only through main_7's validation gate, so
// regenerating a .step meant recomputing its boolean (tor x tor alone is minutes).
// This tool loads already-computed BReps from .pb files and writes STEP directly:
// seconds, and immune to whatever else is rebuilding main_7.exe.
//
//   cmake --build build --config Release --target pb2step
//   ./build/Release/pb2step.exe <out.step> <in1.pb> [in2.pb in3.pb ...]
//
// One .pb = one BRep (BRep::pb_load). With a single input the tool writes a plain
// single-solid STEP. With several inputs it writes ONE colored file in the
// boolean_steps convention: #1 red (operand A), #2 blue (operand B), #3+ green
// (result), so `pb2step chair.step chair0.pb chair1.pb chair_cut.pb` reproduces the
// A/B/result file with no cutting.
#include <cstdio>
#include <string>
#include <vector>
#include "brep.h"
#include "color.h"
#include "file_step.h"
using namespace session_cpp;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
            "usage: pb2step <out.step> <in1.pb> [in2.pb in3.pb ...]\n"
            "  1 input  -> plain STEP of that BRep\n"
            "  N inputs -> one colored STEP: #1 red, #2 blue, #3+ green\n");
        return 2;
    }
    const std::string out = argv[1];
    std::vector<BRep> breps;
    breps.reserve(argc - 2);
    for (int i = 2; i < argc; ++i) {
        BRep b = BRep::pb_load(argv[i]);
        std::printf("loaded %-40s faces=%d solid=%d\n",
                    argv[i], b.face_count(), b.is_solid() ? 1 : 0);
        breps.push_back(std::move(b));
    }
    if (breps.size() == 1) {
        file_step::write_file_step_brep(breps[0], out);
        std::printf("wrote %s (1 solid, %d faces)\n", out.c_str(), breps[0].face_count());
        return 0;
    }
    // color by role: red = A, blue = B, green = every result solid after that
    for (size_t i = 0; i < breps.size(); ++i) {
        if (i == 0)      breps[i].surfacecolor = Color(0.85f, 0.25f, 0.20f);
        else if (i == 1) breps[i].surfacecolor = Color(0.20f, 0.45f, 0.85f);
        else             breps[i].surfacecolor = Color(0.35f, 0.75f, 0.40f);
    }
    std::vector<const BRep*> parts;
    for (auto& b : breps) parts.push_back(&b);
    // product name = output basename without directory or .step suffix
    std::string name = out;
    if (size_t s = name.find_last_of("/\\"); s != std::string::npos) name = name.substr(s + 1);
    if (size_t d = name.rfind(".step"); d != std::string::npos) name = name.substr(0, d);
    file_step::write_file_step_breps(parts, name, out);
    int tot = 0;
    for (auto& b : breps) tot += b.face_count();
    std::printf("wrote %s (%zu solids, %d faces)\n", out.c_str(), breps.size(), tot);
    return 0;
}
