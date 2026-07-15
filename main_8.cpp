// main_8 — mesh-boolean driver. Exercises Mesh::boolean_difference (marching-cubes CSG) on two
// imported chair solids and reports watertightness + volume. Env:
//   SESSION_CHAIRS=<dir>   read chair0.stp / chair1.stp, difference chair0 - chair1
//   SESSION_MC_N=<int>     grid resolution per longest bbox axis (default 96)
//   SESSION_MC_OUT=<path>  dump result mesh as .obj
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <string>
#include <map>
#include "brep.h"
#include "mesh.h"
#include "file_step.h"
using namespace session_cpp;

int main(int, char**) {
    if (std::getenv("SESSION_MC_SELFTEST")) {
        int N = std::getenv("SESSION_MC_N") ? std::atoi(std::getenv("SESSION_MC_N")) : 48;
        Mesh a = Mesh::create_box(2, 2, 2), b = Mesh::create_box(2, 2, 2);
        b.transform(Xform::translation(1, 0, 0));   // overlap 1x2x2 = 4
        Mesh d = a.boolean_difference(b, N), u = a.boolean_union(b, N), i = a.boolean_intersection(b, N);
        std::printf("diff:  closed %d vol %.4f (expect ~4)\n", d.is_closed()?1:0, std::abs(d.volume()));
        std::printf("union: closed %d vol %.4f (expect ~12)\n", u.is_closed()?1:0, std::abs(u.volume()));
        std::printf("inter: closed %d vol %.4f (expect ~4)\n", i.is_closed()?1:0, std::abs(i.volume()));
        return 0;
    }
    const char* cd = std::getenv("SESSION_CHAIRS");
    if (!cd) { std::fprintf(stderr, "set SESSION_CHAIRS=<dir with chair0.stp/chair1.stp>\n"); return 2; }
    int N = std::getenv("SESSION_MC_N") ? std::atoi(std::getenv("SESSION_MC_N")) : 96;

    auto as = file_step::read_file_step_breps(std::string(cd) + "/chair0.stp");
    auto bs = file_step::read_file_step_breps(std::string(cd) + "/chair1.stp");
    if (as.empty() || bs.empty()) { std::fprintf(stderr, "failed to read chairs\n"); return 2; }
    Mesh mA = as[0].mesh(), mB = bs[0].mesh();
    std::printf("meshes: A faces %zu closed %d | B faces %zu closed %d\n",
                mA.number_of_faces(), mA.is_closed()?1:0, mB.number_of_faces(), mB.is_closed()?1:0);

    auto t0 = std::chrono::steady_clock::now();
    Mesh out = mA.boolean_difference(mB, N);
    auto t1 = std::chrono::steady_clock::now();
    std::printf("boolean_difference N=%d: faces %zu naked %zu closed %d vol %.4f (%.1fs)\n",
                N, out.number_of_faces(), out.naked_edges(true).size(),
                out.is_closed()?1:0, std::abs(out.volume()),
                std::chrono::duration<double>(t1-t0).count());

    if (const char* op = std::getenv("SESSION_MC_OUT")) {
        FILE* f = std::fopen(op, "w");
        if (f && !out.vertex.empty()) {
            std::map<size_t, size_t> vk;
            size_t n = 1;
            for (const auto& kv : out.vertex) {
                std::fprintf(f, "v %.6f %.6f %.6f\n", kv.second.x, kv.second.y, kv.second.z);
                vk[kv.first] = n++;
            }
            for (const auto& kv : out.face)
                std::fprintf(f, "f %zu %zu %zu\n", vk[kv.second[0]], vk[kv.second[1]], vk[kv.second[2]]);
            std::fclose(f);
            std::printf("wrote %s\n", op);
        }
    }
    return 0;
}
