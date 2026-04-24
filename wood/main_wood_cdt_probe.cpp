#include "../src/mesh.h"
#include <iostream>
#include <vector>
#include <utility>
namespace session_cpp {
std::vector<std::array<int,3>> cdt_triangulate(
    const std::vector<std::pair<double,double>>&,
    const std::vector<std::vector<std::pair<double,double>>>&);
}

int main() {
    using namespace session_cpp;
    std::vector<std::pair<double,double>> outer = {
        {-1172.487, -530.170},
        { 318.768,  -530.170},
        { 318.768,  -318.102},
        { 414.110,  -347.792},
        { 414.110,  -106.034},
        { 318.768,  -135.724},
        { 318.768,   106.034},
        { 414.110,    76.344},
        { 414.110,   318.102},
        { 318.768,   288.412},
        { 318.768,   530.170},
        {-1172.487,  530.170},
    };
    std::vector<std::vector<std::pair<double,double>>> holes = {
        {{-1006.792,  97.448},{-1006.792,  0.0},{-841.097,  0.0},{-841.097,  97.448}},
        {{-344.012,   97.448},{-344.012,   0.0},{-178.317,  0.0},{-178.317,  97.448}},
    };
    auto t1 = cdt_triangulate(outer, holes);
    std::cout << "with holes: " << t1.size() << "\n";
    auto t2 = cdt_triangulate(outer, {});
    std::cout << "outer only: " << t2.size() << "\n";
    auto t3 = cdt_triangulate(outer, {holes[0]});
    std::cout << "outer + 1 hole: " << t3.size() << "\n";
    return 0;
}
