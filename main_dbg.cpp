#include "brep.h"
#include "primitives.h"
#include <cstdio>
using namespace session_cpp;
int main() {
    struct P { const char* n; BRep b; } ps[] = {{"cyl", BRep::create_cylinder(1,2)}, {"sph", BRep::create_sphere(1)}, {"cone", BRep::create_cone(1,2)}, {"pyr", BRep::create_pyramid(2,1)}, {"tor", BRep::create_torus(2,0.5)}, {"blk", BRep::create_block_with_hole(4,4,2,1)}};
    for (auto& p : ps) { double w = p.b.update_tolerances(); std::printf("%s worst=%.3e:", p.n, w); for (auto& e : p.b.m_edges) std::printf(" %.2e", e.tolerance); std::printf("\n"); }
    NurbsSurface s = Primitives::sphere_surface(0,0,0,1);
    auto [u0,u1] = s.domain(0); auto [v0,v1] = s.domain(1);
    NurbsCurve c = s.iso_curve(1, u0); auto [d0,d1] = c.domain();
    Point a = c.point_at(d0), m = c.point_at(0.5*(d0+d1)), z = c.point_at(d1);
    std::printf("sphere dom u[%g,%g] v[%g,%g] iso(1,u0) dom[%g,%g] start(%g,%g,%g) mid(%g,%g,%g) end(%g,%g,%g)\n", u0,u1,v0,v1,d0,d1,a[0],a[1],a[2],m[0],m[1],m[2],z[0],z[1],z[2]);
    NurbsSurface t = Primitives::torus_surface(0,0,0,2,0.5);
    auto [tu0,tu1] = t.domain(0); auto [tv0,tv1] = t.domain(1);
    NurbsCurve cu = t.iso_curve(1, tu0), cv = t.iso_curve(0, tv0);
    Point cu0 = cu.point_at(cu.domain().first), cum = cu.point_at(0.5*(cu.domain().first+cu.domain().second)), cv0 = cv.point_at(cv.domain().first), cvm = cv.point_at(0.5*(cv.domain().first+cv.domain().second));
    std::printf("torus iso(1,u0): start(%g,%g,%g) mid(%g,%g,%g) | iso(0,v0): start(%g,%g,%g) mid(%g,%g,%g) | corner(%g,%g,%g)\n", cu0[0],cu0[1],cu0[2],cum[0],cum[1],cum[2],cv0[0],cv0[1],cv0[2],cvm[0],cvm[1],cvm[2], t.point_at_corner(0,0)[0], t.point_at_corner(0,0)[1], t.point_at_corner(0,0)[2]);
    return 0;
}
