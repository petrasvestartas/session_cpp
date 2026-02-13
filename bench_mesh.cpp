#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "trimesh_grid.h"
#include "trimesh_delaunay.h"
#include <chrono>
#include <cmath>
#include <iostream>

using namespace session_cpp;

int main() {
    const double PI = 3.14159265358979323846;

    // Build all test surfaces first, then time meshing separately
    struct TestCase { std::string name; NurbsSurface srf; };
    std::vector<TestCase> cases;

    // 1. Freeform 4×4
    {
        std::vector<Point> pts = {
            Point(0,0,0), Point(-1,0.75,2), Point(-1,4.25,2), Point(0,5,0),
            Point(0.75,-1,2), Point(1.25,1.25,4), Point(1.25,3.75,4), Point(0.75,6,2),
            Point(4.25,-1,2), Point(3.75,1.25,4), Point(3.75,3.75,4), Point(4.25,6,2),
            Point(5,0,0), Point(6,0.75,2), Point(6,4.25,2), Point(5,5,0),
        };
        cases.push_back({"freeform_4x4", NurbsSurface::create(false, false, 3, 3, 4, 4, pts)});
    }

    // 2. Irregular 10×10
    {
        int n = 10;
        std::vector<Point> pts;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                double x = i, y = j, z = 0;
                z += 5.0 * exp(-((x-1.5)*(x-1.5)+(y-7.5)*(y-7.5))/0.8);
                z -= 4.0 * exp(-((x-7.5)*(x-7.5)+(y-1.5)*(y-1.5))/1.0);
                z += 2.0*((x-4.5)/4.5)*((x-4.5)/4.5) - 2.0*((y-4.5)/4.5)*((y-4.5)/4.5);
                z += 1.5*sin(PI*x/3)*cos(PI*y/4.5);
                z += sin(PI*(x+y)/6);
                z += 3.0*exp(-((x-3)*(x-3)+(y-3)*(y-3))/0.4);
                z -= 2.0*exp(-((x-6)*(x-6)+(y-7)*(y-7))/2.0);
                pts.push_back(Point(x, y, z));
            }
        cases.push_back({"irregular_10x10", NurbsSurface::create(false, false, 3, 3, n, n, pts)});
    }

    // 3. Dense 15×15
    {
        int n = 15;
        std::vector<Point> pts;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                double x = i * 0.7, y = j * 0.7, z = 0;
                z += 3.0 * sin(PI*x/3) * cos(PI*y/4);
                z += 2.0 * exp(-((x-3)*(x-3)+(y-5)*(y-5))/1.5);
                z -= 1.5 * exp(-((x-7)*(x-7)+(y-2)*(y-2))/0.8);
                pts.push_back(Point(x, y, z));
            }
        cases.push_back({"wavy_15x15", NurbsSurface::create(false, false, 3, 3, n, n, pts)});
    }

    // 4. Dense 20×20
    {
        int n = 20;
        std::vector<Point> pts;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                double x = i * 0.5, y = j * 0.5, z = 0;
                z += 2.0 * sin(PI*x/3) * sin(PI*y/3);
                z += 1.5 * cos(PI*(x+y)/5);
                pts.push_back(Point(x, y, z));
            }
        cases.push_back({"wave_20x20", NurbsSurface::create(false, false, 3, 3, n, n, pts)});
    }

    // 5. Revolve test cases
    {
        auto pa = NurbsCurve::create(false, 3, {
            Point(1.5,0,0), Point(1.5,0,0.3), Point(0.3,0,0.5),
            Point(0.3,0,2.5), Point(0.2,0,3.0), Point(2.0,0,4.5), Point(1.8,0,5.0)});
        cases.push_back({"vase_revolve", Primitives::create_revolve(pa, Point(0,0,0), Vector(0,0,1))});
    }
    {
        NurbsCurve pb(3, true, 3, 9);
        double w = std::sqrt(2.0) / 2.0;
        double cw[] = {1,w,1,w,1,w,1,w,1};
        double ca[] = {1,1,0,-1,-1,-1,0,1,1};
        double sa[] = {0,1,1,1,0,-1,-1,-1,0};
        double ck[] = {0,0,1,1,2,2,3,3,4,4};
        double R=5.0, r=1.5, tcx=14;
        for (int i=0; i<10; i++) pb.set_knot(i, ck[i]);
        for (int i=0; i<9; i++) pb.set_cv_4d(i, (tcx+R+r*ca[i])*cw[i], 0, r*sa[i]*cw[i], cw[i]);
        cases.push_back({"torus_revolve", Primitives::create_revolve(pb, Point(tcx,0,0), Vector(0,0,1))});
    }
    {
        auto pc = NurbsCurve::create(false, 1, {Point(29,0,-0.5), Point(29,0,0.5)});
        cases.push_back({"elbow_revolve", Primitives::create_revolve(pc, Point(26,0,0), Vector(0,0,1), PI/2.0)});
    }
    {
        double w = std::sqrt(2.0) / 2.0;
        double sr=2.0, scx=36;
        NurbsCurve pd(3, true, 3, 5);
        double sk[] = {0,0,1,1,2,2};
        for (int i=0; i<6; i++) pd.set_knot(i, sk[i]);
        double spx[] = {0,sr,sr,sr,0}, spz[] = {-sr,-sr,0,sr,sr}, spw[] = {1,w,1,w,1};
        for (int i=0; i<5; i++) pd.set_cv_4d(i, (scx+spx[i])*spw[i], 0, spz[i]*spw[i], spw[i]);
        cases.push_back({"sphere_revolve", Primitives::create_revolve(pd, Point(scx,0,0), Vector(0,0,1))});
    }
    {
        auto pe = NurbsCurve::create(false, 1, {Point(44,0,3), Point(46,0,0)});
        cases.push_back({"cone_revolve", Primitives::create_revolve(pe, Point(44,0,0), Vector(0,0,1))});
    }
    {
        NurbsCurve rail = NurbsCurve::create(false, 2, {Point(0,0,0), Point(0,5,0), Point(2,9,0)});
        NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
        cases.push_back({"sweep1_circle", Primitives::create_sweep1(rail, profile)});
    }
    {
        auto c2 = Primitives::circle(24, 0, 0, 3.0);
        cases.push_back({"extrusion_circle", Primitives::create_extrusion(c2, Vector(0,0,5))});
    }

    // Benchmark each
    printf("%-25s %8s %8s %8s %12s\n", "Surface", "Verts", "Faces", "Iters", "Time(ms)");
    printf("%-25s %8s %8s %8s %12s\n", "-------", "-----", "-----", "-----", "--------");

    for (auto& tc : cases) {
        // Warm up evaluation cache
        tc.srf.point_at(0.5, 0.5);
        tc.srf.normal_at(0.5, 0.5);

        auto t0 = std::chrono::high_resolution_clock::now();
        TrimeshGrid tri(tc.srf);
        Mesh m = tri.mesh();
        auto t1 = std::chrono::high_resolution_clock::now();

        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        printf("%-25s %8d %8d %8s %10.1f ms\n",
               tc.name.c_str(),
               (int)m.number_of_vertices(),
               (int)m.number_of_faces(),
               "-",
               ms);
    }

    return 0;
}
