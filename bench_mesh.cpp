#include "nurbssurface.h"
#include "nurbscurve.h"
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
