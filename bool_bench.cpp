#define _USE_MATH_DEFINES
#include "polyline.h"
#include "clipper2/clipper.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace session_cpp;
namespace C2 = Clipper2Lib;

// Generate a regular convex N-gon centered at (cx,cy) with radius r
static Polyline make_ngon(int n, double cx, double cy, double r) {
    std::vector<Point> pts;
    pts.reserve(n + 1);
    for (int i = 0; i < n; i++) {
        double a = 2.0 * M_PI * i / n;
        pts.push_back(Point(cx + r * std::cos(a), cy + r * std::sin(a), 0.0));
    }
    pts.push_back(pts[0]);
    return Polyline(pts);
}

static C2::Path64 to_path64(const Polyline& pl) {
    static const double S = 1e9;
    auto pts = pl.get_points();
    int n = (int)pts.size() - 1;
    if (n < 3) n = (int)pts.size();
    C2::Path64 p;
    p.reserve(n);
    for (int i = 0; i < n; i++)
        p.push_back(C2::Point64((int64_t)(pts[i][0] * S), (int64_t)(pts[i][1] * S)));
    return p;
}

static int count_pts(const C2::Paths64& ps) {
    int s = 0;
    for (auto& p : ps) s += (int)p.size();
    return s;
}

int main() {
    const int iters_small  = 50000;
    const int iters_med    = 5000;
    const int iters_large  = 500;
    const int iters_xlarge = 50;

    const char* op_names[] = {"intersect", "union", "difference"};

    std::printf("%-12s  %6s  %10s  %10s  %8s\n",
        "operation", "N", "wrapper(ms)", "raw_c2(ms)", "overhead");
    std::printf("%s\n", std::string(56, '-').c_str());

    for (int op = 0; op < 3; op++) {
        int cases[][2] = {{10,iters_small},{50,iters_med},{100,iters_med},
                          {500,iters_large},{1000,iters_large},{5000,iters_xlarge}};
        for (auto& c : cases) {
            int n = c[0], iters = c[1];
            Polyline a = make_ngon(n, 0.0, 0.0, 1.0);
            Polyline b = make_ngon(n, 0.8, 0.0, 1.0);
            C2::Path64 pa = to_path64(a);
            C2::Path64 pb = to_path64(b);

            // Warm up
            Polyline::boolean_op(a, b, op);
            {
                C2::Paths64 s{pa}, cl{pb};
                if (op == 0) C2::Intersect(s, cl, C2::FillRule::NonZero);
                else if (op == 1) C2::Union(s, cl, C2::FillRule::NonZero);
                else C2::Difference(s, cl, C2::FillRule::NonZero);
            }

            // Bench wrapper (boolean_op: 3D→2D + Clipper2 + back-project)
            volatile int sink1 = 0;
            auto t0 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < iters; i++) {
                auto res = Polyline::boolean_op(a, b, op);
                for (auto& pl : res) sink1 += (int)pl.get_points().size();
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            double wrap_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            // Bench raw Clipper2 (pre-built paths, clip only)
            volatile int sink2 = 0;
            auto t2 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < iters; i++) {
                C2::Paths64 s{pa}, cl{pb};
                C2::Paths64 res;
                if      (op == 0) res = C2::Intersect (s, cl, C2::FillRule::NonZero);
                else if (op == 1) res = C2::Union     (s, cl, C2::FillRule::NonZero);
                else              res = C2::Difference(s, cl, C2::FillRule::NonZero);
                sink2 += count_pts(res);
            }
            auto t3 = std::chrono::high_resolution_clock::now();
            double raw_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

            double ratio = (raw_ms > 0) ? wrap_ms / raw_ms : 0.0;
            // output size from last iteration
            auto res_check = Polyline::boolean_op(a, b, op);
            int out_pts = 0; for (auto& pl : res_check) out_pts += pl.point_count();
            (void)sink1; (void)sink2;
            std::printf("%-12s  %6d  %10.2f  %10.2f  %7.2fx  out=%d\n",
                op_names[op], n, wrap_ms, raw_ms, ratio, out_pts);
        }
        std::printf("\n");
    }
    return 0;
}
