#define _USE_MATH_DEFINES
#include "polyline.h"
#include "boolean_polyline.h"
#include "clipper2/clipper.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace session_cpp;
namespace C2 = Clipper2Lib;

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
    int s = 0; for (auto& p : ps) s += (int)p.size(); return s;
}

int main() {
    const int iters_small  = 50000;
    const int iters_med    = 5000;
    const int iters_large  = 500;
    const int iters_xlarge = 50;
    const char* op_names[] = {"intersect", "union", "difference"};
    const C2::ClipType c2ops[] = {C2::ClipType::Intersection, C2::ClipType::Union, C2::ClipType::Difference};

    std::printf("%-12s  %4s  %9s %9s %7s  %9s %9s %7s  out\n",
        "operation", "N", "full(ms)", "c2_full", "ratio", "engine", "c2_eng", "ratio");
    std::printf("%s\n", std::string(88, '-').c_str());

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
            BooleanPolyline::compute_count(a, b, op);
            { C2::Clipper64 cl; cl.AddSubject({pa}); cl.AddClip({pb});
              C2::Paths64 r; cl.Execute(c2ops[op], C2::FillRule::NonZero, r); }

            // 1. Full pipeline (our wrapper with output)
            volatile int sink1 = 0;
            auto t0 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < iters; i++) {
                auto res = Polyline::boolean_op(a, b, op);
                for (auto& pl : res) sink1 += (int)pl.get_points().size();
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            double full_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            // 2. Full Clipper2 pipeline (with output)
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
            double c2_full_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

            // 3. Engine only (our Vatti, no output Polyline construction)
            volatile int sink3 = 0;
            auto t4 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < iters; i++)
                sink3 += BooleanPolyline::compute_count(a, b, op);
            auto t5 = std::chrono::high_resolution_clock::now();
            double eng_ms = std::chrono::duration<double, std::milli>(t5 - t4).count();

            // 4. Clipper2 engine only (AddPaths + Execute, skip BuildPaths)
            volatile int sink4 = 0;
            auto t6 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < iters; i++) {
                C2::Clipper64 cl;
                cl.AddSubject({pa}); cl.AddClip({pb});
                C2::Paths64 r; cl.Execute(c2ops[op], C2::FillRule::NonZero, r);
                sink4 += count_pts(r);
            }
            auto t7 = std::chrono::high_resolution_clock::now();
            double c2_eng_ms = std::chrono::duration<double, std::milli>(t7 - t6).count();

            int out_pts = BooleanPolyline::compute_count(a, b, op);
            (void)sink1; (void)sink2; (void)sink3; (void)sink4;
            double r1 = c2_full_ms > 0 ? full_ms / c2_full_ms : 0;
            double r2 = c2_eng_ms > 0 ? eng_ms / c2_eng_ms : 0;
            std::printf("%-12s  %4d  %8.1f %8.1f %6.2fx  %8.1f %8.1f %6.2fx  %d\n",
                op_names[op], n, full_ms, c2_full_ms, r1, eng_ms, c2_eng_ms, r2, out_pts);
        }
        std::printf("\n");
    }
    return 0;
}
