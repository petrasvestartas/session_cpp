// main_24 — V2 PERFORMANCE, LIVENESS, MEMORY AND SCALING driver.
//
// Nothing here scores correctness beyond the shared harness src/v2/v2_verdict.h (namespace
// v2v). This driver measures TIME, TERMINATION and SIZE.
//
// Every mode runs ONE thing and exits, so the shell can wrap it in `timeout N` and
// `/usr/bin/time -v` and get a per-operation peak RSS that is not polluted by neighbouring
// cells. A mode that needs many cells (ladder, scale) prints one CSV line per cell and
// flushes, so a run that is killed still yields every cell that completed.
//
// MODES (argv[1]):
//   info                              build banner + operand face counts, no boolean
//   base                              allocate nothing: RSS floor of the process
//   ladder  [nposes]                  8-pair ladder x poses x {cut,common,fuse} x {cur,v2}
//   one     <cur|v2> <pair> <op> <k>  exactly ONE boolean, for /usr/bin/time -v
//   mp      <pair> <k>                mass properties of ONE operand (BRep::volume reference)
//   live    <case> <cur|v2> <op>      one historical liveness case, one op, one kernel
//   scale   <n> <cur|v2> <op>         prism_n x prism_n (n+2 faces each)
//   chairs  <dir> <cur|v2> <op>       the real 20-face STEP models
//
// TIMING METHOD. steady_clock around the boolean call ONLY; the verdict/mass-properties cost
// is timed separately and reported in its own column, because a previous session mistook
// BRep::volume() cost for a boolean hang. In `ladder` and `scale` each cell is run once
// (cold, first call in the process) and then repeated up to REP times while the accumulated
// time is under 400 ms; both the first-call time and the median of the repeats are printed.
//
// CONTROL CELL. Every sweep runs "box x box / cut / cur" first and prints it tagged CONTROL.
// If the control moves between runs, the machine moved, not the kernel.

#include "src/brep.h"
#include "src/brep_massprops.h"
#include "src/file_step.h"
#include "src/mesh.h"
#include "src/nurbscurve.h"
#include "src/polyline.h"
#include "src/v2/brep_v2_boolean.h"
#include "src/v2/v2_verdict.h"
#include "src/xform.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/prctl.h>
#include <unistd.h>
#include <vector>

using namespace session_cpp;

static const char* BANNER = "main_24 perf/liveness/memory/scaling probe v1";

///////////////////////////////////////////////////////////////////////////////////////////
// timing
///////////////////////////////////////////////////////////////////////////////////////////

using clk = std::chrono::steady_clock;

static double ms_since(const clk::time_point& t0) {
    return std::chrono::duration<double, std::milli>(clk::now() - t0).count();
}

// PROCESS CPU TIME. Other agents are building and sweeping on this machine (load average has
// been observed above 60 on 32 cores), so wall time alone is not a property of the kernel.
// CLOCK_PROCESS_CPUTIME_ID is reported next to it in every cell: the kernel is single-threaded,
// so cpu_ms is the load-robust number and wall/cpu is the contention factor.
static double cpu_ms_now() {
    timespec ts{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static std::string loadavg() {
    FILE* f = std::fopen("/proc/loadavg", "r");
    if (!f) return "?";
    char buf[128] = {0};
    if (!std::fgets(buf, sizeof buf, f)) { std::fclose(f); return "?"; }
    std::fclose(f);
    std::string s(buf);
    const size_t nl = s.find('\n');
    if (nl != std::string::npos) s.resize(nl);
    return s;
}

static double median(std::vector<double> v) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

///////////////////////////////////////////////////////////////////////////////////////////
// operands
///////////////////////////////////////////////////////////////////////////////////////////

static BRep moved(const BRep& b, const Xform& x) {
    BRep c = b;
    c.xform = x;
    return c.transformed();
}

// main_16.cpp motion(k), byte for byte: the same poses the acceptance ladder scores.
static Xform motion(int k) {
    if (k == 0) return Xform::identity();
    const double a = 0.7548776662 * k, b = 0.5698402909 * k, c = 0.3141592653 * k;
    Vector axis(std::sin(a * 3.1) + 0.31, std::cos(b * 2.7) - 0.17, std::sin(c * 1.9) + 0.53);
    const double L = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis = Vector(axis[0] / L, axis[1] / L, axis[2] / L);
    const double ang = 0.13 + 3.0 * std::fabs(std::sin(1.7 * k));
    Xform r = Xform::rotation(axis, ang, false);
    Xform t = Xform::translation(0.37 * std::sin(2.1 * k), -0.29 * std::cos(1.3 * k),
                                 0.44 * std::sin(0.9 * k));
    return t * r;
}

struct Pair {
    std::string name;
    BRep A, B;
};

// main_16.cpp build_ladder(), same 8 pairs, same placements.
static std::vector<Pair> ladder() {
    std::vector<Pair> v;
    {
        Pair p;
        p.name = "box_x_box";
        p.A = BRep::create_box(2, 2, 2);
        p.B = moved(BRep::create_box(2, 2, 2), Xform::translation(1.0, 0.5, 0.25));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "sphere_x_sphere";
        p.A = BRep::create_sphere(1.0);
        p.B = moved(BRep::create_sphere(1.0), Xform::translation(1.0, 0, 0));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "sphere_x_cylinder";
        p.A = BRep::create_sphere(2.5);
        p.B = moved(BRep::create_cylinder(1.0, 8.0), Xform::translation(0, 0, -4.0));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "box_x_sphere";
        p.A = BRep::create_box(3, 3, 3);
        p.B = moved(BRep::create_sphere(1.0), Xform::translation(1.5, 0, 0));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "cylinder_x_cylinder";
        p.A = moved(BRep::create_cylinder(1.0, 4.0), Xform::translation(0, 0, -2.0));
        Vector ax(1, 0, 0);
        p.B = moved(moved(BRep::create_cylinder(1.0, 4.0), Xform::translation(0, 0, -2.0)),
                    Xform::rotation(ax, M_PI / 2.0, false));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "box_x_cone";
        p.A = BRep::create_cone(1.0, 2.0);
        p.B = moved(BRep::create_box(4, 4, 4), Xform::translation(0, 0, 3.0));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "cone_x_cone";
        p.A = BRep::create_cone(1.0, 2.0);
        Vector ax(1, 0, 0);
        p.B = moved(moved(BRep::create_cone(1.5, 2.0), Xform::rotation(ax, M_PI, false)),
                    Xform::translation(0, 0, 1.6));
        v.push_back(p);
    }
    {
        Pair p;
        p.name = "torus_x_torus";
        p.A = BRep::create_torus(3.0, 1.0);
        p.B = moved(BRep::create_torus(3.0, 1.0), Xform::translation(0, 0, 1.5));
        v.push_back(p);
    }
    return v;
}

// main_7.cpp placements(), the four entries the historical liveness defects live in.
static Xform xf_of(const std::array<double, 7>& x) {
    Xform t = Xform::translation(x[0], x[1], x[2]);
    if (x[6] == 0.0) return t;
    Xform r = x[3] ? Xform::rotation_x(x[6], true)
              : x[4] ? Xform::rotation_y(x[6], true)
                     : Xform::rotation_z(x[6], true);
    return t * r;
}

static BRep prim_coneR() {   // cone on its side, axis +y
    return moved(BRep::create_cone(2.0, 4.0), xf_of({0, -2.8, 0, 1, 0, 0, -90}));
}
static BRep prim_cyl() {
    return moved(BRep::create_cylinder(1.5, 6.0), xf_of({0, 0, -3, 0, 0, 1, 0}));
}
static BRep prim_box() { return BRep::create_box(4, 4, 4); }
static BRep prim_torR() {   // tilted torus
    return moved(BRep::create_torus(2.0, 0.8), xf_of({0, 0, 0, 1, 0, 0, 45}));
}
static BRep prim_tor() { return BRep::create_torus(2.0, 0.8); }
static BRep prim_tor2() { return moved(BRep::create_torus(2.0, 0.8), xf_of({2, 0, 0, 0, 0, 1, 0})); }

// PRISM FAMILY for the scaling study: a regular n-gon extruded along +Z, built as n+2 planar
// faces (bottom, top, n sides) through BRep::from_polylines. Face count is exactly n+2, the
// geometry is exactly planar, and the volume is closed form, so time-vs-face-count is measured
// with the per-face geometry cost held constant.
static BRep prism(int n, double radius, double h, double z0) {
    std::vector<Point> lo, hi;
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * M_PI * i / n;
        lo.push_back(Point(radius * std::cos(a), radius * std::sin(a), z0));
        hi.push_back(Point(radius * std::cos(a), radius * std::sin(a), z0 + h));
    }
    std::vector<Polyline> pls;
    {
        std::vector<Point> b = lo;
        b.push_back(lo[0]);
        pls.push_back(Polyline(b));
        std::vector<Point> t;
        for (int i = n - 1; i >= 0; --i) t.push_back(hi[i]);
        t.push_back(hi[n - 1]);
        pls.push_back(Polyline(t));
    }
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        std::vector<Point> q{lo[i], lo[j], hi[j], hi[i], lo[i]};
        pls.push_back(Polyline(q));
    }
    return BRep::from_polylines(pls);
}

///////////////////////////////////////////////////////////////////////////////////////////
// one boolean, timed
///////////////////////////////////////////////////////////////////////////////////////////

enum class Op { Cut, Common, Fuse };
static const char* op_name(Op o) { return o == Op::Cut ? "cut" : o == Op::Common ? "common" : "fuse"; }

static Op parse_op(const std::string& s) {
    if (s == "common") return Op::Common;
    if (s == "fuse") return Op::Fuse;
    return Op::Cut;
}

struct Cell {
    double first_ms = -1;      ///< cold call, wall
    double med_ms = -1;        ///< median of the repeats, wall (== first when only one run)
    double first_cpu = -1;     ///< cold call, process CPU
    double med_cpu = -1;       ///< median of the repeats, process CPU
    int reps = 0;
    int faces = -1;
    bool threw = false;
    std::string what;
};

static BRep run_bool(const BRep& A, const BRep& B, Op op, bool v2) {
    if (v2) {
        switch (op) {
            case Op::Cut: return v2sol::v2_cut(A, B);
            case Op::Common: return v2sol::v2_common(A, B);
            case Op::Fuse: return v2sol::v2_fuse(A, B);
        }
    }
    switch (op) {
        case Op::Cut: return A.boolean(B, BRep::BooleanOp::Difference);
        case Op::Common: return A.boolean(B, BRep::BooleanOp::Intersection);
        case Op::Fuse: return A.boolean(B, BRep::BooleanOp::Union);
    }
    return BRep();
}

static Cell timed(const BRep& A, const BRep& B, Op op, bool v2, int max_reps, double budget_ms) {
    Cell c;
    std::vector<double> ts, cs;
    try {
        const double c0 = cpu_ms_now();
        const clk::time_point t0 = clk::now();
        BRep r = run_bool(A, B, op, v2);
        c.first_ms = ms_since(t0);
        c.first_cpu = cpu_ms_now() - c0;
        c.faces = r.face_count();
    } catch (const std::exception& e) {
        c.threw = true;
        c.what = e.what();
        return c;
    } catch (...) {
        c.threw = true;
        c.what = "unknown";
        return c;
    }
    ts.push_back(c.first_ms);
    cs.push_back(c.first_cpu);
    c.reps = 1;
    double acc = c.first_ms;
    while (c.reps < max_reps && acc < budget_ms) {
        const double c0 = cpu_ms_now();
        const clk::time_point t0 = clk::now();
        volatile int f = run_bool(A, B, op, v2).face_count();
        (void)f;
        const double dt = ms_since(t0);
        ts.push_back(dt);
        cs.push_back(cpu_ms_now() - c0);
        acc += dt;
        ++c.reps;
    }
    c.med_ms = median(ts);
    c.med_cpu = median(cs);
    return c;
}

///////////////////////////////////////////////////////////////////////////////////////////

static void hdr(const char* what) {
    std::printf("# %s | %s\n", BANNER, what);
    std::printf("# loadavg at start: %s\n", loadavg().c_str());
    std::printf("#csv pair,pose,op,kernel,first_ms,median_ms,first_cpu_ms,median_cpu_ms,reps,"
                "faces,status\n");
}

static void emit(const char* pair, int pose, Op op, bool v2, const Cell& c) {
    std::printf("%s,%d,%s,%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%s\n", pair, pose, op_name(op),
                v2 ? "v2" : "cur", c.first_ms, c.med_ms, c.first_cpu, c.med_cpu, c.reps, c.faces,
                c.threw ? ("THREW:" + c.what).c_str() : "ok");
    std::fflush(stdout);
}

///////////////////////////////////////////////////////////////////////////////////////////

static int mode_info() {
    std::printf("# %s\n", BANNER);
    for (const Pair& p : ladder())
        std::printf("operand %-20s A_faces=%d B_faces=%d\n", p.name.c_str(), p.A.face_count(),
                    p.B.face_count());
    std::printf("operand %-20s A_faces=%d B_faces=%d\n", "coneR_x_cyl", prim_coneR().face_count(),
                prim_cyl().face_count());
    std::printf("operand %-20s A_faces=%d B_faces=%d\n", "box_x_torR", prim_box().face_count(),
                prim_torR().face_count());
    for (int n : {4, 8, 16, 32, 66, 128}) {
        BRep pr = prism(n, 1.0, 2.0, -1.0);
        const auto v = v2v::v2_verdict_topology_only(pr);
        std::printf("operand prism_%-14d faces=%d naked_real=%d nonmanifold=%d shells=%d\n", n,
                    pr.face_count(), v.naked_real, v.nonmanifold, v.shells);
    }
    return 0;
}

// MESH COST. Stack sampling (gdb; perf is unavailable) put every sample of the slow v2 cells in
//   v2_boolean -> v2_outward_signs -> V2BuilderSolid::perform_areas -> v2_shell_is_hole
//   -> v2_shell_signed_volume -> V2Topo::ensure_meshes -> BRep::face_meshes()
//   -> NurbsSurfaceTrimmed::mesh() -> Delaunay2D
// i.e. v2 triangulates every face with the DISPLAY mesher to get a shell's signed volume. This
// mode times that call directly per operand so the attribution is a measurement, not a stack
// impression: sum over the operands is the floor under any v2 boolean on that pair.
static int mode_mesh() {
    std::printf("# %s | face_meshes() cost per operand\n", BANNER);
    std::printf("# loadavg: %s\n", loadavg().c_str());
    std::printf("#csv operand,faces,mesh_ms,tris\n");
    auto one = [](const char* nm, const BRep& b) {
        const clk::time_point t0 = clk::now();
        const auto ms = b.face_meshes();
        const double dt = ms_since(t0);
        long tris = 0;
        for (const auto& m : ms) tris += (long)m.to_vertices_and_faces().second.size();
        std::printf("%s,%d,%.3f,%ld\n", nm, b.face_count(), dt, tris);
        std::fflush(stdout);
    };
    for (const Pair& p : ladder()) {
        one((p.name + "_A").c_str(), p.A);
        one((p.name + "_B").c_str(), p.B);
    }
    one("prim_coneR", prim_coneR());
    one("prim_cyl", prim_cyl());
    one("prim_torR", prim_torR());
    for (int n : {4, 8, 16, 32, 66, 128})
        one(("prism_" + std::to_string(n)).c_str(), prism(n, 1.0, 4.0, -2.0));
    return 0;
}

// STAGE SPLIT. v2_outward_signs is the only v2 stage callable from outside the driver, and the
// stack samples land inside it, so it is timed on its own against the whole boolean. Everything
// left over is stages 1..8.
static int mode_stages(int pose) {
    std::printf("# %s | v2_outward_signs vs whole boolean\n", BANNER);
    std::printf("# loadavg: %s\n", loadavg().c_str());
    std::printf("#csv pair,pose,signsA_ms,signsB_ms,v2cut_ms,curcut_ms,facesA,facesB\n");
    for (const Pair& p : ladder()) {
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        int sh = 0, ho = 0;
        const clk::time_point t0 = clk::now();
        auto sa = v2sol::v2_outward_signs(A, &sh, &ho);
        const double ta = ms_since(t0);
        const clk::time_point t1 = clk::now();
        auto sb = v2sol::v2_outward_signs(B, &sh, &ho);
        const double tb = ms_since(t1);
        (void)sa; (void)sb;
        const Cell cv = timed(A, B, Op::Cut, true, 1, 0.0);
        const Cell cc = timed(A, B, Op::Cut, false, 1, 0.0);
        std::printf("%s,%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n", p.name.c_str(), pose, ta, tb,
                    cv.first_ms, cc.first_ms, A.face_count(), B.face_count());
        std::fflush(stdout);
    }
    return 0;
}

// SPLIT-FACE MESH COST. The stack samples land in face_meshes() called on the SPLIT operands
// (brep_v2_boolean.cpp:1332-1333, v2_outward_signs(ha.split)), not on the pristine ones. This
// mode reproduces exactly that: imprint A against B with the same public splitter v2 falls back
// to, then time face_meshes() on the imprinted BRep and count its triangles.
static int mode_splitmesh(int pose) {
    std::printf("# %s | face_meshes() on the IMPRINTED operands\n", BANNER);
    std::printf("# loadavg: %s\n", loadavg().c_str());
    std::printf("#csv pair,pose,side,faces_before,faces_after,split_ms,mesh_ms,tris\n");
    for (const Pair& p : ladder()) {
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        for (int side = 0; side < 2; ++side) {
            const BRep& X = side ? B : A;
            const BRep& Y = side ? A : B;
            BRep s;
            const clk::time_point t0 = clk::now();
            try {
                s = X.split_by_brep(Y, 0.0, false);
            } catch (...) {
                std::printf("%s,%d,%c,%d,,SPLIT_THREW,,\n", p.name.c_str(), pose, side ? 'B' : 'A',
                            X.face_count());
                continue;
            }
            const double tsp = ms_since(t0);
            const clk::time_point t1 = clk::now();
            const auto ms = s.face_meshes();
            const double tm = ms_since(t1);
            long tris = 0;
            for (const auto& mm : ms) tris += (long)mm.to_vertices_and_faces().second.size();
            std::printf("%s,%d,%c,%d,%d,%.3f,%.3f,%ld\n", p.name.c_str(), pose, side ? 'B' : 'A',
                        X.face_count(), s.face_count(), tsp, tm, tris);
            std::fflush(stdout);
        }
    }
    return 0;
}

// RESULT MESH COST + TRIM SIZE. gdb breakpoint timing showed 5 BRep::face_meshes_q calls per v2
// boolean and 18 NurbsSurfaceTrimmed::mesh_q calls inside them, each mesh_q taking ~1 s on a BOX
// face -- while face_meshes() on the pristine box costs 0.05 ms for all six. The difference must
// be in the trim loops the imprint leaves behind, so this mode prints, per result, the mesh time
// and the total control-point count of every trim loop.
static long trim_cvs(const BRep& b) {
    long n = 0;
    for (const auto& c : b.m_curves_2d) n += c.cv_count();
    return n;
}

static int mode_resmesh(int pose) {
    std::printf("# %s | mesh cost of the RESULT breps\n", BANNER);
    std::printf("# loadavg: %s\n", loadavg().c_str());
    std::printf("#csv pair,pose,kernel,faces,bool_ms,mesh_ms,tris,trim_cvs\n");
    for (const Pair& p : ladder()) {
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        for (bool v2 : {false, true}) {
            BRep r;
            const clk::time_point t0 = clk::now();
            try {
                r = run_bool(A, B, Op::Cut, v2);
            } catch (...) {
                std::printf("%s,%d,%s,THREW,,,,\n", p.name.c_str(), pose, v2 ? "v2" : "cur");
                continue;
            }
            const double tb = ms_since(t0);
            const clk::time_point t1 = clk::now();
            const auto ms = r.face_meshes();
            const double tm = ms_since(t1);
            long tris = 0;
            for (const auto& mm : ms) tris += (long)mm.to_vertices_and_faces().second.size();
            std::printf("%s,%d,%s,%d,%.3f,%.3f,%ld,%ld\n", p.name.c_str(), pose,
                        v2 ? "v2" : "cur", r.face_count(), tb, tm, tris, trim_cvs(r));
            std::fflush(stdout);
        }
        // the pristine operands, for scale
        std::printf("%s,%d,%s,%d,0,%.3f,,%ld\n", p.name.c_str(), pose, "operandA", A.face_count(),
                    [&] {
                        const clk::time_point t = clk::now();
                        auto m = A.face_meshes();
                        (void)m;
                        return ms_since(t);
                    }(),
                    trim_cvs(A));
        std::fflush(stdout);
    }
    return 0;
}

// COUNTERFACTUAL for the §6 recommendation. v2 gets a shell's signed volume by triangulating with
// the DISPLAY mesher (V2Topo::ensure_meshes -> BRep::face_meshes, brep_v2_solid.cpp:155-158,486).
// brep_massprops::brep_volume computes a signed volume analytically over the same faces and is
// what v2_verdict already uses. This mode times BOTH routes on the SAME imprinted operands -- the
// exact BReps v2 meshes -- so the size of the available win is measured, not asserted.
// Caveat stated in the report: v2 meshes per SHELL subset, this times the whole imprinted BRep;
// it is an order-of-magnitude comparison of the two routes, not a drop-in patch simulation.
static int mode_altvol(int pose) {
    std::printf("# %s | mesh route vs analytic route on the IMPRINTED operands\n", BANNER);
    std::printf("# loadavg: %s\n", loadavg().c_str());
    std::printf("#csv pair,pose,side,faces_in,faces_split,split_ms,mesh_ms,massprops_ms,mp_volume\n");
    for (const Pair& p : ladder()) {
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        for (int side = 0; side < 2; ++side) {
            const BRep& X = side ? B : A;
            const BRep& Y = side ? A : B;
            BRep s;
            const clk::time_point t0 = clk::now();
            try {
                s = X.split_by_brep(Y, 0.0, false);
            } catch (...) {
                std::printf("%s,%d,%c,%d,,SPLIT_THREW,,,\n", p.name.c_str(), pose, side ? 'B' : 'A',
                            X.face_count());
                continue;
            }
            const double tsp = ms_since(t0);
            const clk::time_point t1 = clk::now();
            const auto ms = s.face_meshes();
            const double tm = ms_since(t1);
            (void)ms;
            double vol = 0.0;
            double tv = -1.0;
            const clk::time_point t2 = clk::now();
            try {
                vol = brep_massprops::brep_volume(s);
                tv = ms_since(t2);
            } catch (...) {
                tv = -2.0;
            }
            std::printf("%s,%d,%c,%d,%d,%.3f,%.3f,%.3f,%.9f\n", p.name.c_str(), pose,
                        side ? 'B' : 'A', X.face_count(), s.face_count(), tsp, tm, tv, vol);
            std::fflush(stdout);
        }
    }
    return 0;
}

static int mode_base() {
    std::printf("# %s | base: process floor, no geometry\n", BANNER);
    return 0;
}

static int mode_ladder(int nposes) {
    hdr("ladder: 8 pairs x poses x {cut,common,fuse} x {cur,v2}; boolean call only");
    const std::vector<Pair> ps = ladder();
    // CONTROL: same cell, first, every run.
    {
        const Cell c = timed(ps[0].A, ps[0].B, Op::Cut, false, 20, 400.0);
        std::printf("CONTROL,%d,%s,%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%s\n", 0, "cut", "cur", c.first_ms,
                    c.med_ms, c.first_cpu, c.med_cpu, c.reps, c.faces, c.threw ? "THREW" : "ok");
        std::fflush(stdout);
    }
    for (const Pair& p : ps) {
        for (int k = 0; k < nposes; ++k) {
            const Xform M = motion(k);
            const BRep A = moved(p.A, M), B = moved(p.B, M);
            for (Op op : {Op::Cut, Op::Common, Op::Fuse})
                for (bool v2 : {false, true})
                    emit(p.name.c_str(), k, op, v2, timed(A, B, op, v2, 20, 400.0));
        }
    }
    return 0;
}

// mass-properties / verdict cost on its own, per pair operand and per boolean result.
static int mode_mp(const std::string& pair, int pose) {
    std::printf("# %s | mp\n", BANNER);
    std::printf("#csv pair,pose,what,ms,faces,volume,closed\n");
    for (const Pair& p : ladder()) {
        if (p.name != pair) continue;
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        struct S { const char* n; const BRep* b; };
        for (const S& s : {S{"operandA", &A}, S{"operandB", &B}}) {
            const clk::time_point t0 = clk::now();
            const auto v = v2v::v2_verdict(*s.b);
            const double dt = ms_since(t0);
            std::printf("%s,%d,%s,%.3f,%d,%.9f,%d\n", pair.c_str(), pose, s.n, dt, v.faces,
                        v.volume, (int)v.closed());
        }
        for (Op op : {Op::Cut, Op::Common, Op::Fuse})
            for (bool v2 : {false, true}) {
                BRep r;
                try {
                    r = run_bool(A, B, op, v2);
                } catch (...) {
                    std::printf("%s,%d,%s_%s_result,THREW,,,\n", pair.c_str(), pose, op_name(op),
                                v2 ? "v2" : "cur");
                    continue;
                }
                const clk::time_point t0 = clk::now();
                const auto v = v2v::v2_verdict(r);
                const double dt = ms_since(t0);
                std::printf("%s,%d,verdict_%s_%s,%.3f,%d,%.9f,%d\n", pair.c_str(), pose,
                            op_name(op), v2 ? "v2" : "cur", dt, v.faces, v.volume, (int)v.closed());
                std::fflush(stdout);
            }
    }
    return 0;
}

static int mode_one(const std::string& kern, const std::string& pair, const std::string& ops,
                    int pose) {
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    for (const Pair& p : ladder()) {
        if (p.name != pair) continue;
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        const Cell c = timed(A, B, op, v2, 1, 0.0);
        std::printf("ONE %s pose=%d %s %s first_ms=%.3f cpu_ms=%.3f faces=%d %s load=%s\n",
                    pair.c_str(), pose, op_name(op), v2 ? "v2" : "cur", c.first_ms, c.first_cpu,
                    c.faces, c.threw ? ("THREW:" + c.what).c_str() : "ok", loadavg().c_str());
        return c.threw ? 3 : 0;
    }
    std::printf("ONE unknown pair %s\n", pair.c_str());
    return 2;
}

// SPIN: repeat one cell forever (or n times) so an external sampler (gdb attach; perf is
// unavailable here, perf_event_paranoid=4) can attribute the time. Prints nothing per rep.
static int mode_spin(const std::string& kern, const std::string& pair, const std::string& ops,
                     int pose, int reps) {
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    for (const Pair& p : ladder()) {
        if (p.name != pair) continue;
        const Xform M = motion(pose);
        const BRep A = moved(p.A, M), B = moved(p.B, M);
        // yama ptrace_scope=1 on this box: opt in explicitly so an unrelated gdb can sample us.
        prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
        std::printf("SPIN %s %s %s reps=%d pid=%d\n", pair.c_str(), kern.c_str(), op_name(op),
                    reps, (int)getpid());
        std::fflush(stdout);
        const clk::time_point t0 = clk::now();
        for (int i = 0; i < reps; ++i) {
            volatile int f = run_bool(A, B, op, v2).face_count();
            (void)f;
        }
        std::printf("SPIN done total_ms=%.1f per_rep_ms=%.3f\n", ms_since(t0),
                    ms_since(t0) / reps);
        return 0;
    }
    return 2;
}

static bool live_operands(const std::string& cs, BRep& A, BRep& B) {
    if (cs == "coneRxcyl") { A = prim_coneR(); B = prim_cyl(); return true; }
    if (cs == "boxxtorR") { A = prim_box(); B = prim_torR(); return true; }
    if (cs == "torxtor") { A = prim_tor(); B = prim_tor2(); return true; }
    if (cs == "cylxconeR") { A = prim_cyl(); B = prim_coneR(); return true; }
    if (cs.rfind("tilt", 0) == 0) {   // tilt<deg>: sphere r2.5 x cylinder r1 through centre
        const double deg = std::atof(cs.c_str() + 4);
        A = BRep::create_sphere(2.5);
        Vector ay(0, 1, 0);
        B = moved(moved(BRep::create_cylinder(1.0, 8.0), Xform::translation(0, 0, -4.0)),
                  Xform::rotation(ay, deg * M_PI / 180.0, false));
        return true;
    }
    return false;
}

static int mode_live(const std::string& cs, const std::string& kern, const std::string& ops) {
    BRep A, B;
    if (!live_operands(cs, A, B)) {
        std::printf("LIVE unknown case %s\n", cs.c_str());
        return 2;
    }
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    std::printf("LIVE %s %s %s START A_faces=%d B_faces=%d load=%s\n", cs.c_str(), kern.c_str(),
                op_name(op), A.face_count(), B.face_count(), loadavg().c_str());
    std::fflush(stdout);
    const Cell c = timed(A, B, op, v2, 1, 0.0);
    std::printf("LIVE %s %s %s DONE ms=%.3f cpu_ms=%.3f faces=%d %s\n", cs.c_str(), kern.c_str(),
                op_name(op), c.first_ms, c.first_cpu, c.faces,
                c.threw ? ("THREW:" + c.what).c_str() : "ok");
    std::fflush(stdout);
    return c.threw ? 3 : 0;
}

// mass-properties timing of one live-case operand or result, to separate a boolean hang from a
// BRep::volume() hang (the box x torus / torus x torus historical confusion).
static int mode_livemp(const std::string& cs, const std::string& kern, const std::string& ops) {
    BRep A, B;
    if (!live_operands(cs, A, B)) return 2;
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    BRep r;
    const clk::time_point tb = clk::now();
    try {
        r = run_bool(A, B, op, v2);
    } catch (const std::exception& e) {
        std::printf("LIVEMP %s %s %s boolean THREW %s after %.3f ms\n", cs.c_str(), kern.c_str(),
                    op_name(op), e.what(), ms_since(tb));
        return 3;
    }
    const double tbool = ms_since(tb);
    std::printf("LIVEMP %s %s %s boolean_ms=%.3f faces=%d\n", cs.c_str(), kern.c_str(),
                op_name(op), tbool, r.face_count());
    std::fflush(stdout);
    const clk::time_point tv = clk::now();
    const auto v = v2v::v2_verdict(r);
    std::printf("LIVEMP %s %s %s verdict_ms=%.3f vol=%.9f closed=%d naked=%d\n", cs.c_str(),
                kern.c_str(), op_name(op), ms_since(tv), v.volume, (int)v.closed(), v.naked_real);
    std::fflush(stdout);
    const clk::time_point tw = clk::now();
    double vol = 0.0;
    try {
        vol = r.volume();
    } catch (...) {
        std::printf("LIVEMP BRep::volume THREW after %.3f ms\n", ms_since(tw));
        return 3;
    }
    std::printf("LIVEMP %s %s %s brep_volume_ms=%.3f vol=%.9f\n", cs.c_str(), kern.c_str(),
                op_name(op), ms_since(tw), vol);
    return 0;
}

static int mode_scale(int n, const std::string& kern, const std::string& ops) {
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    // two interpenetrating prisms, second rotated 90 deg about X so the section is a genuine
    // lateral-lateral arrangement rather than a coplanar stack.
    BRep A = prism(n, 1.0, 4.0, -2.0);
    Vector ax(1, 0, 0);
    BRep B = moved(prism(n, 1.0, 4.0, -2.0), Xform::rotation(ax, M_PI / 2.0, false));
    const Cell c = timed(A, B, op, v2, 3, 400.0);
    std::printf("SCALE n=%d faces=%d+%d %s %s first_ms=%.3f median_ms=%.3f first_cpu=%.3f "
                "median_cpu=%.3f reps=%d result_faces=%d %s load=%s\n",
                n, A.face_count(), B.face_count(), kern.c_str(), op_name(op), c.first_ms,
                c.med_ms, c.first_cpu, c.med_cpu, c.reps, c.faces,
                c.threw ? ("THREW:" + c.what).c_str() : "ok", loadavg().c_str());
    std::fflush(stdout);
    return c.threw ? 3 : 0;
}

// BROAD-PHASE ISOLATION. Same prisms, moved far apart so NOTHING intersects: every candidate
// pair is rejected by its bounding box and no section, split or classification work is done.
// The remaining time is the O(n^2) candidate-pair generation plus fixed overhead, which is what
// "the arena has no BVH" costs today.
static int mode_scaledisj(int n, const std::string& kern, const std::string& ops) {
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    BRep A = prism(n, 1.0, 4.0, -2.0);
    BRep B = moved(prism(n, 1.0, 4.0, -2.0), Xform::translation(100.0, 0, 0));
    const Cell c = timed(A, B, op, v2, 5, 400.0);
    std::printf("DISJ n=%d faces=%d+%d %s %s first_ms=%.3f median_ms=%.3f median_cpu=%.3f "
                "reps=%d result_faces=%d %s load=%s\n",
                n, A.face_count(), B.face_count(), kern.c_str(), op_name(op), c.first_ms,
                c.med_ms, c.med_cpu, c.reps, c.faces,
                c.threw ? ("THREW:" + c.what).c_str() : "ok", loadavg().c_str());
    return c.threw ? 3 : 0;
}

static int mode_chairs(const std::string& dir, const std::string& kern, const std::string& ops) {
    const bool v2 = (kern == "v2");
    const Op op = parse_op(ops);
    const clk::time_point tr = clk::now();
    auto as = file_step::read_file_step_breps(dir + "/chair0.stp");
    auto bs = file_step::read_file_step_breps(dir + "/chair1.stp");
    const double tread = ms_since(tr);
    if (as.empty() || bs.empty()) {
        std::printf("CHAIRS read failed (%zu,%zu)\n", as.size(), bs.size());
        return 2;
    }
    std::printf("CHAIRS read_ms=%.3f A_faces=%d B_faces=%d\n", tread, as[0].face_count(),
                bs[0].face_count());
    std::fflush(stdout);
    const Cell c = timed(as[0], bs[0], op, v2, 1, 0.0);
    std::printf("CHAIRS %s %s first_ms=%.3f cpu_ms=%.3f faces=%d %s load=%s\n", kern.c_str(),
                op_name(op), c.first_ms, c.first_cpu, c.faces,
                c.threw ? ("THREW:" + c.what).c_str() : "ok", loadavg().c_str());
    return c.threw ? 3 : 0;
}

///////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // yama ptrace_scope=1 and perf_event_paranoid=4 on this machine: gdb attach is the only
    // available sampler and it needs this opt-in.
    prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
    const std::string m = argc > 1 ? argv[1] : "info";
    if (m == "info") return mode_info();
    if (m == "base") return mode_base();
    if (m == "mesh") return mode_mesh();
    if (m == "resmesh") return mode_resmesh(argc > 2 ? std::atoi(argv[2]) : 0);
    if (m == "splitmesh") return mode_splitmesh(argc > 2 ? std::atoi(argv[2]) : 0);
    if (m == "altvol") return mode_altvol(argc > 2 ? std::atoi(argv[2]) : 0);
    if (m == "stages") return mode_stages(argc > 2 ? std::atoi(argv[2]) : 0);
    if (m == "ladder") return mode_ladder(argc > 2 ? std::atoi(argv[2]) : 3);
    if (m == "one" && argc > 5) return mode_one(argv[2], argv[3], argv[4], std::atoi(argv[5]));
    if (m == "spin" && argc > 6)
        return mode_spin(argv[2], argv[3], argv[4], std::atoi(argv[5]), std::atoi(argv[6]));
    if (m == "mp" && argc > 3) return mode_mp(argv[2], std::atoi(argv[3]));
    if (m == "live" && argc > 4) return mode_live(argv[2], argv[3], argv[4]);
    if (m == "livemp" && argc > 4) return mode_livemp(argv[2], argv[3], argv[4]);
    if (m == "scale" && argc > 4) return mode_scale(std::atoi(argv[2]), argv[3], argv[4]);
    if (m == "scaledisj" && argc > 4)
        return mode_scaledisj(std::atoi(argv[2]), argv[3], argv[4]);
    if (m == "chairs" && argc > 4) return mode_chairs(argv[2], argv[3], argv[4]);
    std::printf("usage: main_24 info|base|ladder N|one K PAIR OP POSE|mp PAIR POSE|"
                "live CASE K OP|livemp CASE K OP|scale N K OP|chairs DIR K OP\n");
    return 1;
}
