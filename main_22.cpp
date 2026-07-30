// main_22 — TIER 3a: THE FREEFORM DIFFICULTY LADDER, BOTH KERNELS.
//
// One question: at which rung of the freeform ladder does each kernel FIRST break?
//
//   L0  box x box                       planar control — must pass
//   L1  freeform, NO seam NO pole       (multi-face bulged patches) x box
//   L2  freeform WITH a seam, no poles  (closed tube + 2 caps) x box
//   L3  sphere topology, seam + poles, as MULTIPLE faces x box
//   L4  ONE periodic face, seam + 2 poles (the shipped blob) x box
//   L5  freeform x freeform
//
// Every verdict comes from the ONE shared harness src/v2/v2_verdict.h (namespace v2v),
// validated by main_17. This driver computes no verdict of its own.
//
// TWO OPERAND ROUTES, on purpose. A rung can fail because the STEP READER destroyed the
// operand or because the BOOLEAN is wrong, and those need different fixes. So every operand
// can be either
//     own:<name>   built IN CODE by this kernel — the reader is not in the path at all
//     step:<path>  read from a FreeCAD/OCCT-authored STEP file
// and `probe` prints the operand's own verdict so a bad operand is never scored as a bad
// boolean.
//
// MODES (argv):
//   main_22 probe  <spec>                          one operand, one verdict line
//   main_22 export <spec> <out.step>               write the operand for FreeCAD/OCCT checking
//   main_22 cell   <v1|v2> <op> <specA> <specB>    one boolean, one verdict line
//
// SPEC     own:box | own:box:4,4,6 | own:pillow | own:blob | own:sphere | own:cyl | own:tube
//          step:/abs/path.step[#i]        (BRep i of the file, default 0)
//          any spec takes an optional @tx,ty,tz translation suffix.
// OP       cut | common | fuse
//
// One cell per process so the sweep can wrap each in `timeout`; every completed cell ends with
// a T3DONE line, so "attempted vs completed" is countable from the log.

#include "src/brep.h"
#include "src/brep_massprops.h"
#include "src/file_step.h"
#include "src/intersection.h"
#include "src/v2/brep_v2_boolean.h"
#include "src/v2/v2_verdict.h"
#include "src/xform.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;
using session_cpp::v2v::V2Verdict;
using session_cpp::v2v::v2_verdict;

static const char* env_or(const char* k, const char* dflt) {
    const char* p = std::getenv(k);
    return (p && p[0]) ? p : dflt;   // getenv()!=nullptr is TRUE for an EMPTY value
}

static double secs_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

/// Same raised quadrature budget main_21 measured to be necessary on trimmed multi-face solids:
/// at the harness default an intact operand can integrate 3.6% off with converged=0, which
/// would measure the quadrature and not the kernel. Nothing else about the harness changes.
static MassPropsOptions score_options() {
    MassPropsOptions o = v2v::v2_verdict_options();
    long long mult = 100;
    const char* p = std::getenv("SESSION_T3_BUDGET");
    if (p && p[0]) mult = std::atoll(p);
    if (mult < 1) mult = 1;
    o.max_surface_evals *= mult;
    o.min_face_evals *= mult;
    return o;
}

///////////////////////////////////////////////////////////////////////////////////////////
// operands built IN CODE (no STEP reader in the path)
///////////////////////////////////////////////////////////////////////////////////////////

/// L4 — the shipped case: BRep::create_sphere(2.5) with its INTERIOR control points scaled by
/// 1 + 0.12*sin(2.1 i + 1.3 j). One periodic face, one seam, two poles. Byte-for-byte the
/// recipe in main_7.cpp:1457 (SESSION_FREEFORM), so the numbers are comparable to the shipped
/// freeform_common_box cell.
static BRep make_blob(double amp) {
    BRep ff = BRep::create_sphere(2.5);
    NurbsSurface s = ff.m_surfaces[0];
    const int nu = s.cv_count(0), nv = s.cv_count(1);
    for (int i = 1; i + 1 < nu; ++i)
        for (int j = 1; j + 1 < nv; ++j) {
            double x, y, z, w;
            if (!s.get_cv_4d(i, j, x, y, z, w)) continue;
            const double f = 1.0 + amp * std::sin(2.1 * i + 1.3 * j);
            s.set_cv_4d(i, j, x * f, y * f, z * f, w);
        }
    ff.m_surfaces[0] = s;
    ff.name = "blob";
    return ff;
}

/// L1 — the pillow: a box whose 6 planar faces become bulged bicubic patches. The bulge
/// sin(pi u) sin(pi v) is ZERO on every boundary and the boundary CVs sample the original
/// plane, so all 12 edges stay exactly straight and shared: no seam, no pole, still watertight.
/// The bulge is along the OUTWARD direction (face centre minus box centre), so the solid is
/// symmetric under sign flip — an asymmetric result is then provably algorithmic.
static BRep make_pillow(double sx, double sy, double sz, double amp) {
    BRep pil = BRep::create_box(sx, sy, sz);
    const double PI_ = 3.14159265358979323846;
    // box centre (create_box is centred on the origin in this kernel; derive it anyway)
    double cx = 0, cy = 0, cz = 0;
    {
        int n = 0;
        for (const auto& S : pil.m_surfaces) {
            auto du = S.domain(0), dv = S.domain(1);
            Point q = S.point_at(0.5 * (du.first + du.second), 0.5 * (dv.first + dv.second));
            cx += q[0]; cy += q[1]; cz += q[2]; ++n;
        }
        if (n) { cx /= n; cy /= n; cz /= n; }
    }
    for (size_t sf = 0; sf < pil.m_surfaces.size(); ++sf) {
        NurbsSurface& S0 = pil.m_surfaces[sf];
        auto du = S0.domain(0), dv = S0.domain(1);
        Point ctr = S0.point_at(0.5 * (du.first + du.second), 0.5 * (dv.first + dv.second));
        double ox = ctr[0] - cx, oy = ctr[1] - cy, oz = ctr[2] - cz;
        const double L = std::sqrt(ox * ox + oy * oy + oz * oz);
        if (L < 1e-12) continue;
        ox /= L; oy /= L; oz /= L;
        const int N = 6;
        std::vector<std::vector<Point>> grid(N, std::vector<Point>(N, Point(0, 0, 0)));
        std::vector<std::vector<double>> wts(N, std::vector<double>(N, 1.0));
        for (int iv = 0; iv < N; ++iv)
            for (int iu = 0; iu < N; ++iu) {
                const double fu = (double)iu / (N - 1), fv = (double)iv / (N - 1);
                Point q = S0.point_at(du.first + (du.second - du.first) * fu,
                                      dv.first + (dv.second - dv.first) * fv);
                const double h = amp * std::sin(PI_ * fu) * std::sin(PI_ * fv);
                grid[iv][iu] = Point(q[0] + ox * h, q[1] + oy * h, q[2] + oz * h);
            }
        const double a = du.first, b = du.second, c = dv.first, d = dv.second;
        std::vector<double> KU = {a, a + (b - a) / 3, a + 2 * (b - a) / 3, b};
        std::vector<double> KV = {c, c + (d - c) / 3, c + 2 * (d - c) / 3, d};
        std::vector<int> MU = {4, 1, 1, 4}, MV = {4, 1, 1, 4};
        NurbsSurface S1 = NurbsSurface::create_from_parameters(grid, wts, KU, KV, MU, MV, 3, 3);
        if (S1.is_valid()) pil.m_surfaces[sf] = S1;
    }
    pil.name = "pillow";
    return pil;
}

/// L2 — a freeform tube: BRep::create_cylinder's lateral surface refined in the AXIAL direction
/// and its interior rows pushed radially, leaving both boundary rows (the two cap circles)
/// exactly where they were, so the caps still fit and the solid stays watertight. The wrap
/// direction is untouched, so the seam survives and there is no pole.
static BRep make_tube(double r, double h, double amp) {
    BRep cyl = BRep::create_cylinder(r, h);
    // the lateral surface is the one that is closed in some direction
    for (size_t si = 0; si < cyl.m_surfaces.size(); ++si) {
        NurbsSurface& S = cyl.m_surfaces[si];
        const int wrap = S.is_closed(0) ? 0 : (S.is_closed(1) ? 1 : -1);
        if (wrap < 0) continue;                    // planar cap: leave alone
        const int axial = 1 - wrap;
        auto da = S.domain(axial);
        for (int k = 1; k <= 3; ++k)
            S.insert_nurbsknot(axial, da.first + (da.second - da.first) * k / 4.0, 1);
        const int nu = S.cv_count(0), nv = S.cv_count(1);
        const int na = (axial == 0) ? nu : nv;
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                const int ia = (axial == 0) ? i : j;
                if (ia == 0 || ia == na - 1) continue;      // keep the cap circles EXACT
                double x, y, z, w;
                if (!S.get_cv_4d(i, j, x, y, z, w)) continue;
                const double t = (double)ia / (double)(na - 1);
                const double f = 1.0 + amp * std::sin(3.14159265358979323846 * t);
                S.set_cv_4d(i, j, x * f, y * f, z, w);      // radial only: axis is z
            }
    }
    cyl.name = "tube";
    return cyl;
}

///////////////////////////////////////////////////////////////////////////////////////////
// spec parsing
///////////////////////////////////////////////////////////////////////////////////////////

static BRep translated(const BRep& b, double tx, double ty, double tz) {
    if (tx == 0.0 && ty == 0.0 && tz == 0.0) return b;
    BRep c = b;
    c.xform = Xform::translation(tx, ty, tz);
    return c.transformed();
}

/// Exits 4 on an unreadable/unknown source so a missing file is never scored as a boolean
/// failure.
static BRep load_spec(const std::string& spec_in) {
    std::string spec = spec_in;
    double tx = 0, ty = 0, tz = 0;
    const size_t at = spec.rfind('@');
    if (at != std::string::npos && spec.find(':') < at) {
        std::sscanf(spec.c_str() + at + 1, "%lf,%lf,%lf", &tx, &ty, &tz);
        spec = spec.substr(0, at);
    }
    if (spec.rfind("step:", 0) == 0) {
        std::string path = spec.substr(5);
        int idx = 0;
        const size_t hash = path.rfind('#');
        if (hash != std::string::npos) {
            idx = std::atoi(path.c_str() + hash + 1);
            path = path.substr(0, hash);
        }
        std::vector<BRep> bs;
        try {
            bs = file_step::read_file_step_breps(path);
        } catch (const std::exception& e) {
            std::printf("T3ERR load_threw spec=%s what=%s\nT3DONE\n", spec_in.c_str(), e.what());
            std::exit(4);
        }
        if (idx < 0 || idx >= (int)bs.size()) {
            std::printf("T3ERR load_empty spec=%s n=%zu\nT3DONE\n", spec_in.c_str(), bs.size());
            std::exit(4);
        }
        return translated(bs[(size_t)idx], tx, ty, tz);
    }
    if (spec.rfind("own:", 0) == 0) {
        std::string n = spec.substr(4);
        std::string args;
        const size_t colon = n.find(':');
        if (colon != std::string::npos) { args = n.substr(colon + 1); n = n.substr(0, colon); }
        double a = 0, b = 0, c = 0;
        const int na = args.empty() ? 0 : std::sscanf(args.c_str(), "%lf,%lf,%lf", &a, &b, &c);
        if (n == "box")
            return translated(BRep::create_box(na >= 1 ? a : 4.0, na >= 2 ? b : 4.0,
                                               na >= 3 ? c : 4.0), tx, ty, tz);
        if (n == "pillow")
            return translated(make_pillow(4.0, 4.0, 4.0, na >= 1 ? a : 0.5), tx, ty, tz);
        if (n == "blob")   return translated(make_blob(na >= 1 ? a : 0.12), tx, ty, tz);
        if (n == "sphere") return translated(BRep::create_sphere(na >= 1 ? a : 2.5), tx, ty, tz);
        if (n == "cyl")
            return translated(BRep::create_cylinder(na >= 1 ? a : 1.5, na >= 2 ? b : 6.0),
                              tx, ty, tz);
        if (n == "tube")
            return translated(make_tube(na >= 1 ? a : 1.5, na >= 2 ? b : 6.0,
                                        na >= 3 ? c : 0.25), tx, ty, tz);
        std::printf("T3ERR unknown_own spec=%s\nT3DONE\n", spec_in.c_str());
        std::exit(4);
    }
    std::printf("T3ERR bad_spec spec=%s\nT3DONE\n", spec_in.c_str());
    std::exit(4);
    return BRep();
}

///////////////////////////////////////////////////////////////////////////////////////////
// printing
///////////////////////////////////////////////////////////////////////////////////////////

static void print_verdict(const char* tag, const std::string& key, const V2Verdict& v,
                          double secs, const std::string& note) {
    std::printf("%s %s F=%d shells=%d solids=%d naked=%d nonman=%d seam=%d degen=%d orphan=%d "
                "edges=%d resid=%.3e area=%.6f vol=%.6f volvalid=%d converged=%d closed=%d "
                "secs=%.2f %s\n",
                tag, key.c_str(), v.faces, v.shells, v.solids, v.naked_real, v.nonmanifold,
                v.seam_edges, v.degenerate, v.orphan, v.edges, v.closure_residual, v.area,
                v.volume, (int)v.volume_valid, (int)v.converged, (int)v.closed(), secs,
                note.c_str());
    std::fflush(stdout);
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";

    if (mode == "probe" && argc >= 3) {
        const std::string spec = argv[2];
        const auto t0 = std::chrono::steady_clock::now();
        BRep b = load_spec(spec);
        const double t_load = secs_since(t0);
        const auto t1 = std::chrono::steady_clock::now();
        const V2Verdict v = v2_verdict(b, score_options());
        char note[192];
        std::snprintf(note, sizeof note, "kvol=%.6f is_solid=%d load_secs=%.2f",
                      b.volume(), b.is_solid() ? 1 : 0, t_load);
        print_verdict("T3PROBE", "spec=" + spec, v, secs_since(t1), note);
        std::printf("T3DONE\n");
        return 0;
    }

    if (mode == "ssi" && argc >= 4) {
        // SURFACE-SURFACE INTERSECTION CENSUS. For every (face of A) x (face of B) pair, how
        // many section curves does the kernel's SSI return, and how long are they? A pair that
        // OCCT sections into a real edge but that returns 0 curves here is a PROVEN miss --
        // with no curve there is nothing to trim with, and the face passes through whole.
        BRep A = load_spec(argv[2]);
        BRep B = load_spec(argv[3]);
        // corner points of every surface patch: the geometric extent the SSI is allowed to use
        auto corners = [](const char* tag, const BRep& X) {
            for (size_t k = 0; k < X.m_surfaces.size(); ++k) {
                auto du = X.m_surfaces[k].domain(0), dv = X.m_surfaces[k].domain(1);
                Point c00 = X.m_surfaces[k].point_at(du.first, dv.first);
                Point c10 = X.m_surfaces[k].point_at(du.second, dv.first);
                Point c01 = X.m_surfaces[k].point_at(du.first, dv.second);
                Point c11 = X.m_surfaces[k].point_at(du.second, dv.second);
                std::printf("T3SURF %s %zu (%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f) "
                            "(%.4f,%.4f,%.4f)\n", tag, k, c00[0], c00[1], c00[2], c10[0], c10[1],
                            c10[2], c01[0], c01[1], c01[2], c11[0], c11[1], c11[2]);
            }
        };
        corners("A", A);
        corners("B", B);
        int pairs = 0, nonempty = 0, ncurves = 0;
        for (size_t i = 0; i < A.m_surfaces.size(); ++i)
            for (size_t j = 0; j < B.m_surfaces.size(); ++j) {
                ++pairs;
                std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> trs;
                try {
                    trs = Intersection::surface_surface(A.m_surfaces[i], B.m_surfaces[j], 1e-6);
                } catch (...) {
                    std::printf("T3SSI a=%zu b=%zu THREW\n", i, j);
                    continue;
                }
                double len = 0.0;
                for (auto& tr : trs) len += std::get<0>(tr).length();
                if (!trs.empty()) { ++nonempty; ncurves += (int)trs.size(); }
                auto dau = A.m_surfaces[i].domain(0), dav = A.m_surfaces[i].domain(1);
                std::printf("T3SSI a=%zu b=%zu curves=%zu len=%.6f Adom=[%.3f,%.3f]x[%.3f,%.3f]",
                            i, j, trs.size(), len, dau.first, dau.second, dav.first, dav.second);
                for (auto& tr : trs) {
                    // does the pcurve on A leave A's own parametric domain?
                    const NurbsCurve& pa = std::get<1>(tr);
                    auto dp = pa.domain();
                    double ulo = 1e30, uhi = -1e30, vlo = 1e30, vhi = -1e30;
                    for (int k = 0; k <= 64; ++k) {
                        Point uv = pa.point_at(dp.first + (dp.second - dp.first) * k / 64.0);
                        ulo = std::min(ulo, uv[0]); uhi = std::max(uhi, uv[0]);
                        vlo = std::min(vlo, uv[1]); vhi = std::max(vhi, uv[1]);
                    }
                    const double outu = std::max(dau.first - ulo, uhi - dau.second);
                    const double outv = std::max(dav.first - vlo, vhi - dav.second);
                    std::printf(" pcA=[%.3f,%.3f]x[%.3f,%.3f] outside_A=%.3e",
                                ulo, uhi, vlo, vhi, std::max(outu, outv));
                }
                for (auto& tr : trs) {
                    const NurbsCurve& c = std::get<0>(tr);
                    auto d = c.domain();
                    Point p0 = c.point_at(d.first), pm = c.point_at(0.5 * (d.first + d.second)),
                          p1 = c.point_at(d.second);
                    std::printf(" | (%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f) mid(%.4f,%.4f,%.4f)",
                                p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], pm[0], pm[1], pm[2]);
                }
                std::printf("\n");
            }
        std::printf("T3SSISUM A=%s B=%s pairs=%d nonempty=%d curves=%d\nT3DONE\n",
                    argv[2], argv[3], pairs, nonempty, ncurves);
        return 0;
    }

    if (mode == "export" && argc >= 4) {
        BRep b = load_spec(argv[2]);
        file_step::write_file_step_brep(b, argv[3]);
        std::printf("T3EXPORT spec=%s out=%s F=%d kvol=%.6f is_solid=%d\nT3DONE\n",
                    argv[2], argv[3], b.face_count(), b.volume(), b.is_solid() ? 1 : 0);
        return 0;
    }

    if (mode == "cell" && argc >= 6) {
        const std::string kern = argv[2], op = argv[3], sa = argv[4], sb = argv[5];
        const std::string key = "kernel=" + kern + " op=" + op + " A=" + sa + " B=" + sb;
        BRep A = load_spec(sa);
        BRep B = load_spec(sb);
        if (std::getenv("SESSION_T3_OPERANDS")) {
            const V2Verdict va = v2_verdict(A, score_options());
            const V2Verdict vb = v2_verdict(B, score_options());
            print_verdict("T3OPA", key, va, 0.0, "");
            print_verdict("T3OPB", key, vb, 0.0, "");
        }
        const auto t0 = std::chrono::steady_clock::now();
        BRep r;
        try {
            if (kern == "v1") {
                r = (op == "cut")      ? A.boolean_difference(B)
                    : (op == "common") ? A.boolean_intersection(B)
                                       : A.boolean_union(B);
            } else {
                v2sol::V2BooleanReport rep;
                r = (op == "cut")      ? v2sol::v2_cut(A, B, 0.0, &rep)
                    : (op == "common") ? v2sol::v2_common(A, B, 0.0, &rep)
                                       : v2sol::v2_fuse(A, B, 0.0, &rep);
                if (std::getenv("SESSION_T3_REPORT"))
                    std::printf("T3REPORT %s %s\n", key.c_str(), rep.str().c_str());
            }
        } catch (const std::exception& e) {
            std::printf("T3CELL %s THREW secs=%.2f what=%s\nT3DONE\n", key.c_str(),
                        secs_since(t0), e.what());
            return 2;
        } catch (...) {
            std::printf("T3CELL %s THREW secs=%.2f what=?\nT3DONE\n", key.c_str(),
                        secs_since(t0));
            return 2;
        }
        const double t_bool = secs_since(t0);
        if (r.face_count() == 0) {
            std::printf("T3CELL %s F=0 shells=0 solids=0 naked=0 nonman=0 seam=0 degen=0 "
                        "orphan=0 edges=0 resid=1.000e+00 area=0.000000 vol=0.000000 volvalid=0 "
                        "converged=1 closed=0 secs=%.2f note=EMPTY_RESULT\n",
                        key.c_str(), t_bool);
            std::printf("T3DONE\n");
            return 0;
        }
        const auto t1 = std::chrono::steady_clock::now();
        const V2Verdict v = v2_verdict(r, score_options());
        char note[160];
        std::snprintf(note, sizeof note, "kvol=%.6f score_secs=%.2f",
                      (v.naked_real == 0 && v.nonmanifold == 0) ? r.volume() : 0.0,
                      secs_since(t1));
        print_verdict("T3CELL", key, v, t_bool, note);
        if (const char* sd = std::getenv("SESSION_T3_SAVE"))
            if (sd[0]) file_step::write_file_step_brep(r, sd);
        std::printf("T3DONE\n");
        return 0;
    }

    std::printf("usage: main_22 probe <spec> | export <spec> <out.step> | "
                "cell <v1|v2> <cut|common|fuse> <specA> <specB>\n");
    (void)env_or;
    return 1;
}
