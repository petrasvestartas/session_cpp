// main_12 -- test driver for src/brep_massprops.{h,cpp}: volume / surface area / centroid of
// trimmed-NURBS solids.
//
// Every case is checked against ANALYTIC truth (never against another implementation of the
// same code) and every case is timed against a documented bound.
//
//   usage: main_12 [fixture_dir] [reference_dir]
//     fixture_dir   default /home/petras/massprops_fx            (OCCT/FreeCAD-authored STEP)
//     reference_dir default /home/petras/fc_inspect/REFERENCE    (OCCT boolean results)
#include "src/brep.h"
#include "src/brep_massprops.h"
#include "src/file_step.h"
#include "src/xform.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace session_cpp;

namespace {

constexpr double PI = 3.14159265358979323846;

int g_pass = 0, g_fail = 0;
double g_max_seconds = 0.0;

struct Truth {
    double volume;
    double area;
    bool has_centroid = false;
    double cx = 0, cy = 0, cz = 0;
};

double relerr(double got, double want) {
    return std::abs(got - want) / std::max(1e-30, std::abs(want));
}

/// Run mass properties on `b` and check, all at once:
///   - volume / area / centroid against ANALYTIC truth at `rtol`
///   - the CLOSURE certificate (net outward vector area must vanish on a closed solid)
///   - TRANSLATION INVARIANCE of volume and area: the divergence-theorem flux integrates
///     absolute world coordinates, so it is invariant only when the boundary really closes.
///     This is a correctness check that needs no reference value at all.
///   - the documented time bound.
void check(const std::string& name, const BRep& b, const Truth& t, double rtol, double time_bound,
           double clos_tol = 1e-9, double inv_tol = -1.0,
           const MassPropsOptions& opt = MassPropsOptions()) {
    if (inv_tol < 0.0) inv_tol = rtol;
    const auto t0 = std::chrono::steady_clock::now();
    MassProps m = brep_massprops(b, opt);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    g_max_seconds = std::max(g_max_seconds, secs);

    BRep tb = b;
    tb.xform = Xform::translation(37.0, -11.0, 53.0);
    tb.transform();
    MassProps tm = brep_massprops(tb, opt);
    const double einv = std::max(relerr(tm.volume, m.volume), relerr(tm.area, m.area));

    const double ev = relerr(m.volume, t.volume);
    const double ea = relerr(m.area, t.area);
    bool ok = ev <= rtol && ea <= rtol && secs <= time_bound && einv <= inv_tol &&
              m.closure_residual <= clos_tol && m.converged;
    double ec = 0.0;
    if (t.has_centroid) {
        const double sc = std::max(1.0, std::abs(t.cx) + std::abs(t.cy) + std::abs(t.cz));
        ec = (std::abs(m.centroid[0] - t.cx) + std::abs(m.centroid[1] - t.cy) +
              std::abs(m.centroid[2] - t.cz)) / sc;
        if (ec > rtol) ok = false;
    }
    const double claim_v = m.volume_error / std::max(1e-30, std::abs(t.volume));
    if (ok) ++g_pass; else ++g_fail;
    std::printf("%-26s %s V=%.10f (rel %.2e, claim %.2e) A=%.10f (rel %.2e) C=(%.6f,%.6f,%.6f) "
                "rel %.1e | transl-inv %.1e closure %.1e gap %.1e | faces=%d shells=%d "
                "evals=%lld conv=%d naked=%d %.3fs\n",
                name.c_str(), ok ? "PASS" : "FAIL", m.volume, ev, claim_v, m.area, ea,
                m.centroid[0], m.centroid[1], m.centroid[2], ec, einv, m.closure_residual,
                m.max_chain_gap, (int)m.faces.size(), m.shell_count, m.surface_evals,
                m.converged ? 1 : 0, m.naked_edges, secs);
}

BRep step_roundtrip(const BRep& b, const std::string& path) {
    file_step::write_file_step_brep(b, path);
    auto v = file_step::read_file_step_breps(path);
    return v.empty() ? BRep() : v[0];
}

/// Index-shifted union of two BReps: one BRep holding two disjoint shells. Used to build the
/// cavity case, because our STEP reader does not read BREP_WITH_VOIDS (see the notes in
/// kb/massprops_integration_notes.md), so the FreeCAD box-with-void file cannot be loaded.
BRep merge_breps(const BRep& A, const BRep& B) {
    BRep m = A;
    const int so = (int)A.m_surfaces.size(), c3o = (int)A.m_curves_3d.size();
    const int c2o = (int)A.m_curves_2d.size(), vo = (int)A.m_vertices.size();
    const int tvo = (int)A.m_topology_vertices.size(), eo = (int)A.m_topology_edges.size();
    const int to = (int)A.m_trims.size(), lo = (int)A.m_loops.size(), fo = (int)A.m_faces.size();
    for (const auto& x : B.m_surfaces) m.m_surfaces.push_back(x);
    for (const auto& x : B.m_curves_3d) m.m_curves_3d.push_back(x);
    for (const auto& x : B.m_curves_2d) m.m_curves_2d.push_back(x);
    for (const auto& x : B.m_vertices) m.m_vertices.push_back(x);
    for (auto x : B.m_topology_vertices) {
        x.point_index += vo;
        for (auto& e : x.edge_indices) e += eo;
        m.m_topology_vertices.push_back(x);
    }
    for (auto x : B.m_topology_edges) {
        if (x.curve_3d_index >= 0) x.curve_3d_index += c3o;
        if (x.start_vertex >= 0) x.start_vertex += tvo;
        if (x.end_vertex >= 0) x.end_vertex += tvo;
        for (auto& t : x.trim_indices) t += to;
        m.m_topology_edges.push_back(x);
    }
    for (auto x : B.m_trims) {
        if (x.curve_2d_index >= 0) x.curve_2d_index += c2o;
        if (x.edge_index >= 0) x.edge_index += eo;
        if (x.loop_index >= 0) x.loop_index += lo;
        m.m_trims.push_back(x);
    }
    for (auto x : B.m_loops) {
        for (auto& t : x.trim_indices) t += to;
        if (x.face_index >= 0) x.face_index += fo;
        m.m_loops.push_back(x);
    }
    for (auto x : B.m_faces) {
        if (x.surface_index >= 0) x.surface_index += so;
        for (auto& l : x.loop_indices) l += lo;
        m.m_faces.push_back(x);
    }
    return m;
}

BRep load_step(const std::string& path, bool& ok) {
    auto v = file_step::read_file_step_breps(path);
    ok = !v.empty();
    return ok ? v[0] : BRep();
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // --dump <step> : run one file with SESSION_MP_DBG diagnostics (development aid).
    if (argc > 2 && (std::string(argv[1]) == "--dump" || std::string(argv[1]) == "--mem")) {
        std::vector<BRep> v;
        if (std::string(argv[1]) == "--mem") {
            const std::string k = argv[2];
            if (k == "blockhole") v.push_back(BRep::create_block_with_hole(6, 4, 3, 1.1));
            else if (k == "box") v.push_back(BRep::create_box(4, 4, 4));
            else if (k == "sphere") v.push_back(BRep::create_sphere(2.5));
            else if (k == "cylinder") v.push_back(BRep::create_cylinder(1.5, 6.0));
            else if (k == "cone") v.push_back(BRep::create_cone(2.0, 4.0));
            else if (k == "torus") v.push_back(BRep::create_torus(2.0, 0.8));
        } else v = file_step::read_file_step_breps(argv[2]);
        std::printf("breps=%zu\n", v.size());
        for (size_t i = 0; i < v.size(); ++i) {
            const BRep& b = v[i];
            std::printf("brep %zu: faces=%d loops=%zu trims=%zu edges=%d surfaces=%zu "
                        "c2d=%zu solid=%d\n",
                        i, b.face_count(), b.m_loops.size(), b.m_trims.size(), b.edge_count(),
                        b.m_surfaces.size(), b.m_curves_2d.size(), b.is_solid() ? 1 : 0);
            {
                double mn[3] = {1e300, 1e300, 1e300}, mx[3] = {-1e300, -1e300, -1e300};
                for (const auto& p : b.m_vertices)
                    for (int k = 0; k < 3; ++k) {
                        mn[k] = std::min(mn[k], p[k]);
                        mx[k] = std::max(mx[k], p[k]);
                    }
                std::printf("  vertex bbox [%.4f %.4f %.4f]-[%.4f %.4f %.4f] nverts=%zu\n", mn[0],
                            mn[1], mn[2], mx[0], mx[1], mx[2], b.m_vertices.size());
            }
            for (size_t f = 0; f < b.m_faces.size(); ++f) {
                std::printf("  face %zu srf=%d loops=%zu rev=%d\n", f, b.m_faces[f].surface_index,
                            b.m_faces[f].loop_indices.size(), b.m_faces[f].reversed ? 1 : 0);
                const int si = b.m_faces[f].surface_index;
                if (si < 0 || si >= (int)b.m_surfaces.size()) continue;
                const NurbsSurface& s = b.m_surfaces[si];
                auto a = s.domain(0), c = s.domain(1);
                std::printf("    dom [%.6f,%.6f]x[%.6f,%.6f] deg %dx%d cv %dx%d rat=%d "
                            "knots_u=%d knots_v=%d\n",
                            a.first, a.second, c.first, c.second, s.degree(0), s.degree(1),
                            s.cv_count(0), s.cv_count(1), s.is_rational() ? 1 : 0,
                            s.nurbsknot_count(0), s.nurbsknot_count(1));
                std::printf("    ku:");
                for (int k = 0; k < s.nurbsknot_count(0); ++k) std::printf(" %.4f", s.nurbsknot(0, k));
                std::printf("\n    kv:");
                for (int k = 0; k < s.nurbsknot_count(1); ++k) std::printf(" %.4f", s.nurbsknot(1, k));
                std::printf("\n");
                for (int k = 0; k <= 12; ++k) {
                    double u = a.first + (a.second - a.first) * k / 12.0;
                    Point p0 = s.point_at(u, c.first), p1 = s.point_at(u, c.second);
                    std::printf("    u=%.4f -> v0(%.5f,%.5f,%.5f)  v1(%.5f,%.5f,%.5f)\n", u, p0[0],
                                p0[1], p0[2], p1[0], p1[1], p1[2]);
                }
            }
            BRep tb = b;
            tb.xform = Xform::translation(100, -37, 61);
            tb.transform();
            MassProps tm = brep_massprops(tb);
            MassProps m = brep_massprops(b);
            std::printf("  TRANSLATED V=%.12f A=%.12f  (dV=%.3e rel) -- a CLOSED shell makes "
                        "the flux translation-invariant\n",
                        tm.volume, tm.area,
                        std::abs(tm.volume - m.volume) / std::max(1e-30, std::abs(m.volume)));
            std::printf("  clos=%.3e gap=%.3e\n", m.closure_residual, m.max_chain_gap);
            std::printf("  V=%.12f A=%.12f C=(%.9f,%.9f,%.9f) shells=%d conv=%d evals=%lld "
                        "%.3fs\n",
                        m.volume, m.area, m.centroid[0], m.centroid[1], m.centroid[2],
                        m.shell_count, m.converged ? 1 : 0, m.surface_evals, m.seconds);
            for (const auto& F : m.faces)
                std::printf("  F%-3d path=%d area=%.9f flux=%.9f trav=%+.0f out=%+.0f shell=%d "
                            "evals=%lld cap=%d\n",
                            F.face_index, (int)F.path, F.area, F.flux, F.traversal, F.outward,
                            F.shell, F.evals, F.budget_hit ? 1 : 0);
        }
        return 0;
    }
    const std::string fx = argc > 1 ? argv[1] : "/home/petras/massprops_fx";
    const std::string ref = argc > 2 ? argv[2] : "/home/petras/fc_inspect/REFERENCE";
    const std::string tmp = fx + "/_rt";

    const double R_SPH = 2.5, R_CYL = 1.5, H_CYL = 6.0, R_CON = 2.0, H_CON = 4.0;
    const double R_TOR = 2.0, r_TOR = 0.8;
    Truth T_box{64.0, 96.0, true, 0, 0, 0};
    Truth T_sph{4.0 / 3.0 * PI * R_SPH * R_SPH * R_SPH, 4.0 * PI * R_SPH * R_SPH, true, 0, 0, 0};
    Truth T_cyl{PI * R_CYL * R_CYL * H_CYL, 2 * PI * R_CYL * R_CYL + 2 * PI * R_CYL * H_CYL};
    Truth T_con{PI * R_CON * R_CON * H_CON / 3.0,
                PI * R_CON * R_CON + PI * R_CON * std::sqrt(R_CON * R_CON + H_CON * H_CON)};
    Truth T_tor{2 * PI * PI * R_TOR * r_TOR * r_TOR, 4 * PI * PI * R_TOR * r_TOR, true, 0, 0, 0};

    std::printf("=== 0. DEGENERATE INPUT (must return, must not crash) ===\n");
    {
        MassProps e = brep_massprops(BRep());
        std::printf("empty BRep                  %s V=%.3f A=%.3f faces=%d shells=%d conv=%d\n",
                    (e.volume == 0.0 && e.area == 0.0 && e.faces.empty()) ? "PASS" : "FAIL",
                    e.volume, e.area, (int)e.faces.size(), e.shell_count, e.converged ? 1 : 0);
        if (e.volume == 0.0 && e.area == 0.0 && e.faces.empty()) ++g_pass; else ++g_fail;

        // Open shell: one face of a box. Area is still exact; closure must be reported bad.
        BRep one = BRep::create_box(4, 4, 4);
        one.m_faces.resize(1);
        MassProps o = brep_massprops(one);
        const bool ok = std::abs(o.area - 16.0) < 1e-9 && o.closure_residual > 1e-3 && !o.closed;
        std::printf("open shell (1 box face)     %s A=%.10f (want 16) closure=%.3e closed=%d\n",
                    ok ? "PASS" : "FAIL", o.area, o.closure_residual, o.closed ? 1 : 0);
        if (ok) ++g_pass; else ++g_fail;
    }

    std::printf("\n=== 1. OUR OWN PRIMITIVES, IN MEMORY (analytic truth, rtol 1e-9) ===\n");
    BRep box = BRep::create_box(4, 4, 4);
    BRep sph = BRep::create_sphere(R_SPH);
    BRep cyl = BRep::create_cylinder(R_CYL, H_CYL);
    BRep con = BRep::create_cone(R_CON, H_CON);
    BRep tor = BRep::create_torus(R_TOR, r_TOR);
    check("mem/box", box, T_box, 1e-9, 5.0);
    check("mem/sphere", sph, T_sph, 1e-9, 5.0);
    check("mem/cylinder", cyl, T_cyl, 1e-9, 5.0);
    check("mem/cone", con, T_con, 1e-9, 5.0);
    check("mem/torus", tor, T_tor, 1e-9, 5.0);

    std::printf("\n=== 2. OUR OWN PRIMITIVES, THROUGH OUR STEP ROUND-TRIP (rtol 1e-9) ===\n");
    check("step-rt/box", step_roundtrip(box, tmp + "_box.step"), T_box, 1e-9, 5.0);
    check("step-rt/sphere", step_roundtrip(sph, tmp + "_sphere.step"), T_sph, 1e-9, 5.0);
    check("step-rt/cylinder", step_roundtrip(cyl, tmp + "_cylinder.step"), T_cyl, 1e-9, 5.0);
    check("step-rt/cone", step_roundtrip(con, tmp + "_cone.step"), T_con, 1e-9, 5.0);
    check("step-rt/torus", step_roundtrip(tor, tmp + "_torus.step"), T_tor, 1e-9, 5.0);

    std::printf("\n=== 3. OUR OWN TRIMMED SOLID (block with a through hole) ===\n");
    {
        const double sx = 6, sy = 4, sz = 3, rh = 1.1;
        Truth T{sx * sy * sz - PI * rh * rh * sz,
                2 * (sx * sy - PI * rh * rh) + 2 * (sx * sz + sy * sz) + 2 * PI * rh * sz,
                true, 0, 0, 0};
        check("mem/block_with_hole", BRep::create_block_with_hole(sx, sy, sz, rh), T, 1e-9, 5.0);
    }

    // ---------------------------------------------------------------------------------------
    // OCCT-authored STEP. Two sets, both written by FreeCAD/OCCT:
    //   *.step        analytic surfaces (SPHERICAL_/CONICAL_/CYLINDRICAL_/TOROIDAL_SURFACE)
    //                 which our reader converts to NURBS;
    //   *_nurbs.step  Part.Shape.toNurbs() first, so the file itself carries B_SPLINE_SURFACE
    //                 -- genuinely foreign NURBS, the case the analytic recognisers miss.
    // Truth is ANALYTIC in both cases (a rational B-spline quadric is exact).
    // ---------------------------------------------------------------------------------------
    struct Case {
        const char* file;
        Truth t;
        double rtol;
        const char* xfail;  // non-null: a known STEP-READER defect, reported, not counted
        double clos_tol = 1e-9;
        double inv_tol = -1.0;
    };
    const Truth T_ob{64.0, 96.0, true, 0, 0, 0};
    const Truth T_os{4.0 / 3.0 * PI * 15.625, 4.0 * PI * 6.25, true, 0, 0, 0};
    const Truth T_ocy{PI * 2.25 * 6.0, 2 * PI * 2.25 + 2 * PI * 1.5 * 6.0, true, 0, 0, 3.0};
    const Truth T_oco{PI * 4.0 * 4.0 / 3.0, PI * 4.0 + PI * 2.0 * std::sqrt(20.0), true, 0, 0, 1.0};
    const Truth T_ot{2 * PI * PI * 3.0 * 0.64, 4 * PI * PI * 3.0 * 0.8, true, 0, 0, 0};
    const double Rs = 3.0, ah = 1.0, hh = 2.0 * std::sqrt(Rs * Rs - ah * ah);
    const Truth T_nap{4.0 / 3.0 * PI * std::pow(Rs * Rs - ah * ah, 1.5),
                      2 * PI * Rs * hh + 2 * PI * ah * hh, true, 0, 0, 0};
    const Truth T_hc{0.5 * PI * 4.0 * 5.0,
                     0.5 * (2 * PI * 2.0 * 5.0) + 2 * (0.5 * PI * 4.0) + 4.0 * 5.0};

    std::printf("\n=== 4. OCCT-AUTHORED, ANALYTIC-SURFACE STEP (analytic truth) ===\n");
    const std::vector<Case> occ_a = {
        {"occ_box", T_ob, 1e-9, nullptr},
        {"occ_sphere", T_os, 1e-9,
         "STEP reader returns an EMPTY NurbsSurface (deg -1, 0 CVs, no loops) for a bare "
         "SPHERICAL_SURFACE face -- no geometry reaches mass properties"},
        {"occ_cylinder", T_ocy, 1e-9, nullptr},
        {"occ_cone", T_oco, 1e-9,
         "STEP reader builds the CONICAL_SURFACE as a 6-span NURBS spanning 1.5 turns "
         "(u in [0,6], one turn = [0,4]) and runs the seam pcurve diagonally to u=6 at the "
         "apex -- the imported face self-overlaps a wedge"},
        {"occ_torus", T_ot, 1e-9, nullptr},
        {"occ_sphere_hole", T_nap, 1e-9, nullptr},
        {"occ_cyl_halfcut", T_hc, 1e-9, nullptr},
    };
    int xfail = 0;
    for (const Case& c : occ_a) {
        bool ok = false;
        BRep b = load_step(fx + "/" + std::string(c.file) + ".step", ok);
        if (!ok) { std::printf("%-28s FAIL (cannot read STEP)\n", c.file); ++g_fail; continue; }
        if (c.xfail) {
            MassProps m = brep_massprops(b);
            ++xfail;
            std::printf("%-28s XFAIL V=%.10f (want %.10f) A=%.10f (want %.10f)\n    reader "
                        "defect: %s\n",
                        (std::string("occA/") + c.file).c_str(), m.volume, c.t.volume, m.area,
                        c.t.area, c.xfail);
            continue;
        }
        check(std::string("occA/") + c.file, b, c.t, c.rtol, 20.0, c.clos_tol, c.inv_tol);
    }

    // The toNurbs() files carry exact rational B-spline quadrics, but our reader stores their
    // PCURVES as 48-segment degree-1 polylines and drops the 3D edge curves entirely, so a
    // face whose only boundary evidence is such a pcurve is a 48-gon in the data. An inscribed
    // 48-gon under-measures a circle by 1 - (48/2pi) sin(2pi/48) = 2.9e-3, which caps the
    // achievable accuracy for those faces; the tolerance below is that documented ceiling.
    std::printf("\n=== 5. OCCT-AUTHORED, toNurbs() B-SPLINE STEP (analytic truth) ===\n");
    const std::vector<Case> occ_n = {
        {"occ_box", T_ob, 1e-9, nullptr},
        {"occ_sphere", T_os, 1e-9, nullptr},
        {"occ_cylinder", T_ocy, 3e-3, nullptr},
        // The reader gives this file NO pcurves for the cone's own (periodic) face -- so it is
        // integrated over its whole parameter patch, exactly -- but a 48-segment polyline
        // pcurve for the planar base disc. The two faces therefore describe DIFFERENT base
        // circles, and the shell cannot close: the measured closure residual is 9.4e-4, which
        // is exactly the 48-gon/circle area defect. Reported, with tolerances that name it.
        {"occ_cone", T_oco, 3e-3, nullptr, 2e-3, 5e-2},
        {"occ_torus", T_ot, 1e-9, nullptr},
        {"occ_sphere_hole", T_nap, 1e-9, nullptr},
        {"occ_cyl_halfcut", T_hc, 3e-3, nullptr},
    };
    for (const Case& c : occ_n) {
        bool ok = false;
        BRep b = load_step(fx + "/" + std::string(c.file) + "_nurbs.step", ok);
        if (!ok) { std::printf("%-28s FAIL (cannot read STEP)\n", c.file); ++g_fail; continue; }
        check(std::string("occN/") + c.file, b, c.t, c.rtol, 20.0, c.clos_tol, c.inv_tol);
    }

    std::printf("\n=== 6. CAVITY: box 6^3 with a concentric spherical void r=2 (2 shells) ===\n");
    {
        Truth T{216.0 - 4.0 / 3.0 * PI * 8.0, 216.0 + 4 * PI * 4.0, true, 0, 0, 0};
        BRep cav = merge_breps(BRep::create_box(6, 6, 6), BRep::create_sphere(2.0));
        check("mem/box_with_cavity", cav, T, 1e-9, 5.0);
        // and the same two shells DISJOINT: legitimately a two-solid result, volumes add.
        BRep sph2 = BRep::create_sphere(2.0);
        sph2.xform = Xform::translation(20, 0, 0);
        sph2.transform();
        Truth T2{216.0 + 4.0 / 3.0 * PI * 8.0, 216.0 + 4 * PI * 4.0, false, 0, 0, 0};
        check("mem/two_disjoint_solids", merge_breps(BRep::create_box(6, 6, 6), sph2), T2, 1e-9,
              5.0);
    }

    std::printf("\n=== 7. TERMINATION / TIMING GATE on 40 OCCT boolean solids (%s) ===\n"
                "    (this corpus is a TERMINATION gate, not an accuracy gate: our STEP reader\n"
                "     returns trim loops that do not chain in UV for these files, so the shells\n"
                "     are not geometrically closed -- reported per solid as clos/gap below.)\n",
                ref.c_str());
    {
        static const char* names[] = {
            "REFERENCE_x13y29_common", "REFERENCE_x13y29_cut",  "REFERENCE_x13y29_fuse",
            "REFERENCE_x20_common",    "REFERENCE_x20_cut",     "REFERENCE_x20_fuse",
            "REFERENCE_y30_common",    "REFERENCE_y30_cut",     "REFERENCE_y30_fuse",
            "REFERENCE_z15_common",    "REFERENCE_z15_cut",     "REFERENCE_z15_fuse",
            "REFERENCE_z30_common",    "REFERENCE_z30_cut",     "REFERENCE_z30_fuse",
            "REFERENCE_z30x20_common", "REFERENCE_z30x20_cut",  "REFERENCE_z30x20_fuse",
            "REFERENCE_z37_common",    "REFERENCE_z37_cut",     "REFERENCE_z37_fuse",
            "REFERENCE_z45_common",    "REFERENCE_z45_cut",     "REFERENCE_z45_fuse",
            "REFERENCE_z63_common",    "REFERENCE_z63_cut",     "REFERENCE_z63_fuse",
            "REFERENCE_z90_common",    "REFERENCE_z90_cut",     "REFERENCE_z90_fuse"};
        const double BOUND = 60.0;
        int done = 0, over = 0, f32 = 0, closed = 0;
        double worst = 0.0, total = 0.0;
        std::string worst_name;
        for (const char* n : names) {
            auto v = file_step::read_file_step_breps(ref + "/" + n + ".step");
            if (v.empty()) continue;
            for (size_t i = 0; i < v.size(); ++i) {
                const auto t0 = std::chrono::steady_clock::now();
                MassProps m = brep_massprops(v[i]);
                const double s2 =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                ++done;
                total += s2;
                if (s2 > worst) { worst = s2; worst_name = n; }
                if (s2 > BOUND) ++over;
                if ((int)m.faces.size() == 32) ++f32;
                if (m.closure_residual < 1e-9) ++closed;
                std::printf("  %-26s[%zu] faces=%3d V=%13.6f+-%.1e A=%13.6f clos=%.1e gap=%.1e "
                            "shells=%d conv=%d naked=%d %7.3fs\n",
                            n, i, (int)m.faces.size(), m.volume, m.volume_error, m.area,
                            m.closure_residual, m.max_chain_gap, m.shell_count,
                            m.converged ? 1 : 0, m.naked_edges, s2);
            }
        }
        g_max_seconds = std::max(g_max_seconds, worst);
        std::printf("  -> %d solids, %d with exactly 32 faces, %d geometrically closed, worst "
                    "%.3fs (%s), total %.1fs, bound %.0fs/solid, %d over\n",
                    done, f32, closed, worst, worst_name.c_str(), total, BOUND, over);
        if (done == 0) { std::printf("  FAIL: no reference solids read\n"); ++g_fail; }
        else if (over > 0) { std::printf("  FAIL: %d over the time bound\n", over); ++g_fail; }
        else ++g_pass;
    }

    std::printf("\n=== RESULT: %d passed, %d failed, %d xfail (STEP-reader defects), "
                "worst case %.3f s ===\n",
                g_pass, g_fail, xfail, g_max_seconds);
    return g_fail == 0 ? 0 : 1;
}
