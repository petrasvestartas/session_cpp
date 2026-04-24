#include "mini_test.h"
#include "file_step.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"
#include <filesystem>
#include <cmath>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("FileStep", "NurbsCurve Round Trip") {
    std::filesystem::create_directories("./serialization");
    std::string path = "./serialization/test_step_nurbscurve.step";

    // Build a cubic non-rational curve with 4 CVs
    std::vector<Point> pts = {Point(0,0,0), Point(1,2,0), Point(2,2,0), Point(3,0,0)};
    NurbsCurve nc = NurbsCurve::create(false, 3, pts);
    MINI_CHECK(nc.is_valid());
    MINI_CHECK(nc.degree() == 3);
    MINI_CHECK(nc.cv_count() == 4);

    file_step::write_file_step_nurbscurves({nc}, path);
    MINI_CHECK(std::filesystem::exists(path));

    auto curves = file_step::read_file_step_nurbscurves(path);
    MINI_CHECK(curves.size() >= 1);
    const NurbsCurve& back = curves[0];
    MINI_CHECK(back.is_valid());
    MINI_CHECK(back.degree() == 3);
    MINI_CHECK(back.cv_count() == 4);
    MINI_CHECK(back.m_is_rat == 0);

    // Check knot round-trip
    auto kn_orig = nc.get_nurbsknots();
    auto kn_back = back.get_nurbsknots();
    MINI_CHECK(kn_orig.size() == kn_back.size());
    for (size_t i = 0; i < kn_orig.size() && i < kn_back.size(); i++)
        MINI_CHECK(std::abs(kn_orig[i] - kn_back[i]) < 1e-10);

    // Check CV round-trip
    for (int i = 0; i < 4; i++) {
        Point p_orig = nc.get_cv(i);
        Point p_back = back.get_cv(i);
        MINI_CHECK(std::abs(p_orig[0] - p_back[0]) < 1e-10);
        MINI_CHECK(std::abs(p_orig[1] - p_back[1]) < 1e-10);
        MINI_CHECK(std::abs(p_orig[2] - p_back[2]) < 1e-10);
    }
    std::filesystem::remove(path);
}

MINI_TEST("FileStep", "NurbsCurve Rational Round Trip") {
    std::filesystem::create_directories("./serialization");
    std::string path = "./serialization/test_step_nurbscurve_rat.step";

    // Build a degree-2 rational curve (quarter circle) with 3 CVs
    // Exact rational NURBS quarter arc: w = [1, cos(pi/4), 1], CVs at (1,0,0),(1,1,0),(0,1,0)
    NurbsCurve nc(3, true, 3, 3); // order=3 (degree-2), 3 CVs, rational
    double w_mid = std::cos(3.14159265358979323846 / 4.0);
    // knots: full = {0,0,0,1,1,1}, internal = {0,1} → m_nurbsknot size = order + cv_count - 2 = 3+3-2=4
    nc.m_nurbsknot = {0.0, 0.0, 1.0, 1.0};
    double* cv = nc.cv_array();
    // CV0: (1,0,0) w=1 → homogeneous (1,0,0,1)
    cv[0]=1.0; cv[1]=0.0; cv[2]=0.0; cv[3]=1.0;
    // CV1: (1,1,0) w=w_mid → homogeneous (w_mid, w_mid, 0, w_mid)
    cv[4]=w_mid*1.0; cv[5]=w_mid*1.0; cv[6]=0.0; cv[7]=w_mid;
    // CV2: (0,1,0) w=1 → homogeneous (0,1,0,1)
    cv[8]=0.0; cv[9]=1.0; cv[10]=0.0; cv[11]=1.0;

    MINI_CHECK(nc.is_valid());
    MINI_CHECK(nc.degree() == 2);
    MINI_CHECK(nc.cv_count() == 3);
    MINI_CHECK(nc.m_is_rat == 1);

    file_step::write_file_step_nurbscurves({nc}, path);
    MINI_CHECK(std::filesystem::exists(path));

    auto curves = file_step::read_file_step_nurbscurves(path);
    MINI_CHECK(curves.size() >= 1);
    const NurbsCurve& back = curves[0];
    MINI_CHECK(back.is_valid());
    MINI_CHECK(back.degree() == 2);
    MINI_CHECK(back.cv_count() == 3);
    MINI_CHECK(back.m_is_rat == 1);

    // Check CVs and weights survive round-trip
    const double* cv_back = back.m_cv.data();
    int s = back.m_cv_stride;
    for (int i = 0; i < 3; i++) {
        double w_orig = cv[i*4+3];
        double w_back = cv_back[i*s+3];
        MINI_CHECK(std::abs(w_orig - w_back) < 1e-10);
        if (std::abs(w_orig) > 1e-12 && std::abs(w_back) > 1e-12) {
            MINI_CHECK(std::abs(cv[i*4+0]/w_orig - cv_back[i*s+0]/w_back) < 1e-10);
            MINI_CHECK(std::abs(cv[i*4+1]/w_orig - cv_back[i*s+1]/w_back) < 1e-10);
        }
    }
    std::filesystem::remove(path);
}

MINI_TEST("FileStep", "NurbsSurface Round Trip") {
    std::filesystem::create_directories("./serialization");
    std::string path = "./serialization/test_step_nurbssurface.step";

    // Build a bicubic patch 4x4 CVs
    std::vector<Point> pts;
    for (int u = 0; u < 4; u++)
        for (int v = 0; v < 4; v++)
            pts.emplace_back((double)u, (double)v, std::sin(u + v) * 0.5);
    NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);
    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.degree(0) == 3);
    MINI_CHECK(srf.degree(1) == 3);
    MINI_CHECK(srf.cv_count(0) == 4);
    MINI_CHECK(srf.cv_count(1) == 4);

    file_step::write_file_step_nurbssurfaces({srf}, path);
    MINI_CHECK(std::filesystem::exists(path));

    auto surfaces = file_step::read_file_step_nurbssurfaces(path);
    MINI_CHECK(surfaces.size() >= 1);
    const NurbsSurface& back = surfaces[0];
    MINI_CHECK(back.is_valid());
    MINI_CHECK(back.degree(0) == 3);
    MINI_CHECK(back.degree(1) == 3);
    MINI_CHECK(back.cv_count(0) == 4);
    MINI_CHECK(back.cv_count(1) == 4);
    MINI_CHECK(back.m_is_rat == 0);

    // Check knot round-trip
    auto ku_orig = srf.m_nurbsknot[0];
    auto kv_orig = srf.m_nurbsknot[1];
    auto ku_back = back.m_nurbsknot[0];
    auto kv_back = back.m_nurbsknot[1];
    MINI_CHECK(ku_orig.size() == ku_back.size());
    MINI_CHECK(kv_orig.size() == kv_back.size());
    for (size_t i = 0; i < ku_orig.size() && i < ku_back.size(); i++)
        MINI_CHECK(std::abs(ku_orig[i] - ku_back[i]) < 1e-10);

    // Check CV round-trip
    for (int u = 0; u < 4; u++) for (int v = 0; v < 4; v++) {
        Point p_orig = srf.get_cv(u, v);
        Point p_back = back.get_cv(u, v);
        MINI_CHECK(std::abs(p_orig[0] - p_back[0]) < 1e-10);
        MINI_CHECK(std::abs(p_orig[1] - p_back[1]) < 1e-10);
        MINI_CHECK(std::abs(p_orig[2] - p_back[2]) < 1e-10);
    }
    std::filesystem::remove(path);
}

MINI_TEST("FileStep", "NurbsSurface Rational Round Trip") {
    std::filesystem::create_directories("./serialization");
    std::string path = "./serialization/test_step_nurbssurface_rat.step";

    // Build a rational bicubic surface 3x3 CVs (bilinear rational patch with uniform weights)
    NurbsSurface srf(3, true, 3, 3, 3, 3);
    // Set clamped knots: internal = {0,1} for degree-2 with 3 CVs → nurbsknot size=3+3-2=4
    srf.m_nurbsknot[0] = {0.0, 0.0, 1.0, 1.0};
    srf.m_nurbsknot[1] = {0.0, 0.0, 1.0, 1.0};
    double w = 0.8;
    for (int u = 0; u < 3; u++) for (int v = 0; v < 3; v++) {
        double x = (double)u, y = (double)v, z = std::sin(u+v)*0.3;
        srf.set_cv_4d(u, v, w*x, w*y, w*z, w);
    }
    MINI_CHECK(srf.is_valid());
    MINI_CHECK(srf.m_is_rat == 1);

    file_step::write_file_step_nurbssurfaces({srf}, path);
    MINI_CHECK(std::filesystem::exists(path));

    auto surfaces = file_step::read_file_step_nurbssurfaces(path);
    MINI_CHECK(surfaces.size() >= 1);
    const NurbsSurface& back = surfaces[0];
    MINI_CHECK(back.is_valid());
    MINI_CHECK(back.degree(0) == 2);
    MINI_CHECK(back.degree(1) == 2);
    MINI_CHECK(back.cv_count(0) == 3);
    MINI_CHECK(back.cv_count(1) == 3);
    MINI_CHECK(back.m_is_rat == 1);

    // Check weight round-trip
    for (int u = 0; u < 3; u++) for (int v = 0; v < 3; v++) {
        double x1, y1, z1, w1, x2, y2, z2, w2;
        srf.get_cv_4d(u, v, x1, y1, z1, w1);
        back.get_cv_4d(u, v, x2, y2, z2, w2);
        MINI_CHECK(std::abs(w1 - w2) < 1e-10);
        if (std::abs(w1) > 1e-12 && std::abs(w2) > 1e-12) {
            MINI_CHECK(std::abs(x1/w1 - x2/w2) < 1e-10);
            MINI_CHECK(std::abs(y1/w1 - y2/w2) < 1e-10);
        }
    }
    std::filesystem::remove(path);
}

MINI_TEST("FileStep", "NurbsSurfaceTrimmed Round Trip") {
    std::filesystem::create_directories("./serialization");
    std::string path = "./serialization/test_step_nurbssurface_trimmed.step";

    // Build a bicubic surface
    std::vector<Point> pts;
    for (int u = 0; u < 4; u++)
        for (int v = 0; v < 4; v++)
            pts.emplace_back((double)u, (double)v, 0.0);
    NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);

    // Build a 2D outer trim loop as a closed square in UV: (0,0)→(1,0)→(1,1)→(0,1)→(0,0)
    std::vector<Point> loop_pts = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0), Point(0,0,0)};
    NurbsCurve outer(2, false, 2, 5);
    outer.m_nurbsknot = {0.0, 1.0, 2.0, 3.0, 4.0};
    double* cv = outer.cv_array();
    for (int i = 0; i < 5; i++) { cv[i*2+0] = loop_pts[i][0]; cv[i*2+1] = loop_pts[i][1]; }
    MINI_CHECK(outer.is_valid());

    NurbsSurfaceTrimmed trimmed = NurbsSurfaceTrimmed::create(srf, outer);
    MINI_CHECK(trimmed.m_surface.is_valid());

    file_step::write_file_step_nurbssurfaces_trimmed({trimmed}, path);
    MINI_CHECK(std::filesystem::exists(path));

    // Verify the written file contains a valid STEP file by reading surfaces back
    auto surfaces = file_step::read_file_step_nurbssurfaces(path);
    MINI_CHECK(surfaces.size() >= 1);
    const NurbsSurface& back_srf = surfaces[0];
    MINI_CHECK(back_srf.is_valid());
    MINI_CHECK(back_srf.degree(0) == 3);
    MINI_CHECK(back_srf.degree(1) == 3);
    MINI_CHECK(back_srf.cv_count(0) == 4);
    MINI_CHECK(back_srf.cv_count(1) == 4);

    // Verify the trim curve (outer loop) is also in the file
    auto ncurves = file_step::read_file_step_nurbscurves(path);
    MINI_CHECK(ncurves.size() >= 1);

    std::filesystem::remove(path);
}

MINI_TEST("FileStep", "BRep Read Schoring") {
    std::string step_path = "session_data/elements/schoring_foot_0.step";
    if (!std::filesystem::exists(step_path)) return;

    auto breps = file_step::read_file_step_breps(step_path);
    MINI_CHECK(breps.size() == 3);

    size_t total_faces = 0, total_edges = 0, total_verts = 0;
    for (const auto& b : breps) {
        total_faces += b.face_count();
        total_edges += b.edge_count();
        total_verts += b.vertex_count();
    }
    MINI_CHECK(total_faces == 38);
    MINI_CHECK(total_edges == 103);
    MINI_CHECK(total_verts == 74);

    for (const auto& b : breps) {
        MINI_CHECK(b.is_valid());
        MINI_CHECK(b.m_surfaces.size() == (size_t)b.face_count());
        MINI_CHECK(b.m_curves_3d.size() == (size_t)b.edge_count());
        MINI_CHECK(b.m_curves_2d.size() == b.m_trims.size());
    }

    // Verify point count via read_file_step_points
    auto pts = file_step::read_file_step_points(step_path);
    MINI_CHECK(pts.size() == 350);
}

} // namespace session_cpp
