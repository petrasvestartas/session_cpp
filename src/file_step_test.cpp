#include "mini_test.h"
#include "file_step.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"
#include "tolerance.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    /// Path of a file in the serialization folder, created when missing.
    static std::string serialization_path(const std::string& name) {

        std::filesystem::create_directories("serialization");

        return "serialization/" + name;
    }

    MINI_TEST("FileStep", "NurbsCurve Round Trip") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::Point;
        // using session_cpp::file_step;

        const std::string path = serialization_path("test_step_nurbscurve.step");
        const std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(1, 2, 0),
            Point(2, 2, 0),
            Point(3, 0, 0),
        };
        const NurbsCurve nc = NurbsCurve::create(false, 3, pts);

        MINI_CHECK(nc.is_valid());
        MINI_CHECK(nc.degree() == 3);
        MINI_CHECK(nc.cv_count() == 4);

        MINI_CHECK(file_step::write_file_step_nurbscurves({nc}, path));
        MINI_CHECK(!file_step::write_file_step_nurbscurves({nc}, serialization_path("missing/test_step_nurbscurve.step")));
        MINI_CHECK(std::filesystem::exists(path));

        const std::vector<NurbsCurve> curves = file_step::read_file_step_nurbscurves(path);

        MINI_CHECK(curves.size() >= 1);

        const NurbsCurve& back = curves[0];

        MINI_CHECK(back.is_valid());
        MINI_CHECK(back.degree() == 3);
        MINI_CHECK(back.cv_count() == 4);
        MINI_CHECK(back.m_is_rat == 0);

        const std::vector<double> kn_orig = nc.get_nurbsknots();
        const std::vector<double> kn_back = back.get_nurbsknots();

        MINI_CHECK(kn_orig.size() == kn_back.size());

        for (size_t i = 0; i < kn_orig.size() && i < kn_back.size(); i++)
            MINI_CHECK(std::abs(kn_orig[i] - kn_back[i]) < 1e-10);

        for (int i = 0; i < 4; i++) {
            const Point p_orig = nc.get_cv(i);
            const Point p_back = back.get_cv(i);

            MINI_CHECK(std::abs(p_orig[0] - p_back[0]) < 1e-10);
            MINI_CHECK(std::abs(p_orig[1] - p_back[1]) < 1e-10);
            MINI_CHECK(std::abs(p_orig[2] - p_back[2]) < 1e-10);
        }

        std::filesystem::remove(path);
    }

    MINI_TEST("FileStep", "NurbsCurve Rational Round Trip") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::file_step;

        const std::string path = serialization_path("test_step_nurbscurve_rat.step");
        NurbsCurve nc(3, true, 3, 3);
        const double w_mid = std::cos(Tolerance::PI / 4.0);
        nc.m_nurbsknot = {0.0, 0.0, 1.0, 1.0};
        double* cv = nc.cv_array();
        cv[0] = 1.0;
        cv[1] = 0.0;
        cv[2] = 0.0;
        cv[3] = 1.0;
        cv[4] = w_mid * 1.0;
        cv[5] = w_mid * 1.0;
        cv[6] = 0.0;
        cv[7] = w_mid;
        cv[8] = 0.0;
        cv[9] = 1.0;
        cv[10] = 0.0;
        cv[11] = 1.0;

        MINI_CHECK(nc.is_valid());
        MINI_CHECK(nc.degree() == 2);
        MINI_CHECK(nc.cv_count() == 3);
        MINI_CHECK(nc.m_is_rat == 1);

        MINI_CHECK(file_step::write_file_step_nurbscurves({nc}, path));
        MINI_CHECK(std::filesystem::exists(path));

        const std::vector<NurbsCurve> curves = file_step::read_file_step_nurbscurves(path);

        MINI_CHECK(curves.size() >= 1);

        const NurbsCurve& back = curves[0];

        MINI_CHECK(back.is_valid());
        MINI_CHECK(back.degree() == 2);
        MINI_CHECK(back.cv_count() == 3);
        MINI_CHECK(back.m_is_rat == 1);

        const double* cv_back = back.m_cv.data();
        const int s = back.m_cv_stride;

        for (int i = 0; i < 3; i++) {
            const double w_orig = cv[i * 4 + 3];
            const double w_back = cv_back[i * s + 3];

            MINI_CHECK(std::abs(w_orig - w_back) < 1e-10);

            if (std::abs(w_orig) > 1e-12 && std::abs(w_back) > 1e-12) {
                MINI_CHECK(std::abs(cv[i * 4 + 0] / w_orig - cv_back[i * s + 0] / w_back) < 1e-10);
                MINI_CHECK(std::abs(cv[i * 4 + 1] / w_orig - cv_back[i * s + 1] / w_back) < 1e-10);
            }
        }

        std::filesystem::remove(path);
    }

    MINI_TEST("FileStep", "NurbsSurface Round Trip") {
        // using session_cpp::NurbsSurface;
        // using session_cpp::Point;
        // using session_cpp::file_step;

        const std::string path = serialization_path("test_step_nurbssurface.step");
        std::vector<Point> pts;

        for (int u = 0; u < 4; u++)
            for (int v = 0; v < 4; v++)
                pts.emplace_back((double)u, (double)v, std::sin(u + v) * 0.5);

        const NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);

        MINI_CHECK(srf.is_valid());
        MINI_CHECK(srf.degree(0) == 3);
        MINI_CHECK(srf.degree(1) == 3);
        MINI_CHECK(srf.cv_count(0) == 4);
        MINI_CHECK(srf.cv_count(1) == 4);

        MINI_CHECK(file_step::write_file_step_nurbssurfaces({srf}, path));
        MINI_CHECK(std::filesystem::exists(path));

        const std::vector<NurbsSurface> surfaces = file_step::read_file_step_nurbssurfaces(path);

        MINI_CHECK(surfaces.size() >= 1);

        const NurbsSurface& back = surfaces[0];

        MINI_CHECK(back.is_valid());
        MINI_CHECK(back.degree(0) == 3);
        MINI_CHECK(back.degree(1) == 3);
        MINI_CHECK(back.cv_count(0) == 4);
        MINI_CHECK(back.cv_count(1) == 4);
        MINI_CHECK(back.m_is_rat == 0);

        const std::vector<double> ku_orig = srf.m_nurbsknot[0];
        const std::vector<double> kv_orig = srf.m_nurbsknot[1];
        const std::vector<double> ku_back = back.m_nurbsknot[0];
        const std::vector<double> kv_back = back.m_nurbsknot[1];

        MINI_CHECK(ku_orig.size() == ku_back.size());
        MINI_CHECK(kv_orig.size() == kv_back.size());

        for (size_t i = 0; i < ku_orig.size() && i < ku_back.size(); i++)
            MINI_CHECK(std::abs(ku_orig[i] - ku_back[i]) < 1e-10);

        for (int u = 0; u < 4; u++)
            for (int v = 0; v < 4; v++) {
                const Point p_orig = srf.get_cv(u, v);
                const Point p_back = back.get_cv(u, v);

                MINI_CHECK(std::abs(p_orig[0] - p_back[0]) < 1e-10);
                MINI_CHECK(std::abs(p_orig[1] - p_back[1]) < 1e-10);
                MINI_CHECK(std::abs(p_orig[2] - p_back[2]) < 1e-10);
            }

        std::filesystem::remove(path);
    }

    MINI_TEST("FileStep", "NurbsSurface Rational Round Trip") {
        // using session_cpp::NurbsSurface;
        // using session_cpp::file_step;

        const std::string path = serialization_path("test_step_nurbssurface_rat.step");
        NurbsSurface srf(3, true, 3, 3, 3, 3);
        srf.m_nurbsknot[0] = {0.0, 0.0, 1.0, 1.0};
        srf.m_nurbsknot[1] = {0.0, 0.0, 1.0, 1.0};

        const double w = 0.8;

        for (int u = 0; u < 3; u++)
            for (int v = 0; v < 3; v++) {
                const double x = (double)u;
                const double y = (double)v;
                const double z = std::sin(u + v) * 0.3;
                srf.set_cv_4d(u, v, w * x, w * y, w * z, w);
            }

        MINI_CHECK(srf.is_valid());
        MINI_CHECK(srf.m_is_rat == 1);

        MINI_CHECK(file_step::write_file_step_nurbssurfaces({srf}, path));
        MINI_CHECK(std::filesystem::exists(path));

        const std::vector<NurbsSurface> surfaces = file_step::read_file_step_nurbssurfaces(path);

        MINI_CHECK(surfaces.size() >= 1);

        const NurbsSurface& back = surfaces[0];

        MINI_CHECK(back.is_valid());
        MINI_CHECK(back.degree(0) == 2);
        MINI_CHECK(back.degree(1) == 2);
        MINI_CHECK(back.cv_count(0) == 3);
        MINI_CHECK(back.cv_count(1) == 3);
        MINI_CHECK(back.m_is_rat == 1);

        for (int u = 0; u < 3; u++)
            for (int v = 0; v < 3; v++) {
                double x1 = 0.0;
                double y1 = 0.0;
                double z1 = 0.0;
                double w1 = 0.0;
                double x2 = 0.0;
                double y2 = 0.0;
                double z2 = 0.0;
                double w2 = 0.0;
                srf.get_cv_4d(u, v, x1, y1, z1, w1);
                back.get_cv_4d(u, v, x2, y2, z2, w2);

                MINI_CHECK(std::abs(w1 - w2) < 1e-10);

                if (std::abs(w1) > 1e-12 && std::abs(w2) > 1e-12) {
                    MINI_CHECK(std::abs(x1 / w1 - x2 / w2) < 1e-10);
                    MINI_CHECK(std::abs(y1 / w1 - y2 / w2) < 1e-10);
                }
            }

        std::filesystem::remove(path);
    }

    MINI_TEST("FileStep", "NurbsSurfaceTrimmed Round Trip") {
        // using session_cpp::NurbsCurve;
        // using session_cpp::NurbsSurface;
        // using session_cpp::NurbsSurfaceTrimmed;
        // using session_cpp::Point;
        // using session_cpp::file_step;

        const std::string path = serialization_path("test_step_nurbssurface_trimmed.step");
        std::vector<Point> pts;

        for (int u = 0; u < 4; u++)
            for (int v = 0; v < 4; v++)
                pts.emplace_back((double)u, (double)v, 0.0);

        const NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);
        const std::vector<Point> loop_pts = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
            Point(0, 0, 0),
        };
        NurbsCurve outer(2, false, 2, 5);
        outer.m_nurbsknot = {0.0, 1.0, 2.0, 3.0, 4.0};
        double* cv = outer.cv_array();

        for (int i = 0; i < 5; i++) {
            cv[i * 2 + 0] = loop_pts[i][0];
            cv[i * 2 + 1] = loop_pts[i][1];
        }

        MINI_CHECK(outer.is_valid());

        const NurbsSurfaceTrimmed trimmed = NurbsSurfaceTrimmed::create(srf, outer);

        MINI_CHECK(trimmed.m_surface.is_valid());

        MINI_CHECK(file_step::write_file_step_nurbssurfaces_trimmed({trimmed}, path));
        MINI_CHECK(std::filesystem::exists(path));

        const std::vector<NurbsSurface> surfaces = file_step::read_file_step_nurbssurfaces(path);

        MINI_CHECK(surfaces.size() >= 1);

        const NurbsSurface& back_srf = surfaces[0];

        MINI_CHECK(back_srf.is_valid());
        MINI_CHECK(back_srf.degree(0) == 3);
        MINI_CHECK(back_srf.degree(1) == 3);
        MINI_CHECK(back_srf.cv_count(0) == 4);
        MINI_CHECK(back_srf.cv_count(1) == 4);

        const std::vector<NurbsCurve> ncurves = file_step::read_file_step_nurbscurves(path);

        MINI_CHECK(ncurves.size() >= 1);

        std::filesystem::remove(path);
    }

    MINI_TEST("FileStep", "NurbsSurfaceTrimmed Read Vertex") {
        // using session_cpp::file_step;
        // using session_cpp::NurbsSurfaceTrimmed;
        // using session_cpp::Point;

        const std::string path = serialization_path("test_step_trimmed_vertex.step");
        const std::string text =
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
            "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
            "#2=CARTESIAN_POINT('',(0.,1.,0.));\n"
            "#3=CARTESIAN_POINT('',(1.,0.,0.));\n"
            "#4=CARTESIAN_POINT('',(1.,1.,0.));\n"
            "#5=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n"
            "#6=CARTESIAN_POINT('',(2.,3.,0.));\n"
            "#7=CARTESIAN_POINT('',(4.,5.,0.));\n"
            "#8=VERTEX_POINT('',#6);\n"
            "#9=VERTEX_POINT('',#7);\n"
            "#10=DIRECTION('',(1.,1.,0.));\n"
            "#11=VECTOR('',#10,1.);\n"
            "#12=LINE('',#6,#11);\n"
            "#13=EDGE_CURVE('',#8,#9,#12,.T.);\n"
            "#14=ORIENTED_EDGE('',*,*,#13,.T.);\n"
            "#15=EDGE_LOOP('',(#14));\n"
            "#16=FACE_OUTER_BOUND('',#15,.T.);\n"
            "#17=ADVANCED_FACE('',(#16),#5,.T.);\n"
            "ENDSEC;\nEND-ISO-10303-21;\n";

        MINI_CHECK((std::ofstream(path) << text).good());

        const std::vector<NurbsSurfaceTrimmed> trimmed = file_step::read_file_step_nurbssurfaces_trimmed(path);

        MINI_CHECK(trimmed.size() == 1);
        MINI_CHECK(trimmed[0].m_outer_loop.cv_count() == 2);

        const Point start = trimmed[0].m_outer_loop.get_cv(0);
        const Point end = trimmed[0].m_outer_loop.get_cv(1);

        MINI_CHECK(std::abs(start[0] - 2.0) < 1e-10);
        MINI_CHECK(std::abs(start[1] - 3.0) < 1e-10);
        MINI_CHECK(std::abs(end[0] - 4.0) < 1e-10);
        MINI_CHECK(std::abs(end[1] - 5.0) < 1e-10);

        std::filesystem::remove(path);
    }

    MINI_TEST("FileStep", "BRep Read Schoring") {
        // using session_cpp::file_step;
        // using session_cpp::BRep;
        // using session_cpp::BRepEdge;
        // using session_cpp::Point;

        const std::filesystem::path step_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
            "session_data" / "elements" / "schoring_foot_0.step";

        if (!std::filesystem::exists(step_path))
            return;

        const std::vector<BRep> breps = file_step::read_file_step_breps(step_path.string());

        MINI_CHECK(breps.size() == 3);

        size_t total_faces = 0;
        size_t total_edges = 0;
        size_t total_verts = 0;

        for (const BRep& b : breps) {
            total_faces += b.face_count();
            total_edges += b.edge_count();
            total_verts += b.vertex_count();
        }

        MINI_CHECK(total_faces == 38);
        MINI_CHECK(total_edges == 103);
        MINI_CHECK(total_verts == 74);

        for (const BRep& b : breps) {
            MINI_CHECK(b.is_valid());
            MINI_CHECK(b.m_surfaces.size() == (size_t)b.face_count());
            MINI_CHECK(b.m_curves_3d.size() == (size_t)b.edge_count());
            MINI_CHECK(b.shell_count() == 1 && b.solid_count() == 1);

            for (const BRepEdge& e : b.m_edges)
                MINI_CHECK(!e.pcurves.empty());
        }

        const std::vector<Point> pts = file_step::read_file_step_points(step_path.string());

        MINI_CHECK(pts.size() == 350);
    }

    MINI_TEST("FileStep", "BRep Round Trip") {
        // using session_cpp::BRep;
        // using session_cpp::file_step;

        const std::string path = serialization_path("test_brep_roundtrip.step");
        BRep cyl = BRep::create_cylinder(1.0, 2.0);
        cyl.name = "cylinder";

        MINI_CHECK(file_step::write_file_step_brep(cyl, path));

        const std::vector<BRep> breps = file_step::read_file_step_breps(path);

        MINI_CHECK(breps.size() == 1);
        MINI_CHECK(breps[0].is_valid());
        MINI_CHECK(breps[0].face_count() == 3);
        MINI_CHECK(breps[0].edge_count() == 3);
        MINI_CHECK(breps[0].vertex_count() == 2);
        MINI_CHECK(breps[0].is_solid());
        MINI_CHECK(std::abs(breps[0].volume() - cyl.volume()) < 0.05 * cyl.volume());

        std::filesystem::remove(path);
    }

} // namespace session_cpp
