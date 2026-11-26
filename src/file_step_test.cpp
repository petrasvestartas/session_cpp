#include "mini_test.h"
#include "file_step.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"
#include "brep_measure.h"
#include "brep_acceptance.h"
#include "brep_classify.h"
#include "curve_surface_intersection.h"
#include <algorithm>
#include "tolerance.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    /// Path of a file in the serialization folder, created when missing.
    static std::string serialization_path(const std::string& name) {

        std::filesystem::create_directories("serialization");

        return "serialization/" + name;
    }

    /// STEP BReps as the NURBS their consumers read.
    static std::vector<BRep> nurbs_breps(std::vector<BRep> breps) {
        for (BRep& b : breps)
            b = b.as_nurbs();
        return breps;
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

        const std::vector<BRep> breps = nurbs_breps(file_step::read_file_step_breps(step_path.string()));

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

        const std::vector<BRep> breps = nurbs_breps(file_step::read_file_step_breps(path));

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

namespace session_cpp {
namespace {
void rhino_step_whole_pcurves(const BRep& shape) {
    size_t images = 0;
    for (const auto& edge : shape.m_edges) {
        MINI_CHECK(!edge.pcurves.empty());
        NurbsCurve world;
        if (edge.degenerated) {
            MINI_CHECK(edge.start_vertex == edge.end_vertex);
            const auto& point = shape.m_vertices.at(edge.start_vertex).point;
            world = NurbsCurve(3, false, 2, 2);
            world.m_nurbsknot={0, 1};
            world.set_cv(0, point);
            world.set_cv(1, point);
        } else
            world = shape.m_curves_3d.at(edge.curve_3d_index).nurbs();
        for (const auto& image : edge.pcurves) {
            MINI_CHECK(image.curve_2d_index>=0);
            const auto& surface = shape.m_surfaces.at(image.surface_index).nurbs();
            for (int pc_index : {image.curve_2d_index, image.curve_2d_index_2}) {
                if (pc_index < 0)
                    continue;
                const auto& pc = shape.m_curves_2d.at(pc_index).nurbs();
                // The contract is normalized raw-parameter correspondence;
                // imported world and UV domains need not be identical.
                MINI_CHECK(curve_surface_intersection::same_parameter(world, pc, surface, 0.001) == curve_surface_intersection::ParameterMatch::Matched);
                ++images;
            }
        }
    }
    MINI_CHECK(images>0);
}

void rhino_step_input_contract(const std::string& family) {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path()/"tests"/"fixtures"/"rhino_booleans";
    std::ifstream manifest_stream(root/"manifest.json"),reference_stream(root/"compact-reference.json");
    MINI_CHECK(manifest_stream.good());
    MINI_CHECK(reference_stream.good());
    nlohmann::json manifest,reference;
    manifest_stream>>manifest;
    reference_stream>>reference;
    const auto family_it = std::find_if(reference["families"].begin(),reference["families"].end(),[&](const auto& f){
            return f["family"] == family;
        });
    MINI_CHECK(family_it != reference["families"].end());
    const auto& oracle = *family_it;
    MINI_CHECK(oracle["compact_probes"].size() == 32);
    for(int operand = 0;operand<2;++operand) {
        const auto& identity = oracle["input_order"][operand];
        MINI_CHECK(identity["object_table_index"] == operand);
        const std::string relative = identity["qualified_step"];
        const auto evidence = std::find_if(manifest["files"].begin(),manifest["files"].end(),[&](const auto& f){
            return f["relative_step"] == relative;
        });
        MINI_CHECK(evidence != manifest["files"].end());
        MINI_CHECK((*evidence)["roundtrip_evidence_qualified"] == true);
        MINI_CHECK((*evidence)["original_object_guid"] == identity["object_guid"]);
        const auto imported = nurbs_breps(file_step::read_file_step_breps((root/relative).string(),0.001));
        MINI_CHECK(imported.size() == 1);
        const auto& shape = imported.at(0);
        MINI_CHECK(shape.is_valid());
        MINI_CHECK(shape.is_solid());
        for (const auto& shell : shape.m_shells) {
            MINI_CHECK(brep_acceptance::shell_vertex_links(shape, shell) ==
                       brep_acceptance::VertexLinks::Matched);
        }
        rhino_step_whole_pcurves(shape);
        const double expected = identity["signed_volume"].get<double>();
        const double allowance = 1e-7*std::abs(expected)+std::abs(identity["reported_volume_error"].get<double>())+(*evidence)["volume_delta"].get<double>();
        const double actual = brep_measure::volume(shape,1e-7);
        MINI_CHECK(std::isfinite(actual));
    MINI_CHECK(actual>0);
        MINI_CHECK(std::abs(actual-expected)<=allowance);
        for (const auto& probe : oracle["compact_probes"]) {
            const auto& xyz = probe["point"];
            const auto actual_state = brep_classify::point(shape,Point(xyz[0].get<double>(),xyz[1].get<double>(),xyz[2].get<double>()),0.001);
            const bool inside = probe["states"][operand].get<int>() == 1;
            MINI_CHECK(actual_state==(inside?brep_classify::State::Inside:brep_classify::State::Outside));
        }
    }
}
}
MINI_TEST("FileStep", "Rhino Boxes Strict Native Import") {
    rhino_step_input_contract("Boxes");
}
MINI_TEST("FileStep", "Rhino Cones Strict Native Import") {
    rhino_step_input_contract("Cones");
}
MINI_TEST("FileStep", "Rhino Cylinders Strict Native Import") {
    rhino_step_input_contract("Cylinders");
}
MINI_TEST("FileStep", "Rhino Pyramids Strict Native Import") {
    rhino_step_input_contract("Pyramids");
}
MINI_TEST("FileStep", "Rhino Spheres Strict Native Import") {
    rhino_step_input_contract("Spheres");
}
MINI_TEST("FileStep", "Rhino Torus Strict Native Import") {
    rhino_step_input_contract("Torus");
}
MINI_TEST("FileStep", "Rhino Twisted Strict Native Import") {
    rhino_step_input_contract("Twisted");
}

MINI_TEST("FileStep", "Oblique Affine Support Preserves Raw Chart") {
    // This determinant-one shear preserves the independent box volume eight.
    const auto shear = [](const Point& p) {
        return Point(32.0 + p[0] + 0.5 * p[1], -16.0 + p[1] + 0.25 * p[2], 8.0 + p[2]);
    };
    for (bool rational : {false, true}) {
        BRep source = BRep::create_box(2.0, 2.0, 2.0).as_nurbs();
        for (auto& vertex : source.m_vertices)
            vertex.point = shear(vertex.point);
        for (Curve& curve_ : source.m_curves_3d) { NurbsCurve curve = curve_.nurbs(); for (int i = 0; i < curve.cv_count(); ++i)
                MINI_CHECK(curve.set_cv(i, shear(curve.get_cv(i)))); curve_ = curve; }
        for (Surface& surface_ : source.m_surfaces) { NurbsSurface surface = surface_.nurbs();
            MINI_CHECK(surface.domain(0) == std::make_pair(0.0, 1.0));
            MINI_CHECK(surface.domain(1) == std::make_pair(0.0, 1.0));
            for (int i = 0; i < surface.cv_count(0); ++i)
                for (int j = 0; j < surface.cv_count(1); ++j)
                    MINI_CHECK(surface.set_cv(i, j, shear(surface.get_cv(i, j))));
            MINI_CHECK(surface.set_domain(0, -7.0, 13.0));
            MINI_CHECK(surface.set_domain(1, 32.0, 35.0));
            if (rational) {
                MINI_CHECK(surface.to_rational());
                for (int i = 0; i < 2; ++i)
                    for (int j = 0; j < 2; ++j) {
                        const Point p = surface.get_cv(i, j);
                        MINI_CHECK(surface.set_cv_4d(i, j, 2 * p[0], 2 * p[1], 2 * p[2], 2));
                    }
            }
        surface_ = surface; }
        for (Curve2d& curve_ : source.m_curves_2d) { NurbsCurve curve = curve_.nurbs(); for (int i = 0; i < curve.cv_count(); ++i) {
                const Point uv = curve.get_cv(i);
                MINI_CHECK(curve.set_cv(i, Point(-7.0 + 20.0 * uv[0], 32.0 + 3.0 * uv[1], 0.0)));
            } curve_ = curve; }
        const auto& support = source.m_surfaces.front().nurbs();
        const Vector eu = support.get_cv(1, 0) - support.get_cv(0, 0);
        const Vector ev = support.get_cv(0, 1) - support.get_cv(0, 0);
        MINI_CHECK(std::abs(eu.dot(ev)) > 0.0); // The Gram cross term is required.
        const auto frozen = source.jsondump();
        const std::string path = serialization_path(rational ? "oblique_raw_rational.step" : "oblique_raw.step");
        MINI_CHECK(file_step::write_file_step_brep(source, path));
        const auto imported = nurbs_breps(file_step::read_file_step_breps(path, 1e-7));
        MINI_CHECK(imported.size() == 1);
        const BRep& result = imported.front();
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.is_solid());
        MINI_CHECK(result.face_count() == 6);
        for (const auto& shell : result.m_shells)
            MINI_CHECK(brep_acceptance::shell_vertex_links(result, shell) == brep_acceptance::VertexLinks::Matched);
        size_t checked = 0;
        for (const auto& edge : result.m_edges) {
            MINI_CHECK(!edge.degenerated);
            const auto& world = result.m_curves_3d.at(edge.curve_3d_index).nurbs();
            for (const auto& image : edge.pcurves)
                for (int index : {image.curve_2d_index, image.curve_2d_index_2}) {
                    if (index < 0)
                        continue;
                    MINI_CHECK(curve_surface_intersection::same_parameter(
                        world, result.m_curves_2d.at(index).nurbs(), result.m_surfaces.at(image.surface_index).nurbs(), 1e-7) ==
                        curve_surface_intersection::ParameterMatch::Matched);
                    ++checked;
                }
        }
        MINI_CHECK(checked == 24);
        MINI_CHECK(std::abs(brep_measure::volume(result, 1e-8) - 8.0) < 1e-6);
        MINI_CHECK(source.jsondump() == frozen);
        std::filesystem::remove(path);
    }
}

}

namespace session_cpp {
namespace {
std::string step_original_fixture(const std::string& name) {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "tests/fixtures/rhino_booleans/inputs" / (name + ".step");
    std::ifstream input(path);
    MINI_CHECK(input.good());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}
void step_replace_original(std::string& text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    MINI_CHECK(at != std::string::npos);
    if (at != std::string::npos) text.replace(at, from.size(), to);
}
std::vector<BRep> step_import_proposal(const std::string& text, const std::string& name) {
    const auto path = serialization_path(name);
    std::ofstream output(path); output << text; output.close();
    MINI_CHECK(output.good());
    const auto shapes = nurbs_breps(file_step::read_file_step_breps(path, .001));
    std::filesystem::remove(path);
    return shapes;
}
}
MINI_TEST("FileStep", "Original Like Seam PC Selection Ignores Association Order") {
    for (const std::string family : {"Cones", "Torus"}) {
        auto source = step_original_fixture("Input" + family + "-object-0");
        const auto baseline = step_import_proposal(source, "source_pc_before_" + family + ".step");
        if (family == "Cones")
            step_replace_original(source, "(#36,#38),.PCURVE_S1.", "(#38,#36),.PCURVE_S1.");
        else {
            step_replace_original(source, "(#31,#33),.PCURVE_S1.", "(#33,#31),.PCURVE_S1.");
            step_replace_original(source, "(#32,#34),.PCURVE_S1.", "(#34,#32),.PCURVE_S1.");
        }
        const auto reordered = step_import_proposal(source, "source_pc_after_" + family + ".step");
        MINI_CHECK(baseline.size() == 1 && reordered.size() == 1);
        if (baseline.size() != 1 || reordered.size() != 1) continue;
        const auto& a = baseline.front(); const auto& b = reordered.front();
        MINI_CHECK(a.is_valid() && a.is_solid() && b.is_valid() && b.is_solid());
        rhino_step_whole_pcurves(a); rhino_step_whole_pcurves(b);
        MINI_CHECK(a.edge_count() == b.edge_count() && a.face_count() == b.face_count());
        int aliases = 0;
        for (const auto& edge : b.m_edges) for (const auto& image : edge.pcurves) {
            const auto& surface = b.m_surfaces.at(image.surface_index).nurbs();
            if (surface.degree(0) != 2) continue;
            for (int index : {image.curve_2d_index, image.curve_2d_index_2}) if (index >= 0) {
                const auto& pc = b.m_curves_2d.at(index).nurbs();
                MINI_CHECK(pc.degree() == 1 && pc.cv_count() == 2);
            }
            if (image.curve_2d_index_2 < 0) continue;
            const auto& pc = b.m_curves_2d.at(image.curve_2d_index).nurbs();
            const auto& other = b.m_curves_2d.at(image.curve_2d_index_2).nurbs();
            const auto first = pc.point_at_start(), second = other.point_at_start();
            // Two distinct ORIGINAL chart aliases, not two copies of a projected trace.
            MINI_CHECK(first.distance(second) > 1);
            ++aliases;
        }
        MINI_CHECK(aliases == (family == "Cones" ? 1 : 2));
        for (const auto& shell : b.m_shells)
            MINI_CHECK(brep_acceptance::shell_vertex_links(b, shell) == brep_acceptance::VertexLinks::Matched);
        // The exact 3D curve control data does not depend on associated-PC ordering.
        MINI_CHECK(a.m_curves_3d.size() == b.m_curves_3d.size());
        for (size_t i = 0; i < a.m_curves_3d.size() && i < b.m_curves_3d.size(); ++i) {
            MINI_CHECK(a.m_curves_3d[i].nurbs().m_cv == b.m_curves_3d[i].nurbs().m_cv);
            MINI_CHECK(a.m_curves_3d[i].nurbs().m_nurbsknot == b.m_curves_3d[i].nurbs().m_nurbsknot);
        }
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("FileStep", "Original Trimmed Sphere Circle Retains Exact Sense") {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "tests/fixtures/rhino_booleans/inputs/InputSpheres-object-0.step";
    std::ifstream input(path); MINI_CHECK(input.good());
    const std::string original(std::istreambuf_iterator<char>(input), {});
    for (int variant = 0; variant < 3; ++variant) {
        std::string text = original;
        if (variant > 0) {
            const std::string from = "TRIMMED_CURVE($,#38,(#119),(#120),.T.,.CARTESIAN.)";
            const std::string to = variant == 1 ?
                "TRIMMED_CURVE($,#38,(#120),(#119),.F.,.CARTESIAN.)" :
                "TRIMMED_CURVE($,#38,(#119),(#120),.F.,.CARTESIAN.)";
            const auto at = text.find(from); MINI_CHECK(at != std::string::npos);
            if (at != std::string::npos) text.replace(at, from.size(), to);
            if (variant == 1) {
                const std::string edge = "EDGE_CURVE('',#39,#40,#35,.T.)";
                const auto at_edge = text.find(edge); MINI_CHECK(at_edge != std::string::npos);
                if (at_edge != std::string::npos)
                    text.replace(at_edge, edge.size(), "EDGE_CURVE('',#39,#40,#35,.F.)");
            }
        }
        const auto temporary = serialization_path("original_trimmed_circle_" + std::to_string(variant) + ".step");
        std::ofstream output(temporary); output << text; output.close(); MINI_CHECK(output.good());
        const auto shapes = nurbs_breps(file_step::read_file_step_breps(temporary, .001));
        std::filesystem::remove(temporary);
        MINI_CHECK(shapes.size() == 1); if (shapes.size() != 1) continue;
        const auto& shape = shapes.front();
        MINI_CHECK(std::count_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; }) == 1);
        const auto& curve = shape.m_curves_3d.at(std::find_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; })->curve_3d_index).nurbs();
        MINI_CHECK(curve.degree() == 2 && curve.m_is_rat != 0 && curve.cv_count() == 5);
        const Point center(-8.31850601603522, 164.175382362864, 0);
        const double radius = 16.1387051137751;
        MINI_CHECK(curve.point_at_start().distance(center + Vector(0,0,-radius)) < 1e-10);
        MINI_CHECK(curve.point_at_end().distance(center + Vector(0,0,radius)) < 1e-10);
        // Independent circle basis from original STEP axis: N cross X = (-ny,nx,0).
        const double nx = .871147005504842, ny = .491022295624085;
        const double norm = std::sqrt(nx*nx + ny*ny);
        const Vector side(-ny / norm, nx / norm, 0);
        const Point middle = center + side * (variant == 2 ? -radius : radius);
        MINI_CHECK(curve.point_at(curve.domain_middle()).distance(middle) < 1e-10);
        for (int k = 0; k <= 8; ++k) {
            const Point p = curve.point_at(curve.domain_start() +
                (curve.domain_end() - curve.domain_start()) * k / 8.);
            MINI_CHECK(std::abs(p.distance(center) - radius) < 1e-10);
        }
        // Variant2 deliberately changes the physical hemisphere; its stale PCs must
        // not be counted as a successful import of the original sphere.
    }
}

MINI_TEST("FileStep", "Original Circle Cap Retains Rational World Law") {
    const std::string original = step_original_fixture("InputCylinders-object-0");
    NurbsCurve original_world, original_cap;
    NurbsSurface original_support;
    for (int variant = 0; variant < 4; ++variant) {
        std::string text = original;
        if (variant == 1 || variant == 2)
            step_replace_original(text, "#103=DIRECTION('',(0.,1.,0.));",
                "#103=DIRECTION('',(1.,0.,0.));");
        if (variant == 1)
            step_replace_original(text,
                "#99=DIRECTION('',(-0.765256854353062,0.643725055334696,0.));",
                "#99=DIRECTION('',(-0.643725055334696,-0.765256854353062,0.));");
        if (variant == 3) {
            // Compensate the 2D frame's positive normal by reversing the world
            // circle normal. This checks the cap law, not whole-solid orientation.
            step_replace_original(text, "#98=DIRECTION('',(0.,0.,1.));",
                "#98=DIRECTION('',(0.,0.,-1.));");
            step_replace_original(text, "#93=AXIS2_PLACEMENT_3D($,#149,#102,#103);",
                "#93=AXIS2_PLACEMENT_2D($,#149,#103);");
            step_replace_original(text, "#149=CARTESIAN_POINT('',(0.,0.,0.));",
                "#149=CARTESIAN_POINT('',(0.,0.));");
            step_replace_original(text, "#103=DIRECTION('',(0.,1.,0.));",
                "#103=DIRECTION('',(0.,1.));");
        }
        const auto shapes = step_import_proposal(text, "original_circle_cap_" + std::to_string(variant) + ".step");
        MINI_CHECK(shapes.size() == 1);
        const auto& shape = shapes.front();
        MINI_CHECK(shape.is_valid() == (variant != 3)); // #98 reverses circle #62 of edge #59 but its pcurve #52 on the cylinder runs as before.
        const auto& edge = shape.m_edges.at(0);
        const auto& world = shape.m_curves_3d.at(edge.curve_3d_index).nurbs();
        MINI_CHECK(world.is_rational());
        MINI_CHECK(world.cv_count() == 9);
        size_t cap_images = 0;
        for (const auto& image : edge.pcurves) {
            if (image.surface_index != 1) continue;
            const auto match = curve_surface_intersection::same_parameter(world,
                shape.m_curves_2d.at(image.curve_2d_index).nurbs(), shape.m_surfaces.at(image.surface_index).nurbs(), .001);
            if (variant == 0) {
                original_world = world;
                original_cap = shape.m_curves_2d.at(image.curve_2d_index).nurbs();
                original_support = shape.m_surfaces.at(image.surface_index).nurbs();
            }
            if (variant != 2)
                MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Matched);
            if (variant == 2) {
                // Original PC frame N=-Z, X=+Y becomes X=+X: (u,v)->(v,-u).
                // This explicit candidate changes raw phase, not the circle locus.
                // A future importer may repair it; do not pin fallback failure.
                NurbsCurve shifted = original_cap;
                for (int i = 0; i < shifted.cv_count(); ++i) {
                    double x, y, z, w;
                    MINI_CHECK(shifted.get_cv_4d(i, x, y, z, w));
                    MINI_CHECK(shifted.set_cv_4d(i, y, -x, z, w));
                }
                MINI_CHECK(curve_surface_intersection::same_parameter(original_world,
                    shifted, original_support, .001) !=
                    curve_surface_intersection::ParameterMatch::Matched);
            }
            ++cap_images;
        }
        MINI_CHECK(cap_images == 1);
        if (variant < 2) rhino_step_whole_pcurves(shape);
    }
}

} // namespace session_cpp

namespace session_cpp {
MINI_TEST("FileStep", "Rounded Trimmed Sphere Quarter Proposal Preserves Original PCs") {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "tests/fixtures/rhino_booleans/inputs/InputSpheres-object-1.step";
    const auto shapes = nurbs_breps(file_step::read_file_step_breps(path.string(), .001));
    MINI_CHECK(shapes.size() == 1);
    if (shapes.size() != 1) return;
    const auto& shape = shapes.front();
    MINI_CHECK(shape.is_valid() && shape.is_solid() &&
        std::count_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; }) == 1);
    rhino_step_whole_pcurves(shape);
    const auto seam = std::find_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; });
    const auto& world = shape.m_curves_3d.at(seam->curve_3d_index).nurbs();
    MINI_CHECK(world.degree() == 2 && world.m_is_rat != 0 && world.cv_count() == 5);
    const Point center(2.15514933833421, 167.204904258093, 3.49070132425338);
    const double radius = 16.1387051137751;
    for (int k = 0; k <= 12; ++k) {
        const auto point = world.point_at(world.domain_start() +
            (world.domain_end() - world.domain_start()) * k / 12.);
        MINI_CHECK(std::abs(point.distance(center) - radius) < 1e-10);
    }
    int aliases = 0;
    for (const auto& image : seam->pcurves)
        for (int index : {image.curve_2d_index, image.curve_2d_index_2}) if (index >= 0) {
            const auto& pc = shape.m_curves_2d.at(index).nurbs();
            MINI_CHECK(pc.degree() == 1 && pc.cv_count() == 2);
            MINI_CHECK(pc.domain_start() == -25.3506187119439);
            MINI_CHECK(pc.domain_end() == 25.3506187119439);
            ++aliases;
        }
    MINI_CHECK(aliases == 2);
}

MINI_TEST("FileStep", "Trimmed Circle Quarter Proposal Keeps Distinct Small And Nonquarter Arcs") {
    // OPEN import diagnostics: no closed-solid or unchanged-PC acceptance claim.
    for (double angle : {Tolerance::PI + .01, 1e-11}) {
        std::ostringstream step;
        step << std::setprecision(17)
             << "ISO-10303-21;\nHEADER;ENDSEC;DATA;\n"
             << "#1=CARTESIAN_POINT('',(0.,0.,0.));\n"
             << "#2=DIRECTION('',(0.,0.,1.));#3=DIRECTION('',(1.,0.,0.));\n"
             << "#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);#5=CIRCLE('',#4,1.);\n"
             << "#6=CARTESIAN_POINT('',(1.,0.,0.));\n"
             << "#7=CARTESIAN_POINT('',(" << std::cos(angle) << ',' << std::sin(angle) << ",0.));\n"
             << "#8=TRIMMED_CURVE('',#5,(#6),(#7),.T.,.CARTESIAN.);\n"
             << "#9=VERTEX_POINT('',#6);#10=VERTEX_POINT('',#7);\n"
             << "#11=EDGE_CURVE('',#9,#10,#8,.T.);#12=ORIENTED_EDGE('',*,*,#11,.T.);\n"
             << "#13=EDGE_LOOP('',(#12));#14=FACE_OUTER_BOUND('',#13,.T.);\n"
             << "#15=PLANE('',#4);#16=ADVANCED_FACE('',(#14),#15,.T.);\n"
             << "#17=OPEN_SHELL('',(#16));#18=SHELL_BASED_SURFACE_MODEL('',(#17));\n"
             << "ENDSEC;END-ISO-10303-21;\n";
        const auto temporary = serialization_path(angle > 1 ? "trim_nonquarter.step" : "trim_small_open.step");
        std::ofstream output(temporary);
        output << step.str();
        output.close();
        MINI_CHECK(output.good());
        const auto shapes = file_step::read_file_step_breps(temporary, 1e-7);
        std::filesystem::remove(temporary);
        MINI_CHECK(shapes.size() == 1);
        if (shapes.size() != 1) continue;
        const auto& shape = shapes.front();
        MINI_CHECK(shape.edge_count() == 1);
        // The edge runs over its range of the file's circle, in the angle from the reference direction.
        MINI_CHECK(shape.m_curves_3d.at(shape.m_edges.front().curve_3d_index).kind() == Curve::Kind::Circle && shape.edge_range(0).first == 0.0);
        const NurbsCurve curve = shape.edge_nurbs(0);
        MINI_CHECK(curve.degree() == 2 && curve.m_is_rat != 0);
        MINI_CHECK(curve.cv_count() == (angle > 1 ? 7 : 3));
        MINI_CHECK(std::abs((curve.domain_end() - curve.domain_start()) - angle) < 1e-14);
        MINI_CHECK(curve.point_at_start().distance(Point(1,0,0)) < 1e-14);
        MINI_CHECK(curve.point_at_end().distance(Point(std::cos(angle),std::sin(angle),0)) < 1e-14);
        const auto middle = curve.point_at(curve.domain_middle());
        MINI_CHECK(middle.distance(Point(std::cos(angle*.5),std::sin(angle*.5),0)) < 1e-13);
        if (angle < 1) MINI_CHECK(curve.domain_end() - curve.domain_start() < 1e-9);
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("FileStep", "Original Sphere And Cone Pole Bridges Are Certified") {
    for (const std::string family : {"Spheres", "Cones"}) {
        for (int operand = 0; operand < 2; ++operand) {
            const auto text = step_original_fixture("Input" + family + "-object-" + std::to_string(operand));
            const auto shapes = step_import_proposal(text, "certified_poles_" + family + std::to_string(operand) + ".step");
            MINI_CHECK(shapes.size() == 1);
            const auto& shape = shapes.front();
            MINI_CHECK(shape.is_valid() && shape.is_solid());
            MINI_CHECK(std::count_if(shape.m_edges.begin(), shape.m_edges.end(),
                [](const auto& edge) { return edge.degenerated; }) == (family == "Spheres" ? 2 : 1));
            rhino_step_whole_pcurves(shape);
            for (const auto& face : shape.m_faces) {
                for (const auto& wire : face.wires) {
                    const auto edges = shape.wire_edges(wire);
                    for (size_t i = 0; i < edges.size(); ++i) {
                        const auto& first = edges[i];
                        const auto& next = edges[(i + 1) % edges.size()];
                        const auto& a = shape.m_curves_2d.at(shape.pcurve_index(first.index,
                            static_cast<int>(&face - shape.m_faces.data()), first.orientation)).nurbs();
                        const auto& b = shape.m_curves_2d.at(shape.pcurve_index(next.index,
                            static_cast<int>(&face - shape.m_faces.data()), next.orientation)).nurbs();
                        const Point end = first.orientation == BRepOrientation::Forward ? a.point_at_end() : a.point_at_start();
                        const Point start = next.orientation == BRepOrientation::Forward ? b.point_at_start() : b.point_at_end();
                        MINI_CHECK(end.distance(start) < 1e-10);
                    }
                }
            }
        }
    }
}
MINI_TEST("FileStep", "Noncollapsed Cylinder Boundary Is Not A Pole Bridge") {
    BRep source = BRep::create_cylinder(2, 3);
    const auto& support = source.m_surfaces.front().nurbs();
    const auto u = support.domain(0), v = support.domain(1);
    const Point point = support.point_at(u.first, v.first);
    NurbsCurve constant(3, false, 2, 2);
    constant.m_nurbsknot = {0, 1};
    MINI_CHECK(constant.set_cv(0, point));
    MINI_CHECK(constant.set_cv(1, point));
    const auto bridge = NurbsCurve::create(false, 1,
        {Point(u.first, v.first, 0), Point(u.second, v.first, 0)});
    MINI_CHECK(curve_surface_intersection::same_parameter(constant, bridge, support, .001) !=
        curve_surface_intersection::ParameterMatch::Matched);
    // Keep only the two opposite uses of the body seam. Its missing circular
    // boundaries are not collapsed, even though their endpoints share vertex IDs.
    auto& wire = source.m_wires.at(source.m_faces.front().wires.front().index);
    wire.edges.erase(std::remove_if(wire.edges.begin(), wire.edges.end(), [&](const auto& use) {
        const auto& edge = source.m_edges.at(use.index);
        return edge.start_vertex == edge.end_vertex;
    }), wire.edges.end());
    MINI_CHECK(wire.edges.size() == 2);
    source.m_shells.front().faces = {{0, BRepOrientation::Forward}};
    source.m_faces.erase(source.m_faces.begin() + 1, source.m_faces.end());
    const auto path = serialization_path("noncollapsed_cylinder_gap.step");
    MINI_CHECK(file_step::write_file_step_brep(source, path));
    const auto imported = nurbs_breps(file_step::read_file_step_breps(path, .001));
    std::filesystem::remove(path);
    MINI_CHECK(imported.size() == 1);
    MINI_CHECK(std::none_of(imported.front().m_edges.begin(), imported.front().m_edges.end(),
        [](const auto& edge) { return edge.degenerated; }));
}
}

namespace session_cpp {
namespace {
std::string step_fixture(const std::string& name) {
    std::ifstream input(std::filesystem::path(__FILE__).parent_path().parent_path() / "tests/fixtures/step" / name);
    MINI_CHECK(input.good());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}
std::vector<BRep> step_read_text(const std::string& text, const std::string& name) {
    const auto path = serialization_path(name);
    std::ofstream(path) << text;
    std::vector<BRep> shapes;
    try {
        shapes = nurbs_breps(file_step::read_file_step_breps(path));
    } catch (...) {
        std::filesystem::remove(path);
        throw;
    }
    std::filesystem::remove(path);
    return shapes;
}
int step_uncertified_pcurves(const BRep& shape) {
    int failed = 0;
    for (const auto& edge : shape.m_edges)
        for (const auto& image : edge.pcurves)
            for (int index : {image.curve_2d_index, image.curve_2d_index_2})
                if (index >= 0 && !edge.degenerated)
                    failed += curve_surface_intersection::same_parameter(shape.m_curves_3d.at(edge.curve_3d_index).nurbs(), shape.m_curves_2d.at(index).nurbs(),
                        shape.m_surfaces.at(image.surface_index).nurbs(), 1e-7) != curve_surface_intersection::ParameterMatch::Matched;
    return failed;
}
}
MINI_TEST("FileStep", "BRep Read Bound And Face Senses") {
    // DATA in a header string, FACE_BOUND .F. loops, ADVANCED_FACE .F. on inward planes and a FACE_SURFACE
    const auto shapes = step_read_text(step_fixture("senses_cube.step"), "senses_cube.step");
    MINI_CHECK(shapes.size() == 1);
    MINI_CHECK(shapes[0].face_count() == 6);
    MINI_CHECK(shapes[0].is_valid() && shapes[0].is_solid());
    MINI_CHECK(std::abs(shapes[0].volume_exact() - 1.0) < 1e-12);
    MINI_CHECK(step_uncertified_pcurves(shapes[0]) == 0);
}
MINI_TEST("FileStep", "BRep Read Unsupported Surface Throws") {
    std::string text = step_fixture("senses_cube.step");
    const size_t at = text.rfind("PLANE('',#");
    const size_t end = text.find(')', at);
    text.replace(end, 1, ",1.,.F.)");
    text.replace(at, 5, "OFFSET_SURFACE");
    bool thrown = false;
    try {
        step_read_text(text, "offset_cube.step");
    } catch (const std::runtime_error&) {
        thrown = true;
    }
    MINI_CHECK(thrown);
}
MINI_TEST("FileStep", "BRep Read Ellipse On Cylinder Exact") {
    const auto shapes = step_read_text(step_fixture("oblique_cylinder.step"), "oblique_cylinder.step");
    MINI_CHECK(shapes.size() == 1);
    MINI_CHECK(shapes[0].is_valid() && shapes[0].is_solid());
    const auto [volume, leak] = brep_measure::volume_closure(shapes[0]);
    MINI_CHECK(leak < 1e-8 && std::abs(volume - 2.0 * Tolerance::PI) < 1e-12 + leak);
    MINI_CHECK(step_uncertified_pcurves(shapes[0]) == 0);
}
MINI_TEST("FileStep", "BRep Read Loose Vertex Carries Tolerance") {
    std::string text = step_fixture("senses_cube.step");
    text.replace(text.find("(1.,1.,1.)"), 10, "(1.,1.,1.00001)");
    const auto shapes = step_read_text(text, "loose_cube.step");
    MINI_CHECK(shapes.size() == 1 && shapes[0].is_solid());
    double edge = 0.0;
    for (const auto& e : shapes[0].m_edges)
        edge = std::max(edge, e.tolerance);
    MINI_CHECK(std::abs(edge - 1e-5) < 1e-12);
    const auto [volume, leak] = brep_measure::volume_closure(shapes[0]);
    MINI_CHECK(leak < 1e-5 && std::abs(volume - 1.0) < 1e-12 + leak);
}
MINI_TEST("FileStep", "BRep Primitives Round Trip Exact") {
    for (const BRep& shape : {BRep::create_cylinder(1.0, 2.0), BRep::create_cone(1.0, 2.0), BRep::create_sphere(1.5), BRep::create_torus(2.0, 0.5),
                              BRep::create_block_with_hole(2.0, 2.0, 1.0, 0.3)}) {
        const std::string path = serialization_path("round_trip_" + shape.name + ".step");
        MINI_CHECK(file_step::write_file_step_brep(shape, path));
        const auto read = nurbs_breps(file_step::read_file_step_breps(path));
        MINI_CHECK(read.size() == 1 && read[0].is_solid());
        MINI_CHECK(std::abs(read[0].volume_exact() - shape.volume_exact()) < 1e-12 * shape.volume_exact());
        MINI_CHECK(step_uncertified_pcurves(read[0]) == 0);
        std::filesystem::remove(path);
    }
}
MINI_TEST("FileStep", "BRep Write Pcurves Round Trip Exact") {
    // SURFACE_CURVE and SEAM_CURVE carry the pcurves, so a second round trip reads the first one's BRep again
    const std::string first = serialization_path("pcurves_first.step");
    const std::string second = serialization_path("pcurves_second.step");
    MINI_CHECK(file_step::write_file_step_brep(BRep::create_cylinder(1.0, 2.0).as_nurbs(), first));
    std::ifstream input(first);
    const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    input.close();
    MINI_CHECK(text.find("SEAM_CURVE(") != std::string::npos && text.find("PCURVE(") != std::string::npos);
    const auto once = nurbs_breps(file_step::read_file_step_breps(first));
    MINI_CHECK(once.size() == 1 && file_step::write_file_step_brep(once[0], second));
    const auto twice = nurbs_breps(file_step::read_file_step_breps(second));
    MINI_CHECK(twice.size() == 1 && twice[0].is_solid());
    MINI_CHECK(twice[0].volume_exact() == once[0].volume_exact());
    MINI_CHECK(twice[0].m_curves_2d.size() == once[0].m_curves_2d.size());
    std::filesystem::remove(first);
    std::filesystem::remove(second);
}
MINI_TEST("FileStep", "BRep Read Seamless Loops Seam At Angle Nearest Zero") {
    // circles with vertices at angles 0.3 and -2: the seam runs from the first, the second is split there
    const std::string path = serialization_path("seamless_cylinder.step");
    std::ofstream(path) << step_fixture("seamless_cylinder.step");
    std::vector<file_step::StepIds> ids;
    const auto shapes = nurbs_breps(file_step::read_file_step_breps(path, 1e-7, &ids));
    std::filesystem::remove(path);
    MINI_CHECK(shapes.size() == 1 && ids.size() == 1);
    MINI_CHECK(shapes[0].is_valid() && shapes[0].is_solid());
    MINI_CHECK(std::abs(shapes[0].volume_exact() - 2.0 * Tolerance::PI) < 1e-12);
    MINI_CHECK(step_uncertified_pcurves(shapes[0]) == 0);
    const auto& side = shapes[0].m_faces[1];
    std::vector<int> wire;
    for (const auto& edge : shapes[0].wire_edges(side.wires[0]))
        wire.push_back(ids[0].edges[edge.index]);
    std::sort(wire.begin(), wire.end());
    MINI_CHECK(ids[0].faces == std::vector<int>({25, 46, 34}));
    MINI_CHECK(wire == std::vector<int>({-16, -16, 0, 0, 8}));
}
MINI_TEST("FileStep", "BRep Read Hole Across The Seam Joins The Loop") {
    // a window across the seam angle 0.3, closed by a face of its own: its arcs are split on the seam, which runs up to it and on from it
    const std::string path = serialization_path("seamless_window.step");
    std::ofstream(path) << step_fixture("seamless_window.step");
    std::vector<file_step::StepIds> ids;
    const auto shapes = nurbs_breps(file_step::read_file_step_breps(path, 1e-7, &ids));
    std::filesystem::remove(path);
    MINI_CHECK(shapes.size() == 1 && shapes[0].is_valid() && shapes[0].is_solid());
    MINI_CHECK(std::abs(shapes[0].volume_exact() - 2.0 * Tolerance::PI) < 1e-12);
    MINI_CHECK(step_uncertified_pcurves(shapes[0]) == 0);
    const auto& side = shapes[0].m_faces[1];
    std::vector<int> wire;
    for (const auto& edge : shapes[0].wire_edges(side.wires[0]))
        wire.push_back(ids[0].edges[edge.index]);
    std::sort(wire.begin(), wire.end());
    MINI_CHECK(ids[0].faces == std::vector<int>({25, 89, 34, 82}) && side.wires.size() == 1);
    MINI_CHECK(wire == std::vector<int>({-64, -64, -53, -53, -16, -16, 0, 0, 0, 0, 8, 58, 69}));
}
MINI_TEST("FileStep", "BRep Read Loose Circle On Cone Carries Its Distance") {
    // a circle 1e-4 wider than the cone is 1e-4 cos(semi-angle) from it along the normal
    const auto shapes = step_read_text(step_fixture("loose_cone.step"), "loose_cone.step");
    MINI_CHECK(shapes.size() == 1 && shapes[0].is_solid());
    double edge = 0.0;
    for (const auto& e : shapes[0].m_edges)
        edge = std::max(edge, e.tolerance);
    MINI_CHECK(std::abs(edge - 1e-4 * std::cos(std::atan(0.5))) < 1e-12);
}
MINI_TEST("FileStep", "BRep Read Entity Ids And Counts") {
    const std::string path = serialization_path("ids_cube.step");
    std::ofstream(path) << step_fixture("senses_cube.step");
    std::vector<file_step::StepIds> ids;
    const auto shapes = nurbs_breps(file_step::read_file_step_breps(path, 1e-7, &ids));
    const auto counts = file_step::read_file_step_entity_counts(path);
    std::filesystem::remove(path);
    MINI_CHECK(shapes.size() == 1 && ids.size() == 1);
    MINI_CHECK(ids[0].faces == std::vector<int>({44, 72, 92, 108, 124, 136}));
    MINI_CHECK(ids[0].edges.size() == shapes[0].m_edges.size());
    MINI_CHECK(std::count(ids[0].edges.begin(), ids[0].edges.end(), 0) == 0);
    MINI_CHECK(counts.at("ADVANCED_FACE") == 5 && counts.at("FACE_SURFACE") == 1);
}
MINI_TEST("FileStep", "BRep Read Assembly Places Each Instance") {
    // the cube placed twice under an assembly, the second turned a quarter about z, as STEPControl_ActorRead transfers it
    std::string text = step_fixture("senses_cube.step");
    text.insert(text.rfind("ENDSEC;"),
        "#900=CARTESIAN_POINT('',(0.,0.,0.));#901=DIRECTION('',(0.,0.,1.));#902=DIRECTION('',(1.,0.,0.));#903=AXIS2_PLACEMENT_3D('',#900,#901,#902);\n"
        "#904=CARTESIAN_POINT('',(10.,0.,0.));#905=AXIS2_PLACEMENT_3D('',#904,#901,#902);\n"
        "#906=CARTESIAN_POINT('',(0.,5.,0.));#907=DIRECTION('',(0.,1.,0.));#908=AXIS2_PLACEMENT_3D('',#906,#901,#907);\n"
        "#910=PRODUCT_DEFINITION('assembly','',$,$);#911=PRODUCT_DEFINITION('cube','',$,$);\n"
        "#912=PRODUCT_DEFINITION_SHAPE('','',#910);#913=PRODUCT_DEFINITION_SHAPE('','',#911);\n"
        "#914=SHAPE_REPRESENTATION('',(#903,#905,#908),#143);\n"
        "#915=SHAPE_DEFINITION_REPRESENTATION(#912,#914);#916=SHAPE_DEFINITION_REPRESENTATION(#913,#144);\n"
        "#917=NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#910,#911,$);#918=PRODUCT_DEFINITION_SHAPE('','',#917);\n"
        "#919=ITEM_DEFINED_TRANSFORMATION('','',#903,#905);\n"
        "#920=(REPRESENTATION_RELATIONSHIP('','',#144,#914)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#919)SHAPE_REPRESENTATION_RELATIONSHIP());\n"
        "#921=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#920,#918);\n"
        "#922=NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#910,#911,$);#923=PRODUCT_DEFINITION_SHAPE('','',#922);\n"
        "#924=ITEM_DEFINED_TRANSFORMATION('','',#903,#908);\n"
        "#925=(REPRESENTATION_RELATIONSHIP('','',#144,#914)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#924)SHAPE_REPRESENTATION_RELATIONSHIP());\n"
        "#926=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#925,#923);\n");
    const auto shapes = step_read_text(text, "assembly_cube.step");
    MINI_CHECK(shapes.size() == 2);
    const std::array<std::array<double, 4>, 2> boxes = {{{10, 11, 0, 1}, {-1, 0, 5, 6}}};
    for (size_t i = 0; i < shapes.size(); i++) {
        double box[4] = {INFINITY, -INFINITY, INFINITY, -INFINITY};
        for (const auto& v : shapes[i].m_vertices)
            for (int d = 0; d < 2; d++) {
                box[2 * d] = std::min(box[2 * d], v.point[d]);
                box[2 * d + 1] = std::max(box[2 * d + 1], v.point[d]);
            }
        for (int k = 0; k < 4; k++)
            MINI_CHECK(std::abs(box[k] - boxes[i][k]) < 1e-12);
        MINI_CHECK(shapes[i].is_solid() && std::abs(shapes[i].volume_exact() - 1.0) < 1e-12);
    }
}
MINI_TEST("FileStep", "BRep Write Assembly Places Each Instance") {
    // one cube written once and placed twice, the second turned a quarter about z; read back with its part and placements, each
    // instance the cube as a file of its own reads it, placed
    const auto cube = step_read_text(step_fixture("senses_cube.step"), "senses_cube.step");
    const Xform moved = Xform::translation(10, 0, 0);
    const Xform turned = Xform::from_matrix({0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 5, 0, 1});
    const std::string path = serialization_path("assembly_written.step");
    MINI_CHECK(file_step::write_file_step_brep(cube[0], path));
    const auto alone = nurbs_breps(file_step::read_file_step_breps(path));
    MINI_CHECK(file_step::write_file_step_assembly(cube, {{0, moved}, {0, turned}}, "assembly", path));
    MINI_CHECK(!file_step::write_file_step_assembly(cube, {{1, moved}}, "assembly", path));
    std::vector<BRep> parts;
    std::vector<std::pair<int, Xform>> instances;
    const auto shapes = nurbs_breps(file_step::read_file_step_breps(path, 1e-7, nullptr, &parts, &instances));
    std::filesystem::remove(path);
    MINI_CHECK(shapes.size() == 2 && parts.size() == 1 && instances.size() == 2);
    MINI_CHECK(instances[0].first == 0 && instances[1].first == 0 && instances[0].second.m == moved.m && instances[1].second.m == turned.m);
    for (size_t i = 0; i < shapes.size(); i++) {
        const BRep expected = alone[0].transformed(i == 0 ? moved : turned);
        for (size_t k = 0; k < expected.m_vertices.size(); k++)
            MINI_CHECK(shapes[i].m_vertices[k].point.distance(expected.m_vertices[k].point) == 0.0);
        MINI_CHECK(shapes[i].is_solid() && shapes[i].volume_exact() == expected.volume_exact());
    }
}
MINI_TEST("FileStep", "BRep Read Geometric Curve Set As Vertices And Edges") {
    std::string text = step_fixture("senses_cube.step");
    text.insert(text.rfind("ENDSEC;"), "#930=CARTESIAN_POINT('',(1.,2.,3.));#931=GEOMETRIC_CURVE_SET('',(#930));\n");
    const auto shapes = step_read_text(text, "curve_set_cube.step");
    MINI_CHECK(shapes.size() == 2 && shapes[1].face_count() == 0 && shapes[1].m_vertices.size() == 1);
    MINI_CHECK(shapes[1].m_vertices[0].point.distance(Point(1, 2, 3)) == 0.0);
}
MINI_TEST("FileStep", "BRep Read Face Of Two Outer Loops As Two Faces") {
    // two disjoint counterclockwise loops bound a face each, as ShapeFix_Face::FixSplitFace splits them
    const auto shapes = step_read_text(step_fixture("two_patches.step"), "two_patches.step");
    MINI_CHECK(shapes.size() == 1 && shapes[0].face_count() == 2);
    for (const auto& face : shapes[0].m_faces)
        MINI_CHECK(face.wires.size() == 1);
}
MINI_TEST("FileStep", "BRep Read Loop Back Through Its Vertex As Two Wires") {
    // one loop around a square and on from the same vertex around a clockwise triangle: the boundary and a hole touching it, as
    // ShapeFix_Face::FixLoopWire splits them
    const auto shapes = step_read_text(step_fixture("keyhole_loop.step"), "keyhole_loop.step");
    MINI_CHECK(shapes.size() == 1 && shapes[0].face_count() == 1 && shapes[0].m_faces[0].wires.size() == 2);
    MINI_CHECK(shapes[0].wire_edges(shapes[0].m_faces[0].wires[0]).size() == 5 && shapes[0].wire_edges(shapes[0].m_faces[0].wires[1]).size() == 3);
}
MINI_TEST("FileStep", "BRep Read Degenerated Edge Only Where A Gap Spans No Length") {
    // a projected gap at a collapsed row takes a degenerated edge; one along the surface between edge ends around their vertex does not,
    // as ShapeFix_Wire::FixLacking asks ShapeAnalysis_Surface::IsDegenerated at ratio 10
    const auto shapes = step_read_text(step_fixture("lacking_gaps.step"), "lacking_gaps.step");
    MINI_CHECK(shapes.size() == 1 && shapes[0].face_count() == 2);
    std::array<int, 2> degenerated = {0, 0}, edges = {0, 0};
    for (int f = 0; f < 2; f++)
        for (const auto& wire : shapes[0].m_faces[f].wires)
            for (const auto& edge : shapes[0].wire_edges(wire)) {
                edges[f]++;
                degenerated[f] += shapes[0].m_edges[edge.index].degenerated;
            }
    MINI_CHECK(edges[0] == 4 && degenerated[0] == 1 && edges[1] == 3 && degenerated[1] == 0);
}

MINI_TEST("FileStep", "BRep Read Seam Given One Line Twice And Hole At The Turn") {
    // two faces of nist_edm/chamber.stp: a tube closed in v whose seam edge has both its pcurves on the line v = 0 in the file, and a full-turn
    // cylinder with a hole next to its seam: the seam's second pcurve lies a period across, the hole in the turn of the outer loop
    const auto shapes = step_read_text(step_fixture("seam_on_one_line.step"), "seam_on_one_line.step");
    MINI_CHECK(shapes.size() == 1 && shapes[0].face_count() == 2);
    const BRep& b = shapes[0];
    const auto [v0, v1] = b.m_surfaces[b.m_faces[0].surface_index].domain(1);
    int seams = 0;
    for (const BRepEdge& e : b.m_edges)
        for (const BRepCurveOnSurface& pc : e.pcurves)
            if (pc.surface_index == b.m_faces[0].surface_index && pc.curve_2d_index_2 >= 0) {
                const double a = b.m_curves_2d[pc.curve_2d_index].point_at(b.pcurve_range(pc, false).first)[1];
                const double z = b.m_curves_2d[pc.curve_2d_index_2].point_at(b.pcurve_range(pc, true).first)[1];
                seams += std::abs(std::abs(a - z) - (v1 - v0)) < 1e-9;
            }
    MINI_CHECK(seams == 1);
    std::array<double, 2> lo = {1e300, 1e300}, hi = {-1e300, -1e300};
    for (size_t w = 0; w < b.m_faces[1].wires.size() && w < 2; w++)
        for (const BRepRef& use : b.wire_edges(b.m_faces[1].wires[w]))
            for (const BRepCurveOnSurface& pc : b.m_edges[use.index].pcurves)
                if (pc.surface_index == b.m_faces[1].surface_index)
                    for (int k = 0; k <= 8; k++) {
                        const auto [t0, t1] = b.pcurve_range(pc, false);
                        const double u = b.m_curves_2d[pc.curve_2d_index].point_at(t0 + (t1 - t0) * k / 8.0)[0];
                        lo[w] = std::min(lo[w], u);
                        hi[w] = std::max(hi[w], u);
                    }
    MINI_CHECK(b.m_faces[1].wires.size() == 2 && lo[0] < lo[1] && hi[1] < hi[0]);
    const std::vector<Mesh> meshes = b.face_meshes();
    MINI_CHECK(meshes.size() == 2 && meshes[0].number_of_faces() > 0 && meshes[1].number_of_faces() > 0);
}

MINI_TEST("FileStep", "Analytic Entities Kept And Written") {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "session_step_analytic";
    std::filesystem::create_directories(dir);
    const double pi = 3.14159265358979323846;
    const std::vector<std::tuple<BRep, double, std::string, Surface::Kind>> solids = {
        {BRep::create_cylinder(1.5, 4), pi * 9.0, "CYLINDRICAL_SURFACE", Surface::Kind::Cylinder}, {BRep::create_cone(1.5, 4), pi * 3.0, "CONICAL_SURFACE", Surface::Kind::Cone},
        {BRep::create_frustum(2, 1, 3), pi * 7.0, "CONICAL_SURFACE", Surface::Kind::Cone}, {BRep::create_sphere(1.5), pi * 4.5, "SPHERICAL_SURFACE", Surface::Kind::Sphere},
        {BRep::create_torus(3, 1), 6.0 * pi * pi, "TOROIDAL_SURFACE", Surface::Kind::Torus}, {BRep::create_box(2, 3, 4), 24.0, "PLANE", Surface::Kind::Plane},
        {BRep::create_block_with_hole(4, 4, 2, 1), 32.0 - 2.0 * pi, "CYLINDRICAL_SURFACE", Surface::Kind::Cylinder}};
    const Xform move = Xform::translation(3, -2, 5) * Xform::rotation_z(0.7) * Xform::rotation_x(0.4);
    for (const auto& [solid, volume, entity, kind] : solids)
        for (const Xform& placed : {Xform::identity(), move, Xform::scale_xyz(1, -1, 1)}) {
            const BRep source = solid.transformed(placed);
            const std::string path = (dir / (solid.name + ".step")).string();
            MINI_CHECK(file_step::write_file_step_brep(source, path));
            std::stringstream text;
            text << std::ifstream(path).rdbuf();
            MINI_CHECK(text.str().find(entity + "(") != std::string::npos && text.str().find("B_SPLINE_SURFACE") == std::string::npos);
            MINI_CHECK(text.str().find("B_SPLINE_CURVE") == std::string::npos && text.str().find("PCURVE") == std::string::npos);
            MINI_CHECK(kind == Surface::Kind::Plane || kind == Surface::Kind::Sphere || kind == Surface::Kind::Torus || text.str().find("CIRCLE(") != std::string::npos);
            const std::vector<BRep> read = file_step::read_file_step_breps(path);
            MINI_CHECK(read.size() == 1);
            const BRep& b = read[0];
            MINI_CHECK(std::any_of(b.m_surfaces.begin(), b.m_surfaces.end(), [&](const Surface& s) { return s.kind() == kind; }));
            for (const BRepFace& f : b.m_faces)
                MINI_CHECK(b.m_surfaces[f.surface_index].kind() != Surface::Kind::Nurbs);
            for (int e = 0; e < b.edge_count(); ++e)
                MINI_CHECK(b.m_edges[e].degenerated || b.m_curves_3d[b.m_edges[e].curve_3d_index].kind() == Curve::Kind::Line || b.m_curves_3d[b.m_edges[e].curve_3d_index].kind() == Curve::Kind::Circle);
            // The file's placement is kept: a face's surface is one of the source's, whatever window it was read in.
            for (const BRepFace& f : b.m_faces)
                MINI_CHECK(std::any_of(source.m_surfaces.begin(), source.m_surfaces.end(), [&](const Surface& s) { return s.coincident(b.m_surfaces[f.surface_index], 1e-9) != 0; }));
            // A mirrored solid keeps its faces' senses: its volume is the source's, negative.
            MINI_CHECK(b.is_valid() && b.is_solid() && std::abs(b.volume_exact() - source.volume_exact()) < 1e-7 && std::abs(std::abs(b.volume_exact()) - volume) < 1e-7);
            // Along every edge its pcurve on the surface, in true parameters, is the 3D curve.
            double gap = 0.0;
            for (int e = 0; e < b.edge_count(); ++e)
                for (const BRepCurveOnSurface& use : b.m_edges[e].pcurves)
                    for (int k = 0; k < (use.curve_2d_index_2 >= 0 ? 2 : 1) && !b.m_edges[e].degenerated; ++k)
                        for (double f : {0.0, 0.3, 0.5, 0.8, 1.0}) {
                            const Curve world = b.edge_curve(e);
                            const Curve2d pc = b.pcurve(use, k == 1);
                            const Point uv = pc.point_at(pc.domain().first + f * (pc.domain().second - pc.domain().first));
                            gap = std::max(gap, b.m_surfaces[use.surface_index].point_at(uv[0], uv[1]).distance(world.point_at(world.domain().first + f * (world.domain().second - world.domain().first))));
                        }
            MINI_CHECK(gap < 1e-9);
            const BRep plain = b.as_nurbs();
            MINI_CHECK(plain.is_nurbs() && plain.is_valid() && plain.is_solid() && std::abs(plain.volume_exact() - source.volume_exact()) < 1e-7);
        }
    const std::string bellow = "/home/user/session_data/brep_boolean_bench/nist_edm/bellow.stp";
    if (std::filesystem::exists(bellow)) {
        int cones = 0;
        for (const BRep& b : file_step::read_file_step_breps(bellow))
            for (const Surface& s : b.m_surfaces)
                cones += s.kind() == Surface::Kind::Cone;
        MINI_CHECK(cones == 64);
    }
}

MINI_TEST("FileStep", "Sphere Cap With Its Seam To The Pole Round Trips") {
    // A cap of a sphere above a latitude: its loop runs the circle, up the seam to the pole and back down it. Read back, the seam is the loop's own: no second one is made to the pole.
    const double pi = 3.14159265358979323846, r = 2.0, v0 = 0.4, z0 = r * std::sin(v0), rho = r * std::cos(v0);
    const BRepOrientation F = BRepOrientation::Forward, R = BRepOrientation::Reversed;
    BRep b;
    const Point o(0, 0, 0);
    const Vector x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    const int a = b.add_vertex(Point(rho, 0, z0)), n = b.add_vertex(Point(0, 0, r));
    const int e_circle = b.add_edge(b.add_curve_3d(Curve::circle(Point(0, 0, z0), x, y, rho)), a, a);
    const int e_seam = b.add_edge(b.add_curve_3d(Curve::circle(o, x, z, r, v0, 0.5 * pi)), a, n);
    const int e_pole = b.add_edge(-1, n, n);
    const int sphere = b.add_surface(Surface::sphere(o, z, x, r, {0.0, 2.0 * pi, v0, 0.5 * pi}));
    b.add_pcurve(e_circle, sphere, b.add_curve_2d(Curve2d::line(0, v0, 2.0 * pi, v0)));
    b.add_pcurve(e_seam, sphere, b.add_curve_2d(Curve::line(Point(2.0 * pi, 0, 0), Vector(0, 1, 0), v0, 0.5 * pi)), b.add_curve_2d(Curve::line(Point(0, 0, 0), Vector(0, 1, 0), v0, 0.5 * pi)));
    b.add_pcurve(e_pole, sphere, b.add_curve_2d(Curve2d::line(0, 0.5 * pi, 2.0 * pi, 0.5 * pi)));
    const int f_cap = b.add_face(sphere, {{b.add_wire({{e_circle, F}, {e_seam, F}, {e_pole, R}, {e_seam, R}}), F}});
    const int plane = b.add_surface(Surface::plane(Point(0, 0, z0), z * -1.0, x, {-r, r, -r, r}));
    b.add_pcurve(e_circle, plane, b.add_curve_2d(plane_pcurve(b.m_surfaces[plane], b.m_curves_3d[b.m_edges[e_circle].curve_3d_index])));
    const int f_disc = b.add_face(plane, {{b.add_wire({{e_circle, R}}), F}});
    b.add_solid({{b.add_shell({{f_cap, F}, {f_disc, F}}), F}});
    const double h = r - z0, volume = pi * h * h * (3.0 * r - h) / 3.0;
    MINI_CHECK(b.is_valid() && b.is_solid() && std::abs(b.volume_exact() - volume) < 1e-8);
    const std::string path = (std::filesystem::temp_directory_path() / "sphere_cap_seam.step").string();
    MINI_CHECK(file_step::write_file_step_brep(b, path));
    const std::vector<BRep> read = file_step::read_file_step_breps(path);
    MINI_CHECK(read.size() == 1 && read[0].face_count() == 2 && read[0].is_valid() && read[0].is_solid());
    MINI_CHECK(read.size() == 1 && std::abs(read[0].volume_exact() - volume) < 1e-8);
    MINI_CHECK(read.size() == 1 && std::count_if(read[0].m_surfaces.begin(), read[0].m_surfaces.end(), [](const Surface& s) { return s.kind() == Surface::Kind::Sphere; }) == 1);
}
}
