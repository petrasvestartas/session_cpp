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
            world = shape.m_curves_3d.at(edge.curve_3d_index);
        for (const auto& image : edge.pcurves) {
            MINI_CHECK(image.curve_2d_index>=0);
            const auto& surface = shape.m_surfaces.at(image.surface_index);
            for (int pc_index : {image.curve_2d_index, image.curve_2d_index_2}) {
                if (pc_index < 0)
                    continue;
                const auto& pc = shape.m_curves_2d.at(pc_index);
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
        const auto imported = file_step::read_file_step_breps((root/relative).string(),0.001);
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
        BRep source = BRep::create_box(2.0, 2.0, 2.0);
        for (auto& vertex : source.m_vertices)
            vertex.point = shear(vertex.point);
        for (auto& curve : source.m_curves_3d)
            for (int i = 0; i < curve.cv_count(); ++i)
                MINI_CHECK(curve.set_cv(i, shear(curve.get_cv(i))));
        for (auto& surface : source.m_surfaces) {
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
        }
        for (auto& curve : source.m_curves_2d)
            for (int i = 0; i < curve.cv_count(); ++i) {
                const Point uv = curve.get_cv(i);
                MINI_CHECK(curve.set_cv(i, Point(-7.0 + 20.0 * uv[0], 32.0 + 3.0 * uv[1], 0.0)));
            }
        const auto& support = source.m_surfaces.front();
        const Vector eu = support.get_cv(1, 0) - support.get_cv(0, 0);
        const Vector ev = support.get_cv(0, 1) - support.get_cv(0, 0);
        MINI_CHECK(std::abs(eu.dot(ev)) > 0.0); // The Gram cross term is required.
        const auto frozen = source.jsondump();
        const std::string path = serialization_path(rational ? "oblique_raw_rational.step" : "oblique_raw.step");
        MINI_CHECK(file_step::write_file_step_brep(source, path));
        const auto imported = file_step::read_file_step_breps(path, 1e-7);
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
            const auto& world = result.m_curves_3d.at(edge.curve_3d_index);
            for (const auto& image : edge.pcurves)
                for (int index : {image.curve_2d_index, image.curve_2d_index_2}) {
                    if (index < 0)
                        continue;
                    MINI_CHECK(curve_surface_intersection::same_parameter(
                        world, result.m_curves_2d.at(index), result.m_surfaces.at(image.surface_index), 1e-7) ==
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
    const auto shapes = file_step::read_file_step_breps(path, .001);
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
            const auto& surface = b.m_surfaces.at(image.surface_index);
            if (surface.degree(0) != 2) continue;
            for (int index : {image.curve_2d_index, image.curve_2d_index_2}) if (index >= 0) {
                const auto& pc = b.m_curves_2d.at(index);
                MINI_CHECK(pc.degree() == 1 && pc.cv_count() == 2);
            }
            if (image.curve_2d_index_2 < 0) continue;
            const auto& pc = b.m_curves_2d.at(image.curve_2d_index);
            const auto& other = b.m_curves_2d.at(image.curve_2d_index_2);
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
            MINI_CHECK(a.m_curves_3d[i].m_cv == b.m_curves_3d[i].m_cv);
            MINI_CHECK(a.m_curves_3d[i].m_nurbsknot == b.m_curves_3d[i].m_nurbsknot);
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
        const auto shapes = file_step::read_file_step_breps(temporary, .001);
        std::filesystem::remove(temporary);
        MINI_CHECK(shapes.size() == 1); if (shapes.size() != 1) continue;
        const auto& shape = shapes.front();
        MINI_CHECK(std::count_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; }) == 1);
        const auto& curve = shape.m_curves_3d.at(std::find_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; })->curve_3d_index);
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
        MINI_CHECK(shape.is_valid());
        const auto& edge = shape.m_edges.at(0);
        const auto& world = shape.m_curves_3d.at(edge.curve_3d_index);
        MINI_CHECK(world.is_rational());
        MINI_CHECK(world.cv_count() == 9);
        size_t cap_images = 0;
        for (const auto& image : edge.pcurves) {
            if (image.surface_index != 1) continue;
            const auto match = curve_surface_intersection::same_parameter(world,
                shape.m_curves_2d.at(image.curve_2d_index), shape.m_surfaces.at(image.surface_index), .001);
            if (variant == 0) {
                original_world = world;
                original_cap = shape.m_curves_2d.at(image.curve_2d_index);
                original_support = shape.m_surfaces.at(image.surface_index);
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
    const auto shapes = file_step::read_file_step_breps(path.string(), .001);
    MINI_CHECK(shapes.size() == 1);
    if (shapes.size() != 1) return;
    const auto& shape = shapes.front();
    MINI_CHECK(shape.is_valid() && shape.is_solid() &&
        std::count_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; }) == 1);
    rhino_step_whole_pcurves(shape);
    const auto seam = std::find_if(shape.m_edges.begin(), shape.m_edges.end(), [](const auto& edge) { return !edge.degenerated; });
    const auto& world = shape.m_curves_3d.at(seam->curve_3d_index);
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
            const auto& pc = shape.m_curves_2d.at(index);
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
        const auto& curve = shape.m_curves_3d.at(shape.m_edges.front().curve_3d_index);
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
                            static_cast<int>(&face - shape.m_faces.data()), first.orientation));
                        const auto& b = shape.m_curves_2d.at(shape.pcurve_index(next.index,
                            static_cast<int>(&face - shape.m_faces.data()), next.orientation));
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
    const auto& support = source.m_surfaces.front();
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
    const auto path = serialization_path("noncollapsed_cylinder_gap.step");
    MINI_CHECK(file_step::write_file_step_brep(source, path));
    const auto imported = file_step::read_file_step_breps(path, .001);
    std::filesystem::remove(path);
    MINI_CHECK(imported.size() == 1);
    MINI_CHECK(std::none_of(imported.front().m_edges.begin(), imported.front().m_edges.end(),
        [](const auto& edge) { return edge.degenerated; }));
}
}
