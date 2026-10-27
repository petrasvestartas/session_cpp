#include "brep_classify.h"
#include "curve_surface_intersection.h"
#include "mini_test.h"
#include "split.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>

namespace session_cpp {
namespace split_parameter_tests {

constexpr double Tolerance = 1e-7;

BRep plane() {
    NurbsSurface support(3, false, 2, 2, 2, 2);
    support.m_nurbsknot[0] = {0, 1};
    support.m_nurbsknot[1] = {0, 1};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(support.set_cv(i, j, {double(i), double(j), 0}));
    BRep result;
    result.add_surface(support);
    const std::array<Point, 4> corners = {Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)};
    for (const Point& corner : corners)
        result.add_vertex(corner, Tolerance);
    std::vector<BRepRef> boundary;
    for (int i = 0; i < 4; ++i) {
        const NurbsCurve curve = NurbsCurve::create(false, 1, {corners[i], corners[(i + 1) % 4]});
        const int edge = result.add_edge(result.add_curve_3d(curve), i, (i + 1) % 4, Tolerance);
        result.add_pcurve(edge, 0, result.add_curve_2d(curve));
        boundary.push_back({edge, BRepOrientation::Forward});
    }
    result.add_face(0, {{result.add_wire(boundary), BRepOrientation::Forward}}, Tolerance);
    result.add_shell({{0, BRepOrientation::Forward}});
    MINI_CHECK(result.is_valid());
    return result;
}

split::FaceCut cut(bool nonlinear, bool affine) {
    split::FaceCut result;
    result.world = NurbsCurve::create(false, 1, {{0, 0.5, 0}, {1, 0.5, 0}});
    result.pcurve = NurbsCurve::create(false, 2, {{0, 0.5, 0}, {nonlinear ? 0.25 : 0.5, 0.5, 0}, {1, 0.5, 0}});
    result.section = 7;
    if (affine) {
        MINI_CHECK(result.world.set_domain(3, 11));
        MINI_CHECK(result.pcurve.set_domain(-2, 6));
    }
    return result;
}

void check_regions(const split::FaceSplit& result) {
    MINI_CHECK(result.brep.is_valid());
    MINI_CHECK(result.faces.size() == 2);
    std::set<int> regions;
    for (const Point& probe : {Point(0.5, 0.25, 0), Point(0.5, 0.75, 0)}) {
        std::vector<int> inside;
        for (int face : result.faces) {
            const auto state = brep_classify::face_uv(result.brep, face, probe, Tolerance);
            MINI_CHECK(state == brep_classify::State::Inside || state == brep_classify::State::Outside);
            if (state == brep_classify::State::Inside)
                inside.push_back(face);
        }
        MINI_CHECK(inside.size() == 1);
        MINI_CHECK(regions.insert(inside.front()).second);
    }
    std::set<int> edges;
    for (int face : result.faces)
        for (const BRepRef& wire : result.brep.m_faces[face].wires)
            for (const BRepRef& edge : result.brep.wire_edges(wire))
                edges.insert(edge.index);
    for (int index : edges) {
        const BRepEdge& edge = result.brep.m_edges[index];
        const NurbsCurve& world = result.brep.m_curves_3d[edge.curve_3d_index];
        for (const BRepCurveOnSurface& pc : edge.pcurves)
            for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
                if (ci < 0)
                    continue;
                const auto match = curve_surface_intersection::same_parameter(world,
                    result.brep.m_curves_2d[ci], result.brep.m_surfaces[pc.surface_index], Tolerance);
                MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Matched);
            }
    }
}

void nonlinear_case(bool affine) {
    const BRep source = plane();
    const split::FaceCut input = cut(true, affine);
    const std::filesystem::path folder = "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    const std::string name = std::string("native_split_nonlinear_parameter") + (affine ? "_affine" : "");
    nlohmann::ordered_json report = {{"passed", false}, {"source", source.jsondump()},
        {"world", input.world.jsondump()}, {"pcurve", input.pcurve.jsondump()}, {"tolerance", Tolerance},
        {"analytic_maximum_paired_gap", 0.125}, {"same_geometric_image", true}};
    const auto save = [&]() { std::ofstream(folder / (name + ".json")) << report.dump(2); };
    save();
    try {
        const split::FaceSplit result = split::split_brep_face_by_pcurves(source, 0, {input}, Tolerance);
        report["outcome"] = "repaired";
        report["result"] = result.brep.jsondump();
        save();
        check_regions(result);
    } catch (const split::CurveError& error) {
        report["outcome"] = "explicit unsupported correspondence";
        report["error"] = error.what();
        report["failure"] = {{"world", error.curve.jsondump()}, {"pcurve", error.pcurve.jsondump()},
            {"surface", error.surface.jsondump()}, {"distance", error.distance}, {"tolerance", error.tolerance}};
        save();
        MINI_CHECK(error.distance > error.tolerance);
        const auto match = curve_surface_intersection::same_parameter(error.curve, error.pcurve, error.surface, error.tolerance);
        MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Mismatched);
    }
    report["passed"] = true;
    save();
}

}

MINI_TEST("Split Parameter Contract", "Same Image Is Not Same Parameter") {
    const BRep source = split_parameter_tests::plane();
    for (bool affine : {false, true}) {
        const auto cut = split_parameter_tests::cut(true, affine);
        const auto match = curve_surface_intersection::same_parameter(cut.world, cut.pcurve,
            source.m_surfaces[0], split_parameter_tests::Tolerance);
        MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Mismatched);
        const Point world = cut.world.point_at((cut.world.domain_start() + cut.world.domain_end()) * 0.5);
        const Point uv = cut.pcurve.point_at((cut.pcurve.domain_start() + cut.pcurve.domain_end()) * 0.5);
        MINI_CHECK(std::abs(world.distance(source.m_surfaces[0].point_at(uv[0], uv[1])) - 0.125) <= 1e-14);
    }
}

MINI_TEST("Split Parameter Contract", "Affine Range Correspondence Is Accepted") {
    const BRep source = split_parameter_tests::plane();
    const auto cut = split_parameter_tests::cut(false, true);
    const auto match = curve_surface_intersection::same_parameter(cut.world, cut.pcurve,
        source.m_surfaces[0], split_parameter_tests::Tolerance);
    MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Matched);
    split_parameter_tests::check_regions(split::split_brep_face_by_pcurves(source, 0, {cut}, split_parameter_tests::Tolerance));
}

MINI_TEST("Split Parameter Contract", "Nonlinear Correspondence Is Repaired Or Rejected") {
    split_parameter_tests::nonlinear_case(false);
    split_parameter_tests::nonlinear_case(true);
}

}
