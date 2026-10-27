#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "intersection.h"
#include "nurbs_offset.h"
#include "primitives.h"
#include "session.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#ifdef SESSION_WITH_OCCT
#error Native viewer evidence must be built without OCCT
#endif

using namespace session_cpp;

namespace {

const double EvidenceTolerance = 1e-6;
const Color SourceColor(75 / 255.f, 125 / 255.f, 170 / 255.f);
const Color ToolColor(200 / 255.f, 145 / 255.f, 85 / 255.f);
const Color ResultColor(180 / 255.f, 110 / 255.f, 155 / 255.f);
const Color IsoColor(55 / 255.f, 45 / 255.f, 65 / 255.f);

struct Evidence {
    std::filesystem::path folder;
    nlohmann::ordered_json cases = nlohmann::ordered_json::array();
    int failures = 0;

    void checkpoint() const {
        const nlohmann::ordered_json report = {
            {"backend", "session_cpp_native"}, {"boolean_acceptance", "not covered by this exporter"},
            {"cases", cases}, {"failures", failures},
            {"geometry_placement", "Original parameterized world coordinates, without display transforms"},
            {"legend", {{"source_a", "muted blue (75,125,170)"}, {"source_b", "muted orange (200,145,85)"},
                {"result", "muted mauve (180,110,155)"}, {"surface_iso_curves", "dark plum (55,45,65); exact native iso-curves"}}},
            {"renderer", "session_viewer native selftest; this program exports scenes only"}
        };
        std::ofstream(folder / "native_geometry_cases.json") << report.dump(2);
    }

    void save(const std::string& name, Session& scene, nlohmann::ordered_json report) {
        const std::string file = name + ".pb";
        scene.pb_dump((folder / file).string());
        report["name"] = name;
        report["scene"] = file;
        if (!report.value("passed", false))
            ++failures;
        cases.push_back(report);
        checkpoint();
        std::cout << name << ": " << (report.value("passed", false) ? "passed" : "failed") << '\n';
    }

    void fail(const std::string& name, const std::exception& error) {
        ++failures;
        cases.push_back({{"error", error.what()}, {"name", name}, {"passed", false}});
        checkpoint();
        std::cout << name << ": " << error.what() << '\n';
    }
};

void add_curve(Session& scene, NurbsCurve curve, const Color& color) {
    curve.linecolors = {color};
    curve.width = 3.0;
    scene.add_nurbscurve(curve);
}

void add_surface(Session& scene, NurbsSurface surface, const Color& color) {
    surface.facecolors = {color};
    surface.linecolors = {Color::black()};
    scene.add_nurbssurface(surface);
}

void add_iso_curves(Session& scene, const NurbsSurface& surface) {
    for (int direction = 0; direction < 2; ++direction) {
        const std::pair<double, double> fixed = surface.domain(1 - direction);
        for (int i = 1; i < 8; ++i) {
            const double parameter = fixed.first + (fixed.second - fixed.first) * i / 8.0;
            NurbsCurve curve = surface.iso_curve(direction, parameter);
            if (!curve.is_valid())
                throw std::runtime_error("Native surface iso-curve is invalid");
            curve.name = "native offset surface iso-curve";
            curve.linecolors = {IsoColor};
            curve.width = 1.3;
            scene.add_nurbscurve(curve);
        }
    }
}

void add_hit(Session& scene, Point point) {
    point.name = "native intersection";
    point.pointcolor = ResultColor;
    point.width = 10.0;
    scene.add_point(point);
}

NurbsSurface plane(double height) {
    return NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {{-1, -1, height}, {-1, 1, height}, {1, -1, height}, {1, 1, height}}
    );
}

NurbsSurface saddle(double sign) {
    std::vector<Point> points;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            points.push_back({double(i), double(j), 0.15 * sign * (i - 1.5) * (j - 1.5) + (sign < 0.0 ? 0.1 : 0.0)});
    return NurbsSurface::create(
        false,
        false,
        3,
        3,
        4,
        4,
        points
    );
}

NurbsSurface bowl() {
    const std::array<double, 3> heights = {1.0, -1.0, 1.0};
    std::vector<Point> points;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({i - 1.0, j - 1.0, heights[i] + heights[j]});
    return NurbsSurface::create(
        false,
        false,
        2,
        2,
        3,
        3,
        points
    );
}

void curve_offset_case(
    Evidence& evidence,
    const std::string& name,
    NurbsCurve source,
    double distance
) {
    source.set_domain(-7.0, 3.0);
    source.name = "source";
    NurbsCurve result = nurbs_offset::curve(source, distance, {0, 0, 1}, EvidenceTolerance);
    result.name = "native offset";
    double error = 0.0;
    for (int i = 0; i <= 997; ++i) {
        const double t = -7.0 + 10.0 * i / 997.0;
        const std::vector<Vector> derivatives = source.evaluate(t, 1);
        const Vector side = derivatives[1].cross({0, 0, 1}).normalized();
        const Point expected = source.point_at(t) + side * distance;
        error = std::max(error, expected.distance(result.point_at(t)));
    }
    Session scene(name);
    add_curve(scene, source, SourceColor);
    add_curve(scene, result, ResultColor);
    evidence.save(name, scene, {
        {"distance", distance}, {"kind", "curve_offset"}, {"max_sampled_residual", error},
        {"parameter_domain", {source.domain_start(), source.domain_end()}},
        {"passed", result.is_valid() && result.domain() == source.domain() && error <= EvidenceTolerance},
        {"result_control_points", result.cv_count()}, {"samples", 998}, {"tolerance", EvidenceTolerance}
    });
}

void surface_offset_case(
    Evidence& evidence,
    const std::string& name,
    NurbsSurface source,
    double distance
) {
    source.set_domain(0, 13.0, 17.0);
    source.set_domain(1, -9.0, -2.0);
    source.name = "source";
    NurbsSurface result = nurbs_offset::surface(source, distance, EvidenceTolerance);
    result.name = "native offset";
    double error = 0.0;
    for (int i = 0; i < 47; ++i)
        for (int j = 0; j < 43; ++j) {
            const double u = 13.0 + 4.0 * (i + 0.37) / 47.0;
            const double v = -9.0 + 7.0 * (j + 0.61) / 43.0;
            const Point expected = source.point_at(u, v) + source.normal_at(u, v) * distance;
            error = std::max(error, expected.distance(result.point_at(u, v)));
        }
    Session scene(name);
    add_surface(scene, source, SourceColor);
    add_surface(scene, result, ResultColor);
    Session result_scene(name + " result only");
    add_surface(result_scene, result, ResultColor);
    add_iso_curves(result_scene, result);
    const std::string result_file = name + "_result.pb";
    result_scene.pb_dump((evidence.folder / result_file).string());
    evidence.save(name, scene, {
        {"distance", distance}, {"kind", "surface_offset"}, {"max_sampled_residual", error},
        {"parameter_domain_u", {13.0, 17.0}}, {"parameter_domain_v", {-9.0, -2.0}},
        {"passed", result.is_valid() && result.domain(0) == source.domain(0)
            && result.domain(1) == source.domain(1) && error <= EvidenceTolerance},
        {"result_control_points", {result.cv_count(0), result.cv_count(1)}},
        {"result_scene", result_file}, {"result_iso_curves", 14},
        {"samples", 47 * 43}, {"tolerance", EvidenceTolerance}
    });
}

void curve_curve_case(Evidence& evidence, int kind) {
    const std::string names[] = {"curve_crossing", "curve_tangent", "curve_overlap"};
    const NurbsCurve a = NurbsCurve::create(false, 1, {{-3, 0, 0}, {3, 0, 0}});
    const std::array<NurbsCurve, 3> others = {
        NurbsCurve::create(false, 1, {{0, -2, 0}, {0, 2, 0}}),
        Primitives::circle(0, 1, 0, 1),
        NurbsCurve::create(false, 1, {{-1, 0, 0}, {4, 0, 0}})
    };
    const NurbsCurve& b = others[kind];
    const curve_intersection::Result result = curve_intersection::intersect(a, b, EvidenceTolerance);
    Session scene(names[kind]);
    add_curve(scene, a, SourceColor);
    add_curve(scene, b, ToolColor);
    double error = 0.0;
    for (const curve_intersection::Hit& hit : result.points) {
        error = std::max(error, a.point_at(hit.a).distance(b.point_at(hit.b)));
        add_hit(scene, hit.point);
    }
    for (const curve_intersection::Overlap& overlap : result.overlaps) {
        NurbsCurve shared = a;
        if (!shared.trim(overlap.a0, overlap.a1))
            throw std::runtime_error("Failed to trim native overlap for viewer scene");
        add_curve(scene, shared, ResultColor);
    }
    const bool counts = kind == 2 ? result.overlaps.size() == 1 && result.points.empty()
        : result.points.size() == 1 && result.overlaps.empty();
    evidence.save(names[kind], scene, {
        {"complete", result.complete}, {"failure", result.failure}, {"kind", "curve_curve"},
        {"max_sampled_residual", error}, {"overlaps", result.overlaps.size()},
        {"passed", result.complete && counts && error <= EvidenceTolerance},
        {"points", result.points.size()}, {"tolerance", EvidenceTolerance}
    });
}

void curve_surface_case(Evidence& evidence, bool tangent) {
    const std::string name = tangent ? "freeform_curve_surface_tangent" : "freeform_curve_surface_two_roots";
    const NurbsSurface source = bowl();
    const double z = tangent ? 0.04 : 0.29;
    const NurbsCurve curve = NurbsCurve::create(false, 1, {{-1, 0.2, z}, {1, 0.2, z}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, source, EvidenceTolerance);
    Session scene(name);
    add_surface(scene, source, SourceColor);
    add_curve(scene, curve, ToolColor);
    double error = 0.0;
    for (const curve_surface_intersection::Hit& hit : result.points) {
        error = std::max(error, curve.point_at(hit.t).distance(source.point_at(hit.u, hit.v)));
        add_hit(scene, hit.point);
    }
    evidence.save(name, scene, {
        {"complete", result.complete}, {"failure", result.failure}, {"kind", "curve_surface"},
        {"max_sampled_residual", error}, {"overlaps", result.overlaps.size()},
        {"passed", result.complete && result.points.size() == (tangent ? 1 : 2) && error <= EvidenceTolerance},
        {"points", result.points.size()}, {"tolerance", EvidenceTolerance}
    });
}

void surface_surface_case(Evidence& evidence, bool interior_loop) {
    const std::string name = interior_loop ? "freeform_small_interior_loop" : "freeform_two_surface_branches";
    const NurbsSurface a = interior_loop ? bowl() : saddle(1.0);
    const NurbsSurface b = interior_loop ? plane(0.01) : saddle(-1.0);
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections = Intersection::surface_surface(a, b, 1e-7);
    Session scene(name);
    add_surface(scene, a, SourceColor);
    add_surface(scene, b, ToolColor);
    double error = 0.0;
    double length = 0.0;
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections) {
        const NurbsCurve& curve = std::get<0>(section);
        const NurbsCurve& pa = std::get<1>(section);
        const NurbsCurve& pb = std::get<2>(section);
        add_curve(scene, curve, ResultColor);
        length += curve.length();
        for (int i = 0; i <= 509; ++i) {
            const double t = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 509.0;
            const Point world = curve.point_at(t);
            const Point ua = pa.point_at(t);
            const Point ub = pb.point_at(t);
            error = std::max({error, world.distance(a.point_at(ua[0], ua[1])), world.distance(b.point_at(ub[0], ub[1]))});
        }
    }
    const bool geometry = interior_loop ? std::abs(length - 0.2 * 3.141592653589793) < 1e-5 : sections.size() == 2;
    evidence.save(name, scene, {
        {"acceptance_tolerance", 2e-6}, {"kind", "surface_surface"}, {"length", length},
        {"max_sampled_residual", error}, {"passed", geometry && !sections.empty() && error <= 2e-6},
        {"samples_per_section", 510}, {"sections", sections.size()}, {"tolerance", 1e-7}
    });
}

}

int main(int argc, char** argv) {
    if (argc > 1) {
        try {
            if (argc != 5 || std::string(argv[1]) != "--reexport-brep")
                throw std::invalid_argument("Usage: native_viewer_evidence --reexport-brep input.brep.json output.pb source|result");
            const std::filesystem::path input = argv[2];
            const std::filesystem::path output = argv[3];
            const std::string role = argv[4];
            if (role != "source" && role != "result")
                throw std::invalid_argument("Re-export role must be source or result");
            if (output.extension() != ".pb" || std::filesystem::exists(output))
                throw std::invalid_argument("Re-export requires a new .pb output path");
            BRep geometry = BRep::file_json_load(input.string());
            if (!geometry.is_valid())
                throw std::invalid_argument("Re-export input has invalid BRep references");
            auto original = geometry.jsondump();
            const auto old_color = original.at("surfacecolor");
            geometry.surfacecolor = role == "source" ? SourceColor : ResultColor;
            auto recolored = geometry.jsondump();
            original.erase("surfacecolor");
            recolored.erase("surfacecolor");
            if (original != recolored)
                throw std::runtime_error("Display re-export changed fields other than surfacecolor");
            if (!output.parent_path().empty())
                std::filesystem::create_directories(output.parent_path());
            Session scene(geometry.name);
            scene.add_brep(geometry);
            scene.pb_dump(output.string());
            const nlohmann::ordered_json report = {{"source", std::filesystem::absolute(input).string()},
                {"scene", output.filename().string()}, {"role", role}, {"original_color", old_color},
                {"display_color", geometry.surfacecolor.jsondump()}, {"geometry_unchanged", true},
                {"offset_recomputed", false}, {"display_transforms", false}};
            std::ofstream(output.string() + ".reexport.json") << report.dump(2);
            std::cout << "Re-exported unchanged BRep geometry to " << output.string() << '\n';
            return 0;
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            return 1;
        }
    }
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    Evidence evidence{root / "build/native-geometry-tests/viewer"};
    std::filesystem::create_directories(evidence.folder);
    const std::array<NurbsCurve, 3> curves = {
        NurbsCurve::create(false, 1, {{0, 0, 0}, {3, 0, 0}}),
        Primitives::circle(0, 0, 0, 2),
        NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0.4}, {2, -1, -0.2}, {3, 1, 1}})
    };
    const std::string curve_names[] = {"line", "circle", "freeform_curve"};
    for (int i = 0; i < 3; ++i)
        for (double distance : {-0.2, 0.2}) {
            const std::string name = curve_names[i] + (distance < 0 ? "_offset_negative" : "_offset_positive");
            try {
                curve_offset_case(evidence, name, curves[i], distance);
            } catch (const std::exception& error) {
                evidence.fail(name, error);
            }
        }
    const std::array<NurbsSurface, 4> surfaces = {plane(0.0), Primitives::sphere_surface(0, 0, 0, 2),
        Primitives::cylinder_surface(0, 0, 0, 2, 3), saddle(1.0)};
    const std::string surface_names[] = {"plane", "sphere", "cylinder", "freeform_surface"};
    for (int i = 0; i < 4; ++i)
        for (double distance : {-0.2, 0.2}) {
            const std::string name = surface_names[i] + (distance < 0 ? "_offset_negative" : "_offset_positive");
            try {
                surface_offset_case(evidence, name, surfaces[i], distance);
            } catch (const std::exception& error) {
                evidence.fail(name, error);
            }
        }
    for (int i = 0; i < 7; ++i)
        try {
            if (i < 3)
                curve_curve_case(evidence, i);
            else if (i < 5)
                curve_surface_case(evidence, i == 4);
            else
                surface_surface_case(evidence, i == 6);
        } catch (const std::exception& error) {
            evidence.fail("intersection_case_" + std::to_string(i), error);
        }
    evidence.checkpoint();
    return evidence.failures ? 1 : 0;
}

