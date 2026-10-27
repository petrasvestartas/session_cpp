#include "brep_section.h"
#include "file_step.h"
#include "intersection.h"
#include "section_fit_error.h"
#include "session.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

#ifdef SESSION_WITH_OCCT
#error Native chair sections must be built without OCCT
#endif

using namespace session_cpp;

namespace {

BRep face_only(const BRep& source, int face) {
    BRep result = source;
    result.m_faces = {source.m_faces[face]};
    result.m_shells.clear();
    result.m_solids.clear();
    return result;
}

nlohmann::ordered_json samples(const brep_section::FaceCurve& section, const NurbsSurface& a, const NurbsSurface& b) {
    nlohmann::ordered_json points = nlohmann::ordered_json::array();
    double gap = 0.0;
    for (int i = 0; i <= 200; ++i) {
        const double t = section.world.domain_start() + (section.world.domain_end() - section.world.domain_start()) * i / 200.0;
        const Point p = section.world.point_at(t);
        const Point ua = section.first_pcurve.point_at(t), ub = section.second_pcurve.point_at(t);
        gap = std::max({gap, p.distance(a.point_at(ua[0], ua[1])), p.distance(b.point_at(ub[0], ub[1]))});
        points.push_back({p[0], p[1], p[2]});
    }
    return {{"world", section.world.jsondump()}, {"first_pcurve", section.first_pcurve.jsondump()},
        {"second_pcurve", section.second_pcurve.jsondump()}, {"max_sampled_gap", gap}, {"samples", points}};
}

nlohmann::ordered_json fit_failure(const BRep& a, const BRep& b, double tolerance) {
    const NurbsSurface& first = a.m_surfaces[a.m_faces[0].surface_index];
    const NurbsSurface& second = b.m_surfaces[b.m_faces[0].surface_index];
    try {
        const auto result = Intersection::surface_surface(first, second, tolerance);
        nlohmann::ordered_json lines = nlohmann::ordered_json::array();
        for (const auto& section : result)
            lines.push_back({{"world", std::get<0>(section).jsondump()},
                {"first_pcurve", std::get<1>(section).jsondump()},
                {"second_pcurve", std::get<2>(section).jsondump()}});
        return {{"direct_sections", result.size()}, {"lines", lines}};
    } catch (const section_fit::Error& error) {
        nlohmann::ordered_json points = nlohmann::ordered_json::array();
        for (const Point& p : error.samples)
            points.push_back({p[0], p[1], p[2]});
        nlohmann::ordered_json uv = nlohmann::ordered_json::array();
        for (const Point& p : error.pcurve_samples)
            uv.push_back({p[0], p[1], p[2]});
        nlohmann::ordered_json uv_second = nlohmann::ordered_json::array();
        for (const Point& p : error.second_pcurve_samples)
            uv_second.push_back({p[0], p[1], p[2]});
        nlohmann::ordered_json world = nlohmann::ordered_json::array();
        for (const Point& p : error.world_samples)
            world.push_back({p[0], p[1], p[2]});
        return {{"samples", points}, {"parameters", error.parameters}, {"closed", error.closed},
            {"first_surface", first.jsondump()}, {"second_surface", second.jsondump()},
            {"pcurve_samples", uv}, {"second_pcurve_samples", uv_second}, {"world_samples", world}};
    } catch (const std::runtime_error& error) {
        return {{"error", error.what()}};
    }
}

}

int main(int argc, char** argv) {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const auto folder = root / "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    const auto chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    if (chairs.size() != 2 || !chairs[0].is_solid() || !chairs[1].is_solid())
        return 2;
    const int first = argc > 1 ? std::stoi(argv[1]) : -1;
    const int second = argc > 2 ? std::stoi(argv[2]) : -1;
    const double tolerance = 1e-6;
    nlohmann::ordered_json records = nlohmann::ordered_json::array();
    int failures = 0;
    for (int a = 0; a < chairs[0].face_count(); ++a)
        for (int b = 0; b < chairs[1].face_count(); ++b) {
            if ((first >= 0 && first != a) || (second >= 0 && second != b))
                continue;
            const auto start = std::chrono::steady_clock::now();
            const BRep fa = face_only(chairs[0], a), fb = face_only(chairs[1], b);
            const auto result = brep_section::faces(fa, fb, tolerance);
            nlohmann::ordered_json data = {{"a", a}, {"b", b}, {"complete", result.complete},
                {"failure", result.failure}, {"sections", nlohmann::ordered_json::array()}};
            Session scene("native_chair_sections_" + std::to_string(a) + "_" + std::to_string(b));
            BRep view_a = fa, view_b = fb;
            view_a.surfacecolor = Color(40 / 255.f, 140 / 255.f, 225 / 255.f, 1.f);
            view_b.surfacecolor = Color(235 / 255.f, 150 / 255.f, 55 / 255.f, 1.f);
            scene.add_brep(view_a);
            scene.add_brep(view_b);
            bool passed = result.complete;
            for (const auto& section : result.curves) {
                const auto record = samples(section, fa.m_surfaces[fa.m_faces[0].surface_index], fb.m_surfaces[fb.m_faces[0].surface_index]);
                passed = passed && record["max_sampled_gap"].get<double>() <= tolerance;
                data["sections"].push_back(record);
                NurbsCurve curve = section.world;
                curve.name = "native section";
                curve.linecolors = {Color(220 / 255.f, 25 / 255.f, 180 / 255.f, 1.f)};
                curve.width = 3;
                scene.add_nurbscurve(curve);
            }
            data["passed"] = passed;
            if (!result.complete)
                data["interpolation_diagnostic"] = fit_failure(fa, fb, tolerance);
            else if (first >= 0 && second >= 0)
                data["support_diagnostic"] = fit_failure(fa, fb, tolerance);
            data["seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            failures += passed ? 0 : 1;
            if (!result.curves.empty() || !passed)
                scene.pb_dump((folder / (scene.name + ".pb")).string());
            records.push_back(data);
            const std::string tag = first < 0 && second < 0 ? "all" : std::to_string(first) + "_" + std::to_string(second);
            std::ofstream(folder / ("chair_sections_" + tag + ".json")) << nlohmann::ordered_json{
                {"backend", "session_cpp_native"}, {"tolerance", tolerance}, {"failures", failures}, {"pairs", records}}.dump(2);
        }
    return failures ? 1 : 0;
}
