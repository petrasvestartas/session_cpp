#include "brep_section.h"
#include "brep_classify.h"
#include "curve_surface_intersection.h"
#include "mini_test.h"
#include "xform.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

using namespace session_cpp;
using namespace session_cpp::mini_test;

MINI_TEST("BRep Native Section", "Curve Through Trimmed Bore") {
    const BRep bore = BRep::create_block_with_hole(4, 4, 2, 1);
    const NurbsCurve axis = NurbsCurve::create(false, 1, {{0, 0, -3}, {0, 0, 3}});
    const brep_section::CurveResult hole = brep_section::curve(axis, bore, 1e-7);
    if (!hole.complete)
        throw std::runtime_error(hole.failure);
    MINI_CHECK(hole.points.empty() && hole.overlaps.empty());
    const NurbsCurve crossing = NurbsCurve::create(false, 1, {{-3, 0.2, 0}, {3, 0.2, 0}});
    const brep_section::CurveResult wall = brep_section::curve(crossing, bore, 1e-7);
    if (!wall.complete)
        throw std::runtime_error(wall.failure);
    MINI_CHECK(wall.points.size() == 4 && wall.overlaps.empty());
    for (const brep_section::CurveHit& hit : wall.points) {
        const NurbsSurface& surface = bore.m_surfaces[bore.m_faces[hit.face].surface_index];
        MINI_CHECK(hit.point.distance(crossing.point_at(hit.t)) < 1e-7);
        MINI_CHECK(hit.point.distance(surface.point_at(hit.u, hit.v)) < 1e-7);
    }
}

MINI_TEST("BRep Native Section", "Coplanar Curve Across Hole") {
    const BRep bore = BRep::create_block_with_hole(4, 4, 2, 1);
    const NurbsCurve crossing = NurbsCurve::create(false, 1, {{-3, 0, 1}, {3, 0, 1}});
    const brep_section::CurveResult result = brep_section::curve(crossing, bore, 1e-7);
    if (!result.complete)
        throw std::runtime_error(result.failure);
    double length = 0.0;
    for (const brep_section::CurveSpan& span : result.overlaps) {
        const Point a = crossing.point_at(span.first);
        const Point b = crossing.point_at(span.last);
        length += a.distance(b);
        MINI_CHECK(std::abs((a[0] + b[0]) * 0.5) >= 1.0);
        MINI_CHECK(span.pcurve.domain_start() == span.first && span.pcurve.domain_end() == span.last);
    }
    MINI_CHECK(result.overlaps.size() == 2);
    MINI_CHECK(std::abs(length - 2.0) < 1e-6);
}

MINI_TEST("BRep Native Section", "Transverse Boxes Paired Pcurves") {
    const BRep first = BRep::create_box(2, 2, 2);
    const BRep second = first.transformed(Xform::translation(0.5, 0.7, 0.9));
    const brep_section::FaceResult result = brep_section::faces(first, second, 1e-7);
    if (!result.complete)
        throw std::runtime_error(result.failure);
    MINI_CHECK(result.curves.size() == 6);
    for (const brep_section::FaceCurve& section : result.curves)
        for (int i = 0; i <= 8; ++i) {
            const double t = section.world.domain_start() +
                (section.world.domain_end() - section.world.domain_start()) * i / 8.0;
            const Point p = section.world.point_at(t);
            const Point ua = section.first_pcurve.point_at(t);
            const Point ub = section.second_pcurve.point_at(t);
            const NurbsSurface& a = first.m_surfaces[first.m_faces[section.first_face].surface_index];
            const NurbsSurface& b = second.m_surfaces[second.m_faces[section.second_face].surface_index];
            MINI_CHECK(p.distance(a.point_at(ua[0], ua[1])) < 1e-7);
            MINI_CHECK(p.distance(b.point_at(ub[0], ub[1])) < 1e-7);
            MINI_CHECK(brep_classify::face_uv(first, section.first_face, ua, 1e-7) != brep_classify::State::Outside);
            MINI_CHECK(brep_classify::face_uv(second, section.second_face, ub, 1e-7) != brep_classify::State::Outside);
        }
}

MINI_TEST("BRep Native Section", "Coincident Support Is Explicit") {
    const BRep sphere = BRep::create_sphere(1.0);
    const brep_section::FaceResult result = brep_section::faces(sphere, sphere, 1e-7);
    MINI_CHECK(result.complete);
    MINI_CHECK(result.curves.empty() && result.overlaps.size() == 1);
    MINI_CHECK(result.overlaps[0].first_face == 0 && result.overlaps[0].second_face == 0);

    BRep remapped = sphere;
    MINI_CHECK(remapped.m_surfaces[0].set_domain(0, 10.0, 14.0));
    for (NurbsCurve& pcurve : remapped.m_curves_2d)
        for (int i = 0; i < pcurve.cv_count(); ++i)
            pcurve.cv(i)[0] += 10.0 * pcurve.weight(i);
    MINI_CHECK(remapped.is_solid());
    const brep_section::FaceResult unsupported = brep_section::faces(sphere, remapped, 1e-7);
    MINI_CHECK(!unsupported.complete && !unsupported.failure.empty());
}

MINI_TEST("BRep Native Section", "Consecutive Retained Endpoint Intervals") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_section_retained_endpoint_runs.json");
    MINI_CHECK(input.good());
    const nlohmann::json fixture = nlohmann::json::parse(input);
    const BRep first = BRep::jsonload(fixture.at("first_support"));
    const BRep second = BRep::jsonload(fixture.at("second_support"));
    const double tolerance = fixture.at("tolerance").get<double>();
    const auto endpoint = [&](int i) {
        const nlohmann::json& p = fixture.at("expected_endpoints").at(i);
        return Point(p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>());
    };
    for (bool swap : {false, true}) {
        const BRep& a = swap ? second : first;
        const BRep& b = swap ? first : second;
        const brep_section::FaceResult result = brep_section::faces(a, b, tolerance);
        if (!result.complete)
            throw std::runtime_error(result.failure);
        MINI_CHECK(result.curves.size() == 1 && result.overlaps.empty());
        const brep_section::FaceCurve& section = result.curves[0];
        const Point start = section.world.point_at_start();
        const Point end = section.world.point_at_end();
        MINI_CHECK(std::min(start.distance(endpoint(0)) + end.distance(endpoint(1)),
            start.distance(endpoint(1)) + end.distance(endpoint(0))) < tolerance * 2);
        MINI_CHECK(section.world.domain() == section.first_pcurve.domain());
        MINI_CHECK(section.world.domain() == section.second_pcurve.domain());
        MINI_CHECK(curve_surface_intersection::same_parameter(section.world, section.first_pcurve,
            a.m_surfaces[a.m_faces[section.first_face].surface_index], tolerance)
            == curve_surface_intersection::ParameterMatch::Matched);
        MINI_CHECK(curve_surface_intersection::same_parameter(section.world, section.second_pcurve,
            b.m_surfaces[b.m_faces[section.second_face].surface_index], tolerance)
            == curve_surface_intersection::ParameterMatch::Matched);
    }
}

MINI_TEST("BRep Native Section", "Rejected Hole Separates Retained Runs") {
    const NurbsCurve outer = NurbsCurve::create(false, 1,
        {{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}, {0, 0, 0}});
    const NurbsCurve inner = NurbsCurve::create(false, 1,
        {{1, 1, 0}, {3, 1, 0}, {3, 3, 0}, {1, 3, 0}, {1, 1, 0}});
    const BRep first = BRep::from_nurbscurves({outer}, {{inner}});
    const BRep second = BRep::from_nurbscurves({NurbsCurve::create(false, 1,
        {{-1, 2, -1}, {5, 2, -1}, {5, 2, 1}, {-1, 2, 1}, {-1, 2, -1}})});
    const double tolerance = 1e-7;
    for (bool swap : {false, true}) {
        const BRep& a = swap ? second : first;
        const BRep& b = swap ? first : second;
        const brep_section::FaceResult result = brep_section::faces(a, b, tolerance);
        if (!result.complete)
            throw std::runtime_error(result.failure);
        MINI_CHECK(result.curves.size() == 2 && result.overlaps.empty());
        double length = 0;
        for (const brep_section::FaceCurve& section : result.curves) {
            const Point start = section.world.point_at_start();
            const Point end = section.world.point_at_end();
            length += start.distance(end);
            MINI_CHECK(std::max(start[0], end[0]) <= 1 + tolerance || std::min(start[0], end[0]) >= 3 - tolerance);
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world, section.first_pcurve,
                a.m_surfaces[a.m_faces[section.first_face].surface_index], tolerance)
                == curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world, section.second_pcurve,
                b.m_surfaces[b.m_faces[section.second_face].surface_index], tolerance)
                == curve_surface_intersection::ParameterMatch::Matched);
        }
        MINI_CHECK(std::abs(length - 2) < tolerance);
    }
}
