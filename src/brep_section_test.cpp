#include "brep_section.h"
#include "brep_interference_internal.h"
#include "closest.h"
#include "nurbs_bounds.h"
#include <numeric>
#include "intersection.h"
#include "brep_classify.h"
#include "nurbs_coincidence.h"
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
    nurbs_coincidence::ParameterMap forward, inverse;
    MINI_CHECK(nurbs_coincidence::parameter_mapping(sphere.m_surfaces[0], remapped.m_surfaces[0], 1e-7, forward));
    MINI_CHECK(nurbs_coincidence::parameter_mapping(remapped.m_surfaces[0], sphere.m_surfaces[0], 1e-7, inverse));
    MINI_CHECK(forward.axis[0] == 0 && forward.axis[1] == 1);
    MINI_CHECK(forward.scale[0] == 1 && forward.scale[1] == 1);
    MINI_CHECK(forward.offset[0] == 10 && forward.offset[1] == 0);
    MINI_CHECK(inverse.axis[0] == 0 && inverse.axis[1] == 1);
    MINI_CHECK(inverse.scale[0] == 1 && inverse.scale[1] == 1);
    MINI_CHECK(inverse.offset[0] == -10 && inverse.offset[1] == 0);
    for (const BRepEdge& edge : remapped.m_edges)
        if (!edge.degenerated)
            for (const BRepCurveOnSurface& relation : edge.pcurves)
                for (int index : {relation.curve_2d_index, relation.curve_2d_index_2})
                    if (index >= 0)
                        MINI_CHECK(curve_surface_intersection::same_parameter(remapped.m_curves_3d[edge.curve_3d_index],
                            remapped.m_curves_2d[index], remapped.m_surfaces[relation.surface_index], 1e-7)
                            == curve_surface_intersection::ParameterMatch::Matched);
    const brep_section::FaceResult mapped = brep_section::faces(sphere, remapped, 1e-7);
    MINI_CHECK(mapped.complete && mapped.failure.empty());
    MINI_CHECK(mapped.curves.empty() && mapped.overlaps.size() == 1);
    MINI_CHECK(mapped.overlaps[0].first_face == 0 && mapped.overlaps[0].second_face == 0);
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


MINI_TEST("BRep Native Section", "Exact Local Trim Chart Preserves Physical Crossing") {
    const NurbsCurve horizontal = NurbsCurve::create(false, 1,
        {{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{-1,-1,0}});
    const NurbsCurve vertical = NurbsCurve::create(false, 1,
        {{-2,0,-1},{2,0,-1},{2,0,1},{-2,0,1},{-2,0,-1}});
    BRep first = BRep::from_nurbscurves({horizontal});
    BRep second = BRep::from_nurbscurves({vertical});
    const double shift[2] = {0x1p20, -0x1p19};
    for (BRep* shape : {&first, &second}) {
        for (NurbsSurface& surface : shape->m_surfaces)
            for (int d = 0; d < 2; ++d) {
                const auto domain = surface.domain(d);
                MINI_CHECK(surface.set_domain(d, domain.first+shift[d], domain.second+shift[d]));
            }
        for (NurbsCurve& pc : shape->m_curves_2d) {
            MINI_CHECK(pc.to_rational());
            for (int i = 0; i < pc.cv_count(); ++i) {
                const Point point = pc.get_cv(i);
                MINI_CHECK(pc.set_cv_4d(i, 2*(point[0]+shift[0]), 2*(point[1]+shift[1]), 0, 2));
            }
        }
        MINI_CHECK(shape->is_valid());
    }
    const auto first_before = first.jsondump(), second_before = second.jsondump();
    const double tolerance = 1e-7;
    for (bool swapped : {false, true}) {
        const BRep& a = swapped ? second : first;
        const BRep& b = swapped ? first : second;
        const auto result = brep_section::faces(a, b, tolerance);
        if (!result.complete) throw std::runtime_error(result.failure);
        MINI_CHECK(result.curves.size() == 1 && result.overlaps.empty());
        for (const auto& section : result.curves) {
            const Point start = section.world.point_at_start(), end = section.world.point_at_end();
            const Point low(-1,0,0), high(1,0,0);
            MINI_CHECK(std::min(start.distance(low)+end.distance(high),
                start.distance(high)+end.distance(low)) <= 2*tolerance);
            MINI_CHECK(section.world.domain() == section.first_pcurve.domain());
            MINI_CHECK(section.world.domain() == section.second_pcurve.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world, section.first_pcurve,
                a.m_surfaces[a.m_faces[section.first_face].surface_index], tolerance)
                == curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world, section.second_pcurve,
                b.m_surfaces[b.m_faces[section.second_face].surface_index], tolerance)
                == curve_surface_intersection::ParameterMatch::Matched);
            for (int i = 0; i <= 8; ++i) {
                const double t = section.world.domain_start() +
                    (section.world.domain_end()-section.world.domain_start())*i/8;
                const Point world = section.world.point_at(t);
                const Point ua = section.first_pcurve.point_at(t), ub = section.second_pcurve.point_at(t);
                MINI_CHECK(ua[0] > shift[0]-16 && ub[0] > shift[0]-16); // Raw chart survives output.
                MINI_CHECK(world.distance(a.m_surfaces[a.m_faces[section.first_face].surface_index].point_at(ua[0],ua[1])) <= tolerance);
                MINI_CHECK(world.distance(b.m_surfaces[b.m_faces[section.second_face].surface_index].point_at(ub[0],ub[1])) <= tolerance);
            }
        }
    }
    MINI_CHECK(first.jsondump() == first_before && second.jsondump() == second_before);
}


MINI_TEST("BRep Native Section", "Public Corner Contact Preserves Isolated Point") {
    const double tolerance = 1e-7;
    const BRep a = BRep::create_box(2, 2, 2);
    const BRep b = a.transformed(Xform::translation(2, 2, 2));
    const auto before_a = a.jsondump(), before_b = b.jsondump();
    // Independent closed-set intersection: [-1,1]^3 cap [1,3]^3 = {(1,1,1)}.
    // This is a topological vertex contact, not an unsupported surface tangency.
    for (bool swap : {false, true}) {
        const auto result = swap ? Intersection::brep_brep(b, a, tolerance)
                                 : Intersection::brep_brep(a, b, tolerance);
        MINI_CHECK(result.curves.empty());
        MINI_CHECK(result.points.size() == 1);
        if (result.points.size() == 1)
            MINI_CHECK(result.points[0].distance({1, 1, 1}) <= tolerance);
    }
    const BRep separated = a.transformed(Xform::translation(2.001, 2, 2));
    const auto empty = Intersection::brep_brep(a, separated, tolerance);
    MINI_CHECK(empty.points.empty() && empty.curves.empty());
    MINI_CHECK(a.jsondump() == before_a && b.jsondump() == before_b);
}

MINI_TEST("BRep Native Section", "Preserved Analytic Pullback Raw Parameters And Wrap") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_analytic_sphere_pullback_raw_parameters.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::ordered_json::parse(input);
    const auto& records = fixture.at("records");
    MINI_CHECK(records.size() == 2);
    const NurbsSurface first = NurbsSurface::jsonload(records[0].at("first_support"));
    const NurbsSurface second = NurbsSurface::jsonload(records[0].at("second_support"));
    const auto before_a = first.jsondump(), before_b = second.jsondump();
    // This is the original public Boolean default tolerance, not a relaxed fit target.
    const double tolerance = 1e-7;
    for (const auto& record : records) {
        const auto world = NurbsCurve::jsonload(record.at("world"));
        auto incorrect = NurbsCurve::jsonload(record.at("first_pcurve"));
        MINI_CHECK(incorrect.set_domain(world.domain_start(), world.domain_end()));
        // Merely fixing the endpoints retains the WRONG chord-length parameter law.
        MINI_CHECK(curve_surface_intersection::same_parameter(world, incorrect, first, tolerance) !=
                   curve_surface_intersection::ParameterMatch::Matched);
    }
    for (bool swap : {false, true}) {
        const auto& a = swap ? second : first;
        const auto& b = swap ? first : second;
        const auto triples = Intersection::surface_surface(a, b, tolerance);
        MINI_CHECK(triples.size() == 2);
        bool ordinary = false, wrapped = false;
        int positive_half = 0, negative_half = 0;
        const double radius = std::sqrt(.75);
        const Point upper(.5, 0, radius), lower(.5, 0, -radius);
        for (const auto& triple : triples) {
            const auto& world = std::get<0>(triple);
            const auto& pa = std::get<1>(triple);
            const auto& pb = std::get<2>(triple);
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            // Swapping operands may reverse/rotate world parameters, but must
            // still cover BOTH physical seam-free halves of the same circle.
            const Point start = world.point_at_start(), end = world.point_at_end();
            MINI_CHECK(std::min(std::max(start.distance(upper), end.distance(lower)),
                                std::max(start.distance(lower), end.distance(upper))) <= tolerance);
            const Point middle = world.point_at(world.domain_middle());
            MINI_CHECK(std::abs(middle[0] - .5) <= tolerance && std::abs(middle[2]) <= tolerance);
            MINI_CHECK(std::abs(std::abs(middle[1]) - radius) <= tolerance);
            if (middle[1] > 0)
                ++positive_half;
            else
                ++negative_half;
            ordinary = ordinary || world.domain() == std::pair<double, double>(.25, .75);
            wrapped = wrapped || world.domain() == std::pair<double, double>(.75, 1.25);
        }
        MINI_CHECK(positive_half == 1 && negative_half == 1);
        if (!swap)
            MINI_CHECK(ordinary && wrapped);
    }
    MINI_CHECK(first.jsondump() == before_a && second.jsondump() == before_b);
}


namespace {
BRep inventory_uv_rectangle(const NurbsSurface& support,double u0,double u1,double v0,double v1);
BRep inventory_compound(BRep a,const BRep& b);
BRep inventory_exact_face(const std::array<Point,4>& corners) {
    NurbsSurface support(3,false,2,2,2,2);
    support.m_nurbsknot[0]=support.m_nurbsknot[1]={0,1};
    for (int i=0;i<2;++i)
        for (int j=0;j<2;++j)
            MINI_CHECK(support.set_cv(i,j,corners[j?(i?2:3):(i?1:0)]));
    return inventory_uv_rectangle(support,0,1,0,1);
}
NurbsCurve inventory_rectangle(double x0,double x1,double y0,double y1,double z) {
    return NurbsCurve::create(false,1,{{x0,y0,z},{x1,y0,z},{x1,y1,z},{x0,y1,z},{x0,y0,z}});
}
}

MINI_TEST("BRep Native Section", "Shared Inventory Keeps Mixed Curve And Point Ownership") {
    const BRep first=inventory_compound(
        inventory_exact_face({Point(-1,-1,0),Point(1,-1,0),Point(1,1,0),Point(-1,1,0)}),
        inventory_exact_face({Point(4,4,0),Point(5,4,0),Point(5,5,0),Point(4,5,0)}));
    const BRep second=inventory_compound(
        inventory_exact_face({Point(-2,0,-1),Point(2,0,-1),Point(2,0,1),Point(-2,0,1)}),
        inventory_exact_face({Point(5,5,0),Point(6,5,0),Point(6,6,0),Point(5,6,0)}));
    MINI_CHECK(first.is_valid() && second.is_valid());
    const auto frozen_a=first.jsondump(),frozen_b=second.jsondump();
    const double tolerance=1e-7;
    for (bool swap:{false,true}) {
        const auto& a=swap?second:first;
        const auto& b=swap?first:second;
        const auto inventory=brep_section::interferences(a,b,tolerance);
        if (!inventory.complete)
            throw std::runtime_error(inventory.failure);
        MINI_CHECK(inventory.expected_pairs==4 && inventory.coverage.size()==4);
        // A certified returned line does not claim all original support branches.
        MINI_CHECK(!inventory.coverage_complete());
        MINI_CHECK(inventory.curve_representatives.size()==1);
        MINI_CHECK(inventory.support_points.size()==1);
        size_t isolated=0,owned=0;
        for (const auto& node:inventory.nodes) {
            if (node.isolated) {
                ++isolated;
                MINI_CHECK(node.point.distance({5,5,0})<=tolerance);
                MINI_CHECK(!node.support_points.empty());
            } else {
                ++owned;
                MINI_CHECK(std::abs(std::abs(node.point[0])-1)<=tolerance);
                MINI_CHECK(std::abs(node.point[1])<=tolerance && std::abs(node.point[2])<=tolerance);
                MINI_CHECK(!node.boundary_events.empty());
            }
        }
        MINI_CHECK(isolated==1 && owned==2);
        MINI_CHECK(!inventory.boundary_events.empty());
        for (const auto& event:inventory.boundary_events) {
            const auto& owner=event.operand?b:a;
            const auto& edge=owner.m_edges[event.edge];
            const auto& world=owner.m_curves_3d[edge.curve_3d_index];
            MINI_CHECK(event.hit.t>=world.domain_start() && event.hit.t<=world.domain_end());
            MINI_CHECK(!event.uses.empty());
            MINI_CHECK(world.point_at(event.hit.t).distance(event.hit.point)<=tolerance);
        }
        const auto section=Intersection::brep_brep(a,b,tolerance);
        MINI_CHECK(section.curves.size()==1 && section.points.size()==1);
        MINI_CHECK(section.coverage==SectionCoverage::LegacyLocusUnqualified);
        if (!section.curves.empty()) {
            const auto& line=section.curves.front();
            const Point lo(-1,0,0),hi(1,0,0);
            MINI_CHECK(std::min(std::max(line.point_at_start().distance(lo),line.point_at_end().distance(hi)),
                std::max(line.point_at_start().distance(hi),line.point_at_end().distance(lo)))<=tolerance);
        }
    }
    MINI_CHECK(first.jsondump()==frozen_a && second.jsondump()==frozen_b);
}

MINI_TEST("BRep Native Section", "Shared Inventory Qualifies Original Corner And Exclusion") {
    const BRep first=BRep::create_box(2,2,2);
    const BRep contact=first.transformed(Xform::translation(2,2,2));
    const BRep separated=first.transformed(Xform::translation(2.01,2,2));
    const auto frozen_a=first.jsondump(),frozen_b=contact.jsondump(),frozen_c=separated.jsondump();
    for (bool swap:{false,true}) {
        const auto& a=swap?contact:first;
        const auto& b=swap?first:contact;
        const auto inventory=brep_section::interferences(a,b,1e-7);
        if (!inventory.complete)
            throw std::runtime_error(inventory.failure);
        MINI_CHECK(inventory.coverage_complete());
        MINI_CHECK(inventory.expected_pairs==36 && inventory.coverage.size()==36);
        MINI_CHECK(inventory.curves.empty() && inventory.nodes.size()==1);
        MINI_CHECK(!inventory.boundary_events.empty());
        for (const auto& point:inventory.support_points) {
            MINI_CHECK(point.event.first_preimages.complete && point.event.second_preimages.complete);
            MINI_CHECK(point.event.point.distance({1,1,1})<=1e-7);
        }
        const auto section=Intersection::brep_brep(a,b,1e-7);
        MINI_CHECK(section.coverage==SectionCoverage::OriginalDomainQualified);
        MINI_CHECK(section.curves.empty() && section.points.size()==1);
        const auto empty=swap?brep_section::interferences(separated,first,1e-7):
                              brep_section::interferences(first,separated,1e-7);
        if (!empty.complete)
            throw std::runtime_error(empty.failure);
        MINI_CHECK(empty.coverage_complete() && empty.nodes.empty() && empty.curves.empty());
    }
    MINI_CHECK(first.jsondump()==frozen_a && contact.jsondump()==frozen_b && separated.jsondump()==frozen_c);
}


namespace {
BRep inventory_uv_rectangle(const NurbsSurface& support, double u0, double u1, double v0, double v1) {
    BRep result;
    result.add_surface(support);
    const std::array<Point, 4> uv = {Point(u0, v0, 0), Point(u1, v0, 0), Point(u1, v1, 0), Point(u0, v1, 0)};
    std::array<int, 4> vertices;
    for (int i = 0; i < 4; ++i) {
        const auto world = support.point_at(uv[i][0], uv[i][1]);
        vertices[i] = -1;
        for (int j = 0; j < i; ++j)
            if (world.distance(result.m_vertices[vertices[j]].point) == 0)
                vertices[i] = vertices[j];
        if (vertices[i] < 0)
            vertices[i] = result.add_vertex(world, 1e-7);
    }
    std::vector<BRepRef> wire;
    for (int i = 0; i < 4; ++i) {
        const int next = (i + 1) % 4, direction = i % 2;
        auto world = support.iso_curve(direction, direction ? uv[i][0] : uv[i][1]);
        const double lo = direction ? v0 : u0, hi = direction ? v1 : u1;
        MINI_CHECK(world.trim(lo, hi));
        if (i >= 2)
            world.reverse();
        bool constant = true;
        for (int cv = 1; cv < world.cv_count(); ++cv)
            constant = constant && world.get_cv(cv).distance(world.get_cv(0)) == 0;
        const int edge =
            result.add_edge(constant ? -1 : result.add_curve_3d(world), vertices[i], vertices[next], 1e-7);
        NurbsCurve pc(3, false, 2, 2);
        pc.m_nurbsknot = {lo, hi};
        MINI_CHECK(pc.set_cv(0, uv[i]) && pc.set_cv(1, uv[next]));
        result.add_pcurve(edge, 0, result.add_curve_2d(pc));
        if (!constant)
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pc, support, 1e-7) ==
                       curve_surface_intersection::ParameterMatch::Matched);
        wire.push_back({edge, BRepOrientation::Forward});
    }
    result.add_face(0, {{result.add_wire(wire), BRepOrientation::Forward}}, 1e-7);
    MINI_CHECK(result.is_valid());
    return result;
}
}

MINI_TEST("BRep Native Section", "Shared Inventory Retains Excluded Contact Incidence") {
    NurbsSurface sa(3,false,2,2,2,2),sb(3,false,2,2,2,2);
    sa.m_nurbsknot[0]=sa.m_nurbsknot[1]={0,1};
    sb.m_nurbsknot[0]=sb.m_nurbsknot[1]={0,1};
    for (int i=0;i<2;++i)
        for (int j=0;j<2;++j) {
            MINI_CHECK(sa.set_cv(i,j,{2.*i,2.*j,0}));
            MINI_CHECK(sb.set_cv(i,j,{2.+2.*i,2.+2.*j,0}));
        }
    const auto first=inventory_uv_rectangle(sa,0,.5,0,.5);
    const auto second=inventory_uv_rectangle(sb,0,1,0,1);
    const auto frozen_a=first.jsondump(),frozen_b=second.jsondump();
    for (bool swap:{false,true}) {
        const auto result=swap?brep_section::interferences(second,first,1e-7):
                               brep_section::interferences(first,second,1e-7);
        if (!result.complete)
            throw std::runtime_error(result.failure);
        MINI_CHECK(result.coverage_complete() && result.curves.empty() && result.nodes.empty());
        MINI_CHECK(result.support_points.size()==1);
        if (!result.support_points.empty()) {
            const auto& incidence=result.support_points.front();
            MINI_CHECK(!incidence.retained);
            MINI_CHECK(incidence.event.point.distance({2,2,0})<=1e-7);
            MINI_CHECK(incidence.event.first_preimages.complete && incidence.event.second_preimages.complete);
            MINI_CHECK((swap?incidence.second_membership:incidence.first_membership)==
                       surface_point_preimages::Membership::Excluded);
            MINI_CHECK((swap?incidence.first_membership:incidence.second_membership)==
                       surface_point_preimages::Membership::Included);
        }
    }
    MINI_CHECK(first.jsondump()==frozen_a && second.jsondump()==frozen_b);
}


namespace {
// Indexed append deliberately preserves distinct coincident original vertex IDs.
BRep inventory_compound(BRep a,const BRep& b) {
    const int nv=a.vertex_count(),ne=a.edge_count(),nf=a.face_count();
    const int nw=static_cast<int>(a.m_wires.size()),ns=static_cast<int>(a.m_surfaces.size());
    const int nc=static_cast<int>(a.m_curves_3d.size()),np=static_cast<int>(a.m_curves_2d.size());
    const int nh=static_cast<int>(a.m_shells.size());
    a.m_vertices.insert(a.m_vertices.end(),b.m_vertices.begin(),b.m_vertices.end());
    a.m_curves_3d.insert(a.m_curves_3d.end(),b.m_curves_3d.begin(),b.m_curves_3d.end());
    a.m_curves_2d.insert(a.m_curves_2d.end(),b.m_curves_2d.begin(),b.m_curves_2d.end());
    a.m_surfaces.insert(a.m_surfaces.end(),b.m_surfaces.begin(),b.m_surfaces.end());
    for (auto edge:b.m_edges) {
        edge.start_vertex+=nv;edge.end_vertex+=nv;
        if (edge.curve_3d_index>=0) edge.curve_3d_index+=nc;
        for (auto& image:edge.pcurves) {
            image.surface_index+=ns;
            if (image.curve_2d_index>=0) image.curve_2d_index+=np;
            if (image.curve_2d_index_2>=0) image.curve_2d_index_2+=np;
        }
        a.m_edges.push_back(std::move(edge));
    }
    for (auto wire:b.m_wires) {
        for (auto& edge:wire.edges) edge.index+=ne;
        a.m_wires.push_back(std::move(wire));
    }
    for (auto face:b.m_faces) {
        face.surface_index+=ns;
        for (auto& wire:face.wires) wire.index+=nw;
        a.m_faces.push_back(std::move(face));
    }
    for (auto shell:b.m_shells) {
        for (auto& face:shell.faces) face.index+=nf;
        a.m_shells.push_back(std::move(shell));
    }
    for (auto solid:b.m_solids) {
        for (auto& shell:solid.shells) shell.index+=nh;
        a.m_solids.push_back(std::move(solid));
    }
    return a;
}
}

MINI_TEST("BRep Native Section", "Shared Inventory Ignores Unused Component Bridge") {
    const auto box=BRep::create_box(2,2,2);
    const auto owner=inventory_compound(box.transformed(Xform::translation(-1,-1,-1)),
                                        box.transformed(Xform::translation(1,1,1)));
    MINI_CHECK(owner.is_valid() && owner.is_solid() && owner.solid_count()==2);
    std::vector<int> faces(owner.face_count());
    std::iota(faces.begin(),faces.end(),0);
    const auto before=brep_section::detail::active_components(owner,faces);
    std::vector<int> origins;
    for (int i=0;i<owner.vertex_count();++i)
        if (owner.m_vertices[i].point.distance({0,0,0})==0) origins.push_back(i);
    MINI_CHECK(origins.size()==2);
    if (origins.size()!=2) return;
    MINI_CHECK(before[origins[0]]!=before[origins[1]]);
    BRep bridged=owner;
    const int bridge=bridged.add_edge(bridged.add_curve_3d(
        NurbsCurve::create(false,1,{{0,0,0},{0,0,0}})),origins[0],origins[1],1e-7);
    MINI_CHECK(bridged.is_valid() && bridged.is_solid());
    for (int face:faces) {
        const auto boundary=brep_face_boundary::extract(bridged,face);
        MINI_CHECK(boundary.complete);
        for (const auto& wire:boundary.wires)
            for (const auto& use:wire.uses) MINI_CHECK(use.edge.index!=bridge);
    }
    MINI_CHECK(brep_section::detail::active_components(bridged,faces)==before);
    // det=2; all nonzero points have x>0,y<0, hence both boxes meet B only at 0.
    const Xform shear(std::array<double,16>{1,-1,0,0, 2,-1,1,0, 1,-2,1,0, 0,0,0,1});
    const auto other=BRep::create_box(1,1,1).transformed(Xform::translation(.5,.5,.5)).transformed(shear);
    const auto frozen=bridged.jsondump(),frozen_other=other.jsondump(),frozen_owner=owner.jsondump();
    for (bool swap:{false,true}) {
        const auto clean=swap?other.boolean_union(owner,1e-7):owner.boolean_union(other,1e-7);
        const auto result=swap?other.boolean_union(bridged,1e-7):bridged.boolean_union(other,1e-7);
        MINI_CHECK(clean.is_valid() && clean.is_solid() && clean.solid_count()==3);
        MINI_CHECK(result.is_valid() && result.is_solid() && result.solid_count()==3);
        MINI_CHECK(std::abs(clean.volume_exact(1e-7)-18)<=1e-7);
        MINI_CHECK(std::abs(result.volume_exact(1e-7)-18)<=1e-7);
    }
    MINI_CHECK(bridged.jsondump()==frozen && other.jsondump()==frozen_other && owner.jsondump()==frozen_owner);
}

MINI_TEST("BRep Native Section", "Shared Inventory Declares Inactive Missing Pcurve Unsupported") {
    const auto original=BRep::create_box(2,2,2);
    for (auto orientation:{BRepOrientation::Internal,BRepOrientation::External}) {
        BRep source=original;
        const auto& surface=source.m_surfaces[source.m_faces[0].surface_index];
        const auto u=surface.domain(0),v=surface.domain(1);
        const Point a=surface.point_at(std::lerp(u.first,u.second,.25),std::midpoint(v.first,v.second));
        const Point b=surface.point_at(std::lerp(u.first,u.second,.75),std::midpoint(v.first,v.second));
        const int va=source.add_vertex(a,1e-7),vb=source.add_vertex(b,1e-7);
        const int edge=source.add_edge(source.add_curve_3d(NurbsCurve::create(false,1,{a,b})),va,vb,1e-7);
        source.m_faces[0].wires.push_back({source.add_wire({{edge,orientation}}),BRepOrientation::Forward});
        MINI_CHECK(source.is_valid() && source.is_solid());
        const auto boundary=brep_face_boundary::extract(source,0);
        MINI_CHECK(boundary.complete && boundary.wires.back().uses.front().pcurve==-1);
        const auto frozen=source.jsondump();
        for (bool swap:{false,true}) {
            const auto events=swap?brep_section::interferences(original,source,1e-7):
                                   brep_section::interferences(source,original,1e-7);
            MINI_CHECK(!events.complete && !events.coverage_complete());
            MINI_CHECK(!events.unresolved_incidences.empty());
            if (!events.unresolved_incidences.empty()) {
                const auto& failure=events.unresolved_incidences.front();
                MINI_CHECK(failure.kind==brep_section::FailureKind::InactiveBoundaryUse);
                MINI_CHECK(failure.edge==edge && failure.use.pcurve==-1 && failure.use.orientation==orientation);
            }
        }
        MINI_CHECK(source.jsondump()==frozen);
    }
}

MINI_TEST("BRep Native Section", "Shared Pole Contact Keeps Mixed Degenerate UV Ownership") {
    const auto sphere=BRep::create_sphere(2);
    const auto plane=inventory_exact_face({Point(0,0,2),Point(2,0,2),Point(2,2,2),Point(0,2,2)});
    const auto frozen_s=sphere.jsondump(),frozen_p=plane.jsondump();
    int degenerate=-1;bool ordinary_endpoint=false;
    const auto boundary=brep_face_boundary::extract(sphere,0);
    MINI_CHECK(boundary.complete);
    for (const auto& wire:boundary.wires)
        for (const auto& use:wire.uses) {
            const auto& edge=sphere.m_edges[use.edge.index];
            if (sphere.m_vertices[edge.start_vertex].point.distance({0,0,2})==0 && use.degenerated)
                degenerate=use.edge.index;
            if (!use.degenerated)
                ordinary_endpoint=ordinary_endpoint || sphere.m_vertices[edge.start_vertex].point.distance({0,0,2})==0 ||
                    sphere.m_vertices[edge.end_vertex].point.distance({0,0,2})==0;
        }
    MINI_CHECK(degenerate>=0 && ordinary_endpoint);
    for (bool swap:{false,true}) {
        const auto events=swap?brep_section::interferences(plane,sphere,1e-7):
                               brep_section::interferences(sphere,plane,1e-7);
        if (!events.complete)
            throw std::runtime_error(events.failure);
        MINI_CHECK(events.coverage_complete() && events.nodes.size()==1 && events.curves.empty());
        bool retained=false,collapsed=false,represented=false,ordinary=false;
        for (const auto& incidence:events.support_points) {
            retained=retained || incidence.retained;
            const auto& preimages=swap?incidence.event.second_preimages:incidence.event.first_preimages;
            for (const auto& domain:preimages.domains)
                collapsed=collapsed || (domain.uv[0]<domain.uv[1] && domain.uv[2]==2 && domain.uv[3]==2);
        }
        for (const auto& use:events.uv_only_incidences)
            if (use.operand==static_cast<int>(swap) && use.edge==degenerate) {
                represented=true;
                MINI_CHECK(use.vertex==sphere.m_edges[degenerate].start_vertex);
                MINI_CHECK(!use.uses.empty() && use.uses.size()==use.pcurves.size());
                for (size_t i=0;i<use.uses.size();++i) {
                    const auto& original=sphere.m_curves_2d[use.uses[i].pcurve];
                    MINI_CHECK(use.pcurves[i].domain()==original.domain());
                    const auto& retained_pc=use.pcurves[i];
                    MINI_CHECK(retained_pc.dimension()==original.dimension() &&
                               retained_pc.is_rational()==original.is_rational() &&
                               retained_pc.degree()==original.degree() && retained_pc.cv_count()==original.cv_count());
                    MINI_CHECK(retained_pc.m_nurbsknot==original.m_nurbsknot);
                    for (int cv=0;cv<original.cv_count();++cv)
                        for (int d=0;d<original.dimension()+static_cast<int>(original.is_rational());++d)
                            MINI_CHECK(retained_pc.cv(cv)[d]==original.cv(cv)[d]);
                }
            }
        if (!events.nodes.empty()) {
            const auto& node=events.nodes.front();
            MINI_CHECK(node.isolated && node.point.distance({0,0,2})<=1e-7);
            MINI_CHECK(!node.uv_only_incidences.empty());
            for (size_t index:node.boundary_events) {
                const auto& event=events.boundary_events[index];
                if (event.operand==static_cast<int>(swap)) {
                    ordinary=true;
                    MINI_CHECK(event.vertex==sphere.m_edges[degenerate].start_vertex);
                }
            }
        }
        MINI_CHECK(retained && collapsed && represented && ordinary);
        const auto section=swap?Intersection::brep_brep(plane,sphere,1e-7):Intersection::brep_brep(sphere,plane,1e-7);
        MINI_CHECK(section.points.size()==1 && section.curves.empty());
        MINI_CHECK(section.coverage==SectionCoverage::OriginalDomainQualified);
    }
    MINI_CHECK(sphere.jsondump()==frozen_s && plane.jsondump()==frozen_p);
}

MINI_TEST("BRep Native Section", "Point Ownership Does Not Add CC And Source Residuals") {
    const double tolerance=1e-4;
    const auto source=NurbsCurve::create(false,1,{{-1,0,0},{1,0,0}});
    auto raw_source=source;
    MINI_CHECK(raw_source.set_domain(-1,1));
    const auto target=NurbsCurve::create(false,1,{{.9*tolerance,0,0},{.9*tolerance,1,0}});
    const Point candidate(-.2*tolerance,0,0);
    // CC may legitimately supply raw hit a=0,b=0 with residual .9Tol.
    MINI_CHECK(raw_source.point_at(0).distance(target.point_at(0))<tolerance);
    MINI_CHECK(nurbs_interval::curve_within_ball(raw_source,{- .2*tolerance,0},
        {candidate[0],candidate[1],candidate[2]},tolerance*.25));
    // But the actual candidate is 1.1Tol away from the purported target owner.
    MINI_CHECK(candidate.distance(target.point_at(0))>tolerance);
    MINI_CHECK(!brep_section::detail::owns_point(target,0,candidate,tolerance));
    const Point included(-.05*tolerance,0,0);
    MINI_CHECK(included.distance(target.point_at(0))<tolerance);
    MINI_CHECK(brep_section::detail::owns_point(target,0,included,tolerance));
}


MINI_TEST("BRep Native Section", "UV Only Ownership Certifies Interior Not Only Endpoints") {
    // Exact S(u,v)=(u,v(1-v),0). Its v-edge ends at P twice but reaches .25 away.
    NurbsSurface surface(3,false,2,3,2,3);
    surface.m_nurbsknot[0]={0,1};surface.m_nurbsknot[1]={0,0,1,1};
    for (int i=0;i<2;++i)
        for (int j=0;j<3;++j)
            MINI_CHECK(surface.set_cv(i,j,{static_cast<double>(i),j==1?.5:0,0}));
    auto source=inventory_uv_rectangle(surface,0,1,0,1);
    const auto& last=source.m_wires[source.m_faces[0].wires.front().index].edges.back();
    auto& declared=source.m_edges[last.index];
    MINI_CHECK(declared.start_vertex==declared.end_vertex);
    declared.degenerated=true;declared.curve_3d_index=-1;
    MINI_CHECK(source.is_valid());
    const auto other=inventory_exact_face({Point(-1,-1,0),Point(0,-1,0),Point(0,0,0),Point(-1,0,0)});
    nurbs_interval::SurfaceBounds actual;
    MINI_CHECK(nurbs_interval::surface_bounds(surface,{0,0,.5,.5},{0,0,0},actual,false));
    MINI_CHECK(actual.point[1].lo>.2);
    const auto frozen=source.jsondump(),frozen_other=other.jsondump();
    for (bool swap:{false,true}) {
        const auto events=swap?brep_section::interferences(other,source,1e-7):
                               brep_section::interferences(source,other,1e-7);
        MINI_CHECK(!events.complete && !events.coverage_complete());
        bool unresolved=false;
        for (const auto& failure:events.unresolved_incidences)
            unresolved=unresolved || (failure.kind==brep_section::FailureKind::UnrepresentedDegenerateUse &&
                                      failure.edge==last.index && failure.operand==static_cast<int>(swap));
        MINI_CHECK(unresolved);
    }
    MINI_CHECK(source.jsondump()==frozen && other.jsondump()==frozen_other);
}


namespace {
NurbsCurve ownership_yz(double x, double lower) {
    return NurbsCurve::create(false,1,{{x,lower,-1},{x,1,-1},{x,1,1},{x,lower,1},{x,lower,-1}});
}

// Whole ORIGINAL spans, directed lower distance. A failed bound fails this test;
// it never accepts isolation by sampling or a Closest proposal.
bool ownership_curve_excludes(const NurbsCurve& curve, const Point& point, double tolerance) {
    const auto spans = curve.get_span_vector();
    const auto allowed = nurbs_interval::Interval(tolerance) * nurbs_interval::Interval(tolerance);
    if (spans.size()<2 || !nurbs_interval::finite(allowed)) return false;
    for (size_t i=1;i<spans.size();++i) {
        nurbs_interval::CurveBounds bound;
        if (!nurbs_interval::curve_bounds(curve,{spans[i-1],spans[i]},
            {point[0],point[1],point[2]},bound)) return false;
        nurbs_interval::Interval squared;
        for (const auto coordinate:bound.point) {
            if (!nurbs_interval::finite(coordinate)) return false;
            const double lower=coordinate.lo>0?coordinate.lo:coordinate.hi<0?-coordinate.hi:0;
            const nurbs_interval::Interval nearest(lower);
            squared=squared+nearest*nearest;
        }
        if (!nurbs_interval::finite(squared) || !(squared.lo>allowed.hi)) return false;
    }
    return true;
}
}

MINI_TEST("BRep Native Section", "Trimmed Point Contact Does Not Manufacture Positive Curve") {
    const double tolerance=std::ldexp(1.,-10);
    const double p=15*tolerance/64,q=63*tolerance/64;
    const Point point(p,0,0);
    // Exact dyadic proof: p<Tol/4, q<Tol, but p^2+q^2 =4194/4096 Tol^2 >Tol^2.
    MINI_CHECK(p<tolerance*.25 && q<tolerance);
    MINI_CHECK(p*p+q*q>tolerance*tolerance);
    const auto xy=NurbsCurve::create(false,1,
        {{-1,-1,0},{1,-1,0},{1,0,0},{-1,0,0},{-1,-1,0}});
    const BRep first=BRep::from_nurbscurves({xy,ownership_yz(0,q)});
    const BRep second=BRep::from_nurbscurves({ownership_yz(p,0)});
    MINI_CHECK(first.is_valid() && second.is_valid());
    MINI_CHECK(first.face_count()==2 && second.face_count()==1);
    // Despite q<Tol, the existing conservative physical->UV metric makes y=0
    // strictly Outside A2's actual rectangle. This excludes B's lower border.
    const auto& support=first.m_surfaces.at(first.m_faces.at(1).surface_index);
    const auto [u,v,gap]=Closest::surface_point(support,Point(0,0,0));
    MINI_CHECK(std::isfinite(gap));
    MINI_CHECK(support.point_at(u,v).distance(Point(0,0,0))<1e-12);
    MINI_CHECK(brep_classify::face_uv(first,1,Point(u,v,0),
        tolerance/nurbs_bounds::surface_metric(support))==brep_classify::State::Outside);
    const auto before_a=first.jsondump(),before_b=second.jsondump();
    for (bool swap:{false,true}) {
        const auto& a=swap?second:first;
        const auto& b=swap?first:second;
        const auto inventory=brep_section::interferences(a,b,tolerance);
        if (!inventory.complete) throw std::runtime_error(inventory.failure);
        MINI_CHECK(inventory.curves.empty());
        size_t isolated=0;
        for (const auto& node:inventory.nodes)
            if (node.isolated && node.point.distance(point)<tolerance/1000) ++isolated;
        MINI_CHECK(isolated==1);
        const auto section=Intersection::brep_brep(a,b,tolerance);
        size_t public_isolated=0;
        for (const auto& candidate:section.points)
            if (candidate.distance(point)<tolerance/1000) ++public_isolated;
        MINI_CHECK(public_isolated==1);
    }
    MINI_CHECK(first.jsondump()==before_a && second.jsondump()==before_b);
}


MINI_TEST("BRep Native Section", "Curve Ownership Cannot Spend Source Cap Plus Full Hit Tolerance") {
    const double tolerance=std::ldexp(1.,-10);
    const double p=15*tolerance/64,q=63*tolerance/64;
    const Point point(p,0,0);
    // Exact dyadic proof: p<Tol/4, q<Tol, but p^2+q^2 =4194/4096 Tol^2 >Tol^2.
    MINI_CHECK(p<tolerance*.25 && q<tolerance);
    MINI_CHECK(p*p+q*q>tolerance*tolerance);
    const auto xy=NurbsCurve::create(false,1,
        {{-1,-1,0},{1,-1,0},{1,0,0},{-1,0,0},{-1,-1,0}});
    const BRep first=BRep::from_nurbscurves({xy,ownership_yz(0,q)});
    const auto xz=NurbsCurve::create(false,1,
        {{-1,q,-1},{0,q,-1},{0,q,1},{-1,q,1},{-1,q,-1}});
    const BRep second=BRep::from_nurbscurves({ownership_yz(p,0),xz});
    MINI_CHECK(first.is_valid() && second.is_valid());
    MINI_CHECK(first.face_count()==2 && second.face_count()==2);
    // Despite q<Tol, the existing conservative physical->UV metric makes y=0
    // strictly Outside A2's actual rectangle. This excludes B's lower border.
    const auto& support=first.m_surfaces.at(first.m_faces.at(1).surface_index);
    const auto [u,v,gap]=Closest::surface_point(support,Point(0,0,0));
    MINI_CHECK(std::isfinite(gap));
    MINI_CHECK(support.point_at(u,v).distance(Point(0,0,0))<1e-12);
    MINI_CHECK(brep_classify::face_uv(first,1,Point(u,v,0),
        tolerance/nurbs_bounds::surface_metric(support))==brep_classify::State::Outside);
    const auto before_a=first.jsondump(),before_b=second.jsondump();
    for (bool swap:{false,true}) {
        const auto& a=swap?second:first;
        const auto& b=swap?first:second;
        const auto inventory=brep_section::interferences(a,b,tolerance);
        if (!inventory.complete) throw std::runtime_error(inventory.failure);
        MINI_CHECK(!inventory.curves.empty());
        // Verify ALL actual incidences, not only the public representative.
        // If the producer emits a closer curve, preserve the inventory and
        // diagnose it; do not adjust the fixture or weaken this assertion.
        for (const auto& curve:inventory.curves)
            MINI_CHECK(ownership_curve_excludes(curve.world,point,tolerance));
        size_t isolated=0;
        for (const auto& node:inventory.nodes)
            if (node.isolated && node.point.distance(point)<tolerance/1000) ++isolated;
        MINI_CHECK(isolated==1);
        const auto section=Intersection::brep_brep(a,b,tolerance);
        size_t public_isolated=0;
        for (const auto& candidate:section.points)
            if (candidate.distance(point)<tolerance/1000) ++public_isolated;
        MINI_CHECK(public_isolated==1);
    }
    MINI_CHECK(first.jsondump()==before_a && second.jsondump()==before_b);
}


MINI_TEST("BRep Native Section", "Regular Meridian Retains Unrepresented Pole UV Uses") {
    const auto sphere=BRep::create_sphere(2);
    const auto plane=inventory_exact_face({Point(0,-3,-3),Point(0,3,-3),Point(0,3,3),Point(0,-3,3)});
    const auto frozen_s=sphere.jsondump(),frozen_p=plane.jsondump();
    for (bool swap:{false,true}) {
        const auto& a=swap?plane:sphere;
        const auto& b=swap?sphere:plane;
        const auto events=brep_section::interferences(a,b,1e-7);
        if (!events.complete)
            throw std::runtime_error(events.failure);
        MINI_CHECK(!events.curves.empty() && !events.curve_representatives.empty());
        MINI_CHECK(!events.boundary_ownership_complete && !events.coverage_complete());
        MINI_CHECK(events.support_points.empty() && events.uv_only_incidences.empty());
        bool north=false,south=false;
        for (const auto& curve:events.curves) {
            MINI_CHECK(curve_surface_intersection::same_parameter(curve.world,curve.first_pcurve,
                a.m_surfaces[a.m_faces[curve.first_face].surface_index],1e-7)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(curve.world,curve.second_pcurve,
                b.m_surfaces[b.m_faces[curve.second_face].surface_index],1e-7)==
                curve_surface_intersection::ParameterMatch::Matched);
        }
        for (const auto& missing:events.unrepresented_uv_incidences) {
            MINI_CHECK(missing.operand==static_cast<int>(swap));
            MINI_CHECK(missing.node<events.nodes.size());
            const auto& edge=sphere.m_edges[missing.edge];
            MINI_CHECK(edge.degenerated && missing.vertex==edge.start_vertex);
            MINI_CHECK(missing.use.face==0 && missing.use.pcurve>=0);
            MINI_CHECK(missing.pcurve_domain==sphere.m_curves_2d[missing.use.pcurve].domain());
            if (missing.node>=events.nodes.size()) continue;
            const auto& node=events.nodes[missing.node];
            MINI_CHECK(!node.isolated && node.support_points.empty());
            MINI_CHECK(!node.unrepresented_uv_incidences.empty());
            bool original_endpoint=false;
            for (size_t index:node.boundary_events) {
                const auto& event=events.boundary_events[index];
                original_endpoint=original_endpoint || (event.operand==missing.operand && event.vertex==missing.vertex);
            }
            MINI_CHECK(original_endpoint);
            north=north || node.point.distance({0,0,2})<=1e-7;
            south=south || node.point.distance({0,0,-2})<=1e-7;
        }
        MINI_CHECK(north && south);
        for (const Point target:{Point(0,2,0),Point(0,-2,0),Point(0,0,2),Point(0,0,-2)}) {
            bool represented=false;
            for (size_t index:events.curve_representatives) {
                const auto& curve=events.curves[index].world;
                const auto [parameter,gap]=Closest::curve_point(curve,target);
                represented=represented || (std::isfinite(parameter) && std::isfinite(gap) &&
                                             curve.point_at(parameter).distance(target)<=1e-7);
            }
            MINI_CHECK(represented); // Independent physical meridian coverage probes, not a full-domain proof.
        }
        const auto section=Intersection::brep_brep(a,b,1e-7);
        MINI_CHECK(!section.curves.empty() && section.points.empty());
        MINI_CHECK(section.coverage==SectionCoverage::LegacyLocusUnqualified);
    }
    MINI_CHECK(sphere.jsondump()==frozen_s && plane.jsondump()==frozen_p);
}


MINI_TEST("BRep Native Section", "Whole Target Euclidean Hull Excludes Only Proved Images") {
    const double tolerance=std::ldexp(1.,-10),p=15*tolerance/64,q=63*tolerance/64;
    const Point point(p,0,0);
    MINI_CHECK(p<tolerance && q<tolerance && p*p+q*q>tolerance*tolerance);
    for (bool rational:{false,true})
        for (bool reverse:{false,true}) {
            auto target=NurbsCurve::create(false,1,{{0,q,-1},{0,q,1}});
            MINI_CHECK(target.set_domain(-7,13));
            if (rational) {
                MINI_CHECK(target.set_cv_4d(0,0,2*q,-2,2));
                MINI_CHECK(target.set_cv_4d(1,0,3*q,3,3));
            }
            if (reverse) MINI_CHECK(target.reverse());
            const auto before=target.jsondump();
            MINI_CHECK(brep_section::detail::curve_excludes_point(target,point,tolerance));
            // P' lies ON the actual target image, so even a loose hull must not reject it.
            MINI_CHECK(!brep_section::detail::curve_excludes_point(target,{0,q,0},tolerance));
            MINI_CHECK(target.jsondump()==before);
        }
    auto unsupported=NurbsCurve::create(false,1,{{0,q,-1},{0,q,1}});
    MINI_CHECK(unsupported.set_cv_4d(0,0,2*q,-2,2));
    MINI_CHECK(unsupported.set_cv_4d(1,0,-q,-1,-1));
    MINI_CHECK(!brep_section::detail::curve_excludes_point(unsupported,point,tolerance));
}


namespace session_cpp {
namespace {
void check_original_trim_receipts(const BRep& a,const BRep& b,double tolerance,bool require_overlap) {
    const auto before_a=a.jsondump(),before_b=b.jsondump();
    const auto inventory=brep_section::interferences(a,b,tolerance);
    if (!inventory.complete) throw std::runtime_error(inventory.failure);
    MINI_CHECK(!inventory.original_trim_laws.empty());
    bool side[2]={false,false},overlap=false;
    for (const auto& law:inventory.original_trim_laws) {
        const auto& original=law.original;
        MINI_CHECK(original.world.domain()==original.first_pcurve.domain());
        MINI_CHECK(original.world.domain()==original.second_pcurve.domain());
        for (const auto& event:law.events) {
            MINI_CHECK(event.operand==0 || event.operand==1);
            const auto& owner=event.operand==0?a:b;
            const int face=event.operand==0?original.first_face:original.second_face;
            const auto& pc=event.operand==0?original.first_pcurve:original.second_pcurve;
            MINI_CHECK(event.face==face && event.wire>=0 && event.occurrence>=0);
            const auto found=std::find_if(owner.m_faces[face].wires.begin(),owner.m_faces[face].wires.end(),
                [&](const BRepRef& wire){return wire.index==event.wire;});
            MINI_CHECK(found!=owner.m_faces[face].wires.end());
            if (found==owner.m_faces[face].wires.end()) continue;
            const auto uses=owner.wire_edges(*found);
            MINI_CHECK(event.occurrence<static_cast<int>(uses.size()));
            if (event.occurrence>=static_cast<int>(uses.size())) continue;
            const auto& use=uses[event.occurrence];
            MINI_CHECK(use.index==event.edge && use.orientation==event.orientation);
            MINI_CHECK(owner.pcurve_index(use.index,face,use.orientation)==event.pcurve);
            const auto& boundary=owner.m_curves_2d[event.pcurve];
            MINI_CHECK(event.section_parameter>=pc.domain_start() && event.section_parameter<=pc.domain_end());
            MINI_CHECK(event.boundary_parameter>=boundary.domain_start() && event.boundary_parameter<=boundary.domain_end());
            const auto& support=owner.m_surfaces[owner.m_faces[face].surface_index];
            const double uv_tol=tolerance/nurbs_bounds::surface_metric(support);
            MINI_CHECK(pc.point_at(event.section_parameter).distance(boundary.point_at(event.boundary_parameter))<=uv_tol);
            MINI_CHECK(curve_surface_intersection::same_parameter(original.world,pc,support,tolerance)==
                curve_surface_intersection::ParameterMatch::Matched);
            side[event.operand]=true;overlap=overlap || event.overlap_endpoint;
        }
    }
    MINI_CHECK(side[0] && side[1]);
    if (require_overlap) MINI_CHECK(overlap);
    MINI_CHECK(a.jsondump()==before_a && b.jsondump()==before_b);
}
}
MINI_TEST("BRep Native Section", "Original Trim Receipts Preserve Both Raw Parameters And Uses") {
    const auto first=inventory_exact_face({Point(-1,-1,0),Point(1,-1,0),Point(1,1,0),Point(-1,1,0)});
    for (bool boundary:{false,true}) {
        const double x=boundary?1:0;
        auto second=inventory_exact_face({Point(x,-1,-1),Point(x,1,-1),Point(x,1,1),Point(x,-1,1)});
        for (auto& pc:second.m_curves_2d) MINI_CHECK(pc.set_domain(-7,13));
        for (auto& world:second.m_curves_3d) MINI_CHECK(world.set_domain(32,35));
        check_original_trim_receipts(first,second,1e-7,boundary);
        check_original_trim_receipts(second,first,1e-7,boundary);
    }
}
} // namespace session_cpp
