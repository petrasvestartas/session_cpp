#include "brep_section.h"
#include "brep_assemble.h"
#include "section_original_endpoint_alias.h"
#include "section_exact_knot_preimages.h"
#include "brep_polygon_trim_coverage.h"
#include "section_exact_linear_event.h"
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
        // This exact fixture now has a maximal original-domain line AND its
        // complete polygon trim proof; the independent corner contact and both
        // off-diagonal exclusions also have original-domain receipts.
        for (const auto& pair:inventory.coverage) {
            MINI_CHECK(pair.trim_ownership_complete);
            if (pair.first_face==0 && pair.second_face==0)
                MINI_CHECK(pair.support==surface_section::Coverage::OriginalDomainCurve);
            else if (pair.first_face==1 && pair.second_face==1)
                MINI_CHECK(pair.support==surface_section::Coverage::GlobalContactPoint);
            else
                MINI_CHECK(pair.support==surface_section::Coverage::OriginalDomainExcluded);
        }
        MINI_CHECK(inventory.coverage_complete());
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
        // Every pair's original support/trim theorem was asserted above.
        MINI_CHECK(section.coverage==SectionCoverage::OriginalDomainQualified);
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
        size_t owned_count=0;
        for (const auto& law:inventory.original_trim_laws)
            for (const auto& owned:law.owned_intervals) {
                ++owned_count;
                MINI_CHECK(owned.first<owned.last && owned.node<inventory.nodes.size());
                MINI_CHECK(owned.events.size()==owned.boundary_parameters.size());
                MINI_CHECK(owned.events.size()==owned.boundary_laws.size());
                for (int d=0;d<3;++d)
                    MINI_CHECK(nurbs_coincidence::exact_products_equal(point[d],owned.weight,owned.homogeneous[d],1.));
                if (owned.node>=inventory.nodes.size()) continue;
                const auto& node=inventory.nodes[owned.node];
                MINI_CHECK(nurbs_interval::curve_within_ball(law.original.world,{owned.first,owned.last},
                    {node.point[0],node.point[1],node.point[2]},tolerance));
                const brep_section::detail::ExactLinearEvent exact{owned.homogeneous,owned.weight,owned.center};
                bool first_raw=false,last_raw=false;
                for (size_t i=0;i<owned.events.size();++i) {
                    const size_t index=owned.events[i];
                    MINI_CHECK(index<law.events.size());
                    if (index>=law.events.size()) continue;
                    const auto& original=law.events[index];
                    first_raw=first_raw || original.section_parameter==owned.first;
                    last_raw=last_raw || original.section_parameter==owned.last;
                    const int other_face=original.operand==0?law.original.second_face:law.original.first_face;
                    bool matched=false;
                    for (size_t event:node.boundary_events) {
                        const auto& boundary=inventory.boundary_events[event];
                        if (boundary.operand!=original.operand || boundary.edge!=original.edge ||
                            boundary.hit.face!=other_face || boundary.hit.t<owned.boundary_parameters[i].first ||
                            boundary.hit.t>owned.boundary_parameters[i].second ||
                            !brep_section::detail::exact_event_at_raw_parameter(exact,owned.boundary_laws[i],boundary.hit.t)) continue;
                        for (const auto& use:boundary.uses)
                            matched=matched || (use.face==original.face && use.wire==original.wire &&
                                use.occurrence==original.occurrence && use.pcurve==original.pcurve);
                    }
                    MINI_CHECK(matched);
                }
                MINI_CHECK(first_raw && last_raw);
            }
        MINI_CHECK(owned_count>0);
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

namespace session_cpp {
MINI_TEST("BRep Native Section", "Adjacent Original Supports Preserve Refined Boundary Aliases") {
    const auto original = BRep::create_cylinder(1,2);
    auto refined = original;
    for (auto& world : refined.m_curves_3d) {
        MINI_CHECK(world.increase_degree(world.degree()+1));
        MINI_CHECK(world.insert_nurbsknot(world.domain_start()+.375*(world.domain_end()-world.domain_start())));
    }
    for (auto& pc : refined.m_curves_2d) {
        MINI_CHECK(pc.increase_degree(pc.degree()+2));
        MINI_CHECK(pc.insert_nurbsknot(pc.domain_start()+.625*(pc.domain_end()-pc.domain_start())));
    }
    const auto frozen_original = original.jsondump(), frozen_refined = refined.jsondump();
    for (bool swap : {false,true}) {
        const auto& a = swap ? refined : original;
        const auto& b = swap ? original : refined;
        const auto result = brep_section::interferences(a,b,1e-7);
        if (!result.complete) throw std::runtime_error(result.failure);
        MINI_CHECK(!result.curves.empty() && !result.boundary_events.empty());
        bool adjacent = false;
        for (const auto& curve : result.curves) {
            adjacent = adjacent || curve.first_face != curve.second_face;
            MINI_CHECK(curve.world.domain() == curve.first_pcurve.domain());
            MINI_CHECK(curve.world.domain() == curve.second_pcurve.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(curve.world,curve.first_pcurve,
                a.m_surfaces[a.m_faces[curve.first_face].surface_index],1e-7) ==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(curve.world,curve.second_pcurve,
                b.m_surfaces[b.m_faces[curve.second_face].surface_index],1e-7) ==
                curve_surface_intersection::ParameterMatch::Matched);
        }
        MINI_CHECK(adjacent); // Cap/cylinder incidences need the edge's other original support law.
    }
    MINI_CHECK(original.jsondump() == frozen_original && refined.jsondump() == frozen_refined);
}
} // namespace session_cpp

MINI_TEST("BRep Native Section", "Positive Short Section Is Not An Isolated Contact") {
    const double tol=std::ldexp(1.,-10), h=tol/8;
    const auto a=inventory_exact_face({Point(-1,-h,0),Point(1,-h,0),Point(1,h,0),Point(-1,h,0)});
    const auto b=inventory_exact_face({Point(0,-1,-1),Point(0,1,-1),Point(0,1,1),Point(0,-1,1)});
    const auto before_a=a.jsondump(),before_b=b.jsondump();
    for (bool swap:{false,true}) {
        const auto result=brep_section::interferences(swap?b:a,swap?a:b,tol);
        if (!result.complete) throw std::runtime_error(result.failure);
        bool positive=false;
        for (const auto& section:result.curves) {
            MINI_CHECK(section.world.domain()==section.first_pcurve.domain());
            MINI_CHECK(section.world.domain()==section.second_pcurve.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world,section.first_pcurve,
                (swap?b:a).m_surfaces[(swap?b:a).m_faces[section.first_face].surface_index],tol)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world,section.second_pcurve,
                (swap?a:b).m_surfaces[(swap?a:b).m_faces[section.second_face].surface_index],tol)==
                curve_surface_intersection::ParameterMatch::Matched);
            const auto p=section.world.point_at_start(),q=section.world.point_at_end();
            positive=positive || (std::abs(p[1]-q[1])>=h && std::abs(p[1]-q[1])<=4*h);
        }
        MINI_CHECK(positive);
    }
    MINI_CHECK(a.jsondump()==before_a && b.jsondump()==before_b);
}

MINI_TEST("BRep Native Section", "Tangential Line Remains In Shared Inventory") {
    // Exact polynomial supports z=x*x and z=0 meet along x=z=0, -1<=y<=1.
    // A contact need not bound retained material, but is still a section locus.
    const double tol=1e-7;
    NurbsSurface curved(3,false,3,2,3,2);
    curved.m_nurbsknot[0]={0,0,1,1};curved.m_nurbsknot[1]={0,1};
    for (int i=0;i<3;++i) for (int j=0;j<2;++j)
        MINI_CHECK(curved.set_cv(i,j,{double(i-1),double(2*j-1),i==1?-1.:1.}));
    const auto a=inventory_uv_rectangle(curved,0,1,0,1);
    const auto b=inventory_exact_face({Point(-1,-1,0),Point(1,-1,0),Point(1,1,0),Point(-1,1,0)});
    const auto before_a=a.jsondump(),before_b=b.jsondump();
    for (bool swap:{false,true}) {
        const auto result=brep_section::interferences(swap?b:a,swap?a:b,1e-7);
        if (!result.complete) throw std::runtime_error(result.failure);
        bool line=false;
        for (const auto& section:result.curves) {
            MINI_CHECK(section.world.domain()==section.first_pcurve.domain());
            MINI_CHECK(section.world.domain()==section.second_pcurve.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world,section.first_pcurve,
                (swap?b:a).m_surfaces[(swap?b:a).m_faces[section.first_face].surface_index],tol)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(section.world,section.second_pcurve,
                (swap?a:b).m_surfaces[(swap?a:b).m_faces[section.second_face].surface_index],tol)==
                curve_surface_intersection::ParameterMatch::Matched);
            const auto p=section.world.point_at_start(),q=section.world.point_at_end();
            line=line || (std::abs(p[1]-q[1])>1 && std::abs(p[0])<1e-7 && std::abs(q[0])<1e-7 && std::abs(p[2])<1e-7 && std::abs(q[2])<1e-7);
        }
        MINI_CHECK(line);
    }
    MINI_CHECK(a.jsondump()==before_a && b.jsondump()==before_b);
}

namespace session_cpp {
MINI_TEST("BRep Native Section", "Exact Original Linear Event Keeps Projective And Raw Identity") {
    const double tolerance=std::ldexp(1.,-10),x=15*tolerance/64;
    const auto a=NurbsCurve::create(false,1,{{-1,0,0},{1,0,0}});
    const auto b=NurbsCurve::create(false,1,{{x,0,-1},{x,0,1}});
    const auto event=brep_section::detail::exact_linear_crossing(a,b);
    MINI_CHECK(event.has_value());
    if (!event) return;
    MINI_CHECK(nurbs_coincidence::exact_products_equal(x,event->weight,event->homogeneous[0],1.));
    MINI_CHECK(event->homogeneous[1]==0 && event->homogeneous[2]==0);
    MINI_CHECK((event->center==Point(x,0,0)));
    for (bool reverse:{false,true}) {
        auto world=NurbsCurve::create(false,1,{{x,-1,0},{x,1,0}});
        MINI_CHECK(world.set_domain(-7,13));
        if (reverse) MINI_CHECK(world.reverse());
        const auto frozen=world.jsondump();
        MINI_CHECK(brep_section::detail::exact_event_on_raw_interval(*event,world,2,4));
        MINI_CHECK(!brep_section::detail::exact_event_on_raw_interval(*event,world,-7,2));
        MINI_CHECK(!brep_section::detail::exact_event_on_raw_interval(*event,world,4,13));
        MINI_CHECK(world.jsondump()==frozen);
        auto nearby=world;
        for (int i=0;i<2;++i) {auto point=nearby.get_cv(i);point[0]+=tolerance/8;MINI_CHECK(nearby.set_cv(i,point));}
        MINI_CHECK(!brep_section::detail::exact_event_on_raw_interval(*event,nearby,2,4));
    }
    const double original_parameter=1+x;
    MINI_CHECK(brep_section::detail::exact_event_at_raw_parameter(*event,a,original_parameter));
    const double other_parameter=std::nextafter(original_parameter,2.);
    MINI_CHECK(a.point_at(other_parameter).distance(event->center)<tolerance);
    MINI_CHECK(!brep_section::detail::exact_event_at_raw_parameter(*event,a,other_parameter));
    auto rounded_difference=NurbsCurve::create(false,1,{{x,-.02,0},{x,1.02,0}});
    MINI_CHECK(rounded_difference.set_domain(0,1));
    MINI_CHECK(brep_section::detail::exact_axis_event_on_raw_interval(*event,rounded_difference,0,.5));
    MINI_CHECK(!brep_section::detail::exact_axis_event_on_raw_interval(*event,rounded_difference,.5,1));
    MINI_CHECK(rounded_difference.reverse());
    MINI_CHECK(brep_section::detail::exact_axis_event_on_raw_interval(*event,rounded_difference,.5,1));
    auto skew=b;
    for (int i=0;i<2;++i) {auto point=skew.get_cv(i);point[1]=tolerance/8;MINI_CHECK(skew.set_cv(i,point));}
    MINI_CHECK(!brep_section::detail::exact_linear_crossing(a,skew));
    const auto separate=NurbsCurve::create(false,1,{{x+tolerance/8,0,-1},{x+tolerance/8,0,1}});
    const auto other=brep_section::detail::exact_linear_crossing(a,separate);
    MINI_CHECK(other.has_value());
    if (other) MINI_CHECK(!nurbs_coincidence::exact_products_equal(event->homogeneous[0],other->weight,
        other->homogeneous[0],event->weight));
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Exact Original Raw Singleton Keeps Rotated Rational Law") {
    NurbsCurve original(3,true,2,2);
    original.m_nurbsknot={-7,13};
    MINI_CHECK(original.set_cv_4d(0,-2,-4,0,2));
    MINI_CHECK(original.set_cv_4d(1,3,6,0,3));
    const auto cross=NurbsCurve::create(false,1,{{0,0,-1},{0,0,1}});
    const auto exact=brep_section::detail::exact_linear_crossing(original,cross);
    MINI_CHECK(exact.has_value());
    if (!exact) return;
    const auto frozen=original.jsondump();
    for (bool reverse:{false,true}) {
        auto curve=original;
        if (reverse) MINI_CHECK(curve.reverse());
        const double parameter=reverse?5:1;
        MINI_CHECK(brep_section::detail::exact_event_at_raw_parameter(*exact,curve,parameter));
        const double nearby=std::nextafter(parameter,13.);
        MINI_CHECK(nearby!=parameter && curve.get_cv(0).distance(curve.get_cv(1))>0);
        MINI_CHECK(!brep_section::detail::exact_event_at_raw_parameter(*exact,curve,nearby));
    }
    MINI_CHECK(original.jsondump()==frozen);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Endpoint Preimages Keep Original Rational Meridian Knots") {
    NurbsCurve curve(3,true,3,5);
    curve.m_nurbsknot={-7,-7,3,3,13,13};
    const double weight=std::sqrt(.5);
    MINI_CHECK(curve.set_cv_4d(0,0,0,2,1));
    MINI_CHECK(curve.set_cv_4d(1,2*weight,0,2*weight,weight));
    MINI_CHECK(curve.set_cv_4d(2,2,0,0,1));
    MINI_CHECK(curve.set_cv_4d(3,2*weight,0,-2*weight,weight));
    MINI_CHECK(curve.set_cv_4d(4,0,0,-2,1));
    MINI_CHECK(curve.is_valid());
    MINI_CHECK(brep_section::detail::exact_raw_clamped(curve));
    const std::array<double,4> north={0,0,2,1},south={0,0,-2,1};
    const auto frozen=curve.jsondump();
    for (bool reverse:{false,true}) {
        auto original=curve;
        if (reverse) MINI_CHECK(original.reverse());
        for (bool shifted:{false,true}) {
            auto raw=original;
            if (shifted) MINI_CHECK(raw.set_domain(32,35));
            MINI_CHECK(brep_section::detail::endpoint_preimages_only(raw,north));
            MINI_CHECK(brep_section::detail::endpoint_preimages_only(raw,south));
        }
    }
    MINI_CHECK(curve.jsondump()==frozen);
    NurbsCurve nonbinary(3,true,3,3);
    nonbinary.m_nurbsknot={-7,-7,13,13};
    MINI_CHECK(nonbinary.set_cv_4d(0,1,-2,4,3));
    MINI_CHECK(nonbinary.set_cv_4d(1,2,0,1,2));
    MINI_CHECK(nonbinary.set_cv_4d(2,4,0,0,5));
    const std::array<double,4> exact={1,-2,4,3};
    MINI_CHECK(brep_section::detail::endpoint_preimages_only(nonbinary,exact));
}

MINI_TEST("BRep Native Section", "Endpoint Preimages Reject Returning And Constant Raw Occurrences") {
    const std::array<double,4> zero={0,0,0,1};
    NurbsCurve returning(3,false,2,4);
    returning.m_nurbsknot={0,1,2,3};
    for (int i=0;i<4;++i) MINI_CHECK(returning.set_cv(i,{i==1?1.:i==3?2.:0.,0,0}));
    MINI_CHECK(returning.is_valid() && returning.point_at(2).distance({0,0,0})==0);
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(returning,zero));
    NurbsCurve interior(3,false,4,4);
    interior.m_nurbsknot={0,0,0,1,1,1};
    const std::array<double,4> x={0,1,-2,3};
    for (int i=0;i<4;++i) MINI_CHECK(interior.set_cv(i,{x[i],0,0}));
    MINI_CHECK(interior.point_at(.5).distance({0,0,0})==0);
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(interior,zero));
    NurbsCurve constant(3,true,3,3);
    constant.m_nurbsknot={0,0,1,1};
    MINI_CHECK(constant.set_cv_4d(0,1,-2,4,3));
    MINI_CHECK(constant.set_cv_4d(1,2,-4,8,6));
    MINI_CHECK(constant.set_cv_4d(2,4,-8,16,12));
    const std::array<double,4> exact={1,-2,4,3};
    const auto frozen=constant.jsondump();
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(constant,exact));
    auto bulge=constant;
    MINI_CHECK(bulge.set_cv_4d(1,2,std::nextafter(-4.,-3.),8,6));
    MINI_CHECK(brep_section::detail::endpoint_preimages_only(bulge,exact));
    MINI_CHECK(constant.jsondump()==frozen);
}

MINI_TEST("BRep Native Section", "Endpoint Preimages Require Exact Clamping And Continuous Positive Basis") {
    const std::array<double,4> zero={0,0,0,1};
    NurbsCurve almost(3,false,3,3);
    almost.m_nurbsknot={0,std::ldexp(1.,-45),1,1};
    MINI_CHECK(almost.set_cv(0,{0,0,0}));
    MINI_CHECK(almost.set_cv(1,{1,0,0}));
    MINI_CHECK(almost.set_cv(2,{2,0,0}));
    MINI_CHECK(almost.is_valid() && almost.m_nurbsknot[0]!=almost.m_nurbsknot[1]);
    MINI_CHECK(!brep_section::detail::exact_raw_clamped(almost));
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(almost,zero));
    NurbsCurve jump(3,false,2,4);
    jump.m_nurbsknot={0,1,1,2};
    for (int i=0;i<4;++i) MINI_CHECK(jump.set_cv(i,{double(i),0,0}));
    MINI_CHECK(jump.is_valid());
    MINI_CHECK(brep_section::detail::exact_raw_clamped(jump));
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(jump,zero));
    NurbsCurve tiny(3,false,2,3);
    tiny.m_nurbsknot={0,std::nextafter(0.,1.),1};
    for (int i=0;i<3;++i) MINI_CHECK(tiny.set_cv(i,{double(i),0,0}));
    MINI_CHECK(tiny.is_valid());
    MINI_CHECK(brep_section::detail::endpoint_preimages_only(tiny,zero));
    auto negative=tiny;
    MINI_CHECK(negative.to_rational());
    negative.cv(1)[3]=-1;
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(negative,zero));
}
} // namespace session_cpp

namespace session_cpp {
namespace {
std::vector<int> point_use_loop(BRep& owner,const std::vector<Point>& points) {
    std::vector<int> vertices,edges;
    for (const auto& point:points) vertices.push_back(owner.add_vertex(point,1e-7));
    std::vector<BRepRef> references;
    for (size_t i=0;i<points.size();++i) {
        const size_t next=(i+1)%points.size();
        auto world=NurbsCurve::create(false,1,{points[i],points[next]});
        MINI_CHECK(world.set_domain(-7,13));
        NurbsCurve pc(3,false,2,2);pc.m_nurbsknot={32,35};
        MINI_CHECK(pc.set_cv(0,{(points[i][0]+1)*.5,(points[i][1]+1)*.5,0}));
        MINI_CHECK(pc.set_cv(1,{(points[next][0]+1)*.5,(points[next][1]+1)*.5,0}));
        const int edge=owner.add_edge(owner.add_curve_3d(world),vertices[i],vertices[next],1e-7);
        owner.add_pcurve(edge,0,owner.add_curve_2d(pc));
        MINI_CHECK(curve_surface_intersection::same_parameter(world,pc,owner.m_surfaces[0],1e-7)==
            curve_surface_intersection::ParameterMatch::Matched);
        references.push_back({edge,BRepOrientation::Forward});edges.push_back(edge);
    }
    owner.m_faces[0].wires.push_back({owner.add_wire(references),BRepOrientation::Reversed});
    return edges;
}
}
MINI_TEST("BRep Native Section", "Point Ownership Visits Later Original Interior Boundary Uses") {
    auto plane=inventory_exact_face({Point(-1,-1,0),Point(1,-1,0),Point(1,1,0),Point(-1,1,0)});
    // Diagnostic trim: two hole boundaries touch at P. This asserts incidence
    // preservation, not regular-solid/material acceptance for the touching holes.
    const auto first=point_use_loop(plane,{{0,0,0},{-.25,-.25,0},{0,-.5,0},{.25,-.25,0}});
    const auto second=point_use_loop(plane,{{-.25,0,0},{.25,0,0},{.25,.5,0},{-.25,.5,0}});
    auto sphere=BRep::create_sphere(1);sphere.transform(Xform::translation(0,0,1));
    MINI_CHECK(plane.is_valid() && sphere.is_valid());
    const auto frozen_p=plane.jsondump(),frozen_s=sphere.jsondump();
    for (bool swap:{false,true}) {
        const auto result=swap?brep_section::interferences(sphere,plane,1e-7):
                               brep_section::interferences(plane,sphere,1e-7);
        if (!result.complete) throw std::runtime_error(result.failure);
        bool endpoint_a=false,endpoint_b=false,interior=false;
        for (const auto& event:result.boundary_events) {
            if (event.operand!=static_cast<int>(swap)) continue;
            if (event.edge==first.front()) endpoint_a=endpoint_a || event.hit.t==-7;
            if (event.edge==first.back()) endpoint_b=endpoint_b || event.hit.t==13;
            if (event.edge==second.front()) {
                MINI_CHECK(event.vertex==-1 && event.hit.t==3);
                MINI_CHECK(plane.m_curves_3d[plane.m_edges[event.edge].curve_3d_index].point_at(event.hit.t)==Point(0,0,0));
                MINI_CHECK(!event.uses.empty());
                interior=true;
            }
        }
        MINI_CHECK(endpoint_a && endpoint_b && interior);
    }
    MINI_CHECK(plane.jsondump()==frozen_p && sphere.jsondump()==frozen_s);
}

MINI_TEST("BRep Native Section", "Original Linear Point Uses Keep Variable Weights And Raw Domains") {
    auto first=BRep::create_box(2,2,2);
    for (auto& curve:first.m_curves_3d) {
        MINI_CHECK(curve.set_domain(-7,13));
        for (int i=0;i<curve.cv_count();++i) {
            const auto point=curve.get_cv(i);const double weight=i?3:2;
            MINI_CHECK(curve.set_cv_4d(i,point[0]*weight,point[1]*weight,point[2]*weight,weight));
        }
    }
    for (auto& curve:first.m_curves_2d) {
        MINI_CHECK(curve.set_domain(32,35));
        for (int i=0;i<curve.cv_count();++i) {
            const auto point=curve.get_cv(i);const double weight=i?3:2;
            MINI_CHECK(curve.set_cv_4d(i,point[0]*weight,point[1]*weight,point[2]*weight,weight));
        }
    }
    const auto second=BRep::create_box(2,2,2).transformed(Xform::translation(2,2,2));
    const auto frozen_a=first.jsondump(),frozen_b=second.jsondump();
    for (bool swap:{false,true}) {
        const auto result=swap?brep_section::interferences(second,first,1e-7):
                               brep_section::interferences(first,second,1e-7);
        if (!result.complete) throw std::runtime_error(result.failure);
        MINI_CHECK(result.nodes.size()==1 && result.curves.empty());
        for (size_t edge=0;edge<first.m_edges.size();++edge) {
            const auto& source=first.m_edges[edge];
            const auto& world=first.m_curves_3d[source.curve_3d_index];
            for (bool end:{false,true}) {
                const Point point=end?world.point_at_end():world.point_at_start();
                if (point!=Point(1,1,1)) continue;
                bool found=false;
                for (const auto& event:result.boundary_events)
                    if (event.operand==static_cast<int>(swap) && event.edge==static_cast<int>(edge) &&
                        event.hit.t==(end?13:-7) && event.vertex==(end?source.end_vertex:source.start_vertex)) found=true;
                MINI_CHECK(found);
            }
        }
    }
    MINI_CHECK(first.jsondump()==frozen_a && second.jsondump()==frozen_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Sole Interior Boundary Contact Binds Existing Support Node") {
    auto plane=inventory_exact_face({Point(-1,0,0),Point(1,0,0),Point(1,1,0),Point(-1,1,0)});
    for (auto& world:plane.m_curves_3d) MINI_CHECK(world.set_domain(-7,13));
    for (auto& pc:plane.m_curves_2d) MINI_CHECK(pc.set_domain(32,35));
    auto sphere=BRep::create_sphere(1);sphere.transform(Xform::translation(0,0,1));
    const auto frozen_p=plane.jsondump(),frozen_s=sphere.jsondump();
    for (bool swap:{false,true}) {
        const auto result=swap?brep_section::interferences(sphere,plane,1e-7):
                               brep_section::interferences(plane,sphere,1e-7);
        if (!result.complete) throw std::runtime_error(result.failure);
        MINI_CHECK(result.nodes.size()==1 && result.curves.empty());
        bool interior=false;
        for (const auto& node:result.nodes) {
            MINI_CHECK(!node.support_points.empty() && node.point==Point(0,0,0));
            for (size_t index:node.boundary_events) {
                const auto& event=result.boundary_events[index];
                if (event.operand==static_cast<int>(swap)) {
                    MINI_CHECK(event.edge==0 && event.vertex==-1 && event.hit.t==3);
                    MINI_CHECK(!event.uses.empty() && event.uses.front().face==0);
                    interior=true;
                }
            }
        }
        MINI_CHECK(interior);
        // No original plane vertex bridges P: this is a true interior-only owner.
        for (const auto& vertex:plane.m_vertices) MINI_CHECK(vertex.point!=Point(0,0,0));
    }
    MINI_CHECK(plane.jsondump()==frozen_p && sphere.jsondump()==frozen_s);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Nearby Exact Support Points Keep Their Own Raw Incidences") {
    const double tolerance=std::ldexp(1.,-10),delta=tolerance/8;
    auto plane=inventory_exact_face({Point(-1,0,0),Point(1,0,0),Point(1,1,0),Point(-1,1,0)});
    for (auto& world:plane.m_curves_3d) MINI_CHECK(world.set_domain(-7,13));
    for (auto& pc:plane.m_curves_2d) MINI_CHECK(pc.set_domain(32,35));
    for (bool reverse_faces:{false,true}) {
        const auto left=BRep::create_sphere(1).transformed(Xform::translation(0,0,1));
        const auto right=BRep::create_sphere(1).transformed(Xform::translation(delta,0,1));
        // Independent source-face compound for interference ownership only.
        // The overlapping source sphere solids are not a regular Boolean operand claim.
        const auto contacts=reverse_faces?inventory_compound(right,left):inventory_compound(left,right);
        const auto frozen_p=plane.jsondump(),frozen_c=contacts.jsondump();
        for (bool swap:{false,true}) {
            const auto result=swap?brep_section::interferences(contacts,plane,tolerance):
                                   brep_section::interferences(plane,contacts,tolerance);
            if (!result.complete) throw std::runtime_error(result.failure);
            MINI_CHECK(result.nodes.size()==2 && result.curves.empty());
            MINI_CHECK(result.coverage.size()==2 && result.support_points.size()==2);
            for (const auto& coverage:result.coverage)
                MINI_CHECK(coverage.support==surface_section::Coverage::GlobalContactPoint);
            bool first=false,second=false;
            for (const auto& node:result.nodes) {
                const bool is_first=node.point[0]==0 && node.point[1]==0 && node.point[2]==0;
                const bool is_second=node.point[0]==delta && node.point[1]==0 && node.point[2]==0;
                MINI_CHECK(is_first!=is_second && !node.support_points.empty());
                first=first||is_first;second=second||is_second;
                const double raw=3+10*(is_second?delta:0);
                const int other_face=(is_second!=reverse_faces)?1:0;
                for (size_t index:node.support_points) {
                    const auto& witness=result.support_points[index].event.exact_witness;
                    for (int d=0;d<3;++d)
                        MINI_CHECK(nurbs_coincidence::exact_products_equal(node.point[d],witness[3],witness[d],1.));
                }
                int plane_uses=0;
                for (size_t index:node.boundary_events) {
                    const auto& event=result.boundary_events[index];
                    if (event.operand!=static_cast<int>(swap)) continue;
                    MINI_CHECK(event.edge==0 && event.vertex==-1 && event.hit.t==raw);
                    MINI_CHECK(event.hit.face==other_face);
                    for (int d=0;d<3;++d) MINI_CHECK(event.hit.point[d]==node.point[d]);
                    MINI_CHECK(std::any_of(event.uses.begin(),event.uses.end(),[](const auto& use){return use.face==0;}));
                    ++plane_uses;
                }
                MINI_CHECK(plane_uses==1);
            }
            MINI_CHECK(first && second && 3+10*delta!=3);
        }
        MINI_CHECK(plane.jsondump()==frozen_p && contacts.jsondump()==frozen_c);
    }
}
} // namespace session_cpp

namespace session_cpp {
namespace {
NurbsCurve polygon_qualification_line(const Point& a, const Point& b, double lo, double hi, double weight) {
    NurbsCurve result(3, true, 2, 2);
    result.m_nurbsknot = {lo, hi};
    MINI_CHECK(result.set_cv_4d(0, a[0] * weight, a[1] * weight, a[2] * weight, weight));
    MINI_CHECK(result.set_cv_4d(1, b[0] * weight, b[1] * weight, b[2] * weight, weight));
    return result;
}
int polygon_qualification_wire(BRep& owner, const std::vector<Point>& points, bool reverse) {
    const auto& support = owner.m_surfaces[0];
    std::vector<int> vertices;
    for (const auto& uv : points)
        vertices.push_back(owner.add_vertex(support.point_at(uv[0], uv[1]), 1e-7));
    std::vector<BRepRef> uses;
    for (size_t i = 0; i < points.size(); ++i) {
        const size_t next = (i + 1) % points.size();
        const auto world = polygon_qualification_line(owner.m_vertices[vertices[i]].point,
                                                      owner.m_vertices[vertices[next]].point, -7, 13, 2);
        const auto uv = polygon_qualification_line(points[i], points[next], 32, 35, 3);
        const int edge = owner.add_edge(owner.add_curve_3d(world), vertices[i], vertices[next], 1e-7);
        owner.add_pcurve(edge, 0, owner.add_curve_2d(uv));
        uses.push_back({edge, BRepOrientation::Forward});
    }
    const int index = owner.add_wire(uses);
    owner.m_faces[0].wires.push_back({index, reverse ? BRepOrientation::Reversed : BRepOrientation::Forward});
    return index;
}
BRep polygon_qualification_face(bool vertical, const std::vector<std::vector<Point>>& loops,
                                bool reverse = false) {
    BRep owner;
    NurbsSurface support(3, true, 2, 2, 2, 2);
    support.m_nurbsknot[0] = support.m_nurbsknot[1] = {0, 1};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            const Point point =
                vertical ? Point(-2 + 4 * i, 0, -2 + 4 * j) : Point(-2 + 4 * i, -2 + 4 * j, 0);
            MINI_CHECK(support.set_cv_4d(i, j, point[0] * 5, point[1] * 5, point[2] * 5, 5));
        }
    owner.add_surface(support);
    owner.add_face(0, {}, 1e-7);
    for (const auto& loop : loops)
        polygon_qualification_wire(owner, loop, reverse);
    MINI_CHECK(owner.is_valid());
    return owner;
}
std::vector<Point> polygon_qualification_rectangle(double lo, double hi, double bottom = 0, double top = 1) {
    return {{lo, bottom, 0}, {hi, bottom, 0}, {hi, top, 0}, {lo, top, 0}};
}
brep_section::FaceResult polygon_qualification_inventory(const BRep& a, const BRep& b,const char* stage="polygon") {
    auto inventory = brep_section::interferences(a, b, 1e-7);
    if (!inventory.complete)
        throw std::runtime_error(inventory.failure);
    MINI_CHECK(inventory.coverage.size() == 1 && inventory.original_trim_laws.size() == 1);
    MINI_CHECK(inventory.coverage[0].support == surface_section::Coverage::OriginalDomainCurve);
    if (!inventory.coverage[0].trim_ownership_complete)
        throw std::runtime_error(std::string("Exact polygon trim ownership failed at ")+stage);
    MINI_CHECK(inventory.coverage[0].trim_ownership_complete);
    MINI_CHECK(inventory.original_trim_laws[0].trim_ownership_complete);
    MINI_CHECK(inventory.coverage_complete());
    return inventory;
}
} // namespace

MINI_TEST("BRep Native Section", "Exact Polygon Trim Coverage Keeps Holes And Original Raw Uses") {
    const auto outer = polygon_qualification_rectangle(.125, .875);
    const auto hole = polygon_qualification_rectangle(.375, .625, .25, .75);
    const auto whole = polygon_qualification_rectangle(0, 1);
    for (bool reverse : {false, true})
        for (bool swap : {false, true}) {
            const auto a = polygon_qualification_face(false, {outer, hole}, reverse);
            const auto b = polygon_qualification_face(true, {whole}, !reverse);
            const auto frozen_a = a.jsondump(), frozen_b = b.jsondump();
            const auto inventory =
                swap ? polygon_qualification_inventory(b, a) : polygon_qualification_inventory(a, b);
            const auto& law = inventory.original_trim_laws[0];
            // Exact x acceptance: [-1.5,-.5] union [.5,1.5], with no hole reinsertion.
            MINI_CHECK(law.retained_pieces.size() == 2 && law.partition.size() == 6);
            for (const auto& piece : law.retained_pieces) {
                const auto& world = inventory.curves[piece.curve].world;
                const double x = world.point_at(std::midpoint(piece.first, piece.last))[0];
                MINI_CHECK((x > -1.5 && x < -.5) || (x > .5 && x < 1.5));
            }
            MINI_CHECK(a.jsondump() == frozen_a && b.jsondump() == frozen_b);
        }
}

MINI_TEST("BRep Native Section", "Exact Polygon Trim Coverage Retains Collinear And Tangent Contacts") {
    const auto whole = polygon_qualification_rectangle(0, 1);
    const auto b = polygon_qualification_face(true, {whole});
    const auto nested =
        polygon_qualification_face(false, {whole, polygon_qualification_rectangle(.25, .75, .25, .75),
                                           polygon_qualification_rectangle(.375, .625, .375, .625)});
    const auto nested_result = polygon_qualification_inventory(nested, b,"nested holes");
    MINI_CHECK(nested_result.original_trim_laws[0].retained_pieces.size() == 2);
    const auto disjoint = polygon_qualification_face(false, {polygon_qualification_rectangle(0, 1, .75, 1)});
    const auto empty = polygon_qualification_inventory(disjoint, b,"trimmed empty");
    MINI_CHECK(empty.curves.empty() && empty.nodes.empty());
    const auto collinear =
        polygon_qualification_face(false, {whole, polygon_qualification_rectangle(.25, .75, .5, .75)});
    const auto first = polygon_qualification_inventory(collinear, b,"collinear boundary");
    MINI_CHECK(first.original_trim_laws[0].retained_pieces.size() == 3);
    MINI_CHECK(first.original_trim_laws[0].owned_intervals.empty());
    const std::vector<Point> triangle = {{.5, .5, 0}, {.75, .75, 0}, {.25, .75, 0}};
    const auto tangent = polygon_qualification_face(false, {whole, triangle});
    const auto second = polygon_qualification_inventory(tangent, b,"tangent vertex");
    MINI_CHECK(second.original_trim_laws[0].retained_pieces.size() == 2);
    const auto contact = polygon_qualification_face(false, {triangle});
    const auto isolated = polygon_qualification_inventory(contact, b,"isolated vertex");
    MINI_CHECK(isolated.curves.empty() && isolated.nodes.size() == 1 && isolated.nodes[0].isolated);
    MINI_CHECK(isolated.nodes[0].point[0] == 0 && isolated.nodes[0].point[1] == 0 &&
               isolated.nodes[0].point[2] == 0);
}

MINI_TEST("BRep Native Section",
          "Exact Polygon Trim Coverage Rejects Missing Extra And Wrong Piece Receipts") {
    const auto whole = polygon_qualification_rectangle(0, 1);
    const auto a =
        polygon_qualification_face(false, {whole, polygon_qualification_rectangle(.25, .75, .25, .75)});
    const auto b = polygon_qualification_face(true, {whole});
    const auto original = polygon_qualification_inventory(a, b);
    MINI_CHECK(
        brep_section::detail::polygon_trim_coverage(original, original.original_trim_laws[0], a, b, 1e-7));
    for (int variant = 0; variant < 7; ++variant) {
        auto changed = original;
        auto& law = changed.original_trim_laws[0];
        if (variant == 0)
            law.events.pop_back();
        if (variant == 1)
            law.events.push_back(law.events.front());
        if (variant == 2)
            law.partition.erase(law.partition.begin() + 1);
        if (variant == 3)
            law.retained_pieces.pop_back();
        if (variant == 4)
            law.retained_pieces[0].last = law.partition.back();
        if (variant == 5)
            changed.nodes.clear();
        if (variant == 6)
            law.owned_intervals.push_back({});
        MINI_CHECK(!brep_section::detail::polygon_trim_coverage(changed, law, a, b, 1e-7));
    }
    auto weighted = a;
    const auto pole = weighted.m_surfaces[0].get_cv(0, 0);
    MINI_CHECK(weighted.m_surfaces[0].set_cv_4d(0, 0, pole[0] * 3, pole[1] * 3, pole[2] * 3, 3));
    MINI_CHECK(!brep_section::detail::polygon_trim_coverage(original, original.original_trim_laws[0],
                                                            weighted, b, 1e-7));
    auto gap = a;
    auto& uv = gap.m_curves_2d[0];
    auto point = uv.get_cv(1);
    point[1] = std::nextafter(point[1], 1.);
    MINI_CHECK(uv.set_cv(1, point));
    MINI_CHECK(
        !brep_section::detail::polygon_trim_coverage(original, original.original_trim_laws[0], gap, b, 1e-7));
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Exact Polygon Trim Coverage Declines Nonrepresentable Root") {
    const auto line = polygon_qualification_line({0, .5, 0}, {1, .5, 0}, -7, 13, 2);
    const auto edge = polygon_qualification_line({0, 0, 0}, {1, 1.5, 0}, 32, 35, 3);
    brep_section::detail::PolygonTrimEdge use;
    use.uv = &edge;
    MINI_CHECK(brep_section::detail::polygon_linear(edge, use.ends));
    brep_section::detail::PolygonTrimProof proof;
    // Exact root x=1/3 is not binary64; whole ownership remains explicitly unqualified.
    MINI_CHECK(!brep_section::detail::polygon_edge_events(proof, use, line));
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Original Rational Meridian Contact Keeps Its Interior Knot And Seam Uses") {
    const auto first=BRep::create_sphere(1);
    const auto second=first.transformed(Xform::translation(2,0,0));
    const auto frozen_a=first.jsondump(),frozen_b=second.jsondump();
    const std::array<double,4> witness{1,0,0,1};
    const auto& meridian=first.m_curves_3d[first.m_edges[0].curve_3d_index];
    std::vector<double> raw;
    MINI_CHECK(brep_section::detail::original_knot_preimages_only(meridian,witness,raw));
    MINI_CHECK(raw==std::vector<double>{1});
    MINI_CHECK(!brep_section::detail::endpoint_preimages_only(meridian,witness));
    MINI_CHECK(!brep_section::detail::original_knot_is_preimage(meridian,witness,std::nextafter(1.,2.)));
    for (bool swap:{false,true}) {
        const auto result=swap?brep_section::interferences(second,first,1e-7):
                               brep_section::interferences(first,second,1e-7);
        if (!result.complete) throw std::runtime_error(result.failure);
        MINI_CHECK(result.curves.empty() && result.support_points.size()==1 && result.nodes.size()==1);
        MINI_CHECK(result.coverage.size()==1 && result.coverage[0].support==surface_section::Coverage::GlobalContactPoint);
        MINI_CHECK(result.nodes[0].support_points.size()==1 && result.nodes[0].boundary_events.size()==1);
        const auto& event=result.boundary_events[result.nodes[0].boundary_events[0]];
        MINI_CHECK(event.operand==static_cast<int>(swap) && event.edge==0 && event.vertex==-1 && event.hit.t==1);
        MINI_CHECK(event.uses.size()==2 && result.uv_only_incidences.empty());
    }
    MINI_CHECK(first.jsondump()==frozen_a && second.jsondump()==frozen_b);
}
MINI_TEST("BRep Native Section", "Original Knot Preimages Retain Returning Occurrences And Decline Unproved Roots") {
    const std::array<double,4> point{0,0,0,3};
    NurbsCurve returning(3,true,2,5);
    returning.m_nurbsknot={-7,-2,3,8,13};
    const std::array<double,5> values{-1,0,1,0,-1},weights{2,3,5,3,2};
    for (int i=0;i<5;++i)
        MINI_CHECK(returning.set_cv_4d(i,values[i]*weights[i],0,0,weights[i]));
    const auto frozen=returning.jsondump();
    for (bool reverse:{false,true}) {
        auto curve=returning;
        if (reverse) MINI_CHECK(curve.reverse());
        std::vector<double> raw;
        MINI_CHECK(brep_section::detail::original_knot_preimages_only(curve,point,raw));
        MINI_CHECK(raw==std::vector<double>({-2,8}));
        for (double t:raw) MINI_CHECK(brep_section::detail::original_knot_is_preimage(curve,point,t));
    }
    const auto crossing=NurbsCurve::create(false,1,{Point(-1,0,0),Point(1,0,0)});
    const auto constant=NurbsCurve::create(false,1,{Point(0,0,0),Point(0,0,0)});
    std::vector<double> raw{123};
    MINI_CHECK(!brep_section::detail::original_knot_preimages_only(crossing,point,raw) && raw.empty());
    MINI_CHECK(!brep_section::detail::original_knot_preimages_only(constant,point,raw) && raw.empty());
    NurbsCurve jump(3,false,2,4);jump.m_nurbsknot={0,1,1,2};
    for (int i=0;i<4;++i) MINI_CHECK(jump.set_cv(i,{i<2?-1.:1.,0,0}));
    MINI_CHECK(jump.is_valid());
    MINI_CHECK(!brep_section::detail::original_knot_preimages_only(jump,point,raw));
    MINI_CHECK(returning.jsondump()==frozen);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Own Trim Admission Retains Excluded Original Uses Without Contact Nodes") {
    const auto whole=polygon_qualification_rectangle(0,1);
    const auto own=polygon_qualification_face(false,{whole,
        polygon_qualification_rectangle(.25,.75,.25,.75),
        polygon_qualification_rectangle(.375,.625,.375,.625)});
    const auto target=polygon_qualification_face(true,{whole});
    const auto frozen_own=own.jsondump(),frozen_target=target.jsondump();
    for (bool swap:{false,true}) {
        const auto inventory=swap?polygon_qualification_inventory(target,own,"excluded source point swapped"):
                                  polygon_qualification_inventory(own,target,"excluded source point");
        size_t excluded=0;
        for (size_t index=0;index<inventory.boundary_events.size();++index) {
            const auto& event=inventory.boundary_events[index];
            if (event.retained) continue;
            MINI_CHECK(event.operand==static_cast<int>(swap));
            MINI_CHECK(event.source_parameters.size()==event.uses.size() && !event.uses.empty());
            for (size_t i=0;i<event.uses.size();++i) {
                const auto& use=event.uses[i];
                const auto& pc=own.m_curves_2d[use.pcurve];
                const double raw=event.source_parameters[i];
                MINI_CHECK(raw>=pc.domain_start() && raw<=pc.domain_end());
                MINI_CHECK(brep_classify::face_uv(own,use.face,pc.point_at(raw),
                    1e-7/nurbs_bounds::surface_metric(own.m_surfaces[own.m_faces[use.face].surface_index]))==
                    brep_classify::State::Outside);
            }
            for (const auto& node:inventory.nodes)
                MINI_CHECK(std::find(node.boundary_events.begin(),node.boundary_events.end(),index)==node.boundary_events.end());
            ++excluded;
        }
        MINI_CHECK(excluded>=2);
        // A nested-hole border lying ON the other support is a positive raw
        // boundary overlap, yet is wholly outside its OWN face material.
        const auto shifted=target.transformed(Xform::translation(0,-.5,0));
        const auto span_inventory=swap?polygon_qualification_inventory(shifted,own,"excluded source span swapped"):
                                       polygon_qualification_inventory(own,shifted,"excluded source span");
        bool excluded_span=false;
        for (const auto& span:span_inventory.excluded_boundary_spans)
            if (span.operand==static_cast<int>(swap) && span.own_outside) {
                MINI_CHECK(span.first<span.last && span.use.face==0);
                const auto& world=own.m_curves_3d[own.m_edges[span.edge].curve_3d_index];
                MINI_CHECK(span.first>=world.domain_start() && span.last<=world.domain_end());
                excluded_span=true;
            }
        MINI_CHECK(excluded_span);
        MINI_CHECK(span_inventory.original_trim_laws[0].retained_pieces.size()==2);
    }
    MINI_CHECK(own.jsondump()==frozen_own && target.jsondump()==frozen_target);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Own Trim Cuts Cannot Enlarge A Partial Original Boundary Overlap") {
    const auto whole=polygon_qualification_rectangle(0,1);
    const auto source=polygon_qualification_face(false,{whole,
        polygon_qualification_rectangle(.8125,.9375,0,.125)});
    const auto target=inventory_exact_face({Point(-1,-2,0),Point(1,-2,0),Point(1,0,0),Point(-1,0,0)});
    const auto frozen_a=source.jsondump(),frozen_b=target.jsondump();
    for (bool swap:{false,true}) {
        const auto inventory=swap?brep_section::interferences(target,source,1e-7):
                                  brep_section::interferences(source,target,1e-7);
        if (!inventory.complete) throw std::runtime_error(inventory.failure);
        MINI_CHECK(!inventory.curves.empty());
        for (const auto& curve:inventory.curves) {
            for (double t:{curve.world.domain_start(),std::midpoint(curve.world.domain_start(),curve.world.domain_end()),
                           curve.world.domain_end()}) {
                const auto point=curve.world.point_at(t);
                MINI_CHECK(point[0]>=-1 && point[0]<=1 && point[1]>=-2 && point[1]<=0 && point[2]==0);
            }
            const auto& first=swap?target:source;const auto& second=swap?source:target;
            MINI_CHECK(curve_surface_intersection::same_parameter(curve.world,curve.first_pcurve,
                first.m_surfaces[first.m_faces[curve.first_face].surface_index],1e-7)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(curve.world,curve.second_pcurve,
                second.m_surfaces[second.m_faces[curve.second_face].surface_index],1e-7)==
                curve_surface_intersection::ParameterMatch::Matched);
        }
    }
    MINI_CHECK(source.jsondump()==frozen_a && target.jsondump()==frozen_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Original Closed Endpoint Alias Uses Identity Not Near Raw Parameters") {
    for (bool shifted:{false,true}) for (bool reversed:{false,true}) {
        auto owner=BRep::create_cylinder(1,2);
        int index=-1;
        for (int i=0;i<owner.edge_count();++i)
            if (!owner.m_edges[i].degenerated && owner.m_edges[i].start_vertex==owner.m_edges[i].end_vertex)
                index=i;
        MINI_CHECK(index>=0);
        const auto& edge=owner.m_edges[index];
        auto& world=owner.m_curves_3d[edge.curve_3d_index];
        if (shifted) MINI_CHECK(world.set_domain(-7,13));
        if (reversed) MINI_CHECK(world.reverse());
        const Point point=owner.m_vertices[edge.start_vertex].point;
        brep_section::BoundaryEvent first,last;
        first.operand=last.operand=0;first.edge=last.edge=index;
        first.vertex=last.vertex=edge.start_vertex;
        first.hit.t=world.domain_start();last.hit.t=world.domain_end();
        first.uses.push_back({0,0,0,0});last.uses=first.uses;
        const auto frozen=owner.jsondump();
        MINI_CHECK(brep_section::detail::original_closed_endpoint_alias(first,last,owner,1e-7));
        MINI_CHECK(brep_section::detail::original_closed_endpoint_alias(last,first,owner,1e-7));
        const double near=first.hit.t+(last.hit.t-first.hit.t)*std::ldexp(1.,-30);
        MINI_CHECK(near>first.hit.t && world.point_at(near).distance(point)<2.5e-8);
        auto interior=first;interior.hit.t=near;
        MINI_CHECK(!brep_section::detail::original_closed_endpoint_alias(interior,last,owner,1e-7));
        auto other_operand=last;other_operand.operand=1;
        MINI_CHECK(!brep_section::detail::original_closed_endpoint_alias(first,other_operand,owner,1e-7));
        auto other_vertex=last;other_vertex.vertex=(edge.start_vertex+1)%owner.vertex_count();
        MINI_CHECK(other_vertex.vertex!=edge.start_vertex);
        MINI_CHECK(!brep_section::detail::original_closed_endpoint_alias(first,other_vertex,owner,1e-7));
        MINI_CHECK(owner.jsondump()==frozen);
    }
}
MINI_TEST("BRep Native Section", "Original Endpoint Alias Does Not Collapse Returning Interior Occurrences") {
    auto owner=BRep::create_cylinder(1,2);
    int index=-1;
    for (int i=0;i<owner.edge_count();++i)
        if (!owner.m_edges[i].degenerated && owner.m_edges[i].start_vertex==owner.m_edges[i].end_vertex)
            index=i;
    MINI_CHECK(index>=0);
    const auto edge=owner.m_edges[index];
    const Point p=owner.m_vertices[edge.start_vertex].point;
    NurbsCurve returning(3,false,2,5);returning.m_nurbsknot={0,1,2,3,4};
    const std::array<Point,5> points{p,p+Vector(0,2,0),p,p+Vector(0,-2,0),p};
    for (int i=0;i<5;++i) MINI_CHECK(returning.set_cv(i,points[i]));
    owner.m_curves_3d[edge.curve_3d_index]=returning;
    brep_section::BoundaryEvent a,b;
    a.operand=b.operand=0;a.edge=b.edge=index;a.vertex=b.vertex=edge.start_vertex;
    a.hit.t=0;b.hit.t=4;a.uses.push_back({0,0,0,0});b.uses=a.uses;
    MINI_CHECK(brep_section::detail::original_closed_endpoint_alias(a,b,owner,1e-7));
    auto interior=a;interior.hit.t=2;
    MINI_CHECK(!brep_section::detail::original_closed_endpoint_alias(interior,b,owner,1e-7));
    MINI_CHECK(returning.point_at(1).distance(p)>1);
}
} // namespace session_cpp


namespace session_cpp {
MINI_TEST("BRep Native Section", "Original Closed Edge Endpoint Aliases Keep Both Raw Occurrences") {
    auto owner=BRep::create_cylinder(1,2);
    int selected=-1,original_edge=-1;
    for (int f=0;f<owner.face_count() && selected<0;++f) {
        const auto& face=owner.m_faces[f];
        if (face.wires.size()!=1) continue;
        const auto uses=owner.wire_edges(face.wires[0]);
        if (uses.size()!=1) continue;
        const auto& edge=owner.m_edges[uses[0].index];
        if (!edge.degenerated && edge.start_vertex==edge.end_vertex) {
            selected=f;original_edge=uses[0].index;
        }
    }
    MINI_CHECK(selected>=0 && original_edge>=0);
    owner.m_faces={owner.m_faces[selected]};
    owner.m_solids.clear();owner.m_shells.clear();
    MINI_CHECK(owner.is_valid() && owner.face_count()==1 && owner.solid_count()==0);
    const auto& edge=owner.m_edges[original_edge];
    const auto& world=owner.m_curves_3d[edge.curve_3d_index];
    MINI_CHECK(brep_section::detail::exact_raw_clamped(world));
    MINI_CHECK(world.domain_start()<world.domain_end());
    const auto point=owner.m_vertices[edge.start_vertex].point;
    const std::array<double,4> witness{point[0],point[1],point[2],1};
    MINI_CHECK(brep_section::detail::endpoint_preimages_only(world,witness));
    MINI_CHECK(world.point_at(std::midpoint(world.domain_start(),world.domain_end())).distance(point)>1);
    for (double t:{world.domain_start(),world.domain_end()})
        MINI_CHECK(nurbs_interval::curve_within_ball(world,{t,t},{point[0],point[1],point[2]},2.5e-8));
    const BRep other=owner;
    const auto frozen=owner.jsondump(),frozen_other=other.jsondump();
    const auto result=brep_section::interferences(owner,other,1e-7);
    if (!result.complete) throw std::runtime_error(result.failure);
    for (int operand:{0,1}) {
        size_t first=std::numeric_limits<size_t>::max(),last=first;
        size_t first_count=0,last_count=0;
        for (size_t n=0;n<result.nodes.size();++n)
            for (size_t index:result.nodes[n].boundary_events) {
                const auto& event=result.boundary_events[index];
                if (event.operand!=operand || event.edge!=original_edge || event.vertex!=edge.start_vertex)
                    continue;
                if (event.hit.t==world.domain_start()) {
                    if (first_count) MINI_CHECK(first==n);
                    first=n;++first_count;
                }
                if (event.hit.t==world.domain_end()) {
                    if (last_count) MINI_CHECK(last==n);
                    last=n;++last_count;
                }
            }
        MINI_CHECK(first_count>0 && last_count>0 && first==last);
    }
    MINI_CHECK(owner.jsondump()==frozen && other.jsondump()==frozen_other);
}
} // namespace session_cpp


namespace session_cpp {
MINI_TEST("BRep Native Section", "Original Knot Endpoint Owns Positive Unmaterialized Raw Interval") {
    const auto root=std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root/"tests/fixtures/native_original_knot_interval_context.json");
    MINI_CHECK(input.good());const auto fixture=nlohmann::json::parse(input);
    const auto first=BRep::jsonload(fixture.at("first_brep"));
    const auto second=BRep::jsonload(fixture.at("second_brep"));
    const auto frozen_a=first.jsondump(),frozen_b=second.jsondump();
    const double lo=fixture.at("first"),hi=fixture.at("last"),tol=fixture.at("tolerance");
    MINI_CHECK(lo<hi);
    for (bool swap:{false,true}) {
        const auto inventory=swap?brep_section::interferences(second,first,tol):
                                  brep_section::interferences(first,second,tol);
        if (!inventory.complete) throw std::runtime_error(inventory.failure);
        // The original order has a top (+Z) singleton-knot cap. Swapping reverses
        // the section and exposes the bottom (-Z) original closed endpoint aliases.
        const double z=swap?-1.:1.;
        size_t owned=0;
        for (const auto& law:inventory.original_trim_laws) for (const auto& cap:law.owned_intervals) {
            if (cap.first!=lo || cap.last!=hi || cap.center[0]!=0 ||
                cap.center[1]!=.0009613037109375 || cap.center[2]!=z) continue;
            ++owned;
            MINI_CHECK(cap.first<cap.last && cap.events.size()==(swap?3:2));
            MINI_CHECK(cap.events.size()==cap.boundary_parameters.size() && cap.node<inventory.nodes.size());
            const auto& node=inventory.nodes[cap.node];
            std::vector<std::tuple<int,double,bool>> actual;
            for (size_t i=0;i<cap.events.size();++i) {
                const auto& event=law.events[cap.events[i]];
                MINI_CHECK(event.section_parameter>=cap.first && event.section_parameter<=cap.last);
                MINI_CHECK(cap.boundary_parameters[i].first==cap.boundary_parameters[i].second);
                MINI_CHECK(cap.boundary_parameters[i].first==event.boundary_parameter);
                actual.push_back({event.operand,event.boundary_parameter,event.overlap_endpoint});
                bool bound=false;
                for (size_t receipt:node.boundary_events) {
                    const auto& point=inventory.boundary_events[receipt];
                    if (!point.retained || point.operand!=event.operand || point.edge!=event.edge ||
                        point.hit.t!=cap.boundary_parameters[i].first) continue;
                    for (const auto& use:point.uses)
                        if (use.face==event.face && use.wire==event.wire && use.occurrence==event.occurrence &&
                            use.pcurve==event.pcurve) bound=true;
                }
                MINI_CHECK(bound);
            }
            std::vector<std::tuple<int,double,bool>> expected=swap?
                std::vector<std::tuple<int,double,bool>>{{0,1.5,true},{1,0,false},{1,5.998077392578125,true}}:
                std::vector<std::tuple<int,double,bool>>{{0,4.498558044433594,true},{1,3,true}};
            std::sort(actual.begin(),actual.end());std::sort(expected.begin(),expected.end());
            MINI_CHECK(actual==expected);
            if (swap) {
                const auto& edge=first.m_edges[1];
                const auto& world=first.m_curves_3d[edge.curve_3d_index];
                MINI_CHECK(edge.start_vertex==edge.end_vertex && world.domain_start()==0 &&
                           world.domain_end()==5.998077392578125);
                const auto& vertex=first.m_vertices[edge.start_vertex].point;
                for (int d=0;d<3;++d)
                    MINI_CHECK(nurbs_coincidence::exact_products_equal(vertex[d],cap.weight,cap.homogeneous[d],1.));
            }
            MINI_CHECK(nurbs_interval::curve_within_ball(law.original.world,{cap.first,cap.last},
                {node.point[0],node.point[1],node.point[2]},tol));
        }
        MINI_CHECK(owned==1);
    }
    MINI_CHECK(first.jsondump()==frozen_a && second.jsondump()==frozen_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("BRep Native Section", "Partial Original Uses Reject Pooled Section Fallback") {
    BRep owner;
    auto support=NurbsSurface::create(false,false,1,1,2,2,
        {Point(-2,-2,0),Point(-2,2,0),Point(2,-2,0),Point(2,2,0)});
    MINI_CHECK(support.set_domain(0,-2,2) && support.set_domain(1,-2,2));
    const int surface=owner.add_surface(support);
    const std::array<Point,4> corners{Point(0,0,0),Point(1,0,0),Point(1,1,0),Point(0,1,0)};
    std::array<int,4> vertices;
    for (size_t i=0;i<vertices.size();++i)
        vertices[i]=owner.add_vertex(corners[i]);
    std::vector<BRepRef> uses;
    for (size_t i=0;i<vertices.size();++i) {
        const auto direction=corners[(i+1)%4]-corners[i];
        auto source=NurbsCurve::create(false,1,
            {corners[i]-direction,corners[i]+direction*2});
        MINI_CHECK(source.set_domain(-1,2));
        const int edge=owner.add_edge(owner.add_curve_3d(source),vertices[i],vertices[(i+1)%4]);
        owner.m_edges[edge].world_range=BRepCurveUseRange{0,1};
        owner.add_pcurve(edge,surface,owner.add_curve_2d(source));
        owner.m_edges[edge].pcurves.back().primary_range=BRepCurveUseRange{0,1};
        uses.push_back({edge,BRepOrientation::Forward});
    }
    owner.add_face(surface,{{owner.add_wire(uses),BRepOrientation::Forward}});
    MINI_CHECK(owner.is_valid());
    const auto frozen=owner.jsondump();
    const auto input=NurbsCurve::create(false,1,{Point(.5,.5,-1),Point(.5,.5,1)});
    for (int api=0;api<3;++api) {
        bool rejected=false;
        try {
            if (api==0)
                (void)brep_section::curve(input,owner,1e-7);
            else if (api==1)
                (void)brep_section::faces(owner,owner,1e-7);
            else
                (void)brep_section::interferences(owner,owner,1e-7);
        } catch (const brep_section::RangedInputError& error) {
            rejected=true;
            MINI_CHECK(error.operand==0 && error.face==0 && error.edge==0 && error.pcurve==-1);
            MINI_CHECK(error.range.first==0 && error.range.second==1);
        }
        MINI_CHECK(rejected);
        MINI_CHECK(owner.jsondump()==frozen);
    }
    bool assembly_rejected=false;
    try {
        (void)brep_assemble::assemble({{&owner,0,false}},1e-7);
    } catch (const std::invalid_argument& error) {
        assembly_rejected=std::string(error.what())==
            "BRep assembly does not yet support partial original curve uses";
    }
    MINI_CHECK(assembly_rejected && owner.jsondump()==frozen);
}
} // namespace session_cpp
