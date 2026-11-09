#pragma once
#include "brep_polygon_trim_geometry.h"
#include "nurbs_interval.h"
#include <tuple>
#include <numeric>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace session_cpp::brep_section::detail {
inline auto polygon_event_key(const OriginalTrimEvent& event) {
    return std::tuple{event.operand,
                      event.face,
                      event.wire,
                      event.occurrence,
                      event.edge,
                      event.pcurve,
                      event.orientation,
                      event.section_parameter,
                      event.boundary_parameter,
                      event.overlap_endpoint};
}
inline bool polygon_event_inventory(const OriginalTrimLaw& law, const PolygonTrimProof& first,
                                    const PolygonTrimProof& second, std::vector<double>& cuts) {
    std::vector<OriginalTrimEvent> expected = first.events;
    expected.insert(expected.end(), second.events.begin(), second.events.end());
    auto actual = law.events;
    for (const auto& event : actual)
        if (!std::isfinite(event.section_parameter) || !std::isfinite(event.boundary_parameter))
            return false;
    const auto less = [](const auto& a, const auto& b) {
        return polygon_event_key(a) < polygon_event_key(b);
    };
    std::sort(expected.begin(), expected.end(), less);
    std::sort(actual.begin(), actual.end(), less);
    if (expected.size() != actual.size())
        return false;
    cuts = {law.original.world.domain_start(), law.original.world.domain_end()};
    for (size_t i = 0; i < expected.size(); ++i) {
        if (!std::isfinite(actual[i].section_parameter) || !std::isfinite(actual[i].boundary_parameter) ||
            polygon_event_key(expected[i]) != polygon_event_key(actual[i]))
            return false;
        cuts.push_back(expected[i].section_parameter);
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    return cuts == law.partition;
}
inline bool polygon_included(brep_classify::State state) {
    return state == brep_classify::State::Inside || state == brep_classify::State::Boundary;
}
inline bool polygon_pair_state(const OriginalTrimLaw& law, const PolygonTrimProof& first,
                               const PolygonTrimProof& second, double t, bool& included) {
    const auto a = polygon_state(first, law.original.first_pcurve, t);
    const auto b = polygon_state(second, law.original.second_pcurve, t);
    if (!a || !b)
        return false;
    included = polygon_included(*a) && polygon_included(*b);
    return true;
}
inline bool polygon_piece(const OriginalTrimLaw& law, const FaceCurve& piece, double lo, double hi) {
    if (piece.first_face != law.original.first_face || piece.second_face != law.original.second_face)
        return false;
    const NurbsCurve* originals[] = {&law.original.world, &law.original.first_pcurve,
                                     &law.original.second_pcurve};
    const NurbsCurve* pieces[] = {&piece.world, &piece.first_pcurve, &piece.second_pcurve};
    for (int i = 0; i < 3; ++i) {
        if (pieces[i]->domain_start() != lo || pieces[i]->domain_end() != hi)
            return false;
        std::array<Point, 2> endpoints;
        if (!polygon_linear(*pieces[i], endpoints))
            return false;
        for (int end = 0; end < 2; ++end) {
            Point original;
            if (!polygon_line_point(*originals[i], end ? hi : lo, original) ||
                !polygon_same_point(original, endpoints[end]))
                return false;
        }
    }
    return true;
}
inline bool polygon_partition(const FaceResult& inventory, const OriginalTrimLaw& law,
                              const PolygonTrimProof& first, const PolygonTrimProof& second,
                              const std::vector<double>& cuts, std::vector<bool>& kept) {
    size_t index = 0;
    for (size_t i = 1; i < cuts.size(); ++i) {
        const double mid = std::midpoint(cuts[i - 1], cuts[i]);
        bool included;
        if (!(cuts[i - 1] < mid && mid < cuts[i]) || !polygon_pair_state(law, first, second, mid, included))
            return false;
        kept.push_back(included);
        if (!included)
            continue;
        if (index == law.retained_pieces.size())
            return false;
        const auto& piece = law.retained_pieces[index++];
        if (piece.first != cuts[i - 1] || piece.last != cuts[i] || piece.curve >= inventory.curves.size() ||
            !polygon_piece(law, inventory.curves[piece.curve], piece.first, piece.last))
            return false;
    }
    return index == law.retained_pieces.size();
}
inline bool polygon_accepted_range(double lo, double hi, const std::vector<double>& cuts,
                                   const std::vector<bool>& kept) {
    if (!(lo < hi) || lo < cuts.front() || hi > cuts.back())
        return false;
    for (size_t i = 1; i < cuts.size(); ++i)
        if (lo < cuts[i] && hi > cuts[i - 1] && !kept[i - 1])
            return false;
    return true;
}
inline bool polygon_extra_curves(const FaceResult& inventory, const OriginalTrimLaw& law,
                                 const std::vector<double>& cuts, const std::vector<bool>& kept) {
    for (const auto& curve : inventory.curves) {
        if (curve.first_face != law.original.first_face || curve.second_face != law.original.second_face)
            continue;
        std::array<Point, 2> ends;
        double first, last;
        if (!polygon_linear(curve.world, ends) ||
            !polygon_line_parameter(law.original.world, ends[0], first) ||
            !polygon_line_parameter(law.original.world, ends[1], last))
            return false;
        if (!polygon_accepted_range(std::min(first, last), std::max(first, last), cuts, kept))
            return false;
        const NurbsCurve* original[] = {&law.original.first_pcurve, &law.original.second_pcurve};
        const NurbsCurve* actual[] = {&curve.first_pcurve, &curve.second_pcurve};
        for (int operand = 0; operand < 2; ++operand) {
            std::array<Point, 2> uv;
            if (actual[operand]->domain() != curve.world.domain() || !polygon_linear(*actual[operand], uv))
                return false;
            for (int end = 0; end < 2; ++end) {
                Point expected;
                if (!polygon_line_point(*original[operand], end ? last : first, expected) ||
                    !polygon_same_point(expected, uv[end]))
                    return false;
            }
        }
    }
    return true;
}
inline bool polygon_contact_receipt(const FaceResult& inventory, const OriginalTrimLaw& law,
                                    const PolygonTrimEdge& edge, const OriginalTrimEvent& expected,
                                    double tolerance, bool isolated) {
    Point point, uv, owner_point;
    double owner_raw;
    if (!polygon_line_point(law.original.world, expected.section_parameter, point) ||
        !polygon_line_point(*edge.uv, expected.boundary_parameter, uv) ||
        !polygon_line_parameter(*edge.world, point, owner_raw))
        return false;
    // Equal normalized original boundary laws: no Closest-derived raw correspondence.
    double a, b, c, d;
    if (!event_difference(expected.boundary_parameter, edge.uv->domain_start(), a) ||
        !event_difference(edge.uv->domain_end(), edge.uv->domain_start(), b) ||
        !event_difference(owner_raw, edge.world->domain_start(), c) ||
        !event_difference(edge.world->domain_end(), edge.world->domain_start(), d) ||
        !nurbs_coincidence::exact_products_equal(a, d, c, b) ||
        !polygon_line_point(*edge.world, owner_raw, owner_point) || !polygon_same_point(owner_point, point))
        return false;
    const int opposite = expected.operand ? law.original.first_face : law.original.second_face;
    for (const auto& node : inventory.nodes) {
        if (isolated && !node.isolated)
            continue;
        if (!nurbs_interval::curve_within_ball(
                law.original.world, {expected.section_parameter, expected.section_parameter},
                {node.point[0], node.point[1], node.point[2]}, tolerance * .25))
            continue;
        for (size_t index : node.boundary_events) {
            if (index >= inventory.boundary_events.size())
                return false;
            const auto& event = inventory.boundary_events[index];
            if (event.operand != expected.operand || event.edge != expected.edge ||
                event.hit.face != opposite || event.hit.t != owner_raw)
                continue;
            for (const auto& use : event.uses)
                if (use.face == expected.face && use.wire == expected.wire &&
                    use.occurrence == expected.occurrence && use.pcurve == expected.pcurve &&
                    use.orientation == expected.orientation)
                    return true;
        }
    }
    return false;
}
inline bool polygon_contacts(const FaceResult& inventory, const OriginalTrimLaw& law,
                             const PolygonTrimProof& first, const PolygonTrimProof& second,
                             const std::vector<double>& cuts, const std::vector<bool>& kept,
                             double tolerance) {
    for (size_t i = 0; i < cuts.size(); ++i) {
        bool included;
        if (!polygon_pair_state(law, first, second, cuts[i], included))
            return false;
        if (!included)
            continue;
        const bool isolated = !(i > 0 && kept[i - 1]) && !(i < kept.size() && kept[i]);
        bool receipt = false;
        for (const auto* proof : {&first, &second})
            for (const auto& wire : proof->wires)
                for (const auto& edge : wire)
                    for (const auto& event : proof->events)
                        if (event.section_parameter == cuts[i] && event.operand == edge.use.operand &&
                            event.wire == edge.use.wire && event.occurrence == edge.use.occurrence) {
                            if (!polygon_contact_receipt(inventory, law, edge, event, tolerance, isolated))
                                return false;
                            receipt = true;
                        }
        // A support-domain endpoint strictly inside both trims is represented by its closed retained piece.
        if (isolated && !receipt)
            return false;
    }
    return true;
}
inline bool polygon_extra_events(const FaceResult& inventory, const OriginalTrimLaw& law,
                                 const PolygonTrimProof& first, const PolygonTrimProof& second) {
    for (const auto& support : inventory.support_points)
        if (support.retained && support.first_face == law.original.first_face &&
            support.second_face == law.original.second_face)
            return false;
    for (const auto& event : inventory.boundary_events) {
        if (!event.retained) continue;
        const int own = event.operand ? law.original.second_face : law.original.first_face;
        const int opposite = event.operand ? law.original.first_face : law.original.second_face;
        if ((event.operand != 0 && event.operand != 1) || event.hit.face != opposite)
            continue;
        for (const auto& use : event.uses) {
            if (use.face != own)
                continue;
            const auto& proof = event.operand ? second : first;
            bool found = false;
            for (const auto& wire : proof.wires)
                for (const auto& edge : wire) {
                    if (edge.use.edge != event.edge || edge.use.wire != use.wire ||
                        edge.use.occurrence != use.occurrence || edge.use.pcurve != use.pcurve)
                        continue;
                    Point original;
                    if (!polygon_line_point(*edge.world, event.hit.t, original))
                        return false;
                    for (const auto& expected : proof.events) {
                        if (expected.edge != event.edge || expected.wire != use.wire ||
                            expected.occurrence != use.occurrence || expected.pcurve != use.pcurve)
                            continue;
                        Point section;
                        if (polygon_line_point(law.original.world, expected.section_parameter, section) &&
                            polygon_same_point(original, section)) {
                            bool included;
                            if (!polygon_pair_state(law, first, second, expected.section_parameter,
                                                    included) ||
                                !included)
                                return false;
                            found = true;
                        }
                    }
                }
            if (!found)
                return false;
        }
    }
    return true;
}
inline bool observe_polygon_trim_failure(const char* stage,const FaceResult& inventory,
    const OriginalTrimLaw& law,const BRep& a,const BRep& b,double tolerance,
    const PolygonTrimProof* first=nullptr,const PolygonTrimProof* second=nullptr) {
    const char* path=std::getenv("SESSION_POLYGON_TRIM_FAILURE");
    if (!path || !*path) return false;
    try {
        nlohmann::ordered_json saved={{"stage",stage},{"tolerance",tolerance},
            {"first_face",law.original.first_face},{"second_face",law.original.second_face},
            {"first_brep",a.jsondump()},{"second_brep",b.jsondump()},
            {"world",law.original.world.jsondump()},{"first_pcurve",law.original.first_pcurve.jsondump()},
            {"second_pcurve",law.original.second_pcurve.jsondump()},
            {"partition",law.partition},{"actual_events",nlohmann::ordered_json::array()},
            {"expected_events",nlohmann::ordered_json::array()},{"pieces",nlohmann::ordered_json::array()},
            {"curves",nlohmann::ordered_json::array()},{"nodes",nlohmann::ordered_json::array()},
            {"boundary_events",nlohmann::ordered_json::array()}};
        const auto event_json=[](const OriginalTrimEvent& event) {
            return nlohmann::ordered_json{{"operand",event.operand},{"face",event.face},{"wire",event.wire},
                {"occurrence",event.occurrence},{"edge",event.edge},{"pcurve",event.pcurve},
                {"orientation",static_cast<int>(event.orientation)},{"section_parameter",event.section_parameter},
                {"boundary_parameter",event.boundary_parameter},{"overlap_endpoint",event.overlap_endpoint}};
        };
        for (const auto& event:law.events) saved["actual_events"].push_back(event_json(event));
        for (const auto* proof:{first,second}) if (proof)
            for (const auto& event:proof->events) saved["expected_events"].push_back(event_json(event));
        for (const auto& piece:law.retained_pieces)
            saved["pieces"].push_back({{"first",piece.first},{"last",piece.last},{"curve",piece.curve}});
        for (const auto& curve:inventory.curves)
            saved["curves"].push_back({{"first_face",curve.first_face},{"second_face",curve.second_face},
                {"world",curve.world.jsondump()},{"first_pcurve",curve.first_pcurve.jsondump()},
                {"second_pcurve",curve.second_pcurve.jsondump()}});
        for (const auto& node:inventory.nodes)
            saved["nodes"].push_back({{"point",{node.point[0],node.point[1],node.point[2]}},
                {"boundary_events",node.boundary_events},{"isolated",node.isolated}});
        for (const auto& event:inventory.boundary_events) {
            nlohmann::ordered_json row={{"operand",event.operand},{"edge",event.edge},
                {"face",event.hit.face},{"raw",event.hit.t},{"retained",event.retained},
                {"source_parameters",event.source_parameters},{"uses",nlohmann::ordered_json::array()}};
            for (const auto& use:event.uses)
                row["uses"].push_back({{"face",use.face},{"wire",use.wire},{"occurrence",use.occurrence},
                    {"pcurve",use.pcurve},{"orientation",static_cast<int>(use.orientation)}});
            saved["boundary_events"].push_back(std::move(row));
        }
        std::ofstream output;output.exceptions(std::ios::failbit|std::ios::badbit);
        output.open(path,std::ios::app);output<<saved.dump()<<'\n';output.flush();output.close();
    } catch (const std::exception& error) {
        std::cerr<<"Polygon trim failure capture failed: "<<error.what()<<'\n';
    }
    return false;
}
inline bool polygon_trim_coverage(const FaceResult& inventory, const OriginalTrimLaw& law, const BRep& a,
                                  const BRep& b, double tolerance) {
    const auto failure=[&](const char* stage,const PolygonTrimProof* first=nullptr,const PolygonTrimProof* second=nullptr) {
        return observe_polygon_trim_failure(stage,inventory,law,a,b,tolerance,first,second);
    };
    if (!law.owned_intervals.empty() || !(tolerance > 0) || !std::isfinite(tolerance))
        return failure("eligibility");
    const auto& original = law.original;
    if (original.world.domain() != original.first_pcurve.domain() ||
        original.world.domain() != original.second_pcurve.domain()) return failure("raw_domains");
    if (!surface_section::affine_plane_curve_coverage(
            a.m_surfaces[a.m_faces[original.first_face].surface_index],
            b.m_surfaces[b.m_faces[original.second_face].surface_index],
            {{original.world, original.first_pcurve, original.second_pcurve}})) return failure("support");
    auto first = polygon_boundary(a, law.original.first_face, 0, tolerance);
    auto second = polygon_boundary(b, law.original.second_face, 1, tolerance);
    if (!first || !second) return failure("boundary",first?&*first:nullptr,second?&*second:nullptr);
    if (!polygon_all_events(*first, law.original.first_pcurve)) return failure("first_events",&*first,&*second);
    if (!polygon_all_events(*second, law.original.second_pcurve)) return failure("second_events",&*first,&*second);
    std::vector<double> cuts;
    std::vector<bool> kept;
    if (!polygon_event_inventory(law,*first,*second,cuts)) return failure("event_inventory",&*first,&*second);
    if (!polygon_partition(inventory,law,*first,*second,cuts,kept)) return failure("partition",&*first,&*second);
    if (!polygon_extra_curves(inventory,law,cuts,kept)) return failure("extra_curves",&*first,&*second);
    if (!polygon_extra_events(inventory,law,*first,*second)) return failure("extra_events",&*first,&*second);
    if (!polygon_contacts(inventory,law,*first,*second,cuts,kept,tolerance)) return failure("contacts",&*first,&*second);
    return true;
}
} // namespace session_cpp::brep_section::detail
