#pragma once
#include "brep_section.h"
#include "section_exact_linear_event.h"
#include "nurbs_interval.h"

namespace session_cpp::brep_section::detail {
/// Ordinary-node alias only: both literal ends of ONE original closed edge, never an interior raw visit.
inline bool original_closed_endpoint_alias(const BoundaryEvent& first,const BoundaryEvent& last,
    const BRep& owner,double tolerance) {
    const double cap=tolerance*.25;
    if (!std::isfinite(cap) || !(cap>0) || first.operand!=last.operand ||
        (first.operand!=0 && first.operand!=1) || first.edge!=last.edge || first.edge<0 ||
        first.edge>=owner.edge_count() || first.vertex<0 || first.vertex!=last.vertex ||
        first.vertex>=owner.vertex_count() || first.uses.empty() || last.uses.empty()) return false;
    const auto& edge=owner.m_edges[first.edge];
    if (edge.degenerated || edge.start_vertex!=edge.end_vertex || edge.start_vertex!=first.vertex ||
        edge.curve_3d_index<0 || edge.curve_3d_index>=static_cast<int>(owner.m_curves_3d.size())) return false;
    const auto& world=owner.m_curves_3d[edge.curve_3d_index];
    if (!exact_raw_clamped(world) || world.dimension()!=3 || nurbs_coincidence::curve_constant_image(world))
        return false;
    const double a=world.domain_start(),b=world.domain_end();
    if (!((first.hit.t==a && last.hit.t==b) || (first.hit.t==b && last.hit.t==a))) return false;
    for (int i=0;i<world.cv_count();++i)
        if (!std::isfinite(world.weight(i)) || !(world.weight(i)>0)) return false;
    const auto& point=owner.m_vertices[first.vertex].point;
    return nurbs_interval::curve_within_ball(world,{first.hit.t,first.hit.t},
               {point[0],point[1],point[2]},cap) &&
           nurbs_interval::curve_within_ball(world,{last.hit.t,last.hit.t},
               {point[0],point[1],point[2]},cap);
}
} // namespace session_cpp::brep_section::detail
