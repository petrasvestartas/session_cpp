#pragma once
#include "brep_section.h"
#include "curve_surface_intersection.h"
#include "nurbs_interval.h"
#include "split.h"
#include <limits>
#include <algorithm>

namespace session_cpp::brep_boolean::detail {
struct CutEndpointReceipt {
    size_t node=std::numeric_limits<size_t>::max();
    int section=-1, end=-1;
    double world_parameter=0;
    std::vector<size_t> boundary_events;
};
struct CutJoinError : std::runtime_error {
    int face;
    CutEndpointReceipt first,second;
    CutJoinError(int owner,CutEndpointReceipt a,CutEndpointReceipt b,const char* reason)
        :std::runtime_error(reason),face(owner),first(std::move(a)),second(std::move(b)) {}
};
inline bool same_raw_curve(const NurbsCurve& a,const NurbsCurve& b) {
    if (a.dimension()!=b.dimension() || a.is_rational()!=b.is_rational() || a.degree()!=b.degree() ||
        a.cv_count()!=b.cv_count() || a.m_nurbsknot!=b.m_nurbsknot)
        return false;
    for (int i=0;i<a.cv_count();++i)
        for (int d=0;d<a.dimension()+(a.is_rational()?1:0);++d)
            if (a.cv(i)[d]!=b.cv(i)[d])
                return false;
    return true;
}
inline CutEndpointReceipt cut_endpoint_receipt(const split::FaceCut& cut,int end,int operand,int face,
    const brep_section::FaceResult& inventory,double tolerance) {
    CutEndpointReceipt receipt;
    receipt.section=cut.section;
    receipt.end=end;
    receipt.world_parameter=end?cut.world.domain_end():cut.world.domain_start();
    if (cut.section<0 || static_cast<size_t>(cut.section)>=inventory.curves.size())
        return receipt;
    const auto& original=inventory.curves[cut.section];
    const int own_face=operand?original.second_face:original.first_face;
    if (own_face!=face || !same_raw_curve(cut.world,original.world) ||
        !same_raw_curve(cut.pcurve,operand?original.second_pcurve:original.first_pcurve))
        return receipt;
    for (size_t n=0;n<inventory.nodes.size();++n) {
        std::vector<size_t> matches;
        for (size_t index:inventory.nodes[n].boundary_events) {
            if (index>=inventory.boundary_events.size())
                throw CutJoinError(face,receipt,{},"Invalid canonical boundary-event index");
            const auto& event=inventory.boundary_events[index];
            if (event.operand!=0 && event.operand!=1)
                continue;
            const int event_face=event.operand?original.second_face:original.first_face;
            const int target_face=event.operand?original.first_face:original.second_face;
            if (event.hit.face!=target_face ||
                std::none_of(event.uses.begin(),event.uses.end(),[&](const auto& use){return use.face==event_face;}))
                continue;
            for (const auto& owner:event.owners)
                if (owner.curve==static_cast<size_t>(cut.section) && owner.source_parameter==event.hit.t &&
                    owner.target_parameter==receipt.world_parameter)
                    matches.push_back(index);
        }
        if (matches.empty())
            continue;
        const auto& center=inventory.nodes[n].point;
        if (!nurbs_interval::curve_within_ball(cut.world,{receipt.world_parameter,receipt.world_parameter},
                {center[0],center[1],center[2]},tolerance*.25))
            throw CutJoinError(face,receipt,{},"Original cut endpoint lacks its canonical node singleton certificate");
        if (receipt.node!=std::numeric_limits<size_t>::max() && receipt.node!=n)
            throw CutJoinError(face,receipt,{},"Original cut endpoint has conflicting canonical nodes");
        receipt.node=n;
        receipt.boundary_events=std::move(matches);
    }
    return receipt;
}
inline void bind_cut_endpoints(split::FaceCut& cut,int operand,int face,
    const brep_section::FaceResult& inventory,const NurbsSurface& support,double tolerance) {
    cut.canonical_endpoints={};
    const auto first=cut_endpoint_receipt(cut,0,operand,face,inventory,tolerance);
    const auto last=cut_endpoint_receipt(cut,1,operand,face,inventory,tolerance);
    const auto absent=std::numeric_limits<size_t>::max();
    if (first.node==absent && last.node==absent)
        return;
    if (curve_surface_intersection::same_parameter(cut.world,cut.pcurve,support,tolerance)!=
            curve_surface_intersection::ParameterMatch::Matched)
        throw CutJoinError(face,first,last,"Canonical cut endpoint lacks original world/pcurve law");
    cut.canonical_endpoints[0]={first.node,first.world_parameter,first.boundary_events};
    cut.canonical_endpoints[1]={last.node,last.world_parameter,last.boundary_events};
}
} // namespace session_cpp::brep_boolean::detail
