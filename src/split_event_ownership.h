#pragma once
#include "split.h"
#include "nurbs_interval.h"
#include "nurbs_coincidence.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace session_cpp::split::detail {
struct EventSource {
    const NurbsCurve* uv=nullptr;
    bool boundary=false;
    int first_vertex=-1,last_vertex=-1;
};
// Only a topologically closed original edge with exact clamped UV endpoints
// can identify its two endpoint aliases. Interior visits remain distinct.
inline bool closed_boundary_endpoints(const EventSource& source) {
    if (!source.boundary || !source.uv || source.first_vertex<0 || source.first_vertex!=source.last_vertex)
        return false;
    const auto& curve=*source.uv;
    if (!curve.is_valid() || !curve.is_clamped() || curve.dimension()<2 || curve.dimension()>3)
        return false;
    const int last=curve.cv_count()-1;
    const double a=curve.weight(0),b=curve.weight(last);
    if (!(a>0) || !(b>0) || !std::isfinite(a) || !std::isfinite(b)) return false;
    for (int d=0;d<curve.dimension();++d)
        if (!std::isfinite(curve.cv(0)[d]) || !std::isfinite(curve.cv(last)[d]) ||
            !nurbs_coincidence::exact_products_equal(curve.cv(0)[d],b,curve.cv(last)[d],a))
            return false;
    return true;
}
inline bool closed_endpoint_pair(const EventSource& source,double a,double b) {
    return closed_boundary_endpoints(source) &&
        ((a==source.uv->domain_start() && b==source.uv->domain_end()) ||
         (b==source.uv->domain_start() && a==source.uv->domain_end()));
}
// A positive-weight rational linear span is injective in any coordinate whose
// endpoint Cartesian enclosures are separated. Do not alias reentrant occurrences.
inline bool injective_linear_span(const NurbsCurve& curve) {
    if (!curve.is_valid() || curve.dimension()<2 || curve.dimension()>3 || curve.degree()!=1 || curve.cv_count()!=2)
        return false;
    const auto controls=nurbs_interval::curve_controls(curve,{curve.domain_start(),curve.domain_end()},{0,0,0});
    if (controls.size()!=2 || !(controls[0][3].lo>0) || !(controls[1][3].lo>0))
        return false;
    for (int d=0;d<2;++d) {
        const auto a=controls[0][d]/controls[0][3],b=controls[1][d]/controls[1][3];
        if (nurbs_interval::finite(a) && nurbs_interval::finite(b) && (a.hi<b.lo || b.hi<a.lo))
            return true;
    }
    return false;
}

using BoundaryPointKey=std::pair<size_t,double>;
inline std::optional<BoundaryPointKey> boundary_point_key(
    const ArrangementPointIncidence& incidence,const std::vector<EventSource>& sources) {
    if (incidence.source>=sources.size() || incidence.other_source>=sources.size() ||
        !std::isfinite(incidence.parameter) || !std::isfinite(incidence.other_parameter))
        return std::nullopt;
    const bool first=sources[incidence.source].boundary,second=sources[incidence.other_source].boundary;
    if (first==second) {
        if (incidence.source==incidence.other_source &&
            closed_endpoint_pair(sources[incidence.source],incidence.parameter,incidence.other_parameter))
            return BoundaryPointKey{incidence.source,sources[incidence.source].uv->domain_start()};
        return std::nullopt;
    }
    const size_t index=first?incidence.source:incidence.other_source;
    double parameter=first?incidence.parameter:incidence.other_parameter;
    if (closed_boundary_endpoints(sources[index]) && parameter==sources[index].uv->domain_end())
        parameter=sources[index].uv->domain_start();
    return BoundaryPointKey{index,parameter};
}

// A proof-only kernel: all original curves and raw event receipts are immutable.
// Contains must be a whole original-interval certificate, never point sampling.
template<class Contains>
std::optional<BoundaryPointKey> boundary_vertex_key(
    const NurbsCurve& original_span,size_t source,const std::vector<EventSource>& sources,
    const std::vector<ArrangementPointIncidence>& incidence,const std::vector<double>& required,
    bool nonpoint,double lo,double hi,const Point& vertex,double tolerance,size_t& budget,Contains contains) {
    if (source>=sources.size() || !sources[source].uv || sources[source].boundary || nonpoint ||
        !std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(tolerance) || !(tolerance>0) || !(lo<hi) || !injective_linear_span(original_span) || incidence.empty())
        return std::nullopt;
    for (int d=0;d<3;++d)
        if (!std::isfinite(vertex[d]))
            return std::nullopt;
    const auto key=boundary_point_key(incidence.front(),sources);
    if (!key)
        return std::nullopt;
    bool first=false,last=false;
    for (const auto& event:incidence) {
        if (budget==0)
            throw std::runtime_error("Boundary event ownership exceeds the bounded workload");
        --budget;
        if (!std::isfinite(event.first) || !std::isfinite(event.last) ||
            !std::isfinite(event.other_first) || !std::isfinite(event.other_last) ||
            !(event.first<event.last) || !(event.other_first<event.other_last) ||
            event.parameter<event.first || event.parameter>event.last ||
            event.other_parameter<event.other_first || event.other_parameter>event.other_last ||
            boundary_point_key(event,sources)!=key ||
            (event.source==source && (event.parameter<lo || event.parameter>hi)) ||
            (event.other_source==source && (event.other_parameter<lo || event.other_parameter>hi)) ||
            !sources[event.source].uv || !sources[event.other_source].uv ||
            !contains(*sources[event.source].uv,event.parameter,event.parameter,vertex,tolerance) ||
            !contains(*sources[event.other_source].uv,event.other_parameter,event.other_parameter,vertex,tolerance))
            return std::nullopt;
        if (event.source==source) {
            first=first || event.parameter==lo;
            last=last || event.parameter==hi;
        }
    }
    if (!first || !last || !contains(*sources[source].uv,lo,hi,vertex,tolerance))
        return std::nullopt;
    for (double parameter:required) {
        if (!std::isfinite(parameter))
            return std::nullopt;
        if (parameter>=lo && parameter<=hi && std::none_of(incidence.begin(),incidence.end(),
            [&](const auto& event){return event.source==source && event.parameter==parameter;}))
            return std::nullopt;
    }
    return key;
}
} // namespace session_cpp::split::detail
