#pragma once
#include "split.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace session_cpp::brep_boolean::detail {
struct OriginalEventIdentityError : std::invalid_argument {
    split::EdgeEvent first,second;
    OriginalEventIdentityError(split::EdgeEvent a,split::EdgeEvent b,const char* reason)
        :std::invalid_argument(reason),first(a),second(b) {}
};
// This is the actual Boolean event consumer, including required splitting and
// receipt validation. Canonical identity and raw occurrence are separate facts.
inline split::FaceSplit split_preserved_original_events(const BRep& owner,
    std::vector<split::EdgeEvent> events,double tolerance) {
    for (const auto& event:events)
        if (!std::isfinite(event.parameter) || event.vertex<0)
            throw OriginalEventIdentityError(event,event,"Original event lacks a finite raw parameter or canonical vertex");
    std::sort(events.begin(),events.end(),[](const auto& a,const auto& b) {
        if (a.edge!=b.edge) return a.edge<b.edge;
        if (a.parameter!=b.parameter) return a.parameter<b.parameter;
        return a.vertex<b.vertex;
    });
    std::vector<split::EdgeEvent> retained;
    for (const auto& event:events) {
        if (!retained.empty() && retained.back().edge==event.edge && retained.back().parameter==event.parameter) {
            if (retained.back().vertex!=event.vertex)
                throw OriginalEventIdentityError(retained.back(),event,
                    "One original edge raw event has conflicting canonical vertex identities");
            continue; // Only the identical full (edge,raw parameter,vertex) receipt.
        }
        retained.push_back(event);
    }
    auto result=split::split_brep_edges_at_events(owner,retained,tolerance);
    for (const auto& event:retained)
        if (std::none_of(result.edge_events.begin(),result.edge_events.end(),[&](const auto& receipt) {
                return receipt.edge==event.edge && receipt.parameter==event.parameter && receipt.vertex==event.vertex;
            }))
            throw std::runtime_error("Original edge event receipt lost its canonical owner-local vertex or raw parameter");
    return result;
}
} // namespace session_cpp::brep_boolean::detail
