#pragma once
#include "section_exact_linear_event.h"
#include <algorithm>

namespace session_cpp::brep_section::detail {
struct OriginalKnotSigns {
    std::vector<std::array<int,3>> signs;
    std::vector<double> knots;
    size_t degree=0,count=0;
    double first=0,last=0;
};
inline bool original_knot_signs(const NurbsCurve& curve,const std::array<double,4>& witness,
    OriginalKnotSigns& data) {
    if (!exact_raw_clamped(curve) || curve.dimension()!=3 ||
        !std::isfinite(witness[3]) || !(witness[3]>0)) return false;
    for (int d=0;d<3;++d) if (!std::isfinite(witness[d])) return false;
    data.degree=static_cast<size_t>(curve.degree());
    data.count=static_cast<size_t>(curve.cv_count());
    data.first=curve.domain_start();data.last=curve.domain_end();
    const auto& compact=curve.m_nurbsknot;
    for (size_t i=0;i<compact.size();) {
        size_t j=i+1;
        while (j<compact.size() && compact[j]==compact[i]) ++j;
        if (compact[i]>data.first && compact[i]<data.last && j-i>data.degree) return false;
        i=j;
    }
    data.signs.resize(data.count);
    for (size_t i=0;i<data.count;++i) {
        const double weight=curve.weight(static_cast<int>(i));
        if (!std::isfinite(weight) || !(weight>0)) return false;
        for (int d=0;d<3;++d) {
            const double coordinate=curve.cv(static_cast<int>(i))[d];
            if (!std::isfinite(coordinate)) return false;
            const double sign=exact_orientation::orient2d(0.,0.,witness[3],witness[d],weight,coordinate);
            data.signs[i][d]=sign>0?1:sign<0?-1:0;
        }
    }
    data.knots=compact;
    data.knots.insert(data.knots.begin(),compact.front());
    data.knots.push_back(compact.back());
    return true;
}
inline bool original_knot_value_is_point(const OriginalKnotSigns& data,double parameter) {
    const auto zero=[&](size_t i) {
        return data.signs[i][0]==0 && data.signs[i][1]==0 && data.signs[i][2]==0;
    };
    if (parameter==data.first) return zero(0);
    if (parameter==data.last) return zero(data.count-1);
    if (!(parameter>data.first && parameter<data.last) ||
        !std::binary_search(data.knots.begin(),data.knots.end(),parameter)) return false;
    bool active=false;
    for (size_t i=0;i<data.count;++i)
        if (data.knots[i]<parameter && parameter<data.knots[i+data.degree+1]) {
            active=true;
            if (!zero(i)) return false;
        }
    return active;
}
/// Prove this literal ORIGINAL knot occurrence equals the projective point.
inline bool original_knot_is_preimage(const NurbsCurve& curve,const std::array<double,4>& witness,double parameter) {
    OriginalKnotSigns data;
    return original_knot_signs(curve,witness,data) && original_knot_value_is_point(data,parameter);
}
/// Prove ALL point preimages are the returned ORIGINAL knots; continuum/open-span roots decline.
inline bool original_knot_preimages_only(const NurbsCurve& curve,const std::array<double,4>& witness,
    std::vector<double>& parameters) {
    parameters.clear();
    OriginalKnotSigns data;
    if (!original_knot_signs(curve,witness,data)) return false;
    std::vector<double> found;
    if (original_knot_value_is_point(data,data.first)) found.push_back(data.first);
    for (size_t span=data.degree;span<data.count;++span) {
        const double lo=data.knots[span],hi=data.knots[span+1];
        if (!(lo<hi)) continue;
        if (!endpoint_active_signs_exclude(data.signs,data.knots,span-data.degree,span,data.degree,lo,hi))
            return false;
        if (lo>data.first && !endpoint_active_signs_exclude(
                data.signs,data.knots,span-data.degree,span,data.degree,lo,lo)) {
            if (!original_knot_value_is_point(data,lo)) return false;
            found.push_back(lo);
        }
    }
    if (original_knot_value_is_point(data,data.last)) found.push_back(data.last);
    parameters=std::move(found);
    return true;
}
} // namespace session_cpp::brep_section::detail
