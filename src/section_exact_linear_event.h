#pragma once
#include "nurbscurve.h"
#include "nurbs_coincidence.h"
#include "exact_orientation.h"
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace session_cpp::brep_section::detail {
struct ExactLinearEvent {
    std::array<double,3> homogeneous;
    double weight=0;
    Point center;
};
inline bool event_product(double a,double b,double& result) {
    result=a*b;
    return std::isfinite(result) && nurbs_coincidence::exact_products_equal(a,b,result,1.);
}
inline bool event_difference(double a,double b,double& result) {
    return nurbs_coincidence::exact_difference(a,b,result) && std::isfinite(result);
}
inline bool event_sum_products(double a,double b,double c,double d,double& result) {
    double first,second;
    return event_product(a,b,first) && event_product(c,d,second) &&
        event_difference(first,-second,result);
}
inline bool event_line(const NurbsCurve& curve,std::array<double,3>& start,std::array<double,3>& direction) {
    if (!curve.is_valid() || curve.dimension()!=3 || curve.degree()!=1 || curve.cv_count()!=2)
        return false;
    std::array<double,3> end;
    for (int i=0;i<2;++i) {
        const double* cv=curve.cv(i);
        const double w=curve.is_rational()?cv[3]:1.;
        if (!std::isfinite(w) || !(w>0)) return false;
        for (int d=0;d<3;++d) {
            const double x=cv[d]/w;
            if (!std::isfinite(x) || !nurbs_coincidence::exact_products_equal(x,w,cv[d],1.))
                return false;
            (i?end:start)[d]=x;
        }
    }
    bool positive=false;
    for (int d=0;d<3;++d) {
        if (!event_difference(end[d],start[d],direction[d])) return false;
        positive=positive || direction[d]!=0;
    }
    return positive;
}
inline std::optional<ExactLinearEvent> exact_linear_crossing(const NurbsCurve& first,const NurbsCurve& second) {
    std::array<double,3> a,d,b,e,q;
    if (!event_line(first,a,d) || !event_line(second,b,e)) return std::nullopt;
    for (int k=0;k<3;++k)
        if (!event_difference(b[k],a[k],q[k])) return std::nullopt;
    for (int i=0;i<3;++i) for (int j=i+1;j<3;++j) {
        double denominator,s,t;
        if (!event_sum_products(d[i],e[j],-d[j],e[i],denominator) || denominator==0 ||
            !event_sum_products(q[i],e[j],-q[j],e[i],s) ||
            !event_sum_products(q[i],d[j],-q[j],d[i],t)) continue;
        if (denominator<0) {denominator=-denominator;s=-s;t=-t;}
        if (s<0 || s>denominator || t<0 || t>denominator) continue;
        ExactLinearEvent result;result.weight=denominator;
        bool equal=true;
        for (int k=0;k<3;++k) {
            double other;
            if (!event_sum_products(a[k],denominator,d[k],s,result.homogeneous[k]) ||
                !event_sum_products(b[k],denominator,e[k],t,other) || other!=result.homogeneous[k]) {
                equal=false;break;
            }
            result.center[k]=result.homogeneous[k]/denominator;
            equal=equal && std::isfinite(result.center[k]);
        }
        if (equal) return result;
    }
    return std::nullopt;
}
inline bool exact_axis_event_on_raw_interval(const ExactLinearEvent& event,const NurbsCurve& world,double lo,double hi,bool singleton=false) {
    if (!world.is_valid() || world.dimension()!=3 || world.degree()!=1 || world.cv_count()!=2 ||
        (!(lo<hi) && !(singleton && lo==hi)) || !std::isfinite(lo) || !std::isfinite(hi) ||
        lo<world.domain_start() || hi>world.domain_end() ||
        !std::isfinite(event.weight) || !(event.weight>0)) return false;
    const double w0=world.is_rational()?world.cv(0)[3]:1.;
    const double w1=world.is_rational()?world.cv(1)[3]:1.;
    if (!std::isfinite(w0) || !(w0>0) || w0!=w1) return false;
    int varying=-1;
    std::array<double,3> first,last,point;
    for (int d=0;d<3;++d) {
        first[d]=world.cv(0)[d]/w0;last[d]=world.cv(1)[d]/w0;
        point[d]=event.homogeneous[d]/event.weight;
        if (!std::isfinite(first[d]) || !std::isfinite(last[d]) || !std::isfinite(point[d]) ||
            !nurbs_coincidence::exact_products_equal(first[d],w0,world.cv(0)[d],1.) ||
            !nurbs_coincidence::exact_products_equal(last[d],w0,world.cv(1)[d],1.) ||
            !nurbs_coincidence::exact_products_equal(point[d],event.weight,event.homogeneous[d],1.)) return false;
        if (first[d]!=last[d]) {
            if (varying>=0) return false;
            varying=d;
        } else if (first[d]!=point[d]) return false;
    }
    if (varying<0) return false;
    const double low=exact_orientation::orient2d(world.domain_start(),first[varying],
        world.domain_end(),last[varying],lo,point[varying]);
    const double high=exact_orientation::orient2d(world.domain_start(),first[varying],
        world.domain_end(),last[varying],hi,point[varying]);
    return first[varying]<last[varying]?(low>=0 && high<=0):(low<=0 && high>=0);
}
inline bool exact_event_at_raw_parameter(const ExactLinearEvent& event,const NurbsCurve& world,double parameter) {
    if (!world.is_valid() || world.dimension()!=3 || world.degree()!=1 || world.cv_count()!=2 ||
        !std::isfinite(parameter) || parameter<world.domain_start() || parameter>world.domain_end() ||
        !std::isfinite(event.weight) || !(event.weight>0)) return false;
    const double w0=world.is_rational()?world.cv(0)[3]:1.;
    const double w1=world.is_rational()?world.cv(1)[3]:1.;
    double dt0,dt1,weight;
    if (std::isfinite(w0) && w0>0 && std::isfinite(w1) && w1>0 &&
        event_difference(parameter,world.domain_start(),dt0) &&
        event_difference(world.domain_end(),parameter,dt1) &&
        event_sum_products(dt1,w0,dt0,w1,weight) && weight>0) {
        bool equal=true;
        for (int d=0;d<3;++d) {
            double homogeneous;
            equal=equal && std::isfinite(event.homogeneous[d]) &&
                event_sum_products(dt1,world.cv(0)[d],dt0,world.cv(1)[d],homogeneous) &&
                nurbs_coincidence::exact_products_equal(homogeneous,event.weight,event.homogeneous[d],weight);
        }
        if (equal) return true;
    }
    return exact_axis_event_on_raw_interval(event,world,parameter,parameter,true);
}
inline bool exact_event_on_raw_interval(const ExactLinearEvent& event,const NurbsCurve& world,double lo,double hi) {
    if (exact_axis_event_on_raw_interval(event,world,lo,hi)) return true;
    if (!(lo<hi) || lo<world.domain_start() || hi>world.domain_end() ||
        !std::isfinite(lo) || !std::isfinite(hi) || !(event.weight>0)) return false;
    std::array<double,3> a,d,n;
    if (!event_line(world,a,d)) return false;
    if (world.is_rational() && world.cv(0)[3]!=world.cv(1)[3]) return false;
    int pivot=-1;
    for (int k=0;k<3;++k) {
        double aw;
        if (!event_product(a[k],event.weight,aw) ||
            !event_difference(event.homogeneous[k],aw,n[k])) return false;
        if (d[k]!=0 && pivot<0) pivot=k;
    }
    if (pivot<0) return false;
    for (int k=0;k<3;++k)
        if (!nurbs_coincidence::exact_products_equal(n[k],d[pivot],n[pivot],d[k])) return false;
    double denominator,width,numerator,lower,upper;
    if (!event_product(event.weight,d[pivot],denominator) ||
        !event_difference(world.domain_end(),world.domain_start(),width) ||
        !event_sum_products(world.domain_start(),denominator,width,n[pivot],numerator) ||
        !event_product(lo,denominator,lower) || !event_product(hi,denominator,upper)) return false;
    if (denominator<0) {denominator=-denominator;numerator=-numerator;lower=-lower;upper=-upper;}
    return denominator>0 && lower<=numerator && numerator<=upper;
}

inline bool endpoint_active_signs_exclude(const std::vector<std::array<int,3>>& signs,
    const std::vector<double>& knots,size_t first,size_t last,size_t degree,double lo,double hi) {
    for (int d=0;d<3;++d) {
        bool positive=false,negative=false;
        for (size_t i=first;i<=last;++i) {
            const bool active=lo==hi?(knots[i]<lo && lo<knots[i+degree+1]):
                (knots[i]<=lo && hi<=knots[i+degree+1]);
            if (!active) continue;
            positive=positive || signs[i][d]>0;
            negative=negative || signs[i][d]<0;
        }
        if (positive!=negative) return true;
    }
    return false;
}

/// Test literal original endpoint knot multiplicities, without a knot tolerance.
inline bool exact_raw_clamped(const NurbsCurve& curve) {
    if (!curve.is_valid() || curve.degree()<1) return false;
    const size_t degree=static_cast<size_t>(curve.degree()),count=static_cast<size_t>(curve.cv_count());
    const auto& compact=curve.m_nurbsknot;
    if (count<=degree || compact.size()!=count+degree-1) return false;
    const double start=compact[degree-1],end=compact[count-1];
    if (!std::isfinite(start) || !std::isfinite(end) || !(start<end)) return false;
    for (size_t i=0;i<compact.size();++i)
        if (!std::isfinite(compact[i]) || (i && compact[i]<compact[i-1])) return false;
    for (size_t i=0;i<degree;++i)
        if (compact[i]!=start || compact[compact.size()-1-i]!=end) return false;
    return true;
}

/// Prove every exact original-point preimage lies at a clamped raw endpoint.
inline bool endpoint_preimages_only(const NurbsCurve& curve,const std::array<double,4>& witness) {
    if (!exact_raw_clamped(curve) || curve.dimension()!=3 ||
        !std::isfinite(witness[3]) || !(witness[3]>0)) return false;
    for (int d=0;d<3;++d)
        if (!std::isfinite(witness[d])) return false;
    const size_t degree=static_cast<size_t>(curve.degree()),count=static_cast<size_t>(curve.cv_count());
    const auto& compact=curve.m_nurbsknot;
    const double start=compact[degree-1],end=compact[count-1];
    for (size_t i=0;i<compact.size();) {
        size_t j=i+1;
        while (j<compact.size() && compact[j]==compact[i]) ++j;
        if (compact[i]>start && compact[i]<end && j-i>degree) return false;
        i=j;
    }
    std::vector<std::array<int,3>> signs(count);
    for (size_t i=0;i<count;++i) {
        const double* cv=curve.cv(static_cast<int>(i));
        const double weight=curve.is_rational()?cv[3]:1.;
        if (!std::isfinite(weight) || !(weight>0)) return false;
        for (int d=0;d<3;++d) {
            if (!std::isfinite(cv[d])) return false;
            const double sign=exact_orientation::orient2d(0.,0.,witness[3],witness[d],weight,cv[d]);
            signs[i][d]=sign>0?1:sign<0?-1:0;
        }
    }
    std::vector<double> knots=compact;
    knots.insert(knots.begin(),compact.front());
    knots.push_back(compact.back());
    for (size_t span=degree;span<count;++span) {
        const double lo=knots[span],hi=knots[span+1];
        if (!(lo<hi)) continue;
        if (!endpoint_active_signs_exclude(signs,knots,span-degree,span,degree,lo,hi)) return false;
        if (lo>start && !endpoint_active_signs_exclude(signs,knots,span-degree,span,degree,lo,lo))
            return false;
    }
    return true;
}
} // namespace session_cpp::brep_section::detail
