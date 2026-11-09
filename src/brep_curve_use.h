#pragma once
#include "brep.h"
#include "nurbs_coincidence.h"
#include "nurbs_interval.h"
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace session_cpp::brep_curve_use {
inline bool valid_domain(const NurbsCurve& source,const std::pair<double,double>& range) {
    if (!source.is_valid() || !std::isfinite(range.first) || !std::isfinite(range.second) || !(range.first<range.second)) return false;
    const auto full=source.domain();
    return range.first>=full.first && range.second<=full.second;
}
inline std::pair<double,double> domain(const NurbsCurve& source,const std::optional<BRepCurveUseRange>& range) {
    if (!source.is_valid()) throw std::invalid_argument("Curve use has an invalid original curve");
    const auto value=range?std::pair{range->first,range->last}:source.domain();
    if (!valid_domain(source,value)) throw std::invalid_argument("Curve use range must be positive and inside its original source");
    return value;
}
// First tranche: exact normalized endpoint correspondence, never a rounded mapped singleton.
inline bool exact_correspondence(const NurbsCurve& world,const std::pair<double,double>& wr,
                                 const NurbsCurve& pc,const std::pair<double,double>& pr) {
    if (!valid_domain(world,wr) || !valid_domain(pc,pr)) return false;
    const auto wd=world.domain(),pd=pc.domain();
    if (wd==pd && wr==pr) return true;
    const auto endpoint=[&](double w,double p) {
        if ((w==wd.first && p==pd.first) || (w==wd.second && p==pd.second)) return true;
        double a,b,c,d;
        return nurbs_coincidence::exact_difference(w,wd.first,a) &&
            nurbs_coincidence::exact_difference(wd.second,wd.first,b) && b>0 &&
            nurbs_coincidence::exact_difference(p,pd.first,c) &&
            nurbs_coincidence::exact_difference(pd.second,pd.first,d) && d>0 &&
            std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(d) &&
            nurbs_coincidence::exact_products_equal(a,d,c,b);
    };
    return endpoint(wr.first,pr.first) && endpoint(wr.second,pr.second);
}
// Borrowed readonly view. Owning const BRep/original curve must outlive the view.
// It is not an independent NURBS and never changes/extracts poles or knot domains.
class CurveUse {
    const NurbsCurve* source_;
    std::pair<double,double> range_;
    void check(double t) const {
        if (!std::isfinite(t) || t<range_.first || t>range_.second)
            throw std::out_of_range("Parameter lies outside the active original curve use");
    }
public:
    CurveUse(const NurbsCurve& source,std::pair<double,double> range):source_(&source),range_(range) {
        if (!valid_domain(source,range)) throw std::invalid_argument("Invalid original curve use");
    }
    const NurbsCurve& original_curve() const { return *source_; }
    std::pair<double,double> domain() const { return range_; }
    bool full_domain() const { return range_==source_->domain(); }
    Point point_at(double t) const { check(t);return source_->point_at(t); }
    Point point_at_start() const { return point_at(range_.first); }
    Point point_at_end() const { return point_at(range_.second); }
    std::vector<Vector> evaluate(double t,int derivatives=2) const {
        check(t);if (derivatives<0 || derivatives>2) throw std::invalid_argument("Curve use exposes D0/D1/D2");
        return source_->evaluate(t,derivatives);
    }
    CurveUse subrange(double first,double last) const {
        if (first<range_.first || last>range_.second) throw std::out_of_range("Subrange enlarges an active curve use");
        return CurveUse(*source_,{first,last});
    }
    bool bounds(const std::array<double,2>& interval,const std::array<double,3>& origin,nurbs_interval::CurveBounds& result) const {
        if (!std::isfinite(interval[0]) || !std::isfinite(interval[1]) || interval[0]>interval[1] ||
            interval[0]<range_.first || interval[1]>range_.second) return false;
        return nurbs_interval::curve_bounds(*source_,interval,origin,result);
    }
    bool bounds(const std::array<double,3>& origin,nurbs_interval::CurveBounds& result) const {
        return bounds({range_.first,range_.second},origin,result);
    }
};
inline bool has_restricted_uses(const BRep& owner) { return owner.has_restricted_curve_uses(); }
inline CurveUse world(const BRep& owner,int edge) {
    const auto range=owner.edge_domain(edge);
    return CurveUse(owner.m_curves_3d.at(owner.m_edges.at(edge).curve_3d_index),range);
}
inline CurveUse pcurve(const BRep& owner,int edge,int face,BRepOrientation orientation=BRepOrientation::Forward) {
    const auto range=owner.pcurve_domain(edge,face,orientation);
    const int index=owner.pcurve_index(edge,face,orientation);
    return CurveUse(owner.m_curves_2d.at(index),range);
}
} // namespace session_cpp::brep_curve_use
