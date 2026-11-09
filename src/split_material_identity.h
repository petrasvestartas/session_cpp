#pragma once
#include "nurbs_coincidence.h"
#include <utility>
#include <cmath>
namespace session_cpp::split::detail {
inline bool exact_material_raw_alias(std::pair<double,double> uv,std::pair<double,double> world,
    double parameter,double mapped) {
    if (!std::isfinite(parameter) || !std::isfinite(mapped) ||
        !(uv.first<uv.second) || !(world.first<world.second))
        return false;
    if (parameter==uv.first) return mapped==world.first;
    if (parameter==uv.second) return mapped==world.second;
    if (!(parameter>uv.first && parameter<uv.second && mapped>world.first && mapped<world.second))
        return false;
    double uwidth,wwidth,du,dw;
    return nurbs_coincidence::exact_difference(uv.second,uv.first,uwidth) &&
        nurbs_coincidence::exact_difference(world.second,world.first,wwidth) &&
        nurbs_coincidence::exact_difference(parameter,uv.first,du) &&
        nurbs_coincidence::exact_difference(mapped,world.first,dw) &&
        nurbs_coincidence::exact_products_equal(dw,uwidth,du,wwidth);
}
} // namespace session_cpp::split::detail
