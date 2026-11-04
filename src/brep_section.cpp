#include "brep_section.h"
#include "brep_classify.h"
#include "brep_face_boundary.h"
#include "brep_interference_internal.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "intersection.h"
#include "nurbs_bounds.h"
#include "nurbs_coincidence.h"
#include "nurbs_interval.h"
#include <map>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace session_cpp::brep_section {
namespace {

struct Bounds {
    Point lo;
    Point hi;

    explicit Bounds(const NurbsSurface& surface) : lo(surface.get_cv(0, 0)), hi(lo) {
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j) {
                const Point p = surface.get_cv(i, j);
                if (!(surface.weight(i, j) > 0.0))
                    throw std::invalid_argument("Native BRep sections require positive rational weights");
                for (int d = 0; d < 3; ++d) {
                    lo[d] = std::min(lo[d], p[d]);
                    hi[d] = std::max(hi[d], p[d]);
                }
            }
    }

    bool overlaps(const Bounds& other, double tolerance) const {
        for (int d = 0; d < 3; ++d)
            if (hi[d] + tolerance < other.lo[d] || other.hi[d] + tolerance < lo[d])
                return false;
        return true;
    }
};

double uv_tolerance(const NurbsSurface& surface, double tolerance) {
    return tolerance / nurbs_bounds::surface_metric(surface);
}

std::vector<int> active_faces(const BRep& brep) {
    std::vector<int> faces;
    if (brep.m_solids.empty()) {
        for (int i = 0; i < brep.face_count(); ++i)
            faces.push_back(i);
        return faces;
    }
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces)
                if (const BRepOrientation orientation = brep_compose(shell.orientation, face.orientation);
                    orientation == BRepOrientation::Forward || orientation == BRepOrientation::Reversed)
                    faces.push_back(face.index);
    std::sort(faces.begin(), faces.end());
    faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
    return faces;
}

// A shared exact UV translation changes neither metric nor raw curve parameters.
bool translate_parameters_exact(NurbsCurve& curve, const Point& origin) {
    if (curve.dimension() < 1 || curve.dimension() > 3)
        return false;
    for (int i = 0; i < curve.cv_count(); ++i) {
        double* cv = curve.cv(i);
        const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
        for (int d = 0; d < curve.dimension(); ++d) {
            const double product = origin[d] * weight;
            double local;
            if (!nurbs_coincidence::exact_products_equal(origin[d], weight, product, 1.0)
                || !nurbs_coincidence::exact_difference(cv[d], product, local))
                return false;
            cv[d] = local;
        }
    }
    return true;
}

void trim_parameters(
    const BRep& brep,
    int face,
    const NurbsCurve& pcurve,
    double tolerance,
    std::vector<double>& parameters,
    std::vector<OriginalTrimEvent>* receipts = nullptr,
    int operand = -1
) {
    for (const BRepRef& wire : brep.m_faces[face].wires) {
        const auto uses = brep.wire_edges(wire);
        for (size_t occurrence = 0; occurrence < uses.size(); ++occurrence) {
            const auto& edge = uses[occurrence];
            const int index = brep.pcurve_index(edge.index, face, edge.orientation);
            if (index < 0)
                throw std::runtime_error("Native section encountered a missing face pcurve");
            const NurbsCurve& boundary = brep.m_curves_2d[index];
            NurbsCurve local_section = pcurve, local_boundary = boundary;
            const Point origin = boundary.get_cv(0);
            const bool exact = translate_parameters_exact(local_section, origin)
                && translate_parameters_exact(local_boundary, origin);
            // Only a/b parameters escape this call; all original UV curves stay unchanged.
            const curve_intersection::Result events = exact
                ? curve_intersection::intersect(local_section, local_boundary, tolerance)
                : curve_intersection::intersect(pcurve, boundary, tolerance);
            if (!events.complete) {
                if (const char* path = std::getenv("SESSION_SECTION_CC_FAILURE")) {
                    try {
                        std::ofstream output(path, std::ios::app);
                        if (!output)
                            throw std::runtime_error("cannot open capture file");
                        nlohmann::ordered_json saved;
                        saved["failure"] = events.failure;
                        saved["face"] = face;
                        saved["boundary_edge"] = edge.index;
                        saved["uv_tolerance"] = tolerance;
                        saved["section_pcurve"] = pcurve.jsondump();
                        saved["boundary_pcurve"] = brep.m_curves_2d[index].jsondump();
                        output << saved.dump() << '\n';
                        if (!output)
                            throw std::runtime_error("cannot write capture record");
                        output.flush();
                        if (!output)
                            throw std::runtime_error("cannot flush capture file");
                        output.close();
                        if (output.fail())
                            throw std::runtime_error("cannot close capture file");
                    } catch (const std::exception& error) {
                        std::cerr << "Section CC failure capture failed: " << error.what() << '\n';
                    } catch (...) {
                        std::cerr << "Section CC failure capture failed: unknown error\n";
                    }
                }
                throw std::runtime_error(events.failure);
            }
            for (const curve_intersection::Hit& point : events.points) {
                parameters.push_back(point.a);
                if (receipts)
                    receipts->push_back({wire.index,static_cast<int>(occurrence),operand,face,edge.index,index,edge.orientation,point.a,point.b,false});
            }
            for (const curve_intersection::Overlap& overlap : events.overlaps) {
                parameters.push_back(overlap.a0);
                parameters.push_back(overlap.a1);
                if (receipts) {
                    receipts->push_back({wire.index,static_cast<int>(occurrence),operand,face,edge.index,index,edge.orientation,overlap.a0,overlap.b0,true});
                    receipts->push_back({wire.index,static_cast<int>(occurrence),operand,face,edge.index,index,edge.orientation,overlap.a1,overlap.b1,true});
                }
            }
        }
    }
}

void sort_parameters(std::vector<double>& parameters) {
    std::sort(parameters.begin(), parameters.end());
    const double scale = std::max({std::abs(parameters.front()), std::abs(parameters.back()),
                                  parameters.back() - parameters.front()});
    const double epsilon = 32.0 * std::numeric_limits<double>::epsilon() * scale;
    parameters.erase(std::unique(parameters.begin(), parameters.end(), [epsilon](double a, double b) {
        return b - a <= epsilon;
    }), parameters.end());
}

bool retained(const BRep& brep, int face, const NurbsCurve& pcurve, double t, double tolerance) {
    const brep_classify::State state = brep_classify::face_uv(brep, face, pcurve.point_at(t), tolerance);
    if (state == brep_classify::State::Unknown)
        throw std::runtime_error("Native section could not classify a trimming interval");
    return state != brep_classify::State::Outside;
}

NurbsCurve trimmed(const NurbsCurve& curve, double first, double last) {
    NurbsCurve result = curve;
    if (!result.trim(first, last)) {
        if (const char* path = std::getenv("SESSION_SECTION_TRIM_FAILURE"); path && *path) {
            try {
                std::ofstream output;
                output.exceptions(std::ios::failbit | std::ios::badbit);
                output.open(path, std::ios::app);
                output << nlohmann::ordered_json{{"first",first},{"last",last},
                    {"domain",curve.domain()},{"curve",curve.jsondump()}}.dump() << '\n';
                output.flush();
                output.close();
            } catch (const std::exception& error) {
                std::cerr << "Section trim failure capture failed: " << error.what() << '\n';
            }
        }
        throw std::runtime_error("Native section could not trim a curve interval");
    }
    return result;
}

void append_overlap(
    CurveResult& result,
    const BRep& brep,
    int face,
    const curve_surface_intersection::Overlap& overlap,
    double tolerance
) {
    if (!overlap.pcurve.is_valid())
        throw std::runtime_error("Native curve/face overlap has no parameter-space curve");
    std::vector<double> parameters = {overlap.t0, overlap.t1};
    trim_parameters(brep, face, overlap.pcurve, tolerance, parameters);
    sort_parameters(parameters);
    for (size_t i = 1; i < parameters.size(); ++i)
        if (retained(brep, face, overlap.pcurve, std::midpoint(parameters[i - 1], parameters[i]), tolerance))
            result.overlaps.push_back({face, parameters[i - 1], parameters[i],
                trimmed(overlap.pcurve, parameters[i - 1], parameters[i])});
}

void check_input(const BRep& brep, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !brep.is_valid())
        throw std::invalid_argument("Native BRep sections require valid geometry and finite positive tolerance");
}

bool unsupported_parameterization(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    for (int variant = 0; variant < 8; ++variant) {
        NurbsSurface candidate = b;
        if ((variant & 4) && !candidate.transpose())
            continue;
        if ((variant & 1) && !candidate.reverse(0))
            continue;
        if ((variant & 2) && !candidate.reverse(1))
            continue;
        bool valid = true;
        for (int d = 0; d < 2; ++d) {
            const std::pair<double, double> domain = a.domain(d);
            valid = candidate.set_domain(d, domain.first, domain.second) && valid;
        }
        if (valid && nurbs_coincidence::same_parameters(a, candidate, tolerance))
            return true;
    }
    return false;
}

}

CurveResult curve(const NurbsCurve& input, const BRep& brep, double tolerance) {
    check_input(brep, tolerance);
    CurveResult result;
    try {
        for (int face : active_faces(brep)) {
            const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
            const double uv_tol = uv_tolerance(surface, tolerance);
            const curve_surface_intersection::Result events = curve_surface_intersection::intersect(input, surface, tolerance);
            if (!events.complete)
                throw std::runtime_error(events.failure);
            for (const curve_surface_intersection::Hit& hit : events.points) {
                const brep_classify::State state = brep_classify::face_uv(brep, face, {hit.u, hit.v, 0}, uv_tol);
                if (state == brep_classify::State::Unknown)
                    throw std::runtime_error("Native curve/face hit has unresolved trim classification");
                if (state != brep_classify::State::Outside)
                    result.points.push_back({hit.point, face, hit.t, hit.u, hit.v, hit.tangent});
            }
            for (const curve_surface_intersection::Overlap& overlap : events.overlaps)
                append_overlap(result, brep, face, overlap, uv_tol);
        }
        std::sort(result.points.begin(), result.points.end(), [](const CurveHit& a, const CurveHit& b) { return a.t < b.t; });
    } catch (const std::runtime_error& error) {
        result.complete = false;
        result.failure = error.what();
    }
    return result;
}

FaceResult faces(const BRep& first, const BRep& second, double tolerance) {
    check_input(first, tolerance);
    check_input(second, tolerance);
    FaceResult result;
    int first_face = -1;
    int second_face = -1;
    try {
        for (int a : active_faces(first)) {
            first_face = a;
            const NurbsSurface& sa = first.m_surfaces[first.m_faces[a].surface_index];
            const Bounds box(sa);
            const double ta = uv_tolerance(sa, tolerance);
            for (int b : active_faces(second)) {
                second_face = b;
                const NurbsSurface& sb = second.m_surfaces[second.m_faces[b].surface_index];
                if (!box.overlaps(Bounds(sb), tolerance))
                    continue;
                nurbs_coincidence::ParameterMap mapping;
                if (nurbs_coincidence::same_parameters(sa, sb, tolerance)
                    || nurbs_coincidence::parameter_mapping(sa, sb, tolerance, mapping)
                    || nurbs_coincidence::common_basis_agreement(sa, sb, tolerance)
                    || nurbs_coincidence::coplanar(sa, sb, tolerance)) {
                    result.overlaps.push_back({a, b});
                    continue;
                }
                if (unsupported_parameterization(sa, sb, tolerance))
                    throw std::runtime_error("Native section has coincident supports with unsupported parameter mapping");
                const double tb = uv_tolerance(sb, tolerance);
                for (const auto& section : Intersection::surface_surface(sa, sb, tolerance)) {
                    const NurbsCurve& world = std::get<0>(section);
                    const NurbsCurve& ua = std::get<1>(section);
                    const NurbsCurve& ub = std::get<2>(section);
                    if (world.domain() != ua.domain() || world.domain() != ub.domain())
                        throw std::runtime_error("Native section pcurves have mismatched parameter domains");
                    std::vector<double> parameters = {world.domain_start(), world.domain_end()};
                    trim_parameters(first, a, ua, ta, parameters);
                    trim_parameters(second, b, ub, tb, parameters);
                    sort_parameters(parameters);
                    size_t run_begin = 0;
                    bool retaining = false;
                    for (size_t i = 1; i < parameters.size(); ++i) {
                        const double middle = std::midpoint(parameters[i - 1], parameters[i]);
                        const bool keep = retained(first, a, ua, middle, ta) && retained(second, b, ub, middle, tb);
                        if (keep && !retaining) {
                            run_begin = i - 1;
                            retaining = true;
                        }
                        if (retaining && (!keep || i + 1 == parameters.size())) {
                            const size_t run_end = keep ? i : i - 1;
                            result.curves.push_back({a, b, trimmed(world, parameters[run_begin], parameters[run_end]),
                                trimmed(ua, parameters[run_begin], parameters[run_end]),
                                trimmed(ub, parameters[run_begin], parameters[run_end])});
                            retaining = false;
                        }
                    }
                }
            }
        }
    } catch (const std::runtime_error& error) {
        result.complete = false;
        result.failure = std::string(error.what()) + "; face pair " + std::to_string(first_face)
            + "," + std::to_string(second_face);
    }
    return result;
}


// Inclusion is existential. These parameter proposals are certified images of
// the exact point by the preimage domain proof, not sampled geometry agreement.
static surface_point_preimages::Membership
point_trim_membership(const BRep& brep, int face, const surface_point_preimages::Result& preimages,
                      double tolerance, Point& included) {
    using Membership = surface_point_preimages::Membership;
    return surface_point_preimages::trimmed_membership(preimages, [&](const auto& domain) {
        const auto& uv = domain.uv;
        const bool singleton = uv[0] == uv[1] && uv[2] == uv[3];
        for (double u : {0., .25, .5, .75, 1.})
            for (double v : {0., .25, .5, .75, 1.}) {
                const Point point(std::clamp(std::lerp(uv[0], uv[1], u), uv[0], uv[1]),
                                  std::clamp(std::lerp(uv[2], uv[3], v), uv[2], uv[3]), 0);
                const auto state = brep_classify::face_uv(brep, face, point, tolerance);
                if (state == brep_classify::State::Inside || state == brep_classify::State::Boundary) {
                    included = point;
                    return Membership::Included;
                }
                if (singleton)
                    return state == brep_classify::State::Outside ? Membership::Excluded
                                                                  : Membership::Unresolved;
            }
        // No finite collection of excluded samples proves an edge/cell excluded.
        return Membership::Unresolved;
    });
}


namespace {
using I = nurbs_interval::Interval;
using Membership = surface_point_preimages::Membership;

// The distance between two parameters NurbsCurve::trim can not tell apart.
double trim_resolution(const NurbsCurve& curve) {
    const double first = curve.domain_start(), last = curve.domain_end();
    return 4.0 * std::numeric_limits<double>::epsilon() * std::max({std::abs(first), std::abs(last), last - first});
}

double snapped_to_knot(double value, const NurbsCurve& curve, double resolution) {
    for (double knot : curve.m_nurbsknot)
        if (std::abs(value - knot) <= resolution)
            return knot;
    return value;
}

// A cut nearer a knot or another cut than NurbsCurve::trim can tell apart is that knot or that cut.
void exact_cuts(std::vector<double>& cuts, const std::vector<const NurbsCurve*>& originals) {
    const NurbsCurve& original = *originals.front();
    const double resolution = trim_resolution(original);
    for (double& value : cuts) {
        if (!std::isfinite(value) || value < original.domain_start() || value > original.domain_end())
            throw std::runtime_error("Interference has an invalid original raw cut");
        for (const NurbsCurve* curve : originals)
            value = snapped_to_knot(value, *curve, resolution);
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(), [resolution](double a, double b) {
        return b - a <= resolution;
    }), cuts.end());
}

NurbsCurve exact_trim(const NurbsCurve& curve, double lo, double hi) {
    if (!(lo < hi))
        throw std::runtime_error("Interference requires a positive original raw interval");
    NurbsCurve result = trimmed(curve, lo, hi);
    if (result.domain_start() != lo || result.domain_end() != hi)
        throw std::runtime_error("Interference trim lost a required original raw event");
    return result;
}

bool original_hull(const NurbsSurface& surface, std::array<I, 3>& box) {
    if (surface.dimension()!=3)
        return false;
    bool first = true;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const double weight = surface.weight(i,j);
            if (!std::isfinite(weight) || !(weight > 0))
                return false;
            for (int d = 0; d < 3; ++d) {
                const I coordinate = I(surface.cv(i,j)[d]) / I(weight);
                if (!nurbs_interval::finite(coordinate))
                    return false;
                box[d] = first ? coordinate : I(std::min(box[d].lo, coordinate.lo),
                                                std::max(box[d].hi, coordinate.hi));
            }
            first = false;
        }
    return !first;
}

bool excluded(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    std::array<I,3> x,y;
    if (!original_hull(a,x) || !original_hull(b,y))
        return false;
    for (int d=0;d<3;++d) {
        const I gap=x[d]-y[d];
        if (gap.lo>tolerance || gap.hi< -tolerance)
            return true;
    }
    return false;
}

bool projective_cv(const NurbsCurve& curve, int index,
                   const surface_point_preimages::ProjectivePoint& point) {
    if (curve.dimension()!=3)
        return false;
    const double weight=curve.weight(index);
    if (!std::isfinite(weight) || !(weight>0))
        return false;
    for (int d=0;d<3;++d)
        if (!nurbs_coincidence::exact_products_equal(curve.cv(index)[d],point[3],point[d],weight))
            return false;
    return true;
}

bool same_projective(const surface_point_preimages::ProjectivePoint& a,
                     const surface_point_preimages::ProjectivePoint& b) {
    for (int d=0;d<3;++d)
        if (!nurbs_coincidence::exact_products_equal(a[d],b[3],b[d],a[3]))
            return false;
    return true;
}

std::map<int,std::vector<EdgeUse>> edge_uses(const BRep& brep,int face) {
    const auto boundary=brep_face_boundary::extract(brep,face);
    if (!boundary.complete)
        throw std::runtime_error(boundary.failure);
    std::map<int,std::vector<EdgeUse>> result;
    for (const auto& wire:boundary.wires)
        for (size_t i=0;i<wire.uses.size();++i) {
            const auto& use=wire.uses[i];
            if (!use.degenerated)
                result[use.edge.index].push_back({face,wire.reference.index,static_cast<int>(i),
                    use.pcurve,use.edge.orientation,wire.reference.orientation});
        }
    return result;
}

// The compatibility classifier has no admitted material semantics for I/E uses.
// Reject explicitly before reading their possibly absent pcurve; do not erase them.
void validate_boundary_policy(const BRep& owner,int face,int operand) {
    const auto boundary=brep_face_boundary::extract(owner,face);
    if (!boundary.complete)
        throw std::runtime_error(boundary.failure);
    for (const auto& wire:boundary.wires)
        for (size_t i=0;i<wire.uses.size();++i) {
            const auto& use=wire.uses[i];
            const EdgeUse identity{face,wire.reference.index,static_cast<int>(i),use.pcurve,
                use.edge.orientation,wire.reference.orientation};
            if (!brep_face_boundary::detail::active(wire.reference.orientation) ||
                !brep_face_boundary::detail::active(use.edge.orientation) || use.pcurve<0)
                throw InventoryError("Interference inventory does not admit Internal/External boundary uses",
                    {FailureKind::InactiveBoundaryUse,operand,use.edge.index,identity});
        }
}

void paired(const FaceCurve& curve,const BRep& a,const BRep& b,double tolerance) {
    for (int side=0;side<2;++side) {
        const auto& owner=side?b:a;
        const int face=side?curve.second_face:curve.first_face;
        const auto& pc=side?curve.second_pcurve:curve.first_pcurve;
        if (curve.world.domain()!=pc.domain() ||
            curve_surface_intersection::same_parameter(curve.world,pc,
                owner.m_surfaces[owner.m_faces[face].surface_index],tolerance)
                !=curve_surface_intersection::ParameterMatch::Matched)
            throw std::runtime_error("Interference lacks whole original paired agreement");
    }
}

void support_curves(FaceResult& result,int a,int b,const BRep& first,const BRep& second,
                    const surface_section::Result& events,double tolerance) {
    const double ta=uv_tolerance(first.m_surfaces[first.m_faces[a].surface_index],tolerance);
    const double tb=uv_tolerance(second.m_surfaces[second.m_faces[b].surface_index],tolerance);
    for (const auto& [world,ua,ub]:events.curves) {
        if (world.domain()!=ua.domain() || world.domain()!=ub.domain())
            throw std::runtime_error("Interference support law has different raw domains");
        std::vector<double> cuts{world.domain_start(),world.domain_end()};
        OriginalTrimLaw law{{a,b,world,ua,ub},{}};
        trim_parameters(first,a,ua,ta,cuts,&law.events,0);
        trim_parameters(second,b,ub,tb,cuts,&law.events,1);
        result.original_trim_laws.push_back(std::move(law));
        exact_cuts(cuts,{&world,&ua,&ub});
        for (size_t i=1;i<cuts.size();++i) {
            const double mid=std::midpoint(cuts[i-1],cuts[i]);
            if (!(cuts[i-1]<mid && mid<cuts[i]))
                throw std::runtime_error("Interference trim interval has no interior representative");
            if (!retained(first,a,ua,mid,ta) || !retained(second,b,ub,mid,tb))
                continue;
            FaceCurve curve{a,b,exact_trim(world,cuts[i-1],cuts[i]),
                exact_trim(ua,cuts[i-1],cuts[i]),exact_trim(ub,cuts[i-1],cuts[i])};
            paired(curve,first,second,tolerance);
            result.curves.push_back(std::move(curve));
        }
    }
}

// Failure-only original-input witness; no additional geometry queries.
void observe_boundary_failure(const BRep& source, const NurbsSurface& target, int first_face,
    int second_face, int operand, int edge_index, const std::vector<EdgeUse>& uses,
    double tolerance, const std::string& failure) {
    const char* path = std::getenv("SESSION_SECTION_BOUNDARY_FAILURE");
    if (!path || !*path)
        return;
    try {
        const auto& edge = source.m_edges[edge_index];
        const int own_face = operand ? second_face : first_face;
        const auto& world = source.m_curves_3d[edge.curve_3d_index];
        nlohmann::ordered_json selected = nlohmann::ordered_json::array();
        for (const auto& use : uses)
            selected.push_back({{"face",use.face},{"wire",use.wire},{"occurrence",use.occurrence},
                {"pcurve_index",use.pcurve},{"orientation",static_cast<int>(use.orientation)},
                {"wire_orientation",static_cast<int>(use.wire_orientation)},
                {"pcurve",source.m_curves_2d.at(use.pcurve).jsondump()}});
        nlohmann::ordered_json aliases = nlohmann::ordered_json::array();
        for (const auto& association : edge.pcurves) {
            nlohmann::ordered_json row{{"surface_index",association.surface_index},
                {"primary_index",association.curve_2d_index},{"secondary_index",association.curve_2d_index_2}};
            for (const auto& entry : {std::pair{"primary",association.curve_2d_index},
                                    std::pair{"secondary",association.curve_2d_index_2}})
                if (entry.second >= 0)
                    row[entry.first] = source.m_curves_2d.at(entry.second).jsondump();
            aliases.push_back(std::move(row));
        }
        const nlohmann::ordered_json row{{"first_face",first_face},{"second_face",second_face},
            {"operand",operand},{"source_face",own_face},{"edge",edge_index},
            {"start_vertex",edge.start_vertex},{"end_vertex",edge.end_vertex},
            {"world",world.jsondump()},{"world_domain",world.domain()},
            {"selected_uses",selected},{"aliases",aliases},
            {"source_surface",source.m_surfaces[source.m_faces[own_face].surface_index].jsondump()},
            {"target_surface",target.jsondump()},{"tolerance",tolerance},{"failure",failure}};
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(path, std::ios::app);
        output << row.dump() << '\n';
        output.flush();
        output.close();
    } catch (const std::exception& error) {
        std::cerr << "Boundary intersection failure capture failed: " << error.what() << '\n';
    }
}

bool pcurve_inside_support(const NurbsCurve& curve, const NurbsSurface& surface) {
    for (int i = 0; i < curve.cv_count(); ++i) {
        const double weight = curve.weight(i);
        if (!std::isfinite(weight) || !(weight > 0))
            return false;
        for (int d = 0; d < 2; ++d) {
            const double value = curve.cv(i)[d];
            const auto domain = surface.domain(d);
            if (!std::isfinite(value))
                return false;
            const auto lower = nurbs_interval::Interval(value) - nurbs_interval::Interval(domain.first) * nurbs_interval::Interval(weight);
            const auto upper = nurbs_interval::Interval(value) - nurbs_interval::Interval(domain.second) * nurbs_interval::Interval(weight);
            if ((!nurbs_coincidence::exact_products_equal(value,1,domain.first,weight) &&
                    (!nurbs_interval::finite(lower) || lower.lo < 0)) ||
                (!nurbs_coincidence::exact_products_equal(value,1,domain.second,weight) &&
                    (!nurbs_interval::finite(upper) || upper.hi > 0)))
                return false;
        }
    }
    return true;
}

// Reuse original owning laws before rediscovering an on-support intersection.
// Surface correspondence only proposes coordinates; both original whole laws must certify.
bool original_boundary_overlap(curve_surface_intersection::Result& result, const BRep& owner,
    int face, const NurbsCurve& world, const std::vector<EdgeUse>& uses,
    const NurbsSurface& target, double tolerance) {
    const auto& source = owner.m_surfaces[owner.m_faces[face].surface_index];
    nurbs_coincidence::ParameterMap mapping;
    const bool mapped = nurbs_coincidence::parameter_mapping(source,target,tolerance,mapping);
    if (!mapped && !nurbs_coincidence::common_basis_agreement(source,target,tolerance))
        return false;
    for (const auto& use : uses) {
        if (use.pcurve < 0 || use.pcurve >= static_cast<int>(owner.m_curves_2d.size()))
            return false;
        auto original = owner.m_curves_2d[use.pcurve];
        if (!original.set_domain(world.domain_start(),world.domain_end()) ||
            curve_surface_intersection::same_parameter(world,original,source,tolerance) !=
                curve_surface_intersection::ParameterMatch::Matched)
            return false;
        auto proposed = original;
        if (mapped) {
            bool finite = true;
            for (int i = 0; i < proposed.cv_count(); ++i) {
                const double weight = original.weight(i);
                const double u = mapping.scale[0] * original.cv(i)[mapping.axis[0]] + mapping.offset[0] * weight;
                const double v = mapping.scale[1] * original.cv(i)[mapping.axis[1]] + mapping.offset[1] * weight;
                finite = finite && std::isfinite(u) && std::isfinite(v) && std::isfinite(weight) && weight > 0 &&
                    proposed.set_cv_4d(i,u,v,0,weight);
            }
            if (!finite)
                return false;
        }
        if (proposed.domain() == world.domain() && pcurve_inside_support(proposed,target) &&
            curve_surface_intersection::same_parameter(world,proposed,target,tolerance) ==
                curve_surface_intersection::ParameterMatch::Matched) {
            result.overlaps.push_back({world.domain_start(),world.domain_end(),std::move(proposed)});
        } else {
            return false; // Never accept only a subset of original selected aliases.
        }
    }
    return !result.overlaps.empty();
}

// This pass preserves every original edge use; it does not choose one owning face.
void boundary_pair(FaceResult& result,const BRep& first,const BRep& second,int a,int b,
                   int operand,double tolerance) {
    const BRep& source=operand?second:first;
    const BRep& other=operand?first:second;
    const int own_face=operand?b:a, other_face=operand?a:b;
    const auto& surface=other.m_surfaces[other.m_faces[other_face].surface_index];
    const double uv_tol=uv_tolerance(surface,tolerance);
    for (const auto& [edge_index,uses]:edge_uses(source,own_face)) {
        const auto& edge=source.m_edges[edge_index];
        const auto& world=source.m_curves_3d[edge.curve_3d_index];
        curve_surface_intersection::Result events;
        if (!original_boundary_overlap(events,source,own_face,world,uses,surface,tolerance))
            events=curve_surface_intersection::intersect(world,surface,tolerance);
        if (!events.complete) {
            observe_boundary_failure(source,surface,a,b,operand,edge_index,uses,tolerance,events.failure);
            throw std::runtime_error(events.failure);
        }
        const auto append_point=[&](double t,const Point& uv,bool tangent) {
            if (!std::isfinite(t) || t<world.domain_start() || t>world.domain_end() ||
                !std::isfinite(uv[0]) || !std::isfinite(uv[1]))
                throw std::runtime_error("Boundary event has invalid original raw coordinates");
            // One point of the edge met from several faces is one raw event.
            const double resolution=trim_resolution(world);
            t=snapped_to_knot(t,world,resolution);
            for (const auto& previous:result.boundary_events)
                if (previous.operand==operand && previous.edge==edge_index && std::abs(previous.hit.t-t)<=resolution)
                    t=previous.hit.t;
            const auto state=brep_classify::face_uv(other,other_face,uv,uv_tol);
            if (state==brep_classify::State::Unknown)
                throw std::runtime_error("Interference boundary trim ownership is unresolved");
            if (state==brep_classify::State::Outside)
                return;
            const int vertex=t==world.domain_start()?edge.start_vertex:
                t==world.domain_end()?edge.end_vertex:-1;
            result.boundary_events.push_back({operand,edge_index,vertex,
                {world.point_at(t),other_face,t,uv[0],uv[1],tangent},uses,{},true});
        };
        for (const auto& hit:events.points)
            append_point(hit.t,{hit.u,hit.v,0},hit.tangent);
        for (const auto& overlap:events.overlaps) {
            if (!overlap.pcurve.is_valid())
                throw std::runtime_error("Interference boundary overlap lacks its original law");
            std::vector<double> cuts{overlap.t0,overlap.t1};
            trim_parameters(other,other_face,overlap.pcurve,uv_tol,cuts);
            exact_cuts(cuts,{&world,&overlap.pcurve});
            std::vector<bool> kept(cuts.size()-1,false);
            for (size_t i=1;i<cuts.size();++i) {
                const double mid=std::midpoint(cuts[i-1],cuts[i]);
                if (!(cuts[i-1]<mid && mid<cuts[i]))
                    throw std::runtime_error("Boundary overlap has no interior raw representative");
                kept[i-1]=retained(other,other_face,overlap.pcurve,mid,uv_tol);
                if (!kept[i-1])
                    continue;
                for (const auto& use:uses) {
                    if (use.pcurve<0 || use.pcurve>=static_cast<int>(source.m_curves_2d.size()))
                        throw InventoryError("Boundary overlap has no admitted source pcurve",
                            {FailureKind::InactiveBoundaryUse,operand,edge_index,use});
                    auto own=source.m_curves_2d[use.pcurve];
                    if (!own.set_domain(world.domain_start(),world.domain_end()))
                        throw std::runtime_error("Boundary use cannot propose original normalized correspondence");
                    auto own_piece=exact_trim(own,cuts[i-1],cuts[i]);
                    auto other_piece=exact_trim(overlap.pcurve,cuts[i-1],cuts[i]);
                    FaceCurve curve{a,b,exact_trim(world,cuts[i-1],cuts[i]),
                        operand?other_piece:own_piece,operand?own_piece:other_piece};
                    paired(curve,first,second,tolerance);
                    result.curves.push_back(std::move(curve));
                }
            }
            for (size_t i=0;i<cuts.size();++i)
                append_point(cuts[i],overlap.pcurve.point_at(cuts[i]),true);
        }
    }
}

bool interval_event(const NurbsCurve& curve,double a,double b,const Point& point,double tolerance) {
    return a==b || nurbs_interval::curve_within_ball(curve,{std::min(a,b),std::max(a,b)},
        {point[0],point[1],point[2]},tolerance*.25);
}

bool curve_excludes(const NurbsCurve& curve,const Point& point,double tolerance) {
    return detail::curve_excludes_point(curve,point,tolerance);
}

const NurbsCurve& event_source(const BoundaryEvent& event,const BRep& a,const BRep& b) {
    const auto& owner=event.operand?b:a;
    return owner.m_curves_3d[owner.m_edges[event.edge].curve_3d_index];
}

bool exact_vertex(const Point& vertex,const surface_point_preimages::ProjectivePoint& witness) {
    if (!std::isfinite(witness[3]) || !(witness[3]>0))
        return false;
    for (int d=0;d<3;++d)
        if (!std::isfinite(vertex[d]) ||
            !nurbs_coincidence::exact_products_equal(vertex[d],witness[3],witness[d],1.))
            return false;
    return true;
}

bool excludes_vertex_ball(const Point& vertex,const surface_point_preimages::ProjectivePoint& point,I radius) {
    I lower;
    for (int d=0;d<3;++d) {
        const I gap=I(point[d])/I(point[3])-I(vertex[d]);
        if (!nurbs_interval::finite(gap))
            return false;
        const double distance=gap.lo>0?gap.lo:gap.hi<0?-gap.hi:0;
        lower=lower+I(distance)*I(distance);
    }
    const I allowed=radius*radius;
    return nurbs_interval::finite(lower) && nurbs_interval::finite(allowed) && lower.lo>allowed.hi;
}

void degenerate_point_uses(FaceResult& result,const BRep& owner,int face,int operand,
                           size_t point_index,double tolerance) {
    const auto& contact=result.support_points[point_index].event;
    const auto boundary=brep_face_boundary::extract(owner,face);
    if (!boundary.complete)
        throw std::runtime_error(boundary.failure);
    std::map<int,UVOnlyIncidence> related;
    for (const auto& wire:boundary.wires)
        for (size_t i=0;i<wire.uses.size();++i) {
            const auto& use=wire.uses[i];
            if (!use.degenerated)
                continue;
            const auto& edge=owner.m_edges[use.edge.index];
            const auto& vertex=owner.m_vertices[edge.start_vertex].point;
            const EdgeUse identity{face,wire.reference.index,static_cast<int>(i),use.pcurve,
                use.edge.orientation,wire.reference.orientation};
            const auto unresolved=[&]() {
                throw InventoryError("Original degenerate UV-only contact incidence is unresolved",
                    {FailureKind::UnrepresentedDegenerateUse,operand,use.edge.index,identity});
            };
            if (use.pcurve<0 || use.pcurve>=static_cast<int>(owner.m_curves_2d.size()))
                unresolved();
            const auto& pc=owner.m_curves_2d[use.pcurve];
            if (!(tolerance*.25>0) || !std::isfinite(tolerance*.25))
                unresolved();
            const bool same=exact_vertex(vertex,contact.exact_witness);
            NurbsCurve constant(3,true,2,2);
            constant.m_nurbsknot={pc.domain_start(),pc.domain_end()};
            for (int end=0;end<2;++end) {
                const auto h=same?contact.exact_witness:
                    surface_point_preimages::ProjectivePoint{vertex[0],vertex[1],vertex[2],1};
                if (!constant.set_cv_4d(end,h[0],h[1],h[2],h[3]))
                    unresolved();
            }
            if (curve_surface_intersection::same_parameter(constant,pc,
                owner.m_surfaces[owner.m_faces[face].surface_index],tolerance*.25)
                !=curve_surface_intersection::ParameterMatch::Matched)
                unresolved();
            if (!same) {
                // Whole original PC lies in the certified vertex ball, which excludes P.
                if (!excludes_vertex_ball(vertex,contact.exact_witness,I(tolerance)+I(tolerance*.25)))
                    unresolved();
                continue;
            }
            auto& incidence=related[use.edge.index];
            incidence.operand=operand;incidence.edge=use.edge.index;
            incidence.vertex=edge.start_vertex;incidence.support_point=point_index;
            incidence.uses.push_back(identity);incidence.pcurves.push_back(pc);
        }
    for (auto& [edge,incidence]:related)
        result.uv_only_incidences.push_back(std::move(incidence));
}

void point_boundary_events(FaceResult& result,const BRep& a,const BRep& b,double tolerance) {
    for (size_t point_index=0;point_index<result.support_points.size();++point_index) {
        const auto& point=result.support_points[point_index];
        if (!point.retained)
            continue;
        for (int side=0;side<2;++side) {
            const auto& owner=side?b:a;
            const int face=side?point.second_face:point.first_face;
            degenerate_point_uses(result,owner,face,side,point_index,tolerance);
            const int other_face=side?point.first_face:point.second_face;
            const auto other_uv=side?point.first_inside:point.second_inside;
            const auto own_uv=side?point.second_inside:point.first_inside;
            bool found=false;
            for (const auto& [index,uses]:edge_uses(owner,face)) {
                const auto& edge=owner.m_edges[index];
                const auto& curve=owner.m_curves_3d[edge.curve_3d_index];
                if (!curve.is_clamped())
                    continue;
                for (int end=0;end<2;++end)
                    if (projective_cv(curve,end?curve.cv_count()-1:0,point.event.exact_witness)) {
                        const double t=end?curve.domain_end():curve.domain_start();
                        if (!nurbs_interval::curve_within_ball(curve,{t,t},
                            {point.event.point[0],point.event.point[1],point.event.point[2]},tolerance*.25))
                            throw std::runtime_error("Point endpoint cannot establish its original incidence");
                        result.boundary_events.push_back({side,index,end?edge.end_vertex:edge.start_vertex,
                            {point.event.point,other_face,t,other_uv[0],other_uv[1],true},uses,{},true});
                        found=true;
                    }
            }
            const auto state=brep_classify::face_uv(owner,face,own_uv,
                uv_tolerance(owner.m_surfaces[owner.m_faces[face].surface_index],tolerance));
            const bool uv_owned=std::any_of(result.uv_only_incidences.begin(),result.uv_only_incidences.end(),
                [&](const UVOnlyIncidence& use) { return use.support_point==point_index && use.operand==side; });
            if (state==brep_classify::State::Boundary && !found && !uv_owned)
                throw std::runtime_error("Support point needs an unresolved original boundary raw incidence");
        }
    }
}

bool boundary_equivalent(const BoundaryEvent& p,const BoundaryEvent& q,
                         const BRep& a,const BRep& b,double tolerance) {
    const auto& x=event_source(p,a,b);
    const auto& y=event_source(q,a,b);
    if (p.operand==q.operand && p.edge==q.edge)
        return interval_event(x,p.hit.t,q.hit.t,q.hit.point,tolerance);
    const auto events=curve_intersection::intersect(x,y,tolerance);
    if (!events.complete)
        throw std::runtime_error(events.failure);
    for (const auto& hit:events.points)
        if (interval_event(x,p.hit.t,hit.a,q.hit.point,tolerance) &&
            interval_event(y,q.hit.t,hit.b,q.hit.point,tolerance))
            return true;
    for (const auto& overlap:events.overlaps)
        if (p.hit.t>=std::min(overlap.a0,overlap.a1) && p.hit.t<=std::max(overlap.a0,overlap.a1) &&
            q.hit.t>=std::min(overlap.b0,overlap.b1) && q.hit.t<=std::max(overlap.b0,overlap.b1) &&
            nurbs_interval::curve_within_ball(x,{p.hit.t,p.hit.t},
                {q.hit.point[0],q.hit.point[1],q.hit.point[2]},tolerance*.25) &&
            nurbs_interval::curve_within_ball(y,{q.hit.t,q.hit.t},
                {q.hit.point[0],q.hit.point[1],q.hit.point[2]},tolerance*.25))
            return true;
    return false;
}

void group_curves(FaceResult& result,double tolerance) {
    // Keep every incidence. Only the public view chooses one established whole common range.
    for (size_t i=0;i<result.curves.size();++i) {
        bool grouped=false;
        size_t representative=i;
        for (size_t previous:result.curve_representatives) {
            const auto& x=result.curves[i].world;
            const auto& y=result.curves[previous].world;
            const auto events=curve_intersection::intersect(x,y,tolerance);
            if (!events.complete)
                throw std::runtime_error(events.failure);
            for (const auto& common:events.overlaps)
                if (std::min(common.a0,common.a1)==x.domain_start() &&
                    std::max(common.a0,common.a1)==x.domain_end() &&
                    std::min(common.b0,common.b1)==y.domain_start() &&
                    std::max(common.b0,common.b1)==y.domain_end())
                    grouped=true;
            if (grouped) {
                representative=previous;
                break;
            }
        }
        if (!grouped)
            result.curve_representatives.push_back(i);
        result.curve_representative.push_back(representative);
    }
}

void support_nodes(FaceResult& result,double tolerance) {
    for (size_t i=0;i<result.support_points.size();++i) {
        auto& point=result.support_points[i];
        if (!point.retained)
            continue;
        for (const auto& curve:result.curves) {
            if (curve.world.is_clamped() &&
                (projective_cv(curve.world,0,point.event.exact_witness) ||
                 projective_cv(curve.world,curve.world.cv_count()-1,point.event.exact_witness)))
                point.isolated=false;
            else if (!curve_excludes(curve.world,point.event.point,tolerance))
                throw std::runtime_error("Mixed point/curve branch ownership is unresolved");
        }
        size_t node=result.nodes.size();
        for (size_t n=0;n<result.nodes.size();++n)
            if (!result.nodes[n].support_points.empty() && same_projective(point.event.exact_witness,
                result.support_points[result.nodes[n].support_points.front()].event.exact_witness)) {
                node=n;
                break;
            }
        if (node==result.nodes.size())
            result.nodes.push_back({point.event.point,{},{},{},{},point.isolated});
        result.nodes[node].support_points.push_back(i);
        for (size_t use=0;use<result.uv_only_incidences.size();++use)
            if (result.uv_only_incidences[use].support_point==i)
                result.nodes[node].uv_only_incidences.push_back(use);
        result.nodes[node].isolated=result.nodes[node].isolated && point.isolated;
    }
}

void boundary_nodes(FaceResult& result,const BRep& a,const BRep& b,double tolerance) {
    for (size_t i=0;i<result.boundary_events.size();++i) {
        auto& point=result.boundary_events[i];
        const auto& source=event_source(point,a,b);
        for (size_t curve_index=0;curve_index<result.curves.size();++curve_index) {
            const auto& curve=result.curves[curve_index];
            const auto events=curve_intersection::intersect(source,curve.world,tolerance);
            if (!events.complete)
                throw std::runtime_error(events.failure);
            const auto own=[&](double a,double b) {
                if (!detail::owns_point(curve.world,b,point.hit.point,tolerance)) {
                    if (curve_excludes(curve.world,point.hit.point,tolerance))
                        return; // Entire original target is rigorously outside this point's Tol ball.
                    throw InventoryError("CC incidence lacks a direct original target singleton ownership proof",
                        {FailureKind::CurvePointOwnership,point.operand,point.edge,point.uses.front()});
                }
                point.owners.push_back({curve_index,a,b});
                point.isolated=false;
            };
            for (const auto& hit:events.points)
                if (interval_event(source,point.hit.t,hit.a,point.hit.point,tolerance))
                    own(hit.a,hit.b);
            for (const auto& common:events.overlaps)
                if (point.hit.t>=std::min(common.a0,common.a1) && point.hit.t<=std::max(common.a0,common.a1)) {
                    if (common.a0==common.a1)
                        throw std::runtime_error("Positive common range has a zero source raw interval");
                    // Proposal only: the direct target singleton certificate is authoritative.
                    const double fraction=(point.hit.t-common.a0)/(common.a1-common.a0);
                    own(point.hit.t,std::lerp(common.b0,common.b1,fraction));
                }
        }
        size_t node=result.nodes.size();
        for (size_t n=0;n<result.nodes.size();++n) {
            bool same=false;
            for (size_t j:result.nodes[n].boundary_events)
                same=same || boundary_equivalent(point,result.boundary_events[j],a,b,tolerance);
            for (size_t j:result.nodes[n].support_points) {
                const auto& support=result.support_points[j];
                const int own_face=point.operand?support.second_face:support.first_face;
                const int other_face=point.operand?support.first_face:support.second_face;
                const bool incident=point.hit.face==other_face && std::any_of(point.uses.begin(),point.uses.end(),
                    [&](const EdgeUse& use){return use.face==own_face;});
                if (incident && source.is_clamped() &&
                    ((point.hit.t==source.domain_start() && projective_cv(source,0,support.event.exact_witness)) ||
                     (point.hit.t==source.domain_end() && projective_cv(source,source.cv_count()-1,support.event.exact_witness))))
                    same=true;
            }
            same=same && nurbs_interval::curve_within_ball(source,{point.hit.t,point.hit.t},
                {result.nodes[n].point[0],result.nodes[n].point[1],result.nodes[n].point[2]},tolerance*.25);
            if (same) {
                node=n;
                break;
            }
        }
        if (node==result.nodes.size())
            result.nodes.push_back({point.hit.point,{},{},{},{},point.isolated});
        for (const auto& ownership:point.owners)
            if (!detail::owns_point(result.curves[ownership.curve].world,ownership.target_parameter,
                                    result.nodes[node].point,tolerance))
                throw InventoryError("Canonical point center lacks direct target curve ownership",
                    {FailureKind::CurvePointOwnership,point.operand,point.edge,point.uses.front()});
        result.nodes[node].boundary_events.push_back(i);
        result.nodes[node].isolated=result.nodes[node].isolated && point.isolated;
    }
}

bool same_use(const EdgeUse& a,const EdgeUse& b) {
    return a.face==b.face && a.wire==b.wire && a.occurrence==b.occurrence &&
        a.pcurve==b.pcurve && a.orientation==b.orientation && a.wire_orientation==b.wire_orientation;
}

void retain_unrepresented_uv_uses(FaceResult& result,const BRep& first,const BRep& second) {
    for (size_t node_index=0;node_index<result.nodes.size();++node_index) {
        auto& node=result.nodes[node_index];
        for (size_t event_index:node.boundary_events) {
            const auto& event=result.boundary_events[event_index];
            if (event.vertex<0)
                continue; // No established original vertex identity: do not guess a pole.
            const auto& owner=event.operand?second:first;
            for (const auto& source_use:event.uses) {
                const auto boundary=brep_face_boundary::extract(owner,source_use.face);
                if (!boundary.complete)
                    throw std::runtime_error(boundary.failure);
                for (const auto& wire:boundary.wires)
                    for (size_t i=0;i<wire.uses.size();++i) {
                        const auto& use=wire.uses[i];
                        if (!use.degenerated || !brep_face_boundary::detail::active(wire.reference.orientation) ||
                            !brep_face_boundary::detail::active(use.edge.orientation))
                            continue;
                        const auto& edge=owner.m_edges[use.edge.index];
                        if (edge.start_vertex!=event.vertex)
                            continue; // Same original vertex, not a world-distance coincidence.
                        const EdgeUse identity{source_use.face,wire.reference.index,static_cast<int>(i),use.pcurve,
                            use.edge.orientation,wire.reference.orientation};
                        const bool represented=std::any_of(node.uv_only_incidences.begin(),node.uv_only_incidences.end(),
                            [&](size_t index) {
                                const auto& incidence=result.uv_only_incidences[index];
                                return incidence.operand==event.operand && incidence.edge==use.edge.index &&
                                    std::any_of(incidence.uses.begin(),incidence.uses.end(),
                                        [&](const EdgeUse& image){return same_use(image,identity);});
                            });
                        const bool recorded=std::any_of(node.unrepresented_uv_incidences.begin(),node.unrepresented_uv_incidences.end(),
                            [&](size_t index) {
                                const auto& incidence=result.unrepresented_uv_incidences[index];
                                return incidence.operand==event.operand && incidence.edge==use.edge.index &&
                                    same_use(incidence.use,identity);
                            });
                        if (represented || recorded)
                            continue;
                        if (use.pcurve<0 || use.pcurve>=static_cast<int>(owner.m_curves_2d.size()))
                            throw InventoryError("Relevant original UV-only use has no admitted pcurve",
                                {FailureKind::InactiveBoundaryUse,event.operand,use.edge.index,identity});
                        node.unrepresented_uv_incidences.push_back(result.unrepresented_uv_incidences.size());
                        result.unrepresented_uv_incidences.push_back({event.operand,use.edge.index,event.vertex,
                            node_index,identity,owner.m_curves_2d[use.pcurve].domain()});
                    }
            }
        }
    }
}

void finalize_inventory(FaceResult& result,const BRep& a,const BRep& b,double tolerance) {
    point_boundary_events(result,a,b,tolerance);
    group_curves(result,tolerance);
    support_nodes(result,tolerance);
    boundary_nodes(result,a,b,tolerance);
    retain_unrepresented_uv_uses(result,a,b);
}

} // namespace

FaceResult interferences(const BRep& first,const BRep& second,double tolerance) {
    check_input(first,tolerance);
    check_input(second,tolerance);
    FaceResult result;
    const auto af=active_faces(first),bf=active_faces(second);
    result.expected_pairs=af.size()*bf.size();
    try {
        for (int face:af)
            validate_boundary_policy(first,face,0);
        for (int face:bf)
            validate_boundary_policy(second,face,1);
        for (int a:af)
            for (int b:bf) {
                const auto& sa=first.m_surfaces[first.m_faces[a].surface_index];
                const auto& sb=second.m_surfaces[second.m_faces[b].surface_index];
                result.coverage.push_back({a,b});
                auto& receipt=result.coverage.back();
                if (excluded(sa,sb,tolerance)) {
                    receipt.support=surface_section::Coverage::OriginalDomainExcluded;
                    receipt.trim_ownership_complete=true;
                    continue;
                }
                const auto events=surface_section::intersect(sa,sb,tolerance);
                receipt.support=events.coverage;
                receipt.unresolved_domains=events.unresolved_domains;
                receipt.failure=events.failure;
                if (!events.complete)
                    throw std::runtime_error(events.failure);
                for (const auto& point:events.points) {
                    Point ua,ub;
                    const auto ma=point_trim_membership(first,a,point.first_preimages,
                        uv_tolerance(sa,tolerance),ua);
                    const auto mb=point_trim_membership(second,b,point.second_preimages,
                        uv_tolerance(sb,tolerance),ub);
                    const bool included=ma==Membership::Included && mb==Membership::Included;
                    result.support_points.push_back({a,b,point,ua,ub,ma,mb,included,true});
                    if (ma==Membership::Excluded || mb==Membership::Excluded) {
                        receipt.trim_ownership_complete=true;
                        continue;
                    }
                    if (!included)
                        throw std::runtime_error("Original support point trim ownership is unresolved");
                    receipt.trim_ownership_complete=point.first_preimages.complete && point.second_preimages.complete;
                }
                if (events.coincident)
                    result.overlaps.push_back({a,b});
                support_curves(result,a,b,first,second,events,tolerance);
                if (events.coverage==surface_section::Coverage::OriginalDomainExcluded) {
                    receipt.trim_ownership_complete=true;
                    continue;
                }
                if (events.coverage==surface_section::Coverage::GlobalContactPoint) {
                    // Never turn tolerance overlaps into positive curves after an original-domain point proof.
                    // Original boundary incidences are materialized below from exact endpoint witnesses.
                    continue;
                }
                boundary_pair(result,first,second,a,b,0,tolerance);
                boundary_pair(result,first,second,a,b,1,tolerance);
            }
        finalize_inventory(result,first,second,tolerance);
        result.boundary_ownership_complete=result.unrepresented_uv_incidences.empty();
    } catch (const InventoryError& error) {
        result.complete=false;
        result.failure=error.what();
        result.unresolved_incidences.push_back(error.incidence);
    } catch (const std::runtime_error& error) {
        result.complete=false;
        result.failure=error.what();
    }
    return result;
}

} // namespace session_cpp::brep_section
