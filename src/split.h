#pragma once

#include "brep.h"
#include "brep_classify.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "curve_self_intersection.h"
#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace session_cpp {

class Line;
class Polyline;

} // namespace session_cpp

namespace session_cpp::split {

struct SamplingError : std::invalid_argument {
    NurbsCurve curve;
    NurbsCurve part;
    double tolerance;
    size_t visited;

    SamplingError(const NurbsCurve& source, const NurbsCurve& active, double tol, size_t count)
        : std::invalid_argument("Trim sampling exceeds the bounded workload"),
          curve(source), part(active), tolerance(tol), visited(count) {}
};

struct SelfIntersectionError : std::invalid_argument {
    NurbsCurve world, pcurve;
    curve_self_intersection::Result events;
    size_t source;
    int source_edge, section;
    double tolerance;
    SelfIntersectionError(const NurbsCurve& geometry, const NurbsCurve& uv,
        const curve_self_intersection::Result& result, size_t index, int edge, int identity, double tol)
        : std::invalid_argument(describe(uv, result, edge, identity, tol)), world(geometry), pcurve(uv), events(result),
          source(index), source_edge(edge), section(identity), tolerance(tol) {}

    /// The failure with the curve it is about: which source, its shape, how far the search got and where it stopped.
    static std::string describe(const NurbsCurve& uv, const curve_self_intersection::Result& result, int edge, int identity, double tol) {
        std::ostringstream text;
        text.precision(17);
        text << result.failure << "; " << (edge >= 0 ? "edge " : "section ") << (edge >= 0 ? edge : identity)
             << ", pcurve degree " << uv.degree() << (uv.is_rational() ? " rational, " : ", ") << uv.cv_count() << " control points over ["
             << uv.domain_start() << ", " << uv.domain_end() << "], tolerance " << tol << ", " << result.cells << " cells";
        if (result.has_failure_pair)
            text << ", stopped at depth " << result.failure_depth << " on [" << result.failure_pair[0] << ", " << result.failure_pair[1]
                 << "] against [" << result.failure_pair[2] << ", " << result.failure_pair[3] << "]";
        return text.str();
    }
};

struct DomainError : std::invalid_argument {
    size_t source;
    int source_edge, section;
    double first, last;
    Point query;
    brep_classify::State state;
    std::pair<double, double> world_domain, pcurve_domain;
    DomainError(size_t index, int edge, int identity, double lo, double hi, const Point& point,
        brep_classify::State classification, std::pair<double, double> world,
        std::pair<double, double> pcurve, const char* reason)
        : std::invalid_argument(reason), source(index), source_edge(edge), section(identity),
          first(lo), last(hi), query(point), state(classification),
          world_domain(world), pcurve_domain(pcurve) {}
};

struct FaceCut {
    NurbsCurve world;
    NurbsCurve pcurve;
    int section = -1;
    bool placed = false; // The section stage vouches that it meets the boundary and the cuts of other families at its ends only.
    int family = -1; // Placed cuts of one family, sections with one face of the other solid, can still cross where the two surfaces touch.
    std::array<int, 2> vertices = {-1, -1}; // Vertex of the model at each end that lies on the face boundary.
    std::array<int, 2> nodes = {-1, -1}; // Id shared by the cut ends that meet at one point inside the face.
};

struct CurveError : std::invalid_argument {
    NurbsCurve curve;
    NurbsCurve pcurve;
    NurbsSurface surface;
    double parameter;
    double distance;
    double tolerance;

    CurveError(const NurbsCurve& world, const NurbsCurve& uv, const NurbsSurface& support,
        double at, double gap, double tol, const std::string& reason)
        : std::invalid_argument("Split curve/trim correspondence " + reason), curve(world), pcurve(uv),
          surface(support), parameter(at), distance(gap), tolerance(tol) {}
};

struct EdgePiece {
    int source_edge = -1;
    int section = -1;
    int edge = -1;
    double first = 0.0;
    double last = 0.0;
    bool exact = false; // No edge of the other operand lies along it or crosses it: source and range place it in an assembly.
};

/// A required parameter on one ORIGINAL owning edge; vertex is an optional existing BRep vertex ID.
struct EdgeEvent {
    int edge = -1;
    double parameter = 0.0;
    int vertex = -1;
};

/// Preserve the original edge/range when a required partition cannot be represented or certified.
struct EdgeEventError : std::invalid_argument {
    int edge;
    double first;
    double last;
    EdgeEventError(int index, double lo, double hi, const std::string& reason)
        : std::invalid_argument(reason), edge(index), first(lo), last(hi) {}
};

struct FaceSplit {
    BRep brep;
    std::vector<int> faces;
    std::vector<EdgePiece> edges;
    std::vector<EdgeEvent> edge_events; // ORIGINAL edge/raw parameter -> resolved owner-local vertex receipt.
    bool placed = false; // The arrangement was read off the ends of placed cuts; false when the curves were intersected.
};

struct ArrangementParentDiagnostic {
    size_t hole = 0, outer = 0;
    bool shared = false, area_eligible = false, inside_evaluated = false, contained = false;
};

struct ArrangementCycleDiagnostic {
    std::vector<Point> points;
    std::vector<size_t> edges;
    double area = 0;
    size_t runs = 0;
    size_t first_source = 0;
    double first_parameter = 0;
    double last_parameter = 0;
    bool probe_evaluated = false;
    Point probe;
    std::string disposition;
};

struct ArrangementPairDiagnostic {
    size_t first_source = 0, second_source = 0;
    NurbsCurve first, second;
    std::vector<double> first_hits, second_hits;
    size_t overlaps = 0;
};

struct ArrangementSpanDiagnostic {
    size_t source = 0;
    int edge = -1, section = -1;
    double a = 0, b = 0;
    std::array<double, 2> lo, hi, query_lo, query_hi;
};

/// Original source/raw incidence from one complete CC point query; no tolerance deduplication.
struct ArrangementPointIncidence {
    size_t source=0, other_source=0;
    double parameter=0, other_parameter=0;
    double first=0, last=0, other_first=0, other_last=0;
};

/// Positive raw interval represented by one proved original boundary event vertex.
struct ArrangementVertexOwnership {
    size_t node=0, source=0, boundary_source=0;
    int source_edge=-1, section=-1;
    double first=0, last=0, boundary_parameter=0;
    Point vertex;
    std::vector<double> raw_parameters;
    std::vector<ArrangementPointIncidence> incidences;
};

struct ArrangementDiagnostic {
    size_t sources = 0, spans = 0, nodes = 0, half_edges = 0, retained_cycles = 0, regions = 0;
    double uv_tolerance = 0;
    std::vector<ArrangementCycleDiagnostic> cycles;
    std::vector<ArrangementVertexOwnership> vertex_owned_intervals;
    std::vector<ArrangementParentDiagnostic> parent_tests;
    std::vector<ArrangementPairDiagnostic> boundary_cut_pairs;
    std::vector<ArrangementSpanDiagnostic> span_boxes;
    std::vector<std::pair<size_t, size_t>> candidate_pairs;
    std::string first_discarded_reason;
    std::vector<Point> first_discarded_uv;
};

/// Split existing edges at world raw events, updating every original wire use and pcurve alias.
/// Faces retain their indices; the result lists affected faces and ORIGINAL-world EdgePiece ranges.
FaceSplit split_brep_edges_at_events(const BRep& brep, const std::vector<EdgeEvent>& events,
                                     double tolerance);

/// Partition a face using existing pcurves, retaining source-edge and section identity.
FaceSplit split_brep_face_by_pcurves(
    const BRep& brep,
    int face_index,
    const std::vector<FaceCut>& cutters,
    double tolerance
);

/// Observe existing arrangement decisions without additional geometry evaluations.
FaceSplit split_brep_face_by_pcurves(
    const BRep& brep, int face_index, const std::vector<FaceCut>& cutters,
    double tolerance, ArrangementDiagnostic* diagnostic
);

/// The same partition made in the model itself: FaceSplit::brep stays empty, and a split that throws leaves the model unspecified.
/// `validated` says the model has passed an earlier split at this tolerance, so only the edges made here are certified.
FaceSplit split_brep_face_in_place(
    BRep& brep, int face_index, const std::vector<FaceCut>& cutters, double tolerance, bool validated
);

// ═══════════════════════════════════════════════════════════════════════════
// Split
// ═══════════════════════════════════════════════════════════════════════════
/// Split a curve at isolated 3D intersections, retaining every piece and rejecting overlapping cutters.
std::vector<NurbsCurve> split_curve_by_curves(
    const NurbsCurve& curve,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Partition one face inside its owning BRep, retaining all regions and shared shell topology.
BRep split_brep_face_by_curves(
    const BRep& brep,
    int face_index,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Wrap a surface's natural boundary in a BRep and partition it with on-surface curves.
BRep split_surface_by_curves(
    const NurbsSurface& surface,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Split a line at isolated 3D intersections, retaining line types and display attributes.
std::vector<Line> split_line_by_curves(
    const Line& line,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Split a polyline, retaining each original corner, piece order and display attributes.
std::vector<Polyline> split_polyline_by_curves(
    const Polyline& polyline,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

} // namespace session_cpp::split
