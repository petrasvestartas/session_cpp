#pragma once

#include "brep.h"
#include "surface_section.h"
#include <array>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <string>
#include <vector>

namespace session_cpp::brep_section {

/// Partial original uses require interval-aware Section workers; no pooled-law fallback.
struct RangedInputError : std::runtime_error {
    int operand, face, edge, pcurve;
    std::pair<double,double> range;
    RangedInputError(int owner,int owner_face,int owner_edge,int image,std::pair<double,double> raw)
        :std::runtime_error("Native Section does not yet support partial original curve uses"),
         operand(owner),face(owner_face),edge(owner_edge),pcurve(image),range(raw) {}
};

struct CurveHit {
    Point point;
    int face;
    double t;
    double u;
    double v;
    bool tangent;
};

struct CurveSpan {
    int face;
    double first;
    double last;
    NurbsCurve pcurve;
};

struct CurveResult {
    std::vector<CurveHit> points;
    std::vector<CurveSpan> overlaps;
    bool complete = true;
    std::string failure;
};

struct FaceCurve {
    int first_face;
    int second_face;
    NurbsCurve world;
    NurbsCurve first_pcurve;
    NurbsCurve second_pcurve;
    bool touching = false; // The surfaces part again either side: a section locus that bounds no material.
};

/// Legacy coincidence candidate; whole-domain qualification and trim-area handling are separate.
struct FaceOverlap {
    int first_face;
    int second_face;
};

/// An original edge use, including periodic aliases. No sampled world deduplication.
struct EdgeUse {
    int face = -1;
    int wire = -1;
    int occurrence = -1;
    int pcurve = -1;
    BRepOrientation orientation = BRepOrientation::Forward;
    BRepOrientation wire_orientation = BRepOrientation::Forward;
};

enum class FailureKind { None, InactiveBoundaryUse, UnrepresentedDegenerateUse, CurvePointOwnership };
struct UnresolvedIncidence {
    FailureKind kind = FailureKind::None;
    int operand = -1;
    int edge = -1;
    EdgeUse use;
};
struct InventoryError : std::runtime_error {
    UnresolvedIncidence incidence;
    InventoryError(const char* message,UnresolvedIncidence source)
        :std::runtime_error(message),incidence(std::move(source)) {}
};
struct CurveOwnership {
    size_t curve = 0;
    double source_parameter = 0;
    double target_parameter = 0;
};

struct BoundaryEvent {
    int operand = -1;
    int edge = -1;
    int vertex = -1; // ORIGINAL owner-local endpoint, never a global node ID.
    CurveHit hit;
    std::vector<EdgeUse> uses;
    std::vector<CurveOwnership> owners;
    bool isolated = true;
    bool retained = true; // Excluded original uses remain provenance, never contact nodes.
    std::vector<double> source_parameters; // Actual ORIGINAL selected-PC raw proposals, parallel to uses.
};

struct BoundarySpanExclusion {
    int operand=-1,edge=-1,other_face=-1;
    EdgeUse use;
    double first=0,last=0; // ORIGINAL source-world raw interval, never erased/merged.
    bool own_outside=false,target_outside=false;
};

struct SupportPoint {
    int first_face = -1;
    int second_face = -1;
    surface_section::PointEvent event;
    Point first_inside, second_inside;
    surface_point_preimages::Membership first_membership = surface_point_preimages::Membership::Unresolved;
    surface_point_preimages::Membership second_membership = surface_point_preimages::Membership::Unresolved;
    bool retained = false;
    bool isolated = true;
};

/// A degenerate boundary has a real UV use and existing vertex, but no world raw t.
struct UVOnlyIncidence {
    int operand = -1;
    int edge = -1;
    int vertex = -1;
    size_t support_point = 0;
    std::vector<EdgeUse> uses;
    std::vector<NurbsCurve> pcurves; // Original aliases and their exact raw domains.
};

/// A relevant original degenerate use not yet split/owned by a regular curve node.
/// No synthetic SupportPoint or world raw parameter is assigned to this UV incidence.
struct UnrepresentedUVIncidence {
    int operand = -1;
    int edge = -1;
    int vertex = -1;
    size_t node = 0;
    EdgeUse use;
    std::pair<double,double> pcurve_domain{};
};

struct PairCoverage {
    int first_face = -1;
    int second_face = -1;
    surface_section::Coverage support = surface_section::Coverage::Unverified;
    bool trim_ownership_complete = false;
    std::array<double, 8> unresolved_domains{};
    std::string failure;
    bool qualified() const {
        return surface_section::qualified(support) && trim_ownership_complete;
    }
};

struct PointNode {
    Point point;
    std::vector<size_t> boundary_events;
    std::vector<size_t> support_points;
    std::vector<size_t> uv_only_incidences;
    std::vector<size_t> unrepresented_uv_incidences;
    bool isolated = true;
};

/// One complete CC trim event on an ORIGINAL face-associated boundary use.
struct OriginalTrimEvent {
    int wire = -1;
    int occurrence = -1;
    int operand = -1;
    int face = -1;
    int edge = -1;
    int pcurve = -1;
    BRepOrientation orientation = BRepOrientation::Forward;
    double section_parameter = 0;
    double boundary_parameter = 0;
    bool overlap_endpoint = false;
};

/// Original laws and all unmerged raw trim identities before piece materialization.
/// A positive original raw interval represented by one exact shared boundary event.
struct OriginalTrimOwnership {
    double first = 0, last = 0;
    std::array<double,3> homogeneous{};
    double weight = 0;
    Point center;
    size_t node = static_cast<size_t>(-1);
    std::vector<size_t> events;
    std::vector<std::pair<double,double>> boundary_parameters;
    std::vector<NurbsCurve> boundary_laws;
};

/// An emitted support piece, bound to its original raw law at the actual push site.
struct OriginalTrimPiece {
    double first=0,last=0;
    size_t curve=0;
};
struct OriginalTrimLaw {
    FaceCurve original;
    std::vector<OriginalTrimEvent> events;
    std::vector<OriginalTrimOwnership> owned_intervals;
    std::vector<double> partition;
    std::vector<OriginalTrimPiece> retained_pieces;
    bool trim_ownership_complete=false;
};

struct FaceResult {
    std::vector<OriginalTrimLaw> original_trim_laws;
    std::vector<FaceCurve> curves; // Every face-pair incidence is retained.
    std::vector<FaceOverlap> overlaps;
    std::vector<BoundaryEvent> boundary_events; // Includes retained and excluded original use receipts.
    std::vector<BoundarySpanExclusion> excluded_boundary_spans;
    std::vector<SupportPoint> support_points;
    std::vector<UVOnlyIncidence> uv_only_incidences;
    std::vector<UnrepresentedUVIncidence> unrepresented_uv_incidences;
    std::vector<PointNode> nodes;
    std::vector<PairCoverage> coverage;
    std::vector<UnresolvedIncidence> unresolved_incidences;
    std::vector<size_t> curve_representatives; // Public view; never erases incidences.
    std::vector<size_t> curve_representative; // Each incidence -> proved whole-range representative.
    size_t expected_pairs = 0;
    bool boundary_ownership_complete = false;
    // Compatibility execution status only. This is NOT an original-domain theorem.
    bool complete = true;
    std::string failure;
    bool coverage_complete() const {
        if (!complete || !boundary_ownership_complete || coverage.size() != expected_pairs)
            return false;
        for (const auto& pair : coverage)
            if (!pair.qualified())
                return false;
        return true;
    }
};

CurveResult curve(const NurbsCurve& curve, const BRep& brep, double tolerance);

/// Legacy support-locus worker; complete is computational status, not full coverage.
FaceResult faces(const BRep& first, const BRep& second, double tolerance);

/// Shared public Section / Boolean inventory. Strict qualification is coverage_complete().
FaceResult interferences(const BRep& first, const BRep& second, double tolerance);

}
