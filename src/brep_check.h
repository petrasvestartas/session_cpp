#pragma once

#include "brep.h"
#include <string>
#include <vector>

namespace session_cpp::brep_check {

/// BRepCheck_Status, named for what was measured
enum class Status {
    BadReference, // An index out of its table, a non-finite point or tolerance, an invalid surface or curve.
    VertexOffEdge, // A vertex farther from the end of an incident edge curve or lifted pcurve than its tolerance.
    EdgeOffPCurve, // The edge curve and a pcurve lifted onto its surface apart by more than the edge tolerance.
    SameParameter, // The lifted pcurve on the edge curve, but apart from it at the same parameter (BRepCheck_InvalidSameParameterFlag).
    BadRange, // An edge or pcurve range empty or reversed.
    PCurveSelfIntersect, // A pcurve crossing itself in uv.
    LoopSelfIntersect, // Two edges of one wire crossing in uv.
    LoopsIntersect, // Two wires of one face crossing in uv.
    WireOpen, // Consecutive edges of a wire not sharing a vertex, or the wire not closing.
    WireOpenUV, // Consecutive pcurves of a wire apart in uv even modulo the periods, or by a period on a face with a seam.
    LoopOrientation, // The outer wire clockwise in uv.
    SeamBoundary, // An edge with one pcurve on a surface used on both sides by a face of it: a boundary on its own seam that is no seam pair.
    ToleranceOutOfProportion, // A tolerance above a fraction of the entity's size (a degenerated edge has none) and above a multiple of the median, or an edge's above a multiple of the reference.
    EdgeUses, // A closed shell's edge used an odd number of times, or any shell's edge more than twice and not as often one way as the other.
    ShellOpen, // A shell of a solid with free edges.
    ShellNotOrientable, // A shell edge used an even number of times, not as often one way as the other.
    SolidInsideOut, // A solid enclosing negative volume.
    DegeneratedWithoutPCurve, // A degenerated edge with no pcurve.
    EdgeWithoutPCurve, // An edge used by a face with no pcurve on its surface.
};

enum class Entity { Vertex, Edge, Wire, Face, Shell, Solid };

struct Issue {
    Status status;
    Entity entity;
    int index = -1;
    double value = 0.0;
    std::string message;
};

struct Options {
    int samples = 16; // Segments per edge and pcurve.
    double min_tolerance = 1e-7; // Added to every tolerance (Precision::Confusion).
    double size_fraction = 0.5; // A tolerance above this fraction of its entity's size...
    double median_factor = 1000.0; // ...and above this multiple of the BRep's median tolerance is out of proportion.
    double reference_tolerance = 0.0; // Where the BRep came from parts this loose (a boolean's operands), an edge tolerance...
    double reference_factor = 100.0; // ...above this multiple of it is out of proportion too.
};

struct Report {
    std::vector<Issue> issues;

    /// Whether nothing was found.
    bool valid() const { return issues.empty(); }

    /// Whether any issue has this status.
    bool has(Status status) const;

    /// "valid", or the count and the first issues: "3 issues: edge 4 off its pcurve by 0.9 (tolerance 1e-07); ..."
    std::string summary(int first = 3) const;
};

/// Name of a status, as in the summary.
std::string name(Status status);

/// BRepLib_ValidateEdge: the largest distance between an edge's 3D curve and its pcurves lifted onto their surfaces at the same parameter, sampled as check samples it; 0 for a degenerated edge
double deviation(const BRep& brep, int edge, int samples = Options{}.samples);

/// BRepCheck_Analyzer: every vertex, edge, wire, face, shell and solid measured natively on its Surface, Curve and Curve2d, without meshing
Report check(const BRep& brep, const Options& options = {});

}
