#pragma once

#include "brep.h"
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace session_cpp::bop {

/// Axis-aligned box grown by a tolerance (Bnd_Box)
struct Box {
    std::array<double, 3> lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    void add(const Point& p, double grow = 0.0);
    bool overlaps(const Box& other, double gap = 0.0) const;
};

/// Sampled loops of a face in its surface parameters: the uv classifier of the filler and builders
struct Region {
    /// The pcurve piece a loop segment samples and how far it bulges from the segment
    struct Arc {
        const Curve* curve = nullptr;
        double t0 = 0.0, t1 = 0.0, bulge = 0.0;
        Vector turn{0, 0, 0}; // whole periods the piece is moved by
    };
    std::vector<std::vector<Point>> loops;
    std::vector<std::vector<Arc>> arcs; // per loop, one per segment or none
    std::vector<int> winds; // per loop: -1 when it closes in uv, else the periodic direction it runs once around, its last point its first moved by the period
    double eps = 1e-9; // uv distance that counts as on a loop
    std::array<double, 2> period{0.0, 0.0}; // the surface's period per direction, else 0
    bool whole = false; // an analytic surface: its parameters run on past a period, and a loop may lie in any turn
    std::array<double, 2> lo{1e300, 1e300}, hi{-1e300, -1e300};
    void add_loop(std::vector<Point> loop, int winding = -1);
    /// The periodic direction a wire runs once around, given where its uv ends against where it started; -1 when it closes
    int winding(const Point& from, const Point& to) const;
    /// The direction the loops wind around, -1 when all close: the region is then a band and is classified across it
    int band() const;
    /// 1 inside, 0 on a loop, -1 outside; a closed direction is tried shifted by its period
    int state(const Point& uv) const;
    double area() const;
    /// A point well inside: the middle of the widest inside run of a few scanlines
    bool interior(Point& uv) const;
};

/// The parameter step after which a surface repeats itself, per direction: its span when closed, less when a patch of revolution overlaps itself, else 0
std::array<double, 2> surface_period(const NurbsSurface& s);

/// Surface::period of an analytic surface; of a freeform one, as above
std::array<double, 2> surface_period(const Surface& s);

/// The loops of a face in its surface parameters, uv tolerance from the face tolerance and the surface's speed
Region face_region(const BRep& brep, int face, double tolerance);

/// A piece of a shared curve, as BOPDS keeps a split edge: the curve itself, never copied or refitted, with the range of the piece; run backwards when t0 > t1, and for a pcurve moved by whole periods of its surface
struct Span {
    std::shared_ptr<const Curve> curve;
    double t0 = 0.0, t1 = 0.0;
    std::array<double, 2> shift{0.0, 0.0};
    /// The point at s in [0, 1] along the piece
    Point at(double s) const;
    Span reversed() const { return {curve, t1, t0, shift}; }
    /// The piece as a curve of its own, for a BRep whose edges carry no range: an analytic curve with the range as its domain, a freeform one trimmed
    Curve copy() const;
};

/// A span over a curve that outlives it (an operand's)
Span borrowed(const Curve& curve, double t0, double t1);

/// A span over the whole of a curve it owns
Span owned(Curve curve);

/// The pcurve an edge has on a face for a use of that orientation, over its range (BRep_Tool::CurveOnSurface); without a curve when the edge has none there
Span pcurve_span(const BRep& brep, int edge, int face, BRepOrientation orientation);

/// The 3D curve of an edge over its range (BRep_Tool::Curve)
Span edge_span(const BRep& brep, int edge);

/// BOPDS_Pave
struct Pave {
    int vertex = -1;
    double t = 0.0;
};

/// BOPDS_PaveBlock: a piece of an edge or a section curve between two paves; its image is one edge of the result
struct PaveBlock {
    int edge = -1; // DS edge, or -1 for a section piece
    int curve = -1; // DS curve of a section piece
    Pave first, last;
    int common = -1; // DS common block
};

/// BOPDS_CommonBlock: pave blocks of different edges lying on each other
struct CommonBlock {
    std::vector<int> blocks;
};

struct Vertex {
    Point point;
    double tolerance = 0.0;
    int operand = -1; // -1 for a vertex made by an interference alone
    int local = -1;
    int sd = -1; // the vertex it was merged into (same domain)
    double reach = 0.0; // how far along its edge an EE or EF vertex's common part runs (ExtendedTolerance)
    bool joint = false; // made only where two pieces of one section curve meet: joined away in the result
};

struct Edge {
    int operand = 0, local = 0;
    double tolerance = 0.0;
    bool degenerated = false;
    double t0 = 0.0, t1 = 0.0; // its range on its curve
    std::shared_ptr<const Curve> piece; // the curve over that range alone, when the edge shares its curve
    std::vector<Pave> paves;
    std::vector<int> blocks; // in parameter order
    Box box;
};

/// BOPDS_FaceInfo: On blocks are the blocks of the face's own edges, In blocks edges of the other operand lying in it, Sc blocks section pieces
struct Face {
    int operand = 0, local = 0;
    double tolerance = 0.0;
    std::vector<int> edges; // DS edges of its wires
    std::vector<int> seams; // those its wires use twice
    std::vector<int> in_blocks, sc_blocks;
    std::vector<int> in_vertices; // vertices of the other operand met inside it (VF, EF)
    Region region;
    NurbsSurface patch; // the surface over the uv box of the region
    Box box;
};

/// BOPDS_Curve: a face/face section with its pcurves on both faces
struct Section {
    int face_a = -1, face_b = -1;
    std::shared_ptr<const Curve> world, pcurve_a, pcurve_b; // as bop_ff returned them
    double tolerance = 0.0;
    std::vector<Pave> paves;
};

/// BOPDS_DS over two operands: shapes with their own growing tolerance, paves, pave blocks, common blocks, face info, section curves and the interference log
struct DS {
    std::array<const BRep*, 2> operands{};
    double fuzzy = 0.0;
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<PaveBlock> blocks;
    std::vector<CommonBlock> commons;
    std::vector<Section> curves;
    std::array<int, 2> vertex_offset{}, edge_offset{}, face_offset{};
    std::vector<std::array<int, 2>> same_domain_faces; // DS faces of FF pairs on one surface
    std::map<std::pair<int, int>, Span> pcurves; // (block, DS face) -> pcurve of an In or Sc block, along the block
    std::vector<std::string> log; // interferences that were found but not used, and refused geometry

    const BRep& brep(int operand) const { return *operands[operand]; }
    const Curve& edge_curve(const Edge& e) const { return brep(e.operand).m_curves_3d[brep(e.operand).m_edges[e.local].curve_3d_index]; }
    const Surface& face_surface(const Face& f) const { return brep(f.operand).m_surfaces[brep(f.operand).m_faces[f.local].surface_index]; }
    /// The curve of an edge over its range alone, in its true parameter
    const Curve& curve_of(const Edge& e) const { return e.piece ? *e.piece : edge_curve(e); }
    const Surface& surface_of(const Face& f) const { return face_surface(f); }
    int real(int v) const {
        while (vertices[v].sd >= 0)
            v = vertices[v].sd;
        return v;
    }
    /// The vertices merged into one new vertex whose ball holds all of theirs (BOPTools_AlgoTools::MakeVertex)
    int merge(const std::vector<int>& group);
    int add_vertex(const Point& p, double tolerance);
    /// The block's own geometry: the edge or section curve and its parameter range
    const Curve& block_curve(const PaveBlock& b) const { return b.edge >= 0 ? curve_of(edges[b.edge]) : *curves[b.curve].world; }
    /// The block as its curve and range
    Span block_span(const PaveBlock& b) const { return b.edge >= 0 ? borrowed(edge_curve(edges[b.edge]), b.first.t, b.last.t) : Span{curves[b.curve].world, b.first.t, b.last.t, {}}; }
    /// The pcurve of a piece [t0, t1] of an operand's edge on one of its faces, as the face's wire uses it: the edge's own pcurve and the matching range
    Span edge_pcurve(const Edge& e, int local_face, BRepOrientation orientation, double t0, double t1) const;
    Point block_point(const PaveBlock& b, double s) const;
    double block_tolerance(const PaveBlock& b) const { return b.edge >= 0 ? edges[b.edge].tolerance : curves[b.curve].tolerance; }
    /// A block whose ends became one vertex that covers it: no edge of the result
    bool micro(const PaveBlock& b) const { return b.first.vertex == b.last.vertex && block_point(b, 0.5).distance(vertices[b.first.vertex].point) <= vertices[b.first.vertex].tolerance + block_tolerance(b); }
    /// Blocks sharing the image of b: its common block or b alone
    std::vector<int> family(int b) const { return blocks[b].common >= 0 ? commons[blocks[b].common].blocks : std::vector<int>{b}; }
};

/// BOPAlgo_PaveFiller::PerformInternal on two solids
DS fill(const BRep& a, const BRep& b, double fuzzy);

/// Where IntAna_QuadQuadGeo starts the circle or ellipse that the plane through o with normal n cuts from a sphere (kind 2, centre c, radius size), a cylinder (3, axis through c, radius size) or, across its axis, a cone (4): the X direction DirToAx2 gives; false for any other section
bool conic_origin(const Point& o, Vector n, int kind, const Point& c, const Vector& axis, double size, const Point& fallback, Point& origin);

/// A flat bilinear patch S(u, v) = origin + a s + b r with s, r its normalized parameters
bool affine_patch(const NurbsSurface& surface, Point& origin, Vector& a, Vector& b);

/// The pcurve of a curve's piece [t0, t1] on a surface it lies on, its parameter affine in the curve's, in the surface's true parameters: a 2D line where the piece runs at one speed along one (a line on a plane, a ruling or a circle about the axis), a 2D circle or ellipse under one in a plane, exact on a flat bilinear patch, interpolated elsewhere
Curve project(const Curve& curve, double t0, double t1, const Surface& surface);

/// The parameter in [lo, hi] of the point of a curve's piece nearest to p: by closed form on an analytic curve, on the turn of a circle or ellipse that passes the piece
double parameter(const Curve& curve, const Point& p, double lo, double hi);

/// The parameter of a curve at a parameter of its nurbs(): the angle on a circle or ellipse, whose NURBS runs rationally between its knots
double true_parameter(const Curve& curve, double t);

/// The parameters of a surface under a point of its nurbs(), each angle likewise
Point true_parameters(const Surface& surface, double u, double v);

/// How many pieces a curve is sampled in, and whether it is straight
int samples(const Curve& curve);
bool straight(const Curve& curve);

/// The range of a pcurve under [t0, t1] of a curve whose parameter maps affinely from `domain` onto the pcurve's domain
Span ranged(const std::shared_ptr<const Curve>& pcurve, std::pair<double, double> domain, double t0, double t1);

}
