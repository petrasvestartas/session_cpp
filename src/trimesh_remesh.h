#pragma once

#include "nurbssurface.h"
#include "mesh.h"
#include "tolerance.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// FlatMap64 (reuse from trimesh_delaunay.h)
///////////////////////////////////////////////////////////////////////////////////////////

template<typename V> class FlatMap64;

///////////////////////////////////////////////////////////////////////////////////////////
// HalfedgeMesh — lightweight halfedge structure for remeshing
///////////////////////////////////////////////////////////////////////////////////////////

struct RVertex {
    double u, v;
    double px, py, pz;
    double nx, ny, nz;
    int halfedge = -1;
    uint8_t flags = 0;
    static constexpr uint8_t POLE = 1;
    static constexpr uint8_t BOUNDARY = 2;
    static constexpr uint8_t DELETED = 4;
};

struct RHalfedge {
    int next = -1;
    int twin = -1;
    int vertex = -1;   // head vertex
    int face = -1;
};

struct RFace {
    int halfedge = -1;  // -1 if deleted
};

struct HalfedgeMesh {
    std::vector<RVertex> verts;
    std::vector<RHalfedge> hedges;
    std::vector<RFace> faces;

    int add_vertex(double u, double v);
    int add_face(int va, int vb, int vc);
    int tail(int he) const { return hedges[hedges[hedges[he].next].next].vertex; }
    bool is_boundary_vertex(int vi) const { return (verts[vi].flags & RVertex::BOUNDARY) != 0; }
    bool is_deleted_vertex(int vi) const { return (verts[vi].flags & RVertex::DELETED) != 0; }
    bool is_pole(int vi) const { return (verts[vi].flags & RVertex::POLE) != 0; }
    int valence(int vi) const;
    double face_area(int fi) const;
    double edge_length_3d(int he) const;
};

///////////////////////////////////////////////////////////////////////////////////////////
// TrimeshRemesh — Kangaroo2-style iterative remesher
///////////////////////////////////////////////////////////////////////////////////////////

class TrimeshRemesh {
public:
    explicit TrimeshRemesh(const NurbsSurface& surface);

    TrimeshRemesh& set_max_angle(double degrees);
    TrimeshRemesh& set_max_edge_length(double length);
    TrimeshRemesh& set_min_edge_length(double length);
    TrimeshRemesh& set_max_chord_height(double height);
    TrimeshRemesh& set_iterations(int iterations);

    Mesh mesh() const;

private:
    const NurbsSurface& m_surface;
    double m_max_angle = 20.0;
    double m_max_edge_length = 0.0;
    double m_min_edge_length = 0.0;
    double m_max_chord_height = 0.0;
    int m_iterations = 25;

    double compute_bbox_diagonal() const;

    void build_initial_mesh(HalfedgeMesh& hm) const;
    void split_long_edges(HalfedgeMesh& hm, double max_len) const;
    void collapse_short_edges(HalfedgeMesh& hm, double min_len) const;
    void flip_edges(HalfedgeMesh& hm) const;
    void smooth_vertices(HalfedgeMesh& hm, double strength) const;
    void project_to_surface(HalfedgeMesh& hm) const;
    void compact(HalfedgeMesh& hm) const;
    Mesh to_output_mesh(const HalfedgeMesh& hm) const;
};

} // namespace session_cpp
