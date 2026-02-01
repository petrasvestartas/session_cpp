#pragma once

#include "nurbssurface.h"
#include "mesh.h"
#include "point.h"
#include "vector.h"
#include "tolerance.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Delaunay2D - Internal 2D constrained Delaunay triangulation engine (Bowyer-Watson)
///////////////////////////////////////////////////////////////////////////////////////////

struct Vertex2D {
    double x = 0.0;
    double y = 0.0;
};

struct Triangle {
    int v[3] = {-1, -1, -1};       // vertex indices (CCW order)
    int adj[3] = {-1, -1, -1};     // adjacent triangle index opposite each vertex
    bool constrained[3] = {false, false, false}; // is edge opposite v[i] a constraint?
    bool alive = true;
};

class Delaunay2D {
public:
    std::vector<Vertex2D> vertices;
    std::vector<Triangle> triangles;
    int super_v[3] = {-1, -1, -1};

    Delaunay2D(double xmin, double ymin, double xmax, double ymax);

    int insert(double x, double y);
    void insert_constraint(int v0, int v1);
    void cleanup();
    std::vector<std::array<int, 3>> get_triangles() const;

    static double in_circumcircle(double ax, double ay, double bx, double by,
                                   double cx, double cy, double dx, double dy);
    static double orient2d(double ax, double ay, double bx, double by,
                           double cx, double cy);
    static void circumcenter(double ax, double ay, double bx, double by,
                             double cx, double cy, double& ux, double& uy);

private:
    void update_adjacency(int tri_idx, int va, int vb, int new_neighbor);
    void link_new_triangles(const std::vector<int>& new_tris);
    int find_triangle_containing(double x, double y) const;
    bool edge_intersects_segment(int ea, int eb, int sa, int sb) const;
    void legalize_edge(int tri_idx, int edge_idx);
};

///////////////////////////////////////////////////////////////////////////////////////////
// NurbsTriangulation - Surface-aware mesher (public API)
///////////////////////////////////////////////////////////////////////////////////////////

class NurbsTriangulation {
public:
    explicit NurbsTriangulation(const NurbsSurface& surface);

    NurbsTriangulation& set_max_angle(double degrees);
    NurbsTriangulation& set_max_edge_length(double length);
    NurbsTriangulation& set_min_edge_length(double length);
    NurbsTriangulation& set_max_chord_height(double height);
    NurbsTriangulation& set_max_iterations(int iterations);

    double get_max_angle() const { return m_max_angle; }
    double get_max_edge_length() const { return m_max_edge_length; }
    double get_min_edge_length() const { return m_min_edge_length; }
    double get_max_chord_height() const { return m_max_chord_height; }
    int get_max_iterations() const { return m_max_iterations; }

    Mesh mesh() const;

private:
    const NurbsSurface& m_surface;
    double m_max_angle = 20.0;
    double m_max_edge_length = 0.0;
    double m_min_edge_length = 0.0;
    double m_max_chord_height = 0.0;
    int m_max_iterations = 5000;

    double compute_bbox_diagonal() const;
    int find_worst_triangle(const Delaunay2D& dt, double max_edge, double chord_tol,
                            double angle_rad, double min_edge) const;
};

} // namespace session_cpp
