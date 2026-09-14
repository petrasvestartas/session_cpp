#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

/// Adaptive mesh of a NURBS surface: a quadtree in UV split where normals turn or chords deviate, T-junctions fanned, poles and seams shared
class RemeshNurbsSurfaceAdaptive {
public:
    explicit RemeshNurbsSurfaceAdaptive(const NurbsSurface& surface);

    /// Largest normal turn across a cell in degrees, 20 by default
    RemeshNurbsSurfaceAdaptive& set_max_angle(double degrees);

    /// Longest cell edge; 0 for no limit
    RemeshNurbsSurfaceAdaptive& set_max_edge_length(double length);

    /// Shortest cell edge still split; 0 for no limit
    RemeshNurbsSurfaceAdaptive& set_min_edge_length(double length);

    /// Largest chord height; 0 for 0.5 percent of the bbox diagonal
    RemeshNurbsSurfaceAdaptive& set_max_chord_height(double height);

    double get_max_angle() const { return m_max_angle; }
    double get_max_edge_length() const { return m_max_edge_length; }
    double get_min_edge_length() const { return m_min_edge_length; }
    double get_max_chord_height() const { return m_max_chord_height; }

    /// Triangle mesh with u, v vertex attributes and fan normals
    Mesh mesh() const;

private:
    const NurbsSurface& m_surface;
    double m_max_angle = 20.0;
    double m_max_edge_length = 0.0;
    double m_min_edge_length = 0.0;
    double m_max_chord_height = 0.0;
};

} // namespace session_cpp
