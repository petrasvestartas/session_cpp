#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

/// Adaptive mesh of a NURBS surface: a quadtree in UV split where normals turn or chords deviate, T-junctions fanned, poles and seams shared.
class RemeshNurbsSurfaceAdaptive {
private:
    const NurbsSurface& m_surface; // Surface to mesh.
    double m_max_angle = 20.0; // Largest normal turn across a cell in degrees.
    double m_max_edge_length = 0.0; // Longest cell edge, 0 for no limit.
    double m_min_edge_length = 0.0; // Shortest cell edge still split, 0 for no limit.
    double m_max_chord_height = 0.0; // Largest chord height, 0 for 0.5 percent of the bbox diagonal.

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct over a surface with the default tolerances.
    explicit RemeshNurbsSurfaceAdaptive(const NurbsSurface& surface);

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the largest normal turn in degrees.
    double get_max_angle() const {
        return m_max_angle;
    }

    /// Return the longest cell edge.
    double get_max_edge_length() const {
        return m_max_edge_length;
    }

    /// Return the shortest cell edge still split.
    double get_min_edge_length() const {
        return m_min_edge_length;
    }

    /// Return the largest chord height.
    double get_max_chord_height() const {
        return m_max_chord_height;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Largest normal turn across a cell in degrees, 20 by default.
    RemeshNurbsSurfaceAdaptive& set_max_angle(double degrees);

    /// Longest cell edge; 0 for no limit.
    RemeshNurbsSurfaceAdaptive& set_max_edge_length(double length);

    /// Shortest cell edge still split; 0 for no limit.
    RemeshNurbsSurfaceAdaptive& set_min_edge_length(double length);

    /// Largest chord height; 0 for 0.5 percent of the bbox diagonal.
    RemeshNurbsSurfaceAdaptive& set_max_chord_height(double height);

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Triangle mesh with u, v vertex attributes and fan normals.
    Mesh mesh() const;
};

} // namespace session_cpp
