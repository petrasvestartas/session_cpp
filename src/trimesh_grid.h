#pragma once

#include "nurbssurface.h"
#include "mesh.h"
#include "tolerance.h"

namespace session_cpp {

class TrimeshGrid {
public:
    explicit TrimeshGrid(const NurbsSurface& surface);

    TrimeshGrid& set_max_angle(double degrees);
    TrimeshGrid& set_max_edge_length(double length);
    TrimeshGrid& set_min_edge_length(double length);
    TrimeshGrid& set_max_chord_height(double height);

    double get_max_angle() const { return m_max_angle; }
    double get_max_edge_length() const { return m_max_edge_length; }
    double get_min_edge_length() const { return m_min_edge_length; }
    double get_max_chord_height() const { return m_max_chord_height; }

    Mesh mesh() const;

private:
    const NurbsSurface& m_surface;
    double m_max_angle = 20.0;
    double m_max_edge_length = 0.0;
    double m_min_edge_length = 0.0;
    double m_max_chord_height = 0.0;

    double compute_bbox_diagonal() const;
};

} // namespace session_cpp
