#pragma once

namespace session_cpp {

class SessionConfig {
public:
    bool explode_mesh_faces = false;
    double scale_factor = 1.0;

    void reset() { explode_mesh_faces = false; scale_factor = 1.0; }
};

extern SessionConfig SESSION_CONFIG;

} // namespace session_cpp
