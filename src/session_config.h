#pragma once

namespace session_cpp {

class SessionConfig {
public:
    bool explode_mesh_faces = false;

    void reset() { explode_mesh_faces = false; }
};

extern SessionConfig SESSION_CONFIG;

} // namespace session_cpp
