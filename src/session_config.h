#pragma once

namespace session_cpp {

/** @class SessionConfig
 * @brief Runtime settings used by session operations.
 */
class SessionConfig {
public:
    bool explode_mesh_faces = false; ///< Whether meshing emits one face per triangle.
    double scale_factor = 1.0;       ///< Scale applied by external session adapters.

    /// Restores every setting to its default value.
    void reset() {
        explode_mesh_faces = false;
        scale_factor = 1.0;
    }
};

/// Process-wide settings used by session operations.
extern SessionConfig SESSION_CONFIG;

} // namespace session_cpp
