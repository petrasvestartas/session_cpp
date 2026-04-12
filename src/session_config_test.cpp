#include "mini_test.h"
#include "session_config.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("SessionConfig", "Runtime Modification") {
        // uncomment #include "session_config.h"
        // SessionConfig holds global rendering/export flags.
        // explode_mesh_faces: when true, each mesh face is a separate mesh (for coloring)
        // scale_factor: unit conversion multiplier applied during export (1.0 = meters)
        // Modify config at runtime, then reset() restores all defaults.

        MINI_CHECK(!SESSION_CONFIG.explode_mesh_faces);
        SESSION_CONFIG.explode_mesh_faces = true;
        MINI_CHECK(SESSION_CONFIG.explode_mesh_faces);
        MINI_CHECK(std::abs(SESSION_CONFIG.scale_factor - 1.0) < 1e-10);
        SESSION_CONFIG.scale_factor = 0.001;
        MINI_CHECK(std::abs(SESSION_CONFIG.scale_factor - 0.001) < 1e-10);
        SESSION_CONFIG.reset();
        MINI_CHECK(!SESSION_CONFIG.explode_mesh_faces);
        MINI_CHECK(std::abs(SESSION_CONFIG.scale_factor - 1.0) < 1e-10);
    }

}
