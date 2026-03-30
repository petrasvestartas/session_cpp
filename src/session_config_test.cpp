#include "mini_test.h"
#include "session_config.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("SessionConfig", "Default Values") {
        MINI_CHECK(!SESSION_CONFIG.explode_mesh_faces);
        MINI_CHECK(std::abs(SESSION_CONFIG.scale_factor - 1.0) < 1e-10);
    }

    MINI_TEST("SessionConfig", "Runtime Modification") {
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
