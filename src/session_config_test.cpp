#include "mini_test.h"
#include "session_config.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("SessionConfig", "Runtime Modification") {
        SESSION_CONFIG.reset();
        SessionConfig config;
        SessionConfig other;

        MINI_CHECK(!config.explode_mesh_faces);
        MINI_CHECK(config.scale_factor == 1.0);
        config.explode_mesh_faces = true;
        config.scale_factor = 0.001;
        MINI_CHECK(config.explode_mesh_faces);
        MINI_CHECK(config.scale_factor == 0.001);
        MINI_CHECK(!other.explode_mesh_faces);
        MINI_CHECK(other.scale_factor == 1.0);
        MINI_CHECK(!SESSION_CONFIG.explode_mesh_faces);
        MINI_CHECK(SESSION_CONFIG.scale_factor == 1.0);
        SESSION_CONFIG.explode_mesh_faces = true;
        SESSION_CONFIG.scale_factor = 0.001;
        MINI_CHECK(SESSION_CONFIG.explode_mesh_faces);
        MINI_CHECK(SESSION_CONFIG.scale_factor == 0.001);
        SESSION_CONFIG.reset();
        MINI_CHECK(!SESSION_CONFIG.explode_mesh_faces);
        MINI_CHECK(SESSION_CONFIG.scale_factor == 1.0);
        config.reset();
        MINI_CHECK(!config.explode_mesh_faces);
        MINI_CHECK(config.scale_factor == 1.0);
    }

}
