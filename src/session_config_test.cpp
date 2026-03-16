#include "mini_test.h"
#include "session_config.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("SessionConfig", "Default Values") {
        MINI_CHECK(SESSION_CONFIG.explode_mesh_faces == false);
    }

    MINI_TEST("SessionConfig", "Runtime Modification") {
        MINI_CHECK(SESSION_CONFIG.explode_mesh_faces == false);
        SESSION_CONFIG.explode_mesh_faces = true;
        MINI_CHECK(SESSION_CONFIG.explode_mesh_faces == true);
        SESSION_CONFIG.reset();
        MINI_CHECK(SESSION_CONFIG.explode_mesh_faces == false);
    }

}
