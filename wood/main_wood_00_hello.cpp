// main_wood_00_hello.cpp — minimal face-to-face example with explicit session.
//
// Two 1000×500×15 mm plates sharing the Y=0 edge.
// Output: session_data/WoodF2F_hello.pb
//
// ── Rhino viewer (paste into Rhino 8 ScriptEditor, venv: session_py) ──────
//
//   #! python3
//   # venv: session_py
//
//   import importlib
//   import session_rhino.session
//   importlib.reload(session_rhino.session)
//   from session_rhino.session import Session
//
//   filepath = r"C:\brg\code_rust\session\session_data\WoodF2F_hello.pb"
//
//   scene = Session.load(filepath)
//   scene.draw(delete=True)
//
// ─────────────────────────────────────────────────────────────────────────
#include "wood/wood_session.h"
#include "../src/session.h"
using namespace session_cpp;

int main() {
    using namespace wood_session::globals;
    reset_defaults();

    std::vector<Polyline> polylines = {
        // element 0 — top face (Z = 0)
        Polyline({
            {-500,   0,   0},
            { 500,   0,   0},
            { 500, 500,   0},
            {-500, 500,   0},
            {-500,   0,   0},
        }),
        // element 0 — bottom face (Z = -15)
        Polyline({
            {-500,   0, -15},
            { 500,   0, -15},
            { 500, 500, -15},
            {-500, 500, -15},
            {-500,   0, -15},
        }),
        // element 1 — top face (shares Y=0 edge)
        Polyline({
            {-500, -500,   0},
            { 500, -500,   0},
            { 500,   0,   0},
            {-500,   0,   0},
            {-500, -500,   0},
        }),
        // element 1 — bottom face
        Polyline({
            {-500, -500, -15},
            { 500, -500, -15},
            { 500,   0, -15},
            {-500,   0, -15},
            {-500, -500, -15},
        }),
    };

    Session session("WoodF2F");

    // Add input plates so they appear as a group in the viewer.
    auto g_input = session.add_group("InputPlates");
    for (size_t i = 0; i < polylines.size(); i++) {
        auto pl = std::make_shared<Polyline>(polylines[i]);
        pl->name = "plate_" + std::to_string(i);
        session.add_polyline(pl, g_input);
    }

    // Build ElementPlates from the flat polyline list (even=bottom, odd=top).
    std::vector<ElementPlate> plates;
    for (size_t i = 0; i + 1 < polylines.size(); i += 2)
        plates.emplace_back(polylines[i], polylines[i+1], "plate_" + std::to_string(i/2));
    using namespace wood_session::globals;
    DATA_SET_INPUT_NAME  = "hello";
    DATA_SET_OUTPUT_FILE = "WoodF2F_hello.pb";
    auto merged = get_connection_zones(plates, session, face_to_face);
    loft_merged_elements(session, merged);

    // Write to protobuf.
    auto pb = (internal::session_data_dir() / "WoodF2F_hello.pb").string();
    session.pb_dump(pb);
    return 0;
}
