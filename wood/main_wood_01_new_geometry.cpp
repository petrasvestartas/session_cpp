// main_wood_01_new_geometry.cpp — annen_corner from hardcoded polylines.
//
// Coordinates extracted verbatim from session_data/annen_corner.obj.
// 6 elements (12 polylines), same joint parameters as the file-backed test.
// Output: session_data/WoodF2F_annen_corner_custom.pb
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
//   filepath = r"C:\brg\code_rust\session\session_data\WoodF2F_annen_corner_custom.pb"
//
//   scene = Session.load(filepath)
//   scene.draw(delete=True)
//
// ─────────────────────────────────────────────────────────────────────────
#include "wood/wood_session.h"
#include "../src/session.h"

using namespace session_cpp;
using namespace wood_session;

int main() {

    // Load global wood parameters.
    globals::reset_defaults();
    globals::JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    globals::JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    globals::DATA_SET_INPUT_NAME  = "annen_corner_custom";
    globals::DATA_SET_OUTPUT_FILE = "WoodF2F_annen_corner_custom.pb";

    // Main Input - Polylines
    std::vector<Polyline> polylines = {
        // pair 0 — vertical plate at X≈2142 (top)
        Polyline({
            { 2142.00812383331, -530.170014651827,  1172.48735988095},
            { 2142.00812383328, -530.170014651827,  -318.768161457628},
            { 2142.00812383328,  530.170014651827,  -318.768161457628},
            { 2142.00812383331,  530.170014651827,  1172.48735988095},
            { 2142.00812383331, -530.170014651827,  1172.48735988095},
        }),
        // pair 0 (bottom)
        Polyline({
            { 2223.41611737943, -530.170014651827,  1172.48735988095},
            { 2223.4161173794,  -530.170014651827,  -364.483096862165},
            { 2223.4161173794,   530.170014651827,  -364.483096862165},
            { 2223.41611737943,  530.170014651827,  1172.48735988095},
            { 2223.41611737943, -530.170014651827,  1172.48735988095},
        }),
        // pair 1 — diagonal plate (top)
        Polyline({
            {  868.697507990728, -530.170014651827, -1094.99177969239},
            { 2142.00812383328,  -530.170014651827,  -318.768161457628},
            { 2142.00812383328,   530.170014651827,  -318.768161457628},
            {  868.697507990728,  530.170014651827, -1094.99177969239},
            {  868.697507990728, -530.170014651827, -1094.99177969239},
        }),
        // pair 1 (bottom)
        Polyline({
            {  868.697507990728, -530.170014651827, -1190.33388895175},
            { 2223.4161173794,   -530.170014651827,  -364.483096862165},
            { 2223.4161173794,    530.170014651827,  -364.483096862165},
            {  868.697507990728,  530.170014651827, -1190.33388895175},
            {  868.697507990728, -530.170014651827, -1190.33388895175},
        }),
        // pair 2 — central connector at Y≈97 (top)
        Polyline({
            {  868.697507990739, 97.4481444578238,  396.263741646029},
            { 2142.00812383329,  97.4481444579578, 1172.48735988079},
            { 2142.00812383328,  97.4481444582489, -318.768161457628},
            {  868.697507990728, 97.4481444581148, -1094.99177969239},
            {  868.697507990739, 97.4481444578238,  396.263741646029},
        }),
        // pair 2 (bottom, Y≈0)
        Polyline({
            {  868.697507990739, -4.39627001469489e-10,  396.263741646029},
            { 2142.00812383329,  -3.05590219795704e-10, 1172.48735988079},
            { 2142.00812383328,  -1.45519152283669e-11, -318.768161457628},
            {  868.697507990728, -1.48588696902152e-10, -1094.99177969239},
            {  868.697507990739, -4.39627001469489e-10,  396.263741646029},
        }),
        // pair 3 — vertical plate at X≈-633 (top)
        Polyline({
            { -632.906073423899, -530.170014651827, 1190.33388895175},
            { -632.906073423932, -530.170014651827, -400.176155003755},
            { -632.906073423932,  530.170014651827, -400.176155003755},
            { -632.906073423899,  530.170014651827, 1190.33388895175},
            { -632.906073423899, -530.170014651827, 1190.33388895175},
        }),
        // pair 3 (bottom, X≈-714)
        Polyline({
            { -714.314066970026, -530.170014651827, 1190.33388895175},
            { -714.314066970057, -530.170014651827, -318.768161457628},
            { -714.314066970057,  530.170014651827, -318.768161457628},
            { -714.314066970026,  530.170014651827, 1190.33388895175},
            { -714.314066970026, -530.170014651827, 1190.33388895175},
        }),
        // pair 4 — horizontal plate at Z≈-400 (top)
        Polyline({
            { -2223.41611737943, -530.170014651827, -400.176155003755},
            {  -632.906073423932, -530.170014651827, -400.176155003755},
            {  -632.906073423932,  530.170014651827, -400.176155003755},
            { -2223.41611737943,  530.170014651827, -400.176155003755},
            { -2223.41611737943, -530.170014651827, -400.176155003755},
        }),
        // pair 4 (bottom, Z≈-319)
        Polyline({
            { -2223.41611737943, -530.170014651827, -318.768161457628},
            {  -714.314066970057, -530.170014651827, -318.768161457628},
            {  -714.314066970057,  530.170014651827, -318.768161457628},
            { -2223.41611737943,  530.170014651827, -318.768161457628},
            { -2223.41611737943, -530.170014651827, -318.768161457628},
        }),
        // pair 5 — connector at Y≈0 (top)
        Polyline({
            { -2223.41611737942, -3.05590219795704e-10, 1190.33388895159},
            {  -714.314066970046, -3.05590219795704e-10, 1190.33388895159},
            {  -714.314066970057, -1.45519152283669e-11, -318.768161457628},
            { -2223.41611737943, -1.45519152283669e-11, -318.768161457628},
            { -2223.41611737942, -3.05590219795704e-10, 1190.33388895159},
        }),
        // pair 5 (bottom, Y≈97)
        Polyline({
            { -2223.41611737942, 97.4481444579578, 1190.33388895159},
            {  -714.314066970046, 97.4481444579578, 1190.33388895159},
            {  -714.314066970057, 97.4481444582489, -318.768161457628},
            { -2223.41611737943, 97.4481444582489, -318.768161457628},
            { -2223.41611737942, 97.4481444579578, 1190.33388895159},
        }),
    };

    // Build WoodElements from the flat polyline list (even=bottom, odd=top).
    std::vector<WoodElement> elements;
    for (size_t i = 0; i + 1 < polylines.size(); i += 2)
        elements.emplace_back(polylines[i], polylines[i+1]);

    // Run the joint-detection algorithm.
    std::vector<WoodJoint> joints = get_connection_zones(elements, face_to_face);

    // Session for visualization and export.
    Session session(globals::DATA_SET_INPUT_NAME);
    std::shared_ptr<TreeNode> g_input = session.add_group("InputPlates");
    for (size_t i = 0; i < polylines.size(); i++) {
        std::shared_ptr<Polyline> pl = std::make_shared<Polyline>(polylines[i]);
        pl->name = "plate_" + std::to_string(i);
        session.add_polyline(pl, g_input);
    }
    fill_session(session, elements, joints);

    session.pb_dump((internal::session_data_dir() / globals::DATA_SET_OUTPUT_FILE).string());
    return 0;
}
