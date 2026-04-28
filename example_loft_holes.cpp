// Standalone loft-with-holes example — demonstrates the three problematic
// polyline shapes the recent Mesh::loft fixes now handle correctly:
//   plate_rect    - simple rectangle + one rectangular hole (baseline)
//   plate_annen   - 12-vert concave outline with 3 trapezoidal teeth + 2 holes
//                   (the annen_corner plate_0 shape that triggered the CDT bug)
//   plate_collin  - rectangle whose bottom edge is subdivided by 2 collinear
//                   midpoints (exercises the strip_shared_collinear pre-pass)
// Writes session_data/example_loft_holes_cpp.pb — load in Rhino to inspect.

#include <filesystem>
#include "src/session.h"
#include "src/mesh.h"
#include "src/polyline.h"
using namespace session_cpp;

int main() {
    Session session("LoftHoles");
    auto g_mesh = session.add_group("Meshes");

    // ── plate_rect ────────────────────────────────────────────────────────
    std::vector<Polyline> rect_bot = {
        Polyline({
            { 0,  0, 0},
            {10,  0, 0},
            {10, 10, 0},
            { 0, 10, 0},
            { 0,  0, 0},
        }),
        Polyline({
            {3, 3, 0},
            {3, 7, 0},
            {7, 7, 0},
            {7, 3, 0},
            {3, 3, 0},
        }),
    };
    std::vector<Polyline> rect_top = {
        Polyline({
            { 0,  0, 2},
            {10,  0, 2},
            {10, 10, 2},
            { 0, 10, 2},
            { 0,  0, 2},
        }),
        Polyline({
            {3, 3, 2},
            {3, 7, 2},
            {7, 7, 2},
            {7, 3, 2},
            {3, 3, 2},
        }),
    };
    auto m_rect = std::make_shared<Mesh>(Mesh::loft(rect_bot, rect_top));
    m_rect->name = "plate_rect";
    session.add_mesh(m_rect, g_mesh);

    // ── plate_annen (plate_0 from annen_corner) ───────────────────────────
    // Vertical plate at X≈2142/2223; 3 joint "teeth" along the bottom edge;
    // two rectangular through-holes for cross-joints.
    std::vector<Polyline> annen_bot = {
        Polyline({
            {2142.008, -530.170, 1172.487},
            {2142.008, -530.170, -318.768},
            {2142.008, -318.102, -318.768},
            {2142.008, -347.792, -414.110},
            {2142.008, -106.034, -414.110},
            {2142.008, -135.724, -318.768},
            {2142.008,  106.034, -318.768},
            {2142.008,   76.344, -414.110},
            {2142.008,  318.102, -414.110},
            {2142.008,  288.412, -318.768},
            {2142.008,  530.170, -318.768},
            {2142.008,  530.170, 1172.487},
            {2142.008, -530.170, 1172.487},
        }),
        Polyline({
            {2142.008, 97.448,  841.097},
            {2142.008,  0.000,  841.097},
            {2142.008,  0.000, 1006.792},
            {2142.008, 97.448, 1006.792},
            {2142.008, 97.448,  841.097},
        }),
        Polyline({
            {2142.008, 97.448, 178.317},
            {2142.008,  0.000, 178.317},
            {2142.008,  0.000, 344.012},
            {2142.008, 97.448, 344.012},
            {2142.008, 97.448, 178.317},
        }),
    };
    std::vector<Polyline> annen_top = {
        Polyline({
            {2223.416, -530.170, 1172.487},
            {2223.416, -530.170, -269.141},
            {2223.416, -318.102, -269.141},
            {2223.416, -347.792, -364.483},
            {2223.416, -106.034, -364.483},
            {2223.416, -135.724, -269.141},
            {2223.416,  106.034, -269.141},
            {2223.416,   76.344, -364.483},
            {2223.416,  318.102, -364.483},
            {2223.416,  288.412, -269.141},
            {2223.416,  530.170, -269.141},
            {2223.416,  530.170, 1172.487},
            {2223.416, -530.170, 1172.487},
        }),
        Polyline({
            {2223.416, 97.448,  841.097},
            {2223.416,  0.000,  841.097},
            {2223.416,  0.000, 1006.792},
            {2223.416, 97.448, 1006.792},
            {2223.416, 97.448,  841.097},
        }),
        Polyline({
            {2223.416, 97.448, 178.317},
            {2223.416,  0.000, 178.317},
            {2223.416,  0.000, 344.012},
            {2223.416, 97.448, 344.012},
            {2223.416, 97.448, 178.317},
        }),
    };
    auto m_annen = std::make_shared<Mesh>(Mesh::loft(annen_bot, annen_top));
    m_annen->name = "plate_annen";
    session.add_mesh(m_annen, g_mesh);

    // ── plate_collin ──────────────────────────────────────────────────────
    // 8-vert rectangle: bottom edge split by 2 collinear midpoints. The CDT
    // internally strips them; without the pre-pass, side walls would create
    // naked edges at those midpoints.
    std::vector<Polyline> col_bot = {
        Polyline({
            { 0, 0, 0},
            { 4, 0, 0},
            { 7, 0, 0},
            {12, 0, 0},
            {12, 5, 0},
            { 0, 5, 0},
            { 0, 0, 0},
        }),
    };
    std::vector<Polyline> col_top = {
        Polyline({
            { 0, 0, 1.5},
            { 4, 0, 1.5},
            { 7, 0, 1.5},
            {12, 0, 1.5},
            {12, 5, 1.5},
            { 0, 5, 1.5},
            { 0, 0, 1.5},
        }),
    };
    auto m_collin = std::make_shared<Mesh>(Mesh::loft(col_bot, col_top));
    m_collin->name = "plate_collin";
    session.add_mesh(m_collin, g_mesh);

    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto pb = (base / "session_data" / "example_loft_holes_cpp.pb").string();
    session.pb_dump(pb);

    for (auto& m : {m_rect, m_annen, m_collin}) {
        std::cout << m->name << ": V=" << m->vertex.size()
                  << " F=" << m->face.size()
                  << " closed=" << m->is_closed() << std::endl;
    }
    std::cout << "wrote " << pb << std::endl;
    return 0;
}
