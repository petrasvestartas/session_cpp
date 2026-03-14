#include <cassert>
#include <filesystem>
#include <format>
#include "mesh.h"
#include "session.h"

using namespace session_cpp;

int main() {

    // --- multi-panel: find adjacent panels via shared wall edges ---
    std::vector<std::vector<Point>> top7 = {
        {{250,-250,500},{250,250,500},{-250,250,500},{-250,-250,500},{250,-250,500}},
        {{-250,500,250},{-250,250,500},{250,250,500},{250,500,250},{-250,500,250}},
        {{250,-250,500},{500,-250,250},{500,250,250},{250,250,500},{250,-250,500}},
        {{250,500,250},{250,250,500},{500,250,250},{250,500,250}},
        {{-250,500,250},{250,500,250},{250,500,-250},{-250,500,-250},{-250,500,250}},
        {{250,500,250},{500,250,250},{500,250,-250},{250,500,-250},{250,500,250}},
        {{500,-250,250},{500,-250,-250},{500,250,-250},{500,250,250},{500,-250,250}},
    };
    std::vector<std::vector<Point>> bot7 = {
        {{270.710678,-250,550},{270.710678,265.891862,550},{265.891862,270.710678,550},{-250,270.710678,550},{-250,-250,550},{270.710678,-250,550}},
        {{270.710678,-250,550},{550,-250,270.710678},{550,265.891862,270.710678},{270.710678,265.891862,550},{270.710678,-250,550}},
        {{-250,550,270.710678},{-250,270.710678,550},{265.891862,270.710678,550},{265.891862,550,270.710678},{-250,550,270.710678}},
        {{265.891862,550,270.710678},{265.891862,270.710678,550},{270.710678,265.891862,550},{550,265.891862,270.710678},{550,270.710678,265.891862},{270.710678,550,265.891862},{265.891862,550,270.710678}},
        {{-250,550,270.710678},{265.891862,550,270.710678},{270.710678,550,265.891862},{270.710678,550,-250},{-250,550,-250},{-250,550,270.710678}},
        {{270.710678,550,265.891862},{550,270.710678,265.891862},{550,270.710678,-250},{270.710678,550,-250},{270.710678,550,265.891862}},
        {{550,-250,270.710678},{550,-250,-250},{550,270.710678,-250},{550,270.710678,265.891862},{550,265.891862,270.710678},{550,-250,270.710678}},
    };
    auto [panels, adj, top_mesh, bot_mesh] = Mesh::loft_panels(top7, bot7);

    // --- visual debug dump ---
    Session session("loft_debug");

    auto top_mesh_layer = session.add_group("top_mesh");
    session.add(session.add_mesh(std::make_shared<Mesh>(top_mesh)), top_mesh_layer);
    auto bot_mesh_layer = session.add_group("bot_mesh");
    session.add(session.add_mesh(std::make_shared<Mesh>(bot_mesh)), bot_mesh_layer);

    // panel meshes: blue=top cap, red=bot cap, gray=quad wall, yellow=tri wall
    auto layer_panels = session.add_group("panels");
    for (size_t i = 0; i < panels.size(); i++) {
        std::vector<Color> face_colors;
        face_colors.reserve(panels[i].face_roles.size());
        for (const auto& [face_key, role] : panels[i].face_roles) {
            switch (role) {
                case LoftFaceRole::TopCap:   face_colors.push_back(Color::blue()); break;
                case LoftFaceRole::BotCap:   face_colors.push_back(Color::red());  break;
                case LoftFaceRole::TriWall:  face_colors.push_back(Color::yellow()); break;
                case LoftFaceRole::QuadWall: face_colors.push_back(Color::grey());break;
            }
        }
        panels[i].mesh.set_facecolors(face_colors);
        session.add(session.add_mesh(std::make_shared<Mesh>(panels[i].mesh)), layer_panels);
    }

    // face centroids labelled with panel index
    auto layer_centroids = session.add_group("centroids");
    for (size_t i = 0; i < panels.size(); i++) {
        auto c = panels[i].mesh.centroid();
        c.name = std::format("p{}", i);
        session.add(session.add_point(std::make_shared<Point>(c)), layer_centroids);
    }

    // adjacency: for each shared edge — text dot at midpoint labelled "p{i}f{idx}<->p{j}f{idx}"
    auto layer_adj = session.add_group("adjacency");
    for (const auto& [i, wi, pj, wj] : adj) {
        const auto& w = panels[i].wall_faces[wi];
        auto pt = *panels[i].mesh.face_centroid(w.face_key);
        pt.name = std::format("p{} f{} - p{} f{}", i, w.face_index, pj, panels[pj].wall_faces[wj].face_index);
        session.add(session.add_point(std::make_shared<Point>(pt)), layer_adj);
    }

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "loft_debug.pb").string();
    session.pb_dump(fp);

    return 0;
}
