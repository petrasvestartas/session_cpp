#include "session.h"

using namespace session_cpp;

int main() {

    Session session;


    std::vector<Point> vertices = Polyline::from_sides(6, 1.0, false).get_points();
    Mesh mesh = Mesh::from_vertices_and_faces(vertices, {{0, 1, 2, 3, 4, 5}});
    std::string sstr = mesh.str();
    std::string srepr = mesh.repr();
    Mesh mcopy = mesh;
    mesh.name = "hexagon";

    std::vector<Color> palette = Color::palette();

    // set_objectcolor does not change color_mode
    mesh.set_objectcolor(Color::grey());

    // set_pointcolors → color_mode = PointColors
    std::vector<Color> pc;
    pc.reserve(mesh.number_of_vertices());
    for (size_t i = 0; i < mesh.number_of_vertices(); ++i)
        pc.emplace_back(palette[i % palette.size()]);
    mesh.set_pointcolors(std::move(pc));

    // set_facecolors → color_mode = FaceColors
    std::vector<Color> fc;
    fc.reserve(mesh.number_of_faces());
    for (size_t i = 0; i < mesh.number_of_faces(); ++i)
        fc.emplace_back(palette[i % palette.size()]);
    mesh.set_facecolors(std::move(fc));

    // set_linecolors does not change color_mode
    std::vector<Color> lc;
    std::vector<double> lw(mesh.number_of_edges(), 0.1);
    lc.reserve(mesh.number_of_edges());
    for (size_t i = 0; i < mesh.number_of_edges(); ++i)
        lc.emplace_back(palette[i % palette.size()]);
    mesh.set_linecolors(std::move(lc), std::move(lw));

    // // clear_facecolors reverts color_mode only if currently FaceColors
    // mesh.color_mode = ColorMode::FACECOLORS;
    // mesh.clear_facecolors();

    // // clear_pointcolors does not revert if color_mode != PointColors
    // mesh.color_mode = ColorMode::FACECOLORS;
    // mesh.clear_pointcolors();

    // // clear_linecolors does not change color_mode
    // mesh.color_mode = ColorMode::POINTCOLORS;
    // mesh.clear_linecolors();

    session.add_mesh(std::make_shared<Mesh>(mesh));


    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "mesh_recheck.pb").string();
    session.pb_dump(filepath);

    return 0;
}
