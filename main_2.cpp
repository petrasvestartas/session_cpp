#include "color.h"
#include "mesh.h"
#include "session.h"
#include <filesystem>

using namespace session_cpp;

int main() {

    Session session;

    Mesh mesh = Mesh::from_polylines({
        {
            {1.28955, 0, 1.127558},
            {0.85791, 0, 0.225512},
            {0.64209, -0.866025, -0.225512},
            {0.85791, -1.732051, 0.225512},
            {1.458565, -1.732051, 1.127558},
            {1.50537, -0.866025, 1.578581},
        },
        {
            {0.64209, 0.866025, -0.225512},
            {0.114274, 0.866025, -0.686294},
            {-0.00537, 0, -1.578581},
            {0.21045, -0.866025, -1.127558},
            {0.64209, -0.866025, -0.225512},
            {0.85791, 0, 0.225512},
        },
        {
            {1.28955, 1.732051, 1.127558},
            {0.85791, 1.732051, 0.225512},
            {0.64209, 0.866025, -0.225512},
            {0.85791, 0, 0.225512},
            {1.28955, -0, 1.127558},
            {1.853404, 0.866025, 1.578581},
        },
    }, 0.001);



    mesh.name = "hexagon";

    std::vector<Color> pal = {
        Color::red(), Color::orange(), Color::yellow(),
        Color::green(), Color::blue(), Color::violet()
    };

    mesh.set_objectcolor(Color::grey());

    // point colors
    std::vector<Color> pc;
    pc.reserve(mesh.number_of_vertices());
    for (size_t i = 0; i < mesh.number_of_vertices(); ++i)
        pc.emplace_back(pal[i % pal.size()]);
    mesh.set_pointcolors(std::move(pc));

    // face colors
    std::vector<Color> fc;
    fc.reserve(mesh.number_of_faces());
    for (size_t i = 0; i < mesh.number_of_faces(); ++i)
        fc.emplace_back(pal[i % pal.size()]);
    mesh.set_facecolors(std::move(fc));

    // edge colors
    std::vector<Color> lc;
    lc.reserve(mesh.number_of_edges());
    std::vector<double> lw(mesh.number_of_edges(), 0.1);
    for (size_t i = 0; i < mesh.number_of_edges(); ++i)
        lc.emplace_back(pal[i % pal.size()]);
    mesh.set_linecolors(std::move(lc), std::move(lw));

    // set it again to change from face colors to vertex colors
    for (size_t i = 0; i < mesh.number_of_vertices(); ++i) 
        pc.emplace_back(pal[i % pal.size()]);
    mesh.set_pointcolors(std::move(pc));
        


    // override: set color_mode directly to visualize any previously set array
    //
    
    mesh.color_mode = ColorMode::OBJECTCOLOR;
    mesh.color_mode = ColorMode::POINTCOLORS;
    mesh.color_mode = ColorMode::FACECOLORS;
    // mesh.color_mode = ColorMode::NONE;


    const std::vector<Color>& pc_ref = mesh.get_pointcolors();
    const std::vector<Color>& fc_ref = mesh.get_facecolors();
    const std::vector<Color>& lc_ref = mesh.get_linecolors();
    std::cout << pc_ref.size() << " " << fc_ref.size() << " " << lc_ref.size() << "\n";

    mesh.clear_facecolors();   // clears array; reverts color_mode to ObjectColor ONLY IF currently FaceColors
    mesh.clear_linecolors();   // clears array; does NOT change color_mode
    mesh.clear_pointcolors();  // clears array; reverts color_mode to ObjectColor ONLY IF currently PointColors

    session.add_mesh(std::make_shared<Mesh>(mesh));

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "mesh_recheck.pb").string();
    session.pb_dump(filepath);

    return 0;
}
