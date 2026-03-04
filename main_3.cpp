#include "session.h"
#include "mesh.h"
#include "primitives.h"
#include "color.h"
#include "xform.h"
#include <filesystem>

using namespace session_cpp;

int main() {
    Session session("mesh_color");

    Mesh base = Primitives::dodecahedron(1000.0);
    std::vector<Color> pal = {
        Color::red(), Color::orange(), Color::yellow(),
        Color::green(), Color::blue(), Color::violet()
    };

    // a) Edge pipes — linecolors trigger pipes in rhino_mesh.py
    Mesh mesh_e = base;
    std::vector<Color> ec; ec.reserve(mesh_e.get_linecolors().size());
    std::vector<double> ew; ew.reserve(mesh_e.get_linecolors().size());
    for (size_t i = 0; i < mesh_e.get_linecolors().size(); ++i) { ec.push_back(pal[i % 6]); ew.push_back(5.0 + i); }
    mesh_e.set_linecolors(std::move(ec), std::move(ew));
    session.add_mesh(std::make_shared<Mesh>(mesh_e));

    // b) Vertex colors — rhino_mesh.py applies as vertex color array
    Mesh mesh_v = base;
    mesh_v.transform(Xform::translation(6000, 0, 0));
    std::vector<Color> pc; pc.reserve(mesh_v.get_pointcolors().size());
    for (size_t i = 0; i < mesh_v.get_pointcolors().size(); ++i) pc.push_back(pal[i % 6]);
    mesh_v.set_pointcolors(std::move(pc));
    session.add_mesh(std::make_shared<Mesh>(mesh_v));

    // c) Face colors — rhino_mesh.py unwelds + assigns per-face vertex colors
    Mesh mesh_f = base;
    mesh_f.transform(Xform::translation(12000, 0, 0));
    std::vector<Color> fc; fc.reserve(mesh_f.get_facecolors().size());
    for (size_t i = 0; i < mesh_f.get_facecolors().size(); ++i) fc.push_back(pal[i % 6]);
    mesh_f.set_facecolors(std::move(fc));
    session.add_mesh(std::make_shared<Mesh>(mesh_f));

    // d) Face colors + edge pipes simultaneously
    Mesh mesh_ef = base;
    mesh_ef.transform(Xform::translation(18000, 0, 0));
    std::vector<Color> efc; efc.reserve(mesh_ef.get_facecolors().size());
    for (size_t i = 0; i < mesh_ef.get_facecolors().size(); ++i) efc.push_back(pal[i % 6]);
    mesh_ef.set_facecolors(std::move(efc));
    std::vector<Color> elc; elc.reserve(mesh_ef.get_linecolors().size());
    std::vector<double> elw; elw.reserve(mesh_ef.get_linecolors().size());
    for (size_t i = 0; i < mesh_ef.get_linecolors().size(); ++i) { elc.push_back(pal[i % 6]); elw.push_back(50.0); }
    mesh_ef.set_linecolors(std::move(elc), std::move(elw));
    session.add_mesh(std::make_shared<Mesh>(mesh_ef));

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path()
                            / "session_data" / "mesh.pb").string();
    session.pb_dump(filepath);
    return 0;
}
