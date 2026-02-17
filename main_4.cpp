#include "session.h"
#include "nurbssurface.h"
#include "primitives.h"
#include "mesh.h"
#include "boundingbox.h"
#include <iostream>

using namespace session_cpp;

int main() {
    Session session("annen");

    auto surfaces = Primitives::annen_surfaces();
    std::cout << "Loaded " << surfaces.size() << " Annen surfaces" << std::endl;

    for (size_t i = 0; i < surfaces.size(); i++) {
        surfaces[i].name = "annen_" + std::to_string(i);

        auto bb = BoundingBox::from_nurbssurface(surfaces[i]);
        auto lo = bb.min_point();
        auto hi = bb.max_point();

        Mesh m = Primitives::chevron_mesh(surfaces[i], 4, 900.0, 0.5, 0.05799);
        m.name = "chevron_" + std::to_string(i);

        std::cout << "  [" << i << "] cv:" << surfaces[i].cv_count(0) << "x" << surfaces[i].cv_count(1)
                  << " bb:[" << lo[0] << "," << lo[1] << "," << lo[2]
                  << "]-[" << hi[0] << "," << hi[1] << "," << hi[2]
                  << "] mesh: v=" << m.number_of_vertices() << " f=" << m.number_of_faces() << std::endl;

        session.add_nurbssurface(std::make_shared<NurbsSurface>(surfaces[i]));
        session.add_mesh(std::make_shared<Mesh>(m));
    }

    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/annen.pb");
    std::cout << "Annen: " << session.objects.nurbssurfaces->size() << " surfaces, "
              << session.objects.meshes->size() << " meshes" << std::endl;
    return 0;
}
