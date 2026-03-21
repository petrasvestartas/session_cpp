#include <filesystem>
#include <iomanip>
#include <string>
#include "color.h"
#include "mesh.h"
#include "polyline.h"
#include "session.h"
using namespace session_cpp;

int main() {
    Session session("Mesh Geometry");

    Mesh mesh = Mesh::create_dodecahedron(1.5);
    mesh.name = "box_2x2x2";

    std::cout << std::setprecision(15) << mesh.area() << std::endl;
    std::cout << std::setprecision(15) << mesh.volume() << std::endl;
    std::cout << mesh.centroid() << std::endl;
    session.add(session.add_mesh(std::make_shared<Mesh>(mesh)));

    // ─────────────────────────────────────────────────────────────────────
    // Layer 8: Dihedral Angle
    // ─────────────────────────────────────────────────────────────────────
    std::shared_ptr<TreeNode> grp_dihedral = session.add_group("Dihedral Angle");
    {
        auto [angles, arcs, points] = mesh.dihedral_angles(0.3);
        for (const auto& [edge, angle] : angles)
            std::cout << "Edge (" << edge.first << ", " << edge.second << ") -> " << angle << " deg" << std::endl;
        for (const Polyline& arc : arcs)
            session.add(session.add_polyline(std::make_shared<Polyline>(arc)), grp_dihedral);
        for (const Point& pt : points)
            session.add(session.add_point(std::make_shared<Point>(pt)), grp_dihedral);
    }

    for (size_t fk : mesh.faces()) {
        auto area     = mesh.face_area(fk);
        auto centroid = mesh.face_centroid(fk);
        auto normal   = mesh.face_normal(fk);
        if (!area || !centroid || !normal) continue;
        std::cout << "Face " << fk << ": Area=" << *area
                  << ", Centroid=" << *centroid
                  << ", Normal=" << *normal << std::endl;
    }

    for (size_t vk : mesh.vertices()) {
    }

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "Mesh_Geometry.pb").string();
    session.pb_dump(fp);
    return 0;
}
