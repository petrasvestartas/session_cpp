#include <cassert>
#include <filesystem>
#include <set>
#include "mesh.h"
#include "session.h"

using namespace session_cpp;

int main() {
    std::vector<Point> pts = {{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}, {2,0,0}};
    Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0,1,2,3}, {1,4,2}});
    auto vkeys = mesh.vertices();
    size_t v0 = vkeys[0], v1 = vkeys[1], v2 = vkeys[2];
    auto fkeys = mesh.faces();
    size_t f0 = fkeys[0];

    // vertex_position
    std::optional<Point> pos = mesh.vertex_position(v0);
    assert(pos.has_value());
    assert(TOLERANCE.is_point_close(*pos, Point(0.0, 0.0, 0.0)));
    assert(!mesh.vertex_position(999).has_value());

    // face_vertices
    std::optional<std::vector<size_t>> fv = mesh.face_vertices(f0);
    assert(fv.has_value());
    assert(fv->size() == 4);
    assert((*fv)[0] == v0 && (*fv)[1] == v1 && (*fv)[2] == v2 && (*fv)[3] == vkeys[3]);

    // vertex_neighbors
    std::vector<size_t> nb = mesh.vertex_neighbors(v1);
    assert(nb.size() == 3);

    // vertex_faces
    std::vector<size_t> vf0 = mesh.vertex_faces(v0);
    assert(vf0.size() == 1);
    std::vector<size_t> vf1 = mesh.vertex_faces(v1);
    assert(vf1.size() == 2);

    // vertex_edges
    std::vector<std::pair<size_t, size_t>> ve = mesh.vertex_edges(v1);
    assert(ve.size() == 3);
    std::set<std::pair<size_t, size_t>> ve_set(ve.begin(), ve.end());
    assert(ve_set.contains({v1, v0}));
    assert(ve_set.contains({v1, v2}));
    assert(ve_set.contains({v1, vkeys[4]}));

    // face_edges
    std::vector<std::pair<size_t, size_t>> fe = mesh.face_edges(f0);
    assert(fe.size() == 4);
    assert(fe[0] == std::make_pair(v0, v1));
    assert(fe[1] == std::make_pair(v1, v2));
    assert(fe[2] == std::make_pair(v2, v3));
    assert(fe[3] == std::make_pair(v3, v0));

    // face_neighbors
    std::vector<size_t> fn0 = mesh.face_neighbors(f0);
    assert(fn0.size() == 1);
    assert(fn0[0] == fkeys[1]);

    // edge_vertices
    std::array<size_t, 2> ev = mesh.edge_vertices(v0, v1);
    assert(ev[0] == v0 && ev[1] == v1);

    // edge_faces
    std::pair<std::optional<size_t>, std::optional<size_t>> ef_inner = mesh.edge_faces(v1, v2);
    assert(ef_inner.first.has_value() && ef_inner.second.has_value());
    std::pair<std::optional<size_t>, std::optional<size_t>> ef_boundary = mesh.edge_faces(v0, v1);
    assert(ef_boundary.first.has_value() != ef_boundary.second.has_value());

    // edge_edges
    std::vector<std::pair<size_t, size_t>> ee = mesh.edge_edges(v1, v2);
    assert(ee.size() == 4);
    std::set<std::pair<size_t, size_t>> ee_set(ee.begin(), ee.end());
    assert(!ee_set.contains({v1, v2}));
    assert(!ee_set.contains({v2, v1}));

    // --- Rhino visualization ---
    Session session("connectivity_debug");
    session.add_mesh(std::make_shared<Mesh>(mesh));

    for (size_t vk : mesh.vertices()) {
        Point p = *mesh.vertex_position(vk);
        p.name = "v" + std::to_string(vk);
        p.pointcolor = Color::red();
        p.width = 3.0;
        session.add_point(std::make_shared<Point>(p));
    }

    for (size_t fk : mesh.faces()) {
        auto fverts = *mesh.face_vertices(fk);
        double cx = 0, cy = 0, cz = 0;
        for (size_t vk : fverts) {
            Point vp = *mesh.vertex_position(vk);
            cx += vp[0]; cy += vp[1]; cz += vp[2];
        }
        double n = (double)fverts.size();
        Point cp(cx/n, cy/n, cz/n);
        std::string label = "f" + std::to_string(fk) + ":";
        for (size_t vk : fverts) label += " " + std::to_string(vk);
        cp.name = label;
        cp.pointcolor = Color::blue();
        cp.width = 3.0;
        session.add_point(std::make_shared<Point>(cp));
    }

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "connectivity_debug.pb").string();
    session.pb_dump(fp);
    return 0;
}
