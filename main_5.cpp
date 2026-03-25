#include <filesystem>
#include <numbers>
#include <cmath>
#include "mesh.h"
#include "remesh_cdt.h"
#include "session.h"

using namespace session_cpp;

static Mesh mesh_from_cdt(
    const std::vector<std::pair<double,double>>& border,
    const std::vector<std::vector<std::pair<double,double>>>& holes,
    double ox = 0.0, double oy = 0.0)
{
    std::vector<std::pair<double,double>> flat;
    for (auto& p : border) flat.push_back(p);
    for (auto& h : holes) for (auto& p : h) flat.push_back(p);

    auto tris = cdt_triangulate(border, holes);

    Mesh m;
    for (auto& p : flat)
        m.add_vertex(Point(p.first + ox, p.second + oy, 0.0));
    for (auto& t : tris)
        m.add_face({(size_t)t[0], (size_t)t[1], (size_t)t[2]});
    return m;
}

int main() {
    Session session("RemeshCDT");

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "RemeshCDT.pb").string();

    // Triangle — simplest valid input: 3 vertices, 1 output face
    {
        Mesh m = mesh_from_cdt(
            {{0,0},{1,0},{0,1}},
            {});
        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Square — minimal quad: 4 vertices, 2 output faces
    {
        Mesh m = mesh_from_cdt(
            {{0,0},{2,0},{2,2},{0,2}},
            {}, 3.0, 0.0);
        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Convex polygon — regular hexagon: 6 vertices, 4 output faces
    {
        std::vector<std::pair<double,double>> hex;
        for (int i = 0; i < 6; ++i) {
            double a = i * std::numbers::pi / 3.0;
            hex.push_back({std::cos(a), std::sin(a)});
        }
        Mesh m = mesh_from_cdt(hex, {}, 7.0, 0.0);
        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Non-convex polygon — L-shape: reflex vertex forces interior diagonal
    {
        Mesh m = mesh_from_cdt(
            {{0,0},{3,0},{3,1},{1,1},{1,3},{0,3}},
            {}, 11.0, 0.0);
        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Polygon with rectangular hole — area = outer - inner
    {
        Mesh m = mesh_from_cdt(
            {{0,0},{4,0},{4,4},{0,4}},
            {{{1,1},{1,3},{3,3},{3,1}}},
            16.0, 0.0);
        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    // Polygon with two holes — frame with two rectangular cutouts
    {
        Mesh m = mesh_from_cdt(
            {{0,0},{6,0},{6,3},{0,3}},
            {{{0.5,0.5},{2.5,0.5},{2.5,2.5},{0.5,2.5}},
             {{3.5,0.5},{5.5,0.5},{5.5,2.5},{3.5,2.5}}},
            22.0, 0.0);
        session.add_mesh(std::make_shared<Mesh>(std::move(m)));
    }

    session.pb_dump(fp);
    return 0;
}
