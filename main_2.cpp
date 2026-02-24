#include "session.h"
#include "primitives.h"
#include <iostream>
#include <filesystem>

using namespace session_cpp;

static void stats(const char* name, const Mesh& grid, const Mesh& adaptive) {
    std::cout << name << ": grid=" << grid.face.size() << " adaptive=" << adaptive.face.size() << "\n";
}

int main() {

    Session session;

    // Each row: mesh() (grid) | mesh_delaunay() (adaptive quadtree)

    // 1. Sphere — two poles, closed U, rational
    NurbsSurface sph1 = Primitives::sphere_surface(0, 0, 0, 3.0);
    Mesh m1a = sph1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m1a));

    NurbsSurface sph3 = Primitives::sphere_surface(10, 0, 0, 3.0);
    Mesh m1c = sph3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m1c));
    stats("Sphere", m1a, m1c);

    // 2. Cone — singular apex (pole), closed U
    double dy = 12;
    NurbsSurface cone1 = Primitives::cone_surface(0, dy, 0, 2.0, 6.0);
    Mesh m2a = cone1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m2a));

    NurbsSurface cone3 = Primitives::cone_surface(8, dy, 0, 2.0, 6.0);
    Mesh m2c = cone3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m2c));
    stats("Cone", m2a, m2c);

    // 3. Torus — doubly closed (U and V), rational
    dy = 24;
    NurbsSurface tor1 = Primitives::torus_surface(0, dy, 0, 4.0, 1.5);
    Mesh m3a = tor1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m3a));

    NurbsSurface tor3 = Primitives::torus_surface(14, dy, 0, 4.0, 1.5);
    Mesh m3c = tor3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m3c));
    stats("Torus", m3a, m3c);

    // 4. Loft — varying radius circles, closed U, multi-span V
    dy = 38;
    NurbsSurface loft1 = Primitives::create_loft({
        Primitives::circle(0, dy, 0, 2.0), Primitives::circle(0, dy, 2, 1.0),
        Primitives::circle(0, dy, 4, 1.5), Primitives::circle(0, dy, 6, 0.8)}, 3);
    Mesh m4a = loft1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m4a));

    NurbsSurface loft3 = Primitives::create_loft({
        Primitives::circle(8, dy, 0, 2.0), Primitives::circle(8, dy, 2, 1.0),
        Primitives::circle(8, dy, 4, 1.5), Primitives::circle(8, dy, 6, 0.8)}, 3);
    Mesh m4c = loft3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m4c));
    stats("Loft", m4a, m4c);

    // 5. Extrusion (circle) — closed U, linear V, rational
    dy = 52;
    Vector ext_dir(0, 0, 5);

    NurbsSurface ext1 = Primitives::create_extrusion(Primitives::circle(0, dy, 0, 3.0), ext_dir);
    Mesh m5a = ext1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m5a));

    NurbsSurface ext3 = Primitives::create_extrusion(Primitives::circle(10, dy, 0, 3.0), ext_dir);
    Mesh m5c = ext3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m5c));
    stats("Extrusion", m5a, m5c);

    // 6. Ruled — bilinear (degree 1×1), tests twist subdivision
    dy = 64;
    auto ra = NurbsCurve::create(false, 1, {Point(0, dy, 0), Point(5, dy, 5)});
    auto rb = NurbsCurve::create(false, 1, {Point(0, dy + 5, 5), Point(5, dy + 5, 0)});

    NurbsSurface rul1 = Primitives::create_ruled(ra, rb);
    Mesh m6a = rul1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m6a));

    NurbsSurface rul3 = Primitives::create_ruled(ra, rb);
    Mesh m6c = rul3.mesh_delaunay(45.0);
    m6c.xform = Xform::translation(8, 0, 0);
    m6c.transform();
    session.add_mesh(std::make_shared<Mesh>(m6c));
    stats("Ruled", m6a, m6c);

    // 7. Sweep1 — circle along curved rail
    dy = 76;
    NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);

    NurbsCurve rail1 = NurbsCurve::create(false, 2, {Point(0, dy, 0), Point(0, dy + 5, 0), Point(2, dy + 9, 0)});
    NurbsSurface sw1a = Primitives::create_sweep1(rail1, profile);
    Mesh m7a = sw1a.mesh();
    session.add_mesh(std::make_shared<Mesh>(m7a));

    NurbsCurve rail3 = NurbsCurve::create(false, 2, {Point(8, dy, 0), Point(8, dy + 5, 0), Point(10, dy + 9, 0)});
    NurbsSurface sw1c = Primitives::create_sweep1(rail3, profile);
    Mesh m7c = sw1c.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m7c));
    stats("Sweep1", m7a, m7c);

    // 8. Sweep2 — two rails + cross sections
    dy = 90;
    NurbsCurve r1 = NurbsCurve::create(false, 2, {Point(0, dy - 1, 0), Point(1, dy + 3, 0), Point(2, dy + 4, 0)});
    NurbsCurve r2 = NurbsCurve::create(false, 2, {Point(4, dy - 1, 0), Point(4, dy + 3, 0), Point(3, dy + 4, 0)});
    NurbsCurve sh1 = NurbsCurve::create(false, 2, {Point(0, dy - 1, 0), Point(2, dy - 1, 2), Point(4, dy - 1, 0)});
    NurbsCurve sh2 = NurbsCurve::create(false, 2, {Point(2, dy + 4, 0), Point(2.5, dy + 4, 1.5), Point(3, dy + 4, 0)});

    NurbsSurface sw2a = Primitives::create_sweep2(r1, r2, {sh1, sh2});
    Mesh m8a = sw2a.mesh();
    session.add_mesh(std::make_shared<Mesh>(m8a));

    NurbsCurve r5 = NurbsCurve::create(false, 2, {Point(8, dy - 1, 0), Point(9, dy + 3, 0), Point(10, dy + 4, 0)});
    NurbsCurve r6 = NurbsCurve::create(false, 2, {Point(12, dy - 1, 0), Point(12, dy + 3, 0), Point(11, dy + 4, 0)});
    NurbsCurve sh5 = NurbsCurve::create(false, 2, {Point(8, dy - 1, 0), Point(10, dy - 1, 2), Point(12, dy - 1, 0)});
    NurbsCurve sh6 = NurbsCurve::create(false, 2, {Point(10, dy + 4, 0), Point(10.5, dy + 4, 1.5), Point(11, dy + 4, 0)});
    NurbsSurface sw2c = Primitives::create_sweep2(r5, r6, {sh5, sh6});
    Mesh m8c = sw2c.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m8c));
    stats("Sweep2", m8a, m8c);

    // 9. Edge surface (Coons patch) — 4 boundary curves
    dy = 104;
    auto south = NurbsCurve::create(false, 3, {Point(1, dy, 0), Point(1, dy + 2, 3), Point(1, dy + 5, 3), Point(1, dy + 7, 0)});
    auto west  = NurbsCurve::create(false, 2, {Point(10, dy, 0), Point(5.5, dy, 3.5), Point(1, dy, 0)});
    auto north = NurbsCurve::create(false, 3, {Point(10, dy, 0), Point(10, dy + 2, 3), Point(10, dy + 5, 3), Point(10, dy + 7, 0)});
    auto east  = NurbsCurve::create(false, 2, {Point(10, dy + 7, 0), Point(5.5, dy + 7, 3.5), Point(1, dy + 7, 0)});

    NurbsSurface edge1 = Primitives::create_edge(south, west, north, east);
    Mesh m9a = edge1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m9a));

    auto s3 = NurbsCurve::create(false, 3, {Point(15, dy, 0), Point(15, dy + 2, 3), Point(15, dy + 5, 3), Point(15, dy + 7, 0)});
    auto w3 = NurbsCurve::create(false, 2, {Point(24, dy, 0), Point(19.5, dy, 3.5), Point(15, dy, 0)});
    auto n3 = NurbsCurve::create(false, 3, {Point(24, dy, 0), Point(24, dy + 2, 3), Point(24, dy + 5, 3), Point(24, dy + 7, 0)});
    auto e3 = NurbsCurve::create(false, 2, {Point(24, dy + 7, 0), Point(19.5, dy + 7, 3.5), Point(15, dy + 7, 0)});
    NurbsSurface edge3 = Primitives::create_edge(s3, w3, n3, e3);
    Mesh m9c = edge3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m9c));
    stats("Edge", m9a, m9c);

    // 10. Wave — multi-span freeform (13×13 CVs, 10 spans)
    dy = 118;
    NurbsSurface wav1 = Primitives::wave_surface(5.0, 1.5);
    Mesh m10a = wav1.mesh();
    m10a.xform = Xform::translation(0, dy, 0);
    m10a.transform();
    session.add_mesh(std::make_shared<Mesh>(m10a));

    NurbsSurface wav3 = Primitives::wave_surface(5.0, 1.5);
    Mesh m10c = wav3.mesh_delaunay(45.0);
    m10c.xform = Xform::translation(8, dy, 0);
    m10c.transform();
    session.add_mesh(std::make_shared<Mesh>(m10c));
    stats("Wave", m10a, m10c);

    // 11. Planar — mesh() early exit: 2 triangles
    dy = 132;
    auto bnd1 = NurbsCurve::create(false, 1, {
        Point(0, dy, 0), Point(6, dy, 0), Point(6, dy + 4, 0), Point(0, dy + 4, 0), Point(0, dy, 0)});
    NurbsSurface pln1 = Primitives::create_planar(bnd1);
    Mesh m11a = pln1.mesh();
    session.add_mesh(std::make_shared<Mesh>(m11a));

    auto bnd3 = NurbsCurve::create(false, 1, {
        Point(10, dy, 0), Point(16, dy, 0), Point(16, dy + 4, 0), Point(10, dy + 4, 0), Point(10, dy, 0)});
    NurbsSurface pln3 = Primitives::create_planar(bnd3);
    Mesh m11c = pln3.mesh_delaunay(45.0);
    session.add_mesh(std::make_shared<Mesh>(m11c));
    stats("Planar", m11a, m11c);

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "surface_meshing.pb").string();
    session.pb_dump(filepath);

    return 0;
}
