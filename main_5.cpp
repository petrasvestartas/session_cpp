#include "brep.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "session.h"
#include "primitives.h"
#include <filesystem>
#include <iostream>

using namespace session_cpp;

// Bicubic "pillow" solid: top dome (z>0) + bottom dome (z<0), 4 shared cubic arc edges.
// Equatorial edges bow outward in XY (visible NURBS curves in Rhino).
BRep make_pillow(const std::string& nm, double r, double h) {
    BRep b;
    b.name = nm;

    double m = r * 4.0 / 3.0;   // outward bow (quarter-circle tangent approximation)
    double ri = r / 3.0;         // inner CV XY offset

    Point c0(-r,-r,0), c1(r,-r,0), c2(r,r,0), c3(-r,r,0);

    // Top dome: boundary z=0, interior z=+h; bottom: same boundary, interior z=-h
    Point cvT[4][4] = {
        {c0,               Point(-m,-ri,0),   Point(-m,ri,0),   c3             },
        {Point(-ri,-m,0),  Point(-ri,-ri, h), Point(-ri,ri, h), Point(-ri,m,0) },
        {Point( ri,-m,0),  Point( ri,-ri, h), Point( ri,ri, h), Point( ri,m,0) },
        {c1,               Point( m,-ri,0),   Point( m,ri,0),   c2             }
    };
    Point cvB[4][4] = {
        {c0,               Point(-m,-ri,0),   Point(-m,ri,0),   c3              },
        {Point(-ri,-m,0),  Point(-ri,-ri,-h), Point(-ri,ri,-h), Point(-ri,m,0)  },
        {Point( ri,-m,0),  Point( ri,-ri,-h), Point( ri,ri,-h), Point( ri,m,0)  },
        {c1,               Point( m,-ri,0),   Point( m,ri,0),   c2              }
    };

    auto make_srf = [&](const Point cv[4][4]) {
        NurbsSurface s;
        s.create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                s.set_cv(i, j, cv[i][j]);
        return b.add_surface(s);
    };
    int sT = make_srf(cvT), sB = make_srf(cvB);

    int v0 = b.add_vertex(c0), v1 = b.add_vertex(c1);
    int v2 = b.add_vertex(c2), v3 = b.add_vertex(c3);

    auto c3d = [&](std::initializer_list<Point> pts) {
        return b.add_curve_3d(NurbsCurve::create(false, 3, pts));
    };
    // 4 shared cubic arc edges (j=0 row, i=3 col, j=3 row, i=0 col of top dome)
    int ce0 = c3d({cvT[0][0],cvT[1][0],cvT[2][0],cvT[3][0]}); // v0→v1
    int ce1 = c3d({cvT[3][0],cvT[3][1],cvT[3][2],cvT[3][3]}); // v1→v2
    int ce2 = c3d({cvT[0][3],cvT[1][3],cvT[2][3],cvT[3][3]}); // v3→v2
    int ce3 = c3d({cvT[0][0],cvT[0][1],cvT[0][2],cvT[0][3]}); // v0→v3

    int te[4] = { b.add_edge(ce0,v0,v1), b.add_edge(ce1,v1,v2),
                  b.add_edge(ce2,v3,v2), b.add_edge(ce3,v0,v3) };

    Point uv[4] = {Point(0,0,0),Point(1,0,0),Point(1,1,0),Point(0,1,0)};

    // Top face: CCW from above (e0 fwd, e1 fwd, e2 rev, e3 rev)
    int fT = b.add_face(sT, false), lT = b.add_loop(fT, BRepLoopType::Outer);
    bool revT[4] = {false, false, true, true};
    for (int ei = 0; ei < 4; ++ei) {
        int c2d = b.add_curve_2d(NurbsCurve::create(false, 1, {uv[ei], uv[(ei+1)%4]}));
        b.add_trim(c2d, te[ei], lT, revT[ei], BRepTrimType::Boundary);
    }

    // Bottom face: reversed normals, opposite edge traversal for watertight solid
    int fB = b.add_face(sB, true), lB = b.add_loop(fB, BRepLoopType::Outer);
    bool revB[4] = {true, true, false, false};
    for (int ei = 0; ei < 4; ++ei) {
        int c2d = b.add_curve_2d(NurbsCurve::create(false, 1, {uv[ei], uv[(ei+1)%4]}));
        b.add_trim(c2d, te[ei], lB, revB[ei], BRepTrimType::Boundary);
    }

    return b;
}

// Build a single bicubic-patch BRep from a 4×4 control point grid.
// Boundary edges are degree-3 NURBS curves (surface isoparametric curves).
// UV trim domain is [0,1]×[0,1] with 4 linear trim edges.
BRep make_bicubic_patch(const std::string& nm, const Point cv[4][4]) {
    BRep b;
    b.name = nm;

    NurbsSurface srf;
    srf.create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            srf.set_cv(i, j, cv[i][j]);
    int si = b.add_surface(srf);

    // Corners: (u=0,v=0)=cv[0][0], (u=1,v=0)=cv[3][0], (u=1,v=1)=cv[3][3], (u=0,v=1)=cv[0][3]
    int vi[4] = { b.add_vertex(cv[0][0]), b.add_vertex(cv[3][0]),
                  b.add_vertex(cv[3][3]), b.add_vertex(cv[0][3]) };

    // Boundary isoparametric 3D curves (degree-3 NURBS)
    // e0: v=0 row,  v0→v1    e1: u=1 col, v1→v2
    // e2: v=1 row,  v3→v2    e3: u=0 col, v0→v3
    auto c3d = [&](std::initializer_list<Point> pts) {
        return b.add_curve_3d(NurbsCurve::create(false, 3, pts));
    };
    int ce[4] = {
        c3d({cv[0][0], cv[1][0], cv[2][0], cv[3][0]}),
        c3d({cv[3][0], cv[3][1], cv[3][2], cv[3][3]}),
        c3d({cv[0][3], cv[1][3], cv[2][3], cv[3][3]}),
        c3d({cv[0][0], cv[0][1], cv[0][2], cv[0][3]})
    };
    int te[4] = {
        b.add_edge(ce[0], vi[0], vi[1]),
        b.add_edge(ce[1], vi[1], vi[2]),
        b.add_edge(ce[2], vi[3], vi[2]),  // v=1: v3→v2
        b.add_edge(ce[3], vi[0], vi[3])   // u=0: v0→v3
    };

    int fi = b.add_face(si, false);
    int li = b.add_loop(fi, BRepLoopType::Outer);
    Point uv[4] = { Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0) };
    // CCW loop: v0→v1→v2→v3; e2 and e3 are used reversed
    bool rev[4] = { false, false, true, true };
    for (int k = 0; k < 4; ++k) {
        int c2d = b.add_curve_2d(NurbsCurve::create(false, 1, {uv[k], uv[(k+1)%4]}));
        b.add_trim(c2d, te[k], li, rev[k], BRepTrimType::Boundary);
    }
    return b;
}

// Add BRep + translated mesh side by side
void add_with_mesh(Session& session, BRep& brep, double mesh_offset_x) {
    Mesh m = brep.mesh();
    m.xform = Xform::translation(mesh_offset_x, 0, 0);
    m.transform();
    m.name = brep.name + "_mesh";
    std::cout << brep << " => mesh: " << m.number_of_faces() << "f "
              << m.number_of_vertices() << "v\n";
    session.add_brep(std::make_shared<BRep>(brep));
    session.add_mesh(std::make_shared<Mesh>(m));
}

int main() {
    Session session("brep_demo");
    std::string base = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data").string();
    double dx = 0.0;  // mesh offset to the right of each brep

    // Example 1: Box BRep from factory
    BRep box = BRep::create_box(4.0, 6.0, 3.0);
    add_with_mesh(session, box, dx);

    // Example 2: L-shaped extrusion via from_polylines
    double w = 4.0, d = 6.0, h = 3.0, notch = 2.0;
    std::vector<Point> bottom = {
        Point(0,0,0), Point(w,0,0), Point(w,notch,0),
        Point(notch,notch,0), Point(notch,d,0), Point(0,d,0)
    };
    std::vector<Point> top;
    for (auto& p : bottom) top.push_back(Point(p[0], p[1], h));
    int nb = (int)bottom.size();

    std::vector<Polyline> faces;
    std::vector<Point> bot_rev(bottom.rbegin(), bottom.rend());
    bot_rev.push_back(bot_rev[0]);
    faces.push_back(Polyline(bot_rev));
    std::vector<Point> top_closed = top;
    top_closed.push_back(top[0]);
    faces.push_back(Polyline(top_closed));
    for (int i = 0; i < nb; ++i) {
        int j = (i + 1) % nb;
        faces.push_back(Polyline({bottom[i], bottom[j], top[j], top[i], bottom[i]}));
    }
    BRep lshape = BRep::from_polylines(faces);
    lshape.name = "lshape";
    lshape.xform = Xform::translation(15, 0, 0);
    lshape.transform();
    add_with_mesh(session, lshape, dx);

    // Example 3: Cylinder
    BRep cyl = BRep::create_cylinder(1.5, 4.0);
    cyl.xform = Xform::translation(30, 0, 0);
    cyl.transform();
    add_with_mesh(session, cyl, dx);

    // Example 4: Sphere
    BRep sph = BRep::create_sphere(2.0);
    sph.xform = Xform::translation(45, 0, 0);
    sph.transform();
    add_with_mesh(session, sph, dx);

    // Example 5: Flat plate with circular hole
    NurbsCurve outer_rect = NurbsCurve::create(false, 1,
        {Point(-5,-5,0), Point(5,-5,0), Point(5,5,0), Point(-5,5,0), Point(-5,-5,0)});
    NurbsCurve hole1 = Primitives::circle(0, 0, 0, 2.0);
    BRep plate_hole = BRep::from_nurbscurves({outer_rect}, {{hole1}});
    plate_hole.name = "plate_hole";
    plate_hole.xform = Xform::translation(0, 15, 0);
    plate_hole.transform();
    add_with_mesh(session, plate_hole, dx);

    // Example 6: Plate with two circular holes
    NurbsCurve outer2 = NurbsCurve::create(false, 1,
        {Point(-8,-4,0), Point(8,-4,0), Point(8,4,0), Point(-8,4,0), Point(-8,-4,0)});
    NurbsCurve hole_left = Primitives::circle(-4, 0, 0, 1.5);
    NurbsCurve hole_right = Primitives::circle(4, 0, 0, 1.5);
    BRep plate_2holes = BRep::from_nurbscurves({outer2}, {{hole_left, hole_right}});
    plate_2holes.name = "plate_2holes";
    plate_2holes.xform = Xform::translation(20, 15, 0);
    plate_2holes.transform();
    add_with_mesh(session, plate_2holes, dx);

    // Example 7: Circular disc
    NurbsCurve disc_outer = Primitives::circle(0, 0, 0, 3.0);
    BRep disc = BRep::from_nurbscurves({disc_outer});
    disc.name = "disc";
    disc.xform = Xform::translation(40, 15, 0);
    disc.transform();
    add_with_mesh(session, disc, dx);

    // Example 8: Block with cylindrical through-hole
    BRep block_hole = BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5);
    block_hole.xform = Xform::translation(0, 30, 0);
    block_hole.transform();
    add_with_mesh(session, block_hole, dx);

    // Example 9: Re-meshed cylinder (verify direct-mesh routing improvement)
    BRep cyl2 = BRep::create_cylinder(2.0, 5.0);
    cyl2.xform = Xform::translation(20, 30, 0);
    cyl2.transform();
    add_with_mesh(session, cyl2, dx);

    // Example 10: Re-meshed sphere (verify direct-mesh routing improvement)
    BRep sph2 = BRep::create_sphere(3.0);
    sph2.xform = Xform::translation(40, 30, 0);
    sph2.transform();
    add_with_mesh(session, sph2, dx);

    // Example 11: Bicubic dome — 1 double-curved face, 4 cubic arc boundaries
    // z[i][j]: 0 at corners, 1 at edge midpoints, 2 at interior (dome peak)
    {
        double xi[4] = {-3,-1,1,3}, yj[4] = {-3,-1,1,3};
        int zh[4][4] = {{0,1,1,0},{1,2,2,1},{1,2,2,1},{0,1,1,0}};
        Point cv[4][4];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                cv[i][j] = Point(xi[i], yj[j], zh[i][j]);
        BRep dome = make_bicubic_patch("dome", cv);
        dome.xform = Xform::translation(0, 45, 0);
        dome.transform();
        add_with_mesh(session, dome, dx);
    }

    // Example 12: Two joined bicubic domes — 2 faces sharing a cubic arc edge
    {
        double xi[4] = {-3,-1,1,3};
        double ya[4] = {-3,-1,1,3}, yb[4] = {3,5,7,9};
        int zh[4][4] = {{0,1,1,0},{1,2,2,1},{1,2,2,1},{0,1,1,0}};
        Point cvA[4][4], cvB[4][4];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                cvA[i][j] = Point(xi[i], ya[j], zh[i][j]);
                cvB[i][j] = Point(xi[i], yb[j], zh[i][j]);
            }

        BRep b;
        b.name = "two_domes";

        // 2 surfaces
        auto make_srf = [&](const Point cv[4][4]) {
            NurbsSurface s;
            s.create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    s.set_cv(i, j, cv[i][j]);
            return b.add_surface(s);
        };
        int sA = make_srf(cvA), sB = make_srf(cvB);

        // 6 vertices: 4 from Patch A + 2 new corners of Patch B top edge
        int v0 = b.add_vertex(cvA[0][0]), v1 = b.add_vertex(cvA[3][0]);
        int v2 = b.add_vertex(cvA[3][3]), v3 = b.add_vertex(cvA[0][3]);  // shared
        int v4 = b.add_vertex(cvB[0][3]), v5 = b.add_vertex(cvB[3][3]);

        // 7 3D curves
        auto c3d = [&](std::initializer_list<Point> pts) {
            return b.add_curve_3d(NurbsCurve::create(false, 3, pts));
        };
        // Patch A edges
        int c0 = c3d({cvA[0][0],cvA[1][0],cvA[2][0],cvA[3][0]}); // v0→v1
        int c1 = c3d({cvA[3][0],cvA[3][1],cvA[3][2],cvA[3][3]}); // v1→v2
        int c2 = c3d({cvA[0][3],cvA[1][3],cvA[2][3],cvA[3][3]}); // v3→v2 (SHARED)
        int c3 = c3d({cvA[0][0],cvA[0][1],cvA[0][2],cvA[0][3]}); // v0→v3
        // Patch B unique edges
        int c4 = c3d({cvB[3][0],cvB[3][1],cvB[3][2],cvB[3][3]}); // v2→v5
        int c5 = c3d({cvB[0][3],cvB[1][3],cvB[2][3],cvB[3][3]}); // v4→v5
        int c6 = c3d({cvB[0][0],cvB[0][1],cvB[0][2],cvB[0][3]}); // v3→v4

        // 7 topology edges
        int te0 = b.add_edge(c0,v0,v1), te1 = b.add_edge(c1,v1,v2);
        int te2 = b.add_edge(c2,v3,v2);  // shared: A uses rev=true, B uses rev=false
        int te3 = b.add_edge(c3,v0,v3);
        int te4 = b.add_edge(c4,v2,v5), te5 = b.add_edge(c5,v4,v5);
        int te6 = b.add_edge(c6,v3,v4);

        // Patch A face & loop
        int fA = b.add_face(sA, false), lA = b.add_loop(fA, BRepLoopType::Outer);
        Point uv[4] = {Point(0,0,0),Point(1,0,0),Point(1,1,0),Point(0,1,0)};
        auto trim = [&](int loop, int edge, bool rev) {
            // just placeholder — we build per-loop below
            (void)loop; (void)edge; (void)rev;
        };
        (void)trim;
        {
            auto add = [&](Point a, Point b2, int edge, bool r) {
                int c2d = b.add_curve_2d(NurbsCurve::create(false, 1, {a, b2}));
                b.add_trim(c2d, edge, lA, r, BRepTrimType::Boundary);
            };
            add(uv[0],uv[1],te0,false); add(uv[1],uv[2],te1,false);
            add(uv[2],uv[3],te2,true);  add(uv[3],uv[0],te3,true);
        }

        // Patch B face & loop
        int fB = b.add_face(sB, false), lB = b.add_loop(fB, BRepLoopType::Outer);
        {
            auto add = [&](Point a, Point b2, int edge, bool r) {
                int c2d = b.add_curve_2d(NurbsCurve::create(false, 1, {a, b2}));
                b.add_trim(c2d, edge, lB, r, BRepTrimType::Boundary);
            };
            add(uv[0],uv[1],te2,false); add(uv[1],uv[2],te4,false);  // shared edge fwd
            add(uv[2],uv[3],te5,true);  add(uv[3],uv[0],te6,true);
        }

        b.xform = Xform::translation(15, 45, 0);
        b.transform();
        add_with_mesh(session, b, dx);
    }

    // Example 13: Bicubic saddle — 1 double-curved face, anticlastic curvature
    // z[i][j] = (i-1.5)*(j-1.5)*0.8 — a hyperbolic paraboloid patch
    {
        double xi[4] = {-3,-1,1,3}, yj[4] = {-3,-1,1,3};
        Point cv[4][4];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                cv[i][j] = Point(xi[i], yj[j], (i-1.5)*(j-1.5)*0.8);
        BRep saddle = make_bicubic_patch("saddle", cv);
        saddle.xform = Xform::translation(35, 45, 0);
        saddle.transform();
        add_with_mesh(session, saddle, dx);
    }

    // Example 14: Bicubic pillow — closed solid, 2 double-curved faces, 4 cubic arc edges
    BRep pillow = make_pillow("pillow", 3.0, 2.0);
    pillow.xform = Xform::translation(0, 60, 0);
    pillow.transform();
    add_with_mesh(session, pillow, dx);

    // Save
    session.json_dump(base + "/brep_demo.json");
    session.pb_dump(base + "/brep_demo.pb");
    std::cout << "Saved to session_data/brep_demo.{json,pb}\n";
    return 0;
}
