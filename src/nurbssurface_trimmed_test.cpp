#include "mini_test.h"
#include "nurbssurface_trimmed.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "primitives.h"
#include "mesh.h"
#include "color.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

// A collapsed planar corner takes its own triangle fan, never normal_at's +Z sentinel.
MINI_TEST("NurbsSurfaceTrimmed", "Singular Planar Normal") {
    NurbsSurfaceTrimmed trimmed;
    trimmed.m_surface = NurbsSurface::create(false,false,1,1,2,2,{
        Point(-1,0,0),Point(0,0,1),Point(1,0,0),Point(0,0,1)});
    TrimLoops loops;
    loops.uv = {{Point(0,0,0),Point(1,0,0),Point(1,1,0),Point(0,1,0)}};
    Mesh mesh = trimmed.mesh_loops(loops,5.0,0.001);
    MINI_CHECK(!mesh.face.empty());
    bool apex = false;
    for (const auto& [key, vertex] : mesh.vertex) {
        auto normal = vertex.normal().value();
        MINI_CHECK(std::abs(normal[0]) < 1e-12 && std::abs(normal[2]) < 1e-12);
        MINI_CHECK(std::abs(std::abs(normal[1])-1.0) < 1e-12);
        apex = apex || vertex.z == 1.0;
    }
    MINI_CHECK(apex);
}

MINI_TEST("NurbsSurfaceTrimmed", "Crease Loops") {
    NurbsSurfaceTrimmed ts;
    ts.m_surface = NurbsSurface::create(false, false, 1, 1, 3, 2, {
        Point(0,0,0), Point(0,1,0), Point(1,0,0), Point(1,1,0), Point(2,0,1), Point(2,1,1)});
    TrimLoops loops;
    loops.uv = {{Point(.1,.1,0),Point(1.9,.1,0),Point(1.9,.9,0),Point(.1,.9,0)},
                {Point(.8,.4,0),Point(1.2,.4,0),Point(1.2,.6,0),Point(.8,.6,0)}};
    Mesh mesh = ts.mesh_loops(loops,20.0,.005);
    MINI_CHECK(mesh.vertex.size()==16 && mesh.face.size()==12);
    int flat=0, tilted=0;
    for (const auto& [key,vd] : mesh.vertex) {
        if (vd.attributes.at("u") == 1.0) {
            bool interval=false;
            for (const auto& [name,value] : vd.attributes) if (name.rfind("boundary_interval/",0)==0) interval=true;
            MINI_CHECK(interval && vd.z == 0.0);
            auto normal=vd.normal().value();
            if (std::abs(normal[0])<1e-12) ++flat;
            if (std::abs(normal[0]+std::sqrt(.5))<1e-12) ++tilted;
        }
    }
    MINI_CHECK(flat==4 && tilted==4);
    for (const auto& [key,face] : mesh.face) {
        double low=INFINITY, high=-INFINITY,u=0,v=0;
        for (size_t key : face) {double x=mesh.vertex[key].attributes.at("u");low=std::min(low,x);high=std::max(high,x);u+=x;v+=mesh.vertex[key].attributes.at("v");}
        MINI_CHECK(!(low<1.0 && high>1.0));
        u/=3;v/=3;
        MINI_CHECK(!(u>.8 && u<1.2 && v>.4 && v<.6));
    }
}

MINI_TEST("NurbsSurfaceTrimmed", "Mesh Loops") {
    NurbsSurface planar = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0.0, 0.0, 0.0), Point(0.0, 4.0, 0.0),
        Point(4.0, 0.0, 0.0), Point(4.0, 4.0, 0.0),
    });
    for (const auto& surface : {planar, Primitives::wave_surface(1.0, 0.5)}) {
        NurbsSurfaceTrimmed ts;
        ts.m_surface = surface;
        TrimLoops loops;
        for (const auto& range : {std::pair<double, double>{0.0, 1.0}, {0.25, 0.75}}) {
            auto [low, high] = range;
            std::vector<Point> uv, xyz;
            std::vector<std::pair<double, double>> corners = {{low, low}, {high, low}, {high, high}, {low, high}};
            for (size_t side = 0; side < 4; ++side) {
                auto a = corners[side], b = corners[(side + 1) % 4];
                for (int sample = 0; sample < 8; ++sample) {
                    double t = sample / 8.0;
                    uv.push_back(Point(a.first + t*(b.first-a.first), a.second + t*(b.second-a.second), 0.0));
                }
            }
            for (const auto& p : uv) xyz.push_back(ts.m_surface.point_at(p[0], p[1]));
            loops.uv.push_back(uv); loops.xyz.push_back(xyz);
        }
        Mesh mesh = ts.mesh_loops(loops, 20.0, 0.005);
        MINI_CHECK(!mesh.face.empty());
        for (size_t li = 0; li < loops.xyz.size(); ++li) {
            const auto& points = loops.xyz[li];
            for (size_t sample = 0; sample < points.size(); ++sample) {
                const auto& p = points[sample];
                std::string key = "boundary/" + std::to_string(li) + "/" + std::to_string(sample);
                bool found = false;
                for (const auto& [vk, vd] : mesh.vertex) {
                    if (vd.attributes.count(key)) {
                        MINI_CHECK(vd.x == p[0] && vd.y == p[1] && vd.z == p[2]);
                        found = true;
                        break;
                    }
                }
                MINI_CHECK(found);
            }
        }
        for (const auto& [fk, vertices] : mesh.face) {
            double u = 0.0, v = 0.0;
            for (size_t key : vertices) { u += mesh.vertex[key].attributes["u"]; v += mesh.vertex[key].attributes["v"]; }
            u /= vertices.size(); v /= vertices.size();
            MINI_CHECK(!(u > 0.25 && u < 0.75 && v > 0.25 && v < 0.75));
        }
        loops.xyz[0].pop_back();
        MINI_CHECK(ts.mesh_loops(loops, 20.0, 0.005).face.empty());
    }
}


    MINI_TEST("NurbsSurfaceTrimmed", "Constructor") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        // Create a bilinear surface
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, Point(6.0, 0.0, 0.0));
        srf.set_cv(0, 1, Point(0.0, 6.0, 0.0));
        srf.set_cv(1, 1, Point(6.0, 6.0, 0.0));

        // Outer trim loop (rectangle in UV space)
        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0.0),
            Point(0.9, 0.1, 0.0),
            Point(0.9, 0.9, 0.0),
            Point(0.1, 0.9, 0.0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);

        // String representations
        std::string sstr = ts.str();
        std::string srepr = ts.repr();

        // Copy (new guid())
        NurbsSurfaceTrimmed tscopy = ts;

        MINI_CHECK(ts.is_valid());
        MINI_CHECK(ts.is_trimmed());
        MINI_CHECK(ts.name == "my_nurbssurface_trimmed");
        MINI_CHECK(!ts.guid().empty());
        MINI_CHECK(sstr.find("NurbsSurfaceTrimmed") != std::string::npos);
        MINI_CHECK(srepr.find("name=my_nurbssurface_trimmed") != std::string::npos);
        MINI_CHECK(tscopy.is_valid());
        MINI_CHECK(tscopy.guid() != ts.guid());
        MINI_CHECK(tscopy == ts);
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Constructor Planar") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        // Planar curve boundary
        std::vector<Point> pts = {
            Point(0, 0, 0),
            Point(3, 1, 0),
            Point(5, 0.5, 0),
            Point(6, 3, 0),
            Point(4, 5, 0),
            Point(1, 4, 0),
        };
        NurbsCurve bnd = NurbsCurve::create(true, 3, pts);
        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create_planar(bnd);

        // Rotated planar
        pts = {
            Point(0, 0, 0),
            Point(3, 1, -2),
            Point(5, 2, -3),
            Point(4, 4, 0),
            Point(1, 3, 2),
        };
        bnd = NurbsCurve::create(true, 3, pts);
        ts = NurbsSurfaceTrimmed::create_planar(bnd);

        // Triangle
        bnd = NurbsCurve::create(true, 1, {
            Point(0, 0, 0),
            Point(6, 3, 3),
            Point(2, 5, 1),
        });
        ts = NurbsSurfaceTrimmed::create_planar(bnd);

        // Trapezoid
        bnd = NurbsCurve::create(true, 1, {
            Point(0, 0, 6),
            Point(5, 0, 6),
            Point(4, 4, 2),
            Point(1, 4, 2),
        });
        ts = NurbsSurfaceTrimmed::create_planar(bnd);

        // Rectangle with a hole
        bnd = NurbsCurve::create(true, 1, {
            Point(0, 0, 0),
            Point(6, 0, 0),
            Point(6, 6, 0),
            Point(0, 6, 0),
        });
        ts = NurbsSurfaceTrimmed::create_planar(bnd);
        ts.add_hole(NurbsCurve::create(true, 1, {
            Point(2, 2, 0),
            Point(4, 2, 0),
            Point(4, 4, 0),
            Point(2, 4, 0),
        }));

        // Hexagon with 2 holes
        double R = 4.0;
        pts = {};
        for (int k = 0; k < 6; ++k) {
            double a = k * Tolerance::PI / 3.0;
            pts.push_back(Point(R*cos(a), R*sin(a), R*cos(a)*0.5));
        }
        bnd = NurbsCurve::create(true, 1, pts);
        ts = NurbsSurfaceTrimmed::create_planar(bnd);
        ts.add_holes({
            NurbsCurve::create(true, 1, {
                Point(1.5, 0.5, 0.75),
                Point(2.5, 0.5, 1.25),
                Point(2.0, 1.5, 1.0),
            }),
            NurbsCurve::create(true, 1, {
                Point(-2, -0.5, -1),
                Point(-1, -0.5, -0.5),
                Point(-1, -1.5, -0.5),
                Point(-2, -1.5, -1),
            }),
        });
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Constructor Hole") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "primitives.h"

        // Create surface with bump
        int n = 8;
        std::vector<Point> pts;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                double x = i, y = j;
                double r2 = (x - 1.5) * (x - 1.5) + (y - 1.5) * (y - 1.5);
                double z = 5.0 * exp(-r2 / 1.0) + 0.3 * sin(Tolerance::PI * x / 7) * sin(Tolerance::PI * y / 7);
                pts.push_back(Point(x, y, z));
            }

        NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, n, n, pts);

        // Create outer loop (full boundary in UV)
        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);

        // Add hole
        NurbsCurve hole = Primitives::circle(3.5, 3.5, 0, 1.0);
        ts.add_hole(hole);

        MINI_CHECK(ts.is_valid());
        MINI_CHECK(ts.is_trimmed());
        MINI_CHECK(ts.inner_loop_count() == 1);
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Accessors") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(5, 0, 0));
        srf.set_cv(0, 1, Point(0, 5, 0));
        srf.set_cv(1, 1, Point(5, 5, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0),
            Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0),
            Point(0.1, 0.9, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);
        ts.name = "test_accessors";
        ts.width = 2.5;

        NurbsSurface got_srf = ts.surface();
        NurbsCurve got_loop = ts.get_outer_loop();

        MINI_CHECK(ts.is_valid());
        MINI_CHECK(ts.is_trimmed());
        MINI_CHECK(ts.name == "test_accessors");
        MINI_CHECK(ts.width == 2.5);
        MINI_CHECK(got_srf.is_valid());
        MINI_CHECK(got_loop.is_valid());
        MINI_CHECK(ts.inner_loop_count() == 0);
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Add Inner Loop") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(10, 0, 0));
        srf.set_cv(0, 1, Point(0, 10, 0));
        srf.set_cv(1, 1, Point(10, 10, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);

        // Add inner loop (hole in UV)
        NurbsCurve hole1 = NurbsCurve::create(true, 1, {
            Point(0.2, 0.2, 0),
            Point(0.4, 0.2, 0),
            Point(0.4, 0.4, 0),
            Point(0.2, 0.4, 0),
        });
        NurbsCurve hole2 = NurbsCurve::create(true, 1, {
            Point(0.6, 0.6, 0),
            Point(0.8, 0.6, 0),
            Point(0.8, 0.8, 0),
            Point(0.6, 0.8, 0),
        });

        ts.add_inner_loop(hole1);
        ts.add_inner_loop(hole2);

        NurbsCurve got = ts.get_inner_loop(0);

        MINI_CHECK(ts.inner_loop_count() == 2);
        MINI_CHECK(got.is_valid());

        ts.clear_inner_loops();
        MINI_CHECK(ts.inner_loop_count() == 0);
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Point At") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(4, 0, 0));
        srf.set_cv(0, 1, Point(0, 4, 0));
        srf.set_cv(1, 1, Point(4, 4, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);

        auto dom_u = ts.surface().domain(0);
        auto dom_v = ts.surface().domain(1);
        double u_mid = (dom_u.first + dom_u.second) / 2.0;
        double v_mid = (dom_v.first + dom_v.second) / 2.0;

        Point pt = ts.point_at(u_mid, v_mid);
        Vector nm = ts.normal_at(u_mid, v_mid);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(std::abs(nm[2]), 1.0));
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Mesh") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "mesh.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(6, 0, 0));
        srf.set_cv(0, 1, Point(0, 6, 0));
        srf.set_cv(1, 1, Point(6, 6, 0));

        // Untrimmed mesh
        Mesh m_full = srf.mesh();

        // Trimmed mesh (smaller outer boundary)
        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0),
            Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0),
            Point(0.1, 0.9, 0),
        });
        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);
        Mesh m = ts.mesh();

        // Trimmed with hole
        NurbsCurve hole = NurbsCurve::create(true, 1, {
            Point(0.3, 0.3, 0),
            Point(0.7, 0.3, 0),
            Point(0.7, 0.7, 0),
            Point(0.3, 0.7, 0),
        });
        NurbsSurfaceTrimmed ts_hole = NurbsSurfaceTrimmed::create(srf, outer);
        ts_hole.add_inner_loop(hole);
        Mesh m_hole = ts_hole.mesh();

        MINI_CHECK(!m.is_empty());
        MINI_CHECK(m.number_of_vertices() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
        MINI_CHECK(m.number_of_faces() > 0);
        MINI_CHECK(m_hole.number_of_faces() > 0);

        // Planar circle (rational NURBS outer loop)
        double cw = std::sqrt(2.0) / 2.0;
        double ccx[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
        double ccy[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
        double cwt[9] = {1, cw, 1, cw, 1, cw, 1, cw, 1};
        NurbsCurve circle_loop(3, true, 3, 9);
        circle_loop.m_nurbsknot = {0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0};
        for (int i = 0; i < 9; ++i)
            circle_loop.set_cv_4d(i, (0.5 + 0.5*ccx[i])*cwt[i], (0.5 + 0.5*ccy[i])*cwt[i], 0.0, cwt[i]);
        NurbsSurfaceTrimmed ts_circ = NurbsSurfaceTrimmed::create(srf, circle_loop);
        Mesh mc = ts_circ.mesh();
        MINI_CHECK(!mc.is_empty());
        MINI_CHECK(mc.number_of_vertices() >= 30);
        MINI_CHECK(mc.number_of_faces() >= 30);
        for (const auto& [vk, vd] : mc.vertex) {
            double nx = 0, ny = 0, nz = 0;
            auto it = vd.attributes.find("nx"); if (it != vd.attributes.end()) nx = it->second;
            it = vd.attributes.find("ny"); if (it != vd.attributes.end()) ny = it->second;
            it = vd.attributes.find("nz"); if (it != vd.attributes.end()) nz = it->second;
            MINI_CHECK(std::sqrt(nx*nx + ny*ny + nz*nz) > 0.5);
        }
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Split By UV Curves") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "primitives.h"

        NurbsSurface srf = Primitives::wave_surface(10.0, 1.0);
        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        std::vector<Point> pts = {Point(u0 + (u1-u0)*0.4, v0, 0.0), Point(u0 + (u1-u0)*0.6, v1, 0.0)};
        NurbsCurve line = NurbsCurve::create(false, 1, pts);

        std::vector<NurbsSurfaceTrimmed> parts = NurbsSurfaceTrimmed::split_by_uv_curves(srf, {line});

        MINI_CHECK(parts.size() == 2);
        MINI_CHECK(parts[0].is_trimmed());
        MINI_CHECK(parts[1].is_trimmed());

        NurbsCurve circle = Primitives::circle((u0+u1)*0.5, (v0+v1)*0.5, 0.0, (u1-u0)*0.2);

        std::vector<NurbsSurfaceTrimmed> ring = NurbsSurfaceTrimmed::split_by_uv_curves(srf, {circle});

        MINI_CHECK(ring.size() == 2);
        MINI_CHECK(ring[0].inner_loop_count() + ring[1].inner_loop_count() == 1);

        NurbsCurve dangling = NurbsCurve::create(false, 1, {Point(3.0, 3.0, 0.0), Point(5.0, 5.0, 0.0)});

        std::vector<NurbsSurfaceTrimmed> whole = NurbsSurfaceTrimmed::split_by_uv_curves(srf, {dangling});

        MINI_CHECK(whole.size() == 1);
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Transformation") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "xform.h"

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(1, 0, 0));
        srf.set_cv(0, 1, Point(0, 1, 0));
        srf.set_cv(1, 1, Point(1, 1, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);
        Xform ts_xf = Xform::translation(10.0, 20.0, 30.0);
        NurbsSurfaceTrimmed ts2 = ts.transformed(ts_xf);

        auto dom_u = ts2.surface().domain(0);
        auto dom_v = ts2.surface().domain(1);
        Point pt = ts2.point_at(dom_u.first, dom_v.first);

        MINI_CHECK(TOLERANCE.is_close(pt[0], 10.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 20.0));
        MINI_CHECK(TOLERANCE.is_close(pt[2], 30.0));
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Json Roundtrip") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(5, 0, 0));
        srf.set_cv(0, 1, Point(0, 5, 0));
        srf.set_cv(1, 1, Point(5, 5, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0),
            Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0),
            Point(0.1, 0.9, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);
        ts.name = "test_nurbssurface_trimmed";
        ts.width = 2.0;
        ts.surfacecolor = Color(255, 128, 64, 255);

        // JSON object
        nlohmann::ordered_json json = ts.jsondump();
        NurbsSurfaceTrimmed loaded_json = NurbsSurfaceTrimmed::jsonload(json);

        // String
        std::string json_string = ts.file_json_dumps();
        NurbsSurfaceTrimmed loaded_json_string = NurbsSurfaceTrimmed::file_json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbssurface_trimmed.json").string();
        ts.file_json_dump(filename);
        NurbsSurfaceTrimmed loaded_from_file = NurbsSurfaceTrimmed::file_json_load(filename);

        MINI_CHECK(loaded_json == ts);
        MINI_CHECK(loaded_json_string == ts);
        MINI_CHECK(loaded_from_file == ts);
    }

    MINI_TEST("NurbsSurfaceTrimmed", "Protobuf Roundtrip") {
        // uncomment #include "nurbssurface_trimmed.h"
        // uncomment #include "nurbssurface.h"
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, Point(0, 0, 0));
        srf.set_cv(1, 0, Point(5, 0, 0));
        srf.set_cv(0, 1, Point(0, 5, 0));
        srf.set_cv(1, 1, Point(5, 5, 0));

        NurbsCurve outer = NurbsCurve::create(true, 1, {
            Point(0.1, 0.1, 0),
            Point(0.9, 0.1, 0),
            Point(0.9, 0.9, 0),
            Point(0.1, 0.9, 0),
        });

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);
        ts.name = "test_nurbssurface_trimmed";
        ts.width = 2.0;
        ts.surfacecolor = Color(255, 128, 64, 255);

        // String
        std::string proto_string = ts.pb_dumps();
        NurbsSurfaceTrimmed loaded_proto_string = NurbsSurfaceTrimmed::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbssurface_trimmed.bin").string();
        ts.pb_dump(filename);
        NurbsSurfaceTrimmed loaded = NurbsSurfaceTrimmed::pb_load(filename);

        MINI_CHECK(loaded_proto_string == ts);
        MINI_CHECK(loaded == ts);
    }

} // namespace session_cpp
