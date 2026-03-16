#include "mini_test.h"
#include "fmt/format.h"
#include "mesh.h"
#include "color.h"
#include "point.h"
#include "line.h"
#include "polyline.h"
#include "xform.h"
#include "tolerance.h"
#include "encoders.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Mesh", "Constructor") {
        // uncomment #include "mesh.h"

        std::vector<Point> vertices = Polyline::from_sides(6, 1.0, false).get_points();
        Mesh mesh = Mesh::from_vertices_and_faces(vertices, {{0, 1, 2, 3, 4, 5}});
        std::string sstr = mesh.str();
        std::string srepr = mesh.repr();
        Mesh mcopy = mesh;
        MINI_CHECK(mesh.is_valid());
        mesh.name = "hexagon";

        std::vector<Color> palette = Color::palette();

        // set_objectcolor does not change color_mode
        mesh.set_objectcolor(Color::grey());
        MINI_CHECK(mesh.color_mode == ColorMode::OBJECTCOLOR);

        // set_pointcolors → color_mode = PointColors
        std::vector<Color> pc;
        pc.reserve(mesh.number_of_vertices());
        for (size_t i = 0; i < mesh.number_of_vertices(); ++i)
            pc.emplace_back(palette[i % palette.size()]);
        mesh.set_pointcolors(std::move(pc));
        MINI_CHECK(mesh.color_mode == ColorMode::POINTCOLORS);
        MINI_CHECK(mesh.get_pointcolors().size() == mesh.number_of_vertices());

        // set_facecolors → color_mode = FaceColors
        std::vector<Color> fc;
        fc.reserve(mesh.number_of_faces());
        for (size_t i = 0; i < mesh.number_of_faces(); ++i)
            fc.emplace_back(palette[i % palette.size()]);
        mesh.set_facecolors(std::move(fc));
        MINI_CHECK(mesh.color_mode == ColorMode::FACECOLORS);
        MINI_CHECK(mesh.get_facecolors().size() == mesh.number_of_faces());

        // set_linecolors does not change color_mode
        std::vector<Color> lc;
        std::vector<double> lw(mesh.number_of_edges(), 0.1);
        lc.reserve(mesh.number_of_edges());
        for (size_t i = 0; i < mesh.number_of_edges(); ++i)
            lc.emplace_back(palette[i % palette.size()]);
        mesh.set_linecolors(std::move(lc), std::move(lw));
        MINI_CHECK(mesh.color_mode == ColorMode::FACECOLORS);
        MINI_CHECK(mesh.get_linecolors().size() == mesh.number_of_edges());

        // clear_facecolors reverts color_mode only if currently FaceColors
        mesh.color_mode = ColorMode::FACECOLORS;
        MINI_CHECK(mesh.color_mode == ColorMode::FACECOLORS);
        mesh.clear_facecolors();
        MINI_CHECK(mesh.color_mode == ColorMode::OBJECTCOLOR);
        MINI_CHECK(mesh.get_facecolors().empty());

        // clear_pointcolors does not revert if color_mode != PointColors
        mesh.color_mode = ColorMode::FACECOLORS;
        MINI_CHECK(mesh.color_mode == ColorMode::FACECOLORS);
        mesh.clear_pointcolors();
        MINI_CHECK(mesh.color_mode == ColorMode::FACECOLORS);

        // clear_linecolors does not change color_mode
        mesh.color_mode = ColorMode::POINTCOLORS;
        mesh.clear_linecolors();
        MINI_CHECK(mesh.color_mode == ColorMode::POINTCOLORS);
        MINI_CHECK(mesh.get_linecolors().empty());
    }

    MINI_TEST("Mesh", "From Polylines") {
        // uncomment #include "mesh.h"

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
        MINI_CHECK(mesh.is_valid());
    }

    MINI_TEST("Mesh", "From Lines") {
        // uncomment #include "mesh.h"

        std::vector<Line> lines = {
            Line::from_points(Point(4.948083, -0.149798, 1.00765),
                              Point(4.395544, -0.996413, 1.196018)),
            Line::from_points(Point(3.866593, 0.371225, 1.376346),
                              Point(4.567265, 0.584361, 1.137476)),
            Line::from_points(Point(3.915298, -0.157402, 1.359741),
                              Point(3.282977, -0.051356, 1.575309)),
            Line::from_points(Point(4.286215, -0.224964, 1.23329),
                              Point(3.607284, -0.987075, 1.464748)),
            Line::from_points(Point(3.744351, 0.971574, 1.41802),
                              Point(3.266367, 0.841359, 1.580972)),
            Line::from_points(Point(4.567265, 0.584361, 1.137476),
                              Point(4.948083, -0.149798, 1.00765)),
            Line::from_points(Point(4.395544, -0.996413, 1.196018),
                              Point(3.607284, -0.987075, 1.464748)),
            Line::from_points(Point(3.915298, -0.157402, 1.359741),
                              Point(4.286215, -0.224964, 1.23329)),
            Line::from_points(Point(3.282977, -0.051356, 1.575309),
                              Point(3.266367, 0.841359, 1.580972)),
            Line::from_points(Point(3.744351, 0.971574, 1.41802),
                              Point(3.866593, 0.371225, 1.376346)),
        };
        Mesh mesh = Mesh::from_lines(lines, true);
        MINI_CHECK(mesh.is_valid());
    }

    MINI_TEST("Mesh", "From Polygon With Holes") {
        // uncomment #include "mesh.h"

        Mesh mesh = Mesh::from_polygon_with_holes({
            {
                {8.940934, 0.917382, 0.049546},
                {8.930493, 1.36458, 0.251429},
                {8.954508, 1.595448, 0.346958},
                {9.457671, 1.821395, 0.298639},
                {9.717078, 1.014296, -0.136839},
                {9.363048, 0.91534, -0.07616},
                {9.33327, 0.459713, -0.269899},
                {9.065708, 0.635281, -0.112748},
            },
            {
                {7.494779, -0.556523, -0.178103},
                {6.542877, 0.148384, 0.416685},
                {6.967337, 2.119511, 1.167431},
                {11.204553, 2.961749, 0.289102},
                {9.658416, 0.465135, -0.363618},
                {10.247775, -1.032727, -1.203717},
            },
            {
                {7.922105, 0.548716, 0.186877},
                {7.410178, 0.844297, 0.469625},
                {7.408889, 1.185147, 0.621527},
                {7.885956, 1.424645, 0.586947},
                {8.178727, 1.32996, 0.458299},
                {8.307609, 0.88254, 0.2213},
                {7.950364, 0.924872, 0.345738},
            },
        }, true);
        MINI_CHECK(mesh.is_valid());

        Mesh mesh_sorted = Mesh::from_polygon_with_holes({
            {{1, 1, 0}, {3, 1, 0}, {3, 3, 0}, {1, 3, 0}},
            {{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}},
        }, true);
        MINI_CHECK(mesh_sorted.is_valid());
    }

    MINI_TEST("Mesh", "Loft") {
        // uncomment #include "mesh.h"

        std::vector<Polyline> bottom = {
            Polyline(std::vector<Point>{
                {13.20069,-0.556523,-0.178103},
                {12.248787,0.148384,0.416685},
                {12.673247,2.119511,1.167431},
                {16.910464,2.961749,0.289102},
                {15.364327,0.465135,-0.363618},
                {15.953685,-1.032727,-1.203717},
                {13.20069,-0.556523,-0.178103},
            }),
            Polyline(std::vector<Point>{
                {14.646845,0.917382,0.049546},
                {14.636404,1.36458,0.251429},
                {14.660418,1.595448,0.346958},
                {15.163581,1.821395,0.298639},
                {15.422988,1.014296,-0.136839},
                {15.068958,0.91534,-0.07616},
                {15.03918,0.459713,-0.269899},
                {14.771618,0.635281,-0.112748},
                {14.646845,0.917382,0.049546},
            }),
            Polyline(std::vector<Point>{
                {13.628016,0.548716,0.186877},
                {13.116088,0.844297,0.469625},
                {13.114799,1.185147,0.621527},
                {13.591866,1.424645,0.586947},
                {13.884637,1.32996,0.458299},
                {14.013519,0.88254,0.2213},
                {13.656275,0.924872,0.345738},
                {13.628016,0.548716,0.186877},
            }),
        };
        std::vector<Polyline> top = {
            Polyline(std::vector<Point>{
                {13.375135,-0.818817,0.411936},
                {12.423233,-0.113909,1.006724},
                {12.847692,1.857217,1.75747},
                {17.084909,2.699455,0.879141},
                {15.538772,0.202841,0.226421},
                {16.12813,-1.295021,-0.613678},
                {13.375135,-0.818817,0.411936},
            }),
            Polyline(std::vector<Point>{
                {14.82129,0.655088,0.639585},
                {14.810849,1.102286,0.841468},
                {14.834864,1.333154,0.936997},
                {15.338026,1.559101,0.888678},
                {15.597433,0.752002,0.4532},
                {15.243404,0.653046,0.513879},
                {15.213626,0.197419,0.32014},
                {14.946063,0.372987,0.477291},
                {14.82129,0.655088,0.639585},
            }),
            Polyline(std::vector<Point>{
                {13.802461,0.286422,0.776916},
                {13.290534,0.582003,1.059664},
                {13.289245,0.922853,1.211566},
                {13.766312,1.162351,1.176986},
                {14.059082,1.067666,1.048338},
                {14.187964,0.620246,0.811339},
                {13.83072,0.662578,0.935777},
                {13.802461,0.286422,0.776916},
            }),
        };
        Mesh mesh = Mesh::loft(bottom, top, true);
        MINI_CHECK(mesh.is_valid());
        MINI_CHECK(mesh.is_closed());

        Mesh mesh_no_cap = Mesh::loft(bottom, top, false);
        MINI_CHECK(mesh_no_cap.is_valid());
        MINI_CHECK(!mesh_no_cap.is_closed());
    }

    MINI_TEST("Mesh", "From Polygon With Holes Many") {
        // uncomment #include "mesh.h"

        std::vector<std::vector<std::vector<Point>>> inputs;
        for (int i = 0; i < 4; ++i) {
            double x = i * 7.0;
            inputs.push_back({
                {{x, 0, 0}, {x+5, 0, 0}, {x+5, 5, 0}, {x, 5, 0}},
                {{x+1, 1, 0}, {x+4, 1, 0}, {x+4, 4, 0}, {x+1, 4, 0}},
            });
        }
        std::vector<Mesh> meshes = Mesh::from_polygon_with_holes_many(inputs);
        MINI_CHECK(meshes[0].is_valid());
        MINI_CHECK(meshes[1].is_valid());
        MINI_CHECK(meshes[2].is_valid());
        MINI_CHECK(meshes[3].is_valid());
        std::vector<Mesh> meshes_seq = Mesh::from_polygon_with_holes_many(inputs, false, false);
        MINI_CHECK(meshes_seq[0].number_of_faces() == meshes[0].number_of_faces());
    }

    MINI_TEST("Mesh", "Loft Many") {
        // uncomment #include "mesh.h"

        std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>> loft_inputs;
        for (int i = 0; i < 6; ++i) {
            double x = i * 3.0;
            Polyline b(std::vector<Point>{
                {x, 0, 0}, {x+1, 0, 0}, {x+1, 1, 0}, {x, 1, 0}, {x, 0, 0}});
            Polyline t(std::vector<Point>{
                {x, 0, 1+i*0.5}, {x+1, 0, 1+i*0.5}, {x+1, 1, 1+i*0.5}, {x, 1, 1+i*0.5}, {x, 0, 1+i*0.5}});
            loft_inputs.push_back({{b}, {t}});
        }
        std::vector<Mesh> meshes = Mesh::loft_many(loft_inputs);

        MINI_CHECK(meshes[0].is_valid());
        MINI_CHECK(meshes[0].is_closed());
        MINI_CHECK(meshes[1].is_valid());
        MINI_CHECK(meshes[1].is_closed());
        MINI_CHECK(meshes[2].is_valid());
        MINI_CHECK(meshes[2].is_closed());
        MINI_CHECK(meshes[3].is_valid());
        MINI_CHECK(meshes[3].is_closed());
        MINI_CHECK(meshes[4].is_valid());
        MINI_CHECK(meshes[4].is_closed());
        MINI_CHECK(meshes[5].is_valid());
        MINI_CHECK(meshes[5].is_closed());

        std::vector<Mesh> meshes_seq = Mesh::loft_many(loft_inputs, true, false);

        MINI_CHECK(meshes_seq[0].is_valid());
        MINI_CHECK(meshes_seq[0].is_closed());
        MINI_CHECK(meshes_seq[1].is_valid());
        MINI_CHECK(meshes_seq[1].is_closed());
        MINI_CHECK(meshes_seq[2].is_valid());
        MINI_CHECK(meshes_seq[2].is_closed());
        MINI_CHECK(meshes_seq[3].is_valid());
        MINI_CHECK(meshes_seq[3].is_closed());
        MINI_CHECK(meshes_seq[4].is_valid());
        MINI_CHECK(meshes_seq[4].is_closed());
        MINI_CHECK(meshes_seq[5].is_valid());
        MINI_CHECK(meshes_seq[5].is_closed());

    }

    MINI_TEST("Mesh", "Loft with quads and triangles") {
        // uncomment #include "mesh.h"

        std::vector<std::vector<Point>> top7 = {
            {
                {250,-250,500},
                {250,250,500},
                {-250,250,500},
                {-250,-250,500},
                {250,-250,500},
            },
            {
                {-250,500,250},
                {-250,250,500},
                {250,250,500},
                {250,500,250},
                {-250,500,250},
            },
            {
                {250,-250,500},
                {500,-250,250},
                {500,250,250},
                {250,250,500},
                {250,-250,500},
            },
            {
                {250,500,250},
                {250,250,500},
                {500,250,250},
                {250,500,250},
            },
            {
                {-250,500,250},
                {250,500,250},
                {250,500,-250},
                {-250,500,-250},
                {-250,500,250},
            },
            {
                {250,500,250},
                {500,250,250},
                {500,250,-250},
                {250,500,-250},
                {250,500,250},
            },
            {
                {500,-250,250},
                {500,-250,-250},
                {500,250,-250},
                {500,250,250},
                {500,-250,250},
            },
        };
        std::vector<std::vector<Point>> bot7 = {
            {
                {270.710678,-250,550},
                {270.710678,265.891862,550},
                {265.891862,270.710678,550},
                {-250,270.710678,550},
                {-250,-250,550},
                {270.710678,-250,550},
            },
            {
                {270.710678,-250,550},
                {550,-250,270.710678},
                {550,265.891862,270.710678},
                {270.710678,265.891862,550},
                {270.710678,-250,550},
            },
            {
                {-250,550,270.710678},
                {-250,270.710678,550},
                {265.891862,270.710678,550},
                {265.891862,550,270.710678},
                {-250,550,270.710678},
            },
            {
                {265.891862,550,270.710678},
                {265.891862,270.710678,550},
                {270.710678,265.891862,550},
                {550,265.891862,270.710678},
                {550,270.710678,265.891862},
                {270.710678,550,265.891862},
                {265.891862,550,270.710678},
            },
            {
                {-250,550,270.710678},
                {265.891862,550,270.710678},
                {270.710678,550,265.891862},
                {270.710678,550,-250},
                {-250,550,-250},
                {-250,550,270.710678},
            },
            {
                {270.710678,550,265.891862},
                {550,270.710678,265.891862},
                {550,270.710678,-250},
                {270.710678,550,-250},
                {270.710678,550,265.891862},
            },
            {
                {550,-250,270.710678},
                {550,-250,-250},
                {550,270.710678,-250},
                {550,270.710678,265.891862},
                {550,265.891862,270.710678},
                {550,-250,270.710678},
            },
        };
        auto [panels, adj, top_mesh, bot_mesh] = Mesh::loft_panels(top7, bot7);

        // Color faces: blue=top cap, red=bot cap, gray=quad wall, yellow=tri wall
        for (size_t i = 0; i < panels.size(); i++) {
            std::vector<Color> face_colors;
            face_colors.reserve(panels[i].face_roles.size());
            for (const auto& [face_key, role] : panels[i].face_roles) {
                switch (role) {
                    case LoftFaceRole::TopCap:   face_colors.push_back(Color::blue()); break;
                    case LoftFaceRole::BotCap:   face_colors.push_back(Color::red());  break;
                    case LoftFaceRole::TriWall:  face_colors.push_back(Color::yellow()); break;
                    case LoftFaceRole::QuadWall: face_colors.push_back(Color::grey());break;
                }
            }
            panels[i].mesh.set_facecolors(face_colors);
        }

        // face centroids labelled with panel index
        for (size_t i = 0; i < panels.size(); i++) {
            auto c = panels[i].mesh.centroid();
            c.name = fmt::format("p{}", i);
        }

        // adjacency: for each shared edge — text dot at midpoint labelled "p{i}f{idx}<->p{j}f{idx}"
        for (const auto& [i, wi, pj, wj] : adj) {
            const auto& w = panels[i].wall_faces[wi];
            auto pt = *panels[i].mesh.face_centroid(w.face_key);
            pt.name = fmt::format("p{} f{} - p{} f{}", i, w.face_index, pj, panels[pj].wall_faces[wj].face_index);
        }

        MINI_CHECK(panels.size() == 7);
        MINI_CHECK(panels[0].mesh.is_valid());
        MINI_CHECK(panels[1].mesh.is_valid());
        MINI_CHECK(panels[2].mesh.is_valid());
        MINI_CHECK(panels[3].mesh.is_valid());
        MINI_CHECK(panels[4].mesh.is_valid());
        MINI_CHECK(panels[5].mesh.is_valid());
        MINI_CHECK(panels[6].mesh.is_valid());

        MINI_CHECK(adj.size() == 9);
        MINI_CHECK(adj[0].pi == 0 && adj[0].pj == 2);
        MINI_CHECK(adj[1].pi == 0 && adj[1].pj == 1);
        MINI_CHECK(adj[2].pi == 1 && adj[2].pj == 3);
        MINI_CHECK(adj[3].pi == 1 && adj[3].pj == 4);
        MINI_CHECK(adj[4].pi == 2 && adj[4].pj == 6);
        MINI_CHECK(adj[5].pi == 2 && adj[5].pj == 3);
        MINI_CHECK(adj[6].pi == 3 && adj[6].pj == 5);
        MINI_CHECK(adj[7].pi == 4 && adj[7].pj == 5);
        MINI_CHECK(adj[8].pi == 5 && adj[8].pj == 6);
          
    }

    MINI_TEST("Mesh", "Boolean Queries") {
        // uncomment #include "mesh.h"

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

        size_t v0 = 1;
        size_t v1 = 2;
        size_t v2 = 3;
        size_t f0 = 0;

        bool empty = mesh.is_empty();
        MINI_CHECK(!empty);

        bool valid = mesh.is_valid();
        MINI_CHECK(valid);

        bool closed = mesh.is_closed();
        MINI_CHECK(!closed);

        bool vertex_on_boundary = mesh.is_vertex_on_boundary(v0);
        MINI_CHECK(!vertex_on_boundary);

        bool edge_not_on_boundary = mesh.is_edge_on_boundary(v0, v1);
        MINI_CHECK(!edge_not_on_boundary);

        bool edge_on_boundary = mesh.is_edge_on_boundary(v1, v2);
        MINI_CHECK(edge_on_boundary);

        bool face_on_boundary = mesh.is_face_on_boundary(f0);
        MINI_CHECK(face_on_boundary);
    }

    MINI_TEST("Mesh", "Attributes") {
        // uncomment #include "mesh.h"

        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);

        size_t n_vertices = mesh.number_of_vertices();
        MINI_CHECK(n_vertices == 8);

        size_t n_faces = mesh.number_of_faces();
        MINI_CHECK(n_faces == 6);

        size_t n_edges = mesh.number_of_edges();
        MINI_CHECK(n_edges == 12);

        size_t euler = mesh.euler();
        MINI_CHECK(euler == 2);

        auto [pts, fidx] = mesh.to_vertices_and_faces();
        MINI_CHECK(fidx.size() == n_faces);
        MINI_CHECK(pts.size() == n_vertices);
        MINI_CHECK(TOLERANCE.is_point_close(pts[0], Point(-0.5, -0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[1], Point( 0.5, -0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[2], Point( 0.5,  0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[3], Point(-0.5,  0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[4], Point(-0.5, -0.5,  0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[5], Point( 0.5, -0.5,  0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[6], Point( 0.5,  0.5,  0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(pts[7], Point(-0.5,  0.5,  0.5)));
        MINI_CHECK((fidx[0] == std::vector<size_t>{0, 3, 2, 1}));
        MINI_CHECK((fidx[1] == std::vector<size_t>{4, 5, 6, 7}));
        MINI_CHECK((fidx[2] == std::vector<size_t>{0, 1, 5, 4}));
        MINI_CHECK((fidx[3] == std::vector<size_t>{2, 3, 7, 6}));
        MINI_CHECK((fidx[4] == std::vector<size_t>{0, 4, 7, 3}));
        MINI_CHECK((fidx[5] == std::vector<size_t>{1, 2, 6, 5}));


        std::map<size_t, size_t> vertex_to_index = mesh.vertex_index();
        MINI_CHECK(vertex_to_index.size() == n_vertices);
        MINI_CHECK(vertex_to_index[0] == 0);
        MINI_CHECK(vertex_to_index[1] == 1);
        MINI_CHECK(vertex_to_index[2] == 2);
        MINI_CHECK(vertex_to_index[3] == 3);
        MINI_CHECK(vertex_to_index[4] == 4);
        MINI_CHECK(vertex_to_index[5] == 5);
        MINI_CHECK(vertex_to_index[6] == 6);
        MINI_CHECK(vertex_to_index[7] == 7);

        // vertices / faces / edges
        auto vertices = mesh.vertices();
        MINI_CHECK(vertices.size() == 8);
        MINI_CHECK(vertices[0] == 0);
        MINI_CHECK(vertices[1] == 1);
        MINI_CHECK(vertices[2] == 2);
        MINI_CHECK(vertices[3] == 3);
        MINI_CHECK(vertices[4] == 4);
        MINI_CHECK(vertices[5] == 5);
        MINI_CHECK(vertices[6] == 6);
        MINI_CHECK(vertices[7] == 7);

        auto faces = mesh.faces();
        MINI_CHECK(faces.size() == 6);
        MINI_CHECK(faces[0] == 0);
        MINI_CHECK(faces[1] == 1);
        MINI_CHECK(faces[2] == 2);
        MINI_CHECK(faces[3] == 3);
        MINI_CHECK(faces[4] == 4);
        MINI_CHECK(faces[5] == 5);

        auto edges = mesh.edges();
        MINI_CHECK(edges.size() == 12);
        MINI_CHECK(edges[0]  == std::make_pair(0ul, 1ul));
        MINI_CHECK(edges[1]  == std::make_pair(0ul, 3ul));
        MINI_CHECK(edges[2]  == std::make_pair(0ul, 4ul));
        MINI_CHECK(edges[3]  == std::make_pair(1ul, 2ul));
        MINI_CHECK(edges[4]  == std::make_pair(1ul, 5ul));
        MINI_CHECK(edges[5]  == std::make_pair(2ul, 3ul));
        MINI_CHECK(edges[6]  == std::make_pair(2ul, 6ul));
        MINI_CHECK(edges[7]  == std::make_pair(3ul, 7ul));
        MINI_CHECK(edges[8]  == std::make_pair(4ul, 5ul));
        MINI_CHECK(edges[9]  == std::make_pair(4ul, 7ul));
        MINI_CHECK(edges[10] == std::make_pair(5ul, 6ul));
        MINI_CHECK(edges[11] == std::make_pair(6ul, 7ul));

        MINI_CHECK(mesh.naked_edges(true).size() == 0);
        MINI_CHECK(mesh.naked_faces(false).size() == 6);

        // remove one face — box becomes open, check naked
        mesh.remove_face(mesh.faces()[0]);

        auto ne = mesh.naked_edges(true);
        MINI_CHECK(ne.size() == 4);
        MINI_CHECK(ne[0] == std::make_pair(0ul, 1ul));
        auto ni = mesh.naked_edges(false);
        MINI_CHECK(ni.size() == 8);
        auto nv = mesh.naked_vertices(true);
        MINI_CHECK(nv.size() == 4);
        auto nvi = mesh.naked_vertices(false);
        MINI_CHECK(nvi.size() == 4);
        auto nf = mesh.naked_faces(true);
        MINI_CHECK(nf.size() == 4);
        auto nfi = mesh.naked_faces(false);
        MINI_CHECK(nfi.size() == 1);
        // sparse keys via remove_vertex: key != index after removal
        size_t kr = mesh.vertices()[3];
        mesh.remove_vertex(kr);
        vertex_to_index = mesh.vertex_index();
        MINI_CHECK(vertex_to_index.size() == 7);
        MINI_CHECK(vertex_to_index[0] == 0);
        MINI_CHECK(vertex_to_index[1] == 1);
        MINI_CHECK(vertex_to_index[2] == 2);
        MINI_CHECK(vertex_to_index.count(3) == 0);
        MINI_CHECK(vertex_to_index[4] == 3);
        MINI_CHECK(vertex_to_index[5] == 4);
        MINI_CHECK(vertex_to_index[6] == 5);
        MINI_CHECK(vertex_to_index[7] == 6);
    }

    MINI_TEST("Mesh", "Vertex and Face Operations") {
        // uncomment #include "mesh.h"

        double hx = 0.5, hy = 0.5, hz = 0.5;
        std::vector<Point> verts = {
            Point(-hx, -hy, -hz), Point( hx, -hy, -hz), Point( hx,  hy, -hz), Point(-hx,  hy, -hz),
            Point(-hx, -hy,  hz), Point( hx, -hy,  hz), Point( hx,  hy,  hz), Point(-hx,  hy,  hz),
        };
        std::vector<std::vector<size_t>> faces = {
            {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 4, 7, 3}, {1, 2, 6, 5},
        };

        Mesh mesh = Mesh();

        
        for (const auto& v : verts) 
            mesh.add_vertex(v);
    
        for (const auto& f : faces) 
            mesh.add_face(f);

        // add_face: invalid (too few vertices)
        MINI_CHECK(!mesh.add_face({0, 1}, std::nullopt).has_value());
        // add_face: invalid (duplicate vertex)
        MINI_CHECK(!mesh.add_face({0, 1, 0}, std::nullopt).has_value());

        // remove_vertex(0): removes vertex 0 + 3 adjacent faces (0,2,4)
        // vertices → [1,2,3,4,5,6,7], faces → [1,3,5]
        mesh.remove_vertex(0);
        MINI_CHECK(mesh.number_of_vertices() == 7);
        MINI_CHECK(mesh.number_of_faces() == 3);

        // remove_edge(1,2): removes face 5 [1,2,6,5], faces → [1,3]
        mesh.remove_edge(1, 2);
        MINI_CHECK(mesh.number_of_faces() == 2);

        // remove_face(1): removes face 1 [4,5,6,7], faces → [3]
        mesh.remove_face(1);
        MINI_CHECK(mesh.number_of_faces() == 1);

        // clear
        mesh.clear();
        MINI_CHECK(mesh.is_empty());

        // rebuild
        for (const auto& v : verts) mesh.add_vertex(v);
        for (const auto& f : faces) mesh.add_face(f);

        // unweld and weld
        mesh = mesh.unweld();
        MINI_CHECK(mesh.number_of_vertices() == 24);
        mesh = mesh.weld(0.001);
        MINI_CHECK(mesh.number_of_vertices() == 8);
        MINI_CHECK(mesh.number_of_faces() == 6);
        // face 0: 0 1 2 3, face 1: 4 5 6 7, face 2: 0 3 5 4
        // face 3: 2 1 7 6, face 4: 0 4 7 1, face 5: 3 2 6 5
        auto fv0 = *mesh.face_vertices(0); auto fv1 = *mesh.face_vertices(1);
        auto fv2 = *mesh.face_vertices(2); auto fv3 = *mesh.face_vertices(3);
        auto fv4 = *mesh.face_vertices(4); auto fv5 = *mesh.face_vertices(5);
        MINI_CHECK(fv0[0] == 0 && fv0[1] == 1 && fv0[2] == 2 && fv0[3] == 3);
        MINI_CHECK(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);

        // flip_face(0): face 0 → [3,2,1,0], faces 1-5 unchanged
        mesh.flip_face(0);
        fv0 = *mesh.face_vertices(0); fv1 = *mesh.face_vertices(1);
        fv2 = *mesh.face_vertices(2); fv3 = *mesh.face_vertices(3);
        fv4 = *mesh.face_vertices(4); fv5 = *mesh.face_vertices(5);
        MINI_CHECK(fv0[0] == 3 && fv0[1] == 2 && fv0[2] == 1 && fv0[3] == 0);
        MINI_CHECK(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);

        // unify_winding: face 0 restored to [0,1,2,3], faces 1-5 unchanged
        mesh.unify_winding();
        fv0 = *mesh.face_vertices(0); fv1 = *mesh.face_vertices(1);
        fv2 = *mesh.face_vertices(2); fv3 = *mesh.face_vertices(3);
        fv4 = *mesh.face_vertices(4); fv5 = *mesh.face_vertices(5);
        MINI_CHECK(fv0[0] == 0 && fv0[1] == 1 && fv0[2] == 2 && fv0[3] == 3);
        MINI_CHECK(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);

        // flip: face 0 → [3,2,1,0], face 1 → [7,6,5,4], face 2 → [4,5,3,0]
        // face 3 → [6,7,1,2], face 4 → [1,7,4,0], face 5 → [5,6,2,3]
        mesh.flip();
        fv0 = *mesh.face_vertices(0); fv1 = *mesh.face_vertices(1);
        fv2 = *mesh.face_vertices(2); fv3 = *mesh.face_vertices(3);
        fv4 = *mesh.face_vertices(4); fv5 = *mesh.face_vertices(5);
        MINI_CHECK(fv0[0] == 3 && fv0[1] == 2 && fv0[2] == 1 && fv0[3] == 0);
        MINI_CHECK(fv1[0] == 7 && fv1[1] == 6 && fv1[2] == 5 && fv1[3] == 4);
        MINI_CHECK(fv2[0] == 4 && fv2[1] == 5 && fv2[2] == 3 && fv2[3] == 0);
        MINI_CHECK(fv3[0] == 6 && fv3[1] == 7 && fv3[2] == 1 && fv3[3] == 2);
        MINI_CHECK(fv4[0] == 1 && fv4[1] == 7 && fv4[2] == 4 && fv4[3] == 0);
        MINI_CHECK(fv5[0] == 5 && fv5[1] == 6 && fv5[2] == 2 && fv5[3] == 3);

        // orient_outward: face 0 → [0,1,2,3], face 1 → [4,5,6,7], face 2 → [0,3,5,4]
        // face 3 → [2,1,7,6], face 4 → [0,4,7,1], face 5 → [3,2,6,5]
        mesh.orient_outward();
        fv0 = *mesh.face_vertices(0); fv1 = *mesh.face_vertices(1);
        fv2 = *mesh.face_vertices(2); fv3 = *mesh.face_vertices(3);
        fv4 = *mesh.face_vertices(4); fv5 = *mesh.face_vertices(5);
        MINI_CHECK(fv0[0] == 0 && fv0[1] == 1 && fv0[2] == 2 && fv0[3] == 3);
        MINI_CHECK(fv1[0] == 4 && fv1[1] == 5 && fv1[2] == 6 && fv1[3] == 7);
        MINI_CHECK(fv2[0] == 0 && fv2[1] == 3 && fv2[2] == 5 && fv2[3] == 4);
        MINI_CHECK(fv3[0] == 2 && fv3[1] == 1 && fv3[2] == 7 && fv3[3] == 6);
        MINI_CHECK(fv4[0] == 0 && fv4[1] == 4 && fv4[2] == 7 && fv4[3] == 1);
        MINI_CHECK(fv5[0] == 3 && fv5[1] == 2 && fv5[2] == 6 && fv5[3] == 5);
    }


    MINI_TEST("Mesh", "Connectivity Queries") {
        // uncomment #include "mesh.h"

        std::vector<Point> pts = {{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}, {2,0,0}};
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0,1,2,3}, {1,4,2}});
        auto vi = mesh.vertex_index();
        auto vkeys = mesh.vertices();
        size_t v0 = vkeys[0], v1 = vkeys[1], v2 = vkeys[2], v3 = vkeys[3], v4 = vkeys[4];
        auto fkeys = mesh.faces();
        size_t f0 = fkeys[0], f1 = fkeys[1];

        // vertex_position
        std::optional<Point> pos = mesh.vertex_position(v0);
        MINI_CHECK(pos.has_value());
        MINI_CHECK(TOLERANCE.is_point_close(*pos, Point(0.0, 0.0, 0.0)));
        MINI_CHECK(!mesh.vertex_position(999).has_value());

        // face_vertices
        std::optional<std::vector<size_t>> fv = mesh.face_vertices(f0);
        MINI_CHECK(fv.has_value());
        MINI_CHECK(fv->size() == 4);
        MINI_CHECK((*fv)[0] == v0 && (*fv)[1] == v1 && (*fv)[2] == v2 && (*fv)[3] == v3);

        // vertex_neighbors
        std::vector<size_t> nb = mesh.vertex_neighbors(v1);
        MINI_CHECK(nb.size() == 3);

        // vertex_faces
        std::vector<size_t> vf0 = mesh.vertex_faces(v0);
        MINI_CHECK(vf0.size() == 1);
        std::vector<size_t> vf1 = mesh.vertex_faces(v1);
        MINI_CHECK(vf1.size() == 2);

        // vertex_edges
        std::vector<std::pair<size_t, size_t>> ve = mesh.vertex_edges(v1);
        MINI_CHECK(ve.size() == 3);
        std::set<std::pair<size_t, size_t>> ve_set(ve.begin(), ve.end());
        MINI_CHECK(ve_set.contains({v1, v0}));
        MINI_CHECK(ve_set.contains({v1, v2}));
        MINI_CHECK(ve_set.contains({v1, v4}));

        // face_edges
        std::vector<std::pair<size_t, size_t>> fe = mesh.face_edges(f0);
        MINI_CHECK(fe.size() == 4);
        MINI_CHECK(fe[0] == std::make_pair(v0, v1));
        MINI_CHECK(fe[1] == std::make_pair(v1, v2));
        MINI_CHECK(fe[2] == std::make_pair(v2, v3));
        MINI_CHECK(fe[3] == std::make_pair(v3, v0));

        // face_neighbors
        std::vector<size_t> fn0 = mesh.face_neighbors(f0);
        MINI_CHECK(fn0.size() == 1);
        MINI_CHECK(fn0[0] == f1);

        // edge_vertices
        std::array<size_t, 2> ev = mesh.edge_vertices(v0, v1);
        MINI_CHECK(ev[0] == v0 && ev[1] == v1);

        // edge_faces
        std::pair<std::optional<size_t>, std::optional<size_t>> ef_inner = mesh.edge_faces(v1, v2);
        MINI_CHECK(ef_inner.first.has_value() && ef_inner.second.has_value());
        std::pair<std::optional<size_t>, std::optional<size_t>> ef_boundary = mesh.edge_faces(v0, v1);
        MINI_CHECK(ef_boundary.first.has_value() != ef_boundary.second.has_value());

        // edge_edges
        std::vector<std::pair<size_t, size_t>> ee = mesh.edge_edges(v1, v2);
        MINI_CHECK(ee.size() == 4);
        std::set<std::pair<size_t, size_t>> ee_set(ee.begin(), ee.end());
        MINI_CHECK(!ee_set.contains({v1, v2}));
        MINI_CHECK(!ee_set.contains({v2, v1}));
    }

    MINI_TEST("Mesh", "Geometric Properties") {
        // uncomment #include "mesh.h"

        std::vector<Point> pts = {{0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}};
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0,1,3}, {0,3,2}});
        auto vi = mesh.vertex_index();
        auto vkeys = mesh.vertices();
        size_t v0 = vkeys[0], v1 = vkeys[1], v3 = vkeys[3];
        size_t f0 = mesh.faces()[0];

        // face_normal
        std::optional<Vector> fn = mesh.face_normal(f0);
        MINI_CHECK(fn.has_value());
        MINI_CHECK(TOLERANCE.is_close((*fn)[2], 1.0));

        // vertex_normal
        std::optional<Vector> vn = mesh.vertex_normal(v0);
        MINI_CHECK(vn.has_value());
        MINI_CHECK(std::abs((*vn)[2]) == 1.0);

        // vertex_normal_weighted
        std::optional<Vector> vnw = mesh.vertex_normal_weighted(v0, NormalWeighting::Angle);
        MINI_CHECK(vnw.has_value());
        MINI_CHECK(TOLERANCE.is_close((*vnw)[2], 1.0));

        // face_area
        std::optional<double> area = mesh.face_area(f0);
        MINI_CHECK(area.has_value());
        MINI_CHECK(TOLERANCE.is_close(*area, 0.5));

        // vertex_angle_in_face
        std::optional<double> angle = mesh.vertex_angle_in_face(v0, f0);
        MINI_CHECK(angle.has_value());
        MINI_CHECK(TOLERANCE.is_close(*angle, TOLERANCE.PI / 2.0));
        MINI_CHECK(!mesh.vertex_angle_in_face(999, f0).has_value());

        // dihedral_angle — interior edge v0-v3 shared by f0 and f1 (coplanar = PI)
        std::optional<double> da = mesh.dihedral_angle(v3, v0);
        MINI_CHECK(da.has_value());
        MINI_CHECK(TOLERANCE.is_close(*da, TOLERANCE.PI));
        // boundary edge — only one face
        MINI_CHECK(!mesh.dihedral_angle(v0, v1).has_value());

        // face_normals
        std::map<size_t, Vector> fns = mesh.face_normals();
        MINI_CHECK(fns.size() == 2);
        MINI_CHECK(TOLERANCE.is_close(fns[f0][2], 1.0));

        // vertex_normals
        std::map<size_t, Vector> vns = mesh.vertex_normals();
        MINI_CHECK(vns.size() == mesh.number_of_vertices());
        MINI_CHECK(TOLERANCE.is_close(vns[v0][2], 1.0));

        // vertex_normals_weighted
        std::map<size_t, Vector> vnsw = mesh.vertex_normals_weighted(NormalWeighting::Angle);
        MINI_CHECK(vnsw.size() == mesh.number_of_vertices());
        MINI_CHECK(TOLERANCE.is_close(vnsw[v0][2], 1.0));

        // area
        Mesh box = Mesh::create_box(2.0, 2.0, 2.0);
        MINI_CHECK(TOLERANCE.is_close(box.area(), 24.0));

        // volume
        MINI_CHECK(TOLERANCE.is_close(box.volume(), 8.0));
    }

    MINI_TEST("Mesh", "Transformation") {
        // uncomment #include "mesh.h"

        std::vector<Point> pts = {{0,0,0}, {1,0,0}, {0,1,0}};
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0,1,2}});
        size_t v0 = mesh.vertices()[0];

        // transform() — apply stored xform in-place; xform field unchanged
        Mesh mesh1 = mesh;
        mesh1.xform = Xform::translation(0.0, 0.0, 1.0);
        mesh1.transform();
        MINI_CHECK(!mesh1.xform.is_identity());
        MINI_CHECK((*mesh1.vertex_position(v0))[2] == 1.0);

        // transform(const Xform&) — apply given xform in-place; stored xform unchanged
        Mesh mesh2 = mesh;
        Xform x = Xform::translation(0.0, 0.0, 1.0);
        mesh2.transform(x);
        MINI_CHECK(mesh2.xform.is_identity());
        MINI_CHECK((*mesh2.vertex_position(v0))[2] == 1.0);

        // transformed() — copy with stored xform applied
        Mesh mesh3 = mesh;
        mesh3.xform = Xform::translation(0.0, 0.0, 10.0);
        Mesh mesh3t = mesh3.transformed();
        MINI_CHECK(!mesh3t.xform.is_identity());
        MINI_CHECK((*mesh3t.vertex_position(v0))[2] == 10.0);

        // transformed(const Xform&) — copy with given xform applied
        Mesh mesh4 = mesh;
        x = Xform::translation(0.0, 0.0, 10.0);
        Mesh mesh4t = mesh4.transformed(x);
        MINI_CHECK(mesh4t.xform.is_identity());
        MINI_CHECK((*mesh4t.vertex_position(v0))[2] == 10.0);
    }

    MINI_TEST("Mesh", "Json Roundtrip") {
        // uncomment #include "mesh.h"

        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh";
        mesh.set_objectcolor(Color(255, 0, 0, 255));
        std::vector<Color> fc;
        fc.reserve(mesh.number_of_faces());
        for (size_t i = 0; i < mesh.number_of_faces(); ++i) fc.push_back(Color(255, 0, 0, 255));
        mesh.set_facecolors(std::move(fc));

        // JSON object
        mesh.xform = Xform::translation(1.0, 2.0, 3.0);
        nlohmann::ordered_json json = mesh.jsondump();
        Mesh loaded_json = Mesh::jsonload(json);
        MINI_CHECK(loaded_json.name == mesh.name);
        MINI_CHECK(loaded_json.get_objectcolor() == mesh.get_objectcolor());
        MINI_CHECK(loaded_json.color_mode == mesh.color_mode);
        MINI_CHECK(loaded_json.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded_json.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK(loaded_json.xform == mesh.xform);

        // String
        std::string json_string = mesh.json_dumps();
        Mesh loaded_string = Mesh::json_loads(json_string);
        MINI_CHECK(loaded_string.name == mesh.name);
        MINI_CHECK(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh.json").string();
        mesh.json_dump(filename);
        Mesh loaded_file = Mesh::json_load(filename);
        MINI_CHECK(loaded_file.name == mesh.name);
        MINI_CHECK(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded_file.number_of_faces() == mesh.number_of_faces());

        // Triangulation roundtrip
        std::vector<std::vector<Point>> polys = {{Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)}};
        Mesh pmesh = Mesh::from_polylines(polys, std::nullopt);
        MINI_CHECK(!pmesh.get_triangulation().empty());
        nlohmann::ordered_json pjson = pmesh.jsondump();
        Mesh loaded_tri = Mesh::jsonload(pjson);
        size_t fk = pmesh.get_triangulation().begin()->first;
        MINI_CHECK(!loaded_tri.get_triangulation().empty());
        MINI_CHECK(loaded_tri.get_triangulation().count(fk) > 0);

        // Face holes roundtrip
        Mesh hmesh = Mesh::from_polygon_with_holes(
            {{Point(0,0,0),Point(4,0,0),Point(4,4,0),Point(0,4,0)},
             {Point(1,1,0),Point(3,1,0),Point(3,3,0),Point(1,3,0)}}, true);
        MINI_CHECK(!hmesh.get_face_holes().empty());
        Mesh loaded_holes = Mesh::jsonload(hmesh.jsondump());
        size_t hfk = hmesh.get_face_holes().begin()->first;
        MINI_CHECK(!loaded_holes.get_face_holes().empty());
        MINI_CHECK(loaded_holes.get_face_holes().at(hfk) == hmesh.get_face_holes().at(hfk));
    }

    MINI_TEST("Mesh", "Protobuf Roundtrip") {
        // uncomment #include "mesh.h"

        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh_proto";
        mesh.set_objectcolor(Color(255, 0, 0, 255));
        std::vector<Color> fc;
        fc.reserve(mesh.number_of_faces());
        for (size_t i = 0; i < mesh.number_of_faces(); ++i) fc.push_back(Color(255, 0, 0, 255));
        mesh.set_facecolors(std::move(fc));

        // String
        std::string proto_string = mesh.pb_dumps();
        Mesh loaded_string = Mesh::pb_loads(proto_string);
        MINI_CHECK(loaded_string.name == mesh.name);
        MINI_CHECK(loaded_string.get_objectcolor() == mesh.get_objectcolor());
        MINI_CHECK(loaded_string.color_mode == mesh.color_mode);
        MINI_CHECK(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh.bin").string();
        mesh.pb_dump(filename);
        Mesh loaded_file = Mesh::pb_load(filename);
        MINI_CHECK(loaded_file.name == mesh.name);
        MINI_CHECK(loaded_file.get_objectcolor() == mesh.get_objectcolor());
        MINI_CHECK(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded_file.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK(loaded_file.guid == mesh.guid);

        // Triangulation roundtrip
        std::vector<std::vector<Point>> polys = {{Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)}};
        Mesh pmesh = Mesh::from_polylines(polys, std::nullopt);
        MINI_CHECK(!pmesh.get_triangulation().empty());
        Mesh loaded_tri = Mesh::pb_loads(pmesh.pb_dumps());
        size_t fk = pmesh.get_triangulation().begin()->first;
        MINI_CHECK(!loaded_tri.get_triangulation().empty());
        MINI_CHECK(loaded_tri.get_triangulation().count(fk) > 0);

        // Face holes roundtrip
        Mesh hmesh = Mesh::from_polygon_with_holes(
            {{Point(0,0,0),Point(4,0,0),Point(4,4,0),Point(0,4,0)},
             {Point(1,1,0),Point(3,1,0),Point(3,3,0),Point(1,3,0)}}, true);
        MINI_CHECK(!hmesh.get_face_holes().empty());
        Mesh loaded_holes = Mesh::pb_loads(hmesh.pb_dumps());
        size_t hfk = hmesh.get_face_holes().begin()->first;
        MINI_CHECK(!loaded_holes.get_face_holes().empty());
        MINI_CHECK(loaded_holes.get_face_holes().at(hfk) == hmesh.get_face_holes().at(hfk));
    }

} // namespace session_cpp
