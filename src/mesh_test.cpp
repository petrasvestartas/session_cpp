#include "mini_test.h"
#include "fmt/format.h"
#include "mesh.h"
#include "color.h"
#include "point.h"
#include "line.h"
#include "polyline.h"
#include "xform.h"
#include "tolerance.h"
#include "file_encoders.h"
#include "session.h"

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
            {
                {1, 1, 0},
                {3, 1, 0},
                {3, 3, 0},
                {1, 3, 0},
            },
            {
                {0, 0, 0},
                {4, 0, 0},
                {4, 4, 0},
                {0, 4, 0},
            },
        }, true);
        MINI_CHECK(mesh_sorted.is_valid());
    }

    MINI_TEST("Mesh", "Loft") {
        // uncomment #include "mesh.h"

        std::vector<Polyline> bottom = {
            Polyline(std::vector<Point>{
                {13.20069, -0.556523, -0.178103},
                {12.248787, 0.148384, 0.416685},
                {12.673247, 2.119511, 1.167431},
                {16.910464, 2.961749, 0.289102},
                {15.364327, 0.465135, -0.363618},
                {15.953685, -1.032727, -1.203717},
                {13.20069, -0.556523, -0.178103},
            }),
            Polyline(std::vector<Point>{
                {14.646845, 0.917382, 0.049546},
                {14.636404, 1.36458, 0.251429},
                {14.660418, 1.595448, 0.346958},
                {15.163581, 1.821395, 0.298639},
                {15.422988, 1.014296, -0.136839},
                {15.068958, 0.91534, -0.07616},
                {15.03918, 0.459713, -0.269899},
                {14.771618, 0.635281, -0.112748},
                {14.646845, 0.917382, 0.049546},
            }),
            Polyline(std::vector<Point>{
                {13.628016, 0.548716, 0.186877},
                {13.116088, 0.844297, 0.469625},
                {13.114799, 1.185147, 0.621527},
                {13.591866, 1.424645, 0.586947},
                {13.884637, 1.32996, 0.458299},
                {14.013519, 0.88254, 0.2213},
                {13.656275, 0.924872, 0.345738},
                {13.628016, 0.548716, 0.186877},
            }),
        };
        std::vector<Polyline> top = {
            Polyline(std::vector<Point>{
                {13.375135, -0.818817, 0.411936},
                {12.423233, -0.113909, 1.006724},
                {12.847692, 1.857217, 1.75747},
                {17.084909, 2.699455, 0.879141},
                {15.538772, 0.202841, 0.226421},
                {16.12813, -1.295021, -0.613678},
                {13.375135, -0.818817, 0.411936},
            }),
            Polyline(std::vector<Point>{
                {14.82129, 0.655088, 0.639585},
                {14.810849, 1.102286, 0.841468},
                {14.834864, 1.333154, 0.936997},
                {15.338026, 1.559101, 0.888678},
                {15.597433, 0.752002, 0.4532},
                {15.243404, 0.653046, 0.513879},
                {15.213626, 0.197419, 0.32014},
                {14.946063, 0.372987, 0.477291},
                {14.82129, 0.655088, 0.639585},
            }),
            Polyline(std::vector<Point>{
                {13.802461, 0.286422, 0.776916},
                {13.290534, 0.582003, 1.059664},
                {13.289245, 0.922853, 1.211566},
                {13.766312, 1.162351, 1.176986},
                {14.059082, 1.067666, 1.048338},
                {14.187964, 0.620246, 0.811339},
                {13.83072, 0.662578, 0.935777},
                {13.802461, 0.286422, 0.776916},
            }),
        };
        Mesh mesh = Mesh::loft(bottom, top, true);

        MINI_CHECK(mesh.is_valid());
        MINI_CHECK(mesh.is_closed());

        Mesh mesh_no_cap = Mesh::loft(bottom, top, false);
        MINI_CHECK(mesh_no_cap.is_valid());
        MINI_CHECK(!mesh_no_cap.is_closed());
    }

    MINI_TEST("Mesh", "Loft concave with holes and collinear") {
        // uncomment #include "mesh.h"

        std::vector<Polyline> annen_bot = {
            Polyline({
                {2142.008, -530.170, 1172.487},
                {2142.008, -530.170, -318.768},
                {2142.008, -318.102, -318.768},
                {2142.008, -347.792, -414.110},
                {2142.008, -106.034, -414.110},
                {2142.008, -135.724, -318.768},
                {2142.008,  106.034, -318.768},
                {2142.008,   76.344, -414.110},
                {2142.008,  318.102, -414.110},
                {2142.008,  288.412, -318.768},
                {2142.008,  530.170, -318.768},
                {2142.008,  530.170, 1172.487},
                {2142.008, -530.170, 1172.487},
            }),
            Polyline({
                {2142.008, 97.448,  841.097},
                {2142.008,  0.000,  841.097},
                {2142.008,  0.000, 1006.792},
                {2142.008, 97.448, 1006.792},
                {2142.008, 97.448,  841.097},
            }),
            Polyline({
                {2142.008, 97.448, 178.317},
                {2142.008,  0.000, 178.317},
                {2142.008,  0.000, 344.012},
                {2142.008, 97.448, 344.012},
                {2142.008, 97.448, 178.317},
            }),
        };
        std::vector<Polyline> annen_top = {
            Polyline({
                {2223.416, -530.170, 1172.487},
                {2223.416, -530.170, -269.141},
                {2223.416, -318.102, -269.141},
                {2223.416, -347.792, -364.483},
                {2223.416, -106.034, -364.483},
                {2223.416, -135.724, -269.141},
                {2223.416,  106.034, -269.141},
                {2223.416,   76.344, -364.483},
                {2223.416,  318.102, -364.483},
                {2223.416,  288.412, -269.141},
                {2223.416,  530.170, -269.141},
                {2223.416,  530.170, 1172.487},
                {2223.416, -530.170, 1172.487},
            }),
            Polyline({
                {2223.416, 97.448,  841.097},
                {2223.416,  0.000,  841.097},
                {2223.416,  0.000, 1006.792},
                {2223.416, 97.448, 1006.792},
                {2223.416, 97.448,  841.097},
            }),
            Polyline({
                {2223.416, 97.448, 178.317},
                {2223.416,  0.000, 178.317},
                {2223.416,  0.000, 344.012},
                {2223.416, 97.448, 344.012},
                {2223.416, 97.448, 178.317},
            }),
        };
        Mesh annen = Mesh::loft(annen_bot, annen_top, true);
        MINI_CHECK(annen.is_valid());
        MINI_CHECK(annen.is_closed());
        MINI_CHECK(annen.vertex.size() == 40);
        MINI_CHECK(annen.face.size() == 22);

        std::vector<Polyline> col_bot = {
            Polyline({
                { 0, 0, 0},
                { 4, 0, 0},
                { 7, 0, 0},
                {12, 0, 0},
                {12, 5, 0},
                { 0, 5, 0},
                { 0, 0, 0},
            }),
        };
        std::vector<Polyline> col_top = {
            Polyline({
                { 0, 0, 1.5},
                { 4, 0, 1.5},
                { 7, 0, 1.5},
                {12, 0, 1.5},
                {12, 5, 1.5},
                { 0, 5, 1.5},
                { 0, 0, 1.5},
            }),
        };
        Mesh colmesh = Mesh::loft(col_bot, col_top, true);
        MINI_CHECK(colmesh.is_valid());
        MINI_CHECK(colmesh.is_closed());
        MINI_CHECK(colmesh.vertex.size() == 12);
        MINI_CHECK(colmesh.face.size() == 8);
    }

    MINI_TEST("Mesh", "From Polygon With Holes Many") {
        // uncomment #include "mesh.h"

        std::vector<std::vector<std::vector<Point>>> inputs;
        for (int i = 0; i < 4; ++i) {
            double x = i * 7.0;
            inputs.push_back({
                {
                    {x, 0, 0},
                    {x+5, 0, 0},
                    {x+5, 5, 0},
                    {x, 5, 0},
                },
                {
                    {x+1, 1, 0},
                    {x+4, 1, 0},
                    {x+4, 4, 0},
                    {x+1, 4, 0},
                },
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
                {x, 0, 0},
                {x+1, 0, 0},
                {x+1, 1, 0},
                {x, 1, 0},
                {x, 0, 0}});
            Polyline t(std::vector<Point>{
                {x, 0, 1+i*0.5},
                {x+1, 0, 1+i*0.5},
                {x+1, 1, 1+i*0.5},
                {x, 1, 1+i*0.5},
                {x, 0, 1+i*0.5}});
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
                {250, -250, 500},
                {250, 250, 500},
                {-250, 250, 500},
                {-250, -250, 500},
                {250, -250, 500},
            },
            {
                {-250, 500, 250},
                {-250, 250, 500},
                {250, 250, 500},
                {250, 500, 250},
                {-250, 500, 250},
            },
            {
                {250, -250, 500},
                {500, -250, 250},
                {500, 250, 250},
                {250, 250, 500},
                {250, -250, 500},
            },
            {
                {250, 500, 250},
                {250, 250, 500},
                {500, 250, 250},
                {250, 500, 250},
            },
            {
                {-250, 500, 250},
                {250, 500, 250},
                {250, 500, -250},
                {-250, 500, -250},
                {-250, 500, 250},
            },
            {
                {250, 500, 250},
                {500, 250, 250},
                {500, 250, -250},
                {250, 500, -250},
                {250, 500, 250},
            },
            {
                {500, -250, 250},
                {500, -250, -250},
                {500, 250, -250},
                {500, 250, 250},
                {500, -250, 250},
            },
        };
        std::vector<std::vector<Point>> bot7 = {
            {
                {270.710678, -250, 550},
                {270.710678, 265.891862, 550},
                {265.891862, 270.710678, 550},
                {-250, 270.710678, 550},
                {-250, -250, 550},
                {270.710678, -250, 550},
            },
            {
                {270.710678, -250, 550},
                {550, -250, 270.710678},
                {550, 265.891862, 270.710678},
                {270.710678, 265.891862, 550},
                {270.710678, -250, 550},
            },
            {
                {-250, 550, 270.710678},
                {-250, 270.710678, 550},
                {265.891862, 270.710678, 550},
                {265.891862, 550, 270.710678},
                {-250, 550, 270.710678},
            },
            {
                {265.891862, 550, 270.710678},
                {265.891862, 270.710678, 550},
                {270.710678, 265.891862, 550},
                {550, 265.891862, 270.710678},
                {550, 270.710678, 265.891862},
                {270.710678, 550, 265.891862},
                {265.891862, 550, 270.710678},
            },
            {
                {-250, 550, 270.710678},
                {265.891862, 550, 270.710678},
                {270.710678, 550, 265.891862},
                {270.710678, 550, -250},
                {-250, 550, -250},
                {-250, 550, 270.710678},
            },
            {
                {270.710678, 550, 265.891862},
                {550, 270.710678, 265.891862},
                {550, 270.710678, -250},
                {270.710678, 550, -250},
                {270.710678, 550, 265.891862},
            },
            {
                {550, -250, 270.710678},
                {550, -250, -250},
                {550, 270.710678, -250},
                {550, 270.710678, 265.891862},
                {550, 265.891862, 270.710678},
                {550, -250, 270.710678},
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
    }

    MINI_TEST("Mesh", "Create Dodecahedron") {
        // uncomment #include "mesh.h"

        Mesh m = Mesh::create_dodecahedron(2.0);

        MINI_CHECK(m.is_valid());
        MINI_CHECK(m.number_of_vertices() == 20);
        MINI_CHECK(m.number_of_faces() == 12);
    }

    MINI_TEST("Mesh", "Vertex and Face Operations") {
        // uncomment #include "mesh.h"

        double hx = 0.5, hy = 0.5, hz = 0.5;
        std::vector<Point> verts = {
            Point(-hx, -hy, -hz),
            Point( hx, -hy, -hz),
            Point( hx,  hy, -hz),
            Point(-hx,  hy, -hz),
            Point(-hx, -hy,  hz),
            Point( hx, -hy,  hz),
            Point( hx,  hy,  hz),
            Point(-hx,  hy,  hz),
        };
        std::vector<std::vector<size_t>> faces = {
            {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 4, 7, 3}, {1, 2, 6, 5},
        };

        Mesh mesh = Mesh();

        
        for (const auto& v : verts) 
            mesh.add_vertex(v);
    
        for (const auto& f : faces) 
            mesh.add_face(f);

        MINI_CHECK(!mesh.add_face({0, 1}, std::nullopt).has_value());
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

        std::vector<Point> pts = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0,1,2,3}, {1,4,2}});

        auto v = mesh.vertices();
        auto f = mesh.faces();

        // edge edges
        // edge 1 - 2, edges: 1-0, 1-4, 2-3, 2-4
        std::optional<std::vector<std::pair<size_t, size_t>>> ee = mesh.edge_edges(1, 2);
        if (ee){

            size_t u0 = (*ee)[0].first;
            size_t v0 = (*ee)[0].second;
            Line l0 = mesh.edge_line(u0, v0).value();
            Point mid0 = l0.center();
            mid0.name = "e" + std::to_string(u0) + "-" + std::to_string(v0);

            size_t u1 = (*ee)[1].first;
            size_t v1 = (*ee)[1].second;
            Line l1 = mesh.edge_line(u1, v1).value();
            Point mid1 = l1.center();
            mid1.name = "e" + std::to_string(u1) + "-" + std::to_string(v1);

            size_t u2 = (*ee)[2].first;
            size_t v2 = (*ee)[2].second;
            Line l2 = mesh.edge_line(u2, v2).value();
            Point mid2 = l2.center();
            mid2.name = "e" + std::to_string(u2) + "-" + std::to_string(v2);

            size_t u3 = (*ee)[3].first;
            size_t v3 = (*ee)[3].second;
            Line l3 = mesh.edge_line(u3, v3).value();
            Point mid3 = l3.center();
            mid3.name = "e" + std::to_string(u3) + "-" + std::to_string(v3);

            std::set<std::pair<size_t, size_t>> ee_set(ee->begin(), ee->end());

            MINI_CHECK(ee->size() == 4);
            MINI_CHECK(((*ee)[0] == std::make_pair(1ul, 0ul)) || ((*ee)[0] == std::make_pair(0ul, 1ul)));
            MINI_CHECK(((*ee)[1] == std::make_pair(1ul, 4ul)) || ((*ee)[1] == std::make_pair(4ul, 1ul)));
            MINI_CHECK(((*ee)[2] == std::make_pair(2ul, 3ul)) || ((*ee)[2] == std::make_pair(3ul, 2ul)));
            MINI_CHECK(((*ee)[3] == std::make_pair(2ul, 4ul)) || ((*ee)[3] == std::make_pair(4ul, 2ul)));  
        }

        // edge faces
        // edge 1-2, faces: 0, 1
        std::optional<std::vector<size_t>> ef = mesh.edge_faces(1, 2);
        if (ef) {
            size_t ef0 = (*ef)[0];
            size_t ef1 = (*ef)[1];
            Point efp0 = *mesh.face_centroid(ef0);
            efp0.name = "f" + std::to_string(ef0);
            Point efp1 = *mesh.face_centroid(ef1);
            efp1.name = "f" + std::to_string(ef1);

            MINI_CHECK(ef->size() == 2);
            MINI_CHECK(ef0 == 0 && ef1 == 1);
        }

        // face_edges
        // face 0, edges: 0-1, 1-2, 2-3, 3-0
        std::optional<std::vector<std::pair<size_t, size_t>>> fe = mesh.face_edges(f[0]);
        if (fe) {
            Line l0 = mesh.edge_line((*fe)[0].first, (*fe)[0].second).value();
            Line l1 = mesh.edge_line((*fe)[1].first, (*fe)[1].second).value();
            Line l2 = mesh.edge_line((*fe)[2].first, (*fe)[2].second).value();
            Line l3 = mesh.edge_line((*fe)[3].first, (*fe)[3].second).value();
            Point lmid0 = l0.center();
            lmid0.name = "e" + std::to_string((*fe)[0].first) + "-" + std::to_string((*fe)[0].second);
            Point lmid1 = l1.center();
            lmid1.name = "e" + std::to_string((*fe)[1].first) + "-" + std::to_string((*fe)[1].second);
            Point lmid2 = l2.center();
            lmid2.name = "e" + std::to_string((*fe)[2].first) + "-" + std::to_string((*fe)[2].second);
            Point lmid3 = l3.center();
            lmid3.name = "e" + std::to_string((*fe)[3].first) + "-" + std::to_string((*fe)[3].second);

            MINI_CHECK(fe->size() == 4);
            MINI_CHECK(((*fe)[0] == std::make_pair(0ul, 1ul)));
            MINI_CHECK(((*fe)[1] == std::make_pair(1ul, 2ul)));
            MINI_CHECK(((*fe)[2] == std::make_pair(2ul, 3ul)));
            MINI_CHECK(((*fe)[3] == std::make_pair(3ul, 0ul)));
        }

        // face_faces
        // face 0, adjacent faces: 1
        std::optional<std::vector<size_t>> ff = mesh.face_faces(f[0]);
        if (ff) {
            size_t ff0 = (*ff)[0];
            Point ffp = *mesh.face_centroid(ff0);
            ffp.name = "f" + std::to_string(ff0);

            MINI_CHECK(ff->size() == 1);
            MINI_CHECK(ff0 == 1);
        }

        // face points
        std::optional<std::vector<Point>> points = mesh.face_points(f[0]);
        if (points) {
            size_t pointcount = (*points).size();

            MINI_CHECK(pointcount == 4);
        }

        // face polyline
        std::optional<Polyline> pl = mesh.face_polyline(f[0]);
        if (pl) {
            size_t pointcount = (*pl).point_count();

            MINI_CHECK(pointcount == 4);
        }

        // face_vertices
        // face 0 vertices: 0, 1, 2, 3
        std::optional<std::vector<size_t>> fv = mesh.face_vertices(f[0]);
        if(fv.has_value()) {
            size_t fv0 = (*fv)[0];
            size_t fv1 = (*fv)[1];
            size_t fv2 = (*fv)[2];
            size_t fv3 = (*fv)[3];
            Point p0 = *mesh.vertex_point(fv0);
            p0.name = std::to_string(fv0);
            Point p1 = *mesh.vertex_point(fv1);
            p1.name = std::to_string(fv1);
            Point p2 = *mesh.vertex_point(fv2);
            p2.name = std::to_string(fv2);
            Point p3 = *mesh.vertex_point(fv3);
            p3.name = std::to_string(fv3);

            MINI_CHECK(fv0 == 0);
            MINI_CHECK(fv1 == 1);
            MINI_CHECK(fv2 == 2);
            MINI_CHECK(fv3 == 3);
            MINI_CHECK(fv->size() == 4);
        }
    
        // vertex_edges
        // vertex 3, edges 1-0, 1-2, 1-4
        std::optional<std::vector<std::pair<size_t, size_t>>> ve = mesh.vertex_edges(v[1]);
        if (ve) {
            Point vp = *mesh.vertex_point(v[1]);
            vp.name = "v" + std::to_string(v[1]);

            Line l0 = mesh.edge_line((*ve)[0].first, (*ve)[0].second).value();
            Line l1 = mesh.edge_line((*ve)[1].first, (*ve)[1].second).value();
            Line l2 = mesh.edge_line((*ve)[2].first, (*ve)[2].second).value();
            Point lmid0 = l0.center();
            lmid0.name = "e" + std::to_string((*ve)[0].first) + "-" + std::to_string((*ve)[0].second);
            Point lmid1 = l1.center();
            lmid1.name = "e" + std::to_string((*ve)[1].first) + "-" + std::to_string((*ve)[1].second);
            Point lmid2 = l2.center();
            lmid2.name = "e" + std::to_string((*ve)[2].first) + "-" + std::to_string((*ve)[2].second);

            MINI_CHECK(((*ve)[0] == std::make_pair(1ul, 0ul)));
            MINI_CHECK(((*ve)[1] == std::make_pair(1ul, 2ul)));
            MINI_CHECK(((*ve)[2] == std::make_pair(1ul, 4ul)));
            MINI_CHECK((*ve).size() == 3);
        }

        // vertex_faces
        std::optional<std::vector<size_t>> vf = mesh.vertex_faces(v[1]);
        // vertex 3, faces 0, 1
        if (vf) {

            Point vp = *mesh.vertex_point(v[1]);
            vp.name = "v" + std::to_string(v[1]);

            Point fp0 = *mesh.face_centroid((*vf)[0]);
            fp0.name = "f" + std::to_string((*vf)[0]);
            Point fp1 = *mesh.face_centroid((*vf)[1]);
            fp1.name = "f" + std::to_string((*vf)[1]);

            MINI_CHECK(vf->size() == 2);
            MINI_CHECK((*vf)[0] == 0);
            MINI_CHECK((*vf)[1] == 1);
        }

        // vertex_vertices
        // vertex 1, neighbors 0, 2, 4
        std::optional<std::vector<size_t>> vn = mesh.vertex_vertices(v[1]);
        if (vn) {
            Point p0 = *mesh.vertex_point(v[1]);
            p0.name = "main" + std::to_string(v[1]); 

            Point np0 = *mesh.vertex_point((*vn)[0]);
            np0.name = std::to_string((*vn)[0]);
            Point np1 = *mesh.vertex_point((*vn)[1]);
            np1.name = std::to_string((*vn)[1]);
            Point np2 = *mesh.vertex_point((*vn)[2]);
            np2.name = std::to_string((*vn)[2]);

            MINI_CHECK((*vn)[0] == 0);
            MINI_CHECK((*vn)[1] == 2);
            MINI_CHECK((*vn)[2] == 4);
            MINI_CHECK(vn->size() == 3);
        }

    }


    MINI_TEST("Mesh", "Geometric Properties") {
        // uncomment #include "mesh.h"

        Mesh mesh = Mesh::create_dodecahedron(1.5);

        // area
        double area = mesh.area();

        MINI_CHECK(TOLERANCE.is_close(area, 46.4528898159021));

        // centroid
        Point centroid = mesh.centroid();
        MINI_CHECK(TOLERANCE.is_point_close(centroid, Point(0.0, 0.0, 0.0)));

        // dihedral angle
        auto [angles, arcs, points] = mesh.dihedral_angles(0.3);

        for (const auto& [edge, angle] : angles) {
            double angle_in_degrees = angle;
            MINI_CHECK(TOLERANCE.is_close(angle_in_degrees, 116.565051177078));
        }

        // face area
        for (size_t f : mesh.faces()) {
            auto face_area = mesh.face_area(f);
            MINI_CHECK(face_area.has_value());
            MINI_CHECK(TOLERANCE.is_close(*face_area, 3.87107415132518));
        }
        
        // face centroid
        std::vector<Point> centroids;
        for (size_t f : mesh.faces()) 
            centroids.emplace_back(*mesh.face_centroid(f));

        MINI_CHECK(TOLERANCE.is_point_close(centroids[0],  Point( 0.878115294937453,  0.0,                1.420820393249937)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[1],  Point( 1.420820393249937,  0.878115294937453, 0.0              )));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[2],  Point( 0.0,                1.420820393249937,  0.878115294937453)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[3],  Point( 0.878115294937453,  0.0,               -1.420820393249937)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[4],  Point( 0.0,                1.420820393249937, -0.878115294937453)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[5],  Point( 0.0,               -1.420820393249937,  0.878115294937453)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[6],  Point( 1.420820393249937, -0.878115294937453, 0.0              )));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[7],  Point( 0.0,               -1.420820393249937, -0.878115294937453)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[8],  Point(-1.420820393249937,  0.878115294937453, 0.0              )));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[9],  Point(-0.878115294937453,  0.0,                1.420820393249937)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[10], Point(-0.878115294937453,  0.0,               -1.420820393249937)));
        MINI_CHECK(TOLERANCE.is_point_close(centroids[11], Point(-1.420820393249937, -0.878115294937453, 0.0              )));

        // face normal / s
        std::map<size_t, Vector> face_normals = mesh.face_normals();
        for (size_t f : mesh.faces()) {
            auto fn = mesh.face_normal(f);
            MINI_CHECK(fn.has_value());
            MINI_CHECK(TOLERANCE.is_vector_close(face_normals.at(f), *fn));
        }

        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[0],  Vector( 0.5257311121191336,  0.0,                 0.8506508083520400)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[1],  Vector( 0.8506508083520400,  0.5257311121191336,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[2],  Vector( 0.0,                 0.8506508083520400,  0.5257311121191336)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[3],  Vector( 0.5257311121191336,  0.0,                -0.8506508083520400)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[4],  Vector( 0.0,                 0.8506508083520400, -0.5257311121191336)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[5],  Vector( 0.0,                -0.8506508083520400,  0.5257311121191336)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[6],  Vector( 0.8506508083520400, -0.5257311121191336,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[7],  Vector( 0.0,                -0.8506508083520400, -0.5257311121191336)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[8],  Vector(-0.8506508083520400,  0.5257311121191336,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[9],  Vector(-0.5257311121191336,  0.0,                 0.8506508083520400)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[10], Vector(-0.5257311121191336,  0.0,                -0.8506508083520400)));
        MINI_CHECK(TOLERANCE.is_vector_close(face_normals[11], Vector(-0.8506508083520400, -0.5257311121191336,  0.0               )));

        // vertex angle in face
        for (const size_t f : mesh.faces()) {
            auto fv = *mesh.face_vertices(f);
            for (const size_t v : fv) {
                auto angle = mesh.vertex_angle_in_face(v, f);
                MINI_CHECK(angle.has_value());
                MINI_CHECK(TOLERANCE.is_close(*angle, 1.8849555921538759));
            }
        }

        // vertex normal / s
        std::map<size_t, Vector> vertex_normals = mesh.vertex_normals();
        for (const size_t v : mesh.vertices()){
            auto vn = mesh.vertex_normal(v);
            MINI_CHECK(vn.has_value());
            MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals.at(v), *vn));
        }

        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[0],  Vector( 0.5773502691896258,  0.5773502691896258,  0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[1],  Vector( 0.0,                 0.3568220897730899,  0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[2],  Vector( 0.0,                -0.3568220897730899,  0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[3],  Vector( 0.5773502691896257, -0.5773502691896258,  0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[4],  Vector( 0.9341723589627158,  0.0,                 0.3568220897730899)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[5],  Vector( 0.9341723589627158,  0.0,                -0.3568220897730899)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[6],  Vector( 0.5773502691896258,  0.5773502691896257, -0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[7],  Vector( 0.3568220897730899,  0.9341723589627158,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[8],  Vector(-0.3568220897730899,  0.9341723589627157,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[9],  Vector(-0.5773502691896258,  0.5773502691896258,  0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[10], Vector( 0.5773502691896258, -0.5773502691896258, -0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[11], Vector( 0.0,                -0.3568220897730899, -0.9341723589627157)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[12], Vector( 0.0,                 0.3568220897730899, -0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[13], Vector(-0.5773502691896257,  0.5773502691896258, -0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[14], Vector(-0.5773502691896258, -0.5773502691896257,  0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[15], Vector(-0.3568220897730899, -0.9341723589627157,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[16], Vector( 0.3568220897730899, -0.9341723589627158,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[17], Vector(-0.5773502691896258, -0.5773502691896258, -0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[18], Vector(-0.9341723589627157,  0.0,                -0.3568220897730899)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals[19], Vector(-0.9341723589627158,  0.0,                 0.3568220897730899)));

        // vertex normal weighted / s
        std::map<size_t, Vector> vertex_normals_weighted = mesh.vertex_normals_weighted(NormalWeighting::Angle);
        for (const size_t v : mesh.vertices()){
            auto vnw = mesh.vertex_normal_weighted(v, NormalWeighting::Angle);
            MINI_CHECK(vnw.has_value());
            MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted.at(v), *vnw));
        }

        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[0],  Vector( 0.5773502691896257,  0.5773502691896257,  0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[1],  Vector( 0.0,                 0.3568220897730899,  0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[2],  Vector( 0.0,                -0.3568220897730899,  0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[3],  Vector( 0.5773502691896257, -0.5773502691896257,  0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[4],  Vector( 0.9341723589627158,  0.0,                 0.3568220897730899)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[5],  Vector( 0.9341723589627158,  0.0,                -0.3568220897730899)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[6],  Vector( 0.5773502691896258,  0.5773502691896257, -0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[7],  Vector( 0.3568220897730899,  0.9341723589627158,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[8],  Vector(-0.3568220897730899,  0.9341723589627158,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[9],  Vector(-0.5773502691896257,  0.5773502691896258,  0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[10], Vector( 0.5773502691896257, -0.5773502691896258, -0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[11], Vector( 0.0,                -0.3568220897730899, -0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[12], Vector( 0.0,                 0.3568220897730899, -0.9341723589627158)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[13], Vector(-0.5773502691896257,  0.5773502691896257, -0.5773502691896258)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[14], Vector(-0.5773502691896258, -0.5773502691896257,  0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[15], Vector(-0.3568220897730900, -0.9341723589627158,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[16], Vector( 0.3568220897730899, -0.9341723589627158,  0.0               )));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[17], Vector(-0.5773502691896257, -0.5773502691896257, -0.5773502691896257)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[18], Vector(-0.9341723589627158,  0.0,                -0.3568220897730899)));
        MINI_CHECK(TOLERANCE.is_vector_close(vertex_normals_weighted[19], Vector(-0.9341723589627158,  0.0,                 0.3568220897730899)));


        // volume
        double volume = mesh.volume();
        MINI_CHECK(TOLERANCE.is_close(volume, 25.8630264921081));

    }

    MINI_TEST("Mesh", "Transformation") {
        // uncomment #include "mesh.h"

        std::vector<Point> pts = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
        };
        Mesh mesh = Mesh::from_vertices_and_faces(pts, {{0,1,2}});
        size_t v0 = mesh.vertices()[0];

        // transform() — apply stored xform in-place; xform field unchanged
        Mesh mesh1 = mesh;
        mesh1.xform = Xform::translation(0.0, 0.0, 1.0);
        mesh1.transform();

        MINI_CHECK(!mesh1.xform.is_identity());
        MINI_CHECK((*mesh1.vertex_point(v0))[2] == 1.0);

        // transform(const Xform&) — apply given xform in-place; stored xform unchanged
        Mesh mesh2 = mesh;
        Xform x = Xform::translation(0.0, 0.0, 1.0);
        mesh2.transform(x);
        MINI_CHECK(mesh2.xform.is_identity());
        MINI_CHECK((*mesh2.vertex_point(v0))[2] == 1.0);

        // transformed() — copy with stored xform applied
        Mesh mesh3 = mesh;
        mesh3.xform = Xform::translation(0.0, 0.0, 10.0);
        Mesh mesh3t = mesh3.transformed();
        MINI_CHECK(!mesh3t.xform.is_identity());
        MINI_CHECK((*mesh3t.vertex_point(v0))[2] == 10.0);

        // transformed(const Xform&) — copy with given xform applied
        Mesh mesh4 = mesh;
        x = Xform::translation(0.0, 0.0, 10.0);
        Mesh mesh4t = mesh4.transformed(x);
        MINI_CHECK(mesh4t.xform.is_identity());
        MINI_CHECK((*mesh4t.vertex_point(v0))[2] == 10.0);
    }

    MINI_TEST("Mesh", "Json Roundtrip") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh";
        mesh.xform = Xform::translation(1.0, 2.0, 3.0);

        // JSON object
        nlohmann::ordered_json json = mesh.jsondump();
        Mesh loaded_json = Mesh::jsonload(json);

        // String
        std::string json_string = mesh.file_json_dumps();
        Mesh loaded_string = Mesh::file_json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh.json").string();
        mesh.file_json_dump(filename);
        Mesh loaded_file = Mesh::file_json_load(filename);

        MINI_CHECK(loaded_json == mesh);
        MINI_CHECK(loaded_string == mesh);
        MINI_CHECK(loaded_file == mesh);

        // Triangulation roundtrip
        std::vector<std::vector<Point>> polys = {{
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        }};
        Mesh pmesh = Mesh::from_polylines(polys, std::nullopt);
        Mesh loaded_tri = Mesh::jsonload(pmesh.jsondump());
        size_t fk = pmesh.get_triangulation().begin()->first;
        MINI_CHECK(!loaded_tri.get_triangulation().empty());
        MINI_CHECK(loaded_tri.get_triangulation().count(fk) > 0);

        // Face holes roundtrip
        Mesh hmesh = Mesh::from_polygon_with_holes(
            {
                {
                    Point(0, 0, 0),
                    Point(4, 0, 0),
                    Point(4, 4, 0),
                    Point(0, 4, 0),
                },
                {
                    Point(1, 1, 0),
                    Point(3, 1, 0),
                    Point(3, 3, 0),
                    Point(1, 3, 0),
                },
            }, true);
        Mesh loaded_holes = Mesh::jsonload(hmesh.jsondump());
        size_t hfk = hmesh.get_face_holes().begin()->first;
        MINI_CHECK(!loaded_holes.get_face_holes().empty());
        MINI_CHECK(loaded_holes.get_face_holes().at(hfk) == hmesh.get_face_holes().at(hfk));
    }

    MINI_TEST("Mesh", "Protobuf Roundtrip") {
        // uncomment #include "mesh.h"
        // uncomment #include "point.h"
        // uncomment #include <filesystem>

        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.name = "test_mesh_proto";
        mesh.xform = Xform::translation(1.0, 2.0, 3.0);

        // String
        std::string proto_string = mesh.pb_dumps();
        Mesh loaded_string = Mesh::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh.bin").string();
        mesh.pb_dump(filename);
        Mesh loaded_file = Mesh::pb_load(filename);

        MINI_CHECK(loaded_string == mesh);
        MINI_CHECK(loaded_file == mesh);

        // Triangulation roundtrip
        std::vector<std::vector<Point>> polys = {{
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        }};
        Mesh pmesh = Mesh::from_polylines(polys, std::nullopt);
        Mesh loaded_tri = Mesh::pb_loads(pmesh.pb_dumps());
        size_t fk = pmesh.get_triangulation().begin()->first;
        MINI_CHECK(!loaded_tri.get_triangulation().empty());
        MINI_CHECK(loaded_tri.get_triangulation().count(fk) > 0);

        // Face holes roundtrip
        Mesh hmesh = Mesh::from_polygon_with_holes(
            {
                {
                    Point(0, 0, 0),
                    Point(4, 0, 0),
                    Point(4, 4, 0),
                    Point(0, 4, 0),
                },
                {
                    Point(1, 1, 0),
                    Point(3, 1, 0),
                    Point(3, 3, 0),
                    Point(1, 3, 0),
                },
            }, true);
        Mesh loaded_holes = Mesh::pb_loads(hmesh.pb_dumps());
        size_t hfk = hmesh.get_face_holes().begin()->first;
        MINI_CHECK(!loaded_holes.get_face_holes().empty());
        MINI_CHECK(loaded_holes.get_face_holes().at(hfk) == hmesh.get_face_holes().at(hfk));
    }

    MINI_TEST("Mesh", "Edges") {
        // uncomment #include "mesh.h"

        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        size_t v0 = mesh.vertices()[0];
        size_t v1 = mesh.vertices()[1];
        auto edges = mesh.edges();

        MINI_CHECK(edges.size() == 12);
        MINI_CHECK(edges[0] == std::make_pair(v0, v1));
    }

    MINI_TEST("Mesh", "Loft plate_failing 15-vert outer + 4 holes") {
        std::vector<Polyline> bot_polylines = {
            Polyline(std::vector<Point>{
                { 734.392021, -1906.59468,  1101.588031},
                { 632.396858, -1838.597905,  948.595287},
                { 624.453132, -1769.270846,  984.70313 },
                { 113.775484, -1428.81908,   218.686657},
                { 121.719209, -1498.146139,  182.578814},
                {  15.607979, -1427.40532,    23.411969},
                {   0.0,      -1441.0,        -18.0     },
                {   0.0,      -1893.0,       -357.0     },
                {  13.416408, -1917.0,       -348.167184},
                { 104.290124, -1917.0,       -166.419752},
                { 118.441096, -1964.169906,  -173.495238},
                { 664.077103, -1964.169906,   917.776777},
                { 649.926131, -1917.0,        924.852263},
                { 736.583592, -1917.0,       1098.167184},
                { 734.392021, -1906.59468,  1101.588031},
            }),
            Polyline(std::vector<Point>{
                { 322.544527, -1917.0,       270.089054},
                { 213.417326, -1917.0,        51.834651},
                { 199.266354, -1869.830094,   58.910137},
                { 308.393555, -1869.830094,  277.16454 },
                { 322.544527, -1917.0,       270.089054},
            }),
            Polyline(std::vector<Point>{
                { 540.79893,  -1917.0,       706.59786 },
                { 431.671728, -1917.0,       488.343457},
                { 417.520757, -1869.830094,  495.418943},
                { 526.647958, -1869.830094,  713.673346},
                { 540.79893,  -1917.0,       706.59786 },
            }),
            Polyline(std::vector<Point>{
                { 424.153936, -1667.753669,  660.242619},
                { 526.289465, -1735.844022,  813.445914},
                { 530.261328, -1770.507552,  795.391992},
                { 428.125798, -1702.417199,  642.188697},
                { 424.153936, -1667.753669,  660.242619},
            }),
            Polyline(std::vector<Point>{
                { 219.882876, -1531.572963,  353.83603 },
                { 322.018406, -1599.663316,  507.039325},
                { 325.990269, -1634.326846,  488.985403},
                { 223.854739, -1566.236493,  335.782108},
                { 219.882876, -1531.572963,  353.83603 },
            }),
        };
        std::vector<Polyline> top_polylines = {
            Polyline(std::vector<Point>{
                { 711.660594, -1906.59468,  1126.880036},
                { 605.549364, -1835.85386,   967.713191},
                { 601.577501, -1801.190331,  985.767113},
                {  90.899853, -1460.738565,  219.75064 },
                {  94.871715, -1495.402095,  201.696718},
                {  -9.83197,  -1425.599638,   44.641191},
                { -25.439949, -1439.194318,    3.229221},
                { -25.439949, -1893.0,       -337.12504 },
                { -12.023541, -1917.0,       -328.292224},
                {  75.988181, -1917.0,       -152.26878 },
                { 104.290124, -2011.339811,  -166.419752},
                { 649.926131, -2011.339811,   924.852263},
                { 621.624188, -1917.0,        939.003234},
                { 713.852165, -1917.0,       1123.459189},
                { 711.660594, -1906.59468,  1126.880036},
            }),
            Polyline(std::vector<Point>{
                { 308.393555, -1964.169906,  277.16454 },
                { 199.266354, -1964.169906,   58.910137},
                { 185.115382, -1917.0,        65.985623},
                { 294.242584, -1917.0,       284.240026},
                { 308.393555, -1964.169906,  277.16454 },
            }),
            Polyline(std::vector<Point>{
                { 526.647958, -1964.169906,  713.673346},
                { 417.520757, -1964.169906,  495.418943},
                { 403.369785, -1917.0,       502.494429},
                { 512.496987, -1917.0,       720.748832},
                { 526.647958, -1964.169906,  713.673346},
            }),
            Polyline(std::vector<Point>{
                { 401.278305, -1699.673154,  661.306602},
                { 503.413834, -1767.763507,  814.509897},
                { 507.385697, -1802.427037,  796.455975},
                { 405.250167, -1734.336684,  643.25268 },
                { 401.278305, -1699.673154,  661.306602},
            }),
            Polyline(std::vector<Point>{
                { 197.007245, -1563.492448,  354.900013},
                { 299.142775, -1631.582801,  508.103307},
                { 303.114638, -1666.246331,  490.049386},
                { 200.979108, -1598.155978,  336.846091},
                { 197.007245, -1563.492448,  354.900013},
            }),
        };
        Mesh m = Mesh::loft(bot_polylines, top_polylines, true, true);
        printf("Loft plate_failing: faces=%zu  vertices=%zu  valid=%d  closed=%d\n",
               m.faces().size(), m.vertices().size(), (int)m.is_valid(), (int)m.is_closed());
        MINI_CHECK(m.is_valid());

        // Write to session pb so it can be inspected in Rhino
        Session session;
        session.add_mesh(std::make_shared<Mesh>(m));
        std::string pb_path = "C:/Users/Petras/Desktop/plate_failing_loft.pb";
        session.pb_dump(pb_path);
        printf("Session written to: %s\n", pb_path.c_str());
    }

    MINI_TEST("Mesh", "Loft plate_v2 15-vert outer + 3 holes") {
        std::vector<Polyline> top_polylines = {
            Polyline(std::vector<Point>{
                { 734.392021,  -28.40532,  1101.588031},
                { 630.839301,  -97.440466,  946.258951},
                { 602.668732,  -21.881034,  974.757956},
                {  90.636822, -363.235641,  206.710092},
                { 118.807391, -438.795073,  178.211087},
                {  15.607979, -507.59468,    23.411969},
                {  21.213203, -518.0,        21.213203},
                {1478.786797, -518.0,      1478.786797},
                {1476.953362, -502.635574, 1488.476681},
                {1323.309106, -400.20607,  1411.654553},
                {1323.309106, -350.20607,  1449.154553},
                { 921.429178,  -82.286119, 1248.214589},
                { 921.429178, -132.286119, 1210.714589},
                { 773.046638,  -33.364426, 1136.523319},
                { 734.392021,  -28.40532,  1101.588031},
            }),
            Polyline(std::vector<Point>{
                {1055.389154, -196.592769, 1296.444577},
                {1189.34913,  -285.89942,  1363.424565},
                {1189.34913,  -310.89942,  1344.674565},
                {1055.389154, -221.592769, 1277.694577},
                {1055.389154, -196.592769, 1296.444577},
            }),
            Polyline(std::vector<Point>{
                { 411.941252, -196.202593,  653.289308},
                { 514.347634, -127.931671,  806.898881},
                { 528.432919, -165.711387,  792.649378},
                { 426.026537, -233.982309,  639.039805},
                { 411.941252, -196.202593,  653.289308},
            }),
            Polyline(std::vector<Point>{
                { 207.128489, -332.744435,  346.070162},
                { 309.53487,  -264.473514,  499.679735},
                { 323.620155, -302.25323,   485.430233},
                { 221.213773, -370.524151,  331.82066 },
                { 207.128489, -332.744435,  346.070162},
            }),
        };
        std::vector<Polyline> bot_polylines = {
            Polyline(std::vector<Point>{
                { 717.764591,  -24.335988, 1136.036032},
                { 607.106921,  -98.107768,  970.049526},
                { 593.021636,  -60.328052,  984.299029},
                {  80.989727, -401.682659,  216.251164},
                {  95.075011, -439.462375,  202.001662},
                { -28.206905, -521.650319,   17.078787},
                { -22.601681, -532.055639,   14.880022},
                {1489.346823, -532.055639, 1526.828525},
                {1487.513388, -516.691213, 1536.51841 },
                {1323.309106, -407.221692, 1454.416269},
                {1323.309106, -382.221692, 1473.166269},
                { 921.429178, -114.30174,  1272.226305},
                { 921.429178, -139.30174,  1253.476305},
                { 756.419209,  -29.295094, 1170.97132 },
                { 717.764591,  -24.335988, 1136.036032},
            }),
            Polyline(std::vector<Point>{
                {1055.389154, -228.608391, 1320.456293},
                {1189.34913,  -317.915041, 1387.436281},
                {1189.34913,  -342.915041, 1368.686281},
                {1055.389154, -253.608391, 1301.706293},
                {1055.389154, -228.608391, 1320.456293},
            }),
            Polyline(std::vector<Point>{
                { 402.294157, -234.649611,  662.830381},
                { 504.700539, -166.37869,   816.439954},
                { 518.785824, -204.158406,  802.190451},
                { 416.379442, -272.429327,  648.580878},
                { 402.294157, -234.649611,  662.830381},
            }),
            Polyline(std::vector<Point>{
                { 197.481393, -371.191453,  355.611235},
                { 299.887775, -302.920532,  509.220808},
                { 313.97306,  -340.700248,  494.971305},
                { 211.566678, -408.971169,  341.361733},
                { 197.481393, -371.191453,  355.611235},
            }),
        };
        Mesh m = Mesh::loft(top_polylines, bot_polylines, true, true);
        printf("Loft plate_v2: faces=%zu  vertices=%zu  valid=%d  closed=%d\n",
               m.faces().size(), m.vertices().size(), (int)m.is_valid(), (int)m.is_closed());
        MINI_CHECK(m.is_valid());

        Session session;
        session.add_mesh(std::make_shared<Mesh>(m));
        std::string pb_path = "C:/Users/Petras/Desktop/plate_v2_loft.pb";
        session.pb_dump(pb_path);
        printf("Session written to: %s\n", pb_path.c_str());
    }

    MINI_TEST("Mesh", "Loft plate_v3 15-vert outer + 3 holes") {
        std::vector<Polyline> top_polylines = {
            Polyline(std::vector<Point>{
                { 734.392021,  352.59468,  1101.588031},
                { 618.973111,  275.648741,  928.459666},
                { 618.973111,  369.988552,  999.214525},
                { 106.941201,   28.633945,  231.16666 },
                { 106.941201,  -65.705866,  160.411802},
                {  15.607979, -126.59468,    23.411969},
                {  21.213203, -137.0,        21.213203},
                {1478.786797, -137.0,      1478.786797},
                {1476.953362, -121.635574, 1488.476681},
                {1323.309106,  -19.20607,  1411.654553},
                {1323.309106,   30.79393,  1449.154553},
                { 921.429178,  298.713881, 1248.214589},
                { 921.429178,  248.713881, 1210.714589},
                { 773.046638,  347.635574, 1136.523319},
                { 734.392021,  352.59468,  1101.588031},
            }),
            Polyline(std::vector<Point>{
                {1055.389154,  184.407231, 1296.444577},
                {1189.34913,    95.10058,  1363.424565},
                {1189.34913,    70.10058,  1344.674565},
                {1055.389154,  159.407231, 1277.694577},
                {1055.389154,  184.407231, 1296.444577},
            }),
            Polyline(std::vector<Point>{
                { 414.160347,  186.276804,  656.61795 },
                { 516.566729,  254.547725,  810.227523},
                { 516.566729,  207.377819,  774.850093},
                { 414.160347,  139.106898,  621.240521},
                { 414.160347,  186.276804,  656.61795 },
            }),
            Polyline(std::vector<Point>{
                { 209.347583,   49.734961,  349.398804},
                { 311.753965,  118.005882,  503.008377},
                { 311.753965,   70.835977,  467.630948},
                { 209.347583,    2.565055,  314.021375},
                { 209.347583,   49.734961,  349.398804},
            }),
        };
        std::vector<Polyline> bot_polylines = {
            Polyline(std::vector<Point>{
                { 717.764591,  356.664012, 1136.036032},
                { 618.973111,  290.803025,  987.848811},
                { 618.973111,  337.972931, 1023.226241},
                { 106.941201,   -3.381676,  255.178376},
                { 106.941201,  -50.551581,  219.800947},
                { -28.206905, -140.650319,   17.078787},
                { -22.601681, -151.055639,   14.880022},
                {1489.346823, -151.055639, 1526.828525},
                {1487.513388, -135.691213, 1536.51841 },
                {1323.309106,  -26.221692, 1454.416269},
                {1323.309106,   -1.221692, 1473.166269},
                { 921.429178,  266.69826,  1272.226305},
                { 921.429178,  241.69826,  1253.476305},
                { 756.419209,  351.704906, 1170.97132 },
                { 717.764591,  356.664012, 1136.036032},
            }),
            Polyline(std::vector<Point>{
                {1055.389154,  152.391609, 1320.456293},
                {1189.34913,    63.084959, 1387.436281},
                {1189.34913,    38.084959, 1368.686281},
                {1055.389154,  127.391609, 1301.706293},
                {1055.389154,  152.391609, 1320.456293},
            }),
            Polyline(std::vector<Point>{
                { 414.160347,  154.261182,  680.629666},
                { 516.566729,  222.532104,  834.239239},
                { 516.566729,  175.362198,  798.861809},
                { 414.160347,  107.091277,  645.252236},
                { 414.160347,  154.261182,  680.629666},
            }),
            Polyline(std::vector<Point>{
                { 209.347583,   17.71934,   373.41052 },
                { 311.753965,   85.990261,  527.020093},
                { 311.753965,   38.820356,  491.642664},
                { 209.347583,  -29.450566,  338.033091},
                { 209.347583,   17.71934,   373.41052 },
            }),
        };
        Mesh m = Mesh::loft(top_polylines, bot_polylines, true, true);
        printf("Loft plate_v3: faces=%zu  vertices=%zu  valid=%d  closed=%d\n",
               m.faces().size(), m.vertices().size(), (int)m.is_valid(), (int)m.is_closed());
        MINI_CHECK(m.is_valid());

        Session session;
        session.add_mesh(std::make_shared<Mesh>(m));
        std::string pb_path = "C:/Users/Petras/Desktop/plate_v3_loft.pb";
        session.pb_dump(pb_path);
        printf("Session written to: %s\n", pb_path.c_str());
    }

    MINI_TEST("Mesh", "Vertex Neighbors") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto n0 = *mesh.vertex_neighbors(0);
        auto n0v = *mesh.vertex_vertices(0);
        std::sort(n0.begin(), n0.end());
        std::sort(n0v.begin(), n0v.end());
        MINI_CHECK(n0 == n0v);
        MINI_CHECK(n0.size() == 3);
    }

    MINI_TEST("Mesh", "Vertices On Boundary") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        MINI_CHECK(mesh.vertices_on_boundary().size() == 0);
        mesh.remove_face(mesh.faces()[0]);
        MINI_CHECK(mesh.vertices_on_boundary().size() == 4);
    }

    MINI_TEST("Mesh", "Edges On Boundary") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        MINI_CHECK(mesh.edges_on_boundary().size() == 0);
        mesh.remove_face(mesh.faces()[0]);
        MINI_CHECK(mesh.edges_on_boundary().size() == 4);
    }

    MINI_TEST("Mesh", "Faces On Boundary") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        MINI_CHECK(mesh.faces_on_boundary().size() == 0);
        mesh.remove_face(mesh.faces()[0]);
        MINI_CHECK(mesh.faces_on_boundary().size() == 4);
    }

    MINI_TEST("Mesh", "Halfedge Face") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto f = mesh.halfedge_face({0, 3});
        MINI_CHECK(f.has_value());
        MINI_CHECK(*f == 0);
        mesh.remove_face(0);
        MINI_CHECK(!mesh.halfedge_face({0, 3}).has_value());
    }

    MINI_TEST("Mesh", "Halfedge After Before") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto after = mesh.halfedge_after({0, 3});
        auto before = mesh.halfedge_before({0, 3});
        MINI_CHECK(after.has_value());
        MINI_CHECK((*after == std::make_pair<size_t, size_t>(3, 2)));
        MINI_CHECK(before.has_value());
        MINI_CHECK((*before == std::make_pair<size_t, size_t>(1, 0)));
    }

    MINI_TEST("Mesh", "Halfedge Loop") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto loop = mesh.halfedge_loop({0, 3});
        MINI_CHECK(loop.size() == 1);
        MINI_CHECK((loop[0] == std::make_pair<size_t, size_t>(0, 3)));
    }

    MINI_TEST("Mesh", "Halfedge Strip") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto strip = mesh.halfedge_strip({0, 3});
        MINI_CHECK(strip.size() == 5);
        MINI_CHECK((strip[0] == std::make_pair<size_t, size_t>(0, 3)));
        MINI_CHECK((strip[strip.size() - 1] == std::make_pair<size_t, size_t>(0, 3)));
    }

    MINI_TEST("Mesh", "Vertex Sample") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto s = mesh.vertex_sample(3, 42);
        MINI_CHECK(s.size() == 3);
        std::set<size_t> uniq(s.begin(), s.end());
        MINI_CHECK(uniq.size() == 3);
        auto s2 = mesh.vertex_sample(3, 42);
        MINI_CHECK(s == s2);
    }

    MINI_TEST("Mesh", "Edge Sample") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto s = mesh.edge_sample(2, 7);
        MINI_CHECK(s.size() == 2);
        auto s2 = mesh.edge_sample(2, 7);
        MINI_CHECK(s == s2);
    }

    MINI_TEST("Mesh", "Face Sample") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto s = mesh.face_sample(2, 11);
        MINI_CHECK(s.size() == 2);
        auto s2 = mesh.face_sample(2, 11);
        MINI_CHECK(s == s2);
    }

    MINI_TEST("Mesh", "Face Center") {
        Mesh mesh = Mesh::create_box(2.0, 2.0, 2.0);
        auto c = *mesh.face_center(0);
        auto cc = *mesh.face_centroid(0);
        MINI_CHECK(c == cc);
    }

    MINI_TEST("Mesh", "Face Polygon") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto poly = *mesh.face_polygon(0);
        auto pts = poly.get_points();
        MINI_CHECK(pts.size() == 5);
        MINI_CHECK(pts.front() == pts.back());
    }

    MINI_TEST("Mesh", "Flip Cycles") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        auto n0 = *mesh.face_normal(0);
        mesh.flip_cycles();
        auto n0b = *mesh.face_normal(0);
        MINI_CHECK(std::abs(n0[0] + n0b[0]) < Tolerance::ZERO_TOLERANCE);
        MINI_CHECK(std::abs(n0[1] + n0b[1]) < Tolerance::ZERO_TOLERANCE);
        MINI_CHECK(std::abs(n0[2] + n0b[2]) < Tolerance::ZERO_TOLERANCE);
    }

    MINI_TEST("Mesh", "Face Normal Unitized") {
        Mesh mesh = Mesh::create_box(2.0, 2.0, 2.0);
        auto nu = *mesh.face_normal_unitized(0, true);
        auto nn = *mesh.face_normal_unitized(0, false);
        MINI_CHECK(std::abs(nu.magnitude() - 1.0) < Tolerance::ZERO_TOLERANCE);
        MINI_CHECK(nn.magnitude() > 1.0);
    }

    MINI_TEST("Mesh", "Default Attributes") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes({{"is_support", 0.0}, {"load_z", 0.0}});
        mesh.update_default_face_attributes({{"stress", 0.0}});
        mesh.update_default_edge_attributes({{"weight", 1.0}});
        MINI_CHECK(mesh.default_vertex_attributes["is_support"] == 0.0);
        MINI_CHECK(mesh.default_vertex_attributes["load_z"] == 0.0);
        MINI_CHECK(mesh.default_face_attributes["stress"] == 0.0);
        MINI_CHECK(mesh.default_edge_attributes["weight"] == 1.0);
    }

    MINI_TEST("Mesh", "Vertex Attribute") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes({{"is_support", 0.0}});
        mesh.set_vertex_attribute(0, "is_support", 1.0);
        MINI_CHECK(*mesh.vertex_attribute(0, "is_support") == 1.0);
        MINI_CHECK(*mesh.vertex_attribute(1, "is_support") == 0.0);
    }

    MINI_TEST("Mesh", "Face Attribute") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_face_attributes({{"stress", 0.0}});
        mesh.set_face_attribute(0, "stress", 2.5);
        MINI_CHECK(*mesh.face_attribute(0, "stress") == 2.5);
        MINI_CHECK(*mesh.face_attribute(1, "stress") == 0.0);
    }

    MINI_TEST("Mesh", "Edge Attribute") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_edge_attributes({{"weight", 1.0}});
        mesh.set_edge_attribute({0, 1}, "weight", 5.0);
        MINI_CHECK(*mesh.edge_attribute({0, 1}, "weight") == 5.0);
        MINI_CHECK(*mesh.edge_attribute({0, 3}, "weight") == 1.0);
    }

    MINI_TEST("Mesh", "Vertices Attribute Bulk") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes({{"is_support", 0.0}});
        std::vector<size_t> keys = {0, 1, 2};
        mesh.set_vertices_attribute("is_support", 1.0, &keys);
        auto vals = mesh.vertices_attribute("is_support");
        MINI_CHECK(*vals[0] == 1.0);
        MINI_CHECK(*vals[1] == 1.0);
        MINI_CHECK(*vals[2] == 1.0);
        MINI_CHECK(*vals[3] == 0.0);
    }

    MINI_TEST("Mesh", "Vertices Where") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes({{"is_support", 0.0}});
        std::vector<size_t> keys = {0, 2, 4};
        mesh.set_vertices_attribute("is_support", 1.0, &keys);
        auto sup = mesh.vertices_where({{"is_support", 1.0}});
        std::sort(sup.begin(), sup.end());
        MINI_CHECK(sup.size() == 3);
        MINI_CHECK((sup == std::vector<size_t>{0, 2, 4}));
    }

    MINI_TEST("Mesh", "Faces Where") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_face_attributes({{"tag", 0.0}});
        mesh.set_face_attribute(2, "tag", 7.0);
        mesh.set_face_attribute(4, "tag", 7.0);
        auto out = mesh.faces_where({{"tag", 7.0}});
        std::sort(out.begin(), out.end());
        MINI_CHECK((out == std::vector<size_t>{2, 4}));
    }

    MINI_TEST("Mesh", "Edges Where") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_edge_attributes({{"weight", 0.0}});
        mesh.set_edge_attribute({0, 1}, "weight", 3.0);
        auto out = mesh.edges_where({{"weight", 3.0}});
        MINI_CHECK(out.size() == 1);
        MINI_CHECK((out[0] == std::make_pair<size_t, size_t>(0, 1)));
    }

    MINI_TEST("Mesh", "Vertices Where Predicate") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_vertex_attributes({{"load", 0.0}});
        mesh.set_vertex_attribute(0, "load", 5.0);
        mesh.set_vertex_attribute(1, "load", 10.0);
        auto big = mesh.vertices_where_predicate([](size_t, const std::map<std::string, double>& a) {
            auto it = a.find("load");
            return it != a.end() && it->second > 4.0;
        });
        std::sort(big.begin(), big.end());
        MINI_CHECK((big == std::vector<size_t>{0, 1}));
    }

    MINI_TEST("Mesh", "Faces Where Predicate") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_face_attributes({{"area", 0.0}});
        mesh.set_face_attribute(0, "area", 2.0);
        mesh.set_face_attribute(3, "area", 4.0);
        auto big = mesh.faces_where_predicate([](size_t, const std::map<std::string, double>& a) {
            auto it = a.find("area");
            return it != a.end() && it->second > 1.0;
        });
        std::sort(big.begin(), big.end());
        MINI_CHECK((big == std::vector<size_t>{0, 3}));
    }

    MINI_TEST("Mesh", "Edges Where Predicate") {
        Mesh mesh = Mesh::create_box(1.0, 1.0, 1.0);
        mesh.update_default_edge_attributes({{"weight", 0.0}});
        mesh.set_edge_attribute({0, 1}, "weight", 5.0);
        auto big = mesh.edges_where_predicate([](std::pair<size_t, size_t>, const std::map<std::string, double>& a) {
            auto it = a.find("weight");
            return it != a.end() && it->second > 1.0;
        });
        MINI_CHECK(big.size() == 1);
        MINI_CHECK((big[0] == std::make_pair<size_t, size_t>(0, 1)));
    }

} // namespace session_cpp
