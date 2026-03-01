#include "mini_test.h"
#include "mesh.h"
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

        const int sides = 6;

        // Create hexagon vertices in XY plane
        std::vector<Point> vertices;
        for (int i = 0; i < sides; ++i) {
            double angle = 2.0 * TOLERANCE.PI * i / sides;
            double x = 1.0 * std::cos(angle);
            double y = 1.0 * std::sin(angle);
            vertices.push_back({x, y, 0.0});
        }

        std::vector<std::vector<size_t>> faces = {
            {0, 1, 2, 3, 4, 5},
        };

        Mesh mesh = Mesh::from_vertices_and_faces(vertices, faces);

        std::string sstr = mesh.str();
        std::string srepr = mesh.repr();

        // Copy (new guid)
        Mesh mcopy = mesh;

        MINI_CHECK(mesh.is_valid());
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
        });
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
        for (auto& m : meshes) MINI_CHECK(m.is_valid());
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
        for (auto& m : meshes) {
            MINI_CHECK(m.is_valid());
            MINI_CHECK(m.is_closed());
        }
        std::vector<Mesh> meshes_seq = Mesh::loft_many(loft_inputs, true, false);
        MINI_CHECK(meshes_seq[0].number_of_faces() == meshes[0].number_of_faces());
    }

    MINI_TEST("Mesh", "Boolean Queries") {
        // uncomment #include "mesh.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0, 0, 0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1, 0, 0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0, 1, 0), std::nullopt);
        size_t f0 = *mesh.add_face({v0, v1, v2}, std::nullopt);

        bool not_empty = mesh.is_empty();
        MINI_CHECK(!not_empty);

        bool valid = mesh.is_valid();
        MINI_CHECK(valid);

        bool closed = mesh.is_closed();
        MINI_CHECK(!closed);

        bool vertex_on_boundary = mesh.is_vertex_on_boundary(v0);
        MINI_CHECK(vertex_on_boundary);

        bool edge_on_boundary = mesh.is_edge_on_boundary(v0, v1);
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

        auto [vertices, faces] = mesh.to_vertices_and_faces();
        MINI_CHECK(faces.size() == n_faces);
        MINI_CHECK(vertices.size() == n_vertices);
        MINI_CHECK(TOLERANCE.is_point_close(vertices[0], Point(-0.5, -0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[1], Point( 0.5, -0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[2], Point( 0.5,  0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[3], Point(-0.5,  0.5, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[4], Point(-0.5, -0.5,  0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[5], Point( 0.5, -0.5,  0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[6], Point( 0.5,  0.5,  0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(vertices[7], Point(-0.5,  0.5,  0.5)));
        MINI_CHECK((faces[0] == std::vector<size_t>{0, 3, 2, 1}));
        MINI_CHECK((faces[1] == std::vector<size_t>{4, 5, 6, 7}));
        MINI_CHECK((faces[2] == std::vector<size_t>{0, 1, 5, 4}));
        MINI_CHECK((faces[3] == std::vector<size_t>{2, 3, 7, 6}));
        MINI_CHECK((faces[4] == std::vector<size_t>{0, 4, 7, 3}));
        MINI_CHECK((faces[5] == std::vector<size_t>{1, 2, 6, 5}));

        std::map<size_t, size_t> vindex = mesh.vertex_index();
        MINI_CHECK(vindex.size() == n_vertices);
        MINI_CHECK(vindex[0] == 0);
        MINI_CHECK(vindex[1] == 1);
        MINI_CHECK(vindex[2] == 2);
        MINI_CHECK(vindex[3] == 3);
        MINI_CHECK(vindex[4] == 4);
        MINI_CHECK(vindex[5] == 5);
        MINI_CHECK(vindex[6] == 6);
        MINI_CHECK(vindex[7] == 7);

    }

    MINI_TEST("Mesh", "Vertex and Face Operations") {
        // uncomment #include "mesh.h"

        // add_vertex — nullopt key auto-assigns sequentially from 0
        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(1.0, 2.0, 3.0), std::nullopt);
        MINI_CHECK(v0 == 0);
        MINI_CHECK(mesh.number_of_vertices() == 1);
        MINI_CHECK(!mesh.is_empty());
        size_t v1 = mesh.add_vertex(Point(4.0, 5.0, 6.0), 42);
        MINI_CHECK(v1 == 42);
        MINI_CHECK(mesh.number_of_vertices() == 2);

        // add_face
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        std::optional<size_t> f = mesh.add_face({v0, v1, v2}, std::nullopt);
        MINI_CHECK(f.has_value());
        std::optional<size_t> invalid1 = mesh.add_face({v0, v1}, std::nullopt);
        MINI_CHECK(!invalid1.has_value());
        std::optional<size_t> invalid2 = mesh.add_face({v0, v1, v0}, std::nullopt);
        MINI_CHECK(!invalid2.has_value());

        // clear
        mesh.clear();
        MINI_CHECK(mesh.is_empty());
        MINI_CHECK(mesh.number_of_vertices() == 0);
        MINI_CHECK(mesh.number_of_faces() == 0);

        // unify_winding — two triangles sharing edge p1-p2, f1 has same-direction halfedge (wrong winding)
        size_t p0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t p1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t p2 = mesh.add_vertex(Point(1.0, 1.0, 0.0), std::nullopt);
        size_t p3 = mesh.add_vertex(Point(2.0, 1.0, 0.0), std::nullopt);
        size_t f0 = *mesh.add_face({p0, p1, p2}, std::nullopt);  // +z normal
        size_t f1 = *mesh.add_face({p1, p2, p3}, std::nullopt);  // -z normal (wrong: same halfedge dir)

        std::optional<Vector> n0_before = mesh.face_normal(f0);
        std::optional<Vector> n1_before = mesh.face_normal(f1);
        MINI_CHECK(n0_before.has_value() && n1_before.has_value());
        MINI_CHECK(n0_before->dot(*n1_before) < 0.0);  // wrong: normals point opposite ways

        mesh.unify_winding();

        std::optional<Vector> n0_after = mesh.face_normal(f0);
        std::optional<Vector> n1_after = mesh.face_normal(f1);
        MINI_CHECK(n0_after.has_value() && n1_after.has_value());
        MINI_CHECK(n0_after->dot(*n1_after) > 0.0);  // correct: normals agree
    }

    MINI_TEST("Mesh", "Unweld") {
        // uncomment #include "mesh.h"

        Mesh box = Mesh::create_box(1.0, 1.0, 1.0);
        Mesh u = box.unweld();

        MINI_CHECK(u.number_of_faces() == box.number_of_faces());
        MINI_CHECK(u.number_of_vertices() == 24);
        for (auto& [vk, vdata] : u.vertex)
            MINI_CHECK(u.vertex_faces(vk).size() == 1);
    }

    MINI_TEST("Mesh", "Connectivity Queries") {
        // uncomment #include "mesh.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(1.0, 1.0, 0.0), std::nullopt);
        size_t v3 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        size_t v4 = mesh.add_vertex(Point(2.0, 0.0, 0.0), std::nullopt);
        size_t f0 = *mesh.add_face({v0, v1, v2, v3}, std::nullopt);
        size_t f1 = *mesh.add_face({v1, v4, v2}, std::nullopt);

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

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(-1.0, 0.0, 0.0), std::nullopt);
        size_t v3 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        size_t f0 = *mesh.add_face({v0, v1, v3}, std::nullopt);
        size_t f1 = *mesh.add_face({v0, v3, v2}, std::nullopt);

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
    }



    MINI_TEST("Mesh", "Transformation") {
        // uncomment #include "mesh.h"

        Mesh mesh;
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

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

        Mesh mesh;
        mesh.name = "test_mesh";
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

        // JSON object
        nlohmann::ordered_json json = mesh.jsondump();
        Mesh loaded_json = Mesh::jsonload(json);
        MINI_CHECK(loaded_json.name == mesh.name);
        MINI_CHECK(loaded_json.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded_json.number_of_faces() == mesh.number_of_faces());

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
    }

    MINI_TEST("Mesh", "Protobuf Roundtrip") {
        // uncomment #include "mesh.h"
        
        Mesh mesh;
        mesh.name = "test_mesh_proto";
        size_t v0 = mesh.add_vertex(Point(0.0, 0.0, 0.0), std::nullopt);
        size_t v1 = mesh.add_vertex(Point(1.0, 0.0, 0.0), std::nullopt);
        size_t v2 = mesh.add_vertex(Point(0.0, 1.0, 0.0), std::nullopt);
        mesh.add_face({v0, v1, v2}, std::nullopt);

        // String
        std::string proto_string = mesh.pb_dumps();
        Mesh loaded_string = Mesh::pb_loads(proto_string);
        MINI_CHECK(loaded_string.name == mesh.name);
        MINI_CHECK(loaded_string.number_of_vertices() == mesh.number_of_vertices());

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_mesh.bin").string();
        mesh.pb_dump(filename);
        Mesh loaded_file = Mesh::pb_load(filename);
        MINI_CHECK(loaded_file.name == mesh.name);
        MINI_CHECK(loaded_file.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK(loaded_file.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK(loaded_file.guid == mesh.guid);
    }





} // namespace session_cpp
