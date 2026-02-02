#include "mini_test.h"
#include "triangulation_nurbs.h"
#include "nurbssurface.h"
#include "mesh.h"
#include "point.h"
#include "vector.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

    static NurbsSurface make_flat_surface() {
        NurbsSurface surf;
        surf.create_raw(3, false, 2, 2, 2, 2);
        surf.make_clamped_uniform_knot_vector(0, 1.0);
        surf.make_clamped_uniform_knot_vector(1, 1.0);
        surf.set_cv(0, 0, Point(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, Point(1.0, 1.0, 0.0));
        return surf;
    }

    static NurbsSurface make_bump_surface() {
        NurbsSurface surf;
        surf.create_raw(3, false, 3, 3, 4, 4);
        surf.make_clamped_uniform_knot_vector(0, 1.0);
        surf.make_clamped_uniform_knot_vector(1, 1.0);
        std::vector<Point> cvs = {
            Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0), Point(0.0, 2.0, 0.0), Point(0.0, 3.0, 0.0),
            Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 2.0), Point(1.0, 2.0, 2.0), Point(1.0, 3.0, 0.0),
            Point(2.0, 0.0, 0.0), Point(2.0, 1.0, 2.0), Point(2.0, 2.0, 2.0), Point(2.0, 3.0, 0.0),
            Point(3.0, 0.0, 0.0), Point(3.0, 1.0, 0.0), Point(3.0, 2.0, 0.0), Point(3.0, 3.0, 0.0),
        };
        int idx = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                surf.set_cv(i, j, cvs[idx++]);
        return surf;
    }

    static NurbsSurface make_sphere_surface(double radius) {
        double w = std::sqrt(2.0) / 2.0;
        NurbsSurface surf;
        surf.create_raw(3, true, 3, 3, 9, 5);
        surf.name = "unit_sphere";

        double u_knots[] = {0, 0, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5,
                            TOLERANCE.PI, TOLERANCE.PI, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.5,
                            TOLERANCE.PI * 2.0, TOLERANCE.PI * 2.0};
        for (int i = 0; i < 10; ++i) surf.set_knot(0, i, u_knots[i]);

        double v_knots[] = {-TOLERANCE.PI * 0.5, -TOLERANCE.PI * 0.5, 0, 0,
                            TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.5};
        for (int i = 0; i < 6; ++i) surf.set_knot(1, i, v_knots[i]);

        double lat_weights[] = {w, 0.5, w, 0.5, w};
        double lat_z[] = {-radius, -radius * w, 0.0, radius * w, radius};
        double lat_r[] = {0.0, radius * w, radius, radius * w, 0.0};

        for (int j = 0; j < 5; ++j) {
            double r = lat_r[j];
            double z = lat_z[j];
            double angles[] = {0, TOLERANCE.PI * 0.25, TOLERANCE.PI * 0.5, TOLERANCE.PI * 0.75,
                               TOLERANCE.PI, TOLERANCE.PI * 1.25, TOLERANCE.PI * 1.5, TOLERANCE.PI * 1.75,
                               TOLERANCE.PI * 2.0};
            for (int i = 0; i < 9; ++i) {
                double x = r * std::cos(angles[i]);
                double y = r * std::sin(angles[i]);
                surf.set_cv(i, j, Point(x, y, z));
                double weight = (i % 2 == 0) ? w : lat_weights[j];
                if (j == 0 || j == 4) weight = w;
                surf.set_weight(i, j, weight);
            }
        }
        return surf;
    }

    MINI_TEST("TriangulationNurbs", "constructor") {
        NurbsSurface surf = make_flat_surface();
        NurbsTriangulation mesher(surf);

        mesher.set_max_angle(30.0);
        mesher.set_max_edge_length(0.5);
        mesher.set_min_edge_length(0.01);
        mesher.set_max_chord_height(0.001);
        mesher.set_max_iterations(1000);

        MINI_CHECK(mesher.get_max_angle() == 30.0);
        MINI_CHECK(mesher.get_max_edge_length() == 0.5);
        MINI_CHECK(mesher.get_min_edge_length() == 0.01);
        MINI_CHECK(mesher.get_max_chord_height() == 0.001);
        MINI_CHECK(mesher.get_max_iterations() == 1000);
    }

    MINI_TEST("TriangulationNurbs", "mesh_flat_plane") {
        NurbsSurface surf = make_flat_surface();
        NurbsTriangulation mesher(surf);
        Mesh m = mesher.mesh();

        bool all_z_zero = true;
        for (const auto& [vk, vd] : m.vertex) {
            if (std::abs(vd.z) > 1e-6) { all_z_zero = false; break; }
        }

        MINI_CHECK(m.number_of_vertices() >= 4);
        MINI_CHECK(m.number_of_faces() >= 2);
        MINI_CHECK(all_z_zero);
    }

    MINI_TEST("TriangulationNurbs", "mesh_curved_surface") {
        NurbsSurface surf = make_bump_surface();
        NurbsTriangulation mesher(surf);
        Mesh m = mesher.mesh();

        NurbsSurface flat_surf = make_flat_surface();
        NurbsTriangulation flat_mesher(flat_surf);
        Mesh flat_m = flat_mesher.mesh();

        MINI_CHECK(m.number_of_vertices() >= 4);
        MINI_CHECK(m.number_of_faces() > 2);
        MINI_CHECK(m.number_of_faces() > flat_m.number_of_faces());
    }

    MINI_TEST("TriangulationNurbs", "mesh_vertex_normals") {
        NurbsSurface surf = make_bump_surface();
        NurbsTriangulation mesher(surf);
        Mesh m = mesher.mesh();

        bool all_have_normals = true;
        for (const auto& [vk, vd] : m.vertex) {
            auto n = vd.normal();
            if (!n.has_value()) { all_have_normals = false; break; }
            double len = std::sqrt(n->at(0) * n->at(0) + n->at(1) * n->at(1) + n->at(2) * n->at(2));
            if (len < 0.5) { all_have_normals = false; break; }
        }

        MINI_CHECK(m.number_of_vertices() >= 4);
        MINI_CHECK(all_have_normals);
    }

    MINI_TEST("TriangulationNurbs", "mesh_sphere") {
        double radius = 2.0;
        NurbsSurface surf = make_sphere_surface(radius);
        NurbsTriangulation mesher(surf);
        mesher.set_max_angle(20.0);
        Mesh m = mesher.mesh();

        int near_count = 0;
        double tol = radius * 0.3;
        for (const auto& [vk, vd] : m.vertex) {
            double dist = std::sqrt(vd.x * vd.x + vd.y * vd.y + vd.z * vd.z);
            if (std::abs(dist - radius) <= tol) near_count++;
        }
        bool most_near_radius = near_count >= static_cast<int>(m.number_of_vertices()) * 8 / 10;

        MINI_CHECK(m.number_of_vertices() >= 8);
        MINI_CHECK(m.number_of_faces() >= 8);
        MINI_CHECK(most_near_radius);
    }

    MINI_TEST("TriangulationNurbs", "mesh_parameters") {
        NurbsSurface surf = make_bump_surface();

        NurbsTriangulation coarse(surf);
        coarse.set_max_angle(90.0).set_max_edge_length(10.0).set_max_chord_height(10.0);
        Mesh m_coarse = coarse.mesh();

        NurbsTriangulation fine(surf);
        fine.set_max_angle(5.0).set_max_edge_length(0.5).set_max_chord_height(0.01);
        Mesh m_fine = fine.mesh();

        MINI_CHECK(m_coarse.number_of_faces() >= 2);
        MINI_CHECK(m_fine.number_of_faces() >= 2);
        MINI_CHECK(m_fine.number_of_faces() > m_coarse.number_of_faces());
    }

} // namespace session_cpp
