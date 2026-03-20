#include <cmath>
#include <filesystem>
#include <string>
#include <vector>
#include "color.h"
#include "line.h"
#include "mesh.h"
#include "polyline.h"
#include "session.h"
#include "xform.h"
using namespace session_cpp;

static Line make_arrow(const Point& o, const Vector& d, double s) {
    return Line::from_points(o, Point(o[0]+d[0]*s, o[1]+d[1]*s, o[2]+d[2]*s));
}

int main() {
    Session session("Mesh Geometry");

    // Single mesh: 2×2×2 box, area=24, vol=8
    Mesh mesh = Mesh::create_dodecahedron(1.5);
    mesh.name = "box_2x2x2";
    auto vkeys = mesh.vertices();
    auto fkeys = mesh.faces();
    const double arrow_s = 0.5;

    // ─────────────────────────────────────────────────────────────────────
    // Layer 1: Mesh
    // ─────────────────────────────────────────────────────────────────────
    auto grp_mesh = session.add_group("Mesh");
    {
        auto m = std::make_shared<Mesh>(mesh);
        m->set_objectcolor(Color(180, 180, 180));
        session.add(session.add_mesh(m), grp_mesh);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 2: Centroids
    // ─────────────────────────────────────────────────────────────────────
    auto grp_centroids = session.add_group("Centroids");
    {
        Point mc = mesh.centroid();
        auto pt = std::make_shared<Point>(mc[0], mc[1], mc[2], "mesh_centroid");
        pt->pointcolor = Color::white();
        session.add(session.add_point(pt), grp_centroids);

        for (size_t fk : fkeys) {
            auto fc = mesh.face_centroid(fk);
            if (!fc) continue;
            auto fpt = std::make_shared<Point>(
                (*fc)[0], (*fc)[1], (*fc)[2],
                "face_centroid_f" + std::to_string(fk));
            fpt->pointcolor = Color::white();
            session.add(session.add_point(fpt), grp_centroids);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 3: Face Normals
    // ─────────────────────────────────────────────────────────────────────
    auto grp_fnormals = session.add_group("Face Normals");
    {
        auto fns = mesh.face_normals();
        for (auto& [fk, fn] : fns) {
            auto fc = mesh.face_centroid(fk);
            if (!fc) continue;
            Line arrow = make_arrow(*fc, fn, arrow_s);
            arrow.name = "fn_f" + std::to_string(fk);
            arrow.linecolor = Color(220, 50, 50);
            session.add(session.add_line(std::make_shared<Line>(arrow)), grp_fnormals);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 4: Vertex Normals (Area)
    // ─────────────────────────────────────────────────────────────────────
    auto grp_vnarea = session.add_group("Vertex Normals (Area)");
    {
        auto vns = mesh.vertex_normals();
        for (auto& [vk, vn] : vns) {
            auto vp = mesh.vertex_point(vk);
            if (!vp) continue;
            Line arrow = make_arrow(*vp, vn, arrow_s);
            arrow.name = "vn_area_v" + std::to_string(vk);
            arrow.linecolor = Color(50, 100, 220);
            session.add(session.add_line(std::make_shared<Line>(arrow)), grp_vnarea);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 5: Vertex Normals (Angle)
    // ─────────────────────────────────────────────────────────────────────
    auto grp_vnangle = session.add_group("Vertex Normals (Angle)");
    {
        auto vnsw = mesh.vertex_normals_weighted(NormalWeighting::Angle);
        for (auto& [vk, vn] : vnsw) {
            auto vp = mesh.vertex_point(vk);
            if (!vp) continue;
            Line arrow = make_arrow(*vp, vn, arrow_s);
            arrow.name = "vn_angle_v" + std::to_string(vk);
            arrow.linecolor = Color(0, 200, 200);
            session.add(session.add_line(std::make_shared<Line>(arrow)), grp_vnangle);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 6: Corner Angles
    // ─────────────────────────────────────────────────────────────────────
    auto grp_corners = session.add_group("Corner Angles");
    {
        const double arc_r = 0.2;
        const int arc_n = 8;
        for (size_t fk : fkeys) {
            auto fvs_opt = mesh.face_vertices(fk);
            if (!fvs_opt) continue;
            auto& fvs = *fvs_opt;
            int n = (int)fvs.size();
            for (int i = 0; i < n; i++) {
                size_t vk = fvs[i];
                size_t pk = fvs[(i - 1 + n) % n];
                size_t nk = fvs[(i + 1) % n];
                auto vp = mesh.vertex_point(vk);
                auto pp = mesh.vertex_point(pk);
                auto np = mesh.vertex_point(nk);
                if (!vp || !pp || !np) continue;
                auto theta_opt = mesh.vertex_angle_in_face(vk, fk);
                if (!theta_opt) continue;
                double theta = *theta_opt;
                if (std::abs(std::sin(theta)) < 1e-10) continue;
                double ux = (*pp)[0]-(*vp)[0], uy = (*pp)[1]-(*vp)[1], uz = (*pp)[2]-(*vp)[2];
                double vx = (*np)[0]-(*vp)[0], vy = (*np)[1]-(*vp)[1], vz = (*np)[2]-(*vp)[2];
                double ulen = std::sqrt(ux*ux + uy*uy + uz*uz);
                double vlen = std::sqrt(vx*vx + vy*vy + vz*vz);
                if (ulen < 1e-10 || vlen < 1e-10) continue;
                ux /= ulen; uy /= ulen; uz /= ulen;
                vx /= vlen; vy /= vlen; vz /= vlen;
                std::vector<Point> arc_pts;
                for (int j = 0; j <= arc_n; j++) {
                    double t  = (double)j / arc_n;
                    double w1 = std::sin((1.0 - t) * theta) / std::sin(theta);
                    double w2 = std::sin(t * theta) / std::sin(theta);
                    arc_pts.push_back(Point(
                        (*vp)[0] + (w1*ux + w2*vx) * arc_r,
                        (*vp)[1] + (w1*uy + w2*vy) * arc_r,
                        (*vp)[2] + (w1*uz + w2*vz) * arc_r));
                }
                auto arc = std::make_shared<Polyline>(arc_pts);
                arc->name = "angle_v" + std::to_string(vk) + "_f" + std::to_string(fk)
                          + "=" + std::to_string(theta);
                arc->linecolor = Color(240, 140, 0);
                session.add(session.add_polyline(arc), grp_corners);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 7: Tributary Area
    // ─────────────────────────────────────────────────────────────────────
    auto grp_trib = session.add_group("Tributary Area");
    {
        for (size_t vk : vkeys) {
            auto vp = mesh.vertex_point(vk);
            if (!vp) continue;
            auto vfaces_opt = mesh.vertex_faces(vk);
            if (!vfaces_opt) continue;
            double total = 0.0;
            for (size_t fk : *vfaces_opt) {
                auto fc = mesh.face_centroid(fk);
                auto fa = mesh.face_area(fk);
                auto fvs = mesh.face_vertices(fk);
                if (!fc || !fa || !fvs) continue;
                total += *fa / (double)fvs->size();
                std::vector<Point> seg_pts = {
                    Point((*vp)[0], (*vp)[1], (*vp)[2]),
                    Point((*fc)[0], (*fc)[1], (*fc)[2]),
                };
                auto seg = std::make_shared<Polyline>(seg_pts);
                seg->name = "trib_v" + std::to_string(vk) + "_f" + std::to_string(fk);
                seg->linecolor = Color(50, 200, 80);
                session.add(session.add_polyline(seg), grp_trib);
            }
            auto pt = std::make_shared<Point>(
                (*vp)[0], (*vp)[1], (*vp)[2],
                "trib_v" + std::to_string(vk) + "=" + std::to_string(total));
            pt->pointcolor = Color(50, 200, 80);
            session.add(session.add_point(pt), grp_trib);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 8: Dihedral Angle
    // ─────────────────────────────────────────────────────────────────────
    auto grp_dihedral = session.add_group("Dihedral Angle");
    {
        auto fns = mesh.face_normals();
        for (auto& [fk, fn] : fns) {
            auto fc = mesh.face_centroid(fk);
            if (!fc) continue;
            Line arrow = make_arrow(*fc, fn, arrow_s);
            arrow.name = "fn_f" + std::to_string(fk);
            arrow.linecolor = Color(220, 50, 50);
            session.add(session.add_line(std::make_shared<Line>(arrow)), grp_dihedral);
        }

        const double arc_r = 0.3;
        const int arc_n = 12;
        const double pi = std::acos(-1.0);
        for (auto& [u, v] : mesh.edges()) {
            auto da  = mesh.dihedral_angle(u, v);
            auto ep0 = mesh.vertex_point(u);
            auto ep1 = mesh.vertex_point(v);
            if (!da || !ep0 || !ep1) continue;
            auto ef = mesh.edge_faces(u, v);
            if (!ef || ef->size() < 2) continue;
            double mx = ((*ep0)[0]+(*ep1)[0])*0.5;
            double my = ((*ep0)[1]+(*ep1)[1])*0.5;
            double mz = ((*ep0)[2]+(*ep1)[2])*0.5;
            double ex = (*ep1)[0]-(*ep0)[0], ey = (*ep1)[1]-(*ep0)[1], ez = (*ep1)[2]-(*ep0)[2];
            double elen = std::sqrt(ex*ex + ey*ey + ez*ez);
            if (elen < 1e-10) continue;
            ex /= elen; ey /= elen; ez /= elen;
            auto fc0 = mesh.face_centroid((*ef)[0]);
            auto fc1 = mesh.face_centroid((*ef)[1]);
            if (!fc0 || !fc1) continue;
            double d0x = (*fc0)[0]-mx, d0y = (*fc0)[1]-my, d0z = (*fc0)[2]-mz;
            double dot0 = d0x*ex + d0y*ey + d0z*ez;
            d0x -= dot0*ex; d0y -= dot0*ey; d0z -= dot0*ez;
            double d0len = std::sqrt(d0x*d0x + d0y*d0y + d0z*d0z);
            if (d0len < 1e-10) continue;
            d0x /= d0len; d0y /= d0len; d0z /= d0len;
            double d1x = (*fc1)[0]-mx, d1y = (*fc1)[1]-my, d1z = (*fc1)[2]-mz;
            double dot1 = d1x*ex + d1y*ey + d1z*ez;
            d1x -= dot1*ex; d1y -= dot1*ey; d1z -= dot1*ez;
            double d1len = std::sqrt(d1x*d1x + d1y*d1y + d1z*d1z);
            if (d1len < 1e-10) continue;
            d1x /= d1len; d1y /= d1len; d1z /= d1len;
            double theta = std::acos(std::max(-1.0, std::min(1.0, d0x*d1x + d0y*d1y + d0z*d1z)));
            if (std::abs(std::sin(theta)) < 1e-10) continue;
            double deg = theta * 180.0 / pi;
            std::vector<Point> arc_pts;
            for (int j = 0; j <= arc_n; j++) {
                double t  = (double)j / arc_n;
                double w1 = std::sin((1.0-t)*theta) / std::sin(theta);
                double w2 = std::sin(t*theta) / std::sin(theta);
                arc_pts.push_back(Point(
                    mx + (w1*d0x + w2*d1x)*arc_r,
                    my + (w1*d0y + w2*d1y)*arc_r,
                    mz + (w1*d0z + w2*d1z)*arc_r));
            }
            auto arc = std::make_shared<Polyline>(arc_pts);
            arc->name = "dihedral_e"+std::to_string(u)+"_"+std::to_string(v)+"="+std::to_string(deg);
            arc->linecolor = Color(240, 220, 0);
            session.add(session.add_polyline(arc), grp_dihedral);
            auto tpt = std::make_shared<Point>(
                arc_pts[arc_n/2][0], arc_pts[arc_n/2][1], arc_pts[arc_n/2][2],
                std::to_string(deg));
            tpt->pointcolor = Color(240, 220, 0);
            session.add(session.add_point(tpt), grp_dihedral);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 9: Area Volume
    // ─────────────────────────────────────────────────────────────────────
    auto grp_area_vol = session.add_group("Area Volume");
    {
        auto m = std::make_shared<Mesh>(mesh);
        m->name = "area=" + std::to_string(mesh.area()) + "_vol=" + std::to_string(mesh.volume());
        m->set_objectcolor(Color(100, 180, 100));
        session.add(session.add_mesh(m), grp_area_vol);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 10: Transformation
    // ─────────────────────────────────────────────────────────────────────
    auto grp_xform = session.add_group("Transformation");
    {
        // transform() — apply stored xform in-place
        Mesh m1 = mesh;
        m1.name = "transform_stored_xform";
        m1.xform = Xform::translation(4.0, 0.0, 0.0);
        m1.transform();
        m1.set_objectcolor(Color(200, 100, 50));
        session.add(session.add_mesh(std::make_shared<Mesh>(m1)), grp_xform);

        // transform(Xform&) — apply given xform in-place
        Mesh m2 = mesh;
        m2.name = "transform_given_xform";
        m2.transform(Xform::translation(8.0, 0.0, 0.0));
        m2.set_objectcolor(Color(50, 150, 200));
        session.add(session.add_mesh(std::make_shared<Mesh>(m2)), grp_xform);

        // transformed() — copy with stored xform applied
        Mesh m3 = mesh;
        m3.xform = Xform::translation(12.0, 0.0, 0.0);
        Mesh m3t = m3.transformed();
        m3t.name = "transformed_stored_copy";
        m3t.set_objectcolor(Color(180, 50, 180));
        session.add(session.add_mesh(std::make_shared<Mesh>(m3t)), grp_xform);

        // transformed(Xform&) — copy with given xform applied
        Mesh m4t = mesh.transformed(Xform::translation(16.0, 0.0, 0.0));
        m4t.name = "transformed_given_xform";
        m4t.set_objectcolor(Color(200, 200, 50));
        session.add(session.add_mesh(std::make_shared<Mesh>(m4t)), grp_xform);
    }

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "Mesh_Geometry.pb").string();
    session.pb_dump(fp);
    return 0;
}
