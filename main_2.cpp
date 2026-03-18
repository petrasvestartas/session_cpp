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

    // ── Flat mesh (4 verts, 2 triangles, XY plane) ───────────────────────
    // v0=(0,0,0) v1=(1,0,0) v2=(-1,0,0) v3=(0,1,0)
    // f0={0,1,3}  f1={0,3,2}
    std::vector<Point> fpts = {
        Point(0.0,  0.0, 0.0),
        Point(1.0,  0.0, 0.0),
        Point(-1.0, 0.0, 0.0),
        Point(0.0,  1.0, 0.0),
    };
    Mesh flat = Mesh::from_vertices_and_faces(fpts, {{0,1,3}, {0,3,2}});
    flat.name = "flat_mesh";
    auto fvkeys = flat.vertices();
    auto ffkeys = flat.faces();
    const double arrow_s = 0.5;

    // ── Dihedral "tent" mesh (90° dihedral, y=3) ─────────────────────────
    // v0=(0,3,0) v1=(2,3,0) v2=(1,4,0) v3=(1,3,-1)
    // f0={0,1,2} normal=(0,0,1)  f1={1,0,3} normal=(0,-1,0)
    // shared edge v0–v1, dihedral_angle = PI/2
    std::vector<Point> dpts = {
        Point(0.0, 3.0,  0.0),
        Point(2.0, 3.0,  0.0),
        Point(1.0, 4.0,  0.0),
        Point(1.0, 3.0, -1.0),
    };
    Mesh dmesh = Mesh::from_vertices_and_faces(dpts, {{0,1,2}, {1,0,3}});
    dmesh.name = "dihedral_tent";
    auto dvkeys = dmesh.vertices();

    // ── Box mesh (area=24, vol=8, centered at y=6) ────────────────────────
    Mesh box = Mesh::create_box(2.0, 2.0, 2.0);
    box.transform(Xform::translation(0.0, 6.0, 0.0));
    box.name = "box_area=24.0_vol=8.0";

    // ── Base triangle for transform demos (x=5) ───────────────────────────
    std::vector<Point> tpts = {
        Point(5.0, 0.0, 0.0),
        Point(6.0, 0.0, 0.0),
        Point(5.5, 1.0, 0.0),
    };
    Mesh tmesh = Mesh::from_vertices_and_faces(tpts, {{0,1,2}});

    // ─────────────────────────────────────────────────────────────────────
    // Layer 1: Mesh
    // ─────────────────────────────────────────────────────────────────────
    auto grp_mesh = session.add_group("Mesh");
    {
        auto mf = std::make_shared<Mesh>(flat);
        mf->set_objectcolor(Color(180, 180, 180));
        session.add(session.add_mesh(mf), grp_mesh);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 2: Centroids
    // ─────────────────────────────────────────────────────────────────────
    auto grp_centroids = session.add_group("Centroids");
    {
        Point mc = flat.centroid();
        auto pt = std::make_shared<Point>(mc[0], mc[1], mc[2], "mesh_centroid");
        pt->pointcolor = Color::white();
        session.add(session.add_point(pt), grp_centroids);

        for (size_t fk : ffkeys) {
            auto fc = flat.face_centroid(fk);
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
        auto fns = flat.face_normals();
        for (auto& [fk, fn] : fns) {
            auto fc = flat.face_centroid(fk);
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
        auto vns = flat.vertex_normals();
        for (auto& [vk, vn] : vns) {
            auto vp = flat.vertex_point(vk);
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
        auto vnsw = flat.vertex_normals_weighted(NormalWeighting::Angle);
        for (auto& [vk, vn] : vnsw) {
            auto vp = flat.vertex_point(vk);
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
        for (size_t fk : ffkeys) {
            auto fvs_opt = flat.face_vertices(fk);
            if (!fvs_opt) continue;
            auto& fvs = *fvs_opt;
            int n = (int)fvs.size();
            for (int i = 0; i < n; i++) {
                size_t vk = fvs[i];
                size_t pk = fvs[(i - 1 + n) % n];
                size_t nk = fvs[(i + 1) % n];
                auto vp = flat.vertex_point(vk);
                auto pp = flat.vertex_point(pk);
                auto np = flat.vertex_point(nk);
                if (!vp || !pp || !np) continue;
                auto theta_opt = flat.vertex_angle_in_face(vk, fk);
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
        for (size_t vk : fvkeys) {
            auto vp = flat.vertex_point(vk);
            if (!vp) continue;
            auto vfaces_opt = flat.vertex_faces(vk);
            if (!vfaces_opt) continue;
            double total = 0.0;
            for (size_t fk : *vfaces_opt) {
                auto fc = flat.face_centroid(fk);
                auto fa = flat.face_area(fk);
                auto fvs = flat.face_vertices(fk);
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
        auto dm = std::make_shared<Mesh>(dmesh);
        dm->set_objectcolor(Color(160, 160, 200));
        session.add(session.add_mesh(dm), grp_dihedral);

        auto dfns = dmesh.face_normals();
        for (auto& [fk, fn] : dfns) {
            auto fc = dmesh.face_centroid(fk);
            if (!fc) continue;
            Line arrow = make_arrow(*fc, fn, arrow_s);
            arrow.name = "dfn_f" + std::to_string(fk);
            arrow.linecolor = Color(220, 50, 50);
            session.add(session.add_line(std::make_shared<Line>(arrow)), grp_dihedral);
        }

        auto da   = dmesh.dihedral_angle(dvkeys[0], dvkeys[1]);
        auto ep0  = dmesh.vertex_point(dvkeys[0]);
        auto ep1  = dmesh.vertex_point(dvkeys[1]);
        if (da && ep0 && ep1) {
            auto eline = std::make_shared<Line>(Line::from_points(*ep0, *ep1));
            eline->name = "da=" + std::to_string(*da);
            eline->linecolor = Color(240, 220, 0);
            eline->width = 3.0;
            session.add(session.add_line(eline), grp_dihedral);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 9: Area Volume
    // ─────────────────────────────────────────────────────────────────────
    auto grp_area_vol = session.add_group("Area Volume");
    {
        auto bm = std::make_shared<Mesh>(box);
        bm->set_objectcolor(Color(100, 180, 100));
        session.add(session.add_mesh(bm), grp_area_vol);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Layer 10: Transformation
    // ─────────────────────────────────────────────────────────────────────
    auto grp_xform = session.add_group("Transformation");
    {
        // transform() — apply stored xform in-place; xform field unchanged
        Mesh m1 = tmesh;
        m1.name = "transform_stored_xform";
        m1.xform = Xform::translation(0.0, 0.0, 1.0);
        m1.transform();
        m1.set_objectcolor(Color(200, 100, 50));
        session.add(session.add_mesh(std::make_shared<Mesh>(m1)), grp_xform);

        // transform(Xform&) — apply given xform in-place; stored xform unchanged
        Mesh m2 = tmesh;
        m2.name = "transform_given_xform";
        m2.transform(Xform::translation(0.0, 0.0, 2.0));
        m2.set_objectcolor(Color(50, 150, 200));
        session.add(session.add_mesh(std::make_shared<Mesh>(m2)), grp_xform);

        // transformed() — copy with stored xform applied; xform kept
        Mesh m3 = tmesh;
        m3.xform = Xform::translation(0.0, 0.0, 3.0);
        Mesh m3t = m3.transformed();
        m3t.name = "transformed_stored_copy";
        m3t.set_objectcolor(Color(180, 50, 180));
        session.add(session.add_mesh(std::make_shared<Mesh>(m3t)), grp_xform);

        // transformed(Xform&) — copy with given xform applied; xform identity
        Mesh m4t = tmesh.transformed(Xform::translation(0.0, 0.0, 4.0));
        m4t.name = "transformed_given_xform";
        m4t.set_objectcolor(Color(200, 200, 50));
        session.add(session.add_mesh(std::make_shared<Mesh>(m4t)), grp_xform);
    }

    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "Mesh_Geometry.pb").string();
    session.pb_dump(fp);
    return 0;
}
