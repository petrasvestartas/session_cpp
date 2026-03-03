#include "session.h"
#include "objects.h"
#include "mesh.h"
#include "point.h"
#include "color.h"
#include "polyline.h"
#include <filesystem>
#include <vector>
#include <array>
#include <map>
#include <cmath>
#include <iostream>


//   c:\brg\code_rust\session\session_cpp\main_1.cpp                            
                                                                             
//   I dont care about face colors, remove coloring                             
//   nor name of mesh                                                           
                                                                             
//   Dont use                                                                   
                                                                             
//   using V3d = std::array<double, 3>;                                         
//   using PolyRaw = std::vector<V3d>;                                          
//   using PolysRaw = std::vector<PolyRaw>;                                     
                                                                             
//   Instead use Polyline class                                                 
                                                                             
//   Implement new feature: at the center of loft edge quad draw a line in      
//   the direction of a mesh edge, the center can be computer by taking 4       
//   corner points and making the line in both direction by half length of      
//   the edge lnegth, the orientation must be quad face orientation  

using namespace session_cpp;
using V3d = std::array<double, 3>;
using PolyRaw = std::vector<V3d>;
using PolysRaw = std::vector<PolyRaw>;

static std::pair<PolysRaw, PolysRaw> load_polys_from_pb(const std::string& filepath) {
    auto obj = Objects::pb_load(filepath);
    PolysRaw top, bot;
    for (const auto& pl : *obj.polylines) {
        PolyRaw raw;
        const auto& c = pl->_coords;
        for (size_t i = 0; i + 2 < c.size(); i += 3)
            raw.push_back({c[i], c[i+1], c[i+2]});
        if (pl->name.rfind("top_", 0) == 0)
            top.push_back(std::move(raw));
        else
            bot.push_back(std::move(raw));
    }
    return {top, bot};
}

static std::array<double,3> newell_normal(const std::vector<Point>& pts) {
    double nx=0, ny=0, nz=0;
    int n = (int)pts.size();
    for (int i = 0; i < n; i++) {
        const auto& a = pts[i];
        const auto& b = pts[(i+1)%n];
        nx += (a[1]-b[1]) * (a[2]+b[2]);
        ny += (a[2]-b[2]) * (a[0]+b[0]);
        nz += (a[0]-b[0]) * (a[1]+b[1]);
    }
    return {nx, ny, nz};
}

static std::vector<std::vector<Point>> to_polys(const PolysRaw& raw) {
    std::vector<std::vector<Point>> result;
    result.reserve(raw.size());
    for (const auto& poly : raw) {
        std::vector<Point> pts;
        pts.reserve(poly.size());
        for (const auto& v : poly)
            pts.push_back(Point(v[0], v[1], v[2]));
        // Remove closing point and any near-duplicate consecutive vertices (incl. wrap-around)
        const double tol = 1e-3;
        bool changed = true;
        while (changed && pts.size() >= 3) {
            changed = false;
            std::vector<Point> clean;
            for (size_t i = 0; i < pts.size(); i++) {
                if (pts[i].distance(pts[(i+1) % pts.size()]) > tol)
                    clean.push_back(pts[i]);
                else
                    changed = true;
            }
            pts = clean;
        }
        result.push_back(std::move(pts));
    }
    return result;
}

struct WallFaceInfo {
    size_t face_key;
    bool is_quad;
    size_t top_v0, top_v1;
    size_t bot_v0, bot_v1;
};

struct LoftPanel {
    size_t top_face_key, bot_face_key;
    Mesh mesh;
    std::vector<WallFaceInfo> wall_faces;
    std::map<size_t, size_t> orig_top_to_local;
    std::map<size_t, size_t> orig_bot_to_local;
};

static LoftPanel build_panel(size_t tfk, size_t bfk,
                              const Mesh& top_mesh, const Mesh& bot_mesh) {
    LoftPanel panel;
    panel.top_face_key = tfk;
    panel.bot_face_key = bfk;

    auto top_vkeys = *top_mesh.face_vertices(tfk);
    auto bot_vkeys = *bot_mesh.face_vertices(bfk);

    std::vector<Point> top_pts, bot_pts;
    for (auto vk : top_vkeys) top_pts.push_back(*top_mesh.vertex_position(vk));
    for (auto vk : bot_vkeys) bot_pts.push_back(*bot_mesh.vertex_position(vk));

    // Merge collinear segments
    {
        auto merge = [](std::vector<Point>& pts, std::vector<size_t>& vkeys) {
            const double tol = Tolerance::APPROXIMATION;
            const double zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
            bool changed = true;
            while (changed) {
                changed = false;
                int m = (int)pts.size();
                if (m < 3) break;
                std::vector<Point> np; std::vector<size_t> nk;
                for (int i = 0; i < m; i++) {
                    int p=(i-1+m)%m, nx=(i+1)%m;
                    double ax=pts[i][0]-pts[p][0], ay=pts[i][1]-pts[p][1], az=pts[i][2]-pts[p][2];
                    double bx=pts[nx][0]-pts[i][0], by=pts[nx][1]-pts[i][1], bz=pts[nx][2]-pts[i][2];
                    double cx=ay*bz-az*by, cy=az*bx-ax*bz, cz=ax*by-ay*bx;
                    double a2=ax*ax+ay*ay+az*az, b2=bx*bx+by*by+bz*bz;
                    if (a2<zt2||b2<zt2||cx*cx+cy*cy+cz*cz<tol*tol*a2*b2) changed=true;
                    else { np.push_back(pts[i]); nk.push_back(vkeys[i]); }
                }
                pts=np; vkeys=nk;
            }
        };
        merge(top_pts, top_vkeys);
        merge(bot_pts, bot_vkeys);
    }
    int n = (int)top_pts.size();
    int m = (int)bot_pts.size();

    double tcx=0,tcy=0,tcz=0, bcx=0,bcy=0,bcz=0;
    for (auto& p : top_pts) { tcx+=p[0]; tcy+=p[1]; tcz+=p[2]; }
    for (auto& p : bot_pts) { bcx+=p[0]; bcy+=p[1]; bcz+=p[2]; }
    tcx/=n; tcy/=n; tcz/=n;
    bcx/=m; bcy/=m; bcz/=m;
    double ax=tcx-bcx, ay=tcy-bcy, az=tcz-bcz;
    double alen=std::sqrt(ax*ax+ay*ay+az*az);
    if (alen>1e-12) {ax/=alen; ay/=alen; az/=alen;}

    auto [tnx,tny,tnz] = newell_normal(top_pts);
    if (tnx*ax+tny*ay+tnz*az < 0) {
        std::reverse(top_pts.begin(), top_pts.end());
        std::reverse(top_vkeys.begin(), top_vkeys.end());
    }
    auto [bnx,bny,bnz] = newell_normal(bot_pts);
    if (bnx*ax+bny*ay+bnz*az < 0) {
        std::reverse(bot_pts.begin(), bot_pts.end());
        std::reverse(bot_vkeys.begin(), bot_vkeys.end());
    }

    for (int i = 0; i < n; i++) {
        size_t lk = panel.mesh.add_vertex(top_pts[i]);
        panel.orig_top_to_local[top_vkeys[i]] = lk;
    }
    for (int j = 0; j < m; j++) {
        size_t lk = panel.mesh.add_vertex(bot_pts[j]);
        panel.orig_bot_to_local[bot_vkeys[j]] = lk;
    }

    // Top cap
    {
        std::vector<size_t> top_cap;
        for (auto vk : top_vkeys) top_cap.push_back(panel.orig_top_to_local[vk]);
        panel.mesh.add_face(top_cap);
    }

    std::vector<Point> top_mids(n), bot_mids(m);
    for (int i = 0; i < n; i++)
        top_mids[i] = Point((top_pts[i][0]+top_pts[(i+1)%n][0])*0.5,
                            (top_pts[i][1]+top_pts[(i+1)%n][1])*0.5,
                            (top_pts[i][2]+top_pts[(i+1)%n][2])*0.5);
    for (int j = 0; j < m; j++)
        bot_mids[j] = Point((bot_pts[j][0]+bot_pts[(j+1)%m][0])*0.5,
                            (bot_pts[j][1]+bot_pts[(j+1)%m][1])*0.5,
                            (bot_pts[j][2]+bot_pts[(j+1)%m][2])*0.5);

    std::vector<int> bot_to_top(m, -1);
    std::vector<double> bot_dist(m, 1e300);
    for (int j = 0; j < m; j++)
        for (int i = 0; i < n; i++) {
            double d = bot_mids[j].distance(top_mids[i]);
            if (d < bot_dist[j]) { bot_dist[j] = d; bot_to_top[j] = i; }
        }

    std::vector<int> top_to_bot(n, -1);
    std::vector<double> top_dist(n, 1e300);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++) {
            double d = top_mids[i].distance(bot_mids[j]);
            if (d < top_dist[i]) { top_dist[i] = d; top_to_bot[i] = j; }
        }

    double avg = 0;
    for (int j = 0; j < m; j++) avg += bot_dist[j];
    avg /= m;
    double threshold = avg * 2.0;

    std::vector<bool> top_used(n, false);
    for (int j = 0; j < m; j++) {
        size_t b0 = panel.orig_bot_to_local[bot_vkeys[j]];
        size_t b1 = panel.orig_bot_to_local[bot_vkeys[(j+1)%m]];
        int ti = bot_to_top[j];
        if (ti >= 0 && bot_dist[j] <= threshold && top_to_bot[ti] == j) {
            size_t t0 = panel.orig_top_to_local[top_vkeys[ti]];
            size_t t1 = panel.orig_top_to_local[top_vkeys[(ti+1)%n]];
            double bdx = bot_pts[(j+1)%m][0]-bot_pts[j][0],
                   bdy = bot_pts[(j+1)%m][1]-bot_pts[j][1],
                   bdz = bot_pts[(j+1)%m][2]-bot_pts[j][2];
            double tdx = top_pts[(ti+1)%n][0]-top_pts[ti][0],
                   tdy = top_pts[(ti+1)%n][1]-top_pts[ti][1],
                   tdz = top_pts[(ti+1)%n][2]-top_pts[ti][2];
            if (bdx*tdx + bdy*tdy + bdz*tdz < 0) std::swap(t0, t1);
            auto fk = panel.mesh.add_face({b0, b1, t1, t0});
            if (fk) {
                WallFaceInfo w; w.face_key = *fk; w.is_quad = true;
                w.top_v0 = top_vkeys[ti]; w.top_v1 = top_vkeys[(ti+1)%n];
                w.bot_v0 = bot_vkeys[j];  w.bot_v1 = bot_vkeys[(j+1)%m];
                panel.wall_faces.push_back(w);
            }
            top_used[ti] = true;
        } else {
            double best_d = 1e300; int best_tv = 0;
            for (int i = 0; i < n; i++) {
                double d = bot_mids[j].distance(top_pts[i]);
                if (d < best_d) { best_d = d; best_tv = i; }
            }
            size_t tv = panel.orig_top_to_local[top_vkeys[best_tv]];
            auto fk = panel.mesh.add_face({b0, b1, tv});
            if (fk) { WallFaceInfo w; w.face_key = *fk; w.is_quad = false; panel.wall_faces.push_back(w); }
        }
    }

    for (int i = 0; i < n; i++) {
        if (top_used[i]) continue;
        size_t t0 = panel.orig_top_to_local[top_vkeys[i]];
        size_t t1 = panel.orig_top_to_local[top_vkeys[(i+1)%n]];
        double best_d = 1e300; int best_bv = 0;
        for (int j = 0; j < m; j++) {
            double d = top_mids[i].distance(bot_pts[j]);
            if (d < best_d) { best_d = d; best_bv = j; }
        }
        size_t bv = panel.orig_bot_to_local[bot_vkeys[best_bv]];
        auto fk = panel.mesh.add_face({t1, t0, bv});
        if (fk) { WallFaceInfo w; w.face_key = *fk; w.is_quad = false; panel.wall_faces.push_back(w); }
    }

    // Bot cap
    {
        std::vector<size_t> bot_cap;
        for (int j = 0; j < m; j++)
            bot_cap.push_back(panel.orig_bot_to_local[bot_vkeys[j]]);
        panel.mesh.add_face(bot_cap);
    }

    panel.mesh.unify_winding();
    return panel;
}

static Color panel_color(int idx) {
    const Color colors[] = {
        Color::orange(), Color::yellow(), Color::lime(),    Color::green(),
        Color::mint(),   Color::cyan(),   Color::blue(),    Color::violet(),
        Color::magenta(),Color::pink(),   Color::red(),     Color::grey()
    };
    return colors[idx % 12];
}

static void run_dataset(const PolysRaw& top_raw, const PolysRaw& bot_raw,
                        const std::string& name, const std::string& sdir) {
    auto top_pts = to_polys(top_raw);
    auto bot_pts = to_polys(bot_raw);

    Mesh top_mesh = Mesh::from_polylines(top_pts, 0.001);
    Mesh bot_mesh = Mesh::from_polylines(bot_pts, 0.001);
    top_mesh.facecolors = {Color::blue()};
    bot_mesh.facecolors = {Color::green()};

    // 1-to-1 optimal matching by centroid proximity: sort all pairs by distance,
    // assign closest pairs first (each face used at most once)
    auto face_centroid = [](const Mesh& m, size_t fk) -> Point {
        auto vkeys = *m.face_vertices(fk);
        double cx=0,cy=0,cz=0;
        for (auto vk : vkeys) { auto p=*m.vertex_position(vk); cx+=p[0]; cy+=p[1]; cz+=p[2]; }
        return Point(cx/vkeys.size(), cy/vkeys.size(), cz/vkeys.size());
    };
    std::vector<size_t> tfks, bfks;
    for (auto& [fk, _] : top_mesh.face) tfks.push_back(fk);
    for (auto& [fk, _] : bot_mesh.face) bfks.push_back(fk);
    std::vector<std::tuple<double, size_t, size_t>> dists;
    dists.reserve(tfks.size() * bfks.size());
    for (size_t ti = 0; ti < tfks.size(); ti++)
        for (size_t bi = 0; bi < bfks.size(); bi++)
            dists.push_back({face_centroid(top_mesh, tfks[ti]).distance(
                             face_centroid(bot_mesh, bfks[bi])), ti, bi});
    std::sort(dists.begin(), dists.end());
    std::vector<bool> top_used(tfks.size(), false), bot_used(bfks.size(), false);
    std::map<size_t, size_t> face_match;
    for (auto& [d, ti, bi] : dists) {
        if (top_used[ti] || bot_used[bi]) continue;
        face_match[tfks[ti]] = bfks[bi];
        top_used[ti] = true;
        bot_used[bi] = true;
    }

    std::vector<LoftPanel> panels;
    for (auto& [tfk, bfk] : face_match)
        panels.push_back(build_panel(tfk, bfk, top_mesh, bot_mesh));

    Session session(name);
    session.add_mesh(std::make_shared<Mesh>(top_mesh));
    session.add_mesh(std::make_shared<Mesh>(bot_mesh));
    for (int i = 0; i < (int)panels.size(); i++) {
        panels[i].mesh.facecolors = {panel_color(i)};
        session.add_mesh(std::make_shared<Mesh>(panels[i].mesh));
        for (const auto& w : panels[i].wall_faces) {
            if (!w.is_quad) continue;
            auto pt0 = *panels[i].mesh.vertex_position(panels[i].orig_top_to_local.at(w.top_v0));
            auto pt1 = *panels[i].mesh.vertex_position(panels[i].orig_top_to_local.at(w.top_v1));
            auto pb0 = *panels[i].mesh.vertex_position(panels[i].orig_bot_to_local.at(w.bot_v0));
            auto pb1 = *panels[i].mesh.vertex_position(panels[i].orig_bot_to_local.at(w.bot_v1));
            Point mid0((pt0[0]+pb0[0])*0.5, (pt0[1]+pb0[1])*0.5, (pt0[2]+pb0[2])*0.5);
            Point mid1((pt1[0]+pb1[0])*0.5, (pt1[1]+pb1[1])*0.5, (pt1[2]+pb1[2])*0.5);
            session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{mid0, mid1}));
        }
    }
    std::string filepath = (std::filesystem::path(sdir) / (name + "_out.pb")).string();
    session.pb_dump(filepath);
    std::cout << "Saved " << filepath << "\n";
}

int main() {
    std::string sdir = (std::filesystem::path(__FILE__).parent_path().parent_path()
                        / "session_data").string();
    for (int i = 0; i <= 11; i++) {
        std::string name = "mesh_quad_tri_loft" + std::to_string(i);
        auto [top, bot] = load_polys_from_pb(sdir + "/" + name + ".pb");
        run_dataset(top, bot, name, sdir);
    }
    return 0;
}
