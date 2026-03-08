#include "session.h"
#include "src/trimesh_cdt.h"

static double ms_from_polylines = 0, ms_build_panel = 0, ms_brep = 0, ms_pb_dump = 0;
#define TIMED(var, ...) do { \
    auto _t0 = std::chrono::high_resolution_clock::now(); \
    __VA_ARGS__; \
    var += std::chrono::duration<double,std::milli>( \
        std::chrono::high_resolution_clock::now() - _t0).count(); \
} while(0)

using namespace session_cpp;

static std::pair<std::vector<Polyline>, std::vector<Polyline>> load_polys_from_pb(const std::string& filepath) {
    auto obj = Objects::pb_load(filepath);
    std::vector<Polyline> top, bot;
    for (const auto& pl : *obj.polylines) {
        std::vector<Point> pts;
        const auto& c = pl->_coords;
        for (size_t i = 0; i + 2 < c.size(); i += 3)
            pts.push_back(Point(c[i], c[i+1], c[i+2]));
        if (pl->name.rfind("top_", 0) == 0)
            top.emplace_back(pts);
        else
            bot.emplace_back(pts);
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

static std::vector<std::vector<Point>> to_polys(const std::vector<Polyline>& polylines) {
    std::vector<std::vector<Point>> result;
    result.reserve(polylines.size());
    for (const auto& pl : polylines) {
        std::vector<Point> pts;
        const auto& c = pl._coords;
        for (size_t i = 0; i + 2 < c.size(); i += 3)
            pts.push_back(Point(c[i], c[i+1], c[i+2]));
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

/// Removes collinear or near-zero-length vertices from a polygon in-place.
/// Uses cross-product magnitude against APPROXIMATION tolerance to detect collinearity.
/// Repeats until no more vertices are removed; stops early if fewer than 3 remain.
static void merge_collinear(std::vector<Point>& pts, std::vector<size_t>& vkeys) {
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
}

/// Returns p moved toward (cx, cy, cz) by exactly gap units.
/// If the distance is below 1e-10 the point is returned unchanged.
static Point offset_toward(const Point& p, double cx, double cy, double cz, double gap) {
    double dx=cx-p[0], dy=cy-p[1], dz=cz-p[2];
    double len=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (len>1e-10) { dx*=gap/len; dy*=gap/len; dz*=gap/len; }
    return Point(p[0]+dx, p[1]+dy, p[2]+dz);
}

/// Returns the average position of all vertices of face fk in mesh m.
static Point face_centroid(const Mesh& m, size_t fk) {
    auto vkeys = *m.face_vertices(fk);
    double cx=0,cy=0,cz=0;
    for (auto vk : vkeys) { auto p=*m.vertex_position(vk); cx+=p[0]; cy+=p[1]; cz+=p[2]; }
    return Point(cx/vkeys.size(), cy/vkeys.size(), cz/vkeys.size());
}

/// Returns the midpoint of the vertical edge connecting top vertex tvk and bottom vertex bvk
/// within the local mesh of panel p.
static Point vmed(const LoftPanel& p, size_t tvk, size_t bvk) {
    auto pt = *p.mesh.vertex_position(p.orig_top_to_local.at(tvk));
    auto pb = *p.mesh.vertex_position(p.orig_bot_to_local.at(bvk));
    return Point((pt[0]+pb[0])*0.5, (pt[1]+pb[1])*0.5, (pt[2]+pb[2])*0.5);
}

/// Creates a named layer node under the session root.
/// If r >= 0, sets the node color so the Rhino reader can apply it to the Rhino layer.
static std::shared_ptr<TreeNode> add_layer(Session& session, const std::string& n, int r = -1, int g = -1, int b = -1) {
    auto node = std::make_shared<TreeNode>(n);
    if (r >= 0) node->color = Color(r, g, b);
    session.add(node);
    return node;
}

static LoftPanel build_panel(size_t tfk, size_t bfk,
                              const Mesh& top_mesh, const Mesh& bot_mesh,
                              bool open_top = false,
                              bool skip_triangles = false,
                              double edge_gap = 0.0) {
    LoftPanel panel;
    panel.top_face_key = tfk;
    panel.bot_face_key = bfk;

    auto top_vkeys = *top_mesh.face_vertices(tfk);
    auto bot_vkeys = *bot_mesh.face_vertices(bfk);

    std::vector<Point> top_pts, bot_pts;
    for (auto vk : top_vkeys) top_pts.push_back(*top_mesh.vertex_position(vk));
    for (auto vk : bot_vkeys) bot_pts.push_back(*bot_mesh.vertex_position(vk));

    merge_collinear(top_pts, top_vkeys);
    merge_collinear(bot_pts, bot_vkeys);
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

    {
        std::vector<size_t> top_cap;
        for (auto vk : top_vkeys) top_cap.push_back(panel.orig_top_to_local[vk]);
        auto fk = panel.mesh.add_face(top_cap);
        if (fk) panel.top_face_key = *fk;
        if (fk && top_cap.size() >= 3) {
            auto [nx, ny, nz] = newell_normal(top_pts);
            double mag = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (mag > 1e-12) {
                nx /= mag; ny /= mag; nz /= mag;
                double ux = 1, uy = 0, uz = 0;
                if (std::abs(nx) > 0.9) { ux = 0; uy = 1; }
                double dot = ux*nx + uy*ny + uz*nz;
                ux -= dot*nx; uy -= dot*ny; uz -= dot*nz;
                double um = std::sqrt(ux*ux + uy*uy + uz*uz);
                ux /= um; uy /= um; uz /= um;
                double vx = ny*uz - nz*uy, vy = nz*ux - nx*uz, vz = nx*uy - ny*ux;
                std::vector<std::pair<double,double>> bpts;
                for (const auto& p : top_pts)
                    bpts.push_back({p[0]*ux + p[1]*uy + p[2]*uz, p[0]*vx + p[1]*vy + p[2]*vz});
                auto tris = cdt_triangulate(bpts, {});
                if (!tris.empty()) {
                    std::vector<std::array<size_t,3>> tri_list;
                    for (const auto& t : tris)
                        tri_list.push_back({top_cap[t[0]], top_cap[t[1]], top_cap[t[2]]});
                    panel.mesh.set_face_triangulation(*fk, std::move(tri_list));
                }
            }
        }
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
            std::optional<size_t> fk;
            if (edge_gap > 0.0) {
                auto pb0 = *panel.mesh.vertex_position(b0);
                auto pb1 = *panel.mesh.vertex_position(b1);
                auto pt0 = *panel.mesh.vertex_position(t0);
                auto pt1 = *panel.mesh.vertex_position(t1);
                double cx=(pb0[0]+pb1[0]+pt0[0]+pt1[0])*0.25;
                double cy=(pb0[1]+pb1[1]+pt0[1]+pt1[1])*0.25;
                double cz=(pb0[2]+pb1[2]+pt0[2]+pt1[2])*0.25;
                size_t nb0 = panel.mesh.add_vertex(offset_toward(pb0, cx, cy, cz, edge_gap));
                size_t nb1 = panel.mesh.add_vertex(offset_toward(pb1, cx, cy, cz, edge_gap));
                fk = panel.mesh.add_face({nb0, nb1, t1, t0});
            } else {
                fk = panel.mesh.add_face({b0, b1, t1, t0});
            }
            if (fk) {
                WallFaceInfo w; w.face_key = *fk; w.is_quad = true;
                w.top_v0 = top_vkeys[ti]; w.top_v1 = top_vkeys[(ti+1)%n];
                w.bot_v0 = bot_vkeys[j];  w.bot_v1 = bot_vkeys[(j+1)%m];
                panel.wall_faces.push_back(w);
            }
            top_used[ti] = true;
        } else if (!skip_triangles) {
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

    if (!skip_triangles) {
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
    }

    if (!open_top) {
        std::vector<size_t> bot_cap;
        for (int j = 0; j < m; j++)
            bot_cap.push_back(panel.orig_bot_to_local[bot_vkeys[j]]);
        panel.mesh.add_face(bot_cap);
    }

    panel.mesh.unify_winding();
    return panel;
}

static NurbsCurve make_polyline_loop(const std::vector<Point>& pts) {
    int n = (int)pts.size();
    NurbsCurve crv(3, false, 2, n+1);
    for (int k = 0; k <= n; k++) crv.set_knot(k, (double)k);
    for (int k = 0; k < n; k++) crv.set_cv(k, pts[k]);
    crv.set_cv(n, pts[0]);
    return crv;
}

static std::vector<NurbsCurve> make_wall_circles(
    const Point& pt0,
    const Point& pb0, const Point& pb1,
    const Point& a, const Point& b,
    double circle_rad, double division_dist)
{
    double e1x=pb1[0]-pb0[0], e1y=pb1[1]-pb0[1], e1z=pb1[2]-pb0[2];
    double e2x=pt0[0]-pb0[0], e2y=pt0[1]-pb0[1], e2z=pt0[2]-pb0[2];
    double nx=e1y*e2z-e1z*e2y, ny=e1z*e2x-e1x*e2z, nz=e1x*e2y-e1y*e2x;
    double nlen=std::sqrt(nx*nx+ny*ny+nz*nz);
    std::vector<NurbsCurve> circles;
    if (nlen < 1e-12) return circles;
    nx/=nlen; ny/=nlen; nz/=nlen;
    double dx=b[0]-a[0], dy=b[1]-a[1], dz=b[2]-a[2];
    double dlen=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (dlen < 1e-12) return circles;
    dx/=dlen; dy/=dlen; dz/=dlen;
    double tx=ny*dz-nz*dy, ty=nz*dx-nx*dz, tz=nx*dy-ny*dx;
    int ndiv = std::max(1, (int)std::round(dlen / division_dist));
    for (int i = 1; i < ndiv; i++) {
        double t = dlen * i / ndiv;
        Point ctr(a[0]+t*dx, a[1]+t*dy, a[2]+t*dz);
        const double sw = std::sqrt(2.0) / 2.0;
        double cxs[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
        double cys[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
        double wts[] = {1, sw, 1, sw, 1, sw, 1, sw, 1};
        NurbsCurve crv(3, true, 3, 9);
        double knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
        for (int k = 0; k < 10; k++) crv.set_knot(k, knots[k]);
        for (int k = 0; k < 9; k++) {
            double px = ctr[0] + circle_rad*(cxs[k]*dx + cys[k]*tx);
            double py = ctr[1] + circle_rad*(cxs[k]*dy + cys[k]*ty);
            double pz = ctr[2] + circle_rad*(cxs[k]*dz + cys[k]*tz);
            crv.set_cv_4d(k, px*wts[k], py*wts[k], pz*wts[k], wts[k]);
        }
        circles.push_back(crv);
    }
    return circles;
}

static void run_dataset(const std::vector<Polyline>& top_raw, const std::vector<Polyline>& bot_raw,
                        const std::string& name, const std::string& sdir,
                        bool open_top = false, bool skip_triangles = false, double edge_gap = 0.0) {
    auto top_pts = to_polys(top_raw);
    auto bot_pts = to_polys(bot_raw);

    Mesh top_mesh, bot_mesh;
    TIMED(ms_from_polylines,
        top_mesh = Mesh::from_polylines(top_pts, 0.001);
        bot_mesh = Mesh::from_polylines(bot_pts, 0.001);
    );

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
    TIMED(ms_build_panel,
        for (auto& [tfk, bfk] : face_match)
            panels.push_back(build_panel(tfk, bfk, top_mesh, bot_mesh, open_top, skip_triangles, edge_gap));
    );

    std::vector<LoftPanel> orig_panels;
    TIMED(ms_build_panel,
        for (auto& [tfk, bfk] : face_match)
            orig_panels.push_back(build_panel(tfk, bfk, top_mesh, bot_mesh));
    );

    Session session(name);
    auto layer_bot    = add_layer(session, "bottom mesh",              200, 100,  50);
    auto layer_top    = add_layer(session, "top mesh",                  50, 150, 220);
    auto layer_closed = add_layer(session, "loft closed meshes",       180, 180, 180);
    auto layer_folded = add_layer(session, "folded meshes with holes", 240, 160,  40);
    session.add(session.add_mesh(std::make_shared<Mesh>(top_mesh)), layer_bot);
    session.add(session.add_mesh(std::make_shared<Mesh>(bot_mesh)), layer_top);
    for (auto& p : orig_panels)
        session.add(session.add_mesh(std::make_shared<Mesh>(p.mesh)), layer_closed);

    // Build bot mesh edge map: interior edges shared by two panels use the averaged centerline
    using BotEdge = std::pair<size_t, size_t>; // canonical (min, max) orig bot vertex keys
    std::map<BotEdge, std::pair<Point,Point>> interior_line; // pre-computed averaged centerline
    {
        std::map<BotEdge, std::vector<std::pair<size_t, const WallFaceInfo*>>> bot_edge_map;
        for (size_t pi = 0; pi < panels.size(); pi++)
            for (const auto& w : panels[pi].wall_faces) {
                if (!w.is_quad) continue;
                bot_edge_map[{std::min(w.bot_v0,w.bot_v1), std::max(w.bot_v0,w.bot_v1)}].push_back({pi, &w});
            }
        for (const auto& [be, entries] : bot_edge_map) {
            if (entries.size() < 2) continue;
            const LoftPanel& p0 = panels[entries[0].first]; const WallFaceInfo* w0 = entries[0].second;
            const LoftPanel& p1 = panels[entries[1].first]; const WallFaceInfo* w1 = entries[1].second;
            Point p0a = vmed(p0, w0->top_v0, w0->bot_v0), p0b = vmed(p0, w0->top_v1, w0->bot_v1);
            Point p1a, p1b;
            if (w1->bot_v0 == w0->bot_v0) {
                p1a = vmed(p1, w1->top_v0, w1->bot_v0); p1b = vmed(p1, w1->top_v1, w1->bot_v1);
            } else {
                p1a = vmed(p1, w1->top_v1, w1->bot_v1); p1b = vmed(p1, w1->top_v0, w1->bot_v0);
            }
            interior_line[be] = {
                Point((p0a[0]+p1a[0])*0.5, (p0a[1]+p1a[1])*0.5, (p0a[2]+p1a[2])*0.5),
                Point((p0b[0]+p1b[0])*0.5, (p0b[1]+p1b[1])*0.5, (p0b[2]+p1b[2])*0.5)
            };
        }
    }

    const double division_dist = 100.0; // target spacing between circle centres
    const double circle_rad    = 5.0; // circle radius

    for (auto& panel : panels) {
        auto mesh_node = session.add_mesh(std::make_shared<Mesh>(panel.mesh));
        session.add(mesh_node, layer_folded);
        for (const auto& w : panel.wall_faces) {
            if (!w.is_quad) continue;
            auto pt0 = *panel.mesh.vertex_position(panel.orig_top_to_local.at(w.top_v0));
            auto pb0 = *panel.mesh.vertex_position(panel.orig_bot_to_local.at(w.bot_v0));
            auto pb1 = *panel.mesh.vertex_position(panel.orig_bot_to_local.at(w.bot_v1));
            BotEdge be{std::min(w.bot_v0,w.bot_v1), std::max(w.bot_v0,w.bot_v1)};
            Point a, b;
            auto it = interior_line.find(be);
            if (it != interior_line.end()) {
                a = it->second.first; b = it->second.second;
            } else {
                a = vmed(panel, w.top_v0, w.bot_v0); b = vmed(panel, w.top_v1, w.bot_v1);
            }
            session.add(session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{a, b})), mesh_node);
            for (const auto& crv : make_wall_circles(pt0, pb0, pb1, a, b, circle_rad, division_dist))
                session.add(session.add_nurbscurve(std::make_shared<NurbsCurve>(crv)), mesh_node);
        }
    }

    auto layer_brep = add_layer(session, "brep panels", 100, 200, 120);
    for (auto& panel : panels) {
        std::vector<NurbsCurve> outer_curves;
        std::vector<std::vector<NurbsCurve>> hole_curves;
        {
            auto cap_lkeys = *panel.mesh.face_vertices(panel.top_face_key);
            std::vector<Point> cap_pts;
            for (auto lk : cap_lkeys) cap_pts.push_back(*panel.mesh.vertex_position(lk));
            outer_curves.push_back(make_polyline_loop(cap_pts));
            hole_curves.push_back({});
        }
        for (const auto& w : panel.wall_faces) {
            if (!w.is_quad) continue;
            auto face_lkeys = *panel.mesh.face_vertices(w.face_key);
            std::vector<Point> face_pts;
            for (auto lk : face_lkeys) face_pts.push_back(*panel.mesh.vertex_position(lk));
            outer_curves.push_back(make_polyline_loop(face_pts));
            auto pt0 = *panel.mesh.vertex_position(panel.orig_top_to_local.at(w.top_v0));
            auto pb0 = *panel.mesh.vertex_position(panel.orig_bot_to_local.at(w.bot_v0));
            auto pb1 = *panel.mesh.vertex_position(panel.orig_bot_to_local.at(w.bot_v1));
            BotEdge be{std::min(w.bot_v0,w.bot_v1), std::max(w.bot_v0,w.bot_v1)};
            Point a, b;
            auto it = interior_line.find(be);
            if (it != interior_line.end()) {
                a = it->second.first; b = it->second.second;
            } else {
                a = vmed(panel, w.top_v0, w.bot_v0); b = vmed(panel, w.top_v1, w.bot_v1);
            }
            hole_curves.push_back(make_wall_circles(pt0, pb0, pb1, a, b, circle_rad, division_dist));
        }
        BRep brep;
        TIMED(ms_brep, brep = BRep::from_nurbscurves(outer_curves, hole_curves));
        auto brep_node = session.add_brep(std::make_shared<BRep>(brep));
        session.add(brep_node, layer_brep);
    }
    std::string filepath = (std::filesystem::path(sdir) / (name + "_out.pb")).string();
    TIMED(ms_pb_dump, session.pb_dump(filepath));
    std::cout << "Saved " << filepath << "\n";
}

int main() {
    std::string sdir = (std::filesystem::path(__FILE__).parent_path().parent_path()
                        / "session_data").string();
    auto t_main = std::chrono::high_resolution_clock::now();
    for (int i = 0; i <= 12; i++) {
        std::string name = "mesh_quad_tri_loft" + std::to_string(i);
        auto [top, bot] = load_polys_from_pb(sdir + "/" + name + ".pb");
        run_dataset(top, bot, name, sdir, true, true, 0.1);
    }
    double total = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now() - t_main).count();
    std::cout << "\n=== Profile (all 12 datasets) ===\n"
              << "  from_polylines : " << (int)ms_from_polylines << " ms\n"
              << "  build_panel    : " << (int)ms_build_panel    << " ms\n"
              << "  brep           : " << (int)ms_brep           << " ms\n"
              << "  pb_dump        : " << (int)ms_pb_dump        << " ms\n"
              << "  total          : " << (int)total             << " ms\n";
    return 0;
}
