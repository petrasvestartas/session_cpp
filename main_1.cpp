#include "session.h"

static double ms_build_panel = 0, ms_brep = 0, ms_pb_dump = 0;
#define TIMED(var, ...) do { \
    auto _t0 = std::chrono::high_resolution_clock::now(); \
    __VA_ARGS__; \
    var += std::chrono::duration<double,std::milli>( \
        std::chrono::high_resolution_clock::now() - _t0).count(); \
} while(0)

using namespace session_cpp;

// ── User-facing configuration ─────────────────────────────────────────────────
// Fraction of each polyline's bbox diagonal used as near-duplicate vertex
// removal threshold.  Keeps only genuine near-duplicates (≈0.01% of extent).
static constexpr double VERTEX_DEDUP_RATIO      = 1e-4;

// Hard upper bound on the near-duplicate removal threshold (model units).
// Prevents VERTEX_DEDUP_RATIO from becoming too large on big polygons and
// collapsing valid short edges.  Raise if near-duplicate artifacts in the
// input data are known to exceed this distance.
static constexpr double VERTEX_DEDUP_MAX_TOL    = 0.5;

// Precision used when building the shared-vertex mesh from the raw polylines.
// Two vertices whose coordinates round to the same grid cell of this size are
// merged into one.  Must be smaller than VERTEX_DEDUP_TOLERANCE.
static constexpr double VERTEX_MERGE_PRECISION  = 0.001;

// Target arc-length spacing between consecutive circle centres placed along
// each interior wall edge.
static constexpr double CIRCLE_DIVISION_DIST   = 100.0;

// Radius of each decorative circle arc placed on the wall faces.
static constexpr double CIRCLE_RADIUS          = 5.0;
// ─────────────────────────────────────────────────────────────────────────────

static std::pair<std::vector<Polyline>, std::vector<Polyline>> load_polys_from_pb(const std::string& filepath) {
    auto obj = Objects::pb_load(filepath);
    std::vector<Polyline> top, bot;
    for (const auto& pl : *obj.polylines) {
        if (pl->name.rfind("top_", 0) == 0)
            top.push_back(*pl);
        else
            bot.push_back(*pl);
    }
    return {top, bot};
}

static std::vector<std::vector<Point>> to_polys(const std::vector<Polyline>& polylines) {
    std::vector<std::vector<Point>> result;
    result.reserve(polylines.size());
    for (const auto& pl : polylines) {
        auto raw = pl.get_points();
        double lo[3]={1e300,1e300,1e300}, hi[3]={-1e300,-1e300,-1e300};
        for (auto& p : raw)
            for (int k=0;k<3;k++) { lo[k]=std::min(lo[k],p[k]); hi[k]=std::max(hi[k],p[k]); }
        double d = std::sqrt((hi[0]-lo[0])*(hi[0]-lo[0])+(hi[1]-lo[1])*(hi[1]-lo[1])+(hi[2]-lo[2])*(hi[2]-lo[2]));
        double tol = std::min(std::max(d * VERTEX_DEDUP_RATIO, 1e-10), VERTEX_DEDUP_MAX_TOL);
        if (raw.empty()) { result.push_back({}); continue; }
        std::vector<Point> clean = {raw[0]};
        for (size_t i = 1; i < raw.size(); i++)
            if (raw[i].distance(raw[i-1]) > tol)
                clean.push_back(raw[i]);
        while (clean.size() >= 3 && clean.back().distance(clean.front()) <= tol)
            clean.pop_back();
        result.push_back(std::move(clean));
    }
    return result;
}

/// Returns the midpoint of the vertical edge connecting top vertex tvk and bottom vertex bvk
/// within the local mesh of panel p.
static Point vmed(const LoftPanel& p, size_t tvk, size_t bvk) {
    auto pt = *p.mesh.vertex_point(p.orig_top_to_local.at(tvk));
    auto pb = *p.mesh.vertex_point(p.orig_bot_to_local.at(bvk));
    return Point((pt[0]+pb[0])*0.5, (pt[1]+pb[1])*0.5, (pt[2]+pb[2])*0.5);
}

/// Creates a named layer node under the session root.
/// If r >= 0, sets the node color so the Rhino reader can apply it to the Rhino layer.
static std::shared_ptr<TreeNode> add_group(Session& session, const std::string& n, int r = -1, int g = -1, int b = -1) {
    auto node = std::make_shared<TreeNode>(n);
    if (r >= 0) node->color = Color(r, g, b);
    session.add(node);
    return node;
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
                        const std::string& name, const std::string& sdir) {
    auto top_pts = to_polys(top_raw);
    auto bot_pts = to_polys(bot_raw);

    LoftResult loft_result;
    TIMED(ms_build_panel,
        loft_result = Mesh::loft_panels(top_pts, bot_pts, VERTEX_MERGE_PRECISION);
    );
    Mesh& top_mesh = loft_result.top_mesh;
    Mesh& bot_mesh = loft_result.bot_mesh;
    std::vector<LoftPanel>& panels = loft_result.panels;

    Session session(name);
    auto layer_bot    = add_group(session, "bottom mesh",              200, 100,  50);
    auto layer_top    = add_group(session, "top mesh",                  50, 150, 220);
    auto layer_closed = add_group(session, "loft closed meshes",       180, 180, 180);
    auto layer_folded = add_group(session, "folded meshes with holes", 240, 160,  40);
    session.add(session.add_mesh(std::make_shared<Mesh>(top_mesh)), layer_bot);
    session.add(session.add_mesh(std::make_shared<Mesh>(bot_mesh)), layer_top);
    for (auto& p : panels)
        session.add(session.add_mesh(std::make_shared<Mesh>(p.mesh)), layer_closed);

    // Build bot mesh edge map: interior edges shared by two panels use the averaged centerline
    using BotEdge = std::pair<size_t, size_t>; // canonical (min, max) orig bot vertex keys
    std::map<BotEdge, std::pair<Point,Point>> interior_line; // pre-computed averaged centerline
    {
        std::map<BotEdge, std::vector<std::pair<size_t, const LoftWallFace*>>> bot_edge_map;
        for (size_t pi = 0; pi < panels.size(); pi++)
            for (const auto& w : panels[pi].wall_faces) {
                if (!w.is_quad) continue;
                bot_edge_map[{std::min(w.bot_v0,w.bot_v1), std::max(w.bot_v0,w.bot_v1)}].push_back({pi, &w});
            }
        for (const auto& [be, entries] : bot_edge_map) {
            if (entries.size() < 2) continue;
            const LoftPanel& p0 = panels[entries[0].first]; const LoftWallFace* w0 = entries[0].second;
            const LoftPanel& p1 = panels[entries[1].first]; const LoftWallFace* w1 = entries[1].second;
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

    const double division_dist = CIRCLE_DIVISION_DIST;
    const double circle_rad    = CIRCLE_RADIUS;

    for (auto& panel : panels) {
        auto mesh_node = session.add_mesh(std::make_shared<Mesh>(panel.mesh));
        session.add(mesh_node, layer_folded);
        for (const auto& w : panel.wall_faces) {
            if (!w.is_quad) continue;
            auto pt0 = *panel.mesh.vertex_point(panel.orig_top_to_local.at(w.top_v0));
            auto pb0 = *panel.mesh.vertex_point(panel.orig_bot_to_local.at(w.bot_v0));
            auto pb1 = *panel.mesh.vertex_point(panel.orig_bot_to_local.at(w.bot_v1));
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

    auto layer_brep = add_group(session, "brep panels", 100, 200, 120);
    for (auto& panel : panels) {
        std::vector<NurbsCurve> outer_curves;
        std::vector<std::vector<NurbsCurve>> hole_curves;
        if (!panel.top_vertices.empty()) {
            std::vector<Point> pts;
            for (auto lk : panel.top_vertices) pts.push_back(*panel.mesh.vertex_point(lk));
            outer_curves.push_back(make_polyline_loop(pts));
            hole_curves.push_back({});
        }
        for (const auto& w : panel.wall_faces) {
            auto face_lkeys = *panel.mesh.face_vertices(w.face_key);
            std::vector<Point> face_pts;
            for (auto lk : face_lkeys) face_pts.push_back(*panel.mesh.vertex_point(lk));
            outer_curves.push_back(make_polyline_loop(face_pts));
            if (!w.is_quad) { hole_curves.push_back({}); continue; }
            auto pt0 = *panel.mesh.vertex_point(panel.orig_top_to_local.at(w.top_v0));
            auto pb0 = *panel.mesh.vertex_point(panel.orig_bot_to_local.at(w.bot_v0));
            auto pb1 = *panel.mesh.vertex_point(panel.orig_bot_to_local.at(w.bot_v1));
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
        if (!panel.bot_vertices.empty()) {
            std::vector<Point> pts;
            for (auto lk : panel.bot_vertices) pts.push_back(*panel.mesh.vertex_point(lk));
            outer_curves.push_back(make_polyline_loop(pts));
            hole_curves.push_back({});
        }
        BRep brep;
        TIMED(ms_brep, brep = BRep::from_nurbscurves(outer_curves, hole_curves));
        if (!brep.m_faces.empty()) {
            size_t nwall = outer_curves.size() - 2;
            brep.m_faces[0].facecolor = Color(0, 255, 0, 255);
            for (size_t fi = 1; fi <= nwall; fi++)
                brep.m_faces[fi].facecolor = Color(255, 255, 255, 255);
            brep.m_faces.back().facecolor = Color(255, 0, 0, 255);
        }
        auto brep_node = session.add_brep(std::make_shared<BRep>(brep));
        session.add(brep_node, layer_brep);
    }
    std::string filepath = (std::filesystem::path(sdir) / (name + "_out.pb")).string();
    TIMED(ms_pb_dump, session.pb_dump(filepath));
    std::cout << "Saved " << filepath << "\n";
}

int main() {
    // std::string sdir = (std::filesystem::path(__FILE__).parent_path().parent_path()
    //                     / "session_data").string();
    // auto t_main = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i <= 13; i++) {
    //     std::string name = "mesh_quad_tri_loft" + std::to_string(i);
    //     auto [top, bot] = load_polys_from_pb(sdir + "/" + name + ".pb");
    //     run_dataset(top, bot, name, sdir);
    // }
    // double total = std::chrono::duration<double,std::milli>(
    //     std::chrono::high_resolution_clock::now() - t_main).count();
    // std::cout << "\n=== Profile (all 12 datasets) ===\n"
    //           << "  build_panel    : " << (int)ms_build_panel    << " ms\n"
    //           << "  brep           : " << (int)ms_brep           << " ms\n"
    //           << "  pb_dump        : " << (int)ms_pb_dump        << " ms\n"
    //           << "  total          : " << (int)total             << " ms\n";
    return 0;
}
