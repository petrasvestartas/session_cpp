#include "closest.h"
#include "nurbscurve.h"
#include "mesh.h"
#include "pointcloud.h"
#include "bvh.h"
#include "aabb.h"
#include "obj.h"
#include <cstdio>
#include <queue>
#include <random>

using namespace session_cpp;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
struct Timer {
    std::chrono::high_resolution_clock::time_point t0;
    void start() { t0 = std::chrono::high_resolution_clock::now(); }
    double ms() const {
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();
    }
};

static BvhAABB make_aabb(double x0, double y0, double z0,
                          double x1, double y1, double z1) {
    constexpr double eps = 0.001;
    double lx = std::min(x0, x1) - eps, hx = std::max(x0, x1) + eps;
    double ly = std::min(y0, y1) - eps, hy = std::max(y0, y1) + eps;
    double lz = std::min(z0, z1) - eps, hz = std::max(z0, z1) + eps;
    return {(lx+hx)*0.5, (ly+hy)*0.5, (lz+hz)*0.5,
            (hx-lx)*0.5, (hy-ly)*0.5, (hz-lz)*0.5};
}

static BvhAABB line_aabb(const Line& l) {
    return make_aabb(l.start()[0], l.start()[1], l.start()[2],
                     l.end()[0],   l.end()[1],   l.end()[2]);
}

static BvhAABB polyline_aabb(const Polyline& pl) {
    auto pts = pl.get_points();
    double lx=1e308, ly=1e308, lz=1e308, hx=-1e308, hy=-1e308, hz=-1e308;
    for (const auto& p : pts) {
        lx = std::min(lx, p[0]); hx = std::max(hx, p[0]);
        ly = std::min(ly, p[1]); hy = std::max(hy, p[1]);
        lz = std::min(lz, p[2]); hz = std::max(hz, p[2]);
    }
    return make_aabb(lx, ly, lz, hx, hy, hz);
}

static BvhAABB curve_aabb(const NurbsCurve& c) {
    double lx=1e308, ly=1e308, lz=1e308, hx=-1e308, hy=-1e308, hz=-1e308;
    for (int i = 0; i < c.cv_count(); i++) {
        Point p = c.get_cv(i);
        lx = std::min(lx, p[0]); hx = std::max(hx, p[0]);
        ly = std::min(ly, p[1]); hy = std::max(hy, p[1]);
        lz = std::min(lz, p[2]); hz = std::max(hz, p[2]);
    }
    return make_aabb(lx, ly, lz, hx, hy, hz);
}

static double aabb_min_dist(const BvhAABB& b, const Point& p) {
    double dx = std::max(0.0, std::max(b.cx-b.hx - p[0], p[0] - b.cx-b.hx));
    double dy = std::max(0.0, std::max(b.cy-b.hy - p[1], p[1] - b.cy-b.hy));
    double dz = std::max(0.0, std::max(b.cz-b.hz - p[2], p[2] - b.cz-b.hz));
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static double compute_ws(const std::vector<BvhAABB>& aabbs) {
    double mx = 0;
    for (const auto& b : aabbs) {
        mx = std::max(mx, std::fabs(b.cx-b.hx));
        mx = std::max(mx, std::fabs(b.cx+b.hx));
        mx = std::max(mx, std::fabs(b.cy-b.hy));
        mx = std::max(mx, std::fabs(b.cy+b.hy));
        mx = std::max(mx, std::fabs(b.cz-b.hz));
        mx = std::max(mx, std::fabs(b.cz+b.hz));
    }
    return std::max(2.2 * mx, 10.0);
}

template<typename DistFn>
static int bvh_closest(const BVH& bvh, const Point& tp, DistFn&& fn, double& best) {
    if (!bvh.root) return -1;
    int best_id = -1;
    using E = std::pair<double, const BVHNode*>;
    std::priority_queue<E, std::vector<E>, std::greater<E>> pq;
    pq.push({aabb_min_dist(bvh.root->aabb, tp), bvh.root});
    while (!pq.empty()) {
        auto [d, n] = pq.top(); pq.pop();
        if (d >= best) break;
        if (n->is_leaf()) {
            double dist = fn(n->object_id, tp);
            if (dist < best) { best = dist; best_id = n->object_id; }
        } else {
            if (n->left)  { double ld = aabb_min_dist(n->left->aabb,  tp); if (ld < best) pq.push({ld, n->left});  }
            if (n->right) { double rd = aabb_min_dist(n->right->aabb, tp); if (rd < best) pq.push({rd, n->right}); }
        }
    }
    return best_id;
}

template<typename DistFn>
static void aabb_closest(const AABBTree& tree, int ni, const Point& tp,
                         DistFn&& fn, int& best_id, double& best) {
    const auto& nd = tree.nodes[ni];
    if (aabb_min_dist(nd.aabb, tp) >= best) return;
    if (nd.object_id >= 0) {
        double d = fn(nd.object_id, tp);
        if (d < best) { best = d; best_id = nd.object_id; }
        return;
    }
    int left = ni + 1, right = nd.right;
    double ld = aabb_min_dist(tree.nodes[left].aabb,  tp);
    double rd = aabb_min_dist(tree.nodes[right].aabb, tp);
    if (ld <= rd) {
        if (ld < best) aabb_closest(tree, left,  tp, fn, best_id, best);
        if (rd < best) aabb_closest(tree, right, tp, fn, best_id, best);
    } else {
        if (rd < best) aabb_closest(tree, right, tp, fn, best_id, best);
        if (ld < best) aabb_closest(tree, left,  tp, fn, best_id, best);
    }
}

static void header(const char* title, int n_obj, int n_qry) {
    std::printf("\n=== %s (%d objects, %d queries) ===\n", title, n_obj, n_qry);
}

static void row(const char* label, double build_ms, double query_ms) {
    std::printf("  %-12s  build %8.2f ms | query %8.2f ms | total %8.2f ms\n",
                label, build_ms, query_ms, build_ms + query_ms);
}

static int verify(const char* label, const std::vector<double>& ref,
                  const std::vector<double>& test, double tol = 1e-4) {
    int bad = 0;
    double max_err = 0;
    for (size_t i = 0; i < ref.size(); i++) {
        double err = std::fabs(ref[i] - test[i]);
        if (err > tol) { bad++; max_err = std::max(max_err, err); }
    }
    if (bad == 0)
        std::printf("  %-12s  PASS  all %zu match (tol %.0e)\n", label, ref.size(), tol);
    else
        std::printf("  %-12s  FAIL  %d/%zu mismatch  max_err=%.6f\n", label, bad, ref.size(), max_err);
    return bad;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    std::mt19937 rng(42);
    Timer timer;
    int total_fail = 0;

    // -----------------------------------------------------------------------
    // 1. LINES  (10 000 lines, 1000 queries)
    // -----------------------------------------------------------------------
    constexpr int N_LINES = 500, Q_LINES = 100;
    std::uniform_real_distribution<double> ld(-500.0, 500.0);
    std::vector<Line> lines(N_LINES);
    for (int i = 0; i < N_LINES; i++)
        lines[i] = Line(ld(rng), ld(rng), ld(rng), ld(rng), ld(rng), ld(rng));

    std::vector<Point> lq(Q_LINES);
    for (int i = 0; i < Q_LINES; i++) lq[i] = Point(ld(rng), ld(rng), ld(rng));

    auto line_dist = [&](int id, const Point& tp) {
        auto [cp, t, d] = Closest::line_point(lines[id], tp);
        return d;
    };

    // brute force — store results
    std::vector<double> bf_line_d(Q_LINES);
    timer.start();
    for (int i = 0; i < Q_LINES; i++) {
        double best = 1e308;
        for (int j = 0; j < N_LINES; j++) { double d = line_dist(j, lq[i]); if (d < best) best = d; }
        bf_line_d[i] = best;
    }
    double bf_lines = timer.ms();

    std::vector<BvhAABB> la(N_LINES);
    for (int i = 0; i < N_LINES; i++) la[i] = line_aabb(lines[i]);

    // BVH
    std::vector<double> bvh_line_d(Q_LINES);
    timer.start();
    BVH bvh_l;
    bvh_l.build_from_aabbs(la.data(), la.size(), compute_ws(la));
    double bvh_lb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_LINES; i++) { double best = 1e308; bvh_closest(bvh_l, lq[i], line_dist, best); bvh_line_d[i] = best; }
    double bvh_lq = timer.ms();

    // AABB
    std::vector<double> aabb_line_d(Q_LINES);
    timer.start();
    AABBTree aabb_l;
    aabb_l.build(la.data(), la.size());
    double aabb_lb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_LINES; i++) { double best = 1e308; int id = -1; aabb_closest(aabb_l, 0, lq[i], line_dist, id, best); aabb_line_d[i] = best; }
    double aabb_lq = timer.ms();

    header("Lines", N_LINES, Q_LINES);
    row("Brute", 0, bf_lines);
    row("BVH", bvh_lb, bvh_lq);
    row("AABB", aabb_lb, aabb_lq);
    total_fail += verify("BVH", bf_line_d, bvh_line_d);
    total_fail += verify("AABB", bf_line_d, aabb_line_d);

    // -----------------------------------------------------------------------
    // 2. POLYLINES  (5000 polylines × 8 pts, 1000 queries)
    // -----------------------------------------------------------------------
    constexpr int N_PL = 500, Q_PL = 100, PL_PTS = 8;
    std::vector<Polyline> pls(N_PL);
    for (int i = 0; i < N_PL; i++) {
        std::vector<Point> pts(PL_PTS);
        double bx = ld(rng), by = ld(rng), bz = ld(rng);
        for (int j = 0; j < PL_PTS; j++)
            pts[j] = Point(bx + ld(rng)*0.1, by + ld(rng)*0.1, bz + ld(rng)*0.1);
        pls[i] = Polyline(pts);
    }

    std::vector<Point> pq(Q_PL);
    for (int i = 0; i < Q_PL; i++) pq[i] = Point(ld(rng), ld(rng), ld(rng));

    auto pl_dist = [&](int id, const Point& tp) {
        auto [cp, t, d] = Closest::polyline_point(pls[id], tp);
        return d;
    };

    std::vector<double> bf_pl_d(Q_PL);
    timer.start();
    for (int i = 0; i < Q_PL; i++) {
        double best = 1e308;
        for (int j = 0; j < N_PL; j++) { double d = pl_dist(j, pq[i]); if (d < best) best = d; }
        bf_pl_d[i] = best;
    }
    double bf_pl = timer.ms();

    std::vector<BvhAABB> pa(N_PL);
    for (int i = 0; i < N_PL; i++) pa[i] = polyline_aabb(pls[i]);

    std::vector<double> bvh_pl_d(Q_PL);
    timer.start();
    BVH bvh_p;
    bvh_p.build_from_aabbs(pa.data(), pa.size(), compute_ws(pa));
    double bvh_pb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_PL; i++) { double best = 1e308; bvh_closest(bvh_p, pq[i], pl_dist, best); bvh_pl_d[i] = best; }
    double bvh_pq = timer.ms();

    std::vector<double> aabb_pl_d(Q_PL);
    timer.start();
    AABBTree aabb_p;
    aabb_p.build(pa.data(), pa.size());
    double aabb_pb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_PL; i++) { double best = 1e308; int id = -1; aabb_closest(aabb_p, 0, pq[i], pl_dist, id, best); aabb_pl_d[i] = best; }
    double aabb_pq = timer.ms();

    header("Polylines", N_PL, Q_PL);
    row("Brute", 0, bf_pl);
    row("BVH", bvh_pb, bvh_pq);
    row("AABB", aabb_pb, aabb_pq);
    total_fail += verify("BVH", bf_pl_d, bvh_pl_d);
    total_fail += verify("AABB", bf_pl_d, aabb_pl_d);

    // -----------------------------------------------------------------------
    // 3. NURBS CURVES  (500 curves × degree 3, 200 queries)
    // -----------------------------------------------------------------------
    constexpr int N_CRV = 100, Q_CRV = 50;
    std::vector<NurbsCurve> crvs(N_CRV);
    for (int i = 0; i < N_CRV; i++) {
        double bx = ld(rng), by = ld(rng), bz = ld(rng);
        std::vector<Point> pts(5);
        for (int j = 0; j < 5; j++)
            pts[j] = Point(bx + j*10.0 + ld(rng)*2.0, by + ld(rng)*5.0, bz + ld(rng)*5.0);
        crvs[i] = NurbsCurve::create(false, 3, pts);
    }

    std::vector<Point> cq(Q_CRV);
    for (int i = 0; i < Q_CRV; i++) cq[i] = Point(ld(rng), ld(rng), ld(rng));

    auto crv_dist = [&](int id, const Point& tp) {
        auto [t, d] = Closest::curve_point(crvs[id], tp);
        return d;
    };

    // brute force
    std::vector<double> bf_crv_d(Q_CRV);
    std::vector<int> bf_crv_id(Q_CRV);
    timer.start();
    for (int i = 0; i < Q_CRV; i++) {
        double best = 1e308; int bi = -1;
        for (int j = 0; j < N_CRV; j++) { double d = crv_dist(j, cq[i]); if (d < best) { best = d; bi = j; } }
        bf_crv_d[i] = best; bf_crv_id[i] = bi;
    }
    double bf_crv = timer.ms();

    // per-curve AABBs (shared by BVH and AABB)
    std::vector<BvhAABB> ca(N_CRV);
    for (int i = 0; i < N_CRV; i++) ca[i] = curve_aabb(crvs[i]);

    // BVH per whole curve
    std::vector<double> bvh_crv_d(Q_CRV);
    timer.start();
    BVH bvh_c;
    bvh_c.build_from_aabbs(ca.data(), ca.size(), compute_ws(ca));
    double bvh_cb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_CRV; i++) { double best = 1e308; bvh_closest(bvh_c, cq[i], crv_dist, best); bvh_crv_d[i] = best; }
    double bvh_cq = timer.ms();

    // AABB per whole curve
    std::vector<double> aabb_crv_d(Q_CRV);
    timer.start();
    AABBTree aabb_c;
    aabb_c.build(ca.data(), ca.size());
    double aabb_cb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_CRV; i++) { double best = 1e308; int id = -1; aabb_closest(aabb_c, 0, cq[i], crv_dist, id, best); aabb_crv_d[i] = best; }
    double aabb_cq = timer.ms();

    // Helper: build proxy segments + AABB tree from a list of (curve_id, points) pairs
    struct ProxySeg { int curve_id; };
    auto build_proxy = [&](auto& segs, auto& aabbs, auto& tree,
                           const std::vector<std::pair<int, std::vector<Point>>>& curve_pts) {
        segs.clear(); aabbs.clear();
        for (const auto& [cid, pts] : curve_pts) {
            for (size_t j = 1; j < pts.size(); j++) {
                segs.push_back({cid});
                aabbs.push_back(make_aabb(
                    std::min(pts[j-1][0], pts[j][0]), std::min(pts[j-1][1], pts[j][1]), std::min(pts[j-1][2], pts[j][2]),
                    std::max(pts[j-1][0], pts[j][0]), std::max(pts[j-1][1], pts[j][1]), std::max(pts[j-1][2], pts[j][2])));
            }
        }
        tree.build(aabbs.data(), aabbs.size());
    };

    // Query helper: traverse tree with per-curve cache
    auto query_proxy = [&](const AABBTree& tree, const std::vector<ProxySeg>& segs,
                           std::vector<double>& results) {
        for (int i = 0; i < Q_CRV; i++) {
            double best = 1e308;
            int best_seg = -1;
            std::unordered_map<int, double> cache;
            auto refine = [&](int seg_id, const Point& tp) -> double {
                int cid = segs[seg_id].curve_id;
                auto it = cache.find(cid);
                if (it != cache.end()) return it->second;
                auto [t, d] = Closest::curve_point(crvs[cid], tp);
                cache[cid] = d;
                return d;
            };
            aabb_closest(tree, 0, cq[i], refine, best_seg, best);
            results[i] = best;
        }
    };

    // 1) Uniform 20/curve
    std::vector<ProxySeg> u20_segs;
    std::vector<BvhAABB> u20_aabbs;
    AABBTree u20_tree;
    timer.start();
    {
        std::vector<std::pair<int, std::vector<Point>>> pts(N_CRV);
        for (int i = 0; i < N_CRV; i++) {
            auto [d0, d1] = crvs[i].domain();
            auto& [cid, pv] = pts[i]; cid = i;
            pv.resize(21);
            for (int j = 0; j <= 20; j++) pv[j] = crvs[i].point_at(d0 + (d1-d0)*j/20.0);
        }
        build_proxy(u20_segs, u20_aabbs, u20_tree, pts);
    }
    double u20_cb = timer.ms();
    std::vector<double> u20_d(Q_CRV);
    timer.start(); query_proxy(u20_tree, u20_segs, u20_d); double u20_cq = timer.ms();

    // 2) Hybrid: CV complexity → variable sample count (uniform per curve)
    //    Metric: control_polygon_length / chord_length  (O(n_cv), no curve eval)
    //    ratio ≈ 1 → nearly straight → 5 segs
    //    ratio > 3 → very curvy      → 100 segs
    //    Linear interpolation, clamped [5, 100]
    std::vector<ProxySeg> hyb_segs;
    std::vector<BvhAABB> hyb_aabbs;
    AABBTree hyb_tree;
    std::vector<int> hyb_n_per(N_CRV);
    timer.start();
    {
        std::vector<std::pair<int, std::vector<Point>>> pts(N_CRV);
        for (int i = 0; i < N_CRV; i++) {
            int ncv = crvs[i].cv_count();
            double poly_len = 0;
            for (int j = 1; j < ncv; j++)
                poly_len += crvs[i].get_cv(j).distance(crvs[i].get_cv(j-1));
            double chord = crvs[i].get_cv(0).distance(crvs[i].get_cv(ncv-1));
            double ratio = (chord > 1e-10) ? poly_len / chord : 3.0;
            // map ratio [1, 3] → samples [10, 80]
            int n = 5 + static_cast<int>((std::clamp(ratio, 1.0, 3.0) - 1.0) / 2.0 * 95.0);
            hyb_n_per[i] = n;
            auto [d0, d1] = crvs[i].domain();
            auto& [cid, pv] = pts[i]; cid = i;
            pv.resize(n + 1);
            for (int j = 0; j <= n; j++) pv[j] = crvs[i].point_at(d0 + (d1-d0)*j/(double)n);
        }
        build_proxy(hyb_segs, hyb_aabbs, hyb_tree, pts);
    }
    double hyb_cb = timer.ms();
    std::vector<double> hyb_d(Q_CRV);
    timer.start(); query_proxy(hyb_tree, hyb_segs, hyb_d); double hyb_cq = timer.ms();

    // stats for hybrid
    int hyb_min = *std::min_element(hyb_n_per.begin(), hyb_n_per.end());
    int hyb_max = *std::max_element(hyb_n_per.begin(), hyb_n_per.end());

    // 3) Uniform 50/curve
    std::vector<ProxySeg> u50_segs;
    std::vector<BvhAABB> u50_aabbs;
    AABBTree u50_tree;
    timer.start();
    {
        std::vector<std::pair<int, std::vector<Point>>> pts(N_CRV);
        for (int i = 0; i < N_CRV; i++) {
            auto [d0, d1] = crvs[i].domain();
            auto& [cid, pv] = pts[i]; cid = i;
            pv.resize(51);
            for (int j = 0; j <= 50; j++) pv[j] = crvs[i].point_at(d0 + (d1-d0)*j/50.0);
        }
        build_proxy(u50_segs, u50_aabbs, u50_tree, pts);
    }
    double u50_cb = timer.ms();
    std::vector<double> u50_d(Q_CRV);
    timer.start(); query_proxy(u50_tree, u50_segs, u50_d); double u50_cq = timer.ms();

    int n_u20 = static_cast<int>(u20_segs.size());
    int n_hyb = static_cast<int>(hyb_segs.size());
    int n_u50 = static_cast<int>(u50_segs.size());
    header("NurbsCurves", N_CRV, Q_CRV);
    row("Brute", 0, bf_crv);
    row("BVH", bvh_cb, bvh_cq);
    row("AABB", aabb_cb, aabb_cq);
    std::printf("  --- proxy+cache (AABB tree, per-curve Newton) ---\n");
    std::printf("  Uniform 20/crv (%d segs):\n", n_u20);
    row("U20+AABB", u20_cb, u20_cq);
    std::printf("  Hybrid CV-ratio (%d segs, %.1f avg/crv, range %d-%d):\n",
                n_hyb, n_hyb/(double)N_CRV, hyb_min, hyb_max);
    row("Hybrid+AABB", hyb_cb, hyb_cq);
    std::printf("  Uniform 50/crv (%d segs):\n", n_u50);
    row("U50+AABB", u50_cb, u50_cq);
    total_fail += verify("BVH", bf_crv_d, bvh_crv_d);
    total_fail += verify("AABB", bf_crv_d, aabb_crv_d);
    total_fail += verify("U20+AABB", bf_crv_d, u20_d);
    total_fail += verify("Hybrid+AABB", bf_crv_d, hyb_d);
    total_fail += verify("U50+AABB", bf_crv_d, u50_d);

    // -----------------------------------------------------------------------
    // 4. MESH — bunny.obj  (1000 queries)
    // -----------------------------------------------------------------------
    constexpr int Q_MESH = 100;
    std::string bunny_path = (std::filesystem::path(__FILE__).parent_path().parent_path()
                              / "session_data" / "bunny.obj").string();
    Mesh bunny = obj::read_obj(bunny_path);
    std::printf("\n  Bunny: %zu vertices, %zu faces\n",
                bunny.number_of_vertices(), bunny.number_of_faces());

    std::uniform_real_distribution<double> md(-0.2, 0.2);
    std::vector<Point> mq(Q_MESH);
    for (int i = 0; i < Q_MESH; i++) mq[i] = Point(md(rng), md(rng), md(rng));

    std::vector<double> bvh_mesh_d(Q_MESH), aabb_mesh_d(Q_MESH);

    timer.start();
    for (int i = 0; i < Q_MESH; i++) { auto [cp, fk, d] = Closest::mesh_point(bunny, mq[i]); bvh_mesh_d[i] = d; }
    double bvh_mesh = timer.ms();

    timer.start();
    for (int i = 0; i < Q_MESH; i++) { auto [cp, fk, d] = Closest::mesh_point_aabb(bunny, mq[i]); aabb_mesh_d[i] = d; }
    double aabb_mesh = timer.ms();

    header("Mesh (bunny)", (int)bunny.number_of_faces(), Q_MESH);
    row("BVH", 0, bvh_mesh);
    row("AABB", 0, aabb_mesh);
    total_fail += verify("AABB vs BVH", bvh_mesh_d, aabb_mesh_d);

    // -----------------------------------------------------------------------
    // 5. POINTCLOUD  (100 000 points, 1000 queries)
    // -----------------------------------------------------------------------
    constexpr int N_PC = 5000, Q_PC = 100;
    std::vector<Point> cloud(N_PC);
    for (int i = 0; i < N_PC; i++) cloud[i] = Point(ld(rng), ld(rng), ld(rng));
    PointCloud pc(cloud, {}, {});

    std::vector<Point> pcq(Q_PC);
    for (int i = 0; i < Q_PC; i++) pcq[i] = Point(ld(rng), ld(rng), ld(rng));

    std::vector<double> bf_pc_d(Q_PC);
    timer.start();
    for (int i = 0; i < Q_PC; i++) { auto [cp, idx, d] = Closest::pointcloud_point(pc, pcq[i]); bf_pc_d[i] = d; }
    double bf_pc = timer.ms();

    std::vector<BvhAABB> pca(N_PC);
    for (int i = 0; i < N_PC; i++)
        pca[i] = {cloud[i][0], cloud[i][1], cloud[i][2], 0.001, 0.001, 0.001};

    auto pc_dist = [&](int id, const Point& tp) { return cloud[id].distance(tp); };

    std::vector<double> bvh_pc_d(Q_PC), aabb_pc_d(Q_PC);

    timer.start();
    BVH bvh_pc;
    bvh_pc.build_from_aabbs(pca.data(), pca.size(), compute_ws(pca));
    double bvh_pcb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_PC; i++) { double best = 1e308; bvh_closest(bvh_pc, pcq[i], pc_dist, best); bvh_pc_d[i] = best; }
    double bvh_pcq = timer.ms();

    timer.start();
    AABBTree aabb_pc;
    aabb_pc.build(pca.data(), pca.size());
    double aabb_pcb = timer.ms();
    timer.start();
    for (int i = 0; i < Q_PC; i++) { double best = 1e308; int id = -1; aabb_closest(aabb_pc, 0, pcq[i], pc_dist, id, best); aabb_pc_d[i] = best; }
    double aabb_pcq = timer.ms();

    header("PointCloud", N_PC, Q_PC);
    row("Brute", 0, bf_pc);
    row("BVH", bvh_pcb, bvh_pcq);
    row("AABB", aabb_pcb, aabb_pcq);
    total_fail += verify("BVH", bf_pc_d, bvh_pc_d);
    total_fail += verify("AABB", bf_pc_d, aabb_pc_d);

    // -----------------------------------------------------------------------
    std::printf("\n%s\n", total_fail == 0 ? "ALL VERIFIED OK" : "*** VERIFICATION FAILURES ***");
    return total_fail > 0 ? 1 : 0;
}
