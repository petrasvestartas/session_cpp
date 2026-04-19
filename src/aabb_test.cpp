#include "mini_test.h"
#include "aabb.h"
#include "closest.h"
#include "color.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "pointcloud.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include "vector.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("AABBTree", "Constructor") {
    // uncomment #include "aabb.h"
    // uncomment #include "closest.h"
    // AABBTree: O(n log n) build, O(log n) cull — prune candidates before exact test
    std::vector<AABB> boxes = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(5.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(10.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };
    auto pairs = Closest::boxes_closest(boxes, 0.0);

    MINI_CHECK(pairs.empty());

    std::vector<AABB> boxes_near = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(1.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };
    auto pairs_near = Closest::boxes_closest(boxes_near, 0.0);

    MINI_CHECK(pairs_near.size() == 1);
    MINI_CHECK(pairs_near[0].first == 0);
    MINI_CHECK(pairs_near[0].second == 1);
}

MINI_TEST("AABBTree", "Build Empty") {
    // uncomment #include "aabb.h"
    AABBTree tree;
    tree.build(nullptr, 0);

    MINI_CHECK(tree.empty());
}

MINI_TEST("AABBTree", "Build Single") {
    // uncomment #include "aabb.h"
    AABB aabb = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    AABBTree tree;
    tree.build(&aabb, 1);

    MINI_CHECK(tree.size() == 1);
    MINI_CHECK(tree.nodes[0].object_id == 0);
}

MINI_TEST("AABBTree", "Build Multiple") {
    // uncomment #include "aabb.h"
    std::vector<AABB> aabbs = {
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {5.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {10.0, 0.0, 0.0, 1.0, 1.0, 1.0}
    };
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 5);
    MINI_CHECK(tree.nodes[0].object_id == -1);
}

MINI_TEST("AABBTree", "Node Count") {
    // uncomment #include "aabb.h"
    std::vector<AABB> aabbs;
    for (int i = 0; i < 100; i++) {
        aabbs.push_back({static_cast<double>(i), 0.0, 0.0, 0.5, 0.5, 0.5});
    }
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 199);
}

MINI_TEST("AABBTree", "Mesh Point Aabb") {
    // uncomment #include "aabb.h"
    // uncomment #include "closest.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "primitives.h"
    Mesh m = Primitives::cube(2.0);

    auto [cp1, fk1, d1] = Closest::mesh_point_aabb(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    auto [cp2, fk2, d2] = Closest::mesh_point_aabb(m, Point(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("AABBTree", "Mesh Point Aabb Matches Bvh") {
    // uncomment #include "aabb.h"
    // uncomment #include "closest.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "primitives.h"
    Mesh m = Primitives::cube(2.0);
    Point tp(0.3, 0.7, 1.5);

    auto [cp_bvh, fk_bvh, d_bvh] = Closest::mesh_point(m, tp);
    auto [cp_aabb, fk_aabb, d_aabb] = Closest::mesh_point_aabb(m, tp);

    MINI_CHECK(TOLERANCE.is_close(d_bvh, d_aabb));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[0], cp_aabb[0]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[1], cp_aabb[1]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[2], cp_aabb[2]));
}

MINI_TEST("Aabb", "Constructor") {
    // uncomment #include "aabb.h"
    // uncomment #include "point.h"
    // AABB(0,0,0, 1,2,3) — dims 2×4×6
    AABB a(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);

    MINI_CHECK(TOLERANCE.is_close(a.area(), 88.0));
    MINI_CHECK(a.center() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(a.diagonal(), 2.0 * std::sqrt(14.0)));
    MINI_CHECK(a.is_valid());
    MINI_CHECK(TOLERANCE.is_close(a.volume(), 48.0));

    MINI_CHECK(a.closest_point(Point(0.0, 0.0, 0.0)) == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a.closest_point(Point(10.0, 0.0, 0.0)) == Point(1.0, 0.0, 0.0));
    MINI_CHECK(a.contains(Point(0.0, 0.0, 0.0)));
    MINI_CHECK(!a.contains(Point(10.0, 0.0, 0.0)));

    MINI_CHECK(a.corner(false, false, false) == Point(-1.0, -2.0, -3.0));
    MINI_CHECK(a.corner(true, true, true) == Point(1.0, 2.0, 3.0));
    MINI_CHECK(a.get_corners().size() == 8);
    MINI_CHECK(a.get_edges().size() == 12);

    MINI_CHECK(a.point_at(1.0, 0.0, 0.0) == Point(1.0, 0.0, 0.0));
    MINI_CHECK(a.point_at(0.0, 0.0, 0.0) == Point(0.0, 0.0, 0.0));

    MINI_CHECK(a.intersects(AABB(0.5, 0.0, 0.0, 0.5, 0.5, 0.5)));
    MINI_CHECK(!a.intersects(AABB(10.0, 0.0, 0.0, 0.5, 0.5, 0.5)));
    AABB b(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    a.union_with(b);
    MINI_CHECK(a.min_point() == Point(-1.0, -2.0, -3.0));
    MINI_CHECK(a.max_point() == Point(6.0, 2.0, 3.0));
    AABB c = AABB::merge(AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0), AABB(4.0, 0.0, 0.0, 1.0, 1.0, 1.0));
    MINI_CHECK(c.min_point() == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(c.max_point() == Point(5.0, 1.0, 1.0));
}

MINI_TEST("Aabb", "From Geometry") {
    // uncomment #include "line.h"
    // uncomment #include "polyline.h"
    // uncomment #include "pointcloud.h"
    // uncomment #include "nurbscurve.h"
    // uncomment #include "nurbssurface.h"
    AABB a_pt = AABB::from_point(Point(1.0, 2.0, 3.0), 0.5);

    MINI_CHECK(a_pt.center() == Point(1.0, 2.0, 3.0));
    MINI_CHECK(TOLERANCE.is_close(a_pt.hx, 0.5));

    AABB a_pts = AABB::from_points({
        Point(0.0, 0.0, 0.0),
        Point(3.0, 4.0, 5.0),
    }, 0.0);

    MINI_CHECK(a_pts.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a_pts.max_point() == Point(3.0, 4.0, 5.0));

    Line ln(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
    AABB a_line = AABB::from_line(ln, 1.0);

    MINI_CHECK(a_line.min_point() == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(a_line.max_point() == Point(5.0, 1.0, 1.0));

    Polyline pl({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 2.0, 0.0),
    });
    AABB a_pl = AABB::from_polyline(pl, 0.0);

    MINI_CHECK(a_pl.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a_pl.max_point() == Point(2.0, 2.0, 0.0));

    Mesh cube = Primitives::cube(2.0);
    AABB a_mesh = AABB::from_mesh(cube, 0.0);

    MINI_CHECK(a_mesh.min_point() == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(a_mesh.max_point() == Point(1.0, 1.0, 1.0));

    PointCloud pc(
        {
            Point(0.0, 0.0, 0.0),
            Point(4.0, 2.0, 6.0),
        },
        {
            Vector(0.0, 0.0, 1.0),
            Vector(0.0, 0.0, 1.0),
        },
        {
            Color(255, 0, 0, 255),
            Color(0, 255, 0, 255),
        }
    );
    AABB a_pc = AABB::from_pointcloud(pc, 0.0);

    MINI_CHECK(a_pc.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a_pc.max_point() == Point(4.0, 2.0, 6.0));

    NurbsCurve curve = NurbsCurve::create(false, 2, {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
    });
    AABB a_nc = AABB::from_nurbscurve(curve, 0.5, false);

    MINI_CHECK(a_nc.is_valid());
    MINI_CHECK(a_nc.contains(Point(1.5, 0.0, 0.0)));

    NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(2.0, 2.0, 2.0),
    });
    AABB a_ns = AABB::from_nurbssurface(surf, 0.0);

    MINI_CHECK(a_ns.is_valid());
    MINI_CHECK(TOLERANCE.is_close(a_ns.volume(), 8.0));
}

MINI_TEST("AABBTree", "Query Aabb") {
    // uncomment #include "aabb.h"
    std::vector<AABB> aabbs = {
        {0.0, 0.0, 0.0, 0.5, 0.5, 0.5},
        {5.0, 0.0, 0.0, 0.5, 0.5, 0.5},
        {10.0, 0.0, 0.0, 0.5, 0.5, 0.5},
    };
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    std::vector<int> hits = tree.query_aabb(AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

    MINI_CHECK(hits.size() == 1);
    MINI_CHECK(hits[0] == 0);

    std::vector<int> none = tree.query_aabb(AABB(20.0, 0.0, 0.0, 0.5, 0.5, 0.5));

    MINI_CHECK(none.empty());

    std::vector<int> all = tree.query_aabb(AABB(5.0, 0.0, 0.0, 10.0, 1.0, 1.0));

    MINI_CHECK(all.size() == 3);
}

}
