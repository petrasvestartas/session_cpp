// main_11 — BDS arena standalone test driver.
// Tests src/brep_bds.h/.cpp: the shared intersection data structure of the v2 boolean kernel.
// Every test asserts one of the six invariants the structure exists to make unrepresentable
// (see the header for I1..I6), plus a GOLDEN STRUCTURE test: a known small configuration must
// produce a byte-identical arena signature. The project previously had ZERO test references to
// the scaffold or SharedEdgePool in any language; this is that tripwire.
// Prints one line per cell:  [BDS] <name> PASS|FAIL <detail>;  exit 0 iff all pass.
#include "src/brep.h"
#include "src/brep_bds.h"
#include "src/nurbscurve.h"
#include "src/nurbssurface.h"
#include "src/xform.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace session_cpp;

static int g_pass = 0, g_fail = 0;

static void cell(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("[BDS] %-52s %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL",
                detail.empty() ? "" : " ", detail.c_str());
    ok ? ++g_pass : ++g_fail;
}

static std::string sfmt(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[512];
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return std::string(buf);
}

static NurbsCurve line3(const Point& a, const Point& b) {
    return NurbsCurve::create(false, 1, {a, b});
}

/// Bitwise equality of two doubles — the only honest test of "covers exactly" and of
/// "order-independent fusion".
static bool bits_eq(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
static bool pt_bits_eq(const Point& a, const Point& b) {
    return bits_eq(a[0], b[0]) && bits_eq(a[1], b[1]) && bits_eq(a[2], b[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////
// I6 — identity is by arena index; nothing is found by proximity
///////////////////////////////////////////////////////////////////////////////////////////

static void run_identity() {
    {
        BdsArena ds;
        const int a = ds.append_vertex(Point(1, 2, 3), 1e-7);
        const int b = ds.append_vertex(Point(1, 2, 3), 1e-7);
        // Two vertices at BIT-IDENTICAL coordinates stay distinct. There is no weld radius and
        // no coordinate hash: only an explicit fuse_vertices() can merge them.
        cell("I6 identical_coords_stay_distinct_until_fused",
             a != b && ds.nb_shapes() == 2 && ds.resolve_sd(a) == a,
             sfmt("a=%d b=%d", a, b));
        const int f = ds.fuse_vertices({a, b});
        cell("I6 fusion_is_the_only_merge_path",
             f != a && f != b && ds.resolve_sd(a) == f && ds.resolve_sd(b) == f);
    }
    {
        BRep box = BRep::create_box(2, 2, 2);
        BdsArena ds;
        ds.init({&box});
        bool ok = true;
        for (int i = 0; i < ds.nb_shapes(); ++i) ok = ok && ds.shape(i).index == i;
        for (size_t i = 0; i < box.m_topology_edges.size(); ++i) {
            const int e = ds.index_of_edge(0, (int)i);
            ok = ok && ds.is_edge(e) && ds.shape(e).source == (int)i && ds.shape(e).operand == 0;
        }
        cell("G1 arena_index_is_identity_and_roundtrips", ok,
             sfmt("shapes=%d", ds.nb_shapes()));

        const int nv = (int)box.m_topology_vertices.size();
        const int ne = (int)box.m_topology_edges.size();
        const int nf = (int)box.m_faces.size();
        cell("G2 box_census_8v_12e_6f_1s",
             nv == 8 && ne == 12 && nf == 6 && ds.nb_shapes() == nv + ne + nf + 1,
             sfmt("v=%d e=%d f=%d shapes=%d", nv, ne, nf, ds.nb_shapes()));

        bool subs_ok = true;
        for (int i = 0; i < ds.nb_shapes(); ++i) {
            if (!ds.is_face(i)) continue;
            int nes = 0, nvs = 0;
            for (int s : ds.shape(i).subs) (ds.is_edge(s) ? nes : nvs)++;
            subs_ok = subs_ok && nes == 4 && nvs == 4;
        }
        cell("G2 face_subs_are_4_edges_and_4_vertices_no_wires", subs_ok);

        bool edge_in_two_faces = true;
        for (int i = 0; i < ds.nb_shapes(); ++i) {
            if (!ds.is_edge(i)) continue;
            int n = 0;
            for (int j = 0; j < ds.nb_shapes(); ++j)
                if (ds.is_face(j) && ds.shape(j).has_sub(i)) ++n;
            edge_in_two_faces = edge_in_two_faces && n == 2;
        }
        cell("G6 every_edge_is_shared_by_exactly_two_faces", edge_in_two_faces);

        cell("G3 single_operand_yields_one_range_rank0",
             ds.nb_ranges() == 1 && ds.range(0).first == 0 &&
                 ds.range(0).last == ds.nb_shapes() - 1 && ds.rank(0) == 0 &&
                 ds.rank(ds.append_vertex(Point(9, 9, 9), 1e-7)) == -1);
    }
    {
        // The arena reads no coordinate to decide identity, so a rigid translation cannot change
        // the STRUCTURE of the arena at all. This is the property the v1 splitter loses.
        BRep b0 = BRep::create_box(2, 2, 2);
        BRep b1 = BRep::create_box(2, 2, 2);
        b1.xform = Xform::translation(137.0, -41.5, 9.25);
        b1.transform();
        BdsArena d0, d1;
        d0.init({&b0});
        d1.init({&b1});
        bool same = d0.nb_shapes() == d1.nb_shapes();
        for (int i = 0; same && i < d0.nb_shapes(); ++i)
            same = d0.shape(i).type == d1.shape(i).type && d0.shape(i).subs == d1.shape(i).subs;
        cell("G2 structure_is_invariant_under_rigid_motion", same);
    }
    {
        // Two operands: contiguous, non-overlapping ranges; rank separates them.
        BRep a = BRep::create_box(2, 2, 2);
        BRep b = BRep::create_box(1, 1, 1);
        BdsArena ds;
        ds.init({&a, &b});
        const bool ok = ds.nb_ranges() == 2 && ds.range(0).first == 0 &&
                        ds.range(0).last + 1 == ds.range(1).first &&
                        ds.range(1).last == ds.nb_source_shapes() - 1 &&
                        ds.rank(ds.range(0).last) == 0 && ds.rank(ds.range(1).first) == 1;
        cell("G3 two_operands_partition_the_index_space", ok,
             sfmt("r0=[%d,%d] r1=[%d,%d]", ds.range(0).first, ds.range(0).last,
                  ds.range(1).first, ds.range(1).last));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// I1 — an edge is never mutated; splitting is a view materialised later
///////////////////////////////////////////////////////////////////////////////////////////

static void run_immutability() {
    BRep box = BRep::create_box(2, 2, 2);
    std::vector<unsigned long long> before;
    for (const NurbsCurve& c : box.m_curves_3d) before.push_back(bds_curve_fingerprint(c));

    BdsArena ds;
    ds.init({&box});
    const int e = ds.index_of_edge(0, 0);
    const std::pair<double,double> r = ds.edge_range(e);
    const double tm = 0.5 * (r.first + r.second);
    const Point pm = ds.edge_curve(e)->point_at(tm);
    const int v = ds.append_vertex(pm, 1e-9);
    const BdsArena::PaveStatus st = ds.add_pave(e, tm, v);
    const int nb1 = ds.update_pave_blocks(e);
    const int nb2 = ds.update_pave_blocks(e);   // second call must be a no-op

    std::vector<unsigned long long> after;
    for (const NurbsCurve& c : box.m_curves_3d) after.push_back(bds_curve_fingerprint(c));
    cell("I1 source_curves_bit_identical_after_subdivision", before == after,
         sfmt("curves=%zu", before.size()));
    cell("I1 pave_insert_then_materialise_gives_two_blocks",
         st == BdsArena::PaveStatus::Inserted && nb1 == 2,
         sfmt("status=%d blocks=%d", (int)st, nb1));
    cell("I1 materialising_twice_is_idempotent", nb2 == nb1, sfmt("%d vs %d", nb1, nb2));
    cell("I1 blocks_cover_the_edge_bitwise_exactly", ds.pave_blocks_cover_edge(e));

    // The edge record itself exposes no setter; its range is what it was at append time.
    const std::pair<double,double> r2 = ds.edge_range(e);
    cell("I1 edge_range_unchanged_by_subdivision",
         bits_eq(r.first, r2.first) && bits_eq(r.second, r2.second));
}

///////////////////////////////////////////////////////////////////////////////////////////
// I4 — paves sorted, unique, always naming a real vertex; adversarial input
///////////////////////////////////////////////////////////////////////////////////////////

static void run_paves() {
    BdsArena ds;
    const int v0 = ds.append_vertex(Point(0, 0, 0), 1e-9);
    const int v1 = ds.append_vertex(Point(10, 0, 0), 1e-9);
    const int e = ds.append_edge(line3(Point(0, 0, 0), Point(10, 0, 0)), v0, v1, 1e-9);

    // REVERSED insertion order: 7, then 2, then 5.
    const int a7 = ds.append_vertex(Point(7, 0, 0), 1e-9);
    const int a2 = ds.append_vertex(Point(2, 0, 0), 1e-9);
    const int a5 = ds.append_vertex(Point(5, 0, 0), 1e-9);
    bool ins = true;
    ins = ins && ds.add_pave(e, 7.0, a7) == BdsArena::PaveStatus::Inserted;
    ins = ins && ds.add_pave(e, 2.0, a2) == BdsArena::PaveStatus::Inserted;
    ins = ins && ds.add_pave(e, 5.0, a5) == BdsArena::PaveStatus::Inserted;
    const int nb = ds.update_pave_blocks(e);

    std::vector<BdsPave> ps;
    const bool tiling = ds.paves(e, ps);
    bool sorted = true, params = ps.size() == 5;
    for (size_t i = 1; i < ps.size(); ++i) sorted = sorted && ps[i - 1].t < ps[i].t;
    const double want[5] = {0, 2, 5, 7, 10};
    for (size_t i = 0; params && i < 5; ++i) params = params && ps[i].t == want[i];
    cell("I4 paves_sorted_after_reversed_insertion", ins && sorted && params && nb == 4,
         sfmt("blocks=%d paves=%zu", nb, ps.size()));
    cell("I4 tiling_law_paves_equals_blocks_plus_one", tiling && (int)ps.size() == nb + 1);

    bool real = true;
    for (const BdsPave& p : ps) real = real && ds.is_vertex(p.vertex);
    cell("I4 every_pave_names_a_real_vertex", real);

    // Adversarial 1: the SAME vertex again, at the same parameter.
    const BdsArena::PaveStatus dup = ds.add_pave(e, 5.0, a5);
    // Adversarial 2: a DISTINCT vertex at a parameter 1e-16 away — model-space coincident.
    const int a5b = ds.append_vertex(Point(5, 0, 0), 1e-9);
    const BdsArena::PaveStatus eps = ds.add_pave(e, 5.0 + 1e-16, a5b);
    // Adversarial 3: a distinct vertex at exactly the same 3D point, unrelated parameter.
    const int a5c = ds.append_vertex(Point(5, 0, 0), 1e-9);
    const BdsArena::PaveStatus coin = ds.add_pave(e, 5.5, a5c);
    // Adversarial 4: outside the range, and a bad vertex / bad edge.
    const int aout = ds.append_vertex(Point(11, 0, 0), 1e-9);
    const BdsArena::PaveStatus oor = ds.add_pave(e, 11.0, aout);
    const BdsArena::PaveStatus bad_v = ds.add_pave(e, 3.0, 9999);
    const BdsArena::PaveStatus bad_e = ds.add_pave(v0, 3.0, aout);
    const int nb_after = ds.update_pave_blocks(e);

    cell("I4 duplicate_vertex_rejected", dup == BdsArena::PaveStatus::DuplicateVertex);
    cell("I4 parameter_1e-16_apart_rejected_in_model_space",
         eps == BdsArena::PaveStatus::CoincidentVertex, sfmt("status=%d", (int)eps));
    cell("I4 distinct_vertex_at_same_point_rejected",
         coin == BdsArena::PaveStatus::CoincidentVertex);
    cell("I4 out_of_range_pave_rejected", oor == BdsArena::PaveStatus::OutOfRange);
    cell("I4 bad_vertex_and_bad_edge_rejected",
         bad_v == BdsArena::PaveStatus::BadVertex && bad_e == BdsArena::PaveStatus::BadEdge);
    cell("I4 rejected_paves_created_no_blocks", nb_after == 4, sfmt("blocks=%d", nb_after));
    cell("I4 blocks_still_cover_edge_exactly", ds.pave_blocks_cover_edge(e));

    // Closed edge: one distinct vertex, two paves at the two range ends (the seam rule).
    BdsArena cs;
    const int cv = cs.append_vertex(Point(1, 0, 0), 1e-9);
    NurbsCurve loop = NurbsCurve::create(false, 1,
        {Point(1, 0, 0), Point(0, 1, 0), Point(-1, 0, 0), Point(0, -1, 0), Point(1, 0, 0)});
    const int ce = cs.append_edge(loop, cv, cv, 1e-9);
    std::vector<BdsPave> cps;
    const bool ctile = cs.paves(ce, cps);
    cell("I4 closed_edge_seam_rule_two_paves_one_vertex",
         cs.edge_geom(ce).closed && cps.size() == 2 && cps[0].vertex == cv &&
             cps[1].vertex == cv && ctile && cs.pave_blocks(ce).size() == 1,
         sfmt("paves=%zu blocks=%zu", cps.size(), cs.pave_blocks(ce).size()));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Pave-block interval arithmetic
///////////////////////////////////////////////////////////////////////////////////////////

static void run_intervals() {
    BdsPaveBlock pb;
    pb.original_edge = 42;
    pb.pave1 = {7, 0.25};
    pb.pave2 = {9, 3.75};
    const BdsPave at{11, 1.5};
    BdsPB a, b;
    const bool ok = bds_split_interval(pb, at, a, b);
    const bool exact = ok && bits_eq(a->pave1.t, pb.pave1.t) && bits_eq(a->pave2.t, at.t) &&
                       bits_eq(b->pave1.t, at.t) && bits_eq(b->pave2.t, pb.pave2.t) &&
                       a->pave1.vertex == pb.pave1.vertex && a->pave2.vertex == at.vertex &&
                       b->pave1.vertex == at.vertex && b->pave2.vertex == pb.pave2.vertex;
    cell("INT split_at_pave_covers_original_exactly_bitwise", exact);
    cell("INT split_halves_inherit_only_the_original_edge",
         ok && a->original_edge == 42 && b->original_edge == 42 && a->edge == -1 &&
             b->edge == -1 && !a->has_shrunk && !b->has_shrunk);

    BdsPB x, y;
    cell("INT split_outside_range_refused",
         !bds_split_interval(pb, BdsPave{11, 4.0}, x, y) &&
             !bds_split_interval(pb, BdsPave{11, 0.25}, x, y) &&
             !bds_split_interval(pb, BdsPave{11, 3.75}, x, y));

    // update() with three pending paves: four blocks, contiguous, summing to the original span.
    BdsPaveBlock u;
    u.original_edge = 3;
    u.pave1 = {0, 0.0};
    u.pave2 = {1, 10.0};
    u.append_ext_pave({5, 7.0});
    u.append_ext_pave({6, 2.0});
    u.append_ext_pave({7, 5.0});
    const bool refused = !u.append_ext_pave({6, 2.0});   // fenced by vertex index
    std::vector<BdsPB> out;
    u.update(out, true);
    double sum = 0.0;
    bool contiguous = out.size() == 4;
    for (size_t i = 0; i < out.size(); ++i) {
        sum += out[i]->length_param();
        if (i) contiguous = contiguous && bits_eq(out[i - 1]->pave2.t, out[i]->pave1.t);
    }
    cell("INT update_fence_refuses_repeat_vertex", refused);
    cell("INT update_yields_n_minus_1_contiguous_blocks",
         contiguous && bits_eq(out.front()->pave1.t, 0.0) && bits_eq(out.back()->pave2.t, 10.0) &&
             sum == 10.0,
         sfmt("blocks=%zu sum=%.17g", out.size(), sum));
    cell("INT update_clears_pending_paves", !u.is_to_update());
}

///////////////////////////////////////////////////////////////////////////////////////////
// I3 — vertex fusion is idempotent and order independent (BRepLib::BoundingVertex)
///////////////////////////////////////////////////////////////////////////////////////////

static void run_fusion() {
    {   // Two balls, neither containing the other: exact smallest enclosing sphere.
        std::vector<Point> p = {Point(0, 0, 0), Point(1, 0, 0)};
        std::vector<double> t = {1e-3, 2e-3};
        Point c; double tol = 0.0;
        bds_bounding_vertex(p, t, c, tol);
        const bool ok = std::abs(c[0] - 0.5005) < 1e-15 && std::abs(c[1]) < 1e-18 &&
                        std::abs(tol - 0.5015) < 1e-15;
        cell("I3 two_ball_exact_smallest_enclosing_sphere", ok,
             sfmt("c=(%.15g,%g,%g) tol=%.15g", c[0], c[1], c[2], tol));
        // Both input balls are inside the result, to the last bit of the analytic answer.
        bool covers = true;
        for (size_t i = 0; i < p.size(); ++i) {
            const double d = std::sqrt((c[0]-p[i][0])*(c[0]-p[i][0]) + (c[1]-p[i][1])*(c[1]-p[i][1]) +
                                       (c[2]-p[i][2])*(c[2]-p[i][2]));
            covers = covers && d + t[i] <= tol + 1e-15;
        }
        cell("I3 two_ball_result_covers_both_inputs", covers);
    }
    {   // One ball inside the other: the result IS the big ball, untouched.
        std::vector<Point> p = {Point(0, 0, 0), Point(0.1, 0, 0)};
        std::vector<double> t = {1.0, 0.1};
        Point c; double tol = 0.0;
        bds_bounding_vertex(p, t, c, tol);
        cell("I3 two_ball_containment_returns_the_larger_ball",
             pt_bits_eq(c, p[0]) && bits_eq(tol, 1.0));
    }
    {   // Order independence, n == 2: swapping the inputs must be bit-identical.
        std::vector<Point> p = {Point(0.3, -1.2, 4.4), Point(-2.1, 0.7, 1.9)};
        std::vector<double> t = {3e-4, 7e-5};
        Point c1, c2; double t1 = 0, t2 = 0;
        bds_bounding_vertex(p, t, c1, t1);
        std::vector<Point> pr = {p[1], p[0]};
        std::vector<double> tr = {t[1], t[0]};
        bds_bounding_vertex(pr, tr, c2, t2);
        cell("I3 two_ball_order_independent_bitwise", pt_bits_eq(c1, c2) && bits_eq(t1, t2));
    }
    {   // Order independence, n > 2: the sorted-mean rule over five permutations.
        std::vector<Point> p = {Point(0.1, 0.2, 0.3), Point(-4.0, 1.5, 2.25),
                                Point(7.125, -3.5, 0.0), Point(0.0, 0.0, 9.5),
                                Point(-1.75, 6.25, -2.5)};
        std::vector<double> t = {1e-5, 2e-5, 3e-5, 4e-5, 5e-5};
        Point c0; double t0 = 0;
        bds_bounding_vertex(p, t, c0, t0);
        bool same = true;
        const int perms[5][5] = {{4,3,2,1,0},{2,0,4,1,3},{1,4,0,3,2},{3,2,1,0,4},{0,4,3,2,1}};
        for (int k = 0; k < 5; ++k) {
            std::vector<Point> pp;
            std::vector<double> tt;
            for (int j = 0; j < 5; ++j) { pp.push_back(p[perms[k][j]]); tt.push_back(t[perms[k][j]]); }
            Point c; double tl = 0;
            bds_bounding_vertex(pp, tt, c, tl);
            same = same && pt_bits_eq(c, c0) && bits_eq(tl, t0);
        }
        cell("I3 sorted_mean_order_independent_over_5_permutations", same,
             sfmt("tol=%.17g", t0));
        bool covers = true;
        for (size_t i = 0; i < p.size(); ++i) {
            const double d = std::sqrt((c0[0]-p[i][0])*(c0[0]-p[i][0]) + (c0[1]-p[i][1])*(c0[1]-p[i][1]) +
                                       (c0[2]-p[i][2])*(c0[2]-p[i][2]));
            covers = covers && d + t[i] <= t0 + 1e-15;
        }
        cell("I3 sorted_mean_result_covers_every_member", covers);
    }
    {   // Arena level: idempotent, order independent, and it mints exactly one vertex.
        BdsArena ds;
        const int a = ds.append_vertex(Point(0, 0, 0), 1e-5);
        const int b = ds.append_vertex(Point(1e-6, 0, 0), 2e-5);
        const int c = ds.append_vertex(Point(0, 1e-6, 0), 3e-5);
        const int n_before = ds.nb_shapes();
        const int f1 = ds.fuse_vertices({a, b, c});
        const int n_mid = ds.nb_shapes();
        const int f2 = ds.fuse_vertices({a, b, c});
        const int f3 = ds.fuse_vertices({c, a, b});
        const int f4 = ds.fuse_vertices({b, c, a, b, a});   // duplicates in the request
        cell("I3 arena_fusion_idempotent_same_index_no_second_vertex",
             f1 == f2 && n_mid == n_before + 1 && ds.nb_shapes() == n_mid && ds.nb_fusions() == 1,
             sfmt("f=%d shapes %d->%d", f1, n_before, ds.nb_shapes()));
        cell("I3 arena_fusion_order_independent", f1 == f3 && f1 == f4);
        cell("I3 arena_fusion_geometry_identical_on_repeat",
             pt_bits_eq(ds.vertex_point(f1), ds.vertex_point(f2)) &&
                 bits_eq(ds.vertex(f1).tol, ds.vertex(f2).tol));
        cell("I3 fused_tolerance_covers_every_member", [&] {
            bool ok = true;
            for (int v : {a, b, c}) {
                const Point& p = ds.vertex_point(v);
                const Point& q = ds.vertex_point(f1);
                const double d = std::sqrt((p[0]-q[0])*(p[0]-q[0]) + (p[1]-q[1])*(p[1]-q[1]) +
                                           (p[2]-q[2])*(p[2]-q[2]));
                ok = ok && d + ds.tolerance(v) <= ds.vertex(f1).tol + 1e-18;
            }
            return ok;
        }());
        cell("I3 members_redirect_to_the_fused_vertex",
             ds.resolve_sd(a) == f1 && ds.resolve_sd(b) == f1 && ds.resolve_sd(c) == f1 &&
                 ds.vertex(f1).origin == a);
        cell("I3 single_member_cluster_is_the_member_itself", ds.fuse_vertices({a}) == a);
    }
    {   // Same-domain chain: transitive, and capped so a cycle terminates (OCCT hangs here).
        BdsArena ds;
        for (int i = 0; i < 10; ++i) ds.append_vertex(Point(i, 0, 0), 1e-9);
        ds.add_shape_sd(5, 7);
        ds.add_shape_sd(7, 9);
        const bool chain = ds.resolve_sd(5) == 9 && !ds.sd_cycle_detected();
        ds.add_shape_sd(3, 3);
        int out = -1;
        const bool refused = !ds.has_shape_sd(3, out);
        BdsArena cyc;
        for (int i = 0; i < 10; ++i) cyc.append_vertex(Point(i, 0, 0), 1e-9);
        cyc.add_shape_sd(5, 7);
        cyc.add_shape_sd(7, 5);
        (void)cyc.resolve_sd(5);
        cell("G11 same_domain_chain_walks_transitively", chain);
        cell("G11 same_domain_self_binding_refused", refused);
        cell("G11 same_domain_cycle_terminates_and_is_reported", cyc.sd_cycle_detected());
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// I2 — coincidence is ONE common block
///////////////////////////////////////////////////////////////////////////////////////////

/// Two collinear overlapping edges paved at the overlap ends: EA[0,1] and EB offset by `off`.
struct Overlap {
    BdsArena ds;
    int vA0 = -1, vA1 = -1, vB0 = -1, vB1 = -1, ea = -1, eb = -1;
    void build(double off, double tolA, double tolB, double dy = 0.0) {
        vA0 = ds.append_vertex(Point(0, 0, 0), tolA);
        vA1 = ds.append_vertex(Point(1, 0, 0), tolA);
        ea = ds.append_edge(line3(Point(0, 0, 0), Point(1, 0, 0)), vA0, vA1, tolA);
        vB0 = ds.append_vertex(Point(off, dy, 0), tolB);
        vB1 = ds.append_vertex(Point(off + 1, dy, 0), tolB);
        eb = ds.append_edge(line3(Point(off, dy, 0), Point(off + 1, dy, 0)), vB0, vB1, tolB);
    }
};

static void run_common_blocks() {
    Overlap o;
    o.build(0.5, 1e-7, 1e-7);
    BdsArena& ds = o.ds;
    ds.add_pave(o.ea, 0.5, o.vB0);
    ds.add_pave(o.eb, 0.5, o.vA1);
    ds.update_pave_blocks();
    const std::vector<BdsPB>& pa = ds.pave_blocks(o.ea);
    const std::vector<BdsPB>& pb = ds.pave_blocks(o.eb);
    cell("I2 overlap_paved_into_two_blocks_per_edge", pa.size() == 2 && pb.size() == 2,
         sfmt("a=%zu b=%zu", pa.size(), pb.size()));
    cell("I2 coincident_pieces_have_the_same_bounds", pa[1]->has_same_bounds(*pb[0]));

    BdsCB cb = ds.make_common_block({pa[1], pb[0]});
    cell("I2 one_common_block_references_both_pieces",
         cb && cb->members.size() == 2 && ds.common_block(pa[1]) == cb &&
             ds.common_block(pb[0]) == cb,
         sfmt("members=%zu", cb ? cb->members.size() : 0));
    cell("I2 representative_is_the_minimal_original_edge",
         cb && cb->representative()->original_edge == o.ea && !cb->representative_forced);
    cell("I2 real_pave_block_canonicalises_both_members",
         ds.real_pave_block(pa[1]) == cb->representative() &&
             ds.real_pave_block(pb[0]) == cb->representative());

    BdsCB cb2 = ds.make_common_block({pa[1], pb[0]});
    cell("I2 rebuilding_the_block_returns_the_same_object_no_duplicate_members",
         cb2 == cb && cb->members.size() == 2);

    cb->set_edge(77);
    cell("I2 set_edge_names_one_edge_for_every_member",
         pa[1]->edge == 77 && pb[0]->edge == 77 && cb->edge() == 77);

    cb->set_representative(pb[0]);
    cell("I2 forced_representative_is_recorded_as_forced",
         cb->representative() == pb[0] && cb->representative_forced &&
             cb->members.size() == 2);

    cell("I2 group_smaller_than_two_is_refused", !ds.make_common_block({pa[0]}));
    cell("I2 coincidence_predicate_agrees_on_the_overlapping_pieces",
         bds_check_coincidence(ds, pa[1], pb[0], 1e-9) &&
             !bds_check_coincidence(ds, pa[0], pb[1], 1e-9));

    // Common-block tolerance: two parallel edges offset by d. The measured answer is
    // tol(other edge) + d, exactly (kb/port_01_bds_arena.md T5).
    {
        const double d = 1e-4, t2 = 3e-6;
        BdsArena p;
        const int a0 = p.append_vertex(Point(0, 0, 0), 1e-7);
        const int a1 = p.append_vertex(Point(10, 0, 0), 1e-7);
        const int e1 = p.append_edge(line3(Point(0, 0, 0), Point(10, 0, 0)), a0, a1, 1e-7);
        const int b0 = p.append_vertex(Point(0, d, 0), t2);
        const int b1 = p.append_vertex(Point(10, d, 0), t2);
        const int e2 = p.append_edge(line3(Point(0, d, 0), Point(10, d, 0)), b0, b1, t2);
        BdsCB c = p.make_common_block({p.pave_blocks(e1)[0], p.pave_blocks(e2)[0]});
        const double want = t2 + d;
        cell("G7 common_block_tolerance_is_the_measured_offset",
             c && std::abs(c->tol - want) < 1e-12,
             sfmt("tol=%.12g want=%.12g", c ? c->tol : -1.0, want));
        // A single member with no faces early-exits to the representative edge's own tolerance.
        BdsCB solo = std::make_shared<BdsCommonBlock>();
        solo->add(p.pave_blocks(e1)[0]);
        cell("G7 single_member_no_faces_early_exits_to_edge_tolerance",
             std::abs(p.compute_tolerance_of_cb(solo) - 1e-7) < 1e-18);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// I5 — every tolerance is a model-space distance; UV conversion happens at the point of use
///////////////////////////////////////////////////////////////////////////////////////////

static NurbsSurface plane_patch(int n, double size) {
    // n x n control points of a flat square [0,size]^2 in z = 0, degree 1. The UV domain is
    // [0, n-1]^2, so the SAME geometry can be presented with different parameterisations.
    std::vector<Point> cps;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            cps.push_back(Point(size * i / (n - 1), size * j / (n - 1), 0.0));
    return NurbsSurface::create(false, false, 1, 1, n, n, cps);
}

static void run_tolerance_model() {
    const double tol3d = 1e-6, size = 10.0;
    NurbsSurface s1 = plane_patch(2, size);   // domain [0,1]^2, |dS/du| = 10
    NurbsSurface s2 = plane_patch(3, size);   // domain [0,2]^2, |dS/du| =  5
    double du1 = 0, dv1 = 0, du2 = 0, dv2 = 0;
    const bool ok1 = bds_uv_tolerance(s1, 0.5, 0.5, tol3d, du1, dv1);
    const bool ok2 = bds_uv_tolerance(s2, 1.0, 1.0, tol3d, du2, dv2);
    const double d1 = s1.domain(0).second - s1.domain(0).first;
    const double d2 = s2.domain(0).second - s2.domain(0).first;
    cell("I5 uv_tolerance_is_tol3d_over_surface_metric",
         ok1 && ok2 && std::abs(du1 - tol3d / 10.0) < 1e-18 &&
             std::abs(du2 - tol3d / 5.0) < 1e-18,
         sfmt("du1=%.6g du2=%.6g dom1=%g dom2=%g", du1, du2, d1, d2));
    // The parameter-space number differs by 2x between the two presentations; the model-space
    // number it represents is IDENTICAL. That is the whole point of I5: a domain-relative
    // constant would have silently inflated with the domain.
    cell("I5 model_space_distance_is_invariant_to_reparameterisation",
         std::abs(du1 * (size / d1) - tol3d) < 1e-18 &&
             std::abs(du2 * (size / d2) - tol3d) < 1e-18 && std::abs(du2 / du1 - 2.0) < 1e-12);

    // A degenerate metric (a pole) is reported by name, not survived as a lucky near-zero.
    std::vector<Point> deg = {Point(0, 0, 0), Point(0, 0, 0), Point(10, 0, 0), Point(10, 0, 0)};
    NurbsSurface pole = NurbsSurface::create(false, false, 1, 1, 2, 2, deg);
    double pu = -1, pv = -1;
    cell("I5 degenerate_metric_is_reported_not_survived",
         !bds_uv_tolerance(pole, 0.5, 0.5, tol3d, pu, pv));

    NurbsCurve c = line3(Point(0, 0, 0), Point(4, 0, 0));   // arc-length domain [0,4]
    double dt = 0;
    const bool okc = bds_t_tolerance(c, 2.0, tol3d, dt);
    cell("I5 curve_parameter_tolerance_is_tol3d_over_speed",
         okc && std::abs(dt - tol3d) < 1e-15, sfmt("dt=%.6g", dt));

    // Tolerance growth is recorded at the merge site; there is no global epsilon.
    BdsArena ds;
    const int v = ds.append_vertex(Point(0, 0, 0), 1e-9);
    ds.absorb_tolerance(v, 5e-4);
    ds.absorb_tolerance(v, 1e-9);
    cell("I5 absorb_tolerance_is_monotone_at_the_merge_site",
         std::abs(ds.tolerance(v) - 5e-4) < 1e-18 && std::abs(ds.vertex(v).tol - 5e-4) < 1e-18);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Face info, interference log, type table
///////////////////////////////////////////////////////////////////////////////////////////

static void run_face_info() {
    BRep box = BRep::create_box(2, 2, 2);
    BdsArena ds;
    ds.init({&box});
    int f0 = -1, f1 = -1, shared_edge = -1;
    for (int i = 0; i < ds.nb_shapes() && shared_edge < 0; ++i) {
        if (!ds.is_face(i)) continue;
        for (int j = i + 1; j < ds.nb_shapes() && shared_edge < 0; ++j) {
            if (!ds.is_face(j)) continue;
            for (int s : ds.shape(i).subs)
                if (ds.is_edge(s) && ds.shape(j).has_sub(s)) { f0 = i; f1 = j; shared_edge = s; break; }
        }
    }
    ds.update_face_info_on(f0);
    ds.update_face_info_on(f1);
    const BdsFaceInfo& i0 = ds.face_info(f0);
    cell("G8 face_info_on_is_derived_from_the_boundary",
         i0.pb_on.size() == 4 && i0.v_on.size() == 4,
         sfmt("on=%d von=%zu", i0.pb_on.size(), i0.v_on.size()));

    std::vector<const BdsPaveBlock*> first;
    for (const BdsPB& p : i0.pb_on) first.push_back(p.get());
    ds.update_face_info_on(f0);
    std::vector<const BdsPaveBlock*> second;
    for (const BdsPB& p : ds.face_info(f0).pb_on) second.push_back(p.get());
    cell("G8 recomputing_on_is_idempotent_pointer_for_pointer", first == second);

    // The shared edge's pave block is the SAME OBJECT in both faces' On sets. That single
    // shared object is how one edge reaches two faces without any coordinate comparison.
    const BdsPB& sp = ds.pave_blocks(shared_edge)[0];
    cell("G6 shared_edge_block_is_one_object_in_both_faces",
         ds.face_info(f0).pb_on.contains(sp) && ds.face_info(f1).pb_on.contains(sp),
         sfmt("f0=%d f1=%d e=%d", f0, f1, shared_edge));

    BdsFaceInfo& mf = ds.change_face_info(f0);
    mf.pb_in.add(sp);
    const int in_before = mf.pb_in.size();
    ds.refine_face_info_in(f0);
    cell("G8 refine_in_removes_exactly_the_on_intersection",
         in_before == 1 && ds.face_info(f0).pb_in.size() == 0);
}

static void run_interf_and_types() {
    BdsArena ds;
    for (int i = 0; i < 6; ++i) ds.append_vertex(Point(i, 0, 0), 1e-9);
    const int i1 = ds.add_interf(BdsInterfType::VV, 1, 2);
    const int i2 = ds.add_interf(BdsInterfType::VV, 2, 1);   // same unordered pair
    const int i3 = ds.add_interf(BdsInterfType::VE, 3, 4);
    ds.change_interf(i3).new_vertex = 5;
    cell("G7 interference_log_is_idempotent_and_symmetric",
         i1 == i2 && ds.interfs().size() == 2 && ds.has_interf(1, 2) && ds.has_interf(2, 1) &&
             !ds.has_interf(1, 4) && ds.has_interf(4));
    cell("G7 interference_records_what_it_created",
         ds.interfs()[i3].has_new_vertex() && ds.interfs()[i3].new_vertex == 5 &&
             ds.nb_interf(BdsInterfType::VV) == 1 && ds.nb_interf(BdsInterfType::VE) == 1);

    const BdsType T[4] = {BdsType::Vertex, BdsType::Edge, BdsType::Face, BdsType::Solid};
    const BdsInterfType want[4][4] = {
        {BdsInterfType::VV, BdsInterfType::VE, BdsInterfType::VF, BdsInterfType::None},
        {BdsInterfType::VE, BdsInterfType::EE, BdsInterfType::EF, BdsInterfType::None},
        {BdsInterfType::VF, BdsInterfType::EF, BdsInterfType::FF, BdsInterfType::None},
        {BdsInterfType::None, BdsInterfType::None, BdsInterfType::None, BdsInterfType::None}};
    bool table = true;
    for (int a = 0; a < 4; ++a)
        for (int b = 0; b < 4; ++b) table = table && bds_pair_type(T[a], T[b]) == want[a][b];
    cell("G9 pair_type_table_matches_BOPDS_Tools", table);

    // Box gap arithmetic: the pair test is a MODEL-SPACE sum of tolerances, nothing else.
    // Two points 1.0 apart. The pair is a candidate iff gap(a) + gap(b) >= 1.0 — a sum of
    // model-space tolerances, never a global epsilon. Values chosen exactly representable so
    // the threshold is tested to the bit.
    BdsBox b1, b2;
    b1.add(Point(0, 0, 0));
    b2.add(Point(1.0, 0, 0));
    b1.set_gap(0.25);
    b2.set_gap(0.75);
    const bool at_threshold = !b1.is_out(b2);   // 0.25 + 0.75 == 1.0 -> touching counts as in
    b2.set_gap(0.5);
    const bool below_threshold = b1.is_out(b2); // 0.25 + 0.5 == 0.75 < 1.0 -> out
    cell("I5 box_pair_test_is_the_sum_of_model_space_gaps",
         at_threshold && below_threshold);
}

///////////////////////////////////////////////////////////////////////////////////////////
// GOLDEN STRUCTURE — a known configuration must produce a byte-identical arena
///////////////////////////////////////////////////////////////////////////////////////////

// Two unit edges of two operands overlapping on x in [0.5, 1]:
//   A: (0,0,0) -> (1,0,0)          B: (0.5,0,0) -> (1.5,0,0)
// B's start vertex lies on A at t=0.5 and A's end vertex lies on B at t=0.5, so each edge paves
// into two blocks and the overlapping pair becomes ONE common block. This is the smallest
// configuration that exercises vertices, edges, paves, blocks, a common block with a measured
// tolerance and an interference record at once.
static BdsArena* build_golden() {
    BdsArena* ds = new BdsArena();
    const int vA0 = ds->append_vertex(Point(0, 0, 0), 1e-7);
    const int vA1 = ds->append_vertex(Point(1, 0, 0), 1e-7);
    const int ea = ds->append_edge(line3(Point(0, 0, 0), Point(1, 0, 0)), vA0, vA1, 1e-7);
    const int vB0 = ds->append_vertex(Point(0.5, 0, 0), 1e-7);
    const int vB1 = ds->append_vertex(Point(1.5, 0, 0), 1e-7);
    const int eb = ds->append_edge(line3(Point(0.5, 0, 0), Point(1.5, 0, 0)), vB0, vB1, 1e-7);
    ds->add_pave(ea, 0.5, vB0);
    ds->add_pave(eb, 0.5, vA1);
    ds->update_pave_blocks();
    ds->make_common_block({ds->pave_blocks(ea)[1], ds->pave_blocks(eb)[0]});
    ds->add_interf(BdsInterfType::EE, ea, eb);
    return ds;
}

static const char* GOLDEN =
"shapes=6 source=0 ranges=0 fusions=0 interf=1\n"
"V0 op=-1 src=-1 tol=1e-07 subs=[] p=(0,0,0) vtol=1e-07 origin=-1\n"
"V1 op=-1 src=-1 tol=1e-07 subs=[] p=(1,0,0) vtol=1e-07 origin=-1\n"
"E2 op=-1 src=-1 tol=1e-07 subs=[0,1] t=[0,1] closed=0\n"
"  PB0 [0@0,3@0.5] edge=-1 cb=-1\n"
"  PB1 [3@0.5,1@1] edge=-1 cb=0\n"
"V3 op=-1 src=-1 tol=1e-07 subs=[] p=(0.5,0,0) vtol=1e-07 origin=-1\n"
"V4 op=-1 src=-1 tol=1e-07 subs=[] p=(1.5,0,0) vtol=1e-07 origin=-1\n"
"E5 op=-1 src=-1 tol=1e-07 subs=[3,4] t=[0,1] closed=0\n"
"  PB0 [3@0,1@0.5] edge=-1 cb=0\n"
"  PB1 [1@0.5,4@1] edge=-1 cb=-1\n"
"CB0 rep_edge=2 members=2 tol=1e-07 forced=0 faces=[] blocks=[2:0.5-1,5:0-0.5]\n"
"I0 EE a=2 b=5 nv=-1\n";

static void run_golden() {
    BdsArena* ds = build_golden();
    const std::string sig = ds->signature();
    const bool match = (sig == GOLDEN);
    cell("GOLD known_configuration_produces_the_exact_expected_arena", match);
    if (!match || std::getenv("BDS_GOLDEN_PRINT")) {
        std::printf("---- actual ----\n%s---- expected ----\n%s----\n", sig.c_str(), GOLDEN);
    }
    // The signature is a pure function of the structure: rebuilding must reproduce it exactly.
    BdsArena* ds2 = build_golden();
    cell("GOLD signature_is_reproducible_across_rebuilds", ds2->signature() == sig);

    std::string why;
    cell("GOLD golden_arena_satisfies_check_invariants", ds->check_invariants(&why), why);

    BRep box = BRep::create_box(2, 2, 2);
    BdsArena bx;
    bx.init({&box});
    for (int i = 0; i < bx.nb_shapes(); ++i)
        if (bx.is_edge(i)) bx.change_pave_blocks(i);
    std::string why2;
    cell("GOLD box_arena_satisfies_check_invariants", bx.check_invariants(&why2), why2);
    delete ds;
    delete ds2;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    run_identity();
    run_immutability();
    run_paves();
    run_intervals();
    run_fusion();
    run_common_blocks();
    run_tolerance_model();
    run_face_info();
    run_interf_and_types();
    run_golden();
    std::printf("[BDS] TOTAL %d/%d\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
