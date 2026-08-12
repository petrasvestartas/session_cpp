// main_10 — P3 same-domain subsystem standalone test driver (session B).
// Tests src/brep_samedomain.h/.cpp against coincident-face pairs constructed directly from
// primitives: A-op-A cells, flush caps/stacks (occt ZD4/ZD5/ZN4/ZP1 families), partial
// overlaps, box x cyl line contact. Asserts SD detection, orientation relation, the BuildBOP
// op-table, ON-classification, and coincidence-only boolean assembly.
// Prints one line per cell:  [SD] <name> PASS|FAIL <detail>;  exit 0 iff all pass.
#include "src/brep.h"
#include "src/brep_samedomain.h"
#include "src/brep_commonblock.h"
#include "src/nurbscurve.h"
#include "src/xform.h"
#include <cmath>
#include <cstdio>
#include <set>
#include <string>

using namespace session_cpp;

static int g_pass = 0, g_fail = 0;

static void cell(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("[SD] %-44s %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL",
                detail.empty() ? "" : " ", detail.c_str());
    ok ? ++g_pass : ++g_fail;
}

static BRep place(BRep b, double tx, double ty, double tz, int axis = 2, double deg = 0.0) {
    Xform t = Xform::translation(tx, ty, tz);
    if (deg != 0.0) {
        Xform r = axis == 0 ? Xform::rotation_x(deg, true)
                : axis == 1 ? Xform::rotation_y(deg, true)
                            : Xform::rotation_z(deg, true);
        b.transform(t * r);
    } else {
        b.transform(t);
    }
    return b;
}

static int naked_edges(const BRep& r) {
    int nk = 0;
    for (const auto& e : r.m_topology_edges)
        if ((int)e.trim_indices.size() == 1) ++nk;
    return nk;
}

static const double TOL = 1e-6;

// detect over two operands, return the detector
static SameDomain detect_pair(const BRep& A, const BRep& B, int solidA = 0, int solidB = 1) {
    SameDomain sd(TOL);
    sd.add_solid(A, 0, solidA);
    sd.add_solid(B, 1, solidB);
    sd.detect();
    return sd;
}

static int count_orient(const SameDomain& sd, int orient) {
    int c = 0;
    for (const auto& p : sd.pairs())
        if (p.orient == orient) ++c;
    return c;
}

static const char* st_name(SDState s) {
    switch (s) {
        case SDState::In: return "In";
        case SDState::Out: return "Out";
        case SDState::OnSame: return "OnSame";
        case SDState::OnOpposite: return "OnOpp";
    }
    return "?";
}

// Per-face state dump for a pair (SESSION_SD_DUMP=1). The integration hook debugging aid.
static void dump_pair(const std::string& tag, const BRep& A, const BRep& B) {
    if (!std::getenv("SESSION_SD_DUMP")) return;
    std::vector<double> osA = A.face_outward_signs(), osB = B.face_outward_signs();
    for (int i = 0; i < A.face_count(); ++i)
        std::printf("[SD-DUMP] %s A f%d osign=%+.0f nsamp=%zu state=%s\n", tag.c_str(), i, osA[i],
                    SameDomain::samples_in_face(A, i, 16).size(),
                    st_name(sd_classify_face(A, i, B, TOL, osA, osB)));
    for (int i = 0; i < B.face_count(); ++i)
        std::printf("[SD-DUMP] %s B f%d osign=%+.0f nsamp=%zu state=%s\n", tag.c_str(), i, osB[i],
                    SameDomain::samples_in_face(B, i, 16).size(),
                    st_name(sd_classify_face(B, i, A, TOL, osB, osA)));
}

///////////////////////////////////////////////////////////////////////////////////////////
// K — canonical keys / edge signatures
///////////////////////////////////////////////////////////////////////////////////////////

static void run_keys() {
    BRep box = BRep::create_box(1, 1, 1);
    cell("K1 key_self_equal",
         SameDomain::face_key(box, 0, TOL) == SameDomain::face_key(box, 0, TOL));
    {
        std::set<SDFaceKey> keys;
        for (int fi = 0; fi < box.face_count(); ++fi)
            keys.insert(SameDomain::face_key(box, fi, TOL));
        cell("K2 key_distinct_faces", (int)keys.size() == box.face_count(),
             "distinct=" + std::to_string(keys.size()));
    }
    {
        BRep twin = BRep::create_box(1, 1, 1);
        bool all = true;
        for (int fi = 0; fi < box.face_count(); ++fi)
            all = all && SameDomain::face_key(box, fi, TOL) == SameDomain::face_key(twin, fi, TOL);
        cell("K3 key_twin_match", all);
    }
    {
        BRep moved = place(BRep::create_box(1, 1, 1), 0.1, 0, 0);
        bool none = true;
        for (int fi = 0; fi < box.face_count(); ++fi)
            none = none && !(SameDomain::face_key(box, fi, TOL) == SameDomain::face_key(moved, fi, TOL));
        cell("K4 key_translated_differs", none);
    }
    {
        SDFaceKey kb = SameDomain::face_key(box, 0, TOL);
        BRep sph = BRep::create_sphere(1);
        SDFaceKey ks = SameDomain::face_key(sph, 0, TOL);
        // BOPTools_Set semantics: degenerate edges SKIPPED, multiplicity COUNTED. The sphere's
        // two poles are degenerate (excluded); its seam edge is used twice by the wire, so it
        // contributes 2 -- a deduplicating key would report 1 and collide faces OCCT separates.
        cell("K5 sig_counts_degenerate_excluded", (int)kb.sigs.size() == 4 && (int)ks.sigs.size() == 2,
             "box_f0=" + std::to_string(kb.sigs.size()) + " sph_f0=" + std::to_string(ks.sigs.size()));
    }
    {
        // Multiplicity-sensitivity, stated directly: the key is a MULTISET of edge signatures.
        BRep sph = BRep::create_sphere(1);
        SDFaceKey ks = SameDomain::face_key(sph, 0, TOL);
        bool dup = ks.sigs.size() == 2 && ks.sigs[0] == ks.sigs[1];
        cell("K6 key_multiplicity_sensitive", dup,
             dup ? "seam counted twice" : "seam deduplicated (BOPTools_Set divergence)");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// D — SD detection (AreFacesSameDomain analog + classes)
///////////////////////////////////////////////////////////////////////////////////////////

static void run_detect() {
    BRep box = BRep::create_box(1, 1, 1);
    {
        BRep twin = BRep::create_box(1, 1, 1);
        SameDomain sd = detect_pair(box, twin);
        cell("D1 identical_boxes_6_pairs",
             (int)sd.pairs().size() == 6 && count_orient(sd, 0) == 6 && sd.group_count() == 6,
             "pairs=" + std::to_string(sd.pairs().size()) +
                 " same=" + std::to_string(count_orient(sd, 0)));
        bool via_planar = true;
        for (const auto& p : sd.pairs()) via_planar = via_planar && p.via == 0;
        cell("D2 planar_shortcut_used", via_planar);
        bool minrep = true;
        for (const auto& p : sd.pairs()) minrep = minrep && sd.rep(p.j) == p.i && sd.rep(p.i) == p.i;
        cell("D3 min_index_representative", minrep);
        auto gs = sd.groups();
        bool g2 = (int)gs.size() == 6;
        for (const auto& g : gs) g2 = g2 && (int)g.size() == 2;
        cell("D4 groups_6x2", g2);
    }
    {
        BRep twin = BRep::create_box(1, 1, 1);
        SameDomain sd = detect_pair(box, twin, 0, 0);   // same solid id
        cell("D5 same_solid_guard", sd.pairs().empty(),
             "pairs=" + std::to_string(sd.pairs().size()));
    }
    {
        BRep A = BRep::create_cylinder(1, 2), B = BRep::create_cylinder(1, 2);
        SameDomain sd = detect_pair(A, B);
        cell("D6 identical_cylinders_all_faces",
             (int)sd.pairs().size() == A.face_count() && count_orient(sd, 0) == A.face_count(),
             "pairs=" + std::to_string(sd.pairs().size()) + "/" + std::to_string(A.face_count()));
    }
    {
        BRep A = BRep::create_sphere(1), B = BRep::create_sphere(1);
        SameDomain sd = detect_pair(A, B);
        bool geom = !sd.pairs().empty();
        for (const auto& p : sd.pairs()) geom = geom && p.via == 1;
        cell("D7 identical_spheres_geometric_path",
             (int)sd.pairs().size() == A.face_count() && count_orient(sd, 0) == A.face_count() && geom,
             "pairs=" + std::to_string(sd.pairs().size()) + "/" + std::to_string(A.face_count()));
    }
    {
        BRep B = place(BRep::create_box(1, 1, 1), 0, 0, 1);   // stacked: shared wall z=0.5
        SameDomain sd = detect_pair(box, B);
        cell("D8 stacked_boxes_wall_opposite",
             (int)sd.pairs().size() == 1 && count_orient(sd, 1) == 1,
             "pairs=" + std::to_string(sd.pairs().size()) +
                 " opp=" + std::to_string(count_orient(sd, 1)));
    }
    {
        BRep A = BRep::create_cylinder(1, 2);
        BRep B = place(BRep::create_cylinder(1, 2), 0, 0, 2);   // ZD4 flush caps
        SameDomain sd = detect_pair(A, B);
        cell("D9 flush_caps_ZD4_opposite",
             (int)sd.pairs().size() == 1 && count_orient(sd, 1) == 1,
             "pairs=" + std::to_string(sd.pairs().size()));
    }
    {
        BRep A = BRep::create_cylinder(1, 2);
        BRep B = place(BRep::create_cylinder(1, 2), 0, 0, 2, 2, 90);   // ZD5 seam twin
        SameDomain sd = detect_pair(A, B);
        cell("D10 seam_twin_ZD5_cap_found",
             (int)sd.pairs().size() == 1 && count_orient(sd, 1) == 1,
             "pairs=" + std::to_string(sd.pairs().size()));
    }
    {
        BRep A = BRep::create_box(2, 2, 1);
        BRep B = place(BRep::create_box(1, 1, 1), 0, 0, 1);   // partial coplanar overlap
        SameDomain sd = detect_pair(A, B);
        cell("D11 partial_overlap_no_pair", sd.pairs().empty(),
             "pairs=" + std::to_string(sd.pairs().size()));
    }
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_cylinder(1, 4), 3, 0, -2);   // ZO2-like line tangency
        SameDomain sd = detect_pair(A, B);
        cell("D12 line_contact_no_pair", sd.pairs().empty(),
             "pairs=" + std::to_string(sd.pairs().size()));
    }
    {
        BRep B = place(BRep::create_box(1, 1, 1), 5, 0, 0);
        SameDomain sd = detect_pair(box, B);
        cell("D13 disjoint_no_pair", sd.pairs().empty());
    }
    {
        BRep A = BRep::create_torus(2, 0.5), B = BRep::create_torus(2, 0.5);
        SameDomain sd = detect_pair(A, B);
        cell("D14 identical_tori_all_faces",
             (int)sd.pairs().size() == A.face_count() && count_orient(sd, 0) == A.face_count(),
             "pairs=" + std::to_string(sd.pairs().size()) + "/" + std::to_string(A.face_count()));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// T — BuildBOP orientation op-table (pure table, verified vs BOPAlgo_Builder.cxx:641-737)
///////////////////////////////////////////////////////////////////////////////////////////

static void run_table() {
    using V = SDVerdict;
    using S = SDState;
    auto t = [](SDOp op, int operand, S s) { return sd_select_face(op, operand, s); };
    cell("T1 table_fuse",
         t(SDOp::Fuse, 0, S::Out) == V::Keep && t(SDOp::Fuse, 1, S::Out) == V::Keep &&
         t(SDOp::Fuse, 0, S::In) == V::Drop && t(SDOp::Fuse, 1, S::In) == V::Drop &&
         t(SDOp::Fuse, 0, S::OnSame) == V::Keep && t(SDOp::Fuse, 1, S::OnSame) == V::Drop &&
         t(SDOp::Fuse, 0, S::OnOpposite) == V::Drop && t(SDOp::Fuse, 1, S::OnOpposite) == V::Drop);
    cell("T2 table_common",
         t(SDOp::Common, 0, S::In) == V::Keep && t(SDOp::Common, 1, S::In) == V::Keep &&
         t(SDOp::Common, 0, S::Out) == V::Drop && t(SDOp::Common, 1, S::Out) == V::Drop &&
         t(SDOp::Common, 0, S::OnSame) == V::Keep && t(SDOp::Common, 1, S::OnSame) == V::Drop &&
         t(SDOp::Common, 0, S::OnOpposite) == V::Drop && t(SDOp::Common, 1, S::OnOpposite) == V::Drop);
    cell("T3 table_cut",
         t(SDOp::Cut, 0, S::Out) == V::Keep && t(SDOp::Cut, 0, S::In) == V::Drop &&
         t(SDOp::Cut, 1, S::In) == V::KeepReversed && t(SDOp::Cut, 1, S::Out) == V::Drop &&
         t(SDOp::Cut, 0, S::OnOpposite) == V::Keep && t(SDOp::Cut, 1, S::OnOpposite) == V::Drop &&
         t(SDOp::Cut, 0, S::OnSame) == V::Drop && t(SDOp::Cut, 1, S::OnSame) == V::Drop);
    cell("T4 table_cut21",
         t(SDOp::Cut21, 1, S::Out) == V::Keep && t(SDOp::Cut21, 1, S::In) == V::Drop &&
         t(SDOp::Cut21, 0, S::In) == V::KeepReversed && t(SDOp::Cut21, 0, S::Out) == V::Drop &&
         t(SDOp::Cut21, 1, S::OnOpposite) == V::Keep && t(SDOp::Cut21, 0, S::OnOpposite) == V::Drop &&
         t(SDOp::Cut21, 0, S::OnSame) == V::Drop && t(SDOp::Cut21, 1, S::OnSame) == V::Drop);
}

///////////////////////////////////////////////////////////////////////////////////////////
// C — ON-classification (in / out / ON-same / ON-opposite)
///////////////////////////////////////////////////////////////////////////////////////////

static void run_classify() {
    BRep box = BRep::create_box(1, 1, 1);
    std::vector<double> os_box = box.face_outward_signs();
    {
        BRep twin = BRep::create_box(1, 1, 1);
        std::vector<double> os_twin = twin.face_outward_signs();
        bool all = true;
        for (int fi = 0; fi < box.face_count(); ++fi)
            all = all && sd_classify_face(box, fi, twin, TOL, os_box, os_twin) == SDState::OnSame;
        cell("C1 identical_boxes_all_OnSame", all);
    }
    {
        BRep B = place(BRep::create_box(1, 1, 1), 0, 0, 1);
        std::vector<double> osB = B.face_outward_signs();
        int n_on_opp = 0, n_out = 0;
        SDState top = SDState::Out;
        for (int fi = 0; fi < box.face_count(); ++fi) {
            SDState s = sd_classify_face(box, fi, B, TOL, os_box, osB);
            if (s == SDState::OnOpposite) { ++n_on_opp; top = s; }
            if (s == SDState::Out) ++n_out;
        }
        cell("C2 stacked_top_OnOpp_rest_Out", n_on_opp == 1 && n_out == 5 && top == SDState::OnOpposite,
             "onopp=" + std::to_string(n_on_opp) + " out=" + std::to_string(n_out));
    }
    {
        BRep big = BRep::create_box(2, 2, 2);
        std::vector<double> os_big = big.face_outward_signs();
        bool all_in = true;
        for (int fi = 0; fi < box.face_count(); ++fi)
            all_in = all_in && sd_classify_face(box, fi, big, TOL, os_box, os_big) == SDState::In;
        cell("C3 nested_small_box_all_In", all_in);
    }
    {
        // ZN4: cylinder embedded in box, caps flush with box top/bottom walls
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_cylinder(1, 4), 0, 0, -2);
        std::vector<double> osA = A.face_outward_signs();
        std::vector<double> osB = B.face_outward_signs();
        int n_on_same = 0, n_in = 0;
        for (int fi = 0; fi < B.face_count(); ++fi) {
            SDState s = sd_classify_face(B, fi, A, TOL, osB, osA);
            if (s == SDState::OnSame) ++n_on_same;
            if (s == SDState::In) ++n_in;
        }
        cell("C4 ZN4_flush_caps_OnSame_wall_In",
             n_on_same == 2 && n_in == B.face_count() - 2,
             "onsame=" + std::to_string(n_on_same) + " in=" + std::to_string(n_in));
        // partial-overlap doctrine: the box top WALL as a whole samples Out (fragment-level
        // states appear only post-split; see kb/p3_integration_notes.md)
        int n_wall_on = 0;
        for (int fi = 0; fi < A.face_count(); ++fi)
            if (sd_classify_face(A, fi, B, TOL, osA, osB) != SDState::Out) ++n_wall_on;
        cell("C5 ZN4_whole_walls_sample_Out", n_wall_on == 0,
             "non_out=" + std::to_string(n_wall_on));
    }
    {
        // ZP1-like: cylinder standing on the box top face (outside), base cap flush
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_cylinder(1, 4), 1, 1, 2);
        std::vector<double> osA = A.face_outward_signs();
        std::vector<double> osB = B.face_outward_signs();
        int n_on_opp = 0, n_out = 0;
        for (int fi = 0; fi < B.face_count(); ++fi) {
            SDState s = sd_classify_face(B, fi, A, TOL, osB, osA);
            if (s == SDState::OnOpposite) ++n_on_opp;
            if (s == SDState::Out) ++n_out;
        }
        cell("C6 ZP1_base_cap_OnOpp_rest_Out",
             n_on_opp == 1 && n_out == B.face_count() - 1,
             "onopp=" + std::to_string(n_on_opp) + " out=" + std::to_string(n_out));
    }
    {
        // external line tangency (ZO2 family): ON must NOT fire on measure-zero contact
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_cylinder(1, 4), 3, 0, -2);
        std::vector<double> osA = A.face_outward_signs();
        std::vector<double> osB = B.face_outward_signs();
        bool all_out = true;
        for (int fi = 0; fi < B.face_count(); ++fi)
            all_out = all_out && sd_classify_face(B, fi, A, TOL, osB, osA) == SDState::Out;
        for (int fi = 0; fi < A.face_count(); ++fi)
            all_out = all_out && sd_classify_face(A, fi, B, TOL, osA, osB) == SDState::Out;
        cell("C7 line_contact_all_Out", all_out);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// E — coincidence-only boolean assembly (sd_boolean_coincident)
///////////////////////////////////////////////////////////////////////////////////////////

static void check_solid_result(const std::string& name, const BRep& R, int want_faces,
                               double want_vol, double rel = 1e-9) {
    double vol = R.face_count() > 0 ? R.volume() : 0.0;
    int nk = naked_edges(R);
    bool ok = R.face_count() == want_faces && nk == 0 && R.is_solid() &&
              std::abs(vol - want_vol) <= rel * std::max(1.0, std::abs(want_vol));
    char buf[160];
    std::snprintf(buf, sizeof buf, "faces=%d/%d naked=%d solid=%d vol=%.9f/%.9f",
                  R.face_count(), want_faces, nk, R.is_solid() ? 1 : 0, vol, want_vol);
    cell(name, ok, buf);
}

static void check_empty_result(const std::string& name, const BRep& R) {
    cell(name, R.face_count() == 0, "faces=" + std::to_string(R.face_count()));
}

static void run_aopa(const std::string& tag, const BRep& A) {
    double vol = A.volume();
    BRep common = sd_boolean_coincident(A, A, SDOp::Common, TOL);
    check_solid_result("E " + tag + "_AopA_common", common, A.face_count(), vol);
    BRep fuse = sd_boolean_coincident(A, A, SDOp::Fuse, TOL);
    check_solid_result("E " + tag + "_AopA_fuse", fuse, A.face_count(), vol);
    BRep cut = sd_boolean_coincident(A, A, SDOp::Cut, TOL);
    check_empty_result("E " + tag + "_AopA_cut", cut);
}

static void run_booleans() {
    run_aopa("box", BRep::create_box(1, 1, 1));
    run_aopa("cyl", BRep::create_cylinder(1, 2));
    run_aopa("sphere", BRep::create_sphere(1));
    run_aopa("cone", BRep::create_cone(1, 2));
    run_aopa("torus", BRep::create_torus(2, 0.5));

    {
        BRep A = BRep::create_box(1, 1, 1);
        BRep B = place(BRep::create_box(1, 1, 1), 0, 0, 1);
        check_solid_result("E stacked_boxes_fuse",
                           sd_boolean_coincident(A, B, SDOp::Fuse, TOL), 10, 2.0);
        check_solid_result("E stacked_boxes_cut",
                           sd_boolean_coincident(A, B, SDOp::Cut, TOL), 6, 1.0);
        check_empty_result("E stacked_boxes_common",
                           sd_boolean_coincident(A, B, SDOp::Common, TOL));
        check_solid_result("E stacked_boxes_cut21",
                           sd_boolean_coincident(A, B, SDOp::Cut21, TOL), 6, 1.0);
    }
    {
        BRep A = BRep::create_box(1, 1, 1);
        BRep B = place(BRep::create_box(1, 1, 1), 1, 0, 0);
        check_solid_result("E side_boxes_fuse",
                           sd_boolean_coincident(A, B, SDOp::Fuse, TOL), 10, 2.0);
        check_solid_result("E side_boxes_cut",
                           sd_boolean_coincident(A, B, SDOp::Cut, TOL), 6, 1.0);
    }
    {
        // ZD4: stacked equal cylinders, cap-on-cap
        BRep A = BRep::create_cylinder(1, 2);
        BRep B = place(BRep::create_cylinder(1, 2), 0, 0, 2);
        double va = A.volume(), vb = B.volume();
        dump_pair("ZD4", A, B);
        BRep fuse = sd_boolean_coincident(A, B, SDOp::Fuse, TOL);
        check_solid_result("E ZD4_stacked_cyls_fuse", fuse,
                           A.face_count() + B.face_count() - 2, va + vb);
        BRep cut = sd_boolean_coincident(A, B, SDOp::Cut, TOL);
        check_solid_result("E ZD4_stacked_cyls_cut", cut, A.face_count(), va);
        check_empty_result("E ZD4_stacked_cyls_common",
                           sd_boolean_coincident(A, B, SDOp::Common, TOL));
    }
    {
        // ZD5 seam twin: B rotated 90 about its own axis; results must match ZD4
        BRep A = BRep::create_cylinder(1, 2);
        BRep B = place(BRep::create_cylinder(1, 2), 0, 0, 2, 2, 90);
        double va = A.volume(), vb = B.volume();
        BRep fuse = sd_boolean_coincident(A, B, SDOp::Fuse, TOL);
        check_solid_result("E ZD5_seam_twin_fuse", fuse,
                           A.face_count() + B.face_count() - 2, va + vb);
    }
    {
        // nested boxes: cavity cut + common + empty cut21
        BRep A = BRep::create_box(2, 2, 2);
        BRep B = BRep::create_box(1, 1, 1);
        BRep cut = sd_boolean_coincident(A, B, SDOp::Cut, TOL);
        double vol = cut.face_count() > 0 ? cut.volume() : 0.0;
        char buf[96];
        std::snprintf(buf, sizeof buf, "faces=%d vol=%.9f", cut.face_count(), vol);
        cell("E nested_cut_cavity", cut.face_count() == 12 && std::abs(vol - 7.0) <= 1e-6, buf);
        check_solid_result("E nested_common", sd_boolean_coincident(A, B, SDOp::Common, TOL),
                           B.face_count(), B.volume());
        check_empty_result("E nested_cut21", sd_boolean_coincident(A, B, SDOp::Cut21, TOL));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// P — PARTIAL coincidence: common blocks, per-region splitting, region op-table
///////////////////////////////////////////////////////////////////////////////////////////

static NurbsCurve seg(const Point& a, const Point& b) {
    return NurbsCurve::create(false, 1, {a, b});
}

// Fraction of the chain covered by coincident blocks.
static double coincident_frac(const CBSplit& sp) {
    if (sp.blocks.empty()) return 0.0;
    double tot = sp.blocks.back().t1 - sp.blocks.front().t0, cov = 0.0;
    for (const auto& b : sp.blocks)
        if (b.coincident) cov += b.length_param();
    return tot > 0 ? cov / tot : 0.0;
}

// Parameter fraction along the chain at which the FIRST coincidence transition happens.
static double first_transition_frac(const CBSplit& sp) {
    if (sp.blocks.size() < 2) return -1.0;
    double t0 = sp.blocks.front().t0, t1 = sp.blocks.back().t1;
    return t1 > t0 ? (sp.blocks[0].t1 - t0) / (t1 - t0) : -1.0;
}

static void run_partial() {
    const double band = 1e-6;
    CBParams prm;
    prm.band = band;
    prm.on = CBOn::Face;

    // ---- FC03: two 4x4x4 boxes sharing HALF a face. A spans y[-2,2], B spans y[0,4] on x=2.
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 4, 2, 0);
        // Chain along the shared plane x=2, z=0, running y from -2 (A only) to 4 (B only).
        NurbsCurve c = seg(Point(2, -2, 0), Point(2, 4, 0));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        // Expect 3 blocks: [-2,0] A-only, [0,2] COINCIDENT, [2,4] B-only.
        bool shape = sp.blocks.size() == 3 && !sp.blocks[0].coincident &&
                     sp.blocks[1].coincident && !sp.blocks[2].coincident;
        double f0 = first_transition_frac(sp);
        bool cuts = std::abs(f0 - 1.0 / 3.0) < 1e-4 && std::abs(coincident_frac(sp) - 1.0 / 3.0) < 1e-4;
        char buf[160];
        std::snprintf(buf, sizeof buf, "blocks=%zu coincident=%d cut@%.6f frac=%.6f",
                      sp.blocks.size(), sp.n_coincident, f0, coincident_frac(sp));
        cell("P1 FC03_half_face_split_3_blocks", shape && cuts && sp.n_coincident == 1, buf);
        // The synthesised boundary paves must be exactly the region ends (y=0 and y=2).
        Point p0 = sp.blocks[1].point_at_frac(0.0), p1 = sp.blocks[1].point_at_frac(1.0);
        bool paves = std::abs(p0[1] - 0.0) < 1e-4 && std::abs(p1[1] - 2.0) < 1e-4;
        std::snprintf(buf, sizeof buf, "y0=%.6f y1=%.6f", p0[1], p1[1]);
        cell("P2 FC03_paves_at_region_boundary", paves, buf);
        // Whole-chain logic cannot express this: the chain is neither wholly on nor wholly off.
        cell("P3 FC03_not_whole_chain_decidable",
             coincident_frac(sp) > 0.0 && coincident_frac(sp) < 1.0);
    }

    // ---- FC02: B's face strictly INSIDE A's larger face -> chain enters and leaves the region.
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(2, 2, 2), 3, 0, 0);   // face x=2 spans y,z in [-1,1]
        NurbsCurve c = seg(Point(2, -2, 0), Point(2, 2, 0));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        bool shape = sp.blocks.size() == 3 && !sp.blocks[0].coincident &&
                     sp.blocks[1].coincident && !sp.blocks[2].coincident;
        Point p0 = sp.blocks.size() == 3 ? sp.blocks[1].point_at_frac(0.0) : Point(0, 0, 0);
        Point p1 = sp.blocks.size() == 3 ? sp.blocks[1].point_at_frac(1.0) : Point(0, 0, 0);
        bool paves = std::abs(p0[1] + 1.0) < 1e-4 && std::abs(p1[1] - 1.0) < 1e-4;
        char buf[160];
        std::snprintf(buf, sizeof buf, "blocks=%zu y0=%.6f y1=%.6f", sp.blocks.size(), p0[1], p1[1]);
        cell("P4 FC02_inner_region_split", shape && paves, buf);
    }

    // ---- Two cylinders sharing a PARTIAL lateral strip (FC21 class): coaxial, overlapping z.
    {
        BRep A = BRep::create_cylinder(1, 4);                       // z in [0,4]
        BRep B = place(BRep::create_cylinder(1, 4), 0, 0, 2);       // z in [2,6]
        // Chain running UP the shared lateral surface at theta=0, from z=0 to z=6.
        NurbsCurve c = seg(Point(1, 0, 0), Point(1, 0, 6));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        // Coincident strip is z in [2,4] = middle third of the chain.
        bool shape = sp.blocks.size() == 3 && sp.blocks[1].coincident;
        Point p0 = sp.blocks.size() == 3 ? sp.blocks[1].point_at_frac(0.0) : Point(0, 0, 0);
        Point p1 = sp.blocks.size() == 3 ? sp.blocks[1].point_at_frac(1.0) : Point(0, 0, 0);
        bool zs = std::abs(p0[2] - 2.0) < 1e-4 && std::abs(p1[2] - 4.0) < 1e-4;
        char buf[160];
        std::snprintf(buf, sizeof buf, "blocks=%zu z0=%.6f z1=%.6f frac=%.6f",
                      sp.blocks.size(), p0[2], p1[2], coincident_frac(sp));
        cell("P5 FC21_cyl_partial_lateral_strip", shape && zs, buf);
    }

    // ---- Box face partly coincident with a cylinder's flat CAP (cap larger than the box face).
    {
        BRep A = BRep::create_box(2, 2, 2);                          // top face z=1, x,y in [-1,1]
        BRep B = place(BRep::create_cylinder(2, 2), 0, 0, 1);        // base cap z=1, radius 2
        // Chain across the shared plane z=1 at y=0, from x=-3 (outside both) to x=3.
        NurbsCurve c = seg(Point(-3, 0, 1), Point(3, 0, 1));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        // Coincident where BOTH: box face |x|<=1 AND cap x^2<=4 -> x in [-1,1] = middle third.
        int nc = sp.n_coincident;
        double cf = coincident_frac(sp);
        bool ok = nc == 1 && std::abs(cf - 1.0 / 3.0) < 1e-3;
        char buf[160];
        std::snprintf(buf, sizeof buf, "blocks=%zu coincident=%d frac=%.6f",
                      sp.blocks.size(), nc, cf);
        cell("P6 boxface_partly_on_cyl_cap", ok, buf);
    }

    // ---- Fully coincident chain: must yield ONE block, no transitions (suppression case).
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 4, 0, 0);   // full-face flush at x=2
        NurbsCurve c = seg(Point(2, -1, 0), Point(2, 1, 0));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        bool ok = sp.blocks.size() == 1 && sp.blocks[0].coincident && sp.n_transitions == 0;
        cell("P7 full_coincidence_single_block", ok,
             "blocks=" + std::to_string(sp.blocks.size()) +
                 " trans=" + std::to_string(sp.n_transitions));
    }

    // ---- No coincidence at all: ONE non-coincident block (never a spurious split).
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 20, 0, 0);
        NurbsCurve c = seg(Point(2, -1, 0), Point(2, 1, 0));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        cell("P8 no_coincidence_single_block",
             sp.blocks.size() == 1 && !sp.blocks[0].coincident && sp.n_coincident == 0,
             "blocks=" + std::to_string(sp.blocks.size()));
    }

    // ---- Tolerance: ComputeToleranceOfCB over a chain offset from the boundary by a known gap.
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 4, 0, 0);
        double gap = 1e-3;
        NurbsCurve c = seg(Point(2 + gap, -1, 0), Point(2 + gap, 1, 0));
        CBParams p2 = prm;
        p2.band = 1e-2;                       // wide enough that the offset chain still counts
        CBSplit sp = cb_split_chain(c, A, B, p2);
        bool one = sp.blocks.size() == 1 && sp.blocks[0].coincident;
        double tol = one ? sp.blocks[0].tolerance : -1.0;
        // tol = max over 11 interior samples of (tol(mate)=0 + distance) = the gap.
        bool ok = one && std::abs(tol - gap) < 1e-9;
        char buf[128];
        std::snprintf(buf, sizeof buf, "tol=%.9f gap=%.9f", tol, gap);
        cell("P9 cb_tolerance_equals_measured_gap", ok, buf);
    }

    // ---- PerformCommonBlocks grouping: two chains on the SAME shared face group together.
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 4, 0, 0);
        NurbsCurve c1 = seg(Point(2, -1, 0), Point(2, 1, 0));
        NurbsCurve c2 = seg(Point(2, -1, 1), Point(2, 1, 1));
        CBSplit s1 = cb_split_chain(c1, A, B, prm);
        CBSplit s2 = cb_split_chain(c2, A, B, prm);
        std::vector<CommonBlock> all;
        for (auto& b : s1.blocks) all.push_back(b);
        for (auto& b : s2.blocks) all.push_back(b);
        std::vector<int> grp = cb_perform_common_blocks(all);
        bool grouped = all.size() == 2 && grp[0] == 0 && grp[1] == 0;
        cell("P10 perform_common_blocks_groups_shared", grouped,
             "blocks=" + std::to_string(all.size()) + " grp=" +
                 (grp.size() == 2 ? std::to_string(grp[0]) + "," + std::to_string(grp[1]) : "?"));
    }
    {
        // Singleton groups are skipped, exactly as OCCT does (< 2 PBs -> no common block).
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 20, 0, 0);
        NurbsCurve c = seg(Point(2, -1, 0), Point(2, 1, 0));
        CBSplit sp = cb_split_chain(c, A, B, prm);
        std::vector<CommonBlock> all(sp.blocks.begin(), sp.blocks.end());
        std::vector<int> grp = cb_perform_common_blocks(all);
        cell("P11 singleton_group_skipped", grp.size() == 1 && grp[0] == -1);
    }

    // ---- Per-REGION face partition: a partially coincident face splits into >= 2 states.
    {
        BRep A = BRep::create_box(4, 4, 4);
        BRep B = place(BRep::create_box(4, 4, 4), 4, 2, 0);   // half-face overlap at x=2
        std::vector<double> osA = A.face_outward_signs(), osB = B.face_outward_signs();
        // Find A's face whose interior point sits on the x=2 plane.
        int fx = -1;
        for (int fi = 0; fi < A.face_count(); ++fi) {
            Point p; double u, v;
            if (SameDomain::point_in_face(A, fi, p, u, v) && std::abs(p[0] - 2.0) < 1e-9) fx = fi;
        }
        auto regs = cb_partition_face(A, fx, 0, B, band, 64, osA, osB);
        int n_on = 0, n_out = 0;
        for (const auto& r : regs) {
            if (r.state == SDState::OnOpposite || r.state == SDState::OnSame) ++n_on;
            if (r.state == SDState::Out) ++n_out;
        }
        char buf[128];
        std::snprintf(buf, sizeof buf, "face=%d regions=%zu on=%d out=%d", fx, regs.size(), n_on, n_out);
        cell("P12 partial_face_partitions_into_regions",
             fx >= 0 && regs.size() >= 2 && n_on == 1 && n_out == 1, buf);

        // Region op-table: the SAME face now yields DIFFERENT verdicts per region -- the whole
        // point of per-region granularity. Fuse: ON-opposite region drops, Out region keeps.
        SDVerdict v_on = SDVerdict::Drop, v_out = SDVerdict::Drop;
        for (const auto& r : regs) {
            if (r.state == SDState::OnOpposite) v_on = cb_select_region(SDOp::Fuse, r);
            if (r.state == SDState::Out) v_out = cb_select_region(SDOp::Fuse, r);
        }
        cell("P13 region_op_table_differs_within_face",
             v_on == SDVerdict::Drop && v_out == SDVerdict::Keep);
        // Cut(A,B): the ON-opposite region is the wall CUT keeps once, from the objects' side.
        SDVerdict c_on = SDVerdict::Drop;
        for (const auto& r : regs)
            if (r.state == SDState::OnOpposite) c_on = cb_select_region(SDOp::Cut, r);
        cell("P14 region_op_table_cut_keeps_ON_opposite", c_on == SDVerdict::Keep);
    }
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    run_keys();
    run_detect();
    run_table();
    run_classify();
    run_booleans();
    run_partial();
    std::printf("[SD] TOTAL %d/%d\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
