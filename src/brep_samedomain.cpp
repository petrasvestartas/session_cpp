#include "brep_samedomain.h"
#include "closest.h"
#include "plane.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>

namespace session_cpp {

namespace {

constexpr double SD_CONFUSION = 1e-7;   // Precision::Confusion analog
// IntTools_Tools::IntermediatePoint PAR_T = 10*e^-pi (IntTools_Tools.cxx:254-259). OCCT uses it
// for the interior-point probe, for the probe parameter inside the hatch domain, and for
// IsValidBlockForFaces midpoints -- deliberately NOT 0.5, because a true midpoint lands on
// symmetric coincidences (seams, symmetry planes, and -- measured here -- the exact boundary of
// a half-face coincidence region, where the nearest-face tie makes the ON verdict a coin flip).
constexpr double SD_PAR_T = 0.43213918;
constexpr int SD_CLOSED_SAMPLES = 48;   // divisible by 4: quadrant-symmetric circle sampling

inline int64_t quant(double x, double grid) { return (int64_t)std::llround(x / grid); }

// Raw (unquantized) edge signature for the tolerant fallback matching.
struct RawSig {
    bool closed = false;
    double a[3] = {0, 0, 0}, b[3] = {0, 0, 0}, m[3] = {0, 0, 0};
};

bool raw_sig(const BRep& br, int edge, double grid, RawSig& rs, Point* smin, Point* smax) {
    if (edge < 0 || edge >= (int)br.m_topology_edges.size()) return false;
    const BRepEdge& e = br.m_topology_edges[edge];
    if (e.curve_3d_index < 0 || e.curve_3d_index >= (int)br.m_curves_3d.size()) return false;
    const NurbsCurve& c = br.m_curves_3d[e.curve_3d_index];
    auto dm = c.domain();
    int n = SD_CLOSED_SAMPLES;
    std::vector<Point> s(n + 1);
    double xmin = 1e300, ymin = 1e300, zmin = 1e300, xmax = -1e300, ymax = -1e300, zmax = -1e300;
    for (int i = 0; i <= n; ++i) {
        s[i] = c.point_at(dm.first + (dm.second - dm.first) * i / n);
        xmin = std::min(xmin, s[i][0]); ymin = std::min(ymin, s[i][1]); zmin = std::min(zmin, s[i][2]);
        xmax = std::max(xmax, s[i][0]); ymax = std::max(ymax, s[i][1]); zmax = std::max(zmax, s[i][2]);
    }
    if (smin) *smin = Point(xmin, ymin, zmin);
    if (smax) *smax = Point(xmax, ymax, zmax);
    double diag = std::sqrt((xmax - xmin) * (xmax - xmin) + (ymax - ymin) * (ymax - ymin) +
                            (zmax - zmin) * (zmax - zmin));
    if (diag < grid) return false;   // degenerate edge (pole/apex) -- excluded like OCCT
    rs.closed = s[0].distance(s[n]) < grid;
    if (rs.closed) {
        // phase-independent: sample centroid + radius stats (seam-twin invariance)
        double cx = 0, cy = 0, cz = 0;
        for (int i = 0; i < n; ++i) { cx += s[i][0]; cy += s[i][1]; cz += s[i][2]; }
        cx /= n; cy /= n; cz /= n;
        double rmean = 0, rmin = 1e300, rmax = 0;
        for (int i = 0; i < n; ++i) {
            double r = std::sqrt((s[i][0] - cx) * (s[i][0] - cx) + (s[i][1] - cy) * (s[i][1] - cy) +
                                 (s[i][2] - cz) * (s[i][2] - cz));
            rmean += r; rmin = std::min(rmin, r); rmax = std::max(rmax, r);
        }
        rmean /= n;
        rs.a[0] = cx; rs.a[1] = cy; rs.a[2] = cz;
        rs.b[0] = cx; rs.b[1] = cy; rs.b[2] = cz;
        rs.m[0] = rmean; rs.m[1] = rmax - rmin; rs.m[2] = 0.0;
    } else {
        const Point& p0 = s[0];
        const Point& p1 = s[n];
        Point pm = s[n / 2];   // curve reversal maps the domain midpoint to itself
        bool swap = false;
        for (int i = 0; i < 3; ++i) {
            int64_t q0 = quant(p0[i], grid), q1 = quant(p1[i], grid);
            if (q0 != q1) { swap = q0 > q1; break; }
        }
        const Point& pa = swap ? p1 : p0;
        const Point& pb = swap ? p0 : p1;
        for (int i = 0; i < 3; ++i) { rs.a[i] = pa[i]; rs.b[i] = pb[i]; rs.m[i] = pm[i]; }
    }
    return true;
}

SDEdgeSig quantize_sig(const RawSig& rs, double grid) {
    SDEdgeSig sig;
    for (int i = 0; i < 3; ++i) {
        sig.k[i] = quant(rs.a[i], grid);
        sig.k[3 + i] = quant(rs.b[i], grid);
        sig.k[6 + i] = quant(rs.m[i], grid);
    }
    return sig;
}

// Edge USES of a face, one entry per trim -- MULTIPLICITY-SENSITIVE, matching BOPTools_Set
// (BOPTools_Set.cxx:120-172): equality is "equal count + IsSame containment", so an edge that
// appears twice in the wires (a seam) counts as 2 on both sides. Deduplicating here would make
// the key multiplicity-insensitive and collide faces OCCT keeps distinct.
// (OCCT additionally inserts an INTERNAL-oriented edge twice; this kernel has no INTERNAL
// orientation -- BRepTrimType is Boundary/Mated/Seam/Singular -- so that rule is vacuous here.)
std::vector<int> face_edge_uses(const BRep& br, int face) {
    std::vector<int> out;
    if (face < 0 || face >= (int)br.m_faces.size()) return out;
    for (int li : br.m_faces[face].loop_indices) {
        if (li < 0 || li >= (int)br.m_loops.size()) continue;
        for (int ti : br.m_loops[li].trim_indices) {
            if (ti < 0 || ti >= (int)br.m_trims.size()) continue;
            int ei = br.m_trims[ti].edge_index;
            if (ei < 0) continue;
            out.push_back(ei);
        }
    }
    return out;
}

// Even-odd point-in-material test on a face's sampled trim loops (outer minus holes).
bool uv_in_trims(const BRep& br, int fi, double u, double v) {
    const BRepFace& face = br.m_faces[fi];
    bool in_outer = false, have_outer = false;
    for (int li : face.loop_indices) {
        if (li < 0 || li >= (int)br.m_loops.size()) continue;
        const BRepLoop& loop = br.m_loops[li];
        std::vector<std::array<double, 2>> poly;
        for (int ti : loop.trim_indices) {
            if (ti < 0 || ti >= (int)br.m_trims.size()) continue;
            int c2 = br.m_trims[ti].curve_2d_index;
            if (c2 < 0 || c2 >= (int)br.m_curves_2d.size()) continue;
            const NurbsCurve& pc = br.m_curves_2d[c2];
            auto dc = pc.domain();
            int ns = std::min(std::max(pc.cv_count() * 2, 16), 128);
            for (int k = 0; k < ns; ++k) {
                Point q = pc.point_at(dc.first + (dc.second - dc.first) * k / ns);
                poly.push_back({q[0], q[1]});
            }
        }
        if (poly.size() < 3) continue;
        bool inp = false;
        for (size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++)
            if (((poly[a][1] > v) != (poly[b][1] > v)) &&
                (u < (poly[b][0] - poly[a][0]) * (v - poly[a][1]) /
                         (poly[b][1] - poly[a][1] + 1e-30) + poly[a][0]))
                inp = !inp;
        if (loop.type == BRepLoopType::Outer) { in_outer = inp; have_outer = true; }
        else if (inp) return false;
    }
    return have_outer ? in_outer : false;
}

// Closest point of p on the 3D trim boundary of face fi (sampled). Returns distance.
double closest_on_trim(const BRep& br, int fi, const Point& p, double& ou, double& ov) {
    int si = br.m_faces[fi].surface_index;
    const NurbsSurface& S = br.m_surfaces[si];
    double bd = 1e300;
    for (int li : br.m_faces[fi].loop_indices) {
        if (li < 0 || li >= (int)br.m_loops.size()) continue;
        for (int ti : br.m_loops[li].trim_indices) {
            if (ti < 0 || ti >= (int)br.m_trims.size()) continue;
            int c2 = br.m_trims[ti].curve_2d_index;
            if (c2 < 0 || c2 >= (int)br.m_curves_2d.size()) continue;
            const NurbsCurve& pc = br.m_curves_2d[c2];
            auto dc = pc.domain();
            int ns = std::min(std::max(pc.cv_count(), 8), 24);
            for (int k = 0; k <= ns; ++k) {
                Point q = pc.point_at(dc.first + (dc.second - dc.first) * k / ns);
                double dd = S.point_at(q[0], q[1]).distance(p);
                if (dd < bd) { bd = dd; ou = q[0]; ov = q[1]; }
            }
        }
    }
    return bd;
}

// Nearest point on the TRIMMED boundary of `other` (contains_point_exact doctrine) with
// tie-robust face pick: among near-equal distances take the face with the largest |dp|.
struct NearestHit {
    double d = 1e300;
    int fb = -1;
    double u = 0, v = 0;
    double dp = 0;          // (p - q) . outward normal at q
    Vector nout = Vector(0, 0, 0);
    bool on_boundary = false;   // q lies on the face's TRIM BOUNDARY (rim) -> dp sign invalid
};

NearestHit nearest_on_solid(const BRep& other, const Point& p, const std::vector<double>& osign) {
    struct Cand { double d; int fb; double u, v; bool bnd; };
    std::vector<Cand> cands;
    double best_d = 1e300;
    for (int fb = 0; fb < (int)other.m_faces.size(); ++fb) {
        int si = other.m_faces[fb].surface_index;
        if (si < 0 || si >= (int)other.m_surfaces.size()) continue;
        auto [u, v, d] = Closest::surface_point(other.m_surfaces[si], p);
        bool bnd = false;
        if (!uv_in_trims(other, fb, u, v)) {
            double ou = u, ov = v;
            d = closest_on_trim(other, fb, p, ou, ov);
            u = ou; v = ov;
            bnd = true;   // nearest point is on an EDGE of this face, not its interior
        }
        cands.push_back({d, fb, u, v, bnd});
        best_d = std::min(best_d, d);
    }
    NearestHit hit;
    double band = std::max(1e-9, best_d * 1e-6);
    for (const auto& c : cands) {
        if (c.d > best_d + band) continue;
        int si = other.m_faces[c.fb].surface_index;
        Vector n = other.m_surfaces[si].normal_at(c.u, c.v);
        double s = (c.fb < (int)osign.size()) ? osign[c.fb] : 1.0;
        Point q = other.m_surfaces[si].point_at(c.u, c.v);
        double dp = (p[0] - q[0]) * n[0] * s + (p[1] - q[1]) * n[1] * s + (p[2] - q[2]) * n[2] * s;
        // Prefer an INTERIOR hit over an equidistant boundary hit: only an interior hit has a
        // meaningful signed side. Among equals, the largest |dp| is the most decisive.
        bool better = hit.fb < 0 || (hit.on_boundary && !c.bnd) ||
                      (hit.on_boundary == c.bnd && std::abs(dp) > std::abs(hit.dp));
        if (better) {
            hit.d = c.d; hit.fb = c.fb; hit.u = c.u; hit.v = c.v; hit.dp = dp;
            hit.nout = Vector(n[0] * s, n[1] * s, n[2] * s);
            hit.on_boundary = c.bnd;
        }
    }
    return hit;
}

} // namespace

///////////////////////////////////////////////////////////////////////////////////////////
// SameDomain
///////////////////////////////////////////////////////////////////////////////////////////

SameDomain::SameDomain(double key_tol, double fuzz)
    : m_key_tol(std::max(key_tol, 1e-12)), m_fuzz(fuzz) {}

int SameDomain::add_face(const BRep& b, int face, int operand, int solid, double tol) {
    SDFaceRec r;
    r.brep = &b;
    r.face = face;
    r.operand = operand;
    r.solid = solid < 0 ? operand : solid;
    r.tol = tol;
    m_faces.push_back(r);
    m_detected = false;
    return (int)m_faces.size() - 1;
}

void SameDomain::add_solid(const BRep& b, int operand, int solid) {
    for (int fi = 0; fi < (int)b.m_faces.size(); ++fi) add_face(b, fi, operand, solid);
}

int SameDomain::find(int i) const {
    while (m_rep[i] != i) i = m_rep[i];
    return i;
}

void SameDomain::detect() {
    int n = (int)m_faces.size();
    m_pairs.clear();
    m_rep.resize(n);
    for (int i = 0; i < n; ++i) m_rep[i] = i;

    // per-operand orientation caches (one face_outward_signs per distinct BRep)
    std::map<const BRep*, std::array<std::vector<double>, 1>> sign_cache;
    std::map<const BRep*, std::pair<std::vector<Point>, std::vector<Vector>>> pn_cache;
    for (auto& r : m_faces) {
        if (sign_cache.count(r.brep)) continue;
        std::vector<Point> P3s;
        std::vector<Vector> Ns;
        sign_cache[r.brep] = {r.brep->face_outward_signs(&P3s, &Ns)};
        pn_cache[r.brep] = {P3s, Ns};
    }

    // per-face payload: key, raw sigs, bbox, interior point, outward normal, planar
    std::vector<std::vector<RawSig>> raws(n);
    std::vector<double> osign_face(n, 1.0);
    for (int i = 0; i < n; ++i) {
        SDFaceRec& r = m_faces[i];
        const BRep& br = *r.brep;
        double xmin = 1e300, ymin = 1e300, zmin = 1e300;
        double xmax = -1e300, ymax = -1e300, zmax = -1e300;
        for (int ei : face_edge_uses(br, r.face)) {
            RawSig rs;
            Point smin, smax;
            if (!raw_sig(br, ei, m_key_tol, rs, &smin, &smax)) continue;
            raws[i].push_back(rs);
            r.key.sigs.push_back(quantize_sig(rs, m_key_tol));
            xmin = std::min(xmin, smin[0]); ymin = std::min(ymin, smin[1]); zmin = std::min(zmin, smin[2]);
            xmax = std::max(xmax, smax[0]); ymax = std::max(ymax, smax[1]); zmax = std::max(zmax, smax[2]);
        }
        std::sort(r.key.sigs.begin(), r.key.sigs.end());
        r.edge_count = (int)r.key.sigs.size();
        r.bmin = Point(xmin, ymin, zmin);
        r.bmax = Point(xmax, ymax, zmax);
        const auto& sgn = sign_cache[r.brep][0];
        const auto& pn = pn_cache[r.brep];
        double s = (r.face < (int)sgn.size()) ? sgn[r.face] : 1.0;
        osign_face[i] = s;
        if (r.face < (int)pn.first.size()) {
            r.pin = pn.first[r.face];
            const Vector& nn = pn.second[r.face];
            r.nout = Vector(nn[0] * s, nn[1] * s, nn[2] * s);
        }
        int si = br.m_faces[r.face].surface_index;
        r.planar = (si >= 0 && si < (int)br.m_surfaces.size()) &&
                   br.m_surfaces[si].is_planar(nullptr, SD_CONFUSION);
        r.bounded = true;
    }

    // candidate pairs: exact-key buckets (fast path)
    std::map<SDFaceKey, std::vector<int>> buckets;
    for (int i = 0; i < n; ++i)
        if (m_faces[i].edge_count > 0) buckets[m_faces[i].key].push_back(i);
    std::vector<std::pair<int, int>> cands;
    std::vector<char> seen(n * n, 0);
    auto push_cand = [&](int i, int j) {
        if (i > j) std::swap(i, j);
        if (seen[i * n + j]) return;
        seen[i * n + j] = 1;
        cands.push_back({i, j});
    };
    for (auto& kv : buckets)
        for (size_t a = 0; a + 1 < kv.second.size(); ++a)
            for (size_t b2 = a + 1; b2 < kv.second.size(); ++b2)
                push_cand(kv.second[a], kv.second[b2]);

    // tolerant fallback (quantization straddles): edge count + bbox + bipartite sig match
    auto sig_near = [&](const RawSig& x, const RawSig& y, double band) {
        if (x.closed != y.closed) return false;
        auto near3 = [&](const double* p, const double* q) {
            return std::abs(p[0] - q[0]) <= band && std::abs(p[1] - q[1]) <= band &&
                   std::abs(p[2] - q[2]) <= band;
        };
        if (!near3(x.m, y.m)) return false;
        return (near3(x.a, y.a) && near3(x.b, y.b)) || (near3(x.a, y.b) && near3(x.b, y.a));
    };
    for (int i = 0; i < n; ++i) {
        if (m_faces[i].edge_count == 0) continue;
        for (int j = i + 1; j < n; ++j) {
            if (seen[i * n + j]) continue;
            if (m_faces[j].edge_count != m_faces[i].edge_count) continue;
            double ti = m_faces[i].tol > 0 ? m_faces[i].tol : m_key_tol;
            double tj = m_faces[j].tol > 0 ? m_faces[j].tol : m_key_tol;
            double band = ti + tj + std::max(m_fuzz, SD_CONFUSION);
            bool boxok = true;
            for (int c = 0; c < 3 && boxok; ++c)
                boxok = std::abs(m_faces[i].bmin[c] - m_faces[j].bmin[c]) <= band &&
                        std::abs(m_faces[i].bmax[c] - m_faces[j].bmax[c]) <= band;
            if (!boxok) continue;
            std::vector<char> used(raws[j].size(), 0);
            bool all = true;
            for (const auto& rx : raws[i]) {
                bool got = false;
                for (size_t b2 = 0; b2 < raws[j].size() && !got; ++b2)
                    if (!used[b2] && sig_near(rx, raws[j][b2], band)) { used[b2] = 1; got = true; }
                if (!got) { all = false; break; }
            }
            if (all) push_cand(i, j);
        }
    }

    // confirm: same-solid guard -> planar shortcut / geometric AreFacesSameDomain
    for (auto& [i, j] : cands) {
        const SDFaceRec& fi = m_faces[i];
        const SDFaceRec& fj = m_faces[j];
        // Zero-thickness guard FIRST -- it also gates the planar shortcut below, which is the
        // one path that can assert SD with no geometric test at all (Builder_2.cxx:776-785).
        if (fi.solid == fj.solid) continue;
        double ti = fi.tol > 0 ? fi.tol : m_key_tol;
        double tj = fj.tol > 0 ? fj.tol : m_key_tol;
        // Asymmetric band: F1 = the lower-indexed face, its edge tolerances raise BOTH terms.
        double class_band = tj;
        double band = sd_band(*fi.brep, fi.face, ti, tj, m_fuzz, &class_band);
        int orient_nat = 0;
        bool sd = false;
        int via = 1;
        if (fi.planar && fj.planar && fi.bounded && fj.bounded && fi.key == fj.key) {
            sd = true;
            via = 0;
            int sii = fi.brep->m_faces[fi.face].surface_index;
            int sij = fj.brep->m_faces[fj.face].surface_index;
            Vector ni = fi.brep->m_surfaces[sii].normal_at(0.5, 0.5);
            Vector nj = fj.brep->m_surfaces[sij].normal_at(0.5, 0.5);
            orient_nat = (ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2]) > 0 ? 0 : 1;
        } else {
            sd = faces_same_domain(*fi.brep, fi.face, *fj.brep, fj.face, band, class_band,
                                   &orient_nat);
        }
        if (!sd) continue;
        bool same_nat = orient_nat == 0;
        bool same_sign = osign_face[i] * osign_face[j] > 0;
        SDPairRec pr;
        pr.i = i; pr.j = j;
        pr.orient = (same_nat == same_sign) ? 0 : 1;
        pr.via = via;
        m_pairs.push_back(pr);
        int ri = find(i), rj = find(j);
        if (ri != rj) m_rep[std::max(ri, rj)] = std::min(ri, rj);
    }
    // path-compress to min-index representative
    for (int i = 0; i < n; ++i) m_rep[i] = find(i);
    m_detected = true;
}

int SameDomain::rep(int i) const { return m_detected ? m_rep[i] : i; }

bool SameDomain::same_domain(int i, int j) const {
    return m_detected && m_rep[i] == m_rep[j];
}

int SameDomain::group_count() const {
    if (!m_detected) return 0;
    std::map<int, int> cnt;
    for (int i = 0; i < (int)m_faces.size(); ++i) cnt[m_rep[i]]++;
    int g = 0;
    for (auto& kv : cnt)
        if (kv.second >= 2) ++g;
    return g;
}

std::vector<std::vector<int>> SameDomain::groups() const {
    std::map<int, std::vector<int>> by_rep;
    if (m_detected)
        for (int i = 0; i < (int)m_faces.size(); ++i) by_rep[m_rep[i]].push_back(i);
    std::vector<std::vector<int>> out;
    for (auto& kv : by_rep)
        if (kv.second.size() >= 2) out.push_back(kv.second);
    return out;
}

double SameDomain::max_edge_tolerance(const BRep& b, int face) {
    // OCCT scans the max tolerance of the face's NON-DEGENERATE edges. This kernel stores no
    // per-entity tolerances (BRepEdge has no tolerance field -- that is phase P5), so there is
    // nothing to scan and this returns 0. The call site keeps OCCT's asymmetric structure so
    // that wiring P5 in later is a one-line change here. See audit E3 / trap 5.
    (void)b; (void)face;
    return 0.0;
}

double SameDomain::sd_band(const BRep& A, int fa, double tolF1, double tolF2, double fuzz,
                           double* class_band) {
    // BOPTools_AlgoTools.cxx:1174-1199 (audit E3): aTolEMax is scanned over F1's EDGES ONLY and
    // then raises BOTH aTolF1 and aTolF2 -- F2's own edge tolerances never enter the band.
    double tolE = max_edge_tolerance(A, fa);
    double t1 = std::max(tolF1, tolE);
    double t2 = std::max(tolF2, tolE);
    if (class_band) *class_band = t2;   // classifier band is the FACE tolerance, not the sum
    return t1 + t2 + std::max(fuzz, SD_CONFUSION);
}

bool SameDomain::faces_same_domain(const BRep& A, int fa, const BRep& B, int fb,
                                   double band, double class_band, int* orient) {
    // ONE-DIRECTIONAL and FAIL-CLOSED (audit E2 / trap 1-2): the interior point comes from A.fa
    // ONLY, and "cannot decide" is reported as "not same domain" -- there is deliberately no
    // PointNearEdge fallback (that exists only in IsSplitToReverse).
    Point p;
    double ua = 0, va = 0;
    if (!point_in_face(A, fa, p, ua, va)) return false;
    int si = B.m_faces[fb].surface_index;
    if (si < 0 || si >= (int)B.m_surfaces.size()) return false;
    const NurbsSurface& SB = B.m_surfaces[si];
    // IsValidPointForFace (IntTools_Context.cxx:647-673) carries THREE tolerances, not one:
    //  (a) the projector's own tolerance, onto the face's UV patch (not the infinite surface);
    //  (b) a STRICT unsquared rejection at the summed band: `dist > band` fails, `== band` passes;
    //  (c) a classifier that accepts TopAbs_ON as inside, banded by the FACE tolerance of F2.
    // Collapsing (b) and (c) into one band (the old code used `band` for both) makes the ON
    // acceptance far looser than OCCT's, since band = tolF1 + tolF2 + fuzz >= class_band.
    auto [u, v, d] = Closest::surface_point(SB, p);
    if (d > band) return false;                                   // (b) strict reject
    bool valid = false;
    if (uv_in_trims(B, fb, u, v)) {
        valid = true;                                             // (c) classifier: IN
    } else {
        double ou = u, ov = v;
        double bd = closest_on_trim(B, fb, p, ou, ov);
        if (bd <= class_band) { valid = true; u = ou; v = ov; }   // (c) classifier: ON
    }
    if (!valid) return false;
    if (orient) {
        int sa = A.m_faces[fa].surface_index;
        Vector na = A.m_surfaces[sa].normal_at(ua, va);
        Vector nb = SB.normal_at(u, v);
        *orient = (na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2]) > 0 ? 0 : 1;
    }
    return true;
}

SDFaceKey SameDomain::face_key(const BRep& b, int face, double key_tol) {
    SDFaceKey key;
    double grid = std::max(key_tol, 1e-12);
    for (int ei : face_edge_uses(b, face)) {
        RawSig rs;
        if (!raw_sig(b, ei, grid, rs, nullptr, nullptr)) continue;
        key.sigs.push_back(quantize_sig(rs, grid));
    }
    std::sort(key.sigs.begin(), key.sigs.end());
    return key;
}

bool SameDomain::edge_sig(const BRep& b, int edge, double key_tol, SDEdgeSig& out) {
    double grid = std::max(key_tol, 1e-12);
    RawSig rs;
    if (!raw_sig(b, edge, grid, rs, nullptr, nullptr)) return false;
    out = quantize_sig(rs, grid);
    return true;
}

bool SameDomain::point_in_face(const BRep& b, int face, Point& p3, double& u, double& v) {
    if (face < 0 || face >= (int)b.m_faces.size()) return false;
    int si = b.m_faces[face].surface_index;
    if (si < 0 || si >= (int)b.m_surfaces.size()) return false;
    const NurbsSurface& S = b.m_surfaces[si];
    auto [u0, u1] = S.domain(0);
    auto [v0, v1] = S.domain(1);
    // 43.2 % of each span, not the midpoint -- see SD_PAR_T.
    double cu = u0 + (u1 - u0) * SD_PAR_T, cv = v0 + (v1 - v0) * SD_PAR_T;
    if (uv_in_trims(b, face, cu, cv)) {
        u = cu; v = cv;
        p3 = S.point_at(u, v);
        return true;
    }
    // grid scan; keep the most interior candidate (max min-distance to the domain walls
    // as a cheap proxy is wrong for trims -- use first-found on a coarse-to-fine ladder)
    for (int nn : {8, 16, 32}) {
        for (int iu = 1; iu < nn; ++iu)
            for (int iv = 1; iv < nn; ++iv) {
                double su = u0 + (u1 - u0) * iu / nn, sv = v0 + (v1 - v0) * iv / nn;
                if (uv_in_trims(b, face, su, sv)) {
                    u = su; v = sv;
                    p3 = S.point_at(u, v);
                    return true;
                }
            }
    }
    return false;
}

std::vector<std::array<double, 2>> SameDomain::samples_in_face(const BRep& b, int face,
                                                               int want) {
    std::vector<std::array<double, 2>> out;
    if (face < 0 || face >= (int)b.m_faces.size()) return out;
    int si = b.m_faces[face].surface_index;
    if (si < 0 || si >= (int)b.m_surfaces.size()) return out;
    const NurbsSurface& S = b.m_surfaces[si];
    auto [u0, u1] = S.domain(0);
    auto [v0, v1] = S.domain(1);
    Point p;
    double pu = 0, pv = 0;
    if (!point_in_face(b, face, p, pu, pv)) return out;
    out.push_back({pu, pv});
    if (want <= 1) return out;
    // Spread grid scan; irrational offsets keep samples off symmetric loci (seams, tangency
    // lines, face centroids) that would otherwise be sampled systematically.
    for (int n = 4; n <= 12 && (int)out.size() < want; n += 4) {
        for (int iu = 1; iu < n && (int)out.size() < want; ++iu)
            for (int iv = 1; iv < n && (int)out.size() < want; ++iv) {
                double fu = (iu + 0.13212) / n, fv = (iv + 0.31121) / n;
                double su = u0 + (u1 - u0) * fu, sv = v0 + (v1 - v0) * fv;
                if (!uv_in_trims(b, face, su, sv)) continue;
                bool dup = false;
                double du = (u1 - u0) * 1e-3, dv = (v1 - v0) * 1e-3;
                for (const auto& e : out)
                    if (std::abs(e[0] - su) < du && std::abs(e[1] - sv) < dv) { dup = true; break; }
                if (!dup) out.push_back({su, sv});
            }
    }
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// BuildBOP orientation op-table
///////////////////////////////////////////////////////////////////////////////////////////

SDVerdict sd_select_sd_face(SDOp op, int operand, bool orient_same) {
    return sd_select_face(op, operand, orient_same ? SDState::OnSame : SDState::OnOpposite);
}

SDVerdict sd_select_face(SDOp op, int operand, SDState state) {
    bool objIN = (op == SDOp::Common || op == SDOp::Cut21);
    bool toolIN = (op == SDOp::Common || op == SDOp::Cut);
    bool takeIN = (operand == 0) ? objIN : toolIN;
    bool sameOriNeeded = (objIN == toolIN);
    int keeper = (op == SDOp::Cut21) ? 1 : 0;   // the OUT-state side (object for Common)
    switch (state) {
        case SDState::In:
            if (!takeIN) return SDVerdict::Drop;
            return sameOriNeeded ? SDVerdict::Keep : SDVerdict::KeepReversed;
        case SDState::Out:
            return takeIN ? SDVerdict::Drop : SDVerdict::Keep;
        case SDState::OnSame:
            return (sameOriNeeded && operand == keeper) ? SDVerdict::Keep : SDVerdict::Drop;
        case SDState::OnOpposite:
            return (!sameOriNeeded && operand == keeper) ? SDVerdict::Keep : SDVerdict::Drop;
    }
    return SDVerdict::Drop;
}

///////////////////////////////////////////////////////////////////////////////////////////
// ON-classification
///////////////////////////////////////////////////////////////////////////////////////////

double sd_distance_to_boundary(const BRep& b, const Point& p, int* face) {
    static const std::vector<double> no_sign;
    NearestHit hit = nearest_on_solid(b, p, no_sign);
    if (face) *face = hit.fb;
    return hit.fb < 0 ? 1e300 : hit.d;
}

SDState sd_classify_face(const BRep& S, int face, const BRep& other, double tol,
                         const std::vector<double>& osign_S,
                         const std::vector<double>& osign_other,
                         int probes) {
    return sd_classify_samples(S, face, SameDomain::samples_in_face(S, face, std::max(probes, 1)),
                               other, tol, osign_S, osign_other);
}

SDState sd_classify_samples(const BRep& S, int face,
                            const std::vector<std::array<double, 2>>& uvs,
                            const BRep& other, double tol,
                            const std::vector<double>& osign_S,
                            const std::vector<double>& osign_other) {
    std::vector<double> sgS_local, sgO_local;
    const std::vector<double>* sgS = &osign_S;
    const std::vector<double>* sgO = &osign_other;
    if (osign_S.empty()) { sgS_local = S.face_outward_signs(); sgS = &sgS_local; }
    if (osign_other.empty()) { sgO_local = other.face_outward_signs(); sgO = &sgO_local; }

    if (uvs.empty()) return SDState::Out;
    if (face < 0 || face >= (int)S.m_faces.size()) return SDState::Out;
    int si = S.m_faces[face].surface_index;
    if (si < 0 || si >= (int)S.m_surfaces.size()) return SDState::Out;
    const NurbsSurface& SS = S.m_surfaces[si];
    double s = (face < (int)sgS->size()) ? (*sgS)[face] : 1.0;

    // AREAL verdict: ON only when EVERY probe is coincident. One probe landing on a tangency
    // line or on a smaller flush cap must not promote the whole face to ON.
    int n_on = 0, n_same = 0, n_opp = 0, n_in = 0, n_valid = 0;
    bool trace = std::getenv("SESSION_SD_TRACE") != nullptr;
    // Parity fallback for silhouette/rim hits (below). Built lazily: most faces never need it.
    Mesh omesh;
    bool omesh_built = false;
    auto other_contains = [&](const Point& p) {
        if (!omesh_built) { omesh = other.mesh(); omesh_built = true; }
        return other.contains_point(omesh, p);
    };
    for (const auto& uv : uvs) {
        Point p = SS.point_at(uv[0], uv[1]);
        NearestHit hit = nearest_on_solid(other, p, *sgO);
        if (hit.fb < 0) continue;
        if (trace)
            std::printf("[SD-TRACE] f%d uv(%.4f,%.4f) p(%.4f,%.4f,%.4f) -> fb%d d=%.3e dp=%+.3e\n",
                        face, uv[0], uv[1], p[0], p[1], p[2], hit.fb, hit.d, hit.dp);
        ++n_valid;
        if (hit.d <= tol) {
            ++n_on;
            Vector n = SS.normal_at(uv[0], uv[1]);
            double dot = (n[0] * s) * hit.nout[0] + (n[1] * s) * hit.nout[1] +
                         (n[2] * s) * hit.nout[2];
            // Only DECISIVE votes count. dot ~ 0 means the nearest point is on an edge/corner of
            // the other solid (its face normal is perpendicular to ours), where the orientation
            // relation is undefined -- counting those as "same" flips a region's verdict.
            if (dot > 1e-9) ++n_same;
            else if (dot < -1e-9) ++n_opp;
            continue;
        }
        // Signed side of the nearest boundary. Valid ONLY when the nearest point lies in a face
        // INTERIOR: (a) a nearest point on a trim boundary means the probe is nearest to an EDGE,
        // where the face normal is not the separating direction (and our sampled rim point makes
        // (p-q).n = cos(dtheta)-1 < 0 systematically -- a false "inside" bias of order the
        // sampling sagitta); (b) |dp|/d is the cosine between (p-q) and the normal, so a
        // silhouette hit (p radially aligned with a coaxial cylinder) makes dp pure round-off.
        // Both cases settle by ray parity against the tessellated boundary, which needs no local
        // frame at the hit.
        bool reliable = !hit.on_boundary && std::abs(hit.dp) > 1e-6 * hit.d;
        bool inside = reliable ? (hit.dp < 0) : other_contains(p);
        if (inside) ++n_in;
    }
    if (n_valid == 0) return SDState::Out;
    if (n_on == n_valid) return (n_same >= n_opp) ? SDState::OnSame : SDState::OnOpposite;
    // Partial coincidence / tangency: report the majority In/Out of the non-ON probes.
    // Exact per-region states need the face split first (kb/p3_integration_notes.md).
    int n_off = n_valid - n_on;
    return (n_in * 2 > n_off) ? SDState::In : SDState::Out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Coincidence-only boolean assembly (test/reference driver)
///////////////////////////////////////////////////////////////////////////////////////////

BRep sd_boolean_coincident(const BRep& A, const BRep& B, SDOp op, double tol) {
    std::vector<double> osA = A.face_outward_signs();
    std::vector<double> osB = B.face_outward_signs();
    std::vector<int> keptA, keptB, revA, revB;
    for (int fi = 0; fi < (int)A.m_faces.size(); ++fi) {
        SDState st = sd_classify_face(A, fi, B, tol, osA, osB);
        SDVerdict vd = sd_select_face(op, 0, st);
        if (vd == SDVerdict::Drop) continue;
        keptA.push_back(fi);
        if (vd == SDVerdict::KeepReversed) revA.push_back((int)keptA.size() - 1);
    }
    for (int fi = 0; fi < (int)B.m_faces.size(); ++fi) {
        SDState st = sd_classify_face(B, fi, A, tol, osB, osA);
        SDVerdict vd = sd_select_face(op, 1, st);
        if (vd == SDVerdict::Drop) continue;
        keptB.push_back(fi);
        if (vd == SDVerdict::KeepReversed) revB.push_back((int)keptB.size() - 1);
    }
    if (keptA.empty() && keptB.empty()) return BRep();
    BRep R;
    bool have = false;
    if (!keptA.empty()) {
        R = A.subset(keptA);
        for (int k : revA) R.m_faces[k].reversed = !R.m_faces[k].reversed;
        have = true;
    }
    if (!keptB.empty()) {
        BRep Rb = B.subset(keptB);
        for (int k : revB) Rb.m_faces[k].reversed = !Rb.m_faces[k].reversed;
        if (have) R.append_brep(Rb);
        else R = Rb;
    }
    R.sew_coincident_edges(tol);
    return R;
}

} // namespace session_cpp
