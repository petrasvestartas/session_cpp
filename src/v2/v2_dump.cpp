#include "v2_dump.h"

#include "brep_massprops.h"
#include "brep_v2_boolean.h"
#include "nurbssurface.h"
#include "plane.h"
#include "v2_verdict.h"
#include "xform.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace session_cpp {
namespace v2dump {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Formatting
///////////////////////////////////////////////////////////////////////////////////////////

std::string v2dump_r(double v) {
    if (!(v == v)) return "nan";
    if (std::fabs(v) < 1e-12) v = 0.0;
    char b[64];
    std::snprintf(b, sizeof(b), "%.9g", v);
    return std::string(b);
}

std::string v2dump_p3(const Point& p) {
    return v2dump_r(p[0]) + "," + v2dump_r(p[1]) + "," + v2dump_r(p[2]);
}

std::string v2dump_ints(const std::vector<int>& v) {
    if (v.empty()) return "-";
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += ",";
        s += std::to_string(v[i]);
    }
    return s;
}

namespace {

struct Box3 {
    double lo[3] = {1e300, 1e300, 1e300};
    double hi[3] = {-1e300, -1e300, -1e300};
    void add(const Point& p) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
        }
    }
    bool empty() const { return lo[0] > hi[0]; }
    double diagonal() const {
        if (empty()) return 0.0;
        const double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    std::string str() const {
        if (empty()) return "void";
        return v2dump_r(lo[0]) + "," + v2dump_r(lo[1]) + "," + v2dump_r(lo[2]) + ";" +
               v2dump_r(hi[0]) + "," + v2dump_r(hi[1]) + "," + v2dump_r(hi[2]);
    }
};

std::string bds_box_str(const BdsBox& b) {
    if (b.is_void()) return "void";
    return v2dump_r(b.lo[0]) + "," + v2dump_r(b.lo[1]) + "," + v2dump_r(b.lo[2]) + ";" +
           v2dump_r(b.hi[0]) + "," + v2dump_r(b.hi[1]) + "," + v2dump_r(b.hi[2]);
}

double vlen(double x, double y, double z) { return std::sqrt(x * x + y * y + z * z); }

const char* type_name(BdsType t) {
    switch (t) {
        case BdsType::Vertex: return "VERTEX";
        case BdsType::Edge: return "EDGE";
        case BdsType::Face: return "FACE";
        default: return "SOLID";
    }
}

/// Sorted, printed, then flushed — the discipline that makes two runs diff cleanly.
void flush_sorted(std::ostream& os, std::vector<std::string>& lines) {
    std::sort(lines.begin(), lines.end());
    for (const auto& l : lines) os << l << "\n";
    lines.clear();
}

}  // namespace

std::string v2dump_curve_kind(const NurbsCurve& c) {
    const std::pair<double, double> d = c.domain();
    const int N = 32;
    std::vector<Point> s;
    s.reserve(N + 1);
    for (int k = 0; k <= N; ++k)
        s.push_back(c.point_at(d.first + (d.second - d.first) * (k / double(N))));

    Box3 bx;
    for (const Point& p : s) bx.add(p);
    const double scale = std::max(1e-12, bx.diagonal());
    if (scale <= 1e-9) return "Degenerated";

    // Line: every sample within 1e-9 * scale of the chord through the two ends.
    {
        const Point& a = s.front();
        const Point& b = s.back();
        const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
        const double L = vlen(ux, uy, uz);
        if (L > 1e-12 * scale) {
            double worst = 0.0;
            for (const Point& p : s) {
                const double wx = p[0] - a[0], wy = p[1] - a[1], wz = p[2] - a[2];
                const double cx = wy * uz - wz * uy, cy = wz * ux - wx * uz, cz = wx * uy - wy * ux;
                worst = std::max(worst, vlen(cx, cy, cz) / L);
            }
            if (worst <= 1e-9 * scale) return "Line";
        }
    }

    // Circle: circumcentre of three well-separated samples, then every sample must be
    // equidistant from it and coplanar with them.
    {
        const Point& A = s[0];
        const Point& B = s[N / 3];
        const Point& C = s[(2 * N) / 3];
        const double ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
        const double vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
        const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double n2 = nx * nx + ny * ny + nz * nz;
        if (n2 > 1e-24) {
            const double u2 = ux * ux + uy * uy + uz * uz;
            const double v2 = vx * vx + vy * vy + vz * vz;
            // (|u|^2 (v x n) + |v|^2 (n x u)) / (2 |n|^2)
            const double a1x = vy * nz - vz * ny, a1y = vz * nx - vx * nz, a1z = vx * ny - vy * nx;
            const double a2x = ny * uz - nz * uy, a2y = nz * ux - nx * uz, a2z = nx * uy - ny * ux;
            const double cx = A[0] + (u2 * a1x + v2 * a2x) / (2.0 * n2);
            const double cy = A[1] + (u2 * a1y + v2 * a2y) / (2.0 * n2);
            const double cz = A[2] + (u2 * a1z + v2 * a2z) / (2.0 * n2);
            const double R = vlen(A[0] - cx, A[1] - cy, A[2] - cz);
            if (R > 1e-12 * scale) {
                const double inv = 1.0 / std::sqrt(n2);
                double worst = 0.0;
                for (const Point& p : s) {
                    const double dx = p[0] - cx, dy = p[1] - cy, dz = p[2] - cz;
                    worst = std::max(worst, std::fabs(vlen(dx, dy, dz) - R));
                    worst = std::max(worst, std::fabs((dx * nx + dy * ny + dz * nz) * inv));
                }
                if (worst <= 1e-7 * scale) return "Circle";
            }
        }
    }
    return "BSpline";
}

std::string v2dump_surface_kind(const NurbsSurface& s) {
    return s.is_planar(nullptr, 1e-9) ? "Plane" : "Curved";
}

///////////////////////////////////////////////////////////////////////////////////////////
// 1. Specs
///////////////////////////////////////////////////////////////////////////////////////////

double V2DumpSpec::get(const char* k, double d) const {
    auto it = num.find(k);
    return it == num.end() ? d : it->second;
}

V2DumpSpec v2dump_parse_spec(const std::string& s) {
    V2DumpSpec sp;
    sp.raw = s;
    std::vector<std::string> parts;
    std::string cur;
    for (char c : s) {
        if (c == ',') {
            parts.push_back(cur);
            cur.clear();
        } else
            cur.push_back(c);
    }
    parts.push_back(cur);
    sp.type = parts.empty() ? "" : parts[0];
    for (size_t i = 1; i < parts.size(); ++i) {
        const std::string& p = parts[i];
        const size_t eq = p.find('=');
        if (eq == std::string::npos) {
            if (p == "center") sp.center = true;
            continue;
        }
        sp.num[p.substr(0, eq)] = std::atof(p.substr(eq + 1).c_str());
    }
    return sp;
}

BRep v2dump_build_shape(const V2DumpSpec& sp, std::string* why) {
    BRep b;
    const double h = sp.get("h", 1.0);
    double zshift = 0.0;
    if (sp.type == "sphere") {
        b = BRep::create_sphere(sp.get("r", 1.0));
    } else if (sp.type == "cylinder") {
        b = BRep::create_cylinder(sp.get("r", 1.0), h);
        // BRep::create_cylinder puts the base at z=0; OCCT's BRepPrimAPI does the same, so
        // `center` means the same -h/2 shift in both tracers.
        if (sp.center) zshift = -h / 2.0;
    } else if (sp.type == "cone") {
        if (std::fabs(sp.get("r2", 0.0)) > 1e-12) {
            if (why) *why = "BRep::create_cone builds an APEX cone only (r2 must be 0)";
            return BRep();
        }
        b = BRep::create_cone(sp.get("r1", 1.0), h);
        if (sp.center) zshift = -h / 2.0;
    } else if (sp.type == "torus") {
        b = BRep::create_torus(sp.get("r1", 2.0), sp.get("r2", 0.5));
    } else if (sp.type == "box") {
        // create_box takes FULL extents and is ALWAYS centred, so an un-centred OCCT box needs
        // the +d/2 shift rather than the other way round.
        const double dx = sp.get("dx", 1.0), dy = sp.get("dy", 1.0), dz = sp.get("dz", 1.0);
        b = BRep::create_box(dx, dy, dz);
        if (!sp.center) {
            Xform t = Xform::translation(dx / 2.0, dy / 2.0, dz / 2.0);
            b.transform(t);
        }
    } else {
        if (why) *why = "unknown shape type '" + sp.type + "'";
        return BRep();
    }

    Xform T = Xform::translation(0, 0, zshift);
    if (sp.has("rotx")) T = Xform::rotation_x(sp.get("rotx", 0), true) * T;
    if (sp.has("roty")) T = Xform::rotation_y(sp.get("roty", 0), true) * T;
    if (sp.has("rotz")) T = Xform::rotation_z(sp.get("rotz", 0), true) * T;
    if (sp.has("tx") || sp.has("ty") || sp.has("tz"))
        T = Xform::translation(sp.get("tx", 0), sp.get("ty", 0), sp.get("tz", 0)) * T;
    b.transform(T);
    return b;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Operand dump
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

/// An edge is DEGENERATE when its 3D curve has (essentially) zero length, or when every trim
/// that uses it is Singular. Both rules are the ones v2_verdict/is_solid use; nothing here
/// invents a third.
bool edge_is_degenerate(const BRep& b, int e) {
    const BRepEdge& ed = b.m_topology_edges[(size_t)e];
    if (ed.curve_3d_index >= 0 && ed.curve_3d_index < (int)b.m_curves_3d.size()) {
        const NurbsCurve& c = b.m_curves_3d[(size_t)ed.curve_3d_index];
        if (c.length(1e-9) < 1e-12) return true;
    }
    if (ed.trim_indices.empty()) return false;
    for (int t : ed.trim_indices)
        if (b.m_trims[(size_t)t].type != BRepTrimType::Singular) return false;
    return true;
}

}  // namespace

void v2dump_operand(std::ostream& os, int argi, const V2DumpSpec& sp, const BRep& b) {
    MassPropsOptions mo = v2v::v2_verdict_options();
    const MassProps mp = brep_massprops(b, mo);
    const v2v::V2Verdict vd = v2v::v2_verdict(b, mo);

    os << "ARG i=" << argi << " spec=" << sp.raw << " type=SOLID nsolid=" << (vd.closed() ? 1 : 0)
       << " nshell=" << vd.shells << " nface=" << (int)b.m_faces.size()
       << " nedge=" << (int)b.m_topology_edges.size()
       << " nvert=" << (int)b.m_topology_vertices.size() << " vol=" << v2dump_r(mp.volume)
       << " area=" << v2dump_r(mp.area) << " valid=" << (vd.closed() ? 1 : 0) << "\n";

    std::map<int, double> face_area;
    for (const auto& f : mp.faces) face_area[f.face_index] = f.area;

    for (int i = 0; i < (int)b.m_faces.size(); ++i) {
        const BRepFace& f = b.m_faces[(size_t)i];
        const NurbsSurface* s = (f.surface_index >= 0 && f.surface_index < (int)b.m_surfaces.size())
                                    ? &b.m_surfaces[(size_t)f.surface_index]
                                    : nullptr;
        std::string kind = "none";
        std::string u0 = "-", u1 = "-", v0 = "-", v1 = "-";
        int uclo = 0, vclo = 0;
        if (s) {
            kind = v2dump_surface_kind(*s);
            const std::pair<double, double> du = s->domain(0), dv = s->domain(1);
            u0 = v2dump_r(du.first);
            u1 = v2dump_r(du.second);
            v0 = v2dump_r(dv.first);
            v1 = v2dump_r(dv.second);
            uclo = s->is_closed(0) ? 1 : 0;
            vclo = s->is_closed(1) ? 1 : 0;
        }
        os << "AFACE a=" << argi << " i=" << (i + 1) << " surf=" << kind << " u0=" << u0
           << " u1=" << u1 << " v0=" << v0 << " v1=" << v1 << " uper=" << uclo << " vper=" << vclo
           << " uclo=" << uclo << " vclo=" << vclo << " ori=" << (f.reversed ? "REV" : "FWD")
           << " tol=" << v2dump_r(BDS_CONFUSION) << " area=" << v2dump_r(face_area.count(i) ? face_area[i] : 0.0)
           << "\n";
        std::vector<std::string> lines;
        for (int li : f.loop_indices) {
            if (li < 0 || li >= (int)b.m_loops.size()) continue;
            for (int ti : b.m_loops[(size_t)li].trim_indices) {
                if (ti < 0 || ti >= (int)b.m_trims.size()) continue;
                const BRepTrim& tr = b.m_trims[(size_t)ti];
                std::ostringstream ss;
                ss << "AFEDGE a=" << argi << " f=" << (i + 1) << " e=" << (tr.edge_index + 1)
                   << " seam=" << (tr.type == BRepTrimType::Seam ? 1 : 0)
                   << " degen=" << (tr.type == BRepTrimType::Singular ? 1 : 0)
                   << " ori=" << (tr.reversed ? "REV" : "FWD");
                lines.push_back(ss.str());
            }
        }
        flush_sorted(os, lines);
    }

    for (int i = 0; i < (int)b.m_topology_edges.size(); ++i) {
        const BRepEdge& e = b.m_topology_edges[(size_t)i];
        std::string ct = "Degenerated", t0 = "-", t1 = "-", len = "-";
        const bool degen = edge_is_degenerate(b, i);
        if (e.curve_3d_index >= 0 && e.curve_3d_index < (int)b.m_curves_3d.size()) {
            const NurbsCurve& c = b.m_curves_3d[(size_t)e.curve_3d_index];
            const std::pair<double, double> d = c.domain();
            t0 = v2dump_r(d.first);
            t1 = v2dump_r(d.second);
            if (!degen) {
                ct = v2dump_curve_kind(c);
                len = v2dump_r(c.length(1e-9));
            }
        }
        os << "AEDGE a=" << argi << " i=" << (i + 1) << " curve=" << ct << " t0=" << t0
           << " t1=" << t1 << " len=" << len << " tol=" << v2dump_r(BDS_CONFUSION)
           << " degen=" << (degen ? 1 : 0)
           << " closed=" << (e.start_vertex == e.end_vertex ? 1 : 0) << " v1=" << (e.start_vertex + 1)
           << " v2=" << (e.end_vertex + 1) << "\n";
    }

    for (int i = 0; i < (int)b.m_topology_vertices.size(); ++i) {
        const int pi = b.m_topology_vertices[(size_t)i].point_index;
        const Point p = (pi >= 0 && pi < (int)b.m_vertices.size()) ? b.m_vertices[(size_t)pi]
                                                                   : Point(0, 0, 0);
        os << "AVERT a=" << argi << " i=" << (i + 1) << " p=" << v2dump_p3(p)
           << " tol=" << v2dump_r(BDS_CONFUSION) << "\n";
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Arena dump
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

struct PBKey {
    int orig = -1, e = -1;
    double t0 = 0, t1 = 0;
    BdsPB pb;
};

PBKey make_key(const BdsPB& pb) {
    PBKey k;
    k.pb = pb;
    k.orig = pb->original_edge;
    k.e = pb->edge;
    pb->range(k.t0, k.t1);
    return k;
}

bool pb_less(const PBKey& a, const PBKey& b) {
    if (a.orig != b.orig) return a.orig < b.orig;
    if (a.t0 != b.t0) return a.t0 < b.t0;
    if (a.t1 != b.t1) return a.t1 < b.t1;
    return a.e < b.e;
}

std::string pb_ref(const BdsPB& pb) {
    const PBKey k = make_key(pb);
    return std::to_string(k.orig) + ":" + v2dump_r(k.t0) + ":" + v2dump_r(k.t1);
}

/// Stable id per common block, in arena pave-block-pool order (occt_trace's CollectCB).
std::map<const BdsCommonBlock*, int> collect_cb(const BdsArena& ds) {
    std::map<const BdsCommonBlock*, int> ids;
    int next = 0;
    for (int i = 0; i < ds.nb_shapes(); ++i) {
        if (!ds.is_edge(i) || !ds.has_pave_blocks(i)) continue;
        for (const BdsPB& pb : ds.pave_blocks(i)) {
            if (!ds.is_common_block(pb)) continue;
            const BdsCommonBlock* cb = ds.common_block(pb).get();
            if (!ids.count(cb)) ids[cb] = next++;
        }
    }
    return ids;
}

}  // namespace

void v2dump_arena(std::ostream& os, const BdsArena& ds, const char* tag) {
    os << "DS tag=" << tag << " nbshapes=" << ds.nb_shapes()
       << " nbsource=" << ds.nb_source_shapes() << " nbranges=" << ds.nb_ranges() << "\n";
    for (int i = 0; i < ds.nb_ranges(); ++i)
        os << "RANGE tag=" << tag << " i=" << i << " first=" << ds.range(i).first
           << " last=" << ds.range(i).last << "\n";

    for (int i = 0; i < ds.nb_shapes(); ++i) {
        const BdsShape& si = ds.shape(i);
        std::vector<int> sub = si.subs;
        std::sort(sub.begin(), sub.end());
        os << "SI tag=" << tag << " i=" << i << " type=" << type_name(si.type)
           << " rank=" << ds.rank(i) << " new=" << (ds.is_new_shape(i) ? 1 : 0)
           << " ref=" << si.reference << " flag=" << si.flag << " brep=" << (si.has_brep() ? 1 : 0)
           << " interf=" << (ds.has_interf(i) ? 1 : 0) << " nsub=" << sub.size()
           << " sub=" << v2dump_ints(sub) << " box=" << bds_box_str(si.box) << "\n";
    }

    for (int i = 0; i < ds.nb_shapes(); ++i) {
        if (!ds.is_vertex(i)) continue;
        int sd = -1, isd = 0;
        if (ds.has_shape_sd(i, isd)) sd = isd;
        os << "DSVERT tag=" << tag << " i=" << i << " p=" << v2dump_p3(ds.vertex_point(i))
           << " tol=" << v2dump_r(ds.tolerance(i)) << " new=" << (ds.is_new_shape(i) ? 1 : 0)
           << " sd=" << sd << "\n";
    }
    {
        std::vector<std::string> lines;
        for (int i = 0; i < ds.nb_shapes(); ++i) {
            int isd = 0;
            if (ds.has_shape_sd(i, isd))
                lines.push_back("SD tag=" + std::string(tag) + " i=" + std::to_string(i) +
                                " sd=" + std::to_string(isd));
        }
        flush_sorted(os, lines);
    }

    const std::map<const BdsCommonBlock*, int> cbids = collect_cb(ds);

    for (int i = 0; i < ds.nb_shapes(); ++i) {
        if (!ds.is_edge(i) || !ds.has_pave_blocks(i)) continue;
        std::vector<BdsPave> lp;
        ds.paves(i, lp);
        for (size_t k = 0; k < lp.size(); ++k)
            os << "PAVE tag=" << tag << " e=" << i << " k=" << k << " t=" << v2dump_r(lp[k].t)
               << " v=" << lp[k].vertex << "\n";

        std::vector<PBKey> keys;
        for (const BdsPB& pb : ds.pave_blocks(i)) keys.push_back(make_key(pb));
        std::sort(keys.begin(), keys.end(), pb_less);
        for (size_t j = 0; j < keys.size(); ++j) {
            const BdsPB& pb = keys[j].pb;
            int v1 = -1, v2 = -1;
            pb->indices(v1, v2);
            int cbid = -1;
            if (ds.is_common_block(pb)) {
                auto it = cbids.find(ds.common_block(pb).get());
                if (it != cbids.end()) cbid = it->second;
            }
            const std::string sr =
                pb->has_shrunk ? (v2dump_r(pb->ts1) + ":" + v2dump_r(pb->ts2)) : std::string("-");
            const std::string etol =
                (pb->edge >= 0 && ds.valid(pb->edge)) ? v2dump_r(ds.tolerance(pb->edge)) : "-";
            os << "PB tag=" << tag << " e=" << i << " k=" << j << " orig=" << pb->original_edge
               << " t0=" << v2dump_r(keys[j].t0) << " t1=" << v2dump_r(keys[j].t1) << " v1=" << v1
               << " v2=" << v2 << " edge=" << keys[j].e << " etol=" << etol << " cb=" << cbid
               << " split=" << (pb->is_split_edge() ? 1 : 0)
               << " splittable=" << (pb->splittable ? 1 : 0) << " shrunk=" << sr << "\n";
        }
    }

    {
        std::vector<std::pair<int, const BdsCommonBlock*>> ord;
        for (const auto& p : cbids) ord.emplace_back(p.second, p.first);
        std::sort(ord.begin(), ord.end());
        for (const auto& p : ord) {
            const BdsCommonBlock* cb = p.second;
            std::vector<PBKey> keys;
            for (const BdsPB& pb : cb->members) keys.push_back(make_key(pb));
            std::sort(keys.begin(), keys.end(), pb_less);
            std::string pbs;
            for (size_t j = 0; j < keys.size(); ++j) {
                if (j) pbs += "|";
                pbs += pb_ref(keys[j].pb);
            }
            std::vector<int> fs = cb->faces;
            std::sort(fs.begin(), fs.end());
            os << "CB tag=" << tag << " id=" << p.first << " tol=" << v2dump_r(cb->tol)
               << " edge=" << cb->edge() << " npb=" << keys.size()
               << " pbs=" << (pbs.empty() ? "-" : pbs) << " nfaces=" << fs.size()
               << " faces=" << v2dump_ints(fs) << "\n";
        }
    }

    for (int i = 0; i < ds.nb_shapes(); ++i) {
        if (!ds.is_face(i) || !ds.has_face_info(i)) continue;
        const BdsFaceInfo& fi = ds.face_info(i);
        struct SetRef {
            const char* name;
            const BdsPBSet* pbs;
            const std::vector<int>* vs;
        };
        const SetRef sets[3] = {{"In", &fi.pb_in, &fi.v_in},
                                {"On", &fi.pb_on, &fi.v_on},
                                {"Sc", &fi.pb_sc, &fi.v_sc}};
        std::vector<int> vin = fi.v_in, von = fi.v_on, vsc = fi.v_sc;
        std::sort(vin.begin(), vin.end());
        std::sort(von.begin(), von.end());
        std::sort(vsc.begin(), vsc.end());
        os << "FI tag=" << tag << " f=" << i << " nIn=" << fi.pb_in.size()
           << " nOn=" << fi.pb_on.size() << " nSc=" << fi.pb_sc.size()
           << " vIn=" << v2dump_ints(vin) << " vOn=" << v2dump_ints(von)
           << " vSc=" << v2dump_ints(vsc) << "\n";
        for (int s = 0; s < 3; ++s) {
            std::vector<PBKey> keys;
            for (int j = 0; j < sets[s].pbs->size(); ++j) keys.push_back(make_key((*sets[s].pbs)[j]));
            std::sort(keys.begin(), keys.end(), pb_less);
            for (size_t j = 0; j < keys.size(); ++j)
                os << "FIPB tag=" << tag << " f=" << i << " set=" << sets[s].name << " k=" << j
                   << " orig=" << keys[j].orig << " t0=" << v2dump_r(keys[j].t0)
                   << " t1=" << v2dump_r(keys[j].t1) << " edge=" << keys[j].e << "\n";
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Interference dump (probe arena)
///////////////////////////////////////////////////////////////////////////////////////////

void v2dump_interf(std::ostream& os, const BdsArena& ds, const v2int::V2Interf& in,
                   const char* tag) {
    std::vector<std::string> lines;

    for (const BdsInterf& x : ds.interfs()) {
        if (x.type == BdsInterfType::VV)
            lines.push_back("IVV tag=" + std::string(tag) + " src=probe i1=" +
                            std::to_string(x.a) + " i2=" + std::to_string(x.b) + " new=" +
                            std::to_string(x.new_vertex));
    }
    flush_sorted(os, lines);

    for (const BdsInterf& x : ds.interfs()) {
        if (x.type == BdsInterfType::VE)
            lines.push_back("IVE tag=" + std::string(tag) + " src=probe i1=" +
                            std::to_string(x.a) + " i2=" + std::to_string(x.b) + " t=" +
                            v2dump_r(x.ta) + " new=" + std::to_string(x.new_vertex));
    }
    flush_sorted(os, lines);

    for (const v2int::V2VFRecord& r : in.vf_records())
        lines.push_back("IVF tag=" + std::string(tag) + " src=probe i1=" +
                        std::to_string(r.vertex) + " i2=" + std::to_string(r.face) + " u=" +
                        v2dump_r(r.u) + " v=" + v2dump_r(r.v) + " new=-1 dist=" +
                        v2dump_r(r.dist));
    flush_sorted(os, lines);

    for (const v2int::V2EERecord& r : in.ee_records())
        lines.push_back("IEE tag=" + std::string(tag) + " src=probe i1=" +
                        std::to_string(r.edge_a) + " i2=" + std::to_string(r.edge_b) + " ctype=" +
                        (r.type == v2int::V2PartType::Edge ? "EDGE" : "VERTEX") + " r1=" +
                        v2dump_r(r.ta) + ":" + v2dump_r(r.ta) + " r2=" + v2dump_r(r.tb) + ":" +
                        v2dump_r(r.tb) + " new=" + std::to_string(r.new_vertex) + " tangential=" +
                        (r.tangential ? "1" : "0"));
    flush_sorted(os, lines);

    for (const v2int::V2EFRecord& r : in.ef_records()) {
        const char* kind = r.kind == v2int::V2EFKind::Transversal
                               ? "TRANSVERSAL"
                               : (r.kind == v2int::V2EFKind::Tangential ? "TANGENTIAL"
                                                                        : "COINCIDENT");
        lines.push_back("IEF tag=" + std::string(tag) + " src=probe i1=" +
                        std::to_string(r.edge) + " i2=" + std::to_string(r.face) + " ctype=" +
                        (r.type == v2int::V2PartType::Edge ? "EDGE" : "VERTEX") + " r1=" +
                        v2dump_r(r.t0) + ":" + v2dump_r(r.t1) + " new=" +
                        std::to_string(r.new_vertex) + " kind=" + kind);
    }
    flush_sorted(os, lines);
}

///////////////////////////////////////////////////////////////////////////////////////////
// 5. Section dump
///////////////////////////////////////////////////////////////////////////////////////////

void v2dump_section(std::ostream& os, const BdsArena& ds, const v2sec::V2Section& sec,
                    const char* tag) {
    (void)ds;
    // group curves by face pair, exactly as OCCT groups BOPDS_Curves under one BOPDS_InterfFF
    std::map<std::pair<int, int>, std::vector<int>> byPair;
    const std::vector<v2sec::V2Curve>& cs = sec.curves();
    for (int i = 0; i < (int)cs.size(); ++i)
        byPair[{cs[(size_t)i].face1, cs[(size_t)i].face2}].push_back(i);

    for (const auto& kv : byPair) {
        const int f1 = kv.first.first, f2 = kv.first.second;
        os << "IFF tag=" << tag << " i1=" << f1 << " i2=" << f2 << " tangent=0"
           << " ncurves=" << kv.second.size() << " npoints=0\n";
        int c = 0;
        for (int ci : kv.second) {
            const v2sec::V2Curve& cu = cs[(size_t)ci];
            std::string type = "Other", t0 = "-", t1 = "-", len = "-", p0 = "-", p1 = "-";
            Box3 bx;
            if (cu.c3d) {
                const std::pair<double, double> d = cu.c3d->domain();
                type = v2dump_curve_kind(*cu.c3d);
                t0 = v2dump_r(d.first);
                t1 = v2dump_r(d.second);
                len = v2dump_r(v2sec::v2sec_arclen(*cu.c3d, d.first, d.second));
                p0 = v2dump_p3(cu.c3d->point_at(d.first));
                p1 = v2dump_p3(cu.c3d->point_at(d.second));
                for (int k = 0; k <= 32; ++k)
                    bx.add(cu.c3d->point_at(d.first + (d.second - d.first) * (k / 32.0)));
            }
            int npb = 0;
            for (const v2sec::V2Block& b : cu.blocks)
                if (b.kept) ++npb;

            os << "SEC tag=" << tag << " f1=" << f1 << " f2=" << f2 << " c=" << c
               << " type=" << type << " geom=NurbsCurve t0=" << t0 << " t1=" << t1
               << " len=" << len << " tol=" << v2dump_r(cu.tol)
               << " tantol=" << v2dump_r(cu.tang_tol) << " p0=" << p0 << " p1=" << p1
               << " c2d1=1 c2d2=1 npb=" << npb << " box=" << bx.str()
               << " closed=" << (cu.closed ? 1 : 0) << " dev1=" << v2dump_r(cu.dev1)
               << " dev2=" << v2dump_r(cu.dev2) << " carrier=" << cu.carrier_edge << "\n";

            // 2D footprints on both faces. NOTE: our surfaces are NURBS with their own
            // parameterisation ([0,4] for a full circle direction, not [0,2pi]), so these
            // numbers are NOT numerically comparable to OCCT's; they are dumped because the
            // SHAPE of the footprint (does it wrap? does it leave the rectangle?) is.
            for (int w = 0; w < 2; ++w) {
                if (cu.trail.empty()) continue;
                double umin = 1e100, umax = -1e100, vmin = 1e100, vmax = -1e100;
                std::string samples;
                const int n = (int)cu.trail.size();
                for (int q = 0; q < n; ++q) {
                    const v2sec::V2PntOn2S& s = cu.trail[(size_t)q];
                    const double u = w == 0 ? s.u1 : s.u2;
                    const double v = w == 0 ? s.v1 : s.v2;
                    umin = std::min(umin, u);
                    umax = std::max(umax, u);
                    vmin = std::min(vmin, v);
                    vmax = std::max(vmax, v);
                    if (q == 0 || q == n / 2 || q == n - 1)
                        samples += (samples.empty() ? "" : "|") + v2dump_r(u) + ":" + v2dump_r(v);
                }
                os << "SEC2D tag=" << tag << " f1=" << f1 << " f2=" << f2 << " c=" << c
                   << " face=" << (w == 0 ? f1 : f2) << " umin=" << v2dump_r(umin)
                   << " umax=" << v2dump_r(umax) << " vmin=" << v2dump_r(vmin)
                   << " vmax=" << v2dump_r(vmax) << " s=" << samples << "\n";
            }

            int k = 0;
            for (const v2sec::V2Block& b : cu.blocks) {
                if (!b.kept) continue;
                const std::string etol =
                    (b.edge >= 0 && ds.valid(b.edge)) ? v2dump_r(ds.tolerance(b.edge)) : "-";
                os << "SECPB tag=" << tag << " f1=" << f1 << " f2=" << f2 << " c=" << c
                   << " k=" << k << " t0=" << v2dump_r(b.t0) << " t1=" << v2dump_r(b.t1)
                   << " v1=" << b.v0 << " v2=" << b.v1 << " edge=" << b.edge << " etol=" << etol
                   << " len=" << v2dump_r(b.length) << "\n";
                ++k;
            }
            ++c;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// 6. Result dump
///////////////////////////////////////////////////////////////////////////////////////////

V2DumpResultCounts v2dump_result(std::ostream& os, const BRep& res) {
    V2DumpResultCounts rc;
    if (res.face_count() == 0) {
        os << "RES null=1\n";
        return rc;
    }
    const MassPropsOptions mo = v2v::v2_verdict_options();
    const v2v::V2Verdict vd = v2v::v2_verdict(res, mo);
    const MassProps mp = brep_massprops(res, mo);

    rc.nsolid = vd.solids;
    rc.nshell = vd.shells;
    rc.nface = (int)res.m_faces.size();
    rc.nedge = (int)res.m_topology_edges.size();
    rc.nvert = (int)res.m_topology_vertices.size();
    rc.nnaked = vd.naked_real;
    rc.ndegen = vd.degenerate;
    rc.vol = vd.volume;
    rc.area = vd.area;
    rc.closure = vd.closure_residual;
    rc.valid = vd.closed() ? 1 : 0;

    os << "RES type=SOLID nsolid=" << rc.nsolid << " nshell=" << rc.nshell << " nface=" << rc.nface
       << " nedge=" << rc.nedge << " nvert=" << rc.nvert << " naked=" << rc.nnaked
       << " ndegen=" << rc.ndegen << " vol=" << v2dump_r(rc.vol) << " area=" << v2dump_r(rc.area)
       << " valid=" << rc.valid << " closure=" << v2dump_r(rc.closure)
       << " nonmanifold=" << vd.nonmanifold << " seam=" << vd.seam_edges << "\n";

    // per-shell volumes come from brep_massprops (shell_volumes), never from metric code here
    for (size_t i = 0; i < mp.shell_volumes.size(); ++i)
        os << "RESSOLID i=" << (i + 1) << " nface=-1 vol=" << v2dump_r(mp.shell_volumes[i])
           << " ori=FWD\n";

    // shell membership: FaceMassProps::shell, again from the shared module
    {
        std::map<int, std::vector<int>> byShell;
        for (const auto& f : mp.faces)
            if (f.shell >= 0) byShell[f.shell].push_back(f.face_index + 1);
        for (auto& kv : byShell) {
            std::sort(kv.second.begin(), kv.second.end());
            os << "RESSHELL i=" << (kv.first + 1) << " nface=" << kv.second.size()
               << " closed=" << (vd.naked_real == 0 ? 1 : 0)
               << " faces=" << v2dump_ints(kv.second) << "\n";
        }
    }

    std::map<int, double> face_area;
    for (const auto& f : mp.faces) face_area[f.face_index] = f.area;

    for (int i = 0; i < (int)res.m_faces.size(); ++i) {
        const BRepFace& f = res.m_faces[(size_t)i];
        const NurbsSurface* s = (f.surface_index >= 0 && f.surface_index < (int)res.m_surfaces.size())
                                    ? &res.m_surfaces[(size_t)f.surface_index]
                                    : nullptr;
        std::string kind = "none", u0 = "-", u1 = "-", v0 = "-", v1 = "-";
        if (s) {
            kind = v2dump_surface_kind(*s);
            const std::pair<double, double> du = s->domain(0), dv = s->domain(1);
            u0 = v2dump_r(du.first);
            u1 = v2dump_r(du.second);
            v0 = v2dump_r(dv.first);
            v1 = v2dump_r(dv.second);
        }
        int nedge = 0, nseam = 0;
        for (int li : f.loop_indices) {
            if (li < 0 || li >= (int)res.m_loops.size()) continue;
            for (int ti : res.m_loops[(size_t)li].trim_indices) {
                ++nedge;
                if (res.m_trims[(size_t)ti].type == BRepTrimType::Seam) ++nseam;
            }
        }
        os << "RESFACE i=" << (i + 1) << " surf=" << kind << " u0=" << u0 << " u1=" << u1
           << " v0=" << v0 << " v1=" << v1 << " ori=" << (f.reversed ? "REV" : "FWD")
           << " tol=" << v2dump_r(BDS_CONFUSION)
           << " area=" << v2dump_r(face_area.count(i) ? face_area[i] : 0.0)
           << " nwire=" << f.loop_indices.size() << " nedge=" << nedge << " nseam=" << nseam
           << "\n";
        int w = 0;
        for (int li : f.loop_indices) {
            if (li < 0 || li >= (int)res.m_loops.size()) continue;
            for (int ti : res.m_loops[(size_t)li].trim_indices) {
                const BRepTrim& tr = res.m_trims[(size_t)ti];
                os << "RESFEDGE f=" << (i + 1) << " w=" << w << " e=" << (tr.edge_index + 1)
                   << " ori=" << (tr.reversed ? "REV" : "FWD")
                   << " seam=" << (tr.type == BRepTrimType::Seam ? 1 : 0)
                   << " degen=" << (tr.type == BRepTrimType::Singular ? 1 : 0)
                   << " pc=" << (tr.curve_2d_index >= 0 ? 1 : 0) << " uv0=- uv1=-\n";
            }
            ++w;
        }
    }

    for (int i = 0; i < (int)res.m_topology_edges.size(); ++i) {
        const BRepEdge& e = res.m_topology_edges[(size_t)i];
        std::string ct = "Degenerated", t0 = "-", t1 = "-", len = "-";
        bool degen = true;
        if (e.curve_3d_index >= 0 && e.curve_3d_index < (int)res.m_curves_3d.size()) {
            const NurbsCurve& c = res.m_curves_3d[(size_t)e.curve_3d_index];
            const double L = c.length(1e-9);
            degen = L < 1e-12;
            const std::pair<double, double> d = c.domain();
            t0 = v2dump_r(d.first);
            t1 = v2dump_r(d.second);
            if (!degen) {
                ct = v2dump_curve_kind(c);
                len = v2dump_r(L);
            }
        }
        os << "RESEDGE i=" << (i + 1) << " curve=" << ct << " t0=" << t0 << " t1=" << t1
           << " len=" << len << " tol=" << v2dump_r(BDS_CONFUSION) << " degen=" << (degen ? 1 : 0)
           << " nface=" << e.trim_indices.size() << "\n";
    }

    for (int i = 0; i < (int)res.m_topology_vertices.size(); ++i) {
        const int pi = res.m_topology_vertices[(size_t)i].point_index;
        const Point p = (pi >= 0 && pi < (int)res.m_vertices.size()) ? res.m_vertices[(size_t)pi]
                                                                     : Point(0, 0, 0);
        os << "RESVERT i=" << (i + 1) << " p=" << v2dump_p3(p)
           << " tol=" << v2dump_r(BDS_CONFUSION) << "\n";
    }
    return rc;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 7. The whole case
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

void stage_summary(std::ostream& os, const BdsArena& ds, const char* stage, int nffc,
                   int ntan) {
    int npb = 0, nfi = 0, nsd = 0;
    for (int i = 0; i < ds.nb_shapes(); ++i) {
        if (ds.is_edge(i) && ds.has_pave_blocks(i)) npb += (int)ds.pave_blocks(i).size();
        if (ds.is_face(i) && ds.has_face_info(i)) ++nfi;
        int isd = 0;
        if (ds.has_shape_sd(i, isd)) ++nsd;
    }
    const int ncb = (int)collect_cb(ds).size();
    os << "STAGE name=" << stage << " nbshapes=" << ds.nb_shapes()
       << " nbsource=" << ds.nb_source_shapes() << " nsd=" << nsd << " npb=" << npb
       << " ncb=" << ncb << " nfaceinfo=" << nfi
       << " VV=" << ds.nb_interf(BdsInterfType::VV) << " VE=" << ds.nb_interf(BdsInterfType::VE)
       << " VF=" << ds.nb_interf(BdsInterfType::VF) << " EE=" << ds.nb_interf(BdsInterfType::EE)
       << " EF=" << ds.nb_interf(BdsInterfType::EF) << " FF=" << ds.nb_interf(BdsInterfType::FF)
       << " ffcurves=" << nffc << " ffpoints=0 fftangent=" << ntan << "\n";
}

/// VERBATIM REPLICA of the post-section pave attachment in v2sol_run_front
/// (src/v2/brep_v2_boolean.cpp, the loop `for (const auto& c : S.curves())` after the egap
/// measurement, through `F.ds.update_pave_blocks()`). Without it the arena dumped here has no
/// interior pave on any operand edge and the trace understates the production state — measured:
/// box x box showed 40 pave blocks on 40 edges, 0 split, where the production front splits 4.
/// Kept as a copy rather than a call because the original is a file-static inside a file this
/// module does not own; any change there must be mirrored here.
void attach_section_paves(BdsArena& ds, const v2sec::V2Section& S,
                          const std::vector<const BRep*>& ops, double tol) {
    // per-edge measured gap between the edge's own curve and its face's surface
    std::vector<double> egap((size_t)ds.nb_shapes(), 0.0);
    for (int q = 0; q < ds.nb_shapes(); ++q) {
        if (!ds.is_edge(q)) continue;
        const int r = ds.rank(q);
        if (r != 0 && r != 1) continue;
        const BRep& b = *ops[(size_t)r];
        const int le = ds.shape(q).source;
        const NurbsSurface* srf = nullptr;
        for (size_t ti = 0; ti < b.m_trims.size() && !srf; ++ti) {
            if (b.m_trims[ti].edge_index != le) continue;
            const int li = b.m_trims[ti].loop_index;
            if (li < 0 || li >= (int)b.m_loops.size()) continue;
            const int fi = b.m_loops[(size_t)li].face_index;
            if (fi < 0 || fi >= (int)b.m_faces.size()) continue;
            const int si = b.m_faces[(size_t)fi].surface_index;
            if (si >= 0 && si < (int)b.m_surfaces.size()) srf = &b.m_surfaces[(size_t)si];
        }
        const NurbsCurve* ec = ds.edge_curve(q);
        if (!srf || !ec) continue;
        const std::pair<double, double> er = ds.edge_range(q);
        double g = 0.0;
        for (int k = 0; k <= 16; ++k) {
            const Point p = ec->point_at(er.first + (er.second - er.first) * (k / 16.0));
            const std::pair<double, double> uv = srf->closest_parameters(p);
            const Point s = srf->point_at(uv.first, uv.second);
            g = std::max(g, vlen(s[0] - p[0], s[1] - p[1], s[2] - p[2]));
        }
        egap[(size_t)q] = g;
    }

    for (const v2sec::V2Curve& c : S.curves()) {
        const double lim = std::max(tol, 1e-7) * 100.0;
        double chord = 0.0;
        for (size_t q = 1; q < c.trail.size(); ++q) {
            const Point& a = c.trail[q].p;
            const Point& b = c.trail[q - 1].p;
            chord = std::max(chord, vlen(a[0] - b[0], a[1] - b[1], a[2] - b[2]));
        }
        const double sec_lim = std::max(c.tol, std::max(c.dev1, c.dev2)) * 8.0 + chord + lim;
        auto project = [&](int q, const Point& p, double& t_out) -> double {
            const NurbsCurve* qc = ds.edge_curve(q);
            if (!qc) return 1e300;
            const std::pair<double, double> qr = ds.edge_range(q);
            double bt = qr.first, bd = 1e300;
            for (int k = 0; k <= 256; ++k) {
                const double t = qr.first + (qr.second - qr.first) * (k / 256.0);
                const Point e = qc->point_at(t);
                const double d = vlen(e[0] - p[0], e[1] - p[1], e[2] - p[2]);
                if (d < bd) { bd = d; bt = t; }
            }
            for (int it = 0; it < 60; ++it) {
                const double h = (qr.second - qr.first) * std::pow(0.5, it + 1) * 0.5;
                const double tm = std::max(qr.first, bt - h), tp = std::min(qr.second, bt + h);
                const Point em = qc->point_at(tm), ep = qc->point_at(tp);
                const double dm = vlen(em[0] - p[0], em[1] - p[1], em[2] - p[2]);
                const double dp = vlen(ep[0] - p[0], ep[1] - p[1], ep[2] - p[2]);
                if (dm < bd) { bd = dm; bt = tm; }
                if (dp < bd) { bd = dp; bt = tp; }
            }
            t_out = bt;
            return bd;
        };
        for (const v2sec::V2Pave& pv : c.paves) {
            if (pv.vertex < 0) continue;
            const Point p = ds.vertex_point(pv.vertex);
            const double named_lim = std::max(lim, sec_lim);
            int named_edge = -1;
            if (pv.edge >= 0 && pv.face >= 0 && ds.is_face(pv.face)) {
                const int fop = ds.rank(pv.face);
                if (fop >= 0) named_edge = ds.index_of_edge(fop, pv.edge);
            }
            for (int q = 0; q < ds.nb_shapes(); ++q) {
                if (!ds.is_edge(q) || ds.rank(q) < 0) continue;
                double t = 0.0;
                const double d = project(q, p, t);
                const double gate = (q == named_edge ? named_lim : lim) + egap[(size_t)q];
                if (d > gate) continue;
                ds.add_pave(q, t, pv.vertex);
            }
        }
    }
    ds.update_pave_blocks();
}

Box3 brep_box(const BRep& b) {
    Box3 bx;
    for (const Point& p : b.m_vertices) bx.add(p);
    for (const auto& s : b.m_surfaces) {
        const std::pair<double, double> du = s.domain(0), dv = s.domain(1);
        for (int i = 0; i <= 4; ++i)
            for (int j = 0; j <= 4; ++j)
                bx.add(s.point_at(du.first + (du.second - du.first) * (i / 4.0),
                                  dv.first + (dv.second - dv.first) * (j / 4.0)));
    }
    return bx;
}

}  // namespace

int v2dump_case(std::ostream& os, const V2DumpSpec& spa, const V2DumpSpec& spb,
                const V2DumpOptions& opt) {
    // PHASE TIMING IS OFF BY DEFAULT AND MUST STAY OFF. A wall-clock number is not reproducible,
    // so emitting one unconditionally would destroy the byte-stability the whole comparison rests
    // on. Gate on (p && p[0]): getenv() returns a non-null EMPTY string for `FOO=`, and reading
    // that as "set" is one of the measurement errors this program exists to avoid.
    const char* tenv = std::getenv("SESSION_V2_DUMP_TIME");
    const bool timing = (tenv && tenv[0]);
    const auto t_now = [] { return std::chrono::steady_clock::now(); };
    const auto t_ms = [](std::chrono::steady_clock::time_point a,
                         std::chrono::steady_clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const auto t_start = t_now();
    auto t_mark = t_start;

    std::string whyA, whyB;
    const BRep A = v2dump_build_shape(spa, &whyA);
    const BRep B = v2dump_build_shape(spb, &whyB);
    if (A.face_count() == 0 || B.face_count() == 0) {
        os << "TRACE v=1 name=" << opt.name << " op=" << opt.op << " kernel=v2 error=1\n";
        os << "SPECERR a=" << whyA << " b=" << whyB << "\n";
        return 2;
    }

    os << "TRACE v=1 name=" << opt.name << " op=" << opt.op << " kernel=v2\n";
    os << "SPEC a=" << spa.raw << " b=" << spb.raw << "\n";
    // WHICH STAGES THIS KERNEL POPULATES. `interf` is a PROBE (see the header): the production
    // v2 boolean does not run VV/VE/EE/VF/EF. `imgface` and per-face pcurve uv are absent.
    os << "CAP stages=input,ds,dsverts,paves,pblocks,cblocks,sec,secpb,resface,"
          "resedge,resvert,resshell,ressolid,volume"
       << " probe=" << (opt.run_interf ? "interf" : "-")
       << " missing=faceinfo,imgface,imgedge,ffpoints,resfedge_uv,seam_flag_from_pcurve"
       << " ds_tags=final"
       << " policy=eager_pave_block_pools\n";

    v2dump_operand(os, 0, spa, A);
    v2dump_operand(os, 1, spb, B);

    // ---- tolerance: the SAME formula v2sol_run_front uses ---------------------------------
    Box3 both = brep_box(A);
    {
        const Box3 bb = brep_box(B);
        for (int k = 0; k < 3; ++k) {
            both.lo[k] = std::min(both.lo[k], bb.lo[k]);
            both.hi[k] = std::max(both.hi[k], bb.hi[k]);
        }
    }
    const double scale = std::max(1e-9, both.diagonal());
    const double tol = std::max(1e-9, scale * 1e-6);
    os << "TOL scale=" << v2dump_r(scale) << " tol=" << v2dump_r(tol)
       << " init=" << v2dump_r(std::max(tol, 1e-7)) << "\n";

    const std::vector<const BRep*> ops = {&A, &B};

    // ---- phase 2: interference probe (separate arena) --------------------------------------
    if (opt.run_interf) {
        BdsArena probe;
        probe.init(ops, std::max(tol, 1e-7));
        for (int i = 0; i < probe.nb_shapes(); ++i)
            if (probe.is_edge(i)) probe.change_pave_blocks(i);
        stage_summary(os, probe, "after_Init", 0, 0);
        v2int::V2InterfOptions io;
        io.fuzzy = std::max(tol, 1e-7);
        v2int::V2Interf in(probe, ops, io);
        in.perform_vv();
        stage_summary(os, probe, "after_VV", 0, 0);
        in.perform_ve();
        stage_summary(os, probe, "after_VE", 0, 0);
        in.perform_ee();
        stage_summary(os, probe, "after_EE", 0, 0);
        in.perform_vf();
        stage_summary(os, probe, "after_VF", 0, 0);
        in.perform_ef();
        stage_summary(os, probe, "after_EF", 0, 0);
        v2dump_interf(os, probe, in, "final");
        const v2int::V2InterfStats& st = in.stats();
        if (timing) {
            os << "TIME phase=interf ms=" << v2dump_r(t_ms(t_mark, t_now())) << "\n";
            t_mark = t_now();
        }
        os << "PROBE vv_fused=" << st.vv_fused << " ve_paves=" << st.ve_paves
           << " ee_vertex=" << st.ee_vertex_parts << " ee_edge=" << st.ee_edge_parts
           << " ee_newv=" << st.ee_new_vertices << " ee_cb=" << st.ee_common_blocks
           << " vf_in=" << st.vf_in << " ef_transversal=" << st.ef_transversal
           << " ef_tangential=" << st.ef_tangential << " ef_coincident=" << st.ef_coincident
           << " ef_newv=" << st.ef_new_vertices << "\n";
    }

    // ---- phase 1: the arena replica the production front builds ----------------------------
    BdsArena ds;
    ds.init(ops, std::max(tol, 1e-7));
    for (int i = 0; i < ds.nb_shapes(); ++i)
        if (ds.is_edge(i)) ds.change_pave_blocks(i);

    v2sec::V2SectionParams prm;
    if (opt.trail_samples > 0)
        prm.trail_samples = opt.trail_samples;
    else {
        const char* p = std::getenv("SESSION_V2_TRAIL_N");
        prm.trail_samples = (p && p[0]) ? std::max(16, std::atoi(p)) : 768;
    }
    v2sec::V2Section S(ds, ops, prm);
    const int nok = S.perform_all();
    const v2sec::V2SectionStats& ss = S.stats();
    stage_summary(os, ds, "after_FF", (int)S.curves().size(), ss.tangent);
    attach_section_paves(ds, S, ops, tol);
    stage_summary(os, ds, "final", (int)S.curves().size(), ss.tangent);
    v2dump_arena(os, ds, "final");
    v2dump_section(os, ds, S, "final");
    os << "SECSTAT pairs=" << ss.pairs << " ok=" << ss.ok << " tangent=" << ss.tangent
       << " empty=" << ss.empty << " nogeom=" << ss.nogeom << " failed=" << ss.failed
       << " curves=" << ss.curves << " blocks=" << ss.blocks << " kept=" << ss.kept
       << " dropped=" << ss.dropped << " trim_paves=" << ss.trim_paves
       << " seam_paves=" << ss.seam_paves << " bound_paves=" << ss.bound_paves
       << " section_edges=" << ss.section_edges << " fused_groups=" << ss.fused_groups
       << " common_blocks=" << ss.common_blocks << " max_dev=" << v2dump_r(ss.max_dev)
       << " okpairs=" << nok << "\n";

    if (timing) {
        os << "TIME phase=front ms=" << v2dump_r(t_ms(t_mark, t_now())) << "\n";
        t_mark = t_now();
    }

    // ---- phase 3: the production boolean ---------------------------------------------------
    V2DumpResultCounts rc;
    v2sol::V2BooleanReport rep;
    if (opt.run_result) {
        v2sol::V2Op op = v2sol::V2Op::Cut;
        if (opt.op == "common") op = v2sol::V2Op::Common;
        else if (opt.op == "fuse") op = v2sol::V2Op::Fuse;
        else if (opt.op == "cut21") op = v2sol::V2Op::Cut21;
        v2sol::V2BooleanOptions bo;
        const BRep res = v2sol::v2_boolean(A, B, op, bo, &rep);
        // SEPARATED ON PURPOSE. Phase 3 is v2_boolean AND the shared verdict/mass-properties
        // pass, and attributing their combined cost to either one would be a guess; the two
        // marks below measure them apart.
        if (timing) {
            os << "TIME phase=boolean ms=" << v2dump_r(t_ms(t_mark, t_now())) << "\n";
            t_mark = t_now();
        }
        os << "BOP errors=" << (rep.stage_fail.empty() ? 0 : 1) << " warnings="
           << (int)rep.alerts.size() << "\n";
        if (!rep.stage_fail.empty()) os << "BOPERR " << rep.stage_fail << "\n";
        rc = v2dump_result(os, res);
        if (timing) {
            os << "TIME phase=verdict ms=" << v2dump_r(t_ms(t_mark, t_now())) << "\n";
            t_mark = t_now();
        }
    }

    int npb = 0, nfi = 0, nsecpb = 0, nnewv = 0, nsd = 0;
    for (int i = 0; i < ds.nb_shapes(); ++i) {
        if (ds.is_edge(i) && ds.has_pave_blocks(i)) npb += (int)ds.pave_blocks(i).size();
        if (ds.is_face(i) && ds.has_face_info(i)) ++nfi;
        if (ds.is_vertex(i) && ds.is_new_shape(i)) ++nnewv;
        int isd = 0;
        if (ds.has_shape_sd(i, isd)) ++nsd;
    }
    for (const auto& c : S.curves())
        for (const auto& b : c.blocks)
            if (b.kept) ++nsecpb;

    os << "SUMMARY name=" << opt.name << " op=" << opt.op << " dsshapes=" << ds.nb_shapes()
       << " dssource=" << ds.nb_source_shapes() << " newverts=" << nnewv << " sd=" << nsd
       << " pb=" << npb << " cb=" << (int)collect_cb(ds).size() << " faceinfo=" << nfi
       << " VV=" << ds.nb_interf(BdsInterfType::VV) << " VE=" << ds.nb_interf(BdsInterfType::VE)
       << " VF=" << ds.nb_interf(BdsInterfType::VF) << " EE=" << ds.nb_interf(BdsInterfType::EE)
       << " EF=" << ds.nb_interf(BdsInterfType::EF) << " FF=" << ds.nb_interf(BdsInterfType::FF)
       << " seccurves=" << (int)S.curves().size() << " secpoints=0 fftangent=" << ss.tangent
       << " secpb=" << nsecpb << " res_solid=" << rc.nsolid << " res_shell=" << rc.nshell
       << " res_face=" << rc.nface << " res_edge=" << rc.nedge << " res_vert=" << rc.nvert
       << " res_naked=" << rc.nnaked << " res_degen=" << rc.ndegen
       << " res_vol=" << v2dump_r(rc.vol) << " res_area=" << v2dump_r(rc.area)
       << " res_valid=" << rc.valid << " pf_err=0 bop_err="
       << (rep.stage_fail.empty() ? 0 : 1) << "\n";
    return 0;
}

}  // namespace v2dump
}  // namespace session_cpp
