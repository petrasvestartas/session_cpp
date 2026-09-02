#include <map>
#include "file_step.h"
#include "point.h"
#include "vector.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"
#include "closest.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <cstddef>
#include <cctype>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <functional>

namespace session_cpp { namespace file_step {

// ============================================================
// Layer 1: ISO 10303-21 parser
// ============================================================

enum class StepTag { Ref, Num, Str, Enum, List, Null };

struct StepParam {
    StepTag tag = StepTag::Null;
    int ref_id = 0;
    double num = 0.0;
    std::string str;
    std::vector<StepParam> list;
};

struct StepSubEntity {
    std::string type;
    std::vector<StepParam> params;
};

struct StepEntity {
    int id = 0;
    std::vector<StepSubEntity> parts; // 1 for simple, N for complex
    bool has(const std::string& t) const {
        for (const auto& p : parts) if (p.type == t) return true;
        return false;
    }
    const StepSubEntity* find(const std::string& t) const {
        for (const auto& p : parts) if (p.type == t) return &p;
        return nullptr;
    }
};

struct StepFile {
    std::unordered_map<int, StepEntity> entities;
    std::vector<int> ids_of_type(const std::string& t) const {
        std::vector<int> out;
        for (const auto& kv : entities)
            if (kv.second.has(t)) out.push_back(kv.first);
        return out;
    }
    std::unordered_map<std::string, size_t> count_by_type() const {
        std::unordered_map<std::string, size_t> cnt;
        for (const auto& kv : entities)
            for (const auto& p : kv.second.parts) cnt[p.type]++;
        return cnt;
    }
};

// ---- Lexer helpers ----

static void skip_ws(const char*& p, const char* end) {
    while (p < end && std::isspace((unsigned char)*p)) p++;
}

static bool consume(const char*& p, const char* end, char c) {
    skip_ws(p, end);
    if (p < end && *p == c) { p++; return true; }
    return false;
}

static bool isident(char c) {
    return std::isupper((unsigned char)c) || std::isdigit((unsigned char)c) || c == '_';
}

static StepParam parse_param(const char*& p, const char* end);

static std::vector<StepParam> parse_params(const char*& p, const char* end) {
    std::vector<StepParam> out;
    consume(p, end, '(');
    while (true) {
        skip_ws(p, end);
        if (p >= end || *p == ')') break;
        out.push_back(parse_param(p, end));
        skip_ws(p, end);
        if (p < end && *p == ',') p++;
    }
    consume(p, end, ')');
    return out;
}

static StepParam parse_param(const char*& p, const char* end) {
    skip_ws(p, end);
    StepParam r;
    if (p >= end) return r;
    char c = *p;
    if (c == '#') {
        p++;
        int id = 0;
        while (p < end && std::isdigit((unsigned char)*p)) id = id * 10 + (*p++ - '0');
        r.tag = StepTag::Ref; r.ref_id = id;
        return r;
    }
    if (c == '$' || c == '*') { p++; r.tag = StepTag::Null; return r; }
    if (c == '(') {
        r.tag = StepTag::List;
        r.list = parse_params(p, end);
        return r;
    }
    if (c == '\'') {
        p++;
        while (p < end) {
            if (*p == '\'') {
                p++;
                if (p < end && *p == '\'') { r.str += '\''; p++; }
                else break;
            } else { r.str += *p++; }
        }
        r.tag = StepTag::Str; return r;
    }
    if (c == '.') {
        p++;
        while (p < end && *p != '.') r.str += *p++;
        if (p < end) p++;
        r.tag = StepTag::Enum; return r;
    }
    if (std::isdigit((unsigned char)c) || c == '-' || c == '+') {
        char* ep;
        r.num = std::strtod(p, &ep);
        p = ep;
        r.tag = StepTag::Num; return r;
    }
    if (std::isupper((unsigned char)c)) {
        while (p < end && isident(*p)) r.str += *p++;
        skip_ws(p, end);
        if (p < end && *p == '(') {
            r.tag = StepTag::Enum; // typed param — treat str as typed name, ignore sub-params
            const char* tmp = p;
            parse_params(tmp, end); // consume and discard
            p = tmp;
        } else {
            r.tag = StepTag::Enum;
        }
        return r;
    }
    p++; return r; // unknown → null
}

static void parse_step_string(const std::string& content, StepFile& sf) {
    const char* p = content.data();
    const char* end = p + content.size();
    while (p < end) {
        skip_ws(p, end);
        if (p >= end) break;
        if (*p != '#') { while (p < end && *p != '\n') p++; continue; }
        p++;
        int id = 0;
        while (p < end && std::isdigit((unsigned char)*p)) id = id * 10 + (*p++ - '0');
        if (!consume(p, end, '=')) continue;
        skip_ws(p, end);
        if (p >= end) break;
        StepEntity ent;
        ent.id = id;
        if (*p == '(') {
            // complex entity: #id=(TYPE1(...)TYPE2(...)...)
            p++; // consume outer '('
            while (p < end) {
                skip_ws(p, end);
                if (*p == ')') { p++; break; }
                if (!std::isupper((unsigned char)*p)) break;
                StepSubEntity sub;
                while (p < end && isident(*p)) sub.type += *p++;
                skip_ws(p, end);
                if (p < end && *p == '(') sub.params = parse_params(p, end);
                ent.parts.push_back(std::move(sub));
            }
        } else {
            // simple entity: #id=TYPE(...)
            StepSubEntity sub;
            while (p < end && isident(*p)) sub.type += *p++;
            skip_ws(p, end);
            if (p < end && *p == '(') sub.params = parse_params(p, end);
            ent.parts.push_back(std::move(sub));
        }
        sf.entities.emplace(id, std::move(ent));
        // skip to semicolon
        bool in_str = false;
        while (p < end) {
            if (*p == '\'' && !in_str) in_str = true;
            else if (*p == '\'' && in_str) in_str = false;
            else if (*p == ';' && !in_str) { p++; break; }
            p++;
        }
    }
}

static StepFile parse_step_file(const std::string& filepath) {
    std::ifstream in(filepath);
    std::stringstream buf; buf << in.rdbuf();
    std::string raw = buf.str();
    // strip comments
    std::string text;
    text.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ) {
        if (i + 1 < raw.size() && raw[i] == '/' && raw[i+1] == '*') {
            i += 2;
            while (i + 1 < raw.size() && !(raw[i] == '*' && raw[i+1] == '/')) i++;
            i += 2;
        } else { text += raw[i++]; }
    }
    // extract DATA section
    auto lo = text.find("DATA");
    if (lo == std::string::npos) return StepFile{};
    auto semi = text.find(';', lo);
    if (semi == std::string::npos) return StepFile{};
    auto endsec = text.find("ENDSEC", semi);
    if (endsec == std::string::npos) return StepFile{};
    std::string data = text.substr(semi + 1, endsec - semi - 1);
    StepFile sf;
    parse_step_string(data, sf);
    return sf;
}

// ============================================================
// Layer 2: Knot utilities
// ============================================================

// STEP (vals, mults) → full knot vector
static std::vector<double> expand_knots(const std::vector<double>& vals,
                                        const std::vector<int>& mults) {
    std::vector<double> flat;
    for (size_t i = 0; i < vals.size() && i < mults.size(); i++)
        for (int j = 0; j < mults[i]; j++) flat.push_back(vals[i]);
    return flat;
}

// full knot vector → (vals, mults)
static std::pair<std::vector<double>, std::vector<int>> compress_knots(
    const std::vector<double>& flat) {
    std::vector<double> vals;
    std::vector<int> mults;
    for (double v : flat) {
        if (vals.empty() || std::abs(v - vals.back()) > 1e-12) {
            vals.push_back(v);
            mults.push_back(1);
        } else {
            mults.back()++;
        }
    }
    return {vals, mults};
}

// Our m_nurbsknot is the full knot vector minus first and last element.
// Reconstruct full: prepend and append boundary knots.
static std::vector<double> full_from_internal(const std::vector<double>& internal) {
    if (internal.empty()) return {};
    std::vector<double> full;
    full.push_back(internal.front());
    full.insert(full.end(), internal.begin(), internal.end());
    full.push_back(internal.back());
    return full;
}

// Extract internal from full: strip first and last element.
static std::vector<double> internal_from_full(const std::vector<double>& full) {
    if (full.size() < 2) return full;
    return std::vector<double>(full.begin() + 1, full.end() - 1);
}

// ============================================================
// Layer 3: StepReader
// ============================================================

struct Axis2 { Point origin; Vector ax, ay, az; bool ok = false; };

class StepReader {
    const StepFile& sf;
    std::unordered_map<int, Point> pt_cache;
    std::unordered_map<int, Vector> dir_cache;
    std::unordered_map<int, Axis2> ax_cache;

    const StepEntity* get(int id) const {
        auto it = sf.entities.find(id);
        return (it == sf.entities.end()) ? nullptr : &it->second;
    }

    // Extract coords from CARTESIAN_POINT params
    static std::vector<double> coords(const std::vector<StepParam>& params) {
        for (const auto& p : params)
            if (p.tag == StepTag::List) {
                std::vector<double> out;
                for (const auto& v : p.list) if (v.tag == StepTag::Num) out.push_back(v.num);
                if (!out.empty()) return out;
            }
        return {};
    }

    // Extract first ref from params
    static int first_ref(const std::vector<StepParam>& params) {
        for (const auto& p : params) if (p.tag == StepTag::Ref) return p.ref_id;
        return -1;
    }

    // Extract all refs from params (first list of refs found)
    static std::vector<int> ref_list(const std::vector<StepParam>& params) {
        for (const auto& p : params)
            if (p.tag == StepTag::List) {
                std::vector<int> out;
                for (const auto& v : p.list) if (v.tag == StepTag::Ref) out.push_back(v.ref_id);
                if (!out.empty()) return out;
            }
        return {};
    }

    // Extract all ints from a LIST param
    static std::vector<int> int_list(const StepParam& p) {
        std::vector<int> out;
        if (p.tag == StepTag::List)
            for (const auto& v : p.list) if (v.tag == StepTag::Num) out.push_back((int)v.num);
        return out;
    }

    // Extract all doubles from a LIST param
    static std::vector<double> dbl_list(const StepParam& p) {
        std::vector<double> out;
        if (p.tag == StepTag::List)
            for (const auto& v : p.list) if (v.tag == StepTag::Num) out.push_back(v.num);
        return out;
    }

    // Extract all doubles from a LIST of LISTs (2D grid, outer=rows)
    static std::vector<std::vector<double>> dbl_list_list(const StepParam& p) {
        std::vector<std::vector<double>> out;
        if (p.tag == StepTag::List)
            for (const auto& row : p.list) out.push_back(dbl_list(row));
        return out;
    }

    // Extract 2D grid of refs: outer list → rows, inner list → refs
    static std::vector<std::vector<int>> ref_list_list(const StepParam& p) {
        std::vector<std::vector<int>> out;
        if (p.tag == StepTag::List)
            for (const auto& row : p.list) {
                std::vector<int> row_refs;
                if (row.tag == StepTag::List)
                    for (const auto& v : row.list) if (v.tag == StepTag::Ref) row_refs.push_back(v.ref_id);
                out.push_back(std::move(row_refs));
            }
        return out;
    }

public:
    explicit StepReader(const StepFile& s) : sf(s) {}

    Point get_point(int id) {
        auto it = pt_cache.find(id);
        if (it != pt_cache.end()) return it->second;
        Point pt(0, 0, 0);
        const StepEntity* e = get(id);
        if (e) {
            const StepSubEntity* sub = e->find("CARTESIAN_POINT");
            if (sub) {
                auto c = coords(sub->params);
                if (c.size() >= 3) pt = Point(c[0], c[1], c[2]);
                else if (c.size() == 2) pt = Point(c[0], c[1], 0.0);
            }
        }
        return pt_cache[id] = pt;
    }

    Vector get_direction(int id) {
        auto it = dir_cache.find(id);
        if (it != dir_cache.end()) return it->second;
        Vector v(0, 0, 1);
        const StepEntity* e = get(id);
        if (e) {
            const StepSubEntity* sub = e->find("DIRECTION");
            if (sub) {
                auto c = coords(sub->params);
                if (c.size() >= 3) v = Vector(c[0], c[1], c[2]);
            }
        }
        return dir_cache[id] = v;
    }

    Axis2 get_axis2(int id) {
        auto it = ax_cache.find(id);
        if (it != ax_cache.end()) return it->second;
        Axis2 a;
        const StepEntity* e = get(id);
        if (e) {
            const StepSubEntity* sub = e->find("AXIS2_PLACEMENT_3D");
            if (sub) {
                std::vector<int> refs;
                for (const auto& p : sub->params) if (p.tag == StepTag::Ref) refs.push_back(p.ref_id);
                if (!refs.empty()) {
                    a.origin = get_point(refs[0]);
                    a.az = (refs.size() > 1) ? get_direction(refs[1]) : Vector(0,0,1);
                    double ln = std::sqrt(a.az[0]*a.az[0]+a.az[1]*a.az[1]+a.az[2]*a.az[2]);
                    if (ln > 1e-12) a.az = Vector(a.az[0]/ln, a.az[1]/ln, a.az[2]/ln);
                    if (refs.size() > 2) {
                        a.ax = get_direction(refs[2]);
                    } else {
                        a.ax = (std::abs(a.az[0]) < 0.9) ? Vector(1,0,0) : Vector(0,1,0);
                    }
                    double dot = a.ax[0]*a.az[0]+a.ax[1]*a.az[1]+a.ax[2]*a.az[2];
                    a.ax = Vector(a.ax[0]-dot*a.az[0], a.ax[1]-dot*a.az[1], a.ax[2]-dot*a.az[2]);
                    double xn = std::sqrt(a.ax[0]*a.ax[0]+a.ax[1]*a.ax[1]+a.ax[2]*a.ax[2]);
                    if (xn > 1e-12) a.ax = Vector(a.ax[0]/xn, a.ax[1]/xn, a.ax[2]/xn);
                    a.ay = Vector(a.az[1]*a.ax[2]-a.az[2]*a.ax[1],
                                  a.az[2]*a.ax[0]-a.az[0]*a.ax[2],
                                  a.az[0]*a.ax[1]-a.az[1]*a.ax[0]);
                    a.ok = true;
                }
            }
        }
        return ax_cache[id] = a;
    }

    // Read B_SPLINE_CURVE_WITH_KNOTS (simple or complex with RATIONAL_B_SPLINE_CURVE)
    NurbsCurve get_nurbs_curve(int id) {
        const StepEntity* e = get(id);
        if (!e) return NurbsCurve();

        // Simple entity: B_SPLINE_CURVE_WITH_KNOTS(name,degree,(pts),form,closed,self,mults,knots,spec)
        const StepSubEntity* bsc = e->find("B_SPLINE_CURVE_WITH_KNOTS");
        const StepSubEntity* bsc_base = e->find("B_SPLINE_CURVE");
        const StepSubEntity* rat = e->find("RATIONAL_B_SPLINE_CURVE");

        int degree = 0;
        std::vector<int> pt_refs;
        std::vector<int> mults_i;
        std::vector<double> knots_d;
        std::vector<double> weights;

        if (bsc && !bsc_base) {
            // Simple entity: all attrs collapsed into B_SPLINE_CURVE_WITH_KNOTS
            // params: [0]=name, [1]=degree, [2]=pts_list, [3]=form, [4]=closed, [5]=self, [6]=mults, [7]=knots, [8]=spec
            const auto& pp = bsc->params;
            if (pp.size() < 8) return NurbsCurve();
            if (pp[1].tag == StepTag::Num) degree = (int)pp[1].num;
            if (pp[2].tag == StepTag::List) for (const auto& v : pp[2].list) if (v.tag == StepTag::Ref) pt_refs.push_back(v.ref_id);
            if (pp[6].tag == StepTag::List) mults_i = int_list(pp[6]);
            if (pp[7].tag == StepTag::List) knots_d = dbl_list(pp[7]);
        } else if (bsc_base && bsc) {
            // Complex entity: B_SPLINE_CURVE has degree+pts, B_SPLINE_CURVE_WITH_KNOTS has mults+knots
            const auto& base_pp = bsc_base->params;
            const auto& knt_pp = bsc->params;
            if (!base_pp.empty() && base_pp[0].tag == StepTag::Num) degree = (int)base_pp[0].num;
            if (base_pp.size() > 1 && base_pp[1].tag == StepTag::List)
                for (const auto& v : base_pp[1].list) if (v.tag == StepTag::Ref) pt_refs.push_back(v.ref_id);
            if (!knt_pp.empty()) mults_i = int_list(knt_pp[0]);
            if (knt_pp.size() > 1) knots_d = dbl_list(knt_pp[1]);
        } else {
            return NurbsCurve();
        }

        if (pt_refs.empty() || mults_i.empty() || knots_d.empty()) return NurbsCurve();
        int order = degree + 1;
        int cv_count = (int)pt_refs.size();

        auto full = expand_knots(knots_d, mults_i);
        if ((int)full.size() != cv_count + order) return NurbsCurve();
        auto internal = internal_from_full(full);

        bool is_rat = (rat != nullptr);
        if (is_rat && !rat->params.empty() && rat->params[0].tag == StepTag::List)
            weights = dbl_list(rat->params[0]);

        NurbsCurve nc(3, is_rat, order, cv_count);
        if ((int)nc.m_nurbsknot.size() != (int)internal.size()) return NurbsCurve();
        std::copy(internal.begin(), internal.end(), nc.m_nurbsknot.begin());

        double* cv = nc.cv_array();
        int stride = nc.m_cv_stride;
        for (int i = 0; i < cv_count; i++) {
            Point pt = get_point(pt_refs[i]);
            if (is_rat) {
                double w = (i < (int)weights.size()) ? weights[i] : 1.0;
                cv[i*stride+0] = w * pt[0];
                cv[i*stride+1] = w * pt[1];
                cv[i*stride+2] = w * pt[2];
                cv[i*stride+3] = w;
            } else {
                cv[i*stride+0] = pt[0];
                cv[i*stride+1] = pt[1];
                cv[i*stride+2] = pt[2];
            }
        }
        return nc;
    }

    // Read B_SPLINE_SURFACE_WITH_KNOTS (simple or complex with RATIONAL_B_SPLINE_SURFACE)
    NurbsSurface get_nurbs_surface(int id) {
        const StepEntity* e = get(id);
        if (!e) return NurbsSurface();

        const StepSubEntity* bss = e->find("B_SPLINE_SURFACE_WITH_KNOTS");
        const StepSubEntity* bss_base = e->find("B_SPLINE_SURFACE");
        const StepSubEntity* rat = e->find("RATIONAL_B_SPLINE_SURFACE");

        int u_deg = 0, v_deg = 0;
        std::vector<std::vector<int>> ctrl_pts; // ctrl_pts[u][v]
        std::vector<int> u_mults, v_mults;
        std::vector<double> u_knots, v_knots;
        std::vector<std::vector<double>> weights; // weights[u][v]

        if (bss && !bss_base) {
            // Simple: name,u_deg,v_deg,ctrl_pts,form,u_closed,v_closed,self,u_mults,v_mults,u_knots,v_knots,spec
            const auto& pp = bss->params;
            if (pp.size() < 12) return NurbsSurface();
            if (pp[1].tag == StepTag::Num) u_deg = (int)pp[1].num;
            if (pp[2].tag == StepTag::Num) v_deg = (int)pp[2].num;
            if (pp[3].tag == StepTag::List) ctrl_pts = ref_list_list(pp[3]);
            if (pp[8].tag == StepTag::List) u_mults = int_list(pp[8]);
            if (pp[9].tag == StepTag::List) v_mults = int_list(pp[9]);
            if (pp[10].tag == StepTag::List) u_knots = dbl_list(pp[10]);
            if (pp[11].tag == StepTag::List) v_knots = dbl_list(pp[11]);
        } else if (bss_base && bss) {
            // Complex: B_SPLINE_SURFACE has u_deg,v_deg,ctrl_pts; B_SPLINE_SURFACE_WITH_KNOTS has mults/knots
            const auto& base_pp = bss_base->params;
            const auto& knt_pp = bss->params;
            if (base_pp.size() < 3) return NurbsSurface();
            if (base_pp[0].tag == StepTag::Num) u_deg = (int)base_pp[0].num;
            if (base_pp[1].tag == StepTag::Num) v_deg = (int)base_pp[1].num;
            if (base_pp[2].tag == StepTag::List) ctrl_pts = ref_list_list(base_pp[2]);
            if (knt_pp.size() > 3) {
                u_mults = int_list(knt_pp[0]);
                v_mults = int_list(knt_pp[1]);
                u_knots = dbl_list(knt_pp[2]);
                v_knots = dbl_list(knt_pp[3]);
            }
        } else {
            return NurbsSurface();
        }

        if (ctrl_pts.empty() || u_mults.empty() || v_mults.empty()) return NurbsSurface();

        int cv_u = (int)ctrl_pts.size();
        int cv_v = ctrl_pts.empty() ? 0 : (int)ctrl_pts[0].size();
        if (cv_u == 0 || cv_v == 0) return NurbsSurface();

        auto full_u = expand_knots(u_knots, u_mults);
        auto full_v = expand_knots(v_knots, v_mults);
        if ((int)full_u.size() != cv_u + u_deg + 1) return NurbsSurface();
        if ((int)full_v.size() != cv_v + v_deg + 1) return NurbsSurface();
        auto int_u = internal_from_full(full_u);
        auto int_v = internal_from_full(full_v);

        bool is_rat = (rat != nullptr);
        if (is_rat && !rat->params.empty() && rat->params[0].tag == StepTag::List)
            weights = dbl_list_list(rat->params[0]);

        NurbsSurface srf(3, is_rat, u_deg + 1, v_deg + 1, cv_u, cv_v);
        if ((int)srf.m_nurbsknot[0].size() != (int)int_u.size()) return NurbsSurface();
        if ((int)srf.m_nurbsknot[1].size() != (int)int_v.size()) return NurbsSurface();
        std::copy(int_u.begin(), int_u.end(), srf.m_nurbsknot[0].begin());
        std::copy(int_v.begin(), int_v.end(), srf.m_nurbsknot[1].begin());

        for (int u = 0; u < cv_u; u++) {
            for (int v = 0; v < cv_v && v < (int)ctrl_pts[u].size(); v++) {
                Point pt = get_point(ctrl_pts[u][v]);
                if (is_rat) {
                    double w = (u < (int)weights.size() && v < (int)weights[u].size()) ? weights[u][v] : 1.0;
                    srf.set_cv_4d(u, v, w*pt[0], w*pt[1], w*pt[2], w);
                } else {
                    srf.set_cv(u, v, pt);
                }
            }
        }
        return srf;
    }

    // EDGE_CURVE geometry may be wrapped: SURFACE_CURVE / SEAM_CURVE('',#basis,(#pcurves),enum)
    // carry the 3D basis curve as their first reference. Dereference for 3D evaluation.
    int basis_curve_of(int curve_id) {
        const StepEntity* e = get(curve_id);
        if (!e) return curve_id;
        const StepSubEntity* sc = e->find("SURFACE_CURVE");
        if (!sc) sc = e->find("SEAM_CURVE");
        if (!sc) return curve_id;
        int ref = first_ref(sc->params);
        return ref >= 0 ? ref : curve_id;
    }

    // Sample 3D points along a curve entity
    std::vector<Point> sample_curve(int curve_id, const Point& v_start, const Point& v_end, int n) {
        curve_id = basis_curve_of(curve_id);
        const StepEntity* e = get(curve_id);
        if (!e) return {v_start, v_end};
        if (e->has("B_SPLINE_CURVE_WITH_KNOTS")) {
            NurbsCurve nc = get_nurbs_curve(curve_id);
            if (nc.cv_count() >= nc.order() && nc.order() >= 2) {
                auto kts = nc.get_nurbsknots();
                if (kts.empty()) return {v_start, v_end};
                int deg = nc.order() - 1;
                double tmin = kts[deg > 0 ? deg - 1 : 0];
                double tmax = kts[kts.size() - (deg > 0 ? deg : 1)];
                std::vector<Point> pts;
                for (int i = 0; i < n; i++) {
                    double t = (n > 1) ? tmin + (tmax - tmin) * i / (n - 1) : tmin;
                    pts.push_back(nc.point_at(t));
                }
                return pts;
            }
        }
        if (e->has("LINE")) return {v_start, v_end};
        if (e->has("CIRCLE")) {
            const StepSubEntity* sub = e->find("CIRCLE");
            if (!sub) return {v_start, v_end};
            int ax_ref = first_ref(sub->params);
            double rad = 0;
            for (const auto& p : sub->params) if (p.tag == StepTag::Num) rad = p.num;
            if (ax_ref < 0 || rad == 0) return {v_start, v_end};
            Axis2 a = get_axis2(ax_ref);
            if (!a.ok) return {v_start, v_end};
            auto angle_of = [&](const Point& pt) {
                double dx=pt[0]-a.origin[0], dy=pt[1]-a.origin[1], dz=pt[2]-a.origin[2];
                double u=dx*a.ax[0]+dy*a.ax[1]+dz*a.ax[2], v2=dx*a.ay[0]+dy*a.ay[1]+dz*a.ay[2];
                return std::atan2(v2, u);
            };
            double sa = angle_of(v_start), ea = angle_of(v_end);
            if (ea <= sa) ea += 2.0 * 3.14159265358979323846;
            std::vector<Point> pts;
            for (int i = 0; i < n; i++) {
                double t = (n > 1) ? (double)i / (n - 1) : 0.0;
                double ang = sa + t * (ea - sa);
                pts.emplace_back(
                    a.origin[0]+rad*(std::cos(ang)*a.ax[0]+std::sin(ang)*a.ay[0]),
                    a.origin[1]+rad*(std::cos(ang)*a.ax[1]+std::sin(ang)*a.ay[1]),
                    a.origin[2]+rad*(std::cos(ang)*a.ax[2]+std::sin(ang)*a.ay[2]));
            }
            return pts;
        }
        if (e->has("TRIMMED_CURVE")) {
            const StepSubEntity* sub = e->find("TRIMMED_CURVE");
            if (sub) {
                int ref = first_ref(sub->params);
                if (ref >= 0) return sample_curve(ref, v_start, v_end, n);
            }
        }
        return {v_start, v_end};
    }

    // Project 3D samples onto UV (for PLANE and CYLINDRICAL_SURFACE)
    enum class ProjKind { None, Plane, Cyl };
    struct Proj { ProjKind kind = ProjKind::None; Axis2 a; };

    Proj get_projector(int surface_id) {
        Proj pr;
        const StepEntity* e = get(surface_id);
        if (!e) return pr;
        if (e->has("PLANE")) {
            const StepSubEntity* sub = e->find("PLANE");
            if (!sub) return pr;
            int ref = first_ref(sub->params);
            if (ref < 0) return pr;
            pr.a = get_axis2(ref); pr.kind = ProjKind::Plane;
        } else if (e->has("CYLINDRICAL_SURFACE")) {
            const StepSubEntity* sub = e->find("CYLINDRICAL_SURFACE");
            if (!sub) return pr;
            int ref = first_ref(sub->params);
            if (ref < 0) return pr;
            pr.a = get_axis2(ref); pr.kind = ProjKind::Cyl;
        }
        return pr;
    }

    std::pair<double,double> project(const Proj& pr, const Point& pt) {
        double dx=pt[0]-pr.a.origin[0], dy=pt[1]-pr.a.origin[1], dz=pt[2]-pr.a.origin[2];
        if (pr.kind == ProjKind::Plane) {
            return { dx*pr.a.ax[0]+dy*pr.a.ax[1]+dz*pr.a.ax[2],
                     dx*pr.a.ay[0]+dy*pr.a.ay[1]+dz*pr.a.ay[2] };
        } else { // Cyl: canonical U = angle * 2/pi so 1 unit = 90deg, full circle = 4 units
            double xl=dx*pr.a.ax[0]+dy*pr.a.ax[1]+dz*pr.a.ax[2];
            double yl=dx*pr.a.ay[0]+dy*pr.a.ay[1]+dz*pr.a.ay[2];
            double hl=dx*pr.a.az[0]+dy*pr.a.az[1]+dz*pr.a.az[2];
            const double pi = 3.14159265358979323846;
            return { std::atan2(yl, xl) * 2.0 / pi, hl };
        }
    }

    // Build NurbsSurface from PLANE/CYLINDRICAL/B_SPLINE given UV bounds
    // Build surface from entity id into 'out'; returns true if successful
    bool fill_surface(int id, double u0, double u1, double v0, double v1, NurbsSurface& out) {
        const StepEntity* e = get(id);
        if (!e) return false;
        const double pi_ = 3.14159265358979323846;
        // Canonical cyl UV: 1 unit = 90deg (π/2 rad), full circle = 4 units.
        // No U padding for cylinders: integer arc count must exactly cover the trim span.
        bool is_closed_cyl = e->has("CYLINDRICAL_SURFACE") &&
                             std::abs((u1 - u0) - 4.0) < 0.2;
        double pad_v = std::max(1e-6, 0.01*(v1-v0));
        v0 -= pad_v; v1 += pad_v;
        if (!is_closed_cyl && !e->has("CYLINDRICAL_SURFACE")) {
            double pad_u = std::max(1e-6, 0.01*(u1-u0));
            u0 -= pad_u; u1 += pad_u;
        }

        if (e->has("B_SPLINE_SURFACE_WITH_KNOTS")) {
            out = get_nurbs_surface(id);
            return out.is_valid();
        }

        if (e->has("PLANE")) {
            const StepSubEntity* sub = e->find("PLANE");
            if (!sub) return false;
            int ax_ref = first_ref(sub->params);
            if (ax_ref < 0) return false;
            Axis2 a = get_axis2(ax_ref);
            if (!a.ok) return false;
            out.create_raw(3, false, 2, 2, 2, 2);
            out.m_nurbsknot[0] = {u0, u1};
            out.m_nurbsknot[1] = {v0, v1};
            out.set_cv(0, 0, Point(a.origin[0]+u0*a.ax[0]+v0*a.ay[0], a.origin[1]+u0*a.ax[1]+v0*a.ay[1], a.origin[2]+u0*a.ax[2]+v0*a.ay[2]));
            out.set_cv(0, 1, Point(a.origin[0]+u0*a.ax[0]+v1*a.ay[0], a.origin[1]+u0*a.ax[1]+v1*a.ay[1], a.origin[2]+u0*a.ax[2]+v1*a.ay[2]));
            out.set_cv(1, 0, Point(a.origin[0]+u1*a.ax[0]+v0*a.ay[0], a.origin[1]+u1*a.ax[1]+v0*a.ay[1], a.origin[2]+u1*a.ax[2]+v0*a.ay[2]));
            out.set_cv(1, 1, Point(a.origin[0]+u1*a.ax[0]+v1*a.ay[0], a.origin[1]+u1*a.ax[1]+v1*a.ay[1], a.origin[2]+u1*a.ax[2]+v1*a.ay[2]));
            return true;
        }

        if (e->has("CYLINDRICAL_SURFACE")) {
            const StepSubEntity* sub = e->find("CYLINDRICAL_SURFACE");
            if (!sub) return false;
            int ax_ref = -1;
            double radius = 1.0;
            for (const auto& p : sub->params) {
                if (p.tag == StepTag::Ref && ax_ref < 0) ax_ref = p.ref_id;
                else if (p.tag == StepTag::Num) radius = p.num;
            }
            if (ax_ref < 0) return false;
            Axis2 a = get_axis2(ax_ref);
            if (!a.ok) return false;
            // Canonical U: 1 unit = π/2 rad.  n_spans = number of quarter-circle arcs.
            int n_spans = is_closed_cyl ? 4
                        : std::max(1, (int)std::ceil(std::abs(u1 - u0) - 1e-9));
            if (is_closed_cyl) u1 = u0 + 4.0;
            int n_u = 2 * n_spans + 1;
            out.create_raw(3, true, 3, 2, n_u, 2);
            // Integer-spaced knots (same pattern as Primitives::cylinder_surface)
            { int k = 0;
              out.m_nurbsknot[0][k++] = u0; out.m_nurbsknot[0][k++] = u0;
              for (int s = 1; s < n_spans; s++) { out.m_nurbsknot[0][k++] = u0+s; out.m_nurbsknot[0][k++] = u0+s; }
              out.m_nurbsknot[0][k++] = u1; out.m_nurbsknot[0][k++] = u1; }
            out.m_nurbsknot[1] = {v0, v1};
            // Exact Primitives-compatible CVs: on-curve at arc boundaries, off-curve at
            // tangent-intersection corners.  Weight = 1 for on-curve, √2/2 for corners.
            const double w = std::sqrt(2.0) / 2.0;
            for (int i = 0; i < n_u; i++) {
                double lx, ly, wi;
                if (i % 2 == 0) {
                    // On-curve at arc boundary: exact circle point
                    double ang = (u0 + i/2) * (pi_ / 2.0);
                    lx = radius * std::cos(ang);
                    ly = radius * std::sin(ang);
                    wi = 1.0;
                } else {
                    // Off-curve: tangent-intersection corner = sum of adjacent arc endpoints
                    double a0 = (u0 + i/2)     * (pi_ / 2.0);
                    double a1 = (u0 + i/2 + 1) * (pi_ / 2.0);
                    lx = radius * (std::cos(a0) + std::cos(a1));
                    ly = radius * (std::sin(a0) + std::sin(a1));
                    wi = w;
                }
                for (int vi = 0; vi < 2; vi++) {
                    double h = (vi == 0) ? v0 : v1;
                    double px = a.origin[0] + lx*a.ax[0] + ly*a.ay[0] + h*a.az[0];
                    double py = a.origin[1] + lx*a.ax[1] + ly*a.ay[1] + h*a.az[1];
                    double pz = a.origin[2] + lx*a.ax[2] + ly*a.ay[2] + h*a.az[2];
                    out.set_cv_4d(i, vi, wi*px, wi*py, wi*pz, wi);
                }
            }
            return true;
        }
        return false;
    }

    // kept as pass-through for get_projector callers
    NurbsSurface get_surface(int id, double u0, double u1, double v0, double v1) {
        NurbsSurface out;
        fill_surface(id, u0, u1, v0, v1, out);
        return out;
    }

    // ---- Analytic surface reading (CYLINDRICAL/CONICAL/SPHERICAL/TOROIDAL) ----
    // STEP-canonical charts (radians): cyl (s=angle, t=height along axis), cone (s=angle,
    // t=distance along the SLANT from the reference plane; radius(t) = R + t*sin(alpha),
    // z(t) = t*cos(alpha)), sphere (s=longitude, t=latitude in [-pi/2,pi/2]), torus
    // (s=major angle, t=minor angle). The kernel NurbsSurface is built on an integer
    // quarter-arc chart (1 unit = 90 deg) exactly like Primitives::*_surface, so
    // recognize_solid sees reimported primitives as native ones.
    struct AnFace { int kind = 0; Axis2 a; double R = 0, r2 = 0; };  // kind: 2 cyl, 3 cone, 4 sphere, 5 torus

    AnFace get_analytic_srf(int id) {
        AnFace A;
        const StepEntity* e = get(id);
        if (!e) return A;
        const char* kinds[4] = {"CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE", "TOROIDAL_SURFACE"};
        int kk[4] = {2, 3, 4, 5};
        for (int i = 0; i < 4; ++i) {
            const StepSubEntity* sub = e->find(kinds[i]);
            if (!sub) continue;
            int ax_ref = -1;
            std::vector<double> nums;
            for (const auto& p : sub->params) {
                if (p.tag == StepTag::Ref && ax_ref < 0) ax_ref = p.ref_id;
                else if (p.tag == StepTag::Num) nums.push_back(p.num);
            }
            if (ax_ref < 0) return A;
            A.a = get_axis2(ax_ref);
            if (!A.a.ok) return A;
            A.kind = kk[i];
            A.R = nums.empty() ? 0.0 : nums[0];
            A.r2 = nums.size() > 1 ? nums[1] : 0.0;   // cone: semi-angle (rad); torus: minor radius
            return A;
        }
        return A;
    }

    void an_local(const AnFace& A, const Point& p, double& x, double& y, double& z) {
        double wx = p[0]-A.a.origin[0], wy = p[1]-A.a.origin[1], wz = p[2]-A.a.origin[2];
        x = wx*A.a.ax[0] + wy*A.a.ax[1] + wz*A.a.ax[2];
        y = wx*A.a.ay[0] + wy*A.a.ay[1] + wz*A.a.ay[2];
        z = wx*A.a.az[0] + wy*A.a.az[1] + wz*A.a.az[2];
    }

    // Canonical (s,t) of a 3D point. radial_ok=false at poles/apex (angle undefined).
    void an_st_of(const AnFace& A, const Point& p, double& s, double& t, bool& radial_ok) {
        double x, y, z;
        an_local(A, p, x, y, z);
        double rho = std::sqrt(x*x + y*y);
        radial_ok = rho > 1e-9;
        switch (A.kind) {
            case 2: s = std::atan2(y, x); t = z; break;
            case 3: { s = std::atan2(y, x);
                      double ca = std::cos(A.r2);
                      t = (std::abs(ca) > 1e-12) ? z / ca : z; break; }
            case 4: s = std::atan2(y, x); t = std::atan2(z, rho); break;
            default: s = std::atan2(y, x); t = std::atan2(z, rho - A.R); break;
        }
    }

    Point an_eval(const AnFace& A, double s, double t) {
        double cs = std::cos(s), sn = std::sin(s);
        auto at = [&](double lx, double ly, double lz) {
            return Point(A.a.origin[0] + lx*A.a.ax[0] + ly*A.a.ay[0] + lz*A.a.az[0],
                         A.a.origin[1] + lx*A.a.ax[1] + ly*A.a.ay[1] + lz*A.a.az[1],
                         A.a.origin[2] + lx*A.a.ax[2] + ly*A.a.ay[2] + lz*A.a.az[2]);
        };
        switch (A.kind) {
            case 2: return at(A.R*cs, A.R*sn, t);
            case 3: { double r = A.R + t*std::sin(A.r2); return at(r*cs, r*sn, t*std::cos(A.r2)); }
            case 4: { double ct = std::cos(t), st = std::sin(t);
                      return at(A.R*ct*cs, A.R*ct*sn, A.R*st); }
            default: { double ct = std::cos(t), st = std::sin(t);
                       double r = A.R + A.r2*ct; return at(r*cs, r*sn, A.r2*st); }
        }
    }

    // Angle -> parameter within ONE 90-degree rational-quadratic span (w = sqrt(2)/2):
    // x(tau) = (1-tau)^2 + 2w tau(1-tau), y(tau) = 2w tau(1-tau) + tau^2, theta = atan2(y,x).
    // Monotone; a few Newton steps from the linear seed converge to 1e-15.
    static double arc_param_of_angle(double theta) {
        const double pi_2 = 1.5707963267948966;
        if (theta <= 0) return 0.0;
        if (theta >= pi_2) return 1.0;
        const double w = std::sqrt(2.0) / 2.0;
        double tau = theta / pi_2;
        for (int it = 0; it < 8; ++it) {
            double o = 1.0 - tau;
            double x = o*o + 2*w*tau*o, y = 2*w*tau*o + tau*tau;
            double dx = -2*o + 2*w*(1 - 2*tau), dy = 2*w*(1 - 2*tau) + 2*tau;
            double f = std::atan2(y, x) - theta;
            double r2 = x*x + y*y;
            double df = (x*dy - y*dx) / (r2 > 1e-30 ? r2 : 1e-30);
            if (std::abs(df) < 1e-30) break;
            double step = f / df;
            tau -= step;
            if (tau < 0) tau = 0; else if (tau > 1) tau = 1;
            if (std::abs(step) < 1e-15) break;
        }
        return tau;
    }

    // Map a canonical angle (radians) into the integer quarter-arc chart anchored at
    // quarter index q0 (i.e. chart 0 == q0*pi/2), correcting for the projective
    // parameterization inside each span.
    static double chart_u_of_angle(double ang, int q0) {
        const double pi_2 = 1.5707963267948966;
        double q = ang / pi_2 - q0;
        double spanf = std::floor(q + 1e-12);
        double theta = (q - spanf) * pi_2;
        return spanf + arc_param_of_angle(theta);
    }

    // Build the kernel NurbsSurface for an analytic window. u: nsu quarter-arc spans
    // starting at quarter index su0 (integer knots). v: cyl/cone LINEAR [t0,t1] (real
    // knots); sphere/torus quarter-arc spans starting at sv0 (integer knots).
    NurbsSurface build_analytic_nurbs(const AnFace& A, int su0, int nsu, double t0, double t1,
                                      int sv0, int nsv) {
        const double pi_2 = 1.5707963267948966;
        const double w = std::sqrt(2.0) / 2.0;
        int nu = 2*nsu + 1;
        // u-direction node arrays: even = on-arc point at angle, odd = tangent corner (sum
        // of adjacent on-arc dirs, weight w) -- the Primitives::*_surface pattern.
        std::vector<double> ca(nu), sa(nu), cw(nu);
        for (int i = 0; i < nu; ++i) {
            if (i % 2 == 0) {
                double ang = (su0 + i/2) * pi_2;
                ca[i] = std::cos(ang); sa[i] = std::sin(ang); cw[i] = 1.0;
            } else {
                double a0 = (su0 + i/2) * pi_2, a1 = (su0 + i/2 + 1) * pi_2;
                ca[i] = std::cos(a0) + std::cos(a1);
                sa[i] = std::sin(a0) + std::sin(a1);
                cw[i] = w;
            }
        }
        auto set_uknots = [&](NurbsSurface& srf) {
            int k = 0;
            srf.m_nurbsknot[0][k++] = 0; srf.m_nurbsknot[0][k++] = 0;
            for (int s2 = 1; s2 < nsu; ++s2) { srf.m_nurbsknot[0][k++] = s2; srf.m_nurbsknot[0][k++] = s2; }
            srf.m_nurbsknot[0][k++] = nsu; srf.m_nurbsknot[0][k++] = nsu;
        };
        auto place = [&](double lx, double ly, double lz, double& px, double& py, double& pz) {
            px = A.a.origin[0] + lx*A.a.ax[0] + ly*A.a.ay[0] + lz*A.a.az[0];
            py = A.a.origin[1] + lx*A.a.ax[1] + ly*A.a.ay[1] + lz*A.a.az[1];
            pz = A.a.origin[2] + lx*A.a.ax[2] + ly*A.a.ay[2] + lz*A.a.az[2];
        };
        NurbsSurface srf;
        if (A.kind == 2 || A.kind == 3) {
            srf = NurbsSurface(3, true, 3, 2, nu, 2);
            set_uknots(srf);
            srf.m_nurbsknot[1] = {t0, t1};
            for (int j = 0; j < 2; ++j) {
                double t = j == 0 ? t0 : t1;
                double r = (A.kind == 2) ? A.R : A.R + t*std::sin(A.r2);
                double z = (A.kind == 2) ? t : t*std::cos(A.r2);
                for (int i = 0; i < nu; ++i) {
                    double px, py, pz;
                    place(r*ca[i], r*sa[i], z, px, py, pz);
                    srf.set_cv_4d(i, j, cw[i]*px, cw[i]*py, cw[i]*pz, cw[i]);
                }
            }
            return srf;
        }
        // sphere/torus: v is a quarter-arc profile too
        int nv = 2*nsv + 1;
        std::vector<double> cb(nv), sb(nv), vw(nv);
        for (int j = 0; j < nv; ++j) {
            if (j % 2 == 0) {
                double ang = (sv0 + j/2) * pi_2;
                cb[j] = std::cos(ang); sb[j] = std::sin(ang); vw[j] = 1.0;
            } else {
                double a0 = (sv0 + j/2) * pi_2, a1 = (sv0 + j/2 + 1) * pi_2;
                cb[j] = std::cos(a0) + std::cos(a1);
                sb[j] = std::sin(a0) + std::sin(a1);
                vw[j] = w;
            }
        }
        srf = NurbsSurface(3, true, 3, 3, nu, nv);
        set_uknots(srf);
        { int k = 0;
          srf.m_nurbsknot[1][k++] = 0; srf.m_nurbsknot[1][k++] = 0;
          for (int s2 = 1; s2 < nsv; ++s2) { srf.m_nurbsknot[1][k++] = s2; srf.m_nurbsknot[1][k++] = s2; }
          srf.m_nurbsknot[1][k++] = nsv; srf.m_nurbsknot[1][k++] = nsv; }
        for (int j = 0; j < nv; ++j) {
            for (int i = 0; i < nu; ++i) {
                double r, z;
                if (A.kind == 4) { r = A.R * cb[j]; z = A.R * sb[j]; }
                else             { r = A.R + A.r2 * cb[j]; z = A.r2 * sb[j]; }
                double px, py, pz;
                place(r*ca[i], r*sa[i], z, px, py, pz);
                double wij = cw[i] * vw[j];
                srf.set_cv_4d(i, j, wij*px, wij*py, wij*pz, wij);
            }
        }
        return srf;
    }

    // File pcurve of an edge on a given surface, sampled into canonical (s,t) points.
    // SEAM_CURVE carries TWO pcurves on the same surface; ORDER picks the use: first for
    // the FORWARD oriented-edge use, second for the REVERSED one.
    std::vector<Point> pcurve_st_samples(int ec_geom_id, int surface_ref, bool forward_use, int n) {
        const StepEntity* e = get(ec_geom_id);
        if (!e) return {};
        const StepSubEntity* sc = e->find("SURFACE_CURVE");
        bool is_seam = false;
        if (!sc) { sc = e->find("SEAM_CURVE"); is_seam = (sc != nullptr); }
        if (!sc) return {};
        std::vector<int> pc_refs;
        for (const auto& p : sc->params)
            if (p.tag == StepTag::List)
                for (const auto& v : p.list)
                    if (v.tag == StepTag::Ref) pc_refs.push_back(v.ref_id);
        std::vector<int> mine;
        for (int pid : pc_refs) {
            const StepEntity* pe = get(pid);
            if (!pe) continue;
            const StepSubEntity* pc = pe->find("PCURVE");
            if (!pc) continue;
            std::vector<int> refs;
            for (const auto& p : pc->params) if (p.tag == StepTag::Ref) refs.push_back(p.ref_id);
            if (refs.size() < 2 || refs[0] != surface_ref) continue;
            mine.push_back(refs[1]);   // DEFINITIONAL_REPRESENTATION
        }
        if (mine.empty()) return {};
        int pick = (is_seam && mine.size() > 1 && !forward_use) ? 1 : 0;
        const StepEntity* dr = get(mine[pick]);
        if (!dr) return {};
        const StepSubEntity* drs = dr->find("DEFINITIONAL_REPRESENTATION");
        if (!drs) return {};
        int c2_ref = -1;
        for (const auto& p : drs->params) {
            if (p.tag == StepTag::List)
                for (const auto& v : p.list) if (v.tag == StepTag::Ref) { c2_ref = v.ref_id; break; }
            if (c2_ref >= 0) break;
        }
        if (c2_ref < 0) return {};
        NurbsCurve c2 = get_nurbs_curve(c2_ref);   // 2D CPs read as (u,v,0)
        if (!c2.is_valid() || c2.cv_count() < c2.order()) return {};
        auto kts = c2.get_nurbsknots();
        if (kts.empty()) return {};
        int deg = c2.order() - 1;
        double tmin = kts[deg > 0 ? deg - 1 : 0];
        double tmax = kts[kts.size() - (deg > 0 ? deg : 1)];
        std::vector<Point> out;
        for (int i = 0; i < n; ++i) {
            double t = (n > 1) ? tmin + (tmax - tmin) * i / (n - 1) : tmin;
            Point p = c2.point_at(t);
            out.emplace_back(p[0], p[1], 0.0);
        }
        return out;
    }
};

// ============================================================
// BRep assembly from STEP (reads MANIFOLD_SOLID_BREP)
// ============================================================

static NurbsCurve polyline_nurbs(const std::vector<Point>& pts, int dim) {
    int n = (int)pts.size();
    if (n < 2) return NurbsCurve(dim, false, 2, 0);
    NurbsCurve nc(dim, false, 2, n);
    for (int i = 0; i < n; i++) nc.m_nurbsknot[i] = (double)i;
    double* cv = nc.cv_array();
    for (int i = 0; i < n; i++) for (int d = 0; d < dim; d++) cv[i*dim+d] = pts[i][d];
    return nc;
}


struct BRepBuilder {
    StepReader& r;
    const StepFile& sf;
    BRep brep;
    std::unordered_map<int, int> vmap; // STEP vertex_point id → topology vertex index
    std::unordered_map<int, int> emap; // STEP edge_curve id → BRepEdge index
    std::vector<BRepRef> face_refs;    // faces of the shell being built, with same_sense orientation

    explicit BRepBuilder(StepReader& reader, const StepFile& s) : r(reader), sf(s) {}

    // One edge use in loop-traversal order: the pcurve runs the way the loop walks it and is
    // flipped into the edge's own direction when stored (OCCT SameParameter convention).
    struct PendingEdge { int edge; bool reversed; NurbsCurve c2d; };

    // Existing vertex within `tol` of `q`, else a new one (poles/apices of degenerated edges).
    int vertex_at(const Point& q, double tol) {
        for (int i = 0; i < (int)brep.m_vertices.size(); ++i)
            if (brep.m_vertices[i].point.distance(q) <= tol) return i;
        return brep.add_vertex(q);
    }

    // BRep_CurveOnSurface / BRep_CurveOnClosedSurface: the second use of an edge on the SAME
    // surface is a seam; the FORWARD use keeps curve_2d_index, the REVERSED one curve_2d_index_2.
    void attach_pcurve(int edge, int si, int c2, bool reversed_use) {
        for (auto& pc : brep.m_edges[edge].pcurves)
            if (pc.surface_index == si) {
                if (reversed_use) pc.curve_2d_index_2 = c2;
                else { pc.curve_2d_index_2 = pc.curve_2d_index; pc.curve_2d_index = c2; }
                return;
            }
        brep.add_pcurve(edge, si, c2);
    }

    // OCCT face: surface + oriented wires (outer first) + same_sense orientation in the shell.
    int finish_face(int si, bool reversed_face, const std::vector<std::vector<PendingEdge>>& loops) {
        std::vector<BRepRef> wires;
        for (const auto& lp : loops) {
            std::vector<BRepRef> refs;
            for (const auto& pe : lp) {
                NurbsCurve c = pe.c2d;
                if (pe.reversed) c.reverse();
                attach_pcurve(pe.edge, si, brep.add_curve_2d(c), pe.reversed);
                refs.push_back({pe.edge, pe.reversed ? BRepOrientation::Reversed : BRepOrientation::Forward});
            }
            if (!refs.empty()) wires.push_back({brep.add_wire(refs), BRepOrientation::Forward});
        }
        int fi = brep.add_face(si, wires);
        face_refs.push_back({fi, reversed_face ? BRepOrientation::Reversed : BRepOrientation::Forward});
        return fi;
    }

    int get_vertex(int vp_id) {
        auto it = vmap.find(vp_id);
        if (it != vmap.end()) return it->second;
        Point pt(0,0,0);
        auto eit = sf.entities.find(vp_id);
        if (eit != sf.entities.end()) {
            const StepSubEntity* sub = eit->second.find("VERTEX_POINT");
            if (sub) {
                for (const auto& p : sub->params) if (p.tag == StepTag::Ref) { pt = r.get_point(p.ref_id); break; }
            }
        }
        return vmap[vp_id] = brep.add_vertex(pt);
    }

    // Get or create BRepEdge from EDGE_CURVE entity id
    int get_edge(int ec_id) {
        auto it = emap.find(ec_id);
        if (it != emap.end()) return it->second;

        const StepEntity* ec_ent = nullptr;
        auto eit = sf.entities.find(ec_id);
        if (eit == sf.entities.end()) return -1;
        ec_ent = &eit->second;
        const StepSubEntity* ec = ec_ent->find("EDGE_CURVE");
        if (!ec) return -1;

        std::vector<int> refs;
        for (const auto& p : ec->params) if (p.tag == StepTag::Ref) refs.push_back(p.ref_id);
        if (refs.size() < 3) return -1;

        int sv = get_vertex(refs[0]);
        int ev = get_vertex(refs[1]);
        int curve_id = r.basis_curve_of(refs[2]);
        Point vs = brep.m_vertices[sv].point;
        Point ve = brep.m_vertices[ev].point;

        // Build exact NurbsCurve for 3D edge curve
        NurbsCurve crv3d;
        bool got = false;
        const auto& curve_ent = sf.entities.find(curve_id);
        if (curve_ent != sf.entities.end()) {
            if (curve_ent->second.has("B_SPLINE_CURVE_WITH_KNOTS")) {
                crv3d = r.get_nurbs_curve(curve_id);
                got = crv3d.is_valid();
            } else if (curve_ent->second.has("CIRCLE")) {
                // Build exact rational NURBS arc
                const StepSubEntity* circ = curve_ent->second.find("CIRCLE");
                int ax_ref = -1; double rad = 0;
                for (const auto& p : circ->params) {
                    if (p.tag == StepTag::Ref && ax_ref < 0) ax_ref = p.ref_id;
                    else if (p.tag == StepTag::Num) rad = p.num;
                }
                if (ax_ref >= 0 && rad > 0) {
                    Axis2 a = r.get_axis2(ax_ref);
                    if (a.ok) {
                        auto angle_of = [&](const Point& pt) {
                            double dx=pt[0]-a.origin[0],dy=pt[1]-a.origin[1],dz=pt[2]-a.origin[2];
                            return std::atan2(dx*a.ay[0]+dy*a.ay[1]+dz*a.ay[2], dx*a.ax[0]+dy*a.ax[1]+dz*a.ax[2]);
                        };
                        const double pi = 3.14159265358979323846;
                        double sa = angle_of(vs), ea = angle_of(ve);
                        double dx=ve[0]-vs[0],dy=ve[1]-vs[1],dz=ve[2]-vs[2];
                        bool closed = (dx*dx+dy*dy+dz*dz < 1e-20);
                        if (closed) ea = sa + 2.0*pi;
                        else if (ea <= sa) ea += 2.0*pi;
                        double span = ea - sa;
                        int ns = std::max(1,(int)std::ceil(std::abs(span)/(pi*0.5)));
                        int n_cp = 2*ns+1;
                        double da = span/(2.0*ns), wm = std::cos(da);
                        crv3d = NurbsCurve(3, true, 3, n_cp);
                        { int k=0; crv3d.m_nurbsknot[k++]=sa; crv3d.m_nurbsknot[k++]=sa;
                          for(int s=1;s<ns;s++){double ak=sa+s*span/ns; crv3d.m_nurbsknot[k++]=ak; crv3d.m_nurbsknot[k++]=ak;}
                          crv3d.m_nurbsknot[k++]=ea; crv3d.m_nurbsknot[k++]=ea; }
                        double* cv = crv3d.cv_array();
                        for(int i=0;i<n_cp;i++){
                            bool mid=(i%2==1); double ang,w,r2;
                            if(!mid){ang=sa+(i/2)*span/ns;w=1.0;r2=rad;}
                            else{ang=sa+((i/2)+0.5)*span/ns;w=wm;r2=rad/wm;}
                            double ca=std::cos(ang),sa2=std::sin(ang);
                            double px=a.origin[0]+r2*(ca*a.ax[0]+sa2*a.ay[0]);
                            double py=a.origin[1]+r2*(ca*a.ax[1]+sa2*a.ay[1]);
                            double pz=a.origin[2]+r2*(ca*a.ax[2]+sa2*a.ay[2]);
                            cv[i*4+0]=w*px;cv[i*4+1]=w*py;cv[i*4+2]=w*pz;cv[i*4+3]=w;
                        }
                        got = true;
                    }
                }
            }
        }
        if (!got) {
            auto samples = r.sample_curve(curve_id, vs, ve, 16);
            if (samples.size() < 2) samples = {vs, ve};
            crv3d = polyline_nurbs(samples, 3);
        }

        return emap[ec_id] = brep.add_edge(brep.add_curve_3d(crv3d), sv, ev);
    }

    // Assemble a face on an analytic quadric surface. Returns false to fall back to the
    // legacy projection path (which then builds whatever it can).
    bool add_face_analytic(const std::vector<int>& bound_refs, int surface_ref,
                           bool same_sense, const StepReader::AnFace& an) {
        const double pi_2 = 1.5707963267948966;
        const double period = 4.0;                  // full circle in quarter-arc chart units
        struct AEdge { int edge_idx; bool reversed; bool seam; std::vector<Point> st; };
        struct ALoop { bool is_outer; bool projected = false; std::vector<AEdge> edges; };
        std::vector<ALoop> loops;

        for (int bid : bound_refs) {
            const auto& bent = sf.entities.find(bid);
            if (bent == sf.entities.end()) continue;
            bool is_outer = bent->second.has("FACE_OUTER_BOUND");
            if (!is_outer && !bent->second.has("FACE_BOUND")) continue;
            const StepSubEntity* bsub = bent->second.find(is_outer ? "FACE_OUTER_BOUND" : "FACE_BOUND");
            if (!bsub) continue;
            int loop_ref = -1; bool bound_orient = true;
            for (const auto& p : bsub->params) {
                if (p.tag == StepTag::Ref && loop_ref < 0) loop_ref = p.ref_id;
                else if (p.tag == StepTag::Enum) bound_orient = (p.str == "T");
            }
            if (loop_ref < 0) continue;
            const auto& lent = sf.entities.find(loop_ref);
            if (lent == sf.entities.end()) continue;
            const StepSubEntity* loop = lent->second.find("EDGE_LOOP");
            if (!loop) continue;
            std::vector<int> oe_refs;
            for (const auto& p : loop->params)
                if (p.tag == StepTag::List)
                    for (const auto& v : p.list) if (v.tag == StepTag::Ref) oe_refs.push_back(v.ref_id);

            ALoop lp; lp.is_outer = is_outer;
            for (int oe_id : oe_refs) {
                const auto& oent = sf.entities.find(oe_id);
                if (oent == sf.entities.end()) continue;
                const StepSubEntity* oe = oent->second.find("ORIENTED_EDGE");
                if (!oe) continue;
                int ec_ref = -1; bool oe_orient = true;
                for (const auto& p : oe->params) {
                    if (p.tag == StepTag::Ref) ec_ref = p.ref_id;
                    else if (p.tag == StepTag::Enum) oe_orient = (p.str == "T");
                }
                if (ec_ref < 0) continue;
                int edge_idx = get_edge(ec_ref);
                if (edge_idx < 0) continue;
                bool rev = !oe_orient;
                if (!bound_orient) rev = !rev;

                int geom_id = -1;
                {
                    const auto& ecent = sf.entities.find(ec_ref);
                    if (ecent != sf.entities.end()) {
                        const StepSubEntity* ec2 = ecent->second.find("EDGE_CURVE");
                        if (ec2) {
                            std::vector<int> rfs;
                            for (const auto& p : ec2->params) if (p.tag == StepTag::Ref) rfs.push_back(p.ref_id);
                            if (rfs.size() >= 3) geom_id = rfs[2];
                        }
                    }
                }
                bool seam = false;
                if (geom_id >= 0) {
                    const auto& geit = sf.entities.find(geom_id);
                    seam = geit != sf.entities.end() && geit->second.has("SEAM_CURVE");
                }
                std::vector<Point> st;
                if (geom_id >= 0)
                    st = r.pcurve_st_samples(geom_id, surface_ref, oe_orient, 48);
                if (st.size() < 2) {
                    lp.projected = true;
                    // Projection fallback: 3D samples -> canonical (s,t), branch-unwrapped
                    // along the loop traversal (seeded from the previous edge's endpoint).
                    const BRepEdge& be = brep.m_edges[edge_idx];
                    Point vs = brep.m_vertices[be.start_vertex].point;
                    Point ve = brep.m_vertices[be.end_vertex].point;
                    auto samples = r.sample_curve(geom_id, vs, ve, 48);
                    if (samples.size() < 2) return false;
                    st.clear();
                    double ps = 0, pt = 0; bool have_prev = false;
                    if (!lp.edges.empty()) {
                        const AEdge& pe = lp.edges.back();
                        if (!pe.st.empty()) {
                            const Point& q = pe.reversed ? pe.st.front() : pe.st.back();
                            ps = q[0]; pt = q[1]; have_prev = true;
                        }
                    }
                    // traversal order for seeding: if this use is reversed, unwrap from the
                    // END of the sample list backwards, then restore curve order.
                    std::vector<Point> ordered = samples;
                    if (rev) std::reverse(ordered.begin(), ordered.end());
                    // The FIRST sample of the FIRST edge of a loop has no branch reference, and
                    // when it sits on a pole/apex its angle is atan2 of two coordinates that are
                    // pure round-off (OCCT writes a cone apex as (-4.4e-16, 1.1e-31, 4)), i.e.
                    // an arbitrary value that then anchors the whole loop. The chart window is
                    // sized from these angles: occt_prim_cone came out SIX quarter-spans wide
                    // (540 degrees) instead of four, and the self-overlapping cone integrated to
                    // 19.5403488074 against a truth of 16.7551608191 -- solid, closed-looking,
                    // 16.6% wrong. Seed the branch from the first RADIALLY VALID sample instead;
                    // a no-op when sample 0 is already valid (the shift rounds to zero).
                    if (!have_prev)
                        for (size_t k = 0; k < ordered.size(); ++k) {
                            double s2, t2; bool ok2;
                            r.an_st_of(an, ordered[k], s2, t2, ok2);
                            if (ok2) { ps = s2; pt = t2; have_prev = true; break; }
                        }
                    std::vector<Point> st_ord;
                    for (size_t k = 0; k < ordered.size(); ++k) {
                        double s, t; bool ok;
                        r.an_st_of(an, ordered[k], s, t, ok);
                        if (!ok && (k > 0 || have_prev)) s = k > 0 ? st_ord.back()[0] : ps;
                        double rs = k > 0 ? st_ord.back()[0] : (have_prev ? ps : s);
                        s -= 2*pi_2*2 * std::round((s - rs) / (2*pi_2*2));
                        if (an.kind == 5) {
                            double rt = k > 0 ? st_ord.back()[1] : (have_prev ? pt : t);
                            t -= 2*pi_2*2 * std::round((t - rt) / (2*pi_2*2));
                        }
                        st_ord.emplace_back(s, t, 0.0);
                    }
                    if (rev) std::reverse(st_ord.begin(), st_ord.end());
                    st = std::move(st_ord);
                }
                lp.edges.push_back({edge_idx, rev, seam, std::move(st)});
            }
            if (!lp.edges.empty()) loops.push_back(std::move(lp));
        // FACE_BOUND vs FACE_OUTER_BOUND: BOTH are legal STEP spellings of an outer
        // boundary. Rhino writes FACE_OUTER_BOUND; OCCT and FreeCAD write FACE_BOUND. Trusting
        // the entity NAME meant every OCCT/FreeCAD-authored file loaded with its outer boundary
        // treated as a HOLE -- the complement of the intended region. Measured: chair0 vol
        // 19.2149 instead of 80.3011; an OCCT cylinder reading 9.4269 = 28.2743/3, which is
        // where the "1/3 volume" cluster came from (NOT a flux-formula factor).
        // Decide GEOMETRICALLY, and ONLY when the file gave no explicit outer bound, so files
        // that do use FACE_OUTER_BOUND stay bit-for-bit identical: the outer loop is the one
        // with the largest UV extent (a hole lies strictly inside it). A face with several
        // holes emits several FACE_BOUNDs, so picking the LARGEST -- rather than flipping them
        // all to outer -- is what makes multi-hole faces correct.
        {
            bool any_outer = false;
            for (const auto& l : loops) if (l.is_outer) any_outer = true;
            if (!any_outer && !loops.empty()) {
                size_t best = 0; double best_a = -1.0;
                for (size_t i = 0; i < loops.size(); ++i) {
                    double mnu=1e300, mnv=1e300, mxu=-1e300, mxv=-1e300;
                    for (const auto& e : loops[i].edges)
                        for (const auto& q : e.st) {
                            mnu = std::min(mnu, q[0]); mxu = std::max(mxu, q[0]);
                            mnv = std::min(mnv, q[1]); mxv = std::max(mxv, q[1]);
                        }
                    double a = (mxu > mnu && mxv > mnv) ? (mxu-mnu)*(mxv-mnv) : 0.0;
                    if (a > best_a) { best_a = a; best = i; }
                }
                loops[best].is_outer = true;
            }
        }
        }
        if (loops.empty()) return false;

        // Window in canonical params
        double smin = 1e300, smax = -1e300, tmin = 1e300, tmax = -1e300;
        for (const auto& lp : loops)
            for (const auto& ae : lp.edges)
                for (const auto& p : ae.st) {
                    smin = std::min(smin, p[0]); smax = std::max(smax, p[0]);
                    tmin = std::min(tmin, p[1]); tmax = std::max(tmax, p[1]);
                }
        if (smin > smax) return false;
        int su0 = (int)std::floor(smin/pi_2 + 1e-9);
        int su1 = (int)std::ceil (smax/pi_2 - 1e-9);
        int nsu = std::max(1, su1 - su0);
        if (nsu > 16) return false;
        int sv0 = 0, nsv = 0;
        double t0 = tmin, t1 = tmax;
        if (an.kind == 4 || an.kind == 5) {
            sv0 = (int)std::floor(tmin/pi_2 + 1e-9);
            int sv1 = (int)std::ceil (tmax/pi_2 - 1e-9);
            if (an.kind == 4) { sv0 = std::max(sv0, -1); sv1 = std::min(sv1, 1); }
            nsv = std::max(1, sv1 - sv0);
            if (nsv > 16) return false;
        } else {
            if (t1 - t0 < 1e-12) return false;
        }

        NurbsSurface srf = r.build_analytic_nurbs(an, su0, nsu, t0, t1, sv0, nsv);
        if (!srf.is_valid()) return false;
        double scale3 = an.R + std::abs(an.r2) + 1.0;

        // Map every (s,t) into the chart
        auto to_chart = [&](const Point& p) {
            double u = StepReader::chart_u_of_angle(p[0], su0);
            double v = (an.kind == 4 || an.kind == 5)
                       ? StepReader::chart_u_of_angle(p[1], sv0)
                       : p[1];
            return Point(u, v, 0.0);
        };

        int srf_idx = brep.add_surface(srf);
        std::stable_partition(loops.begin(), loops.end(), [](const ALoop& l) { return l.is_outer; });
        std::vector<std::vector<PendingEdge>> pending;

        for (auto& lp : loops) {
            std::vector<PendingEdge> pl;
            // Traversal-order uv chains
            std::vector<std::vector<Point>> chains(lp.edges.size());
            for (size_t k = 0; k < lp.edges.size(); ++k) {
                std::vector<Point> uv;
                uv.reserve(lp.edges[k].st.size());
                for (const auto& p : lp.edges[k].st) uv.push_back(to_chart(p));
                if (lp.edges[k].reversed) std::reverse(uv.begin(), uv.end());
                chains[k] = std::move(uv);
            }
            // Close period jumps between consecutive edges (shift whole subsequent chains).
            // ONLY for projection-fallback loops: file pcurves are branch-consistent by
            // construction, and a seam pair legitimately sits one period apart -- shifting
            // it would collapse the seam.
            for (size_t k = 1; lp.projected && k < chains.size(); ++k) {
                if (chains[k].empty() || chains[k-1].empty()) continue;
                double du = chains[k-1].back()[0] - chains[k].front()[0];
                int n = (int)std::round(du / period);
                if (n != 0) for (auto& p : chains[k]) p[0] += n * period;
                if (an.kind == 5) {
                    double dv = chains[k-1].back()[1] - chains[k].front()[1];
                    int m = (int)std::round(dv / period);
                    if (m != 0) for (auto& p : chains[k]) p[1] += m * period;
                }
            }
            for (size_t k = 0; k < lp.edges.size(); ++k) {
                const AEdge& ae = lp.edges[k];
                if (chains[k].size() < 2) continue;
                pl.push_back({ae.edge_idx, ae.reversed, polyline_nurbs(chains[k], 2)});
                // Gap to the next edge start: a jump along a degenerate iso (pole/apex)
                // becomes a synthesized degenerated edge (the native create_sphere/cone shape).
                const std::vector<Point>& nxt = chains[(k + 1) % chains.size()];
                if (nxt.empty()) continue;
                const Point& a2 = chains[k].back();
                const Point& b2 = nxt.front();
                double gap = std::abs(a2[0]-b2[0]) + std::abs(a2[1]-b2[1]);
                if (gap > 1e-7) {
                    auto ang_of = [&](double u) { return (su0 + u) * pi_2; };
                    auto angv_of = [&](double v) {
                        return (an.kind == 4 || an.kind == 5) ? (sv0 + v) * pi_2 : v; };
                    Point p3a = r.an_eval(an, ang_of(a2[0]), angv_of(a2[1]));
                    Point p3b = r.an_eval(an, ang_of(b2[0]), angv_of(b2[1]));
                    if (p3a.distance(p3b) < scale3 * 1e-6) {
                        int vd = vertex_at(p3a, scale3 * 1e-6);
                        pl.push_back({brep.add_edge(-1, vd, vd), false, polyline_nurbs({a2, b2}, 2)});
                    }
                }
            }
            pending.push_back(pl);
        }
        finish_face(srf_idx, !same_sense, pending);
        return true;
    }

    // VERTEX_LOOP bound: the legal STEP spelling of "this face has NO edge boundary -- it is
    // the whole surface". A full sphere written by OCCT/FreeCAD is exactly this: one
    // ADVANCED_FACE, one FACE_BOUND, one VERTEX_LOOP holding a single VERTEX_POINT at a pole.
    // Skipping it (the reader only ever looked for EDGE_LOOP) imported the sphere as a
    // 1-face/0-edge/0-volume husk -- and because an empty operand makes a boolean return A
    // unchanged, `box CUT sphere` then reported closed=1 solids=1 vol=64 (the untouched box):
    // a SILENT FALSE PASS, not a visible failure.
    //
    // The face's trim is the FULL parameter domain. Which boundary curves that domain needs is
    // read off the surface itself, so the same code serves the analytic sphere and the closed
    // B-spline sphere our OWN writer emits with VERTEX_LOOPs (see emit_brep_shells):
    //   closed in u + degenerate at both v ends  -> sphere: seam meridian (twice) + 2 poles
    //   closed in u + closed in v                -> torus: u-seam (twice) + v-seam (twice)
    // Poles get REAL degenerate edges appearing ONCE in the wire while the seam edge appears
    // TWICE -- the OCCT convention create_sphere/create_cone already mint (SESSION_NO_POLE_EDGE
    // opts out there), so an imported primitive has the same topology as a native one.
    bool add_face_vertex_loop(const std::vector<int>& vl_vertex_ids, int surface_ref,
                              bool same_sense) {
        NurbsSurface srf;
        StepReader::AnFace an = r.get_analytic_srf(surface_ref);
        if (an.kind == 4)        srf = r.build_analytic_nurbs(an, 0, 4, 0, 0, -1, 2);
        else if (an.kind == 5)   srf = r.build_analytic_nurbs(an, 0, 4, 0, 0,  0, 4);
        else if (an.kind == 0)   srf = r.get_nurbs_surface(surface_ref);
        if (!srf.is_valid()) return false;

        auto du = srf.domain(0), dv = srf.domain(1);
        const int NS = 17;
        auto at = [&](int i, int j) {
            return srf.point_at(du.first + (du.second - du.first) * i / (NS - 1),
                                dv.first + (dv.second - dv.first) * j / (NS - 1));
        };
        double scale = 0.0;
        for (int i = 0; i < NS; ++i)
            for (int j = 0; j < NS; ++j) scale = std::max(scale, at(i, j).distance(at(0, 0)));
        if (!(scale > 0)) return false;
        double tol = scale * 1e-7;
        auto all_same = [&](bool along_u, int fixed) {
            Point p0 = along_u ? at(0, fixed) : at(fixed, 0);
            for (int k = 1; k < NS; ++k)
                if ((along_u ? at(k, fixed) : at(fixed, k)).distance(p0) > tol) return false;
            return true;
        };
        bool closed_u = true, closed_v = true;
        for (int k = 0; k < NS; ++k) {
            if (at(0, k).distance(at(NS - 1, k)) > tol) closed_u = false;
            if (at(k, 0).distance(at(k, NS - 1)) > tol) closed_v = false;
        }
        bool degen_v0 = all_same(true, 0), degen_v1 = all_same(true, NS - 1);
        if (!closed_u) return false;                       // an unbounded/open chart cannot be
        if (!((degen_v0 && degen_v1) || closed_v)) return false;  // bounded by a vertex alone

        // Reuse the STEP VERTEX_POINTs the file gave us where they coincide, so shared vertex
        // identity survives; mint one otherwise. Matched by POSITION first, so a VERTEX_LOOP
        // vertex that sits on neither boundary never becomes an orphan topology vertex.
        auto step_point_of = [&](int vp_id) {
            const auto& eit = sf.entities.find(vp_id);
            if (eit != sf.entities.end()) {
                const StepSubEntity* sub = eit->second.find("VERTEX_POINT");
                if (sub)
                    for (const auto& p : sub->params)
                        if (p.tag == StepTag::Ref) return r.get_point(p.ref_id);
            }
            return Point(1e300, 1e300, 1e300);
        };
        auto topo_vertex_at = [&](const Point& q) {
            for (int vid : vl_vertex_ids)
                if (step_point_of(vid).distance(q) <= tol) return get_vertex(vid);
            return brep.add_vertex(q);
        };
        auto uv_line = [&](double u0, double v0, double u1, double v1) {
            return NurbsCurve::create(false, 1, {Point(u0, v0, 0), Point(u1, v1, 0)});
        };

        int si = brep.add_surface(srf);
        std::vector<PendingEdge> wire;

        if (degen_v0 && degen_v1) {                                   // sphere-like
            int v_lo = topo_vertex_at(at(0, 0)), v_hi = topo_vertex_at(at(0, NS - 1));
            NurbsCurve seam = srf.iso_curve(1, du.first);             // v-varying meridian
            if (!seam.is_valid()) return false;
            int ei_seam = brep.add_edge(brep.add_curve_3d(seam), v_lo, v_hi);
            int ei_lo = brep.add_edge(-1, v_lo, v_lo);
            int ei_hi = brep.add_edge(-1, v_hi, v_hi);
            wire.push_back({ei_lo, false, uv_line(du.first, dv.first, du.second, dv.first)});
            wire.push_back({ei_seam, false, uv_line(du.second, dv.first, du.second, dv.second)});
            wire.push_back({ei_hi, false, uv_line(du.second, dv.second, du.first, dv.second)});
            wire.push_back({ei_seam, true, uv_line(du.first, dv.second, du.first, dv.first)});
        } else {                                                      // torus-like
            int v0 = topo_vertex_at(at(0, 0));
            NurbsCurve c_u = srf.iso_curve(1, du.first);              // u-seam (varies v)
            NurbsCurve c_v = srf.iso_curve(0, dv.first);              // v-seam (varies u)
            if (!c_u.is_valid() || !c_v.is_valid()) return false;
            int ei_u = brep.add_edge(brep.add_curve_3d(c_u), v0, v0);
            int ei_v = brep.add_edge(brep.add_curve_3d(c_v), v0, v0);
            wire.push_back({ei_v, false, uv_line(du.first, dv.first, du.second, dv.first)});
            wire.push_back({ei_u, false, uv_line(du.second, dv.first, du.second, dv.second)});
            wire.push_back({ei_v, true, uv_line(du.second, dv.second, du.first, dv.second)});
            wire.push_back({ei_u, true, uv_line(du.first, dv.second, du.first, dv.first)});
        }
        finish_face(si, !same_sense, {wire});
        return true;
    }

    void add_face(int face_id) {
        const auto& feit = sf.entities.find(face_id);
        if (feit == sf.entities.end()) return;
        const StepSubEntity* face = feit->second.find("ADVANCED_FACE");
        if (!face) return;

        std::vector<int> bound_refs;
        int surface_ref = -1;
        bool same_sense = true;
        for (const auto& p : face->params) {
            if (p.tag == StepTag::List) {
                for (const auto& v : p.list) if (v.tag == StepTag::Ref) bound_refs.push_back(v.ref_id);
            } else if (p.tag == StepTag::Ref) {
                surface_ref = p.ref_id;
            } else if (p.tag == StepTag::Enum) {
                same_sense = (p.str == "T");
            }
        }

        // VERTEX_LOOP-only face = full parameter domain (see add_face_vertex_loop). Only when
        // the file gives NO edge loop at all: a face that mixes an EDGE_LOOP with a VERTEX_LOOP
        // keeps the edge-driven path unchanged, so no already-working file moves.
        if (surface_ref >= 0) {
            std::vector<int> vl_verts;
            bool any_edge_loop = false;
            for (int bid : bound_refs) {
                const auto& bent = sf.entities.find(bid);
                if (bent == sf.entities.end()) continue;
                const StepSubEntity* bsub = bent->second.find("FACE_OUTER_BOUND");
                if (!bsub) bsub = bent->second.find("FACE_BOUND");
                if (!bsub) continue;
                int loop_ref = -1;
                for (const auto& p : bsub->params)
                    if (p.tag == StepTag::Ref) { loop_ref = p.ref_id; break; }
                const auto& lent = sf.entities.find(loop_ref);
                if (lent == sf.entities.end()) continue;
                if (lent->second.has("EDGE_LOOP")) { any_edge_loop = true; continue; }
                const StepSubEntity* vl = lent->second.find("VERTEX_LOOP");
                if (!vl) continue;
                for (const auto& p : vl->params)
                    if (p.tag == StepTag::Ref) { vl_verts.push_back(p.ref_id); break; }
            }
            if (!any_edge_loop && !vl_verts.empty()
                && add_face_vertex_loop(vl_verts, surface_ref, same_sense))
                return;
        }

        // Analytic quadric faces (cyl/cone/sphere/torus): rebuild the exact kernel-canonical
        // rational NURBS window and bind the file's own pcurves (or projected samples) in it.
        if (surface_ref >= 0) {
            StepReader::AnFace an = r.get_analytic_srf(surface_ref);
            if (an.kind >= 2 && add_face_analytic(bound_refs, surface_ref, same_sense, an))
                return;
        }

        // Get projector for UV mapping (PLANE or CYL)
        StepReader::Proj proj;
        if (surface_ref >= 0) proj = r.get_projector(surface_ref);

        // No analytic projector (B-spline surface; chairs-class files carry no pcurves
        // at all) -> build the surface up front and project by closest point. fill_surface
        // ignores the bounds for B-splines, the only surface kind without a projector.
        NurbsSurface proj_srf;
        bool have_proj_srf = false;
        if (proj.kind == StepReader::ProjKind::None && surface_ref >= 0)
            have_proj_srf = r.fill_surface(surface_ref, 0, 1, 0, 1, proj_srf) && proj_srf.is_valid();

        // Pre-scan UV extents
        auto uv_pts_of_sample = [&](const std::vector<Point>& samples) {
            std::vector<Point> uv;
            if (proj.kind == StepReader::ProjKind::None) {
                if (!have_proj_srf || samples.empty())
                    return std::vector<Point>{{Point(0,0,0), Point(1,0,0)}};
                // Warm-start window keeps successive samples on one branch of a closed
                // surface; a window miss (distance blows past the file's incidence error)
                // falls back to a full-domain search.
                auto [du0, du1] = proj_srf.domain(0);
                auto [dv0, dv1] = proj_srf.domain(1);
                double wu = (du1 - du0) * 0.1, wv = (dv1 - dv0) * 0.1;
                double d_ref = 0, pu = 0, pv = 0;
                for (size_t k = 0; k < samples.size(); ++k) {
                    double u, v, d;
                    if (k == 0) {
                        std::tie(u, v, d) = Closest::surface_point(proj_srf, samples[k]);
                        d_ref = d;
                    } else {
                        std::tie(u, v, d) = Closest::surface_point(proj_srf, samples[k],
                            pu - wu, pu + wu, pv - wv, pv + wv);
                        if (d > 10 * d_ref + 1e-9)
                            std::tie(u, v, d) = Closest::surface_point(proj_srf, samples[k]);
                    }
                    uv.emplace_back(u, v, 0.0);
                    pu = u; pv = v;
                }
                return uv;
            }
            for (const auto& s : samples) {
                auto [u, v] = r.project(proj, s);
                uv.emplace_back(u, v, 0.0);
            }
            return uv;
        };

        struct LoopEdge { int edge_idx; bool reversed; std::vector<Point> uv;
                          NurbsCurve pc2d; bool exact = false; };
        struct Loop { bool is_outer; std::vector<LoopEdge> edges; };
        std::vector<Loop> loops;

        for (int bid : bound_refs) {
            const auto& bent = sf.entities.find(bid);
            if (bent == sf.entities.end()) continue;
            bool is_outer = bent->second.has("FACE_OUTER_BOUND");
            if (!is_outer && !bent->second.has("FACE_BOUND")) continue;

            const StepSubEntity* bsub = bent->second.find(is_outer ? "FACE_OUTER_BOUND" : "FACE_BOUND");
            if (!bsub) continue;
            int loop_ref = -1; bool bound_orient = true;
            for (const auto& p : bsub->params) {
                if (p.tag == StepTag::Ref && loop_ref < 0) loop_ref = p.ref_id;
                else if (p.tag == StepTag::Enum) bound_orient = (p.str == "T");
            }
            if (loop_ref < 0) continue;

            const auto& lent = sf.entities.find(loop_ref);
            if (lent == sf.entities.end()) continue;
            const StepSubEntity* loop = lent->second.find("EDGE_LOOP");
            if (!loop) continue;

            std::vector<int> oe_refs;
            for (const auto& p : loop->params)
                if (p.tag == StepTag::List) for (const auto& v : p.list) if (v.tag == StepTag::Ref) oe_refs.push_back(v.ref_id);

            Loop lp; lp.is_outer = is_outer;
            for (int oe_id : oe_refs) {
                const auto& oent = sf.entities.find(oe_id);
                if (oent == sf.entities.end()) continue;
                const StepSubEntity* oe = oent->second.find("ORIENTED_EDGE");
                if (!oe) continue;
                int ec_ref = -1; bool oe_orient = true;
                for (const auto& p : oe->params) {
                    if (p.tag == StepTag::Ref) ec_ref = p.ref_id;
                    else if (p.tag == StepTag::Enum) oe_orient = (p.str == "T");
                }
                if (ec_ref < 0) continue;
                int edge_idx = get_edge(ec_ref);
                if (edge_idx < 0) continue;
                bool rev = !oe_orient;
                if (!bound_orient) rev = !rev;

                // Build 3D samples for UV projection
                const BRepEdge& be = brep.m_edges[edge_idx];
                Point vs = brep.m_vertices[be.start_vertex].point;
                Point ve = brep.m_vertices[be.end_vertex].point;
                auto samples = r.sample_curve(
                    [&]() -> int {
                        const auto& ecent = sf.entities.find(ec_ref);
                        if (ecent == sf.entities.end()) return -1;
                        const StepSubEntity* ec2 = ecent->second.find("EDGE_CURVE");
                        if (!ec2) return -1;
                        std::vector<int> rfs;
                        for (const auto& p : ec2->params) if (p.tag == StepTag::Ref) rfs.push_back(p.ref_id);
                        return rfs.size() >= 3 ? rfs[2] : -1;
                    }(),
                    vs, ve, have_proj_srf ? 48 : 16);
                if (samples.empty()) samples = {vs, ve};
                auto uv = uv_pts_of_sample(samples);

                // Unwrap cylindrical seams (canonical U: 1 unit = 90deg, full circle = 4)
                if (proj.kind == StepReader::ProjKind::Cyl && uv.size() > 1) {
                    for (size_t k = 1; k < uv.size(); k++) {
                        double du = uv[k][0] - uv[k-1][0];
                        if (du >  2.0) uv[k] = Point(uv[k][0]-4.0, uv[k][1], 0);
                        else if (du < -2.0) uv[k] = Point(uv[k][0]+4.0, uv[k][1], 0);
                    }
                }
                LoopEdge le2{edge_idx, rev, std::move(uv), NurbsCurve(), false};
                // Planar faces: the projector is AFFINE, so projecting the exact 3D edge
                // curve's control points yields an EXACT rational pcurve (a sampled 48-gon
                // circle loses 0.3% cap area -- visible directly in cylinder volume).
                if (proj.kind == StepReader::ProjKind::Plane) {
                    const BRepEdge& be2 = brep.m_edges[edge_idx];
                    if (be2.curve_3d_index >= 0 && be2.curve_3d_index < (int)brep.m_curves_3d.size()) {
                        const NurbsCurve& c3 = brep.m_curves_3d[be2.curve_3d_index];
                        if (c3.is_valid() && c3.cv_count() >= 2) {
                            bool rat = c3.m_is_rat != 0;
                            NurbsCurve p2(3, rat, c3.order(), c3.cv_count());
                            p2.m_nurbsknot = c3.m_nurbsknot;
                            bool okcv = true;
                            double* ocv = p2.cv_array();
                            int ost = p2.m_cv_stride;
                            for (int ci = 0; ci < c3.cv_count() && okcv; ++ci) {
                                const double* cv = c3.m_cv.data() + ci * c3.m_cv_stride;
                                double wgt = rat ? cv[3] : 1.0;
                                if (rat && std::abs(wgt) < 1e-300) { okcv = false; break; }
                                Point e3(rat ? cv[0]/wgt : cv[0], rat ? cv[1]/wgt : cv[1],
                                         rat ? cv[2]/wgt : cv[2]);
                                auto [uu, vv] = r.project(proj, e3);
                                ocv[ci*ost+0] = uu * wgt;
                                ocv[ci*ost+1] = vv * wgt;
                                ocv[ci*ost+2] = 0.0;
                                if (rat) ocv[ci*ost+3] = wgt;
                            }
                            if (okcv && p2.is_valid()) { le2.pc2d = p2; le2.exact = true; }
                        }
                    }
                } else if (have_proj_srf && proj_srf.degree(0) == 1 && proj_srf.degree(1) == 1) {
                    // A bilinear patch is affine too: project the exact 3D curve CVs.
                    Point p00 = proj_srf.get_cv(0, 0), p10 = proj_srf.get_cv(1, 0), p01 = proj_srf.get_cv(0, 1);
                    double eu[3] = {p10[0]-p00[0], p10[1]-p00[1], p10[2]-p00[2]};
                    double ev[3] = {p01[0]-p00[0], p01[1]-p00[1], p01[2]-p00[2]};
                    double eu2 = eu[0]*eu[0]+eu[1]*eu[1]+eu[2]*eu[2], ev2 = ev[0]*ev[0]+ev[1]*ev[1]+ev[2]*ev[2];
                    const BRepEdge& be2 = brep.m_edges[edge_idx];
                    if (eu2 > 1e-28 && ev2 > 1e-28 && be2.curve_3d_index >= 0) {
                        const NurbsCurve& c3 = brep.m_curves_3d[be2.curve_3d_index];
                        if (c3.is_valid() && c3.cv_count() >= 2) {
                            NurbsCurve p2(3, c3.is_rational(), c3.order(), c3.cv_count());
                            for (int k = 0; k < c3.nurbsknot_count(); ++k) p2.set_nurbsknot(k, c3.nurbsknot(k));
                            for (int ci = 0; ci < c3.cv_count(); ++ci) {
                                auto [wx, wy, wz, wgt] = c3.get_cv_4d(ci);
                                double dx = wx/wgt - p00[0], dy = wy/wgt - p00[1], dz = wz/wgt - p00[2];
                                double uu = (dx*eu[0]+dy*eu[1]+dz*eu[2]) / eu2;
                                double vv = (dx*ev[0]+dy*ev[1]+dz*ev[2]) / ev2;
                                if (c3.is_rational()) p2.set_cv_4d(ci, uu*wgt, vv*wgt, 0.0, wgt);
                                else p2.set_cv(ci, Point(uu, vv, 0));
                            }
                            if (p2.is_valid()) { le2.pc2d = p2; le2.exact = true; }
                        }
                    }
                }
                lp.edges.push_back(std::move(le2));
            }
            if (!lp.edges.empty()) loops.push_back(std::move(lp));
        // FACE_BOUND vs FACE_OUTER_BOUND: BOTH are legal STEP spellings of an outer
        // boundary. Rhino writes FACE_OUTER_BOUND; OCCT and FreeCAD write FACE_BOUND. Trusting
        // the entity NAME meant every OCCT/FreeCAD-authored file loaded with its outer boundary
        // treated as a HOLE -- the complement of the intended region. Measured: chair0 vol
        // 19.2149 instead of 80.3011; an OCCT cylinder reading 9.4269 = 28.2743/3, which is
        // where the "1/3 volume" cluster came from (NOT a flux-formula factor).
        // Decide GEOMETRICALLY, and ONLY when the file gave no explicit outer bound, so files
        // that do use FACE_OUTER_BOUND stay bit-for-bit identical: the outer loop is the one
        // with the largest UV extent (a hole lies strictly inside it). A face with several
        // holes emits several FACE_BOUNDs, so picking the LARGEST -- rather than flipping them
        // all to outer -- is what makes multi-hole faces correct.
        {
            bool any_outer = false;
            for (const auto& l : loops) if (l.is_outer) any_outer = true;
            if (!any_outer && !loops.empty()) {
                size_t best = 0; double best_a = -1.0;
                for (size_t i = 0; i < loops.size(); ++i) {
                    double mnu=1e300, mnv=1e300, mxu=-1e300, mxv=-1e300;
                    for (const auto& e : loops[i].edges)
                        for (const auto& q : e.uv) {
                            mnu = std::min(mnu, q[0]); mxu = std::max(mxu, q[0]);
                            mnv = std::min(mnv, q[1]); mxv = std::max(mxv, q[1]);
                        }
                    double a = (mxu > mnu && mxv > mnv) ? (mxu-mnu)*(mxv-mnv) : 0.0;
                    if (a > best_a) { best_a = a; best = i; }
                }
                loops[best].is_outer = true;
            }
        }
        }

        // Chain-align UV values on periodic surfaces (analytic cylinder chart: 1 unit = 90deg,
        // full circle = 4 units; closed B-spline surfaces: the domain span in that direction).
        // Problem: per-edge seam unwrap makes each edge continuous, but a seam edge projects
        // onto the same branch for BOTH of its uses, and opposite-winding circles land in
        // disjoint windows. Fix: chain each edge so its loop-traversal start aligns with the
        // previous end, one period shift per chain. Inner loops (holes) are also normalized to
        // the outer loop's U center.
        double tau_u = 0.0, tau_v = 0.0;
        if (proj.kind == StepReader::ProjKind::Cyl) tau_u = 4.0;
        else if (have_proj_srf) {
            auto [du0, du1] = proj_srf.domain(0);
            auto [dv0, dv1] = proj_srf.domain(1);
            double scale = proj_srf.point_at(du0, dv0).distance(proj_srf.point_at(du1, dv1)) + 1e-9;
            bool closed_u = true, closed_v = true;
            for (int k = 0; k <= 4; ++k) {
                double fu = du0 + (du1 - du0) * k / 4.0, fv = dv0 + (dv1 - dv0) * k / 4.0;
                if (proj_srf.point_at(du0, fv).distance(proj_srf.point_at(du1, fv)) > scale * 1e-6) closed_u = false;
                if (proj_srf.point_at(fu, dv0).distance(proj_srf.point_at(fu, dv1)) > scale * 1e-6) closed_v = false;
            }
            if (closed_u) tau_u = du1 - du0;
            if (closed_v) tau_v = dv1 - dv0;
        }
        if (tau_u > 0.0 || tau_v > 0.0) {
            const double tau = tau_u;
            for (auto& lp : loops) {
                Point prev_end(0, 0, 0);
                bool have_prev = false;
                for (auto& le : lp.edges) {
                    if (le.uv.empty()) continue;
                    int start_idx = le.reversed ? (int)le.uv.size()-1 : 0;
                    int end_idx   = le.reversed ? 0 : (int)le.uv.size()-1;
                    if (have_prev) {
                        const Point& st = le.uv[start_idx];
                        int n = tau_u > 0.0 ? (int)std::round((prev_end[0] - st[0]) / tau_u) : 0;
                        int m = tau_v > 0.0 ? (int)std::round((prev_end[1] - st[1]) / tau_v) : 0;
                        if (n != 0 || m != 0)
                            for (auto& p : le.uv) p = Point(p[0] + n * tau_u, p[1] + m * tau_v, 0.0);
                    }
                    prev_end = le.uv[end_idx];
                    have_prev = true;
                }
            }

            // Compute outer loop U center for inner-loop normalization
            double outer_ucenter = 0;
            bool have_outer = false;
            for (const auto& lp : loops) {
                if (!lp.is_outer) continue;
                double sum = 0; int cnt = 0;
                for (const auto& le : lp.edges)
                    for (const auto& p : le.uv) { sum += p[0]; cnt++; }
                if (cnt > 0) { outer_ucenter = sum / cnt; have_outer = true; }
                break;
            }
            // Shift inner loops to align with outer loop's U window
            if (have_outer && tau > 0.0) {
                for (auto& lp : loops) {
                    if (lp.is_outer) continue;
                    double sum = 0; int cnt = 0;
                    for (const auto& le : lp.edges)
                        for (const auto& p : le.uv) { sum += p[0]; cnt++; }
                    if (cnt == 0) continue;
                    int n = (int)std::round((outer_ucenter - sum / cnt) / tau);
                    if (n != 0)
                        for (auto& le : lp.edges)
                            for (auto& p : le.uv) p[0] += n * tau;
                }
            }
        }

        // Compute UV bounds for surface sizing
        double umin=1e30, umax=-1e30, vmin=1e30, vmax=-1e30;
        for (const auto& lp : loops)
            for (const auto& le : lp.edges)
                for (const auto& p : le.uv) {
                    if (p[0]<umin) umin=p[0];
                    if (p[0]>umax) umax=p[0];
                    if (p[1]<vmin) vmin=p[1];
                    if (p[1]>vmax) vmax=p[1];
                }
        if (umin > umax) { umin=-1; umax=1; vmin=-1; vmax=1; }

        NurbsSurface srf;
        if (have_proj_srf) srf = proj_srf;
        else if (surface_ref >= 0) r.fill_surface(surface_ref, umin, umax, vmin, vmax, srf);
        int srf_idx = brep.add_surface(srf);
        std::stable_partition(loops.begin(), loops.end(), [](const Loop& l) { return l.is_outer; });
        std::vector<std::vector<PendingEdge>> pending;
        for (const auto& lp : loops) {
            std::vector<PendingEdge> pl;
            for (const auto& le : lp.edges) {
                // le.uv / le.pc2d are sampled in CURVE order; PendingEdge wants traversal order.
                NurbsCurve crv2d;
                if (le.exact) {
                    crv2d = le.pc2d;
                    if (le.reversed) crv2d.reverse();
                } else {
                    std::vector<Point> uv = le.uv;
                    if (le.reversed) std::reverse(uv.begin(), uv.end());
                    crv2d = polyline_nurbs(uv, 2);
                }
                pl.push_back({le.edge_idx, le.reversed, crv2d});
            }
            pending.push_back(pl);
        }
        finish_face(srf_idx, !same_sense, pending);
    }

    BRep build_from_shell(int shell_id) {
        const auto& sent = sf.entities.find(shell_id);
        if (sent == sf.entities.end()) return BRep();
        if (!sent->second.has("CLOSED_SHELL") && !sent->second.has("OPEN_SHELL")) return BRep();

        const StepSubEntity* shell_sub = sent->second.find("CLOSED_SHELL");
        if (!shell_sub) shell_sub = sent->second.find("OPEN_SHELL");

        std::vector<int> step_faces;
        for (const auto& p : shell_sub->params)
            if (p.tag == StepTag::List)
                for (const auto& v : p.list) if (v.tag == StepTag::Ref) step_faces.push_back(v.ref_id);

        brep.name = "step_brep";
        for (int f : step_faces) add_face(f);
        if (!face_refs.empty()) {
            int sh = brep.add_shell(face_refs);
            if (sent->second.has("CLOSED_SHELL")) brep.add_solid({{sh, BRepOrientation::Forward}});
        }
        return std::move(brep);
    }
};

// ============================================================
// Layer 4: StepWriter
// ============================================================

class StepWriter {
public:
    int next_id = 1;
    std::vector<std::string> lines;

    int new_id() { return next_id++; }

    static std::string fmt(double v) {
        char buf[64];
        if (v == (int)v && std::abs(v) < 1e15)
            std::snprintf(buf, sizeof(buf), "%d.", (int)v);
        else
            std::snprintf(buf, sizeof(buf), "%.15g", v);
        std::string s = buf;
        // ISO 10303-21 REAL needs a decimal point in the mantissa and uppercase E:
        // %g yields "1e-06" for the clamped uncertainty of an EMPTY result, which
        // strict parsers reject ("Incorrect Syntax"), and collapses near-integers
        // (-1.9999999999999998) to bare "-2".
        size_t e = s.find('e');
        if (e != std::string::npos) {
            s[e] = 'E';
            if (s.substr(0, e).find('.') == std::string::npos) { s.insert(e, "."); }
        } else if (s.find('.') == std::string::npos) {
            s += ".";
        }
        return s;
    }

    template<typename T>
    std::string fmt_list(const std::vector<T>& items, const char* sep = ",") {
        std::string s = "(";
        for (size_t i = 0; i < items.size(); i++) {
            if (i) s += sep;
            if constexpr (std::is_same_v<T, int>) s += std::to_string(items[i]);
            else if constexpr (std::is_same_v<T, double>) s += fmt(items[i]);
            else s += items[i]; // string
        }
        return s + ")";
    }

    std::string fmt_ref_list(const std::vector<int>& ids) {
        std::string s = "(";
        for (size_t i = 0; i < ids.size(); i++) {
            if (i) s += ",";
            s += "#" + std::to_string(ids[i]);
        }
        return s + ")";
    }

    // Write a line with auto-id
    void emit_line(const std::string& body) {
        lines.push_back("#" + std::to_string(next_id - 1) + "=" + body + ";");
    }

public:
    int write_point(double x, double y, double z) {
        int id = new_id();
        lines.push_back("#" + std::to_string(id) + "=CARTESIAN_POINT('',(" + fmt(x) + "," + fmt(y) + "," + fmt(z) + "));");
        return id;
    }

    int write_point_2d(double u, double v) {
        int id = new_id();
        lines.push_back("#" + std::to_string(id) + "=CARTESIAN_POINT('',(" + fmt(u) + "," + fmt(v) + "));");
        return id;
    }

    int write_nurbs_curve(const NurbsCurve& nc, bool as_2d = false) {
        if (!nc.is_valid() || nc.cv_count() < nc.order()) return -1;

        // Write control points
        std::vector<int> pt_ids;
        const double* cv = nc.m_cv.data();
        int stride = nc.m_cv_stride;
        bool is_rat = nc.m_is_rat != 0;
        std::vector<double> weights;
        // Respect the curve DIMENSION: a dim-2 UV trim curve has stride 2, and reading
        // cv[i*stride+2] takes the NEXT control point's x -- and one past the array end on
        // the last point (an uninitialized read that intermittently wrote "-nan" into the
        // STEP file and broke the whole parse on read-back).
        int cdim = nc.m_dim;
        for (int i = 0; i < nc.cv_count(); i++) {
            double x, y, z;
            if (is_rat) {
                double w = cv[i*stride+cdim];
                if (std::abs(w) < 1e-14) w = 1.0;
                x = cv[i*stride+0]/w; y = cv[i*stride+1]/w;
                z = (cdim > 2) ? cv[i*stride+2]/w : 0.0;
                weights.push_back(w);
            } else {
                x = cv[i*stride+0]; y = cv[i*stride+1];
                z = (cdim > 2) ? cv[i*stride+2] : 0.0;
            }
            // Parameter-space (pcurve) control points are 2D per STEP; the surrounding
            // DEFINITIONAL_REPRESENTATION context declares a 2-coordinate space.
            pt_ids.push_back(as_2d ? write_point_2d(x, y) : write_point(x, y, z));
        }

        auto full = full_from_internal(nc.m_nurbsknot);
        auto [kvals, kmults] = compress_knots(full);
        int degree = nc.m_order - 1;

        int id = new_id();
        if (!is_rat) {
            // Simple entity
            std::string s = "#" + std::to_string(id) + "=B_SPLINE_CURVE_WITH_KNOTS(''," +
                std::to_string(degree) + "," + fmt_ref_list(pt_ids) +
                ",.UNSPECIFIED.,.F.,.U.," + fmt_list(kmults) + "," + fmt_list(kvals) +
                ",.UNSPECIFIED.);";
            lines.push_back(s);
        } else {
            // Complex entity: BOUNDED_CURVE + B_SPLINE_CURVE + B_SPLINE_CURVE_WITH_KNOTS + RATIONAL_B_SPLINE_CURVE
            // Complex instance MUST list every supertype (strict readers -- OCCT, Rhino --
            // reject partial complexes and silently DROP the curve, which drops every face
            // bound that references it): canonical alphabetical order per Part 21.
            std::string s = "#" + std::to_string(id) + "=(BOUNDED_CURVE()B_SPLINE_CURVE(" +
                std::to_string(degree) + "," + fmt_ref_list(pt_ids) +
                ",.UNSPECIFIED.,.F.,.U.)B_SPLINE_CURVE_WITH_KNOTS(" +
                fmt_list(kmults) + "," + fmt_list(kvals) +
                ",.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(" +
                fmt_list(weights) + ")REPRESENTATION_ITEM(''));";
            lines.push_back(s);
        }
        return id;
    }

    int write_nurbs_surface(const NurbsSurface& srf) {
        if (!srf.is_valid()) return -1;

        int cv_u = srf.m_cv_count[0], cv_v = srf.m_cv_count[1];
        bool is_rat = srf.m_is_rat != 0;
        std::vector<std::vector<int>> pt_ids(cv_u, std::vector<int>(cv_v));
        std::vector<std::vector<double>> weight_grid(cv_u, std::vector<double>(cv_v, 1.0));

        for (int u = 0; u < cv_u; u++) {
            for (int v = 0; v < cv_v; v++) {
                double x, y, z, w = 1.0;
                if (is_rat) {
                    srf.get_cv_4d(u, v, x, y, z, w);
                    if (std::abs(w) > 1e-14) { x/=w; y/=w; z/=w; }
                    weight_grid[u][v] = w;
                } else {
                    Point pt = srf.get_cv(u, v);
                    x = pt[0]; y = pt[1]; z = pt[2];
                }
                pt_ids[u][v] = write_point(x, y, z);
            }
        }

        auto full_u = full_from_internal(srf.m_nurbsknot[0]);
        auto full_v = full_from_internal(srf.m_nurbsknot[1]);
        auto [ku_vals, ku_mults] = compress_knots(full_u);
        auto [kv_vals, kv_mults] = compress_knots(full_v);
        int u_deg = srf.m_order[0] - 1, v_deg = srf.m_order[1] - 1;

        // Build control_points_list string: ((#p00,#p01,...),(#p10,...),...)
        std::string cpts = "(";
        for (int u = 0; u < cv_u; u++) {
            if (u) cpts += ",";
            cpts += "(";
            for (int v = 0; v < cv_v; v++) {
                if (v) cpts += ",";
                cpts += "#" + std::to_string(pt_ids[u][v]);
            }
            cpts += ")";
        }
        cpts += ")";

        int id = new_id();
        if (!is_rat) {
            std::string s = "#" + std::to_string(id) + "=B_SPLINE_SURFACE_WITH_KNOTS(''," +
                std::to_string(u_deg) + "," + std::to_string(v_deg) + "," + cpts +
                ",.UNSPECIFIED.,.F.,.F.,.U.," +
                fmt_list(ku_mults) + "," + fmt_list(kv_mults) + "," +
                fmt_list(ku_vals) + "," + fmt_list(kv_vals) + ",.UNSPECIFIED.);";
            lines.push_back(s);
        } else {
            // Build weight grid string
            std::string wgrid = "(";
            for (int u = 0; u < cv_u; u++) {
                if (u) wgrid += ",";
                wgrid += "(";
                for (int v = 0; v < cv_v; v++) {
                    if (v) wgrid += ",";
                    wgrid += fmt(weight_grid[u][v]);
                }
                wgrid += ")";
            }
            wgrid += ")";
            // Complete complex instance (all supertypes, canonical order) -- strict readers
            // drop partial complexes, which silently deletes every rational surface (spheres,
            // cylinders, tori) from the imported model.
            std::string s = "#" + std::to_string(id) + "=(BOUNDED_SURFACE()B_SPLINE_SURFACE(" +
                std::to_string(u_deg) + "," + std::to_string(v_deg) + "," + cpts +
                ",.UNSPECIFIED.,.F.,.F.,.U.)B_SPLINE_SURFACE_WITH_KNOTS(" +
                fmt_list(ku_mults) + "," + fmt_list(kv_mults) + "," +
                fmt_list(ku_vals) + "," + fmt_list(kv_vals) +
                ",.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(" +
                wgrid + ")REPRESENTATION_ITEM('')SURFACE());";
            lines.push_back(s);
        }
        return id;
    }

    // Write NurbsSurfaceTrimmed as ADVANCED_FACE + topology
    // outer_loop is a 2D NurbsCurve in UV space; we sample it to get 3D edge via surface evaluation
    // Shared 2D parametric representation context (one per file), for pcurve
    // DEFINITIONAL_REPRESENTATIONs.
    int ctx2d_cache = -1;
    int ctx2d() {
        if (ctx2d_cache < 0)
            ctx2d_cache = write_raw("(GEOMETRIC_REPRESENTATION_CONTEXT(2)"
                                    "PARAMETRIC_REPRESENTATION_CONTEXT()"
                                    "REPRESENTATION_CONTEXT('2D SPACE',''))");
        return ctx2d_cache;
    }

    // Parameter-space image of an edge on one surface: PCURVE -> DEFINITIONAL_REPRESENTATION
    // -> dim-2 B-spline. Importers need these to build wires on PERIODIC surfaces (a torus
    // face bounded only by 3D seam curves imports with its bounds dropped at 2x area).
    int write_pcurve(int srf_id, const NurbsCurve& uv) {
        int c2 = write_nurbs_curve(uv, /*as_2d=*/true);
        if (c2 < 0 || srf_id < 0) return -1;
        int dr = write_raw("DEFINITIONAL_REPRESENTATION('',(#" + std::to_string(c2) + "),#"
                           + std::to_string(ctx2d()) + ")");
        return write_raw("PCURVE('',#" + std::to_string(srf_id) + ",#" + std::to_string(dr) + ")");
    }

    int write_trimmed_face(const NurbsSurfaceTrimmed& trimmed) {
        int srf_id = write_nurbs_surface(trimmed.m_surface);
        if (srf_id < 0) return -1;

        auto write_loop_as_face_bound = [&](const NurbsCurve& loop_2d, bool is_outer) -> int {
            // Evaluate 3D positions of the trim curve
            // For simplicity, sample the 2D curve and evaluate on the surface
            auto kts = loop_2d.get_nurbsknots();
            if (kts.empty() || loop_2d.cv_count() < loop_2d.order()) return -1;
            int deg = loop_2d.order() - 1;
            double tmin = kts[deg > 0 ? deg-1 : 0];
            double tmax = kts[kts.size() - (deg > 0 ? deg : 1)];
            int n_samples = std::max(2, loop_2d.cv_count() * 2);

            std::vector<Point> pts3d;
            for (int i = 0; i < n_samples; i++) {
                double t = (n_samples > 1) ? tmin + (tmax-tmin)*i/(n_samples-1) : tmin;
                Point uv = loop_2d.point_at(t);
                Point p3d = trimmed.m_surface.point_at(uv[0], uv[1]);
                pts3d.push_back(p3d);
            }
            if (pts3d.empty()) return -1;

            // Write start/end vertices (same for closed loop)
            int v0_pt = write_point(pts3d.front()[0], pts3d.front()[1], pts3d.front()[2]);
            int v0 = new_id(); lines.push_back("#" + std::to_string(v0) + "=VERTEX_POINT('',#" + std::to_string(v0_pt) + ");");

            // Write the 3D curve as a polyline NURBS
            std::vector<int> sample_pt_ids;
            for (const auto& p : pts3d) sample_pt_ids.push_back(write_point(p[0], p[1], p[2]));

            int crv3d_id = new_id();
            lines.push_back("#" + std::to_string(crv3d_id) + "=B_SPLINE_CURVE_WITH_KNOTS(''," +
                std::to_string(1) + "," + fmt_ref_list(sample_pt_ids) +
                ",.POLYLINE_FORM.,.T.,.U.," +
                fmt_list(std::vector<int>(n_samples, 1)) + "," +
                [&]() { std::vector<double> kv; for(int i=0;i<n_samples;i++) kv.push_back((double)i); return fmt_list(kv); }() +
                ",.UNSPECIFIED.);");

            // Write the 2D curve (PCURVE)
            write_nurbs_curve(loop_2d); // 2D trim curve in UV space

            // Write edge curve
            int ec_id = new_id();
            lines.push_back("#" + std::to_string(ec_id) + "=EDGE_CURVE('',#" + std::to_string(v0) +
                ",#" + std::to_string(v0) + ",#" + std::to_string(crv3d_id) + ",.T.);");

            // Oriented edge
            int oe_id = new_id();
            lines.push_back("#" + std::to_string(oe_id) + "=ORIENTED_EDGE('',*,*,#" + std::to_string(ec_id) + ",.T.);");

            // Edge loop
            int el_id = new_id();
            lines.push_back("#" + std::to_string(el_id) + "=EDGE_LOOP('',(" + "#" + std::to_string(oe_id) + "));");

            // Face bound
            int fb_id = new_id();
            std::string fb_type = is_outer ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            lines.push_back("#" + std::to_string(fb_id) + "=" + fb_type + "('',#" + std::to_string(el_id) + ",.T.);");
            return fb_id;
        };

        int outer_bound = write_loop_as_face_bound(trimmed.m_outer_loop, true);
        if (outer_bound < 0) return -1;

        std::string bounds = "(#" + std::to_string(outer_bound);
        for (const auto& inner : trimmed.m_inner_loops) {
            int ib = write_loop_as_face_bound(inner, false);
            if (ib >= 0) bounds += ",#" + std::to_string(ib);
        }
        bounds += ")";

        int face_id = new_id();
        lines.push_back("#" + std::to_string(face_id) + "=ADVANCED_FACE(''," + bounds + ",#" + std::to_string(srf_id) + ",.T.);");
        return face_id;
    }

    // Emit an arbitrary entity body ("TYPE(args)") with a fresh id; returns the id.
    int write_raw(const std::string& body) {
        int id = new_id();
        lines.push_back("#" + std::to_string(id) + "=" + body + ";");
        return id;
    }

    // Wrap the given shell content into the AP214 PRODUCT / SHAPE_DEFINITION_REPRESENTATION
    // skeleton CAD importers REQUIRE to find any geometry at all: Rhino (and OCCT's strict
    // STEPControl_Reader) locate transferable roots exclusively through
    // SHAPE_DEFINITION_REPRESENTATION -> PRODUCT; a DATA section full of bare ADVANCED_FACEs
    // has ZERO roots and imports as nothing. `shell_face_ids` become one CLOSED_SHELL +
    // MANIFOLD_SOLID_BREP when `closed`, else one OPEN_SHELL + SHELL_BASED_SURFACE_MODEL.
    void finish_shape(const std::vector<int>& shell_face_ids, bool closed, const std::string& name,
                      double uncertainty = 1e-6) {
        finish_shape(std::vector<std::vector<int>>{shell_face_ids}, closed, name, uncertainty);
    }

    // AP214 surface-color presentation chain; returns the PRESENTATION_STYLE_ASSIGNMENT
    // to hang STYLED_ITEMs on (one per ADVANCED_FACE -- the binding readers support best).
    int color_style(double r, double g, double b) {
        int c  = write_raw("COLOUR_RGB(''," + fmt(r) + "," + fmt(g) + "," + fmt(b) + ")");
        int fc = write_raw("FILL_AREA_STYLE_COLOUR('',#" + std::to_string(c) + ")");
        int fa = write_raw("FILL_AREA_STYLE('',(#" + std::to_string(fc) + "))");
        int sf = write_raw("SURFACE_STYLE_FILL_AREA(#" + std::to_string(fa) + ")");
        int ss = write_raw("SURFACE_SIDE_STYLE('',(#" + std::to_string(sf) + "))");
        int su = write_raw("SURFACE_STYLE_USAGE(.BOTH.,#" + std::to_string(ss) + ")");
        return write_raw("PRESENTATION_STYLE_ASSIGNMENT((#" + std::to_string(su) + "))");
    }

    // Multi-shell form: one CLOSED_SHELL + MANIFOLD_SOLID_BREP per face group. A boolean
    // whose operands touch only along a measure-zero contact (tangent line, shared corner)
    // is TWO watertight shells; wrapping both faces sets in ONE solid makes the reader's
    // shell splitter orphan one of them (BRepCheck SubshapeNotInShape, volume -0).
    // One body per shell group, honoring a PER-SHELL closed flag (a colored multi-part
    // file mixes watertight operands with a possibly-open result; a global flag wrote
    // EVERY part as OPEN_SHELL as soon as one part was open).
    std::vector<int> make_bodies(const std::vector<std::vector<int>>& shells, bool closed) {
        std::vector<int> bodies;
        for (const auto& sf : shells) {
            if (sf.empty()) continue;
            int shell = write_raw(std::string(closed ? "CLOSED_SHELL" : "OPEN_SHELL")
                                  + "(''," + fmt_ref_list(sf) + ")");
            if (closed) bodies.push_back(write_raw("MANIFOLD_SOLID_BREP('',#" + std::to_string(shell) + ")"));
            else        bodies.push_back(write_raw("SHELL_BASED_SURFACE_MODEL('',(#" + std::to_string(shell) + "))"));
        }
        return bodies;
    }

    void finish_shape(const std::vector<std::vector<int>>& shells, bool closed, const std::string& name,
                      double uncertainty = 1e-6, const std::vector<int>* styled_items = nullptr) {
        // An EMPTY result (e.g. cone x torus common: the cone threads the hole without
        // touching) still gets the product skeleton with a bare SHAPE_REPRESENTATION --
        // a file with a naked DATA section fails strict parsers ("Incorrect Syntax").
        finish_product(make_bodies(shells, closed), closed, name, uncertainty, styled_items);
    }

    void finish_product(const std::vector<int>& bodies, bool closed, const std::string& name,
                        double uncertainty = 1e-6, const std::vector<int>* styled_items = nullptr) {
        int o  = write_point(0, 0, 0);
        int dz = write_raw("DIRECTION('',(0.,0.,1.))");
        int dx = write_raw("DIRECTION('',(1.,0.,0.))");
        int ax = write_raw("AXIS2_PLACEMENT_3D('',#" + std::to_string(o) + ",#" + std::to_string(dz)
                           + ",#" + std::to_string(dx) + ")");
        int lu = write_raw("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
        int au = write_raw("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
        int su = write_raw("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
        // The uncertainty IS the sewing tolerance importers use: understate it (1e-6) and a
        // reader cannot sew section edges whose cross-operand copies agree only to the
        // kernel's coincidence tolerance -- shells stay open and healing re-orients them
        // arbitrarily (box fuse sphere imported as box MINUS sphere). State the honest value.
        // An empty result has no bbox: the diag-scaled uncertainty arrives as inf/NaN and
        // "LENGTH_MEASURE(inf)" fails strict parsers ("Incorrect Syntax"). Clamp to 1e-6.
        if (!std::isfinite(uncertainty) || uncertainty <= 0.0) uncertainty = 1e-6;
        int un = write_raw("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" + fmt(uncertainty)
                           + "),#" + std::to_string(lu) + ",'distance_accuracy_value','')");
        int gc = write_raw("(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#"
                           + std::to_string(un) + "))GLOBAL_UNIT_ASSIGNED_CONTEXT((#" + std::to_string(lu)
                           + ",#" + std::to_string(au) + ",#" + std::to_string(su)
                           + "))REPRESENTATION_CONTEXT('',''))");
        int ac = write_raw("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
        write_raw("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#"
                  + std::to_string(ac) + ")");
        int pc = write_raw("PRODUCT_CONTEXT('',#" + std::to_string(ac) + ",'mechanical')");
        int pr = write_raw("PRODUCT('" + name + "','" + name + "','',(#" + std::to_string(pc) + "))");
        int pf = write_raw("PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(pr) + ")");
        int dc = write_raw("PRODUCT_DEFINITION_CONTEXT('part definition',#" + std::to_string(ac) + ",'design')");
        int pd = write_raw("PRODUCT_DEFINITION('design','',#" + std::to_string(pf) + ",#" + std::to_string(dc) + ")");
        int ps = write_raw("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(pd) + ")");
        std::string rep_type = bodies.empty() ? "SHAPE_REPRESENTATION"
                             : closed         ? "ADVANCED_BREP_SHAPE_REPRESENTATION"
                                              : "MANIFOLD_SURFACE_SHAPE_REPRESENTATION";
        std::string items = "(#" + std::to_string(ax);
        for (int b : bodies) items += ",#" + std::to_string(b);
        items += ")";
        int rp = write_raw(rep_type + "('" + name + "'," + items + ",#" + std::to_string(gc) + ")");
        write_raw("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ps) + ",#" + std::to_string(rp) + ")");
        // Face colors: styled items live in their own presentation representation bound to
        // the SAME geometric context.
        if (styled_items && !styled_items->empty()) {
            std::string si = "(";
            for (size_t k = 0; k < styled_items->size(); ++k) {
                if (k) si += ",";
                si += "#" + std::to_string((*styled_items)[k]);
            }
            si += ")";
            write_raw("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION(''," + si
                      + ",#" + std::to_string(gc) + ")");
        }
    }

    std::string emit(const std::string& schema = "AUTOMOTIVE_DESIGN") const {
        std::string out = "ISO-10303-21;\nHEADER;\n";
        out += "FILE_DESCRIPTION((''),'2;1');\n";
        out += "FILE_NAME('','',(''),(''),'','','');\n";
        out += "FILE_SCHEMA(('" + schema + "'));\n";
        out += "ENDSEC;\nDATA;\n";
        for (const auto& l : lines) out += l + "\n";
        out += "ENDSEC;\nEND-ISO-10303-21;\n";
        return out;
    }
};

// ============================================================
// Public API
// ============================================================

std::vector<Point> read_file_step_points(const std::string& filepath) {
    StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<Point> out;
    for (int id : sf.ids_of_type("CARTESIAN_POINT"))
        out.push_back(r.get_point(id));
    return out;
}

std::vector<NurbsCurve> read_file_step_nurbscurves(const std::string& filepath) {
    StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<NurbsCurve> out;
    for (int id : sf.ids_of_type("B_SPLINE_CURVE_WITH_KNOTS")) {
        NurbsCurve nc = r.get_nurbs_curve(id);
        if (nc.is_valid()) out.push_back(std::move(nc));
    }
    return out;
}

std::vector<NurbsSurface> read_file_step_nurbssurfaces(const std::string& filepath) {
    StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<NurbsSurface> out;
    for (int id : sf.ids_of_type("B_SPLINE_SURFACE_WITH_KNOTS")) {
        NurbsSurface srf = r.get_nurbs_surface(id);
        if (srf.is_valid()) out.push_back(std::move(srf));
    }
    return out;
}

std::vector<NurbsSurfaceTrimmed> read_file_step_nurbssurfaces_trimmed(const std::string& filepath) {
    StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<NurbsSurfaceTrimmed> out;
    // ADVANCED_FACE with B_SPLINE_SURFACE_WITH_KNOTS surface → NurbsSurfaceTrimmed
    for (int face_id : sf.ids_of_type("ADVANCED_FACE")) {
        const StepEntity* face_ent = nullptr;
        for (const auto& kv : sf.entities) if (kv.first == face_id) { face_ent = &kv.second; break; }
        if (!face_ent) continue;
        const StepSubEntity* face = face_ent->find("ADVANCED_FACE");
        if (!face) continue;

        int surface_ref = -1;
        std::vector<int> bound_refs;
        for (const auto& p : face->params) {
            if (p.tag == StepTag::List) {
                for (const auto& v : p.list) if (v.tag == StepTag::Ref) bound_refs.push_back(v.ref_id);
            } else if (p.tag == StepTag::Ref) {
                surface_ref = p.ref_id;
            }
        }
        if (surface_ref < 0) continue;

        // Only handle NURBS surfaces for this function
        auto surf_it = sf.entities.find(surface_ref);
        if (surf_it == sf.entities.end()) continue;
        if (!surf_it->second.has("B_SPLINE_SURFACE_WITH_KNOTS")) continue;

        NurbsSurface srf = r.get_nurbs_surface(surface_ref);
        if (!srf.is_valid()) continue;

        // Find outer loop and collect 2D curves from PCURVE or project
        NurbsCurve outer_loop;
        bool got_outer = false;
        for (int bid : bound_refs) {
            auto bit = sf.entities.find(bid);
            if (bit == sf.entities.end()) continue;
            // accept BOTH spellings (see the FACE_BOUND note above); this legacy path takes
            // the first bound as the outer one, which is what it already assumed.
            bool is_outer = bit->second.has("FACE_OUTER_BOUND");
            const StepSubEntity* bsub = is_outer ? bit->second.find("FACE_OUTER_BOUND")
                                                 : bit->second.find("FACE_BOUND");
            if (!bsub) continue;
            int loop_ref = -1;
            for (const auto& p : bsub->params) if (p.tag == StepTag::Ref) { loop_ref = p.ref_id; break; }
            if (loop_ref < 0) continue;

            auto lit = sf.entities.find(loop_ref);
            if (lit == sf.entities.end()) continue;
            const StepSubEntity* lsub = lit->second.find("EDGE_LOOP");
            if (!lsub) continue;

            // Collect ordered UV points by sampling 3D edge curves then projecting
            // Try to find PCURVE-based 2D curves first (look for B_SPLINE_CURVE_WITH_KNOTS in 2D context)
            std::vector<int> oe_refs;
            for (const auto& p : lsub->params)
                if (p.tag == StepTag::List)
                    for (const auto& v : p.list) if (v.tag == StepTag::Ref) oe_refs.push_back(v.ref_id);

            std::vector<Point> uv_pts;
            for (int oe_id : oe_refs) {
                auto oeit = sf.entities.find(oe_id);
                if (oeit == sf.entities.end()) continue;
                const StepSubEntity* oe = oeit->second.find("ORIENTED_EDGE");
                if (!oe) continue;
                int ec_ref = -1;
                for (const auto& p : oe->params)
                    if (p.tag == StepTag::Ref) ec_ref = p.ref_id;
                if (ec_ref < 0) continue;
                auto ecit = sf.entities.find(ec_ref);
                if (ecit == sf.entities.end()) continue;
                const StepSubEntity* ec = ecit->second.find("EDGE_CURVE");
                if (!ec) continue;
                // Get start vertex and curve
                std::vector<int> ec_refs;
                for (const auto& p : ec->params) if (p.tag == StepTag::Ref) ec_refs.push_back(p.ref_id);
                if (ec_refs.size() < 3) continue;
                Point vs = r.get_point(ec_refs[0]);
                Point ve = r.get_point(ec_refs[1]);
                auto samples = r.sample_curve(ec_refs[2], vs, ve, 8);
                for (const auto& s : samples) {
                    // Project onto surface UV (Newton iteration would be ideal, but use approx)
                    // For NURBS surfaces, UV projection is complex; use (0,0) as placeholder
                    uv_pts.emplace_back(s[0], s[1], 0.0); // placeholder
                }
            }
            if (uv_pts.size() >= 2) {
                outer_loop = polyline_nurbs(uv_pts, 2);
                got_outer = true;
            }
            break;
        }
        if (!got_outer) continue;

        NurbsSurfaceTrimmed nst;
        nst.m_surface = srf;
        nst.m_outer_loop = outer_loop;
        out.push_back(std::move(nst));
    }
    return out;
}

std::vector<BRep> read_file_step_breps(const std::string& filepath) {
    StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<BRep> out;
    // Deterministic FILE order (ascending entity id): the entity map is unordered, and a
    // multi-solid file (A red / B blue / result green) must read back as written.
    //
    // ROOT SPELLINGS. Enumerating only MANIFOLD_SOLID_BREP made three other legal roots read as
    // an EMPTY file -- and an empty operand makes a boolean return the other operand unchanged,
    // which every closure-based verdict scores as a PASS (the same silent-false-pass mechanism
    // as the unread VERTEX_LOOP). BREP_WITH_VOIDS is a SUBTYPE of manifold_solid_brep but is
    // written under its own name, so has("MANIFOLD_SOLID_BREP") is false for it;
    // SHELL_BASED_SURFACE_MODEL carries a list of OPEN_SHELLs, which build_from_shell already
    // accepted -- only this scan never reached them. Measured on the repo corpus:
    // step_import/hammer.step (45 faces) and step_import/fuse.step (10) read as 0 breps, and
    // massprops_fx/occ_box_cavity.step (a box with a cavity) likewise.
    // Void shells of a BREP_WITH_VOIDS are read as their own BReps (the kernel has no cavity
    // container); the outer shell stays first.
    std::vector<std::pair<int, int>> roots;   // (entity id, shell ref) in file order
    std::vector<int> ids;
    for (const auto& kv : sf.entities) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());
    for (int id : ids) {
        const StepEntity& e = sf.entities.find(id)->second;
        auto refs_of = [&](const StepSubEntity* s, bool list_too) {
            std::vector<int> rr;
            if (!s) return rr;
            for (const auto& p : s->params) {
                if (p.tag == StepTag::Ref) rr.push_back(p.ref_id);
                else if (list_too && p.tag == StepTag::List)
                    for (const auto& v : p.list) if (v.tag == StepTag::Ref) rr.push_back(v.ref_id);
            }
            return rr;
        };
        if (e.has("MANIFOLD_SOLID_BREP") || e.has("BREP_WITH_VOIDS")) {
            const StepSubEntity* s = e.find("MANIFOLD_SOLID_BREP");
            if (!s) s = e.find("BREP_WITH_VOIDS");
            auto rr = refs_of(s, true);
            for (int sh : rr) roots.emplace_back(id, sh);
        } else if (e.has("SHELL_BASED_SURFACE_MODEL")) {
            for (int sh : refs_of(e.find("SHELL_BASED_SURFACE_MODEL"), true))
                roots.emplace_back(id, sh);
        }
    }
    for (const auto& [id, shell_ref0] : roots) {
        (void)id;
        int shell_ref = shell_ref0;
        // ORIENTED_CLOSED_SHELL wraps the real shell with a sense flag (void shells).
        const auto& sh = sf.entities.find(shell_ref);
        if (sh != sf.entities.end() && sh->second.has("ORIENTED_CLOSED_SHELL")) {
            const StepSubEntity* os = sh->second.find("ORIENTED_CLOSED_SHELL");
            for (const auto& p : os->params) if (p.tag == StepTag::Ref) { shell_ref = p.ref_id; break; }
        }
        if (shell_ref < 0) continue;
        BRepBuilder builder(r, sf);
        BRep b = builder.build_from_shell(shell_ref);
        if (!b.m_faces.empty()) out.push_back(std::move(b));
    }
    return out;
}

static void write_step_string(const std::string& content, const std::string& filepath) {
    std::ofstream out(filepath);
    out << content;
}

void write_file_step_nurbscurves(const std::vector<NurbsCurve>& curves, const std::string& filepath) {
    StepWriter w;
    for (const auto& nc : curves) w.write_nurbs_curve(nc);
    write_step_string(w.emit(), filepath);
}

void write_file_step_nurbssurfaces(const std::vector<NurbsSurface>& surfaces, const std::string& filepath) {
    StepWriter w;
    for (const auto& srf : surfaces) w.write_nurbs_surface(srf);
    write_step_string(w.emit(), filepath);
}

void write_file_step_nurbssurfaces_trimmed(const std::vector<NurbsSurfaceTrimmed>& trimmed, const std::string& filepath) {
    StepWriter w;
    std::vector<int> face_ids;
    for (const auto& t : trimmed) {
        int fid = w.write_trimmed_face(t);
        if (fid >= 0) face_ids.push_back(fid);
    }
    w.finish_shape(face_ids, false, "trimmed");   // importers need the product skeleton
    write_step_string(w.emit(), filepath);
}

// Emit one BRep into the writer: shared vertices and edges are written ONCE and referenced by
// every adjacent face, degenerated edges are omitted (a wire of only degenerated edges becomes a
// VERTEX_LOOP, as OCCT writes a full sphere), and each face's same_sense flag is its orientation
// in the shell. Returns one face-id group per shell (free faces form a last, open group) with
// its closed flag; `diag_out` receives the vertex bounding-box diagonal.
static std::vector<std::pair<std::vector<int>, bool>> emit_brep_shells(StepWriter& w, const BRep& brep, double& diag_out) {
    std::map<int, int> vid, eid, sid;
    auto vertex_id = [&](int vi) {
        auto it = vid.find(vi);
        if (it != vid.end()) return it->second;
        const Point& p = brep.m_vertices[vi].point;
        int pt = w.write_point(p[0], p[1], p[2]);
        return vid[vi] = w.write_raw("VERTEX_POINT('',#" + std::to_string(pt) + ")");
    };
    auto edge_id = [&](int ei) {
        auto it = eid.find(ei);
        if (it != eid.end()) return it->second;
        const BRepEdge& e = brep.m_edges[ei];
        int c = w.write_nurbs_curve(brep.m_curves_3d[e.curve_3d_index]);
        if (c < 0) return eid[ei] = -1;
        return eid[ei] = w.write_raw("EDGE_CURVE('',#" + std::to_string(vertex_id(e.start_vertex)) + ",#"
                                     + std::to_string(vertex_id(e.end_vertex)) + ",#" + std::to_string(c) + ",.T.)");
    };
    auto face_id = [&](int fi, BRepOrientation fo) {
        const BRepFace& f = brep.m_faces[fi];
        if (!sid.count(f.surface_index)) sid[f.surface_index] = w.write_nurbs_surface(brep.m_surfaces[f.surface_index]);
        int srf = sid[f.surface_index];
        if (srf < 0) return -1;
        std::vector<int> bounds;
        for (size_t wi = 0; wi < f.wires.size(); ++wi) {
            std::vector<int> oes;
            int any_vertex = -1;
            for (const auto& er : brep.wire_edges(f.wires[wi])) {
                const BRepEdge& e = brep.m_edges[er.index];
                if (any_vertex < 0) any_vertex = e.start_vertex;
                if (e.degenerated) continue;
                int ec = edge_id(er.index);
                if (ec < 0) continue;
                oes.push_back(w.write_raw("ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + ",."
                                          + (er.orientation == BRepOrientation::Forward ? "T" : "F") + ".)"));
            }
            int loop;
            if (!oes.empty()) loop = w.write_raw("EDGE_LOOP(''," + w.fmt_ref_list(oes) + ")");
            else if (any_vertex >= 0) loop = w.write_raw("VERTEX_LOOP('',#" + std::to_string(vertex_id(any_vertex)) + ")");
            else continue;
            bounds.push_back(w.write_raw(std::string(wi == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND")
                                         + "('',#" + std::to_string(loop) + ",.T.)"));
        }
        if (bounds.empty()) return -1;
        return w.write_raw("ADVANCED_FACE(''," + w.fmt_ref_list(bounds) + ",#" + std::to_string(srf) + ",."
                           + (fo == BRepOrientation::Forward ? "T" : "F") + ".)");
    };

    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (const auto& v : brep.m_vertices)
        for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], v.point[k]); hi[k] = std::max(hi[k], v.point[k]); }
    diag_out = brep.m_vertices.empty() ? 1.0
             : std::sqrt((hi[0]-lo[0])*(hi[0]-lo[0]) + (hi[1]-lo[1])*(hi[1]-lo[1]) + (hi[2]-lo[2])*(hi[2]-lo[2]));

    std::vector<std::pair<std::vector<int>, bool>> groups;
    std::vector<bool> in_shell(brep.m_faces.size(), false);
    for (int si = 0; si < brep.shell_count(); ++si) {
        std::vector<int> ids;
        for (const auto& fr : brep.m_shells[si].faces) {
            in_shell[fr.index] = true;
            int id = face_id(fr.index, fr.orientation);
            if (id >= 0) ids.push_back(id);
        }
        if (!ids.empty()) groups.push_back({ids, brep.is_closed(si)});
    }
    std::vector<int> free_ids;
    for (int fi = 0; fi < brep.face_count(); ++fi) {
        if (in_shell[fi]) continue;
        int id = face_id(fi, BRepOrientation::Forward);
        if (id >= 0) free_ids.push_back(id);
    }
    if (!free_ids.empty()) groups.push_back({free_ids, false});
    return groups;
}

void write_file_step_brep(const BRep& brep, const std::string& filepath) {
    StepWriter w;
    double diag = 1.0;
    std::vector<int> bodies;
    bool any_closed = false;
    for (const auto& [ids, closed] : emit_brep_shells(w, brep, diag)) {
        std::vector<int> bb = w.make_bodies({ids}, closed);
        bodies.insert(bodies.end(), bb.begin(), bb.end());
        any_closed = any_closed || closed;
    }
    w.finish_product(bodies, any_closed, brep.name.empty() ? "brep" : brep.name, diag * 1e-4);
    write_step_string(w.emit(), filepath);
}

void write_file_step_breps(const std::vector<const BRep*>& breps, const std::string& name,
                           const std::string& filepath) {
    // Several bodies in ONE file, each colored from its OWN surfacecolor (AP214 presentation
    // chain: STYLED_ITEM per ADVANCED_FACE -> ... -> COLOUR_RGB).
    StepWriter w;
    std::vector<int> bodies;
    std::vector<int> styled;
    bool any_closed = false;
    double diag_all = 1.0;
    for (const BRep* b : breps) {
        if (!b) continue;
        double diag = 1.0;
        std::vector<std::pair<std::vector<int>, bool>> groups = emit_brep_shells(w, *b, diag);
        diag_all = std::max(diag_all, diag);
        const Color& col = b->surfacecolor;
        int psa = w.color_style(col.r, col.g, col.b);
        for (const auto& [ids, closed] : groups) {
            std::vector<int> bb = w.make_bodies({ids}, closed);
            bodies.insert(bodies.end(), bb.begin(), bb.end());
            any_closed = any_closed || closed;
            for (int fid : ids)
                styled.push_back(w.write_raw("STYLED_ITEM('',(#" + std::to_string(psa) + "),#" + std::to_string(fid) + ")"));
        }
    }
    w.finish_product(bodies, any_closed, name, diag_all * 1e-4, &styled);
    write_step_string(w.emit(), filepath);
}

} } // namespace session_cpp::file_step
