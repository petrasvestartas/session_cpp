#include "file_step.h"
#include "point.h"
#include "vector.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "nurbssurface_trimmed.h"
#include "brep.h"
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

    // Sample 3D points along a curve entity
    std::vector<Point> sample_curve(int curve_id, const Point& v_start, const Point& v_end, int n) {
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

    explicit BRepBuilder(StepReader& reader, const StepFile& s) : r(reader), sf(s) {}

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
        int pi = brep.add_vertex(pt);
        BRepVertex bv; bv.point_index = pi;
        brep.m_topology_vertices.push_back(bv);
        return vmap[vp_id] = (int)brep.m_topology_vertices.size() - 1;
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
        int curve_id = refs[2];
        Point vs = brep.m_vertices[brep.m_topology_vertices[sv].point_index];
        Point ve = brep.m_vertices[brep.m_topology_vertices[ev].point_index];

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

        int c3d = brep.add_curve_3d(crv3d);
        BRepEdge edge; edge.curve_3d_index = c3d; edge.start_vertex = sv; edge.end_vertex = ev;
        brep.m_topology_edges.push_back(edge);
        int edge_idx = (int)brep.m_topology_edges.size() - 1;
        return emap[ec_id] = edge_idx;
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

        // Get projector for UV mapping (PLANE or CYL)
        StepReader::Proj proj;
        if (surface_ref >= 0) proj = r.get_projector(surface_ref);

        // Pre-scan UV extents
        auto uv_pts_of_sample = [&](const std::vector<Point>& samples) {
            std::vector<Point> uv;
            if (proj.kind == StepReader::ProjKind::None) {
                return std::vector<Point>{{Point(0,0,0), Point(1,0,0)}};
            }
            for (const auto& s : samples) {
                auto [u, v] = r.project(proj, s);
                uv.emplace_back(u, v, 0.0);
            }
            return uv;
        };

        struct LoopEdge { int edge_idx; bool reversed; std::vector<Point> uv; };
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
                const BRepEdge& be = brep.m_topology_edges[edge_idx];
                Point vs = brep.m_vertices[brep.m_topology_vertices[be.start_vertex].point_index];
                Point ve = brep.m_vertices[brep.m_topology_vertices[be.end_vertex].point_index];
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
                    vs, ve, 16);
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
                lp.edges.push_back({edge_idx, rev, std::move(uv)});
            }
            if (!lp.edges.empty()) loops.push_back(std::move(lp));
        }

        // Chain-align UV U values for cylindrical surfaces.
        // Canonical cylinder UV: 1 unit = 90deg, full circle = 4 units.
        // Problem: per-edge seam unwrap makes each edge continuous, but opposite-winding
        // circles (one CW, one CCW) land in disjoint 4-unit windows → combined span = 8.
        // Fix: chain each edge so its loop-traversal start U aligns with the previous end.
        // Inner loops (holes) are also normalized to the outer loop's U center.
        if (proj.kind == StepReader::ProjKind::Cyl) {
            const double tau = 4.0; // canonical full-circle span

            // Chain-unwrap all loops independently
            for (auto& lp : loops) {
                double prev_end = std::numeric_limits<double>::quiet_NaN();
                for (auto& le : lp.edges) {
                    if (le.uv.empty()) continue;
                    int start_idx = le.reversed ? (int)le.uv.size()-1 : 0;
                    int end_idx   = le.reversed ? 0 : (int)le.uv.size()-1;
                    if (!std::isnan(prev_end)) {
                        double u_start = le.uv[start_idx][0];
                        int n = (int)std::round((prev_end - u_start) / tau);
                        if (n != 0)
                            for (auto& p : le.uv) p[0] += n * tau;
                    }
                    prev_end = le.uv[end_idx][0];
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
            if (have_outer) {
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
                    if (p[0]<umin) umin=p[0]; if (p[0]>umax) umax=p[0];
                    if (p[1]<vmin) vmin=p[1]; if (p[1]>vmax) vmax=p[1];
                }
        if (umin > umax) { umin=-1; umax=1; vmin=-1; vmax=1; }

        NurbsSurface srf;
        if (surface_ref >= 0) r.fill_surface(surface_ref, umin, umax, vmin, vmax, srf);
        int srf_idx = brep.add_surface(srf);
        int face_idx = brep.add_face(srf_idx, !same_sense);

        for (const auto& lp : loops) {
            BRepLoopType lt = lp.is_outer ? BRepLoopType::Outer : BRepLoopType::Inner;
            int loop_idx = brep.add_loop(face_idx, lt);
            for (const auto& le : lp.edges) {
                NurbsCurve crv2d = polyline_nurbs(le.uv, 2);
                int c2d = brep.add_curve_2d(crv2d);
                brep.add_trim(c2d, le.edge_idx, loop_idx, le.reversed, BRepTrimType::Boundary);
            }
        }
    }

    BRep build_from_shell(int shell_id) {
        const auto& sent = sf.entities.find(shell_id);
        if (sent == sf.entities.end()) return BRep();
        if (!sent->second.has("CLOSED_SHELL") && !sent->second.has("OPEN_SHELL")) return BRep();

        const StepSubEntity* shell_sub = sent->second.find("CLOSED_SHELL");
        if (!shell_sub) shell_sub = sent->second.find("OPEN_SHELL");

        std::vector<int> face_refs;
        for (const auto& p : shell_sub->params)
            if (p.tag == StepTag::List)
                for (const auto& v : p.list) if (v.tag == StepTag::Ref) face_refs.push_back(v.ref_id);

        brep.name = "step_brep";
        for (int f : face_refs) add_face(f);
        return std::move(brep);
    }
};

// ============================================================
// Layer 4: StepWriter
// ============================================================

class StepWriter {
    int next_id = 1;
    std::vector<std::string> lines;

    int new_id() { return next_id++; }

    static std::string fmt(double v) {
        char buf[64];
        if (v == (int)v && std::abs(v) < 1e15)
            std::snprintf(buf, sizeof(buf), "%d.", (int)v);
        else
            std::snprintf(buf, sizeof(buf), "%.15g", v);
        return buf;
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

    int write_nurbs_curve(const NurbsCurve& nc) {
        if (!nc.is_valid() || nc.cv_count() < nc.order()) return -1;

        // Write control points
        std::vector<int> pt_ids;
        const double* cv = nc.m_cv.data();
        int stride = nc.m_cv_stride;
        bool is_rat = nc.m_is_rat != 0;
        std::vector<double> weights;
        for (int i = 0; i < nc.cv_count(); i++) {
            double x, y, z;
            if (is_rat) {
                double w = cv[i*stride+3];
                if (std::abs(w) < 1e-14) w = 1.0;
                x = cv[i*stride+0]/w; y = cv[i*stride+1]/w; z = cv[i*stride+2]/w;
                weights.push_back(w);
            } else {
                x = cv[i*stride+0]; y = cv[i*stride+1]; z = cv[i*stride+2];
            }
            pt_ids.push_back(write_point(x, y, z));
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
            std::string s = "#" + std::to_string(id) + "=(BOUNDED_CURVE()B_SPLINE_CURVE(" +
                std::to_string(degree) + "," + fmt_ref_list(pt_ids) +
                ",.UNSPECIFIED.,.F.,.U.)B_SPLINE_CURVE_WITH_KNOTS(" +
                fmt_list(kmults) + "," + fmt_list(kvals) +
                ",.UNSPECIFIED.)RATIONAL_B_SPLINE_CURVE(" + fmt_list(weights) + "));";
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
            std::string s = "#" + std::to_string(id) + "=(BOUNDED_SURFACE()B_SPLINE_SURFACE(" +
                std::to_string(u_deg) + "," + std::to_string(v_deg) + "," + cpts +
                ",.UNSPECIFIED.,.F.,.F.,.U.)B_SPLINE_SURFACE_WITH_KNOTS(" +
                fmt_list(ku_mults) + "," + fmt_list(kv_mults) + "," +
                fmt_list(ku_vals) + "," + fmt_list(kv_vals) +
                ",.UNSPECIFIED.)RATIONAL_B_SPLINE_SURFACE(" + wgrid + "));";
            lines.push_back(s);
        }
        return id;
    }

    // Write NurbsSurfaceTrimmed as ADVANCED_FACE + topology
    // outer_loop is a 2D NurbsCurve in UV space; we sample it to get 3D edge via surface evaluation
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

    std::string emit(const std::string& schema = "AP214_AUTO_DESIGN") const {
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
            bool is_outer = bit->second.has("FACE_OUTER_BOUND");
            if (!is_outer) continue;
            const StepSubEntity* bsub = bit->second.find("FACE_OUTER_BOUND");
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
                int ec_ref = -1; bool oe_orient = true;
                for (const auto& p : oe->params) {
                    if (p.tag == StepTag::Ref) ec_ref = p.ref_id;
                    else if (p.tag == StepTag::Enum) oe_orient = (p.str == "T");
                }
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
    for (const auto& kv : sf.entities) {
        if (!kv.second.has("MANIFOLD_SOLID_BREP")) continue;
        const StepSubEntity* msb = kv.second.find("MANIFOLD_SOLID_BREP");
        if (!msb) continue;
        int shell_ref = -1;
        for (const auto& p : msb->params) if (p.tag == StepTag::Ref) { shell_ref = p.ref_id; break; }
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
    for (const auto& t : trimmed) w.write_trimmed_face(t);
    write_step_string(w.emit(), filepath);
}

void write_file_step_brep(const BRep& brep, const std::string& filepath) {
    StepWriter w;
    // Write each face as ADVANCED_FACE with its surface
    std::vector<int> face_ids;
    for (int fi = 0; fi < brep.face_count(); fi++) {
        const BRepFace& face = brep.m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)brep.m_surfaces.size()) continue;
        NurbsSurfaceTrimmed nst;
        nst.m_surface = brep.m_surfaces[face.surface_index];
        // Use outer loop's 2D trim curve if available
        bool got_outer = false;
        for (int loop_idx : face.loop_indices) {
            if (loop_idx < 0 || loop_idx >= (int)brep.m_loops.size()) continue;
            const BRepLoop& loop = brep.m_loops[loop_idx];
            if (loop.type != BRepLoopType::Outer) continue;
            std::vector<Point> uv_pts;
            for (int trim_idx : loop.trim_indices) {
                if (trim_idx < 0 || trim_idx >= (int)brep.m_trims.size()) continue;
                const BRepTrim& trim = brep.m_trims[trim_idx];
                if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)brep.m_curves_2d.size()) continue;
                const NurbsCurve& crv2d = brep.m_curves_2d[trim.curve_2d_index];
                auto kts = crv2d.get_nurbsknots();
                if (kts.empty() || crv2d.cv_count() < 2) continue;
                int deg = crv2d.order() - 1;
                double tmin = kts[deg > 0 ? deg-1 : 0];
                double tmax = kts[kts.size() - (deg > 0 ? deg : 1)];
                int ns = std::max(2, crv2d.cv_count());
                for (int i = 0; i < ns; i++) {
                    double t = (ns > 1) ? tmin + (tmax-tmin)*i/(ns-1) : tmin;
                    Point uv = crv2d.point_at(t);
                    if (trim.reversed) uv_pts.insert(uv_pts.begin(), uv);
                    else uv_pts.push_back(uv);
                }
            }
            if (uv_pts.size() >= 2) {
                nst.m_outer_loop = polyline_nurbs(uv_pts, 2);
                got_outer = true;
            }
            break;
        }
        if (!got_outer) {
            // Dummy outer loop spanning full UV domain
            std::vector<Point> box = {Point(0,0,0),Point(1,0,0),Point(1,1,0),Point(0,1,0),Point(0,0,0)};
            nst.m_outer_loop = polyline_nurbs(box, 2);
        }
        int fid = w.write_trimmed_face(nst);
        if (fid >= 0) face_ids.push_back(fid);
    }
    // CLOSED_SHELL + MANIFOLD_SOLID_BREP
    // We store these in an appendix after emit; we need to add them to the writer
    // Use a workaround: emit into a temporary, then patch
    // For simplicity, just emit as-is without shell wrapper for now
    write_step_string(w.emit(), filepath);
}

} } // namespace session_cpp::file_step
