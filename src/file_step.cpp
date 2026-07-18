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
        if (have_proj_srf) srf = proj_srf;
        else if (surface_ref >= 0) r.fill_surface(surface_ref, umin, umax, vmin, vmax, srf);
        int srf_idx = brep.add_surface(srf);
        int face_idx = brep.add_face(srf_idx, !same_sense);

        for (const auto& lp : loops) {
            BRepLoopType lt = lp.is_outer ? BRepLoopType::Outer : BRepLoopType::Inner;
            int loop_idx = brep.add_loop(face_idx, lt);
            for (const auto& le : lp.edges) {
                // Kernel convention: pcurve domain order == LOOP TRAVERSAL order (the region
                // samplers in volume()/face_interior concatenate pcurve samples without looking
                // at trim.reversed). le.uv is sampled in CURVE order -> reverse for reversed
                // trims; the trim flag keeps its edge-relative meaning.
                std::vector<Point> uv = le.uv;
                if (le.reversed) std::reverse(uv.begin(), uv.end());
                NurbsCurve crv2d = polyline_nurbs(uv, 2);
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
    std::vector<int> face_ids;
    for (const auto& t : trimmed) {
        int fid = w.write_trimmed_face(t);
        if (fid >= 0) face_ids.push_back(fid);
    }
    w.finish_shape(face_ids, false, "trimmed");   // importers need the product skeleton
    write_step_string(w.emit(), filepath);
}

// ---- Analytic quadric export -------------------------------------------------------------
// Importers are battle-tested on ANALYTIC surface entities (OCCT/Parasolid never export
// quadrics as B-splines); periodic rational B-splines are the least-trodden path and Rhino
// drops their trims (every torus face then renders as a FULL torus -- "multiple tori").
// Recognition is verification-gated: a residual over a sample grid must be ~exact, else the
// face stays a B-spline.
struct AnalyticSrf {
    int kind = 0;                 // 0 bspline, 1 plane, 2 cylinder, 3 cone, 4 sphere, 5 torus
    Point C{0,0,0};               // location (plane origin / cyl base / cone APEX / center)
    Vector Z{0,0,1}, X{1,0,0}, Y{0,1,0};
    double R = 0, r2 = 0;         // radius / (major,minor) / cone: r2 = semi-angle (rad)
};

static AnalyticSrf detect_analytic_srf(const NurbsSurface& S) {
    AnalyticSrf A;
    if (!S.is_valid()) return A;
    auto [u0, u1] = S.domain(0);
    auto [v0, v1] = S.domain(1);
    auto P = [&](double fu, double fv) { return S.point_at(u0 + (u1-u0)*fu, v0 + (v1-v0)*fv); };
    double scale = P(0,0).distance(P(0.5,0.5)) + P(0,0).distance(P(1,1)) + 1e-9;
    double tol = scale * 1e-7 + 1e-9;
    auto sub = [](const Point& a, const Point& b) { return Vector(a[0]-b[0], a[1]-b[1], a[2]-b[2]); };
    auto dot = [](const Vector& a, const Vector& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; };
    auto crs = [](const Vector& a, const Vector& b) {
        return Vector(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]); };
    auto nrm = [&](Vector v) -> Vector {
        double m = std::sqrt(dot(v, v));
        return m > 1e-14 ? Vector(v[0]/m, v[1]/m, v[2]/m) : Vector(0,0,0); };

    // PLANE: constant normal + on-plane residual
    {
        Vector n = S.normal_at(0.5*(u0+u1), 0.5*(v0+v1));
        Point O = P(0, 0);
        bool ok = std::sqrt(dot(n, n)) > 0.5;
        for (int i = 0; i <= 4 && ok; ++i)
            for (int j = 0; j <= 4 && ok; ++j)
                if (std::abs(dot(sub(P(i/4.0, j/4.0), O), n)) > tol) ok = false;
        if (ok) {
            A.kind = 1; A.C = O; A.Z = nrm(n);
            Vector xr = sub(P(1, 0), O);
            xr = Vector(xr[0]-dot(xr,A.Z)*A.Z[0], xr[1]-dot(xr,A.Z)*A.Z[1], xr[2]-dot(xr,A.Z)*A.Z[2]);
            A.X = nrm(xr);
            if (std::sqrt(dot(A.X, A.X)) < 0.5) { Vector yr = sub(P(0, 1), O); A.X = nrm(yr); }
            A.Y = crs(A.Z, A.X);
            return A;
        }
    }
    // center of the u-iso ring at fv (average of 4 quarter points; exact for full circles)
    auto ring_c = [&](double fv) {
        Point a = P(0, fv), b = P(0.25, fv), c = P(0.5, fv), d = P(0.75, fv);
        return Point(0.25*(a[0]+b[0]+c[0]+d[0]), 0.25*(a[1]+b[1]+c[1]+d[1]), 0.25*(a[2]+b[2]+c[2]+d[2]));
    };
    // SPHERE
    {
        Point Ps = P(0.5, 0), Pn = P(0.5, 1);
        Point Ct(0.5*(Ps[0]+Pn[0]), 0.5*(Ps[1]+Pn[1]), 0.5*(Ps[2]+Pn[2]));
        double Rt = 0.5*Ps.distance(Pn);
        bool ok = Rt > tol;
        for (int i = 0; i <= 4 && ok; ++i)
            for (int j = 0; j <= 4 && ok; ++j)
                if (std::abs(P(i/4.0, j/4.0).distance(Ct) - Rt) > tol) ok = false;
        if (ok) {
            A.kind = 4; A.C = Ct; A.R = Rt;
            A.Z = nrm(sub(Pn, Ps));
            Vector w = sub(P(0, 0.5), Ct);
            double h = dot(w, A.Z);
            A.X = nrm(Vector(w[0]-h*A.Z[0], w[1]-h*A.Z[1], w[2]-h*A.Z[2]));
            A.Y = crs(A.Z, A.X);
            return A;
        }
    }
    // CYLINDER / CONE: rings at v0 and v1 with (near-)constant per-ring radius
    {
        Point c0 = ring_c(0), c1 = ring_c(1);
        double r0 = P(0, 0).distance(c0), r1 = P(0, 1).distance(c1);
        double h = c0.distance(c1);
        if (h > tol) {
            Vector ax = nrm(sub(c1, c0));
            auto ring_ok = [&](double fv, const Point& cc, double rr) {
                for (int i = 0; i <= 4; ++i) {
                    Point q = P(i/4.0, fv);
                    Vector w = sub(q, cc);
                    double z = dot(w, ax);
                    double rad = std::sqrt(std::max(0.0, dot(w, w) - z*z));
                    if (std::abs(rad - rr) > tol || std::abs(z) > tol) return false;
                }
                return true;
            };
            if (ring_ok(0, c0, r0) && ring_ok(1, c1, r1)) {
                // verify a middle ring interpolates linearly (rules out tori etc.)
                Point cm = ring_c(0.5);
                double rm = P(0, 0.5).distance(cm);
                double rlin = 0.5*(r0 + r1);
                if (std::abs(rm - rlin) < tol * 4) {
                    if (std::abs(r0 - r1) < tol) {
                        A.kind = 2; A.C = c0; A.Z = ax; A.R = r0;
                        A.X = nrm(sub(P(0, 0), c0)); A.Y = crs(A.Z, A.X);
                        if (std::sqrt((A.X[0]*A.X[0]+A.X[1]*A.X[1]+A.X[2]*A.X[2])) > 0.5) return A;
                    } else {
                        // cone: apex where the radius extrapolates to zero
                        double t_apex = r0 / (r0 - r1);            // along c0->c1
                        Point apex(c0[0] + (c1[0]-c0[0])*t_apex,
                                   c0[1] + (c1[1]-c0[1])*t_apex,
                                   c0[2] + (c1[2]-c0[2])*t_apex);
                        // axis FROM apex toward the larger-radius ring
                        Vector axc = (r0 > r1) ? nrm(sub(c0, apex)) : nrm(sub(c1, apex));
                        double rb = std::max(r0, r1);
                        double hb = (r0 > r1) ? apex.distance(c0) : apex.distance(c1);
                        if (hb > tol) {
                            A.kind = 3; A.C = apex; A.Z = axc;
                            A.R = 0.0; A.r2 = std::atan2(rb, hb);   // semi-angle
                            Point pb = (r0 > r1) ? P(0, 0) : P(0, 1);
                            Vector w = sub(pb, apex);
                            double z = dot(w, A.Z);
                            A.X = nrm(Vector(w[0]-z*A.Z[0], w[1]-z*A.Z[1], w[2]-z*A.Z[2]));
                            A.Y = crs(A.Z, A.X);
                            return A;
                        }
                    }
                }
            }
        }
    }
    // TORUS: tube-circle centres trace the major circle
    {
        auto tube_c = [&](double fu) {
            Point a = P(fu, 0), b = P(fu, 0.5);
            return Point(0.5*(a[0]+b[0]), 0.5*(a[1]+b[1]), 0.5*(a[2]+b[2]));
        };
        Point c1 = tube_c(0), c2 = tube_c(0.5), c3 = tube_c(0.25);
        Point Ct(0.5*(c1[0]+c2[0]), 0.5*(c1[1]+c2[1]), 0.5*(c1[2]+c2[2]));
        Vector a1 = sub(c1, Ct), a3 = sub(c3, Ct);
        Vector Zt = nrm(crs(a1, a3));
        double Rmaj = std::sqrt(dot(a1, a1));
        double rmin = P(0, 0).distance(c1);
        if (std::sqrt(dot(Zt, Zt)) > 0.5 && Rmaj > tol && rmin > tol && rmin < Rmaj) {
            bool ok = true;
            for (int i = 0; i <= 4 && ok; ++i)
                for (int j = 0; j <= 4 && ok; ++j) {
                    Vector w = sub(P(i/4.0, j/4.0), Ct);
                    double z = dot(w, Zt);
                    double rho = std::sqrt(std::max(0.0, dot(w, w) - z*z));
                    if (std::abs(std::hypot(rho - Rmaj, z) - rmin) > tol) ok = false;
                }
            if (ok) {
                A.kind = 5; A.C = Ct; A.Z = Zt; A.R = Rmaj; A.r2 = rmin;
                A.X = nrm(a1); A.Y = crs(A.Z, A.X);
                return A;
            }
        }
    }
    return A;
}

// STEP-canonical parameters of a 3D point on an analytic surface. `radial_ok` is false at
// axis-degenerate points (poles/apex) where the angle is undefined.
static void analytic_params_of(const AnalyticSrf& A, const Point& p,
                               double& sv, double& tv, bool& radial_ok) {
    double wx = p[0]-A.C[0], wy = p[1]-A.C[1], wz = p[2]-A.C[2];
    double x = wx*A.X[0]+wy*A.X[1]+wz*A.X[2];
    double y = wx*A.Y[0]+wy*A.Y[1]+wz*A.Y[2];
    double z = wx*A.Z[0]+wy*A.Z[1]+wz*A.Z[2];
    double rho = std::sqrt(x*x + y*y);
    radial_ok = rho > 1e-9;
    switch (A.kind) {
        case 1: sv = x; tv = y; radial_ok = true; break;
        case 2: sv = std::atan2(y, x); tv = z; break;
        case 3: { sv = std::atan2(y, x);
                  double ca = std::cos(A.r2);
                  tv = (ca > 1e-12) ? z / ca : z; break; }      // v along the slant
        case 4: sv = std::atan2(y, x); tv = std::atan2(z, rho); break;
        default: sv = std::atan2(y, x); tv = std::atan2(z, rho - A.R); break;
    }
}

// Forward evaluation at canonical (s,t) -- the written analytic surface the reader binds
// pcurves against (the kernel NurbsSurface chart is unrelated). Inverse of analytic_params_of.
static Point analytic_eval(const AnalyticSrf& A, double s, double t) {
    double cs = std::cos(s), sn = std::sin(s);
    double rx = cs * A.X[0] + sn * A.Y[0];
    double ry = cs * A.X[1] + sn * A.Y[1];
    double rz = cs * A.X[2] + sn * A.Y[2];
    switch (A.kind) {
        case 1:
            return Point(A.C[0] + s * A.X[0] + t * A.Y[0],
                         A.C[1] + s * A.X[1] + t * A.Y[1],
                         A.C[2] + s * A.X[2] + t * A.Y[2]);
        case 2:
            return Point(A.C[0] + A.R * rx + t * A.Z[0],
                         A.C[1] + A.R * ry + t * A.Z[1],
                         A.C[2] + A.R * rz + t * A.Z[2]);
        case 3: {
            double ca = std::cos(A.r2), sa = std::sin(A.r2);
            return Point(A.C[0] + t * (ca * A.Z[0] + sa * rx),
                         A.C[1] + t * (ca * A.Z[1] + sa * ry),
                         A.C[2] + t * (ca * A.Z[2] + sa * rz));
        }
        case 4: {
            double ct = std::cos(t), st = std::sin(t);
            return Point(A.C[0] + A.R * (ct * rx + st * A.Z[0]),
                         A.C[1] + A.R * (ct * ry + st * A.Z[1]),
                         A.C[2] + A.R * (ct * rz + st * A.Z[2]));
        }
        default: {
            double ct = std::cos(t), st = std::sin(t);
            double rr = A.R + A.r2 * ct;
            return Point(A.C[0] + rr * rx + A.r2 * st * A.Z[0],
                         A.C[1] + rr * ry + A.r2 * st * A.Z[1],
                         A.C[2] + rr * rz + A.r2 * st * A.Z[2]);
        }
    }
}

static Vector analytic_normal_of(const AnalyticSrf& A, const Point& p) {
    double wx = p[0]-A.C[0], wy = p[1]-A.C[1], wz = p[2]-A.C[2];
    double x = wx*A.X[0]+wy*A.X[1]+wz*A.X[2];
    double y = wx*A.Y[0]+wy*A.Y[1]+wz*A.Y[2];
    double z = wx*A.Z[0]+wy*A.Z[1]+wz*A.Z[2];
    double rho = std::sqrt(x*x + y*y);
    auto mk = [&](double cx, double cy, double cz) {
        return Vector(cx*A.X[0]+cy*A.Y[0]+cz*A.Z[0],
                      cx*A.X[1]+cy*A.Y[1]+cz*A.Z[1],
                      cx*A.X[2]+cy*A.Y[2]+cz*A.Z[2]);
    };
    if (A.kind == 1) return Vector(A.Z[0], A.Z[1], A.Z[2]);
    if (rho < 1e-12) return Vector(0,0,0);
    double cu = x/rho, su = y/rho;
    if (A.kind == 2) return mk(cu, su, 0.0);
    if (A.kind == 3) { double ca = std::cos(A.r2), sa = std::sin(A.r2);
                       return mk(ca*cu, ca*su, -sa); }
    if (A.kind == 4) { double m = std::sqrt(rho*rho + z*z); if (m < 1e-12) return Vector(0,0,0);
                       return mk(x/m, y/m, z/m); }
    double cv = (rho - A.R), m = std::hypot(cv, z);
    if (m < 1e-12) return Vector(0,0,0);
    return mk(cv/m*cu, cv/m*su, z/m);
}

// Emit one BRep's faces into the writer; returns the per-component shell face-id groups
// (see write_file_step_brep for the conventions). All state is local, so several breps can
// be emitted into ONE writer (multi-solid colored files) -- entity ids simply continue and
// coincident vertices of different breps are deliberately NOT deduplicated across calls.
static std::vector<std::vector<int>> emit_brep_shells(StepWriter& w, const BRep& brep, double& diag_out) {

    // True outward orientation per face (shell-orientation propagation): STEP importers
    // orient MANIFOLD_SOLID_BREP shells from the ADVANCED_FACE same_sense flags; writing
    // .T. everywhere hands a mis-oriented shell to the reader's healing, which can close
    // it into the WRONG solid (box fuse sphere imported with the CUT's volume).
    std::vector<double> fsign = brep.face_outward_signs();

    // Model diagonal: drives both the export fit tolerance and the declared uncertainty.
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p2 : brep.m_vertices) {
            xmn=std::min(xmn,p2[0]); ymn=std::min(ymn,p2[1]); zmn=std::min(zmn,p2[2]);
            xmx=std::max(xmx,p2[0]); ymx=std::max(ymx,p2[1]); zmx=std::max(zmx,p2[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (!(diag > 0)) diag = 1.0;
    }
    // Tangency fidelity (CGM-style): section arcs meet iso circles TANGENTIALLY at
    // Steinmetz/spiric tangent points, so the reader's re-projected pcurve crosses the seam
    // wherever the written 3D curve deviates -- at 1e-5*diag the tor x tor lens wires turned
    // self-intersecting and OCCT replaced them with natural bounds (+6 full tori).
    double fit3d = diag * 1e-7;

    // OCCT-style export compression: dense degree-1 section polylines (pullback lifts,
    // 2000-4000 CVs -> 3 MB files and 4000-segment trim edges in Rhino) become tolerance-
    // fitted cubic B-splines -- the form OCCT's own booleans export. Deviation is checked
    // at (subsampled) polyline vertices and stays far inside the declared uncertainty, so
    // import watertightness is unaffected; the kernel's internal geometry is untouched.
    auto compress_curve = [&](const NurbsCurve& c, double tol) -> NurbsCurve {
        // Fit dense degree-1 polylines as compact cubics. Two tolerance tiers: the exact
        // tier (caller's tol, ~diag*1e-7) preserves analytic-quality geometry bit-tight;
        // marched SECTION polylines carry ~1e-3-scale sampling noise the exact tier can
        // never fit, so a looser section tier (diag*2e-5, still 50x under Rhino's 1e-3)
        // catches them -- without it 497 raw deg-1 curves (up to 147 CVs) shipped per
        // chair file: 2.1 MB, sliver-segment rendering, and Rhino trim rejection.
        if (!c.is_valid() || c.degree() != 1 || c.is_rational() || c.cv_count() <= 12) return c;
        std::vector<Point> pts;
        pts.reserve(c.cv_count());
        for (int i = 0; i < c.cv_count(); ++i) pts.push_back(c.get_cv(i));
        int step = std::max(1, (int)pts.size() / 200);
        for (double tt : {tol, std::max(tol, diag * 2e-5)}) {
            for (int ncv = 8; ncv <= 256 && ncv < (int)pts.size(); ncv *= 2) {
                NurbsCurve fit = NurbsCurve::create_fitted(pts, ncv, 3);
                if (!fit.is_valid()) continue;
                // pin the fit's endpoints EXACTLY onto the polyline's (clamped cubic:
                // endpoint == CV) -- the writer's vertex dedup keys on endpoint location,
                // and a tt-sized endpoint drift would split shared wire vertices.
                fit.set_cv(0, pts.front());
                fit.set_cv(fit.cv_count() - 1, pts.back());
                double worst = 0.0;
                for (size_t i = 0; i < pts.size() && worst <= tt; i += (size_t)step) {
                    double t = fit.closest_parameter(pts[i]);
                    worst = std::max(worst, fit.point_at(t).distance(pts[i]));
                }
                if (worst <= tt)
                    return fit;
            }
            if (tt >= std::max(tol, diag * 2e-5)) break;   // tiers exhausted
        }
        return c;
    };

    // 0. Surfaces (lazy, deduped): recognized quadrics are written as ANALYTIC entities
    // (TOROIDAL/SPHERICAL/CYLINDRICAL/CONICAL_SURFACE, PLANE) -- the dialect importers are
    // actually tested on. Periodicity of analytic surfaces is canonical, so pcurves may
    // legally run past 2*pi and the whole seam pathology disappears; Rhino keeps the trims
    // instead of rendering full tori.
    auto fnum = [](double v) -> std::string {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.15g", v);
        std::string r(buf);
        if (r.find('.') == std::string::npos && r.find('e') == std::string::npos
            && r.find('E') == std::string::npos && r.find("inf") == std::string::npos
            && r.find("nan") == std::string::npos) r += ".";
        return r;
    };
    std::vector<int> srf_step_id(brep.m_surfaces.size(), -1);
    std::vector<AnalyticSrf> srf_an(brep.m_surfaces.size());
    std::vector<char> srf_an_done(brep.m_surfaces.size(), 0);
    auto analytic_of = [&](int si) -> const AnalyticSrf& {
        if (!srf_an_done[si]) { srf_an[si] = detect_analytic_srf(brep.m_surfaces[si]); srf_an_done[si] = 1; }
        return srf_an[si];
    };
    auto surface_id = [&](int si) -> int {
        if (si < 0 || si >= (int)brep.m_surfaces.size()) return -1;
        if (srf_step_id[si] >= 0) return srf_step_id[si];
        const AnalyticSrf& A = analytic_of(si);
        if (A.kind == 0) {
            srf_step_id[si] = w.write_nurbs_surface(brep.m_surfaces[si]);
            return srf_step_id[si];
        }
        int loc = w.write_point(A.C[0], A.C[1], A.C[2]);
        int zd = w.write_raw("DIRECTION('',(" + fnum(A.Z[0]) + "," + fnum(A.Z[1]) + "," + fnum(A.Z[2]) + "))");
        int xd = w.write_raw("DIRECTION('',(" + fnum(A.X[0]) + "," + fnum(A.X[1]) + "," + fnum(A.X[2]) + "))");
        int ax = w.write_raw("AXIS2_PLACEMENT_3D('',#" + std::to_string(loc) + ",#" + std::to_string(zd)
                             + ",#" + std::to_string(xd) + ")");
        std::string ent;
        switch (A.kind) {
            case 1: ent = "PLANE('',#" + std::to_string(ax) + ")"; break;
            case 2: ent = "CYLINDRICAL_SURFACE('',#" + std::to_string(ax) + "," + fnum(A.R) + ")"; break;
            case 3: ent = "CONICAL_SURFACE('',#" + std::to_string(ax) + "," + fnum(A.R) + "," + fnum(A.r2) + ")"; break;
            case 4: ent = "SPHERICAL_SURFACE('',#" + std::to_string(ax) + "," + fnum(A.R) + ")"; break;
            default: ent = "TOROIDAL_SURFACE('',#" + std::to_string(ax) + "," + fnum(A.R) + "," + fnum(A.r2) + ")"; break;
        }
        srf_step_id[si] = w.write_raw(ent);
        return srf_step_id[si];
    };

    // Remap a pcurve into the analytic surface's canonical parameterization: sample its 3D
    // image, take unwrapped atan2 parameters, anchor the branch at the midpoint, compress.
    // Continuous chart lift: the unwrapped analytic (s,t) over the kernel chart rectangle.
    // The chart is simply connected, so this lift is unique up to ONE constant 2*pi*K per
    // surface -- snapping every pcurve's samples to it puts all pcurves of a surface in a
    // single shared branch frame: seam mates land exactly one period apart and every loop
    // closes in the written plane BY CONSTRUCTION. (All chain-order anchoring heuristics
    // failed here: greedy euler walks through seam junctions can wander whole periods with
    // zero junction error and zero net closure defect.)
    struct ChartLift { int N; double u0, du, v0, dv; std::vector<double> S, T; };
    std::map<int, ChartLift> chart_lifts;
    auto chart_lift_of = [&](int si) -> const ChartLift& {
        auto it = chart_lifts.find(si);
        if (it != chart_lifts.end()) return it->second;
        const AnalyticSrf& A = analytic_of(si);
        const NurbsSurface& S = brep.m_surfaces[si];
        auto duL = S.domain(0); auto dvL = S.domain(1);
        ChartLift L;
        L.N = 33;
        L.u0 = duL.first; L.du = (duL.second - duL.first) / (L.N - 1);
        L.v0 = dvL.first; L.dv = (dvL.second - dvL.first) / (L.N - 1);
        L.S.assign(L.N * L.N, 0.0); L.T.assign(L.N * L.N, 0.0);
        const double PI_ = 3.14159265358979323846;
        for (int j = 0; j < L.N; ++j)
            for (int i = 0; i < L.N; ++i) {
                double sv, tv; bool rok;
                analytic_params_of(A, S.point_at(L.u0 + L.du * i, L.v0 + L.dv * j), sv, tv, rok);
                double rs = 0, rt = 0; bool have = false;   // unwrap ref: left, else below
                if (i > 0)      { rs = L.S[j*L.N + i-1];     rt = L.T[j*L.N + i-1];     have = true; }
                else if (j > 0) { rs = L.S[(j-1)*L.N];       rt = L.T[(j-1)*L.N];       have = true; }
                if (have) {
                    if (!rok) sv = rs;
                    else sv -= 2*PI_ * std::round((sv - rs) / (2*PI_));
                    if (A.kind == 5) tv -= 2*PI_ * std::round((tv - rt) / (2*PI_));
                } else {
                    sv -= 2*PI_ * std::floor(sv / (2*PI_));
                    if (A.kind == 5) tv -= 2*PI_ * std::floor(tv / (2*PI_));
                }
                L.S[j*L.N + i] = sv; L.T[j*L.N + i] = tv;
            }
        return chart_lifts.emplace(si, std::move(L)).first->second;
    };
    // Raw remap: unwrapped canonical (s,t) samples of the pcurve's 3D image, NO branch anchor.
    auto remap_samples = [&](int si, const NurbsCurve& pc) -> std::vector<Point> {
        const AnalyticSrf& A = analytic_of(si);
        const NurbsSurface& S = brep.m_surfaces[si];
        auto pd = pc.domain();
        int n = std::min(std::max(pc.cv_count() * 2, 48), 512);
        std::vector<Point> st;
        st.reserve(n + 1);
        const double PI_ = 3.14159265358979323846;
        double prev_s = 0, prev_t = 0;
        bool have_prev = false;
        for (int i = 0; i <= n; ++i) {
            Point uv = pc.point_at(pd.first + (pd.second - pd.first) * i / n);
            Point p3 = S.point_at(uv[0], uv[1]);
            double sv, tv; bool rok;
            analytic_params_of(A, p3, sv, tv, rok);
            if (!rok && have_prev) sv = prev_s;              // pole/apex: hold the angle
            if (have_prev && A.kind >= 2) {
                while (sv - prev_s >  PI_) sv -= 2*PI_;
                while (sv - prev_s < -PI_) sv += 2*PI_;
            }
            if (have_prev && A.kind == 5) {
                while (tv - prev_t >  PI_) tv -= 2*PI_;
                while (tv - prev_t < -PI_) tv += 2*PI_;
            }
            prev_s = sv; prev_t = tv; have_prev = true;
            st.push_back(Point(sv, tv, 0.0));
        }
        if (A.kind >= 2 && st.size() >= 2) {
            const ChartLift& CL = chart_lift_of(si);
            size_t mid = st.size() / 2;
            Point uvm = pc.point_at(pd.first + (pd.second - pd.first) * (double)mid / n);
            int ii = (int)std::round((uvm[0] - CL.u0) / CL.du);
            int jj = (int)std::round((uvm[1] - CL.v0) / CL.dv);
            ii = std::min(std::max(ii, 0), CL.N - 1);
            jj = std::min(std::max(jj, 0), CL.N - 1);
            double gs = CL.S[jj*CL.N + ii], gt = CL.T[jj*CL.N + ii];
            double sh_s = 2*PI_ * std::round((gs - st[mid][0]) / (2*PI_));
            double sh_t = (A.kind == 5) ? 2*PI_ * std::round((gt - st[mid][1]) / (2*PI_)) : 0.0;
            if (sh_s != 0.0 || sh_t != 0.0)
                for (auto& q : st) q = Point(q[0] + sh_s, q[1] + sh_t, q[2]);
        }
        return st;
    };
    auto fit_samples = [&](int si, std::vector<Point> st, const NurbsCurve& fallback) -> NurbsCurve {
        const NurbsSurface& S = brep.m_surfaces[si];
        auto du = S.domain(0); auto dv = S.domain(1);
        double span = std::max(du.second - du.first, dv.second - dv.first);
        NurbsCurve out = NurbsCurve::create(false, 1, st);
        if (!out.is_valid()) return compress_curve(fallback, span * 5e-5);
        return compress_curve(out, 2e-4 * 6.283185307179586);
    };
    auto remap_pcurve = [&](int si, const NurbsCurve& pc) -> NurbsCurve {
        const AnalyticSrf& A = analytic_of(si);
        const NurbsSurface& S = brep.m_surfaces[si];
        auto du = S.domain(0); auto dv = S.domain(1);
        double span = std::max(du.second - du.first, dv.second - dv.first);
        if (A.kind == 0) return compress_curve(pc, span * 5e-5);
        return fit_samples(si, remap_samples(si, pc), pc);
    };

    // 1. Topology vertices (lazy, deduped by topology index).
    std::vector<int> vert_step_id(brep.m_topology_vertices.size(), -1);
    // One VERTEX_POINT per LOCATION, not per kernel vertex: boolean results carry duplicate
    // vertices at seam/section junctions (box_cut_tor: 20 entities on 10 locations), and OCCT
    // builds wires from SHARED VERTEX TOPOLOGY -- with duplicates the loop arrives as loose
    // edges, so the reader re-chains them by 3D proximity and wanders whole periods across
    // the torus chart no matter how perfect the pcurves are.
    std::map<std::tuple<long long, long long, long long>, int> vert_by_loc;
    // Loop-junction anchors for CLOSED edges (filled in PASS A). A closed border circle's
    // kernel vertex can sit anywhere on it (co_refine rebuilds curves with arbitrary starts:
    // box_cut_cyl's bore circles start at theta=-pi/2 while the wire junctions with the seam
    // at theta=0) -- then the written wire is vertex-DISCONNECTED (the circle is an island
    // self-loop) and importers re-chain by proximity. Rotating the written 3D curve to start
    // at the junction and using the junction vertex makes the wire vertex-connected.
    std::map<int, Point> edge_anchor;
    // Canonical direction per edge = the FIRST chained occurrence's traversal (Parasolid
    // lesson: ONE constructive source of truth). The written 3D curve, pullback pcurves,
    // vertices and every ORIENTED_EDGE sense all derive from it -- independent geometric
    // re-derivations (stored-direction vs first-trim vs written-W) disagreed pairwise and
    // produced annulus wires with both circles traversed the same way (prim_cylinder wall
    // imported 30.29 of 42.41 with a synthetic bridging line).
    std::map<int, std::pair<int, char>> edge_canon;   // ei -> (trim, from_first)
    auto vertex_id = [&](int tv) -> int {
        if (tv < 0 || tv >= (int)brep.m_topology_vertices.size()) return -1;
        if (vert_step_id[tv] >= 0) return vert_step_id[tv];
        int pi = brep.m_topology_vertices[tv].point_index;
        if (pi < 0 || pi >= (int)brep.m_vertices.size()) return -1;
        const Point& p = brep.m_vertices[pi];
        double q = diag > 0 ? diag * 1e-7 : 1e-9;
        std::tuple<long long, long long, long long> key(
            (long long)std::llround(p[0] / q),
            (long long)std::llround(p[1] / q),
            (long long)std::llround(p[2] / q));
        auto it = vert_by_loc.find(key);
        if (it != vert_by_loc.end()) { vert_step_id[tv] = it->second; return it->second; }
        int pt = w.write_point(p[0], p[1], p[2]);
        vert_step_id[tv] = w.write_raw("VERTEX_POINT('',#" + std::to_string(pt) + ")");
        vert_by_loc[key] = vert_step_id[tv];
        return vert_step_id[tv];
    };
    auto vertex_at_point = [&](const Point& p) -> int {
        double q = diag > 0 ? diag * 1e-7 : 1e-9;
        std::tuple<long long, long long, long long> key(
            (long long)std::llround(p[0] / q),
            (long long)std::llround(p[1] / q),
            (long long)std::llround(p[2] / q));
        auto it = vert_by_loc.find(key);
        if (it != vert_by_loc.end()) return it->second;
        int pt = w.write_point(p[0], p[1], p[2]);
        int vid = w.write_raw("VERTEX_POINT('',#" + std::to_string(pt) + ")");
        vert_by_loc[key] = vid;
        return vid;
    };

    std::vector<std::vector<std::pair<int,char>>> loop_chain(brep.m_loops.size());
    std::vector<char> trim_sense(brep.m_trims.size(), 1);

    // Does this trim traverse its edge's 3D curve forward? Open curves: compare the trim's
    // traversal START (surface-mapped pcurve end) with the curve ends. Closed curves carry
    // no direction in their endpoints: compare mid tangents (trim tangent vs curve tangent
    // at the curve parameter closest to the trim midpoint).
    // Reference geometry for sense decisions = the edge's FIRST trim's pcurve image -- the
    // exact curve edge_id writes. The stored kernel edge curve is UNRELIABLE here: co_refine
    // and sew leave rep-curves decoupled from trim extents (a tangent-point stub's edge can
    // carry a LONG arc from elsewhere), and senses computed against it come out scrambled.
    auto edge_first_trim = [&](int ei, int si_want, const NurbsCurve** pc_out, const NurbsSurface** srf_out) -> bool {
        const BRepEdge& E = brep.m_topology_edges[ei];
        for (int ti : E.trim_indices) {
            if (ti < 0 || ti >= (int)brep.m_trims.size()) continue;
            const BRepTrim& T2 = brep.m_trims[ti];
            int li = T2.loop_index;
            if (li < 0 || li >= (int)brep.m_loops.size()) continue;
            int fi2 = brep.m_loops[li].face_index;
            if (fi2 < 0 || fi2 >= (int)brep.m_faces.size()) continue;
            int si = brep.m_faces[fi2].surface_index;
            if (si < 0 || si >= (int)brep.m_surfaces.size()) continue;
            if (si_want >= 0 && si != si_want) continue;
            if (T2.curve_2d_index < 0 || T2.curve_2d_index >= (int)brep.m_curves_2d.size()) continue;
            const NurbsCurve& pc2 = brep.m_curves_2d[T2.curve_2d_index];
            if (!pc2.is_valid()) continue;
            *pc_out = &pc2;
            *srf_out = &brep.m_surfaces[si];
            return true;
        }
        return false;
    };
    // trim_pc_srf: resolve a trim's pcurve + surface.
    auto trim_pc_srf = [&](int ti, const NurbsCurve** pc_out, const NurbsSurface** srf_out) -> bool {
        if (ti < 0 || ti >= (int)brep.m_trims.size()) return false;
        const BRepTrim& T2 = brep.m_trims[ti];
        int li = T2.loop_index;
        if (li < 0 || li >= (int)brep.m_loops.size()) return false;
        int fi2 = brep.m_loops[li].face_index;
        if (fi2 < 0 || fi2 >= (int)brep.m_faces.size()) return false;
        int si = brep.m_faces[fi2].surface_index;
        if (si < 0 || si >= (int)brep.m_surfaces.size()) return false;
        if (T2.curve_2d_index < 0 || T2.curve_2d_index >= (int)brep.m_curves_2d.size()) return false;
        const NurbsCurve& pc2 = brep.m_curves_2d[T2.curve_2d_index];
        if (!pc2.is_valid()) return false;
        *pc_out = &pc2;
        *srf_out = &brep.m_surfaces[si];
        return true;
    };
    auto trim_forward = [&](const BRepTrim& T, const NurbsSurface& srf, bool trav_from_first) -> bool {
        if (T.curve_2d_index < 0 || T.curve_2d_index >= (int)brep.m_curves_2d.size()) return true;
        if (T.edge_index < 0 || T.edge_index >= (int)brep.m_topology_edges.size()) return true;
        auto itc = edge_canon.find(T.edge_index);
        if (itc == edge_canon.end()) return true;
        const NurbsCurve* pc0 = nullptr;
        const NurbsSurface* S0 = nullptr;
        if (!trim_pc_srf(itc->second.first, &pc0, &S0)) return true;
        bool canon_ff = itc->second.second != 0;
        const NurbsCurve& pc = brep.m_curves_2d[T.curve_2d_index];
        if (!pc.is_valid()) return true;
        auto pd = pc.domain();
        auto qd = pc0->domain();
        auto img = [&](const NurbsCurve& c, const NurbsSurface& S, double t) {
            Point uv = c.point_at(t);
            return S.point_at(uv[0], uv[1]);
        };
        // canonical TRAVERSAL start/end (not raw stored direction)
        Point R0 = img(*pc0, *S0, canon_ff ? qd.first : qd.second);
        Point R1 = img(*pc0, *S0, canon_ff ? qd.second : qd.first);
        Point A = img(pc, srf, trav_from_first ? pd.first : pd.second);
        if (R0.distance(R1) > diag * 1e-7)          // open reference: endpoint decides
            return A.distance(R0) <= A.distance(R1);
        // closed reference: march both traversals briefly and compare sample ordering
        const int M = 128;
        Point B = img(pc, srf,
                      trav_from_first ? pd.first + (pd.second - pd.first) * 0.02
                                      : pd.second - (pd.second - pd.first) * 0.02);
        int iA = 0, iB = 0;
        double dA = 1e300, dB = 1e300;
        for (int i = 0; i < M; ++i) {
            double f = (double)i / M;
            double t = canon_ff ? qd.first + (qd.second - qd.first) * f
                                : qd.second - (qd.second - qd.first) * f;
            Point q = img(*pc0, *S0, t);
            double da = A.distance(q), db = B.distance(q);
            if (da < dA) { dA = da; iA = i; }
            if (db < dB) { dB = db; iB = i; }
        }
        return ((iB - iA + M) % M) < M / 2;
    };

    // 2. Shared edges (lazy): EDGE_CURVE whose geometry is a SURFACE_CURVE (or SEAM_CURVE
    // when both trims lie on the SAME face) carrying the edge's 3D curve PLUS its pcurves.
    // Importers need the pcurves to build wires on PERIODIC surfaces: without them a torus
    // face bounded only by 3D seam curves imports with its bounds dropped at 2x area, and
    // a cylinder wall mis-heals.
    // One written arc of an edge: an exact CIRCLE (ref direction at the arc start, angular
    // range [0, ext]) or a spline over its own domain. f0/f1 = the fraction of the FULL
    // edge traversal this arc covers (0..1 for unsplit edges) -- it maps the stored pcurve
    // of kind-0 fallbacks onto the arc.
    struct ArcSpec {
        bool circle = false;
        Point cc{0, 0, 0};
        Vector cref{1, 0, 0}, cax{0, 0, 1};
        double crad = 0;
        NurbsCurve spline;
        double a = 0, b = 1;
        double f0 = 0, f1 = 1;
        int vs = -1, ve = -1;
    };
    // Decimate a SameParameter deg-1 pullback polyline: keep every s-th node while the
    // surface image of the linear interpolant stays within diag*2e-6 of the dense nodes.
    // Knots remain written-curve parameters, so the SameParameter binding is untouched.
    // Near-straight pcurves (plane borders, chair patch edges) collapse to 2-3 points --
    // the fixed dense resampling was 86% of the 2.7MB chair file.
    auto decimate_pullback = [&](std::vector<Point>& uvp, std::vector<double>& kts,
                                 auto&& fwd, const std::vector<Point>& p3) {
        int n = (int)uvp.size();
        if (n < 9 || (int)kts.size() != n || (int)p3.size() != n) return;
        double tol = diag * 2e-6;
        for (int s : {128, 96, 64, 48, 32, 16, 8, 4, 2}) {
            if ((n - 1) % s) continue;
            bool ok = true;
            for (int i = 0; i < n && ok; ++i) {
                int j = (i / s) * s;
                if (i == j) continue;
                double f = (double)(i - j) / (double)s;
                double us = uvp[j][0] + (uvp[j + s][0] - uvp[j][0]) * f;
                double vs = uvp[j][1] + (uvp[j + s][1] - uvp[j][1]) * f;
                if (fwd(us, vs).distance(p3[i]) > tol) ok = false;
            }
            if (!ok) continue;
            std::vector<Point> u2;
            std::vector<double> k2;
            for (int i = 0; i < n; i += s) { u2.push_back(uvp[i]); k2.push_back(kts[i]); }
            uvp.swap(u2);
            kts.swap(k2);
            return;
        }
    };
    auto emit_arc_edge = [&](int ei, const ArcSpec& AS) -> int {
        const BRepEdge& E = brep.m_topology_edges[ei];
        auto fnum = [](double x) {
            char b[64];
            std::snprintf(b, sizeof b, "%.15g", x == 0.0 ? 0.0 : x);
            std::string r2s(b);
            if (r2s.find('.') == std::string::npos && r2s.find('e') == std::string::npos
                && r2s.find('E') == std::string::npos) r2s += ".";
            return r2s;
        };
        int cid;
        if (AS.circle) {
            int pc_ = w.write_raw("CARTESIAN_POINT('',(" + fnum(AS.cc[0]) + "," + fnum(AS.cc[1]) + "," + fnum(AS.cc[2]) + "))");
            int dz = w.write_raw("DIRECTION('',(" + fnum(AS.cax[0]) + "," + fnum(AS.cax[1]) + "," + fnum(AS.cax[2]) + "))");
            int dx = w.write_raw("DIRECTION('',(" + fnum(AS.cref[0]) + "," + fnum(AS.cref[1]) + "," + fnum(AS.cref[2]) + "))");
            int ax = w.write_raw("AXIS2_PLACEMENT_3D('',#" + std::to_string(pc_) + ",#" + std::to_string(dz) + ",#" + std::to_string(dx) + ")");
            cid = w.write_raw("CIRCLE('',#" + std::to_string(ax) + "," + fnum(AS.crad) + ")");
        } else {
            cid = w.write_nurbs_curve(AS.spline);
        }
        if (cid < 0) return -1;
        const double W_a = AS.a, W_b = AS.b;
        // Evaluate the WRITTEN geometry at its own parameter (SameParameter source).
        auto W_eval = [&](double t) -> Point {
            if (!AS.circle) return AS.spline.point_at(t);
            Vector Yd(AS.cax[1] * AS.cref[2] - AS.cax[2] * AS.cref[1],
                      AS.cax[2] * AS.cref[0] - AS.cax[0] * AS.cref[2],
                      AS.cax[0] * AS.cref[1] - AS.cax[1] * AS.cref[0]);
            double cs = std::cos(t), sn = std::sin(t);
            return Point(AS.cc[0] + AS.crad * (cs * AS.cref[0] + sn * Yd[0]),
                         AS.cc[1] + AS.crad * (cs * AS.cref[1] + sn * Yd[1]),
                         AS.cc[2] + AS.crad * (cs * AS.cref[2] + sn * Yd[2]));
        };
        std::vector<int> pcs;
        std::vector<int> pc_faces;
        std::vector<char> pc_fwd;
        for (int ti : E.trim_indices) {
            if (ti < 0 || ti >= (int)brep.m_trims.size()) continue;
            const BRepTrim& T = brep.m_trims[ti];
            int li = T.loop_index;
            if (li < 0 || li >= (int)brep.m_loops.size()) continue;
            int fi2 = brep.m_loops[li].face_index;
            if (fi2 < 0 || fi2 >= (int)brep.m_faces.size()) continue;
            int si = brep.m_faces[fi2].surface_index;
            int sid = surface_id(si);
            if (sid < 0 || T.curve_2d_index < 0 || T.curve_2d_index >= (int)brep.m_curves_2d.size()) continue;
            const NurbsSurface& S = brep.m_surfaces[si];
            // SameParameter pcurve: pull back the WRITTEN 3D geometry at its OWN parameters
            // (t_i uniform in the written domain, uv_i = analytic inverse, deg-1 with knots
            // = t_i). OCCT's reader binds ALL of an edge's representations to ONE parameter
            // range; independently-fitted pcurves (own chord domains) fail its SameParameter
            // check, so it silently RE-PROJECTED every edge from the 3D curve -- pcurves
            // never mattered, and any face whose projection is unstable (arcs grazing the
            // v=0 branch line, tangency cusps) imported as its full periodic surface.
            // Planes (kind 1) take the same exact pullback -- their inverse is trivial and
            // phase-correct for split arcs, where the stored full-loop pcurve cannot serve.
            NurbsCurve uvc;
            const AnalyticSrf& A_e = analytic_of(si);
            if (A_e.kind >= 1) {
                const double PI_ = 3.14159265358979323846;
                const double TWOPI = 6.283185307179586;
                int n = 128;
                std::vector<Point> uvp;
                std::vector<double> kts;
                std::vector<Point> w3;
                uvp.reserve(n + 1);
                kts.reserve(n + 1);
                w3.reserve(n + 1);
                double ps = 0, pt = 0;
                bool hp = false;
                for (int i2 = 0; i2 <= n; ++i2) {
                    double t = W_a + (W_b - W_a) * i2 / n;
                    Point Pw = W_eval(t);
                    double sv, tv; bool rok;
                    analytic_params_of(A_e, Pw, sv, tv, rok);
                    if (!rok && hp) sv = ps;
                    if (hp && A_e.kind >= 2) {          // planes carry no period to unwrap
                        while (sv - ps >  PI_) sv -= TWOPI;
                        while (sv - ps < -PI_) sv += TWOPI;
                        if (A_e.kind == 5) {
                            while (tv - pt >  PI_) tv -= TWOPI;
                            while (tv - pt < -PI_) tv += TWOPI;
                        }
                    }
                    ps = sv; pt = tv; hp = true;
                    uvp.push_back(Point(sv, tv, 0.0));
                    kts.push_back(t);
                    w3.push_back(Pw);
                }
                if (A_e.kind >= 2) {
                    // branch anchor at the MID sample: curve STARTS often sit exactly on the
                    // seam (junction-rotated circles), where both branch choices are 3D-identical
                    // and the nearest-sample match picks arbitrarily -- prim_cylinder's bottom
                    // circle anchored to u=0 and unwrapped down to [-2pi,0], one period below its
                    // loop. The interior midpoint is 3D-unique, so its branch is unambiguous.
                    std::vector<Point> st = remap_samples(si, brep.m_curves_2d[T.curve_2d_index]);
                    const NurbsCurve& pcs_ = brep.m_curves_2d[T.curve_2d_index];
                    auto pdd = pcs_.domain();
                    size_t mid = uvp.size() / 2;
                    Point Wm = W_eval(W_a + (W_b - W_a) * (double)mid / (double)n);
                    double bd = 1e300; size_t bj = 0;
                    size_t M = st.size() > 1 ? st.size() - 1 : 1;
                    for (size_t j = 0; j < st.size(); ++j) {
                        Point uv = pcs_.point_at(pdd.first + (pdd.second - pdd.first) * j / (double)M);
                        double dd = Wm.distance(S.point_at(uv[0], uv[1]));
                        if (dd < bd) { bd = dd; bj = j; }
                    }
                    double sh_s, sh_t = 0.0;
                    if (!st.empty() && bd < diag * 1e-4) {
                        sh_s = TWOPI * std::round((st[bj][0] - uvp[mid][0]) / TWOPI);
                        if (A_e.kind == 5)
                            sh_t = TWOPI * std::round((st[bj][1] - uvp[mid][1]) / TWOPI);
                    } else {   // decoupled stored pcurve: fall back to mid-sample cell snap
                        double sm = uvp[mid][0];
                        sh_s = -TWOPI * std::floor(sm / TWOPI);
                        if (A_e.kind == 5) {
                            double tm = uvp[mid][1];
                            sh_t = -TWOPI * std::floor(tm / TWOPI);
                        }
                    }
                    if (sh_s != 0.0 || sh_t != 0.0)
                        for (auto& q : uvp) q = Point(q[0] + sh_s, q[1] + sh_t, q[2]);
                }
                decimate_pullback(uvp, kts,
                                  [&](double us, double vs) { return analytic_eval(A_e, us, vs); },
                                  w3);
                std::vector<double> w1(uvp.size(), 1.0);
                std::vector<int> mults(uvp.size(), 1);
                mults.front() = 2; mults.back() = 2;
                uvc = NurbsCurve::create_from_parameters(uvp, w1, kts, mults, 1, false);
            }
            if (!uvc.is_valid()) {
                uvc = remap_pcurve(si, brep.m_curves_2d[T.curve_2d_index]);
                // co-direct fallback pcurves with the written curve; a split arc compares
                // its OWN fraction window [f0,f1] of the stored full-edge pcurve
                auto pd0 = brep.m_curves_2d[T.curve_2d_index].domain();
                const NurbsCurve& pc0 = brep.m_curves_2d[T.curve_2d_index];
                auto img = [&](double f) {
                    Point q = pc0.point_at(pd0.first + (pd0.second - pd0.first) * f);
                    return S.point_at(q[0], q[1]);
                };
                double g0 = AS.f0 + (AS.f1 - AS.f0) * 0.05;
                double g1 = AS.f0 + (AS.f1 - AS.f0) * 0.95;
                double err_fwd = img(g0).distance(W_eval(W_a + (W_b - W_a) * 0.05))
                               + img(g1).distance(W_eval(W_a + (W_b - W_a) * 0.95));
                double err_rev = img(1.0 - g0).distance(W_eval(W_a + (W_b - W_a) * 0.05))
                               + img(1.0 - g1).distance(W_eval(W_a + (W_b - W_a) * 0.95));
                if (err_rev < err_fwd) uvc.reverse();
                // SameParameter domain alignment for the fallback (kind-0) path: without it
                // the reader rejects the pcurve and re-projects onto the closed B-spline
                // surface, wandering across the seam (freeform_blob imported as a sliver).
                {
                    int n2 = 96;
                    std::vector<Point> up;
                    std::vector<double> kt;
                    auto udm = uvc.domain();
                    for (int i2 = 0; i2 <= n2; ++i2) {
                        double f = AS.f0 + (AS.f1 - AS.f0) * (double)i2 / n2;
                        up.push_back(uvc.point_at(udm.first + (udm.second - udm.first) * f));
                        kt.push_back(W_a + (W_b - W_a) * (double)i2 / n2);
                    }
                    std::vector<Point> p3b(up.size());
                    for (size_t i3 = 0; i3 < up.size(); ++i3)
                        p3b[i3] = S.point_at(up[i3][0], up[i3][1]);
                    decimate_pullback(up, kt,
                                      [&](double us, double vs) { return S.point_at(us, vs); },
                                      p3b);
                    std::vector<double> w1(up.size(), 1.0);
                    std::vector<int> mu(up.size(), 1);
                    mu.front() = 2; mu.back() = 2;
                    NurbsCurve u2 = NurbsCurve::create_from_parameters(up, w1, kt, mu, 1, false);
                    if (u2.is_valid()) uvc = u2;
                }
            }
            if (std::getenv("SESSION_STEP_DBG")) {
                auto ud = uvc.domain();
                Point u0 = uvc.point_at(ud.first), u1 = uvc.point_at(ud.second);
                double s25 = 0, t25 = 0; bool r25;
                analytic_params_of(analytic_of(si), W_eval(W_a + (W_b - W_a) * 0.25), s25, t25, r25);
                std::fprintf(stderr, "[EDGE25] circ=%d q(%.3f,%.3f)\n", AS.circle ? 1 : 0, s25, t25);
                std::fprintf(stderr, "[EDGE] ei=%d ti=%d canon=%d cff=%d Wdir(%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f) pc(%.2f,%.2f)->(%.2f,%.2f) valid=%d\n",
                             ei, ti, edge_canon.count(ei) ? edge_canon[ei].first : -1,
                             edge_canon.count(ei) ? (int)edge_canon[ei].second : -1,
                             W_eval(W_a)[0], W_eval(W_a)[1], W_eval(W_a)[2],
                             W_eval(W_b)[0], W_eval(W_b)[1], W_eval(W_b)[2],
                             u0[0], u0[1], u1[0], u1[1], uvc.is_valid() ? 1 : 0);
            }
            int pc = w.write_pcurve(sid, uvc);
            if (pc >= 0) { pcs.push_back(pc); pc_faces.push_back(fi2); pc_fwd.push_back(trim_sense[ti]); }
        }
        int geom = cid;
        if (pcs.size() == 2) {
            bool same_face = (pc_faces[0] == pc_faces[1]);
            // SEAM_CURVE: both pcurves reference the SAME surface, so only their ORDER tells
            // the importer which side each occurrence uses. OCCT's own writer puts the
            // REVERSED-occurrence pcurve FIRST (verified on its box_cut_tor reference file:
            // .T. walks pc2's side, .F. walks pc1's) -- and the order only becomes load-
            // bearing next to CLOSED border circles, where vertex connectivity (start==end,
            // one vertex) cannot anchor the branch: with .T.-first, box_cut_cyl's bore
            // accumulated u in [0, 3.5*pi] and failed BRepCheck.
            // Reader binding verified on prim_cylinder wall: pc1 <-> the FORWARD (.T.)
            // occurrence. (The earlier .F.-first flip was never load-bearing -- the fix that
            // mattered then was closed-edge junction rotation.)
            if (same_face && !pc_fwd[0] && pc_fwd[1]) {
                std::swap(pcs[0], pcs[1]);
                std::swap(pc_fwd[0], pc_fwd[1]);
            }
            const char* kind = same_face ? "SEAM_CURVE" : "SURFACE_CURVE";
            geom = w.write_raw(std::string(kind) + "('',#" + std::to_string(cid) + ",(#"
                               + std::to_string(pcs[0]) + ",#" + std::to_string(pcs[1]) + "),.PCURVE_S1.)");
        } else if (pcs.size() == 1) {
            geom = w.write_raw("SURFACE_CURVE('',#" + std::to_string(cid) + ",(#"
                               + std::to_string(pcs[0]) + "),.PCURVE_S1.)");
        }
        return w.write_raw("EDGE_CURVE('',#" + std::to_string(AS.vs) + ",#" + std::to_string(AS.ve)
                           + ",#" + std::to_string(geom) + ",.T.)");
    };

    // Closed edges (built start == end) are SPLIT into TWO arcs sharing a new halfway
    // vertex: Rhino refuses to trim a face whose loop contains ONE closed EDGE_CURVE with
    // identical start/end VERTEX_POINT (cylinder caps, sphere section circles, spiric
    // loops all displayed as OPEN/untrimmed faces), while two vertex-bounded arcs import
    // everywhere. The second id is -1 for open (unsplit) edges.
    std::vector<std::pair<int, int>> edge_step_id(brep.m_topology_edges.size(), {-1, -1});
    auto edge_id = [&](int ei) -> std::pair<int, int> {
        const std::pair<int, int> none(-1, -1);
        if (ei < 0 || ei >= (int)brep.m_topology_edges.size()) return none;
        if (edge_step_id[ei].first >= 0) return edge_step_id[ei];
        const BRepEdge& E = brep.m_topology_edges[ei];
        if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)brep.m_curves_3d.size()) return none;
        // Written 3D geometry = the FIRST trim's pcurve image, NOT the stored edge curve.
        // co_refine/sew leave edge rep-curves decoupled from trim extents (cyl_common_cyl2's
        // tangent-point stub e#243: trims describe a 0.012-rad stub, the stored 3D curve a
        // LONG arc from the other tangent region, vertices at ITS ends) -- written verbatim
        // that poisons SameParameter and BRepCheck reports UnorientableShape. The trim image
        // guarantees 3D curve == pcurve == wire junctions; vertices are its endpoints.
        NurbsCurve c3built;
        int si0 = -1;
        const NurbsCurve* pcs0 = nullptr;
        const NurbsSurface* srf0 = nullptr;
        bool canon_ff = true;
        {
            auto itc = edge_canon.find(ei);
            if (itc != edge_canon.end() && trim_pc_srf(itc->second.first, &pcs0, &srf0)) {
                canon_ff = itc->second.second != 0;
                const BRepTrim& Tc = brep.m_trims[itc->second.first];
                int lic = Tc.loop_index;
                si0 = brep.m_faces[brep.m_loops[lic].face_index].surface_index;
            } else {
                for (int ti : E.trim_indices) {
                    if (!trim_pc_srf(ti, &pcs0, &srf0)) { pcs0 = nullptr; continue; }
                    const BRepTrim& Tc = brep.m_trims[ti];
                    si0 = brep.m_faces[brep.m_loops[Tc.loop_index].face_index].surface_index;
                    break;
                }
            }
            if (pcs0) {
                auto pd = pcs0->domain();
                int n = std::min(std::max(pcs0->cv_count() * 2, 64), 512);
                std::vector<Point> ps;
                ps.reserve(n + 1);
                for (int i = 0; i <= n; ++i) {
                    double f = (double)i / n;
                    double t = canon_ff ? pd.first + (pd.second - pd.first) * f
                                        : pd.second - (pd.second - pd.first) * f;
                    Point uv = pcs0->point_at(t);
                    ps.push_back(srf0->point_at(uv[0], uv[1]));
                }
                c3built = NurbsCurve::create(false, 1, ps);
            }
            if (!pcs0 || !c3built.is_valid())
                c3built = brep.m_curves_3d[E.curve_3d_index];
            else {
                // Prefer the KERNEL curve when it geometrically matches the trim image:
                // exact section curves (spiric arcs, analytic circles) then reach the file
                // verbatim. A 256-sample refit deviates ~diag*1e-5 BETWEEN samples, and for
                // arcs grazing the tube seam at TANGENT POINTS that deviation flips the
                // reader's re-projected pcurve across the seam -- the UV wire turns
                // self-intersecting, OCCT drops it and restores natural bounds (tor_cut_tor2
                // imported +6 FULL tori). Decoupled rep-curves fail the match and keep the
                // trim-image rebuild.
                const NurbsCurve& ck = brep.m_curves_3d[E.curve_3d_index];
                if (ck.is_valid()) {
                    auto db2 = c3built.domain();
                    // Strict on purpose: lenient matching (diag*2e-5 for rational/deg>=2)
                    // was tried to adopt imported-brep reader curves verbatim, but their
                    // ~1e-5 projection-fit endpoints land past the diag*1e-7 vertex dedup,
                    // wires lose shared vertices and chairs fell from SOLIDS 2 to 0. The
                    // chair bloat was never the 3D curves (compress_curve already fits
                    // 16-CV cubics) -- it was the dense fallback pcurves, fixed by
                    // decimate_pullback.
                    double tolm = diag * 1e-6;
                    bool same = true;
                    for (int q = 0; q <= 8 && same; ++q) {
                        Point pb = c3built.point_at(db2.first + (db2.second - db2.first) * q / 8.0);
                        Point pk = ck.point_at(ck.closest_parameter(pb));
                        if (pb.distance(pk) > tolm) same = false;
                    }
                    if (same) {
                        auto dk = ck.domain();
                        Point b0 = c3built.point_at(db2.first);
                        Point k0 = ck.point_at(dk.first), k1 = ck.point_at(dk.second);
                        // CLOSED curves: keep the trim image -- the kernel curve starts at an
                        // arbitrary point and cannot be endpoint-oriented, so adopting it
                        // as-is destroyed the canonical direction AND the junction start
                        // (prim_cylinder's top circle written CCW against a CW traversal;
                        // closed iso edges become exact CIRCLEs anyway).
                        if (k0.distance(k1) >= diag * 1e-9) {
                            if (b0.distance(k0) <= b0.distance(k1)) {
                                c3built = ck;
                            } else {
                                NurbsCurve rk = ck;
                                if (rk.reverse()) c3built = rk;
                            }
                        }
                    }
                }
            }
        }
        bool rotated = false;
        Point anchor;
        // written-geometry capture for SameParameter pcurve pullback
        bool W_circle = false;
        Point W_cc(0, 0, 0);
        Vector W_cref(1, 0, 0), W_cax(0, 0, 1);
        double W_crad = 0, W_a = 0, W_b = 1;
        NurbsCurve W_spline;
        {
            auto d3o = c3built.domain();
            Point A0 = c3built.point_at(d3o.first), B0 = c3built.point_at(d3o.second);
            auto ita = edge_anchor.find(ei);
            if (ita != edge_anchor.end() && A0.distance(B0) < diag * 1e-7
                && A0.distance(ita->second) > diag * 1e-6) {
                // canonical traversal normally starts at the junction already; this path
                // only fires when the canonical trim's own pcurve starts elsewhere
                anchor = ita->second;
                rotated = true;
                int n = 256;
                std::vector<Point> ps;
                ps.reserve(n + 1);
                if (pcs0) {
                    // Resample through the PCURVE, not the deg-1 c3built: pcurve chords stay
                    // ON the surface (a sphere-parallel's UV chord IS the parallel), while
                    // c3built's 3D chords cut inside it -- the rotated resample then baked
                    // mid-chord points in as data, compress_curve could no longer fit a
                    // smooth curve through them, and the written 65-gon (sag ~2e-3) made
                    // Rhino drop the trims of every face it bounded (box x sph z-discs).
                    auto pdr = pcs0->domain();
                    auto at_f = [&](double f) {
                        double t = canon_ff ? pdr.first + (pdr.second - pdr.first) * f
                                            : pdr.second - (pdr.second - pdr.first) * f;
                        Point uv = pcs0->point_at(t);
                        return srf0->point_at(uv[0], uv[1]);
                    };
                    double f0 = 0, bd = 1e300;
                    for (int i = 0; i < n; ++i) {
                        double d = at_f((double)i / n).distance(anchor);
                        if (d < bd) { bd = d; f0 = (double)i / n; }
                    }
                    for (int i = 0; i <= n; ++i) {
                        double f = f0 + (double)i / n;
                        if (f > 1.0) f -= 1.0;
                        ps.push_back(at_f(f));
                    }
                } else {
                    double t0 = c3built.closest_parameter(anchor);
                    double len = d3o.second - d3o.first;
                    for (int i = 0; i <= n; ++i) {
                        double t = t0 + len * i / n;
                        if (t > d3o.second) t -= len;
                        ps.push_back(c3built.point_at(t));
                    }
                }
                ps.front() = anchor;
                ps.back() = anchor;
                NurbsCurve rot = NurbsCurve::create(false, 1, ps);
                if (rot.is_valid()) c3built = rot;
                else rotated = false;
            }
            // ISO-LINE edges on analytic surfaces are written as exact CIRCLEs. A fitted
            // near-iso spline oscillates across the iso (the torus outer equator v=0 is the
            // tube-angle BRANCH LINE: atan2 flips 0 <-> 2*pi with the sign of z), so the
            // reader's re-projected pcurve zigzags across the period and the wire is dropped
            // as self-intersecting -- tor_cut_tor2's six lens faces imported as FULL tori.
            // OCCT's own writer emits CIRCLE here (verified in its reference file).
            // The pcurve must live in si0's chart: an edge whose FIRST trim sits on the
            // mate face (plane) remapped plane-UV coordinates through the sphere chart,
            // so the sphere-parallel z-disc circles were never recognized as ISO circles.
            const NurbsCurve* pcI = nullptr;
            const NurbsSurface* srfI = nullptr;
            if (si0 >= 0 && edge_first_trim(ei, si0, &pcI, &srfI)) {
                const AnalyticSrf& A = analytic_of(si0);
                if (A.kind >= 2) {
                    std::vector<Point> stI = remap_samples(si0, *pcI);
                    double smin = 1e300, smax = -1e300, tmin = 1e300, tmax = -1e300;
                    for (auto& q : stI) {
                        smin = std::min(smin, q[0]); smax = std::max(smax, q[0]);
                        tmin = std::min(tmin, q[1]); tmax = std::max(tmax, q[1]);
                    }
                    const double iso_tol = 1e-5;
                    bool iso_s = (smax - smin) < iso_tol;   // u = const (meridian/ruling)
                    bool iso_t = (tmax - tmin) < iso_tol;   // v = const (parallel/equator)
                    Point cc(0, 0, 0); Vector cax(0, 0, 0); double crad = -1.0;
                    double s0v = 0.5 * (smin + smax), t0v = 0.5 * (tmin + tmax);
                    if (iso_t && !iso_s) {
                        if (A.kind == 5) {          // torus parallel
                            cc = Point(A.C[0] + A.r2 * std::sin(t0v) * A.Z[0],
                                       A.C[1] + A.r2 * std::sin(t0v) * A.Z[1],
                                       A.C[2] + A.r2 * std::sin(t0v) * A.Z[2]);
                            crad = A.R + A.r2 * std::cos(t0v);
                            cax = A.Z;
                        } else if (A.kind == 2) {   // cylinder circle
                            cc = Point(A.C[0] + t0v * A.Z[0], A.C[1] + t0v * A.Z[1], A.C[2] + t0v * A.Z[2]);
                            crad = A.R;
                            cax = A.Z;
                        } else if (A.kind == 3) {   // cone circle (t = slant from apex)
                            double ca = std::cos(A.r2), sa = std::sin(A.r2);
                            cc = Point(A.C[0] + t0v * ca * A.Z[0], A.C[1] + t0v * ca * A.Z[1], A.C[2] + t0v * ca * A.Z[2]);
                            crad = t0v * sa;
                            cax = A.Z;
                        } else if (A.kind == 4) {   // sphere parallel
                            cc = Point(A.C[0] + A.R * std::sin(t0v) * A.Z[0],
                                       A.C[1] + A.R * std::sin(t0v) * A.Z[1],
                                       A.C[2] + A.R * std::sin(t0v) * A.Z[2]);
                            crad = A.R * std::cos(t0v);
                            cax = A.Z;
                        }
                    } else if (iso_s && !iso_t) {
                        if (A.kind == 5) {          // torus meridian (tube circle)
                            double cs = std::cos(s0v), sn = std::sin(s0v);
                            cc = Point(A.C[0] + A.R * (cs * A.X[0] + sn * A.Y[0]),
                                       A.C[1] + A.R * (cs * A.X[1] + sn * A.Y[1]),
                                       A.C[2] + A.R * (cs * A.X[2] + sn * A.Y[2]));
                            crad = A.r2;
                            cax = Vector(-sn * A.X[0] + cs * A.Y[0],
                                         -sn * A.X[1] + cs * A.Y[1],
                                         -sn * A.X[2] + cs * A.Y[2]);
                        } else if (A.kind == 4) {   // sphere meridian (great circle)
                            double cs = std::cos(s0v), sn = std::sin(s0v);
                            cc = A.C;
                            crad = A.R;
                            cax = Vector(-sn * A.X[0] + cs * A.Y[0],
                                         -sn * A.X[1] + cs * A.Y[1],
                                         -sn * A.X[2] + cs * A.Y[2]);
                        }
                    }
                    if (crad > diag * 1e-7) {
                        auto d3b = c3built.domain();
                        Point P0 = c3built.point_at(d3b.first);
                        Point Pe = c3built.point_at(d3b.first + (d3b.second - d3b.first) * 0.02);
                        // circle natural direction at P0 must match the built curve
                        Vector rp(P0[0] - cc[0], P0[1] - cc[1], P0[2] - cc[2]);
                        Vector tccw(cax[1] * rp[2] - cax[2] * rp[1],
                                    cax[2] * rp[0] - cax[0] * rp[2],
                                    cax[0] * rp[1] - cax[1] * rp[0]);
                        Vector tv(Pe[0] - P0[0], Pe[1] - P0[1], Pe[2] - P0[2]);
                        if (tccw[0] * tv[0] + tccw[1] * tv[1] + tccw[2] * tv[2] < 0)
                            cax = Vector(-cax[0], -cax[1], -cax[2]);
                        // ref direction = center -> curve start (vertex parameter 0)
                        double rl = std::sqrt(rp[0] * rp[0] + rp[1] * rp[1] + rp[2] * rp[2]);
                        if (rl > diag * 1e-9) {
                            // entities are written per ARC (emit_arc_edge): a closed circle
                            // splits into two arcs, each with its own ref direction
                            W_circle = true;
                            W_cc = cc; W_cref = Vector(rp[0] / rl, rp[1] / rl, rp[2] / rl);
                            W_cax = cax; W_crad = crad;
                            // angular extent from the built curve's endpoints (CCW around cax)
                            Point PE = c3built.point_at(d3b.second);
                            Vector re(PE[0] - cc[0], PE[1] - cc[1], PE[2] - cc[2]);
                            Vector Yd(cax[1] * W_cref[2] - cax[2] * W_cref[1],
                                      cax[2] * W_cref[0] - cax[0] * W_cref[2],
                                      cax[0] * W_cref[1] - cax[1] * W_cref[0]);
                            double xa = re[0]*W_cref[0] + re[1]*W_cref[1] + re[2]*W_cref[2];
                            double ya = re[0]*Yd[0] + re[1]*Yd[1] + re[2]*Yd[2];
                            double ang = std::atan2(ya, xa);
                            const double TWOPI = 6.283185307179586;
                            if (ang < 1e-9) ang += TWOPI;
                            W_a = 0.0; W_b = ang;
                        }
                    }
                }
            }
            if (!W_circle) {
                W_spline = compress_curve(c3built, fit3d);
                auto dW = W_spline.domain();
                W_a = dW.first; W_b = dW.second;
            }
        }
        // Vertices = the built curve's endpoints, location-deduped. Kernel vertex indices
        // are ignored entirely: they can sit on a DIFFERENT curve than the trims describe.
        auto d3 = c3built.domain();
        bool closed_e = rotated
            || c3built.point_at(d3.first).distance(c3built.point_at(d3.second)) < diag * 1e-7;
        int v0, v1;
        if (rotated) {
            v0 = v1 = vertex_at_point(anchor);
        } else {
            v0 = vertex_at_point(c3built.point_at(d3.first));
            v1 = closed_e ? v0 : vertex_at_point(c3built.point_at(d3.second));
        }
        if (v0 < 0 || v1 < 0) return none;
        ArcSpec base;
        base.circle = W_circle;
        base.cc = W_cc; base.cref = W_cref; base.cax = W_cax; base.crad = W_crad;
        base.spline = W_spline;
        base.a = W_a; base.b = W_b;
        base.vs = v0; base.ve = v1;
        if (closed_e) {
            // split at the parameter midpoint; arc1 keeps the junction anchor as its start
            // (the rotation above already put the anchor at parameter W_a)
            ArcSpec a1 = base, a2 = base;
            bool split_ok = false;
            Point pm(0, 0, 0);
            if (W_circle) {
                double half = 0.5 * (W_b - W_a);
                Vector Yd(W_cax[1] * W_cref[2] - W_cax[2] * W_cref[1],
                          W_cax[2] * W_cref[0] - W_cax[0] * W_cref[2],
                          W_cax[0] * W_cref[1] - W_cax[1] * W_cref[0]);
                double cs = std::cos(half), sn = std::sin(half);
                pm = Point(W_cc[0] + W_crad * (cs * W_cref[0] + sn * Yd[0]),
                           W_cc[1] + W_crad * (cs * W_cref[1] + sn * Yd[1]),
                           W_cc[2] + W_crad * (cs * W_cref[2] + sn * Yd[2]));
                Vector r2(pm[0] - W_cc[0], pm[1] - W_cc[1], pm[2] - W_cc[2]);
                double r2l = std::sqrt(r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2]);
                if (r2l > diag * 1e-9) {
                    a1.b = half;                        // arc1: [0, half] on cref
                    a2.cref = Vector(r2[0] / r2l, r2[1] / r2l, r2[2] / r2l);
                    a2.b = W_b - half;                  // arc2: [0, rest] on its OWN ref dir
                    split_ok = true;
                }
            } else {
                NurbsCurve cl, cr;
                if (W_spline.split(0.5 * (W_a + W_b), cl, cr) && cl.is_valid() && cr.is_valid()) {
                    pm = cl.point_at(cl.domain().second);
                    a1.spline = cl;
                    auto dl = cl.domain();
                    a1.a = dl.first; a1.b = dl.second;
                    a2.spline = cr;
                    auto dr = cr.domain();
                    a2.a = dr.first; a2.b = dr.second;
                    split_ok = true;
                }
            }
            if (split_ok) {
                int vm = vertex_at_point(pm);
                if (vm >= 0 && vm != v0) {
                    a1.f0 = 0.0; a1.f1 = 0.5;
                    a2.f0 = 0.5; a2.f1 = 1.0;
                    a1.vs = v0; a1.ve = vm;
                    a2.vs = vm; a2.ve = v1;
                    int e1 = emit_arc_edge(ei, a1);
                    int e2 = emit_arc_edge(ei, a2);
                    if (e1 >= 0 && e2 >= 0) {
                        edge_step_id[ei] = {e1, e2};
                        return edge_step_id[ei];
                    }
                }
            }
        }
        int e0 = emit_arc_edge(ei, base);
        if (e0 < 0) return none;
        edge_step_id[ei] = {e0, -1};
        return edge_step_id[ei];
    };

    // 3a. PASS A (no writing): chain every loop head-to-tail and record each trim's
    // traversal + 3D-forward sense. Stored trim directions are EDGE-relative in this kernel,
    // not loop-chaining (both seam runs of a cylinder/sphere store the same direction), so
    // ORIENTED_EDGE senses derived from them leave wires unclosable and importers close them
    // through EXTRAPOLATED parameter space (torus face at 2x area over u in [-4,0]). Chaining
    // runs on the 3D images of the pcurve endpoints (periodic-safe; a tiny UV term breaks
    // ties between coincident seam mates), then each chain is flipped as a whole when its
    // MEASURED winding disagrees with the target (material-left iff same_sense). The senses
    // feed both the ORIENTED_EDGEs and SEAM_CURVE pcurve ordering in edge_id.
    for (int fi = 0; fi < brep.face_count(); fi++) {
        const BRepFace& face = brep.m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)brep.m_surfaces.size()) continue;
        const NurbsSurface& srf = brep.m_surfaces[face.surface_index];
        // STEP/OCCT winding convention: loops are wound MATERIAL-LEFT in the WRITTEN
        // parameterization, ALWAYS -- same_sense alone carries which side the solid is on.
        // (Winding-follows-same_sense was wrong: cut cells imported right only by double
        // negative -- our quadric charts are opposite-handed to the canonical analytic
        // parameterization, so the remap flips winding -- and fuse cells inverted: every
        // torus face of box fuse tor SUBTRACTED, tor_fuse_tor2 shattered into 4 shells.)
        bool dpflip = false;
        {
            const AnalyticSrf& A = analytic_of(face.surface_index);
            if (A.kind > 0) {
                auto duA = srf.domain(0); auto dvA = srf.domain(1);
                Point pm = srf.point_at(0.5*(duA.first+duA.second), 0.5*(dvA.first+dvA.second));
                Vector nb = srf.normal_at(0.5*(duA.first+duA.second), 0.5*(dvA.first+dvA.second));
                Vector na = analytic_normal_of(A, pm);
                dpflip = (na[0]*nb[0] + na[1]*nb[1] + na[2]*nb[2]) < 0;
            }
        }
        // STEP winding rule (verified against OCCT's own export + its imported wire
        // orientations): the outer bound is CCW in the written chart iff the ADVANCED_FACE
        // same_sense is .T. -- NOT unconditionally material-left. ss_written = outward XOR
        // dpflip and written-chart handedness = kernel-chart XOR dpflip, so the dpflips
        // cancel: required material-left measured in the KERNEL chart = outward. (The old
        // "material-left always" rule was calibrated while broken vertex topology forced
        // the reader to RE-PROJECT pcurves, hiding the winding semantics entirely.)
        bool target_ml = (fi < (int)fsign.size()) ? (fsign[fi] >= 0.0) : true;
        for (int loop_idx : face.loop_indices) {
            if (loop_idx < 0 || loop_idx >= (int)brep.m_loops.size()) continue;
            const BRepLoop& loop = brep.m_loops[loop_idx];
            struct Leg { int ti; char from_first; Point a, b; Point ua, ub; Point d0, d1; };
            std::vector<Leg> legs;
            for (int trim_idx : loop.trim_indices) {
                if (trim_idx < 0 || trim_idx >= (int)brep.m_trims.size()) continue;
                const BRepTrim& trim = brep.m_trims[trim_idx];
                if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)brep.m_curves_2d.size()) continue;
                const NurbsCurve& pc = brep.m_curves_2d[trim.curve_2d_index];
                if (!pc.is_valid()) continue;
                auto pd = pc.domain();
                Point ua = pc.point_at(pd.first), ub = pc.point_at(pd.second);
                // near-end UV tangents for junction turn decisions (bowtie tie-break)
                Point qa = pc.point_at(pd.first + (pd.second - pd.first) * 0.03);
                Point qb = pc.point_at(pd.first + (pd.second - pd.first) * 0.97);
                Point d0(qa[0] - ua[0], qa[1] - ua[1], 0.0);   // start -> into curve
                Point d1(ub[0] - qb[0], ub[1] - qb[1], 0.0);   // out of curve at end
                legs.push_back({trim_idx, 1, srf.point_at(ua[0], ua[1]), srf.point_at(ub[0], ub[1]),
                                ua, ub, d0, d1});
            }
            // Chain in KERNEL UV with PERIODIC wrap distance: a boundary that CROSSES a
            // kernel seam (box_cut_tor's bite arcs straddle both) jumps a full chart period
            // there, so non-periodic distance breaks the walk. Wrapping brings back the
            // seam-mate tie (3D-identical, kernel-UV one period apart) -- two deterministic
            // tie-breaks resolve every junction: (1) prefer UNWRAPPED connections (same-cell
            // continuation beats a wrapped jump), (2) avoid hopping straight to the SAME
            // edge's other trim (the mate), which is a zero-area spur.
            auto duF = srf.domain(0); auto dvF = srf.domain(1);
            double per_u = duF.second - duF.first, per_v = dvF.second - dvF.first;
            bool closed_u = srf.point_at(duF.first, 0.5*(dvF.first+dvF.second))
                            .distance(srf.point_at(duF.second, 0.5*(dvF.first+dvF.second))) < diag * 1e-7;
            bool closed_v = srf.point_at(0.5*(duF.first+duF.second), dvF.first)
                            .distance(srf.point_at(0.5*(duF.first+duF.second), dvF.second)) < diag * 1e-7;
            double eps_pref = 1e-7 * std::max(per_u, per_v);
            auto uvdist = [&](const Point& a2, const Point& b2) {
                double du2 = std::abs(a2[0] - b2[0]);
                double dv2 = std::abs(a2[1] - b2[1]);
                double wu = closed_u ? std::min(du2, per_u - du2) : du2;
                double wv = closed_v ? std::min(dv2, per_v - dv2) : dv2;
                double d = std::sqrt(wu*wu + wv*wv);
                if (wu != du2 || wv != dv2) d += eps_pref;      // wrapped: mild penalty
                return d;
            };
            // Greedy walk with turn-directed tie-breaks. TANGENT-POINT junctions
            // (Steinmetz cylinders, parallel-axis tori) have several legs at the SAME point;
            // distance cannot decide, and an arbitrary pick chains a bowtie -- the imported
            // face fails BRepCheck as UnorientableShape. Among exact ties take the sharpest
            // turn of one HANDEDNESS: consistent turning walks the simple loop. Which
            // handedness is right depends on the material side, so walk LEFT first and
            // re-walk RIGHT if the measured side disagrees with the target.
            auto walk = [&](double turn_sign, std::vector<int>& ord) {
                std::vector<char> used(legs.size(), 0);
                ord.clear();
                if (legs.empty()) return;
                for (auto& L : legs) L.from_first = 1;
                used[0] = 1; ord.push_back(0);
                Point uv_tail = legs[0].ub;
                Point dir_in = legs[0].d1;
                int prev_edge = brep.m_trims[legs[0].ti].edge_index;
                for (size_t k = 1; k < legs.size(); ++k) {
                    int best = -1; char bff = 1; double bd = 1e300;
                    for (size_t j = 0; j < legs.size(); ++j) {
                        if (used[j]) continue;
                        double pen = (brep.m_trims[legs[j].ti].edge_index == prev_edge
                                      && prev_edge >= 0) ? eps_pref * 0.1 : 0.0;
                        double da = uvdist(uv_tail, legs[j].ua) + pen;
                        double db = uvdist(uv_tail, legs[j].ub) + pen;
                        if (da < bd) { bd = da; best = (int)j; bff = 1; }
                        if (db < bd) { bd = db; best = (int)j; bff = 0; }
                    }
                    if (best < 0) break;
                    {
                        double tie = bd + 1e-12 * (per_u + per_v);
                        double best_turn = -1e300; int tb = -1; char tff = 1;
                        for (size_t j = 0; j < legs.size(); ++j) {
                            if (used[j]) continue;
                            double pen = (brep.m_trims[legs[j].ti].edge_index == prev_edge
                                          && prev_edge >= 0) ? eps_pref * 0.1 : 0.0;
                            for (int e2 = 0; e2 < 2; ++e2) {
                                double d = (e2 ? uvdist(uv_tail, legs[j].ub)
                                               : uvdist(uv_tail, legs[j].ua)) + pen;
                                if (d > tie) continue;
                                Point do_ = e2 ? Point(-legs[j].d1[0], -legs[j].d1[1], 0.0)
                                               : legs[j].d0;
                                double turn = turn_sign
                                    * std::atan2(dir_in[0]*do_[1] - dir_in[1]*do_[0],
                                                 dir_in[0]*do_[0] + dir_in[1]*do_[1]);
                                if (turn > best_turn) { best_turn = turn; tb = (int)j; tff = e2 ? 0 : 1; }
                            }
                        }
                        if (tb >= 0) { best = tb; bff = tff; }
                    }
                    used[best] = 1; legs[best].from_first = bff;
                    ord.push_back(best);
                    uv_tail = bff ? legs[best].ub : legs[best].ua;
                    dir_in = bff ? legs[best].d1 : Point(-legs[best].d0[0], -legs[best].d0[1], 0.0);
                    prev_edge = brep.m_trims[legs[best].ti].edge_index;
                }
            };
            auto measure_ml = [&](const std::vector<int>& ord) {
                std::vector<std::pair<int,char>> chain_dirs;
                for (int oi : ord) chain_dirs.push_back({legs[oi].ti, legs[oi].from_first});
                return brep.loop_material_left(loop_idx, &chain_dirs);
            };
            std::vector<int> order;
            walk(+1.0, order);
            // A LEFT walk never bowties; when the measured material side disagrees with the
            // target, REVERSING the whole chain keeps the loop simple and flips the side.
            // (Re-walking with RIGHT turns instead re-opened period wandering on the CUT
            // cells' reversed faces -- tor_cut_tor2's B faces imported as FULL tori.)
            if (!legs.empty() && measure_ml(order) != target_ml) {
                std::reverse(order.begin(), order.end());
                for (auto& L : legs) L.from_first = L.from_first ? 0 : 1;
            }
            auto& chain = loop_chain[loop_idx];
            for (int oi : order) {
                const Leg& L = legs[oi];
                chain.push_back({L.ti, L.from_first});
                int ei2 = brep.m_trims[L.ti].edge_index;
                if (ei2 >= 0 && !edge_canon.count(ei2))
                    edge_canon[ei2] = {L.ti, L.from_first};
                trim_sense[L.ti] = trim_forward(brep.m_trims[L.ti], srf, L.from_first != 0) ? 1 : 0;
            }
            // Junction anchors for closed legs: the 3D point where the chain ENTERS a closed
            // edge is the wire junction its written copy must start at (see edge_anchor).
            for (size_t k = 0; k < order.size(); ++k) {
                const Leg& L = legs[order[k]];
                if (L.a.distance(L.b) > diag * 1e-7) continue;       // open leg
                const BRepTrim& T = brep.m_trims[L.ti];
                if (T.edge_index < 0 || edge_anchor.count(T.edge_index)) continue;
                if (order.size() < 2) continue;                       // 1-edge loop: any start
                const Leg& P = legs[order[k > 0 ? k - 1 : order.size() - 1]];
                Point junc = P.from_first ? P.b : P.a;   // previous-in-cycle leg's end
                // anchor iff the EDGE's own 3D curve starts elsewhere (trim endpoints already
                // sit at the junction -- the mismatch is edge-curve vs wire, not trim vs wire)
                int ci3 = brep.m_topology_edges[T.edge_index].curve_3d_index;
                if (ci3 < 0 || ci3 >= (int)brep.m_curves_3d.size()) continue;
                const NurbsCurve& c3L = brep.m_curves_3d[ci3];
                Point E0 = c3L.point_at(c3L.domain().first);
                if (junc.distance(E0) > diag * 1e-6)
                    edge_anchor[T.edge_index] = junc;
            }
            if (std::getenv("SESSION_STEP_DBG")) {
                for (size_t k = 0; k < order.size(); ++k) {
                    const Leg& L = legs[order[k]];
                    std::fprintf(stderr, "[CHAIN] f=%d loop=%d leg%zu ti=%d ff=%d sense=%d uv(%.2f,%.2f)->(%.2f,%.2f)\n",
                                 fi, loop_idx, k, L.ti, (int)L.from_first, (int)trim_sense[L.ti],
                                 (L.from_first ? L.ua : L.ub)[0], (L.from_first ? L.ua : L.ub)[1],
                                 (L.from_first ? L.ub : L.ua)[0], (L.from_first ? L.ub : L.ua)[1]);
                }
                std::fprintf(stderr, "[STEPDBG] f=%d loop=%d kind=%d outward_chart=%d dpflip=%d ml=%d target=%d\n",
                             fi, loop_idx, analytic_of(face.surface_index).kind,
                             (fi < (int)fsign.size() && fsign[fi] >= 0.0) ? 1 : 0,
                             dpflip ? 1 : 0, measure_ml(order) ? 1 : 0, target_ml ? 1 : 0);
            }
        }
    }

    // 3b. PASS B: faces -> chained oriented edges -> bounds -> ADVANCED_FACE.
    std::vector<int> face_ids;
    std::vector<int> face_kfi;   // kernel face index per face_ids entry (shell grouping)
    for (int fi = 0; fi < brep.face_count(); fi++) {
        const BRepFace& face = brep.m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)brep.m_surfaces.size()) continue;
        const NurbsSurface& srf = brep.m_surfaces[face.surface_index];
        int srf_id = surface_id(face.surface_index);
        if (srf_id < 0) continue;
        bool outward = (fi < (int)fsign.size()) ? (fsign[fi] >= 0.0) : true;
        {
            // same_sense refers to the WRITTEN surface's natural normal: when an analytic
            // entity replaces our B-spline chart, their naturals can oppose.
            const AnalyticSrf& A = analytic_of(face.surface_index);
            if (A.kind > 0) {
                auto duA = srf.domain(0); auto dvA = srf.domain(1);
                Point pm = srf.point_at(0.5*(duA.first+duA.second), 0.5*(dvA.first+dvA.second));
                Vector nb = srf.normal_at(0.5*(duA.first+duA.second), 0.5*(dvA.first+dvA.second));
                Vector na = analytic_normal_of(A, pm);
                double dp = na[0]*nb[0] + na[1]*nb[1] + na[2]*nb[2];
                if (dp < 0) outward = !outward;
            }
        }
        std::string bounds = "(";
        int nbounds = 0;
        for (int loop_idx : face.loop_indices) {
            if (loop_idx < 0 || loop_idx >= (int)brep.m_loops.size()) continue;
            const BRepLoop& loop = brep.m_loops[loop_idx];
            std::vector<int> oe_ids;
            std::vector<Point> pole_pts;
            bool only_own_seams = true;   // every REAL edge is a seam wholly owned by this face
            for (const auto& [trim_idx, from_first] : loop_chain[loop_idx]) {
                const BRepTrim& trim = brep.m_trims[trim_idx];
                // Pole runs (3D-degenerate, real UV span): STEP has no degenerate edges;
                // importers re-add them (ShapeFix FixDegenerated) when the rest of the wire
                // is consistent. Skip: primitives mark them Singular (edge -1); boolean
                // results give them zero-length edges.
                bool degen = false;
                Point pole(0, 0, 0);
                if (trim.curve_2d_index >= 0 && trim.curve_2d_index < (int)brep.m_curves_2d.size()) {
                    const NurbsCurve& pcq = brep.m_curves_2d[trim.curve_2d_index];
                    auto pdq = pcq.domain();
                    Point qa = pcq.point_at(pdq.first), qb = pcq.point_at(pdq.second);
                    Point qm = pcq.point_at(0.5*(pdq.first+pdq.second));
                    Point A3 = srf.point_at(qa[0], qa[1]), B3 = srf.point_at(qb[0], qb[1]);
                    Point M3 = srf.point_at(qm[0], qm[1]);
                    auto duq = srf.domain(0); auto dvq = srf.domain(1);
                    double spanq = std::max(duq.second - duq.first, dvq.second - dvq.first);
                    // pole run: the WHOLE 3D image is one point (a closed seam iso only has
                    // coincident endpoints -- its midpoint is far)
                    if (A3.distance(B3) < diag * 1e-7 && A3.distance(M3) < diag * 1e-7
                        && qa.distance(qb) > spanq * 1e-3) { degen = true; pole = A3; }
                }
                if (trim.edge_index < 0 || degen) {
                    bool dup = false;
                    for (auto& q : pole_pts) if (q.distance(pole) < diag * 1e-6) dup = true;
                    if (!dup && trim.curve_2d_index >= 0) pole_pts.push_back(pole);
                    continue;
                }
                {   // is this edge a seam wholly owned by this face?
                    const BRepEdge& E2 = brep.m_topology_edges[trim.edge_index];
                    bool own_seam = E2.trim_indices.size() == 2;
                    for (int t2 : E2.trim_indices) {
                        int l2 = (t2 >= 0 && t2 < (int)brep.m_trims.size()) ? brep.m_trims[t2].loop_index : -1;
                        int f2 = (l2 >= 0 && l2 < (int)brep.m_loops.size()) ? brep.m_loops[l2].face_index : -1;
                        if (f2 != fi) own_seam = false;
                    }
                    if (!own_seam) only_own_seams = false;
                }
                std::pair<int, int> ec = edge_id(trim.edge_index);
                if (ec.first < 0) continue;
                bool fwd = trim_sense[trim_idx] != 0;
                // split closed edge: BOTH arcs in traversal order (arc1 then arc2 forward,
                // arc2 then arc1 reversed), each with the trim's sense
                if (ec.second >= 0 && !fwd) std::swap(ec.first, ec.second);
                oe_ids.push_back(w.write_raw("ORIENTED_EDGE('',*,*,#" + std::to_string(ec.first)
                                             + (fwd ? ",.T.)" : ",.F.)")));
                if (ec.second >= 0)
                    oe_ids.push_back(w.write_raw("ORIENTED_EDGE('',*,*,#" + std::to_string(ec.second)
                                                 + (fwd ? ",.T.)" : ",.F.)")));
            }
            // CLOSED B-SPLINE face whose only real boundary is its own seam: OCCT bounds such
            // faces with VERTEX_LOOPs at the poles and NO edges (verified on its NurbsConvert
            // sphere export) -- a clamped seam pair plus skipped poles imports as a zero-width
            // sliver instead (freeform_blob). Emit the canonical form.
            if (analytic_of(face.surface_index).kind == 0 && only_own_seams
                && !pole_pts.empty() && !oe_ids.empty()) {
                for (auto& q : pole_pts) {
                    int vx = vertex_at_point(q);
                    int vl = w.write_raw("VERTEX_LOOP('',#" + std::to_string(vx) + ")");
                    int fb2 = w.write_raw("FACE_BOUND('',#" + std::to_string(vl) + ",.T.)");
                    if (nbounds) bounds += ",";
                    bounds += "#" + std::to_string(fb2);
                    ++nbounds;
                }
                continue;
            }
            if (oe_ids.empty()) continue;
            std::string el = "EDGE_LOOP('',(";
            for (size_t k = 0; k < oe_ids.size(); ++k) {
                if (k) el += ",";
                el += "#" + std::to_string(oe_ids[k]);
            }
            el += "))";
            int el_id = w.write_raw(el);
            const char* fb = (loop.type == BRepLoopType::Outer) ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            int fb_id = w.write_raw(std::string(fb) + "('',#" + std::to_string(el_id) + ",.T.)");
            if (nbounds) bounds += ",";
            bounds += "#" + std::to_string(fb_id);
            ++nbounds;
        }
        bounds += ")";
        if (nbounds == 0) continue;
        face_ids.push_back(w.write_raw("ADVANCED_FACE(''," + bounds + ",#" + std::to_string(srf_id)
                                       + (outward ? ",.T.)" : ",.F.)")));
        face_kfi.push_back(fi);
    }

    // Group written faces into edge-connected components -- one shell (and one
    // MANIFOLD_SOLID_BREP) per component. A fuse whose operands touch only along a
    // measure-zero contact (tangent line, shared corner) is two watertight shells:
    // wrapped in ONE solid, the reader splits the shell and orphans one half
    // (ecyl_fuse_ecylP imported SubshapeNotInShape, volume -0).
    std::vector<int> uf(brep.m_faces.size());
    for (size_t k = 0; k < uf.size(); ++k) uf[k] = (int)k;
    std::function<int(int)> uf_find = [&](int x) { while (uf[x] != x) x = uf[x] = uf[uf[x]]; return x; };
    for (const auto& E : brep.m_topology_edges) {
        int f0 = -1;
        for (int ti : E.trim_indices) {
            if (ti < 0 || ti >= (int)brep.m_trims.size()) continue;
            int li = brep.m_trims[ti].loop_index;
            if (li < 0 || li >= (int)brep.m_loops.size()) continue;
            int f = brep.m_loops[li].face_index;
            if (f < 0 || f >= (int)brep.m_faces.size()) continue;
            if (f0 < 0) f0 = f; else uf[uf_find(f)] = uf_find(f0);
        }
    }
    std::map<int, std::vector<int>> shell_groups;
    for (size_t k = 0; k < face_ids.size(); ++k)
        shell_groups[uf_find(face_kfi[k])].push_back(face_ids[k]);
    std::vector<std::vector<int>> shells;
    for (auto& kv : shell_groups) shells.push_back(kv.second);
    diag_out = diag;
    return shells;
}

void write_file_step_brep(const BRep& brep, const std::string& filepath) {
    // Our own STEP writer (no external kernel): the BRep's SHARED topology is written
    // faithfully -- topology vertices and edges are emitted ONCE and referenced by every
    // adjacent face (that shared referencing is what makes the shell sew into a solid on
    // import), edge geometry is the REAL m_curves_3d entry (rational weights included),
    // and the whole shell is wrapped as CLOSED_SHELL + MANIFOLD_SOLID_BREP inside the
    // AP214 product skeleton importers use to locate roots.
    StepWriter w;
    double diag = 1.0;
    std::vector<std::vector<int>> shells = emit_brep_shells(w, brep, diag);
    // Uncertainty: with exact wire topology (shared vertices, trim-built edge curves) the
    // only residuals are fit tolerances (~diag*1e-5). The old sew-scale diag*5e-3 made OCCT
    // treat thin tangent-cusp lens wires (tor x tor) as DEGENERATE: it dropped them and
    // restored natural bounds, importing each lens face as a FULL torus (+6 x 25.27 volume).
    w.finish_shape(shells, brep.is_solid(), brep.name.empty() ? "brep" : brep.name, diag * 1e-4);
    write_step_string(w.emit(), filepath);
}

void write_file_step_breps(const std::vector<const BRep*>& breps, const std::string& name,
                           const std::string& filepath) {
    // Several solids in ONE file, each colored from its OWN surfacecolor (AP214
    // presentation chain: STYLED_ITEM per ADVANCED_FACE -> ... -> COLOUR_RGB). Used to
    // ship boolean OPERANDS side by side (A red, B blue) before the operation is made.
    StepWriter w;
    std::vector<int> bodies;
    std::vector<int> styled;
    bool any_closed = false;
    double diag_all = 1.0;
    for (const BRep* b : breps) {
        if (!b) continue;
        double diag = 1.0;
        std::vector<std::vector<int>> sh = emit_brep_shells(w, *b, diag);
        diag_all = std::max(diag_all, diag);
        // PER-BREP closed flag: a watertight operand stays MANIFOLD_SOLID_BREP even when
        // it shares the file with an open result shell.
        bool closed = b->is_solid();
        any_closed = any_closed || closed;
        std::vector<int> bb = w.make_bodies(sh, closed);
        bodies.insert(bodies.end(), bb.begin(), bb.end());
        const Color& col = b->surfacecolor;
        int psa = w.color_style(col.r, col.g, col.b);
        for (const auto& grp : sh)
            for (int fid : grp)
                styled.push_back(w.write_raw("STYLED_ITEM('',(#" + std::to_string(psa)
                                             + "),#" + std::to_string(fid) + ")"));
    }
    w.finish_product(bodies, any_closed, name, diag_all * 1e-4, &styled);
    write_step_string(w.emit(), filepath);
}

} } // namespace session_cpp::file_step
