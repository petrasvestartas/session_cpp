#include "file_step.h"
#include "closest.h"
#include "nurbs_coincidence.h"
#include "curve_surface_intersection.h"
#include "nurbs_hermite.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include "fmt/core.h"
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace session_cpp {
namespace file_step {

// ═══════════════════════════════════════════════════════════════════════════
// ISO 10303-21 parser
// ═══════════════════════════════════════════════════════════════════════════
/// Kind of value a StepParam holds.
enum class StepTag {
    Ref, // Entity reference.
    Num, // Real or integer.
    Str, // Quoted string.
    Enum, // Enum literal or bare identifier; a typed value TYPE(x) keeps x in list.
    List, // Parenthesised list.
    Null, // Unset or derived value.
};

/// One parameter of an entity instance.
struct StepParam {
    StepTag tag = StepTag::Null; // Which member is set.
    int ref_id = 0; // Entity id for Ref.
    double num = 0.0; // Value for Num.
    std::string str; // Text for Str and Enum.
    std::vector<StepParam> list; // Items for List.
};

/// One TYPE(params) part of an entity instance.
struct StepSubEntity {
    std::string type; // Entity type name.
    std::vector<StepParam> params; // Parameters in file order.
};

/// One entity: a single part for a simple instance, several for a complex one.
struct StepEntity {
    std::vector<StepSubEntity> parts; // Sub-entities of the instance.

    /// Return whether any sub-entity carries type t.
    bool has(const std::string& t) const {

        for (const StepSubEntity& p : parts)
            if (p.type == t)
                return true;

        return false;
    }

    /// Return the first sub-entity of type t, or null.
    const StepSubEntity* find(const std::string& t) const {

        for (const StepSubEntity& p : parts)
            if (p.type == t)
                return &p;

        return nullptr;
    }
};

/// Entities of a parsed file by id.
struct StepFile {
    std::unordered_map<int, StepEntity> entities; // Entities by id.

    /// Return the sorted ids of every entity carrying type t.
    std::vector<int> ids_of_type(const std::string& t) const {

        std::vector<int> out;

        for (const std::pair<const int, StepEntity>& kv : entities)
            if (kv.second.has(t))
                out.push_back(kv.first);

        std::sort(out.begin(), out.end());

        return out;
    }
};

const double PI = 3.14159265358979323846; // Circle constant.
const double PI_2 = 1.5707963267948966; // Quarter turn.
const int MAX_DEPTH = 8; // Deepest list nesting parsed recursively.
const int NS = 17; // Samples per side of a surface grid.

/// Read position in a STEP text.
struct Cursor {
    std::string_view s; // Text being parsed.
    size_t p; // Current position.
    size_t end; // End of text.
};

/// Advance the cursor past whitespace.
static void skip_ws(Cursor& c) {

    while (c.p < c.end && std::isspace((unsigned char)c.s[c.p]))
        c.p++;
}

/// Advance past ch when it is next, skipping whitespace first.
static bool consume(Cursor& c, char ch) {

    skip_ws(c);

    if (c.p < c.end && c.s[c.p] == ch) {
        c.p++;

        return true;
    }

    return false;
}

/// Return whether ch can start or continue an identifier.
static bool isident(char ch) {
    return std::isupper((unsigned char)ch) || std::isdigit((unsigned char)ch) || ch == '_';
}

/// Read an optionally signed integer.
static int parse_int(Cursor& c) {

    int id = 0;

    while (c.p < c.end && std::isdigit((unsigned char)c.s[c.p]))
        id = id * 10 + (c.s[c.p++] - '0');

    return id;
}

/// Read a real, an integer or an enum literal as a double.
static double parse_number(Cursor& c) {

    const size_t start = c.p;

    if (c.p < c.end && (c.s[c.p] == '+' || c.s[c.p] == '-'))
        c.p++;

    while (c.p < c.end && (std::isdigit((unsigned char)c.s[c.p]) || c.s[c.p] == '.'))
        c.p++;

    if (c.p < c.end && (c.s[c.p] == 'e' || c.s[c.p] == 'E')) {
        size_t q = c.p + 1;

        if (q < c.end && (c.s[q] == '+' || c.s[q] == '-'))
            q++;

        if (q < c.end && std::isdigit((unsigned char)c.s[q])) {
            c.p = q;

            while (c.p < c.end && std::isdigit((unsigned char)c.s[c.p]))
                c.p++;
        }
    }

    return std::strtod(std::string(c.s.substr(start, c.p - start)).c_str(), nullptr);
}

/// Read an identifier.
static std::string parse_ident(Cursor& c) {

    const size_t start = c.p;

    while (c.p < c.end && isident(c.s[c.p]))
        c.p++;

    return std::string(c.s.substr(start, c.p - start));
}

/// Read a quoted string, unescaping doubled quotes.
static std::string parse_string(Cursor& c) {

    std::string out;
    c.p++;

    while (c.p < c.end) {
        if (c.s[c.p] != '\'') {
            out += c.s[c.p++];
            continue;
        }

        c.p++;

        if (c.p >= c.end || c.s[c.p] != '\'')
            break;

        out += '\'';
        c.p++;
    }

    return out;
}

/// Read one parameter: reference, number, string, enum, list or sub-entity.
static StepParam parse_param(Cursor& c, int depth);

/// Read a parenthesised parameter list, recursing one level deeper.
static std::vector<StepParam> parse_params(Cursor& c, int depth) {

    std::vector<StepParam> out;

    if (!consume(c, '('))
        return out;

    while (c.p < c.end) {
        skip_ws(c);

        if (c.p >= c.end || c.s[c.p] == ')')
            break;

        out.push_back(parse_param(c, depth));
        skip_ws(c);

        if (c.p < c.end && c.s[c.p] == ',')
            c.p++;
    }

    if (c.p < c.end)
        c.p++;

    return out;
}

/// Skip a parenthesised group without recursing, for lists nested deeper than MAX_DEPTH.
static void skip_list(Cursor& c) {

    int open = 0;

    while (c.p < c.end) {
        if (c.s[c.p] == '(')
            open++;

        if (c.s[c.p] == ')')
            open--;

        c.p++;

        if (open == 0)
            return;
    }
}

/// Read one parameter: reference, number, string, enum, list or sub-entity.
static StepParam parse_param(Cursor& c, int depth) {

    skip_ws(c);

    StepParam r;

    if (c.p >= c.end)
        return r;

    const char ch = c.s[c.p];

    if (ch == '#') {
        c.p++;
        r.tag = StepTag::Ref;
        r.ref_id = parse_int(c);
    } else if (ch == '$' || ch == '*') {
        c.p++;
    } else if (ch == '(') {
        r.tag = StepTag::List;

        if (depth < MAX_DEPTH)
            r.list = parse_params(c, depth + 1);
        else
            skip_list(c);
    } else if (ch == '\'') {
        r.tag = StepTag::Str;
        r.str = parse_string(c);
    } else if (ch == '.') {
        c.p++;

        const size_t start = c.p;

        while (c.p < c.end && c.s[c.p] != '.')
            c.p++;

        r.tag = StepTag::Enum;
        r.str = std::string(c.s.substr(start, c.p - start));

        if (c.p < c.end)
            c.p++;
    } else if (std::isdigit((unsigned char)ch) || ch == '-' || ch == '+') {
        r.tag = StepTag::Num;
        r.num = parse_number(c);
    } else if (std::isupper((unsigned char)ch)) {
        r.tag = StepTag::Enum;
        r.str = parse_ident(c);
        skip_ws(c);

        if (c.p < c.end && c.s[c.p] == '(')
            r.list = parse_params(c, depth + 1);
    } else {
        c.p++;
    }

    return r;
}

/// Read one TYPE(params) instance.
static StepSubEntity parse_sub_entity(Cursor& c) {

    StepSubEntity sub;
    sub.type = parse_ident(c);
    skip_ws(c);

    if (c.p < c.end && c.s[c.p] == '(')
        sub.params = parse_params(c, 0);

    return sub;
}

/// Advance past the next semicolon.
static void skip_statement(Cursor& c) {

    bool in_str = false;

    while (c.p < c.end) {
        const char ch = c.s[c.p++];

        if (ch == '\'')
            in_str = !in_str;

        if (ch == ';' && !in_str)
            return;
    }
}

/// Fill sf from the DATA sections of a STEP text: entity instances between DATA; and ENDSEC; only.
static void parse_step_string(const std::string& content, StepFile& sf) {

    Cursor c{content, 0, content.size()};
    bool data = false;

    while (c.p < c.end) {
        skip_ws(c);

        if (c.p >= c.end)
            break;

        if (c.s[c.p] != '#' || !data) {
            const std::string keyword = parse_ident(c);

            if (keyword == "DATA" || keyword == "ENDSEC")
                data = keyword == "DATA";

            skip_statement(c);
            continue;
        }

        c.p++;

        const int id = parse_int(c);

        if (!consume(c, '='))
            continue;

        skip_ws(c);

        if (c.p >= c.end)
            break;

        StepEntity ent;

        if (c.s[c.p] == '(') {
            c.p++;

            while (c.p < c.end) {
                skip_ws(c);

                if (c.p >= c.end || !std::isupper((unsigned char)c.s[c.p]))
                    break;

                ent.parts.push_back(parse_sub_entity(c));
            }

            if (!consume(c, ')')) {
                skip_statement(c);
                continue;
            }
        } else {
            ent.parts.push_back(parse_sub_entity(c));
        }

        sf.entities.emplace(id, std::move(ent));
        skip_statement(c);
    }
}

/// Remove /* */ comments outside strings from the raw text.
static std::string strip_comments(const std::string& raw) {

    std::string text;
    text.reserve(raw.size());

    size_t i = 0;
    bool in_str = false;

    while (i < raw.size()) {
        if (raw[i] == '\'')
            in_str = !in_str;

        if (!in_str && i + 1 < raw.size() && raw[i] == '/' && raw[i + 1] == '*') {
            i += 2;

            while (i + 1 < raw.size() && !(raw[i] == '*' && raw[i + 1] == '/'))
                i++;

            i += 2;
        } else {
            text += raw[i++];
        }
    }

    return text;
}

/// Read and parse a STEP file.
static StepFile parse_step_file(const std::string& filepath) {

    StepFile sf;
    std::ifstream in(filepath);

    if (!in)
        return sf;

    std::stringstream buf;
    buf << in.rdbuf();

    parse_step_string(strip_comments(buf.str()), sf);

    return sf;
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameter access
// ═══════════════════════════════════════════════════════════════════════════
/// Return the first reference parameter, or -1.
static int first_ref(const std::vector<StepParam>& params) {

    for (const StepParam& p : params)
        if (p.tag == StepTag::Ref)
            return p.ref_id;

    return -1;
}

/// Return every reference parameter in order.
static std::vector<int> all_refs(const std::vector<StepParam>& params) {

    std::vector<int> out;

    for (const StepParam& p : params)
        if (p.tag == StepTag::Ref)
            out.push_back(p.ref_id);

    return out;
}

/// Return every reference inside the list parameters.
static std::vector<int> list_refs(const std::vector<StepParam>& params) {

    std::vector<int> out;

    for (const StepParam& p : params)
        for (int ref : all_refs(p.list))
            out.push_back(ref);

    return out;
}

/// Return every numeric parameter in order.
static std::vector<double> nums(const std::vector<StepParam>& params) {

    std::vector<double> out;

    for (const StepParam& p : params)
        if (p.tag == StepTag::Num)
            out.push_back(p.num);

    return out;
}

/// Return the numbers of a list parameter as integers.
static std::vector<int> int_list(const StepParam& p) {

    std::vector<int> out;

    for (double v : nums(p.list))
        out.push_back((int)v);

    return out;
}

/// Return the numbers of a list parameter.
static std::vector<double> dbl_list(const StepParam& p) {
    return nums(p.list);
}

/// Return the numbers of a list-of-lists parameter.
static std::vector<std::vector<double>> dbl_list_list(const StepParam& p) {

    std::vector<std::vector<double>> out;

    for (const StepParam& row : p.list)
        out.push_back(dbl_list(row));

    return out;
}

/// Return the references of a list-of-lists parameter.
static std::vector<std::vector<int>> ref_list_list(const StepParam& p) {

    std::vector<std::vector<int>> out;

    for (const StepParam& row : p.list)
        out.push_back(all_refs(row.list));

    return out;
}

/// Numbers of the first list parameter that holds any.
static std::vector<double> coords(const std::vector<StepParam>& params) {

    for (const StepParam& p : params) {
        const std::vector<double> out = dbl_list(p);

        if (!out.empty())
            return out;
    }

    return {};
}

/// Last enum parameter as a flag (.T. is true), fallback when there is none.
static bool last_flag(const std::vector<StepParam>& params, bool fallback) {

    bool out = fallback;

    for (const StepParam& p : params)
        if (p.tag == StepTag::Enum)
            out = p.str == "T";

    return out;
}

/// Degree, control point ids and knots of a B-spline curve entity.
struct CurveParams {
    int degree = 0; // Polynomial degree.
    std::vector<int> pt_refs; // CARTESIAN_POINT ids.
    std::vector<int> mults; // Knot multiplicities.
    std::vector<double> knots; // Distinct knot values.
};

/// Degrees, control point id grid and knots of a B-spline surface entity.
struct SurfaceParams {
    int u_deg = 0; // Degree in u.
    int v_deg = 0; // Degree in v.
    std::vector<std::vector<int>> ctrl_pts; // CARTESIAN_POINT ids, rows along u.
    std::vector<int> u_mults; // Knot multiplicities in u.
    std::vector<int> v_mults; // Knot multiplicities in v.
    std::vector<double> u_knots; // Distinct knot values in u.
    std::vector<double> v_knots; // Distinct knot values in v.
};

/// Multiplicities and values of the knots a B-spline subtype implies for n control points: clamped uniform, Bezier or uniform.
static std::pair<std::vector<int>, std::vector<double>> implicit_knots(const StepEntity& e, const std::string& kind, int degree, int n) {

    std::vector<int> mults;
    std::vector<double> knots;

    if (e.has("UNIFORM_" + kind)) {
        for (int i = 0; i < n + degree + 1; i++) {
            mults.push_back(1);
            knots.push_back(i - degree);
        }
    } else if (e.has("QUASI_UNIFORM_" + kind) || e.has("BEZIER_" + kind)) {
        const int spans = e.has("BEZIER_" + kind) ? 1 : n - degree;

        for (int i = 0; i <= spans; i++) {
            mults.push_back(i == 0 || i == spans ? degree + 1 : 1);
            knots.push_back(i);
        }
    }

    return {mults, knots};
}

/// B_SPLINE_CURVE_WITH_KNOTS parameters, simple or split across a complex instance, knots implied by a uniform, quasi-uniform or Bezier subtype; none when missing, short or empty.
static std::optional<CurveParams> curve_params(const StepEntity& e) {

    const StepSubEntity* bsc = e.find("B_SPLINE_CURVE_WITH_KNOTS");
    const StepSubEntity* base = e.find("B_SPLINE_CURVE");
    CurveParams cp;

    if (!bsc) {
        const StepSubEntity* simple = base ? base : e.find("QUASI_UNIFORM_CURVE");

        if (!simple)
            simple = e.find("BEZIER_CURVE");

        if (!simple)
            simple = e.find("UNIFORM_CURVE");

        const size_t at = base ? 0 : 1;

        if (!simple || simple->params.size() < at + 2)
            return std::nullopt;

        cp.degree = (int)simple->params[at].num;
        cp.pt_refs = all_refs(simple->params[at + 1].list);
        std::tie(cp.mults, cp.knots) = implicit_knots(e, "CURVE", cp.degree, (int)cp.pt_refs.size());
    } else if (!base) {
        const std::vector<StepParam>& pp = bsc->params;

        if (pp.size() < 8)
            return std::nullopt;

        cp.degree = (int)pp[1].num;
        cp.pt_refs = all_refs(pp[2].list);
        cp.mults = int_list(pp[6]);
        cp.knots = dbl_list(pp[7]);
    } else {
        const std::vector<StepParam>& bp = base->params;
        const std::vector<StepParam>& kp = bsc->params;

        if (bp.size() < 2 || kp.size() < 2)
            return std::nullopt;

        cp.degree = (int)bp[0].num;
        cp.pt_refs = all_refs(bp[1].list);
        cp.mults = int_list(kp[0]);
        cp.knots = dbl_list(kp[1]);
    }

    if (cp.pt_refs.empty() || cp.mults.empty() || cp.knots.empty())
        return std::nullopt;

    return cp;
}

/// B_SPLINE_SURFACE_WITH_KNOTS parameters, simple or split across a complex instance, knots implied by a uniform, quasi-uniform or Bezier subtype; none when missing, short or empty.
static std::optional<SurfaceParams> surface_params(const StepEntity& e) {

    const StepSubEntity* bss = e.find("B_SPLINE_SURFACE_WITH_KNOTS");
    const StepSubEntity* base = e.find("B_SPLINE_SURFACE");
    SurfaceParams sp;

    if (!bss) {
        const StepSubEntity* simple = base ? base : e.find("QUASI_UNIFORM_SURFACE");

        if (!simple)
            simple = e.find("BEZIER_SURFACE");

        if (!simple)
            simple = e.find("UNIFORM_SURFACE");

        const size_t at = base ? 0 : 1;

        if (!simple || simple->params.size() < at + 3)
            return std::nullopt;

        sp.u_deg = (int)simple->params[at].num;
        sp.v_deg = (int)simple->params[at + 1].num;
        sp.ctrl_pts = ref_list_list(simple->params[at + 2]);

        if (sp.ctrl_pts.empty())
            return std::nullopt;

        std::tie(sp.u_mults, sp.u_knots) = implicit_knots(e, "SURFACE", sp.u_deg, (int)sp.ctrl_pts.size());
        std::tie(sp.v_mults, sp.v_knots) = implicit_knots(e, "SURFACE", sp.v_deg, (int)sp.ctrl_pts[0].size());
    } else if (!base) {
        const std::vector<StepParam>& pp = bss->params;

        if (pp.size() < 12)
            return std::nullopt;

        sp.u_deg = (int)pp[1].num;
        sp.v_deg = (int)pp[2].num;
        sp.ctrl_pts = ref_list_list(pp[3]);
        sp.u_mults = int_list(pp[8]);
        sp.v_mults = int_list(pp[9]);
        sp.u_knots = dbl_list(pp[10]);
        sp.v_knots = dbl_list(pp[11]);
    } else {
        const std::vector<StepParam>& bp = base->params;
        const std::vector<StepParam>& kp = bss->params;

        if (bp.size() < 3 || kp.size() < 4)
            return std::nullopt;

        sp.u_deg = (int)bp[0].num;
        sp.v_deg = (int)bp[1].num;
        sp.ctrl_pts = ref_list_list(bp[2]);
        sp.u_mults = int_list(kp[0]);
        sp.v_mults = int_list(kp[1]);
        sp.u_knots = dbl_list(kp[2]);
        sp.v_knots = dbl_list(kp[3]);
    }

    if (sp.ctrl_pts.empty() || sp.ctrl_pts[0].empty() || sp.u_mults.empty() || sp.v_mults.empty())
        return std::nullopt;

    return sp;
}

// ═══════════════════════════════════════════════════════════════════════════
// Knot utilities
// ═══════════════════════════════════════════════════════════════════════════
/// Repeat each knot value by its multiplicity.
static std::vector<double> expand_knots(const std::vector<double>& vals, const std::vector<int>& mults) {

    std::vector<double> flat;

    for (size_t i = 0; i < vals.size() && i < mults.size(); i++)
        for (int j = 0; j < mults[i]; j++)
            flat.push_back(vals[i]);

    return flat;
}

/// Collapse a flat knot vector into values and multiplicities.
static std::pair<std::vector<double>, std::vector<int>> compress_knots(const std::vector<double>& flat) {

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

/// Add the two clamped end knots to an internal knot vector.
static std::vector<double> full_from_internal(const std::vector<double>& internal) {

    if (internal.empty())
        return {};

    std::vector<double> full;
    full.push_back(internal.front());
    full.insert(full.end(), internal.begin(), internal.end());
    full.push_back(internal.back());

    return full;
}

/// Drop the two clamped end knots of a full knot vector.
static std::vector<double> internal_from_full(const std::vector<double>& full) {

    if (full.size() < 2)
        return full;

    return std::vector<double>(full.begin() + 1, full.end() - 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// Analytic geometry
// ═══════════════════════════════════════════════════════════════════════════
/// Orthonormal frame of an AXIS2_PLACEMENT_3D.
struct Axis2 {
    Point origin = Point(0, 0, 0); // Frame origin.
    Vector ax = Vector(1, 0, 0); // Frame x axis.
    Vector ay = Vector(0, 1, 0); // Frame y axis.
    Vector az = Vector(0, 0, 1); // Frame z axis.
    bool ok = false; // Whether the frame was read.
};

/// Affine parameter projector of a plane or a bilinear patch.
struct Proj {
    int kind = 0; // 0 none, 1 affine.
    Axis2 a; // Surface frame.
    double u0 = 0.0;
    double v0 = 0.0;
};

/// Analytic surface of a face.
struct AnFace {
    int kind = 0; // 2 cylinder, 3 cone, 4 sphere, 5 torus.
    Axis2 a; // Surface frame.
    double radius = 0.0; // Main radius.
    double r2 = 0.0; // Cone semi-angle or torus minor radius.
};

/// Return the point at local coordinates in the axis frame.
static Point axis_point(const Axis2& a, double lx, double ly, double lz) {
    return a.origin + a.ax * lx + a.ay * ly + a.az * lz;
}

/// Return the angle of pt around the axis in radians.
static double angle_of(const Axis2& a, const Point& pt) {

    const Vector d = pt - a.origin;

    return std::atan2(d.dot(a.ay), d.dot(a.ax));
}

/// Parameter within one quarter-arc rational span (w = sqrt(2)/2) whose angle is theta.
static double arc_param_of_angle(double theta) {

    if (theta <= 0)
        return 0.0;

    if (theta >= PI_2)
        return 1.0;

    const double w = std::sqrt(2.0) / 2.0;
    double tau = theta / PI_2;

    for (int it = 0; it < 8; it++) {
        const double o = 1.0 - tau;
        const double x = o * o + 2 * w * tau * o;
        const double y = 2 * w * tau * o + tau * tau;
        const double dx = -2 * o + 2 * w * (1 - 2 * tau);
        const double dy = 2 * w * (1 - 2 * tau) + 2 * tau;
        const double f = std::atan2(y, x) - theta;
        const double df = (x * dy - y * dx) / std::max(x * x + y * y, 1e-30);

        if (std::abs(df) < 1e-30)
            break;

        const double step = f / df;
        tau = std::clamp(tau - step, 0.0, 1.0);

        if (std::abs(step) < 1e-15)
            break;
    }

    return tau;
}

/// Chart coordinate of an angle: quarter-arc spans counted from quarter q0, corrected for the projective span parameterization.
static double chart_u_of_angle(double ang, int q0) {

    const double q = ang / PI_2 - q0;
    const double spanf = std::floor(q + 1e-12);

    return spanf + arc_param_of_angle((q - spanf) * PI_2);
}

/// Cos, sin and weight of the quarter-arc chart nodes from quarter q0: even nodes on the arc, odd nodes at the tangent corners.
static std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> arc_nodes(double q0, int nspans) {

    const int n = 2 * nspans + 1;
    std::vector<double> ca(n);
    std::vector<double> sa(n);
    std::vector<double> cw(n);

    for (int i = 0; i < n; i++) {
        const bool mid = i % 2 == 1;
        const double a0 = (q0 + i / 2) * PI_2;
        const double a1 = (q0 + i / 2 + 1) * PI_2;
        ca[i] = mid ? std::cos(a0) + std::cos(a1) : std::cos(a0);
        sa[i] = mid ? std::sin(a0) + std::sin(a1) : std::sin(a0);
        cw[i] = mid ? std::sqrt(2.0) / 2.0 : 1.0;
    }

    return {ca, sa, cw};
}

/// Integer knots of nspans quarter arcs: 0, 0, 1, 1, ..., nspans, nspans.
static std::vector<double> quarter_knots(int nspans) {

    std::vector<double> knots = {0.0, 0.0};

    for (int s = 1; s < nspans; s++) {
        knots.push_back(s);
        knots.push_back(s);
    }

    knots.push_back(nspans);
    knots.push_back(nspans);

    return knots;
}

/// Canonical (s, t) of a 3D point; radial_ok is false at a pole or apex where the angle is undefined.
static std::tuple<double, double, bool> an_st_of(const AnFace& an, const Point& p) {

    const Vector d = p - an.a.origin;
    const double x = d.dot(an.a.ax);
    const double y = d.dot(an.a.ay);
    const double z = d.dot(an.a.az);
    const double rho = std::sqrt(x * x + y * y);
    const double s = std::atan2(y, x);
    const bool radial_ok = rho > 1e-9;

    if (an.kind == 2)
        return {s, z, radial_ok};

    // The foot on the generator through the point (ElSLib::ConeParameters), so a point off the cone keeps its distance; past the apex the generator runs on the opposite side.
    if (an.kind == 3) {
        const double sa = std::sin(an.r2);
        const double ca = std::cos(an.r2);
        const bool beyond = std::abs((-rho - an.radius) * ca - z * sa) < std::abs((rho - an.radius) * ca - z * sa);

        return {beyond ? s + PI : s, ((beyond ? -rho : rho) - an.radius) * sa + z * ca, radial_ok};
    }

    if (an.kind == 4)
        return {s, std::atan2(z, rho), radial_ok};

    // A spindle torus (minor radius beyond the major one) also reaches a point from the opposite side of the axis.
    const double outer = std::abs((rho - an.radius) * (rho - an.radius) + z * z - an.r2 * an.r2);
    const double inner = std::abs((rho + an.radius) * (rho + an.radius) + z * z - an.r2 * an.r2);

    if (inner < outer)
        return {s + PI, std::atan2(z, -rho - an.radius), radial_ok};

    return {s, std::atan2(z, rho - an.radius), radial_ok};
}

/// Evaluate the analytic surface at chart parameters s, t.
static Point an_eval(const AnFace& an, double s, double t) {

    const double cs = std::cos(s);
    const double sn = std::sin(s);

    if (an.kind == 2)
        return axis_point(an.a, an.radius * cs, an.radius * sn, t);

    if (an.kind == 3) {
        const double r = an.radius + t * std::sin(an.r2);

        return axis_point(an.a, r * cs, r * sn, t * std::cos(an.r2));
    }

    const double ct = std::cos(t);
    const double st = std::sin(t);

    if (an.kind == 4)
        return axis_point(an.a, an.radius * ct * cs, an.radius * ct * sn, an.radius * st);

    const double r = an.radius + an.r2 * ct;

    return axis_point(an.a, r * cs, r * sn, an.r2 * st);
}

/// Kernel NURBS window of an analytic surface: nsu quarter arcs from quarter su0 in u; v is linear on [t0, t1] for cylinder and cone, nsv quarter arcs from sv0 for sphere and torus.
static NurbsSurface build_analytic_nurbs(const AnFace& an, int su0, int nsu, double t0, double t1, int sv0, int nsv) {

    std::vector<double> ca;
    std::vector<double> sa;
    std::vector<double> cw;
    std::tie(ca, sa, cw) = arc_nodes(su0, nsu);

    const int nu = 2 * nsu + 1;

    if (an.kind == 2 || an.kind == 3) {
        NurbsSurface srf(3, true, 3, 2, nu, 2);
        srf.m_nurbsknot[0] = quarter_knots(nsu);
        srf.m_nurbsknot[1] = {t0, t1};

        for (int j = 0; j < 2; j++) {
            const double t = j == 0 ? t0 : t1;
            const double r = an.kind == 2 ? an.radius : an.radius + t * std::sin(an.r2);
            const double z = an.kind == 2 ? t : t * std::cos(an.r2);

            for (int i = 0; i < nu; i++) {
                const Point p = axis_point(an.a, r * ca[i], r * sa[i], z);

                if (!srf.set_cv_4d(i, j, cw[i] * p[0], cw[i] * p[1], cw[i] * p[2], cw[i]))
                    return NurbsSurface();
            }
        }

        return srf;
    }

    std::vector<double> cb;
    std::vector<double> sb;
    std::vector<double> vw;
    std::tie(cb, sb, vw) = arc_nodes(sv0, nsv);

    const int nv = 2 * nsv + 1;
    NurbsSurface srf(3, true, 3, 3, nu, nv);
    srf.m_nurbsknot[0] = quarter_knots(nsu);
    srf.m_nurbsknot[1] = quarter_knots(nsv);

    for (int j = 0; j < nv; j++) {
        const double r = an.kind == 4 ? an.radius * cb[j] : an.radius + an.r2 * cb[j];
        const double z = an.kind == 4 ? an.radius * sb[j] : an.r2 * sb[j];

        for (int i = 0; i < nu; i++) {
            const Point p = axis_point(an.a, r * ca[i], r * sa[i], z);
            const double wij = cw[i] * vw[j];

            if (!srf.set_cv_4d(i, j, wij * p[0], wij * p[1], wij * p[2], wij))
                return NurbsSurface();
        }
    }

    return srf;
}

/// Rational surface of revolution of a profile about the frame's z axis: four quarter arcs in u from angle phi0, the profile in v.
static NurbsSurface revolution_nurbs(const Axis2& a, const NurbsCurve& profile, double phi0) {

    std::vector<double> ca;
    std::vector<double> sa;
    std::vector<double> cw;
    std::tie(ca, sa, cw) = arc_nodes(phi0 / PI_2, 4);
    const int nv = profile.cv_count();
    NurbsSurface srf(3, true, 3, profile.order(), 9, nv);
    srf.m_nurbsknot[0] = quarter_knots(4);
    srf.m_nurbsknot[1] = profile.m_nurbsknot;

    for (int j = 0; j < nv; j++) {
        const Point p = profile.get_cv(j);
        const double w = profile.weight(j);
        const Point o = a.origin + a.az * (p - a.origin).dot(a.az);
        const Vector r = p - o;
        const Vector q = a.az.cross(r);

        for (int i = 0; i < 9; i++) {
            const Point c = o + r * ca[i] + q * sa[i];
            const double wij = cw[i] * w;

            if (!srf.set_cv_4d(i, j, wij * c[0], wij * c[1], wij * c[2], wij))
                return NurbsSurface();
        }
    }

    return srf.is_valid() ? srf : NurbsSurface();
}

/// Bilinear patch of the plane with axis a over [u0, u1] x [v0, v1].
static NurbsSurface plane_surface(const Axis2& a, double u0, double u1, double v0, double v1) {

    NurbsSurface out(3, false, 2, 2, 2, 2);
    out.m_nurbsknot[0] = {u0, u1};
    out.m_nurbsknot[1] = {v0, v1};

    const bool ok = out.set_cv(0, 0, axis_point(a, u0, v0, 0.0)) && out.set_cv(0, 1, axis_point(a, u0, v1, 0.0)) &&
        out.set_cv(1, 0, axis_point(a, u1, v0, 0.0)) && out.set_cv(1, 1, axis_point(a, u1, v1, 0.0));

    return ok ? out : NurbsSurface();
}

/// Parameter-space image of a 3D point under an affine projector.
static std::pair<double, double> project(const Proj& pr, const Point& pt) {

    const Vector d = pt - pr.a.origin;

    return {pr.u0 + d.dot(pr.a.ax), pr.v0 + d.dot(pr.a.ay)};
}

/// Require an exact Cartesian parallelogram before proposing an affine inverse.
static bool affine_patch_frame(const NurbsSurface& srf, Point& origin, Vector& eu, Vector& ev) {
    if (!srf.is_valid() || srf.dimension() != 3 || srf.degree(0) != 1 || srf.degree(1) != 1 ||
        srf.cv_count(0) != 2 || srf.cv_count(1) != 2)
        return false;
    const double weight = srf.weight(0, 0);
    if (!(weight > 0.0) || !std::isfinite(weight))
        return false;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            double x, y, z, w;
            if (!srf.get_cv_4d(i, j, x, y, z, w) || w != weight)
                return false;
            const Point point = srf.get_cv(i, j);
            const double homogeneous[3] = {x, y, z};
            for (int d = 0; d < 3; ++d)
                if (!nurbs_coincidence::exact_products_equal(point[d], w, homogeneous[d], 1.0))
                    return false;
        }
    origin = srf.get_cv(0, 0);
    const Point pu = srf.get_cv(1, 0), pv = srf.get_cv(0, 1), opposite = srf.get_cv(1, 1);
    for (int d = 0; d < 3; ++d) {
        double upper;
        if (!nurbs_coincidence::exact_difference(pu[d], origin[d], eu[d]) ||
            !nurbs_coincidence::exact_difference(pv[d], origin[d], ev[d]) ||
            !nurbs_coincidence::exact_difference(opposite[d], pu[d], upper) || upper != ev[d])
            return false;
    }
    return true;
}

/// Raw-chart Gram dual; the rounded inverse remains a proposal checked on each edge.
static Proj bilinear_projector(const NurbsSurface& srf) {
    Proj pr;
    Vector eu, ev;
    if (!affine_patch_frame(srf, pr.a.origin, eu, ev))
        return pr;
    const auto u = srf.domain(0), v = srf.domain(1);
    const double du = u.second - u.first, dv = v.second - v.first;
    if (!std::isfinite(u.first) || !std::isfinite(v.first) || !std::isfinite(du) ||
        !std::isfinite(dv) || !(du > 0.0) || !(dv > 0.0))
        return pr;
    nurbs_interval::Interval uu, vv, uv;
    for (int d = 0; d < 3; ++d) {
        uu = uu + nurbs_interval::Interval(eu[d]) * nurbs_interval::Interval(eu[d]);
        vv = vv + nurbs_interval::Interval(ev[d]) * nurbs_interval::Interval(ev[d]);
        uv = uv + nurbs_interval::Interval(eu[d]) * nurbs_interval::Interval(ev[d]);
    }
    const auto determinant = uu * vv - uv * uv;
    const double eu2 = eu.dot(eu), ev2 = ev.dot(ev), cross = eu.dot(ev);
    const double det = std::fma(eu2, ev2, -cross * cross);
    if (!nurbs_interval::finite(determinant) || !(determinant.lo > 0.0) ||
        !std::isfinite(det) || !(det > 0.0))
        return pr;
    pr.a.ax = (eu * ev2 - ev * cross) * (du / det);
    pr.a.ay = (ev * eu2 - eu * cross) * (dv / det);
    for (int d = 0; d < 3; ++d)
        if (!std::isfinite(pr.a.ax[d]) || !std::isfinite(pr.a.ay[d]))
            return Proj();
    pr.u0 = u.first;
    pr.v0 = v.first;
    pr.kind = 1;
    pr.a.ok = true;
    return pr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve helpers
// ═══════════════════════════════════════════════════════════════════════════
/// n points evenly spaced in parameter over the curve domain.
static std::vector<Point> sample_nurbs(const NurbsCurve& nc, int n) {

    double tmin = 0.0;
    double tmax = 0.0;
    std::tie(tmin, tmax) = nc.domain();

    std::vector<Point> pts;

    for (int i = 0; i < n; i++)
        pts.push_back(nc.point_at(n > 1 ? tmin + (tmax - tmin) * i / (n - 1) : tmin));

    return pts;
}

/// Degree-1 curve through the points with integer knots, dim 2 or 3; invalid for fewer than two points.
static NurbsCurve polyline_nurbs(const std::vector<Point>& pts, int dim) {

    const int n = (int)pts.size();

    if (n < 2)
        return NurbsCurve();

    NurbsCurve nc(dim, false, 2, n);

    for (int i = 0; i < n; i++) {
        nc.m_nurbsknot[i] = (double)i;

        if (!nc.set_cv(i, pts[i]))
            return NurbsCurve();
    }

    return nc;
}

/// Exact rational arc on the circle (axis a, radius rad) or the ellipse (radii rad, rad2) from vs to ve, the full curve when they coincide.
static NurbsCurve circle_nurbs(const Axis2& a, double rad, const Point& vs, const Point& ve,
                               int requested_spans = 0, bool force_open = false, double rad2 = 0.0) {

    const double ry = rad2 > 0.0 ? rad2 : rad;
    const auto angle = [&](const Point& p) { return std::atan2((p - a.origin).dot(a.ay) / ry, (p - a.origin).dot(a.ax) / rad); };
    const double sa = angle(vs);
    double ea = angle(ve);

    if (force_open && ea == sa)
        return NurbsCurve(); // Distinct Cartesian trims with unresolved angular extent.

    if (!force_open && vs.distance(ve) < 1e-10)
        ea = sa + 2.0 * PI;
    else if (ea <= sa)
        ea += 2.0 * PI;

    const double span = ea - sa;
    const int ns = requested_spans > 0 ? requested_spans : std::max(1, (int)std::ceil(std::abs(span) / PI_2));
    if (ns < 1 || !(span > 0) || !(span / ns < PI))
        return NurbsCurve();
    const int n_cp = 2 * ns + 1;
    const double wm = std::cos(span / (2.0 * ns));
    NurbsCurve crv(3, true, 3, n_cp);
    crv.m_nurbsknot[0] = sa;
    crv.m_nurbsknot[1] = sa;

    for (int s = 1; s < ns; s++) {
        crv.m_nurbsknot[2 * s] = sa + s * span / ns;
        crv.m_nurbsknot[2 * s + 1] = sa + s * span / ns;
    }

    crv.m_nurbsknot[2 * ns] = ea;
    crv.m_nurbsknot[2 * ns + 1] = ea;

    for (int i = 0; i < n_cp; i++) {
        const bool mid = i % 2 == 1;
        const double ang = sa + (i / 2 + (mid ? 0.5 : 0.0)) * span / ns;
        const double w = mid ? wm : 1.0;
        const Point p = a.origin + (a.ax * (rad * std::cos(ang)) + a.ay * (ry * std::sin(ang))) * (1.0 / (mid ? wm : 1.0));

        if (!crv.set_cv_4d(i, w * p[0], w * p[1], w * p[2], w))
            return NurbsCurve();
    }

    return crv;
}

/// Exact rational arc of the hyperbola (axis a, semi-axes ra, rb) from vs to ve: one conic span, weight cosh of half the parameter span.
static NurbsCurve hyperbola_nurbs(const Axis2& a, double ra, double rb, const Point& vs, const Point& ve) {

    const double t0 = std::asinh((vs - a.origin).dot(a.ay) / rb);
    const double t1 = std::asinh((ve - a.origin).dot(a.ay) / rb);
    const double h = 0.5 * (t1 - t0);
    const double w = std::cosh(h);
    const Point m = a.origin + (a.ax * (ra * std::cosh(t0 + h)) + a.ay * (rb * std::sinh(t0 + h))) * (1.0 / w);
    NurbsCurve crv(3, true, 3, 3);
    crv.m_nurbsknot = {0, 0, 1, 1};

    if (!(h != 0.0) || !crv.set_cv_4d(0, vs[0], vs[1], vs[2], 1.0) || !crv.set_cv_4d(1, w * m[0], w * m[1], w * m[2], w) ||
        !crv.set_cv_4d(2, ve[0], ve[1], ve[2], 1.0))
        return NurbsCurve();

    return crv;
}

/// Degree-1 pcurve from (u0, v0) to (u1, v1).
static NurbsCurve uv_line(double u0, double v0, double u1, double v1) {
    return NurbsCurve::create(false, 1, {Point(u0, v0, 0), Point(u1, v1, 0)});
}

/// Exact pcurve of a 3D curve under an affine projector: control points map one to one, weights unchanged.
static NurbsCurve exact_pcurve(const Proj& proj, const NurbsCurve& c3) {

    if (proj.kind != 1 || !c3.is_valid() || c3.cv_count() < 2)
        return NurbsCurve();

    NurbsCurve p2(3, c3.is_rational(), c3.order(), c3.cv_count());
    p2.m_nurbsknot = c3.m_nurbsknot;

    for (int ci = 0; ci < c3.cv_count(); ci++) {
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        double w = 0.0;
        std::tie(wx, wy, wz, w) = c3.get_cv_4d(ci);

        if (std::abs(w) < 1e-300)
            return NurbsCurve();

        double u = 0.0;
        double v = 0.0;
        std::tie(u, v) = project(proj, Point(wx / w, wy / w, wz / w));

        if (!p2.set_cv_4d(ci, u * w, v * w, 0.0, w))
            return NurbsCurve();
    }

    return p2.is_valid() ? p2 : NurbsCurve();
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface grid
// ═══════════════════════════════════════════════════════════════════════════
/// ns x ns surface points over the domain, row-major with u slowest.
static std::vector<Point> surface_grid(const NurbsSurface& srf, int ns) {

    double u0 = 0.0;
    double u1 = 0.0;
    double v0 = 0.0;
    double v1 = 0.0;
    std::tie(u0, u1) = srf.domain(0);
    std::tie(v0, v1) = srf.domain(1);

    std::vector<Point> grid;

    for (int i = 0; i < ns; i++)
        for (int j = 0; j < ns; j++)
            grid.push_back(Point(srf.point(u0 + (u1 - u0) * i / (ns - 1), v0 + (v1 - v0) * j / (ns - 1))));

    return grid;
}

/// Return the largest distance from the first grid point.
static double grid_scale(const std::vector<Point>& grid) {

    double scale = 0.0;

    for (const Point& p : grid)
        scale = std::max(scale, p.distance(grid[0]));

    return scale;
}

/// True when the first and last row (along_u) or column coincide within tol.
static bool grid_closed(const std::vector<Point>& grid, int ns, double tol, bool along_u) {

    for (int k = 0; k < ns; k++) {
        const Point& a = along_u ? grid[k] : grid[k * ns];
        const Point& b = along_u ? grid[(ns - 1) * ns + k] : grid[k * ns + ns - 1];

        if (a.distance(b) > tol)
            return false;
    }

    return true;
}

/// True when column j collapses to one point (a pole or apex).
static bool grid_degenerate(const std::vector<Point>& grid, int ns, double tol, int j) {

    for (int k = 1; k < ns; k++)
        if (grid[k * ns + j].distance(grid[j]) > tol)
            return false;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// StepReader
// ═══════════════════════════════════════════════════════════════════════════
/// Entity access over a parsed file with points, directions and frames cached by id.
class StepReader {
    const StepFile& sf; // Parsed file.
    std::unordered_map<int, Point> pt_cache; // Points by id.
    std::unordered_map<int, Vector> dir_cache; // Directions by id.
    std::unordered_map<int, Axis2> ax_cache; // Frames by id.

public:

    /// Construct over a parsed file.
    explicit StepReader(const StepFile& s) : sf(s) {}

    /// Radians per plane angle unit of the first GLOBAL_UNIT_ASSIGNED_CONTEXT: its CONVERSION_BASED_UNIT factor (degrees), else 1.
    double angle_unit() const {

        for (int id : sf.ids_of_type("GLOBAL_UNIT_ASSIGNED_CONTEXT"))
            for (int unit : list_refs(get(id)->find("GLOBAL_UNIT_ASSIGNED_CONTEXT")->params)) {
                const StepEntity* u = get(unit);
                const StepSubEntity* conversion = u && u->has("PLANE_ANGLE_UNIT") ? u->find("CONVERSION_BASED_UNIT") : nullptr;
                const StepEntity* measure = conversion ? get(first_ref(conversion->params)) : nullptr;
                const StepSubEntity* with_unit = measure ? measure->find("PLANE_ANGLE_MEASURE_WITH_UNIT") : nullptr;
                const std::vector<double> factor = with_unit && !with_unit->params.empty() ? nums(with_unit->params[0].list) : std::vector<double>();

                if (u && u->has("PLANE_ANGLE_UNIT"))
                    return factor.empty() ? 1.0 : factor[0];
            }

        return 1.0;
    }

    /// Return the entity with this id, or null.
    const StepEntity* get(int id) const {

        const auto it = sf.entities.find(id);

        return it == sf.entities.end() ? nullptr : &it->second;
    }

    /// Read a CARTESIAN_POINT, caching by id.
    Point get_point(int id) {

        const auto it = pt_cache.find(id);

        if (it != pt_cache.end())
            return it->second;

        Point pt(0, 0, 0);
        const StepEntity* e = get(id);
        const StepSubEntity* sub = e ? e->find("CARTESIAN_POINT") : nullptr;
        const std::vector<double> c = sub ? coords(sub->params) : std::vector<double>();

        if (c.size() >= 3)
            pt = Point(c[0], c[1], c[2]);
        else if (c.size() == 2)
            pt = Point(c[0], c[1], 0.0);

        return pt_cache[id] = pt;
    }

    /// Read the CARTESIAN_POINT of a VERTEX_POINT, none when missing.
    std::optional<Point> get_vertex_point(int id) {

        const StepEntity* e = get(id);
        const StepSubEntity* sub = e ? e->find("VERTEX_POINT") : nullptr;
        const int ref = sub ? first_ref(sub->params) : -1;

        if (ref < 0)
            return std::nullopt;

        return get_point(ref);
    }

    /// Read a DIRECTION as a unit vector, caching by id.
    Vector get_direction(int id) {

        const auto it = dir_cache.find(id);

        if (it != dir_cache.end())
            return it->second;

        Vector v(0, 0, 1);
        const StepEntity* e = get(id);
        const StepSubEntity* sub = e ? e->find("DIRECTION") : nullptr;
        const std::vector<double> c = sub ? coords(sub->params) : std::vector<double>();

        if (c.size() >= 3)
            v = Vector(c[0], c[1], c[2]);

        return dir_cache[id] = v;
    }

    /// AXIS2_PLACEMENT_3D as an orthonormal frame: az normalized, ax made orthogonal to it, ay = az x ax.
    Axis2 get_axis2(int id) {

        const auto it = ax_cache.find(id);

        if (it != ax_cache.end())
            return it->second;

        Axis2 a;
        const StepEntity* e = get(id);
        const StepSubEntity* sub = e ? e->find("AXIS2_PLACEMENT_3D") : nullptr;
        const std::vector<int> refs = sub ? all_refs(sub->params) : std::vector<int>();

        if (refs.empty())
            return ax_cache[id] = a;

        a.origin = get_point(refs[0]);
        a.az = refs.size() > 1 ? get_direction(refs[1]) : Vector(0, 0, 1);

        const double ln = a.az.magnitude();

        if (ln > 1e-12)
            a.az = a.az * (1.0 / ln);

        if (refs.size() > 2)
            a.ax = get_direction(refs[2]);
        else {
            // No reference direction: the X gp_Ax2(P, V) takes, so angles start where OCCT's do.
            const double x = a.az[0], y = a.az[1], z = a.az[2], ax = std::abs(x), ay = std::abs(y), az = std::abs(z);
            a.ax = ay <= ax && ay <= az ? (ax > az ? Vector(-z, 0, x) : Vector(z, 0, -x)) : ax <= ay && ax <= az ? (ay > az ? Vector(0, -z, y) : Vector(0, z, -y)) : ax > ay ? Vector(-y, x, 0) : Vector(y, -x, 0);
        }

        a.ax = a.ax - a.az * a.ax.dot(a.az);

        const double xn = a.ax.magnitude();

        if (xn > 1e-12)
            a.ax = a.ax * (1.0 / xn);

        a.ay = a.az.cross(a.ax);
        a.ok = true;

        return ax_cache[id] = a;
    }

    /// B_SPLINE_CURVE_WITH_KNOTS, simple or complex with RATIONAL_B_SPLINE_CURVE; invalid when malformed.
    NurbsCurve get_nurbs_curve(int id) {

        const StepEntity* e = get(id);
        const std::optional<CurveParams> cp = e ? curve_params(*e) : std::nullopt;

        if (!cp)
            return NurbsCurve();

        const int order = cp->degree + 1;
        const int cv_count = (int)cp->pt_refs.size();
        const std::vector<double> full = expand_knots(cp->knots, cp->mults);

        if ((int)full.size() != cv_count + order)
            return NurbsCurve();

        const std::vector<double> internal = internal_from_full(full);
        const StepSubEntity* rat = e->find("RATIONAL_B_SPLINE_CURVE");
        const bool is_rat = rat != nullptr;
        const std::vector<double> weights =
            is_rat && !rat->params.empty() ? dbl_list(rat->params[0]) : std::vector<double>();

        NurbsCurve nc(3, is_rat, order, cv_count);

        if (nc.m_nurbsknot.size() != internal.size())
            return NurbsCurve();

        nc.m_nurbsknot = internal;

        for (int i = 0; i < cv_count; i++) {
            const Point pt = get_point(cp->pt_refs[i]);
            const double w = is_rat && i < (int)weights.size() ? weights[i] : 1.0;

            if (!nc.set_cv_4d(i, w * pt[0], w * pt[1], w * pt[2], w))
                return NurbsCurve();
        }

        return nc;
    }

    /// B_SPLINE_SURFACE_WITH_KNOTS, simple or complex with RATIONAL_B_SPLINE_SURFACE; invalid when malformed.
    NurbsSurface get_nurbs_surface(int id) {

        const StepEntity* e = get(id);
        const std::optional<SurfaceParams> sp = e ? surface_params(*e) : std::nullopt;

        if (!sp)
            return NurbsSurface();

        const int cv_u = (int)sp->ctrl_pts.size();
        const int cv_v = (int)sp->ctrl_pts[0].size();
        const std::vector<double> full_u = expand_knots(sp->u_knots, sp->u_mults);
        const std::vector<double> full_v = expand_knots(sp->v_knots, sp->v_mults);

        if ((int)full_u.size() != cv_u + sp->u_deg + 1 || (int)full_v.size() != cv_v + sp->v_deg + 1)
            return NurbsSurface();

        const StepSubEntity* rat = e->find("RATIONAL_B_SPLINE_SURFACE");
        const bool is_rat = rat != nullptr;
        const std::vector<std::vector<double>> weights =
            is_rat && !rat->params.empty() ? dbl_list_list(rat->params[0]) : std::vector<std::vector<double>>();

        NurbsSurface srf(3, is_rat, sp->u_deg + 1, sp->v_deg + 1, cv_u, cv_v);
        srf.m_nurbsknot[0] = internal_from_full(full_u);
        srf.m_nurbsknot[1] = internal_from_full(full_v);

        for (int u = 0; u < cv_u; u++)
            for (int v = 0; v < cv_v && v < (int)sp->ctrl_pts[u].size(); v++) {
                const Point pt = get_point(sp->ctrl_pts[u][v]);
                const double w = is_rat && u < (int)weights.size() && v < (int)weights[u].size() ? weights[u][v] : 1.0;

                if (!srf.set_cv_4d(u, v, w * pt[0], w * pt[1], w * pt[2], w))
                    return NurbsSurface();
            }

        return srf.is_valid() ? srf : NurbsSurface();
    }

    /// The SURFACE_CURVE, SEAM_CURVE or INTERSECTION_CURVE part of a curve entity, or null.
    const StepSubEntity* surface_curve(int curve_id) const {

        const StepEntity* e = get(curve_id);

        for (const char* type : {"SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"})
            if (const StepSubEntity* sc = e ? e->find(type) : nullptr)
                return sc;

        return nullptr;
    }

    /// The 3D basis curve behind a SURFACE_CURVE, SEAM_CURVE or INTERSECTION_CURVE, the id itself otherwise.
    int basis_curve_of(int curve_id) const {

        const StepSubEntity* sc = surface_curve(curve_id);

        const int ref = sc ? first_ref(sc->params) : -1;

        return ref >= 0 ? ref : curve_id;
    }

    /// n points along a curve entity: a B-spline or a circle arc between the vertices, else the two vertices.
    std::vector<Point> sample_curve(int curve_id, const Point& v_start, const Point& v_end, int n) {

        const std::vector<Point> ends = {v_start, v_end};
        int id = basis_curve_of(curve_id);

        for (int depth = 0; depth < MAX_DEPTH; depth++) {
            const StepEntity* e = get(id);
            const StepSubEntity* tc = e ? e->find("TRIMMED_CURVE") : nullptr;

            if (!tc)
                break;

            id = basis_curve_of(first_ref(tc->params));
        }

        const StepEntity* e = get(id);

        if (!e)
            return ends;

        if (e->has("B_SPLINE_CURVE_WITH_KNOTS")) {
            const NurbsCurve nc = get_nurbs_curve(id);

            return nc.is_valid() ? sample_nurbs(nc, n) : ends;
        }

        const StepSubEntity* circle = e->find("CIRCLE");

        if (!circle)
            return ends;

        const int ax_ref = first_ref(circle->params);
        const std::vector<double> rr = nums(circle->params);
        const double rad = rr.empty() ? 0.0 : rr[0];
        const Axis2 a = get_axis2(ax_ref);

        if (ax_ref < 0 || rad == 0 || !a.ok)
            return ends;

        const double sa = angle_of(a, v_start);
        double ea = angle_of(a, v_end);

        if (ea <= sa)
            ea += 2.0 * PI;

        std::vector<Point> pts;

        for (int i = 0; i < n; i++) {
            const double ang = n > 1 ? sa + (ea - sa) * i / (n - 1) : sa;
            pts.push_back(a.origin + (a.ax * std::cos(ang) + a.ay * std::sin(ang)) * rad);
        }

        return pts;
    }

    /// Return the parameter projector of a surface, caching by id.
    Proj get_projector(int surface_id) {

        Proj pr;
        const StepEntity* e = get(surface_id);
        const StepSubEntity* plane = e ? e->find("PLANE") : nullptr;
        const int ref = plane ? first_ref(plane->params) : -1;

        if (ref < 0)
            return pr;

        pr.a = get_axis2(ref);
        pr.kind = 1;

        return pr;
    }

    /// Kernel surface of a surface entity: the B-spline itself, or a plane patch over the padded uv window.
    NurbsSurface fill_surface(int id, double u0, double u1, double v0, double v1) {

        const StepEntity* e = get(id);

        if (e && surface_params(*e))
            return get_nurbs_surface(id);

        const StepSubEntity* plane = e ? e->find("PLANE") : nullptr;
        const int ax_ref = plane ? first_ref(plane->params) : -1;
        const Axis2 a = get_axis2(ax_ref);

        if (ax_ref < 0 || !a.ok)
            return NurbsSurface();

        const double pad_u = std::max(1e-6, 0.01 * (u1 - u0));
        const double pad_v = std::max(1e-6, 0.01 * (v1 - v0));

        return plane_surface(a, u0 - pad_u, u1 + pad_u, v0 - pad_v, v1 + pad_v);
    }

    /// Axis of a SURFACE_OF_REVOLUTION as a frame (origin on the axis, ax toward the swept B-spline's first control point off the axis, az along the axis) with that B-spline; none otherwise.
    std::optional<std::pair<Axis2, NurbsCurve>> revolution(int id) {

        const StepEntity* e = get(id);
        const StepSubEntity* sub = e ? e->find("SURFACE_OF_REVOLUTION") : nullptr;
        const std::vector<int> refs = sub ? all_refs(sub->params) : std::vector<int>();
        const StepSubEntity* axis = refs.size() > 1 && get(refs[1]) ? get(refs[1])->find("AXIS1_PLACEMENT") : nullptr;
        const std::vector<int> ax_refs = axis ? all_refs(axis->params) : std::vector<int>();
        const NurbsCurve profile = ax_refs.size() > 1 ? get_nurbs_curve(refs[0]) : NurbsCurve();

        if (!profile.is_valid())
            return std::nullopt;

        Axis2 a;
        a.origin = get_point(ax_refs[0]);
        a.az = get_direction(ax_refs[1]);
        a.az = a.az * (1.0 / a.az.magnitude());

        for (int j = 0; j < profile.cv_count() && !a.ok; j++) {
            const Vector r = profile.get_cv(j) - a.origin;
            const Vector radial = r - a.az * r.dot(a.az);

            if (radial.magnitude() > 1e-9 * (1.0 + r.magnitude())) {
                a.ax = radial * (1.0 / radial.magnitude());
                a.ay = a.az.cross(a.ax);
                a.ok = true;
            }
        }

        return a.ok ? std::optional<std::pair<Axis2, NurbsCurve>>({a, profile}) : std::nullopt;
    }

    /// CYLINDRICAL, CONICAL, SPHERICAL or TOROIDAL_SURFACE as an analytic face; kind 0 otherwise.
    AnFace get_analytic_srf(int id) {

        AnFace an;
        const StepEntity* e = get(id);

        if (!e)
            return an;

        const std::vector<std::string> kinds =
            {"CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE", "TOROIDAL_SURFACE"};

        for (size_t i = 0; i < kinds.size(); i++) {
            const StepSubEntity* sub = e->find(kinds[i]);

            if (!sub)
                continue;

            const int ax_ref = first_ref(sub->params);

            if (ax_ref < 0)
                return an;

            an.a = get_axis2(ax_ref);

            if (!an.a.ok)
                return an;

            const std::vector<double> rr = nums(sub->params);
            an.kind = (int)i + 2;
            an.radius = rr.empty() ? 0.0 : rr[0];
            an.r2 = rr.size() > 1 ? rr[1] * (an.kind == 3 ? angle_unit() : 1.0) : 0.0;

            return an;
        }

        return an;
    }

    /// Source 2D analytic frame; do not interpret a 2D direction as a 3D default axis.
    Axis2 pcurve_circle_axis(int id) {
        const StepEntity* entity = get(id);
        const StepSubEntity* placement = entity ? entity->find("AXIS2_PLACEMENT_2D") : nullptr;
        if (!placement) return get_axis2(id);
        const auto references = all_refs(placement->params);
        Axis2 axis;
        if (references.empty()) return axis;
        const StepEntity* origin = get(references[0]);
        const StepSubEntity* point = origin ? origin->find("CARTESIAN_POINT") : nullptr;
        const auto xy = point ? coords(point->params) : std::vector<double>();
        if (xy.size() != 2 || !std::isfinite(xy[0]) || !std::isfinite(xy[1])) return axis;
        axis.origin = Point(xy[0], xy[1], 0);
        if (references.size() > 1) {
            const StepEntity* direction = get(references[1]);
            const StepSubEntity* item = direction ? direction->find("DIRECTION") : nullptr;
            const auto values = item ? coords(item->params) : std::vector<double>();
            if (values.size() != 2 || !std::isfinite(values[0]) || !std::isfinite(values[1])) return axis;
            const double norm = std::hypot(values[0], values[1]);
            if (!(norm > 0) || !std::isfinite(norm)) return axis;
            axis.ax = Vector(values[0] / norm, values[1] / norm, 0);
        }
        axis.ay = Vector(-axis.ax[1], axis.ax[0], 0);
        axis.ok = true;
        return axis;
    }

    /// Transfer the original rational cos/sin law, not equal-angle samples, to its source UV circle.
    NurbsCurve source_circle_pcurve(int geometry_id, int pcurve_id, const NurbsCurve& world) {
        const StepEntity* source = get(basis_curve_of(geometry_id));
        const StepSubEntity* circle = source ? source->find("CIRCLE") : nullptr;
        const StepEntity* image = get(pcurve_id);
        const StepSubEntity* pcircle = image ? image->find("CIRCLE") : nullptr;
        if (!circle || !pcircle || !world.is_valid() || world.dimension() != 3) return NurbsCurve();
        const Axis2 from = get_axis2(first_ref(circle->params));
        const Axis2 to = pcurve_circle_axis(first_ref(pcircle->params));
        const auto radii = nums(circle->params), pradii = nums(pcircle->params);
        if (!from.ok || !to.ok || radii.empty() || pradii.empty() || !(radii[0] > 0) ||
            !(pradii[0] > 0) || !std::isfinite(radii[0]) || !std::isfinite(pradii[0]) ||
            to.origin[2] != 0 || to.ax[2] != 0 || to.ay[2] != 0) return NurbsCurve();
        const double ratio = pradii[0] / radii[0];
        if (!std::isfinite(ratio)) return NurbsCurve();
        NurbsCurve result(3, world.is_rational(), world.order(), world.cv_count());
        result.m_nurbsknot = world.m_nurbsknot;
        for (int i = 0; i < world.cv_count(); ++i) {
            const double weight = world.weight(i);
            if (!(weight > 0) || !std::isfinite(weight)) return NurbsCurve();
            const Vector radial = world.get_cv(i) - from.origin;
            const Point uv = to.origin + to.ax * (ratio * radial.dot(from.ax)) +
                to.ay * (ratio * radial.dot(from.ay));
            if (!std::isfinite(uv[0]) || !std::isfinite(uv[1]) ||
                !result.set_cv_4d(i, uv[0] * weight, uv[1] * weight, 0, weight)) return NurbsCurve();
        }
        return result.is_valid() ? result : NurbsCurve();
    }

    /// Point and scaled direction of a LINE (its VECTOR's orientation times magnitude); none otherwise.
    std::optional<std::pair<Point, Vector>> line_of(int id) {
        const StepEntity* e = get(id);
        const StepSubEntity* line = e ? e->find("LINE") : nullptr;
        const std::vector<int> refs = line ? all_refs(line->params) : std::vector<int>();
        const StepEntity* v = refs.size() > 1 ? get(refs[1]) : nullptr;
        const StepSubEntity* vector = v ? v->find("VECTOR") : nullptr;
        const std::vector<double> magnitude = vector ? nums(vector->params) : std::vector<double>();
        if (!vector || magnitude.empty()) return std::nullopt;
        const StepEntity* d = get(first_ref(vector->params));
        const std::vector<double> xyz = d && d->find("DIRECTION") ? coords(d->find("DIRECTION")->params) : std::vector<double>();
        if (xyz.size() < 2) return std::nullopt;
        return std::make_pair(get_point(refs[0]), Vector(xyz[0], xyz[1], xyz.size() > 2 ? xyz[2] : 0.0) * magnitude[0]);
    }

    /// A 2D LINE pcurve over the parameters its edge's 3D basis takes at the world curve's ends (a SURFACE_CURVE shares one parameter).
    NurbsCurve source_line_pcurve(int geometry_id, int pcurve_id, const NurbsCurve& world) {
        const int basis = basis_curve_of(geometry_id);
        const auto image = line_of(pcurve_id);
        const auto line3 = line_of(basis);
        const auto parameter = [&](const Point& p) {
            if (line3) return (p - line3->first).dot(line3->second) / line3->second.dot(line3->second);
            const NurbsCurve c = get_nurbs_curve(basis);
            return c.is_valid() ? c.closest_parameter(p) : NAN;
        };
        const double t0 = parameter(world.point_at_start()), t1 = parameter(world.point_at_end());
        if (!image || !std::isfinite(t0) || !std::isfinite(t1) || t0 == t1) return NurbsCurve();
        return NurbsCurve::create(false, 1, {image->first + image->second * t0, image->first + image->second * t1});
    }

    /// A 2D TRIMMED_CURVE of a B-spline or LINE between PARAMETER_VALUE trims; invalid otherwise.
    NurbsCurve trimmed_pcurve(int id) {
        const StepEntity* e = get(id);
        const StepSubEntity* trim = e ? e->find("TRIMMED_CURVE") : nullptr;
        if (!trim || trim->params.size() < 4) return NurbsCurve();
        double t[2] = {NAN, NAN};
        for (int k = 0; k < 2; ++k)
            for (const StepParam& item : trim->params[2 + k].list)
                if (item.str == "PARAMETER_VALUE" && !item.list.empty()) t[k] = item.list[0].num;
        const int basis = first_ref(trim->params);
        NurbsCurve c = get_nurbs_curve(basis);
        if (const auto line = line_of(basis)) c = NurbsCurve::create(false, 1, {line->first + line->second * t[0], line->first + line->second * t[1]});
        else if (!c.is_valid() || !std::isfinite(t[0] + t[1]) || !c.trim(std::min(t[0], t[1]), std::max(t[0], t[1])) || (t[0] > t[1] && !c.reverse())) return NurbsCurve();
        return c;
    }

    /// All original NURBS pcurves on this exact support, including like-seams.
    std::vector<NurbsCurve> original_pcurves(int geometry_id, int surface_id, const NurbsCurve& world) {
        const StepSubEntity* source = surface_curve(geometry_id);
        std::vector<NurbsCurve> curves;
        if (!source) return curves;
        for (int associated : list_refs(source->params)) {
            const StepEntity* item = get(associated);
            const StepSubEntity* pc = item ? item->find("PCURVE") : nullptr;
            const auto references = pc ? all_refs(pc->params) : std::vector<int>();
            if (references.size() < 2 || references[0] != surface_id) continue;
            const StepEntity* representation = get(references[1]);
            const StepSubEntity* definition = representation ? representation->find("DEFINITIONAL_REPRESENTATION") : nullptr;
            if (definition && definition->params.empty()) definition = representation->find("REPRESENTATION");
            const auto items = definition ? list_refs(definition->params) : std::vector<int>();
            if (items.empty()) continue;
            NurbsCurve curve = get_nurbs_curve(items.front());
            if (!curve.is_valid()) curve = trimmed_pcurve(items.front());
            if (!curve.is_valid()) curve = source_circle_pcurve(geometry_id, items.front(), world);
            if (!curve.is_valid()) curve = source_line_pcurve(geometry_id, items.front(), world);
            if (curve.is_valid()) curves.push_back(std::move(curve));
        }
        return curves;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// Topology access
// ═══════════════════════════════════════════════════════════════════════════
/// One face bound: outer flag, orientation and the ORIENTED_EDGE ids of its EDGE_LOOP.
struct Bound {
    bool is_outer = false; // Whether the bound is FACE_OUTER_BOUND.
    bool orient = true; // Bound orientation flag.
    std::vector<int> oe_refs; // ORIENTED_EDGE ids.
};

/// Read a FACE_BOUND or FACE_OUTER_BOUND with its EDGE_LOOP.
static std::optional<Bound> bound_loop(const StepReader& r, int bid) {

    const StepEntity* bent = r.get(bid);
    const StepSubEntity* bsub = bent ? bent->find("FACE_OUTER_BOUND") : nullptr;

    if (bent && !bsub)
        bsub = bent->find("FACE_BOUND");

    if (!bsub)
        return std::nullopt;

    const StepEntity* lent = r.get(first_ref(bsub->params));
    const StepSubEntity* loop = lent ? lent->find("EDGE_LOOP") : nullptr;

    if (!loop)
        return std::nullopt;

    Bound b;
    b.is_outer = bent->has("FACE_OUTER_BOUND");
    b.orient = last_flag(bsub->params, true);
    b.oe_refs = list_refs(loop->params);

    return b;
}

/// EDGE_CURVE id (-1 when missing) and orientation of an ORIENTED_EDGE.
static std::pair<int, bool> oriented_edge(const StepReader& r, int oe_id) {

    const StepEntity* oent = r.get(oe_id);
    const StepSubEntity* oe = oent ? oent->find("ORIENTED_EDGE") : nullptr;

    if (!oe)
        return {-1, true};

    const std::vector<int> refs = all_refs(oe->params);

    return {refs.empty() ? -1 : refs.back(), last_flag(oe->params, true)};
}

/// Start vertex, end vertex and geometry ids of an EDGE_CURVE; empty when missing.
static std::vector<int> edge_refs(const StepReader& r, int ec_ref) {

    const StepEntity* ecent = r.get(ec_ref);
    const StepSubEntity* ec = ecent ? ecent->find("EDGE_CURVE") : nullptr;

    return ec ? all_refs(ec->params) : std::vector<int>();
}

/// Return the geometry id of an EDGE_CURVE, or -1.
static int edge_geom_id(const StepReader& r, int ec_ref) {

    const std::vector<int> refs = edge_refs(r, ec_ref);

    return refs.size() >= 3 ? refs[2] : -1;
}

// ═══════════════════════════════════════════════════════════════════════════
// BRep assembly from STEP
// ═══════════════════════════════════════════════════════════════════════════
/// One edge use in loop-traversal order; c2d is flipped into the edge direction when stored.
struct PendingEdge {
    int edge = -1; // Brep edge index.
    bool reversed = false; // Whether the edge runs against the loop.
    NurbsCurve c2d; // Parameter-space curve.
    NurbsCurve along; // The curve in the edge's direction when it came so, stored without reversing it twice (not exact in floating point).
};

/// One edge use with its parameter-space samples in curve order; pc2d is an exact pcurve when exact.
struct LoopEdge {
    int edge_idx = -1; // Brep edge index.
    bool reversed = false; // Whether the edge runs against the loop.
    std::vector<Point> uv; // Sampled uv points.
    NurbsCurve pc2d; // Parameter-space curve.
    bool exact = false; // Whether pc2d is exact rather than sampled.
    bool source_pcurve = false; // Original chart; never shift only its UV samples.
    double error = INFINITY; // Deviation of a file pcurve that is not certified, the fit's fallback.
};

/// One face loop with its edge uses in traversal order.
struct Loop {
    bool is_outer = false; // Whether the loop is the outer boundary.
    bool projected = false; // Whether the uv came from projection.
    std::vector<LoopEdge> edges; // Edges in traversal order.
};

/// Chart window of an analytic face: quarter arcs from su0 in u, from sv0 in v for sphere and torus, [t0, t1] otherwise.
struct Window {
    int su0 = 0; // First quarter arc in u.
    int nsu = 0; // Quarter arcs in u.
    int sv0 = 0; // First quarter arc in v.
    int nsv = 0; // Quarter arcs in v.
    double t0 = 0.0; // Start of the linear domain.
    double t1 = 0.0; // End of the linear domain.
};

/// (umin, umax, vmin, vmax) over the samples of one loop; umin > umax when there are none.
static std::tuple<double, double, double, double> loop_bounds(const Loop& lp) {

    double umin = 1e300;
    double umax = -1e300;
    double vmin = 1e300;
    double vmax = -1e300;

    for (const LoopEdge& le : lp.edges)
        for (const Point& p : le.uv) {
            umin = std::min(umin, p[0]);
            umax = std::max(umax, p[0]);
            vmin = std::min(vmin, p[1]);
            vmax = std::max(vmax, p[1]);
        }

    return {umin, umax, vmin, vmax};
}

/// Return the uv bounds over every loop.
static std::tuple<double, double, double, double> loops_bounds(const std::vector<Loop>& loops) {

    double umin = 1e300;
    double umax = -1e300;
    double vmin = 1e300;
    double vmax = -1e300;

    for (const Loop& lp : loops) {
        double u0 = 0.0;
        double u1 = 0.0;
        double v0 = 0.0;
        double v1 = 0.0;
        std::tie(u0, u1, v0, v1) = loop_bounds(lp);
        umin = std::min(umin, u0);
        umax = std::max(umax, u1);
        vmin = std::min(vmin, v0);
        vmax = std::max(vmax, v1);
    }

    return {umin, umax, vmin, vmax};
}

/// Mark the loop with the largest uv extent as outer when none is marked (OCCT and FreeCAD write FACE_BOUND for the outer boundary).
static void pick_outer_loop(std::vector<Loop>& loops) {

    for (const Loop& l : loops)
        if (l.is_outer)
            return;

    if (loops.empty())
        return;

    size_t best = 0;
    double best_a = -1.0;

    for (size_t i = 0; i < loops.size(); i++) {
        double u0 = 0.0;
        double u1 = 0.0;
        double v0 = 0.0;
        double v1 = 0.0;
        std::tie(u0, u1, v0, v1) = loop_bounds(loops[i]);

        const double a = u1 > u0 && v1 > v0 ? (u1 - u0) * (v1 - v0) : 0.0;

        if (a > best_a) {
            best_a = a;
            best = i;
        }
    }

    loops[best].is_outer = true;
}

/// Reorder so outer loops come before inner ones.
static void outer_first(std::vector<Loop>& loops) {

    std::vector<Loop> ordered;

    for (const Loop& l : loops)
        if (l.is_outer)
            ordered.push_back(l);

    for (const Loop& l : loops)
        if (!l.is_outer)
            ordered.push_back(l);

    loops = ordered;
}

/// Mean u of the samples of a loop, none when it has no samples.
static std::optional<double> loop_ucenter(const Loop& lp) {

    double sum = 0.0;
    int cnt = 0;

    for (const LoopEdge& le : lp.edges)
        for (const Point& p : le.uv) {
            sum += p[0];
            cnt++;
        }

    if (cnt == 0)
        return std::nullopt;

    return sum / cnt;
}

/// Periods of a B-spline surface: the domain span of each closed direction, 0 when open.
static std::pair<double, double> surface_periods(const NurbsSurface& proj_srf) {

    if (!proj_srf.is_valid())
        return {0.0, 0.0};

    double du0 = 0.0;
    double du1 = 0.0;
    double dv0 = 0.0;
    double dv1 = 0.0;
    std::tie(du0, du1) = proj_srf.domain(0);
    std::tie(dv0, dv1) = proj_srf.domain(1);

    const double scale = proj_srf.point_at(du0, dv0).distance(proj_srf.point_at(du1, dv1)) + 1e-9;
    bool closed_u = true;
    bool closed_v = true;

    for (int k = 0; k <= 4; k++) {
        const double fu = du0 + (du1 - du0) * k / 4.0;
        const double fv = dv0 + (dv1 - dv0) * k / 4.0;

        if (proj_srf.point_at(du0, fv).distance(proj_srf.point_at(du1, fv)) > scale * 1e-6)
            closed_u = false;

        if (proj_srf.point_at(fu, dv0).distance(proj_srf.point_at(fu, dv1)) > scale * 1e-6)
            closed_v = false;
    }

    return {closed_u ? du1 - du0 : 0.0, closed_v ? dv1 - dv0 : 0.0};
}

/// Shift each edge by whole periods so its traversal start meets the previous edge's end.
static void chain_loops(std::vector<Loop>& loops, double tau_u, double tau_v) {

    if (tau_u <= 0.0 && tau_v <= 0.0)
        return;

    for (Loop& lp : loops) {
        if (std::any_of(lp.edges.begin(), lp.edges.end(), [](const LoopEdge& edge) { return edge.source_pcurve; }))
            continue;
        Point prev_end(0, 0, 0);
        bool have_prev = false;

        for (LoopEdge& le : lp.edges) {
            if (le.uv.empty())
                continue;

            // Samples of one edge stay on one branch where the edge reaches the seam line.
            for (size_t k = 1; k < le.uv.size(); k++) {
                le.uv[k][0] += tau_u > 0.0 ? tau_u * std::round((le.uv[k - 1][0] - le.uv[k][0]) / tau_u) : 0.0;
                le.uv[k][1] += tau_v > 0.0 ? tau_v * std::round((le.uv[k - 1][1] - le.uv[k][1]) / tau_v) : 0.0;
            }

            const Point st = le.reversed ? le.uv.back() : le.uv.front();
            const int n = have_prev && tau_u > 0.0 ? (int)std::round((prev_end[0] - st[0]) / tau_u) : 0;
            const int m = have_prev && tau_v > 0.0 ? (int)std::round((prev_end[1] - st[1]) / tau_v) : 0;

            for (Point& p : le.uv) {
                p[0] += n * tau_u;
                p[1] += m * tau_v;
            }

            prev_end = le.reversed ? le.uv.front() : le.uv.back();
            have_prev = true;
        }
    }
}

/// Seam aliases on a closed chart: when both uses of an edge in one loop lie on one branch, one moves by a period, the use whose other neighbour then meets it, else the use running in +v (+u) to the larger u (smaller v).
static void split_seams(std::vector<Loop>& loops, double tau_u, double tau_v) {

    const auto first = [](const LoopEdge& le) { return le.reversed ? le.uv.back() : le.uv.front(); };
    const auto last = [](const LoopEdge& le) { return le.reversed ? le.uv.front() : le.uv.back(); };

    for (Loop& lp : loops) {
        const size_t n = lp.edges.size();

        for (size_t i = 0; i < n; i++)
            for (size_t j = i + 1; j < n; j++) {
                LoopEdge& a = lp.edges[i];
                LoopEdge& b = lp.edges[j];

                if (a.edge_idx != b.edge_idx || a.uv.size() < 2 || b.uv.size() < 2 || a.source_pcurve || b.source_pcurve)
                    continue;

                const Vector run = last(a) - first(a);
                const int fixed = std::abs(run[0]) <= std::abs(run[1]) ? 0 : 1;
                const double tau = fixed == 0 ? tau_u : tau_v;

                if (!(tau > 0.0) || std::abs(a.uv[0][fixed] - b.uv[0][fixed]) > 0.5 * tau)
                    continue;

                int shift = 0;
                LoopEdge* moved = nullptr;

                for (const auto& [k, other] : {std::make_pair(i, j), std::make_pair(j, i)}) {
                    const LoopEdge& next = lp.edges[(k + 1) % n];
                    const LoopEdge& prev = lp.edges[(k + n - 1) % n];
                    const int to_next = (k + 1) % n == other ? 0 : (int)std::round((first(next)[fixed] - last(lp.edges[k])[fixed]) / tau);
                    const int to_prev = (k + n - 1) % n == other ? 0 : (int)std::round((last(prev)[fixed] - first(lp.edges[k])[fixed]) / tau);

                    if (!moved && (to_next != 0 || to_prev != 0)) {
                        moved = &lp.edges[k];
                        shift = to_next != 0 ? to_next : to_prev;
                    }
                }

                if (!moved) {
                    moved = (fixed == 0) == (run[1 - fixed] > 0.0) ? &a : &b;
                    shift = 1;
                }

                for (Point& p : moved->uv)
                    p[fixed] += shift * tau;
            }
    }
}

/// Loops on a closed surface moved together by whole periods so their middle lies in its domain: chaining starts wherever the first edge does, a seam line included.
static void domain_loops(std::vector<Loop>& loops, const NurbsSurface& srf, double tau_u, double tau_v) {

    for (const Loop& lp : loops)
        if (std::any_of(lp.edges.begin(), lp.edges.end(), [](const LoopEdge& edge) { return edge.source_pcurve; }))
            return;

    double box[4] = {0.0, 0.0, 0.0, 0.0};
    std::tie(box[0], box[1], box[2], box[3]) = loops_bounds(loops);

    for (int d = 0; d < 2; d++) {
        const double tau = d == 0 ? tau_u : tau_v;
        const double n = tau > 0.0 ? std::floor((0.5 * (box[2 * d] + box[2 * d + 1]) - srf.domain(d).first) / tau) : 0.0;

        for (Loop& lp : loops)
            for (LoopEdge& le : lp.edges)
                for (Point& p : le.uv)
                    p[d] -= n * tau;
    }
}

/// Shift each inner loop by whole u periods onto the outer loop's u window.
static void center_inner_loops(std::vector<Loop>& loops, double tau_u) {

    if (tau_u <= 0.0)
        return;

    // The middle of the outer loop's span: a hole anywhere in a full turn then stays within half a turn of it.
    std::optional<double> outer;

    for (const Loop& lp : loops)
        if (lp.is_outer) {
            double lo = 1e300, hi = -1e300;

            for (const LoopEdge& le : lp.edges)
                for (const Point& p : le.uv)
                    lo = std::min(lo, p[0]), hi = std::max(hi, p[0]);

            if (lo <= hi)
                outer = 0.5 * (lo + hi);

            break;
        }

    if (!outer)
        return;

    for (Loop& lp : loops) {
        if (std::any_of(lp.edges.begin(), lp.edges.end(), [](const LoopEdge& edge) { return edge.source_pcurve; }))
            continue;
        const std::optional<double> center = lp.is_outer ? std::nullopt : loop_ucenter(lp);

        if (!center)
            continue;

        const int n = (int)std::round((*outer - *center) / tau_u);

        for (LoopEdge& le : lp.edges)
            for (Point& p : le.uv)
                p[0] += n * tau_u;
    }
}

/// Pending edges of a loop in traversal order: the exact pcurve when there is one, else the sampled polyline.
static std::vector<PendingEdge> pending_of(const Loop& lp) {

    std::vector<PendingEdge> pl;

    for (const LoopEdge& le : lp.edges) {
        NurbsCurve crv2d = le.pc2d;

        if (le.reversed && !crv2d.reverse())
            throw std::runtime_error("STEP pcurve could not be reversed");

        pl.push_back({le.edge_idx, le.reversed, crv2d, le.pc2d});
    }

    return pl;
}

/// Parameter-space images of 3D samples: the analytic projection, or a warm-started closest-point search on proj_srf.
static std::vector<Point> uv_of_samples(
    const Proj& proj,
    const NurbsSurface& proj_srf,
    const std::vector<Point>& samples
) {

    std::vector<Point> uv;

    if (proj.kind != 0) {
        for (const Point& s : samples) {
            double u = 0.0;
            double v = 0.0;
            std::tie(u, v) = project(proj, s);
            uv.emplace_back(u, v, 0.0);
        }

        return uv;
    }

    if (!proj_srf.is_valid() || samples.empty())
        return {Point(0, 0, 0), Point(1, 0, 0)};

    double du0 = 0.0;
    double du1 = 0.0;
    double dv0 = 0.0;
    double dv1 = 0.0;
    std::tie(du0, du1) = proj_srf.domain(0);
    std::tie(dv0, dv1) = proj_srf.domain(1);

    const double wu = (du1 - du0) * 0.1;
    const double wv = (dv1 - dv0) * 0.1;
    double d_ref = 0.0;
    double pu = 0.0;
    double pv = 0.0;

    for (size_t k = 0; k < samples.size(); k++) {
        double u = 0.0;
        double v = 0.0;
        double d = 0.0;

        if (k == 0) {
            std::tie(u, v, d) = Closest::surface_point(proj_srf, samples[k]);
            d_ref = d;
        } else {
            std::tie(u, v, d) = Closest::surface_point(proj_srf, samples[k], pu - wu, pu + wu, pv - wv, pv + wv);

            if (d > 10 * d_ref + 1e-9)
                std::tie(u, v, d) = Closest::surface_point(proj_srf, samples[k]);
        }

        uv.emplace_back(u, v, 0.0);
        pu = u;
        pv = v;
    }

    return uv;
}

/// Map a surface parameter point into the window chart.
static Point chart_point(const AnFace& an, const Window& w, const Point& p) {

    const bool angular = an.kind == 4 || an.kind == 5;

    return Point(chart_u_of_angle(p[0], w.su0), angular ? chart_u_of_angle(p[1], w.sv0) : p[1], 0.0);
}

/// Evaluate the analytic surface at a window chart point.
static Point chart_eval(const AnFace& an, const Window& w, const Point& q) {

    const bool angular = an.kind == 4 || an.kind == 5;

    return an_eval(an, (w.su0 + q[0]) * PI_2, angular ? (w.sv0 + q[1]) * PI_2 : q[1]);
}

/// Edges of a loop shifted by whole turns so each meets the one before, from an edge leaving a pole or apex on: the angle is free there, so the loop closes through it.
/// An outer loop runs counterclockwise.
static void align_at_poles(Loop& lp, const AnFace& an, bool outer) {

    const size_t count = lp.edges.size();
    const auto start = [&](size_t k) { return lp.edges[k].reversed ? lp.edges[k].uv.back() : lp.edges[k].uv.front(); };
    const auto at_pole = [&](size_t k) { return !std::get<2>(an_st_of(an, an_eval(an, start(k)[0], start(k)[1]))); };
    size_t first = 0;

    while (first < count && !at_pole(first))
        first++;

    for (size_t i = 1; first < count && i < count; i++) {
        const size_t k = (first + i) % count;
        const LoopEdge& prev = lp.edges[(k + count - 1) % count];
        const double n = at_pole(k) ? 0.0 : std::round(((prev.reversed ? prev.uv.front() : prev.uv.back())[0] - start(k)[0]) / (2.0 * PI));

        for (Point& p : lp.edges[k].uv)
            p[0] += n * 2.0 * PI;
    }

    // An outer loop through two poles (a lune) bounds the side on its left: the stretch from its last pole moves by a turn while the loop runs clockwise.
    size_t last = first;

    for (size_t i = 1; first < count && i < count; i++)
        last = at_pole((first + i) % count) ? (first + i) % count : last;

    const auto area = [&]() {
        double a = 0.0;
        Point before = lp.edges.back().reversed ? lp.edges.back().uv.front() : lp.edges.back().uv.back();

        for (const LoopEdge& le : lp.edges)
            for (size_t m = 0; m < le.uv.size(); m++) {
                const Point& p = le.uv[le.reversed ? le.uv.size() - 1 - m : m];
                a += before[0] * p[1] - p[0] * before[1];
                before = p;
            }

        return a;
    };

    for (double turn : {2.0 * PI, -4.0 * PI})
        if (outer && last != first && area() < 0.0)
            for (size_t k = last; k != first; k = (k + 1) % count)
                for (Point& p : lp.edges[k].uv)
                    p[0] += turn;
}

/// Chart polyline of every edge of a loop, shifted by whole periods so consecutive edges of a projected loop meet, except at a pole or apex where u is free.
static std::vector<std::vector<Point>> analytic_chains(const Loop& lp, const AnFace& an, const Window& w) {

    const double period = 4.0;
    std::vector<std::vector<Point>> chains;

    for (const LoopEdge& le : lp.edges) {
        std::vector<Point> uv;

        for (const Point& p : le.uv)
            uv.push_back(chart_point(an, w, p));

        if (le.reversed)
            std::reverse(uv.begin(), uv.end());

        chains.push_back(uv);
    }

    for (size_t k = 1; lp.projected && k < chains.size(); k++) {
        const Point& st = lp.edges[k].reversed ? lp.edges[k].uv.back() : lp.edges[k].uv.front();

        if (chains[k].empty() || chains[k - 1].empty() || !std::get<2>(an_st_of(an, an_eval(an, st[0], st[1]))))
            continue;

        const int n = (int)std::round((chains[k - 1].back()[0] - chains[k].front()[0]) / period);
        const int m = an.kind == 5 ? (int)std::round((chains[k - 1].back()[1] - chains[k].front()[1]) / period) : 0;

        for (Point& p : chains[k]) {
            p[0] += n * period;
            p[1] += m * period;
        }
    }

    return chains;
}

/// Chart window of the loops; none when they are empty or wider than 16 quarter arcs.
static std::optional<Window> analytic_window(const std::vector<Loop>& loops, const AnFace& an) {

    double smin = 0.0;
    double smax = 0.0;
    double tmin = 0.0;
    double tmax = 0.0;
    std::tie(smin, smax, tmin, tmax) = loops_bounds(loops);

    if (smin > smax)
        return std::nullopt;

    Window w;
    w.su0 = (int)std::floor(smin / PI_2 + 1e-9);
    w.nsu = std::max(1, (int)std::ceil(smax / PI_2 - 1e-9) - w.su0);
    w.t0 = tmin;
    w.t1 = tmax;

    if (w.nsu > 16)
        return std::nullopt;

    if (an.kind == 4 || an.kind == 5) {
        w.sv0 = (int)std::floor(tmin / PI_2 + 1e-9);

        int sv1 = (int)std::ceil(tmax / PI_2 - 1e-9);

        if (an.kind == 4) {
            w.sv0 = std::max(w.sv0, -1);
            sv1 = std::min(sv1, 1);
        }

        w.nsv = std::max(1, sv1 - w.sv0);

        if (w.nsv > 16)
            return std::nullopt;
    } else if (tmax - tmin < 1e-12) {
        return std::nullopt;
    } else {
        bool interior = false;

        for (const Loop& lp : loops)
            for (const LoopEdge& le : lp.edges)
                for (const Point& p : le.uv)
                    interior = interior || p[1] > std::max(le.uv.front()[1], le.uv.back()[1]) + 1e-12 * (tmax - tmin) ||
                        p[1] < std::min(le.uv.front()[1], le.uv.back()[1]) - 1e-12 * (tmax - tmin);

        const double pad = interior ? 0.01 * (tmax - tmin) : 0.0;
        const double apex = an.kind == 3 && std::sin(an.r2) != 0.0 ? -an.radius / std::sin(an.r2) : NAN;
        w.t0 = apex <= tmin && apex > tmin - pad ? apex : tmin - pad;
        w.t1 = apex >= tmax && apex < tmax + pad ? apex : tmax + pad;
    }

    return w;
}

/// Canonical (s, t) where the loop left off: the end of its last edge, else the first sample that projects; false when neither exists.
static std::tuple<double, double, bool> st_start(const AnFace& an, const Loop& lp, const std::vector<Point>& ordered) {

    if (!lp.edges.empty() && !lp.edges.back().uv.empty()) {
        const LoopEdge& pe = lp.edges.back();
        const Point q = pe.reversed ? pe.uv.front() : pe.uv.back();

        return {q[0], q[1], true};
    }

    for (size_t k = 0; k < ordered.size(); k++) {
        double s = 0.0;
        double t = 0.0;
        bool ok = false;
        std::tie(s, t, ok) = an_st_of(an, ordered[k]);

        if (ok)
            return {s, t, true};
    }

    return {0.0, 0.0, false};
}

/// Canonical (s, t) of 3D samples, s (and t on a torus) shifted by whole turns next to the sample before, the first next to (ps, pt).
static std::vector<Point> st_unwrapped(
    const AnFace& an,
    const std::vector<Point>& ordered,
    double ps,
    double pt,
    bool have_prev
) {

    std::vector<Point> st;

    for (size_t k = 0; k < ordered.size(); k++) {
        double s = 0.0;
        double t = 0.0;
        bool ok = false;
        std::tie(s, t, ok) = an_st_of(an, ordered[k]);

        if (!ok && (k > 0 || have_prev))
            s = k > 0 ? st.back()[0] : ps;

        const double rs = k > 0 ? st.back()[0] : (have_prev ? ps : s);
        s -= 2 * PI * std::round((s - rs) / (2 * PI));

        if (an.kind == 5) {
            const double rt = k > 0 ? st.back()[1] : (have_prev ? pt : t);
            t -= 2 * PI * std::round((t - rt) / (2 * PI));
        }

        st.emplace_back(s, t, 0.0);
    }

    return st;
}

/// (EDGE_CURVE id, reversed) of each edge of a bound in wire order on the surface: the loop runs backwards when its orientation differs from the face's same_sense, the face then carrying the sense (StepToTopoDS_TranslateFace).
static std::vector<std::pair<int, bool>> bound_uses(const StepReader& r, const Bound& b, bool same_sense) {

    const bool flip = b.orient != same_sense;
    std::vector<std::pair<int, bool>> uses;

    for (int oe_id : b.oe_refs) {
        const std::pair<int, bool> oe = oriented_edge(r, oe_id);
        uses.push_back({oe.first, oe.second == flip});
    }

    if (flip)
        std::reverse(uses.begin(), uses.end());

    return uses;
}

/// Uv on a surface chart of a world point, on the branch next to a guess.
using Preimage = std::function<Point(const Point&, const Point&)>;

/// First and second derivatives of the preimage of a world curve at t: the surface jets solved in least squares; false where the chart is singular.
static bool uv_jets(const NurbsCurve& world, const NurbsSurface& srf, double t, const Point& uv, Vector& d1, Vector& d2) {

    std::array<xyz, 3> c;
    std::array<xyz, 6> s;

    if (!world.jet(t, 2, c) || !srf.jet(uv[0], uv[1], 2, s))
        return false;

    const double a = s[3].dot(s[3]);
    const double b = s[3].dot(s[1]);
    const double d = s[1].dot(s[1]);
    const double det = a * d - b * b;

    if (!(det > 1e-20 * a * d))
        return false;

    const auto solve = [&](const xyz& r) {
        return Vector((d * s[3].dot(r) - b * s[1].dot(r)) / det, (a * s[1].dot(r) - b * s[3].dot(r)) / det, 0.0);
    };
    d1 = solve(c[1]);
    d2 = solve(c[2] - s[5] * (d1[0] * d1[0]) - s[4] * (2.0 * d1[0] * d1[1]) - s[2] * (d1[1] * d1[1]));

    return std::isfinite(d1[0] + d1[1] + d2[0] + d2[1]);
}

/// Point of a quintic Hermite piece at fraction f.
static Point quintic_at(const nurbs_hermite::QuinticSpan& s, double f) {

    const double h = s.last - s.first;
    std::array<Point, 6> c = {s.start, s.start + s.start_derivative * (h / 5), s.start + s.start_derivative * (2 * h / 5) + s.start_second * (h * h / 20),
        s.end - s.end_derivative * (2 * h / 5) + s.end_second * (h * h / 20), s.end - s.end_derivative * (h / 5), s.end};

    for (int level = 5; level > 0; level--)
        for (int k = 0; k < level; k++)
            c[k] = c[k] + (c[k + 1] - c[k]) * f;

    return c[0];
}

/// Two clamped curves of one degree joined where the first ends and the second starts, the second's weights scaled to meet the first's; invalid otherwise.
static NurbsCurve joined(const NurbsCurve& a, const NurbsCurve& b) {

    const int p = a.degree();
    const int na = a.cv_count();
    const int nb = b.cv_count();
    NurbsCurve c(3, a.is_rational() || b.is_rational(), p + 1, na + nb - 1);

    if (b.degree() != p || !(b.weight(0) > 0.0))
        return NurbsCurve();

    c.m_nurbsknot = a.m_nurbsknot;

    for (size_t k = p; k < b.m_nurbsknot.size(); k++)
        c.m_nurbsknot.push_back(b.m_nurbsknot[k] - b.domain_start() + a.domain_end());

    const double scale = a.weight(na - 1) / b.weight(0);

    for (int i = 0; i < na + nb - 1; i++) {
        const bool first = i < na;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
        std::tie(x, y, z, w) = first ? a.get_cv_4d(i) : b.get_cv_4d(i - na + 1);
        const double f = first ? 1.0 : scale;

        if (!c.set_cv_4d(i, x * f, y * f, z * f, w * f))
            return NurbsCurve();
    }

    return c.is_valid() ? c : NurbsCurve();
}

/// Pcurve of a world curve on a surface in the world's own parameter, its ends at the preimages of first and last: rational linear pieces where the preimage keeps one coordinate (an iso line or circle, exact), else quintic Hermite pieces of the preimage and its jets; both split at the world's knots and at the surface knot lines crossed, the quintic ones halved until within tolerance / 64 of the preimage, or a quarter of the curve's distance from the surface where that is larger (ShapeFix_Edge::FixSameParameter: the tolerance carries the rest). Seeds are uv guesses at equal world parameter steps; error is the deviation from the world curve reached, its distance from the surface included. Invalid, without fitting, when that distance at the breaks already reaches budget.
static NurbsCurve fit_pcurve(const NurbsCurve& world, const NurbsSurface& srf, const Preimage& pre, const std::vector<Point>& seeds,
                             const Point& first, const Point& last, double tol, double& error, double budget = INFINITY) {

    const double a = world.domain_start();
    const double b = world.domain_end();
    const auto near = [&](const Point& p, const std::vector<Point>& guesses, double t) {
        const double f = std::clamp((t - a) / (b - a), 0.0, 1.0) * (guesses.size() - 1);
        const size_t i = std::min((size_t)f, guesses.size() - 2);

        return pre(p, guesses[i] + (guesses[i + 1] - guesses[i]) * (f - i));
    };
    // The seeds swept by continuation (ShapeAnalysis_Surface::NextValueOfUV): each preimage from the previous one, unless it lands farther than the tolerance plus twice the previous gap and the seed's lands closer.
    std::vector<Point> track(2 * seeds.size() - 1);
    double seen = 0.0;

    for (size_t k = 0; k < track.size(); k++) {
        const double t = a + (b - a) * k / (track.size() - 1);
        const Point p = k == 0 ? first : k + 1 == track.size() ? last : world.point_at(t);
        const auto gap = [&](const Point& uv) { return Point(srf.point(uv[0], uv[1])).distance(p); };
        track[k] = k == 0 ? near(p, seeds, t) : pre(p, track[k - 1]);
        double g = gap(track[k]);

        if (k == 0 || g > tol + 2.0 * seen) {
            const Point other = near(p, seeds, t);
            const double h = gap(other);

            if (k > 0 && h < g) {
                track[k] = other;
                g = h;
            }
        }

        seen = g;
    }

    const auto value = [&](double t) { return near(t == a ? first : t == b ? last : world.point_at(t), track, t); };
    std::vector<double> breaks = world.get_span_vector();
    const std::vector<double> lines[2] = {srf.get_span_vector(0), srf.get_span_vector(1)};

    const int samples = std::clamp(256 / (int)breaks.size(), 2, 16);

    for (size_t i = 1, n = breaks.size(); i < n; i++) {
        double t0 = breaks[i - 1];
        Point p0 = value(t0);

        for (int k = 1; k <= samples; k++) {
            const double t1 = breaks[i - 1] + (breaks[i] - breaks[i - 1]) * k / samples;
            const Point p1 = value(t1);

            for (int dir = 0; dir < 2; dir++)
                for (double knot : lines[dir]) {
                    double lo = t0;
                    double hi = t1;
                    double flo = p0[dir] - knot;
                    double fhi = p1[dir] - knot;
                    double m = lo;
                    int side = 0;

                    if (!(flo * fhi < 0))
                        continue;

                    // Regula falsi, Illinois variant: the end kept twice has its value halved.
                    for (int it = 0; it < 60 && hi - lo > (b - a) * 1e-14; it++) {
                        m = lo + (hi - lo) * flo / (flo - fhi);
                        const double fm = value(m)[dir] - knot;

                        if (std::abs(fm) <= 1e-14 * (1.0 + std::abs(knot)))
                            break;

                        if (fm * flo > 0) {
                            lo = m;
                            flo = fm;
                            fhi *= side == -1 ? 0.5 : 1.0;
                            side = -1;
                        } else {
                            hi = m;
                            fhi = fm;
                            flo *= side == 1 ? 0.5 : 1.0;
                            side = 1;
                        }
                    }

                    breaks.push_back(m);
                }

            t0 = t1;
            p0 = p1;
        }
    }

    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end(), [&](double x, double y) { return y - x <= (b - a) * 1e-5; }), breaks.end());
    breaks.back() = b;

    std::vector<Point> at;
    std::vector<Point> mid;
    error = 0.0;

    for (size_t i = 0; i < breaks.size(); i++) {
        at.push_back(value(breaks[i]));

        if (i > 0) {
            const double t = 0.5 * (breaks[i - 1] + breaks[i]);
            mid.push_back(value(t));
            error = std::max(error, Point(srf.point(mid.back()[0], mid.back()[1])).distance(world.point_at(t)));
        }
    }

    const double off = error;

    if (off >= budget)
        return NurbsCurve();

    // At a pole the collapsed coordinate of an end follows the curve leaving it.
    for (Point* end : {&at.front(), &at.back()}) {
        const std::vector<Vector> d = srf.evaluate((*end)[0], (*end)[1], 1);
        const Point near = value(end == &at.front() ? a + (b - a) * 1e-7 : b - (b - a) * 1e-7);

        for (int dir = 0; dir < 2 && d.size() == 3; dir++)
            if (d[2 - dir].magnitude() <= 1e-9 * d[1 + dir].magnitude())
                (*end)[dir] = near[dir];
    }

    const auto deviation = [&](const NurbsCurve& pc, double t0, double t1) {
        double e = 0.0;

        for (int k = 1; k < 8; k++) {
            const Point q = pc.point_at(t0 + (t1 - t0) * k / 8.0);
            e = std::max(e, Point(srf.point(q[0], q[1])).distance(world.point_at(t0 + (t1 - t0) * k / 8.0)));
        }

        return e;
    };

    for (int fixed = 0; fixed < 2; fixed++) {
        const int moving = 1 - fixed;
        const double c = at[0][fixed];
        bool iso = true;

        for (const std::vector<Point>* points : {&at, &mid})
            for (const Point& p : *points)
                iso = iso && std::abs(p[fixed] - c) <= 1e-13 * (1.0 + std::abs(c));

        NurbsCurve pc(3, true, 2, (int)breaks.size());
        pc.m_nurbsknot = breaks;
        double w = 1.0;

        for (size_t i = 0; iso && i < breaks.size(); i++) {
            if (i > 0)
                w *= (mid[i - 1][moving] - at[i - 1][moving]) / (at[i][moving] - mid[i - 1][moving]);

            Point q = at[i];
            q[fixed] = c;
            iso = w > 0.0 && std::isfinite(w) && pc.set_cv_4d((int)i, q[0] * w, q[1] * w, 0.0, w);
        }

        error = 0.0;

        for (size_t i = 1; iso && i < breaks.size(); i++)
            error = std::max(error, deviation(pc, breaks[i - 1], breaks[i]));

        if (iso && pc.is_valid() && error <= off + tol / 64)
            return pc;
    }

    // One-sided jets inside the span toward t + h: the surface jets solved where the world curve is on the surface, else fourth-order differences of the preimage.
    const auto jets = [&](double t, double h, Vector& d1, Vector& d2) {
        const double inside = t + h * 1e-9;
        const Point p = value(inside);

        if (Point(srf.point(p[0], p[1])).distance(world.point_at(inside)) <= tol * 1e-3 && uv_jets(world, srf, inside, p, d1, d2))
            return true;

        std::array<Point, 5> f;

        for (int k = 0; k < 5; k++)
            f[k] = value(t + h * 0.01 * k);

        const double step = h * 0.01;
        d1 = ((f[1] - f[0]) * 48.0 - (f[2] - f[0]) * 36.0 + (f[3] - f[0]) * 16.0 - (f[4] - f[0]) * 3.0) * (1.0 / (12.0 * step));
        d2 = ((f[1] - f[0]) * -104.0 + (f[2] - f[0]) * 114.0 - (f[3] - f[0]) * 56.0 + (f[4] - f[0]) * 11.0) * (1.0 / (12.0 * step * step));

        return std::isfinite(d1[0] + d1[1] + d2[0] + d2[1]);
    };
    std::vector<nurbs_hermite::QuinticSpan> spans;
    double far = 0.0;
    error = 0.0;
    std::function<bool(double, double, const Point&, const Point&, int, double)> fill = [&](double t0, double t1, const Point& p0, const Point& p1, int depth, double above) {
        nurbs_hermite::QuinticSpan s{t0, t1, p0, p1, Vector(), Vector(), Vector(), Vector()};

        if (!jets(t0, t1 - t0, s.start_derivative, s.start_second) || !jets(t1, t0 - t1, s.end_derivative, s.end_second))
            return false;

        double e = 0.0;
        double gap = 0.0;

        for (int k = 1; k < 8; k++) {
            const Point q = quintic_at(s, k / 8.0);
            const Point p = value(t0 + (t1 - t0) * k / 8.0);
            const Point on = Point(srf.point(p[0], p[1]));
            e = std::max(e, Point(srf.point(q[0], q[1])).distance(on));
            gap = std::max(gap, on.distance(world.point_at(t0 + (t1 - t0) * k / 8.0)));
        }

        // Halving stops where it no longer halves the deviation: the preimage is not smooth there (a loose curve near a fold or a stall).
        if (e > std::max(tol / 64, gap / 4) && depth < 30 && spans.size() < 4000 && (depth < 4 || e < 0.5 * above)) {
            const double tm = 0.5 * (t0 + t1);
            const Point pm = value(tm);

            return fill(t0, tm, p0, pm, depth + 1, e) && fill(tm, t1, pm, p1, depth + 1, e);
        }

        error = std::max(error, e + gap);
        far = std::max(far, gap);
        spans.push_back(s);

        return true;
    };

    bool filled = true;

    for (size_t i = 1; filled && i < breaks.size(); i++)
        filled = fill(breaks[i - 1], breaks[i], at[i - 1], at[i], 0, INFINITY);

    const NurbsCurve quintic = filled ? nurbs_hermite::quintic(spans) : NurbsCurve();

    if (quintic.is_valid() && error <= std::max(1e3 * tol, 2.0 * far))
        return quintic;

    // Where the preimage does not settle (a curve far off its surface, or stalling over a collapsed stretch of it) the polyline of the swept seeds in the world's parameter bounds the error instead.
    track.front() = at.front();
    track.back() = at.back();
    NurbsCurve line = NurbsCurve::create(false, 1, track);
    double line_error = 0.0;

    for (size_t k = 0; k < track.size(); k++)
        line.m_nurbsknot[k] = a + (b - a) * k / (track.size() - 1);

    for (size_t k = 1; k < track.size() && line.is_valid(); k++)
        line_error = std::max(line_error, deviation(line, line.m_nurbsknot[k - 1], line.m_nurbsknot[k]));

    if (quintic.is_valid() && error <= line_error)
        return quintic;

    error = line_error;

    return line;
}

/// Bounds of a surface's partial derivatives as BSplSLib::Resolution takes them: its degree times the largest step between control points
/// over the knot span that step covers, by the spread of its weights.
static std::array<double, 2> partial_bounds(const NurbsSurface& s) {

    std::array<double, 2> bound = {0.0, 0.0};
    double low = INFINITY, high = 0.0;

    for (int i = 0; i < s.cv_count(0); i++)
        for (int j = 0; j < s.cv_count(1); j++) {
            low = std::min(low, s.weight(i, j));
            high = std::max(high, s.weight(i, j));
        }

    for (int dir = 0; dir < 2; dir++)
        for (int i = 0; i + 1 < s.cv_count(dir); i++)
            for (int j = 0; j < s.cv_count(1 - dir); j++) {
                const double span = s.nurbsknot(dir, i + s.degree(dir)) - s.nurbsknot(dir, i);
                const Point p = dir == 0 ? s.get_cv(i, j) : s.get_cv(j, i);
                const Point q = dir == 0 ? s.get_cv(i + 1, j) : s.get_cv(j, i + 1);

                if (span > 0.0)
                    bound[dir] = std::max(bound[dir], s.degree(dir) * p.distance(q) / span * high / low);
            }

    return bound;
}

/// Closest uv of p on a surface by Newton from a guess, on the guess's branch of the periods tau_u, tau_v (0 when open).
static Point newton_uv(const NurbsSurface& srf, const Point& p, const Point& guess, double tau_u, double tau_v) {

    const std::pair<double, double> du = {srf.m_nurbsknot[0][srf.degree(0) - 1], srf.m_nurbsknot[0][srf.cv_count(0) - 1]};
    const std::pair<double, double> dv = {srf.m_nurbsknot[1][srf.degree(1) - 1], srf.m_nurbsknot[1][srf.cv_count(1) - 1]};
    const double su = tau_u > 0.0 ? tau_u * std::floor((guess[0] - du.first) / tau_u) : 0.0;
    const double sv = tau_v > 0.0 ? tau_v * std::floor((guess[1] - dv.first) / tau_v) : 0.0;
    double u = std::clamp(guess[0] - su, du.first, du.second);
    double v = std::clamp(guess[1] - sv, dv.first, dv.second);

    for (int it = 0; it < 50; it++) {
        std::array<xyz, 6> s;

        if (!srf.jet(u, v, 2, s))
            break;

        // Gauss-Newton for four steps, then Newton on the squared distance (its Hessian takes the second partials, so a point off the
        // surface converges quadratically where Gauss-Newton crawls), Gauss-Newton again where that Hessian is not positive.
        const xyz r = s[0] - xyz(p);
        const xyz& pu = s[3];
        const xyz& pv = s[1];
        double a = pu.dot(pu);
        double b = pu.dot(pv);
        double d = pv.dot(pv);
        const double na = a + s[5].dot(r), nb = b + s[4].dot(r), nd = d + s[2].dot(r);

        if (it >= 4 && na > 0.0 && nd > 0.0 && na * nd - nb * nb > 1e-12 * na * nd) {
            a = na;
            b = nb;
            d = nd;
        }

        const double det = a * d - b * b;

        if (!(a > 0.0 || d > 0.0))
            break;

        // Where one partial vanishes (a collapsed boundary) the step runs along the other.
        const bool singular = !(det > 1e-24 * a * d) || !(std::min(a, d) > 0.0);
        double nu = singular ? (a > 0.0 ? u - pu.dot(r) / a : u) : u - (d * pu.dot(r) - b * pv.dot(r)) / det;
        double nv = singular ? (a > 0.0 ? v : v - pv.dot(r) / d) : v - (a * pv.dot(r) - b * pu.dot(r)) / det;

        // A step leaving the domain stops on its boundary, the other coordinate then minimised along it.
        const bool out_u = nu < du.first || nu > du.second;

        if (out_u && d > 0.0)
            nv = v - pv.dot(r) / d;

        if ((nv < dv.first || nv > dv.second) && !out_u && a > 0.0)
            nu = u - pu.dot(r) / a;

        nu = std::clamp(nu, du.first, du.second);
        nv = std::clamp(nv, dv.first, dv.second);
        const bool done = std::abs(nu - u) <= 1e-12 * (du.second - du.first) && std::abs(nv - v) <= 1e-12 * (dv.second - dv.first);
        u = nu;
        v = nv;

        if (done)
            break;
    }

    // On the seam of a closed direction both ends are the point: the one next to the guess.
    u += su + (tau_u > 0.0 ? tau_u * std::round((guess[0] - u - su) / tau_u) : 0.0);
    v += sv + (tau_v > 0.0 ? tau_v * std::round((guess[1] - v - sv) / tau_v) : 0.0);

    return Point(u, v, 0.0);
}

/// Chart uv of p on an analytic window, on the turn next to the guess; at a pole or apex the guess's u.
static Point analytic_uv(const AnFace& an, const Window& w, const Point& p, const Point& guess) {

    double s = 0.0;
    double t = 0.0;
    bool radial = false;
    std::tie(s, t, radial) = an_st_of(an, p);
    Point q = chart_point(an, w, Point(s, t, 0.0));
    q[0] = radial ? q[0] + 4.0 * std::round((guess[0] - q[0]) / 4.0) : guess[0];

    if (an.kind == 5)
        q[1] += 4.0 * std::round((guess[1] - q[1]) / 4.0);

    return q;
}

/// Face ids of a CLOSED_SHELL in sets joined by shared edges, one solid each (as OCCT splits a shell of disjoint parts), or of an OPEN_SHELL in one set; none for anything else.
static std::vector<std::vector<int>> shell_parts(const StepReader& r, int shell_id) {

    const StepEntity* sent = r.get(shell_id);
    const StepSubEntity* shell = sent ? sent->find("CLOSED_SHELL") : nullptr;
    const std::vector<int> faces = shell ? list_refs(shell->params) : sent && sent->find("OPEN_SHELL") ? list_refs(sent->find("OPEN_SHELL")->params) : std::vector<int>();

    if (!shell || faces.empty())
        return faces.empty() ? std::vector<std::vector<int>>() : std::vector<std::vector<int>>{faces};

    std::vector<int> part(faces.size());
    std::unordered_map<int, int> edge_part;

    for (size_t i = 0; i < faces.size(); i++)
        part[i] = (int)i;

    const auto root = [&](int i) {
        while (part[i] != i)
            i = part[i] = part[part[i]];

        return i;
    };

    for (size_t i = 0; i < faces.size(); i++) {
        const StepEntity* face = r.get(faces[i]);
        const StepSubEntity* sub = face ? face->find("ADVANCED_FACE") : nullptr;

        if (face && !sub)
            sub = face->find("FACE_SURFACE");

        for (int bid : sub ? list_refs(sub->params) : std::vector<int>())
            if (const std::optional<Bound> b = bound_loop(r, bid))
                for (int oe : b->oe_refs) {
                    const int ec = oriented_edge(r, oe).first;
                    const auto it = edge_part.emplace(ec, (int)i).first;
                    part[root((int)i)] = root(it->second);
                }
    }

    std::map<int, std::vector<int>> sets;

    for (size_t i = 0; i < faces.size(); i++)
        sets[root((int)i)].push_back(faces[i]);

    std::vector<std::vector<int>> out;

    for (const auto& kv : sets)
        out.push_back(kv.second);

    return out;
}

/// Faces of a closed shell oriented consistently (ShapeFix_Shell): within each set of faces joined by edges used twice, the faces whose uses repeat a neighbour's direction are flipped, the larger part keeping the file's orientation.
static void orient_shell(const BRep& brep, std::vector<BRepRef>& faces) {

    std::vector<std::vector<std::pair<int, int>>> uses(brep.m_edges.size());

    for (int k = 0; k < (int)faces.size(); k++)
        for (const BRepRef& wr : brep.m_faces[faces[k].index].wires)
            for (const BRepRef& er : brep.wire_edges(wr))
                if (!brep.m_edges[er.index].degenerated)
                    uses[er.index].push_back({k, brep_compose(faces[k].orientation, er.orientation) == BRepOrientation::Forward ? 1 : -1});

    std::vector<std::vector<std::pair<int, int>>> adjacent(faces.size());

    for (const std::vector<std::pair<int, int>>& u : uses)
        if (u.size() == 2 && u[0].first != u[1].first) {
            adjacent[u[0].first].push_back({u[1].first, u[0].second == u[1].second});
            adjacent[u[1].first].push_back({u[0].first, u[0].second == u[1].second});
        }

    std::vector<int> flip(faces.size(), -1);

    for (int seed = 0; seed < (int)faces.size(); seed++) {
        if (flip[seed] >= 0)
            continue;

        std::vector<int> part{seed};
        flip[seed] = 0;

        for (size_t i = 0; i < part.size(); i++)
            for (const std::pair<int, int>& next : adjacent[part[i]])
                if (flip[next.first] < 0) {
                    flip[next.first] = flip[part[i]] ^ next.second;
                    part.push_back(next.first);
                }

        const size_t flipped = std::count_if(part.begin(), part.end(), [&](int k) { return flip[k] == 1; });

        for (int k : part)
            if (flip[k] == (2 * flipped > part.size() ? 0 : 1))
                faces[k].orientation = brep_reverse(faces[k].orientation);
    }
}

/// BRep of one STEP shell, built face by face.
class BRepBuilder {
    StepReader& r; // Entity reader.
    double tolerance;
    BRep brep; // Brep under construction.
    std::unordered_map<int, int> vmap; // Brep vertex by VERTEX_POINT id.
    std::unordered_map<int, int> emap; // Brep edge by EDGE_CURVE id.
    std::vector<BRepRef> face_refs; // Face references in file order.
    std::map<int, std::tuple<int, int, double>> splits; // Edge split for a missing seam: its pieces and the normalized parameter of the cut.
    std::map<int, Surface> forms; // The file's analytic surface of a brep surface, over the window its face was read in.

public:

    /// Construct over an entity reader.
    BRepBuilder(StepReader& reader, double precision) : r(reader), tolerance(precision) {}

    /// The second use of an edge on the same surface is a seam: the forward use keeps curve_2d_index, the reversed one curve_2d_index_2.
    void attach_pcurve(int edge, int si, int c2, bool reversed_use) {

        for (BRepCurveOnSurface& pc : brep.m_edges[edge].pcurves) {
            if (pc.surface_index != si)
                continue;

            if (reversed_use) {
                pc.curve_2d_index_2 = c2;
            } else {
                pc.curve_2d_index_2 = pc.curve_2d_index;
                pc.curve_2d_index = c2;
            }

            return;
        }

        brep.add_pcurve(edge, si, c2);
    }

    /// Face from its surface and loops (outer first), oriented in the shell by reversed_face; when the closed outer loop runs clockwise on the surface (an inconsistent file) every loop is turned around (ShapeFix_Face::FixOrientation).
    void finish_face(int si, bool reversed_face, std::vector<std::vector<PendingEdge>> loops) {

        if (loops.empty() || loops.front().empty())
            throw std::runtime_error("face has no edge loop");

        double area = 0.0;
        double size = 0.0;

        for (const PendingEdge& pe : loops.front())
            for (int k = 0; k < 16; k++) {
                const double t0 = pe.c2d.domain_start();
                const double t1 = pe.c2d.domain_end();
                const Point a = pe.c2d.point_at(t0 + (t1 - t0) * k / 16.0);
                const Point b = pe.c2d.point_at(t0 + (t1 - t0) * (k + 1) / 16.0);
                area += a[0] * b[1] - b[0] * a[1];
                size = std::max(size, a.distance(loops.front().front().c2d.point_at_start()));
            }

        if (area < 0.0 && loops.front().back().c2d.point_at_end().distance(loops.front().front().c2d.point_at_start()) <= 1e-6 * size)
            for (std::vector<PendingEdge>& lp : loops) {
                std::reverse(lp.begin(), lp.end());

                for (PendingEdge& pe : lp) {
                    pe.reversed = !pe.reversed;

                    if (!pe.c2d.reverse())
                        throw std::runtime_error("pcurve could not be reversed");
                }
            }

        // A loop that comes back to one of its vertices at the same point of the surface is two loops, split there
        // (ShapeFix_Face::FixLoopWire); at a seam or across a pole the points differ.
        const auto start = [&](const PendingEdge& pe) { return pe.reversed ? brep.m_edges[pe.edge].end_vertex : brep.m_edges[pe.edge].start_vertex; };
        bool split = false;
        for (size_t k = 0; k < loops.size(); k++)
            for (size_t a = 0; a < loops[k].size(); a++)
                for (size_t b = a + 2; b < loops[k].size() && b - a < loops[k].size() - 1; b++) {
                    const Point p = loops[k][a].c2d.point_at_start(), q = loops[k][b].c2d.point_at_start();
                    if (start(loops[k][a]) != start(loops[k][b]) || p.distance(q) > 1e-9 * (1.0 + std::abs(p[0]) + std::abs(p[1])))
                        continue;
                    std::vector<PendingEdge> piece(loops[k].begin() + a, loops[k].begin() + b);
                    loops[k].erase(loops[k].begin() + a, loops[k].begin() + b);
                    loops.push_back(std::move(piece));
                    split = true;
                    b = a + 1;
                }

        // Loops running counterclockwise each bound a face of their own with the clockwise loops inside them (ShapeFix_Face::FixSplitFace).
        std::vector<std::vector<Point>> polygons;
        std::vector<double> areas;

        for (const std::vector<PendingEdge>& lp : loops) {
            polygons.emplace_back();
            areas.push_back(0.0);

            for (const PendingEdge& pe : lp)
                for (int k = 0; k < 16; k++)
                    polygons.back().push_back(pe.c2d.point_at(pe.c2d.domain_start() + (pe.c2d.domain_end() - pe.c2d.domain_start()) * k / 16.0));

            for (size_t k = 0; k < polygons.back().size(); k++) {
                const Point& a = polygons.back()[k];
                const Point& b = polygons.back()[(k + 1) % polygons.back().size()];
                areas.back() += a[0] * b[1] - b[0] * a[1];
            }
        }

        // The outer loop runs counterclockwise: a clockwise first loop a split left first gives way to the largest that does.
        const size_t largest = std::max_element(areas.begin(), areas.end()) - areas.begin();
        if (split && areas[0] < 0.0 && areas[largest] > 0.0) {
            std::swap(loops[0], loops[largest]);
            std::swap(polygons[0], polygons[largest]);
            std::swap(areas[0], areas[largest]);
        }

        const auto inside = [](const std::vector<Point>& polygon, const Point& p) {
            bool in = false;

            for (size_t k = 0, j = polygon.size() - 1; k < polygon.size(); j = k++)
                if ((polygon[k][1] > p[1]) != (polygon[j][1] > p[1]) &&
                    p[0] < polygon[j][0] + (polygon[k][0] - polygon[j][0]) * (p[1] - polygon[j][1]) / (polygon[k][1] - polygon[j][1]))
                    in = !in;

            return in;
        };
        std::vector<size_t> outers = {0};

        for (size_t k = 1; k < loops.size(); k++)
            if (areas[k] > 0.0 && !inside(polygons[0], polygons[k][0]))
                outers.push_back(k);

        // Each loop's face: its own when it bounds one, else the first that holds it, else the first.
        std::vector<size_t> owner(loops.size(), 0);

        for (size_t k = 0; k < loops.size(); k++)
            for (size_t o : outers)
                if (o == k || (outers.size() > 1 && std::find(outers.begin(), outers.end(), k) == outers.end() && inside(polygons[o], polygons[k][0]))) {
                    owner[k] = o;
                    break;
                }

        for (size_t o : outers) {
            std::vector<BRepRef> wires;

            for (size_t k = 0; k < loops.size(); k++) {
                if (owner[k] != o)
                    continue;

                std::vector<BRepRef> refs;

                for (const PendingEdge& pe : loops[k]) {
                    NurbsCurve c = pe.reversed && pe.along.is_valid() ? pe.along : pe.c2d;

                    if (!pe.reversed || pe.along.is_valid() || c.reverse())
                        attach_pcurve(pe.edge, si, brep.add_curve_2d(c), pe.reversed);

                    refs.push_back({pe.edge, pe.reversed ? BRepOrientation::Reversed : BRepOrientation::Forward});
                }

                if (!refs.empty())
                    wires.insert(k == o ? wires.begin() : wires.end(), {brep.add_wire(refs), BRepOrientation::Forward});
            }

            const int fi = brep.add_face(si, wires);
            face_refs.push_back({fi, reversed_face ? BRepOrientation::Reversed : BRepOrientation::Forward});
        }
    }

    /// Return the brep vertex of a VERTEX_POINT, creating it once.
    int get_vertex(int vp_id) {

        const auto it = vmap.find(vp_id);

        if (it != vmap.end())
            return it->second;

        const std::optional<Point> p = r.get_vertex_point(vp_id);

        if (!p)
            throw std::runtime_error("vertex #" + std::to_string(vp_id) + " is not a VERTEX_POINT");

        return vmap[vp_id] = brep.add_vertex(*p);
    }

    /// Cartesian trim selectors of a circle are physical bounds, not disposable wrappers.
    std::optional<Point> cartesian_trim(const StepParam& selector) {
        for (int id : all_refs(selector.list)) {
            const StepEntity* point = r.get(id);
            if (point && point->has("CARTESIAN_POINT")) return r.get_point(id);
        }
        return std::nullopt;
    }

    /// Partition is a proposal; preserve the actual trim angles and circular locus.
    int trimmed_circle_quarter_count(const Axis2& axis, double radius, const Point& start, const Point& end) const {
        const double first = angle_of(axis, start);
        double last = angle_of(axis, end);
        if (last == first) return 0;
        if (last < first) last += 2 * PI;
        const double span = last - first;
        const int quarters = static_cast<int>(std::floor(span / PI_2 + .5));
        if (quarters < 1 || quarters > 4 || !(span / quarters < PI)) return 0;
        const double endpoint_gap = 2 * radius * std::abs(std::sin((span - quarters * PI_2) * .5));
        return std::isfinite(endpoint_gap) && endpoint_gap <= tolerance ? quarters : 0;
    }

    NurbsCurve trimmed_circle(const StepSubEntity& trim, const Point& start, const Point& end,
                              bool edge_sense) {
        if (trim.params.size() < 6 || trim.params[4].tag != StepTag::Enum ||
            (trim.params[4].str != "T" && trim.params[4].str != "F") ||
            trim.params[5].tag != StepTag::Enum || trim.params[5].str != "CARTESIAN")
            return NurbsCurve();
        const int basis_id = first_ref(trim.params);
        const StepEntity* basis = r.get(basis_id);
        const StepSubEntity* circle = basis ? basis->find("CIRCLE") : nullptr;
        const auto first = cartesian_trim(trim.params[2]), last = cartesian_trim(trim.params[3]);
        if (!circle || !first || !last || first->distance(*last) == 0) return NurbsCurve();
        const Point expected_start = edge_sense ? *first : *last;
        const Point expected_end = edge_sense ? *last : *first;
        if (!(start.distance(expected_start) <= tolerance) || !(end.distance(expected_end) <= tolerance))
            return NurbsCurve();
        const int axis_id = first_ref(circle->params);
        const auto radii = nums(circle->params);
        const Axis2 axis = r.get_axis2(axis_id);
        if (!axis.ok || radii.empty() || !std::isfinite(radii[0]) || !(radii[0] > 0)) return NurbsCurve();
        for (const Point& endpoint : {start, end}) {
            const double angle = angle_of(axis, endpoint);
            const Point on_circle = axis.origin + (axis.ax * std::cos(angle) + axis.ay * std::sin(angle)) * radii[0];
            if (!(on_circle.distance(endpoint) <= tolerance)) return NurbsCurve();
        }
        const bool forward = (trim.params[4].str == "T") == edge_sense;
        const Point& arc_start = forward ? start : end;
        const Point& arc_end = forward ? end : start;
        const int quarters = trimmed_circle_quarter_count(axis, radii[0], arc_start, arc_end);
        NurbsCurve result = circle_nurbs(axis, radii[0], arc_start, arc_end, quarters, true);
        if (!forward && !result.reverse()) return NurbsCurve();
        return result;
    }

    /// The part of a basis B-spline between the edge vertices, the whole curve when its ends are already there.
    NurbsCurve trimmed_to(NurbsCurve c, const Point& vs, const Point& ve) const {

        const double t0 = c.closest_parameter(vs);
        const double t1 = c.closest_parameter(ve);
        const double span = c.domain_end() - c.domain_start();

        if ((std::abs(t0 - c.domain_start()) <= 1e-9 * span && std::abs(t1 - c.domain_end()) <= 1e-9 * span) ||
            (vs.distance(ve) <= tolerance && c.point_at_start().distance(c.point_at_end()) <= tolerance))
            return c;

        const double first = !(t0 < t1) && c.point_at_start().distance(vs) <= tolerance ? c.domain_start() : t0;
        const double last = !(first < t1) && c.point_at_end().distance(ve) <= tolerance ? c.domain_end() : t1;
        NurbsCurve head = c;
        NurbsCurve tail = c;

        if (!(first < last) && c.point_at_start().distance(c.point_at_end()) <= tolerance && head.trim(t0, c.domain_end()) &&
            tail.trim(c.domain_start(), t1))
            return joined(head, tail);

        if (!(first < last) || !c.trim(first, last))
            throw std::runtime_error("edge vertices do not bound a part of its B-spline curve");

        return c;
    }

    /// Exact 3D curve of an edge from vs to ve on its basis: line, polyline, circle, ellipse, hyperbola or B-spline, trimmed to the vertices; throws for any other curve.
    NurbsCurve edge_curve(int curve_id, const Point& vs, const Point& ve, bool edge_sense, int depth = 0) {

        const StepEntity* e = r.get(curve_id);

        if (!e || depth > MAX_DEPTH)
            throw std::runtime_error("edge curve #" + std::to_string(curve_id) + " is missing");

        if (const StepSubEntity* trim = e->find("TRIMMED_CURVE")) {
            const NurbsCurve arc = trimmed_circle(*trim, vs, ve, edge_sense);

            return arc.is_valid() ? arc : edge_curve(r.basis_curve_of(first_ref(trim->params)), vs, ve, edge_sense, depth + 1);
        }

        if (e->has("LINE"))
            return NurbsCurve::create(false, 1, {vs, ve});

        if (const StepSubEntity* polyline = e->find("POLYLINE")) {
            std::vector<Point> pts;

            for (int id : list_refs(polyline->params))
                pts.push_back(r.get_point(id));

            if (!edge_sense)
                std::reverse(pts.begin(), pts.end());

            return NurbsCurve::create(false, 1, pts);
        }

        if (curve_params(*e)) {
            NurbsCurve c = r.get_nurbs_curve(curve_id);

            if (!c.is_valid() || (!edge_sense && !c.reverse()))
                throw std::runtime_error("B-spline curve #" + std::to_string(curve_id) + " is invalid");

            return trimmed_to(c, vs, ve);
        }

        for (const char* type : {"CIRCLE", "ELLIPSE", "HYPERBOLA"}) {
            const StepSubEntity* conic = e->find(type);

            if (!conic)
                continue;

            const Axis2 a = r.get_axis2(first_ref(conic->params));
            const std::vector<double> rr = nums(conic->params);

            if (!a.ok || rr.empty() || !(rr[0] > 0.0) || (std::string(type) != "CIRCLE" && (rr.size() < 2 || !(rr[1] > 0.0))))
                throw std::runtime_error(std::string(type) + " #" + std::to_string(curve_id) + " is invalid");

            if (e->has("HYPERBOLA"))
                return hyperbola_nurbs(a, rr[0], rr[1], vs, ve);

            const double r2 = e->has("ELLIPSE") ? rr[1] : 0.0;
            NurbsCurve arc = edge_sense ? circle_nurbs(a, rr[0], vs, ve, 0, false, r2) : circle_nurbs(a, rr[0], ve, vs, 0, false, r2);

            if (!edge_sense && !arc.reverse())
                return NurbsCurve();

            return arc;
        }

        throw std::runtime_error("curve #" + std::to_string(curve_id) + " " + (e->parts.empty() ? "" : e->parts[0].type) + " is not supported");
    }

    /// BRep edge of an EDGE_CURVE with its exact curve, made once.
    int get_edge(int ec_id) {

        const auto it = emap.find(ec_id);

        if (it != emap.end())
            return it->second;

        const std::vector<int> refs = edge_refs(r, ec_id);

        if (refs.size() < 3)
            throw std::runtime_error("edge #" + std::to_string(ec_id) + " is not an EDGE_CURVE");

        const int sv = get_vertex(refs[0]);
        const int ev = get_vertex(refs[1]);
        const bool edge_sense = last_flag(r.get(ec_id)->find("EDGE_CURVE")->params, true);
        const NurbsCurve crv3d = edge_curve(r.basis_curve_of(refs[2]), brep.m_vertices[sv].point, brep.m_vertices[ev].point, edge_sense);

        if (!crv3d.is_valid())
            throw std::runtime_error("edge #" + std::to_string(ec_id) + " has no valid curve");

        return emap[ec_id] = brep.add_edge(brep.add_curve_3d(crv3d), sv, ev);
    }

    /// Pcurve of an edge use on srf, fitted from uv seeds in the edge direction (its ends at the vertices where the curve reaches them within tolerance), or the file's pcurve with deviation file_error where that is closer; returned in the traversal direction, the edge tolerance carrying the deviation.
    NurbsCurve use_pcurve(int edge, bool reversed, const NurbsSurface& srf, const Preimage& pre, const std::vector<Point>& seeds,
                          const NurbsCurve& file = NurbsCurve(), double file_error = INFINITY) {

        const BRepEdge& be = brep.m_edges[edge];

        if (be.curve_3d_index < 0 || brep.m_curves_3d[be.curve_3d_index].nurbs().get_span_vector().empty())
            throw std::runtime_error("edge " + std::to_string(edge) + " has no curve to fit a pcurve along (degenerated " + std::to_string(be.degenerated) + ")");

        const NurbsCurve& world = brep.m_curves_3d[be.curve_3d_index].nurbs();
        const Point& vs = brep.m_vertices[be.start_vertex].point;
        const Point& ve = brep.m_vertices[be.end_vertex].point;
        double error = 0.0;
        NurbsCurve pc = fit_pcurve(world, srf, pre, seeds, world.point_at_start().distance(vs) <= tolerance ? vs : world.point_at_start(),
                                   world.point_at_end().distance(ve) <= tolerance ? ve : world.point_at_end(), tolerance, error, file_error);

        if (!pc.is_valid() || file_error < error) {
            pc = file;
            error = file_error;
        }

        if (!pc.is_valid() || (reversed && !pc.reverse()))
            throw std::runtime_error("edge pcurve could not be fitted");

        // Control points kept in the domain of an open direction, so a pcurve along a boundary does not leave it by rounding.
        const std::pair<double, double> periods = surface_periods(srf);

        for (int i = 0; i < pc.cv_count(); i++) {
            auto [x, y, z, w] = pc.get_cv_4d(i);

            for (int d = 0; d < 2; d++)
                if ((d == 0 ? periods.first : periods.second) == 0.0)
                    (d == 0 ? x : y) = w * std::clamp((d == 0 ? x : y) / w, srf.domain(d).first, srf.domain(d).second);

            pc.set_cv_4d(i, x, y, z, w);
        }

        brep.m_edges[edge].tolerance = std::max(be.tolerance, error);

        return pc;
    }

    /// Canonical (s, t) samples of an edge use in curve order, unwrapped along the traversal after the uses already in lp.
    std::vector<Point> use_st(const AnFace& an, const Loop& lp, int edge, bool reversed) {

        std::vector<Point> ordered = sample_nurbs(brep.m_curves_3d[brep.m_edges[edge].curve_3d_index].nurbs(), 48);

        if (reversed)
            std::reverse(ordered.begin(), ordered.end());

        double ps = 0.0;
        double pt = 0.0;
        bool have_prev = false;
        std::tie(ps, pt, have_prev) = st_start(an, lp, ordered);
        std::vector<Point> st = st_unwrapped(an, ordered, ps, pt, have_prev);

        if (!std::get<2>(an_st_of(an, ordered[0])))
            st[0][0] = st[1][0];

        if (reversed)
            std::reverse(st.begin(), st.end());

        return st;
    }

    /// Seam at angle s of a turned surface from the feet of a to b on it (the iso line ShapeFix_Face::FixMissingSeam inserts): the generator of a cylinder or cone, the meridian arc of a sphere or torus.
    static NurbsCurve seam_curve(const AnFace& an, double s, const Point& a, const Point& b) {

        const Point pa = an_eval(an, s, std::get<1>(an_st_of(an, a)));
        const Point pb = an_eval(an, s, std::get<1>(an_st_of(an, b)));

        if (an.kind == 2 || an.kind == 3)
            return NurbsCurve::create(false, 1, {pa, pb});

        Axis2 meridian;
        meridian.ax = an.a.ax * std::cos(s) + an.a.ay * std::sin(s);
        meridian.origin = an.kind == 4 ? an.a.origin : an.a.origin + meridian.ax * an.radius;
        meridian.ay = an.a.az * (std::get<1>(an_st_of(an, pb)) >= std::get<1>(an_st_of(an, pa)) ? 1.0 : -1.0);
        meridian.az = meridian.ax.cross(meridian.ay);
        meridian.ok = true;

        return circle_nurbs(meridian, an.kind == 4 ? an.radius : an.r2, pa, pb, 0, true);
    }

    /// Split the edge of a loop that crosses angle s (about the axis of an) there, for this loop and, through splits, for every other use.
    void split_at_angle(Loop& lp, const AnFace& an, double s) {

        for (size_t k = 0; k < lp.edges.size(); k++) {
            const LoopEdge le = lp.edges[k];
            const NurbsCurve world = brep.m_curves_3d[brep.m_edges[le.edge_idx].curve_3d_index].nurbs();
            const auto off = [&](double t) { return std::remainder(std::get<0>(an_st_of(an, world.point_at(t))) - s, 2.0 * PI); };
            const int n = (int)le.uv.size() - 1;

            for (int i = 0; i < n; i++) {
                double lo = world.domain_start() + (world.domain_end() - world.domain_start()) * i / n;
                double hi = world.domain_start() + (world.domain_end() - world.domain_start()) * (i + 1) / n;

                // A vertex on the angle, off it by rounding, is no crossing.
                if (!(off(lo) * off(hi) < 0.0) || std::abs(off(lo) - off(hi)) > PI || std::min(std::abs(off(lo)), std::abs(off(hi))) <= 1e-12)
                    continue;

                for (int it = 0; it < 80 && std::nextafter(lo, hi) < hi; it++)
                    (off(0.5 * (lo + hi)) * off(lo) > 0.0 ? lo : hi) = 0.5 * (lo + hi);

                const double t = 0.5 * (lo + hi);
                NurbsCurve head = world;
                NurbsCurve tail = world;

                if (!(t > world.domain_start()) || !(t < world.domain_end()) || !head.trim(world.domain_start(), t) || !tail.trim(t, world.domain_end()))
                    return;

                const BRepEdge e = brep.m_edges[le.edge_idx];
                const int p = brep.add_vertex(world.point_at(t));
                const int e1 = brep.add_edge(brep.add_curve_3d(head), e.start_vertex, p);
                const int e2 = brep.add_edge(brep.add_curve_3d(tail), p, e.end_vertex);
                splits[le.edge_idx] = {e1, e2, (t - world.domain_start()) / (world.domain_end() - world.domain_start())};
                LoopEdge a = le;
                LoopEdge b = le;
                a.edge_idx = le.reversed ? e2 : e1;
                b.edge_idx = le.reversed ? e1 : e2;
                lp.edges[k] = a;
                lp.edges.insert(lp.edges.begin() + k + 1, b);

                return;
            }
        }
    }

    /// The edge, or its two pieces in the traversal order of the use once it has been split.
    std::vector<int> pieces(int edge, bool reversed) const {

        const auto it = splits.find(edge);

        if (it == splits.end())
            return {edge};

        return reversed ? std::vector<int>{std::get<1>(it->second), std::get<0>(it->second)} : std::vector<int>{std::get<0>(it->second), std::get<1>(it->second)};
    }

    /// Every other use of a split edge replaced by its two pieces, its pcurves cut at the same normalized parameter.
    void apply_splits() {

        for (BRepWire& w : brep.m_wires)
            for (size_t k = 0; k < w.edges.size(); k++) {
                const auto it = splits.find(w.edges[k].index);

                if (it == splits.end())
                    continue;

                const bool reversed = w.edges[k].orientation == BRepOrientation::Reversed;
                const int first = reversed ? std::get<1>(it->second) : std::get<0>(it->second);
                const int second = reversed ? std::get<0>(it->second) : std::get<1>(it->second);
                w.edges[k].index = first;
                w.edges.insert(w.edges.begin() + k + 1, {second, w.edges[k].orientation});
            }

        for (const auto& [edge, pieces] : splits)
            for (const BRepCurveOnSurface& pc : std::vector<BRepCurveOnSurface>(brep.m_edges[edge].pcurves)) {
                int images[2][2] = {{-1, -1}, {-1, -1}};

                for (int side = 0; side < 2; side++) {
                    const int c = side == 0 ? pc.curve_2d_index : pc.curve_2d_index_2;

                    if (c < 0)
                        continue;

                    const NurbsCurve whole = brep.m_curves_2d[c].nurbs();
                    const double t = whole.domain_start() + (whole.domain_end() - whole.domain_start()) * std::get<2>(pieces);
                    NurbsCurve head = whole;
                    NurbsCurve tail = whole;

                    if (head.trim(whole.domain_start(), t) && tail.trim(t, whole.domain_end())) {
                        images[0][side] = brep.add_curve_2d(head);
                        images[1][side] = brep.add_curve_2d(tail);
                    }
                }

                for (int piece = 0; piece < 2; piece++)
                    if (images[piece][0] >= 0)
                        brep.add_pcurve(piece == 0 ? std::get<0>(pieces) : std::get<1>(pieces), pc.surface_index, images[piece][0], images[piece][1]);
            }
    }

    /// Loops of a face on a turned surface that close around the axis with no seam (as some exporters write them; other loops stay holes) joined through a seam (generator or meridian) used both ways, as ShapeFix_Face::FixMissingSeam does:
    /// two such loops at a vertex both have, else at the vertex whose angle lies nearest 0 (the other loop split there), the nearest 0 first; one such loop on a sphere to the pole it turns around, a degenerated edge there; unchanged otherwise.
    void join_seamless(std::vector<Loop>& loops, const AnFace& an) {

        const auto start = [&](const LoopEdge& le) { return le.reversed ? brep.m_edges[le.edge_idx].end_vertex : brep.m_edges[le.edge_idx].start_vertex; };
        const auto angle = [&](int v) { return std::remainder(std::get<0>(an_st_of(an, brep.m_vertices[v].point)), 2.0 * PI); };
        const auto foot = [&](const LoopEdge& le) {
            const NurbsCurve& w = brep.m_curves_3d[brep.m_edges[le.edge_idx].curve_3d_index].nurbs();
            return le.reversed ? w.point_at_end() : w.point_at_start();
        };
        std::vector<size_t> around;
        double turn = 0.0;

        // A loop of loose edges turns by a whole turn up to its gaps, a closed one by none.

        for (size_t k = 0; k < loops.size(); k++) {
            const Loop& lp = loops[k];
            const double t = (lp.edges.back().reversed ? lp.edges.back().uv.front() : lp.edges.back().uv.back())[0] -
                (lp.edges.front().reversed ? lp.edges.front().uv.back() : lp.edges.front().uv.front())[0];

            // A loop that runs along one edge there and back has its seam already.
            const bool seamed = std::any_of(lp.edges.begin(), lp.edges.end(), [&](const LoopEdge& le) {
                return std::count_if(lp.edges.begin(), lp.edges.end(), [&](const LoopEdge& other) { return other.edge_idx == le.edge_idx; }) > 1;
            });

            if (std::abs(std::abs(t) - 2.0 * PI) <= 1e-2 && !seamed) {
                around.push_back(k);
                turn = t;
            }
        }

        if (around.size() != 2 && !(around.size() == 1 && an.kind == 4))
            return;

        Loop a = loops[around[0]];
        Loop b = around.size() == 2 ? loops[around[1]] : Loop();
        const auto at = [&](const Loop& lp, double s) {
            return std::any_of(lp.edges.begin(), lp.edges.end(), [&](const LoopEdge& le) { return std::abs(std::remainder(angle(start(le)) - s, 2.0 * PI)) * an.radius <= tolerance; });
        };
        double best = INFINITY;
        bool shared = false;
        bool in_b = false;

        // A vertex both loops have comes first, then the nearest to 0.
        for (const Loop* lp : {&a, &b})
            for (const LoopEdge& le : lp->edges) {
                const double s = angle(start(le));
                const bool both = at(lp == &a ? b : a, s);

                if (both > shared || (both == shared && std::abs(s) < std::abs(best))) {
                    best = s;
                    shared = both;
                    in_b = lp == &b;
                }
            }

        if (!shared && !b.edges.empty())
            split_at_angle(in_b ? a : b, an, best);

        const auto on_seam = [&](const LoopEdge& le) { return std::abs(std::remainder(angle(start(le)) - best, 2.0 * PI)) * an.radius <= tolerance; };
        size_t i = std::find_if(a.edges.begin(), a.edges.end(), on_seam) - a.edges.begin();
        // The material lies left of a loop on the surface, so the cap of a loop turning up in s holds the north pole.
        const Point pole = an_eval(an, best, turn > 0.0 ? PI_2 : -PI_2);
        size_t j = std::find_if(b.edges.begin(), b.edges.end(), on_seam) - b.edges.begin();

        if (i == a.edges.size() || (!b.edges.empty() && j == b.edges.size()))
            return;

        // Of several vertices on the seam line the two nearest: the seam never runs over an edge that lies on it.
        for (size_t ia = 0; ia < a.edges.size(); ia++)
            for (size_t jb = 0; jb < b.edges.size(); jb++)
                if (on_seam(a.edges[ia]) && on_seam(b.edges[jb]) && foot(a.edges[ia]).distance(foot(b.edges[jb])) < foot(a.edges[i]).distance(foot(b.edges[j]))) {
                    i = ia;
                    j = jb;
                }

        // The other loops are split where they cross the seam, as ShapeFix_ComposeShell cuts every wire at it; a hole met there twice
        // between the joined loops is cut in two halves the seam runs to and on from, its stretch inside the hole dropped.
        const auto height = [&](const Point& p) { return std::get<1>(an_st_of(an, p)); };
        const Point pa = foot(a.edges[i]);
        const Point pb = b.edges.empty() ? pole : foot(b.edges[j]);
        std::vector<LoopEdge> halves[2];
        std::vector<Point> cuts = {pa, pb};
        std::vector<int> ends = {start(a.edges[i]), b.edges.empty() ? brep.add_vertex(pole) : start(b.edges[j])};

        for (size_t k = 0; k < loops.size(); k++) {
            if (std::find(around.begin(), around.end(), k) != around.end())
                continue;

            Loop& lp = loops[k];

            for (size_t count = 0; count != lp.edges.size();) {
                count = lp.edges.size();
                split_at_angle(lp, an, best);
            }

            std::vector<size_t> at_seam;

            for (size_t e = 0; e < lp.edges.size(); e++)
                if (on_seam(lp.edges[e]))
                    at_seam.push_back(e);

            if (b.edges.empty() || cuts.size() > 2 || at_seam.size() != 2)
                continue;

            const Point p0 = brep.m_vertices[start(lp.edges[at_seam[0]])].point;
            const Point p1 = brep.m_vertices[start(lp.edges[at_seam[1]])].point;

            if (!((height(p0) - height(pa)) * (height(p0) - height(pb)) < 0.0 && (height(p1) - height(pa)) * (height(p1) - height(pb)) < 0.0))
                continue;

            const bool flip = std::abs(height(p1) - height(pa)) < std::abs(height(p0) - height(pa));
            const size_t near = at_seam[flip], far = at_seam[!flip];
            std::rotate(lp.edges.begin(), lp.edges.begin() + near, lp.edges.end());
            const size_t split = (far + lp.edges.size() - near) % lp.edges.size();
            halves[0].assign(lp.edges.begin(), lp.edges.begin() + split);
            halves[1].assign(lp.edges.begin() + split, lp.edges.end());
            cuts.insert(cuts.begin() + 1, {flip ? p1 : p0, flip ? p0 : p1});
            ends.insert(ends.begin() + 1, {start(lp.edges[0]), start(lp.edges[split])});
            lp.edges.clear();
        }

        std::vector<LoopEdge> seams;

        for (size_t k = 0; k < cuts.size(); k += 2) {
            LoopEdge seam;
            seam.edge_idx = brep.add_edge(brep.add_curve_3d(seam_curve(an, best, cuts[k], cuts[k + 1])), ends[k], ends[k + 1]);
            seams.push_back(seam);
        }

        std::vector<LoopEdge> uses(a.edges.begin() + i, a.edges.end());
        uses.insert(uses.end(), a.edges.begin(), a.edges.begin() + i);

        for (size_t k = 0; k < seams.size(); k++) {
            uses.push_back(seams[k]);

            if (k == 0)
                uses.insert(uses.end(), halves[0].begin(), halves[0].end());
        }

        uses.insert(uses.end(), b.edges.begin() + j, b.edges.end());
        uses.insert(uses.end(), b.edges.begin(), b.edges.begin() + j);

        for (size_t k = seams.size(); k-- > 0;) {
            seams[k].reversed = true;

            // A cap starts down the seam from the pole, so its two uses lie a turn apart and the degenerated edge closes them.
            if (b.edges.empty())
                uses.insert(uses.begin(), seams[k]);
            else
                uses.push_back(seams[k]);

            if (k == 1)
                uses.insert(uses.end(), halves[1].begin(), halves[1].end());
        }

        Loop joined;
        joined.is_outer = joined.projected = true;

        for (LoopEdge le : uses) {
            le.uv = use_st(an, joined, le.edge_idx, le.reversed);
            joined.edges.push_back(le);
        }

        if (!b.edges.empty())
            loops[around[1]].edges.clear();

        loops[around[0]] = joined;

        for (Loop& lp : loops)
            lp.is_outer = &lp == &loops[around[0]];

        loops.erase(std::remove_if(loops.begin(), loops.end(), [](const Loop& lp) { return lp.edges.empty(); }), loops.end());
    }

    /// Loops of an analytic face, each edge use with canonical (s, t) samples of its curve unwrapped along the traversal; inner loops shifted by whole turns onto the outer one.
    std::vector<Loop> analytic_loops(const std::vector<int>& bound_refs, bool same_sense, const AnFace& an) {

        std::vector<Loop> loops;

        for (int bid : bound_refs) {
            const std::optional<Bound> b = bound_loop(r, bid);

            if (!b)
                throw std::runtime_error("bound #" + std::to_string(bid) + " is not an edge loop");

            Loop lp;
            lp.is_outer = b->is_outer;
            lp.projected = true;

            for (const std::pair<int, bool>& use : bound_uses(r, *b, same_sense))
                for (int edge : pieces(get_edge(use.first), use.second)) {
                    LoopEdge le;
                    le.edge_idx = edge;
                    le.reversed = use.second;
                    le.uv = use_st(an, lp, le.edge_idx, le.reversed);
                    lp.edges.push_back(le);
                }

            align_at_poles(lp, an, lp.is_outer || bound_refs.size() == 1);
            loops.push_back(lp);
        }

        join_seamless(loops, an);
        pick_outer_loop(loops);
        split_seams(loops, 2.0 * PI, an.kind == 5 ? 2.0 * PI : 0.0);
        center_inner_loops(loops, 2.0 * PI);

        return loops;
    }

    /// Fitted pcurves of one loop in the chart, plus a degenerated edge across each pole or apex gap between consecutive edges.
    std::vector<PendingEdge> analytic_pending(const Loop& lp, const AnFace& an, const Window& w, const NurbsSurface& srf) {

        const std::vector<std::vector<Point>> chains = analytic_chains(lp, an, w);
        const Preimage pre = [&](const Point& p, const Point& guess) { return analytic_uv(an, w, p, guess); };
        std::vector<PendingEdge> pl;

        for (size_t k = 0; k < lp.edges.size(); k++) {
            std::vector<Point> seeds = chains[k];

            if (lp.edges[k].reversed)
                std::reverse(seeds.begin(), seeds.end());

            pl.push_back({lp.edges[k].edge_idx, lp.edges[k].reversed, use_pcurve(lp.edges[k].edge_idx, lp.edges[k].reversed, srf, pre, seeds)});
        }

        std::vector<PendingEdge> out;

        for (size_t k = 0; k < pl.size(); k++) {
            out.push_back(pl[k]);

            const PendingEdge& next = pl[(k + 1) % pl.size()];
            const Point a2 = pl[k].c2d.point_at_end();
            const Point b2 = next.c2d.point_at_start();
            const BRepEdge& be = brep.m_edges[pl[k].edge];
            const int vertex = pl[k].reversed ? be.start_vertex : be.end_vertex;

            if (std::abs(a2[0] - b2[0]) + std::abs(a2[1] - b2[1]) <= 1e-12 * (1.0 + std::abs(a2[0]) + std::abs(a2[1])) ||
                chart_eval(an, w, a2).distance(chart_eval(an, w, b2)) > (an.radius + std::abs(an.r2) + 1.0) * 1e-9)
                continue;

            out.push_back({brep.add_edge(-1, vertex, vertex), false, polyline_nurbs({a2, b2}, 2)});
        }

        return out;
    }

    /// Face on a cylinder, cone, sphere or torus: the exact kernel window of its loops with pcurves fitted in it.
    void add_face_analytic(const std::vector<int>& bound_refs, bool same_sense, const AnFace& an) {

        std::vector<Loop> loops = analytic_loops(bound_refs, same_sense, an);
        const std::optional<Window> w = analytic_window(loops, an);

        if (!w)
            throw std::runtime_error("analytic face has no chart window");

        const NurbsSurface srf = build_analytic_nurbs(an, w->su0, w->nsu, w->t0, w->t1, w->sv0, w->nsv);

        if (!srf.is_valid())
            throw std::runtime_error("analytic surface is invalid");

        const int srf_idx = brep.add_surface(srf);
        forms[srf_idx] = form_of(an, w->su0, w->nsu, w->t0, w->t1, w->sv0, w->nsv);
        outer_first(loops);

        std::vector<std::vector<PendingEdge>> pending;

        for (const Loop& lp : loops)
            pending.push_back(analytic_pending(lp, an, *w, srf));

        finish_face(srf_idx, !same_sense, pending);
    }

    /// Point of a VERTEX_POINT, far away when missing.
    Point step_point_of(int vp_id) {
        return r.get_vertex_point(vp_id).value_or(Point(1e300, 1e300, 1e300));
    }

    /// The file vertex at q when one of the given VERTEX_POINTs sits there, else a new vertex.
    int topo_vertex_at(const std::vector<int>& vl_vertex_ids, const Point& q, double tol) {

        for (int vid : vl_vertex_ids)
            if (step_point_of(vid).distance(q) <= tol)
                return get_vertex(vid);

        return brep.add_vertex(q);
    }

    /// Kernel surface of a VERTEX_LOOP face: the whole sphere or torus, the B-spline itself, invalid otherwise.
    NurbsSurface vertex_loop_surface(int surface_ref) {

        const AnFace an = r.get_analytic_srf(surface_ref);

        if (an.kind == 4)
            return build_analytic_nurbs(an, 0, 4, 0, 0, -1, 2);

        if (an.kind == 5)
            return build_analytic_nurbs(an, 0, 4, 0, 0, 0, 4);

        if (an.kind == 0)
            return r.get_nurbs_surface(surface_ref);

        return NurbsSurface();
    }

    /// Wire of a sphere-like surface: a degenerated edge at each pole and the seam used both ways; empty when the seam is invalid.
    std::vector<PendingEdge> pole_wire(
        const NurbsSurface& srf,
        const std::vector<Point>& grid,
        const std::vector<int>& vl_vertex_ids,
        double tol
    ) {

        double u0 = 0.0;
        double u1 = 0.0;
        double v0 = 0.0;
        double v1 = 0.0;
        std::tie(u0, u1) = srf.domain(0);
        std::tie(v0, v1) = srf.domain(1);

        const int v_lo = topo_vertex_at(vl_vertex_ids, grid[0], tol);
        const int v_hi = topo_vertex_at(vl_vertex_ids, grid[NS - 1], tol);
        const NurbsCurve seam = srf.iso_curve(1, u0);

        if (!seam.is_valid())
            return {};

        const int ei_seam = brep.add_edge(brep.add_curve_3d(seam), v_lo, v_hi);
        const int ei_lo = brep.add_edge(-1, v_lo, v_lo);
        const int ei_hi = brep.add_edge(-1, v_hi, v_hi);

        return {
            {ei_lo, false, uv_line(u0, v0, u1, v0)},
            {ei_seam, false, uv_line(u1, v0, u1, v1)},
            {ei_hi, false, uv_line(u1, v1, u0, v1)},
            {ei_seam, true, uv_line(u0, v1, u0, v0)},
        };
    }

    /// Wire of a torus-like surface: the u seam and the v seam each used both ways; empty when a seam is invalid.
    std::vector<PendingEdge> seam_wire(
        const NurbsSurface& srf,
        const std::vector<Point>& grid,
        const std::vector<int>& vl_vertex_ids,
        double tol
    ) {

        double u0 = 0.0;
        double u1 = 0.0;
        double v0 = 0.0;
        double v1 = 0.0;
        std::tie(u0, u1) = srf.domain(0);
        std::tie(v0, v1) = srf.domain(1);

        const int vtx = topo_vertex_at(vl_vertex_ids, grid[0], tol);
        const NurbsCurve c_u = srf.iso_curve(1, u0);
        const NurbsCurve c_v = srf.iso_curve(0, v0);

        if (!c_u.is_valid() || !c_v.is_valid())
            return {};

        const int ei_u = brep.add_edge(brep.add_curve_3d(c_u), vtx, vtx);
        const int ei_v = brep.add_edge(brep.add_curve_3d(c_v), vtx, vtx);

        return {
            {ei_v, false, uv_line(u0, v0, u1, v0)},
            {ei_u, false, uv_line(u1, v0, u1, v1)},
            {ei_v, true, uv_line(u1, v1, u0, v1)},
            {ei_u, true, uv_line(u0, v1, u0, v0)},
        };
    }

    /// Face bounded only by VERTEX_LOOPs: the whole surface, with seam and pole edges read off the surface (sphere-like or torus-like).
    bool add_face_vertex_loop(const std::vector<int>& vl_vertex_ids, int surface_ref, bool same_sense) {

        const NurbsSurface srf = vertex_loop_surface(surface_ref);

        if (!srf.is_valid())
            return false;

        const std::vector<Point> grid = surface_grid(srf, NS);
        const double tol = grid_scale(grid) * 1e-7;

        if (!(tol > 0))
            return false;

        const bool closed_u = grid_closed(grid, NS, tol, true);
        const bool closed_v = grid_closed(grid, NS, tol, false);
        const bool degen_v0 = grid_degenerate(grid, NS, tol, 0);
        const bool degen_v1 = grid_degenerate(grid, NS, tol, NS - 1);

        if (!closed_u || !((degen_v0 && degen_v1) || closed_v))
            return false;

        const int si = brep.add_surface(srf);
        const AnFace whole = r.get_analytic_srf(surface_ref);

        if (whole.kind == 4 || whole.kind == 5)
            forms[si] = form_of(whole, 0, 4, 0.0, 0.0, whole.kind == 4 ? -1 : 0, whole.kind == 4 ? 2 : 4);

        const std::vector<PendingEdge> wire =
            degen_v0 && degen_v1 ? pole_wire(srf, grid, vl_vertex_ids, tol) : seam_wire(srf, grid, vl_vertex_ids, tol);

        if (wire.empty())
            return false;

        finish_face(si, !same_sense, {wire});

        return true;
    }

    /// VERTEX_POINT ids of the VERTEX_LOOP bounds; empty when any bound is an EDGE_LOOP.
    std::vector<int> vertex_loop_ids(const std::vector<int>& bound_refs) const {

        std::vector<int> ids;

        for (int bid : bound_refs) {
            const StepEntity* bent = r.get(bid);
            const StepSubEntity* bsub = bent ? bent->find("FACE_OUTER_BOUND") : nullptr;

            if (bent && !bsub)
                bsub = bent->find("FACE_BOUND");

            const StepEntity* lent = bsub ? r.get(first_ref(bsub->params)) : nullptr;

            if (!lent)
                continue;

            if (lent->has("EDGE_LOOP"))
                return {};

            const StepSubEntity* vl = lent->find("VERTEX_LOOP");
            const int ref = vl ? first_ref(vl->params) : -1;

            if (ref >= 0)
                ids.push_back(ref);
        }

        return ids;
    }

    /// The forward one of two seam aliases: for +v the boundary of larger u, for +u that of smaller v.
    static int forward_seam(const std::vector<NurbsCurve>& curves) {
        const Point a = curves[0].point_at_start(), b = curves[0].point_at_end();
        const Point m0 = curves[0].point_at(curves[0].domain_middle()), m1 = curves[1].point_at(curves[1].domain_middle());
        const int varying = std::abs(b[1] - a[1]) > std::abs(b[0] - a[0]) ? 1 : 0;
        const bool maximum = (varying == 1) == (b[varying] > a[varying]);
        return (m0[1 - varying] > m1[1 - varying]) == maximum ? 0 : 1;
    }

    /// The file's pcurve of an edge on a B-spline surface: one certified against the world curve at the tolerance (error 0), else the one of least deviation (error); the forward or reversed seam alias by the use.
    std::optional<NurbsCurve> source_pcurve(int edge, int geometry_id, int surface_id, const NurbsSurface& support, bool forward, double& error) {
        const NurbsCurve& world = brep.m_curves_3d[brep.m_edges[edge].curve_3d_index].nurbs();
        const StepEntity* source = r.get(surface_id);
        if (!support.is_valid() || !source || !surface_params(*source)) return std::nullopt;
        const auto deviation = [&](const NurbsCurve& pc) {
            double d = 0.0;
            for (int k = 0; k <= 16; ++k) {
                const Point q = pc.point_at(pc.domain_start() + (pc.domain_end() - pc.domain_start()) * k / 16.0);
                d = std::max(d, Point(support.point(q[0], q[1])).distance(world.point_at(world.domain_start() + (world.domain_end() - world.domain_start()) * k / 16.0)));
            }
            return d;
        };
        std::vector<NurbsCurve> accepted, loose;
        const std::pair<double, double> periods = surface_periods(support);
        for (NurbsCurve pc : r.original_pcurves(geometry_id, surface_id, world)) {
            const Point middle = pc.point_at(pc.domain_middle());
            double shift[2] = {0.0, 0.0};
            for (int d = 0; d < 2; ++d) {
                const double period = d == 0 ? periods.first : periods.second;
                const auto domain = support.domain(d);
                if (period > 0.0 && (middle[d] < domain.first - 1e-9 * period || middle[d] > domain.second + 1e-9 * period))
                    shift[d] = -period * std::floor((middle[d] - domain.first) / period);
            }
            for (int i = 0; i < pc.cv_count() && (shift[0] != 0.0 || shift[1] != 0.0); ++i) {
                const auto [x, y, z, w] = pc.get_cv_4d(i);
                pc.set_cv_4d(i, x + shift[0] * w, y + shift[1] * w, z, w);
            }
            NurbsCurve flipped = pc;
            if (!flipped.reverse()) continue;
            if (deviation(flipped) < deviation(pc)) std::swap(pc, flipped);
            const bool duplicate = std::any_of(accepted.begin(), accepted.end(), [&](const NurbsCurve& item) { return item.jsondump() == pc.jsondump(); });
            if (duplicate) continue;
            if (curve_surface_intersection::same_parameter(world, pc, support, tolerance) == curve_surface_intersection::ParameterMatch::Matched) accepted.push_back(pc);
            else loose.push_back(pc);
        }
        std::vector<NurbsCurve>& pick = accepted.empty() ? loose : accepted;
        if (pick.empty()) return std::nullopt;
        // A seam the file gives one boundary line for twice has its second alias a period across.
        for (int d = 0; d < 2 && pick.size() == 2; ++d) {
            const double period = d == 0 ? periods.first : periods.second;
            const auto along = [&](const NurbsCurve& pc, double bound) {
                for (double f : {0.0, 0.5, 1.0})
                    if (std::abs(pc.point_at(pc.domain_start() + f * (pc.domain_end() - pc.domain_start()))[d] - bound) > 1e-9 * period) return false;
                return true;
            };
            const auto domain = support.domain(d);
            const bool low = along(pick[0], domain.first) && along(pick[1], domain.first);
            if (!(period > 0.0) || !(low || (along(pick[0], domain.second) && along(pick[1], domain.second)))) continue;
            for (int i = 0; i < pick[1].cv_count(); ++i) {
                const auto [x, y, z, w] = pick[1].get_cv_4d(i);
                const double shift = (low ? period : -period) * w;
                pick[1].set_cv_4d(i, x + (d == 0 ? shift : 0.0), y + (d == 1 ? shift : 0.0), z, w);
            }
        }
        if (pick.size() > 1 && deviation(pick[1]) < deviation(pick[0])) std::swap(pick[0], pick[1]);
        const int selected = pick.size() == 2 && deviation(pick[1]) <= 2.0 * deviation(pick[0]) + tolerance ? forward_seam(pick) : -1;
        const NurbsCurve& pc = selected < 0 ? pick.front() : pick[forward ? selected : 1 - selected];
        error = accepted.empty() ? deviation(pc) : 0.0;
        return pc;
    }

    /// Loops of a face on a plane or B-spline surface: uv seeds of each edge use in curve order, exact pcurves under an affine projector or certified from the file.
    void projected_loops(
        const std::vector<int>& bound_refs,
        const Proj& proj,
        const NurbsSurface& proj_srf,
        int surface_ref,
        bool same_sense,
        std::vector<Loop>& loops
    ) {

        const Proj exact = proj.kind == 1 ? proj : bilinear_projector(proj_srf);

        for (int bid : bound_refs) {
            const std::optional<Bound> b = bound_loop(r, bid);

            if (!b)
                throw std::runtime_error("bound #" + std::to_string(bid) + " is not an edge loop");

            Loop lp;
            lp.is_outer = b->is_outer;

            for (const std::pair<int, bool>& use : bound_uses(r, *b, same_sense)) {
                LoopEdge le;
                le.edge_idx = get_edge(use.first);
                le.reversed = use.second;
                const NurbsCurve& world = brep.m_curves_3d[brep.m_edges[le.edge_idx].curve_3d_index].nurbs();
                const std::optional<NurbsCurve> pc = source_pcurve(le.edge_idx, edge_geom_id(r, use.first), surface_ref, proj_srf, !le.reversed, le.error);

                if (pc) {
                    le.pc2d = *pc;
                    le.exact = le.error == 0.0;
                    le.source_pcurve = true;
                    le.uv = sample_nurbs(le.pc2d, 48);
                    lp.edges.push_back(std::move(le));
                    continue;
                }

                le.uv = uv_of_samples(proj, proj_srf, sample_nurbs(world, 48));
                le.pc2d = exact_pcurve(exact, world);
                le.exact = le.pc2d.is_valid() && (proj.kind == 1 || curve_surface_intersection::same_parameter(world, le.pc2d, proj_srf, tolerance) ==
                    curve_surface_intersection::ParameterMatch::Matched);
                lp.edges.push_back(le);
            }

            loops.push_back(lp);
        }

        pick_outer_loop(loops);
    }

    /// Pending edges of a loop, with a degenerated edge across each uv gap at one vertex whose image stays within its reach.
    std::vector<PendingEdge> projected_pending(const Loop& loop, const NurbsSurface& support) {
        const auto original = pending_of(loop);
        std::vector<PendingEdge> result;
        for (size_t i = 0; i < original.size(); ++i) {
            const auto& current = original[i];
            const auto& next = original[(i + 1) % original.size()];
            result.push_back(current);
            if (!support.is_valid() || !current.c2d.is_valid() || !next.c2d.is_valid())
                continue;
            const auto& first_edge = brep.m_edges.at(current.edge);
            const auto& second_edge = brep.m_edges.at(next.edge);
            const int vertex = current.reversed ? first_edge.start_vertex : first_edge.end_vertex;
            const int next_vertex = next.reversed ? second_edge.end_vertex : second_edge.start_vertex;
            if (vertex != next_vertex)
                continue;
            const Point a = current.c2d.point_at_end(), b = next.c2d.point_at_start();
            if (a.distance(b) <= 1e-9 * (1.0 + std::abs(a[0]) + std::abs(a[1])))
                continue;
            // A gap whose image stays within the vertex's reach, the tolerance its edges carry or the distance of their ends from it, takes a
            // degenerated edge where that image spans under a tenth of the length the surface's partial bounds give the gap
            // (ShapeFix_Wire::FixLacking with ShapeAnalysis_Surface::IsDegenerated at ratio 10): a pole, or a vertex off its curves; an
            // overlap of two pcurves along the surface keeps its gap, the vertex's tolerance covering it.
            const Point& point = brep.m_vertices.at(vertex).point;
            const Point pa = Point(support.point(a[0], a[1])), pb = Point(support.point(b[0], b[1]));
            const Point pm = Point(support.point(0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])));
            const double allowed = std::max({tolerance, first_edge.tolerance, second_edge.tolerance, pa.distance(point), pb.distance(point)}) * (1.0 + 1e-9);
            bool collapsed = true;
            for (int k = 1; k < 8 && collapsed; ++k)
                collapsed = Point(support.point(a[0] + (b[0] - a[0]) * k / 8, a[1] + (b[1] - a[1]) * k / 8)).distance(point) <= allowed;
            const std::array<double, 2> bound = partial_bounds(support);
            if (collapsed && std::hypot((b[0] - a[0]) * bound[0], (b[1] - a[1]) * bound[1]) > 10.0 * std::max({pa.distance(pb), pm.distance(pa), pm.distance(pb)}))
                result.push_back({brep.add_edge(-1, vertex, vertex), false, polyline_nurbs({a, b}, 2)});
        }
        return result;
    }

    /// Start angle of a revolution's window: the profile's when an edge is used twice (a seam), else the middle of the widest angular gap between samples of the face's edges.
    double revolution_start(const std::vector<int>& bound_refs, const Axis2& a) {

        std::vector<double> angles;
        std::vector<int> seen;

        for (int bid : bound_refs) {
            const std::optional<Bound> b = bound_loop(r, bid);

            for (int oe_id : b ? b->oe_refs : std::vector<int>()) {
                const int ec = oriented_edge(r, oe_id).first;

                if (std::find(seen.begin(), seen.end(), ec) != seen.end())
                    return 0.0;

                seen.push_back(ec);

                for (const Point& p : sample_nurbs(brep.m_curves_3d[brep.m_edges[get_edge(ec)].curve_3d_index].nurbs(), 16))
                    angles.push_back(angle_of(a, p));
            }
        }

        std::sort(angles.begin(), angles.end());

        if (angles.empty())
            return 0.0;

        double start = angles.back() + 0.5 * (angles.front() + 2.0 * PI - angles.back());
        double widest = angles.front() + 2.0 * PI - angles.back();

        for (size_t i = 1; i < angles.size(); i++)
            if (angles[i] - angles[i - 1] > widest) {
                widest = angles[i] - angles[i - 1];
                start = 0.5 * (angles[i] + angles[i - 1]);
            }

        return start;
    }

    /// Face on a plane, with a padded patch over its loops, on a B-spline surface or on a surface of revolution; pcurves exact, certified from the file or fitted.
    void add_face_projected(const std::vector<int>& bound_refs, int surface_ref, bool same_sense) {

        const Proj proj = r.get_projector(surface_ref);
        const auto rev = r.revolution(surface_ref);
        const NurbsSurface proj_srf = rev ? revolution_nurbs(rev->first, rev->second, revolution_start(bound_refs, rev->first))
            : proj.kind == 0 ? r.fill_surface(surface_ref, 0, 1, 0, 1) : NurbsSurface();

        if (proj.kind == 0 && !proj_srf.is_valid()) {
            const StepEntity* e = r.get(surface_ref);
            throw std::runtime_error("surface #" + std::to_string(surface_ref) + " " + (e && !e->parts.empty() ? e->parts[0].type : "") + " is not supported");
        }

        std::vector<Loop> loops;
        projected_loops(bound_refs, proj, proj_srf, surface_ref, same_sense, loops);

        double tau_u = 0.0;
        double tau_v = 0.0;
        std::tie(tau_u, tau_v) = surface_periods(proj_srf);
        chain_loops(loops, tau_u, tau_v);
        split_seams(loops, tau_u, tau_v);
        center_inner_loops(loops, tau_u);
        domain_loops(loops, proj_srf, tau_u, tau_v);

        NurbsSurface srf = proj_srf;

        if (!srf.is_valid()) {
            double umin = 0.0;
            double umax = 0.0;
            double vmin = 0.0;
            double vmax = 0.0;
            std::tie(umin, umax, vmin, vmax) = loops_bounds(loops);
            srf = r.fill_surface(surface_ref, umin, umax, vmin, vmax);
        }

        const int srf_idx = brep.add_surface(srf);

        if (proj.kind == 1 && !proj_srf.is_valid())
            forms[srf_idx] = Surface::plane(proj.a.origin, proj.a.az, proj.a.ax, {srf.domain(0).first, srf.domain(0).second, srf.domain(1).first, srf.domain(1).second});

        const Preimage pre = [&](const Point& p, const Point& guess) { return newton_uv(srf, p, guess, tau_u, tau_v); };
        outer_first(loops);

        std::vector<std::vector<PendingEdge>> pending;

        for (Loop& lp : loops) {
            for (LoopEdge& le : lp.edges)
                if (!le.exact)
                    le.pc2d = use_pcurve(le.edge_idx, false, srf, pre, le.uv, le.pc2d, le.error);

            pending.push_back(projected_pending(lp, srf));
        }

        finish_face(srf_idx, !same_sense, pending);
    }

    /// The file's cylinder, cone, sphere or torus with its own placement, over the window a face was read in: the angle runs from the file's reference direction.
    static Surface form_of(const AnFace& an, int su0, int nsu, double t0, double t1, int sv0, int nsv) {

        const bool round = an.kind == 4 || an.kind == 5;
        const std::array<double, 4> window = {su0 * PI_2, (su0 + nsu) * PI_2, round ? sv0 * PI_2 : t0, round ? (sv0 + nsv) * PI_2 : t1};

        switch (an.kind) {
        case 2: return Surface::cylinder(an.a.origin, an.a.az, an.a.ax, an.radius, window);
        case 3: return Surface::cone(an.a.origin, an.a.az, an.a.ax, an.radius, an.r2, window);
        case 4: return Surface::sphere(an.a.origin, an.a.az, an.a.ax, an.radius, window);
        default: return Surface::torus(an.a.origin, an.a.az, an.a.ax, an.radius, an.r2, window);
        }
    }

    /// The file's LINE, CIRCLE or ELLIPSE behind an edge curve entity, with its own placement; none for any other curve.
    std::optional<Curve> curve_of(int curve_id, int depth = 0) {

        const StepEntity* e = r.get(curve_id);

        if (!e || depth > MAX_DEPTH)
            return std::nullopt;

        if (const StepSubEntity* trim = e->find("TRIMMED_CURVE"))
            return curve_of(r.basis_curve_of(first_ref(trim->params)), depth + 1);

        if (const auto line = r.line_of(curve_id))
            return line->second.magnitude() > 0.0 ? std::optional<Curve>(Curve::line(line->first, line->second, 0.0, 1.0)) : std::nullopt;

        for (const char* type : {"CIRCLE", "ELLIPSE"})
            if (const StepSubEntity* conic = e->find(type)) {
                const Axis2 a = r.get_axis2(first_ref(conic->params));
                const std::vector<double> rr = nums(conic->params);

                if (!a.ok || rr.empty() || !(rr[0] > 0.0) || (rr.size() > 1 && !(rr[1] > 0.0)))
                    return std::nullopt;

                return rr.size() > 1 ? Curve::ellipse(a.origin, a.ax, a.ay, rr[0], rr[1]) : Curve::circle(a.origin, a.ax, a.ay, rr[0]);
            }

        return std::nullopt;
    }

    /// Geometry as the file has it, after the faces are built on NURBS: each edge on a line, circle or ellipse of the file runs over a range of that curve in its own parameter (the angle from the file's reference direction), any other line or circle is recognised; each face on a plane, cylinder, cone, sphere or torus of the file gets that surface and pcurves in its true parameters.
    void to_analytic(const StepIds& ids) {

        std::map<std::pair<int, bool>, int> pool;

        for (size_t i = 0; i < brep.m_edges.size(); i++) {
            BRepEdge& e = brep.m_edges[i];

            // A pcurve on a freeform surface runs over the parameter of the edge's B-spline: such an edge keeps it.
            if (e.curve_3d_index < 0 || !std::all_of(e.pcurves.begin(), e.pcurves.end(), [&](const BRepCurveOnSurface& use) { return forms.count(use.surface_index) > 0; }))
                continue;

            const NurbsCurve n = brep.m_curves_3d[e.curve_3d_index].nurbs();
            const int ec = std::abs(ids.edges[i]);
            const std::vector<int> refs = ec > 0 ? edge_refs(r, ec) : std::vector<int>();
            std::optional<Curve> c = refs.size() > 2 ? curve_of(r.basis_curve_of(refs[2])) : std::nullopt;

            // An edge the reader made (a seam) is a line or circle of its surface; a B-spline of the file stays one.
            if (!c) {
                const Curve found = ec == 0 ? Curve::recognized(n) : Curve(n);

                if (found.kind() != Curve::Kind::Nurbs)
                    e.curve_3d_index = brep.add_curve_3d(found);

                continue;
            }

            // The side of the curve the edge runs along: its middle lies between its ends.
            const Point p0 = n.point_at_start(), p1 = n.point_at_end(), pm = n.point_at(0.5 * (n.domain_start() + n.domain_end()));
            const double turn = c->period();
            double t0 = 0.0, t1 = 0.0;
            bool ok = false, reversed = false;

            for (int side = 0; side < 2 && !ok; side++) {
                if (side == 1)
                    c->reverse();

                reversed = side == 1;
                t0 = c->closest_parameter(p0);
                t1 = c->closest_parameter(p1);
                const double tm = c->closest_parameter(pm);

                if (turn > 0.0)
                    t1 = p0.distance(p1) <= tolerance && n.is_closed() ? t0 + turn : Curve::wrapped(t1, t0, turn);

                const double at = Curve::wrapped(tm, t0, turn);
                ok = t0 < at && at < t1 && c->tangent_at(at).dot(n.tangent_at(0.5 * (n.domain_start() + n.domain_end()))) > 0.0;
            }

            double off = 0.0;

            for (int k = 0; k <= 8 && ok; k++) {
                const Point q = n.point_at(n.domain_start() + (n.domain_end() - n.domain_start()) * k / 8.0);
                off = std::max(off, c->closest_point(q).distance(q));
            }

            if (!ok || off > std::max(tolerance, 1e-9 * (1.0 + c->radius())))
                continue;

            const auto found = pool.find({refs[2], reversed});
            e.curve_3d_index = found != pool.end() ? found->second : (pool[{refs[2], reversed}] = brep.add_curve_3d(*c));
            e.range = {t0, t1};
        }

        for (const auto& [si, form] : forms)
            brep.m_surfaces[si] = form;

        for (size_t i = 0; i < brep.m_edges.size(); i++) {
            BRepEdge& e = brep.m_edges[i];
            const std::optional<Curve> world = e.curve_3d_index >= 0 ? std::optional<Curve>(brep.edge_curve((int)i)) : std::nullopt;

            for (BRepCurveOnSurface& use : e.pcurves) {
                const auto form = forms.find(use.surface_index);

                if (form == forms.end())
                    continue;

                const Surface& s = form->second;
                const bool plane = s.kind() == Surface::Kind::Plane;

                // On a plane the pcurve of a freeform edge is already in the plane's own lengths.
                if (plane && world && world->kind() == Curve::Kind::Nurbs)
                    continue;

                for (int* index : {&use.curve_2d_index, &use.curve_2d_index_2}) {
                    if (*index < 0)
                        continue;

                    // The pcurve the face was read with, in quarter turns from the window's start, as true parameters: exact at its ends, a guide between them.
                    const NurbsCurve chart = brep.m_curves_2d[*index].nurbs();
                    const auto truth = [&](const Point& q) {
                        if (plane)
                            return q;

                        const double u = Curve::angle(s.domain(0).first + q[0] * PI_2, s.domain(0).first, s.domain(0).second);
                        const bool round = s.kind() == Surface::Kind::Sphere || s.kind() == Surface::Kind::Torus;

                        return Point(u, round ? Curve::angle(s.domain(1).first + q[1] * PI_2, s.domain(1).first, s.domain(1).second) : q[1], 0.0);
                    };

                    if (!world) {
                        const Point a = truth(chart.point_at_start()), b = truth(chart.point_at_end());

                        if (a.distance(b) > 0.0)
                            *index = brep.add_curve_2d(Curve2d::line(a[0], a[1], b[0], b[1]));

                        continue;
                    }

                    const std::pair<double, double> t = world->domain();
                    double deviation = 0.0;
                    const Curve2d pc = true_pcurve(s, *world, [&](double at) {
                        return truth(chart.point_at(chart.domain_start() + (at - t.first) / (t.second - t.first) * (chart.domain_end() - chart.domain_start())));
                    }, deviation);
                    *index = brep.add_curve_2d(pc);
                    e.tolerance = std::max(e.tolerance, deviation > 1e-9 ? deviation : e.tolerance);
                }
            }
        }
    }

    /// ADVANCED_FACE or FACE_SURFACE: vertex-loop face, analytic face, or face on a plane or B-spline surface.
    void add_face(int face_id) {

        const StepEntity* fent = r.get(face_id);
        const StepSubEntity* face = fent ? fent->find("ADVANCED_FACE") : nullptr;

        if (fent && !face)
            face = fent->find("FACE_SURFACE");

        if (!face)
            throw std::runtime_error("not an ADVANCED_FACE or FACE_SURFACE");

        const std::vector<int> bound_refs = list_refs(face->params);
        const int surface_ref = first_ref(face->params);
        const bool same_sense = last_flag(face->params, true);
        const std::vector<int> vl_ids = vertex_loop_ids(bound_refs);

        if (!vl_ids.empty()) {
            if (!add_face_vertex_loop(vl_ids, surface_ref, same_sense))
                throw std::runtime_error("vertex-loop face is not a closed sphere- or torus-like surface");

            return;
        }

        const AnFace an = r.get_analytic_srf(surface_ref);

        if (an.kind >= 2)
            add_face_analytic(bound_refs, same_sense, an);
        else
            add_face_projected(bound_refs, surface_ref, same_sense);
    }

    /// BRep of faces of a CLOSED_SHELL (one solid, its faces oriented consistently) or OPEN_SHELL (one shell) with tolerances raised to the gaps between its curves and vertices; throws naming the first face that cannot be built.
    BRep build_from_shell(int shell_id, const std::vector<int>& faces, StepIds& ids) {

        const StepEntity* sent = r.get(shell_id);
        brep.name = "step_brep";

        for (int f : faces) {
            try {
                add_face(f);
            } catch (const std::exception& e) {
                throw std::runtime_error("STEP face #" + std::to_string(f) + ": " + e.what());
            }

            ids.faces.resize(brep.m_faces.size(), f);
        }

        apply_splits();
        ids.edges.assign(brep.m_edges.size(), 0);

        for (const auto& [ec, edge] : emap)
            ids.edges[edge] = ec;

        for (const auto& [edge, cut] : splits)
            ids.edges[std::get<0>(cut)] = ids.edges[std::get<1>(cut)] = -std::abs(ids.edges[edge]);

        if (sent->has("CLOSED_SHELL"))
            orient_shell(brep, face_refs);

        if (!face_refs.empty()) {
            const int sh = brep.add_shell(face_refs);

            if (sent->has("CLOSED_SHELL"))
                brep.add_solid({{sh, BRepOrientation::Forward}});
        }

        to_analytic(ids);
        brep.update_tolerances();

        return std::move(brep);
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// StepWriter
// ═══════════════════════════════════════════════════════════════════════════
/// ISO 10303-21 REAL: a decimal point in the mantissa and an uppercase E.
static std::string fmt(double v) {

    std::string s;

    if (std::abs(v) < 1e15 && v == (double)(long long)v)
        s = fmt::format("{}.", (long long)v);
    else
        s = fmt::format("{}", v);

    const size_t e = s.find('e');

    if (e != std::string::npos) {
        s[e] = 'E';

        if (s.substr(0, e).find('.') == std::string::npos)
            s.insert(e, ".");
    } else if (s.find('.') == std::string::npos) {
        s += ".";
    }

    return s;
}

/// Format integers as a STEP list.
static std::string fmt_int_list(const std::vector<int>& items) {

    std::string s = "(";

    for (size_t i = 0; i < items.size(); i++)
        s += (i ? "," : "") + std::to_string(items[i]);

    return s + ")";
}

/// Format doubles as a STEP list.
static std::string fmt_dbl_list(const std::vector<double>& items) {

    std::string s = "(";

    for (size_t i = 0; i < items.size(); i++)
        s += (i ? "," : "") + fmt(items[i]);

    return s + ")";
}

/// Format ids as a STEP reference list.
static std::string fmt_ref_list(const std::vector<int>& ids) {

    std::string s = "(";

    for (size_t i = 0; i < ids.size(); i++)
        s += (i ? ",#" : "#") + std::to_string(ids[i]);

    return s + ")";
}

/// Format id rows as a STEP list of reference lists.
static std::string fmt_ref_grid(const std::vector<std::vector<int>>& rows) {

    std::string s = "(";

    for (size_t i = 0; i < rows.size(); i++)
        s += (i ? "," : "") + fmt_ref_list(rows[i]);

    return s + ")";
}

/// Format double rows as a STEP list of lists.
static std::string fmt_dbl_grid(const std::vector<std::vector<double>>& rows) {

    std::string s = "(";

    for (size_t i = 0; i < rows.size(); i++)
        s += (i ? "," : "") + fmt_dbl_list(rows[i]);

    return s + ")";
}

/// Entity lines of a STEP file under construction.
class StepWriter {
    int next_id = 1; // Next free entity id.
    std::vector<std::string> lines; // Emitted entity lines.

public:

    /// Return the next free entity id.
    int new_id() {
        return next_id++;
    }

    /// Emit "#id=body;" with a fresh id and return the id.
    int write_raw(const std::string& body) {

        const int id = new_id();
        lines.push_back("#" + std::to_string(id) + "=" + body + ";");

        return id;
    }

    /// Emit a CARTESIAN_POINT and return its id.
    int write_point(double x, double y, double z) {
        return write_raw("CARTESIAN_POINT('',(" + fmt(x) + "," + fmt(y) + "," + fmt(z) + "))");
    }

    /// B_SPLINE_CURVE_WITH_KNOTS, as a complete complex instance when rational, on 2D points for a pcurve; -1 for an invalid curve.
    int write_nurbs_curve(const NurbsCurve& nc, bool planar = false) {

        if (!nc.is_valid())
            return -1;

        std::vector<int> pt_ids;
        std::vector<double> weights;

        for (int i = 0; i < nc.cv_count(); i++) {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
            std::tie(x, y, z, w) = nc.get_cv_4d(i);

            if (std::abs(w) < 1e-14)
                w = 1.0;

            pt_ids.push_back(planar ? write_raw("CARTESIAN_POINT('',(" + fmt(x / w) + "," + fmt(y / w) + "))") : write_point(x / w, y / w, z / w));
            weights.push_back(w);
        }

        std::vector<double> kvals;
        std::vector<int> kmults;
        std::tie(kvals, kmults) = compress_knots(full_from_internal(nc.m_nurbsknot));

        const std::string degree = std::to_string(nc.m_order - 1);

        if (!nc.is_rational())
            return write_raw(
                "B_SPLINE_CURVE_WITH_KNOTS(''," + degree + "," + fmt_ref_list(pt_ids) + ",.UNSPECIFIED.,.F.,.U.," +
                fmt_int_list(kmults) + "," + fmt_dbl_list(kvals) + ",.UNSPECIFIED.)"
            );

        return write_raw(
            "(BOUNDED_CURVE()B_SPLINE_CURVE(" + degree + "," + fmt_ref_list(pt_ids) + ",.UNSPECIFIED.,.F.,.U.)" +
            "B_SPLINE_CURVE_WITH_KNOTS(" + fmt_int_list(kmults) + "," + fmt_dbl_list(kvals) + ",.UNSPECIFIED.)" +
            "CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(" + fmt_dbl_list(weights) +
            ")REPRESENTATION_ITEM(''))"
        );
    }

    /// B_SPLINE_SURFACE_WITH_KNOTS, as a complete complex instance when rational; -1 for an invalid surface.
    int write_nurbs_surface(const NurbsSurface& srf) {

        if (!srf.is_valid())
            return -1;

        const int cv_u = srf.m_cv_count[0];
        const int cv_v = srf.m_cv_count[1];
        std::vector<std::vector<int>> pt_ids(cv_u, std::vector<int>(cv_v));
        std::vector<std::vector<double>> weight_grid(cv_u, std::vector<double>(cv_v, 1.0));

        for (int u = 0; u < cv_u; u++)
            for (int v = 0; v < cv_v; v++) {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                double w = 1.0;

                if (!srf.get_cv_4d(u, v, x, y, z, w))
                    return -1;

                if (std::abs(w) < 1e-14)
                    w = 1.0;

                pt_ids[u][v] = write_point(x / w, y / w, z / w);
                weight_grid[u][v] = w;
            }

        std::vector<double> ku_vals;
        std::vector<int> ku_mults;
        std::vector<double> kv_vals;
        std::vector<int> kv_mults;
        std::tie(ku_vals, ku_mults) = compress_knots(full_from_internal(srf.m_nurbsknot[0]));
        std::tie(kv_vals, kv_mults) = compress_knots(full_from_internal(srf.m_nurbsknot[1]));

        const std::string degrees = std::to_string(srf.m_order[0] - 1) + "," + std::to_string(srf.m_order[1] - 1);
        const std::string knots = fmt_int_list(ku_mults) + "," + fmt_int_list(kv_mults) + "," + fmt_dbl_list(ku_vals) +
            "," + fmt_dbl_list(kv_vals);

        if (!srf.is_rational())
            return write_raw(
                "B_SPLINE_SURFACE_WITH_KNOTS(''," + degrees + "," + fmt_ref_grid(pt_ids) +
                ",.UNSPECIFIED.,.F.,.F.,.U.," + knots + ",.UNSPECIFIED.)"
            );

        return write_raw(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE(" + degrees + "," + fmt_ref_grid(pt_ids) +
            ",.UNSPECIFIED.,.F.,.F.,.U.)" + "B_SPLINE_SURFACE_WITH_KNOTS(" + knots +
            ",.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()" + "RATIONAL_B_SPLINE_SURFACE(" +
            fmt_dbl_grid(weight_grid) + ")REPRESENTATION_ITEM('')SURFACE())"
        );
    }

    /// FACE_OUTER_BOUND or FACE_BOUND of one closed trim loop: its 3D image sampled as a polyline edge on one vertex.
    int write_loop_as_face_bound(const NurbsSurfaceTrimmed& trimmed, const NurbsCurve& loop_2d, bool is_outer) {

        if (!loop_2d.is_valid())
            return -1;

        std::vector<Point> pts3d;

        for (const Point& uv : sample_nurbs(loop_2d, std::max(2, loop_2d.cv_count() * 2)))
            pts3d.push_back(trimmed.m_surface.point_at(uv[0], uv[1]));

        const int v0 =
            write_raw("VERTEX_POINT('',#" + std::to_string(write_point(pts3d[0][0], pts3d[0][1], pts3d[0][2])) + ")");

        const int crv3d = write_nurbs_curve(polyline_nurbs(pts3d, 3));

        if (crv3d < 0)
            return -1;

        if (write_nurbs_curve(loop_2d) < 0)
            return -1;

        const int ec = write_raw(
            "EDGE_CURVE('',#" + std::to_string(v0) + ",#" + std::to_string(v0) + ",#" + std::to_string(crv3d) + ",.T.)"
        );

        const int oe = write_raw("ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + ",.T.)");
        const int el = write_raw("EDGE_LOOP('',(#" + std::to_string(oe) + "))");

        return write_raw(
            std::string(is_outer ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "('',#" + std::to_string(el) + ",.T.)"
        );
    }

    /// ADVANCED_FACE of a trimmed surface; -1 when the surface or the outer loop cannot be written.
    int write_trimmed_face(const NurbsSurfaceTrimmed& trimmed) {

        const int srf_id = write_nurbs_surface(trimmed.m_surface);

        if (srf_id < 0)
            return -1;

        std::vector<int> bounds;
        bounds.push_back(write_loop_as_face_bound(trimmed, trimmed.m_outer_loop, true));

        if (bounds[0] < 0)
            return -1;

        for (const NurbsCurve& inner : trimmed.m_inner_loops) {
            const int ib = write_loop_as_face_bound(trimmed, inner, false);

            if (ib >= 0)
                bounds.push_back(ib);
        }

        return write_raw("ADVANCED_FACE(''," + fmt_ref_list(bounds) + ",#" + std::to_string(srf_id) + ",.T.)");
    }

    /// AP214 surface-color chain; returns the PRESENTATION_STYLE_ASSIGNMENT for STYLED_ITEMs.
    int color_style(double r, double g, double b) {

        const int c = write_raw("COLOUR_RGB(''," + fmt(r) + "," + fmt(g) + "," + fmt(b) + ")");
        const int fc = write_raw("FILL_AREA_STYLE_COLOUR('',#" + std::to_string(c) + ")");
        const int fa = write_raw("FILL_AREA_STYLE('',(#" + std::to_string(fc) + "))");
        const int sf = write_raw("SURFACE_STYLE_FILL_AREA(#" + std::to_string(fa) + ")");
        const int ss = write_raw("SURFACE_SIDE_STYLE('',(#" + std::to_string(sf) + "))");
        const int su = write_raw("SURFACE_STYLE_USAGE(.BOTH.,#" + std::to_string(ss) + ")");

        return write_raw("PRESENTATION_STYLE_ASSIGNMENT((#" + std::to_string(su) + "))");
    }

    /// CLOSED_SHELL + MANIFOLD_SOLID_BREP or OPEN_SHELL + SHELL_BASED_SURFACE_MODEL over the faces; -1 when there are none.
    int write_body(const std::vector<int>& faces, bool closed) {

        if (faces.empty())
            return -1;

        const int shell =
            write_raw(std::string(closed ? "CLOSED_SHELL" : "OPEN_SHELL") + "(''," + fmt_ref_list(faces) + ")");

        if (closed)
            return write_raw("MANIFOLD_SOLID_BREP('',#" + std::to_string(shell) + ")");

        return write_raw("SHELL_BASED_SURFACE_MODEL('',(#" + std::to_string(shell) + "))");
    }

    /// Emit a DIRECTION and return its id.
    int write_direction(const Vector& d) {
        return write_raw("DIRECTION('',(" + fmt(d[0]) + "," + fmt(d[1]) + "," + fmt(d[2]) + "))");
    }

    /// AXIS2_PLACEMENT_3D at an origin with its axis and reference direction.
    int write_axis(const Point& origin, const Vector& z, const Vector& x) {
        const int o = write_point(origin[0], origin[1], origin[2]);
        const int dz = write_direction(z);
        return write_raw("AXIS2_PLACEMENT_3D('',#" + std::to_string(o) + ",#" + std::to_string(dz) + ",#" + std::to_string(write_direction(x)) + ")");
    }

    /// AXIS2_PLACEMENT_3D of a placement's frame: its translation, its z and x columns.
    int write_axis(const Xform& x) {

        const int o = write_point(x.m[12], x.m[13], x.m[14]);
        const int dz = write_raw("DIRECTION('',(" + fmt(x.m[8]) + "," + fmt(x.m[9]) + "," + fmt(x.m[10]) + "))");
        const int dx = write_raw("DIRECTION('',(" + fmt(x.m[0]) + "," + fmt(x.m[1]) + "," + fmt(x.m[2]) + "))");

        return write_raw("AXIS2_PLACEMENT_3D('',#" + std::to_string(o) + ",#" + std::to_string(dz) + ",#" + std::to_string(dx) + ")");
    }

    /// Representation context (millimetres, radians, uncertainty the sewing tolerance) and AP214 application context; returns the
    /// representation, application, product and product definition contexts.
    std::array<int, 4> write_context(double uncertainty) {

        const int lu = write_raw("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
        const int au = write_raw("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
        const int su = write_raw("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");

        if (!std::isfinite(uncertainty) || uncertainty <= 0.0)
            uncertainty = 1e-6;

        const int un = write_raw(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" + fmt(uncertainty) + "),#" + std::to_string(lu) +
            ",'distance_accuracy_value','')"
        );

        const int gc = write_raw(
            "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" + std::to_string(un) +
            "))GLOBAL_UNIT_ASSIGNED_CONTEXT((#" + std::to_string(lu) + ",#" + std::to_string(au) + ",#" +
            std::to_string(su) + "))REPRESENTATION_CONTEXT('',''))"
        );

        const int ac = write_raw("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
        write_raw(
            "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#" + std::to_string(ac) +
            ")"
        );

        const int pc = write_raw("PRODUCT_CONTEXT('',#" + std::to_string(ac) + ",'mechanical')");
        const int dc = write_raw("PRODUCT_DEFINITION_CONTEXT('part definition',#" + std::to_string(ac) + ",'design')");

        return {gc, ac, pc, dc};
    }

    /// PRODUCT with its definition, its representation of the items and the SHAPE_DEFINITION_REPRESENTATION joining them; returns the
    /// product definition and the representation.
    std::pair<int, int> write_product(const std::string& name, const std::string& type, const std::vector<int>& items, const std::array<int, 4>& c) {

        const int pr = write_raw("PRODUCT('" + name + "','" + name + "','',(#" + std::to_string(c[2]) + "))");
        const int pf = write_raw("PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(pr) + ")");
        const int pd = write_raw("PRODUCT_DEFINITION('design','',#" + std::to_string(pf) + ",#" + std::to_string(c[3]) + ")");
        const int ps = write_raw("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(pd) + ")");
        const int rp = write_raw(type + "('" + name + "'," + fmt_ref_list(items) + ",#" + std::to_string(c[0]) + ")");
        write_raw("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ps) + ",#" + std::to_string(rp) + ")");

        return {pd, rp};
    }

    /// Representation type of bodies: advanced B-rep when one is closed, manifold surface otherwise, plain shape without any.
    static std::string representation_type(const std::vector<int>& bodies, bool closed) {
        return bodies.empty() ? "SHAPE_REPRESENTATION" : closed ? "ADVANCED_BREP_SHAPE_REPRESENTATION" : "MANIFOLD_SURFACE_SHAPE_REPRESENTATION";
    }

    /// AP214 PRODUCT and SHAPE_DEFINITION_REPRESENTATION skeleton importers need to find the bodies; uncertainty is the sewing tolerance.
    void finish_product(
        const std::vector<int>& bodies,
        bool closed,
        const std::string& name,
        double uncertainty = 1e-6,
        const std::vector<int>& styled_items = {}
    ) {

        std::vector<int> items = {write_axis(Xform::identity())};
        const std::array<int, 4> c = write_context(uncertainty);
        items.insert(items.end(), bodies.begin(), bodies.end());
        write_product(name, representation_type(bodies, closed), items, c);

        if (!styled_items.empty())
            write_raw(
                "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION(''," + fmt_ref_list(styled_items) + ",#" +
                std::to_string(c[0]) + ")"
            );
    }

    /// Return the complete STEP text with header and data sections.
    std::string emit() const {

        std::string out = "ISO-10303-21;\nHEADER;\n";
        out += "FILE_DESCRIPTION((''),'2;1');\n";
        out += "FILE_NAME('','',(''),(''),'','','');\n";
        out += "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n";
        out += "ENDSEC;\nDATA;\n";

        for (const std::string& l : lines)
            out += l + "\n";

        out += "ENDSEC;\nEND-ISO-10303-21;\n";

        return out;
    }
};

/// Write one BRep into a StepWriter: vertices, edges and surfaces once each, degenerated edges omitted, a wire of only degenerated edges as a VERTEX_LOOP.
class BRepEmitter {
    StepWriter& w; // Entity writer.
    const BRep& brep; // Brep being emitted.
    std::map<int, int> vid; // VERTEX_POINT id by vertex.
    std::map<int, int> eid; // EDGE_CURVE id by edge.
    std::map<int, int> sid; // Surface id by surface.
    std::map<int, bool> flipped; // Whether the written surface's normal runs against the brep surface's.
    int context_2d = -1; // Parameter-space context of every PCURVE.

public:

    /// Construct over a writer and the brep to emit.
    BRepEmitter(StepWriter& writer, const BRep& b) : w(writer), brep(b) {}

    /// Return the VERTEX_POINT id of a brep vertex, emitting it once.
    int vertex_id(int vi) {

        const auto it = vid.find(vi);

        if (it != vid.end())
            return it->second;

        const Point& p = brep.m_vertices[vi].point;

        return vid[vi] = w.write_raw("VERTEX_POINT('',#" + std::to_string(w.write_point(p[0], p[1], p[2])) + ")");
    }

    /// Return the EDGE_CURVE id of a brep edge, emitting it once.
    int edge_id(int ei) {

        const auto it = eid.find(ei);

        if (it != eid.end())
            return it->second;

        const BRepEdge& e = brep.m_edges[ei];
        const Curve curve = brep.edge_curve(ei);
        const NurbsCurve& world = curve.nurbs();
        const bool analytic = curve.kind() == Curve::Kind::Line || curve.kind() == Curve::Kind::Circle || curve.kind() == Curve::Kind::Ellipse;
        // A line, circle or ellipse as itself, trimmed by the edge's vertices and running with its parameter; pcurves only where edge, pcurve and surface are all B-splines in one parameter.
        const int c = !analytic ? w.write_nurbs_curve(world)
            : curve.kind() == Curve::Kind::Line ? w.write_raw("LINE('',#" + std::to_string(w.write_point(curve.origin()[0], curve.origin()[1], curve.origin()[2])) + ",#" +
                std::to_string(w.write_raw("VECTOR('',#" + std::to_string(w.write_direction(curve.x())) + ",1.)")) + ")")
            : w.write_raw(std::string(curve.kind() == Curve::Kind::Circle ? "CIRCLE" : "ELLIPSE") + "('',#" + std::to_string(w.write_axis(curve.origin(), curve.z(), curve.x())) + "," + fmt(curve.radius()) +
                (curve.kind() == Curve::Kind::Circle ? "" : "," + fmt(curve.minor())) + ")");

        if (c < 0)
            return eid[ei] = -1;

        std::vector<int> pcurves;
        bool seam = false;

        for (const BRepCurveOnSurface& pc : e.pcurves)
            for (int k = 0; k < 2 && !analytic && brep.m_surfaces[pc.surface_index].kind() == Surface::Kind::Nurbs; k++) {
                const int ci = k == 0 ? pc.curve_2d_index : pc.curve_2d_index_2;
                NurbsCurve c2d = ci >= 0 && brep.m_curves_2d[ci].kind() == Curve::Kind::Nurbs ? brep.pcurve_nurbs(pc, k == 1) : NurbsCurve();
                const int srf = c2d.is_valid() ? surface_id(pc.surface_index) : -1;
                const int curve = srf >= 0 && c2d.set_domain(world.domain_start(), world.domain_end()) ? w.write_nurbs_curve(c2d, true) : -1;

                if (curve < 0)
                    continue;

                if (context_2d < 0)
                    context_2d = w.write_raw("(GEOMETRIC_REPRESENTATION_CONTEXT(2)PARAMETRIC_REPRESENTATION_CONTEXT()REPRESENTATION_CONTEXT('2D SPACE',''))");

                const int rep = w.write_raw("DEFINITIONAL_REPRESENTATION('',(#" + std::to_string(curve) + "),#" + std::to_string(context_2d) + ")");
                pcurves.push_back(w.write_raw("PCURVE('',#" + std::to_string(srf) + ",#" + std::to_string(rep) + ")"));
                seam = seam || k == 1;
            }

        const int geometry = pcurves.empty() ? c : w.write_raw(std::string(seam ? "SEAM_CURVE" : "SURFACE_CURVE") + "('',#" + std::to_string(c) + "," + fmt_ref_list(pcurves) + ",.PCURVE_S1.)");
        const int sv = vertex_id(e.start_vertex);
        const int ev = vertex_id(e.end_vertex);

        return eid[ei] = w.write_raw(
                   "EDGE_CURVE('',#" + std::to_string(sv) + ",#" + std::to_string(ev) + ",#" + std::to_string(geometry) +
                   ",.T.)"
               );
    }

    /// Return the surface id of a brep surface, emitting it once.
    int surface_id(int si) {

        const auto it = sid.find(si);

        if (it != sid.end())
            return it->second;

        const Surface& s = brep.m_surfaces[si];
        const Surface::Kind kind = s.kind();

        if (kind != Surface::Kind::Plane && kind != Surface::Kind::Cylinder && kind != Surface::Kind::Cone && kind != Surface::Kind::Sphere && kind != Surface::Kind::Torus)
            return sid[si] = w.write_nurbs_surface(s.nurbs());

        // The same surface on a right-handed placement, a cone widening along its axis; the face's sense follows the normal of what is written.
        const Vector z = kind == Surface::Kind::Cone && s.minor() < 0.0 ? s.z() * -1.0 : s.z();
        const double angle = std::abs(s.minor());
        const std::array<double, 4> any = {0.0, 1.0, 0.0, 1.0};
        const Surface written = kind == Surface::Kind::Plane ? Surface::plane(s.origin(), z, s.x(), any) : kind == Surface::Kind::Cylinder ? Surface::cylinder(s.origin(), z, s.x(), s.radius(), any)
            : kind == Surface::Kind::Cone ? Surface::cone(s.origin(), z, s.x(), s.radius(), angle, any) : kind == Surface::Kind::Sphere ? Surface::sphere(s.origin(), z, s.x(), s.radius(), any)
            : Surface::torus(s.origin(), z, s.x(), s.radius(), s.minor(), any);
        const double u = 0.5 * (s.domain(0).first + s.domain(0).second) + 1e-3, v = 0.5 * (s.domain(1).first + s.domain(1).second) + 1e-3;
        const std::pair<double, double> uv = written.closest_parameters(s.point_at(u, v));
        flipped[si] = written.normal_at(uv.first, uv.second).dot(s.normal_at(u, v)) < 0.0;
        const std::string axis = "('',#" + std::to_string(w.write_axis(s.origin(), z, s.x()));

        switch (kind) {
        case Surface::Kind::Plane: return sid[si] = w.write_raw("PLANE" + axis + ")");
        case Surface::Kind::Cylinder: return sid[si] = w.write_raw("CYLINDRICAL_SURFACE" + axis + "," + fmt(s.radius()) + ")");
        case Surface::Kind::Cone: return sid[si] = w.write_raw("CONICAL_SURFACE" + axis + "," + fmt(s.radius()) + "," + fmt(angle) + ")");
        case Surface::Kind::Sphere: return sid[si] = w.write_raw("SPHERICAL_SURFACE" + axis + "," + fmt(s.radius()) + ")");
        default: return sid[si] = w.write_raw("TOROIDAL_SURFACE" + axis + "," + fmt(s.radius()) + "," + fmt(s.minor()) + ")");
        }
    }

    /// EDGE_LOOP of the non-degenerated edges, a VERTEX_LOOP when there are none, -1 for an empty wire.
    int wire_id(const BRepRef& wire, int surface) {

        std::vector<int> oes;
        int any_vertex = -1;
        // The whole of a sphere or torus is bounded by seams and poles alone, each seam there and back: a VERTEX_LOOP, as its natural bounds are written.
        const Surface::Kind kind = brep.m_surfaces[surface].kind();
        std::map<int, int> uses;

        for (const BRepRef& er : brep.wire_edges(wire))
            if (!brep.m_edges[er.index].degenerated)
                uses[er.index] += er.orientation == BRepOrientation::Forward ? 1 : -1;

        const bool whole = (kind == Surface::Kind::Sphere || kind == Surface::Kind::Torus) && std::all_of(uses.begin(), uses.end(), [](const auto& use) { return use.second == 0; });

        for (const BRepRef& er : brep.wire_edges(wire)) {
            const BRepEdge& e = brep.m_edges[er.index];

            if (any_vertex < 0)
                any_vertex = e.start_vertex;

            if (e.degenerated || whole)
                continue;

            const int ec = edge_id(er.index);

            if (ec < 0)
                continue;

            const std::string sense = er.orientation == BRepOrientation::Forward ? "T" : "F";
            oes.push_back(w.write_raw("ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + ",." + sense + ".)"));
        }

        if (!oes.empty())
            return w.write_raw("EDGE_LOOP(''," + fmt_ref_list(oes) + ")");

        if (any_vertex >= 0)
            return w.write_raw("VERTEX_LOOP('',#" + std::to_string(vertex_id(any_vertex)) + ")");

        return -1;
    }

    /// Return the ADVANCED_FACE id of a brep face, emitting it once: the loops run on the surface, so a reversed face's bounds carry .F. as its same_sense does.
    int face_id(int fi, BRepOrientation fo) {

        const BRepFace& f = brep.m_faces[fi];
        const int srf = surface_id(f.surface_index);

        if (srf < 0)
            return -1;

        std::vector<int> bounds;
        const std::string sense = fo == BRepOrientation::Forward ? "T" : "F";

        for (size_t wi = 0; wi < f.wires.size(); wi++) {
            const int loop = wire_id(f.wires[wi], f.surface_index);

            if (loop < 0)
                continue;

            bounds.push_back(w.write_raw(
                std::string(wi == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "('',#" + std::to_string(loop) + ",." + sense + ".)"
            ));
        }

        if (bounds.empty())
            return -1;

        return w.write_raw(
            "ADVANCED_FACE(''," + fmt_ref_list(bounds) + ",#" + std::to_string(srf) + ",." + ((sense == "T") != flipped[f.surface_index] ? "T" : "F") + ".)"
        );
    }
};

/// Face-id groups of a brep written into w: one per shell with its closed flag, then the free faces as an open group.
static std::vector<std::pair<std::vector<int>, bool>> emit_brep_shells(StepWriter& w, const BRep& brep) {


    BRepEmitter em(w, brep);
    std::vector<std::pair<std::vector<int>, bool>> groups;
    std::vector<bool> in_shell(brep.m_faces.size(), false);

    for (int si = 0; si < brep.shell_count(); si++) {
        std::vector<int> ids;

        for (const BRepRef& fr : brep.m_shells[si].faces) {
            in_shell[fr.index] = true;

            const int id = em.face_id(fr.index, fr.orientation);

            if (id >= 0)
                ids.push_back(id);
        }

        if (!ids.empty())
            groups.push_back({ids, brep.is_closed(si)});
    }

    std::vector<int> free_ids;

    for (int fi = 0; fi < brep.face_count(); fi++) {
        const int id = in_shell[fi] ? -1 : em.face_id(fi, BRepOrientation::Forward);

        if (id >= 0)
            free_ids.push_back(id);
    }

    if (!free_ids.empty())
        groups.push_back({free_ids, false});

    return groups;
}

/// Bounding-box diagonal of the vertices, 1 when there are none.
static double vertex_diagonal(const BRep& brep) {

    if (brep.m_vertices.empty())
        return 1.0;

    Point lo(1e300, 1e300, 1e300);
    Point hi(-1e300, -1e300, -1e300);

    for (const BRepVertex& v : brep.m_vertices)
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], v.point[k]);
            hi[k] = std::max(hi[k], v.point[k]);
        }

    return lo.distance(hi);
}

/// Write the STEP text to a file; false when it cannot be written.
static bool write_step_string(const std::string& content, const std::string& filepath) {

    std::ofstream out(filepath);
    out << content;
    out.close();

    return !out.fail();
}

// ═══════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════
std::vector<Point> read_file_step_points(const std::string& filepath) {

    const StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<Point> out;

    for (int id : sf.ids_of_type("CARTESIAN_POINT"))
        out.push_back(r.get_point(id));

    return out;
}

std::vector<NurbsCurve> read_file_step_nurbscurves(const std::string& filepath) {

    const StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<NurbsCurve> out;

    for (int id : sf.ids_of_type("B_SPLINE_CURVE_WITH_KNOTS")) {
        const NurbsCurve nc = r.get_nurbs_curve(id);

        if (nc.is_valid())
            out.push_back(nc);
    }

    return out;
}

std::vector<NurbsSurface> read_file_step_nurbssurfaces(const std::string& filepath) {

    const StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<NurbsSurface> out;

    for (int id : sf.ids_of_type("B_SPLINE_SURFACE_WITH_KNOTS")) {
        const NurbsSurface srf = r.get_nurbs_surface(id);

        if (srf.is_valid())
            out.push_back(srf);
    }

    return out;
}

/// Outer trim of a face as a dim-2 polyline of sampled 3D edge points (x, y), from the first bound with an EDGE_LOOP.
static NurbsCurve trimmed_outer_loop(StepReader& r, const std::vector<int>& bound_refs) {

    for (int bid : bound_refs) {
        const std::optional<Bound> b = bound_loop(r, bid);

        if (!b)
            continue;

        std::vector<Point> uv_pts;

        for (int oe_id : b->oe_refs) {
            const std::vector<int> ecr = edge_refs(r, oriented_edge(r, oe_id).first);

            if (ecr.size() < 3)
                continue;

            const Point vs = r.get_vertex_point(ecr[0]).value_or(Point(0, 0, 0));
            const Point ve = r.get_vertex_point(ecr[1]).value_or(Point(0, 0, 0));

            for (const Point& s : r.sample_curve(ecr[2], vs, ve, 8))
                uv_pts.emplace_back(s[0], s[1], 0.0);
        }

        return polyline_nurbs(uv_pts, 2);
    }

    return NurbsCurve();
}

std::vector<NurbsSurfaceTrimmed> read_file_step_nurbssurfaces_trimmed(const std::string& filepath) {

    const StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<NurbsSurfaceTrimmed> out;

    for (int face_id : sf.ids_of_type("ADVANCED_FACE")) {
        const StepEntity* fent = r.get(face_id);
        const StepSubEntity* face = fent ? fent->find("ADVANCED_FACE") : nullptr;

        if (!face)
            continue;

        const int surface_ref = first_ref(face->params);
        const StepEntity* surf = r.get(surface_ref);

        if (!surf || !surf->has("B_SPLINE_SURFACE_WITH_KNOTS"))
            continue;

        NurbsSurfaceTrimmed nst;
        nst.m_surface = r.get_nurbs_surface(surface_ref);
        nst.m_outer_loop = trimmed_outer_loop(r, list_refs(face->params));

        if (nst.m_surface.is_valid() && nst.m_outer_loop.is_valid())
            out.push_back(nst);
    }

    return out;
}

/// The solid model of an entity: MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS or SHELL_BASED_SURFACE_MODEL; none otherwise.
static const StepSubEntity* model_of(const StepReader& r, int id) {

    const StepEntity* e = r.get(id);

    for (const char* type : {"MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS", "SHELL_BASED_SURFACE_MODEL"})
        if (const StepSubEntity* sub = e ? e->find(type) : nullptr)
            return sub;

    return nullptr;
}

/// Solid models with their placements, as STEPControl_ActorRead transfers an assembly: from each product no
/// NEXT_ASSEMBLY_USAGE_OCCURRENCE uses down its tree, the models of its shape representations (and of those a
/// SHAPE_REPRESENTATION_RELATIONSHIP joins them to), each child moved by the ITEM_DEFINED_TRANSFORMATION of its
/// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION, from the axis in the child's representation to the axis in the parent's.
/// Without an assembly every model once, in file order, unmoved.
static std::vector<std::pair<int, std::optional<Xform>>> placed_models(StepReader& r, const std::vector<int>& order) {

    std::vector<std::pair<int, std::optional<Xform>>> out;
    std::map<int, int> shape_of; // PRODUCT_DEFINITION_SHAPE by the product definition or occurrence it defines.
    std::map<int, std::vector<int>> reps; // Representations by PRODUCT_DEFINITION_SHAPE.
    std::map<int, std::vector<int>> joined; // Representations a relationship without transformation joins.
    std::map<int, int> placement_of; // Representation relationship by the PRODUCT_DEFINITION_SHAPE of an occurrence.
    std::vector<std::array<int, 3>> uses; // Occurrence, parent and child product definitions.

    for (int id : order) {
        const StepEntity* e = r.get(id);
        const std::vector<int> refs = all_refs(e->parts.front().params);

        if (model_of(r, id))
            out.push_back({id, std::nullopt});
        else if (e->has("PRODUCT_DEFINITION_SHAPE") && !refs.empty())
            shape_of[refs[0]] = id;
        else if (e->has("SHAPE_DEFINITION_REPRESENTATION") && refs.size() >= 2)
            reps[refs[0]].push_back(refs[1]);
        else if (e->has("NEXT_ASSEMBLY_USAGE_OCCURRENCE") && refs.size() >= 2)
            uses.push_back({id, refs[0], refs[1]});
        else if (e->has("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") && refs.size() >= 2)
            placement_of[refs[1]] = refs[0];
        else if (e->has("SHAPE_REPRESENTATION_RELATIONSHIP") && !e->has("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") && refs.size() >= 2) {
            joined[refs[0]].push_back(refs[1]);
            joined[refs[1]].push_back(refs[0]);
        }
    }

    if (uses.empty())
        return out;

    // The representations of a product definition, closed under the joining relationships.
    const auto closure = [&](int pd) {
        std::vector<int> found = shape_of.count(pd) ? reps[shape_of[pd]] : std::vector<int>();

        for (size_t k = 0; k < found.size(); k++)
            for (int next : joined[found[k]])
                if (std::find(found.begin(), found.end(), next) == found.end())
                    found.push_back(next);

        return found;
    };
    const auto frame = [&](int axis) {
        const Axis2 a = r.get_axis2(axis);
        return std::pair<Xform, Xform>{Xform::frame_to_world(a.origin, a.ax, a.ay, a.az), Xform::world_to_frame(a.origin, a.ax, a.ay, a.az)};
    };
    const std::vector<std::pair<int, std::optional<Xform>>> unplaced = out;
    out.clear();
    std::function<void(int, const Xform&, int)> visit = [&](int pd, const Xform& placement, int depth) {
        for (int rep : closure(pd))
            for (int item : list_refs(r.get(rep) ? r.get(rep)->parts.front().params : std::vector<StepParam>()))
                if (model_of(r, item))
                    out.push_back({item, placement});

        for (const auto& [occurrence, parent, child] : uses) {
            const auto relation = parent == pd && depth < 64 && shape_of.count(occurrence) ? placement_of.find(shape_of[occurrence]) : placement_of.end();
            const StepEntity* rel = relation != placement_of.end() ? r.get(relation->second) : nullptr;
            const StepSubEntity* related = rel ? rel->find("REPRESENTATION_RELATIONSHIP") : nullptr;
            const StepSubEntity* with = rel ? rel->find("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") : nullptr;
            const StepEntity* transform = with ? r.get(first_ref(with->params)) : nullptr;
            const std::vector<int> sides = related ? all_refs(related->params) : std::vector<int>();
            const std::vector<int> axes = transform && transform->has("ITEM_DEFINED_TRANSFORMATION") ? all_refs(transform->parts.front().params) : std::vector<int>();

            if (sides.size() < 2 || axes.size() < 2)
                continue;

            const std::vector<int> mine = closure(child);
            const bool first_is_child = std::find(mine.begin(), mine.end(), sides[0]) != mine.end();
            const int from = first_is_child ? axes[0] : axes[1];
            const int to = first_is_child ? axes[1] : axes[0];
            visit(child, placement * frame(to).first * frame(from).second, depth + 1);
        }
    };

    for (const auto& [pd, shape] : shape_of)
        if (std::none_of(uses.begin(), uses.end(), [&](const std::array<int, 3>& use) { return use[2] == pd; }) &&
            (reps.count(shape) || std::any_of(uses.begin(), uses.end(), [&](const std::array<int, 3>& use) { return use[1] == pd; })))
            visit(pd, Xform::identity(), 0);

    return out.empty() ? unplaced : out;
}

std::vector<BRep> read_file_step_breps(
    const std::string& filepath,
    double tolerance,
    std::vector<StepIds>* ids,
    std::vector<BRep>* parts,
    std::vector<std::pair<int, Xform>>* instances
) {

    if (!std::isfinite(tolerance) || tolerance <= 0)
        throw std::invalid_argument("STEP BRep tolerance must be finite and positive");

    const StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<int> order;

    for (const std::pair<const int, StepEntity>& kv : sf.entities)
        order.push_back(kv.first);

    std::sort(order.begin(), order.end());

    std::map<int, std::vector<std::pair<BRep, StepIds>>> built;
    std::map<int, int> first_part; // Index in parts of a model's first BRep.
    int part_count = 0;
    std::vector<BRep> out;
    const auto record = [&](const BRep& part, int index, const Xform& placement) {
        if (parts && index == (int)parts->size())
            parts->push_back(part);

        if (instances)
            instances->push_back({index, placement});
    };

    for (const auto& [model, placement] : placed_models(r, order)) {
        if (!built.count(model)) {
            const StepSubEntity* root = model_of(r, model);
            std::vector<int> shell_refs = all_refs(root->params);

            for (int sh : list_refs(root->params))
                shell_refs.push_back(sh);

            for (int shell_ref : shell_refs) {
                const StepEntity* sh = r.get(shell_ref);
                const StepSubEntity* os = sh ? sh->find("ORIENTED_CLOSED_SHELL") : nullptr;
                const int inner = os ? first_ref(os->params) : -1;

                for (const std::vector<int>& faces : shell_parts(r, inner >= 0 ? inner : shell_ref)) {
                    BRepBuilder builder(r, tolerance);
                    StepIds part;
                    BRep b = builder.build_from_shell(inner >= 0 ? inner : shell_ref, faces, part);

                    if (!b.m_faces.empty())
                        built[model].push_back({std::move(b), std::move(part)});
                }
            }

            first_part[model] = part_count;
            part_count += (int)built[model].size();
        }

        for (size_t k = 0; k < built[model].size(); k++) {
            const auto& [b, part] = built[model][k];
            out.push_back(placement ? b.transformed(*placement) : b);
            record(b, first_part[model] + (int)k, placement.value_or(Xform::identity()));

            if (ids)
                ids->push_back(part);
        }
    }

    // A GEOMETRIC_CURVE_SET or GEOMETRIC_SET as a BRep without faces: its points as vertices, its B-spline curves as edges (as
    // STEPControl_ActorRead transfers it to a compound of vertices and edges).
    for (int id : order) {
        const StepEntity* e = r.get(id);
        const StepSubEntity* set = e->find("GEOMETRIC_CURVE_SET");

        if (!set)
            set = e->find("GEOMETRIC_SET");

        if (!set)
            continue;

        BRep b;
        b.name = "step_curves";

        for (int element : list_refs(set->params)) {
            const StepEntity* item = r.get(element);
            const NurbsCurve curve = item && !item->has("CARTESIAN_POINT") ? r.get_nurbs_curve(element) : NurbsCurve();

            if (item && item->has("CARTESIAN_POINT"))
                b.add_vertex(r.get_point(element));
            else if (curve.is_valid())
                b.add_edge(b.add_curve_3d(curve), b.add_vertex(curve.point_at_start()), b.add_vertex(curve.point_at_end()));
        }

        record(b, part_count++, Xform::identity());
        out.push_back(std::move(b));

        if (ids)
            ids->push_back({});
    }

    return out;
}

std::map<std::string, int> read_file_step_entity_counts(const std::string& filepath) {

    std::map<std::string, int> counts;

    for (const auto& [id, e] : parse_step_file(filepath).entities) {
        std::string type;

        for (const StepSubEntity& part : e.parts)
            type += (type.empty() ? "" : "+") + part.type;

        counts[type]++;
    }

    return counts;
}

bool write_file_step_nurbscurves(const std::vector<NurbsCurve>& curves, const std::string& filepath) {

    StepWriter w;

    for (const NurbsCurve& nc : curves)
        w.write_nurbs_curve(nc);

    return write_step_string(w.emit(), filepath);
}

bool write_file_step_nurbssurfaces(const std::vector<NurbsSurface>& surfaces, const std::string& filepath) {

    StepWriter w;

    for (const NurbsSurface& srf : surfaces)
        w.write_nurbs_surface(srf);

    return write_step_string(w.emit(), filepath);
}

bool write_file_step_nurbssurfaces_trimmed(
    const std::vector<NurbsSurfaceTrimmed>& trimmed,
    const std::string& filepath
) {

    StepWriter w;
    std::vector<int> face_ids;

    for (const NurbsSurfaceTrimmed& t : trimmed) {
        const int fid = w.write_trimmed_face(t);

        if (fid >= 0)
            face_ids.push_back(fid);
    }

    std::vector<int> bodies;
    const int body = w.write_body(face_ids, false);

    if (body >= 0)
        bodies.push_back(body);

    w.finish_product(bodies, false, "trimmed");
    return write_step_string(w.emit(), filepath);
}

bool write_file_step_brep(const BRep& brep, const std::string& filepath) {

    StepWriter w;
    std::vector<int> bodies;
    bool any_closed = false;

    for (const std::pair<std::vector<int>, bool>& group : emit_brep_shells(w, brep)) {
        const std::vector<int>& ids = group.first;
        const bool closed = group.second;
        const int body = w.write_body(ids, closed);

        if (body >= 0)
            bodies.push_back(body);

        any_closed = any_closed || closed;
    }

    w.finish_product(bodies, any_closed, brep.name.empty() ? "brep" : brep.name, vertex_diagonal(brep) * 1e-4);
    return write_step_string(w.emit(), filepath);
}

bool write_file_step_breps(
    const std::vector<const BRep*>& breps,
    const std::string& name,
    const std::string& filepath
) {

    StepWriter w;
    std::vector<int> bodies;
    std::vector<int> styled;
    bool any_closed = false;
    double diag = 1.0;

    for (const BRep* b : breps) {
        if (!b)
            continue;

        const std::vector<std::pair<std::vector<int>, bool>> groups = emit_brep_shells(w, *b);
        diag = std::max(diag, vertex_diagonal(*b));

        const int psa = w.color_style(b->surfacecolor.r, b->surfacecolor.g, b->surfacecolor.b);

        for (const std::pair<std::vector<int>, bool>& group : groups) {
            const std::vector<int>& ids = group.first;
            const bool closed = group.second;
            const int body = w.write_body(ids, closed);

            if (body >= 0)
                bodies.push_back(body);

            any_closed = any_closed || closed;

            for (int fid : ids)
                styled.push_back(
                    w.write_raw("STYLED_ITEM('',(#" + std::to_string(psa) + "),#" + std::to_string(fid) + ")")
                );
        }
    }

    w.finish_product(bodies, any_closed, name, diag * 1e-4, styled);
    return write_step_string(w.emit(), filepath);
}

bool write_file_step_assembly(
    const std::vector<BRep>& parts,
    const std::vector<std::pair<int, Xform>>& instances,
    const std::string& name,
    const std::string& filepath
) {

    if (std::any_of(instances.begin(), instances.end(), [&](const std::pair<int, Xform>& i) { return i.first < 0 || i.first >= (int)parts.size(); }))
        return false;

    StepWriter w;
    double diag = 1.0;

    for (const BRep& part : parts)
        diag = std::max(diag, vertex_diagonal(part));

    const int origin = w.write_axis(Xform::identity());
    const std::array<int, 4> c = w.write_context(diag * 1e-4);
    std::vector<std::pair<int, int>> made;

    for (const BRep& part : parts) {
        std::vector<int> bodies;
        bool closed = false;

        for (const std::pair<std::vector<int>, bool>& group : emit_brep_shells(w, part)) {
            const int body = w.write_body(group.first, group.second);

            if (body >= 0)
                bodies.push_back(body);

            closed = closed || group.second;
        }

        std::vector<int> items = {origin};
        items.insert(items.end(), bodies.begin(), bodies.end());
        made.push_back(w.write_product(part.name.empty() ? "part" : part.name, StepWriter::representation_type(bodies, closed), items, c));
    }

    std::vector<int> axes = {origin};

    for (const std::pair<int, Xform>& instance : instances)
        axes.push_back(w.write_axis(instance.second));

    const auto [root, assembly] = w.write_product(name, "SHAPE_REPRESENTATION", axes, c);

    // Each instance: the part's representation related to the assembly's by the transformation from the part's origin to the
    // instance's axis, in the context of the occurrence.
    for (size_t k = 0; k < instances.size(); k++) {
        const auto [definition, representation] = made[instances[k].first];
        const int use = w.write_raw("NEXT_ASSEMBLY_USAGE_OCCURRENCE('" + std::to_string(k + 1) + "','','',#" + std::to_string(root) + ",#" +
                                    std::to_string(definition) + ",$)");
        const int shape = w.write_raw("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(use) + ")");
        const int transform = w.write_raw("ITEM_DEFINED_TRANSFORMATION('','',#" + std::to_string(origin) + ",#" + std::to_string(axes[k + 1]) + ")");
        const int relation = w.write_raw("(REPRESENTATION_RELATIONSHIP('','',#" + std::to_string(representation) + ",#" + std::to_string(assembly) +
                                         ")REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#" + std::to_string(transform) +
                                         ")SHAPE_REPRESENTATION_RELATIONSHIP())");
        w.write_raw("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#" + std::to_string(relation) + ",#" + std::to_string(shape) + ")");
    }

    return write_step_string(w.emit(), filepath);
}

} // namespace file_step
} // namespace session_cpp
