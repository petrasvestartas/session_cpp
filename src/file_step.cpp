#include "file_step.h"
#include "closest.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include "fmt/core.h"
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
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
    Enum, // Enum literal or bare identifier.
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
            parse_params(c, depth + 1);
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

/// Fill sf from the DATA section of a STEP text.
static void parse_step_string(const std::string& content, StepFile& sf) {

    Cursor c{content, 0, content.size()};

    while (c.p < c.end) {
        skip_ws(c);

        if (c.p >= c.end)
            break;

        if (c.s[c.p] != '#') {
            while (c.p < c.end && c.s[c.p] != '\n')
                c.p++;

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

/// Remove /* */ comments from the raw text.
static std::string strip_comments(const std::string& raw) {

    std::string text;
    text.reserve(raw.size());

    size_t i = 0;

    while (i < raw.size()) {
        if (i + 1 < raw.size() && raw[i] == '/' && raw[i + 1] == '*') {
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

    const std::string text = strip_comments(buf.str());
    const size_t lo = text.find("DATA");

    if (lo == std::string::npos)
        return sf;

    const size_t semi = text.find(';', lo);

    if (semi == std::string::npos)
        return sf;

    const size_t endsec = text.find("ENDSEC", semi);

    if (endsec == std::string::npos)
        return sf;

    parse_step_string(text.substr(semi + 1, endsec - semi - 1), sf);

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

/// B_SPLINE_CURVE_WITH_KNOTS parameters, simple or split across a complex instance; none when missing, short or empty.
static std::optional<CurveParams> curve_params(const StepEntity& e) {

    const StepSubEntity* bsc = e.find("B_SPLINE_CURVE_WITH_KNOTS");

    if (!bsc)
        return std::nullopt;

    const StepSubEntity* base = e.find("B_SPLINE_CURVE");
    CurveParams cp;

    if (!base) {
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

/// B_SPLINE_SURFACE_WITH_KNOTS parameters, simple or split across a complex instance; none when missing, short or empty.
static std::optional<SurfaceParams> surface_params(const StepEntity& e) {

    const StepSubEntity* bss = e.find("B_SPLINE_SURFACE_WITH_KNOTS");

    if (!bss)
        return std::nullopt;

    const StepSubEntity* base = e.find("B_SPLINE_SURFACE");
    SurfaceParams sp;

    if (!base) {
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

/// Parameter projector of a surface: a plane or a cylinder on the quarter-arc chart.
struct Proj {
    int kind = 0; // 0 none, 1 plane, 2 cylinder.
    Axis2 a; // Surface frame.
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

    if (an.kind == 3) {
        const double ca = std::cos(an.r2);

        return {s, std::abs(ca) > 1e-12 ? z / ca : z, radial_ok};
    }

    if (an.kind == 4)
        return {s, std::atan2(z, rho), radial_ok};

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

/// Bilinear patch of the plane with axis a over [u0, u1] x [v0, v1].
static NurbsSurface plane_surface(const Axis2& a, double u0, double u1, double v0, double v1) {

    NurbsSurface out(3, false, 2, 2, 2, 2);
    out.m_nurbsknot[0] = {u0, u1};
    out.m_nurbsknot[1] = {v0, v1};

    const bool ok = out.set_cv(0, 0, axis_point(a, u0, v0, 0.0)) && out.set_cv(0, 1, axis_point(a, u0, v1, 0.0)) &&
        out.set_cv(1, 0, axis_point(a, u1, v0, 0.0)) && out.set_cv(1, 1, axis_point(a, u1, v1, 0.0));

    return ok ? out : NurbsSurface();
}

/// Rational cylinder patch on the quarter-arc chart (1 unit = 90 degrees) over [u0, u1] x [v0, v1]; a span of 4 closes it.
static NurbsSurface cylinder_surface(const Axis2& a, double radius, double u0, double u1, double v0, double v1) {

    const bool closed = std::abs((u1 - u0) - 4.0) < 0.2;
    const int n_spans = closed ? 4 : std::max(1, (int)std::ceil(std::abs(u1 - u0) - 1e-9));

    if (closed)
        u1 = u0 + 4.0;

    const int n_u = 2 * n_spans + 1;
    NurbsSurface out(3, true, 3, 2, n_u, 2);
    std::vector<double> knots = {u0, u0};

    for (int s = 1; s < n_spans; s++) {
        knots.push_back(u0 + s);
        knots.push_back(u0 + s);
    }

    knots.push_back(u1);
    knots.push_back(u1);
    out.m_nurbsknot[0] = knots;
    out.m_nurbsknot[1] = {v0, v1};

    std::vector<double> ca;
    std::vector<double> sa;
    std::vector<double> cw;
    std::tie(ca, sa, cw) = arc_nodes(u0, n_spans);

    for (int i = 0; i < n_u; i++)
        for (int j = 0; j < 2; j++) {
            const Point p = axis_point(a, radius * ca[i], radius * sa[i], j == 0 ? v0 : v1);

            if (!out.set_cv_4d(i, j, cw[i] * p[0], cw[i] * p[1], cw[i] * p[2], cw[i]))
                return NurbsSurface();
        }

    return out;
}

/// Parameter-space image of a 3D point: plane coordinates, or cylinder (angle in quarter turns, height).
static std::pair<double, double> project(const Proj& pr, const Point& pt) {

    const Vector d = pt - pr.a.origin;

    if (pr.kind == 1)
        return {d.dot(pr.a.ax), d.dot(pr.a.ay)};

    return {std::atan2(d.dot(pr.a.ay), d.dot(pr.a.ax)) * 2.0 / PI, d.dot(pr.a.az)};
}

/// Affine projector of a bilinear patch from its corner p00; kind 0 when the patch is not bilinear or degenerate.
static Proj bilinear_projector(const NurbsSurface& srf) {

    Proj pr;

    if (!srf.is_valid() || srf.degree(0) != 1 || srf.degree(1) != 1)
        return pr;

    const Point p00 = srf.get_cv(0, 0);
    const Vector eu = srf.get_cv(1, 0) - p00;
    const Vector ev = srf.get_cv(0, 1) - p00;
    const double eu2 = eu.dot(eu);
    const double ev2 = ev.dot(ev);

    if (eu2 <= 1e-28 || ev2 <= 1e-28)
        return pr;

    pr.kind = 1;
    pr.a.origin = p00;
    pr.a.ax = eu * (1.0 / eu2);
    pr.a.ay = ev * (1.0 / ev2);
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

/// Exact rational arc on the circle (axis a, radius rad) from vs to ve, the full circle when they coincide.
static NurbsCurve circle_nurbs(const Axis2& a, double rad, const Point& vs, const Point& ve) {

    const double sa = angle_of(a, vs);
    double ea = angle_of(a, ve);

    if (vs.distance(ve) < 1e-10)
        ea = sa + 2.0 * PI;
    else if (ea <= sa)
        ea += 2.0 * PI;

    const double span = ea - sa;
    const int ns = std::max(1, (int)std::ceil(std::abs(span) / PI_2));
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
        const double r2 = mid ? rad / wm : rad;
        const Point p = a.origin + (a.ax * std::cos(ang) + a.ay * std::sin(ang)) * r2;

        if (!crv.set_cv_4d(i, w * p[0], w * p[1], w * p[2], w))
            return NurbsCurve();
    }

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

/// Keep consecutive cylinder samples on one branch of the quarter-arc chart (period 4).
static void unwrap_seam(std::vector<Point>& uv) {

    for (size_t k = 1; k < uv.size(); k++) {
        const double du = uv[k][0] - uv[k - 1][0];

        if (du > 2.0)
            uv[k][0] -= 4.0;
        else if (du < -2.0)
            uv[k][0] += 4.0;
    }
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
            grid.push_back(srf.point_at(u0 + (u1 - u0) * i / (ns - 1), v0 + (v1 - v0) * j / (ns - 1)));

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
        else
            a.ax = std::abs(a.az[0]) < 0.9 ? Vector(1, 0, 0) : Vector(0, 1, 0);

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

    /// The 3D basis curve behind a SURFACE_CURVE or SEAM_CURVE, the id itself otherwise.
    int basis_curve_of(int curve_id) const {

        const StepEntity* e = get(curve_id);
        const StepSubEntity* sc = e ? e->find("SURFACE_CURVE") : nullptr;

        if (e && !sc)
            sc = e->find("SEAM_CURVE");

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
        const StepSubEntity* cyl = e ? e->find("CYLINDRICAL_SURFACE") : nullptr;
        const StepSubEntity* sub = plane ? plane : cyl;
        const int ref = sub ? first_ref(sub->params) : -1;

        if (ref < 0)
            return pr;

        pr.a = get_axis2(ref);
        pr.kind = plane ? 1 : 2;

        return pr;
    }

    /// Kernel surface of a surface entity: the B-spline itself, or a plane or cylinder patch over the padded uv window.
    NurbsSurface fill_surface(int id, double u0, double u1, double v0, double v1) {

        const StepEntity* e = get(id);

        if (!e)
            return NurbsSurface();

        if (e->has("B_SPLINE_SURFACE_WITH_KNOTS"))
            return get_nurbs_surface(id);

        const StepSubEntity* plane = e->find("PLANE");
        const StepSubEntity* cyl = e->find("CYLINDRICAL_SURFACE");
        const StepSubEntity* sub = plane ? plane : cyl;
        const int ax_ref = sub ? first_ref(sub->params) : -1;
        const Axis2 a = get_axis2(ax_ref);

        if (ax_ref < 0 || !a.ok)
            return NurbsSurface();

        const double pad_v = std::max(1e-6, 0.01 * (v1 - v0));

        if (plane) {
            const double pad_u = std::max(1e-6, 0.01 * (u1 - u0));

            return plane_surface(a, u0 - pad_u, u1 + pad_u, v0 - pad_v, v1 + pad_v);
        }

        const std::vector<double> rr = nums(sub->params);

        return cylinder_surface(a, rr.empty() ? 1.0 : rr[0], u0, u1, v0 - pad_v, v1 + pad_v);
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
            an.r2 = rr.size() > 1 ? rr[1] : 0.0;

            return an;
        }

        return an;
    }

    /// Canonical (s, t) samples of the pcurve an edge carries on a surface; a SEAM_CURVE holds two, the second for the reversed use.
    std::vector<Point> pcurve_st_samples(int ec_geom_id, int surface_ref, bool forward_use, int n) {

        const StepEntity* e = get(ec_geom_id);
        const StepSubEntity* sc = e ? e->find("SURFACE_CURVE") : nullptr;
        const bool is_seam = e && !sc && e->has("SEAM_CURVE");

        if (is_seam)
            sc = e->find("SEAM_CURVE");

        if (!sc)
            return {};

        std::vector<int> mine;

        for (int pid : list_refs(sc->params)) {
            const StepEntity* pe = get(pid);
            const StepSubEntity* pc = pe ? pe->find("PCURVE") : nullptr;
            const std::vector<int> refs = pc ? all_refs(pc->params) : std::vector<int>();

            if (refs.size() >= 2 && refs[0] == surface_ref)
                mine.push_back(refs[1]);
        }

        if (mine.empty())
            return {};

        const int pick = is_seam && mine.size() > 1 && !forward_use ? 1 : 0;
        const StepEntity* dr = get(mine[pick]);
        const StepSubEntity* drs = dr ? dr->find("DEFINITIONAL_REPRESENTATION") : nullptr;
        const std::vector<int> c2_refs = drs ? list_refs(drs->params) : std::vector<int>();

        if (c2_refs.empty())
            return {};

        const NurbsCurve c2 = get_nurbs_curve(c2_refs[0]);

        return c2.is_valid() ? sample_nurbs(c2, n) : std::vector<Point>();
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
};

/// One edge use with its parameter-space samples in curve order; pc2d is an exact pcurve when exact.
struct LoopEdge {
    int edge_idx = -1; // Brep edge index.
    bool reversed = false; // Whether the edge runs against the loop.
    std::vector<Point> uv; // Sampled uv points.
    NurbsCurve pc2d; // Parameter-space curve.
    bool exact = false; // Whether pc2d is exact rather than sampled.
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

/// Periods of the parameter chart: 4 in u for the analytic cylinder, the domain span of each closed direction of a B-spline surface, 0 when open.
static std::pair<double, double> surface_periods(const Proj& proj, const NurbsSurface& proj_srf) {

    if (proj.kind == 2)
        return {4.0, 0.0};

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
        Point prev_end(0, 0, 0);
        bool have_prev = false;

        for (LoopEdge& le : lp.edges) {
            if (le.uv.empty())
                continue;

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

/// Shift each inner loop by whole u periods onto the outer loop's u window.
static void center_inner_loops(std::vector<Loop>& loops, double tau_u) {

    if (tau_u <= 0.0)
        return;

    std::optional<double> outer;

    for (const Loop& lp : loops)
        if (lp.is_outer) {
            outer = loop_ucenter(lp);
            break;
        }

    if (!outer)
        return;

    for (Loop& lp : loops) {
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

        if (!le.exact || (le.reversed && !crv2d.reverse())) {
            std::vector<Point> uv = le.uv;

            if (le.reversed)
                std::reverse(uv.begin(), uv.end());

            crv2d = polyline_nurbs(uv, 2);
        }

        pl.push_back({le.edge_idx, le.reversed, crv2d});
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

/// Chart polyline of every edge of a loop, shifted by whole periods so consecutive edges of a projected loop meet.
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
        if (chains[k].empty() || chains[k - 1].empty())
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

/// BRep of one STEP shell, built face by face.
class BRepBuilder {
    StepReader& r; // Entity reader.
    BRep brep; // Brep under construction.
    std::unordered_map<int, int> vmap; // Brep vertex by VERTEX_POINT id.
    std::unordered_map<int, int> emap; // Brep edge by EDGE_CURVE id.
    std::vector<BRepRef> face_refs; // Face references in file order.

public:

    /// Construct over an entity reader.
    explicit BRepBuilder(StepReader& reader) : r(reader) {}

    /// Existing vertex within tol of q, else a new one.
    int vertex_at(const Point& q, double tol) {

        for (size_t i = 0; i < brep.m_vertices.size(); i++)
            if (brep.m_vertices[i].point.distance(q) <= tol)
                return (int)i;

        return brep.add_vertex(q);
    }

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

    /// Face from its surface and loops (outer first), oriented in the shell by reversed_face.
    void finish_face(int si, bool reversed_face, const std::vector<std::vector<PendingEdge>>& loops) {

        std::vector<BRepRef> wires;

        for (const std::vector<PendingEdge>& lp : loops) {
            std::vector<BRepRef> refs;

            for (const PendingEdge& pe : lp) {
                NurbsCurve c = pe.c2d;

                if (!pe.reversed || c.reverse())
                    attach_pcurve(pe.edge, si, brep.add_curve_2d(c), pe.reversed);

                refs.push_back({pe.edge, pe.reversed ? BRepOrientation::Reversed : BRepOrientation::Forward});
            }

            if (!refs.empty())
                wires.push_back({brep.add_wire(refs), BRepOrientation::Forward});
        }

        const int fi = brep.add_face(si, wires);
        face_refs.push_back({fi, reversed_face ? BRepOrientation::Reversed : BRepOrientation::Forward});
    }

    /// Return the brep vertex of a VERTEX_POINT, creating it once.
    int get_vertex(int vp_id) {

        const auto it = vmap.find(vp_id);

        if (it != vmap.end())
            return it->second;

        return vmap[vp_id] = brep.add_vertex(r.get_vertex_point(vp_id).value_or(Point(0, 0, 0)));
    }

    /// Exact 3D curve of an edge basis: the B-spline itself or a rational arc of a CIRCLE, invalid otherwise.
    NurbsCurve edge_curve(int curve_id, const Point& vs, const Point& ve) {

        const StepEntity* e = r.get(curve_id);

        if (!e)
            return NurbsCurve();

        if (e->has("B_SPLINE_CURVE_WITH_KNOTS"))
            return r.get_nurbs_curve(curve_id);

        const StepSubEntity* circle = e->find("CIRCLE");

        if (!circle)
            return NurbsCurve();

        const int ax_ref = first_ref(circle->params);
        const std::vector<double> rr = nums(circle->params);
        const double rad = rr.empty() ? 0.0 : rr[0];
        const Axis2 a = r.get_axis2(ax_ref);

        if (ax_ref < 0 || rad <= 0 || !a.ok)
            return NurbsCurve();

        return circle_nurbs(a, rad, vs, ve);
    }

    /// BRep edge of an EDGE_CURVE, made once: exact curve when possible, else a sampled polyline.
    int get_edge(int ec_id) {

        const auto it = emap.find(ec_id);

        if (it != emap.end())
            return it->second;

        const std::vector<int> refs = edge_refs(r, ec_id);

        if (refs.size() < 3)
            return -1;

        const int sv = get_vertex(refs[0]);
        const int ev = get_vertex(refs[1]);
        const int curve_id = r.basis_curve_of(refs[2]);
        const Point vs = brep.m_vertices[sv].point;
        const Point ve = brep.m_vertices[ev].point;
        NurbsCurve crv3d = edge_curve(curve_id, vs, ve);

        if (!crv3d.is_valid())
            crv3d = polyline_nurbs(r.sample_curve(curve_id, vs, ve, 16), 3);

        return emap[ec_id] = brep.add_edge(brep.add_curve_3d(crv3d), sv, ev);
    }

    /// Projection fallback: 3D samples of an edge mapped to canonical (s, t), branch-unwrapped along the loop traversal.
    std::vector<Point> st_projected(const AnFace& an, int geom_id, int edge_idx, bool rev, const Loop& lp) {

        const BRepEdge& be = brep.m_edges[edge_idx];
        std::vector<Point> ordered =
            r.sample_curve(geom_id, brep.m_vertices[be.start_vertex].point, brep.m_vertices[be.end_vertex].point, 48);

        if (ordered.size() < 2)
            return {};

        if (rev)
            std::reverse(ordered.begin(), ordered.end());

        double ps = 0.0;
        double pt = 0.0;
        bool have_prev = false;
        std::tie(ps, pt, have_prev) = st_start(an, lp, ordered);

        std::vector<Point> st = st_unwrapped(an, ordered, ps, pt, have_prev);

        if (rev)
            std::reverse(st.begin(), st.end());

        return st;
    }

    /// Loops of an analytic face with canonical (s, t) samples from the file pcurves or from projection.
    bool analytic_loops(
        const std::vector<int>& bound_refs,
        int surface_ref,
        const AnFace& an,
        std::vector<Loop>& loops
    ) {

        for (int bid : bound_refs) {
            const std::optional<Bound> b = bound_loop(r, bid);

            if (!b)
                continue;

            Loop lp;
            lp.is_outer = b->is_outer;

            for (int oe_id : b->oe_refs) {
                int ec_ref = -1;
                bool oe_orient = true;
                std::tie(ec_ref, oe_orient) = oriented_edge(r, oe_id);

                const int edge_idx = ec_ref < 0 ? -1 : get_edge(ec_ref);

                if (edge_idx < 0)
                    continue;

                const int geom_id = edge_geom_id(r, ec_ref);
                LoopEdge le;
                le.edge_idx = edge_idx;
                le.reversed = oe_orient != b->orient;

                if (geom_id >= 0)
                    le.uv = r.pcurve_st_samples(geom_id, surface_ref, oe_orient, 48);

                if (le.uv.size() < 2) {
                    lp.projected = true;
                    le.uv = st_projected(an, geom_id, edge_idx, le.reversed, lp);

                    if (le.uv.empty())
                        return false;
                }

                lp.edges.push_back(le);
            }

            if (!lp.edges.empty())
                loops.push_back(lp);

            pick_outer_loop(loops);
        }

        return !loops.empty();
    }

    /// Pending edges of one loop in the chart, plus a degenerated edge across each pole or apex gap between consecutive edges.
    std::vector<PendingEdge> analytic_pending(const Loop& lp, const AnFace& an, const Window& w, double scale3) {

        const std::vector<std::vector<Point>> chains = analytic_chains(lp, an, w);
        std::vector<PendingEdge> pl;

        for (size_t k = 0; k < lp.edges.size(); k++) {
            if (chains[k].size() < 2)
                continue;

            pl.push_back({lp.edges[k].edge_idx, lp.edges[k].reversed, polyline_nurbs(chains[k], 2)});

            const std::vector<Point>& nxt = chains[(k + 1) % chains.size()];

            if (nxt.empty())
                continue;

            const Point a2 = chains[k].back();
            const Point b2 = nxt.front();

            if (std::abs(a2[0] - b2[0]) + std::abs(a2[1] - b2[1]) <= 1e-7)
                continue;

            const Point p3a = chart_eval(an, w, a2);
            const Point p3b = chart_eval(an, w, b2);

            if (p3a.distance(p3b) >= scale3 * 1e-6)
                continue;

            const int vd = vertex_at(p3a, scale3 * 1e-6);
            pl.push_back({brep.add_edge(-1, vd, vd), false, polyline_nurbs({a2, b2}, 2)});
        }

        return pl;
    }

    /// Face on a cylinder, cone, sphere or torus: the exact kernel window with the file pcurves bound in it; false falls back to projection.
    bool add_face_analytic(const std::vector<int>& bound_refs, int surface_ref, bool same_sense, const AnFace& an) {

        std::vector<Loop> loops;

        if (!analytic_loops(bound_refs, surface_ref, an, loops))
            return false;

        const std::optional<Window> w = analytic_window(loops, an);

        if (!w)
            return false;

        const NurbsSurface srf = build_analytic_nurbs(an, w->su0, w->nsu, w->t0, w->t1, w->sv0, w->nsv);

        if (!srf.is_valid())
            return false;

        const double scale3 = an.radius + std::abs(an.r2) + 1.0;
        const int srf_idx = brep.add_surface(srf);
        outer_first(loops);

        std::vector<std::vector<PendingEdge>> pending;

        for (const Loop& lp : loops)
            pending.push_back(analytic_pending(lp, an, *w, scale3));

        finish_face(srf_idx, !same_sense, pending);

        return true;
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

    /// Loops of a face on a projected surface: uv samples in curve order, exact pcurves under an affine projector.
    void projected_loops(
        const std::vector<int>& bound_refs,
        const Proj& proj,
        const NurbsSurface& proj_srf,
        std::vector<Loop>& loops
    ) {

        const int n = proj_srf.is_valid() ? 48 : 16;
        Proj exact;

        if (proj.kind == 1)
            exact = proj;
        else if (proj.kind == 0)
            exact = bilinear_projector(proj_srf);

        for (int bid : bound_refs) {
            const std::optional<Bound> b = bound_loop(r, bid);

            if (!b)
                continue;

            Loop lp;
            lp.is_outer = b->is_outer;

            for (int oe_id : b->oe_refs) {
                int ec_ref = -1;
                bool oe_orient = true;
                std::tie(ec_ref, oe_orient) = oriented_edge(r, oe_id);

                const int edge_idx = ec_ref < 0 ? -1 : get_edge(ec_ref);

                if (edge_idx < 0)
                    continue;

                const BRepEdge& be = brep.m_edges[edge_idx];
                const Point vs = brep.m_vertices[be.start_vertex].point;
                const Point ve = brep.m_vertices[be.end_vertex].point;
                LoopEdge le;
                le.edge_idx = edge_idx;
                le.reversed = oe_orient != b->orient;
                le.uv = uv_of_samples(proj, proj_srf, r.sample_curve(edge_geom_id(r, ec_ref), vs, ve, n));

                if (proj.kind == 2)
                    unwrap_seam(le.uv);

                if (be.curve_3d_index >= 0)
                    le.pc2d = exact_pcurve(exact, brep.m_curves_3d[be.curve_3d_index]);

                le.exact = le.pc2d.is_valid();
                lp.edges.push_back(le);
            }

            if (!lp.edges.empty())
                loops.push_back(lp);

            pick_outer_loop(loops);
        }
    }

    /// Face projected onto its plane, cylinder chart or B-spline surface, with a filled surface when the projection has none.
    void add_face_projected(const std::vector<int>& bound_refs, int surface_ref, bool same_sense) {

        const Proj proj = r.get_projector(surface_ref);
        const NurbsSurface proj_srf = proj.kind == 0 ? r.fill_surface(surface_ref, 0, 1, 0, 1) : NurbsSurface();
        std::vector<Loop> loops;
        projected_loops(bound_refs, proj, proj_srf, loops);

        double tau_u = 0.0;
        double tau_v = 0.0;
        std::tie(tau_u, tau_v) = surface_periods(proj, proj_srf);
        chain_loops(loops, tau_u, tau_v);
        center_inner_loops(loops, tau_u);

        NurbsSurface srf = proj_srf;

        if (!srf.is_valid()) {
            double umin = 0.0;
            double umax = 0.0;
            double vmin = 0.0;
            double vmax = 0.0;
            std::tie(umin, umax, vmin, vmax) = loops_bounds(loops);

            if (umin > umax) {
                umin = -1.0;
                umax = 1.0;
                vmin = -1.0;
                vmax = 1.0;
            }

            srf = r.fill_surface(surface_ref, umin, umax, vmin, vmax);
        }

        const int srf_idx = brep.add_surface(srf);
        outer_first(loops);

        std::vector<std::vector<PendingEdge>> pending;

        for (const Loop& lp : loops)
            pending.push_back(pending_of(lp));

        finish_face(srf_idx, !same_sense, pending);
    }

    /// ADVANCED_FACE: vertex-loop face, analytic face, or projection onto the plane, cylinder chart or B-spline surface.
    void add_face(int face_id) {

        const StepEntity* fent = r.get(face_id);
        const StepSubEntity* face = fent ? fent->find("ADVANCED_FACE") : nullptr;

        if (!face)
            return;

        const std::vector<int> bound_refs = list_refs(face->params);
        const int surface_ref = first_ref(face->params);
        const bool same_sense = last_flag(face->params, true);
        const std::vector<int> vl_ids = vertex_loop_ids(bound_refs);

        if (!vl_ids.empty() && add_face_vertex_loop(vl_ids, surface_ref, same_sense))
            return;

        const AnFace an = r.get_analytic_srf(surface_ref);

        if (an.kind >= 2 && add_face_analytic(bound_refs, surface_ref, same_sense, an))
            return;

        add_face_projected(bound_refs, surface_ref, same_sense);
    }

    /// BRep of a CLOSED_SHELL (one solid) or OPEN_SHELL (one shell), empty for anything else.
    BRep build_from_shell(int shell_id) {

        const StepEntity* sent = r.get(shell_id);
        const StepSubEntity* shell = sent ? sent->find("CLOSED_SHELL") : nullptr;

        if (sent && !shell)
            shell = sent->find("OPEN_SHELL");

        if (!shell)
            return BRep();

        brep.name = "step_brep";

        for (int f : list_refs(shell->params))
            add_face(f);

        if (!face_refs.empty()) {
            const int sh = brep.add_shell(face_refs);

            if (sent->has("CLOSED_SHELL"))
                brep.add_solid({{sh, BRepOrientation::Forward}});
        }

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
        s = fmt::format("{:.15g}", v);

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

    /// B_SPLINE_CURVE_WITH_KNOTS, as a complete complex instance when rational; -1 for an invalid curve.
    int write_nurbs_curve(const NurbsCurve& nc) {

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

            pt_ids.push_back(write_point(x / w, y / w, z / w));
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

    /// AP214 PRODUCT and SHAPE_DEFINITION_REPRESENTATION skeleton importers need to find the bodies; uncertainty is the sewing tolerance.
    void finish_product(
        const std::vector<int>& bodies,
        bool closed,
        const std::string& name,
        double uncertainty = 1e-6,
        const std::vector<int>& styled_items = {}
    ) {

        const int o = write_point(0, 0, 0);
        const int dz = write_raw("DIRECTION('',(0.,0.,1.))");
        const int dx = write_raw("DIRECTION('',(1.,0.,0.))");
        const int ax = write_raw(
            "AXIS2_PLACEMENT_3D('',#" + std::to_string(o) + ",#" + std::to_string(dz) + ",#" + std::to_string(dx) + ")"
        );

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
        const int pr = write_raw("PRODUCT('" + name + "','" + name + "','',(#" + std::to_string(pc) + "))");
        const int pf = write_raw("PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(pr) + ")");
        const int dc = write_raw("PRODUCT_DEFINITION_CONTEXT('part definition',#" + std::to_string(ac) + ",'design')");
        const int pd =
            write_raw("PRODUCT_DEFINITION('design','',#" + std::to_string(pf) + ",#" + std::to_string(dc) + ")");

        const int ps = write_raw("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(pd) + ")");
        std::string rep_type = "MANIFOLD_SURFACE_SHAPE_REPRESENTATION";

        if (bodies.empty())
            rep_type = "SHAPE_REPRESENTATION";
        else if (closed)
            rep_type = "ADVANCED_BREP_SHAPE_REPRESENTATION";

        std::vector<int> items = {ax};
        items.insert(items.end(), bodies.begin(), bodies.end());

        const int rp = write_raw(rep_type + "('" + name + "'," + fmt_ref_list(items) + ",#" + std::to_string(gc) + ")");
        write_raw("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ps) + ",#" + std::to_string(rp) + ")");

        if (!styled_items.empty())
            write_raw(
                "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION(''," + fmt_ref_list(styled_items) + ",#" +
                std::to_string(gc) + ")"
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
        const int c = w.write_nurbs_curve(brep.m_curves_3d[e.curve_3d_index]);

        if (c < 0)
            return eid[ei] = -1;

        const int sv = vertex_id(e.start_vertex);
        const int ev = vertex_id(e.end_vertex);

        return eid[ei] = w.write_raw(
                   "EDGE_CURVE('',#" + std::to_string(sv) + ",#" + std::to_string(ev) + ",#" + std::to_string(c) +
                   ",.T.)"
               );
    }

    /// Return the surface id of a brep surface, emitting it once.
    int surface_id(int si) {

        const auto it = sid.find(si);

        if (it != sid.end())
            return it->second;

        return sid[si] = w.write_nurbs_surface(brep.m_surfaces[si]);
    }

    /// EDGE_LOOP of the non-degenerated edges, a VERTEX_LOOP when there are none, -1 for an empty wire.
    int wire_id(const BRepRef& wire) {

        std::vector<int> oes;
        int any_vertex = -1;

        for (const BRepRef& er : brep.wire_edges(wire)) {
            const BRepEdge& e = brep.m_edges[er.index];

            if (any_vertex < 0)
                any_vertex = e.start_vertex;

            if (e.degenerated)
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

    /// Return the ADVANCED_FACE id of a brep face, emitting it once.
    int face_id(int fi, BRepOrientation fo) {

        const BRepFace& f = brep.m_faces[fi];
        const int srf = surface_id(f.surface_index);

        if (srf < 0)
            return -1;

        std::vector<int> bounds;

        for (size_t wi = 0; wi < f.wires.size(); wi++) {
            const int loop = wire_id(f.wires[wi]);

            if (loop < 0)
                continue;

            bounds.push_back(w.write_raw(
                std::string(wi == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "('',#" + std::to_string(loop) + ",.T.)"
            ));
        }

        if (bounds.empty())
            return -1;

        const std::string sense = fo == BRepOrientation::Forward ? "T" : "F";

        return w.write_raw(
            "ADVANCED_FACE(''," + fmt_ref_list(bounds) + ",#" + std::to_string(srf) + ",." + sense + ".)"
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

std::vector<BRep> read_file_step_breps(const std::string& filepath) {

    const StepFile sf = parse_step_file(filepath);
    StepReader r(sf);
    std::vector<int> ids;

    for (const std::pair<const int, StepEntity>& kv : sf.entities)
        ids.push_back(kv.first);

    std::sort(ids.begin(), ids.end());

    std::vector<int> shell_refs;

    for (int id : ids) {
        const StepEntity* e = r.get(id);
        const StepSubEntity* root = e->find("MANIFOLD_SOLID_BREP");

        if (!root)
            root = e->find("BREP_WITH_VOIDS");

        if (!root)
            root = e->find("SHELL_BASED_SURFACE_MODEL");

        if (!root)
            continue;

        for (int sh : all_refs(root->params))
            shell_refs.push_back(sh);

        for (int sh : list_refs(root->params))
            shell_refs.push_back(sh);
    }

    std::vector<BRep> out;

    for (int shell_ref : shell_refs) {
        const StepEntity* sh = r.get(shell_ref);
        const StepSubEntity* os = sh ? sh->find("ORIENTED_CLOSED_SHELL") : nullptr;
        const int inner = os ? first_ref(os->params) : -1;
        BRepBuilder builder(r);
        BRep b = builder.build_from_shell(inner >= 0 ? inner : shell_ref);

        if (!b.m_faces.empty())
            out.push_back(std::move(b));
    }

    return out;
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

} // namespace file_step
} // namespace session_cpp
