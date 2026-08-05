#include "v3_geom.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using session_cpp::Point;
using session_cpp::Vector;
using session_cpp::NurbsCurve;
using session_cpp::NurbsSurface;

namespace v3 {

V3 to_v3(const Point& p) { return {p[0], p[1], p[2]}; }
V3 to_v3(const Vector& v) { return {v[0], v[1], v[2]}; }
Point to_point(const V3& p) { return Point(p.x, p.y, p.z); }

double wrap_2pi(double a) {
    double r = std::fmod(a, TWO_PI);
    if (r < 0) r += TWO_PI;
    return r;
}
double unwrap_near(double a, double ref) {
    double r = wrap_2pi(a);
    double d = r - wrap_2pi(ref);
    if (d > PI) d -= TWO_PI;
    if (d < -PI) d += TWO_PI;
    return ref + d;
}

Frame Frame::from_z(const V3& origin, const V3& zaxis) {
    Frame fr;
    fr.o = origin;
    fr.z = zaxis.normalized();
    V3 a = std::abs(fr.z.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
    fr.x = a.cross(fr.z).normalized();
    fr.y = fr.z.cross(fr.x);
    return fr;
}

// ============================================================================
// Cur
// ============================================================================

Cur cur_line(const V3& p, const V3& dir) {
    Cur c; c.k = Cur::LINE; c.f.o = p; c.f.x = dir.normalized(); return c;
}
Cur cur_circle(const Frame& fr, double radius) {
    Cur c; c.k = Cur::CIRCLE; c.f = fr; c.r = radius; return c;
}
Cur cur_circle(const V3& center, const V3& normal, double radius) {
    return cur_circle(Frame::from_z(center, normal), radius);
}
Cur cur_ellipse(const Frame& fr, double major_r, double minor_r) {
    Cur c; c.k = Cur::ELLIPSE; c.f = fr; c.r = major_r; c.ry = minor_r; return c;
}
Cur cur_nurbs(const NurbsCurve& nc) {
    Cur c; c.k = Cur::NURBS; c.nrb = nc; return c;
}
Cur cur_poly(const std::vector<V3>& pts, bool closed) {
    Cur c; c.k = Cur::POLY; c.poly = pts; c.poly_closed = closed; return c;
}

V3 Cur::eval(double t) const {
    switch (k) {
    case LINE:    return f.o + f.x * t;
    case CIRCLE:  return f.to_world(r * std::cos(t), r * std::sin(t), 0);
    case ELLIPSE: return f.to_world(r * std::cos(t), ry * std::sin(t), 0);
    case NURBS:   return to_v3(nrb.point_at(t));
    case POLY: {
        int n = (int)poly.size();
        if (n == 0) return {};
        if (n == 1) return poly[0];
        double tt = t; // index space
        if (poly_closed) {
            tt = std::fmod(tt, n);
            if (tt < 0) tt += n;
        } else {
            tt = std::max(0.0, std::min((double)n - 1.0, tt));
        }
        int i = (int)std::floor(tt);
        if (i >= n - 1) {
            if (poly_closed) return poly[n - 1] + (poly[0] - poly[n - 1]) * (tt - (n - 1));
            return poly[n - 1];
        }
        double fr = tt - i;
        return poly[i] + (poly[i + 1] - poly[i]) * fr;
    }
    }
    return {};
}
V3 Cur::d1(double t) const {
    switch (k) {
    case LINE:    return f.x;
    case CIRCLE:  return f.to_world(-r * std::sin(t), r * std::cos(t), 0) - f.o;
    case ELLIPSE: return f.to_world(-r * std::sin(t), ry * std::cos(t), 0) - f.o;
    case NURBS: {
        auto d = nrb.evaluate(t, 1);
        return d.size() > 1 ? to_v3(d[1]) : V3{};
    }
    case POLY: {
        int n = (int)poly.size();
        if (n < 2) return {};
        double tt = std::max(0.0, std::min((double)n - 1.0 - 1e-9, t));
        int i = (int)std::floor(tt);
        return poly[i + 1] - poly[i]; // per unit index
    }
    }
    return {};
}
V3 Cur::d2(double t) const {
    switch (k) {
    case LINE:    return {0, 0, 0};
    case CIRCLE:  return f.to_world(-r * std::cos(t), -r * std::sin(t), 0) - f.o;
    case ELLIPSE: return f.to_world(-r * std::cos(t), -ry * std::sin(t), 0) - f.o;
    case NURBS: {
        auto d = nrb.evaluate(t, 2);
        return d.size() > 2 ? to_v3(d[2]) : V3{};
    }
    case POLY: return {0, 0, 0};
    }
    return {};
}

bool Cur::project(const V3& p, double& t) const {
    switch (k) {
    case LINE: t = (p - f.o).dot(f.x); return true;
    case CIRCLE: {
        V3 w = p - f.o;
        t = std::atan2(w.dot(f.y), w.dot(f.x));
        return true;
    }
    case POLY: {
        int n = (int)poly.size();
        if (n == 0) return false;
        double best = 1e300;
        int bi = 0;
        double bf = 0;
        int seg_n = poly_closed ? n : n - 1;
        for (int i = 0; i < seg_n; i++) {
            V3 a = poly[i], b = poly[(i + 1) % n];
            V3 ab = b - a;
            double L2 = ab.norm2();
            double fr = L2 > 1e-300 ? std::max(0.0, std::min(1.0, (p - a).dot(ab) / L2)) : 0.0;
            double d = (a + ab * fr - p).norm2();
            if (d < best) { best = d; bi = i; bf = fr; }
        }
        t = bi + bf;
        return true;
    }
    case ELLIPSE:
    case NURBS: {
        // Newton on g(t) = (C(t)-p)·C'(t) = 0, seeded by coarse sampling.
        double t0 = 0, t1 = 0;
        if (k == ELLIPSE) {
            V3 w = p - f.o;
            t0 = std::atan2(w.dot(f.y), w.dot(f.x)) - 1.0;
            t1 = t0 + 2.0;
        } else {
            auto dom = nrb.domain();
            t0 = dom.first; t1 = dom.second;
        }
        double best_t = t0, best_d = 1e300;
        for (int i = 0; i <= 64; i++) {
            double ti = t0 + (t1 - t0) * i / 64.0;
            double d = (eval(ti) - p).norm2();
            if (d < best_d) { best_d = d; best_t = ti; }
        }
        t = best_t;
        for (int it = 0; it < 30; it++) {
            V3 c = eval(t), c1 = d1(t), c2 = d2(t);
            double g = (c - p).dot(c1);
            double gp = c1.norm2() + (c - p).dot(c2);
            if (std::abs(gp) < 1e-300) break;
            double dt = g / gp;
            t -= dt;
            if (std::abs(dt) < 1e-13 * (1.0 + std::abs(t))) break;
        }
        return true;
    }
    }
    return false;
}

// ============================================================================
// Srf
// ============================================================================

Srf srf_plane(const V3& origin, const V3& normal) {
    Srf s; s.k = Srf::PLANE; s.f = Frame::from_z(origin, normal); return s;
}
Srf srf_plane(const V3& origin, const V3& normal, const V3& xdir) {
    Srf s; s.k = Srf::PLANE;
    s.f.o = origin; s.f.z = normal.normalized();
    V3 x = xdir - s.f.z * xdir.dot(s.f.z);
    s.f.x = x.normalized(); s.f.y = s.f.z.cross(s.f.x);
    return s;
}
Srf srf_cylinder(const V3& axis_pt, const V3& axis_dir, double radius) {
    Srf s; s.k = Srf::CYLINDER; s.f = Frame::from_z(axis_pt, axis_dir); s.r = radius; return s;
}
Srf srf_cone(const V3& apex, const V3& axis_dir, double semi_angle, double ref_radius) {
    Srf s; s.k = Srf::CONE; s.f = Frame::from_z(apex, axis_dir);
    s.r = ref_radius; s.r2 = semi_angle; return s;
}
Srf srf_sphere(const V3& center, double radius) {
    Srf s; s.k = Srf::SPHERE; s.f = Frame::from_z(center, {0, 0, 1}); s.r = radius; return s;
}
Srf srf_torus(const V3& center, const V3& axis_dir, double major_r, double minor_r) {
    Srf s; s.k = Srf::TORUS; s.f = Frame::from_z(center, axis_dir);
    s.r = major_r; s.r2 = minor_r; return s;
}
Srf srf_nurbs(const NurbsSurface& s) {
    Srf r; r.k = Srf::NURBS; r.nrb = s; return r;
}

V3 Srf::eval(double u, double v) const {
    double cu = std::cos(u), su = std::sin(u);
    switch (k) {
    case PLANE: return f.to_world(u, v, 0);
    case CYLINDER: return f.to_world(r * cu, r * su, v);
    case CONE: {
        double rho = r + v * std::sin(r2);
        return f.to_world(rho * cu, rho * su, v * std::cos(r2));
    }
    case SPHERE: {
        double cv = std::cos(v), sv = std::sin(v);
        return f.to_world(r * cv * cu, r * cv * su, r * sv);
    }
    case TORUS: {
        double cv = std::cos(v), sv = std::sin(v);
        double rho = r + r2 * cv;
        return f.to_world(rho * cu, rho * su, r2 * sv);
    }
    case NURBS: return to_v3(nrb.point_at(u, v));
    }
    return {};
}

void Srf::d0d1(double u, double v, V3& p, V3& du, V3& dv) const {
    double cu = std::cos(u), su = std::sin(u);
    p = eval(u, v);
    switch (k) {
    case PLANE: du = f.x; dv = f.y; return;
    case CYLINDER:
        du = f.x * (-r * su) + f.y * (r * cu); dv = f.z; return;
    case CONE: {
        double sa = std::sin(r2), ca = std::cos(r2);
        double rho = r + v * sa;
        du = (f.x * (-su) + f.y * cu) * rho;
        dv = (f.x * cu + f.y * su) * sa + f.z * ca;
        return;
    }
    case SPHERE: {
        double cv = std::cos(v), sv = std::sin(v);
        du = (f.x * (-su) + f.y * cu) * (r * cv);
        dv = (f.x * cu + f.y * su) * (-r * sv) + f.z * (r * cv);
        return;
    }
    case TORUS: {
        double cv = std::cos(v), sv = std::sin(v);
        du = (f.x * (-su) + f.y * cu) * (r + r2 * cv);
        dv = (f.x * cu + f.y * su) * (-r2 * sv) + f.z * (r2 * cv);
        return;
    }
    case NURBS: {
        auto d = nrb.evaluate(u, v, 1);
        // project convention: evaluate returns [S, Sv, Su, ...]
        du = d.size() > 2 ? to_v3(d[2]) : V3{};
        dv = d.size() > 1 ? to_v3(d[1]) : V3{};
        return;
    }
    }
}

V3 Srf::normal(double u, double v) const {
    V3 p, du, dv;
    d0d1(u, v, p, du, dv);
    V3 n = du.cross(dv);
    if (n.norm2() < 1e-24 && has_implicit()) {
        V3 g = gradF(p);           // robust at sphere poles (du vanishes)
        if (g.norm2() > 1e-24) n = g;
    }
    return n.normalized() * nflip;
}

double Srf::F(const V3& p) const {
    V3 w = p - f.o;
    double h = w.dot(f.z);
    switch (k) {
    case PLANE: return h;
    case CYLINDER: return w.norm2() - h * h - r * r;
    case SPHERE: return w.norm2() - r * r;
    case CONE: {
        double ta = std::tan(r2);
        return (w.norm2() - h * h) - h * h * ta * ta; // double cone
    }
    case TORUS: {
        double g = w.norm2() + r * r - r2 * r2;
        return g * g - 4 * r * r * (w.norm2() - h * h);
    }
    case NURBS: return 0.0;
    }
    return 0.0;
}

V3 Srf::gradF(const V3& p) const {
    V3 w = p - f.o;
    double h = w.dot(f.z);
    switch (k) {
    case PLANE: return f.z;
    case CYLINDER: return (w - f.z * h) * 2.0;
    case SPHERE: return w * 2.0;
    case CONE: {
        double ta = std::tan(r2);
        return (w - f.z * h) * 2.0 - f.z * (2.0 * h * ta * ta);
    }
    case TORUS: {
        double g = w.norm2() + r * r - r2 * r2;
        return w * (4.0 * g) - (w - f.z * h) * (8.0 * r * r);
    }
    case NURBS: return {};
    }
    return {};
}

bool Srf::uv_of(const V3& p, double& u, double& v) const {
    V3 w = p - f.o;
    switch (k) {
    case PLANE:
        u = w.dot(f.x); v = w.dot(f.y); return true;
    case CYLINDER:
        u = wrap_2pi(std::atan2(w.dot(f.y), w.dot(f.x))); v = w.dot(f.z); return true;
    case CONE: {
        double h = w.dot(f.z);
        double ca = std::cos(r2);
        if (std::abs(ca) < 1e-12) return false;
        v = h / ca;
        u = wrap_2pi(std::atan2(w.dot(f.y), w.dot(f.x)));
        return true;
    }
    case SPHERE: {
        double rr = w.norm();
        if (rr < 1e-300) return false;
        double s = w.dot(f.z) / rr;
        s = std::max(-1.0, std::min(1.0, s));
        v = std::asin(s);
        double cv = std::cos(v);
        if (std::abs(cv) < 1e-12) { u = 0; v = (s > 0 ? PI / 2 : -PI / 2); return true; }
        u = wrap_2pi(std::atan2(w.dot(f.y), w.dot(f.x)));
        return true;
    }
    case TORUS: {
        u = wrap_2pi(std::atan2(w.dot(f.y), w.dot(f.x)));
        V3 radial = f.x * std::cos(u) + f.y * std::sin(u);
        V3 q = f.o + radial * r;         // tube-circle center
        V3 d = p - q;
        v = std::atan2(d.dot(f.z), d.dot(radial));
        v = wrap_2pi(v);
        return true;
    }
    case NURBS: {
        auto [u0, u1] = nrb.domain(0);
        auto [v0, v1] = nrb.domain(1);
        // coarse seed
        double bu = 0.5 * (u0 + u1), bv = 0.5 * (v0 + v1), bd = 1e300;
        for (int i = 0; i <= 8; i++)
            for (int j = 0; j <= 8; j++) {
                double uu = u0 + (u1 - u0) * i / 8.0, vv = v0 + (v1 - v0) * j / 8.0;
                double d = (to_v3(nrb.point_at(uu, vv)) - p).norm2();
                if (d < bd) { bd = d; bu = uu; bv = vv; }
            }
        u = bu; v = bv;
        for (int it = 0; it < 40; it++) {
            V3 sp, du, dv;
            d0d1(u, v, sp, du, dv);
            V3 r = sp - p;
            double a = du.norm2(), b = du.dot(dv), c = dv.norm2();
            double det = a * c - b * b;
            if (det < 1e-300) break;
            double ru = r.dot(du), rv = r.dot(dv);
            double ddu = (-ru * c + rv * b) / det;
            double ddv = (-rv * a + ru * b) / det;
            u += ddu; v += ddv;
            if ((du * ddu + dv * ddv).norm() < 1e-11) return true;
        }
        V3 sp = to_v3(nrb.point_at(u, v));
        return (sp - p).norm() < 1e-7;
    }
    }
    return false;
}

void Srf::canonical_domain(double& u0, double& u1, double& v0, double& v1) const {
    switch (k) {
    case PLANE: u0 = -1; u1 = 1; v0 = -1; v1 = 1; return;
    case CYLINDER: case CONE: u0 = 0; u1 = TWO_PI; v0 = 0; v1 = 1; return;
    case SPHERE: u0 = 0; u1 = TWO_PI; v0 = -PI / 2; v1 = PI / 2; return;
    case TORUS: u0 = 0; u1 = TWO_PI; v0 = 0; v1 = TWO_PI; return;
    case NURBS: {
        auto d0 = nrb.domain(0), d1 = nrb.domain(1);
        u0 = d0.first; u1 = d0.second; v0 = d1.first; v1 = d1.second; return;
    }
    }
}

// ============================================================================
// Recognition (residual-gated)
// ============================================================================

namespace {

// Jacobi eigensolver for symmetric 3x3 (recognition fits only).
void jacobi_eig3(const double a_in[3][3], double evals[3], V3 evecs[3]) {
    double a[3][3];
    std::memcpy(a, a_in, sizeof(a));
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int sweep = 0; sweep < 60; sweep++) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-300) break;
        for (int p = 0; p < 3; p++)
            for (int q = p + 1; q < 3; q++) {
                if (std::abs(a[p][q]) < 1e-300) continue;
                // Jacobi: zero a[p][q] with A' = J^T A J, J = [[c,-s],[s,c]].
                // tan(2t) = 2 a[p][q] / (a[p][p] - a[q][q])  (NOT the negated form)
                double theta = 0.5 * std::atan2(2 * a[p][q], a[p][p] - a[q][q]);
                double c = std::cos(theta), s = std::sin(theta);
                for (int k = 0; k < 3; k++) {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp + s * akq;
                    a[k][q] = -s * akp + c * akq;
                }
                for (int k = 0; k < 3; k++) {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk + s * aqk;
                    a[q][k] = -s * apk + c * aqk;
                }
                for (int k = 0; k < 3; k++) {
                    double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp + s * vkq;
                    v[k][q] = -s * vkp + c * vkq;
                }
            }
    }
    for (int k = 0; k < 3; k++) { evals[k] = a[k][k]; evecs[k] = {v[0][k], v[1][k], v[2][k]}; }
}

struct Grid {
    // N=5: domain fractions 0,.25,.5,.75,1 align EXACTLY with the quarter-arc
    // knot boundaries of the project's quadric charts, so ring averages hit
    // true 90-degree points.
    static constexpr int N = 5;
    V3 p[N][N];
    V3 n[N][N];
    double us[N], vs[N];
    double scale = 1.0;
    double tol = 1e-7;
};

bool fill_grid(const NurbsSurface& s, Grid& g) {
    auto [u0, u1] = s.domain(0);
    auto [v0, v1] = s.domain(1);
    for (int i = 0; i < Grid::N; i++) {
        g.us[i] = u0 + (u1 - u0) * i / (Grid::N - 1.0);
        g.vs[i] = v0 + (v1 - v0) * i / (Grid::N - 1.0);
    }
    V3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (int i = 0; i < Grid::N; i++)
        for (int j = 0; j < Grid::N; j++) {
            g.p[i][j] = to_v3(s.point_at(g.us[i], g.vs[j]));
            // normals from derivatives (normal_at hides degeneracy behind a
            // unit (0,0,1)); project convention: evaluate = [S, Sv, Su, ...],
            // normal direction follows normal_at = normalize(Sv x Su).
            auto d = s.evaluate(g.us[i], g.vs[j], 1);
            V3 n{0, 0, 0};
            if (d.size() > 2) {
                V3 sv = to_v3(d[1]), su = to_v3(d[2]);
                n = su.cross(sv); // normal_at convention = normalize(Su x Sv)
                if (n.norm() > 1e-300) n = n.normalized();
                else n = {0, 0, 0};
            }
            g.n[i][j] = n;
            lo.x = std::min(lo.x, g.p[i][j].x); hi.x = std::max(hi.x, g.p[i][j].x);
            lo.y = std::min(lo.y, g.p[i][j].y); hi.y = std::max(hi.y, g.p[i][j].y);
            lo.z = std::min(lo.z, g.p[i][j].z); hi.z = std::max(hi.z, g.p[i][j].z);
        }
    g.scale = (hi - lo).norm() + 1e-9;
    g.tol = g.scale * 1e-7 + 1e-9;
    return true;
}

// Verify candidate: roundtrip residual + set nflip so normal() matches source.
bool verify(const Grid& g, Srf& a) {
    static bool dbg = std::getenv("V3DBG") != nullptr;
    double worst = 0;
    for (int i = 0; i < Grid::N; i++)
        for (int j = 0; j < Grid::N; j++) {
            double u, v;
            if (!a.uv_of(g.p[i][j], u, v)) {
                if (dbg) std::fprintf(stderr, "[verify] uv_of failed at %d,%d\n", i, j);
                return false;
            }
            worst = std::max(worst, a.eval(u, v).dist(g.p[i][j]));
        }
    if (dbg) std::fprintf(stderr, "[verify] kind=%d worst=%.3g tol*20=%.3g\n",
                          (int)a.k, worst, g.tol * 20);
    if (worst > g.tol * 20) return false;
    // normal sign from a non-degenerate sample
    for (int i = 1; i < Grid::N - 1; i++)
        for (int j = 1; j < Grid::N - 1; j++) {
            double u, v;
            a.uv_of(g.p[i][j], u, v);
            V3 p, du, dv;
            a.d0d1(u, v, p, du, dv);
            V3 n = du.cross(dv);
            if (n.norm() > 1e-9 * g.scale * g.scale) {
                a.nflip = n.normalized().dot(g.n[i][j]) >= 0 ? 1.0 : -1.0;
                return true;
            }
        }
    a.nflip = 1.0;
    return true;
}

bool try_plane(const Grid& g, Srf& out) {
    V3 n0{0, 0, 0};
    for (int i = 0; i < Grid::N; i++)
        for (int j = 0; j < Grid::N; j++)
            if (g.n[i][j].norm() > 0.5) n0 = g.n[i][j];
    if (n0.norm() < 0.5) return false;
    for (int i = 0; i < Grid::N; i++)
        for (int j = 0; j < Grid::N; j++) {
            // skip degenerate normals (pyramid apex rows report (0,0,1) garbage)
            if (g.n[i][j].norm() > 0.5 && g.n[i][j].dot(n0) < 1.0 - 1e-6) return false;
            if (std::abs((g.p[i][j] - g.p[0][0]).dot(n0)) > g.tol * 20) return false;
        }
    V3 x = g.p[Grid::N - 1][0] - g.p[0][0];
    x = x - n0 * x.dot(n0);
    if (x.norm() < g.tol) x = g.p[0][Grid::N - 1] - g.p[0][0];
    out = srf_plane(g.p[0][0], n0, x);
    return verify(g, out);
}

bool try_sphere(const Grid& g, Srf& out) {
    // Kåsa fit: minimize ||P-c|^2 - r^2| linearized
    double ata[3][3] = {};
    double atb[3] = {};
    const V3& p0 = g.p[0][0];
    for (int i = 0; i < Grid::N; i++)
        for (int j = 0; j < Grid::N; j++) {
            V3 d = g.p[i][j] - p0;
            double rhs = 0.5 * (g.p[i][j].norm2() - p0.norm2());
            double row[3] = {d.x, d.y, d.z};
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) ata[r][c] += row[r] * row[c];
                atb[r] += row[r] * rhs;
            }
        }
    // solve 3x3
    double det = ata[0][0] * (ata[1][1] * ata[2][2] - ata[1][2] * ata[2][1])
               - ata[0][1] * (ata[1][0] * ata[2][2] - ata[1][2] * ata[2][0])
               + ata[0][2] * (ata[1][0] * ata[2][1] - ata[1][1] * ata[2][0]);
    if (std::abs(det) < 1e-300) return false;
    auto cof = [&](int r, int c) {
        double m[4]; int k = 0;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
            if (i != r && j != c) m[k++] = ata[i][j];
        double v = m[0] * m[3] - m[1] * m[2];
        return ((r + c) & 1) ? -v : v;
    };
    V3 c;
    double* cptr = &c.x;
    for (int col = 0; col < 3; col++) {
        double s = 0;
        for (int row = 0; row < 3; row++) s += cof(row, col) * atb[row];
        cptr[col] = s / det;
    }
    double r = 0;
    for (int i = 0; i < Grid::N; i++)
        for (int j = 0; j < Grid::N; j++) r += g.p[i][j].dist(c);
    r /= Grid::N * Grid::N;
    if (r < g.tol * 100) return false;
    // frame: z toward the v-increasing pole direction if poles exist, else
    // from the parameterization cross of iso tangents.
    V3 pn = g.p[Grid::N / 2][Grid::N - 1] - c;
    V3 ps = g.p[Grid::N / 2][0] - c;
    V3 z;
    if (pn.norm() > g.tol * 100 && ps.norm() > g.tol * 100 &&
        (pn + ps).norm() < 0.05 * r)  // antipodal -> full sphere chart
        z = (pn - ps).normalized();
    else
        z = pn.normalized();
    // x: direction of the u=0 ring point
    V3 w = g.p[0][Grid::N / 2] - c;
    w = w - z * w.dot(z);
    if (w.norm() < 1e-9 * r) { out = srf_sphere(c, r); out.f.z = z; out.f = Frame::from_z(c, z); }
    else {
        Srf s; s.k = Srf::SPHERE; s.r = r;
        s.f.o = c; s.f.z = z; s.f.x = w.normalized(); s.f.y = z.cross(s.f.x);
        out = s;
    }
    return verify(g, out);
}

// ring centers/radii at each v-row (exact when the row is a full circle:
// with N=5 the u-samples sit at 0/90/180/270 degrees of the quarter-arc chart)
void ring_stats(const Grid& g, int j, V3& c, double& rho) {
    c = (g.p[0][j] + g.p[1][j] + g.p[2][j] + g.p[3][j]) * 0.25;
    rho = 0;
    for (int i = 0; i < Grid::N; i++) rho += g.p[i][j].dist(c);
    rho /= Grid::N;
}

bool try_cyl_cone_torus(const Grid& g, Srf& out) {
    // ring stats per v-row
    V3 rc[Grid::N]; double rho[Grid::N];
    for (int j = 0; j < Grid::N; j++) ring_stats(g, j, rc[j], rho[j]);

    // axis: ring centers are collinear ON the revolution axis for
    // cylinder/cone/torus -> PCA max-eigenvector of the ring-center spread.
    // (normal covariance is wrong for torus: its covariance is near-isotropic.)
    V3 axis;
    {
        V3 cen{0, 0, 0};
        for (int j = 0; j < Grid::N; j++) cen += rc[j];
        cen = cen / Grid::N;
        double M[3][3] = {};
        for (int j = 0; j < Grid::N; j++) {
            V3 d = rc[j] - cen;
            double row[3] = {d.x, d.y, d.z};
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) M[r][c] += row[r] * row[c];
        }
        double evals[3]; V3 evecs[3];
        jacobi_eig3(M, evals, evecs);
        int kmax = 0;
        for (int k = 1; k < 3; k++) if (evals[k] > evals[kmax]) kmax = k;
        if (evals[kmax] < 1e-24) return false; // coincident centers: sphere-like
        axis = evecs[kmax].normalized();
    }

    // ring centers must be (near-)collinear on the axis for cyl/cone;
    // for torus they lie on the axis too (spread along it).
    // project ring centers on axis
    double h[Grid::N];
    double hmean = 0;
    for (int j = 0; j < Grid::N; j++) { h[j] = (rc[j] - rc[0]).dot(axis); hmean += h[j]; }
    hmean /= Grid::N;
    double off_axis = 0;
    for (int j = 0; j < Grid::N; j++) {
        V3 d = rc[j] - rc[0] - axis * h[j];
        off_axis = std::max(off_axis, d.norm());
    }
    {
        static bool dbg = std::getenv("V3DBG") != nullptr;
        if (dbg) {
            std::fprintf(stderr, "[cct] axis=(%.3f,%.3f,%.3f) off_axis=%.3g tol50=%.3g rho=",
                         axis.x, axis.y, axis.z, off_axis, g.tol * 50);
            for (int j = 0; j < Grid::N; j++) std::fprintf(stderr, "%.4f ", rho[j]);
            std::fprintf(stderr, "\n");
        }
    }
    if (off_axis > g.tol * 50) return false;

    // classify by rho(h): constant -> cylinder, linear -> cone, circle -> torus
    double h0 = h[0], h1 = h[Grid::N - 1];
    double span = h1 - h0;
    bool conic_span = std::abs(span) > g.tol * 100;

    // CYLINDER
    {
        double rmean = 0;
        for (int j = 0; j < Grid::N; j++) rmean += rho[j];
        rmean /= Grid::N;
        double var = 0;
        for (int j = 0; j < Grid::N; j++) var = std::max(var, std::abs(rho[j] - rmean));
        if (rmean > g.tol * 100 && var < g.tol * 20) {
            // axis point: centroid of ring centers
            V3 c{0, 0, 0};
            for (int j = 0; j < Grid::N; j++) c += rc[j];
            c = c / Grid::N;
            // orient z so v increases along z
            V3 z = axis;
            if (conic_span && span < 0) z = axis * -1.0;
            V3 w = g.p[0][Grid::N / 2] - c;
            w = w - z * w.dot(z);
            Srf s; s.k = Srf::CYLINDER; s.r = rmean;
            s.f.o = c; s.f.z = z;
            s.f.x = w.normalized();
            if (s.f.x.norm() < 0.5) { out = srf_cylinder(c, z, rmean); }
            else { s.f.y = z.cross(s.f.x); out = s; }
            if (verify(g, out)) return true;
        }
    }
    // CONE (radius linear in h, nonzero slope)
    if (conic_span) {
        // least squares rho = a + b*h
        double sh = 0, sr = 0, shh = 0, shr = 0;
        for (int j = 0; j < Grid::N; j++) {
            sh += h[j]; sr += rho[j]; shh += h[j] * h[j]; shr += h[j] * rho[j];
        }
        double n = Grid::N;
        double den = n * shh - sh * sh;
        if (std::abs(den) > 1e-300) {
            double b = (n * shr - sh * sr) / den;
            double a = (sr - b * sh) / n;
            double lin_err = 0;
            for (int j = 0; j < Grid::N; j++)
                lin_err = std::max(lin_err, std::abs(rho[j] - (a + b * h[j])));
            if (std::abs(b) > 1e-6 && lin_err < g.tol * 20) {
                // apex at h where rho=0: h_a = -a/b (relative to rc[0])
                V3 z = axis * (span > 0 ? 1.0 : -1.0);
                // recompute h along z
                double ha = -a / b;   // apex offset from rc[0] along `axis`
                V3 apex = rc[0] + axis * ha;
                // semi-angle: |b| vs axis; sign: radius grows along +z
                double alpha = std::atan(std::abs(b));
                // axis must point from apex toward increasing radius:
                double dr_dz = b * (span > 0 ? 1.0 : -1.0);
                if (dr_dz < 0) z = z * -1.0;
                V3 w = g.p[0][Grid::N / 2] - apex;
                w = w - z * w.dot(z);
                Srf s; s.k = Srf::CONE; s.r = 0; s.r2 = alpha;
                s.f.o = apex; s.f.z = z;
                s.f.x = w.norm() > 1e-12 ? w.normalized() : Frame::from_z(apex, z).x;
                s.f.y = z.cross(s.f.x);
                out = s;
                if (verify(g, out)) return true;
            }
        }
    }
    // TORUS: rho_j = R + r*cos(v_j), z_j = r*sin(v_j) with z_j along axis
    {
        // use rows j=0..N-1 with (rho_j, z_j) lying on circle (rho-R)^2 + z^2 = r^2
        // ring centers rc[j] are ON the axis; their axial position = z_j.
        double zj[Grid::N];
        for (int j = 0; j < Grid::N; j++) zj[j] = h[j];
        // fit circle in (rho, z): Kasa 2D
        double srr = 0, srz = 0, szz = 0, sb = 0, sbr = 0, sbz = 0, sr = 0, sz = 0, sn = Grid::N;
        for (int j = 0; j < Grid::N; j++) {
            double x = rho[j], y = zj[j];
            double b = -(x * x + y * y);
            srr += x * x; srz += x * y; szz += y * y;
            sb += b; sbr += x * b; sbz += y * b; sr += x; sz += y;
        }
        // solve [srr srz sr; srz szz sz; sr sz sn] [D E F]^T = [sbr sbz sb]
        double A[3][3] = {{srr, srz, sr}, {srz, szz, sz}, {sr, sz, sn}};
        double B[3] = {sbr, sbz, sb};
        // gaussian elimination
        bool ok = true;
        for (int col = 0; col < 3 && ok; col++) {
            int piv = col;
            for (int r2 = col + 1; r2 < 3; r2++)
                if (std::abs(A[r2][col]) > std::abs(A[piv][col])) piv = r2;
            if (std::abs(A[piv][col]) < 1e-300) ok = false;
            std::swap(A[piv], A[col]); // swaps B rows too? no - do manual
            double tmpB = B[piv]; B[piv] = B[col]; B[col] = tmpB;
            // note: swapping A rows element-wise rows
            for (int r2 = col + 1; r2 < 3; r2++) {
                double f = A[r2][col] / A[col][col];
                for (int c2 = col; c2 < 3; c2++) A[r2][c2] -= f * A[col][c2];
                B[r2] -= f * B[col];
            }
        }
        if (ok) {
            double X[3];
            for (int r2 = 2; r2 >= 0; r2--) {
                double s = B[r2];
                for (int c2 = r2 + 1; c2 < 3; c2++) s -= A[r2][c2] * X[c2];
                X[r2] = s / A[r2][r2];
            }
            double R = -X[0] / 2, zoff = -X[1] / 2;
            double rr = std::sqrt(std::max(0.0, R * R + zoff * zoff - X[2]));
            double fit_err = 0;
            for (int j = 0; j < Grid::N; j++) {
                double d = std::sqrt((rho[j] - R) * (rho[j] - R) + (zj[j] - zoff) * (zj[j] - zoff));
                fit_err = std::max(fit_err, std::abs(d - rr));
            }
            if (R > g.tol * 100 && rr > g.tol * 100 && rr < R && fit_err < g.tol * 50) {
                V3 z = axis;
                V3 center = rc[0] + axis * zoff;
                V3 w = g.p[0][Grid::N / 2] - center;
                w = w - z * w.dot(z);
                Srf s; s.k = Srf::TORUS; s.r = R; s.r2 = rr;
                s.f.o = center; s.f.z = z;
                s.f.x = w.norm() > 1e-12 ? w.normalized() : Frame::from_z(center, z).x;
                s.f.y = z.cross(s.f.x);
                out = s;
                if (verify(g, out)) return true;
            }
        }
    }
    return false;
}

} // namespace

Srf recognize(const NurbsSurface& s, double tol) {
    if (!s.is_valid()) return srf_nurbs(s);
    (void)tol;
    Grid g;
    fill_grid(s, g);
    Srf out;
    static bool dbg = std::getenv("V3DBG") != nullptr;
    if (try_plane(g, out)) return out;
    if (dbg) std::fprintf(stderr, "[recog] not plane\n");
    if (try_sphere(g, out)) return out;
    if (dbg) std::fprintf(stderr, "[recog] not sphere\n");
    if (try_cyl_cone_torus(g, out)) return out;
    if (dbg) std::fprintf(stderr, "[recog] not cyl/cone/torus\n");
    return srf_nurbs(s);
}

bool recognize_curve_pts(const V3* pts, int n, double tol, Cur& out) {
    if (n < 2) return false;
    double scale = pts[0].dist(pts[n - 1]);
    for (int i = 0; i < n; i++) scale = std::max(scale, pts[0].dist(pts[i]));
    scale += 1e-9;
    double gt = std::max(tol, scale * 1e-7);

    // LINE
    {
        V3 d = (pts[n - 1] - pts[0]);
        double L = d.norm();
        if (L > gt) {
            d = d / L;
            bool ok = true;
            for (int i = 1; i < n - 1 && ok; i++) {
                V3 w = pts[i] - pts[0];
                if (d.cross(w).norm() > gt * 20) ok = false;
            }
            if (ok) { out = cur_line(pts[0], d); return true; }
        }
    }
    if (n < 5) return false;
    // CIRCLE / ELLIPSE: fit plane via PCA then conic in plane
    {
        V3 cen{0, 0, 0};
        for (int i = 0; i < n; i++) cen += pts[i];
        cen = cen / n;
        double M[3][3] = {};
        for (int i = 0; i < n; i++) {
            V3 d = pts[i] - cen;
            double row[3] = {d.x, d.y, d.z};
            for (int r = 0; r < 3; r++)
                for (int cc = 0; cc < 3; cc++) M[r][cc] += row[r] * row[cc];
        }
        double evals[3]; V3 evecs[3];
        jacobi_eig3(M, evals, evecs);
        int kmin = 0;
        for (int k = 1; k < 3; k++) if (evals[k] < evals[kmin]) kmin = k;
        V3 nn = evecs[kmin].normalized();
        // planarity
        bool planar = true;
        for (int i = 0; i < n; i++)
            if (std::abs((pts[i] - cen).dot(nn)) > gt * 20) planar = false;
        if (planar && nn.norm() > 0.5) {
            Frame fr = Frame::from_z(cen, nn);
            // 2D conic fit: A x^2 + B xy + C y^2 + D x + E y = 1 (scaled)
            // accumulate normal equations (5 params)
            double ata[5][5] = {}, atb[5] = {};
            for (int i = 0; i < n; i++) {
                V3 w = pts[i] - cen;
                double x = w.dot(fr.x), y = w.dot(fr.y);
                double row[5] = {x * x, x * y, y * y, x, y};
                double b = 1.0;
                for (int r = 0; r < 5; r++) {
                    for (int cc = 0; cc < 5; cc++) ata[r][cc] += row[r] * row[cc];
                    atb[r] += row[r] * b;
                }
            }
            // gaussian solve 5x5 with pivoting
            bool ok = true;
            for (int col = 0; col < 5 && ok; col++) {
                int piv = col;
                for (int r = col + 1; r < 5; r++)
                    if (std::abs(ata[r][col]) > std::abs(ata[piv][col])) piv = r;
                if (std::abs(ata[piv][col]) < 1e-300) ok = false;
                else {
                    for (int cc = 0; cc < 5; cc++) std::swap(ata[piv][cc], ata[col][cc]);
                    std::swap(atb[piv], atb[col]);
                    for (int r = col + 1; r < 5; r++) {
                        double f = ata[r][col] / ata[col][col];
                        for (int cc = col; cc < 5; cc++) ata[r][cc] -= f * ata[col][cc];
                        atb[r] -= f * atb[col];
                    }
                }
            }
            double X[5] = {};
            if (ok) {
                for (int r = 4; r >= 0; r--) {
                    double s = atb[r];
                    for (int cc = r + 1; cc < 5; cc++) s -= ata[r][cc] * X[cc];
                    X[r] = s / ata[r][r];
                }
                double A = X[0], B = X[1], C = X[2], D = X[3], E = X[4];
                // conic must be ellipse
                double disc = B * B - 4 * A * C;
                if (disc < 0) {
                    // center: grad = 0 -> [2A B; B 2C] c = -[D E]
                    double det2 = 4 * A * C - B * B;
                    double cx = (-2 * C * D + B * E) / det2;
                    double cy = (-2 * A * E + B * D) / det2;
                    // axes: eigen of [A B/2; B/2 C]
                    double tr = A + C, dt2 = std::sqrt((A - C) * (A - C) + B * B);
                    double l1 = (tr - dt2) / 2, l2 = (tr + dt2) / 2;
                    // F_c (value at center, incl. the -1 constant) and scale K = -F_c
                    double Fc = A * cx * cx + B * cx * cy + C * cy * cy + D * cx + E * cy;
                    double K = 1.0 - Fc;
                    // radii^2 = K / l_i : K and both eigenvalues must share a sign
                    if (l1 * K > 0 && l2 * K > 0) {
                        double r1 = std::sqrt(K / l1), r2 = std::sqrt(K / l2);
                        double rmaj, rmin, lmaj;
                        if (r1 >= r2) { rmaj = r1; rmin = r2; lmaj = l1; }
                        else { rmaj = r2; rmin = r1; lmaj = l2; }
                        // major-axis direction: eigenvector of lmaj
                        V3 v1;
                        {
                            double ax = B / 2, ay = lmaj - A;   // (B/2, lmaj-A)
                            double bx = lmaj - C, by = B / 2;   // (lmaj-C, B/2)
                            if (ax * ax + ay * ay >= bx * bx + by * by) v1 = fr.x * ax + fr.y * ay;
                            else v1 = fr.x * bx + fr.y * by;
                        }
                        if (v1.norm() < 1e-12) v1 = fr.x;
                        v1 = v1.normalized();
                        V3 c3 = cen + fr.x * cx + fr.y * cy;
                        Frame ef;
                        ef.o = c3; ef.z = nn;
                        ef.x = v1; ef.y = nn.cross(v1);
                        // verify: max residual
                        double worst = 0;
                        Cur ec = cur_ellipse(ef, rmaj, rmin);
                        for (int i = 0; i < n; i++) {
                            double t;
                            ec.project(pts[i], t);
                            worst = std::max(worst, ec.eval(t).dist(pts[i]));
                        }
                        if (worst < gt * 50) {
                            // circle hypothesis: near-equal radii -> refit radius
                            // as mean distance (eigenvalue splitting inflates
                            // the conic radii for near-circular data)
                            if (std::abs(rmaj - rmin) < 0.005 * rmaj) {
                                double rm = 0;
                                for (int i = 0; i < n; i++) rm += pts[i].dist(c3);
                                rm /= n;
                                out = cur_circle(ef, rm);
                                return true;
                            }
                            out = ec;
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

Cur recognize_curve(const NurbsCurve& c, double tol) {
    if (!c.is_valid()) return cur_nurbs(c);
    auto dom = c.domain();
    constexpr int N = 8;
    V3 pts[N];
    for (int i = 0; i < N; i++)
        pts[i] = to_v3(c.point_at(dom.first + (dom.second - dom.first) * i / (N - 1.0)));
    Cur out;
    if (recognize_curve_pts(pts, N, tol, out)) return out;
    return cur_nurbs(c);
}

} // namespace v3
