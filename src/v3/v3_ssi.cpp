#include "v3_ssi.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace v3 {

namespace {

constexpr double EPS_ANG = 1e-9;   // angle parallelism epsilon
constexpr double EPS_POS = 1e-9;   // positional epsilon (scaled by caller tol where needed)

int solve_quadratic(double a, double b, double c, double* roots) {
    if (std::abs(a) < 1e-300) {
        if (std::abs(b) < 1e-300) return 0;
        roots[0] = -c / b;
        return 1;
    }
    double disc = b * b - 4 * a * c;
    if (disc < 0) return 0;
    double sq = std::sqrt(disc);
    double q = -0.5 * (b + (b >= 0 ? sq : -sq));
    if (std::abs(q) < 1e-300) { roots[0] = 0; return 1; }
    roots[0] = q / a;
    roots[1] = c / q;
    return 2;
}

// 2D conic A x^2 + B xy + C y^2 + D x + E y + F = 0 (F included here) in a
// plane frame -> ELLIPSE/CIRCLE curve. False when not a non-degenerate ellipse.
bool conic2d_ellipse(double A, double B, double C, double D, double E, double K0,
                     const Frame& pf, Cur& out) {
    double disc = B * B - 4 * A * C;
    if (disc >= 0) return false;
    double det2 = 4 * A * C - B * B;
    if (std::abs(det2) < 1e-300) return false;
    double cx = (-2 * C * D + B * E) / det2;
    double cy = (-2 * A * E + B * D) / det2;
    double tr = A + C, dt2 = std::sqrt((A - C) * (A - C) + B * B);
    double l1 = (tr - dt2) / 2, l2 = (tr + dt2) / 2;
    double Fc = A * cx * cx + B * cx * cy + C * cy * cy + D * cx + E * cy + K0;
    double K = -Fc;
    if (l1 <= 1e-300 || l2 <= 1e-300 || K <= 0) return false;
    double rmaj = std::sqrt(K / l1), rmin = std::sqrt(K / l2);
    double ax = B / 2, ay = l1 - A;
    double bx = l1 - C, by = B / 2;
    V3 v1;
    if (ax * ax + ay * ay >= bx * bx + by * by) v1 = pf.x * ax + pf.y * ay;
    else v1 = pf.x * bx + pf.y * by;
    if (v1.norm() < 1e-12) v1 = pf.x;
    v1 = v1.normalized();
    Frame ef;
    ef.o = pf.o + pf.x * cx + pf.y * cy;
    ef.x = v1;
    ef.z = pf.z;
    ef.y = pf.z.cross(v1);
    if (std::abs(rmaj - rmin) < 1e-9 * rmaj) out = cur_circle(ef, 0.5 * (rmaj + rmin));
    else out = cur_ellipse(ef, rmaj, rmin);
    return true;
}

// circle at height h along axis: center on axis line, given radius, axis dir z
Cur axis_circle(const V3& center, const V3& axis, double radius) {
    return cur_circle(Frame::from_z(center, axis), radius);
}

void add_circle(SSIResult& out, const Srf& a, const Srf& b,
                const V3& center, const V3& normal, double radius, double tol3d) {
    if (radius < 1e-7) return; // tangent point — not a curve
    Cur c = axis_circle(center, normal, radius);
    sample_exact(a, b, c, 0, TWO_PI, true, out, tol3d);
}

// ---- same domain -------------------------------------------------------------
bool same_surface(const Srf& a, const Srf& b, double tol) {
    if (a.k != b.k || a.k == Srf::NURBS) return false;
    switch (a.k) {
    case Srf::PLANE:
        return std::abs(a.f.z.dot(b.f.z)) > 1 - EPS_ANG &&
               std::abs((a.f.o - b.f.o).dot(a.f.z)) < tol;
    case Srf::CYLINDER: {
        if (std::abs(a.f.z.dot(b.f.z)) < 1 - EPS_ANG) return false;
        V3 w = b.f.o - a.f.o;
        double d = (w - a.f.z * w.dot(a.f.z)).norm();
        return d < tol && std::abs(a.r - b.r) < tol;
    }
    case Srf::SPHERE:
        return a.f.o.dist(b.f.o) < tol && std::abs(a.r - b.r) < tol;
    case Srf::CONE:
        return a.f.o.dist(b.f.o) < tol && std::abs(a.r2 - b.r2) < 1e-9 &&
               std::abs(a.f.z.dot(b.f.z)) > 1 - EPS_ANG;
    case Srf::TORUS:
        return a.f.o.dist(b.f.o) < tol && std::abs(a.r - b.r) < tol &&
               std::abs(a.r2 - b.r2) < tol &&
               std::abs(a.f.z.dot(b.f.z)) > 1 - EPS_ANG;
    default: return false;
    }
}

// ---- exact pair cases --------------------------------------------------------
// Each returns true when it fully determined the result (possibly empty).

bool plane_plane(const Srf& p1, const Srf& p2, SSIResult& out, double tol3d) {
    V3 n1 = p1.f.z, n2 = p2.f.z;
    V3 dir = n1.cross(n2);
    if (dir.norm2() < EPS_ANG * EPS_ANG) return true; // parallel: empty or same-domain (checked earlier)
    dir = dir.normalized();
    double d1 = p1.f.o.dot(n1), d2 = p2.f.o.dot(n2);
    double cs = n1.dot(n2);
    double den = 1 - cs * cs;
    V3 pt = (n1 * (d1 - d2 * cs) + n2 * (d2 - d1 * cs)) / den;
    Cur line = cur_line(pt, dir);
    // bound the line to a generous segment: project both plane origins +- scale
    double sc = 1000.0;
    sample_exact(p1, p2, line, -sc, sc, false, out, tol3d);
    // trim to real extent later by face clipping; keep only midpoint region
    if (!out.curves.empty()) {
        auto& c = out.curves.back();
        // keep points within sc of pt (all of them); nothing to do
        (void)c;
    }
    return true;
}

bool plane_cylinder(const Srf& pl, const Srf& cy, SSIResult& out, double tol3d) {
    V3 n = pl.f.z, a = cy.f.z;
    double an = a.dot(n);
    double d = (cy.f.o - pl.f.o).dot(n); // signed distance axis point -> plane
    if (std::abs(an) > 1 - EPS_ANG) {
        // axis perpendicular to plane: circle
        double t = -d / an;
        V3 center = cy.f.o + a * t;
        add_circle(out, pl, cy, center, a, cy.r, tol3d);
        return true;
    }
    if (std::abs(an) < EPS_ANG) {
        // axis parallel to plane: 0/1/2 lines
        if (std::abs(d) > cy.r + tol3d) return true;
        V3 base = cy.f.o - n * d; // closest point on axis projected onto plane
        double s = std::sqrt(std::max(0.0, cy.r * cy.r - d * d));
        V3 m = n.cross(a).normalized();
        double sc = 1000.0;
        if (s < tol3d) {
            sample_exact(pl, cy, cur_line(base, a), -sc, sc, false, out, tol3d);
        } else {
            sample_exact(pl, cy, cur_line(base + m * s, a), -sc, sc, false, out, tol3d);
            sample_exact(pl, cy, cur_line(base - m * s, a), -sc, sc, false, out, tol3d);
        }
        return true;
    }
    // oblique: ellipse
    double t = -d / an;
    V3 center = cy.f.o + a * t;
    V3 e1 = (a - n * an).normalized();       // major direction (in plane)
    V3 e2 = n.cross(e1).normalized();
    double rmaj = cy.r / std::abs(an); // major = r/cos(tilt), minor = r
    Frame ef;
    ef.o = center; ef.x = e1; ef.y = e2; ef.z = n;
    Cur el = cur_ellipse(ef, rmaj, cy.r);
    sample_exact(pl, cy, el, 0, TWO_PI, true, out, tol3d);
    return true;
}

bool plane_sphere(const Srf& pl, const Srf& sp, SSIResult& out, double tol3d) {
    double d = (sp.f.o - pl.f.o).dot(pl.f.z);
    if (std::abs(d) > sp.r - tol3d) return true; // empty or tangent point
    V3 center = sp.f.o - pl.f.z * d;
    double r = std::sqrt(sp.r * sp.r - d * d);
    add_circle(out, pl, sp, center, pl.f.z, r, tol3d);
    return true;
}

bool plane_cone(const Srf& pl, const Srf& cn, SSIResult& out, double tol3d) {
    V3 n = pl.f.z, a = cn.f.z;
    double an = a.dot(n);
    double alpha = cn.r2;
    double beta = std::acos(std::max(-1.0, std::min(1.0, an))); // angle(axis, normal)
    V3 w0 = pl.f.o - cn.f.o;
    double apex_dist = w0.dot(n); // signed apex -> plane
    if (std::abs(an) > 1 - EPS_ANG) {
        // plane perpendicular to axis: circle (or apex point)
        double t = apex_dist / an; // along a from apex to plane
        if (std::abs(t) * std::tan(alpha) < 1e-7) return true; // apex only
        V3 center = cn.f.o + a * t;
        add_circle(out, pl, cn, center, a, std::abs(t) * std::tan(alpha), tol3d);
        return true;
    }
    if (std::abs(apex_dist) < tol3d) {
        // plane through apex: generator lines if beta < 90-alpha... i.e. axis
        // nearly in plane => lines exist when |component of a perp to n| >= cos? solve:
        V3 ap = a - n * an; // axis dir projected into plane
        double apl = ap.norm();
        double x = std::cos(alpha) / apl; // m . a = cos alpha, m in plane
        if (apl < EPS_ANG) return true;   // axis perp plane (handled above)
        if (std::abs(x) > 1.0) return true; // apex only
        double y = std::sqrt(std::max(0.0, 1.0 - x * x));
        V3 e1 = ap / apl;
        V3 e2 = n.cross(e1).normalized();
        if (y < 1e-9) {
            sample_exact(pl, cn, cur_line(cn.f.o, e1 * (x > 0 ? 1.0 : -1.0)),
                         1e-6, 1000.0, false, out, tol3d);
        } else {
            V3 m1 = e1 * x + e2 * y, m2 = e1 * x - e2 * y;
            sample_exact(pl, cn, cur_line(cn.f.o, m1), 1e-6, 1000.0, false, out, tol3d);
            sample_exact(pl, cn, cur_line(cn.f.o, m2), 1e-6, 1000.0, false, out, tol3d);
        }
        return true;
    }
    // general conic: reduce 2D conic in the plane frame; ellipse -> exact,
    // parabola/hyperbola -> caller falls back to the walker.
    double ta = std::tan(alpha);
    double k = 1 + ta * ta;
    Frame pf = pl.f;
    V3 e1 = pf.x, e2 = pf.y;
    double e1a = e1.dot(a), e2a = e2.dot(a), w0a = w0.dot(a);
    double A = 1 - k * e1a * e1a;
    double B = -2 * k * e1a * e2a;
    double C = 1 - k * e2a * e2a;
    double D = 2 * (w0.dot(e1) - k * w0a * e1a);
    double E = 2 * (w0.dot(e2) - k * w0a * e2a);
    double K0 = w0.dot(w0) - k * w0a * w0a;
    Cur conic;
    double margin = 1e-6;
    if (beta > alpha + margin && conic2d_ellipse(A, B, C, D, E, K0, pf, conic)) {
        sample_exact(pl, cn, conic, 0, TWO_PI, true, out, tol3d);
        return true;
    }
    return false; // parabola / hyperbola / degenerate: walker
}

bool sphere_sphere(const Srf& s1, const Srf& s2, SSIResult& out, double tol3d) {
    V3 d12 = s2.f.o - s1.f.o;
    double d = d12.norm();
    if (d < EPS_POS) return true; // concentric: same-domain checked earlier
    if (d > s1.r + s2.r - tol3d || d < std::abs(s1.r - s2.r) + tol3d) return true;
    double a = (s1.r * s1.r - s2.r * s2.r + d * d) / (2 * d);
    double h2 = s1.r * s1.r - a * a;
    if (h2 <= 0) return true;
    V3 center = s1.f.o + d12 * (a / d);
    add_circle(out, s1, s2, center, d12 / d, std::sqrt(h2), tol3d);
    return true;
}

bool cylinder_cylinder(const Srf& c1, const Srf& c2, SSIResult& out, double tol3d) {
    V3 a1 = c1.f.z, a2 = c2.f.z;
    V3 w = a1.cross(a2);
    double wl = w.norm();
    if (wl < EPS_ANG) {
        // parallel axes: cross-section circle-circle in the perpendicular plane
        V3 d = c2.f.o - c1.f.o;
        V3 dr = d - a1 * d.dot(a1);
        double dist = dr.norm();
        if (dist < EPS_POS) return true; // coaxial: same-domain (checked) or empty
        if (dist > c1.r + c2.r - tol3d || dist < std::abs(c1.r - c2.r) + tol3d) return true;
        V3 e1 = dr / dist;
        V3 e2 = a1.cross(e1);
        double x = (c1.r * c1.r - c2.r * c2.r + dist * dist) / (2 * dist);
        double y2 = c1.r * c1.r - x * x;
        if (y2 < 0) return true;
        double y = std::sqrt(y2);
        double sc = 1000.0;
        V3 base = c1.f.o + e1 * x;
        if (y < tol3d) {
            sample_exact(c1, c2, cur_line(base, a1), -sc, sc, false, out, tol3d);
        } else {
            sample_exact(c1, c2, cur_line(base + e2 * y, a1), -sc, sc, false, out, tol3d);
            sample_exact(c1, c2, cur_line(base - e2 * y, a1), -sc, sc, false, out, tol3d);
        }
        return true;
    }
    // do axes intersect? closest points
    V3 dd = c2.f.o - c1.f.o;
    double a1a2 = a1.dot(a2);
    double den = 1 - a1a2 * a1a2;
    double t1 = (dd.dot(a1) - dd.dot(a2) * a1a2) / den;
    double t2 = (dd.dot(a1) * a1a2 - dd.dot(a2)) / den * -1.0;
    // recompute t2 properly: minimize |c1 + t1 a1 - c2 - t2 a2|
    // [1 -cs; -cs 1][t1; -t2]... solve directly:
    double rhs1 = dd.dot(a1), rhs2 = -dd.dot(a2);
    t1 = (rhs1 - a1a2 * rhs2) / den;
    t2 = (a1a2 * rhs1 - rhs2) / den;
    V3 p1 = c1.f.o + a1 * t1, p2 = c2.f.o + a2 * t2;
    if (p1.dist(p2) > tol3d * 10) return false; // skew: walker
    if (std::abs(c1.r - c2.r) > tol3d * 10) return false; // Steinmetz needs equal radii
    V3 P0 = (p1 + p2) * 0.5;
    V3 e1 = (a1 + a2).normalized();
    V3 e2 = (a1 - a2).normalized();
    V3 wn = w / wl;
    double th = std::acos(std::max(-1.0, std::min(1.0, a1a2)));
    double r = c1.r;
    Frame f1; f1.o = P0; f1.x = e1; f1.y = wn; f1.z = e1.cross(wn);
    Frame f2; f2.o = P0; f2.x = e2; f2.y = wn; f2.z = e2.cross(wn);
    Cur el1 = cur_ellipse(f1, r / std::sin(th / 2), r);
    Cur el2 = cur_ellipse(f2, r / std::cos(th / 2), r);
    sample_exact(c1, c2, el1, 0, TWO_PI, true, out, tol3d);
    sample_exact(c1, c2, el2, 0, TWO_PI, true, out, tol3d);
    return true;
}

// sphere center on cylinder axis -> circles
bool cylinder_sphere(const Srf& cy, const Srf& sp, SSIResult& out, double tol3d) {
    V3 w = sp.f.o - cy.f.o;
    double h = w.dot(cy.f.z);
    V3 radial = w - cy.f.z * h;
    if (radial.norm() > tol3d * 10) return false; // center not on axis: walker
    if (sp.r < cy.r - tol3d) return true;
    double dz2 = sp.r * sp.r - cy.r * cy.r;
    if (dz2 < 0) return true;
    double dz = std::sqrt(dz2);
    add_circle(out, cy, sp, cy.f.o + cy.f.z * (h + dz), cy.f.z, cy.r, tol3d);
    if (dz > tol3d)
        add_circle(out, cy, sp, cy.f.o + cy.f.z * (h - dz), cy.f.z, cy.r, tol3d);
    return true;
}

bool cylinder_cone(const Srf& cy, const Srf& cn, SSIResult& out, double tol3d) {
    // coaxial only
    if (std::abs(cy.f.z.dot(cn.f.z)) < 1 - EPS_ANG) return false;
    V3 w = cy.f.o - cn.f.o;
    V3 radial = w - cn.f.z * w.dot(cn.f.z);
    if (radial.norm() > tol3d * 10) return false;
    double ta = std::tan(cn.r2);
    if (ta < 1e-12) return true;
    double zstar = cy.r / ta;
    // direction from apex toward cylinder: choose sign so z* lands on the nappe
    add_circle(out, cy, cn, cn.f.o + cn.f.z * zstar, cn.f.z, cy.r, tol3d);
    return true;
}

bool sphere_cone(const Srf& sp, const Srf& cn, SSIResult& out, double tol3d) {
    if (std::abs(sp.f.z.dot(cn.f.z)) < 1 - EPS_ANG) return false;
    V3 w = sp.f.o - cn.f.o;
    double h0 = w.dot(cn.f.z);
    V3 radial = w - cn.f.z * h0;
    if (radial.norm() > tol3d * 10) return false;
    double ta = std::tan(cn.r2);
    // z^2 (1+ta^2) - 2 h0 z + h0^2 - R^2 = 0
    double roots[2];
    int n = solve_quadratic(1 + ta * ta, -2 * h0, h0 * h0 - sp.r * sp.r, roots);
    for (int i = 0; i < n; i++) {
        double z = roots[i];
        double rho = z * ta;
        if (rho < 1e-7) continue;
        add_circle(out, sp, cn, cn.f.o + cn.f.z * z, cn.f.z, rho, tol3d);
    }
    return true;
}

bool cone_cone(const Srf& c1, const Srf& c2, SSIResult& out, double tol3d) {
    // shared apex: generator lines
    if (c1.f.o.dist(c2.f.o) < tol3d * 10) {
        V3 a1 = c1.f.z, a2 = c2.f.z;
        double d = a1.dot(a2);
        if (std::abs(d) > 1 - EPS_ANG) return true; // coaxial: apex only / same-domain
        V3 w = a1.cross(a2);
        V3 wn = w.normalized();
        double ca = std::cos(c1.r2), cb = std::cos(c2.r2);
        // m = alpha a1 + beta a2 + gamma wn
        double alpha = (ca - cb * d) / (1 - d * d);
        double beta = (cb - ca * d) / (1 - d * d);
        double g2 = 1 - alpha * alpha - beta * beta - 2 * alpha * beta * d;
        if (g2 < 0) return true;
        double g = std::sqrt(g2);
        V3 m0 = a1 * alpha + a2 * beta;
        if (g < 1e-9) {
            sample_exact(c1, c2, cur_line(c1.f.o, m0.normalized()), 1e-6, 1000.0, false, out, tol3d);
        } else {
            sample_exact(c1, c2, cur_line(c1.f.o, (m0 + wn * g).normalized()), 1e-6, 1000.0, false, out, tol3d);
            sample_exact(c1, c2, cur_line(c1.f.o, (m0 - wn * g).normalized()), 1e-6, 1000.0, false, out, tol3d);
        }
        return true;
    }
    // coaxial cones: only meet at a point (or same domain) -> nothing
    if (std::abs(c1.f.z.dot(c2.f.z)) > 1 - EPS_ANG) {
        V3 w = c2.f.o - c1.f.o;
        V3 radial = w - c1.f.z * w.dot(c1.f.z);
        if (radial.norm() < tol3d * 10) return true;
    }
    return false;
}

bool plane_torus(const Srf& pl, const Srf& to, SSIResult& out, double tol3d) {
    if (std::abs(pl.f.z.dot(to.f.z)) < 1 - EPS_ANG) return false; // not perpendicular: walker
    double h = (pl.f.o - to.f.o).dot(to.f.z);
    if (std::abs(h) > to.r2 + tol3d) return true;
    double dz = std::sqrt(std::max(0.0, to.r2 * to.r2 - h * h));
    V3 base = to.f.o + to.f.z * h;
    add_circle(out, pl, to, base, to.f.z, to.r + dz, tol3d);
    if (to.r - dz > 1e-7)
        add_circle(out, pl, to, base, to.f.z, to.r - dz, tol3d);
    return true;
}

// torus x quadric coaxial specials: quadric axis == torus axis
bool torus_coaxial(const Srf& to, const Srf& q, SSIResult& out, double tol3d) {
    if (q.k == Srf::SPHERE || q.k == Srf::CYLINDER || q.k == Srf::CONE) {
        if (std::abs(q.f.z.dot(to.f.z)) < 1 - EPS_ANG) return false;
    } else return false;
    if (q.k == Srf::CYLINDER) {
        V3 w = q.f.o - to.f.o;
        V3 radial = w - to.f.z * w.dot(to.f.z);
        if (radial.norm() > tol3d * 10) return false;
        double d2 = to.r2 * to.r2 - (q.r - to.r) * (q.r - to.r);
        if (d2 < 0) return true;
        double dz = std::sqrt(d2);
        add_circle(out, to, q, to.f.o + to.f.z * dz, to.f.z, q.r, tol3d);
        if (dz > tol3d)
            add_circle(out, to, q, to.f.o - to.f.z * dz, to.f.z, q.r, tol3d);
        return true;
    }
    if (q.k == Srf::SPHERE) {
        V3 w = q.f.o - to.f.o;
        double h0 = w.dot(to.f.z);
        V3 radial = w - to.f.z * h0;
        if (radial.norm() > tol3d * 10) return false;
        if (std::abs(h0) > tol3d * 10) return false; // off-plane center: walker
        // rho = (Rs^2 - r^2 + R^2) / (2R)
        double rho = (q.r * q.r - to.r2 * to.r2 + to.r * to.r) / (2 * to.r);
        double z2 = q.r * q.r - rho * rho;
        if (rho <= 0 || z2 < 0) return true;
        double z = std::sqrt(z2);
        add_circle(out, to, q, to.f.o + to.f.z * z, to.f.z, rho, tol3d);
        if (z > tol3d)
            add_circle(out, to, q, to.f.o - to.f.z * z, to.f.z, rho, tol3d);
        return true;
    }
    // CONE coaxial: rho = z tan alpha ; (rho - R)^2 + z^2 = r^2
    {
        V3 w = q.f.o - to.f.o;
        V3 radial = w - to.f.z * w.dot(to.f.z);
        if (radial.norm() > tol3d * 10) return false;
        double ta = std::tan(q.r2);
        double h0 = (to.f.o - q.f.o).dot(q.f.z); // torus center height from apex
        // rho(z) = z*ta (z from apex); torus: (rho - R)^2 + (z - h0)^2 = r^2
        double A = ta * ta + 1, Bq = -2 * (to.r * ta + h0), Cq = to.r * to.r + h0 * h0 - to.r2 * to.r2;
        double roots[2];
        int n = solve_quadratic(A, Bq, Cq, roots);
        for (int i = 0; i < n; i++) {
            double z = roots[i];
            double rho = z * ta;
            if (rho < 1e-7) continue;
            add_circle(out, to, q, q.f.o + q.f.z * z, q.f.z, rho, tol3d);
        }
        return true;
    }
}

bool torus_torus(const Srf& t1, const Srf& t2, SSIResult& out, double tol3d) {
    if (std::abs(t1.f.z.dot(t2.f.z)) < 1 - EPS_ANG) return false;
    V3 w = t2.f.o - t1.f.o;
    V3 radial = w - t1.f.z * w.dot(t1.f.z);
    if (radial.norm() > tol3d * 10) return false;
    double h = w.dot(t1.f.z);
    // coaxial tori: (rho - R1)^2 + z^2 = r1^2 ; (rho - R2)^2 + (z-h)^2 = r2^2
    // subtract: -2 rho (R1-R2) + R1^2 - R2^2 + z^2 - (z-h)^2 = r1^2 - r2^2
    // -> -2 rho dR + 2 z h - h^2 = r1^2 - r2^2 - R1^2 + R2^2
    double dR = t1.r - t2.r;
    if (std::abs(dR) < EPS_POS) {
        // then need 2 z h - h^2 = r1^2 - r2^2 -> single z plane
        if (std::abs(h) < EPS_POS) return true;
        double z = (t1.r2 * t1.r2 - t2.r2 * t2.r2 + h * h) / (2 * h);
        double r2v = t1.r2 * t1.r2 - z * z;
        if (r2v < 0) return true;
        double rho = std::sqrt(r2v);
        if (rho < 1e-7) return true;
        add_circle(out, t1, t2, t1.f.o + t1.f.z * z, t1.f.z, rho, tol3d);
        return true;
    }
    return false; // general coaxial case: quadratic in (rho, z); walker is safer
}

} // namespace

// ============================================================================
// sampling of exact curves with continuous UVs
// ============================================================================

void sample_exact(const Srf& a, const Srf& b, const Cur& c, double t0, double t1,
                  bool closed, SSIResult& out, double tol3d) {
    SecCurve sc;
    sc.has_exact = true;
    sc.exact = c;
    sc.closed = closed;
    int n;
    if (c.k == Cur::LINE) {
        n = 32; // lines need interior samples for region clipping (runs)
    } else {
        double rmax = std::max(c.r, c.k == Cur::ELLIPSE ? c.ry : c.r);
        double dt = 2.0 * std::acos(std::max(-1.0, 1.0 - tol3d / std::max(rmax, 1e-9)));
        n = std::max(8, (int)std::ceil((t1 - t0) / std::max(dt, 1e-3)));
        n = std::min(n, 4096);
    }
    double pu1 = 0, pv1 = 0, pu2 = 0, pv2 = 0;
    for (int i = 0; i <= n; i++) {
        if (closed && i == n) break; // closed: do not repeat first point
        double t = t0 + (t1 - t0) * i / n;
        SecPoint sp;
        sp.t = t;
        sp.p = c.eval(t);
        double u1, v1, u2, v2;
        if (!a.uv_of(sp.p, u1, v1) || !b.uv_of(sp.p, u2, v2)) continue;
        if (i > 0) {
            if (a.periodic_u()) u1 = unwrap_near(u1, pu1);
            if (a.periodic_v()) v1 = unwrap_near(v1, pv1);
            if (b.periodic_u()) u2 = unwrap_near(u2, pu2);
            if (b.periodic_v()) v2 = unwrap_near(v2, pv2);
        }
        sp.u1 = u1; sp.v1 = v1; sp.u2 = u2; sp.v2 = v2;
        pu1 = u1; pv1 = v1; pu2 = u2; pv2 = v2;
        sc.pts.push_back(sp);
    }
    if (sc.pts.size() >= 2) {
        sc.t0 = sc.pts.front().t;
        sc.t1 = sc.pts.back().t;
        out.curves.push_back(std::move(sc));
    }
}

// ============================================================================
// dispatch
// ============================================================================

bool ssi_exact(const Srf& a, const Srf& b, SSIResult& out, double tol3d) {
    if (same_surface(a, b, std::max(tol3d, 1e-7) * 10)) {
        out.same_domain = true;
        return true;
    }
    using K = Srf::K;
    K ka = a.k, kb = b.k;
    // canonical order for the pair switch
    auto swap_res = [&](SSIResult& r) {
        for (auto& c : r.curves)
            for (auto& p : c.pts) {
                std::swap(p.u1, p.u2);
                std::swap(p.v1, p.v2);
            }
    };
    bool done = false;
    bool swapped = false;
    K x = ka, y = kb;
    const Srf* A = &a;
    const Srf* B = &b;
    if ((int)x > (int)y) { std::swap(x, y); std::swap(A, B); swapped = true; }

    if (x == K::PLANE && y == K::PLANE) done = plane_plane(*A, *B, out, tol3d);
    else if (x == K::PLANE && y == K::CYLINDER) done = plane_cylinder(*A, *B, out, tol3d);
    else if (x == K::PLANE && y == K::CONE) done = plane_cone(*A, *B, out, tol3d);
    else if (x == K::PLANE && y == K::SPHERE) done = plane_sphere(*A, *B, out, tol3d);
    else if (x == K::PLANE && y == K::TORUS) done = plane_torus(*A, *B, out, tol3d);
    else if (x == K::CYLINDER && y == K::CYLINDER) done = cylinder_cylinder(*A, *B, out, tol3d);
    else if (x == K::CYLINDER && y == K::SPHERE) done = cylinder_sphere(*A, *B, out, tol3d);
    else if (x == K::CYLINDER && y == K::CONE) done = cylinder_cone(*A, *B, out, tol3d);
    else if (x == K::CYLINDER && y == K::TORUS) done = torus_coaxial(*B, *A, out, tol3d);
    else if (x == K::CONE && y == K::SPHERE) done = sphere_cone(*B, *A, out, tol3d);
    else if (x == K::CONE && y == K::CONE) done = cone_cone(*A, *B, out, tol3d);
    else if (x == K::CONE && y == K::TORUS) done = torus_coaxial(*B, *A, out, tol3d);
    else if (x == K::SPHERE && y == K::SPHERE) done = sphere_sphere(*A, *B, out, tol3d);
    else if (x == K::SPHERE && y == K::TORUS) done = torus_coaxial(*B, *A, out, tol3d);
    else if (x == K::TORUS && y == K::TORUS) done = torus_torus(*A, *B, out, tol3d);

    if (done && swapped) swap_res(out);
    return done;
}

} // namespace v3
