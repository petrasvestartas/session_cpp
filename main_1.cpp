#include "session.h"
#include "intersection.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "primitives.h"
#include <filesystem>
#include <cmath>
#include <iostream>

using namespace session_cpp;

static NurbsSurface plane_rect(const Plane& pl, double half) {
    Point o = pl.origin();
    Vector x = pl.x_axis(), y = pl.y_axis();
    std::vector<Point> cp = {
        Point(o[0] - half*x[0] - half*y[0], o[1] - half*x[1] - half*y[1], o[2] - half*x[2] - half*y[2]),
        Point(o[0] + half*x[0] - half*y[0], o[1] + half*x[1] - half*y[1], o[2] + half*x[2] - half*y[2]),
        Point(o[0] - half*x[0] + half*y[0], o[1] - half*x[1] + half*y[1], o[2] - half*x[2] + half*y[2]),
        Point(o[0] + half*x[0] + half*y[0], o[1] + half*x[1] + half*y[1], o[2] + half*x[2] + half*y[2]),
    };
    return NurbsSurface::create(false, false, 1, 1, 2, 2, cp);
}

static void add_cuts(Session& s, const NurbsSurface& srf, const Plane& plane, Color col, double half) {
    auto curves = Intersection::surface_plane(srf, plane);
    auto [d0, d1] = srf.domain(0);
    auto [e0, e1] = srf.domain(1);
    std::cout << "plane O(" << plane.origin() << ") N(" << plane.z_axis()
              << ") closed_u=" << srf.is_closed(0) << " closed_v=" << srf.is_closed(1)
              << " domain=[" << d0 << "," << d1 << "]x[" << e0 << "," << e1 << "]"
              << " => " << curves.size() << " curves\n";
    for (size_t i = 0; i < curves.size(); i++) {
        auto& c = curves[i];
        auto [t0, t1] = c.domain();
        Point p0 = c.point_at(t0), p1 = c.point_at(t1);
        double gap = p0.distance(p1);
        std::cout << "  [" << i << "] deg=" << c.degree() << " cvs=" << c.cv_count()
                  << " rational=" << c.is_rational()
                  << " closed=" << c.is_closed()
                  << " gap=" << gap << "\n";
        c.linecolors = {col};
        s.add_nurbscurve(std::make_shared<NurbsCurve>(c));
    }
    NurbsSurface rect = plane_rect(plane, half);
    rect.facecolors = {col};
    s.add_nurbssurface(std::make_shared<NurbsSurface>(rect));
}

int main() {
    Session session("surface_plane");

    // Torus — horizontal cuts + tilted + steep
    double R = 6, r = 2;
    NurbsSurface torus = Primitives::torus_surface(0, 0, 0, R, r);
    session.add_nurbssurface(std::make_shared<NurbsSurface>(torus));
    // Horizontal cuts at various z
    double zz[] = {-1.0, 0.0, 1.0};
    Color cc[] = {Color::red(), Color::green(), Color::blue()};
    for (int i = 0; i < 3; i++) {
        Point pp(0, 0, zz[i]); Vector pn(0, 0, 1);
        add_cuts(session, torus, Plane::from_point_normal(pp, pn), cc[i], 10);
    }
    // Tilted
    Point tp(0, 0, 0); Vector tn(0.3, 0.2, 1);
    add_cuts(session, torus, Plane::from_point_normal(tp, tn), Color::orange(), 10);
    // Steep — vertical cut through center
    Point sp(0, 0, 0); Vector sn(1, 0, 0);
    add_cuts(session, torus, Plane::from_point_normal(sp, sn), Color::magenta(), 10);

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "surface_plane.pb").string();
    session.pb_dump(filepath);

    return 0;
}
