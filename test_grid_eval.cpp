#include "src/nurbssurface.h"
#include <iostream>
#include <iomanip>

using namespace session_cpp;

int main() {
    NurbsSurface srf(3, false, 4, 4, 5, 5);
    srf.make_clamped_uniform_knot_vector(0, 1.0);
    srf.make_clamped_uniform_knot_vector(1, 1.0);

    // Set control points
    srf.set_cv(0, 0, Point(0.0, 0.0, -2.5));
    srf.set_cv(0, 1, Point(0.0, 1.0, 0.0));
    srf.set_cv(0, 2, Point(0.0, 2.0, 0.0));
    srf.set_cv(0, 3, Point(0.0, 3.0, 0.0));
    srf.set_cv(0, 4, Point(0.0, 4.0, -2.5));

    srf.set_cv(1, 0, Point(1.0, 0.0, 0.0));
    srf.set_cv(1, 1, Point(1.0, 1.0, 0.0));
    srf.set_cv(1, 2, Point(1.0, 2.0, 5.0));
    srf.set_cv(1, 3, Point(1.0, 3.0, 0.0));
    srf.set_cv(1, 4, Point(1.0, 4.0, 0.0));

    srf.set_cv(2, 0, Point(2.0, 0.0, 0.0));
    srf.set_cv(2, 1, Point(2.0, 1.0, 0.0));
    srf.set_cv(2, 2, Point(2.0, 2.0, 0.0));
    srf.set_cv(2, 3, Point(2.0, 3.0, 0.0));
    srf.set_cv(2, 4, Point(2.0, 4.0, 0.0));

    srf.set_cv(3, 0, Point(3.0, 0.0, 0.0));
    srf.set_cv(3, 1, Point(3.0, 1.0, 0.0));
    srf.set_cv(3, 2, Point(3.0, 2.0, 0.0));
    srf.set_cv(3, 3, Point(3.0, 3.0, 0.0));
    srf.set_cv(3, 4, Point(3.0, 4.0, 0.0));

    srf.set_cv(4, 0, Point(4.0, 0.0, -2.5));
    srf.set_cv(4, 1, Point(4.0, 1.0, 0.0));
    srf.set_cv(4, 2, Point(4.0, 2.0, 0.0));
    srf.set_cv(4, 3, Point(4.0, 3.0, 0.0));
    srf.set_cv(4, 4, Point(4.0, 4.0, -2.5));

    auto [u_min, u_max] = srf.domain(0);
    auto [v_min, v_max] = srf.domain(1);

    std::cout << std::fixed << std::setprecision(3);
    
    std::cout << "First 10 points (row 0):" << std::endl;
    for (int j = 0; j < 10; j++) {
        double u = u_min + (u_max - u_min) * 0 / 9.0;
        double v = v_min + (v_max - v_min) * j / 9.0;
        Point pt = srf.point_at(u, v);
        std::cout << "(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
    }

    std::cout << "\nNext 10 points (row 1):" << std::endl;
    for (int j = 0; j < 10; j++) {
        double u = u_min + (u_max - u_min) * 1 / 9.0;
        double v = v_min + (v_max - v_min) * j / 9.0;
        Point pt = srf.point_at(u, v);
        std::cout << "(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
    }

    std::cout << "\nRow 2 points:" << std::endl;
    for (int j = 0; j < 10; j++) {
        double u = u_min + (u_max - u_min) * 2 / 9.0;
        double v = v_min + (v_max - v_min) * j / 9.0;
        Point pt = srf.point_at(u, v);
        std::cout << "(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")" << std::endl;
    }

    return 0;
}
