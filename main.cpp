#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
#include <iostream>
#include <iomanip>

using namespace session_cpp;

int main() {
    std::vector<Point> points = {
        Point(1.957614, 1.140253, -0.191281),
        Point(0.912252, 1.886721, 0),
        Point(3.089381, 2.701879, -0.696251),
        Point(5.015145, 1.189141, 0.35799),
        Point(1.854155, 0.514663, 0.347694),
        Point(3.309532, 1.328666, 0),
        Point(3.544072, 2.194233, 0.696217),
        Point(2.903513, 2.091287, 0.696217),
        Point(2.752484, 1.45432, 0),
        Point(2.406227, 1.288248, 0),
        Point(2.15032, 1.868606, 0)
    };

    auto curve = NurbsCurve::create(false, 2, points);

    std::cout << std::fixed << std::setprecision(6);
    Point o; Vector x, y, z;

    std::cout << "=== perpendicular_frame_at comparison with Rhino ===\n" << std::endl;

    curve.perpendicular_frame_at(0.0, true, o, x, y, z);
    std::cout << "At t=0.0:" << std::endl;
    std::cout << "  Mine:  origin=" << o << std::endl;
    std::cout << "         xaxis=" << x << std::endl;
    std::cout << "         yaxis=" << y << std::endl;
    std::cout << "         zaxis=" << z << std::endl;
    std::cout << "  Rhino: origin={1.957614, 1.140253, -0.191281}" << std::endl;
    std::cout << "         xaxis={0.532768, 0.809399, -0.247046}" << std::endl;
    std::cout << "         yaxis={-0.261214, -0.120387, -0.957744}" << std::endl;
    std::cout << "         zaxis={-0.804938, 0.574787, 0.147288}" << std::endl;

    std::cout << std::endl;
    curve.perpendicular_frame_at(0.5, true, o, x, y, z);
    std::cout << "At t=0.5:" << std::endl;
    std::cout << "  Mine:  origin=" << o << std::endl;
    std::cout << "         xaxis=" << x << std::endl;
    std::cout << "         yaxis=" << y << std::endl;
    std::cout << "         zaxis=" << z << std::endl;
    std::cout << "  Rhino: origin={3.156927, 1.335111, 0.130489}" << std::endl;
    std::cout << "         xaxis={0.632708, -0.703687, 0.323272}" << std::endl;
    std::cout << "         yaxis={0.327335, -0.135297, -0.935172}" << std::endl;
    std::cout << "         zaxis={0.701806, 0.697509, 0.144738}" << std::endl;

    std::cout << std::endl;
    curve.perpendicular_frame_at(1.0, true, o, x, y, z);
    std::cout << "At t=1.0:" << std::endl;
    std::cout << "  Mine:  origin=" << o << std::endl;
    std::cout << "         xaxis=" << x << std::endl;
    std::cout << "         yaxis=" << y << std::endl;
    std::cout << "         zaxis=" << z << std::endl;
    std::cout << "  Rhino: origin={2.15032, 1.868606, 0}" << std::endl;
    std::cout << "         xaxis={0.183308, 0.080829, 0.979727}" << std::endl;
    std::cout << "         yaxis={0.896446, 0.395285, -0.200338}" << std::endl;
    std::cout << "         zaxis={-0.403464, 0.914995, 0}" << std::endl;

    return 0;
}
