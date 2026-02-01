#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
#include "src/plane.h"
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

    // Length

    // Get point at parameter t
    Point point_at = curve.point_at(0.5);

    // Get point and derivatives at parameter t
    std::vector<Vector> derivatives = curve.evaluate(0.5, 2);

    // Tangent vector at parameter t
    Vector tangent = curve.tangent_at(0.5);

    // normalized=true (default): t in [0,1] mapped to domain
    Plane f = curve.plane_at(0.5, true);



    // Perpendicular frame at (RMF with Frenet initialization, matches Rhino)
    Plane pf = curve.perpendicular_plane_at(0.5, true);

    // Get multiple rotation minimization frames along the curve (matches Rhino)
    auto frames = curve.get_perpendicular_planes(4);
    for (const auto& frame : frames) {
        std:: cout << frame << std::endl;
    }

    return 0;
}
