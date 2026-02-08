#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "mesh.h"
#include <fstream>
#include <cmath>

using namespace session_cpp;

int main() {
    Session session("nurbs_meshing");

    // Closed loft
    NurbsCurve c1 = Primitives::circle(0, 0, 0.0, 2.0);
    NurbsCurve c2 = Primitives::circle(0, 0, 2.0, 1.0);
    NurbsCurve c3 = Primitives::circle(0, 0, 4.0, 1.5);
    NurbsCurve c4 = Primitives::circle(0, 0, 6.0, 0.8);

    auto closed_srf = std::make_shared<NurbsSurface>(NurbsSurface::create_loft({c1, c2, c3, c4}, 3));
    closed_srf->mesh(5);
    session.add_surface(closed_srf);

    // Open loft
    std::vector<std::vector<Point>> points = {
        {Point(10, -12, 0), Point(10, -10, 3), Point(10, -7, 3), Point(10, -5, 0)},
        {Point(5.5, -12, 3.5), Point(5.5, -10.0, 1.5), Point(5.5, -7.0, 1.5), Point(5.5, -5, 3.5)},
        {Point(1, -12, -0), Point(1, -10, 3.0), Point(1, -7, 3.0), Point(1, -5, -0)},
    };

    std::vector<NurbsCurve> curves = {
        NurbsCurve::create(false, 3, points[0]),
        NurbsCurve::create(false, 3, points[1]),
        NurbsCurve::create(false, 3, points[2])
    };

    auto open_srf = std::make_shared<NurbsSurface>(NurbsSurface::create_loft(curves, 3));
    open_srf->mesh(5);
    session.add_surface(open_srf);

    std::cout << open_srf->str() << std::endl;
    std::cout << open_srf->repr() << std::endl;

    session.json_dump("C:/pc/3_code/code_rust/session/session_data/nurbs_meshing.json");      
    return 0;
}
