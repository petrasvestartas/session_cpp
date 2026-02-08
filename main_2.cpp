#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "mesh.h"
#include <fstream>
#include <cmath>

using namespace session_cpp;

int main() {
    std::cout << "main_2.cpp" << std::endl;

    std::vector<Point> points = {
            // i=0
            Point(0.0, 0.0, 0.0),
            Point(-1.0, 0.75, 2.0),
            Point(-1.0, 4.25, 2.0),
            Point(0.0, 5.0, 0.0),
            // i=1
            Point(0.75, -1.0, 2.0),
            Point(1.25, 1.25, 4.0),
            Point(1.25, 3.75, 4.0),
            Point(0.75, 6.0, 2.0),
            // i=2
            Point(4.25, -1.0, 2.0),
            Point(3.75, 1.25, 4.0),
            Point(3.75, 3.75, 4.0),
            Point(4.25, 6.0, 2.0),
            // i=3
            Point(5.0, 0.0, 0.0),
            Point(6.0, 0.75, 2.0),
            Point(6.0, 4.25, 2.0),
            Point(5.0, 5.0, 0.0),
        };

    NurbsSurface s = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

    // Minimal and Full String Representation
    std::string sstr = s.str();
    std::string srepr = s.repr();

    // Get mesh
    Mesh m = s.mesh();

    // Copy (duplicates everything except guid)
    NurbsSurface scopy = s;
    NurbsSurface sother = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

    // Point division matching Rhino's 4x6 grid
    auto [v, uv] = s.divide_by_count(4, 6);


    Session session("nurbs_meshing_2");
    session.add_surface(std::make_shared<NurbsSurface>(scopy));
    session.json_dump("C:/pc/3_code/code_rust/session/session_data/nurbs_meshing_2.json");    


    return 0;
}
