#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "mesh.h"
#include <fstream>
#include <cmath>

using namespace session_cpp;

int main() {
    std::vector<Point> pts_south = {Point(1, 20.569076, 0), Point(1, 22.569076, 3.0), Point(1, 25.569076, 3.0), Point(1, 27.569076, 0)};
    std::vector<Point> pts_west  = {Point(10, 20.569076, 0), Point(5.5, 20.569076, 3.5), Point(1, 20.569076, 0)};
    std::vector<Point> pts_north = {Point(10, 20.569076, 0), Point(10, 22.569076, 3), Point(10, 25.569076, 3), Point(10, 27.569076, 0)};
    std::vector<Point> pts_east  = {Point(10, 27.569076, 0), Point(5.5, 27.569076, 3.5), Point(1, 27.569076, 0)};

    NurbsCurve south = NurbsCurve::create(false, 3, pts_south);
    NurbsCurve west  = NurbsCurve::create(false, 2, pts_west);
    NurbsCurve north = NurbsCurve::create(false, 3, pts_north);
    NurbsCurve east  = NurbsCurve::create(false, 2, pts_east);

    NurbsSurface surf = NurbsSurface::create_edge(south, west, north, east);
    Mesh m = surf.mesh(15);

    Session session("nurbs_meshing");
    session.add_surface(std::make_shared<NurbsSurface>(surf));
    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/nurbs_edge.pb");
    return 0;
}
