#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "mesh.h"

using namespace session_cpp;

int main() {
    auto uc0 = NurbsCurve::create(false, 2, {Point(10,9.569076,0), Point(5.5,9.569076,3.5), Point(1,9.569076,0)});
    auto uc1 = NurbsCurve::create(false, 2, {Point(10,16.569076,0), Point(5.5,16.569076,3.5), Point(1,16.569076,0)});
    auto vc0 = NurbsCurve::create(false, 3, {Point(1,9.569076,0), Point(1,11.569076,3.0), Point(1,14.569076,3.0), Point(1,16.569076,0)});
    auto vc1 = NurbsCurve::create(false, 2, {Point(4.236484,9.569076,1.612033), Point(3,13.069076,4.250144), Point(3.667141,16.569076,1.459684)});
    auto vc2 = NurbsCurve::create(false, 2, {Point(7.295129,16.569076,1.471513), Point(8,13.069076,4.250144), Point(6.99265,9.569076,1.557456)});
    auto vc3 = NurbsCurve::create(false, 3, {Point(10,9.569076,0), Point(10,11.569076,3), Point(10,14.569076,3), Point(10,16.569076,0)});

    NurbsSurface srf = NurbsSurface::create_network({uc0, uc1}, {vc0, vc1, vc2, vc3});
    Mesh m = srf.mesh();

    Session session("create_network");
    session.add_surface(std::make_shared<NurbsSurface>(srf));
    session.add_mesh(std::make_shared<Mesh>(m));
    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/create_network.pb");

    return 0;
}
