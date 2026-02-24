#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "mesh.h"
#include "line.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "boundingbox.h"
#include "xform.h"
#include <iostream>

using namespace session_cpp;

int main() {
    Session session("surface_plane");
        const int sides = 6;

    std::vector<Line> lines = {
        Line::from_points(Point(3.723844, 0.971574, 0), Point(3.218847, 0.841359, 0)),
        Line::from_points(Point(3.723844, 0.971574, 0), Point(3.852993, 0.371225, 0)),
        Line::from_points(Point(3.852993, 0.371225, 0), Point(4.593264, 0.584361, 0)),
        Line::from_points(Point(3.904452, -0.157402, 0), Point(3.236395, -0.051356, 0)),
        Line::from_points(Point(4.995604, -0.149798, 0), Point(4.411839, -0.996413, 0)),
        Line::from_points(Point(3.904452, -0.157402, 0), Point(4.29633, -0.224964, 0)),
        Line::from_points(Point(4.29633, -0.224964, 0), Point(3.57903, -0.987075, 0)),
        Line::from_points(Point(3.236395, -0.051356, 0), Point(3.218847, 0.841359, 0)),
        Line::from_points(Point(4.593264, 0.584361, 0), Point(4.995604, -0.149798, 0)),
        Line::from_points(Point(4.411839, -0.996413, 0), Point(3.57903, -0.987075, 0)),
    };

    Mesh mesh = Mesh::from_lines(lines);

    std::cout << "is mesh valid: " << mesh.is_valid() << "\n";

    session.add_mesh(std::make_shared<Mesh>(mesh));

    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/mesh.pb");

    return 0;
}
