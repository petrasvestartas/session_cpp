#include "session.h"
#include "primitives.h"
#include <iostream>
#include <filesystem>
#include <iomanip>

using namespace session_cpp;

int main() {

    Session session;

    std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(-1.0, 0.75, 2.0),
        Point(-1.0, 4.25, 2.0),
        Point(0.0, 5.0, 0.0),
        Point(0.75, -1.0, 2.0),
        Point(1.25, 1.25, 4.0),
        Point(1.25, 3.75, 4.0),
        Point(0.75, 6.0, 2.0),
        Point(4.25, -1.0, 2.0),
        Point(3.75, 1.25, 4.0),
        Point(3.75, 3.75, 4.0),
        Point(4.25, 6.0, 2.0),
        Point(5.0, 0.0, 0.0),
        Point(6.0, 0.75, 2.0),
        Point(6.0, 4.25, 2.0),
        Point(5.0, 5.0, 0.0),
    };

    NurbsSurface s = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

    double u = 0.25, v = 0.25;
    std::cout << std::setprecision(15);

    // point_at(u, v) - returns Point
    Point p1 = s.point_at(u, v);
    std::cout << "point_at(u,v): " << p1 << std::endl;

    // normal_at(u, v) - returns Vector
    Vector n1 = s.normal_at(u, v);
    std::cout << "normal_at(u,v): " << n1 << std::endl;

    // evaluate(u, v, num_derivs) - returns vector of derivatives
    auto derivs = s.evaluate(u, v, 1);
    std::cout << "evaluate(u,v,1): point=" << derivs[0] << ", Su=" << derivs[1] << ", Sv=" << derivs[2] << std::endl;

    // point_at_corner(u_end, v_end) - corner point
    Point p_corner = s.point_at_corner(1, 1);
    std::cout << "point_at_corner(1,1): " << p_corner << std::endl;

    // Serialization
    session.add_nurbssurface(std::make_shared<NurbsSurface>(s));
    session.add_point(std::make_shared<Point>(p1));
    session.add_point(std::make_shared<Point>(p_corner));
    session.add_line(std::make_shared<Line>(Line::from_points(p1, p1 + n1)));
    session.add_line(std::make_shared<Line>(Line::from_points(p1, p1 + derivs[1])));
    session.add_line(std::make_shared<Line>(Line::from_points(p1, p1 + derivs[2])));
    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "surface.pb").string();
    session.pb_dump(filepath);

    return 0;
}
