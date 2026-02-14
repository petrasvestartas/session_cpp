#include "session.h"
#include "primitives.h"
#include <iostream>
#include <filesystem>

using namespace session_cpp;

int main() {


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
    s.make_rational();


    // const - to read, non-const point to write
    const double* const_pointer_cv = s.cv(0,0);
    std::cout << const_pointer_cv[2] << std::endl; // 0
    double* pointer_cv = s.cv(0,0);
    pointer_cv[2] = 10.0; // modifies surface because it is a pointer
    std::cout << pointer_cv[2] << std::endl; // 10

    // typicial setters and getters
    Point cv = s.get_cv(0,0);
    std::cout << cv << std::endl; // 0,0,10
    double x, y, z, w;
    s.get_cv_4d(0,0, x, y, z, w);
    std::cout << x << " " << y << " " << z  << " " << w << std::endl; // 0 0 10 1

    s.set_cv(0,0, Point(0, 0, 5));
    std::cout <<  s.get_cv(0,0) << " " << w << std::endl; // 0 0 5 1
    s.set_cv_4d(0,0, 0, 0, 4, 2);
    std::cout << s.get_cv(0,0) << " " << s.weight(0,0) << std::endl; // 0 0 5 1 << why here 1 ? 
    std::cout << s.cv(0,0)[2] << std::endl; // 0 0 5 1 << why here 1 ? 


    w = s.weight(0,0);
    s.set_weight(0,0,1);
    std::cout << s.weight(0,0) << std::endl; // 1

    




    return 0;
}
