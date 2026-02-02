#include "src/nurbscurve.h"
#include "src/nurbssurface.h"
#include "src/point.h"
#include "src/vector.h"
#include "src/plane.h"
#include <iostream>
#include <iomanip>

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

    NurbsSurface surface = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

    // Minimal and Full String Representation
    std::string sstr = surface.str();
    std::string srepr = surface.repr();

    // Copy (duplicates everything except guid)
    NurbsSurface scopy = surface;
    NurbsSurface sother = NurbsSurface::create(false, false, 3, 3, 4, 4, points);

    // Point division
    auto [divided, _] = surface.divide_by_count(5,6);

    for (const auto& row : divided)
        
        for (const auto& pt : row)
            std::cout << pt << std::endl;

    return 0;
}
