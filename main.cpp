#include "nurbscurve.h"
#include "point.h"
#include "vector.h"

using namespace session_cpp;


int main() {

    std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(2.0, 0.0, 0.0)
    };

    NurbsCurve curve = NurbsCurve::create(false, 2, points);

    // Minimal and Full String Representation
    std::string cstr = curve.str();
    std::string crepr = curve.repr();

    // Copy (duplicates everything except guid)
    NurbsCurve ccopy = curve;
    NurbsCurve cother = NurbsCurve::create(false, 2, points);
        
    return 0;
}
