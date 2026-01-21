#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
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

    std::vector<double> params = {0.0, 0.25, 0.5, 0.75, 1.0};
    auto frames = curve.get_perpendicular_frames(params);

    for (size_t i = 0; i < params.size(); ++i) {
        const auto& [o, t, n, b] = frames[i];
        std::cout << o << "\n";
        std::cout << t << "\n";
        std::cout << n << "\n";
        std::cout << b << "\n";
    }



    return 0;
}
