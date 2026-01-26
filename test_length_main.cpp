#include <iostream>
#include <iomanip>
#include "src/nurbscurve.h"
int main() {
    std::vector<session::Point> pts = {
        {1.957614, 1.140253, -0.191281}, {0.912252, 1.886721, 0},
        {3.089381, 2.701879, -0.696251}, {5.015145, 1.189141, 0.35799},
        {1.854155, 0.514663, 0.347694}, {3.309532, 1.328666, 0},
        {3.544072, 2.194233, 0.696217}, {2.903513, 2.091287, 0.696217},
        {2.752484, 1.45432, 0}, {2.406227, 1.288248, 0}, {2.15032, 1.868606, 0}
    };
    auto c = session::NurbsCurve::create(false, 2, pts);
    std::cout << std::fixed << std::setprecision(10) << "C++:    " << c.length() << std::endl;
}
