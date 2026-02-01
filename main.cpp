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

    NurbsCurve curve = NurbsCurve::create(false, 2, points);
    std::cout << curve.point_at(curve.domain_start()) << std::endl;

    // // to_polyline_adaptive
    // std::vector<Point> adaptive_pts;
    // std::vector<double> adaptive_params;
    // std::cout << "Adaptive polyline:" << std::endl;
    // curve.to_polyline_adaptive(adaptive_pts, &adaptive_params, 0.1, 0.0, 0.0);
    // for(auto& p : adaptive_pts)
    //     std::cout << "rg.Point3d(" << p << ")," << std::endl;
    

    // // divide_by_count
    // std::vector<Point> div_pts;
    // std::vector<double> div_params;
    // std::cout << "Divide by count (10):" << std::endl;
    // curve.divide_by_count(10, div_pts, &div_params, true);
    // for(auto& p : div_pts)
    //     std::cout << "rg.Point3d(" << p << ")," << std::endl;
    


    // // divide_by_length
    // std::vector<Point> len_pts;
    // std::vector<double> len_params;
    // curve.divide_by_length(0.5, len_pts, &len_params);

    // std::cout << "Divide by length (0.5):" << std::endl;
    // for(auto& p : len_pts)
    //     std::cout << "rg.Point3d(" << p << ")," << std::endl;


    return 0;
}
