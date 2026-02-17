#include "session.h"
#include "primitives.h"
#include <iostream>
#include <filesystem>
#include <iomanip>

using namespace session_cpp;

int main() {

    
    Session session;


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

    auto [division_points, vectors, uvs0] = s.divide_by_count_points(3, 3);
    auto [planes, uvs1] = s.divide_by_count_planes(3, 3);

    std::cout << std::setprecision(15);
    for (size_t i = 0; i <= 3; i++) {
        for (size_t j = 0; j <= 3; j++) {
            Vector x_axis = planes[i][j].x_axis();
            Vector y_axis = planes[i][j].y_axis();
            std::cout << "MINI_CHECK(TOLERANCE.is_vector_close(planes[" << i << "][" << j << "].y_axis(), Vector(" << y_axis[0] << ", " << y_axis[1] << ", " << y_axis[2] << ")));" << std::endl;
            // std::cout << y_axis[0] << ", " << y_axis[1] << ", " << y_axis[2] << std::endl;
        }
    }




    // Serialization

    session.add_nurbssurface(std::make_shared<NurbsSurface>(s));

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "surface.pb").string();
    session.pb_dump(filepath);



    




    return 0;
}
