#include "session.h"
#include "primitives.h"
#include <iostream>
#include <filesystem>

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

    // Get domain 0 - 1
    std::pair<double, double> domain_u = s.domain(0);
    std::pair<double, double> domain_v = s.domain(1);

    std::cout << domain_u.first << std::endl; 
    std::cout << domain_u.second << std::endl; 

    // Set Domain
    bool is_set_u = s.set_domain(0, -1.1, 2.3);
    bool is_set_v = s.set_domain(1, -5.1, 1.3);
    std::cout << (s.domain(1).first == -5.1) << std::endl; 
    std::cout << (s.domain(1).second == 1.3) << " result " << s.domain(1).second  << std::endl; 

    // Get sorted list of distinct knot values
    std::vector<double> span_vector = s.get_span_vector(0);
    double first_item = span_vector.front();
    double last_item = span_vector.back();




    // Serialization

    session.add_nurbssurface(std::make_shared<NurbsSurface>(s));

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "surface.pb").string();
    session.pb_dump(filepath);



    




    return 0;
}
