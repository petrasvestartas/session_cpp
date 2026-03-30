#include "session.h"
#include "intersection.h"
#include <fstream>
#include <iostream>

using namespace session_cpp;

int main() {
    // Two overlapping squares
    Polyline sq_a({
        Point(-1.0, -1.0, 0.0),
        Point( 1.0, -1.0, 0.0),
        Point( 1.0,  1.0, 0.0),
        Point(-1.0,  1.0, 0.0),
    });
    Polyline sq_b({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 2.0, 0.0),
        Point(0.0, 2.0, 0.0),
    });

    auto isect = Intersection::polyline_boolean(sq_a, sq_b, 0);
    auto uni   = Intersection::polyline_boolean(sq_a, sq_b, 1);
    auto diff  = Intersection::polyline_boolean(sq_a, sq_b, 2);

    std::cout << "intersection: " << isect.size() << " polygons";
    if (!isect.empty()) std::cout << ", " << isect[0].point_count() << " pts";
    std::cout << "\n";

    std::cout << "union: " << uni.size() << " polygons";
    if (!uni.empty()) std::cout << ", " << uni[0].point_count() << " pts";
    std::cout << "\n";

    std::cout << "difference: " << diff.size() << " polygons";
    if (!diff.empty()) std::cout << ", " << diff[0].point_count() << " pts";
    std::cout << "\n";

    // Write results to protobuf as Session
    auto to_session = [](const std::vector<Polyline>& polys) {
        Session s;
        for (auto& p : polys)
            s.objects.polylines->push_back(std::make_shared<Polyline>(p));
        return s;
    };
    to_session(isect).pb_dump("bool_intersect.pb");
    to_session(uni).pb_dump("bool_union.pb");
    to_session(diff).pb_dump("bool_diff.pb");

    // Print intersection polygon points
    if (!isect.empty()) {
        std::cout << "\nintersection points:\n";
        for (auto& pt : isect[0].get_points())
            std::cout << "  (" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")\n";
    }

    return 0;
}
