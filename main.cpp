#include "src/nurbscurve.h"
#include "src/point.h"
#include <iostream>

using namespace session_cpp;

int main() {
    std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 1.0, 0.0)
    };

    NurbsCurve curve = NurbsCurve::create(false, 2, points);

    // is_singular
    std::cout << "is_singular: " << curve.is_singular() << std::endl;

    // is_duplicate
    // NurbsCurve curve2 = NurbsCurve::create(false, 2, points);
    // std::cout << "is_duplicate (same): " << curve.is_duplicate(curve2, false) << std::endl;
    // std::vector<Point> other_pts = {Point(0,0,0), Point(5,5,0), Point(10,0,0)};
    // NurbsCurve curve3 = NurbsCurve::create(false, 2, other_pts);
    // std::cout << "is_duplicate (diff): " << curve.is_duplicate(curve3, false) << std::endl;

    // // get_next_discontinuity
    // double t_out = 0.0;
    // double t0 = curve.domain_start();
    // double t1 = curve.domain_end();
    // bool found = curve.get_next_discontinuity(2, t0, t1, t_out);
    // std::cout << "get_next_discontinuity C1: found=" << found;
    // if (found) std::cout << " t=" << t_out;
    // std::cout << std::endl;

    // is_continuous
    // double mid = (t0 + t1) * 0.5;
    // std::cout << "is_continuous C1 at mid: " << curve.is_continuous(1, mid) << std::endl;

    // // ---- trim ----
    {
        NurbsCurve c = NurbsCurve::create(false, 2, points);
        std::vector<Point> division_points;
        c.divide_by_count(20, division_points);
        std::cout << "division points before trim:" << std::endl;
        for (const auto& pt : division_points) {
            std::cout  << pt << std::endl;
        }

        
        std::cout << "\n--- trim ---" << std::endl;
        std::cout << "before: domain=[" << c.domain_start() << ", " << c.domain_end() << "]" << std::endl;
        double a = c.domain_start() + (c.domain_end() - c.domain_start()) / 3.0;
        double b = c.domain_start() + 2.0 * (c.domain_end() - c.domain_start()) / 3.0;
        c.trim(a, b);
        std::cout << "after trim(" << a << ", " << b << "): domain=[" << c.domain_start() << ", " << c.domain_end() << "]" << std::endl;
        std::cout << "start: " << c.point_at_start() << std::endl;
        std::cout << "end:   " << c.point_at_end() << std::endl;

        c.divide_by_count(20, division_points);
        std::cout << "division points after trim:" << std::endl;
        for (const auto& pt : division_points) {
            std::cout  << pt << std::endl;
        }
    }

    // // ---- increase_degree ----
    // {
    //     NurbsCurve c = NurbsCurve::create(false, 2, points);
    //     std::cout << "\n--- increase_degree ---" << std::endl;
    //     std::cout << "before: degree=" << c.degree() << " cv_count=" << c.cv_count() << std::endl;
    //     c.increase_degree(3);
    //     std::cout << "after increase_degree(3): degree=" << c.degree() << " cv_count=" << c.cv_count() << std::endl;
    //     std::cout << "start: " << c.point_at_start() << "  end: " << c.point_at_end() << std::endl;
    // }

    // // ---- change_dimension ----
    // {
    //     NurbsCurve c = NurbsCurve::create(false, 2, points);
    //     std::cout << "\n--- change_dimension ---" << std::endl;
    //     std::cout << "before: dim=" << c.dimension() << " point_at_start=" << c.point_at_start() << std::endl;
    //     c.change_dimension(2);
    //     std::cout << "after change_dimension(2): dim=" << c.dimension() << std::endl;
    // }

    // // ---- change_closed_curve_seam ----
    // {
    //     std::vector<Point> closed_pts = {
    //         Point(1.0, 0.0, 0.0),
    //         Point(0.0, 1.0, 0.0),
    //         Point(-1.0, 0.0, 0.0),
    //         Point(0.0, -1.0, 0.0)
    //     };
    //     NurbsCurve c = NurbsCurve::create(true, 2, closed_pts);
    //     std::cout << "\n--- change_closed_curve_seam ---" << std::endl;
    //     std::cout << "before: start=" << c.point_at_start() << " domain=[" << c.domain_start() << ", " << c.domain_end() << "]" << std::endl;
    //     double seam_t = c.domain_start() + (c.domain_end() - c.domain_start()) * 0.25;
    //     c.change_closed_curve_seam(seam_t);
    //     std::cout << "after seam(" << seam_t << "): start=" << c.point_at_start() << " domain=[" << c.domain_start() << ", " << c.domain_end() << "]" << std::endl;
    // }

    // // ---- reparameterize ----
    // {
    //     NurbsCurve c = NurbsCurve::create(false, 2, points);
    //     c.make_rational();
    //     std::cout << "\n--- reparameterize ---" << std::endl;
    //     std::cout << "before: domain=[" << c.domain_start() << ", " << c.domain_end() << "]" << std::endl;
    //     Point mid_before = c.point_at(c.domain_middle());
    //     std::cout << "mid point: " << mid_before << std::endl;
    //     c.reparameterize(2.0);
    //     std::cout << "after reparameterize(2.0): domain=[" << c.domain_start() << ", " << c.domain_end() << "]" << std::endl;
    //     Point mid_after = c.point_at(c.domain_middle());
    //     std::cout << "mid point: " << mid_after << std::endl;
    // }

    // // ---- make_clamped_uniform_knot_vector ----
    // {
    //     NurbsCurve c = NurbsCurve::create(false, 2, points);
    //     std::cout << "\n--- make_clamped_uniform_knot_vector ---" << std::endl;
    //     auto knots_before = c.get_knots();
    //     std::cout << "before knots:";
    //     for (auto k : knots_before) std::cout << " " << k;
    //     std::cout << std::endl;
    //     c.make_clamped_uniform_knot_vector(1.0);
    //     auto knots_after = c.get_knots();
    //     std::cout << "after knots: ";
    //     for (auto k : knots_after) std::cout << " " << k;
    //     std::cout << std::endl;
    // }

    // // ---- make_periodic_uniform_knot_vector ----
    // {
    //     std::vector<Point> closed_pts = {
    //         Point(1.0, 0.0, 0.0),
    //         Point(0.0, 1.0, 0.0),
    //         Point(-1.0, 0.0, 0.0),
    //         Point(0.0, -1.0, 0.0)
    //     };
    //     NurbsCurve c = NurbsCurve::create(true, 2, closed_pts);
    //     std::cout << "\n--- make_periodic_uniform_knot_vector ---" << std::endl;
    //     auto knots_before = c.get_knots();
    //     std::cout << "before knots:";
    //     for (auto k : knots_before) std::cout << " " << k;
    //     std::cout << std::endl;
    //     c.make_periodic_uniform_knot_vector(1.0);
    //     auto knots_after = c.get_knots();
    //     std::cout << "after knots: ";
    //     for (auto k : knots_after) std::cout << " " << k;
    //     std::cout << std::endl;
    // }

    // // ---- serialization ----
    // {
    //     NurbsCurve c = NurbsCurve::create(false, 2, points);
    //     std::cout << "\n--- serialization ---" << std::endl;

    //     // json in-memory
    //     auto j = c.jsondump();
    //     std::cout << "jsondump keys: type=" << j["type"] << " degree=" << j["degree"] << " cv_count=" << j["cv_count"] << std::endl;
    //     NurbsCurve from_j = NurbsCurve::jsonload(j);
    //     std::cout << "jsonload: degree=" << from_j.degree() << " cv_count=" << from_j.cv_count() << std::endl;

    //     // json file
    //     c.json_dump("test_curve.json");
    //     NurbsCurve from_file = NurbsCurve::json_load("test_curve.json");
    //     std::cout << "json_load from file: degree=" << from_file.degree() << " cv_count=" << from_file.cv_count() << std::endl;

    //     // protobuf in-memory
    //     std::string proto_data = c.to_protobuf();
    //     std::cout << "to_protobuf size: " << proto_data.size() << " bytes" << std::endl;
    //     NurbsCurve from_proto = NurbsCurve::from_protobuf(proto_data);
    //     std::cout << "from_protobuf: degree=" << from_proto.degree() << " cv_count=" << from_proto.cv_count() << std::endl;

    //     // protobuf file
    //     c.protobuf_dump("test_curve.pb");
    //     NurbsCurve from_pb_file = NurbsCurve::protobuf_load("test_curve.pb");
    //     std::cout << "protobuf_load from file: degree=" << from_pb_file.degree() << " cv_count=" << from_pb_file.cv_count() << std::endl;
    // }

    return 0;
}
