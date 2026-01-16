#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include "plane.h"

using namespace session_cpp;

int main() {

    std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 1.0, 0.0)
    };

    NurbsCurve curve = NurbsCurve::create(false, 2, points);
    curve.set_domain(0.0, 1.0);

    /////////////////////////////////////////////
    // Validation
    /////////////////////////////////////////////

    bool is_valid = curve.is_valid();
    std::cout << "is_valid " << is_valid << std::endl;


    // Check whole knot vector for
    // For correct size: order + cv_count - 2
    // Non-decreasing (can repeat, can't go down)
    // Valid domain exists
    bool is_valid_knot_vector = curve.is_valid_knot_vector();
    std::cout << "is_valid_knot_vector " << is_valid_knot_vector << std::endl;

    /////////////////////////////////////////////
    // Accessors
    /////////////////////////////////////////////
    // Memory layout 2-2D, 3-3D
    int dimension = curve.dimension();
    std::cout << "dimension " << dimension << std::endl;

    // Degree - Polynomial order, 1=linear, 2=quadratic, 3=cubic
    int degree = curve.degree();
    std::cout << "degree " << degree << std::endl;

    // Order = degree + 1, control points + order = knots
    int order = curve.order();
    std::cout << "order " << order << std::endl;

    // Number of control vertices
    int cv_count = curve.cv_count();
    std::cout << "cv_count " << cv_count << std::endl;

    // Number of floats per 1 control vertex
    int cv_size = curve.cv_size();
    std::cout << "cv_size " << cv_size << std::endl;

    // The knots are a list of (degree+control_points-1) numbers
    int knot_count = curve.knot_count();
    std::cout << "knot_count " << knot_count << std::endl;

    // Span = a knot interval where a single polynomial segment is evaluated
    // Knot vector: [0, 0, 0 ↑, 1 ↑, 2 ↑, 3, 3, 3]  (cubic, 5 CVs)
    int span_count = curve.span_count();
    std::cout << "span_count " << span_count << std::endl;


    /////////////////////////////////////////////////////
    // Control Vertex Access
    //  m_cv = [x0, y0, z0, (w0), x1, y1, z1, (w1), ...]
    //          └─── CV 0 ───┘    └─── CV 1 ───┘
    /////////////////////////////////////////////////////

    // Get pointer to control vertex
    // Each CV occupies m_cv_stride doubles (3 for non-rational, 4 for rational)
    // cv(index) returns pointer to m_cv[index * m_cv_stride]
    double* p = curve.cv(1);
    std::cout << "xcoord " << p[0] << " " << p[1] << " " << p[2] << std::endl;


    // Returns the control vertex as Point object
    Point cv_point = curve.get_cv(1);
    std::cout << "cv_point " << cv_point << std::endl;


    // Raw homogeneous coords
    double x, y, z, w;
    curve.get_cv_4d(1, x, y, z, w);
    std::cout << "x y z w " << x << " " << y << " " << z << " " << w << std::endl;


    // Use for regular points on curve, Polyline, B-Spline
    curve.set_cv(2, Point(2.0, 0.5, 0.0));
    std::cout << "set_cv " << curve.get_cv(2) << std::endl;


    // Use for rational curvers like circles, ellipses
    // Auto-converts to rational if w != 1.0
    curve.set_cv_4d(2, 2.0, 0.5, 1.0, 0.707);
    curve.get_cv_4d(2, x, y, z, w);
    std::cout << "set_cv_4d " << x << " " << y << " " << z << " " << w << std::endl;


    // Get weight of a control vertex (1.0 if non-rational)
    double weight = curve.weight(2);
    std::cout << "weight " << weight << std::endl;


    // Set the weight of a control vertex
    curve.set_weight(2, 0.5);
    std::cout << "weight " << curve.weight(2) << std::endl;


    /////////////////////////////////////////////////////
    // Knot Access
    /////////////////////////////////////////////////////

    // Get knot value at index
    double knot3 = curve.knot(3);
    std::cout << "knot3 " << knot3 << std::endl;


    // Set knot value at index
    bool success = curve.set_knot(4, 2);
    std::cout << "set_knot " << curve.knot(4) << std::endl;


    // Knot multiplicity = how many times the same value repeats at a given index.
    // [0, 0, 0.5, 1, 1]:
    //   Index:  0    1    2    3    4
    //   Value:  0    0   0.5   1    2
    //           └─┬─┘         
    //          mult=2        
    int m0 = curve.knot_multiplicity(0);  // 2 (two 0's)
    int m1 = curve.knot_multiplicity(1);  // 2 (still counting the 0's)
    int m2 = curve.knot_multiplicity(2);  // 1 (single 0.5)
    int m3 = curve.knot_multiplicity(3);  // 1 (single 1's)
    int m4 = curve.knot_multiplicity(4);  // 1 (single 2)
    std::cout << "knot_multiplicity " << m0 << " " << m1 << " " << m2 << " " << m3 << " " << m4 << std::endl;


    // Superflous knots are used for extension of clamped curves
    double superfluous_knot = curve.superfluous_knot(1);  // get next logical knot
    curve.insert_knot(superfluous_knot);  // extend the curve
    std::cout << "superfluous_knot " << superfluous_knot << std::endl;


    // Direct memory access to knot values, fast, read-only
    // Vector return is slower and makes a copy
    const double* knots = curve.knot_array();
    double k0 = knots[0];
    std::vector<double> knot_vector = curve.get_knots();
    std::cout << "knots " << k0 << std::endl;
    for(auto i = 0; i < knot_vector.size(); i++)
        std::cout << "knot_vector " << knot_vector[i] << std::endl;


    // Control vertex array access
    const double* cvs = curve.cv_array();
    double cx0 = cvs[0];
    std::cout << "cx0 " << cx0 << std::endl;


    /////////////////////////////////////////////////////
    // Domain & Parameterization
    /////////////////////////////////////////////////////

    // get start and end of the curve interval
    std::pair<double, double> interval = curve.domain();
    double start = interval.first;
    double end = interval.second;
    std::cout << "start " << start << " end " << end << std::endl;


    // Get start, middle and end values of the interval
    start = curve.domain_start();
    double middle = curve.domain_middle();
    end = curve.domain_end();
    std::cout << "start " << start << " middle " << middle << " end " << end << std::endl;


    // Change curve domain
    curve.set_domain(0.0, 1.0);
    std::cout << "set_domain " << curve.domain_start() << " " << curve.domain_end() << std::endl;


    // Span of distict knot intervals
    std::vector<double> intervals =  curve.get_span_vector();
    for(auto & value : intervals)
        std::cout << "intervals " << value << std::endl;


    /////////////////////////////////////////////////////
    // Geometric checks
    /////////////////////////////////////////////////////

    // Is rational is related to control points having weights
    // is_rational = false means control points [x, y, z]
    // is_rational = false means control points [xw, yw, zw]
    // Rational curves are used to represent:
    // circles, ellipses, parabolas, hyperbolas exactly
    bool is_rational = curve.is_rational();
    std::cout << "is_rational " << is_rational << std::endl;

    // Ends, periodic, linearity, planar, arc, lies on plane, open, or polyline
    bool closed = curve.is_closed();
    bool periodic = curve.is_periodic();
    bool linear = curve.is_linear();
    bool planar = curve.is_planar();
    bool arc = curve.is_arc();
    Plane plane = Plane::xy_plane();
    bool on_plane = curve.is_in_plane(plane);
    bool is_open = curve.is_natural();
    bool is_polyline = curve.is_polyline();
    std::cout << "closed " << closed << std::endl;
    std::cout << "periodic " << periodic << std::endl;
    std::cout << "linear " << linear << std::endl;
    std::cout << "planar " << planar << std::endl;
    std::cout << "arc " << arc << std::endl;
    std::cout << "on_plane " << on_plane << std::endl;
    std::cout << "is_open " << is_open << std::endl;
    std::cout << "is_polyline " << is_polyline << std::endl;



    // std::vector<Point> points = {
    //     Point(20, 3.727273, 0),
    //     Point(21.8, 7.327273, 0),
    //     Point(23.6, 3.727273, 0),
    //     Point(25.4, 3.727273, 0),
    //     Point(27.8, 3.727273, 0),
    //     Point(30.2, 3.727273, 0),
    //     Point(30.8, 6.127273, 0),
    //     Point(29, 6.727273, 0),
    //     Point(30.2, 9.727273, 0),
    //     Point(32, 7.927273, 0)
    // };

    // auto curve = NurbsCurve::create(false, 2, points);
    // std::cout << std::fixed << std::setprecision(6);

    // double params[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    // std::cout << "=== frame_at ===" << std::endl;
    // for (double t : params) {
    //     Point o; Vector tan, norm, binorm;
    //     curve.frame_at(t, true, o, tan, norm, binorm);
    //     std::cout << o << std::endl;
    // }
    // for (double t : params) {
    //     Point o; Vector tan, norm, binorm;
    //     curve.frame_at(t, true, o, tan, norm, binorm);
    //     std::cout << tan << std::endl;
    // }
    // for (double t : params) {
    //     Point o; Vector tan, norm, binorm;
    //     curve.frame_at(t, true, o, tan, norm, binorm);
    //     std::cout << norm << std::endl;
    // }
    // for (double t : params) {
    //     Point o; Vector tan, norm, binorm;
    //     curve.frame_at(t, true, o, tan, norm, binorm);
    //     std::cout << binorm << std::endl;
    // }

    // std::cout << "\n=== perpendicular_frame_at ===" << std::endl;
    // for (double t : params) {
    //     Point o; Vector x, y, z;
    //     curve.perpendicular_frame_at(t, true, o, x, y, z);
    //     std::cout << o << std::endl;
    // }
    // for (double t : params) {
    //     Point o; Vector x, y, z;
    //     curve.perpendicular_frame_at(t, true, o, x, y, z);
    //     std::cout << x << std::endl;
    // }
    // for (double t : params) {
    //     Point o; Vector x, y, z;
    //     curve.perpendicular_frame_at(t, true, o, x, y, z);
    //     std::cout << y << std::endl;
    // }
    // for (double t : params) {
    //     Point o; Vector x, y, z;
    //     curve.perpendicular_frame_at(t, true, o, x, y, z);
    //     std::cout << z << std::endl;
    // }

    // // Performance test
    // std::cout << "\n=== Performance ===" << std::endl;
    // const int iterations = 10000;

    // auto start = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < iterations; i++) {
    //     Point o; Vector t, n, b;
    //     curve.frame_at(0.5, true, o, t, n, b);
    // }
    // auto end = std::chrono::high_resolution_clock::now();
    // auto frame_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // start = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < iterations; i++) {
    //     Point o; Vector x, y, z;
    //     curve.perpendicular_frame_at(0.5, true, o, x, y, z);
    // }
    // end = std::chrono::high_resolution_clock::now();
    // auto perp_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // std::cout << "frame_at: " << frame_us / 1000.0 << " ms for " << iterations << " calls (" << frame_us * 1000.0 / iterations << " ns/call)" << std::endl;
    // std::cout << "perpendicular_frame_at: " << perp_us / 1000.0 << " ms for " << iterations << " calls (" << perp_us * 1000.0 / iterations << " ns/call)" << std::endl;
}
