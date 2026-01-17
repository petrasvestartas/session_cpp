#include "mini_test.h"
#include "nurbscurve.h"
#include "point.h"
#include "plane.h"
#include "vector.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("NurbsCurve", "constructor") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        // The first the curve is closed or open
        // For linear curves use degree 1
        // When 3 points use degree 2 curve, Rhino default
        // When x>3 points use degree 3 curve
        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        // Minimal and Full String Representation
        std::string cstr = curve.str();
        std::string crepr = curve.repr();

        // Copy (duplicates everything except guid)
        NurbsCurve ccopy = curve;
        NurbsCurve cother = NurbsCurve::create(false, 2, points);

        // Point division
        std::vector<Point> divided;
        curve.divide_by_count(10, divided);

        MINI_CHECK(curve.is_valid() == true);
        MINI_CHECK(curve.cv_count() == 4);
        MINI_CHECK(curve.degree() == 2);
        MINI_CHECK(curve.order() == 3);
        MINI_CHECK(curve.name == "my_nurbscurve");
        MINI_CHECK(!curve.guid.empty());
        MINI_CHECK(cstr == "degree=2, cvs=4");
        MINI_CHECK(crepr == "NurbsCurve(my_nurbscurve, dim=3, order=3, cvs=4, rational=false)");
        MINI_CHECK(ccopy.cv_count() == curve.cv_count());
        MINI_CHECK(ccopy.guid != curve.guid);
    }

    MINI_TEST("NurbsCurve", "attributes") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "plane.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        /////////////////////////////////////////////
        // Validation
        /////////////////////////////////////////////

        // Whole curve
        bool is_valid = curve.is_valid();
        MINI_CHECK(is_valid == true);

        // Check whole knot vector for
        // For correct size: order + cv_count - 2
        // Non-decreasing (can repeat, can't go down)
        // Valid domain exists
        bool is_valid_knot_vector = curve.is_valid_knot_vector();
        MINI_CHECK(is_valid_knot_vector == true);

        /////////////////////////////////////////////
        // Accessors
        /////////////////////////////////////////////
        // Memory layout 2-2D, 3-3D
        int dimension = curve.dimension();
        MINI_CHECK(dimension == 3);
        // Degree - Polynomial order, 1=linear, 2=quadratic, 3=cubic
        int degree = curve.degree();
        MINI_CHECK(degree == 2);
        // Is rational is related to control points having weights
        // is_rational = false means control points [x, y, z]
        // is_rational = false means control points [xw, yw, zw]
        // Rational curves are used to represent:
        // Order = degree + 1, control points + order = knots
        int order = curve.order();
        MINI_CHECK(order == 3);
        // Number of control vertices
        int cv_count = curve.cv_count();
        MINI_CHECK(cv_count == 4);
        // Number of floats per 1 control vertex
        int cv_size = curve.cv_size();
        MINI_CHECK(cv_size == 3);
        // The knots are a list of (degree+control_points-1) numbers
        int knot_count = curve.knot_count();
        MINI_CHECK(knot_count == 5);
        // Span = a knot interval where a single polynomial segment is evaluated
        // Knot vector: [0, 0, 0 ↑, 1 ↑, 2 ↑, 3, 3, 3]  (cubic, 5 CVs)
        int span_count = curve.span_count();
        MINI_CHECK(span_count == 2);
        /////////////////////////////////////////////////////
        // Control Vertex Access
        //  m_cv = [x0, y0, z0, (w0), x1, y1, z1, (w1), ...]
        //          └─── CV 0 ───┘    └─── CV 1 ───┘
        /////////////////////////////////////////////////////

        // Get pointer to control vertex
        // Each CV occupies m_cv_stride doubles:
        // (3 for non-rational, 4 for rational)
        // cv(index) returns pointer to m_cv[index * m_cv_stride]
        double* p = curve.cv(1);
        MINI_CHECK(p[0] == 1.0 && p[1] == 1.0 && p[2] == 0.0);

        // Returns the control vertex as Point object
        Point cv_point = curve.get_cv(1);
        MINI_CHECK(cv_point == Point(1.0, 1.0, 0.0));

        // Raw homogeneous coords
        double x, y, z, w;
        curve.get_cv_4d(1, x, y, z, w);
        MINI_CHECK(x == 1.0 && y == 1.0 && z == 0.0 && w == 1.0);

        // Use for regular points on curve, Polyline, B-Spline
        curve.set_cv(2, Point(2.0, 0.0, 0.5));
        MINI_CHECK(curve.get_cv(2)[0] == 2.0 && curve.get_cv(2)[1] == 0.0 && curve.get_cv(2)[2] == 0.5);

        // Use for rational curvers like circles, ellipses
        curve.set_cv_4d(2, 2.0, 0.0, 0.5, 0.707);
        curve.get_cv_4d(2, x, y, z, w);
        MINI_CHECK(x == 2.0 && y == 0.0 && z == 0.5 && w == 0.707);

        // Get weight of a control vertex (1.0 if non-rational)
        double weight = curve.weight(2);
        MINI_CHECK(weight == 0.707);

        // Set the weight of a control vertex
        curve.set_weight(2, 0.5);
        MINI_CHECK(curve.weight(2) == 0.5);

        /////////////////////////////////////////////////////
        // Knot Access
        /////////////////////////////////////////////////////

        // Get knot value at index
        double knot3 = curve.knot(3);
        MINI_CHECK(knot3 == 2);

        // Set knot value at index
        // ATTENTION you can brake increasing rule
        curve.set_knot(4, 2);
        MINI_CHECK(curve.knot(4) == 2);

        // Count repeated knots at index [0, 0, 1, 1, 2]
        int m0 = curve.knot_multiplicity(0);  // 2 (two 0's)
        int m1 = curve.knot_multiplicity(1);  // 2 (still counting the 0's)
        int m2 = curve.knot_multiplicity(2);  // 1 (single 0.5)
        int m3 = curve.knot_multiplicity(3);  // 2 (single 1's)
        int m4 = curve.knot_multiplicity(4);  // 2 (single 2)
        MINI_CHECK(m0 == 2);
        MINI_CHECK(m1 == 2);
        MINI_CHECK(m2 == 1);
        MINI_CHECK(m3 == 2);
        MINI_CHECK(m4 == 2);

        // Superflous knots are used for extension of clamped curves
        // For knot vector [0, 0, 0.5, 1, 2]: 2*knot[4] - knot[1] = 2*2 - 0 = 4
        double superfluous_knot = curve.superfluous_knot(1);
        MINI_CHECK(superfluous_knot == 4);

        // Direct memory access to knot values, fast, read-only
        // Vector return is slower and makes a copy
        const double* knots = curve.knot_array();
        double k0 = knots[0];
        std::vector<double> knot_vector = curve.get_knots();
        MINI_CHECK(k0 == 0.0);
        MINI_CHECK(knot_vector[0] == 0.0 && knot_vector[1] == 0.0 &&
                   knot_vector[2] == 1.0 && knot_vector[3] == 2.0 &&
                   knot_vector[4] == 2.0);

        // Control vertex array access
        const double* cvs = curve.cv_array();
        double cx0 = cvs[0];
        MINI_CHECK(cx0 == 0.0);

        /////////////////////////////////////////////////////
        // Domain & Parameterization - HERE
        /////////////////////////////////////////////////////

        // get start and end of the curve interval
        std::pair<double, double> interval = curve.domain();
        double start = interval.first;
        double end = interval.second;
        MINI_CHECK(start == 0.0 && end == 2.0);

        // Get start, middle and end values of the interval
        start = curve.domain_start();
        double middle = curve.domain_middle();
        end = curve.domain_end();
        MINI_CHECK(start == 0.0 && middle == 1.0 && end == 2.0);

        // Change curve domain
        curve.set_domain(0.0, 1.0);
        MINI_CHECK(curve.domain_start() == 0.0 && curve.domain_middle() == 0.5 && curve.domain_end() == 1.0);

        // Span of distict knot intervals
        std::vector<double> intervals =  curve.get_span_vector();
        MINI_CHECK(intervals[0] == 0.0 && intervals[1] == 0.5 && intervals[2] == 1.0);

        /////////////////////////////////////////////////////
        // Geometric checks
        /////////////////////////////////////////////////////
        // Is rational is related to control points having weights
        // is_rational = false means control points [x, y, z]
        // is_rational = false means control points [xw, yw, zw]
        // Rational curves are used to represent:
        // circles, ellipses, parabolas, hyperbolas exactly
        bool is_rational = curve.is_rational();
        MINI_CHECK(is_rational == true);

        // circles, ellipses, parabolas, hyperbolas exactly
        bool closed = curve.is_closed();
        bool periodic = curve.is_periodic();
        bool linear = curve.is_linear();
        bool planar = curve.is_planar();
        bool arc = curve.is_arc();
        Plane plane = Plane::xy_plane();
        bool on_plane = curve.is_in_plane(plane);
        bool is_open = curve.is_natural();
        bool is_polyline = curve.is_polyline();

        MINI_CHECK(closed == false);
        MINI_CHECK(periodic == false);
        MINI_CHECK(linear == false);
        MINI_CHECK(planar == false);
        MINI_CHECK(arc == false);
        MINI_CHECK(on_plane == false);
        MINI_CHECK(is_open == true);
        MINI_CHECK(is_polyline == false);
    }

    MINI_TEST("NurbsCurve", "Conversions") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        // to_polyline_adaptive
        std::vector<Point> adaptive_pts;
        std::vector<double> adaptive_params;
        curve.to_polyline_adaptive(adaptive_pts, &adaptive_params, 0.1, 0.0, 0.0);

        MINI_CHECK(adaptive_pts.size() == 27);
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[0], Point(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[1], Point(0.183105468750000, 0.348632812500000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[2], Point(0.357421875000000, 0.644531250000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[3], Point(0.679687500000000, 1.078125000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[4], Point(0.966796875000000, 1.300781250000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[5], Point(1.097167968750000, 1.333007812500000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[6], Point(1.159057617187500, 1.329345703125000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[7], Point(1.218750000000000, 1.312500000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[8], Point(1.331542968750000, 1.239257812500000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[9], Point(1.435546875000000, 1.113281250000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[10], Point(1.625000000000000, 0.781250000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[11], Point(1.812500000000000, 0.570312500000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[12], Point(1.906250000000000, 0.517578125000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[13], Point(2.000000000000000, 0.500000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[14], Point(2.093750000000000, 0.517578125000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[15], Point(2.187500000000000, 0.570312500000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[16], Point(2.375000000000000, 0.781250000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[17], Point(2.564453125000000, 1.113281250000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[18], Point(2.668457031250000, 1.239257812500000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[19], Point(2.781250000000000, 1.312500000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[20], Point(2.840942382812500, 1.329345703125000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[21], Point(2.902832031250000, 1.333007812500000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[22], Point(3.033203125000000, 1.300781250000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[23], Point(3.320312500000000, 1.078125000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[24], Point(3.642578125000000, 0.644531250000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[25], Point(3.816894531250000, 0.348632812500000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[26], Point(4.000000000000000, 0.000000000000000, 0.000000000000000)));

        // divide_by_count
        std::vector<Point> div_pts;
        std::vector<double> div_params;
        curve.divide_by_count(10, div_pts, &div_params, true);

        MINI_CHECK(div_pts.size() == 10);
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[0], Point(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[1], Point(0.328571015882635, 0.598213506310667, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[2], Point(0.740744941524856, 1.140321234797829, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[3], Point(1.338523997492639, 1.232716041998164, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[4], Point(1.712929663130383, 0.664818756620870, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[5], Point(2.287070327006695, 0.664818745295462, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[6], Point(2.661475993133979, 1.232716033043460, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[7], Point(3.259255052521522, 1.140321240507253, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[8], Point(3.671428981912368, 0.598213509892612, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(div_pts[9], Point(4.000000000000000, 0.000000000000000, 0.000000000000000)));

        // divide_by_length
        std::vector<Point> len_pts;
        std::vector<double> len_params;
        curve.divide_by_length(0.5, len_pts, &len_params);

        MINI_CHECK(len_pts.size() == 13);
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[0], Point(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[1], Point(0.235272731384047, 0.441110443734231, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[2], Point(0.504276692145966, 0.862299318703470, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[3], Point(0.843085062978891, 1.227533014827472, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[4], Point(1.302050970444518, 1.264156212040698, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[5], Point(1.579813544869556, 0.853113314150178, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[6], Point(1.928691287815458, 0.510169864866836, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[7], Point(2.340857741884085, 0.732368000404634, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[8], Point(2.597735401548903, 1.160594587288875, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[9], Point(3.032790392631424, 1.300960469420597, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[10], Point(3.407806728972739, 0.976991467650206, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[11], Point(3.691337413616094, 0.565615072909225, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[12], Point(3.934494402948975, 0.128829830906625, 0.000000000000000)));
    }

    MINI_TEST("NurbsCurve", "frame_at") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

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

        // normalized=true (default): t in [0,1] mapped to domain
        Point o;
        Vector t, n, b;
        curve.frame_at(0.5, true, o, t, n, b);

        MINI_CHECK(TOLERANCE.is_close(o[0], 3.156927375000000) && TOLERANCE.is_close(o[1], 1.335111500000000) && TOLERANCE.is_close(o[2], 0.130488875000000));
        MINI_CHECK(TOLERANCE.is_close(t[0], 0.701806140304030) && TOLERANCE.is_close(t[1], 0.697509131556264) && TOLERANCE.is_close(t[2], 0.144738221721788));
        MINI_CHECK(TOLERANCE.is_close(n[0], -0.513930504714161) && TOLERANCE.is_close(n[1], 0.355053088776962) && TOLERANCE.is_close(n[2], 0.780905077761815));
        MINI_CHECK(TOLERANCE.is_close(b[0], 0.493298669931115) && TOLERANCE.is_close(b[1], -0.622429365908747) && TOLERANCE.is_close(b[2], 0.607649657861031));

        MINI_CHECK(curve.frame_at(-0.1, true, o, t, n, b) == false);
        MINI_CHECK(curve.frame_at(1.1, true, o, t, n, b) == false);
        MINI_CHECK(curve.frame_at(curve.domain_start(), false, o, t, n, b) == true);
        MINI_CHECK(curve.frame_at(curve.domain_end(), false, o, t, n, b) == true);
        MINI_CHECK(curve.frame_at(curve.domain_start() - 0.1, false, o, t, n, b) == false);
    }

    MINI_TEST("NurbsCurve", "perpendicular_frame_at") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

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

        Point o;
        Vector t, n, b;
        curve.perpendicular_frame_at(0.5, true, o, t, n, b);


        MINI_CHECK(TOLERANCE.is_close(o[0], 3.156927375000000) && TOLERANCE.is_close(o[1], 1.335111500000000) && TOLERANCE.is_close(o[2], 0.130488875000000));
        MINI_CHECK(TOLERANCE.is_close(t[0], -0.530889276962602) && TOLERANCE.is_close(t[1], 0.647586483405068) && TOLERANCE.is_close(t[2], -0.546615332859574));
        MINI_CHECK(TOLERANCE.is_close(n[0], -0.474999702128807) && TOLERANCE.is_close(n[1], 0.306778027114924) && TOLERANCE.is_close(n[2], 0.824780288960047));
        MINI_CHECK(TOLERANCE.is_close(b[0], 0.701806140314880) && TOLERANCE.is_close(b[1], 0.697509131546342) && TOLERANCE.is_close(b[2], 0.144738221716994));
        MINI_CHECK(curve.perpendicular_frame_at(-0.1, true, o, t, n, b) == false);
        MINI_CHECK(curve.perpendicular_frame_at(1.1, true, o, t, n, b) == false);
        MINI_CHECK(curve.perpendicular_frame_at(curve.domain_start(), false, o, t, n, b) == true);
        MINI_CHECK(curve.perpendicular_frame_at(curve.domain_end(), false, o, t, n, b) == true);
        MINI_CHECK(curve.perpendicular_frame_at(curve.domain_start() - 0.1, false, o, t, n, b) == false);
        
    }

    MINI_TEST("NurbsCurve", "is_valid") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve_0 = NurbsCurve::create(false, 2, points);
        NurbsCurve curve_1;
        bool is_valid_0 = curve_0.is_valid();
        bool is_valid_1 = curve_1.is_valid();

        MINI_CHECK(is_valid_0 == true);
        MINI_CHECK(is_valid_1 == false);
    }

    MINI_TEST("NurbsCurve", "control vertices") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include <vector>

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        // Get all knots
        for (int i = 0; i < curve.cv_count(); i++) {
            //double k = curve.knot(i);
        }

        Point cv0 = curve.get_cv(0);
        Point cv1 = curve.get_cv(1);
        Point cv2 = curve.get_cv(2);

        MINI_CHECK(std::abs(cv0[0] - 0.0) < 0.01);
        MINI_CHECK(std::abs(cv1[0] - 1.0) < 0.01);
        MINI_CHECK(std::abs(cv2[0] - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "set_cv") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        curve.set_cv(1, Point(1.5, 2.0, 0.0));
        Point cv1 = curve.get_cv(1);

        MINI_CHECK(std::abs(cv1[0] - 1.5) < 0.01);
        MINI_CHECK(std::abs(cv1[1] - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "point_at") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        Point pt_mid = curve.point_at(t_mid);

        MINI_CHECK(pt_mid[0] > 0.0);
        MINI_CHECK(pt_mid[0] < 2.0);
    }

    MINI_TEST("NurbsCurve", "point_at_start") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        Point pt_start = curve.point_at_start();

        MINI_CHECK(std::abs(pt_start[0] - 0.0) < 0.01);
        MINI_CHECK(std::abs(pt_start[1] - 0.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "point_at_end") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        Point pt_end = curve.point_at_end();

        MINI_CHECK(std::abs(pt_end[0] - 2.0) < 0.01);
        MINI_CHECK(std::abs(pt_end[1] - 0.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "domain") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        auto [t0, t1] = curve.domain();

        MINI_CHECK(t0 < t1);
    }

    MINI_TEST("NurbsCurve", "is_closed") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points_open = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };
        NurbsCurve curve_open = NurbsCurve::create(false, 2, points_open);

        std::vector<Point> points_closed = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0)
        };
        NurbsCurve curve_closed = NurbsCurve::create(false, 3, points_closed);

        MINI_CHECK(curve_open.is_closed() == false);
        MINI_CHECK(curve_closed.is_closed() == true);
    }

    MINI_TEST("NurbsCurve", "length") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 1, points);
        double length = curve.length();

        MINI_CHECK(std::abs(length - 1.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "reverse") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        Point pt_start_before = curve.point_at_start();
        Point pt_end_before = curve.point_at_end();
        curve.reverse();
        Point pt_start_after = curve.point_at_start();
        Point pt_end_after = curve.point_at_end();

        MINI_CHECK(std::abs(pt_start_before[0] - pt_end_after[0]) < 0.01);
        MINI_CHECK(std::abs(pt_end_before[0] - pt_start_after[0]) < 0.01);
    }

    MINI_TEST("NurbsCurve", "make_rational") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        MINI_CHECK(curve.is_rational() == false);
        curve.make_rational();
        MINI_CHECK(curve.is_rational() == true);
    }

    MINI_TEST("NurbsCurve", "tangent_at") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        Vector tangent = curve.tangent_at(t_mid);

        MINI_CHECK(std::isfinite(tangent[0]));
        MINI_CHECK(std::isfinite(tangent[1]));
    }

    MINI_TEST("NurbsCurve", "knot_count") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        int knot_count = curve.knot_count();

        MINI_CHECK(knot_count == curve.order() + curve.cv_count() - 2);
    }

    MINI_TEST("NurbsCurve", "cv_size") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        MINI_CHECK(curve.cv_size() == 3);
        curve.make_rational();
        MINI_CHECK(curve.cv_size() == 4);
    }

    MINI_TEST("NurbsCurve", "weight") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        curve.make_rational();
        double w = curve.weight(0);
        curve.set_weight(1, 2.0);
        double w1 = curve.weight(1);

        MINI_CHECK(std::abs(w - 1.0) < 0.01);
        MINI_CHECK(std::abs(w1 - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "is_linear") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points_linear = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0)
        };
        NurbsCurve curve_linear = NurbsCurve::create(false, 1, points_linear);

        std::vector<Point> points_curved = {
            Point(0.0, 0.0, 0.0),
            Point(0.5, 1.0, 0.0),
            Point(1.0, 0.0, 0.0)
        };
        NurbsCurve curve_curved = NurbsCurve::create(false, 2, points_curved);

        MINI_CHECK(curve_linear.is_linear() == true);
        MINI_CHECK(curve_curved.is_linear() == false);
    }

    MINI_TEST("NurbsCurve", "json_roundtrip") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        std::string filename = "serialization/test_nurbscurve.json";
        curve.json_dump(filename);
        NurbsCurve loaded = NurbsCurve::json_load(filename);

        MINI_CHECK(loaded.is_valid() == true);
        MINI_CHECK(loaded.cv_count() == 3);
        MINI_CHECK(loaded.degree() == 2);
        MINI_CHECK(loaded.order() == 3);
    }

    MINI_TEST("NurbsCurve", "protobuf_roundtrip") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        std::string filename = "serialization/test_nurbscurve.bin";
        curve.protobuf_dump(filename);
        NurbsCurve loaded = NurbsCurve::protobuf_load(filename);

        MINI_CHECK(loaded.is_valid() == true);
        MINI_CHECK(loaded.cv_count() == 3);
        MINI_CHECK(loaded.degree() == 2);
        MINI_CHECK(loaded.order() == 3);
    }

    MINI_TEST("NurbsCurve", "degree") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        MINI_CHECK(curve.degree() == 2);
        MINI_CHECK(curve.order() == 3);
    }



    MINI_TEST("NurbsCurve", "is_rational") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        MINI_CHECK(curve.is_rational() == false);
        curve.make_rational();
        MINI_CHECK(curve.is_rational() == true);
    }

    MINI_TEST("NurbsCurve", "set_weight") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        curve.make_rational();

        MINI_CHECK(std::abs(curve.weight(1) - 1.0) < 0.01);
        curve.set_weight(1, 2.0);
        MINI_CHECK(std::abs(curve.weight(1) - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "knot") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        double knot0 = curve.knot(0);
        double knot1 = curve.knot(1);

        MINI_CHECK(std::abs(knot0 - 0.0) < 0.01);
        MINI_CHECK(knot1 >= knot0);
    }

    MINI_TEST("NurbsCurve", "set_knot") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        bool result = curve.set_knot(0, 0.5);

        MINI_CHECK(result == true);
        MINI_CHECK(std::abs(curve.knot(0) - 0.5) < 0.01);
    }

    MINI_TEST("NurbsCurve", "set_domain") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        bool result = curve.set_domain(0.0, 10.0);
        auto [t0, t1] = curve.domain();

        MINI_CHECK(result == true);
        MINI_CHECK(std::abs(t0 - 0.0) < 0.01);
        MINI_CHECK(std::abs(t1 - 10.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "span_count") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        int spans = curve.span_count();

        MINI_CHECK(spans == 1);
    }

    MINI_TEST("NurbsCurve", "get_span_vector") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        std::vector<double> spans = curve.get_span_vector();

        MINI_CHECK(spans.size() >= 2);
    }

    MINI_TEST("NurbsCurve", "evaluate") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        std::vector<Vector> result = curve.evaluate(t_mid, 1);

        MINI_CHECK(result.size() >= 1);
    }

    MINI_TEST("NurbsCurve", "is_periodic") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        MINI_CHECK(curve.is_periodic() == false);
    }

    MINI_TEST("NurbsCurve", "make_non_rational") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        curve.make_rational();

        MINI_CHECK(curve.is_rational() == true);
        curve.make_non_rational();
        MINI_CHECK(curve.is_rational() == false);
    }

    MINI_TEST("NurbsCurve", "divide_by_count") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);
        std::vector<Point> divided_points;
        std::vector<double> params;
        bool result = curve.divide_by_count(5, divided_points, &params, true);

        MINI_CHECK(result == true);
        MINI_CHECK(divided_points.size() == 5);
    }

    MINI_TEST("NurbsCurve", "intersect_plane") {
        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_domain(0.0, 1.0);
        Plane plane = Plane::xy_plane();
        std::vector<double> intersections = curve.intersect_plane(plane);

        MINI_CHECK(intersections.size() >= 0);
    }

    // MINI_TEST("NurbsCurve", "create_circle") {
    //     NurbsCurve c = NurbsCurve::create_circle(0.0, 0.0, 0.0, 1.0);
    //     auto [t0, t1] = c.domain();

    //     Point p0 = c.point_at(t0);
    //     Point p_quarter = c.point_at(t0 + (t1 - t0) * 0.25);
    //     Point p_half = c.point_at(t0 + (t1 - t0) * 0.5);

    //     MINI_CHECK(TOLERANCE.is_close(p0[0], 1.0));
    //     MINI_CHECK(TOLERANCE.is_close(p0[1], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_quarter[0], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_quarter[1], 1.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_half[0], -1.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_half[1], 0.0));
    // }

    // MINI_TEST("NurbsCurve", "create_ellipse") {
    //     NurbsCurve c = NurbsCurve::create_ellipse(0.0, 0.0, 0.0, 2.0, 1.0);
    //     auto [t0, t1] = c.domain();

    //     Point p0 = c.point_at(t0);
    //     Point p_quarter = c.point_at(t0 + (t1 - t0) * 0.25);

    //     MINI_CHECK(TOLERANCE.is_close(p0[0], 2.0));
    //     MINI_CHECK(TOLERANCE.is_close(p0[1], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_quarter[0], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_quarter[1], 1.0));
    // }

    // MINI_TEST("NurbsCurve", "create_arc") {
    //     NurbsCurve c = NurbsCurve::create_arc(Point(0, 0, 0), Point(1, 1, 0), Point(2, 0, 0));
    //     auto [t0, t1] = c.domain();

    //     Point p0 = c.point_at(t0);
    //     Point p1 = c.point_at(t1);

    //     MINI_CHECK(TOLERANCE.is_close(p0[0], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p0[1], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p1[0], 2.0));
    //     MINI_CHECK(TOLERANCE.is_close(p1[1], 0.0));
    // }

    // MINI_TEST("NurbsCurve", "create_parabola") {
    //     NurbsCurve c = NurbsCurve::create_parabola(Point(-1, 1, 0), Point(0, 0, 0), Point(1, 1, 0));
    //     auto [t0, t1] = c.domain();

    //     Point p0 = c.point_at(t0);
    //     Point p_mid = c.point_at(0.5);
    //     Point p1 = c.point_at(t1);

    //     MINI_CHECK(TOLERANCE.is_close(p0[0], -1.0));
    //     MINI_CHECK(TOLERANCE.is_close(p0[1], 1.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_mid[0], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p_mid[1], 0.0));
    //     MINI_CHECK(TOLERANCE.is_close(p1[0], 1.0));
    //     MINI_CHECK(TOLERANCE.is_close(p1[1], 1.0));
    // }

    // MINI_TEST("NurbsCurve", "create_hyperbola") {
    //     NurbsCurve c = NurbsCurve::create_hyperbola(Point(0, 0, 0), 1.0, 1.0, 1.0);
    //     auto [t0, t1] = c.domain();

    //     Point p_mid = c.point_at(0.5);

    //     MINI_CHECK(p_mid[0] > 0.0);
    //     MINI_CHECK(TOLERANCE.is_close(p_mid[1], 0.0));
    // }

    // MINI_TEST("NurbsCurve", "create_spiral") {
    //     NurbsCurve c = NurbsCurve::create_spiral(1.0, 2.0, 1.0, 5.0);
    //     auto [t0, t1] = c.domain();

    //     Point p0 = c.point_at(t0);
    //     Point p1 = c.point_at(t1);

    //     MINI_CHECK(TOLERANCE.is_close(p0[2], 0.0));
    //     MINI_CHECK(p1[2] > 4.0);
    // }

} // namespace session_cpp
