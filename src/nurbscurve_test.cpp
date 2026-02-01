#include "mini_test.h"
#include "nurbscurve.h"
#include "point.h"
#include "plane.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"
#include "intersection.h"
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
        MINI_CHECK(cstr == "NurbsCurve(name=my_nurbscurve, degree=2, cvs=4)");
        MINI_CHECK(crepr.find("name=my_nurbscurve") != std::string::npos);
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

        // Insert knot into curve
        // Useful for splittle curves at a parameter
        // Increase local control without changing shape
        NurbsCurve copy_curve = curve; 
        Point before_pt = copy_curve.point_at(1.5);
        copy_curve.insert_knot(1.5, 1);
        MINI_CHECK(TOLERANCE.is_point_close(before_pt, copy_curve.point_at(1.5)));

        // Check if the curve is clamped at start, end, or both
        bool is_clamped_start = curve.is_clamped(0);
        bool is_clamped_end = curve.is_clamped(1);
        bool is_clamped_both = curve.is_clamped(2);
        MINI_CHECK(is_clamped_start == true && is_clamped_end == true && is_clamped_both == true);

        // Useful for controling curve by cv on lying on it
        double greville0 = curve.greville_abcissa(0);
        MINI_CHECK(TOLERANCE.is_close(greville0, 0.0));

        std::vector<double> greville;
        curve.get_greville_abcissae(greville);
        MINI_CHECK(greville.size() == 4);
        MINI_CHECK(TOLERANCE.is_close(greville[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(greville[1], 0.5));
        MINI_CHECK(TOLERANCE.is_close(greville[2], 1.5));
        MINI_CHECK(TOLERANCE.is_close(greville[3], 2.0));

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

        double t_out = 0.0;
        bool found = curve.get_next_discontinuity(2, curve.domain_start(), curve.domain_end(), t_out);
        MINI_CHECK(found == true && t_out == 0.5);

        // Is rational is related to control points having weights
        // is_rational = false means control points [x, y, z]
        // is_rational = false means control points [xw, yw, zw]
        // Rational curves are used to represent:
        // circles, ellipses, parabolas, hyperbolas exactly
        bool is_rational = curve.is_rational();
        bool closed = curve.is_closed();
        bool periodic = curve.is_periodic();
        bool linear = curve.is_linear();
        bool planar = curve.is_planar();
        bool arc = curve.is_arc();
        Plane plane = Plane::xy_plane();
        bool on_plane = curve.is_in_plane(plane);
        bool is_open = curve.is_natural();
        bool is_polyline = curve.is_polyline();
        bool is_singular = curve.is_singular();
        bool is_duplicate = curve.is_duplicate(curve, false);
        bool is_continuous = curve.is_continuous(1, curve.domain_middle());

        MINI_CHECK(is_rational == true);
        MINI_CHECK(closed == false);
        MINI_CHECK(periodic == false);
        MINI_CHECK(linear == false);
        MINI_CHECK(planar == false);
        MINI_CHECK(arc == false);
        MINI_CHECK(on_plane == false);
        MINI_CHECK(is_open == false);
        MINI_CHECK(is_polyline == false);
        MINI_CHECK(is_singular == false);
        MINI_CHECK(is_duplicate == true);
        MINI_CHECK(is_continuous == true);
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
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[13], Point(2.0, 0.5, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(adaptive_pts[26], Point(4.0, 0.0, 0.0)));

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
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[0], Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[6], Point(1.928691287815458, 0.510169864866836, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(len_pts[12], Point(3.934494402948975, 0.128829830906625, 0.0)));
    }

    MINI_TEST("NurbsCurve", "Evaluation"){
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

        // Length
        MINI_CHECK(TOLERANCE.is_close(curve.length(), 11.3010276326));

        // Get point at parameter t
        Point point_at = curve.point_at(0.5);
        MINI_CHECK(TOLERANCE.is_close(point_at[0], 1.445733625) && TOLERANCE.is_close(point_at[1], 1.80199875) && TOLERANCE.is_close(point_at[2], -0.134851625));

        // Get point and derivatives at parameter t
        std::vector<Vector> derivatives = curve.evaluate(0.5, 2);
        MINI_CHECK(derivatives.size() == 3);
        MINI_CHECK(TOLERANCE.is_close(derivatives[0][0], 1.445733625) && TOLERANCE.is_close(derivatives[0][1], 1.80199875) && TOLERANCE.is_close(derivatives[0][2], -0.134851625));
        MINI_CHECK(TOLERANCE.is_close(derivatives[1][0], 0.0432025) && TOLERANCE.is_close(derivatives[1][1], 1.154047) && TOLERANCE.is_close(derivatives[1][2], -0.1568445));
        MINI_CHECK(TOLERANCE.is_close(derivatives[2][0], 4.267853) && TOLERANCE.is_close(derivatives[2][1], -0.677778) && TOLERANCE.is_close(derivatives[2][2], -1.078813));

        // Tangent vector at parameter t
        Vector tangent = curve.tangent_at(0.5);
        MINI_CHECK(TOLERANCE.is_close(tangent[0], 0.037069134389828) && TOLERANCE.is_close(tangent[1], 0.990209443486538) && TOLERANCE.is_close(tangent[2], -0.134577625575985));

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

        // Perpendicular frame at (RMF with Frenet initialization, matches Rhino)
        curve.perpendicular_frame_at(0.5, true, o, t, n, b);
        MINI_CHECK(TOLERANCE.is_point_close(o, Point(3.156927375000000, 1.335111500000000, 0.130488875000000)));
        MINI_CHECK(TOLERANCE.is_vector_close(t, Vector(0.632703652329189, -0.703685357647999, 0.323284713157168)));
        MINI_CHECK(TOLERANCE.is_vector_close(n, Vector(0.327344206830723, -0.135306795251661, -0.935167279909370)));
        MINI_CHECK(TOLERANCE.is_vector_close(b, Vector(0.701806140314880, 0.697509131546342, 0.144738221716994)));
        MINI_CHECK(curve.perpendicular_frame_at(-0.1, true, o, t, n, b) == false);
        MINI_CHECK(curve.perpendicular_frame_at(1.1, true, o, t, n, b) == false);
        MINI_CHECK(curve.perpendicular_frame_at(curve.domain_start(), false, o, t, n, b) == true);
        MINI_CHECK(curve.perpendicular_frame_at(curve.domain_end(), false, o, t, n, b) == true);
        MINI_CHECK(curve.perpendicular_frame_at(curve.domain_start() - 0.1, false, o, t, n, b) == false);

        // Get multiple rotation minimization frames along the curve (matches Rhino)
        std::vector<double> params = {0.0, 0.25, 0.5, 0.75, 1.0};
        auto frames = curve.get_perpendicular_frames(params, true);
        MINI_CHECK(frames.size() == 5);
        // Frame 0 (start)
        auto [o0, t0, n0, b0] = frames[0];
        MINI_CHECK(TOLERANCE.is_point_close(o0, Point(1.957614, 1.140253, -0.191281)));
        MINI_CHECK(TOLERANCE.is_vector_close(t0, Vector(0.532767753269467, 0.809398954921174, -0.247046256496055)));
        MINI_CHECK(TOLERANCE.is_vector_close(n0, Vector(-0.261213903019039, -0.120386647366337, -0.957744408496053)));
        MINI_CHECK(TOLERANCE.is_vector_close(b0, Vector(-0.804938393882267, 0.574787253606414, 0.147288136473484)));
        // Frame 2 (middle)
        auto [o2, t2, n2, b2] = frames[2];
        MINI_CHECK(TOLERANCE.is_point_close(o2, Point(3.156927375000000, 1.335111500000000, 0.130488875000000)));
        MINI_CHECK(TOLERANCE.is_vector_close(t2, Vector(0.632703652329189, -0.703685357647999, 0.323284713157168)));
        MINI_CHECK(TOLERANCE.is_vector_close(n2, Vector(0.327344206830723, -0.135306795251661, -0.935167279909370)));
        MINI_CHECK(TOLERANCE.is_vector_close(b2, Vector(0.701806140314880, 0.697509131546342, 0.144738221716994)));
        // Frame 4 (end)
        auto [o4, t4, n4, b4] = frames[4];
        MINI_CHECK(TOLERANCE.is_point_close(o4, Point(2.150320000000000, 1.868606000000000, 0.000000000000000)));
        MINI_CHECK(TOLERANCE.is_vector_close(t4, Vector(0.183261707605497, 0.080808692422033, 0.979737261593412)));
        MINI_CHECK(TOLERANCE.is_vector_close(n4, Vector(0.896455027206172, 0.395289116914872, -0.200287039634224)));
        MINI_CHECK(TOLERANCE.is_vector_close(b4, Vector(-0.403464410725777, 0.914995338391241, 0.000000000000000)));

        // Points
        Point p0 = curve.point_at_start();
        Point p1 = curve.point_at_middle();
        Point p2 = curve.point_at_end();
        MINI_CHECK(TOLERANCE.is_close(p0[0], 1.957614) && TOLERANCE.is_close(p0[1], 1.140253) && TOLERANCE.is_close(p0[2], -0.191281));
        MINI_CHECK(TOLERANCE.is_close(p1[0], 3.156927375) && TOLERANCE.is_close(p1[1], 1.3351115) && TOLERANCE.is_close(p1[2], 0.130488875));
        MINI_CHECK(TOLERANCE.is_close(p2[0], 2.15032) && TOLERANCE.is_close(p2[1], 1.868606) && TOLERANCE.is_close(p2[2], 0.0));

        curve.set_start_point(Point(1.957614, 1.140253, 2.0));
        curve.set_end_point(Point(2.15032, 1.868606, 2.0));
        MINI_CHECK(TOLERANCE.is_close(curve.point_at_start()[2], 2.0));
        MINI_CHECK(TOLERANCE.is_close(curve.point_at_end()[2], 2.0));
    }

    MINI_TEST("NurbsCurve", "Modifications"){
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


        // Reverse the curve
        NurbsCurve curve_reversed = curve;
        curve_reversed.reverse();
        MINI_CHECK(TOLERANCE.is_point_close(curve_reversed.point_at_start(), curve.point_at_end()));

        // Swap coordinates axes
        curve.swap_coordinates(0, 1);
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(0), Point(0.0, 0.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(1), Point(2.0, 1.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(2), Point(0.0, 2.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(3), Point(2.0, 3.0, 0.0)));
        MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(4), Point(0.0, 4.0, 0.0)));

        // Trim curve at domain parameter
        NurbsCurve ct = curve;
        double a = ct.domain_start() + (ct.domain_end() - ct.domain_start()) / 3.0;
        double b = ct.domain_start() + 2.0 * (ct.domain_end() - ct.domain_start()) / 3.0;
        ct.trim(a, b);
        MINI_CHECK(ct.length() < curve.length());

        // Split curve at domain middle
        NurbsCurve curve_left;
        NurbsCurve curve_right;
        double split_t = curve.domain_middle();
        curve.split(split_t, curve_left, curve_right);
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(split_t), curve_left.point_at_end()));
        MINI_CHECK(TOLERANCE.is_point_close(curve.point_at(split_t), curve_right.point_at_start()));

        // Extend curve smoothly at both ends
        NurbsCurve curve_extended = curve;
        curve_extended.extend(curve.domain_start()-0.5, curve.domain_end()+0.5);
        MINI_CHECK(curve_extended.length() > curve.length());

        // Enable curve weights - Make rational or non-rational
        NurbsCurve curve_rational = curve;
        double original_length = curve.length();
        curve_rational.make_rational();
        curve_rational.set_weight(2, 10);
        MINI_CHECK(curve_rational.length() != original_length);

        curve_rational.make_non_rational(true);  // force=true, sets all weights to 1.0
        MINI_CHECK(curve_rational.length() == original_length);

        // Clamp ends - create unclamped curve manually
        std::vector<Point> points_open = points;
        NurbsCurve curve_open(3, false, 3, 5);  // dim=3, non-rational, order=3 (deg 2), 5 CVs

        for (int i = 0; i < 5; ++i)
            curve_open.set_cv(i, points_open[i]);

        for (int i = 0; i < curve_open.knot_count(); ++i)
            curve_open.set_knot(i, i * 1.0);

        // Now clamp, making 2 knots at the ends the same
        curve_open.clamp_end(2);
        std::vector<double> knots = curve_open.get_knots();
        MINI_CHECK(TOLERANCE.is_close(knots[0], knots[1]));
        MINI_CHECK(TOLERANCE.is_close(knots[knots.size() - 2], knots[knots.size() - 1]));

        // Increase degree without change the shape
        NurbsCurve raised = curve;
        raised.increase_degree(3);
        MINI_CHECK(curve.degree() != raised.degree() && TOLERANCE.is_point_close(curve.point_at_middle(), raised.point_at_middle()) );

        // Change closed curve seam
        std::vector<Point> closed_pts = {
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(-1.0, 0.0, 0.0),
            Point(0.0, -1.0, 0.0)
        };
        NurbsCurve c = NurbsCurve::create(true, 2, closed_pts);
        Point expected_start = c.point_at(c.domain_middle());
        c.change_closed_curve_seam(c.domain_middle());
        MINI_CHECK(TOLERANCE.is_point_close(c.point_at_start(), expected_start));


    }

    MINI_TEST("NurbsCurve", "transformations"){
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include "xform.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };


        // transform() - Apply stored xform (in-place)                                                                                                                                                         
        NurbsCurve curve1 = NurbsCurve::create(false, 2, points);                                                         
        curve1.xform = Xform::translation(0.0, 0.0, 1.0);  
        curve1.transform();  // applies stored xform, modifies curve1
        MINI_CHECK(curve1.xform.is_identity() == false);
        MINI_CHECK(curve1.cv(0)[2] == 1.0);

        // transform(const Xform&) - Apply custom xform (in-place)
        NurbsCurve curve2 = NurbsCurve::create(false, 2, points);
        Xform x = Xform::translation(0.0, 0.0, 1.0);
        curve2.transform(x);  // modifies curve2 directly
        MINI_CHECK(curve2.xform.is_identity() == true);
        MINI_CHECK(curve2.cv(0)[2] == 1.0);

        // transformed() - Get copy with stored xform applied
        NurbsCurve curve3 = NurbsCurve::create(false, 2, points);
        curve3.xform = Xform::translation(0.0, 0.0, 10.0);
        NurbsCurve curve3_transformed = curve3.transformed();
        MINI_CHECK(curve3_transformed.xform.is_identity() == false);
        MINI_CHECK(curve3_transformed.cv(0)[2] == 10.0);
        
        // transformed(const Xform&) - Get copy with custom xform
        NurbsCurve curve4 = NurbsCurve::create(false, 2, points);
        x = Xform::translation(0.0, 0.0, 10.0);
        NurbsCurve curve4_transformed = curve4.transformed(x); 
        MINI_CHECK(curve4_transformed.xform.is_identity() == true);
        MINI_CHECK(curve4_transformed.cv(0)[2] == 10.0);

    }

    MINI_TEST("NurbsCurve", "json_roundtrip") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include "xform.h"
        // uncomment #include <filesystem>

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };
        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        //   jsondump()      │ ordered_json │ to JSON object (internal use)                                                                          
        //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
        //   json_dumps()    │ std::string  │ to JSON string                  
        //   json_loads(s)   │ std::string  │ from JSON string                
        //   json_dump(path) │ file         │ write to file                   
        //   json_load(path) │ file         │ read from file  

        // JSON object
        nlohmann::ordered_json json = curve.jsondump();
        NurbsCurve loaded_json = NurbsCurve::jsonload(json);

        // String
        std::string json_string = curve.json_dumps();
        NurbsCurve loaded_json_string = NurbsCurve::json_loads(json_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbscurve.json").string();
        curve.json_dump(filename);
        NurbsCurve loaded_from_file = NurbsCurve::json_load(filename);

        MINI_CHECK(loaded_json == curve);
        MINI_CHECK(loaded_json_string == curve);
        MINI_CHECK(loaded_from_file == curve);
    }

    MINI_TEST("NurbsCurve", "protobuf_roundtrip") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include "xform.h"
        // uncomment #include <filesystem>


        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };
        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        // String
        std::string proto_string = curve.pb_dumps();
        NurbsCurve loaded_proto_string = NurbsCurve::pb_loads(proto_string);

        // File
        std::string filename = (std::filesystem::path(__FILE__).parent_path().parent_path() / "serialization" / "test_nurbscurve.bin").string();
        curve.pb_dump(filename);
        NurbsCurve loaded = NurbsCurve::pb_load(filename);

        MINI_CHECK(loaded_proto_string == curve);
        MINI_CHECK(loaded == curve);
    }

} // namespace session_cpp
