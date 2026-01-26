#pragma once
#include "point.h"
#include "vector.h"
#include "line.h"
#include "polyline.h"
#include "tolerance.h"
#include <tuple>
#include <vector>

namespace session_cpp {

class NurbsCurve;
class NurbsSurface;

class Closest {
public:

  /**
   * @brief Find closest point on NURBS curve to test point
   * @param curve NURBS curve
   * @param test_point Point to find closest point to
   * @param t0 Start of search interval (0=use curve start)
   * @param t1 End of search interval (0=use curve end)
   * @return Pair of (parameter, distance)
   */
  static std::pair<double, double> curve_point(
    const NurbsCurve& curve,
    const Point& test_point,
    double t0 = 0.0,
    double t1 = 0.0
  );

  /**
   * @brief Find closest point on line to test point
   * @param line Line segment
   * @param test_point Point to find closest point to
   * @return Tuple of (closest_point, parameter, distance)
   */
  static std::tuple<Point, double, double> line_point(
    const Line& line,
    const Point& test_point
  );

  /**
   * @brief Find closest point on polyline to test point
   * @param polyline Polyline
   * @param test_point Point to find closest point to
   * @return Tuple of (closest_point, parameter, distance)
   */
  static std::tuple<Point, double, double> polyline_point(
    const Polyline& polyline,
    const Point& test_point
  );

  /**
   * @brief Find closest point on NURBS surface to test point
   * @param surface NURBS surface
   * @param test_point Point to find closest point to
   * @param u0 Start of U search interval
   * @param u1 End of U search interval
   * @param v0 Start of V search interval
   * @param v1 End of V search interval
   * @return Tuple of (u_param, v_param, distance)
   */
  static std::tuple<double, double, double> surface_point(
    const NurbsSurface& surface,
    const Point& test_point,
    double u0 = 0.0,
    double u1 = 0.0,
    double v0 = 0.0,
    double v1 = 0.0
  );

};

} // namespace session_cpp
