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
class Mesh;
class PointCloud;
class AABBTree;

class Closest {
public:

  static std::pair<double, double> curve_point(
    const NurbsCurve& curve,
    const Point& test_point,
    double t0 = 0.0,
    double t1 = 0.0
  );

  static std::tuple<Point, double, double> line_point(
    const Line& line,
    const Point& test_point
  );

  static std::tuple<Point, double, double> polyline_point(
    const Polyline& polyline,
    const Point& test_point
  );

  static std::tuple<double, double, double> surface_point(
    const NurbsSurface& surface,
    const Point& test_point,
    double u0 = 0.0,
    double u1 = 0.0,
    double v0 = 0.0,
    double v1 = 0.0
  );

  static std::tuple<Point, size_t, double> mesh_point(
    const Mesh& mesh,
    const Point& test_point
  );

  static std::tuple<Point, size_t, double> mesh_point_aabb(
    const Mesh& mesh,
    const Point& test_point
  );

  static std::tuple<Point, size_t, double> pointcloud_point(
    const PointCloud& cloud,
    const Point& test_point
  );

};

} // namespace session_cpp
